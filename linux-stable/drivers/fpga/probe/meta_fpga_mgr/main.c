#include <linux/fpga/fpga-mgr.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include <common_fpga/fpgafeat.h>

static int meta_fpga_i2c_reg_read(void *ctx, unsigned int reg, unsigned int *val)
{
	struct i2c_client *i2c = ctx;
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x1, reg & 0xff);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x1, (reg >> 8) & 0xff);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x4, 0x0);
	if (ret)
		return ret;

	ret = i2c_smbus_read_byte_data(i2c, 0x6);
	if (ret < 0)
		return ret;

	*val = ret & 0xff;

	ret = i2c_smbus_read_byte_data(i2c, 0x6);
	if (ret < 0)
		return ret;

	*val |= (ret & 0xff) << 8;

	return 0;
}

static int meta_fpga_i2c_reg_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct i2c_client *i2c = ctx;
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x1, reg & 0xff);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x1, (reg >> 8) & 0xff);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x2, val & 0xff);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(i2c, 0x2, (val >> 8) & 0xff);
	if (ret)
		return ret;

	return i2c_smbus_write_byte_data(i2c, 0x3, 0x0);
}

static bool meta_fpga_accessible_reg(struct device *dev, unsigned int reg)
{
	struct stcmtk_common_fpga *common_fpga = dev_get_drvdata(dev);
	struct fpga_manager *fpga_mgr = common_fpga->fpga_defined_fields;
	enum fpga_mgr_states state;

	if (fpga_mgr->mops->state)
		state = fpga_mgr->mops->state(fpga_mgr);
	else
		state = fpga_mgr->state;

	return (state == FPGA_MGR_STATE_OPERATING);
}

static const struct regmap_config meta_fpga_i2c_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 16,
	.reg_read	= meta_fpga_i2c_reg_read,
	.reg_write	= meta_fpga_i2c_reg_write,
	.writeable_reg	= meta_fpga_accessible_reg,
	.readable_reg	= meta_fpga_accessible_reg,
	/*
	 * FIXME
	 * This number was copy-pasted from grif-fpga-mgr, but the meaning
	 * is not clear. .max_register is used in {etn,grif}-io modules.
	 */
	.max_register	= 4095,
};

static dev_t meta_fpga_dev_t;

static int meta_fpga_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	struct stcmtk_common_fpga *common_fpga;
	struct device_node *fpga_mgr_node;
	struct fpga_manager *fpga_mgr;
	struct device *meta_dev;
	struct device *dev;
	int ret;

	dev = &i2c->dev;

	common_fpga = devm_kzalloc(dev, sizeof(struct stcmtk_common_fpga),
				   GFP_KERNEL);
	if (!common_fpga) {
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, common_fpga);

	ret = stcmtk_class_create(THIS_MODULE, "fpga", NULL);
	if (ret)
		return ret;

	ret = alloc_chrdev_region(&meta_fpga_dev_t, 0, 256, "meta-fpga-mgr");
	if (ret) {
		stcmtk_class_destroy();
		return ret;
	}

	meta_dev = stcmtk_device_create(dev, meta_fpga_dev_t, common_fpga,
					"fpga%d", 0);
	if (IS_ERR(meta_dev)) {
		unregister_chrdev_region(meta_fpga_dev_t, 256);
		stcmtk_class_destroy();
		return PTR_ERR(meta_dev);
	}

	fpga_mgr_node = of_parse_phandle(dev->of_node, "fpga-mgr", 0);
	if (!fpga_mgr_node) {
		stcmtk_device_destroy(meta_fpga_dev_t);
		unregister_chrdev_region(meta_fpga_dev_t, 256);
		stcmtk_class_destroy();
		return -ENOENT;
	}

	fpga_mgr = of_fpga_mgr_get(fpga_mgr_node);
	if (IS_ERR(fpga_mgr)) {
		of_node_put(fpga_mgr_node);
		stcmtk_device_destroy(meta_fpga_dev_t);
		unregister_chrdev_region(meta_fpga_dev_t, 256);
		stcmtk_class_destroy();
		return PTR_ERR(fpga_mgr);
	}

	of_node_put(fpga_mgr_node);

	common_fpga->dev = dev;
	common_fpga->abstract_fpga = meta_dev;
	common_fpga->fpga_defined_fields = fpga_mgr;
	common_fpga->regmap = devm_regmap_init(dev, NULL, i2c,
					       &meta_fpga_i2c_regmap_config);
	common_fpga->features = stcmtk_fpga_parse_features(common_fpga);

	finish_completion();

	return 0;
}

static int meta_fpga_remove(struct i2c_client *i2c)
{
	struct stcmtk_common_fpga *common_fpga = i2c_get_clientdata(i2c);
	struct fpga_manager *fpga_mgr = common_fpga->fpga_defined_fields;

	fpga_mgr_put(fpga_mgr);

	if (common_fpga->features)
		stcmtk_fpgafeat_deinit(common_fpga->dev, &common_fpga->features);

	stcmtk_device_destroy(meta_fpga_dev_t);
	unregister_chrdev_region(meta_fpga_dev_t, 256);
	stcmtk_class_destroy();

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id meta_fpga_of_ids[] = {
	{ .compatible = "stcmtk,m720-10g-meta-fpga-mgr" },
	{},
};
MODULE_DEVICE_TABLE(of, meta_fpga_of_ids);
#endif /* IS_ENABLED(CONFIG_OF) */

static struct i2c_driver meta_fpga_driver = {
	.driver = {
		.name = "meta-fpga-mgr",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(meta_fpga_of_ids),
	},
	.probe = meta_fpga_probe,
	.remove = meta_fpga_remove,
};
module_i2c_driver(meta_fpga_driver);

MODULE_DESCRIPTION("Meta FPGA Manager");
MODULE_LICENSE("GPL v2");
