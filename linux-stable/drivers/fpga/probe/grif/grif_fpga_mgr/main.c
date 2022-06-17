#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/gpio/consumer.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#include <asm/io.h>
#include <asm/neon.h>

#include "mem_test.h"
#include "grif_fpga.h"
#include <common_fpga/fpgafeat.h>
#include "bench.h"

#define DRV_VERSION "1.0.6"

#define GRIF_FPGA_MAX_MINORS 256

/* FPGA register size equals 16 bit then register byte addr = numeric addr * 2
 *
 */
#define EIM_TO_BYTE_ADDR(numeric_addr) (numeric_addr * 2)

static DEFINE_IDA(fpga_ida);
static int fpga_major;

static const char fpga_fw_name_default[] = "ecp5_sspi_fw.img";

//FIXME: Do we have to set p to NULL after returning if error?
#define GET_REGULATOR(p, id, dev) \
{ \
	p = devm_regulator_get(dev, id); \
	if (IS_ERR(p)) { \
		dev_err(dev, "Can't get %s regulator!\n", id); \
		return PTR_ERR(p); \
	} \
}

static int grif_fpga_power_init(struct stcmtk_common_fpga *f)
{
	struct device *d = f->dev;
	struct grif_fpga *grif = (struct grif_fpga*)f->fpga_defined_fields;

	GET_REGULATOR(grif->vcc_supply, "vcc", d);
	GET_REGULATOR(grif->vcc_tra_supply, "vcc-tra", d);
	GET_REGULATOR(grif->vcc_aux_supply, "vcc-aux", d);

	return 0;
}

static int grif_fpga_enable_power(struct stcmtk_common_fpga *g)
{
	struct grif_fpga *grif = (struct grif_fpga*)g->fpga_defined_fields;

	struct regulator *enable_seq[] = {
		grif->vcc_supply,
		grif->vcc_tra_supply,
		grif->vcc_aux_supply
	};

	int seq_len = ARRAY_SIZE(enable_seq);
	int i, err;

	for (i = 0; i < seq_len; ++i) {
		err = regulator_enable(enable_seq[i]);
		if (err) {
			goto err_power_on_fault;
		}
	}

	return 0;

err_power_on_fault:
	/* Rollback */
	while(i) {
		--i;
		regulator_disable(enable_seq[i]);
	}

	return err;
}

static void grif_fpga_disable_power(struct stcmtk_common_fpga *g)
{
	struct grif_fpga *grif = (struct grif_fpga*)g->fpga_defined_fields;

	struct regulator *disable_seq[] = {
		grif->vcc_aux_supply,
		grif->vcc_tra_supply,
		grif->vcc_supply
	};

	int seq_len = ARRAY_SIZE(disable_seq);
	int i;

	for (i = 0; i < seq_len; ++i) {
		regulator_disable(disable_seq[i]);
	}
}


/**
 *	grif_get_fpga_manager_inst	-	loads firmware to Grif FPGA
 *	@dev: grif_fpga device
 *
 *	Returns pointer to FPGA's fpga_manager instance from devicve tree.
 *	Or ERR_PTR if an error occurs. Use IS_ERR() to check it.
 *
 *	Don't forget about fpga_manager releasing with fpga_mgr_put() function.
 */
static struct fpga_manager *grif_get_fpga_manager_inst(struct device *dev)
{
	struct fpga_manager *mgr = NULL;
	struct device_node *ecp5_node = NULL;

	ecp5_node = of_parse_phandle(dev->of_node, "ecp5-fpga-mgr", 0);
	if (!ecp5_node) {
		dev_info(dev, "ECP5 FPGA manager DT node not found!\n");
		return ERR_PTR(-ENOENT);
	}

	mgr = of_fpga_mgr_get(ecp5_node);
	if (IS_ERR(mgr)) {
		dev_err(dev, "ECP5 FPGA manager device is not ready.\n");
		of_node_put(ecp5_node);
		return ERR_PTR(-ENODEV);
	}
	of_node_put(ecp5_node);

	return mgr;
}

/**
 *	_grif_fpga_state_get	- check FPGA status
 *	@mgr: Pointer to struct grif_fpga
 *
 *	Checks current FPGA state.
 *
 *	Return FPGA_MGR_STATE_OPERATING if FPGA are ready to work or
 *	FPGA_MGR_STATE_UNKNOWN if not.
 */
static int _grif_fpga_state_get(struct stcmtk_common_fpga *g)
{
	int ret = FPGA_MGR_STATE_OPERATING;
	struct fpga_manager *mgr = grif_get_fpga_manager_inst(g->dev);
	if (IS_ERR(mgr))
		return FPGA_MGR_STATE_UNKNOWN;

	ret = mgr->mops->state(mgr);
	fpga_mgr_put(mgr);

	return ret;
}

/**
 * 	grif_fpga_state_get	- checks FPGA status
 * 	@mgr: Pointer to struct of target FPGA
 *
 * 	Checks current FPGA state. Function for external using. See
 * 	grif_fpga_ops.
 *
 * 	Return 0 is FPGA is ready to operate and -ENODEV otherwise.
 *
 */
int grif_fpga_state_get(struct stcmtk_common_fpga *grif_fpga)
{
	if (_grif_fpga_state_get(grif_fpga) != FPGA_MGR_STATE_OPERATING) {
		return -ENODEV;
	}

	return 0;
}

/**
 *	grif_fpga_fw_load	-	loads firmware to Grif FPGA
 *	@grif_fpga: Pointer to current grif_fpga struct
 *	@fw:   information about used firmware.
 *
 *	Flash firmware to Grif FPGA using fpga_manager kernel framework.
 *	struct grif_firmware *fw has to be filled with pointer to memory
 *	buffer with firmware data and size of this buffer before function
 *	calling.
 *
 *	If *fw equals NULL, function uses default /lib/firmware/ecp5_sspi_fw.img
 *	file.
 */
static int grif_fpga_fw_load(struct stcmtk_common_fpga *grif_fpga,
					struct grif_firmware *fw)
{
	int ret = 0;
	struct fpga_manager *mgr = NULL;
	struct fpga_image_info *info = NULL;
	const char *fpga_fw_name = NULL;
	struct grif_fpga *grif = (struct grif_fpga*)grif_fpga->fpga_defined_fields;

	/* Borrow the fpga_manager */
	mgr = grif_get_fpga_manager_inst(grif_fpga->dev);
	if (IS_ERR(mgr))
	{
		return -ENODEV;
	}

	dev_info(grif_fpga->abstract_fpga, "Got \"%s\"\n", mgr->name);

	/*Flash FPGA here, then release ecp5 SPI device*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,16,0)
	info = fpga_image_info_alloc(grif_fpga->dev);
#else
	info = kzalloc(sizeof(*info), GFP_KERNEL);
#endif
	if (!info) {
		dev_info(grif_fpga->abstract_fpga,
			"Unable to allocate fpga_image_info\n");
		fpga_mgr_put(mgr);
		return -ENOMEM;
	}

	grif->state = FPGA_MGR_STATE_RESET;

	info->flags = 0;

	if(of_property_read_string(grif_fpga->dev->of_node,
					"firmware-name", &fpga_fw_name) != 0) {
		fpga_fw_name = fpga_fw_name_default;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,16,0)
	if (fw == NULL) {
		info->firmware_name = devm_kmalloc(grif_fpga->dev,
						   strlen(fpga_fw_name),
						   GFP_KERNEL);
		strcpy(info->firmware_name, fpga_fw_name);
	}
	else {
		info->buf = (const char*) fw->buf;
		info->count = fw->size;
	}
	ret = fpga_mgr_load(mgr, info);
#else
	if (fw == NULL)
		ret = fpga_mgr_firmware_load(mgr, info, fpga_fw_name);
	else
		ret = fpga_mgr_buf_load(mgr, info,
					(const char*)fw->buf, fw->size);
#endif
	if (ret) {
		dev_info(grif_fpga->abstract_fpga,
				"fpga_mgr_firmware_load returns %d\n", ret);
	}

	fpga_mgr_put(mgr);

	grif->state = _grif_fpga_state_get(grif_fpga);
	if (grif->state != FPGA_MGR_STATE_OPERATING) {
		dev_err(grif_fpga->abstract_fpga, "FPGA not ready!\n");
		ret = -ENODEV;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,16,0)
	fpga_image_info_free(info);
#else
	kfree(info);
#endif

	return ret;
}

static int grif_fpga_eim_reg_read_regmap_cb(void *context, unsigned int reg,
							unsigned int *val)
{
	struct stcmtk_common_fpga *f = context;
	void __iomem *base = ((struct grif_fpga*)f->fpga_defined_fields)->private_data;
	void __iomem *addr = base + (EIM_TO_BYTE_ADDR(reg) * FPGA_SPACE_BYTE_OFFSET);

	*val = ioread16(addr);
	return 0;
}

static int grif_fpga_eim_reg_write_regmap_cb(void *context, unsigned int reg,
							unsigned int val)
{
	struct stcmtk_common_fpga *f = context;
	void __iomem *base = ((struct grif_fpga*)f->fpga_defined_fields)->private_data;
	void __iomem *addr = base + (EIM_TO_BYTE_ADDR(reg) * FPGA_SPACE_BYTE_OFFSET);

	iowrite16(val, addr);
	return 0;
}

static bool grif_fpga_eim_accessible_reg(struct device *dev, unsigned int reg)
{
	struct stcmtk_common_fpga *g = dev_get_drvdata(dev);

	if (((struct grif_fpga*)g->fpga_defined_fields)->state == FPGA_MGR_STATE_OPERATING)
		return true;
	else
		return false;
}

static const struct regmap_config grif_fpga_eim_regmap_config = {
	.reg_bits	= 16,
	.reg_stride	= 1,
	.val_bits	= 16,
	.fast_io	= true,
	.reg_read	= grif_fpga_eim_reg_read_regmap_cb,
	.reg_write	= grif_fpga_eim_reg_write_regmap_cb,
	.readable_reg   = grif_fpga_eim_accessible_reg,
	.writeable_reg  = grif_fpga_eim_accessible_reg,
	.max_register   = 4095,
};

#define SPI_WRITE_CODE (0x00)
#define SPI_READ_CODE  (0x01)

static int grif_fpga_spi_reg_read_regmap_cb(void *context, unsigned int reg,
							unsigned int *val)
{
	int ret = 0;
	unsigned char txbuf[2];
	unsigned char rxbuf[3];

	struct stcmtk_common_fpga *f = context;
	struct grif_fpga *grif = (struct grif_fpga*)f->fpga_defined_fields;

	txbuf[0] = (reg >> 4) & 0xff;
	txbuf[1] = (reg & 0xf) << 4 | SPI_READ_CODE;

	rxbuf[0] = 0x00;
	rxbuf[1] = 0x00;
	rxbuf[2] = 0x00;

	mutex_lock(&grif->regmap_lock);
	gpiod_set_value(grif->cs, 0);

	ret = spi_write_then_read(to_spi_device(f->dev), txbuf, sizeof(txbuf),
					   rxbuf, sizeof(rxbuf));

	gpiod_set_value(grif->cs, 1);
	mutex_unlock(&grif->regmap_lock);

	if (likely(ret == 0)) {
		if (rxbuf[0]) {
			pr_err("Error during read from FPGA: 0x%x\n", rxbuf[0]);
                        ret = -EIO;
		} else {
			*val = rxbuf[1] << 8 | rxbuf[2];
		}
	}

	return ret;
}

static int grif_fpga_spi_reg_write_regmap_cb(void *context, unsigned int reg,
							unsigned int val)
{
	int ret = 0;
	unsigned char buf[5];

	struct stcmtk_common_fpga *f = context;
	struct grif_fpga *grif = (struct grif_fpga*)f->fpga_defined_fields;

	buf[0] = (reg >> 4) & 0xff;
	buf[1] = (reg & 0xf) << 4 | SPI_WRITE_CODE;
	buf[2] = 0x00;
	buf[3] = (val >> 8) & 0xff;
	buf[4] = (val) & 0xff;

	mutex_lock(&grif->regmap_lock);
	gpiod_set_value(grif->cs, 0);

	ret = spi_write(to_spi_device(f->dev), buf, sizeof(buf));

	gpiod_set_value(grif->cs, 1);
	mutex_unlock(&grif->regmap_lock);

	return ret;
}

static bool grif_fpga_spi_accessible_reg(struct device *dev, unsigned int reg)
{
	struct stcmtk_common_fpga *g = dev_get_drvdata(dev);

	if (((struct grif_fpga*)g->fpga_defined_fields)->state == FPGA_MGR_STATE_OPERATING)
		return true;
	else
		return false;
}


void grif_fpga_spi_lock_regmap_cb(void *context) {
  // empty function to avoid redundant locking by regmap.
  // Locking is performed explicitly in the driver (look for regmap_lock)
}

void grif_fpga_spi_unlock_regmap_cb(void *context) {
  // empty function to avoid redundant locking by regmap.
  // Locking is performed explicitly in the driver (look for regmap_lock)
}

static const struct regmap_config grif_fpga_spi_regmap_config = {
	.reg_bits	= 12,
	.reg_stride	= 1,
	.val_bits	= 16,
	.lock           = grif_fpga_spi_lock_regmap_cb,
	.unlock         = grif_fpga_spi_unlock_regmap_cb,
	.reg_read	= grif_fpga_spi_reg_read_regmap_cb,
	.reg_write	= grif_fpga_spi_reg_write_regmap_cb,
	.readable_reg   = grif_fpga_spi_accessible_reg,
	.writeable_reg  = grif_fpga_spi_accessible_reg,
	.max_register   = 4095,
};


static ssize_t grif_mgr_read(struct file *file, char __user *buf, size_t count,
								loff_t *offset)
{
	return -EPERM;
}

static ssize_t grif_mgr_write(struct file *file, const char __user *buf,
						size_t count, loff_t *offset)
{
	struct stcmtk_common_fpga *grif_fpga = file->private_data;
	struct grif_firmware fw;
	char *kern_buf = NULL;
	int ret = 0;

	kern_buf = memdup_user(buf, count);
	if (IS_ERR(kern_buf))
		return PTR_ERR(kern_buf);


	fw.buf = kern_buf;
	fw.size = count;
	ret = grif_fpga_fw_load(grif_fpga, &fw);
	if (ret) {
		kfree(kern_buf);
		return -EFAULT;
	}

	/* There is no the property and regmap exists */
	if (!of_property_read_bool(grif_fpga->dev->of_node,
				 "no-parse-fpga-features") &&
	    !IS_ERR(grif_fpga->regmap))
		grif_fpga->features = stcmtk_fpga_parse_features((struct stcmtk_common_fpga *)grif_fpga);

	kfree(kern_buf);
	return count;
}

static int grif_mgr_open(struct inode *inode, struct file *file)
{
	struct stcmtk_common_fpga *grif_fpga =
		container_of(inode->i_cdev, struct stcmtk_common_fpga, cdev);

	file->private_data = grif_fpga;

	return 0;
}

static int grif_mgr_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct grif_fpga_ops ops = {
	.state_get = grif_fpga_state_get,
};

static const struct file_operations grif_mgr_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= grif_mgr_read,
	.write		= grif_mgr_write,
	.open		= grif_mgr_open,
	.release	= grif_mgr_release,
};

/**
 * Bus-independent FPGA initialization sequence.
 */
static int grif_fpga_init(struct stcmtk_common_fpga *grif_fpga)
{
	int id, err = 0;
	const char *name = NULL;
	struct grif_fpga *fpga_defined = (struct grif_fpga*)grif_fpga->fpga_defined_fields;

	/* Get FPGA name */
	if(of_property_read_string(grif_fpga->dev->of_node,
							"fpga-name", &name)) {
		fpga_defined->name = "unnamed";
	}
	else {
		fpga_defined->name = name;
	}

	/* Check if test mode is needed */
	if (of_property_read_bool(grif_fpga->dev->of_node,
		"no-parse-fpga-features"))
		fpga_defined->test_mode = true;

	/* Try to get all needed regulators. */
	err = grif_fpga_power_init(grif_fpga);
	if (err) {
		dev_err(grif_fpga->dev, "reg %d\n", err);
		if (err == -EPROBE_DEFER) {
			dev_info(grif_fpga->dev, "Power suppliers aren't ready. Try later.\n");
			return err;
		}
		else {
			dev_err(grif_fpga->dev, "Power init failed: %d\n", err);
			return err;
		}
	}

	/* Get ID for new FPGA device */
	id = ida_simple_get(&fpga_ida, 0, 0, GFP_KERNEL);
	if(id < 0) {
		err = id;
		dev_err(grif_fpga->dev, "Unable to get ID!\n");
		return err;
	}

	grif_fpga->cdev_minor = id;

	/* Create the device with ID and class */
	cdev_init(&grif_fpga->cdev, &grif_mgr_fops);
	err = cdev_add(&grif_fpga->cdev, MKDEV(fpga_major, id), 1);
	if(err)
		goto err_cdev;

	grif_fpga->abstract_fpga = stcmtk_device_create(grif_fpga->dev,
					MKDEV(fpga_major, id), grif_fpga,
					"fpga%d", id);
	if (IS_ERR(grif_fpga->abstract_fpga)) {
		err = PTR_ERR(grif_fpga->abstract_fpga);
		goto err_device;
	}

	/* Enable FPGA power regulators. */
	err = grif_fpga_enable_power(grif_fpga);
	if (err)
		goto err_device;

	/* Flash default ECP5 firmware from /lib/firmware */
	if (grif_fpga_fw_load(grif_fpga, NULL))
		dev_info(grif_fpga->abstract_fpga, "FPGA flashing failed\n");
	else {
		dev_info(grif_fpga->abstract_fpga, "FPGA flashing done\n");
	}

	dev_set_drvdata(grif_fpga->abstract_fpga, grif_fpga);

	finish_completion();

	return 0;

err_device:
	cdev_del(&grif_fpga->cdev);
err_cdev:
	ida_simple_remove(&fpga_ida, id);
	return -ENODEV;
}

/**
 * Bus-independent FPGA deinitialization sequence.
 */
static void grif_fpga_deinit(struct stcmtk_common_fpga *grif_fpga)
{
	if(grif_fpga->features != NULL) {
		stcmtk_fpgafeat_deinit(grif_fpga->dev, &grif_fpga->features);
	}

	stcmtk_device_destroy(MKDEV(fpga_major, grif_fpga->cdev_minor));
	cdev_del(&grif_fpga->cdev);
	ida_simple_remove(&fpga_ida, grif_fpga->cdev_minor);
	put_device(grif_fpga->dev);
	grif_fpga_disable_power(grif_fpga);
}

/**
 * Bus-dependent FPGA initialization sequence.
 */
static int grif_fpga_eim_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;
	int err = 0;
	struct stcmtk_common_fpga *common_fpga = NULL;
	struct grif_fpga *fpga_defined;

	struct fpga_manager *mgr = grif_get_fpga_manager_inst(&pdev->dev);
	/* Check the fpga_manager existance */
	if (IS_ERR(mgr))
		if (PTR_ERR(mgr) == -ENODEV) {
			dev_info(&pdev->dev, "Will try probe later");
			return -EPROBE_DEFER;
		}
		else
			return PTR_ERR(mgr);
	else
		/* Else free it for future use and continue probing. */
		fpga_mgr_put(mgr);

	common_fpga = devm_kzalloc(&pdev->dev, sizeof(struct stcmtk_common_fpga),
								GFP_KERNEL);
	if(!common_fpga)
		return -ENOMEM;

	common_fpga->fpga_defined_fields = devm_kzalloc(&pdev->dev,
						    sizeof(struct grif_fpga),
						    GFP_KERNEL);

	if(!common_fpga->fpga_defined_fields)
		return -ENOMEM;

	fpga_defined = (struct grif_fpga*)common_fpga->fpga_defined_fields;
	common_fpga->dev = &pdev->dev;
	dev_set_drvdata(common_fpga->dev, common_fpga);
	fpga_defined->ops = &ops;

	/* Get platform device resources and remap it to IO range */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base)) {
		dev_err(common_fpga->dev, "ioremap failed!\n");
		return PTR_ERR(base);
	}

	/* Save IO base address to FPGA drv. data */
	fpga_defined->private_data = base;

	/* Do not create regmap in test mode */
	if (!fpga_defined->test_mode) {
		common_fpga->regmap = devm_regmap_init(common_fpga->dev, NULL,
				common_fpga, &grif_fpga_eim_regmap_config);
		if (IS_ERR(common_fpga->regmap)) {
			dev_err(common_fpga->dev, "regmap init failed!\n");
			err =  PTR_ERR(common_fpga->regmap);
			goto end_probe;
		}
	}

	err = grif_fpga_init(common_fpga);
	if (err)
		goto err_power_on_fault;

	/* Do not parse features in test mode */
	if (!fpga_defined->test_mode)
		common_fpga->features = stcmtk_fpga_parse_features((struct stcmtk_common_fpga *)common_fpga);

	return 0;

err_power_on_fault:
	grif_fpga_disable_power(common_fpga);
end_probe:
	return err;
}

/**
 * Bus-dependent FPGA deinitialization sequence.
 */
static int grif_fpga_eim_remove(struct platform_device *pdev)
{
	struct stcmtk_common_fpga *grif_fpga = dev_get_drvdata(&pdev->dev);
	if (grif_fpga) {
		grif_fpga_deinit(grif_fpga);
	}
	return 0;
}

static const struct of_device_id grif_fm_eim_of_match[] = {
	{.compatible = "stcmtk,grif-fpga-mgr-eim",},
	{},
};
MODULE_DEVICE_TABLE(of, grif_fm_eim_of_match);

static struct platform_driver grif_fpga_eim_driver = {
	.probe = grif_fpga_eim_probe,
	.remove = grif_fpga_eim_remove,
	.driver = {
		.name = "grif_fpga_manager",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(grif_fm_eim_of_match),
	},
};

/**
 * Bus-dependent FPGA initialization sequence.
 */
static int grif_fpga_spi_probe(struct spi_device *spidev)
{
	int err = 0;
	struct stcmtk_common_fpga *common_fpga = NULL;
	struct grif_fpga *fpga_defined;

	struct fpga_manager *mgr = grif_get_fpga_manager_inst(&spidev->dev);
	// Check the fpga_manager existance
	if (IS_ERR(mgr))
		if (PTR_ERR(mgr) == -ENODEV) {
			dev_info(&spidev->dev, "Will try probe later");
			return -EPROBE_DEFER;
		}
		else
			return PTR_ERR(mgr);
	else
		// Else free it for future use and continue probing.
		fpga_mgr_put(mgr);

	common_fpga = devm_kzalloc(&spidev->dev, sizeof(struct stcmtk_common_fpga),
								GFP_KERNEL);
	if(!common_fpga)
		return -ENOMEM;

	common_fpga->fpga_defined_fields = devm_kzalloc(&spidev->dev,
							sizeof(struct grif_fpga),
							GFP_KERNEL);

	if(!common_fpga->fpga_defined_fields)
		return -ENOMEM;

	fpga_defined = (struct grif_fpga*)common_fpga->fpga_defined_fields;
	common_fpga->dev = &spidev->dev;
	dev_set_drvdata(common_fpga->dev, common_fpga);
	fpga_defined->ops = &ops;
	mutex_init(&fpga_defined->regmap_lock);

	common_fpga->regmap = devm_regmap_init(common_fpga->dev, NULL,
				common_fpga, &grif_fpga_spi_regmap_config);

	if (IS_ERR(common_fpga->regmap)) {
		dev_err(common_fpga->dev, "regmap init failed!\n");
		err =  PTR_ERR(common_fpga->regmap);

		return err;
	}

	err = grif_fpga_init(common_fpga);
	if (err) {
		grif_fpga_disable_power(common_fpga);
		return err;
	}

	fpga_defined->cs = gpiod_get(&spidev->dev, "cs", GPIOD_OUT_LOW);

	if (IS_ERR(fpga_defined->cs)) {
		err = PTR_ERR(fpga_defined->cs);
		dev_err(&spidev->dev, "Failed to get CS GPIO: %d\n", err);
		return err;
	}

	/* Do not parse features in test mode */
	if (!fpga_defined->test_mode)
		common_fpga->features = stcmtk_fpga_parse_features((struct stcmtk_common_fpga *)common_fpga);

	gpiod_put(fpga_defined->cs);

	return 0;
}

static int grif_fpga_spi_remove(struct spi_device *spidev)
{
	struct stcmtk_common_fpga *grif_fpga = dev_get_drvdata(&spidev->dev);
	if (grif_fpga) {
		grif_fpga_deinit(grif_fpga);
	}

	return 0;
}

static const struct of_device_id grif_fm_spi_of_match[] = {
	{.compatible = "stcmtk,grif-fpga-mgr-spi",},
	{},
};
MODULE_DEVICE_TABLE(of, grif_fm_spi_of_match);

static const struct spi_device_id grif_fpga_mgr_spi_id[] = {
	{.name = "grif_fpga_mgr_spi"},
	{}
};
MODULE_DEVICE_TABLE(spi, grif_fpga_mgr_spi_id);

static struct spi_driver grif_fpga_spi_driver = {
	.probe = grif_fpga_spi_probe,
	.remove = grif_fpga_spi_remove,
	.driver = {
		.name = "grif_fpga_manager",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(grif_fm_spi_of_match),
	},
};

static ssize_t fpga_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stcmtk_common_fpga *g = dev_get_drvdata(dev);
	struct grif_fpga *grif = (struct grif_fpga*)g->fpga_defined_fields;

	if (!g)
		return -ENODEV;
	//TODO: add correct number of elements
	return snprintf(buf, strlen(grif->name), "%s\n", grif->name);
}

static DEVICE_ATTR(name, S_IRUGO, fpga_name_show, NULL);

static ssize_t througput_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stcmtk_common_fpga *grif_fpga = dev_get_drvdata(dev);

	if (!grif_fpga)
		return -ENODEV;

	return sprintf(buf, "Usage: mem_offset mem_size bench_repeats\n# echo \"0 32768 1000\" > /sys/class/fpga/fpga0/bench\n");
}

static ssize_t througput_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	struct stcmtk_common_fpga *grif_fpga = dev_get_drvdata(dev);

	struct througput_test_settings settings = {
		.cycles = 1000,
		.io_size = TEST_IO_ALL,
		.rw_mode = TEST_MODE_ALL,
		.access_mode = TEST_ACCESS_ALL,
	};

	int ret = 0;

	if (!grif_fpga)
		return -ENODEV;

	settings.dev = grif_fpga->dev;
	settings.base = ((struct grif_fpga*)grif_fpga->fpga_defined_fields)->private_data;

	ret = sscanf(buf, "%d %d %d", &settings.mem_offset, &settings.mem_size,  &settings.bench_repeats);

	if (ret != 3)
		return -ENXIO;

	grif_througput_test(&settings);

	return count;
}

static DEVICE_ATTR(bench, S_IRUGO | S_IWUSR, througput_test_show, througput_test_store);


static ssize_t mem_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct grif_fpga *grif_fpga = dev_get_drvdata(dev);

	if (!grif_fpga)
		return -ENODEV;

	return sprintf(buf, "Usage: test_repeats\n# echo \"1000\" > /sys/class/fpga/fpga0/mem_test\n");
}

static ssize_t mem_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	struct grif_fpga *grif_fpga = dev_get_drvdata(dev);

	int ret = 0;
	int test_repeats = 1000;

	if (!grif_fpga)
		return -ENODEV;

	ret = sscanf(buf, "%d", &test_repeats );

	if (ret != 1)
		return -ENXIO;

	grif_mem_test( grif_fpga, test_repeats );
	return count;
}

static DEVICE_ATTR( mem_test, S_IRUGO | S_IWUSR, mem_test_show, mem_test_store);

static struct attribute *fpga_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_bench.attr,
	&dev_attr_mem_test.attr,
	NULL,
};

static const struct attribute_group fpga_group = {
	.attrs = fpga_attrs,
};

const struct attribute_group *fpga_groups[] = {
	&fpga_group,
	NULL,
};

/* This actions have to be fulfilled on module shutdown process */
static void exit_actions(void)
{
	stcmtk_class_destroy();
	ida_destroy(&fpga_ida);
}

static int __init grif_fpga_mgr_init(void)
{
	int err = 0;
	dev_t fpga_dev;

	err = stcmtk_class_create(THIS_MODULE, "fpga", fpga_groups);
	if (err)
		return err;

	err = alloc_chrdev_region(&fpga_dev, 0, GRIF_FPGA_MAX_MINORS, "fpga");
	if (err) {
		exit_actions();
		return err;
	}

	fpga_major = MAJOR(fpga_dev);

	err = platform_driver_register(&grif_fpga_eim_driver);
	if (err)
		exit_actions();

	err = spi_register_driver(&grif_fpga_spi_driver);
	if (err) {
		platform_driver_unregister(&grif_fpga_eim_driver);
		exit_actions();
	}

	return err;
}

static void __exit grif_fpga_mgr_exit(void)
{
	platform_driver_unregister(&grif_fpga_eim_driver);
	spi_unregister_driver(&grif_fpga_spi_driver);
	exit_actions();
	return;
}

module_init(grif_fpga_mgr_init);
module_exit(grif_fpga_mgr_exit);

MODULE_SOFTDEP("pre: ecp5_fpga_mgr");

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("STC Metrotek Grif platform FPGA Manager");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
