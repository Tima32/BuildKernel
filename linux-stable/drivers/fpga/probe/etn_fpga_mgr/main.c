#define pr_fmt(fmt) "ETN_FPGA: %s:%d: " fmt, __FUNCTION__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/of_platform.h>
#include <linux/fs.h>

#include "main.h"
#include "etn_fpga.h"

#define ETN_FPGA_MAX_MINORS 256

static DEFINE_IDA(fpga_ida);

static struct fpga_manager *etn_get_fpga_mgr(struct device *dev)
{
	struct fpga_manager *mgr = NULL;
	struct device_node *altr_node = of_parse_phandle(dev->of_node,
							 "altr-fpga-mgr", 0);

	if (!altr_node) {
		dev_err(dev, "Altera SOCFPGA manager DT node not found!\n");
		return ERR_PTR(-ENOENT);
	}

	mgr = of_fpga_mgr_get(altr_node);
	if (IS_ERR(mgr))
		dev_err(dev, "Failed to get reference to FPGA mgr: %ld\n",
			PTR_ERR(mgr));

	of_node_put(altr_node);

	return mgr;
}

int etn_fpga_state_get(struct stcmtk_common_fpga *etn_fpga)
{
	int state = -1;
	struct fpga_manager *mgr = NULL;

	if (!etn_fpga) {
		pr_err("etn_fpga is NULL\n");
		return -EINVAL;
	}

	mgr = ((struct etn_fpga*)etn_fpga->fpga_defined_fields)->mgr;
	if (!mgr) {
		pr_err("Invalid mgr from container_of\n");
		return -EINVAL;
	}

	state = mgr->mops->state(mgr);
	if (state == FPGA_MGR_STATE_OPERATING) {
		return ETN_FPGA_STATE_USER_MODE;
	} else {
		return ETN_FPGA_STATE_INVALID;
	}
}
EXPORT_SYMBOL(etn_fpga_state_get);

int etn_fpga_ref_get(struct stcmtk_common_fpga *etn_fpga)
{
	int old;

	if (!etn_fpga) {
		pr_err("etn_fpga is NULL\n");
		return -EINVAL;
	}

	struct etn_fpga *fpga_defined = (struct etn_fpga*)etn_fpga->fpga_defined_fields;
	old = atomic_inc_return(&fpga_defined->refcnt);
	pr_debug("Inc refcnt = %d\n", old);
	return 0;
}
EXPORT_SYMBOL(etn_fpga_ref_get);

int etn_fpga_ref_put(struct stcmtk_common_fpga *etn_fpga)
{
	int old;

	if (!etn_fpga)
		return -EINVAL;

	struct etn_fpga *fpga_defined = (struct etn_fpga*)etn_fpga->fpga_defined_fields;
	old = atomic_dec_return(&fpga_defined->refcnt);
	pr_debug("Dec refcnt = %d\n", old);
	return 0;
}
EXPORT_SYMBOL(etn_fpga_ref_put);

static char *fw_path = "fpga/fpga.rbf";
module_param(fw_path, charp, S_IRUGO);
MODULE_PARM_DESC(fw_path,
		 "Path to firmware used to flash FPGA at module load time.");

/**
 * Flash FPGA firmware
 *
 * If we failed to flash FPGA we just move on in probe.
 *
 * @param etn_fpga - pointer to current stcmtk_common_fpga struct
 * @param fw       - information about used firmware.
 *
 * If fw equals NULL, function uses default /lib/firmware/fpga/fpga.rbf file.
 */
static int etn_fpga_load_fw(struct stcmtk_common_fpga *etn_fpga, struct etn_firmware *fw)
{
	int ret = 0;
	struct fpga_manager *mgr = ((struct etn_fpga*)etn_fpga->fpga_defined_fields)->mgr;
	struct fpga_image_info *info = NULL;
	const char *fpga_fw_name = NULL;

	info = fpga_image_info_alloc(etn_fpga->dev);
	if (!info) {
		dev_info(etn_fpga->dev,
			"Unable to allocate fpga_image_info\n");
		return -ENOMEM;
	}

	info->flags = 0;

	if (of_property_read_string(etn_fpga->dev->of_node,
					"firmware-name", &fpga_fw_name) != 0) {
		fpga_fw_name = fw_path;
	}

	if (fw == NULL) {
		info->firmware_name = devm_kstrdup(etn_fpga->dev, fpga_fw_name,
						GFP_KERNEL);
		strcpy(info->firmware_name, fpga_fw_name);
	}
	else {
		info->buf = (const char*) fw->buf;
		info->count = fw->size;
	}
	ret = fpga_mgr_load(mgr, info);

	if (ret) {
		dev_info(etn_fpga->dev,
				"fpga_mgr_firmware_load returns %d\n", ret);
		goto err;
	}

	if (etn_fpga_state_get(etn_fpga) != ETN_FPGA_STATE_USER_MODE) {
		dev_err(etn_fpga->dev, "FPGA not ready!\n");
		ret = -ENODEV;
	}

err:
	fpga_image_info_free(info);

	return ret;
}

static ssize_t etn_mgr_write(struct file *file, const char __user *buf,
                             size_t count, loff_t *offset)
{
	struct stcmtk_common_fpga *etn_fpga = file->private_data;
	struct etn_firmware etn_fw;
	char *kern_buf = NULL;
	int ret = 0;

	kern_buf = vmemdup_user(buf, count);
	if (IS_ERR(kern_buf)) {
		return PTR_ERR(kern_buf);
	}

	etn_fw.buf  = kern_buf;
	etn_fw.size = count;
	ret = etn_fpga_load_fw(etn_fpga, &etn_fw);

	if (ret) {
		pr_err("Failed to flash FPGA: %d\n", ret);
		vfree(kern_buf);
		return -EFAULT;
	}

	pr_info("Firmware loaded\n");

	vfree(kern_buf);
	return count;
}

static int etn_mgr_open(struct inode *inode, struct file *file)
{
	struct stcmtk_common_fpga *etn_fpga = container_of(inode->i_cdev,
						struct stcmtk_common_fpga,
						cdev);

	file->private_data = etn_fpga;

	return 0;
}

static int etn_mgr_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t etn_mgr_read(struct file *file, char __user *buf, size_t count,
								loff_t *offset)
{
	return -EPERM;
}

/* major number for device of "fpga" class */
static int etn_fpga_major;

static const struct file_operations etn_chrdev_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = etn_mgr_read,
	.open = etn_mgr_open,
	.write = etn_mgr_write,
	.release = etn_mgr_release
};

static int etn_fpga_reg_read_regmap_cb(void *ctx, unsigned int reg,
							unsigned int *val)
{
	struct stcmtk_common_fpga *f = ctx;
	void __iomem *reg_address = ((struct etn_fpga*)(f->fpga_defined_fields))->io_base + reg * 2;

	*val = ioread16(reg_address);

	return 0;
}

static int etn_fpga_reg_write_regmap_cb(void *ctx, unsigned int reg,
							unsigned int val)
{
	struct stcmtk_common_fpga *f = ctx;
	void __iomem *reg_address = ((struct etn_fpga*)(f->fpga_defined_fields))->io_base + reg * 2;

	iowrite16(val, reg_address);

	return 0;
}
static bool etn_fpga_accessiable_reg_regmap_cb(struct device *dev, unsigned int reg)
{
	struct stcmtk_common_fpga *etn_fpga = dev_get_drvdata(dev);
	if (!etn_fpga) {
		dev_err(dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	if (etn_fpga_state_get(etn_fpga) == ETN_FPGA_STATE_USER_MODE)
		return true;

	return false;
}

static struct regmap_config etn_fpga_regmap_config = {
	.reg_bits         = 16,
	.reg_stride       = 1,
	.val_bits         = 16,
	.fast_io          = true,
	.reg_read         = etn_fpga_reg_read_regmap_cb,
	.reg_write        = etn_fpga_reg_write_regmap_cb,
	.readable_reg     = etn_fpga_accessiable_reg_regmap_cb,
	.writeable_reg    = etn_fpga_accessiable_reg_regmap_cb,
	.max_register     = 0xffff,
};

static int etn_fpga_probe(struct platform_device *pdev)
{
	struct fpga_manager *mgr;
	struct resource *res;
	void __iomem *io_base;
	int ret = 0;
	int id = 0;
	struct stcmtk_common_fpga *common_fpga = NULL;
	struct etn_fpga *fpga_defined;

	common_fpga = devm_kzalloc(&pdev->dev, sizeof(struct stcmtk_common_fpga), GFP_KERNEL);
	if (!common_fpga)
		return -ENOMEM;

	common_fpga->fpga_defined_fields = devm_kzalloc(&pdev->dev, sizeof(struct etn_fpga),
						GFP_KERNEL);
	if(!common_fpga->fpga_defined_fields)
		return -ENOMEM;

	fpga_defined = (struct etn_fpga*)common_fpga->fpga_defined_fields;
	atomic_set(&fpga_defined->refcnt, 0);

	mgr = etn_get_fpga_mgr(&pdev->dev);
	if (IS_ERR(mgr)) {
		ret = PTR_ERR(mgr);
		goto err;
	}

	fpga_defined->mgr = mgr;
	id = ida_simple_get(&fpga_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		ret = id;
		dev_err(common_fpga->dev, "Unable to get ID\n");
		goto err_ida;
	}
	common_fpga->cdev_minor = id;

	/* create chrdev "fpga0" of class "fpga" to create
	 * /sys/class/fpga/fpga0/ for storing features list there */
	cdev_init(&common_fpga->cdev, &etn_chrdev_ops);
	ret = cdev_add(&common_fpga->cdev, MKDEV(etn_fpga_major, id), 1);
	if (ret)
		goto err_cdev;

	common_fpga->dev = &pdev->dev;
	dev_set_drvdata(common_fpga->dev, common_fpga);

	common_fpga->abstract_fpga = stcmtk_device_create(common_fpga->dev,
			MKDEV(etn_fpga_major, id), common_fpga, "fpga%d", id);
	if (IS_ERR(common_fpga->dev)) {
		ret = PTR_ERR(common_fpga->dev);
		goto err_device;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res)) {
		dev_err(&pdev->dev, "Unable to get resource!\n");
		ret = PTR_ERR(res);
		goto err_get_res;
	}

	io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(io_base)) {
		dev_err(&pdev->dev, "Unable to map resource!\n");
		ret = PTR_ERR(io_base);
		goto err_ioremap;
	}

	fpga_defined->io_base = io_base;

	common_fpga->regmap = devm_regmap_init(&pdev->dev, NULL, common_fpga,
						&etn_fpga_regmap_config);
	if (IS_ERR(common_fpga->regmap)) {
		dev_err(&pdev->dev, "Unable to init FPGA regmap!\n");
		ret = PTR_ERR(common_fpga->regmap);
		goto err_regmap;
	}


	/* Try to flash FPGA */
	etn_fpga_load_fw(common_fpga, NULL);
	if (etn_fpga_state_get(common_fpga) == ETN_FPGA_STATE_USER_MODE) {
		stcmtk_fpga_parse_features(common_fpga);
	}

	dev_set_drvdata(common_fpga->abstract_fpga, common_fpga);

	finish_completion();

	return 0;

err_regmap:
err_ioremap:
err_get_res:
	stcmtk_device_destroy(MKDEV(etn_fpga_major, id));

err_device:
	cdev_del(&common_fpga->cdev);

err_cdev:
	ida_simple_remove(&fpga_ida, id);

err_ida:
err:
	return ret;
}

static int etn_fpga_remove(struct platform_device *pdev)
{
	struct stcmtk_common_fpga *etn_fpga = dev_get_drvdata(&pdev->dev);
	if (!etn_fpga) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	if (etn_fpga->features)
		stcmtk_fpgafeat_deinit(etn_fpga->dev, &etn_fpga->features);
	stcmtk_device_destroy(MKDEV(etn_fpga_major, etn_fpga->cdev_minor));
	cdev_del(&etn_fpga->cdev);
	ida_simple_remove(&fpga_ida, etn_fpga->cdev_minor);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id etn_fpga_of_match[] = {
	{ .compatible = "stcmtk,etn-fpga-mgr", },
	{},
};

MODULE_DEVICE_TABLE(of, etn_fpga_of_match);
#endif

static struct platform_driver etn_fpga_driver = {
	.remove = etn_fpga_remove,
	.driver = {
		.name	= "etn_fpga_manager",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(etn_fpga_of_match),
	},
};

static int __init etn_fpga_init(void)
{
	int err = 0;
	dev_t fpga_dev;

	err = stcmtk_class_create(THIS_MODULE, "fpga", NULL);
	if (err) {
		return err;
	}

	err = alloc_chrdev_region(&fpga_dev, 0, ETN_FPGA_MAX_MINORS, "etn_fpga_mgr");
	if (err < 0) {
		stcmtk_class_destroy();
		ida_destroy(&fpga_ida);
		return err;
	}

	etn_fpga_major = MAJOR(fpga_dev);
	return platform_driver_probe(&etn_fpga_driver, etn_fpga_probe);
}

static void __exit etn_fpga_exit(void)
{
	platform_driver_unregister(&etn_fpga_driver);
	stcmtk_class_destroy();
	ida_destroy(&fpga_ida);
}

module_init(etn_fpga_init);
module_exit(etn_fpga_exit);

MODULE_AUTHOR("Alan Tull <atull@altera.com>");
MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("SoC FPGA Manager");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
