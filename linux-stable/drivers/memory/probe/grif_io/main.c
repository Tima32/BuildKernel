#include <linux/slab.h>
#include <linux/module.h>

#include <linux/tty.h>

#include <grif/grif_fpga_mgr/grif_fpga.h>
#include <common_fpga/fpgafeat.h>
#include <linux/spi/spi.h>
#include <linux/of_platform.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/regmap.h>

#include "main.h"

int grif_io_open(struct inode *inode, struct file *fp);
loff_t grif_io_llseek(struct file *fp, loff_t off, int whence);
int grif_io_release(struct inode *inode, struct file *fp);
int grif_io_mmap(struct file *fp, struct vm_area_struct *vma);
ssize_t grif_io_read(struct file *fp, char __user *ubuf, size_t len, loff_t *offp);
ssize_t grif_io_write(struct file *fp, const char __user *ubuf, size_t len, loff_t *offp);

struct drvdata {
	u32		 major;
	u32		 minor;
	dev_t		 dev_num;
	struct stcmtk_common_fpga *fpga;
	struct cdev	 chardev;
	struct class	 *sys_class;
	struct regmap	 *regmap;
};

/*
 * File operations
 */
int grif_io_open(struct inode *inode, struct file *fp)
{
	fp->private_data = container_of(inode->i_cdev, struct drvdata, chardev);
	return 0;
}

loff_t grif_io_llseek(struct file *fp, loff_t off, int whence)
{
	loff_t np = 0;
	struct drvdata *privdata = (struct drvdata *) fp->private_data;
	int nxfer, max_register;
	size_t maxaddr_bytes;
	u32 nxfer_rem;

	nxfer = regmap_get_val_bytes(privdata->regmap);
	if (nxfer <= 0)
		return -EINVAL;

	max_register = regmap_get_max_register(privdata->regmap);
	if (max_register <= 0)
		return -EINVAL;

	maxaddr_bytes = nxfer * max_register;

	iodbg("llseek %lld, whence %d\n", off, whence);

	div_u64_rem(off, nxfer, &nxfer_rem);
	if (nxfer_rem){
		pr_err(NAME"Cannot convert to word address, bytes in word %d \n", nxfer);
		return -EINVAL;
	}

	switch (whence) {
	case SEEK_SET:
		np = off;
		break;
	case SEEK_CUR:
		np = fp->f_pos + off;
		break;
	case SEEK_END:
		np = maxaddr_bytes + off;
		break;
	}
	if (np < 0 || np > maxaddr_bytes)
		return -EPROTO;
	fp->f_pos = np;
	return np;
}

int grif_io_release(struct inode *inode, struct file *fp)
{
	return 0;
}

int grif_io_mmap(struct file *fp, struct vm_area_struct *vma)
{
	return 0;
}

ssize_t grif_io_read(struct file *fp, char __user *ubuf, size_t len, loff_t *offp)
{
	/* TODO: EIM_BLOCK_SIZE is something only fpga_mgr should know about when
	 * implementing regmap operations.
	 */
	struct drvdata *privdata = (struct drvdata *) fp->private_data;
	size_t nleft;
	size_t nxfer = regmap_get_val_bytes(privdata->regmap);
	u32 temp = 0;
	u32 nxfer_rem;

	iodbg("try read %d bytes from %d offset\n", len, (int)*offp);

	if (len % nxfer) {
		pr_err(NAME"Need a multiple of %d bytes to read\n", nxfer);
		return -EINVAL;
		}


	div_u64_rem(*offp, nxfer, &nxfer_rem);
	if (nxfer_rem){
		pr_err(NAME"Cannot convert to word address, bytes in word %d \n", nxfer);
		return -EINVAL;
	}

	nleft = len;

	while (nleft > 0) {
		iodbg(NAME": regmap_read(regmap, %llu, &value)\n", *offp);

		if (regmap_read(privdata->regmap,
				div_u64(*offp, nxfer), &temp)) {
			pr_err(NAME"read error\n");
			return -EFAULT;
		}

		if (copy_to_user(ubuf, &temp, nxfer))
			return -EFAULT;

		nleft -= nxfer;
		ubuf  += nxfer;
		*offp += nxfer;
	}

	return (len - nleft);
}


ssize_t grif_io_write(struct file *fp, const char __user *ubuf, size_t len,
								loff_t *offp)
{
	size_t nleft;
	struct drvdata *privdata = fp->private_data;
	size_t nxfer = regmap_get_val_bytes(privdata->regmap);
	u32 temp = 0;
	u32 nxfer_rem;

	iodbg("try write %d bytes to %d offset\n", len, (int)*offp);

	if (len % nxfer) {
		pr_err(NAME"Need a multiple of %d bytes to write\n", nxfer);
		return -EINVAL;
		}

	div_u64_rem(*offp, nxfer, &nxfer_rem);
	if (nxfer_rem){
		pr_err(NAME"Cannot convert to word address, bytes in word %d \n", nxfer);
		return -EINVAL;
	}

	nleft = len;

	while (nleft > 0) {
		if (copy_from_user(&temp, ubuf, nxfer))
			return -EFAULT;

		iodbg(NAME": regmap_write(regmap, %llu, %u)\n", *offp, temp);
		if (regmap_write(privdata->regmap,
				div_u64(*offp, nxfer), temp)) {
			pr_err(NAME": write error\n");
			return -EFAULT;
		}

		nleft -= nxfer;
		ubuf  += nxfer;
		*offp += nxfer;
	}

	return (len - nleft);
}

static struct file_operations fops = {
	owner	: THIS_MODULE,
	open	: grif_io_open,
	llseek	: grif_io_llseek,
	release	: grif_io_release,
	mmap	: grif_io_mmap,
	read	: grif_io_read,
	write	: grif_io_write,
};

/*
 * device initialization
 */

static int grif_io_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node = (&pdev->dev)->of_node;
	struct stcmtk_common_fpga *fpga_dev = NULL;
	struct drvdata *privdata;

	iodbg("probing @ %s:%d\n", __func__, __LINE__);

	privdata = devm_kzalloc(&pdev->dev, sizeof(struct drvdata), GFP_KERNEL);
	if (!privdata) {
		pr_err(NAME": can't allocate memory\n");
		goto anything_failed;
	}

	fpga_dev = stcmtk_get_fpga(node);

	if (!fpga_dev) {
		dev_err(&pdev->dev, "can't get FPGA manager.");
		goto anything_failed;
	}

	privdata->fpga = fpga_dev;
	privdata->regmap = dev_get_regmap(fpga_dev->dev, NULL);
	if (!privdata->regmap) {
		iodbg("unable to find the registers map @ %s:%d\n", __func__, __LINE__);
		goto fpgadev_failed;
	}

	ret = alloc_chrdev_region(&privdata->dev_num, 0, 1, "fpga_regs");
	if (ret < 0)
	{
		goto challoc_failed;
	}

	privdata->major = MAJOR(privdata->dev_num);
	privdata->minor = MINOR(privdata->dev_num);

	privdata->sys_class = class_create(THIS_MODULE, "grif_io");
	if (IS_ERR(&(privdata->sys_class)))
	{
		goto class_failed;
	}

	//TODO: register grif_io with its sequenca number, like grif_io0,
	//grif_io1, etc.
	privdata->chardev.owner = THIS_MODULE;
	kobject_set_name(&(privdata->chardev.kobj), "grif_io");
	cdev_init(&(privdata->chardev), &fops);
	if (cdev_add(&(privdata->chardev),
			MKDEV(privdata->major, privdata->minor), 1))
	{
		goto cdev_failed;
	}

	device_create(privdata->sys_class, NULL, MKDEV(privdata->major,
	      privdata->minor), &(privdata->chardev), "grif_io");

	platform_set_drvdata(pdev, privdata);

	iodbg("init done\n");

	return 0;

cdev_failed:
	kobject_put(&(privdata->chardev.kobj));
	class_destroy(privdata->sys_class);
class_failed:
challoc_failed:
fpgadev_failed:
	stcmtk_put_fpga(fpga_dev);
anything_failed:
	devm_kfree(&pdev->dev, privdata);
	pr_err(NAME"init failed\n");
	return -ENODEV;
}

static int grif_io_remove(struct platform_device *pdev)
{
	struct drvdata *privdata;

	privdata = (struct drvdata *) platform_get_drvdata(pdev);

	iodbg("removing @ %s:%d\n", __func__, __LINE__);

	device_destroy(privdata->sys_class, MKDEV(privdata->major,
						privdata->minor));

	cdev_del(&(privdata->chardev));
	unregister_chrdev_region(MKDEV(privdata->major, privdata->minor), 1);

	class_destroy(privdata->sys_class);

	unregister_chrdev_region(privdata->dev_num, 1);

	stcmtk_put_fpga(privdata->fpga);

	devm_kfree(&pdev->dev, privdata);

	iodbg("exit\n");
	return 0;
}

static const struct of_device_id grif_io_match[] = {
	{ .compatible = "stcmtk,grif-io" },
	{},
};
MODULE_DEVICE_TABLE(of, grif_io_match);

static struct platform_driver grif_io_driver = {
	.driver = {
		.name = NAME,
		.of_match_table = grif_io_match,
	},
	.probe = grif_io_probe,
	.remove = grif_io_remove,
};

static int __init grif_io_init(void)
{
	int ret;

	ret = platform_driver_register(&grif_io_driver);
	if (ret)
		iodbg("error registering device\n");

	return ret;
}

static void __exit grif_io_exit(void)
{
	platform_driver_unregister(&grif_io_driver);
}

MODULE_SOFTDEP("pre: grif_fpga_mgr");

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("FPGA IO through cdev driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("grif_io");

module_init(grif_io_init);
module_exit(grif_io_exit);
