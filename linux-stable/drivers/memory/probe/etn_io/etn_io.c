/**
 * etn_io.c: SoC FPGA char device driver.
 *
 * Implements /dev/etn with open/close/read/write operations.
 * Provides I/O access to FPGA.
 * Actual I/O redirected to mapped FPGA memory.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#ifndef DRV_VERSION
#define DRV_VERSION "2.0.3"
#endif

#define DEV_NAME "etn"

struct drvdata {
	struct platform_device *pdev;
	struct fpga_io         *io;
	struct miscdevice      miscdev;
	struct regmap          *regmap;
};

/* TODO: remove global ponters */
static struct drvdata *gl_data = NULL;

static int etn_open(struct inode *inode, struct file *fp)
{
	struct drvdata *data =
		container_of(fp->private_data, struct drvdata, miscdev);

	if (!data)
		return -EINVAL;

	fp->private_data = data;
	return 0;
}

static int etn_release(struct inode *inode, struct file *fp)
{
	struct drvdata *data = fp->private_data;
	if (!data)
		return -EINVAL;

	return 0;
}

static loff_t etn_llseek(struct file *fp, loff_t off, int whence)
{
	loff_t newpos;
	struct drvdata *data = (struct drvdata *)fp->private_data;
	struct device *dev = &data->pdev->dev;
	int nxfer, max_register;
	size_t maxaddr_bytes;
	u32 nxfer_rem;


	nxfer = regmap_get_val_bytes(data->regmap);
	if (nxfer <= 0)
		return -EINVAL;

	max_register = regmap_get_max_register(data->regmap);
	if (max_register <= 0)
		return -EINVAL;

	maxaddr_bytes = nxfer * max_register;

	div_u64_rem(off, nxfer, &nxfer_rem);
	if (nxfer_rem){
		dev_err(dev, "Cannot convert to word address, bytes in word %d \n", nxfer);
		return -EINVAL;
	}

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = fp->f_pos + off;
		break;
	case SEEK_END:
		newpos = maxaddr_bytes + off;
		break;
	default:
		return -EINVAL;
	}

	if (newpos < 0 || newpos >= maxaddr_bytes) {
		return -EPROTO;
	}

	fp->f_pos = newpos;

	return newpos;
}

static ssize_t etn_read(struct file *fp, char __user *buf,
                 size_t len, loff_t *pos)
{
	struct drvdata *data = fp->private_data;
	struct device *dev = &data->pdev->dev;
	size_t nleft;
	size_t nxfer = regmap_get_val_bytes(data->regmap);
	u32 temp = 0;
	u32 nxfer_rem;

	if (len % nxfer) {
		dev_err(dev, "Need a multiple of %d bytes to read\n", nxfer);
		return -EINVAL;
	}

	div_u64_rem(*pos, nxfer, &nxfer_rem);
	if (nxfer_rem){
		dev_err(dev, "Cannot convert to word address, bytes in word %d \n", nxfer);
		return -EINVAL;
	}

	nleft = len;
	while (nleft > 0) {
		dev_dbg(dev, "regmap_read(regmap, %llu, &value)\n", *pos);

		if (regmap_read(data->regmap,
				div_u64(*pos, nxfer), &temp)) {
			dev_err(dev, "read error\n");
			return -EFAULT;
		}

		if (copy_to_user(buf, &temp, nxfer))
			return -EFAULT;

		nleft -= nxfer;
		buf  += nxfer;
		*pos += nxfer;
	}

	return (len - nleft);
}

static ssize_t etn_write(struct file *fp, const char __user *buf,
                  size_t len, loff_t *pos)
{
	size_t nleft;
	struct drvdata *data = fp->private_data;
	struct device *dev = &data->pdev->dev;
	size_t nxfer = regmap_get_val_bytes(data->regmap);
	u32 temp = 0;
	u32 nxfer_rem;


	if (len % nxfer) {
		dev_err(dev, "Need a multiple of %d bytes to write\n", nxfer);
		return -EINVAL;
	}

	div_u64_rem(*pos, nxfer, &nxfer_rem);
	if (nxfer_rem){
		dev_err(dev, "Cannot convert to word address, bytes in word %d \n", nxfer);
		return -EINVAL;
	}

	nleft = len;

	while (nleft > 0) {
		if (copy_from_user(&temp, buf, nxfer))
			return -EFAULT;

		dev_dbg(dev, "regmap_write(regmap, %llu, %u)\n", *pos, temp);
		if (regmap_write(data->regmap,
				div_u64(*pos, nxfer), temp)) {
			dev_err(dev, "write error\n");
			return -EFAULT;
		}

		nleft -= nxfer;
		buf  += nxfer;
		*pos += nxfer;
	}

	return (len - nleft);
}

const struct file_operations etn_fops = {
	.owner   = THIS_MODULE,
	.open    = etn_open,
	.release = etn_release,
	.llseek  = etn_llseek,
	.read    = etn_read,
	.write   = etn_write,
};

static const struct miscdevice etn_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = DEV_NAME,
	.fops  = &etn_fops,
};

static void etn_free(struct drvdata *data)
{
	if (data) {
		if( data->miscdev.this_device ) {
			misc_deregister(&data->miscdev);
			data->miscdev.this_device = NULL;
		}

		kfree(data);
	}
}

static int etn_io_probe(struct platform_device *pdev)
{
	struct drvdata *d;
	struct stcmtk_common_fpga *f = NULL;

	if (gl_data != NULL) {
		dev_err(&pdev->dev, "/dev/%s is already registered!\n", DEV_NAME);
		return -EBUSY;
	}

	f = stcmtk_get_fpga((&pdev->dev)->of_node);
	if (!f) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	if (etn_fpga_state_get(f) != ETN_FPGA_STATE_USER_MODE) {
		dev_err(&pdev->dev, "Won't register I/O module when FPGA is not flashed\n");
		return -EIO;
	}

	if (etn_fpga_ref_get(f)) {
		dev_err(&pdev->dev, "Failed to get ref on FPGA\n");
		return -EIO;
	}

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d) {
		dev_err(&pdev->dev, "Failed to allocate memory for /dev/%s\n", DEV_NAME);
		goto err_put;
	}

	dev_dbg(&pdev->dev, "Registering miscdev\n");
	d->miscdev = etn_miscdevice;
	if (misc_register(&d->miscdev)) {
		dev_err(&pdev->dev, "Cannot register misc device\n");
		goto err_free;
	}

	gl_data = d;
	d->pdev = pdev;
	platform_set_drvdata(pdev, d);
	d->regmap = f->regmap;

	return 0;

err_free:
	if (d)
		kfree(d);
err_put:
	etn_fpga_ref_put(f);

	return -EINVAL;
}

static int etn_io_remove(struct platform_device *pdev)
{
	struct drvdata *d;
	struct stcmtk_common_fpga *f = stcmtk_get_fpga((&pdev->dev)->of_node);
	if (!f) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	d = platform_get_drvdata(pdev);
	if (d)
		etn_free(d);

	/* We don't need FPGA device anymore */
	etn_fpga_ref_put(f);

	/* NULL global data */
	gl_data = NULL;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id etn_io_of_match[] = {
	{ .compatible = "stcmtk,etn-io", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, etn_io_of_match);
#endif

static struct platform_driver etn_io_driver = {
	.probe = etn_io_probe,
	.remove = etn_io_remove,
	.driver = {
		.name	= "etn_io",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(etn_io_of_match),
	},
};

module_platform_driver(etn_io_driver);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("I/O access to Altera SoC FPGA interface");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
