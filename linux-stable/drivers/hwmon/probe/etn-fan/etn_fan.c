#define pr_fmt(fmt) "ETN_FAN: %s:%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>

#include <asm/page.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "etn_fan.h"

struct et_fan_fb {
	struct stcmtk_common_fpga	*fpga;
	struct fpga_feature		*fan;
	struct platform_device		*pdev;
};

static struct fan_regs fan_regs;

/* TODO:
 *   Move this functions (and same from etn-fb.ko) to library.
 */
static void fpga_write_cr(struct et_fan_fb *fandev, int reg, u16 val)
{
	struct stcmtk_common_fpga *fpga = fandev->fpga;
	regmap_write(fpga->regmap, fan_regs.cr + reg, val);
}

static u16 fpga_read_cr(struct et_fan_fb *fandev, int reg)
{
	unsigned tmp;
	struct stcmtk_common_fpga *fpga = fandev->fpga;

	regmap_read(fpga->regmap, fan_regs.cr + reg, &tmp);

	return (u16)tmp;
}

/* FPGA has only 4 modes. So, we round ppm:
 *   - 0 ppm         to off
 *   - [1,127] ppm   to 30%
 *   - [128,255] ppm to 60%
 *   - 256 ppm       to 100%
 */
static u8 pwm_to_mode(u8 pwm)
{
	if (pwm == 0)
		return MODE_PWM_OFF;

	if (pwm == 255)
		return MODE_PWM_MAX;

	if (pwm < 128)
		return MODE_PWM_30PCT;
	else
		return MODE_PWM_60PCT;
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	unsigned long pwm;
	ssize_t ret;
	struct et_fan_fb *fandev = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_PWM)
		return -EINVAL;

	fpga_write_cr(fandev, FAN_MAIN_CR, pwm_to_mode(pwm));

	ret = count;
	return ret;
}

static ssize_t show_pwm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned long mode;
	u8 pwm = 0;

	struct et_fan_fb *fandev = dev_get_drvdata(dev);

	mode = fpga_read_cr(fandev, FAN_MAIN_CR);

	switch (mode) {
	case MODE_PWM_OFF:
		pwm = 0;
		break;

	case MODE_PWM_30PCT:
		pwm = MAX_PWM * 30 / 100;
		break;

	case MODE_PWM_60PCT:
		pwm = MAX_PWM * 60 / 100;
		break;

	case MODE_PWM_MAX:
		pwm = MAX_PWM;
		break;
	}

	return sprintf(buf, "%d\n", pwm);
}

static SENSOR_DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm, 0);

static struct attribute *etn_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(etn);

static int etn_fan_feature(struct et_fan_fb *fandev)
{
	struct fpga_feature *fan;

	fan = stcmtk_fpga_get_feature(fandev->fpga, FPGA_FEATURE_FAN);
	if (!fan)
		return -1;

	fan_regs.cr = stcmtk_get_cr_base_on_port(fandev->fan, 0);
	fan_regs.sr = stcmtk_get_sr_base_on_port(fandev->fan, 0);

	return 0;
}

static int etn_fan_probe(struct platform_device *pdev)
{
	struct device *hwmon_dev;
	struct et_fan_fb *fandev;
	int ret;

	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;

	fandev = devm_kzalloc(&pdev->dev, sizeof(*fandev), GFP_KERNEL);
	if (!fandev)
		return -ENOMEM;


	fandev->pdev = pdev;

	fandev->fpga = stcmtk_get_fpga(np);
	if (!fandev->fpga) {
		dev_err(dev, "Failed to get target FPGA\n");
		return -ENODEV;
	}

	ret = etn_fpga_ref_get(fandev->fpga);
	if (ret) {
		dev_err(dev, "Failed to increment FPGA reference count\n");
		goto err_ref;
	}

	fandev->fan = stcmtk_fpga_get_feature(fandev->fpga, FPGA_FEATURE_FAN);
	if (!fandev->fan) {
		dev_err(dev, "Failed to get FPGA_FEATURE_FAN\n");
		goto err_put;
	}

	fan_regs.cr = stcmtk_get_cr_base_on_port(fandev->fan, 0);
	fan_regs.sr = stcmtk_get_sr_base_on_port(fandev->fan, 0);

	hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
							   "etnfan",
							   fandev,
							   etn_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);

err_put:
	etn_fpga_ref_put(fandev->fpga);
err_ref:
	return -EFAULT;
}

static int etn_fan_remove(struct platform_device *pdev)
{
	struct et_fan_fb *fandev = platform_get_drvdata(pdev);

	if(fandev)
		etn_fpga_ref_put(fandev->fpga);

	return 0;
}

static const struct of_device_id etn_of_match[] = {
	{.compatible = "etn,fan",},
	{},
};
MODULE_DEVICE_TABLE(of, etn_of_match);

static struct platform_driver etn_fan_driver = {
	.probe = etn_fan_probe,
	.remove = etn_fan_remove,
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(etn_of_match),
		   },
};

static int __init etn_fan_init(void)
{
	if (platform_driver_register(&etn_fan_driver)) {
		pr_err("Failed to probe ETN platform driver\n");
		return -ENXIO;
	}
	return 0;
}

static void __exit etn_fan_exit(void)
{
	platform_driver_unregister(&etn_fan_driver);
}

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("ETN fan driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(etn_fan_init);
module_exit(etn_fan_exit);
