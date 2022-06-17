/**
 * GRIF GPIO controller driver.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/version.h>

#include <linux/regmap.h>

#include <linux/platform_device.h>
#include <common_fpga/fpgafeat.h>

#include "main.h"

/* TODO: remove hardcoded registers offsets */

#define SFP_CTL_FETURE_VER_SR (0)

/* TODO: get GPIO num from feature SR */
#define GRIF_GPIO_GPIOS_NUM (4)

enum sfp_ctl_gpio_cr_regs {
	GPIO_NUM_CR = 0,
	DIR_CR,
	OUTPUT_DATA_CR,
	SET_CR
};

#define DIR_CR_BIT (0)
#define OUTPUT_DATA_CR_BIT (0)
#define SET_CR_STB (0)

enum sfp_ctl_gpio_sr_regs {
	VER_SR,
	INPUT_DATA_SR,
	GPIO_AMOUNT_SR
};

#define INPUT_DATA_SR_BIT (0)
#define INPUT_DATA_SR_DIR_BIT (1)

static int grif_gpio_apply_changes(struct grif_gpio *priv)
{
	int addr;
	int ret;

	/* Apply changes */
	addr = priv->base_reg.cr + SET_CR;
	ret = regmap_write(priv->regmap, addr, 0);
	if (ret)
		return ret;

	return regmap_write(priv->regmap, addr, BIT(SET_CR_STB));
}

static int grif_gpio_get_gpio_amount(struct grif_gpio* priv)
{
	unsigned reg;

	int addr;
	int ret;

	mutex_lock(&priv->fpga_lock);

	addr = priv->base_reg.sr + GPIO_AMOUNT_SR;

	ret = regmap_read(priv->regmap, addr, &reg);

	if (ret)
		goto exit;

	// Now ret is amount of GPIOs
	ret = reg;

exit:
	mutex_unlock(&priv->fpga_lock);

	return ret;
}

static int grif_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct grif_gpio *priv = gpiochip_get_data(chip);
	unsigned reg;

	int addr;
	int ret;

	if (offset >= priv->chip.ngpio)
		return -ENODEV;

	mutex_lock(&priv->fpga_lock);

	/* Select the GPIO in FPGA */
	addr = priv->base_reg.cr + GPIO_NUM_CR;
	ret = regmap_write(priv->regmap, addr, offset);
	if (ret)
		goto exit;

	addr = priv->base_reg.cr + DIR_CR;
	ret = regmap_read(priv->regmap, addr, &reg);
	if (ret)
		goto exit;

	addr = priv->base_reg.sr + INPUT_DATA_SR;
	ret = regmap_read(priv->regmap, addr, &reg);

	if (ret == 0)
		ret = reg & BIT(INPUT_DATA_SR_BIT);

exit:
	mutex_unlock(&priv->fpga_lock);
	return ret;
}

static void grif_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct grif_gpio *priv = gpiochip_get_data(chip);
	int addr;
	int ret;

	if (offset >= priv->chip.ngpio)
		return;

	mutex_lock(&priv->fpga_lock);

	/* Select gpio */
	addr = priv->base_reg.cr + GPIO_NUM_CR;
	ret = regmap_write(priv->regmap, addr, offset);
	if (ret)
		goto exit;

	/* Set direction */
	addr = priv->base_reg.cr + OUTPUT_DATA_CR;
	ret = regmap_write(priv->regmap, addr,
			   (val & 0x1) << OUTPUT_DATA_CR_BIT);
	if (ret)
		goto exit;

	/* Apply changes. An error is no matter in this step */
	grif_gpio_apply_changes(priv);
exit:
	mutex_unlock(&priv->fpga_lock);

}

static int grif_gpio_get_dir(struct gpio_chip *chip, unsigned offset)
{
	struct grif_gpio *priv = gpiochip_get_data(chip);
	unsigned reg;

	int addr;
	int ret;

	if (offset >= priv->chip.ngpio)
		return -ENODEV;

	mutex_lock(&priv->fpga_lock);

	addr = priv->base_reg.cr + GPIO_NUM_CR;
	ret = regmap_write(priv->regmap, addr, offset);
	if (ret)
		goto exit;

	addr = priv->base_reg.sr + INPUT_DATA_SR;
	ret = regmap_read(priv->regmap, addr, &reg);
	if (ret == 0)
		/* 0 is output for the callback, but in FPGA 0 is input */
		ret = !((reg >> INPUT_DATA_SR_DIR_BIT) & 0x1);

exit:
	mutex_unlock(&priv->fpga_lock);
	return ret;
}

static int grif_gpio_dir_in(struct gpio_chip *chip, unsigned offset)
{
	struct grif_gpio *priv = gpiochip_get_data(chip);

	int addr;
	int ret;

	if (offset >= priv->chip.ngpio)
		return -ENODEV;

	mutex_lock(&priv->fpga_lock);

	addr = priv->base_reg.cr + GPIO_NUM_CR;
	ret = regmap_write(priv->regmap, addr, offset);
	if (ret)
		goto exit;

	addr = priv->base_reg.cr + DIR_CR;
	ret = regmap_write(priv->regmap, addr, 0);

	if (ret)
		goto exit;

	ret = grif_gpio_apply_changes(priv);

exit:
	mutex_unlock(&priv->fpga_lock);
	return ret;
}

static int grif_gpio_dir_out(struct gpio_chip *chip, unsigned offset, int val)
{
	struct grif_gpio *priv = gpiochip_get_data(chip);

	int addr;
	int ret;

	if (offset >= priv->chip.ngpio)
	  return -ENODEV;

	mutex_lock(&priv->fpga_lock);

	addr = priv->base_reg.cr + GPIO_NUM_CR;
	ret = regmap_write(priv->regmap, addr, offset);
	if (ret)
		goto exit;

	/* Direction and value should be applied together, but
	   value is set before direction to avoid some dangerous configurations
	   that can occur if an error raises during FPGA interconnect. */
	addr = priv->base_reg.cr + OUTPUT_DATA_CR;
	ret = regmap_write(priv->regmap, addr,
		    (val & 0x1) << OUTPUT_DATA_CR_BIT);
	if (ret)
		goto exit;

	/* Set direction */
	addr = priv->base_reg.cr + DIR_CR;
	ret = regmap_write(priv->regmap, addr, BIT(DIR_CR_BIT));
	if (ret)
		goto exit;

	if (ret)
		goto exit;

	ret = grif_gpio_apply_changes(priv);

exit:
	mutex_unlock(&priv->fpga_lock);
	return ret;
}


int init_platform_data(struct grif_gpio *data)
{
	struct device *dev = &data->pdev->dev;
	struct device_node *np = dev->of_node;
	struct stcmtk_common_fpga *fpga;
	struct fpga_feature *gpio_ctl_feat;
	int base_cr;
	int base_sr;

	fpga = stcmtk_get_fpga(np);
	if (IS_ERR_OR_NULL(fpga)) {
		dev_err(dev, "Can't get FPGA device from phandle");
		return -ENODEV;
	}

	gpio_ctl_feat = stcmtk_fpga_get_feature(fpga, FPGA_FEATURE_GPIO_CTL);
	if (!gpio_ctl_feat) {
		dev_err(dev, "Can't get GPIO_CTL feature from the fpga manager\n");
		return -EFAULT;
	}

	base_cr = stcmtk_get_cr_base_on_port(gpio_ctl_feat, 0);
	base_sr = stcmtk_get_sr_base_on_port(gpio_ctl_feat, 0);

	data->fpga = (void *)fpga;
	data->regmap = fpga->regmap;
	data->base_reg.cr = base_cr;
	data->base_reg.sr = base_sr;

	return 0;
}


void put_platform_data(struct grif_gpio *data)
{
	struct stcmtk_common_fpga *fpga = data->fpga;
	if (fpga)
		stcmtk_put_fpga(fpga);
}


/* TODO: add port support (port_mask) and device tree property for it */
static int grif_gpio_probe(struct platform_device *pdev)
{
	struct grif_gpio *priv = NULL;
	int ret = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	ret = init_platform_data(priv);
	if (ret)
		goto fpga_err;

	priv->chip.parent = &pdev->dev;
	priv->chip.owner = THIS_MODULE;
	priv->chip.label = dev_name(&pdev->dev);
	priv->chip.base = -1;
	/* Lock all gpios. We have to read gpios amount later */
	priv->chip.ngpio = 0;
	priv->chip.get = grif_gpio_get;
	priv->chip.set = grif_gpio_set;
	priv->chip.get_direction = grif_gpio_get_dir;
	priv->chip.direction_input = grif_gpio_dir_in;
	priv->chip.direction_output = grif_gpio_dir_out;

	mutex_init(&priv->fpga_lock);

	ret = grif_gpio_get_gpio_amount(priv);
	if (ret <= 0) {
		dev_err(&pdev->dev, "Can't get gpios amount\n");
		if (ret == 0)
			ret = -EINVAL;

		goto fpga_err;
	}

	priv->chip.ngpio = ret;

	platform_set_drvdata(pdev, priv);

	ret = gpiochip_add_data(&priv->chip, priv);
	if (ret)
		goto fpga_err;

	return ret;

fpga_err:
	put_platform_data(priv);
	return ret;
}

static int grif_gpio_remove(struct platform_device *pdev)
{
	struct grif_gpio *priv = pdev->dev.driver_data;

	put_platform_data(priv);

	return 0;
}

static const struct of_device_id grif_of_match[] = {
	{ .compatible = "stcmtk,grif-gpio-expander", },
	{ .compatible = "stcmtk,soc-gpio-expander", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, grif_of_match);

static struct platform_driver grif_gpio_driver = {
	.driver = {
		.name = "grif_gpio",
		.of_match_table = of_match_ptr(grif_of_match),
	},
	.probe = grif_gpio_probe,
	.remove = grif_gpio_remove,
};
module_platform_driver(grif_gpio_driver);

/* TODO: can we remove sfp from here? */
MODULE_SOFTDEP("pre: grif_fpga_mgr post: sfp");

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("GRIF platform driver for FPGA-based GPIO expander");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.2");
