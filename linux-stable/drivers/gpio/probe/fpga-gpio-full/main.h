#ifndef GRIF_GPIO_EXPANDER__H
#define GRIF_GPIO_EXPANDER__H

#include <linux/gpio/driver.h>

struct base_reg {
	int cr;
	int sr;
};

struct grif_gpio {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct mutex fpga_lock;
	struct base_reg base_reg;
	void *fpga;

	struct gpio_chip chip;
};

#endif
