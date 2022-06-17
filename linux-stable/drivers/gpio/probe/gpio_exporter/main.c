#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

#define DIRECTION_MAY_CHANGE true

struct gpio_exporter {
	struct gpio_descs *gpios;
	const char **names;
};

static int gpio_exporter_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *devp = &pdev->dev;
	struct gpio_exporter *ex;
	int gpio_names_count, gpio_dirs_count, i = 0, err, dirconf = 0;

	ex = devm_kzalloc(devp, sizeof(struct gpio_exporter), GFP_KERNEL);
	if (!ex)
		return -ENOMEM;

	gpio_names_count = of_property_count_strings(np, "exported-gpio-names");

	gpio_dirs_count = of_property_count_strings(np,
						"exported-gpio-directions");

	ex->gpios = devm_gpiod_get_array(devp, "exported", GPIOD_ASIS);
	if (IS_ERR(ex->gpios)) {
		err = PTR_ERR(ex->gpios);
		if (err == -EPROBE_DEFER)
			dev_warn(devp, "defer probing...\n");
		else
			dev_err(devp, "error while probe. Reason: %d!\n", err);
		return err;
	}

	if (ex->gpios->ndescs != gpio_names_count) {
		dev_err(devp,
			"the number of gpios is not equal the number of gpio names!\n");
		return -EINVAL;
	}

	if (ex->gpios->ndescs == gpio_dirs_count) {
		dirconf = 1;
	}
	else if (gpio_dirs_count == -EINVAL) {
		dev_info(devp, "GPIO directions settings is not presented. Using default directions.");
		dirconf = 0;
	}
	else {
		/* Error handling */
		if (gpio_dirs_count >= 0) {
			dev_err(devp, "the number of gpios is not equal the number of gpio directions settings!");
			return -EINVAL;
		}
		else {
			dev_err(devp, "Can't read gpio direction settings. Reason: %d\n",
							gpio_dirs_count);
			return gpio_dirs_count;
		}
	}

	ex->names = devm_kzalloc(devp, sizeof(char *) * gpio_names_count,
				 GFP_KERNEL);
	if (!ex->names)
		return -ENOMEM;

	platform_set_drvdata(pdev, ex);

	for (i = 0; i < ex->gpios->ndescs; ++i) {
		err = of_property_read_string_index(np, "exported-gpio-names",
						    i, &ex->names[i]);
		if (err) {
			dev_err(devp, "Could not get gpio name %d\n", i);
			return err;
		}

		if (dirconf) {
			const char *dir;
			err = of_property_read_string_index(np,
					"exported-gpio-directions", i, &dir);

			if (err) {
				dev_err(devp, "Could not get gpio direction string %d\n", i);
				return err;
			}

			if (strcmp(dir, "out") == 0 ||
				strcmp(dir, "low") == 0) {
				err = gpiod_direction_output(ex->gpios->desc[i], 0);
			}
			else if (strcmp(dir, "high") == 0) {
				err = gpiod_direction_output(ex->gpios->desc[i], 1);
			}
			else if (strcmp(dir, "in") == 0) {
				err = gpiod_direction_input(ex->gpios->desc[i]);
			}
			else {
				dev_err(devp, "Unknown direction setting \"%s\" for \"%s\"\n",
							dir, ex->names[i]);
				err = -EINVAL;
			}

			if (err) {
				dev_err(devp, "Can't set direction \"%s\" for \"%s\"\n",
							dir, ex->names[i]);
				return err;
			}
		}

		err = gpiod_export(ex->gpios->desc[i], DIRECTION_MAY_CHANGE);
		if (err)
			return err;

		err = gpiod_export_link(devp, ex->names[i], ex->gpios->desc[i]);
		if (err)
			return err;
	}
	return 0;
}

static int gpio_exporter_remove(struct platform_device *pdev)
{
	int i;
	struct gpio_exporter *ex = platform_get_drvdata(pdev);

	for (i = 0; i < ex->gpios->ndescs  ; ++i) {
		sysfs_remove_link(&pdev->dev.kobj, ex->names[i]);
		gpiod_unexport(ex->gpios->desc[i]);
	}

	return 0;
}

static const struct of_device_id gpio_exporter_match[] = {
	{.compatible = "linux,gpio-exporter",},
	{},
};
MODULE_DEVICE_TABLE(of, gpio_exporter_match);

static struct platform_driver gpio_exporter_driver = {
	.remove = gpio_exporter_remove,
	.probe = gpio_exporter_probe,
	.driver = {
		.name = "gpio_exporter",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_exporter_match),
	},
};

module_platform_driver(gpio_exporter_driver);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("The module exports GPIO from device tree to /sys/.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio_exporter");

