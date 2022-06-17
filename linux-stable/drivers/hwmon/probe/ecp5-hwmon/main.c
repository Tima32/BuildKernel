#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <grif/grif_fpga_mgr/grif_fpga.h>
#include <common_fpga/fpgafeat.h>

#define ECP5_TEMP_MON_MAIN_CR     0
#define ECP5_TEMP_MON_MAIN_CR_STB 0

#define ECP5_TEMP_MON_TEMP_SR     1
#define ECP5_TEMP_MON_TEMP_MASK   GENMASK(5, 0)

struct ecp5_hwmon {
	struct device *hwmon_dev;
	struct platform_device *pdev;
	struct regmap *regmap;
	struct fpga_feature *hwmon_feat;
};

static const char *default_name = "ecp5_temperature";

/* An debugfs entry and it's name for the module. */
static struct dentry *d;

static const char *dname = "ecp5_temperature_values";

/*
 * ECP5 Sensor temperature table. See "Power Consumption and Management for ECP5
 * and ECP5-5G Devices", Table 4
 */
#define TEMP_TABLE_VALS_COUNT 64
static const int temp_table[] = {
	-58, -56, -54, -52, -45, -44, -43, -42,
	-41, -40, -39, -38, -37, -36, -30, -20,
	-10,  -4,   0,   4,  10,  21,  22,  23,
	 24,  25,  26,  27,  28,  29,  40,  50,
	 60,  70,  76,  80,  82,  82,  83,  84,
	 85,  86,  87,  88,  89,  95,  96,  97,
	 98,  99, 100, 101, 102, 103, 104, 105,
	106, 107, 108, 116, 120, 124, 128, 132
};

static ssize_t ecp5_temp1_label_show(struct device *dev,
				struct device_attribute *dev_attr, char *buffer)
{
	const char *label = of_get_property(dev->of_node, "label", NULL);

	if (!label)
		label = default_name;

	return snprintf(buffer, PAGE_SIZE, "%s\n", label);
}

static ssize_t ecp5_temp1_value_show(struct device *dev,
				struct device_attribute *dev_attr, char *buffer)
{
	int err;
	struct ecp5_hwmon *data = dev_get_drvdata(dev);
	struct fpga_feature *f = data->hwmon_feat;
	unsigned raw_data = 0;

	/* Perform a strobe */
	err = regmap_write(data->regmap, f->cr_base + ECP5_TEMP_MON_MAIN_CR,
						BIT(ECP5_TEMP_MON_MAIN_CR_STB));
	if (err) {
		dev_err(dev, "Strobe set error %d\n", err);
		goto err;
	}

	err = regmap_write(data->regmap, f->cr_base + ECP5_TEMP_MON_MAIN_CR, 0);
	if (err) {
		dev_err(dev, "Strobe clear error %d\n", err);
		goto err;
	}

	msleep(1);

	/* Read raw data */
	err = regmap_read(data->regmap,
				f->sr_base + ECP5_TEMP_MON_TEMP_SR, &raw_data);
	if (err) {
		dev_err(dev, "Read value error %d\n", err);
		goto err;
	}

	return snprintf(buffer, PAGE_SIZE, "%d\n",
			temp_table[raw_data & ECP5_TEMP_MON_TEMP_MASK] * 1000);

err:
	/* Show the critical temperature in case of an regmap I/O error */
	return snprintf(buffer, PAGE_SIZE, "132000\n");
}

static DEVICE_ATTR(temp1_label, S_IRUGO, ecp5_temp1_label_show, NULL);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, ecp5_temp1_value_show, NULL, 1);

static struct attribute *ecp5_attrs[] = {
	&dev_attr_temp1_label.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ecp5);

static int ecp5_hwmon_probe(struct platform_device *pdev)
{
	struct ecp5_hwmon *data;
	struct stcmtk_common_fpga *fpga;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Coundn't allocate hwmon data.\n");
		return -ENOMEM;
	}

	fpga = stcmtk_get_fpga(np);
	if (!fpga || IS_ERR_VALUE(fpga)) {
		dev_err(&pdev->dev, "Couldn't get FPGA manager.");
		return -ENODEV;
	}

	data->regmap = dev_get_regmap(fpga->dev, NULL);
	if (!data->regmap) {
		dev_warn(&pdev->dev, "Couldn't access FPGA regmap. Deferring...");
		stcmtk_put_fpga(fpga);
		return -EPROBE_DEFER;
	}

	data->hwmon_feat = stcmtk_fpga_get_feature(fpga,
						FPGA_FEATURE_ECP5_TEMP_MON);
	if (!data->hwmon_feat) {
		dev_err(&pdev->dev, "Couldn't get ECP5_TEMP_MON feature.");
		stcmtk_put_fpga(fpga);
		return -ENODEV;
	}

	data->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
					default_name, data, ecp5_groups);

	return 0;
}

static const struct of_device_id ecp5_hwmon_of_match[] = {
	{.compatible = "lattice,ecp5-hwmon",},
	{},
};
MODULE_DEVICE_TABLE(of, ecp5_hwmon_of_match);

static struct platform_driver ecp5_hwmon_driver = {
	.driver = {
		.name = "ecp5_hwmon",
		.of_match_table = of_match_ptr(ecp5_hwmon_of_match),
	},
	.probe = ecp5_hwmon_probe,
};

static int ecp5_debug_show_tmp_values(struct seq_file *s, void *data)
{
	int i;

	seq_puts(s, "------------------------------------\n");
	seq_puts(s, "      ECP5 Temperature points\n");
	seq_puts(s, " Point  |  Value    Point  |  Value\n");
	seq_puts(s, "------------------------------------\n");

	for (i = 0; i < TEMP_TABLE_VALS_COUNT / 2; ++i) {
		seq_printf(s,
		    "    %2d  |  %3d	 %2d  |  %3d\n"
			, i, temp_table[i], i + 32, temp_table[i + 32]);
	}

	seq_puts(s, "------------------------------------\n");
	seq_puts(s, " Values are in degrees Celsius.\n");

	return 0;
}

static int ecp5_debug_open(struct inode *i, struct file *f)
{
	return single_open(f, ecp5_debug_show_tmp_values, NULL);
}

static const struct file_operations dentry_fops = {
	.open = ecp5_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init ecp5_hwmon_init(void)
{
	int err;

	d = debugfs_create_file(dname, S_IRUGO, NULL, NULL, &dentry_fops);
	if (!d) {
		pr_warn("ECP5 HWMON: Failed to create the debugfs entry. "
							"Going ahead...");
	}
	else {
		pr_info("ECP5 HWMON: See /sys/kernel/debug/%s file", dname);
		pr_info("ECP5 HWMON: to find out all the available temperature");
		pr_info("ECP5 HWMON: points of the sensor.");
	}

	err = platform_driver_register(&ecp5_hwmon_driver);
	if (err) {
		return err;
	}

	return 0;
}

static void __exit ecp5_hwmon_exit(void)
{
	platform_driver_unregister(&ecp5_hwmon_driver);
	if (d)
		debugfs_remove(d);
	return;
}

module_init(ecp5_hwmon_init);
module_exit(ecp5_hwmon_exit);

MODULE_SOFTDEP("pre: grif_fpga_mgr");

MODULE_AUTHOR("STC Metrotek <system@metrotek.ru>");
MODULE_DESCRIPTION("Lattice ECP5 hardware monitor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.1");
