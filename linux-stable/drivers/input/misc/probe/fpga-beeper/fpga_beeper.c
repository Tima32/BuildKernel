#include <linux/input.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#define BEEPER_OSC_PHASE_W0_CR 3
#define BEEPER_OSC_PHASE_W1_CR 4

struct beeper_data {
	struct stcmtk_common_fpga	*etn_fpga;
	struct fpga_feature		*feat;
};

static int fpga_beeper_event(struct input_dev *input_dev, unsigned int type,
		unsigned int code, int value)
{
	uint64_t phase = 0;
	struct beeper_data *b_data = input_get_drvdata(input_dev);
	struct regmap *regmap = b_data->etn_fpga->regmap;
	struct fpga_feature *fpga_feat = b_data->feat;

	if (type != EV_SND || code != SND_TONE || value < 0)
		return -ENOTSUPP;

	if (!b_data)
		return -EINVAL;

	regmap = b_data->etn_fpga->regmap;
	fpga_feat = b_data->feat;

	if (value > 0) {
		/* phase = 2^32 / CLK_FREQ * freq, where CLK_FREQ=62_500_000
		 * 2^32 / CLK_FREQ = 68.72 = 69*/

		phase = 69 * value;
		regmap_write(regmap, fpga_feat->cr_base + BEEPER_OSC_PHASE_W0_CR,
				phase & 0xffff);
		regmap_write(regmap, fpga_feat->cr_base + BEEPER_OSC_PHASE_W1_CR,
				(phase >> 16) & 0xffff);
		/* Touch the strobe to update beeper phase */
		regmap_write(regmap, fpga_feat->cr_base, 0x0000);
		regmap_write(regmap, fpga_feat->cr_base, 0x8000);

		/* Turn the beeper on */
		regmap_update_bits(regmap, fpga_feat->cr_base, 1, 1);
	}
	else {
		/* Turn the beeper off */
		regmap_update_bits(regmap, fpga_feat->cr_base, 1, 0);
	}

	return 0;
	
}

static int fpga_beeper_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct beeper_data *b_data = NULL;
	struct input_dev *input_dev = NULL;
	
	b_data = devm_kzalloc(&pdev->dev, sizeof(struct beeper_data), GFP_KERNEL);
	if (!b_data) {
		dev_err(&pdev->dev, "Unable to allocate memory for beeper data\n");
		return -ENOMEM;
	}

	b_data->etn_fpga = stcmtk_get_fpga((pdev->dev).of_node);
	if (!b_data->etn_fpga) {
		dev_err(&pdev->dev, "Unable to get etn_fpga\n");
		ret = -ENODEV;
		goto fpga_err;
	}

	ret = etn_fpga_ref_get(b_data->etn_fpga);
	if (ret) {
		dev_err(&pdev->dev, "Unable to increment reference count\n");
		goto ref_err;
	}

	b_data->feat = stcmtk_fpga_get_feature(b_data->etn_fpga, FPGA_FEATURE_BEEPER);
	if (!b_data->feat) {
		dev_err(&pdev->dev, "Unable to get FPGA_FEATURE_BEEPER\n");
		ret = -ENODEV;
		goto feat_err;
	}

	input_dev = devm_input_allocate_device(&pdev->dev);
	if (!input_dev) {
		dev_err(&pdev->dev, "Unable to allocate input device\n");
		ret = -ENOMEM;
		goto input_dev_err;
	}

	input_dev->name = pdev->name;
	input_dev->event = fpga_beeper_event;

	input_set_capability(input_dev, EV_SND, SND_TONE);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register input device\n");
		goto input_dev_err;
	}

	platform_set_drvdata(pdev, b_data);
	input_set_drvdata(input_dev, b_data);

	return 0;

input_dev_err:
feat_err:
	etn_fpga_ref_put(b_data->etn_fpga);
ref_err:
fpga_err:
	return ret;
}

static int fpga_beeper_remove(struct platform_device *pdev)
{
	struct beeper_data *b_data = platform_get_drvdata(pdev);

	return etn_fpga_ref_put(b_data->etn_fpga);
}

static const struct of_device_id fpga_beeper_id[] = {
	{.compatible = "stcmtk,fpga-beeper"},
	{/* sentinel */}
};

MODULE_DEVICE_TABLE(of, fpga_beeper_id);

static struct platform_driver fpga_beeper_platform_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = fpga_beeper_id
	},
	.probe = fpga_beeper_probe,
	.remove = fpga_beeper_remove
};
module_platform_driver(fpga_beeper_platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("Beeper driver for PROBE devices");
