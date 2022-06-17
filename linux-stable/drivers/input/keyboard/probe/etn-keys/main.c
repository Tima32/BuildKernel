#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/input/adp5589.h>
#include <linux/input/matrix_keypad.h>

static struct i2c_client *adp_keys;

#define ADP_KEY(row, col)	((col) + ((row) * 11))

static int etn_keys_dt_read_keymap(struct device *dev,
				   struct adp5589_kpad_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	int i;
	const u32 *dt_keymap;
	unsigned short *keymap;
	int keymap_len;

	dt_keymap = of_get_property(node, "linux,keymap", &keymap_len);
	if (!dt_keymap) {
		dev_err(dev, "missing dt keymap\n");
		return -ENODEV;
	}

	if (keymap_len % sizeof(u32)) {
		dev_err(dev, "malformed keymap (len=%i)\n", keymap_len);
		return -EINVAL;
	}

	keymap_len /= sizeof(u32);

	keymap = devm_kzalloc(dev, ADP5589_KEYMAPSIZE * sizeof(u32),
			      GFP_KERNEL);
	if (!keymap)
		return -ENOMEM;

	for (i = 0; i < keymap_len; i++) {
		u32 val;
		u16 key;
		u8 row, col;

		val = be32_to_cpup(&dt_keymap[i]);

		row = KEY_ROW(val);
		col = KEY_COL(val);
		key = KEY_VAL(val);

		if (row > ADP5589_MAX_ROW_NUM) {
			dev_err(dev, "invalid row number (%i)\n", row);
			return -EINVAL;
		}

		if (col > ADP5589_MAX_COL_NUM) {
			dev_err(dev, "invalid column number (%i)\n", col);
			return -EINVAL;
		}

		pdata->keypad_en_mask |= ADP_ROW(row);
		pdata->keypad_en_mask |= ADP_COL(col);

		keymap[ADP_KEY(row, col)] = key;
	}

	pdata->keymap = keymap;
	pdata->keymapsize = ADP5589_KEYMAPSIZE;

	return 0;
}

static int etn_keys_dt_read_pulls(struct device *dev,
				  struct adp5589_kpad_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	unsigned i;

	pdata->pull_dis_mask = 0;
	pdata->pullup_en_300k = 0;
	pdata->pullup_en_100k = 0;
	pdata->pulldown_en_300k = 0;

	of_property_read_u32(node, "adp5589,pulldown-300k",
			&pdata->pulldown_en_300k);

	of_property_read_u32(node, "adp5589,pullup-300k",
			&pdata->pullup_en_300k);

	of_property_read_u32(node, "adp5589,pullup-100k",
			&pdata->pullup_en_100k);

	of_property_read_u32(node, "adp5589,pull-disable",
			&pdata->pull_dis_mask);

	/* Check for misconfiguration */
	for (i = 1; i != 0; i <<= 1) {
		int s = 0;

		if (pdata->pulldown_en_300k & i)
			s++;
		if (pdata->pullup_en_300k & i)
			s++;
		if (pdata->pullup_en_100k & i)
			s++;
		if (pdata->pull_dis_mask & i)
			s++;

		if (s > 1) {
			dev_err(dev, "misconfigured pull resistors\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int __ms_to_cycle_time(unsigned t)
{
	if (t >= 40)
		return ADP5589_SCAN_CYCLE_40ms;
	else if (t >= 30)
		return ADP5589_SCAN_CYCLE_30ms;
	else if (t >= 20)
		return ADP5589_SCAN_CYCLE_20ms;
	else
		return ADP5589_SCAN_CYCLE_10ms;
}

static int etn_keys_dt_fill(struct device *dev,
			    struct adp5589_kpad_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	int error;
	u32 t;

	error = etn_keys_dt_read_keymap(dev, pdata);
	if (error)
		return error;

	error = etn_keys_dt_read_pulls(dev, pdata);
	if (error)
		return error;

	if (!of_property_read_u32(node, "adp5589,scan-cycle-time-ms", &t))
		pdata->scan_cycle_time = __ms_to_cycle_time(t);

	pdata->repeat = !of_property_read_bool(node, "linux,no-autorepeat");

	return 0;
}


static struct adp5589_kpad_platform_data *
etn_keys_get_dt_data(struct device *dev)
{
	struct adp5589_kpad_platform_data *pdata;
	int err;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	err = etn_keys_dt_fill(dev, pdata);
	if (err) {
		devm_kfree(dev, pdata);
		return ERR_PTR(err);
	}

	dev->platform_data = pdata;

	return pdata;
}

static int etn_keys_probe(struct platform_device *pdev)
{
	int err;
	struct device_node *np;
	struct i2c_adapter *i2c;
	static struct i2c_board_info adp_keys_info = {
		.type = "adp5589-keys",
	};
	u32 i2c_addr;
	int irq;
	struct adp5589_kpad_platform_data *pdata;

	np = of_parse_phandle(pdev->dev.of_node, "i2c-bus", 0);
	if (!np) {
		dev_err(&pdev->dev, "Invalid i2c-bus\n");

		return -ENODEV;
	}

	i2c = of_find_i2c_adapter_by_node(np);
	if (!i2c) {
		dev_warn(&pdev->dev,
			 "Failed to get i2c adapter %s, defer probing\n",
			 np->full_name);

		return -EPROBE_DEFER;
	}

	of_node_put(np);

	err = of_property_read_u32(pdev->dev.of_node, "addr",
				   &i2c_addr);
	if (err || (i2c_addr > USHRT_MAX)) {
		dev_err(&pdev->dev, "Invalid i2c address\n");

		return err ? err : -EINVAL;
	}

	adp_keys_info.addr = i2c_addr;

	irq = of_irq_get(pdev->dev.of_node, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "Invalid IRQ\n");

		return irq ? irq : -EFAULT;
	}

	adp_keys_info.irq = irq;

	pdata = etn_keys_get_dt_data(&pdev->dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	adp_keys_info.platform_data = pdata;

	adp_keys = i2c_new_client_device(i2c, &adp_keys_info);
	if (!adp_keys)
		return -EFAULT;

	return 0;
}

static int etn_keys_remove(struct platform_device *pdev)
{
	i2c_unregister_device(adp_keys);

	return 0;
}

static const struct of_device_id etn_keys_id[] = {
	{ .compatible = "etn,adp5589" },
	{},
};
MODULE_DEVICE_TABLE(of, etn_keys_id);

static struct platform_driver etn_keys_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = etn_keys_id,
	},
	.probe  = etn_keys_probe,
	.remove = etn_keys_remove,
};

module_platform_driver(etn_keys_driver);

MODULE_LICENSE("GPL");

MODULE_SOFTDEP("post: adp5589-keys");
