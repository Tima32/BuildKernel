#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#define CD_PROP_NAME "charge-detect-voltage-microvolt"
#define ETM_BAT_POLL_PERIOD (1000)

struct etm_battery {
	struct power_supply *psy;
	struct iio_channel *voltage_channel;
	struct delayed_work work;
	struct power_supply_battery_info *info;
	int status;
	int charge_detect_voltage_uv;
};

static enum power_supply_property etm_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
};

static struct etm_battery* to_etm_bat(struct power_supply *psy)
{
	return power_supply_get_drvdata(psy);
}

static int etm_bat_get_mvolts(struct etm_battery *bat, int *result)
{
	return iio_read_channel_processed(bat->voltage_channel, result);
}

static int etm_bat_calculate_charge(struct etm_battery *bat, long mv_now)
{
	u64 divident, divider;
	u64 uv_max = bat->info->constant_charge_voltage_max_uv;
	u64 uv_min = bat->info->voltage_min_design_uv;
	u64 uah_max = bat->info->charge_full_design_uah;

	/*Check if voltage is more then possible (charger is connected). */
	if ((mv_now * 1000) > uv_max)
		return uah_max;
	
	/* Check if battery voltage is critically low. */
	if ((mv_now * 1000) < uv_min) {
		dev_warn(&bat->psy->dev,
			"battery voltage is critical: %ld mV!\n", mv_now);
		return 0;
	}

	/* Calculate charge. */
	divident = ((mv_now * 1000) - uv_min) * (uah_max);
	divider = (uv_max - uv_min);

	/* Have to use div64_u64() to divide 64-bit integers in Kernel */
	return (int)div64_u64(divident, divider);
}

static void etm_bat_power_changed(struct etm_battery *bat)
{
	int ret = 0, status;
	int mv_now;

	status = bat->status;
	//status here
	ret = etm_bat_get_mvolts(bat, &mv_now);
	if (ret)
		bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
	else if ((mv_now * 1000) > bat->charge_detect_voltage_uv)
		bat->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		bat->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (status != bat->status)
		power_supply_changed(bat->psy);	
}

static void etm_bat_work(struct work_struct *w)
{
	struct delayed_work *dwork = to_delayed_work(w);
	struct etm_battery *bat = container_of(dwork, struct etm_battery, work);

	etm_bat_power_changed(bat);
	schedule_delayed_work(&bat->work,
				msecs_to_jiffies(ETM_BAT_POLL_PERIOD));
}

static int etm_bat_get_property(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *v)
{
	struct etm_battery *bat = to_etm_bat(psy);
	int mvolt_now = 0;
	int ret = 0;

	switch(prop) {
	case POWER_SUPPLY_PROP_STATUS:
		v->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		v->intval = bat->info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = etm_bat_get_mvolts(bat, &mvolt_now);
		if (ret)
			goto err;
		v->intval = etm_bat_calculate_charge(bat, mvolt_now);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = etm_bat_get_mvolts(bat, &mvolt_now);
		if (ret)
			goto err;
		v->intval = mvolt_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		v->intval = bat->info->constant_charge_voltage_max_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		v->intval = bat->info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = etm_bat_get_mvolts(bat, &mvolt_now);
		if (ret)
			goto err;
		v->intval = (etm_bat_calculate_charge(bat, mvolt_now) * 100) /
				bat->info->charge_full_design_uah;
		break;
	default:
		return -EINVAL;
	}

err:
	return ret;
}

static int etm_battery_probe(struct platform_device *pdev)
{
	struct etm_battery *bat;
	struct power_supply_desc *psyd;
	struct power_supply_config psyc;
	struct property *charger_mv_property;
	int ret = 0;

	bat = devm_kzalloc(&pdev->dev, sizeof(struct etm_battery), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->info = devm_kzalloc(&pdev->dev,
			sizeof( struct power_supply_battery_info), GFP_KERNEL);
	if (!bat->info)
		return -ENOMEM;

	platform_set_drvdata(pdev, bat);

	charger_mv_property = of_find_property(pdev->dev.of_node, CD_PROP_NAME, NULL);
	if (!charger_mv_property) {
		dev_err(&pdev->dev, "\"%s\" property not found!\n",
								CD_PROP_NAME);
		return -ENODEV;
	}

	bat->charge_detect_voltage_uv =
				be32_to_cpup(charger_mv_property->value);

	bat->voltage_channel = devm_iio_channel_get(&pdev->dev, "voltage");
	if(IS_ERR(bat->voltage_channel)) {
		dev_err(&pdev->dev, "Unable to get voltage channel!");
		return -ENODEV;
	}

	psyc.drv_data = bat;
	psyc.of_node = pdev->dev.of_node;

	bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
	psyd = devm_kzalloc(&pdev->dev,
			sizeof(struct power_supply_desc), GFP_KERNEL);
	if (!psyd) {
		dev_err(&pdev->dev, "Unable to allocate PSY description\n");
		return -ENOMEM;
	}

	psyd->name = pdev->dev.of_node->name;
	psyd->type = POWER_SUPPLY_TYPE_BATTERY;
	psyd->get_property = etm_bat_get_property;
	psyd->properties = etm_bat_props;
	psyd->num_properties = ARRAY_SIZE(etm_bat_props);

	bat->psy = devm_power_supply_register(&pdev->dev, psyd, &psyc);
	if (IS_ERR(bat->psy)) {
		ret = PTR_ERR(bat->psy);
		dev_err(&pdev->dev, "Unable to register power supply: %d\n",
									ret);
		goto err_psy_reg;
	}

	ret = power_supply_get_battery_info(bat->psy, bat->info);
	if (ret) {
		dev_err(&pdev->dev, "Unable to get simple battery info: %d\n",
									ret);
		goto err_psy_reg;
	}

	INIT_DELAYED_WORK(&bat->work, etm_bat_work);
	schedule_delayed_work(&bat->work,
					msecs_to_jiffies(ETM_BAT_POLL_PERIOD));

	return 0;

err_psy_reg:
	return ret;
}

static int etm_battery_remove(struct platform_device *pdev)
{	
	struct etm_battery *bat;
	bat = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&bat->work);
	return 0;
}

static const struct of_device_id etm_battery_match[] = {
	{.compatible = "stcmtk,etm-battery"},
	{},
};
MODULE_DEVICE_TABLE(of, etm_battery_match);

static struct platform_driver etm_battery_driver = {
	.probe = etm_battery_probe,
	.remove = etm_battery_remove,
	.driver = {
		.name = "etm-battery",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(etm_battery_match),
	},
};

module_platform_driver(etm_battery_driver);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("Fuel gauge for ETM battery.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:etm-battery");
