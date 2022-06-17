/*
 * Battery driver for ETN platform
 * Based on B4.5 battery driver
 *
 * Corrected to bq40z50 for LiON chemistry and ltc4100 charger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>

static unsigned int polling_interval = 3000;
module_param(polling_interval, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(polling_interval, "Battery state polling interval (ms), default 2000");

static unsigned int low_voltage = 9700;
module_param(low_voltage, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(low_voltage, "Low battery voltage (mV), default 9700");

static unsigned int critical_voltage = 9400;
module_param(critical_voltage, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(critical_voltage, "Critical battery voltage (mV), default 9400");

static unsigned int low_capacity = 7;
module_param(low_capacity, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(low_capacity, "Low battery capacity (%), default 7");

static unsigned int critical_capacity = 0;
module_param(critical_capacity, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(critical_capacity, "Critical battery capacity (%), default 0");

enum smartbat_device {
	SMARTBAT_BATTERY,
	SMARTBAT_MANAGER,
};

enum smartbat_battery_command {
	SMARTBAT_BCMD_MODE      = 0x03,   /* BatteryMode */
	SMARTBAT_BCMD_TEMP      = 0x08,   /* Temperature */
	SMARTBAT_BCMD_V         = 0x09,   /* Voltage */
	SMARTBAT_BCMD_I         = 0x0a,   /* Current */
	SMARTBAT_BCMD_AVGI      = 0x0b,   /* AverageCurrent */
	SMARTBAT_BCMD_RELCHG    = 0x0d,   /* RelativeStateOfCharge */
	SMARTBAT_BCMD_ABSCHG    = 0x0e,   /* AbsoluteStateOfCharge */
	SMARTBAT_BCMD_REMCAP    = 0x0f,   /* RemainingCapacity */
	SMARTBAT_BCMD_CHGCAP    = 0x10,   /* FullChargeCapacity */
	SMARTBAT_BCMD_RUNTIME_E = 0x11,   /* RunTimeToEmpty */
	SMARTBAT_BCMD_AVGTIME_E = 0x12,   /* AverageTimeToEmpty */
	SMARTBAT_BCMD_AVGTIME_F = 0x13,   /* AverageTimeToFull */
	SMARTBAT_BCMD_CHGI      = 0x14,   /* ChargingCurrent */
	SMARTBAT_BCMD_CHGV      = 0x15,   /* ChargingVoltage */
	SMARTBAT_BCMD_STATUS    = 0x16,   /* BatteryStatus */
	SMARTBAT_BCMD_CYCLECT   = 0x17,   /* CycleCount */
	SMARTBAT_BCMD_DESCAP    = 0x18,   /* DesignCapacity */
	SMARTBAT_BCMD_DESV      = 0x19,   /* DesignVoltage */
	SMARTBAT_BCMD_SPEC      = 0x1a,   /* SpecificationInfo */
	SMARTBAT_BCMD_DATE      = 0x1b,   /* ManufactureDate */
	SMARTBAT_BCMD_SERIAL    = 0x1c,   /* SerialNumber */
	SMARTBAT_BCMD_MANUF     = 0x20,   /* ManufacturerName */
	SMARTBAT_BCMD_NAME      = 0x21,   /* DeviceName */
	SMARTBAT_BCMD_CHEM      = 0x22,   /* DeviceChemistry */
};

//rem by jury093
enum smartbat_manager_command {
	SMARTBAT_MCMD_STATE     = 0x11,   /* BatterySystemState */
	SMARTBAT_MCMD_STATE_C   = 0x13,   /* BatterySystemStateCont */
	SMARTBAT_MCMD_INFO      = 0x3c,   /* BatterySystemInfo */
};

enum smartbat_mode {
	SMARTBAT_CAPACITY_MODE             = 0x8000,
};

enum smartbat_alarm {
	SMARTBAT_REMAINING_TIME_ALARM      = 0x0100,
	SMARTBAT_REMAINING_CAPACITY_ALARM  = 0x0200,
	SMARTBAT_TERMINATE_DISCHARGE_ALARM = 0x0800,
	SMARTBAT_OVER_TEMP_ALARM           = 0x1000,
	SMARTBAT_TERMINATE_CHARGE_ALARM    = 0x4000,
	SMARTBAT_OVER_CHARGED_ALARM        = 0x8000,
};

enum smartbat_status {
	SMARTBAT_FULLY_DISCHARGED          = 0x0010,
	SMARTBAT_FULLY_CHARGED             = 0x0020,
	SMARTBAT_DISCHARGING               = 0x0040,
	SMARTBAT_INITIALIZED               = 0x0080,
};

enum smartbat_error {
	SMARTBAT_OK                        = 0x0000,
	SMARTBAT_BUSY                      = 0x0001,
	SMARTBAT_RESERVED_COMMAND          = 0x0002,
	SMARTBAT_UNSUPPORTED_COMMAND       = 0x0003,
	SMARTBAT_ACCESS_DENIED             = 0x0004,
	SMARTBAT_OVER_UNDERFLOW            = 0x0005,
	SMARTBAT_BAD_SIZE                  = 0x0006,
	SMARTBAT_UNKNOWN_ERROR             = 0x0007,
};

static enum power_supply_property etnbat_charge_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CALIBRATE,
};

static enum power_supply_property etnbat_energy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CALIBRATE,
};

static enum power_supply_property etnbat_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct etnbat {
	struct power_supply *battery;
	struct power_supply_desc bat_desc;
	struct power_supply *manager;
	struct power_supply_desc mgr_desc;

	struct i2c_client *bclient;
	struct i2c_client *mclient;

	struct workqueue_struct *wq;
	struct delayed_work dwork;

	struct input_dev *idev;

	/* battery info (static data) */
	s32 mode;
	s32 spec;
	s32 design_voltage;
	s32 design_capacity;
	int technology;
	char model_name[I2C_SMBUS_BLOCK_MAX];
	char manufacturer[I2C_SMBUS_BLOCK_MAX];
	char serial_number[6];

	/* battery state (dynamic data) */
	s32 ac_online;
	int status;
	s32 cycle_count;
	s32 voltage_now;
	s32 current_now;
	s32 current_avg;
	s32 capacity;
	int capacity_level;
	s32 charge_full;
	s32 charge_now;
	s32 temp;
	s32 time_to_empty_now;
	s32 time_to_empty_avg;
	s32 time_to_full_avg;
	s32 calibrate;

	struct mutex state_mutex;
};

static struct etnbat *etnbat;

static inline int etnbat_scale(int log)
{
	int scale = 1;

	while (log--)
		scale *= 10;

	return scale;
}

static inline int etnbat_vscale(struct etnbat *bat)
{
	return etnbat_scale((bat->spec & 0x0f00) >> 8);
}

static inline int etnbat_ipscale(struct etnbat *bat)
{
	return etnbat_scale((bat->spec & 0xf000) >> 12);
}

static inline int etnbat_capscale(struct etnbat *bat)
{
	return ((bat->mode & SMARTBAT_CAPACITY_MODE) ? 10 : 1) * etnbat_ipscale(bat);
}

static void etnbat_queue_work(struct etnbat *bat, int nodelay)
{
	unsigned long delay;

	if (nodelay) {
		delay = 0;
		cancel_delayed_work_sync(&bat->dwork);
	} else {
		delay = msecs_to_jiffies(polling_interval);
		if (delay >= HZ)
			delay = round_jiffies_relative(delay);
	}

	queue_delayed_work(bat->wq, &bat->dwork, delay);
}
//my modify jury
static s32 etnbat_read_word(struct i2c_client *client, u8 reg)
{
	s32 val;
	int retries = 3; // change from "1" by jury

	while (retries > 0) {
	    val = i2c_smbus_read_word_data(client, reg);
		if (val >= 0)
			break;
		retries--;
	}

	if (val < 0) {
		dev_err(&client->dev, "%s failed, reg=0x%02x, err=%d\n",
			__func__, reg, val);
	}
	
	return val;
}

static s32 etnbat_read_block(struct i2c_client *client, u8 reg, u8 *buf)
{
	u8 block[I2C_SMBUS_BLOCK_MAX];
	s32 len;

	len = i2c_smbus_read_i2c_block_data(client, reg,
					    I2C_SMBUS_BLOCK_MAX, block);
	if (len < 0) {
		dev_err(&client->dev, "%s failed, reg=0x%02x, err=%d\n",
			__func__, reg, len);
	} else {
		len = block[0];
		if (len > I2C_SMBUS_BLOCK_MAX - 1)
			len = I2C_SMBUS_BLOCK_MAX - 1;
		memcpy(buf, &block[1], len);
	}

	return len;
}

static s32 etnbat_read_string(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 len;

	if ((len = etnbat_read_block(client, reg, buf)) >= 0)
		buf[len] = '\0';

	return len;
}

static void etnbat_read_battery_info(struct etnbat *bat)
{
	struct i2c_client *bclient = bat->bclient;
	u8 buf[I2C_SMBUS_BLOCK_MAX];
	s32 n;

	bat->mode = etnbat_read_word(bclient, SMARTBAT_BCMD_MODE);
	bat->spec = etnbat_read_word(bclient, SMARTBAT_BCMD_SPEC);
	bat->design_voltage = etnbat_read_word(bclient, SMARTBAT_BCMD_DESV);
	bat->design_capacity = etnbat_read_word(bclient, SMARTBAT_BCMD_DESCAP);

	if (etnbat_read_string(bclient, SMARTBAT_BCMD_CHEM, buf) >= 0) {
		if (!strcasecmp("NiCd", buf))
			bat->technology = POWER_SUPPLY_TECHNOLOGY_NiCd;
		else if (!strcasecmp("NiMH", buf))
			bat->technology = POWER_SUPPLY_TECHNOLOGY_NiMH;
		else if (!strcasecmp("LION", buf))
			bat->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		else
			bat->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	} else
		bat->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

	if (etnbat_read_string(bclient, SMARTBAT_BCMD_NAME, buf) >= 0)
		sprintf(bat->model_name, "%s", buf);

	if (etnbat_read_string(bclient, SMARTBAT_BCMD_MANUF, buf) >= 0)
		sprintf(bat->manufacturer, "%s", buf);

	if ((n = etnbat_read_word(bclient, SMARTBAT_BCMD_SERIAL)) >= 0)
		sprintf(bat->serial_number, "%d", (u16) n);
}

static void etnbat_monitor_work(struct work_struct *work)
{
	struct etnbat *bat = container_of(work, struct etnbat, dwork.work);
	struct i2c_client *bclient = bat->bclient;
	struct i2c_client *mclient = bat->mclient;
	s32 ac_online, status, cycle_count, voltage_now;
	s32 current_now, current_avg, capacity, charge_full, charge_now;
	s32 temp, time_to_empty_now, time_to_empty_avg, time_to_full_avg, calibrate;
	int capacity_level, need_report_key = 0;

	ac_online = etnbat_read_word(mclient, SMARTBAT_MCMD_STATE_C);
	status = etnbat_read_word(bclient, SMARTBAT_BCMD_STATUS);
	cycle_count = etnbat_read_word(bclient, SMARTBAT_BCMD_CYCLECT);
	voltage_now = etnbat_read_word(bclient, SMARTBAT_BCMD_V);
	current_now = etnbat_read_word(bclient, SMARTBAT_BCMD_I);
	current_avg = etnbat_read_word(bclient, SMARTBAT_BCMD_AVGI);
	capacity = etnbat_read_word(bclient, SMARTBAT_BCMD_RELCHG);
	charge_full = etnbat_read_word(bclient, SMARTBAT_BCMD_CHGCAP);
	charge_now = etnbat_read_word(bclient, SMARTBAT_BCMD_REMCAP);
	temp = etnbat_read_word(bclient, SMARTBAT_BCMD_TEMP);
	time_to_empty_now = etnbat_read_word(bclient, SMARTBAT_BCMD_RUNTIME_E);
	time_to_empty_avg = etnbat_read_word(bclient, SMARTBAT_BCMD_AVGTIME_E);
	time_to_full_avg = etnbat_read_word(bclient, SMARTBAT_BCMD_AVGTIME_F);

	calibrate = etnbat_read_word(bclient, SMARTBAT_BCMD_MODE);

	if (voltage_now >= 0 && capacity >= 0) {
		if (voltage_now <= critical_voltage || capacity <= critical_capacity)
			capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else if (voltage_now <= low_voltage || capacity <= low_capacity)
			capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (capacity > low_capacity && capacity < 100)
			capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (capacity == 100)
			capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else
			capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	} else
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	mutex_lock(&etnbat->state_mutex);

	if (ac_online != bat->ac_online || capacity_level != bat->capacity_level)
		need_report_key = 1;

	if (status >= 0) {
		if (!(status & SMARTBAT_INITIALIZED))
			bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
		else if (status & SMARTBAT_FULLY_CHARGED)
			bat->status = POWER_SUPPLY_STATUS_FULL;
		else if (status & SMARTBAT_DISCHARGING)
			bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			bat->status = POWER_SUPPLY_STATUS_CHARGING;
	} else
		bat->status = POWER_SUPPLY_STATUS_UNKNOWN;

	bat->ac_online = ac_online;
	bat->cycle_count = cycle_count;
	bat->voltage_now = voltage_now;
	bat->current_now = current_now;
	bat->current_avg = current_avg;
	bat->capacity = capacity;
	bat->capacity_level = capacity_level;
	bat->charge_full = charge_full;
	bat->charge_now = charge_now;
	bat->temp = temp;
	bat->time_to_empty_now = time_to_empty_now;
	bat->time_to_empty_avg = time_to_empty_avg;
	bat->time_to_full_avg = time_to_full_avg;
	bat->calibrate = calibrate;

	mutex_unlock(&etnbat->state_mutex);

	if (need_report_key) {
		input_report_key(bat->idev, KEY_BATTERY, 1);
		input_report_key(bat->idev, KEY_BATTERY, 0);
		input_sync(bat->idev);
	}

	etnbat_queue_work(bat, 0);
}

static int etnbat_get_manager_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct etnbat *bat = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&etnbat->state_mutex);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat->ac_online < 0)
			ret = -ENODATA;
		else
		    if(bat->ac_online & 0x8000)
			val->intval = 1;
		    else
			val->intval = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&etnbat->state_mutex);

	return ret;
}

static int etnbat_get_battery_info(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct etnbat *bat = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (bat->design_voltage < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * bat->design_voltage * etnbat_vscale(bat);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (bat->design_capacity < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * bat->design_capacity * etnbat_capscale(bat);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bat->technology;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bat->model_name;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = bat->manufacturer;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = bat->serial_number;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int etnbat_get_battery_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct etnbat *bat = power_supply_get_drvdata(psy);
	int ret = 0;

	if (psp == POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN
	    || psp == POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN
	    || psp == POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN
	    || psp == POWER_SUPPLY_PROP_TECHNOLOGY
	    || psp == POWER_SUPPLY_PROP_MODEL_NAME
	    || psp == POWER_SUPPLY_PROP_MANUFACTURER
	    || psp == POWER_SUPPLY_PROP_SERIAL_NUMBER)
	{
		return etnbat_get_battery_info(psy, psp, val);
	}

	mutex_lock(&etnbat->state_mutex);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (bat->cycle_count < 0)
			ret = -ENODATA;
		else
			val->intval = bat->cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (bat->voltage_now < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * bat->voltage_now * etnbat_vscale(bat);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (bat->current_now < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * (s16) bat->current_now * etnbat_ipscale(bat);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (bat->current_avg < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * (s16) bat->current_avg * etnbat_ipscale(bat);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (bat->capacity < 0)
			ret = -ENODATA;
		else
			val->intval = bat->capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = bat->capacity_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (bat->charge_full < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * bat->charge_full * etnbat_capscale(bat);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (bat->charge_now < 0)
			ret = -ENODATA;
		else
			val->intval = 1000 * bat->charge_now * etnbat_capscale(bat);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (bat->temp < 0)
			ret = -ENODATA;
		else
			val->intval = bat->temp - 2731;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		if (bat->time_to_empty_now < 0 || bat->time_to_empty_now == 65535)
			ret = -ENODATA;
		else
			val->intval = 60 * bat->time_to_empty_now;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		if (bat->time_to_empty_avg < 0 || bat->time_to_empty_avg == 65535)
			ret = -ENODATA;
		else
			val->intval = 60 * bat->time_to_empty_avg;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		if (bat->time_to_full_avg < 0 || bat->time_to_full_avg == 65535)
			ret = -ENODATA;
		else
			val->intval = 60 * bat->time_to_full_avg;
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
		if(bat->calibrate & 0x0080)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&etnbat->state_mutex);

	return ret;
}

static int etnbat_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct etnbat *bat = etnbat;
	struct device *dev = &client->dev;
	struct i2c_adapter *adapter = client->adapter;
	struct power_supply *psy;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {};

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(dev, "SMBus word r/w transactions not supported\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		dev_err(dev, "I2C block read transactions not supported\n");
		return -ENODEV;
	}

	if (id->driver_data == SMARTBAT_BATTERY) {
		bat->bclient = client;
		etnbat_read_battery_info(bat);
		if (bat->mode < 0 || bat->spec < 0) {
			dev_err(dev, "Can't determine battery operating mode\n");
			return -ENODEV;
		}
		psy_desc = &bat->bat_desc;
		psy_desc->name = "BAT0";
		psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
		if (bat->mode & SMARTBAT_CAPACITY_MODE) {
			psy_desc->properties = etnbat_energy_props;
			psy_desc->num_properties = ARRAY_SIZE(etnbat_energy_props);
		} else {
			psy_desc->properties = etnbat_charge_props;
			psy_desc->num_properties = ARRAY_SIZE(etnbat_charge_props);
		}
		psy_desc->get_property = etnbat_get_battery_property;
	} else {
		bat->mclient = client;
		psy_desc = &bat->mgr_desc;
		psy_desc->name = "AC0";
		psy_desc->type = POWER_SUPPLY_TYPE_MAINS;
		psy_desc->properties = etnbat_ac_props;
		psy_desc->num_properties = ARRAY_SIZE(etnbat_ac_props);
		psy_desc->get_property = etnbat_get_manager_property;
	}

	psy_cfg.drv_data = bat;

	psy = power_supply_register(dev, psy_desc, &psy_cfg);
	if (IS_ERR(psy)) {
		dev_err(dev, "Failed to register device\n");
		return PTR_ERR(psy);
	}

	if (id->driver_data == SMARTBAT_BATTERY)
		bat->battery = psy;
	else
		bat->manager = psy;

	if (bat->bclient && bat->mclient)
		etnbat_queue_work(bat, 1);

	i2c_set_clientdata(client, bat);

	return 0;
}

static int etnbat_remove(struct i2c_client *client)
{
	struct etnbat *bat = i2c_get_clientdata(client);
	struct power_supply *psy;

	cancel_delayed_work_sync(&bat->dwork);

	if (client == bat->bclient) {
		bat->bclient = NULL;
		psy = bat->battery;
	} else {
		bat->mclient = NULL;
		psy = bat->manager;
	}

	power_supply_unregister(psy);

	return 0;
}

static void etnbat_alert(struct i2c_client *client,
			 enum i2c_alert_protocol protocol, unsigned int data)
{
	struct etnbat *bat = i2c_get_clientdata(client);

	if (bat->bclient && bat->mclient)
		etnbat_queue_work(bat, 1);
}

static const struct i2c_device_id etnbat_ids[] = {
	{ "bq40z50", SMARTBAT_BATTERY },
	{ "ltc4100", SMARTBAT_MANAGER },
	{ }
};
MODULE_DEVICE_TABLE(i2c, etnbat_ids);

MODULE_ALIAS("i2c:etn-battery");

static struct i2c_driver etnbat_driver = {
	.driver = {
		.name  = "etn-bat",
		.owner = THIS_MODULE,
	},

	.probe    = etnbat_probe,
	.remove   = etnbat_remove,
	.alert    = etnbat_alert,
	.id_table = etnbat_ids,
};

static int __init etnbat_init(void)
{
	int err;

	etnbat = kzalloc(sizeof(*etnbat), GFP_KERNEL);
	if (!etnbat) {
		err = -ENOMEM;
		goto init_err_out1;
	}

	etnbat->status = POWER_SUPPLY_STATUS_UNKNOWN;
	etnbat->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	mutex_init(&etnbat->state_mutex);

	INIT_DELAYED_WORK(&etnbat->dwork, etnbat_monitor_work);

	etnbat->wq = create_singlethread_workqueue("etnbat_wq");
	if (!etnbat->wq) {
		err = -ENOMEM;
		goto init_err_out2;
	}

	etnbat->idev = input_allocate_device();
	if (!etnbat->idev) {
		err = -ENOMEM;
		goto init_err_out3;
	}

	set_bit(EV_KEY, etnbat->idev->evbit);
	set_bit(KEY_BATTERY, etnbat->idev->keybit);

	etnbat->idev->name = "etn-bat";

	err = input_register_device(etnbat->idev);
	if (err) {
		input_free_device(etnbat->idev);
		goto init_err_out3;
	}

	err = i2c_add_driver(&etnbat_driver);
	if (err)
		goto init_err_out4;

	return 0;

init_err_out4:
	input_unregister_device(etnbat->idev);
init_err_out3:
	destroy_workqueue(etnbat->wq);
init_err_out2:
	kfree(etnbat);
init_err_out1:
	return err;
}

static void __exit etnbat_exit(void)
{
	i2c_del_driver(&etnbat_driver);
	input_unregister_device(etnbat->idev);
	destroy_workqueue(etnbat->wq);
	kfree(etnbat);
}

MODULE_DESCRIPTION("Battery driver for ETN platform");
MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_LICENSE("GPL");

module_init(etnbat_init);
module_exit(etnbat_exit);
