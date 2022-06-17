#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>

#include "nt25l90.h"

#define DRIVER_NAME "nt25l90_driver"
#define OPTICAL_DRIVER_MAX_MINORS 255

static DEFINE_IDA(optical_ida);
static dev_t optical_dev;
static struct class *optical_drv_class;


static bool nt25l90_volatile_reg(struct device *dev, unsigned int reg)
{
    if (reg >= 0xA0 && reg <= 0xB6) {
        return true;
    }
    return false;
}

static bool nt25l90_readable_reg(struct device *dev, unsigned int reg)
{
    if ((reg == 0x16 || reg == 0x3D) ||
        (reg >= 0x60 && reg <= 0x6D) ||
        (reg >= 0xA0 && reg <= 0xB6) ||
        (reg == 0xD0 || reg == 0xD1) ||
        (reg == 0xE5 || reg == 0xEA)) {
        return true;
    }
    return false;
}

static bool nt25l90_writeable_reg(struct device *dev, unsigned int reg)
{
    /* Check this again because 0xB4 (0, 1, 2 bits) and 0xB6(5, 6, 7) are writeable */
    if ((reg == 0x16 || reg == 0x3D) ||
        (reg >= 0x60 && reg <= 0x6D) ||
        (reg >= 0xB0 && reg <= 0xB4) ||
        (reg == 0xB6)                ||
        (reg == 0xD0 || reg == 0xD1) ||
        (reg == 0xE5 || reg == 0xEA)) {
        return true;
    }
    return false;
}

static const struct regmap_config nt25l90_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,

    .readable_reg = nt25l90_readable_reg,
    .writeable_reg = nt25l90_writeable_reg,
    .volatile_reg = nt25l90_volatile_reg,
    .cache_type = REGCACHE_RBTREE,
};

static umode_t nt25l90_is_visible(const void *drvdata,
                                  enum hwmon_sensor_types type,
                                  u32 attr, 
                                  int channel)
{
    switch(type) {
    case hwmon_temp:
    case hwmon_curr:
    case hwmon_in:
    case hwmon_power:
        return 0444;
    default:
        return 0;
    }
}

/* Return temperature in milliDegrees */
static int nt25l90_read_temp(struct regmap *regm, u32 attr, long *val)
{
    unsigned int reg_val;
    unsigned int msb;

    switch(attr) {
    case hwmon_temp_input:
        /* Firstly, we have to read MSB bits from A0h register and save it */
        regmap_read(regm, 0xA0, &reg_val);
        msb = reg_val;
        /* Secondly, we have to read LSB bits from A1h register*/
        regmap_read(regm, 0xA1, &reg_val);
        reg_val = (msb << 8) | reg_val;
        /* Calculate temperature */
        *val = ((reg_val * 1874) / 65536 - 537) * 100;
        return 0;
    
    default:
        return -EOPNOTSUPP;
    }
}
/* Return current in milliAmperes */
static int nt25l90_read_curr(struct regmap *regm, 
                             u32 attr,
                             int channel,
                             long *val)
{
    unsigned int reg_val;
    unsigned int msb = 0;

    switch(attr) {
    case hwmon_curr_input:
        /* Bias current */
        if (channel == 0) {
            /* Read MSB */
            regmap_read(regm, 0xA4, &reg_val);
            msb = reg_val;
            /* Read LSB */
            regmap_read(regm, 0xA5, &reg_val);
        }
        /* Modulation current */
        else if (channel == 1) {
            regmap_read(regm, 0xA6, &reg_val);
            msb = reg_val;
            regmap_read(regm, 0xA7, &reg_val);
        }
        reg_val = (msb << 8) | reg_val;
        /* Calculate current */
        *val = (reg_val * 16 / 512) / 10;
        return 0;
    
    default:
        return -EOPNOTSUPP;
    }
}
/* Return voltage in milliVoltages */
static int nt25l90_read_voltage(struct regmap *regm,
                                u32 attr,
                                int channel,
                                long *val)
{
    unsigned int reg_val;
    unsigned int msb = 0;

    switch(attr) {
    case hwmon_in_input:
        /* Tx Supply Voltage */
        if (channel == 0) {
            regmap_read(regm, 0xAA, &reg_val);
            msb = reg_val;
            regmap_read(regm, 0xAB, &reg_val);
        }
        /* Rx Supply Voltage */
        else if (channel == 1) {
            regmap_read(regm, 0xAC, &reg_val);
            msb = reg_val;
            regmap_read(regm, 0xAD, &reg_val);
        }
        /* Digital Supply Voltage */
        else if (channel == 2) {
            regmap_read(regm, 0xAE, &reg_val);
            msb = reg_val;
            regmap_read(regm, 0xAF, &reg_val);
        }
        reg_val = (msb << 8) | reg_val;
        /* Calculate voltage in milliVoltages */
        *val = (reg_val * 4000) / 65536;
        return 0;
    
    default:
        return -EOPNOTSUPP;
    }
}
/* Return power in nanoAmperes */
static int nt25l90_read_power(struct regmap *regm,
                              u32 attr,
                              int channel,
                              long *val)
{
    unsigned int reg_val;
    unsigned int msb = 0;

    switch(attr) {
    case hwmon_power_input:
        /* Tx Power */
        if (channel == 0) {
            regmap_read(regm, 0xA2, &reg_val);
            msb = reg_val;
            regmap_read(regm, 0xA3, &reg_val);
            reg_val = (msb << 8) | reg_val;

            *val = reg_val * 1125000 / 5120;
        }
        /* Rx Power */
        else if (channel == 1) {
            regmap_read(regm, 0xA8, &reg_val);
            msb = reg_val;
            regmap_read(regm, 0xA9, &reg_val);
            reg_val = (msb << 8) | reg_val;

            *val = reg_val * 16000 / 512;
        }
        
        return 0;
    default:
        return -EOPNOTSUPP;
    }
}

static int nt25l90_read(struct device *dev,
                        enum hwmon_sensor_types type,
                        u32 attr,
                        int channel,
                        long *val)
{
    struct regmap *regm = (struct regmap*) dev_get_drvdata(dev);
    if (!regm) {
        pr_err("regmap is NULL");
        return -EOPNOTSUPP;
    }

    switch(type) {
    case hwmon_temp:
        return nt25l90_read_temp(regm, attr, val);
    
    case hwmon_curr:
        return nt25l90_read_curr(regm, attr, channel, val);
    
    case hwmon_in:
        return nt25l90_read_voltage(regm, attr, channel, val);

    case hwmon_power:
        return nt25l90_read_power(regm, attr, channel, val);

    default:
        return -EOPNOTSUPP;
    }
}

/* TEMPERATURE CONFIG */
static const u32 nt25l90_temp_config[] = {
    HWMON_T_INPUT,
    0
};

static const struct hwmon_channel_info nt25l90_temp = {
    .type = hwmon_temp,
    .config = nt25l90_temp_config,
};

/* CURRENT CONFIG */
static const u32 nt25l90_curr_config[] = {
    HWMON_C_INPUT, /* Bias current */
    HWMON_C_INPUT, /* Modulation current */
    0
};

static const struct hwmon_channel_info nt25l90_curr = {
    .type = hwmon_curr,
    .config = nt25l90_curr_config,
};

/* VOLTAGE CONFIG */
static const u32 nt25l90_voltage_config[] = {
    HWMON_I_INPUT, /* Tx Supply Voltage */
    HWMON_I_INPUT, /* Rx Supply Voltage */
    HWMON_I_INPUT, /* Digital Supply Voltage */
    0
};

static const struct hwmon_channel_info nt25l90_voltage = {
    .type = hwmon_in,
    .config = nt25l90_voltage_config,
};

/* POWER CONFIG */
static const u32 nt25l90_power_config[] = {
    HWMON_P_INPUT, /* Tx Power */
    HWMON_P_INPUT, /* Rx Power */
    0
};

static const struct hwmon_channel_info nt25l90_power = {
    .type = hwmon_power,
    .config = nt25l90_power_config,
};

static const struct hwmon_channel_info *nt25l90_info[] = {
    &nt25l90_temp,
    &nt25l90_curr,
    &nt25l90_voltage,
    &nt25l90_power,
    NULL
};

static const struct hwmon_ops nt25l90_hwmon_ops = {
    .is_visible = nt25l90_is_visible,
    .read = nt25l90_read,
};

static const struct hwmon_chip_info driver_hwmon_info = {
    .ops = &nt25l90_hwmon_ops,
    .info = nt25l90_info,
};

static ssize_t regulator_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    struct device_data *dev_data = dev_get_drvdata(dev);

    if (!dev_data) {
        pr_err("Device data is NULL");
        return -EOPNOTSUPP;
    }
    return sprintf(buf, "%u enabled regulators\n", dev_data->regulator_cnt);
}

static ssize_t regulator_store(struct device *dev, 
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    struct device_data *dev_data = dev_get_drvdata(dev);
    int err;

    
    if (strncmp(buf, "enable", 6) == 0) {
        err = regulator_enable(dev_data->regulator);
        if (err) {
            pr_err("Can't enable regulator\n");
        }
        else {
            dev_data->regulator_cnt++;
            printk(KERN_INFO "Regulator is enabled\n");
        }
    }
    else if (strncmp(buf, "disable", 7) == 0 && dev_data->regulator_cnt > 0) {
        regulator_disable(dev_data->regulator);
        dev_data->regulator_cnt--;
        printk(KERN_INFO "Regulator is disabled\n");
    }
    else {
        printk(KERN_INFO "Only enable/disable operations are available.\n");
    }
    return count;
}

static DEVICE_ATTR(regulator_controller, S_IRUGO | S_IWUSR,
                   regulator_show, regulator_store);

static struct attribute *regulator_controller_attr[] = {
    &dev_attr_regulator_controller.attr,
    NULL
};

static const struct attribute_group regulator_attr_group = {
    .attrs = regulator_controller_attr,
};

static ssize_t reg_cntrl_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    struct device_data *dev_data = dev_get_drvdata(dev);
    int err = 0;
    unsigned int reg = 0;
    unsigned int val = 0;

    if (!dev_data) {
        pr_err("device_data is NULL");
        return -EOPNOTSUPP; 
    }

    err = kstrtouint(attr->attr.name, 16, &reg);
    if (err) {
        pr_err("Unable to parse register name");
        return -EOPNOTSUPP;
    }

    regmap_read(dev_data->regm, reg, &val);

    return sprintf(buf, "0x%x", val);
}

static ssize_t reg_cntrl_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    struct device_data *dev_data;
    char *ptr;
    char *buf_begin;
    char *part;
    unsigned int parsed_val;
    unsigned int reg;
    unsigned int val;
    unsigned int mask;
    int err;
    int arg_cnt = 0;

    dev_data = dev_get_drvdata(dev);
    if (!dev_data) {
        pr_err("Device data is NULL\n");
        return -EOPNOTSUPP;
    }

    ptr = kmalloc((count + 1) * sizeof(char), GFP_KERNEL);
    if (!ptr) {
        pr_err("Can't allocate memory to parse buffer");
        return -ENOMEM;
    }
    ptr = strncpy(ptr, buf, count);
    ptr[count] = '\0';
    /* Need to save pointer on buffer's beginning, because strsep changes ptr */
    buf_begin = ptr;

    /* Buffer parsing */
    while ((part = strsep(&ptr, " ")) != NULL) {
        err = kstrtouint(part, 0, &parsed_val);
        if (err) {
            pr_err("Parsing error. Can't parse %s\n", part);
            kfree(buf_begin);
            return -EOPNOTSUPP;
        }

        if (arg_cnt == 1)
            val = parsed_val;
        else if (arg_cnt == 0)
            mask = parsed_val;
        arg_cnt++;
    }

    err = kstrtouint(attr->attr.name, 16, &reg);
    if (err) {
        pr_err("Parsing error. Unable to parse reg value");
        kfree(buf_begin);
        return -EOPNOTSUPP;
    }

    /* Write value in register */
    if (arg_cnt == 2) {
        regmap_update_bits(dev_data->regm, reg, mask, val);
    }
    else {
        pr_err("Need to specify value and mask to update register value.\n");
        kfree(buf_begin);
        return -EOPNOTSUPP;
    }

    kfree(buf_begin);
    return count;
}

static struct attribute *reg_controller_attrs[NT25L90_REG_CNT];

static const struct attribute_group register_attr_group = {
    .attrs = reg_controller_attrs,
};

const struct attribute_group *nt25l90_groups[] = {
    &register_attr_group,
    &regulator_attr_group,
    NULL,
};

static struct device_attribute *nt25l90_create_attrs(struct device *dev)
{
    struct device_attribute *nt25l90_regs_sysfs_attr;
    int i = 0;

    nt25l90_regs_sysfs_attr = devm_kzalloc(dev, sizeof(struct device_attribute) * NT25L90_REG_CNT,
                                           GFP_KERNEL);
    if (!nt25l90_regs_sysfs_attr) {
        pr_err("Unable to alloc memory for sysfs attrs\n");
        return NULL;
    }
    for (i = 0; i < NT25L90_REG_CNT; i++) {
        nt25l90_regs_sysfs_attr[i].attr.name = nt25l90_regs_map[i];
        nt25l90_regs_sysfs_attr[i].attr.mode = S_IRUGO | S_IWUSR;
        nt25l90_regs_sysfs_attr[i].show = reg_cntrl_show;
        nt25l90_regs_sysfs_attr[i].store = reg_cntrl_store;
        reg_controller_attrs[i] = &nt25l90_regs_sysfs_attr[i].attr;
	sysfs_attr_init(reg_controller_attrs[i]);
    }
    return nt25l90_regs_sysfs_attr;
}

static int optical_driver_of_node_match(struct device *dev, const void *data)
{
    return ((struct device_data *)dev->driver_data)->id == *((int *)data);
}

/* Get optical device data with specific @id */
struct device_data *get_optical_device(int id)
{
    struct device *optical_dev = NULL;
    struct device_data *data = NULL;

    optical_dev = class_find_device(optical_drv_class, NULL, &id,
           optical_driver_of_node_match);
    if (!optical_dev) {
        pr_err("Can't find device from DT node\n");
        goto err_dev;
    }
    
    data = optical_dev->driver_data;
    if (IS_ERR(data)) {
        pr_err("can't get data\n");
        goto err_data;
    }
    return data;

err_data:
err_dev:
    return NULL;
}
EXPORT_SYMBOL(get_optical_device);

static int nt25l90_probe(struct i2c_client *client, const struct i2c_device_id *device)
{
    struct device_data *dev_data;
    struct device *hwmon_dev;
    struct device *dev = &client->dev;
    struct regmap *regm;
    int id;
    int err;

    regm = devm_regmap_init_i2c(client, &nt25l90_regmap_config);
    if (IS_ERR(regm)) {
        pr_err("Can't init regmap\n");
        return PTR_ERR(regm);
    }

    hwmon_dev = devm_hwmon_device_register_with_info(
        dev, client->name, regm, &driver_hwmon_info, NULL
    );
    if (IS_ERR(hwmon_dev)) {
        pr_err("Error with hwmon device registration\n");
        return PTR_ERR(hwmon_dev);
    }
    
    dev_data = devm_kzalloc(dev, sizeof(struct device_data), GFP_KERNEL);
    if (!dev_data) {
        return -ENOMEM;
    }

    dev_data->regm = regm;
    dev_data->regulator_cnt = 0;
    dev_data->regulator = devm_regulator_get(dev, "laser_driver_supply");
    if (IS_ERR(dev_data->regulator)) {
        pr_err("Can't get regulator\n");
        return -ENODEV;
    }

    /* Get id for new device */
    id = ida_simple_get(&optical_ida, 0, 0, GFP_KERNEL);
    if (id < 0) {
        err = id;
        dev_err(dev, "Unable to get ID\n");
        return err;
    }
    dev_data->id = id;
    dev_data->attrs = nt25l90_create_attrs(dev);

    /* This device is parameter of _store and _show functions. */
    dev = device_create(optical_drv_class, NULL, optical_dev, dev_data, "od%d", id);
    if (IS_ERR(dev)) {
        pr_err("Can't create optical device\n");
        return PTR_ERR(dev);
    }
    i2c_set_clientdata(client, dev_data);
    return 0;
}

static int nt25l90_remove(struct i2c_client *client)
{
    struct device_data *dev_data = i2c_get_clientdata(client);
    int i;

    /* Disabling all enabled regulators */
    if (dev_data->regulator_cnt > 0) {
        for (i = 0; i < dev_data->regulator_cnt; i++) {
            regulator_disable(dev_data->regulator);
        }
    }
    return 0;
}

static const struct of_device_id nt25l90_of_match_table[] = {
    {.compatible = "linux,nt25l90"},
    {}
};

MODULE_DEVICE_TABLE(of, nt25l90_of_match_table);

static const struct i2c_device_id nt25l90_device_ids[] = {
    {"nt25l90", 0}, 
    {}
};
MODULE_DEVICE_TABLE(i2c, nt25l90_device_ids);

static struct i2c_driver nt25l90_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = nt25l90_of_match_table
    },
    .probe = nt25l90_probe,
    .remove = nt25l90_remove,
    .id_table = nt25l90_device_ids,
};

static int __init nt25l90_init(void)
{
    int err;
    
    optical_drv_class = class_create(THIS_MODULE, "optical_driver");
    optical_drv_class->dev_groups = nt25l90_groups;
    err = register_chrdev_region(optical_dev, OPTICAL_DRIVER_MAX_MINORS, "od");
    if (err) {
        pr_err("Can't register char device region");
        return -ENOMEM;
    }
    return i2c_add_driver(&nt25l90_driver);
}

static void __exit nt25l90_exit(void)
{
    device_destroy(optical_drv_class, optical_dev);
    class_destroy(optical_drv_class);
    ida_destroy(&optical_ida);
    i2c_del_driver(&nt25l90_driver);
}

module_init(nt25l90_init);
module_exit(nt25l90_exit);
MODULE_VERSION("1.2.0");
MODULE_LICENSE("GPL v2");
