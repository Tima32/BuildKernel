#ifndef NT25L90_H_
#define NT25L90_H_
#include <linux/device.h>
#include <linux/regmap.h>

#define NT25L90_REG_CNT 43
static char *nt25l90_regs_map[] = {
    /* Non-volatile memory password registers */
    "0x16", "0x3D",
    /* Non-volatile memory user registers */
    "0x60", "0x61", "0x62", "0x63", "0x64", "0x65", "0x66",
    "0x67", "0x68", "0x69", "0x6A", "0x6B", "0x6C", "0x6D",
    /* Volatile general user registers */
    "0xA0", "0xA1", "0xA2", "0xA3", "0xA4",
    "0xA5", "0xA6", "0xA7", "0xA8", "0xA9",
    "0xAA", "0xAB", "0xAC", "0xAD", "0xAE",
    "0xAF", "0xB0", "0xB1", "0xB2", "0xB3",
    "0xB4", "0xB5", "0xB6",
    /* Factory setting non-volatile memory registers */
    "0xD0", "0xD1",
    /* Non-volatile memory program registers */
    "0xE5", "0xEA"
};
/**
 * struct device_data - store data between _store and _show functions.
 * @regm - regmap
 * @regulator - regulator
 * @regulator_cnt - count of enabled regulators
 */
struct device_data {
    int id;
    struct regmap *regm;
    struct regulator *regulator;
    size_t regulator_cnt;
    struct device_attribute *attrs;
};


struct device_data *get_optical_device(int);

#endif

