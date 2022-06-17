#ifndef GRIF_FPGA_MGR_GRIF_FPGA__H
#define GRIF_FPGA_MGR_GRIF_FPGA__H

#include <linux/fpga/fpga-mgr.h>
#include <linux/regulator/consumer.h>

/*
 * Actually every FPGA register are shifted by four FPGA words from previous
 * register. Every FPGA word equals two CPU bytes (16 bit), so you have to
 * multiply byte address by 8 to access FPGA numeric register address correctly
 * (and by 4 to access FPGA byte register address).
 *
 * | 1st FPGA word   |     dummy       |    dummy        |    dummy        |
 * |0      7|8     15|16    23|24    31|32    39|40    47|48    55|56    63|
 *
 * | 2nd FPGA word   |     dummy       |    dummy        |    dummy        |
 * |0      7|8     15|16    23|24    31|32    39|40    47|48    55|56    63|
 */
#define FPGA_SPACE_BYTE_OFFSET	4

#define WAIT_TIMEOUT 5000 /* 5 sec */

struct stcmtk_common_fpga;

/**
 * Primary Grif-device FPGA description.
 *
 * @name:           Pointer to FPGA name string. Should be received from device
 *                  tree.
 * @state:          Current FPGA state.
 * @ops:            FPGA file operations for its character device.
 * @private_data:   Pointer to side data for different purposes.
 * @vcc_supply:     Pointer to Main VCC Supply regulator.
 * @vcc_tra_supply: Pointer to Transcievers VCC Supply regulator.
 * @vcc_aux_supply: Pointer to Aux VCC Supply regulator.
 * @test_mode:      If true, do not parse features, do not create regmap.
 * @cs              Chip Select GPIO node. For SPI mode only.
 */
struct grif_fpga {
	const char *name;
	enum fpga_mgr_states state;
	struct grif_fpga_ops *ops;
	void *private_data;
	struct regulator *vcc_supply;
	struct regulator *vcc_tra_supply;
	struct regulator *vcc_aux_supply;
	bool test_mode;
	struct gpio_desc *cs;
	struct mutex regmap_lock;
};

/**
 * FPGA firmware data description.
 *
 * @size:   Firmware data size in bytes.
 * @buf:    Pointer to firmware data.
 *
 */
struct grif_firmware {
	size_t size;
	char *buf;
};

struct grif_fpga_ops {
	int (*state_get)(struct stcmtk_common_fpga *g);
};

#define GRIF_FPGA_STATE(g)   g->ops->state_get(g)

int grif_fpga_state_get(struct stcmtk_common_fpga *grif_fpga);
#endif // GRIF_FPGA_MGR_GRIF_FPGA__H
