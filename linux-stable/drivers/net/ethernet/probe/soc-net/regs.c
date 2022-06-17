#include <linux/regmap.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>
#include "main.h"

/*
 * FPGA common register access functions
 */

/*
 * Read FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number.
 * @return Readed data.
 */
u16 fpga_fw_read_reg(struct stcmtk_common_fpga *fpga, int reg)
{
	unsigned int tmp;

	if (regmap_read(fpga->regmap, reg, &tmp))
		dev_err(fpga->dev, "Couldn't read FPGA register\n");

	return (u16) tmp & 0xffff;
}

/* 
 * Write FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param val - Data for write.
 */
void fpga_fw_write_reg(struct stcmtk_common_fpga *fpga, int reg, u16 val)
{
	if (regmap_write(fpga->regmap, reg, (unsigned int) val))
		dev_err(fpga->dev, "Couldn't write FPGA register\n");
}

/*
 * Read particular bits from FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param msb - Most significant bit for reading (included)
 * @param lsb - Least significant bit for reading (included)
 * @return Readed data.
 */
u16 fpga_fw_read_pos(struct stcmtk_common_fpga *fpga, int reg, int msb, int lsb)
{
	u16 mask = (1 << (msb - lsb + 1)) - 1, tmp;

	tmp = fpga_fw_read_reg(fpga, reg);
	return (tmp >> lsb) & mask;
}

/*
 * Write particular bits to FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param msb - Most significant bit for writing (included).
 * @param lsb - Least significant bit for writing  (included).
 * @param val - Data for write.
 */
void fpga_fw_write_pos(struct stcmtk_common_fpga *fpga, int reg, int msb, int lsb,
		       u16 val)
{
	u16 mask = (1 << (msb - lsb + 1)) - 1, tmp;

	tmp = fpga_fw_read_reg(fpga, reg) & ~(mask << lsb);
	fpga_fw_write_reg(fpga, reg, tmp | ((val & mask) << lsb));
}

/*
 * Write '1' to selected bit in FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param bit - Bit index.
 */
void fpga_fw_set_bit(struct stcmtk_common_fpga *fpga, int reg, int bit)
{
	unsigned long tmp;

	tmp = fpga_fw_read_reg(fpga, reg);
	set_bit(bit, &tmp);
	fpga_fw_write_reg(fpga, reg, tmp);
}

/*
 * Write '0' to selected bit in FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number.
 * @param bit - Bit index.
 */
void fpga_fw_clear_bit(struct stcmtk_common_fpga *fpga, int reg, int bit)
{
	unsigned long tmp = fpga_fw_read_reg(fpga, reg);

	clear_bit(bit, &tmp);
	fpga_fw_write_reg(fpga, reg, tmp);
}

/*
 * Test selected bit in FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number.
 * @param bit - Bit index.
 * @return 1 if bit set, 0 otherwise.
 */
int fpga_fw_test_bit(struct stcmtk_common_fpga *fpga, int reg, int bit)
{
	unsigned long tmp = fpga_fw_read_reg(fpga, reg);

	return test_bit(bit, &tmp);
}

/*
 * Wait until selected bit in FPGA register take required value.
 * Timeout 1 HZ.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number.
 * @param bit - Bit index.
 * @param sense - Required value.
 * @return 0 if success, -ETIME if timeout.
 */
int fpga_fw_wait(struct stcmtk_common_fpga *fpga, int reg, int bit, int sense)
{
	unsigned long timeout = round_jiffies(jiffies + HZ);

	while (!!fpga_fw_test_bit(fpga, reg, bit) ^ !!sense) {
		if (time_after(jiffies, timeout)) {
			dev_warn(fpga->dev,
				 "%s: bit never set or cleared: reg=%d, bit=%d, sense=%d\n",
				 __func__, reg, bit, !!sense);

			return -ETIME;
		}
	}

	return 0;
}

void fpga_write_nic_reg(struct et_netdev *et_port, unsigned int reg,
			unsigned int value)
{
	unsigned int nic_base_reg;
	struct stcmtk_common_fpga *fpga = et_port->fpga;

	nic_base_reg = stcmtk_get_cr_base_on_port(et_port->nic_feature,
							et_port->port_num);
	fpga_fw_write_reg(fpga, nic_base_reg + reg, value);
}
