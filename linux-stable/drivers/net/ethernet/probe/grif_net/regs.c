#include <linux/regmap.h>
#include <linux/ethtool.h>

#include "main.h"
#include "regs.h"

//*******************************************************************
// FPGA common register access functions
//*******************************************************************

/** 
 * Read FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number.
 * @return Readed data.
*/ 
u16 fpga_fw_read_reg(struct grif_netdev *port, int reg)
{
	unsigned int tmp;

	if (regmap_read(port->regmap, reg, &tmp))
		pr_err(DRV_NAME": Couldn't read FPGA register");

	return (u16) tmp & 0xffff;
}

/** 
 * Write FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param val - Data for write.
*/ 
void fpga_fw_write_reg(struct grif_netdev *port, int reg, u16 val)
{
	if (regmap_write(port->regmap, reg, (unsigned int) val))
		pr_err(DRV_NAME": Couldn't write FPGA register");
}

/** 
 * Read particular bits from FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param msb - Most significant bit for reading (included)
 * @param lsb - Least significant bit for reading (included)
 * @return Readed data.
*/ 
u16 fpga_fw_read_pos(struct grif_netdev *port,
                            int reg, int msb, int lsb)
{
	u16 mask = (1 << (msb - lsb + 1)) - 1, tmp;

	tmp = fpga_fw_read_reg(port, reg);
	return (tmp >> lsb) & mask;
}

/** 
 * Write particular bits to FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param msb - Most significant bit for writing (included).
 * @param lsb - Least significant bit for writing  (included).
 * @param val - Data for write.
*/ 
void fpga_fw_write_pos(struct grif_netdev *port,
                              int reg, int msb, int lsb, u16 val)
{
	u16 mask = (1 << (msb - lsb + 1)) - 1, tmp;

	tmp = fpga_fw_read_reg(port, reg) & ~(mask << lsb);
	fpga_fw_write_reg(port, reg, tmp | ((val & mask) << lsb));
}

/** 
 * Write '1' to selected bit in FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param bit - Bit index. 
*/ 
void fpga_fw_set_bit(struct grif_netdev *port, int reg, int bit)
{
	unsigned long tmp;

	tmp = fpga_fw_read_reg(port, reg);
	set_bit(bit, &tmp);
	fpga_fw_write_reg(port, reg, tmp);
}

/** 
 * Write '0' to selected bit in FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param bit - Bit index. 
*/ 
void fpga_fw_clear_bit(struct grif_netdev *port, int reg, int bit)
{
	unsigned long tmp = fpga_fw_read_reg(port, reg);

	clear_bit(bit, &tmp);
	fpga_fw_write_reg(port, reg, tmp);
}

/** 
 * Test selected bit in FPGA register.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param bit - Bit index. 
 * @return 1 if bit set, 0 otherwise.
*/ 
int fpga_fw_test_bit(struct grif_netdev *port, int reg, int bit)
{
	unsigned long tmp = fpga_fw_read_reg(port, reg);

	return test_bit(bit, &tmp);
}

/** 
 * Wait until selected bit in FPGA register take required value.
 * Timeout 1 HZ.
 *
 * @param fpga - FPGA device pointer.
 * @param reg - Register number. 
 * @param bit - Bit index. 
 * @param sense - Required value.
 * @return 0 if success, -ETIME if timeout.
*/ 
int fpga_fw_wait(struct grif_netdev *port, int reg, int bit, int sense)
{
	unsigned long timeout = round_jiffies(jiffies + HZ);

	while (!!fpga_fw_test_bit(port, reg, bit) ^ !!sense) {
		if (time_after(jiffies, timeout)) {
			dev_warn(&port->pdev->dev,
				 "%s: bit never set or cleared: "
				 "reg=%d, bit=%d, sense=%d\n",
				 __func__, reg, bit, !!sense);
			return -ETIME;
		}
	}

	return 0;
}

//*******************************************************************
// FPGA PHY helper functions
//*******************************************************************

void fpga_phy_off(struct grif_netdev *port)
{
        int cr = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num);
        fpga_fw_clear_bit(port, cr + TRX_CR, TRX_CR_EG_NRST);
}

void fpga_phy_on(struct grif_netdev *port)
{
        int cr = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num);
        fpga_fw_set_bit(port, cr + TRX_CR, TRX_CR_EG_NRST);
}

void fpga_phy_reset(struct grif_netdev *port)
{
        fpga_phy_off(port);
        mdelay(1);
        fpga_phy_on(port);

        // XXX:
        //   Time for PHY turning on.
        mdelay(10);
}

//*******************************************************************
// FPGA MAC helper functions
//*******************************************************************

/*
 * fpga_set_promisc -- enable or disable hardware promisc mode
 */
void fpga_set_promisc(struct grif_netdev *grif_port, bool promisc)
{
	int main_cr = stcmtk_get_cr_base_on_port(grif_port->nic_feature,
		grif_port->port_num) + NIC_MAIN_CR;

	fpga_fw_write_reg(grif_port, main_cr, promisc
			? NIC_MAIN_CR_PROMISC : NIC_MAIN_CR_PRIMARY_MAC );
}

void fpga_set_rx_frame_size(struct grif_netdev *grif_port, int frame_size)
{
	int reg_addr = stcmtk_get_cr_base_on_port(grif_port->nic_feature,
		grif_port->port_num) + NIC_CTRL_RX_SIZE_CR;

	fpga_fw_write_reg(grif_port, reg_addr, (u16)frame_size);
}

void fpga_get_rx_frame_size(struct grif_netdev *grif_port, int *frame_size)
{
	int reg_addr = stcmtk_get_cr_base_on_port(grif_port->nic_feature,
			grif_port->port_num) + NIC_CTRL_RX_SIZE_CR;

	*frame_size = (int)fpga_fw_read_reg(grif_port, reg_addr);
}
void fpga_get_max_rx_frame_size(struct grif_netdev *grif_port, int *frame_size)
{
	int reg_addr = stcmtk_get_sr_base_on_port(grif_port->nic_feature,
			grif_port->port_num) + NIC_CTRL_RX_MAX_SIZE_SR;

	*frame_size = (int)fpga_fw_read_reg(grif_port, reg_addr);
}

void fpga_get_max_tx_frame_size(struct grif_netdev *grif_port, int *frame_size)
{
	int reg_addr = stcmtk_get_sr_base_on_port(grif_port->nic_feature,
		grif_port->port_num) + NIC_CTRL_TX_MAX_SIZE_SR;

	*frame_size = (int)fpga_fw_read_reg(grif_port, reg_addr);
}

void fpga_get_mac_address(struct grif_netdev *grif_port, u8 *addr)
{
	int mac_cr = stcmtk_get_cr_base_on_port(grif_port->nic_feature,
		grif_port->port_num) + HOST_MAC_W0_CR;

	uint16_t w0, w1, w2;

	w0 = fpga_fw_read_reg(grif_port, mac_cr + 0);
	w1 = fpga_fw_read_reg(grif_port, mac_cr + 1);
	w2 = fpga_fw_read_reg(grif_port, mac_cr + 2);

	addr[0] = w0 & 0xFF;
	addr[1] = w0 >> 8;
	addr[2] = w1 & 0xFF;
	addr[3] = w1 >> 8;
	addr[4] = w2 & 0xFF;
	addr[5] = w2 >> 8;
}

void fpga_set_mac_address(struct grif_netdev *grif_port, u8 *dev_addr)
{
	int mac_cr = stcmtk_get_cr_base_on_port(grif_port->nic_feature,
		grif_port->port_num) + HOST_MAC_W0_CR;

	fpga_fw_write_reg(grif_port, mac_cr + 2, (dev_addr[5] << 8) | dev_addr[4]);
	fpga_fw_write_reg(grif_port, mac_cr + 1, (dev_addr[3] << 8) | dev_addr[2]);
	fpga_fw_write_reg(grif_port, mac_cr + 0, (dev_addr[1] << 8) | dev_addr[0]);
}

/**
 * Select speed mode in FPGA MAC core.
 *
 * @param fpga - FPGA device pointer.
 * @param speed - Speed mode (10/100/1000).
 */
void fpga_set_port_speed(struct grif_netdev *port, int speed)
{
        int speed_for_fpga;
        uint16_t cr_base;

        switch(speed) {
        case SPEED_10:
                netdev_dbg(port->netdev, "Setting speed 10");
                speed_for_fpga = PORT_SPEED_10;
                break;
        case SPEED_100:
                netdev_dbg(port->netdev, "Setting speed 100");
                speed_for_fpga = PORT_SPEED_100;
                break;
        case SPEED_1000:
                netdev_dbg(port->netdev, "Setting speed 1000");
                speed_for_fpga = PORT_SPEED_1000;
                break;
	case SPEED_UNKNOWN:
                netdev_warn(port->netdev, "Unknown port speed. Won't be changed.\n");
                return;
        default:
                netdev_warn(port->netdev, "Unsupported port speed %d. Won't be changed.\n", speed);
                return;

        }

        /* Setting RX speed */
        cr_base = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num) + PORT_SPEED_CR;
        fpga_fw_write_pos(port, cr_base,
                          PORT_SPEED_CR_RX_SPEED_B3, PORT_SPEED_CR_RX_SPEED_B0,
                          speed_for_fpga);

        /* Setting TX speed */
        cr_base = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num) + PORT_SPEED_CR;
        fpga_fw_write_pos(port, cr_base,
                          PORT_SPEED_CR_TX_SPEED_B3, PORT_SPEED_CR_TX_SPEED_B0,
                          speed_for_fpga);
}

u8 fpga_get_aneg(struct grif_netdev *port)
{
        uint16_t aneg_cr = stcmtk_get_cr_base_on_port(port->phy_feature,
			port->port_num) + AUTONEG_CR;

	if (fpga_fw_test_bit(port, aneg_cr, AUTONEG_CR_AUTONEG_EN))
		return AUTONEG_ENABLE;
	else
		return AUTONEG_DISABLE;
}

void fpga_set_aneg(struct grif_netdev *port, u8 aneg)
{
        uint16_t aneg_cr = stcmtk_get_cr_base_on_port(port->phy_feature,
			port->port_num) + AUTONEG_CR;

	if (aneg == AUTONEG_ENABLE) {
		if (!fpga_fw_test_bit(port, aneg_cr, AUTONEG_CR_AUTONEG_EN))
			fpga_fw_set_bit(port, aneg_cr, AUTONEG_CR_AUTONEG_EN);
	} else if (aneg == AUTONEG_DISABLE) {
		if (fpga_fw_test_bit(port, aneg_cr, AUTONEG_CR_AUTONEG_EN))
			fpga_fw_clear_bit(port, aneg_cr, AUTONEG_CR_AUTONEG_EN);
	}
}
