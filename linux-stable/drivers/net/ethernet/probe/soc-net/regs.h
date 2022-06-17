#ifndef SOC_NET_REGS__H
#define SOC_NET_REGS__H

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

/*
 * FPGA PHY features registers
 */

// FPGA PHY features register offsets
#define PHY_CR_PORT0	9
#define PHY_CR_PORT1	21

#define PHY_SR_PORT0	33
#define PHY_SR_PORT1	37

#define MDIO_CR	0
#define MDIO_CR_RST	0
#define MDIO_CR_RUN	1
#define MDIO_CR_COP_B0	2
#define MDIO_CR_COP_B1	3

#define MDIOPHYAD_CR	1
#define MDIOPHYAD_CR_PHYAD_B0	0
#define MDIOPHYAD_CR_PHYAD_B4	4

#define MDIODEVAD_CR 2
#define MDIODEVAD_CR_DEVAD_B0 0
#define MDIODEVAD_CR_DEVAD_B4 4

#define MDIODATALO_CR	3

#define MDIODIV_CR		4
#define MDIODIV_CR_DIV_B0	0
#define MDIODIV_CR_DIV_B5	5

#define TRX_CR	8
#define TRX_CR_EG_NRST	0

// 0000 - 10Gb/s
// 1001 - 10Mb/s;
// 1010 - 100 Mb/s;
// 1100 - 1000 Mb/s
#define	PORT_SPEED_10000	(0x0)
#define	PORT_SPEED_1000		(0xC)
#define	PORT_SPEED_100		(0xA)
#define	PORT_SPEED_10		(0x9)

#define PORT_SPEED_CR	9
#define PORT_SPEED_CR_RX_SPEED_B0	0
#define PORT_SPEED_CR_RX_SPEED_B3	3

#define PORT_SPEED_CR_TX_SPEED_B0	4
#define PORT_SPEED_CR_TX_SPEED_B3	7

#define MDIO_SR	1
#define MDIO_SR_BUSY	0
#define MDIO_SR_DATAVAL	1
#define MDIO_SR_FW_BUSY	2

#define MDIODATALO_SR	2

#define MDIO_OP_ADDR		0x00
#define MDIO_OP_WRITE		0x01
#define MDIO_OP_READ_C22	0x02
#define MDIO_OP_READ_C45	0x03
#define MDIO_OP_READ_ADDR	0x02

#define MII_MARVELL_PHY_PAGE		22
#define MII_88E1510_PHY_LED_PAGE	3
#define MII_88E1510_PHY_LED_CTRL	16

/*
 *  PHY LEDs configurations
 *    1) For copper: 
 *         LED[0] (green)  - Solid link, Off no link
 *         LED[1] (unused) - Off 
 *         LED[2] (orange) - Blink on activity, else off
 *
 *    2) For fiber:      
 *         LED[0] (green)  - Off
 *         LED[1] (unused) - Off 
 *         LED[2] (orange) - Off
 *
 *  See 88E1510-88E1512 Datasheet:
 *    Page 86, 2.26.4, Table 50
 *    Page 159, Table 124
 */
#define MII_88E1510_PHY_LED_COPPER	0x1480
#define MII_88E1510_PHY_LED_FIBER	0x1888

#define HOST_MAC_W0_CR	0
#define NIC_MAIN_CR	7
#define NIC_CTRL_MTU	8

#define NIC_MAIN_CR_PRIMARY_MAC	0x0
#define NIC_MAIN_CR_PROMISC	0x3

u16 fpga_fw_read_reg(struct stcmtk_common_fpga *fpga, int reg);
void fpga_fw_write_reg(struct stcmtk_common_fpga *fpga, int reg, u16 val);

u16 fpga_fw_read_pos(struct stcmtk_common_fpga *fpga, int reg, int msb, int lsb);
void fpga_fw_write_pos(struct stcmtk_common_fpga *fpga, int reg, int msb, int lsb,
		       u16 val);

void fpga_fw_set_bit(struct stcmtk_common_fpga *fpga, int reg, int bit);
void fpga_fw_clear_bit(struct stcmtk_common_fpga *fpga, int reg, int bit);
int fpga_fw_test_bit(struct stcmtk_common_fpga *fpga, int reg, int bit);

int fpga_fw_wait(struct stcmtk_common_fpga *fpga, int reg, int bit, int sense);

void fpga_write_nic_reg(struct et_netdev *et_port, unsigned int reg,
			unsigned int value);

#endif /* SOC_NET_REGS__H */
