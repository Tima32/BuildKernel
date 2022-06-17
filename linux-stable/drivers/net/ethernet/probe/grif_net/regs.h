#ifndef _GRIF_REGS_H
#define _GRIF_REGS_H

//*******************************************************************
// FPGA NIC and CPUBUF features registers
//*******************************************************************

// TX enable
#define TX_EN_CR                             0x0000

// TX IRQ enable
// XXX:
//   TX IRQ not used now */
#define TX_IRQ_EN_CR                         0x0010

// For tx packet size, in bytes, without CRC
#define TX_PKT_SIZE_CR                       0x0020

// TX buffer status (1 -- buffer overflow, 0 -- buffer not overflow)
#define TX_BUFF_STAT_SR                      0x0030

// Flag for tx packet, request for saving tx timestamp
#define TX_TIMESTAMP_SAVE_CR                 0x0040

// Strob to reset tx timestamp valid signal after reading timestamp
#define TX_TIMESTAMP_RST_STB_CR              0x0050

// TX timestamp valid signal
#define TX_TIMESTAMP_VALID_SR                0x0060

// First address of TX timestamp (low 2 bytes)
#define TX_TIMESTAMP_VAL_W0_SR               0x0070

// Second address of TX timestamp
#define TX_TIMESTAMP_VAL_W1_SR               0x0071

// Third address of TX timestamp
#define TX_TIMESTAMP_VAL_W2_SR               0x0072

// Forth address of TX timestamp
#define TX_TIMESTAMP_VAL_W3_SR               0x0073

// Fifth address of TX timestamp (high 2 bytes)
#define TX_TIMESTAMP_VAL_W4_SR               0x0074

// First address of TX packet buffer
#define TX_WR_DATA_W0                        0x0100


// First address of RX packet buffer
#define RX_PKT_DATA_SR                       0x0000

// RX enable
#define RX_EN_CR                             0x2000

// RX IRQ enable, level
#define RX_IRQ_EN_CR                         0x2010

// Meta information -- availability of RX packet and its size
// XXX:
//   Reading from this register does not cause
//   reading from FPGA FIFO with meta information.
//
// Format:
// * Bit  [0]    -- packet availability
// * Bits [2:1]  -- reserved (always 0)
// * Bits [15:3] -- packet size (in bytes, without CRC)
//                  For example, for smallest Ethernet packet there will be 60.
#define RX_PKT_STATUS_SR                     0x2020

// Similar to the previous register, but
// reading from this register cause reading
// from FPGA FIFO with meta information.
#define RX_PKT_STATUS_WITH_POP_SR            0x2030

// Same as RX_PKT_STATUS_WITH_POP_SR and RX_PKT_STATUS_UPDATE_CR at the same time.
#define RX_PKT_STATUS_WITH_POP_AND_UPDATE_SR 0x2040

// Reports to FPGA that we have already processed the current packet
// and associated resources can be released.
#define RX_PKT_STATUS_UPDATE_CR              0x2050

// Errors count register. Unused now.
#define RX_PKT_SIZE_MISMATCH_DBG_CNT_SR      0x2060



/* Primary MAC */
#define HOST_MAC_W0_CR   0
/* Alternate MAC - not supported */
#define ALT_HOST_MAC_W0_CR   4

#define NIC_MAIN_CR      7
  /* Primary MAC mode */
  #define NIC_MAIN_CR_PRIMARY_MAC   0x0
  /* Alternate MAC mode - not supported */
  #define NIC_MAIN_CR_ALTERNATE_MAC 0x2
  /* Promisc MAC mode */
  #define NIC_MAIN_CR_PROMISC       0x3

/* Named NIC_CTRL_MTU in FPGA code, but it's not MTU */
#define NIC_CTRL_RX_SIZE_CR 8

/* STATUS REGISTERS */

#define NIC_CTRL_RX_MAX_SIZE_SR 1
#define NIC_CTRL_TX_MAX_SIZE_SR 2

//*******************************************************************
// FPGA PHY features registers
//*******************************************************************

#define MDIO_CR 0
  #define MDIO_CR_RST 0
  #define MDIO_CR_RUN 1
  #define MDIO_CR_COP_B0 2
  #define MDIO_CR_COP_B1 3

#define MDIOPHYAD_CR 1
  #define MDIOPHYAD_CR_PHYAD_B0 0
  #define MDIOPHYAD_CR_PHYAD_B4 4

#define MDIODEVAD_CR 2
  #define MDIODEVAD_CR_DEVAD_B0 0
  #define MDIODEVAD_CR_DEVAD_B4 4

#define MDIODATALO_CR 3

#define MDIODIV_CR 4
  #define MDIODIV_CR_DIV_B0 0
  #define MDIODIV_CR_DIV_B5 5

#define MDIO_SR 1
  #define MDIO_SR_BUSY 0
  #define MDIO_SR_DATAVAL 1
  #define MDIO_SR_FW_BUSY 2

#define MDIODATALO_SR 2

#define MDIO_OP_WRITE                   0x1
#define MDIO_OP_READ                    0x2

#define TRX_CR 8
  #define TRX_CR_EG_NRST   0

// 0000 - 10Gb/s
// 1001 - 10Mb/s;
// 1010 - 100 Mb/s;
// 1100 - 1000 Mb/s
#define PORT_SPEED_1000 (0xC)
#define PORT_SPEED_100  (0xA)
#define PORT_SPEED_10   (0x9)

#define PORT_SPEED_CR 9
  #define PORT_SPEED_CR_RX_SPEED_B0  0
  #define PORT_SPEED_CR_RX_SPEED_B3  3

  #define PORT_SPEED_CR_TX_SPEED_B0  4
  #define PORT_SPEED_CR_TX_SPEED_B3  7

#define	AUTONEG_CR		12
#define	AUTONEG_CR_AUTONEG_EN	0

void fpga_fw_write_reg(struct grif_netdev *port, int reg, u16 val);
u16 fpga_fw_read_reg(struct grif_netdev *port, int reg);

int fpga_fw_wait(struct grif_netdev *port, int reg, int bit, int sense);
int fpga_fw_test_bit(struct grif_netdev *port, int reg, int bit);

void fpga_fw_set_bit(struct grif_netdev *port, int reg, int bit);
void fpga_fw_clear_bit(struct grif_netdev *port, int reg, int bit);

void fpga_fw_write_pos(struct grif_netdev *port, int reg, int msb, int lsb, u16 val);
u16 fpga_fw_read_pos(struct grif_netdev *port, int reg, int msb, int lsb);

void fpga_phy_reset(struct grif_netdev *grif_port);
void fpga_phy_on(struct grif_netdev *grif_port);
void fpga_phy_off(struct grif_netdev *grif_port);

void fpga_set_promisc(struct grif_netdev *grif_port, bool promisc);
void fpga_set_rx_frame_size(struct grif_netdev *grif_port, int frame_size);
void fpga_get_rx_frame_size(struct grif_netdev *grif_port, int *frame_size);
void fpga_get_max_rx_frame_size(struct grif_netdev *grif_port, int *frame_size);
void fpga_get_max_tx_frame_size(struct grif_netdev *grif_port, int *frame_size);

void fpga_get_mac_address(struct grif_netdev *grif_port, u8 *addr);
void fpga_set_mac_address(struct grif_netdev *grif_port, u8 *addr);
void fpga_set_port_speed(struct grif_netdev *port, int speed);
void fpga_set_aneg(struct grif_netdev *port, u8 aneg);
u8 fpga_get_aneg(struct grif_netdev *port);

#endif
