#define	NETDMA_BASE_PORT0	0x2000
#define	NETDMA_BASE_PORT1	0x4000

#define	NETDMA_CONTROL		0x00
#define	NETDMA_STATUS		0x04
#define	NETDMA_TX_STATUS	0x08
#define	NETDMA_RX_REPORT	0x0C
#define	NETDMA_TX_DESC		0x10
#define	NETDMA_RX_DESC		0x14

#define	CTRL_RESET		BIT(0)
#define	CTRL_RX_IRQ_ENABLE	BIT(1)
#define	CTRL_TX_IRQ_ENABLE	BIT(2)

#define	STAT_RX_REPORT_BUFF_EMPTY	BIT(1)
#define	STAT_RX_DESC_BUFFER_FULL	BIT(4)
#define	STAT_TX_DESC_BUFFER_FULL	BIT(5)

#define	TX_DESC_TIMESTAMP_OFFSET	24

unsigned int fpga_read_dma_reg(struct et_netdev *et_port, unsigned int reg);
void fpga_write_dma_reg(struct et_netdev *et_port, unsigned int reg,
			unsigned int value);
void fpga_dma_reg_set_bit(struct et_netdev *et_port, unsigned int reg,
			  unsigned int bit);
bool fpga_dma_reg_test_bit(struct et_netdev *et_port, unsigned int reg,
			   unsigned int bit);
void fpga_dma_reg_clear_bit(struct et_netdev *et_port, unsigned int reg,
			    unsigned int bit);
