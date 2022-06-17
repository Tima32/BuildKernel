#define	TX_TS_BASE_PORT0	0x7000
#define	TX_TS_BASE_PORT1	0x8000
#define	TX_TS_LOW	0x0
#define	TX_TS_MID	0x4
#define	TX_TS_HIGH	0x8

#define	TX_TS_VALID_BIT	BIT(31)

#define	TICKS_NS_SHIFT	3

u64 fpga_read_tx_timestamp(struct et_netdev *et_port);
