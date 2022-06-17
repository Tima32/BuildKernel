#ifndef SOC_NET_MAIN__H
#define SOC_NET_MAIN__H

#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/net_tstamp.h>

#define	DRV_NAME	KBUILD_MODNAME
#define	DRV_VERSION	"0.10"

struct dma_buff {
	struct sk_buff	*skb;
	unsigned char	*skb_data;
	size_t		data_len;
	dma_addr_t	dma_addr;
};

struct et_netdev {
	struct net_device	*netdev;
	struct platform_device	*pdev;
	struct mii_bus		*mdio_bus;
	struct phylink		*phylink;
	struct phylink_config	phylink_config;
	struct stcmtk_common_fpga	*fpga;
	struct fpga_feature		*nic_feature;
	int		port_num;
	unsigned int	tx_irq;
	unsigned int	rx_irq;
	u32 msg_enable;

	struct napi_struct rx_napi;
	struct napi_struct tx_napi;

#define TX_RING_DEPTH	64
#define RX_RING_DEPTH	63
	struct dma_buff tx_ring[TX_RING_DEPTH];
	struct dma_buff rx_ring[RX_RING_DEPTH];

	size_t	data_size;

	size_t	tx_head;
	size_t	tx_tail;
	size_t	rx_head;
	size_t	rx_tail;

	struct hwtstamp_config	hw_ts_config;
	struct work_struct	tx_ts_work;
	struct sk_buff	*tx_ts_skb;
	unsigned long	tx_ts_start;
	spinlock_t	tx_ts_lock;

	struct gpio_desc	*link_led_gpio;
};

#endif /* SOC_NET_MAIN__H */
