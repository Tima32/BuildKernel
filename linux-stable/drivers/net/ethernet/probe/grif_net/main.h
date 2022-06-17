#ifndef GRIF_NET_MAIN__H
#define GRIF_NET_MAIN__H

#include <linux/netdevice.h>
#include <linux/net_tstamp.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/dmaengine.h>

#include <grif/grif_fpga_mgr/grif_fpga.h>
#include <common_fpga/fpgafeat.h>

#define DRV_NAME "grif_net"

#define RX_RING_DEPTH	256
#define TX_RING_DEPTH	256

// Network settings
#define NAPI_WEIGHT    64

struct dma_buff {
	struct sk_buff	*skb;
	dma_addr_t	dma_addr;
	dma_cookie_t	cookie;
	size_t		len;
	u16		tx_ts_need;
	unsigned char	*skb_data;
};

struct grif_netdev {
	struct net_device *netdev;
	struct platform_device *pdev;
	struct regmap *regmap;

	struct fpga_feature *tx_timestamp_feature;
	struct fpga_feature *nic_feature;
	struct fpga_feature *phy_feature;

	struct mii_bus *mdio_bus;

	phy_interface_t interface;
	struct phylink *phylink;
	struct phylink_config phylink_config;
	struct gpio_desc *link_gpio;
	struct gpio_desc *link_led_gpio;
	int link_irq;

	struct napi_struct napi;
	struct work_struct restart_work;

	struct dma_buff rx_ring[RX_RING_DEPTH];
	struct dma_buff tx_ring[TX_RING_DEPTH];

	int port_num;
	u32 msg_enable;

	unsigned int rx_head;
	unsigned int rx_tail;

	unsigned int tx_head;
	unsigned int tx_tail;

	unsigned int max_rx_frame_size;
	unsigned int max_tx_frame_size;

	/* TX timestamp will appear asynchronously with respect
	 * to processing of DMA-descriptor. So we use
	 * a work_struct to periodically polling FPGA registers. */
	struct work_struct tx_hwtstamp_work;

	/* TX timestamp passed via control/status registers.
	 * At most one TX packet may be marked for time stamping.
	 * So we need to remember which sk_buff wants a TX timestamp */
	struct sk_buff *tx_hwtstamp_skb;

	/* Time of packet transmition request.
	 * Need for timeout calculation.
	 * Current timeout -- 1 HZ */
	unsigned long tx_hwtstamp_start;

	spinlock_t tx_hwtstamp_lock;

	/* Unused now */
	u32 tx_hwtstamp_timeouts;

	struct hwtstamp_config hwtstamp_config;

	int rx_irq;
	int tx_irq;

        void __iomem *regs_tx;
        void __iomem *regs_rx;

	int tx_is_stopped;

	/* Function to process received packets */
	int (*rx_poll)(struct grif_netdev *grif_port, int budget);
	/* Function to pass skb up to network stack */
	void (*receive)(struct napi_struct *napi, struct sk_buff *skb);

	struct dma_chan *dma_rx_chan;
	struct dma_chan	*dma_tx_chan;
	dma_addr_t	rx_buff;
	dma_addr_t	tx_buff;
};

#endif // GRIF_NET_MAIN__H
