#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/net_tstamp.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>

#include "main.h"
#include "regs.h"
#include "net.h"
#include "timestamp.h"

/*
 * grif_net_change_rx_flags -- enable or disable promisc mode
 */
static void grif_net_change_rx_flags(struct net_device *netdev, int flags)
{
	struct grif_netdev *grif_port;

	grif_port = netdev_priv(netdev);

	/* We react only on promiscuous mode setting/unsetting although the
	 * kernel may call this function for changes in other netdev flags.
	 */
	if (flags != IFF_PROMISC)
		return;

	fpga_set_promisc(grif_port, netdev->flags & IFF_PROMISC);
}

static int grif_net_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	/* Maximal frame size for requested MTU */
	int rx_frame_size_for_new_mtu = ETH_HLEN + VLAN_HLEN + new_mtu
			+ ETH_FCS_LEN;

	if (rx_frame_size_for_new_mtu > grif_port->max_rx_frame_size) {
		netdev_info(netdev, "Maximum frame size (%d) for requested "
				"MTU (%d) is larger than maximum received "
				"frame size (%d) supported on this FPGA "
				"firmware", rx_frame_size_for_new_mtu, new_mtu,
				grif_port->max_rx_frame_size);
		return -EINVAL;
	}

	netdev->mtu = new_mtu;

	return 0;
}


void grif_net_init_mac_address(struct grif_netdev *grif_port)
{
	struct net_device *netdev = grif_port->netdev;
	u8 tmpaddr[ETH_ALEN];

	/* Try existing MAC for interface */
	memcpy(tmpaddr, netdev->dev_addr, ETH_ALEN);
	if (is_valid_ether_addr(tmpaddr)) {
		netdev_dbg(netdev, "MAC Address is read from configuration: %pM\n", tmpaddr);
	} else {
		/* Try reading MAC address from FPGA. */
		fpga_get_mac_address(grif_port, tmpaddr);
		netdev_dbg(netdev, "MAC Address is read from FPGA: %pM\n", tmpaddr);
	}

	if (!is_valid_ether_addr(tmpaddr)) {
		/* FPGA values are invalid, generate random MAC. */
		random_ether_addr(tmpaddr);
		netdev_dbg(netdev, "MAC Address is set to random_ether_addr: %pM\n", tmpaddr);
	}
	fpga_set_mac_address(grif_port, tmpaddr);
	memcpy(netdev->dev_addr, tmpaddr, ETH_ALEN);
}

static int grif_net_set_mac_address(struct net_device *netdev, void *addr)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);
	struct sockaddr *sock_addr = addr;
	int ret;

	ret = eth_prepare_mac_addr_change(netdev, addr);
	if (ret < 0)
		return ret;

	fpga_set_mac_address(grif_port, sock_addr->sa_data);

	eth_commit_mac_addr_change(netdev, addr);
	return 0;
}

static int grif_add_rx_buffer(struct grif_netdev *grif_port, int seq_num)
{
	struct net_device *netdev = grif_port->netdev;
	struct sk_buff *skb;
	struct dma_buff *buff;
	unsigned int next_head;

	next_head = (grif_port->rx_head + 1) % RX_RING_DEPTH;

	//skb = netdev_alloc_skb_ip_align(netdev, grif_port->max_rx_frame_size);
	skb = __netdev_alloc_skb(netdev, grif_port->max_rx_frame_size,
				 GFP_ATOMIC | GFP_DMA);
        //printk("ALLOC: SKB ADDR = 0x%p, DATA ADDR = 0x%p\n", skb, skb->data);

	if (unlikely(skb == NULL)) {
		netif_err(grif_port, drv, netdev,
			  "Failed to allocate a socket buffer for RX\n");
		return -ENOMEM;
	}

	buff = &grif_port->rx_ring[grif_port->rx_head];
	buff->skb = skb;
	grif_port->rx_head = next_head;

	return 0;
}

static void grif_clean_rx_ring(struct grif_netdev *grif_port)
{
	int i;

	for (i = 0; i < RX_RING_DEPTH; i++) {
		if (grif_port->rx_ring[i].skb) {
			dev_kfree_skb_any(grif_port->rx_ring[i].skb);

			grif_port->rx_ring[i].skb = NULL;
		}
	}

	grif_port->rx_head = 0;
	grif_port->rx_tail = 0;
}

static void grif_clean_tx_ring(struct grif_netdev *grif_port)
{
	size_t i;

	for (i = 0; i < TX_RING_DEPTH; i++) {
		kfree(grif_port->tx_ring[i].skb_data);
		grif_port->tx_ring[i].skb_data = NULL;
	}

	grif_port->tx_head = 0;
	grif_port->tx_tail = 0;
}


static int grif_init_rx_ring(struct grif_netdev *grif_port)
{
	int i, err;

	/* Fill up the RX descriptor FIFO */
	for (i = 0; i < RX_RING_DEPTH; i++) {
		err = grif_add_rx_buffer(grif_port, i);
		if (err == -ENOMEM) {
			grif_clean_rx_ring(grif_port);
			return err;
		}
	}

	return 0;
}

static int grif_init_tx_ring(struct grif_netdev *grif_port)
{
	size_t i;

	for (i = 0; i < TX_RING_DEPTH; i++) {
		grif_port->tx_ring[i].skb_data = kzalloc(grif_port->max_tx_frame_size,
							 GFP_DMA);
		if (!grif_port->tx_ring[i].skb_data) {
			grif_clean_tx_ring(grif_port);
			return -ENOMEM;
		}
	}

	return 0;
}

static inline void write_rx_reg(struct grif_netdev *grif_port, u16 reg_addr, u16 val)
{
	u16 data[4] = { 0 };

        data[0] = val;
        memcpy_toio(grif_port->regs_rx + reg_addr*2, &data, 8);
}

static inline void write_tx_reg(struct grif_netdev *grif_port, u16 reg_addr, u16 val)
{
	u16 data[4] = { 0 };

        data[0] = val;
        memcpy_toio(grif_port->regs_tx + reg_addr*2, &data, 8);
}

void grif_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	napi_gro_receive(napi, skb);
}

void grif_receive_skb(struct napi_struct *napi, struct sk_buff *skb)
{
	netif_receive_skb(skb);
}

static size_t rx_queue_len(struct grif_netdev *grif_port)
{
	unsigned int rx_head = grif_port->rx_head;
	unsigned int rx_tail = grif_port->rx_tail;

	if (rx_head > rx_tail)
		return (RX_RING_DEPTH + rx_tail - rx_head);
	else
		return (rx_tail - rx_head);
}

static void dma_rx_schedule(void *data);

static void dma_copy_schedule(struct grif_netdev *grif_port)
{
	struct device *dev = &grif_port->pdev->dev;
	struct net_device *netdev = grif_port->netdev;
	struct sk_buff *skb;
	unsigned int rx_tail, rx_bytes, dma_len;
	size_t rx_pkt_num;
	u16 data[4] = { 0 };
	dma_addr_t dst_addr;
	struct dma_async_tx_descriptor *desc = NULL;

	memcpy_fromio(&data, grif_port->regs_rx + RX_PKT_STATUS_SR*2, 8);
	rx_pkt_num = data[0];

	if (rx_pkt_num == 0 ||
	    rx_queue_len(grif_port) + rx_pkt_num > RX_RING_DEPTH - 1) {
		return;
	}

	while (rx_pkt_num) {
		memcpy_fromio(&data,
			      grif_port->regs_rx + RX_PKT_STATUS_WITH_POP_SR*2,
			      8);
		rx_bytes = data[0] >> 3;

		rx_tail = grif_port->rx_tail;
		skb = grif_port->rx_ring[rx_tail].skb;

		dma_len = ((rx_bytes + 3) / 4) * 4;
		dst_addr = dma_map_single(dev, skb->data, dma_len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, dst_addr)) {
			netif_err(grif_port, drv, netdev,
					"Failed to map RX packet data to DMA\n");
			BUG();
		}

		desc = dmaengine_prep_dma_memcpy(grif_port->dma_rx_chan,
				dst_addr, grif_port->rx_buff, dma_len, 0);
		if (!desc) {
			netif_err(grif_port, drv, netdev,
					"Failed to prepare RX DMA transaction\n");
			BUG();
		}

		grif_port->rx_ring[rx_tail].dma_addr = dst_addr;
		grif_port->rx_ring[rx_tail].len      = rx_bytes;

		if (rx_pkt_num == 1)
			desc->callback = dma_rx_schedule;

		desc->callback_param = grif_port;

		grif_port->rx_ring[rx_tail].cookie = dmaengine_submit(desc);

		grif_port->rx_tail = (rx_tail + 1) % RX_RING_DEPTH;
		rx_pkt_num--;

		dma_async_issue_pending(grif_port->dma_rx_chan);
	}
}

static void dma_rx_schedule(void *data)
{
	struct grif_netdev *grif_port = data;

	dma_copy_schedule(grif_port);
	napi_schedule(&grif_port->napi);
}

int grif_rx_dma_poll(struct grif_netdev *grif_port,int budget)
{
	struct net_device *netdev = grif_port->netdev;
	struct device	*dev = &grif_port->pdev->dev;
	size_t		rx_head;
	struct sk_buff	*skb;
	dma_addr_t	dma_addr;
	enum dma_status	status;
	size_t		len;
	size_t		dma_len;
	int		howmany = 0;
	char		ts[TS_SIZE] = {0};
	u64		ns;

	while (rx_queue_len(grif_port) > 0 && howmany < budget) {
		rx_head = grif_port->rx_head;
		status = dmaengine_tx_status(grif_port->dma_rx_chan,
				grif_port->rx_ring[rx_head].cookie, NULL);

		if (status != DMA_COMPLETE)
			break;

		skb	= grif_port->rx_ring[rx_head].skb;
		len	= grif_port->rx_ring[rx_head].len;
		dma_len	= ((len + 3) / 4) * 4;
		dma_addr = grif_port->rx_ring[rx_head].dma_addr;

		dma_unmap_single(dev, dma_addr, dma_len, DMA_FROM_DEVICE);

		memcpy(ts, skb->data, TS_SIZE);

		skb_put(skb, len);
		skb_pull(skb, TS_SIZE);

		ns = le64_to_cpup( (__le64*)ts ) << TICKS_NS_SHIFT;

		skb_hwtstamps(skb)->hwtstamp = ns_to_ktime(ns);

		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += len;

		skb->protocol = eth_type_trans(skb, netdev);

		skb->ip_summed = CHECKSUM_UNNECESSARY;

		grif_port->receive(&grif_port->napi, skb);

		if (unlikely(grif_add_rx_buffer(grif_port, howmany))) {
			netif_err(grif_port, drv, netdev,
					"Failed to add a new descriptor to RX ring\n");
			BUG();
		}

		howmany++;
	}

	return howmany;
}

int grif_rx_poll(struct grif_netdev *grif_port, int budget)
{
	struct net_device *netdev = grif_port->netdev;
	struct sk_buff *skb;
	unsigned int rx_tail, rx_bytes, rx_pkt_num;
	int howmany = 0;
        char ts[TS_SIZE] = {0};
        u64 ns;
        u8 *pkt_data;
	int rx_err;
	u16 data[4] = { 0 };

	/* FIXME:
	 *   Add this error count to sysfs or interface statistics.
	 *
        memcpy_fromio(&data, grif_port->regs_rx + RX_PKT_SIZE_MISMATCH_DBG_CNT_SR*2, 8);
	rx_err = data[0];
	netdev->stats.rx_errors = rx_err;

	if( rx_err ) {
		printk("RX ERR: %d [port %d]\n", rx_err, grif_port->port_num);
	}
	*/

        memcpy_fromio(&data, grif_port->regs_rx + RX_PKT_STATUS_SR*2, 8);
        rx_pkt_num = data[0];

        while (rx_pkt_num && (howmany < budget)) {
		memcpy_fromio(&data,
			      grif_port->regs_rx + RX_PKT_STATUS_WITH_POP_SR*2,
			      8);
		rx_bytes = data[0] >> 3;

                rx_tail = grif_port->rx_tail;
                skb = grif_port->rx_ring[rx_tail].skb;
                grif_port->rx_ring[rx_tail].skb = NULL;

                pkt_data = (u8*)skb->data;
                memcpy_fromio(pkt_data, grif_port->regs_rx + RX_PKT_DATA_SR*2, rx_bytes);
                memcpy(ts, skb->data, TS_SIZE);

                skb_put(skb, rx_bytes);
                skb_pull(skb, TS_SIZE);

                //  ns = 0;
                ns = le64_to_cpup( (__le64*)ts ) << TICKS_NS_SHIFT;

                skb_hwtstamps(skb)->hwtstamp = ns_to_ktime(ns);

                netdev->stats.rx_packets++;
                netdev->stats.rx_bytes += rx_bytes;

                skb->protocol = eth_type_trans(skb, netdev);

                skb->ip_summed = CHECKSUM_UNNECESSARY;
                //skb->ip_summed = CHECKSUM_NONE;

		grif_port->receive(&grif_port->napi, skb);

                grif_port->rx_tail = (grif_port->rx_tail + 1) % RX_RING_DEPTH;

                if (unlikely(grif_add_rx_buffer(grif_port, howmany))) {
                        netif_err(grif_port, drv, netdev,
                               "Failed to add a new descriptor to RX ring\n");
			BUG();
                }

                howmany++;
		rx_pkt_num--;

		if (!rx_pkt_num && (howmany < budget)) {
			memcpy_fromio(&data,
				      grif_port->regs_rx + RX_PKT_STATUS_SR*2,
				      8);
			rx_pkt_num = data[0];
		}
        }

        return howmany;
}

int grif_net_poll(struct napi_struct *napi, int budget)
{
	struct grif_netdev *grif_port = container_of(napi, struct grif_netdev, napi);
        unsigned int rx_done;

	rx_done = grif_port->rx_poll(grif_port, budget);

	/* If all packets were processed, turn on RX IRQs. */
	if (rx_done < budget) {
		napi_complete(napi);

                // RX IRQ EN
		if (rx_queue_len(grif_port) == 0)
			write_rx_reg(grif_port, RX_IRQ_EN_CR, 1);
        }

	return rx_done;
}

static irqreturn_t grif_isr_tx(int irq, void *dev_id)
{
	struct grif_netdev *grif_port = dev_id;
	int tx_buff_overflow;
	u16 data[4] = { 0 };

        memcpy_fromio(&data, grif_port->regs_tx + TX_BUFF_STAT_SR*2, 8);
	tx_buff_overflow = data[0];

	// First time FPGA will rise TX interrupt immediately after turning on.
	if (grif_port->tx_is_stopped == -1 ) {
		grif_port->tx_is_stopped = 0;
		return IRQ_HANDLED;
	}

	if (grif_port->tx_is_stopped) {
		grif_port->tx_is_stopped = 0;

		if (likely(netif_queue_stopped(grif_port->netdev)))
			netif_wake_queue(grif_port->netdev);
		else
			// Very strange!
			printk("TX IRQ: %d [port %d] -- buffer overflow status is different\n", grif_port->tx_irq, grif_port->port_num);
	} else {
		if(tx_buff_overflow) {
			grif_port->tx_is_stopped = 1;
			netif_stop_queue(grif_port->netdev);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t grif_dma_isr(int irq, void *dev_id)
{
	struct grif_netdev *grif_port = dev_id;

	write_rx_reg(grif_port, RX_IRQ_EN_CR, 0);
	dma_copy_schedule(grif_port);

	return IRQ_HANDLED;
}

static irqreturn_t grif_isr(int irq, void *dev_id)
{
	struct grif_netdev *grif_port = dev_id;

	/* Turn off RX IRQ and enable NAPI */
	if (likely(napi_schedule_prep(&grif_port->napi))) {
                // RX IRQ DIS
                write_rx_reg(grif_port, RX_IRQ_EN_CR, 0);
		__napi_schedule(&grif_port->napi);
	}

	return IRQ_HANDLED;
}


static unsigned char *skb_data_bounce(struct dma_buff *tx_ring_buff,
				      struct sk_buff *skb)
{
	skb_frag_t *frag;
	size_t i, offset = skb_headlen(skb);
	unsigned char *skb_data = tx_ring_buff->skb_data;

	memcpy(skb_data, skb->data, skb_headlen(skb));

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		frag = &skb_shinfo(skb)->frags[i];

		memcpy(skb_data + offset,
		       (u8*)page_address(skb_frag_page(frag)) +
		       frag->bv_offset,
		       skb_frag_size(frag));

		offset += skb_frag_size(frag);
	}

	return skb_data;
}

// Write packet data to FPGA memory.
// XXX:
//   Driver must use transactions in multiples of 32 or 4 bytes.
//   Otherwise transaction with only 1 valid byte will be generated on EIM bus.
//   But our EIM bus does not support byte enable signal.
static inline void write_tx_pkt(struct grif_netdev *grif_port, u8 *data, u16 len)
{
	u16 len_roundup = ((len + 3) / 4) * 4;

	memcpy_toio(grif_port->regs_tx + TX_WR_DATA_W0*2, &data[0], len_roundup);
}

static netdev_tx_t grif_net_start_xmit(struct sk_buff *skb,
				   struct net_device *netdev)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);
	u16 timestamp_need = 0;
	unsigned char *skb_data = skb->data;
	unsigned int tx_tail = grif_port->tx_tail;
	unsigned int tx_head = grif_port->tx_head;
	unsigned long irq_flags;

	if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		spin_lock_irqsave(&grif_port->tx_hwtstamp_lock, irq_flags);
		if (!grif_port->tx_hwtstamp_skb) {
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			grif_port->tx_hwtstamp_skb = skb_get(skb);
			grif_port->tx_hwtstamp_start = jiffies;
			timestamp_need = 1;
		} else {
			skb_tx_timestamp(skb);
		}
		spin_unlock_irqrestore(&grif_port->tx_hwtstamp_lock, irq_flags);

		if (timestamp_need)
			schedule_work(&grif_port->tx_hwtstamp_work);
	}

	if (skb_shinfo(skb)->nr_frags) {
		skb_data = skb_data_bounce(&grif_port->tx_ring[tx_tail], skb);
		/*
		 * TODO: there is no need of tx_tail and tx_head here since we
		 * have no queue, but we should have a queue.
		 */
		grif_port->tx_tail = (tx_tail + 1) % TX_RING_DEPTH;
		grif_port->tx_head = (tx_head + 1) % TX_RING_DEPTH;
	}

	write_tx_pkt(grif_port, skb_data, skb->len);
        write_tx_reg(grif_port, TX_TIMESTAMP_SAVE_CR, timestamp_need);
        write_tx_reg(grif_port, TX_PKT_SIZE_CR, skb->len);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;

	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
}

static inline size_t tx_queue_len(struct grif_netdev *grif_port)
{
	unsigned int tx_head = grif_port->tx_head;
	unsigned int tx_tail = grif_port->tx_tail;

	if (tx_head > tx_tail)
		return (TX_RING_DEPTH + tx_tail - tx_head);
	else
		return (tx_tail - tx_head);
}

static void tx_callback(void *data)
{
	struct grif_netdev *grif_port = data;
	struct net_device  *netdev = grif_port->netdev;
	struct device	   *dev	= &grif_port->pdev->dev;
	size_t		tx_head	= grif_port->tx_head;
	struct dma_buff	*tx_buff = &grif_port->tx_ring[tx_head];
	dma_addr_t	dma_addr = tx_buff->dma_addr;
	size_t		pkt_len	= tx_buff->len;
	size_t		dma_len	= ((pkt_len + 3) / 4) * 4;
	u16 timestamp_need	= tx_buff->tx_ts_need;

	if (tx_head == grif_port->tx_tail)
		BUG();

	write_tx_reg(grif_port, TX_TIMESTAMP_SAVE_CR, timestamp_need);
	write_tx_reg(grif_port, TX_PKT_SIZE_CR, pkt_len);

	dma_unmap_single(dev, dma_addr, dma_len, DMA_TO_DEVICE);

	grif_port->tx_head = (tx_head + 1) % TX_RING_DEPTH;

	if (unlikely(netif_queue_stopped(netdev))) {
		grif_port->tx_is_stopped = 0;
		netif_wake_queue(netdev);
	}
}

netdev_tx_t grif_net_start_dma_xmit(struct sk_buff *skb,
				    struct net_device *netdev)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);
	u16 timestamp_need = 0;
	struct device *dev =  &grif_port->pdev->dev;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_addr_t dma_addr;
	size_t dma_len = ((skb->len + 3) / 4) * 4; /* round up to multiple
						    * of 4 bytes */
	unsigned char *skb_data;
	unsigned int tx_tail = grif_port->tx_tail;
	unsigned long irq_flags;

	// TODO: this is common part with grif_net_start_xmit()
	if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		spin_lock_irqsave(&grif_port->tx_hwtstamp_lock, irq_flags);
		if (!grif_port->tx_hwtstamp_skb) {
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			grif_port->tx_hwtstamp_skb = skb_get(skb);
			grif_port->tx_hwtstamp_start = jiffies;
			timestamp_need = 1;
		} else {
			skb_tx_timestamp(skb);
		}
		spin_unlock_irqrestore(&grif_port->tx_hwtstamp_lock, irq_flags);

		if (timestamp_need)
			schedule_work(&grif_port->tx_hwtstamp_work);
	}
	// TODO: ^^^

	skb_data = skb_data_bounce(&grif_port->tx_ring[tx_tail], skb);

	dma_addr = dma_map_single(dev, skb_data, dma_len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		netif_err(grif_port, drv, netdev,
			  "Failed to map TX packet data to DMA\n");
		goto mapping_err;
	}

	desc = dmaengine_prep_dma_memcpy(grif_port->dma_tx_chan,
			grif_port->tx_buff, dma_addr, dma_len, 0);
	if (!desc) {
		netif_err(grif_port, drv, netdev,
			  "Failed to prepare TX DMA transaction\n");
		goto prep_err;
	}

	grif_port->tx_ring[tx_tail].dma_addr	= dma_addr;
	grif_port->tx_ring[tx_tail].len		= skb->len;
	grif_port->tx_ring[tx_tail].tx_ts_need	= timestamp_need;

	desc->callback = tx_callback;
	desc->callback_param = grif_port;

	grif_port->tx_tail = (tx_tail + 1) % TX_RING_DEPTH;

	if (tx_queue_len(grif_port) >= (TX_RING_DEPTH - 1)) {
		grif_port->tx_is_stopped = 1;
		netif_stop_queue(netdev);
	}

	dmaengine_submit(desc);
	dma_async_issue_pending(grif_port->dma_tx_chan);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;

	goto out;

prep_err:
	dma_unmap_single(dev, dma_addr, dma_len, DMA_TO_DEVICE);
mapping_err:
	netdev->stats.tx_dropped++;
out:
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/* Read TX hardware timestamp from FPGA.
*  Return:
*    0 -- if timestamp not ready yet.
*    timestamp in ns otherwise. */
static u64 read_tx_timestamp_ns(struct grif_netdev *port)
{
        #define TIMESTAMP_SIZE 5
        u64 ts = 0;
	u16 data[TIMESTAMP_SIZE];
        int tx_timestamp_valid;
        int i;

       //TODO: можно ли меньше 8ми байт?
        memcpy_fromio(&data, port->regs_tx + TX_TIMESTAMP_VALID_SR*2, 8);
        tx_timestamp_valid = data[0];

        if (!tx_timestamp_valid)
                return 0;

       //TODO: можно ли запросить 10 байт, не будет ли проблем с eim?
        memcpy_fromio(&data, port->regs_tx + TX_TIMESTAMP_VAL_W0_SR*2, 10);

        //TODO: но зачем нам 80 бит, если тут всё равно конвертим в 64??
	for (i = 0; i < TIMESTAMP_SIZE; i++) {
		ts = (u64) data[i] << (16*i) | ts;
        }
        
	// Timestamp in FPGA will lock untils we read and reset it
        write_tx_reg(port, TX_TIMESTAMP_RST_STB_CR, 0);
        write_tx_reg(port, TX_TIMESTAMP_RST_STB_CR, 1);

        return (ts << TICKS_NS_SHIFT);
}

/* TX timestamp will appear asynchronously. So we use
 * a work_struct to periodically polling FPGA registers. */
void grif_tx_hwtstamp_work(struct work_struct *work)
{
	struct grif_netdev *grif_port =
                container_of(work, struct grif_netdev, tx_hwtstamp_work);
        struct skb_shared_hwtstamps hwtstamps = { 0 };
	unsigned long irq_flags;
        u64 ns;

	spin_lock_irqsave(&grif_port->tx_hwtstamp_lock, irq_flags);
	if (time_after(jiffies, grif_port->tx_hwtstamp_start + HZ)) {
		netif_err(grif_port, drv, grif_port->netdev,
			  "Timeout in getting TX timestamp\n");
		goto free_skb;
	}

	ns = read_tx_timestamp_ns(grif_port);
	if (!ns) {
		spin_unlock_irqrestore(&grif_port->tx_hwtstamp_lock, irq_flags);
		schedule_work(&grif_port->tx_hwtstamp_work);
		return;
	}

	hwtstamps.hwtstamp = ns_to_ktime(ns);
	skb_tstamp_tx(grif_port->tx_hwtstamp_skb, &hwtstamps);
free_skb:
	dev_kfree_skb_any(grif_port->tx_hwtstamp_skb);
	grif_port->tx_hwtstamp_skb = NULL;
	spin_unlock_irqrestore(&grif_port->tx_hwtstamp_lock, irq_flags);
}

void grif_net_restart_work(struct work_struct *work)
{
	struct grif_netdev *grif_port =
		container_of(work, struct grif_netdev, restart_work);
	struct net_device *netdev = grif_port->netdev;

	netif_tx_lock(netdev);

	netif_wake_queue(netdev);

	netif_tx_unlock(netdev);
}

static struct net_device_stats *grif_net_get_stats(struct net_device *netdev)
{
	return &netdev->stats;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
static void grif_net_get_stats64(struct net_device *netdev,
						struct rtnl_link_stats64 *storage)
#else
static struct rtnl_link_stats64 *grif_net_get_stats64(struct net_device *netdev,
						struct rtnl_link_stats64 *storage)
#endif
{
	storage->rx_packets		= netdev->stats.rx_packets;
	storage->tx_packets		= netdev->stats.tx_packets;
	storage->rx_bytes		= netdev->stats.rx_bytes;
	storage->tx_bytes		= netdev->stats.tx_bytes;
	storage->rx_errors		= netdev->stats.rx_errors;
	storage->tx_errors		= netdev->stats.tx_errors;
	storage->rx_dropped		= netdev->stats.rx_dropped;
	storage->tx_dropped		= netdev->stats.tx_dropped;
	storage->multicast		= netdev->stats.multicast;
	storage->collisions		= netdev->stats.collisions;
	storage->rx_length_errors	= netdev->stats.rx_length_errors;
	storage->rx_over_errors		= netdev->stats.rx_over_errors;
	storage->rx_crc_errors		= netdev->stats.rx_crc_errors;
	storage->rx_frame_errors	= netdev->stats.rx_frame_errors;
	storage->rx_fifo_errors		= netdev->stats.rx_fifo_errors;
	storage->rx_missed_errors	= netdev->stats.rx_missed_errors;
	storage->tx_aborted_errors	= netdev->stats.tx_aborted_errors;
	storage->tx_carrier_errors	= netdev->stats.tx_carrier_errors;
	storage->tx_fifo_errors		= netdev->stats.tx_fifo_errors;
	storage->tx_heartbeat_errors	= netdev->stats.tx_heartbeat_errors;
	storage->tx_window_errors	= netdev->stats.tx_window_errors;
	storage->rx_compressed		= netdev->stats.rx_compressed;
	storage->tx_compressed		= netdev->stats.tx_compressed;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
	return NULL;
#endif
}

static int grif_net_open(struct net_device *netdev)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);
	int err;

	grif_port->rx_head = 0;
	grif_port->rx_tail = 0;
	grif_port->tx_head = 0;
	grif_port->tx_tail = 0;
        grif_port->max_rx_frame_size = 2000;

	// See grif_isr_tx
	grif_port->tx_is_stopped = -1;

	/* Prepare RX ring to receive packets */
	err = grif_init_rx_ring(grif_port);
	if (err) {
		netif_err(grif_port, ifup, netdev,
				"Failed to initialize RX ring\n");
		return err;
	}

	err = grif_init_tx_ring(grif_port);
	if (err) {
		netif_err(grif_port, ifup, netdev,
				"Failed to initialize TX ring\n");
		goto err_clean_rx_ring;
	}

	/* Register Rx interrupt */
	if (grif_port->dma_rx_chan)
		err = request_irq(grif_port->rx_irq, grif_dma_isr,
				IRQF_TRIGGER_HIGH | IRQF_SHARED, DRV_NAME,
				grif_port);
	else
		err = request_irq(grif_port->rx_irq, grif_isr,
				IRQF_TRIGGER_HIGH | IRQF_SHARED, DRV_NAME,
				grif_port);
	if (err) {
		netif_err(grif_port, ifup, netdev,
				"Cannot allocate interrupt %d\n", grif_port->rx_irq);
		goto err_clean_tx_ring;
	}

	err = request_irq(grif_port->tx_irq, grif_isr_tx, IRQF_TRIGGER_RISING |
			IRQF_SHARED, DRV_NAME, grif_port);
	if (err) {
		netif_err(grif_port, ifup, netdev,
				"Cannot allocate interrupt %d\n", grif_port->tx_irq);
		goto err_clean_tx_ring;
	}

	fpga_set_mac_address(grif_port, netdev->dev_addr);

	/* Enable NAPI polling */
	napi_enable(&grif_port->napi);

	/* Start PHYLINK */
	if (grif_port->phylink){
		phylink_start(grif_port->phylink);
	}

        // Enable TX & RX
        write_tx_reg(grif_port, TX_EN_CR, 1);
        write_rx_reg(grif_port, RX_EN_CR, 1);

        // Enable TX & RX IRQs
        write_tx_reg(grif_port, TX_IRQ_EN_CR, 1);
        write_rx_reg(grif_port, RX_IRQ_EN_CR, 1);

	/* Start transmit queue */
	netif_start_queue(netdev);

	return 0;

err_clean_tx_ring:
	grif_clean_tx_ring(grif_port);
err_clean_rx_ring:
	grif_clean_rx_ring(grif_port);
	return err;
}

static int grif_net_stop(struct net_device *netdev)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	netif_tx_disable(netdev);

        // Disable TX & RX
        write_tx_reg(grif_port, TX_EN_CR, 0);
        write_rx_reg(grif_port, RX_EN_CR, 0);

        // Disable TX & RX IRQs
        write_tx_reg(grif_port, TX_IRQ_EN_CR, 0);
        write_rx_reg(grif_port, RX_IRQ_EN_CR, 0);

	if (grif_port->phylink)
	{
		phylink_stop(grif_port->phylink);
	}

	/* Wait here for poll to complete */
	napi_disable(&grif_port->napi);

	/* Free Rx IRQ */
	free_irq(grif_port->rx_irq, grif_port);
	free_irq(grif_port->tx_irq, grif_port);

	grif_clean_rx_ring(grif_port);

	grif_clean_tx_ring(grif_port);

	return 0;
}

static int grif_hwtstamp_set(struct net_device *netdev, struct ifreq *ifr)
{
        struct hwtstamp_config config;

        if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
                return -EFAULT;

        if (config.flags)
                return -EINVAL;

        switch (config.tx_type) {
        case HWTSTAMP_TX_OFF:
                break;
        case HWTSTAMP_TX_ON:
                break;
        default:
                return -ERANGE;
        }

        return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

static int grif_hwtstamp_get(struct net_device *netdev, struct ifreq *ifr)
{
	  struct grif_netdev *grif_port = netdev_priv(netdev);

          return copy_to_user(ifr->ifr_data, &grif_port->hwtstamp_config, sizeof(grif_port->hwtstamp_config)) ? -EFAULT : 0;
}

static int grif_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);
	int rc = -EOPNOTSUPP;

	switch (cmd) {
                case SIOCSHWTSTAMP:
                        return grif_hwtstamp_set(netdev, ifr);
                case SIOCGHWTSTAMP:
                        return grif_hwtstamp_get(netdev, ifr);

		case SIOCGMIIPHY:
		case SIOCGMIIREG:
		case SIOCSMIIREG:
			if (!grif_port->phylink)
				return -EINVAL;

			rc = phylink_mii_ioctl(grif_port->phylink, ifr, cmd);
			break;
		default:
			break;
	}
	return rc;
}

const struct net_device_ops grif_netdev_ops = {
	.ndo_open	     = grif_net_open,
	.ndo_stop	     = grif_net_stop,
	.ndo_get_stats	     = grif_net_get_stats,
	.ndo_get_stats64     = grif_net_get_stats64,
	.ndo_start_xmit      = grif_net_start_xmit,
	.ndo_change_rx_flags = grif_net_change_rx_flags,
	.ndo_change_mtu      = grif_net_change_mtu,
	.ndo_validate_addr   = eth_validate_addr,
	.ndo_set_mac_address = grif_net_set_mac_address,
	.ndo_do_ioctl        = grif_ioctl,
};
