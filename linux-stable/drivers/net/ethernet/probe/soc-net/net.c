#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/phylink.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "main.h"
#include "regs.h"
#include "dma_regs.h"
#include "timestamp_regs.h"

#define ETH_JUMBO_FRAME	9600
#define ETH_JUMBO_MTU	(ETH_JUMBO_FRAME - VLAN_ETH_HLEN - ETH_FCS_LEN)
#define	TS_SIZE	16


static int et_fpga_write_rx_desc(struct et_netdev *et_port,
				 dma_addr_t dma_addr)
{
	if (fpga_dma_reg_test_bit(et_port, NETDMA_STATUS,
				  STAT_RX_DESC_BUFFER_FULL))
		return -ENOMEM;

	fpga_write_dma_reg(et_port, NETDMA_RX_DESC, dma_addr);

	return 0;
}

static int et_add_rx_buff(struct et_netdev *et_port, size_t pos)
{
	struct sk_buff *skb;
	struct dma_buff *buff;
	dma_addr_t dma_addr;
	struct net_device *netdev = et_port->netdev;
	size_t head = et_port->rx_head;
	size_t next_head = (head + 1) % RX_RING_DEPTH;

	skb = __netdev_alloc_skb(netdev, et_port->data_size,
				 GFP_ATOMIC | GFP_DMA);
	if (unlikely(!skb)) {
		return -ENOMEM;
	}

	dma_addr = dma_map_single(&et_port->pdev->dev, skb->data,
				  et_port->data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&et_port->pdev->dev, dma_addr)) {
		dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	if (et_fpga_write_rx_desc(et_port, dma_addr)) {
		dma_unmap_single(&et_port->pdev->dev, dma_addr,
				 et_port->data_size, DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	buff = &et_port->rx_ring[head];
	buff->skb = skb;
	buff->dma_addr = dma_addr;
	et_port->rx_head = next_head;

	return 0;
}

static void et_fpga_set_mac(struct et_netdev *et_port, const u8 *mac_addr)
{
	fpga_write_nic_reg(et_port, HOST_MAC_W0_CR + 2,
			   (mac_addr[5] << 8) | mac_addr[4]);
	fpga_write_nic_reg(et_port, HOST_MAC_W0_CR + 1,
			   (mac_addr[3] << 8) | mac_addr[2]);
	fpga_write_nic_reg(et_port, HOST_MAC_W0_CR + 0,
			   (mac_addr[1] << 8) | mac_addr[0]);
}

static void et_fpga_enable_rx_irq(struct et_netdev *et_port)
{
	fpga_dma_reg_set_bit(et_port, NETDMA_CONTROL, CTRL_RX_IRQ_ENABLE);
}

static void et_fpga_disable_rx_irq(struct et_netdev *et_port)
{
	fpga_dma_reg_clear_bit(et_port, NETDMA_CONTROL, CTRL_RX_IRQ_ENABLE);
}

static bool et_fpga_rx_is_empty(struct et_netdev *et_port)
{
	return fpga_dma_reg_test_bit(et_port, NETDMA_STATUS,
				     STAT_RX_REPORT_BUFF_EMPTY);
}

static size_t et_fpga_rx_report_len(struct et_netdev *et_port)
{
	u32 report = fpga_read_dma_reg(et_port, NETDMA_RX_REPORT);

	return (report & 0x3fff);
}

static int et_rx_poll(struct et_netdev *et_port, int budget)
{
	struct net_device *netdev = et_port->netdev;
	int howmany = 0;
	size_t rx_tail;
	struct dma_buff *rx_buff;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	size_t dma_len, pkt_len;
	u8 rx_ts[TS_SIZE] = {0};
	u64 rx_ts_ns;

	while (!et_fpga_rx_is_empty(et_port) && howmany < budget) {
		rx_tail = et_port->rx_tail;
		rx_buff = &et_port->rx_ring[rx_tail];
		dma_addr = rx_buff->dma_addr;
		dma_len = et_port->data_size;
		skb = rx_buff->skb;

		dma_unmap_single(&et_port->pdev->dev, dma_addr, dma_len,
				 DMA_FROM_DEVICE);
		rx_buff->skb = NULL;
		rx_buff->dma_addr = 0;

		pkt_len = et_fpga_rx_report_len(et_port) - TS_SIZE;

		memcpy(rx_ts, skb->data, TS_SIZE);
		rx_ts_ns = le64_to_cpup( (__le64*)rx_ts ) << TICKS_NS_SHIFT;

		skb_put(skb, pkt_len + TS_SIZE);
		skb_pull(skb, TS_SIZE);
		skb->protocol = eth_type_trans(skb, netdev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb_hwtstamps(skb)->hwtstamp = ns_to_ktime(rx_ts_ns);

		netif_receive_skb(skb);

		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += pkt_len;
		et_port->rx_tail = (rx_tail + 1) % RX_RING_DEPTH;

		if (unlikely(et_add_rx_buff(et_port, et_port->rx_head))) {
			netif_err(et_port, drv, netdev,
				  "Failed to add buffer to RX ring\n");
			BUG();
		}

		howmany++;
	}

	return howmany;
}

static int et_rx_napi_poll(struct napi_struct *napi, int budget)
{
	struct et_netdev *et_port = container_of(napi, struct et_netdev, rx_napi);
	int rx_done;

	rx_done = et_rx_poll(et_port, budget);

	if (rx_done < budget) {
		napi_complete(napi);
		et_fpga_enable_rx_irq(et_port);
	}

	return rx_done;
}

static irqreturn_t et_rx_isr(int irq, void *data)
{
	struct et_netdev *et_port = data;

	if (likely(napi_schedule_prep(&et_port->rx_napi))) {
		et_fpga_disable_rx_irq(et_port);
		__napi_schedule(&et_port->rx_napi);
	}

	return IRQ_HANDLED;
}

static size_t et_fpga_tx_cnt(struct et_netdev *et_port)
{
	u32 status = fpga_read_dma_reg(et_port, NETDMA_TX_STATUS);

	return (0xFF & (status >> 0));
}

static inline size_t tx_queue_len(struct et_netdev *et_port)
{
	size_t tx_head = et_port->tx_head;
	size_t tx_tail = et_port->tx_tail;

	if (tx_head > tx_tail)
		return (TX_RING_DEPTH + tx_tail - tx_head);
	else
		return (tx_tail - tx_head);
}

static int et_tx_poll(struct et_netdev *et_port, int budget)
{
	struct net_device *netdev = et_port->netdev;
	struct device *dev = &et_port->pdev->dev;
	size_t tx_head;
	struct dma_buff *tx_buff;
	dma_addr_t dma_addr;
	size_t dma_len;
	struct sk_buff *skb;
	size_t howmany = 0;
	size_t tx_done = et_fpga_tx_cnt(et_port);

	if (unlikely(tx_done > tx_queue_len(et_port)))
		BUG();

	while ((tx_done > 0) && (howmany < budget)) {
		tx_head	= et_port->tx_head;
		tx_buff	= &et_port->tx_ring[tx_head];
		dma_addr = tx_buff->dma_addr;
		dma_len	= tx_buff->data_len;
		skb	= tx_buff->skb;

		dma_unmap_single(dev, dma_addr, dma_len, DMA_TO_DEVICE);
		dev_kfree_skb_any(skb);

		tx_buff->dma_addr = 0;
		tx_buff->skb = NULL;
		et_port->tx_head = (tx_head + 1) % TX_RING_DEPTH;
		howmany++;
		tx_done--;

		if (unlikely(netif_queue_stopped(netdev)))
			netif_wake_queue(netdev);

		if ((tx_done == 0) && (howmany < budget) &&
		    (et_port->tx_head != et_port->tx_tail))
			tx_done = et_fpga_tx_cnt(et_port);
	}

	return howmany;
}

static void et_fpga_enable_tx_irq(struct et_netdev *et_port)
{
	fpga_dma_reg_set_bit(et_port, NETDMA_CONTROL, CTRL_TX_IRQ_ENABLE);
}

static void et_fpga_disable_tx_irq(struct et_netdev *et_port)
{
	fpga_dma_reg_clear_bit(et_port, NETDMA_CONTROL, CTRL_TX_IRQ_ENABLE);
}

static int et_tx_napi_poll(struct napi_struct *napi, int budget)
{
	struct et_netdev *et_port = container_of(napi, struct et_netdev, tx_napi);
	int tx_done;

	tx_done = et_tx_poll(et_port, budget);

	if (tx_done < budget) {
		napi_complete(napi);
		et_fpga_enable_tx_irq(et_port);
	}

	return tx_done;
}

static irqreturn_t et_tx_isr(int irq, void *data)
{
	struct et_netdev *et_port = data;

	if (likely(napi_schedule_prep(&et_port->tx_napi))) {
		et_fpga_disable_tx_irq(et_port);
		__napi_schedule(&et_port->tx_napi);
	}

	return IRQ_HANDLED;
}

static void et_tx_ts_work(struct work_struct *work)
{
	struct et_netdev *et_port = container_of(work, struct et_netdev, tx_ts_work);
	struct skb_shared_hwtstamps hwtstamps = { 0 };
	unsigned long irq_flags;
	u64 tx_ts_ns;

	spin_lock_irqsave(&et_port->tx_ts_lock, irq_flags);
	if (time_after(jiffies, et_port->tx_ts_start + HZ)) {
		netif_err(et_port, drv, et_port->netdev,
			  "Timeout in getting TX timestamp\n");
		goto free_skb;
	}

	tx_ts_ns = fpga_read_tx_timestamp(et_port);
	if (!tx_ts_ns) {
		spin_unlock_irqrestore(&et_port->tx_ts_lock, irq_flags);
		schedule_work(&et_port->tx_ts_work);
		return;
	}

	hwtstamps.hwtstamp = ns_to_ktime(tx_ts_ns);
	skb_tstamp_tx(et_port->tx_ts_skb, &hwtstamps);
free_skb:
	dev_kfree_skb_any(et_port->tx_ts_skb);
	et_port->tx_ts_skb = NULL;
	spin_unlock_irqrestore(&et_port->tx_ts_lock, irq_flags);
}

static int et_netdev_set_mtu(struct net_device *netdev, int new_mtu);

/* this function is called on register_netdev() for any late initialization */
static int et_netdev_init(struct net_device *netdev)
{
	const u8* mac_addr;
	struct et_netdev *et_port = netdev_priv(netdev);

	/* initialize MAC address */
	mac_addr = of_get_mac_address(et_port->pdev->dev.of_node);
	if (mac_addr) {
		et_fpga_set_mac(et_port, mac_addr);
		memcpy(netdev->dev_addr, mac_addr, ETH_ALEN);
	}

	netdev->max_mtu = ETH_JUMBO_MTU;
	et_netdev_set_mtu(netdev, netdev->mtu);

	/* initialize NAPI */
#define	NAPI_WEIGHT	64
	netif_napi_add(netdev, &et_port->rx_napi, et_rx_napi_poll,
		       min(RX_RING_DEPTH, NAPI_WEIGHT));
	netif_napi_add(netdev, &et_port->tx_napi, et_tx_napi_poll,
		       min(TX_RING_DEPTH, NAPI_WEIGHT));

	INIT_WORK(&et_port->tx_ts_work, et_tx_ts_work);

	return 0;
}

static void et_clean_rx_ring(struct et_netdev *et_port)
{
	size_t i;
	struct dma_buff *buff;

	for (i = 0; i < RX_RING_DEPTH; i++) {
		buff = &et_port->rx_ring[i];

		if (buff->skb) {
			dma_unmap_single(&et_port->pdev->dev, buff->dma_addr,
					 et_port->data_size, DMA_FROM_DEVICE);

			dev_kfree_skb_any(buff->skb);

			buff->skb = NULL;
			buff->dma_addr = 0;
		}
	}

	et_port->rx_head = 0;
	et_port->rx_tail = 0;
}

static int et_init_rx_ring(struct et_netdev *et_port)
{
	size_t i;
	int err;

	for (i = 0; i < RX_RING_DEPTH; i++) {
		err = et_add_rx_buff(et_port, i);
		if (err) {
			et_clean_rx_ring(et_port);
			return err;
		}
	}

	return 0;
}

static void et_clean_tx_ring(struct et_netdev *et_port)
{
	size_t i;
	struct dma_buff *buff;

	for (i = 0; i < TX_RING_DEPTH; i++) {
		buff = &et_port->tx_ring[i];

		if (buff->skb_data) {
			kfree(buff->skb_data);
			buff->skb_data = NULL;
		}

		if (buff->skb) {
			dma_unmap_single(&et_port->pdev->dev, buff->dma_addr,
					 buff->data_len, DMA_TO_DEVICE);
			buff->dma_addr = 0;

			dev_kfree_skb_any(buff->skb);
			buff->skb = NULL;
		}
	}

	et_port->tx_head = 0;
	et_port->tx_tail = 0;
}

static int et_init_tx_ring(struct et_netdev *et_port)
{
	size_t i;
	struct dma_buff *buff;

	for (i = 0; i < TX_RING_DEPTH; i++) {
		buff = &et_port->tx_ring[i];

		buff->skb = NULL;
		buff->dma_addr = 0;
		buff->skb_data = kzalloc(ETH_JUMBO_FRAME, GFP_KERNEL | GFP_DMA);
		if (!buff->skb_data) {
			et_clean_tx_ring(et_port);
			return -ENOMEM;
		}
	}

	return 0;
}

static void et_fpga_dma_reset(struct et_netdev *et_port)
{
	fpga_dma_reg_set_bit(et_port, NETDMA_CONTROL, CTRL_RESET);
	fpga_dma_reg_clear_bit(et_port, NETDMA_CONTROL, CTRL_RESET);
}

/* this function is called when device goes up */
static int et_netdev_open(struct net_device *netdev)
{
	int err = 0;
	struct et_netdev *et_port = netdev_priv(netdev);

	et_fpga_dma_reset(et_port);

	/* prepare RX ring for receiving packets */
	et_port->rx_head = 0;
	et_port->rx_tail = 0;
	err = et_init_rx_ring(et_port);
	if (err) {
		netdev_err(netdev, "Failed to init RX ring\n");
		goto err;
	}

	/* prepare TX ring for transmiting packets */
	et_port->tx_head = 0;
	et_port->tx_tail = 0;
	err = et_init_tx_ring(et_port);
	if (err) {
		netdev_err(netdev, "Failed to init TX ring\n");
		goto clean_rx;
	}

	/* register RX interrupt */
	err = request_irq(et_port->rx_irq, et_rx_isr,
			  IRQF_SHARED, DRV_NAME,
			  et_port);
	if (err) {
		netdev_err(netdev, "Failed to request RX IRQ\n");
		goto clean_tx;
	}

	/* register TX interrupt */
	err = request_irq(et_port->tx_irq, et_tx_isr,
			  IRQF_SHARED, DRV_NAME,
			  et_port);
	if (err) {
		netdev_err(netdev, "Failed to request TX IRQ\n");
		goto free_rx_irq;
	}

	et_fpga_enable_rx_irq(et_port);
	et_fpga_enable_tx_irq(et_port);

	napi_enable(&et_port->rx_napi);
	napi_enable(&et_port->tx_napi);

	if(et_port->phylink)
		phylink_start(et_port->phylink);

	return 0;

free_rx_irq:
	free_irq(et_port->rx_irq, et_port);
clean_tx:
	et_clean_tx_ring(et_port);
clean_rx:
	et_clean_rx_ring(et_port);
err:
	return err;
}

/* this function is called when device goes down */
static int et_netdev_stop(struct net_device *netdev)
{
	struct et_netdev *et_port = netdev_priv(netdev);

	et_fpga_disable_tx_irq(et_port);
	et_fpga_disable_rx_irq(et_port);

	if(et_port->phylink)
		phylink_stop(et_port->phylink);

	napi_disable(&et_port->rx_napi);
	napi_disable(&et_port->tx_napi);

	et_fpga_dma_reset(et_port);

	/* free interrupts */
	free_irq(et_port->rx_irq, et_port);
	free_irq(et_port->tx_irq, et_port);

	et_clean_rx_ring(et_port);
	et_clean_tx_ring(et_port);

	cancel_work_sync(&et_port->tx_ts_work);

	return 0;
}

static int et_fpga_write_tx_desc(struct et_netdev *et_port,
				 dma_addr_t dma_addr, size_t len,
				 bool tx_ts_need)
{
	if (fpga_dma_reg_test_bit(et_port, NETDMA_STATUS,
				  STAT_TX_DESC_BUFFER_FULL))
		return -ENOMEM;

	fpga_write_dma_reg(et_port, NETDMA_TX_DESC, dma_addr);
	fpga_write_dma_reg(et_port, NETDMA_TX_DESC,
			   len | ((size_t)tx_ts_need << TX_DESC_TIMESTAMP_OFFSET));

	return 0;
}

static unsigned char *skb_data_bounce(struct dma_buff *tx_ring_buff,
				      struct sk_buff *skb)
{
	skb_frag_t *frag;
	size_t i, offset = skb_headlen(skb);
	unsigned char *skb_data = tx_ring_buff->skb_data;

	/* copy linear data */
	memcpy(skb_data, skb->data, skb_headlen(skb));

	/* append fragmented data, if present */
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

static netdev_tx_t et_netdev_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct et_netdev *et_port = netdev_priv(netdev);
	struct device *dev = &et_port->pdev->dev;
	size_t tx_tail = et_port->tx_tail;
	unsigned char *skb_data;
	dma_addr_t dma_addr;
	size_t data_len = skb->len;
	bool tx_ts_need = false;
	unsigned long irq_flags;

	/* make shure data will be always DMA aligned */
	skb_data = skb_data_bounce(&et_port->tx_ring[tx_tail], skb);

	dma_addr = dma_map_single(dev, skb_data, data_len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		netif_err(et_port, drv, netdev,
			  "Failed to map TX data to DMA\n");
		goto map_err;
	}

	et_port->tx_ring[tx_tail].data_len = data_len;
	et_port->tx_ring[tx_tail].dma_addr = dma_addr;
	et_port->tx_ring[tx_tail].skb = skb;

	et_port->tx_tail = (tx_tail + 1) % (TX_RING_DEPTH);

	if (tx_queue_len(et_port) >= (TX_RING_DEPTH - 1)) {
		netif_stop_queue(netdev);
	}

	if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
		spin_lock_irqsave(&et_port->tx_ts_lock, irq_flags);
		if (!et_port->tx_ts_skb) {
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			et_port->tx_ts_skb = skb_get(skb);
			et_port->tx_ts_start = jiffies;
			tx_ts_need = true;
		} else {
			skb_tx_timestamp(skb);
		}
		spin_unlock_irqrestore(&et_port->tx_ts_lock, irq_flags);

		if (tx_ts_need)
			schedule_work(&et_port->tx_ts_work);
	}

	if (et_fpga_write_tx_desc(et_port, dma_addr, data_len, tx_ts_need)) {
		netif_err(et_port, drv, netdev,
			  "Failed to write TX DMA descriptor to FPGA\n");
		goto write_err;
	}

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;

	goto out;

write_err:
	dma_unmap_single(dev, dma_addr, data_len, DMA_TO_DEVICE);

	if (!netif_queue_stopped(netdev))
		netif_stop_queue(netdev);

	et_port->tx_tail = (tx_tail + TX_RING_DEPTH - 1) % TX_RING_DEPTH;
	et_port->tx_ring[tx_tail].dma_addr = 0;
	et_port->tx_ring[tx_tail].skb = NULL;
map_err:
	netdev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
out:

	return NETDEV_TX_OK;
}

static void et_fpga_set_promisc(struct et_netdev *et_port, bool promisc)
{
	fpga_write_nic_reg(et_port, NIC_MAIN_CR,
		promisc ? NIC_MAIN_CR_PROMISC : NIC_MAIN_CR_PRIMARY_MAC);
}

/* this function is called when multicast or promiscuous is enabled */
static void et_netdev_set_rx_flags(struct net_device *netdev, int flags)
{
	struct et_netdev *et_port = netdev_priv(netdev);

	/* we only care about promiscuous */
	if (flags != IFF_PROMISC)
		return;

	et_fpga_set_promisc(et_port, netdev->flags & IFF_PROMISC);
}

static int et_netdev_set_mac(struct net_device *netdev, void *addr)
{
	int ret;
	struct sockaddr *sock_addr = addr;
	struct et_netdev *et_port = netdev_priv(netdev);

	ret = eth_prepare_mac_addr_change(netdev, addr);
	if (ret < 0)
		return ret;

	et_fpga_set_mac(et_port, sock_addr->sa_data);

	eth_commit_mac_addr_change(netdev, addr);
	return 0;
}

static int et_netdev_check_mac(struct net_device *netdev)
{
	return eth_validate_addr(netdev);
}

static int et_hw_ts_set(struct et_netdev *et_port, struct ifreq *ifr)
{
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	if (config.flags ||
	    (config.tx_type != HWTSTAMP_TX_OFF &&
	     config.tx_type != HWTSTAMP_TX_ON))
		return -EINVAL;

	memcpy(&et_port->hw_ts_config, &config, sizeof(config));

	return 0;
}

static int et_hw_ts_get(struct et_netdev *et_port, struct ifreq *ifr)
{
	if (copy_to_user(ifr->ifr_data, &et_port->hw_ts_config, sizeof(et_port->hw_ts_config)))
		return -EFAULT;

	return 0;
}

static int et_netdev_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	int rc = -EOPNOTSUPP;
	struct et_netdev *et_port = netdev_priv(netdev);

	switch (cmd) {
		case SIOCSHWTSTAMP:
			return et_hw_ts_set(et_port, ifr);
		case SIOCGHWTSTAMP:
			return et_hw_ts_get(et_port, ifr);
		case SIOCGMIIPHY:
		case SIOCGMIIREG:
		case SIOCSMIIREG:
			if (!et_port->phylink)
				return -EINVAL;

			rc = phylink_mii_ioctl(et_port->phylink, ifr, cmd);
			break;
		default:
			break;
	}

	return rc;
}

static void et_fpga_set_mtu(struct et_netdev *et_port, int new_mtu)
{
	fpga_write_nic_reg(et_port, NIC_CTRL_MTU, new_mtu);
}

static int et_netdev_set_mtu(struct net_device *netdev, int new_mtu)
{
	int new_frame_len;
	struct et_netdev *et_port = netdev_priv(netdev);

	if (netif_running(netdev))
		return -EBUSY;

#define	DMA_DATAUNIT_BYTE_SIZE	8
	new_frame_len = VLAN_ETH_HLEN + new_mtu + ETH_FCS_LEN;
	new_frame_len += DMA_DATAUNIT_BYTE_SIZE - (new_frame_len % DMA_DATAUNIT_BYTE_SIZE);

	netdev->mtu = new_mtu;
	et_port->data_size = new_frame_len + TS_SIZE;
	et_fpga_set_mtu(et_port, new_frame_len);

	return 0;
}

const struct net_device_ops et_netdev_ops = {
	.ndo_init	= et_netdev_init,
	.ndo_open	= et_netdev_open,
	.ndo_stop	= et_netdev_stop,
	.ndo_start_xmit	= et_netdev_xmit,
	.ndo_change_rx_flags = et_netdev_set_rx_flags,
	.ndo_set_mac_address = et_netdev_set_mac,
	.ndo_validate_addr = et_netdev_check_mac,
	.ndo_do_ioctl	= et_netdev_ioctl,
	.ndo_change_mtu	= et_netdev_set_mtu,
};
