#ifndef GRIF_NET_NET__H
#define GRIF_NET_NET__H

void grif_net_init_mac_address(struct grif_netdev *grif_port);

void grif_handle_link_change(struct net_device *netdev);
int grif_net_poll(struct napi_struct *napi, int budget);
int grif_rx_poll(struct grif_netdev *grif_port, int budget);
int grif_rx_dma_poll(struct grif_netdev *grif_port, int budget);
void grif_net_restart_work(struct work_struct *work);
void grif_tx_hwtstamp_work(struct work_struct *work);

void grif_receive_skb(struct napi_struct *napi, struct sk_buff *skb);
void grif_gro_receive(struct napi_struct *napi, struct sk_buff *skb);

netdev_tx_t grif_net_start_dma_xmit(struct sk_buff *skb,
				    struct net_device *netdev);

#endif // GRIF_NET_NET__H
