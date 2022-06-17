/** 
 * GRIF network device driver.
 *
 * Uses grif_io driver for FPGA communication.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/sfp.h>

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/phylink.h>

#include <linux/regmap.h>

#include <linux/dmaengine.h>
#include <linux/platform_data/dma-imx.h>

#include "main.h"
#include "net.h"
#include "mdio.h"
#include "regs.h"

extern const struct net_device_ops grif_netdev_ops;
extern const struct ethtool_ops grif_ethtool_ops;

// FIXME
//   Delete global variable.
#define   H2F_ADDR     0x50000000
#define   H2F_LEN      0x20000
void __iomem *regs_base;

static irqreturn_t grif_link_isr(int irq, void *dev_id)
{
	struct grif_netdev *grif_port = dev_id;

	phylink_mac_change(grif_port->phylink, gpiod_get_value(grif_port->link_gpio));

	return IRQ_HANDLED;
}

static void grif_phylink_validate(struct phylink_config *config,
				  unsigned long *supported,
				  struct phylink_link_state *state)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);
	struct net_device *ndev = grif_port->netdev;

	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	netdev_info(ndev, "PHYLINK: Validating link interface %s",
			phy_modes(state->interface));

	/*
	 * Supported MAC interfaces
	 *
	 * There is two use cases and multiple `state->interface` options:
	 *  * interfaces for PCS/PMA in FPGA (example Lattice PCS GbE IP-core)
	 *    - "internal"  static  from dts     when initializing fixed link
	 *    - "1000BASEX" dynamic from phylink when 1000Base-X SFP plugged
	 *    - "SGMII"     dynamic from phylink when SGMII      SFP plugged
	 *  * interfaces for PCS/PMA in external PHY (example Marvell 88E1512)
	 *    - "RGMII"     static  from dts     when initializing external phy
	 *
	 * We don't support SGMII on internal PCS/PMA in FPGA because
	 * of lack of information about current link state, speed,
	 * pause mode (flow-control), etc ...
	 */
	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_INTERNAL &&
	    state->interface != PHY_INTERFACE_MODE_1000BASEX &&
	    state->interface != PHY_INTERFACE_MODE_10GBASER &&
	    !phy_interface_mode_is_rgmii(state->interface)) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return;
	}

	//bitmap_copy(mask, supported, __ETHTOOL_LINK_MODE_MASK_NBITS);

	/* Support Autonegotiation */
	phylink_set(mask, Autoneg);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_NA:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:

		phylink_set_port_modes(mask);

		/* Support 10/100/1000Base-T */
		phylink_set(mask, 10baseT_Full);
		phylink_set(mask, 100baseT_Full);
		phylink_set(mask, 1000baseT_Full);

		/* Support 1000Base-X */
		phylink_set(mask, 1000baseX_Full);

		/* We don't support half-duplex modes */
		phylink_clear(mask, 10baseT_Half);
		phylink_clear(mask, 100baseT_Half);
		phylink_clear(mask, 1000baseT_Half);

		/* We don't support flow control */
		phylink_clear(mask, Pause);
		phylink_clear(mask, Asym_Pause);

		bitmap_and(supported, supported, mask,
			__ETHTOOL_LINK_MODE_MASK_NBITS);
		bitmap_and(state->advertising, state->advertising, mask,
			 __ETHTOOL_LINK_MODE_MASK_NBITS);
		break;
	case PHY_INTERFACE_MODE_INTERNAL:
		if (state->speed == SPEED_1000)
			goto base_x;
		/* fall-through */
	case PHY_INTERFACE_MODE_10GBASER:
		phylink_set(mask, 10000baseKX4_Full);
		phylink_set(mask, 10000baseKR_Full);
		phylink_set(mask, 10000baseR_FEC);
		phylink_set(mask, 10000baseCR_Full);
		phylink_set(mask, 10000baseSR_Full);
		phylink_set(mask, 10000baseLR_Full);
		phylink_set(mask, 10000baseLRM_Full);
		phylink_set(mask, 10000baseER_Full);
		phylink_clear(mask, Pause);
		phylink_clear(mask, Asym_Pause);
		if (state->speed == SPEED_10000)
			goto base_r_out;
		/* fall-through */
	case PHY_INTERFACE_MODE_1000BASEX:
base_x:
		/* Ethtool 4.8 and greater see this bit.
		 * Earlier versions show wrong information. */
		phylink_set(mask, 1000baseX_Full);

base_r_out:
		phylink_set(mask, FIBRE);

		bitmap_copy(supported, mask, __ETHTOOL_LINK_MODE_MASK_NBITS);
		bitmap_copy(state->advertising, mask,
				__ETHTOOL_LINK_MODE_MASK_NBITS);
		break;
	default:
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		break;
	}
}

static void grif_phylink_mac_config(struct phylink_config *config,
				    unsigned int mode,
				    const struct phylink_link_state *state)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);
	struct net_device *ndev = grif_port->netdev;

        netdev_dbg(ndev, "PHYLINK: MAC Link set speed %d request", state->speed);

        fpga_set_port_speed(grif_port, state->speed);

	if (grif_port->interface == PHY_INTERFACE_MODE_INTERNAL)
		fpga_set_aneg(grif_port, state->an_enabled);
}

static void grif_phylink_mac_pcs_get_state(struct phylink_config *config,
					   struct phylink_link_state *state)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);
	struct net_device *ndev = grif_port->netdev;

	int link = 1;
	netdev_dbg(ndev, "PHYLINK: MAC Link state request");

	state->speed = 1000;
	state->duplex = 1;
	state->pause = 0;

	if (grif_port->link_gpio) {
		link = gpiod_get_value(grif_port->link_gpio);
	}

	state->link = (link == 1) ? 1 : 0;
}

static void grif_phylink_mac_link_up(struct phylink_config *config,
				     struct phy_device *phy, unsigned int mode,
				     phy_interface_t interface, int speed,
				     int duplex, bool tx_pause, bool rx_pause)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);
	struct net_device *ndev = grif_port->netdev;

	if (grif_port->link_led_gpio != NULL)
		gpiod_set_value(grif_port->link_led_gpio, 1);

	netdev_err(ndev, "PHYLINK: MAC Link Up request");
}

static void grif_phylink_mac_link_down(struct phylink_config *config,
				       unsigned int mode,
				       phy_interface_t interface)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);
	struct net_device *ndev = grif_port->netdev;

	if (grif_port->link_led_gpio != NULL)
		gpiod_set_value(grif_port->link_led_gpio, 0);

	netdev_err(ndev, "PHYLINK: MAC Link Down request");
}

static void grif_phylink_mac_an_restart(struct phylink_config *config)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);
	struct net_device *ndev = grif_port->netdev;

	netdev_dbg(ndev, "PHYLINK: MAC Link Autonegotiation restart request");
}

static const struct phylink_mac_ops grif_phylink_ops = {
	.validate = grif_phylink_validate,
	.mac_config = grif_phylink_mac_config,
	.mac_link_up = grif_phylink_mac_link_up,
	.mac_link_down = grif_phylink_mac_link_down,
	.mac_pcs_get_state = grif_phylink_mac_pcs_get_state,
	.mac_an_restart = grif_phylink_mac_an_restart,
};

static void grif_phylink_get_fixed_state(struct phylink_config *config,
					 struct phylink_link_state *state)
{
	struct grif_netdev *grif_port = container_of(config, struct grif_netdev,
						     phylink_config);

	if (grif_port->link_gpio)
		state->link = !!gpiod_get_value_cansleep(grif_port->link_gpio);

	state->an_enabled = fpga_get_aneg(grif_port);
}

static int grif_phylink_create(struct grif_netdev *grif_port)
{
	struct net_device *ndev = grif_port->netdev;
	struct device *dev = &grif_port->pdev->dev;
	struct phylink *phylink = NULL;
	int phy_mode;
	int res;

	phy_mode = fwnode_get_phy_mode(dev->fwnode);
	if (phy_mode < 0) {
		dev_err(dev, "Invalid \"phy-mode\"\n");

		return phy_mode;
	}

	grif_port->interface = phy_mode;
	grif_port->phylink_config.dev = &ndev->dev;
	grif_port->phylink_config.type = PHYLINK_NETDEV;
	grif_port->phylink_config.get_fixed_state = grif_phylink_get_fixed_state;

	/* Try to create phylink */
	phylink = phylink_create(&grif_port->phylink_config, dev->fwnode,
				 phy_mode, &grif_phylink_ops);

	if (IS_ERR(phylink)) {
		netdev_err(ndev, "Unable to create phylink\n");
		return PTR_ERR(phylink);
	}

	grif_port->phylink = phylink;

	/* Connecting to PHYLINK */
	res = phylink_of_phy_connect(grif_port->phylink, dev->of_node, 0);
	if (res) {
		netdev_err(ndev, "Unable to connect phy to phylink\n");
		return res;
	}

	return 0;
}

static void grif_dma_exit(struct grif_netdev *grif_port)
{
	if (grif_port->dma_rx_chan)
		dma_release_channel(grif_port->dma_rx_chan);

	if (grif_port->dma_tx_chan)
		dma_release_channel(grif_port->dma_tx_chan);
}

static struct dma_chan *__dma_init_chan(struct grif_netdev *grif_port,
					const char *name)
{
	int ret;
	struct dma_chan *chan = NULL;
	struct device *dev = &grif_port->pdev->dev;
	struct dma_slave_config slave_config = {
		.direction = DMA_MEM_TO_MEM,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES, };

	chan = dma_request_slave_channel(dev, name);
	if (!chan)
		goto err;

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret)
		goto err;

	goto out;

err:
	if (chan) {
		dma_release_channel(chan);
		chan = NULL;
	}
out:
	return chan;
}

static void grif_dma_init(struct grif_netdev *grif_port)
{
	struct device *dev = &grif_port->pdev->dev;

	grif_port->dma_rx_chan = __dma_init_chan(grif_port, "rx");
	if (grif_port->dma_rx_chan) {
		grif_port->rx_poll = grif_rx_dma_poll;
	} else {
		dev_warn(dev, "Cannot use SDMA in RX. Use MMIO instead\n");
		grif_port->rx_poll = grif_rx_poll;
	}

	grif_port->dma_tx_chan = __dma_init_chan(grif_port, "tx");
	if (grif_port->dma_tx_chan) {
		struct net_device *netdev = grif_port->netdev;
		static struct net_device_ops ops;

		ops = grif_netdev_ops;
		ops.ndo_start_xmit = grif_net_start_dma_xmit;

		netdev->netdev_ops = &ops;
	} else
		dev_warn(dev, "Cannot use SDMA in TX. Use MMIO instead\n");
}

static int __netdev_register(struct grif_netdev *grif_port)
{
	grif_net_init_mac_address(grif_port);

	pr_debug("Registering netdev\n");
	if (register_netdev(grif_port->netdev)) {
		pr_err("Cannot register net device for port %d\n", grif_port->port_num);
		return -ENODEV;
	}

	return 0;
}

static void __netdev_unregister(struct grif_netdev *grif_port)
{
	if (!grif_port) return;

	pr_debug("Unregistering netdev\n");
	if (grif_port->netdev) {
		if (netif_device_present(grif_port->netdev)) {
			cancel_work_sync(&grif_port->restart_work);
			if (grif_port->netdev->reg_state == NETREG_REGISTERED)
				unregister_netdev(grif_port->netdev);
			if (grif_port->phylink)
				phylink_destroy(grif_port->phylink);
		}

		cancel_work_sync(&grif_port->tx_hwtstamp_work);
	}
}

static void __netdev_free(struct grif_netdev *grif_port)
{
	if (grif_port) {
		free_netdev(grif_port->netdev);
	}
}

static int grif_register(struct grif_netdev *grif_port)
{
	int ret;

	if (__netdev_register(grif_port)) {
		netdev_err(grif_port->netdev, "Failed to register netdev for GRIF port %d\n", grif_port->port_num);
		return -ENODEV;
	}

	ret = grif_phylink_create(grif_port);
	if (ret) {
		netdev_err(grif_port->netdev, "Cannot create PHYLINK\n");
		goto err;
	}

	return 0;

err:
	__netdev_unregister(grif_port);
	return ret;
}

static void grif_unregister(struct grif_netdev *grif_port)
{
	if (grif_port) {
		if (grif_port->phylink) {
			rtnl_lock();
			phylink_disconnect_phy(grif_port->phylink);
			rtnl_unlock();
		}

		__netdev_unregister(grif_port);
	}
}

static int grif_netdev_remove(struct platform_device *pdev)
{
	struct grif_netdev *grif_port;

	grif_port = platform_get_drvdata(pdev);

	if (grif_port) {
		netdev_dbg(grif_port->netdev, "Removing grif_port %p %d on pdev %p\n",
				grif_port, grif_port->port_num, pdev);

		grif_dma_exit(grif_port);

		grif_unregister(grif_port);
		__netdev_free(grif_port);

		if (grif_port->mdio_bus)
			mdiobus_unregister(grif_port->mdio_bus);
	}
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct grif_netdev *grif_netdev_alloc(struct platform_device *pdev, int port_num)
{
	struct grif_netdev *grif_port = NULL;
	struct net_device *netdev = NULL;
	const u8 *mac_addr;

	netdev = alloc_etherdev(sizeof(struct grif_netdev));
	if (!netdev) {
		dev_err(&pdev->dev, "Cannot allocate netdev\n");
		goto err;
	}
	dev_dbg(&pdev->dev,"Allocated netdev %p on pdev %p\n", netdev, pdev);

	SET_NETDEV_DEV(netdev, &pdev->dev);

	grif_port = netdev_priv(netdev);
	grif_port->netdev = netdev;

	mac_addr = of_get_mac_address(pdev->dev.of_node);
	if (mac_addr)
		memcpy(netdev->dev_addr, mac_addr, ETH_ALEN);

	grif_port->port_num = port_num;

	grif_port->tx_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	grif_port->rx_irq = irq_of_parse_and_map(pdev->dev.of_node, 1);

	netdev->netdev_ops = &grif_netdev_ops;

	netdev->ethtool_ops = &grif_ethtool_ops;

	netif_napi_add(netdev, &grif_port->napi, grif_net_poll, NAPI_WEIGHT);

	INIT_WORK(&grif_port->restart_work, grif_net_restart_work);
        INIT_WORK(&grif_port->tx_hwtstamp_work, grif_tx_hwtstamp_work);

	return grif_port;

err:
	return NULL;
}

static int grif_netdev_probe(struct platform_device *pdev)
{
	struct grif_netdev *grif_port;
	struct stcmtk_common_fpga *grif_fpga;
	struct device_node *np = pdev->dev.of_node, *mdio_node;
	struct property *prop = NULL;
	struct gpio_desc *phy_reset_gpio = NULL;
	struct gpio_desc *link_gpio = NULL;
	int port_num = -1;

	prop = of_find_property(np, "port-num", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "No DeviceTree property for grif_net device");
		return -ENODEV;
	}
	port_num = be32_to_cpup(prop->value);

	grif_port = grif_netdev_alloc(pdev, port_num);
	if (!grif_port)
		goto err_free_netdev;

	grif_fpga = stcmtk_get_fpga(np);
	if (!grif_fpga || IS_ERR_VALUE(grif_fpga)) {
		dev_err(&pdev->dev, "Couldn't access grif fpga manager");
		goto err_free_netdev;
	}

	grif_port->regmap = dev_get_regmap(grif_fpga->dev, NULL);
	if (!grif_port->regmap) {
		dev_err(&pdev->dev, "Couldn't get the register map");
		goto err_free_netdev;
	}

        regs_base = ioremap(H2F_ADDR, H2F_LEN);
        if (!regs_base) {
		printk("Failed to map H2F address space\n");
		goto err_free_netdev;
	}

        if (grif_port->port_num) {
		grif_port->regs_tx = regs_base + 2*0x5000;
		grif_port->regs_rx = regs_base + 2*0x6000;
		grif_port->tx_buff = H2F_ADDR + 2*0x5000 + 2*TX_WR_DATA_W0;
		grif_port->rx_buff = H2F_ADDR + 2*0x6000 + 2*RX_PKT_DATA_SR;
        } else {
		grif_port->regs_tx = regs_base + 2*0x1000;
		grif_port->regs_rx = regs_base + 2*0x2000;
		grif_port->tx_buff = H2F_ADDR + 2*0x1000 + 2*TX_WR_DATA_W0;
		grif_port->rx_buff = H2F_ADDR + 2*0x2000 + 2*RX_PKT_DATA_SR;
        }

	grif_port->nic_feature    = stcmtk_fpga_get_feature(grif_fpga,
							FPGA_FEATURE_NIC_CTRL);
	grif_port->phy_feature    = stcmtk_fpga_get_feature(grif_fpga,
							FPGA_FEATURE_MX_PHY);
	if (!grif_port->nic_feature || !grif_port->phy_feature) {
		dev_err(&pdev->dev, "Couln't get features from the fpga manager\n");
		goto err_free_netdev;
	}


	dev_info(&pdev->dev,
			"Allocated grif_port 0x%p port_num %d on pdev 0x%p",
			grif_port, port_num, pdev);

	grif_port->pdev = pdev;
	platform_set_drvdata(pdev, grif_port);

	spin_lock_init(&grif_port->tx_hwtstamp_lock);

	if (grif_port->nic_feature->version >= 3) {
		fpga_get_max_rx_frame_size(grif_port, &grif_port->max_rx_frame_size);
		fpga_get_max_tx_frame_size(grif_port, &grif_port->max_tx_frame_size);
	}
	else {
		fpga_get_rx_frame_size(grif_port, &grif_port->max_rx_frame_size);
		grif_port->max_tx_frame_size = grif_port->max_rx_frame_size;
	}

	dev_info(&pdev->dev, "FPGA maximal received frame size is %d bytes",
			grif_port->max_rx_frame_size);
	if (grif_port->max_rx_frame_size < VLAN_ETH_FRAME_LEN + ETH_FCS_LEN) {
		dev_err(&pdev->dev,
				"FPGA maximal received frame size %d is invalid",
				grif_port->max_rx_frame_size);
		goto err_free_netdev;
	}

	dev_info(&pdev->dev, "FPGA maximal transmitted frame size is %d bytes",
			grif_port->max_tx_frame_size);
	if (grif_port->max_tx_frame_size < VLAN_ETH_FRAME_LEN + ETH_FCS_LEN) {
		dev_err(&pdev->dev,
				"FPGA maximal transmitted frame size %d is invalid",
				grif_port->max_tx_frame_size);
		goto err_free_netdev;
	}

	/*
	 * ???: Here we set max MTU in respect with VLAN interfaces built
	 * on top of our MAC and relied on our MAC in processing tagged frames
	 */
	grif_port->netdev->max_mtu = grif_port->max_tx_frame_size - ETH_HLEN - VLAN_HLEN - ETH_FCS_LEN;

	/* Try to find phy-reset-gpios in device-tree */
	phy_reset_gpio = devm_gpiod_get(&pdev->dev, "phy-reset", GPIOD_ASIS);
	if (IS_ERR(phy_reset_gpio)){
		/* Using FPGA GPIO to reset external PHY */
		fpga_phy_reset(grif_port);
	}
	else {
		/* TODO: Add support for PHY reset GPIO from CPU */
	}

	/* Try to find link-gpios in device-tree */
	grif_port->link_gpio = NULL;
	link_gpio = devm_gpiod_get(&pdev->dev, "link", GPIOD_ASIS);
	if (!IS_ERR(link_gpio)) {
		int err;
		grif_port->link_gpio = link_gpio;
		grif_port->link_irq = gpiod_to_irq(grif_port->link_gpio);
		err = devm_request_irq(&pdev->dev, grif_port->link_irq,
				grif_link_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				DRV_NAME, grif_port);
		if (err) {
			dev_err(&pdev->dev,
					"Cannot allocate link interrupt %d\n",
					grif_port->link_irq);
			goto err_free_netdev;
		}
	}

	grif_port->link_led_gpio = devm_gpiod_get(&pdev->dev, "link-led",
								GPIOD_OUT_LOW);
	if (IS_ERR(grif_port->link_led_gpio)) {
		dev_warn(&pdev->dev, "GPIO for link LED is not presented\n");
		grif_port->link_led_gpio = NULL;
	}

	/* Try to find "mdio" node in device tree and probe MDIO bus */
	mdio_node = of_get_child_by_name(np, "mdio");
	if (mdio_node) {
		grif_mdio_bus_init(grif_port, mdio_node);
	}

	if (of_get_property(np, "use-gro", NULL))
		grif_port->receive = grif_gro_receive;
	else
		grif_port->receive = grif_receive_skb;

	grif_port->netdev->hw_features = NETIF_F_SG;

	if (of_get_property(np, "use-tx-sg", NULL))
		grif_port->netdev->features |= NETIF_F_SG;

	grif_dma_init(grif_port);

	if (grif_register(grif_port)) {
		dev_err(&pdev->dev,"Cannot register devices\n");
		goto err_free_netdev;
	}

	return 0;

err_free_netdev:
	platform_set_drvdata(pdev, NULL);
	__netdev_free(grif_port);

	return -EINVAL;
}

static const struct of_device_id grif_of_match[] = {
	{ .compatible = "stcmtk,grif-net", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, grif_of_match);

static struct platform_driver grif_net_driver = {
	.probe = grif_netdev_probe,
	.remove = grif_netdev_remove,
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(grif_of_match),
	},
};

static int __init grif_net_init(void)
{
	/*
	 * Don't try to use platform_driver_register here because
	 * it always return 0 even if probe failed, so the module
	 * will always load.
	 */
	return platform_driver_register(&grif_net_driver);
}

static void __exit grif_net_exit(void)
{
	platform_driver_unregister(&grif_net_driver);
}

MODULE_SOFTDEP("pre: grif_fpga_mgr grif_gpio sfp");

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("GRIF platform driver for FPGA network devices");
MODULE_LICENSE("GPL");

module_init(grif_net_init);
module_exit(grif_net_exit);
