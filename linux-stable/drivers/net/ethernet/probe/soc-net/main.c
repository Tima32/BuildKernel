#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/phylink.h>
#include <linux/sfp.h>
#include <linux/rtnetlink.h>
#include <linux/gpio/consumer.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "main.h"
#include "phy.h"
#include "net.h"

extern const struct net_device_ops et_netdev_ops;
extern const struct ethtool_ops et_ethtool_ops;

static struct et_netdev *et_netdev_alloc(struct device *dev,
					 unsigned int port_num)
{
	struct net_device *netdev;
	struct et_netdev *et_port;

	netdev = alloc_etherdev(sizeof(struct et_netdev));
	if (!netdev) {
		dev_err(dev, "Cannot allocate netdev\n");
		return NULL;
	}

	SET_NETDEV_DEV(netdev, dev);

	et_port = netdev_priv(netdev);
	et_port->netdev = netdev;
	et_port->port_num = port_num;

	netdev->netdev_ops = &et_netdev_ops;
	netdev->ethtool_ops = &et_ethtool_ops;

	return et_port;
}

static int et_remove(struct platform_device *pdev)
{
	struct phylink *pl;
	struct et_netdev *et_port = platform_get_drvdata(pdev);

	if (!et_port)
		goto out;

	if (et_port->link_led_gpio)
		devm_gpiod_put(&pdev->dev, et_port->link_led_gpio);

	pl = et_port->phylink;

	if (et_port->netdev) {
		if (et_port->netdev->reg_state == NETREG_REGISTERED)
			unregister_netdev(et_port->netdev);

		free_netdev(et_port->netdev);
		et_port->netdev = NULL;
	}

	if (pl) {
		rtnl_lock();
		phylink_disconnect_phy(pl);
		rtnl_unlock();

		phylink_destroy(pl);
	}

out:
	return 0;
}

static int et_probe(struct platform_device *pdev)
{
	u32 port_num;
	struct et_netdev *et_port;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int err = 0;
	static int probe_retry = 10;

	err = of_property_read_u32(np, "port-num", &port_num);
	if (err) {
		dev_err(dev, "No port number property in DT\n");
		return err;
	}

	et_port = et_netdev_alloc(dev, port_num);
	if (!et_port)
		return -ENOMEM;

	et_port->pdev = pdev;
	platform_set_drvdata(pdev, et_port);

	spin_lock_init(&et_port->tx_ts_lock);

	et_port->tx_irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!et_port->tx_irq) {
		dev_err(dev, "Failed to obtain TX IRQ\n");
		et_remove(pdev);

		return -EFAULT;
	}

	et_port->rx_irq = irq_of_parse_and_map(dev->of_node, 1);
	if (!et_port->rx_irq) {
		dev_err(dev, "Failed to obtain RX IRQ\n");
		et_remove(pdev);

		return -EFAULT;
	}

	et_port->fpga = stcmtk_get_fpga(np);
	if (!et_port->fpga) {
		dev_err(dev, "Failed to get target FPGA\n");
		et_remove(pdev);

		return -ENODEV;
	}

	et_port->nic_feature = stcmtk_fpga_get_feature(et_port->fpga,
						    FPGA_FEATURE_NIC_CTRL);

	err = et_phy_init(et_port);
	if (err) {
		et_remove(pdev);

		/* It might happen that we try to load
		 * before MDIO bus is done with PHYs */
		if ((err == -ENODEV) && (probe_retry > 0)) {
			dev_warn(dev, "No phy to connect, will try later\n");
			probe_retry--;
			msleep(500);

			return -EPROBE_DEFER;
		}

		return err;
	}

	err = register_netdev(et_port->netdev);
	if (err) {
		dev_err(dev, "Failed to register netdev\n");
		et_remove(pdev);

		return err;
	}

	return 0;
}

static const struct of_device_id soc_net_id[] = {
	{ .compatible = "stcmtk,etln-10g" },
	{ .compatible = "stcmtk,etln-1g" },
	{},
};
MODULE_DEVICE_TABLE(of, soc_net_id);

static struct platform_driver soc_net_driver = {
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= soc_net_id,
	},
	.probe	= et_probe,
	.remove	= et_remove,
};

module_platform_driver(soc_net_driver);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("SOC platform driver for FPGA network devices");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
