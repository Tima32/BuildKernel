#include <linux/ethtool.h>
#include <linux/phylink.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/of_platform.h>

#include "main.h"
#include "regs.h"
#include <grif_phc_mux/export.h>

int match(struct device *dev, void *data)
{
	return 1;
};

static void grif_ethtool_get_drvinfo(struct net_device *netdev,
				     struct ethtool_drvinfo *info)
{
	u16 fw_version = 0;

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));

	snprintf(info->fw_version, sizeof(info->fw_version), "%u.%u.%u-%u",
		 (fw_version >> 12) & 0xF, (fw_version >> 8) & 0xF,
		 (fw_version >>  4) & 0xF, (fw_version >> 0) & 0xF);
}

static u32 grif_ethtool_get_msglevel(struct net_device *netdev)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	return grif_port->msg_enable;
}

static void grif_ethtool_set_msglevel(struct net_device *netdev, uint32_t data)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);
	grif_port->msg_enable = data;
}

static int grif_ethtool_ksettings_get(struct net_device *netdev, struct ethtool_link_ksettings *kset)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	if (!grif_port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_ksettings_get(grif_port->phylink, kset);
}

static int grif_ethtool_ksettings_set(struct net_device *netdev, const struct ethtool_link_ksettings *kset)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	if (!grif_port->phylink)
		return -ENOTSUPP;

	if (grif_port->interface == PHY_INTERFACE_MODE_INTERNAL)
		fpga_set_aneg(grif_port, kset->base.autoneg);

	return phylink_ethtool_ksettings_set(grif_port->phylink, kset);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,17,0)
static int grif_ethtool_get_module_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom, u8 *val)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	if (!grif_port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_get_module_eeprom(grif_port->phylink, eeprom, val);
}

static int grif_ethtool_get_module_info(struct net_device *netdev, struct ethtool_modinfo *info)
{
	struct grif_netdev *grif_port = netdev_priv(netdev);

	if (!grif_port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_get_module_info(grif_port->phylink, info);
}
#endif

static int grif_ethtool_get_ts_info(struct net_device *dev, struct ethtool_ts_info *info)
{
	unsigned int property_len = 4;
	void *prop_ptr = NULL;
	struct device_node *fpga_net_node = NULL;
	struct platform_device *grif_phc_mux_dev = NULL;
	struct grif_netdev *grif_port = netdev_priv(dev);

        info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
                                SOF_TIMESTAMPING_RX_SOFTWARE |
                                SOF_TIMESTAMPING_SOFTWARE |
                                SOF_TIMESTAMPING_TX_HARDWARE |
                                SOF_TIMESTAMPING_RX_HARDWARE |
                                SOF_TIMESTAMPING_RAW_HARDWARE;

	/* When the PHC is available, `get_grif_phc_index` returns its
	 * index. Otherwise it returns -1. Its return value is guaranteed to be
	 * valid for phc_index.
	 */

	// Works only for one multiplexer
	info->phc_index = -1;
	fpga_net_node = dev_of_node(&grif_port->pdev->dev);
	if(fpga_net_node) {
		prop_ptr = of_get_property(fpga_net_node, "phc-mux",
			&property_len);
		if(prop_ptr) {
			grif_phc_mux_dev = of_find_device_by_node(
				of_find_node_by_phandle(be32_to_cpup(prop_ptr)));
			info->phc_index = get_grif_phc_index(
				&grif_phc_mux_dev->dev, grif_port->port_num);
			dev_info(&grif_port->pdev->dev,
				"Phc-mux Device Tree entry found\n");
			dev_info(&grif_port->pdev->dev,
				"phc_index = %d\n", info->phc_index);
		}
		else {
			prop_ptr = of_get_property(fpga_net_node, "phc",
				&property_len);
			if(prop_ptr) {
				if(!be32_to_cpup(of_get_property(
					of_find_node_by_phandle(
					be32_to_cpup(prop_ptr)),
					"stcmtk,port-num", &property_len))) {
					info->phc_index = 0;
				}
				else {
					info->phc_index = 1;
				};
				dev_info(&grif_port->pdev->dev,
					"Phc-mux Device Tree entry not found\n");
				dev_info(&grif_port->pdev->dev,
					"Phc Device Tree entry found\n");
				dev_info(&grif_port->pdev->dev,
					"phc_index = %d\n", info->phc_index);
			}
			else {
				dev_info(&grif_port->pdev->dev,
					"Phc-mux and phc devices not found\n");
			};
		};
	}
	else {
		dev_info(&grif_port->pdev->dev,
			"%s node not found\n", dev->name);
	};

        info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);

        info->rx_filters = BIT(HWTSTAMP_FILTER_ALL) | BIT(HWTSTAMP_FILTER_NONE);

        return 0;
}
const struct ethtool_ops grif_ethtool_ops = {
	.get_drvinfo		= grif_ethtool_get_drvinfo,
	.get_msglevel		= grif_ethtool_get_msglevel,
	.set_msglevel		= grif_ethtool_set_msglevel,
	.get_ts_info		= grif_ethtool_get_ts_info,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= grif_ethtool_ksettings_get,
	.set_link_ksettings	= grif_ethtool_ksettings_set,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,17,0)
	.get_module_eeprom	= grif_ethtool_get_module_eeprom,
	.get_module_info	= grif_ethtool_get_module_info,
#endif
};
