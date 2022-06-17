#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/phylink.h>
#include <linux/net_tstamp.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>
#include <etn_phc_mux/export.h>
#include "main.h"

static void et_ethtool_get_drvinfo(struct net_device *netdev,
				   struct ethtool_drvinfo *info)
{
	struct et_netdev *et_port = netdev_priv(netdev);
	u16 fw_info = et_port->fpga->features->fpga_version;

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));

	snprintf(info->fw_version, sizeof(info->fw_version), "%u.%u.%u-%u",
		 (fw_info >> 12) & 0xF, (fw_info >> 8) & 0xF,
		 (fw_info >> 4) & 0xF, fw_info & 0xF);
}

static int et_ethtool_get_settings(struct net_device *netdev, struct ethtool_link_ksettings *cmd)
{
	struct et_netdev *et_port = netdev_priv(netdev);

	if (!et_port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_ksettings_get(et_port->phylink, cmd);
}

static int et_ethtool_set_settings(struct net_device *netdev,
				   const struct ethtool_link_ksettings *cmd)
{
	struct et_netdev *et_port = netdev_priv(netdev);

	if (!et_port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_ksettings_set(et_port->phylink, cmd);
}

static u32 et_ethtool_get_msglevel(struct net_device *netdev)
{
	struct et_netdev *et_port = netdev_priv(netdev);

	return et_port->msg_enable;
}

static void et_ethtool_set_msglevel(struct net_device *netdev, uint32_t data)
{
	struct et_netdev *et_port = netdev_priv(netdev);
	et_port->msg_enable = data;
}

static int et_ethtool_get_ts_info(struct net_device *netdev,
				  struct ethtool_ts_info *info)
{
	struct et_netdev *et_port = netdev_priv(netdev);

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	if (IS_ENABLED(CONFIG_ETN_PHC_MUX))
		info->phc_index = get_etn_phc_index(et_port->port_num);
	else
		info->phc_index = -1;

	info->tx_types = BIT(HWTSTAMP_TX_ON) | BIT(HWTSTAMP_TX_OFF);
	info->rx_filters = BIT(HWTSTAMP_FILTER_ALL) | BIT(HWTSTAMP_FILTER_NONE);

	return 0;
}

const struct ethtool_ops et_ethtool_ops = {
	.get_drvinfo		= et_ethtool_get_drvinfo,
	.get_msglevel		= et_ethtool_get_msglevel,
	.set_msglevel		= et_ethtool_set_msglevel,
	.get_ts_info		= et_ethtool_get_ts_info,
	.get_link_ksettings	= et_ethtool_get_settings,
	.set_link_ksettings	= et_ethtool_set_settings,
	.get_link		= ethtool_op_get_link,
};
