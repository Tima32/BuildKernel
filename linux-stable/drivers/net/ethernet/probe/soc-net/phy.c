#include <linux/phylink.h>
#include <linux/of_net.h>
#include <linux/gpio/consumer.h>

#include "main.h"
#include "regs.h"

/* Validate and update the link configuration. */
static void et_phylink_validate(struct phylink_config *config,
				unsigned long *supported,
				struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	if (state->interface != PHY_INTERFACE_MODE_10GBASER &&
			state->interface != PHY_INTERFACE_MODE_XAUI &&
	    state->interface != PHY_INTERFACE_MODE_SGMII &&
	    state->interface != PHY_INTERFACE_MODE_1000BASEX &&
	    !phy_interface_mode_is_rgmii(state->interface)) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return;
	}

	phylink_set(mask, TP);
	phylink_set(mask, FIBRE);

	switch (state->interface) {
		case PHY_INTERFACE_MODE_10GBASER:
		case PHY_INTERFACE_MODE_XAUI:
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
			/* Fall-through */
		case PHY_INTERFACE_MODE_SGMII:
		case PHY_INTERFACE_MODE_RGMII:
		case PHY_INTERFACE_MODE_RGMII_ID:
		case PHY_INTERFACE_MODE_1000BASEX:
			phylink_set(mask, 1000baseX_Full);
			phylink_set(mask, 1000baseT_Full);
			phylink_set(mask, 100baseT_Full);
			phylink_set(mask, 100baseT_Half);
			phylink_set(mask, 10baseT_Full);
			phylink_set(mask, 10baseT_Half);
			break;
		default:
			bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
			break;
	}

	bitmap_and(supported, supported, mask, __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);

}

static void et_phylink_mac_config(struct phylink_config *config,
				  unsigned int mode,
				  const struct phylink_link_state *state)
{
	struct et_netdev *et_port = container_of(config, struct et_netdev,
						 phylink_config);
	struct fpga_feature *phy;
	struct stcmtk_common_fpga *fpga = et_port->fpga;
	int mac_speed_for_fpga;
	int base_cr;

	phy = stcmtk_fpga_get_feature(fpga, FPGA_FEATURE_MX_PHY);
	base_cr = stcmtk_get_cr_base_on_port(phy, et_port->port_num);

	switch(state->speed) {
		case 10:
			mac_speed_for_fpga = PORT_SPEED_10;
			break;
		case 100:
			mac_speed_for_fpga = PORT_SPEED_100;
			break;
		case 1000:
			mac_speed_for_fpga = PORT_SPEED_1000;
			break;
		case 10000:
			mac_speed_for_fpga = PORT_SPEED_10000;
			break;
		default:
			mac_speed_for_fpga = PORT_SPEED_1000;
			break;
	}

	fpga_fw_write_pos(et_port->fpga, base_cr + PORT_SPEED_CR,
			  PORT_SPEED_CR_RX_SPEED_B3, PORT_SPEED_CR_RX_SPEED_B0,
			  mac_speed_for_fpga);
	fpga_fw_write_pos(et_port->fpga, base_cr + PORT_SPEED_CR,
			  PORT_SPEED_CR_TX_SPEED_B3, PORT_SPEED_CR_TX_SPEED_B0,
			  mac_speed_for_fpga);
}

static void et_phylink_mac_an_restart(struct phylink_config *config)
{
	/* TODO:
	 * Restart 802.3z BaseX autonegotiation. */
}

static void et_phylink_mac_link_down(struct phylink_config *config,
				     unsigned int mode,
				     phy_interface_t interface)
{
	struct et_netdev *et_port = container_of(config, struct et_netdev,
						 phylink_config);

	if (et_port->link_led_gpio)
		gpiod_set_value(et_port->link_led_gpio, 0);
}

static void et_phylink_mac_link_up(struct phylink_config *config,
		struct phy_device *phy, unsigned int mode, phy_interface_t interface,
		int speed, int duplex, bool tx_pause, bool rx_pause)
{
	struct et_netdev *et_port = container_of(config, struct et_netdev,
						 phylink_config);
	int status;

	status = phy_read(phy, MII_BMSR);
	if(!(status & 0x7000)) {
		if (et_port->link_led_gpio)
			gpiod_set_value(et_port->link_led_gpio, 1);
	}
}

static const struct phylink_mac_ops et_phylink_ops = {
	.validate	= et_phylink_validate,
	.mac_config	= et_phylink_mac_config,
	.mac_an_restart	= et_phylink_mac_an_restart,
	.mac_link_down	= et_phylink_mac_link_down,
	.mac_link_up	= et_phylink_mac_link_up,
};

int et_phy_init(struct et_netdev *et_port)
{
	struct phylink *phylink;
	struct device *dev = &et_port->pdev->dev;
	struct device *ndev = &et_port->netdev->dev;
	int phy_mode;
	int err = 0;

	phy_mode = fwnode_get_phy_mode(dev->fwnode);
	if (phy_mode < 0) {
		dev_err(dev, "Invalid \"phy-mode\"\n");

		return phy_mode;
	}

	et_port->phylink_config.dev = ndev;
	et_port->phylink_config.type = PHYLINK_NETDEV;

	phylink = phylink_create(&et_port->phylink_config, dev->fwnode,
				 phy_mode, &et_phylink_ops);
	if (IS_ERR(phylink)) {
		dev_err(dev, "Failed to create phylink\n");

		return PTR_ERR(phylink);
	}

	err = phylink_of_phy_connect(phylink, dev->of_node, 0);
	if (err) {
		dev_err(dev, "Failed to connect phy to phylink\n");
		phylink_destroy(phylink);

		return err;
	}

	et_port->phylink = phylink;

	et_port->link_led_gpio = devm_gpiod_get(dev, "link-led", GPIOD_OUT_LOW);
	if (IS_ERR(et_port->link_led_gpio)) {
		dev_warn(dev, "GPIO for link LED is not present\n");
		et_port->link_led_gpio = NULL;
	}

	return 0;
}
