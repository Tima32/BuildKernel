#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/platform_device.h>
#include <linux/of_mdio.h>
#include "main.h"
#include "regs.h"
#include "mdio.h"

/** 
 * Perform specific MDIO command
 * Support Clause 22 only.
 *
 * @param fpga - FPGA device pointer.
 * @param op - Code operation:
 *               MDIO_OP_WRITE -- write data.
 *               MDIO_OP_READ -- read data.
 * @param data - Data value. Ignored in MDIO_OP_READ.
 * @return error code (negative value) if error,
 *         0 if write operation success,
 *         data if read operating success.
 *         TODO:
 *           Check work when error code returned.
 */
static int grif_mdio_command(struct grif_netdev *port, int op, u16 data)
{
	int sr = stcmtk_get_sr_base_on_port(port->phy_feature, port->port_num);
	int cr = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num);

	if (fpga_fw_wait(port, sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	if (op == MDIO_OP_WRITE)
		fpga_fw_write_reg(port, cr + MDIODATALO_CR, data);

	fpga_fw_write_pos(port, cr + MDIO_CR,
			  MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0,
			  op);

	fpga_fw_clear_bit(port, cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(port, cr + MDIO_CR, MDIO_CR_RUN);

	if (op == MDIO_OP_READ) {
		if (fpga_fw_wait(port, sr + MDIO_SR, MDIO_SR_DATAVAL, 1))
			return -ETIME;

		return fpga_fw_read_reg(port, sr + MDIODATALO_SR );
	}

	return 0;
}


/** 
 * Perform MDIO read command. 
 * Support Clause 22 only.
 *
 * @param bus - MDIO bus pointer.
 * @param mii_id - PHY address.
 * @param regnum - Register number.
 * @return data value if success, error code (negative) otherwise.
 */
static int grif_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
        struct grif_netdev *port = bus->priv;
        int cr = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num);

        // Set PHY address 
        fpga_fw_write_pos(port, cr + MDIOPHYAD_CR,
                          MDIOPHYAD_CR_PHYAD_B4, MDIOPHYAD_CR_PHYAD_B0, mii_id);

        // Set register address
        fpga_fw_write_pos(port, cr + MDIODEVAD_CR,
                          MDIODEVAD_CR_DEVAD_B4, MDIODEVAD_CR_DEVAD_B0, regnum);


        // Read data
        return grif_mdio_command(port, MDIO_OP_READ, 0);
}



/** 
 * Perform MDIO write command. 
 * Support Clause 22 only.
 *
 * @param bus - MDIO bus pointer.
 * @param mii_id - PHY address.
 * @param regnum - Register number.
 * @param data - Data for writing.
 * @return 0 if success, error code otherwise.
 */
static int grif_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 data)
{
        struct grif_netdev *port = bus->priv;
        int cr = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num);
        
        // Set PHY address 
        fpga_fw_write_pos(port, cr + MDIOPHYAD_CR,
                          MDIOPHYAD_CR_PHYAD_B4, MDIOPHYAD_CR_PHYAD_B0, mii_id);

        // Set register address
        fpga_fw_write_pos(port, cr + MDIODEVAD_CR,
                          MDIODEVAD_CR_DEVAD_B4, MDIODEVAD_CR_DEVAD_B0, regnum);

        // Write data
        return grif_mdio_command(port, MDIO_OP_WRITE, data);
}


/** 
 * Perform MDIO reset.
 *
 * @param bus - MDIO bus pointer.
 * @param mii_id - PHY address.
 * @param regnum - Register number.
 * @param data - Data for writing.
 * @return 0 if success, error code otherwise.
 */
static int fpga_mdio_reset(struct mii_bus *bus)
{
        struct grif_netdev *port = bus->priv;
	int cr = stcmtk_get_cr_base_on_port(port->phy_feature, port->port_num);

        // Reset MDIO IP core 
        fpga_fw_set_bit(port, cr + MDIO_CR, MDIO_CR_RST);

        // Set MDIO clock division factor 
        fpga_fw_write_pos(port, cr + MDIODIV_CR,
                          MDIODIV_CR_DIV_B5, MDIODIV_CR_DIV_B0, 50);

        // Return MDIO IP core to its normal operating mode 
        fpga_fw_clear_bit(port, cr + MDIO_CR, MDIO_CR_RST);

        return 0;
}

int grif_mdio_bus_init(struct grif_netdev *grif_port, struct device_node *node)
{
	struct mii_bus *mdio;
	struct device *dev = &grif_port->pdev->dev;

	mdio = devm_mdiobus_alloc(dev);
	if (!mdio)
	{
		pr_err("Cannot allocate mdio_bus\n");
		return -ENOMEM;
	}

	mdio->name = "grif_mdio";
	mdio->read = &grif_mdio_read;
	mdio->write = &grif_mdio_write;
	mdio->reset = &fpga_mdio_reset;
	mdio->priv = grif_port;
	mdio->parent = &grif_port->pdev->dev;

	snprintf(mdio->id, MII_BUS_ID_SIZE, "grif-mdio:%d", grif_port->port_num);

	grif_port->mdio_bus = mdio;
	return devm_of_mdiobus_register(dev, mdio, node);
}
