#include <linux/phy.h>
#include <linux/of_mdio.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "main.h"
#include "regs.h"

struct fpga_reg {
	int cr;
	int sr;
};

struct et_mdio_bus {
	struct stcmtk_common_fpga	*fpga;
	struct 				fpga_reg mdio_base_reg;
	int 				bus_num;
};

static int et_1g_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct et_mdio_bus *bus_data = bus->priv;
	struct stcmtk_common_fpga *fpga = bus_data->fpga;
	struct fpga_reg mdio_base_reg = bus_data->mdio_base_reg;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIOPHYAD_CR,
			  MDIOPHYAD_CR_PHYAD_B4, MDIOPHYAD_CR_PHYAD_B0, addr);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIODEVAD_CR,
			  MDIODEVAD_CR_DEVAD_B4, MDIODEVAD_CR_DEVAD_B0, regnum);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0, MDIO_OP_READ_C22);

	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_DATAVAL, 1))
		return -ETIME;

	return fpga_fw_read_reg(fpga, mdio_base_reg.sr + MDIODATALO_SR);
}

static int et_10g_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	int dev, phy;
	struct et_mdio_bus *bus_data = bus->priv;
	struct stcmtk_common_fpga *fpga = bus_data->fpga;
	struct fpga_reg mdio_base_reg = bus_data->mdio_base_reg;

	/* FIXME: document this magic */
	if (addr & 0x8000)
		dev = (addr >> 5) & 0x1f;
	else
		dev = (regnum >> 16) & 0x1f;

	phy = addr & 0x1f;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIOPHYAD_CR,
			  MDIOPHYAD_CR_PHYAD_B4, MDIOPHYAD_CR_PHYAD_B0, phy);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIODEVAD_CR,
			  MDIODEVAD_CR_DEVAD_B4, MDIODEVAD_CR_DEVAD_B0, dev);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	fpga_fw_write_reg(fpga, mdio_base_reg.cr + MDIODATALO_CR, regnum);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0, MDIO_OP_ADDR);

	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0, MDIO_OP_READ_C45);

	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_DATAVAL, 1))
		return -ETIME;

	return fpga_fw_read_reg(fpga, mdio_base_reg.sr + MDIODATALO_SR);
}

static int et_1g_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct et_mdio_bus *bus_data = bus->priv;
	struct stcmtk_common_fpga *fpga = bus_data->fpga;
	struct fpga_reg mdio_base_reg = bus_data->mdio_base_reg;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIOPHYAD_CR,
			  MDIOPHYAD_CR_PHYAD_B4, MDIOPHYAD_CR_PHYAD_B0, addr);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIODEVAD_CR,
			  MDIODEVAD_CR_DEVAD_B4, MDIODEVAD_CR_DEVAD_B0, regnum);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	fpga_fw_write_reg(fpga, mdio_base_reg.cr + MDIODATALO_CR, val);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0, MDIO_OP_WRITE);


	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_DATAVAL, 1))
		return -ETIME;

	return 0;
}

static int et_10g_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	int dev, phy;
	struct et_mdio_bus *bus_data = bus->priv;
	struct stcmtk_common_fpga *fpga = bus_data->fpga;
	struct fpga_reg mdio_base_reg = bus_data->mdio_base_reg;

	if(addr & 0x8000)
		dev = (addr >> 5) & 0x1f;
	else
		dev = (regnum >> 16) & 0x1f;

	phy = addr & 0x1f;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIOPHYAD_CR,
			  MDIOPHYAD_CR_PHYAD_B4, MDIOPHYAD_CR_PHYAD_B0, phy);
	// Set register address
	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIODEVAD_CR,
			  MDIODEVAD_CR_DEVAD_B4, MDIODEVAD_CR_DEVAD_B0, dev);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0, MDIO_OP_ADDR);

	// put register address
	fpga_fw_write_reg(fpga, mdio_base_reg.cr + MDIODATALO_CR, regnum);

	// start mdio transaction
	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_BUSY, 0))
		return -ETIME;

	// put data
	fpga_fw_write_reg(fpga, mdio_base_reg.cr + MDIODATALO_CR, val);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_COP_B1,
			  MDIO_CR_COP_B0, MDIO_OP_WRITE);

	// start mdio transaction
	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);
	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RUN);

	if (fpga_fw_wait(fpga, mdio_base_reg.sr + MDIO_SR, MDIO_SR_DATAVAL, 1))
		return -ETIME;

	return 0;
}

static int et_mdio_reset(struct mii_bus *bus)
{
	struct et_mdio_bus *bus_data = bus->priv;
	struct stcmtk_common_fpga *fpga = bus_data->fpga;
	struct fpga_reg mdio_base_reg = bus_data->mdio_base_reg;

	fpga_fw_set_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RST);

	fpga_fw_write_pos(fpga, mdio_base_reg.cr + MDIODIV_CR,
			  MDIODIV_CR_DIV_B5, MDIODIV_CR_DIV_B0, 50);

	fpga_fw_clear_bit(fpga, mdio_base_reg.cr + MDIO_CR, MDIO_CR_RST);

	return 0;
}

static int et_init_mdio_regs(struct et_mdio_bus *bus_data)
{
	struct fpga_feature *phy;
	struct stcmtk_common_fpga *fpga = bus_data->fpga;

	phy = stcmtk_fpga_get_feature(fpga, FPGA_FEATURE_MX_PHY);
	if (!phy)
		return -EFAULT;

	bus_data->mdio_base_reg.cr = stcmtk_get_cr_base_on_port(phy, bus_data->bus_num);
	bus_data->mdio_base_reg.sr = stcmtk_get_sr_base_on_port(phy, bus_data->bus_num);

	fpga_fw_clear_bit(fpga, bus_data->mdio_base_reg.cr + TRX_CR,
			  TRX_CR_EG_NRST);
	mdelay(1);
	fpga_fw_set_bit(fpga, bus_data->mdio_base_reg.cr + TRX_CR,
			TRX_CR_EG_NRST);
	mdelay(10);

	return 0;
}

static int et_mdio_probe(struct platform_device *pdev)
{
	int err;
	struct mii_bus *mdio;
	struct et_mdio_bus *et_mdio;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	mdio = mdiobus_alloc_size(sizeof(struct et_mdio_bus));
	if (!mdio) {
		dev_err(dev, "Failed to allocate MDIO bus\n");

		return -ENOMEM;
	}

	et_mdio = mdio->priv;

	if (of_device_is_compatible(np, "stcmtk,et-10g-mdio")) {
		et_mdio->bus_num = 0;
		mdio->read	= &et_10g_mdio_read;
		mdio->write	= &et_10g_mdio_write;
	} else if (of_device_is_compatible(np, "stcmtk,et-1g-mdio")) {
		u32 mdio_num;

		err = of_property_read_u32(np, "mdio-num", &mdio_num);
		if (err) {
			dev_err(dev, "No mdio bus number property in DT\n");
			goto out_err;
		}

		et_mdio->bus_num = mdio_num;
		mdio->read	= &et_1g_mdio_read;
		mdio->write	= &et_1g_mdio_write;
	} else {
		err = -EINVAL;
		dev_err(dev, "Invalid compatible string in DT\n");
		goto out_err;
	}

	mdio->name	= "soc-mdio";
	mdio->reset	= &et_mdio_reset;
	mdio->parent	= dev;

	snprintf(mdio->id, MII_BUS_ID_SIZE, "%s-%d", mdio->name, et_mdio->bus_num);

	et_mdio->fpga = stcmtk_get_fpga(np);
	if (!et_mdio->fpga) {
		dev_err(dev, "Failed to get target FPGA\n");
		err = -ENODEV;

		goto out_err;
	}

	err = et_init_mdio_regs(et_mdio);
	if (err) {
		dev_err(dev, "Failed to init MDIO registers in FPGA\n");

		goto out_err;
	}

	err = of_mdiobus_register(mdio, np);
	if (err) {
		dev_err(dev, "Failed to register MDIO bus\n");

		goto out_err;
	}

	mdio->owner	= THIS_MODULE;
	platform_set_drvdata(pdev, mdio);

	return 0;

out_err:
	mdiobus_free(mdio);

	return err;
}

static int et_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *mdio = platform_get_drvdata(pdev);

	mdiobus_unregister(mdio);
	mdiobus_free(mdio);

	return 0;
}

static const struct of_device_id soc_mdio_id[] = {
	{ .compatible = "stcmtk,et-10g-mdio" },
	{ .compatible = "stcmtk,et-1g-mdio" },
	{},
};
MODULE_DEVICE_TABLE(of, soc_mdio_id);

static struct platform_driver et_mdio_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
		.of_match_table = soc_mdio_id,
	},
	.probe	= et_mdio_probe,
	.remove	= et_mdio_remove,
};

module_platform_driver(et_mdio_driver);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("SOC platform driver for FPGA MDIO bus");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");
