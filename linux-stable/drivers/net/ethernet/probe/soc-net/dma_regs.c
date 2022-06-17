#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "main.h"
#include "dma_regs.h"

static unsigned int dma_base_from_port(struct et_netdev *et_port)
{
	if (et_port->port_num)
		return NETDMA_BASE_PORT1;
	else
		return NETDMA_BASE_PORT0;
}

unsigned int fpga_read_dma_reg(struct et_netdev *et_port, unsigned int reg)
{
	struct stcmtk_common_fpga *fpga = et_port->fpga;
	unsigned int dma_base_reg = dma_base_from_port(et_port);

	return ioread32(((struct etn_fpga*)fpga->fpga_defined_fields)->io_base + dma_base_reg + reg);
}

void fpga_write_dma_reg(struct et_netdev *et_port, unsigned int reg,
			unsigned int value)
{
	struct stcmtk_common_fpga *fpga = et_port->fpga;
	unsigned int dma_base_reg = dma_base_from_port(et_port);

	return iowrite32(value, ((struct etn_fpga*)fpga->fpga_defined_fields)->io_base + dma_base_reg + reg);
}

void fpga_dma_reg_set_bit(struct et_netdev *et_port, unsigned int reg,
			  unsigned int bit)
{
	unsigned int reg_val = fpga_read_dma_reg(et_port, reg);

	reg_val |= bit;
	fpga_write_dma_reg(et_port, reg, reg_val);
}

bool fpga_dma_reg_test_bit(struct et_netdev *et_port, unsigned int reg,
			   unsigned int bit)
{
	unsigned int reg_val = fpga_read_dma_reg(et_port, reg);

	return (reg_val & bit);
}

void fpga_dma_reg_clear_bit(struct et_netdev *et_port, unsigned int reg,
			    unsigned int bit)
{
	unsigned int reg_val = fpga_read_dma_reg(et_port, reg);

	reg_val &= ~bit;
	fpga_write_dma_reg(et_port, reg, reg_val);
}
