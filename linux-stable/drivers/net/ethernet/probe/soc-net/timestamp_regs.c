#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "main.h"
#include "timestamp_regs.h"

static unsigned int ts_base_from_port(struct et_netdev *et_port)
{
	if (et_port->port_num)
		return TX_TS_BASE_PORT1;
	else
		return TX_TS_BASE_PORT0;
}

static u32 tx_ts_read_reg(struct et_netdev *et_port, unsigned int reg)
{
	struct stcmtk_common_fpga *fpga = et_port->fpga;
	unsigned int ts_base_reg = ts_base_from_port(et_port);

	return ioread32(((struct etn_fpga*)fpga->fpga_defined_fields)->io_base + ts_base_reg + reg);
}

u64 fpga_read_tx_timestamp(struct et_netdev *et_port)
{
	u32 ts_high;
	u32 ts_mid;
	u32 ts_low;

	ts_high = tx_ts_read_reg(et_port, TX_TS_HIGH);

	if (!(ts_high & TX_TS_VALID_BIT))
		return 0;

	ts_mid = tx_ts_read_reg(et_port, TX_TS_MID);
	ts_low = tx_ts_read_reg(et_port, TX_TS_LOW);

	return (((u64)ts_mid << 32) | ts_low) << TICKS_NS_SHIFT;
}
