#ifndef ETN_FPGA_MGR_ETN_FPGA__H
#define ETN_FPGA_MGR_ETN_FPGA__H

#include <linux/types.h>

struct stcmtk_common_fpga;

enum ETN_FPGA_STATE {
	ETN_FPGA_STATE_USER_MODE = 0,
	ETN_FPGA_STATE_INVALID ,
};

struct etn_fpga {
	struct fpga_manager *mgr;
	void __iomem *io_base;
	atomic_t refcnt;
};

struct etn_firmware {
	size_t size;
	char *buf;
};

int etn_fpga_state_get(struct stcmtk_common_fpga *etn_fpga);
int etn_fpga_ref_get(struct stcmtk_common_fpga *etn_fpga);
int etn_fpga_ref_put(struct stcmtk_common_fpga *etn_fpga);


#endif // ETN_FPGA_MGR_ETN_FPGA__H
