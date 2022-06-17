Grif FPGA Manager
=================

grif_fpga_mgr - linux module provides interface to the FPGA from user space.

Module functions:
  * FPGA features support
  * FPGA flashing
  * Kernel API

FPGA features
-------------

Each FPGA firmware has its own subset of features - things that it can do.
Feature descriptors are located in the zero address of firmware image.

Feature is comprised of:

* name
* description
* type - Feature index in enum for fast lookup (without strncmp)
* cr_base - Base address for feature control registers
* cr_cnt - Amount of feature control registers
* sr_base - Base address for feature status registers
* sr_cnt - Amount of feature status registers
* port_mask - Mask of ports supported by this feature
* version - Version of this feature

IMPORTANT - cr_count (and sr_count) shows amount of registers for single port. If
feature is supported on 2 ports then *real* amount of register will be 
"2 * cr_count".

port_mask set bit indicates that feature support on port corresponding to bit
number. For example port_mask 0x3 = 0b11 means that feature is supported on
ports 0 and 1.

Here are the features with its descriptions (from features.h)

		FPGA_FEATURE_MX_FPGA_ID    = "Features descriptor",
		FPGA_FEATURE_MX_PHY        = "Low-level access to physical periphery",
		FPGA_FEATURE_NIC_CTRL      = "Network interface controller support",
		FPGA_FEATURE_LB            = "Smart Loopback support",
		FPGA_FEATURE_FLOW_GEN      = "Flow generators support",
		FPGA_FEATURE_SIGN_ANLZ     = "Traffic analyzer support",
		FPGA_FEATURE_RMON_STAT     = "Port statistic (RMON) support",
		FPGA_FEATURE_BERT_GEN      = "BER test generator"
		FPGA_FEATURE_BERT_ANLZ     = "BER test analyzer"
		FPGA_FEATURE_TX_CIR        = "Cost income ratio"
		FPGA_FEATURE_TWAMP_CPU     = "TWAMP"
		FPGA_FRATURE_MPT_ANLZ      = "Performance test analyzer"
		FPGA_FEATURE_MPT_RX        = "MPT receiver"
		FPGA_FEATURE_RX_RATE_LIMIT = "RX rate limit"
		FPGA_FEATURE_SFP_CTRL      = "SFP regs read/write"

Each firmware must contain MX_FPGA_ID feature. It is required to obtain other
feature properties. It also contains information on currently used firmware like
name, version, ROM id. These three are shown to userspace as `firmware_info`
sysfs file in the following format:

```
SHA:+9f75640,bldr:d.bychkov,time:07.06.18/18:26 15.15.15-15 0x2
         `             `             `                   `   `- ROM Header version
          `             `             `                   `---- Firmware version
           `             `             `----------------------- Build time and date
            `             `------------------------------------ Builder name
             `------------------------------------------------- Git commit hash
```

When driver is probing or when FPGA is reflashed features as well as firmware
information are reread and reparsed.

It is possible to suppress feature parsing process after fpga flashing during the
probe or after reflashing. To do it just add boolean property named
"no-parse-fpga-features" into the manager's device tree node.

Features are exported to userspace via sysfs as kset. Where every kobject of
that kset contains attributes describing feature. It looks like that:

	/sys/class/fpga_manager/fpga0/device
	 +- ...
	 +- firmware_info
	 +- features
		 +- MX_FPGA_ID
		 +- MX_PHY
		 +- ...
		 +- NIC_CTRL
			  +- description
			  +- type
			  +- port_mask
			  +- cr_base
			  +- cr_cnt
			  +- sr_base
			  +- sr_cnt
			  +- version

Flashing FPGA at module load time
---------------------------------

Typically on each startup the FPGA is flashed. This can be done in various ways
(e.g. runit/udev scripts). To do this modules that hold reference count
described above have to be unloaded and reloaded again after flashing FPGA,
which brings unnecessary delay.

To avoid this, FPGA is flashed every time `grif_fpga_mgr.ko` is loaded. The path
to firmware is in module source code and defaults to "ecp5_sspi_fw.img". The 
path is relative to "/lib/firmware". If the specified file does not exist, 
nothing is done.

Flashing FPGA at module operating time
--------------------------------------

You are able to use character device /dev/fpga for FPGA firmware loading. It is
SoC-like temporary way to reflash FPGA on-the-fly. It will be reworked in the
future.

To reflash FPGA this way You have to know the size of firmware in bytes. Call
this command:

```
IMAGE=/lib/firmware/ecp5_sspi_fw.img; SIZE=`stat -L -c%s ${IMAGE}`; dd if=${IMAGE} of=/dev/fpga bs=${SIZE} count=1
```

Troughput measuring and memory test
-----------------------------------

There is an attribute in sysfs for each fpga /sys/class/fpga/fpgaX/bench
It can be used for test EIM memory consistency and throughput

Example:

```
# Test 32768 bytes from offset 0 with 1000 times averaging
echo "0 32768 1000" > /sys/class/fpga/fpga0/bench
```

Kernel API
----------

The driver uses "regmap" linux framework to access FPGA register space. There
is the way to access FPGA regmap from another linux module:

```
#include "grif_fpga_mgr/grif_fpga.h"
 
struct regmap *regmap = NULL;
 
/* Take a pointer to grif_fpga from grif-fpga-mgr */
struct grif_fpga *g = stcmtk_get_fpga(dev.of_node);
if(!g) {
	    printf("FPGA not present!\n");
	        return -ENODEV;
}
 
/* Take the pointer to regmap. */
regmap = g->regmap;
```

This method supposes that You added pointer to target FPGA device tree node in
your device's node like:

```
my-cool-device {
	compatible = "stcmtk,my-cool-device";
	target-fpga = <&fpga0>;
}
```

Now You can access FPGA register space via "regmap" functions:

* regmap_read(regmap, reg, &data);
* regmap_write(regmap, reg, data);
* regmap_update_bits(regmap, reg, mask, data);

If FPGA is not in operating mode after reflashing, this function will return an
error, 0 otherwise.

You can also use only kernel functions to take regmap:

```
/* Take the pointer to FPGA device tree node */
struct device_node *fpga_node = of_parse_phandle(node, "target-fpga", 0);


/* Take the pointer to FPGA's "struct device"*/
struct platform_device *fpga_dev = of_find_device_by_node(fpga_node);


/* Take the regmap from FPGA's device */
struct regmap *regmap = dev_get_regmap(&fpga_dev->dev, NULL);
```

Device Tree binding
-------------------

Required properties:
- fpga-name: attribute that will be in /sys/class/fpga/fpga0/name.
- ecp5-fpga-mgr: phandle to ECP5 manager node.

Optional properties:
- no-parse-fpga-features: boolean forces driver do not acess fpga memory for feature parsing
- vcc-supply: phandle to main FPGA power regulator.
- vcc-tra-supply: phandle to in-FPGA-transceivers power regulator.
- vcc-aux-supply: phandle to FPGA aux power regulator.

It is not necessary to specify all the suppliers. Dummy regulator will be used
instead of unspecified supplier.

Example:

```
fpga0: fpga@0 {
	compatible = "stcmtk,grif-fpga-mgr";
	fpga-name = "Main FPGA";
	ecp5-fpga-mgr = <&ecp5>;
	vcc-supply = <&reg_fpga_vcc>;
	vcc-tra-supply = <&reg_fpga_vcc_tra>;
	vcc-aux-supply = <&reg_fpga_vcc_aux>;
};
```

See also
--------

Details about regmap are in linux source tree: "include/linux/regmap.h"

