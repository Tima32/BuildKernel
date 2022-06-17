grif-phc-mux
===========

grif-phc-mux -- driver to control mapping between network ports and PHC clocks.

DESCRIPTION
-----------

Some of Grif-based devices have a pair of network interface cards
(NIC) and PTP clocks associated with them that are implemented in the FPGA.

The application logic requires this infrastructure to work in one of two modes:

* both NICs use the same PTP clock
* each NIC uses its own dedicated PTP clock

This driver introduces these operation modes without significantly impacting NIC
and PTP clock drivers. It exports functions that allow:

* the network driver to determine the index of the PTP clock that should be used
    in the current mode;
* the PTP clock driver to notify about a new clock, specifying its index.

This driver identifies PTP clocks and NICs by the `port-num` device tree
property.

Depending on mode selected by userspace, this driver chooses the right mapping
between PTP clocks and NICs and sets the mode of hardware multiplexer by
writing to a register.

USAGE
-----

This module provides driver which handles platform devices with `compatible`
property being `"stcmtk,phc-mux"`. An example DT for such a device is:

```dts
grif-phc-mux {
	compatible = "stcmtk,phc-mux";
        target-fpga = <&fpga0>;
};
```

Once the driver is registered within the kernel and starts handling such device
it creates file named `separate_phc_mode` in the sysfs directory of its device.
Such a directory is usually located at `/sys/devices/soc0/<my-device-name>`.
One can set the contents of this file to:

* `1` -- switch the driver to mode with separate PTP clock being used by each NIC
* `0` -- switch the driver to mode with single PTP clock being shared by the NICs (default).

FILES
-----

* `export.h` -- declarations of functions exported by this module;
* `grif_phc_mux.c` -- the module source code;
* `Makefile` -- Kernel Build System instructions.
