etn_phc -- PTP Hardware Clock (PHC) driver for SoC-based devices
================================================================

DESCRIPTION
-----------

This driver provides access to PHC clocks implemented in FPGA of SoC-based
devices. It registers a platform driver that handles platform device with
`compatible = "etn,phc"`.

Device tree
-----------

Example of a Device Tree node that is handled by this driver:

```dts
phc {
	compatible = "etn,phc";
	stcmtk,port-num = <0>;
	reg = <0xabababa 0xbab>;
	interrupts = <0x0 0x2C 0x1>;
	target-fpga = <&fpga>;
};
```

It contains the following properties:

* `compatible` which must equal `"etn,phc"`;
* `stcmtk,port-num` that can be either `<0>` or `<1>` corresponding to the
    network port index that is associated with this PTP clock device;
* `reg` specifies the range of device registers;
* `interrupts` specifies the interrupt this device has;
* `target-fpga` is a phandle to an "fpga" node which registers we need to read/write.

Refer to the FPGA firmware documentation for the values of these properties.

TODO
----

* Check the order of device registration and resources allocation for bugs (it
    seems to have some).

Authors
-------

STC Metrotek System Team <system@metrotek.ru>
