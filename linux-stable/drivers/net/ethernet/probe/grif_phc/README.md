grif-phc -- PTP Hardware Clock (PHC) driver for Grif-based devices
================================================================

DESCRIPTION
-----------

This driver provides access to PHC clocks implemented in FPGA of Grif-based
devices. It registers a platform driver that handles platform device with
`compatible = "stcmtk,phc"`.

USAGE  
-----------

Example of a Device Tree node that is handled by this driver:

```dts
phc {
	compatible = "stcmtk,phc";
	stcmtk,port-num = <0>;
	target-fpga = <&fpga0>;
};
```

It contains the following properties:

* `compatible` which must equal `"stcmtk,phc"`;
* `stcmtk,port-num` that can be either `<0>` or `<1>` corresponding to the
   port index that is associated with this PTP clock device.
  
  Getting and setting PHC clock value can be done via phc_ctl command.  
  Example:  
  phc_ctl /dev/ptp0 set <value> - set PHC clock value  
  phc_ctl /dev/ptp0 get - get PHC clock value  
