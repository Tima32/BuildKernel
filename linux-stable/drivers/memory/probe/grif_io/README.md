# Grif FPGA IO access

## Synopsis

This driver provides an access point - a `/dev/fpga_regs` char device to work with
the FPGA address space. 

## Description

### Device tree and `grif-fpga-mgr`

In order to confine all of the EIM address space configuration to one place, a 
`regmap` kernel mechanism is utilized within `grif-fpga-mgr` and `grif-io`. 

When loaded, `grif-io` will use its device tree node containing a `target-fpga`
phandle to locate the FPGA, and request the regmap for this FPGA. Then, instead of
relying on ioread/iowrite (as in `etn-io`) it will only use callbacks provided with
the regmap.

This is an **example** device tree node:
```
&weim {
       <...>
       ranges = <0 0 0x50000000 0x08000000>;

       fpga0: fpga@0,0 {
              compatible = "stcmtk,grif-fpga-mgr";
              <...>
       };

       fpga-io {
              compatible = "stcmtk,grif-io";
              target-fpga = <&fpga0>;
              <...>
       };
};
```

## Links
etn-io [http://repo.ddg/common/sys/drivers/soc-drivers/tree/master/etn_io]
