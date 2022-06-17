Grif GPIO driver
=========================

This driver make `gpiochip` device for GPIO_CTL feture in FPGA.

Actually, nodes can have platform-specific meanings, but
now the driver is the next generation of 'grif_gpio' driver for
SFP_CTL feature.

So it has 4 nodes, and the nodes are:
  - TX_SD  -- 0
  - RX_LOS -- 1
  - TX_DIS -- 2
  - TX_FLT -- 3

## Limitations

  - Hardcoded gpios count (must be provided by FPGA using an SR)
  - `grif_fpga_manager` dependency for feature info
  - Only for `port` 0 (no `port_mask` support)

## Device Tree bindings

Example:

```
sfp0_gpio: fpga-sfp0-gpio {
	compatible = "stcmtk,grif-gpio-expander";
	target-fpga = <&fpga0>;
	gpio-controller;
	#gpio-cells = <2>;
	gpio-line-names = "tx-sd", "rx-los", "tx-dis", "tx-flt";
};
```
