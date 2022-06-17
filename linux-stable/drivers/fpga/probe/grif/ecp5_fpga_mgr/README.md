# FPGA Manager Driver for Lattice ECP5 FPGA

## Synopsis

FPGA Manager Driver for Lattice ECP5 FPGA programming through SPI interface

## Driver interface

This driver implements standart FPGA Manager callbacks.

```
static const struct fpga_manager_ops ecp5_fpga_ops = {
        .state = ecp5_fpga_ops_state,
        .write_init = ecp5_fpga_ops_write_init,
        .write = ecp5_fpga_ops_write,
        .write_complete = ecp5_fpga_ops_write_complete,
};
```

Additional info about FPGA Manager framework:
https://www.kernel.org/doc/Documentation/fpga/fpga-mgr.txt

FPGA programming can be done from other driver throug FPGA Manager API calls:

```
/* get reference from device node */
struct fpga_manager *mgr = of_fpga_mgr_get(dn);

/* load bitstream via fw layer from filesystem /lib/firmware */
fpga_mgr_firmware_load(mgr, 0, “ecp_sspi_fw.img”);

/* drop reference */
fpga_mgr_put(mgr);
```

## Device tree binding

```
&ecspi1 {
        pinctrl-names = "default";
        pinctrl-0 = <&pinctrl_ecspi1>;
        clock-frequency = <20000000>;
        status = "okay";
        fsl,spi-num-chipselects = <1>;
        /* Actually there is no CS muxed to the pin  because we need to
         * drive CS manually from the Lattice ECP5 FPGA driver
         * see pinmux definition for details
         */
        cs-gpios = <0>;

        ecp5: ecp5@0 {
                pinctrl-names = "default";
                pinctrl-0 = <&pinctrl_ecp5 &pinctrl_ecp5_spi_cs>;
                compatible = "lattice,ecp5-fpga-mgr";
                status = "okay";
                spi-max-frequency = <20000000>;
                reg = <0>;
                cs-gpios = <&gpio3 26 GPIO_ACTIVE_LOW>;
                done-gpios = <&gpio3 21 GPIO_ACTIVE_HIGH>;
                programn-gpios = <&gpio3 23 GPIO_ACTIVE_LOW>;
                initn-gpios = <&gpio3 22 GPIO_ACTIVE_LOW>;
        };
};

```

SCLK has to be provided to the ECP5 while SS (Slave Select) is not asserted
which presents a problem with the hardware SPI driver. Therefore, SS is
exported as a GPIO pin and managed separately.

```
pinctrl_ecspi1: ecspi1grp {
	fsl,pins = <
	     /*
	      * This pin shoud be muxed in a separate group so that we can
	      * use it as a gpio. See below.
	      *
	      * MX6UL_PAD_LCD_DATA21__ECSPI1_SS0          0x000010B0
	      */
		MX6UL_PAD_LCD_DATA20__ECSPI1_SCLK         0x000010B0
		MX6UL_PAD_LCD_DATA22__ECSPI1_MOSI         0x000010B0
		MX6UL_PAD_LCD_DATA23__ECSPI1_MISO         0x000010B0
	>;
};

pinctrl_ecp5_spi_cs: ecp5-spi-cs-grp {
	/*
	 *  Here we mux PAD_UART4_RX_DATA (H17) to GPIO1_IO29 in place
	 *  of ECSPI2_SS0 to be able to drive CS. This is a workaround for
	 *  Lattice ECP5 FPGA Slave SPI programming algo, see Lattice
	 *  Programming Tools User Guide, ch. Slave SPI Embedded.
	 */
	fsl,pins = <
		MX6UL_PAD_LCD_DATA21__GPIO3_IO26          0x000010B0
	>;
};

pinctrl_ecp5: ecp5grp {
	fsl,pins = <
		/* FPGA ECP5 SysConfig */
		MX6UL_PAD_LCD_DATA16__GPIO3_IO21          0x000010B0 /* DONE */
		MX6UL_PAD_LCD_DATA17__GPIO3_IO22          0x000010B0 /* INITN */
		MX6UL_PAD_LCD_DATA18__GPIO3_IO23          0x000010B0 /* PROGRAMN */
	>;
};
```

## Links

FPGA Manager Driver for Lattice iCE40  [https://elixir.bootlin.com/linux/latest/source/drivers/fpga/ice40-spi.c]

