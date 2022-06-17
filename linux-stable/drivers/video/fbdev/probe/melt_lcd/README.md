# Melt mt-12232a LCD framebuffer driver

## Synopsis

ETM features a monochrome MELT MT-12232A LCD that is connected via a
parallel GPIO bus. There were no ready-made (and similar enough) drivers so
this is developed from scratch, based loosely on the `ETN` driver, `skeletonfb`
and `metronome` drivers.

## Notes on implementation

### Flow

* User Space
  1. An App `mmap`s `/dev/fbX`.
  2. A (16-bit high color emulation) framebuffer is presented to the App which may
now be switching individual pixel states by writing at arbitrary mmapped addresses.
* Kernel Space
  3. The framebuffer is configured to be allocated on read-only memory pages, so that
when the App attempts a write, a custom page fault handler is called. It updates 
the LCD to reflect the state of the framebuffer. This approach is as described
in `Documentation/fb/deferred_io.txt` (also, see the `metronome` driver).
  4. Commands are sent to the LCD through the parallel gpio-based "port"

### Making friends with Qt
Despite whatever it claims in the documentation, Qt doesn't seem to work with
monochrome 1 bpp or even gray scale 8 bpp framebuffers. Therefore, this driver
emulates the least expensive supported format - (high) 16-bit color. Zero means
`off`, and any non-zero value is interpreted as `on` state for a given pixel.

### Device tree node

These are given as an **example**, your values will be different. Also make sure
not to confuse the order of pins in the `data-bus-gpios` section.

```
pinctrl_lcd: mtfbgrp {
    fsl,pins = <
        MX6UL_PAD_GPIO1_IO05__GPIO1_IO05        0x000010B0      /* RDWR */
        MX6UL_PAD_GPIO1_IO08__GPIO1_IO08        0x000010B0      /* A0 */
        MX6UL_PAD_GPIO1_IO09__GPIO1_IO09        0x000010B0      /* DB0 */
        MX6UL_PAD_JTAG_MOD__GPIO1_IO10          0x17099         /* DB1 */
        MX6UL_PAD_JTAG_TMS__GPIO1_IO11          0x17099         /* DB2 */
        MX6UL_PAD_JTAG_TDO__GPIO1_IO12          0x17099         /* DB3 */
        MX6UL_PAD_JTAG_TDI__GPIO1_IO13          0x17099         /* DB4 */
        MX6UL_PAD_UART3_CTS_B__GPIO1_IO26       0x000010B0      /* DB7 */
        MX6UL_PAD_UART3_RTS_B__GPIO1_IO27       0x000010B0      /* RES */
        MX6UL_PAD_UART5_TX_DATA__GPIO1_IO30     0x000010B0      /* CS */
        MX6UL_PAD_UART4_RX_DATA__GPIO1_IO29     0x000010B0      /* DB5 */
        MX6UL_PAD_UART4_TX_DATA__GPIO1_IO28     0x000010B0      /* DB6 */
        MX6UL_PAD_GPIO1_IO00__GPIO1_IO00        0x000010B0      /* ESTROBE */
    >;
};
```
```
lcd {   
    compatible = "melt,mt-12232a";
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_lcd>;
    data-bus-gpios = <&gpio1 9  GPIO_ACTIVE_HIGH   /* DB0   */
                      &gpio1 10 GPIO_ACTIVE_HIGH   /* DB1   */
                      &gpio1 11 GPIO_ACTIVE_HIGH   /* DB2   */
                      &gpio1 12 GPIO_ACTIVE_HIGH   /* DB3   */
                      &gpio1 13 GPIO_ACTIVE_HIGH   /* DB4   */
                      &gpio1 29 GPIO_ACTIVE_HIGH   /* DB5   */
                      &gpio1 28 GPIO_ACTIVE_HIGH   /* DB6   */
                      &gpio1 26 GPIO_ACTIVE_HIGH>; /* DB7   */
    reset-gpios    = <&gpio1 27 GPIO_ACTIVE_LOW>;  /* RESET */
    cs-gpios       = <&gpio1 30 GPIO_ACTIVE_HIGH>; /* CS    */
    estr-gpios     = <&gpio1 0  GPIO_ACTIVE_HIGH>; /* ESTR  */
    rdwr-gpios     = <&gpio1 5  GPIO_ACTIVE_HIGH>; /* RDWR  */
    a0-gpios       = <&gpio1 8  GPIO_ACTIVE_HIGH>; /* A0    */
    no-clear-on-init /* Skip display clearing on device probing */
    status = "okay";
};
```

### Choices and debugging

There are several defines in the header file for the driver that let one choose
a debugging verbosity (only errors / high-level actions / literally _everything_)
and supported features. 

The features are:
* Screen memory - it's possible to read from the screen RAM to recover the current
state of the pixels, but it seems much faster to just write the full framebuffer
on the device. So the reading protocol is implemented but not used.

### Logo generation

During boot the driver will attempt to initialize the LCD and display a logo while
the rest of the system is booting, so that users aren't bored with an empty screen.

The logo file is `logo_linux_clut224.ppm` and it can be obtained by:
1. Drawing the logo in GIMP (picture size 122 x 32)
2. Exporting the logo as PNM (RAW data format)
3. Running `ppmquant 224 logo.ppm` on it to quantize the colors
4. Running `pnmnoraw logo.ppm` on it to convert to the final format

## Miscellaneous documentation supplements

### Paging

These timings aren't specified, but experiments show that filling the screen in order
`CS = 0; page = 0..3; CS = 1; page = 0..3` is way faster than
`CS = 0; page = 0; CS = 1; page = 1; ...`
```
+-------------++-------------+
|   CS = 0    ||   CS = 1    |
|  page = 0   ||             |
+-------------++-------------+
+-------------++-------------+
|             ||             |
|  page = 1   ||             |
+-------------++-------------+
+-------------++-------------+
|             ||             |
|             ||             |
+-------------++-------------+
+-------------++-------------+
|             ||   CS = 1    |
|             ||  page = 3   |
+-------------++-------------+
```

## Links
LCD Documentation [http://www.melt.com.ru/docs/MT-12232A.pdf]
ETN Driver [http://repo.ddg/common/sys/drivers/soc-drivers/tree/master/etn_fb]
sceletonfb [https://elixir.bootlin.com/linux/v4.9/source/drivers/video/fbdev/skeletonfb.c]
metronome [https://elixir.bootlin.com/linux/v4.9/source/drivers/video/fbdev/metronomefb.c]
