esp8089
======

ESP8089 Linux driver

v1.9 imported from the Rockchip Linux kernel github repo

Modified to build as a standalone module for SPI devices.

Using:

 modprobe esp8089

Device tree binding exaple:

```
&ecspi3 {
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_ecspi3>;
	clock-frequency = <20000000>;
	status = "okay";
	fsl,spi-num-chipselects = <1>;
	cs-gpios = <0>;

	esp8266: esp8266@0 {
		pinctrl-names = "default";
		pinctrl-0 = <&pinctrl_wifi>;
		compatible = "espressif,esp8266","espressif,esp8089";
		spi-max-frequency = <20000000>;
		reg = <0>;
		enable-gpio = <&gpio1 20 GPIO_ACTIVE_HIGH>;
		int-gpio = <&gpio3 13 GPIO_ACTIVE_HIGH>;
		status = "okay";
	};
};
```
This version does not using CS (SS) signal from SPI master.
