# PROBE FPGA Beeper linux driver

## Synopsis

This module implements beeper using FPGA feature FPGA_FEAT_BEEPER.

## Description

Modules uses device tree node of the following format:
```
/ {
	fpga-beeper {
		compatible = "stcmtk,fpga-beeper";
		target-fpga = <&fpga>;
	};
};
```
Module uses Linux Input API so it's supports utility `beep`.

## Authors

STC Metrotek System Team <system@metrotek.ru>

## Date

Thu, 15 Oct 2020 13:16:36 +0300
