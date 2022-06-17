# ecp5-hwmon

Hardware monitor driver for Lattice ECP5.

## Description

Driver for the Lattice ECP5 internal temperature monitor. It implements standard linux **hwmon** subsystem insine */sys/class/hwmon*.

Available attributes:

- name: the name of the sensor. Equals "ecp5_temperature".
- temp1_input: contains temperature value from the ECP5 sensor
- temp1_label: contains sensor's label (configured in device tree)

### Temperature points

Described in Lattice's "Power Consumption and Management for ECP5 and ECP5-5G Devices" technical note (TN1266).
See "Table 4" for actual temperature values.
Also you can read "/sys/kernel/debug/ecp5_temperature_values" file in a system where the driver is successfully loaded.

### Device tree bindings

Example:

```c
	ecp5-hwmon {
		compatible = "lattice,ecp5-hwmon";
		target-fpga = <&fpga0>;
		label = "internal ECP5 temperature";
	};
```

where `target-fpga` is a phandle to an FPGA to which you want to connect the driver. `label` is a string that appears in *temp1_label* attribute.

### FPGA features

The driver fetches "ECP5_TEMP_MON" feature. If there is no the feature in you FPGA firmware, the driver will give up device probing.

## Authors

STC Metrotek System Team <system@metrotek.ru>