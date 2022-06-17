gpio-exporter
=============

Synopsis
--------

The module takes GPIO list from device tree and exports GPIOs to
_/sys/bus/platform/device/gpio-exporter_ directory.

Description
-----------

The module actually takes device tree node with **"linux,gpio-exporter"**
compatibility and applies **"gpio_exporter"** driver to node's platform device.
The node has to contain a number of GPIO descriptions as array in *"exported-gpios"*
property:

```plain
/ {
        gpio-exporter {
                compatible = "linux,gpio-exporter";
                exported-gpios = <&gpio2 7 GPIO_ACTIVE_HIGH>,
                                <&gpio3 10 GPIO_ACTIVE_HIGH>;
                exported-gpio-names = "cpu-grsn","pmic-intb";
        };
};
```

In this example platform device with *"linux,gpio-exporter"* compatibility
contains *"gpios"* property that describes GPIO array (port, pin, mode) and
*"exported-gpio-names"* that contains name for every GPIO.

The module export GPIOs and creates special symlinks to them during exporter
platform device probing.

Symlinks is placed in _/sys/bus/paltform/devices/${exporter_name}/&{symlink_name}_.
For example, GPIO *"cpu-grsn"* is located in
_/sys/bus/platform/devices/gpio-exporter/cpu-grsn_.

Symlinks point to common GPIO parameters like _value_, _direction_, etc.

Additionally You can configure initial gpio directions and values. You can add
*"exported-gpio-directions"* array under gpio names. Direction options is /sys/class/gpio-like.
For example: *exported-gpio-names = "out", "in" "high";*. It is important to specify
as many options as many gpios are used. Available options are:

- out - set direction to output and initial value to 0,
- low - same as out
- high - set direction to output and initial value to 1,
- in - set direction to input (default)

If You don't use directions configuration, gpio-exporter uses default directions
(input).

Author
------

STC Metrotek System Team <system@metrotek.ru>

See also
--------

GPIO sysfs: https://www.kernel.org/doc/Documentation/gpio/sysfs.txt

Date
----

Tue, 07 Aug 2018 13:41:52 +0300
