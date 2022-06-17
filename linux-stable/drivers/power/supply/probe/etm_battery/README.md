ETM Battery driver
==================

Synopsis
--------

ETM ADC battery driver.

Description
-----------

The driver provides information about battery's static and dynamic parameters:

- Full charge level
- Maximum and minimum voltage
- Current charge and voltage
- Current capacity in percents
- Charge status (Charging/Discharging)

All the parameters can be founded in */sys/class/power_supply/{bat_name}/* directory.

Driver requires device tree node like:

```plain
/ {
        bat: battery {
                compatible = "simple-battery";
                voltage-min-design-microvolt = <6200000>;
                charge-full-design-microamp-hours = <2600000>;
                constant_charge_voltage_max_microvolt = <8400000>;
        };

        my-cool-battery {
                compatible = "stcmtk,etm-battery";
                monitored-battery = <&bat>;
                io-channels = <&adc1 5>;
                io-channel-names = "voltage";
                charge-detect-voltage-microvolt = <10000000>;
        };
};
```

**battery** - is a "simple-battery" (see kernel documentation: bindings/power/supply/battery.txt).

**io-channels** and **io-channel-names** - are ant attributes, that represent ADC device and it's channel for measuring battery voltage.

**charge-detect-voltage-microvolt** - represents minimal voltage on battery in uV while the charger is connected.

In this case the path to battery parameters in in */sys/class/power_sypply/my-cool-battery*.

**IMPORTANT:** The ADC has to measure battery voltage in real voltage range: from zero to max battery (or AC) voltage level (not from 0 to 3V3 (or 5V) like the most ADCs do). So You have to correct ADC's regulator voltages in DTS, like:

```plain
reg_vref_adc: vref_adc@1 {
        compatible = "regulator-fixed";
        regulator-name = "ADC_REF_STUB";
        /* This regulator is used for ETM battery ADC reference
         * voltage representing. ADC measures battery voltage
         * behind the voltage divider with factor = 5.7. So
         * real battery voltage equals ADC voltage * 5.7, and
         * max ADC voltage is 3.3 * 5.7 = 18.81
         */
        regulator-min-microvolt = <18810000>;
        regulator-max-microvolt = <18810000>;
        regulator-always-on;
};
```

Author
------

STC Metrotek System Team <system@metrotek.ru>

Date
----

Tue, 04 Sep 2018 15:56:32 +0300
