.. SPDX-License-Identifier: GPL-2.0-or-later

Kernel driver max42500
======================

Supported chips:
  * Analog Devices MAX42500

    Prefix: 'max42500'

    Addresses scanned: I2C 0x28 to 0x2B

    Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/max42500.pdf

Author: Kent Libetario <Kent.Libetario@analog.com>

Description
-----------

This driver supports hardware monitoring of MAX42500 power system with up
to seven voltage monitor inputs. Each input has programmable overvoltage
(OV) and undervoltage (UV) thresholds where two of the inputs support
dynamic voltage scaling (DVS). Additionally, the MAX42500 features a
programmable flexible power sequence recorder that stores timestamps
separately. And also, the MAX42500 has a programmable challenge and
response watchdog with a configurable RESET output.


Usage Notes
-----------

This driver does not auto-detect devices. You will have to instantiate the
devices explicitly. Please see Documentation/i2c/instantiating-devices.rst
for details.

Due to its multi-functionality, the MAX42500 is split into three drivers:
the mfd driver as the main device, and the hwmon and watchdog drivers as
the sub-devices. The mfd driver is a client to the core driver and
consumed by both the hwmon and watchdog drivers.

Optionally, two power management GPIOs are provided by the mfd driver and
consumed only by the hwmon sub-device driver. The pins are fixed-outputs
to control the voltage monitor comparators and trigger the power sequence
timestamp recording of the device. Please see the datasheet for details.

For the two GPIO output pins to be consumed, the device tree will look
like this:

.. code-block::

    i2c {
        #address-cells = <1>;
        #size-cells = <0>;
        poweroff-gpios
        sleepoff-gpios

        hwmon@28 {
            compatible = "adi,max42500";
            reg = <0x28>;
        };
    };

Otherwise, the two pins maybe omitted in the device tree if unused.


Platform data support
---------------------

The Hwmon driver supports standard Hwmon ABI driver and sysfs platform
data. While the watchdog driver supports the standard watchdog ABI driver
and sysfs platform data.


Hwmon Sysfs entries
-------------------

The following attributes are supported. Limits are read-write; all other
attributes are read-only.


Chip
~~~~

======================= =======================================================
chip_pec		 Enable or disable PEC: PECE bit in CONFIG1 register
======================= =======================================================

In
~~

======================= ======================================================
in[1-7]_label		 "VMON[1-7]"
in[1-7]_enable		 Enable or disable voltage monitors: VM1 to VM7 bits of
			 VMON register
in[1-5]_min		 Nominal Voltage set point: VIN1 to VIN5 registers
in[1-5]_lcrit		 IN1-IN5 UV threshold: UV1 to UV5 nibbles of OVUV1 to
			 OVUV5 registers
in[6-7]_lcrit		 IN6-IN7 UV threshold: VINU6 to VINU7 registers
in[1-5]_crit		 IN1-IN5 OV threshold: OV1 to OV5 nibbles of OVUV1 to
			 OVUV5 registers
in[6-7]_crit		 IN6-IN7 OV threshold: VINO6 to VINO7 registers
in[1-7]_reset_history	 Enable or disable reset mapping: IN1 to IN7 bits of
			 RSTMAP register
======================= ======================================================

Power
~~~~~

=============================== ===============================================
power[1-7]_label		  "STATUS[1-7]"
power_enable			  OFF comparator status: STATOFF register
power_lcrit_alarm		  UV comparator status: STATUV register
power_crit_alarm		  OV comparator status: STATOV register
power[1-7]_average_interval_min  Power-Down sequence time-stamp: DTIME1 to
				  DTIME7 registers
power[1-7]_average_interval_max  Power-Up sequence time-stamp: UTIME1 to
				  UTIME7 registers
=============================== ===============================================


Watchdog Sysfs entries
----------------------

======================= =======================================================
start			 Enable the watchdog: WDEN bit of WDCFG2 register
stop			 Disable the watchdog: WDEN bit of WDCFG2 register
ping			 Set the watchdog key to the device: WDKEY register
status			 Watchdog status: WDSTAT register
set_timeout		 Watchdog Clock Divider: WDIV bits of WDCDIV register
set_pretimeout		 First Update Extension: 1UD bit of WDCFG2 register
restart 		 Reset Hold or Active Timeout: RHLD bits of RSTCTRL
			 register
======================= =======================================================

..
