.. SPDX-License-Identifier: GPL-2.0-or-later

Kernel driver acer-aspire14-ec
==============================

Supported chips:

  * Acer Aspire 14 AI 52MT/52M embedded controller

    Prefix: 'aspire14_ec'

    Addresses scanned: none

    Datasheet: unavailable

Author: Ethan Jimmy <duhxu0@gmail.com>

Description
-----------

This driver monitors the fan on the Acer Aspire 14 AI (Aspire A14-52M
and A14-52MT). The embedded controller is an I2C HID device (vendor
0x1025, product 0x174b) that also provides the keyboard.

The device is autodetected on those models. The same product id is used
by other Acer keyboards; those machines stay with hid-generic. On the
Aspire A14-52M and A14-52MT this driver provides both the keyboard and
the fan sensor.

Fan speed is reported in RPM through the standard hwmon attribute
``fan1_input``. The driver does not change fan speed.

Sysfs entries
-------------

======================= ======================================================
fan1_input		Fan speed in RPM
======================= ======================================================
