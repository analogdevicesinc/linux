.. SPDX-License-Identifier: GPL-2.0-only

Kernel driver simatic-ipc-ec-sensors
====================================

Supported systems:

  * Siemens SIMATIC IPC BX-54A

    Prefix: 'simatic_ipc_ec_hwmon', 'simatic_ipc_ec_<fan name>'

    Addresses: embedded controller (EC) address space, described by ACPI

Authors:

  - Gao Nian <nian.gao@siemens.com>
  - Benedikt Niedermayr <benedikt.niedermayr@siemens.com>

Description
-----------

This driver exposes the temperature sensors and fans managed by the embedded
controller (EC) of Siemens SIMATIC IPCs.

The firmware describes these sensors with vendor specific ACPI devices instead
of following the standard ACPI EC model, so they are not covered by the
generic ACPI drivers.

Only the temperature limits supported by the EC are exposed.

A separate hwmon device is registered for each fan.

Sysfs attributes
----------------

Temperature device (``simatic_ipc_ec_hwmon``):

======================= ==== ==================================================
Name                    Perm Description
======================= ==== ==================================================
``temp[1-*]_input``     RO   Temperature in millidegree Celsius.
``temp[1-*]_min``       RW   Low temperature limit in millidegree Celsius.
``temp[1-*]_max``       RW   High temperature limit in millidegree Celsius.
``temp[1-*]_crit``      RW   Critical temperature limit in millidegree Celsius.
======================= ==== ==================================================

Temperature values and limits are stored by the EC as signed degree Celsius,
so written values are rounded to whole degrees and clamped to the range
-128000 to 127000.

Fan device (``simatic_ipc_ec_<fan name>``):

======================= ==== ==================================================
Name                    Perm Description
======================= ==== ==================================================
fan1_input              RO   Fan speed in RPM.
fan1_label              RO   Fan name as provided by the firmware.
pwm1                    RW   Fan duty level (0-255).
pwm1_enable             RW   Fan control mode:

                             - 0: full speed
                             - 1: manual control via ``pwm1``
                             - 2: automatic control by the EC
======================= ==== ==================================================
