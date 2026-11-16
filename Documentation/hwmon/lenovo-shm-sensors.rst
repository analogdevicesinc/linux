.. SPDX-License-Identifier: GPL-2.0-or-later

Kernel driver lenovo-shm-sensors
================================

Supported systems:

  * Lenovo ThinkStation P3 TWR (DMI board name ``106F``)

Author: Jiqi Li <lijq9@lenovo.com>

Description
-----------

This driver supports the fan and temperature sensors that the Nuvoton
NCT6801D embedded controller publishes in one of its shared-memory
windows, as found on some Lenovo systems.

The controller has no hardware monitor logical device: LDN 0Bh is the
ATXPGD/BEEP/DDR4 block, so the eSIO register window used by the nct6683
driver does not exist on this part. The EC firmware writes the sensor
values into a shared-memory window instead, mapped by the platform
firmware and reachable through logical device 0Fh.

The layout inside the window is defined by the platform firmware rather
than by the controller, and therefore differs between machines. The
driver selects a layout by DMI and then checks that the candidate window
holds plausible sensor data before exposing it, so a machine whose
firmware lays the window out differently reports no sensors rather than
incorrect ones.

Sysfs interface
---------------

``/sys/class/hwmon/hwmon[X]``, where ``[X]`` is an arbitrary number
assigned by the kernel.

======================= ======= =====================================
Name                    Perm    Description
======================= ======= =====================================
``fan[1-10]_input``     RO      Fan speed in RPM.
``fan[1-10]_label``     RO      Fan label, when the layout names it.
``temp[1-5]_input``     RO      Temperature in millidegrees Celsius.
``temp[1-5]_label``     RO      Temperature label.
======================= ======= =====================================

A fan channel reading ``0xffff`` and a temperature channel reading
``0xff`` are reported as unavailable rather than as a value. A fan
reading of 0 RPM means the fan is stopped or not connected.

Notes
-----

The ACPI firmware on these systems also defines methods that read the
same window under a mutex. Those methods are gated by a flag in the
BIOS-controlled WMIM region. While the flag is clear the driver reads
the window directly; if it reports that ACPI has the sensor interface
enabled, the driver refuses to bind.
