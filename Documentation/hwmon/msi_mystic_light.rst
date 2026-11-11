.. SPDX-License-Identifier: GPL-2.0-or-later

Kernel driver msi_mystic_light
==============================

Supported devices:

* MSI Mystic Light USB controller (USB ID 1462:7c94) on MSI
  MAG B550M MORTAR MAX WIFI (MS-7C94)

* MSI Mystic Light USB controller (USB ID 1462:7d75) on MSI
  MAG B650 TOMAHAWK WIFI (MS-7D75)

* MSI Mystic Light USB controller (USB ID 1462:7d76) on MSI
  MAG B650M MORTAR WIFI (MS-7D76)

* MSI Mystic Light USB controller (USB ID 0db0:0076) on these MSI
  800-series motherboards:

  - MSI MAG B850M MORTAR WIFI (MS-7E61)

Author: Tomasz Bojanowski <tomasz@bojanowski.me>

Description
-----------

On various MSI series motherboards the RGB LED headers ("Mystic Light") are
driven by a microcontroller attached over USB. The controller also measures
a few voltage rails that are not monitored by the Super I/O chip (see
nct6683). This driver reports these voltages.

The controller has the same USB ID on different boards, while the rails
that are present, and their position in the sensor report, depend on the
motherboard. The driver is therefore only registered on supported boards,
which are identified by DMI. On other boards the controller is left to
hid-generic.

When probing the controller, the driver reads the sensor report once. If
the report cannot be read or does not have the expected format, the driver
does not bind to the device. Only the rails that return a reading are
exposed.

On 500- and 600-series motherboards, the driver requests sensor data with
an output report. Sensor data is subsequently reported by the controller
with a raw event.

On 800-series motherboards, the driver never writes to the controller.
The only request it sends is a read of HID feature report 0xf0.

While the driver is bound, the device is kept open, so its interrupt endpoint
is polled as it is with the vendor software. The hidraw interface remains
available, so user space tools that control the LEDs keep working.


Usage Notes
-----------

The sensor values are cached for one second. If the sensor report cannot
be read later, or does not have the expected format, reading fails with an
error, for example -EPROTO.

Sysfs entries
-------------

=============== =========================================================
in[0-2]_input   Voltage in millivolts
in[0-2]_label   Rail name
=============== =========================================================

MSI MAG B550M MORTAR MAX WIFI (MS-7C94)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

======= ========= ========================================
Channel Label     Typical value
======= ========= ========================================
in0     CPU VDDP  varies (may be 0 on systems with no APU)
in1     PROM Core 1.04 V
======= ========= ========================================

MSI MAG B650 TOMAHAWK WIFI (MS-7D75)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

======= ========= ==========================
Channel Label     Typical value
======= ========= ==========================
in0     CPU 1P8   1.80 V
in1     CHIP Core 1.04 V
in2     VDD MISC  1.10 V
======= ========= ==========================

MSI MAG B650M MORTAR WIFI (MS-7D76)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

======= ========= ==========================
Channel Label     Typical value
======= ========= ==========================
in0     CPU 1P8   1.80 V
in1     CHIP Core 1.04 V
in2     VDD MISC  1.10 V
======= ========= ==========================

MSI MAG B850M MORTAR WIFI (MS-7E61)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

======= ========= ==========================
Channel Label     Typical value
======= ========= ==========================
in0     CPU 1P8   1.79 V
in1     CHIP Core 1.04 V
in2     VDD MISC  1.10 V
======= ========= ==========================
