.. SPDX-License-Identifier: GPL-2.0

.. include:: <isonum.txt>

Kernel driver ionic-hwmon
=========================

Supported chips:

  * AMD Pensando Ethernet adapters whose firmware advertises
    ``IONIC_DEV_CAP_HWMON``

    Prefix: 'ionic'

Author:

	Arun Kumar Prakash <ArunKumar.Prakash@amd.com>

Description
-----------

AMD Pensando Ethernet adapters carry on-board sensors that the adapter
firmware can report.  The ionic Ethernet driver creates an ``ionic.hwmon``
auxiliary device for adapters whose firmware advertises sensors, and the
ionic-hwmon driver binds to it and exposes the readings to the Linux hwmon
subsystem, so userspace utilities such as ``sensors`` can read them.  The
hwmon device is a child of the adapter's PCI function.

The adapter reports which sensors it has when the hwmon device is created,
so the files that appear depend on the adapter and its firmware version.
The hwmon device is removed and re-created across a firmware recovery, and
the sensors are read again when it is re-created.

The labels are reported once, with the capabilities, and do not change while
the device is bound.

sysfs interface
---------------

Attribute files exist only for the channels and attributes the firmware
reports, within ``/sys/class/hwmon/hwmon[X]``:

========================= ======= =====================================
Name                      Perm    Description
========================= ======= =====================================
``temp[X]_input``         RO      Temperature in millidegrees Celsius
                                  (m\ |deg|\ C).  May read below zero.
``temp[X]_label``         RO      Temperature sensor label.
``temp[X]_crit``          RO      Critical temperature threshold, in
                                  millidegrees Celsius.
``temp[X]_crit_alarm``    RO      Critical temperature alarm indicator.
``in[X]_input``           RO      Voltage in millivolts (mV).
``in[X]_label``           RO      Voltage sensor label.
``power[X]_input``        RO      Power in microwatts (uW).
``power[X]_label``        RO      Power sensor label.
``power[X]_max``          RO      Maximum power threshold, in
                                  microwatts.
========================= ======= =====================================

Labels are strings supplied by the firmware.  A channel that reports an
empty label has no ``_label`` attribute.
