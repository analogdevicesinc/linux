.. SPDX-License-Identifier: GPL-2.0-only

Open Sensor Fusion
==================

Open Sensor Fusion is a sensor aggregation hub interface. The Linux IIO driver
receives device-to-host frames over UART and discovers sensor channels from
capability reports. Device Tree describes the hub using the
``opensensorfusion,osf-green`` compatible with an ``opensensorfusion,osf``
fallback; individual sensors are discovered at runtime. See the binding in
``Documentation/devicetree/bindings/iio/opensensorfusion,osf.yaml``.

The OSF Device-to-Host 0.0 specification is maintained by the Open Sensor Fusion
project. The canonical specification_ defines the wire layout, compatibility
rules, reserved fields and physical units. This document describes the Linux
receiver profile and its mapping to IIO.

.. _specification: https://github.com/opensensorfusion/opensensorfusion-protocol/blob/ca9cdea1ae550c2b4d6f29f87877adae99470744/spec/osf-d2h-0.0.md

Supported receive profile
-------------------------

The driver supports the fixed OSF0 frame layout with protocol major version 0.
Compatible minor versions use the same known message layouts. The decoder
requires exact lengths for known messages, validates CRC and bounds before
interpreting payloads, and tolerates reserved padding. Unsupported major
versions and unknown message types are ignored. Unsupported capability entries are skipped
individually. A CRC-valid application rejection consumes the complete frame;
an invalid frame candidate is resynchronized without trusting its payload length.

The receive path handles sensor samples, complete capability reports and
device status. Its limits are 4096 bytes per frame and 16 supported sensor
descriptors. These are implementation limits, not protocol-wide maxima.

IIO mapping
-----------

The supported sensor classes map to:

* accelerometer: ``IIO_ACCEL`` X/Y/Z, in m/s^2;
* gyroscope: ``IIO_ANGL_VEL`` X/Y/Z, in rad/s;
* magnetometer: ``IIO_MAGN`` X/Y/Z, in gauss;
* temperature: ``IIO_TEMP``, in millidegrees Celsius.

Each registered channel exposes signed ``RAW`` data and descriptor-derived
``SCALE``. Multiplying RAW by SCALE gives a value in the channel's IIO unit.
Buffered samples use signed 32-bit values with native CPU endianness. Active
channels are packed into the scan, with initialized padding before an optional
64-bit timestamp.

Discovery and session lifetime
------------------------------

Each capability report is a complete inventory snapshot, not an incremental
fragment. The driver validates all supported descriptors before registering
IIO devices. An initial report containing supported descriptors establishes
the supported inventory. An empty or entirely unsupported initial report
establishes no supported inventory and leaves discovery open for a later
supported complete report. Supported descriptors must advertise a nonzero scale.
A sample's channel count, format and scale must match its registered descriptor
before it can update the latest RAW cache or be published to a buffer. A
rejected sample preserves previously accepted data.

After inventory establishment, later complete reports continue to be processed.
Equivalent supported inventories are idempotent. Reports are compared by sensor
type and index, independently of entry order. Reserved padding, advisory flags and unsupported additions do not change
the supported inventory. A supported key appearing or disappearing, or a
change in its channel count, format or scale, faults the bound session.

After a session fault, new cache updates and buffer publication stop, and
direct RAW reads fail. Existing IIO devices remain until driver teardown.
An equivalent report does not clear the fault; explicit unbind/rebind starts
fresh discovery. No fault-specific userspace ABI is exposed.

Timestamps
----------

Buffered samples use the selected IIO host clock when they are pushed into
the buffer. The device timestamp is not correlated with that clock and is not
used as the IIO buffer timestamp. Device timestamp or sequence discontinuities
alone do not start a new host session.
