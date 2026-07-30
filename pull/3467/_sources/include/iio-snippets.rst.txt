:orphan:

This page contains a few loose documentation snippets used in various spots.

IIO device files
================

.. start-iio-device-files

Each and every IIO device, typically a hardware chip, has a device folder under
``/sys/bus/iio/devices/iio:deviceX``. Where X is the IIO index of the device. Under
every of these directory folders reside a set of files, depending on the
characteristics and features of the hardware device in question. These files
are consistently generalized and documented in the IIO ABI documentation. In
order to determine which IIO deviceX corresponds to which hardware device, the
user can read the name file ``/sys/bus/iio/devices/iio:deviceX/name``. In case
the sequence in which the iio device drivers are loaded/registered is constant,
the numbering is constant and may be known in advance.

.. end-iio-device-files

IIO devices with trigger consumer interface
===========================================

.. start-iio-devices-with-trigger-consumer-interface

If deviceX supports triggered sampling, it's a so called trigger consumer and
there will be an additional folder ``/sys/bus/iio/device/iio:deviceX/trigger``.
In this folder there is a file called ``current_trigger``, allowing controlling
and viewing the current trigger source connected to deviceX. Available trigger
sources can be identified by reading the name file
``/sys/bus/iio/devices/triggerY/name``. The same trigger source can connect to
multiple devices, so a single trigger may initialize data capture or reading
from a number of sensors, converters, etc.

.. hint::

   **Trigger Consumers:**

   Currently triggers are only used for the filling of software ring
   buffers and as such any device supporting INDIO_RING_TRIGGERED has the
   consumer interface automatically created.

**Description:** Read name of triggerY

.. shell::

   /sys/bus/iio/devices/triggerY
   $ cat name
     irqtrig56

**Description:** Make irqtrig56 (trigger using system IRQ56, likely a GPIO
IRQ), to current trigger of deviceX

.. shell::

   /sys/bus/iio/devices/iio:deviceX/trigger
   $ echo irqtrig56 > current_trigger

**Description:** Read current trigger source of deviceX

.. shell::

   /sys/bus/iio/devices/iio:deviceX/trigger
   $ cat current_trigger
     irqtrig56

.. end-iio-devices-with-trigger-consumer-interface

Standalone trigger drivers
==========================

.. start-standalone-trigger-drivers

.. list-table::
   :header-rows: 1

   * - Name
     - Description
   * - iio-trig-gpio
     - Provides support for using GPIO pins as IIO triggers.
   * - iio-trig-rtc
     - Provides support for using periodic capable real time clocks as IIO triggers.
   * - iio-trig-sysfs
     - Provides support for using SYSFS entry as IIO triggers.
   * - iio-trig-bfin-timer
     - Provides support for using a Blackfin timer as IIO triggers.

.. end-standalone-trigger-drivers

Buffer management
=================

.. start-buffer-management

The Industrial I/O subsystem provides support for various ring buffer based
data acquisition methods. Apart from device specific hardware buffer support,
the user can chose between two different software ring buffer implementations.
One is the IIO lock free software ring, and the other is based on Linux kfifo.
Devices with buffer support feature an additional sub-folder in the
``/sys/bus/iio/devices/deviceX/`` folder hierarchy. Called ``deviceX:bufferY``,
where Y defaults to 0, for devices with a single buffer.

Every buffer implementation features a set of files:

**length**
   Get/set the number of sample sets that may be held by the buffer.

**enable**
   Enables/disables the buffer. This file should be written last, after length and selection of scan elements.

**watermark**
   A single positive integer specifying the maximum number of scan
   elements to wait for.
   Poll will block until the watermark is reached.
   Blocking read will wait until the minimum between the requested
   read amount or the low water mark is available.
   Non-blocking read will retrieve the available samples from the
   buffer even if there are less samples then watermark level. This
   allows the application to block on poll with a timeout and read
   the available samples after the timeout expires and thus have a
   maximum delay guarantee.

**data_available**
   A read-only value indicating the bytes of data available in the
   buffer. In the case of an output buffer, this indicates the
   amount of empty space available to write data to. In the case of
   an input buffer, this indicates the amount of data available for
   reading.

**length_align_bytes**
   Using the high-speed interface. DMA buffers may have an alignment
   requirement for the buffer length. Newer versions of the kernel will report
   the alignment requirements associated with a device through the
   ``length_align_bytes`` property.

**scan_elements**
   The scan_elements directory contains interfaces for elements that will be
   captured for a single triggered sample set in the buffer.

.. end-buffer-management

Typical ADC scan elements
=========================

.. start-typical-adc-scan-elements

**in_voltageX_en / in_voltageX-voltageY_en / timestamp_en:**
   Scan element control for triggered data capture.
   Writing 1 will enable the scan element, writing 0 will disable it

**in_voltageX_type / in_voltageX-voltageY_type / timestamp_type:**
   Description of the scan element data storage within the buffer
   and therefore in the form in which it is read from user-space.
   Form is [s|u]bits/storage-bits. s or u specifies if signed
   (2's complement) or unsigned. bits is the number of bits of
   data and storage-bits is the space (after padding) that it
   occupies in the buffer. Note that some devices will have
   additional information in the unused bits so to get a clean
   value, the bits value must be used to mask the buffer output
   value appropriately. The storage-bits value also specifies the
   data alignment. So u12/16 will be a unsigned 12 bit integer
   stored in a 16 bit location aligned to a 16 bit boundary.
   For other storage combinations this attribute will be extended
   appropriately.

**in_voltageX_index / in_voltageX-voltageY_index / timestamp_index:**
   A single positive integer specifying the position of this
   scan element in the buffer. Note these are not dependent on
   what is enabled and may not be contiguous. Thus for user-space
   to establish the full layout these must be used in conjunction
   with all _en attributes to establish which channels are present,
   and the relevant _type attributes to establish the data storage
   format.

.. end-typical-adc-scan-elements

Event Management
================

.. start-event-management

The Industrial I/O subsystem provides support for passing hardware generated
events up to userspace.

In IIO events are not used for passing normal readings from the sensing devices
to userspace, but rather for out of band information. Normal data reaches
userspace through a low overhead character device - typically via either
software or hardware buffer. The stream format is pseudo fixed, so is described
and controlled via sysfs rather than adding headers to the data describing what
is in it.

Pretty much all IIO events correspond to thresholds on some value derived from
one or more raw readings from the sensor. They are provided by the underlying
hardware.

Examples include:

* Straight crossing a voltage threshold
* Moving average crosses a threshold
* Motion detectors (lots of ways of doing this).
* Thresholds on sum squared or rms values.
* Rate of change thresholds.
* Lots more variants...

Events have timestamps.

The Interface:

* Single user at a time.

* Simple chrdev per device (aggregation across devices doesn't really make
  sense for IIO as you tend to really care which sensor caused the event rather
  than just that it happened.)

The format is:

.. code-block:: c

   /**
    * struct iio_event_data - The actual event being pushed to userspace
    * @id:         event identifier
    * @timestamp:  best estimate of time of event occurrence (often from
    *              the interrupt handler)
    */
   struct iio_event_data {
       u64 id;
       s64 timestamp;
   };

.. end-event-management

Typical event attributes
========================

.. start-typical-event-attributes

**/sys/bus/iio/devices/iio:deviceX/events**
   Configuration of which hardware generated events are passed up
   to user-space.

Threshold Events
----------------

**<type>Z[_name]_thresh[_rising|falling]_en**
   Event generated when channel passes a threshold in the specified
   (_rising|_falling) direction. If the direction is not specified,
   then either the device will report an event which ever direction
   a single threshold value is called in (e.g.
   ``<type>[Z][_name]_<raw|input>_thresh_value``) or
   ``<type>[Z][_name]_<raw|input>_thresh_rising_value`` and
   ``<type>[Z][_name]_<raw|input>_thresh_falling_value`` may take
   different values, but the device can only enable both thresholds
   or neither.
   Note the driver will assume the last p events requested are
   to be enabled where p is however many it supports (which may
   vary depending on the exact set requested. So if you want to be
   sure you have set what you think you have, check the contents of
   these attributes after everything is configured. Drivers may
   have to buffer any parameters so that they are consistent when
   a given event type is enabled a future point (and not those for
   whatever event was previously enabled).

**<type>Z[_name]_thresh[_rising|falling]_value**
   Specifies the value of threshold that the device is comparing
   against for the events enabled by
   ``<type>Z[_name]_thresh[_rising|falling]_en``.
   If separate attributes exist for the two directions, but
   direction is not specified for this attribute, then a single
   threshold value applies to both directions.
   The raw or input element of the name indicates whether the
   value is in raw device units or in processed units (as _raw
   and _input do on sysfs direct channel read attributes).

Rate of Change Events
---------------------

**<type>[Z][_name]_roc[_rising|falling]_en**
   Event generated when channel passes a threshold on the rate of
   change (1st differential) in the specified (_rising|_falling)
   direction. If the direction is not specified, then either the
   device will report an event which ever direction a single
   threshold value is called in (e.g.
   ``<type>[Z][_name]_<raw|input>_roc_value``) or
   ``<type>[Z][_name]_<raw|input>_roc_rising_value`` and
   ``<type>[Z][_name]_<raw|input>_roc_falling_value`` may take
   different values, but the device can only enable both rate of
   change thresholds or neither.
   Note the driver will assume the last p events requested are
   to be enabled where p is however many it supports (which may
   vary depending on the exact set requested. So if you want to be
   sure you have set what you think you have, check the contents of
   these attributes after everything is configured. Drivers may
   have to buffer any parameters so that they are consistent when
   a given event type is enabled a future point (and not those for
   whatever event was previously enabled).

**<type>[Z][_name]_roc[_rising|falling]_value**
   Specifies the value of rate of change threshold that the
   device is comparing against for the events enabled by
   ``<type>[Z][_name]_roc[_rising|falling]_en``.
   If separate attributes exist for the two directions,
   but direction is not specified for this attribute,
   then a single threshold value applies to both directions.
   The raw or input element of the name indicates whether the
   value is in raw device units or in processed units (as _raw
   and _input do on sysfs direct channel read attributes).

Magnitude Events
----------------

**<type>Z[_name]_mag[_rising|falling]_en**
   Similar to ``in_accel_x_thresh[_rising|_falling]_en``, but here the
   magnitude of the channel is compared to the threshold, not its
   signed value.

**<type>Z[_name]_mag[_rising|falling]_value**
   The value to which the magnitude of the channel is compared. If
   number or direction is not specified, applies to all channels of
   this type.

Temporal Conditions
-------------------

**<type>[Z][_name][_thresh|_roc][_rising|falling]_period**
   Period of time (in seconds) for which the condition must be
   met before an event is generated. If direction is not
   specified then this period applies to both directions.

.. end-typical-event-attributes

Low level register access via debugfs (direct_reg_access)
=========================================================

.. start-low-level-register-access-via-debugfs-direct-reg-access

Some IIO drivers feature an optional debug facility, allowing users to read or
write registers directly. Special care needs to be taken when using this
feature, since you can modify registers on the back of the driver.

.. tip::

   To simplify direct register access you may want to use the libiio
   ``iio_reg`` command line utility.

Accessing debugfs requires root privileges.

In order to identify if the IIO device in question feature this option you
first need to identify the IIO device number.

Therefore read the name attribute of each IIO device:

.. shell::

   $ grep "" /sys/bus/iio/devices/iio\:device*/name
     /sys/bus/iio/devices/iio:device0/name:ad7291
     /sys/bus/iio/devices/iio:device1/name:ad9361-phy
     /sys/bus/iio/devices/iio:device2/name:xadc
     /sys/bus/iio/devices/iio:device3/name:adf4351-udc-rx-pmod
     /sys/bus/iio/devices/iio:device4/name:adf4351-udc-tx-pmod
     /sys/bus/iio/devices/iio:device5/name:cf-ad9361-dds-core-lpc
     /sys/bus/iio/devices/iio:device6/name:cf-ad9361-lpc

Change directory to ``/sys/kernel/debug/iio/iio:deviceX`` and check if the
``direct_reg_access`` file exists.

.. shell::

   $ cd /sys/kernel/debug/iio/iio\:device1
   $ ls direct_reg_access
    direct_reg_access

Reading
-------

.. shell::

   /sys/kernel/debug/iio/iio:device1
   $ echo 0x7 > direct_reg_access
   $ cat direct_reg_access
     0x40

Writing
-------

Write ADDRESS VALUE:

.. shell::

   /sys/kernel/debug/iio/iio:device1
   $ echo 0x7 0x50 > direct_reg_access
   $ cat direct_reg_access
     0x50

Accessing HDL CORE registers
----------------------------

Special ADI device driver convention for devices that have both:

* a SPI/I2C control interface
* and some sort of HDL Core with registers (AXI)

In this case when accessing the HDL Core Registers always set BIT31.

The register map for the ADI HDL IP cores are documented at each IP page at
:external+hdl:ref:`library`, section "Register Map".

.. shell::

   /sys/kernel/debug/iio/iio:device6
   $ echo 0x80000000 > direct_reg_access
   $ cat direct_reg_access
     0x80062

.. end-low-level-register-access-via-debugfs-direct-reg-access

IIO pointers
============

.. start-iio-pointers

* IIO mailing list: linux-iio@vger.kernel.org
* `IIO Linux Kernel Documentation sysfs-bus-iio-* <https://www.kernel.org/doc/Documentation/ABI/testing>`_
* `IIO Documentation <https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio>`_
* :external+documentation:ref:`iio-oscilloscope`
* :external+documentation:ref:`libiio`
* :external+documentation:ref:`libiio internals`
* `IIO High Speed <https://events.static.linuxfound.org/sites/events/files/slides/iio_high_speed.pdf>`_
* `Software Defined Radio using the IIO framework <http://video.fosdem.org/2015/devroom-software_defined_radio/iiosdr.mp4>`_
* `libiio introduction <https://www.youtube.com/watch?v=p_VntEwUe24>`_

.. end-iio-pointers
