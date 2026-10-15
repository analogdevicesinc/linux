.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-accelerometer/adxl372


.. _adxl372:

ADXL372
=======

ADXL372 Input 3-Axis Digital Accelerometer Linux Driver.

Supported Devices
-----------------

- :adi:`ADXL372`

Evaluation Boards
-----------------

- :adi:`EVAL-ADXL372Z`

Description
-----------

This is a Linux industrial I/O (:external+system-level:ref:`iio`) subsystem driver for the
three-axis ADXL372 accelerometer, with SPI and I2C bus interfaces. The
industrial I/O subsystem provides a unified framework for drivers for many
different types of converters and sensors using a number of different physical
interfaces (i2c, spi, etc). See :external+system-level:ref:`iio` for more information.

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl372.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl372.c>`__

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - core driver
     - `adxl372.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl372.c>`__
     -
   * - spi driver
     - `adxl372_spi.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl372_spi.c>`__
     -
   * - i2c driver
     - `adxl372_i2c.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl372_i2c.c>`__
     -
   * - header
     - `adxl372.h <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl372.h>`__
     -
   * - Documentation
     - `adi,adxl372.yaml <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/iio/accel/adi,adxl372.yaml>`__
     -
   * - ABI documentation
     - `sysfs-bus-iio-accel-adxl372 <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/ABI/testing/sysfs-bus-iio-accel-adxl372>`__
     -

Example device tree

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - dts
     - :git+linux:`rpi-adxl372-overlay.dts <rpi-4.19.y:arch/arm/boot/dts/overlays/rpi-adxl372-overlay.dts>`
     -

Adding Linux driver support
---------------------------

Devicetree configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Example platform device initialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Required devicetree properties:

- compatible: Needs to be the name of the device. E.g. ``adi,adxl372``
- reg: the I2C address or SPI chip select number for the device

Required properties for SPI bus usage:

- spi-max-frequency: Maximum SPI clock frequency.

Optional properties:

- interrupts: a list of interrupt specifiers

Example for a SPI device node:

::

   #address-cells = <1>;
   #size-cells = <0>;
   status = "okay";

   adxl372@0 {
       compatible = "adi,adxl372";
       reg = <0>;
       spi-max-frequency = <1000000>;
       interrupts = <25 2>;
       interrupt-parent = <&gpio>;
   };

Example for a I2C device node:

::

   #address-cells = <1>;
   #size-cells = <0>;
   status = "okay";

   adxl372@53 {
       compatible = "adi,adxl372";
       reg = <0x53>;
       interrupts = <25 2>;
       interrupt-parent = <&gpio>;
   };

Driver testing
--------------

.. code-block:: text

   root:/> cd /sys/bus/iio/devices/
   root:/sys/bus/iio/devices> ls
   iio:device0

   root:/sys/bus/iio/devices> cd iio\:device0

   root:/sys/bus/iio/devices/iio:device0> ls -l
   drwxr-xr-x 2 root root    0 Oct 25 04:25 buffer
   -rw-r--r-- 1 root root 4096 Oct 25 04:25 current_timestamp_clock
   -r--r--r-- 1 root root 4096 Oct 25 04:25 dev
   -rw-r--r-- 1 root root 4096 Oct 25 04:28 in_accel_filter_low_pass_3db_frequency
   -r--r--r-- 1 root root 4096 Oct 25 04:25 in_accel_filter_low_pass_3db_frequency_available
   -rw-r--r-- 1 root root 4096 Oct 25 04:27 in_accel_sampling_frequency
   -rw-r--r-- 1 root root 4096 Oct 25 04:25 in_accel_scale
   -rw-r--r-- 1 root root 4096 Oct 25 04:25 in_accel_x_raw
   -rw-r--r-- 1 root root 4096 Oct 25 04:25 in_accel_y_raw
   -rw-r--r-- 1 root root 4096 Oct 25 04:25 in_accel_z_raw
   -r--r--r-- 1 root root 4096 Oct 25 04:25 name
   lrwxrwxrwx 1 root root    0 Oct 25 04:25 of_node -> ../../../../../../../../firmware/devicetree/base/soc/spi@7e204000/adxl372@0
   drwxr-xr-x 2 root root    0 Oct 25 04:25 power
   -r--r--r-- 1 root root 4096 Oct 25 04:25 sampling_frequency_available
   drwxr-xr-x 2 root root    0 Oct 25 04:25 scan_elements
   lrwxrwxrwx 1 root root    0 Oct 25 04:25 subsystem -> ../../../../../../../../bus/iio
   drwxr-xr-x 2 root root    0 Oct 25 04:25 trigger
   -rw-r--r-- 1 root root 4096 Oct 25 04:25 uevent

Identify the device
~~~~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device0> cat name
   adxl372

Basic operation
~~~~~~~~~~~~~~~

Show scale
^^^^^^^^^^

**Description:** scale to be applied to in_accel\_*_raw in order to obtain the
acceleration.

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_scale
   0.958241

Show axis x measurement
~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Raw unscaled acceleration measurement on x axis

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_x_raw
   -5

Set sampling frequency
~~~~~~~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device0> cat sampling_frequency_available
   400 800 1600 3200 6400
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_sampling_frequency
   6400
   root:/sys/bus/iio/devices/iio:device0> echo 3200 > in_accel_sampling_frequency
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_sampling_frequency
   3200

Show available bandwidths for the current set frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_sampling_frequency
   6400
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency_available
   200 400 800 1600 3200
   root:/sys/bus/iio/devices/iio:device0>

Change the bandwidth
~~~~~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency_available
   200 400 800 1600 3200
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency
   3200
   root:/sys/bus/iio/devices/iio:device0> echo 1600 > in_accel_filter_low_pass_3db_frequency
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency
   1600
   root:/sys/bus/iio/devices/iio:device0> echo 3200 > in_accel_sampling_frequency
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency_available
   200 400 800 1600
   root:/sys/bus/iio/devices/iio:device0> echo 1600 > in_accel_sampling_frequency
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency
   800

Using the FIFO Buffer
~~~~~~~~~~~~~~~~~~~~~

**Description:** The ADXL372 includes a deep, 512 sample FIFO buffer. The 512
FIFO samples can be allotted in several ways, such as the following:

- 170 sample sets of concurrent 3-axis data
- 256 sample sets of concurrent 2-axis data (user selectable)
- 512 sample sets of single-axis data
- 170 sets of impact event peak (x, y, z)

Trigger management
^^^^^^^^^^^^^^^^^^

**Description:** The FIFO is configured in **Stream Mode** and **FIFO_FULL**
interrupt is enabled. The interrupt will be triggered when the FIFO watermark is
reached.

.. code-block:: text

   root:/sys/bus/iio/devices/iio:device0> cat trigger/current_trigger
   adxl372-dev0

Substitute the actual IIO device number.

Buffer management
^^^^^^^^^^^^^^^^^

**Description:** The scan_elements directory contains interfaces for elements
that will be captured for a single triggered sample set in the buffer

::

   root:/sys/bus/iio/devices/iio:device0/scan_elements> ls -l
   -rw-r--r-- 1 root root 4096 Oct 25 08:08 in_accel_x_en
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_accel_x_index
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_accel_x_type
   -rw-r--r-- 1 root root 4096 Oct 25 08:08 in_accel_y_en
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_accel_y_index
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_accel_y_type
   -rw-r--r-- 1 root root 4096 Oct 25 08:08 in_accel_z_en
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_accel_z_index
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_accel_z_type
   -rw-r--r-- 1 root root 4096 Oct 25 08:08 in_timestamp_en
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_timestamp_index
   -r--r--r-- 1 root root 4096 Oct 25 08:08 in_timestamp_type
   root:/sys/bus/iio/devices/iio:device0/scan_elements>

Before enabling the buffer, a few steps need to be completed.

- For example, if we want the FIFO to store sample sets of concurrent 3-axis
  data, we need to enable the scan elements:

::

   root:/sys/bus/iio/devices/iio:device0/scan_elements> echo 1 > in_accel_x_en
   root:/sys/bus/iio/devices/iio:device0/scan_elements> echo 1 > in_accel_y_en
   root:/sys/bus/iio/devices/iio:device0/scan_elements> echo 1 > in_accel_z_en

::

   root:/sys/bus/iio/devices/iio:device0/buffer> ls
   enable hwfifo_enabled  hwfifo_watermark  hwfifo_watermark_max
   hwfifo_watermark_min  length  watermark
   root:/sys/bus/iio/devices/iio:device0/buffer>

- Set the buffer length:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 1024 > length

- Set the fifo watermak:

Because we enabled all three channels, then the watermark can be set to a maximum of 170 sample sets. ::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 170 > watermark

The FIFO can store impact-event peak acceleration. With the buffer disabled,
select the device’s peak trigger. In the correct IIO device directory,
substitute the actual device number in the trigger name:

.. code-block:: bash

   set -eu
   echo 0 > buffer/enable
   echo adxl372-dev0-peak > trigger/current_trigger
   cat trigger/current_trigger

Then configure the scan elements and enable the buffer. Peak acquisition stores
XYZ sets internally; the driver repacks the selected axes into the IIO scan.
Peak mode is selected through the trigger.

- Enable the buffer:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 1 > enable

To read samples: ::

   root:/sys/bus/iio/devices/iio:device0/buffer> cat /dev/iio:device0

Troubleshooting
---------------

Low level register access via debugfs (direct_reg_access)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some IIO drivers feature an optional debug facility, allowing users to read or
write registers directly. Special care needs to be taken when using this
feature, since you can modify registers on the back of the driver. Accessing
debugfs requires root privileges.

In order to identify if the IIO device in question feature this option you first
need to identify the IIO device number.

Therefore read the name attribute of each IIO device

::

   root@analog:~# grep "" /sys/bus/iio/devices/iio\:device/name
   adxl372
   root@analog:~#

Change directory to /sys/kernel/debug/iio/iio:deviceX and check if the
direct_reg_access file exists.

::

   root@analog:~# cd /sys/kernel/debug/iio/iio\:device0/
   root@analog:/sys/kernel/debug/iio/iio:device0# ls direct_reg_access
   ls direct_reg_access
   root@analog:/sys/kernel/debug/iio/iio:device0#

Reading

::

   root@analog:/sys/kernel/debug/iio/iio:device0# echo 0x0 > direct_reg_access

   root@analog:/sys/kernel/debug/iio/iio:device0# cat direct_reg_access
   0xAD
   root@analog:/sys/kernel/debug/iio/iio:device0#

Writing

Write ADDRESS VALUE

::

   root@analog:/sys/kernel/debug/iio/iio:device0# echo 0x3D 0x80 > direct_reg_access
   root@analog:/sys/kernel/debug/iio/iio:device0# cat direct_reg_access
   0x80
   root@analog:/sys/kernel/debug/iio/iio:device0#

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
