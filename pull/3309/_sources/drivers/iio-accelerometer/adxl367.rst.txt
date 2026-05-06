.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-accelerometer/adxl367

.. _adxl367:

ADXL367
=======

ADXL367 Input 3-Axis Digital Accelerometer Linux Driver.

- :adi:`Product Page <ADXL367>`

Supported Devices
-----------------

- :adi:`ADXL367`

Evaluation Boards
-----------------

- :adi:`EVAL-ADXL367Z`
- :adi:`EVAL-ADXL367-SDP`

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
     -
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl367.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl367.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - core driver
     - :git+linux:`main:drivers/iio/accel/adxl367.c`
     -
   * - spi driver
     - :git+linux:`main:drivers/iio/accel/adxl367_spi.c`
     -
   * - i2c driver
     - :git+linux:`main:drivers/iio/accel/adxl367_i2c.c`
     -
   * - header
     - :git+linux:`drivers/iio/accel/adxl367.h <main:drivers/iio/iio/accel/adxl367.h>`
     -
   * - Documentation
     - :git+linux:`adi,adxl367.yaml <main:./Documentation/devicetree/bindings/iio/accel/adi,adxl367.yaml>`
     -

Example device tree

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - dts
     - :git+linux:`rpi-adxl367-overlay.dts <rpi-4.19.y:arch/arm/boot/dts/overlays/rpi-adxl367-overlay.dts>`
     -

Example platform device initialization
--------------------------------------

Required devicetree properties:

- compatible: Needs to be the name of the device. E.g. ``adi,adxl367``
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

   adxl367@0 {
       compatible = "adi,adxl367";
       reg = <0>;
       spi-max-frequency = <1000000>;
       interrupt-parent = <&gpio>;
       interrupts = <25 IRQ_TYPE_EDGE_RISING>;
   };

Example for a I2C device node:

::

   #address-cells = <1>;
   #size-cells = <0>;
   status = "okay";

   adxl367@53 {
       compatible = "adi,adxl367";
       reg = <53>;
       interrupt-parent = <&gpio>;
       interrupts = <25 IRQ_TYPE_EDGE_RISING>;
   };

Driver testing
--------------

::

   root:/> cd /sys/bus/iio/devices/
   root:/sys/bus/iio/devices> ls
   iio:device0

   root:/sys/bus/iio/devices> cd iio\:device0

   root:/sys/bus/iio/devices/iio:device0> ls -l
   drwxr-xr-x 2 root root    0 May  3 16:17 buffer
   -r--r--r-- 1 root root 4096 May  3 16:17 dev
   drwxr-xr-x 2 root root    0 May  3 16:17 events
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_scale
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_scale_available
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_x_raw
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_y_raw
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_z_raw
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_temp_offset
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_temp_raw
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_temp_scale
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_voltage_offset
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_voltage_raw
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_voltage_scale
   -r--r--r-- 1 root root 4096 May  3 16:17 name
   lrwxrwxrwx 1 root root    0 May  3 16:17 of_node -> ../../../../../../../../firmware/devicetree/base/soc/spi@7e204000/adxl367@0
   drwxr-xr-x 2 root root    0 May  3 16:17 power
   -rw-r--r-- 1 root root 4096 May  3 16:17 sampling_frequency
   -r--r--r-- 1 root root 4096 May  3 16:17 sampling_frequency_available
   drwxr-xr-x 2 root root    0 May  3 16:17 scan_elements
   lrwxrwxrwx 1 root root    0 May  3 16:17 subsystem -> ../../../../../../../../bus/iio
   -rw-r--r-- 1 root root 4096 May  3 16:17 uevent

Show device name
~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device0> cat name
   adxl367

Show scale
^^^^^^^^^^

**Description:** scale to be applied to in_accel\_*_raw in order to obtain the
acceleration.

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_scale
   0.002394347

Show axis x measurement
'''''''''''''''''''''''

**Description:** Raw unscaled acceleration measurement on x axis

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_x_raw
   -5

Set sampling frequency
''''''''''''''''''''''

::

   root:/sys/bus/iio/devices/iio:device0> cat sampling_frequency_available
   12.500000 25.000000 50.000000 100.000000 200.000000 400.000000
   root:/sys/bus/iio/devices/iio:device0> cat sampling_frequency
   12.500000
   root:/sys/bus/iio/devices/iio:device0> echo 400.000000 > sampling_frequency
   root:/sys/bus/iio/devices/iio:device0> cat sampling_frequency
   400.000000

Using the FIFO Buffer
~~~~~~~~~~~~~~~~~~~~~

**Description:** The ADXL367 includes a deep, 512 sample FIFO buffer. The 512
FIFO samples can be allotted in several ways, such as the following:

- 512 sample sets of single-axis data
- 256 sample sets of concurrent 2-channel data
- 170 sample sets of concurrent 3-channel data
- 128 sample sets of concurrent 4-channel data

Buffer management
^^^^^^^^^^^^^^^^^

**Description:** The scan_elements directory contains interfaces for elements
that will be captured for a single triggered sample set in the buffer

::

   root:/sys/bus/iio/devices/iio:device0> cd scan_elements/
   root:/sys/bus/iio/devices/iio:device0/scan_elements> ls -l
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_x_en
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_x_index
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_x_type
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_y_en
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_y_index
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_y_type
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_accel_z_en
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_z_index
   -r--r--r-- 1 root root 4096 May  3 16:17 in_accel_z_type
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_temp_en
   -r--r--r-- 1 root root 4096 May  3 16:17 in_temp_index
   -r--r--r-- 1 root root 4096 May  3 16:17 in_temp_type
   -rw-r--r-- 1 root root 4096 May  3 16:17 in_voltage_en
   -r--r--r-- 1 root root 4096 May  3 16:17 in_voltage_index
   -r--r--r-- 1 root root 4096 May  3 16:17 in_voltage_type

Before enabling the buffer, a few steps need to be completed.

- For example, if we want the FIFO to store sample sets of concurrent 3-channel
  data, we need to enable the scan elements:

::

   root:/sys/bus/iio/devices/iio:device0/scan_elements> echo 1 > in_accel_x_en
   root:/sys/bus/iio/devices/iio:device0/scan_elements> echo 1 > in_accel_y_en
   root:/sys/bus/iio/devices/iio:device0/scan_elements> echo 1 > in_accel_z_en

::

   root:/sys/bus/iio/devices/iio:device0/scan_elements> cd ../buffer/
   root:/sys/bus/iio/devices/iio:device0/buffer> ls -l
   -r--r--r-- 1 root root 4096 May  3 16:17 data_available
   -rw-r--r-- 1 root root 4096 May  3 16:17 enable
   -r--r--r-- 1 root root 4096 May  3 16:17 hwfifo_enabled
   -r--r--r-- 1 root root 4096 May  3 16:17 hwfifo_watermark
   -r--r--r-- 1 root root 4096 May  3 16:17 hwfifo_watermark_max
   -r--r--r-- 1 root root 4096 May  3 16:17 hwfifo_watermark_min
   -rw-r--r-- 1 root root 4096 May  3 16:17 length
   -rw-r--r-- 1 root root 4096 May  3 16:17 watermark

- Set the buffer length:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 1024 > length

- Set the watermak:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 512 > watermark

- Enable the buffer:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 1 > enable

- Read the samples:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> cat /dev/iio:device0

Low level register access via debugfs (direct_reg_access)
---------------------------------------------------------

Some IIO drivers feature an optional debug facility, allowing users to read or
write registers directly. Special care needs to be taken when using this
feature, since you can modify registers on the back of the driver. Accessing
debugfs requires root privileges.

::

   root:/> /sys/kernel/debug/iio/iio:device0/
   root:/sys/kernel/debug/iio/iio:device0> ls direct_reg_access
   direct_reg_access

Reading

::

   root:/sys/kernel/debug/iio/iio:device0> echo 0x0 > direct_reg_access
   root:/sys/kernel/debug/iio/iio:device0> cat direct_reg_access
   0xAD

Writing

Write ADDRESS VALUE

::

   root@analog:/sys/kernel/debug/iio/iio:device0> echo 0x3D 0x80 > direct_reg_access
   root@analog:/sys/kernel/debug/iio/iio:device0> cat direct_reg_access
   0x80

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
