.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-accelerometer/adxl355

.. _adxl355:

ADXL355
=======

ADXL355 Low Noise, Low Drift, Low Power, 3-Axis MEMS Accelerometers.

Supported Devices
-----------------

- :adi:`ADXL355`

Evaluation Boards
-----------------

- :adi:`EVAL-ADXL355-PMDZ`
- :adi:`EVAL-ADXL35X`

Description
-----------

This is a Linux industrial I/O (:external+documentation:ref:`iio`) subsystem
driver, targeting serial interface accelerometers. The industrial I/O subsystem
provides a unified framework for drivers for many different types of converters
and sensors using a number of different physical interfaces (i2c, spi, etc). See
:external+documentation:ref:`iio` for more information.

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl355_core.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl355_core.c>`__
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
     - :git-linux:`drivers/iio/accel/adxl355_core.c`
     -
   * - spi driver
     - :git-linux:`drivers/iio/accel/adxl355_spi.c`
     -
   * - i2c driver
     - :git-linux:`drivers/iio/accel/adxl355_i2c.c`
     -
   * - header
     - :git-linux:`drivers/iio/accel/adxl355.h`
     -
   * - Documentation
     - :git-linux:`adi,adxl355.yaml <Documentation/devicetree/bindings/iio/accel/adi,adxl355.yaml>`
     -

Example platform device initialization
--------------------------------------

Required devicetree properties:

- compatible: Needs to be the name of the device. E.g. ``adi,adxl355``
- reg: the I2C address or SPI chip select number for the device

Required properties for SPI bus usage:

- spi-max-frequency: Maximum SPI clock frequency.

Optional properties:

- interrupts: a list of interrupt specifiers

Example for a SPI device node:

::

   #include <dt-bindings/gpio/gpio.h>
   #include <dt-bindings/interrupt-controller/irq.h>
   spi {
           #address-cells = <1>;
           #size-cells = <0>;
           accelerometer@0 {
                   compatible = "adi,adxl355";
                   reg = <0>;
                   spi-max-frequency = <1000000>;
                   interrupt-parent = <&gpio>;
                   interrupts = <25 IRQ_TYPE_EDGE_RISING>;
                   interrupt-names = "DRDY";
           };
   };

Example for a I2C device node:

::

   i2c {
            #address-cells = <1>;
            #size-cells = <0>;
            /* Example for a I2C device node */
            accelerometer@1d {
                    compatible = "adi,adxl355";
                    reg = <0x1d>;
                    interrupt-parent = <&gpio>;
                    interrupts = <25 IRQ_TYPE_EDGE_RISING>;
                    interrupt-names = "DRDY";
            };
    };

Driver testing
--------------

::

   root:/> cd /sys/bus/iio/devices/
   root:/sys/bus/iio/devices> ls
   iio:device0

   root:/sys/bus/iio/devices> cd iio\:device0

   root:/sys/bus/iio/devices/iio:device0> ls -l
   drwxr-xr-x  2 root root    0 Nov 11 15:44 buffer
   -rw-r--r--  1 root root 4096 Nov 11 15:44 current_timestamp_clock
   -r--r--r--  1 root root 4096 Nov 11 15:44 dev
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_filter_high_pass_3db_frequency
   -r--r--r--  1 root root 4096 Nov 11 15:44 in_accel_filter_high_pass_3db_frequency_available
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_sampling_frequency
   -r--r--r--  1 root root 4096 Nov 11 15:44 in_accel_sampling_frequency_available
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_scale
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_x_calibbias
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_x_raw
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_y_calibbias
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_y_raw
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_z_calibbias
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_accel_z_raw
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_temp_offset
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_temp_raw
   -rw-r--r--  1 root root 4096 Nov 11 15:44 in_temp_scale
   -r--r--r--  1 root root 4096 Nov 11 15:44 name
   drwxr-xr-x  2 root root    0 Nov 11 15:44 power
   drwxr-xr-x  2 root root    0 Nov 11 15:44 scan_elements
   lrwxrwxrwx  1 root root    0 Nov 11 15:44 subsystem -> ../../bus/iio
   drwxr-xr-x  2 root root    0 Nov 11 15:44 trigger
   -rw-r--r--  1 root root 4096 Nov 11 15:44 uevent

Show device name
~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device0> cat name
   adxl355

Show scale
^^^^^^^^^^

**Description:** scale to be applied to in_accel\_*_raw in order to obtain the
acceleration.

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_scale
   0.000038245

Show axis x measurement
'''''''''''''''''''''''

**Description:** Raw unscaled acceleration measurement on x axis

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_x_raw
   -33631

Set sampling frequency
''''''''''''''''''''''

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_sampling_frequency_available
   4000.000000 2000.000000 1000.000000 500.000000 250.000000 125.000000 62.500000 31.250000 15.625000 7.813000 3.906000

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_sampling_frequency
   6400
   root:/sys/bus/iio/devices/iio:device0> echo 4000 > in_accel_sampling_frequency
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_sampling_frequency
   4000.000000

Show available bandwidths for the current set frequency
'''''''''''''''''''''''''''''''''''''''''''''''''''''''

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency_available
   0.000000 9.880000 2.483360 0.621800 0.154480 0.038160 0.009520
   root:/sys/bus/iio/devices/iio:device0>

Change the bandwidth
''''''''''''''''''''

::

   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency_available
   0.000000 9.880000 2.483360 0.621800 0.154480 0.038160 0.009520
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency
   0
   root:/sys/bus/iio/devices/iio:device0> echo 0.621800 > in_accel_filter_low_pass_3db_frequency
   root:/sys/bus/iio/devices/iio:device0> cat in_accel_filter_low_pass_3db_frequency
   0.621800

Trigger management
^^^^^^^^^^^^^^^^^^

::

   root:/sys/bus/iio/devices/iio:device0> cat trigger/current_trigger
   adxl355-dev-

Buffer management
^^^^^^^^^^^^^^^^^

**Description:** The scan_elements directory contains interfaces for elements
that will be captured for a single triggered sample set in the buffer

::

   root:/sys/bus/iio/devices/iio:device0/scan_elements> ls -l
   -rw-r--r-- 1 root root 4096 Nov 11 15:44 in_accel_x_en
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_accel_x_index
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_accel_x_type
   -rw-r--r-- 1 root root 4096 Nov 11 15:44 in_accel_y_en
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_accel_y_index
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_accel_y_type
   -rw-r--r-- 1 root root 4096 Nov 11 15:44 in_accel_z_en
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_accel_z_index
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_accel_z_type
   -rw-r--r-- 1 root root 4096 Nov 11 15:44 in_temp_en
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_temp_index
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_temp_type
   -rw-r--r-- 1 root root 4096 Nov 11 15:44 in_timestamp_en
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_timestamp_index
   -r--r--r-- 1 root root 4096 Nov 11 15:44 in_timestamp_type
   root:/sys/bus/iio/devices/iio:device0/scan_elements>

::

   root:/sys/bus/iio/devices/iio:device0/buffer> ls
   data_available  enable  length  watermark
   root:/sys/bus/iio/devices/iio:device0/buffer>

- Set the buffer length:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 1024 > length

- Enable the buffer:

::

   root:/sys/bus/iio/devices/iio:device0/buffer> echo 1 > enable

To read samples: ::

   root:/sys/bus/iio/devices/iio:device0/buffer> cat /dev/iio:device0

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
