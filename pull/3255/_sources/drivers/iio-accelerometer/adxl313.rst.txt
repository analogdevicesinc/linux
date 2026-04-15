.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-accelerometer/adxl313

.. _adxl313:

ADXL312
=======

ADXL312, ADXL313, ADXL314 Low Noise, Low Drift, Low Power, 3-Axis MEMS Accelerometers.

Supported Devices
-----------------

- :adi:`ADXL312`
- :adi:`ADXL313`
- :adi:`ADXL314`

Evaluation Boards
-----------------

- :adi:`EVAL-ADXL312Z <en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/eval-adxl312.html#eb-overview>`
- :adi:`EVAL-ADXL313Z <en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/eval-adxl313.html#eb-overview>`
- :adi:`EVAL-ADXL314Z <en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADXL314Z.html>`

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl313_core.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/accel/adxl313_core.c>`__
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
     - :git+linux:`main:drivers/iio/accel/adxl313_core.c`
     -
   * - spi driver
     - :git+linux:`main:drivers/iio/accel/adxl313_spi.c`
     -
   * - i2c driver
     - :git+linux:`main:drivers/iio/accel/adxl313_i2c.c`
     -
   * - header
     - :git+linux:`main:drivers/iio/accel/adxl313.h`
     -
   * - Documentation
     - :git+linux:`adi,adxl313.yaml <main:Documentation/devicetree/bindings/iio/accel/adi,adxl313.yaml>`
     -

Example platform device initialization
--------------------------------------

Required devicetree properties:

- compatible: Needs to be the name of the device. E.g. ``adi,adxl313``,
  ``adi,adxl314``
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

       /* Example for a SPI device node */
       accelerometer@0 {
           compatible = "adi,adxl313";
           reg = <0>;
           spi-max-frequency = <5000000>;
           interrupt-parent = <&gpio0>;
           interrupts = <0 IRQ_TYPE_LEVEL_HIGH>;
           interrupt-names = "INT1";
       };
   };

Example for a I2C device node:

::

   #include <dt-bindings/gpio/gpio.h>
   #include <dt-bindings/interrupt-controller/irq.h>
   i2c0 {
       #address-cells = <1>;
       #size-cells = <0>;

       /* Example for a I2C device node */
       accelerometer@53 {
           compatible = "adi,adxl313";
           reg = <0x53>;
           interrupt-parent = <&gpio0>;
           interrupts = <0 IRQ_TYPE_LEVEL_HIGH>;
           interrupt-names = "INT1";
       };
   };

Driver testing
--------------

::

   root@analog:~# cd /sys/bus/iio/devices/
   root@analog:/sys/bus/iio/devices# ls
   iio:device0  iio_sysfs_trigger
   root@analog:/sys/bus/iio/devices# cd iio\:device0
   root@analog:/sys/bus/iio/devices/iio:device0# ls -l
   total 0
   -r--r--r-- 1 root root 4096 Sep 23 14:33 dev
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_sampling_frequency
   -r--r--r-- 1 root root 4096 Sep 23 14:33 in_accel_sampling_frequency_available
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_scale
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_x_calibbias
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_x_raw
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_y_calibbias
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_y_raw
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_z_calibbias
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 in_accel_z_raw
   -r--r--r-- 1 root root 4096 Sep 23 14:33 name
   lrwxrwxrwx 1 root root    0 Sep 23 14:33 of_node -> ../../../../../../../../firmware/devicetree/base/soc/spi@7e204000/adxl313@0
   drwxr-xr-x 2 root root    0 Sep 23 14:33 power
   lrwxrwxrwx 1 root root    0 Sep 23 14:33 subsystem -> ../../../../../../../../bus/iio
   -rw-r--r-- 1 root root 4096 Sep 23 14:33 uevent

Show device name
~~~~~~~~~~~~~~~~

::

   root@analog:/sys/bus/iio/devices/iio:device0# cat name
   adxl313

Show scale
^^^^^^^^^^

**Description:** scale to be applied to in_accel\_*_raw in order to obtain the
acceleration.

::

   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_scale
   0.009576806

Show axis x measurement
'''''''''''''''''''''''

**Description:** Raw unscaled acceleration measurement on x axis

::

   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_x_raw
   -14

Set sampling frequency
''''''''''''''''''''''

::

   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_sampling_frequency_available
   6.250000 12.500000 25.000000 50.000000 100.000000 200.000000 400.000000 800.000000 1600.000000 3200.000000
   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_sampling_frequency
   100.000000
   root@analog:/sys/bus/iio/devices/iio:device0# echo 12.5 > in_accel_sampling_frequency
   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_sampling_frequency
   12.500000

Set calibbias for the Z channel
'''''''''''''''''''''''''''''''

::

   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_z_calibbias
   0
   root@analog:/sys/bus/iio/devices/iio:device0# echo 4 > in_accel_z_calibbias
   root@analog:/sys/bus/iio/devices/iio:device0# cat in_accel_z_calibbias
   4

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
