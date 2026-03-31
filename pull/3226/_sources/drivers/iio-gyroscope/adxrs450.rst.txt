.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-gyroscope/adxrs450

.. _adxrs450:

ADXRS450
========

ADXRS450 IIO Gyroscope.

Supported Devices
-----------------

- :adi:`ADXRS450`
- :adi:`ADXRS453`

Evaluation Boards
-----------------

- :adi:`EVAL-ADXL313-Z-M`
- :adi:`EVAL-ADXRS450Z`
- :adi:`EVAL-ADXRS453Z`
- `PmodGYRO2 <http://www.digilentinc.com/Products/Detail.cfm?Prod=PMOD-GYRO2>`__

ADXRS450: ±300°/sec High Vibration Immunity Digital Gyro
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ADXRS450 is a complete angular rate sensor with integrated signal
conditioning and an SPI digital interface. It provides superior vibration
rejection through a unique quad sensor design that rejects the influence of
linear acceleration, enabling accurate rate sensing in harsh environments.

- :adi:`Product Page <ADXRS450>`

ADXRS453: ±300°/sec High Vibration Immunity Digital Gyro
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ADXRS453 provides the same quad sensor design as the ADXRS450 with
enhanced performance specifications. It offers improved bias stability and
sensitivity, making it suitable for precision navigation and motion sensing
applications requiring high accuracy and vibration immunity.

- :adi:`Product Page <ADXRS453>`

Adding ADXRS450 to the Kernel
-----------------------------

To add support for the ADXRS450 to the kernel build system, a few things must be
enabled properly for things to work.The configuration is as following:

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The ADXRS450 Driver depends on **CONFIG_SPI**

::

   Linux Kernel Configuration
     Device Drivers  --->
           <*>     Industrial I/O support  --->
             <M>   Analog Devices ADXRS450/3 Digital Output Gyroscope SPI driver

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/gyro/adxrs450.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/gyro/adxrs450.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - driver
     - `drivers/iio/gyro/adxrs450.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/gyro/adxrs450.c>`__

Example platform device initialization
--------------------------------------

Below is an example which is used on Blackfin board file.

.. include:: /include/platform-and-bus-model.rst
   :start-after: .. start-declaring-spi-slave-devices
   :end-before: .. end-declaring-spi-slave-devices

.. code:: c

   static struct spi_board_info board_spi_board_info[] __initdata = {
   #if defined(CONFIG_ADXRS450) \
       || defined(CONFIG_ADXRS450_MODULE)
       {
           .modalias = "adxrs450",
           .bus_num = 0,
           .chip_select = GPIO_PF10 + MAX_CTRL_CS, /* GPIO controlled SSEL */
           .max_speed_hz = 2000000,
           .mode = SPI_MODE_0,
       },
   #endif
   };

.. code:: c

   static int __init board_init(void)
   {
       [--snip--]

       spi_register_board_info(board_spi_board_info, ARRAY_SIZE(board_spi_board_info));

       [--snip--]

       return 0;
   }
   arch_initcall(board_init);

Hardware configuration
----------------------

.. figure:: https://wiki.analog.com/_media/software/driver/linux/adxrs450_eval_lr.jpg
   :width: 600px

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/iio-gyroscope/pmodgyro2_lr.jpg
   :width: 600px

Driver testing
--------------

Module loading
~~~~~~~~~~~~~~

::

   root:/> modprobe adxrs450
   adxrs450 spi0.18: The Part ID is 0x5201
   adxrs450 spi0.18: The Serial Number is 0xbaf2

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-device-files
   :end-before: .. end-iio-device-files

::

   root:/> cd /sys/bus/iio/devices/
   root:/sys/bus/iio/devices> ls
   iio:device0
   root:/sys/bus/iio/devices> cd iio:device0

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > ls -l
   -r--r--r--    1 root     root          4096 Jan  3 16:24 dev
   -rw-r--r--    1 root     root          4096 Jan  3 16:24 in_anglvel_z_calibbias
   -rw-r--r--    1 root     root          4096 Jan  3 16:24 in_anglvel_z_quadrature_correction_raw
   -r--r--r--    1 root     root          4096 Jan  3 16:24 in_anglvel_z_raw
   -rw-r--r--    1 root     root          4096 Jan  3 16:24 in_anglvel_z_scale
   -r--r--r--    1 root     root          4096 Jan  3 16:24 in_temp0_raw
   -rw-r--r--    1 root     root          4096 Jan  3 16:24 in_temp0_scale
   -r--r--r--    1 root     root          4096 Jan  3 16:24 name
   drwxr-xr-x    2 root     root             0 Jan  3 16:24 power
   lrwxrwxrwx    1 root     root             0 Jan  3 16:24 subsystem -> ../../../../../bus/iio
   -rw-r--r--    1 root     root          4096 Jan  3 16:24 uevent

Show device name
^^^^^^^^^^^^^^^^

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0> cat name
   adxrs450

Show angular rate scale
'''''''''''''''''''''''

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0> cat in_anglvel_z_scale
   0.000218166

Show angular rate
'''''''''''''''''

Rotate the device and read angular rate sampling values from sysfs nodes like
this:

Rotate in positive direction
''''''''''''''''''''''''''''

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_anglvel_z_raw
   42
   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_anglvel_z_raw
   1456

Rotate in negative direction
''''''''''''''''''''''''''''

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_anglvel_z_raw
   -1657
   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_anglvel_z_raw
   -17215

Show quadrature correction
''''''''''''''''''''''''''

This attribute is used to read the amount of quadrature error present in the
device at a given time.

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_anglvel_z_quadrature_correction_raw
   9

Set dynamic null correction
'''''''''''''''''''''''''''

This attribute is used to make small adjustments to the rateout of the device.
This 10-bit value allows the user to adjust the static rateout of the device by
up to ±6.4°/sec.

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_gyro_z_raw
   40

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > echo 40 > in_anglvel_z_calibbias

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0 > cat in_anglvel_z_raw
   1

Show temperature
''''''''''''''''

::

   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0> cat in_temp0_scale
   200
   root:/sys/devices/platform/bfin-spi.0/spi0.18/iio:device0> cat in_temp0_raw
   105

**T** = *in_temp0_raw* \* *in_temp0_scale* = 105 \* 200 = 21000 milli degree
Celsius

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
