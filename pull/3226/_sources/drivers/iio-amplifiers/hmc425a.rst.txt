.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-amplifiers/hmc425a

.. _hmc425a:

HMC425A
=======

HMC425A Digital Step Attenuator Linux Driver.

Supported Devices
-----------------

- :adi:`HMC425A`
- :adi:`HMC540S`

Evaluation Boards
-----------------

- :adi:`EVAL-HMC425A`
- :adi:`EVAL-HMC540S`

Description
-----------

This is a Linux industrial I/O (:external+documentation:ref:`iio`) subsystem
driver, targeting Digital Step Attenuator IIO devices with gpio interface. The
industrial I/O subsystem provides a unified framework for drivers for many
different types of converters and sensors using a number of different physical
interfaces (i2c, spi, etc). See :external+documentation:ref:`iio` for more
information.

- HMC425A 0.5 dB LSB GaAs MMIC 6-BIT DIGITAL POSITIVE CONTROL ATTENUATOR, 2.2 -
  8.0 GHz

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
   * - :git-linux:`git <drivers/iio/amplifiers/hmc425a.c>`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/amplifiers/hmc425a.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git-linux:`drivers/iio/amplifiers/hmc425a.c`
     -
   * - Documentation
     - `Documentation/ABI/testing/sysfs-bus-iio <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/ABI/testing/sysfs-bus-iio>`__
     -

Devicetree
~~~~~~~~~~

properties
^^^^^^^^^^

**compatible:**

- enum:
- adi,hmc425a

**vcc-supply:**

- description: digital voltage regulator (see regulator/regulator.txt)
- maxItems: 1

**ctrl-gpios:**

- description: Must contain an array of 6 GPIO specifiers, referring to the GPIO
  pins connected to the control pins V1-V6.
- maxItems: 6

**required:**

- compatible
- ctrl-gpios

Example
^^^^^^^

.. code:: dts

   #include <dt-bindings/gpio/gpio.h>

   &gpio {
       #address-cells = <1>;
       #size-cells = <0>;
       gpio_hmc425a: hmc425a {
           compatible = "adi,hmc425a";
           ctrl-gpios = <&gpio 40 GPIO_ACTIVE_HIGH>,
           <&gpio 39 GPIO_ACTIVE_HIGH>,
           <&gpio 38 GPIO_ACTIVE_HIGH>,
           <&gpio 37 GPIO_ACTIVE_HIGH>,
           <&gpio 36 GPIO_ACTIVE_HIGH>,
           <&gpio 35 GPIO_ACTIVE_HIGH>;

           vcc-supply = <&foo>;
       };
   }

Adding Linux driver support
---------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The HMC425A Driver depends on **CONFIG_GPIOLIB**

::

   Linux Kernel Configuration
       Device Drivers  --->
           <*>     Industrial I/O support --->
               --- Industrial I/O support
                   Amplifiers  --->
                       <*> Analog Devices HMC425A DSA

Hardware configuration
----------------------

Driver testing
--------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-device-files
   :end-before: .. end-iio-device-files

::

   root:/> cd /sys/bus/iio/devices/
   root:/sys/bus/iio/devices> ls
   iio:device0
   root:/sys/bus/iio/devices> iio:device0

   root:/> ls -l
   drwxr-xr-x    2 root     root             0 Jan  1 00:00 .
   drwxr-xr-x    3 root     root             0 Jan  1 00:00 ..
   -r--r--r--    1 root     root          4096 Jan  1 00:00 dev
   -r--r--r--    1 root     root          4096 Jan  1 00:00 name
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 out_voltage0_hardwaregain
   lrwxrwxrwx    1 root     root             0 Jan  1 00:00 subsystem -> ../../../../../../../../../bus/iio
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 uevent

Show device name
~~~~~~~~~~~~~~~~

::

   root:/> cd /sys/bus/iio/devices/iio\:device0/
   root:/> cat name
   hmc425a

Set ChannelY Gain
^^^^^^^^^^^^^^^^^

/sys/bus/iio/devices/iio:deviceX/out_voltageY_hardwaregain

Hardware applied gain factor. If shared across all channels, <type>_hardwaregain
is used.

::

   root:/> cat out_voltage0_hardwaregain
   -31.500000 dB

   root:/> echo -10 > out_voltage0_hardwaregain
   root:/> cat out_voltage0_hardwaregain
   -10.000000 dB

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
