.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/gpio/adp5588-gpio

.. _adp5588-gpio:

ADP5588 GPIO
============

ADP5588 GPIO Linux Driver.

Supported Devices
-----------------

- :adi:`ADP5587`
- :adi:`ADP5588`

Evaluation Boards
-----------------

- :adi:`ADP5588-EVALZ`
- :adi:`ADP5587CB-EVALZ`
- :adi:`ADP5587CP-EVALZ`

Description
-----------

Configuration
-------------

.. figure:: https://wiki.analog.com/_media/software/driver/linux/adp5588_typ_op.png
   :width: 600px

Software configurable features
------------------------------

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/gpio/gpio-adp5588.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/gpio/gpio-adp5588.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - driver
     - `drivers/gpio/gpio-adp5588.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/gpio/gpio-adp5588.c>`__
   * - include
     - `include/linux/i2c/adp5588.h <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/include/linux/i2c/adp5588.h>`__

Example platform device initialization
--------------------------------------

.. include:: /include/platform-and-bus-model.rst
   :start-after: .. start-platform-data
   :end-before: .. end-platform-data

:git-linux:`/i2c_board_info/-EOF <include/linux/i2c/adp5588.h>`

:git-linux:`adp5588_gpio_data{} <arch/blackfin/mach-bf537/boards/stamp.c>`

.. include:: /include/platform-and-bus-model.rst
   :start-after: .. start-declaring-i2c-devices
   :end-before: .. end-declaring-i2c-devices

.. code:: C

   static struct i2c_board_info __initdata bfin_i2c_board_info[] = {
   #if defined(CONFIG_GPIO_ADP5588) || defined(CONFIG_GPIO_ADP5588_MODULE)
       {
           I2C_BOARD_INFO("adp5588-gpio", 0x34),
           .irq = IRQ_PG0,
           .platform_data = (void *)&adp5588_gpio_data,
       },
   #endif
   }

Adding Linux driver support
---------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The ADP5588 Driver depends on **CONFIG_I2C** IRQ-Chip interrupt controller
   support is not available in case this driver is build as Module

::

   Device Drivers  --->
   [*] GPIO Support  --->

       --- GPIO Support
       [ ]   Debug GPIO calls
       [*]   /sys/class/gpio/... (sysfs interface)
             *** Memory mapped GPIO expanders: ***
       < >   IT8761E GPIO support
             *** I2C GPIO expanders: ***
       < >   Maxim MAX7300 GPIO expander
       < >   MAX7319, MAX7320-7327 I2C Port Expanders
       < >   PCA953x, PCA955x, TCA64xx, and MAX7310 I/O ports
       < >   PCF857x, PCA{85,96}7x, and MAX732[89] I2C GPIO expanders
       [ ]   Semtech SX150x I2C GPIO expander
       < >   GPIO Support for ADP5520 PMIC
       <*>   ADP5588 I2C GPIO expander
       [*]     Interrupt controller support for ADP5588

Hardware configuration
----------------------

.. figure:: https://wiki.analog.com/_media/software/driver/linux/adp5588_demo_brd_and_stamp.jpg
   :width: 600px

There is no dedicated Blackfin STAMP evaluation board for the ADP5588. During
test and driver development we used the ADP5588 Demo Mother/Daughter Board.

It can be easily wired to the Blackfin STAMP TWI/I2C header.

.. list-table::
   :header-rows: 1

   * - BF537-STAMP (P10) TWI/I2C header
     -
     - ADP5588 Daughter Board
   * - PIN
     - Function
     - PIN/Function
   * - 2
     - (+3.3V)
     - VCC
   * - 5
     - SCL
     - SCL
   * - 6
     - SDA
     - SDA
   * - 10
     - PORTG0
     - INTB
   * - 20
     - GND
     - GND

.. tip::

   On the ADP5588 Demo Mother Board replace R30 (10kOhm PULL-UP resistor on
   /INTB strobe) with a 1-3kOhm resistor. The 10kOhm resistor is too weak -
   Blackfin might see an additional falling edge interrupt on the rising edge of
   /INTB.

Driver testing
--------------

When the driver is loaded, you should see positive output that it found the
ADP5588 GPIO device.

::

   root:/> modprobe adp5588-gpio
   adp5588-gpio 0-0034: gpios 50..67 on a adp5588-gpio Rev. 4

For more information see also here:
:external+upstream:doc:`userspace-api/gpio/sysfs`

::

   root:/> echo 67 > /sys/class/gpio/export

   root:/> echo low > /sys/class/gpio/gpio67/direction
   root:/> echo high > /sys/class/gpio/gpio67/direction

   root:/> echo in > /sys/class/gpio/gpio67/direction
   root:/> cat /sys/class/gpio/gpio67/value
   1
   root:/> cat /sys/class/gpio/gpio67/value
   0
   root:/

More Information
----------------
