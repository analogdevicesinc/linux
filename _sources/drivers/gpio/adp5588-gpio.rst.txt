.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/gpio/adp5588-gpio


.. _adp5588-gpio:

ADP5588 GPIO
============

The standalone ``gpio-adp5588`` driver was removed from the kernel. ADP5588
GPIO support is now provided by ``drivers/input/keyboard/adp5588-keys.c``
(``CONFIG_KEYBOARD_ADP5588``). For GPIO-only operation, use
``compatible = "adi,adp5588"`` (or ``"adi,adp5587"``), ``gpio-controller``
and ``#gpio-cells = <2>``, and omit ``keypad,num-rows`` and
``keypad,num-columns``.

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
~~~~~~~~~~~~~

.. figure:: https://wiki.analog.com/_media/software/driver/linux/adp5588_typ_op.png
   :width: 600px

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

Adding Linux driver support
---------------------------

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

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

Devicetree configuration
~~~~~~~~~~~~~~~~~~~~~~~~

For GPIO-only operation, declare the device with ``gpio-controller`` and
``#gpio-cells = <2>`` and without ``keypad,num-rows``/``keypad,num-columns``:

.. code-block:: dts

   #include <dt-bindings/gpio/gpio.h>

   &i2c0 {
       #address-cells = <1>;
       #size-cells = <0>;

       gpio@34 {
           compatible = "adi,adp5588";
           reg = <0x34>;
           #gpio-cells = <2>;
           gpio-controller;
       };
   };

Driver testing
--------------

When the ``adp5588-keys`` driver probes, the ADP5588 GPIOs are registered as
a gpiochip. The examples below use the sysfs GPIO interface
(``CONFIG_GPIO_SYSFS``); GPIO numbers depend on the system.

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
