.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/misc/dpot

.. _dpot:

Digital Potentiometer
=====================

Digital Potentiometer Linux Driver.

Supported Devices
-----------------

- :adi:`AD5160`
- :adi:`AD5161`
- :adi:`AD5165`
- :adi:`AD5170`
- :adi:`AD5171`
- :adi:`AD5172`
- :adi:`AD5201`
- :adi:`AD5203`
- :adi:`AD5204`
- :adi:`AD5206`
- :adi:`AD5207`
- :adi:`AD5231`
- :adi:`AD5232`
- :adi:`AD5233`
- :adi:`AD5235`
- :adi:`AD5242`
- :adi:`AD5243`
- :adi:`AD5245`
- :adi:`AD5246`
- :adi:`AD5247`
- :adi:`AD5248`
- :adi:`AD5251`
- :adi:`AD5252`
- :adi:`AD5253`
- :adi:`AD5254`
- :adi:`AD5255`
- :adi:`AD5258`
- :adi:`AD5259`
- :adi:`AD5260`
- :adi:`AD5262`
- :adi:`AD5270`
- :adi:`AD5271`
- :adi:`AD5272`
- :adi:`AD5273`
- :adi:`AD5274`
- :adi:`AD5280`
- :adi:`AD5282`
- :adi:`AD5290`
- :adi:`AD5291`
- :adi:`AD5292`
- :adi:`AD5293`
- :adi:`AD7376`
- :adi:`AD8400`
- :adi:`AD8402`
- :adi:`AD8403`
- :adi:`ADN2850`
- :adi:`ADN2860`

Reference Circuits
------------------

- :adi:`CN0287`
- :adi:`CN0357`

Evaluation Boards
-----------------

- :adi:`EVAL-AD5160EBZ`
- :adi:`EVAL-AD5161EBZ`
- :adi:`EVAL-AD5165EBZ`
- :adi:`EVAL-AD5171DBZ`
- :adi:`EVAL-AD5172SDZ`
- :adi:`EVAL-AD5232SDZ`
- :adi:`EVAL-AD5235SDZ`
- :adi:`EVAL-AD5242EBZ`
- :adi:`EVAL-AD5243SDZ`
- :adi:`EVAL-AD5245EBZ`
- :adi:`EVAL-AD5246DBZ`
- :adi:`EVAL-AD5247DBZ`
- :adi:`EVAL-AD5252SDZ`
- :adi:`EVAL-AD5254SDZ`
- :adi:`EVAL-AD5258DBZ`
- :adi:`EVAL-AD5259DBZ`
- :adi:`EVAL-AD5262EBZ`
- :adi:`EVAL-AD5270SDZ`
- :adi:`EVAL-AD5272SDZ`
- :adi:`EVAL-AD5273DBZ`
- :adi:`EVAL-AD5282EBZ`
- :adi:`EVAL-AD5290EBZ`
- :adi:`EVAL-AD5292EBZ`
- :adi:`EVAL-AD7376EBZ`
- :adi:`EVAL-AD8403SDZ`
- :adi:`EVAL-ADN2850SDZ`
- PMOD-DPOT

:git-linux:`/DEVID/-/Documentation/ <drivers/misc/ad525x_dpot.c>`

Description
-----------

The ad525x_dpot driver exports a simple sysfs interface. This allows you to work
with the immediate resistance settings as well as update the saved startup
settings. Access to the factory programmed tolerance is also provided, but
interpretation of this settings is required by the end application according to
the specific part in use.

Files
~~~~~

Each dpot device will have a set of **eeprom**, **rdac**, and **tolerance**
files. How many depends on the actual part you have, as will the range of
allowed values.

The **eeprom** files are used to program the startup value of the device.

The **rdac** files are used to program the immediate value of the device.

The **tolerance** files are the read-only factory programmed tolerance settings
and may vary greatly on a part-by-part basis. For exact interpretation of this
field, please consult the datasheet for your part. This is presented as a hex
file for easier parsing.

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/misc/ad525x_dpot.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/misc/ad525x_dpot.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - driver
     - `drivers/misc/ad525x_dpot.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/misc/ad525x_dpot.c>`__
   * - i2c bus support
     - `drivers/misc/ad525x_dpot-i2c.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/misc/ad525x_dpot-i2c.c>`__
   * - spi bus support
     - `drivers/misc/ad525x_dpot-spi.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/misc/ad525x_dpot-spi.c>`__
   * - include
     - `drivers/misc/ad525x_dpot.h <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/misc/ad525x_dpot.h>`__

Example platform device initialization
--------------------------------------

.. include:: /include/platform-and-bus-model.rst
   :start-after: .. start-platform-data
   :end-before: .. end-platform-data

Example Platform / Board file (I2C Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. include:: /include/platform-and-bus-model.rst
   :start-after: .. start-declaring-i2c-devices
   :end-before: .. end-declaring-i2c-devices

.. code:: C

   static struct i2c_board_info __initdata bfin_i2c_board_info[] = {
   #if defined(CONFIG_AD525X_DPOT) || defined(CONFIG_AD525X_DPOT_MODULE)
       {
           I2C_BOARD_INFO("ad5245", 0x2c),
       },
       {
           I2C_BOARD_INFO("ad5245", 0x2d),
       },
   #endif
   }

Example Platform / Board file (SPI Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. include:: /include/platform-and-bus-model.rst
   :start-after: .. start-declaring-spi-slave-devices
   :end-before: .. end-declaring-spi-slave-devices

.. code:: C

   static struct spi_board_info bfin_spi_board_info[] __initdata = {
   #if defined(CONFIG_AD525X_DPOT) || defined(CONFIG_AD525X_DPOT_MODULE)
       {
           .modalias = "ad5291",
           .max_speed_hz = 5000000,     /* max spi clock (SCK) speed in HZ */
           .bus_num = 0,
           .chip_select = 1,
       },
   #endif
   };

.. tip::

   *Old Method*

   .. code:: C

      static struct spi_board_info bfin_spi_board_info[] __initdata = {
      #if defined(CONFIG_AD525X_DPOT) || defined(CONFIG_AD525X_DPOT_MODULE)
          {
              .modalias = "ad_dpot",
              .platform_data = "ad5291",
              .max_speed_hz = 5000000,     /* max spi clock (SCK) speed in HZ */
              .bus_num = 0,
              .chip_select = 1,
          },
      #endif
      };

Adding Linux driver support
---------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The ad525x_dpot driver depends on **CONFIG_SPI** or **CONFIG_I2C**

::

   Device Drivers  --->
     [*] Misc devices  --->
       <*>   Analog Devices Digital Potentiometers
       <*>     support I2C bus connection
       <*>     support SPI bus connection

Hardware configuration
----------------------

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/misc/pmoddpot_lr.png
   :width: 600px

Driver testing
--------------

Locate the device in your sysfs tree. This is probably easiest by going into the
common i2c directory and locating the device by the i2c slave address.

::

   # ls /sys/bus/i2c/devices/
   0-0022  0-0027  0-002f

So assuming the device in question is on the first i2c bus and has the slave
address of 0x2f, we descend (unrelated sysfs entries have been trimmed).

::

   # ls /sys/bus/i2c/devices/0-002f/
   eeprom0 rdac0 tolerance0

You can use simple reads/writes to access these files:

::

   # cd /sys/bus/i2c/devices/0-002f/

   # cat eeprom0
   0
   # echo 10 > eeprom0
   # cat eeprom0
   10

   # cat rdac0
   5
   # echo 3 > rdac0
   # cat rdac0
   3

More Information
----------------
