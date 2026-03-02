.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/serial/max310x

.. _max310x:

MAX310x
=======

MAX310x Driver.

Supported devices
-----------------

:adi:`MAX3107 <en/products/max3107.html>`

:adi:`MAX3108 <en/products/max3108.html>`

:adi:`MAX3109 <en/products/max3109.html>`

:adi:`MAX14830 <en/products/max14830.html>`

Evaluation boards
-----------------

:adi:`MAX3107EVKIT <en/products/max3107.html#product-evaluationkit>`

:adi:`MAX14830EVKIT <en/products/max14830.html#product-evaluationkit>`

Overview
--------

The MAX310x devices are advanced UART with 128 words each of receive and
transmit FIFO that can be controlled through I²C or high-speed SPI™. Baud rates
up to 24Mbps make them suitable for today’s high data rate applications.

A PLL, predivider, and fractional baud-rate generator allow high-resolution
baud-rate programming and minimize the dependency of baud rate on reference
clock frequency.

The :adi:`MAX3109 <en/products/interface/controllers-expanders/MAX3109.html>`
is a dual-channel device, while the
:adi:`MAX14830 <en/products/interface/controllers-expanders/MAX14830.html>` is
quad-channel.

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/tty/serial/max310x.c>`__
     - Yes
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - driver
     - `drivers/tty/serial/max310x.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/tty/serial/max310x.c>`__
   * - Documentation
     - `Documentation/devicetree/bindings/serial/maxim,max310x.txt <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/serial/maxim,max310x.txt>`__

Example device tree

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - i2c dts
     - :git-linux:`rpi-max14830-i2c-overlay.dts <rpi-5.10.y:arch/arm/boot/dts/overlays/rpi-max14830-i2c-overlay.dts>`
     -
   * - spi dts
     - :git-linux:`rpi-max14830-spi-overlay.dts <rpi-5.10.y:arch/arm/boot/dts/overlays/rpi-max14830-spi-overlay.dts>`
     -

Driver testing
--------------

The driver exposes character devices starting with the ttyMAX name.

To connect to these devices, use a Serial Terminal Emulaor (like GTKTerm,
Picocom, Minicom, Tera Term).

For example, to use Picocom to connect to UART port 0 of a MAX14830 device using
a 9600 baud-rate, use the following command.

::

   picocom -b 9600 /dev/ttyMAX0
