.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/spi/spi_engine

.. _spi_engine:

SPI Engine
==========

SPI Engine Peripheral Linux Driver.

Supported Devices
-----------------

- :external+hdl:ref:`spi_engine`

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
   * - :git+linux:`git <main:drivers/spi/spi-axi-spi-engine.c>`
     - `yes (No offloading support) <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/spi/spi-axi-spi-engine.c>`__
     -
     -

Files
^^^^^

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git+linux:`spi-axi-spi-engine.c <main:drivers/spi/spi-axi-spi-engine.c>`
     -

Device initialization
---------------------

Devicetree bindings
~~~~~~~~~~~~~~~~~~~

**Required properties:**

- **compatible**: Must be ``adi,axi-spi-engine-1.00.a````
- **reg**: Physical base address and size of the register map.
- **interrupts**: Property with a value describing the interrupt number.
- **clock-names**: List of input clock names - ``s_axi_aclk``, ``spi_clk``
- **clocks**: Clock phandles and specifiers (See clock bindings for details on
  clock-names and clocks).
- **#address-cells**: Must be <1>
- **#size-cells**: Must be <0>

**Optional subnodes**

Subnodes are use to represent the SPI slave devices connected to the SPI master.
They follow the generic SPI bindings as outlined in spi-bus.txt.

Example
^^^^^^^

::

   spi@@44a00000 {
       compatible = "adi,axi-spi-engine-1.00.a";
       reg = <0x44a00000 0x1000>;
       interrupts = <0 56 4>;
       clocks = <&clkc 15 &clkc 15>;
       clock-names = "s_axi_aclk", "spi_clk";
       #address-cells = <1>;
       #size-cells = <0>;
       /* SPI devices */
   };

More information
~~~~~~~~~~~~~~~~

-  :external+hdl:ref:`spi_engine`

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
