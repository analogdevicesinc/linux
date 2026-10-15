.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-dac/axi-ad3552r


.. _ad3552r-hs:

AD3552R High Speed
==================

AD3552R Dual Channel, 16-Bit, 33 MUPS, Multispan, Multi-IO SPI DAC Linux device driver.

The AD3552R is a low drift, dual channel, ultra-fast, 16-bit accuracy, current
output digital-to-analog converter (DAC) that can be configured in multiple
voltage span ranges. The AD3552R operates with a fixed 2.5 V reference. Each DAC
incorporates three drift-compensating feedback resistors for the required
external trans-impedance amplifier (TIA) that scales the output voltage. Offset
and gain scaling registers allow for the generation of multiple output span
ranges, such as 0 V to 2.5 V, 0 V to 5 V, 0 V to 10 V, −5 V to +5 V, and −10 V
to +10 V, and custom intermediate ranges with full 16-bit resolution. The DAC
can operate in fast mode for maximum speed or precision mode for maximum
accuracy.

Supported Devices
-----------------

- :adi:`AD3541R`
- :adi:`AD3542R`
- :adi:`AD3551R`
- :adi:`AD3552R`

Evaluation Boards
-----------------

- :adi:`EVAL-AD3552RFMC1Z <EVAL-AD3552R>`
- :adi:`EVAL-AD3552RFMC2Z <EVAL-AD3552R>`

Description
-----------

This is a Linux industrial I/O (:external+system-level:ref:`iio`) subsystem
driver for the high-speed variant of the AD35xxR DACs. The DAC is driven
through the AXI AD3552R FPGA IP core, which implements the (dual or quad) SPI
data lanes, DDR streaming and DMA transfers needed to reach the maximum sample
rate.

The driver is split in two parts using the IIO backend framework:

- ``adi-axi-dac``: the AXI AD3552R IP core driver (backend). It also acts as
  the SPI-like bus controller for the DAC.
- ``ad3552r-hs``: the DAC driver (frontend), probed from a child node of the
  IP core node.

This driver replaces the former out-of-tree ``axi-ad3552r`` driver
(compatible ``adi,axi-ad3552r`` handled by a single driver). The device tree
layout and the sysfs interface differ from that driver.

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
   * - :git+linux:`main:drivers/iio/dac/ad3552r-hs.c`
     - Yes
   * - :git+linux:`main:drivers/iio/dac/adi-axi-dac.c`
     - Yes

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - DAC driver (frontend)
     - :git+linux:`main:drivers/iio/dac/ad3552r-hs.c`
   * - common code
     - :git+linux:`main:drivers/iio/dac/ad3552r-common.c`
   * - include
     - :git+linux:`main:drivers/iio/dac/ad3552r.h`
   * - AXI DAC IP core driver (backend)
     - :git+linux:`main:drivers/iio/dac/adi-axi-dac.c`
   * - kernel documentation
     - :git+linux:`main:Documentation/iio/ad3552r.rst`

Adding Linux driver support
---------------------------

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

::

   Linux Kernel Configuration
       Device Drivers  --->
           <*>     Industrial I/O support --->
                     Digital to analog converters  --->
                         [--snip--]
                         <*>   Analog Devices AD3552R DAC High Speed driver
                         [--snip--]
                         <*>   Analog Devices Generic AXI DAC IP core driver
                         [--snip--]

Devicetree configuration
~~~~~~~~~~~~~~~~~~~~~~~~

The AXI AD3552R IP core node (backend):

- **compatible**: ``adi,axi-ad3552r``
- **reg**: IP core register region.
- **dmas**, **dma-names**: Transmit DMA channel, ``"tx"``.
- **clocks**, **clock-names**: ``s_axi_aclk`` (AXI bus clock) and ``dac_clk``
  (DAC interface clock).
- **#io-backend-cells**: ``<0>``.

The DAC child node (frontend):

- **compatible**: ``adi,ad3541r``, ``adi,ad3542r``, ``adi,ad3551r`` or
  ``adi,ad3552r``.
- **reg**: ``<0>``.
- **spi-max-frequency**: Maximum frequency for register access.
- **io-backends**: Phandle to the AXI AD3552R IP core node.
- **channel@N** sub-nodes with ``adi,output-range-microvolt`` (or
  ``custom-output-range-config``).

Optional properties include ``reset-gpios``, ``ldac-gpios``, ``vref-supply``
and ``adi,vref-out-en``. See
:git+linux:`main:Documentation/devicetree/bindings/iio/dac/adi,axi-dac.yaml`
and :git+linux:`main:Documentation/devicetree/bindings/iio/dac/adi,ad3552r.yaml`.

Example (from the AXI DAC binding):

.. code-block:: dts

   #include <dt-bindings/gpio/gpio.h>

   axi_dac: spi@44a70000 {
       compatible = "adi,axi-ad3552r";
       reg = <0x44a70000 0x1000>;
       dmas = <&dac_tx_dma 0>;
       dma-names = "tx";
       #io-backend-cells = <0>;
       clocks = <&clkc 15>, <&ref_clk>;
       clock-names = "s_axi_aclk", "dac_clk";

       #address-cells = <1>;
       #size-cells = <0>;

       dac@0 {
           compatible = "adi,ad3552r";
           reg = <0>;
           reset-gpios = <&gpio0 92 GPIO_ACTIVE_HIGH>;
           io-backends = <&axi_dac>;
           spi-max-frequency = <20000000>;

           #address-cells = <1>;
           #size-cells = <0>;

           channel@0 {
               reg = <0>;
               adi,output-range-microvolt = <(-10000000) (10000000)>;
           };
       };
   };

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - :adi:`EVAL-AD3552R` Device Tree
     - :git+linux:`main:arch/arm/boot/dts/xilinx/zynq-zed-adv7511-ad3552r-hs.dts`

Driver testing
--------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-device-files
   :end-before: .. end-iio-device-files

Identify the device
~~~~~~~~~~~~~~~~~~~

The IIO device name is ``ad3552r`` for all supported variants.

::

   root:/sys/bus/iio/devices/iio:device1> cat name
   ad3552r

Channel attributes
~~~~~~~~~~~~~~~~~~

Each of the two output channels exposes:

- **out_voltageN_raw**: DAC code. Writes are rejected with ``EBUSY`` while the
  buffer is enabled.
- **out_voltageN_scale**, **out_voltageN_offset**: Conversion to millivolts,
  ``(raw + offset) * scale``, derived from the configured output range.
- **out_voltageN_sampling_frequency**: Read-only streaming sample rate, in Hz.
  It is computed from the ``dac_clk`` rate and the number of SPI data lanes:
  ``dac_clk / 4`` for the quad-SPI AD3551R/AD3552R and ``dac_clk / 8`` for the
  dual-SPI AD3541R/AD3542R. Writes return ``EINVAL``.

Data source (debugfs)
~~~~~~~~~~~~~~~~~~~~~

With CONFIG_DEBUG_FS enabled, ``data_source`` and ``data_source_available`` in
``/sys/kernel/debug/iio/iio:deviceX/`` select the streamed data source, for
example ``normal`` or ``ramp-16bit``.

.. code-block:: bash

   root:/sys/kernel/debug/iio/iio:device1# cat data_source
   normal
   root:/sys/kernel/debug/iio/iio:device1# echo -n ramp-16bit > data_source
   root:/sys/kernel/debug/iio/iio:device1# cat data_source
   ramp-16bit

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
