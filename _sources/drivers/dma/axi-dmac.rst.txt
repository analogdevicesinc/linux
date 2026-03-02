.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/axi-dmac

.. _axi-dmac:

AXI DMAC
========

Analog Device AXI-DMAC DMA Controller Linux Driver.

Supported Devices
-----------------

- :external+hdl:ref:`axi_dmac`

Description
-----------

The AXI DMAC is a high-speed, high-throughput, general purpose DMA controller
intended to be used to transfer data between system memory and other peripherals
like high-speed converters.

Features
~~~~~~~~

- Supports multiple interface types

  - AXI3/4 memory mapped
  - AXI4 Streaming
  - ADI FIFO interface

- Zero-latency transfer switch-over architecture

  - Allows **continuous** high-speed streaming

- Cyclic transfers
- 2D transfers

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
   * - `drivers/dma/dma-axi-dmac.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/dma/dma-axi-dmac.c>`__
     - Yes
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git-linux:`drivers/dma/dma-axi-dmac.c`
     -
   * - include
     - :git-linux:`include/dt-bindings/dma/axi-dmac.h`
     -
   * - Documentation
     - :git-linux:`Documentation/devicetree/bindings/dma/adi,axi-dmac.txt`
     -

Example platform device initialization
--------------------------------------

The AXI-DMAC driver is a platform driver and can currently only be instantiated
via device tree.

**Required properties:**

- compatible: Must be ``adi,axi-dmac-1.00.a``.
- reg: Specification for the controllers memory mapped register map.
- interrupts: Specification for the controllers interrupt.
- clocks: Phandle and specifier to the controllers AXI interface clock
- #dma-cells: Must be 1.

**Required sub-nodes:**

- adi,channels: This sub-node must contain a sub-node for each DMA channel. For
  the channel sub-nodes the following bindings apply. They must match then
  configuration options of the peripheral as it was instantiated.

**Required properties for adi,channels sub-node:**

- #size-cells: Must be 0
- #address-cells: Must be 1

**Required channel sub-node properties:**

- reg: Which channel this node refers to.
- adi,length-width: Width of the DMA transfer length register.
- adi,source-bus-width, adi,destination-bus-width: Width of the source or
  destination bus in bits.
- adi,source-bus-type, adi,destination-bus-type: Type of the source or
  destination bus. Must be one of the following:
- 0 (AXI_DMAC_TYPE_AXI_MM): Memory mapped AXI interface
- 1 (AXI_DMAC_TYPE_AXI_STREAM): Streaming AXI interface
- 2 (AXI_DMAC_TYPE_AXI_FIFO): FIFO interface

**Optional channel properties:**

- adi,cyclic: Must be set if the channel supports hardware cyclic DMA transfers.
- adi,2d: Must be set if the channel supports hardware 2D DMA transfers.

DMA clients connected to the AXI-DMAC DMA controller must use the format
described in the dma.txt file using a one cell specifier. The value of the
specifier refers to the DMA channel index.

Example:

::

   dma: dma@7c420000 {
       compatible = "adi,axi-dmac-1.00.a";
       reg = <0x7c420000 0x10000>;
       interrupts = <0 57 0>;
       clocks = <&clkc 16>;
       #dma-cells = <1>;

       adi,channels {
           #size-cells = <0>;
           #address-cells = <1>;

           dma-channel@0 {
               reg = <0>;
               adi,source-bus-width = <32>;
               adi,source-bus-type = <ADI_AXI_DMAC_TYPE_MM_AXI>;
               adi,destination-bus-width = <64>;
               adi,destination-bus-type = <ADI_AXI_DMAC_TYPE_FIFO>;
           };
       };
   };

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

::

   Linux Kernel Configuration
       Device Drivers  --->
       <*>    DMA --->

               <*>   DMA Engine support  --->
           [--snip--]
           <*>   Analog Devices AXI-DMAC DMA support
           [--snip--]

More Information
~~~~~~~~~~~~~~~~

- `Linux DMAEngine documentation <https://www.kernel.org/doc/html/latest/driver-api/dmaengine/index.html>`__
