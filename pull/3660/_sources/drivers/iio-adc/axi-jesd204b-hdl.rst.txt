.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-adc/axi-jesd204b-hdl


.. _axi-jesd204b-hdl:

AXI JESD204B HDL
================

AXI JESD204B HDL Linux Driver.

.. warning::

   This driver is RETIRED, DEPRECATED and not used anymore in any of the recent
   projects!

   Please see here instead:

   * :ref:`axi_jesd204_tx`
   * :ref:`axi_jesd204_rx`

Supported IP Cores
------------------

#. deprecated

Description
-----------

The AXI JESD204 HDL driver is the driver for the HDL interface core which is
used on various FPGA designs interfacing to the :adi:`AD9250`. The
driver is implemented as an Open Firmware Device Tree (DT) platform driver. It’s
register map can be found here:
:external+hdl:ref:`generic-adc-register-access`

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
   * - :git+linux:`main:drivers/iio/jesd204/axi_jesd204b_v51.c`
     - No
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
     - :git+linux:`main:drivers/iio/jesd204/axi_jesd204b_v51.c`
     -
   * - include
     - :git+linux:`main:drivers/iio/jesd204/axi_jesd204b_v51.h`
     -

Adding Linux driver support
---------------------------

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

::

   Linux Kernel Configuration
       Device Drivers  --->
           [--snip--]
           <*> JESD204 High-Speed Serial Interface Framework  --->
           [--snip--]
           <*> Industrial I/O support  --->
               [--snip--]
               <*>   Generic AXI JESD204B configuration driver
               [--snip--]

Devicetree configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Example platform device initialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The AXI JESD204B driver is a platform driver and can currently only be
instantiated via device tree.

Required devicetree properties:

- **compatible**: ``xlnx,jesd204-5.1``, ``xlnx,jesd204-5.2`` or
  ``xlnx,jesd204-6.0``, matching the Xilinx JESD204 IP core version
- **reg**: Base address and register area size
- **clocks**: Input clock phandle
- **clock-output-names**: Name of the clock output registered by the driver
- **xlnx,frames-per-multiframe**: Number of frames per multi-frame
- **xlnx,bytes-per-frame**: Number of bytes (octets) per frame
- **xlnx,subclass**: JESD204B subclass

Optional devicetree properties:

- **xlnx,lanesync-enable**: Enable lane synchronization (boolean)
- **xlnx,scramble-enable**: Enable scrambling (boolean)
- **xlnx,sysref-always-enable**: Enable SYSREF always mode (boolean)
- **xlnx,node-is-transmit**: Set to ``<1>`` for a transmit link (u32)
- **xlnx,lanes**: Number of lanes; read from the core if omitted

Example (adjust the clock provider and link parameters for your board):

::

   axi_jesd204b_rx2_0: axi-jesd204b-rx2@77a00000 {
       compatible = "xlnx,jesd204-5.1";
       reg = <0x77a00000 0x10000>;
       clocks = <&ad9250_clkin>;
       clock-output-names = "jesd_clk";
       #clock-cells = <0>;
       xlnx,frames-per-multiframe = <32>;
       xlnx,bytes-per-frame = <2>;
       xlnx,subclass = <1>;
       xlnx,lanesync-enable;
       xlnx,scramble-enable;
   };

Driver testing
--------------

::

   # cd /sys/bus/platform/devices/44a91000.jesd204/
   # ls -al
   total 0
   drwxr-xr-x    2 root     root             0 Jan  1 00:21 .
   drwxr-xr-x   25 root     root             0 Jan  1 00:00 ..
   lrwxrwxrwx    1 root     root             0 Jan  1 00:21 driver -> ../../../bus/platform/drivers/cf_axi_jesd204b_v51
   -rw-r--r--    1 root     root          4096 Jan  1 00:21 driver_override
   -r--------    1 root     root          4096 Jan  1 00:21 lane0_info
   -r--------    1 root     root          4096 Jan  1 00:21 lane0_syncstat
   -r--------    1 root     root          4096 Jan  1 00:21 lane1_info
   -r--------    1 root     root          4096 Jan  1 00:21 lane1_syncstat
   -r--------    1 root     root          4096 Jan  1 00:21 lane2_info
   -r--------    1 root     root          4096 Jan  1 00:21 lane2_syncstat
   -r--------    1 root     root          4096 Jan  1 00:21 lane3_info
   -r--------    1 root     root          4096 Jan  1 00:21 lane3_syncstat
   -r--r--r--    1 root     root          4096 Jan  1 00:21 modalias
   -rw-------    1 root     root          4096 Jan  1 00:21 reg_access
   lrwxrwxrwx    1 root     root             0 Jan  1 00:21 subsystem -> ../../../bus/platform
   -rw-r--r--    1 root     root          4096 Jan  1 00:21 uevent

   # cat lane0_info
   DID: 0, BID: 1, LID: 1, L: 4, SCR: 1, F: 1
   K: 32, M: 2, N: 14, CS: 0, S: 1, N': 16, HD: 1
   FCHK: 0x81, CF: 0
   ADJCNT: 0, PHYADJ: 0, ADJDIR: 0, JESDV: 1, SUBCLASS: 1
   MFCNT : 0x0
   ILACNT: 0x0
   ERRCNT: 0x0
   BUFCNT: 0x14
   LECNT: 0x0
   FC: 500000000
   #
   # cat lane0_syncstat
   NOT_IN_TAB: 0, DISPARITY: 0, UNEXPECTED_K: 0
   #
