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

Supported Devices
-----------------

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
   * - :git-linux:`drivers/iio/jesd204/axi_jesd204b_v51.c`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/jesd204/axi_jesd204b_v51.c>`__
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
     - :git-linux:`drivers/iio/jesd204/axi_jesd204b_v51.c`
     -
   * - include
     - :git-linux:`drivers/iio/jesd204/axi_jesd204b_v51.h`
     -

Example platform device initialization
--------------------------------------

The AXI JESD204B driver is a platform driver and can currently only be
instantiated via device tree.

Required devicetree properties:

- **compatible**: Should always be ``*xlnx,axi-jesd204b-rx2-1.00.a*`` or
  ``*xlnx,axi-jesd204b-rx4-1.00.a*``
- **reg**: Base address and register area size. This parameter expects a
  register range.
- **clocks**: Clock provider phandle
- **clock-name**: Clock input name string
- **jesd,frames-per-multiframe**: Number of frames per multi-frame
- **jesd,bytes-per-frame**: Number of bytes (octets) per frame
- **jesd,lanesync_en**: Enable lane synchronization
- **jesd,scramble_en**: Enable scrambling

Example:

::

   axi_jesd204b_rx2_0: axi-jesd204b-rx2@77a00000 {
       compatible = "xlnx,axi-jesd204b-rx2-1.00.a";
       reg = < 0x77a00000 0x10000 >;
       clocks = <&ad9250_clkin>;
       clock-names = "clkin";
       jesd,lanesync_en;
       jesd,scramble_en;
       jesd,frames-per-multiframe = <32>;
       jesd,bytes-per-frame = <2>;
       xlnx,cf-buftype = <0x0>;
       xlnx,dphase-timeout = <0x8>;
       xlnx,family = "kintex7";
       xlnx,num-mem = <0x1>;
       xlnx,num-reg = <0x1>;
       xlnx,s-axi-min-size = <0x1ff>;
       xlnx,slv-awidth = <0x20>;
       xlnx,slv-dwidth = <0x20>;
       xlnx,use-wstrb = <0x0>;
   } ;

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

Adding Linux driver support
---------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

::

   Linux Kernel Configuration
       Device Drivers  --->
       <*>     Industrial I/O support --->
           --- Industrial I/O support
           -*-   Enable ring buffer support within IIO
           -*-     Industrial I/O lock free software ring
           -*-   Enable triggered sampling support

                 *** Analog to digital converters ***
           [--snip--]

           <*>   Generic AXI JESD204B configuration driver

           [--snip--]

Driver Testing
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
