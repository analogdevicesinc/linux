.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-adc/axi-jesd204b-gt-hdl

.. _axi-jesd204b-gt-hdl:

AXI JESD204B GT HDL
===================

AXI JESD204B GT HDL Linux Driver.

.. warning::

   This driver is RETIRED, DEPRECATED and not used anymore in any of the recent
   projects!

   Please see here instead: :ref:`axi_adxcvr`

Supported Devices
-----------------

This driver supports the

- Deprecated

Description
-----------

The AXI JESD204 GT HDL driver is the driver for the Gigabit Tranceiver (GTX,
GTH, etc.) HDL interface core which is used on various FPGA designs. The driver
is implemented as an Open Firmware Device Tree (DT) platform driver. It’s
register map can be found here:
:external+hdl:ref:`generic-adc-register-access`

This driver is used by the JESD Eye Scan - See here
:dokuwiki:`JESD204 Eye Scan </resources/tools-software/linux-software/jesd_eye_scan>`

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
   * - :git-linux:`drivers/iio/jesd204/axi_jesd204b_gt.c`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/jesd204/axi_jesd204b_gt.c>`__
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
     - :git-linux:`drivers/iio/jesd204/axi_jesd204b_gt.c`
     -
   * - include
     - :git-linux:`drivers/iio/jesd204/axi_jesd204b_gt.h`
     -

Example platform device initialization
--------------------------------------

The AXI JESD204B driver is a platform driver and can currently only be
instantiated via device tree. Depending on which HDL Version (PCORE_VERSION) the
HDL supports two links, one receive and one transmit, depending on
[rx|tx]-[sys|out]-clk-select is given both or not.

Starting with PCORE_VERSION > 7.0.0, the HDL and driver supports multiple links.
And the setup is done per lane basis. The device tree entry then features
multiple per link child nodes.

Required devicetree properties (legacy (flat) mode & PCORE_VERSION < 7.0.0):

- **compatible**: Should always be ``*xlnx,axi-jesd-gt-1.0*`` or
  ``*adi,axi-jesd-gt-1.0*``
- **reg**: Base address and register area size. This parameter expects a
  register range.
- **clocks**: Clock provider phandle
- **clock-name**: Clock input name string
- **adi,lanes**: Number of JESD lanes used
- **adi,rx-sys-clk-select**: RX SYS Clock Select
- **adi,rx-out-clk-select**: RX OUT Clock Select
- **adi,tx-sys-clk-select**: TX SYS Clock Select
- **adi,tx-out-clk-select**: TX OUT Clock Select
- **adi,use-cpll-enable**: Use CPLL, if not enabled driver defaults to QPLL
- **adi,use-lpm-enable**: Use LPM mode, if not enabled driver defaults to DFE
- **adi,sysref-external-enable**: Use external sysref, if not enabled driver
  defaults to internal.

Required devicetree properties (PCORE_VERSION > 7.0.0):

- **compatible**: Should always be ``*xlnx,axi-jesd-gt-1.0*`` or
  ``*adi,axi-jesd-gt-1.0*``
- **reg**: Base address and register area size. This parameter expects a
  register range.
- **clocks**: Clock provider phandle
- **clock-name**: Clock input name string
- **adi,lanes**: Number of JESD lanes used
- **adi,first-lane**: First lane used with this link
- **adi,link-is-transmit-enable**: Link is transmit direction.
- **adi,sys-clk-select**: RX SYS Clock Select
- **adi,out-clk-select**: RX OUT Clock Select
- **adi,use-cpll-enable**: Use CPLL, if not enabled driver defaults to QPLL
- **adi,use-lpm-enable**: Use LPM mode, if not enabled driver defaults to DFE
- **adi,sysref-external-enable**: Use external sysref, if not enabled driver
  defaults to internal.

**Example:**

::

   axi_daq2_gt: axi-jesd-gt-rx-tx@44a60000 {
       #address-cells = <1>;
       #size-cells = <0>;
       #clock-cells = <1>;
       compatible = "xlnx,axi-jesd-gt-1.0";
       reg = < 0x44a60000 0x10000 >;

       axi_daq2_gt_rx0:link@0 {
           #clock-cells = <0>;
           clocks = <&clk0_ad9523 1>, <&clk0_ad9523 6>;
           clock-names = "conv", "sysref";
           clock-output-names = "adc_gt_clk";
           reg = <0>;
           adi,lanes = <4>;
           adi,first-lane = <0>;
           adi,sys-clk-select = <3>;
           adi,out-clk-select = <4>;
           adi,use-lpm-enable;
       };

       axi_daq2_gt_tx0:link@1 {
           #clock-cells = <0>;
           clocks = <&clk0_ad9523 7>, <&clk0_ad9523 13>;
           clock-names = "conv", "sysref";
           clock-output-names = "dac_gt_clk";
           reg = <0>;
           adi,lanes = <4>;
           adi,first-lane = <0>;
           adi,link-is-transmit-enable;
           adi,sys-clk-select = <3>;
           adi,out-clk-select = <4>;
           adi,use-lpm-enable;
       };
   } ;

**Example (legacy mode):**

::

   axi_daq2_gt: axi-jesd-gt-rx-tx@44a60000 {
       #clock-cells = <1>;
       compatible = "xlnx,axi-jesd-gt-1.0";
       reg = < 0x44a60000 0x10000 >;

       clocks = <&clk0_ad9523 1>, <&clk0_ad9523 6>, <&clk0_ad9523 7>, <&clk0_ad9523 13>;
       clock-names = "adc_clk", "adc_sysref", "dac_sysref", "dac_clk";
       clock-output-names = "adc_gt_clk", "dac_gt_clk";

       adi,rx-sys-clk-select = <3>;
       adi,rx-out-clk-select = <4>;
       adi,tx-sys-clk-select = <3>;
       adi,tx-out-clk-select = <4>;

       adi,lanes = <0x4>;
       adi,use-lpm-enable;
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

   # cd 44a60000.axi-jesd-gt/

   # ls -al
   total 0
   drwxr-xr-x    2 root     root             0 Jan  1 00:21 .
   drwxr-xr-x   25 root     root             0 Jan  1 00:00 ..
   lrwxrwxrwx    1 root     root             0 Jan  1 00:24 driver -> ../../../bus/platform/drivers/cf_axi_jesd204b_gt
   -rw-r--r--    1 root     root          4096 Jan  1 00:24 driver_override
   --w-------    1 root     root          4096 Jan  1 00:24 enable
   -r--------    1 root     root        132600 Jan  1 00:24 eye_data
   -r--------    1 root     root          4096 Jan  1 00:24 info
   -r--r--r--    1 root     root          4096 Jan  1 00:24 modalias
   --w-------    1 root     root          4096 Jan  1 00:24 prescale
   -rw-------    1 root     root          4096 Jan  1 00:24 reg_access
   lrwxrwxrwx    1 root     root             0 Jan  1 00:24 subsystem -> ../../../bus/platform
   -rw-r--r--    1 root     root          4096 Jan  1 00:24 uevent

   # cat info
   x65,y255 CDRDW: 40
