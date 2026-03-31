.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/jesd204/axi_adxcvr

.. _axi_adxcvr:

AXI ADXCVR
==========

ADI JESD204B/C AXI_ADXCVR Highspeed Transceivers Linux Driver.

Supported Devices
-----------------

- :external+hdl:ref:`axi_adxcvr`

Description
-----------

The JESD204B/C AXI_ADXCVR Highspeed Transceivers peripheral driver is a simple
driver that supports the :external+hdl:ref:`axi_adxcvr`. The driver reads some
configuration from the `devicetree <https://en.wikipedia.org/wiki/devicetree>`__
and configures the peripheral accordingly. After configuration has completed the
JESD204 link is enabled, this driver also supports PHY layer
`PRBS <https://en.wikipedia.org/wiki/PRBS>`__ generation and checking, or
:dokuwiki:`2D statistical EyeScan </resources/tools-software/linux-software/jesd_eye_scan>`.

This driver also work in conjunction with the :ref:`jesd204-fsm-framework`, in
this case JESD204B/C link configuration is provided by the JESD204-FSM
framework.

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
   * - :git-linux:`drivers/iio/jesd204/axi_adxcvr.c`
     - No
     -

Files Xilinx
~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git-linux:`drivers/iio/jesd204/axi_adxcvr.c`
     -
   * - driver
     - :git-linux:`drivers/iio/jesd204/axi_adxcvr_eyescan.c`
     -
   * - driver
     - :git-linux:`drivers/iio/jesd204/xilinx_transceiver.c`
     -
   * - incude
     - :git-linux:`drivers/iio/jesd204/axi_adxcvr.h`
     -
   * - incude
     - :git-linux:`drivers/iio/jesd204/axi_adxcvr_eyescan.h`
     -
   * - incude
     - :git-linux:`drivers/iio/jesd204/xilinx_transceiver.h`
     -
   * - dt-bindings
     - :git-linux:`include/dt-bindings/jesd204/adxcvr.h`
     -

Files Intel / Altera
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git-linux:`drivers/iio/jesd204/altera_adxcvr.c`
     -
   * - driver
     - :git-linux:`drivers/iio/jesd204/altera_a10_atx_pll.c`
     -
   * - driver
     - :git-linux:`drivers/iio/jesd204/altera_a10_cdr_pll.c`
     -

Example platform device initialization
--------------------------------------

The AXI JESD204B driver is a platform driver and can currently only be
instantiated via device tree.

Deprecated Non-jesd204-fsm mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Required devicetree properties:

- **compatible**: Must always be ``adi,axi-adxcvr-1.00``
- **reg**: Base address and register area size. This parameter expects a
  register range
- **clock-names**: List of input clock names - ``s_axi_aclk``, ``conv``,
  ``div40``
- **clocks**: Clock phandles and specifiers (See clock bindings for details on
  clock-names and clocks)
- **clock-output-names**: Generated clocks
- **adi,sys-clk-select**: 2 bit variable. For ultrascale, it selects the PLL
  reference clock source to be forwarded to the OUTCLK MUX: 0-CPLL, 3-QPLL0.
  Check RX/TXSYSCLKSEL parameter in the transceiver documentation for the FPGA
  you’re using. See section \"Software Guidelines\" at
  :external+hdl:ref:`axi_adxcvr`
- **adi,out-clk-select**: 3 bit variable. Controls the OUTCLKSEL multiplexer,
  controlling what will be forwarded to OUTCLK pin. Check RX/TXOUTCLKSEL
  parameter in the transceiver documentation for the FPGA you’re using See
  section \"Software Guidelines\" at :external+hdl:ref:`axi_adxcvr`

Optional devicetree properties:

- **adi,use-lpm-enable**: If set, the transceiver will be used in LPM mode.
  Otherwise, will be used in DFE mode. See transceiver documentation for details
- **adi,use-cpll-enable**: If set, the CPLL will be used for these transceivers

Optional devicetree clock-names:

- **conv2**: Optional SERDES (CPLL/QPLL) REFCLK from a difference source which
  rate and state must be in sync with the main conv clk.

Example:

.. code:: dts

   #include <dt-bindings/jesd204/adxcvr.h>

           axi_adrv9009_adxcvr_rx: axi-adxcvr-rx@84a40000 {
               #address-cells = <1>;
               #size-cells = <0>;
               compatible = "adi,axi-adxcvr-1.0";
               reg = <0x0 0x84a40000 0x1000>;

               clocks = <&hmc7044 5>, <&hmc7044 7>;
               clock-names = "conv", "div40";

               #clock-cells = <1>;
               clock-output-names = "rx_gt_clk", "rx_out_clk";

               adi,sys-clk-select = <XCVR_CPLL>;
               adi,out-clk-select = <XCVR_REFCLK>;
               adi,use-lpm-enable;
           };

           axi_adrv9009_adxcvr_tx: axi-adxcvr-tx@84a20000 {
               #address-cells = <1>;
               #size-cells = <0>;
               compatible = "adi,axi-adxcvr-1.0";
               reg = <0x0 0x84a20000 0x1000>;

               clocks = <&hmc7044 4>, <&hmc7044 6>;
               clock-names = "conv", "div40";

               #clock-cells = <1>;
               clock-output-names = "tx_gt_clk", "tx_out_clk";

               adi,sys-clk-select = <XCVR_QPLL>;
               adi,out-clk-select = <XCVR_REFCLK>;
           };

jesd204-fsm mode
~~~~~~~~~~~~~~~~

When using the :ref:`jesd204-fsm-framework` the div40 or Link/Device clock is
handled by the Link Layer Peripheral Linux device driver.

Required devicetree properties:

- **compatible**: Must always be ``adi,axi-adxcvr-1.00``
- **reg**: Base address and register area size. This parameter expects a
  register range
- **clock-names**: List of input clock names - ``s_axi_aclk``, ``conv``
- **clocks**: Clock phandles and specifiers (See clock bindings for details on
  clock-names and clocks)
- **clock-output-names**: Generated clocks
- **adi,sys-clk-select**: 2 bit variable. For ultrascale, it selects the PLL
  reference clock source to be forwarded to the OUTCLK MUX: 0-CPLL, 3-QPLL0.
  Check RX/TXSYSCLKSEL parameter in the transceiver documentation for the FPGA
  you’re using. See section \"Software Guidelines\" at
  :external+hdl:ref:`axi_adxcvr`
- **adi,out-clk-select**: 3 bit variable. Controls the OUTCLKSEL multiplexer,
  controlling what will be forwarded to OUTCLK pin. Check RX/TXOUTCLKSEL
  parameter in the transceiver documentation for the FPGA you’re using. See
  section \"Software Guidelines\" at :external+hdl:ref:`axi_adxcvr`
- **jesd204-device**: Adds device to the jesd204-fsm kernel framework
- **jesd204-inputs**: jesd204-fsm devices phandles and specifiers (used to build
  the link topology)

Optional devicetree properties:

- **adi,use-lpm-enable**: If set, the transceiver will be used in LPM mode.
  Otherwise, will be used in DFE mode. See transceiver documentation for details

Example:

.. code:: dts

   #include <dt-bindings/jesd204/adxcvr.h>

           axi_adrv9009_adxcvr_rx: axi-adxcvr-rx@84a40000 {
               #address-cells = <1>;
               #size-cells = <0>;
               compatible = "adi,axi-adxcvr-1.0";
               reg = <0x0 0x84a40000 0x1000>;

               clocks = <&hmc7044 5>;
               clock-names = "conv";

               #clock-cells = <1>;
               clock-output-names = "rx_gt_clk", "rx_out_clk";

               adi,sys-clk-select = <XCVR_CPLL>;
               adi,out-clk-select = <XCVR_REFCLK>;
               adi,use-lpm-enable;

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs =  <&hmc7044 0 FRAMER_LINK_RX>;
           };

           axi_adrv9009_adxcvr_tx: axi-adxcvr-tx@84a20000 {
               #address-cells = <1>;
               #size-cells = <0>;
               compatible = "adi,axi-adxcvr-1.0";
               reg = <0x0 0x84a20000 0x1000>;

               clocks = <&hmc7044 4>;
               clock-names = "conv";

               #clock-cells = <1>;
               clock-output-names = "tx_gt_clk", "tx_out_clk";

               adi,sys-clk-select = <XCVR_QPLL>;
               adi,out-clk-select = <XCVR_REFCLK>;

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs =  <&hmc7044 0 DEFRAMER_LINK_TX>;
           };

Enabling Linux driver support
-----------------------------

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

               <*>   JESD204 High-Speed Serial Interface Support  --->
           [--snip--]
           <*>   Analog Devices AXI ADXCVR PHY Support
           [--snip--]

Sysfs Interface
---------------

This driver provides advanced diagnostics, status and control via
`sysfs <https://en.wikipedia.org/wiki/sysfs>`__. Depending on the direction (RX
vs. TX) the provided sysfs attributes vary. A brief summary is with examples are
provided below.

Low level register access via reg_access
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This device driver features an optional debug facility, allowing users to read
or write registers directly. Special care needs to be taken when using this
feature, since you can modify registers on the back of the driver.

The DEBUG register access sysfs attribute ``reg_access`` allows access to the
the AXI HDL Core register space as well as to the Dynamic Reconfiguration Port
(DRP) interfaces. Accessing this sysfs attribute requires root privileges.

AXI register access
^^^^^^^^^^^^^^^^^^^

- See section \"Register map\" at :external+hdl:ref:`axi_adxcvr`

**Format:**

::

   axi <REGISTER ADDRESS> [<WRITE VAL>]

Example reading AXI register at offset 0 (REG_VERSION) ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo axi 0x0 >
   reg_access
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat reg_access
   0x110261

Example writing AXI register at offset 8 (REG_SCRATCH) ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo axi 0x8 0x1234 > reg_access
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat reg_access
   0x1234

DRP register access
^^^^^^^^^^^^^^^^^^^

**Please see Transceiver User Guides for further details**

- :xilinx:`7 Series FPGAs GTX/GTH Transceivers User Guide - Xilinx <support/documentation/user_guides/ug476_7Series_Transceivers.pdf>`
- :xilinx:`Ultrascale Architecture GTH Transceivers User Guide - Xilinx <support/documentation/user_guides/ug576-ultrascale-gth-transceivers.pdf>`

**Format:**

::

   drp <DRP PORT> <REGISTER ADDRESS> [<WRITE VAL>]

.. list-table::
   :header-rows: 1

   * - DRP Port
     - Comment
   * - 0
     - Common register space
   * - 0x100 + Lane
     - Channel register space

**Example reading OUT_DIV:** ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo drp 0x101 0x88 > reg_access
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat reg_access
   0x2AA

axi-adxcvr-rx
~~~~~~~~~~~~~

::

   root@analog:~# cd /sys/bus/platform/devices/84a60000.axi-adxcvr-rx
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# ls -al
   total 0
   drwxr-xr-x  4 root root       0 Feb  6 13:41 .
   drwxr-xr-x 12 root root       0 Feb  6 13:41 ..
   lrwxrwxrwx  1 root root       0 Feb  6 13:41 driver -> ../../../../bus/platform/drivers/axi_adxcvr
   -rw-r--r--  1 root root    4096 Feb  6 13:42 driver_override
   -rw-r--r--  1 root root    4096 Feb  6 13:42 enable
   -r--r--r--  1 root root 2091000 Feb  6 13:42 eye_data
   -r--r--r--  1 root root    4096 Feb  6 13:42 eyescan_info
   drwxr-xr-x  3 root root       0 Feb  6 13:41 jesd204:6
   -r--r--r--  1 root root    4096 Feb  6 13:42 modalias
   lrwxrwxrwx  1 root root       0 Feb  6 13:42 of_node -> ../../../../firmware/devicetree/base/fpga-axi@0/axi-adxcvr-rx@84a60000
   drwxr-xr-x  2 root root       0 Feb  6 13:42 power
   --w-------  1 root root    4096 Feb  6 13:42 prbs_counter_reset
   -rw-r--r--  1 root root    4096 Feb  6 13:42 prbs_error_counters
   -rw-r--r--  1 root root    4096 Feb  6 13:42 prbs_select
   -r--r--r--  1 root root    4096 Feb  6 13:42 prbs_status
   -rw-r--r--  1 root root    4096 Feb  6 13:42 prescale
   -rw-r--r--  1 root root    4096 Feb  6 13:42 reg_access
   lrwxrwxrwx  1 root root       0 Feb  6 13:41 subsystem -> ../../../../bus/platform
   -rw-r--r--  1 root root    4096 Feb  6 13:41 uevent

.. _axi_adxcvr prbs-select:

PRBS Select
^^^^^^^^^^^

Writing ``prbs_select`` selects the
`PRBS <https://en.wikipedia.org/wiki/PRBS>`__ type. (accepted values depend on
the GT architecture). Reading returns the selected type.

.. list-table::
   :header-rows: 1

   * - Value
     - Comment
   * - 0
     - PRBS_DISABLE
   * - 7
     - PRBS7
   * - 9
     - PRBS9
   * - 15
     - PRBS15
   * - 23
     - PRBS23
   * - 31
     - PRBS31

**Example:** ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo 23 > prbs_select
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat prbs_select
   23
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx#

prbs_status
^^^^^^^^^^^

Reading returns a status string. Values are:

.. list-table::
   :header-rows: 1

   * - Value
     - Comment
   * - unlocked
     - The PRBS is unlocked
   * - valid
     - he PRBS is locked and no errors are detected
   * - error
     - The PRBS has locked but errors were detected

.. tip::

   These conditions are sticky and can be cleared by writing 1 to
   ``prbs_counter_reset``

**Example:** ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat prbs_status
   valid

prbs_error_counters
^^^^^^^^^^^^^^^^^^^

Reading returns a string with error counts for each physical JESD204 lane.

**Example:** ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat prbs_error_counters
   0 0 0 0 0 0 0 0

prbs_counter_reset
^^^^^^^^^^^^^^^^^^

Writing 1 will clear the ``prbs_status`` and reset the ``prbs_error_counters``

**Example:** ::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo 1 > prbs_counter_reset

JESD204 Eye Scan
^^^^^^^^^^^^^^^^

Please see here:
:dokuwiki:`JESD204 Eye Scan </resources/tools-software/linux-software/jesd_eye_scan>`

Following commands visualize how to control the eyescan from the command line.
``eyescan_info`` provides the necessary information to reconstruct the eye from
the binary bitmap retrieved from ``eye_data``. Such as the resolution format,
lane rate, number of lanes, CDR data width, LMP mode, etc. Using the
``prescale`` attribute the iteration time at each offset can be controlled, so
increasing ``prescale`` will also increase the time a eye capture needs to
complete. Writing ``enable`` with the physical lane number, will start the
capture processes. While the capture is in progress reading ``eye_data`` will
block. If this is not desired the user can poll enable, which will return
**-EBUSY** while a capture is in progress. Once the capture finished the lane
number of the last captured lane is returned.

**Reading the eyescan_info:**

::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat eyescan_info
   x65,y255 CDRDW: 40 LPM: 1 NL: 8 LR: 15000000

**Configure the prescale:**

::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo 1 > prescale
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat prescale
   1

**Enable the eye capture:**

::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# echo 1 > enable
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat enable
   cat: enable: Device or resource busy

**Retrieve the binary bitmap: (needs post processing)**

::

   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# cat enable
   1
   root@analog:/sys/bus/platform/devices/84a60000.axi-adxcvr-rx# hexdump eye_data
   0000000 ffff 00a7 ffff 00a7 ffff 00a8 ffff 00a7
   0000010 ffff 00a7 ffff 00a9 ffff 00aa ffff 00ac
   0000020 ffff 00b1 ffff 00bb ffff 00c7 ffff 00d6
   0000030 ffff 00ed ffff 010b ffff 012c ffff 015e
   0000040 ffff 019a ffff 01f6 ffff 0287 ffff 0373
   0000050 ffff 0544 ffff 08ce ffff 11ce ffff 27d6
   0000060 ffff 75b9 a10c ffff 23c3 ffff 05b4 ffff

   //[-- snip --]//

axi-adxcvr-tx
~~~~~~~~~~~~~

::

   root@analog:~# cd /sys/bus/platform/devices/84b60000.axi-adxcvr-tx
   root@analog:/sys/bus/platform/devices/84b60000.axi-adxcvr-tx# ls -l
   total 0
   lrwxrwxrwx 1 root root    0 Feb  6 13:41 driver -> ../../../../bus/platform/drivers/axi_adxcvr
   -rw-r--r-- 1 root root 4096 Feb  6 14:32 driver_override
   drwxr-xr-x 3 root root    0 Feb  6 13:41 jesd204:7
   -r--r--r-- 1 root root 4096 Feb  6 14:32 modalias
   lrwxrwxrwx 1 root root    0 Feb  6 14:32 of_node -> ../../../../firmware/devicetree/base/fpga-axi@0/axi-adxcvr-tx@84b60000
   drwxr-xr-x 2 root root    0 Feb  6 14:32 power
   --w------- 1 root root 4096 Feb  6 14:32 prbs_error_inject
   -rw-r--r-- 1 root root 4096 Feb  6 14:32 prbs_select
   -rw-r--r-- 1 root root 4096 Feb  6 14:32 reg_access
   lrwxrwxrwx 1 root root    0 Feb  6 13:41 subsystem -> ../../../../bus/platform
   -rw-r--r-- 1 root root 4096 Feb  6 13:41 uevent

prbs_select
^^^^^^^^^^^

Writing ``prbs_select`` selects the
`PRBS <https://en.wikipedia.org/wiki/PRBS>`__ type. (accepted values depend on
the GT architecture). Reading returns the selected type.

.. list-table::
   :header-rows: 1

   * - Value
     - Comment
   * - 0
     - PRBS_DISABLE
   * - 7
     - PRBS7
   * - 9
     - PRBS9
   * - 15
     - PRBS15
   * - 23
     - PRBS23
   * - 31
     - PRBS31

**Example:** ::

   root@analog:/sys/bus/platform/devices/84b60000.axi-adxcvr-tx# echo 15 > prbs_select
   root@analog:/sys/bus/platform/devices/84b60000.axi-adxcvr-tx# cat prbs_select
   15
   root@analog:/sys/bus/platform/devices/84b60000.axi-adxcvr-tx#

prbs_error_inject
^^^^^^^^^^^^^^^^^

Writing 1 injects a series of errors´(used for testing)

**Example:** ::

   root@analog:/sys/bus/platform/devices/84b60000.axi-adxcvr-tx# echo 1 > prbs_error_inject
   root@analog:/sys/bus/platform/devices/84b60000.axi-adxcvr-tx#

More Information
~~~~~~~~~~~~~~~~

- :external+hdl:ref:`jesd204`
- :ref:`jesd204-fsm-framework`
- :dokuwiki:`JESD204B Status Utility </resources/tools-software/linux-software/jesd_status>`
