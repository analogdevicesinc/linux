.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/jesd204/axi_jesd204_rx

.. _axi_jesd204_rx:

AXI JESD204B RX
===============

ADI JESD204B/C Receive Peripheral Linux Driver.

Supported Devices
-----------------

- :external+hdl:ref:`axi_jesd204_rx`

Description
-----------

The AXI JESD204B RX peripheral driver is a simple driver that supports the
:external+hdl:ref:`axi_jesd204_rx`. The driver reads JESD204B link configuration
data from the devicetree and configures the peripheral accordingly. After
configuration has completed the JESD204B link is enabled. Link state can be
monitored through sysfs files.

This driver also work in conjunction with the :ref:`jesd204-fsm-framework`, in
this case JESD204B link configuration is provided by the JESD204-FSM framework.

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
   * - :git+linux:`main:drivers/iio/jesd204/axi_jesd204_rx.c`
     - No
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git+linux:`main:drivers/iio/jesd204/axi_jesd204_rx.c`
     -

Example platform device / device tree initialization
----------------------------------------------------

The AXI JESD204B driver is a platform driver and can currently only be
instantiated via device tree.

Deprecated Non-jesd204-fsm mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Required devicetree properties:

- **compatible**: Must always be ``adi,axi-jesd204-rx-1.00.a``
- **reg**: Base address and register area size. This parameter expects a
  register range.
- **interrupts**: Property with a value describing the interrupt number.
- **clock-names**: List of input clock names - ``s_axi_aclk``, ``device_clk``
- **clocks**: Clock phandles and specifiers (See clock bindings for details on
  clock-names and clocks).
- **adi,frames-per-multiframe**: Number of frames per multi-frame (``K``)
- **adi,octets-per-frame**: Number of octets per frame (``F``)

Optional devicetree properties:

- **adi,high-density**: If specified the JESD204B link is configured for high
  density (``HD``) operation.
- **adi,subclass**: The JESD204B subclass.

Example:

::

   jesd204b-rx@77b00000 {
       compatible = "adi,axi-jesd204-rx-1.00.a";
       reg = <0x77b00000 0x10000>;
       interrupts = <0 56 4>;

       clock-names = "s_axi_aclk", "device_clk";
       clocks = <&clkc 14>, <&ad9528 13>;

       adi,octets-per-frame = <32>;
       adi,frames-per-multiframe = <4>;
   };

jesd204-fsm mode
~~~~~~~~~~~~~~~~

When using the :ref:`jesd204-fsm-framework` the JESD link parameters are
broadcasted by the framework to all link components.

Required devicetree properties:

- **compatible**: Must always be ``adi,axi-jesd204-rx-1.00.a``
- **reg**: Base address and register area size. This parameter expects a
  register range.
- **interrupts**: Property with a value describing the interrupt number.
- **clock-names**: List of input clock names - ``s_axi_aclk``, ``device_clk``,
  ``lane_clk``
- **clocks**: Clock phandles and specifiers (See clock bindings for details on
  clock-names and clocks).
- **jesd204-device**: Adds device to the jesd204-fsm kernel framework
- **jesd204-inputs**: jesd204-fsm devices phandles and specifiers (used to build
  the link topology)

Optional devicetree clock-names:

- **link_clk**: When using the link layer gearbox in order to support NP=12
  modes, ``device_clk`` and ``link_clock`` need to be different. This is typical
  the case in case ``SYNTH_DATA_PATH_WIDTH`` and ``TPL_DATA_PATH_WIDTH``
  differs. Then ``device_clk`` becomes ``link_clk`` times
  ``SYNTH_DATA_PATH_WIDTH`` / ``TPL_DATA_PATH_WIDTH``. ``link_clk`` still needs
  to be ``lane_clk`` / 40 for **JESD204B/8B10B** encoding and ``lane_clk`` / 66
  for **JESD204C/64B66B** encoding.
- **conv2**: Optional SERDES (CPLL/QPLL) REFCLK from a difference source which
  rate and state must be in sync with the main conv clk.

Example:

::

   axi_ad9081_rx_jesd: axi-jesd204-rx@84a90000 {
       compatible = "adi,axi-jesd204-rx-1.0";
       reg = <0x84a90000 0x1000>;

       interrupts = <0 107 IRQ_TYPE_LEVEL_HIGH>;

       clocks = <&zynqmp_clk 71>, <&hmc7044 10>, <&axi_ad9081_adxcvr_rx 0>;
       clock-names = "s_axi_aclk", "device_clk", "lane_clk";

       jesd204-device;
       #jesd204-cells = <2>;
       jesd204-inputs = <&axi_ad9081_adxcvr_rx 0 FRAMER_LINK0_RX>;
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
           <*>   Analog Devices AXI JESD204B RX Support
               [--snip--]

Sysfs Interface
---------------

This driver provides advanced diagnostics, status and via
`sysfs <https://en.wikipedia.org/wiki/sysfs>`__. A brief summary is with
examples are provided below.

::

   root@analog:/sys/bus/platform/devices/84a90000.axi-jesd204-rx# ls -al
   total 0
   drwxr-xr-x  4 root root    0 Feb  6 13:41 .
   drwxr-xr-x 12 root root    0 Feb  6 13:41 ..
   lrwxrwxrwx  1 root root    0 Feb  6 13:41 driver -> ../../../../bus/platform/drivers/axi-jesd204-rx
   -rw-r--r--  1 root root 4096 Feb  6 16:27 driver_override
   -r--r--r--  1 root root 4096 Feb  6 16:27 encoder
   drwxr-xr-x  3 root root    0 Feb  6 13:41 jesd204:4
   -r--------  1 root root 4096 Feb  6 16:27 lane0_info
   -r--------  1 root root 4096 Feb  6 16:27 lane1_info
   -r--------  1 root root 4096 Feb  6 16:27 lane2_info
   -r--------  1 root root 4096 Feb  6 16:27 lane3_info
   -r--------  1 root root 4096 Feb  6 16:27 lane4_info
   -r--------  1 root root 4096 Feb  6 16:27 lane5_info
   -r--------  1 root root 4096 Feb  6 16:27 lane6_info
   -r--------  1 root root 4096 Feb  6 16:27 lane7_info
   -r--r--r--  1 root root 4096 Feb  6 16:27 modalias
   lrwxrwxrwx  1 root root    0 Feb  6 16:27 of_node -> ../../../../firmware/devicetree/base/fpga-axi@0/axi-jesd204-rx@84a90000
   drwxr-xr-x  2 root root    0 Feb  6 16:27 power
   -r--r--r--  1 root root 4096 Feb  6 16:27 status
   lrwxrwxrwx  1 root root    0 Feb  6 13:41 subsystem -> ../../../../bus/platform
   -rw-r--r--  1 root root 4096 Feb  6 13:41 uevent

**Reading the device/link status:**

.. list-table::
   :header-rows: 1

   * - What
     - Comment
     -
     -
   * - Link is [enabled|disabled]
     - Link state indicator. In case link state is disabled, that either means the link was never enabled or that an error occurred and the FSM rolled back and disabled the link. In the jesd204-fsm case this can be prevented by using the ``jesd204-ignore-errors;`` devicetree property when placed in the jesd204-fsm jesd204-top-device node.
     -
     -
   * - Link status
     - Depending on the encoding JESD204B/C 8B10B/64B66B there can be different values ``RESET``, ``WAIT``, ``CGS``, ``ILAS``, etc., ``DATA`` Please see link layer documentation for the state machine: :external+hdl:ref:`axi_jesd204_rx` \\ :external+hdl:ref:`axi_jesd204_tx` In general ``DATA`` is the desired state indicating proper operation.
     -
     -
   * - SYNC is [asserted|deasserted] (8B10B only)
     - State of the external ``SYNC`` signal.
     -
     -
   * - SYSREF captured: [Yes|No] (Subclass 1 only)
     - Yes indicates that a ``SYSREF`` pulse was captured.
     -
     -
   * - SYSREF alignment error: [Yes|No] (Subclass 1 only)
     - Yes indicates that a ``SYSREF`` event has been observed which was unaligned, in regards to the ``LMFC/LEMC`` period, to a previously recorded ``SYSREF`` event.
     -
     -
   * - Lane rate
     - The ``SERDES`` lane rate / bit clock.
     -
     -
   * - [LMFC|LEMC] rate
     - Frequency of the internal local multiframe clocks (``LMFC``)/ local-multiblock-clock (``LEMC``).
     -
     -
   * - Lane rate / [40|66]
     - Is equal to the desired ``Link Clock`` frequency.
     -
     -
   * - Reported Link Clock
     - Is the ``Link Clock`` frequency which is reported by the common clock framework. If this rate is different from the desired ``Link Clock`` frequency, there is likely a problem. For example the clock provider wasn’t able to set the desired value.
     -
     -
   * - Measured Link Clock
     - Is the measured ``Link Clock``, using a frequency counter inside the Link Layer Peripheral. For proper operation desired, reported and measured frequency must match.
     -
     -
   * - Desired Device Clock
     - In case this value is different from the Desired ``Link Clock``, this indicates Dual Clock Operation required by the Gearbox. Please see Gearbox/Dual Clock Operation in the Link Layer Peripheral documentation: :external+hdl:ref:`axi_jesd204_rx` \\ :external+hdl:ref:`axi_jesd204_tx` The ``Device Clock`` is typically generated by the clock provider which also provides ``SYSREF``.
     -
     -
   * - Reported Device Clock
     - Is the ``Device Clock`` frequency which is reported by the common clock framework. If this rate is different from the desired ``Device Clock`` frequency, there is likely a problem. For example the clock provider wasn’t able to set the desired value.
     -
     -
   * - Measured Device Clock
     - Is the measured ``Device Clock``, using a frequency counter inside the Link Layer Peripheral. For proper operation desired, reported and measured frequency must match.
     -
     -

::

   root@analog:/sys/bus/platform/devices/84a90000.axi-jesd204-rx# cat status
   Link is enabled
   Measured Link Clock: 375.034 MHz
   Reported Link Clock: 375.000 MHz
   Measured Device Clock: 375.034 MHz
   Reported Device Clock: 375.000 MHz
   Desired Device Clock: 375.000 MHz
   Lane rate: 15000.000 MHz
   Lane rate / 40: 375.000 MHz
   LMFC rate: 46.875 MHz
   Link status: DATA
   SYSREF captured: Yes
   SYSREF alignment error: No

**Reading the Encoder used:**

::

   root@analog:/sys/bus/platform/devices/84a90000.axi-jesd204-rx# cat encoder
   8b10b

**Reading the Lane Status:**

::

   root@analog:/sys/bus/platform/devices/84a90000.axi-jesd204-rx# cat lane0_info
   Errors: 0
   CGS state: DATA
   Initial Frame Synchronization: Yes
   Lane Latency: 3 Multi-frames and 13 Octets
   Initial Lane Alignment Sequence: Yes
   DID: 0, BID: 0, LID: 2, L: 8, SCR: 1, F: 1
   K: 32, M: 4, N: 16, CS: 0, N': 16, S: 1, HD: 0
   FCHK: 0x9, CF: 0
   ADJCNT: 0, PHADJ: 0, ADJDIR: 0, JESDV: 1, SUBCLASS: 1
   FC: 15000000

More Information
~~~~~~~~~~~~~~~~

- :external+hdl:ref:`jesd204`
