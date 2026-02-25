.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-dds/axi-dac-dds-hdl

.. _axi-dac-dds-hdl:

AXI DAC HDL
===========

AXI DAC HDL Linux Driver.

Supported Devices
-----------------

- :adi:`AD9122`
- :adi:`AD9136`
- :adi:`AD9144`
- :adi:`AD9152`
- :adi:`AD9154`
- :adi:`AD9162`
- :adi:`AD9171`
- :adi:`AD9172`
- :adi:`AD9173`
- :adi:`AD9174`
- :adi:`AD9175`
- :adi:`AD9176`
- :adi:`AD9361`
- :adi:`AD9364`
- :adi:`AD9371`
- :adi:`AD9739A`
- :adi:`AD9783`

Supported Boards
----------------

| This driver supports the

- :dokuwiki:`AD-FMCOMMS1-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms1-ebz>`
- :dokuwiki:`AD-FMCOMMS2-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms2-ebz>`
- :dokuwiki:`AD-FMCOMMS3-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms3-ebz>`
- :dokuwiki:`AD-FMCOMMS4-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms4-ebz>`
- :dokuwiki:`AD-FMCOMMS5-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms5-ebz>`
- :dokuwiki:`AD-FMCOMMS11-EBZ User Guide </resources/eval/user-guides/ad-fmcomms11-ebz>`
- :dokuwiki:`AD-FMCDAQ2-EBZ FMC Card <resources/eval/user-guides/ad-fmcdaq2-ebz>`
- :dokuwiki:`AD-FMCDAQ3-EBZ User Guide </resources/eval/user-guides/ad-fmcdaq3-ebz>`
- :external+hdl:ref:`ad9739a_fmc`
- :external+hdl:ref:`ad9739a_fmc`
- :dokuwiki:`EVALUATING THE AD9780/AD9781/AD9783 DIGITAL-TO-ANALOG CONVERTERS </resources/eval/dpg/eval-ad9783>`
- :external+hdl:ref:`ad9783_ebz`
- :dokuwiki:`ADRV9009 &amp; ADRV9008 Prototyping Platform User Guide </resources/eval/user-guides/adrv9009>`
- :dokuwiki:`AD9171/AD9172/AD9173/AD9174/AD9175/AD9176 Evaluation Board </resources/eval/dpg/eval-ad9172>`
- :dokuwiki+deprecated:`ADRV9371 FMC Card <resources/eval/user-guides/mykonos>`

Supported HDL Cores
-------------------

- :external+hdl:ref:`axi_dac`
- :external+hdl:ref:`ad_ip_jesd204_tpl_dac`
- :external+hdl:ref:`axi_ad9361`
- :external+hdl:ref:`axi_ad9783`
- :dokuwiki+deprecated:`/resources/fpga/docs/axi_ad9371`
- :dokuwiki+deprecated:`/resources/fpga/docs/axi_ad9144`

Sub device Documentation (linked mode)
--------------------------------------

- :ref:`ad9172`

Description
-----------

The AXI DAC DDS HDL driver is the driver for various HDL interface cores which
are used on different FPGA designs. The driver is implemented as an Linux IIO
driver. It’s register map can be found here:
:external+hdl:ref:`generic-dac-register-access`

This driver is independent from the physical layer. So it’s being used with CMOS
or LVDS type interfaces or the :external+hdl:ref:`jesd204`.

There are basically two use case scenarios for this driver, in which this driver
controls only the HDL/FPGA transport layer capture core. This mode is called
``standalone mode``, and the converter is fully configured and controlled by a
separate driver. The Linux common clock framework is utilized so that this
driver knows the sampling frequency of the connected converter DAC device.
Alternatively this driver can also be used in a ``linked mode``, where the
converter device typically a SPI device must instantiate first. If this has
happened this AXI-DAC driver will then probe as well. (Deferred probe mechanism)
Finally both the HDL core platform device together with the converter SPI device
will register a common IIO device, which will then exhibit a common set of
attributes and channels. The converter SPI device driver is handled in a
separate source file, which can be found in the same directory this driver
exists. The device tree phandle ``spibus-connected`` is used to connect the
AXI-DAC driver with the SPI control driver.

Sometimes there is a common HDL/FPGA transport layer core, which handles both
RX/TX or ADC/DMA. This single physical core is then handled by two independent
IIO drivers each for one transport data direction. It’s physical address
register space is then also split or divided, typically spaced by 0x4000. A good
example for this case is the :external+hdl:ref:`axi_ad9361` HDL core.

The HDL/FPGA transport layer capture core driver portion implements a polyphase
dual tone DDS core per channel together with an DMA based waveform buffer
mechanism. The buffer can be filled by arbitrary data, which is then typically
cyclically repeated or used in a streaming fashion.

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/iio-dds/axi-dac-dds-overview.png
   :width: 800px

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
   * - :git-linux:`drivers/iio/frequency/cf_axi_dds.c`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/frequency/cf_axi_dds.c>`__
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
     - :git-linux:`drivers/iio/frequency/ad9122.c`
     -
   * - driver
     - :git-linux:`drivers/iio/frequency/ad9144.c`
     -
   * - driver
     - :git-linux:`drivers/iio/frequency/ad9162.c`
     -
   * - driver
     - :git-linux:`drivers/iio/frequency/ad9172.c`
     -
   * - driver
     - :git-linux:`drivers/iio/frequency/cf_axi_dds.c`
     -
   * - driver
     - :git-linux:`drivers/iio/frequency/cf_axi_dds_buffer_stream.c`
     -
   * - include
     - :git-linux:`drivers/iio/frequency/cf_axi_adc.h <drivers/iio/frequency/cf_axi_dds.h>`
     -

Example platform device initialization
--------------------------------------

The AXI DAC/DDS driver is a platform driver and can only be instantiated via
device tree.

Required devicetree properties:

- **compatible**: Should always be one of these:

  - adi,axi-ad9122-6.00.a
  - adi,axi-ad9136-1.0
  - adi,axi-ad9144-1.0
  - adi,axi-ad9162-1.0
  - adi,axi-ad9172-1.0
  - adi,axi-ad9361x2-dds-6.00.a
  - adi,axi-ad9361-dds-6.00.a
  - adi,axi-ad9364-dds-6.00.a
  - adi,axi-ad9371-tx-1.0
  - adi,axi-ad9739a-8.00.b
  - adi,axi-ad9963-dds-1.00.a
  - adi,axi-adrv9009-tx-1.0

- **reg**: Base address and register area size. This parameter expects a
  register range.

Optional Parameters:

- **adi,axi-dds-default-scale**: The power up DDS scale in 16-bit fractional
  representation. On driver probe the default mode is DDS with some default
  frequency at 0.25 Full Scale. Often this behavior is undesired. The best way
  to mute the DDS on startup is to set this to 0.
- **adi,axi-dds-default-frequency**: The power up DDS frequency for all tones.
- **adi,axi-dds-parity-enable**: If set the HDL core uses Parity Mode. (Default
  is Frame Mode)
- **adi,axi-dds-parity-type-odd**: If set the HDL core use odd parity. (Default
  is even)
- **adi,axi-dds-1-rf-channel**: If set the HDL core expects 1 RF channel.
  (Default 2 channels)
- **adi,axi-interpolation-core-available**: If the FPGA system features an
  additional interpolation filter, this attribute should be set.
- **adi,axi-pl-fifo-enable**: If the FPGA system features a PL FIFO
  (Programmable Logic), this attribute should be set.
- **dmas**: DMA specifiers for the tx dma. See the DMA client binding:
  Documentation/devicetree/bindings/dma/dma.txt
- **dma-names**: DMA request name. Should be ``tx`` if a dma is present.
- **spibus-connected**: Phandle to the SPI device (control interface) on which
  the DAC can be found

Example:

::

   &spi0 {
       status = "okay";

       adc0_ad9361: ad9361-phy@0 {
           #address-cells = <1>;
           #size-cells = <0>;
           #clock-cells = <1>;
           compatible = "ad9361";

           /* SPI Setup */
           reg = <0>;
           spi-cpha;
           spi-max-frequency = <10000000>;

           /* Clocks */
           clocks = <&ad9361_clkin 0>;
           clock-names = "ad9361_ext_refclk";
           clock-output-names = "rx_sampl_clk", "tx_sampl_clk";

       [--- snip ---]

       }
   }

   &fpga_axi {
       tx_dma: dma@7c420000 {
           compatible = "adi,axi-dmac-1.00.a";
           reg = <0x7c420000 0x10000>;
           #dma-cells = <1>;
           interrupts = <0 56 0>;
           clocks = <&clkc 16>;

           dma-channel {
               adi,source-bus-width = <64>;
               adi,destination-bus-width = <64>;
               adi,type = <1>;
               adi,cyclic;
           };
       };

       cf_ad9361_dac_core_0: cf-ad9361-dds-core-lpc@79024000 {
           compatible = "adi,axi-ad9361-dds-6.00.a";
           reg = <0x79024000 0x1000>;
           clocks = <&adc0_ad9361 13>;
           clock-names = "sampl_clk";
           dmas = <&tx_dma 0>;
           dma-names = "tx";
           adi,axi-dds-default-scale = <0>;
       };
   };

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The AXI ADC HDL driver may depend on **CONFIG_SPI**

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

                 *** Direct Digital Synthesis ***
           [--snip--]

           <*>   Analog Devices CoreFPGA AXI DDS driver
           <*>   Analog Devices AD9122 DAC

           [--snip--]

Hardware configuration
----------------------

In case the driver probes successfully and the device gets instantiated. Your
systems kernel messages should include a line, which may look like the one shown
below.

::

   cf_axi_dds 79024000.cf-ad9361-dds-core-lpc: Analog Devices CF_AXI_DDS_DDS MASTER (8.00.b) at 0x79024000 mapped to 0xf0998000, probed DDS AD9361

Driver testing
--------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-device-files
   :end-before: .. end-iio-device-files

Some device attributes control the DDS HDL Core, others features of the DAC and
associated clock providers.

::

   analog:/sys/bus/iio/devices/iio:device2# cd /sys/bus/iio/devices/
   root@analog:/sys/bus/iio/devices# ls
   iio:device0  iio:device1  iio:device2  iio:device3
   root@analog:/sys/bus/iio/devices#

   root@analog:/sys/bus/iio/devices# cd iio\:device2
   root@analog:/sys/bus/iio/devices/iio:device2# ls -l
   total 0
   drwxr-xr-x 5 root root    0 Jan  1 00:00 .
   drwxr-xr-x 4 root root    0 Jan  1 00:00 ..
   drwxrwxrwx 2 root root    0 Jan  1 00:00 buffer
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 dev
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 name
   lrwxrwxrwx 1 root root    0 Jan  1 00:00 of_node -> ../../../../../firmware/devicetree/base/fpga-axi@0/cf-ad9361-dds-core-lpc@79024000
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage0_TX1_I_F1_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage0_TX1_I_F1_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage0_TX1_I_F1_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage0_TX1_I_F1_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage1_TX1_I_F2_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage1_TX1_I_F2_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage1_TX1_I_F2_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage1_TX1_I_F2_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage2_TX1_Q_F1_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage2_TX1_Q_F1_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage2_TX1_Q_F1_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage2_TX1_Q_F1_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage3_TX1_Q_F2_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage3_TX1_Q_F2_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage3_TX1_Q_F2_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage3_TX1_Q_F2_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage4_TX2_I_F1_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage4_TX2_I_F1_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage4_TX2_I_F1_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage4_TX2_I_F1_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage5_TX2_I_F2_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage5_TX2_I_F2_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage5_TX2_I_F2_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage5_TX2_I_F2_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage6_TX2_Q_F1_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage6_TX2_Q_F1_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage6_TX2_Q_F1_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage6_TX2_Q_F1_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage7_TX2_Q_F2_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage7_TX2_Q_F2_phase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage7_TX2_Q_F2_raw
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage7_TX2_Q_F2_scale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_altvoltage_sampling_frequency
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage0_calibphase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage0_calibscale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage1_calibphase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage1_calibscale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage2_calibphase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage2_calibscale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage3_calibphase
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage3_calibscale
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 out_voltage_sampling_frequency
   drwxrwxrwx 2 root root    0 Jan  1 00:00 power
   drwxrwxrwx 2 root root    0 Jan  1 00:00 scan_elements
   lrwxrwxrwx 1 root root    0 Jan  1 00:00 subsystem -> ../../../../../bus/iio
   -rw-rw-rw- 1 root root 4096 Jan  1 00:00 uevent
   root@analog:/sys/bus/iio/devices/iio:device2#

Show device name
~~~~~~~~~~~~~~~~

::

   root@analog:/sys/bus/iio/devices/iio:device2# cat name
   cf-ad9361-dds-core-lpc

Show/Set DDS frequency
^^^^^^^^^^^^^^^^^^^^^^

Values are in Hz.

::

   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_frequency
   9279985
   root@analog:/sys/bus/iio/devices/iio:device2# echo 500000 > out_altvoltage0_TX1_I_F1_frequency
   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_frequency
   500163

Show/Set DDS Phase
''''''''''''''''''

Values are in milli degrees. ::

   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_phase
   89995
   root@analog:/sys/bus/iio/devices/iio:device2# echo 91000 > out_altvoltage0_TX1_I_F1_phase
   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_phase
   90995

Enable/Disable DDS Channel
''''''''''''''''''''''''''

1 for enable, 0 for disable.

With the introduction of the big channel MUX (REG_CHAN_CNTRL_7, DAC_DDS_SEL)
starting with HDV Version > 7.00, the RAW attribute has some side effects, that
need to be understood. In many situations for example when the Buffer mode is
used, it’s not desirable to write this attribute.

HDL Version < 7.00.a
''''''''''''''''''''

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/iio-dds/dds_data_sel_v7-.png
   :width: 500px

HDL Version > 7.00.a
''''''''''''''''''''

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/iio-dds/dds_data_sel_v7+.png
   :width: 500px

::

   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_raw
   1

Show/Set DDS Scale
''''''''''''''''''

DDS amplitude: range 0.00 … 1.00 (relative to full scale)

.. tip::

   When disabling the DAC Buffer/DMA mode and a transition into DDS tone output
   mode is not desired - Scale of all channels should be set to 0.00. See also
   device tree attribute: adi,axi-dds-default-scale

::

   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_scale
   0.500000
   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_scale
   1.000000 0.500000 0.250000 0.125000 ...
   root@analog:/sys/bus/iio/devices/iio:device2# echo 0.25 > out_altvoltage0_TX1_I_F1_scale
   root@analog:/sys/bus/iio/devices/iio:device2# cat out_altvoltage0_TX1_I_F1_scale
   0.250000

DAC internal or external Gain, DC Offset, and Phase adjustments
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Depending on the platform. These attributes may control converter internal or
external processing blocks.

The AD9122 features Gain, DC Offset, and Phase adjustment for sideband
suppression. These features can be controlled via following attributes:

- out_voltage0_calibbias
- out_voltage0_calibscale
- out_voltage0_phase
- out_voltage1_calibbias
- out_voltage1_calibscale
- out_voltage1_phase

::

   root@analog:/sys/bus/iio/devices/iio:device2# grep "" out_voltage
   out_voltage0_calibbias:0
   out_voltage0_calibscale:505
   out_voltage0_phase:0
   out_voltage1_calibbias:0
   out_voltage1_calibscale:505
   out_voltage1_phase:0

External synchronization
''''''''''''''''''''''''

The :external+hdl:ref:`ad_ip_jesd204_tpl_dac` core supports the EXT_SYNC
feature, allowing to synchronize multiple channels within a DAC or across
multiple instances, see ``External synchronization`` section of
:external+hdl:ref:`ad_ip_jesd204_tpl_dac`. This feature can also synchronize
between the :external+hdl:ref:`ad_ip_jesd204_tpl_adc` and
:external+hdl:ref:`ad_ip_jesd204_tpl_dac` core.

There are two device attributes which allows controlling this feature:
``sync_start_enable`` and ``sync_start_enable_available`` reading the later
returns the available modes which depend on HDL core synthesis parameters. The
options are explained below. Reading ‘sync_start_enable’ returns either ‘arm’
while waiting for the external synchronization signal or ‘disarm’ otherwise.

- ``arm``: Setting this key will arm the trigger mechanism sensitive to an
  external sync signal. Once the external sync signal goes high it synchronizes
  channels within a DAC, and across multiple instances. This key has an effect
  only the EXT_SYNC synthesis parameter is set.

- ``disarm``: Setting this key will disarm the trigger mechanism sensitive to an
  external sync signal. This key has an effect only the EXT_SYNC synthesis
  parameter is set.

- ``trigger_manual``: Setting this key will issue an external sync event if it
  is hooked up inside the fabric. This key has an effect only the EXT_SYNC
  synthesis parameter is set.

Example:
''''''''

::

   root@analog:/sys/bus/iio/devices/iio:device3# cat sync_start_enable_available
   arm disarm trigger_manual
   root@analog:/sys/bus/iio/devices/iio:device3# cat sync_start_enable
   disarm
   root@analog:/sys/bus/iio/devices/iio:device3# echo arm > sync_start_enable
   root@analog:/sys/bus/iio/devices/iio:device3# cat sync_start_enable
   arm
   root@analog:/sys/bus/iio/devices/iio:device3# echo trigger_manual > sync_start_enable
   root@analog:/sys/bus/iio/devices/iio:device3# cat sync_start_enable
   disarm

Buffer management
~~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device3> ls -l buffer
   total 0
   -rw-r--r--    1 root     root          4096 Jan  1 00:12 enable
   -rw-r--r--    1 root     root          4096 Jan  1 00:12 length
   -r--r--r--    1 root     root          4096 Jan  1 00:12 watermark

.. include:: /include/iio-snippets.rst
   :start-after: .. start-buffer-management
   :end-before: .. end-buffer-management

::

   root:/sys/bus/iio/devices/iio:device3> ls -l scan_elements
   total 0
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 out_voltage0_en
   -r--r--r--    1 root     root          4096 Jan  1 00:00 out_voltage0_index
   -r--r--r--    1 root     root          4096 Jan  1 00:00 out_voltage0_type
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 out_voltage1_en
   -r--r--r--    1 root     root          4096 Jan  1 00:00 out_voltage1_index
   -r--r--r--    1 root     root          4096 Jan  1 00:00 out_voltage1_type

.. include:: /include/iio-snippets.rst
   :start-after: .. start-typical-adc-scan-elements
   :end-before: .. end-typical-adc-scan-elements

Low level register access
~~~~~~~~~~~~~~~~~~~~~~~~~

.. include:: /include/iio-snippets.rst
   :start-after: .. start-low-level-register-access-via-debugfs-direct-reg-access
   :end-before: .. end-low-level-register-access-via-debugfs-direct-reg-access

AD9122 Clocking concept
~~~~~~~~~~~~~~~~~~~~~~~

There are three different clocks associated with the DAC.

- Data/Interface Clock (DCI)
- DAC Clock (DACCLK)
- Reference Clock (REFCLK)

The Data/Interface Clock (DCI) is shared with the DDS HDL Core.

The default AD9122 driver configuration uses a concept called Direct Clocking.
This means an external clock provider must supply DCI and DACCLK. In case the
DAC interpolation feature is not used and the DAC operates in Word-Mode then DCI
= DACCLK.

When N interpolation is used following equation must be satisfied.

N = {1, 2, 4, 8}

DACCLK = N \* DCI and DACCLK <= 1230MHz

The *out_altvoltage_interpolation_frequency_available* attribute allows you to
query the supported interpolation frequencies for a given Interface Clock
(*out_altvoltage_X_sampling_frequency*) by exercising possible settings on the
clock provider. In case the clock provider is not capable providing exactly the
requested rate, the mode is removed from the list.

**So following example:**

Assuming the clock provider can only provide following rates: 1000MHz / X, X =
{1..1023}.

This results in following rates:

1000, 500, 333.33, 250, 200, 166.66, 142.85, 125, 111.11, …

- In case the interface clock is set to 125MHz, then 1x, 2x, 4x and 8x
  interpolation is possible.

- Assuming the interface clock is set to 166.66MHz, then only 1x, 2x
  interpolation is possible. 4x interpolation won’t work since 666.66MHz cannot
  be supplied by the clock provider.

The half-band interpolation filters have selectable pass bands that allow the
center frequencies to be moved in increments of one-half their input data rate.
The premodulation block provides a digital upconversion of the incoming waveform
by one-half the incoming data rate, fDATA. This can be used to frequency-shift
base- band input data to the center of the interpolation filter pass band.

The available center shift frequencies for a given Interface Clock
(*out_altvoltage_X_sampling_frequency*), can be queried using the
*out_altvoltage_interpolation_center_shift_frequency_availabl*\ e attribute.

::

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat out_altvoltage_1A_sampling_frequency
   491520000

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat out_altvoltage_interpolation_frequency_available
   491520000 983040000

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat out_altvoltage_interpolation_frequency
   491520000

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat out_altvoltage_interpolation_center_shift_frequency
   0

::

   //[ Set data clock to 122.88 MHz ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# echo 122880000 >
   out_altvoltage_1A_sampling_frequency

   //[ List available interpolation DAC frequencies ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat
   out_altvoltage_interpolation_frequency_available
   122880000 245760000 491520000 983040000

   //[ Select 8x interpolation ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# echo 983040000 >
   out_altvoltage_interpolation_frequency

   //[ List available center shift frequencies ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat
   out_altvoltage_interpolation_center_shift_frequency_available
   0 61440000 122880000 184320000 245760000 307200000 368640000 430080000
   491520000 552960000 614400000 675840000 737280000 798720000 860160000
   921600000

   //[ Select 2x interpolation ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# echo 245760000 >
   out_altvoltage_interpolation_frequency

   //[ List available center shift frequencies ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# cat
   out_altvoltage_interpolation_center_shift_frequency_available
   0 61440000 122880000 184320000

   //[ Select center shift frequencies ]//

   root@linaro-ubuntu-desktop:/sys/bus/iio/devices/iio:device4# echo 61440000 >
   out_altvoltage_interpolation_center_shift_frequency

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
