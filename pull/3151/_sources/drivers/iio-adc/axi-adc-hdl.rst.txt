.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-adc/axi-adc-hdl

.. _axi-adc-hdl:

AXI ADC HDL
===========

AXI ADC HDL Linux Driver.

Supported Devices
-----------------

- :adi:`AD6676`
- :adi:`AD9208`
- :adi:`AD9234`
- :adi:`AD9250`
- :adi:`AD9265`
- :adi:`AD9361`
- :adi:`AD9364`
- :adi:`AD9371`
- :adi:`AD9434`
- :adi:`AD9467`
- :adi:`AD9625`
- :adi:`AD9643`
- :adi:`AD9649`
- :adi:`AD9652`
- :adi:`AD9680`
- :adi:`AD9683`
- :adi:`AD9684`

Supported Boards
----------------

This driver supports the

- :dokuwiki:`AD-FMCOMMS1-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms1-ebz>`
- :dokuwiki:`AD-FMCOMMS2-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms2-ebz>`
- :dokuwiki:`AD-FMCOMMS3-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms3-ebz>`
- :dokuwiki:`AD-FMCOMMS4-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms4-ebz>`
- :dokuwiki:`AD-FMCOMMS5-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms5-ebz>`
- :dokuwiki:`AD-FMCOMMS6-EBZ FMC Card <resources/eval/user-guides/ad-fmcomms6-ebz>`
- :external+hdl:ref:`fmcjesdadc1`
- :dokuwiki:`AD-FMCADC2-EBZ FMC Card <resources/eval/user-guides/ad-fmcadc2-ebz>`
- :dokuwiki:`AD-FMCDAQ2-EBZ FMC Card <resources/eval/user-guides/ad-fmcdaq2-ebz>`
- :external+hdl:ref:`ad9467_fmc`
- :external+hdl:ref:`ad9467_fmc`
- :external+hdl:ref:`fmcjesdadc1`
- :external+hdl:ref:`ad9265_fmc`
- :dokuwiki:`ADRV9371 FMC Card <resources/eval/user-guides/mykonos>`

Sub device Documentation (linked mode)
--------------------------------------

- :ref:`ad9208`

Description
-----------

The AXI ADC HDL driver is the driver for :external+hdl:ref:`axi_adc` which is
used on various FPGA designs. The driver is implemented as an Linux IIO driver.
It’s register map can be found here:
:external+hdl:ref:`generic-adc-register-access`

This driver is split into two parts. A control driver let’s call it SPI-ADC
which configures the converter internal control registers, this part is
typically instantiated via the SPI bus. (see:
:git-linux:`ad9467.c <drivers/iio/adc/ad9467.c>`,
:git-linux:`ad9361_conv.c <drivers/iio/adc/ad9361_conv.c>` or
:git-linux:`ad9371_conv.c <mykonos:drivers/iio/adc/ad9361_conv.c>`) Device
probing for the data capture driver (AXI-ADC) which controls the AXI HDL core
registers and the DMA, is delayed until the SPI control driver is fully probed.
The device tree phandle ``**spibus-connected**`` is used to connect the capture
driver with is SPI control driver. This split is required since the AXI-ADC and
the SPI-ADC parts are instantiated via different busses. The AXI-ADC driver
registers the IIO device, the SPI-ADC instance doesn’t. However a shared data
structure (struct axiadc_converter) is used so that the methods local to the
SPI-ADC driver can extend the IIO attributes provided the AXI-ADC driver. There
is also a callback provided (post_setup) which calls a from the AXI-ADC into the
AXI-SPI driver after the AXI-ADC is fully alive. This post setup callback is
then typically used to finally configure the digital data path, test and tune
the digital data interface etc.

For the **AD9361** and **AD9371** family of transceivers, things are a bit more
differentiated. In fact these devices have a separate IIO device for the radio
control portion. We call them the PHY devices.
(:git-linux:`ad9361.c <drivers/iio/adc/ad9361.c>` and
:git-linux:`ad9371.c <drivers/iio/adc/ad9371.c>` ) The **PHY** drivers
are intended to be independent from our AXI-ADC capture drivers and underlying
HDL designs. Therefore things related to the AXI-ADC driver are located in the
ad93X1_conv.c files.

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
   * - :git-linux:`drivers/iio/adc/cf_axi_adc_core.c`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/iio/adc/cf_axi_adc_core.c>`__
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
     - :git-linux:`drivers/iio/adc/ad6676.c`
     -
   * - driver
     - :git-linux:`drivers/iio/adc/ad9208.c`
     -
   * - driver
     - :git-linux:`drivers/iio/adc/ad9467.c`
     -
   * - driver
     - :git-linux:`drivers/iio/adc/ad9680.c`
     -
   * - driver
     - :git-linux:`drivers/iio/adc/ad9361_conv.c`
     -
   * - driver
     - :git-linux:`drivers/iio/adc/ad9371_conv.c <drivers/iio/adc/ad9361_conv.c>`
     -
   * - driver
     - :git-linux:`drivers/iio/adc/adrv9009_conv.c`
     -
   * - core driver
     - :git-linux:`drivers/iio/adc/cf_axi_adc_core.c`
     -
   * - core driver
     - :git-linux:`drivers/iio/adc/cf_axi_adc_ring_stream.c`
     -
   * - core include
     - :git-linux:`drivers/iio/adc/cf_axi_adc.h`
     -

Example platform device initialization
--------------------------------------

The AXI ADC driver is a platform driver and can currently only be instantiated
via device tree.

Required devicetree properties:

- **compatible**: Should always be one of these:

  - ``xlnx,cf-ad9467-core-1.00.a``
  - ``xlnx,axi-adc-1c-1.00.a``
  - ``xlnx,axi-ad9234-1.00.a``
  - ``xlnx,axi-ad9250-1.00.a``
  - ``xlnx,axi-ad9434-1.00.a``
  - ``adi,axi-ad9643-6.00.a``
  - ``adi,axi-ad9361-6.00.a``
  - ``adi,axi-ad9371-6.00.a``
  - ``adi,axi-ad9680-1.0``
  - ``adi,axi-ad9625-1.0``
  - ``adi,axi-ad6676-1.0``
  - ``adi,axi-ad9684-1.0``
  - ``adi,axi-ad9371-rx-1.0``
  - ``adi,axi-ad9684-1.0``
  - ``adi,axi-adrv9009-rx-1.0``
  - ``adi,axi-ad9208-1.0``
  - For a complete list see driver source: static const struct of_device_id
    axiadc_of_match[]

- **reg**: Base address and register area size. This parameter expects a
  register range.
- **spibus-connected**: Phandle to the SPI device on which the AD9467/AD9643 can
  be found
- **dmas**: List of DMA controller phandle. DMA specifiers for tx and rx dma.
  See the DMA client binding, Documentation/devicetree/bindings/dma/dma.txt
- **dma-names**: DMA request names should include ``tx`` and ``rx`` if present.

Example:

::

   &fmc_spi {
       adc_ad9467: ad9467@0 {
           #address-cells = <1>;
           #size-cells = <0>;
           compatible = "ad9467";
           reg = <0>;
           spi-max-frequency = <10000000>;
           clocks = <&clk_ad9517 3>;
           clock-names = "adc_clk";

           adi,spi-3wire-enable;
       };
   };

   &fpga_axi {
       rx_dma: rx-dmac@44a30000 {
           compatible = "adi,axi-dmac-1.00.a";
           reg = <0x44A30000 0x10000>;
           #dma-cells = <1>;
           interrupts = <0 57 0>;
           clocks = <&clkc 16>;

           adi,channels {
               #size-cells = <0>;
               #address-cells = <1>;

               dma-channel@0 {
                   reg = <0>;
                   adi,source-bus-width = <16>;
                   adi,source-bus-type = <2>;
                   adi,destination-bus-width = <64>;
                   adi,destination-bus-type = <0>;
               };
           };
       };

       cf_ad9467_core_0: cf-ad9467-core-lpc@44a00000 {
           compatible = "xlnx,cf-ad9467-core-1.00.a";
           reg = <0x44A00000 0x10000>;
           dmas = <&rx_dma 0>;
           dma-names = "rx";

           spibus-connected = <&adc_ad9467>;
       };
   };

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The AXI ADC HDL driver depends on **CONFIG_SPI**

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
               -*- Analog Devices High-Speed AXI ADC driver core
                   <*> Analog Devices AD9208 and similar high speed ADCs
                   <*> Analog Devices AD9371 RF Transceiver driver
                   <*> Analog Devices ADRV9009/ADRV9008 RF Transceiver driver
                   <*> Analog Devices AD6676 Wideband IF Receiver driver
                   <*> Analog Devices AD9467 etc. high speed ADCs
                   <*> Analog Devices AD9680 and similar high speed ADCs
           [--snip--]

Hardware configuration
----------------------

Driver testing
--------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-device-files
   :end-before: .. end-iio-device-files

::

   root:/> cd /sys/bus/iio/devices/
   root:/sys/bus/iio/devices> ls
   iio:device4  iio:trigger0

   root:/sys/bus/iio/devices> cd iio:device4

   root:/sys/bus/iio/devices/iio:device4> ls -l
   drwxr-xr-x    2 root     root             0 Jan  1 00:00 buffer
   -r--r--r--    1 root     root          4096 Jan  1 00:00 dev
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_calibbias
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_calibphase
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_calibscale
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_filter_high_pass_3db_frequency
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_test_mode
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_calibbias
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_calibphase
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_calibscale
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_filter_high_pass_3db_frequency
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_test_mode
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_sampling_frequency
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_scale
   -r--r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_scale_available
   -r--r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_test_mode_available
   -r--r--r--    1 root     root          4096 Jan  1 00:00 name
   drwxr-xr-x    2 root     root             0 Jan  1 00:00 scan_elements
   lrwxrwxrwx    1 root     root             0 Jan  1 00:00 subsystem -> ../../../../bus/iio
   -rw-r--r--    1 root     root          4096 Jan  1 00:00 uevent
   root:/sys/bus/iio/devices/iio:device4>

Show device name
~~~~~~~~~~~~~~~~

::

   root:/sys/bus/iio/devices/iio:device4> cat name
   cf-ad9643-core-lpc

Show scale
^^^^^^^^^^

**Description:** scale to be applied to in_voltageX_raw in order to obtain the
measured voltage in millivolts.

::

   root:/sys/bus/iio/devices/iio:device4> cat in_voltage_scale
   0.026703

Show available scales
'''''''''''''''''''''

::

   root:/sys/bus/iio/devices/iio:device4> cat in_voltage_scale_available
   0.031738 0.031403 0.031067 0.030731 0.030396 0.030060 0.029724 0.029388 0.029053 0.028717 0.028381 0.028046 0.027710 0.027374 0.027039 0.026703 0.026367 0.026031 0.025696 0.025360 0.025024 0.024689 0.024353 0.024017 0.023682 0.023346 0.023010 0.022675 0.022339 0.022003 0.021667 0.021332

Set ADC calibration gain
''''''''''''''''''''''''

**Description:** in_voltage0_calibscale in_voltage1_calibscale

Set the channel calibration gain. Writing to these files will set the
calibration gain for the respective channel. Valid values are in the range of
0..1.999999

::

   root:/sys/bus/iio/devices/iio:device4> cat in_voltage0_calibscale
   1.000000

Set ADC calibration bias
''''''''''''''''''''''''

**Description:** in_voltage0_calibbias in_voltage1_calibbias

Set the channel calibration bias/offset. Writing to these files will set the
calibration bias for the respective channel. Valid values are in the range of
+/- 16384.

::

   root:/sys/bus/iio/devices/iio:device4> cat in_voltage0_calibscale
   1.0

Show available ADC test modes
'''''''''''''''''''''''''''''

**Description:**

Show available test modes supported by the underlying ADC. These test modes are
typically used to test the high speed digital interface between the converter
and interface adaptor.

::

   root:/sys/bus/iio/devices/iio:device4> cat in_voltage_test_mode_available
   off midscale_short pos_fullscale neg_fullscale checkerboard pn_long pn_short one_zero_toggle

Set ADC test mode
'''''''''''''''''

**Description:** in_voltage0_test_mode in_voltage1_test_mode

Enter test modes supported by the underlying ADC. These test modes are typically
used to test the high speed digital interface between the converter and
interface adaptor.

::

   root:/sys/bus/iio/devices/iio:device4> echo one_zero_toggle > in_voltage0_test_mode
   root:/sys/bus/iio/devices/iio:device4> cat in_voltage0_test_mode
   one_zero_toggle
   root:/sys/bus/iio/devices/iio:device4> echo off > in_voltage0_test_mode

External Synchronization
''''''''''''''''''''''''

The :external+hdl:ref:`ad_ip_jesd204_tpl_adc` core supports the EXT_SYNC
feature, allowing to synchronize multiple channels within a ADC or across
multiple instances, see ``External synchronization`` section of
:external+hdl:ref:`ad_ip_jesd204_tpl_adc`. This feature can also synchronize
between the :external+hdl:ref:`ad_ip_jesd204_tpl_adc` and
:external+hdl:ref:`ad_ip_jesd204_tpl_dac` core.

There are two device attributes which allows controlling this feature:
``sync_start_enable`` and ``sync_start_enable_available`` reading the later
returns the available modes which depend on HDL core synthesis parameters. The
options are explained below. Reading ‘sync_start_enable’ returns either ‘arm’
while waiting for the external synchronization signal or ‘disarm’ otherwise.

- ``arm``: Setting this key will arm the trigger mechanism sensitive to an
  external sync signal. Once the external sync signal goes high it synchronizes
  channels within a ADC, and across multiple instances. This key has an effect
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

   root:/sys/bus/iio/devices/iio:device4/buffer> ls
   bytes_per_datum          enable                   subsystem
   length                   uevent
   root:/sys/bus/iio/devices/iio:device4/buffer>

.. include:: /include/iio-snippets.rst
   :start-after: .. start-buffer-management
   :end-before: .. end-buffer-management

::

   root:/sys/bus/iio/devices/iio:device4/scan_elements> ls
   in_voltage0_en
   in_voltage0_index
   in_voltage0_type
   in_voltage1_en
   in_voltage1_index
   in_voltage1_type
   root:/sys/bus/iio/devices/iio:device4/scan_elements>

.. include:: /include/iio-snippets.rst
   :start-after: .. start-typical-adc-scan-elements
   :end-before: .. end-typical-adc-scan-elements

More Information
----------------

.. include:: /include/iio-snippets.rst
   :start-after: .. start-iio-pointers
   :end-before: .. end-iio-pointers
