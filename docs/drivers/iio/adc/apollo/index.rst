AD9084/AD9088
=============

The Apollo mixed signal front-end (MxFE®) is a highly integrated device with a
16-bit, 28/16 GSPS maximum sample rate, RF digital-to-analog converter (DAC)
core, and 12-bit, 20/8 GSPS maximum sample rate, RF analog-to-digital converter
(ADC) core. The :adi:`AD9084`/:adi:`AD9088` supports eight transmit channels
and eight receive channels. The :adi:`AD9084`/:adi:`AD9088` is well suited for
applications requiring both wideband ADCs and DACs to process signal(s) having
wide instantaneous bandwidth. The device features a 48 lane, 28.1/32.51 Gbps
JESD204C or 16/20 Gbps JESD204B data transceiver port, an on-chip clock
multiplier, and a digital signal processing (DSP) capability targeted at either
wideband or multi-band, direct to RF applications. The
:adi:`AD9084`/:adi:`AD9088` also features a bypass mode that allows the full
bandwidth capability of the ADC and/or DAC cores to bypass the DSP datapaths.
The device also features low latency loopback and frequency hopping modes
targeted at phased array radar systems and electronic warfare applications.

Supported Devices
-----------------

* :adi:`AD9084`
  (:adi:`Data Sheet <media/en/technical-documentation/data-sheets/ad9084.pdf>`)
* :adi:`AD9088`
  (:adi:`Data Sheet <media/en/technical-documentation/data-sheets/ad9088.pdf>`)

Evaluation Boards
-----------------

* | :adi:`EVAL-AD9084`
  | :adi:`User Guide (UG-2326) <media/en/technical-documentation/user-guides/eval-ad9084-ug-2326.pdf>`

Source Code
-----------

Is the driver in mainland? :red:`No`.

The driver is divided into two groups: The API, and the Linux driver. The API
is as an abstraction layer between the application and the hardware and is
agnostic processor and operating system integration. The Linux Driver uses the
API to connect the device to the Linux Kernel subsystems.

.. note::

   The standalone sources for the API can be requested at :adi:`AD9084` main
   page.

Linux Driver

* :git-linux:`ad9088.c <drivers/iio/adc/apollo/ad9088.c>` - Main driver
* :git-linux:`ad9088.h <drivers/iio/adc/apollo/ad9088.h>` - Driver header
* :git-linux:`ad9088_dt.c <drivers/iio/adc/apollo/ad9088_dt.c>` - Device tree parsing
* :git-linux:`ad9088_bmem.c <drivers/iio/adc/apollo/ad9088_bmem.c>` - Buffer memory interface
* :git-linux:`ad9088_cal.c <drivers/iio/adc/apollo/ad9088_cal.c>` - Calibration save/restore
* :git-linux:`ad9088_debugfs.c <drivers/iio/adc/apollo/ad9088_debugfs.c>` - Debugfs interface
* :git-linux:`ad9088_fft.c <drivers/iio/adc/apollo/ad9088_fft.c>` - FFT sniffer support
* :git-linux:`ad9088_ffh.c <drivers/iio/adc/apollo/ad9088_ffh.c>` - Fast frequency hopping
* :git-linux:`ad9088_jesd204_fsm.c <drivers/iio/adc/apollo/ad9088_jesd204_fsm.c>` - JESD204 FSM callbacks
* :git-linux:`ad9088_mcs.c <drivers/iio/adc/apollo/ad9088_mcs.c>` - MCS calibration
* :git-linux:`ad9088_cal_dump.c <drivers/iio/adc/apollo/tools/ad9088_cal_dump.c>` - Calibration dump tool

API:

* :git-linux:`adi_inc <drivers/iio/adc/apollo/adi_inc>`
* :git-linux:`adi_utils <drivers/iio/adc/apollo/adi_utils>`
* :git-linux:`private <drivers/iio/adc/apollo/private>`
* :git-linux:`public <drivers/iio/adc/apollo/public>`

Interrelated Device Drivers
~~~~~~~~~~~~~~~~~~~~~~~~~~~

These are drivers that are connected to or is a dependency of ``ad9088.c``.

* :dokuwiki:`JESD204 (FSM) Interface Linux Kernel Framework <resources/tools-software/linux-drivers/jesd204/jesd204-fsm-framework>`
* :dokuwiki:`JESD204 Interface Framework <resources/fpga/peripherals/jesd204>`

* Transport Layer Receive AXI-ADC driver

  * Sources:

    * :git-linux:`drivers/iio/adc/cf_axi_adc_core.c`
    * :git-linux:`drivers/iio/adc/cf_axi_adc_ring_stream.c`
    * :git-linux:`drivers/iio/adc/cf_axi_adc.h`

  * Documentation:

    * :dokuwiki:`AXI ADC HDL Linux Driver <resources/tools-software/linux-drivers/iio-adc/axi-adc-hdl>`

* Transport Layer Transmit AXI-DAC / DDS driver

  * Sources:

    * :git-linux:`drivers/iio/frequency/cf_axi_dds.c`
    * :git-linux:`drivers/iio/frequency/cf_axi_adc.h`

  * Documentation:

    * :dokuwiki:`AXI DAC HDL Linux Driver <resources/tools-software/linux-drivers/iio-dds/axi-dac-dds-hdl>`

* Link Layer AXI JESD204B HDL driver

  * Sources:

    * :git-linux:`drivers/iio/jesd204/axi_jesd204_rx.c`
    * :git-linux:`drivers/iio/jesd204/axi_jesd204_tx.c`

  * Documentation:

    * :dokuwiki:`JESD204B/C Transmit Linux Driver <resources/tools-software/linux-drivers/jesd204/axi_jesd204_tx>`
    * :dokuwiki:`JESD204B/C Receive Linux Driver <resources/tools-software/linux-drivers/jesd204/axi_jesd204_rx>`

* PHY Layer AXI JESD204B GT (Gigabit Transceiver) HDL driver (XILINX/ALTERA-INTEL)

  * Sources:

    * :git-linux:`drivers/iio/jesd204/axi_adxcvr.c`

  * Documentation:

    * :dokuwiki:`JESD204B/C AXI_ADXCVR Highspeed Transceivers Linux Driver <resources/tools-software/linux-drivers/jesd204/axi_adxcvr>`

.. Add cfir pfir to docs


Devicetree
----------

The following example devicetrees are provided:

* :git-linux:`AD9084-VCU118 RevB <arch/microblaze/boot/dts/vcu118_quad_ad9084_revB.dts>`: vcu118_quad_ad9084_revB.dts
* :git-linux:`AD9084-VCU118 26p4 RevB Ext <arch/microblaze/boot/dts/vcu118_quad_ad9084_26p4_revB_ext.dts>`: vcu118_quad_ad9084_26p4_revB_ext.dts
* :git-linux:`AD9084-VCU118 26p4 RevB <arch/microblaze/boot/dts/vcu118_quad_ad9084_26p4_revB.dts>`: vcu118_quad_ad9084_26p4_revB.dts
* :git-linux:`AD9084-VCU118 204C_M4_L8_NP12_16p2_6x1 <arch/microblaze/boot/dts/vcu118_ad9084_204C_M4_L8_NP12_16p2_6x1.dts>`: vcu118_ad9084_204C_M4_L8_NP12_16p2_6x1.dts
* :git-linux:`AD9084-VCU118 ad9084 <arch/microblaze/boot/dts/vcu118_ad9084.dts>`: vcu118_ad9084.dts
* :git-linux:`AD9084-VCU118 RevB Ext <arch/microblaze/boot/dts/vcu118_quad_ad9084_revB_ext.dts>`: vcu118_quad_ad9084_revB_ext.dts
* :git-linux:`AD9084-VCU118 204C_M4_L8_NP16_20p0_4x4 <arch/microblaze/boot/dts/vcu118_ad9084_204C_M4_L8_NP16_20p0_4x4.dts>`: vcu118_ad9084_204C_M4_L8_NP16_20p0_4x4.dts
* :git-linux:`AD9084-VCK190 <arch/arm64/boot/dts/xilinx/versal-vck190-reva-ad9084.dts>`: versal-vck190-reva-ad9084.dts
* :git-linux:`AD9084-VCK190 204C_M4_L4_NP16_20p0_4x4 <arch/arm64/boot/dts/xilinx/versal-vck190-reva-ad9084-204C-M4-L4-NP16-20p0-4x4.dts>`: versal-vck190-reva-ad9084-204C-M4-L4-NP16-20p0-4x4.dts

Profile
~~~~~~~

* ``adi,device-profile-fw-name``: Set the profile firmware to use.

See :ref:`apollo profile` for more information.

Calibration
~~~~~~~~~~~

* ``adi,device-calibration-data-name``: Set the calibration data firmware file
  to load during driver probe. This enables automatic calibration restore from
  a previously saved calibration file, significantly reducing boot time.

If not present, the driver performs normal initialization with full calibration.
See :ref:`calibration data management` for more information.

Bring-up
~~~~~~~~~

* ``adi,delayed-serdes-jrx-cal-en``: Run SERDES calculation during the
  "link running" phase instead of the earlier "clocks enable" phase.

Format
~~~~~~

* ``adi,rx-real-channel-en``: Disable complex Q/I samples for RX path.
* ``adi,tx-real-channel-en``:  Disable complex Q/I samples for RX path.

In practice, doesn't set the channel modifier, treating each channel as
independent real channel.
If not present, even number are I samples, and odd Q samples.

Trigger
~~~~~~~

* ``adi,trigger-sync-en``: Enable trigger synchronization.
* ``adi,trig-req-gpio``: Set the GPIO that trigger Enable trigger synchronization.
  If not provided, the SPI trigger will be used for trigger synchronization.

If the option is enabled and no trigger is provided, the synchronization will
time out.

Multi-Chip Synchronization (MCS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ``adi,mcs-track-decimation``: TDC decimation rate for MCS tracking calibration.
  A larger value improves precision at the expense of longer TDC measurement time.
  For systems using gapped periodic SYSREF, keep this value below 32768.
  Default: 1023. Type: u16.

  .. code:: dts

     trx0_ad9084: ad9088@0 {
         compatible = "adi,ad9088";
         /* ... other properties ... */
         adi,mcs-track-decimation = /bits/ 16 <1023>;
     };

Topology
~~~~~~~~

* ``adi,side-b-use-seperate-tpl-en``: Set side B to use own transport layer.
* ``adi,standalone-enable``: Allocate own IIO device to device.
  Top device always allocates IIO device through ``axiadc_converter``,
  therefore has no effect on it.
* ``adi,multidevice-instance-count``: Define the **total** number of devices in
  the system on the main device.
  Only present if the subordinates uses ``adi,standalone-enable``.

``adi,multidevice-instance-count`` is a companion of the ``adi,standalone-enable``
attribute.
For a topology of 4 devices, the main device has it set to 4, and the 3 subordinates
contain the latter attribute.

SPI
~~~

* ``adi,spi-3wire-enable``: Use 3-wire SPI for SPI0.

HSCI
~~~~

A few options are available to tune the HSCI interface.

* ``adi,axi-hsci-connected``: Sets the HSCI controller for the HSCI interface.
  If absent, only SPI is used to communicate with the device, considerably slower.
* ``adi,hsci-auto-linkup-mode-en``: Enable HSCI auto-linkup mode.
* ``adi,hsci-disable-after-boot-en``: Use SPI as the interface after initial device initialization.

GPIO
~~~~

* ``adi,gpio-exports``: Expose a list of the 30s device GPIOs into a GPIO controller.

A more detailed usage is provided :ref:`here <apollo gpio>`.

Spectrum Sniffer
~~~~~~~~~~~~~~~~

* ``interrupt-names``: ``fft_done_A"`` and ``fft_done_B`` for the spectrum
  sniffer interrupt signal for side a and b, respectively.

A more detailed usage is provided :ref:`here <apollo fft>`.

Compile the driver
------------------

Configure kernel with ``make menuconfig``
(alternatively use ``make xconfig`` or ``make qconfig``).

.. code:: text

   Linux Kernel Configuration
       Device Drivers  --->
           <*> JESD204 High-Speed Serial Interface Framework
       <*> Industrial I/O support --->
           --- Industrial I/O support
            -*-   Enable buffer support within IIO
            <*>     IIO callback buffer used for push in-kernel interfaces
            -*-     Industrial I/O DMA buffer infrastructure
            -*-     Industrial I/O DMA buffer integration with DMAEngine
            [*]       Enables I/O DMA buffer legacy MMAP support
            -*-     Industrial I/O HW buffering
            -*-     Industrial I/O buffering based on kfifo
            -*-     Industrial I/O triggered buffer support
            -*-   Enable IIO configuration via configfs
            -*-   Enable triggered sampling support
            <*>   Enable software IIO device support
            <*>   Enable software triggers support
            <*>   Enable triggered events support
            <*>   Analog Devices AXI ADXCVR PHY Support
            <*>   Generic AXI JESD204B configuration driver
            <*>   Analog Devices AXI JESD204B TX Support
            <*>   Analog Devices AXI JESD204B RX Support
            <*>   Analog Devices Generic IIO fake device driver
                  Light sensors  --->
                  Logic Analyzers  --->
            <*>   AXI AION Trigger support

              Analog to digital converters --->
            -*- Analog Devices High-Speed AXI ADC driver core
                    <*> Analog Devices AD9088 and similar Mixed Signal Front End (MxFE)

       Frequency Synthesizers DDS/PLL  --->
               Direct Digital Synthesis  --->
                <*> Analog Devices CoreFPGA AXI DDS driver
           Clock Generator/Distribution  --->
                        <*> Analog Devices HMC7044, HMC7043 Clock Jitter Attenuator with JESD204B
                        <*> Analog Devices ADF4030 10-Channel Precision Synchronizer

       <*>   JESD204 High-Speed Serial Interface Support  --->
           --- JESD204 High-Speed Serial Interface Support
           <*>   Analog Devices AXI ADXCVR PHY Support
           <*>   Analog Devices AXI JESD204B TX Support
           <*>   Analog Devices AXI JESD204B RX Support

.. _apollo profile:

Device profile
--------------

The device profiles can be generated with the profile generator tool that is
part of the :adi:`ACE` Evaluation Software. Profile modifications can be done
through :external+pyadi-jif:doc:`pyadi-jif <index>`.
In particular, the a Python Jupyter Notebook at :git-pyadi-jif:`examples/triton/triton_vcu118.ipynb`
guides you through the profile generation and modification, with sample files
at the :git-pyadi-jif:`directory of the notebook <examples/triton>`.
During runtime, some profile options are reconfigured, such as
:ref:`apollo ffh`.

With the device profile generated, compile into the kernel by adding it to the
*firmware* folder and appending too the ``CONFIG_EXTRA_FIRMWARE`` kernel
symbol. The *firmware* folder is set by the symbol
``CONFIG_EXTRA_FIRMWARE_DIR``, if it is set to something else, add to that path
instead. Then, update devicetree property ``adi,device-profile-fw-name``.

You can have multiple profiles compiled into the kernel image by passing
multiple profile paths, however only one set in the devicetree property.
Consider using devicetree overlays to swapping profiles.

Usage
-----

General IIO and driver conventions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Controlling the MxFE is done via the IIO sysfs interface. For convenience users
can use :external+documentation:ref:`libiio` and its various programming
languages bindings.

The MxFE IIO device under /sys/bus/iio/devices/iio:deviceX features a set of
channels and device attributes, which are explained here. Some basic, but
important concepts are explained in the bullet list below:

* Channels prefixed with ``**in_**voltageX`` apply to Receive (RX) ADC data paths.
* Channels prefixed with ``**out**_voltageX`` apply to Transmit (TX) DAC data paths.
* Each channel has a complex modifier ``in_voltageX_**i**`` and
  ``in_voltageX_**q**`` or ``out_voltageX_**i**`` and ``out_voltageX_**q**``.
  Controlling a channel attribute for the ``i`` modified channel will
  simultaneously control the ``q`` modified channel and vice versa. So,
  writing/reading only needs to happen once, since they are mirrored. The
  complex IQ modifiers are only important for the data buffers, sine I & Q are
  individual data components.
* IIO channels ``[in|out]_voltageX`` apply to the channelizer data paths (Fine DDC/DUC).

  - Each IIO channel has some ``[in|out]_voltageX_[i|q]_**channel**_[attributes]``
  - In case the channelizer data paths (Fine DDC/DUC) are bypassed
    (decimation=1 or interpolation=1), the X still applies to a data path.

* Each channelizer data path (FDDC, FDUC) connects at least to one Coarse
  DDC/DUC (CDDC/CDUC), which maps then to one or more ADCs/DACs depending on
  configuration. These are called main data paths and are controlled via the
  ``[in|out]_voltageX_[i|q]_**main**_[attributes]`` attributes.

  - Be aware, since multiple channels can map to the same main data path
    (CDDC/CDUC), changing a main attribute of one channel will also update same
    attribute of any other channel that maps to the same main data path
    (CDDC/CDUC).
  - The crossbar mapping between Fine and Coarse Digital Up/Down Converters,
    ADCs/DACs is defined in the device tree.

* Device attributes (without ``**in_**voltageX``or ``**out_**voltageX`` prefix)
  apply to the entire device.

Driver testing
~~~~~~~~~~~~~~

This device can be found under */sys/bus/iio/devices/*

One way to check if the device and driver are present is using **iio_info**:

.. shell::
   :no-path:

   $iio_info
        iio:device0: ad4052 (buffer capable)
                1 channels found:
                        voltage0:  (input, index: 0, format: le:s16/32>>0)
                        2 channel-specific attributes found:
                                attr  0: raw value: 12167
                                attr  1: sampling_frequency value: 1000000
                1 device-specific attributes found:
                                attr  0: waiting_for_supplier value: 0
                3 buffer-specific attributes found:
                                attr  0: data_available value: 0
                                attr  1: direction value: in
                                attr  2: length_align_bytes value: 8
                1 debug attributes found:
                                debug attr  0: direct_reg_access value: 0x10
                No trigger on this device

You can go to the device folder using:

.. shell::
   :no-path:

   $cd $(grep -rw /sys/bus/iio/devices/*/name -e "ad9088" -l | xargs dirname)


.. collapsible:: In the folder there are several files that can set specific device attributes:

   .. shell::

      /sys/bus/iio/devices/iio:device8
      $ls -l
       total 0
       drwxr-xr-x 2 root root    0 Feb 11 15:17 buffer
       drwxr-xr-x 2 root root    0 Feb 11 15:17 buffer0
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 calibration_data
       --w------- 1 root root 4096 Feb 11 15:17 cfir_config
       -r--r--r-- 1 root root 4096 Feb 11 15:17 dev
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_temp0_input
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_temp0_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_bmem_sample_delay
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_cnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_cnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_cnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_cnco_select
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_fnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_fnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_fnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_ffh_fnco_select
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_loopback
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_bmem_sample_delay
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_hb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_tb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_nyquist_zone
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_sampling_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_test_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_bmem_sample_delay
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_cnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_cnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_cnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_cnco_select
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_fnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_fnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_fnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_ffh_fnco_select
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_loopback
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_bmem_sample_delay
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_hb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_tb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_nyquist_zone
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_sampling_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_test_mode
       ...
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_adc_frequency
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_channel_nco_frequency_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_channel_nco_mixer_mode_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_loopback_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_main_nco_frequency_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_main_nco_mixer_mode_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_nyquist_zone_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_sampling_frequency
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_sampling_mode_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_test_mode_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_ctrl
       -r--r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_error
       -r--r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_paused
       --w------- 1 root root 4096 Feb 11 15:17 jesd204_fsm_resume
       -r--r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_state
       -r--r--r-- 1 root root 4096 Feb 11 15:17 loopback1_blend_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 loopback1_blend_side_a
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 loopback1_blend_side_b
       -r--r--r-- 1 root root 4096 Feb 11 15:17 name
       lrwxrwxrwx 1 root root    0 Feb 11 15:17 of_node -> ../../../../../firmware/devicetree/base/fpga-axi@0/axi-ad9084-rx-hpc@a4a10000
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_cnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_cnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_cnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_cnco_select
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_fnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_fnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_fnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_ffh_fnco_select
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_invsinc_en
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_cnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_cnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_cnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_cnco_select
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_fnco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_fnco_index
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_fnco_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_ffh_fnco_select
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_invsinc_en
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_mixer_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_test_tone_scale
       ...
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_channel_nco_frequency_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_channel_nco_mixer_mode_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_dac_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_main_nco_frequency_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_main_nco_mixer_mode_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_sampling_frequency
       --w------- 1 root root 4096 Feb 11 15:17 pfilt_config
       drwxr-xr-x 2 root root    0 Feb 11 15:17 power
       drwxr-xr-x 2 root root    0 Feb 11 15:17 scan_elements
       lrwxrwxrwx 1 root root    0 Feb 11 15:17 subsystem -> ../../../../../bus/iio
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 sync_start_enable
       -r--r--r-- 1 root root 4096 Feb 11 15:17 sync_start_enable_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 uevent
       -r--r--r-- 1 root root 4096 Feb 11 15:17 waiting_for_supplier

Show channel name
^^^^^^^^^^^^^^^^^

The channel label contains the data path and it is shown with:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $cat in_voltage2_q_label
    Side-B:FDDC0->CDDC0->ADC0

ADC Rate
^^^^^^^^

What: ``in_voltage_adc_frequency``

Read only attribute which returns the RX ADC rate Hz.

.. shell::

   /sys/bus/iio/devices/iio:device2
   $cat in_voltage_adc_frequency
    4000000000

RX Sample Rate
^^^^^^^^^^^^^^

What: ``in_voltage_sampling_frequency``

Read only attribute which returns the RX digital IQ base-band rate in
Hz. This must not be confused with the ADC rate, which is
(Main_decimation \* Channel_decimation) time higher. Main_decimation,
Channel_decimation are defined in the device tree.

in_voltage_sampling_frequency = :math:`in\_voltage\_adc\_frequency / (Main\_decimation * Channel\_decimation)`

.. shell::

   /sys/bus/iio/devices/iio:device2
   $cat in_voltage_sampling_frequency
    250000000

DAC Rate
^^^^^^^^

What: ``out_voltage_dac_frequency``

Read only attribute which returns the TX DAC rate Hz.

.. shell::

   /sys/bus/iio/devices/iio:device2
   $cat out_voltage_dac_frequency
    20000000000

TX Sample Rate
^^^^^^^^^^^^^^

What: ``out_voltage_sampling_frequency``

Read only attribute which returns the TX digital IQ baseband rate in
Hz. This must not be confused with the DAC rate, which is
(Main_interpolation \* Channel_interpolation) time higher.
Main_interpolation, Channel_interpolation are defined in the device
tree.

out_voltage_sampling_frequency = :math:`f_{DAC} / ({Main\_interpolation} * {Channel\_interpolation}`)

.. shell::

   /sys/bus/iio/devices/iio:device2
   $cat out_voltage_sampling_frequency
    250000000

NCO Frequency Control
^^^^^^^^^^^^^^^^^^^^^

Main Data Path
++++++++++++++

What: ``[in|out]_voltageX_[i|q]_main_nco_frequency``

Sets the main data path (CDDC/CDUC) NCO frequency (f\ :sub:`Carrier`) in Hz

- ``out_voltageX_i_main_nco_frequency`` Range is: :math:`−f_{DAC}/2 ≤ f_{Carrier} < +f_{DAC}/2`
- ``in_voltageX_i_main_nco_frequency`` Range is: :math:`−f_{ADC}/2 ≤ f_{Carrier} < +f_{ADC}/2`

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 1000000000 > out_voltage0_i_main_nco_frequency
   $cat out_voltage0_i_main_nco_frequency
    1000000000

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 300000000 > in_voltage0_i_main_nco_frequency
   $cat in_voltage0_i_main_nco_frequency
    300000000

Channel Data Path
+++++++++++++++++

What: ``[in|out]_voltageX_[i|q]_channel_nco_frequency``

Sets the channel data path (FDDC/FDUC) NCO frequency (f\ :sub:`Carrier`) in Hz

``out_voltageX_i_channel_nco_frequency`` range is:

 .. math::

    −(f_{DAC}/Main\_interpolation)/2 ≤ f_{Carrier} < +(f_{DAC}/Main\_interpolation)/2

``in_voltageX_i_channel_nco_frequency`` range is:

.. math::

   −(f_{ADC}/Main\_decimation)/2 ≤ f_{Carrier}< +(f_{ADC}/Main\_decimation)/2

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 1000000000 > out_voltage0_i_channel_nco_frequency
   $cat out_voltage0_i_channel_nco_frequency
    1000000000

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 300000000 > out_voltage0_i_channel_nco_frequency
   $cat out_voltage0_i_channel_nco_frequency
    300000000

NCO Phase Control
^^^^^^^^^^^^^^^^^

Main Data Path
++++++++++++++++

What: ``[in|out]_voltageX_[i|q]_main_nco_phase``

Sets the main data path (CDDC/CDUC) NCO phase offset in milli degrees

Range is: −180° ≤ Degrees Offset ≤ +180° (Values are in milli degrees.)

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 66000 > out_voltage0_i_main_nco_phase
   $cat out_voltage0_i_main_nco_phase
    66000

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo -42000 > in_voltage0_i_main_nco_phase
   $cat in_voltage0_i_main_nco_phase
    -42000

Channel Data Path
+++++++++++++++++

What: ``[in|out]_voltageX_[i|q]_channel_nco_phase``

Sets the channel data path (FDDC/FDUC) NCO phase offset in milli degrees

Range is: −180° ≤ Degrees Offset ≤ +180° (Values are in milli degrees.)

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 13123 > out_voltage0_i_channel_nco_phase
   $cat out_voltage0_i_channel_nco_phase
    13123

TX NCO Channel Digital Gain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

What: ``out_voltageX_[i|q]_channel_nco_gain_scale``

The input data into each channelizer stage can be rescaled prior to
additional processing. This feature is useful in multiband applications
to prevent digital clipping when the outputs of two or more channelizer
stages are summed in the main datapath to produce a multiband band
signal.
The gain/scale is set via ``out_voltageX_[i|q]_channel_nco_gain_scale`` attribute.

Range is: :math:`0 ≤ Gain ≤ 1.999 (−∞ dB < dBGain ≤ +6.018 dB)`

.. shell::

   /sys/bus/iio/devices/iio:device2
   $echo 0.707 > out_voltage0_i_channel_nco_gain_scale
   $cat out_voltage0_i_channel_nco_gain_scale
    0.707

NCO Mixer Mode
^^^^^^^^^^^^^^

What: ``[in|out]_voltageX_[i|q]_[main|channel]_nco_mixer_mode``

Configures the NCO mixer mode for the main (CDDC/CDUC) or channel (FDDC/FDUC)
data path. The mixer mode controls how the NCO frequency is applied to the
signal.

Available modes can be read from the ``[in|out]_voltage_[main|channel]_nco_mixer_mode_available``
attribute:

* ``var_IF``: Variable IF mode (default)
* ``zero_IF``: Zero IF mode
* ``fs/4_IF``: Fs/4 IF mode
* ``test_tone``: Test tone mode

.. shell::

   /sys/bus/iio/devices/iio:device8
   $cat in_voltage_main_nco_mixer_mode_available
    var_IF zero_IF fs/4_IF test_tone
   $echo var_IF > in_voltage0_i_main_nco_mixer_mode
   $cat in_voltage0_i_main_nco_mixer_mode
    var_IF

NCO Test Tone
^^^^^^^^^^^^^

What: ``[in|out]_voltageX_[i|q]_[main|channel]_nco_test_tone_scale``

Sets the NCO test tone scale for path testing. The test tone scale controls
the amplitude of the internally generated test tone. The value is a floating
point scale factor.

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 0.5 > in_voltage0_i_main_nco_test_tone_scale
   $cat in_voltage0_i_main_nco_test_tone_scale
    0.500000000

BMEM Sample Delay
^^^^^^^^^^^^^^^^^

What: ``in_voltageX_[i|q]_[main|channel]_bmem_sample_delay``

Sets the buffer memory (BMEM) sample delay for the main (CDDC) or channel
(FDDC) data path. This delay is used to align data paths when capturing
samples to the buffer memory.

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 10 > in_voltage0_i_main_bmem_sample_delay
   $cat in_voltage0_i_main_bmem_sample_delay
    10

Sampling Mode
^^^^^^^^^^^^^

What: ``in_voltageX_[i|q]_sampling_mode``

Configures the RX channel sampling mode. Available modes can be read from
the ``in_voltage_sampling_mode_available`` attribute:

* ``random``: Random sampling mode (default)
* ``sequential``: Sequential sampling mode

.. shell::

   /sys/bus/iio/devices/iio:device8
   $cat in_voltage_sampling_mode_available
    random sequential
   $echo random > in_voltage0_i_sampling_mode
   $cat in_voltage0_i_sampling_mode
    random

Temperature Sensor
^^^^^^^^^^^^^^^^^^

What: ``in_temp0_input``, ``in_temp0_label``

The AD9088 includes an internal temperature sensor. The temperature can be
read in millidegrees Celsius via the ``in_temp0_input`` attribute. The
``in_temp0_label`` attribute provides the channel data path label.

.. shell::

   /sys/bus/iio/devices/iio:device8
   $cat in_temp0_label
    Side-A:FDDC0->CDDC0->ADC0
   $cat in_temp0_input
    53000

Calibration Data
^^^^^^^^^^^^^^^^

What: ``calibration_data``

The ``calibration_data`` attribute provides access to the device calibration
data. This can be used to save and restore calibration across power cycles.
See :ref:`calibration data management` for detailed usage.

PFIR
^^^^

The PFIR (Programmable Finite Impulse Response) configuration is defined the
configuration ``pfilt_config`` (write-only).

.. literalinclude: /../drivers/iio/adc/apollo/ad9088.c
   :prepend: /**
   :start-at: * ad9088_parse_pfilt -
   :end-before: static int ad9088_parse_pfilt

The ``pfilt_config`` input must have the following format:

.. code:: ini

   mode: <imode> <qmode>
   gain: <ix> <iy> <qx> <qy>
   scalar_gain: <ix> <iy> <qx> <qy>
   dest: <terminal> <pfilt_sel> <bank_sel>
   hc_delay: <delay>
   mode_switch_en: <value>
   mode_switch_add_en: <value>
   real_data_mode_en: <value>
   quad_mode_en: <value>
   selection_mode: <mode>
   <sval>

* Each line represents a parameter or setting for the PFIR filter.
* Lines starting with '#' are considered comments and are ignored.

The format for each parameter is as follows:

- | mode: <imode> <qmode>

  Sets the mode for the PFIR filter. The <imode> and <qmode> values should be
  one of the predefined filter modes: ``disabled``, ``real_n4``, ``real_n2``,
  ``undef``, ``matrix``, ``undef``, ``complex_half``, ``real_n``.

- | gain: <ix> <iy> <qx> <qy>

  Sets the gain values for the PFIR filter. The <ix>, <iy>, <qx>,
  and <qy> values should be integers representing the gain in dB.

- | scalar_gain: <ix> <iy> <qx> <qy>

  Sets the scalar gain values for the PFIR filter. The <ix>, <iy>,
  <qx>, and <qy> values should be integers representing the scalar
  gain.

- | dest: <terminal> <pfilt_sel> <bank_sel>

  Sets the destination for the PFIR filter. The <terminal> value should be
  either ``rx`` or ``tx``. The <pfilt_sel> value should be one of the
  predefined filter selects: ``pfilt_a0``, ``pfilt_a1``, ``pfilt_b0``,
  ``pfilt_b1``, ``pfilt_all``, ``pfilt_mask``. The <bank_sel> value should be
  one of the predefined filter banks: ``bank_0``, ``bank_1``, ``bank_2``,
  ``bank_3``, ``bank_all``, ``bank_mask``.

- | hc_delay: <delay>

  Sets the high cut delay value for the PFIR filter. The <delay>
  value should be an unsigned 8-bit integer.

- | mode_switch_en: <value>

  Sets the mode switch enable value for the PFIR filter. The <value>
  should be either 0 or 1.

- | mode_switch_add_en: <value>

  Sets the mode switch add enable value for the PFIR filter. The
  <value> should be either 0 or 1.

- | real_data_mode_en: <value>

  Sets the real data mode enable value for the PFIR filter. The
  <value> should be either 0 or 1.

- | quad_mode_en: <value>

  Sets the quad mode enable value for the PFIR filter. The <value>
  should be either 0 or 1.

- | selection_mode: <mode>

  Sets the profile selection mode for the PFIR filter. The <mode> value
  should be one of the predefined profile selection modes: ``direct_regmap``,
  ``direct_gpio``, ``direct_gpio1``, ``trig_regmap``, ``trig_gpio``,
  ``trig_gpio1``.

- | <sval>

  Sets the coefficient values for the PFIR filter. The <sval> value
  should be an integer representing the coefficient value.

CFIR
^^^^

The CFIR configuration is defined by enabling ``[in|out]_voltage<index>_[i|q]_cfir_profile_sel``,
profile selection ``[in|out]_voltage<index>_[i|q]_cfir_profile_sel``
and the configuration ``cfir_config`` (write-only).

The ``cfir_config`` input must have the following format:

.. literalinclude: /../drivers/iio/adc/apollo/ad9088.c
   :prepend: /**
   :start-at: * ad9088_parse_cfilt -
   :end-before: static int ad9088_parse_cfilt

The ``cfir_config`` input must have the following format:

.. code:: ini

   dest: <terminal> <cfir_select> <cfir_profile> <cfir_datapath>
   gain: <gain_value>
   complex_scalar: <scalar_i> <scalar_q>
   bypass: <bypass_value>
   sparse_filt_en: <sparse_filt_en_value>
   32taps_en: <32taps_en_value>
   coeff_transfer: <coeff_transfer_value>
   enable: <enable_value> <enable_profile_value>
   selection_mode: <selection_mode_value>
   <cfir_coeff0_i> <cfir_coeff0_q>
   <cfir_coeff1_i> <cfir_coeff1_q>
   ...
   <cfir_coeffN_i> <cfir_coeffN_q>

Each line in the string represents a specific configuration parameter.
The ``dest`` line specifies the destination terminal, CFIR select, CFIR profile,
and CFIR datapath. The ``gain`` line specifies the gain value. The ``complex_scalar``
line specifies the complex scalar values. The ``bypass`` line specifies the bypass
value. The ``sparse_filt_en`` line specifies the sparse filter enable value.
The ``32taps_en`` line specifies the 32 taps enable value. The ``coeff_transfer``
line specifies the coefficient transfer value. The ``enable`` line specifies the
enable value and enable profile value. The ``selection_mode`` line specifies the
selection mode value. The <cfir_coeff_i> and <cfir_coeff_q> lines specify the
CFIR coefficient values.

.. shell::

   /sys/bus/iio/devices/iio:device8
   $ls | grep cfir
    cfir_config
    in_voltage0_i_cfir_en
    in_voltage0_i_cfir_profile_sel
    in_voltage1_i_cfir_en
    in_voltage1_i_cfir_profile_sel
    ...
    out_voltage0_i_cfir_en
    out_voltage0_i_cfir_profile_sel
    ...
    out_voltage3_q_cfir_en
    out_voltage3_q_cfir_profile_sel
   $echo 1 > in_voltage0_i_cfir_en
   $cat in_voltage0_i_cfir_en
    1

One main use case of CFIR is to equalize the RF carriers independent of PFILT.

Loopback
^^^^^^^^

The four loopback modes are available as channel attributes.
The value is always written to a side: A or B, so
enabling loopback for in_voltage0 will also enable for in_voltage1.

.. shell::

   /sys/bus/iio/devices/iio:device8
   $ls | grep loopback
    in_voltage0_i_loopback
    in_voltage0_q_loopback
    in_voltage1_i_loopback
    in_voltage1_q_loopback
    in_voltage2_i_loopback
    in_voltage2_q_loopback
    in_voltage3_i_loopback
    in_voltage3_q_loopback
    in_voltage_loopback_available
    loopback1_blend_available
    loopback1_blend_side_a
    loopback1_blend_side_b
   $cat in_voltage_loopback_available
    off loopback0 loopback1 loopback2 loopback3_jesd
   $echo loopback1 > in_voltage2_i_loopback
   $cat in_voltage3_q_loopback
    loopback1

If the ADC and DAC does not have the same sampling frequency, writing will
fail with -EINVAL.
Consult the User Guide for all configuration requirements before enabling the loopback.

.. _apollo fft:

Spectrum Sniffer
^^^^^^^^^^^^^^^^

The Spectrum Sniffer provides users information with fast indications of where
the strong and weak signals are within the digitized signal bandwidth.

The interrupt-names ``fft_done_A`` and ``fft_done_B`` maps the spectrum
sniffer interrupt for side A and B, respectively. The debug attributes for the
spectrum sniffer for each side are only probed if the respective interrupt is
present.

With the spectrum sniffer interrupts mapped, an additional iio device is probed
and the fft properties are exposed as properties and debug properties.

.. shell::

   $cd /sys/bus/iio/devices/iio:device6
   $ls
    buffer   max_threshold  mode_available  power          uevent
    buffer0  min_threshold  name            scan_elements  waiting_for_supplier
    dev      mode           of_node         subsystem

- ``buffer``/``buffer0``: Allocated buffer to store and read captured data.

- ``max_threshold``: Maximum threshold for comparison with magnitude Output
  from FFT Engine.

  | Default: ``20``
  | Range: ``0-255``

- ``min_threshold``: Minimum threshold for comparison with magnitude Output
  from FFT Engine.

  | Default: ``0``
  | Range: ``0-255``

- ``mode``: Spectrum output mode during data capture, affects both IQ and
  Magnitude modes.

  | ``normal``: Normal mode - uses averaging when alpha_factor is set.
  | ``instant``: Instantaneous/Debug mode - captures single snapshot.
  | Default: ``instant``

- ``mode_available``: Returns all valid spectrum output modes.

- ``adc_select``: Select which ADC input to use for the FFT sniffer.

  | ``adc0``: ADC 0 (available on 4T4R and 8T8R)
  | ``adc1``: ADC 1 (available on 4T4R and 8T8R)
  | ``adc2``: ADC 2 (8T8R only)
  | ``adc3``: ADC 3 (8T8R only)
  | Default: ``adc0``

- ``adc_select_available``: Returns all valid ADC selections for the device.

Debugfs Attributes
++++++++++++++++++

Advanced configuration is available through debugfs. These attributes allow
fine-tuning of the FFT sniffer behavior and are primarily intended for
debugging and development.

.. shell::

   $cd /sys/kernel/debug/iio/iio:device6
   $ls
    adc                   direct_reg_access  min_threshold
    adc_sampling_rate_Hz  dither_enable      real_mode
    alpha_factor          fft_enable_sel     run_fft_engine_background
    bottom_fft_enable     fft_hold_sel       sort_enable
    continuous_mode       low_power_enable   window_enable
    delay_capture_ms      max_threshold

**Initialization Parameters:**

- ``adc``: ADC to configure and use for FFT capture. This is a bitmask value.

  | Default: ``ADI_APOLLO_ADC_0``

- ``fft_hold_sel``: Control the Sniffer ``fft_hold`` bit by SPI operation
  (via API function) or GPIO to hold overwriting of data to registers.

  | ``0``: Select GPIO control.
  | ``1``: Select registers control.
  | Default: ``0``

- ``fft_enable_sel``: Control the Sniffer ``fft_enable`` bit by SPI operation
  (via API function) or GPIO to enable the FFT Engine.

  | ``0``: Select GPIO control.
  | ``1``: Select registers control.
  | Default: ``1``

- ``real_mode``: Select between real and complex FFT operation.

  | ``0``: Select Complex FFT. Pair of converters used for I and Q sampling.
  | ``1``: Select Real FFT.
  | Default: ``0``

- ``max_threshold``: Maximum threshold for comparison with magnitude output
  from FFT Engine. Same as the device property.

  | Default: ``255``
  | Range: ``0-255``

- ``min_threshold``: Minimum threshold for comparison with magnitude Output
  from FFT Engine.

  | Default: ``0``
  | Range: ``0-255``

**FFT Processing Parameters:**

- ``sort_enable``: Incoming magnitude data is sorted from least to greatest
  before it is stored. When sorting is disabled, magnitude data is left
  unsorted and is ordered by its bin number. Sorting is only available in
  magnitude mode.

  | ``0``: Disable.
  | ``1``: Enable.
  | Default: ``1``

- ``continuous_mode``: In continuous mode, the sniffer continuously stores new
  data, guaranteeing that the latest data is stored. In single mode data is
  only stored once between the time that ``fft_hold`` is set to low, and the
  data is obtained. Hence, the user may miss critical data during this period
  in single mode. Continuous mode is only supported in magnitude mode.

  | ``0``: Single mode.
  | ``1``: Continuous mode.
  | Default: ``0``

- ``bottom_fft_enable``: Each sniffer instance has one top and one bottom FFT
  engine that are 50% overlapped and compute in parallel. With this feature,
  the top and bottom FFT engine results are averaged and assist in avoiding
  critical time domain events

  | ``0``: Disable.
  | ``1``: Enable.
  | Default: ``0``

- ``window_enable``: A hard-coded frequency domain Hamming window is applied to
  and is optimized for narrow spectral peaks to optimize the resolution
  bandwidth. in I/Q output mode, windowing will deemphasize data towards the
  edges of the 512-bin spectrum boundary, and thus the threshold may not be
  reached resulting in data not being captured.

  | ``0``: Disable.
  | ``1``: Enable.
  | Default: ``1``

- ``low_power_enable``: Low power mode disables continuous mode and exponential
  averaging. The expected power savings for this mode is 600mW.

  | ``0``: Disable.
  | ``1``: Enable.
  | Default: ``1``

- ``dither_enable``: Dither is added to input samples to reduce spurs across
  the spectrum.

  | ``0``: Disable.
  | ``1``: Enable.
  | Default: ``1``

**Averaging and Signal Processing:**

- ``alpha_factor``: Exponential averaging is enabled by setting the
  alpha_factor between 1 and 8 and disabled when set to 0. The alpha factor
  sets :math:`\alpha` in the equation
  :math:`y[n] = 2^{-\alpha}x[n]+(1-2^{-\alpha}y[n-1])`
  and directly controls the weight of the previous sample. The average is reset
  to zero when data is stored or if FFT computation is stopped. Exponential
  averaging is only available in single mode and magnitude mode.

  | ``0``: Disable.
  | ``1-8``: Enable with specified alpha value.
  | Default: ``0``

**Capture Control:**

- ``run_fft_engine_background``: Set to 1 for GPIO control, to skip writing
  the ``fft_enable`` register.

  | ``0``: Disable.
  | ``1``: Enable.
  | Default: ``0``

- ``delay_capture_ms``: Delay capture by an amount of time.

  | Default: ``100``
  | Unit: ``ms``

- ``adc_sampling_rate_Hz``: Read the ADC sampling rate (read-only).

The acquired data is pushed to the device's IIO buffer.

Feature Availability by Mode
++++++++++++++++++++++++++++

The following table shows which features are available in each sniffer mode:

.. list-table:: FFT Sniffer Feature Matrix
   :header-rows: 1
   :widths: 30 15 15 15 15

   * - Feature
     - Normal Mag
     - Instant Mag
     - Normal IQ
     - Instant IQ
   * - ``sort_enable``
     - ✓
     - ✓
     - ✗
     - ✗
   * - ``continuous_mode``
     - ✓
     - ✗
     - ✗
     - ✗
   * - ``alpha_factor`` (averaging)
     - ✓
     - ✗
     - ✗
     - ✗
   * - ``bottom_fft_enable``
     - ✓
     - ✓
     - ✓
     - ✓
   * - ``window_enable``
     - ✓
     - ✓
     - ✓
     - ✓
   * - ``dither_enable``
     - ✓
     - ✓
     - ✓
     - ✓
   * - ``low_power_enable``
     - ✓
     - ✓
     - ✓
     - ✓
   * - Threshold detection
     - ✓
     - ✓
     - ✗
     - ✗
   * - FFT bins output
     - 256
     - 256
     - 512
     - 512

Modes of Operation
++++++++++++++++++

The spectrum sniffer supports four distinct modes of operation, determined by
the combination of ``mode`` setting and channel selection (IQ vs Magnitude):

**Magnitude Modes** (enable ``in_magn0_en`` channel):

1. **Normal Magnitude** (``mode=normal``): Captures magnitude spectrum with
   optional exponential averaging (when ``alpha_factor`` > 0), sorting
   (``sort_enable=1``), and continuous capture (``continuous_mode=1``).
   Best for ongoing spectrum monitoring.

2. **Instant Magnitude** (``mode=instant``): Single-shot magnitude capture
   without averaging. Best for quick snapshots of spectrum state.

**IQ Modes** (enable ``in_voltage0_i_en`` and ``in_voltage0_q_en`` channels):

3. **Normal IQ** (``mode=normal``): Captures complex I/Q FFT data. Sorting,
   continuous mode, and exponential averaging are not available in IQ mode.
   Best for detailed spectral analysis requiring phase information.

4. **Instant IQ** (``mode=instant``): Single-shot complex I/Q capture.
   Best for debug and calibration purposes.

.. note::

   The mode is automatically determined based on which channels are enabled
   in ``scan_elements/``. Enabling the magnitude channel selects magnitude
   mode, while enabling I/Q channels selects IQ mode.

IIO Channels
++++++++++++

The FFT sniffer exposes the following IIO channels:

- ``in_index``: FFT bin index (0-255 for real mode, 0-511 for complex mode)
- ``in_voltage0_i``: In-phase (I) FFT data for IQ mode
- ``in_voltage0_q``: Quadrature (Q) FFT data for IQ mode
- ``in_magn0``: Magnitude FFT data for magnitude mode

**Channel Scan Masks** - valid combinations:

- Index + I + Q: For IQ mode captures (``in_index_en=1``, ``in_voltage0_i_en=1``,
  ``in_voltage0_q_en=1``)
- Index + Magnitude: For magnitude mode captures (``in_index_en=1``,
  ``in_magn0_en=1``)

Data Format
+++++++++++

**Magnitude Mode** (real FFT, 256 bins):

Each sample contains:

- Bin index (9 bits unsigned, 0-255)
- Magnitude value (9 bits unsigned)

**IQ Mode** (complex FFT, 512 bins):

Each sample contains:

- Bin index (9 bits unsigned, 0-511)
- I data (9 bits signed)
- Q data (9 bits signed)

All values are packed into 16-bit little-endian format.

Example Usage
+++++++++++++

**Capture Magnitude Spectrum:**

.. shell::

   # Configure for magnitude mode with sorting
   $cd /sys/bus/iio/devices/iio:device6
   $echo normal > mode
   $echo 1 > /sys/kernel/debug/iio/iio:device6/sort_enable

   # Enable magnitude channels
   $echo 1 > scan_elements/in_index_en
   $echo 1 > scan_elements/in_magn0_en

   # Configure and enable buffer
   $echo 256 > buffer0/length
   $echo 1 > buffer0/enable

   # Capture data
   $cat /dev/iio:device6 | head -c 1024 > spectrum.bin

**Capture IQ Spectrum:**

.. shell::

   # Configure for IQ mode
   $cd /sys/bus/iio/devices/iio:device6
   $echo instant > mode

   # Enable IQ channels
   $echo 1 > scan_elements/in_index_en
   $echo 1 > scan_elements/in_voltage0_i_en
   $echo 1 > scan_elements/in_voltage0_q_en

   # Configure and enable buffer
   $echo 512 > buffer0/length
   $echo 1 > buffer0/enable

   # Capture data
   $cat /dev/iio:device6 | head -c 3072 > iq_spectrum.bin

.. _apollo ffh:

Fast Frequency Hopping
^^^^^^^^^^^^^^^^^^^^^^

The NCO frequency and phase settings can be stored as a set of up-to 32
profiles, each with a 32-bit FTW and 16-bit phase offset word, to be
selectively assigned to the NCO during runtime. This allows to quickly change
(hop) the NCO frequency, known as Fast Frequency Hopping (FFH).

Tx FFH FNCO
+++++++++++

The user configures the NCO FFH frequency by writing the index (0 to 31) to
``out_voltageX_[i|q]_ffh_fnco_index`` followed by the frequency value to
``out_voltageX_[i|q]_ffh_fnco_frequency``, for example:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > out_voltage0_i_ffh_fnco_index
   $echo $((16#20000000)) > out_voltage0_ffh_fnco_frequency

Then hop to the desired frequency writing the index to
``out_voltageX_[i|q]_ffh_fnco_select``, and write -1 to disable the feature;
for example:

.. important::

   If the trigger mode is not set to 4 (``ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP``),
   the method will return -EINVAL;

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > out_voltage0_i_ffh_fnco_select

Tx FFH CNCO
+++++++++++

The user configures the NCO FFH frequency by writing the index (0 to 15) to
``out_voltageX_[i|q]_ffh_cnco_index`` followed by the frequency value to
``out_voltageX_[i|q]_ffh_cnco_frequency``, for example:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > out_voltage0_i_ffh_cnco_index
   $echo $((16#20000000)) > out_voltage0_ffh_cnco_frequency

Then hop to the desired frequency writing the index to
``out_voltageX_[i|q]_ffh_cnco_select``, for example:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > out_voltage0_i_ffh_fnco_select

``out_voltageX_[i|q]_ffh_cnco_frequency`` at index 0 is equivalent to
``in_voltage0_i_main_nco_frequency``

Trigger mode
++++++++++++

By default, the profile will hop on the register access for the profile set call.
Other options are available and configurable with
``out_voltageX_[i|q]_ffh_fnco_mode``.

The options are:

* 0: ``ADI_APOLLO_NCO_CHAN_SEL_TRIG_AUTO``, Trigger based hopping, auto Hopping Mode.
* 1: ``ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP``, Trigger based hopping, scheduled Regmap.
* 2: ``ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO``, Trigger based hopping, scheduled GPIO.
* 3: ``ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO``, Direct GPIO profile select, all params hop together.
* 4: ``ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP``, Direct spi/hsci nco profile select, all params hop together (default).

For example:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > out_voltage0_i_ffh_fnco_mode

Rx FFH FNCO
+++++++++++

The RX path also supports Fast Frequency Hopping. The user configures the
NCO FFH frequency by writing the index (0 to 31) to
``in_voltageX_[i|q]_ffh_fnco_index`` followed by the frequency value to
``in_voltageX_[i|q]_ffh_fnco_frequency``, for example:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > in_voltage0_i_ffh_fnco_index
   $echo 500000000 > in_voltage0_i_ffh_fnco_frequency

Then hop to the desired frequency writing the index to
``in_voltageX_[i|q]_ffh_fnco_select``, and write -1 to disable the feature:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > in_voltage0_i_ffh_fnco_select

The trigger mode is configured via ``in_voltageX_[i|q]_ffh_fnco_mode``.

Rx FFH CNCO
+++++++++++

The user configures the coarse NCO FFH frequency by writing the index (0 to 31) to
``in_voltageX_[i|q]_ffh_cnco_index`` followed by the frequency value to
``in_voltageX_[i|q]_ffh_cnco_frequency``:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > in_voltage0_i_ffh_cnco_index
   $echo 1000000000 > in_voltage0_i_ffh_cnco_frequency

Then hop to the desired frequency writing the index to
``in_voltageX_[i|q]_ffh_cnco_select``:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $echo 1 > in_voltage0_i_ffh_cnco_select

The trigger mode is configured via ``in_voltageX_[i|q]_ffh_cnco_mode``.

.. _apollo bmem:

Buffer Memory (BMEM) Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The AD9088 contains on-chip buffer memory (BMEM) that can be used for data
capture and playback. The driver exposes a separate IIO device for BMEM
operations, named ``ad9088-bmem`` (or with a device-specific label suffix).

**Overview:**

* 128KB SRAM per BMEM instance (32K 32-bit words)
* 8 BMEM instances in 8T8R mode (A0-A3, B0-B3), 4 in 4T4R mode (A0-A1, B0-B1)
* Supports sample delay configuration for data path alignment
* Supports frequency hopping delay profiles

**IIO Device Registration:**

The BMEM interface registers as a separate IIO device with channels corresponding
to each ADC path:

* 8T8R mode: 8 channels (0-3 for Side A, 4-7 for Side B)
* 4T4R mode: 4 channels (0-1 for Side A, 2-3 for Side B)

.. shell::

   $ls /sys/bus/iio/devices/
    iio:device8    # Main AD9088 device
    iio:device9    # BMEM device (ad9088-bmem or <label>-bmem)

**Channel Attributes:**

Each BMEM channel exposes the following sysfs attributes:

.. shell::

   $ls /sys/bus/iio/devices/iio:device9/
    buffer/
    in_voltage0_delay_hop_array
    in_voltage0_delay_hop_parity_check_en
    in_voltage0_delay_hop_sel_mode
    in_voltage0_delay_hop_trig_sclr_en
    in_voltage0_delay_sample
    in_voltage0_delay_sample_config_value
    in_voltage0_delay_sample_parity_check_en
    in_voltage0_delay_start
    in_voltage0_sampling_frequency
    ...
    scan_elements/

BMEM Channel Attributes
+++++++++++++++++++++++

**Sample Delay Configuration:**

- ``in_voltageX_delay_sample``: Set the sample delay value directly. This is a
  simplified interface for setting the delay without other configuration options.

  | Range: 0-65535 (16-bit)
  | Default: 0

  .. shell::
     :no-path:

     # Set 100 sample delay on channel 0
     $echo 100 > in_voltage0_delay_sample
     $cat in_voltage0_delay_sample
      100

- ``in_voltageX_delay_sample_config_value``: Set the sample delay value through
  the full configuration structure. Updates the delay and applies parity settings.

  | Range: 0-65535 (16-bit)
  | Default: 0

- ``in_voltageX_delay_sample_parity_check_en``: Enable or disable parity checking
  for sample delay mode.

  | ``0``: Disable parity check
  | ``1``: Enable parity check (default)

**Frequency Hopping Delay Configuration:**

For fast frequency hopping applications, BMEM supports 4 delay profiles that
can be cycled through or selected via GPIO.

- ``in_voltageX_delay_hop_array``: Configure all 4 delay hop profiles at once.
  Write 4 space-separated values (one per profile).

  | Format: ``<delay0> <delay1> <delay2> <delay3>``
  | Range per value: 0-65535 (16-bit)
  | Default: ``0 0 0 0``

  .. shell::
     :no-path:

     # Set hop delay profiles: 0, 100, 200, 300
     $echo "0 100 200 300" > in_voltage0_delay_hop_array
     $cat in_voltage0_delay_hop_array
      0 100 200 300

- ``in_voltageX_delay_hop_sel_mode``: Select how hop profiles are chosen.

  | ``0``: Cycle through profiles sequentially (0→1→2→3→0...)
  | ``1``: GPIO selects next profile
  | Default: 0

- ``in_voltageX_delay_hop_trig_sclr_en``: Enable trigger mode self-clear.
  When enabled, the trigger automatically clears after activation.

  | ``0``: Disable self-clear
  | ``1``: Enable self-clear (default)

- ``in_voltageX_delay_hop_parity_check_en``: Enable or disable parity checking
  for hop delay mode.

  | ``0``: Disable parity check
  | ``1``: Enable parity check (default)

- ``in_voltageX_delay_start``: Trigger the delay operation for this channel's
  BMEM. Write ``1`` to start the delay operation. Read always returns ``0``.

  .. shell::
     :no-path:

     # Trigger delay start on channel 0
     $echo 1 > in_voltage0_delay_start

**Sampling Frequency:**

- ``in_voltageX_sampling_frequency``: Read the ADC sampling rate for this
  channel (read-only). Value is in Hz.

  .. shell::
     :no-path:

     $cat in_voltage0_sampling_frequency
      2949120000

BMEM Buffer Operations
++++++++++++++++++++++

The BMEM device supports IIO buffer operations for capturing ADC samples.

**Enabling Capture:**

.. shell::
   :no-path:

   # Enable channels for capture
   $echo 1 > /sys/bus/iio/devices/iio:device9/scan_elements/in_voltage0_en
   $echo 1 > /sys/bus/iio/devices/iio:device9/scan_elements/in_voltage1_en

   # Set buffer length (samples per channel)
   $echo 8192 > /sys/bus/iio/devices/iio:device9/buffer/length

   # Enable buffer to start capture
   $echo 1 > /sys/bus/iio/devices/iio:device9/buffer/enable

   # Read captured data
   $cat /dev/iio:device9 > captured_data.bin

   # Disable buffer
   $echo 0 > /sys/bus/iio/devices/iio:device9/buffer/enable

**Data Format:**

Captured samples are 16-bit signed values in little-endian format. When multiple
channels are enabled, samples are interleaved in channel order.

.. _apollo gpio:

GPIOs & GPIO chip
^^^^^^^^^^^^^^^^^

The device contains 30 general purpose GPIOs that can be used to trigger various
changes and can be exported into a :external+linux-upstream:c:struct:`gpio_chip`
using the ``adi,gpio-exports`` devicetree property:

.. code:: dts

   trx0_ad9084: ad9084@0 {
        adi,gpio-exports = /bits/ 8 <15 16 17 18>;
   };

Some of the device GPIOs are mapped in the HDL design to a axi_gpio IP Core, and
the apollo's and axi_gpio's can be attached to other drivers in the device tree:

.. tip::

   ``0`` is the index of item in the ``adi,gpio-exports``, and in the previous
   example, is equivalent to the device's GPIO 15. Also ensure to check the
   schematics and HDL design for the routes between apollo's GPIO, axi_gpio's
   and other input options.

.. code:: dts

   my_driver {
        /* ... */
        provider-gpios = <&axi_gpio 0 GPIO_ACTIVE_HIGH>;
        consumer-gpios = <&tx0_ad9084 0 GPIO_ACTIVE_HIGH>;
   }

Or from user space through `gpiod <https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/>`__:

.. shell::

   $gpiodetect
    gpiochip0 [a4000000.gpio] (64 lines)
    gpiochip1 [versal_gpio] (58 lines)
    gpiochip2 [pmc_gpio] (116 lines)
    gpiochip3 [ad9088] (4 lines)
   $gpioget ad9088 3
    0
   $gpioset a4000000.gpio 3=1
   $gpioget ad9088 3
    1

Debug system
^^^^^^^^^^^^

An additional debug interface is provided through debugfs:

.. shell::

   $cd /sys/kernel/debug/iio/iio\:device8 ; pwd
    /sys/kernel/debug/iio/iio:device8
   $ls
    api_version                 jrx_phase_adjust_calc       mcs_fg_track_cal_run
    bist_2d_eyescan_jrx         jtx_lane_drive_swing        mcs_init
    bist_prbs_error_counters_jrx jtx_lane_post_emphasis     mcs_init_cal_status
    bist_prbs_select_jrx        jtx_lane_pre_emphasis       mcs_track_cal_setup
    bist_prbs_select_jtx        mcs_bg_track_cal_freeze     mcs_track_status
    chip_version                mcs_bg_track_cal_run        misc
    clk_pwr_stat                mcs_cal_run                 pseudorandom_err_check
    die_id                      mcs_dt0_measurement         status
    direct_reg_access           mcs_dt1_measurement         temperature_status
    hsci_enable                 mcs_dt_restore              uuid

JESD204 Link Status
^^^^^^^^^^^^^^^^^^^

Get link statuses:

.. shell::
   :no-path:

   $cat status
    JRX ADI_APOLLO_LINK_A0: JESD204C Subclass=1 L=4 M=4 F=2 S=1 Np=16 CS=0 link_en=Enabled
        Lane0 status: Reset
        Lane1 status: Link is good
        Lane2 status: Reset
        Lane3 status: Link is good
        Lane4 status: Reset
        Lane5 status: Link is good
        Lane6 status: Reset
        Lane7 status: Link is good
        Lane8 status: Reset
        Lane9 status: Reset
        Lane10 status: Reset
        Lane11 status: Reset
        User status: Ready, SYSREF Phase: Locked
    JRX ADI_APOLLO_LINK_B0: JESD204C Subclass=1 L=4 M=4 F=2 S=1 Np=16 CS=0 link_en=Enabled
        Lane0 status: Reset
        Lane1 status: Link is good
        Lane2 status: Reset
        Lane3 status: Link is good
        Lane4 status: Reset
        Lane5 status: Reset
        Lane6 status: Reset
        Lane7 status: Link is good
        Lane8 status: Reset
        Lane9 status: Reset
        Lane10 status: Link is good
        Lane11 status: Reset
        User status: Ready, SYSREF Phase: Locked
    JTX ADI_APOLLO_LINK_A0: JESD204C Subclass=1 L=4 M=4 F=2 S=1 Np=16 CS=0 link_en=Enabled
        PLL locked, PHASE established, MODE valid
    JTX ADI_APOLLO_LINK_B0: JESD204C Subclass=1 L=4 M=4 F=2 S=1 Np=16 CS=0 link_en=Enabled
        PLL locked, PHASE established, MODE valid

Communication Protocol
^^^^^^^^^^^^^^^^^^^^^^

- ``hsci_enable``: Select communication protocol for register access.

  | ``0``: SPI protocol
  | ``1``: HSCI protocol (High-Speed Communication Interface)

.. shell::
   :no-path:

   $cat hsci_enable
    1
   $echo 0 > hsci_enable
   $cat hsci_enable
    0

MCS Calibration
^^^^^^^^^^^^^^^

Multi-Chip Synchronization (MCS) calibration is used for synchronizing multiple
AD9088 devices to achieve ±10ps alignment accuracy. The MCS flow uses BSYNC
Time-of-Flight (ToF) measurements to compensate for path delays between the
ADF4030 precision synchronizer and each Apollo device.

MCS calibration is performed automatically by the driver during initialization
when the ``io-channels`` property with ``bsync`` and ``clk`` channel references
are defined in the Apollo device tree node:

.. code:: dts

   trx0_ad9084: ad9088@0 {
       compatible = "adi,ad9088";
       /* ... other properties ... */
       io-channels = <&adf4030 5>, <&adf4382 0>;
       io-channel-names = "bsync", "clk";

       /* Optional: TDC decimation rate (default: 1023) */
       adi,mcs-track-decimation = /bits/ 16 <1023>;
   };

**Automatic MCS Flow (per UG-2300):**

When io-channels are configured, the driver automatically performs:

1. **BSYNC Time-of-Flight Measurement**: Measures round-trip path delay between
   ADF4030 and Apollo using delta-T0 and delta-T1 measurements.

2. **Path Delay Compensation**: Calculates and applies negative phase offset to
   the ADF4030 output channel to compensate for cable and PCB trace delays.

3. **MCS Init Calibration**: Aligns internal SYSREF to external SYSREF within
   ±0.4 clock cycles. The driver validates the calibration and returns an error
   if alignment fails.

4. **Tracking Calibration Setup**: Configures the TDC decimation rate (from
   device tree or default 1023) and enables tracking.

5. **ADF4382 Auto-Align**: Enables the ADF4382's automatic phase alignment
   feature for continuous synchronization.

6. **Foreground Tracking Calibration**: Runs initial tracking calibration to
   establish baseline phase correction values.

7. **Background Tracking Calibration**: Starts continuous background tracking
   to maintain synchronization over time and temperature.

.. note::

   **Dual Clock Mode Limitation**: MCS tracking calibration is only supported
   in single clock mode due to hardware limitations. In dual clock mode, the
   driver performs MCS init calibration for both A-side and B-side, but skips
   tracking calibration.

**Driver Validation and Warnings:**

The driver performs several validation checks during MCS calibration:

- **MCS Init Cal Failure**: If init calibration fails (SYSREF not locked or
  alignment outside tolerance), the driver returns an error and stops the
  JESD204 state machine.

- **Trigger Phase Margin**: Warns if trigger phase is outside 25%-75% of the
  SYSREF period, which may cause ±1 cycle latency jitter.

- **Path Delay Sanity Check**: Warns if calculated path delay is very small
  (<5% of period) or near maximum (>45% of period), indicating potential
  hardware issues.

For manual control or debugging, MCS calibration attributes are also available
through debugfs.

**Available debugfs attributes:**

.. shell::
   :no-path:

   $ls /sys/kernel/debug/iio/iio:device8/mcs_*
    mcs_bg_track_cal_freeze  mcs_dt1_measurement    mcs_init
    mcs_bg_track_cal_run     mcs_dt_restore         mcs_init_cal_status
    mcs_cal_run              mcs_fg_track_cal_run   mcs_track_cal_setup
    mcs_dt0_measurement      mcs_track_cal_validate mcs_track_status

**Delta-T Measurement:**

- ``mcs_dt0_measurement``: Trigger delta-T0 measurement. Write ``1`` to run.
  Read returns the measured delta-T value in picoseconds.

- ``mcs_dt1_measurement``: Trigger delta-T1 measurement. Write ``1`` to run.
  Read returns the measured delta-T value in picoseconds.

- ``mcs_dt_restore``: Restore delta-T measurement. Write ``1`` to restore.

**Initial Calibration:**

- ``mcs_init``: Initialize MCS calibration. Write ``1`` to setup.

- ``mcs_cal_run``: Run MCS initial calibration. Write ``1`` to execute.
  Read returns ``Passed`` or ``Failed`` based on calibration status.

- ``mcs_init_cal_status``: Read-only. Returns detailed initial calibration
  status including per-side ADC and phase correction results.

**Tracking Calibration:**

- ``mcs_track_cal_setup``: Setup tracking calibration. Write ``1`` to configure.

- ``mcs_fg_track_cal_run``: Run foreground tracking calibration. Write ``1`` to
  execute. Foreground calibration runs once and blocks until complete.

- ``mcs_bg_track_cal_run``: Control background tracking calibration.
  Write ``1`` to start, ``0`` to abort. Read returns current run state.

- ``mcs_bg_track_cal_freeze``: Freeze/unfreeze background tracking calibration.
  Write ``1`` to freeze, ``0`` to unfreeze. Read returns current freeze state.
  Only valid when background tracking is running.

- ``mcs_track_status``: Read-only. Returns tracking calibration status including
  iteration count, phase errors, and per-ADC correction values.

- ``mcs_track_cal_validate``: Read-only. Validates that MCS tracking calibration
  values in Apollo firmware match the ADF4382 hardware bleed current values.
  Use this to verify that tracking calibration is maintaining synchronization.

  .. shell::
     :no-path:

     $cat mcs_track_cal_validate
      MCS Tracking Cal Validation:
        ADF4382 HW:  bleed_pol=0 coarse=5 fine=128
        Apollo FW:   bleed_pol=0 coarse=5 fine=128
        Status:      SYNCHRONIZED

  If values don't match, check for hardware issues or tracking calibration faults.

**Manual MCS Calibration Sequence:**

When MCS is not automatically configured via device tree io-channels, the full
MCS calibration sequence requires coordination between the Apollo device, the
ADF4030 SYSREF generator, and the ADF4382 clock source. The sequence involves
delta-T measurements to calculate path delay and phase compensation.

.. warning::

   The BSYNC signal direction must be carefully managed to avoid bus contention.
   Both Apollo and ADF4030 can drive this signal. The ``apollo_sysref_X_output_enable``
   attribute controls the direction: when enabled (1), ADF4030 drives BSYNC to Apollo;
   when disabled (0), Apollo drives BSYNC back to ADF4030 for the return path
   measurement. Failure to sequence this correctly may result in signal contention
   and incorrect measurements.

The following example shows the complete MCS sequence for a single Apollo device
(device index 0). In a multi-device system, repeat this sequence for each device.

.. code:: shell

   # Set paths for Apollo, ADF4030, and ADF4382
   APOLLO=/sys/kernel/debug/iio/iio:deviceX
   ADF4030=/sys/bus/iio/devices/iio:deviceY
   ADF4382=/sys/bus/iio/devices/iio:deviceZ

   # Step 1: Enable SYSREF output and initialize MCS
   echo 1 > $ADF4030/apollo_sysref_0_output_enable
   echo 1 > $APOLLO/mcs_init

   # Step 2: Delta-T0 measurement (with SYSREF enabled)
   echo 1 > $APOLLO/mcs_dt0_measurement
   apollo_delta_t0=$(cat $APOLLO/mcs_dt0_measurement)
   adf4030_delta_t0=$(cat $ADF4030/apollo_sysref_0_phase)

   # Step 3: Disable SYSREF output for Delta-T1 measurement
   echo 0 > $ADF4030/apollo_sysref_0_output_enable

   # Step 4: Delta-T1 measurement (Apollo driving BSYNC back to ADF4030)
   echo 1 > $APOLLO/mcs_dt1_measurement
   apollo_delta_t1=$(cat $APOLLO/mcs_dt1_measurement)
   adf4030_delta_t1=$(cat $ADF4030/apollo_sysref_0_phase)

   # Step 5: Restore delta-T state
   echo 1 > $APOLLO/mcs_dt_restore

   # Step 6: Calculate path delay (values in femtoseconds)
   # bsync_period = 1e15 / sysref_frequency
   # calc_delay = (adf4030_delta_t0 - adf4030_delta_t1) - (apollo_delta_t1 - apollo_delta_t0)
   # round_trip_delay = (calc_delay + bsync_period) % bsync_period
   # path_delay = round_trip_delay / 2

   # Step 7: Re-enable SYSREF and apply phase compensation
   echo 1 > $ADF4030/apollo_sysref_0_output_enable
   echo -$path_delay > $ADF4030/apollo_sysref_0_phase

   # Step 8: Run MCS initial calibration
   echo 1 > $APOLLO/mcs_cal_run
   cat $APOLLO/mcs_cal_run  # Should show "Passed"

   # Step 9: Setup tracking calibration (single clock mode only)
   echo 1 > $APOLLO/mcs_track_cal_setup

   # Step 10: Enable ADF4382 auto-align for automatic phase adjustment
   # Note: No explicit phase value is set - auto-align handles this automatically
   echo 1 > $ADF4382/out_altvoltage0_en_auto_align

   # Step 11: Run foreground tracking calibration, then start background tracking
   echo 1 > $APOLLO/mcs_fg_track_cal_run
   echo 1 > $APOLLO/mcs_bg_track_cal_run

   # Step 12: Verify calibration status
   cat $APOLLO/mcs_init_cal_status
   cat $APOLLO/mcs_track_status

   # Step 13: (Optional) Validate tracking cal synchronicity
   cat $APOLLO/mcs_track_cal_validate

**Checking Calibration Status:**

.. shell::
   :no-path:

   $cat mcs_init_cal_status
   $cat mcs_track_status

**Stopping Background Tracking Calibration:**

.. shell::
   :no-path:

   $echo 0 > mcs_bg_track_cal_run

Device Information
^^^^^^^^^^^^^^^^^^

Attributes for reading device identification and version information.

- ``api_version``: Read the Apollo API version.

  .. shell::
     :no-path:

     $cat api_version
      1.10.0

- ``chip_version``: Read chip version, revision, and grade.

  .. shell::
     :no-path:

     $cat chip_version
      AD9084 Rev. 4 Grade 3

- ``die_id``: Read the die identification number.

  .. shell::
     :no-path:

     $cat die_id
      DieID 9

- ``uuid``: Read the unique device identifier (128-bit).

  .. shell::
     :no-path:

     $cat uuid
      334d0b28248a328c8e4abc27eb592153

Temperature Monitoring
^^^^^^^^^^^^^^^^^^^^^^

- ``temperature_status``: Read detailed temperature measurements from all
  on-chip thermal sensors.

  .. shell::
     :no-path:

     $cat temperature_status
      TMU (deg C): serdes_pll=47 mpu_a=52 mpu_b=53 adc_a=58 clk_a=51 adc_b=61 clk_b=53 clk_c=51 (avg: 53, avg mask: 0x01fe)

  The output shows temperatures in degrees Celsius for:

  * ``serdes_pll``: SERDES PLL temperature
  * ``mpu_a/mpu_b``: Microprocessor unit A/B temperatures
  * ``adc_a/adc_b``: ADC A/B temperatures
  * ``clk_a/clk_b/clk_c``: Clock domain temperatures
  * ``avg``: Averaged temperature
  * ``avg mask``: Bitmask indicating which sensors are included in the average

Clock Power Status
^^^^^^^^^^^^^^^^^^

- ``clk_pwr_stat``: Read clock input power detection status.

  .. shell::
     :no-path:

     $cat clk_pwr_stat
      Clock input power detection A: GOOD
      Clock input power detection B: UNUSED

  Status values: ``GOOD``, ``UNUSED``, or power-related status indicators.

JESD204 BIST (Built-In Self-Test)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The BIST subsystem provides PRBS (Pseudo-Random Binary Sequence) testing
for JESD204 link integrity verification.

**PRBS Pattern Selection:**

- ``bist_prbs_select_jrx``: Select PRBS pattern for JRX (receive) testing.
  Write the PRBS polynomial order to enable, or 0 to disable.

  | ``0``: Disable PRBS checker
  | ``7``: PRBS7 pattern
  | ``9``: PRBS9 pattern
  | ``15``: PRBS15 pattern
  | ``31``: PRBS31 pattern

  .. shell::
     :no-path:

     # Enable PRBS7 checker
     $echo 7 > bist_prbs_select_jrx
     # Disable PRBS checker
     $echo 0 > bist_prbs_select_jrx

- ``bist_prbs_select_jtx``: Select PRBS pattern for JTX (transmit) testing.
  Write the PRBS polynomial order to enable, or 0 to disable.

  | ``0``: Disable PRBS generator
  | ``7``: PRBS7 pattern
  | ``9``: PRBS9 pattern
  | ``15``: PRBS15 pattern
  | ``31``: PRBS31 pattern

  .. shell::
     :no-path:

     # Enable PRBS31 generator
     $echo 31 > bist_prbs_select_jtx

**PRBS Error Counters:**

- ``bist_prbs_error_counters_jrx``: Read PRBS error counters for all JRX lanes.
  Shows errors/total for each active lane. Read-only.

  .. shell::
     :no-path:

     $cat bist_prbs_error_counters_jrx
      A: lane-5 0/0
      A: lane-1 0/0
      A: lane-3 0/0
      A: lane-7 0/0
      B: lane-1 0/0
      B: lane-7 0/0
      B: lane-10 0/0
      B: lane-3 0/0

**JESD Eye Scan:**

- ``bist_2d_eyescan_jrx``: Trigger a 2D eye scan for JRX SERDES lanes.
  Write lane parameters to initiate the scan; read returns eye diagram data.

  Write format: ``<lane> [prbs] [duration_ms]``

  | ``lane``: Physical lane number (0-23, where 0-11 is side A, 12-23 is side B)
  | ``prbs``: PRBS pattern (7, 9, 15, or 31). Default: 7
  | ``duration_ms``: Measurement duration in milliseconds. Default: 10

  .. shell::
     :no-path:

     # Scan lane 1 with PRBS7
     $echo 1 > bist_2d_eyescan_jrx
     # Scan lane 5 with PRBS31 and 100ms duration
     $echo "5 31 100" > bist_2d_eyescan_jrx
     $cat bist_2d_eyescan_jrx
      # lane 5 spo_steps 16 rate 24750000 spo_left 12 spo_right 12 version 1
      -16,100,-100
      -15,104,-104
      ...

JTX Lane Configuration
^^^^^^^^^^^^^^^^^^^^^^

Configure JESD204 transmit lane signal integrity parameters. Each attribute
takes three space-separated values: ``<link_side_mask> <lane> <value>``.

**Link side mask values:**

| ``1``: Side A only (ADI_APOLLO_LINK_SIDE_A)
| ``4``: Side B only (ADI_APOLLO_LINK_SIDE_B)
| ``5``: Both sides (ADI_APOLLO_LINK_SIDE_ALL)

**Lane number:** Physical lane index 0-11 (per side).

- ``jtx_lane_drive_swing``: Set the output drive swing level for JTX lanes.

  | ``0``: 1000 mV swing (ADI_APOLLO_SER_SWING_1000)
  | ``1``: 850 mV swing (ADI_APOLLO_SER_SWING_850)
  | ``2``: 750 mV swing (ADI_APOLLO_SER_SWING_750)
  | ``3``: 500 mV swing (ADI_APOLLO_SER_SWING_500)

  .. shell::
     :no-path:

     # Set lane 3 on both sides to 850mV swing
     $echo "5 3 1" > jtx_lane_drive_swing

- ``jtx_lane_pre_emphasis``: Set pre-emphasis for JTX lanes to compensate
  for high-frequency signal loss.

  | ``0``: 0 dB pre-emphasis (ADI_APOLLO_SER_PRE_EMP_0DB)
  | ``1``: 3 dB pre-emphasis (ADI_APOLLO_SER_PRE_EMP_3DB)
  | ``2``: 6 dB pre-emphasis (ADI_APOLLO_SER_PRE_EMP_6DB)

  .. shell::
     :no-path:

     # Set lane 5 on side A to 3dB pre-emphasis
     $echo "1 5 1" > jtx_lane_pre_emphasis

- ``jtx_lane_post_emphasis``: Set post-emphasis (de-emphasis) for JTX lanes.

  | ``0``: 0 dB post-emphasis (ADI_APOLLO_SER_POST_EMP_0DB)
  | ``1``: 3 dB post-emphasis (ADI_APOLLO_SER_POST_EMP_3DB)
  | ``2``: 6 dB post-emphasis (ADI_APOLLO_SER_POST_EMP_6DB)
  | ``3``: 9 dB post-emphasis (ADI_APOLLO_SER_POST_EMP_9DB)
  | ``4``: 12 dB post-emphasis (ADI_APOLLO_SER_POST_EMP_12DB)

  .. shell::
     :no-path:

     # Set lane 7 on side B to 6dB post-emphasis
     $echo "4 7 2" > jtx_lane_post_emphasis

JRX Phase Adjustment
^^^^^^^^^^^^^^^^^^^^

- ``jrx_phase_adjust_calc``: Trigger JRX phase adjustment calculation for
  JESD204 link synchronization. Write any value to trigger calculation;
  read returns the calculated phase adjustment value.

Pseudorandom Error Check
^^^^^^^^^^^^^^^^^^^^^^^^

- ``pseudorandom_err_check``: Read the pseudorandom number (PN) sequence
  checker status for all digital channels. Used to verify data path integrity.

  .. shell::
     :no-path:

     $cat pseudorandom_err_check
      CH0 : PN9 : Out of Sync : PN Error
      CH1 : PN9 : Out of Sync : PN Error
      CH2 : PN9 : Out of Sync : PN Error
      CH3 : PN9 : Out of Sync : PN Error
      CH4 : PN9 : Out of Sync : PN Error
      CH5 : PN9 : Out of Sync : PN Error
      CH6 : PN9 : Out of Sync : PN Error
      CH7 : PN9 : Out of Sync : PN Error
      CH8 : UNDEF : In Sync : PN Error
      CH9 : UNDEF : In Sync : PN Error
      CH10 : UNDEF : In Sync : PN Error
      CH11 : UNDEF : In Sync : PN Error
      CH12 : UNDEF : In Sync : PN Error
      CH13 : UNDEF : In Sync : PN Error
      CH14 : UNDEF : In Sync : PN Error
      CH15 : UNDEF : In Sync : PN Error
      CH16 : UNDEF : In Sync : PN Error

  Each line shows:

  * Channel number (CHx)
  * PN sequence type (PN9, PN23, UNDEF, etc.)
  * Sync status (In Sync / Out of Sync)
  * Error status (PN Error / OK)

Miscellaneous
^^^^^^^^^^^^^

- ``misc``: General-purpose miscellaneous register for debug operations.

- ``direct_reg_access``: Standard IIO debugfs attribute for direct SPI
  register read/write access. Useful for low-level debugging.

  .. shell::
     :no-path:

     # Read register 0xF0
     $echo 0xF0 > direct_reg_access
     $cat direct_reg_access
      0xF0

.. _calibration data management:

Calibration Data Management
----------------------------

The AD9088 driver provides a comprehensive calibration data management system that allows you to save and restore device calibration across power cycles. This significantly reduces boot time and maintains consistent performance.

Overview
~~~~~~~~

During normal device initialization, the AD9088 performs extensive calibration of ADCs, SERDES transceivers, and clock conditioning. This process can take considerable time. The calibration data management feature allows you to:

* Save all calibration data to a binary file
* Restore calibration data from file during driver probe
* Skip time-consuming calibration procedures on subsequent boots
* Maintain factory or field calibration data

Calibration Data Components
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The calibration system saves the following data:

* **ADC Calibration**: Sequential and random mode calibration for all ADCs
* **SERDES RX Calibration**: Calibration for all SERDES RX 12-packs
* **Clock Conditioning Calibration**: Clock conditioning calibration for both sides

File Format
~~~~~~~~~~~

Calibration data is stored in a structured binary format with:

* **Magic Number**: ``0x41443930`` ("AD90") for file identification
* **Version**: Format version (currently 2)
* **Device Metadata**: Chip ID (AD9084/AD9088) and configuration (4T4R/8T8R)
* **Section Headers**: Offsets and sizes for each calibration component
* **CRC32 Checksum**: Data integrity validation

Sysfs Interface
~~~~~~~~~~~~~~~

Calibration data is accessed via a binary sysfs attribute:

.. code:: text

   /sys/bus/iio/devices/iio:deviceX/calibration_data

Saving Calibration Data
^^^^^^^^^^^^^^^^^^^^^^^^

After device initialization and calibration, save the calibration data:

.. shell::
   :no-path:

   $cat /sys/bus/iio/devices/iio:device8/calibration_data > /lib/firmware/ad9088_cal.bin
   $ls -lh /lib/firmware/ad9088_cal.bin
    -rw-r--r-- 1 root root 100K Feb 11 15:30 /lib/firmware/ad9088_cal.bin

The save operation:

1. Freezes ADC and SERDES background calibration
2. Updates firmware CRC
3. Reads all calibration data (ADC, SERDES RX, Clock Conditioning)
4. Builds binary file with header and CRC32
5. Unfreezes background calibration

Restoring Calibration Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^

To manually restore calibration data:

.. shell::
   :no-path:

   $cat /lib/firmware/ad9088_cal.bin > /sys/bus/iio/devices/iio:device8/calibration_data

The restore operation validates:

* Magic number and file format version
* Chip ID matches current device
* Device configuration (4T4R/8T8R) matches
* CRC32 integrity

.. note::

   Large calibration files are automatically handled via multi-write accumulation
   (kernel typically splits large writes into 4KB chunks).

Automatic Calibration Load
~~~~~~~~~~~~~~~~~~~~~~~~~~~

For automatic calibration restore during driver probe, add the calibration
firmware property to your device tree:

.. code:: dts

   &spi0 {
       trx0_ad9084: ad9088@ {
           compatible = "adi,ad9088";
           reg = <0>;
           spi-max-frequency = <10000000>;

           /* Automatically load calibration data at boot */
           adi,device-calibration-data-name = "ad9088_cal.bin";

           /* ... other properties ... */
       };
   };

When this property is present, the driver will:

1. Request the firmware file from ``/lib/firmware/`` during probe
2. Validate the calibration data
3. Restore calibration to hardware
4. Continue with normal device initialization

If the property is not present, the driver performs normal initialization with full calibration.

.. important::

   * The calibration file must be placed in ``/lib/firmware/`` before boot
   * If the file is missing or invalid, driver probe will fail
   * Calibration data is loaded after firmware but before other HW configuration
   * Calibration data must match the device (same chip ID and configuration)

Recommended Workflow
~~~~~~~~~~~~~~~~~~~~

First Boot - Capture Calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Boot without calibration restore** (don't add device tree property yet):

   .. shell::
      :no-path:

      $# Device boots and performs full calibration

2. **Save calibration data**:

   .. shell::
      :no-path:

      $cat /sys/bus/iio/devices/iio:device8/calibration_data > /lib/firmware/ad9088_cal.bin

3. **Verify the saved data**:

   .. shell::
      :no-path:

      $ls -lh /lib/firmware/ad9088_cal.bin
      $# Should show file size (typically 50-200KB)

4. **Test restore** (optional but recommended):

   .. shell::
      :no-path:

      $cat /lib/firmware/ad9088_cal.bin > /sys/bus/iio/devices/iio:device8/calibration_data
      $dmesg | grep -i calibration
       ad9088 spi0.0: Calibration data restored successfully

5. **Update device tree** to enable automatic restore:

   .. code:: dts

      adi,device-calibration-data-name = "ad9088_cal.bin";

6. **Rebuild and deploy** device tree

Subsequent Boots
^^^^^^^^^^^^^^^^

On subsequent boots with the device tree property configured, the driver will:

* Automatically load ``/lib/firmware/ad9088_cal.bin``
* Restore calibration data to hardware
* Skip time-consuming calibration procedures
* Boot faster while maintaining performance

Calibration Dump Tool
~~~~~~~~~~~~~~~~~~~~~~

A standalone utility ``ad9088_cal_dump`` is provided for inspecting calibration files:

.. code:: bash

   $cd drivers/iio/adc/apollo/tools
   $make
   $./ad9088_cal_dump /lib/firmware/ad9088_cal.bin

The tool displays:

* File size and CRC validation status
* Header information (magic, version, chip ID, configuration)
* Section offsets and sizes
* Data preview (first 16 bytes of each section)
* Warnings for uninitialized or corrupted data

Example output:

.. code:: text

   File: /lib/firmware/ad9088_cal.bin
   Size: 102464 bytes

   === CRC Validation ===

   Stored CRC:     0x12345678
   Calculated CRC: 0x12345678
   Status:         [OK]

   === AD9088 Calibration Data Header ===

   Magic Number:        0x41443930 ('AD90') [OK]
   Version:             1 [OK]
   Chip ID:             0x9088 (AD9088)
   Configuration:       8T8R (8 TX, 8 RX)
   Number of ADCs:      8
   Number of DACs:      8
   Number of SERDES RX: 4
   Number of SERDES TX: 4

   === Calibration Sections ===

   ADC Calibration:
     Offset: 0x00000040 (64 bytes)
     Size:   0x0000C800 (51200 bytes)
     Per Mode: 25600 bytes
     Per ADC:  3200 bytes

   [... additional sections ...]

Error Handling
~~~~~~~~~~~~~~

Common Errors
^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Error
     - Cause
     - Solution
   * - Invalid magic
     - Wrong file format
     - Use file saved by this driver
   * - Version mismatch
     - Incompatible format version
     - Re-save calibration with current driver
   * - Chip ID mismatch
     - File from different device
     - Use calibration from same chip type
   * - Config mismatch
     - 4T4R vs 8T8R mismatch
     - Use calibration from same device config
   * - CRC error
     - File corruption
     - Re-save calibration data
   * - Size mismatch
     - Truncated file
     - Check file was completely written

Debugging
^^^^^^^^^

Enable kernel debug messages:

.. shell::
   :no-path:

   $echo 8 > /proc/sys/kernel/printk
   $echo 'file ad9088_cal.c +p' > /sys/kernel/debug/dynamic_debug/control
   $dmesg | grep -i "calibration\|ad9088"

Successful save output:

.. code:: text

   ad9088 spi1.0: Saving calibration data...
   ad9088 spi1.0: SERDES JRX enabled mask: 0x0003
   ad9088 spi1.0: Freezing ADC background calibration...
   ad9088 spi1.0: Freezing SERDES JRX background calibration (mask: 0x0003)...
   ad9088 spi1.0: Updating calibration data CRC...
   ad9088 spi1.0: CRC update status: 0
   ad9088 spi1.0: Reading ADC calibration data...
   ad9088 spi1.0: Unfreezing ADC background calibration...
   ad9088 spi1.0: Unfreezing SERDES JRX background calibration...
   ad9088 spi1.0: Reading SERDES RX calibration data...
   ad9088 spi1.0: Reading clock conditioning calibration data...
   ad9088 spi1.0: Calibration data saved: 60708 bytes (ADC: 58624, SERDES RX: 1760, Clk Cond: 240)


Successful restore output:

.. code:: text

   ad9088 spi1.0: Starting calibration restore: 60708 bytes expected
   ad9088 spi1.0: All calibration data received, restoring...
   ad9088 spi1.0: Restoring calibration data...
   ad9088 spi1.0: SERDES JRX enabled mask: 0x0003
   ad9088 spi1.0: Restoring ADC calibration data...
   ad9088 spi1.0: Restoring SERDES RX calibration data...
   ad9088 spi1.0: Restoring clock conditioning calibration data...
   ad9088 spi1.0: Calibration data restored successfully


Best Practices
~~~~~~~~~~~~~~

1. **Verify Saved Data**: Always verify the saved file can be restored before
   relying on it in production

2. **Temperature Considerations**: Calibration data is temperature-dependent:

   * Save calibration at operating temperature
   * Re-calibrate if temperature changes significantly
   * Consider separate calibration files for different temperature ranges

3. **Version Control**: Track calibration data with device information:

   .. shell::
      :no-path:

      $echo "Chip ID: $(dmesg | grep AD9088 | grep 'Chip ID')" > /lib/firmware/ad9088_cal.txt
      $echo "Date: $(date)" >> /lib/firmware/ad9088_cal.txt
      $echo "Config: $(dmesg | grep 'is_8t8r')" >> /lib/firmware/ad9088_cal.txt

Security Considerations
~~~~~~~~~~~~~~~~~~~~~~~

* The ``calibration_data`` sysfs attribute has 0600 permissions (root read/write only)
* Calibration data is device-specific and cannot be shared between different chip instances
* CRC32 validation ensures data integrity
* No sensitive information is stored in calibration data

Source Code
~~~~~~~~~~~

Calibration-related source files:

* :git-linux:`ad9088_cal.c <drivers/iio/adc/apollo/ad9088_cal.c>` - Calibration save/restore implementation
* :git-linux:`ad9088_cal.h <drivers/iio/adc/apollo/ad9088.h>` - Calibration data structures
* :git-linux:`ad9088_cal_dump.c <drivers/iio/adc/apollo/tools/ad9088_cal_dump.c>` - Calibration dump tool


