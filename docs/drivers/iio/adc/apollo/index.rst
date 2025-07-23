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

* :git-linux:`ad9088.c <drivers/iio/adc/apollo/ad9088.c>`
* :git-linux:`ad9088.h <drivers/iio/adc/apollo/ad9088.h>`
* :git-linux:`ad9088_dt.c <drivers/iio/adc/apollo/ad9088_dt.c>`
* :git-linux:`ad9088_fft.c <drivers/iio/adc/apollo/ad9088_fft.c>`

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

Profile
~~~~~~~

* ``adi,device-profile-fw-name``: Set the profile firmware to use.

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

* ``adi,trigger-sync-en``: Enable trigger synchronization, needed to reset FSCR.
* ``adi,trig-req-gpio``: Set the GPIO that trigger Enable trigger synchronization.
  If not provided, the SPI trigger will be used for trigger synchronization.

If the option is enabled and no trigger is provided, the synchronization will
time out.

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
       --w------- 1 root root 4096 Feb 11 15:17 cfir_config
       -r--r--r-- 1 root root 4096 Feb 11 15:17 dev
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_temp0_input
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_temp0_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_frequency_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_en
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_loopback
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_hb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_main_tb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_nyquist_zone
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_i_test_mode
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_frequency_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_en
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_loopback
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_hb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_main_tb1_6db_digital_gain_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_nyquist_zone
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage0_q_test_mode
       ...
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_adc_frequency
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_loopback_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_main_nco_frequency_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_nyquist_zone_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_sampling_frequency
       -r--r--r-- 1 root root 4096 Feb 11 15:17 in_voltage_test_mode_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_ctrl
       -r--r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_error
       -r--r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_paused
       --w------- 1 root root 4096 Feb 11 15:17 jesd204_fsm_resume
       -r--r--r-- 1 root root 4096 Feb 11 15:17 jesd204_fsm_state
       -r--r--r-- 1 root root 4096 Feb 11 15:17 loopback1_blend_available
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 loopback1_blend_side_a
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 loopback1_blend_side_b
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_bg_tacking_cal_run
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_cal_run
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_dt0_measurement
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_dt1_measurement
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_dt1_restore
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_fg_tacking_cal_run
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_init
       -r--r--r-- 1 root root 4096 Feb 11 15:17 mcs_init_cal_status
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 mcs_tracking_init
       -r--r--r-- 1 root root 4096 Feb 11 15:17 mcs_tracking_status
       -r--r--r-- 1 root root 4096 Feb 11 15:17 name
       lrwxrwxrwx 1 root root    0 Feb 11 15:17 of_node -> ../../../../../firmware/devicetree/base/fpga-axi@0/axi-ad9084-rx-hpc@a4a10000
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_gain_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_channel_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_invsinc_en
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_i_main_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_cfir_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_cfir_profile_sel
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_gain_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_channel_nco_test_tone_scale
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_invsinc_en
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_label
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_phase
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_test_tone_en
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage0_q_main_nco_test_tone_scale
       ...
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_channel_nco_frequency_available
       -r--r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_dac_frequency
       -rw-r--r-- 1 root root 4096 Feb 11 15:17 out_voltage_main_nco_frequency_available
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

- mode: <imode> <qmode>

  - Sets the mode for the PFIR filter. The <imode> and <qmode> values should be
    one of the predefined filter modes: ``disabled``, ``real_n4``, ``real_n2``,
    ``undef``, ``matrix``, ``undef``, ``complex_half``, ``real_n``.

- gain: <ix> <iy> <qx> <qy>

  - Sets the gain values for the PFIR filter. The <ix>, <iy>, <qx>,
    and <qy> values should be integers representing the gain in dB.

- scalar_gain: <ix> <iy> <qx> <qy>

  - Sets the scalar gain values for the PFIR filter. The <ix>, <iy>,
    <qx>, and <qy> values should be integers representing the scalar
    gain.

- dest: <terminal> <pfilt_sel> <bank_sel>

  - Sets the destination for the PFIR filter. The <terminal> value should be
    either ``rx`` or ``tx``. The <pfilt_sel> value should be one of the
    predefined filter selects: ``pfilt_a0``, ``pfilt_a1``, ``pfilt_b0``,
    ``pfilt_b1``, ``pfilt_all``, ``pfilt_mask``. The <bank_sel> value should be
    one of the predefined filter banks: ``bank_0``, ``bank_1``, ``bank_2``,
    ``bank_3``, ``bank_all``, ``bank_mask``.

- hc_delay: <delay>

  - Sets the high cut delay value for the PFIR filter. The <delay>
    value should be an unsigned 8-bit integer.

- mode_switch_en: <value>

  - Sets the mode switch enable value for the PFIR filter. The <value>
    should be either 0 or 1.

- mode_switch_add_en: <value>

  - Sets the mode switch add enable value for the PFIR filter. The
    <value> should be either 0 or 1.

- real_data_mode_en: <value>

  - Sets the real data mode enable value for the PFIR filter. The
    <value> should be either 0 or 1.

- quad_mode_en: <value>

  - Sets the quad mode enable value for the PFIR filter. The <value>
    should be either 0 or 1.

- selection_mode: <mode>

  - Sets the profile selection mode for the PFIR filter. The <mode> value
    should be one of the predefined profile selection modes: ``direct_regmap``,
    ``direct_gpio``, ``direct_gpio1``, ``trig_regmap``, ``trig_gpio``,
    ``trig_gpio1``.

- <sval>

  - Sets the coefficient values for the PFIR filter. The <sval> value
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

The user configures the NCO FFH frequency by writing the index (0 to 31) to
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

Debug system
^^^^^^^^^^^^

An additional debug provided through debugfs:

.. shell::

   $cd /sys/kernel/debug/iio/iio\:device8 ; pwd
    /sys/kernel/debug/iio/iio:device8

Read API version:

.. shell::
   :no-path:

   $cat api_version
    0.4.40

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

Change protocol to use, HSCI (1) or SPI (0):

.. shell::
   :no-path:

   $cat hsci_enable
    0
   $echo 1 > hsci_enable
   $cat hsci_enable
    1

