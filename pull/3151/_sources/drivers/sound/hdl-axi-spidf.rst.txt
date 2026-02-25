.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/sound/hdl-axi-spidf

.. _hdl-axi-spidf:

HDL AXI SPIDF
=============

HDL AXI SPIDF Linux Driver.

Supported Devices
-----------------

- HDL AXI SPDIF

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-spdif.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-spdif.c>`__
     -
     -

Files
^^^^^

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - `sound/soc/adi/axi-spdif.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-spdif.c>`__
     -

Example device initialization
-----------------------------

The AXI SPDIF driver is a platform driver and can currently only be instantiated
via device tree.

Required devicetree properties:

- compatible: Should always be ``adi,axi-spdif-tx-1.00.a``
- reg: Base address and register area size
- clock-frequency: Clock frequency applied at the spdif_data_clk pin in HZ

Example:

::

   axi_spdif_tx_0: axi-spdif-tx@75c00000 {
       compatible = "adi,axi-spdif-tx-1.00.a";
       reg = <0x75c00000 0x1000>;
       clock-frequency = <12288000>;
   };

DAI configuration
~~~~~~~~~~~~~~~~~

The driver will register one CPU-DAI named after the device itself.

Supported DAI formats
^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Name
     - Supported by driver
     - Description
   * - SND_SOC_DAIFMT_I2S
     - no
     - I2S mode
   * - SND_SOC_DAIFMT_RIGHT_J
     - no
     - Right Justified mode
   * - SND_SOC_DAIFMT_LEFT_J
     - no
     - Left Justified mode
   * - SND_SOC_DAIFMT_DSP_A
     - no
     - data MSB after FRM LRC
   * - SND_SOC_DAIFMT_DSP_B
     - no
     - data MSB during FRM LRC
   * - SND_SOC_DAIFMT_AC97
     - no
     - AC97 mode
   * - SND_SOC_DAIFMT_PDM
     - no
     - Pulse density modulation
   * - SND_SOC_DAIFMT_SPDIF
     - yes
     - SPDIF mode
   * -
     -
     -
   * - SND_SOC_DAIFMT_NB_NF
     - yes
     - Normal bit- and frameclock
   * - SND_SOC_DAIFMT_NB_IF
     - no
     - Normal bitclock, inverted frameclock
   * - SND_SOC_DAIFMT_IB_NF
     - no
     - Inverted frameclock, normal bitclock
   * - SND_SOC_DAIFMT_IB_IF
     - no
     - Inverted bit- and frameclock
   * -
     -
     -
   * - SND_SOC_DAIFMT_CBM_CFM
     - no
     - Codec bit- and frameclock master
   * - SND_SOC_DAIFMT_CBS_CFM
     - no
     - Codec bitclock slave, frameclock master
   * - SND_SOC_DAIFMT_CBM_CFS
     - no
     - Codec bitclock master, frameclock slave
   * - SND_SOC_DAIFMT_CBS_CFS
     - yes
     - Codec bit- and frameclock slave

Example DAI configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code:: c

   static struct snd_soc_dai_link hdmi_dai_link = {
       .name = "HDMI",
       .stream_name = "HDMI",
       .cpu_dai_name = "75c00000.axi-spdif-tx",
       .platform_name = "xilinx_pcm_audio.2",
       .codec_name = "adv7511.0-0039",
       .codec_dai_name = "adv7511",
       .dai_fmt = SND_SOC_DAIFMT_SPDIF |
               SND_SOC_DAIFMT_NB_NF |
               SND_SOC_DAIFMT_CBS_CFS,
   };

   static struct snd_soc_card hdmi_card = {
       .name = "HDMI monitor",
       .owner = THIS_MODULE,
       .dai_link = &hdmi_dai_link,
       .num_links = 1,
   };

ADV7511 HDMI + SPDIF board driver
---------------------------------

The HDL AXI SPDIF driver is currently used in conjunction with the ADV7511 HDMI
transmitter on various FPGA platforms. For these platforms there exist a ASoC
board driver which provides the necessary information on how both device are
interconnected, so that a ALSA sound card can be instantiated.

Source
~~~~~~

Status
^^^^^^

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
     -
   * - :git-linux:`In progress <sound/soc/adi/adv7511_hdmi.c>`
     - `In progress <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/adv7511_hdmi.c>`__
     -
     -

Files
^^^^^

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git-linux:`sound/soc/adi/adv7511_hdmi.c`
     -

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

Enable ALSA SoC evaluation board driver:

::

   Device Drivers  --->
   <*> Sound card support  --->
   <*>   Advanced Linux Sound Architecture  --->
   <*>     ALSA for SoC audio support  --->
   <*>       SoC Audio for Xilinx based boards
   <*>       ADV7511 HDMI transmitter sound support

Example device initialization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ADV7511 HDMI sound board driver is a platform driver and can currently only
be instantiated via device tree.

Required devicetree properties:

- compatible: Should always be ``adv7511-hdmi-snd``
- cpu-dai: Phandle to the SPDIF device devicetree entry
- pcm: Phandle to the PCM device devicetree entry

Example:

::

   axi_dma_0: axidma@40400000 {
       #address-cells = <1>;
       #size-cells = <1>;
       #dma-cells = <1>;
       compatible = "xlnx,axi-dma";
       reg = <0x40400000 0x1000>;
       xlnx,sg-include-stscntrl-strm = <0x0>;
       dma-channel@40400000 {
           compatible = "xlnx,axi-dma-mm2s-channel";
           interrupts = <0 58 0x4>;
           xlnx,datawidth = <0x20>;
           xlnx,include-dre = <0x0>;
       };
   };

   xilinx_pcm_audio: xilinx_pcm_audio {
       compatible = "xilinx-pcm-audio";
       #size-cells = <0>;
       #address-cells = <1>;

       playback: stream@0 {
           reg = <0>;
           dma-request = <&axi_dma_0 0>;
       };
   };

   axi_spdif_tx_0: axi-spdif-tx@0x75c00000 {
       compatible = "adi,axi-spdif-tx-1.00.a";
       reg = <0x75c00000 0x1000>;
       clock-frequency = <12288000>;
   };

   adv7511_hdmi_snd: adv7511_hdmi_snd {
       compatible = "adv7511-hdmi-snd";
       cpu-dai = <&axi_spdif_tx_0>;
       pcm = <&xilinx_pcm_audio>;
   };

Driver testing
~~~~~~~~~~~~~~

Make sure the sound card is properly instantiated.

::

   root:/> aplay -l
   card 0: monitor [HDMI monitor], device 0: HDMI adv7511-0 []
     Subdevices: 1/1
     Subdevice #0: subdevice #0

To test audio playback you can use the *speaker-test* utility, which allows to
playback several different test patterns. For a extensive description on the
*speaker-test* utility and the different options it supports please refer to the
`speaker-test man page <http://linux.die.net/man/1/speaker-test>`__.

::

   root:/> speaker-test -c 2
   speaker-test 1.0.24.2

   Playback device is default
   Stream parameters are 48000Hz, S16_LE, 2 channels
   Using 16 octaves of pink noise
   Rate set to 48000Hz (requested 48000Hz)
   Buffer size range from 512 to 2097152
   Period size range from 256 to 262143
   Requested buffer time 20000 us
   Periods = 4
   was set period_size = 320
   was set buffer_size = 960
    0 - Front Left
    1 - Front Right
   ....

To test audio record you can use the *arecord* utility. *arecord* will record
the incoming audio signal and write it to a wav file.

::

   root:/> arecord -f S16 -r 48000 -c 2 > test.wav
   ...

To playback a wav file you can use the *aplay* utility. You can also create a
audio loop-back by sending the output of *arecord* to *aplay*. This will send
the incoming audio stream back via the outgoing audio stream. ::

   root:/> arecord -f S16 -r 48000 -c 2 | aplay
   ...

For more information on the the *aplay* and *arecord* utilities please refer to
the `aplay and arecord man page <http://linux.die.net/man/1/arecord>`__.

More information
~~~~~~~~~~~~~~~~

- :dokuwiki+deprecated:`AD-FMCOMMS1-EBZ Reference Design <resources/fpga/xilinx/fmc/ad-fmcomms1-ebz>`
- :external+documentation:ref:`linux-kernel zynq`

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
