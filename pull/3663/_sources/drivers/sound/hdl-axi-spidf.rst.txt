.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/sound/hdl-axi-spidf


.. _hdl-axi-spidf:

HDL AXI SPIDF
=============

HDL AXI SPIDF Linux Driver.

Supported IP Cores
------------------

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
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - `sound/soc/adi/axi-spdif.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-spdif.c>`__
     -

Adding Linux driver support
---------------------------

Devicetree configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Example device initialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The AXI SPDIF driver is a platform driver and can currently only be instantiated
via device tree.

Required devicetree properties:

-  compatible: Should always be ``adi,axi-spdif-tx-1.00.a``

-  reg: Base address and register area size

-  clocks: AXI bus clock and reference clock.

-  clock-names: ``axi`` and ``ref``, respectively.

-  dmas: Transmit DMA channel.

-  dma-names: ``tx``.

Example:

::

   axi_spdif_tx_0: axi-spdif-tx@75c00000 {
       compatible = "adi,axi-spdif-tx-1.00.a";
       reg = <0x75c00000 0x1000>;
       clocks = <&clkc 15>, <&audio_clock>;
       clock-names = "axi", "ref";
       dmas = <&dmac_s 0>;
       dma-names = "tx";
   };

Board integration
~~~~~~~~~~~~~~~~~

ADV7511 HDMI + SPDIF sound card
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The HDL AXI SPDIF core is used in conjunction with the ADV7511 HDMI transmitter
on various FPGA platforms. The sound card is described with the generic
``simple-audio-card`` binding, linking the AXI SPDIF CPU DAI with the ADV7511
HDMI audio codec DAI.

Kernel configuration
""""""""""""""""""""

::

   Device Drivers  --->
       <*> Sound card support  --->
           <*>   Advanced Linux Sound Architecture  --->
               <*>     ALSA for SoC audio support  --->
                   <*>   Audio support for Analog Devices reference designs
                   <*>     AXI-SPDIF support
                   [--snip--]
                   <*>   ASoC Simple sound card support
       Graphics support  --->
           Display Interface Bridges  --->
               <*> ADV7511 encoder
               [*]   ADV7511 HDMI Audio driver

Example device initialization
"""""""""""""""""""""""""""""

Required properties:

- The AXI SPDIF node must define ``#sound-dai-cells = <0>``.
- The ADV7511 node must define ``#sound-dai-cells = <1>``; DAI index ``1``
  selects its SPDIF audio input.

Example (from :git+linux:`main:arch/arm/boot/dts/xilinx/zynq-zed-adv7511.dtsi`):

.. code-block:: dts

   axi_spdif_tx_0: axi-spdif-tx@75c00000 {
       compatible = "adi,axi-spdif-tx-1.00.a";
       reg = <0x75c00000 0x1000>;
       dmas = <&dmac_s 0>;
       dma-names = "tx";
       clocks = <&clkc 15>, <&audio_clock>;
       clock-names = "axi", "ref";

       #sound-dai-cells = <0>;
   };

   adv7511_hdmi_snd {
       compatible = "simple-audio-card";
       simple-audio-card,name = "HDMI monitor";
       simple-audio-card,widgets =
           "Speaker", "Speaker";
       simple-audio-card,routing =
           "Speaker", "TX";

       simple-audio-card,dai-link@0 {
           cpu {
               sound-dai = <&axi_spdif_tx_0>;
               frame-master;
               bitclock-master;
           };
           codec {
               sound-dai = <&adv7511 1>;
           };
       };
   };

Driver testing
""""""""""""""

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

This AXI SPDIF driver provides stereo S16_LE playback only. The following
arecord and record-to-playback loopback examples require a separate capture
device and do not test capture through this transmitter.

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
""""""""""""""""

- :dokuwiki+deprecated:`AD-FMCOMMS1-EBZ Reference Design <resources/fpga/xilinx/fmc/ad-fmcomms1-ebz>`
- :external+system-level:ref:`linux-kernel zynq`

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help

Driver testing
--------------

DAI formats and clocks
~~~~~~~~~~~~~~~~~~~~~~

DAI configuration
^^^^^^^^^^^^^^^^^

The driver will register one CPU-DAI named after the device itself.

Supported DAI formats
"""""""""""""""""""""

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
