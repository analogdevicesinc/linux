.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/sound/hdl-axi-i2s

.. _hdl-axi-i2s:

HDL AXI I2S
===========

HDL AXI I2S Linux Driver.

Supported Devices
-----------------

- HDL AXI I2S

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-i2s.c>`__
     - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-i2s.c>`__
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
     - `sound/soc/adi/axi-i2s.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/axi-i2s.c>`__
     -

Example device initialization
-----------------------------

The AXI I2S driver is a platform driver and can currently only be instantiated
via device tree.

Required devicetree properties:

- compatible : Must be ``adi,axi-i2s-1.00.a``
- reg : Must contain I2S registers location and length
- clocks : Clock specifier providing a handle to the controllers clocks. The
  controller expects two clocks, the clock used for the AXI interface and the
  clock used for the sample rate base frequency.
- clock-names : ``axi`` for the clock to the AXI interface, ``i2s`` for the
  sample rate base clock.
- dma : Specifiers for the DMA channels that are used by the core.
- dma-names : ``tx`` for the transmit channel, ``rx`` for the receive channel.

Example:

::

   axi_i2s_0: axi-i2s@77600000 {
       compatible = "adi,axi-i2s-1.00.a";
       reg = <0x77600000 0x1000>;
       clocks = <&fpga_clock &audio_clock>;
       clock-names = "axi", "ref";
       dmas = <&ps7_dma 0 &ps7_dma 1>;
       dma-names = "tx", "rx";
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
     - yes
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
     - no
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

   static struct snd_soc_dai_link zed_adau1761_dai_link = {
       .name = "adau1761",
       .stream_name = "adau1761",
       .codec_dai_name = "adau-hifi",
       .dai_fmt = SND_SOC_DAIFMT_I2S |
               SND_SOC_DAIFMT_NB_NF |
               SND_SOC_DAIFMT_CBS_CFS,
       .ops = &zed_adau1761_ops,
   };

   static struct snd_soc_card zed_adau1761_card = {
       .name = "ZED ADAU161",
       .owner = THIS_MODULE,
       .dai_link = &zed_adau1761_dai_link,
       .num_links = 1,
       .dapm_widgets = zed_adau1761_widgets,
       .num_dapm_widgets = ARRAY_SIZE(zed_adau1761_widgets),
       .dapm_routes = zed_adau1761_routes,
       .num_dapm_routes = ARRAY_SIZE(zed_adau1761_routes),
       .fully_routed = true,
   };

I2S + ADAU1761 ZED board driver
-------------------------------

The HDL I2S SPDIF driver is currently used in conjunction with the ADAU1761
audio codec on the `ZED board <http://zedboard.org/content/overview>`__. For
this platform there exist a ASoC board driver which provides the necessary
information on how both device are interconnected, so that a ALSA sound card can
be instantiated.

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
   * - :git-linux:`git <sound/soc/adi/zed_adau1761.c>`
     - `In progress <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/adi/zed_adau1761.c>`__
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
     - :git-linux:`sound/soc/adi/zed_adau1761.c`
     -

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

Enable ALSA SoC evaluation board driver:

::

   Device Drivers  --->
   <*> Sound card support  --->
   <*>   Advanced Linux Sound Architecture  --->
   <*>     ALSA for SoC audio support  --->
   <*>       Audio support for Analog Devices reference designs
   <*>         ZED board sound support

Example device initialization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ZED board ADAU1761 sound board driver is a platform driver and can currently
only be instantiated via device tree.

Required devicetree properties:

- compatible: Should always be ``adv7511-hdmi-snd``
- audio-codec: Phandle to the ADAU1761 device devicetree entry
- cpu-dai: Phandle to the I2S device devicetree entry

Example:

::

   axi_iic_0: i2c@41600000 {
       compatible = "xlnx,axi-iic-1.01.b", "xlnx,xps-iic-2.00.a";
       interrupt-parent = <&gic>;
       interrupts = <0 57 0x4>;
       reg = <0x41600000 0x10000>;

       #size-cells = <0>;
       #address-cells = <1>;

       adau1761: adau1761@3b {
           compatible = "adi,adau1761";
           reg = <0x3b>;
       };
   };

   axi_i2s: axi-i2s@0x77600000 {
       compatible = "adi,axi-i2s-1.00.a";
       reg = <0x77600000 0x1000>;
       clocks = <&fpga_clock &audio_clock>;
       clock-names = "axi", "ref";
       dmas = <&ps7_dma 0 &ps7_dma 1>;
       dma-names = "tx", "rx";
   };

   zed_adau1761_snd: zed_adau1761_snd {
       compatible = "zed-adau1761-snd";
       audio-codec = <&adau1761>;
       cpu-dai = <&axi_i2s>;
   };

Driver testing
~~~~~~~~~~~~~~

Make sure the sound card is properly instantiated.

::

   root:/> aplay -l
   card 0: ...

To test audio playback you can use the speaker test utility, which allows to
playback several different test patterns. For a extensive description on the
speaker-test utility and the different options it supports please refer to the
`speaker-test man page <http://linux.die.net/man/1/speaker-test>`__.

::

   root:/> speakertest -c 2
   speaker-test 1.0.24.2

   Playback device is default
   Stream parameters are 48000Hz, S32_LE, 2 channels
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

   root:/> arecord -f S32 -r 48000 -c 2 > test.wav
   ...

To playback a wav file you can use the *aplay* utility. You can also create a
audio loop-back by sending the output of *arecord* to *aplay*. This will send
the incoming audio stream back via the outgoing audio stream. ::

   root:/> arecord -f S32 -r 48000 -c 2 | aplay
   ...

For more information on the the *aplay* and *arecord* utilities please refer to
the `aplay and arecord man page <http://linux.die.net/man/1/arecord>`__.

More information
~~~~~~~~~~~~~~~~

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
