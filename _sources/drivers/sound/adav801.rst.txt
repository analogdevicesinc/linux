.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/sound/adav801

.. _adav801:

ADAV80X
=======

ADAV80X Sound CODEC Linux Driver.

Supported devices
-----------------

- :adi:`ADAV801`
- :adi:`ADAV803`

ADAV801 and ADAV803 differs in serial interface. ADAV801 uses SPI, while ADAV803
uses I2C.

Reference Circuits
------------------

- :adi:`CN0219`

Evaluation Boards
-----------------

- :adi:`EVAL-ADAV801EBZ`
- :adi:`EVAL-ADAV803EBZ`

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
   * - `git <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/codecs/adav80x.c>`__
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/codecs/adav80x.c>`__
     -
     -

Files
^^^^^

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - driver
     - `sound/soc/codecs/adav80x.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/codecs/adav80x.c>`__
   * - include
     - `sound/soc/codecs/adav80x.h <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/codecs/adav80x.h>`__

Supported Features
------------------

ASoC DAPM widgets
~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Name
     - Description
   * - VINL
     - Left Analog Input
   * - VINR
     - Right Analog Input
   * - VOUTL
     - Left Analog Output
   * - VOUTR
     - Right Analog Output

ALSA controls
~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Name
     - Description
   * - Master Playback Volume
     - DAC Volume
   * - Master Playback Switch
     - Mute/Unmute DAC output
   * - Master Capture Switch
     - ADC Volume
   * - Master Capture Volume
     - Mute/Unmute ADC input
   * - PGA Capture Volume
     - Input PGA gain
   * - ADC High Pass Filter Switch
     - Enable/Disable ADC high-pass filter
   * - Playback De-emphasis Switch
     - Enable/Disable playback de-emphassis
   * - Capture Select
     - Select the serial port capture source. Possible values: ``ADC``,
       ``Playback``, ``Aux Playback``
   * - Aux Capture Select
     - Select the auxiliary serial port capture source. Possible values:
       ``ADC``, ``Playback``, ``Aux Playback``
   * - DAC Select
     - Select the DAC signal source. Possible values: ``ADC``, ``Playback``,
       ``Aux Playback``

Supported DAI formats
~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Name
     - Supported by driver
     - Description
   * - SND_SOC_DAIFMT_I2S
     - yes
     - I2S mode
   * - SND_SOC_DAIFMT_RIGHT_J
     - yes
     - Right Justified mode
   * - SND_SOC_DAIFMT_LEFT_J
     - yes
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
     - yes
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

Currently unsupported features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Automatic level control
- on-chip sample rate converter (Both serial ports have to use the same sample
  rate)
- S/PDIF input and output

ADAV80X evaluation board driver
-------------------------------

There is no dedicated Blackfin STAMP evaluation board for the ADAV80X. During
test and driver development we used the EVAL-ADAV801/ADAV803 board.

It can be easily wired to the Blackfin STAMP SPORT header.

Source Code
~~~~~~~~~~~

Status
^^^^^^

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
     -
   * - :git+linux:`git  <5.12:sound/soc/blackfin/bfin-eval-adav80x.c>`
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/blackfin/bfin-eval-adav80x.c>`__
     -
     -

Files
^^^^^

.. list-table::
   :header-rows: 1

   * - Function
     - File
   * - driver
     - `sound/soc/blackfin/bfin-eval-adav80x.c <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/sound/soc/blackfin/bfin-eval-adav80x.c>`__

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

For ADAV801, enable SPI:

::

   Device Drivers  --->
   [*] SPI support  --->
   ***   SPI Master Controller Drivers ***
   <*>   SPI controller driver for ADI Blackfin5xx

For ADAV803, enable I2C:

::

   Device Drivers  --->
   [*] I2C support  --->
   [*]   I2C Hardware Bus support  --->
   ***     I2C system bus drivers (mostly embedded / system-on-chip) ***
   <*>       Blackfin TWI I2C support
   (100)     Blackfin TWI I2C clock (kHz)

Enable ALSA SoC evaluation board driver:

::

   Device Drivers  --->
   <M> Sound card support  --->
   <M>   Advanced Linux Sound Architecture  --->
   <M>     ALSA for SoC audio support  --->
   <M>       Support for the EVAL-ADAV80X boards on Blackfin eval boards

Hardware configuration
~~~~~~~~~~~~~~~~~~~~~~

Default setting for the 12-way DIP switch: from SW11 to SW0 - 110001110001. Keep
all switch and jumpers as default value defined in the manual, except:

- Playback Port Source: SW11: 0, SW10: 0 (Make it tri-state since we connect
  playback port to Blackfin SPORT)
- ADAV80x MCLK Source: SW0: 0, LK4: A, LK5: ON, LK6: ON (Use the on board cystal
  oscillator as MCLK source).

To test using BF537-STAMP board, connect **play** and **rec** port on the
ADAV80X evaluation board with SPORT0 on STAMP. Connect **CPORT** pins also to
corresponding pins STAMP. Also, please make sure to connect the **GND** of the
two boards together to get better signal.

Driver testing
~~~~~~~~~~~~~~

Load the driver and make sure the sound card is properly instantiated.

::

   root:/> modprobe snd-bf5xx-i2s
   root:/> modprobe snd-soc-bf5xx-i2s
   root:/> modprobe snd-soc-adav80x
   root:/> modprobe snd-soc-bfin-eval-adav80x
   dma rx:3 tx:4, err irq:45, regs:ffc00800
   asoc: ADAV80X <-> bf5xx-i2s mapping ok

::

   root:/> modprobe snd-pcm-oss
   root:/> tone
   TONE: generating sine wave at 1000 Hz...

   root:/> arecord -f cd | aplay
   Recording WAVE 'stdin' : Signed 16 bit Little Endian, Rate 44100 Hz, Stereo
   Playing WAVE 'stdin' : Signed 16 bit Little Endian, Rate 44100 Hz, Stereo
