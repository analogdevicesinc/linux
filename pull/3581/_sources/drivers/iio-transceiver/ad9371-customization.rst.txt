.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/ad9371-customization

.. _ad9371-customization:

AD9371
======

AD9371/AD9375 Device Driver Customization.

.. note::

   There are configuration options that must be set properly. Some others allow
   you to set defaults, but can be changed anytime later using the driver API.
   But most of these options don’t need to be changed at all.

   If unsure please see the manual or don’t change!

The Linux platform allows you to examine and determine optimal settings for your
target application: See here:
:dokuwiki:`AD9371/AD9375 Advanced Plugin </resources/tools-software/linux-software/ad9371_advanced_plugin>`

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-rx-framer-bank-id
     - 0
     - JESD204B Configuration Bank ID extension to Device ID. Range is 0..15
   * - adi,jesd204-rx-framer-device-id
     - 0
     - JESD204B Configuration Device ID link identification number. Range is
       0..255
   * - adi,jesd204-rx-framer-lane0-id
     - 0
     - JESD204B Configuration starting Lane ID. If more than one lane is used,
       each lane will increment from the Lane0 ID. Range is 0..31
   * - adi,jesd204-rx-framer-m
     - 4
     - Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive chain
       (I and Q)
   * - adi,jesd204-rx-framer-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K must be modulo 4.
       Where, F=2*M/numberOfLanes
   * - adi,jesd204-rx-framer-scramble
     - 1
     - Scrambling off if framerScramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-rx-framer-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF, 1 = use external SYSREF
   * - adi,jesd204-rx-framer-serializer-lanes-enabled
     - 3
     - Serializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1
       enabled, etc
   * - adi,jesd204-rx-framer-serializer-lane-crossbar
     - 0xe4
     - Lane crossbar to map framer lane outputs to physical lanes
   * - adi,jesd204-rx-framer-serializer-amplitude
     - 22
     - Serializer amplitude setting. Default = 22. Range is 0..31
   * - adi,jesd204-rx-framer-pre-emphasis
     - 4
     - Serializer pre-emphasis setting. Default = 4 Range is 0..7
   * - adi,jesd204-rx-framer-invert-lane-polarity
     - 0
     - Lane inversion select. Default = 0. Where, bit[0] = 0 will invert lane
       [0], bit[1] = 0 will invert lane 1, etc.
   * - adi,jesd204-rx-framer-lmfc-offset
     - 0
     - LMFC offset value for deterministic latency setting. Range is 0..31
   * - adi,jesd204-rx-framer-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, 0 = not set
   * - adi,jesd204-rx-framer-enable-auto-chan-xbar
     - 0
     - Flag for determining if auto channel select for the xbar should be set.
       Where, if > 0 = set, ‘0’ = not set
   * - adi,jesd204-rx-framer-obs-rx-syncb-select
     - 0
     - Selects SYNCb input source. Where, 0 = use RXSYNCB for this framer, 1 =
       use OBSRX_SYNCB for this framer
   * - adi,jesd204-rx-framer-rx-syncb-mode
     - 0
     - Flag for determining if CMOS mode for RX Sync signal is used. Where, if >
       0 = CMOS, ‘0’ = LVDS
   * - adi,jesd204-rx-framer-over-sample
     - 0
     - Selects framer bit repeat or oversampling mode for lane rate matching.
       Where, 0 = bitRepeat mode (changes effective lanerate), 1 = overSample
       (maintains same lane rate between ObsRx framer and Rx framer and
       oversamples the ADC samples)

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-obs-framer-bank-id
     - 0
     - JESD204B Configuration Bank ID extension to Device ID. Range is 0..15
   * - adi,jesd204-obs-framer-device-id
     - 0
     - JESD204B Configuration Device ID link identification number. Range is
       0..255
   * - adi,jesd204-obs-framer-lane0-id
     - 0
     - JESD204B Configuration starting Lane ID. If more than one lane is used,
       each lane will increment from the Lane0 ID. Range is 0..31
   * - adi,jesd204-obs-framer-m
     - 2
     - Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive chain
       (I and Q)
   * - adi,jesd204-obs-framer-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K must be modulo 4.
       Where, F=2*M/numberOfLanes
   * - adi,jesd204-obs-framer-scramble
     - 1
     - Scrambling off if framerScramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-obs-framer-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF, 1 = use external SYSREF
   * - adi,jesd204-obs-framer-serializer-lanes-enabled
     - 0xc
     - Serializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1
       enabled, etc
   * - adi,jesd204-obs-framer-serializer-lane-crossbar
     - 0xe4
     - Lane crossbar to map framer lane outputs to physical lanes
   * - adi,jesd204-obs-framer-serializer-amplitude
     - 22
     - Serializer amplitude setting. Default = 22. Range is 0..31
   * - adi,jesd204-obs-framer-pre-emphasis
     - 4
     - Serializer pre-emphasis setting. Default = 4 Range is 0..7
   * - adi,jesd204-obs-framer-invert-lane-polarity
     - 0
     - Lane inversion select. Default = 0. Where, bit[0] = 0 will invert lane
       [0], bit[1] = 0 will invert lane 1, etc.
   * - adi,jesd204-obs-framer-lmfc-offset
     - 0
     - LMFC offset value for deterministic latency setting. Range is 0..31
   * - adi,jesd204-obs-framer-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, 0 = not set
   * - adi,jesd204-obs-framer-enable-auto-chan-xbar
     - 0
     - Flag for determining if auto channel select for the xbar should be set.
       Where, if > 0 = set, ‘0’ = not set
   * - adi,jesd204-obs-framer-obs-rx-syncb-select
     - 1
     - Selects SYNCb input source. Where, 0 = use RXSYNCB for this framer, 1 =
       use OBSRX_SYNCB for this framer
   * - adi,jesd204-obs-framer-rx-syncb-mode
     - 0
     - Flag for determining if CMOS mode for RX Sync signal is used. Where, if >
       0 = CMOS, ‘0’ = LVDS
   * - adi,jesd204-obs-framer-over-sample
     - 1
     - Selects framer bit repeat or oversampling mode for lane rate matching.
       Where, 0 = bitRepeat mode (changes effective lanerate), 1 = overSample
       (maintains same lane rate between ObsRx framer and Rx framer and
       oversamples the ADC samples)

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-deframer-bank-id
     - 0
     - Extension to Device ID. Range is 0..15
   * - adi,jesd204-deframer-device-id
     - 0
     - Link identification number. Range is 0..255
   * - adi,jesd204-deframer-lane0-id
     - 0
     - Lane0 ID. Range is 0..31
   * - adi,jesd204-deframer-m
     - 4
     - Number of DACs (0, 2, or 4) - 2 DACs per transmit chain (I and Q)
   * - adi,jesd204-deframer-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K = modulo 4. Where,
       F=2*M/numberOfLanes
   * - adi,jesd204-deframer-scramble
     - 1
     - Scrambling off if scramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-deframer-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF, 1 = external SYSREF
   * - adi,jesd204-deframer-deserializer-lanes-enabled
     - 0x0F
     - Deserializer lane select bit field. Where, [0] = Lane0 enabled, [1] =
       Lane1 enabled, etc
   * - adi,jesd204-deframer-deserializer-lane-crossbar
     - 0xE4
     - Lane crossbar to map physical lanes to deframer lane inputs [1:0] =
       Deframer Input 0 Lane section, [3:2] = Deframer Input 1 lane select, etc
   * - adi,jesd204-deframer-eq-setting
     - 1
     - Equalizer setting. Applied to all deserializer lanes. Range is 0..4
   * - adi,jesd204-deframer-invert-lane-polarity
     - 0
     - PN inversion per each lane. bit[0] = 1 Invert PN of Lane 0, bit[1] =
       Invert PN of Lane 1, etc
   * - adi,jesd204-deframer-lmfc-offset
     - 0
     - LMFC offset value to adjust deterministic latency. Range is 0..31
   * - adi,jesd204-deframer-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, ‘0’ = not set
   * - adi,jesd204-deframer-enable-auto-chan-xbar
     - 0
     - Flag for determining if auto channel select for the xbar should be set.
       Where, if > 0 = set, ‘0’ = not set
   * - adi,jesd204-deframer-tx-syncb-mode
     - 0
     - Flag for determining if CMOS mode for TX Sync signal is used. Where, if >
       0 = CMOS, ‘0’ = LVDS

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-gain-mode
     - 0
     - Current Rx gain control mode setting
   * - adi,rx1-gain-index
     - 255
     - Rx1 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,rx2-gain-index
     - 255
     - Rx2 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,rx1-max-gain-index
     - 255
     - Max gain index for the currently loaded Rx1 Gain table
   * - adi,rx1-min-gain-index
     - 195
     - Min gain index for the currently loaded Rx1 Gain table
   * - adi,rx2-max-gain-index
     - 255
     - Max gain index for the currently loaded Rx2 Gain table
   * - adi,rx2-min-gain-index
     - 195
     - Min gain index for the currently loaded Rx2 Gain table

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,orx-gain-mode
     - 0
     - Current ORx gain control mode setting
   * - adi,orx1-gain-index
     - 255
     - ORx1 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,orx2-gain-index
     - 255
     - ORx2 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,orx-max-gain-index
     - 255
     - Max gain index for the currently loaded ORx Gain table
   * - adi,orx-min-gain-index
     - 237
     - Min gain index for the currently loaded ORx Gain table

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,sniffer-gain-mode
     - 0
     - Current Sniffer gain control mode setting
   * - adi,sniffer-gain-index
     - 255
     - Current Sniffer gain index. Can be used differently for Manual Gain
       control/AGC
   * - adi,sniffer-max-gain-index
     - 255
     - Max gain index for the currently loaded Sniffer Gain table
   * - adi,sniffer-min-gain-index
     - 203
     - Min gain index for the currently loaded Sniffer Gain table

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-peak-agc-apd-high-thresh
     - 31
     - APD high threshold. Must be greater than apdLowThresh. Min =
       apdLowThresh, Max = 0x3F. 6-bit field.
   * - adi,rx-peak-agc-apd-low-thresh
     - 22
     - APD low threshold. Must be less than apdHighThresh. Min = 0, Max =
       apdHighThresh. 6-bit field.
   * - adi,rx-peak-agc-hb2-high-thresh
     - 181
     - HB2 high threshold. Must be greater than hb2LowThresh. Min =
       hb2LowThresh, Max = 0xFF. 8-bit field.
   * - adi,rx-peak-agc-hb2-low-thresh
     - 128
     - HB2 low threshold. Must be less than hb2HighThresh. Min = 0, Max =
       hb2HighThresh. 8-bit field.
   * - adi,rx-peak-agc-hb2-very-low-thresh
     - 64
     - HB2 very low threshold. Must be less than hb2LowThresh. Min = 0, Max =
       hb2LowThresh. 8-bit field.
   * - adi,rx-peak-agc-apd-high-thresh-exceeded-cnt
     - 6
     - APD high threshold exceeded counter. Sets number of peaks to detect above
       apdHighThresh to cause gain decrement according to apdHighGainStepAttack.
       8-bit field.
   * - adi,rx-peak-agc-apd-low-thresh-exceeded-cnt
     - 4
     - APD low threshold exceeded counter. Sets number of peaks to detect below
       apdLowThresh to cause gain increment according to apdLowGainStepRecovery.
       8-bit field.
   * - adi,rx-peak-agc-hb2-high-thresh-exceeded-cnt
     - 6
     - HB2 high threshold exceeded counter. Sets number of overloads to detect
       above hb2HighThresh to cause gain decrement according to
       hb2HighGainStepAttack. 8-bit field.
   * - adi,rx-peak-agc-hb2-low-thresh-exceeded-cnt
     - 4
     - HB2 low threshold exceeded counter. Sets number of peaks to detect below
       hb2LowThresh to cause gain increment according to hb2LowGainStepRecovery.
       8-bit field.
   * - adi,rx-peak-agc-hb2-very-low-thresh-exceeded-cnt
     - 4
     - HB2 very low threshold exceeded counter. Sets number of peaks to detect
       below hb2VeryLowThresh to cause gain increment according to
       hb2VeryLowGainStepRecovery. 8-bit field.
   * - adi,rx-peak-agc-apd-high-gain-step-attack
     - 4
     - Number of gain indices to decrement gain when apdHighThreshExceededCnt is
       exceeded. 5-bit field.
   * - adi,rx-peak-agc-apd-low-gain-step-recovery
     - 2
     - Number of gain indices to increment gain when apdLowThreshExceededCnt is
       exceeded. 5-bit field
   * - adi,rx-peak-agc-hb2-high-gain-step-attack
     - 4
     - Number of gain indices to decrement gain when hb2HighThreshExceededCnt is
       exceeded. 5-bit field
   * - adi,rx-peak-agc-hb2-low-gain-step-recovery
     - 2
     - Number of gain indices to increment gain when hb2LowThreshExceededCnt is
       exceeded. 5-bit field
   * - adi,rx-peak-agc-hb2-very-low-gain-step-recovery
     - 4
     - Number of gain indices to increment gain when hb2VeryLowThreshExceededCnt
       is exceeded. 5-bit field
   * - adi,rx-peak-agc-apd-fast-attack
     - 1
     - [1] Enables APD fast attack mode - gain decrements immediately when
       apdHighThreshExceededCnt is exceeded. [0] disables APD fast attack mode -
       gain decrements at the expiry of agcGainUpdateCounter. 1-bit field.
   * - adi,rx-peak-agc-hb2-fast-attack
     - 1
     - [1] Enables HB2 fast attack mode - gain decrements immediately when
       hb2HighThreshExceededCnt is exceeded. [0] disables HB2 fast attack mode -
       gain decrements at the expiry of agcGainUpdateCounter. 1-bit field.
   * - adi,rx-peak-agc-hb2-overload-detect-enable
     - 1
     - [1] Enables the HB2 overload detector. [0] Disables the HB2 overload
       detector. 1-bit field.
   * - adi,rx-peak-agc-hb2-overload-duration-cnt
     - 1
     - Sets the samples size window of the HB2 overload detector. If
       hb2OverloadThreshCnt number of overloads are detected, the
       hb2xxxThreshExceededCnt increments. 3-bit field. [001]=1, [001]=4,
       [010]=8, [011]=12, [100]=16, [101]=24, [110]=32, [111]=INVALID
   * - adi,rx-peak-agc-hb2-overload-thresh-cnt
     - 1
     - Sets the number of individual overloads necessary within
       hb2OverloadDurationCnt samples to increment the hb2xxxThreshExceededCnt

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,obs-peak-agc-apd-high-thresh
     - 31
     - APD high threshold. Must be greater than apdLowThresh. Min =
       apdLowThresh, Max = 0x3F. 6-bit field.
   * - adi,obs-peak-agc-apd-low-thresh
     - 22
     - APD low threshold. Must be less than apdHighThresh. Min = 0, Max =
       apdHighThresh. 6-bit field.
   * - adi,obs-peak-agc-hb2-high-thresh
     - 181
     - HB2 high threshold. Must be greater than hb2LowThresh. Min =
       hb2LowThresh, Max = 0xFF. 8-bit field.
   * - adi,obs-peak-agc-hb2-low-thresh
     - 128
     - HB2 low threshold. Must be less than hb2HighThresh. Min = 0, Max =
       hb2HighThresh. 8-bit field.
   * - adi,obs-peak-agc-hb2-very-low-thresh
     - 64
     - HB2 very low threshold. Must be less than hb2LowThresh. Min = 0, Max =
       hb2LowThresh. 8-bit field.
   * - adi,obs-peak-agc-apd-high-thresh-exceeded-cnt
     - 6
     - APD high threshold exceeded counter. Sets number of peaks to detect above
       apdHighThresh to cause gain decrement according to apdHighGainStepAttack.
       8-bit field.
   * - adi,obs-peak-agc-apd-low-thresh-exceeded-cnt
     - 4
     - APD low threshold exceeded counter. Sets number of peaks to detect below
       apdLowThresh to cause gain increment according to apdLowGainStepRecovery.
       8-bit field.
   * - adi,obs-peak-agc-hb2-high-thresh-exceeded-cnt
     - 6
     - HB2 high threshold exceeded counter. Sets number of overloads to detect
       above hb2HighThresh to cause gain decrement according to
       hb2HighGainStepAttack. 8-bit field.
   * - adi,obs-peak-agc-hb2-low-thresh-exceeded-cnt
     - 4
     - HB2 low threshold exceeded counter. Sets number of peaks to detect below
       hb2LowThresh to cause gain increment according to hb2LowGainStepRecovery.
       8-bit field.
   * - adi,obs-peak-agc-hb2-very-low-thresh-exceeded-cnt
     - 4
     - HB2 very low threshold exceeded counter. Sets number of peaks to detect
       below hb2VeryLowThresh to cause gain increment according to
       hb2VeryLowGainStepRecovery. 8-bit field.
   * - adi,obs-peak-agc-apd-high-gain-step-attack
     - 4
     - Number of gain indices to decrement gain when apdHighThreshExceededCnt is
       exceeded. 5-bit field.
   * - adi,obs-peak-agc-apd-low-gain-step-recovery
     - 2
     - Number of gain indices to increment gain when apdLowThreshExceededCnt is
       exceeded. 5-bit field
   * - adi,obs-peak-agc-hb2-high-gain-step-attack
     - 4
     - Number of gain indices to decrement gain when hb2HighThreshExceededCnt is
       exceeded. 5-bit field
   * - adi,obs-peak-agc-hb2-low-gain-step-recovery
     - 2
     - Number of gain indices to increment gain when hb2LowThreshExceededCnt is
       exceeded. 5-bit field
   * - adi,obs-peak-agc-hb2-very-low-gain-step-recovery
     - 4
     - Number of gain indices to increment gain when hb2VeryLowThreshExceededCnt
       is exceeded. 5-bit field
   * - adi,obs-peak-agc-apd-fast-attack
     - 1
     - [1] Enables APD fast attack mode - gain decrements immediately when
       apdHighThreshExceededCnt is exceeded. [0] disables APD fast attack mode -
       gain decrements at the expiry of agcGainUpdateCounter. 1-bit field.
   * - adi,obs-peak-agc-hb2-fast-attack
     - 1
     - [1] Enables HB2 fast attack mode - gain decrements immediately when
       hb2HighThreshExceededCnt is exceeded. [0] disables HB2 fast attack mode -
       gain decrements at the expiry of agcGainUpdateCounter. 1-bit field.
   * - adi,obs-peak-agc-hb2-overload-detect-enable
     - 1
     - [1] Enables the HB2 overload detector. [0] Disables the HB2 overload
       detector. 1-bit field.
   * - adi,obs-peak-agc-hb2-overload-duration-cnt
     - 1
     - Sets the samples size window of the HB2 overload detector. If
       hb2OverloadThreshCnt number of overloads are detected, the
       hb2xxxThreshExceededCnt increments. 3-bit field. [001]=1, [001]=4,
       [010]=8, [011]=12, [100]=16, [101]=24, [110]=32, [111]=INVALID
   * - adi,obs-peak-agc-hb2-overload-thresh-cnt
     - 1
     - Sets the number of individual overloads necessary within
       hb2OverloadDurationCnt samples to increment the hb2xxxThreshExceededCnt

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-pwr-agc-pmd-upper-high-thresh
     - 1
     - Power measurement upper band, high threshold . This value is a positive
       offset to the pmdUpperLowThresh threshold. 4-bit field
   * - adi,rx-pwr-agc-pmd-upper-low-thresh
     - 3
     - Power measurement upper band, low threshold. This value sets the
       threshold in (negative) -dBFS. Byte value must be less than
       pmdLowerHighThresh. 7-bit field
   * - adi,rx-pwr-agc-pmd-lower-high-thresh
     - 12
     - Power measurement lower band, high threshold. This value sets the
       threshold in (negative) -dBFS. Byte value must be greater than
       pmdUpperLowThresh. 7-bit field
   * - adi,rx-pwr-agc-pmd-lower-low-thresh
     - 4
     - Power measurement lower band, low threshold. This value is a negative
       offset to the pmdLowerHighThresh threshold. 4-bit field
   * - adi,rx-pwr-agc-pmd-upper-high-gain-step-attack
     - 4
     - Number of gain indices to decrement gain if pmdUpperHighThresh is
       exceeded by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,rx-pwr-agc-pmd-upper-low-gain-step-attack
     - 2
     - Number of gain indices to decrement gain if pmdUpperLowThresh is exceeded
       by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,rx-pwr-agc-pmd-lower-high-gain-step-recovery
     - 2
     - Number of gain indices to increment gain if pmdLowerHighThresh is not
       exceeded by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,rx-pwr-agc-pmd-lower-low-gain-step-recovery
     - 4
     - Number of gain indices to increment gain if pmdLowerLowThresh is not
       exceeded by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,rx-pwr-agc-pmd-meas-duration
     - 8
     - Number of samples to measure power on. The number of samples
       corresponding to the 4-bit word is 8*2
   * - adi,rx-pwr-agc-pmd-meas-config
     - 2
     - Power measurement configuration. 2-bit field. [00]=PMD disabled, [01]=PMD
       Enabled at HB2 output, [10]=Enabled at RFIR output (recommended),
       [11]=PMD Enabled at BBDC2

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,obs-pwr-agc-pmd-upper-high-thresh
     - 1
     - Power measurement upper band, high threshold . This value is a positive
       offset to the pmdUpperLowThresh threshold. 4-bit field
   * - adi,obs-pwr-agc-pmd-upper-low-thresh
     - 3
     - Power measurement upper band, low threshold. This value sets the
       threshold in (negative) -dBFS. Byte value must be less than
       pmdLowerHighThresh. 7-bit field
   * - adi,obs-pwr-agc-pmd-lower-high-thresh
     - 12
     - Power measurement lower band, high threshold. This value sets the
       threshold in (negative) -dBFS. Byte value must be greater than
       pmdUpperLowThresh. 7-bit field
   * - adi,obs-pwr-agc-pmd-lower-low-thresh
     - 4
     - Power measurement lower band, low threshold. This value is a negative
       offset to the pmdLowerHighThresh threshold. 4-bit field
   * - adi,obs-pwr-agc-pmd-upper-high-gain-step-attack
     - 4
     - Number of gain indices to decrement gain if pmdUpperHighThresh is
       exceeded by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,obs-pwr-agc-pmd-upper-low-gain-step-attack
     - 2
     - Number of gain indices to decrement gain if pmdUpperLowThresh is exceeded
       by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,obs-pwr-agc-pmd-lower-high-gain-step-recovery
     - 2
     - Number of gain indices to increment gain if pmdLowerHighThresh is not
       exceeded by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,obs-pwr-agc-pmd-lower-low-gain-step-recovery
     - 4
     - Number of gain indices to increment gain if pmdLowerLowThresh is not
       exceeded by the end of the agcGainUpdateCounter. 5-bit field
   * - adi,obs-pwr-agc-pmd-meas-duration
     - 8
     - Number of samples to measure power on. The number of samples
       corresponding to the 4-bit word is 8*2
   * - adi,obs-pwr-agc-pmd-meas-config
     - 2
     - Power measurement configuration. 2-bit field. [00]=PMD disabled, [01]=PMD
       Enabled at HB2 output, [10]=Enabled at RFIR output (recommended),
       [11]=PMD Enabled at BBDC2

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-agc-conf-agc-rx1-max-gain-index
     - 255
     - Maximum Rx1 gain index allowed in AGC mode. Must be greater than
       agcRx1MinGainIndex and valid gain index. 8-bit field
   * - adi,rx-agc-conf-agc-rx1-min-gain-index
     - 195
     - Minimum Rx1 gain index allowed in AGC mode. Must be less than
       agcRx1MinGainIndex and valid gain index. 8-bit field
   * - adi,rx-agc-conf-agc-rx2-max-gain-index
     - 255
     - Maximum Rx2 gain index allowed in AGC mode. Must be greater than
       agcRx2MinGainIndex and valid gain index. 8-bit field
   * - adi,rx-agc-conf-agc-rx2-min-gain-index
     - 195
     - Minimum Rx2 gain index allowed in AGC mode. Must be less than
       agcRx2MinGainIndex and valid gain index. 8-bit field
   * - adi,rx-agc-conf-agc-obs-rx-max-gain-index
     - 255
     - Maximum ObsRx gain index allowed in AGC mode. Must be greater than
       agcObsRxMinGainIndex and valid gain index. 8-bit field
   * - adi,rx-agc-conf-agc-obs-rx-min-gain-index
     - 203
     - Minimum ObsRx gain index allowed in AGC mode. Must be less than
       agcObsRxMaxGainIndex and valid gain index. 8-bit field
   * - adi,rx-agc-conf-agc-obs-rx-select
     - 1
     - Sniffer or ObsRx AGC channel select. [1] = SnRx
   * - adi,rx-agc-conf-agc-peak-threshold-mode
     - 1
     - [1] = Peak Threshold Mode, power based gain changes are disabled. [0] =
       Peak and overload detectors are ignored for gain changes
   * - adi,rx-agc-conf-agc-low-ths-prevent-gain-increase
     - 1
     - [1] PMD based gain increments are ignored if apd/hb2LowThreshExceedCnt is
       high [0] apdLowThreshExceededCnt and hb2LowThreshExceededCnt are Don’t
       cares`` to the AGC gain recovery``
   * - adi,rx-agc-conf-agc-gain-update-counter
     - 30720
     - Number of samples for the AGC gain update counter. Counter operates on
       the IQ data rate. 22-bit field. Min = 0x000001, Max = 0x3FFFFF
   * - adi,rx-agc-conf-agc-slow-loop-settling-delay
     - 3
     - Number of IQ data rate clock cycles to wait after a gain change before
       peak/power measurements resume. 7-bit field
   * - adi,rx-agc-conf-agc-peak-wait-time
     - 2
     - Number of IQ data rate clock cycles to wait to enable peak/overload
       detectors after AGC is enabled. 5-bit field. Min = 0x02. Max = 0x1F
   * - adi,rx-agc-conf-agc-reset-on-rx-enable
     - 0
     - [1] = Performs a reset of the AGC slow loop state machine when Rx is
       disabled. [0] = AGC slow loop state machine maintains its state when Rx
       is disabled.
   * - adi,rx-agc-conf-agc-enable-sync-pulse-for-gain-counter
     - 0
     - [1] = Allows sync of agcGainUpdateCounter to the time-slot boundary. GPIO
       setup required. [0] = agcGainUpdateCounter functions as normal

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,obs-agc-conf-agc-rx1-max-gain-index
     - 255
     - Maximum Rx1 gain index allowed in AGC mode. Must be greater than
       agcRx1MinGainIndex and valid gain index. 8-bit field
   * - adi,obs-agc-conf-agc-rx1-min-gain-index
     - 195
     - Minimum Rx1 gain index allowed in AGC mode. Must be less than
       agcRx1MinGainIndex and valid gain index. 8-bit field
   * - adi,obs-agc-conf-agc-rx2-max-gain-index
     - 255
     - Maximum Rx2 gain index allowed in AGC mode. Must be greater than
       agcRx2MinGainIndex and valid gain index. 8-bit field
   * - adi,obs-agc-conf-agc-rx2-min-gain-index
     - 195
     - Minimum Rx2 gain index allowed in AGC mode. Must be less than
       agcRx2MinGainIndex and valid gain index. 8-bit field
   * - adi,obs-agc-conf-agc-obs-rx-max-gain-index
     - 255
     - Maximum ObsRx gain index allowed in AGC mode. Must be greater than
       agcObsRxMinGainIndex and valid gain index. 8-bit field
   * - adi,obs-agc-conf-agc-obs-rx-min-gain-index
     - 203
     - Minimum ObsRx gain index allowed in AGC mode. Must be less than
       agcObsRxMaxGainIndex and valid gain index. 8-bit field
   * - adi,obs-agc-conf-agc-obs-rx-select
     - 1
     - Sniffer or ObsRx AGC channel select. [1] = SnRx
   * - adi,obs-agc-conf-agc-peak-threshold-mode
     - 1
     - [1] = Peak Threshold Mode, power based gain changes are disabled. [0] =
       Peak and overload detectors are ignored for gain changes
   * - adi,obs-agc-conf-agc-low-ths-prevent-gain-increase
     - 1
     - [1] PMD based gain increments are ignored if apd/hb2LowThreshExceedCnt is
       high [0] apdLowThreshExceededCnt and hb2LowThreshExceededCnt are Don’t
       cares`` to the AGC gain recovery``
   * - adi,obs-agc-conf-agc-gain-update-counter
     - 30720
     - Number of samples for the AGC gain update counter. Counter operates on
       the IQ data rate. 22-bit field. Min = 0x000001, Max = 0x3FFFFF
   * - adi,obs-agc-conf-agc-slow-loop-settling-delay
     - 3
     - Number of IQ data rate clock cycles to wait after a gain change before
       peak/power measurements resume. 7-bit field
   * - adi,obs-agc-conf-agc-peak-wait-time
     - 2
     - Number of IQ data rate clock cycles to wait to enable peak/overload
       detectors after AGC is enabled. 5-bit field. Min = 0x02. Max = 0x1F
   * - adi,obs-agc-conf-agc-reset-on-rx-enable
     - 0
     - [1] = Performs a reset of the AGC slow loop state machine when Rx is
       disabled. [0] = AGC slow loop state machine maintains its state when Rx
       is disabled.
   * - adi,obs-agc-conf-agc-enable-sync-pulse-for-gain-counter
     - 0
     - [1] = Allows sync of agcGainUpdateCounter to the time-slot boundary. GPIO
       setup required. [0] = agcGainUpdateCounter functions as normal

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-profile-adc-div
     - 1
     - The divider used to generate the ADC clock (Valid: 1,2)
   * - adi,rx-profile-rx-fir-decimation
     - 2
     - Rx FIR decimation (1,2,4)
   * - adi,rx-profile-rx-dec5-decimation
     - 5
     - Decimation of Dec5 or Dec4 filter (5,4)
   * - adi,rx-profile-en-high-rej-dec5
     - 1
     - If set, and DEC5 filter used, will use a higher rejection DEC5 FIR
       filter. Where, 1 = enabled, 0 = disabled
   * - adi,rx-profile-rhb1-decimation
     - 1
     - RX Halfband1 (HB1) decimation. Can be either 1 or 2
   * - adi,rx-profile-iq-rate_khz
     - 122880
     - Rx IQ data rate in kHz
   * - adi,rx-profile-rf-bandwidth_hz
     - 100000000
     - Rx RF passband bandwidth for the profile
   * - adi,rx-profile-rx-bbf-3db-corner_khz
     - 100000
     - Rx BBF (TIA) 3dB corner in kHz

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,obs-profile-adc-div
     - 1
     - The divider used to generate the ADC clock (Valid: 1,2)
   * - adi,obs-profile-rx-fir-decimation
     - 1
     - Rx FIR decimation (1,2,4)
   * - adi,obs-profile-rx-dec5-decimation
     - 5
     - Decimation of Dec5 or Dec4 filter (5,4)
   * - adi,obs-profile-en-high-rej-dec5
     - 0
     - If set, and DEC5 filter used, will use a higher rejection DEC5 FIR
       filter. Where, 1 = enabled, 0 = disabled
   * - adi,obs-profile-rhb1-decimation
     - 1
     - RX Halfband1 (HB1) decimation. Can be either 1 or 2
   * - adi,obs-profile-iq-rate_khz
     - 245760
     - Rx IQ data rate in kHz
   * - adi,obs-profile-rf-bandwidth_hz
     - 200000000
     - Rx RF passband bandwidth for the profile
   * - adi,obs-profile-rx-bbf-3db-corner_khz
     - 100000
     - Rx BBF (TIA) 3dB corner in kHz

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,sniffer-profile-adc-div
     - 1
     - The divider used to generate the ADC clock (Valid: 1,2)
   * - adi,sniffer-profile-rx-fir-decimation
     - 4
     - Rx FIR decimation (1,2,4)
   * - adi,sniffer-profile-rx-dec5-decimation
     - 5
     - Decimation of Dec5 or Dec4 filter (5,4)
   * - adi,sniffer-profile-en-high-rej-dec5
     - 0
     - If set, and DEC5 filter used, will use a higher rejection DEC5 FIR
       filter. Where, 1 = enabled, 0 = disabled
   * - adi,sniffer-profile-rhb1-decimation
     - 2
     - RX Halfband1 (HB1) decimation. Can be either 1 or 2
   * - adi,sniffer-profile-iq-rate_khz
     - 30720
     - Rx IQ data rate in kHz
   * - adi,sniffer-profile-rf-bandwidth_hz
     - 20000000
     - Rx RF passband bandwidth for the profile
   * - adi,sniffer-profile-rx-bbf-3db-corner_khz
     - 100000
     - Rx BBF (TIA) 3dB corner in kHz

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx-profile-dac-div
     - 1
     - The divider used to generate the DAC clock (ENUM Values DACDIV_2 = 0,
       DACDIV_2p5 = 1, DACDIV_4 = 2)
   * - adi,tx-profile-tx-fir-interpolation
     - 1
     - The TX digital FIR filter interpolation (1,2,4)
   * - adi,tx-profile-thb1-interpolation
     - 2
     - Tx Halfband1 (HB1) filter interpolation (1,2)
   * - adi,tx-profile-thb2-interpolation
     - 1
     - Tx Halfband2 (HB2) filter interpolation (1,2)
   * - adi,tx-profile-tx-input-hb-interpolation
     - 1
     - Interpolation of half band filter before the programmable FIR (valid
       1,2,4)
   * - adi,tx-profile-iq-rate_khz
     - 245760
     - Tx IQ data rate in kHz
   * - adi,tx-profile-primary-sig-bandwidth_hz
     - 75000000
     - Tx primary signal BW
   * - adi,tx-profile-rf-bandwidth_hz
     - 200000000
     - Tx RF passband bandwidth for the profile
   * - adi,tx-profile-tx-dac-3db-corner_khz
     - 189477
     - DAC filter 3dB corner in kHz
   * - adi,tx-profile-tx-bbf-3db-corner_khz
     - 100000
     - Tx BBF 3dB corner in kHz

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,clocks-device-clock_khz
     - 122880
     - CLKPLL and device reference clock frequency in kHz
   * - adi,clocks-clk-pll-vco-freq_khz
     - 9830400
     - CLKPLL VCO frequency in kHz
   * - adi,clocks-clk-pll-vco-div
     - 2
     - CLKPLL VCO divider (VCODIV1 = 0, VCODIV1p5 = 1, VCODIV2 = 2, VCODIV3 = 3)
   * - adi,clocks-clk-pll-hs-div
     - 4
     - CLKPLL high speed clock divider

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx-settings-tx-channels-enable
     - 3
     - The desired Tx channels to enable during initialization
   * - adi,tx-settings-tx-pll-use-external-lo
     - 0
     - Internal LO=0, external LO*2 if =1
   * - adi,tx-settings-tx-pll-lo-frequency_hz
     - 2500000000
     - Tx PLL LO frequency (internal or external LO)
   * - adi,tx-settings-tx-atten-step-size
     - 0
     - Tx Attenuation step size
   * - adi,tx-settings-tx1-atten_mdb
     - 10000
     - Initial and current Tx1 Attenuation
   * - adi,tx-settings-tx2-atten_mdb
     - 10000
     - Initial and current Tx2 Attenuation

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-settings-rx-channels-enable
     - 3
     - The desired Rx Channels to enable during initialization
   * - adi,rx-settings-rx-pll-use-external-lo
     - 0
     - Internal LO = 0, external LO*2 = 1
   * - adi,rx-settings-rx-pll-lo-frequency_hz
     - 2500000000
     - Rx PLL LO Frequency (internal or external LO)
   * - adi,rx-settings-real-if-data
     - 0
     - Flag to choose if complex baseband or real IF data are selected for Rx
       and ObsRx paths. Where, if > 0 = real IF data, ‘0’ = zero IF (IQ) data

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,obs-settings-obs-rx-channels-enable
     - 31
     - The desired ObsRx channels to configure/calibrate during initialization
   * - adi,obs-settings-obs-rx-lo-source
     - 0
     - The sniffer/ORx mixers can use the TX_PLL or SNIFFER_PLL
   * - adi,obs-settings-sniffer-pll-lo-frequency_hz
     - 2600000000
     - SnRx PLL LO frequency in Hz
   * - adi,obs-settings-real-if-data
     - 0
     - Flag to choose if complex baseband or real IF data are selected for Rx
       and ObsRx paths. Where if > 0 = real IF data, ‘0’ = complex data
   * - adi,obs-settings-default-obs-rx-channel
     - 0
     - Default ObsRx channel to enter when radioOn called

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,arm-gpio-use-rx2-enable-pin
     - 0
     - 0= RX1_ENABLE controls RX1 and RX2, 1 = separate RX1_ENABLE/RX2_ENABLE
       pins
   * - adi,arm-gpio-use-tx2-enable-pin
     - 0
     - 0= TX1_ENABLE controls TX1 and TX2, 1 = separate TX1_ENABLE/TX2_ENABLE
       pins
   * - adi,arm-gpio-tx-rx-pin-mode
     - 0
     - 0= ARM command mode, 1 = Pin mode to power up Tx/Rx chains
   * - adi,arm-gpio-orx-pin-mode
     - 0
     - 0= ARM command mode, 1 = Pin mode to power up ObsRx receiver
   * - adi,arm-gpio-orx-trigger-pin
     - 0
     - Select desired GPIO pin (valid 4-15)
   * - adi,arm-gpio-orx-mode2-pin
     - 0
     - Select desired GPIO pin (valid 0-18)
   * - adi,arm-gpio-orx-mode1-pin
     - 0
     - Select desired GPIO pin (valid 0-18)
   * - adi,arm-gpio-orx-mode0-pin
     - 0
     - Select desired GPIO pin (valid 0-18)
   * - adi,arm-gpio-rx1-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-rx2-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-tx1-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-tx2-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-orx1-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-orx2-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-srx-enable-ack
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable
   * - adi,arm-gpio-tx-obs-select
     - 0
     - Select desired GPIO pin (0-15), [4] = Output Enable When 2Tx are used
       with only 1 ORx input, this GPIO tells the BBIC which Tx channel is
       active for calibrations, so BBIC can route correct RF Tx path into the
       single ORx input
   * - adi,arm-gpio-enable-mask
     - 0
     -

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,gpio-3v3-oe-mask
     - 0
     - Pin direction: bit per 3.3v GPIO, 0=Input, 1=Output from Mykonos device
   * - adi,gpio-3v3-src-ctrl3_0
     - 3
     - Mode for GPIO3v3[3:0] pins
   * - adi,gpio-3v3-src-ctrl7_4
     - 3
     - Mode for GPIO3v3[7:4] pins
   * - adi,gpio-3v3-src-ctrl11_8
     - 3
     - Mode for GPIO3v3[11:8] pins

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,gpio-oe-mask
     - 0
     - Output Enable per low voltage GPIO pin (1=output, 0=input)
   * - adi,gpio-src-ctrl3_0
     - 0
     - Mode for low voltage GPIO[3:0] pins
   * - adi,gpio-src-ctrl7_4
     - 0
     - Mode for low voltage GPIO[7:4] pins
   * - adi,gpio-src-ctrl11_8
     - 0
     - Mode for low voltage GPIO[11:8] pins
   * - adi,gpio-src-ctrl15_12
     - 0
     - Mode for low voltage GPIO[15:12] pins
   * - adi,gpio-src-ctrl18_16
     - 0
     - Mode for low voltage GPIO[18:16] pins

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,aux-dac-enable-mask
     - 0
     - Aux DAC enable. One bit per Aux DAC. Where bit[0] = Aux DAC 0, bit[1] =
       Aux DAC 1, etc
   * - adi,aux-dac-value
     - 0
     - Aux DAC value
   * - adi,aux-dac-slope
     - 0
     - Aux DAC slope
   * - adi,aux-dac-vref
     - 0
     - Aux DAC voltage reference value

AD9375 Only
-----------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,dpd-damping
     - 5
     - 1/2^(damping + 8) fraction of previous model ‘forgotten’ per adaptation
       (default: 5 = ‘1/8192’ , valid 0 to 15), 0 = infinite damping
   * - adi,dpd-num-weights
     - 1
     - number of weights to use for int8_cpx weights weights member of this
       structure (default = 1)
   * - adi,dpd-model-version
     - 2
     - DPD model version: one of four different generalized polynomial models: 0
       = same as R0 silicon, 1-3 are new and the best one depends on the PA
       (default: 2)
   * - adi,dpd-high-power-model-update
     - 1
     - 1 = Update saved model whenever peak Tx digital RMS is within 1dB of
       historical peak Tx RMS
   * - adi,dpd-model-prior-weight
     - 20
     - Determines how much weight the loaded prior model has on DPD modeling
       (Valid 0 - 32, default 20)
   * - adi,dpd-robust-modeling
     - 0
     - Default off = 0, 1=enables automatic outlier removal during DPD modeling
   * - adi,dpd-samples
     - 512
     - number of samples to capture (default: 512, valid 64 - 32768)
   * - adi,dpd-outlier-threshold
     - 4096
     - threshold for sample in AM-AM plot outside of 1:1 line to be thrown out.
       (default: 50% = 8192/2, valid 8192 to 1)
   * - adi,dpd-additional-delay-offset
     - 0
     - 16th of an ORx sample (16=1sample), (default 0, valid -64 to 64)
   * - adi,dpd-path-delay-pn-seq-level
     - 255
     - Default 255 (-30dBFs=(20Log10(value/8192)), (valid range 1 to 8191)
   * - adi,dpd-weights0-real
     - 64
     - DPD model error weighting (real/imag valid from -128 to 127)
   * - adi,dpd-weights0-imag
     - 0
     - DPD model error weighting (real/imag valid from -128 to 127)
   * - adi,dpd-weights1-real
     - 0
     - DPD model error weighting (real/imag valid from -128 to 127)
   * - adi,dpd-weights1-imag
     - 0
     - DPD model error weighting (real/imag valid from -128 to 127)
   * - adi,dpd-weights2-real
     - 0
     - DPD model error weighting (real/imag valid from -128 to 127)
   * - adi,dpd-weights2-imag
     - 0
     - DPD model error weighting (real/imag valid from -128 to 127)

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,clgc-tx1-desired-gain
     - -2000
     - (value = 100 \* dB (valid range -32768 to 32767) - total gain and
       attenuation from Mykonos Tx1 output to ORx1 input in (dB \* 100)
   * - adi,clgc-tx2-desired-gain
     - -2000
     - (value = 100 \* dB (valid range -32768 to 32767) - total gain and
       attenuation from Mykonos Tx2 output to ORx2 input in (dB \* 100)
   * - adi,clgc-tx1-atten-limit
     - 0
     - (valid range 0 - 40dB), no default, depends on PA, Protects PA by making
       sure Tx1Atten is not reduced below the limit
   * - adi,clgc-tx2-atten-limit
     - 0
     - (valid range 0 - 40dB), no default, depends on PA, Protects PA by making
       sure Tx2Atten is not reduced below the limit
   * - adi,clgc-tx1-control-ratio
     - 75
     - valid range 1-100, default 45
   * - adi,clgc-tx2-control-ratio
     - 75
     - valid range 1-100, default 45
   * - adi,clgc-allow-tx1-atten-updates
     - 0
     - 0= allow CLGC to run, but Tx1Atten will not be updated. User can still
       read back power measurements. 1=CLGC runs, and Tx1Atten automatically
       updated
   * - adi,clgc-allow-tx2-atten-updates
     - 0
     - 0= allow CLGC to run, but Tx2Atten will not be updated. User can still
       read back power measurements. 1=CLGC runs, and Tx2Atten automatically
       updated
   * - adi,clgc-additional-delay-offset
     - 0
     - 16th of an ORx sample (16=1sample), (default 0, valid -64 to 64)
   * - adi,clgc-path-delay-pn-seq-level
     - 255
     - Default 255 (-30dBFs=(20Log10(value/8192)), (valid range 1 to 8191)
   * - adi,clgc-tx1-rel-threshold
     - 600
     - Threshold for Tx1 in order to stop tracking, value = 100 \* dB, default
       6db then value = 600
   * - adi,clgc-tx2-rel-threshold
     - 600
     - Threshold for Tx2 in order to stop tracking, value = 100 \* dB, default
       6db then value = 600
   * - adi,clgc-tx1-rel-threshold-en
     - 0
     - Threshold feature enable for Tx1, 0 = disable, 1 = enable, default = 0
   * - adi,clgc-tx2-rel-threshold-en
     - 0
     - Threshold feature enable for Tx2, 0 = disable, 1 = enable, default = 0

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,vswr-additional-delay-offset
     - 0
     - 16th of an ORx sample (16=1sample), (default 0, valid -64 to 64)
   * - adi,vswr-path-delay-pn-seq-level
     - 255
     - Default 255 (-30dBFs=(20Log10(value/8192)), (valid range 1 to 8191)
   * - adi,vswr-tx1-vswr-switch-gpio3p3-pin
     - 0
     - 3p3V GPIO pin to use to control VSWR switch for Tx1 (valid 0-11) (output
       from Mykonos)
   * - adi,vswr-tx2-vswr-switch-gpio3p3-pin
     - 1
     - 3p3V GPIO pin to use to control VSWR switch for Tx2 (valid 0-11) (output
       from Mykonos)
   * - adi,vswr-tx1-vswr-switch-polarity
     - 0
     - 3p3v GPIO pin polarity for forward path of Tx1, opposite used for
       reflection path (1 = high level, 0 = low level)
   * - adi,vswr-tx2-vswr-switch-polarity
     - 0
     - 3p3v GPIO pin polarity for forward path of Tx2, opposite used for
       reflection path (1 = high level, 0 = low level)
   * - adi,vswr-tx1-vswr-switch-delay_us
     - 50
     - Delay for Tx1 after flipping the VSWR switch until measurement is made.
       In us resolution with a range from 0 to 255us
   * - adi,vswr-tx2-vswr-switch-delay_us
     - 50
     - Delay for Tx2 after flipping the VSWR switch until measurement is made.
       In us resolution with a range from 0 to 255us
