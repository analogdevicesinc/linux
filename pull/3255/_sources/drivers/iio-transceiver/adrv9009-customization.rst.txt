.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/adrv9009-customization

.. _adrv9009-customization:

ADRV9009
========

ADRV9009/ADRV9008 Device Driver Customization.

.. note::

   There are configuration options that must be set properly. Some others allow
   you to set defaults, but can be changed anytime later using the driver API.
   But most of these options don’t need to be changed at all.

   If unsure please see the manual or don’t change!

The Linux platform allows you to examine and determine optimal settings for your
target application: See here:
:dokuwiki:`ADRV9009/ADRV9008 Advanced Plugin </resources/tools-software/linux-software/adrv9009_advanced_plugin>`

AGC Settings
------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rxagc-peak-agc-under-range-mid-interval
     - 2
     - 2nd update interval for multiple time constant AGC mode. Calculated as
       (agcUnderRangeMidInterval+1)*agcUnderRangeLowInterval_ns. Valid range is
       0 to 63
   * - adi,rxagc-peak-agc-under-range-high-interval
     - 4
     - 3rd update interval for multiple time constant AGC mode. Calculated as
       (agcUnderRangeHighInterval+1)*2nd update interval. Valid range is 0 to 63
   * - adi,rxagc-peak-apd-high-thresh
     - 39
     - AGC APD high threshold. Valid range is 0 to 63
   * - adi,rxagc-peak-apd-low-gain-mode-high-thresh
     - 36
     - AGC APD high threshold in low gain mode. Valid range is 0 to 63.
       Recommended to be 3dB above apdHighThresh
   * - adi,rxagc-peak-apd-low-thresh
     - 23
     - AGC APD low threshold. Valid range is 0 to 63. Recommended to be 3dB
       below apdHighThresh
   * - adi,rxagc-peak-apd-low-gain-mode-low-thresh
     - 19
     - AGC APD low threshold in low gain mode. Valid range is 0 to 63.
       Recommended to be 3dB above apdLowThresh
   * - adi,rxagc-peak-apd-upper-thresh-peak-exceeded-cnt
     - 6
     - AGC APD peak detect upper threshold count. Valid range is 0 to 255
   * - adi,rxagc-peak-apd-lower-thresh-peak-exceeded-cnt
     - 3
     - AGC APD peak detect lower threshold count. Valid range is 0 to 255
   * - adi,rxagc-peak-apd-gain-step-attack
     - 4
     - AGC APD peak detect attack gain step. Valid range is 0 to 31
   * - adi,rxagc-peak-apd-gain-step-recovery
     - 2
     - AGC APD gain index step size for recovery. Valid range is 0 to 31
   * - adi,rxagc-peak-enable-hb2-overload
     - 1
     - Enable or disables the HB2 overload detector.
   * - adi,rxagc-peak-hb2-overload-duration-cnt
     - 1
     - Sets the window of clock cycles (at the HB2 output rate) to meet the
       overload count. (0 = 2 cycles, 1 = 4 cycles, 2 = 8 cycles, 3 = 12 cycles,
       4 = 16 cycles, 5 = 24 cycles, 6 = 32 cycles)
   * - adi,rxagc-peak-hb2-overload-thresh-cnt
     - 4
     - Sets the number of actual overloads required to trigger the overload
       signal. Valid range from 1 to 15
   * - adi,rxagc-peak-hb2-high-thresh
     - 181
     - AGC HB2 output high threshold. Valid range from 0 to 255
   * - adi,rxagc-peak-hb2-under-range-low-thresh
     - 45
     - AGC HB2 output low threshold. Valid range from 0 to 255
   * - adi,rxagc-peak-hb2-under-range-mid-thresh
     - 90
     - AGC HB2 output low threshold for 2nd interval for multiple time constant
       AGC mode. Valid range from 0 to 255
   * - adi,rxagc-peak-hb2-under-range-high-thresh
     - 128
     - AGC HB2 output low threshold for 3rd interval for multiple time constant
       AGC mode. Valid range from 0 to 255
   * - adi,rxagc-peak-hb2-upper-thresh-peak-exceeded-cnt
     - 6
     - AGC HB2 output upper threshold count. Valid range from 0 to 255
   * - adi,rxagc-peak-hb2-lower-thresh-peak-exceeded-cnt
     - 3
     - AGC HB2 output lower threshold count. Valid range from 0 to 255
   * - adi,rxagc-peak-hb2-gain-step-high-recovery
     - 2
     - AGC HB2 gain index step size. Valid range from 0 to 31
   * - adi,rxagc-peak-hb2-gain-step-low-recovery
     - 4
     - AGC HB2 gain index step size, when the HB2 Low Overrange interval 2
       triggers. Valid range from 0 to 31
   * - adi,rxagc-peak-hb2-gain-step-mid-recovery
     - 8
     - AGC HB2 gain index step size, when the HB2 Low Overrange interval 3
       triggers. Valid range from 0 to 31
   * - adi,rxagc-peak-hb2-gain-step-attack
     - 4
     - AGC HB2 output attack gain step. Valid range from 0 to 31
   * - adi,rxagc-peak-hb2-overload-power-mode
     - 1
     - When this bit is set, the dynamic range of the power measurement
       increases from -40dB to ~-60dB (that is, all signal levels from 0dBFS to
       -60dBFS are accurately detected
   * - adi,rxagc-peak-hb2-ovrg-sel
     - 0
     - To be used in fast recovery mode. Clearing this bit enables the decimated
       data overload detection functionality
   * - adi,rxagc-peak-hb2-thresh-config
     - 3
     - Not User Modifiable Initialized to 0x03

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rxagc-power-power-enable-measurement
     - 0
     - Enable the Rx power measurement block. (0/1)
   * - adi,rxagc-power-power-use-rfir-out
     - 1
     - Use output of Rx PFIR for power measurement. (0/1)
   * - adi,rxagc-power-power-use-bbdc2
     - 0
     - Use output of DC offset block for power measurement. (0/1)
   * - adi,rxagc-power-under-range-high-power-thresh
     - 9
     - AGC power measurement detect lower 0 threshold. Valid Range from 0 to
       127.
   * - adi,rxagc-power-under-range-low-power-thresh
     - 2
     - AGC power measurement detect lower 1 threshold. Valid offset from 0 to 31
   * - adi,rxagc-power-under-range-high-power-gain-step-recovery
     - 4
     - AGC power measurement detect lower 0 recovery gain step. Valid range from
       0 to 31
   * - adi,rxagc-power-under-range-low-power-gain-step-recovery
     - 4
     - AGC power measurement detect lower 1 recovery gain step. Valid range from
       0 to 31
   * - adi,rxagc-power-power-measurement-duration
     - 5
     - Average power measurement duration = 8*2^powerMeasurementDuration. Valid
       range from 0 to 31
   * - adi,rxagc-power-rx1-tdd-power-meas-duration
     - 5
     - Measurement duration to detect power for specific slice of the gain
       update counter.
   * - adi,rxagc-power-rx1-tdd-power-meas-delay
     - 1
     - Measurement delay to detect power for specific slice of the gain update
       counter.
   * - adi,rxagc-power-rx2-tdd-power-meas-duration
     - 5
     - Measurement duration to detect power for specific slice of the gain
       update counter.
   * - adi,rxagc-power-rx2-tdd-power-meas-delay
     - 1
     - Measurement delay to detect power for specific slice of the gain update
       counter.
   * - adi,rxagc-power-upper0-power-thresh
     - 2
     - AGC upper 0 (overRangeHighPowerThreshold) threshold for power
       measurement. Valid Range from 0 to 127.
   * - adi,rxagc-power-upper1-power-thresh
     - 0
     - AGC upper 1 (overRangeLowPowerThreshold) threshold for power measurement.
       Valid offset from 0 to 15
   * - adi,rxagc-power-power-log-shift
     - 0
     - Enable Increase in dynamic range of the power measurement from 40dB to
       ~60dB. Provides higher accuracy.

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rxagc-agc-peak-wait-time
     - 4
     - AGC peak wait time. Valid range is from 0 to 31
   * - adi,rxagc-agc-rx1-max-gain-index
     - 255
     - AGC Rx1 max gain index. Valid range is from 0 to 255
   * - adi,rxagc-agc-rx1-min-gain-index
     - 195
     - AGC Rx1 min gain index. Valid range is from 0 to 255
   * - adi,rxagc-agc-rx2-max-gain-index
     - 255
     - AGC Rx2 max gain index. Valid range is from 0 to 255
   * - adi,rxagc-agc-rx2-min-gain-index
     - 195
     - AGC Rx2 min gain index. Valid range is from 0 to 255
   * - adi,rxagc-agc-gain-update-counter_us
     - 250
     - AGC gain update time in micro seconds
   * - adi,rxagc-agc-rx1-attack-delay
     - 10
     - On entering Rx, the Rx1 AGC is kept inactive for a period =
       agcRx1AttackDelay*1us
   * - adi,rxagc-agc-rx2-attack-delay
     - 10
     - On entering Rx, the Rx2 AGC is kept inactive for a period =
       agcRx2AttackDelay*1us
   * - adi,rxagc-agc-slow-loop-settling-delay
     - 16
     - On any gain change, the AGC waits for the time (range 0 to 127) specified
       in AGC clock cycles to allow gain transients to flow through the Rx path
       before starting any measurements.
   * - adi,rxagc-agc-low-thresh-prevent-gain
     - 0
     - Prevent gain index from incrementing if peak thresholds are being
       exceeded
   * - adi,rxagc-agc-change-gain-if-thresh-high
     - 1
     - Enable immediate gain change if high threshold counter is exceeded. Bit 0
       enables ULB high threshold, Bit 1 enables HB2 high threshold
   * - adi,rxagc-agc-peak-thresh-gain-control-mode
     - 1
     - Enable gain change based only on the signal peak threshold over-ranges.
       Power based AGC changes are disabled in this mode.
   * - adi,rxagc-agc-reset-on-rxon
     - 0
     - Reset the AGC slow loop state machine to max gain when the Rx Enable is
       taken low
   * - adi,rxagc-agc-enable-sync-pulse-for-gain-counter
     - 0
     - Enable the AGC gain update counter to be sync’ed to a time-slot boundary.
   * - adi,rxagc-agc-enable-ip3-optimization-thresh
     - 0
     - (API disables feature, this member is ignored) Enable the two-threshold
       AGC loop mode.To improve IIP3. Enable=1, Disable=0 (set to 0 for Talise
       B1 and C0 silicon - this feature is not supported)
   * - adi,rxagc-ip3-over-range-thresh
     - 31
     - (API disables feature, this member is ignored) Overload threshold that
       triggers for a lower peak signal level. Recommended to be set to -17dBFS
   * - adi,rxagc-ip3-over-range-thresh-index
     - 246
     - (API disables feature, this member is ignored) Gain index to jump to if
       current gain causes IP3 overload. Recommended to be set to 246. Valid
       range 0 to 255
   * - adi,rxagc-ip3-peak-exceeded-cnt
     - 4
     - (API disables feature, this member is ignored) Configures the number of
       times the ADC IP3 Overrange threshold is triggered within one gain update
       interval before a gain change is mandated by the gain control loop.
   * - adi,rxagc-agc-enable-fast-recovery-loop
     - 0
     - Enable multiple time constants in AGC loop for fast attack and fast
       recovery.

Gain Control Settings
---------------------

RX
~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-gain-control-gain-mode
     - 0
     - Current Rx gain control mode setting
   * - adi,rx-gain-control-rx1-gain-index
     - 255
     - Rx1 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,rx-gain-control-rx2-gain-index
     - 255
     - Rx2 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,rx-gain-control-rx1-max-gain-index
     - 255
     - Max gain index for the currently loaded Rx1 Gain table
   * - adi,rx-gain-control-rx1-min-gain-index
     - 195
     - Min gain index for the currently loaded Rx1 Gain table
   * - adi,rx-gain-control-rx2-max-gain-index
     - 255
     - Max gain index for the currently loaded Rx2 Gain table
   * - adi,rx-gain-control-rx2-min-gain-index
     - 195
     - Min gain index for the currently loaded Rx2 Gain table

ORX
~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,orx-gain-control-gain-mode
     - 0
     - Current Rx gain control mode setting
   * - adi,orx-gain-control-orx1-gain-index
     - 255
     - ORx1 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,orx-gain-control-orx2-gain-index
     - 255
     - ORx2 Gain Index, can be used in different ways for manual and AGC gain
       control
   * - adi,orx-gain-control-orx1-max-gain-index
     - 255
     - Max gain index for the currently loaded ORx1 Gain table
   * - adi,orx-gain-control-orx1-min-gain-index
     - 195
     - Min gain index for the currently loaded ORx1 Gain table
   * - adi,orx-gain-control-orx2-max-gain-index
     - 255
     - Max gain index for the currently loaded ORx2 Gain table
   * - adi,orx-gain-control-orx2-min-gain-index
     - 195
     - Min gain index for the currently loaded ORx2 Gain table

Auxillary DAC Settings
----------------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,aux-dac-enables
     - 0
     - Aux DAC enable bit for each DAC, where the first ten bits correspond to
       the 10-bit DACs, and the next consecutive two bits enable the 12-bit DACs
   * - adi,aux-dac-vref0
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution0
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values0
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref1
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution1
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values1
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref2
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution2
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values2
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref3
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution3
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values3
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref4
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution4
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values4
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref5
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution5
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values5
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref6
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution6
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values6
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref7
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution7
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values7
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref8
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution8
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values8
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-vref9
     - 3
     - Aux DAC voltage reference value for each of the 10-bit DACs
   * - adi,aux-dac-resolution9
     - 0
     - Aux DAC slope (resolution of voltage change per AuxDAC code) - only
       applies to 10bit DACs (0-9)
   * - adi,aux-dac-values9
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-values10
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values
   * - adi,aux-dac-values11
     - 0
     - Aux DAC values for each 10-bit DAC correspond to the first 10 array
       elements, the next consecutive array elements correspond to the two
       12-bit DAC values

JESD204B Settings
-----------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-ser-amplitude
     - 15
     - Serializer amplitude setting. Default = 15. Range is 0..15
   * - adi,jesd204-ser-pre-emphasis
     - 1
     - Serializer pre-emphasis setting. Default = 1 Range is 0..4
   * - adi,jesd204-ser-invert-lane-polarity
     - 0
     - Serializer Lane PN inversion select. Default = 0. Where, bit[0] = 1 will
       invert lane [0], bit[1] = 1 will invert lane 1, etc.
   * - adi,jesd204-des-invert-lane-polarity
     - 0
     - Deserializer Lane PN inversion select. bit[0] = 1 Invert PN of Lane 0,
       bit[1] = Invert PN of Lane 1, etc
   * - adi,jesd204-des-eq-setting
     - 1
     - Deserializer Equalizer setting. Applied to all deserializer lanes. Range
       is 0..2 (default 2, 0 = max boost)
   * - adi,jesd204-sysref-lvds-mode
     - 1
     - 1 - enable LVDS Input pad with 100ohm internal termination, 0 - enable
       CMOS input pad
   * - adi,jesd204-sysref-lvds-pn-invert
     - 0
     - 0 - sysref LVDS PN is not inverted, 1 - sysref LVDS PN is inverted

FRAMER A
~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-framer-a-bank-id
     - 1
     - JESD204B Configuration Bank ID extension to Device ID. Range is 0..15
   * - adi,jesd204-framer-a-device-id
     - 0
     - JESD204B Configuration Device ID link identification number. Range is
       0..255
   * - adi,jesd204-framer-a-lane0-id
     - 0
     - JESD204B Configuration starting Lane ID. If more than one lane is used,
       each lane will increment from the Lane0 ID. Range is 0..31
   * - adi,jesd204-framer-a-m
     - 4
     - Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive chain
       (I and Q)
   * - adi,jesd204-framer-a-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K must be modulo 4.
       Where, F=2*M/numberOfLanes
   * - adi,jesd204-framer-a-f
     - 4
     - Number of bytes(octets) per frame (Valid 1, 2, 4, 8)
   * - adi,jesd204-framer-a-np
     - 16
     - converter sample resolution (12, 16, 24)
   * - adi,jesd204-framer-a-scramble
     - 1
     - Scrambling off if framerScramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-framer-a-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF(not currently valid), 1 =
       use external SYSREF
   * - adi,jesd204-framer-a-serializer-lanes-enabled
     - 0x03
     - Serializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1
       enabled, etc
   * - adi,jesd204-framer-a-serializer-lane-crossbar
     - 0xE4
     - Lane crossbar to map framer lane outputs to physical lanes
   * - adi,jesd204-framer-a-lmfc-offset
     - 31
     - LMFC offset value for deterministic latency setting. Range is 0..31
   * - adi,jesd204-framer-a-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, 0 = not set
   * - adi,jesd204-framer-a-syncb-in-select
     - 0
     - Selects SYNCb input source. Where, 0 = use SYNCBIN0 for this framer, 1 =
       use SYNCBIN1 for this framer
   * - adi,jesd204-framer-a-over-sample
     - 0
     - Selects framer bit repeat or oversampling mode for lane rate matching.
       Where, 0 = bitRepeat mode (changes effective lanerate), 1 = overSample
       (maintains same lane rate between ObsRx framer and Rx framer and
       oversamples the ADC samples)
   * - adi,jesd204-framer-a-syncb-in-lvds-mode
     - 1
     - 1 - enable LVDS input pad with 100ohm internal termination, 0 - enable
       CMOS input pad
   * - adi,jesd204-framer-a-syncb-in-lvds-pn-invert
     - 0
     - 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted
   * - adi,jesd204-framer-a-enable-manual-lane-xbar
     - 0
     - 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping
       (use serializerLaneCrossbar value with no checking)

FRAMER B
~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-framer-b-bank-id
     - 0
     - JESD204B Configuration Bank ID extension to Device ID. Range is 0..15
   * - adi,jesd204-framer-b-device-id
     - 0
     - JESD204B Configuration Device ID link identification number. Range is
       0..255
   * - adi,jesd204-framer-b-lane0-id
     - 0
     - JESD204B Configuration starting Lane ID. If more than one lane is used,
       each lane will increment from the Lane0 ID. Range is 0..31
   * - adi,jesd204-framer-b-m
     - 4
     - Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive chain
       (I and Q)
   * - adi,jesd204-framer-b-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K must be modulo 4.
       Where, F=2*M/numberOfLanes
   * - adi,jesd204-framer-b-f
     - 4
     - Number of bytes(octets) per frame (Valid 1, 2, 4, 8)
   * - adi,jesd204-framer-b-np
     - 16
     - converter sample resolution (12, 16, 24)
   * - adi,jesd204-framer-b-scramble
     - 1
     - Scrambling off if framerScramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-framer-b-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF(not currently valid), 1 =
       use external SYSREF
   * - adi,jesd204-framer-b-serializer-lanes-enabled
     - 0x0C
     - Serializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1
       enabled, etc
   * - adi,jesd204-framer-b-serializer-lane-crossbar
     - 0xE4
     - Lane crossbar to map framer lane outputs to physical lanes
   * - adi,jesd204-framer-b-lmfc-offset
     - 31
     - LMFC offset value for deterministic latency setting. Range is 0..31
   * - adi,jesd204-framer-b-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, 0 = not set
   * - adi,jesd204-framer-b-syncb-in-select
     - 1
     - Selects SYNCb input source. Where, 0 = use SYNCBIN0 for this framer, 1 =
       use SYNCBIN1 for this framer
   * - adi,jesd204-framer-b-over-sample
     - 0
     - Selects framer bit repeat or oversampling mode for lane rate matching.
       Where, 0 = bitRepeat mode (changes effective lanerate), 1 = overSample
       (maintains same lane rate between ObsRx framer and Rx framer and
       oversamples the ADC samples)
   * - adi,jesd204-framer-b-syncb-in-lvds-mode
     - 1
     - 1 - enable LVDS input pad with 100ohm internal termination, 0 - enable
       CMOS input pad
   * - adi,jesd204-framer-b-syncb-in-lvds-pn-invert
     - 0
     - 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted
   * - adi,jesd204-framer-b-enable-manual-lane-xbar
     - 0
     - 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping
       (use serializerLaneCrossbar value with no checking)

DEFRAMER A
~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-deframer-a-bank-id
     - 0
     - Extension to Device ID. Range is 0..15
   * - adi,jesd204-deframer-a-device-id
     - 0
     - Link identification number. Range is 0..255
   * - adi,jesd204-deframer-a-lane0-id
     - 0
     - Lane0 ID. Range is 0..31
   * - adi,jesd204-deframer-a-m
     - 4
     - Number of DACs (0, 2, or 4) - 2 DACs per transmit chain (I and Q)
   * - adi,jesd204-deframer-a-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K = modulo 4. Where,
       F=2*M/numberOfLanes
   * - adi,jesd204-deframer-a-scramble
     - 1
     - Scrambling off if scramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-deframer-a-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF, 1 = external SYSREF
   * - adi,jesd204-deframer-a-deserializer-lanes-enabled
     - 0x0F
     - Deserializer lane select bit field. Where, [0] = Lane0 enabled, [1] =
       Lane1 enabled, etc
   * - adi,jesd204-deframer-a-deserializer-lane-crossbar
     - 0xE4
     - Lane crossbar to map deframer lane outputs to physical lanes
   * - adi,jesd204-deframer-a-lmfc-offset
     - 17
     - LMFC offset value to adjust deterministic latency. Range is 0..31
   * - adi,jesd204-deframer-a-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, ‘0’ = not set
   * - adi,jesd204-deframer-a-syncb-out-select
     - 0
     - Selects deframer SYNCBOUT pin (0 = SYNCBOUT0, 1 = SYNCBOUT1)
   * - adi,jesd204-deframer-a-np
     - 16
     - converter sample resolution (12, 16)
   * - adi,jesd204-deframer-a-syncb-out-lvds-mode
     - 1
     - 1 - enable LVDS output pad, 0 - enable CMOS output pad
   * - adi,jesd204-deframer-a-syncb-out-lvds-pn-invert
     - 0
     - 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted
   * - adi,jesd204-deframer-a-syncb-out-cmos-slew-rate
     - 0
     - 0 - fastest rise/fall times, 3 - slowest rise/fall times
   * - adi,jesd204-deframer-a-syncb-out-cmos-drive-level
     - 0
     - 0 - normal cmos drive level, 1 - double cmos drive level
   * - adi,jesd204-deframer-a-enable-manual-lane-xbar
     - 0
     - 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping
       (use deserializerLaneCrossbar value with no checking)

DEFRAMER B
~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,jesd204-deframer-b-bank-id
     - 0
     - Extension to Device ID. Range is 0..15
   * - adi,jesd204-deframer-b-device-id
     - 0
     - Link identification number. Range is 0..255
   * - adi,jesd204-deframer-b-lane0-id
     - 0
     - Lane0 ID. Range is 0..31
   * - adi,jesd204-deframer-b-m
     - 0
     - Number of DACs (0, 2, or 4) - 2 DACs per transmit chain (I and Q)
   * - adi,jesd204-deframer-b-k
     - 32
     - Number of frames in a multiframe. Default = 32, F*K = modulo 4. Where,
       F=2*M/numberOfLanes
   * - adi,jesd204-deframer-b-scramble
     - 1
     - Scrambling off if scramble = 0, if framerScramble > 0 scrambling is
       enabled
   * - adi,jesd204-deframer-b-external-sysref
     - 1
     - External SYSREF select. 0 = use internal SYSREF, 1 = external SYSREF
   * - adi,jesd204-deframer-b-deserializer-lanes-enabled
     - 0
     - Deserializer lane select bit field. Where, [0] = Lane0 enabled, [1] =
       Lane1 enabled, etc
   * - adi,jesd204-deframer-b-deserializer-lane-crossbar
     - 0xE4
     - Lane crossbar to map deframer lane outputs to physical lanes
   * - adi,jesd204-deframer-b-lmfc-offset
     - 0
     - LMFC offset value to adjust deterministic latency. Range is 0..31
   * - adi,jesd204-deframer-b-new-sysref-on-relink
     - 0
     - Flag for determining if SYSREF on relink should be set. Where, if > 0 =
       set, ‘0’ = not set
   * - adi,jesd204-deframer-b-syncb-out-select
     - 0
     - Selects deframer SYNCBOUT pin (0 = SYNCBOUT0, 1 = SYNCBOUT1)
   * - adi,jesd204-deframer-b-np
     - 16
     - converter sample resolution (12, 16)
   * - adi,jesd204-deframer-b-syncb-out-lvds-mode
     - 1
     - 1 - enable LVDS output pad, 0 - enable CMOS output pad
   * - adi,jesd204-deframer-b-syncb-out-lvds-pn-invert
     - 0
     - 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted
   * - adi,jesd204-deframer-b-syncb-out-cmos-slew-rate
     - 0
     - 0 - fastest rise/fall times, 3 - slowest rise/fall times
   * - adi,jesd204-deframer-b-syncb-out-cmos-drive-level
     - 0
     - 0 - normal cmos drive level, 1 - double cmos drive level
   * - adi,jesd204-deframer-b-enable-manual-lane-xbar
     - 0
     - 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping
       (use deserializerLaneCrossbar value with no checking)

ARM GPIO Settings
-----------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,arm-gpio-config-orx1-tx-sel0-pin-gpio-pin-sel
     - 0
     - Select desired GPIO pin to input into Talise (valid 0-15)
   * - adi,arm-gpio-config-orx1-tx-sel0-pin-polarity
     - 0
     - Signal polarity (0 = Normal polarity, 1=Talise will invert the signal
       before using)
   * - adi,arm-gpio-config-orx1-tx-sel0-pin-enable
     - 0
     - 1 = Enable Talise ARM use of GPIO signal, 0 = Talise ARM uses ARM command
       to set this signal value
   * - adi,arm-gpio-config-orx1-tx-sel1-pin-gpio-pin-sel
     - 0
     - Select desired GPIO pin to input into Talise (valid 0-15)
   * - adi,arm-gpio-config-orx1-tx-sel1-pin-polarity
     - 0
     - Signal polarity (0 = Normal polarity, 1=Talise will invert the signal
       before using)
   * - adi,arm-gpio-config-orx1-tx-sel1-pin-enable
     - 0
     - 1 = Enable Talise ARM use of GPIO signal, 0 = Talise ARM uses ARM command
       to set this signal value
   * - adi,arm-gpio-config-orx2-tx-sel0-pin-gpio-pin-sel
     - 0
     - Select desired GPIO pin to input into Talise (valid 0-15)
   * - adi,arm-gpio-config-orx2-tx-sel0-pin-polarity
     - 0
     - Signal polarity (0 = Normal polarity, 1=Talise will invert the signal
       before using)
   * - adi,arm-gpio-config-orx2-tx-sel0-pin-enable
     - 0
     - 1 = Enable Talise ARM use of GPIO signal, 0 = Talise ARM uses ARM command
       to set this signal value
   * - adi,arm-gpio-config-orx2-tx-sel1-pin-gpio-pin-sel
     - 0
     - Select desired GPIO pin to input into Talise (valid 0-15)
   * - adi,arm-gpio-config-orx2-tx-sel1-pin-polarity
     - 0
     - Signal polarity (0 = Normal polarity, 1=Talise will invert the signal
       before using)
   * - adi,arm-gpio-config-orx2-tx-sel1-pin-enable
     - 0
     - 1 = Enable Talise ARM use of GPIO signal, 0 = Talise ARM uses ARM command
       to set this signal value
   * - adi,arm-gpio-config-en-tx-tracking-cals-gpio-pin-sel
     - 0
     - Select desired GPIO pin to input into Talise (valid 0-15)
   * - adi,arm-gpio-config-en-tx-tracking-cals-polarity
     - 0
     - Signal polarity (0 = Normal polarity, 1=Talise will invert the signal
       before using)
   * - adi,arm-gpio-config-en-tx-tracking-cals-enable
     - 0
     - 1 = Enable Talise ARM use of GPIO signal, 0 = Talise ARM uses ARM command
       to set this signal value
   * - adi,orx-lo-cfg-disable-aux-pll-relocking
     - 0
     - Disables the ARM from automatically relocking the Aux PLL. Set to 1 when
       using AuxLO as ORx LO source, 0 = default when RFPLL used as ORx LO
       source
   * - adi,orx-lo-cfg-gpio-select
     - 19
     - TAL_GPIO_INVALID = disable pin mode, GPIO0-15 valid

GPIO 3v3 Settings
-----------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,gpio3v3-source-control
     - 0
     - Configures the GPIO source control. Each nibble of this value describes
       the GPIO source for the GPIO output pins. For example a value 0x333, will
       configure all 3 nibbles as bitbang GPIOs. Please refer to the reference
       manual UG-1295
   * - adi,gpio3v3-output-enable-mask
     - 0
     - This attribute will set the 3.3V GPIO direction. The direction can be
       either output or input per pin. A set bit configures the pin to act as
       output.
   * - adi,gpio3v3-output-level-mask
     - 0
     - This attribute will affect the GPIO pins that have their direction set to
       output. The value mask is being set during initialization.

Frequency Hopping Mode Settings
-------------------------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,fhm-config-fhm-gpio-pin
     - 0
     - Maps the Talise ARM GPIO pin(TAL_GPIO_0 - TAL_GPIO_15) for frequency
       hopping. A low to high pulse on this pin triggers freq hopping Setting
       fhmGpioPin = TAL_GPIO_INVALID will unassign ARM GPIO pin mapped to Rf Pll
       Frequency hopping
   * - adi,fhm-config-fhm-min-freq_mhz
     - 100
     - Sets frequency hopping range minimum frequency
   * - adi,fhm-config-fhm-max-freq_mhz
     - 100
     - Sets frequency hopping range maximum frequency

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,fhm-mode-fhm-enable
     - 0
     - 0 - Disables Frequency Hopping, 1 - Enables Frequency Hopping
   * - adi,fhm-mode-enable-mcs-sync
     - 0
     - 0 - Disables MCS Synchronization on FHM enable, 1 - Enables MCS
       Synchronization on FHM enable. Ignored if fhmEnable = 0
   * - adi,fhm-mode-fhm-trigger-mode
     - 0
     - TAL_FHM_GPIO_MODE - Frequency Hop triggered via GPIO low to high pulse
       TAL_FHM_NON_GPIO_MODE - Frequency Hop triggered via ARM command
   * - adi,fhm-mode-fhm-exit-mode
     - 0
     - TAL_FHM_QUICK_EXIT = quick exit on frequency hopping disable,
       TAL_FHM_FULL_EXIT = Full exit on frequency hopping disable. This is
       ignored if fhmEnable = 1
   * - adi,fhm-mode-fhm-init-frequency_hz
     - 2450000000
     - First hop frequency that Rf Pll is configured to on enabling FHM

Gain and Attenuation Control Pin Settings
-----------------------------------------

RX1
~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx1-gain-ctrl-pin-inc-step
     - 1
     - Increment in gain index applied when the increment gain pin is pulsed. A
       value of 0 to 7 applies a step size of 1 to 8
   * - adi,rx1-gain-ctrl-pin-dec-step
     - 1
     - Decrement in gain index applied when the increment gain pin is pulsed. A
       value of 0 to 7 applies a step size of 1 to 8
   * - adi,rx1-gain-ctrl-pin-rx-gain-inc-pin
     - 0
     - GPIO used for the Increment gain input: Rx1 : TAL_GPIO_00 or TAL_GPIO_10,
       Rx2 : TAL_GPIO_03 or TAL_GPIO_13
   * - adi,rx1-gain-ctrl-pin-rx-gain-dec-pin
     - 1
     - GPIO used for the Decrement gain input: Rx1 : TAL_GPIO_01 or TAL_GPIO_11,
       Rx2 : TAL_GPIO_04 or TAL_GPIO_14
   * - adi,rx1-gain-ctrl-pin-enable
     - 0
     - Enable (1) or disable (0) the gain pin control

RX2
~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx2-gain-ctrl-pin-inc-step
     - 1
     - Increment in gain index applied when the increment gain pin is pulsed. A
       value of 0 to 7 applies a step size of 1 to 8
   * - adi,rx2-gain-ctrl-pin-dec-step
     - 1
     - Decrement in gain index applied when the increment gain pin is pulsed. A
       value of 0 to 7 applies a step size of 1 to 8
   * - adi,rx2-gain-ctrl-pin-rx-gain-inc-pin
     - 3
     - GPIO used for the Increment gain input: Rx1 : TAL_GPIO_00 or TAL_GPIO_10,
       Rx2 : TAL_GPIO_03 or TAL_GPIO_13
   * - adi,rx2-gain-ctrl-pin-rx-gain-dec-pin
     - 4
     - GPIO used for the Decrement gain input: Rx1 : TAL_GPIO_01 or TAL_GPIO_11,
       Rx2 : TAL_GPIO_04 or TAL_GPIO_14
   * - adi,rx2-gain-ctrl-pin-enable
     - 0
     - Enable (1) or disable (0) the gain pin control

TX1
~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx1-atten-ctrl-pin-step-size
     - 0
     - The step that will increase or decrease the channel attenuation. This
       parameter sets the change in Tx attenuation for each increment or
       decrement signal received in incr/decr mode. Step of 1 changes
       attenuation by 0.05dB. Valid range is from 0 to 31
   * - adi,tx1-atten-ctrl-pin-tx-atten-inc-pin
     - 4
     - GPIO used to increment Tx attenuation Tx1 : TAL_GPIO_04 or TAL_GPIO_12
       Tx2 : TAL_GPIO_06 or TAL_GPIO_14
   * - adi,tx1-atten-ctrl-pin-tx-atten-dec-pin
     - 5
     - GPIO used to decrement Tx attenuation Tx2 : TAL_GPIO_07 or TAL_GPIO_15
   * - adi,tx1-atten-ctrl-pin-enable
     - 0
     - Enable (1) or disable (0) the attenuation pin control

TX2
~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx2-atten-ctrl-pin-step-size
     - 0
     - The step that will increase or decrease the channel attenuation. This
       parameter sets the change in Tx attenuation for each increment or
       decrement signal received in incr/decr mode. Step of 1 changes
       attenuation by 0.05dB. Valid range is from 0 to 31
   * - adi,tx2-atten-ctrl-pin-tx-atten-inc-pin
     - 6
     - GPIO used to increment Tx attenuation Tx1 : TAL_GPIO_04 or TAL_GPIO_12
       Tx2 : TAL_GPIO_06 or TAL_GPIO_14
   * - adi,tx2-atten-ctrl-pin-tx-atten-dec-pin
     - 7
     - GPIO used to decrement Tx attenuation Tx2 : TAL_GPIO_07 or TAL_GPIO_15
   * - adi,tx2-atten-ctrl-pin-enable
     - 0
     - Enable (1) or disable (0) the attenuation pin control

PA Protection Settings
----------------------

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx-pa-protection-avg-duration
     - 3
     - Number of Tx samples (at JESD204 IQ sample rate) to average for the power
       measurement. samples = 2^(avgDuration + 5), 0 = 32 samples, max:14 =
       524288 samples
   * - adi,tx-pa-protection-tx-atten-step
     - 2
     - if PA protection threshold met, Tx Atten = TxAttenSetting + (txAttenStep
       \* 0.4dB)
   * - adi,tx-pa-protection-tx1-power-threshold
     - 4096
     - tx1PowerThreashold = round(4096 \* 10^(tx1PowerThreshold_dBFS / 10))
       (valid 1-8191)
   * - adi,tx-pa-protection-tx2-power-threshold
     - 4096
     - tx2PowerThreashold = round(4096 \* 10^(tx2PowerThreshold_dBFS / 10))
       (valid 1-8191)
   * - adi,tx-pa-protection-peak-count
     - 4
     - 0=Peak Mode is disabled, if the Tx peak power threshold is exceeded more
       than peakCount times within one average duration, a PA error is flagged
       (Si Rev 0xB1: 0-30, Si Rev 0xC0: 0-31)
   * - adi,tx-pa-protection-tx1-peak-threshold
     - 130
     - 8-bit threshold for Tx1 peak detect. When instantaneous power exceeds
       this threshold, a peak is registered (valid 1-255) tx1PeakThreshold =
       round(128 \* 10^(tx1PeakThreshold_dBFS / 10))
   * - adi,tx-pa-protection-tx2-peak-threshold
     - 130
     - 8-bit threshold for Tx2 peak detect. When instantaneous power exceeds
       this threshold, a peak is registered (valid 1-255) tx2PeakThreashold =
       round(128 \* 10^(tx2PeakThreshold_dBFS / 10))

Profile Settings
----------------

Clocks
~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,dig-clocks-device-clock_khz
     - 245760
     - CLKPLL and device reference clock frequency in kHz
   * - adi,dig-clocks-clk-pll-vco-freq_khz
     - 9830400
     - CLKPLL VCO frequency in kHz
   * - adi,dig-clocks-clk-pll-hs-div
     - 1
     - CLKPLL high speed clock divider (Encoding is: VAL=Clock Divide Ratio;
       0=2.0, 1=2.5, 2=3.0, 3=4.0, 4=5.0)
   * - adi,dig-clocks-rf-pll-use-external-lo
     - 0
     - 1= Use external LO input for RF PLL, 0 = use internal LO generation for
       RF PLL
   * - adi,dig-clocks-rf-pll-phase-sync-mode
     - 0
     - Set RF PLL phase synchronization mode. Adds extra time to lock RF PLL
       when PLL frequency changed. See enum for options

RX
~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-profile-rx-fir-decimation
     - 2
     - Rx FIR decimation (1,2,4)
   * - adi,rx-profile-rx-dec5-decimation
     - 4
     - Decimation of Dec5 or Dec4 filter (5,4)
   * - adi,rx-profile-rhb1-decimation
     - 1
     - RX Halfband1 (HB1) decimation. Can be either 1 or 2
   * - adi,rx-profile-rx-output-rate_khz
     - 245760
     - Rx Output data rate in kHz
   * - adi,rx-profile-rf-bandwidth_hz
     - 200000000
     - Rx RF passband bandwidth for the profile
   * - adi,rx-profile-rx-bbf3d-bcorner_khz
     - 200000
     - Rx BBF (TIA) 3dB corner in kHz
   * - adi,rx-profile-rx-ddc-mode
     - 0
     - Rx DDC mode

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-nco-shifter-band-a-input-band-width_khz
     - 0
     - BandWidth in khz of the BandA input signal
   * - adi,rx-nco-shifter-band-a-input-center-freq_khz
     - 0
     - Center Frequency in khz of the BandA input signal
   * - adi,rx-nco-shifter-band-a-nco1-freq_khz
     - 0
     - BandA NCO1 Frequency shift in khz
   * - adi,rx-nco-shifter-band-a-nco2-freq_khz
     - 0
     - BandA NCO2 Frequency shift in khz
   * - adi,rx-nco-shifter-band-binput-band-width_khz
     - 0
     - BandWidth in khz of the BandB input signal
   * - adi,rx-nco-shifter-band-binput-center-freq_khz
     - 0
     - Center Frequency in khz of the BandB input signal
   * - adi,rx-nco-shifter-band-bnco1-freq_khz
     - 0
     - BandB NCO1 Frequency shift in khz
   * - adi,rx-nco-shifter-band-bnco2-freq_khz
     - 0
     - BandB NCO2 Frequency shift in khz

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,rx-settings-framer-sel
     - 0
     - Rx JESD204b framer configuration enum
   * - adi,rx-settings-rx-channels
     - 3
     - The desired Rx Channels to enable during initialization

ORX
~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,orx-profile-rx-fir-decimation
     - 1
     - ORx FIR decimation (1,2,4)
   * - adi,orx-profile-rx-dec5-decimation
     - 4
     - Decimation of Dec5 or Dec4 filter (5,4)
   * - adi,orx-profile-rhb1-decimation
     - 2
     - ORX Halfband1 (HB1) decimation. Can be either 1 or 2
   * - adi,orx-profile-orx-output-rate_khz
     - 245760
     - ORx Output data rate in kHz
   * - adi,orx-profile-rf-bandwidth_hz
     - 200000000
     - ORx RF passband bandwidth for the profile
   * - adi,orx-profile-rx-bbf3d-bcorner_khz
     - 225000
     - ORx BBF (TIA) 3dB corner in kHz
   * - adi,orx-profile-orx-ddc-mode
     - 0
     - ORx DDC mode

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,obs-settings-framer-sel
     - 1
     - ObsRx JESD204b framer configuration structure
   * - adi,obs-settings-obs-rx-channels-enable
     - 3
     - The desired ObsRx Channel to enable during initialization
   * - adi,obs-settings-obs-rx-lo-source
     - 0
     - Field not used, reserved for future use. The ORx mixers can use the
       RF_PLL or Aux_PLL

TX
~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx-profile-dac-div
     - 1
     - The divider used to generate the DAC clock (1,2)
   * - adi,tx-profile-tx-fir-interpolation
     - 1
     - The TX digital FIR filter interpolation (1,2,4)
   * - adi,tx-profile-thb1-interpolation
     - 2
     - Tx Halfband1 (HB1) filter interpolation (1,2)
   * - adi,tx-profile-thb2-interpolation
     - 2
     - Tx Halfband2 (HB2) filter interpolation (1,2)
   * - adi,tx-profile-thb3-interpolation
     - 2
     - Tx Halfband3 (HB3) filter interpolation (1,2)
   * - adi,tx-profile-tx-int5-interpolation
     - 1
     - Tx Int5 filter interpolation (1,5)
   * - adi,tx-profile-tx-input-rate_khz
     - 245760
     - Tx input data rate in kHz
   * - adi,tx-profile-primary-sig-bandwidth_hz
     - 100000000
     - Tx primary signal BW
   * - adi,tx-profile-rf-bandwidth_hz
     - 225000000
     - Tx RF passband bandwidth for the profile
   * - adi,tx-profile-tx-dac3d-bcorner_khz
     - 225000
     - DAC filter 3dB corner in kHz
   * - adi,tx-profile-tx-bbf3d-bcorner_khz
     - 113000
     - Tx BBF 3dB corner in kHz

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,tx-settings-deframer-sel
     - 0
     - Talise JESD204b deframer select (Deframer A or B, or both)
   * - adi,tx-settings-tx-channels
     - 3
     - The desired Tx channels to enable during initialization
   * - adi,tx-settings-tx-atten-step-size
     - 0
     - Tx Attenuation step size
   * - adi,tx-settings-tx1-atten_md-b
     - 10000
     - Initial and current Tx1 Attenuation
   * - adi,tx-settings-tx2-atten_md-b
     - 10000
     - Initial and current Tx2 Attenuation
   * - adi,tx-settings-dis-tx-data-if-pll-unlock
     - 0
     - Options to disable Transmit data when the RFPLL unlocks
