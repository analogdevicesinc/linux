.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/adrv9002-customization

.. _adrv9002-customization:

ADRV9002 Customization
======================

ADRV9002 Device Driver Customization.

.. note::

   There are configuration options that must be set properly. Some others allow
   you to set defaults, but can be changed anytime later using the driver API.
   But most of these options don’t need to be changed at all.

   If unsure please see the manual or don’t change!

RX and TX Channels
------------------

ADRV9002 can operate different RX and TX channels at different rates due to the
flexibility of the hardware. Making it possible to run four channels
(TX1,TX2,RX1,RX2) all at different rates. However, this will require a specific
configuration in devicetree to make this possible.

The two main categories are combined DMA mode, which we refer to as MIMO mode,
and a split DMA mode. In the first case, a single DMA driver is instantiated
with up to four channels (two complex) per data direction. This allows for
synchronous capture between RX1 and RX2 or transmission with TX1 and TX2.
Alternatively, in split mode two drivers per direction will be instantiated.
This is typically used when different rates are needed between channels and data
transfers between channels are not required to by synchronous.

For examples of these arrangements, look at the following devicetrees:

- :git+linux:`MIMO (combined) mode <e2d92f9152d49473a887aad70486855e5843faa5:arch/arm64/boot/dts/xilinx>`
- :git+linux:`Split mode <e2d92f9152d49473a887aad70486855e5843faa5:arch/arm64/boot/dts/xilinx>`

RX
--

The following attributes are valid for both RX1 and RX2. The user needs to
define a valid devicetree node for each RX to apply changes separately to each
one. Both RXs must be under adi,channels node as can be seen in the example:

::

   adi,channels {
       #address-cells = <1>;
       #size-cells = <0>;

       rx@0 {
           reg = <0>;
           adi,port = <0>;
           adi,agc = <&agc0>;
           adi,pinctrl = <&rx_pinctrl0>;
       };

       rx@1 {
           reg = <1>;
           adi,port = <0>;
           adi,agc = <&agc0>;
           adi,pinctrl = <&rx_pinctrl1>;
       };
   };

Base Settings
~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
     -
   * - reg
     - NA
     - Specifies the RX channel. Either 0=RX1 or 1=RX2. This is a mandatory property.
     -
   * - adi,port
     - NA
     - Specifies the type of the port. Must be 0 for RX. This is a mandatory property.
     -
   * - adi,agc
     - NA
     - Phandle to Automatic Gain Control. :ref:`adrv9002-customization agc-settings`
     -
   * - adi,pinctrl
     - NA
     - Phandle for Gain Pin Control settings. :ref:`adrv9002-customization gain-pin-control-settings`
     -
   * - adi,min-gain-index
     - 183
     - Minimum Gain index. This will be applied to both pinctrl and agc if defined. Valid range from 183 and adi,max-gain-index - 1.
     -
   * - adi,max-gain-index
     - 255
     - Maximum Gain index. This will be applied to both pinctrl and agc if defined. Valid range from adi,min-gain-index + 1 and 255.
     -

.. _adrv9002-customization agc-settings:

AGC Settings
~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,peak-wait-time
     - 4
     - AGC peak wait time. Valid range is from 0 to 31.
   * - adi,gain-update-counter
     - 11520
     - AGC gain update time denominated in AGC clock cycles; Valid range is from
       0 to 4194303.
   * - adi,attack-delax-us
     - 10
     - Delay time, denominated in microseconds, before starting AGC, after
       starting Rx. Valid range is from 0 to 63.
   * - adi,slow-loop-settling-delay
     - 16
     - On any gain change, the AGC waits for the time (range 0 to 127) specified
       in AGC clock cycles to allow gain transients to flow through the Rx path
       before starting any measurements.
   * - adi,low-threshold-prevent-gain-inc
     - 0
     - Prevent gain index from incrementing while peak thresholds are exceeded.
   * - adi,change-gain-threshold-high
     - 3
     - Enable immediate gain change if high threshold counter is exceeded.
   * - adi,agc-mode
     - 1
     - AGC mode can be either power and peak or only peak.
   * - adi,reset-on-rx-on
     - 0
     - Reset the AGC slow loop state machine to max gain when the Rx Enable is
       taken low.
   * - adi,reset-on-rx-on-gain-index
     - 255
     - AGC Reset On gain index. Valid range is from minGainIndex to
       maxGainIndex.
   * - adi,sync-pulse-gain-counter-en
     - 0
     - Enable the AGC gain update counter to be sync’ed to a time-slot boundary.
   * - adi,fast-recovery-loop-en
     - 0
     - Enable multiple time constants in AGC loop for fast attack and fast
       recovery.
   * - adi,no-power-measurement-en
     - 1
     - Disable the Rx power measurement block
   * - adi,power-under-range-high-threshold
     - 4
     - Threshold which defines the lower boundary on the stable region of the
       power detect gain control mode. Valid Range from 0 to 127.
   * - adi,power-under-range-low-threshold
     - 0
     - Offset from adi,power-under-range-high-threshold which defines the outer
       boundary of the power based AGC convergence. Valid offset from 0 to 15
   * - adi,power-under-range-high-gain-step-recovery
     - 4
     - The number of indices that the gain index pointer should be increased
       (gain increase) in the event of the power measurement being less than
       adi,power-under-range-high-threshold but greater than
       adi,power-under-range-low-threshold. Valid range from 0 to 31.
   * - adi,power-under-range-low-gain-step-recovery
     - 4
     - The number of indices that the gain index pointer should be increased
       (gain increase) in the event of the power measurement being less than
       adi,power-under-range-low-threshold. Valid range from 0 to 31.
   * - adi,power-measurement-duration
     - 10
     - Average power measurement duration = 8*2^powerMeasurementDuration. Valid
       range from 0 to 31.
   * - adi,power-measurement-delay
     - 3
     - Average power measurement delay between sucessive measurement. Valid
       range from 0 to 255.
   * - adi,power-rx-tdd-measurement-duration
     - 0
     - Measurement duration to detect power for specific slice of the gain
       update counter. Valid range from 0 to 65535.
   * - adi,power-rx-tdd-measurement-delay
     - 92
     - Measurement delay to detect power for specific slice of the gain update
       counter. Valid range from 0 to 65535.
   * - adi,power-over-range-high-threshold
     - 0
     - Offset from threshold adi,power-over-range-low-threshold which defines
       the outer boundary on the stable region of the power detect gain control
       mode. Valid range from 0 to 15.
   * - adi,power-over-range-low-threshold
     - 1
     - Threshold which defines the upper boundary on the stable region of the
       power detect gain control mode. Valid range from 0 to 127.
   * - adi,power-over-range-high-gain-step-attack
     - 4
     - The number of indices that the gain index pointer should be decreased
       (gain reduction) in the event of the power measurement being greater than
       adi,power-over-range-high-threshold. Valid range from 0 to 31.
   * - adi,power-over-range-low-gain-step-attack
     - 4
     - The number of indices that the gain index pointer should be decreased
       (gain decrease) in the event of the power measurement being less than
       adi,power-over-range-high-threshold but greater than
       adi,power-over-range-low-threshold. Valid range from 0 to 31
   * - adi,agc-power-feedback-high-thres-exceeded
     - 0
     - A pair of DGPIO pins to retrieve the gain change information and power
       detector inner low threshold not exceeded status. Valid range from 0 to
       9.
   * - adi,agc-power-feedback-low-thres-gain-change
     - 0
     - A pair of DGPIO pins to retrieve the power detector inner high threshold
       exceeded status and apd high threshold counter exceeded status. Valid
       range from 0 to 9.
   * - adi,peak-agc-under-range-low-interval
     - 50
     - Update interval for AGC loop mode in AGC clock cycles. Valid range is 0
       to 65535.
   * - adi,peak-agc-under-range-mid-interval
     - 2
     - 2nd update interval for multiple time constant AGC mode. Calculated as
       (adi,peak-agc-under-range-mid-interval+1)*adi,peak-agc-under-range-low-interval_ns. Valid range is 0 to 63.
   * - adi,peak-agc-under-range-high-interval
     - 4
     - 3rd update interval for multiple time constant AGC mode. Calculated as
       (adi,peak-agc-under-range-high-interval+1)*2nd update interval. Valid
       range is 0 to 63.
   * - adi,peak-apd-high-threshold
     - 38
     - AGC APD high threshold. Valid range is 0 to 63.
   * - adi,peak-apd-low-threshold
     - 27
     - AGC APD low threshold. Valid range is 0 to 63. Recommended to be 3dB
       below adi,peak-apd-high-threshol.
   * - adi,peak-apd-upper-threshold-exceeded-count
     - 6
     - AGC APD peak detect upper threshold count. Valid range is 0 to 255.
   * - adi,peak-apd-lower-threshold-exceeded-count
     - 3
     - AGC APD peak detect lower threshold count. Valid range is 0 to 255.
   * - adi,peak-apd-gain-step-attack
     - 4
     - AGC APD peak detect attack gain step. Valid range is 0 to 31.
   * - adi,peak-apd-gain-step-recovery
     - 0
     - AGC APD gain index step size for recovery. Valid range is 0 to 31.
   * - adi,no-peak-hb-overload-en
     - 1
     - Disable the HB overload detector.
   * - adi,peak-hb-overload-duration-count
     - 1
     - Sets the window of clock cycles (at the HB output rate) to meet the
       overload count. (0 = 2 cycles, 1 = 4 cycles, 2 = 8 cycles, 3 = 12 cycles,
       4 = 16 cycles, 5 = 24 cycles, 6 = 32 cycles).
   * - adi,peak-hb-overload-threshold-count
     - 1
     - Sets the number of actual overloads required to trigger the overload
       signal. Valid range from 1 to 15.
   * - adi,peak-hb-high-threshold
     - 255
     - AGC HB output high threshold. Valid range from 0 to 16383.
   * - adi,peak-hb-under-range-low-threshold
     - 61
     - AGC HB output low threshold. Valid range from 0 to 16383
   * - adi,peak-hb-under-range-mid-threshold
     - 114
     - AGC HB output low threshold for 2nd interval for multiple time constant
       AGC mode. Valid range from 0 to 16383.
   * - adi,peak-hb-under-range-high-threshold
     - 161
     - AGC HB output low threshold for 3rd interval for multiple time constant
       AGC mode. Valid range from 0 to 16383.
   * - adi,peak-hb-upper-threshold-exceeded-count
     - 6
     - AGC HB output upper threshold count. Valid range from 0 to 255.
   * - adi,peak-hb-under-range-high-threshold-exceeded-count
     - 3
     - AGC HB output lower threshold count. Valid range from 0 to 255.
   * - adi,peak-hb-gain-step-high-recovery
     - 4
     - AGC HB gain index step size. Valid range from 0 to 31.
   * - adi,peak-hb-gain-step-low-recovery
     - 6
     - AGC HB gain index step size, when the HB Low Overrange interval 2
       triggers. Valid range from 0 to 31.
   * - adi,peak-hb-gain-step-mid-recovery
     - 4
     - AGC HB gain index step size, when the HB Low Overrange interval 3
       triggers. Valid range from 0 to 31.
   * - adi,peak-hb-gain-step-attack
     - 4
     - AGC HB output attack gain step. Valid range from 0 to 31.
   * - adi,peak-hb-overload-power-mode
     - 0
     - Select magnitude measurements or power mearurements. 0 = enable peak mode
       1 = enable I2 + Q2 power mode.
   * - adi,peak-hb-under-range-mid-threshold-exceeded-count
     - 3
     - AGC HB output upper threshold count. Valid range from 0 to 255.
   * - adi,peak-hb-under-range-low-threshold-exceeded-count
     - 3
     - AGC HB output lower threshold count. Valid range from 0 to 255.
   * - adi,agc-peak-feedback-high-thres-counter-exceeded
     - 0
     - A pair of DGPIO pins to retrieve the hb low threshold counter exceeded
       status and apd low threshold counter exceeded status. Valid range from 0
       to 9.
   * - adi,agc-peak-feedback-low-thres-counter-exceeded
     - 0
     - A pair of DGPIO pins to retrieve the hb high threshold counter exceeded
       status and apd high threshold counter exceeded status. Valid range from 0
       to 9.

.. _adrv9002-customization gain-pin-control-settings:

Gain PIN Control Settings
^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,increment-step-size
     - 1
     - Number of indices to increase gain on rising edge on incrementPin (Range:
       1 to 7)
   * - adi,decrement-step-size
     - 1
     - Number of indices to decrease gain on rising edge on decrementPin (Range:
       1 to 7)
   * - adi,increment-pin
     - NA
     - A rising edge on this pin will increment gain by adi,increment-step-size.
       Valid range from 0 to 15. Mandatory property in this node.
   * - adi,decrement-pin
     - NA
     - A rising edge on this pin will decrement gain by adi,decrement-step-size.
       Valid range from 0 to 15. Mandatory property in this node.

TX
~~

The following attributes are valid for both TX1 and TX2. The user needs to
define a valid devicetree node for each TX to apply changes separately to each
one. Both TXs must be under adi,channels node as can be seen in the example:

::

   adi,channels {
       #address-cells = <1>;
       #size-cells = <0>;

           ...

       tx@0 {
           reg = <0>;
           adi,port = <1>;
           adi,pinctrl = <&tx_pinctrl0>;
       };

       tx@1 {
           reg = <1>;
           adi,port = <1>;
           adi,pinctrl = <&tx_pinctrl1>;
       };
   };

Base Settings
~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
     -
   * - reg
     - NA
     - Specifies the TX channel. Either 0=TX1 or 1=TX2. This is a mandatory property.
     -
   * - adi,port
     - NA
     - Specifies the type of the port. Must be 1 for TX. This is a mandatory property.
     -
   * - adi,pinctrl
     - NA
     - Phandle for Attenuation Pin Control settings. :ref:`adrv9002-customization attenuation-pin-control-settings`
     -
   * - adi,dac-full-scale-boost
     - 0
     - Enable full scale DAC.
     -

.. _adrv9002-customization attenuation-pin-control-settings:

Attenuation PIN Control Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - adi,step-size-mdB
     - 50
     - Step size of change in txAttenuation when a rising edge occurs on on
       either adi,increment-pin or adi,increment-pin. Range: 0 mdB to 1550 mdB,
       LSB = 50 mdB
   * - adi,increment-pin
     - NA
     - When a rising edge occurs on this GPIO pin, txAttenuation will increment
       by stepSize_mdB. Valid range from 0 to 15. Mandatory property in this
       node.
   * - adi,decrement-pin
     - NA
     - When a rising edge occurs on this GPIO pin, txAttenuation will decrement
       by adi,step-size-mdB. Valid range from 0 to 15. Mandatory property in
       this node.

GPIO Settings
~~~~~~~~~~~~~

All the gpios configurations should be done under adi,gpios node as shown in the
next example:

::

   adi,gpios {
           #address-cells = <1>;
       #size-cells = <0>;

           gpio@0 {
                   reg = <0>;
                   adi,signal = <2>;
                   adi,polarity = <1>;
                   adi,master = <2>;
           };
   };

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Default
     - Description
   * - reg
     - NA
     - Gpio number. Mandatory property in this node.
   * - adi,signal
     - NA
     - Defines the function of this gpio. Valid range from 0 to 19. Mandatory
       property in this node.
   * - adi,polarity
     - 0
     - Polarity of the GPIO pin (normal or inverted). 0 - normal, 1 - inverted
   * - adi,master
     - 0
     - Whether BBIC or ADRV9001 controls this pin. 0 - BBIC, 2 - ADRV9002

.. warning::

   When configuring GPIOs manually, make sure to not collide to any pin selected
   in another node configuration as adi,increment-pin, adi,decrement-pin,
   adi,agc-power-feedback-high-thres-exceeded, etc…
