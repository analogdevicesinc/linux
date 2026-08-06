# Asynchronous Sample Rate Converter (ASRC)

<!-- source: 308_Asynchronous_Sample_Rate_Converter_ASRC.pdf | original pages 2912–2937 -->

## 34   Asynchronous Sample Rate Converter (ASRC)

Sample rate converters (SRC) are frequently used in digital signal processing audio applications. The most frequently used sample rate conversions are off-loaded into hardware modules that are dedicated for filter processing and reduce the instruction processing load on the core, freeing it up for other tasks.

## Features

The ASRC has the following features and capabilities:

- 4 asynchronous stereo SRCs operating in completer mode are available in each DAI.
- Simple programming model
- Controllable muting options (hardware, software and automatic)
- Automatically senses input and output sample frequencies
- Supports left-justified, I 2 S, right-justified (16-,18-, 20-, 24-bits), and TDM serial port modes
- Daisy-chain configuration in TDM modes (including between DAI0 and DAI1) for input and output ports to create a serial frame
- Different protocols on input/output port allow format conversions
- De-emphasis filter for 32, 44.1 and 48 kHz sampling frequencies
- Up to 192 kHz sample rate input/output continuous sample ratios from 7.5:1 to 1:8
- Group delay (latency of interpolation filter) is 16 samples
- SNR from 128 to 140 dB (depending on processor model)
- Matched phase mode available to compensate for group delays
- Can be used to de-jitter clocks in systems

## Functional Description

Conceptually, the sample rate converter interpolates the serial input data at a rate of 220 and samples the interpolated data stream by the output sample rate. In practice, a 64-tap FIR filter with 220 polyphases, a FIFO, a digital servo loop that measures the time difference between the input and output samples within 5 ps, and a digital circuit to track the sample rate ratio are used to perform the interpolation and output sampling.

## ADSP-SC59x ASRC Register List

Sample Rate Converter Module

Table 34-1: ADSP-SC59x ASRC Register List

| Name       | Description                       |
|------------|-----------------------------------|
| ASRC_CTL01 | Control Register for ASRC 0 and 1 |
| ASRC_CTL23 | Control Register for ASRC 2 and 3 |
| ASRC_MUTE  | Mute Register                     |
| ASRC_RAT01 | Ratio Register for ASRC 0 and 1   |
| ASRC_RAT23 | Ratio Register for ASRC 2 and 3   |

## ASRC Interrupt List

The ASRC interrupts are controlled through the DAI.

Table 34-2: ASRC Interrupt List

|   Interrupt ID | Interrupt Name   | Interrupt Condition     |
|----------------|------------------|-------------------------|
|             20 | DAI0_IRQH        | ASRC initialization     |
|             21 | DAI1_IRQH        | ASRC sample rate change |
|            116 | DAI0_IRQL        |                         |
|            117 | DAI1_IRQL        |                         |

## ASRC Block Diagram

The ASRC Block Diagram figure shows a top level block diagram of the ASRC module and the Core Architecture figure shows architecture details.

Figure 34-1: ASRC Block Diagram

<!-- image -->

Figure 34-2: ASRC Core Architecture

<!-- image -->

## SRU Programming

The SRU (signal routing unit) needs to be programmed to connect the ASRCs to the output pins or any other peripherals. For more information, see the Digital Audio Interface (DAI) chapter.

## Clocking

The ASRC module is in the SCLK0 clock domain. An internal divided version of the SCLK0 clock is generated and used as the fundamental clock for the ASRC module.

## I/O Ports

The I/O ports provide the interface through which data is transferred asynchronously into and out of the SRC modules. The SRC has a 3-wire interface for the serial input and output ports that supports left-justified, I 2 S, and rightjustified (16-, 18-, 20-, 24-bit) modes. Additionally, the serial interfaces support TDM mode for daisy-chaining multiple SRCs to form a frame. The serial output data is dithered down to 20, 18, or 16 bits when 20-, 18-, or 16bit output data is selected.

NOTE: The SRC converts the data from the serial input port to the sample rate of the serial output port. The sample rate at the serial input port can be asynchronous with respect to the output sample rate of the output serial port.

## De-Emphasis Filter

The de-emphasis filter is used to de-emphasize audio data that has been emphasized.

## Mute Control

When either the SRC starts up (or there is a change in sample ratio), the mute out signal ( SRCx\_MUTEOUT ) is asserted (=1). The mute out signal stays high until the SRC settles on the new sample rates. While mute out is asserted high, the mute in signal should be asserted high as well. The mute in signal performs a soft mute of the audio input data when asserted and un-mutes the input audio data softly when deasserted.

Note that it takes 4096 input port FS samples until the audio input data is completely muted and 4096 FS samples until the audio input data is completely unmuted.

## SRC Core

As shown in the ASRC Core Architecture figure, the sample rate converter's RAM FIFO block adjusts the left and right input samples and stores them for the FIR filter's convolution cycle. The ASRCx\_FS\_IP counter provides the write address (for scaling) to the FIFO block and the ramp input to the digital-servo loop. The ROM stores the coefficients for the FIR filter convolution and performs a high-order interpolation between the stored coefficients. The sample rate ratio block measures the sample rate by dynamically altering the ROM coefficients and scaling the FIR filter length and input data. The digital-servo loop automatically tracks the SRCx\_FS\_IP and SRCx\_FS\_OP sample rates and provides the RAM and ROM start addresses for the start of the FIR filter convolution.

- NOTE: Unlike other peripherals, the sample rate converters own local memories (RAM and ROM) which are dedicated for the purpose of sample rate conversion only.

The sample rate converter only operates asynchronously and is always a completer to the input and output ports.

## RAM FIFO

The RAM FIFO receives the left and right input data and adjusts the amplitude of the data for both the soft muting of the SRC and the scaling of the input data by the sample rate ratio before storing the samples in RAM. The input data is scaled by the sample rate ratio because as the FIR filter length of the convolution increases, so does the

amplitude of the convolution output. To keep the output of the FIR filter from saturating, the input data is scaled down by multiplying it by ( SRCx\_FS\_OP )/( SRCx\_FS\_IP ) when SRCx\_FS\_OP &lt; SRCx\_FS\_IP . The FIFO also scales the input data to mute and stop muting the SRC.

## Digital Servo Loop

The digital-servo loop is a ramp filter that provides the initial pointer to the address in RAM and ROM for the start of the FIR convolution. The RAM pointer is the integer output of the ramp filter while the ROM pointer is the fractional part. The digital-servo loop must be able to provide excellent rejection of jitter on the SRCx\_FS\_IP and SRCx\_FS\_OP clocks as well as measure the arrival of the SRCx\_FS\_OP clock within 5 ps. The digital-servo loop also divides the fractional part of the ramp output by the ratio of ( SRCx\_FS\_IP )/( SRCx\_FS\_OP ) for the case when SRCx\_FS\_IP &gt; SRCx\_FS\_OP , to dynamically alter the ROM coefficients.

The digital-servo loop is implemented with a multi-rate filter. To settle the digital-servo loop filter quickly at startup or at a change in the sample rate, a fast mode has been added to the filter. When the digital-servo loop starts up or the sample rate is changed, the digital-servo loop kicks into fast mode to adjust and settle on the new sample rate. Upon sensing the digital-servo loop settling down to some reasonable value, the digital-servo loop kicks into normal or slow mode. During fast mode, the SRCx\_MUTE\_OUT bit of the ASRC is asserted to mute the ASRC input which avoids clicks and pops.

## FIR Filter

The FIR filter is a 64-tap filter in the case of SRCx\_FS\_OP&lt;SRCx\_FS\_IP and is ( SRCx\_FS\_IP )/ ( SRCx\_FS\_OP ) × 64 taps for the case when SRCx\_FS\_IP&gt;SRCx\_FS\_OP . The FIR filter performs its convolution by loading in the starting address of the RAM address pointer and the ROM address pointer from the digitalservo loop at the start of the SRCx\_FS\_OP period. The FIR filter then steps through the RAM by decrementing its address by 1 for each tap, and the ROM pointer increments its address by the ( SRCx\_FS\_OP / SRCx\_FS\_IP ) × 2 20 ratio for SRCx\_FS\_IP&gt;SRCx\_FS\_OP or 2 20  for SRCx\_FS\_OP &lt;SRCx\_FS\_IP . Once the ROM address rolls over, the convolution is complete. The convolution is performed for both the left and right channels, and the multiply/accumulate circuit used for the convolution is shared between the channels.

## Sample Rate Sensing

The ( SRCx\_FS\_IP )/( SRCx\_FS\_OP ) sample rate ratio circuit is used to dynamically alter the coefficients in the ROM for the case when SRCx\_FS\_IP&gt;SRCx\_FS\_OP . The ratio is calculated by comparing the output of an SRCx\_FS\_OP counter to the output of an SRCx\_FS\_IP counter. If ASRCx\_FS\_OP&gt;SRCx\_FS\_IP , the ratio is held at one. If SRCx\_FS\_IP&gt;SRCx\_FS\_OP , the sample rate ratio is updated if it is different by more than two SRCx\_FS\_OP periods from the previous SRCx\_FS\_OP to SRCx\_FS\_IP comparison. This is done to provide some hysteresis to prevent the filter length from oscillating and causing distortion.

## Digital Filter Group Delay

The RAM in the FIFO is 512 words deep for both left and right channels. An offset of 16 samples to the write address, provided by the SRCx\_FS\_IP counter, is added to prevent the RAM read pointer from overlapping the write address. The maximum decimation rate can be calculated from the RAM word: depth = (512 - 16) ÷ 64 taps = 7.75:1.

The 64 samples effect latency in the interpolation filter. This latency (group delay) depends on interpolation or decimation ratio and is determined as follows:

Interpolation or Decimation Ratio (1): GDL = 16/f S\_IN  + 32/f S\_IN  seconds for SRC\_FS\_OP &gt; SRC\_FS\_IP

Interpolation or Decimation Ratio (2): GDL = 16/f S\_IN  + 32/f S\_IN  × f S\_IN /f S\_OUT seconds for SRC\_FS\_OP &lt; SRC\_FS\_IP

## Data Format

The ASRC Data Frame Format by Protocol figure shows the data input format for a frame (stereo data). The frame format is valid for all protocols. For models which do not support matched phase mode the 8-bit data field is ignored.

Figure 34-3: ASRC Data Frame Format by Protocol

<!-- image -->

| LEFT-JUSTIFIED, I 2 S, AND TDM MODES   | LEFT-JUSTIFIED, I 2 S, AND TDM MODES       | LEFT-JUSTIFIED, I 2 S, AND TDM MODES       | LEFT-JUSTIFIED, I 2 S, AND TDM MODES   | LEFT-JUSTIFIED, I 2 S, AND TDM MODES        | LEFT-JUSTIFIED, I 2 S, AND TDM MODES        |
|----------------------------------------|--------------------------------------------|--------------------------------------------|----------------------------------------|---------------------------------------------|---------------------------------------------|
| AUDIO DATA LEFT CHANNEL, 24 BITS       | AUDIO DATA LEFT CHANNEL, 24 BITS           | MATCHED-PHASE DATA, 8 BITS                 | AUDIO DATA RIGHT CHANNEL, 24 BITS      | AUDIO DATA RIGHT CHANNEL, 24 BITS           | MATCHED-PHASE DATA, 8 BITS                  |
| MSB (BIT-63)                           | MSB (BIT-63)                               | MSB (BIT-63)                               | MSB (BIT-63)                           | MSB (BIT-63)                                | LSB (BIT-0)                                 |
| RIGHT-JUSTIFIED MODE                   | RIGHT-JUSTIFIED MODE                       | RIGHT-JUSTIFIED MODE                       | RIGHT-JUSTIFIED MODE                   | RIGHT-JUSTIFIED MODE                        | RIGHT-JUSTIFIED MODE                        |
| MATCHED-PHASE DATA, 8 BITS             | AUDIO DATA LEFT CHANNEL, 16 BITS - 24 BITS | AUDIO DATA LEFT CHANNEL, 16 BITS - 24 BITS | MATCHED-PHASE DATA, 8 BITS             | AUDIO DATA RIGHT CHANNEL, 16 BITS - 24 BITS | AUDIO DATA RIGHT CHANNEL, 16 BITS - 24 BITS |

## Operating Modes

The ASRC can operate in TDM, I 2 S, left-justified, right-justified, and bypass modes. The serial ports of the processor can be used for moving the ASRC data to/from the internal memory.

In I 2 S, left-justified and right-justified modes, the ASRCs operate individually. The serial data provided in the input port is converted to the sample rate of the output port.

## TDM Input Mode

In TDM input port, several ASRCs can be daisy-chained together and connected to the serial input port of a SHARC processor or other processor (see the TDM Input/Output Modes figure). The ASRC IP contains a 64-bit parallel load shift register. When the SRCx\_FS\_IP\_I pulse arrives, each ASRC parallel loads its left and right data into the 64-bit shift register. The input to the shift register is connected to SRCx\_DATA\_IP\_I , while the output is connected to SRCx\_TDM\_IP\_O . By connecting the SRCx\_TDM\_IP\_O to the SRCx\_DATA\_IP\_I of the next ASRC, a large shift register is created, which is clocked by SRCx\_CLK\_IP\_I .

NOTE: In TDM mode, the ASRC drives at the rising edge and samples at the falling edge of the serial clock. In all other modes, the serial clock rising edge is the sampling edge, and the falling edge is the driving edge.

NOTE: The number of ASRCs that can be daisy-chained together is limited by the maximum frequency of SRCx\_CLK\_xx\_I , refer to the data sheet for exact values. For example, if the maximum frequency of

SRCx\_CLK\_xx\_I is x MHz, and the output sample rate is fS , then number of ASRCs (n) that can be connected in daisy chained fashion is: n 64 FS ≤ x MHz.

## TDM Output Mode

As shown in the TDM Input/Output Modes figure, using the TDM output port several ASRCs can be daisy-chained together and connected to the SPORT of this or another processor. The ASRC OP contains a 64-bit parallel load shift register. When the SRCx\_FS\_OP\_I pulse arrives, each ASRC loads its left and right data into the 64-bit shift register. The input to the shift register is connected to SRCx\_TDM\_OP\_I , and the output is connected to SRCx\_DAT\_OP\_O . By connecting the SRCx\_DAT\_OP\_O to the SRCx\_TDM\_OP\_I of the next ASRC, a large shift register is created, which is clocked by SRCx\_CLK\_OP\_I .

As shown in TDM Input/Output Modes , with three ASRCs in a daisy-chain connection, the serial clock for input/or output port is defined as: SYSCLK = 3 × 64 FS = 192 FS.

Figure 34-4: TDM Input/Output Modes

<!-- image -->

## Matched-Phase Mode

The matched-phase mode of the sample rate converter, shown in Typical Configuration for Matched-Phase Mode Operation , is enabled by the ASRC\_CTL01.MPHASE0 , ASRC\_CTL01.MPHASE1 , ASRC\_CTL23.MPHASE2 and ASRC\_CTL23.MPHASE3 bits. This mode is used to match the phase (group delay) between two or more adjacent sample rate converters that are operating with the same input and output clocks.

Figure 34-5: Typical Configuration for Matched-Phase Mode Operation

<!-- image -->

Hysteresis of the ( SRCx\_FS\_OP )/( SRCx\_FS\_IP ) ratio circuit can cause phase mismatching between two ASRCs operating with the same input and output clocks. Since the hysteresis requires a difference of more than two SRCx\_FS\_OP periods to update the SRCx\_FS\_OP and SRCx\_FS\_IP ratios, two ASRCs may have differences in their ratios from 0 to 4 SRCx\_FS\_OP period counts. The ( SRCx\_FS\_OP )/( SRCx\_FS\_IP ) ratio adjusts the filter length of the ASRC, which corresponds directly with the group delay. Thus, the magnitude in the phase difference depends upon the resolution of the SRCx\_FS\_OP and SRCx\_FS\_IP counters. The greater the resolution of the counters, the smaller the phase difference error.

When the completer SRC matched-phase mode bit is set (=1), it accepts the sample rate ratio transmitted by another SRC, (the matched-phase requester) which has its matched-phase mode bit cleared (=0), through its serial output.

The phase requester ASRC device transmits its SRCx\_FS\_OP / SRCx\_FS\_IP ratio through the data output pin ( SRCx\_DAT\_OP\_O ) to the completer's ASRC's data input pins ( SRCx\_TDM\_OP\_I ). The transmitted data (32bit subframe) contains 24-bit data and 8-bits matched phase (see the ASRC Data Frame Format by Protocol figure).

The completer SRCs receive the 8-bit matched phase bits (instead of their own internally-derived ratio) if their SRCx\_MPHASE bits are set to 1, respectively. The SRCx\_FS\_IP and SRCx\_FS\_OP signals may be asynchronous with respect to each other in this mode. Note that there must be 64 SRCx\_CLK\_OP cycles per frame in matched-phase mode (two 24-bits data and two 8-bits phase match).

NOTE: By default, matched phased data is sent on the SRCx\_DAT\_OP\_O pin, but only if the SRCx\_TDM\_OP\_I pin is tied low. The completers' simply ignore the matched phased data if their ASRC\_CTL01.MPHASE1 through ASRC\_CTL23.MPHASE3 bits are cleared (= 0).

## Bypass Mode

When the ASRC\_CTL01.BYP0 , ASRC\_CTL01.BYP1 , ASRC\_CTL23.BYP2 and ASRC\_CTL23.BYP3 bits are set (=1), the input data bypasses the sample rate converter and is sent directly to the serial output port. Dithering is disabled. This mode is ideal when the input and output sample rates are the same and the SRCx\_FS\_IP\_I and

SRCx\_FS\_OP\_I signals are synchronous with respect to each other. In matched phase bypass mode, the SRCx\_FS\_OP\_I signal should come at least one SRCx\_CLK\_xx\_I period before SRCx\_FS\_IP\_I . Cases where this is not met could result in data loss. For example, if internal SPORTS are used then the SRCx\_FS\_OP\_I and SRCx\_FS\_IP\_I signals could be driven by different SPORTs so that the timing of these signals can be controlled by enabling them at different times. This mode can also be used for passing through non-audio data since no processing is performed on the input data.

## De-Emphasis Mode

The ASRC\_CTL01.DEEMPHASIS0 , ASRC\_CTL01.DEEMPHASIS1 , ASRC\_CTL23.DEEMPHASIS2 and ASRC\_CTL23.DEEMPHASIS3 bits choose the type of de-emphasis filter based on the input sample rate for 32, 44.1 or 48 kHz sampling rates.

## Dithering Mode

The ASRC\_CTL01.DITHER0 , ASRC\_CTL01.DITHER1 , ASRC\_CTL23.DITHER2 , and ASRC\_CTL23.DITHER3 control this mode of operation. Serial output data is dithered down to 20, 18, or 16 bits when 20-, 18-, or 16-bit output data is selected. In the case of 20-, 18-, and 16-bit word lengths, the least significant bits of the 24-bit word coming from the SRC into the serial output port are truncated. The DITHER\_EN signal (not user configurable) automatically adds dithering to the 24-bit word before truncating to the appropriate output word length. The 21BIT\_DITHER signal is used for the consumer version of the SRC to reduce the dynamic range performance to approximately 128 dB.

NOTE: The ASRC can be programmed to add the triangular Probability Distribution Function (PDF) dither to the digital audio samples. It is advisable to add dither when the input word width exceeds the output word width, for example the input word is 20 bits, and the output word is 16 bits. A triangular PDF is considered to create the most favorable noise shaping of the residual quantization noise.

## Muting Modes

The mute feature of the ASRC can be controlled automatically in hardware using the MUTE\_IN signal by connecting it to the MUTE\_OUT signal. Automatic muting can be disabled by setting (=1) the ASRC\_MUTE.MUTE0 through ASRC\_MUTE.MUTE3 bits.

NOTE: Note that by default, the ASRC\_MUTE register connects the MUTE\_IN signal to the MUTE\_OUT signal, but not conversely.

## Soft Mute

When the ASRC\_CTL01.SOFTMUTE0 , ASRC\_CTL01.SOFTMUTE1 , ASRC\_CTL23.SOFTMUTE2 and ASRC\_CTL23.SOFTMUTE3 bits are set, the MUTE\_IN signal is asserted, and the ASRC performs a soft mute by linearly decreasing the input data to the ASRC FIFO to zero, (-144 dB) attenuation as described for automatic hardware muting.

A 12-bit counter, clocked by SRCx\_FS\_IP\_I , is used to control the mute attenuation. Therefore, the time it takes from the assertion of the MUTE\_IN signal to -144 dB, full mute attenuation is 4096 FS cycles. Likewise, the time it takes to reach 0 dB mute attenuation from the deassertion of the MUTE\_IN signal is 4096 FS cycles.

## Hard Mute

When the ASRC\_CTL01.HARDMUTE0 , ASRC\_CTL01.HARDMUTE1 , ASRC\_CTL23.HARDMUTE2 and ASRC\_CTL23.HARDMUTE3 bits are set, the ASRC immediately mutes the input data to the ASRC FIFO to zero, (-144 dB) attenuation.

## Auto Mute

When the ASRC\_CTL01.AUTOMUTE0 , ASRC\_CTL01.AUTOMUTE1 , ASRC\_CTL23.AUTOMUTE2 and ASRC\_CTL23.AUTOMUTE3 bits are set, the ASRC communicates with the S/PDIF receiver peripheral to determine when the input should mute.

This mode is useful for automatic detection of non-PCM audio data received from the S/PDIF receiver.

## Interrupts

The following sections provide information about interrupt sources, masking, and servicing.

## Sources

Each ASRC module drives one interrupt signal (mute out asserted). All these signals are connected into the DAI\_IRPTL\_H or DAI\_IRPTL\_L latch registers. The ASRC ports generate interrupts as described below.

## SRC Mute Out

The SRC mute out signal can be used to generate interrupts on their rising edge, falling edge, or both, depending on how the DAI interrupt mask registers ( DAI\_IMSK\_RE / DAI\_IMSK\_FE ) are programmed. This programming allows the generation of DAI\_IRPTL\_H / DAI\_IRPTL\_L interrupts either entering mute, exiting muting or both. The SRCx\_MUTE\_OUT interrupt is generated only once when the SRC is locked (after 4096 FS input samples) and after changes to the sample ratio. Hard mute, soft mute, and auto mute only control the muting of the input data to the SRC.

## Masking

The DAI\_IMSK\_FE , DAI\_IMSK\_RE , and DAI\_IMSK\_PRI registers must be unmasked accordingly. The DAI\_IRQH and DAI\_IRQL signals are routed to the system event controller (SEC) and general interrupt controller (GIC).

## Service

The ISR reads the DAI\_IRPTL\_H and DAI\_IRPTL\_L registers to clear the interrupt request.

## Programming Model

The following is basic information on programming the ASRC module.

1. Program the ASRC\_CTL01 and ASRC\_CTL23 registers and keep the ASRC\_CTL01.EN0 through ASRC\_CTL23.EN3 bits cleared.
2. Set the ASRC\_CTL01.EN0 through ASRC\_CTL23.EN3 bits. After 4096 input port FS cycles the ASRC has un-muted.

## Debug Features

The asynchronous sample rate converter allow the bypass mode. When the ASRC\_CTL01.BYP0 through ASRC\_CTL23.BYP3 bits are set (=1), the input data bypasses the sample rate converter and is sent directly to the serial output port. This mode can be used for testing both ports when the input and output sample rates are at the same frequency, therefore both input and output ports can be routed to the same serial clock and frame sync.

## ADSP-SC59x ASRC Register Descriptions

Sample Rate Converter Module (ASRC) contains the following registers.

Table 34-3: ADSP-SC59x ASRC Register List

| Name       | Description                       |
|------------|-----------------------------------|
| ASRC_CTL01 | Control Register for ASRC 0 and 1 |
| ASRC_CTL23 | Control Register for ASRC 2 and 3 |
| ASRC_MUTE  | Mute Register                     |
| ASRC_RAT01 | Ratio Register for ASRC 0 and 1   |
| ASRC_RAT23 | Ratio Register for ASRC 2 and 3   |

## Control Register for ASRC 0 and 1

The ASRC\_CTL01 register (read/write) controls the operating modes, filters, and data formats used in the ASRC modules 0 and 1.

Figure 34-6: ASRC\_CTL01 Register Diagram

<!-- image -->

Table 34-4: ASRC\_CTL01 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | EN1        | Enable SRC 1. The ASRC_CTL01.EN1 bit enables SRC 1. When (set = 1), or when the sample rate (frame sync) between the input and output changes, the SRC begins its initialization routine where; 1) MUTE_OUT is asserted, 2) soft mute control counter for input samples is set to maximum attenuation (144 dB). Note that SRC power-up completion is finished by clearing the ASRC_RAT01.MUTEOUT1 bit. | Enable SRC 1. The ASRC_CTL01.EN1 bit enables SRC 1. When (set = 1), or when the sample rate (frame sync) between the input and output changes, the SRC begins its initialization routine where; 1) MUTE_OUT is asserted, 2) soft mute control counter for input samples is set to maximum attenuation (144 dB). Note that SRC power-up completion is finished by clearing the ASRC_RAT01.MUTEOUT1 bit. |
| 30 (R/W)           | MPHASE1    | a minimum of 5 SYSCLK cycles. Matched-phase Mode 1. The ASRC_CTL01.MPHASE1 bit configures SRC1 to not use its own internally- generated sample rate ratio but use an externally-generated ratio. Used with TDMda- ta.                                                                                                                                                                                  | a minimum of 5 SYSCLK cycles. Matched-phase Mode 1. The ASRC_CTL01.MPHASE1 bit configures SRC1 to not use its own internally- generated sample rate ratio but use an externally-generated ratio. Used with TDMda- ta.                                                                                                                                                                                  |
| 30 (R/W)           | MPHASE1    | 0                                                                                                                                                                                                                                                                                                                                                                                                      | Matched phase completer disabled                                                                                                                                                                                                                                                                                                                                                                       |
| 30 (R/W)           | MPHASE1    | 1                                                                                                                                                                                                                                                                                                                                                                                                      | Matched phase completer enabled                                                                                                                                                                                                                                                                                                                                                                        |
| 29:28 (R/W)        | LENOUT1    | Length Output 1. The ASRC_CTL01.LENOUT1 bit field selects the serial output word length on SRC1.                                                                                                                                                                                                                                                                                                       | Length Output 1. The ASRC_CTL01.LENOUT1 bit field selects the serial output word length on SRC1.                                                                                                                                                                                                                                                                                                       |
| 29:28 (R/W)        | LENOUT1    | 0                                                                                                                                                                                                                                                                                                                                                                                                      | 24 bits                                                                                                                                                                                                                                                                                                                                                                                                |
| 29:28 (R/W)        | LENOUT1    | 1                                                                                                                                                                                                                                                                                                                                                                                                      | 20 bits                                                                                                                                                                                                                                                                                                                                                                                                |
| 29:28 (R/W)        | LENOUT1    | 2                                                                                                                                                                                                                                                                                                                                                                                                      | 18 bits                                                                                                                                                                                                                                                                                                                                                                                                |
| 29:28 (R/W)        | LENOUT1    | 3                                                                                                                                                                                                                                                                                                                                                                                                      | 16 bits                                                                                                                                                                                                                                                                                                                                                                                                |
| 27:26 (R/W)        | SMODEOUT1  | Serial Mode Output 1. The ASRC_CTL01.SMODEOUT1 bit field selects the serial output format on SRC1.                                                                                                                                                                                                                                                                                                     | Serial Mode Output 1. The ASRC_CTL01.SMODEOUT1 bit field selects the serial output format on SRC1.                                                                                                                                                                                                                                                                                                     |
| 27:26 (R/W)        | SMODEOUT1  | 0                                                                                                                                                                                                                                                                                                                                                                                                      | Left-justified                                                                                                                                                                                                                                                                                                                                                                                         |
| 27:26 (R/W)        | SMODEOUT1  | 1                                                                                                                                                                                                                                                                                                                                                                                                      | I2S                                                                                                                                                                                                                                                                                                                                                                                                    |
| 27:26 (R/W)        | SMODEOUT1  | 2                                                                                                                                                                                                                                                                                                                                                                                                      | TDM                                                                                                                                                                                                                                                                                                                                                                                                    |
| 27:26 (R/W)        | SMODEOUT1  | 3                                                                                                                                                                                                                                                                                                                                                                                                      | Right-justified                                                                                                                                                                                                                                                                                                                                                                                        |
| 25 (R/W)           | DITHER1    | Dither Enable 1. The ASRC_CTL01.DITHER1 bit enables dithering before truncation on SRC1 when a word length less than 24 bits is selected.                                                                                                                                                                                                                                                              | Dither Enable 1. The ASRC_CTL01.DITHER1 bit enables dithering before truncation on SRC1 when a word length less than 24 bits is selected.                                                                                                                                                                                                                                                              |
| 25 (R/W)           | DITHER1    | 0                                                                                                                                                                                                                                                                                                                                                                                                      | Truncation only                                                                                                                                                                                                                                                                                                                                                                                        |
| 25 (R/W)           | DITHER1    | 1                                                                                                                                                                                                                                                                                                                                                                                                      | Dithering before truncation                                                                                                                                                                                                                                                                                                                                                                            |

Table 34-4: ASRC\_CTL01 Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                     |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | SOFTMUTE1   | Soft Mute 1. The ASRC_CTL01.SOFTMUTE1 bit enables soft mute on SRC1.                                                                                                                                        |
| 24 (R/W)           | SOFTMUTE1   | 0 Unmute                                                                                                                                                                                                    |
| 24 (R/W)           | SOFTMUTE1   | 1 Mute                                                                                                                                                                                                      |
| 23:22 (R/W)        | DEEMPHASIS1 | De-emphasize Audio 1. The ASRC_CTL01.DEEMPHASIS1 bits are used to de-emphasize audio data that has been emphasized. The type of de-emphasis filter is based on the input sample rate (SRCx_FS_IP_I signal). |
| 23:22 (R/W)        | DEEMPHASIS1 | 0 No de-emphasis                                                                                                                                                                                            |
| 23:22 (R/W)        | DEEMPHASIS1 | 1 32 kHz                                                                                                                                                                                                    |
| 23:22 (R/W)        | DEEMPHASIS1 | 2 44.1 kHz                                                                                                                                                                                                  |
| 23:22 (R/W)        | DEEMPHASIS1 | 3 48 kHz                                                                                                                                                                                                    |
| 21 (R/W)           | BYP1        | Bypass 1. The ASRC_CTL01.BYP1 bit makes the output of SRC1 the same as the input.                                                                                                                           |
| 20:18 (R/W)        | SMODEIN1    | Serial Mode Input 1. The ASRC_CTL01.SMODEIN1 bit field selects the serial input format for SRC1.                                                                                                            |
| 20:18 (R/W)        | SMODEIN1    | 0 left-justified                                                                                                                                                                                            |
| 20:18 (R/W)        | SMODEIN1    | 1 I2S                                                                                                                                                                                                       |
| 20:18 (R/W)        | SMODEIN1    | 2 TDM                                                                                                                                                                                                       |
| 20:18 (R/W)        | SMODEIN1    | 4 24-bit right-justified                                                                                                                                                                                    |
| 20:18 (R/W)        | SMODEIN1    | 5 20-bit right-justified                                                                                                                                                                                    |
| 20:18 (R/W)        | SMODEIN1    | 6 18-bit right-justified                                                                                                                                                                                    |
| 20:18 (R/W)        | SMODEIN1    | 7 16-bit right-justified                                                                                                                                                                                    |
| 17 (R/W)           | AUTOMUTE1   | Auto Hard Mute 1. The ASRC_CTL01.AUTOMUTE1 bit auto hard mutes SRC1 when non audio is as- serted by the SPDIF receiver.                                                                                     |
| 17 (R/W)           | AUTOMUTE1   | 0 Unmute                                                                                                                                                                                                    |
| 17 (R/W)           | AUTOMUTE1   | 1 Mute                                                                                                                                                                                                      |
| 16 (R/W)           | HARDMUTE1   | Hard Mute 1. The ASRC_CTL01.HARDMUTE1 bit hard mutes SRC1.                                                                                                                                                  |
| 16 (R/W)           | HARDMUTE1   | 0 Unmute                                                                                                                                                                                                    |
| 16 (R/W)           | HARDMUTE1   | 1 Mute                                                                                                                                                                                                      |

Table 34-4: ASRC\_CTL01 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | EN0        | Enable SRC 0. The ASRC_CTL01.EN0 bit enables SRC 0. When (set =1), or when the sample rate (frame sync) between the input and output changes, the SRC begins its initialization routine where; 1) MUTE_OUT is asserted, 2) soft mute control counter for input samples is set to maximum attenuation (144 dB). Note that SRC power-up completion is finished by clearing the ASRC_RAT01.MUTEOUT0 bit. Writes to the ASRC_CTL01 register should be at least one cycle before setting the ASRC_CTL01.EN0 bit. When setting and clearing this bit, it should be held low for a minimum of 5 CLK cycles. |
| 14 (R/W)           | MPHASE0    | Matched-phase Mode 0. The ASRC_CTL01.MPHASE0 bit configures SRC0 to not use its own internally- generated sample rate ratio but use an externally-generated ratio. Used with TDMda- ta.                                                                                                                                                                                                                                                                                                                                                                                                              |
| 13:12 (R/W)        | LENOUT0    | Length Output 0. The ASRC_CTL01.LENOUT0 bit field selects the serial output word length on SRC0.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 11:10 (R/W)        | SMODEOUT0  | 3 16 bits Serial Mode Output 0. The ASRC_CTL01.SMODEOUT0 bit field selects the serial output format on SRC0. 0 Left-justified 1 I2S 2 TDM 3 Right-justified                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 9 (R/W)            | DITHER0    | Dither Enable 0. The ASRC_CTL01.DITHER0 bit enables dithering before truncation on SRC0 when a word length less than 24 bits is selected. 0 Truncation only                                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 34-4: ASRC\_CTL01 Register Fields (Continued)

| Bit No. (Access)   | Bit Name                    | Description/Enumeration                                                                                                                                                                                     |
|--------------------|-----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | SOFTMUTE0                   | Soft Mute 0. The ASRC_CTL01.SOFTMUTE0 bit enables soft mute on SRC0.                                                                                                                                        |
| 7:6 (R/W)          | DEEMPHASIS0                 | De-emphasize Audio 0. The ASRC_CTL01.DEEMPHASIS0 bits are used to de-emphasize audio data that has been emphasized. The type of de-emphasis filter is based on the input sample rate (SRCx_FS_IP_I signal). |
| 5 (R/W)            | BYP0                        | Bypass 0. The ASRC_CTL01.BYP0 bit makes the output of SRC0 the same as the input.                                                                                                                           |
| 4:2 (R/W)          | Serial Mode Input 0.        |                                                                                                                                                                                                             |
|                    | SMODEIN0                    | The ASRC_CTL01.SMODEIN0 bit field selects the serial input format for SRC0. 0 left-justified 1 I2S 2 TDM                                                                                                    |
| 1 (R/W)            | AUTOMUTE0 Auto Hard Mute 0. | 4 24-bit right-justified 5 20-bit right-justified 6 18-bit right-justified 7 16-bit right-justified                                                                                                         |
|                    | HARDMUTE0                   | The ASRC_CTL01.AUTOMUTE0 bit auto hard mutes SRC0 when non audio is as- serted by the SPDIF receiver. 0 Unmute 1 Mute Hard Mute 0. The ASRC_CTL01.HARDMUTE0 bit hard mutes SRC0.                            |
| 0 (R/W)            |                             |                                                                                                                                                                                                             |
|                    | 0                           | Unmute                                                                                                                                                                                                      |
|                    | 1 Mute                      |                                                                                                                                                                                                             |
|                    |                             | (default)                                                                                                                                                                                                   |

## Control Register for ASRC 2 and 3

The ASRC\_CTL23 register (read/write) controls the operating modes, filters, and data formats used in the sample rate converter modules 2 and 3.

Figure 34-7: ASRC\_CTL23 Register Diagram

<!-- image -->

Table 34-5: ASRC\_CTL23 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | EN3        | Enable SRC 3. The ASRC_CTL23.EN3 bit enables SRC 3. When (set =1), or when the sample rate (frame sync) between the input and output changes, the SRC begins its initialization routine where; 1) MUTE_OUT is asserted, 2) soft mute control counter for input samples is set to maximum attenuation (144 dB). Note that SRC power-up completion is finished by clearing the ASRC_RAT23.MUTEOUT3 bit. | Enable SRC 3. The ASRC_CTL23.EN3 bit enables SRC 3. When (set =1), or when the sample rate (frame sync) between the input and output changes, the SRC begins its initialization routine where; 1) MUTE_OUT is asserted, 2) soft mute control counter for input samples is set to maximum attenuation (144 dB). Note that SRC power-up completion is finished by clearing the ASRC_RAT23.MUTEOUT3 bit. |
| 30 (R/W)           | MPHASE3    | Matched-phase Mode 3. The ASRC_CTL23.MPHASE3 bit configures SRC3 to not use its own internally- generated sample rate ratio but use an externally-generated ratio. Used with TDMda- ta.                                                                                                                                                                                                               | Matched-phase Mode 3. The ASRC_CTL23.MPHASE3 bit configures SRC3 to not use its own internally- generated sample rate ratio but use an externally-generated ratio. Used with TDMda- ta.                                                                                                                                                                                                               |
| 30 (R/W)           | MPHASE3    | 0                                                                                                                                                                                                                                                                                                                                                                                                     | Matched phase completer disabled                                                                                                                                                                                                                                                                                                                                                                      |
| 30 (R/W)           | MPHASE3    | 1                                                                                                                                                                                                                                                                                                                                                                                                     | Matched phase completer enabled                                                                                                                                                                                                                                                                                                                                                                       |
| 29:28 (R/W)        | LENOUT3    | Length Output 3. The ASRC_CTL23.LENOUT3 bit field selects the serial output word length on SRC3.                                                                                                                                                                                                                                                                                                      | Length Output 3. The ASRC_CTL23.LENOUT3 bit field selects the serial output word length on SRC3.                                                                                                                                                                                                                                                                                                      |
| 29:28 (R/W)        | LENOUT3    | 0                                                                                                                                                                                                                                                                                                                                                                                                     | 24 bits                                                                                                                                                                                                                                                                                                                                                                                               |
| 29:28 (R/W)        | LENOUT3    | 1                                                                                                                                                                                                                                                                                                                                                                                                     | 20 bits                                                                                                                                                                                                                                                                                                                                                                                               |
| 29:28 (R/W)        | LENOUT3    | 2                                                                                                                                                                                                                                                                                                                                                                                                     | 18 bits                                                                                                                                                                                                                                                                                                                                                                                               |
| 29:28 (R/W)        | LENOUT3    | 3                                                                                                                                                                                                                                                                                                                                                                                                     | 16 bits                                                                                                                                                                                                                                                                                                                                                                                               |
| 27:26 (R/W)        | SMODEOUT3  | Serial Mode Output 3. The ASRC_CTL23.SMODEOUT3 bit field selects the serial output format on SRC3.                                                                                                                                                                                                                                                                                                    | Serial Mode Output 3. The ASRC_CTL23.SMODEOUT3 bit field selects the serial output format on SRC3.                                                                                                                                                                                                                                                                                                    |
| 27:26 (R/W)        | SMODEOUT3  | 0                                                                                                                                                                                                                                                                                                                                                                                                     | Left-justified                                                                                                                                                                                                                                                                                                                                                                                        |
| 27:26 (R/W)        | SMODEOUT3  | 1                                                                                                                                                                                                                                                                                                                                                                                                     | I2S                                                                                                                                                                                                                                                                                                                                                                                                   |
| 27:26 (R/W)        | SMODEOUT3  | 2                                                                                                                                                                                                                                                                                                                                                                                                     | TDM                                                                                                                                                                                                                                                                                                                                                                                                   |
| 27:26 (R/W)        | SMODEOUT3  | 3                                                                                                                                                                                                                                                                                                                                                                                                     | Right-justified                                                                                                                                                                                                                                                                                                                                                                                       |
| 25 (R/W)           | DITHER3    | Dither Enable 3. The ASRC_CTL23.DITHER3 bit enables dithering before truncation on SRC3 when a word length less than 24 bits is selected.                                                                                                                                                                                                                                                             | Dither Enable 3. The ASRC_CTL23.DITHER3 bit enables dithering before truncation on SRC3 when a word length less than 24 bits is selected.                                                                                                                                                                                                                                                             |
| 25 (R/W)           | DITHER3    | 0                                                                                                                                                                                                                                                                                                                                                                                                     | Truncation only                                                                                                                                                                                                                                                                                                                                                                                       |
| 25 (R/W)           | DITHER3    | 1                                                                                                                                                                                                                                                                                                                                                                                                     | Dithering before truncation                                                                                                                                                                                                                                                                                                                                                                           |

Table 34-5: ASRC\_CTL23 Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                     |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | SOFTMUTE3   | Soft Mute 3. The ASRC_CTL23.SOFTMUTE3 bit enables soft mute on SRC3.                                                                                                                                        |
| 24 (R/W)           | SOFTMUTE3   | 0 Unmute                                                                                                                                                                                                    |
| 24 (R/W)           | SOFTMUTE3   | 1 Mute                                                                                                                                                                                                      |
| 23:22 (R/W)        | DEEMPHASIS3 | De-emphasize Audio 3. The ASRC_CTL23.DEEMPHASIS3 bits are used to de-emphasize audio data that has been emphasized. The type of de-emphasis filter is based on the input sample rate (SRCx_FS_IP_I signal). |
| 23:22 (R/W)        | DEEMPHASIS3 | 0 No de-emphasis                                                                                                                                                                                            |
| 23:22 (R/W)        | DEEMPHASIS3 | 1 32 kHz                                                                                                                                                                                                    |
| 23:22 (R/W)        | DEEMPHASIS3 | 2 44.1 kHz                                                                                                                                                                                                  |
| 23:22 (R/W)        | DEEMPHASIS3 | 3 48 kHz                                                                                                                                                                                                    |
| 21 (R/W)           | BYP3        | Bypass 3. The ASRC_CTL23.BYP3 bit makes the output of SRC3 the same as the input.                                                                                                                           |
| 20:18 (R/W)        | SMODEIN3    | Serial Mode Input 3. The ASRC_CTL23.SMODEIN3 bit field selects the serial input format for SRC3.                                                                                                            |
| 20:18 (R/W)        | SMODEIN3    | 0 left-justified                                                                                                                                                                                            |
| 20:18 (R/W)        | SMODEIN3    | 1 I2S                                                                                                                                                                                                       |
| 20:18 (R/W)        | SMODEIN3    | 2 TDM                                                                                                                                                                                                       |
| 20:18 (R/W)        | SMODEIN3    | 4 24-bit right-justified                                                                                                                                                                                    |
| 20:18 (R/W)        | SMODEIN3    | 5 20-bit right-justified                                                                                                                                                                                    |
| 20:18 (R/W)        | SMODEIN3    | 6 18-bit right-justified                                                                                                                                                                                    |
| 20:18 (R/W)        | SMODEIN3    | 7 16-bit right-justified                                                                                                                                                                                    |
| 17 (R/W)           | AUTOMUTE3   | Auto Hard Mute 3. The ASRC_CTL23.AUTOMUTE3 bit auto hard mutes SRC3 when non audio is as- serted by the SPDIF receiver.                                                                                     |
| 17 (R/W)           | AUTOMUTE3   | 0 Unmute                                                                                                                                                                                                    |
| 17 (R/W)           | AUTOMUTE3   | 1 Mute                                                                                                                                                                                                      |
| 16 (R/W)           | HARDMUTE3   | Hard Mute 3. The ASRC_CTL23.HARDMUTE3 bit hard mutes SRC3.                                                                                                                                                  |
| 16 (R/W)           | HARDMUTE3   | 0 Unmute                                                                                                                                                                                                    |
| 16 (R/W)           | HARDMUTE3   | 1 Mute                                                                                                                                                                                                      |

Table 34-5: ASRC\_CTL23 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | EN2        | Enable SRC 2. The ASRC_CTL23.EN2 bit enables SRC 2. When (set =1), or when the sample rate (frame sync) between the input and output changes, the SRC begins its initialization routine where; 1) MUTE_OUT is asserted, 2) soft mute control counter for input samples is set to maximum attenuation (144 dB). Note that SRC power-up completion is finished by clearing the ASRC_RAT23.MUTEOUT2 bit. Writes to the ASRC_CTL23 register should be at least one cycle before setting the ASRC_CTL23.EN2 bit. When setting and clearing this bit, it should be held low for a minimum of 5 CLK cycles. |
| 14 (R/W)           | MPHASE2    | Matched-phase Mode 2. The ASRC_CTL23.MPHASE2 bit configures SRC2 to not use its own internally- generated sample rate ratio but use an externally-generated ratio. Used with TDMda- ta.                                                                                                                                                                                                                                                                                                                                                                                                              |
| 13:12 (R/W)        | LENOUT2    | Length Output 2. The ASRC_CTL23.LENOUT2 bit field selects the serial output word length on SRC2.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 11:10 (R/W)        | SMODEOUT2  | 3 16 bits Serial Mode Output 2. The ASRC_CTL23.SMODEOUT2 bit field selects the serial output format on SRC2. 0 Left-justified 1 I2S 2 TDM 3 Right-justified                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 9 (R/W)            | DITHER2    | Dither Enable 2. The ASRC_CTL23.DITHER2 bit enables dithering before truncation on SRC2 when a word length less than 24 bits is selected. 0 Truncation only                                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 34-5: ASRC\_CTL23 Register Fields (Continued)

| Bit No. (Access)   | Bit Name                    | Description/Enumeration                                                                                                                                                                                     |
|--------------------|-----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | SOFTMUTE2                   | Soft Mute 2. The ASRC_CTL23.SOFTMUTE2 bit enables soft mute on SRC2.                                                                                                                                        |
| 7:6 (R/W)          | DEEMPHASIS2                 | De-emphasize Audio 2. The ASRC_CTL23.DEEMPHASIS2 bits are used to de-emphasize audio data that has been emphasized. The type of de-emphasis filter is based on the input sample rate (SRCx_FS_IP_I signal). |
| 5 (R/W)            | BYP2                        | Bypass 2. The ASRC_CTL23.BYP2 bit makes the output of SRC2 the same as the input.                                                                                                                           |
| 4:2 (R/W)          | Serial Mode Input 2.        |                                                                                                                                                                                                             |
|                    | SMODEIN2                    | The ASRC_CTL23.SMODEIN2 bit field selects the serial input format for SRC2. 0 left-justified 1 I2S 2 TDM                                                                                                    |
| 1 (R/W)            | AUTOMUTE2 Auto Hard Mute 2. | 4 24-bit right-justified 5 20-bit right-justified 6 18-bit right-justified 7 16-bit right-justified                                                                                                         |
|                    | HARDMUTE2                   | The ASRC_CTL23.AUTOMUTE2 bit auto hard mutes SRC2 when non audio is as- serted by the SPDIF receiver. 0 Unmute 1 Mute Hard Mute 2.                                                                          |
| 0 (R/W)            |                             | ASRC_CTL23.HARDMUTE2 bit hard mutes SRC2.                                                                                                                                                                   |
|                    | The                         |                                                                                                                                                                                                             |
|                    | 0                           | Unmute                                                                                                                                                                                                      |
|                    | 1                           |                                                                                                                                                                                                             |
|                    |                             | Mute                                                                                                                                                                                                        |

## Mute Register

This register connects an ASRCx mute input and output when the mute bit is cleared (=0). This allows ASRCx to automatically mute input while the ASRC is initializing (0 = automatic muting and 1 = manual muting). Bit 0 controls ASRC0, bit 1 controls ASRC1, bit 2 controls ASRC2, and bit 3 controls ASRC3.

Figure 34-8: ASRC\_MUTE Register Diagram

<!-- image -->

Table 34-6: ASRC\_MUTE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------|
| 3 (R/W)            | MUTE3      | Mute ASRC3. The ASRC_MUTE.MUTE3 bit automatically mutes ASCR3 output when cleared (=0). |
| 2 (R/W)            | MUTE2      | Mute ASRC2. The ASRC_MUTE.MUTE2 bit automatically mutes ASCR2 output when cleared (=0). |
| 1 (R/W)            | MUTE1      | Mute ASRC1. The ASRC_MUTE.MUTE1 bit automatically mutes ASCR1 output when cleared (=0). |
| 0 (R/W)            | MUTE0      | Mute ASRC0. The ASRC_MUTE.MUTE0 bit automatically mutes ASCR0 output when cleared (=0). |

## Ratio Register for ASRC 0 and 1

The ASRC\_RAT01 register report the mute and I/O sample ratio for ASRC0 and ASRC1.

Figure 34-9: ASRC\_RAT01 Register Diagram

<!-- image -->

Table 34-7: ASRC\_RAT01 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | MUTEOUT1   | Mute Status for ASRC1. The ASRC_RAT01.MUTEOUT1 bit field reports the status of the MUTE_OUT sig- nal. Once the SRCx_MUTEOUT signal is cleared, the ratio can be read. When ASRC1 is enabled or there is a change in the sample ratio, the MUTE_OUT signal is asserted. The MUTE_OUT signal remains asserted until the digital servo loops internal fast set- tling mode is complete. When the digital servo loop has switched to slow settling mode, the MUTE_OUT signal is deasserted. |
| 30:16 (R/NW)       | RATIO1     | Sampling Ratio of Frame Syncs for ASRC1. The ASRC_RAT01.RATIO1 bit field is read to find the ratio of output to input sampling frequency for ASRC1 (SRCx_FS_OP_I/SRCx_FS_IP_I). This ratio is re- ported in 4.11 (integer.fraction) format where the 15-bit value of the normal binary number is comprised of 4 bits for the integer and 11 bits for the fraction.                                                                                                                      |
| 15 (R/NW)          | MUTEOUT0   | Mute Status for ASRC0. The ASRC_RAT01.MUTEOUT0 bit field reports the status of the MUTE_OUT sig- nal. Once the SRCx_MUTEOUT signal is cleared, the ratio can be read. When ASRC0 is enabled or there is a change in the sample ratio, the MUTE_OUT signal is asserted. The MUTE_OUT signal remains asserted until the digital servo loops internal fast set- tling mode is complete. When the digital servo loop has switched to slow settling mode, the MUTE_OUT signal is deasserted. |

Table 34-7: ASRC\_RAT01 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:0 (R/NW)        | RATIO0     | Sampling Ratio of Frame Syncs for ASRC0. The ASRC_RAT01.RATIO0 bit field is read to find the ratio of output to input sampling frequency for ASRC0 (SRCx_FS_OP_I/SRCx_FS_IP_I). This ratio is re- ported in 4.11 (integer.fraction) format where the 15-bit value of the normal binary number is comprised of 4 bits for the integer and 11 bits for the fraction. |

## Ratio Register for ASRC 2 and 3

The ASRC\_RAT23 register report the mute and I/O sample ratio for ASRC0 and ASRC1.

Figure 34-10: ASRC\_RAT23 Register Diagram

<!-- image -->

Table 34-8: ASRC\_RAT23 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | MUTEOUT3   | Mute Status for ASRC3. The ASRC_RAT23.MUTEOUT3 bit field reports the status of the MUTE_OUT sig- nal. Once the SRCx_MUTEOUT signal is cleared, the ratio can be read. When ASRC3 is enabled or there is a change in the sample ratio, the MUTE_OUT signal is asserted. The MUTE_OUT signal remains asserted until the digital servo loops internal fast set- tling mode is complete. When the digital servo loop has switched to slow settling mode, the MUTE_OUT signal is deasserted. |
| 30:16 (R/NW)       | RATIO3     | Sampling Ratio of Frame Syncs for ASRC3. The ASRC_RAT23.RATIO3 bit field is read to find the ratio of output to input sampling frequency for ASRC3 (SRCx_FS_OP_I/SRCx_FS_IP_I). This ratio is re- ported in 4.11 (integer.fraction) format where the 15-bit value of the normal binary number is comprised of 4 bits for the integer and 11 bits for the fraction.                                                                                                                      |
| 15 (R/NW)          | MUTEOUT2   | Mute Status for ASRC2. The ASRC_RAT23.MUTEOUT2 bit field reports the status of the MUTE_OUT sig- nal. Once the SRCx_MUTEOUT signal is cleared, the ratio can be read. When ASRC2 is enabled or there is a change in the sample ratio, the MUTE_OUT signal is asserted. The MUTE_OUT signal remains asserted until the digital servo loops internal fast set- tling mode is complete. When the digital servo loop has switched to slow settling mode, the MUTE_OUT signal is deasserted. |

Table 34-8: ASRC\_RAT23 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:0 (R/NW)        | RATIO2     | Sampling Ratio of Frame Syncs for ASRC2. The ASRC_RAT23.RATIO2 bit field is read to find the ratio of output to input sampling frequency for ASRC2 (SRCx_FS_OP_I/SRCx_FS_IP_I). This ratio is re- ported in 4.11 (integer.fraction) format where the 15-bit value of the normal binary number is comprised of 4 bits for the integer and 11 bits for the fraction. |