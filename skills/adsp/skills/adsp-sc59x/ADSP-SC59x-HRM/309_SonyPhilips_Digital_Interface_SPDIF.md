# Sony/Philips Digital Interface (S/PDIF)

<!-- source: 309_SonyPhilips_Digital_Interface_SPDIF.pdf | original pages 2938–2988 -->

## 35   Sony/Philips Digital Interface (S/PDIF)

The Sony/Philips Digital Interface (S/PDIF) is a standard audio data transfer format that allows the transfer of digital audio signals from one device to another without having to convert them to an analog signal. The digital audio interface carries three types of information; audio data, non-audio data (compressed data), and timing information.

## Features

The S/PDIF interface has the following features.

- Supports one stereo channel or compressed audio streams.
- AES3-compatible S/PDIF transmitter and receiver.
- Transmitting a biphase mark encoded signal that may contain any number of audio channels (compressed or linear pulse code modulation) or non-audio data.
- S/PDIF receiver managing clock recovery with separate S/PDIF on-chip PLL.
- S/PDIF receiver supports the detection of DTS frames of 256, 512, 1024, 2048, and 4096.
- Manage user status information and provide error-handling capabilities in both the transmitter and receiver.
- DAI allows interactions over DAI by serial ports and the external DAI pins to interface to other S/PDIF devices. This includes using the receiver to decode incoming biphase encoded audio streams and passing them via the SPORTs to internal memory for processing-or using the transmitter to encode audio or digital data and transfer it to another S/PDIF receiver in the audio system.

It is important to be familiar with serial digital audio interface standards IEC-60958, EIAJ CP-340, AES3 and AES11.

## ADSP-SC59x SPDIF Register List

The S/PDIF module is a standard audio data transfer format that allows the transfer of digital audio signals from one device to another without having to convert them to an analog signal. A set of registers govern S/PDIF operations. For more information on S/PDIF functionality, see the S/PDIF register descriptions.

Table 35-1: ADSP-SC59x SPDIF Register List

| Name              | Description                      |
|-------------------|----------------------------------|
| SPDIF_RX_CTL      | Receive Control                  |
| SPDIF_RX_STAT     | Receive Status Register          |
| SPDIF_RX_STAT0_A  | Receive Status A0 Register       |
| SPDIF_RX_STAT0_B  | Receive Status B0 Register       |
| SPDIF_RX_STAT1_A  | Receive Status A1 Register       |
| SPDIF_RX_STAT1_B  | Receive Status B1 Register       |
| SPDIF_TX_CTL      | Transmit Control Register        |
| SPDIF_TX_STAT_A0  | Transmit Status A0 Register      |
| SPDIF_TX_STAT_A1  | Transmit Status A1 Register      |
| SPDIF_TX_STAT_A2  | Transmit Status A2 Register      |
| SPDIF_TX_STAT_A3  | Transmit Status A3 Register      |
| SPDIF_TX_STAT_A4  | Transmit Status A4 Register      |
| SPDIF_TX_STAT_A5  | Transmit Status A5 Register      |
| SPDIF_TX_STAT_B0  | Transmit Status B0 Register      |
| SPDIF_TX_STAT_B1  | Transmit Status B1 Register      |
| SPDIF_TX_STAT_B2  | Transmit Status B2 Register      |
| SPDIF_TX_STAT_B3  | Transmit Status B3 Register      |
| SPDIF_TX_STAT_B4  | Transmit Status B4 Register      |
| SPDIF_TX_STAT_B5  | Transmit Status B5 Register      |
| SPDIF_TX_UBUFF_A0 | Transmit User Buffer A0 Register |
| SPDIF_TX_UBUFF_A1 | Transmit User Buffer A1 Register |
| SPDIF_TX_UBUFF_A2 | Transmit User Buffer A2 Register |
| SPDIF_TX_UBUFF_A3 | Transmit User Buffer A3 Register |
| SPDIF_TX_UBUFF_A4 | Transmit User Buffer A4 Register |
| SPDIF_TX_UBUFF_A5 | Transmit User Buffer A5 Register |
| SPDIF_TX_UBUFF_B0 | Transmit User Buffer B0 Register |
| SPDIF_TX_UBUFF_B1 | Transmit User Buffer B1 Register |
| SPDIF_TX_UBUFF_B2 | Transmit User Buffer B2 Register |
| SPDIF_TX_UBUFF_B3 | Transmit User Buffer B3 Register |
| SPDIF_TX_UBUFF_B4 | Transmit User Buffer B4 Register |
| SPDIF_TX_UBUFF_B5 | Transmit User Buffer B5 Register |

Table 35-1: ADSP-SC59x SPDIF Register List (Continued)

| Name             | Description              |
|------------------|--------------------------|
| SPDIF_TX_USRUPDT | User Bit Update Register |

## SRU Programming

The SRU (signal routing unit) is used to connect the S/PDIF transmitter biphase data out to the output pins or to the S/PDIF receiver. The serial clock, frame sync, data, and external sync (if external synchronization is required) inputs also need to be routed through the SRU. For details of the routing, see DAI Routing Capabilities in the Digital Audio Interface (DAI) chapter.

The SRU needs to be programmed in order to connect the S/PDIF receiver to the output pins or any other peripherals and also for the connection to the input biphase stream.

Program the corresponding SRU registers to connect the outputs to the required destinations (see DAI Routing Capabilities inputs to the receiver are routed through the SRU. The extracted clock, frame sync, and data are also routed through the SRU.

## S/PDIF Interrupt List

Table 35-2: S/PDIF Interrupt List

| Interrupt Name   | Interrupt Condition   | Return DAI Register   | Return SEC Register   |   SEC ID |
|------------------|-----------------------|-----------------------|-----------------------|----------|
| DAI0_IRQH        | Block Start           | DAIx_IRPTL            | SEC_ID                |       24 |
| DAI0_IRQL        | Non Audio             |                       |                       |      145 |

## Clocking

The fundamental timing clock of the S/PDIF receiver is CLKO5 from the CDU. When CLKO5 is configured, it supports sampling frequencies of 24 kHz to 192 kHz. The fundamental timing clock of the S/PDIF transmitter is SCLK0 /4.

For information on clock programming, see CDU Programming Model.

## S/PDIF Transmitter

The following sections provide information on the S/PDIF transmitter.

## Functional Description

The S/PDIF transmitter, shown in the S/PDIF Transmitter Block Diagram , resides within the DAI, and its inputs and outputs can be routed via the SRU. It receives audio data in serial format, encloses the specified user status information, and converts it into the biphase encoded signal. The serial data input to the transmitter can be

formatted as left-justified, I 2 S, or right-justified with word widths of 16, 18, 20 or 24 bits. AES3 Output Block shows the detail of the AES block.

The serial data, clock, and frame sync inputs to the S/PDIF transmitter are routed through the signal routing unit (SRU).

The S/PDIF transmitter output may be routed to an output pin via the SRU and then routed to another S/PDIF receiver or to components for off-board connections to other S/PDIF receivers. The output is also available to the S/ PDIF receiver for loop-back testing through SRU.

In addition to encoding the audio data in the bi-phase format, the transmitter also provides a way to easily add the channel status information to the outgoing bi-phase stream. There are status/user registers for a frame (192-bits/24 bytes) in the transmitter that correspond to each channel or subframe.

Figure 35-1: S/PDIF Transmitter Block Diagram

<!-- image -->

Validity bits for both channels may also be controlled by the transmitter control register. Optionally, the user bit, validity bit, and channel status bit are sent to the transmitter with each left/right sample. For each subframe the parity bit is automatically generated and inserted into the biphase encoded data.

A mute control and support for double-frequency single-channel mode are also provided. The serial data input format may be selected as left-justified, I 2 S, or right-justified with 16-, 18-, 20-, or 24-bit word widths. The over sampling clock is also selected by the transmitter control register.

Figure 35-2: AES3 Output Block

<!-- image -->

## Input Data Formats

The I2S and Left-Justified Formats and Right-Justified Formats figures show the format of data that is sent to the S/ PDIF transmitter using a variety of protocol standards.

Figure 35-3: I 2 S and Left-Justified Formats

<!-- image -->

Figure 35-4: Right-Justified Formats

## Operating Modes

The S/PDIF transmitter can operate in standalone and full serial modes. The following sections describe these modes in detail.

## Full Serial Mode

This mode is selected by clearing the SPDIF\_TX\_CTL.AUTO bit. In this mode all the status bits, audio data and the block start bit (indicating start of a frame), come through the serial data stream ( SPDIF\_TX\_DATA\_I ) pin. The transmitter should be enabled after or at the same time as all of the other control bits.

## Standalone Mode

This mode is selected by setting the SPDIF\_TX\_CTL.AUTO bit. In this mode, the block start bit (indicating the start of a frame) is generated internally. The channel status bits come from the channel status buffer registers. The user status bits come from the user bits buffers as shown in the AES3 Output Block figure.

The validity bits are the SPDIF\_TX\_CTL.VALIDR and SPDIF\_TX\_CTL.VALIDL . In this mode only audio data comes from the SPDIF\_TX\_DATA\_I pin. All other data, including the status bit and block start bit is either generated internally or taken from the internal register.

Once the user bits buffer registers ( SPDIF\_TX\_UBUFF\_A0 -SPDIF\_TX\_UBUFF\_B5 ) are programmed, they are used only for the next block of data. This allows programs to change the user bit information in every block of data.

<!-- image -->

To allow user bit updates, write a 0x1 to the SPDIF\_TX\_USRUPDT register that is used for further processing. If the SPDIF\_TX\_CTL.AUTO bit is set:

- and if SPDIF\_TX\_USRUPDT =1, at every 192nd frame end the user status bits are taken from user bits buffers and transmitted. Simultaneously, the SPDIF\_TX\_USRUPDT register is cleared automatically by hardware.
- and if SPDIF\_TX\_USRUPDT =0, at every 192nd frame end the user status bits are updated as zeros and transmitted. The SPDIF\_TX\_USRUPDT register remains low.

In general, for the next block, programs can update user bits buffers at any time during the transfer of the current block (1 block = 192 frames). There are internal buffers to store the user status bits of the current block of transfer. In other words, at the beginning of every new block, the user status bit ( SPDIF\_TX\_CTL.USRPEND bit) from user bits buffers are copied to internal buffers and transmitted in each frame during the transfer.

Note that since a frame contains 192 bits/8 = 24 bytes, six status/user registers are required to store each four bytes.

## Data Output Mode

Two output data formats are supported by the transmitter; two channel mode and single-channel double-frequency (SCDF) mode. The output format is determined by the transmitter control register ( SPDIF\_TX\_CTL ).

In two channel mode, the left channel (channel A) is transmitted when the SPDIF\_TX\_FS\_I is high, and the right channel (channel B) is transmitted when the SPDIF\_TX\_FS\_I is low.

In SCDF mode, the transmitter sends successive audio samples of the same signal across both sub frames, instead of channel A and B. The transmitter transmits at half the sample rate of the input bit stream. The SPDIF\_TX\_CTL.SCDF bit selects SCDF mode. When in SCDF mode, the SPDIF\_TX\_CTL.SCDFLR bit determines whether left or right channel data is transmitted.

## S/PDIF Receiver

The S/PDIF receiver ( S/PDIF Receiver Block Diagram ) is compliant with all common serial digital audio interface standards including IEC-60958, IEC-61937, AES3, and AES11. For the IEC-60958 standard, all the user-data and channel-status bits (as outlined in this document) are not decoded by the S/PDIF receiver. The interface does make all 192 user-data and channel-status pairs available as an output of the block, for post-decoding.

For the IEC-61937 standard, the S/PDIF only detects compressed AC-3 and DTS formats. No decompression is performed.

Figure 35-5: S/PDIF Receiver Block Diagram

<!-- image -->

## Functional Description

If the receiver is used, programs need to enable it using the SPDIF\_RX\_CTL register. After the SRU programming is complete, write to the register with control values. At this point, the receiver attempts to lock.

NOTE: The S/PDIF receiver is disabled at default. If the receiver is used in an application, programs should enable the receiver.

The input to the receiver (SPDIFn\_RX\_I) is a biphase encoded signal that may contain two audio channels (compressed or linear PCM) or non-audio data. The receiver decodes the single biphase encoded stream, producing an I 2 S compatible serial data output that consists of a serial clock, a left-right frame sync, and data (channel A/B). It provides the programmer with several methods of managing the incoming status bit information.

The S/PDIF receiver receives any S/PDIF stream with a sampling frequency range of 24 kHz to 192 kHz. Refer to Clocking for more details.

The channel status bits are collected into memory-mapped registers, while other channel status and user bytes must be handled manually. The block start bit, which replaces the parity bit in the serial I 2 S stream, indicates the reception of the Z preamble and the start of a new block of channel status and data bits.

## Clock Recovery

The S/PDIF receiver recovers the clock that generated the AES3/SPDIF biphase encoded stream from the incoming S/PDIF stream.

This clock is used by the receiver to clock in the biphase encoded data stream and also to provide clocks for either the SPORTs, sample rate converter, or the AES3 and S/PDIF transmitter. The recovered clock may also be used externally to the chip for clocking D/A and A/D converters.

In order to maintain performance, jitter on the clock is sourced to several peripherals.

To comply with the AES11 standard, the recovered left or right clock must be aligned with the preambles within a + or - 5% of the frame period. Since the PLL clock generates a clock 512 times the frame rate clock (512 f SCLK ), this clock can be used and divided down to create the phase aligned jitter-free left or right clock.

## Output Data Format

The extracted 24-bit audio data, V, U, C, and block start bits are sent on the SPDIF\_RX\_DAT pin in 32-bit I 2 S format as shown in I2S and Left-Justified Formats . The frame sync is transmitted on the SPDIF\_RX\_FS pin and serial clock is transmitted on the SPDIF\_RX\_CLK pin. All three pins are routed through the SRU.

<!-- image -->

Figure 35-6: I 2 S Format

## Channel Status

The channel status for the first bytes 4-0 (consumer mode) are collected into memory-mapped registers ( SPDIF\_RX\_STAT0\_A , SPDIF\_RX\_STAT0\_B , SPDIF\_RX\_STAT1\_A and SPDIF\_RX\_STAT1\_B ). All other channel status bytes 23-5 (professional mode) must be manually extracted from the receiver data stream.

NOTE: Only the first 5 channel status bytes (40-bit) for consumer mode of a frame are stored into the S/PDIF receiver status registers.

## Operating Modes

This section describes the receiver channel status for the different modes.

## Compressed or Non-linear Audio Data

The S/PDIF receiver processes compressed as well as non-linear audio data according to the supported standards. The following sections describe how this peripheral handles different data.

MPEG-2, AC-3, DTS, and AAC compressed data may be transmitted without setting either the SPDIF\_RX\_STAT.VALID bit or bit 1 of byte 0. T o detect this data, the IEC61937 and SPMTE 337M standards dictate that there be a 96-bit sync code in the 16-, 20- or 24-bit audio data stream. This sync code consists of four words of zeros followed by a word consisting of 0xF872 and another word consisting of 0x4E1F . When this sync code is detected, the SPDIF\_RX\_STAT.COMPMODE bits hold the information regarding type of compression.

The last two words of the sync code, 0xF872 and 0x4E1F , are called the preamble-A and preamble-B of the burst preamble. Preamble-C of the burst preamble contains burst information and is captured and stored by the receiver. Preamble-D of the burst preamble contains the length code and is captured by the receiver. Even if the validity bit or bit 1 of byte 0 has been set, the receiver still looks for the sync code in order to record the preamble-C and D values. Once the sync code has not been detected in 4096 frames, the preamble-C and D registers are set to zero.

## Emphasized Audio Data

Determination as to whether the received audio data is emphasized or not is done in software using the channel status bits as detailed below.

- In professional mode, (bit 0 of byte 0 = 1), channel status bits 2-4 of byte 0 indicate the audio data is emphasized if they are equal to 110 or 111.
- In consumer mode, (bit 0 of byte 0 = 0), channel status bits 3-5 indicate the audio data is emphasized if they are equal to 100, 010 or 110.

## Single-Channel Double-Frequency Mode

Unlike previous processors, support for single-channel, double-frequency mode (SCDF) is not supported through specific bits within the SPDIF\_RX\_CTL register, but must be implemented in software using the information provided by the CS (channel status) bits.

- 0111-single channel double frequency mode
- 1000-single channel double frequency mode-stereo left
- 1001-single channel double frequency mode-stereo right

## Clock Recovery Modes

The S/PDIF receiver extracts audio data, channel status, and user bits from the biphase encoded AES3 and S/PDIF stream. In addition, a 50% duty cycle reference clock running at the sampling rate of the audio input data is generated for the receiver to recover the oversampling clock.

## Number Controlled Oscillator

The receiver can recover the clock from the biphase encoded stream using an on-chip NCO shown in the following figure. The dedicated NCO is separate from the PLL that supplies the clock to the processor core, which is the default operation of the receiver.

Figure 35-7: S/PDIF Clock Recovery Mechanism

<!-- image -->

The left/right frame reference clock for the NCO is generated using the preambles. The recovered low jitter left/ right frame clock from the NCO attempts to align with the reference clock. However, this recovered left/right clock, like the reference clock, is not phase aligned with the preambles.

## Interrupts

The following sections provide information about interrupt sources, masking, and servicing.

## Sources

The S/PDIF module of each DAI drives five interrupt signals. Four are status signals driven from SPDIFn\_RX and one signal is driven from SPDIFn\_TX (block start). These signals are connected into the DAI\_IRPTL\_L / DAI\_IRPTL\_H latch register.

## Transmit Block Start

The SPDIFn\_TX\_BLKSTART output signal, if routed to any miscellaneous interrupt bits (DAIn\_INT\_31-22 in the DAI\_MISC0 / DAI\_MISC1 registers), triggers a block start interrupt during the last frame of current block.

## Receiver Status

The following receiver status bits generate an interrupt.

- Validity ( SPDIF\_RX\_STAT.VALID )
- Receiver locked ( SPDIF\_RX\_STAT.LOCK )
- No audio ( SPDIF\_RX\_STAT.AUDIOTYPE )

## Receiver Error

The loss of lock ( SPDIF\_RX\_STAT.LOCKLOSS ) bit generates an interrupt.

## Masking

For the S/PDIF receiver the DAI\_IMSK\_RE register must be unmasked accordingly. For the S/PDIF transmit the DAIn\_IMASK\_x register must be unmasked accordingly.

The INTR\_DAI\_IRQH and INTR\_DAI\_IRQL signals are routed to the SEC.

## Service

The ISR reads the DAI\_IRPTL\_H and DAI\_IRPTL\_L registers to clear the interrupt request.

## Programming Model

The following sections provide information on programming the transmitter and receiver.

## Programming the Transmitter

Since the S/PDIF transmitter data input is not available to the core, programming the transmitter is as simple as: 1) connecting the SRU to the on-chip (serial ports or input data port) or off-chip (DAI pins) serial devices that provide the clock and data to be encoded, and 2) selecting the desired mode in the transmitter control register. This setup can be accomplished in three steps.

1. Connect the transmitter's four required input signals and one biphase encoded output in the SRU. The four input signals are the serial clock ( SPDIF\_TX\_CLK\_I ), the serial frame sync ( SPDIF\_TX\_FS\_I ), the serial data ( SPDIF\_TX\_DAT\_I ), and the high frequency clock ( SPDIF\_TX\_HFCLK\_I ) used for the encoding. The only output of the transmitter is SPDIF\_TX\_O .
2. If user bits are required, write 0x1 to the SPDIF\_TX\_USRUPDT register for the first block of transfer. Also route the SPDIF\_TX\_BLK\_START\_O signal to the DAI\_INT\_31-22 ( DAI\_IRPTLx register). This generates interrupts during the last frame of the block (192), allowing changes of user bits for the next block.
3. Initialize the SPDIF\_TX\_CTL register to enable the data encoding.
4. Manually set the block start bit in the data stream once per block (every 384 words). This is necessary if automatic generation of block start information is not enabled using the SPDIF\_TX\_CTL.AUTO bit = 0.

NOTE: For more information, see the "DAI Routing Capabilities" section of the Digital Audio Interface (DAI) chapter.

## Programming the Receiver

Since the S/PDIF receiver data output is not available to the core, programming the peripheral is as simple as connecting the SRU to the on-chip (serial ports) or off-chip (DAI pins) serial devices that provide the clock and data to be decoded, and selecting the desired mode in the receiver control register. This setup can be accomplished in two steps.

1. Connect the input signal and three output signals in the SRU. The only input of the receiver is the biphase encoded stream, SPDIFn\_RX\_I . The three required output signals are the serial clock ( SPDIFn\_RX\_CLK ), the serial frame sync ( SPDIFn\_RX\_FS ), and the serial data ( SPDIFn\_RX\_DAT ). The high frequency clock ( SPDIFn\_RX\_TDMCLK ) derived from the encoded stream is also available if the system requires it.
2. Initialize the SPDIF\_RX\_CTL register to enable the data decoding. Note that this peripheral is disabled by default.

NOTE: For more information, see the "DAI Routing Capabilities" section of the Digital Audio Interface (DAI) chapter.

## Interrupted Data Streams on the Receiver

When using the S/PDIF receiver with data streams that are likely to be interrupted, (in other words unplugged and reconnected), it is necessary to take some extra steps to ensure that the S/PDIF receiver's digital PLL will relock to the stream. The steps to accomplish this are described below.

1. Set up interrupts within the DAI so that the S/PDIF receiver can generate an interrupt when the stream is reconnected.
2. Within the interrupt service routine (ISR), stop and restart the NCO. This is accomplished by setting and then clearing the SPDIF\_RX\_CTL.RST bit.
3. Return from the ISR and continue normal operation.

This method of resetting the NCO has been shown to provide extremely reliable performance when the S/PDIF inputs are interrupted or unplugged momentarily.

## Debug Features

The following feature supports S/PDIF debugging.

## Loopback Routing

The S/PDIF supports an internal loopback mode by using the SRU. For more information about loopback, see "Loopback Routing" in the Digital Audio Interface (DAI) chapter.

## ADSP-SC59x SPDIF Register Descriptions

The S/PDIF module (SPDIF) contains the following registers.

Table 35-3: ADSP-SC59x SPDIF Register List

| Name             | Description                 |
|------------------|-----------------------------|
| SPDIF_RX_CTL     | Receive Control             |
| SPDIF_RX_STAT    | Receive Status Register     |
| SPDIF_RX_STAT0_A | Receive Status A0 Register  |
| SPDIF_RX_STAT0_B | Receive Status B0 Register  |
| SPDIF_RX_STAT1_A | Receive Status A1 Register  |
| SPDIF_RX_STAT1_B | Receive Status B1 Register  |
| SPDIF_TX_CTL     | Transmit Control Register   |
| SPDIF_TX_STAT_A0 | Transmit Status A0 Register |
| SPDIF_TX_STAT_A1 | Transmit Status A1 Register |
| SPDIF_TX_STAT_A2 | Transmit Status A2 Register |
| SPDIF_TX_STAT_A3 | Transmit Status A3 Register |
| SPDIF_TX_STAT_A4 | Transmit Status A4 Register |
| SPDIF_TX_STAT_A5 | Transmit Status A5 Register |
| SPDIF_TX_STAT_B0 | Transmit Status B0 Register |
| SPDIF_TX_STAT_B1 | Transmit Status B1 Register |
| SPDIF_TX_STAT_B2 | Transmit Status B2 Register |
| SPDIF_TX_STAT_B3 | Transmit Status B3 Register |
| SPDIF_TX_STAT_B4 | Transmit Status B4 Register |
| SPDIF_TX_STAT_B5 | Transmit Status B5 Register |

Table 35-3: ADSP-SC59x SPDIF Register List (Continued)

| Name              | Description                      |
|-------------------|----------------------------------|
| SPDIF_TX_UBUFF_A0 | Transmit User Buffer A0 Register |
| SPDIF_TX_UBUFF_A1 | Transmit User Buffer A1 Register |
| SPDIF_TX_UBUFF_A2 | Transmit User Buffer A2 Register |
| SPDIF_TX_UBUFF_A3 | Transmit User Buffer A3 Register |
| SPDIF_TX_UBUFF_A4 | Transmit User Buffer A4 Register |
| SPDIF_TX_UBUFF_A5 | Transmit User Buffer A5 Register |
| SPDIF_TX_UBUFF_B0 | Transmit User Buffer B0 Register |
| SPDIF_TX_UBUFF_B1 | Transmit User Buffer B1 Register |
| SPDIF_TX_UBUFF_B2 | Transmit User Buffer B2 Register |
| SPDIF_TX_UBUFF_B3 | Transmit User Buffer B3 Register |
| SPDIF_TX_UBUFF_B4 | Transmit User Buffer B4 Register |
| SPDIF_TX_UBUFF_B5 | Transmit User Buffer B5 Register |
| SPDIF_TX_USRUPDT  | User Bit Update Register         |

## Receive Control

The SPDIF\_RX\_CTL register is used to enable and control the S/PDIF receiver.

Figure 35-8: SPDIF\_RX\_CTL Register Diagram

<!-- image -->

Table 35-4: SPDIF\_RX\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | INVTDM     | Invert TDMclock. When set to 0, inverted TDMclock or non-inverted Bit clock is selected. When set to 1, non-inverted TDMclock or inverted Blit clock is selected.                                                                                                                                                                                |
| 12:9 (R/W)         | TDMSEL     | Select TDMclock frequency. Select TDMclock frequency                                                                                                                                                                                                                                                                                             |
| 4 (R/W)            | RST        | Reset SPDIF Receiver. The SPDIF_RX_CTL.RST bit resets the receiver.                                                                                                                                                                                                                                                                              |
| 3 (R/W)            | RSTRTAUDIO | Restart Audio. The SPDIF_RX_CTL.RSTRTAUDIO bit restarts the audio once a re-lock has oc- curred. When the S/PDIF receiver loses lock the audio output is set to 0. This bit de- termines the behavior of the audio once lock is re-established. Audio can be manually restarted by toggling this bit high and then low. 0 Manually restart audio |
| 3 (R/W)            | RSTRTAUDIO | 1 Automatically restart audio                                                                                                                                                                                                                                                                                                                    |
| 3 (R/W)            | RSTRTAUDIO |                                                                                                                                                                                                                                                                                                                                                  |

Table 35-4: SPDIF\_RX\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | FASTLOCK   | Fast Lock Select. The SPDIF_RX_CTL.FASTLOCK bit allows the lock mechanism to lock at normal speed or at faster speed. This has the advantage of recovering very quickly whenever the S/PDIF receiver looses lock due to glitches in the signal. In normal mode the S/ PDIF receiver locks after 64 consecutive valid samples, in fast mode the S/PDIF receiv- er locks after 8 consecutive valid samples                                                                                                         | Fast Lock Select. The SPDIF_RX_CTL.FASTLOCK bit allows the lock mechanism to lock at normal speed or at faster speed. This has the advantage of recovering very quickly whenever the S/PDIF receiver looses lock due to glitches in the signal. In normal mode the S/ PDIF receiver locks after 64 consecutive valid samples, in fast mode the S/PDIF receiv- er locks after 8 consecutive valid samples                                                                                                         |
| 2 (R/W)            | FASTLOCK   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Enable normal mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | STRENGTH   | FS strength Control. The SPDIF_RX_CTL.STRENGTH bit controls the strength of the bit clock and Frame sync outputs from the SPDIF receiver. In strong mode these output signals are continued (as best possible) when the receiver notices a loss-of-lock condition. Note that 'as best possible' refers to the fact that this recovered signal may not be accurate, given the loss-of-lock condition. In weak mode these output signals are interrupted as soon as the receiver notices a loss-of-lock condition. | FS strength Control. The SPDIF_RX_CTL.STRENGTH bit controls the strength of the bit clock and Frame sync outputs from the SPDIF receiver. In strong mode these output signals are continued (as best possible) when the receiver notices a loss-of-lock condition. Note that 'as best possible' refers to the fact that this recovered signal may not be accurate, given the loss-of-lock condition. In weak mode these output signals are interrupted as soon as the receiver notices a loss-of-lock condition. |
| 1 (R/W)            | STRENGTH   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Enable strong mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | STRENGTH   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Enable weak mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | EN         | SPDIF Receiver Enable. When the SPDIF_RX_CTL.EN bit =0 the clock to SPDIF is switched off for power savings.                                                                                                                                                                                                                                                                                                                                                                                                     | SPDIF Receiver Enable. When the SPDIF_RX_CTL.EN bit =0 the clock to SPDIF is switched off for power savings.                                                                                                                                                                                                                                                                                                                                                                                                     |
| 0 (R/W)            | EN         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Disable receiver                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | EN         | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Enable receiver                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

## Receive Status Register

The SPDIF\_RX\_STAT register consists of bits that indicate the status of various functions supported by S/PDIF receiver.

Figure 35-9: SPDIF\_RX\_STAT Register Diagram

<!-- image -->

Table 35-5: SPDIF\_RX\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | COMPMODE   | Compression Mode. The SPDIF_RX_STAT.COMPMODE bit field indicates the type of compression mode used in the Digital audio stream. The value in this field indicates the 16-bit burst information as specified by the IEC 62937-2 standard. Use this document to de- code the value in this bit field. | Compression Mode. The SPDIF_RX_STAT.COMPMODE bit field indicates the type of compression mode used in the Digital audio stream. The value in this field indicates the 16-bit burst information as specified by the IEC 62937-2 standard. Use this document to de- code the value in this bit field. |
| 15:12 (R/NW)       | WLCHANB    | Word Length Channel B. The SPDIF_RX_STAT.WLCHANB bit field indicates the S/PDIF word length for channel B. May be decoded as follows (from the S/PDIF standard).                                                                                                                                    | Word Length Channel B. The SPDIF_RX_STAT.WLCHANB bit field indicates the S/PDIF word length for channel B. May be decoded as follows (from the S/PDIF standard).                                                                                                                                    |
|                    |            | 0                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                            |
|                    |            | 1                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                            |
|                    |            | 2                                                                                                                                                                                                                                                                                                   | SPDIF_LENGTH_16                                                                                                                                                                                                                                                                                     |
|                    |            | 3                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                            |
|                    |            | 4                                                                                                                                                                                                                                                                                                   | SPDIF_LENGTH_18                                                                                                                                                                                                                                                                                     |
|                    |            | 5                                                                                                                                                                                                                                                                                                   | SPDIF_LENGTH_22                                                                                                                                                                                                                                                                                     |
|                    |            | 6                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                            |

Table 35-5: SPDIF\_RX\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                              | Description/Enumeration                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 7                                                                                                                                                            | Reserved                                                                                                                                                     |
|                    |            | 8                                                                                                                                                            | SPDIF_LENGTH_19                                                                                                                                              |
|                    |            | 9                                                                                                                                                            | SPDIF_LENGTH_23                                                                                                                                              |
|                    |            | 10                                                                                                                                                           | SPDIF_LENGTH_20                                                                                                                                              |
|                    |            | 11                                                                                                                                                           | SPDIF_LENGTH_24                                                                                                                                              |
|                    |            | 12                                                                                                                                                           | SPDIF_LENGTH_17                                                                                                                                              |
|                    |            | 13                                                                                                                                                           | SPDIF_LENGTH_21                                                                                                                                              |
|                    |            | 14                                                                                                                                                           | Reserved                                                                                                                                                     |
|                    |            | 15                                                                                                                                                           | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | Word Length Channel A. The SPDIF_RX_STAT.WLCHANA bit indicates the S/PDIF word length for chan- nel A. May be decoded as follows (from the S/PDIF standard). | Word Length Channel A. The SPDIF_RX_STAT.WLCHANA bit indicates the S/PDIF word length for chan- nel A. May be decoded as follows (from the S/PDIF standard). |
| 11:8 (R/NW)        | WLCHANA    | 0                                                                                                                                                            | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | 1                                                                                                                                                            | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | 2                                                                                                                                                            | SPDIF_LENGTH_16                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 3                                                                                                                                                            | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | 4                                                                                                                                                            | SPDIF_LENGTH_18                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 5                                                                                                                                                            | SPDIF_LENGTH_22                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 6                                                                                                                                                            | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | 7                                                                                                                                                            | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | 8                                                                                                                                                            | SPDIF_LENGTH_19                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 9                                                                                                                                                            | SPDIF_LENGTH_23                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 10                                                                                                                                                           | SPDIF_LENGTH_20                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 11                                                                                                                                                           | SPDIF_LENGTH_24                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 12                                                                                                                                                           | SPDIF_LENGTH_17                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 13                                                                                                                                                           | SPDIF_LENGTH_21                                                                                                                                              |
| 11:8 (R/NW)        | WLCHANA    | 14                                                                                                                                                           | Reserved                                                                                                                                                     |
| 11:8 (R/NW)        | WLCHANA    | 15                                                                                                                                                           | Reserved                                                                                                                                                     |

Table 35-5: SPDIF\_RX\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/NW)           | LOCKLOSS   | Loss of Lock (sticky). The SPDIF_RX_STAT.LOCKLOSS bit indicates that the system has lost lock. This is different to the lock register, as it is sticky it goes high when system looses lock, but returns to low once SPDIF_RX_CTL.RSTRTAUDIO bit is toggled. This is to allow the programs to poll the lock status. |
| 3 (R/NW)           | LOCK       | Lock Receiver. The SPDIF_RX_STAT.LOCK bit indicates the S/PDIF receiver has successfully locked to the S/PDIF stream and is outputting valid data.                                                                                                                                                                  |
| 2 (R/NW)           | VALID      | Validity Bit. The SPDIF_RX_STAT.VALID bit indicates the ORed bits of channel A and B. 0 Linear PCM data                                                                                                                                                                                                             |
| 1 (R/NW)           | COMPTYPE   | 1 Non-linear audio data Compression Type. The SPDIF_RX_STAT.COMPTYPE bit indicates AC3 or DTS compression. Valid only if SPDIF_RX_STAT.AUDIOTYPE indicates compressed data. 0 AC3 compressed data 1 DTS compressed data                                                                                             |
| 0 (R/NW)           | AUDIOTYPE  | Audio Type. The SPDIF_RX_STAT.AUDIOTYPE bit indicates PCM or compressed data. 0 PCM data                                                                                                                                                                                                                            |

## Receive Status A0 Register

The SPDIF\_RX\_STAT0\_A register holds the receive channel 0 status for bytes 0-3 for sub frame A.

Figure 35-10: SPDIF\_RX\_STAT0\_A Register Diagram

<!-- image -->

Table 35-6: SPDIF\_RX\_STAT0\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | BYTE3      | CS Byte 3. The SPDIF_RX_STAT0_A.BYTE3 bit field contains byte 3 of received channel A status.            |
| 23:16 (R/NW)       | BYTE2      | CS Byte 2. The SPDIF_RX_STAT0_A.BYTE2 bit field contains byte 2 of received channel A status.            |
| 15:8 (R/NW)        | BYTE1      | CS Byte 1. The SPDIF_RX_STAT0_A.BYTE1 bit field contains byte 1 of received channel A status.            |
| 7:0 (R/NW)         | BYTE0      | CS Byte 0. The SPDIF_RX_STAT0_A.BYTE0 bit field contains the status of byte 0-3 of re- ceived channel A. |

## Receive Status B0 Register

The SPDIF\_RX\_STAT0\_B register holds the receive channel 0 status for bytes 0-3 for sub frame B.

Figure 35-11: SPDIF\_RX\_STAT0\_B Register Diagram

<!-- image -->

Table 35-7: SPDIF\_RX\_STAT0\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | BYTE3      | CS Byte 3. The SPDIF_RX_STAT0_B.BYTE3 bit field contains byte 3 of received channel B status. |
| 23:16 (R/NW)       | BYTE2      | CS Byte 2. The SPDIF_RX_STAT0_B.BYTE2 bit field contains byte 2 of received channel B status. |
| 15:8 (R/NW)        | BYTE1      | CS Byte 1. The SPDIF_RX_STAT0_B.BYTE1 bit field contains byte 1 of received channel B status. |
| 7:0 (R/NW)         | BYTE0      | CS Byte 0. The SPDIF_RX_STAT0_B.BYTE0 bit field contains byte 0 of received channel B status. |

## Receive Status A1 Register

The SPDIF\_RX\_STAT1\_A register holds the receive channel 1 status for byte 4 for sub frame A.

Figure 35-12: SPDIF\_RX\_STAT1\_A Register Diagram

<!-- image -->

Table 35-8: SPDIF\_RX\_STAT1\_A Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | BYTE4      | CS Byte 4. The SPDIF_RX_STAT1_A.BYTE4 bit field contains byte 4 of received channel A status. |

## Receive Status B1 Register

The SPDIF\_RX\_STAT1\_B register holds the receive channel 1 status for byte 4 for sub frame B.

Figure 35-13: SPDIF\_RX\_STAT1\_B Register Diagram

<!-- image -->

Table 35-9: SPDIF\_RX\_STAT1\_B Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | BYTE4      | CS Byte 4. The SPDIF_RX_STAT1_B.BYTE4 bit field contains byte 4 of received channel B status. |

## Transmit Control Register

The SPDIF\_TX\_CTL register provides bits the enable/disable the transmitter and configure several options related to data transmission.

Figure 35-14: SPDIF\_TX\_CTL Register Diagram

<!-- image -->

Table 35-10: SPDIF\_TX\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE0B     | Channel Status Byte for Subframe B. The SPDIF_TX_CTL.BYTE0B bit field contains the channel status for the second bytes 95.                                                                                                        |
| 23:16 (R/W)        | BYTE0A     | Channel Status Byte for Subframe A. The SPDIF_TX_CTL.BYTE0A bit field contains the channel status for the first bytes 40.                                                                                                         |
| 15 (R/W)           | EXTSYNC    | External Sync Enable. When the SPDIF_TX_CTL.EXTSYNC bit is set (regardless of the SPDIF_TX_CTL.AUTO bit setting) the internal frame counter is set to zero at an internal LRCLK rising edge followed by an EXTSYNC_I rising edge. |

Table 35-10: SPDIF\_TX\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/NW)          | USRPEND    | User Bits Pending. The SPDIF_TX_CTL.USRPEND bit is set if the update of the internal buffer from the Transmit User Bits Buffer registers has not yet completed.                                                                                                                                                                                                                                 |
| 12 (R/NW)          | BLKSTART   | Block Start. The SPDIF_TX_CTL.BLKSTART bit is a status bit that indicates block start (when the SPDIF_TX_CTL.AUTO bit = 1). is not block start                                                                                                                                                                                                                                                  |
| 12 (R/NW)          | BLKSTART   | 0 Current word                                                                                                                                                                                                                                                                                                                                                                                  |
| 12 (R/NW)          | BLKSTART   | 1 Current word is block start                                                                                                                                                                                                                                                                                                                                                                   |
| 11 (R/W)           | VALIDR     | Validity Bit B. Use the SPDIF_TX_CTL.VALIDR bit with the channel status buffer.                                                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | VALIDL     | Validity Bit A. The SPDIF_TX_CTL.VALIDL bit either manually start block transfer according to input stream status bits or automatically start block transfer. Use the SPDIF_TX_CTL.VALIDL bit with the channel status buffer.                                                                                                                                                                   |
| 10 (R/W)           | VALIDL     | 0 Manually start block transfer                                                                                                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | VALIDL     | 1 Automatically start block transfer                                                                                                                                                                                                                                                                                                                                                            |
| 9 (R/W)            | AUTO       | Automatically Generate Block Start. When enabled, the transmitter is in standalone mode where it inserts block start, chan- nel status, and validity bits on its own. If the channel status or validity buffer needs to be enabled (after the DAI programming is complete), first write to the buffers with the required data and then enable the buffers by setting the SPDIF_TX_CTL.AUTO bit. |
| 8:6 (R/W)          | SMODEIN    | Serial Data Input Format. The SPDIF_TX_CTL.SMODEIN bit selects the data input format.                                                                                                                                                                                                                                                                                                           |
| 8:6 (R/W)          | SMODEIN    | 0 Left-justified                                                                                                                                                                                                                                                                                                                                                                                |
| 8:6 (R/W)          | SMODEIN    | 1 I2S                                                                                                                                                                                                                                                                                                                                                                                           |
| 8:6 (R/W)          | SMODEIN    | 2 Reserved                                                                                                                                                                                                                                                                                                                                                                                      |
| 8:6 (R/W)          | SMODEIN    | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                      |
| 8:6 (R/W)          | SMODEIN    | 4 Right-justified, 24 bits                                                                                                                                                                                                                                                                                                                                                                      |
| 8:6 (R/W)          | SMODEIN    | 5 Right-justified, 20 bits                                                                                                                                                                                                                                                                                                                                                                      |
| 8:6 (R/W)          | SMODEIN    | 6 Right-justified, 18 bits                                                                                                                                                                                                                                                                                                                                                                      |
| 8:6 (R/W)          | SMODEIN    | 7 Right-justified, 16 bits                                                                                                                                                                                                                                                                                                                                                                      |

Table 35-10: SPDIF\_TX\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | SCDFLR     | Select L/R Single-Channel, Double-Frequency Mode. The SPDIF_TX_CTL.SCDFLR bit selects the left or right channel in SCDF mode. 0 Left channel   |
| 4 (R/W)            | SCDF       | Single-Channel, Double-Frequency Mode Enable. The SPDIF_TX_CTL.SCDF bit enables single-channel, double-frequency mode. 0 Two-channel mode      |
| 3:2 (R/W)          | FREQ       | Frequency Multiplier. The SPDIF_TX_CTL.FREQ bit field sets the oversampling ratio. 0 256 x frame sync oversampling                             |
| 1 (R/W)            | MUTE       | Mute. The SPDIF_TX_CTL.MUTE bit mutes the serial data output. 0 Disable Mute 1 Enable Mute                                                     |
| 0 (R/W)            | EN         | Transmitter Enable. The SPDIF_TX_CTL.EN bit enables the transmitter and resets the control registers to their defaults. 0 Transmitter disabled |

## Transmit Status A0 Register

The SPDIF\_TX\_STAT\_A0 register holds the transmit channel 0 status for bytes 1-4 for sub frame A.

Figure 35-15: SPDIF\_TX\_STAT\_A0 Register Diagram

<!-- image -->

Table 35-11: SPDIF\_TX\_STAT\_A0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE4      | Byte 4 Sub Frame A. The SPDIF_TX_STAT_A0.BYTE4 bit field holds the transmit channel 0 status for byte 4 for sub frame A. |
| 23:16 (R/W)        | BYTE3      | Byte 3 Sub Frame A. The SPDIF_TX_STAT_A0.BYTE3 bit field holds the transmit channel 0 status for byte 3 for sub frame A. |
| 15:8 (R/W)         | BYTE2      | Byte 2 Sub Frame A. The SPDIF_TX_STAT_A0.BYTE2 bit field holds the transmit channel 0 status for byte 2 for sub frame A. |
| 7:0 (R/W)          | BYTE1      | Byte 1 Sub Frame A. The SPDIF_TX_STAT_A0.BYTE1 bit field holds the transmit channel 0 status for byte 1 for sub frame A. |

## Transmit Status A1 Register

The SPDIF\_TX\_STAT\_A1 register holds the transmit status for bytes 5-8 for sub frame A.

Figure 35-16: SPDIF\_TX\_STAT\_A1 Register Diagram

<!-- image -->

Table 35-12: SPDIF\_TX\_STAT\_A1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE8      | Byte 8 Sub Frame A. The SPDIF_TX_STAT_A1.BYTE8 bit field contains transmit status for byte 8 of sub frame A. |
| 23:16 (R/W)        | BYTE7      | Byte 7 Sub Frame A. The SPDIF_TX_STAT_A1.BYTE7 bit field contains transmit status for byte 7 of sub frame A. |
| 15:8 (R/W)         | BYTE6      | Byte 6 Sub Frame A. The SPDIF_TX_STAT_A1.BYTE6 bit field contains transmit status for byte 6 of sub frame A. |
| 7:0 (R/W)          | BYTE5      | Byte 5 Sub Frame A. The SPDIF_TX_STAT_A1.BYTE5 bit field contains transmit status for byte 5 of sub frame A. |

## Transmit Status A2 Register

The SPDIF\_TX\_STAT\_A2 register holds the transmit status for bytes 9-12 for sub frame A.

Figure 35-17: SPDIF\_TX\_STAT\_A2 Register Diagram

<!-- image -->

Table 35-13: SPDIF\_TX\_STAT\_A2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE12     | Byte 12 Sub Frame A. The SPDIF_TX_STAT_A2.BYTE12 bit field contains transmit status for byte 12 of sub frame A. |
| 23:16 (R/W)        | BYTE11     | Byte 11 Sub Frame A. The SPDIF_TX_STAT_A2.BYTE11 bit field contains transmit status for byte 11 of sub frame A. |
| 15:8 (R/W)         | BYTE10     | Byte 10 Sub Frame A. The SPDIF_TX_STAT_A2.BYTE10 bit field contains transmit status for byte 10 of sub frame A. |
| 7:0 (R/W)          | BYTE9      | Byte 9 Sub Frame A. The SPDIF_TX_STAT_A2.BYTE9 bit field contains transmit status for byte 9 of sub frame A.    |

## Transmit Status A3 Register

The SPDIF\_TX\_STAT\_A3 register holds the transmit status for bytes 13-16 for sub frame A.

Figure 35-18: SPDIF\_TX\_STAT\_A3 Register Diagram

<!-- image -->

Table 35-14: SPDIF\_TX\_STAT\_A3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE16     | Byte 16 Sub Frame A. The SPDIF_TX_STAT_A3.BYTE16 bit field contains transmit status for byte 16 of sub frame A. |
| 23:16 (R/W)        | BYTE15     | Byte 15 Sub Frame A. The SPDIF_TX_STAT_A3.BYTE15 bit field contains transmit status for byte 15 of sub frame A. |
| 15:8 (R/W)         | BYTE14     | Byte 14 Sub Frame A. The SPDIF_TX_STAT_A3.BYTE14 bit field contains transmit status for byte 14 of sub frame A. |
| 7:0 (R/W)          | BYTE13     | Byte 13 Sub Frame A. The SPDIF_TX_STAT_A3.BYTE13 bit field contains transmit status for byte 13 of sub frame A. |

## Transmit Status A4 Register

The SPDIF\_TX\_STAT\_A4 register holds the transmit status for bytes 17-20 for sub frame A.

Figure 35-19: SPDIF\_TX\_STAT\_A4 Register Diagram

<!-- image -->

Table 35-15: SPDIF\_TX\_STAT\_A4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE20     | Byte 20 Sub Frame A. The SPDIF_TX_STAT_A4.BYTE20 bit field contains transmit status for byte 20 of sub frame A. |
| 23:16 (R/W)        | BYTE19     | Byte 19 Sub Frame A. The SPDIF_TX_STAT_A4.BYTE19 bit field contains transmit status for byte 19 of sub frame A. |
| 15:8 (R/W)         | BYTE18     | Byte 18 Sub Frame A. The SPDIF_TX_STAT_A4.BYTE18 bit field contains transmit status for byte 18 of sub frame A. |
| 7:0 (R/W)          | BYTE17     | Byte 17 Sub Frame A. The SPDIF_TX_STAT_A4.BYTE17 bit field contains transmit status for byte 17 of sub frame A. |

## Transmit Status A5 Register

The SPDIF\_TX\_STAT\_A5 register holds the transmit status for bytes 21-23 for sub frame A.

Figure 35-20: SPDIF\_TX\_STAT\_A5 Register Diagram

<!-- image -->

Table 35-16: SPDIF\_TX\_STAT\_A5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | BYTE23     | Byte 23 Sub Frame A. The SPDIF_TX_STAT_A5.BYTE23 bit field contains transmit status for byte 23 of sub frame A. |
| 15:8 (R/W)         | BYTE22     | Byte 22 Sub Frame A. The SPDIF_TX_STAT_A5.BYTE22 bit field contains transmit status for byte 22 of sub frame A. |
| 7:0 (R/W)          | BYTE21     | Byte 21 Sub Frame A. The SPDIF_TX_STAT_A5.BYTE21 bit field contains transmit status for byte 21 of sub frame A. |

## Transmit Status B0 Register

The SPDIF\_TX\_STAT\_B0 register holds the transmit channel 0 status for bytes 1-4 for sub frame B.

Figure 35-21: SPDIF\_TX\_STAT\_B0 Register Diagram

<!-- image -->

Table 35-17: SPDIF\_TX\_STAT\_B0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE4      | Byte 4 Sub Frame B. The SPDIF_TX_STAT_B0.BYTE4 bit field holds the transmit channel 0 status for byte 4 for sub frame B. |
| 23:16 (R/W)        | BYTE3      | Byte 3 Sub Frame B. The SPDIF_TX_STAT_B0.BYTE3 bit field holds the transmit channel 0 status for byte 3 for sub frame B. |
| 15:8 (R/W)         | BYTE2      | Byte 2 Sub Frame B. The SPDIF_TX_STAT_B0.BYTE2 bit field holds the transmit channel 0 status for byte 2 for sub frame B. |
| 7:0 (R/W)          | BYTE1      | Byte 1 Sub Frame B. The SPDIF_TX_STAT_B0.BYTE1 bit field holds the transmit channel 0 status for byte 1 for sub frame B. |

## Transmit Status B1 Register

The SPDIF\_TX\_STAT\_B1 register holds the transmit status for bytes 5-8 for sub frame B.

Figure 35-22: SPDIF\_TX\_STAT\_B1 Register Diagram

<!-- image -->

Table 35-18: SPDIF\_TX\_STAT\_B1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE8      | Byte 8 Sub Frame B. The SPDIF_TX_STAT_B1.BYTE8 bit field contains transmit status for byte 8 of sub frame B. |
| 23:16 (R/W)        | BYTE7      | Byte 7 Sub Frame B. The SPDIF_TX_STAT_B1.BYTE7 bit field contains transmit status for byte 7 of sub frame B. |
| 15:8 (R/W)         | BYTE6      | Byte 6 Sub Frame B. The SPDIF_TX_STAT_B1.BYTE6 bit field contains transmit status for byte 6 of sub frame B. |
| 7:0 (R/W)          | BYTE5      | Byte 5 Sub Frame B. The SPDIF_TX_STAT_B1.BYTE5 bit field contains transmit status for byte 5 of sub frame B. |

## Transmit Status B2 Register

The SPDIF\_TX\_STAT\_B2 register holds the transmit status for bytes 9-12 for sub frame B.

Figure 35-23: SPDIF\_TX\_STAT\_B2 Register Diagram

<!-- image -->

Table 35-19: SPDIF\_TX\_STAT\_B2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE12     | Byte 12 Sub Frame B. The SPDIF_TX_STAT_B2.BYTE12 bit field contains transmit status for byte 12 of sub frame B. |
| 23:16 (R/W)        | BYTE11     | Byte 11 Sub Frame B. The SPDIF_TX_STAT_B2.BYTE11 bit field contains transmit status for byte 11 of sub frame B. |
| 15:8 (R/W)         | BYTE10     | Byte 10 Sub Frame B. The SPDIF_TX_STAT_B2.BYTE10 bit field contains transmit status for byte 10 of sub frame B. |
| 7:0 (R/W)          | BYTE9      | Byte 9 Sub Frame B. The SPDIF_TX_STAT_B2.BYTE9 bit field contains transmit status for byte 9 of sub frame B.    |

## Transmit Status B3 Register

The SPDIF\_TX\_STAT\_B3 register holds the transmit status for bytes 13-16 for sub frame B.

Figure 35-24: SPDIF\_TX\_STAT\_B3 Register Diagram

<!-- image -->

Table 35-20: SPDIF\_TX\_STAT\_B3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE16     | Byte 16 Sub Frame B. The SPDIF_TX_STAT_B3.BYTE16 bit field contains transmit status for byte 16 of sub frame B. |
| 23:16 (R/W)        | BYTE15     | Byte 15 Sub Frame B. The SPDIF_TX_STAT_B3.BYTE15 bit field contains transmit status for byte 15 of sub frame B. |
| 15:8 (R/W)         | BYTE14     | Byte 14 Sub Frame B. The SPDIF_TX_STAT_B3.BYTE14 bit field contains transmit status for byte 14 of sub frame B. |
| 7:0 (R/W)          | BYTE13     | Byte 13 Sub Frame B. The SPDIF_TX_STAT_B3.BYTE13 bit field contains transmit status for byte 13 of sub frame B. |

## Transmit Status B4 Register

The SPDIF\_TX\_STAT\_B4 register holds the transmit status for bytes 17-20 for sub frame B.

Figure 35-25: SPDIF\_TX\_STAT\_B4 Register Diagram

<!-- image -->

Table 35-21: SPDIF\_TX\_STAT\_B4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE20     | Byte 20 Sub Frame B. The SPDIF_TX_STAT_B4.BYTE20 bit field contains transmit status for byte 20 of sub frame B. |
| 23:16 (R/W)        | BYTE19     | Byte 19 Sub Frame B. The SPDIF_TX_STAT_B4.BYTE19 bit field contains transmit status for byte 19 of sub frame B. |
| 15:8 (R/W)         | BYTE18     | Byte 18 Sub Frame B. The SPDIF_TX_STAT_B4.BYTE18 bit field contains transmit status for byte 18 of sub frame B. |
| 7:0 (R/W)          | BYTE17     | Byte 17 Sub Frame B. The SPDIF_TX_STAT_B4.BYTE17 bit field contains transmit status for byte 17 of sub frame B. |

## Transmit Status B5 Register

The SPDIF\_TX\_STAT\_B5 register holds the transmit status for bytes 21-23 for sub frame B.

Figure 35-26: SPDIF\_TX\_STAT\_B5 Register Diagram

<!-- image -->

Table 35-22: SPDIF\_TX\_STAT\_B5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | BYTE23     | Byte 23 Sub Frame B. The SPDIF_TX_STAT_B5.BYTE23 bit field contains transmit status for byte 23 of sub frame B. |
| 15:8 (R/W)         | BYTE22     | Byte 22 Sub Frame B. The SPDIF_TX_STAT_B5.BYTE22 bit field contains transmit status for byte 22 of sub frame B. |
| 7:0 (R/W)          | BYTE21     | Byte 21 Sub Frame B. The SPDIF_TX_STAT_B5.BYTE21 bit field contains transmit status for byte 21 of sub frame B. |

## Transmit User Buffer A0 Register

The SPDIF\_TX\_UBUFF\_A0 register holds the transmit user buffer data for bytes 0-3 for sub frame A.

Figure 35-27: SPDIF\_TX\_UBUFF\_A0 Register Diagram

<!-- image -->

Table 35-23: SPDIF\_TX\_UBUFF\_A0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE3      | Byte 3 of Subframe A User Bit Buffer. The SPDIF_TX_UBUFF_A0.BYTE3 bit field contains user bit data for byte 0 of sub frame A. |
| 23:16 (R/W)        | BYTE2      | Byte 2 Sub Frame A. The SPDIF_TX_UBUFF_A0.BYTE2 bit field contains user bit data for byte 2 of sub frame A.                   |
| 15:8 (R/W)         | BYTE1      | Byte 1 Sub Frame A. The SPDIF_TX_UBUFF_A0.BYTE1 bit field contains user bit data for byte 1 of sub frame A.                   |
| 7:0 (R/W)          | BYTE0      | Byte 0 Sub Frame A. The SPDIF_TX_UBUFF_A0.BYTE0 bit field contains user bit data for byte 0 of sub frame A.                   |

## Transmit User Buffer A1 Register

The SPDIF\_TX\_UBUFF\_A1 register holds the transmit user buffer data for bytes 4-7 for sub frame A.

Figure 35-28: SPDIF\_TX\_UBUFF\_A1 Register Diagram

<!-- image -->

Table 35-24: SPDIF\_TX\_UBUFF\_A1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE7      | Byte 7 Sub Frame A. The SPDIF_TX_UBUFF_A1.BYTE7 bit field contains user bit data for byte 7 of sub frame A. |
| 23:16 (R/W)        | BYTE6      | Byte 6 Sub Frame A. The SPDIF_TX_UBUFF_A1.BYTE6 bit field contains user bit data for byte 6 of sub frame A. |
| 15:8 (R/W)         | BYTE5      | Byte 5 Sub Frame A. The SPDIF_TX_UBUFF_A1.BYTE5 bit field contains user bit data for byte 5 of sub frame A. |
| 7:0 (R/W)          | BYTE4      | Byte 4 Sub Frame A. The SPDIF_TX_UBUFF_A1.BYTE4 bit field contains user bit data for byte 0 of sub frame A. |

## Transmit User Buffer A2 Register

The SPDIF\_TX\_UBUFF\_A2 register holds the transmit user buffer data for bytes 8-11 for sub frame A.

Figure 35-29: SPDIF\_TX\_UBUFF\_A2 Register Diagram

<!-- image -->

Table 35-25: SPDIF\_TX\_UBUFF\_A2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE11     | Byte 11 Sub Frame A. The SPDIF_TX_UBUFF_A2.BYTE11 bit field contains user bit data for byte 11 of sub frame A. |
| 23:16 (R/W)        | BYTE10     | Byte 10 Sub Frame A. The SPDIF_TX_UBUFF_A2.BYTE10 bit field contains user bit data for byte 10 of sub frame A. |
| 15:8 (R/W)         | BYTE9      | Byte 9 Sub Frame A. The SPDIF_TX_UBUFF_A2.BYTE9 bit field contains user bit data for byte 9 of sub frame A.    |
| 7:0 (R/W)          | BYTE8      | Byte 8 Sub Frame A. The SPDIF_TX_UBUFF_A2.BYTE8 bit field contains user bit data for byte 8 of sub frame A.    |

## Transmit User Buffer A3 Register

The SPDIF\_TX\_UBUFF\_A3 register holds the transmit user buffer data for bytes 12-15 for sub frame A.

Figure 35-30: SPDIF\_TX\_UBUFF\_A3 Register Diagram

<!-- image -->

Table 35-26: SPDIF\_TX\_UBUFF\_A3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE15     | Byte 15 Sub Frame A. The SPDIF_TX_UBUFF_A3.BYTE15 bit field contains user bit data for byte 15 of sub frame A. |
| 23:16 (R/W)        | BYTE14     | Byte 14 Sub Frame A. The SPDIF_TX_UBUFF_A3.BYTE14 bit field contains user bit data for byte 14 of sub frame A. |
| 15:8 (R/W)         | BYTE13     | Byte 13 Sub Frame A. The SPDIF_TX_UBUFF_A3.BYTE13 bit field contains user bit data for byte 13 of sub frame A. |
| 7:0 (R/W)          | BYTE12     | Byte 12 Sub Frame A. The SPDIF_TX_UBUFF_A3.BYTE12 bit field contains user bit data for byte 12 of sub frame A. |

## Transmit User Buffer A4 Register

The SPDIF\_TX\_UBUFF\_A4 register holds the transmit user buffer data for bytes 16-19 for sub frame A.

Figure 35-31: SPDIF\_TX\_UBUFF\_A4 Register Diagram

<!-- image -->

Table 35-27: SPDIF\_TX\_UBUFF\_A4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE19     | Byte 19 Sub Frame A. The SPDIF_TX_UBUFF_A4.BYTE19 bit field contains user bit data for byte 19 of sub frame A. |
| 23:16 (R/W)        | BYTE18     | Byte 18 Sub Frame A. The SPDIF_TX_UBUFF_A4.BYTE18 bit field contains user bit data for byte 18 of sub frame A. |
| 15:8 (R/W)         | BYTE17     | Byte 17 Sub Frame A. The SPDIF_TX_UBUFF_A4.BYTE17 bit field contains user bit data for byte 17 of sub frame A. |
| 7:0 (R/W)          | BYTE16     | Byte 16 Sub Frame A. The SPDIF_TX_UBUFF_A4.BYTE16 bit field contains user bit data for byte 16 of sub frame A. |

## Transmit User Buffer A5 Register

The SPDIF\_TX\_UBUFF\_A5 register holds the transmit user buffer data for bytes 20-23 for sub frame A.

Figure 35-32: SPDIF\_TX\_UBUFF\_A5 Register Diagram

<!-- image -->

Table 35-28: SPDIF\_TX\_UBUFF\_A5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE23     | Byte 23 Sub Frame A. The SPDIF_TX_UBUFF_A5.BYTE23 bit field contains user bit data for byte 23 of sub frame A. |
| 23:16 (R/W)        | BYTE22     | Byte 22 Sub Frame A. The SPDIF_TX_UBUFF_A5.BYTE22 bit field contains user bit data for byte 22 of sub frame A. |
| 15:8 (R/W)         | BYTE21     | Byte 21 Sub Frame A. The SPDIF_TX_UBUFF_A5.BYTE21 bit field contains user bit data for byte 21 of sub frame A. |
| 7:0 (R/W)          | BYTE20     | Byte 20 Sub Frame A. The SPDIF_TX_UBUFF_A5.BYTE20 bit field contains user bit data for byte 20 of sub frame A. |

## Transmit User Buffer B0 Register

The SPDIF\_TX\_UBUFF\_B0 register holds the transmit user buffer data for bytes 0-3 for sub frame B.

Figure 35-33: SPDIF\_TX\_UBUFF\_B0 Register Diagram

<!-- image -->

Table 35-29: SPDIF\_TX\_UBUFF\_B0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE3      | Byte 3 Sub Frame B. The SPDIF_TX_UBUFF_B0.BYTE3 bit field contains user bit data for byte 3 of sub frame B. |
| 23:16 (R/W)        | BYTE2      | Byte 2 Sub Frame B. The SPDIF_TX_UBUFF_B0.BYTE2 bit field contains user bit data for byte 2 of sub frame B. |
| 15:8 (R/W)         | BYTE1      | Byte 1 Sub Frame B. The SPDIF_TX_UBUFF_B0.BYTE1 bit field contains user bit data for byte 1 of sub frame B. |
| 7:0 (R/W)          | BYTE0      | Byte 0 Sub Frame B. The SPDIF_TX_UBUFF_B0.BYTE0 bit field contains user bit data for byte 0 of sub frame B. |

## Transmit User Buffer B1 Register

The SPDIF\_TX\_UBUFF\_B1 register holds the transmit user buffer data for bytes 4-7 for sub frame B.

Figure 35-34: SPDIF\_TX\_UBUFF\_B1 Register Diagram

<!-- image -->

Table 35-30: SPDIF\_TX\_UBUFF\_B1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE7      | Byte 7 Sub Frame B. The SPDIF_TX_UBUFF_B1.BYTE7 bit field contains user bit data for byte 7 of sub frame B. |
| 23:16 (R/W)        | BYTE6      | Byte 6 Sub Frame B. The SPDIF_TX_UBUFF_B1.BYTE6 bit field contains user bit data for byte 6 of sub frame B. |
| 15:8 (R/W)         | BYTE5      | Byte 5 Sub Frame B. The SPDIF_TX_UBUFF_B1.BYTE5 bit field contains user bit data for byte 5 of sub frame B. |
| 7:0 (R/W)          | BYTE4      | Byte 4 Sub Frame B. The SPDIF_TX_UBUFF_B1.BYTE4 bit field contains user bit data for byte 4 of sub frame B. |

## Transmit User Buffer B2 Register

The SPDIF\_TX\_UBUFF\_B2 register holds the transmit user buffer data for bytes 8-11 for sub frame B.

Figure 35-35: SPDIF\_TX\_UBUFF\_B2 Register Diagram

<!-- image -->

Table 35-31: SPDIF\_TX\_UBUFF\_B2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE11     | Byte 11 Sub Frame B. The SPDIF_TX_UBUFF_B2.BYTE11 bit field contains user bit data for byte 11 of sub frame B. |
| 23:16 (R/W)        | BYTE10     | Byte 10 Sub Frame B. The SPDIF_TX_UBUFF_B2.BYTE10 bit field contains user bit data for byte 10 of sub frame B. |
| 15:8 (R/W)         | BYTE9      | Byte 9 Sub Frame B. The SPDIF_TX_UBUFF_B2.BYTE9 bit field contains user bit data for byte 9 of sub frame B.    |
| 7:0 (R/W)          | BYTE8      | Byte 8 Sub Frame B. The SPDIF_TX_UBUFF_B2.BYTE8 bit field contains user bit data for byte 8 of sub frame B.    |

## Transmit User Buffer B3 Register

The SPDIF\_TX\_UBUFF\_B3 register holds the transmit user buffer data for bytes 12-15 for sub frame B.

Figure 35-36: SPDIF\_TX\_UBUFF\_B3 Register Diagram

<!-- image -->

Table 35-32: SPDIF\_TX\_UBUFF\_B3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE15     | Byte 15 Sub Frame B. The SPDIF_TX_UBUFF_B3.BYTE15 bit field contains user bit data for byte 15 of sub frame B. |
| 23:16 (R/W)        | BYTE14     | Byte 14 Sub Frame B. The SPDIF_TX_UBUFF_B3.BYTE14 bit field contains user bit data for byte 14 of sub frame B. |
| 15:8 (R/W)         | BYTE13     | Byte 13 Sub Frame B. The SPDIF_TX_UBUFF_B3.BYTE13 bit field contains user bit data for byte 13 of sub frame B. |
| 7:0 (R/W)          | BYTE12     | Byte 12 Sub Frame B. The SPDIF_TX_UBUFF_B3.BYTE12 bit field contains user bit data for byte 12 of sub frame B. |

## Transmit User Buffer B4 Register

The SPDIF\_TX\_UBUFF\_B4 register holds the transmit user buffer data for bytes 16-19 for sub frame B.

Figure 35-37: SPDIF\_TX\_UBUFF\_B4 Register Diagram

<!-- image -->

Table 35-33: SPDIF\_TX\_UBUFF\_B4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE19     | Byte 19 Sub Frame B. The SPDIF_TX_UBUFF_B4.BYTE19 bit field contains user bit data for byte 19 of sub frame B. |
| 23:16 (R/W)        | BYTE18     | Byte 18 Sub Frame B. The SPDIF_TX_UBUFF_B4.BYTE18 bit field contains user bit data for byte 18 of sub frame B. |
| 15:8 (R/W)         | BYTE17     | Byte 17 Sub Frame B. The SPDIF_TX_UBUFF_B4.BYTE17 bit field contains user bit data for byte 17 of sub frame B. |
| 7:0 (R/W)          | BYTE16     | Byte 16 Sub Frame B. The SPDIF_TX_UBUFF_B4.BYTE16 bit field contains user bit data for byte 16 of sub frame B. |

## Transmit User Buffer B5 Register

The SPDIF\_TX\_UBUFF\_B5 register holds the transmit user buffer data for bytes 20-23 for sub frame B.

Figure 35-38: SPDIF\_TX\_UBUFF\_B5 Register Diagram

<!-- image -->

Table 35-34: SPDIF\_TX\_UBUFF\_B5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | BYTE23     | Byte 23 Sub Frame B. The SPDIF_TX_UBUFF_B5.BYTE23 bit field contains user bit data for byte 23 of sub frame B. |
| 23:16 (R/W)        | BYTE22     | Byte 22 Sub Frame B. The SPDIF_TX_UBUFF_B5.BYTE22 bit field contains user bit data for byte 22 of sub frame B. |
| 15:8 (R/W)         | BYTE21     | Byte 21 Sub Frame B. The SPDIF_TX_UBUFF_B5.BYTE21 bit field contains user bit data for byte 21 of sub frame B. |
| 7:0 (R/W)          | BYTE20     | Byte 20 Sub Frame B. The SPDIF_TX_UBUFF_B5.BYTE20 bit field contains user bit data for byte 20 of sub frame B. |

## User Bit Update Register

After writing to the transmit user buffer registers, a value of 0x1 must be written into the SPDIF\_TX\_USRUPDT register to enable the use of these user buffer bits in the next transfer block.

Figure 35-39: SPDIF\_TX\_USRUPDT Register Diagram

<!-- image -->

Table 35-35: SPDIF\_TX\_USRUPDT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | EN         | Enable. After writing to the transmit user buffer registers, a value of 0x1 must be written into the SPDIF_TX_USRUPDT register to enable the use of these SPDIF_TX_USRUPDT.EN bits in the next transfer block. |