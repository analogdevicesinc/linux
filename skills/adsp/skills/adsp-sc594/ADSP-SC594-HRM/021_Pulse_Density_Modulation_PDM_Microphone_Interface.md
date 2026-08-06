# Pulse Density Modulation (PDM) Microphone Interface

<!-- source: 021_Pulse_Density_Modulation_PDM_Microphone_Interface.pdf | original pages 953–970 -->

## 19   Pulse Density Modulation (PDM) Microphone Interface

The pulse density modulation (PDM) microphone interface converts digital PDM microphone data to I 2 S/TDM format. The microphone data in I 2 S/TDM format is then routed internally to the SPORT/ASRC or externally via the DAI pins. Each DAI unit has one PDM interface.

## PDM Features

The PDM interface includes the following features:

- Four channels of PDM audio inputs from digital microphones
- 16×, 32×, or 64× decimation ratio of PDM to pulse code modulation (PCM) audio data
- 24-bit resolution to support high sound pressure level (SPL) microphones
- 126 dB A-weighted SNR
- 4 kHz to 192 kHz output sampling rate
- Bit clock rates of 64×, 128×, 192×, 256×, 384×, or 512× the output sampling rate

NOTE: Refer to the product data sheet for the maximum BCLK frequency.

- Automatic PDM clock generation
- Target I 2 S or TDM output interface
- Supports up to 16 TDM slots
- Configurable TDM slot routing and sizes

## PDM Functional Description

The following are the typical connections for the PDM module:

- PDM interface-PDM clock (PDM\_CLK0) and PDM data (PDM\_DAT0 and PDM\_DAT1) are typically routed to DAI pins via the Signal Routing Unit (SRU) which is in turn connected to a PDM data source, for example, PDM microphones.
- Target I 2 S/TDM interface -Bit clock (BCLK), frame sync (LRCLK), and serial data (SDATA) signals are typically routed internally to the SPORT using the SRU for the microphone data to reach memory in PCM format.
- Control registers-All PDM related control registers are included in the DAI MMR system.

## ADSP-2159x\_SC592\_SC594 PDM Register List

Pulse Density Modulation interface block

Table 19-1: ADSP-2159x\_SC592\_SC594 PDM Register List

| Name        | Description                       |
|-------------|-----------------------------------|
| PDM_CTL0    | PDM Control Register              |
| PDM_HPF_CTL | High Pass Filter Control Register |
| PDM_RESET   | Software Reset Register           |
| PDM_SP_CTL0 | Serial Port Control0 Register     |
| PDM_SP_CTL1 | Serial Port Control1 Register     |

## PDM Block Diagram

The PDM Interface Block Diagram figure illustrates the interconnection of PDM block in the DAI subsystem.

Figure 19-1: PDM Interface Block Diagram

<!-- image -->

## PDM Architectural Concepts

The PDM interface provides up to four channels of decimation from a 1-bit PDM source to a 24-bit PCM audio stream. The down sampling ratio is typically 64 × f S , where f S  is the PCM output sampling rate. The down sampling ratio can also be set at 32 × f S  or 16 × f S , to facilitate higher output sampling rates. All channels decimate at the same ratio. The 24-bit down-sampled PCM audio data is output via standard stereo (I 2 S, left justified, right justified) or TDM format. The input sources for the PDM interface can be any device that has a target PDM output, such as a digital microphone.

Internally, there are four channels. The PDM Channel Mapping table details the mapping of the PDM\_DATx input pins to the internal channels. If a channel is not in use, the associated PDM\_DATx pin can be disabled to save power.

Table 19-2: PDM Channel Mapping

| Input Pin   | PDM_CLK Edge   |   Internal Channel |
|-------------|----------------|--------------------|
| PDM_DAT0    | Falling        |                  0 |
| PDM_DAT0    | Rising         |                  1 |
| PDM_DAT1    | Falling        |                  2 |
| PDM_DAT1    | Rising         |                  3 |

## PDM Initialization and Clocking

After I 2 S clocks are applied, there are 16 full frame sync cycles of initialization time before the PDM clocks start. Once the PDM clocks start running, 48 additional frame sync cycles elapse before the PDM data is driven on the SDATA pin.

During normal operation, if the bit clock or frame sync is removed then the PDM clock output stops immediately and the PDM interface enters a lower power state. If the clocks resume, the PDM interface relocks to the bit clock and frame sync signals and adjusts the PDM clock output accordingly. The length of time before the PDM clock outputs resume is 4 frames ± 1 frame to lock to the incoming signal. If the format of the clock signals change, the PDM interface detects this change at the end of the frame and stops the PDM clock outputs. Then, the device reconfigures and resumes sending PDM clocks with no user intervention. Again, the PDM clock outputs resume after 4 frames ±1 frame to lock to the incoming signal.

The PDM interface requires a clock rate that is a minimum of 64× the frame sync sample rate. Bit clock rates of 128×, 192×, 256×, 384×, and 512× the frame sync rate are also supported. The PDM interface automatically detects the ratio between the bit clock and frame sync signals and generates a PDM clock output at 64 × the frame synce rate by default. If lower decimation ratios are selected in the PDM\_CTL0 register, the PDM output clock rate corresponds to the configuration set in the PDM\_CTL0.DEC\_RATIO bit field. The minimum sampling rate is 4 kHz and the maximum sampling rate is 192 kHz. The PDM clock range is 256 kHz to 6.144 MHz. Internally, all processing is done at the PDM clock rate.

## SPORT Interface

The PDM interface supports I 2 S and TDM serial output formats. It supports up to 16 TDM slots, with slot widths of 16, 24, or 32-bits. Internal channels are connected to output slots by configuring the appropriate channel slot bit fields in the PDM\_SP\_CTL1 register. By default, each channel is routed to the slot with the same number. For example, the reset value of the PDM\_SP\_CTL1.CH3\_SLOT bit field is 0x0011, connecting channel 3 to slot 3.

Each channel has a drive select bit: PDM\_SP\_CTL1.CH0\_DRV , PDM\_SP\_CTL1.CH1\_DRV , PDM\_SP\_CTL1.CH2\_DRV , or PDM\_SP\_CTL1.CH3\_DRV . If the drive select bit is set to one, the channel is driven on the serial port in the appropriate output slot; and if it is cleared to zero, the channel does not drive (tristate high impedance mode). Note that this feature is useful only when the SPORT data is driven out of the processor via a DAI pin buffer and not when the data is routed internally to the SPORT or ASRC. To use this feature, tie the PDMx\_SDATA\_OE\_O signal to the DAIx\_PBENxx\_I signal of the DAI pin buffer.

It is possible to erroneously configure more than one channel to drive on a single TDM slot. There is no crosschecking of register settings to prevent this configuration, but it will not damage the device. In this case, only the data from the lowest channel number is driven into the slot and the data from the higher channels will not be driven.

The PDM\_SP\_CTL0.SAI\_MODE bit configures the SPORT interface mode. The two modes are stereo and TDM. The primary difference between these two modes is the format of the frame sync clock that is expected and the polarity of the active edge of the clock.

With the PDM\_SP\_CTL0.SAI\_MODE bit and the PDM\_SP\_CTL0.LRCLK\_POL bit set to zero, the serial port is in stereo mode with the clock polarity set to normal. In this mode, there must be only two channels of data sent. The frame starts with the falling edge of the frame sync, with an expected cycle of 50% high and 50% low. Channel 0 sends out data when the clock is low. As soon as the frame sync goes high the data from channel 0 stops and channel 1 begins sending data. Both edges of the frame sync clock are used. If the duty cycle is not 50/50, there may be errors in the resulting data. In this mode of operation, the PDM interface does not expect 32-bit clock transitions for each channel. All bit clock to frame sync ratios are supported.

With the PDM\_SP\_CTL0.SAI\_MODE bit set to one and the PDM\_SP\_CTL0.LRCLK\_POL bit set to zero, the serial port is in TDM mode with the clock polarity set to normal. In this mode, it can transmit anywhere from one channel to as many as four channels spread out across 16 data slots in TDM-16 format.

The PDM interface supports bit clock rates of 64×, 128×, 192×, 256×, 384×, or 512× the output sampling rate. The BCLK frequency however, must not exceed the maximum value specified in the data sheet. These bit clock rates are combined with the three different TDM slot sizes of 16-bit, 24-bit, or 32-bit slots, selected by the PDM\_SP\_CTL0.CH0\_SLOT\_WIDTH bit field. Note that as soon as the next frame sync edge is detected, the PDM interface restarts from slot 0 and any unreceived data from the previous frame is lost. This is how to achieve unusual TDM formats like TDM-5 or TDM-10. In addition, there is only support for TDM-16 or less for placing data into a TDM slot. Data cannot be placed into slots above 16, however it is possible to configure the PDM interface to tristate all unused TDM slots, which includes all the slots above the first 16 slots for modes that have more than 16 slots.

In TDM mode, the frame sync is expected to be a positive going pulse that is at least one-bit clock period wide. A falling edge is not necessary, it is only important that the signal is low long enough to meet the timing specification for a read before transitioning from low to high. The frame starts with the rising edge of this pulse. The data is clocked out according to the slot width and using the data format specified in the PDM\_SP\_CTL0 register. The PDM interface continues to send data until all active channels are sent and then the device waits for the next frame sync clock edge to start sending the next set of frame samples.

In TDM-8 format, when the PDM interface is set to output channel 0 through channel 3 into slot 0 to slot 3, the PDM interface can tristate for the remainder of the frame. This allows another PDM to output four channels onto slot 4 through slot 7. These slots do not have to be consecutive, and the two devices can be configured to interleave their respective data. The serial port can be set up to drive only when there is data to drive into a data slot. If all eight channels are not in use, configuring the drive select bit in the PDM\_SP\_CTL1 for the unused channels will assign those slots to drive or tristate in the TDM data stream. Note that this feature is useful only when the SPORT data is driven out of the chip via a DAI pin buffer and not when the data is routed internally to the SPORT or ASRC. To use this feature, tie the PDMx\_SDATA\_OE\_O signal to the DAIx\_PBENxx\_I signal of the DAI pin buffer.

To invert the IRCLK polarity, set the PDM\_SP\_CTL0.LRCLK\_POL bit to one. With inverted clock polarity in stereo mode, channel 0 is sent out when the frame sync is high, so the start of the frame is a low to high transition. With inverted clock polarity in TDM mode, the frame sync pulse is negative, so the frame starts with the high to low transition.

Configure the PDM\_SP\_CTL0.DATA\_FORMAT bit field to align data within the 32-bit data slot. There is support for left justified mode, delayed by one-bit clock period; and right justified modes for 24-bit, 20-bit, and 16-bit data word sizes.

## SPORT Timing

The Stereo Mode figure is a timing diagram for I 2 S mode with 24-bit data.

Figure 19-2: Stereo Mode

<!-- image -->

The TDM4 Mode Four Channel figure is a timing diagram for TDM4 mode with default channel assignments, 24bit data, and 32-bit slots, and normal polarity clocks.

Figure 19-3: TDM4 Four Channel

<!-- image -->

The TDM4 Mode Two Channel figure is a timing diagram that shows TDM4 mode with only two channels enabled, 24-bit data delayed by zero BCLK, 32-bit slots, and normal polarity clocks.

Figure 19-4: TDM4 Two Channel

<!-- image -->

The TDM8 Mode Four Channel figure is a timing diagram that shows TDM8 mode with all the four channels enabled, 24-bit data delayed by zero BCLK, 32-bit slots, and normal polarity clocks.

Figure 19-5: TDM8 Four Channels

<!-- image -->

The TDM2 Mode Two Channel figure is a timing diagram that shows TDM2 mode with two channels enabled, 24bit data delayed by one BCLK, 32-bit slots, and normal polarity clocks.

Figure 19-6: TDM2 Two Channels

<!-- image -->

## High Pass Filter

There is a first order high-pass filter in the signal path. It is disabled by default and can be enabled by setting the PDM\_HPF\_CTL.EN bit. T o adjust the cutoff frequency, configure the PDM\_HPF\_CTL.FC bit field. The settings are relative to the output sampling rate.

The HPF Cutoff Frequency Selection table shows the setting and the cutoff frequency for common sampling rates.

Table 19-3: HPF Cutoff Frequency Selections

|   PDM_HPF_CTL.HPF_FC |   Multiplication Factor |   48 kHz Sampling Rate Cutoff Frequency (Hz) |   32 kHz Sampling Rate Cutoff Frequency (Hz) |
|----------------------|-------------------------|----------------------------------------------|----------------------------------------------|
|                 0101 |                 0.00505 |                                       242.4  |                                       161.6  |
|                 1001 |                 0.00251 |                                       120.48 |                                        80.32 |

Table 19-3: HPF Cutoff Frequency Selections (Continued)

|   PDM_HPF_CTL.HPF_FC |   Multiplication Factor |   48 kHz Sampling Rate Cutoff Frequency (Hz) |   32 kHz Sampling Rate Cutoff Frequency (Hz) |
|----------------------|-------------------------|----------------------------------------------|----------------------------------------------|
|                 0111 |                0.00125  |                                     60       |                                     40       |
|                 1000 |                0.000623 |                                     29.904   |                                     19.936   |
|                 1001 |                0.000311 |                                     14.928   |                                      9.952   |
|                 1010 |                0.000155 |                                      7.44    |                                      4.96    |
|                 1011 |                7.77e-05 |                                      3.7296  |                                      2.4864  |
|                 1100 |                3.89e-05 |                                      1.8672  |                                      1.2448  |
|                 1101 |                1.94e-05 |                                      0.9312  |                                      0.6208  |
|                 1110 |                9.71e-06 |                                      0.46608 |                                      0.31072 |
|                 1111 |                4.68e-06 |                                      0.23328 |                                      0.15552 |

## PDM Programming Model

The following section provides information on configuring the PDM interface.

## Configuring the PDM Interface

Use the following steps to configure the PDM interface:

Optional high pass filter configuration:

- Configure the PDM\_HPF\_CTL register to enable the high pass filter using the PDM\_HPF\_CTL.EN bit and set the cutoff frequency by programming the PDM\_HPF\_CTL.FC bit field.
1. Configure PDM\_SP\_CTL0 register to set the following parameters:
- a. SPORT mode (Stereo/TDM) with the PDM\_SP\_CTL0.SAI\_MODE bit
- b. SPORT data format (BCLK delay) with PDM\_SP\_CTL0.DATA\_FORMAT bit field
- c. SPORT slot width (16, 24, or 32 bits) with PDM\_SP\_CTL0.CH0\_SLOT\_WIDTH bit field - applicable only for the TDM mode
- d. BCLK capture edge (rising/falling) with the PDM\_SP\_CTL0.BCLK\_POL bit
- e. LRCLK polarity (normal/inverted) with the PDM\_SP\_CTL0.LRCLK\_POL bit
2. Configure the PDM\_SP\_CTL1 register to set the following parameters:
- a. PDM to SPORT channel slot mapping with serial port channel slot bit fields (CHx\_SLOT)
- b. To select if the PDM channel will get driven to the associated SPORT channel slot with the serial port channel drive select bit (CHx\_DRV).

3. Configure the PDM\_CTL0 register to configure the decimation ratio with the PDM\_CTL0.DEC\_RATIO bit field and finally enable the PDM clock with the PDM\_CTL0.CLK0\_EN bit.

## Software Reset

The PDM\_RESET.SOFT bit is set to perform a soft reset of the PDM block. Setting this bit resets internal bit clock counters and audio data, but it does not reset the PDM register settings. This bit is self-clearing.

## ADSP-2159x\_SC592\_SC594 PDM Register Descriptions

Pulse Density Modulation interface block (PDM) contains the following registers.

Table 19-4: ADSP-2159x\_SC592\_SC594 PDM Register List

| Name        | Description                       |
|-------------|-----------------------------------|
| PDM_CTL0    | PDM Control Register              |
| PDM_HPF_CTL | High Pass Filter Control Register |
| PDM_RESET   | Software Reset Register           |
| PDM_SP_CTL0 | Serial Port Control0 Register     |
| PDM_SP_CTL1 | Serial Port Control1 Register     |

## PDM Control Register

The PDM\_CTL0 register (read/write) controls the channel activation, clock enable and Decimation ratio.

Figure 19-7: PDM\_CTL0 Register Diagram

<!-- image -->

Table 19-5: PDM\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 9:8                | DEC_RATIO  | Decimation Ratio.                                                              |
| 4 (R/W)            | CLK0_EN    | PDM Clock0 Enable. 0 PDM_CLK0 output disabled                                  |
| 1 (R/W)            | CH23_EN    | 1 PDM_CLK0 output enabled Enable.                                              |
| 0                  | CH01_EN    | Channels 2/3 (PDM_DAT1) 0 PDM channels 2/3 disabled 1 PDM channels 2/3 enabled |
| (R/W)              |            | Channels 0/1 (PDM_DAT0) Enable. 0 PDM channels 0/1                             |
|                    |            | disabled 1 PDM channels 0/1 enabled                                            |

## High Pass Filter Control Register

The PDM\_HPF\_CTL register (read/write) controls the High Pass Filter.

Figure 19-8: PDM\_HPF\_CTL Register Diagram

<!-- image -->

Table 19-6: PDM\_HPF\_CTL Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                          |
|--------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4                | FC              | High-pass filter cutoff frequency relative to output sample rate.                                                                                                                |
| (R/W)              | EN              | 0 Reserved 1 Reserved 2 Reserved 3 Reserved 4 Reserved 5 0.00505 6 0.00251 7 0.00125 8 0.000623 11 0.0000777 14 0.00000971 15 0.00000486 filter enable. 0 High Pass Filter Off 1 |
| 0                  |                 |                                                                                                                                                                                  |
|                    | High-pass       |                                                                                                                                                                                  |
| (R/W)              |                 | High Pass Filter On                                                                                                                                                              |
|                    |                 | 9 0.000311                                                                                                                                                                       |
|                    | 10              | 0.000155                                                                                                                                                                         |
|                    |                 | 0.0000389                                                                                                                                                                        |
|                    | 12 13 0.0000194 |                                                                                                                                                                                  |

## Software Reset Register

The PDM\_RESET register (read/write) controls the software reset operations.

Figure 19-9: PDM\_RESET Register Diagram

<!-- image -->

Table 19-7: PDM\_RESET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 0                  | SOFT       | Software reset not including register settings. |
| (RX/W)             | SOFT       | 0 N/A                                           |
| (RX/W)             | SOFT       | 1 Write once to soft reset                      |

## Serial Port Control0 Register

The PDM\_SP\_CTL0 register (read/write) controls serial port mode, data format and slot width.

Figure 19-10: PDM\_SP\_CTL0 Register Diagram

<!-- image -->

Table 19-8: PDM\_SP\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name                 | Description/Enumeration                                                                                    |
|--------------------|--------------------------|------------------------------------------------------------------------------------------------------------|
| 9                  | BCLK_POL                 | Serial port - selects bclk polarity.                                                                       |
| 8 (R/W)            | LRCLK_POL                | Serial port - selects lrclk polarity. 0 Normal Polarity 1 Inverted Polarity                                |
| 6 (R/W)            | TRI_STATE CH0_SLOT_WIDTH | Serial port output - tri-state enable. 0 tri-state disabled 1 tri-state enabled                            |
| 5:4 (R/W)          |                          | Serial port - selects slot width. 0 32 BCLK's slot                                                         |
| 3:1 (R/W)          |                          | per 1 16 BCLK's per slot 2 24 BCLK's per slot                                                              |
|                    | DATA_FORMAT              | Serial port - selects data format. 0 Typical I2S mode, delay by 1 1 Left Justified,delay by 0 2 Delay by 8 |

Table 19-8: PDM\_SP\_CTL0 Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration         |
|-----------|------------|---------------------------------|
| (Access)  |            |                                 |
|           |            | 4 Delay by 16                   |
| 0         | SAI_MODE   | Serial port - selects SAI mode. |
| (R/W)     |            | 0 STEREO (I2S, LJ or RJ)        |
|           |            | 1 TDM                           |

## Serial Port Control1 Register

The PDM\_SP\_CTL1 register (read/write) controls the mapping of channel 0/1/2/3 to TDM slots.

Figure 19-11: PDM\_SP\_CTL1 Register Diagram

<!-- image -->

Table 19-9: PDM\_SP\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 31:28              | CH3_SLOT   | Serial Port Channel 3 Slot Selection. |

Table 19-9: PDM\_SP\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                             | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
|                    |            | 15                                                                                                                          | Map channel to TDMslot 15                                                                                                   |
| 24 (R/W)           | CH3_DRV    | Serial Port Channel 3 drive select. This determines whether the associated channel gets driven in its assigned slot or not. | Serial Port Channel 3 drive select. This determines whether the associated channel gets driven in its assigned slot or not. |
|                    |            | 0                                                                                                                           | Channel will not be driven on serial port                                                                                   |
|                    |            | 1                                                                                                                           | Channel will be driven on serial port in selected output slot.                                                              |
| 23:20              | CH2_SLOT   | Serial Port Channel 2 Slot Selection.                                                                                       | Serial Port Channel 2 Slot Selection.                                                                                       |
| (R/W)              |            | 0                                                                                                                           | Map channel to TDMslot 0/I2S Left                                                                                           |
|                    |            | 1                                                                                                                           | Map channel to TDMslot 1/I2S Right                                                                                          |
|                    |            | 2                                                                                                                           | Map channel to TDMslot 2                                                                                                    |
|                    |            | 3                                                                                                                           | Map channel to TDMslot 3                                                                                                    |
|                    |            | 4                                                                                                                           | Map channel to TDMslot 4                                                                                                    |
|                    |            | 5                                                                                                                           | Map channel to TDMslot 5                                                                                                    |
|                    |            | 6                                                                                                                           | Map channel to TDMslot 6                                                                                                    |
|                    |            | 7                                                                                                                           | Map channel to TDMslot 7                                                                                                    |
|                    |            | 8                                                                                                                           | Map channel to TDMslot 8                                                                                                    |
|                    |            | 9                                                                                                                           | Map channel to TDMslot 9                                                                                                    |
|                    |            | 10                                                                                                                          | Map channel to TDMslot 10                                                                                                   |
|                    |            | 11                                                                                                                          | Map channel to TDMslot 11                                                                                                   |
|                    |            | 12                                                                                                                          | Map channel to TDMslot 12                                                                                                   |
|                    |            | 13                                                                                                                          | Map channel to TDMslot 13                                                                                                   |
|                    |            | 14                                                                                                                          | Map channel to TDMslot 14                                                                                                   |
|                    |            | 15                                                                                                                          | Map channel to TDMslot 15                                                                                                   |
| 16 (R/W)           | CH2_DRV    | Serial Port Channel 2 drive select. This determines whether the associated channel gets driven in its assigned slot or not. | Serial Port Channel 2 drive select. This determines whether the associated channel gets driven in its assigned slot or not. |
|                    |            | 0                                                                                                                           | Channel will not be driven on serial port                                                                                   |
|                    |            | 1                                                                                                                           | Channel will be driven on serial port in selected output slot.                                                              |
| 15:12              | CH1_SLOT   | Serial Port Channel 1 Slot Selection.                                                                                       | Serial Port Channel 1 Slot Selection.                                                                                       |
| (R/W)              |            | 0                                                                                                                           | Map channel to TDMslot 0/I2S Left                                                                                           |
|                    |            | 1                                                                                                                           | Map channel to TDMslot 1/I2S Right                                                                                          |

Table 19-9: PDM\_SP\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                             | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
|                    |            | 2                                                                                                                           | Map channel to TDMslot 2                                                                                                    |
|                    |            | 3                                                                                                                           | Map channel to TDMslot 3                                                                                                    |
|                    |            | 4                                                                                                                           | Map channel to TDMslot 4                                                                                                    |
|                    |            | 5                                                                                                                           | Map channel to TDMslot 5                                                                                                    |
|                    |            | 6                                                                                                                           | Map channel to TDMslot 6                                                                                                    |
|                    |            | 7                                                                                                                           | Map channel to TDMslot 7                                                                                                    |
|                    |            | 8                                                                                                                           | Map channel to TDMslot 8                                                                                                    |
|                    |            | 9                                                                                                                           | Map channel to TDMslot 9                                                                                                    |
|                    |            | 10                                                                                                                          | Map channel to TDMslot 10                                                                                                   |
|                    |            | 11                                                                                                                          | Map channel to TDMslot 11                                                                                                   |
|                    |            | 12                                                                                                                          | Map channel to TDMslot 12                                                                                                   |
|                    |            | 13                                                                                                                          | Map channel to TDMslot 13                                                                                                   |
|                    |            | 14                                                                                                                          | Map channel to TDMslot 14                                                                                                   |
|                    |            | 15                                                                                                                          | Map channel to TDMslot 15                                                                                                   |
| 8 (R/W)            | CH1_DRV    | Serial Port Channel 1 drive select. This determines whether the associated channel gets driven in its assigned slot or not. | Serial Port Channel 1 drive select. This determines whether the associated channel gets driven in its assigned slot or not. |
| 8 (R/W)            | CH1_DRV    | 0                                                                                                                           | Channel will not be driven on serial port                                                                                   |
| 8 (R/W)            | CH1_DRV    | 1                                                                                                                           | Channel will be driven on serial port in selected output slot.                                                              |
| 7:4 (R/W)          | CH0_SLOT   | Serial Port Channel 0 Slot Selection.                                                                                       | Serial Port Channel 0 Slot Selection.                                                                                       |
| 7:4 (R/W)          | CH0_SLOT   | 0                                                                                                                           | Map channel to TDMslot 0/I2S Left                                                                                           |
| 7:4 (R/W)          | CH0_SLOT   | 1                                                                                                                           | Map channel to TDMslot 1/I2S Right                                                                                          |
| 7:4 (R/W)          | CH0_SLOT   | 2                                                                                                                           | Map channel to TDMslot 2                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 3                                                                                                                           | Map channel to TDMslot 3                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 4                                                                                                                           | Map channel to TDMslot 4                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 5                                                                                                                           | Map channel to TDMslot 5                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 6                                                                                                                           | Map channel to TDMslot 6                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 7                                                                                                                           | Map channel to TDMslot 7                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 8                                                                                                                           | Map channel to TDMslot 8                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 9                                                                                                                           | Map channel to TDMslot 9                                                                                                    |
| 7:4 (R/W)          | CH0_SLOT   | 10                                                                                                                          | Map channel to TDMslot 10                                                                                                   |

Table 19-9: PDM\_SP\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
|                    |            | 12                                                                                                                          | Map channel to TDMslot 12                                                                                                   |
|                    |            | 13                                                                                                                          | Map channel to TDMslot 13                                                                                                   |
|                    |            | 14                                                                                                                          | Map channel to TDMslot 14                                                                                                   |
|                    |            | 15                                                                                                                          | Map channel to TDMslot 15                                                                                                   |
| 0 (R/W)            | CH0_DRV    | Serial Port Channel 0 drive select. This determines whether the associated channel gets driven in its assigned slot or not. | Serial Port Channel 0 drive select. This determines whether the associated channel gets driven in its assigned slot or not. |
|                    |            | 0                                                                                                                           | Channel will not be driven on serial port                                                                                   |
|                    |            | 1                                                                                                                           | Channel will be driven on serial port in selected output slot.                                                              |