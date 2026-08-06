# Housekeeping ADC (HADC)

<!-- source: 017_Housekeeping_ADC_HADC.pdf | original pages 731–745 -->

## 15   Housekeeping ADC (HADC)

The Housekeeping ADC is a 12-bit (with 10-bit accuracy), successive approximation ADC. It operates from single supply and features throughput rates up to 1 MSPS. The HADC can be used for the collection of housekeeping parameters like voltages, temperatures in the system or for any general-purpose use as well.

NOTE: HADC is used by the TMU for temperature monitor, so it may be unavailable during temperature conversion. For details, refer to the TMU and HADC section.

## HADC Features

The HADC supports following features:

- 12-bit ADC core with built-in sample and hold
- ENOB = 10 bit
- 8 input channels which can be extended to 15 channels by putting an external channel multiplexer
- Throughput rates up to 1 MSPS
- Single ended operation
- External reference nominal at 1.8 V
- Analog input 0 V to 1.8 V.
- Selectable ADC clock frequency through a pre-scaler
- Conversion type adaptable to each application-allows single or continuous conversion with option of auto-scan
- Auto sequencing capability provides up to 8 auto-conversions in a single session. Each conversion can be programmed to select any of the available input channels.

## HADC Functional Description

The HADC provides the analog to digital conversion capability for general-purpose housekeeping tasks, such as voltage and temperature monitoring. The core of HADC is a 12-bit SAR ADC, providing multiple analog input channels.

The HADC has the following functionality:

## Fixed and continuous conversion modes

ADC converts the input channel sequence for a fixed number of times or continuously converts an input channel sequence.

## Auto scanning

All the input channels can be sampled in a sequential manner.

## Channel sequence programming

The sequence of a channel can be selected by programming the channel mask register. If the bit corresponding to the channel is programmed to zero, that channel is included in the auto-scan chain.

## ADSP-SC59x HADC Register List

The Housekeeping ADC (HADC) provides a general purpose, multi-channel successive approximation A-to-D converter. A set of registers governs HADC operations. For more information on HADC functionality, see the HADC register descriptions.

Table 15-1: ADSP-SC59x HADC Register List

| Name          | Description             |
|---------------|-------------------------|
| HADC_CHAN_MSK | Channel Mask Register   |
| HADC_CTL      | Control Register        |
| HADC_DATA[nn] | Channel Data Registers  |
| HADC_IMSK     | Interrupt Mask Register |
| HADC_STAT     | Status Register         |

## ADSP-SC59x HADC Interrupt List

Table 15-2: ADSP-SC59x HADC Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|            186 | HADC0_EVT | HADC0 Event   | Edge          |               |

## ADSP-SC59x HADC Trigger List

Table 15-3: ADSP-SC59x HADC Trigger List Generators

|   Trigger ID | Name      | Description                   | Sensitivity   |
|--------------|-----------|-------------------------------|---------------|
|           33 | HADC0_EOC | HADC0 HADC0 End of Conversion | Edge          |

Table 15-4: ADSP-SC59x HADC Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## HADC Definitions

The following definitions are helpful when using the HADC module.

## Auto-scan

Auto-scan is a feature which allows the multiple channels to be scanned and converted in sequence one after the other.

## HADC Wakeup Time

It is the time required by the module after coming out of power down before it can start converting.

## Signal-to-Noise and Distortion Ratio (SINAD)

The measured ratio of signal-to-noise and distortion at the output of the ADC. The signal is the rms amplitude of the fundamental. Noise is the sum of all non-fundamental signals up to half the sampling frequency (f S /2), excluding dc. The ratio depends on the number of quantization levels in the digitization process; the more levels, the smaller the quantization noise. The theoretical signal-to-noise and distortion ratio for an ideal N-bit converter with a sine wave input is given by:

Signal-to-(Noise + Distortion) = (6.02 N + 1.76) dB

## Total Harmonic Distortion (THD)

The ratio of the rms sum to the harmonics to the fundamental.

## Peak Harmonic or Spurious Noise

The ratio of the rms value of the next largest component in the ADC output spectrum (up to f S /2 and excluding dc) to the rms value of the fundamental. Typically, the value of this specification is determined by the largest harmonic in the spectrum, but for ADCs where the harmonics are buried in the noise floor, it is a noise peak.

## Integral Nonlinearity

The maximum deviation from a straight line passing through the endpoints of the ADC transfer function. The endpoints are zero scale, a point 1 LSB below the first code transition, and full scale, a point 1 LSB above the last code transition.

## Differential Nonlinearity

The difference between the measured and the ideal 1 LSB change between any two adjacent codes in the ADC.

## Offset Error

The deviation of the first code transition (00...000) to (00...001) from the ideal-that is, GND1 + 1 LSB.

## Offset Error Match

The difference in offset error between any two channels.

## Gain Error

The deviation of the last code transition (111...110) to (111...111) from the ideal (that is, REFIN - 1 LSB) after the offset error has been adjusted out.

## Gain Error Matching

The difference in gain error between any two channels.

## Power Supply Rejection Ratio (PSRR)

PSRR is defined as the ratio of the power in the ADC output at full-scale frequency, f, to the power of a 100 mV pp sine wave applied to the ADC VDD supply of frequency, f S . The frequency of the input varies from 5 kHz to 25 MHz. PSRR (dB) = 10 log(Pf/Pf S ) where: Pf is the power at frequency, f, in the ADC output. Pf S  is the power at frequency, f S , in the ADC output.

## HADC Block Diagram

The HADC Block Diagram figure shows the functional blocks within the HADC and the interface to the processor core and the peripherals. The HADC has an internal channel multiplexer that is controlled by a programmable sequencer which selects the desired channel or sequence of channels.

Figure 15-1: HADC Block Diagram

<!-- image -->

## HADC Signal Descriptions

The HADC Signal Descriptions table provides descriptions of the signals used by the HADC.

Table 15-5: HADC Signal Descriptions

| Signal Name   | Signal Description                                                               |
|---------------|----------------------------------------------------------------------------------|
| AVDD          | ADC I/O supply                                                                   |
| AVSS          | I/O ground for analog blocks - shorted with other ground pins within the package |
| VREFP         | External reference for ADC                                                       |
| VREFN         | Ground reference for ADC                                                         |
| VINn          | Analog input at channel n                                                        |

## HADC Architectural Concepts

The HADC is based on a 12-bit SAR ADC that provides a simple register-based access model to obtain the results of conversion. The digital front end of the HADC provides a set of registers to configure the mode of operation, sampling frequency, and input channel selection control. The ADC supports multiple input analog channels which can be individually selected or deselected for conversion. The results of each analog channel are stored in a register. The core can access the register to read the conversion results once the conversion is complete. The HADC also provides the interrupts on completion of each channel conversion to avoid polling by the core. The following sections provide more details about the architecture of the HADC.

## Converter Operation

The housekeeping ADC is a 12-bit successive approximation ADC based around a capacitive DAC. The ADC Acquisition Phase figure and the ADC Conversion Phase figure show simplified schematics of the ADC. The ADC is comprised of control logic, SAR, and a capacitive DAC. The components are used to add and subtract fixed amounts of charge from the sampling capacitor to bring the comparator back into a balanced condition. The ADC Conversion Phase figure shows the ADC during its acquisition phase. SW2 is closed and SW1 is in Position A. The

comparator is held in a balanced condition and the sampling capacitor acquires the signal on the selected V IN  channel.

Figure 15-2: ADC Acquisition Phase

<!-- image -->

When the ADC starts a conversion (see the ADC Conversion Phase figure), SW2 opens and SW1 moves to Position B, causing the comparator to become unbalanced. The control logic and the capacitive DAC are used to add and subtract fixed amounts of charge to bring the comparator back into a balanced condition. When the comparator is rebalanced, the conversion is complete. The control logic generates the ADC output code.

Figure 15-3: ADC Conversion Phase

<!-- image -->

## Auto-Scan

The HADC features auto-scan mode where all the input channels can be sampled in a sequential manner. The number of channels enabled in auto-scan mode can be selected by programming the HADC\_CHAN\_MSK register. If the bit corresponding to the channel is set high, that particular channel is masked, and is not included in the auto-scan chain. In this way programs can sample all, none, or a selected set of channels by writing a high or a low for the individual channel. Auto sequencing allows the system to convert the same channel multiple times, allowing programs to perform oversampling algorithms.

For example, if the HADC\_CHAN\_MSK register bits [3:0] are set to 1101, then channel 0, channel 2 and channel 3 are not included in the auto-scan chain. Whether the conversion is a single or fixed number or continuous depends on the status of HADC\_CTL.CONT bit. If this bit is low, the HADC\_CTL.FIXEDCNV bits determine the number of sequence conversions.

The maximum number of fixed sequence conversions is 15. By default, the first eight channels of the HADC are enabled and extended channels are masked. An extended channel is the increased number of channels when a single channel of ADC is externally multiplexed, effectively increasing the total number of available channels. The total number of channels supported is 15. The program must configure the HADC\_CHAN\_MSK register to enable any desired channel.

## Channel Sequence Programming

The sequence of a channel can be selected by programming the HADC\_CHAN\_MSK register. If the bit corresponding to the channel is programmed to zero, that channel is included in the auto-scan chain. If the program must get the conversion results from a particular channel, then the bit corresponding to that channel should be zero.

Channels 8-15 are virtual channels; in that they can connect an external multiplexer and increase the effective number of channels. As the output of the external multiplexer is routed to channel 7, this channel is not available (or converted) separately when the virtual channels are not masked in the HADC\_CHAN\_MSK register. The auto-scan section has more details.

## ADC Transfer Function

The output coding is straight binary for the analog input channel conversion. The designed code transitions occur at successive LSB values (that is, 1 LSB, 2 LSBs, and so forth). The LSB size is V REF /4096 for the HADC. The ADC Transfer Function figure shows the ideal transfer characteristic for straight binary coding.

Figure 15-4: ADC Transfer Function

<!-- image -->

## Results

The HADC takes 20 cycles of f SAMPLE  for one channel conversion. (The value of the HADC\_CTL.FDIV bit field determines f SAMPLE ). The time taken to complete one sequence depends on the number of channels in the autoscan chain. There is a latency of 1 cycle from the time the channel is selected internally to sample to the time the data is ready. After the end of each channel conversion, the data is written into the corresponding data register. An interrupt is generated (if the interrupt mask is not enabled) to signal that the data is ready for that particular channel.

## HADC Operating Modes

The HADC has two modes of operation described in the following sections.

## Fixed Conversion Mode

In this mode, the ADC converts the input channel sequence for a fixed number of times. The frequency is configured in the HADC\_CTL.FIXEDCNV bit field. T o use this mode, clear the HADC\_CTL.CONT bit.

## Continuous Conversion Mode

In this mode ADC continuously converts an input channel sequence, when the HADC\_CTL.STARTCNV bit is held high. To use this mode, set the HADC\_CTL.CONT bit.

## HADC Event Control

The HADC generates different events depending on the state of the ADC and the status of channel conversions. It can generate an event for each of the following conditions:

- When ADC is ready for conversion
- At the end of sequence conversion
- At the end of each individual channel conversion

Each of these events can generate an interrupt. To generate an interrupt on any desired event, clear the respective bit in the HADC\_IMSK register.

## HADC Programming Model

Following sections provide some guidelines for HADC programming.

## Powering Up the HADC

To power-up the HADC, program the following bits in the HADC\_CTL register.

- Deassert the HADC\_CTL.PD bit (HADC power down)
- Set the HADC\_CTL.NRST bit (Reset)
- Set the HADC\_CTL.ENLS bit (Enable level shifters)

After deasserting HADC\_CTL.PD , the HADC requires a finite wake-up time (t WAKEUP ) before it can start converting. The HADC requires only two f SAMPLE  clocks from the assertion of the HADC\_CTL.NRST bit before the module is ready to convert. ( HADC\_CTL.PD is low). Poll the HADC\_STAT.RDY bit. A 1 on this bit indicates that the HADC is ready to convert data.

## Enabling the HADC

Setting the HADC\_CTL.STARTCNV bit enables the HADC. When this bit is kept high, the HADC can work in either continuous or fixed conversion mode. After the HADC\_CTL.STARTCNV bit is set =1, the HADC\_CHAN\_MSK can still be re-programmed, but the new sequence only comes into effect after the current sequence conversion is complete.

## ADSP-SC59x HADC Register Descriptions

Housekeeping ADC (HADC) contains the following registers.

Table 15-6: ADSP-SC59x HADC Register List

| Name          | Description           |
|---------------|-----------------------|
| HADC_CHAN_MSK | Channel Mask Register |

Table 15-6: ADSP-SC59x HADC Register List (Continued)

| Name          | Description             |
|---------------|-------------------------|
| HADC_CTL      | Control Register        |
| HADC_DATA[nn] | Channel Data Registers  |
| HADC_IMSK     | Interrupt Mask Register |
| HADC_STAT     | Status Register         |

## Channel Mask Register

The HADC\_CHAN\_MSK register provides bits that mask each channel. The LSB corresponds to channel 0 , the second LSB to channel 1 and so on. If a mask bit is set, the corresponding channel is not converted. By default, channels 0-7 are not masked and the extended channels 8-15 are masked.

Figure 15-5: HADC\_CHAN\_MSK Register Diagram

<!-- image -->

Table 15-7: HADC\_CHAN\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Mask for Each Channel. The HADC_CHAN_MSK.VALUE bit field is the mask bit for each channel. The first MSB corresponds to channel 15, the second MSB to channel 14 and so on. If the mask is set for a particular channel, that channel is not converted. By default, channels 0-7 are not masked and the extended channels 8-15 are masked. The first MSB corre- sponds to channel 7, the second MSB to channel 6 and so on. If the mask is set for a particular channel, that channel is not converted. By default, channels 0-7 are not masked. |

## Control Register

The HADC\_CTL register contains control bits that configure various module settings start or reset the module.

Figure 15-6: HADC\_CTL Register Diagram

<!-- image -->

Table 15-8: HADC\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | ENLS       | Enable Level Shifters. Setting the HADC_CTL.ENLS bit enables the level shifters, allowing the HADC ana- log side (which works in the VDD_EXT domain) to work with the digital core and interface is (in the VDD_INT domain).                                                                                                         |
| 12 (R/W)           | DOUTOREOCB | Serial data on DOUT. If the HADC_CTL.DOUTOREOCB bit =1, serial data arrives on the EOC_DOUT pin. If this bit =0 (default) it acts as an EOC only if the external multiplexer is con- nected.                                                                                                                                         |
| 11:8 (R/W)         | FIXEDCNV   | Fixed Conversion. The HADC_CTL.FIXEDCNV bit configures the number of conversions = FIX- EDCNV. This value determines how many times a sequence is converted when the HADC is in fixed conversion mode. This only applies when the HADC_CTL.CONT bit =0. Before changing the HADC_CTL.FIXEDCNV bit, clear the HADC_CTL.NRST bit (=0). |

Table 15-8: HADC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | CONT       | Continuous Conversion. When the HADC_CTL.CONT bit =0, the ADC converts a sequence for a fixed num- ber of times. This number is configured using the HADC_CTL.FIXEDCNV bit field. When the HADC_CTL.CONT bit =1, the ADC continuously converts a given se- quence, provided the HADC_CTL.STARTCNV is held high. | Continuous Conversion. When the HADC_CTL.CONT bit =0, the ADC converts a sequence for a fixed num- ber of times. This number is configured using the HADC_CTL.FIXEDCNV bit field. When the HADC_CTL.CONT bit =1, the ADC continuously converts a given se- quence, provided the HADC_CTL.STARTCNV is held high. |
| 7 (R/W)            | CONT       | 0                                                                                                                                                                                                                                                                                                               | ADC converts sequence for fixed number of times                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | CONT       | 1                                                                                                                                                                                                                                                                                                               | ADC continuously converts given sequence                                                                                                                                                                                                                                                                        |
| 6:3 (R/W)          | FDIV       | Frequency Divider. The HADC_CTL.FDIV bit field configures the f_SAMPLE=f_CLK/(FDIV+1). Select f_CLK and HADC_CTL.FDIV values so that f_SAMPLE is in range of 50 kHz to 22.5 MHz. The minimum value for HADC_CTL.FDIV is 1. Before changing the HADC_CTL.FDIV bits, clear the HADC_CTL.NRST bit.                 | Frequency Divider. The HADC_CTL.FDIV bit field configures the f_SAMPLE=f_CLK/(FDIV+1). Select f_CLK and HADC_CTL.FDIV values so that f_SAMPLE is in range of 50 kHz to 22.5 MHz. The minimum value for HADC_CTL.FDIV is 1. Before changing the HADC_CTL.FDIV bits, clear the HADC_CTL.NRST bit.                 |
| 2 (R/W)            | STARTCNV   | Start conversion. The HADC_CTL.STARTCNV bit needs to be set for the ADC to start converting da- ta. If the ADC is running in non continuous mode, it is reset by hardware after the desired number of conversions is completed.                                                                                 | Start conversion. The HADC_CTL.STARTCNV bit needs to be set for the ADC to start converting da- ta. If the ADC is running in non continuous mode, it is reset by hardware after the desired number of conversions is completed.                                                                                 |
| 2 (R/W)            | STARTCNV   | 0                                                                                                                                                                                                                                                                                                               | No action                                                                                                                                                                                                                                                                                                       |
| 2 (R/W)            | STARTCNV   | 1                                                                                                                                                                                                                                                                                                               | Start converting                                                                                                                                                                                                                                                                                                |
| 1 (R/W)            | PD         | Power down. The HADC_CTL.PD bit powers down the analog circuitry of the ADC. After this bit                                                                                                                                                                                                                     | Power down. The HADC_CTL.PD bit powers down the analog circuitry of the ADC. After this bit                                                                                                                                                                                                                     |
| 1 (R/W)            | PD         | 0                                                                                                                                                                                                                                                                                                               | No action                                                                                                                                                                                                                                                                                                       |
| 1 (R/W)            | PD         | 1                                                                                                                                                                                                                                                                                                               | Power down the analog circuitry of the ADC                                                                                                                                                                                                                                                                      |
| 0 (R/W)            | NRST       | Reset. The HADC_CTL.NRST bit resets the ADC.                                                                                                                                                                                                                                                                    | Reset. The HADC_CTL.NRST bit resets the ADC.                                                                                                                                                                                                                                                                    |
| 0 (R/W)            | NRST       | 0                                                                                                                                                                                                                                                                                                               | Reset the ADC                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | NRST       | 1                                                                                                                                                                                                                                                                                                               | No action                                                                                                                                                                                                                                                                                                       |

## Channel Data Registers

The HADC\_DATA[nn] registers NN ranges from 0-14. Each corresponding to an ADC channel. ADC\_DATA\_0 corresponds to channel 0, ADC\_DATA\_1 to channel 1 and so on.

Figure 15-7: HADC\_DATA[nn] Register Diagram

<!-- image -->

Table 15-9: HADC\_DATA[nn] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/NW)        | VALUE      | Converted Data. The HADC_DATA[nn].VALUE bit field contains the digital code for the sampled analog value. Each channel has its own data register. |

## Interrupt Mask Register

The HADC\_IMSK register masks (disables) or unmasks (enables) the interrupts as programmed.

Figure 15-8: HADC\_IMSK Register Diagram

<!-- image -->

Table 15-10: HADC\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | RDY        | Mask Interrupt when ADC Ready. The HADC_IMSK.RDY bit masks the interrupt generated when ADC is ready to con- vert.                                                                                                                                           |
| 16 (R/W)           | SEQ        | Mask Interrupt at End of Sequence. The HADC_IMSK.SEQ bit masks the interrupt which is generated at the end of se- quence completion.                                                                                                                         |
| 16 (R/W)           | SEQ        | 0 Interrupt is unmasked                                                                                                                                                                                                                                      |
| 15:0 (R/W)         | CHAN       | Channel Mask. The HADC_IMSK.CHAN bit field provides the interrupt mask bit for each channel.N ranges from 0-15. The MSB corresponds to channel 15, the second MSB to channel 14 and so on. If the bit is SET, interrupt is masked for corresponding channel. |

## Status Register

The HADC\_STAT register contains bits that provide status information on the HADC module.

Figure 15-9: HADC\_STAT Register Diagram

<!-- image -->

Table 15-11: HADC\_STAT Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/NW)          | TMUHADC_BUSY | Temperature Conversion Status. The HADC_STAT.TMUHADC_BUSY bit, if high, indicates to the system that tem- perature conversion is ongoing. All ADC conversion requests are ignored.                                                                                                                                                                                      |
| 19:4 (RX/W1C)      | INTRW1C      | Conversion Complete Interrupt. The HADC_STAT.INTRW1C bit field indicates when the corresponding ADC chan- nel completes conversion. Nranges from 0-15 and the MSB corresponds to channel 15, the second MSB to channel 14 and so on. Nranges from 0-7 and the MSB corre- sponds to channel 7, the second MSB to channel 6 and so on. These bits are sticky and are W1C. |
| 3 (RX/W1C)         | SEQOVRW1C    | End of Sequence Conversion. The HADC_STAT.SEQOVRW1C bit indicates the end of a sequence conversion and is a sticky status bit which is W1C.                                                                                                                                                                                                                             |
| 2 (RX/W1C)         | RDYW1C       | Ready to Convert. The HADC_STAT.RDYW1C bit is the stick version of the HADC_STAT.RDY bit.                                                                                                                                                                                                                                                                               |
| 0 (R/NW)           | RDY          | ADC Ready. The HADC_STAT.RDY bit is set (=1) when the ADC is ready to convert data.                                                                                                                                                                                                                                                                                     |