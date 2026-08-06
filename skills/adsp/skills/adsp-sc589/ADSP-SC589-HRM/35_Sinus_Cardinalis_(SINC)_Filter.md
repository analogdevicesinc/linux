## 32   Sinus Cardinalis (SINC) Filter

The sinus cardinalis (SINC) filter module processes four independent sigma-delta bit streams by applying a pair of SINC filters to each stream. A SINC filter converts the bit stream from a sigma-delta front-end modulator into a digital word representing the signal level presented to the modulator.

The filter consists of a set of integration and decimation stages implemented directly in logic for efficient execution. The SINC filter supports capture of current or voltage feedback signals from an isolating analog-to-digital converter (ADC). Each modulator bit stream connects to two SINC filters: a primary filter for controlling feedback; a secondary filter for overcurrent detection. The SINC module includes four filter channels and two modulator clock generators.

## SINC Filter Features

The SINC features include:

- Four-bit stream filter channels for current or voltage feedback signal processing
- Each channel includes two SINC filter pairs:
- A primary filter for feedback signal processing
- A secondary filter for overload detection
- Up to two modulator clock sources with phase control options
- Configuration of SINC filter channels according to a modulator clock selection
- Programmable order and decimation rates
- Primary filters:
- Programmable bias and gain with output saturation
- Dedicated direct memory access (DMA) channels with data interleaving and programmable data ready output triggers
- Secondary filters:
- Detecting a fault when signals exceed amplitude and duration values

- Registers preserving the eight most recent samples before a fault event
- Multiple interrupt trigger sources for overload fault and data overflow events

## SINC Functional Description

The SINC filter has the following functionality:

## Digital filter

The filter removes the modulator sample clock and recovers a digital value of the sampled signal.

## DC gain and data resolution

The DC gain of the digital filter is a function of the order and decimation rate.

## Frequency response

The frequency response of the filter depends on the order, decimation rate, and modulator clock frequency.

## Output scaling

The output scaling and postprocessing functions embedded in the SINC filter blocks differ, depending on the function.

## ADSP-SC58x SINC Register List

The SINC filter module processes four independent sigma-delta bit streams by applying a pair of SINC filters to each stream. A SINC filter converts the bit stream from a sigma-delta front-end modulator into a digital word representing the signal level presented to the modulator. Each modulator bit stream connects to two SINC filters: a primary filter for controlling feedback, and a secondary filter for overcurrent detection. A set of registers governs SINC operations. For more information on SINC functionality, see the SINC register descriptions.

Table 32-1: ADSP-SC58x SINC Register List

| Name          | Description                        |
|---------------|------------------------------------|
| SINC_BIAS0    | Bias for Group 0 Register          |
| SINC_BIAS1    | Bias for Group 1 Register          |
| SINC_CLK      | Clock Control Register             |
| SINC_CTL      | Control Register                   |
| SINC_HIS_STAT | History Status Register            |
| SINC_LEVEL0   | Level Control for Group 0 Register |
| SINC_LEVEL1   | Level Control for Group 1 Register |

Table 32-1: ADSP-SC58x SINC Register List (Continued)

| Name               | Description                                        |
|--------------------|----------------------------------------------------|
| SINC_LIMIT0        | (Amplitude) Limits for Secondary Filter 0 Register |
| SINC_LIMIT1        | (Amplitude) Limits for Secondary Filter 1 Register |
| SINC_LIMIT2        | (Amplitude) Limits for Secondary Filter 2 Register |
| SINC_LIMIT3        | (Amplitude) Limits for Secondary Filter 3 Register |
| SINC_P0SEC_HIST[n] | Pair 0 Secondary (Filter) History n Register       |
| SINC_P1SEC_HIST[n] | Pair 1 Secondary (Filter) History n Register       |
| SINC_P2SEC_HIST[n] | Pair 2 Secondary (Filter) History n Register       |
| SINC_P3SEC_HIST[n] | Pair 3 Secondary (Filter) History n Register       |
| SINC_PHEAD0        | Primary (Filters) Head for Group 0 Register        |
| SINC_PHEAD1        | Primary (Filters) Head for Group 1 Register        |
| SINC_PPTR0         | Primary (Filters) Pointer for Group 0 Register     |
| SINC_PPTR1         | Primary (Filters) Pointer for Group 1 Register     |
| SINC_PTAIL0        | Primary (Filters) Tail for Group 0 Register        |
| SINC_PTAIL1        | Primary (Filters) Tail for Group 1 Register        |
| SINC_RATE0         | Rate Control for Group 0 Register                  |
| SINC_RATE1         | Rate Control for Group 1 Register                  |
| SINC_STAT          | Status Register                                    |

## ADSP-SC58x SINC Interrupt List

Table 32-2: ADSP-SC58x SINC Interrupt List

|   Interrupt ID | Name       | Description   | Sensitivity   | DMA Channel   |
|----------------|------------|---------------|---------------|---------------|
|            144 | SINC0_STAT | SINC0 Status  | Level         |               |

## ADSP-SC58x SINC Trigger List

Table 32-3: ADSP-SC58x SINC Trigger List Masters

|   Trigger ID | Name          | Description                     | Sensitivity   |
|--------------|---------------|---------------------------------|---------------|
|           49 | SINC0_P0_OVLD | SINC0 Pair 0 Overload Indicator | Edge          |
|           50 | SINC0_P1_OVLD | SINC0 Pair 1 Overload Indicator | Edge          |
|           51 | SINC0_P2_OVLD | SINC0 Pair 2 Overload Indicator | Edge          |
|           52 | SINC0_P3_OVLD | SINC0 Pair 3 Overload Indicator | Edge          |

Table 32-3: ADSP-SC58x SINC Trigger List Masters (Continued)

|   Trigger ID | Name        | Description       | Sensitivity   |
|--------------|-------------|-------------------|---------------|
|           53 | SINC0_DATA0 | SINC0 Data Move 0 | Edge          |
|           54 | SINC0_DATA1 | SINC0 Data Move 1 | Edge          |

Table 32-4: ADSP-SC58x SINC Trigger List Slaves

|   Trigger ID | Name        | Description                   | Sensitivity   |
|--------------|-------------|-------------------------------|---------------|
|           42 | SINC0_SYNC0 | SINC0 Synchronization Input 0 | Pulse         |
|           43 | SINC0_SYNC1 | SINC0 Synchronization Input 1 | Pulse         |

## SINC Definitions

To make the best use of the SINC, it is useful to understand the following terms.

## Decimation

Decimation is the process of discarding samples from a data stream.

## Decimation Rate

The decimation rate is the ratio of the filter input data rate to the filter output data rate.

## Filter Order

The SINC filter order is the number of integration and decimation stages in the filter.

## Modulator Order

The modulator order is the number of comparator and integrator stages in a sigma-delta modulator.

## Sigma-Delta Modulator

The sigma-delta modulator is an oversampling analog-to-digital conversion circuit that generates a digital bit stream whose pulse density is proportional to the analog voltage presented to the input.

## SINC Block Diagram

The SINC Block Diagram figure shows the functional blocks within the SINC module.

Figure 32-1: SINC Block Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000000_07363b380905ad9a2fa2f9fc9d7f9f26f7d31d019a5c729220c89fae73415c11.png)

The block diagram shows four SINC filter pairs (SINC0-3), one modulator clock source, and one bank of control registers (units). The module accepts four sigma-delta bit streams from the GPIO input pins and directs the modulator clock source of GROUP 0 to the GPIO output pin. A pulse-width modulation (PWM) signal synchronizes the modulator clock to optimize system performance. Each SINC filter pair includes the primary filter, secondary filter, DMA interface, and overload limit detection functions.

The primary SINC filter transmits its data to memory using DMA. The secondary SINC filter generates overload signals, which can be routed through the trigger routing unit (TRU) to trip a PWM modulator and generate an interrupt request.

The SINC filter pairs are assigned to control unit 0, where multiple channels of current or voltage-feedback share common filter parameters. The primary filters generate high-resolution signals for closing the feedback control loop. The secondary filters are for rapid-overload fault detection, require lower resolution, but a faster response. The primary and secondary filters have programmable order and decimation rates. The primary filters also have the programmable output gain stage, while the secondary filters have the programmable overload limit thresholds.

To use the primary and secondary filters, set up the filter parameters once, prior to using the filters. The feedback control algorithm reads the data from the primary filter directly from memory. A PWM interrupt request signal can generate the algorithm timing signal, or the SINC module generates a data trigger. The data history of the secondary filter is saved in buffer registers once an overload fault signal is detected. The data history supports fault diagnostics.

## SINC Architectural Concepts

The architecture of the SINC includes the following:

- Digital Filter
- DC Gain and Data Resolution
- Frequency Response
- Output Scaling

## Digital Filter

The SINC filter has a transfer function that lends itself to an implementation in digital logic, using a series of summation and decimation functions. The filter removes the modulator sample clock and recovers a digital value of the sampled signal. The filter design matches a bipolar SD modulator. The design produces a 50% pulse density for a 0V input, over 50% for positive inputs and less than 50% for negative inputs.

Figure 32-2: SINC Digital Filter

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000001_8d00a569f353b0c3fda8d9c7c270cd24ab7b09a640f67476aa963847a44a6ccf.png)

The digital filter is a set of accumulators driven by the modulator clock ( M\_CLK ), followed by a set of differentiators driven by the decimation clock ( D\_CLK ). The input accumulators convert the input bit stream into a multibyte word, while the output differentiators derive the average 1's density of the bit stream. The number of accumulator and differentiator stages can be three or four, depending on the order of the filter. The DC gain and bandwidth of the filter are functions of the filter order ( O ) and the decimation rate ( D ), which is the ratio of the modulator to the decimation clock.

The calculation of the transfer function of the SINC filter includes the product of the transfer functions for the accumulators and differentiators, and in the z domain. The following equation gives the calculation:

## DC Gain and Data Resolution

The DC gain of the digital filter is a function of the order and decimation rate. At 100% ones density input, each accumulator stage counts D pulses, and the gain of the filter is given as follows:

<!-- formula-not-decoded -->

The higher the decimation rate, the higher the resolution of the output data. The number of usable data bits is a function of the SNR ; the Filter Order versus Decimation table shows ENOB versus the decimation rate.

Table 32-5: ENOB versus Decimation

| Decimation   | Decimation   |    4 |     5 |     6 |     7 |     8 |    16 |    32 |    64 |   128 |   256 |
|--------------|--------------|------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
| O=3          | SNR (dB)     | 6.42 | 11.47 | 16.41 | 20.57 | 23.55 | 35.02 | 48.59 | 62.26 | 76.46 | 89.59 |
|              | ENOB         |  0.8 |   1.6 |   2.4 |   3.1 |   3.6 |   5.5 |   7.8 |  10.0 |  12.4 |  14.6 |
| O=4          | SNR (dB)     | 9.08 | 14.77 | 19.78 | 23.41 |  25.9 | 38.05 | 51.29 | 64.67 | 79.15 |       |
|              | ENOB         |  1.2 |   2.1 |   3.1 |   3.6 |   4.0 |   6.0 |   8.2 |  10.4 |  12.8 |       |

Notes: ENOB versus order and decimation rate.

Test conditions are for a 1.22 kHz tone and a 10 MHz modulator.

## Frequency Response

The frequency response of the filter depends on the order, decimation rate, and modulator clock frequency, f M . The equation is obtained by substituting e j x Ts for z in the transfer function, where Ts is the period of the modulator clock:

<!-- formula-not-decoded -->

The filter has a linear phase response with a constant group delay given by:

<!-- formula-not-decoded -->

The Frequency Response plots show zeros at multiples the decimation frequency, where the sin term in the numerator goes to zero. This response makes it possible to remove some PWM ripple components from the motor current waveform by matching the decimation frequency to the PWM switching frequency. There are some limitations at lower PWM frequencies based on available decimation rates. High decimation rates limit the bandwidth of the control loop because of the phase delay, which is 3 π radians at the decimation frequency.

<!-- formula-not-decoded -->

Figure 32-5: Frequency Response

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000002_09ed068c32cdeee1b0d9d29ffceb0127e20ae002586b669e35d6f18f6ddfea40.png)

## Output Scaling

The output scaling and postprocessing functions embedded in the SINC filter blocks differ, depending on the function. The primary filter used for feedback signal processing includes the output bias and scaling blocks to present a 16-bit signed integer to the control code. The scaling is required at decimation rates higher than 32 to keep the lower 16 bits of the output word.

The secondary filter supports overload detection functions. The secondary filter can detect signals crossing maximum and minimum thresholds. It has a glitch filter that only accepts faults with a minimum number of counts ( c ) within a certain count window ( w ). The secondary filter has no output scaling, so the minimum and maximum values in the overload registers must be calculated from the DC gain of the secondary filter. The response time to a step input is approximately 2 x O decimation clock cycles.

Figure 32-4: Output Scaling

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000003_e9f73870fc78503fe2e8506917bfb81749e9036b1b96e61b8fe21a415f3d606b.png)

## SINC Operating Modes

The SINC filter module has only one operating mode. The module generates the clock source for a sigma-delta modulator analog front end and filters the output data stream for the modulator. The primary SINC filter transfers its data to memory through DMA. The secondary SINC filter output generates an overload trigger signal that the SINC filter module can use as a PWM trip signal. The SINC control registers enable the module and set up the modulator clock sources, filter parameters, DMA transfers, and interrupts masks, as described in SINC Programming Model.

For self-testing purposes, the SINC data inputs can be driven by bit streams transferred from the SPORT0A port through DMA from a slave memory space (typically M4 main memory). The following programming selections are required to enable SINC self-test mode:

- [3:0] - set each bit to 1 for each SINC channel to be selected for test. If 1, the SINC channel input will be driven by the SPORT, not the associated GPIO.
- Enable filters to route SPORT data to SINC unit with the write: *pREG\_TESYS1\_SINC\_TEST = 0xf
- The SINC unit must be configured as follows.
- Timer Group 0; MCLK0
- The SPORT (A) must be configured as follows.
- External Clock
- 16 bit data recommended
- No multichannel mode
- Enable SPORT0\_A in transmit mode, external clock, internal FS, and I2S Mode *pREG\_SPORT0\_CTL\_A = 0x021548f1
- Any data transmitted on SPORT0 Data A channel is sent to SINC block.

## SINC Data Transfer Modes

The only mode of data transfer between the primary SINC filter and memory is through DMA ( See Primary DMA Configuration and Data Interrupts for more information. Reading the history registers for the secondary filter is the only way to transfer data between the secondary SINC filter and memory. See Overload Detection for more information.

## SINC Signal Modes

The SINC filter has an interrupt request signal and a number of triggers and status signals to indicate system events and errors.

- Primary data transfer trigger:

The SINC filter can generate a trigger after a user-specified number of primary output sets are transferred to memory. There is one trigger source for each filter group. See Primary DMA Configuration and Data Interrupts for more information.

- Secondary data overload trigger:

The SINC filter can generate a trigger when one of the secondary filters detects an overload condition. There is one trigger source for each secondary filter. See Overload Detection for more information.

- SINC status bits:

The SINC status bits indicate secondary filter overload errors, primary filter saturation errors, primary filter transfer count exceeded, and primary filter data buffer errors.

- Secondary filter overload errors:

A number of status bits indicate the type of error and the filter channel when a secondary filter detects an overload condition. The status bits SINC\_STAT.GLIM0 and SINC\_STAT.GLIM1 indicate the control group of the secondary filter that detected the overload. The status bits SINC\_STAT.MAX0 through SINC\_STAT.MAX3 indicate when the error a maximum limit on one of the secondary filter channels is passed, causing the error. The status bits SINC\_STAT.MIN0 through SINC\_STAT.MIN3 indicate when a minimum limit on one of the secondary filter channels is passed, causing the error.

- Primary filter data saturation errors:

A number of status bits indicate the group and filter channel when the SINC filter detects data saturation . The status bits SINC\_STAT.GSAT0 and SINC\_STAT.GSAT1 indicate the filter control group when the SINC filter detects data saturation. The status bits SINC\_STAT.PSAT0 through SINC\_STAT.PSAT3 indicate a primary filter channel that detects data saturation.

- Primary filter transfer count exceeded:

The status bits SINC\_STAT.PCNT0 and SINC\_STAT.PCNT1 are set every time a specified number of primary filter data sets for that filter group are transferred to memory. The primary filter data set for a group is the data for all the channels in the group. The specified number of data sets is the value in the

SINC\_LEVEL0.PCNT -SINC\_LEVEL1.PCNT bits. Write 1 to clear the bits before the next data transfer to generate a trigger.

- Primary filter data buffer errors:

A number of status bits indicate data buffer errors. The status bits SINC\_STAT.FOVF0 and SINC\_STAT.FOVF1 indicate the filter control group when there is a SINC data buffer overflow. This error occurs when a third sample is presented to the buffer before the first sample transfers to memory. The status bits SINC\_STAT.PFAB0 and SINC\_STAT.PFAB1 indicate the filter group when an error occurs while writing the data to memory.

- SINC status interrupt request:

There is a single SINC filter interrupt request output that can indicate secondary filter overload errors, primary filter data saturation, or primary filter data buffer overrun. There is one interrupt mask bit for each of these conditions per filter group. See Interrupt Masking for more information.

## SINC Event Control

The SINC provides status and error bits through different registers to signal the core about its state and various error conditions that occur during its operation. These conditions include:

- Interrupt status related to data overload, data saturation, data FIFO fault conditions
- Error status related to SINC operations
- History status (which do not generate interrupts) related to data FIFO operations

## SINC Interrupt Signals

The interrupt request and trigger signals to the SINC filter module include:

- One interrupt request signal, SINC\_STAT, triggered by fault events, such as detected overload limits and data transfer errors. Manage interrupt request generation with the masking bits in the SINC\_CTL register:
- Bits SINC\_CTL.ELIM0 -SINC\_CTL.ELIM1 can enable (unmask) interrupt request generation on overload faults when the SINC\_STAT.GLIM0 -SINC\_STAT.GLIM1 bit is set, respectively.
- Bits SINC\_CTL.ESAT0 -SINC\_CTL.ESAT1 can mask interrupt request generation on data saturation faults when the SINC\_STAT.GSAT0 -SINC\_STAT.GSAT1 bit is set, respectively.
- Bits SINC\_CTL.EFOVF0 -SINC\_CTL.EFOVF1 can mask interrupt request generation on data buffer overruns when the SINC\_STAT.FOVF0 -SINC\_STAT.FOVF1 bit is set, respectively.

The fault bits in the SINC\_STAT register must be cleared to clear the interrupt request.

- Two data count triggers, one trigger per each control group. The SINC filter module regularly uses the data count triggers to generate a software interrupt request or trigger an event. First, set the SINC\_CTL.EPCNT0 or SINC\_CTL.EPCNT1 masking bit to enable the data count trigger. Then, the TRU must assign the data count master ( SINC0\_DATA0-1 ) to an interrupt request input.

- Four overload triggers, one trigger per each channel. The SINC filter module can use overload triggers to trip the appropriate PWM block in the case of a fault. The overload trigger is always enabled, and the TRU must assign the masters ( SINC0\_P0\_OVLD through SINC0\_P4\_OVLD ) to the appropriate PWM trip input slave ( PWMn\_TRIP\_TRIGn ).

## SINC Status and Error Signals

The status and error signals related to SINC operations are as follows:

- SINC\_STAT signals:
- The amplitude and duration limit error signals for secondary SINC filters: SINC\_STAT.MAX0 through SINC\_STAT.MAX3 , SINC\_STAT.MIN0 through SINC\_STAT.MIN3 , and SINC\_STAT.GLIM0 -SINC\_STAT.GLIM1 .
- The output saturation error signals for primary SINC filters: SINC\_STAT.MAX0 through SINC\_STAT.MAX3 , SINC\_STAT.MIN0 through SINC\_STAT.MIN3 , and SINC\_STAT.GLIM0 -SINC\_STAT.GLIM1 .
- The output FIFO overflow error signals for primary SINC filters: SINC\_STAT.FOVF0 and SINC\_STAT.FOVF1 .
- The output count error signals for primary SINC filters: SINC\_STAT.PCNT0 and SINC\_STAT.PCNT1 .
- The SCB fabric-related error signals for primary SINC filters: SINC\_STAT.PFAB0 -SINC\_STAT.PFAB1 .
- SINC\_CLK signals:
- The phase shift signals for SINC modulator clocks: SINC\_CLK.MREQ0 -SINC\_CLK.MREQ1 .
- SINC\_HIS\_STAT signals:
- The history saved signals for secondary SINC filters: SINC\_HIS\_STAT.P0HISPTR through SINC\_HIS\_STAT.P3HISPTR , which indicate that the data history of the filter is saved in buffer registers due to a detected overload error signal.

## SINC Programming Model

The pin multiplexer enables the device input and output pins and connects the signals to the SINC module. Decide the filter grouping in advance. The filter parameters are defined according to the control register group.

Follow these steps to configure the filters:

1. Define the primary and secondary filter parameters by setting the appropriate bits in the control register for each filter channel group.

2. Set the upper and lower overload limits to maximum for each channel to avoid overload trips due to the filter startup transient.
3. Define the modulator clock frequency and startup mode.
4. Enable the SINC channels and assign them to the selected group of control registers.

Set the running overload limits after the filter settles, which is (order × decimation) modulator clock cycles after startup. When the filters are running, the module transfers its data to data RAM on the dedicated DMA channels. Once configured, the control registers do not need accessing, but the status and some data buffer registers typically are read after fault events.

In general, adjusting filter parameters during operation leads to unpredictable results. However, programs can write to the trigger and interrupt mask registers, as well as to the secondary threshold level registers during operation.

The DC gain of the converter subsystem depends on the gain of the input modulator ( GM ), filter order ( O ), and decimation rate ( D ). The primary filter has an output binary scalar ( s ) to fit data into a 16-bit range:

GM = 0.625 × (D O  ÷ 2 S )

## SINC Programming Concepts

Using the features and event control for the SINC to their greatest potential requires an understanding of some SINC-related concepts:

- Channel Configuration
- Trigger Masking
- Interrupt Masking
- Modulator Clock
- Filter Configuration
- Primary Filter Parameters
- Primary DMA Configuration and Data Interrupts
- Secondary Filter Parameters
- Overload Detection

## Channel Configuration

The control bits, SINC\_CTL.EN0 through SINC\_CTL.EN3 , configure SINC module channels. These control bits enable or disable the selected SINC filter channel and assign the channel to control register group 0.

## Trigger Masking

The SINC module has one data count trigger. The module can use the data count trigger to generate a software interrupt regularly or trigger an event. First, set the SINC\_CTL.EPCNT0 masking bit to enable the data count trigger. Then, the TRU must assign the data count master ( SINC\_DATn ) to an interrupt input.

There are also four overload triggers, one trigger per each channel. The SINC module can use overload triggers to trip the appropriate PWM block when there is a fault. The overload trigger is always enabled, and the TRU must assign the masters ( SINC0\_Pn\_OVLD ) to the appropriate PWM trip input slave ( PWMn\_TRIP\_TRIGn ).

## Interrupt Masking

The SINC filter can generate a SINC\_STAT interrupt signal when triggered by fault events, such as detected overload limits or data transfer errors.

Enable (unmask) interrupt generation with the SINC\_CTL register bits:

- The SINC\_CTL.ELIM0 bit can enable interrupt generation on overload faults when the SINC\_STAT.GLIM0 bit is set.
- The SINC\_CTL.ESAT0 bit can enable interrupt generation on data saturation faults when the SINC\_STAT.GSAT0 bit is set.
- The SINC\_CTL.EFOVF0 bit can enable interrupt generation on data buffer overruns when the SINC\_STAT.FOVF0 bit is set.

The fault bits in the SINC\_STAT register must be cleared to clear the interrupt.

## Modulator Clock

The SINC filter has one modulator clock source. The clock source can be set with an output frequency in the range of 1-20 MHz. The SINC module uses bits in the SINC\_CLK register to control the modulator clock output, frequency, and phase. The SINC module uses the SINC\_CLK.MCEN0 bit field to enable the modulator clock and control the startup behavior of the clock. Start the clock immediately or enable the clock on the first rising edge of an external trigger connected to the SINC0\_SYNCn input of the module. This action synchronizes the modulator clock with a PWM waveform source by routing a PWMn\_SYNC master to the SINC0\_SYNC0 slave using the TRU.

The target frequency is in the range and derived from SCLK0\_0 using an integer divisor in the SINC\_CLK.MDIV0 bits. Write to the SINC\_CLK.MREQ0 bit to adjust the phase of the clock. This adjustment lengthens the next clock period by the number of SCLK0\_0 periods stored in the respective SINC\_CLK.MADJ0 bit field. The SINC\_CLK.MREQ0 bit is cleared automatically once the adjustment is complete.

## Filter Configuration

Configure the primary and secondary filter parameters by setting the appropriate bits in the SINC\_RATE0 , SINC\_LEVEL0 , and SINC\_BIAS0 control registers. Configure the DMA transfers by setting the appropriate bits

in the SINC\_PHEAD0 and SINC\_PTAIL0 registers. Set the maximum and minimum levels for overload detection in the four limit registers, SINC\_LIMIT0 -SINC\_LIMIT3 . Set the overload filtering parameters in the SINC\_LEVEL0 register.

## Primary Filter Parameters

Set the primary filter to the 3 rd or 4 th order by the SINC\_LEVEL0.PORD bit assigned to the channel. Set the decimation rate for the primary filter using the SINC\_RATE0.PDEC bits assigned to the channel. Valid decimation rates are in the range 4-256. Set the phase of the primary filter output relative to the number of modulator clocks after enabling the filter using the SINC\_RATE0.PADJ bits assigned to the channel. Valid SINC\_RATE0.PADJ values are in the range 0 to SINC\_RATE0.PDEC - 1.

The raw filter output is a 32-bit wide integer, has an offset added, and is scaled to a 16-bit number before transfer to memory. Store the 32-bit two's compliment offset value in the SINC\_BIAS0 register of the channel. Set the binary scale factor by a mantissa in the range 4-32 stored in the SINC\_LEVEL0.PSCALE bits. The output is a valid 16bit signed number. If the number is outside of the valid range, the output is saturated to 0x8000 or 0x7FFF , while the SINC\_STAT.PSAT0 or SINC\_STAT.GSAT0 fault bit (according to the channel group) is set.

## Primary DMA Configuration and Data Interrupts

Transfer the primary SINC filter outputs to a circular buffer in data memory using DMA. The output from the primary filter is interleaved with outputs from other primary filters. The interleaving order is from the lowest to the highest numbered filter.

The SINC module stores the circular buffer head address in the SINC\_PHEAD0 register of the channel. It stores the tail address in the SINC\_PTAIL0 register of the channel. The data address wraps around to the head address after the tail address is reached. The head and tail addresses must be 16-bit aligned and can be set to the same address. The SINC\_PPTR0 register of the channel is a read-only register that contains the address of the most recent primary SINC filter data. If there is an overflow condition in the SINC filter output data FIFO due to a delay DMA transfer, the SINC\_STAT.FOVF0 fault bit (according to the channel group) is set.

A SINC data trigger can be generated after a user-specified number of primary filter outputs (data transfers) completes. Specify the data count value by the SINC\_LEVEL0.PCNT bits assigned to the channel, and the trigger is generated every SINC\_LEVEL0.PCNT + 1 data transfers.

The SINC Data Buffer Organization figure shows the SINC data buffer organization. In the figure, SINC\_OUT\_0\_M[n] is the data for the n th most recent sample in the m  th channel in the filter group 0, and n = 0 is the most recent data.

Figure 32-5: SINC Data Buffer Organization

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000004_77e2dc70daac07f334b49b35949cfdd1b5ccd00984a01e99d35b44557a93314f.png)

## Secondary Filter Parameters

Set the secondary filter to the 3 rd or 4 th order by the SINC\_LEVEL0.SORD bit assigned to the channel. Set the decimation rate for the secondary filter using the SINC\_RATE0.SDEC bits assigned to the channel. The secondary filter outputs are limited to 16-bit values. Limit the decimation rate according to the filter order:

- Valid decimation rates are in the range 4-40 for the 3 rd order filters
- Valid decimation rates are in the range of 4-16 for the 4 th order filters

Set the phase of the primary filter output relative to the number of modulator clocks after using the SINC\_RATE0.PADJ and SINC\_RATE0.SDEC bits to enable the filter. Valid SINC\_RATE0.PADJ values are in the range 0 to - 1.

## Overload Detection

The function of the secondary SINC filter is to detect AC current overload conditions and set up the upper and lower limit detection thresholds. There are event count filters on the overload detector outputs to reject short-term transients, if desired. Define the overload thresholds in four 32-bit registers SINC\_LIMIT0 , according to the channel number. Each register contains the 16-bit SINC\_LIMIT0.LMAX and SINC\_LIMIT0.LMIN overload threshold values. The SINC filter module detects an overload condition when the secondary filter output exceeds the threshold for a minimum number of counts ( SINC\_LEVEL0.LCNT ) within the detection window ( SINC\_LEVEL0.LWIN ). When the SINC filter module detects an overload condition, the appropriate SINC0\_Px\_OVLD trigger is generated, and the SINC\_STAT.GLIM0 fault bit is set.

The SINC filter module saves the eight most recent data samples for the secondary filter in a local circular buffer to support diagnostics after a fault is triggered. Since 16-bit data is saved, only four buffer registers are required per channel. For example, the SINC\_P1SEC\_HIST[n] [0-3] registers store the eight most recent 16-bit secondary filter outputs from channel 1. The SINC\_HIS\_STAT register contains four pointers ( SINC\_HIS\_STAT.P0HISPTR through SINC\_HIS\_STAT.P3HISPTR ) to the buffer location of the most recent secondary current samples, per channel.

## ADSP-SC58x SINC Register Descriptions

SINC (SINC) contains the following registers.

Table 32-6: ADSP-SC58x SINC Register List

| Name               | Description                                        |
|--------------------|----------------------------------------------------|
| SINC_BIAS0         | Bias for Group 0 Register                          |
| SINC_BIAS1         | Bias for Group 1 Register                          |
| SINC_CLK           | Clock Control Register                             |
| SINC_CTL           | Control Register                                   |
| SINC_HIS_STAT      | History Status Register                            |
| SINC_LEVEL0        | Level Control for Group 0 Register                 |
| SINC_LEVEL1        | Level Control for Group 1 Register                 |
| SINC_LIMIT0        | (Amplitude) Limits for Secondary Filter 0 Register |
| SINC_LIMIT1        | (Amplitude) Limits for Secondary Filter 1 Register |
| SINC_LIMIT2        | (Amplitude) Limits for Secondary Filter 2 Register |
| SINC_LIMIT3        | (Amplitude) Limits for Secondary Filter 3 Register |
| SINC_P0SEC_HIST[n] | Pair 0 Secondary (Filter) History n Register       |
| SINC_P1SEC_HIST[n] | Pair 1 Secondary (Filter) History n Register       |
| SINC_P2SEC_HIST[n] | Pair 2 Secondary (Filter) History n Register       |
| SINC_P3SEC_HIST[n] | Pair 3 Secondary (Filter) History n Register       |
| SINC_PHEAD0        | Primary (Filters) Head for Group 0 Register        |
| SINC_PHEAD1        | Primary (Filters) Head for Group 1 Register        |
| SINC_PPTR0         | Primary (Filters) Pointer for Group 0 Register     |
| SINC_PPTR1         | Primary (Filters) Pointer for Group 1 Register     |
| SINC_PTAIL0        | Primary (Filters) Tail for Group 0 Register        |
| SINC_PTAIL1        | Primary (Filters) Tail for Group 1 Register        |
| SINC_RATE0         | Rate Control for Group 0 Register                  |
| SINC_RATE1         | Rate Control for Group 1 Register                  |
| SINC_STAT          | Status Register                                    |

## Bias for Group 0 Register

The SINC\_BIAS0 register controls an output bias added to primary SINC filters of group 0.

Figure 32-6: SINC\_BIAS0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000005_c3bcbc7236ff47f144be44f153ea44428ac7def68f087b7d938238acf35b0892.png)

Table 32-7: SINC\_BIAS0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | BIAS       | Bias for Group 0 Primary Filters. The SINC_BIAS0.BIAS bits specify a bias for the primary SINC filters output. The bias is added to the output prior to saturation and DMAmemory transfer. The valid value is represented in two's complement format; thus, must be programmed to be equal to -(d ^ o) / 2, where d = SINC_RATE0.PDEC and o = SINC_LEVEL0.PORD . |

## Bias for Group 1 Register

The SINC\_BIAS1 register controls an output bias added to primary SINC filters of group 1.

Figure 32-7: SINC\_BIAS1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000006_e05ac4b1de0df2b9586c1b3c0e0be415d1338a3e333fe62b298dd47442104ea3.png)

Table 32-8: SINC\_BIAS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | BIAS       | Bias for Group1 Primary Filters. The SINC_BIAS1.BIAS bits specify a bias for the primary SINC filters output. The bias is added to the output prior to saturation and DMAmemory transfer. The valid value is represented in two's complement format; thus, must be programmed to be equal to -(d ^ o) / 2, where where d = SINC_RATE1.PDEC and o = SINC_LEVEL1.PORD . |

## Clock Control Register

The SINC\_CLK register generates and enables two SINC modulator clocks. The register also controls each clock's output, frequency, phase, and start-up behavior.

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000007_2a21a40e7166a58ef2fa3ea14b964c8dd97dd9c109d3730d67d978a2a157f2a4.png)

1 Status

Figure 32-8: SINC\_CLK Register Diagram

Table 32-9: SINC\_CLK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:26 (R/W)        | MDIV1      | Modulator (Clock) Divider for Group 1. The SINC_CLK.MDIV1 bits provide the SCLK0_0 divider to generate the modula- tor clock for group 1. The valid value is between 1 and 63.                                                                                                                                                                                                                                                                                                                                      |
| 24 (R/W1S)         | MREQ1      | Modulator (Clock) Request for Group 1 Status. The SINC_CLK.MREQ1 bit indicates status for a phase shift request of the modula- tor clock for group 1. If the bits state is changed from clear (=0) to set (=1), the following modulator clock 1 period is lengthened by the number of SCLK0_0 periods specified by the SINC_CLK.MADJ1 bits. Any writes to this bit while the bit is set are ignored. The bit is cleared by hardware (and only by hardware) once a requested modulator clock adjustment is complete. |
| 24 (R/W1S)         | MREQ1      | 0 Inactive                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 24 (R/W1S)         | MREQ1      | 1 Active                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23:18 (R/W)        | MADJ1      | Modulator (Clock) Adjustment for Group 1. The SINC_CLK.MADJ1 bits provide the adjustment value for the modulator clock of group 1. The valid value is between 1 and 63 when SINC_CLK.MREQ1 is set (=1). A write to this bit field effects only an active modulator clock adjustment. See the SINC_CLK.MREQ1 bit filed description.                                                                                                                                                                                  |

Table 32-9: SINC\_CLK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17:16 (R/W)        | MCEN1      | Modulator (Clock) Enable for Group 1. The SINC_CLK.MCEN1 bits enable/disable the modulator clock for group 1 and control the clocks start-up behavior. Commence the clock immediately upon making it enabled, or enable and commence upon the next rising edge of PWMSYNC (PWM synchronizing output clock).                                                                                                                                                                                                                    |
| 17:16 (R/W)        | MCEN1      | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 17:16 (R/W)        | MCEN1      | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 17:16 (R/W)        | MCEN1      | 2 Enable and Commence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 17:16 (R/W)        | MCEN1      | 3 Enable and Commence on Next Rising Edge                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 15:10 (R/W)        | MDIV0      | Modulator (Clock) Divider for Group 0. The SINC_CLK.MDIV0 bits provide the SCLK0_0 divider to generate the modula- tor clock for group 0. The valid value is between 1 and 63.                                                                                                                                                                                                                                                                                                                                                 |
| 8 (R/W1S)          | MREQ0      | Modulator (Clock) Request for Group 0 Status. The SINC_CLK.MREQ0 bit indicates status for a phase shift request of the modula- tor clock for group 0. If the bits state is changed from clear (=0) to set (=1), the following modulator clock 0 period is lengthened by the number of SCLK0_0 periods specified by the SINC_CLK.MADJ0 bits. Any writes to this bit while the bit is set are ignored. The bit is cleared by hardware (and only by hardware) once a requested modulator clock adjustment is complete. 0 Inactive |
| 7:2 (R/W)          | MADJ0      | 1 Active Modulator (Clock) Adjustment for Group 0. The SINC_CLK.MADJ0 bits provide the adjustment value for the modulator clock of group 0. The valid value is between 1 and 63 when SINC_CLK.MREQ1 is set (=1). A write to this bit field effects only an active modulator clock adjustment. See the SINC_CLK.MREQ1 bit filed description.                                                                                                                                                                                    |
| 1:0 (R/W)          | MCEN0      | Modulator (Clock) Enable for Group 0. The SINC_CLK.MCEN0 bits enable/disable the modulator clock for group 0 and control the clocks start-up behavior. Commence the clock immediately upon making it enabled, or enable and commence upon the next rising edge of PWMSYNC (PWM synchronizing output clock). 0 Disable                                                                                                                                                                                                          |
| 1:0 (R/W)          | MCEN0      | 2 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1:0 (R/W)          | MCEN0      | and Commence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 1:0 (R/W)          | MCEN0      | 3 Enable and Commence on Next Rising Edge                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1:0 (R/W)          | MCEN0      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Control Register

The SINC\_CTL register masks (disables) and unmasks (enables) SINC high-level interrupt request output signals triggered by fault events. The register also enables and assigns SINC filter pairs to one of two control groups.

Figure 32-9: SINC\_CTL Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000008_882eb99621ca5b41e9a76eefeafb9f924771359976bccc4d1aea87cbb09d6311.png)

Table 32-10: SINC\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ELIM1      | Enable Limit for Group 1. The SINC_CTL.ELIM1 bit enables (unmasks) the SINC_STAT interrupt request on overload conditions if this bit and status bit SINC_STAT.GLIM1 are set (=1). Disable                 |
| 31 (R/W)           | ELIM1      | 0                                                                                                                                                                                                          |
| 30 (R/W)           | ESAT1      | Enable Saturation for Group 1. The SINC_CTL.ESAT1 bit enables (unmasks) the SINC_STAT interrupt request on output saturation conditions if this bit and bit SINC_STAT.GSAT1 are set (=1). 0 Disable Enable |
| 30 (R/W)           | ESAT1      | 1                                                                                                                                                                                                          |
| 30 (R/W)           | ESAT1      |                                                                                                                                                                                                            |

Table 32-10: SINC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | EPCNT1     | Enable Primary Count for Group 1. The SINC_CTL.EPCNT1 bit enables a trigger event on each SINC_DATA1 inter- rupt request if this bit and status bit SINC_STAT.PCNT1 are set (=1).                                                                                                                                               |
| 29 (R/W)           | EPCNT1     | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 28 (R/W)           | EFOVF1     | Enable FIFO Overflow for Group 1. The SINC_CTL.EFOVF1 bit enables (unmasks) the SINC_STAT interrupt request on data FIFO overflow conditions if this bit and status bit SINC_STAT.FOVF1 are set (=1). The SINC_STAT.FOVF1 bit is set (=1) when the group 1 output data FIFO overflows due to delayed SCB fabric ready response. |
| 28 (R/W)           | EFOVF1     | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 28 (R/W)           | EFOVF1     | 1 Enable                                                                                                                                                                                                                                                                                                                        |
| 15 (R/W)           | ELIM0      | Enable Limit for Group 0. The SINC_CTL.ELIM0 bit enables (unmasks) the SINC_STAT interrupt request on overload conditions if this bit and status bit SINC_STAT.GLIM0 are set (=1).                                                                                                                                              |
| 15 (R/W)           | ELIM0      | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 15 (R/W)           | ELIM0      | 1 Enable                                                                                                                                                                                                                                                                                                                        |
| 14 (R/W)           | ESAT0      | Enable Saturation for Group 0. The SINC_CTL.ESAT0 bit enables (unmasks) the SINC_STAT interrupt request on output saturation conditions if this bit and status bit SINC_STAT.GSAT0 are set (=1).                                                                                                                                |
| 14 (R/W)           | ESAT0      | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 14 (R/W)           | ESAT0      | 1 Enable                                                                                                                                                                                                                                                                                                                        |
| 13 (R/W)           | EPCNT0     | Enable Primary Count for Group 0. The SINC_CTL.EPCNT0 bit enables a trigger event on each SINC_DATA0 inter- rupt request if this bit and status bit SINC_STAT.PCNT0 are set (=1).                                                                                                                                               |
| 13 (R/W)           | EPCNT0     | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 13 (R/W)           | EPCNT0     | 1 Enable                                                                                                                                                                                                                                                                                                                        |

Table 32-10: SINC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | EFOVF0     | Enable FIFO Overflow for Group 0. The SINC_CTL.EFOVF0 bit enables (unmasks) the SINC_STAT interrupt request on data FIFO overflow conditions if this bit and status bit SINC_STAT.FOVF0 are set (=1). The SINC_STAT.FOVF0 bit is set (=1) when the group 0 output data FIFO overflows due to delayed SCB fabric ready response. |
| 12 (R/W)           | EFOVF0     | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 7:6 (R/W)          | EN3        | Enable Filter Pair 3. The SINC_CTL.EN3 bits enable/disable and assign SINC filter pair 3 to the control group.                                                                                                                                                                                                                  |
| 7:6 (R/W)          | EN3        | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 7:6 (R/W)          | EN3        | 1 Reserved                                                                                                                                                                                                                                                                                                                      |
| 7:6 (R/W)          | EN3        | 2 Enable and Assign to Group 0                                                                                                                                                                                                                                                                                                  |
| 7:6 (R/W)          | EN3        | 3 Reserved                                                                                                                                                                                                                                                                                                                      |
| 5:4 (R/W)          | EN2        | Enable Filter Pair 2. The SINC_CTL.EN2 bits enable/disable and assign SINC filter pair 2 to the control group.                                                                                                                                                                                                                  |
| 5:4 (R/W)          | EN2        | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 5:4 (R/W)          | EN2        | 1 Reserved                                                                                                                                                                                                                                                                                                                      |
| 5:4 (R/W)          | EN2        | 2 Enable and Assign to Group 0                                                                                                                                                                                                                                                                                                  |
| 5:4 (R/W)          | EN2        | 3 Reserved                                                                                                                                                                                                                                                                                                                      |
| 3:2 (R/W)          | EN1        | Enable Filter Pair 1. The SINC_CTL.EN1 bits enable/disable and assign SINC filter pair 1 to the control group.                                                                                                                                                                                                                  |
| 3:2 (R/W)          | EN1        | 0 Disable                                                                                                                                                                                                                                                                                                                       |
| 3:2 (R/W)          | EN1        | 1 Reserved                                                                                                                                                                                                                                                                                                                      |
| 3:2 (R/W)          | EN1        | 2 Enable and Assign to Group 0                                                                                                                                                                                                                                                                                                  |
| 3:2 (R/W)          | EN1        | 3 Reserved                                                                                                                                                                                                                                                                                                                      |

Table 32-10: SINC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------|
| 1:0                | EN0        | Enable Filter Pair 0. The SINC_CTL.EN0 bits enable/disable and assign SINC filter pair 0 to the control | Enable Filter Pair 0. The SINC_CTL.EN0 bits enable/disable and assign SINC filter pair 0 to the control |
| (R/W)              |            | 0                                                                                                       | Disable                                                                                                 |
|                    |            | 1                                                                                                       | Reserved                                                                                                |
|                    |            | 2                                                                                                       | Enable and Assign to Group 0                                                                            |
|                    |            | 3                                                                                                       | Reserved                                                                                                |

## History Status Register

The SINC\_HIS\_STAT provides status for data histories of secondary SINC filters, in the corresponding history buffer registers. The SINC history buffer registers save the most recent filter samples once an overload fault signal is detected.

Figure 32-10: SINC\_HIS\_STAT Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000009_01aaeefff5979073986c86a85bbc990a81fd58f46d8a3ab6070ffab44364c5ac.png)

Table 32-11: SINC\_HIS\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/NW)       | P3HISPTR   | Pair 3 History Pointer. The SINC_HIS_STAT.P3HISPTR bits indicate the position for the most recent data sample of secondary SINC filter 3 in the corresponding SINC_P3SEC_HIST[n] register block. | Pair 3 History Pointer. The SINC_HIS_STAT.P3HISPTR bits indicate the position for the most recent data sample of secondary SINC filter 3 in the corresponding SINC_P3SEC_HIST[n] register block. |
| 14:12 (R/NW)       | P3HISPTR   | 0                                                                                                                                                                                                | History Register 3, MS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 1                                                                                                                                                                                                | History Register 0, LS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 2                                                                                                                                                                                                | History Register 0, MS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 3                                                                                                                                                                                                | History Register 1, LS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 4                                                                                                                                                                                                | History Register 1, MS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 5                                                                                                                                                                                                | History Register 2, LS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 6                                                                                                                                                                                                | History Register 2, MS                                                                                                                                                                           |
| 14:12 (R/NW)       | P3HISPTR   | 7                                                                                                                                                                                                | History Register 3, LS                                                                                                                                                                           |
| 10:8 (R/NW)        | P2HISPTR   | Pair 2 History Pointer. The SINC_HIS_STAT.P2HISPTR bits indicate the position for the most recent data sample of secondary SINC filter 2 in the corresponding SINC_P2SEC_HIST[n] register block. | Pair 2 History Pointer. The SINC_HIS_STAT.P2HISPTR bits indicate the position for the most recent data sample of secondary SINC filter 2 in the corresponding SINC_P2SEC_HIST[n] register block. |
| 10:8 (R/NW)        | P2HISPTR   | 0                                                                                                                                                                                                | History Register 3, MS                                                                                                                                                                           |
| 10:8 (R/NW)        | P2HISPTR   | 1                                                                                                                                                                                                | History Register 0, LS                                                                                                                                                                           |
| 10:8 (R/NW)        | P2HISPTR   | 2                                                                                                                                                                                                | History Register 0, MS                                                                                                                                                                           |

Table 32-11: SINC\_HIS\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 3 History Register 1, LS                                                                                                                                                                         |
|                    |            | 4 History Register 1, MS                                                                                                                                                                         |
|                    |            | 5 History Register 2, LS                                                                                                                                                                         |
|                    |            | 6 History Register 2, MS                                                                                                                                                                         |
|                    |            | 7 History Register 3, LS                                                                                                                                                                         |
| 6:4 (R/NW)         | P1HISPTR   | Pair 1 History Pointer. The SINC_HIS_STAT.P1HISPTR bits indicate the position for the most recent data sample of secondary SINC filter 1 in the corresponding SINC_P1SEC_HIST[n] register block. |
|                    |            | 0 History Register 3, MS                                                                                                                                                                         |
|                    |            | 1 History Register, LS                                                                                                                                                                           |
|                    |            | 2 History Register 0, MS                                                                                                                                                                         |
|                    |            | 3 History Register 1, LS                                                                                                                                                                         |
|                    |            | 4 History Register 1, MS                                                                                                                                                                         |
|                    |            | 5 History Register 2, LS                                                                                                                                                                         |
|                    |            | 6 History Register 2, MS                                                                                                                                                                         |
|                    |            | 7 History Register 3, LS                                                                                                                                                                         |
| 2:0 (R/NW)         | P0HISPTR   | Pair 0 History Pointer. The SINC_HIS_STAT.P0HISPTR bits indicate the position for the most recent data sample of secondary SINC filter 0 in the corresponding SINC_P0SEC_HIST[n] register block. |
|                    |            | 0 History Register 3, MS                                                                                                                                                                         |
|                    |            | 1 History Register 0, LS                                                                                                                                                                         |
|                    |            | 2 History Register 0, MS                                                                                                                                                                         |
|                    |            | 3 History Register 1, LS                                                                                                                                                                         |
|                    |            | 4 History Register 1, MS                                                                                                                                                                         |
|                    |            | 5 History Register 2, LS                                                                                                                                                                         |
|                    |            | 6 History Register 2, MS                                                                                                                                                                         |
|                    |            | 7 History Register 3, LS                                                                                                                                                                         |

## Level Control for Group 0 Register

The SINC\_LEVEL0 register controls output scaling and count, excursion limit and window, as well as orders for primary and secondary SINC filters assigned to group 0.

Figure 32-11: SINC\_LEVEL0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000010_ad850622ebbcaafd7338bacc14eb2cde5172c38fd0b7a222d673b692fcf98c64.png)

Table 32-12: SINC\_LEVEL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------|
| 30 (R/W)           | PORD       | Primary (Filter) Order. The SINC_LEVEL0.PORD bit determines the order for group 1 primary filters. |

Table 32-12: SINC\_LEVEL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | PSCALE     | Primary (Filter) Scaling. The SINC_LEVEL0.PSCALE bits specify the scaling applied to the output of group 0 primary filters, prior to DMAtransfer to memory. The valid value is between 4 to 32. The SINC integrator, decimator, and bias adjustment produce an integer value up to 32 bits wide. The range of a full-scale signal of a bit stream filtered by a primary SINC filter is approximately (BIAS +- ((0.625 * SINC_RATE0.PDEC ) ^ order)). The value requires about ( ln2( SINC_RATE0.PDEC ) * order ) bits of precision (where 'order' is 3 or 4, as specified by the SINC_LEVEL0.PORD bit. This bit field specifies the bit position of the intermediate value, which is transferred on the MSB of 16-bit DMAsample. Thus, the intermediate value is right-shifted by ( SINC_LEVEL0.PSCALE - 16) bits if SINC_LEVEL0.PSCALE >= 16, or left- shifted by (16 - SINC_LEVEL0.PSCALE ) bits if SINC_LEVEL0.PSCALE < 16. If SINC_LEVEL0.PSCALE >= 16, thus selecting a right shift, the shifted value is rounded up (as if 0.5 * LSB is added) before truncation. Rounding is not necessary for a left shift. If the scaled and rounded value exceeds the range of a signed 16-bit num- ber, the sample is saturated (to 0x8000 or 0x7FFF), and the corresponding saturation status bit ( SINC_STAT.PSAT3 , SINC_STAT.PSAT2 , SINC_STAT.PSAT1 , or SINC_STAT.PSAT0 is set. |
| 23:16 (R/W)        | PCNT       | Primary (Filter) Count. The SINC_LEVEL0.PCNT bits specify the modulo number of outputs for group 0 primary filters. The number must be one less than a desired modulo. Each time the number of outputs specified by this bit filed is transferred, the SINC_STAT.PCNT0 status bit is set (=1). When the SINC_STAT.PCNT0 bit is set (unless masked), it causes a TRU trigger. For example: 8'h00 written to the SINC_LEVEL0.PCNT bit field sets bit SINC_STAT.PCNT0 to 1 after every primary SINC filter output is transferred. 8'hFF written to the SINC_LEVEL0.PCNT bit field sets bit SINC_STAT.PCNT0 to 1 after every 256 primary SINC filter outputs transferred.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14 (R/W)           | SORD       | Secondary (Filter) Order Select. The SINC_LEVEL0.SORD bit determines the order for group 0 secondary filters. 0 Third Order                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 14 (R/W)           | SORD       | 1 Fourth Order                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 32-12: SINC\_LEVEL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:11 (R/W)        | LCNT       | (Excursion) Limit Count. The SINC_LEVEL0.LCNT bits specify the number (count) of output excursions be- yond the amplitude specified for group 0 secondary filters. The number of excursions greater than specified by registers SINC_LIMIT3 , SINC_LIMIT2 , SINC_LIMIT2 , and SINC_LIMIT0 is perceived as an overload and sets (=1) a cor- responding MAX or MIN bit in the SINC_STAT register. The valid count is between 1 to 8. If the count is greater than SINC_LEVEL0.LWIN , the bit fields behavior is as it is equal to SINC_LEVEL0.LWIN . See SINC_LEVEL0.LWIN for details. The valid count must be one less than a desired count: =000 require one excursion above the amplitude limit =111 require eight excursions above the amplitude limit. |
| 10:8 (R/W)         | LWIN       | (Excursion) Limit Window. The SINC_LEVEL0.LWIN bits specify the window size for excursion checking for group 0 secondary filters. The window size is the number of the most recent outputs to be included in a measurement specified by the SINC_LEVEL0.LCNT bits. The valid value must be one less than a desired count (1 to 8), meaning the valid value is 0 to 7.                                                                                                                                                                                                                                                                                                                                                                                     |

## Level Control for Group 1 Register

The SINC\_LEVEL1 register controls output scaling and count, excursion limit and window, as well as orders for primary and secondary SINC filters assigned to group 1.

Figure 32-12: SINC\_LEVEL1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000011_de627aa019000625a3d3d96283bf3a6cf804f068cf9dc2fbc9962a745dd097cc.png)

Table 32-13: SINC\_LEVEL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 30 (R/W)           | PORD       | Primary (Filter) Order. The SINC_LEVEL1.PORD bits determines the order for group 1 primary filters. |

Table 32-13: SINC\_LEVEL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | PSCALE     | Primary (Filter) Scaling. The SINC_LEVEL1.PSCALE bits specify the scaling applied to the output of group 1 primary filters, prior to DMAtransfer to memory. The valid value is between 4 to 32. The SINC integrator, decimator, and bias adjustment produce an integer value up to 32 bits wide. The range of a full-scale signal of a bit stream filtered by a primary SINC filter is approximately (BIAS +- ((0.625 * SINC_RATE1.PDEC ) ^ order)). The value requires about ( ln2( SINC_RATE1.PDEC ) * order ) bits of precision (where 'order' is 3 or 4, as specified by the SINC_LEVEL1.PORD bit. This bit field specifies the bit position of the intermediate value, which is transferred on the MSB of 16-bit DMAsample. Thus, the intermediate value is right-shifted by ( SINC_LEVEL1.PSCALE - 16) bits if SINC_LEVEL1.PSCALE >= 16, or left- shifted by (16 - SINC_LEVEL1.PSCALE ) bits if SINC_LEVEL1.PSCALE < 16. If SINC_LEVEL1.PSCALE >= 16, thus selecting a right shift, the shifted value is rounded up (as if 0.5 * LSB is added) before truncation. Rounding is not necessary for a left shift. If the scaled and rounded value exceeds the range of a signed 16-bit num- ber, the sample is saturated (to 0x8000 or 0x7FFF), and the corresponding saturation status bit ( SINC_STAT.PSAT3 , SINC_STAT.PSAT2 , SINC_STAT.PSAT1 , or SINC_STAT.PSAT0 is set. |
| 23:16 (R/W)        | PCNT       | Primary (Filter) Count. The SINC_LEVEL1.PCNT bits specify the modulo number of outputs for group 1 primary filters. The number must be one less than a desired modulo. Each time the number of outputs specified by this bit filed is transferred, the SINC_STAT.PCNT1 status bit is set (=1). When the SINC_STAT.PCNT1 bit is set (unless masked), it causes a TRU trigger. For example: 8'h00 written to the SINC_LEVEL1.PCNT bit field sets bit SINC_STAT.PCNT1 to 1 after every primary SINC filter output is transferred. 8'hFF written to the SINC_LEVEL1.PCNT bit field sets bit SINC_STAT.PCNT1 to 1 after every 256 primary SINC filter outputs transferred.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14 (R/W)           | SORD       | Secondary (Filter) Order. The SINC_LEVEL1.SORD bit determines the order for group 1 secondary filters. The SINC_LEVEL1.SORD bit determines the order for group 1 secondary filters. 0 Third Order                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 14 (R/W)           | SORD       | 1 Fourth Order                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 32-13: SINC\_LEVEL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:11 (R/W)        | LCNT       | (Excursion) Limit Count. The SINC_LEVEL1.LCNT bits specify the number (count) of output excursions be- yond the amplitude specified for group 1 secondary filters. The number of excursions greater than specified by registers SINC_LIMIT3 , SINC_LIMIT2 , SINC_LIMIT2 , and SINC_LIMIT0 is perceived as an overload and sets (=1) a cor- responding MAX or MIN bit in the SINC_STAT register. The valid count is between 1 to 8. If the count is greater than SINC_LEVEL1.LWIN , the bit fields behavior is as it is equal to SINC_LEVEL1.LWIN . See SINC_LEVEL1.LWIN for details. The valid count must be one less than a desired count: =000 require one excursion above the amplitude limit =111 require eight excursions above the amplitude limit. |
| 10:8 (R/W)         | LWIN       | (Excursion) Limit Window. The SINC_LEVEL1.LWIN bits specify the window size for excursion checking for group 1 secondary filters. The window size is the number of the most recent outputs to be included in a measurement specified by the SINC_LEVEL1.LCNT bits. The valid value must be one less than a desired count (1 to 8), meaning the valid value is 0 to 7.                                                                                                                                                                                                                                                                                                                                                                                     |

## (Amplitude) Limits for Secondary Filter 0 Register

The SINC\_LIMIT0 register controls amplitude limits for a secondary filter of SINC pair 0.

Figure 32-13: SINC\_LIMIT0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000012_28af4deef6776d92355d3c4d7ea51e946d4a7b4abcc3507d9357b0db6f51a730.png)

Table 32-14: SINC\_LIMIT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | LMAX       | Limit Maximum for Secondary Filter 0. The SINC_LIMIT0.LMAX bits specify the output signal conditions for the secon- dary SINC filter 0. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed maximum limit warning bit in register SINC_STAT . |
| 15:0 (R/W)         | LMIN       | Limit Minimum for Secondary Filter 0. The SINC_LIMIT0.LMIN bits specify the output signal conditions for the secon- dary SINC filter 0. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed minimum limit warning bit in register SINC_STAT . |

## (Amplitude) Limits for Secondary Filter 1 Register

The SINC\_LIMIT1 register controls amplitude limits for a secondary filter of SINC pair 1.

Figure 32-14: SINC\_LIMIT1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000013_3f215aa288398375f717c05a48a28ea4b332e60f729a22631f211da2eec576ea.png)

Table 32-15: SINC\_LIMIT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | LMAX       | Limit Maximum for Secondary Filter 1. The SINC_LIMIT1.LMAX bits specify the output signal conditions for the secon- dary SINC filter 1. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed maximum limit warning bit in register SINC_STAT . |
| 15:0 (R/W)         | LMIN       | Limit Minimum for Secondary Filter 1. The SINC_LIMIT1.LMIN bits specify the output signal conditions for the secon- dary SINC filter 1. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed minimum limit warning bit in register SINC_STAT . |

## (Amplitude) Limits for Secondary Filter 2 Register

The SINC\_LIMIT2 register controls amplitude limits for a secondary filter of SINC pair 2.

Figure 32-15: SINC\_LIMIT2 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000014_082a0b6b8c4671db7e3c3c7463749185fec2b2999f9c8c60bb91f9a4d9055a49.png)

Table 32-16: SINC\_LIMIT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | LMAX       | Limit Maximum for Secondary Filter 2. The SINC_LIMIT2.LMAX bits specify the output signal conditions for the secon- dary SINC filter 2. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed maximum limit warning bit in register SINC_STAT . |
| 15:0 (R/W)         | LMIN       | Limit Minimum for Secondary Filter 2. The SINC_LIMIT2.LMIN bits specify the output signal conditions for the secon- dary SINC filter 2. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed minimum limit warning bit in register SINC_STAT . |

## (Amplitude) Limits for Secondary Filter 3 Register

The SINC\_LIMIT3 register controls amplitude limits for a secondary filter of SINC pair 3.

Figure 32-16: SINC\_LIMIT3 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000015_a27ec11ec9997a3ce3163cc3801e63cb0507739b852bbda3e6698595c7ba5a8c.png)

Table 32-17: SINC\_LIMIT3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | LMAX       | Limit Maximum for Secondary Filter 3. The SINC_LIMIT3.LMAX bits specify the output signal conditions for the secon- dary SINC filter 3. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed maximum limit warning bit in register SINC_STAT . |
| 15:0 (R/W)         | LMIN       | Limit Minimum for Secondary Filter 3. The SINC_LIMIT3.LMIN bits specify the output signal conditions for the secon- dary SINC filter 3. In conjunction with bits LCNT and LWIN in register SINC_LEVEL1 or SINC_LEVEL0 , this bit field specifies conditions for an associat- ed minimum limit warning bit in register SINC_STAT . |

## Pair 0 Secondary (Filter) History n Register

The SINC\_P0SEC\_HIST[n] read-only register provides the eight most recent samples produced by secondary SINC filter 0. The 16-bit samples are stored in the 32-bit register in circular manner, starting with the low-order field of the first SINC\_P0SEC\_HIST[n] register. The stored values, one compared to the limit, count, and window settings, set the SINC\_STAT.MAX0 and SINC\_STAT.MIN0 bits.

Figure 32-17: SINC\_P0SEC\_HIST[n] Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000016_7c5e9a81f3c234d84e2318c21f19bd6770fe9aae187e96f62a02515fdbd34131.png)

Table 32-18: SINC\_P0SEC\_HIST[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | HI         | High Data Word. The SINC_P0SEC_HIST[n].HI bits provide the 16-bit sample in the most signif- icant half of the 32- bit register. |
| 15:0 (R/NW)        | LO         | Low Data Word. The SINC_P0SEC_HIST[n].LO bits provide the 16-bit sample in the least signifi- cant half of the 32- bit register. |

## Pair 1 Secondary (Filter) History n Register

The SINC\_P1SEC\_HIST[n] read-only register provides the eight most recent samples produced by secondary SINC filter 1. The 16-bit samples are stored in the 32-bit register in circular manner, starting with the low-order field of the first SINC\_P1SEC\_HIST[n] register. The stored values, compared to the limit, count, and window settings, set the SINC\_STAT.MAX1 and SINC\_STAT.MIN1 bits.

Figure 32-18: SINC\_P1SEC\_HIST[n] Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000017_7c5e9a81f3c234d84e2318c21f19bd6770fe9aae187e96f62a02515fdbd34131.png)

Table 32-19: SINC\_P1SEC\_HIST[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | HI         | High Data Word. The SINC_P1SEC_HIST[n].HI bits provide the 16-bit sample in the most signif- icant half of the 32- bit register. |
| 15:0 (R/NW)        | LO         | Low Data Word. The SINC_P1SEC_HIST[n].LO bits provide the 16-bit sample in the least signifi- cant half of the 32- bit register. |

## Pair 2 Secondary (Filter) History n Register

The SINC\_P2SEC\_HIST[n] read-only register provides the eight most recent samples produced by secondary SINC filter 2. The 16-bit samples are stored in the 32-bit register in circular manner, starting with the low-order field of the first SINC\_P2SEC\_HIST[n] register. The stored values, compared to the limit, count, and window settings, set the SINC\_STAT.MAX2 and SINC\_STAT.MIN2 bits.

Figure 32-19: SINC\_P2SEC\_HIST[n] Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000018_7c5e9a81f3c234d84e2318c21f19bd6770fe9aae187e96f62a02515fdbd34131.png)

Table 32-20: SINC\_P2SEC\_HIST[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | HI         | High Data Word. The SINC_P2SEC_HIST[n].HI bits provide the 16-bit sample in the most signif- icant half of the 32- bit register. |
| 15:0 (R/NW)        | LO         | Low Data Word. The SINC_P2SEC_HIST[n].LO bits provide the 16-bit sample in the least signifi- cant half of the 32- bit register. |

## Pair 3 Secondary (Filter) History n Register

The SINC\_P3SEC\_HIST[n] read-only register provides the eight most recent samples produced by secondary SINC filter 3. The 16-bit samples are stored in the 32-bit register in circular manner, starting with the low-order field of the first SINC\_P3SEC\_HIST[n] register. The stored values, compared to the limit, count, and window settings, set the SINC\_STAT.MAX3 and SINC\_STAT.MIN3 bits.

Figure 32-20: SINC\_P3SEC\_HIST[n] Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000019_7c5e9a81f3c234d84e2318c21f19bd6770fe9aae187e96f62a02515fdbd34131.png)

Table 32-21: SINC\_P3SEC\_HIST[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | HI         | High Data Word. The SINC_P3SEC_HIST[n].HI bits provide the 16-bit sample in the most signif- icant half of the 32- bit register. |
| 15:0 (R/NW)        | LO         | Low Data Word. The SINC_P3SEC_HIST[n].LO bits provide the 16-bit sample in the least signifi- cant half of the 32- bit register. |

## Primary (Filters) Head for Group 0 Register

The SINC\_PHEAD0 register stores the head address for a circular buffer in data memory to which to transfer the primary SINC filter outputs (according to control group 0 assignments).

Figure 32-21: SINC\_PHEAD0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000020_e5b16b51cabe4c26d6dc5e1102a386225b0f2f52fee25acbf4a6c11f1211113a.png)

Table 32-22: SINC\_PHEAD0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:1 (R/W)         | PHEAD      | Primary (Filter) Head Pointer. The SINC_PHEAD0.PHEAD bits hold the pointer (address) for DMAtransfer to memory. Commencing at and wrapping back to SINC_PHEAD0.PHEAD after SINC_PTAIL0.PTAIL is reached, it forms a circular buffer, to which to transfer the primary SINC filter outputs (group 0). The valid address must be 16-bit aligned (address must be even). |

## Primary (Filters) Head for Group 1 Register

The SINC\_PHEAD1 register stores the head address for a circular buffer in data memory to which to transfer the primary SINC filter outputs (according to control group 1 assignments).

Figure 32-22: SINC\_PHEAD1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000021_e5b16b51cabe4c26d6dc5e1102a386225b0f2f52fee25acbf4a6c11f1211113a.png)

Table 32-23: SINC\_PHEAD1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:1 (R/W)         | PHEAD      | Primary (Filter) Head Pointer. The SINC_PHEAD1.PHEAD bits hold the pointer (address) for DMAtransfer to memory. Commencing at and wrapping back to SINC_PHEAD1.PHEAD after SINC_PTAIL1.PTAIL is reached, it forms a circular buffer, to which to transfer the primary SINC filter outputs (group 1). The valid address must be 16-bit aligned (address must be even). |

## Primary (Filters) Pointer for Group 0 Register

The SINC\_PPTR0 read-only register points to a circular buffer holding the most recent results of primary SINC filters, according to control group 0 assignments.

Figure 32-23: SINC\_PPTR0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000022_b735a08dc7926d10c453d171324a3325111ae0377e3e681eddee7ca3995b8d5f.png)

Table 32-24: SINC\_PPTR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | PPTR       | Primary (Filter) Pointer. The SINC_PPTR0.PPTR bits hold the address for the last memory location of the most recent set of primary SINC filter results (group 0). The address is incremented once all of the primary SINC filter data (assigned to group 0 and associated to a particular time stamp) is successfully presented to the system fab- ric. Memory locations beyond the location reported by this register may be partially updat- ed, so the entire circular buffer is not considered valid. Note that in real-time opera- tion, due to fabric latency, write data may be in flight on the system fabric after the point when this bit field is updated. Thus, the write data may not be observed in memory until it has transited the fabric. |

## Primary (Filters) Pointer for Group 1 Register

The SINC\_PPTR1 read-only register points to a circular buffer holding the most recent results of primary SINC filters, according to control group 1 assignments.

Figure 32-24: SINC\_PPTR1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000023_b735a08dc7926d10c453d171324a3325111ae0377e3e681eddee7ca3995b8d5f.png)

Table 32-25: SINC\_PPTR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | PPTR       | Primary (Filter) Pointer. The SINC_PPTR1.PPTR bits hold the address for the last memory location of the most recent set of primary SINC filter results (group 1). The address is incremented once all of the primary SINC filter data (assigned to group 1 and associated to a particular time stamp) is successfully presented to the system fab- ric. Memory locations beyond the location reported by this register may be partially updat- ed, so the entire circular buffer is not considered valid. Note that in real-time opera- tion, due to fabric latency, write data may be in flight on the system fabric after the point when this bit field is updated. Thus, the write data may not be observed in memory until it has transited the fabric. |

## Primary (Filters) Tail for Group 0 Register

The SINC\_PTAIL0 register stores the tail address for a circular buffer in data memory to which to transfer the primary SINC filter outputs (according to control group 1 assignments).

Figure 32-25: SINC\_PTAIL0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000024_610d424fe608aa5bc0bec131b5c9f979fbee55a973dff167a62a9becc49f4969.png)

Table 32-26: SINC\_PTAIL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:1 (R/W)         | PTAIL      | Primary (Filter) Tail Pointer. The SINC_PTAIL0.PTAIL bits hold the pointer (address) for DMAtransfer to memory. Commencing at and wrapping back to SINC_PHEAD0.PHEAD after SINC_PTAIL0.PTAIL is reached, it forms a circular buffer, to which to transfer the primary SINC filter outputs (group 1). The valid address must be 16-bit aligned (address must be even). |

## Primary (Filters) Tail for Group 1 Register

The SINC\_PTAIL1 register stores the tail address for a circular buffer in data memory to which to transfer the primary SINC filter outputs (according to control group 1 assignments).

Figure 32-26: SINC\_PTAIL1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000025_610d424fe608aa5bc0bec131b5c9f979fbee55a973dff167a62a9becc49f4969.png)

Table 32-27: SINC\_PTAIL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:1 (R/W)         | PTAIL      | Primary (Filter) Tail Pointer. The SINC_PTAIL1.PTAIL bits hold the pointer (address) for DMAtransfer to memory. Commencing at and wrapping back to SINC_PHEAD1.PHEAD after SINC_PTAIL1.PTAIL is reached, it forms a circular buffer, to which to transfer the primary SINC filter outputs (group 1). The valid address must be 16-bit aligned (address must be even). |

## Rate Control for Group 0 Register

The SINC\_RATE0 register controls phase adjustments and decimation rates for primary and secondary SINC filters assigned to group 0.

Figure 32-27: SINC\_RATE0 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000026_cc3150cd24bb4120d39902a76528e8125636dcd474ee71053a5611fb1495abc0.png)

Table 32-28: SINC\_RATE0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:25 (R/W)        | SADJ       | Secondary (Filter) Adjustment. The SINC_RATE0.SADJ bits provide the phase adjustment for the decimated out- put of group 0 secondary filters. The valid adjustment is between 0 and ( SINC_RATE0.SDEC - 1), in modulator clock cycles, relative to the time the filter is enabled in the SINC_CTL register. The secondary SINC filter calculates an output in modulator clock cycle equivalent to ( ( SINC_RATE0.SDEC * n) - SINC_RATE0.SADJ ), where n is an integer > 1. This bit field can be changed while the filter is running and takes effect after the next decimation sample is generated. The effect of the change requires time to ripple through the filter: a number of output sample periods is equal to the filter order. |
| 24:16 (R/W)        | PADJ       | Primary (Filter) Adjustment. The SINC_RATE0.PADJ bits provide the phase adjustment for the decimated out- put of group 0 primary filters. The valid adjustment is between 0 and ( SINC_RATE0.PDEC - 1), in modulator clock cycles, relative to the time the filter is enabled in the SINC_CTL register. The primary SINC filter calculates an output in modulator clock cycle equivalent to ( ( SINC_RATE0.PDEC * n) - SINC_RATE0.PADJ ), where n is an integer > 1. This bit field can be changed while the filter is running and takes effect after the next decimation sample is generated. The effect of the change requires time to ripple through the filter: a number of output sample periods is equal to the filter order.       |

Table 32-28: SINC\_RATE0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:9 (R/W)         | SDEC       | Secondary (Filter) Decimation Rate. The SINC_RATE0.SDEC bits provide the decimation rate for group 0 secondary fil- ters. The valid range depends on the SINC order selected. If the third order ( SINC_LEVEL0.SORD = 0), the valid range is 4 to 40. If the forth order ( SINC_LEVEL0.SORD = 1), the valid rate is 4 to 16. |
| 8:0 (R/W)          | PDEC       | Primary (Filter) Decimation Rate. The SINC_RATE0.PDEC bits provide the decimation rate for group 0 primary fil- ters. The valid rate is 256 to 4.                                                                                                                                                                            |

## Rate Control for Group 1 Register

The SINC\_RATE1 register controls phase adjustments and decimation rates for primary and secondary SINC filters assigned to group 1.

Figure 32-28: SINC\_RATE1 Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000027_cc3150cd24bb4120d39902a76528e8125636dcd474ee71053a5611fb1495abc0.png)

Table 32-29: SINC\_RATE1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:25 (R/W)        | SADJ       | Secondary (Filter) Adjustment. The SINC_RATE1.SADJ bits provide the phase adjustment for the decimated out- put of group 1 secondary filters. The valid adjustment is between 0 and ( SINC_RATE1.SDEC - 1), in modulator clock cycles, relative to the time the filter is enabled in the SINC_CTL register. The secondary SINC filter calculates an output in modulator clock cycle equivalent to ( ( SINC_RATE1.SDEC * n) - SINC_RATE1.SADJ ), where n is an integer > 1. This bit field can be changed while the filter is running and takes effect after the next decimation sample is generated. The effect of the change requires time to ripple through the filter: a number of output sample periods is equal to the filter order. |
| 24:16 (R/W)        | PADJ       | Primary (Filter) Adjustment. The SINC_RATE1.PADJ bits provide the phase adjustment for the decimated out- put of group 1 primary filters. The valid adjustment is between 0 and ( SINC_RATE1.PDEC - 1), in modulator clock cycles, relative to the time the filter is enabled in the SINC_CTL register. The primary SINC filter calculates an output in modulator clock cycle equivalent to ( ( SINC_RATE1.PDEC * n) - SINC_RATE1.PADJ ), where n is an integer > 1. This bit field can be changed while the filter is running and takes effect after the next decimation sample is generated. The effect of the change requires time to ripple through the filter: a number of output sample periods is equal to the filter order.       |

Table 32-29: SINC\_RATE1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:9 (R/W)         | SDEC       | Secondary (Filter) Decimation Rate. The SINC_RATE1.SDEC bits provide the decimation rate for group 1 secondary fil- ters. The valid range depends on the SINC order selected. If the third order ( SINC_LEVEL1.SORD = 0), the valid range is 4 to 40. If the forth order ( SINC_LEVEL1.SORD = 1), the valid rate is 4 to 16. |
| 8:0 (R/W)          | PDEC       | Primary (Filter) Decimation Rate. The SINC_RATE1.PDEC bits provide the decimation rate for group 1 primary fil- ters. The valid rate is 256 to 4.                                                                                                                                                                            |

## Status Register

The SINC\_STAT register indicates status for SINC output saturation, amplitude and duration limits, overload conditions, and data transfer errors.

Figure 32-29: SINC\_STAT Register Diagram

![Image](35_Sinus_Cardinalis_(SINC)_Filter_artifacts/image_000028_6f888c480666b6d0a0d100d025876bf33bd1a3efcbb41f2db58c3f46822cf675.png)

Table 32-30: SINC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | GLIM1      | Group 1 Limit Status. The SINC_STAT.GLIM1 indicates status for an amplitude and duration limit of secondary SINC filters assigned to group 1. This bit is set (=1) if any limit specified by registers SINC_LIMIT3 , SINC_LIMIT2 , SINC_LIMIT1 , or SINC_LIMIT0 , within the duration count and window specified by bits SINC_LEVEL0.LCNT and SINC_LEVEL0.LWIN are exceeded. To identify the offending secondary SINC filter, examine the filters status bits SINC_STAT.MAX3 , SINC_STAT.MAX2 , SINC_STAT.MAX1 , SINC_STAT.MAX0 , SINC_STAT.MIN3 , SINC_STAT.MIN2 , SINC_STAT.MIN1 and SINC_STAT.MAX0 according to the group 1 assignments in the SINC_CTL register. 0 Not Exceeded 1 Exceeded                                                                                                                                                                                                                                                                                                    |
| 30 (R/NW)          | GSAT1      | Group 1 Saturation Status. The SINC_STAT.GSAT1 indicates status for the output saturation bit of primary SINC filters assigned to group 1. The bit is set (=1) if any filter of group 1 has its satu- ration status bit set (=1). To identify the offending SINC primary filter, examine bits SINC_STAT.PSAT3 , SINC_STAT.PSAT2 , SINC_STAT.PSAT1 , and SINC_STAT.PSAT0 accord- ing to the group 1 assignments specified by the SINC_CTL.EN3 , SINC_CTL.EN2 , SINC_CTL.EN1 , and SINC_CTL.EN0 bits. 0 Not Set                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 29 (R/W1C)         | PCNT1      | 1 Set Primary (Filter) Count for Group 1 Status. The SINC_STAT.PCNT1 indicates status for the output count of primary SINC fil- ters assigned to group 1. The bit is set (=1) each time the modulo number of outputs (specified by the SINC_LEVEL1.PCNT bits) has been transferred for each primary SINC filter assigned to group 1. Each count in SINC_LEVEL1.PCNT corresponds to one complete set or vector of samples from all SINC filter pairs assigned to group 1. For example, if group 1 is assigned three SINC filters pairs 0, 1, and 3, and SINC_LEVEL1.PCNT is set to 5, then this status bit is set after the transfer of every 5th complete sample vector, comprising 3 x 5= 15 data samples. This bit asserts when the memory transfer on the system SCB fabric is complete, and a valid SCB write data response is received by the SINC filter unit. If this status bit and bit SINC_CTL.EPCNT1 are set (=1), the SINC_DATA1 trigger is asserted. Write 1 to clear. 0 Not Reached |
| 29 (R/W1C)         |            | 1 Reached                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 29 (R/W1C)         |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W1C)         | FOVF1      | FIFO Overflow for Group 1 Status. The SINC_STAT.FOVF1 indicates status for the data output FIFO bit of primary SINC filters assigned to group 1. This bit is set (= 1) if the output FIFO for any filter in group 1 overflows due to slow SCB fabric response. The FIFO for each primary SINC filter contains two data sample locations. An overflow occurs if a third data sam- ple is generated before the first sample's data is transferred into the SCB fabric write data channel. After any overflow signaled by this bit occurs, all further SCB transmissions generated by group 1 are UNSPECIFIED until all SINC filters of the group are shut down and restarted. Clearing this status bit (=0) alone is not sufficient to re-sync theDMA stream. Write 1 to clear. If this status bit and bit SINC_CTL.EFOVF1 are set (=1), the SINC_STAT inter- rupt is asserted. 0 No Overflow |
| 27 (R/W1C)         | PFAB1      | Primary (Filter) Fabric Error for Group 1 Status. The SINC_STAT.PFAB1 indicates error status for the output of any primary SINC filter assigned to group 1. The bit is set (=1) if the SCB fabric provides a write error response for a filter output transfer associated with group 1, or if an overrun occurs for a filter in group 1. An interrupt is requested whenever this bit =1 (not maskable).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 27 (R/W1C)         | PFAB1      | 0 Disabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 23 (R/W1C)         | PSAT3      | Primary (Filter) 3 Saturation Status. The SINC_STAT.PSAT3 bit indicates whether the primary SINC filter 3 requires saturation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 23 (R/W1C)         | PSAT3      | 0 Not Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 23 (R/W1C)         | PSAT3      | 1 Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 22 (R/W1C)         | PSAT2      | Primary (Filter) 2 Saturation Status. The SINC_STAT.PSAT2 bit indicates whether the primary SINC filter 2 requires saturation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 22 (R/W1C)         | PSAT2      | 0 Not Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 22 (R/W1C)         | PSAT2      | 1 Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W1C)         | PSAT1      | Primary (Filter) 1 Saturation Status. The SINC_STAT.PSAT1 bit indicates whether the primary SINC filter 1 requires saturation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 21 (R/W1C)         | PSAT1      | 0 Not Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 21 (R/W1C)         | PSAT1      | 1 Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 20 (R/W1C)         | PSAT0      | Primary (Filter) 0 Saturation Status. The SINC_STAT.PSAT0 bit indicates whether the primary SINC filter 0 requires saturation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 20 (R/W1C)         | PSAT0      | 0 Not Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 20 (R/W1C)         | PSAT0      | 1 Saturated                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 19 (R/W1C)         | MAX3       | Maximum for Secondary Filter 3 Status. The SINC_STAT.MAX3 bit indicates whether the output of the secondary SINC filter 3 exceeded its maximum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT3.LMAX bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN3 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window SINC_LEVEL1.LWIN samples. 0 Not Exceeded 1 Exceeded |
| 19 (R/W1C)         | MAX3       | of of                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 18 (R/W1C)         | MAX2       | Maximum for Secondary Filter 2 Status. The SINC_STAT.MAX2 bit indicates whether the output of the secondary SINC filter 2 exceeded its maximum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT2.LMAX bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN2 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples. Exceeded            |
| 18 (R/W1C)         | MAX2       | 0 Not                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 18 (R/W1C)         | MAX2       | 1 Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | MAX1       | Maximum for Secondary Filter 1 Status. The SINC_STAT.MAX1 bit indicates whether the output of the secondary SINC filter 0 exceeded its maximum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT1.LMAX bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN1 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples. 0 Not Exceeded | Maximum for Secondary Filter 1 Status. The SINC_STAT.MAX1 bit indicates whether the output of the secondary SINC filter 0 exceeded its maximum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT1.LMAX bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN1 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples. 0 Not Exceeded |
| 16 (R/W1C)         | MAX0       | Maximum for Secondary Filter 0 Status. The SINC_STAT.MAX0 bit indicates whether the output of the secondary SINC filter 0 exceeded its maximum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT0.LMAX bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN0 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples.                | Maximum for Secondary Filter 0 Status. The SINC_STAT.MAX0 bit indicates whether the output of the secondary SINC filter 0 exceeded its maximum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT0.LMAX bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN0 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples.                |
| 16 (R/W1C)         | MAX0       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Not Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 16 (R/W1C)         | MAX0       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | GLIM0      | Group 0 Limit Status. The SINC_STAT.GLIM0 indicates status for an amplitude and duration limit of secondary SINC filters assigned to group 0. This bit is set (=1) if any limit specified by registers SINC_LIMIT3 , SINC_LIMIT2 , SINC_LIMIT1 , or SINC_LIMIT0 , within the duration count and window specified by bits SINC_LEVEL1.LCNT and SINC_LEVEL1.LWIN are exceeded. To identify the offending secondary SINC filter, examine the filters status bits SINC_STAT.MAX3 , SINC_STAT.MAX2 , SINC_STAT.MAX1 , SINC_STAT.MAX0 , SINC_STAT.MIN3 , SINC_STAT.MIN2 , SINC_STAT.MIN1 and SINC_STAT.MAX0 according to the group 0 assignments in the SINC_CTL register. 0 Not Exceeded                                                                                                                                                                                                                                                                                                                         |
| 14 (R/NW)          | GSAT0      | 1 Exceeded Group 0 Saturation Status. The SINC_STAT.GSAT0 indicates status for the output saturation bit of primary SINC filters assigned to group 0. The bit is set (=1) if any filter of group 0 has its satu- ration status bit set (=1). To identify the offending SINC primary filter, examine bits SINC_STAT.PSAT3 , SINC_STAT.PSAT2 , SINC_STAT.PSAT1 , and SINC_STAT.PSAT0 accord- ing to the group 0 assignments specified by the SINC_CTL.EN3 , SINC_CTL.EN2 , SINC_CTL.EN1 , and SINC_CTL.EN0 bits. 0 Not Set                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 13 (R/W1C)         | PCNT0      | 1 Set Primary (Filter) Count for Group 0 Status. The SINC_STAT.PCNT0 indicates status for the output count of primary SINC fil- ters assigned to group 0. The bit is set (=1) each time the modulo number of outputs (specified by the SINC_LEVEL0.PCNT bits) has been transferred for each primary SINC filter assigned to group 0. Each count in SINC_LEVEL0.PCNT corresponds to one complete set or vector of samples from all SINC filter pairs assigned to group 1. For example, if group 0 is assigned three SINC filters pairs 0, 1, and 3, and SINC_LEVEL0.PCNT is set to 5, then this status bit is set after the transfer of every 5th complete sample vector, comprising 3 x 5= 15 data samples. This bit asserts when the memory transfer on the system SCB fabric is complete, and a valid SCB write data response is received by the SINC filter unit. If this status bit and bit SINC_CTL.EPCNT0 are set (=1), the SINC_DATA0 trigger is asserted. Write 1 to clear. 0 Not Reached 1 Reached |
| 13 (R/W1C)         |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W1C)         | FOVF0      | FIFO Overflow for Group 0 Status. The SINC_STAT.FOVF0 indicates status for the data output FIFO bit of primary SINC filters assigned to group 0. This bit is set (= 1) if the output FIFO for any filter in group 0 overflows due to slow SCB fabric response. The FIFO for each primary SINC filter contains two data sample locations. An overflow occurs if a third data sam- ple is generated before the first sample's data is transferred into the SCB fabric write data channel. After any overflow signaled by this bit occurs, all further SCB transmissions generated by group 1 are UNSPECIFIED until all SINC filters of the group are shut down and restarted. Clearing this status bit (=0) alone is not sufficient to re-sync theDMA stream. Write 1 to clear. If this status bit and bit SINC_CTL.EFOVF0 are set (=1), the SINC_STAT inter- rupt is asserted. | FIFO Overflow for Group 0 Status. The SINC_STAT.FOVF0 indicates status for the data output FIFO bit of primary SINC filters assigned to group 0. This bit is set (= 1) if the output FIFO for any filter in group 0 overflows due to slow SCB fabric response. The FIFO for each primary SINC filter contains two data sample locations. An overflow occurs if a third data sam- ple is generated before the first sample's data is transferred into the SCB fabric write data channel. After any overflow signaled by this bit occurs, all further SCB transmissions generated by group 1 are UNSPECIFIED until all SINC filters of the group are shut down and restarted. Clearing this status bit (=0) alone is not sufficient to re-sync theDMA stream. Write 1 to clear. If this status bit and bit SINC_CTL.EFOVF0 are set (=1), the SINC_STAT inter- rupt is asserted. |
| 12 (R/W1C)         | FOVF0      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | No Overflow                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 11 (R/W1C)         | PFAB0      | 1 Overflow Primary (Filter) Fabric Error for Group 0 Status. The SINC_STAT.PFAB0 indicates error status for the output of any primary SINC filter assigned to group 0. The bit is set (=1) if the SCB fabric provides a write error response for a filter output transfer associated with group 0, or if an overrun occurs for a filter in group 0. An interrupt is requested whenever this bit is =1 (not maskable).                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 1 Overflow Primary (Filter) Fabric Error for Group 0 Status. The SINC_STAT.PFAB0 indicates error status for the output of any primary SINC filter assigned to group 0. The bit is set (=1) if the SCB fabric provides a write error response for a filter output transfer associated with group 0, or if an overrun occurs for a filter in group 0. An interrupt is requested whenever this bit is =1 (not maskable).                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 11 (R/W1C)         | PFAB0      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Disabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 11 (R/W1C)         | PFAB0      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Enabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | MIN3       | Minimum for Secondary Filter 3 Status. The SINC_STAT.MIN3 bit indicates whether the output of the secondary SINC filter 3 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT3.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN3 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples.                                                                                                                                                                                                           | Minimum for Secondary Filter 3 Status. The SINC_STAT.MIN3 bit indicates whether the output of the secondary SINC filter 3 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT3.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN3 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples.                                                                                                                                                                                                           |
| 3 (R/W1C)          | MIN3       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Not Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 3 (R/W1C)          | MIN3       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | MIN2       | Minimum for Secondary Filter 2 Status. The SINC_STAT.MIN2 bit indicates whether the output of the secondary SINC filter 2 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT2.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN2 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples. 0 Not Exceeded | Minimum for Secondary Filter 2 Status. The SINC_STAT.MIN2 bit indicates whether the output of the secondary SINC filter 2 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT2.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN2 bits. For group 0, the duration limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the duration limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples. 0 Not Exceeded |
| 1 (R/W1C)          | MIN1       | Minimum for Secondary Filter 1 Status. The SINC_STAT.MIN1 bit indicates whether the output of the secondary SINC filter 1 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT1.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN1 bits. For group 0, the limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples.                                  | Minimum for Secondary Filter 1 Status. The SINC_STAT.MIN1 bit indicates whether the output of the secondary SINC filter 1 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT1.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN1 bits. For group 0, the limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples.                                  |
| 1 (R/W1C)          | MIN1       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Not Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1 (R/W1C)          | MIN1       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 32-30: SINC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W1C)          | MIN0       | Minimum for Secondary Filter 0 Status. The SINC_STAT.MIN0 bit indicates whether the output of the secondary SINC filter 0 exceeded its minimum amplitude and duration level. This bit is set (=1) if the limit is exceeded. The amplitude limit is specified by the SINC_LIMIT0.LMIN bits. The duration limit is specified in terms of an excursion count and window for the filter group to which the filter is assigned by the SINC_CTL.EN0 bits. For group 0, the limit is SINC_LEVEL0.LCNT counts within a window of SINC_LEVEL0.LWIN samples. For group 1, the limit is SINC_LEVEL1.LCNT counts within a window of SINC_LEVEL1.LWIN samples. 0 Not Exceeded |
| 0 (R/W1C)          | MIN0       | 1 Exceeded                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 0 (R/W1C)          | MIN0       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |