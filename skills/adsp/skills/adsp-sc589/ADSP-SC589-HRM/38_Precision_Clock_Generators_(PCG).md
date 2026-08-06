## 35   Precision Clock Generators (PCG)

The precision clock generators are used to produce a pair of signals from a clock input signal. The two signals generated are normally used as a serial bit clock and frame sync pair. The PCG is part of the DAI. There are two PCG signals in each DAI: PCGA and PCGB belong to DAI0, PCGC and PCGD belong to DAI1.

## Features

The following list describes the features of the precision clock generators.

- SRU allows the routing of all of the PCG signals in one DAI (four PCG signals in DAI0 or DAI1)
- Input clock selection: SYS\_CLKIN0 for PCG DAI0; SYS\_CLKIN1 for PCG DAI1; SCLK0 for external DAI pins
- Provides four different clock dividers for serial clock, frame sync, phase (20-bit), and pulse width (16-bit)
- Phase shift allows adjustment of the frame sync relative to the serial clock and can be shifted the full period and wrapped around
- Provides pulse width control for arbitrary frame sync signal generation
- Bypass mode for external frame sync manipulation
- External trigger mode starts PCG operation
- No additional jitter is introduced when using off-chip clocks

## Functional Description

The PCG Block Diagram shows the blocks within the module and its connection to the DAI. The following sections provide information on the function of these blocks.

Figure 35-1: PCG Block Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000000_cf0eb4217e9c9281e6ffcdce814f54d4ce1cb10328eaea6d31122e0a896387d7.png)

NOTE: In the PCG Block Diagram , SYS\_CLKIN0 is input to DAI0 and SYS\_CLKIN1 is input to DAI1.

## ADSP-SC58x PCG Register List

Precision Clock Generator

Table 35-1: ADSP-SC58x PCG Register List

| Name      | Description                                           |
|-----------|-------------------------------------------------------|
| PCG_CTLA0 | Precision Clock A Control 0 Register                  |
| PCG_CTLA1 | Precision Clock A Control 1 Register                  |
| PCG_CTLB0 | Precision Clock B Control 0 Register                  |
| PCG_CTLB1 | Precision Clock B Control 1 Register                  |
| PCG_CTLC0 | Precision Clock C Control 0 Register                  |
| PCG_CTLC1 | Precision Clock C Control 1 Register                  |
| PCG_CTLD0 | Precision Clock DControl 0 Register                   |
| PCG_CTLD1 | Precision Clock DControl 1 Register                   |
| PCG_PW1   | Precision Clock Pulse Width Control 1 Register        |
| PCG_PW2   | Precision Clock Pulse Width Control 2 Register        |
| PCG_SYNC1 | Precision Clock Frame Sync Synchronization 1 Register |
| PCG_SYNC2 | Precision Clock Frame Sync Synchronization 2 Register |

## Internal Interface

The fundamental clock of the PCG is SCLK0 . The clock to this module can be shut off for power savings.

## Serial Clock

Each of the four units (A, B, C, and D) produces a clock output. Serial clock generation from a unit is independently enabled and controlled. Sources for the serial clock generation can be either from the CLKIN , SCLK0 , or a DAI pin source. When CLKIN is chosen as input clock in PCG, the PCGs in DAI0 use SYS\_CLKIN0 and the PCGs in DAI1 use SYS\_CLKIN1 . The clock output is derived from the input to the PCG with a 20-bit divisor.

Note that the divider is working in normal mode for PCG0\_CTLx1.CLKDIV &gt; 1. For PCG\_CTLA1.CLKDIV / PCG\_CTLB1.CLKDIV = 0 or 1, the divider operates in bypass mode (input clock is fed directly to its output). In bypass mode, the clock at the output can theoretically run at up to the SCLK0 frequency. Check the data sheet for the specified maximum operation speed of the DAI pin buffers.

Note that the clock output is always set (as closely as possible) to a 50% duty cycle. If the clock divisor is even, the duty cycle of the clock output is exactly 50%. If the clock divisor is odd, then the duty cycle is slightly less than 50%. The low period of the output clock is one input clock period more than the high period of the output clock. For higher values of an odd divisor, the duty cycle is close to 50%.

NOTE: A PCG clock output cannot be fed to its own input.

## Frame Sync

The following sections describe the use of frame syncs in the PCGs.

## Frame Sync Output

Each of the four units (A through D) produces a synchronization signal for framing serial data. The frame sync outputs are much more flexible since they must accommodate the wide variety of serial protocols used by peripherals.

Frame sync generation from a unit is independently enabled and controlled. Sources for the frame sync generation can be either from the crystal buffer output, SCLK0, or an external pin source. There is only one external source pin for both frame sync and clock output for a unit.

If an external source is selected for both frame sync and clock output for a unit, then they operate on the same input signal. Apart from enable and source select control bits, a 20-bit divisor controls frame sync generation.

## Divider Mode Selection

If a frame sync divisor is greater than 1, the PCG frame sync output frequency is equal to the input clock frequency, divided by a 20-bit integer. This integer is specified in the PCG\_CTLA0.FSDIV / PCG\_CTLB0.FSDIV bit field (bits 19:0).

However, if the frame sync divisor is 0 or 1, the PCG's frame sync clock generation unit is bypassed, and the frame sync input is connected directly to the frame sync output. For PCG\_CTLB0.FSDIV =0, 1 the PCG\_PW1 / PCG\_PW2 registers function differently than in normal mode.

## Phase Shift

Phase shift is a frame sync parameter that defines the phase shift of the frame sync relative to the input clock of the same unit. This feature allows the shifting of the frame sync signal in time relative to the clock input signal. Frame sync phase shifting is often required by peripherals that need a frame sync signal to lead or lag a clock signal.

For example, the I 2 S protocol specifies that the frame sync transition from high-to-low occurs one clock cycle before the beginning of a frame. Since an I 2 S frame is 64 clock cycles long, delaying the frame sync by 63 cycles produces the required framing.

Phase shifting is represented as a full 20-bit value. Even when the frame sync is divided by the maximum amount, the phase can be shifted to the full range, from zero to one input clock short of the period.

- NOTE: Phase shifting is specified as a 2 x 10-bit divider value in the PCG\_CTLA0.FSPHASEHI bit field (bits 29:20) and in the PCG\_CTLA1.FSPHASELO bit field (bits 29:20).

A single 20-bit value spans these two-bit fields. The upper half of the word (bits 19:10) is in the PCG\_CTLA0 register, and the lower half (bits 9:0) is in the PCG\_CTLA1 register.

The phase shift between clock and frame sync outputs can be programmed using the PCG\_PW1 / PCG\_PW2 registers and all of the control registers under these conditions:

- The input clock source for the clock generator output and the frame sync generator output is the same.
- The clock and frame sync are enabled at the same time using a single atomic instruction.
- The frame sync divisor is an integral multiple of the clock divisor.
- NOTE: When using a clock and frame sync as a synchronous pair, the units must be enabled in a single atomic instruction before their parameters are modified. Both units must also be disabled in a single atomic instruction.
- NOTE: If the phase shift is 0 (see the Phase and Pulse Width Settings figure), the clock and frame sync outputs rise at the same time.

If the phase shift is 1, the frame sync output transitions one input clock period ahead of the clock transition.

If the phase shift is divisor - 1, the frame sync transitions divisor - 1 input clock periods ahead of the clock transitions.

Figure 35-2: Phase and Pulse Width Settings

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000001_f4f5663477aede89d0debbc7b80f0f04318813004a4ae1f3d0fd9c26bf655609.png)

NOTE: When generating single frame sync pulses (the length of one SPORT clock cycle), take care with respect to the drive and sampling edges. If the rules are violated, for example if the SPORT is not driving data, the module cannot detect a valid sample edge.

## Pulse Width

Pulse width is the number of input clock periods for which the frame sync output is high.

A 16-bit value determines the width of the framing pulse. Settings for pulse width can range from zero to DIV - 1. The pulse width should be less than the divisor of the frame sync. The pulse width of frame sync is specified in the PCG\_PW1.FSA , PCG\_PW1.FSB , PCG\_PW2.FSC , and PCG\_PW2.FSD bits (15-0) and (31-16).

## Default Pulse Width

If the pulse width count is equal to 0 and if the PCG\_CTLA0.FSDIV / PCG\_CTLB0.FSDIV bit field is even, then the actual pulse width of the frame sync output is equal to:

For even divisors: frame sync divisor/2

If the pulse width count is equal to 0 and if the PCG\_CTLA0.FSDIV / PCG\_CTLB0.FSDIV bit field is odd, then the actual pulse width of the frame sync output is equal to:

For odd divisors: frame sync divisor - 1/2

## Input Clock Source Considerations

The core Phase-Locked Loop (PLL) has been designed to provide clocking for the processor core. The performance specifications of this PLL are appropriate for the core. But, they have not been optimized or specified for precision data converters where jitter directly translates into time quantization errors and distortion.

Therefore, the PCG allows the routing of external clock sources which are independent of the core PLL.

## Timing Example for I 2 S Mode

For I 2 S mode, the frame sync should be driven at the falling edge of SPORT clock. In other words, the frame sync edge must coincide with the falling edge of the SPORT clock. To satisfy this requirement, program the phase of the frame sync accordingly in the PCG control registers.

For example, assume that the input clock source for both clock and frame sync are the same and both the clock and frame sync are enabled at the same time. Also, assume that the clock divisor value for generating the required SPORT clock is PCG\_CTLA1.CLKDIV = 4. Then, for a 32-bit word length, the frame sync divisor value is:

```
PCG_CTLA0.FSDIV = 64, PCG_CTLA1.CLKDIV = 256.
```

By default, for phase = 0, the rising edge of both SPORT clock and frame sync coincide. To make sure that the frame sync edges coincide with the falling edge of the SPORT clock, program the phase value as:

PCG\_CTLA1.CLKDIV /2 = 2.

## Cross Mode Connections

The symmetric dual DAI architecture allows cross connections between both PCGs (A,B) and (C,D) to the other DAI. Each PCG (A through D) supports an alternative input clock (PCG0\_EXTx\_I) (see Figure 35-1 PCG Block Diagram) which can be sourced via a DAI pin buffer from the other DAI. Note however if routing a source (clock or FS) only DAI pin buffer 2 to 20 can be used (DAI pin buffer 1 is no longer available and is replaced by the DAI CRS buffer for the other DAI). See DAI Routing Capabilities for more information.

## Operating Modes

The following sections provide information on the operating modes of the precision clock generator.

## Normal Mode

When the frame sync divisor is set to any value other than zero or one, the PCGs operate in normal mode. In normal mode, the divisor determines the frequency of the frame sync output where:

Frequency of Frame Sync Output = Input Frequency/Divisor

The value of the pulse width control determines the high period of the frame sync output. The value of the pulse width control must be less than the value of the divisor.

The value of the phase control determines the phase of the frame sync output. If the phase is zero, then the positive edges of the clock and frame sync coincide when:

- the clock and frame sync dividers are enabled at the same time using an atomic instruction
- the divisors of the clock and frame sync are the same
- the source for the clock and frame sync is the same

The number of input clock cycles that have already elapsed before the frame sync is enabled is equal to the difference between the divisor and the phase values. If the phase is a small fraction of the divisor, then the frame sync appears

to lead the clock. If the phase is only slightly less than the frame sync divisor, then the frame sync appears to lag the clock. The frame sync phase must not be greater than the divisor.

## Bypass Mode

When the frame sync divisor for the frame sync has a value of zero or one, the frame sync is in bypass mode, and the PCG\_PW1 / PCG\_PW2 registers have different functionality than in normal mode.

NOTE: In normal mode, bits 15:0 and 31:18 of the PCG\_PW1 / PCG\_PW2 registers are used to program the pulse width count. In bypass mode, bits 15:2 and 31:18 are ignored. Bits 1:0 and 17:16 are renamed to PCG\_PW1.STROBEA and PCG\_PW1.INVFSA , respectively. This functionality is described in more detail as follows.

If the PCG\_PW1.STROBEA register is cleared, then the input is directly passed (see the Bypass and Inverted Bypass figure) to the frame sync output either inverted or not inverted, depending on the PCG\_PW1.INVFSA bits.

Figure 35-3: Bypass and Inverted Bypass

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000002_ef9a73ccbd72000ca9681a11587b50b0079f2f663def588c77fc5cb7ed70b4d7.png)

## One-Shot Mode

In the one-shot mode operation shown in the One Shot Mode PCG A (MISCA2\_I Input) figure, the PCG produces a series of periods but does not run continuously.

Figure 35-4: One Shot Mode PCG A (MISCA2\_I Input)

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000003_ac707ccad7ce782fba09f9fb2b054cccdc342f356fb5044dfe5eaf694abf25a2.png)

Bypass mode also enables the generation of a strobe pulse (one-shot frame sync). Strobe usage ignores the divider counters and looks to the SRU to provide the input signal. T wo-bit fields determine the operation in this mode.

In the bypass mode, if the PCG\_PW1.STROBEA bit =1, then a one-shot pulse is generated. This one-shot pulse has the duration equal to the period of DAI\_MISCAx\_I for the PCGx unit. This pulse is generated either at the falling or rising edge of the input clock, depending on the value of the PCG\_PW1.INVFSA bit. The output pulse

width is equal to the period of the SRU source signal DAI\_MISCAx\_I . The pulse begins at the second rising edge of MISCAx\_I following a rising edge of the clock input. When the PCG\_PW1.INVFSA bit is set, the pulse begins at the second rising edge of DAI\_MISCAx\_I coinciding with or following a falling edge of the clock input.

NOTE: A strobe period is defined to be the period of the FS input clock signal as specified by the PCG\_CTLA1.FSSRC bit.

## PCG Event Control

The following sections describe the generation and control of PCG events.

## External Event Trigger

The trigger with the external clock is enabled by setting bits 0 and 16 of the PCG\_SYNC1 / PCG\_SYNC2 registers .

Refer to the FS Output Synchronization With External Trigger Input figure. Since the rising edge of the external clock is used to synchronize with the frame sync, the frame sync output is not generated until a rising edge of the external clock is sensed.

Figure 35-5: FS Output Synchronization With External Trigger Input

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000004_70ccc4e8b40d523786dd3dc91c2c9a9494061a21b17421f4401271d3c4f2f08a.png)

## External Event Trigger Delay

The time delay between the rising trigger edge and the start of serial clock and frame sync varies between 2.5 to 3.5 input clock periods. If the input clock and the trigger signal are synchronous, the delay is 3 input clock periods. Consider the following cases:

- SCLK0 is the input source. In this case, if the given trigger event is synchronous to SCLK0, the delay is 3 SCLK0 periods. If the trigger signal is asynchronous with SCLK0, the delay varies from 2.5 SCLK0 periods to 3.5 SCLK0 periods. (It depends on whether the trigger edge occurs in the positive half cycle or negative half cycle of SCLK0.)
- The SYS\_CLKIN0 signal is the input source for PCG AB. The SYS\_CLKIN1 signal is the input source for PCG CD. In this case if the given trigger signal is synchronous to CLKIN, the delay is 3 CLKIN periods. But if they are asynchronous to CLKIN, the delay can vary between 2.5 CLKIN periods to 3.5 CLKIN periods.
- SRU DAI0 is the input source for PCG AB. SRU DAI1 is the input source for PCG CD. If the input clock and trigger signal are synchronous, the delay is exactly 3 input clock periods. If asynchronous, it varies between 2.5 to 3.5 input clock periods depending on the phase difference between the input clock and trigger signal.

## Audio System Example

The PCG Setup for I2S or Left-Justified DAI figure shows an example of the interconnections between the S/PDIF receiver, ASRC, and the PCGs. The interconnections are made by programming the signal routing unit. It shows how to set up two precision clock generators using the S/PDIF receiver and an asynchronous sample rate converter (ASRC) to interface to an external audio DAC. The PCG is configured to provide a fixed ASRC/DAC output sample rate of 65.098 kHz. The input to the S/PDIF receiver is typically 44.1 kHz if supplied by a CD player, but can also be from other source at any nominal sample rates.

Similarly, the phase shift for frame syncs B, C, and D is specified in the corresponding control registers ( PCG\_CTLA0 through PCG\_CTLB1 ).

Figure 35-6: PCG Setup for I 2 S or Left-Justified DAI

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000005_4fa5fdee7bca277cf0dd393fe90a19cbdf23ce1e3cafe8a3c6c1f661d4a3ba0f.png)

Three synchronous clocks are required in audio systems:

1. Frame sync (FS)
2. Serial bit clock (64 FS)
3. Master DAC clock (256 FS)

Since each PCG has only two outputs, this example requires two PCGs. Furthermore, because the digital audio interface requires a fixed-phase relation between serial clock and FS, these two outputs should come from one PCG (PCG A). The master clock comes from the second (PCG B).

The combined PCGs can provide a selection of synchronous clock frequencies to support alternate sample rates for the ASRCs and external DACs. However, the range of choices is limited by CLKIN and the ratio of PCG\_CLKx\_O : serial clock: FS . The ratio is normally fixed at 256:64:1 to support digital audio left-justified, I 2 S and right-justified interface modes.

Many DACs also support 384, 512, and 786x FS for PCG\_CLKx\_O , which allows some additional flexibility in choosing serial clock.

Note that the falling edge of serial clock must always be synchronous with both edges of FS. This condition requires that the phase of the serial clock and FS signals for a common PCG (PCG A) be adjustable.

While the frequency of the master DAC clock ( PCG\_CLKx\_O ) must be synchronous with the sample rate supplied to the external DAC, there is no fixed-phase requirement.

Set the clock divisor and source and low-phase word first, followed by the control register enable bits, which are set together. When the PCG\_PW1 / PCG\_PW2 registers are set to zero (default), the FS pulse width is (divisor 2) for even divisors and (divisor - 1) 2 for odd divisors. Alternatively, the PCG\_PW1 / PCG\_PW2 registers can be set high for exactly one-half the period of CLKIN cycles for a 50% duty cycle, provided the FS divisor is an even number.

## Clock Configuration Examples

For a CLKIN = 33.330 MHz, the two PCGs provide the three synchronous clocks PCGx\_CLK, serial clock and FS for the SRCs and external DAC. These divisors are stored in the PCG\_CTLA1.CLKDIV / PCG\_CTLB1.CLKDIV bit fields.

The integer divisors for several possible sample rates based on 33.330 MHz CLKIN are shown in the Precision Clock Generator Division Ratios (33.330 CLKIN) table.

Table 35-2: Precision Clock Generator Division Ratios (33.330 CLKIN)

|                  | PCG Divisors   | PCG Divisors   |   FSDIV A *1 |
|------------------|----------------|----------------|--------------|
| Sample Rate kHz) | CLKDIV B       | CLKDIV A       |              |
| 130.195          | 1              | 4              |          256 |
| 65.098           | 2              | 8              |          512 |
| 43.398           | 3              | 12             |          768 |
| 32.549           | 4              | 16             |         1024 |
| 26.039           | 5              | 20             |         1280 |
| 21.699           | 6              | 24             |         1536 |
| 18.599           | 7              | 28             |         1792 |

## Programming Model

This section describes the sequence of software steps required for successful PCG operation.

If the PCG is disabled to reprogram a parameter, use a delay after writing to the disable bit. This delay in core clock ( CCLK ) cycles = (PCG source clock period/ CCLK period). In summary, use the following general procedure:

1. Clear the PCG enable bits without modifying any other settings.
2. Wait for N CCLK cycles (N = PCG source clock period/processor clock period).
3. Program all new parameters without setting the PCG enable bit.
4. Enable the PCG.

## Frame Sync Phase Setting

The phase unit requires that the clock and FS are enabled simultaneously in an atomic instruction.

1. Write the clock divider/low 10-bit phase divider to the PCG\_CTLA1 / PCG\_CTLB1 registers.
2. Program the FS divider/high 10-bit phase divider, enabling both the PCG\_CTLA0.CLKEN / PCG\_CTLA0.FSEN and the PCG\_CTLB0.CLKEN / PCG\_CTLB0.FSEN bits.

Note that both units must be disabled in the same way.

## External Event Trigger

The trigger with the external clock is is enabled by setting bits 0 and 16 of the PCG\_SYNC register. Program the phase to 3, so that the rising edge of the external clock is in-sync with the frame sync ( FS Output Synchronization With External Trigger Input ).

Use the following steps.

1. Program the PCG\_SYNC1 / PCG\_SYNC2 registers and the PCG\_CTLA0 through PCG\_CTLB1 registers appropriately.
2. Enable the clock or frame sync, or both.

Since the rising edge of the external clock is used to synchronize with the frame sync, the frame sync output is not generated until a rising edge of the external clock is sensed.

## Debug Features

Take care in cases where any input to the phase unit is modified. Any individual change of the PCG\_CTLA1.CLKDIV or PCG\_CTLA0.FSDIV dividers can cause a failure in the PCG sync operation between the serial clock and the frame sync. Only the programming model ensures a correct setup for phase settings.

## ADSP-SC58x PCG Register Descriptions

Precision Clock Generator (PCG) contains the following registers.

Table 35-3: ADSP-SC58x PCG Register List

| Name      | Description                          |
|-----------|--------------------------------------|
| PCG_CTLA0 | Precision Clock A Control 0 Register |
| PCG_CTLA1 | Precision Clock A Control 1 Register |
| PCG_CTLB0 | Precision Clock B Control 0 Register |
| PCG_CTLB1 | Precision Clock B Control 1 Register |
| PCG_CTLC0 | Precision Clock C Control 0 Register |
| PCG_CTLC1 | Precision Clock C Control 1 Register |

Table 35-3: ADSP-SC58x PCG Register List (Continued)

| Name      | Description                                           |
|-----------|-------------------------------------------------------|
| PCG_CTLD0 | Precision Clock DControl 0 Register                   |
| PCG_CTLD1 | Precision Clock DControl 1 Register                   |
| PCG_PW1   | Precision Clock Pulse Width Control 1 Register        |
| PCG_PW2   | Precision Clock Pulse Width Control 2 Register        |
| PCG_SYNC1 | Precision Clock Frame Sync Synchronization 1 Register |
| PCG_SYNC2 | Precision Clock Frame Sync Synchronization 2 Register |

## Precision Clock A Control 0 Register

The PCG\_CTLA0 register enables the clock, frame sync, and select divisor for the PCG0 clock A signal.

Figure 35-7: PCG\_CTLA0 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000006_458903284be5fc3781f9dc726b0c72318b2420516c1512a2c8e65fb7328ab12b.png)

Table 35-4: PCG\_CTLA0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLA0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLA0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLA0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLA0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLA0.FSDIV >1 PCGx is in normal mode, PCG_CTLA0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock A Control 1 Register

The PCG\_CTLA1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock A signal.

Figure 35-8: PCG\_CTLA1 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000007_b899aa1721b9f5af28016c1e8d1043b558e21e7bc2abff3bd522295d34ad66d4.png)

Table 35-5: PCG\_CTLA1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLA1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock                                                  |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLA1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLA1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLA1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock B Control 0 Register

The PCG\_CTLB0 register enables the clock, frame sync, and select divisor for the PCG0 clock B signal.

Figure 35-9: PCG\_CTLB0 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000008_458903284be5fc3781f9dc726b0c72318b2420516c1512a2c8e65fb7328ab12b.png)

Table 35-6: PCG\_CTLB0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLB0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLB0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLB0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLB0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLB0.FSDIV >1 PCGx is in normal mode, PCG_CTLB0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock B Control 1 Register

The PCG\_CTLB1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock B signal.

Figure 35-10: PCG\_CTLB1 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000009_b899aa1721b9f5af28016c1e8d1043b558e21e7bc2abff3bd522295d34ad66d4.png)

Table 35-7: PCG\_CTLB1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLB1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock                                                  |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLB1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLB1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLB1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock C Control 0 Register

The PCG\_CTLC0 register enables the clock, frame sync, and select divisor for the PCG0 clock C signal.

Figure 35-11: PCG\_CTLC0 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000010_01441c568864ca0417f2227c9e0db706a009d9c04b079da6e4da92881cece3a8.png)

Table 35-8: PCG\_CTLC0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLC0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLC0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLC0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLC0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLC0.FSDIV >1 PCGx is in normal mode, PCG_CTLC0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock C Control 1 Register

The PCG\_CTLC1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock C signal.

Figure 35-12: PCG\_CTLC1 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000011_b899aa1721b9f5af28016c1e8d1043b558e21e7bc2abff3bd522295d34ad66d4.png)

Table 35-9: PCG\_CTLC1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLC1.CLKSRC bit specifies the clock source. 0 CLKIN1 pin selected for clock                                                  |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLC1.FSSRC bit specifies the frame sync source. 0 CLKIN1 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLC1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLC1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock D Control 0 Register

The PCG\_CTLD0 register enables the clock, frame sync, and select divisor for the PCG0 clock D signal.

Figure 35-13: PCG\_CTLD0 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000012_458903284be5fc3781f9dc726b0c72318b2420516c1512a2c8e65fb7328ab12b.png)

Table 35-10: PCG\_CTLD0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLD0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLD0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLD0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLD0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLD0.FSDIV >1 PCGx is in normal mode, PCG_CTLD0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock D Control 1 Register

The PCG\_CTLD1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock D signal.

Figure 35-14: PCG\_CTLD1 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000013_b899aa1721b9f5af28016c1e8d1043b558e21e7bc2abff3bd522295d34ad66d4.png)

Table 35-11: PCG\_CTLD1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLD1.CLKSRC bit specifies the clock source. 0 CLKIN1 pin selected for clock                                                  |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLD1.FSSRC bit specifies the frame sync source. 0 CLKIN1 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLD1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLD1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock Pulse Width Control 1 Register

The PCG\_PW1 register sets the one shot frame sync and the active low frame sync select for PCG A and PCG B.

Figure 35-15: PCG\_PW1 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000014_e71893e28c3ee0f3b5247ae519a2c8f917f0be8b1c1116ab934e7218d99b2392.png)

Table 35-12: PCG\_PW1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | INVFSB     | Active Low Frame Sync Select for Frame in Bypass PCG B. The PCG_PW1.INVFSB bit selects active low or active high frame sync in bypass mode for PCG B.                                                                                       |
| 16 (R/W)           | STROBEB    | One Shot Frame Sync Bypass Mode PCG B. The PCG_PW1.STROBEB bit sets the frame sync pulse in bypass mode for PCG B. This is the duration equal to one period of the DAI_MISCA2_I signal (PCG B) re- peating at the beginning of every frame. |
| 31:16 (R/W)        | FSB        | Pulse Width for Frame Sync PCG B. The PCG_PW1.FSB bit field sets the number of input clock periods for which the frame sync output is high. Pulse width should be less than the divisor of the frame sync.                                  |
| 1 (R/W)            | INVFSA     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW1.INVFSA bit selects active low or active high frame sync for PCG A in bypass mode.                                                                                   |
| 15:0 (R/W)         | FSA        | Pulse Width for Frame Sync PCG A. The PCG_PW1.FSA bit field sets the number of input clock periods for which the frame sync output is high. Pulse width should be less than the divisor of the frame sync.                                  |

Table 35-12: PCG\_PW1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | STROBEA    | One Shot Frame Sync PCG A. The PCG_PW1.STROBEA bit sets the frame sync pulse for PCG A in bypass mode. This is the duration equal to one period of the DAI_MISCA2_I signal (PCG A) re- peating at the beginning of every frame. |

## Precision Clock Pulse Width Control 2 Register

The PCG\_PW2 register sets the one shot frame sync and the active low frame sync select for PCG C and PCG D.

Figure 35-16: PCG\_PW2 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000015_ee0b4245e467a685dbd0f787ca70bed981a9819ec999fc787b540d5a980596a2.png)

Table 35-13: PCG\_PW2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | INVFSD     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW2.INVFSD bit selects active low or active high frame sync for PCGD in bypass mode.                                                                       |
| 16 (R/W)           | STROBED    | One Shot Frame Sync PCG D. The PCG_PW2.STROBED bit sets the frame sync pulse for PCG Din bypass mode. This is the duration equal to one period of the DAI_MISCA4_I signal (PCG D) re- peating at the beginning of every frame. |
| 31:16 (R/W)        | FSD        | Pulse Width for Frame Sync PCG D. The PCG_PW2.FSD bit field sets the number of input clock periods for which the frame sync output is high for PCG D. Pulse width should be less than the divisor of the frame sync.           |
| 1 (R/W)            | INVFSC     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW2.INVFSC bit selects active low or active high frame sync for PCG C in bypass mode.                                                                      |
| 15:0 (R/W)         | FSC        | Pulse Width for Frame Sync PCG C. The PCG_PW2.FSC bit field sets the number of input clock periods for which the frame sync output is high for PCG C. Pulse width should be less than the divisor of the frame sync.           |

Table 35-13: PCG\_PW2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | STROBEC    | One Shot Frame Sync PCG C. The PCG_PW2.STROBEC bit sets the frame sync pulse for PCG C in bypass mode. This is the duration equal to one period of the DAI_MISCA3_I signal (PCG C) re- peating at the beginning of every frame. |

## Precision Clock Frame Sync Synchronization 1 Register

The PCG\_SYNC1 register allows programs to synchronize the clock frame sync units with external frame syncs. Note that the PCG\_CTLA1.CLKSRC bit is overridden if PCG\_SYNC1.CLKASRC bit in the PCG\_SYNC1 register is set.

Figure 35-17: PCG\_SYNC1 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000016_f5351c3bddb720ecc72890eebf0c046d86d826deff8067ae86b263394b6405e9.png)

Table 35-14: PCG\_SYNC1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | FSBSRC     | Frame Sync B Source. The PCG_SYNC1.FSBSRC bit enables the frame sync B input source. 0 Output selected by FSBSOURCE bit                                                                     |
| 18 (R/W)           | CLKBSRC    | Clock B Source. The PCG_SYNC1.CLKBSRC bit enables the clock B input source. 0 Output selected by CLKBSOURCE bit                                                                             |
| 17 (R/W)           | CLKB       | 1 Clock derived from core PLL selected for clock B Clock B Enable. The PCG_SYNC1.CLKB bit enables synchronization of clock B with the external frame sync. 0 Clock disabled 1 Clock enabled |

Table 35-14: PCG\_SYNC1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | FSB        | Frame Sync B Enable. The PCG_SYNC1.FSB bit enables synchronization of frame sync B with the external frame sync.                                                   |
| 3 (R/W)            | FSASRC     | Frame Sync Source. The PCG_SYNC1.FSASRC bit enables the frame sync A input source.                                                                                 |
| 2 (R/W)            | CLKASRC    | Clock A Source. The PCG_SYNC1.CLKASRC bit enables the clock A input source. 0 Output selected by CLKASOURCE bit 1 Clock derived from core PLL selected for clock A |
| 1 (R/W)            | CLKA       | Clock A Enable. The PCG_SYNC1.CLKA bit enables synchronization of clock A with the external frame sync. 0 Clock disabled                                           |
| 0 (R/W)            | FSA        | Frame Sync A Enable. The PCG_SYNC1.FSA bit enables synchronization of frame sync A with the external frame sync.                                                   |

## Precision Clock Frame Sync Synchronization 2 Register

The PCG\_SYNC2 register allows programs to synchronize the clock frame sync units with external frame syncs. Note that the PCG\_CTLD1.CLKSRC bit is overridden if PCG\_SYNC2.CLKDSRC bit in the PCG\_SYNC2 register is set.

Figure 35-18: PCG\_SYNC2 Register Diagram

![Image](38_Precision_Clock_Generators_(PCG)_artifacts/image_000017_df91a12818cb5c083a0b01b341054e742a02be5b90ffc2b2158dd5aca592d0b6.png)

Table 35-15: PCG\_SYNC2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | FSDSRC     | Frame Sync DSource. The PCG_SYNC2.FSDSRC bit enables the frame sync Dinput source. 0 Output selected by FSDSOURCE bit                                                                    |
| 18 (R/W)           | CLKDSRC    | Clock DSource. The PCG_SYNC2.CLKDSRC bit enables the clock Dinput source. 0 Output selected by CLKDSOURCE bit                                                                            |
| 17 (R/W)           | CLKD       | 1 Clock derived from core PLL selected for clockD Clock DEnable. The PCG_SYNC2.CLKD bit enables synchronization of clock Dwith the external frame sync. 0 Clock disabled 1 Clock enabled |

Table 35-15: PCG\_SYNC2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | FSD        | Frame Sync DEnable. The PCG_SYNC2.FSD bit enables synchronization of frame sync Dwith the external frame sync.                                                     |
| 3 (R/W)            | FSCSRC     | Frame Sync Source. The PCG_SYNC2.FSCSRC bit enables the frame sync C input source.                                                                                 |
| 2 (R/W)            | CLKCSRC    | Clock C Source. The PCG_SYNC2.CLKCSRC bit enables the clock C input source. 0 Output selected by CLKCSOURCE bit 1 Clock derived from core PLL selected for clock C |
| 1 (R/W)            | CLKC       | Clock C Enable. The PCG_SYNC2.CLKC bit enables synchronization of clock C with the external frame sync. 0 Clock disabled                                           |
| 0 (R/W)            | FSC        | Frame Sync C Enable. The PCG_SYNC2.FSC bit enables synchronization of frame sync C with the external frame sync.                                                   |