## 31   Precision Clock Generators (PCG)

The precision clock generators are used to produce a pair of signals from a clock input signal. The two signals generated are normally used as a serial bit clock and frame sync pair. The PCG is part of the DAI. There are four PCG units per DAI. PCGA , PCGB , PCGE , and PCGF belong to DAI0; PCGC , PCGD , PCGG , and PCGH belong to DAI1.

Each of these four units (A, B, E, and F) generates one clock (CLKA\_O, CLKB\_O, CLKE\_O, and CLKF\_O) and one frame sync (FSA\_O, FSB\_O, FSE\_O, and FSF\_O) output. In addition to clock and frame sync outputs, each of these four units generates one inverted version of clock output (INV\_CLKA\_O, INV\_CLKB\_O, INV\_CLKE\_O, and INV\_CLKF\_O) and one inverted version of frame sync output (INV\_FSA\_O, INV\_FSB\_O, INV\_FSE\_O, and INV\_FSF\_O ).

## Features

The following list describes the features of the precision clock generators.

- SRU allows the routing of all of the PCG signals in one DAI (two PCG units in DAI)
- FracNPLL output clock that can be used as input clock for all PCGs.
- Input clock selection: SYS\_CLKIN0 and SYS\_CLKIN1 can be used as the input clock for PCG, when configured to use CLKIN as clock source
- Provides four different clock dividers for serial clock, frame sync, phase (20-bit), and pulse width (16-bit)
- Phase shift allows adjustment of the frame sync relative to the serial clock and can be shifted the full period and wrapped around
- Provides pulse width control for arbitrary frame sync signal generation
- Bypass mode for external frame sync manipulation
- External trigger mode starts PCG operation
- No additional jitter is introduced when using off-chip clocks
- Global enable. (See the DAI chapter for details)

## Functional Description

The PCG Block Diagram shows the blocks within the module and its connection to the DAI. The following sections provide information on the function of these blocks.

Figure 31-1: PCG Block Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000000_26d8d936c2c21e015b310818a271d79f46170f347867bc3d9ef2e27603354d69.png)

NOTE: In the PCG Block Diagram , the CLKINSEL bit field determines whether SYS\_CLKIN0 or SYS\_CLKIN1 is input to DAI.

## ADSP-2184x PCG Register List

Precision Clock Generator

Table 31-1: ADSP-2184x PCG Register List

| Name      | Description                          |
|-----------|--------------------------------------|
| PCG_CTLA0 | Precision Clock A Control 0 Register |
| PCG_CTLA1 | Precision Clock A Control 1 Register |
| PCG_CTLB0 | Precision Clock B Control 0 Register |
| PCG_CTLB1 | Precision Clock B Control 1 Register |
| PCG_CTLC0 | Precision Clock C Control 0 Register |
| PCG_CTLC1 | Precision Clock C Control 1 Register |
| PCG_CTLD0 | Precision Clock DControl 0 Register  |
| PCG_CTLD1 | Precision Clock DControl 1 Register  |
| PCG_CTLE0 | Precision Clock E Control 0 Register |
| PCG_CTLE1 | Precision Clock E Control 1 Register |
| PCG_CTLF0 | Precision Clock F Control 0 Register |

Table 31-1: ADSP-2184x PCG Register List (Continued)

| Name      | Description                                           |
|-----------|-------------------------------------------------------|
| PCG_CTLF1 | Precision Clock F Control 1 Register                  |
| PCG_CTLG0 | Precision Clock G Control 0 Register                  |
| PCG_CTLG1 | Precision Clock G Control 1 Register                  |
| PCG_CTLH0 | Precision Clock HControl 0 Register                   |
| PCG_CTLH1 | Precision Clock HControl 1 Register                   |
| PCG_PW1   | Precision Clock Pulse Width Control 1 Register        |
| PCG_PW2   | Precision Clock Pulse Width Control 2 Register        |
| PCG_PW3   | Precision Clock Pulse Width Control 3 Register        |
| PCG_PW4   | Precision Clock Pulse Width Control 4 Register        |
| PCG_SYNC1 | Precision Clock Frame Sync Synchronization 1 Register |
| PCG_SYNC2 | Precision Clock Frame Sync Synchronization 2 Register |
| PCG_SYNC3 | Precision Clock Frame Sync Synchronization 3 Register |
| PCG_SYNC4 | Precision Clock Frame Sync Synchronization 4 Register |

## ADSP-2184x PCG Trigger List

Table 31-2: ADSP-2184x PCG Trigger List Generators

|   Trigger ID | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|          100 |        | Reserved      |               |
|          101 |        | Reserved      |               |
|          102 |        | Reserved      |               |
|          103 |        | Reserved      |               |
|          104 |        | Reserved      |               |
|          105 |        | Reserved      |               |
|          106 |        | Reserved      |               |
|          107 |        | Reserved      |               |

Table 31-3: ADSP-2184x PCG Trigger List Receivers

|   Trigger ID | Name     | Description                         | Sensitivity   |
|--------------|----------|-------------------------------------|---------------|
|          128 | PCG0_HWA | PCG0 PCG-A Hardware trigger control | Pulse         |
|          129 | PCG0_HWB | PCG0 PCG-B Hardware trigger control | Pulse         |
|          130 | PCG0_HWC | PCG0 PCG-C Hardware trigger control | Pulse         |

Table 31-3: ADSP-2184x PCG Trigger List Receivers (Continued)

|   Trigger ID | Name     | Description                         | Sensitivity   |
|--------------|----------|-------------------------------------|---------------|
|          131 | PCG0_HWD | PCG0 PCG-D Hardware trigger control | Pulse         |
|          132 | PCG0_HWE | PCG0 PCG-E Hardware trigger control | Pulse         |
|          133 | PCG0_HWF | PCG0 PCG-F Hardware trigger control | Pulse         |
|          134 | PCG0_HWG | PCG0 PCG-G Hardware trigger control | Pulse         |
|          135 | PCG0_HWH | PCG0 PCG-H Hardware trigger control | Pulse         |

## Internal Interface

The fundamental clock of the PCG is SCLK0 . The clock to this module can be shut off for power savings.

## Serial Clock

Each of the four units (A, B, C, and D) produces a clock output. Serial clock generation from a unit is independently enabled and controlled. Sources for the serial clock generation can be either from the CLKIN, SCLK0, or a DAI pin source.

When CLKIN is chosen as input clock in PCG, the clock source bits ( PCG\_SYNC1.CLKA\_CLKINSEL , PCG\_SYNC1.CLKB\_CLKINSEL ) determine whether the clock source is SYS\_CLKIN0 .

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

However, if the frame sync divisor is 0 or 1, the PCG's frame sync clock generation unit is bypassed, and the frame sync input is connected directly to the frame sync out put. For PCG\_CTLB0.FSDIV =0, 1 the PCG\_PW1 -PCG\_PW4 registers function differently than in normal mode.

## Phase Shift

Phase shift is a frame sync parameter that defines the phase shift of the frame sync relative to the input clock of the same unit. This feature allows the shifting of the frame sync signal in time relative to the clock input signal. Frame sync phase shifting is often required by peripherals that need a frame sync signal to lead or lag a clock signal.

For example, the I 2 S protocol specifies that the frame sync transition from high-to-low occurs one clock cycle before the beginning of a frame. Since an I 2 S frame is 64 clock cycles long, delaying the frame sync by 63 cycles produces the required framing.

Phase shifting is represented as a full 20-bit value. Even when the frame sync is divided by the maximum amount, the phase can be shifted to the full range, from zero to one input clock short of the period.

- NOTE: Phase shifting is specified as a 2 x 10-bit divider value in the PCG\_CTLA0.FSPHASEHI bit field (bits 29:20) and in the PCG\_CTLA1.FSPHASELO bit field (bits 29:20).

A single 20-bit value spans these two-bit fields. The upper half of the word (bits 19:10) is in the PCG\_CTLA0 register, and the lower half (bits 9:0) is in the PCG\_CTLA1 register.

The phase shift between clock and frame sync outputs can be programmed using the PCG\_PW1 -PCG\_PW4 registers function registers and all of the control registers under these conditions:

- The input clock source for the clock generator output and the frame sync generator output is the same.
- The clock and frame sync are enabled at the same time using a single atomic instruction.
- The frame sync divisor is an integral multiple of the clock divisor.
- NOTE: When using a clock and frame sync as a synchronous pair, the units must be enabled in a single atomic instruction before their parameters are modified. Both units must also be disabled in a single atomic instruction.

NOTE: If the phase shift is 0 (see the Phase and Pulse Width Settings figure), the clock and frame sync outputs rise at the same time.

If the phase shift is 1, the frame sync output transitions one input clock period ahead of the clock transition.

If the phase shift is divisor - 1, the frame sync transitions divisor - 1 input clock periods ahead of the clock transitions.

Figure 31-2: Phase and Pulse Width Settings

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000001_2fb651f3d23a9e802f1a452a9f2db0db45d4d11c7ba9cdf28f409817a6cd959e.png)

NOTE: When generating single frame sync pulses (the length of one SPORT clock cycle), take care with respect to the drive and sampling edges. If the rules are violated, for example if the SPORT is not driving data, the module cannot detect a valid sample edge.

## Pulse Width

Pulse width is the number of input clock periods for which the frame sync output is high.

A 16-bit value determines the width of the framing pulse. Settings for pulse width can range from zero to DIV - 1. The pulse width should be less than the divisor of the frame sync. The pulse width of frame sync is specified in the PCG\_PW1.FSA , PCG\_PW1.FSB , PCG\_PW2.FSC , and PCG\_PW2.FSD bits (15-0) and (31-16). .

## Default Pulse Width

If the pulse width count is equal to 0 and if the PCG\_CTLA0.FSDIV / PCG\_CTLB0.FSDIV bit field is even, then the actual pulse width of the frame sync output is equal to:

For even divisors: frame sync divisor/2

If the pulse width count is equal to 0 and if the PCG\_CTLA0.FSDIV / PCG\_CTLB0.FSDIV bit field is odd, then the actual pulse width of the frame sync output is equal to:

For odd divisors: frame sync divisor - 1/2

## Input Clock Source Considerations

The core Phase-Locked Loop (PLL) has been designed to provide clocking for the processor core. The performance specifications of this PLL are appropriate for the core. But they have not been optimized or specified for precision data converters where jitter directly translates into time quantization errors and distortion.

The PCG allows the routing of external clock sources which are independent of the core PLL.

## Timing Example for I 2 S Mode

For I 2 S mode, the frame sync should be driven at the falling edge of SPORT clock. In other words, the frame sync edge must coincide with the falling edge of the SPORT clock. To satisfy this requirement, program the phase of the frame sync accordingly in the PCG control registers.

For example, assume that the input clock source for both clock and frame sync are the same and both the clock and frame sync are enabled at the same time. Also, assume that the clock divisor value for generating the required SPORT clock is PCG\_CTLA1.CLKDIV = 4. Then, for a 32-bit word length, the frame sync divisor value is:

```
PCG_CTLA0.FSDIV = 256, PCG_CTLA1.CLKDIV = 4.
```

By default, for phase = 0, the rising edge of both SPORT clock and frame sync coincide. To make sure that the frame sync edges coincide with the falling edge of the SPORT clock, program the phase value as:

PCG\_CTLA1.CLKDIV /2 = 2.

## Cross Mode Connections

The symmetric dual DAI architecture allows cross connections between both PCGs (A,B) and (C,D) to the other DAI. Each PCG (A through D) supports an alternative input clock (PCG0\_EXTx\_I)(see Figure 31-1 PCG Block Diagram) which can be sourced via a DAI pin buffer from the other DAI. Note however if routing a source (clock or FS) only DAI pin buffer 2 to 20 can be used (DAI pin buffer 1 is no longer available and is replaced by the DAI CRS buffer for the other DAI).

See DAI Routing Capabilities for more information.

## Operating Modes

The following sections provide information on the operating modes of the precision clock generator.

## Normal Mode

When the frame sync divisor is set to any value other than zero or one, the PCGs operate in normal mode. In normal mode, the divisor determines the frequency of the frame sync output where:

Frequency of Frame Sync Output = Input Frequency/Divisor

The value of the pulse width control determines the high period of the frame sync output. The value of the pulse width control must be less than the value of the divisor.

The value of the phase control determines the phase of the frame sync output. When the phase is zero, then the positive edges of the clock and frame sync coincide when:

- the clock and frame sync dividers are enabled at the same time using an atomic instruction
- the divisors of the clock and frame sync are the same
- the source for the clock and frame sync is the same

The number of input clock cycles that have already elapsed before the frame sync is enabled is equal to the difference between the divisor and the phase values. When the phase is a small fraction of the divisor, then the frame sync leads the clock. When the phase is only slightly less than the frame sync divisor, then the frame sync lags the clock. The frame sync phase must not be greater than the divisor.

## Bypass Mode

When the frame sync divisor for the frame sync has a value of zero or one, the frame sync is in bypass mode, and the PCG\_PW1 -PCG\_PW4 registers are registers have different functionality than in normal mode.

NOTE: In normal mode, bits 15:0 and 31:16 of the PCG\_PW1 -PCG\_PW4 registers are used to program the pulse width count. In bypass mode, bits 15:2 and 31:18 are ignored. Bits 1:0 and 17:16 are renamed to PCG\_PW1.STROBEA and PCG\_PW1.INVFSA , respectively. This functionality is described in more detail as follows.

If the PCG\_PW1.STROBEA register is cleared, then the input is directly passed (see the Bypass and Inverted Bypass figure) to the frame sync output either inverted or not inverted, depending on the PCG\_PW1.INVFSA bits.

Figure 31-3: Bypass and Inverted Bypass

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000002_3b5c56a4c253fd887c87a78c3472828bdee6f9d73795551072c1991ae65b0277.png)

## One-Shot Mode

In the one-shot mode operation shown in the One Shot Mode PCG A (MISCA2\_I Input) figure, the PCG produces a series of periods but does not run continuously.

Figure 31-4: One Shot Mode PCG A (MISCA2\_I Input)

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000003_3795e9d2e20e363d199f6d45d4246784fe78353cc7e556d7c155b6ae33bb871c.png)

Bypass mode also enables the generation of a strobe pulse (one-shot frame sync). Strobe usage ignores the divider counters and looks to the SRU to provide the input signal.

In the bypass mode, if the PCG\_PW1.STROBEA bit =1, then a one-shot pulse is generated. This one-shot pulse has the duration equal to the period of DAI\_MISCAx\_I for the PCGx unit. This pulse is generated either at the falling or rising edge of the input clock, depending on the value of the PCG\_PW1.INVFSA bit. The output pulse width is equal to the period of the SRU source signal DAI\_MISCAx\_I . The pulse begins at the second rising edge of MISCAx\_I following a rising edge of the clock input. When the PCG\_PW1.INVFSA bit is set, the pulse begins at the second rising edge of DAI\_MISCAx\_I coinciding with or following a falling edge of the clock input.

NOTE: A strobe period is defined to be the period of the FS input clock signal as specified by the PCG\_CTLA1.FSSRC bit.

## Audio System Example

The PCG Setup for I2S or Left-Justified DAI figure shows an example of the interconnections between the S/PDIF receiver, ASRC, and the PCGs. The interconnections are made by programming the signal routing unit. It shows how to set up two precision clock generators using the S/PDIF receiver and an asynchronous sample rate converter (ASRC) to interface to an external audio DAC. The PCG is configured to provide a fixed ASRC/DAC output sample rate of 65.098 kHz. The input to the S/PDIF receiver is typically 44.1 kHz if supplied by a CD player, but can also be from other source at any nominal sample rates.

Similarly, the phase shift for frame syncs B, C, and D is specified in the corresponding control registers ( PCG\_CTLA0 through PCG\_CTLB1 ).

Figure 31-5: PCG Setup for I 2 S or Left-Justified DAI

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000004_1c41d614b04355f19a98e3c67a2d5a3ef51500f630b1560f5b7fe63614973bc6.png)

Three synchronous clocks are required in audio systems:

1. Frame sync (FS)
2. Serial bit clock (64 FS)
3. Controller DAC clock (256 FS)

Since each PCG has only two outputs, this example requires two PCGs. Furthermore, because the digital audio interface requires a fixed-phase relation between serial clock and FS, these two outputs should come from one PCG (PCG A). The controller clock comes from the second (PCG B).

The combined PCGs can provide a selection of synchronous clock frequencies to support alternate sample rates for the ASRCs and external DACs. However, the range of choices is limited by CLKIN and the ratio of PCG\_CLKx\_O : serial clock: FS . The ratio is normally fixed at 256:64:1 to support digital audio left-justified, I 2 S and right-justified interface modes.

Many DACs also support 384, 512, and 786x FS for PCG\_CLKx\_O , which allows some additional flexibility in choosing serial clock.

Note that the falling edge of serial clock must always be synchronous with both edges of FS. This condition requires that the phase of the serial clock and FS signals for a common PCG (PCG A) be adjustable.

While the frequency of the controller DAC clock ( PCG\_CLKx\_O ) must be synchronous with the sample rate supplied to the external DAC, there is no fixed-phase requirement.

Configure the clock divisor and source and low-phase word first, followed by the control register enable bits, which are set together. When the PCG\_PW1 -PCG\_PW4 registers are configured to zero (default), the FS pulse width is (divisor /2) for even divisors and (divisor-1)/2 for odd divisors. Alternatively, the PCG\_PW1 -PCG\_PW4 registers can be set high for exactly one-half the period of CLKIN cycles for a 50% duty cycle, provided the FS divisor is an even number.

## Hardware Trigger Control

In addition to the existing enable control to generate the clock and frame sync, this feature provides a flexibility to start the clock and frame sync generation upon receiving a hardware trigger (internal completer trigger). This feature applies to all modes of operation.

For example, when the PCG\_SYNC1.HWB\_TRIGEN bit is set, it enables the synchronization of clock A and frame sync A to the external LRCLK sync (in external LRCLK synchronization mode) upon receiving a hardware trigger. Refer to the PCG\_SYNC1.HWA\_TRIGEN / PCG\_SYNC1.HWB\_TRIGEN and PCG\_SYNC2.HWC\_TRIGEN / PCG\_SYNC2.HWD\_TRIGEN bit descriptions.

NOTE: To exercise the hardware trigger based synchronization, program the destination registers ( DAI\_MISC2.IN0 , DAI\_MISC2.IN1 DAI\_MISC2.IN2 , and DAI\_MISC2.IN3 ) to route the trigger signals to the PCGs . For example, if PCGA must be enabled for the hardware trigger based synchronization, program the DAI\_MISC2.IN0 and DAI\_EXTD\_MISC2.IN0 bit fields to 5'b00000 and 2'b01 , respectively. Each PCG has a dedicated trigger control from the TRU unit, there are four trigger controls for DAI0 and four trigger controls for DAI1.

Refer to TRU Completer Trigger list table for the completer trigger information.

## Clock Configuration Examples

For a CLKIN = 33.330 MHz, the two PCGs provide the three synchronous clocks PCGx\_CLK, serial clock and FS for the SRCs and external DAC. These divisors are stored in the PCG\_CTLA1.CLKDIV / PCG\_CTLB1.CLKDIV bit fields.

The integer divisors for several sample rates based on 33.330 MHz CLKIN are shown in the Precision Clock Generator Division Ratios table.

Table 31-4: Precision Clock Generator Division Ratios (33.330 CLKIN)

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

## Global PCG Enable

For some applications, enabling multiple PCGs (CLK and FS) must be synchronized. This feature provides the flexibility to enable multiple PCGs (CLK and FS) globally. The global PCG enable feature works independently, while the legacy PCG enable (CLK and FS) remains the same. The legacy PCG enable can be used when the PCGs need to be enabled individually.

The DAI\_GBL\_PCG\_EN register controls the enabling and disabling of PCGs globally. The global enable feature can be used in the following scenarios:

- to enable all the eight PCGs (includes both DAI0 and DAI1) at the same time
- to enable all four PCGs within each DAIx at the same time
- to enable multiple PCGs (across DAIs) at the same time

NOTE: See the DAI\_GBL\_PCG\_EN register description for details.

## Programming Guidelines

Complete the following steps to globally enable all of the PCGs :

1. Clear the individual (local) PCG enable bits from PCG\_CTL x registers. (for example, PCG\_CTLA0.CLKEN and PCG\_CTLA0.FSEN bits) without modifying any other settings.
2. Wait for n CCLK cycles; n = PCG source clock period/processor clock period.
3. Program the PCG registers ( PCG\_CTL x/ PCG\_SYNC1 ) with the desired configuration without setting the PCG enable bits.
4. Enable all the required PCGs in the DAI\_GBL\_PCG\_EN register that need to be enabled globally.

Complete the following steps to globally disable the PCGs using the DAI\_GBL\_PCG\_EN register:

1. Disable individual PCG bits to disable a particular PCG.
2. Disable DAI0/1 enable bits to disable PCGs within the DAI.
3. Disable the global PCG enable bit in cases where all PCGs must be disabled.

## PCG Event Control

The following sections describe the generation and control of PCG events.

## External Event Trigger

The trigger with the external clock is enabled by setting bits 0 and 16 of the PCG\_SYNC1 / PCG\_SYNC2 / PCG\_SYNC3 / PCG\_SYNC4 registers .

Refer to the FS Output Synchronization With External Trigger Input figure. Since the rising edge of the external clock is used to synchronize with the frame sync, the frame sync output is not generated until a rising edge of the external clock is sensed.

Figure 31-6: FS Output Synchronization With External Trigger Input

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000005_c782fea891cd5d2df88ef8d3e83fb1a8f204a65e30eb6f0b8ed9bc31a465b132.png)

## External Event Trigger Delay

The time delay between the rising trigger edge and the start of serial clock and frame sync varies between 2.5 to 3.5 input clock periods. If the input clock and the trigger signal are synchronous, the delay is 3 input clock periods. Consider the following cases:

- SCLK0 is the input source. In this case, if the given trigger event is synchronous to SCLK0, the delay is 3 SCLK0 periods. If the trigger signal is asynchronous with SCLK0, the delay varies from 2.5 SCLK0 periods to 3.5 SCLK0 periods. (It depends on whether the trigger edge occurs in the positive half cycle or negative half cycle of SCLK0.)
- The CLKIN source bits of the PCG\_SYNC1 register select the input source for PCG. In this case, if the given trigger signal is synchronous to CLKIN, the delay is 3 CLKIN periods. But if they are asynchronous to CLKIN, the delay can vary between 2.5 CLKIN periods to 3.5 CLKIN periods.
- SRU DAI0 is the input source for PCG A, B. If the input clock and trigger signal are synchronous, the delay is exactly 3 input clock periods. If asynchronous, it varies between 2.5 to 3.5 input clock periods depending on the phase difference between the input clock and trigger signal.

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

1. Program the PCG\_SYNC1 -PCG\_SYNC4 registers and the PCG\_CTLA0 through PCG\_CTLB1 registers appropriately.
2. Enable the clock or frame sync, or both.

Since the rising edge of the external clock is used to synchronize with the frame sync, the frame sync output is not generated until a rising edge of the external clock is sensed.

## Debug Features

Take care in cases where any input to the phase unit is modified. Any individual change of the PCG\_CTLA1.CLKDIV or PCG\_CTLA0.FSDIV dividers can cause a failure in the PCG sync operation between the serial clock and the frame sync. Only the programming model ensures a correct setup for phase settings.

## ADSP-2184x PCG Register Descriptions

Precision Clock Generator (PCG) contains the following registers.

Table 31-5: ADSP-2184x PCG Register List

| Name      | Description                          |
|-----------|--------------------------------------|
| PCG_CTLA0 | Precision Clock A Control 0 Register |
| PCG_CTLA1 | Precision Clock A Control 1 Register |
| PCG_CTLB0 | Precision Clock B Control 0 Register |
| PCG_CTLB1 | Precision Clock B Control 1 Register |
| PCG_CTLC0 | Precision Clock C Control 0 Register |
| PCG_CTLC1 | Precision Clock C Control 1 Register |

Table 31-5: ADSP-2184x PCG Register List (Continued)

| Name      | Description                                           |
|-----------|-------------------------------------------------------|
| PCG_CTLD0 | Precision Clock DControl 0 Register                   |
| PCG_CTLD1 | Precision Clock DControl 1 Register                   |
| PCG_CTLE0 | Precision Clock E Control 0 Register                  |
| PCG_CTLE1 | Precision Clock E Control 1 Register                  |
| PCG_CTLF0 | Precision Clock F Control 0 Register                  |
| PCG_CTLF1 | Precision Clock F Control 1 Register                  |
| PCG_CTLG0 | Precision Clock G Control 0 Register                  |
| PCG_CTLG1 | Precision Clock G Control 1 Register                  |
| PCG_CTLH0 | Precision Clock HControl 0 Register                   |
| PCG_CTLH1 | Precision Clock HControl 1 Register                   |
| PCG_PW1   | Precision Clock Pulse Width Control 1 Register        |
| PCG_PW2   | Precision Clock Pulse Width Control 2 Register        |
| PCG_PW3   | Precision Clock Pulse Width Control 3 Register        |
| PCG_PW4   | Precision Clock Pulse Width Control 4 Register        |
| PCG_SYNC1 | Precision Clock Frame Sync Synchronization 1 Register |
| PCG_SYNC2 | Precision Clock Frame Sync Synchronization 2 Register |
| PCG_SYNC3 | Precision Clock Frame Sync Synchronization 3 Register |
| PCG_SYNC4 | Precision Clock Frame Sync Synchronization 4 Register |

## Precision Clock A Control 0 Register

The PCG\_CTLA0 register enables the clock, frame sync, and select divisor for the PCG0 clock A signal.

Figure 31-7: PCG\_CTLA0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000006_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-6: PCG\_CTLA0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLA0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLA0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLA0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLA0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLA0.FSDIV >1 PCGx is in normal mode, PCG_CTLA0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock A Control 1 Register

The PCG\_CTLA1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock A signal.

Figure 31-8: PCG\_CTLA1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000007_7a2c2e96c5fed5e9341a9ba1039cf61cbeed00d4767d788598131f2fa9b4ef39.png)

Table 31-7: PCG\_CTLA1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLA1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock 1 PCG_EXT_DAI0 selected for clock                |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLA1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLA1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLA1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock B Control 0 Register

The PCG\_CTLB0 register enables the clock, frame sync, and select divisor for the PCG0 clock B signal.

Figure 31-9: PCG\_CTLB0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000008_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-8: PCG\_CTLB0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLB0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLB0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLB0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLB0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLB0.FSDIV >1 PCGx is in normal mode, PCG_CTLB0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock B Control 1 Register

The PCG\_CTLB1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock B signal.

Figure 31-10: PCG\_CTLB1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000009_7a2c2e96c5fed5e9341a9ba1039cf61cbeed00d4767d788598131f2fa9b4ef39.png)

Table 31-9: PCG\_CTLB1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLB1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock 1 PCG_EXT_DAI0 selected for clock                |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLB1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLB1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLB1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock C Control 0 Register

The PCG\_CTLC0 register enables the clock, frame sync, and select divisor for the PCG0 clock C signal.

Figure 31-11: PCG\_CTLC0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000010_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-10: PCG\_CTLC0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLC0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLC0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLC0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLC0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLC0.FSDIV >1 PCGx is in normal mode, PCG_CTLC0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock C Control 1 Register

The PCG\_CTLC1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock C signal.

Figure 31-12: PCG\_CTLC1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000011_03f58ef734d56d8a3308c33e5fa79c6edcb52af121e01de2323c407503bdc598.png)

Table 31-11: PCG\_CTLC1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLC1.CLKSRC bit specifies the clock source. 0 CLKIN1 pin selected for clock 1 PCG_EXT_DAI1 selected for clock                |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLC1.FSSRC bit specifies the frame sync source. 0 CLKIN1 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLC1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLC1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock D Control 0 Register

The PCG\_CTLD0 register enables the clock, frame sync, and select divisor for the PCG0 clock D signal.

Figure 31-13: PCG\_CTLD0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000012_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-12: PCG\_CTLD0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLD0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLD0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLD0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLD0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLD0.FSDIV >1 PCGx is in normal mode, PCG_CTLD0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock D Control 1 Register

The PCG\_CTLD1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock D signal.

Figure 31-14: PCG\_CTLD1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000013_03f58ef734d56d8a3308c33e5fa79c6edcb52af121e01de2323c407503bdc598.png)

Table 31-13: PCG\_CTLD1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLD1.CLKSRC bit specifies the clock source. 0 CLKIN1 pin selected for clock 1 PCG_EXT_DAI1 selected for clock                |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLD1.FSSRC bit specifies the frame sync source. 0 CLKIN1 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLD1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLD1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock E Control 0 Register

The PCG\_CTLE0 register enables the clock, frame sync, and select divisor for the PCG0 clock E signal.

Figure 31-15: PCG\_CTLE0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000014_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-14: PCG\_CTLE0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLE0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLE0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLE0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLE0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLE0.FSDIV >1 PCGx is in normal mode, PCG_CTLE0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock E Control 1 Register

The PCG\_CTLE1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock E signal.

Figure 31-16: PCG\_CTLE1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000015_7a2c2e96c5fed5e9341a9ba1039cf61cbeed00d4767d788598131f2fa9b4ef39.png)

Table 31-15: PCG\_CTLE1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLE1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock 1 PCG_EXT_DAI selected for clock                 |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLE1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLE1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLE1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock F Control 0 Register

The PCG\_CTLF0 register enables the clock, frame sync, and select divisor for the PCG0 clock F signal.

Figure 31-17: PCG\_CTLF0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000016_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-16: PCG\_CTLF0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLF0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLF0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLF0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLF0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLF0.FSDIV >1 PCGx is in normal mode, PCG_CTLF0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock F Control 1 Register

The PCG\_CTLF1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock F signal.

Figure 31-18: PCG\_CTLF1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000017_7a2c2e96c5fed5e9341a9ba1039cf61cbeed00d4767d788598131f2fa9b4ef39.png)

Table 31-17: PCG\_CTLF1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLF1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock 1 PCG_EXT_DAI0 selected for clock                |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLF1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLF1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLF1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock G Control 0 Register

The PCG\_CTLG0 register enables the clock, frame sync, and select divisor for the PCG0 clock G signal.

Figure 31-19: PCG\_CTLG0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000018_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-18: PCG\_CTLG0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLG0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLG0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLG0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLG0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLG0.FSDIV >1 PCGx is in normal mode, PCG_CTLG0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock G Control 1 Register

The PCG\_CTLG1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock G signal.

Figure 31-20: PCG\_CTLG1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000019_03f58ef734d56d8a3308c33e5fa79c6edcb52af121e01de2323c407503bdc598.png)

Table 31-19: PCG\_CTLG1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLG1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock 1 PCG_EXT_DAI selected for clock                 |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLG1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLG1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLG1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock H Control 0 Register

The PCG\_CTLH0 register enables the clock, frame sync, and select divisor for the PCG0 clock H signal.

Figure 31-21: PCG\_CTLH0 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000020_ac99279b147aac42c53cb8eab989de3f445496609b68046eae06a497881a9fa2.png)

Table 31-20: PCG\_CTLH0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKEN      | Clock Enable. The PCG_CTLH0.CLKEN bit enables the clock.                                                                                                                                                                                                              |
| 30 (R/W)           | FSEN       | Frame Sync Enable. The PCG_CTLH0.FSEN bit enables the frame sync.                                                                                                                                                                                                     |
| 29:20 (R/W)        | FSPHASEHI  | Phase for Frame Sync High. The PCG_CTLH0.FSPHASEHI bit field represents the upper half of the 20-bit value for the channel A/B/C/D frame sync phase.                                                                                                                  |
| 19:0 (R/W)         | FSDIV      | Frame Sync Divider. The PCG_CTLH0.FSDIV bit field provides the frame sync divider value. This 20-bit field frame sync divider is multiplexed where: PCG_CTLH0.FSDIV >1 PCGx is in normal mode, PCG_CTLH0.FSDIV = 0, 1 PCGx is in bypass mode. 0 PCG is in bypass mode |

## Precision Clock H Control 1 Register

The PCG\_CTLH1 register sets the clock divisor, frame sync source, and clock source for the PCG1 clock H signal.

Figure 31-22: PCG\_CTLH1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000021_03f58ef734d56d8a3308c33e5fa79c6edcb52af121e01de2323c407503bdc598.png)

Table 31-21: PCG\_CTLH1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CLKSRC     | Clock Source. The PCG_CTLH1.CLKSRC bit specifies the clock source. 0 CLKIN0 pin selected for clock 1 PCG_EXT_DAI selected for clock                 |
| 30 (R/W)           | FSSRC      | Frame Sync Source. The PCG_CTLH1.FSSRC bit specifies the frame sync source. 0 CLKIN0 pin selected for frame sync                                    |
| 29:20 (R/W)        | FSPHASELO  | Phase for Frame Sync Low. The PCG_CTLH1.FSPHASELO bit field represents the lower half of the 20-bit value for the channel A/B/C/D frame sync phase. |
| 19:0 (R/W)         | CLKDIV     | Clock Divisor. The PCG_CTLH1.CLKDIV bit field contains the clock divisor value.                                                                     |

## Precision Clock Pulse Width Control 1 Register

The PCG\_PW1 register sets the one shot frame sync and the active low frame sync select for PCG A and PCG B.

Figure 31-23: PCG\_PW1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000022_6d6f4238bdc47d4a84cae7e0b5fddb603f76b0c691f223899dcfc46989943796.png)

Table 31-22: PCG\_PW1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | INVFSB     | Active Low Frame Sync Select for Frame in Bypass PCG B. The PCG_PW1.INVFSB bit selects active low or active high frame sync in bypass mode for PCG B.                                                                                     |
| 16 (R/W)           | STROBEB    | One Shot Frame Sync Bypass Mode PCG B. The PCG_PW1.STROBEB bit sets the frame sync pulse in bypass mode for PCG B. This is the duration equal to one period of the DAI_MISCA3_I signal (PCG B) repeating at the beginning of every frame. |
| 31:16 (R/W)        | FSB        | Pulse Width for Frame Sync PCG B. The PCG_PW1.FSB bit field sets the number of input clock periods for which the frame sync output is high. Pulse width should be less than the divisor of the frame sync.                                |
| 1 (R/W)            | INVFSA     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW1.INVFSA bit selects active low or active high frame sync for PCG A in bypass mode.                                                                                 |
| 15:0 (R/W)         | FSA        | Pulse Width for Frame Sync PCG A. The PCG_PW1.FSA bit field sets the number of input clock periods for which the frame sync output is high. Pulse width should be less than the divisor of the frame sync.                                |

Table 31-22: PCG\_PW1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | STROBEA    | One Shot Frame Sync PCG A. The PCG_PW1.STROBEA bit sets the frame sync pulse for PCG A in bypass mode. This is the duration equal to one period of the DAI_MISCA2_I signal (PCG A) repeating at the beginning of every frame. |

## Precision Clock Pulse Width Control 2 Register

The PCG\_PW2 register sets the one shot frame sync and the active low frame sync select for PCG C and PCG D.

Figure 31-24: PCG\_PW2 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000023_e3f6413724b06e4a7952da56d9804b9144eb5188730f134c5351209592d9f6b5.png)

Table 31-23: PCG\_PW2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | INVFSD     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW2.INVFSD bit selects active low or active high frame sync for PCGD in bypass mode.                                                                     |
| 16 (R/W)           | STROBED    | One Shot Frame Sync PCG D. The PCG_PW2.STROBED bit sets the frame sync pulse for PCG Din bypass mode. This is the duration equal to one period of the DAI_MISCA3_I signal (PCG D) repeating at the beginning of every frame. |
| 31:16 (R/W)        | FSD        | Pulse Width for Frame Sync PCG D. The PCG_PW2.FSD bit field sets the number of input clock periods for which the frame sync output is high for PCG D. Pulse width should be less than the divisor of the frame sync.         |
| 1 (R/W)            | INVFSC     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW2.INVFSC bit selects active low or active high frame sync for PCG C in bypass mode.                                                                    |
| 15:0 (R/W)         | FSC        | Pulse Width for Frame Sync PCG C. The PCG_PW2.FSC bit field sets the number of input clock periods for which the frame sync output is high for PCG C. Pulse width should be less than the divisor of the frame sync.         |

Table 31-23: PCG\_PW2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | STROBEC    | One Shot Frame Sync PCG C. The PCG_PW2.STROBEC bit sets the frame sync pulse for PCG C in bypass mode. This is the duration equal to one period of the DAI_MISCA2_I signal (PCG C) repeating at the beginning of every frame. |

## Precision Clock Pulse Width Control 3 Register

The PCG\_PW3 register sets the one shot frame sync and the active low frame sync select for PCG E and PCG F .

Figure 31-25: PCG\_PW3 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000024_1f5e7e635a428d7c2fe6875c54aa49af1cb4ffaa10a6728bfb9e670d703652f7.png)

Table 31-24: PCG\_PW3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | INVFSF     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW3.INVFSF bit selects active low or active high frame sync for PCG F in bypass mode.                                                                     |
| 16 (R/W)           | STROBEF    | One Shot Frame Sync PCG D. The PCG_PW3.STROBEF bit sets the frame sync pulse for PCG F in bypass mode. This is the duration equal to one period of the DAI_MISCA1_I signal (PCG F) repeating at the beginning of every frame. |
| 31:16 (R/W)        | FSF        | Pulse Width for Frame Sync PCG F. The PCG_PW3.FSF bit field sets the number of input clock periods for which the frame sync output is high for PCG F. Pulse width should be less than the divisor of the frame sync.          |
| 1 (R/W)            | INVFSE     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW3.INVFSE bit selects active low or active high frame sync for PCG E in bypass mode.                                                                     |
| 0 (R/W)            | STROBEE    | One Shot Frame Sync PCG E. The PCG_PW3.STROBEE bit sets the frame sync pulse for PCG E in bypass mode. This is the duration equal to one period of the DAI_MISCA0_I signal (PCG E) repeating at the beginning of every frame. |

Table 31-24: PCG\_PW3 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | FSE        | Pulse Width for Frame Sync PCG E. The PCG_PW3.FSE bit field sets the number of input clock periods for which the frame sync output is high for PCG E. Pulse width should be less than the divisor of the frame sync. |

## Precision Clock Pulse Width Control 4 Register

The PCG\_PW4 register sets the one shot frame sync and the active low frame sync select for PCG G and PCG H.

Figure 31-26: PCG\_PW4 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000025_24b6602569290e36c4ed7bc3e504471e6bde222f44a0abe028e1f0cffcacb35c.png)

Table 31-25: PCG\_PW4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | INVFSH     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW4.INVFSH bit selects active low or active high frame sync forPCGH in bypass mode.                                                                      |
| 16 (R/W)           | STROBEH    | One Shot Frame Sync PCG H. The PCG_PW4.STROBEH bit sets the frame sync pulse for PCG Hin bypass mode. This is the duration equal to one period of the DAI_MISCA1_I signal (PCG H) repeating at the beginning of every frame. |
| 31:16 (R/W)        | FSH        | Pulse Width for Frame Sync PCG H. The PCG_PW4.FSH bit field sets the number of input clock periods for which the frame sync output is high for PCG H. Pulse width should be less than the divisor of the frame sync.         |
| 1 (R/W)            | INVFSG     | Active Low Frame Sync Select for Frame Sync in Bypass Mode. The PCG_PW4.INVFSG bit selects active low or active high frame sync for PCG G in bypass mode.                                                                    |
| 15:0 (R/W)         | FSG        | Pulse Width for Frame Sync PCG G. The PCG_PW4.FSG bit field sets the number of input clock periods for which the frame sync output is high for PCG G. Pulse width should be less than the divisor of the frame sync.         |

Table 31-25: PCG\_PW4 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | STROBEG    | One Shot Frame Sync PCG G. The PCG_PW4.STROBEG bit sets the frame sync pulse for PCG G in bypass mode. This is the duration equal to one period of the DAI_MISCA0_I signal (PCG G) repeating at the beginning of every frame. |

## Precision Clock Frame Sync Synchronization 1 Register

The PCG\_SYNC1 register allows programs to synchronize the clock frame sync units with external frame syncs. Note that the PCG\_CTLA1.CLKSRC bit is overridden if PCG\_SYNC1.CLKASRC bit in the PCG\_SYNC1 register is set.

Figure 31-27: PCG\_SYNC1 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000026_2f3ddbd281a4b1d5336d544b39b05e3471644fee6ae6f715b132c5f5a0fa8ee4.png)

Table 31-26: PCG\_SYNC1 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                           |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | FSB_FP1CLKSEL  | Frame Sync B FracNPll 1 Clock Select. The PCG_SYNC1.FSB_FP1CLKSEL bit when enabled, selects FracNPll 1 out- put clock input clock source to FSB. This feature is enabled only when PCG_BKP_CTLB1.FSSRC bit is set and FSB_FPCLKSEL bit of this register is set.   |
| 29 (R/W)           | FSB_FP1CLKSEL  | 0 PCG_EXTX_DAI0 selected for frame sync                                                                                                                                                                                                                           |
| 28 (R/W)           | CLKB_FP1CLKSEL | Clock B FracNPll 1 Clock Select. The PCG_SYNC1.CLKB_FP1CLKSEL bit when enabled, selects FracNPll 1 out- put clock as input clock source to CLKB. This feature is enabled only when PCG_BKP_CTLB1.CLKSRC bit is set and CLKB_FPCLKSEL bit of this register is set. |
| 28 (R/W)           | CLKB_FP1CLKSEL | 0 PCG_EXT_DAI0 selected for clock                                                                                                                                                                                                                                 |
| 27 (R/W)           | FSB_FPCLKSEL   | 1 FracNPll 1 output clock selected for clock Frame Sync B FracNPll Clock Select. The PCG_SYNC1.FSB_FPCLKSEL bit when enabled, selects FracNPll out- put clock input clock source to FSB. This feature is enabled only when PCG_BKP_CTLB1.FSSRC bit is set.        |
| 27 (R/W)           | FSB_FPCLKSEL   | 0 PCG_EXTX_DAI0 selected for frame sync                                                                                                                                                                                                                           |
| 27 (R/W)           | FSB_FPCLKSEL   | 1 FracNPll output clock selected for frame sync                                                                                                                                                                                                                   |
| 26 (R/W)           | CLKB_FPCLKSEL  | The PCG_SYNC1.CLKB_FPCLKSEL bit when enabled, selects FracNPll out- put clock as input clock source to CLKB. This feature is enabled only when PCG_BKP_CTLB1.CLKSRC bit is set.                                                                                   |
| 26 (R/W)           | CLKB_FPCLKSEL  | 0 PCG_EXT_DAI0 selected for clock                                                                                                                                                                                                                                 |
| 22 (R/W)           | HWB_TRIGEN     | Hardware Trigger Sync B Enable. The PCG_SYNC1.HWB_TRIGEN bit enables synchronization of clock B and frame sync B with the external frame sync only after an internal receiver trigger for PCG B is received. CLKIN Source for FSB.                                |
| 21 (R/W)           | FSB_CLKINSEL   | The PCG_SYNC1.FSB_CLKINSEL bit enables the CLKIN input source for FSB.                                                                                                                                                                                            |
| 21 (R/W)           | FSB_CLKINSEL   | 0 CLKIN0                                                                                                                                                                                                                                                          |
| 21 (R/W)           | FSB_CLKINSEL   | 1 CLKIN1                                                                                                                                                                                                                                                          |

Table 31-26: PCG\_SYNC1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                                                 |
|--------------------|---------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | CLKB_CLKINSEL | CLKIN Source for CLKB. The PCG_SYNC1.CLKB_CLKINSEL bit enables the CLKIN input source for CLKB.                                                                                                                                                                                                         |
| 19 (R/W)           | FSBSRC        | Frame Sync B Source. The PCG_SYNC1.FSBSRC bit enables the frame sync B input source. 0 Output selected by FSBSOURCE bit                                                                                                                                                                                 |
| 18 (R/W)           | CLKBSRC       | Clock B Source. The PCG_SYNC1.CLKBSRC bit enables the clock B input source. 0 Output selected by CLKBSOURCE bit                                                                                                                                                                                         |
| 17 (R/W)           | CLKB          | 1 Clock derived from core PLL selected for clock B Clock B Enable. The PCG_SYNC1.CLKB bit enables synchronization of clock B with the external frame sync. 0 Clock disabled                                                                                                                             |
| 16 (R/W)           | FSB           | 1 Clock enabled Frame Sync B Enable. The PCG_SYNC1.FSB bit enables synchronization of frame sync B with the external frame sync.                                                                                                                                                                        |
| 13 (R/W)           |               | 0 Frame sync disabled 1 Frame sync enabled                                                                                                                                                                                                                                                              |
|                    | FSA_FP1CLKSEL | Frame Sync A FracNPll 1 Clock Select. The PCG_SYNC1.FSA_FP1CLKSEL bit when enabled, selects FracNPll 1 out- put clock input clock source to FSA. This feature is enabled only when PCG_BKP_CTLA1.FSSRC bit is set and FSA_FPCLKSEL bit of this register is set. 0 PCG_EXTX_DAI0 selected for frame sync |

Table 31-26: PCG\_SYNC1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                           |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | CLKA_FP1CLKSEL | Clock A FracNPll 1 Clock Select. The PCG_SYNC1.CLKA_FP1CLKSEL bit when enabled, selects FracNPll 1 out- put clock as input clock source to CLKA. This feature is enabled only when PCG_BKP_CTLA1.CLKSRC bit is set and CLKA_FPCLKSEL bit of this register is set. |
| 11 (R/W)           | FSA_FPCLKSEL   | Frame Sync A FracNPll Clock Select. The PCG_SYNC1.FSA_FPCLKSEL bit when enabled, selects FracNPll out- put clock input clock source to FSA. This feature is enabled only when PCG_BKP_CTLA1.FSSRC bit is set.                                                     |
| 10 (R/W)           | CLKA_FPCLKSEL  | Clock A FracNPll Clock Select. The PCG_SYNC1.CLKA_FPCLKSEL bit when enabled, selects FracNPll out- put clock as input clock source to CLKA. This feature is enabled only when                                                                                     |
| 6                  |                | PCG_BKP_CTLA1.CLKSRC bit is set. 0 PCG_EXT_DAI0 selected for clock 1 FracNPll output clock selected for clock                                                                                                                                                     |
| (R/W)              | HWA_TRIGEN     | Hardware Trigger Sync A Enable. The PCG_SYNC1.HWA_TRIGEN bit enables synchronization of clock A and frame sync A with the external frame sync only after an internal receiver trigger for PCG A is received.                                                      |
| 5 (R/W)            | FSA_CLKINSEL   | CLKIN Source for FSA. The PCG_SYNC1.FSA_CLKINSEL bit enables the CLKIN input source for FSA. 0 CLKIN0 1 CLKIN1                                                                                                                                                    |
| 4 (R/W)            | CLKA_CLKINSEL  | CLKIN source for CLKA. The PCG_SYNC1.CLKA_CLKINSEL bit enables the CLKIN input source for CLKA                                                                                                                                                                    |

Table 31-26: PCG\_SYNC1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | FSASRC     | Frame Sync Source. The PCG_SYNC1.FSASRC bit enables the frame sync A input source.                                                                         |
| 2 (R/W)            | CLKASRC    | Clock A Source. The PCG_SYNC1.CLKASRC bit enables the clock A input source. 0 Output selected by CLKASOURCE bit                                            |
| 1 (R/W)            | CLKA       | 1 Clock derived from core PLL selected for clock A Clock A Enable. The PCG_SYNC1.CLKA bit enables synchronization of clock A with the external frame sync. |
| 0 (R/W)            | FSA        | Frame Sync A Enable. The PCG_SYNC1.FSA bit enables synchronization of frame sync A with the external frame sync. 0 Frame Sync Disabled                     |

## Precision Clock Frame Sync Synchronization 2 Register

The PCG\_SYNC2 register allows programs to synchronize the clock frame sync units with external frame syncs. Note that the PCG\_CTLD1.CLKSRC bit is overridden if PCG\_SYNC2.CLKDSRC bit in the PCG\_SYNC2 register is set.

Figure 31-28: PCG\_SYNC2 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000027_0f17d45ef980d6f32dd816c4a3024709de044acaa7274417d6db4146f88e863b.png)

Table 31-27: PCG\_SYNC2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                  |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/W)           | HWD_TRIGEN   | Hardware Trigger Sync DEnable. The PCG_SYNC2.HWD_TRIGEN bit enables synchronization of clock Dand frame sync Dwith the external frame sync only after an internal receiver trigger for PCGD is received. |
| 21 (R/W)           | FSD_CLKINSEL | CLKIN Source for FSD. The PCG_SYNC2.FSD_CLKINSEL bit enables the CLKIN input source for FSD.                                                                                                             |
| 21 (R/W)           | FSD_CLKINSEL | 0 Selects CLKIN0 source for FSB CLKIN                                                                                                                                                                    |
| 21 (R/W)           | FSD_CLKINSEL | 1 Selects CLKIN1 source for FSB CLKIN                                                                                                                                                                    |

Table 31-27: PCG\_SYNC2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | CLKD_CLKINSEL | CLKIN Source for CLKD. The PCG_SYNC2.CLKD_CLKINSEL bit enables the CLKIN input source for CLKD.                                                                                                                                   |
| 19 (R/W)           | FSDSRC        | Frame Sync DSource. The PCG_SYNC2.FSDSRC bit enables the frame sync Dinput source. 0 Output selected by FSDSOURCE bit                                                                                                             |
| 18 (R/W)           | CLKDSRC       | Clock DSource. The PCG_SYNC2.CLKDSRC bit enables the clock Dinput source. 0 Output selected by CLKDSOURCE bit                                                                                                                     |
| 17 (R/W)           | CLKD          | 1 Clock derived from core PLL selected for clockD Clock DEnable. The PCG_SYNC2.CLKD bit enables synchronization of clock Dwith the external                                                                                       |
| 16                 | FSD           | frame sync. 0 Clock disabled 1 Clock enabled                                                                                                                                                                                      |
| (R/W)              |               | Frame Sync DEnable. The PCG_SYNC2.FSD bit enables synchronization of frame sync Dwith the external frame sync. 0 Frame sync disabled                                                                                              |
| 6 (R/W)            | HWC_TRIGEN    | 1 Frame sync enabled Hardware Trigger Sync C Enable. The PCG_SYNC2.HWC_TRIGEN bit enables synchronization of clock C and frame sync C with the external frame sync only after an internal receiver trigger for PCG C is received. |
| 5 (R/W)            | FSC_CLKINSEL  | CLKIN Source for FSC. The PCG_SYNC2.FSC_CLKINSEL bit enables the CLKIN input source for FSC. 0 Selects CLKIN0 as source for FSA CLKIN                                                                                             |

Table 31-27: PCG\_SYNC2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                            |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | CLKC_CLKINSEL | CLKIN Source for CLKC. The PCG_SYNC2.CLKC_CLKINSEL bit enables the CLKIN input source for CLKC.                                                                    |
| 3 (R/W)            | FSCSRC        | Frame Sync Source. The PCG_SYNC2.FSCSRC bit enables the frame sync C input source. 0 Output selected by FSCSOURCE bit                                              |
| 2 (R/W)            | CLKCSRC       | Clock C Source. The PCG_SYNC2.CLKCSRC bit enables the clock C input source. 0 Output selected by CLKCSOURCE bit 1 Clock derived from core PLL selected for clock C |
| 1 (R/W)            | CLKC          | Clock C Enable. The PCG_SYNC2.CLKC bit enables synchronization of clock C with the external frame sync. 0 Clock disabled                                           |
| 0 (R/W)            | FSC           | Frame Sync C Enable. The PCG_SYNC2.FSC bit enables synchronization of frame sync C with the external frame sync.                                                   |

## Precision Clock Frame Sync Synchronization 3 Register

The PCG\_SYNC3 register allows programs to synchronize the clock frame sync units with external frame syncs. Note that the PCG\_CTLD1.CLKSRC bit is overridden if CLKDSRC bit in the PCG\_SYNC3 register is set.

Figure 31-29: PCG\_SYNC3 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000028_a932a5378c0f79e3a8477655e1dffcfd68d4e7bfa98667bc030fc9eb2847ffab.png)

Table 31-28: PCG\_SYNC3 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | FSF_FP1CLKSEL  | Frame Sync F FracNPll 1 Clock Select. The PCG_SYNC3.FSF_FP1CLKSEL bit when enabled, selects FracNPll 1 output clock input clock source to FSF. This feature is enabled only when PCG_BKP_CTLF1.FSSRC bit is set and FSF_FPCLKSEL bit of this register is set.                                             |
| 29 (R/W)           | FSF_FP1CLKSEL  | 0 PCG_EXTX_DAI0 selected for frame sync                                                                                                                                                                                                                                                                   |
| 28 (R/W)           | CLKF_FP1CLKSEL | Clock F FracNPll 1 Clock Select. The PCG_SYNC3.CLKF_FP1CLKSEL bit when enabled, selects FracNPll 1 out- put clock as input clock source to CLKF. This feature is enabled only when PCG_BKP_CTLF1.CLKSRC bit is set and CLKF_FPCLKSEL bit of this register is set.                                         |
| 28 (R/W)           | CLKF_FP1CLKSEL | 0 PCG_EXT_DAI0 selected for clock                                                                                                                                                                                                                                                                         |
| 27 (R/W)           | FSF_FPCLKSEL   | 1 FracNPll 1 output clock selected for clock Frame Sync F FracNPll Clock Select. The PCG_SYNC3.FSF_FPCLKSEL bit when enabled, selects FracNPll out- put clock input clock source to FSF. This feature is enabled only when PCG_BKP_CTLF1.FSSRC bit is set.                                                |
| 27 (R/W)           | FSF_FPCLKSEL   | 0 PCG_EXTX_DAI0 selected for frame sync                                                                                                                                                                                                                                                                   |
| 27 (R/W)           | FSF_FPCLKSEL   | 1 FracNPll output clock selected for frame sync                                                                                                                                                                                                                                                           |
| 26 (R/W)           | CLKF_FPCLKSEL  | put clock as input clock source to CLKF. This feature is enabled only when PCG_BKP_CTLF1.CLKSRC bit is set. 0                                                                                                                                                                                             |
| 26 (R/W)           | CLKF_FPCLKSEL  | PCG_EXT_DAI0 selected for clock                                                                                                                                                                                                                                                                           |
| 22 (R/W)           | HWF_TRIGEN     | Hardware Trigger Sync F Enable. The PCG_SYNC3.HWF_TRIGEN bit enables synchronization of clock F and frame sync F with the external frame sync only after an internal receiver trigger for PCG-F is received. CLKIN Source for FSF. The PCG_SYNC3.FSF_CLKINSEL bit enables the CLKIN input source for FSF. |
| 21 (R/W)           | FSF_CLKINSEL   |                                                                                                                                                                                                                                                                                                           |
| 21 (R/W)           | FSF_CLKINSEL   | 0 Selects CLKIN0 as source for FSA CLKIN                                                                                                                                                                                                                                                                  |
| 21 (R/W)           | FSF_CLKINSEL   | 1 Selects CLKIN1 as source for FSA CLKIN                                                                                                                                                                                                                                                                  |

Table 31-28: PCG\_SYNC3 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                                               |
|--------------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | CLKF_CLKINSEL | CLKIN Source for CLKF. The PCG_SYNC3.CLKF_CLKINSEL bit enables the CLKIN input source for CLKF.                                                                                                                                                                                                       |
| 19 (R/W)           | FSFSRC        | Frame Sync F Source. The PCG_SYNC3.FSFSRC bit enables the frame sync F input source. 0 Output selected by FSDSOURCE bit                                                                                                                                                                               |
| 18 (R/W)           | CLKFSRC       | Clock F Source. The PCG_SYNC3.CLKFSRC bit enables the clock F input source. 0 Output selected by CLKDSOURCE bit                                                                                                                                                                                       |
| 17 (R/W)           | CLKF          | 1 Clock derived from core PLL selected for clockD Clock F Enable. The PCG_SYNC3.CLKF bit enables synchronization of clock F with the external frame sync.                                                                                                                                             |
| 16 (R/W)           | FSF           | 0 Clock disabled 1 Clock enabled Frame Sync F Enable. The PCG_SYNC3.FSF bit enables synchronization of frame sync F with the external                                                                                                                                                                 |
|                    |               | frame sync. 0 Frame sync disabled 1 Frame sync enabled                                                                                                                                                                                                                                                |
| 13 (R/W)           | FSE_FP1CLKSEL | Frame Sync E FracNPll 1 Clock Select. The PCG_SYNC3.FSE_FP1CLKSEL bit when enabled, selects FracNPll 1 output clock input clock source to FSE. This feature is enabled only when PCG_BKP_CTLE1.FSSRC bit is set and FSE_FPCLKSEL bit of this register is set. 0 PCG_EXTX_DAI0 selected for frame sync |

Table 31-28: PCG\_SYNC3 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                           |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | CLKE_FP1CLKSEL | Clock E FracNPll 1 Clock Select. The PCG_SYNC3.CLKE_FP1CLKSEL bit when enabled, selects FracNPll 1 out- put clock as input clock source to CLKE. This feature is enabled only when PCG_BKP_CTLE1.CLKSRC bit is set and CLKE_FPCLKSEL bit of this register is set. |
| 11 (R/W)           | FSE_FPCLKSEL   | Frame Sync E FracNPll Clock Select. The PCG_SYNC3.FSE_FPCLKSEL bit when enabled, selects FracNPll out- put clock input clock source to FSE. This feature is enabled only when PCG_BKP_CTLE1.FSSRC bit is set.                                                     |
| 10 (R/W)           | CLKE_FPCLKSEL  | Clock E FracNPll Clock Select. The PCG_SYNC3.CLKE_FPCLKSEL bit when enabled, selects FracNPll out- put clock as input clock source to CLKE. This feature is enabled only when                                                                                     |
| 6 (R/W)            |                | 0 PCG_EXT_DAI0 selected for clock 1 FracNPll output clock selected for clock                                                                                                                                                                                      |
|                    | HWE_TRIGEN     | Hardware Trigger Sync E Enable. The PCG_SYNC3.HWE_TRIGEN bit enables synchronization of clock E and frame sync E with the external frame sync only after an internal receiver trigger for PCG-E is received.                                                      |
| 5 (R/W)            | FSE_CLKINSEL   | CLKIN Source for FSE. The PCG_SYNC3.FSE_CLKINSEL bit enables the CLKIN input source for FSE. 0 Selects CLKIN0 as source for FSA CLKIN 1 Selects CLKIN1 as source for FSA CLKIN                                                                                    |
| 4 (R/W)            | CLKE_CLKINSEL  | CLKIN Source for CLKE. The PCG_SYNC3.CLKE_CLKINSEL bit enables the CLKIN input source for CLKE.                                                                                                                                                                   |

Table 31-28: PCG\_SYNC3 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | FSESRC     | Frame Sync Source. The PCG_SYNC3.FSESRC bit enables the frame sync E input source.                                                                         |
| 2 (R/W)            | CLKESRC    | Clock E Source. The PCG_SYNC3.CLKESRC bit enables the clock E input source. 0 Output selected by CLKCSOURCE bit                                            |
| 1 (R/W)            | CLKE       | 1 Clock derived from core PLL selected for clock C Clock E Enable. The PCG_SYNC3.CLKE bit enables synchronization of clock E with the external frame sync. |
| 0 (R/W)            | FSE        | Frame Sync E Enable. The PCG_SYNC3.FSE bit enables synchronization of frame sync E with the external frame sync. 0 Frame sync disabled                     |

## Precision Clock Frame Sync Synchronization 4 Register

The PCG\_SYNC4 register allows programs to synchronize the clock frame sync units with external frame syncs. Note that the PCG\_CTLD1.CLKSRC bit is overridden if CLKDSRC bit in the PCG\_SYNC4 register is set.

Figure 31-30: PCG\_SYNC4 Register Diagram

![Image](34_Precision_Clock_Generators_(PCG)_artifacts/image_000029_b9135fa48ee4133776c1c5e9ca7385b46804b562056a08baefcd1651253ee3af.png)

Table 31-29: PCG\_SYNC4 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                   |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/W)           | HWH_TRIGEN   | Hardware Trigger Sync HEnable. The PCG_SYNC4.HWH_TRIGEN bit enables synchronization of clock Hand frame sync Hwith the external frame sync only after an internal receiver trigger for PCG-H is received. |
| 21 (R/W)           | FSH_CLKINSEL | CLKIN Source for FSH. The PCG_SYNC4.FSH_CLKINSEL bit enables the CLKIN input source for FSH.                                                                                                              |
| 21 (R/W)           | FSH_CLKINSEL | 0 Selects CLKIN0 as source for FSA CLKIN                                                                                                                                                                  |
| 21 (R/W)           | FSH_CLKINSEL | 1 Selects CLKIN1 as source for FSA CLKIN                                                                                                                                                                  |

Table 31-29: PCG\_SYNC4 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | CLKH_CLKINSEL | CLKIN Source for CLKH. The PCG_SYNC4.CLKH_CLKINSEL bit enables the CLKIN input source for CLKH.                                                                                                                                   |
| 19 (R/W)           | FSHSRC        | Frame Sync HSource. The PCG_SYNC4.FSHSRC bit enables the frame sync Hinput source. 0 Output selected by FSDSOURCE bit                                                                                                             |
| 18 (R/W)           | CLKHSRC       | Clock HSource. The PCG_SYNC4.CLKHSRC bit enables the clock Hinput source. 0 Output selected by CLKDSOURCE bit                                                                                                                     |
| 17 (R/W)           | CLKH          | 1 Clock derived from core PLL selected for clockD Clock HEnable. The PCG_SYNC4.CLKH bit enables synchronization of clock Hwith the external                                                                                       |
| 16                 | FSH           | frame sync. 0 Clock disabled 1 Clock enabled                                                                                                                                                                                      |
| (R/W)              |               | Frame Sync HEnable. The PCG_SYNC4.FSH bit enables synchronization of frame sync Hwith the external frame sync. 0 Frame sync disabled                                                                                              |
| 6 (R/W)            | HWG_TRIGEN    | 1 Frame sync enabled Hardware Trigger Sync G Enable. The PCG_SYNC4.HWG_TRIGEN bit enables synchronization of clock G and frame sync G with the external frame sync only after an internal receiver trigger for PCG G is received. |
| 5 (R/W)            | FSG_CLKINSEL  | CLKIN Source for FSG. The PCG_SYNC4.FSG_CLKINSEL bit enables the CLKIN input source for FSG. 0 Selects CLKIN0 as source for FSA CLKIN                                                                                             |

Table 31-29: PCG\_SYNC4 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                            |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | CLKG_CLKINSEL | CLKIN Source for CLKG. The PCG_SYNC4.CLKG_CLKINSEL bit enables the CLKIN input source for CLKG.                                                                    |
| 3 (R/W)            | FSGSRC        | Frame Sync Source. The PCG_SYNC4.FSGSRC bit enables the frame sync G input source.                                                                                 |
| 2 (R/W)            | CLKGSRC       | Clock G Source. The PCG_SYNC4.CLKGSRC bit enables the clock G input source. 0 Output selected by CLKCSOURCE bit 1 Clock derived from core PLL selected for clock C |
| 1 (R/W)            | CLKG          | Clock G Enable. The PCG_SYNC4.CLKG bit enables synchronization of clock G with the external frame sync. 0 Clock disabled                                           |
| 0 (R/W)            | FSG           | Frame Sync G Enable. The PCG_SYNC4.FSG bit enables synchronization of frame sync G with the external frame sync.                                                   |