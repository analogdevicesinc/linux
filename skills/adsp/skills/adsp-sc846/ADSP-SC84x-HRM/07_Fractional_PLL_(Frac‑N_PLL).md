## 4   Fractional PLL (Frac -N PLL)

This processor uses the Fractional Phased-locked Loop (Frac -N PLL) to generate precise and low-jitter audio clock frequencies.

IMPORTANT: In this chapter, ADI uses the abbreviation Frac -N PLL to refer to the entire Fractional PLL module. The Frac-N PLL subblocks are the Fractional PLL (FPLL) and Digital PLL (DPLL). The FPLL subblock generates a clock frequency that is a fractional multiple of CLKIN. The DPLL subblock performs phase alignment with the output clock to an external reference clock (FIN).

The Fractional Phased-locked Loop (Frac-N PLL) operates in one of two modes:

- FPLL Only Mode-generates a clock with frequency values that are fractional multiples of the CLKIN frequency. As an example, the processor can output an audio clock of frequency 24.576 MHz generated from a 25 MHz CLKIN frequency.
- DPLL Plus FPLL Mode-consists of a digital filter and controller (DPLL), combined with the FPLL to create a dual-loop PLL. The DPLL provides fractional and integer parts of divider values to the FPLL block. The Frac -N PLL output clock feeds back the signal to the DPLL for phase-alignment with the external reference clock (FIN). As an example, you can obtain a 1 KHz PPS output from the EMAC block, to use as a reference for generating a 24.576 MHz audio clock that is phase-aligned to the pulses per second (PPS) output.

## Frac-N PLL Features

The Frac -N PLL supports the following features:

- Generation of a clock with frequency values that are fractional multiples of the CLKIN frequency.
- Phase-alignment of the output clock with a selected external reference clock (FIN).
- FIN sources from DAI0/DAI1 PCGx\_EXTCLK, GPIO (FRACNPLL\_PTP\_CLK), or RGMII PTP PPS0-3 signals.
- FPLL frequency, DPLL frequency, and phase lock interrupt and trigger.
- DPLL frequency unlock interrupt.
- FPLL and DPLL trigger wait (TWAIT).
- Routing of the Frac -N PLL signals to the SYS\_CLKOUT.

- Routing of the Frac -N PLL output clock to the Timer and PWM blocks.

## Frac-N PLL Definitions

To make the best use of the Frac-N PLL, it is helpful to understand the following terms.

## Phase-locked Loop (PLL)

A feedback circuit designed to synchronize one circuit board to the phase of its on board clock with an external timing signal. PLL circuits operate by comparing the phase of an external signal to the phase of a clock signal produced by a voltage controlled crystal oscillator (VCXO).

## Fractional Phase-locked Loop (Frac -N PLL)

The entire fractional PLL module used to generate precise and low-jitter audio clock frequencies. The Frac-N PLL consists of two subblocks, the FPLL and the DPLL.

## Fractional PLL (FPLL)

The FPLL subblock of the Frac -N PLL that generates a clock frequency, which is a fractional multiple of the CLKIN.

## Digital PLL (DPLL)

The DPLL subblock of the Frac -N PLL that performs phase alignment with the output clock to an external reference clock (FIN).

## Digital Audio Interfaces (DAI)

Consists of a group audio peripherals connected using the Signal Routing Unit (SRU). The SRU connects inputs and outputs of the DAI peripherals with each other and to the external pins. This arrangement permits peripherals to be interconnected and accommodates different audio configurations without making separate external pin connections.

## Ethernet Media Access Controller (EMAC)

The EMAC peripheral in the processor enables network connectivity to applications through an Ethernet interface. This function enables applications to support TCP/IP based network communication. At the system end, the module supports direct connection with the system crossbar bus for memory or memory-mapped registers (MMR) transactions.

## Pulse-Width Modulator (PWM)

The processor component that provides a way of controlling certain analog quantities, by varying the pulse width of a fixed frequency rectangular waveform.

## FIN (external reference clock)

An external reference clock used to compare against the Frac-N PLL output clock by the DPLL for phase-alignment corrections.

## Phase/Frequency Detector (PFD)

A linear or binary phase detector that compares the phase difference between the input reference signal and the output signal from a voltage controlled oscillator (VCO).

## Precision Clock Generator (PCG)

Used to produce a pair of signals from a clock input signal. The PCG normally generates two signals used as a serial bit clock and frame sync pair.

## Precision Time Protocol (PTP)

A protocol used to synchronize clocks in processors and their peripherals. On a processor, it achieves clock accuracy in the sub-microsecond range, making it suitable for measurement and control systems.

## Pulses-per-Second (PPS)

A signal that has a width of less than one second and a sharply rising or abruptly falling edge that accurately repeats once per second.

## Signal Routine Unit (SRU)

A precision clock generator component that routes all of the precision clock generator (PCG) signals in one digital audio interface (DAI) for external usage.

## Trigger Routing Unit (TRU)

The processor component that provides a system level sequence control without core intervention. It maps the trigger controllers (trigger generators) to trigger targets (triggers receivers). The receiver endpoints can be configured to respond to triggers in different ways.

## Voltage Controlled Oscillator (VCO)

An electronic oscillator where the voltage input controls the oscillation frequency. The applied input voltage determines the instantaneous oscillation frequency.

## Frac -N PLL Functional Description

This section provides a description of the Frac -N PLL, including a list of its registers, interrupts, and functional block diagrams.

## Frac-N PLL Block Diagram

The Frac-N PLL Block Diagram shows the organization and communication of the Frac-N PLL It contains the following major components.

- FPLL subblock
- DPLL subblock
- Clock multiplexers-that switch between functional modes and select the FIN source
- REG-interface-configure and control the FPLL, DPLL, and clock multipliers. The REG-IF also reads the status parameters.

Figure 4-1: Frac -N PLL Block Diagram © Silicon Creations

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000000_d2539d1baf64852740206fe05093fb1656e3a9cfe7307ac3e0b9cd8db3d78ecb.png)

NOTE: 1. The DFT\_CLK is for Analog Devices internal use only.

## FPLL Detailed Block Diagram

The FPLL generates a clock with a frequency value that is a fractional multiple of the CLKIN frequency.

The FPLL Detailed Subblock Diagram shows how the FPLL generates a clock with a frequency value that is a fractional multiple of the CLKIN frequency.

Figure 4-2: FPLL Detailed Subblock Diagram © Silicon Creations

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000001_eef494a5386c30830444ce2553e64f52d5994e6ba1a35159a0efafd699a28cc8.png)

The CLKIN is the input clock source for the FPLL. (See the High Performance SHARC-FX DSP Core With Arm-Based Connectivity/Safety-Preliminary Data Sheet for the specific CLKIN frequency range.) The CLKIN frequency is divided with the FRACNPLL\_FPLL\_CTL1.REFDIV field and then goes to the voltage controlled oscillator (VCO). The VCO multiplies the divided CLKIN with the fractional value using the configured FRACNPLL\_FPLL\_CTL1.INT and FRACNPLL\_FPLL\_CTL0.FRAC fields. Engineers can further divide the VCO output (FOUTVCO) with the post divider (POSTDIV), using the configured FRACNPLL\_FPLL\_CTL1.POSTDIV field. FOUTPOSTDIV is the resulting output clock.

Set (=1) the FRACNPLL\_FPLL\_CTL0.BYPASS bit and bypass the CLKIN directly to the FOUTPOSTDIV output. They can further divide the FOUTPOSTDIV frequency 2, 4, 8, and 16 to generate the FOUTPOSTDIV/2, FOUTPOSTDIV/4, FOUTPOSTDIV/8 or FOUTPOSTDIV/16 signals. The CLKSSCG signal works as a synchronization signal, and feeds back to the DPLL when in the DPLL plus FPLL Mode. This clock typically operates at a low frequency (same frequency as the PLL input clock), but also has a low duty cycle (typically about 10 -20% ).

## DPLL and FPLL Block Diagram

The DPLL synchronizes the FPLL clock output with the on-chip (RGMII PTP PPS) or the off-chip (DAI0/DAI1/ GPIO) reference clock (FIN). The DPLL Detailed Subblock Diagram with FPLL Interface shows a detailed diagram of the DPLL subblock and its interface to the FPLL subblock.

Figure 4-3: DPLL Detailed Subblock Diagram with FPLL Interface © Silicon Creations

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000002_9e357d17d2691ce17c8f9610702bcacd2c73b7c3b23ef79b66ccbf8bc752814a.png)

The DPLL interfaces to the FPLL by controlling its integer and fractional feedback division bits. The DPLL has a broadly tuneable bandwidth to filter noisy references. Because the FPLL has a separate clean reference, its own noise characteristics and not those of the reference dominates the noise at the FPLL output.

The DPLL uses three phases of acquisition: frequency tracking, phase settling control, and phase tracking. Each phase has separate programmable gains to tune the DPLL performance for the specific application.

- Frequency gain control and frequency tolerance control (tracking) -modify the response time of the PLL when in frequency tracking mode and permit tracking or filtering of a reference, tuned to the expected noise of the reference.
- Phase settling control -manages the transition from frequency tracking mode to phase tracking mode with a high gain to ensure the DPLL can lock under all conditions.
- Phase tracking -enables the DPLL to have a very low bandwidth, effectively averaging out higher frequency noise that can happen because of spread spectrum or other noisy input references.

## ADSP-2184x FRACNPLL Register List

The FRACnPLL module combines DPLL and Frac PLL to generate an on-chip, low jitter, precision audio clock.

Table 4-1: ADSP-2184x FRACNPLL Register List

| Name                | Description                       |
|---------------------|-----------------------------------|
| FRACNPLL_CTL        | Integration Control Register      |
| FRACNPLL_DPLL_CTL0  | DPLL Control Register 0           |
| FRACNPLL_DPLL_CTL1  | DPLL Control Register 1           |
| FRACNPLL_DPLL_CTL2  | DPLL Control Register 2           |
| FRACNPLL_DPLL_CTL3  | DPLL Control Register 3           |
| FRACNPLL_DPLL_CTL4  | DPLL Control Register 4           |
| FRACNPLL_DPLL_CTL5  | DPLL Control Register 5           |
| FRACNPLL_DPLL_CTL6  | DPLL Control Register 6           |
| FRACNPLL_DPLL_CTL7  | DPLL Control Register 7           |
| FRACNPLL_DPLL_CTL8  | DPLL Control Register 8           |
| FRACNPLL_DPLL_CTL9  | DPLL Control Register 9           |
| FRACNPLL_DPLL_STAT0 | DPLL Status Register 0            |
| FRACNPLL_DPLL_STAT1 | DPLL Status Register 1            |
| FRACNPLL_DPLL_STAT2 | DPLL Status Register 2            |
| FRACNPLL_FPLL_CTL0  | Fractional PLL Control Register 0 |
| FRACNPLL_FPLL_CTL1  | Fractional PLL Control Register 1 |
| FRACNPLL_STAT       | Integration Status Register       |

## ADSP-2184x FRACNPLL Interrupt List

Table 4-2: ADSP-2184x FRACNPLL Interrupt List

|   Interrupt ID | Name                    | Description                         | Sensitivity   | DMA Channel   |
|----------------|-------------------------|-------------------------------------|---------------|---------------|
|            114 | FRACNPLL0_FRAC_LOCK     | FRACNPLL0 Frac Pll Lock Interrupt   | Edge          |               |
|            115 | FRACNPLL1_FRAC_LOCK     | FRACNPLL1 Frac Pll Lock Interrupt   | Edge          |               |
|            116 | FRACNPLL0_FRAC_UN- LOCK | FRACNPLL0 Frac Pll Unlock Interrupt | Edge          |               |
|            117 | FRACNPLL1_FRAC_UN- LOCK | FRACNPLL1 Frac Pll Unlock Interrupt | Edge          |               |

## ADSP-2184x FRACNPLL Trigger List

Table 4-3: ADSP-2184x FRACNPLL Trigger List Generators

|   Trigger ID | Name                     | Description                     | Sensitivity   |
|--------------|--------------------------|---------------------------------|---------------|
|           55 | FRACNPLL0_FRACPLL_T RIGM | FRACNPLL0 Frac Pll Lock Trigger | Edge          |
|           56 | FRACNPLL1_FRACPLL_T RIGM | FRACNPLL1 Frac Pll Lock Trigger | Edge          |

Table 4-4: ADSP-2184x FRACNPLL Trigger List Receivers

|   Trigger ID | Name                     | Description                      | Sensitivity   |
|--------------|--------------------------|----------------------------------|---------------|
|          100 | FRACNPLL0_FRACPLL_T RIGS | FRACNPLL0 Frac Pll Start Trigger | Pulse         |
|          101 | FRACNPLL1_FRACPLL_T RIGS | FRACNPLL1 Frac Pll Start Trigger | Pulse         |

## Frac -N PLL Architectural Concepts

This section covers the following major concepts:

- Interrupts and Triggers
- DAI Clock Selection
- Frac -N PLL CLKOUT Clock Selection
- Usage of Frac-N PLL Output Clock

## Interrupts and Triggers

NOTE: For the SHARC-FX core, configure the interrupt controller for the FRACNPLL\_FRAC\_LOCK interrupt to use an edge-sensitive interrupt to avoid multiple re-entries inside the ISR.

The Cortex-A55 core does not support the FRACNPLL\_FRAC\_LOCK interrupt.

The following triggers and interrupts are part of the Frac-N PLL.

- Lock Interrupt-Trigger
- DPLL Unlock Interrupt
- TWAIT Trigger Receiver

## Lock Interrupt-Trigger

The Frac-N PLL supports the following lock interrupts.

- FPLL frequency lock (FPLL\_FLOCK)
- DPLL frequency lock (DPLL\_FLOCK)
- DPLL phase lock (DPLL\_PLOCK)

These interrupts can be sent to the SEC/NVIC using the interrupt signal FRACNPLL\_FRAC\_LOCK . The same signal routes the interrupt to the trigger routing unit as the trigger requester signal FRACPLL\_TRIGM and to the GPIO (FRACNPLL\_FLOCK).

Configure the FRACNPLL\_CTL.LOCK\_SEL field to select one of the following values:

- 0-routes none of the three interrupts
- 1-uses the FPLL frequency lock (FPLL\_FLOCK)
- 2-uses the DPLL frequency lock (DPLL\_FLOCK)
- 3-uses the DPLL phase lock (DPLL\_PLOCK)

Figure 4-4: Frac-N PLL Lock Interrupt and Trigger Multiplexer

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000003_c78c2d89cf08929d7d058d9f139eeee6a64b4aaff4d2e19e8eb4e1b61b4e573a.png)

## DPLL Unlock Interrupt

Applications can route the DPLL frequency unlock signal to SEC/NVIC FRAC\_UNLOCK interrupt. This interrupt can detect the loss of the FIN reference clock signal and take any necessary action at the system level.

## TWAIT Trigger Receiver

This processor has an available trigger (receiver) wait or TWAIT to release the power down and reset signal of the FPLL and DPLL. This auto-synchronizes the start of an operation on any available system event. Set (=1) the FRACNPLL\_CTL.F\_TWAIT (FPLL) and FRACNPLL\_CTL.D\_TWAIT (DPLL) bits to indicate one of the two operating modes. (See Frac -N PLL Operating Modes for more information.)

Engineers can configure both PLLs and released from reset by clearing (=0) the FRACNPLL\_CTL.FCPD (required) and the FRACNPLL\_CTL.DCRESET (release from reset) bits. When the respective TWAIT bit is set (=1), an actual reset to FPLL/DPLL does not get unasserted immediately. The reset waits for the trigger to arrive. Once the trigger arrives, the FPLL or DPLL reset is unasserted.

CAUTION: The TWAIT only controls the reset-deassertion. TWAIT bits do not gate the reset assertion to FPLL and DPLL.

## DAI Clock Selection

The Frac-N PLL DAI CLock Multiplexer figure shows how it provides the FPLL output clock FOUTPOSTDIV in available sub-multiples (2, 4, 6, or 8), which can route different values of clock frequencies to the DAI, without disturbing the feedback clock or the DPLL frequency and phase lock.

Figure 4-5: Frac-N PLL DAI Clock Multiplexer

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000004_8f8a57545915f04a8e22a0670719853a80a766fbb2715f7c17dea643d9e07428.png)

For a FOUTPOSTDIV clock of frequency 98.304 MHz, one can program the multiplexer to select output clock values that include: 49.152 MHz (/2), 24.576 MHz (/4), 12.288 MHz (/8), and 6.144 MHz (/16). See the DAI Output Clock Selection table for valid configuration values for the FRACNPLL\_CTL.DAICLK\_SEL field. Engineers can set (=1) a clock gating control after the multiplexer using the FRACNPLL\_CTL.CK\_STBL bit to ensure sending out only a stable clock.

Table 4-5: DAI Output Clock Selection

|   FRACNPLL_CTL. DAICLK_SEL Value | Clock Selected        | Example Frequency   |
|----------------------------------|-----------------------|---------------------|
|                                0 | No Clock              | N/A                 |
|                                1 | FOUTPOSTDIV/16        | 6.144 MHz           |
|                                2 | FOUTPOSTDIV/8         | 12.288 MHz          |
|                                3 | FOUTPOSTDIV/4         | 24.576 MHz          |
|                                4 | FOUTPOSTDIV/2         | 45.152 MHz          |
|                                5 | FOUTPOSTDIV (Default) | 98.304 MHz          |

## Frac -N PLL CLKOUT Clock Selection

The FPLL CLKOUT Multiplexer diagram shows how a multiplexer detects all clocks and synchronization signals (CLKSSCG) for the Frac -N PLL.

Figure 4-6: FPLL CLKOUT Multiplexer

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000005_cb68797c2ff669e9f8f3819351eb6d4ba5119a79f82922a97af65740aa59a2b2.png)

Configure the FRACNPLL\_CTL.CLKOUT\_SEL field to program the multiplexer by using the values in the FPLL\_CLKOUT Multiplexer Selection table. T o meet the maximum operating pad frequency, adequately-added divider stages exist on each clock path (based on their maximum operating frequency) before connecting to the FPLL CLKOUT multiplexer. The output of the CLKOUT multiplexer merges to the main CLKOUT multiplexer. See the Clock Generation Unit (CGU) section for more information.

Table 4-6: FPLL\_CLKOUT Multiplexer Selection

|   FRACNPLL_CTL. CLKOUT_SEL Value | Signal Routed to FPLL_CLKOUT Multiplexer   |
|----------------------------------|--------------------------------------------|
|                                1 | CLKSSCG                                    |
|                                2 | FOUTPOSTDIV/64 (Default)                   |
|                                3 | FOUTVCO/64                                 |

## Usage of Frac-N PLL Output Clock

Users can route the FPLL output clock to the following processor modules:

- Digital Audio Interface (DAI)
- Timer
- Pulse-Width Modulator (PWM)

## Digital Audio Interface (DAI)

The FPLL output clock is routed to one of the PCGx\_EXTCLK signals. The output clock can also be one of the source signals for Group D of the Signal Routing Unit (SRU). This method routes the clock to one or more DAI0/1 pin buffers for external use.

Figure 4-7: FPLL Output to PCG or SRU

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000006_8429a951b03ecc8ab24b81d881dc290ae342aa5dcb209df1c94bae9f511e9c03.png)

## Timer

The FPLL output clock connects to the TM0\_ACLK port of the all the timers (through multiplexers) and with the other TM0\_ACLK sources. Configure the MISCREG\_CLK\_MUX\_SEL.PCG\_CLKSEL field to select the PCG clock or set (=1) the MISCREG\_CLK\_MUX\_SEL.PCG\_CLKSELF bit to select the FPLL clock for the timer.

Figure 4-8: FPLL Output Clock Usage for Timer

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000007_04b5e0fe9b74a32363fcebb21af16268e6a41745158035afa250b95c00ff5567.png)

## Pulse-Width Modulator (PWM)

The FPLL output clock is used as an operating clock for the on chip PWM along with the SYSCLK to synchronize the PWM output with the FPLL output clock. Configure the MISCREG\_CLK\_MUX\_SEL.EPWM\_CLKSEL field to select the PCG clock or set (=1) the MISCREG\_CLK\_MUX\_SEL.EPWM\_CLKSELF bit to select the FPLL clock for the PWM.

Figure 4-9: FPLL Output Clock Usage for PWM

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000008_14ab588ef5a1cc48f826a4adacc54ddd14dc495c1407f80e7f0243ac982c5da9.png)

## Frac -N PLL Operating Modes

The Frac-N PLL operates in one of these two modes:

- FPLL Only Mode-clock generation using only the FPLL
- DPLL Plus FPLL Mode-clock generation synchronous with a lower frequency of an external reference clock source (FIN)

## FPLL Only Mode

The FPLL generates a clock with frequency values that are fractional multiples of the CLKIN frequency. For example, a CLKIN frequency of 25 MHz can generate an audio clock of frequency 24.576 MHz. The output clock can be used from the FPLL either internally with audio peripherals (using the PCG) or driven out of the chip using DAI pin(s) and connected to external audio components. Configure the FRACNPLL\_FPLL\_CTL0 and FRACNPLL\_FPLL\_CTL1 registers to control the FPLL clock generation.

The Clock Multiplexer Configurations for FPLL Only Mode table shows the values for the different clock multiplexer configurations.

Table 4-7: Clock Multiplexer Configurations for FPLL Only Mode

| Multiplexer Name   | Register Bit Field    |   Required Value |   Default Value |
|--------------------|-----------------------|------------------|-----------------|
| FIN                | FRACNPLL_CTL.FIN_SEL  |                0 |               1 |
| FIN-DFT            | FRACNPLL_CTL.FIN_DFT  |                0 |               1 |
| FFB-DFT            | FRACNPLL_CTL.FFB_DFT  |                0 |               1 |
| CLKS-DFT           | FRACNPLL_CTL.CLKS_DFT |                0 |               1 |
| INT-DFT            | FRACNPLL_CTL.DDIV_OVR |                1 |               0 |
| FRAC-DFT           | FRACNPLL_CTL.FDIV_OVR |                1 |               0 |

## DPLL Plus FPLL Mode

DPLL Plus FPLL Mode-clock generation synchronized and multiplied with an external clock reference (FIN).

Use the DPLL or multiply and synchronize the output clock of FPLL with a lower-frequency external reference clock source (FIN). The DPLL provides the fractional and integer part of divider values to the FPLL. The processor feeds back the output clock of the FPLL (FOUTPH1 or FOUTPOSTDIV/2) to the DPLL to align with the FIN external clock reference phase.

## FIN (External Reference Clock) Sources

Use one of the following options to select the FIN:

- Internal Ethernet PTPPPS signal - RGMII/EMAC0 block, that is, EMAC0\_PTPPPS0 through EMAC0\_PTPPPS03.
- External Ethernet PTPPPS or any other reference clock source - the GPIO (FRACNPLL\_PTP\_CLK) of the processor.
- External reference clock source connected - any DAI (0/1) pin of the processor routed using the PCGx\_EXTCLK SRU signal.

Use the following steps to configure the FIN source:

1. Configure the value of the FRACNPLL\_CTL.FIN\_SEL field (=1) to select the FIN from DAI ( PCGx\_EXTCLK ) or configure the value (=2) to select the FIN from the GPIO (FRACNPLL\_PTP\_CLK) or one of the EMAC0\_PTPPPS0 through EMAC0\_PTPPPS3 signals.
2. When the FRACNPLL\_CTL.FIN\_SEL field value (=1), engineers can configure the MISCREG\_CLK\_MUX\_SEL.PCG\_MCLKSEL bit to select the FIN source from different PCGx\_EXTCLK signals. (See the FIN Source Selection using MISCREG\_CLK\_MUX\_SEL.PCG\_MCLKSEL table.)
3. When the FRACNPLL\_CTL.FIN\_SEL field value (=2), the MISCREG\_CLK\_MUX\_SEL.PTP\_RCLKSEL field can select the FIN source from either the GPIO (FRACNPLL\_PTP\_CLK) or one of the EMAC0\_PTPPPS0 through EMAC0\_PTPPPS3 signals. (See FIN Source Selection using MIS-CREG\_CLK\_MUX\_SEL. PTP\_RCLKSEL table).

NOTE: The FRACNPLL\_CTL.REFCLK\_IE bit must be set (=1) when configuring GPIO (FRACNPLL\_PTP\_CLK) as the FIN source.

Figure 4-10: Possible FIN Sources

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000009_612441de0e27aded7ad7ae2f7cd4dd6507fa2cbd9f4ca212b3fe5589ad2a5abf.png)

Table 4-8: FIN Source Selection using MISCREG\_CLK\_MUX\_SEL.PCG\_MCLKSEL

|   SL Number |   MISCREG_CLK_MUX_SEL.PCG_MCLK SEL | FIN Source       |
|-------------|------------------------------------|------------------|
|           1 |                                  0 | DAI0_PCGA_EXTCLK |
|           2 |                                  1 | DAI0_PCGB_EXTCLK |
|           3 |                                  2 | DAI0_PCGC_EXTCLK |
|           4 |                                  3 | DAI0_PCGD_EXTCLK |
|           5 |                                  4 | DAI1_PCGA_EXTCLK |
|           6 |                                  5 | DAI1_PCGB_EXTCLK |
|           7 |                                  6 | DAI1_PCGC_EXTCLK |
|           8 |                                  7 | DAI1_PCGD_EXTCLK |

Table 4-9: FIN source selection using MISCREG\_CLK\_MUX\_SEL.PTP\_RCLKSEL

|   SL Number |   MISCREG_CLK_MUX_SEL.PTP_RCLK SEL | FIN Source              |
|-------------|------------------------------------|-------------------------|
|           1 |                                  0 | GPIO (FRACNPLL_PTP_CLK) |
|           2 |                                  1 | EMAC0_PTPPPS0           |

Table 4-9: FIN source selection using MISCREG\_CLK\_MUX\_SEL.PTP\_RCLKSEL (Continued)

|   SL Number |   MISCREG_CLK_MUX_SEL.PTP_RCLK SEL | FIN Source    |
|-------------|------------------------------------|---------------|
|           3 |                                  2 | EMAC0_PTPPPS1 |
|           4 |                                  3 | EMAC0_PTPPPS2 |
|           5 |                                  4 | EMAC0_PTPPPS3 |

## DPLL Plus FPLL Clock Multiplexer Configuration

The Clock Multiplexer Configurations for the DPLL Plus FPLL Mode table describes the multiplexer configurations.

Table 4-10: Clock Multiplexer Configurations for the DPLL Plus FPLL Mode

| Multiplexer Name   | Register Bit Field    | Required Value   |   Default Value |
|--------------------|-----------------------|------------------|-----------------|
| FIN                | FRACNPLL_CTL.FIN_SEL  | 1 or 2           |               1 |
| FIN-DFT            | FRACNPLL_CTL.FIN_DFT  | 1                |               1 |
| FFB-DFT            | FRACNPLL_CTL.FFB_DFT  | 1                |               1 |
| CLKS-DFT           | FRACNPLL_CTL.CLKS_DFT | 1                |               1 |
| INT-DFT            | FRACNPLL_CTL.DDIV_OVR | 0                |               0 |
| FRAC-DFT           | FRACNPLL_CTL.FDIV_OVR | 0                |               0 |

## Frac -N PLL Programming Model

This section describes the procedures to program the two Frac -N PLL operating modes.

- Programming FPLL Only Mode
- FPLL Only Mode TWAIT
- Programming Model for DPLL Plus FPLL Mode
- DPLL Plus FPLL Mode TWAIT (ADI does not recommend using this procedure.)
- Programming the FIN Unlock and Re-lock DPLL

## Programming FPLL Only Mode

Use the following steps to program the FPLL Only Mode multiplexer configuration:

1. Configure the Frac -N PLL clock multiplexer using Table 4-7 Clock Multiplexer Configurations for FPLL Only Mode.
2. Keep DPLL and FPLL in reset by setting (=1) the FRACNPLL\_CTL.DCRESET and FRACNPLL\_CTL.FCPD ) bits. By default, these bits are set after system reset.

3. Keep the FRACNPLL\_CTL.CK\_STBL bit cleared (=0). By default, these bits are cleared after system reset.
4. Configure the FRACNPLL\_FPLL\_CTL0.FRAC , FRACNPLL\_FPLL\_CTL1.INT , FRACNPLL\_FPLL\_CTL1.REFDIV , and FRACNPLL\_FPLL\_CTL1.POSTDIV fields.
5. Release FPLL from reset by clearing (=0) the FRACNPLL\_CTL.FCPD bit.
6. Wait for the FPLL frequency lock by polling the FRACNPLL\_STAT.FSLOCK bit (blocking mode) or configure the FRACNPLL\_CTL.LOCK\_SEL bit to generate the FPLL frequency lock interrupt (non-blocking mode).
7. Release the clock by setting (=1) the FRACNPLL\_CTL.CK\_STBL bit.

## FPLL Only Mode TWAIT

Use this procedure to invoke the trigger receiver.

1. Configure Frac -N PLL clock multiplexer using Table 4-7 Clock Multiplexer Configurations for FPLL Only Mode.
2. Keep DPLL and FPLL in reset by setting (=1) the FRACNPLL\_CTL.DCRESET and FRACNPLL\_CTL.FCPD bits. By default, these bits are set after system reset.
3. Keep the FRACNPLL\_CTL.CK\_STBL bit cleared (=0). By default, these bits are cleared after system reset.
4. Configure the FRACNPLL\_FPLL\_CTL0.FRAC , FRACNPLL\_FPLL\_CTL1.INT , FRACNPLL\_FPLL\_CTL1.REFDIV , and FRACNPLL\_FPLL\_CTL1.POSTDIV fields.
5. Wait for the FPLL frequency lock by polling the FRACNPLL\_STAT.FSLOCK bit (blocking mode) or configure the FRACNPLL\_CTL.LOCK\_SEL bit to generate the FPLL frequency lock interrupt (non-blocking mode).
6. Release the clock by setting (=1) the FRACNPLL\_CTL.CK\_STBL bit.
7. Set (=1) the FRACNPLL\_CTL.F\_TWAIT bit.
8. Set (=1) the FRACNPLL\_CTL.FCPD bit.

On arrival of the trigger, the actual reset to FPLL is de-asserted and the FPLL starts operating as expected.

## Programming Model for DPLL Plus FPLL Mode

Use the following steps to program the DPLL plus FPLL mode:

1. Configure Frac -N PLL clock multiplexer as per Table 4-10 Clock Multiplexer Configurations for the DPLL Plus FPLL Mode.
2. Keep DPLL and FPLL in reset by setting (=1) the FRACNPLL\_CTL.DCRESET and FRACNPLL\_CTL.FCPD bits. By default, these bits are set after a system reset.

3. Keep the FRACNPLL\_CTL.CK\_STBL bit cleared (=0). By default, these bits are cleared after system reset.
4. Configure parameters in the DPLL registers FRACNPLL\_DPLL\_CTL0 through FRACNPLL\_DPLL\_CTL9 , including values for FRACNPLL\_DPLL\_CTL5.INT\_PRESET and FRACNPLL\_DPLL\_CTL4.FRAC\_PRESET . Clear the FRACNPLL\_DPLL\_CTL4.HOLD and FRACNPLL\_DPLL\_CTL7.PRESET\_EN bits.
5. Release the DPLL reset by clearing (=0) the FRACNPLL\_CTL.DCRESET bit.
6. Set (= 1) the FRACNPLL\_DPLL\_CTL7.PRESET\_EN bit.
7. Set (= 1) the FRACNPLL\_DPLL\_CTL4.HOLD bit.
8. Configure the FRACNPLL\_FPLL\_CTL0 / 1 registers. FRACNPLL\_FPLL\_CTL0.FRAC and FRACNPLL\_FPLL\_CTL1.INT should be set to zero as these values DPLL provides in this mode. FPLL uses the FRACNPLL\_DPLL\_CTL5.INT\_PRESET and FRACNPLL\_DPLL\_CTL4.FRAC\_PRESET values programmed in step four, plus with the values programmed in FRACNPLL\_FPLL\_CTL1.REFDIV , and FRACNPLL\_FPLL\_CTL1.POSTDIV bit fields as a starting point for further frequency and phase lock process.
9. Once the FPLL has asserted a frequency lock, set (=1) the FRACNPLL\_STAT.FSLOCK bit, and wait for the DPLL to assert its frequency lock ( FRACNPLL\_STAT.DSFLOCK =1).
10. Once the DPLL has asserted frequency lock, clear (=0) FRACNPLL\_DPLL\_CTL7.PRESET\_EN and FRACNPLL\_DPLL\_CTL4.HOLD bits and wait for the DPLL to assert its phase lock ( FRACNPLL\_STAT.DSPHASE\_LOCK =1).

The DPLL phase lock can take few milliseconds to seconds. ADI recommends you use a non-blocking mode to generate an interrupt after the DPLL phase lock event.

11. Release the clock by setting (=1) the FRACNPLL\_CTL.CK\_STBL bit.

IMPORTANT: Programming the exact values of the FPLL and DPLL register settings for different FIN and FPLL output clock frequencies is not part of this chapter. The supported values of FIN and FPLL output clock frequencies and the required software routines to program the FPLL and DPLL are available using the FPLL Service along with the CrossCore Embedded Studio® software. ADI recommends you always use this service for programming the Frac-N PLL.

## DPLL Plus FPLL Mode TWAIT

CAUTION: ADI does not recommend attempting to use the TWAIT feature in the DPLL Plus FPLL Mode.

Because of the complicated programming model, using the TWAIT feature for the DPLL plus FPLL mode may require the trigger to arrive somewhere the middle of the previously mentioned Programming Model for DPLL Plus FPLL Mode steps. This means a user must add an interrupt to use the TWAIT feature.

## Programming the FIN Unlock and Re-lock DPLL

When the FIN is temporarily absent, the DPLL can become unlocked. Once the FIN clock signal enables, the DPLL attempts to lock again. To ensure a smooth DPLL unlock and relock sequence, use the following procedure:

1. Configure the DPLL to generate the unlock interrupt. See DPLL Unlock Interrupt for more information.
2. When the DPLL unlocks because of the FIN absence, it generates a DPLL unlock interrupt. Inside the unlock interrupt service routine, set (=1) the FRACNPLL\_DPLL\_CTL4.HOLD bit. Note: the application can use any other interrupt mechanism as well (for example, SPDIF receiver loss of lock interrupt if SPDIF receiver is the source of FIN) instead of DPLL unlock interrupt to signal loss of FIN.
3. DPLL cannot generate lock interrupt when in hold state. So, the application must have a way to interrupt the core when FIN is reconnected back (for example, SPDIF receiver lock interrupt if SPDIF receiver is the source of FIN). Inside the interrupt service routine, clear (=0) the FRACNPLL\_DPLL\_CTL4.HOLD bit to release DPLL hold.
4. Wait for the DPLL to assert a phase lock ( FRACNPLL\_STAT.DSPHASE\_LOCK =1) again. Use the DPLL phase lock interrupt in this case for the non-blocking mode.

## ADSP-2184x FRACNPLL Register Descriptions

The Fractional Phase-locked loop (FRACNPLL) contains the following registers.

Table 4-11: ADSP-2184x FRACNPLL Register List

| Name                | Description                  |
|---------------------|------------------------------|
| FRACNPLL_CTL        | Integration Control Register |
| FRACNPLL_DPLL_CTL0  | DPLL Control Register 0      |
| FRACNPLL_DPLL_CTL1  | DPLL Control Register 1      |
| FRACNPLL_DPLL_CTL2  | DPLL Control Register 2      |
| FRACNPLL_DPLL_CTL3  | DPLL Control Register 3      |
| FRACNPLL_DPLL_CTL4  | DPLL Control Register 4      |
| FRACNPLL_DPLL_CTL5  | DPLL Control Register 5      |
| FRACNPLL_DPLL_CTL6  | DPLL Control Register 6      |
| FRACNPLL_DPLL_CTL7  | DPLL Control Register 7      |
| FRACNPLL_DPLL_CTL8  | DPLL Control Register 8      |
| FRACNPLL_DPLL_CTL9  | DPLL Control Register 9      |
| FRACNPLL_DPLL_STAT0 | DPLL Status Register 0       |
| FRACNPLL_DPLL_STAT1 | DPLL Status Register 1       |
| FRACNPLL_DPLL_STAT2 | DPLL Status Register 2       |

Table 4-11: ADSP-2184x FRACNPLL Register List (Continued)

| Name               | Description                       |
|--------------------|-----------------------------------|
| FRACNPLL_FPLL_CTL0 | Fractional PLL Control Register 0 |
| FRACNPLL_FPLL_CTL1 | Fractional PLL Control Register 1 |
| FRACNPLL_STAT      | Integration Status Register       |

## Integration Control Register

The FRACNPLL\_CTL register controls DPLL reset, DFT modes, clock selections, and trigger wait and power down functions.

Figure 4-11: FRACNPLL\_CTL Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000010_43745c76974e5ce19daef15caea28d9b98667e50c36dfbd3a3388d3d6fbfc4f0.png)

Table 4-12: FRACNPLL\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 31 (R/W)           | DCRESET    | DPLL RESET Control. Resets all internal counters and registers to their default state               |
| 30 (R/W)           | FCPD       | Frac PLL PD Control. Power down for PLL                                                             |
| 24 (R/W)           | REFCLK_IE  | Reference Clock Enable. Input Enable for PLL REFCLK over GPIO                                       |
| 23:21 (R/W)        | LOCK_SEL   | FLOCK and F/PLOCK Selection Between Dand FracN PLL. FPLL FLOCK/PLOCK and DPLL FLOCK/PLOCK selection |

Table 4-12: FRACNPLL\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20:18 (R/W)        | DAICLK_SEL | DAI Clock Select. Selects the output of Frac-N PLL DAI clock multiplexer. 0 - No clock 1 - FOUTPOSTDIV/16 2 - FOUTPOSTDIV/8 3 - FOUTPOSTDIV/4 4 - FOUTPOSTDIV/2 5 - FOUTPOSTDIV        |
| 17:16 (R/W)        | FIN_SEL    | External Reference Clock Select. Multiplex selection for external reference clock. 0 - No clock 1 - Reference clock from DAI 2 - Reference clock from internal or external EMAC-PTPPPS |
| 15:14 (R/W)        | FIN_DFT    | FIN DFT Mode. DFT mode for external reference clock. 0 - No clock 1 - DAI or EMAC ref clock 2 - DFT ref clock                                                                          |
| 13:12 (R/W)        | FFB_DFT    | FFB DFT Mode. DFT mode for feedback clock. 0 - No clock 1 - Feedback clock form FPLL (FOUT1PH0) 2 - DFT feedback clock                                                                 |
| 11:10 (R/W)        | CLKS_DFT   | CLKS DFT Mode. DFT mode for CLKS. 0 - No clock 1 - CLKS form FPLL (CLKSSCG) 2 - DFT CLK                                                                                                |
| 9 (R/W)            | FDIV_OVR   | Fractional Divider Overdrive. Fractional divider override to FPLL in FPLL only mode                                                                                                    |
| 8 (R/W)            | D_TWAIT    | DPLL I/P Trigger Wait.                                                                                                                                                                 |
| 7 (R/W)            | DDIV_OVR   | Integer Divider Overdrive. Integer divider override to FPLL in FPLL only mode                                                                                                          |

Table 4-12: FRACNPLL\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | F_TWAIT    | Frac PLL Trigger Wait. Frac PLL I/P Trigger Wait                                                                                                                                         |
| 2:1 (R/W)          | CLKOUT_SEL | Frac-N PLL CLKOUT Clock Select. Selects the output of FPLL CLKOUT multiplexer which will be given as input to the main CLKOUT multiplexer. 1 - CLKSSCG 2 - FOUTPOSTDIV/64 3 - FOUTVCO/64 |
| 0 (R/W)            | CK_STBL    | Frac PLL CLK Stable. Setting the FRACNPLL_CTL.CK_STBL bit ensures sending out a stable clock by enabling clock gating control after the Frac-N PLL DAI clock multiplexer.                |

## DPLL Control Register 0

The FRACNPLL\_DPLL\_CTL0 register contains the DPLL count offset value used for frequency multiplication in the frequency control loop.

Figure 4-12: FRACNPLL\_DPLL\_CTL0 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000011_6e759701af849d135be54d0558db0c68409012a57c7dc046b8784fce6f4c8caa.png)

Table 4-13: FRACNPLL\_DPLL\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | CNTOFFSET  | DPLL CNTOFFSET Control. Allows for frequency multiplication in the frequency control loop. By setting CNTOFFSET to SYNCCNTMAX / K, where K is the multiplication ratio, the frequency detector will output a zero frequency error when CNTOFFSET pulses have been counted from the reference and SYNCCNTMAX pulses have been counted from the feedback. |

## DPLL Control Register 1

The FRACNPLL\_DPLL\_CTL1 register controls DPLL delay.

Figure 4-13: FRACNPLL\_DPLL\_CTL1 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000012_453f651ff4aa8604355d1b23ef195ee6696426bcf283ebd5a796979dedf83abf.png)

Table 4-14: FRACNPLL\_DPLL\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:0 (R/W)         | DELAYK     | DPLL DELAYK Control. The FRACNPLL_DPLL_CTL1.DELAYK bit field indicates the phase locking loop counts for DELAYK for each of PH_STEP_CNT gain settings fromM*2 K * 2 PH_STEP_CNT down toM*2 K . Total lock count is PH_STEP_CNT*DELAYK |

## DPLL Control Register 2

The FRACNPLL\_DPLL\_CTL2 register shows the DPLL integer divider for FIN and FFB.

Figure 4-14: FRACNPLL\_DPLL\_CTL2 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000013_4c038e8aece330c99565d00e1c5655ead481430dd9678a45fd8174cbe235c891.png)

Table 4-15: FRACNPLL\_DPLL\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DIVFIN     | DPLL DIVFIN Control. The FRACNPLL_DPLL_CTL2.DIVFIN bit field indicates the integer divider for FIN. |
| 15:0 (R/W)         | DIVFFB     | DPLL DIVFFB Control. The FRACNPLL_DPLL_CTL2.DIVFFB bit field indicates the integer divider for FFB. |

## DPLL Control Register 3

The FRACNPLL\_DPLL\_CTL3 register controls the gain for the integral frequency comparison loop and frequency operations.

Figure 4-15: FRACNPLL\_DPLL\_CTL3 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000014_611fa39daf5704cf80f70118519d1f12b7902b761f4c3b4be46bd6b06706f1e8.png)

Table 4-16: FRACNPLL\_DPLL\_CTL3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:6 (R/W)         | FMI        | DPLL FMI Control. The FRACNPLL_DPLL_CTL3.FMI bit field indicates the Mgain for the integral frequency comparison loop where Gain=M * 2 K .                                                        |
| 5:1 (R/W)          | FKI        | DPLL FKI Control. The FRACNPLL_DPLL_CTL3.FKI bit field indicates the K gain for the integral frequency comparison loop where Gain=M * 2 K .                                                       |
| 0 (R/W)            | FDONLY     | DPLL FDONLY Control. The FRACNPLL_DPLL_CTL3.FDONLY bit indicates whether test mode or normal operation is used. 0 - Frequency Control Only (Test Mode) 1 - Normal Operation (Frequency and Phase) |

## DPLL Control Register 4

The FRACNPLL\_DPLL\_CTL4 register controls the hold output state and FRAC output.

Figure 4-16: FRACNPLL\_DPLL\_CTL4 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000015_de8439b73dd37765012b35fb5c2b594413d849b80cde213c2bfe7c4b0e02fa3c.png)

Table 4-17: FRACNPLL\_DPLL\_CTL4 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                   |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | HOLD        | DPLL HOLD Control. The FRACNPLL_DPLL_CTL4.HOLD bit indicates the hold output state of the DPLL. 0 - Normal operation 1 - Hold. The PLL will not phase lock in this state. |
| 23:0 (R/W)         | FRAC_PRESET | DPLL FRAC_PRESET Control. The FRACNPLL_DPLL_CTL4.FRAC_PRESET bit field indicates the preset FRAC output. When PRESET_EN=1, the output is set to this value.               |

## DPLL Control Register 5

The FRACNPLL\_DPLL\_CTL5 register controls some diagnostic operations and ISHIFT fields.

Figure 4-17: FRACNPLL\_DPLL\_CTL5 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000016_321573ae5e9d0c04581114a3ff4ec71b48f6d5898b4837220df1456d0d5b6103.png)

Table 4-18: FRACNPLL\_DPLL\_CTL5 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                   |
|--------------------|-------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | ONTARGETOV  | DPLL ONTARGETOV Control. The FRACNPLL_DPLL_CTL5.ONTARGETOV bit indicates diagnostic control (test mode) or normal operation. 0 - Diagnostic Mode 1 - Normal Operation                                     |
| 19:16 (R/W)        | ISHIFT_HOLD | DPLL ISHIFT_HOLD Control. The FRACNPLL_DPLL_CTL5.ISHIFT_HOLD bit field indicates the ISHIFT val- ue used after exiting from HOLD Gain of 2^-ISHIFT from P-Path to I-Path integra- tor in phase lock mode. |
| 15:12 (R/W)        | ISHIFT      | DPLL ISHIFT Control. The FRACNPLL_DPLL_CTL5.ISHIFT bit field indicates gain of 2 -ISHIFT from P-Path to I-Path integrator in phase lock mode.                                                             |
| 11:0 (R/W)         | INT_PRESET  | DPLL INT_PRESET Control. The FRACNPLL_DPLL_CTL5.INT_PRESET bit field indicates the preset INT output. When PRESET_EN=1b1, the output is set to this value.                                                |

## DPLL Control Register 6

The FRACNPLL\_DPLL\_CTL6 register controls FFB clock delays.

Figure 4-18: FRACNPLL\_DPLL\_CTL6 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000017_b9bcd9183b63fde5c84846cc98cd1a73dfc0ee15a88ce9b38fdcb3676b8df9c5.png)

Table 4-19: FRACNPLL\_DPLL\_CTL6 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | OTDLY      | DPLL OTDLY Control. The FRACNPLL_DPLL_CTL6.OTDLY bit field indicates the delay in FFB clock periods from when FLOCK=1 to when the DPLL transfers control from the frequency comparison loop to the phase comparison loop. The delay allows additional settling and avoids glitches in FLOCK output. The recommended value is 25. |

## DPLL Control Register 7

The FRACNPLL\_DPLL\_CTL7 register controls DPLL bits used for diagnostics and gain functions.

Figure 4-19: FRACNPLL\_DPLL\_CTL7 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000018_f002d28085d1fcf14256a05258c85da4ac96a323804a42b87a08537b3f4e6c31.png)

Table 4-20: FRACNPLL\_DPLL\_CTL7 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | PROGRAM    | DPLL PROGRAM Control. The FRACNPLL_DPLL_CTL7.PROGRAM bit indicates diagnostic control (test mode) or normal operation. 0 - Diagnostic Mode 1 - Normal Operation                                              |
| 23 (R/W)           | PRESET_EN  | DPLL PRESET_EN Control. The FRACNPLL_DPLL_CTL7.PRESET_EN bit indicates whether the preset INT and FRAC values are loaded into DPLL. 0 - Normal Operation 1 - Switch to FRAC_PRESET and INT_PRESET and relock |
| 22:15 (R/W)        | PMP        | DPLL PMP Control. The FRACNPLL_DPLL_CTL7.PMP bit field indicates the Mgain for the propor- tional phase comparison loop where Gain=M * 2 K .                                                                 |
| 14:10 (R/W)        | PKP        | DPLL PKP Control. The FRACNPLL_DPLL_CTL7.PKP bit field indicates the K gain for the propor- tional phase comparison loop where Gain=M * 2 K .                                                                |

Table 4-20: FRACNPLL\_DPLL\_CTL7 Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:5 (R/W)          | PH_STEP_CNT_HOLD | DPLL PH_STEP_CNT_HOLD Control. The FRACNPLL_DPLL_CTL7.PH_STEP_CNT_HOLD bit field indicates the PH_STEP_CNT value used after exiting from HOLD. Number of steps in phase lock process. Initial gain is Gain_init=M*2 K * 2 PH_STEP_CNT . Total lock count is PH_STEP_CNT*DELAYK. |
| 4:0 (R/W)          | PH_STEP_CNT      | DPLL PH_STEP_CNT Control. The FRACNPLL_DPLL_CTL7.PH_STEP_CNT bit field indicates the number of steps in phase lock process. Initial gain is Gain_init=M*2 K * 2 PH_STEP_CNT . Total lock count is PH_STEP_CNT*DELAYK                                                            |

## DPLL Control Register 8

The FRACNPLL\_DPLL\_CTL8 register controls inputs for FIN pulses.

Figure 4-20: FRACNPLL\_DPLL\_CTL8 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000019_02ed7b4a97ef68700a9e8c3f2b7d7dc75bcb039a6360740c1615da412a5b7e28.png)

Table 4-21: FRACNPLL\_DPLL\_CTL8 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | SYNCCNTMAX | DPLL SYNCCNTMAX Control. FIN pulses are counted for FRACNPLL_DPLL_CTL8.SYNCCNTMAX to determine a frequency match. |

## DPLL Control Register 9

The FRACNPLL\_DPLL\_CTL9 register holds the target pulse count used for frequency comparison.

Figure 4-21: FRACNPLL\_DPLL\_CTL9 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000020_ee5913a7ec4ca785dd37b8dace64fbe483d773fab99b113ddde22b84171d34f3.png)

Table 4-22: FRACNPLL\_DPLL\_CTL9 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TARGETCNT  | DPLL TARGETCNT Control. The FRACNPLL_DPLL_CTL9.TARGETCNT bit field indicates whether the num- ber of pulse counts between FIN and FFB is less than TARGETCNT. The frequency comparator declares a frequency match (When FLOCK=1). |

## DPLL Status Register 0

The FRACNPLL\_DPLL\_STAT0 register contains the fractional portion of the feedback divide value.

Figure 4-22: FRACNPLL\_DPLL\_STAT0 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000021_aeda3d912d16ab51f29d7dab6aaa6ac990beeac9f5c31845daa4ed2ccc2b70df.png)

Table 4-23: FRACNPLL\_DPLL\_STAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------|
| 23:0 (R/NW)        | FRAC       | DPLL FRAC Status. The FRACNPLL_DPLL_STAT0.FRAC bit field indicates the fractional portion of feedback divide value. |

## DPLL Status Register 1

The FRACNPLL\_DPLL\_STAT1 register contains the fractional portion of the phase detector integrator output.

Figure 4-23: FRACNPLL\_DPLL\_STAT1 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000022_d45dc53bddd37b381a891a491310745cdf353ae253a5282e10614f827d3965a4.png)

Table 4-24: FRACNPLL\_DPLL\_STAT1 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                       |
|--------------------|-------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 23:0 (R/NW)        | FRAC_PD_OUT | DPLL FRAC_PD_OUT Status. The FRACNPLL_DPLL_STAT1.FRAC_PD_OUT bit field indicates the fractional bits of the phase detector integrator output. |

## DPLL Status Register 2

The FRACNPLL\_DPLL\_STAT2 register contains the integer portion of the phase detector integrator output.

Figure 4-24: FRACNPLL\_DPLL\_STAT2 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000023_7ed4d435163e5511437e8f2ea09f89e561140ab71d31acae571f04fde83dda14.png)

Table 4-25: FRACNPLL\_DPLL\_STAT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 23:12 (R/NW)       | INT_PD_OUT | DPLL INT_PD_OUT Status. The FRACNPLL_DPLL_STAT2.INT_PD_OUT bit field indicates the integer bits of the phase detector integrator output.     |
| 11:0 (R/NW)        | INT        | DPLL INT Status. The FRACNPLL_DPLL_STAT2.INT bit field indicates the integer divider for Frac- NPLL, clocked out on the rising edge of CLKS. |

## Fractional PLL Control Register 0

The FRACNPLL\_FPLL\_CTL0 register contains the fractional divider setting for Fractional-N PLL, various power down controls, and bypass control.

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000024_51ce5549ba27f52c9f1d9a135058b9713c7732fc04ae0174bc97a250b630eac1.png)

Fractional PLL FRAC Control

Figure 4-25: FRACNPLL\_FPLL\_CTL0 Register Diagram

Table 4-26: FRACNPLL\_FPLL\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                             |
|--------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:6 (R/W)         | FRAC          | Fractional PLL FRAC Control. Fractional divider setting for Fractional-N PLL, clocked out on the rising edge of CLKS                                |
| 5 (R/W)            | FOUTVCOPD     | Fractional PLL FOUTVCOPD Control. VCO rate output clock power down                                                                                  |
| 4 (R/W)            | FOUTPOSTDIVPD | Fractional PLL FOUTPOSTDIVPD Control. Post divide power down                                                                                        |
| 3 (R/W)            | FOUT4PHASEPD  | Fractional PLL FOUT4PHASEPD Control. Power down of 4 phase clock generator; 4phase output is also powered down by FOUTPOSTDIVPD                     |
| 2 (R/W)            | DSMPD         | Fractional PLL DSMPD Control. Power down Delta-Sigma Modulator; 0-> DSM is active; 1-> DSM is powered down                                          |
| 1 (R/W)            | DACPD         | Fractional PLL DACPD Control. Power down noise canceling DAC in FRAC mode; 0-> DAC is active (default mode); 1-> DAC is not active (test mode only) |

Table 4-26: FRACNPLL\_FPLL\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 0                  | BYPASS     | Fractional PLL BYPASS Control.  |
| (R/W)              |            | FREF is bypassed to FOUTPOSTDIV |

## Fractional PLL Control Register 1

The FRACNPLL\_FPLL\_CTL1 register contains the reference divide value, post divide setting, and feedback divide value.

Figure 4-26: FRACNPLL\_FPLL\_CTL1 Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000025_43632ed483d3b26164fb67b98d573ea903e32185a0c14b5b31041b8cbaae395e.png)

Table 4-27: FRACNPLL\_FPLL\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 24:19 (R/W)        | REFDIV     | Frac PLL REFDIV Control. Reference divide value (1 to 63)            |
| 18:12 (R/W)        | POSTDIV    | Fractional PLL POSTDIV Control. PLL post divide setting (1 to 127).  |
| 11:0 (R/W)         | INT        | Fractional PLL FBDIV Control. PLL feedback divide value (16 to 4095) |

## Integration Status Register

The FRACNPLL\_STAT status register contains phase, frequency lock status of FPLL and DPLL along with DPLL transition status.

Figure 4-27: FRACNPLL\_STAT Register Diagram

![Image](07_Fractional_PLL_(Frac‑N_PLL)_artifacts/image_000026_40a6168efb49c132ad98292989cd63eac26aec420d93ec5c112836876bb3abff.png)

Table 4-28: FRACNPLL\_STAT Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | FS_PLOCK     | FPLL Phase Lock Status. The FRACNPLL_STAT.FS_PLOCK bit indicates whether the FPLL has achieved a phase lock.                                                                                                                                                                                                                 |
| 3 (R/NW)           | FSLOCK       | FPLL Frequency Lock Status. The FRACNPLL_STAT.FSLOCK bit indicates whether the FPLL has achieved a frequency lock.                                                                                                                                                                                                           |
| 2 (R/NW)           | DSTRANSITION | DPLL Transition Status. The FRACNPLL_STAT.DSTRANSITION bit indicates the control transition out- put. Test Mode Output. When FRACNPLL_STAT.DSTRANSITION = 0, it occurs after transition to fine tune phase control (PHASE LOCK=1b1), When FRACNPLL_STAT.DSTRANSITION = 1, it occurs after phase control engaged (FLOCK=1b1). |
| 1 (R/NW)           | DSPHASE_LOCK | DPLL Phase Lock Status. The FRACNPLL_STAT.DSPHASE_LOCK bit indicates whether the DPLL is in fine tune phase tracking mode. When FRACNPLL_STAT.DSPHASE_LOCK = 1, the DPLL is settled and phase locked. When FRACNPLL_STAT.DSPHASE_LOCK = 0, the DPLL is still settling or (if FLOCK=1b0) not frequency locked.                |

Table 4-28: FRACNPLL\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | DSFLOCK    | DPLL Frequency Lock Status. The FRACNPLL_STAT.DSFLOCK bit indicates whether the DPLL has achieved a frequency lock. When FRACNPLL_STAT.DSFLOCK = 1, the frequencies have maintained lock within TARGETCNT for OTDLY FFB clock periods When FRACNPLL_STAT.DSFLOCK = 0, the frequency comparator has detected a mismatch in FIN and FFB pulse counts. |