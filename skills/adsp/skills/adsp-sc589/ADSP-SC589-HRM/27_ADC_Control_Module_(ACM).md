## 24   ADC Control Module (ACM)

The processor includes an ADC control module (ACM) that provides an interface that synchronizes the controls between the processor and an analog-to-digital converter (ADC). The processor initiates analog-to-digital conversions, based on either external or internal events.

Traditionally, ADC sampling uses processor interrupts and the interrupt service routine programming of the appropriate peripheral for initiating the ADC conversion process. Events initiate the interrupts. The interrupt service routine usually programs the SPORT or SPI peripherals. This traditional approach has some limiting factors:

- The ADC sampling instances are not precisely controlled due to interrupt latencies (which can vary) or due to variable instruction execution cycles
- Consumption of processor instruction cycles can be prohibitive, especially for high frequency of conversionrelated events.
- If the ADC requires control signals with specific set-up, hold, or zero time for sampling time, it is difficult to provide the signals with GP flags in the application. For example, channel select pins, ADC mode select, ADC range pin.

The ADC control module (ACM) provides dedicated hardware to work around these limitations. The module samples the events and provides sampling signals and timing to the ADC in real time. The ADC permits flexible scheduling of sampling instants and provides precise sampling signals to the ADC. The ACM saves processor bandwidth and provides precise control for ADC sampling time. Furthermore, the processor can be interfaced directly to multiple ADCs without any glue logic required.

The ACM synchronizes the ADC conversion process (by providing the ADC clock, the ADC conversion start signal, and related ADC controls). However, other peripherals such as SPORT acquire the actual data from the ADC. The processor does not support ACM operation with the SPI. The ADC/SPORT Interface figure shows how an external ADC can be interfaced using the ACM and SPORT0 peripherals of the processor.

Figure 24-1: ADC/SPORT Interface

NOTE: The processor does not include an on-chip, internal ADC.

## ACM Features

The ADC control module (ACM) offers the following features and capabilities:

- The ACM on the ADSP-SC58x/ADSP-2158x processors operate on the SCLK0\_0 domain.
- The ACM can interface to the ADC at a maximum clock frequency of 30 MHz.
- It provides serial clock, chip select, and five general-purpose control lines capable of controlling the ADC operations. Internally routes serial clock and frame sync (chip select) signals to serial port 0.
- Only SPORT0 performs actual data acquisition from the ADC (no other SPORT instance). The ACM\_CLK and ACM\_FS signals are multiplexed with the SP0\_CLK and SP0\_FS signals. These signals are driven through the DAI0 and not through the GPIO port. The SPORT uses the ACM\_CTL.EN bit to control the connection between SPORT0 and the ACM. By setting the ACM\_CTL.EN bit, the ACM clock and FS are internally multiplexed to the SPORT0 SPORT0\_CLK\_O/SPORT0\_FS\_O outputs of the DAI0.
- The ACM can accept three trigger inputs (one from external on ACM\_T0 and two from the TRU) based on which it can precisely initiate the ADC sampling events. The trigger inputs can be internally generated or externally supplied. The polarity of trigger inputs is configurable.
- The ACM can handle 16 ADC sampling events per valid trigger received. Each event can be independently programmed to specify when to initiate ADC sampling for a trigger input.
- Two independent 32-bit ACM timers that can be used to divide 16 events into two groups of 8 events.
- Automatically stops the ACM timer after completion of associated events, saving power.
- Four-deep pending FIFO to queue the active events when the ACM is busy.
- The ACM can internally generate serial clock up to SCLK ÷ 2 rate. Improved granularity for internal clock generation, allowing both odd and even SCLK:ACLK ratios.

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000000_1c2223846e9a899e3307e2d23f1cb18cc6b39f2e4902a203fb9563274d847919.png)

- The ACM clock can be gated (active only during enabled events) to interface it with SPI-compatible ADCs.
- When initiating an ADC sampling cycle, the width of chip select signal can be configured from 1 ACLK to 256 ACLKs. ADC can also use the width of chip select signal as start of conversion. Further the polarity of this signal can be configured as active-high or active-low signal.
- Auto ACLK adjustment at the time of CS assertion. After assertion of the CS signal, the first edge of ACLK can be configured to be either rising edge or falling edge.
- The ACM provides the five general-purpose control lines that can be programmed for required set-up and hold time based on the ADC sampling cycle. Additionally, zero time can be inserted between two successive sampling cycles.
- The ACM provides 16 event order registers (one per each event) which indicate the order in which events are handled. Optionally, the trigger input of the ACM can clear these registers automatically.
- The ACM hardware flags the appropriate event completion status bit on completion of an event. If an event is missed, the appropriate event missed status bit is flagged. Each event has separate bits. Optionally, the event completion interrupt and event missed interrupt can be triggered on these respective conditions.
- Predictable latency between the internal occurrence of an event and the assertion of a sampling event.
- The ACM can operate as trigger master to provide signal to TRU upon completion of events.

## ACM Functional Description

The ADC control module uses internal ACM timers and the event time register to create events. Enable one of the timers (or both timers) for the ACM operation. Program the appropriate event control register and event time register values. After receiving a valid trigger on the selected trigger input, the timer starts counting. When the timer count matches the time specified in the event time register ( ACM\_EVTIME[n] ), the comparators generate an active event signal to the timing generation unit. The signal starts the ADC access. The event must be enabled and the event must be associated with the timer. The counter continues counting, and for each match with enabled event time, the ACM gives an event signal to the timing generation unit.

## ADSP-SC58x ACM Register List

The ADC control module (ACM) provides an interface that synchronizes the controls between the processor and an analog-to-digital converter (ADC). The analog-to-digital conversions are initiated by the processor, based on external or internal events. A set of registers govern ACM operations. For more information on ACM functionality, see the ACM register descriptions.

Table 24-1: ADSP-SC58x ACM Register List

| Name         | Description             |
|--------------|-------------------------|
| ACM_CTL      | Control Register        |
| ACM_EVCTL[n] | Event NControl Register |

Table 24-1: ADSP-SC58x ACM Register List (Continued)

| Name          | Description                            |
|---------------|----------------------------------------|
| ACM_EVMSK     | Event Complete Interrupt Mask Register |
| ACM_EVORD[n]  | Event NOrder Register                  |
| ACM_EVSTAT    | Event Complete Status Register         |
| ACM_EVTIME[n] | Event NTime Register                   |
| ACM_MEVMSK    | Missed Event Interrupt Mask Register   |
| ACM_MEVSTAT   | Missed Event Status Register           |
| ACM_STAT      | Status Register                        |
| ACM_TC0       | Timing Configuration 0 Register        |
| ACM_TC1       | Timing Configuration 1 Register        |
| ACM_TMR0      | Timer 0 Register                       |
| ACM_TMR1      | Timer 1 Register                       |

## ADSP-SC58x ACM Interrupt List

Table 24-2: ADSP-SC58x ACM Interrupt List

|   Interrupt ID | Name              | Description         | Sensitivity   | DMA Channel   |
|----------------|-------------------|---------------------|---------------|---------------|
|             36 | ACM0_EVT_MISS     | ACM0 Event Miss     |               |               |
|             37 | ACM0_EVT_COMPLETE | ACM0 Event Complete |               |               |

## ADSP-SC58x ACM Trigger List

Table 24-3: ADSP-SC58x ACM Trigger List Masters

|   Trigger ID | Name              | Description         | Sensitivity   |
|--------------|-------------------|---------------------|---------------|
|          132 | ACM0_EVT_COMPLETE | ACM0 Event Complete |               |

Table 24-4: ADSP-SC58x ACM Trigger List Slaves

|   Trigger ID | Name       | Description          | Sensitivity   |
|--------------|------------|----------------------|---------------|
|          135 | ACM0_TRIG2 | ACM0 Trigger Input 2 | Pulse         |
|          136 | ACM0_TRIG3 | ACM0 Trigger Input 3 | Pulse         |

## ACM Event Handling Latency

The ACM ensures a predictable latency between the internal occurrence of an event and the assertion of a sampling event by the timing generation unit (for example, the assertion of CS and other ACM signals). The internal occurrence of an event is when the event time value matches the ACM timer count value.

Latency between occurrence of an event to CS assertion = (t S  + t ED ) SCLK cycles, where:

- t S = ADC control setup cycles programmed in the ACM\_TC0 register
- t ED = 1 SCLK0\_0 cycle latency

This predictable latency is applicable only when events are generated when the timing generation unit is idle. If this unit was processing a prior sampling event, the new event is held in the pending event FIFO. The duration that the new event is held in the pending event FIFO increases the latency.

If an external trigger input is selected as a trigger input, then synchronization to this signal leads to a 3 SCLK0\_0 cycle fixed delay and 1 SCLK0\_0 cycle variability. This result is due to delays in latching asynchronous external triggers. When the external trigger is synchronous to SCLK0\_0, the 1 SCLK0\_0 cycle variability is eliminated. The latency from the external trigger to the start of the count of an ACM timer becomes fixed at 3 SCLK0\_0 cycles. This latency is denoted as t TRIG .

As a result, the total latency between an external trigger and the assertion of an ADC sampling event is:

Total latency = t TRIG + t ED  + t PD + t S

The latency calculation assumes that the sampling event is not queued in the pending event FIFO. The Latency of External Triggers to Assertion of ADC Sampling Events figure shows latency details from the occurrence of external triggers to the assertion of ADC sampling events.

Observe the following timing definitions:

- t TRIG = trigger to timer start delay (3-4 SCLK0\_0 cycles)
- t PD = event time (programmed in the ACM\_EVTIME[n] register of the event)
- t ED = internal event delay (1 SCLK0\_0 cycle)
- t S  = setup time (programmed in the ACM\_TC0 register)
- t CSW = CS width (programmed in the ACM\_TC1.CSW bit field)
- t H = hold time (programmed in the ACM\_TC1 register)

Figure 24-2: Latency of External Triggers to Assertion of ADC Sampling Events

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000001_f037338975c56d203f6e4dcfd43926e81d61224e15a74fab60584e2f896a341e.png)

## ACM Timing Specifications

The AC timing of the ACM signals is specified in the product-specific data sheet. When trigger sources external to the processor are used for triggering the ACM, the minimum pulse width for the sources must be greater than one SCLK0\_0 period. This pulse width is the minimum needed for the ACM trigger logic to detect the source as a valid trigger (for example, external signals on the GPIO, timer, or PWM sync pins).

- When the processor uses the ACM with the SPORTs, the requirements for the SPORT data signals vary. The setup and hold time requirements for the SPORT data signals for the SPORT\_ACLK signal differ from those requirements for an internally-generated or externally-supplied SPORT clock. Consult the product-specific data sheet for information on these timing requirements.

When using gated clock mode ( ACM\_CTL.CLKMOD =1), configure the interfaced serial mode in gated clock mode ( SPORT\_CTL\_A.GCLKEN , SPORT\_CTL\_B.GCLKEN =1). In this case, some conditions must be satisfied to set up the serial port in gated clock mode. These conditions are:

- The serial port needs at least 7 serial clock cycles between enabling the SPORT and first frame sync. If this requirement is not met, the SPORT may drop the first data. (For subsequent data, this requirement is not applicable.)
- Set the frame sync to the inactive (deasserted) state when the SPORT is enabled. Otherwise, one extra cycle (in addition to the cycle mentioned) is needed before the frame sync can be applied. If this requirement is not met, the SPORT may drop the first data.

## ACM External Pin Timing

The ACM clock ( ACM\_CLK ) is derived internally from SCLK0\_0 using the ACM\_TC0.CKDIV divider. The other output signals, such as the ADC control pins ( ACM\_A0 through ACM\_A4 ) and the chip select are driven on the rising edge of SCLK0\_0. As a result, these signals cannot be synchronous to the ACM\_CLK signal.

The ACM uses timing configuration registers ( ACM\_TC0 and ACM\_TC1 ) to configure setup, hold, and other timing parameters of the ADC control signals, the width of the chip select signal, and the frequency of ACM\_CLK . The

polarity of CS and the ACM\_CLK signals can be configured in the ACM control register ( ACM\_CTL ). The timing parameters of the ADC control pins cannot be individually specified.

The Timing Reference figure shows the inactive period of CS as t CSIW . The inactive period of CS is the sum of the three timing parameters; Setup time (t s ), Zero time (t z ) and the Hold time (t H ):

t CSIW = t S + t Z + tH.

Figure 24-3: Timing Reference

Proper specification of the values of these three parameters yields the desired inactive period of CS.

The ADC provides a predictable latency from the occurrence of an internal event to the assertion of an external ADC sampling event. The ADC controls and drives the CS signal on the rising edge of SCLK0\_0. Therefore, the setup time (T s ) of these signals is specified in terms of SCLK0\_0. However, the hold-time and zero-time are specified in terms of ACM\_CLK cycles.

To achieve an accurate timing relationship between the CS and ACM\_CLK (which is normally a free running clock) signals, the ACM\_CLK signal is realigned with the active edge of CS. This realignment of the ACM\_CLK signal ensures that the setup time of the first active edge of ACM\_CLK , relative to the active edge of CS, is at least 1 ACM\_CLK cycle.

The following set of figures in the cases show various scenarios of ACM\_CLK realignment. All of these figures assume an ACM\_CLK:SCLK ratio of 1:4.

- Case 1-Chip select asserted during the high phase of ACM\_CLK
- Case 2-Chip select asserted during the low phase of ACM\_CLK
- Case 3-Chip select asserted right before the falling edge of ACM\_CLK
- Case 4-Chip select asserted right before the rising edge of ACM\_CLK
- Case 5-ACM\_CLK polarity set to 1 ( ACM\_CTL.CLKPOL =1)

Realignment of the ACM clock causes the suppression or extention of clock edges, leading to duty cycle variation. Ensure that systems interfacing with the ACM can tolerate such duty cycle variation.

The set of figures shows both the ACM-generated CS signal, which is output externally onto the appropriate ACM\_FS pin, and the serial port receive frame sync signal. This internal signal is routed to the frame sync input of the appropriate SPORT.

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000002_651256e1d1260c97406d9b11381455b372fedec82b8f69dd50ab2335556274ab.png)

The ACM clock polarity can be configured using the ACM\_CTL.CLKPOL bit. After the CS signal is asserted, the first edge of ACM\_CLK can be configured to be either rising edge or falling edge. Also, by default the clock is free running; it is possible to gate the ACM clock during an inactive CS period using the ACM\_CTL.CLKMOD bit.

## Case 1-Chip Select Asserted During the High Phase of ACM\_CLK ( ACM\_CTL.CLKPOL =0)

The Chip Select Asserted During the High Phase of ACM\_CLK figure shows the realignment of ACM\_CLK when CS is asserted during the high phase of ACM\_CLK . The first edge of ACM\_CLK , after the assertion of CS, is the falling edge.

The two reference clock signals (REF ACLK1 and REF ACLK2) illustrate how the ACM\_CLK signal can be generated from a free running clock (REF ACLK1). This setup meets the timing requirements between the ACM\_CLK and CS signals. The REF ACLK2 signal is based on the free running clock REF ACLK1. However, the REF ACLK2 signal is adjusted such that its period is immediately reset upon the assertion of CS. In the resulting ACM\_CLK signal, the time from the active edge of CS to the falling edge of ACM\_CLK is constant at a period of 1 ACM\_CLK cycle. See ACM\_CLK in the Chip Select Asserted During the High Phase of ACM\_CLK figure.

Figure 24-4: Chip Select Asserted During the High Phase of ACM\_CLK

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000003_b7764edff741700673c6958200c792740adeb05297d6247a179389afa5cd5b40.png)

## Case 2-Chip Select Asserted During the Low Phase of ACM\_CLK ( ACM\_CTL.CLKPOL =0)

Refer to the Chip Select Asserted During the Low Phase of ACM\_CLK figure. When the CS signal is asserted during the low phase of ACM\_CLK , ACM\_CLK is immediately pulled high which causes a duty cycle variation. In this case, (similar to Case 1), the time from the active edge of CS to the falling edge of ACM\_CLK is 1 ACM\_CLK period.

Figure 24-5: Chip Select Asserted During the Low Phase of ACM\_CLK

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000004_3b9bf86fb900cec69ad084a0ab963df7b604254a87b24308cbf4c87e1e7df043.png)

## Case 3-Chip Select Asserted Right Before the Falling Edge of ACM\_CLK ( ACM\_CTL.CLKPOL =1)

Refer to the Chip Select Asserted Right Before the Falling Edge of ACM\_CLK figure. When CS is asserted right before the falling edge of ACM\_CLK , the falling edge of ACM\_CLK is suppressed. This functionality ensures that the time from the active edge of CS to the falling edge of ACM\_CLK is constant at a period of 1 ACM\_CLK cycle.

Figure 24-6: Chip Select Asserted Right Before the Falling Edge of ACM\_CLK

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000005_fa7d5fd36ec3fae758af9721cb04a5aca9e3f5f3e55aa32d3b5764a71aec0b10.png)

## Case 4-Chip Select Asserted Right Before the Rising Edge of ACM\_CLK ( ACM\_CTL.CLKPOL =0)

Refer to the Chip Select Asserted Right Before the Rising Edge of ACM\_CLK figure. When CS is asserted right before the rising edge of ACM\_CLK , the high phase of ACM\_CLK is extended. This extension ensures that the time from the active edge of CS to the falling edge of ACM\_CLK is constant at a period of 1 ACM\_CLK cycle.

Figure 24-7: Chip Select Asserted Right Before the Rising Edge of ACM\_CLK

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000006_f17455685cf8c1b7d2b181d6a99a30b46caa0d70495a1387b89b570a1c44d510.png)

## Case 5-ACM\_CLK Polarity Set to 1 ( ACM\_CTL.CLKPOL =1)

When the ACM\_CLK polarity is set to 1 (the ACM\_CTL.CLKPOL bit is set to 1), the first ACM\_CLK edge after the assertion of CS is the rising edge. The ACM ensures that the time from the active edge of CS to the rising edge of ACM\_CLK has a constant duration of 1 ACM\_CLK cycle. The Polarity Set to 1 figure shows an example diagram of the case where ACM\_CTL.CLKPOL =1.

Figure 24-8: Polarity Set to 1

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000007_35eb7daf253c31d44adb2361735dc503e7bb689042c5a3ad4d2c2bffdc396c10.png)

## ACM Architectural Concepts

The following sections provide information on the architecture of the ACM module.

## Clocking

The ACM on the ADSP-SC58x/ADSP-2158x processors operate on the SCLK0\_0 domain. The term SCLK in this chapter is a generic reference to SCLK0\_0. For more details on SCLK0\_0 programming refer to CDU Functional Description.

## Block Diagram

The ADC Control Module (ACM) consists of two independent 32-bit ACM timers, 16 event register pairs, 16 event comparators and a timing generation unit.

The ACM can accept four trigger inputs (internal as well as external signals). On receiving a valid trigger on selected trigger input, the ACM timer/s start counting (based on the mode of the ACM). The trigger input can be independently selected for each timer.

Two sets of eight event register pairs (a total of 16 event register pairs) determine the ADC controls for each ADC sample. The registers also determine when the sample occurs. The event register pair consists of the event control register ( ACM\_EVCTL[n] ) and the event time register ( ACM\_EVTIME[n] ). The event comparators unit compares the timer of the ACM count with the event time of associated enabled events. When the count matches, the event comparators unit signals to the timing generation unit. The unit starts handling the events by driving the CS and ACM\_A4 to ACM\_A0 signals accordingly.

The ACM Block Diagram shows the structure of the ACM. The following sections discuss these blocks in detail.

Figure 24-9: ACM Block Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000008_21592135e345203b472fdd3726cc5a1b3e24c57de643f73b3238d82de7ad9d29.png)

## Trigger Inputs

The ACM can accept four trigger inputs, based on which the ACM timers start running at the SCLK0\_0 rate. The ACM contains three 32-bit internal timers (timer 0, timer 2, and timer 3). Each timer can be independently configured to use one of these trigger inputs. The ACM uses ACM\_CTL.TRGSEL0 and ACM\_CTL.TRGSEL1 bit fields to select the trigger input for Timer0 and Timer1 respectively.

Out of the 4 trigger inputs to the ACM design:

- ACM\_T0 is driven from the chip I/O
- ACM\_T2 and ACM\_T3 are driven from the TRU

The ACM uses two trigger inputs when both ACM timers are enabled for different trigger inputs. However, it uses only one trigger if both ACM timers are enabled for same trigger input or if a single ACM timer is enabled. The non-selected trigger inputs are do-not-care for the ACM. At most two, and at least one selected trigger input must be active in the system for the ACM to start operation.

The following list briefly describes the possible trigger inputs:

- Trigger input 0 ( ACM\_T0 ) - PE8:

Trigger input 0 is sourced from the PE8 pin of the GPIO port E (sometimes also referred as GPIO[72]). When the ACM is enabled, the input tap on the PE8 pin is enabled. The ACM trigger input can be from any source (internal or external) depending on the PE8 pin configuration.

When the PE8 pin is configured in GPIO mode ( PORT\_FER =0), the source of the GPIO signal can be either internal or external. The source depends on the GPIO direction, configured in the E PORT\_DIR register. When the PE8 pin is configured in function mode ( PORT\_FER =1), the trigger 0 input is sourced in from peripheral signals. The source is based on the PORT\_MUX register setting for the PE8 pin. For example, the ACM can source this trigger input from the PWM\_SYNC signal if the multiplex bits for the PE8 pin are set to b#00. The PWM unit internally generates the PWM\_SYNC signal or it can be provided externally. If the multiplex bits for PE8 are configured to b#01, the trigger input is sourced from the EPPI\_FS1 signal. The EPPI0 unit internally generates this signal or it can be generated externally.

Consider the source of the trigger input when programming the PORT\_FER.PX8 and PORT\_MUX.MUX8 bits for the PE8 pin. Enabling the input tap ensures that the ACM operation does not interfere with the module driving the PE8 pin.

- Trigger input 2 ( ACM\_T2 ) - (PG5) - ACM Slave TriggerID 135:

The trigger routing unit (TRU) of the processor provides system-level sequence control without core intervention. When this trigger input mode is selected, the ACM acts as a trigger slave and accepts a trigger through the trigger slave ID-135. The slave-select ( TRU\_SSR[n].SSR ) field of the SSR 135 register can be configured to receive triggers from a specific trigger master. In this way, ACM\_T2 trigger input can accept triggers asserted by that particular trigger master or through software. The software writes the ID of that trigger master to one of the four fields in the TRU\_MTR register. The trigger response from selected master is internally routed to ACM trigger input. This way, ACM slave trigger ID 135 can receive any one of the 86 internal triggers available.

- Trigger input 3 ( ACM\_T3 ) - ACM Slave TriggerID 136:

Similar to the ACM\_T2 input, the TRU of the processor provides the ACM trigger input 3 ( ACM\_T3 ) internally. When this trigger input mode is selected, the ACM acts as trigger slave and accepts a trigger through trigger slave ID 136. The slave-select ( TRU\_SSR[n].SSR ) field of the SSR 136 register can be configured to receive triggers from a specific trigger master. In this way, the ACM slave trigger ID 136 can accept triggers asserted by that particular trigger master or through software. The software writes the ID of that trigger master to one of the four fields in the TRU\_MTR register. The trigger response from selected master is internally routed to the ACM trigger input. This way, ACM slave trigger ID 136 can receive any one of the 86 internal triggers available.

See the Trigger Routing Unit (TRU) chapter for more details about trigger slaves and trigger masters.

For all trigger input signals, the active edge of the trigger is programmable in the ACM control register as either rising edge or falling edge trigger.

The Detailed ACM Trigger Generation Logic figure shows the detailed ACM trigger generation logic.

Figure 24-10: Detailed ACM Trigger Generation Logic

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000009_eb959810f49d3c81b29aef9ace9e8d567494a6734e518cdfa18c08fee0e18466.png)

When the processor uses external trigger sources external for triggering the ACM timers, the minimum pulse width for these sources must be greater than one SCLK0\_0 period. (For example, ACM\_T1 : ACM\_T0 ).

NOTE: A latency of no more than four SCLK0\_0 cycles exists between external trigger and ACM timer starts counting. Refer to the ACM Event Handling Latency section for further details.

## Timers

The ACM module has two independent 32-bit timers (ACMTMR0 and ACMTMR1) which start counting at the system clock (SCLK0\_0) rate when a valid trigger is detected on a selected trigger input. The timers are independently enabled using the timer enable bits ( ACM\_CTL.TMR0EN , ACM\_CTL.TMR1EN ). The timers can be independently configured for one of the four trigger inputs with configurable polarity of the signal. Enable at least one ACM timer for proper ACM operation.

By default, each ACM timer has 8 events associated with it. If both timers are enabled, Event[7:0] are associated with ACMTMR0 while Event[15:8] are associated with ACMTMR1. However, if only one timer is enabled, all of the event registers are associated with that particular timer. For example, if only ACMTMR1 is enabled (if ACM\_CTL.TMR0EN =0 and ACM\_CTL.TMR1EN =1), ACMTMR1 handles all 16 events, Event[15:0].

The timers start counting when a trigger input occurs that is selected for that particular timer. The timer only stops counting under one of the following conditions:

1. A timer rollover occurs.
2. All the events associated with the trigger have completed.

A timer rollover cannot occur unless the event time register of an event is programmed at some point after the trigger occurs. This practice is contrary to ACM programming guidelines.

In the second case, the exact time at which the timer stops counting depends on the FIFO state when the last event occurred internally.

If a trigger occurs while the timer is counting, the time resets and starts counting again. In this case, some of the events can miss which results in flagging the appropriate status bit and optionally an event missed interrupt.

When an ACM timer is disabled or the ACM itself is disabled, the timer resets to zero.

## Event Register Pairs

An event for the ACM is a point in time where ADC sampling occurs on a particular channel of the ADC with the specified control settings of the ADC.

The ACM can handle a total of 16 events which are grouped into two sets of 8 events. The 8 events can be assigned to each of the timers (if both timers are enabled) or all 16 events can be assigned to one particular timer. (Assignment happens when only one timer is enabled). See the Timers sections for details.

All events can be independently configured and enabled. The enabled events determine the ADC controls and timing for each ADC sampling interval. Each event consists of a register pair with an event control register ( ACM\_EVCTL[n] ) and an event time register ( ACM\_EVTIME[n] ). The ACM\_EVCTL[n] register enables a particular event and determines settings for the ADC control lines ( ACM\_A[4:0] ) for that particular ADC conversion. The ACM uses the ACM\_EVTIME[n] register to determine the time offset from the corresponding ACM timer trigger input to the start of that particular event. (The time offset is when the event occurs based on trigger input.) This time offset can be specified in terms of the system clock of the processor.

Enable at least one event associated with an enabled ACM timer, for ACM to execute ADC sampling.

## Event Comparators Unit

The event comparators block consists of 16 event time comparators which determine when an enabled event could happen. After detecting valid trigger on selected trigger input, ACM timer starts running at SCLK rate. The comparators compare the ACM timer count with the event time specified in the ACM\_EVTIME[n] register of the enabled event. If the time value matches, the comparators indicate an active event signal to the timing generation unit.

If an event happens when another event is ongoing, the occurred event is stored in the pending event FIFO of timing generation unit. If more than one event associated with an ACM timer is active during the same SCLK0\_0 cycle, only the highest priority event is processed. All other events are missed (even if there was space in the pending event FIFO). However, if both timers are enabled and if multiple events associated with both ACM timers are active at the same SCLK0\_0 cycle, then two events are signaled. One highest priority active event for each timer is signaled.

The priority of events is fixed; the event with the lowest event ID has higher priority compared to other events.

When both ACM timers are enabled, Event0 has the highest priority and Event7 has the lowest priority in the Event[0:7] group associated with ACMTIMER0. Similarly, Event8 has the highest priority and Event15 has the lowest priority in the Event[8:15] group associated with ACMTIMER1. So if Event1 and Event5 occur simultaneously, then Event5 is missed, even if space exists in the pending FIFO. But between the Event[0:7] and Event[8:15] group, simultaneous events can be written into the FIFO.

For example, if Event0 and Event9 occur together, then both are written into the FIFO. The Event[0:7] group has higher priority. Therefore, the order of events in the FIFO is Event0 first and then Event9. If Event1, Event5, Event9, Event15 occurred together, then Event5, and Event15 are missed. Event1 and Event9 are put into the FIFO (Event1 first followed by Event9). When both timers trigger events simultaneously, the event triggered by ACMTMR0 is given higher priority.

When only a single ACM timer is enabled, then all 16 events, Event[0:15], are assigned to that timer. Event0 has highest priority; while Event15 has lowest priority. In this case, if Event1, Event5, Event9, Event15 happened together, then only Event1 is placed in forwarded. Event5, Event9, and Event15 are missed, even if space exists in the pending FIFO.

If an event is missed, the ACM\_STAT.EMISS bit and the corresponding bit in the ACM event missed status register ( ACM\_MEVSTAT ) are set.

## Timing Generation Unit

After event signaling from event comparators, the timing generation unit initiates an ADC sampling interval as per settings of that particular event. It generates ADC control signals based on the ACM\_EVCTL[n].EPF bit field setting. The ACM uses timing registers ( ACM\_TC0 , ACM\_TC1 ) to determine the timing of output signals ( ACM\_CLK , CS, A[4:0] ). These registers contains the fields for set-up time, hold time and zero time for the output signals and ACM clock divider.

The pending FIFO is part of the timing generation unit. If an event occurs when the ACM is busy with another event, the occurred event is stored in the pending event FIFO. This pending event is serviced (for example, the ACM starts an ADC conversion for the event that occurred), after completion of an ongoing event.

The pending event FIFO has a depth of 4, so it can hold up to four pending events. If an event occurs when the pending event FIFO is full, that event is missed. If an event is missed, the ACM\_STAT.EMISS bit is set and the corresponding bit in the ACM\_MEVSTAT register also is set.

When the ACM is disabled, all pending entries in the pending FIFO are flushed.

## Status Flags and Interrupts

The ACM provides a read-only status register ( ACM\_STAT ) to check the module activities. Activities include identifying which event is being serviced, whether any event has been missed, or whether all the events have been serviced for the current trigger.

In addition to this register, the ACM also provides two general-purpose status registers, the ACM event completion status register ( ACM\_EVSTAT ) and the ACM missed event status register ( ACM\_MEVSTAT ). The ACM\_EVSTAT register specifies how to service a completed enabled event for a particular trigger cycle. The ACM\_MEVSTAT register specifies which enabled event has been missed for that particular trigger cycle. This information is provided for all 16 events through individual bits.

Based on these status bits, the ACM can generate two interrupts, event-completed or event-missed, for each event. These interrupts can be selectively enabled for particular ACM events through the ACM completed event interrupt mask register ( ACM\_EVMSK ) and the ACM missed event interrupt mask register ( ACM\_MEVMSK ).

The event completion interrupt is generated only after the entire event completes externally. (For example, when the CS signal goes inactive, and the hold time ( TH ) and zero time ( TZ ) periods are completed for that particular event). The ACM uses the ACM\_EVSTAT register to provide the status of each event indicating which event has caused the interrupt. It is also possible to generate this interrupt when all the events associated with an ACM timer are completed for an ACM trigger cycle. Write the relevant W1C (write 1 to clear) bit in the ACM\_EVSTAT register to clear this interrupt.

The event missed interrupt is generated when an enabled event is missed for a trigger cycle and the corresponding mask bit in the ACM\_MEVMSK register is set. The event comparators unit can miss the event when more than one event, related to the same timer, are active during the same SCLK0\_0 cycle. The timing generation unit can miss when an event occurred and the event pending FIFO was full. The ACM uses the ACM\_MEVSTAT register to provide the status of each missed event indicating which event-miss caused the interrupt. Write the relevant W1C bit in the ACM\_MEVSTAT register to clear this interrupt.

NOTE: A status bit set either in the ACM\_EVSTAT or ACM\_MEVSTAT registers triggers an interrupt only if the corresponding bit in the ACM\_EVMSK or ACM\_MEVMSK registers is enabled.

The ACM provides an event completion interrupt upon completion of particular event or all events. The ACM can also provide a trigger output to the trigger routing unit (TRU) of the processor. T rigger slaves can use this trigger output for their operations without requiring core intervention.

## Event Order Registers

For debugging purpose, ACM hardware includes 16 event order registers, one per each event. These registers indicate the order in which the events complete externally.

These registers are denoted as ACM\_EVORD[n] , where n stands for event ID, 0-15. The ACM module uses the 8bit ACM\_EVORD[n].ORD field of this register to indicate the order the ADC data has been captured for the event. This field accumulates the order count every trigger cycle, unless it is cleared in the software. At each trigger cycle, the values of the register are updated. The ACM must read the register at the end of each trigger cycle (as it writes the new order value of the event in the next trigger cycle). Thus, the 8-bit field can store the order of 256 data captures at a stretch, after which it starts the order count from zero again.

All the event registers can be reset in software by setting the ACM\_CTL.ORST bit. When set, it clears all the register values to zero. This bit is auto-cleared to 0 after all ACM\_EVORD[n] registers are cleared. The ACM can select trigger inputs of the timers to automatically clear these registers, when the ACM\_CTL.AOREN bit is set. The ACM uses the ACM\_CTL.OTSEL bit to determine which trigger input to select for this auto-clearing.

The event order functionality is demonstrated with an example where the ACM has only three events enabled (Event1, Event7, and Event13). The Event Order Timing figure shows the event order register value of these events at different stages.

TRIGGER

ECOM

EVT7

EVT1

EVT13

EVT1

EVT7EVT13

EVT13EVT1

EVT7

Event Status

(Completion)

00

00

00

01

03

04

02

05

98

96

97

99

100

101

EO7

EO1

EO13

Figure 24-11: Event Order Timing

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000010_3cdb0d268a8817a89205605c83781c4e94001ea52d2db419546238ab4e853223.png)

## ACM Operation

The ACM Operation, Two Events figure shows the ACM operation where only two events (Event0 and Event3) are enabled. The line labeled 'ADC Controls' depicts the timing of the ADC control signals: ACM\_A4 through ACM\_A0 .

Figure 24-12: ACM Operation, Two Events

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000011_36d2f3541dbb6238cb5b469798002dd783891f050692907a7c8aac096af04d49.png)

The ACM Operation, Two Events figure depicts a usage case. The ACM\_EVTIME[n] 3 time register is programmed with a count value that is greater than the value programmed in Event0. So, Event3 occurs after Event0. There are, however, no restrictions on the order of the different events. Event0 time can be greater than, less than or equal to Event3 time.

If the value in Event0 time register is less than the value in Event3 time, Event3 occurs after Event0. If the value in Event0 is greater than the value in Event3, then Event3 occurs before Event0. If Event0 time is equal to the Event3 time, the events are processed according to their priorities. Only the highest priority event is processed. Other lowerpriority events are missed (even if there is a space in pending event FIFO), as the same ACM timer handles both events. Event0 has the highest priority. So in this case, the timing generation unit processes Event0, while Event3 is missed. In this case, the Event3 missed status bit ( ACM\_MEVSTAT.EV3 ) is set and the event missed bit ( ACM\_STAT.EMISS ) is set, indicating that an event has been missed.

If event times are not sufficiently spaced apart, an event could occur while a previous event is underway (while the chip select of the previous event is asserted). In this situation, the second event is queued in the pending event FIFO. If the pending event FIFO is full, the event is not queued and is missed. This situation happens most commonly when enabling both ACM timers with different trigger inputs and the sources of these triggers are not synchronized together. In this case, it is possible that the events controlled by the two timers overlap. It is therefore important to consider the possibility of events occurring either simultaneously or being missed when enabling events on two asynchronously triggered timers. It is the programs responsibility to ensure that the values in the event time registers do not lead to event misses. The bits corresponding to the missed event in the ACM\_MEVSTAT and ACM\_STAT registers are flagged on a missed event.

When both ACM timers are enabled and they triggered the events simultaneously, the event triggered by ACM\_TMR0 has the higher priority. For example, when an ACM\_TMR0 event (one of events 0 through 7) and ACM\_TMR1 event (one of events 8 through 15) occur simultaneously:

- The timing generation unit processes the ACM\_TMR0 event, or
- The ACM\_TMR0 event is queued in the pending event FIFO before the processing or the queuing of the ACM\_TMR1 event.

When all the events enabled for a given ACM timer are processed, the ACM timer stops incrementing. (This timer action is not reflected in the ACM Operation, Two Events figure). Also, the corresponding event completion bit ( ACM\_STAT.ECOM1 , ACM\_STAT.ECOM0 ) is flagged. The same bit is also reflected in the status register for event compilation ( ACM\_EVSTAT ). This register can optionally generate the event completion interrupt when the corresponding bit in the event completion interrupt mask register ( ACM\_EVMSK ) is set. T wo separate bits are available, one for each ACM timer.

The processor can use the ACM to generate various sequences of ADC sampling events through appropriate programming of event time registers, event control registers, and triggers. For more information, see the use cases described in Emulation Mode Use Case.

## ACM Programming Concepts

Since the ACM module is used with the SPORT, PWM, GP timer, and GPIO, the programming must comply with the following guidelines for reliable operation of the ACM.

- The ACM is a control module and provides clock and chip select and control signals with required timing. But for capturing the data from ADC, the ACM uses one of the halves of SPORT1.
- Enable the ACM before enabling the SPORT. The SPORT can be configured before enabling the ACM. Configure the SPORT in slave mode [external clock ( SPORT\_CTL\_A.ICLK =0), external frame sync ( SPORT\_CTL\_A.IFS =0)] as receiver.

The timings of external ADC decide the settings of SPORT\_CTL\_A.LFS , SPORT\_CTL\_A.LAFS , SPORT\_CTL\_A.CKRE bits. Generally, the SPORT is configured in DSP serial mode to receive the ADC samples, but other operating modes (such as multichannel) are possible.

If the ACM is programmed in gated clock mode ( ACM\_CTL.CLKMOD =1), set the serial port also in gated clock mode ( SPORT\_CTL\_A.GCLKEN =1).

- DMA mode of SPORT operation is preferred, as it saves the processor MIPS when receiving chunks of data. However, receiving ADC samples in core mode is also possible. When using DMA mode, configure the DMA registers of the selected SPORT appropriately and enable DMA before enabling the SPORT. When using the primary and secondary channels of a SPORT to receive data from two ADC channels, use the 2D feature of DMA to de-interleave the data from two channels. When using core mode of SPORT operation, register the core handler to handle the data read requests from the SPORT receiver.

- In addition to SPORT register settings, first set the DAI registers to enable: SPORT data pins, the ACM clock, the CS, and the data pins. Then, configure the PORT registers for the trigger and control pins. When using either of the ACM\_T1 / ACM\_T0 trigger inputs for ACM timers, configure the PORT\_FER and PORT\_MUX bits of the corresponding pins (PE8 or PG5) according to source of trigger input.
- When using ACM\_T[2:3] trigger inputs for ACM timers, the ACM can configure and enable the trigger routing unit (TRU) at this step. Program the corresponding slave trigger ID using the TRU\_SSR[n] register to select the required master trigger. When using these trigger inputs, do not configure the TRU\_SSR[n] register when the ACM is enabled. The default value of this register is zero and the master trigger ID 0 is system reserved.
- Before enabling the ACM (by setting the ACM\_CTL.EN bit), program all the control bits of ACM control register. These control bits include ACM trigger selects ( ACM\_CTL.TRGSEL1 / ACM\_CTL.TRGSEL0 ), trigger input polarities ( ACM\_CTL.TRGPOL1 / ACM\_CTL.TRGPOL0 ), CS signal polarity ( ACM\_CTL.CSPOL ), ACM clock polarity ( ACM\_CTL.CLKPOL ), ACM clock mode ( ACM\_CTL.CLKMOD ), and serial port unit selection and the ACM\_EVORD[n] register settings.
- Configure the ACM timing control registers to define the ACM clock frequency and the setup, hold, and zero time of the ACM control signals.
- Program the timer enabled bits ( ACM\_CTL.TMR0EN / ACM\_CTL.TMR1EN ) together only after the ACM is enabled. Once the bits are programmed, do not change them. Modifying these enable bits in the ACM control register is not recommended while the ACM is in operation. Doing so can cause events to change dependency from one timer to the other and can cause the values in the ACM status registers ( ACM\_EVSTAT and ACM\_STAT ) to be inaccurate. If both timers are required for use, enable them together after the ACM is enabled. If one timer is already enabled, disable and then reenable the ACM and then program both timer enable bits together. Similarly, when both timers are running, disable them together.
- Once the peripherals have been configured, enable the ACM first and then the SPORT DMA and finally the SPORT module itself. Ideally, a trigger should not be active when enabling the ACM.
- After enabling the ACM, configure and enable the event register pairs (event control and event time registers) to create the required events.
- ACLK is an external clock relative to the SPORT peripheral. Observe any SPORT requirements around a minimum number of stable external clock cycles before the assertion of the first SPORT frame sync. The SPORT requires a minimum of 3 clock cycles before it is able to recognize a frame sync. When the SPORT is configured in gated clock mode, this requirement becomes a minimum of 7 SPORT clock cycles. The required number of ACLK cycles should elapse before first assertion of CS. Use any of the following methods:
- Ensure that ACM triggers are generated at least 3 ACLK cycles after the ACM is enabled.
- Ensure that the event time value ( ACM\_EVTIME[n] ) of the first active event is such that 3 ACLK cycles elapse before the event is processed.

- When the minimum number of ACLK cycles before the assertion of CS is not observed, the SPORT can miss the data of the first ADC sampling event. There is a software workaround for fulfilling this requirement. Program the ACM\_TC0.CKDIV value after enabling the SPORT (subsequently after enabling the ACM) but before the trigger is applied. Since the default value of ACM\_TC0.CKDIV is 1, the ACLK frequency is higher. Therefore, the SPORT can receive the required clock cycles within a short period (before the frame sync arrives).
- When using the ACM\_T2 / ACM\_T3 trigger inputs, the master can be enabled as a last step (if it is configured only to provide triggers to the ACM) to generate the triggers.
- While disabling the ACM system, disable the SPORT first, then the DMA. Finally, disable the ACM.

## Emulation Mode Use Case

This section describes the usage modes of the ACM by illustrating how to implement various sequencing ADC sampling modes.

## Single-Shot Sequencing Mode Emulation

In single-shot sequencing mode, all enabled events are sequentially issued one after the other on the occurrence of an ACM trigger. The sequence of events is fixed, starting with Event0 and ending with Event15.

The Single Shot Sequencing figure shows an example of single-shot sequencing mode where only Event0 and Event1 are enabled. The value ETIME0 is written into the ACM\_ET0 register, and ACMTMR0 is enabled in this mode.

Figure 24-13: Single Shot Sequencing

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000012_b0c20b32911b1590dfdd8bd63a13631dc665c4cb586d94e461d8e54293021a90.png)

To emulate this mode of operation using the ACM:

- Configure the appropriate trigger source for initiating ACM activity. Refer to ADC Control Module (ACM) for information on signals that can trigger the ACM counters.
- Enable only one ACM timer (ACMTMR0)
- Enable events and program the event time values as: Event0 time = X, Event1 time = X + Y, Event2 time = X + 2Y where:

X = ETIME0 , the initial time offset from trigger (if needed)

Y = t H + t CSW + t S  + t Z, where t H  is the hold time, t Z is the zero time, and t S  is the setup time for ACM control lines, as specified in ACM timing registers.

For more information, refer to the ACM External Pin Timing section.

NOTE: Y has to be slightly less than the calculated value to ensure that the next event occurs before the first event completes. Then, the next event is in the pending FIFO and enables the transitions between events without a break.

## Continuous Sequencing Mode Emulation

Continuous sequencing mode is similar to single-shot sequencing mode, except in continuous sequencing the event sequencing continuously repeats. As in single-shot mode, the time offset is programmable in continuous mode. The trigger in continuous mode is relevant only for the first time. Therefore, any subsequent triggers after the first active edge of the trigger are neglected.

The Continuous Sequencing figure shows an example of continuous sequencing mode with only two events Event0, Event1 enabled. To emulate continuous sequencing mode using ACM:

Figure 24-14: Continuous Sequencing

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000013_351e4b5f9f0caed6272461ac5f7f647f98605ca7b76184d9b00073b11110e0d9.png)

- Enable only one ACM timer (for example ACMTIMER0). Select a timer trigger input from the TRU (either of the ACM\_T[n] [3:2].

If ACM\_T2 is selected, program ACM slave trigger ID 33 to select ACM event completion (whose master trigger ID is 19) as trigger input. If ACM\_T3 is selected as trigger input, configure ACM slave trigger ID 34.

- Configure rest of the settings required by programming ACM control and timing registers.
- Enable ACM events with required ACM control lines settings.

Program the event time registers as: Event0 time = X, Event1 time = X + Y, Event 2 time = X + 2Y where:

X and Y values, in terms of SCLK, are as described in the single-shot case. (Y must be slightly less than t H  + t CSW + t S  + t Z to avoid any break between events.)

- Configure and enable system event controller (SEC).

Also configure, map, and enable the ACM event completion interrupt. This interrupt occurs on the completion of all enabled ACM events for the current trigger. That means that programs must only set the ACM\_EVMSK.IECOM0 bit (or the ACM\_EVMSK.IECOM1 bit, if using ACMTIMER1) register.

- Enable ACM, SPORT, and SPORT DMA as per the guidelines in the ACM Programming Concepts section.
- Since the ACM trigger input is configured as ACM event completion trigger output, the first trigger is necessary to start the ACM operation. We can provide this dummy trigger by writing master trigger ID into master trigger register ( TRU\_MTR ).

Write the ACM event completion trigger ID (19) into the TRU\_MTR register. This action triggers the ACM timers and the ACM starts handling the events.

- After completing all events, the ACM provides an event completion trigger and the corresponding interrupt is generated. The ACM trigger output is provided to the ACM timers. The timers reset their counter and start running from zero. The ACM rehandles all the enabled ACM events. This sequence continues.

However, in order to provide the trigger outputs properly, clear the interrupt latch in the ISR by clearing the ACM\_EVSTAT.ECOM0S (or ACM\_EVSTAT.ECOM1S , if using ACMTIMER1) bit.

## ADSP-SC58x ACM Register Descriptions

ADC Control Module (ACM) contains the following registers.

Table 24-5: ADSP-SC58x ACM Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| ACM_CTL       | Control Register                       |
| ACM_EVCTL[n]  | Event NControl Register                |
| ACM_EVMSK     | Event Complete Interrupt Mask Register |
| ACM_EVORD[n]  | Event NOrder Register                  |
| ACM_EVSTAT    | Event Complete Status Register         |
| ACM_EVTIME[n] | Event NTime Register                   |
| ACM_MEVMSK    | Missed Event Interrupt Mask Register   |
| ACM_MEVSTAT   | Missed Event Status Register           |
| ACM_STAT      | Status Register                        |
| ACM_TC0       | Timing Configuration 0 Register        |
| ACM_TC1       | Timing Configuration 1 Register        |
| ACM_TMR0      | Timer 0 Register                       |
| ACM_TMR1      | Timer 1 Register                       |

## Control Register

The ACM\_CTL register enables and selects the various modes of operation of the ACM.

Figure 24-15: ACM\_CTL Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000014_aa508431c73f90c4afc12a3c61210db2c0c714f48a00304a333a2300afd6480e.png)

Table 24-6: ACM\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | EPS        | External Peripheral Select. The ACM_CTL.EPS bit selects whether the ACM interfaces to half SPORT A or half SPORT B.                                                                                   | External Peripheral Select. The ACM_CTL.EPS bit selects whether the ACM interfaces to half SPORT A or half SPORT B.                                                                                   |
| 15 (R/W)           | EPS        | 0                                                                                                                                                                                                     | Half SPORT A Interfaces to ACM                                                                                                                                                                        |
| 14 (R/W)           | OTSEL      | Order Trigger Select. The ACM_CTL.OTSEL bit selects whether TMR0 or TMR1 triggers a reset of the event order ( ACM_EVORD[n] ) registers. This bit is applicable only if the ACM_CTL.AOREN bit is set. | Order Trigger Select. The ACM_CTL.OTSEL bit selects whether TMR0 or TMR1 triggers a reset of the event order ( ACM_EVORD[n] ) registers. This bit is applicable only if the ACM_CTL.AOREN bit is set. |
| 14 (R/W)           | OTSEL      | 0                                                                                                                                                                                                     | ACM TMR0 Triggers Reset of Order Registers                                                                                                                                                            |
| 14 (R/W)           | OTSEL      | 1                                                                                                                                                                                                     | ACM TMR1 Triggers Reset of Order Registers                                                                                                                                                            |

Table 24-6: ACM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | AOREN      | Automatic Order Reset Enable. The ACM_CTL.AOREN bit enables automatic reset of the event order ( ACM_EVORD[n] ) registers, based on the selected timer trigger. The ACM_CTL.OTSEL bit selects the trigger. |
| 13 (R/W)           | AOREN      | 0 Disable Automatic Order Reset                                                                                                                                                                            |
| 13 (R/W)           | AOREN      | 1 Enable Automatic Order Reset                                                                                                                                                                             |
| 12 (R/W)           | ORST       | Order Register Reset. The ACM_CTL.ORST bit resets the event order ( ACM_EVORD[n] ) register value to 0. This bit auto-clears to 0 after the ACM_EVORD[n] registers clear.                                  |
| 11 (R/W)           | CLKMOD     | ADC Clock Mode. The ACM_CTL.CLKMOD bit selects gated clock mode ( ACM_CLK is gated when the ADC CS is inactive) or continuous (ACM generates continuous ACM_CLK ).                                         |
| 11 (R/W)           | CLKMOD     | 0 Continuous Clock Mode                                                                                                                                                                                    |
| 10 (R/W)           | CLKPOL     | Clock Polarity. The ACM_CTL.CLKPOL bit selects whether the rising or falling edge of ACM_CLK comes after ADC CS becomes active.                                                                            |
| 10 (R/W)           | CLKPOL     | 0 Falling Edge of Clock After CS                                                                                                                                                                           |
| 9 (R/W)            | CSPOL      | 1 Rising Edge of Clock After CS Chip Select Polarity. The ACM_CTL.CSPOL bit selects whether ADC CS is active high or low.                                                                                  |
| 9 (R/W)            | CSPOL      | 0 Active Low CS                                                                                                                                                                                            |
| 8 (R/W)            | TRGPOL1    | 1 Active High CS Trigger Polarity TMR1. The ACM_CTL.TRGPOL1 bit selects whether the trigger polarity for ACM TMR1 occurs on the falling or rising edge.                                                    |
| 8 (R/W)            | TRGPOL1    | 0 Rising Edge Trigger                                                                                                                                                                                      |
| 7 (R/W)            | TRGPOL0    | Trigger Polarity TMR0. The ACM_CTL.TRGPOL0 bit selects whether the trigger polarity for ACM TMR0 occurs on the falling or rising edge.                                                                     |
| 7 (R/W)            | TRGPOL0    | 0 Rising Edge Trigger                                                                                                                                                                                      |
| 7 (R/W)            | TRGPOL0    | 1 Falling Edge Trigger                                                                                                                                                                                     |
| 7 (R/W)            | TRGPOL0    |                                                                                                                                                                                                            |

Table 24-6: ACM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:5 (R/W)          | TRGSEL1    | Trigger Select TMR1. The ACM_CTL.TRGSEL1 bits select the external trigger for ACM TMR1.                                                                                             |
| 4:3 (R/W)          | TRGSEL0    | Trigger Select TMR0. The ACM_CTL.TRGSEL0 bits select the external trigger for ACM TMR0. 0 Trigger 0 (ACM_T0 Pin) 1 Trigger 1 (ACM_T1 Pin) 2 Trigger 2 (Trigger Input 2 - TRU Slave) |
| 2 (R/W)            | TMR1EN     | TMR1 Enable. The ACM_CTL.TMR1EN bit enables ACM TMR1. 0 Disable ACM TMR1                                                                                                            |
| 1 (R/W)            | TMR0EN     | 1 Enable ACM TMR1 TMR0 Enable.                                                                                                                                                      |
| 0 (R/W)            |            | The ACM_CTL.TMR0EN bit enables ACM TMR0. 0 Disable ACM TMR0 1 Enable ACM TMR0                                                                                                       |
|                    | EN         | ACM Enable. The ACM_CTL.EN bit enables ACM operation. 0 Disable ACM                                                                                                                 |

## Event N Control Register

The ACM\_EVCTL[n] registers each hold the ADC control value corresponding to the event related to the register. These control registers each have an event enable bit, that permits a selective enable of a particular event.

Do not program the ACM\_EVCTL[n] register when an event is active. Program this register before setting trigger and re-program this register after all the events are complete ( ACM\_STAT.ECOM1 or ACM\_STAT.ECOM0 bit is set). If no events are enabled in this register (for example, all ACM\_EVCTL[n].ENAEV bits =0, and the ACM\_STAT.ECOM0 or ACM\_STAT.ECOM1 bits are set) and an interrupt generates (if unmasked) a trigger is applied with the Timer enabled.

Figure 24-16: ACM\_EVCTL[n] Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000015_1e1ad09a78fc8737873160732aedbef8636bfc9d08fc16f3a7672d11a0f0ca10.png)

Table 24-7: ACM\_EVCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:1 (R/W)          | EPF        | Event Parameter Field. The ACM_EVCTL[n].EPF bits select values for the ADC control pins ( ACM_A[n] . These values are output when the enabled event occurs. Selection of ACM_EVCTL[n].EPF values are based on the type of ADC, usage mode, and other items. For more information, see the operating modes section. All ACM_EVCTL[n].EPF bits have the same external pin timing. |
| 0 (R/W)            | ENAEV      | Enable Event. The ACM_EVCTL[n].ENAEV bit causes a sampling event to occur based on the controls selected by the ACM_EVCTL[n].EPF bit field when an event (time com- parison match or other external trigger) occurs. If disabled, the corresponding event has no significance, and the control values are not used. 0 Disable Event                                             |
| 0 (R/W)            | ENAEV      | 1 Enable Event                                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | ENAEV      |                                                                                                                                                                                                                                                                                                                                                                                 |

## Event Complete Interrupt Mask Register

The ACM\_EVMSK register enables interrupts corresponding to status bits in the ACM\_EVSTAT register. When a bit in the ACM\_EVMSK register is set (=1), an interrupt generates when the corresponding event complete bit in the ACM\_EVSTAT register is set.

Figure 24-17: ACM\_EVMSK Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000016_43b54036a724ab676798ae243fe946e6032efaeef112d71e35cadec07072087b.png)

Table 24-8: ACM\_EVMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------|
| 17                 | IECOM1     | Event Complete 1 Interrupt Enable.                                                        |
| (R/W)              | IECOM0     | 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt                                    |
| (R/W)              | 16         | Event Complete 0 Interrupt Enable. 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt |

Table 24-8: ACM\_EVMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |                            |
|--------------------|------------|----------------------------|----------------------------|
| 15                 | EV15       | Event 15 Interrupt Enable. | Event 15 Interrupt Enable. |
| (R/W)              | EV15       | 0                          | Disable (Mask) Interrupt   |
| 15                 | EV15       | 1                          | Enable (Unmask) Interrupt  |
| 14                 | EV14       | Event 14 Interrupt Enable. | Event 14 Interrupt Enable. |
| (R/W)              | EV14       | 0                          | Disable (Mask) Interrupt   |
| (R/W)              | EV14       | 1                          | Enable (Unmask) Interrupt  |
| 13                 | EV13       | Event 13 Interrupt Enable. | Event 13 Interrupt Enable. |
| (R/W)              | EV13       | 0                          | Disable (Mask) Interrupt   |
| (R/W)              | EV13       | 1                          | Enable (Unmask) Interrupt  |
| 12                 | EV12       | Event 12 Interrupt Enable. | Event 12 Interrupt Enable. |
| (R/W)              | EV12       | 0                          | Disable (Mask) Interrupt   |
| (R/W)              | EV12       | 1                          | Enable (Unmask) Interrupt  |
| 11                 | EV11       | Event 11 Interrupt Enable. | Event 11 Interrupt Enable. |
| (R/W)              | EV11       | 0                          | Disable (Mask) Interrupt   |
| (R/W)              | EV11       | 1                          | Enable (Unmask) Interrupt  |
| 10                 | EV10       | Event 10 Interrupt Enable. | Event 10 Interrupt Enable. |
| (R/W)              | EV10       | 0                          | Disable (Mask) Interrupt   |
|                    |            | 1                          | Enable (Unmask) Interrupt  |
| 9                  | EV9        | Event 9 Interrupt Enable.  | Event 9 Interrupt Enable.  |
| (R/W)              |            | 0                          | Disable (Mask) Interrupt   |
|                    |            | 1                          | Enable (Unmask) Interrupt  |
| 8                  | EV8        | Event 8 Interrupt Enable.  | Event 8 Interrupt Enable.  |
| (R/W)              |            | 0                          | Disable (Mask) Interrupt   |
|                    |            | 1                          | Enable (Unmask) Interrupt  |
| 7                  | EV7        | Event 7 Interrupt Enable.  | Event 7 Interrupt Enable.  |
| (R/W)              |            | 0                          | Disable (Mask) Interrupt   |
|                    |            | 1                          | Enable (Unmask) Interrupt  |
| 6                  | EV6        | Event 6 Interrupt Enable.  | Event 6 Interrupt Enable.  |
| (R/W)              |            | 0                          | Disable (Mask) Interrupt   |
|                    |            | 1                          | Enable (Unmask) Interrupt  |

Table 24-8: ACM\_EVMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                          |
|--------------------|-----------------------|----------------------------------------------------------------------------------|
| 5                  | EV5                   | Event 5 Interrupt Enable.                                                        |
| 4 (R/W)            | EV4                   | Event 4 Interrupt Enable. 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt |
| 3 (R/W)            | EV3 Event 3 Interrupt | Enable. 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt                   |
| 2 (R/W)            | EV2                   | Event 2 Interrupt Enable.                                                        |
| 1                  | EV1                   | 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt                           |
| (R/W) 0            | EV0                   | Event 1 Interrupt Enable. 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt |
| (R/W)              |                       | 0 Disable (Mask) Interrupt                                                       |
|                    | Event 0 Interrupt     | Enable.                                                                          |
|                    | 1                     | Enable (Unmask) Interrupt                                                        |

## Event N Order Register

The ACM\_EVORD[n] registers hold the ADC data capture event order. These registers can store the order of 256 data captures at a stretch. The ACM\_EVORD[n] registers also have status bits indicating whether an event misses or completes in the trigger cycle.

Figure 24-18: ACM\_EVORD[n] Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000017_f15ff7a5ae023aa5280b06b3f24dc06184432c95fad0967c375a800a88b9f3fe.png)

Table 24-9: ACM\_EVORD[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/NW)          | EVSTAT     | Event Status. The ACM_EVORD[n].EVSTAT bit reflects the state of the corresponding event bit in the ACM_EVSTAT register.                                                                                                                                                                          |
| 16 (R/NW)          | MEVSTAT    | Missed Event Status. The ACM_EVORD[n].MEVSTAT bit reflects the state of the corresponding event bit in the ACM_MEVSTAT register.                                                                                                                                                                 |
| 7:0 (R/NW)         | ORD        | Order of Event Completion. The ACM_EVORD[n].ORD bits indicate the order of event completion. Zero indi- cates the first event completed (after the ACM is enabled or after the ACM_CTL.ORST bit is set) and 255 indicates the 256th event completed. 0 1st Event Completed 1 2nd Event Completed |
| 7:0 (R/NW)         | ORD        | 255 256th Event Completed                                                                                                                                                                                                                                                                        |
| 7:0 (R/NW)         | ORD        |                                                                                                                                                                                                                                                                                                  |
| 7:0 (R/NW)         | ORD        |                                                                                                                                                                                                                                                                                                  |

## Event Complete Status Register

The ACM\_EVSTAT register identifies which enabled event has occurred for a particular trigger cycle. When an ACM\_EVSTAT bit is cleared (=0), this status indicates that the ACM has not begun or completed conversion for the corresponding event (conversion not done). When an ACM\_EVSTAT bit is set (=1), this status indicates that the ACM has completed conversion for the corresponding event (conversion done).

Figure 24-19: ACM\_EVSTAT Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000018_5cf18e48f0e132f9938c5436caac36d00f862e0ecd281cd6b9bdf78406a9df9e.png)

Table 24-10: ACM\_EVSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | ECOM1S     | Event Complete 1 Status. The ACM_EVSTAT.ECOM1S bit indicates the state of the ACM_STAT.ECOM1 bit. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C and is not cleared by a trigger. |
| 17 (R/W1C)         | ECOM1S     | 0 No Status                                                                                                                                                                                                                                                          |
| 17 (R/W1C)         | ECOM1S     | 1 ACM_STAT.ECOM1 =1 Occurred                                                                                                                                                                                                                                         |

Table 24-10: ACM\_EVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W1C)         | ECOM0S     | Event Complete 0 Status. The ACM_EVSTAT.ECOM0S bit indicates the state of the ACM_STAT.ECOM0 bit. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C and is not cleared by a trigger. |
| 16 (R/W1C)         | ECOM0S     | 0 No Status                                                                                                                                                                                                                                                          |
| 16 (R/W1C)         | ECOM0S     | 1 ACM_STAT.ECOM0 =1 Occurred                                                                                                                                                                                                                                         |
| 15 (R/W1C)         | EV15       | Event 15 Status. The ACM_EVSTAT.EV15 bit indicates when the ACM has completed the conver- sion for event 15. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.                        |
| 15 (R/W1C)         | EV15       | 0 No Event 15 Conversion                                                                                                                                                                                                                                             |
| 15 (R/W1C)         | EV15       | 1 Event 15 Conversion Done                                                                                                                                                                                                                                           |
| 14 (R/W1C)         | EV14       | Event 14 Status. The ACM_EVSTAT.EV14 bit indicates when the ACM has completed the conver- sion for event 14. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.                        |
| 14 (R/W1C)         | EV14       | 0 No Event 14 Conversion                                                                                                                                                                                                                                             |
| 14 (R/W1C)         | EV14       | 1 Event 14 Conversion Done                                                                                                                                                                                                                                           |
| 13 (R/W1C)         | EV13       | Event 13 Status. The ACM_EVSTAT.EV13 bit indicates when the ACM has completed the conver- sion for event 13. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.                        |
| 13 (R/W1C)         | EV13       | 0 No Event 13 Conversion                                                                                                                                                                                                                                             |
| 13 (R/W1C)         | EV13       | 1 Event 13 Conversion Done                                                                                                                                                                                                                                           |
| 12 (R/W1C)         | EV12       | Event 12 Status. The ACM_EVSTAT.EV12 bit indicates when the ACM has completed the conver- sion for event 12. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. 12 Conversion          |
| 12 (R/W1C)         | EV12       | 0 No Event                                                                                                                                                                                                                                                           |
| 12 (R/W1C)         | EV12       | 1 Event 12 Conversion Done                                                                                                                                                                                                                                           |

Table 24-10: ACM\_EVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W1C)         | EV11       | Event 11 Status. The ACM_EVSTAT.EV11 bit indicates when the ACM has completed the conver- sion for event 11. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.         |
| 11 (R/W1C)         | EV11       | 0 No Event 11 Conversion                                                                                                                                                                                                                              |
| 11 (R/W1C)         | EV11       | 1 Event 11 Conversion Done                                                                                                                                                                                                                            |
| 10 (R/W1C)         | EV10       | Event 10 Status. The ACM_EVSTAT.EV10 bit indicates when the ACM has completed the conver- sion for event 10. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.         |
| 10 (R/W1C)         | EV10       | 0 No Event 10 Conversion                                                                                                                                                                                                                              |
| 10 (R/W1C)         | EV10       | 1 Event 10 Conversion Done                                                                                                                                                                                                                            |
| 9 (R/W1C)          | EV9        | Event 9 Status. The ACM_EVSTAT.EV9 bit indicates when the ACM has completed the conversion for event 9. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled),                                                                     |
| 9 (R/W1C)          | EV9        | 0 No Event 9 Conversion                                                                                                                                                                                                                               |
| 9 (R/W1C)          | EV9        | 1 Event 9 Conversion Done                                                                                                                                                                                                                             |
| 8 (R/W1C)          | EV8        | Event 8 Status. The ACM_EVSTAT.EV8 bit indicates that the ACM has completed the conversion for event 8. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.              |
| 8 (R/W1C)          | EV8        | 0 No Event 8 Conversion                                                                                                                                                                                                                               |
| 8 (R/W1C)          | EV8        | 1 Event 8 Conversion Done                                                                                                                                                                                                                             |
| 7 (R/W1C)          | EV7        | Event 7 Status. The ACM_EVSTAT.EV7 bit indicates when the ACM has completed the conversion for event 7. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. 7 Conversion |
| 7 (R/W1C)          | EV7        | 0 No Event                                                                                                                                                                                                                                            |
| 7 (R/W1C)          | EV7        | 1 Event 7 Conversion Done                                                                                                                                                                                                                             |

Table 24-10: ACM\_EVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1C)          | EV6        | Event 6 Status. The ACM_EVSTAT.EV6 bit indicates when the ACM has completed the conversion for event 6. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. |
| 6 (R/W1C)          | EV6        | 0 No Event 6 Conversion                                                                                                                                                                                                                  |
| 5 (R/W1C)          | EV5        | Event 5 Status. The ACM_EVSTAT.EV5 bit indicates when the ACM has completed the conversion for event 5. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. |
| 5 (R/W1C)          | EV5        | 0 No Event 5 Conversion                                                                                                                                                                                                                  |
| 5 (R/W1C)          | EV5        | 1 Event 5 Conversion Done                                                                                                                                                                                                                |
| 4 (R/W1C)          | EV4        | Event 4 Status. The ACM_EVSTAT.EV4 bit indicates when the ACM has completed the conversion for event 4. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. |
| 4 (R/W1C)          | EV4        | 0 No Event 4 Conversion                                                                                                                                                                                                                  |
| 4 (R/W1C)          | EV4        | 1 Event 4 Conversion Done                                                                                                                                                                                                                |
| 3 (R/W1C)          | EV3        | Event 3 Status. The ACM_EVSTAT.EV3 bit indicates when the ACM has completed the conversion for event 3. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. |
| 3 (R/W1C)          | EV3        | 0 No Event 3 Conversion                                                                                                                                                                                                                  |
| 3 (R/W1C)          | EV3        | 1 Event 3 Conversion Done                                                                                                                                                                                                                |
| 2 (R/W1C)          | EV2        | Event 2 Status. The ACM_EVSTAT.EV2 bit indicates when the ACM has completed the conversion for event 2. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. |
| 2 (R/W1C)          | EV2        | 0 No Event 2 Conversion                                                                                                                                                                                                                  |
| 2 (R/W1C)          | EV2        | 1 Event 2 Conversion Done                                                                                                                                                                                                                |

Table 24-10: ACM\_EVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | EV1        | Event 1 Status. The ACM_EVSTAT.EV1 bit indicates when the ACM has completed the conversion for event 1. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. | Event 1 Status. The ACM_EVSTAT.EV1 bit indicates when the ACM has completed the conversion for event 1. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. |
| 0 (R/W1C)          | EV0        | Event 0 Status. The ACM_EVSTAT.EV0 bit indicates when the ACM has completed the conversion for event 0. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled),                                                        | Event 0 Status. The ACM_EVSTAT.EV0 bit indicates when the ACM has completed the conversion for event 0. If set and the corresponding bit in ACM_EVMSK is set (interrupt enabled),                                                        |
| 0 (R/W1C)          | EV0        | 0                                                                                                                                                                                                                                        | No Event 0 Conversion                                                                                                                                                                                                                    |
| 0 (R/W1C)          | EV0        | 1                                                                                                                                                                                                                                        | Event 0 Conversion Done                                                                                                                                                                                                                  |

## Event N Time Register

The ACM\_EVTIME[n] registers each hold a 32-bit event time value. There are 16 event time registers, 8 for each ACM timer (when both timers are enabled). If only one timer is enabled, all 16 of the ACM\_EVTIME[n] registers are assigned to the enabled timer.

Do not program the ACM\_EVTIME[n] register when an event is active. Program this register before setting a trigger and re-program the register after all events are complete ( ACM\_STAT.ECOM1 or ACM\_STAT.ECOM0 bits are set). If no events are enabled in this register (for example, the ACM\_EVCTL[n].ENAEV bits =0 and the ACM\_STAT.ECOM0 or ACM\_STAT.ECOM1 bits =1) and an interrupt generates (if unmasked), a trigger is applied with the Timer enabled.

Figure 24-20: ACM\_EVTIME[n] Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000019_6bd78fb16dd3301d6a2cf6a7973e6859a9b205833c97abbb17c3b7f94dba35bb.png)

Table 24-11: ACM\_EVTIME[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 31:0               | ETIME      | Event Time Value.                                                     |
| (R/W)              |            | The ACM_EVTIME[n].ETIME bit field contains a 32-bit event time value. |

## Missed Event Interrupt Mask Register

The ACM\_MEVMSK register enables interrupts corresponding to status bits in the ACM\_MEVSTAT register. When a bit is set in the ACM\_MEVMSK register, an interrupt generates when the corresponding event missed bit in the ACM\_MEVSTAT register is set.

Figure 24-21: ACM\_MEVMSK Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000020_0f8d9ac09c28d61ba45b3a993d53c52c71409d628473b19e1be647b115da57b4.png)

Table 24-12: ACM\_MEVMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 15 (R/W)           | EV15       | Event 15 Missed Interrupt Enable. 0 Disable (Mask) Interrupt |
| 14 (R/W)           | EV14       | Event 14 Missed Interrupt Enable. 0 Disable (Mask) Interrupt |
| 13                 | EV13       | Event 13 Missed Interrupt Enable. 0 Disable (Mask)           |
| (R/W)              |            | Interrupt 1 Enable (Unmask) Interrupt                        |

Table 24-12: ACM\_MEVMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration           |
|--------------------|------------|-----------------------------------|
| 12 (R/W)           | EV12       | Event 12 Missed Interrupt Enable. |
| 12 (R/W)           | EV12       | 0 Disable (Mask) Interrupt        |
| 12 (R/W)           | EV12       | 1 Enable (Unmask) Interrupt       |
| 11 (R/W)           | EV11       | Event 11 Missed Interrupt Enable. |
| 11 (R/W)           | EV11       | 0 Disable (Mask) Interrupt        |
| 11 (R/W)           | EV11       | 1 Enable (Unmask) Interrupt       |
| 10 (R/W)           | EV10       | Event 10 Missed Interrupt Enable. |
| 10 (R/W)           | EV10       | 0 Disable (Mask) Interrupt        |
| 10 (R/W)           | EV10       | 1 Enable (Unmask) Interrupt       |
| 9 (R/W)            | EV9        | Event 9 Missed Interrupt Enable.  |
| 9 (R/W)            | EV9        | 0 Disable (Mask) Interrupt        |
| 9 (R/W)            | EV9        | 1 Enable (Unmask) Interrupt       |
| 8 (R/W)            | EV8        | Event 8 Missed Interrupt Enable.  |
| 8 (R/W)            | EV8        | 0 Disable (Mask) Interrupt        |
| 8 (R/W)            | EV8        | 1 Enable (Unmask) Interrupt       |
| 7                  | EV7        | Event 7 Missed Interrupt Enable.  |
| (R/W)              | EV7        | 0 Disable (Mask) Interrupt        |
| 7                  | EV7        | 1 Enable (Unmask) Interrupt       |
| 6                  | EV6        | Event 6 Missed Interrupt Enable.  |
| (R/W)              | EV6        | 0 Disable (Mask) Interrupt        |
| 6                  | EV6        | 1 Enable (Unmask) Interrupt       |
| 5                  | EV5        | Event 5 Missed Interrupt Enable.  |
| (R/W)              | EV5        | 0 Disable (Mask) Interrupt        |
| 4                  | EV4        | Event 4 Missed Interrupt Enable.  |
| 4                  | EV4        | 1 Enable (Unmask)                 |
| (R/W) 3            | EV4        | 0 Disable (Mask) Interrupt        |
| (R/W) 3            |            | Interrupt                         |
| (R/W)              | EV3        | Event 3 Missed Interrupt Enable.  |
| (R/W)              | EV3        | 0 Disable (Mask) Interrupt        |
| (R/W)              | EV3        | 1 Enable (Unmask) Interrupt       |

Table 24-12: ACM\_MEVMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                     |
|--------------------|------------|-------------------------------------------------------------|
| 2                  | EV2        | Event 2 Missed Interrupt Enable.                            |
| 1 (R/W)            | EV1        | Event 1 Missed Interrupt Enable. 0 Disable (Mask) Interrupt |
| 0                  | EV0        | Event 0 Missed Interrupt Enable.                            |
| (R/W)              |            | 0 Disable (Mask) Interrupt 1 Enable (Unmask) Interrupt      |

## Missed Event Status Register

The ACM\_MEVSTAT register indicates which enabled event is missed for a particular trigger cycle. When a ACM\_MEVSTAT register bit is set (=1), this status indicates a miss of the corresponding event. This status generates an interrupt if the corresponding bit in the ACM\_MEVMSK register is set.

Figure 24-22: ACM\_MEVSTAT Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000021_1495753e6b835a5b34b1cca68d1d8d564ae8e1623676d4fac249725222ab9e30.png)

Table 24-13: ACM\_MEVSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | EV15       | Event 15 Missed. The ACM_MEVSTAT.EV15 bit indicates a miss of event 15 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C. |
| 15 (R/W1C)         | EV15       | 0 No Event 15 Missed Status                                                                                                                                                                                                        |
| 15 (R/W1C)         | EV15       | 1 Event 15 Missed                                                                                                                                                                                                                  |

Table 24-13: ACM\_MEVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W1C)         | EV14       | Event 14 Missed. The ACM_MEVSTAT.EV14 bit indicates a miss of event 14 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C.        |
| 14 (R/W1C)         | EV14       | 0 No Event 14 Missed Status                                                                                                                                                                                                               |
| 14 (R/W1C)         | EV14       | 1 Event 14 Missed                                                                                                                                                                                                                         |
| 13 (R/W1C)         | EV13       | Event 13 Missed. The ACM_MEVSTAT.EV13 bit indicates a miss of event 13 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C.        |
| 13 (R/W1C)         | EV13       | 0 No Event 13 Missed Status                                                                                                                                                                                                               |
| 13 (R/W1C)         | EV13       | 1 Event 13 Missed                                                                                                                                                                                                                         |
| 12 (R/W1C)         | EV12       | Event 12 Missed. The ACM_MEVSTAT.EV12 bit indicates a miss of event 12 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C.        |
| 12 (R/W1C)         | EV12       | 0 No Event 12 Missed Status                                                                                                                                                                                                               |
| 12 (R/W1C)         | EV12       | 1 Event 12 Missed                                                                                                                                                                                                                         |
| 11 (R/W1C)         | EV11       | Event 11 Missed. The ACM_MEVSTAT.EV11 bit indicates a miss of event 11 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C.        |
| 11 (R/W1C)         | EV11       | 0 No Event 11 Missed Status                                                                                                                                                                                                               |
| 11 (R/W1C)         | EV11       | 1 Event 11 Missed                                                                                                                                                                                                                         |
| 10 (R/W1C)         | EV10       | Event 10 Missed. The ACM_MEVSTAT.EV10 bit indicates a miss of event 10 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C. Status |
| 10 (R/W1C)         | EV10       | 0 No Event 10 Missed                                                                                                                                                                                                                      |
| 10 (R/W1C)         | EV10       | 1 Event 10 Missed                                                                                                                                                                                                                         |

Table 24-13: ACM\_MEVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W1C)          | EV9        | Event 9 Missed. The ACM_MEVSTAT.EV9 bit indicates when a miss of event 9 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condi- tion generates an interrupt. This bit is W1C.        |
| 9 (R/W1C)          | EV9        | 0 No Event 9 Missed Status                                                                                                                                                                                                                  |
| 8 (R/W1C)          | EV8        | Event 8 Missed. The ACM_MEVSTAT.EV8 bit indicates a miss instance of event 8 since the last trig- ger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.    |
| 8 (R/W1C)          | EV8        | 0 No Event 8 Missed Status                                                                                                                                                                                                                  |
| 8 (R/W1C)          | EV8        | 1 Event 8 Missed                                                                                                                                                                                                                            |
| 7 (R/W1C)          | EV7        | Event 7 Missed. The ACM_MEVSTAT.EV7 bit indicates a miss of event 7 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.               |
| 7 (R/W1C)          | EV7        | 0 No Event 7 Missed Status                                                                                                                                                                                                                  |
| 7 (R/W1C)          | EV7        | 1 Event 7 Missed                                                                                                                                                                                                                            |
| 6 (R/W1C)          | EV6        | Event 6 Missed. The ACM_MEVSTAT.EV6 bit indicates a miss of event 6 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.               |
| 6 (R/W1C)          | EV6        | 0 No Event 6 Missed Status                                                                                                                                                                                                                  |
| 6 (R/W1C)          | EV6        | 1 Event 6 Missed                                                                                                                                                                                                                            |
| 5 (R/W1C)          | EV5        | Event 5 Missed. The ACM_MEVSTAT.EV5 bit indicates a miss of event 5 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. Missed Status |
| 5 (R/W1C)          | EV5        | 0 No Event 5                                                                                                                                                                                                                                |
| 5 (R/W1C)          | EV5        | 1 Event 5 Missed                                                                                                                                                                                                                            |

Table 24-13: ACM\_MEVSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W1C)          | EV4        | Event 4 Missed. The ACM_MEVSTAT.EV4 bit indicates a miss of event 4 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.        |
| 4 (R/W1C)          | EV4        | 0 No Event 4 Missed Status                                                                                                                                                                                                           |
| 4 (R/W1C)          | EV4        | 1 Event 4 Missed                                                                                                                                                                                                                     |
| 3 (R/W1C)          | EV3        | Event 3 Missed. The ACM_MEVSTAT.EV3 bit indicates a miss of event 3 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.        |
| 3 (R/W1C)          | EV3        | 0 No Event 3 Missed Status                                                                                                                                                                                                           |
| 3 (R/W1C)          | EV3        | 1 Event 3 Missed                                                                                                                                                                                                                     |
| 2 (R/W1C)          | EV2        | Event 2 Missed. The ACM_MEVSTAT.EV2 bit indicates a miss of event 2 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.        |
| 2 (R/W1C)          | EV2        | 0 No Event 2 Missed Status                                                                                                                                                                                                           |
| 2 (R/W1C)          | EV2        | 1 Event 2 Missed                                                                                                                                                                                                                     |
| 1 (R/W1C)          | EV1        | Event 1 Missed. The ACM_MEVSTAT.EV1 bit indicates a miss of event 1 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C.        |
| 1 (R/W1C)          | EV1        | 0 No Event 1 Missed Status                                                                                                                                                                                                           |
| 1 (R/W1C)          | EV1        | 1 Event 1 Missed                                                                                                                                                                                                                     |
| 0 (R/W1C)          | EV0        | Event 0 Missed. The ACM_MEVSTAT.EV0 bit indicates a miss of event 0 since the last trigger. If set and the corresponding bit in ACM_MEVMSK is set (interrupt enabled), the condition generates an interrupt. This bit is W1C. Status |
| 0 (R/W1C)          | EV0        | 0 No Event 0 Missed                                                                                                                                                                                                                  |
| 0 (R/W1C)          | EV0        | 1 Event 0 Missed                                                                                                                                                                                                                     |

## Status Register

The ACM\_STAT register indicates the ACM event that is currently being serviced, pending events, missed events, and missed triggers.

Figure 24-23: ACM\_STAT Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000022_2de300cfa304f8eb64e97e448b2e2c91ee9380a711572b42c0c05021f9227302.png)

Table 24-14: ACM\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | CEVNT      | Current Event. The ACM_STAT.CEVNT bits indicate to which event (0 through 15) the ongoing ac- cess (current event, if any) corresponds.                                                  |
| 7:4 (R/NW)         | CEVNT      | 0 Current Event Correspond to Event 0                                                                                                                                                    |
| 7:4 (R/NW)         | CEVNT      | 1 Current Event Correspond to Event 1                                                                                                                                                    |
| 7:4 (R/NW)         | CEVNT      | 15 Current Event Correspond to Event 15                                                                                                                                                  |
| 3 (R/NW)           | ECOM1      | Event Completion 1. The ACM_STAT.ECOM1 bit indicates TMR1 event completion for all enabled ACM TMR1 events and the current trigger. The ACM clears this bit with each trigger.           |
| 3 (R/NW)           | ECOM1      | 0 No Status                                                                                                                                                                              |
| 3 (R/NW)           | ECOM1      | 1 ACM TMR1 Events Complete                                                                                                                                                               |
| 2 (R/NW)           | ECOM0      | Event Completion 0. The ACM_STAT.ECOM0 bit indicates TMR0 event completion for all enabled ACM TMR0 events and the current trigger. The ACM clears this bit with each trigger. No Status |
| 2 (R/NW)           | ECOM0      | 0                                                                                                                                                                                        |
| 2 (R/NW)           | ECOM0      | 1 ACM TMR0 Events Complete                                                                                                                                                               |

Table 24-14: ACM\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | EMISS      | Event(s) Missed. The ACM_STAT.EMISS bit indicates a missed event time (any bits in ACM_MEVSTAT set). This bit is cleared by writing into the ACM_MEVSTAT register. 0 No Missed Event(s) 1 Missed Event(s) |
| 0 (R/NW)           | BSY        | Busy. The ACM_STAT.BSY bit indicates when the ACM is busy (an external sampling event in progress; CS is active or about to go active). 0 Idle 1 Busy                                                     |

## Timing Configuration 0 Register

The ACM\_TC0 register determines the frequency of ACM\_CLK (using the ACM\_TC0.CKDIV field) and the setup cycles (using the ACM\_TC0.SC field) for the ADC controls. Setup cycles are specified in terms of SCLK0\_0.

Figure 24-24: ACM\_TC0 Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000023_d637165e54234e6a18722d877e69e22ae2f7eb375033e254fe17bcfab3a313a1.png)

Table 24-15: ACM\_TC0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:16 (R/W)        | SC         | Setup Cycle. The ACM_TC0.SC bits select the ADC control pins and the setup time in SCLK0_0 cycles with respect to the ADC chip select active edge. The setup time may be calculat- ed from: Setup Time = ACM_TC0.SC + 1. The maximum setup cycle time is 4096 * SCLK0_0, and the minimum setup cycle time is 1 SCLK0_0. 0 1 SCLK0_0 Cycle Setup Time                                                                                                                                 |
| 7:0 (R/W)          | CKDIV      | Clock Divisor. The ACM_TC0.CKDIV bits select the frequency of ACM_CLK as a function of the system clock frequency (SCLK0_0) and the value of the CKDIV field according to the formula: ACM_CLK frequency = (SCLK0_0 frequency)/( ACM_TC0.CKDIV + 1) The maximum ACM_CLK frequency is SCLK0_0/2, and the minimum ACM_CLK frequency is SCLK0_0/256. For example, for a 100 MHz SCLK0_0, the ACM_CLK frequency range is from 390 KHz to 50 MHz. The value ACM_TC0.CKDIV =0 is reserved. |

## Timing Configuration 1 Register

The ACM\_TC1 register provides programmability for the active duration of the following ADC controls: chip select (TCSW), Hold Cycles (T H ), and Zero Cycles (T Z ).

Figure 24-25: ACM\_TC1 Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000024_49ea66c4a70b521058ce43a3c013e9e89b5d0c0a87cf6318110533c04c0642a7.png)

Table 24-16: ACM\_TC1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:12 (R/W)        | ZC         | Zero Cycle. The ACM_TC1.ZC bits select the ADC control zero duration. All ADC controls drive low for ACM_TC1.ZC ACM_CLK cycles.                                                                                 |
| 11:8 (R/W)         | HC         | Hold Cycle. The ACM_TC1.HC bits select the ADC control hold duration. All ADC controls are held after the inactive edge of CS for ACM_TC1.HC ACM_CLK cycles.                                                    |
| 7:0 (R/W)          | CSW        | 15 15 Hold Cycles Chip Select Width. The ACM_TC1.CSW bits select the active duration of CS. The CS is active for ACM_TC1.CSW ACM_CLK +1 cycles. 0 1 Active CS Cycle 1 2 Active CS Cycles 15 16 Active CS Cycles |

## Timer 0 Register

The ACM\_TMR0 register contains the active count value for ACM timer 0. Access this read-only value at any time.

Figure 24-26: ACM\_TMR0 Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000025_9696ab2a9e0aca1ab49372a442b8786ec0cddc805def33abbdeedb91deda1809.png)

Table 24-17: ACM\_TMR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 31:0               | TMRCNT     | Active TMR0 Count Value.                                                       |
| (R/NW)             |            | The ACM_TMR0.TMRCNT bit field contains the active count value for ACM timer 0. |

## Timer 1 Register

The ACM\_TMR1 register contains the active count value for ACM timer 1. Programs can access this read-only value at any time.

Figure 24-27: ACM\_TMR1 Register Diagram

![Image](27_ADC_Control_Module_(ACM)_artifacts/image_000026_a82c767d12ad6fc0cd089758c5e0d545efc296d22dfc57bba66082670d510cc3.png)

Table 24-18: ACM\_TMR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 31:0               | TMRCNT     | Active TMR1 Count Value.                                                       |
| (R/NW)             |            | The ACM_TMR1.TMRCNT bit field contains the active count value for ACM timer 1. |