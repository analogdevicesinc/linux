# General-Purpose Timer (TIMER)

<!-- source: 027_General-Purpose_Timer_TIMER.pdf | original pages 1452–1507 -->

## 25   General-Purpose Timer (TIMER)

The general-purpose timer (GP Timer) module serves as a collection of system timers that support various systemlevel functions. These functions include:

- Synchronized PWM waveform output capability
- External signal capture
- External event count
- General time-base functionality

Additionally, interrupt requests can be generated upon completion of timer events. Moreover, GP timers can act both as trigger requesters and trigger completers.

## GP Timer Features

Each timer can be individually configured in any of these modes:

- Pin interrupt capture mode
- Windowed watchdog mode
- Pulse-width count and capture (WDTH\_CAP) mode
- External Event (EXT\_CLK) mode
- Pulse-width modulation (PWM\_OUT) mode

Other features include:

- Synchronous operation
- Consistent management of period and pulse width values
- Autobaud detection for UART module (where available)
- Graceful bit pattern termination when stopping
- Support for center-aligned PWM patterns
- Error detection on implausible pattern values

- All read and write accesses to 32-bit registers are atomic
- Every timer has its dedicated interrupt request output

## ADSP-2159x\_SC592\_SC594 TIMER Register List

The General-Purpose Timer block (TIMER) provides timers that may be used for external event capture and measurement, system timing, and PWM waveform generation. A set of registers governs TIMER operations. For more information on TIMER functionality, see the TIMER register descriptions.

Table 25-1: ADSP-2159x\_SC592\_SC594 TIMER Register List

| Name               | Description                       |
|--------------------|-----------------------------------|
| TIMER_BCAST_DLY    | Broadcast Delay Register          |
| TIMER_BCAST_PER    | Broadcast Period Register         |
| TIMER_BCAST_WID    | Broadcast Width Register          |
| TIMER_DATA_ILAT    | Data Interrupt Latch Register     |
| TIMER_DATA_IMSK    | Data Interrupt Mask Register      |
| TIMER_ERR_TYPE     | Error Type Status Register        |
| TIMER_RUN          | Run Register                      |
| TIMER_RUN_CLR      | Run Clear Register                |
| TIMER_RUN_SET      | Run Set Register                  |
| TIMER_STAT_ILAT    | Status Interrupt Latch Register   |
| TIMER_STAT_IMSK    | Status Interrupt Mask Register    |
| TIMER_STOP_CFG     | Stop Configuration Register       |
| TIMER_STOP_CFG_CLR | Stop Configuration Clear Register |
| TIMER_STOP_CFG_SET | Stop Configuration Set Register   |
| TIMER_TMR[n]_CFG   | Timer n Configuration Register    |
| TIMER_TMR[n]_CNT   | Timer n Counter Register          |
| TIMER_TMR[n]_DLY   | Timer n Delay Register            |
| TIMER_TMR[n]_PER   | Timer n Period Register           |
| TIMER_TMR[n]_WID   | Timer n Width Register            |
| TIMER_TRG_IE       | Trigger Receiver Enable Register  |
| TIMER_TRG_MSK      | Trigger Generator Mask Register   |

## ADSP-2159x\_SC592\_SC594 TIMER Interrupt List

Table 25-2: ADSP-2159x\_SC592\_SC594 TIMER Interrupt List

| Interrupt ID   | Name                      | Description                     | Sensitivity   | DMA Channel   |
|----------------|---------------------------|---------------------------------|---------------|---------------|
| 48             | TIMER0_TMR00              | TIMER0 Timer 0                  | Level         |               |
| 49             | TIMER0_TMR01              | TIMER0 Timer 1                  | Level         |               |
| 50             | TIMER0_TMR02              | TIMER0 Timer 2                  | Level         |               |
| 51             | TIMER0_TMR03              | TIMER0 Timer 3                  | Level         |               |
| 52             | TIMER0_TMR04              | TIMER0 Timer 4                  | Level         |               |
| 53             | TIMER0_TMR05              | TIMER0 Timer 5                  | Level         |               |
| 54             | TIMER0_TMR06              | TIMER0 Timer 6                  | Level         |               |
| 55 56          | TIMER0_TMR07 TIMER0_TMR08 | TIMER0 Timer 7 TIMER0 Timer 8   | Level Level   |               |
| 57             | TIMER0_TMR09              | TIMER0 Timer 9                  | Level         |               |
|                | TIMER0_TMR10              | TIMER0 Timer 10                 | Level         |               |
| 58             |                           |                                 |               |               |
| 59             | TIMER0_TMR11 TIMER0_TMR12 | TIMER0 Timer 11 TIMER0 Timer 12 | Level Level   |               |
| 60             |                           |                                 |               |               |
| 61             | TIMER0_TMR13              | TIMER0 Timer 13                 | Level         |               |
| 62             | TIMER0_TMR14              | TIMER0 Timer 14                 | Level         |               |
| 63             | TIMER0_TMR15              | TIMER0 Timer 15                 | Level         |               |
| 64             | TIMER0_STAT               | TIMER0 Status                   | Level         |               |

## ADSP-2159x\_SC592\_SC594 TIMER Trigger List

Table 25-3: ADSP-2159x\_SC592\_SC594 TIMER Trigger List Generators

|   Trigger ID | Name             | Description    | Sensitivity   |
|--------------|------------------|----------------|---------------|
|          142 | TIMER0_TMR00_MST | TIMER0 Timer 0 | Edge          |
|          143 | TIMER0_TMR01_MST | TIMER0 Timer 1 | Edge          |
|          144 | TIMER0_TMR02_MST | TIMER0 Timer 2 | Edge          |
|          145 | TIMER0_TMR03_MST | TIMER0 Timer 3 | Edge          |
|          146 | TIMER0_TMR04_MST | TIMER0 Timer 4 | Edge          |
|          147 | TIMER0_TMR05_MST | TIMER0 Timer 5 | Edge          |
|          148 | TIMER0_TMR06_MST | TIMER0 Timer 6 | Edge          |
|          149 | TIMER0_TMR07_MST | TIMER0 Timer 7 | Edge          |
|          150 | TIMER0_TMR08_MST | TIMER0 Timer 8 | Edge          |

Table 25-3: ADSP-2159x\_SC592\_SC594 TIMER Trigger List Generators (Continued)

|   Trigger ID | Name             | Description     | Sensitivity   |
|--------------|------------------|-----------------|---------------|
|          151 | TIMER0_TMR09_MST | TIMER0 Timer 9  | Edge          |
|          152 | TIMER0_TMR10_MST | TIMER0 Timer 10 | Edge          |
|          153 | TIMER0_TMR11_MST | TIMER0 Timer 11 | Edge          |
|          154 | TIMER0_TMR12_MST | TIMER0 Timer 12 | Edge          |
|          155 | TIMER0_TMR13_MST | TIMER0 Timer 13 | Edge          |
|          156 | TIMER0_TMR14_MST | TIMER0 Timer 14 | Edge          |
|          157 | TIMER0_TMR15_MST | TIMER0 Timer 15 | Edge          |

Table 25-4: ADSP-2159x\_SC592\_SC594 TIMER Trigger List Receivers

|   Trigger ID | Name              | Description     | Sensitivity   |
|--------------|-------------------|-----------------|---------------|
|          125 | TIMER0_TMR00_SLV0 | TIMER0 Timer 0  | Pulse         |
|          126 | TIMER0_TMR00_SLV1 | TIMER0 Timer 0  | Pulse         |
|          127 | TIMER0_TMR01_SLV0 | TIMER0 Timer 1  | Pulse         |
|          128 | TIMER0_TMR01_SLV1 | TIMER0 Timer 1  | Pulse         |
|          129 | TIMER0_TMR02_SLV0 | TIMER0 Timer 2  | Pulse         |
|          130 | TIMER0_TMR02_SLV1 | TIMER0 Timer 2  | Pulse         |
|          131 | TIMER0_TMR03_SLV0 | TIMER0 Timer 3  | Pulse         |
|          132 | TIMER0_TMR03_SLV1 | TIMER0 Timer 3  | Pulse         |
|          133 | TIMER0_TMR04_SLV0 | TIMER0 Timer 4  | Pulse         |
|          134 | TIMER0_TMR04_SLV1 | TIMER0 Timer 4  | Pulse         |
|          135 | TIMER0_TMR05_SLV0 | TIMER0 Timer 5  | Pulse         |
|          136 | TIMER0_TMR05_SLV1 | TIMER0 Timer 5  | Pulse         |
|          137 | TIMER0_TMR06_SLV0 | TIMER0 Timer 6  | Pulse         |
|          138 | TIMER0_TMR06_SLV1 | TIMER0 Timer 6  | Pulse         |
|          139 | TIMER0_TMR07_SLV0 | TIMER0 Timer 7  | Pulse         |
|          140 | TIMER0_TMR07_SLV1 | TIMER0 Timer 7  | Pulse         |
|          141 | TIMER0_TMR08_SLV0 | TIMER0 Timer 8  | Pulse         |
|          142 | TIMER0_TMR08_SLV1 | TIMER0 Timer 8  | Pulse         |
|          143 | TIMER0_TMR09_SLV0 | TIMER0 Timer 9  | Pulse         |
|          144 | TIMER0_TMR09_SLV1 | TIMER0 Timer 9  | Pulse         |
|          145 | TIMER0_TMR10_SLV0 | TIMER0 Timer 10 | Pulse         |

Table 25-4: ADSP-2159x\_SC592\_SC594 TIMER Trigger List Receivers (Continued)

|   Trigger ID | Name              | Description     | Sensitivity   |
|--------------|-------------------|-----------------|---------------|
|          146 | TIMER0_TMR10_SLV1 | TIMER0 Timer 10 | Pulse         |
|          147 | TIMER0_TMR11_SLV0 | TIMER0 Timer 11 | Pulse         |
|          148 | TIMER0_TMR11_SLV1 | TIMER0 Timer 11 | Pulse         |
|          149 | TIMER0_TMR12_SLV0 | TIMER0 Timer 12 | Pulse         |
|          150 | TIMER0_TMR12_SLV1 | TIMER0 Timer 12 | Pulse         |
|          151 | TIMER0_TMR13_SLV0 | TIMER0 Timer 13 | Pulse         |
|          152 | TIMER0_TMR13_SLV1 | TIMER0 Timer 13 | Pulse         |
|          153 | TIMER0_TMR14_SLV0 | TIMER0 Timer 14 | Pulse         |
|          154 | TIMER0_TMR14_SLV1 | TIMER0 Timer 14 | Pulse         |
|          155 | TIMER0_TMR15_SLV0 | TIMER0 Timer 15 | Pulse         |
|          156 | TIMER0_TMR15_SLV1 | TIMER0 Timer 15 | Pulse         |

## Timer Block Diagram

The Timer Block Diagram shows all of the possible clock sources.

Figure 25-1: Timer Block Diagram

<!-- image -->

## Internal Interface

The processor core always accesses the timer registers through the MMR access bus. Hardware ensures that all read and write operations from and to 32-bit timer registers are atomic. Every timer has a dedicated data interrupt request. There is also one common timer status and error interrupt request output that connects to the system event controller. Whenever a data interrupt request is generated, a data trigger generator pulse is also driven out, if

enabled. Each timer has an individual trigger input line, and each timer can be either started or stopped as a trigger receiver.

In total, the GP timer module can have up to (N + 1) interrupt request output lines and N data trigger lines.

## Internal Timer Connections

The Timers support alternate inputs for the clock/capture (see External Interface). Some signals have internal default alternate connections according to the Timer Signal Routing table.

Table 25-5: Timer Signal Routing

| Timer Signal   | Connection   |
|----------------|--------------|
| TM0_ACLK0      | SYS_CLKIN0   |
| TM0_ACI5       | DAI0_PB04    |
| TM0_ACLK5      | DAI0_PB03    |
| TM0_ACI6       | DAI1_PB04    |
| TM0_ACLK6      | DAI1_PB03    |
| TM0_ACI07      | CNT0_TO      |
| TM0_ACLK7      | SYS_CLKIN1   |
| TM0_ACI8       | DAI0_PB06    |
| TM0_ACLK8      | DAI0_PB05    |
| TM0_ACI9       | DAI1_PB06    |
| TM0_ACLK9      | DAI1_PB05    |
| TM0_ACI14      | DAI0 Group C |
| TM0_ACI15      | DAI1 Group C |

## External Interface

Each GP timer module can support up to 16 individual timers. However, most processors have less than this number. The exact number of timers available on a given processor is available in the data sheet for the processor.

Every timer has one main input/output signal ( TIMER\_TMR[nn] ) and, usually, one auxiliary input pin, used as an alternate capture input ( TIMER\_ACI[nn] ). Each timer can either run with a time base of SCLK0 or can reference an external clock on one of two ( TIMER\_ACLK[nn] ) pins. The TMR\_ALT\_CLK0 signal maps to individual alternate clock (( TIMER\_ACLK[nn] ) pins for one or more timers. For instance, a TM\_ACLK3 pin would provide an alternate site to supply an external signal that would serve as reference clock for TMR3. Likewise, the TMR\_ALT\_CLK1 signal from each timer unit connects internally to provide a single global timer clock pin ( TIMER\_CLK ) for the GP timer module. It is used as an additional time base.

## GP Timer Operating Modes

The following sections provide information on the various operating modes of the GP timer.

## General Operation

The core of every timer is a 32-bit counter that can be interrogated through the read-only TIMER\_TMR[n]\_CNT register. Once the module enables a timer, it loads the timer TIMER\_TMR[n]\_CNT register with a starting value.

A timer can operate in one of several different modes, configured through the TIMER\_TMR[n]\_CFG register for that timer. These modes are: PWMOUT, EXTCLK, WIDCAP , WATCHDOG, PININT, and IDLE. The Timer Mode Descriptions table summarizes the modes.

Table 25-6: Timer Mode Descriptions

| Timer Mode   | Description                                                                                                                                                                                  |
|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PWMOUT       | Generates single or continuous PWMwaveforms with programmable pulse width, period, and delay                                                                                                 |
| EXTCLK       | Counts edges of an externally applied waveform                                                                                                                                               |
| WIDCAP       | Captures pulse width or period of an externally applied waveform                                                                                                                             |
| WATCHDOG     | Monitors pulse width or period of an external signal and compares against a window of accepta- ble values, optionally generating an interrupt when it falls inside or outside of that window |
| PININT       | Can generate an interrupt request on an active edge applied to a timer pin                                                                                                                   |
| IDLE         | Idle; no activity                                                                                                                                                                            |

## Period, Width and Delay Register Interaction

When the timer is started, writes to the buffer registers are immediately copied through to the double-buffered period, pulse width, and delay registers. These values are then ready for use in the first timer period. When a timer is already running, software can write new values to the TIMER\_TMR[n]\_PER , TIMER\_TMR[n]\_WID , and TIMER\_TMR[n]\_DLY registers. The written values are buffered and do not update into the registers until the end of the current period. (The update occurs when the value in the TIMER\_TMR[n]\_CNT register equals the value in the TIMER\_TMR[n]\_PER register.)

If new values are not written to these registers, the value from the previous period is reused. Writes to these registers are atomic; it is not possible for the high word to be written without the low word also being written. Values written to the period, pulse width, and delay registers are always stored in the buffer registers. Reads from the same register always return the current, active value of period, pulse width, or delay value. Written values are not readback until they become active.

The usage of the TIMER\_TMR[n]\_PER , TIMER\_TMR[n]\_WID , and TIMER\_TMR[n]\_DLY registers varies, depending on the mode of the timer specified by the TIMER\_TMR[n]\_CFG.TMODE bits. See the Usage of the Period, Width, and Delay Registers in Different Timer Modes table for more information.

Table 25-7: Usage of the Period, Width, and Delay Registers in Different Timer Modes

| Timer Mode   | TIMER_TMR[n]_PER   | TIMER_TMR[n]_WID   | TIMER_TMR[n]_DLY   |
|--------------|--------------------|--------------------|--------------------|
| IDLE         | Not writable       | Not writable       | Not writable       |

Table 25-7: Usage of the Period, Width, and Delay Registers in Different Timer Modes  (Continued)

| Timer Mode   | TIMER_TMR[n]_PER                                                                                                                                                              | TIMER_TMR[n]_WID                                                                                                                                                              | TIMER_TMR[n]_DLY                                                                                                                                                                |
|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| WATCHDOG     | Update on-the-fly. New value takes effect either at timer start or when an asserting edge on the input sig- nal is sensed.                                                    | Read-only. Retains value of last measured width or period of the in- put signal.                                                                                              | Update on-the-fly. New value takes ef- fect either at timer start or when an as- serting edge on the input signal is sensed.                                                    |
| WIDCAP       | Read-only. Period value captured at the appropriate time and updated from its buffer register simultane- ously with the Width register.                                       | Read-only. Width value captured at the appropriate time and updated from its buffer register simultane- ously with the Period register.                                       | Not used                                                                                                                                                                        |
| PWMOUT       | Update on-the-fly. New value takes effect either at timer start or at the end of the current period. A write followed by immediate read returns the current operating values. | Update on-the-fly. New value takes effect either at timer start or at the end of the current period. A write followed by immediate read returns the current operating values. | Update on-the-fly. New value takes ef- fect either at timer start or at the end of the current period. A write followed by immediate read returns the current operating values. |
| EXTCLK       | Can be updated on-the-fly.                                                                                                                                                    | Not used                                                                                                                                                                      | Not used                                                                                                                                                                        |
| PININT       | Not used                                                                                                                                                                      | Not used                                                                                                                                                                      | Not used                                                                                                                                                                        |

If any of the period, pulse width, and delay registers are not used, then programs cannot write into that register. For example, in WIDCAP mode, the delay registers are not used. So, the program is not allowed to write any value to the TIMER\_TMR[n]\_DLY register. T o prevent undesired operation, program the TIMER\_TMR[n]\_CFG.TMODE bits before programming the period, width, or delay registers.

If a program changes the TIMER\_TMR[n]\_CFG.TMODE bits from a status register to writable register (for example in PWMOUT mode), hardware does not clear these registers. These values are automatically overwritten by new values specified by software.

In PWMOUT mode with small periods, there may not be enough time between updates from the buffer registers to write these registers. The next period can use one old value and one new value. To prevent (width + pulse delay) &gt; period errors, write the width and delay registers before the period register when decreasing the values. Write the period register before the width and delay registers when increasing the value.

## Single-Pulse PWMOUT Mode

In single-pulse PWMOUT mode, the timer generates a single pulse on the TIMER\_TMR[nn] pin. This mode is frequently used to implement a precise delay, often with generating an output trigger. The timer module uses the value in the TIMER\_TMR[n]\_DLY register to control the assertion of a pulse. The value in the TIMER\_TMR[n]\_WID register defines the pulse width. The TIMER\_TMR[n]\_PER is not used and cannot be written in this mode. After completion of the pulse, the timer is automatically stopped, and optionally generates an interrupt. The timer uses the TIMER\_TMR[n]\_CFG.PULSEHI bit to control pulse polarity.

The timer can be configured to generate a data interrupt request after satisfying various conditions specified by the TIMER\_TMR[n]\_CFG.IRQMODE bits.

It is not necessary to clear the relevant TIMER\_RUN bit to stop the timer cleanly. At the end of the pulse, the timer stops automatically and the corresponding TIMER\_RUN bit is cleared. T o generate multiple discrete pulses (as opposed to a continuous PWM waveform), write a 1 to the appropriate TIMER\_RUN bit, and wait for the timer to stop. Then, write another 1 to the same TIMER\_RUN bit.

## Continuous PWMOUT Mode

In continuous PWMOUT mode, the timer generates a repetitive pulse with a well-defined period, duty cycle, and pulse position. The TIMER\_TMR[n]\_DLY , TIMER\_TMR[n]\_PER , and TIMER\_TMR[n]\_WID registers are programmed with the values of the required PWM pulse. After the timer is started, the counter counts towards the value programmed in the TIMER\_TMR[n]\_PER register. Initially, the TIMER\_TMR[nn] pin remains in a deasserted state. The pin toggles to an asserted state when the value in the TIMER\_TMR[n]\_CNT register equals the value in the TIMER\_TMR[n]\_DLY register.

The timer can control the assertion sense of the TIMER\_TMR[nn] pin with the TIMER\_TMR[n]\_CFG.PULSEHI bit. The TIMER\_TMR[nn] pin holds this value for the number of clock cycles specified in the TIMER\_TMR[n]\_WID register. Then, the pin deasserts and holds this value until the completion of the programmed period. The same waveform is generated repeatedly until the timer is disabled.

The timer can be configured to generate a data interrupt request after satisfying any of various conditions specified by the TIMER\_TMR[n]\_CFG.IRQMODE bits.

It is important to guarantee that the programmed period is greater than or equal to the sum of width and delay. Similarly, delay must be less than period. Violating either of these criteria results in an unpredictable waveform on the TIMER\_TMR[nn] pin until the situation is rectified by writing proper values to these registers.

The maximum frequency possible to generate on the TIMER\_TMR[nn] pin is achieved by setting TIMER\_TMR[n]\_PER to 2 and TIMER\_TMR[n]\_WID to 1. This operation makes the TIMER\_TMR[nn] pin toggle each SCLK0 clock cycle (assuming the timer is configured to clock internally), producing a duty cycle of 50%.

When the TIMER\_STOP\_CFG.TMR[nn] bit of a timer is 0, the timer treats a stop operation as a stop-is-pending condition. When terminated with this setting, the timer automatically completes the current waveform and then stops cleanly, remaining in a deasserted state. This funtionality prevents truncation of the current pulse and unwanted PWM patterns at the TIMER\_TMR[nn] pin. The processor can determine when the timer stops running by polling the corresponding TIMER\_RUN.TMR[nn] bit until it reads 0 or by waiting for the last interrupt (if enabled).

Figure 25-2: Signal Generation in Continuous PWMOUT Mode

<!-- image -->

The TIMER\_TMR[n]\_CFG register cannot be reconfigured until after the timer stops and the TIMER\_RUN register reads 0.

Programs can force a timer to stop immediately in PWMOUT mode by writing a 1 to the TIMER\_STOP\_CFG register followed by writing a 1 to the TIMER\_RUN\_CLR register. (Or a program can stop a timer by writing a 0 to the appropriate TIMER\_RUN.TMR[nn] bit.) This operation stops the timer whether the pending stop is waiting for the end of the current period or the end of the current pulse width. The timer can use this feature to regain immediate control of a timer during an error recovery sequence.

Use this feature carefully, as it can corrupt the PWM pattern generated at the TIMER\_TMR[nn] pin, though after such a stop the pin deasserts automatically. Each timer samples its TIMER\_RUN.TMR[nn] bit at the end of each period. It stops cleanly at the end of the first period after the TIMER\_RUN.TMR[nn] bit is low. A timer that is disabled and then restarted (before the end of the current period), continues to run as if nothing happened. Typically, the program disables a PWMOUT timer and then waits for it to stop itself.

## Width Capture (WIDCAP) Mode

The timer uses WIDCAP mode, often called capture mode, to measure pulse widths on the TIMER\_TMR[nn] or ( TIMER\_ACI[nn] ) inputs. The polarity (active high or low) of the input signal can be selected with the TIMER\_TMR[n]\_CFG.PULSEHI bit. The Timer Signal Flow in Width Capture Mode figure shows the control signal flow for WIDCAP\_CAP mode.

Figure 25-3: Timer Signal Flow in Width Capture Mode

<!-- image -->

NOTE: SCLK in the Timer Signal Flow in Width Capture Mode figure is SCLK0.

In this mode, the timer uses the TIMER\_TMR[n]\_CFG.TINSEL bit to select between the TIMER\_TMR[nn] or( TIMER\_ACI[nn] ) input. The internally clocked timer is used to determine the period and pulse width of the externally applied rectangular waveforms.

When a timer is enabled in this mode, the timer resets the count in its TIMER\_TMR[n]\_CNT register to 0x0000 0001. It does not start counting until it detects a leading edge on the selected input pin.

When the timer detects the first leading edge, it starts incrementing. When it detects a trailing edge of a waveform, it captures the current 32-bit value of its TIMER\_TMR[n]\_CNT register into its width buffer register. At the next leading edge, the timer transfers the current 32-bit value of its TIMER\_TMR[n]\_CNT register into its period buffer register. The TIMER\_TMR[n]\_CNT register is reset to 0x0000 0001 again, and the timer continues counting and capturing until it is disabled.

In this mode, programs can measure both the pulse width and the pulse period of a waveform. The timer does not use the TIMER\_TMR[n]\_DLY register in this mode. The timer uses the TIMER\_TMR[n]\_CFG.PULSEHI bit to control the definition of leading edge and trailing edge of the TIMER\_TMR[nn] /( TIMER\_ACI[nn] ) pin.

In WIDCAP mode, the following events always occur at the same time as one unit:

1. The TIMER\_TMR[n]\_PER register is updated from the period buffer register.
2. The TIMER\_TMR[n]\_WID register is updated from the width buffer register.
3. The TIMER\_DATA\_ILAT.TMR[nn] bit is set (if enabled).
4. A timer data trigger pulse is generated (if enabled).

The TIMER\_TMR[n]\_CFG.TMODE bit 0 controls the point in time at which this set of events is executed. Taken together, these four events are called a measurement report. The TIMER\_STAT\_ILAT register is not set at a

measurement report. A measurement report occurs, at most, once per input signal period. The current TIMER\_TMR[n]\_CNT value is always copied to the width buffer and period buffer registers at the trailing and leading edges of the input signal, respectively. But these values are not visible to software. A measurement report event samples the captured values into visible registers and sets the timer interrupt request to signal that the TIMER\_TMR[n]\_PER and the TIMER\_TMR[n]\_WID registers are ready to be read.

When the TIMER\_TMR[n]\_CFG.TMODE bit =b#1011, the measurement report occurs just after the width buffer register captures its value at a falling edge. Then, the TIMER\_TMR[n]\_WID register reports the pulse width measured in the pulse that has ended, but the TIMER\_TMR[n]\_PER register reports the pulse period measured at the end of the previous period. If only the first trailing edge has occurred, then the first period value has not yet been measured at the first measurement report. So, the period value is not valid. A read of the TIMER\_TMR[n]\_PER value in this case returns 0. See the Example of Width Capture Deasserted Mode (TMODE=b#1011) figure for more information.

Figure 25-4: Example of Width Capture Deasserted Mode (TMODE=b#1011)

<!-- image -->

NOTE: SCLK in the Example of Width Capture Deasserted Mode (TMODE=b#1011) figure is SCLK0.

When the TIMER\_TMR[n]\_CFG.TMODE bit =b#1010, the measurement report occurs just after the period buffer register captures its value at a leading edge. Then, the TIMER\_TMR[n]\_PER and TIMER\_TMR[n]\_WID registers report the pulse period and pulse width measured in the period that has ended. Refer to the Example of Width Capture Asserted Mode (TMODE=b#1010) figure for more information.

Figure 25-5: Example of Width Capture Asserted Mode (TMODE=b#1010)

<!-- image -->

NOTE: SCLK in the Example of Width Capture Asserted Mode (TMODE=b#1010) figure is SCLK0.

To measure the pulse width of a waveform that has only one leading edge and one trailing edge, set TIMER\_TMR[n]\_CFG.TMODE = b#1011. When TIMER\_TMR[n]\_CFG.TMODE = b#1010, no period value is captured in the period buffer register. Instead, the timer generates an error report interrupt request (if enabled) when the TIMER\_TMR[n]\_CNT range is exceeded and the counter wraps around. In this case, both the TIMER\_TMR[n]\_PER and TIMER\_TMR[n]\_WID registers read 0 (because no measurement report occurred to copy the value captured in the width buffer register to the TIMER\_TMR[n]\_WID register).

When using the TIMER\_TMR[n]\_CFG.TMODE bit =b#1010 mode to measure the width of a single pulse, programs can disable the timer after taking the interrupt that ends the measurement interval. When desired, restart the timer as appropriate in preparation for another measurement. This procedure prevents the timer from freely running after the width measurement and logging errors generated by the timer count overflowing.

## Width Capture Mode Overflow

A timer status interrupt request (when enabled) is generated when the TIMER\_TMR[n]\_CNT register wraps around from 0xFFFF FFFF to 0 in the absence of a leading edge. At that point, the TIMER\_STAT\_ILAT bit is set and the TIMER\_ERR\_TYPE bits change to indicate a count overflow due to a period greater than the range of the counter. This indication is referred to as an error report. A data interrupt request in WIDCAP mode indicates that a new measurement is ready to be read (a measurement report). Similarly, an interrupt request on the timer status interrupt line (shared interrupt request for all timers) indicates an overflow error when generated in this mode.

The TIMER\_TMR[n]\_PER and TIMER\_TMR[n]\_WID registers are never updated at the time an overflow error is signaled. If the timer overflows and the TIMER\_TMR[n]\_CFG.TMODE bit =b#1010, the TIMER\_TMR[n]\_PER and TIMER\_TMR[n]\_WID registers are not updated. If the timer overflows and the

TIMER\_TMR[n]\_CFG.TMODE bit =b#1011, the TIMER\_TMR[n]\_PER and TIMER\_TMR[n]\_WID registers are updated only if a trailing edge is detected at a previous measurement report.

Software can count the number of error reports between measurement report interrupt requests to measure input signal periods longer than 0xFFFF FFFF . Each error report interrupt request adds a full 2 32 SCLK0 counts to the total for the period, but the width is ambiguous. Ensure that if software monitors only the status interrupt request, then status interrupt requests from all other timers are masked.

Refer to the Example Timing for Width Capture Followed by Period Overflow (TMR\_CFG.TMODE=b#1010) figure. The period is 0x1 0000 0004, but the pulse width could be either 0x0 0000 0002 or 0x1 0000 0002.

Figure 25-6: Example Timing for Width Capture Followed by Period Overflow (TMR\_CFG.TMODE=b#1010)

<!-- image -->

NOTE: SCLK in the Example Timing for Width Capture Followed by Period Overflow figure is SCLK0.

The waveform applied to the TIMER\_TMR[nn] (or ( TIMER\_ACI[nn] )) pin is not required to have a 50% duty cycle. The minimum input low time is little more than one SCLK0 period. The minimum input high time is a little more than one SCLK0 period. (Refer to the product data sheet for details.) The maximum TIMER\_TMR[nn] input frequency is less than SCLK0/2, with a 50% duty cycle. Under these conditions, the WIDCAP mode timer measures: period =2 and pulse width =1.

Figure 25-7: Example Timing for Width Capture Followed by Period Overflow (TMR\_CFG.TMODE=b#1011)

<!-- image -->

NOTE: SCLK in the Example Timing for Width Capture Followed by Period Overflow figure is SCLK0.

## Windowed Watchdog (WATCHDOG) Modes

In windowed watchdog (WATCHDOG) modes, a timer can take inputs from one of the TIMER\_TMR[nn] pins or the TIMER\_ACI[nn] pins. With this mode, the timer can monitor pulse width (width watchdog mode) or pulse period (period watchdog mode) on the input line. It also compares the measured value against a minimum required value and maximum allowed value and generates an interrupt request appropriately. The timer uses the TIMER\_TMR[n]\_CFG.PULSEHI bit to select polarity of the input signal.

The waveform applied to the input pin in watchdog mode is not required to have a 50% duty cycle. The minimum input pulse low time, high time, and total period specifications are available in the product data sheet.

## Windowed Watchdog Width Mode

In windowed watchdog width mode, the timer counter monitors the pulse width of an input signal on one of the TIMER\_TMR[nn] pins or one of the alternate clock pins ( TIMER\_ACLK[nn] ). Program the minimum pulse width (p MIN ) in the TIMER\_TMR[n]\_DLY register and the maximum pulse width (p MAX ) in the TIMER\_TMR[n]\_PER register. Both values are programmed in terms of number of clock cycles (SCLK0 or alternate clock). The timer can generate an interrupt if the deasserting pulse edge occurs:

- Inside the window (p MIN  &lt; pulse width ≤ p MAX ), or
- Outside the window (pulse width ≤ p MIN  or pulse width &gt; p MAX )

After enabling the timer in this mode, it always starts counting at the asserting edge of the input signal. Any pulse that is already active when the timer is enabled is ignored.

With the TIMER\_TMR[n]\_CFG.IRQMODE bit =b#11, the timer generates an interrupt if the timed pulse width exceeds p MAX , or if the pulse width is less than p MIN. After attaining p MAX , the pulse stays at an active level, and the counter keeps on counting until it sees a deasserting edge. When the input pulse is not active, the counter holds its current value until it again sees an asserting edge, or it restarts. An interrupt can also be generated for when the pulse occurs within the specified window condition, by setting TIMER\_TMR[n]\_CFG.IRQMODE =b#10.

In this mode, a trailing edge on the input pin triggers capturing of pulse width into the TIMER\_TMR[n]\_WID register. During the inactive portion of the input signal, the internal counter does not increment. The Watchdog Width Mode Timing figure shows the signal flow in this mode.

Figure 25-8: Watchdog Width Mode Timing

<!-- image -->

NOTE: SCLK in the Watchdog Width Mode Timing figure is SCLK0.

To check only the upper limit on pulse width (p MAX  but not p MIN ), then program p MIN  as 0 or 1. In such a case, it is better to use TIMER\_TMR[n]\_CFG.IRQMODE =b#11. With TIMER\_TMR[n]\_CFG.IRQMODE = b#10, a pulse width of 1 clock cycle results in an interrupt. For details, see the Windowed Watchdog Width Mode Interpretation table.

Table 25-8: Windowed Watchdog Width Mode Interpretation

| Timer Delay           | Timer Period   | Incoming Pulse Width   | IRQMODE= b#10                                     | IRQMODE= b#11                                                     | Error Interrupt?   |
|-----------------------|----------------|------------------------|---------------------------------------------------|-------------------------------------------------------------------|--------------------|
| 0 or 1                | Anything ≥ 1   | PW = 1                 | Interrupt at deassert- ing edge of input sig- nal | No Interrupt                                                      | No Error Interrupt |
| 0 or 1                | Anything ≥ 1   | PW ≤ TMR_PER           | Interrupt at deassert- ing edge of input sig- nal | No Interrupt                                                      | No Error Interrupt |
| 0 or 1                | Anything ≥ 1   | PW > TMR_PER           | No Interrupt                                      | Interrupt when pulse width exceeds Pmax (Period Register) Val- ue | No Error Interrupt |
| > 1 but ≤ (Period -1) | Anything > 1   | PW ≤ TMR_DLY           | No Interrupt                                      | Interrupt at deassert- ing edge of input sig- nal                 | No Error Interrupt |
| > 1 but ≤ (Period -1) | Anything > 1   | TMR_DLY < PW ≤ TMR_PER | Interrupt at deassert- ing edge of input sig- nal | No Interrupt                                                      | No Error Interrupt |
| > 1 but ≤ (Period -1) | Anything > 1   | PW > TMR_PER           | No Interrupt                                      | Interrupt when pulse width exceeds Pmax (Period Register) Val- ue | No Error Interrupt |
| ≥ Period              | -              | PW ≤ TMR_PER           | Undefined                                         | Undefined                                                         | No Error Interrupt |
|                       | -              | PW > TMR_PER           | Undefined                                         | Undefined                                                         | b#11 Error Type    |
| -                     | 0              | -                      | Undefined                                         | Undefined                                                         | b#10 Error Type    |

## Windowed Watchdog Period Mode

In this mode, the timer monitors the number of clock cycles between two consecutive rising or falling edges of an input signal on one of the TIMER\_TMR[nn] or TIMER\_ACI[nn] pins. Program the required minimum number of clock cycles (t MIN) in the TIMER\_TMR[n]\_DLY register and the required maximum allowed number of clock cycles (t MAX) in the TIMER\_TMR[n]\_PER register. Both values are programmed in terms of number of clock cycles (SCLK0) or alternate time clock ( TIMER\_ACLK[nn] ). The timer can generate an interrupt when two consecutive occurrences of an active edge are:

- Within a specified window (t MIN  &lt; Pulse Period ≤ t MAX), or
- Outside a specified window (pulse width ≤ (t MIN or t MAX  &lt; pulse width)

When the TIMER\_TMR[n]\_CFG.IRQMODE bit =b#11 and the pulse period &gt; t MAX  or is ≤ t MIN, the timer generates an interrupt (if unmasked). After attaining the t MAX  value, the counter keeps on counting until it sees an active edge on the input line. An interrupt can also be generated for when the pulse occurs within the specified

window condition, by setting TIMER\_TMR[n]\_CFG.IRQMODE =b#10. Refer to the Watchdog Period Mode Timing figure for timer functionality in period watchdog mode.

Figure 25-9: Watchdog Period Mode Timing

<!-- image -->

NOTE: SCLK in the Watchdog Period Mode Timing figure is SCLK0.

To check only the upper limit on period (the t MAX  value, not the t MIN  value), program t MIN  as 0 or 1. For details, refer to the Windowed Watchdog Period Mode Interpretation table.

Table 25-9: Windowed Watchdog Period Mode Interpretation

| Timer Delay         | Timer Period   | Incoming Pulse Width             | IRQMODE=b#10                                      | IRQMODE =b#11                                                      | Error Interrupt?   |
|---------------------|----------------|----------------------------------|---------------------------------------------------|--------------------------------------------------------------------|--------------------|
| 0 or 1              | Anything ≥ 2   | Pulse Period ≤ TMR_PER           | Interrupt at deassert- ing edge of input sig- nal | No Interrupt                                                       | No Error Interrupt |
| 0 or 1              | Anything ≥ 2   | Pulse Period > TMR_PER           | No Interrupt                                      | Interrupt when pulse period crosses Pmax (Period Register) val- ue | No Error Interrupt |
| > 1 but ≤ Period -1 | Anything ≥ 2   | Pulse Period ≤ TMR_DLY           | No Interrupt                                      | Interrupt at deassert- ing edge of input sig- nal                  | No Error Interrupt |
| > 1 but ≤ Period -1 | Anything ≥ 2   | TMR_DLY < Pulse Period ≤ TMR_PER | Interrupt at deassert- ing edge of input sig- nal | No Interrupt                                                       | No Error Interrupt |
| > 1 but ≤ Period -1 | Anything ≥ 2   | Pulse Period > TMR_PER           | No Interrupt                                      | Interrupt when pulse width exceeds Pmax (Period Register) val- ue  | No Error Interrupt |

Table 25-9: Windowed Watchdog Period Mode Interpretation (Continued)

| Timer Delay   | Timer Period   | Incoming Pulse Width   | IRQMODE=b#10   | IRQMODE =b#11   | Error Interrupt?   |
|---------------|----------------|------------------------|----------------|-----------------|--------------------|
| ≥ Period      | -              | Pulse Period < TMR_PER | Undefined      | Undefined       | No Error Interrupt |
|               |                | Pulse Period ≥ TMR_PER | Undefined      | Undefined       | b#11 Error Type    |
| -             | 0 or 1         | -                      | Undefined      | Undefined       | b#10 Error Type    |

## Pin Interrupt (PININT) Mode

In PININT mode, any active edges on either the TIMER\_TMR[nn] pin or the ( TIMER\_ACI[nn] ) pin can cause an edge-based interrupt, if enabled. (The timer uses the TIMER\_TMR[n]\_CFG.TINSEL register to select the pin). The event on the input pin can set the TIMER\_DATA\_ILAT.TMR[nn] bit and issue a system interrupt request. Program the TIMER\_TMR[n]\_CFG.PULSEHI bit to change active edge polarity.

Since the interrupt request is generated in the SCLK0 clock domain, the width of the input signal must be more than one SCLK0 period. Along with generating the interrupt request, the timer also generates a trigger pulse (configured using the TIMER\_TRG\_MSK register). Due to the configuration of polarity, glitches can cause the generation of an undesired interrupt request at the input. To avoid this problem, programs must ensure that interrupt requests are unmasked only after configuring the desired polarity.

## External Clock (EXTCLK) Mode

The timer uses EXTCLK mode, sometimes referred to as the counter mode, to count external events (signal edges), on either the TIMER\_TMR[nn] or ( TIMER\_ACI[nn] ) input pin. The timer works as a counter clocked by an external source (the signal at the pin), which can be asynchronous to SCLK0. The current count in the TIMER\_TMR[n]\_CNT register represents the number of leading-edge events detected. The TIMER\_TMR[n]\_PER register is programmed with the value of the maximum timer external count before stopping or issuing an interrupt request or trigger.

The TIMER\_TMR[n]\_CFG.PULSEHI bit determines the polarity of the leading edge on the input pin. The timer uses the TIMER\_TMR[n]\_CFG.TINSEL bit to select whether the event is counted on the TIMER\_TMR[nn] or on the ( TIMER\_ACI[nn] ) pin. The TIMER\_STAT\_ILAT.TMR[nn] and TIMER\_ERR\_TYPE bits are set if one of these conditions is met:

- TIMER\_TMR[n]\_CNT wraps around from 0xFFFF FFFF to 0
- The period = 0 at startup
- TIMER\_TMR[n]\_CNT register rolls over (from count = period to count = 0x1)

The TIMER\_TMR[n]\_WID and TIMER\_TMR[n]\_DLY registers are unused in this mode and must not be written.

The EXTCLK Mode Control Flow figure shows a flow diagram for EXTCLK mode.

Figure 25-10: EXTCLK Mode Control Flow

<!-- image -->

The waveform applied to the input pin is not required to have a 50% duty cycle. The minimum input pulse low time, high time, and total period specifications are available in the product data sheet. Program the period to any value from 1 to (2 32 - 1), inclusive.

After the timer has started, it resets the TIMER\_TMR[n]\_CNT register to 0x0 and then waits for the first leading edge on the input pin. This edge causes TIMER\_TMR[n]\_CNT to be incremented to the value 0x1, and every subsequent leading edge increments it by one. After the TIMER\_TMR[n]\_CNT register reaches the value programmed in the TIMER\_TMR[n]\_PER register, the corresponding TIMER\_DATA\_ILAT bit is set, and an interrupt and trigger are both generated (if enabled). The next leading-edge reloads the TIMER\_TMR[n]\_CNT register with 0x1, and the timer continues counting until it is disabled.

## GP Timer Programming Concepts

Using the features, operating modes, and event control for the GP timer to their greatest potential requires an understanding of some GP timer-related concepts.

## Setting Up Constantly Changing Timer Conditions

This task shows how to use different period, pulse width, and delay settings for each of the first three timer periods after the timer starts.

1. Program the first set of TIMER\_TMR[n]\_PER , TIMER\_TMR[n]\_WID , and TIMER\_TMR[n]\_DLY register values.
2. Enable the timer using the TIMER\_RUN register.
3. Immediately program the second set of TIMER\_TMR[n]\_PER , TIMER\_TMR[n]\_WID , and TIMER\_TMR[n]\_DLY register values, as needed.
4. Wait for the first timer interrupt request.
5. Program the third set of TIMER\_TMR[n]\_PER , TIMER\_TMR[n]\_WID , and TIMER\_TMR[n]\_DLY register values.

Each new setting is then programmed when the preceding timer interrupt request is received.

## Configuring, Enabling, and Disabling One or More Timers

1. Configure the relevant timers for the operating mode and other properties using the TIMER\_TMR[n]\_CFG register.
2. Write a 1 to the representative TIMER\_RUN.TMR[nn] bit. Or use the TIMER\_RUN\_SET register to avoid disturbing the settings of other timers that are not going through configuration. The timer is enabled and operating.
3. To stop one or more timers, first program the TIMER\_STOP\_CFG register to determine whether to stop immediately or gracefully upon receiving a stop command.
4. ADDITIONAL INFORMATION: PWMOUT modes are the only modes where a timer can be configured for graceful termination.
4. Write a 0 to the representative TIMER\_RUN.TMR[nn] bits to stop the timer according to their TIMER\_STOP\_CFG settings. Alternately, write a 1 to the appropriate TIMER\_RUN\_CLR.TMR[nn] bits to avoid disturbing the settings of other timers that are not terminating.

The timers stop.

## Configuring Timer Data and Status Interrupts

1. Configure the TIMER\_TMR[n]\_CFG.IRQMODE bit field with the desired interrupt properties.
2. Unmask the interrupt source.
3. Set the TIMER\_TMR[n]\_CFG.IRQMODE field but leave the interrupt masked at the system level to poll the TIMER\_DATA\_ILAT.TMR[nn] bit of the timer without generating an interrupt.
4. Use the TIMER\_STAT\_IMSK register to generate interrupt requests by overflow or error conditions (incorrect programming values). The timer uses the TIMER\_STAT\_ILAT.TMR[nn] bits to report interrupt errors, when the timer status interrupt source is unmasked.
5. To poll the TIMER\_STAT\_ILAT.TMR[nn] bit of the timer without generating an interrupt, unmask the corresponding bit in the TIMER\_STAT\_IMSK register, but leave the interrupt masked at the system level.

## Configuring the Timer as a Trigger Receiver

The timer can be configured to either start or stop or toggle between these two states on the input trigger pulse depending on the configuration of the TIMER\_TMR[n]\_CFG.TGLTRIG and TIMER\_TMR[n]\_CFG.SLAVETRIG bits.

- If TIMER\_TMR[n]\_CFG.TGLTRIG bit =0 and TIMER\_TMR[n]\_CFG.SLAVETRIG bit =1 then the trigger pulse starts timer, if it is stopped.

- If TIMER\_TMR[n]\_CFG.TGLTRIG bit =0 and TIMER\_TMR[n]\_CFG.SLAVETRIG bit =0 then the trigger pulse stops timer, if it is running.
- When TIMER\_TMR[n]\_CFG.TGLTRIG bit =0, the trigger pulse has no effect when the timer is already in the requested state.

If TIMER\_TMR[n]\_CFG.TGLTRIG bit is 1, the trigger pulse starts the timer if it is stopped, or stops the timer if it is running. The TIMER\_TMR[n]\_CFG.SLAVETRIG bit has no effect on trigger mechanism. In continuous PWMOUT mode, the timer stops gracefully or abruptly depending on the stop mechanism programmed in the TIMER\_STOP\_CFG\_CLR register. In other modes, the timer stops immediately.

## Ordered Trigger Toggle Mode

The timer can be configured to either start or stop the timer on the input trigger pulse depending on the configuration of the TIMER\_TMR[n]\_CFG.TGLTRIG and TIMER\_TMR[n]\_CFG.ORDTGLTRIG bits.

- If TIMER\_TMR[n]\_CFG.TGLTRIG bit is 1 and TIMER\_TMR[n]\_CFG.ORDTGLTRIG bit is 0, then the input trigger pulse on trigger receiver 0 or trigger receiver 1 starts the timer if it is stopped, or stops the timer if it is running.
- If TIMER\_TMR[n]\_CFG.TGLTRIG bit is 1 and TIMER\_TMR[n]\_CFG.ORDTGLTRIG bit is 1, then the input trigger pulse on trigger receiver 0 starts timer if it is stopped. If the timer is running, it stops when the input trigger pulse on trigger receiver 1 is detected.

Trigger receiver 0 cannot toggle the timer from the run to halt state. T rigger receiver 1 cannot toggle the timer from the halt to run state.

NOTE: The timer state is not toggled by writes to the TIMER\_RUN , TIMER\_RUN\_SET , and TIMER\_RUN\_CLR registers in this mode.

## Using the Timer Broadcast Feature

The broadcast feature provides a means to update period, width, and delay registers simultaneously across more than one timer.

1. Enable the appropriate broadcast bits ( TIMER\_TMR[n]\_CFG.BPEREN , TIMER\_TMR[n]\_CFG.BWIDEN are TIMER\_TMR[n]\_CFG.BDLYEN ) for the timers involved in the broadcast. The use of these bits depends on which broadcast registers the timer uses ( TIMER\_BCAST\_PER , TIMER\_BCAST\_WID , or TIMER\_BCAST\_DLY ).
2. Program the TIMER\_BCAST\_PER register (for example), to broadcast the period setting across the multiple timers enabled.

The enabled timers load their TIMER\_TMR[n]\_PER registers with the value specified in the TIMER\_BCAST\_PER register.

3. Repeat Step 2 as needed for the TIMER\_BCAST\_WID and TIMER\_BCAST\_DLY register settings.

## Timer Illegal States

The following sections use these definitions:

- Startup. The first clock period during which the timer counter is running after the timer is started by writing the TIMER\_RUN register.
- Rollover. The time when the current count in TIMER\_TMR[n]\_CNT matches the value in TIMER\_TMR[n]\_PER and the counter is reloaded with the value 1.
- Overflow. The timer counter was incremented instead of doing a rollover when it was holding the maximum count value of 0xFFFF FFFF. The counter does not have a large enough range to express the next greater value and so it erroneously loads a new value of 0x0000 0000.
- Unchanged. No new error.

When the TIMER\_ERR\_TYPE register is designated unchanged, it displays the previously reported error code orb# 00 when there has been no error since this timer was enabled.

When the TIMER\_STAT\_ILAT register is unchanged, it reads 0 when there has been no error or overflow since this timer was enabled. Or, it reads 0 if software has performed a W1C to clear any previous error. If software has not acknowledged a previous error, the TIMER\_STAT\_ILAT register reads 1. Software can read the TIMER\_STAT\_ILAT register to check for errors. If a particular bit of a timer is set in this register, software can then read the TIMER\_ERR\_TYPE register for more information. Once detected, software can W1C the appropriate TIMER\_STAT\_ILAT bit to acknowledge the error.

Read the following tables as:

- In mode \_\_ at event \_\_, · if TIMER\_TMR[n]\_PER is \_\_ and TIMER\_TMR[n]\_WID is \_\_ and TIMER\_TMR[n]\_DLY is \_\_, · then TIMER\_ERR\_TYPE is \_\_ and TIMER\_STAT\_ILAT is \_\_.

Startup error conditions do not prevent the timer from starting. Similarly, overflow and rollover error conditions do not stop the timer. Illegal cases can cause unwanted behavior of the TIMER\_TMR[nn] pins.

NOTE: For PININT mode, the timer does not use error functionality.

## Continuous PWMOUT Mode

Table 25-10: Startup Event

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY                | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|---------------------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| ≤ 1                 | Anything other than peri- od *1 | Anything            | Anything                                | b#10              | Set                            |

Table 25-10: Startup Event (Continued)

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY                                | TIMER_TMR[ n]_WID      | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE                       | TIMER_STAT_ILAT (if enabled)       |
|---------------------|-------------------------------------------------|------------------------|-----------------------------------------|---------------------------------------|------------------------------------|
| ≥ 2                 | Anything including 0, ex- cluding TMR_PER value | Anything in- cluding 0 | ≤ PERIOD                                | Unchanged                             | Unchanged                          |
|                     | Anything including 0                            | Anything in- cluding 0 | > PERIOD                                | Unchanged *2 (De- tected at rollover) | Unchanged (Detected at roll- over) |
|                     | Anything                                        | Anything               | > 2 32 - 1                              | b#11                                  | Set                                |
|                     | =Period                                         | =0                     | =Period                                 | No error                              | Unchanged (Detected at roll- over) |

- *1 If delay = period, the #10 error is rolled to a #11 error.
- *2 This case is not detected at startup, but when the timer counts to the complete period.
- *1 In case of graceful termination, this error is not generated.

Table 25-11: Rollover Event

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY                                | TIMER_TMR[ n]_WID      | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE                                               | TIMER_STAT_ILAT (if enabled)   |
|---------------------|-------------------------------------------------|------------------------|-----------------------------------------|---------------------------------------------------------------|--------------------------------|
| ≥ 1                 | Anything                                        | Anything               | Anything                                | b#10[timer running at SCLK0] b#11 [timer running at ALT_CLKx] | Set                            |
| ≥ 2                 | Anything including 0, ex- cluding TMR_PER value | Anything in- cluding 0 | ≤ PERIOD                                | Unchanged                                                     | Unchanged                      |
| ≥ 2                 | Anything including 0, ex- cluding TMR_PER value | Anything >0            | >PERIOD                                 | b#11                                                          | Set                            |
| ≥ 2                 | Anything                                        | Anything               | > 2 32 - 1                              | b#11                                                          | Set                            |
| ≥ 2                 | = Period *1                                     | =0                     | =Period                                 | b#11                                                          | Set                            |
| ≥ 2                 | >Period                                         | =0                     | >Period                                 | Unchanged                                                     | Unchanged                      |

Table 25-12: Overflow Event (On TMR\_PER Register Programming Error Only)

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|--------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| Anything            | Anything           | Anything            | Anything                                | b#01              | Set                            |

## Single Pulse PWMOUT Mode

For single pulse PWMOUT mode, there are no rollover events.

Table 25-13: Startup Event

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY     | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_STAT_ ILAT (if enabled)   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|----------------------|---------------------|-----------------------------------------|---------------------------------|--------------------------------|
| N/A                 | Anything             | == 0                | Anything                                | b#11 *1                         | Set                            |
| N/A                 | Anything including 0 | ≥ 1                 | ≤ 2 32 ˗ 1                              | Unchanged                       | Unchanged                      |
| N/A                 | Anything including 0 | ≥ 1                 | > 2 32 ˗ 1                              | b#11                            | Set                            |

Table 25-14: Overflow Event (On another error, such as DELAY + WIDTH ≥ 2 32 - 1)

|          | TIMER_TMR[n]_DLY   |          | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   |      | TIMER_STAT_ILAT (if enabled)   |
|----------|--------------------|----------|-----------------------------------------|------|--------------------------------|
| Anything | Anything           | Anything | Anything                                | b#01 | Set                            |

## WIDCAP Mode

For WIDCAP mode, the TIMER\_TMR[n]\_PER and TIMER\_TMR[n]\_WID registers are read-only and the TIMER\_TMR[n]\_DLY register is not used. Therefore, no startup or rollover errors are possible.

Table 25-15: Overflow Event

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|--------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| Anything            | N/A                | Anything            | N/A                                     | b#01              | Set                            |

## EXTCLK Mode

Table 25-16: Startup Event

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|--------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| =0                  | N/A                | N/A                 | N/A                                     | b#01              | Set                            |

Table 25-16: Startup Event (Continued)

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|--------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| ≥ 1                 | N/A                | N/A                 | N/A                                     | Unchanged         | Unchanged                      |

Table 25-17: Rollover Event

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|--------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| =0                  | N/A                | N/A                 | N/A                                     | b#01              | Set                            |
| ≥ 1                 | N/A                | N/A                 | N/A                                     | Unchanged         | Unchanged                      |

Table 25-18: Overflow Event (On TMR\_PER Register = 0 Only)

| TIMER_TMR[ n]_PER   | TIMER_TMR[n]_DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|--------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| Anything            | N/A                | N/A                 | N/A                                     | b#01              | Set                            |

## WATCHDOG Events

Table 25-19: Startup Event

| TIMER_TMR[n]_ PER   | TIMER_TMR[n]_ DLY   | TIMER_TMR[ n]_WID             | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE               | TIMER_STAT_ILAT (if enabled)   |
|---------------------|---------------------|-------------------------------|-----------------------------------------|-------------------------------|--------------------------------|
| ≤ Allowed MIN *1    | Anything < PERIOD   | N/A                           | N/A                                     | b#01                          | Set                            |
| > Allowed MIN       | Anything < PERIOD   | N/A                           | N/A                                     | Unchanged                     | Unchanged                      |
| > Allowed MIN       | Anything ≥ PERIOD   | Refer to WATCHDOG Mode tables | Refer to WATCHDOG Mode tables           | Refer to WATCHDOG Mode tables | Refer to WATCHDOG Mode tables  |

- *1 Allowed MIN=0 for width watchdog mode. Allowed MIN=1 for period watchdog mode.

Table 25-20: Rollover Event

| TIMER_TMR[n]_ PER   | TIMER_TMR[n]_ DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|---------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| ≤ Allowed MIN *1    | Anything < PERIOD   | N/A                 | N/A                                     | b#01              | Set                            |

Table 25-20: Rollover Event (Continued)

| TIMER_TMR[n]_ PER   | TIMER_TMR[n]_ DLY   | TIMER_TMR[ n]_WID             | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE               | TIMER_STAT_ILAT (if enabled)   |
|---------------------|---------------------|-------------------------------|-----------------------------------------|-------------------------------|--------------------------------|
| > Allowed MIN       | Anything            | N/A                           | N/A                                     | Unchanged                     | Unchanged                      |
| > Allowed MIN       | Anything ≥ PERIOD   | Refer to WATCHDOG Mode tables | Refer to WATCHDOG Mode tables           | Refer to WATCHDOG Mode tables | Refer to WATCHDOG Mode tables  |

Table 25-21: Overflow Event

| TIMER_TMR[n]_ PER   | TIMER_TMR[n]_ DLY   | TIMER_TMR[ n]_WID   | TIMER_TMR[ n]_WID + TIMER_TMR[ n]_DLY   | TIMER_ERR_ TYPE   | TIMER_STAT_ILAT (if enabled)   |
|---------------------|---------------------|---------------------|-----------------------------------------|-------------------|--------------------------------|
| Anything            | Anything            | N/A                 | N/A                                     | b#01              | Set                            |

## ADSP-2159x\_SC592\_SC594 TIMER Register Descriptions

General-Purpose Timer Block (TIMER) contains the following registers.

Table 25-22: ADSP-2159x\_SC592\_SC594 TIMER Register List

| Name               | Description                       |
|--------------------|-----------------------------------|
| TIMER_BCAST_DLY    | Broadcast Delay Register          |
| TIMER_BCAST_PER    | Broadcast Period Register         |
| TIMER_BCAST_WID    | Broadcast Width Register          |
| TIMER_DATA_ILAT    | Data Interrupt Latch Register     |
| TIMER_DATA_IMSK    | Data Interrupt Mask Register      |
| TIMER_ERR_TYPE     | Error Type Status Register        |
| TIMER_RUN          | Run Register                      |
| TIMER_RUN_CLR      | Run Clear Register                |
| TIMER_RUN_SET      | Run Set Register                  |
| TIMER_STAT_ILAT    | Status Interrupt Latch Register   |
| TIMER_STAT_IMSK    | Status Interrupt Mask Register    |
| TIMER_STOP_CFG     | Stop Configuration Register       |
| TIMER_STOP_CFG_CLR | Stop Configuration Clear Register |
| TIMER_STOP_CFG_SET | Stop Configuration Set Register   |
| TIMER_TMR[n]_CFG   | Timer n Configuration Register    |

Table 25-22: ADSP-2159x\_SC592\_SC594 TIMER Register List (Continued)

| Name             | Description                      |
|------------------|----------------------------------|
| TIMER_TMR[n]_CNT | Timer n Counter Register         |
| TIMER_TMR[n]_DLY | Timer n Delay Register           |
| TIMER_TMR[n]_PER | Timer n Period Register          |
| TIMER_TMR[n]_WID | Timer n Width Register           |
| TIMER_TRG_IE     | Trigger Receiver Enable Register |
| TIMER_TRG_MSK    | Trigger Generator Mask Register  |

## Broadcast Delay Register

For timers with TIMER\_TMR[n]\_CFG.BDLYEN enabled, a write to the TIMER\_BCAST\_DLY register concurrently updates the delay ( TIMER\_TMR[n]\_DLY ) registers of only those timers. A read of the TIMER\_BCAST\_DLY register returns 0x00000000, and no bus error is generated. To read back a written value, read that TMR's TIMER\_TMR[n]\_DLY register.

Figure 25-11: TIMER\_BCAST\_DLY Register Diagram

<!-- image -->

Table 25-23: TIMER\_BCAST\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R0/W)        | VALUE      | Broadcast Delay Value. A write to the TIMER_BCAST_DLY.VALUE bit field concurrently updates the delay ( TIMER_TMR[n]_DLY ) registers of only those timers. A read of the TIMER_BCAST_DLY.VALUE bit field returns 0x0000 0000, and no bus error is generated. |

## Broadcast Period Register

For timers with TIMER\_TMR[n]\_CFG.BPEREN enabled, a write to the TIMER\_BCAST\_PER register concurrently updates the period ( TIMER\_TMR[n]\_PER ) registers of only those timers. A read of TIMER\_BCAST\_PER returns 0x00000000, and no bus error is generated. To read back a written value, read that TMR's TIMER\_TMR[n]\_PER register.

Figure 25-12: TIMER\_BCAST\_PER Register Diagram

<!-- image -->

Table 25-24: TIMER\_BCAST\_PER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R0/W)        | VALUE      | Broadcast Period Value. A write to the TIMER_BCAST_PER.VALUE bit field concurrently updates the peri- od ( TIMER_TMR[n]_PER ) registers of only those timers. A read of the TIMER_BCAST_PER.VALUE bit fields returns 0x0000 0000, and no bus error is generated. |

## Broadcast Width Register

For timers with TIMER\_TMR[n]\_CFG.BWIDEN enabled, a write to the TIMER\_BCAST\_WID register concurrently updates the width ( TIMER\_TMR[n]\_WID ) registers of only those timers. A read of the TIMER\_BCAST\_WID register returns 0x00000000, and no bus error is generated. To read back a written value, read that TMR's TIMER\_TMR[n]\_WID register.

Figure 25-13: TIMER\_BCAST\_WID Register Diagram

<!-- image -->

Table 25-25: TIMER\_BCAST\_WID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R0/W)        | VALUE      | Broadcast Width Value. A write to the TIMER_BCAST_WID.VALUE bit field concurrently updates the width ( TIMER_TMR[n]_WID ) registers of only those timers. A read of the TIMER_BCAST_WID.VALUE bit field returns 0x0000 0000, and no bus error is generated. |

## Data Interrupt Latch Register

The TIMER\_DATA\_ILAT holds the latched interrupt status for interrupt requests that have been unmasked (enabled) by the TIMER\_DATA\_IMSK register and generated according to the conditions selected by the TIMER\_TMR[n]\_CFG.IRQMODE bits. If a bit in TIMER\_DATA\_ILAT is already set and the corresponding interrupt is masked in TIMER\_DATA\_IMSK , the latch holds its old value, leaving the interrupt request asserted until it is reset by software with a W1C operation.

Note that interrupt service routines (ISRs) should clear the appropriate bits in TIMER\_DATA\_ILAT before returning from the ISR, to ensure that the interrupt is not re-issued. To make sure that no timer event is missed, the latch should be reset at the very beginning of the ISR when in EXTCLK mode.

Figure 25-14: TIMER\_DATA\_ILAT Register Diagram

<!-- image -->

Table 25-26: TIMER\_DATA\_ILAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W1C)       | TMR[nn]    | Data Interrupt Latch. For all TIMER_DATA_ILAT.TMR[nn] bits, status of =0 indicates no interrupt is latched, and status of =1 indicates a latched interrupt (indicating an unmasked inter- rupt request from a timer with a condition matching the one selected with correspond- ing TIMER_TMR[n]_CFG.IRQMODE bit has occurred). |

## Data Interrupt Mask Register

Each timer may generate a unique processor data interrupt request signal. The TIMER\_DATA\_IMSK register contains an interrupt mask for these requests, masking (disabling) or unmasking (enabling) the interrupts as programmed. The reset value of the TIMER\_DATA\_IMSK register is 0xFFFF , masking these interrupts after reset.

Figure 25-15: TIMER\_DATA\_IMSK Register Diagram

<!-- image -->

Table 25-27: TIMER\_DATA\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TMR[nn]    | Data Interrupt Mask. For all TIMER_DATA_IMSK.TMR[nn] bits, write =0 unmasks (enables) the corre- sponding data interrupt request, and write =1 masks (disables) the corresponding data interrupt request. |

## Error Type Status Register

The TIMER\_ERR\_TYPE register contains error type status bits for each timer. These bits indicate the type of error (if any) in a running timer. This register is read-only. These status bits are cleared at reset and when a particular timer is enabled.

Each time an error request interrupt is latched in the TIMER\_STAT\_ILAT register, the corresponding TERRx bits in the TIMER\_ERR\_TYPE register are loaded with a code that identifies the type of error that was detected. This status value is held until the next error or until a particular timer is restarted. No bus error is generated if a write is performed on the TIMER\_ERR\_TYPE register.

Figure 25-16: TIMER\_ERR\_TYPE Register Diagram

<!-- image -->

Table 25-28: TIMER\_ERR\_TYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration               |
|--------------------|------------|---------------------------|---------------------------------------|
| 31:30              | TERR15     | Error Type for Timer 15.  | Error Type for Timer 15.              |
| (R/NW)             |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |

Table 25-28: TIMER\_ERR\_TYPE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration               |
|--------------------|------------|---------------------------|---------------------------------------|
| 29:28 (R/NW)       | TERR14     | Error Type for Timer 14.  | Error Type for Timer 14.              |
| 29:28 (R/NW)       | TERR14     | 0                         | No Error                              |
| 29:28 (R/NW)       | TERR14     | 1                         | Counter Overflow Error                |
| 29:28 (R/NW)       | TERR14     | 2                         | PER Register Programming Error        |
| 29:28 (R/NW)       | TERR14     | 3                         | WID or DLY Register Programming Error |
| 27:26 (R/NW)       | TERR13     | Error Type for Timer 13.  | Error Type for Timer 13.              |
|                    |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 25:24 (R/NW)       | TERR12     | Error Type for Timer 12.  | Error Type for Timer 12.              |
|                    |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 23:22 (R/NW)       | TERR11     | Error Type for Timer 11.  | Error Type for Timer 11.              |
|                    |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 21:20 (R/NW)       | TERR10     | Error Type for Timer 10.  | Error Type for Timer 10.              |
|                    |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 19:18              | TERR9      | Error Type for Timer 9.   | Error Type for Timer 9.               |
| (R/NW)             |            | 0                         | No Error                              |
| (R/NW)             |            | 1                         | Counter Overflow Error                |
| (R/NW)             |            | 2                         | PER Register Programming Error        |
| (R/NW)             |            | 3                         | WID or DLY Register Programming Error |

Table 25-28: TIMER\_ERR\_TYPE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration               |
|--------------------|------------|---------------------------|---------------------------------------|
| 17:16 (R/NW)       | TERR8      | Error Type for Timer 8.   | Error Type for Timer 8.               |
| 17:16 (R/NW)       | TERR8      | 0                         | No Error                              |
| 17:16 (R/NW)       | TERR8      | 1                         | Counter Overflow Error                |
| 17:16 (R/NW)       | TERR8      | 2                         | PER Register Programming Error        |
| 17:16 (R/NW)       | TERR8      | 3                         | WID or DLY Register Programming Error |
| 15:14 (R/NW)       | TERR7      | Error Type for Timer 7.   | Error Type for Timer 7.               |
| 15:14 (R/NW)       | TERR7      | 0                         | No Error                              |
| 15:14 (R/NW)       | TERR7      | 1                         | Counter Overflow Error                |
| 15:14 (R/NW)       | TERR7      | 2                         | PER Register Programming Error        |
| 15:14 (R/NW)       | TERR7      | 3                         | WID or DLY Register Programming Error |
| 13:12 (R/NW)       | TERR6      | Error Type for Timer 6.   | Error Type for Timer 6.               |
| 13:12 (R/NW)       | TERR6      | 0                         | No Error                              |
| 13:12 (R/NW)       | TERR6      | 1                         | Counter Overflow Error                |
| 13:12 (R/NW)       | TERR6      | 2                         | PER Register Programming Error        |
| 13:12 (R/NW)       | TERR6      | 3                         | WID or DLY Register Programming Error |
| 11:10 (R/NW)       |            | Error Type for Timer 5.   | Error Type for Timer 5.               |
|                    | TERR5      | 0                         | No Error                              |
|                    | TERR5      | 1                         | Counter Overflow Error                |
|                    | TERR5      | 2                         | PER Register Programming Error        |
|                    | TERR5      | 3                         | WID or DLY Register Programming Error |
| 9:8 (R/NW)         | TERR4      | Error Type for Timer 4.   | Error Type for Timer 4.               |
|                    |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 7:6                | TERR3      | Error Type for Timer 3.   | Error Type for Timer 3.               |
| (R/NW)             |            | 0                         | No Error                              |
| (R/NW)             |            | 1                         | Counter Overflow Error                |
| (R/NW)             |            | 2                         | PER Register Programming Error        |
| (R/NW)             |            | 3                         | WID or DLY Register Programming Error |

Table 25-28: TIMER\_ERR\_TYPE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration               |
|--------------------|------------|---------------------------|---------------------------------------|
| 5:4                | TERR2      | Error Type for Timer 2.   | Error Type for Timer 2.               |
| (R/NW)             |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 3:2                | TERR1      | Error Type for Timer 1.   | Error Type for Timer 1.               |
| (R/NW)             |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |
| 1:0                | TERR0      | Error Type for Timer 0.   | Error Type for Timer 0.               |
| (R/NW)             |            | 0                         | No Error                              |
|                    |            | 1                         | Counter Overflow Error                |
|                    |            | 2                         | PER Register Programming Error        |
|                    |            | 3                         | WID or DLY Register Programming Error |

## Run Register

The TIMER\_RUN allows all timers to be enabled simultaneously, permitting them to run synchronously. For each timer, there is a single start/stop control bit. Writing a 1 to this bit starts the corresponding timer; writing a 0 stops the timer with mechanism specified in the timer stop configuration TIMER\_STOP\_CFG register.

The start/stop control bits can be set/reset individually or in any combination. While starting or stopping one particular timer directly with this register, software must perform a read-modify write, so the bits corresponding to other timers remain unchanged. To avoid this need, software can use the TIMER\_RUN\_CLR register.

Reading the TIMER\_RUN register shows the start status for the corresponding timer. A 1 indicates that the timer is running.

If a timer is in run state (corresponding run bit is =1), a software write of 1 in this bit does not have any effect on the timer state. The write does not result in restarting the timer.

Note that the TIMER\_RUN register is not used in PININT mode. PININT mode starts as soon as the TIMER\_TMR[n]\_CFG.TMODE bits are set to 111.

Figure 25-17: TIMER\_RUN Register Diagram

<!-- image -->

Table 25-29: TIMER\_RUN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TMR[nn]    | Start/Stop Timer n. For all TIMER_RUN.TMR[nn] bits, write =0 for stop, and write =1 for start. Read =1 when timer is running. |

## Run Clear Register

The TIMER\_RUN\_CLR register is an alias register, providing a mechanism to clear a specific start/stop bit in the TIMER\_RUN register without affecting other bits in TIMER\_RUN . To stop a particular timer, software must write a 1 into the corresponding TIMER\_RUN\_CLR bit. Writing a 0 has no effect. Because TIMER\_RUN\_CLR is a writeonly register, the result of any write to this register must be checked by reading the TIMER\_RUN register. A read of the TIMER\_RUN\_CLR returns 0x0000.

Note that the stopping mechanism of a timer may be abrupt or graceful (after completion of current waveform period) depending on the selection in the TIMER\_STOP\_CFG register.

Figure 25-18: TIMER\_RUN\_CLR Register Diagram

<!-- image -->

Table 25-30: TIMER\_RUN\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R0/W1C)      | TMR[nn]    | RUN Clear Alias. For all TIMER_RUN_CLR.TMR[nn] bits, write =0 has no effect, and write =1 for stop (clearing the corresponding in start/stop bit in the TIMER_RUN register). Using TIMER_RUN_CLR to clear start/stop bits permits stopping specific timers without in- fluencing run status of other timers. |

## Run Set Register

The TIMER\_RUN\_SET register is an alias register, providing a mechanism to set a specific start/stop bit in the TIMER\_RUN register without affecting other bits in TIMER\_RUN . To start a particular timer, software must write a 1 into the corresponding TIMER\_RUN\_SET bit. Writing a zero has no effect. For an example, to start timer 3 without affecting any other timer, write 0x0008 into TIMER\_RUN\_SET . Because TIMER\_RUN\_SET is a writeonly register, the result of any write to this register must be checked by reading the TIMER\_RUN register. A read of the TIMER\_RUN\_SET returns 0x0000.

Figure 25-19: TIMER\_RUN\_SET Register Diagram

<!-- image -->

Table 25-31: TIMER\_RUN\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R0/W1S)      | TMR[nn]    | RUN Set Alias. For all TIMER_RUN_SET.TMR[nn] bits, write =0 has no effect, and write =1 for start (setting the corresponding start/stop bit in the TIMER_RUN register). Using TIMER_RUN_SET to set start/stop bits permits starting specific timers without influ- encing the run status of other timers. |

## Status Interrupt Latch Register

The TIMER\_STAT\_ILAT holds the latched interrupt status for error interrupt requests, indicating a timer overflow condition or indicating that prohibited programming has occurred for a timer. These interrupt status bits are sticky and are W1C. The bits in the TIMER\_STAT\_ILAT register provide information regarding each timer interrupt request source.

Figure 25-20: TIMER\_STAT\_ILAT Register Diagram

<!-- image -->

Table 25-32: TIMER\_STAT\_ILAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W1C)       | TMR[nn]    | Status Interrupt Latch. For all TIMER_STAT_ILAT.TMR[nn] bits, status of 0 indicates no error interrupt request is latched, and status of 1 indicates a timer counter overflow or programming error interrupt request is latched. |

## Status Interrupt Mask Register

While each timer may generate a status interrupt request, these requests are OR'ed to generate a single status interrupt signal to the system event controller. The TIMER\_STAT\_IMSK register contains an interrupt mask for these requests, masking (disabling) or unmasking (enabling) the interrupts as programmed. The reset value of the TIMER\_STAT\_IMSK register is 0xFFFF , masking these interrupts after reset.

Figure 25-21: TIMER\_STAT\_IMSK Register Diagram

<!-- image -->

Table 25-33: TIMER\_STAT\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TMR[nn]    | Status Interrupt Mask. For all TIMER_STAT_IMSK.TMR[nn] bits, write =0 unmasks (enables) the corre- sponding status interrupt request, and write =1 masks (disables) the corresponding sta- tus interrupt request. |

## Stop Configuration Register

The TIMER\_STOP\_CFG register selects the stop mode for each timer. Timers may be stopped abruptly (immediate halt - all modes) or gracefully in PWMOUT modes (single pulse and continuous). The halt is achieved through either a write =0 to the corresponding bit in TIMER\_RUN or a write =1 to the corresponding bit in TIMER\_RUN\_CLR . A read of TIMER\_STOP\_CFG returns the last value written.

Figure 25-22: TIMER\_STOP\_CFG Register Diagram

<!-- image -->

Table 25-34: TIMER\_STOP\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TMR[nn]    | Stop Mode Select. For all TIMER_STOP_CFG.TMR[nn] bits, write =0 for graceful termination (PWMOUT modes only), and write =1 for abrupt (immediate halt) on stop. |

## Stop Configuration Clear Register

This is an alias register, providing a mechanism to clear a specific bit in the TIMER\_STOP\_CFG register without affecting other bits in TIMER\_STOP\_CFG . To clear a bit in TIMER\_STOP\_CFG , software must write a 1 to the corresponding bit of TIMER\_STOP\_CFG\_CLR register. Writing a zero has no effect. Because the TIMER\_STOP\_CFG\_CLR register is a write-only register, the result of any write to this register must be checked by reading the TIMER\_STOP\_CFG register. A read of the TIMER\_STOP\_CFG\_CLR register returns 0x0000.

Figure 25-23: TIMER\_STOP\_CFG\_CLR Register Diagram

<!-- image -->

Table 25-35: TIMER\_STOP\_CFG\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R0/W1C)      | TMR[nn]    | STOP_CFG Clear Alias. For all TIMER_STOP_CFG_CLR.TMR[nn] bits, write =0 has no effect, and write =1 for graceful stop in PWMOUTmodes (clearing the corresponding stop mode select bit in the TIMER_STOP_CFG register). Using TIMER_STOP_CFG_CLR to clear stop mode bits permits configuring specific timers without influencing the stop mode configuration of other timers. |

## Stop Configuration Set Register

This is an alias register, providing a mechanism to set a specific bit in the TIMER\_STOP\_CFG register without affecting other bits in TIMER\_STOP\_CFG . To set a bit in the TIMER\_STOP\_CFG register, software must write a 1 to the corresponding bit of the TIMER\_STOP\_CFG\_SET register. Writing a zero has no effect. Because the TIMER\_STOP\_CFG\_SET register is a write-only register, the result of any write to this register must be checked by reading the TIMER\_STOP\_CFG register. A read of the TIMER\_STOP\_CFG\_SET register returns 0x0000.

Figure 25-24: TIMER\_STOP\_CFG\_SET Register Diagram

<!-- image -->

Table 25-36: TIMER\_STOP\_CFG\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R0/W1S)      | TMR[nn]    | STOP_CFG Set Alias. For all TIMER_STOP_CFG_SET.TMR[nn] bits, write =0 has no effect, and write =1 for abrupt stop (setting the corresponding stop mode select bit in the TIMER_STOP_CFG register). Using TIMER_STOP_CFG_SET to set stop mode bits permits configuring specific timers without influencing the stop mode configura- tion of other timers. |

## Timer n Configuration Register

Each timer has a TIMER\_TMR[n]\_CFG register that specifies its operating mode. Only write to a TIMER\_TMR[n]\_CFG register when the corresponding timer is not running.

After disabling a timer operating in PWMOUT mode, verify that the timer has stopped running by checking the start/stop status of the timer in the TIMER\_RUN register before writing to the timer's TIMER\_TMR[n]\_CFG register.

Note that a timer's TIMER\_TMR[n]\_CFG register may be read at any time.

Figure 25-25: TIMER\_TMR[n]\_CFG Register Diagram

<!-- image -->

Table 25-37: TIMER\_TMR[n]\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | ORDTGLTRIG | Ordered Receiver Trigger Toggle Enable. The TIMER_TMR[n]_CFG.ORDTGLTRIG bit controls ordered receive trigger tog- gle mode. The timer(receiver) state is not toggled by writes to the TIMER_RUN , TIMER_RUN_SET and TIMER_RUN_CLR registers when this bit is enabled.                                |
| 16 (R/W)           | TGLTRIG    | the timer if it is running. Receiver Trigger Toggle Enable. The TIMER_TMR[n]_CFG.TGLTRIG bit stops the timer if it is running and starts the timer if it is halted (in the stop state). If the TIMER_TMR[n]_CFG.TGLTRIG bit is set, then the setting of the TIMER_TMR[n]_CFG.SLAVETRIG bit is ignor- |
| 15 (R/W)           | EMURUN     | 1 Receiver trigger toggles timer state Emulation Run. The TIMER_TMR[n]_CFG.EMURUN bit causes the timer to run (count) during emulation.                                                                                                                                                              |
| 14 (R/W)           | BPEREN     | 0 Stop Timer During Emulation 1 Run Timer During Emulation Broadcast Period Enable.                                                                                                                                                                                                                  |
|                    |            | The TIMER_TMR[n]_CFG.BPEREN bit enables updates to the TIMER_TMR[n]_PER register simultaneously across more than one timer. 0 Disable Broadcast to PER Register 1 Enable Broadcast to PER Register                                                                                                   |

Table 25-37: TIMER\_TMR[n]\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | BDLYEN     | Broadcast Delay Enable. The TIMER_TMR[n]_CFG.BDLYEN bit enables updates to the TIMER_TMR[n]_DLY register simultaneously across more than one timer.                                                                                                                                                                                                                                  |
| 11 (R/W)           | OUTDIS     | Output Disable. The TIMER_TMR[n]_CFG.OUTDIS bit enables or disables the timer pin output buffer.                                                                                                                                                                                                                                                                                     |
| 10 (R/W)           | TINSEL     | Timer Input Select (for WIDCAP, WATCHDOG, PININT, EXTCLK modes). 0 Use TMR Pin Input                                                                                                                                                                                                                                                                                                 |
| 9:8 (R/W)          | CLKSEL     | TIMER_TMR[n]_CFG.CLKSEL bit field selects the TIMER clock to use. 0 Use SCLK0 1 Use TMR_ALT_CLK0 as TMR Clock 3 Use TMR_ALT_CLK1 as TMR Clock                                                                                                                                                                                                                                        |
| 7 (R/W)            | PULSEHI    | Clock Select. The                                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | Polarity Response Select. The TIMER_TMR[n]_CFG.PULSEHI bit defines specific behaviors of the timer based on the operating mode. For more information, see the specific operating mode in the Programming Concepts section. 0 Negative Response or Pulse. A Negative Edge Response or Negative Action Pulse on the TMR pin. 1 Positive Response or Pulse. A Positive Edge Response or |
| 6 (R/W)            | SLAVETRIG  | Receiver Trigger Response. The TIMER_TMR[n]_CFG.SLAVETRIG bit controls the trigger response. The trigger pulse has no effect (to stop or start the timer) if the timer is already in the re- quested state.                                                                                                                                                                          |
|                    |            | 0 Pulse Stops Timer if it is Running                                                                                                                                                                                                                                                                                                                                                 |

Table 25-37: TIMER\_TMR[n]\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:4 (R/W)          | IRQMODE    | Interrupt Modes. The TIMER_TMR[n]_CFG.IRQMODE bit field selects the interrupt request mode. Note that any mismatched combination of the TIMER_TMR[n]_CFG.IRQMODE | Interrupt Modes. The TIMER_TMR[n]_CFG.IRQMODE bit field selects the interrupt request mode. Note that any mismatched combination of the TIMER_TMR[n]_CFG.IRQMODE                                                                                                                                                                        |
| 5:4 (R/W)          | IRQMODE    | 0                                                                                                                                                                | Active Edge Mode. The timer generates an interrupt at every active edge. The active edge polarity depends on the state of the TIMER_TMR[n]_CFG.PULSEHI bit. Valid for PININT mode only.                                                                                                                                                 |
| 5:4 (R/W)          | IRQMODE    | 1                                                                                                                                                                | Delay Expired Mode. The timer generates an interrupt when the TIMER_TMR[n]_CNT value reaches the val- ue in the TIMER_TMR[n]_DLY register. This mode is valid for all PWMOUTmodes.                                                                                                                                                      |
| 5:4 (R/W)          | IRQMODE    | 2 is                                                                                                                                                             | Width Plus Delay Expired Mode. The timer generates an interrupt when the TIMER_TMR[n]_CNT value reaches the value in the TIMER_TMR[n]_WID regis- ter plus the value in the TIMER_TMR[n]_DLY regis- ter. (PWMOUT modes only) The timer generates an interrupt if the deasserting edge within the specified window. (WATCHDOG modes only) |
| 3:0 (R/W)          | TMODE      | Timer Mode Select. The TIMER_TMR[n]_CFG.TMODE bit field selects the operating mode of each timer.                                                                | Timer Mode Select. The TIMER_TMR[n]_CFG.TMODE bit field selects the operating mode of each timer.                                                                                                                                                                                                                                       |
| 3:0 (R/W)          | TMODE      | 0-7                                                                                                                                                              | Idle Mode                                                                                                                                                                                                                                                                                                                               |
| 3:0 (R/W)          | TMODE      | 8                                                                                                                                                                | Period Watchdog Mode                                                                                                                                                                                                                                                                                                                    |
| 3:0 (R/W)          | TMODE      | 9                                                                                                                                                                | Width Watchdog Mode                                                                                                                                                                                                                                                                                                                     |

Table 25-37: TIMER\_TMR[n]\_CFG Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration                            |
|-----------|------------|---------------------------|----------------------------------------------------|
| (Access)  |            |                           |                                                    |
|           |            |                        10 | Measurement Report at Asserting Edge of Waveform   |
|           |            |                        11 | Measurement Report at Deasserting Edge of Waveform |
|           |            |                        12 | Continuous PWMOUTMode                              |
|           |            |                        13 | Single Pulse PWMOUTMode                            |
|           |            |                        14 | EXTCLK Mode                                        |
|           |            |                        15 | PININT (pin interrupt) Mode                        |

## Timer n Counter Register

The TIMER\_TMR[n]\_CNT register holds the current timer count. After enabling, the count is re-initialized to either 0x0 or 0x1, depending on the configuration and mode. The TIMER\_TMR[n]\_CNT register is read-only and may be read at any time (whether the timer is running or stopped). Reading the TIMER\_TMR[n]\_CNT register returns an atomic 32-bit value.

Depending on the timer operation mode, the counter increment can be clocked by a number of sources, including SCLK0, the TMR or alternate capture input pins, ACLK[n]. The counter retains its value after the timer is disabled.

Figure 25-26: TIMER\_TMR[n]\_CNT Register Diagram

<!-- image -->

Table 25-38: TIMER\_TMR[n]\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 31:0               | VALUE      | Counter Value.                                                      |
| (R/NW)             |            | The TIMER_TMR[n]_CNT.VALUE bit field holds the current timer count. |

## Timer n Delay Register

The TIMER\_TMR[n]\_DLY register holds the delay value for the corresponding timer. This register's use is based on the selected timer mode.

Figure 25-27: TIMER\_TMR[n]\_DLY Register Diagram

<!-- image -->

Table 25-39: TIMER\_TMR[n]\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Delay Value. The TIMER_TMR[n]_DLY.VALUE bit field holds the delay value for the corre- sponding timer. |

## Timer n Period Register

The TIMER\_TMR[n]\_PER register holds the period value for the corresponding timer. This register's use is based on the selected timer mode.

Figure 25-28: TIMER\_TMR[n]\_PER Register Diagram

<!-- image -->

Table 25-40: TIMER\_TMR[n]\_PER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Period Value. The TIMER_TMR[n]_PER.VALUE bit field holds the period value for the corre- sponding timer. |

## Timer n Width Register

The TIMER\_TMR[n]\_WID register holds the width value for the corresponding timer. This register's use is based on the selected timer mode.

Figure 25-29: TIMER\_TMR[n]\_WID Register Diagram

<!-- image -->

Table 25-41: TIMER\_TMR[n]\_WID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Width Value. The TIMER_TMR[n]_WID.VALUE bit field holds the width value for the corre- sponding timer. |

## Trigger Receiver Enable Register

As a trigger receiver, each timer can generate a unique data trigger pulse signal. The TIMER\_TRG\_IE contains trigger input enable bits for these signals, disabling or enabling the triggers as programmed. The reset value of the TIMER\_TRG\_IE register is 0xFFFF , masking these triggers after reset.

Figure 25-30: TIMER\_TRG\_IE Register Diagram

<!-- image -->

Table 25-42: TIMER\_TRG\_IE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TMR[nn]    | Trigger Input Enable. For all TIMER_TRG_IE.TMR[nn] bits, write =0 disables the corresponding trigger input, and write =1 enables the corresponding trigger input. |

## Trigger Generator Mask Register

As a trigger generator, each timer can generate a unique data trigger pulse signal. The TIMER\_TRG\_MSK register contains a trigger mask for these outputs, masking (disabling) or unmasking (enabling) the triggers as programmed. The reset value of the TIMER\_TRG\_MSK register is 0xFFFF , masking these triggers after reset.

Figure 25-31: TIMER\_TRG\_MSK Register Diagram

<!-- image -->

Table 25-43: TIMER\_TRG\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | TMR[nn]    | Trigger Output Mask. For all TIMER_TRG_MSK.TMR[nn] bits, write =0 unmasks (enables) the corre- sponding data trigger output, and write =1 masks (disables) the corresponding data trigger output. |