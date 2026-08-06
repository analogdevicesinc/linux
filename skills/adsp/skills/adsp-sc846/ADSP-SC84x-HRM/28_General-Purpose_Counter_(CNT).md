## 25   General-Purpose Counter (CNT)

The GP counter converts pulses from incremental position encoders into data that is representative of the actual position of the pulse. This conversion is done by integrating (counting) pulses on one or two inputs. Since integration provides relative position, some devices also feature a zero-position input (zero marker). The GP counter can use the zero position input feature to establish a reference point for verifying that the acquired position does not drift over time. In addition, the GP counter can use the incremental position information to determine speed, if the time intervals are measured.

The GP counter provides flexible ways to establish position information. When used with the GP timer block, the GP counter can allow for the acquisition of coherent position or time stamp information that enables speed calculation.

## GP Counter Features

The GP counter includes the following features:

- 32-bit up or down counter
- Quadrature encode mode (Gray code)
- Binary encoder mode
- Alternative frequency-direction mode
- Timed direction and up or down counting modes
- Zero marker or push-button support
- Capture event timing in association with GP Timer
- Boundary comparison and boundary setting features

## GP Counter Functional Description

The CNT\_UD and CNT\_DG pins accept various forms of incremental inputs. The 32-bit counter processes the inputs. The GP counter uses the CNT\_ZM pin to sense the pressing of a push button.

NOTE: When enabled, the GP counter requires 3 SCLK cycles of initialization before recognizing valid toggles on its input pins.

The three input pins can be filtered (debounced) before the GP counter evaluates them.

The GP counter features a flexible boundary comparison. In all of the operating modes, the counter can be compared to an upper and lower limit. It takes various actions when reaching these limits.

The GP counter has a flexible input selection. Apart from accepting inputs from the CNT0\_UD and CNT0\_DG and PORT pins, the counter can be configured to accept trigger inputs by setting PADS\_PCFG0.CNT0UDSEL and PADS\_PCFG0.CNT0DGSEL bits. Refer to the PADS\_PCFG0 register description in General-Purpose Ports (PORT) chapter for details.

The module can optionally generate an interrupt request to the system through its IRQ line. On many processors, a GP timer module can use an output to generate time stamps on certain events.

## ADSP-2184x CNT Register List

The GP Counter (CNT) provides support for manually controlled rotary controllers, such as the volume wheel on a radio device. This unit also supports industrial encoders.

The CNT converts pulses from incremental position encoders into data that is representative of the actual position. To complete this task, the CNT integrates (counts) pulses on one or two inputs. Because integration provides relative position, a zero position input (zero marker) is usually provided that establishes a reference point, verifying that the acquired position does not drift over time. The incremental position information may also be used to determine speed, if the relevant time intervals are measured. The CNT provides flexible ways to establish position information. When used in conjunction with the General-purpose Timer (TIMER), the CNT allows acquisition of coherent position/time stamp information, enabling speed calculation.

A set of registers govern CNT operations. For more information on CNT functionality, see the CNT register descriptions.

Table 25-1: ADSP-2184x CNT Register List

| Name       | Description             |
|------------|-------------------------|
| CNT_CFG    | Configuration Register  |
| CNT_CMD    | Command Register        |
| CNT_CNTR   | Counter Register        |
| CNT_DEBNCE | Debounce Register       |
| CNT_IMSK   | Interrupt Mask Register |
| CNT_MAX    | Maximum Count Register  |
| CNT_MIN    | Minimum Count Register  |
| CNT_STAT   | Status Register         |

## ADSP-2184x CNT Interrupt List

Table 25-2: ADSP-2184x CNT Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|              9 | CNT0_STAT | CNT0 Status   | Level         |               |

## ADSP-2184x CNT Trigger List

Table 25-3: ADSP-2184x CNT Trigger List Generators

|   Trigger ID | Name      | Description                 | Sensitivity   |
|--------------|-----------|-----------------------------|---------------|
|            5 | CNT0_STAT | CNT0 Status                 | Level         |
|            6 | CNT0_UD   | CNT0 Count Up and Direction | Level         |
|            7 | CNT0_DG   | CNT0 Count Down and Gate    | Level         |
|            8 | CNT0_TO   | CNT0 Output to Timer Block  | Level         |

Table 25-4: ADSP-2184x CNT Trigger List Receivers

|   Trigger ID | Name    | Description                 | Sensitivity   |
|--------------|---------|-----------------------------|---------------|
|            2 | CNT0_UD | CNT0 Count Up and Direction | Pulse         |
|            3 | CNT0_DG | CNT0 Count Down and Gate    | Pulse         |

## GP Counter Operating Modes

The GP counter has the following modes of operation.

1. Quadrature Encoder
2. Binary Encoder
3. Up/Down Counter
4. Direction Counter
5. Timed Direction

With the exception of the timed direction mode, the GP counter can operate with the GP timer block to capture additional timing information (time stamps) associated with events detected by this block.

## Quadrature Encoder Mode

In this mode, the CNT\_UD and CNT\_DG inputs expect a quadrature-encoded signal that is interpreted as a two-bit Gray code. The order of transitions of the CNT\_UD and CNT\_DG inputs determines whether the counter increments or decrements. The CNT\_CNTR register contains the number of transitions that have occurred as shown in the Quadrature Events and Counting Mechanism table. Optionally, an interrupt is generated when both inputs

change within one SCLK cycle. Gray coding prohibits such transitions. Therefore, the CNT\_CNTR register remains unchanged, and an error condition is signaled.

Table 25-5: Quadrature Events and Counting Mechanism

| CNT_COUNTER Reg- ister Value   |   -4 |   -3 |   -2 |   -1 |   0 |   +1 |   +2 |   +3 |   +4 |
|--------------------------------|------|------|------|------|-----|------|------|------|------|
| CDG, CUD Inputs                |   00 |   01 |   11 |   10 |  00 |   01 |   11 |   10 |   00 |

It is possible to reverse the count direction of the Gray-coded signal by enabling the polarity inverter of either the CNT\_UD pin or the CNT\_DG pin. Inverting both pins does not alter the behavior. The GP counter can enable this feature with the CNT\_CFG.CDGINV and CNT\_CFG.CUDINV bits.

As an example, the CNT\_DG and CNT\_UD inputs are 00 and the next transition is to 01. These inputs normally change the counter in increments as shown in the table. If the CNT\_UD polarity is inverted, this condition generates a received input of 01 followed by 00. The results is a decrement of the counter, altering the behavior of the connected hardware.

## Binary Encoder Mode

This mode is almost identical to quadrature encoder mode, with the exception that the CNT\_UD : CNT\_DG inputs expect a binary-encoded signal. The order of transitions of the CNT\_UD and CNT\_DG inputs determines whether the counter increments or decrements. The CNT\_CNTR register contains the number of transitions that have occurred as shown in the Binary Events and Counting Mechanism table. Optionally, an interrupt is generated when the detected code steps by more than 1 (in binary arithmetic) within one SCLK cycle. Such transitions are erroneous. Therefore, the CNT\_CNTR register remains unchanged, and an error condition is signaled.

Table 25-6: Binary Events and Counting Mechanism

| CNT_COUNTER Register Value   |   -4 |   -3 |   -2 |   -1 |   0 |   +1 |   +2 |   +3 |   +4 |
|------------------------------|------|------|------|------|-----|------|------|------|------|
| CDG:CUD Inputs               |   00 |   01 |   10 |   11 |  00 |   01 |   10 |   11 |   00 |

Reversing the CNT\_UD and CNT\_DG pin polarity has a different effect in binary encoder mode than for the quadrature encoder mode. Inverting the polarity of the CNT\_UD pin only or inverting both the CNT\_UD and CNT\_DG pins, results in reversing the count direction.

## Up/Down Counter Mode

In this mode, the counter increments or decrements at every active edge of the input pins. The GP counter uses the CNT\_CFG.CUDINV bit to select an active edge and has the following results.

- If the GP counter module detects an active edge at the CNT\_UD input, the counter increments.
- If the GP counter module detects an active edge at the CNT\_DG input, the counter decrements.
- If simultaneous edges occur on the CNT\_DG and CNT\_UD pins, the counter remains unchanged, and both up-count and down-count events are signaled in the CNT\_STAT register.

## Direction Counter Mode

In this mode, the counter is incremented or decremented at every active edge of the CNT\_DG input pin. The state of the CNT\_UD input determines whether the counter increments or decrements. The GP counter uses the CNT\_CFG.CUDINV bit to select the polarity.

If the GP counter detects an active edge at the CNT\_DG input, the counter value changes by one in the selected direction.

## Timed Direction Mode

In this mode, the counter is incremented or decremented at each SCLK cycle. The state of the CNT\_UD input determines whether the counter increments or decrements. The GP counter uses the CNT\_CFG.CUDINV bit to select the polarity. The CNT\_DG pin can be used to gate the clock. The GP counter uses the CNT\_CFG.CDGINV bit to select the polarity.

## GP Counter Programming Model

The following sections provide information for programming the interface.

## GP Counter General Programming Flow

The following are general guidelines for configuring and enabling the GP counter.

1. Initialize (but do not enable) the GP counter for the desired mode and settings through the CNT\_CFG register.
2. Usually, events of interest are processed using interrupts rather than by polling status bits. In this case, clear all status bits and activate the interrupt generation requests with the CNT\_IMSK register.
3. Configure interrupts at the system level to insure desired interrupt signaling to the system.
4. If timing information is required, set up the relevant GP Timer in width capture mode.
5. Finally, enable interrupt requests and the GP Counter itself using the CNT\_IMSK and CNT\_CFG registers, respectively.

## GP Counter Mode Configuration

The GP counter can use the CNT\_ZM input pin to sense the zero marker output of a rotary device or to detect the pressing of a push button. There are four programming schemes, which are functional in all counter modes:

- Push-button mode
- Zero-marker-zeros-counter mode
- Zero-marker-error mode
- Zero-once mode

## Configuring GP Counter Push-Button Operation

Use the following procedure to configure push-button operation:

1. Set CNT\_IMSK.CZM to enable (unmask) the zero marker interrupt.
2. Select the active edge polarity through the CNT\_CFG.CZMINV bit.
3. Proceed with any other desired configuration steps and enable the peripheral.

An active edge at the CNT\_ZM input sets the CNT\_IMSK.CZM bit.

## Configuring Zero-Marker-Zeros-Counter Mode

The following provides information on configuring zero-marker-zeros-counter mode for the GP counter.

1. Set CNT\_IMSK.CZMZ to enable CNT\_CNTR . The zero marker interrupt zeroes the counter.
2. Set CNT\_CFG.ZMZC to enable ZMZC mode.
3. Select the active edge polarity through the CNT\_CFG.CZMINV bit.
4. Proceed with any other desired configuration steps and enable the peripheral.

This configuration causes an active level at the CNT\_ZM pin to clear the CNT\_CNTR register and keep it cleared until the CNT\_ZM pin is deactivated. In addition, the CNT\_STAT.CZMZ bit is set.

## Configuring Zero-Marker-Error Mode

The GP counter uses this mode to detect discrepancies between counter-value and the zero marker output of certain rotary encoder devices.

1. Set the CNT\_STAT.CZME bit to enable this mode.
2. Select the active edge of the CNT\_ZM pin through the CNT\_CFG.CZMINV bit.
3. Proceed with any other desired configuration steps and enable the peripheral.

When the GP counter detects an active edge at the CNT\_ZM input pin, it compares the four LSBs of the CNT\_CNTR register to zero. If they are not zero, the GP counter uses CNT\_STAT.CZME bit to signal a mismatch.

## Configuring Zero-Once Mode

The GP counter uses this mode to perform an initial reset of the counter-value when it detects an active zero marker. After that, the zero marker is ignored (the counter is no longer reset).

1. Set the CNT\_CMD.W1ZMONCE bit to enable this mode.
2. Select the active edge of the CNT\_ZM pin through the CNT\_CFG.CZMINV bit.
3. Ensure that at least one of the following bits is enabled: CNT\_IMSK.CZM , CNT\_IMSK.CZME , CNT\_IMSK.CZMZ .

4. Proceed with any other desired configuration steps and enable the peripheral.

The CNT\_CNTR register and the CNT\_CMD.W1ZMONCE bit are cleared on the next active edge of the CNT\_ZM pin. Now the CNT\_CMD.W1ZMONCE bit can be read to check whether the event has already occurred.

## Configuring Boundary Auto-Extend Mode

In this mode, hardware modifies the boundary registers ( CNT\_MIN and CNT\_MAX ) whenever the CNT\_CNTR value reaches either of them. The GP counter uses this mode to monitor the widest angle a thumb wheel even if the software did not generate interrupts.

1. Initialize CNT\_CNTR with the desired value.
2. Set both CNT\_MIN and CNT\_MAX to this same value.
3. Configure the CNT\_CFG.BNDMODE field for auto extend mode.
4. Proceed with any other desired configuration steps and enable the peripheral.

The CNT\_MAX register is loaded with the current CNT\_CNTR value when the latter increments beyond the CNT\_MAX value. Similarly, the CNT\_MIN register is loaded with the CNT\_CNTR value when the latter decrements below the CNT\_MIN value. The CNT\_STAT.MAXC and CNT\_STAT.MINC status bits are set when the CNT\_CNTR value matches the respective boundary register value.

## Configuring Boundary Capture Mode

In this mode, the CNT\_CNTR value is latched into the CNT\_MIN register at one detected edge of the CNT\_ZM input pin and latched into the CNT\_MAX boundary register at the opposite edge.

1. To capture the CNT\_ZM pin rising edge into CNT\_MIN and the falling edge into CNT\_MAX , program CNT\_CFG.CZMINV for active high polarity. Conversely, to capture the CNT\_ZM pin falling edge into CNT\_MIN and the rising edge into CNT\_MAX , program CNT\_CFG.CZMINV for active low polarity.
2. Program the CNT\_IMSK.MAXC and CNT\_IMSK.MINC interrupt mask bits according to interrupt generation requirements.
3. Configure the CNT\_CFG.BNDMODE field for boundary capture mode.
4. Proceed with any other desired configuration steps and enable the peripheral.

The CNT\_STAT.MAXC and CNT\_STAT.MINC status bits report the capture event, depending on how interrupt masks are configured.

## Configuring Boundary Compare and Boundary Zero Modes

In these modes, the two boundary registers ( CNT\_MAX and CNT\_MIN ) are compared to the value in the CNT\_CNTR register.

1. Program CNT\_MAX and CNT\_MIN registers with appropriate upper and lower range values.

2. Program the CNT\_IMSK.MAXC and CNT\_IMSK.MINC interrupt mask bits according to interrupt generation requirements.
3. Configure the CNT\_CFG.BNDMODE field for boundary compare mode.
4. Proceed with any other desired configuration steps and enable the peripheral.

If after incrementing, CNT\_CNTR = CNT\_MAX , then the CNT\_STAT.MAXC bit is set. Similarly if after decrementing, CNT\_CNTR = CNT\_MIN , then the CNT\_STAT.MINC bit is set.

Additionally, for boundary zero mode, the counter-value in CNT\_CNTR is set to zero. The CNT\_STAT.MAXC and CNT\_STAT.MINC bits are not set when software updates the CNT\_MAX or CNT\_MIN registers.

## Configuring GP Counter Push-Button Operation

Use the following procedure to configure push-button operation:

1. Set CNT\_IMSK.CZM to enable (unmask) the zero marker interrupt.
2. Select the active edge polarity through the CNT\_CFG.CZMINV bit.
3. Proceed with any other desired configuration steps and enable the peripheral.

An active edge at the CNT\_ZM input sets the CNT\_IMSK.CZM bit.

## GP Counter Programming Concepts

Using the features, operating modes, and event control for the GP counter to their greatest potential requires an understanding of some GP counter-related concepts. Some key aspects to consider are input noise filtering and capturing timing information.

## CNT Input Noise Filtering

In all modes, the three input pins can be filtered to present clean signals to the GP counter logic. The GP counter uses the CNT\_CFG.DEBEN bit to enable or disable this filtering. The Programmable Noise Filtering figure shows the filtering operation for the CNT\_UD pin.

Figure 25-1: Programmable Noise Filtering

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000000_84280865091fc7e856149f2876fb5e4a7a6e07d22d6206cca603977d3b6c2096.png)

The CNT module implements the filtering mechanism using counters for each GP counter pin, where each counter is initialized from the CNT\_DEBNCE.DPRESCALE field. When a transition is detected on a pin, the corresponding counter starts counting up to the programmed number of SCLK cycles. The state of the pin is latched after time t filter  and passed on to the GP counter logic.

The following formula determines the time t filter , given SCLK and the CNT\_DEBNCE.DPRESCALE value, where lower values of CNT\_DEBNCE.DPRESCALE result in shorter debounce delays:

<!-- formula-not-decoded -->

## Capturing Counter Interval and CNT\_CNTR Read Timing

When the count speed is low, it is often useful to capture the time elapsed since the last count event. Program the TIMER\_TMR[n]\_CFG register of the associated GP timer in a width capture mode with the following bit settings.

- TIMER\_TMR[n]\_CFG.PULSEHI = 0
- TIMER\_TMR[n]\_CFG.TMODE = b#1011
- TIMER\_TMR[n]\_CFG.TINSEL = 1

The Capture Intervals figure shows and the following list describes the operation of the GP counter and the GP timer in this mode.

Figure 25-2: Capture Intervals

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000001_89de9d565037061a80813f9792f53dbd82a43ac0e7e70928f48a77c2d7924dce.png)

## NOTE: SCLK in the Capture Intervals figure is SCLK.

1. The CNT\_TO signal generates a pulse every time a count event occurs. In addition, when the processor reads the CNT\_CNTR register, the CNT\_TO signal presents a pulse which is extended (high) until the next count event.
2. The GP timer updates its TIMER\_TMR[n]\_PER register with the period (measured from falling edge to falling edge, because TIMER\_TMR[n]\_CFG.PULSEHI = 0) of the CNT\_TO signal.

3. The TIMER\_TMR[n]\_WID register is updated with the pulse width (the portion where CNT\_TO is low, again because TIMER\_TMR[n]\_CFG.PULSEHI = 0).
4. Both registers are updated at every rising edge of the CNT\_TO signal (because TIMER\_TMR[n]\_CFG.TMODE = b#011).

The TIMER\_TMR[n]\_PER register contains the period between the last two count events. The TIMER\_TMR[n]\_WID register contains the time since the last count event and the read of the CNT\_CNTR register, both measured in SCLK cycles.

Read the CNT\_CNTR register to latch the two time measurements, providing a coherent triplet of information to calculate speed and position.

NOTE: Speed restrictions apply to the use of the CNT\_TO signal. Therefore, programs must not operate at count event rates that are high. For instance, if CNT\_CNTR is incremented or decremented every SCLK cycle (timed direction mode), the CNT\_TO signal is not valid.

## Capturing Time Interval Between Successive Counter Events

When the required timing information is the interval between successive count events, program the associated timer in a width capture mode. Set the TIMER\_TMR[n]\_CFG bit of TIMER\_TMR[n]\_CFG.PULSEHI = 1, TIMER\_TMR[n]\_CFG.TMODE = b#1010 and TIMER\_TMR[n]\_CFG.TINSEL = 1. Typically, this information is sufficient if the speed of GP counter events does not to reach low values.

The Period Register Timing figure shows the operation of the GP counter and the GP timer in this mode.

Figure 25-3: Period Register Timing

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000002_0da30c641044344d88c2a8cbd74a801e8df554599d4e599a6385ac39b72a478b.png)

NOTE: SCLK in the Period Register Timing figure is SCLK.

The CNT\_TO signal generates a pulse every time a count event occurs. The GP timer updates its TIMER\_TMR[n]\_PER register with the period (measured from rising edge to rising edge) of the CNT\_TO signal. The TIMER\_TMR[n]\_PER register is updated at every rising edge of the CNT\_TO signal and contains the number of SCLK cycles that have elapsed since the previous rising edge.

Incidentally, the TIMER\_TMR[n]\_WID register is also updated at the same time, but is of no interest in this mode of operation. If no reads of the CNT\_CNTR register occur between counter events, the TIMER\_TMR[n]\_WID register only contains the width of the CNT\_TO pulse. If a read of CNT\_CNTR has occurred between events, the TIMER\_TMR[n]\_WID register contains the time between the read of CNT\_CNTR and the next event.

This mode can also be used with TIMER\_TMR[n]\_CFG.PULSEHI = 0. In this case, the period of CNT\_TO is measured between falling edges. It results in the same values as in the previous case, only the latching occurs one SCLK cycle later.

## GP Counter Event Control

Eleven events can be signaled to the processor using status information and optional interrupt requests. The GP counter uses the respective bits in the CNT\_IMSK register to enable the interrupt requests. It uses dedicated bits in the CNT\_STAT register to report events. When an interrupt request from the GP counter is serviced, the application software is responsible for correct interpretation of the events. It is recommended to logically AND the content of the CNT\_IMSK and CNT\_STAT registers to identify pending interrupt requests.

Perform a write-one-to-clear (W1C) operation to the CNT\_STAT register to clear the interrupt requests. Hardware does not clear the status bits automatically unless the counter module is disabled.

The following sections describe the events associated with the GP counter.

## Illegal Gray and Binary Code Events

When illegal transitions occur in quadrature encoder or binary encoder modes, the CNT\_STAT.IC bit is set. If enabled by the CNT\_STAT.IC bit, the counter module generates an interrupt request. Set the CNT\_STAT.IC bit only in the quadrature encoder or binary encoder modes.

## Up/Down Count Events

The GP counter uses the CNT\_STAT.UC bit to indicate whether the counter has been incremented. Similarly, the CNT\_STAT.DC bit reports decrements. The two events are independent. For instance, if the counter increments by one and then decrements by two, both bits remain set, even though the resulting counter-value shows a decrement by one.

In up/down counter mode, hardware can detect simultaneous active edges on the CNT\_UD and CNT\_DG inputs. In that case, the CNT\_CNTR remains unchanged, but both the CNT\_STAT.UC and CNT\_STAT.DC bits are set. Interrupt requests for these events can be enabled through the CNT\_IMSK.UC and CNT\_IMSK.DC bits. Use this feature carefully when the counter is clocked at high rates. This suggestion is especially critical when the counter operates in DIR\_TMR mode, as interrupts are generated every SCLK cycle.

These events can also be used for more push buttons, when GP counter features are unnecessary. When up/down counter mode is enabled, the GP counter can use these count events to report interrupts from push buttons that connect to the CNT\_UD and CNT\_DG inputs.

## Zero-Count Events

The CNT\_STAT.CZERO status bit indicates that the CNT\_CNTR has reached a value equal to 0x0000 0000 after an increment or decrement. This bit is not set when the counter value is set to zero by a write to CNT\_CNTR or by setting the CNT\_CMD.W1LCNTZERO bit. If enabled by the CNT\_IMSK.CZERO bit, the GP counter module generates an interrupt request.

## Overflow Events

There are two status bits that indicate whether the signed counter-register has overflowed from a positive to a negative value or conversely. The CNT\_STAT.COV31 bit reports that the 32-bit CNT\_CNTR register has either incremented from 0x7FFF FFFF to 0x8000 0000, or decremented from 0x8000 0000 to 0x7FFF FFFF.

If enabled by the CNT\_IMSK.COV31 bit, an interrupt request is generated. Similarly, in applications where only the lower 16 bits of the counter are of interest, the CNT\_STAT.COV15 status bit reports counter transitions from 0xXXXX 7FFF to 0xXXXX 8000, or from 0xXXXX 8000 to 0xXXXX 7FFF. If enabled by the CNT\_IMSK.COV15 bit, an interrupt request is generated.

## Boundary Match Events

The CNT\_STAT.MINC and CNT\_STAT.MAXC status bits report boundary events as described in Configuring Boundary Capture Mode. These bits are not set if the software updates the CNT\_CNTR , CNT\_MAX , or CNT\_MIN registers or writes to the CNT\_CMD register. The CNT\_IMSK.MINC and CNT\_IMSK.MAXC bits enable interrupt request generation on boundary events.

## Zero Marker Events

The CNT\_STAT.CZM , CNT\_STAT.CZME , and CNT\_STAT.CZMZ bits are associated with zero marker events, as described in Configuring GP Counter Push-Button Operation. Each of these events can optionally generate an interrupt request, when enabled by the corresponding CNT\_IMSK.CZM , CNT\_IMSK.CZME and CNT\_IMSK.CZMZ bits.

## ADSP-2184x CNT Register Descriptions

General-Purpose Counter (CNT) contains the following registers.

Table 25-7: ADSP-2184x CNT Register List

| Name    | Description            |
|---------|------------------------|
| CNT_CFG | Configuration Register |
| CNT_CMD | Command Register       |

Table 25-7: ADSP-2184x CNT Register List (Continued)

| Name       | Description             |
|------------|-------------------------|
| CNT_CNTR   | Counter Register        |
| CNT_DEBNCE | Debounce Register       |
| CNT_IMSK   | Interrupt Mask Register |
| CNT_MAX    | Maximum Count Register  |
| CNT_MIN    | Minimum Count Register  |
| CNT_STAT   | Status Register         |

## Configuration Register

The CNT\_CFG register configures counter modes, configures input pins, and enables the CNT.

Figure 25-4: CNT\_CFG Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000003_8ba69c350b55320174f6b56bf5f3c96b5e4acbbada5ba673f12d53b54ca10e55.png)

Table 25-8: CNT\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | INPDIS     | CUD and CDG Pin Input Disable. The CNT_CFG.INPDIS disables or enables the CNT_UD input pin and the CNT_DG pin. | CUD and CDG Pin Input Disable. The CNT_CFG.INPDIS disables or enables the CNT_UD input pin and the CNT_DG pin. |
| 15 (R/W)           | INPDIS     | 0                                                                                                              | Enable                                                                                                         |
| 15 (R/W)           | INPDIS     | 1                                                                                                              | Pin Input Disable                                                                                              |
| 13:12 (R/W)        | BNDMODE    | Boundary Register Mode. The CNT_CFG.BNDMODE bit field selects the mode for the CNT_MIN and                     | Boundary Register Mode. The CNT_CFG.BNDMODE bit field selects the mode for the CNT_MIN and                     |
| 13:12 (R/W)        | BNDMODE    | 0                                                                                                              | BND_COMP. Boundary Compare Mode                                                                                |
| 13:12 (R/W)        | BNDMODE    | 1                                                                                                              | BND_ZERO. Boundary Zero Mode                                                                                   |
| 13:12 (R/W)        | BNDMODE    | 2                                                                                                              | BND_CAPT. Boundary Capture Mode                                                                                |
| 13:12 (R/W)        | BNDMODE    | 3                                                                                                              | BND_AEXT. Boundary Auto-extend Mode                                                                            |

Table 25-8: CNT\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name                                           | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|----------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | ZMZC                                               | CZM Zeros Counter Enable. The CNT_CFG.ZMZC bit enables or disables level sensitive - active CNT_ZM pin operation to zero the CNT_CNTR register.                                                                                                                                                                                                                                    |
| 10:8 (R/W)         | CNTMODE                                            | 1 Enable Counter Operating Mode. The CNT_CFG.CNTMODE bit field selects the operating mode for the CNT_UD input                                                                                                                                                                                                                                                                     |
| 6 (R/W)            | CZMINV                                             | 5 DIR_TMR. Direction Timer Mode CZM Pin Polarity Invert. The CNT_CFG.CZMINV bit selects the polarity for the CNT_ZM pin. This polarity must be configured before the counter is enabled. It must not change on-the-fly while the counter is enabled. 0 Active High, Rising Edge                                                                                                    |
| 5 (R/W)            | CUDINV CUD Pin Polarity Invert. The CNT_CFG.CUDINV | 1 Active Low, Falling Edge bit selects the polarity for the CNT_UD pin. This polarity                                                                                                                                                                                                                                                                                              |
| 4 (R/W)            |                                                    | must be configured before the counter is enabled. It must not change on-the-fly while the counter is enabled. 0 Active High, Rising Edge 1 Active Low, Falling Edge CDG Pin Polarity Invert. The CNT_CFG.CDGINV bit selects the polarity for the CNT_DG pin. This polarity must be configured before the counter is enabled. It must not change on-the-fly the counter is enabled. |
|                    | CDGINV                                             | while 0 Active High, Rising Edge                                                                                                                                                                                                                                                                                                                                                   |
|                    | 1 Active                                           | Low, Falling Edge                                                                                                                                                                                                                                                                                                                                                                  |

Table 25-8: CNT\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | DEBEN      | Debounce Enable. The CNT_CFG.DEBEN bit enables or disables CNT input debounce filtering opera- tion selected with the CNT_DEBNCE register. 0 Disable 1 Enable |
| 0 (R/W)            | EN         | Counter Enable. The CNT_CFG.EN bit enables or disables CNT operation. 0 Counter Disable 1 Counter Enable                                                      |

## Command Register

The CNT\_CMD register configures the CNT, enabling operations such as zeroing a counter register and copying or swapping boundary registers. These actions are taken by setting the appropriate bit.

Read operations from this register do not return meaningful values, with the exception of the CNT\_CMD.W1ZMONCE bit, where a set bit indicates that the bit has been set by software before, but a zero marker event has not yet been detected on the CNT\_ZM pin yet. For more information, see the CNT functional description.

The CNT\_CNTR , CNT\_MIN , and CNT\_MAX registers can be initialized to zero by setting the CNT\_CMD.W1LCNTZERO , CNT\_CMD.W1LMINZERO , and CNT\_CMD.W1LMAXZERO bits. In addition to clearing registers, the CNT\_CMD register permits modifying the CNT\_MIN and CNT\_MAX boundary registers in a number of ways. The current counter value in the CNT\_CNTR register can be captured and loaded into either of the two boundary registers to create new boundary limits. This operation is performed by setting the CNT\_CMD.W1LMAXCNT and CNT\_CMD.W1LMINCNT bits. Alternatively, the counter can be loaded from CNT\_MAX or CNT\_MIN using the CNT\_CMD.W1LCNTMAX and CNT\_CMD.W1LCNTMIN bits. It is also possible to transfer the current CNT\_MAX value into CNT\_MIN (or conversely) through the CNT\_CMD.W1LMINMAX and CNT\_CMD.W1LMAXMIN bits.

Another counter operation is the ability to only have the zero marker clear the CNT\_CNTR register once. For more information, see the CNT functional description.

It is possible for multiple actions to be performed simultaneously by setting multiple bits in the CNT\_CMD register. However, there are restrictions. The bits associated with each command have been grouped together such that all bits that involve a write to the CNT\_CNTR , CNT\_MAX , or CNT\_MIN registers are located within bits 4-bit groups of the CNT\_CMD register.

Note that a maximum of three commands can be issued at any one time, excluding the CNT\_CMD.W1ZMONCE command. Also, note that CNT\_CMD.W1LCNTMIN , CNT\_CMD.W1LCNTMAX , and CNT\_CMD.W1LCNTZERO bits have to be used exclusively. Never set more than one of them at the same time.

Figure 25-5: CNT\_CMD Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000004_87ff287803f4775d44500e81279820e75bb56b8e1e3ccea1000e542a030f6acd.png)

Table 25-9: CNT\_CMD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | W1ZMONCE   | Write 1 Zero Marker Clear Once Enable. The CNT_CMD.W1ZMONCE enables a single zero marker clear of the CNT_CNTR register. Reading a 1 in this bit indicates that the bit has been set by software before, but no zero marker event has been detected on the CNT_ZM pin yet. |
| 10 (R0/W)          | W1LMAXMIN  | Write 1 MAX Copy from MIN. The CNT_CMD.W1LMAXMIN bit transfers the current CNT_MIN register value into CNT_MAX register.                                                                                                                                                   |
| 9 (R0/W)           | W1LMAXCNT  | Write 1 MAX Capture from CNTR. The CNT_CMD.W1LMAXCNT bit loads the current value in the CNT_CNTR register into the CNT_MAX register to create a new boundary limit.                                                                                                        |
| 8 (R0/W)           | W1LMAXZERO | Write 1 MAX to Zero. Writing a 1 to the CNT_CMD.W1LMAXZERO bit clears the CNT_MAX register.                                                                                                                                                                                |
| 7 (R0/W)           | W1LMINMAX  | Write 1 MIN Copy from MAX. The CNT_CMD.W1LMINMAX bit transfers the current CNT_MAX register value into CNT_MIN register.                                                                                                                                                   |
| 5 (R0/W)           | W1LMINCNT  | Write 1 MIN Capture from CNTR. The CNT_CMD.W1LMINCNT bit loads the current value in the CNT_CNTR register into the CNT_MIN register to create a new boundary limit.                                                                                                        |
| 4 (R0/W)           | W1LMINZERO | Write 1 MIN to Zero. Writing a 1 to the CNT_CMD.W1LMINZERO bit clears the CNT_MIN register.                                                                                                                                                                                |

Table 25-9: CNT\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R0/W)           | W1LCNTMAX  | Write 1 CNTR Load from MAX. The CNT_CMD.W1LCNTMAX bit loads the current value in the CNT_MAX register into the CNT_CNTR register to create a new boundary limit. |
| 2 (R0/W)           | W1LCNTMIN  | Write 1 CNTR Load from MIN. The CNT_CMD.W1LCNTMIN bit loads the current value in the CNT_MIN register into the CNT_CNTR register to create a new boundary limit. |
| 0 (R0/W)           | W1LCNTZERO | Write 1 CNTR to Zero. Writing a 1 to the CNT_CMD.W1LCNTZERO bit clears the CNT_CNTR register.                                                                    |

## Counter Register

The CNT\_CNTR register holds the 32-bit, two's-complement count value. It can be read and written at any time. Hardware ensures that reads and write are atomic, by providing respective shadow registers. This register can be accessed with either 32-bit or 16-bit operations. This allows use of the CNT as a 16-bit counter if sufficient for the application.

Figure 25-6: CNT\_CNTR Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000005_559fad70c8dc1f47817cb238038c7d99e671a09bb59f291ce707f5e7f3db3aa3.png)

Table 25-10: CNT\_CNTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 31:0               | VALUE      | CNTR Value.                                                                  |
| (R/W)              |            | The CNT_CNTR.VALUE bit field holds the 32-bit, two's-complement count value. |

## Debounce Register

The CNT\_DEBNCE register selects the noise filtering characteristic of the three input pins according to the formula:

<!-- formula-not-decoded -->

Figure 25-7: CNT\_DEBNCE Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000006_bffc41073c12ce346cf2b2d3c86aeb9eea4fcd732614afcb9ca0a35d41e4ce40.png)

Table 25-11: CNT\_DEBNCE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | DPRESCALE  | Debounce Prescale. The CNT_DEBNCE.DPRESCALE selects the desired number of input filtering cycles (and resulting input debounce time) in multiples of SCLK. |

## Table 25-11: CNT\_DEBNCE Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration |                                                                  |
|-----------|------------|---------------------------|------------------------------------------------------------------|
| (Access)  |            |                           |                                                                  |
|           |            |                        17 | 131072x Cycles                                                   |
|           |            |                        18 | Reserved from This Value. The values 10010 - 11111 are reserved. |
|           |            |                        31 | Reserved Until This Value                                        |

## Interrupt Mask Register

The CNT\_IMSK register supports enabling (unmasking) interrupt request generation from each of the CNT events.

All bits in CNT\_IMSK either disable/mask an interrupt request (if bit cleared) or enable/unmask an interrupt request (if bit set).

Figure 25-8: CNT\_IMSK Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000007_88182e0e593963a71e42f57be12189bfdda3eb7753ddfbefffc1645c806a087c.png)

Table 25-12: CNT\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | CZMZ       | Counter Zeroed by Zero Marker Interrupt Enable. The CNT_IMSK.CZMZ bit enables (unmasks) the counter zeroed by zero marker interrupt request. 0 Mask Interrupt |
| 9 (R/W)            | CZME       | Zero Marker Error Interrupt Enable. The CNT_IMSK.CZME bit enables (unmasks) the zero marker error interrupt re- quest. 0 Mask Interrupt 1 Unmask Interrupt    |

Table 25-12: CNT\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | CZM        | CZM Pin/Pushbutton Interrupt Enable. The CNT_IMSK.CZM bit enables (unmasks) the CZM pin/pushbutton interrupt request.    |
| 8 (R/W)            | CZM        | 0 Mask Interrupt                                                                                                         |
| 8 (R/W)            | CZM        | 1 Unmask Interrupt                                                                                                       |
| 7 (R/W)            | CZERO      | CNT_CNTR Counts to Zero Interrupt Enable. The CNT_IMSK.CZERO bit enables (unmasks) the counts to zero interrupt request. |
| 7 (R/W)            | CZERO      | 0 Mask Interrupt                                                                                                         |
| 7 (R/W)            | CZERO      | 1 Unmask Interrupt                                                                                                       |
| 6 (R/W)            | COV15      | Bit 15 Overflow Interrupt Enable. The CNT_IMSK.COV15 bit enables (unmasks) the bit 15 overflow interrupt request.        |
| 6 (R/W)            | COV15      | 0 Mask Interrupt                                                                                                         |
| 6 (R/W)            | COV15      | 1 Unmask Interrupt                                                                                                       |
| 5 (R/W)            | COV31      | Bit 31 Overflow Interrupt Enable. The CNT_IMSK.COV31 bit enables (unmasks) the bit 31 overflow interrupt request.        |
| 5 (R/W)            | COV31      | 0 Mask Interrupt                                                                                                         |
| 5 (R/W)            | COV31      | 1 Unmask Interrupt                                                                                                       |
| 4 (R/W)            | MAXC       | Max Count Interrupt Enable. The CNT_IMSK.MAXC bit enables (unmasks) the max count interrupt request.                     |
| 4 (R/W)            | MAXC       | 0 Mask Interrupt                                                                                                         |
| 4 (R/W)            | MAXC       | 1 Unmask Interrupt                                                                                                       |
| 3 (R/W)            | MINC       | Min Count Interrupt Enable. The CNT_IMSK.MINC bit enables (unmasks) the min count interrupt request.                     |
| 3 (R/W)            | MINC       | 0 Mask Interrupt                                                                                                         |
| 3 (R/W)            | MINC       | 1 Unmask Interrupt                                                                                                       |
| 2 (R/W)            | DC         | Downcount Interrupt Enable. The CNT_IMSK.DC bit enables (unmasks) the down count interrupt request.                      |
| 2 (R/W)            | DC         | 0 Mask Interrupt                                                                                                         |
| 2 (R/W)            | DC         | 1 Unmask Interrupt                                                                                                       |

Table 25-12: CNT\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | UC         | Upcount Interrupt Enable. The CNT_IMSK.UC bit enables (unmasks) the up count interrupt request. 0 Mask Interrupt 1 Unmask Interrupt                                                                         |
| 0 (R/W)            | IC         | Illegal Gray/Binary Code Interrupt Enable. The CNT_IMSK.IC bit enables (unmasks) the illegal Gray/Binary Code interrupt request and should only be used in these modes. 0 Mask Interrupt 1 Unmask Interrupt |

## Maximum Count Register

The CNT\_MAX register holds the 32-bit, two's-complement, higher boundary value. It can be read and written at any time. Hardware ensures that reads and write are atomic, by providing respective shadow registers. This register can be accessed with either 32-bit or 16-bit operations. This allows for using the CNT as a 16-bit counter if sufficient for the application.

Figure 25-9: CNT\_MAX Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000008_72ace1838d9038d4cb01a1ae1a4dd980011acf02242ea5ed7c3d7646d35abc6c.png)

Table 25-13: CNT\_MAX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | MAX Value.                                                                               |
| (R/W)              |            | The CNT_MAX.VALUE bit field holds the 32-bit, two's-complement, higher boun- dary value. |

## Minimum Count Register

The CNT\_MIN register holds the 32-bit, two's-complement, lower boundary value. It can be read and written at any time. Hardware ensures that reads and write are atomic, by providing respective shadow registers. This register can be accessed with either 32-bit or 16-bit operations. This allows for using the CNT as a 16-bit counter if sufficient for the application.

Figure 25-10: CNT\_MIN Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000009_4363cb82995c8444eec1202781609511c1d25608f053f6fc688167b5c240245b.png)

Table 25-14: CNT\_MIN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | MIN Value. The CNT_MIN.VALUE bit field holds the 32-bit, two's-complement, lower boundary value. |

## Status Register

The CNT\_STAT register provides status information for each of the CNT events as configured in the CNT\_IMSK register. When a CNT event is detected, the corresponding bit in this register is set. It remains set until either software writes a 1 to the bit (write-1-to-clear) or the CNT is disabled.

All bits in the CNT\_STAT register indicate either no interrupt request pending (if bit cleared) or an interrupt request pending (if bit set).

Figure 25-11: CNT\_STAT Register Diagram

![Image](28_General-Purpose_Counter_(CNT)_artifacts/image_000010_8775305ed571a301849fa75b1994a9016cac4600dc116ca15e21e8c9c1240d73.png)

Table 25-15: CNT\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W1C)         | CZMZ       | Counter Zeroed by Zero Marker Interrupt. The CNT_STAT.CZMZ bit indicates a zero marker error. If the CNT_CFG.ZMZC bit =1, this interrupt request is generated when the CZMII latch reports a significant edge on the CZM input. Once cleared by software the CNT_STAT.CZM bit is not set again when the CZM input remains active without pulsing. |
| 10 (R/W1C)         | CZMZ       | 0 No Error                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W1C)         | CZMZ       | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                  |

Table 25-15: CNT\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W1C)          | CZME       | Zero Marker Error Interrupt. The CNT_STAT.CZME bit behaves similarly to the CNT_STAT.CZM bit, with the exception that CNT_STAT.CZME is not set on the CZM edge when the lower four bits of the CNT_CNTR are not zero. In many applications this indicates an error condition, as the zero marker might be out of sync with the counter.        |
| 8 (R/W1C)          | CZM        | CZM Pin/Pushbutton Interrupt. The CNT_STAT.CZM bit indicates a CZM pin/pushbutton error. This interrupt request is generated when a significant edge is seen on the CZM pin, regardless what mode the counter is operating in. This is often used to sense push buttons (especially with the debouncing circuit enabled). 0 No Error           |
| 7 (R/W1C)          | CZERO      | CNT_CNTR Counts to Zero Interrupt. The CNT_STAT.CZERO bit indicates a counts to zero error. This error is generated when the CNT_CNTR register has incremented or decremented toward 0x0000.0000. The latch is not set when software writes to the CNT_CNTR register directly or when the counter is zeroed by writes to the CNT_CMD register. |
| 6                  | COV15      | 0 No Error 1 Error Occurred                                                                                                                                                                                                                                                                                                                    |
| (R/W1C)            |            | Bit 15 Overflow Interrupt. The CNT_STAT.COV15 bit indicates a bit 15 overflow error. This error is gener- ated when the 16-bit twos-complement CNT_CNTR register has incremented from 0xxxxx.7FFF to 0xxxxx.8000 or decremented from 0xxxxx.8000 to 0xxxxx.7FFF. 0 No Error                                                                    |
| 5 (R/W1C)          | COV31      | Bit 31 Overflow Interrupt. The CNT_STAT.COV31 bit indicates a bit 31 overflow error. This error is gener- ated when the 32-bit twos-complement CNT_CNTR register has incremented from 0x7FFF.FFFF to 0x8000.0000 or decremented from 0x8000.0000 to 0x7FFF.FFFF.                                                                               |
| 5 (R/W1C)          |            | 0 No                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W1C)          |            | Error                                                                                                                                                                                                                                                                                                                                          |
| 5 (R/W1C)          |            | 1 Error Occurred                                                                                                                                                                                                                                                                                                                               |

Table 25-15: CNT\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W1C)          | MAXC       | Max Count Interrupt. The CNT_STAT.MAXC bit indicates a max count error. This interrupt is used in boundary compare (BND_COMP) mode. If after incrementing the CNT_CNTR regis- ter equals CNT_MAX , the CNT_STAT.MAXC bit is set.                                                                                                                                                                                                                 |
| 4 (R/W1C)          | MAXC       | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | MINC       | Min Count Interrupt. The CNT_STAT.MINC bit indicates a minimum count error. This interrupt is used in boundary compare (BND_COMP) mode. If, after decrementing, the CNT_CNTR register equals CNT_MIN , the CNT_STAT.MINC bit is set.                                                                                                                                                                                                             |
| 3 (R/W1C)          | MINC       | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W1C)          | MINC       | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 2 (R/W1C)          | DC         | Down Count Interrupt. The CNT_STAT.DC bit indicates a down count error. This interrupt is generated when the CNT_CNTR register decrements.                                                                                                                                                                                                                                                                                                       |
| 2 (R/W1C)          | DC         | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2 (R/W1C)          | DC         | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 1 (R/W1C)          | UC         | Up Count Interrupt. The CNT_STAT.UC bit indicates an up count interrupt. This interrupt is generated when the CNT_CNTR register increments.                                                                                                                                                                                                                                                                                                      |
| 0 (R/W1C)          | IC         | Illegal Gray/Binary Code Interrupt. The CNT_STAT.IC bit indicates a illegal Gray/Binary Code interrupt and should only be used in these modes. In normal operation those codes can increment or decrement the CNT_CNTR register by one at a time. If the sensed inputs instruct the counter to increment or decrement by two, the CNT_STAT.IC bit is set. Hardware sets the CNT_STAT.IC bit in QUAD_ENC and BIN_ENC encoder modes only. No Error |
| 0 (R/W1C)          | IC         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 0 (R/W1C)          | IC         | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                 |