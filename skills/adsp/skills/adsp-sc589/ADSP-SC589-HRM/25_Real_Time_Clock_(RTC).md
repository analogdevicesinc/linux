## 22   Real Time Clock (RTC)

The processor contains a Real Time Clock (RTC) which provides a set of digital watch features, including time of day, alarm, and stopwatch countdown. It is typically used to implement either a real-time watch or a life counter which counts the elapsed time since the last system reset. The RTC uses dedicated power supply pins and is independent of any reset, which enables it to maintain functionality even when the rest of the processor is powered down.

## RTC Features

The RTC interface has the following features.

- Provides a 1 Hz clock with a second, minute, hour, and day counter (0-32767 days)
- Alarm with a time-of-day interrupt
- Operates on a dedicated supply from an external 3.3 V battery
- Stopwatch function
- Interrupt generation for different events
- Standard two-pin interface with an external 32.768 kHz crystal, 6 pF capacitor on each pin, and a 100 M Ω resistor between the pins
- RTC power switches to the I/O supply when the processor is powered on, saving battery life
- Calibration corrects time once a day; application can use the RTCXTALIN pin to determine calibration settings
- Power down and bus disable

## RTC Functional Description

The RTC provides a set of digital watch features to the processor. The RTC external interface consists of two clock pins, which together with the external components form the reference clock circuit for the RTC. It uses an external 32.768 kHz crystal with external capacitor and operates on a dedicated external 3.3 V Lithium coin cell which is never powered off.

The following are functional characteristics of the RTC.

## Power supply partitioning

The RTC logic is partitioned between the processor core supply voltage and RTC supply voltage. The core of RTC unit functions on a RTC power supply.

## Battery life

To increase the battery life of the external 3.3V cell, most functions reside in the RTC core voltage domain which runs off the processor supply. Only basic clock circuitry resides in the RTC IO voltage.

## Reads/writes to 1Hz registers

There is no latency when reading 1 Hz registers, as the values come from the shadow registers.

Writes to the RTC 1 Hz registers are synchronized to the 1 Hz RTC clock.

## ADSP-SC58x RTC Register List

The Real-Time Clock (RTC) provides clock-related services, including a set of processor events that can be counted during program execution. A set of registers governs RTC operations. For more information on RTC functionality, see the RTC register descriptions.

Table 22-1: ADSP-SC58x RTC Register List

| Name         | Description                        |
|--------------|------------------------------------|
| RTC_ALM      | RTC Alarm Register                 |
| RTC_CLK      | RTC Clock Register                 |
| RTC_IEN      | Interrupt Enable Register          |
| RTC_INIT     | RTC Initialization Register        |
| RTC_INITSTAT | RTC Initialization Status Register |
| RTC_STAT     | RTC Status Register                |
| RTC_STPWTCH  | RTC Stop Watch Register            |

## ADSP-SC58x RTC Interrupt List

Table 22-2: ADSP-SC58x RTC Interrupt List

|   Interrupt ID | Name     | Description   | Sensitivity   | DMA Channel   |
|----------------|----------|---------------|---------------|---------------|
|            165 | RTC0_EVT | RTC0 Event    | Level         |               |

## ADSP-SC58x RTC Trigger List

Table 22-3: ADSP-SC58x RTC Trigger List Masters

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|          134 | RTC0_EVT | RTC0 Event    | Level         |

Table 22-4: ADSP-SC58x RTC Trigger List Slaves

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## RTC Definitions

The following definitions are helpful when using the RTC module.

## RTC HV

The RTC HV is the portion of the RTC which runs on 3.3 V domain and contains the logic for RTC counting/ clock functionality

## RTC LV

The RTC LV operates on the core voltage supply and provides the control/access functionality and user interface to the application

## Oscillator

The oscillator is the heart of the RTC. It works with the dedicated external crystal to generate a 32768 Hz signal which is divided down to a 1Hz clock used to operate the rest of the RTC

## RTC Signal Descriptions

The RTC signals are listed in the following table.

Table 22-5: RTC Signal Descriptions

| Signal Name   | Signal Description                                       |
|---------------|----------------------------------------------------------|
| XTALIN        | Pin of the dedicated 32.768KHz external crystal          |
| XTALOUT       | Pin of the dedicated 32.768KHz external crystal          |
| VDDRTC        | Power supply pin (from Lithium Coin Cell) for RTC module |

## RTC Architectural Concepts

The key use of the RTC is to provide the time keeping function and maintain the time and date in an accurate and reliable manner with minimal power consumption. In addition to time keeping it also provides the stopwatch and alarm features. The RTC uses the internal counters to keep the time of the day in terms of seconds, minutes, hours and days. This data is enough for the user application to extract the date and time information from the RTC.

Interrupts can be issued periodically, either every second, every minute, every hour, or every day. Each of these interrupts can be independently controlled. It is the responsibility of the program to set the correct time by a software write into the RTC\_CLK register. Once set, the counters maintain time as long as the RTC supply is valid. The RTC provides two alarm features, programmed with the RTC alarm register ( RTC\_ALM ). The first is a time of day alarm (hour, minute, and second). When the alarm interrupt is enabled, the RTC generates an interrupt each day at the time specified. The second alarm feature allows the application to specify a day as well as a time.

The RTC also provides a stopwatch function that acts as a countdown timer. The application can program a second count into the RTC stopwatch count register ( RTC\_STPWTCH ). When the stopwatch interrupt is enabled and the specified number of seconds has elapsed, the RTC generates an interrupt.

## RTC Block Diagram

The RTC block diagram provides a top level block diagram of the elements used. The main blocks of the RTC are the individual counters, the alarm register, and the event control.

Figure 22-1: RTC Block Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000000_c345f90bb0ee520321be72cd06255bc8c31503b97a32c486b666d085489f90cf.png)

## Power Supply Partitioning

The RTC logic is partitioned between the processor core supply voltage and RTC supply voltage. The core of RTC unit functions on a RTC power supply so that the RTC can maintain the time even if the processor power is off. When the core supply voltage is absent, interrupts are ignored.

When the processor's I/O supply is above a certain threshold, the RTC switches to the I/O supply to conserve battery power.

NOTE: Battery power supply can operate the RTC when the I/O voltage is turned off.

The RTC is partitioned into two blocks. The counting and clock function is provided in the I/O voltage domain (VDD\_EXT), while the control and access function and the user interface are provided in the core voltage domain. The RTC I/O operates on a dedicated power supply provided by the external 3.3 V (nominal) lithium coin cell. The RTC also has the ability to switch to the I/O supply (VDD\_EXT). The RTC LV operates on the nominal core voltage supply (VDD\_INT). The interface between both blocks is provided by a set of level shifters. The partitioning at chip level is shown in the Power Supply Partition figure.

Figure 22-2: Power Supply Partition

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000001_120f7c2fec5e8e7547ba56246aff7d905a338e76e8a4093ed1cba5f2dbe69393.png)

## Battery Life

To increase the battery life of the external 3.3V cell, most functions reside in the RTC core voltage domain which runs off the processor supply. Only basic clock circuitry resides in the RTC IO voltage. The RTC module provides the following features to extend battery life.

- The seconds, minutes, hours and days counters reside inside the RTC IO voltage.
- The alarm register and comparators reside inside the RTC IO voltage. This allows programs to power down the rest of the chip without the alarm being reset.
- The programmable interface registers, through which the application reads or writes the current time and alarm settings, are part of the RTC core voltage. In order to set the current time and/or alarm, software writes into the shadow registers in the RTC core voltage. The data is then transferred into the corresponding register in the RTC IO voltage by hardware.
- The RTC core voltage runs primarily on the processor's peripheral clock while the RTC IO voltage runs primarily on a self generated 1 Hz clock. The synchronization circuitry sits inside the RTC core voltage.
- The stopwatch circuitry is inside the RTC core voltage and operates on a 1 Hz clock, level shifted from the RTC IO voltage.

## Writes to the 1 Hz Registers

Writes to the RTC 1 Hz registers are synchronized to the 1 Hz RTC clock. When setting the time of day, do not factor in the delay when writing to the RTC 1 Hz registers. The most accurate method of setting the RTC is to

monitor the seconds (1 Hz) event flag or to program an interrupt for this event and then write the current time to the RTC status register ( RTC\_STAT ) in the interrupt service routine (ISR). The new value is inserted ahead of the incremented value. Hardware adds one second to the written value (with appropriate carries into minutes, hours and days) and loads the incremented value at the next 1 Hz tick, when it represents the then-current time.

Writes posted at any time are properly synchronized to the 1 Hz clock. Writes complete at the rising edge of the 1 Hz clock. A write posted just before the 1 Hz tick may not be completed until the 1 Hz tick one second later.

## Reads From the 1 Hz Registers

There is no latency when reading 1 Hz registers, as the values come from the shadow registers. The shadows are updated and ready for reading by the time any RTC interrupts or event flags for that second are asserted. Once the internal core logic completes its initialization sequence after SCLK0\_0 starts, there is no point in time when it is unsafe to read the 1 Hz registers (for synchronization reasons). The registers always return coherent values, although the values may be unknown.

## RTC Operating Modes

The following sections provide information on the operating modes available to the real-time clock.

## Alarm

The RTC provides two alarm features, programmed with the RTC alarm register ( RTC\_ALM ). The first is a time of day alarm (hour, minute, and second). When the alarm interrupt is enabled, the RTC generates an interrupt each day at the time specified.

## Day Alarm

The second alarm feature allows the application to specify a day as well as a time. When the day alarm interrupt is enabled, the RTC generates an interrupt on the day and time specified. The alarm interrupt and day alarm interrupt can be enabled or disabled independently.

## Stopwatch

The RTC stopwatch count register ( RTC\_STPWTCH ) contains the countdown value for the stopwatch. The stopwatch counts down seconds from the programmed value and generates an interrupt (if enabled) when the count reaches 0. The counter stops counting at this point and does not resume counting until a new value is written to the RTC\_STPWTCH register. Once running, the counter may be overwritten with a new value. This allows the stopwatch to be used as a watchdog timer with a precision of one second.

The stopwatch can be programmed to any value between 0 and (2 16  - 1) seconds, which is a range of 18 hours, 12 minutes, and 15 seconds. Typically, software should wait for a 1 Hz tick, then write the RTC\_STPWTCH register. One second later, the RTC\_STPWTCH value changes to the new value and begins decrementing. Because the register write occupies nearly one second, the time from writing a value of N until the stopwatch interrupt is nearly N + 1 seconds. To produce an exact delay, software can compensate by writing N - 1 to get a delay of nearly N seconds. This implies that a delay of 1 second with the stopwatch cannot be achieved. Writing a value of 1 immediately after

a 1 Hz tick results in a stopwatch interrupt nearly two seconds later. To wait one second, software should just wait for the next 1 Hz tick

## Digital Watch Mode

The primary function of the RTC is to maintain an accurate day count and time of day. The RTC accomplishes this by means of four counters:

- 60-second counter
- 60-minute counter
- 24-hour counter
- 32768-day counter

The RTC provides a set of digital watch features. The internal oscillator generates a 32768 Hz signal using the crystal which is scaled down to 1 Hz and used to clock the second, minute, hour and day counters. The 32768 day counter increments each day at midnight (during the change from 23:59:59). The counter operates on the RTC supply (either the external battery or I/O supply) and is active irrespective of the status of the processor core supply (VDD\_INT). When the processor core and I/O supply are valid then the following occurs.

- The current time is updated every second into the RTC clock register ( RTC\_CLK )
- Interrupts can be issued periodically every second, every minute, every hour or every day

Each of the interrupts can be independently controlled, and are described in the RTC Event Control section.

It is the responsibility of the program to set the correct time by a software write into the RTC\_CLK register. Once set, the counters maintain time as long as the RTC supply is valid.

## Calibration for Accuracy

To guard against the possibility of long term (&gt; 1 day) errors, the RTC provides a calibration feature using 4 bits of the RTC\_INIT register (not available for the stopwatch function).

This is a simple a time correction at the end of every day (when the clock register changes from a Day:Hour:Min:Sec value of XXX:23:59:59 to YYY:00:00:00). It functions by adding or subtracting an integer number of seconds (to a maximum of 7) from the start of the next day, to correct accumulated time error over the course of the previous day. The number of seconds that are added or subtracted is defined in the RTC\_INIT.CAL bit field.

As an example, if there is a -50 ppm error in the 1 Hz frequency, this translates into a 86400 ´ 50 ppm seconds (+4.32 seconds) error. That is at time 00:00:00, RTC time is 4.32 seconds ahead of actual time. The RTC can correct this by adding 4 seconds (if 4 is the value written into the calibration register) to the time at 00:00:00. Therefore, from 23:59:59, the timer counter jumps directly to 00:00:04, (there is no 00:00:00 to 00:00:03 time occurrences). At the instant it jumps to 00:00:04, the error reduces to +0.32 seconds over the course of the day, which is only 3.7 ppm.

As a second example, if there is a +50 ppm error in the 1 Hz frequency, this also translates into 86400 ´ 50 ppm seconds (-4.32 seconds) error. In this case the time must be subtracted. This is corrected in the RTC by counting

00:00:00 to 00:00:03 twice, so that the time is effectively subtracted. As soon as the RTC reaches 00:00:04, the error reduces to -0.32 seconds over the course of the previous day and accumulated error is minimized.

When the RTC is powered up for the first time, the calibration values are written once to ensure proper function (if they are not to be used, write 0000). These register bits are sticky, which means that once set, they retain their value irrespective of the status of the core power supply.

The addition or subtraction of time can only be in integer multiples of seconds. Zero to seven seconds can be added or subtracted using 4 bits. The MSB indicates addition (0) or subtraction (1). The three LSBs indicate number of seconds (0 - 7 represented by their binary 3 bit equivalents). Because the clock runs at a time period of one second (@ 1 Hz frequency) 0.25 or 0.5 second resolution is not possible.

The calibration technique introduces a guard band for alarm by definition. In case the alarm is set within the duration of the time (Day:00:00:00 to Day:00:00:06) corrected by the calibration register, then it occurs at the nearest corrected time. This is shown in the following two examples.

If the RTC\_INIT.CAL is 0101, the RTC clock jumps from 23:59:59 to 00:00:05 due to calibration. If the alarm is set to 00:00:01, it occurs at the RTC time 00:00:05.

If the RTC\_INIT.CAL is 1101, then the RTC clock counts from 00:00:00 to 00:00:04 twice and then moves to 00:00:05. If the alarm is set to 00:00:01, it occurs at the RTC time 00:00:05.

## Accuracy

In order to perform calibration on the bench, use the RTXI pin and check the ppm deviation from 32.768 kHz. This ppm error is the same as in the internal 1Hz clock and the calibration register should be updated with the corresponding values as explained in Calibration for Accuracy above.

Note that total accuracy is ≤ ±35 ppm, &lt; ±1.5 minutes per month of error, inclusive of any inaccuracies of the RTC input crystal at room temperature. This is achieved with a crystal of ±10 ppm error at 25°C. A crystal error of ±20 ppm translates into a maximum inaccuracy of ±45 ppm.

## RTC Event Control

The RTC generates multiple events depending on the value on RTC\_CLK , RTC\_ALM and RTC\_STPWTCH registers. It generates an event on each second, minute, hour, day. It also generates an event when an alarm or day alarm or stopwatch expiry occurs.

## RTC Events

The RTC can provide interrupts at several programmable intervals

- Per Second
- Per minute (clock counter x:y:z:59)
- Per hour (clock counter x:y:59:59)
- Per day (clock counter x:23:59:59)

- Everyday at a particular time (day alarm)
- On a particular day and time (time of the day alarm)
- Expiry of Stop Watch counter
- When the 1 Hz clock from RTC HV fails

The RTC can also be programmed to provide an interrupt at the completion of all pending writes to any of the RTC 1Hz registers ( RTC\_ALM , RTC\_CLK , RTC\_INIT and RTC\_STPWTCH ). Any of these interrupts can be individually enabled/disabled through the bits in RTC\_IEN register.

Programs can disable all the RTC interrupts when the processor enters emulation space by setting the RTC\_IEN.EMUDIS bit. Interrupts are not generated even if the individual interrupt enable bits in the RTC\_IEN register are set.

In the service routine, the RTC\_STAT register should be read to identify the cause of the interrupt. While reading the status register the RTC automatically clears the respective status bit, ensuring that the cause has been cleared before ending the routine. Note that the pending RTC interrupt is cleared whenever all enabled and set (=1) bits in the RTC\_STAT register are read, or when all bits in the RTC\_IEN register corresponding to pending events are cleared (=0).

## RTC Status and Error Signals

The RTC contains a status register ( RTC\_STAT ) that provides the status on the errors and other events generated by the module. The module captures the 1 Hz clock failures in this register as well. The RTC\_STAT register contains the RTC event flags and RTC interrupt status. Once set by the event, each bit remains set until cleared by a software read of this register. These sticky bits are independent of the interrupt enable bits in the RTC\_IEN register. Values are cleared by reading RTC\_STAT register, except for the RTC\_STAT.WRPEND , RTC\_STAT.ALM and RTC\_STAT.DAYALM bits. Writes to the RTC\_STAT register have no effect.

## RTC Programming Model

The following sections provide basic programming steps for the RTC interface.

## Power-Up, Power-Down and Reset

The RTC\_INIT.PWDN programmable bit is provided to power down the RTC. Power-down is interpreted as a crystal oscillator disable, which reduces power dissipation to only leakage current. Once set or reset, this bit retains its value unless changed, irrespective of the status of core supply.

The inclusion of the power-down bit ( RTC\_INIT.PWDN ) as well as the possibility that the RTC may not be used in certain applications introduces specific constraints on the power-up and reset behavior of the RTC. These are described below.

1. When the RTC is powered-up for the first time, it remains in an undefined state until the core powers-up and the corresponding RTC\_INIT.PWDN bit is written by software. Programs should clear (=0) the RTC\_INIT.PWDN bit if the RTC function is desired and set it (=1) if it is not.
2. After clearing the RTC\_INIT.PWDN bit the application must wait at least until the first seconds' event before it writes the timer and alarm registers. This is because the oscillator has a startup time before the clock is generated.

This sequence applies only to the first time the RTC supply (battery or I/O) is connected. Once the RTC\_INIT.PWDN bit is set or reset, its value is retained as long as RTC supply (battery or I/O) is valid.

3. After the RTC supply is connected for the first time and the RTC\_INIT.PWDN bit =0, the application is free to power-up and power-down the core supply any number of times without loss of RTC function (provided the RTC supply, battery or I/O, is valid). Conversely, if the RTC\_INIT.PWDN bit =1, then the RTC oscillator remains disabled irrespective of the status of the core supply.
4. The current status of the RTC power-down is updated by hardware into the initialization status register ( RTC\_INITSTAT.PWDN ). This is useful when the rest of the processor wakes up from power-down and needs to know the status of the RTC.
5. Whenever the processor core wakes up from power-down, the values of the RTC\_CLK , RTC\_ALM and RTC\_STPWTCH registers is zero until the first seconds' event after power-up. At the first seconds' event, an arbitrary value is uploaded into these registers. To put them in a defined state software must write the desired value into these registers. If the RTC\_CLK and RTC\_ALM registers have been set before core power-down and subsequent power-up, their values are valid throughout, but can be read by the program only after the first seconds' event after power-up.

## Register Access

The interface to the RTC is through a set of memory-mapped registers. The RTC is configured through software and the current state of the RTC is also determined through reads and writes to these registers. Writes of the alarm, clock, stopwatch and initialization registers is performed in a two step sequence.

1. The desired values are programmed into a shadow register in the processor's core VDD\_INT domain and operating on the processor's peripheral clock.
2. The contents of the shadow register are synchronized onto the contents of the RTC's internal clock register which operates on the 1 Hz clock in the RTC power domain.

To ensure that writes between the core voltage and RTC voltage domain are properly synchronized, all write commands should be issued immediately after a seconds' event in the RTC status register ( RTC\_STAT ). This two step sequence results in a write latency of up to 1 second.

While the write sequence is ongoing, the write pending ( RTC\_STAT.WRPEND ) bit is set and is cleared by hardware when the process is complete. Resetting or powering down the peripherals while a write is in progress, (that is when this bit is set) is forbidden. Subsequent writes to the same register before completion of the previous write are ignored.

- Do not attempt write to any of RTC\_CLK , RTC\_ALM or RTC\_STPWTCH registers when the RTC oscillator is powered down or when the RTC\_INIT.RDEN bit is set.
- During initialization, after a write of the RTC\_INIT register, make sure that the RTC\_STAT.WRPEND bit is cleared before attempting writes to other registers.

The RTC\_INIT register can be written any time. However, programs must ensure that the interrupt enable bits corresponding to Clock, Alarm and Stopwatch features are set only after initializing the RTC\_CLK , RTC\_ALM and RTC\_STPWTCH registers. The RTC\_STAT and RTC\_INITSTAT registers can be read any time.

## ADSP-SC58x RTC Register Descriptions

Real Time Clock (RTC) contains the following registers.

Table 22-6: ADSP-SC58x RTC Register List

| Name         | Description                        |
|--------------|------------------------------------|
| RTC_ALM      | RTC Alarm Register                 |
| RTC_CLK      | RTC Clock Register                 |
| RTC_IEN      | Interrupt Enable Register          |
| RTC_INIT     | RTC Initialization Register        |
| RTC_INITSTAT | RTC Initialization Status Register |
| RTC_STAT     | RTC Status Register                |
| RTC_STPWTCH  | RTC Stop Watch Register            |

## RTC Alarm Register

The RTC\_ALM register is programmed by software for the time (in hours, minutes, and seconds) the alarm interrupt occurs. Reads and writes can occur at any time. The alarm interrupt occurs whenever the hour, minute, and second fields first match those of the RTC\_CLK register. The day interrupt occurs whenever the day, hour, minute, and second fields first match those of the RTC\_CLK status register.

Figure 22-3: RTC\_ALM Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000002_a989b6d7baa5ae78fc944091a07b4d19aa6703056455de28f2508bd9539e3e64.png)

Table 22-7: RTC\_ALM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:17 (R/W)        | DAY        | Alarm Day. The RTC_ALM.DAY bit provides the alarm day from 0 to 32767. The day interrupt occurs whenever the day, hour, minute, and second fields first match those of the RTC status register. |
| 16:12 (R/W)        | HOUR       | Alarm Hour. The RTC_ALM.HOUR bit field configures the alarm to occur whenever the alarm hour (0 to 23) first matches that of the RTC status register.                                           |
| 11:6 (R/W)         | MIN        | Alarm Minute. The RTC_ALM.MIN bit field configures the alarm to occur whenever the alarm mi- nute (0 to 59) first matches that of the RTC status register.                                      |
| 5:0 (R/W)          | SEC        | Alarm Second. The RTC_ALM.SEC bit field configures the alarm to occur whenever the seconds (0 to 59) first match those of the RTC status register.                                              |

## RTC Clock Register

The RTC\_CLK register is used to read or write the current time. It has no reset and an undefined value when the RTC VDD is first powered up. Writing invalid time values is forbidden (for example, an hour value more than 23 and a minute value more than 59). The RTC\_CLK register is updated every second. If the RTC is already running when the core starts up, the values read from RTC\_CLK are zero until the first second event comes. In this case, programs must wait for the second event and then read this register.

Figure 22-4: RTC\_CLK Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000003_43117118d21fd077e19223f88d227b28238c00ac45d6f0ffb5aa956a6172154e.png)

Table 22-8: RTC\_CLK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:17 (R/W)        | DAY        | Day Count. The RTC_CLK.DAY bit provides the day count from 0 to 32767.    |
| 16:12 (R/W)        | HOUR       | Hour Count. The RTC_CLK.HOUR bit provides the hour count from 0 to 23.    |
| 11:6 (R/W)         | MIN        | Minute Count. The RTC_CLK.MIN bit provides the minute count from 0 to 59. |
| 5:0 (R/W)          | SEC        | Second Count. The RTC_CLK.SEC bit provides the second count from 0 to 59. |

## Interrupt Enable Register

The RTC\_IEN register enables interrupts (when the bits are set) or disables interrupts (when bits are cleared).

Figure 22-5: RTC\_IEN Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000004_710342a9db77ca60079d0d1b9694e8d3973712eb5809d273d780331ab4601cf5.png)

Table 22-9: RTC\_IEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | EMUDIS     | Emulator Mode Interrupt Disable. The RTC_IEN register disables or enables RTC interrupts in emulation mode. Inter- rupts are not generated even if individual interrupt enable bits in the RTC_IEN regis- ter are set. 0 Enable interrupt |
| 8 (R/W)            | CLKFAIL    | Clock Fail Interrupt Enable. The RTC_IEN.CLKFAIL bit enables the RTC 1Hz clock fail interrupt. 0 Disable interrupt                                                                                                                        |
| 7 (R/W)            | SW         | Stopwatch Interrupt Enable. The RTC_IEN.SW bit enables the stopwatch interrupt. 0 Disable interrupt 1 Enable interrupt                                                                                                                    |

Table 22-9: RTC\_IEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | DAYALM     | Day Alarm Interrupt Enable. The RTC_IEN.DAYALM bit enables the alarm (day, hour, minute, second) interrupt.                                                                                                                      |
| 5 (R/W)            | ALM        | Alarm Interrupt Enable. The RTC_IEN.ALM bit enables the alarm (hour, minute, second) interrupt. 0 Disable interrupt                                                                                                              |
| 4 (R/W)            | DAY        | Days Interrupt Enable. The RTC_IEN.DAY bit enables the days interrupt. 0 Disable interrupt                                                                                                                                       |
| 3 (R/W)            | HOUR       | Hours Interrupt Enable. The RTC_IEN.HOUR bit enables the hours interrupt.                                                                                                                                                        |
| 2 (R/W)            | MIN        | 0 Disable interrupt 1 Enable interrupt                                                                                                                                                                                           |
|                    |            | Minutes Interrupt Enable. The RTC_IEN.MIN bit enables the minutes interrupt. 0 Minute interrupt disabled 1 Enable interrupt                                                                                                      |
| 1 (R/W)            | SEC        | Seconds Interrupt Enable. The RTC_IEN.SEC bit enables the seconds interrupt. 0 Disable interrupt 1 Enable interrupt                                                                                                              |
| 0 (R/W)            | WRDONE     | Register Write Done Interrupt Enable. The RTC_IEN.WRDONE bit enables the interrupt for register write completion. The RTC_IEN.WRDONE bit is applicable only for the alarm, clock, and stopwatch regis- ters. 0 Disable interrupt |

## RTC Initialization Register

The RTC\_INIT register provides the calibration function, powers down the unit, and disables the output buses.

Figure 22-6: RTC\_INIT Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000005_3e6729590caad9f94e24f8ccfb9508bed04336afd22233aca151d92a444f1788.png)

Table 22-10: RTC\_INIT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | RDEN       | Enable Output Bus. The RTC_INIT.RDEN bit disables the output bus. 0 Enable output bus                                                                                                |
| 4 (R/W)            | PWDN       | RTC Power Down. The RTC_INIT.PWDN bit powers down the RTC module. For complete informa- tion on this function, see the "Power-Up, Power-Down and Reset" section of this chapter.     |
| 3:0 (R/W)          | CAL        | Time Calibration. The RTC_INIT.CAL bit performs calibration the RTC module. For complete infor- mation on this function, see the "Calibration for Accuracy" section of this chapter. |

## RTC Initialization Status Register

The RTC\_INITSTAT register contains values of various status bits which can be used to check the status of RTC when the core comes out of reset.

Figure 22-7: RTC\_INITSTAT Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000006_f9c0d06551f1b282d478ead52f76b45243035e7297b2f6438d29567c41c138a4.png)

Table 22-11: RTC\_INITSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:3 (R/NW)         | CAL        | Calibration Status.                                                                                                                                                                                                                                              |
| 2 (R/NW)           | PWDN       | Status of Power Down. The RTC_INITSTAT.PWDN bit indicates the status of the RTC. 0 Oscillator is powered down                                                                                                                                                    |
| 1 (R/NW)           | DAYALMPND  | Day Alarm Pending. The RTC_INITSTAT.DAYALMPND bit indicates that an alarm has occurred. This indication is useful when the core has powered down or reset in the middle of opera- tion. This bit is cleared when the RTC_INITSTAT register is read.              |
| 0 (R/NW)           | ALMPND     | Alarm Pending. The RTC_INITSTAT.ALMPND bit indicates that an alarm has occurred. This indi- cation is useful when the core has powered down or reset in the middle of operation. This bit is cleared when the RTC_INITSTAT register is read. 0 No alarm occurred |
| 0 (R/NW)           | ALMPND     | 1 Alarm occurred                                                                                                                                                                                                                                                 |
| 0 (R/NW)           | ALMPND     |                                                                                                                                                                                                                                                                  |

## RTC Status Register

The RTC\_STAT register contains the RTC event flags and RTC interrupt status. These bits are sticky. Once set by the event, each bit remains set until cleared by a software read. These sticky bits are independent of the interrupt enable bits in the RTC\_IEN register. Values are cleared by reading the RTC\_STAT register, except for the RTC\_STAT.WRPEND bit, which is read-only. Writes of 0 or 1 to any bit of this register has no effect. This register is cleared at reset.

Figure 22-8: RTC\_STAT Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000007_f1beec74cde8f79da5d91084666e98a37996e97571611f465eed42f65ac4ec11.png)

Table 22-12: RTC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/NW)          | WRZONE     | Write to RTC.                                                                                                                                                               |
| 9 (R/NW)           | CLKFAIL    | RTC Clock fail. The RTC_STAT.CLKFAIL bit indicates whether the RTC 1 Hz clock is functional. 0 = RTC 1 Hz clock is functional 1 = RTC 1 Hz clock failed 0 Clock functioning |

Table 22-12: RTC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | SWEXP      | Stop Watch Expiry. 1 = stopwatch counter expired 0 = stopwatch counter running                                                                            |
| 7 (R/NW)           | DAYALM     | Day Alarm. The RTC_STAT.DAYALM bit indicates that a time of day alarm has occurred (=1). 0 Alarm did not occur                                            |
| 6 (R/NW)           | ALM        | Alarm Flag. The RTC_STAT.ALM bit indicates that an alarm has occurred (=1). 0 Alarm did not occur                                                         |
| 5 (R/NW)           | DAY        | Day event. The RTC_STAT.DAY bit indicates that a day event (clock counter value x:23:59:59) has occurred (=1). 0 Event did not occur                      |
| 4 (R/NW)           | HOUR       | 1 Event occurred Hour event. The RTC_STAT.HOUR bit indicates that a hour event (clock counter value x:y: 59:59) has occurred (=1). 0 Event did not occur  |
| 3 (R/NW)           | MIN        | 1 Event occurred Minute event. The RTC_STAT.MIN bit indicates that a minute event (clock counter value x:y:z:59) has occurred (=1). 0 Event did not occur |
| 2                  |            | 1 Event occurred Second event.                                                                                                                            |
| (R/NW)             | SEC        | The RTC_STAT.SEC bit indicates that a second event has occurred (=1). 0 Event did not occur                                                               |

Table 22-12: RTC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | WRDONE     | Write Done. The RTC_STAT.WRDONE bit shows that the register write is complete. The RTC_STAT.WRDONE bit is applicable only for the alarm, clock and stopwatch regis- ters. 0 Write is done                                         |
| 0 (R/NW)           | WRPEND     | Write Pending. The RTC_STAT.WRPEND bit shows that a write to the RTC_CLK , RTC_ALM , RTC_STPWTCH , or RTC_INIT register is pending. This bit is automatically cleared and set by the hardware. 0 No write pending 1 Write pending |

## RTC Stop Watch Register

The RTC\_STPWTCH register contains the countdown value for the stop watch. The stopwatch counts down seconds from the programmed value and generates an interrupt (if SW\_INTEN=1) when the count reaches 0. The counter stops counting at this point and does not resume counting until a new nonzero value is written to the RTC\_STPWTCH register. Writing a value of 0 to the running stopwatch forces it to stop; no interrupt is generated in this case. The register can be programmed to any value between 0 and (2 16-1 ) seconds (that is, a range of 18 hours, 12 minutes and 15 seconds).

Figure 22-9: RTC\_STPWTCH Register Diagram

![Image](25_Real_Time_Clock_(RTC)_artifacts/image_000008_b34878938b6e72f47e1f4d97f88edf023f0b816bf61c2a90482a81a0859be19c.png)

Table 22-13: RTC\_STPWTCH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 15:0               | VALUE      | Stop Watch Value.                                                         |
| (R/W)              |            | The RTC_STPWTCH register contains the countdown value for the stop watch. |