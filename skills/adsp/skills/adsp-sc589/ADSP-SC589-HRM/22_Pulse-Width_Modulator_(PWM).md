## 19   Pulse-Width Modulator (PWM)

The pulse-width modulator (PWM) module is a flexible and programmable waveform generator. With minimal CPU intervention, the PWM peripheral can generate complex waveforms for:

- Motor control
- Pulse-coded modulation (PCM)
- Digital-to-analog conversion (DAC)
- Power switching
- Power conversion

The PWM module has four PWM pairs capable of three-phase PWM generation for source inverters for AC induction and DC brushless motors.

## PWM Features

Each PWM generation unit features:

- 16-bit center-based PWM generation unit
- Programmable PWM pulse width
- Single or double update modes
- Programmable dead time and switching frequency
- Twos-complement implementation which permits smooth transition to full-on and full-off states
- Dedicated asynchronous PWM shutdown signal
- Debounce filter option on trip inputs that allow the system programmer to filter out short transients

## Functional Description

The following sections provide details on the functionality of the PWM.

- Architectural Concepts

- Timer Units
- Channel Timing Control Unit
- Output Disable and Cross-Over Modes
- Sync Operation Modes

## ADSP-SC58x PWM Register List

The Pulse-Width Modulator unit (PWM) includes multiple timers (providing period flexibility) and channels (providing mode, interrupt, and pulse shape flexibility), permitting a wide variety of PWM output options for motor control and other applications. A set of registers governs PWM operations. For more information on PWM functionality, see the PWM register descriptions.

Table 19-1: ADSP-SC58x PWM Register List

| Name         | Description                                         |
|--------------|-----------------------------------------------------|
| PWM_ACTL     | Channel A Control Register                          |
| PWM_AH0      | Channel A-High Duty-0 Register                      |
| PWM_AH0_HP   | Channel A-High Heightened-Precision Duty-0 Register |
| PWM_AH1      | Channel A-High Duty-1 Register                      |
| PWM_AH1_HP   | Channel A-High Heightened-Precision Duty-1 Register |
| PWM_AH_DUTY0 | Channel A-High Full Duty0 Register                  |
| PWM_AH_DUTY1 | Channel A-High Full Duty1 Register                  |
| PWM_AL0      | Channel A-Low Duty-0 Register                       |
| PWM_AL0_HP   | Channel A-Low Heightened-Precision Duty-0 Register  |
| PWM_AL1      | Channel A-Low Duty-1 Register                       |
| PWM_AL1_HP   | Channel A-Low Heightened-Precision Duty-1 Register  |
| PWM_AL_DUTY0 | Channel A-Low Full Duty0 Register                   |
| PWM_AL_DUTY1 | Channel A-Low Full Duty1 Register                   |
| PWM_BCTL     | Channel B Control Register                          |
| PWM_BH0      | Channel B-High Duty-0 Register                      |
| PWM_BH0_HP   | Channel B-High Heightened-Precision Duty-0 Register |
| PWM_BH1      | Channel B-High Duty-1 Register                      |
| PWM_BH1_HP   | Channel B-High Heightened-Precision Duty-1 Register |
| PWM_BH_DUTY0 | Channel B-High Full Duty0 Register                  |
| PWM_BH_DUTY1 | Channel B-High Full Duty1 Register                  |
| PWM_BL0      | Channel B-Low Duty-0 Register                       |

Table 19-1: ADSP-SC58x PWM Register List (Continued)

| Name         | Description                                               |
|--------------|-----------------------------------------------------------|
| PWM_BL0_HP   | Channel B-Low Heightened-Precision Duty-0 Register        |
| PWM_BL1      | Channel B-Low Duty-1 Register                             |
| PWM_BL1_HP   | Channel B-Low Heightened-Precision Duty-1 Register        |
| PWM_BL_DUTY0 | Channel B-Low Full Duty0 Register                         |
| PWM_BL_DUTY1 | Channel B-Low Full Duty1 Register                         |
| PWM_CCTL     | Channel C Control Register                                |
| PWM_CH0      | Channel C-High Pulse Duty Register 0                      |
| PWM_CH0_HP   | Channel C-High Pulse Heightened-Precision Duty Register 0 |
| PWM_CH1      | Channel C-High Pulse Duty Register 1                      |
| PWM_CH1_HP   | Channel C-High Pulse Heightened-Precision Duty Register 1 |
| PWM_CHANCFG  | Channel Configuration Register                            |
| PWM_CHA_DT   | Channel A Dead-time Register                              |
| PWM_CHB_DT   | Channel B Dead-time Register                              |
| PWM_CHC_DT   | Channel C Dead-time Register                              |
| PWM_CHD_DT   | Channel DDead-time Register                               |
| PWM_CHOPCFG  | Chop Configuration Register                               |
| PWM_CH_DUTY0 | Channel C-High Full Duty0 Register                        |
| PWM_CH_DUTY1 | Channel C-High Full Duty1 Register                        |
| PWM_CL0      | Channel C-Low Pulse Duty Register 0                       |
| PWM_CL0_HP   | Channel C-Low Pulse Duty Register 1                       |
| PWM_CL1      | Channel C-Low Duty-1 Register                             |
| PWM_CL1_HP   | Channel C-Low Heightened-Precision Duty-1 Register        |
| PWM_CL_DUTY0 | Channel C-Low Full Duty0 Register                         |
| PWM_CL_DUTY1 | Channel C-Low Full Duty1 Register                         |
| PWM_CTL      | Control Register                                          |
| PWM_DCTL     | Channel DControl Register                                 |
| PWM_DH0      | Channel D-High Duty-0 Register                            |
| PWM_DH0_HP   | Channel D-High Pulse Heightened-Precision Duty Register 0 |
| PWM_DH1      | Channel D-High Pulse Duty Register 1                      |
| PWM_DH1_HP   | Channel DHigh Pulse Heightened-Precision Duty Register 1  |
| PWM_DH_DUTY0 | Channel D-High Full Duty0 Register                        |

Table 19-1: ADSP-SC58x PWM Register List (Continued)

| Name         | Description                                        |
|--------------|----------------------------------------------------|
| PWM_DH_DUTY1 | Channel D-High Full Duty1 Register                 |
| PWM_DL0      | Channel D-Low Pulse Duty Register 0                |
| PWM_DL0_HP   | Channel D-Low Heightened-Precision Duty-0 Register |
| PWM_DL1      | Channel D-Low Pulse Duty Register 1                |
| PWM_DL1_HP   | Channel D-Low Heightened-Precision Duty-1 Register |
| PWM_DLYA     | Channel A Delay Register                           |
| PWM_DLYB     | Channel B Delay Register                           |
| PWM_DLYC     | Channel C Delay Register                           |
| PWM_DLYD     | Channel DDelay Register                            |
| PWM_DL_DUTY0 | Channel D-Low Full Duty0 Register                  |
| PWM_DL_DUTY1 | Channel D-Low Full Duty1 Register                  |
| PWM_ILAT     | Interrupt Latch Register                           |
| PWM_IMSK     | Interrupt Mask Register                            |
| PWM_STAT     | Status Register                                    |
| PWM_SYNC_WID | Sync Pulse Width Register                          |
| PWM_TM0      | Timer 0 Period Register                            |
| PWM_TM1      | Timer 1 Period Register                            |
| PWM_TM2      | Timer 2 Period Register                            |
| PWM_TM3      | Timer 3 Period Register                            |
| PWM_TM4      | Timer 4 Period Register                            |
| PWM_TRIPCFG  | Trip Configuration Register                        |

## ADSP-SC58x PWM Interrupt List

Table 19-2: ADSP-SC58x PWM Interrupt List

|   Interrupt ID | Name      | Description        | Sensitivity   | DMA Channel   |
|----------------|-----------|--------------------|---------------|---------------|
|             30 | PWM0_SYNC | PWM0 PWMTMRGrouped | Edge          |               |
|             31 | PWM0_TRIP | PWM0 Trip          | Level         |               |
|             32 | PWM1_SYNC | PWM1 PWMTMRGrouped | Edge          |               |
|             33 | PWM1_TRIP | PWM1 Trip          | Level         |               |
|             34 | PWM2_SYNC | PWM2 PWMTMRGrouped | Edge          |               |

Table 19-2: ADSP-SC58x PWM Interrupt List (Continued)

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|             35 | PWM2_TRIP | PWM2 Trip     | Level         |               |

## ADSP-SC58x PWM Trigger List

Table 19-3: ADSP-SC58x PWM Trigger List Masters

|   Trigger ID | Name      | Description        | Sensitivity   |
|--------------|-----------|--------------------|---------------|
|           21 | PWM0_SYNC | PWM0 PWMTMRGrouped | Edge          |
|           22 | PWM1_SYNC | PWM1 PWMTMRGrouped | Edge          |
|           23 | PWM2_SYNC | PWM2 PWMTMRGrouped | Edge          |

Table 19-4: ADSP-SC58x PWM Trigger List Slaves

|   Trigger ID | Name            | Description               | Sensitivity   |
|--------------|-----------------|---------------------------|---------------|
|            8 | PWM0_TRIP_TRIG0 | PWM0 Trip Trigger Slave 0 | Pulse         |
|            9 | PWM0_TRIP_TRIG1 | PWM0 Trip Trigger Slave 1 | Pulse         |
|           10 | PWM0_TRIP_TRIG2 | PWM0 Trip Trigger Slave 2 | Pulse         |
|           11 | PWM1_TRIP_TRIG0 | PWM1 Trip Trigger Slave 0 | Pulse         |
|           12 | PWM1_TRIP_TRIG1 | PWM1 Trip Trigger Slave 1 | Pulse         |
|           13 | PWM1_TRIP_TRIG2 | PWM1 Trip Trigger Slave 2 | Pulse         |
|           14 | PWM2_TRIP_TRIG0 | PWM2 Trip Trigger Slave 0 | Pulse         |
|           15 | PWM2_TRIP_TRIG1 | PWM2 Trip Trigger Slave 1 | Pulse         |
|           16 | PWM2_TRIP_TRIG2 | PWM2 Trip Trigger Slave 2 | Pulse         |

## PWM Definitions

The following definitions are helpful when using the PWM module.

## Chopping

Used to simplify the design of isolated gate drive circuits for PWM inverters. If using a transformer coupled power device gate drive amplifier, then the active PWM signal must be chopped at a high frequency.

## Dead-Time

A short delay introduced between turning off one PWM signal (for example, AH) and turning on the complementary signal (for example, AL). This short time delay permits turning off a power switch (AH in this case) to completely

recover its blocking capability before the complementary switch is turned on. This time delay prevents a potentially destructive short-circuit condition from developing across the dc link capacitor of a typical voltage source inverter.

## Duty Cycle

The proportion of on time to the regular interval or period of time (expressed in percent, 100% being fully on). A low duty cycle corresponds to low power, because the power is off for most of the time.

## Switching Frequency

The average value of voltage (and current) fed to the load is controlled by turning the switch between supply and load on and off at a fast rate. The longer the switch is on compared to the off periods, the higher the total power supplied to the load.

## Architectural Concepts

A clock, whose period is t SCLK0\_0 , drives the PWM controller. The PWM generator produces four pairs (four highside and four low-side) of PWM signals on the eight PWM output pins. Each high and low pair signal constitutes a channel. For example, the PWM\_AL and PWM\_AH signals make up channel A, and the PWM\_BL and PWM\_BH signals make up channel B, and so on.

Each pair of channel outputs references either a main timer or an independent timer. These timers operate on a switching frequency determined by the PWM\_TM0 through PWM\_TM4 registers. There are two duty registers for every PWM output. The registers enable generation of symmetrical or asymmetrical waveforms. The waveforms produce lower harmonic distortion in three-phase PWM inverters, with minimal CPU intervention.

## Block Diagram

The PWM Block Diagram figure shows a block diagram that represents the main functional blocks of the PWM controller.

Figure 19-1: PWM Block Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000000_2e39a1ab75fa13dd1624925cf1d986ce111bb8cf8df9ce776f4ad8571d91d501.png)

The following list describes the primary blocks.

- Each pair of PWM signals references either the main timer or the independent timer.
- PWMTMR0 is the main timer and can trigger the delayed start of the other timers.
- Timing control units, one for each channel, together form the core of the PWM. The unit generates the required complex waveforms on the high-side and low-side outputs for the respective channel.
- Dead-time insertion occurs after the ideal PWM output pair is generated.
- The gate drive unit generates the high-frequency chopping signal and then mixes it with the requisite PWM output signals.
- The PWM shutdown and interrupt controller manage the various PWM shutdown modes for the timing unit and generate the requisite interrupt signals.
- The PWM sync pulse control unit generates the internal PWM\_SYNC pulse and also controls whether the external PWM\_SYNC input pulse is used.

## Timer Units

Five timers make up the time base for the PWM module. The main timer, PWMTMR0 operates at a switching frequency determined by the period register PWM\_TM0 . The four remaining timers (PWMTMR1 through PWMTMR4) can operate at independent switching frequencies determined by their respective registers.

The respective time registers ( PWM\_TM1 through PWM\_TM4 ) can be programmed to work at a multiple of the main timer frequency. In this case, the PWM\_DLYA through PWM\_DLYD registers control the lead-lag phase of a given timer based on the main timer PWMTMR0.

NOTE: The delayed operation of a timer requires one of the following:

- The register value of the timer must be equal to the PWM\_TM0 register value.
- The PWM\_TM0 value must be an integer multiple of each register of the timer. Non-integer multiples are not allowed.

## PWM Timer Period (PWM\_TM) Registers

The 16-bit read/write PWM period registers ( PWM\_TM0 through PWM\_TM4 ) control the PWM switching frequency. The fundamental timing unit of the PWM controller is t SCLK0\_0 . Therefore, the time increment (t SCLK0\_0 ) is 10 ns for a 100-MHz system clock (SCLK0\_0) frequency, f SCLK0\_0 . The value written to the register of a timer is effectively the number of t SCLK0\_0  clock increments in one half of a PWM period. The following equation describes the required timer register value as a function of the desired PWM switching frequency (f PWM ):

PWM\_TM = fSCLK0\_0/2 × fPWM

Therefore, the PWM switching period (T s ) is:

Ts = 2 × PWM\_TM × tSCLK0\_0

For example, for an f SCLK0\_0  of 100 MHz and a desired PWM switching frequency (f PWM ) of 10 kHz (T s  = 100 ms), the correct value to load into the timer register is:

PWM\_TM = 100 × 10 6  ÷ 2 × 10 × 10 3  = 5000

The largest value that can be written to the 16-bit timer register is 0xFFFF = 65,535. For an f SCLK0\_0  of 100 MHz, this value corresponds to a minimum PWM switching frequency of:

f PWM(min) = 100 × 10 6  ÷ 2 x 65535 = 762 Hz

NOTE: Timer register values of 0 and 1 are not defined. Do not use these values when the PWM outputs or PWM is enabled.

## Timer Unit Operation

The PWM timers are up-down counters, and they operate on the peripheral clock with a period of t CK . The period of the PWM timer is divided into two halves. In the first half, the timer roughly counts down from PWM\_TMx/2 to -PWM\_TMx/2. During this half, the PWM\_STAT.TMR0PHASE through PWM\_STAT.TMR4PHASE bits are held at 0. In the second half of the period, the timer roughly counts up from -PWM\_TMx/2 to PWM\_TMx/2. The PWM\_STAT.TMR0PHASE through PWM\_STAT.TMR4PHASE bits indicates a 1 during this half.

The actual partition of the periods varies slightly between odd and even values of the half-period, in the PWM\_TM[n] registers.

If a timer register value is odd, for example 11, then that timer loads +5 at the beginning of the period. The timer counts down from +5 to -5 in the first half, reloads -5 at the midpoint and counts up from -5 to +5 in the second half. The reload values at the period and mid-period boundaries are the same as the previous count. The timer counts 2 × 11 half-periods = 22 total counts in the entire period as shown in the Operation of Timer for Odd Value of PWM\_TM figure.

Figure 19-2: Operation of Timer for Odd Value of PWM\_TM

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000001_0524945355437d53115e137cefee9183e0d1ad00e97e128fe65b2d612f6a1e7c.png)

When the timer register value is even, for example 12, then that timer loads +5 at the beginning of the period. The timer counts from +5 to -6 in the first half, reloads -5 at the midpoint and counts up from -5 to +6 in the second half. The reload values at the period and mid-period boundaries are different from the previous count. It counts 2 × 12 half-periods = 24 total counts in the entire period as shown in the Operation of Timer for Even Value of PWM\_TM figure.

Figure 19-3: Operation of Timer for Even Value of PWM\_TM

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000002_a7bff36048be0641118eb116c924c7a7ca7dd43e6821a3fadd98e91d0c4ac74d.png)

NOTE: In the operation discussed in this section, double-buffering of all channel registers and the timer registers takes place at the period boundary of the respective timers.

## Phase Offset Control

The PWM timers (PWMTMR1 through PWMTMR4) can operate with a programmable phase lag relative to the main timer, PWMTMR0. To implement phase offset for a channel, use the counter-registers for channel delay ( PWM\_DLYA -PWM\_DLYD ) with the PWMTMR0 and set the PWM\_CTL.DLYAEN bit to l.

Phase offset works as follows.

1. If phase lag is used for channel A (and channel A uses PWMTMR1 to generate a duty cycle), when PWMTMR0 reaches its period boundary, it triggers the PWM\_DLYA register. The register counts out the number SCLK0\_0 cycles that are equal to the value programmed in the PWM\_DLYA register.
2. At the end of this count, the PWM\_DLYA register sends out a trigger to PWMTMR1. It receives a synchronization pulse in every period of PWMTMR0 at a point delayed from its period boundary by the value in the PWM\_DLYA register.

For more information on how channels can reference different timers for their outputs, see Channel Timing Control Unit.

NOTE: Satisfy the following conditions when using this feature on timer y for channel Y relative to PWMTMRx.

- Program the PWM\_DLY[n] register to a value less than 2 × PWM\_TM[n] .
- PWM\_TM0 = N × PWM\_TM[n] , where N is an integer.

The function of PWM\_TM[n] (PWMTMR1 in the example) differs in cases where PWM\_TM0 = PWM\_TM1 (Case 1) to cases where PWM\_TM0 = N × PWM\_TM1 (Case 2). The following examples describe both cases.

## Case 1: PWM\_TM0 = PWM\_TMy

When PWM\_TM0 = PWM\_TMy , PWMTMRy restarts its period after receiving the synchronization pulse from the channel delay register ( PWM\_DLY[n] ). If the trigger from the PWM\_DLY[n] register is late, PWMTMRy holds its count until the trigger occurs. If the trigger is a bit early, PWMTMRy reloads without regard to whether it has completed its current period. As a result, PWMTMRy resyncs with PWMTMR0 with the phase lag programmed in the PWM\_DLYA register in every one of its periods.

In this case, the expiration of the delay registers ( PWM\_DLY[n] ) is the period boundary of PWMTMRy. Now, all the double buffered registers related to the given channel update (except the delay registers which are double buffered at the period boundary of PWMTMR0).

The Phase Offset Control Using DELAY figure shows an example where:

- PWM\_TM0 , PWM\_TM1 , and PWM\_TM2 are programmed with the same value.
- PWM\_DLYA and PWM\_DLYB are programmed with values DELAY1 and DELAY2 respectively, such that DELAY2 &gt; DELAY1.

- The outputs of Channel A are referenced to PWMTMR1. The outputs of channel B are referenced to PWMTMR2.

Figure 19-4: Phase Offset Control Using DELAY

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000003_281530cf4efecd8af5e8c5b7770895a7f2598be2fa0d9772194f655704212737.png)

The delay registers are double buffered and the new value of DELAY reloads at the period boundary of PWMTMR0. The two options exist when the new value is different from the older one. The behavior of PWMTMRy in both these cases is discussed. The Impact of New DELAY Value on Timer Count for Equal Timer Periods figure shows the behavior in the two cases. It is assumed that channel B references its outputs to PWMTMR0 and channel A references its outputs to PWMTMR1.

1. The new delay value is higher than the previous value. Here the corresponding PWMTMRy allows more than one time period between consecutive triggers from the channel delay ( PWM\_DLYA -PWM\_DLYD ) registers. In this case, after reaching its period boundary, PWMTMRy holds its count at the period boundary and waits for the trigger from the channel delay register. The Impact of New DELAY Value on Timer Count for Equal Timer Periods figure shows case A functionality.
2. The next delay value programmed is smaller than the previous value. Here, the corresponding PWMTMRy allows only less than one time period between consecutive triggers from the channel delay register. Though the trigger comes earlier in this case, before PWMTMRy has counted out one full period, it reloads and starts its period again. The Impact of New DELAY Value on Timer Count for Equal Timer Periods figure shows case B functionality.

Therefore, PWMTMR1 waits and obeys a synchronization pulse from the PWM\_DLYA register in every one of its periods.

Figure 19-5: Impact of New DELAY Value on Timer Count for Equal Timer Periods

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000004_6fb088793c015d2dcf0b02e23736192babfe3bef2c97520b9ef5bad4c5e79a0c.png)

## Case 2: PWM\_TM0 = N x PWM\_TMy

In this case, within a single period of PWMTMR0 a program can fit multiple periods (N) of PWMTMRy. Additionally, the channel delay register is triggered only once every N periods of PWMTMRy.

The operation is as follows: Every Nth period of PWMTMRy, PWMTMRy expects a synchronization pulse from the PWM\_DLY[n] register. When this register counts out that period and the trigger has not yet arrived, PWMTMRy waits at the end of the period for the trigger. PWMTMRy starts counting down once the trigger arrives. If the trigger comes earlier, PWMTMRy restarts immediately without waiting to complete the period count.

In the intervening periods, PWMTMRy operates independently. As the period ends, PWMTMRy reloads and starts the next period without intervention from the channel delay register.

The Impact of DELAY Value Change for the Multiple Timer Periods shows an example with N = 2. PWMTMRy syncs up with PWMTMR0 every second period, and is free running across every odd period boundary.

Figure 19-6: Impact of DELAY Value Change for Multiple Timer Periods

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000005_cb1c37c15c21b0bceb2d6d2b9b0b87ff28f647a30b2e983c41628edcbd95b9f4.png)

## Channel Timing Control Unit

The channel timing control unit is the core of the PWM. There are four separate channels, each channel controlling a pair of output signals - the high-side output and the low-side output.

## Channel Control

The PWM\_CHANCFG register controls the static configuration of all the channels and is initialized once before the beginning of a PWM operation.

- NOTE: The PWM\_CHANCFG register is not double buffered. Do not change the contents of the register once the PWM is enabled.

Each channel works with a reference timer base. The time base can be either the main timer PWMTMR0 or the appropriate PWMTMRx. Configure the time base with the PWM\_CHANCFG.REFTMRA bit field as follows.

- Channel A works with PWMTMR0 or PWMTMR1
- Channel B works with PWMTMR0 or PWMTMR2
- Channel C works with PWMTMR0 or PWMTMR3
- Channel D works with PWMTMR0 or PWMTMR4

The double-buffered channel control registers ( PWM\_ACTL through PWM\_DCTL ) contain bits that control the dynamic pulse behavior of the channel outputs. These registers have bits that enable or disable outputs and select the pulse position of outputs (explained in the following section).

## Pulse Positioning and Duty Cycle Registers

The PWM uses the PULSEMODEHI and PULSEMODELO bit fields of the PWM\_[n]CTL registers to define the region within the timer period where the output pulses are positioned.

- When the PWM\_CHANCFG.MODELSC bit is 0, the PWM uses the PULSEMODEHI field to specify the pulse positioning for both the high-side and low-side outputs of the channel.
- When the bit is 1, the PWM uses PWM\_ACTL.PULSEMODELO to define the pulse positioning for the lowside output of the channel. It uses the PWM\_ACTL.PULSEMODEHI to define the pulse positioning for the high-side output of the channel.

Each channel output has two duty-cycle registers: PWM\_AH0 and PWM\_AH1 for the high-side output, and PWM\_AL0 and PWM\_AL1 for the low-side output. These registers determine the width of the output pulses. When the PWM\_CHANCFG.MODELSC bit is 0, the high-side duty-cycle registers are used to determine the width of the output pulse for the low side. The duty cycle range that can be programmed into these registers is between -PWM\_TM[n] /2 and + PWM\_TM[n] /2 , when ignoring dead time.

When including dead time for channel A, for PULSEMODEs 00 and 01, the programmed duty cycle is modified. The range is limited between the values [-PWM\_TM[n] /2 + PWM\_CHA\_DT ] and [+ PWM\_TM[n] /2 + PWM\_CHA\_DT ] considering the high-side output. For PULSEMODEs 10 and 11, the high-side duty cycle registers range is limited between values [ PWM\_TM[n] /2 + PWM\_CHA\_DT ] and [-PWM\_TM[n] /2 -PWM\_CHA\_DT ].

The following section explains dead time in detail.

- Switching Dead Time (PWM\_DT) Register

NOTE: Values programmed into these registers that fall outside these limits result in over or under modulation.

## Duty Cycle and Pulse Positioning Control

The PWM\_ACTL.PULSEMODEHI and PWM\_ACTL.PULSEMODELO fields control how the duty cycle registers modify the waveform of the high and low-side outputs. (The PWM\_ACTL.PULSEMODEHI and PWM\_ACTL.PULSEMODELO fields are referred to as pulse mode in the subsequent discussion.)

- Pulse mode = 00 - Produce a symmetrical pulse waveform around the center of the PWM period. In this mode, PWM uses only one of the duty cycle registers for an output. For example, for the AH output, PWM uses only the PWM\_AH0 register. In this mode, the values in the duty cycle registers are scaled such that a value of 0 produces a 50% duty cycle.
- Pulse mode = 01 - Produce an asymmetrical pulse waveform around the center of the PWM period. In this mode, PWM uses both duty cycle registers. For example, for the PWM\_AH output, PWM uses the PWM\_AH0 and PWM\_AH1 registers. In this mode, if the PWM\_AH1 register is programmed with the same value as the PWM\_AH0 register, the output is identical to the output when pulse mode =00.
- Pulse mode = 10 or 11 - Produce pulse waveforms either on the first half or the second half of the PWM period respectively. PWM uses both PWM\_AH0 and PWM\_AH1 registers.

Pulse mode = 10. If the low side works from the low-side duty-cycle registers, strictly adhere to the condition PWM\_AL0 &gt; PWM\_AL1 .

In pulse mode = 11. If the low side works from the low-side duty-cycle registers, strictly adhere to the condition PWM\_AL0 &lt; PWM\_AL1 .

The Pulse Positioning Modes figure shows the pulse positioning modes as previously described for PWM\_AH. In the figure, DUTY0 is the value in the PWM\_AL0 register and DUTY1 is the value in the PWM\_AH1 register. The step signal, count, indicates the output of the timer for channel A. In the example, the signal is configured as active high and dead time is zero.

Figure 19-7: Pulse Positioning Modes

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000006_5900d289679698fabe11baae05e1c9a7210dcc3861eb2708a886091526860d6e.png)

## Channel Low Side Output Dependent Operation Mode and Dead Time

The low-side output waveform can be programmed to depend on the waveform of the high-side output or to be independent. The PWM uses the PWM\_CHANCFG.MODELSC bit to control this functionality.

For example, channel A produces the high-side output PWM\_AH and the low-side output PWM\_AL. When the PWM\_CHANCFG.MODELSC bit =0, the low-side output is also generated using the high-side duty-cycle registers for pulse width, the PWM\_ACTL.PULSEMODEHI bits for pulse positioning and the PWM\_CHANCFG.POLAH bit for polarity. If the PWM\_CH[x]\_DT register is 0, the low-side output is an inverted version of the high-side output.

When the PWM\_CH[x]\_DT register is programmed with a non-zero value, both the high-side and low-side outputs are scaled symmetrically about the points of transition in the zero dead time case. The PWM scales the output by the value programmed in the PWM\_CH[x]\_DT register.

The Channel Outputs in Dependent Mode for Pulse Mode figures show the high and low-side outputs for the case with zero and non-zero dead time for PWM\_ACTL.PULSEMODEHI =00 and 01. DUTY0 is the value programmed into the PWM\_AH0 register. DUTY1 is the value programmed into the PWM\_AH1 register. The PWM\_CHANCFG.POLAH bit =1, indicates that both signals are active high. The PWM\_CHA\_DT register holds the value DT.

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000007_63a6ee72eb60f6cbd73f3092bba78d2d9f05a1d73fc36c41d12f97ced1952cb7.png)

Figure 19-8: Channel Outputs in Dependent Mode for Pulse Mode = 00

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000008_266ff78528b9c6990eff469dbba9c52c5a8ac8e47b96a6c8a369724443e27d08.png)

PULSEMODE = 01

Figure 19-9: Channel Outputs in Dependent Mode for Pulse Mode = 01

The following pair of figures shows the high and low-side outputs for the case with zero and non-zero dead-time for PWM\_ACTL.PULSEMODEHI =10 and 11. In the figures, DUTY0 is the value programmed into PWM\_AH0 register and DUTY1 is the value programmed into the PWM\_AH1 register. PWM\_CHANCFG.POLAH is 1 indicating that both signals are active high. The channel dead-time registers hold the value DT.

NOTE: Using dead time, the guidelines for programming the duty-cycle registers in pulse modes 10 and 11 given in Duty Cycle and Pulse Positioning Control are modified as follows:

Pulse mode 10: PWM\_xH0 - DT &gt; PWM\_xH1 + DT

Pulse mode 11: PWM\_xH0 + DT &lt; PWM\_xH1 - DT

Figure 19-10: Channel Outputs in Dependent Mode for Pulse Mode = 10

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000009_d6db33a1dd7a02691dbb6ff75db8bc82027a902252a06870308af23947399541.png)

Figure 19-11: Channel Outputs in Dependent Mode for Pulse Mode = 11

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000010_c6dd15121e424a1456b23a76bd5e512730d3c7cb800c98b243a77e8c5c2b5f5d.png)

## Channel High Side and Low Side Outputs, Independent Operation Mode

Independent control of the PWM\_AH0 and PWM\_AL0 channel outputs is possible by setting the PWM\_CHANCFG.MODELSA bit to 1. In this case, the PWM module:

- Generates PWM\_AH using the PWM\_AH0 register
- Uses the PWM\_AH1 register to configure pulse width

- Uses the PWM\_ACTL.PULSEMODEHI bit to configure pulse position
- Uses the PWM\_CHANCFG.POLAH bit to configure polarity
- Generates PWM\_AL using PWM\_AL0
- Uses the PWM\_AL1 register to configure pulse width
- Uses the PWM\_ACTL.PULSEMODELO bit to configure pulse position
- Uses the PWM\_CHANCFG.POLAL bit to configure polarity

NOTE: In independent mode, the dead-time insertion is not applicable. The hardware forces dead time to zero.

The PWM\_AH and PWM\_AL in Independent Operation Mode figure shows an example of the independent mode of operation where PWM\_AH and PWM\_AL work from different register bits.

Figure 19-12: PWM\_AH and PWM\_AL in Independent Operation Mode

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000011_0d81ca359d4a1f863f56ba63de3cd4c191ea0c511b768f81c866920e98467dfb.png)

PWM\_AH and PWM\_AL in Independent Mode of operation PWM\_AH and PWM\_AL can be positioned in the timer period with a given phase difference between them. Program the PWM\_ACTL.PULSEMODEHI and PWM\_ACTL.PULSEMODELO bits to different values to achieve this positioning as shown in the Channel Outputs Controlled Independently figure.

Figure 19-13: Channel Outputs Controlled Independently

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000012_d05de916ff91848684ed5f99c034420da72f70f634a08426875236ea57829c03.png)

## Switched Reluctance Motors Application

In typical power converter configurations for switched or variable reluctance motors, motor winding is connected between the two power switches of a given inverter leg. To allow for a complete circuit in the motor winding, turn on both switches at the same time.

PWM uses switched reluctance motors in the following configurations: hard chop, alternate chop, soft chop-bottom on, and soft chop- top on.

The Four SR Mode Types, Active High PWM Output Signals figure shows the four SR mode types as active high PWM output signals.

Hard chop mode contains independently programmed rising edges of the high and low signals of a channel in the same PWM half cycle. Both signals contain independently programmed falling edges in the next PWM half cycle. The PWM\_CHANCFG.POLAH and PWM\_CHANCFG.POLAL bits are programmed to same values.

Alternate chop mode is similar to normal PWM operation except that the PWM channel high and low signal edges are always opposite and are independently programmed. The PWM\_CHANCFG.POLAH and PWM\_CHANCFG.POLAL bits are programmed to opposite values. The low-side invert is the only difference between hard chop mode and alternate chop mode.

Soft chop - bottom on uses a 100% duty on the low side of the channel. Soft chop - top on uses a 100% duty on the high side of the channel. Similar to hard chop mode, PWM uses the PWM\_AH0 duty register for the high channel and the PWM\_AL0 duty register for the low channel.

Figure 19-14: Four SR Mode Types, Active High PWM Output Signals

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000013_60716dfae606630eb003a658a5163ad8d3a3fc0f8cd0466821d2505bc059845b.png)

## Switching Dead Time (PWM\_DT) Register

The second important parameter that must be set up in the initial configuration of the PWM controller is the switching dead time. Dead time is a short delay introduced between turning off one PWM signal (for example, AH) and turning on the complementary signal (for example, AL). This short time delay permits turning off a power switch (AH in this case) to completely recover its blocking capability before the complementary switch is turned on. This time delay prevents a potentially destructive short-circuit condition from developing across the DC link capacitor of a typical voltage source inverter.

The 10-bit, read/write channel A through channel D dead-time registers ( PWM\_CHA\_DT through PWM\_CHD\_DT ) control the dead time for channel-x. If the value carried by any dead-time register is PWMDT, the dead time, Td, for that channel is:

Td = PWM\_CH[x]\_DT × 2 × tSCLK0\_0

Therefore, a dead-time value of 0x00A introduces a 200-ns delay (for an SCLK0\_0 of 100 MHz). The delay occurs between turning off any PWM signal (for example, AH) and then turning on its complementary signal (for example, AL). The length of dead time can be programmed in increments of 2 × t SCLK0\_0  (or 20 ns for an SCLK0\_0 of 100 MHz). The channel A through channel D dead-time registers have a maximum value of 0x3FF (1023 decimal) and correspond to a maximum programmed dead time of:

Td(max)  = 1023 × 2 × t SCLK0\_0  = 1023 × 2 × 10 × 10 -9  = 20.5 µs for an f SCLK0\_0 rate of 100 MHz.

Write 0 to the PWM\_CHA\_DT through PWM\_CHD\_DT registers to program the dead time.

## Duty Cycle with Dead Time Control: Calculations for PULSEMODE 00

The duty cycle registers are scaled so that a value of 0 represents a 50% PWM duty cycle. The switching signals produced are also adjusted to incorporate the programmed dead-time value using the channel dead-time registers

( PWM\_CHA\_DT through PWM\_CHD\_DT ). The unit in this case produces active low signals so that a low level corresponds to a command to turn-on the associated power device.

The Dead Time Between Outputs in Dependent Mode figure shows a typical pair of PWM outputs, PWM\_AH and PWM\_AL. The time values in the figure indicate the integer value in the associated register and can be converted to time by multiplying by the fundamental time increment, t CK . In the example, channel A is working from PWM\_TM0 .

In the example, the pulse mode is set to 00 so that the switching patterns are perfectly symmetrical about the midpoint of the switching period. The dead time is incorporated by moving the switching instants of both PWM signals away from the instant set by the PWM\_AH0 register. Both switching edges are moved by an equal amount (DT × t CK) to preserve the symmetrical output patterns. Also shown is the PWM\_SYNC output pulse whose rising edge denotes the beginning of the switching period, and the PWM\_STAT.TMR0PHASE bit.

Figure 19-15: Dead Time Between Outputs in Dependent Mode

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000014_f6a209b7123792070047ecc8876a079e73354eba5d4ed2d71d5046b3f7d4d6e7.png)

The PWM timing unit produces the resulting on-times (active low) of the PWM signals over the full PWM period (two half-periods). The figure illustrates this timing. The timing can be written per the following equation.

<!-- formula-not-decoded -->

The negative values of T AH  and T AL  are not permitted and the minimum permissible value is zero, corresponding to a 0% duty cycle. In a similar fashion, the maximum value is T s , the PWM switching period, corresponding to a 100% duty cycle. Calculation of duty for other pulse modes can be similarly executed.

## Special Consideration for PWM Operation in Over-Modulation

The PWM timing unit can produce PWM signals with variable duty cycle values at the PWM output pins. In pulse modes 00 and 01, at the extremities of the modulation process, duty cycles of 0% and 100% occur. In pulse modes 01 and 10, at the extremities of the modulation process, duty cycles of 0% and 50% occur. The modulation is called full off when the lower extremity of modulation is set for any PWM timer period for the corresponding channel. The modulation is called full on when the higher extremity of modulation is set for any PWM timer period for the corresponding channel. In between, for other duty cycle values, the operation is termed normal modulation .

## Full On Modulation

In pulse modes 00 and 01, a PWM channel is in full on modulation if the high-side output of that channel is asserted. The output is asserted for the whole duration of the period of the PWM timer that channel is referencing. The conditions for full on modulation are:

- PWM\_xH0 DT &gt; PWM\_TMy/2 for pulse mode 00
- PWM\_xH1 DT &gt; PWM\_TMy/2 for pulse modes 00 and 01

In pulse mode 10, a PWM channel is in full on modulation if the high-side output of that channel is asserted. The output is asserted for the whole duration of the first half period of the PWM timer that the channel is referencing. The conditions for full on modulation are:

- PWM\_xH0 DT &gt; PWM\_TMy/2 for pulse mode 10
- PWM\_xH1 + DT &lt; PWM\_TMy/2 for pulse mode 10

In pulse mode 11, a PWM channel is in full on modulation if the high-side output of that channel is asserted. The output is asserted for the whole duration of the second half period of the PWM timer that the channel is referencing. The conditions for full on modulation are:

- PWM\_xH0 + DT &lt; PWM\_TMy/2 for pulse mode 11
- PWM\_xH1 DT &gt; PWM\_TMy/2 for pulse mode 11

## Full Off Modulation

In pulse modes 00 and 01, a PWM channel is in full off modulation if the high-side output of that channel is deasserted. The output is deasserted for the whole duration of the period of the PWM timer that channel is referencing. The conditions for full off modulation are:

- PWM\_xH0 DT &lt; PWM\_TMy/2 for pulse mode 00
- PWM\_xH1 DT &lt; PWM\_TMy/2 for pulse modes 00 and 01

In pulse mode 10, a PWM channel is in full off modulation if the high-side output of that channel is deasserted. The output is deasserted for the whole duration of the first half period of the PWM timer that the channel is referencing. In the second half-period, it is deasserted anyway. The conditions for full off modulation are:

- PWM\_xH0 DT &lt; PWM\_TMy/2 for pulse mode 10
- PWM\_xH1 + DT &lt; PWM\_xH0 DT for pulse mode 10

In pulse mode 11, a PWM channel is in full off modulation if the high-side output of that channel is deasserted. The output is deasserted for the whole duration of the second half period of the PWM timer that the channel is referencing. In the first half of the period, it is deasserted anyway. The conditions for full off modulation are:

- PWM\_xH0 + DT &gt; PWM\_TMy/2 for pulse mode 11
- PWM\_xH1 DT &gt; PWM\_xH0 + DT for pulse mode 11

## Normal Modulation

All other cases of modulation fall under this category.

## Emergency Dead-Time Delays

Sometimes, during modulation transition, it is necessary to insert more emergency dead-time delays to prevent potential shoot through conditions in the inverter. (For example, when the PWM transitions into or out of full on or full off modulation.) Disabling and enabling usage (related to the PWM\_ACTL.DISHI and PWM\_ACTL.DISLO bits) also can potentially cause outputs to violate shoot-through condition criteria. Another case is when large values vary the phase delay of a PWM timer. These transitions are detected automatically. If appropriate for safety, an emergency dead-time is inserted to prevent shoot through conditions.

There is another atypical case for the insertion of the additional emergency dead time. It occurs when both PWM signals do not toggle within a dead time of each other. In this case, insert more emergency dead time into one of the PWM signals of a given pair during these transitions. The dead-time delay is inserted into the PWM signal that is toggling into the on-state. In effect, an amount (2 × DT × t CK ) from the rising edge of the opposite output delays the turn-on of this signal. After this delay, the PWM signal is allowed to turn-on provided the desired output is still scheduled to be in the on-state after the emergency dead-time delay.

The Over Modulation Transition Example figure illustrates two examples of such a transition. In the figure, PWM\_ACTL.PULSEMODEHI is kept at 1. The PWM\_AH signal has been in full on modulation for some time and, during the current period, its pulse mode is changed to 10, keeping the full on condition. At the half-period boundary, PWM\_AH is forced to transition to a deasserted state because pulse mode is 10. An emergency dead-time is inserted on the low-side output.

Figure 19-16: Over Modulation Transition Example

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000015_bf35370038afdda26d672098392c3d4a33f7f54dd0b9a1f6ec6da1481c1cb89a.png)

## Gate Drive Unit

The gate drive unit of the PWM adds features that simplify the design of isolated gate drive circuits for PWM inverters. When using a transformer coupled power device gate drive amplifier, the active PWM signal must be chopped at a high frequency. The PWM\_CHOPCFG register allows the programming of this chopping mode for high frequency. The chopped active PWM signals can be required for the high-side drivers only, for the low-side drivers only, or for both the high-side and low-side switches. Therefore, independent control of this mode for both high and lowside switches is included with two separate control bits in the PWM\_CHANCFG register.

The High-Side and Low-Side Outputs With Gate Chop Enabled figure shows the typical PWM output signals with high-frequency chopping enabled on both high-side and low-side signals. Chopping of the PWM outputs is enabled by setting bits in PWM\_CHANCFG register. The 8-bit PWM\_CHOPCFG.VALUE value controls the high frequency chopping. The following equation gives the period of this high frequency carrier.

<!-- formula-not-decoded -->

and the chopping frequency is therefore an integral subdivision of the peripheral clock frequency:

<!-- formula-not-decoded -->

The PWM\_CHOPCFG.VALUE value can range from 0 to 255, corresponding to a programmable chopping frequency rate from 122 kHz to 31.25 MHz for a 125 MHz, f CK  rate. Program the gate drive features before enabling the PWM controller. Do not change the gate drive features during normal operation of the controller. Following a reset, clear all bits of the PWM\_CHANCFG register so that high frequency chopping is disabled, by default.

Figure 19-17: High-Side and Low-Side Outputs With Gate Chop Enabled

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000016_26ffb97b67dc0f2de4ca7dbb28b13f0b83160a0e4786976ddb0f7a5355f7619b.png)

## Output Control Feature Precedence

The order of applying output control features to the PWM signal is important and significant. Use the following order for applying the signal features to the PWM output signal.

1. Duty generation
2. Cross-over
3. High-side or low-side disable
4. Emergency dead-time insertion
5. HPPWM correction for wrong programming
6. Gate-drive chopping
7. Polarity
8. Heightened-Precision PWM (HPPWM) edge placement

NOTE: When HPPWM operation is enabled, the cross-over feature and the gate-drive chopping feature must be disabled.

## Operating Modes

The PWM generator is capable of operating in the following modes:

- Sync Operation Modes
- Output Disable and Cross-Over Modes
- Heightened-Precision Edge Placement
- Emulation Mode

## Sync Operation Modes

The PWMx\_SYNC pins can operate as internally generated or externally generated. If its internally generated, PWMs can drive the signal to synchronize other PWMs and other devices. If externally generated, the PWMs can be synchronized externally.

## External (Triggered) PWM Sync Generation

By setting the PWM\_CTL.EXTSYNC bit, the PWM is set up in a mode to expect an external PWM\_SYNC signal on the PWM\_SYNC\_IN pin through the TRU. The trigger source can be any TRU1 trigger master. Multiple PWM units can be precisely synchronized by selecting the same trigger master as the sources for each PWM sync trigger slave. Examples of useful trigger masters include:

- A PWM\_SYNC GPIO master. This option allows synchronizing the PWMs to an off-chip timing source. (The PWM\_CTL.EXTSYNC bit must be programmed to 1 to enable the GPIO input.) The PWM\_SYNC GPIO pads

are connected through the pin mux to configurable trigger masters using PWM master IDs TRGM\_SYS\_PWMn\_SYNC\_IN. Using the TRU, these masters can be connected to the PWM\_SYNC trigger slaves in any desired combination.

- A TTU trigger output. This option can include a member of a TTU trigger group, which can also control the timing of other devices such as ADCs, SINC filter inputs, or GP timers. The TTU supports relative trigger delays so that timing offsets can be applied to manage system latencies with precision.
- A general-purpose timer trigger master
- A software trigger master. This option allows starting one or more PWM units simultaneously by an MMR write.

The external PWM\_SYNC signal only determines the operation of the main timer PWMTMR0.

Synchronize the external sync by setting the PWM\_CTL.EXTSYNCSEL bit to 0 (assumes the external PWM\_SYNC selected is asynchronous).

The external PWM\_SYNC period is expected to be an integer multiple of the value of the PWM\_TM0 period register. When the rising edge of the external PWM\_SYNC is detected, the PWMTMR0 timer is restarted at the beginning of its period. If the external PWM\_SYNC period is not exactly an integer multiple of the internal PWM\_SYNC , the behavior of the PWM channel outputs which are referenced to PWMTMR0 are clipped.

The effect latency from PWM\_SYNC\_IN to the PWM outputs is about three clock cycles in synchronous mode, and five clock cycles in asynchronous mode.

CAUTION: Do not change the value of the PWM\_CTL.EXTSYNC bit while the PWM is enabled ( PWM\_CTL.GLOBEN =1).

## Output Disable and Cross-Over Modes

Each PWM\_ACTL channel control register contains separate enable bits for the high and low-side signals. The PWM module uses the PWM\_ACTL.DISHI and PWM\_ACTL.DISLO bits in the channel A control register to enable or disable the PWM\_AH and PWM\_AL outputs respectively. If the disable bit is set (=1), then the corresponding PWM output is disabled, irrespective of the value of the corresponding duty cycle register. This PWM output signal remains in the OFF state as long as the corresponding enable or disable bit is set.

The cross-over bit ( PWM\_ACTL.XOVR ) allows programs to send the low-side output through the high-side output pin and the high-side output through the low-side output pin.

One example uses the following configuration:

- The PWM\_AH0 register =0
- The PWM\_CHANCFG.MODELSC bit =0
- The PWM\_ACTL.DISLO bit =1
- The PWM\_ACTL.XOVR bit =1

The low-side output remains off, as in the case without crossover. The difference in cross-over is that the high-side output changes character and becomes like the low-side. What actually occurs is that the low-side duty cycle is sent to the high-side output pins, and the high-side duty cycle is sent to the low side pins. Because the PWM\_ACTL.DISLO bit =1, the low-side pin remains off (see Output Control Feature Precedence).

The XOVR and DISHI/DISLO Functionality figure shows this example. In case 1, PWM\_ACTL.XOVR =0; and in case 2, PWM\_ACTL.XOVR =1.

Figure 19-18: XOVR and DISHI/DISLO Functionality

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000017_44b18bcf23c165b31ea2e4ed342f74109e2d706c002b9dff523249e8b8f405d3.png)

## Brushless DC Motor (Electronically Commutated Motor) Control

In the control of an electronically commutated motor (ECM), only two inverter legs are switched at any time. Often, the high-side device in one leg must be switched on at the same time as the low-side driver in a second leg. It is possible to turn on the high-side switch of phase A and the low-side switch of phase B at the same time by:

- Programming identical values for the duty cycles for two PWM channels (for example, PWM\_CH0 = PWM\_CH1 ), and
- Setting the PWM\_BCTL.XOVR bit to crossover the BH and BL pair of PWM signals

To control ECM, normally the third inverter leg (phase C in this example) is disabled for a number of PWM cycles. To implement this function, both the PWM\_CH and PWM\_CL outputs are disabled by setting the PWM\_CCTL.DISHI and PWM\_CCTL.DISLO bits.

In normal ECM operation, each inverter leg is disabled for certain time periods so that the PWM channel registers change based on the position of the rotor shaft (motor commutation).

Figure 19-19: ECM Control

## Heightened-Precision Edge Placement

Heightened-precision edge placement allows a fine-grained edge placement within the system clock period. The Heightened-Precision Steps in a Single SCLK Period figure shows how the SCLK0\_0 aligned edge is moved to finer resolution.

Figure 19-20: Heightened-Precision Steps in a Single SCLK0\_0 Period

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000018_06f2e30d3e83b62d31c4d7c2f54f03060b03fb5889bd55f5c7077d161193a13d.png)

The heightened-precision mode is enabled by setting bits that correspond to the high or low side output option for the channel in the PWM\_CHANCFG register. bit. The PWM\_AH0\_HP and PWM\_AH1\_HP registers work alongside the PWM\_AH0 and PWM\_AH1 registers to provide the overall resolution. The following example explains how signed decimal programming is implied for the heightened-precision duty values.

For the PWM\_AH output, the duty-cycle register-pair PWM\_AH0 and PWM\_AH0\_HP work together in a Q15.8 signed two's complement fixed-point format as shown in the Duty Cycle Notation for Heightened-Precision Edge Placement figure. The weight of bit position at k is 2k.

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000019_6458443575bcbfca55935df2db4460672242e4e332b87725f1d6624daf3937b5.png)

Figure 19-21: Duty Cycle Notation for Heightened-Precision Edge Placement

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000020_70e74e7a2e9c8726b66fd0b6e45880ec5f91d4ec2101625d0ff55a1f4279aab6.png)

In the normal modes of operation (not involving heightened-precision edge placement), only the PWM\_AH0 register value is programmed. The duty value programmed is a two's complement integer value. If a value of -1 is desired, the PWM\_AH0 register is programmed with the number 0xFFFF (which is the two's complement of 1 in 16 bits).

In the heightened-precision mode, if a duty value corresponding to 0.75 is required, the equivalent two's complement value of 0.75 in the Q15.8 format is computed: 1111\_1111\_1111\_1111.0100\_0000 = 0xFFFF.40. In this case, the PWM\_AH0 register is programmed to 0xFFFF and the PWM\_AH0\_HP register is programmed to 0x40.

## Heightened-Precision Edge Placement Example

The following is an example of heightened precision edge placement.

On the positive side of the fractional of the duty cycle, at 2 t CK + 0.25 t CK , the values for the PWM\_AH0 and PWM\_AH0\_HP registers are calculated as follows.

The PWM\_AH0 =0x0002 and PWM\_AH0\_HP =0x40 (bits 7:6).

The PWM\_AH\_DUTY0 register contains the bit fields from both the PWM\_AH0 and PWM\_AH0\_HP registers. Bits [15:14] represent the decimal part or heightened-precision value and bits [31:16] represent the coarse duty cycle. The value for the combined registers is PWM\_AH\_DUTY0 =0x00024000.

On the negative side of the fractional of the duty cycle, at -2 t CK - 0.25 t CK , the values for the PWM\_AH0 and PWM\_AH0\_HP registers are calculated as follows:

The coarse register represents the next count of the coarse value for negative values so that -2 becomes -3. These values are the two's complement of the positive offset (value = 3) PWM\_AH0 = 0xFFFD and PWM\_AH0\_HP =0xC0 (bits 7:6).

To derive the correct format for a negative duty-cycle value, for example, -2.25, use: coarse value + 1 =3 for the coarse value and 1 for 0.25. Write out the absolute value as a 32-bit number first:

0000 0000 0000 0011 (.) 0100 0000 0000 0000

Then take the two's complement of the entire 32-bit number:

1111 1111 1111 1101 (.) 1100 0000 0000 0000

NOTE: This value is also written into the full duty register ( PWM\_AH\_DUTY0 ). The correct value for the combined registers written in the PWM\_AH\_DUTY0 register is 0xFFFDC000.

NOTE: The above examples only consider 2 bits of precision, while the ADSP-CM41xF supports up to 4 bits. That means bits [7:4] of the PWM\_AH\_DUTY0 register can be used to define the higher precision. Bits

[15:12] of represent the decimal part or heightened-precision value and bits [31:16] represent the coarse duty cycle.

## Sample Waveforms for High- and Low-Side with Precision Placement

When the PWM module uses heightened-precision in the dependent mode of operation, both high and low-side outputs shift in the same direction. This operation can result in pulse-expansion of the high-side and pulse-contraction of the low-side or conversely.

The Output Shift in The Same Direction figure shows an example of a case with DT =1, and pulse expansion occurs on the high-side and pulse-contraction on the low-side. It juxtaposes a case where the PWM module does not use heightened-precision and a case where PWM does use it.

Figure 19-22: Output Shift in The Same Direction

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000021_635416f68a27962c338f4bed66a63c50eb25723b43060254ebc66a999ea18eba.png)

The following Precision Placement: PULSEMODE figures illustrate the cases for pulse modes 1, 2, and 3 (pulse mode 00 is a trivial case of pulse mode 01). The pulse modes are configured in the PWM\_ACTL and PWM\_BCTL channel control registers. The figures show what happens to the edges as a decimal part is added to a programmed positive duty. In each case, assume that the original PWM\_AH0.DUTY register value, which is the coarse duty value, changes to the PWM\_AH1.DUTY value after programming the enhanced-resolution ( PWM\_AH0\_HP , PWM\_AH1\_HP ) registers. For example, changing 14 to 14.25 and 10 to 10.75. The figures are not drawn to these numbers. For negative duty values, the shifts are in the opposite direction.

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000022_a03255c043fbaf979dad62c613feb6f42902b8c3fe3ef260c819aad494fe4216.png)

Figure 19-23: Precision Placement: PULSEMODE=01

Figure 19-24: Precision Placement: PULSEMODE=10

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000023_755560294ec66fc958a91e1c477460c83c1e23c92703ae0d9ec9e6fe6ddb313c.png)

Figure 19-25: Precision Placement: PULSEMODE=11

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000024_a2e986e23b7771f29d2f70238565d6408d32a8eae8be43fc9b15f82df73fd65f.png)

## Emulation Mode

Depending upon system configuration, a debug halt may be presented to the PWM unit. The PWM has local control of its response to an emulator halt based on the PWM\_CTL.EMURUN bit setting.

- PWM\_CTL.EMURUN =1. When the processor is halted in emulation mode, the outputs continue to toggle and be driven out of the PWM block. The counters and status register bits are set or reset according to the PWM timer count or period settings.
- PWM\_CTL.EMURUN =0. When the processor is halted in emulation mode, the outputs are shut down (enter their inactive state based on polarity), and all counters that affect status register bits are paused.
- At restart, the PWM counters resume from their paused value. The outputs are still held in their inactive state.

## Event Control

The PWM uses bits in the PWM\_IMSK and PWM\_ILAT registers for event control. These registers allow masking and show masked interrupt request status bits, respectively. The interrupt status bits are latched and held on the interrupt event. The software must write a 1 to clear the interrupt status bit, usually during the interrupt service routine.

The timer period (TMRxPER) interrupt requests are configured using the PWM\_ILAT.TMR0PER -PWM\_ILAT.TMR4PER bits. The PWM uses the interrupts to execute an interrupt service routine (ISR) periodically. The ISR updates the PWM channel control and duty registers (according to a control algorithm based on expected operation and sampled existing operation). The TMRxPER interrupts also can trigger an ADC to sample data for use during the ISR.

The PWM uses the PWM\_CTL.INTSYNCREF bit field to control the PWM\_SYNC interrupt request. The PWM uses the bit field to assign the interrupt request to a system user interrupt of the core. The PWM\_SYNC can be configured to be either internal or externally driven using the PWM\_CTL.EXTSYNC bit. When configured as an external sync, the signal can be further configured as synchronous or asynchronous using the PWM\_CTL.EXTSYNCSEL bit.

As an example, when the PWM\_SYNC interrupt is serviced:

- The ADC samples data
- The data is algorithmically interpreted
- New PWM channel duties are calculated and written to the PWM

More sophisticated implementations include different startup, run time, and shutdown algorithms to determine PWM channel duties based on expected behavior and further features.

During the PWM\_SYNC interrupt driven control loop, only the channel delay registers, the duty registers, and the channel C high pulse duty register values are typically updated. To see programming limitations on the PWM registers, see the "Register Descriptions".

Status information about the PWM is available in the PWM\_STAT register, which stores all status bits, including raw interrupt status bits. In particular, the period boundary of each timer is available, as well as status bits. The PWM uses the status bits to indicate whether the operation is in the first half or the second half of the timer. Additionally, the TRIP status is also available. For more information on TRIP interrupts, see T rip Control Unit .

## Trip Control Unit

The PWM output signals can be shut-off in a number of different ways. The trip inputs ( PWM\_TRIP[n] ) can be mapped to provide either a temporary or permanent shutdown on any channel outputs (hi/lo/pair). This shutdown mechanism is asynchronous so that the associated PWM output disable circuitry does not go through any clocked logic. This functionality ensures correct PWM shutdown even in the event of a loss of the processor clock. In addition to the hardware shutdown features, the PWM system can be shut down in software with the PWM\_CTL.SWTRIP bit.

The external trip signal PWM\_TRIP0 , that comes from GPIO generates the TRIP0 input whereas the trigger slaves PWM0\_TRIP\_TRIGn, PWM1\_TRIP\_TRIGn and PWM2\_TRIP\_TRIGn generate TRIP1 input.

During any external trip event (if not disabled), the PWM outputs are turned off. When a PWM output is turned off, it means that the output level is held at a polarity opposite that given in the PWM\_CHANCFG.POLAH through PWM\_CHANCFG.POLDH (channel high side polarity) bits. The PWM sync pulse continues to operate, when it is already enabled. A PWMTRIP interrupt occurs if unmasked, to notify the software of this event. In dependent mode of operation, both high and low-side outputs refer to the channel high side polarity bits.

Even if the clock to the PWM is damaged, an external trip event turns off the PWM outputs. In this case, the PWMTRIP interrupt request may not occur.

The PWM trip unit processes hardware or software fault conditions and shuts down the PWM channel outputs immediately on the occurrence of these conditions. The PWM can enable the shutdown mechanism separately for each channel. The design also allows for a self-restart mechanism to be enabled on a channel. Self-restart reenables the channel outputs following the fault condition (allowed only on hardware trips) when the PWMTMRy that the channel is using reaches its period boundary.

These sources are active low inputs where a falling edge on either of these pins indicates a fault condition.

The trip unit can shut down an output of a particular channel in response to the fault event from the PWM\_TRIP0 pin. To enable this functionality, program the PWM\_TRIPCFG.EN0A bit corresponding to the channel.

Program the PWM\_TRIPCFG.MODE0A bits to specify the restart mechanism for a channel that has been tripped.

1. If the PWM\_TRIPCFG.MODE0A bit =0, once tripped, a trip condition is registered on this channel in the PWM\_STAT.FLTTRIPA bit and the outputs of that channel are immediately shut down. This condition is called a fault trip condition. To resume channel output when a fault trip occurs, write a 1 to clear the PWM\_STAT.FLTTRIPA bit. A processor write cannot clear the bit when the trip condition is still active. The raw trip status is available for both pins in the PWM\_STAT.RAWTRIP0 register bits.
2. If the PWM\_TRIPCFG.MODE0A bit =1, once tripped, a trip condition is registered on this channel in the PWM\_STAT.SRTRIPA bit and the outputs of that channel are immediately shut down. This condition is

called a self-restart trip condition. If the trip condition is not active at the next period boundary of the PWMTMRy that the channel is using, the status register bit is cleared. The outputs are restored.

The trip input pins have an external pull-down resistor on the chip pin. If the pin becomes disconnected, the processor disables the PWM.

In addition to the hardware trip conditions, a global software trip bit in the PWM\_CTL register allows for a softwareforced fault trip condition. When the global software trip bit is set to 1, irrespective of the values in the PWM\_TRIPCFG register, it sets all the PWM\_STAT.FLTTRIPA bits and also gates the channel outputs. To remove the trip condition from the channel, perform a W1C on the PWM\_STAT.FLTTRIPA bit of the particular channel.

If the PWM\_TRIPCFG.EN0A bit is set to 1 to, for any channel, then the occurrence of a fault condition on the PWMTRIPy bit is logged in the PWM\_STAT.FLTTRIPA register bit. If the corresponding PWM\_IMSK.TRIP0 bit = 1, then an interrupt request is generated. T ripping a channel output does not interfere with PWM\_SYNC generation.

The Operation Under Hardware Fault Conditions figure shows an example where PWMTRIP0 is enabled on channel A as self-restart trip. Channel A works with the PWM\_CHANCFG.POLAH bit =1. In period 2, the PWM\_AH signal is full on modulated, and tries to rise at the period boundary where the self-restart occurs for the channel. However, since the low-side output of the channel was only recently removed due to a trip, the rise edge on PWM\_AH is delayed until the emergency dead-time period is over. PWMTRIP1 is enabled on channel B as a fault trip. Channel B works with the PWM\_CHANCFG.POLAH bit =0. PWMTRIP1 stays low for an extended time period. The first processor write to reenable the channel output fails. The second processor write passes since the fault condition has gone away.

NOTE: Dead time is ensured on re-enabling the channel outputs after trip.

- NOTE: Programs must not allow changes in the configuration or enable bits of PWM\_TRIPCFG register within ± 10 clock cycles of when the external trip pulse toggles. (The configuration or enable bits of PWM\_TRIPCFG register select between trip enable and disable). If this time frame is not followed, then unexpected behavior occurs.

## Programming Model

The following sections provide general (and some application-specific) programming steps for configuring and using the PWM module.

- Programming Model for Three-Phase AC Motor Control

## Programming Model for Three-Phase AC Motor Control

The PWM Module and Interaction with System figure shows how the PWM unit (green) interfaces to both software (blue) and hardware (yellow). The software configures the unit, calculates duty cycles (Duty A, Duty B, Duty C), and services the interrupts generated by the module (PWM SYNC IRQ, TRIP IRQ). The hardware applies the gate signals (AH, AL, BH, BL, CH, CL) to the inverter and provides an over-current trip signal back to the unit (TRIP0).

The typical three-phase AC motor configuration shown in the PWM Module and Interaction with System figure applies for both permanent magnet and induction motor types.

Figure 19-26: PWM Module and Interaction with System

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000025_f4aa3d96b5a8e77d325f5d50d86c5c99a2f08a0816389d07827699e9c8339367.png)

## System Parameters

The following system parameters (characteristics) influence the module configuration for this application. This example system features:

- One three-phase AC machine
- B6 inverter
- SVPWM, including both linear- and over-modulation
- Switching frequency of 20 kHz
- Dead time of 1us
- Trip signal generated by hardware
- Active high-level gate drive
- Core frequency of 200 MHz
- Peripheral clock of 100 MHz

## System State Sequencing

Managing the system state and sequence of states is critically important when programming the PWM module. The PWM System States figure provides an overview of these states.

Figure 19-27: PWM System States

As shown in the state diagram, the module configuration is updated on state transitions (indicated by the arrows). The transitions are initialization, motor start, PWM sync interrupt request (on each), and motor stop. The following sections discuss the transitions in detail.

- PWM Initialization for Motor Control
- PWM Enable for Motor Control
- PWM Response to Sync Interrupt for Motor Control
- PWM Disable (and Stop the Motor) for Motor Control

## PWM Initialization for Motor Control

The processor must program the PWM at power-up as follows and repeat this programming to bring the PWM and the system into a known (safe) state.

1. Place the PWM module in a safe state and set up synchronization of the module using the following bitwise operations on the PWM\_CTL and PWM\_CHANCFG registers:

```
PWM_CTL &= 0xFFE0FF08 PWM_CTL |= 0x20000 PWM_CHANCFG &= 0x80808080 PWM_CHANCFG |= 0x24242424
```

These operations result in the following bit settings:

- Disable PWM ( PWM\_CTL.GLOBEN =0)
- Disable delay for channels A, B, C, D ( PWM\_CTL.DLYAEN through PWM\_CTL.DLYDEN =0). All phases must run with same phase.
- Use internal synchronization by timer TMR0 ( PWM\_CTL.EXTSYNC =0, PWM\_CTL.EXTSYNCSEL =1)
- The same timer, TMR0 ( PWM\_CTL.INTSYNCREF = b#000) synchronizes all phases.

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000026_9bd3e75a0d44d474afd5df062cbbc62de738c905dd1df56045a7981748e02dc6.png)

- Low-side is always the inverse of high-side ( PWM\_CHANCFG.POLAL through PWM\_CHANCFG.POLDL = 1).
- System uses active high gate driver ( PWM\_CHANCFG.ENCHOPAH through PWM\_CHANCFG.ENCHOPDH =1).
- Disable gate chopping ( PWM\_CHANCFG.ENCHOPAL through PWM\_CHANCFG.ENCHOPDL =0). PWM does not use the pulse transformer.
2. Set up the trip and associated interrupt requests using the following bitwise operations on the PWM\_TRIPCFG and PWM\_ILAT registers:

```
PWM_TRIPCFG &= 0xF0F0F0F0 PWM_TRIPCFG |= 0x1010101 PWM_ILAT &= 0xFFE0FFFC PWM_ILAT |= 0x1
```

These operations result in the following bit settings:

- All phases must shut down simultaneously in case of fault: ( PWM\_TRIPCFG.EN0A through PWM\_TRIPCFG.EN0D =0, PWM\_TRIPCFG.MODE0A through PWM\_TRIPCFG.MODE0D =0, PWM\_TRIPCFG.EN1A through PWM\_TRIPCFG.EN1D =0, PWM\_TRIPCFG.MODE1A through PWM\_TRIPCFG.MODE1D =0)
- Enable TRIP0 as fault trigger for all channels ( PWM\_TRIPCFG.EN0A through PWM\_TRIPCFG.MODE1D =1).
- For thermal control and synchronization, SW intervention is needed at trip. Do not use automatic restart of any channels.
- Generate an interrupt at trip on TRIP0 ( PWM\_ILAT.TMR0PER = 1).
3. Configure the PWM channels using the following bitwise operations on the PWM\_TRIPCFG and PWM\_ILAT registers:

```
PWM_CHA_DT=0x32 PWM_CHB_DT=0x32 PWM_CHC_DT=0x32 PWM_TM0 = 0x9C4 PWM_ACTL = 0xFFFFF0000 PWM_BCTL = 0xFFFFF0000 PWM_CCTL = 0xFFFFF0000 PWM_AH0 = 0x0 PWM_BH0 = 0x0 PWM_CH0 = 0x0
```

These operations result in the following bit settings:

- Configure a dead time of 1 µs (DT =0x32 =0x32)
- Configure a PWM frequency of 20 kHz (assuming a system clock frequency of 100Mhz, so a clock divisor of 5000:1) ( PWM\_TM0 =0x9C4)

- Disable all outputs ( PWM\_ACTL.DISHI through PWM\_CCTL.DISHI =0, PWM\_ACTL.DISLO through PWM\_CCTL.DISLO =0)
- Use conventional PWM, disable crossover ( PWM\_ACTL.XOVR through PWM\_CCTL.XOVR =0)
- Use symmetrical pulse position on all outputs ( PWM\_ACTL.PULSEMODEHI through PWM\_CCTL.PULSEMODEHI =0, PWM\_ACTL.PULSEMODELO through PWM\_CCTL.PULSEMODELO =0)
- Set an initial duty-cycle of 50% ( PWM\_AH0 through PWM\_CH0 =0x0)

## PWM Enable for Motor Control

The processor must do the following programming to enable the PWM before starting the motor.

1. Start the PWM timer TMR0 using the PWM\_CTL |=0x1 bitwise operation on the PWM\_CTL register. ADDITIONAL INFORMATION: This operation has the same effect as setting the PWM\_CTL.GLOBEN bit =1.
2. Enable six PWM outputs using the following bitwise operations on the PWM\_ACTL through PWM\_CCTL registers.

```
PWM_ACTL|=0x3 PWM_BCTL|=0x3 PWM_CCTL|=0x3
```

ADDITIONAL INFORMATION: These operations have the same effect as enabling high and low-side channel outputs by setting the PWM\_ACTL.DISHI through PWM\_CCTL.DISHI bits =1 and the PWM\_ACTL.DISLO through PWM\_CCTL.DISLO bits =1.

3. Enable the PWM TRIP0 interrupt using the PWM\_ILAT |= 0x1 bitwise operation on the PWM\_ILAT register.

ADDITIONAL INFORMATION: This operation has the same effect as setting the PWM\_ILAT.TRIP0 bit =1

## PWM Response to Sync Interrupt for Motor Control

When the PWM sync interrupt occurs, the processor could need to update to the PWM duty cycle with a value calculated by the motor control algorithm. This application uses symmetric pulses position and uses dependent high and low-side output. So, the PWM updates only one register for each phase.

1. Write the new duty cycle value (calculated by motor control algorithm) to the timer when the sync interrupt occurs.

The following bitwise operations on the PWM\_AH0 through PWM\_CH0 registers accomplish this task:

```
PWM_AH0 = Duty_A_mc_algorithm_current_value PWM_BH0 = Duty_B_mc_algorithm_current_value PWM_CH0 = Duty_C_mc_algorithm_current_value
```

## PWM Disable (and Stop the Motor) for Motor Control

The processor must program the PWM as follows to stop the motor, disable the PWM, and disable PWM interrupts. These actions place the PWM and system in a safe, passive state.

1. Disable the PWM timer using the PWM\_CTL &amp;= 0xFFFFFFFE bitwise operation on the PWM\_CTL register.

ADDITIONAL INFORMATION: This operation has the same effect as clearing the PWM\_CTL.GLOBEN bit =0.

2. Disable all PWM outputs using the following bitwise operations on the PWM\_ACTL through PWM\_CCTL registers.

```
PWM_ACTL &= 0xFFFFFFFFC PWM_BCTL &= 0xFFFFFFFFC PWM_CCTL &= 0xFFFFFFFFC
```

These operations disable PWM outputs where the PWM\_ACTL.DISHI through PWM\_CCTL.DISHI bits =1 and the PWM\_ACTL.DISLO through PWM\_CCTL.DISLO bits =0)

3. Set the PWM duty-cycle to 50% using the following bitwise operations on the PWM\_AH0 through PWM\_CH0 registers.

```
PWM_AH0 =0x0 PWM_BH0 =0x0 PWM_CH0 =0x0
```

These operations have the same effect as clearing the PWM\_AH0.DUTY through PWM\_CH0.DUTY bit =0.

4. Disable the PWM TRIP0 interrupt request using the PWM\_ILAT &amp;= 0xFFFFFFFE bitwise operation on the PWM\_ILAT register.

ADDITIONAL INFORMATION: This operation has the same effect as clearing the PWM TRIP0 interrupt request PWM\_ILAT.TRIP0 =0.

## ADSP-SC58x PWM Register Descriptions

Pulse-Width Modulator (PWM) contains the following registers.

Table 19-5: ADSP-SC58x PWM Register List

| Name     | Description                |
|----------|----------------------------|
| PWM_ACTL | Channel A Control Register |

Table 19-5: ADSP-SC58x PWM Register List (Continued)

| Name         | Description                                               |
|--------------|-----------------------------------------------------------|
| PWM_AH0      | Channel A-High Duty-0 Register                            |
| PWM_AH0_HP   | Channel A-High Heightened-Precision Duty-0 Register       |
| PWM_AH1      | Channel A-High Duty-1 Register                            |
| PWM_AH1_HP   | Channel A-High Heightened-Precision Duty-1 Register       |
| PWM_AH_DUTY0 | Channel A-High Full Duty0 Register                        |
| PWM_AH_DUTY1 | Channel A-High Full Duty1 Register                        |
| PWM_AL0      | Channel A-Low Duty-0 Register                             |
| PWM_AL0_HP   | Channel A-Low Heightened-Precision Duty-0 Register        |
| PWM_AL1      | Channel A-Low Duty-1 Register                             |
| PWM_AL1_HP   | Channel A-Low Heightened-Precision Duty-1 Register        |
| PWM_AL_DUTY0 | Channel A-Low Full Duty0 Register                         |
| PWM_AL_DUTY1 | Channel A-Low Full Duty1 Register                         |
| PWM_BCTL     | Channel B Control Register                                |
| PWM_BH0      | Channel B-High Duty-0 Register                            |
| PWM_BH0_HP   | Channel B-High Heightened-Precision Duty-0 Register       |
| PWM_BH1      | Channel B-High Duty-1 Register                            |
| PWM_BH1_HP   | Channel B-High Heightened-Precision Duty-1 Register       |
| PWM_BH_DUTY0 | Channel B-High Full Duty0 Register                        |
| PWM_BH_DUTY1 | Channel B-High Full Duty1 Register                        |
| PWM_BL0      | Channel B-Low Duty-0 Register                             |
| PWM_BL0_HP   | Channel B-Low Heightened-Precision Duty-0 Register        |
| PWM_BL1      | Channel B-Low Duty-1 Register                             |
| PWM_BL1_HP   | Channel B-Low Heightened-Precision Duty-1 Register        |
| PWM_BL_DUTY0 | Channel B-Low Full Duty0 Register                         |
| PWM_BL_DUTY1 | Channel B-Low Full Duty1 Register                         |
| PWM_CCTL     | Channel C Control Register                                |
| PWM_CH0      | Channel C-High Pulse Duty Register 0                      |
| PWM_CH0_HP   | Channel C-High Pulse Heightened-Precision Duty Register 0 |
| PWM_CH1      | Channel C-High Pulse Duty Register 1                      |
| PWM_CH1_HP   | Channel C-High Pulse Heightened-Precision Duty Register 1 |
| PWM_CHANCFG  | Channel Configuration Register                            |

Table 19-5: ADSP-SC58x PWM Register List (Continued)

| Name         | Description                                               |
|--------------|-----------------------------------------------------------|
| PWM_CHA_DT   | Channel A Dead-time Register                              |
| PWM_CHB_DT   | Channel B Dead-time Register                              |
| PWM_CHC_DT   | Channel C Dead-time Register                              |
| PWM_CHD_DT   | Channel DDead-time Register                               |
| PWM_CHOPCFG  | Chop Configuration Register                               |
| PWM_CH_DUTY0 | Channel C-High Full Duty0 Register                        |
| PWM_CH_DUTY1 | Channel C-High Full Duty1 Register                        |
| PWM_CL0      | Channel C-Low Pulse Duty Register 0                       |
| PWM_CL0_HP   | Channel C-Low Pulse Duty Register 1                       |
| PWM_CL1      | Channel C-Low Duty-1 Register                             |
| PWM_CL1_HP   | Channel C-Low Heightened-Precision Duty-1 Register        |
| PWM_CL_DUTY0 | Channel C-Low Full Duty0 Register                         |
| PWM_CL_DUTY1 | Channel C-Low Full Duty1 Register                         |
| PWM_CTL      | Control Register                                          |
| PWM_DCTL     | Channel DControl Register                                 |
| PWM_DH0      | Channel D-High Duty-0 Register                            |
| PWM_DH0_HP   | Channel D-High Pulse Heightened-Precision Duty Register 0 |
| PWM_DH1      | Channel D-High Pulse Duty Register 1                      |
| PWM_DH1_HP   | Channel DHigh Pulse Heightened-Precision Duty Register 1  |
| PWM_DH_DUTY0 | Channel D-High Full Duty0 Register                        |
| PWM_DH_DUTY1 | Channel D-High Full Duty1 Register                        |
| PWM_DL0      | Channel D-Low Pulse Duty Register 0                       |
| PWM_DL0_HP   | Channel D-Low Heightened-Precision Duty-0 Register        |
| PWM_DL1      | Channel D-Low Pulse Duty Register 1                       |
| PWM_DL1_HP   | Channel D-Low Heightened-Precision Duty-1 Register        |
| PWM_DLYA     | Channel A Delay Register                                  |
| PWM_DLYB     | Channel B Delay Register                                  |
| PWM_DLYC     | Channel C Delay Register                                  |
| PWM_DLYD     | Channel DDelay Register                                   |
| PWM_DL_DUTY0 | Channel D-Low Full Duty0 Register                         |
| PWM_DL_DUTY1 | Channel D-Low Full Duty1 Register                         |

Table 19-5: ADSP-SC58x PWM Register List (Continued)

| Name         | Description                 |
|--------------|-----------------------------|
| PWM_ILAT     | Interrupt Latch Register    |
| PWM_IMSK     | Interrupt Mask Register     |
| PWM_STAT     | Status Register             |
| PWM_SYNC_WID | Sync Pulse Width Register   |
| PWM_TM0      | Timer 0 Period Register     |
| PWM_TM1      | Timer 1 Period Register     |
| PWM_TM2      | Timer 2 Period Register     |
| PWM_TM3      | Timer 3 Period Register     |
| PWM_TM4      | Timer 4 Period Register     |
| PWM_TRIPCFG  | Trip Configuration Register |

## Channel A Control Register

The PWM\_ACTL register selects the low and high side output pulse mode, enables low and high side output, and enables low/high side output crossover.

Figure 19-28: PWM\_ACTL Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000027_8e7b7d69aa14ee6901ca09880ef16ca3ef055033b0385e4cbe7bbe8b7bb53ac0.png)

Table 19-6: PWM\_ACTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:10 (R/W)        | PULSEMODELO | Low Side Output Pulse Position. The PWM_ACTL.PULSEMODELO bits select the pulse position for Channel A low side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_AL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_AL0 and PWM_AL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_AL0 and PWM_AL1 ). | Low Side Output Pulse Position. The PWM_ACTL.PULSEMODELO bits select the pulse position for Channel A low side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_AL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_AL0 and PWM_AL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_AL0 and PWM_AL1 ). |
|                    |             | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Symmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                    |             | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Asymmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |             | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Left Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |             | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Right Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 19-6: PWM\_ACTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:8 (R/W)          | PULSEMODEHI | High Side Output Pulse Position. The PWM_ACTL.PULSEMODEHI bits select the pulse position for Channel A high side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_AH0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_AH0 and PWM_AH1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_AH0 and PWM_AH1 ). 0 Symmetrical |
| 2 (R/W)            | XOVR        | 3 Right Half high-low Crossover Enable. The PWM_ACTL.XOVR bit enables crossover between the channels high and low side outputs. When enabled, this bit directs the PWMto send the low-side output through the high-side output pin and the high-side output through the low side output pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 1 (R/W)            | DISLO       | 1 Enable Crossover Channel Low Side Output Disable. The PWM_ACTL.DISLO bit enables the channels low side output. 0 Enable Low Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | DISHI       | 1 Disable Low Side Output Channel High Side Output Disable. The PWM_ACTL.DISHI bit enables the channels high side output.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Channel A-High Duty-0 Register

The PWM\_AH0 and PWM\_AH1 registers determine the width for the high side output pulses. The values in these registers select the assertion count (in terms of t CK) of the high side output pulses for the channel A duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_ACTL.PULSEMODEHI bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_AH0 register to determine the assertion and deassertion count for the high side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel A high pulse output for a count less than PWM\_AH0 and deasserts this output for a count greater than PWM\_AH1 .

The value range for the PWM\_AH0 and PWM\_AH1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHA\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_AH0 and PWM\_AH1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHA\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHA\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHA\_DT (two's complement) and -PWM\_TM0 /2 -PWM\_CHA\_DT .

Note that using values in the PWM\_AH0 or PWM\_AH1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-29: PWM\_AH0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000028_3ed5343d2813652a08d2c93a7af64280bdade40553b1c6dc2ac9db790b5e5faf.png)

Table 19-7: PWM\_AH0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count.                                                                 |
| (R/W)              |            | The PWM_AH0.DUTY bits select the duty cycle asserted count for Channel A high side output. |

## Channel A-High Heightened-Precision Duty-0 Register

The PWM\_AH0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_AH0 register, allows programs to specify fractional duty cycles.The PWM\_AH0\_HP register and the PWM\_AH0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_AH0\_HP and the PWM\_AH0 registers are also present in the single full duty register (if available).

Figure 19-30: PWM\_AH0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000029_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-8: PWM\_AH0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration          |
|--------------------|------------|----------------------------------|
| 7:6                | ENHDIV     | Enhanced Precision Divider Bits. |
| (R/W)              |            |                                  |

## Channel A-High Duty-1 Register

The PWM\_AH0 and PWM\_AH1 registers determine the width for the high side output pulses. For more information, see the PWM\_AH0 register description.

Figure 19-31: PWM\_AH1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000030_4e56b37c2f4cd8f0b62f00e4f95b1b6d7a9ed72aeed7d23f1eceb00c48102f6a.png)

Table 19-9: PWM\_AH1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. The PWM_AH1.DUTY bits select the duty cycle de-asserted count for Channel A high side output. |
| (R/W)              |            |                                                                                                                             |

## Channel A-High Heightened-Precision Duty-1 Register

The PWM\_AH1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_AH0 register, allows programs to specify fractional duty cycles.The PWM\_AH1\_HP register and the PWM\_AH1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_AH1\_HP and the PWM\_AH1 registers are also present in the single full duty register (if available).

Figure 19-32: PWM\_AH1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000031_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-10: PWM\_AH1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AH1_HP.ENHDIV bits provide fractional duty cycles for Channel A high side output. |

## Channel A-High Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_AH\_DUTY0 register contains the PWM\_AH\_DUTY0.DUTY bit field from the PWM\_AH0 register and the PWM\_AH\_DUTY0.ENHDIV bit field from the PWM\_AH0\_HP register.

Note that the PWM\_AH\_DUTY0 register reads the PWM\_AH0 and the PWM\_AH0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-33: PWM\_AH\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000032_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-11: PWM\_AH\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_AH_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_AH_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AH_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel A-High Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_AH\_DUTY1 register contains the PWM\_AH\_DUTY1.DUTY bit field from the PWM\_AH1 register and the PWM\_AH\_DUTY1.ENHDIV bit field from the PWM\_AH1\_HP register.

Note that the PWM\_AH\_DUTY1 register reads the PWM\_AH1 and the PWM\_AH1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-34: PWM\_AH\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000033_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-12: PWM\_AH\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_AH_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_AH_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AH_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel A-Low Duty-0 Register

The PWM\_AL0 and PWM\_AL1 registers determine the width for the low side output pulses. The values in these registers select the assertion count (in terms of t CK) of the low side output pulses for the channel A duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_ACTL.PULSEMODELO bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_AL0 register to determine the assertion and deassertion count for the low side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel A low pulse output for count less than PWM\_AL0 and deasserts this output for count greater than PWM\_AL1 .

The value range for the PWM\_AL0 and PWM\_AL1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHA\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_AL0 and PWM\_AL1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHA\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHA\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHA\_DT (two's complement) and -PWM\_TM0 /2 -PWM\_CHA\_DT .

Note that using values in the PWM\_AL0 or PWM\_AL1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-35: PWM\_AL0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000034_92d4cad0ffefd907ba19fefd12ab93f53c021121f436e59d098a5a48238b3955.png)

Table 19-13: PWM\_AL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count. The PWM_AL0.DUTY bits select the duty cycle asserted count for Channel A low |
| (R/W)              |            | side output.                                                                                            |

## Channel A-Low Heightened-Precision Duty-0 Register

The PWM\_AL0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_AL0 register, allows programs to specify fractional duty cycles.The PWM\_AL0\_HP register and the PWM\_AL0 register work together in a Q15.8 signed two's complement fixed-point format. Note that the bit fields in the PWM\_AL0\_HP and the PWM\_AL0 registers are also present in the single full duty register (if available).

Figure 19-36: PWM\_AL0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000035_8a2e0cb350fe2af6cc032b6532452d5f89bb1a5dacf7dd935288e1958f7aaec4.png)

Table 19-14: PWM\_AL0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AL0_HP.ENHDIV bits provide fractional duty cycles for Channel A low side output. |

## Channel A-Low Duty-1 Register

The PWM\_AL0 and PWM\_AL1 registers determine the width for the low side output pulses. For more information, see the PWM\_AL0 register description.

Figure 19-37: PWM\_AL1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000036_4e56b37c2f4cd8f0b62f00e4f95b1b6d7a9ed72aeed7d23f1eceb00c48102f6a.png)

Table 19-15: PWM\_AL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. The PWM_AL1.DUTY bits select the duty cycle de-asserted count for Channel A low side output. |
| (R/W)              |            |                                                                                                                            |

## Channel A-Low Heightened-Precision Duty-1 Register

The PWM\_AL1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_AL1 register, allows programs to specify fractional duty cycles.The PWM\_AL1\_HP register and the PWM\_AL1 register work together in a Q15.8 signed two's complement fixed-point format. Note that the bit fields in the PWM\_AL1\_HP and the PWM\_AL1 registers are also present in the single full duty register (if available).

Figure 19-38: PWM\_AL1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000037_8a2e0cb350fe2af6cc032b6532452d5f89bb1a5dacf7dd935288e1958f7aaec4.png)

Table 19-16: PWM\_AL1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AL1_HP.ENHDIV bits provide fractional duty cycles for Channel A low side output. |

## Channel A-Low Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_AL\_DUTY0 register contains the PWM\_AL\_DUTY0.DUTY bit field from the PWM\_AL0 register and the PWM\_AL\_DUTY0.ENHDIV bit field from the PWM\_AL0\_HP register.

Note that the PWM\_AL\_DUTY0 register reads the PWM\_AL0 and the PWM\_AL0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-39: PWM\_AL\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000038_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-17: PWM\_AL\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_AL_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_AL_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AL_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel A-Low Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_AL\_DUTY1 register contains the PWM\_AL\_DUTY1.DUTY bit field from the PWM\_AL1 register and the PWM\_AL\_DUTY1.ENHDIV bit field from the PWM\_AH0\_HP register.

Note that the PWM\_AL\_DUTY1 register reads the PWM\_AL1 and the PWM\_AL1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-40: PWM\_AL\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000039_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-18: PWM\_AL\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_AL_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_AL_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_AL_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel B Control Register

The PWM\_BCTL register selects the low and high side output pulse mode, enables low and high side output, and enables low/high side output crossover.

Figure 19-41: PWM\_BCTL Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000040_8e7b7d69aa14ee6901ca09880ef16ca3ef055033b0385e4cbe7bbe8b7bb53ac0.png)

Table 19-19: PWM\_BCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:10 (R/W)        | PULSEMODELO | Low Side Output Pulse Position. The PWM_BCTL.PULSEMODELO bits select the pulse position for Channel B low side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_BL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_BL0 and PWM_BL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_BL0 and PWM_BL1 ). | Low Side Output Pulse Position. The PWM_BCTL.PULSEMODELO bits select the pulse position for Channel B low side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_BL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_BL0 and PWM_BL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_BL0 and PWM_BL1 ). |
|                    |             | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Symmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                    |             | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Asymmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |             | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Left Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |             | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Right Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 19-19: PWM\_BCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:8 (R/W)          | PULSEMODEHI | High Side Output Pulse Position. The PWM_BCTL.PULSEMODEHI bits select the pulse position for Channel B high side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_BH0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_BH0 and PWM_BH1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_BH0 and PWM_BH1 ). 0 Symmetrical |
| 2 (R/W)            | XOVR        | 3 Right Half high-low Crossover Enable. The PWM_BCTL.XOVR bit enables crossover between the channels high and low side outputs. When enabled, this bit directs the PWMto send the low-side output through the high-side output pin and the high-side output through the low side output pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 1 (R/W)            | DISLO       | 1 Enable Crossover Channel Low Side Output Disable. The PWM_BCTL.DISLO bit enables the channels low side output. 0 Enable Low Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | DISHI       | 1 Disable Low Side Output Channel High Side Output Disable. The                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |             | PWM_BCTL.DISHI bit enables the channels high side output. 0 Enable High Side Output 1 Disable High Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## Channel B-High Duty-0 Register

The PWM\_BH0 and PWM\_BH1 registers determine the width for the high side output pulses. The values in these registers select the assertion count (in terms of t CK) of the high side output pulses for the channel B duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_BCTL.PULSEMODEHI bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_BH0 register to determine the assertion and deassertion count for the high side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel B high pulse output for count less than PWM\_BH0 and deasserts this output for count greater than PWM\_BH1 .

The value range for the PWM\_BH0 and PWM\_BH1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHB\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_BH0 and PWM\_BH1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHB\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHB\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHB\_DT (two's complement) and -PWM\_TM0 /2 -PWM\_CHB\_DT .

Note that using values in the PWM\_BH0 or PWM\_BH1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-42: PWM\_BH0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000041_619773fe83acd810b0368a63304524846627e1307a374110d2832e8f86cc3659.png)

Table 19-20: PWM\_BH0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count. |
| (R/W)              |            |                            |

## Channel B-High Heightened-Precision Duty-0 Register

The PWM\_BH0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_BH0 register, allows programs to specify fractional duty cycles. The PWM\_BH0\_HP register and the PWM\_BH0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_BH0\_HP and the PWM\_BH0 registers are also present in the single full duty register (if available).

Figure 19-43: PWM\_BH0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000042_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-21: PWM\_BH0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BH0_HP.ENHDIV bits provide fractional duty cycles for Channel B high side output. |

## Channel B-High Duty-1 Register

The PWM\_BH0 and PWM\_BH1 registers determine the width for the high side output pulses. For more information, see the PWM\_BH0 register description.

Figure 19-44: PWM\_BH1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000043_99b70ea8f8fb82649d98d245412e7d64cfc16f375b21f8ab0edcc2b2001e12d3.png)

Table 19-22: PWM\_BH1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. |
| (R/W)              |            |                               |

## Channel B-High Heightened-Precision Duty-1 Register

The PWM\_BH1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_BH1 register, allows programs to specify fractional duty cycles. The PWM\_BH1\_HP register and the PWM\_BH1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_BH1\_HP and the PWM\_BH1 registers are also present in the single full duty register (if available).

Figure 19-45: PWM\_BH1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000044_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-23: PWM\_BH1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BH1_HP.ENHDIV bits provide fractional duty cycles for Channel B high side output. |

## Channel B-High Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_BH\_DUTY0 register contains the PWM\_BH\_DUTY0.DUTY bit field from the PWM\_BH0 register and the PWM\_BH\_DUTY0.ENHDIV bit field from the PWM\_BH0\_HP register.

Note that the PWM\_BH\_DUTY0 register reads the PWM\_BH0 and the PWM\_BH0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-46: PWM\_BH\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000045_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-24: PWM\_BH\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_BH_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_BH_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BH_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel B-High Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_BH\_DUTY1 register contains the PWM\_BH\_DUTY1.DUTY bit field from the PWM\_BH1 register and the PWM\_BH\_DUTY1.ENHDIV bit field from the PWM\_BH1\_HP register.

Note that the PWM\_BH\_DUTY1 register reads the PWM\_BH1 and the PWM\_BH1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-47: PWM\_BH\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000046_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-25: PWM\_BH\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_BH_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_BH_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BH_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel B-Low Duty-0 Register

The PWM\_BL0 and PWM\_BL1 registers determine the width for the low side output pulses. The values in these registers select the assertion count (in terms of t CK) of the low side output pulses for the channel B duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_BCTL.PULSEMODELO bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_BL0 register to determine the assertion and deassertion count for the low side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel B low pulse output for count less than PWM\_BL0 and deasserts this output for count greater than PWM\_BL1 .

The value range for the PWM\_BL0 and PWM\_BL1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHB\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_BL0 and PWM\_BL1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHB\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHB\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHB\_DT (two's complement) and -PWM\_TM0 /2 -PWM\_CHB\_DT .

Note that using values in the PWM\_BL0 or PWM\_BL1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-48: PWM\_BL0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000047_92d4cad0ffefd907ba19fefd12ab93f53c021121f436e59d098a5a48238b3955.png)

Table 19-26: PWM\_BL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count. The PWM_BL0.DUTY bits select the duty cycle asserted count for Channel B low |
| (R/W)              |            | side output.                                                                                            |

## Channel B-Low Heightened-Precision Duty-0 Register

The PWM\_BL0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_BL0 register, allows programs to specify fractional duty cycles. The PWM\_BL0\_HP register and the PWM\_BL0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_BL0\_HP and the PWM\_BL0 registers are also present in the single full duty register ( PWM\_BL\_DUTY0 ).

Figure 19-49: PWM\_BL0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000048_5d065cbb4d4774bcd8c8e2afe3115bed4a28df3f2d1a42414082bcdc3a629c49.png)

Table 19-27: PWM\_BL0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BL0_HP.ENHDIV bits provide fractional duty cycles for Channel B low side output. |

## Channel B-Low Duty-1 Register

The PWM\_BL0 and PWM\_BL1 registers determine the width for the low side output pulses. For more information, see the PWM\_BL0 register description.

Figure 19-50: PWM\_BL1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000049_4e56b37c2f4cd8f0b62f00e4f95b1b6d7a9ed72aeed7d23f1eceb00c48102f6a.png)

Table 19-28: PWM\_BL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | DUTY       | Duty Cycle De-Asserted Count. The PWM_BL1.DUTY bits select the duty cycle de-asserted count for Channel B low side output. |

## Channel B-Low Heightened-Precision Duty-1 Register

The PWM\_BL1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_BL1 register, allows programs to specify fractional duty cycles. The PWM\_BL1\_HP register and the PWM\_BL1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_BL1\_HP and the PWM\_BL1 registers are also present in the single full duty register (if available).

Figure 19-51: PWM\_BL1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000050_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-29: PWM\_BL1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BL1_HP.ENHDIV bits provide fractional duty cycles for Channel B low side output. |

## Channel B-Low Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_BL\_DUTY0 register contains the PWM\_BL\_DUTY0.DUTY bit field from the PWM\_BL0 register and the PWM\_BL\_DUTY0.ENHDIV bit field from the PWM\_BL0\_HP register.

Note that the PWM\_BL\_DUTY0 register reads the PWM\_BL0 and the PWM\_BL0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-52: PWM\_BL\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000051_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-30: PWM\_BL\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_BL_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_BL_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BL_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel B-Low Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_BL\_DUTY1 register contains the PWM\_BL\_DUTY1.DUTY bit field from the PWM\_BL1 register and the PWM\_BL\_DUTY1.ENHDIV bit field from the PWM\_BL1\_HP register.

Note that the PWM\_BL\_DUTY1 register reads the PWM\_BL1 and the PWM\_BL1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-53: PWM\_BL\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000052_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-31: PWM\_BL\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_BL_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_BL_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_BL_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel C Control Register

The PWM\_CCTL register selects the low and high side output pulse mode, enables low and high side output, and enables low/high side output crossover.

Figure 19-54: PWM\_CCTL Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000053_8e7b7d69aa14ee6901ca09880ef16ca3ef055033b0385e4cbe7bbe8b7bb53ac0.png)

Table 19-32: PWM\_CCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:10 (R/W)        | PULSEMODELO | Low Side Output Pulse Position. The PWM_CCTL.PULSEMODELO bits select the pulse position for Channel C low side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_CL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_CL0 and PWM_CL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_CL0 and PWM_CL1 ). | Low Side Output Pulse Position. The PWM_CCTL.PULSEMODELO bits select the pulse position for Channel C low side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_CL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_CL0 and PWM_CL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_CL0 and PWM_CL1 ). |
|                    |             | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Symmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                    |             | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Asymmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |             | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Left Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |             | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Right Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 19-32: PWM\_CCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:8 (R/W)          | PULSEMODEHI | High Side Output Pulse Position. The PWM_CCTL.PULSEMODEHI bits select the pulse position for Channel C high side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_CH0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_CH0 and PWM_CH1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_CH0 and PWM_CH1 ). 0 Symmetrical |
| 2 (R/W)            | XOVR        | 3 Right Half high-low Crossover Enable. The PWM_CCTL.XOVR bit enables crossover between the channels high and low side outputs. When enabled, this bit directs the PWMto send the low-side output through the high-side output pin and the high-side output through the low side output pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 1 (R/W)            | DISLO       | 1 Enable Crossover Channel Low Side Output Disable. The PWM_CCTL.DISLO bit enables the channels low side output. 0 Enable Low Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | DISHI       | 1 Disable Low Side Output Channel High Side Output Disable. The                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |             | PWM_CCTL.DISHI bit enables the channels high side output. 0 Enable High Side Output 1 Disable High Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## Channel C-High Pulse Duty Register 0

The PWM\_CH0 and PWM\_CH1 registers determine the width for the high side output pulses. The values in these registers select the assertion count (in terms of t CK) of the high side output pulses for the channel C duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_CCTL.PULSEMODEHI bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_CH0 register to determine the assertion and deassertion count for the high side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel C high pulse output for count less than PWM\_CH0 and deasserts this output for count greater than PWM\_CH1 .

The value range for the PWM\_CH0 and PWM\_CH1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHC\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_CH0 and PWM\_CH1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHC\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHC\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHC\_DT (two's complement) and -PWM\_TM0 /2 -PWM\_CHC\_DT .

Note that using values in the PWM\_CH0 or PWM\_CH1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-55: PWM\_CH0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000054_3ed5343d2813652a08d2c93a7af64280bdade40553b1c6dc2ac9db790b5e5faf.png)

Table 19-33: PWM\_CH0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count.                                                                 |
| (R/W)              |            | The PWM_CH0.DUTY bits select the duty cycle asserted count for Channel C high side output. |

## Channel C-High Pulse Heightened-Precision Duty Register 0

The PWM\_CH0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_CH0 register, allows programs to specify fractional duty cycles. The PWM\_CH0\_HP register and the PWM\_CH0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_CH0\_HP and the PWM\_CH0 registers are also present in the single full duty register (if available).

Figure 19-56: PWM\_CH0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000055_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-34: PWM\_CH0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CH0_HP.ENHDIV bits provide fractional duty cycles for Channel C high side output. |

## Channel C-High Pulse Duty Register 1

The PWM\_CH0 and PWM\_CH1 registers determine the width for the high side output pulses. For more information, see the PWM\_CH0 register description.

Figure 19-57: PWM\_CH1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000056_99b70ea8f8fb82649d98d245412e7d64cfc16f375b21f8ab0edcc2b2001e12d3.png)

Table 19-35: PWM\_CH1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. The PWM_CH1.DUTY bits select the duty cycle de-asserted count for Channel C high side output. |
| (R/W)              |            |                                                                                                                             |

## Channel C-High Pulse Heightened-Precision Duty Register 1

The PWM\_CH1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_CH1 register, allows programs to specify fractional duty cycles. The PWM\_CH1\_HP register and the PWM\_CH1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_CH1\_HP and the PWM\_CH1 registers are also present in the single full duty register (if available).

Figure 19-58: PWM\_CH1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000057_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-36: PWM\_CH1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CH1_HP.ENHDIV bits provide fractional duty cycles for Channel C high side output. |

## Channel Configuration Register

The PWM\_CHANCFG register configures Channel A, B, C, and D reference timer selection, high and low side output features, and enables high frequency chopping operation. Do not change the value of any bits in the PWM register while the PWM is enabled ( PWM\_CTL.GLOBEN =1).

Figure 19-59: PWM\_CHANCFG Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000058_0e3bd41479a5faf82272e77064e04926ab20a9d53f6086474bd963302492de39.png)

Table 19-37: PWM\_CHANCFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | ENCHOPDL   | Channel DGate Chopping Enable Low Side. The PWM_CHANCFG.ENCHOPDL bit enables mixing of the Channel Dlow side out- put signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register. |
| 30 (R/W)           | ENCHOPDL   | 0 Disable Chopping Channel DLow Side                                                                                                                                                                                    |
| 29 (R/W)           | POLDL      | Channel Dlow side Polarity. The PWM_CHANCFG.POLDL bit selects the Channel Dlow side output polarity (ac- tive-high or active-low).                                                                                      |
| 29 (R/W)           | POLDL      | 0 Active Low                                                                                                                                                                                                            |
| 29 (R/W)           | POLDL      | 1 Active High                                                                                                                                                                                                           |
| 28 (R/W)           | ENHPDH     | Channel Dheightened-precision enable for high side Output. The PWM_CHANCFG.ENHPDH bit enables heightened-precision Channel Dhigh side output.                                                                           |
| 28 (R/W)           | ENHPDH     | 0 Disable HP Output Channel DHigh                                                                                                                                                                                       |
| 28 (R/W)           | ENHPDH     | 1 Enable HP Output Channel DHigh                                                                                                                                                                                        |
| 27 (R/W)           | ENCHOPDH   | Channel DGate Chopping Enable High Side. The PWM_CHANCFG.ENCHOPDH bit enables mixing of the Channel Dhigh side output signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register. |
| 27 (R/W)           | ENCHOPDH   | 0 Disable Chopping Channel DHigh Side                                                                                                                                                                                   |
| 27 (R/W)           | ENCHOPDH   | 1 Enable Chopping Channel DHigh Side                                                                                                                                                                                    |
| 26 (R/W)           | POLDH      | Channel DHigh side Polarity. The PWM_CHANCFG.POLDH bit selects the Channel Dhigh side output polarity (ac- tive-high or active-low).                                                                                    |
| 26 (R/W)           | POLDH      | 0 Active Low                                                                                                                                                                                                            |
| 26 (R/W)           | POLDH      | 1 Active High                                                                                                                                                                                                           |

Table 19-37: PWM\_CHANCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | MODELSD    | Channel DMode of low Side Output. The PWM_CHANCFG.MODELSD bit selects whether the low side output waveform is based on independent controls or whether the low side output depends on the high side output controls. When PWM_CHANCFG.MODELSD =0, the low side output is an inverted form of the high side output, which is generated using the PWM_BH0 and PWM_BH1 registers for pulse width, using the PWM_BCTL.PULSEMODEHI bits for pulse positioning, and PWM_CHANCFG.POLBH bits for polarity. |
| 24 (R/W)           | REFTMRD    | Channel DTimer Reference. The PWM_CHANCFG.REFTMRD bit selects whether the PWMuses PWMTMR1 or PWMTMR0 as the reference timer for Channel Doperation.                                                                                                                                                                                                                                                                                                                                                |
| 22 (R/W)           | ENCHOPCL   | 1 PWMTMR1 is Channel Dreference Channel C Gate Chopping Enable Low Side. The PWM_CHANCFG.ENCHOPCL bit enables mixing of the Channel C low side out- put signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register.                                                                                                                                                                                                                                          |
| 21 (R/W)           | POLCL      | 0 Disable Chopping Channel C Low Side 1 Enable Chopping Channel C Low Side Channel C low side Polarity.                                                                                                                                                                                                                                                                                                                                                                                            |
| 20 (R/W)           |            | The PWM_CHANCFG.POLCL bit selects the Channel C low side output polarity (ac- tive-high or active-low). 0 Active Low 1 Active High                                                                                                                                                                                                                                                                                                                                                                 |
|                    | ENHPCH     | Channel C heightened-precision enable for high side Output. The PWM_CHANCFG.ENHPCH bit enables heightened-precision Channel C high side output.                                                                                                                                                                                                                                                                                                                                                    |

Table 19-37: PWM\_CHANCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | ENCHOPCH   | Channel C Gate Chopping Enable High Side. The PWM_CHANCFG.ENCHOPCH bit enables mixing of the Channel C high side output signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register.                                                                                                                                                                                                                                                                                         |
| 19 (R/W)           | ENCHOPCH   | 0 Disable Chopping Channel C High Side                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 18 (R/W)           | POLCH      | Channel C High side Polarity. The PWM_CHANCFG.POLCH bit selects the Channel C high side output polarity (ac- tive-high or active-low).                                                                                                                                                                                                                                                                                                                                                                            |
| 18 (R/W)           | POLCH      | 0 Active Low                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 17 (R/W)           | MODELSC    | 1 Active High Channel C Mode of low Side Output. The PWM_CHANCFG.MODELSC bit selects whether the low side output waveform is based on independent controls or whether the low side output depends on the high side output controls. When PWM_CHANCFG.MODELSC =0, the low side output is an inverted form of the high side output, which is generated using the PWM_BH0 and PWM_BH1 registers for pulse width, using the PWM_BCTL.PULSEMODEHI bits for pulse positioning, and PWM_CHANCFG.POLBH bits for polarity. |
| 17 (R/W)           | MODELSC    | 0 Invert of high output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 16 (R/W)           | REFTMRC    | Channel C Timer Reference. The PWM_CHANCFG.REFTMRC bit selects whether the PWMuses PWMTMR1 or PWMTMR0 as the reference timer for Channel C operation.                                                                                                                                                                                                                                                                                                                                                             |
| 16 (R/W)           | REFTMRC    | 0 PWMTMR0 is Channel C reference                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 14 (R/W)           | ENCHOPBL   | Channel B Gate Chopping Enable Low Side. The PWM_CHANCFG.ENCHOPBL bit enables mixing of the Channel B low side out- put signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register.                                                                                                                                                                                                                                                                                         |
| 14 (R/W)           | ENCHOPBL   | 0 Disable Chopping Channel B Low Side                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 14 (R/W)           | ENCHOPBL   | 1 Enable Chopping Channel B Low Side                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 19-37: PWM\_CHANCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | POLBL      | Channel B low side Polarity. The PWM_CHANCFG.POLBL bit selects the Channel B low side output polarity (ac- tive-high or active-low).                                                                                                                                                                                                                                                                                                                                                                |
| 13 (R/W)           | POLBL      | 0 Active Low                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 13 (R/W)           | POLBL      | 1 Active High                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 12 (R/W)           | ENHPBH     | Channel B heightened-precision enable for high side Output. The PWM_CHANCFG.ENHPBH bit enables heightened-precision Channel B high side output.                                                                                                                                                                                                                                                                                                                                                     |
| 12 (R/W)           | ENHPBH     | 0 Disable HP Output Channel B High                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | ENHPBH     | 1 Enable HP Output Channel B High                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 11 (R/W)           | ENCHOPBH   | Channel B Gate Chopping Enable High Side. The PWM_CHANCFG.ENCHOPBH bit enables mixing of the Channel B high side output signals with a high-frequency chopping signal, which is configured with the                                                                                                                                                                                                                                                                                                 |
| 11 (R/W)           | ENCHOPBH   | 0 Disable Chopping Channel B High Side                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 11 (R/W)           | ENCHOPBH   | 1 Enable Chopping Channel B High Side                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 10 (R/W)           | POLBH      | Channel B High side Polarity. The PWM_CHANCFG.POLBH bit selects the Channel B high side output polarity (ac- tive-high or active-low).                                                                                                                                                                                                                                                                                                                                                              |
| 10 (R/W)           | POLBH      | 0 Active Low                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W)           | POLBH      | 1 Active High                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 9 (R/W)            | MODELSB    | Channel B Mode of low Side Output. The PWM_CHANCFG.MODELSB bit selects whether the low side output waveform is based on independent controls or whether the low side output depends on the high side output controls. When PWM_CHANCFG.MODELSB =0, the low side output is an inverted form of the high side output, which is generated using the PWM_BH0 and PWM_BH1 registers for pulse width, using the PWM_BCTL.PULSEMODEHI bits for pulse positioning, and PWM_CHANCFG.POLBH bits for polarity. |
| 9 (R/W)            | MODELSB    | 0 Invert of high output                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 9 (R/W)            | MODELSB    | 1 Independent control                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 8 (R/W)            | REFTMRB    | Channel B Timer Reference. The PWM_CHANCFG.REFTMRB bit selects whether the PWMuses PWMTMR1 or PWMTMR0 as the reference timer for Channel B operation.                                                                                                                                                                                                                                                                                                                                               |
| 8 (R/W)            | REFTMRB    | 0 PWMTMR0 is Channel B reference                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 8 (R/W)            | REFTMRB    | 1 PWMTMR1 is Channel B reference                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 19-37: PWM\_CHANCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | ENCHOPAL   | Channel A Gate Chopping Enable Low Side. The PWM_CHANCFG.ENCHOPAL bit enables mixing of the Channel A low side out- put signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register. |
| 6 (R/W)            | ENCHOPAL   | 0 Disable Chopping Channel A Low Side                                                                                                                                                                                     |
| 5 (R/W)            | POLAL      | Channel A low side Polarity. The PWM_CHANCFG.POLAL bit selects the Channel A low side output polarity (ac- tive-high or active-low).                                                                                      |
| 5 (R/W)            | POLAL      | 0 Active Low                                                                                                                                                                                                              |
| 5 (R/W)            | POLAL      | 1 Active High                                                                                                                                                                                                             |
| 4 (R/W)            | ENHPAH     | Channel A heightened-precision enable for high side Output. The PWM_CHANCFG.ENHPAH bit enables heightened-precision Channel A high side output.                                                                           |
| 4 (R/W)            | ENHPAH     | 0 Disable HP Output Channel A High                                                                                                                                                                                        |
| 4 (R/W)            | ENHPAH     | 1 Enable HP Output Channel A High                                                                                                                                                                                         |
| 3 (R/W)            | ENCHOPAH   | Channel A Gate Chopping Enable High Side. The PWM_CHANCFG.ENCHOPAH bit enables mixing of the Channel A high side output signals with a high-frequency chopping signal, which is configured with the PWM_CHOPCFG register. |
| 3 (R/W)            | ENCHOPAH   | 0 Disable Chopping Channel A High Side                                                                                                                                                                                    |
| 3 (R/W)            | ENCHOPAH   | 1 Enable Chopping Channel A High Side                                                                                                                                                                                     |
| 2 (R/W)            | POLAH      | Channel A High side Polarity. The PWM_CHANCFG.POLAH bit selects the Channel A high side output polarity (ac- tive-high or active-low).                                                                                    |
| 2 (R/W)            | POLAH      | 0 Active Low                                                                                                                                                                                                              |
| 2 (R/W)            | POLAH      | 1 Active High                                                                                                                                                                                                             |

Table 19-37: PWM\_CHANCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MODELSA    | Channel A Mode of low Side Output. The PWM_CHANCFG.MODELSA bit selects whether the low side output waveform is based on independent controls or whether the low side output depends on the high side output controls. When PWM_CHANCFG.MODELSA =0, the low side output is an inverted form of the high side output, which is generated using the PWM_AH0 and PWM_AH1 registers for pulse width, using the PWM_ACTL.PULSEMODEHI bits for pulse positioning, and PWM_CHANCFG.POLAH bits for polarity. | Channel A Mode of low Side Output. The PWM_CHANCFG.MODELSA bit selects whether the low side output waveform is based on independent controls or whether the low side output depends on the high side output controls. When PWM_CHANCFG.MODELSA =0, the low side output is an inverted form of the high side output, which is generated using the PWM_AH0 and PWM_AH1 registers for pulse width, using the PWM_ACTL.PULSEMODEHI bits for pulse positioning, and PWM_CHANCFG.POLAH bits for polarity. |
| 0 (R/W)            | REFTMRA    | Channel A Timer Reference. The PWM_CHANCFG.REFTMRA bit selects whether the PWMuses PWMTMR1 or PWMTMR0 as the reference timer for Channel A operation.                                                                                                                                                                                                                                                                                                                                               | Channel A Timer Reference. The PWM_CHANCFG.REFTMRA bit selects whether the PWMuses PWMTMR1 or PWMTMR0 as the reference timer for Channel A operation.                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            | REFTMRA    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | PWMTMR0 is Channel A reference                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 0 (R/W)            | REFTMRA    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | PWMTMR1 is Channel A reference                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

## Channel A Dead-time Register

The PWM\_CHA\_DT register controls the value of dead-time for channel A independently.

Figure 19-60: PWM\_CHA\_DT Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000059_d2646e0239ee40b3697d33bbfd993842e77bb8d5480f74eafcb829b0954f3eb9.png)

Table 19-38: PWM\_CHA\_DT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 9:0                | VALUE      | Dead-time value.          |
| (R/W)              |            |                           |

## Channel B Dead-time Register

The PWM\_CHB\_DT register controls the value of dead-time for channel B independently.

Figure 19-61: PWM\_CHB\_DT Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000060_e17ba1060fc1eaf69bd1d4e783927705d57bf4ca0e869cd2a3fc6d3a164474c4.png)

Table 19-39: PWM\_CHB\_DT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 9:0                | VALUE      | Dead-time value.                                             |
| (R/W)              |            | The PWM_CHB_DT.VALUE bit field contains the dead-time value. |

## Channel C Dead-time Register

The PWM\_CHC\_DT register controls the value of dead-time for channel C independently.

Figure 19-62: PWM\_CHC\_DT Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000061_a1c01668633559bbde524874bf5879637ba46e67ca93e6289826c4e95a7057e9.png)

Table 19-40: PWM\_CHC\_DT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 9:0                | VALUE      | Dead-time value.                                             |
| (R/W)              |            | The PWM_CHC_DT.VALUE bit field contains the dead-time value. |

## Channel D Dead-time Register

The PWM\_CHD\_DT register controls the value of dead-time for channel D independently.

Figure 19-63: PWM\_CHD\_DT Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000062_e17ba1060fc1eaf69bd1d4e783927705d57bf4ca0e869cd2a3fc6d3a164474c4.png)

Table 19-41: PWM\_CHD\_DT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 9:0                | VALUE      | Dead-time value.                                             |
| (R/W)              |            | The PWM_CHD_DT.VALUE bit field contains the dead-time value. |

## Chop Configuration Register

The PWM\_CHOPCFG register holds a divisor value that controls the chopping frequency. The PWM permits a mixing of the output signals with a high-frequency chopping signal to aid with interfacing to pulse transformers. Also note that high-frequency chopping may be independently enabled for each channel's high-side and the low-side outputs using channel control bits. (For example, control chopping for Channel A with the PWM\_CHANCFG.ENCHOPAH and PWM\_CHANCFG.ENCHOPAH bits.)

Figure 19-64: PWM\_CHOPCFG Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000063_4d3bea0f3e42c56d421984363ee282074c90f58b0da41663d67e6dfbbd59e4b2.png)

Table 19-42: PWM\_CHOPCFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Gate Chopping Divisor. The PWM_CHOPCFG.VALUE bits provide the high frequency chopping divisor. When the divisor value is changed, the new period takes effect from the next edge of the chopping signal. The PWM_CHOPCFG.VALUE value may be calculated using ei- ther of the following formulas: CHOPDIV = [(T CHOP /T CK ) / 4] - 1 CHOPDIV = [(f CK / f CHOP ) / 4] - 1 |

## Channel C-High Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_CH\_DUTY0 register contains the PWM\_CH\_DUTY0.DUTY bit field from the PWM\_CH0 register and the PWM\_CH\_DUTY0.ENHDIV bit field from the PWM\_CH0\_HP register.

Note that the PWM\_CH\_DUTY0 register reads the PWM\_CH0 and the PWM\_CH0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-65: PWM\_CH\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000064_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-43: PWM\_CH\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_CH_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_CH_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CH_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel C-High Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_CH\_DUTY1 register contains the PWM\_CH\_DUTY1.DUTY bit field from the PWM\_CH1 register and the PWM\_CH\_DUTY1.ENHDIV bit field from the PWM\_CH1\_HP register.

Note that the PWM\_CH\_DUTY1 register reads the PWM\_CH1 and the PWM\_CH1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-66: PWM\_CH\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000065_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-44: PWM\_CH\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_CH_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_CH_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CH_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel C-Low Pulse Duty Register 0

The PWM\_CL0 and PWM\_CL1 registers determine the width for the low side output pulses. The values in these registers select the assertion count (in terms of t CK) of the low side output pulses for the channel C duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_CCTL.PULSEMODELO bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_CL0 register to determine the assertion and deassertion count for the low side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel C low pulse output for count less than PWM\_CL0 and deasserts this output for count greater than PWM\_CL1 .

The value range for the PWM\_CL0 and PWM\_CL1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHC\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_CL0 and PWM\_CL1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHC\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHC\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHC\_DT (two's complement) to -PWM\_TM0 /2 -PWM\_CHC\_DT .

Note that using values in the PWM\_CL0 or PWM\_CL1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-67: PWM\_CL0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000066_3ed5343d2813652a08d2c93a7af64280bdade40553b1c6dc2ac9db790b5e5faf.png)

Table 19-45: PWM\_CL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count.                                                                |
| (R/W)              |            | The PWM_CL0.DUTY bits select the duty cycle asserted count for Channel C low side output. |

## Channel C-Low Pulse Duty Register 1

The PWM\_CL0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_CL0 register, allows programs to specify fractional duty cycles. The PWM\_CL0\_HP register and the PWM\_CL0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_CL0\_HP and the PWM\_CL0 registers are also present in the single full duty register (if available).

Figure 19-68: PWM\_CL0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000067_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-46: PWM\_CL0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CL0_HP.ENHDIV bits provide fractional duty cycles for Channel C low side output. |

## Channel C-Low Duty-1 Register

The PWM\_CL0 and PWM\_CL1 registers determine the width for the low side output pulses. For more information, see the PWM\_CL0 register description.

Figure 19-69: PWM\_CL1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000068_99b70ea8f8fb82649d98d245412e7d64cfc16f375b21f8ab0edcc2b2001e12d3.png)

Table 19-47: PWM\_CL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. |
| (R/W)              |            |                               |

## Channel C-Low Heightened-Precision Duty-1 Register

The PWM\_CL1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_CL1 register, allows programs to specify fractional duty cycles. The PWM\_CL1\_HP register and the PWM\_CL1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_CL1\_HP and the PWM\_CL1 registers are also present in the single full duty register (if available).

Figure 19-70: PWM\_CL1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000069_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-48: PWM\_CL1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CL1_HP.ENHDIV bits provide fractional duty cycles for Channel C low side output. |

## Channel C-Low Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_CL\_DUTY0 register contains the PWM\_CL\_DUTY0.DUTY bit field from the PWM\_CL0 register and the PWM\_CL\_DUTY0.ENHDIV bit field from the PWM\_CL0\_HP register.

Note that the PWM\_CL\_DUTY0 register reads the PWM\_CL0 and the PWM\_CL0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-71: PWM\_CL\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000070_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-49: PWM\_CL\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_CL_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_CL_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CL_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel C-Low Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_CL\_DUTY1 register contains the PWM\_CL\_DUTY1.DUTY bit field from the PWM\_CL1 register and the PWM\_CL\_DUTY1.ENHDIV bit field from the PWM\_CL1\_HP register.

Note that the PWM\_CL\_DUTY1 register reads the PWM\_CL1 and the PWM\_CL1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-72: PWM\_CL\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000071_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-50: PWM\_CL\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_CL_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_CL_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_CL_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Control Register

The PWM\_CTL register enables the PWM, enables delay counters for the channels, and configures sync features. This register also provides support for tripping a PWM fault condition through software.

Figure 19-73: PWM\_CTL Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000072_f42618eb9ac7cc6167681ed670419d649ab9709dc7b9104a427d9fa23b53ce71.png)

Table 19-51: PWM\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20:18 (R/W)        | INTSYNCREF | Timer reference for Internal Sync. The PWM_CTL.INTSYNCREF bits select the timer reference for the internal sync. Note that all other combinations reserved. | Timer reference for Internal Sync. The PWM_CTL.INTSYNCREF bits select the timer reference for the internal sync. Note that all other combinations reserved. |
|                    |            | 0                                                                                                                                                           | PWMTMR0 provides sync reference                                                                                                                             |
|                    |            | 1                                                                                                                                                           | PWMTMR1 provides sync reference                                                                                                                             |
|                    |            | 2                                                                                                                                                           | PWMTMR2 provides sync reference                                                                                                                             |
|                    |            | 3                                                                                                                                                           | PWMTMR3 provides sync reference                                                                                                                             |
|                    |            | 4                                                                                                                                                           | PWMTMR4 provides sync reference                                                                                                                             |

Table 19-51: PWM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | EXTSYNCSEL | External Sync Select. The PWM_CTL.EXTSYNCSEL bit selects whether the external sync signal is synchro- nous or asynchronous. Note that latency in PWMsync response differs between asyn- chronous and synchronous external sync modes. For more information, see thePWM functional description.                                                                                                                            |
| 17 (R/W)           | EXTSYNCSEL | 0 Asynchronous External Sync                                                                                                                                                                                                                                                                                                                                                                                              |
| 17 (R/W)           | EXTSYNCSEL | 1 Synchronous External Sync                                                                                                                                                                                                                                                                                                                                                                                               |
| 16 (R/W)           | EXTSYNC    | External Sync. The PWM_CTL.EXTSYNC bit selects whether the PWMuses an external or internal sync signal for the main timer (PWMTMR0). Do not change the value of the PWM_CTL.EXTSYNC bit while the PWMis enabled ( PWM_CTL.GLOBEN =1).                                                                                                                                                                                     |
| 16 (R/W)           | EXTSYNC    | 0 Internal sync used                                                                                                                                                                                                                                                                                                                                                                                                      |
| 16 (R/W)           | EXTSYNC    | 1 External sync used                                                                                                                                                                                                                                                                                                                                                                                                      |
| 8 (R/W)            | ADEN       | Asymmetric Dead-time Enable. When symmetric dead-time is enabled ( PWM_CTL.ADEN = 0), in the dependent mode, both the high-side and low-side outputs are reduced by DT cycles on both the assertion and de-assertion edges. When symmetric dead-time is disabled ( PWM_CTL.ADEN = 1), the falling edges of both high and low-side outputs occur at the programmed duty value, but their rise-times are delayed by 2 x DT. |
| 8 (R/W)            | ADEN       | 0 Dead-time is symmetric                                                                                                                                                                                                                                                                                                                                                                                                  |
| 8 (R/W)            | ADEN       | 1 Dead-time is asymmetric                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | DLYDEN     | Enable Delay Counter for Channel D. The PWM_CTL.DLYDEN bit enables the Channel Ddelay counter, supporting phase- offset control. Do not change the value of the PWM_CTL.DLYDEN bit while the PWMis enabled ( PWM_CTL.GLOBEN =1).                                                                                                                                                                                          |
| 7 (R/W)            | DLYDEN     | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | DLYDEN     | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 6 (R/W)            | DLYCEN     | Enable Delay Counter for Channel C. The PWM_CTL.DLYCEN bit enables the Channel C delay counter, supporting phase- offset control. Do not change the value of the PWM_CTL.DLYCEN bit while the PWMis enabled ( PWM_CTL.GLOBEN =1).                                                                                                                                                                                         |
| 6 (R/W)            | DLYCEN     | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 6 (R/W)            | DLYCEN     | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 19-51: PWM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | DLYBEN     | Enable Delay Counter for Channel B. The PWM_CTL.DLYBEN bit enables the Channel B delay counter, supporting phase- offset control. Do not change the value of the PWM_CTL.DLYBEN bit while the PWMis enabled ( PWM_CTL.GLOBEN =1).                                                                               |
| 5 (R/W)            | DLYBEN     | 0 Disable                                                                                                                                                                                                                                                                                                       |
| 4 (R/W)            | DLYAEN     | Enable Delay Counter for Channel A. The PWM_CTL.DLYAEN bit enables the Channel A delay counter, supporting phase- offset control. Do not change the value of the PWM_CTL.DLYAEN bit while the PWMis enabled ( PWM_CTL.GLOBEN =1).                                                                               |
| 4 (R/W)            | DLYAEN     | 0 Disable                                                                                                                                                                                                                                                                                                       |
| 4 (R/W)            | DLYAEN     | 1 Enable                                                                                                                                                                                                                                                                                                        |
| 3 (R/W)            | DUEN       | Double Update Mode Enable. In Single Update Mode, double buffering of all registers happens at the period boun- dary of the PWMtimer. In Double Update Mode, double buffering of all registers (ex- cept delay counter registers) happens at the middle of the period as well as the begin- ning of the period. |
| 3 (R/W)            | DUEN       | 0 Single Update Mode                                                                                                                                                                                                                                                                                            |
| 3 (R/W)            | DUEN       | 1 Double Update Mode                                                                                                                                                                                                                                                                                            |
| 2 (R0/W)           | SWTRIP     | Software Trip. The PWM_CTL.SWTRIP bit permits tripping a fault condition through software, shutting down PWMoutput. This bit always read as 0. If the PWM_CTL.SWTRIP bit and PWM_CTL.GLOBEN bit are set in the same write, the write does not trip the fault condition.                                         |
| 2 (R0/W)           | SWTRIP     | 0 Don't force fault                                                                                                                                                                                                                                                                                             |
| 2 (R0/W)           | SWTRIP     | 1 Force a Fault Trip Condition                                                                                                                                                                                                                                                                                  |
| 1 (R/W)            | EMURUN     | Output Behavior During Emulation Mode. The PWM_CTL.EMURUN bit selects PWMoutput behavior during emulation mode.                                                                                                                                                                                                 |
| 1 (R/W)            | EMURUN     | 0 Disable Outputs                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | EMURUN     | 1 Enable Outputs                                                                                                                                                                                                                                                                                                |

Table 19-51: PWM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | GLOBEN     | Module Enable. The PWM_CTL.GLOBEN bit enables the PWM, enabling all timers and outputs. While this bit is enabled, processor code should not change the value of the PWM_CTL.DLYAEN bit, PWM_CTL.DLYBEN bit, PWM_CTL.DLYCEN bit, PWM_CTL.DLYDEN bit, PWM_CTL.EXTSYNCSEL bit, or any bits in the PWM_CHANCFG register. Note that there is a latency between PWMdisable and the cessation of output waveforms. There is also a latency between PWMenable and start of output waveforms. For the latency description, see the PWMfunctional description. |
| 0 (R/W)            | GLOBEN     | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 0 (R/W)            | GLOBEN     | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## Channel D Control Register

The PWM\_DCTL register selects the low and high side output pulse mode, enables low and high side output, and enables low/high side output crossover.

Figure 19-74: PWM\_DCTL Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000073_2569c529a577feffe3a2efc08ceeea767c875b63378986bf8d6da292b5c9b2bb.png)

Table 19-52: PWM\_DCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:10 (R/W)        | PULSEMODELO | Low Side Output Pulse Position. The PWM_DCTL.PULSEMODELO bits select the pulse position for Channel Dlow side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_DL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_DL0 and PWM_DL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_DL0 and PWM_DL1 ). | Low Side Output Pulse Position. The PWM_DCTL.PULSEMODELO bits select the pulse position for Channel Dlow side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_DL0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_DL0 and PWM_DL1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_DL0 and PWM_DL1 ). |
|                    |             | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Symmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |             | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Asymmetrical                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |             | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Left Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |             | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Right Half                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 19-52: PWM\_DCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:8 (R/W)          | PULSEMODEHI | High Side Output Pulse Position. The PWM_DCTL.PULSEMODEHI bits select the pulse position for Channel Dhigh side output. In symmetrical mode, the channel forms a symmetrical pulse waveform around the center of the PWMperiod. Only one of the duty cycle registers is used for an output in symmetrical mode. Note that in this mode, the values in the PWM_DH0 register is scaled, such that a value of 0 produces 50% duty. In asymmetrical mode, the channel forms an asymmetrical pulse waveform around the center of the PWMperiod. This mode uses both the duty cycle registers ( PWM_DH0 and PWM_DH1 ). In left half or right half mode, the channel forms the pulse waveforms on either the first half (left) or the second half (right) of the PWMperiod. This mode uses both the duty cycle reg- isters ( PWM_DH0 and PWM_DH1 ). 0 Symmetrical |
| 2 (R/W)            | XOVR        | 3 Right Half high-low Crossover Enable. The PWM_DCTL.XOVR bit enables crossover between the channels high and low side outputs. When enabled, this bit directs the PWMto send the low-side output through the high-side output pin and the high-side output through the low side output pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 1 (R/W)            | DISLO       | 1 Enable Crossover Channel Low Side Output Disable. The PWM_DCTL.DISLO bit enables the channels low side output. 0 Enable Low Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | DISHI       | 1 Disable Low Side Output Channel High Side Output Disable. The                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |             | PWM_DCTL.DISHI bit enables the channels high side output. 0 Enable High Side Output 1 Disable High Side Output                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

## Channel D-High Duty-0 Register

The PWM\_DH0 and PWM\_DH1 registers determine the width for the high side output pulses. The values in these registers select the assertion count (in terms of t CK) of the high side output pulses for the channel D duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_DCTL.PULSEMODEHI bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_DH0 register to determine the assertion and deassertion count for the high side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel D high pulse output for count less than PWM\_DH0 and deasserts this output for count greater than PWM\_DH1 .

The value range for the PWM\_DH0 and PWM\_DH1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHD\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_DH0 and PWM\_DH1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHD\_DT (two's complement) to + PWM\_TM0 /2 + PWM\_CHD\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHD\_DT (two's complement) to -PWM\_TM0 /2 -PWM\_CHD\_DT .

Note that using values in the PWM\_DH0 or PWM\_DH1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-75: PWM\_DH0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000074_3ed5343d2813652a08d2c93a7af64280bdade40553b1c6dc2ac9db790b5e5faf.png)

Table 19-53: PWM\_DH0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count.                                                                |
| (R/W)              |            | The PWM_DH0.DUTY bits select the duty cycle asserted count for Channel Dhigh side output. |

## Channel D-High Pulse Heightened-Precision Duty Register 0

The PWM\_DH0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_DH0 register, allows programs to specify fractional duty cycles. The PWM\_DH0\_HP register and the PWM\_DH0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_DH0\_HP and the PWM\_DH0 registers are also present in the single full duty register (if available).

Figure 19-76: PWM\_DH0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000075_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-54: PWM\_DH0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DH0_HP.ENHDIV bits provide fractional duty cycles for Channel Dhigh side output. |

## Channel D-High Pulse Duty Register 1

The PWM\_DH0 and PWM\_DH1 registers determine the width for the high side output pulses. For more information, see the PWM\_DH0 register description.

Figure 19-77: PWM\_DH1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000076_4e56b37c2f4cd8f0b62f00e4f95b1b6d7a9ed72aeed7d23f1eceb00c48102f6a.png)

Table 19-55: PWM\_DH1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. The PWM_DH1.DUTY bits select the duty cycle de-asserted count for Channel Dhigh side output. |
| (R/W)              |            |                                                                                                                            |

## Channel D High Pulse Heightened-Precision Duty Register 1

The PWM\_DH1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_DH1 register, allows programs to specify fractional duty cycles. The PWM\_DH1\_HP register and the PWM\_DH1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_DH1\_HP and the PWM\_DH1 registers are also present in the single full duty register (if available).

Figure 19-78: PWM\_DH1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000077_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-56: PWM\_DH1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DH1_HP.ENHDIV bits provide fractional duty cycles for Channel Dhigh side output. |

## Channel D-High Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_DH\_DUTY0 register contains the PWM\_DH\_DUTY0.DUTY bit field from the PWM\_DH0 register and the PWM\_DH\_DUTY0.ENHDIV bit field from the PWM\_DH0\_HP register.

Note that the PWM\_DH\_DUTY0 register reads the PWM\_DH0 and the PWM\_DH0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-79: PWM\_DH\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000078_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-57: PWM\_DH\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_DH_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_DH_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DH_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel D-High Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_DH\_DUTY1 register contains the PWM\_DH\_DUTY1.DUTY bit field from the PWM\_DH1 register and the PWM\_DH\_DUTY1.ENHDIV bit field from the PWM\_DH1\_HP register.

Note that the PWM\_DH\_DUTY1 register reads the PWM\_DH1 and the PWM\_DH1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-80: PWM\_DH\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000079_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-58: PWM\_DH\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_DH_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_DH_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DH_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel D-Low Pulse Duty Register 0

The PWM\_DL0 and PWM\_DL1 registers determine the width for the low side output pulses. The values in these registers select the assertion count (in terms of t CK) of the low side output pulses for the channel D duty cycle.

The operation of the duty-cycle registers varies, depending on the pulse mode selected with the PWM\_DCTL.PULSEMODELO bits. When the pulse mode is symmetrical, the PWM uses the value in the PWM\_DL0 register to determine the assertion and deassertion count for the low side output pulses. When the pulse mode is asymmetrical, left half, or right half, the PWM asserts channel D low pulse output for count less than PWM\_DL0 and deasserts this output for count greater than PWM\_DL1 .

The value range for the PWM\_DL0 and PWM\_DL1 registers depends on the period of the timer being used by the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 (two's complement) and + PWM\_TM0 /2, when dead time ( PWM\_CHD\_DT ) is not considered.

When dead time is considered for symmetrical and asymmetrical pulse modes, the value range for PWM\_DL0 and PWM\_DL1 depends on the period of the time being used by the channel and the amount of dead time applied to the channel. For example, if PWM\_TM0 is used, the duty cycle values may be between -PWM\_TM0 /2 + PWM\_CHD\_DT (two's complement) and + PWM\_TM0 /2 + PWM\_CHD\_DT .

When dead time is considered for left half or right half pulse modes, if PWM\_TM0 is used, the duty cycle values may be between PWM\_TM0 /2 + PWM\_CHD\_DT (two's complement) and -PWM\_TM0 /2 -PWM\_CHD\_DT .

Note that using values in the PWM\_DL0 or PWM\_DL1 registers that fall outside these limits causes PWM over or under modulation.

For more information about pulse modes and duty cycle selection, see the Functional Description section.

Figure 19-81: PWM\_DL0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000080_3ed5343d2813652a08d2c93a7af64280bdade40553b1c6dc2ac9db790b5e5faf.png)

Table 19-59: PWM\_DL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle Asserted Count.                                                               |
| (R/W)              |            | The PWM_DL0.DUTY bits select the duty cycle asserted count for Channel Dlow side output. |

## Channel D-Low Heightened-Precision Duty-0 Register

The PWM\_DL0\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_DL0 register, allows programs to specify fractional duty cycles. The PWM\_DL0\_HP register and the PWM\_DL0 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_DL0\_HP and the PWM\_DL0 registers are also present in the single full duty register (if available).

Figure 19-82: PWM\_DL0\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000081_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-60: PWM\_DL0\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DL0_HP.ENHDIV bits provide fractional duty cycles for Channel Dlow side output. |

## Channel D-Low Pulse Duty Register 1

The PWM\_DL0 and PWM\_DL1 registers determine the width for the low side output pulses. For more information, see the PWM\_DL0 register description.

Figure 19-83: PWM\_DL1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000082_4e56b37c2f4cd8f0b62f00e4f95b1b6d7a9ed72aeed7d23f1eceb00c48102f6a.png)

Table 19-61: PWM\_DL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 15:0               | DUTY       | Duty Cycle De-Asserted Count. The PWM_DL1.DUTY bits select the duty cycle de-asserted count for Channel Dlow side output. |
| (R/W)              |            |                                                                                                                           |

## Channel D-Low Heightened-Precision Duty-1 Register

The PWM\_DL1\_HP register provides a fine-grained edge placement within the system clock period. This register, in conjunction with the PWM\_DL1 register, allows programs to specify fractional duty cycles. The PWM\_DL1\_HP register and the PWM\_DL1 register work together in a Q15.8 signed two's complement fixed-point format.

Note that the bit fields in the PWM\_DL1\_HP and the PWM\_DL1 registers are also present in the single full duty register (if available).

Figure 19-84: PWM\_DL1\_HP Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000083_cf77be637fb4617faeacd9648875bc1559c4f5cf3d5b8ed1c1e40fa27a184256.png)

Table 19-62: PWM\_DL1\_HP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------|
| 7:6 (R/W)          | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DL1_HP.ENHDIV bits provide fractional duty cycles for Channel Dlow side output. |

## Channel A Delay Register

The PWM\_DLYA register controls a delay for the Channel A timer (only PWMTMR1, PWMTMR2, PWMTMR3 or PWMTMR4) with reference to the main timer (PWMTMR0). To use apply this delay, the delay must be enabled ( PWM\_CTL.DLYAEN =1). For more information about applying the delay, see the PWM Functional Description section. Note that the PWM\_DLYA delay value must be less that less that twice the period value of the timer being used for the channel (for example, if PWMTMR1 is used, PWM\_DLYA must be less than 2xPWMTMR1). Also, note that the period of the main timer must be an integer multiple of the timer being used for the channel (for example, if PWMTMR1 is used, PWMTMR0 = NxPWMTMR1, where N is an integer).

Figure 19-85: PWM\_DLYA Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000084_499694330aa7a6afca4e863003f0eed902e0659afb28edd44356c574acddf361.png)

Table 19-63: PWM\_DLYA Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Channel A Delay Value. The PWM_DLYA.VALUE bits select the phase delay between the main timer (PWMTMR0) and the timer used for Channel A. |

## Channel B Delay Register

The PWM\_DLYB register controls a delay for the Channel B timer (only PWMTMR1, PWMTMR2, PWMTMR3 or PWMTMR4) with reference to the main timer (PWMTMR0). To use apply this delay, the delay must be enabled ( PWM\_CTL.DLYBEN =1). For more information about applying the delay, see the PWM Functional Description section. Note that the PWM\_DLYB delay value must be less that less that twice the period value of the timer being used for the channel (for example, if PWMTMR1 is used, PWM\_DLYB must be less than 2xPWMTMR1). Also, note that the period of the main timer must be an integer multiple of the timer being used for the channel (for example, if PWMTMR1 is used, PWMTMR0 = NxPWMTMR1, where N is an integer).

Figure 19-86: PWM\_DLYB Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000085_551142b650ee9cf5375af89b5064c78573a765b37a93e0a48d02a88b9731a3e8.png)

Table 19-64: PWM\_DLYB Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Channel B Delay Value. The PWM_DLYB.VALUE bits select the phase delay between the main timer (PWMTMR0) and the timer used for Channel B. |

## Channel C Delay Register

The PWM\_DLYC register controls a delay for the Channel C timer (only PWMTMR1, PWMTMR2, PWMTMR3 or PWMTMR4) with reference to the main timer (PWMTMR0). To use apply this delay, the delay must be enabled ( PWM\_CTL.DLYCEN =1). For more information about applying the delay, see the PWM Functional Description section. Note that the PWM\_DLYC delay value must be less that less that twice the period value of the timer being used for the channel (for example, if PWMTMR1 is used, PWM\_DLYC must be less than 2xPWMTMR1). Also, note that the period of the main timer must be an integer multiple of the timer being used for the channel (for example, if PWMTMR1 is used, PWMTMR0 = NxPWMTMR1, where N is an integer).

Figure 19-87: PWM\_DLYC Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000086_4f0527e79ba54257ee70ac5f5df627a865e43be247a8222d281448d2210923cb.png)

Table 19-65: PWM\_DLYC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Channel C Delay Value. The PWM_DLYC.VALUE bits select the phase delay between the main timer (PWMTMR0) and the timer used for Channel C. |

## Channel D Delay Register

The PWM\_DLYD register controls a delay for the Channel D timer (only PWMTMR1, PWMTMR2, PWMTMR3 or PWMTMR4) with reference to the main timer (PWMTMR0). To use apply this delay, the delay must be enabled ( PWM\_CTL.DLYDEN =1). For more information about applying the delay, see the PWM Functional Description section. Note that the PWM\_DLYD delay value must be less that less that twice the period value of the timer being used for the channel (for example, if PWMTMR1 is used, PWM\_DLYD must be less than 2xPWMTMR1). Also, note that the period of the main timer must be an integer multiple of the timer being used for the channel (for example, if PWMTMR1 is used, PWMTMR0 = NxPWMTMR1, where N is an integer).

Figure 19-88: PWM\_DLYD Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000087_0df0679c1ad55b656e81da202eef5c73a56650cb0d430eb1871d406c98e992e9.png)

Table 19-66: PWM\_DLYD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Channel DDelay Value. The PWM_DLYD.VALUE bits select the phase delay between the main timer (PWMTMR0) and the timer used for Channel D. |

## Channel D-Low Full Duty0 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_DL\_DUTY0 register contains the PWM\_DL\_DUTY0.DUTY bit field from the PWM\_DL0 register and the PWM\_DL\_DUTY0.ENHDIV bit field from the PWM\_DL0\_HP register.

Note that the PWM\_DL\_DUTY0 register reads the PWM\_DL0 and the PWM\_DL0\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-89: PWM\_DL\_DUTY0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000088_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-67: PWM\_DL\_DUTY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_DL_DUTY0.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_DL_DUTY0.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DL_DUTY0.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Channel D-Low Full Duty1 Register

The full duty registers can be used instead of the combined duty and heightened-precision duty registers.The PWM\_DL\_DUTY1 register contains the PWM\_DL\_DUTY1.DUTY bit field from the PWM\_DL1 register and the PWM\_DL\_DUTY1.ENHDIV bit field from the PWM\_DL1\_HP register.

Note that the PWM\_DL\_DUTY1 register reads the PWM\_DL1 and the PWM\_DL1\_HP register values and visa-versa.

When heightened-precision edge placement is enabled, bits [15:8] of these registers form the decimal part of a noninteger, fixed-point duty cycle value in Q15.8 format. The lowest bits are ignored.

Figure 19-90: PWM\_DL\_DUTY1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000089_3eaae2c62e051b2b72c1286c4d256294781e1f6142503869e3b655d58d792b61.png)

Table 19-68: PWM\_DL\_DUTY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | DUTY       | Coarse Duty Value. The PWM_DL_DUTY1.DUTY bits determine the output pulse-widths in the normal PWMoperation. When heightened-precision edge placement is enabled, the PWM_DL_DUTY1.DUTY bit field forms the integer part of a non-integer, fixed-point duty cycle value in Q15.8 format. |
| 15:14 (R/W)        | ENHDIV     | Enhanced Precision Divider Bits. The PWM_DL_DUTY1.ENHDIV bits form the decimal part of a non-integer, fixed- point duty cycle value when heightened-precision edge placement is enabled in Q15.8 format.                                                                                |

## Interrupt Latch Register

The PWM\_ILAT register latches the occurrence of unmasked (enabled) PWM interrupt requests. These interrupt requests are unmasked or masked with the PWM\_IMSK register.

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000090_14cfb1abd4f3011506c6bd37d48b285ad74ba0a9aba5bf8a9855a92e49715ed2.png)

Status

Figure 19-91: PWM\_ILAT Register Diagram

Table 19-69: PWM\_ILAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W1C)         | TMR4PER    | PWMTMR4 Period Latched Interrupt Status. The PWM_ILAT.TMR4PER bit indicates the latched status of the PWMTMR4 peri- od boundary interrupt request. |
| 19 (R/W1C)         | TMR3PER    | PWMTMR3 Period Latched Interrupt Status. The PWM_ILAT.TMR3PER bit indicates the latched status of the PWMTMR3 peri- od boundary interrupt request. |
| 18 (R/W1C)         | TMR2PER    | PWMTMR2 Period Latched Interrupt Status. The PWM_ILAT.TMR2PER bit indicates the latched status of the PWMTMR2 peri- od boundary interrupt request. |
| 18 (R/W1C)         |            | 0 No Interrupt Latched                                                                                                                             |
| 18 (R/W1C)         | 1          | Interrupt Latched                                                                                                                                  |

Table 19-69: PWM\_ILAT Register Fields (Continued)

| Bit No. (Access)   | Description/Enumeration                                                                                                                                     |
|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | PWMTMR1 Period Latched Interrupt Status. The PWM_ILAT.TMR1PER bit indicates the latched status of the PWMTMR1 peri- od boundary interrupt request.          |
| 16 (R/W1C)         | PWMTMR0 Period Boundary Interrupt Latched Status. The PWM_ILAT.TMR0PER bit indicates the latched status of the PWMTMR0 peri- od boundary interrupt request. |
| 1 (R/W1C)          | TRIP1 Interrupt Latched Status. The PWM_ILAT.TRIP1 bit indicates the latched status of the TRIP1 interrupt re- quest.                                       |
| 0 (R/W1C)          | TRIP0 Interrupt Latched Status. The PWM_ILAT.TRIP0 bit indicates the latched status of the TRIP0 interrupt re- quest. 0 No Interrupt Latched                |

## Interrupt Mask Register

The PWM\_IMSK register masks (disables) or unmasks (enables) PWM interrupt requests. When an unmasked interrupt occurs, the PWM latches the interrupt status in the PWM\_ILAT register.

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000091_83236cc53b8d572a86ab4f5efb562b99ffd6cf7398a7a2dedc60517c6727baaf.png)

Enable

Figure 19-92: PWM\_IMSK Register Diagram

Table 19-70: PWM\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | TMR4PER    | PWMTMR4 Period Boundary Interrupt Enable. The PWM_IMSK.TMR4PER bit enables (unmasks) the PWMTMR4 period boundary interrupt request. This condition occurs when the timers period boundary is reached ( PWM_STAT.TMR4PER =1). | PWMTMR4 Period Boundary Interrupt Enable. The PWM_IMSK.TMR4PER bit enables (unmasks) the PWMTMR4 period boundary interrupt request. This condition occurs when the timers period boundary is reached ( PWM_STAT.TMR4PER =1). |
| 19 (R/W)           | TMR3PER    | PWMTMR3 Period Boundary Interrupt Enable. The PWM_IMSK.TMR3PER bit enables (unmasks) the PWMTMR3 period boundary interrupt request. This condition occurs when the timers period boundary is reached                         | PWMTMR3 Period Boundary Interrupt Enable. The PWM_IMSK.TMR3PER bit enables (unmasks) the PWMTMR3 period boundary interrupt request. This condition occurs when the timers period boundary is reached                         |
|                    |            | 0                                                                                                                                                                                                                            | Mask PWMTMR3 Period Interrupt                                                                                                                                                                                                |
|                    |            | 1                                                                                                                                                                                                                            | Unmask PWMTMR3 Period Interrupt                                                                                                                                                                                              |

Table 19-70: PWM\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | TMR2PER    | PWMTMR2 Period Boundary Interrupt Enable. The PWM_IMSK.TMR2PER bit enables (unmasks) the PWMTMR2 period boundary interrupt request. This condition occurs when the timers period boundary is reached ( PWM_STAT.TMR2PER =1). |
| 18 (R/W)           | TMR2PER    | 0 Mask PWMTMR2 Period Interrupt                                                                                                                                                                                              |
| 18 (R/W)           | TMR2PER    | 1 Unmask PWMTMR2 Period Interrupt                                                                                                                                                                                            |
| 17 (R/W)           | TMR1PER    | PWMTMR1 Period Boundary Interrupt Enable. The PWM_IMSK.TMR1PER bit enables (unmasks) the PWMTMR1 period boundary interrupt request. This condition occurs when the timers period boundary is reached ( PWM_STAT.TMR1PER =1). |
| 17 (R/W)           | TMR1PER    | 0 Mask PWMTMR1 Period Interrupt                                                                                                                                                                                              |
| 17 (R/W)           | TMR1PER    | 1 Unmask PWMTMR1 Period Interrupt                                                                                                                                                                                            |
| 16 (R/W)           | TMR0PER    | PWMTMR0 Period Boundary Interrupt Enable. The PWM_IMSK.TMR0PER bit enables (unmasks) the PWMTMR0 period boundary interrupt request. This condition occurs when the timers period boundary is reached ( PWM_STAT.TMR0PER =1). |
| 16 (R/W)           | TMR0PER    | 0 Mask PWMTMR0 Period Interrupt                                                                                                                                                                                              |
| 16 (R/W)           | TMR0PER    | 1 Unmask PWMTMR0 Period Interrupt                                                                                                                                                                                            |
| 1 (R/W)            | TRIP1      | TRIP1 Interrupt Enable. The PWM_IMSK.TRIP1 bit enables (unmasks) the TRIP1 interrupt request. This condition occurs when fault input is tripped ( PWM_STAT.TRIP1 =1).                                                        |
| 1 (R/W)            | TRIP1      | 0 Mask TRIP1 Interrupt                                                                                                                                                                                                       |
| 1 (R/W)            | TRIP1      | 1 Unmask TRIP1 Interrupt                                                                                                                                                                                                     |
| 0 (R/W)            | TRIP0      | TRIP0 Interrupt Enable. The PWM_IMSK.TRIP0 bit enables (unmasks) the TRIP0 interrupt request. This condition occurs when fault input is tripped ( PWM_STAT.TRIP0 =1). Interrupt                                              |
| 0 (R/W)            | TRIP0      | 0 Mask TRIP0                                                                                                                                                                                                                 |
| 0 (R/W)            | TRIP0      | 1 Unmask TRIP0 Interrupt                                                                                                                                                                                                     |

## Status Register

The PWM\_STAT register indicates the PWM PWMTRIP1-0 fault and input level status, indicates the Channel A-D fault and self-restart status, and indicates the PWMTMR4-0 phase.

Figure 19-93: PWM\_STAT Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000092_99af7d904d8f93451ed9ab2787805e856ee11d3123381e64839395238b4b8a8e.png)

Table 19-71: PWM\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | HPRDY      | Heightened-Precision Ready Status. The PWM_STAT.HPRDY bit indicates whether or not the PWMis ready for height- ened-precision operation. |
| 31 (R/NW)          | HPRDY      | 0 HPPWM Not Ready For Operation                                                                                                          |
| 31 (R/NW)          | HPRDY      | 1 HPPWM Ready For Operation                                                                                                              |
| 28 (R/W1C)         | TMR4PHASE  | PWMTMR4 Phase Status. The PWM_STAT.TMR4PHASE bit indicates the current phase for the PWMTMR4 waveform.                                   |
| 28 (R/W1C)         | TMR4PHASE  | 0 1st Half Phase                                                                                                                         |
| 28 (R/W1C)         | TMR4PHASE  | 1 2nd Half Phase                                                                                                                         |
| 27 (R/W1C)         | TMR3PHASE  | PWMTMR3 Phase Status. The PWM_STAT.TMR3PHASE bit indicates the current phase for the PWMTMR3 waveform.                                   |
| 27 (R/W1C)         | TMR3PHASE  | 0 1st Half Phase                                                                                                                         |
| 27 (R/W1C)         | TMR3PHASE  | 1 2nd Half Phase                                                                                                                         |
| 26 (R/W1C)         | TMR2PHASE  | PWMTMR2 Phase Status. The PWM_STAT.TMR2PHASE bit indicates the current phase for the PWMTMR2 waveform.                                   |
| 26 (R/W1C)         | TMR2PHASE  | 0 1st Half Phase                                                                                                                         |
| 26 (R/W1C)         | TMR2PHASE  | 1 2nd Half Phase                                                                                                                         |
| 25 (R/W1C)         | TMR1PHASE  | PWMTMR1 Phase Status. The PWM_STAT.TMR1PHASE bit indicates the current phase for the PWMTMR1 waveform.                                   |
| 25 (R/W1C)         | TMR1PHASE  | 0 1st Half Phase                                                                                                                         |
| 25 (R/W1C)         | TMR1PHASE  | 1 2nd Half Phase                                                                                                                         |
| 24 (R/W1C)         | TMR0PHASE  | PWMTMR0 Phase Status. The PWM_STAT.TMR0PHASE bit indicates the current phase for the PWMTMR0 waveform.                                   |
| 24 (R/W1C)         | TMR0PHASE  | 0 1st Half Phase                                                                                                                         |
| 24 (R/W1C)         | TMR0PHASE  | 1 2nd Half Phase                                                                                                                         |

Table 19-71: PWM\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W1C)         | TMR4PER    | PWMTMR4 Period Boundary Status. The PWM_STAT.TMR4PER bit indicates whether or not the PWMTMR4 period boundary has been reached.                                                                       |
| 20 (R/W1C)         | TMR4PER    | 0 PWMTMR4 period boundary not reached                                                                                                                                                                 |
| 20 (R/W1C)         | TMR4PER    | 1 PWMTMR4 period boundary reached                                                                                                                                                                     |
| 19 (R/W1C)         | TMR3PER    | PWMTMR3 Period Boundary Status. The PWM_STAT.TMR3PER bit indicates whether or not the PWMTMR3 period boundary has been reached.                                                                       |
| 19 (R/W1C)         | TMR3PER    | 0 PWMTMR3 period boundary not reached                                                                                                                                                                 |
| 19 (R/W1C)         | TMR3PER    | 1 PWMTMR3 period boundary reached                                                                                                                                                                     |
| 18 (R/W1C)         | TMR2PER    | PWMTMR2 Period Boundary Status. The PWM_STAT.TMR2PER bit indicates whether or not the PWMTMR2 period boundary has been reached.                                                                       |
| 18 (R/W1C)         | TMR2PER    | 0 PWMTMR2 period boundary not reached                                                                                                                                                                 |
| 18 (R/W1C)         | TMR2PER    | 1 PWMTMR2 period boundary reached                                                                                                                                                                     |
| 17 (R/W1C)         | TMR1PER    | PWMTMR1 Period Boundary Status. The PWM_STAT.TMR1PER bit indicates whether or not the PWMTMR1 period boundary has been reached.                                                                       |
| 17 (R/W1C)         | TMR1PER    | 0 PWMTMR1 period boundary not reached                                                                                                                                                                 |
| 17 (R/W1C)         | TMR1PER    | 1 PWMTMR1 period boundary reached                                                                                                                                                                     |
| 16 (R/W1C)         | TMR0PER    | PWMTMR0 Period Boundary Status. The PWM_STAT.TMR0PER bit indicates whether or not the PWMTMR0 period boundary has been reached.                                                                       |
| 16 (R/W1C)         | TMR0PER    | 0 PWMTMR0 period boundary not reached                                                                                                                                                                 |
| 16 (R/W1C)         | TMR0PER    | 1 PWMTMR0 period boundary reached                                                                                                                                                                     |
| 11 (R/NW)          | SRTRIPD    | Self-Restart Trip Status for Channel D. The PWM_STAT.SRTRIPD bit indicates whether the PWMChannel Dself-restart has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit de- scription. |
| 11 (R/NW)          | SRTRIPD    | 0 Channel DSelf-Restart Trip Status is "not tripped"                                                                                                                                                  |
| 11 (R/NW)          | SRTRIPD    | 1 Channel DSelf-Restart Trip Status is "tripped"                                                                                                                                                      |

Table 19-71: PWM\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W1C)         | FLTTRIPD   | Fault Trip Status for Channel D. The PWM_STAT.FLTTRIPD bit indicates whether the PWMChannel Dfault has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit descrip- tion.                 |
| 10 (R/W1C)         | FLTTRIPD   | 0 Channel DFault Trip Status is "not tripped"                                                                                                                                                            |
| 10 (R/W1C)         | FLTTRIPD   | Channel DFault Trip Status is "tripped"                                                                                                                                                                  |
| 9 (R/NW)           | SRTRIPC    | 1 Self-Restart Trip Status for Channel C. The PWM_STAT.SRTRIPC bit indicates whether the PWMChannel C self-restart has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit de- scription. |
| 9 (R/NW)           | SRTRIPC    | 0 Channel C Self-Restart Trip Status is "not tripped"                                                                                                                                                    |
| 9 (R/NW)           | SRTRIPC    | 1 Channel C Self-Restart Trip Status is "tripped"                                                                                                                                                        |
| 8 (R/W1C)          | FLTTRIPC   | Fault Trip Status for Channel C. The PWM_STAT.FLTTRIPC bit indicates whether the PWMChannel C fault has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit descrip- tion.                |
| 8 (R/W1C)          | FLTTRIPC   | 0 Channel C Fault Trip Status is "not tripped"                                                                                                                                                           |
| 8 (R/W1C)          | FLTTRIPC   | 1 Channel C Fault Trip Status is "tripped"                                                                                                                                                               |
| 7 (R/NW)           | SRTRIPB    | Self-Restart Trip Status for Channel B. The PWM_STAT.SRTRIPB bit indicates whether the PWMChannel B self-restart has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit de- scription.   |
| 7 (R/NW)           | SRTRIPB    | 0 Channel B Self-Restart Trip Status is "not tripped"                                                                                                                                                    |
| 7 (R/NW)           | SRTRIPB    | 1 Channel B Self-Restart Trip Status is "tripped"                                                                                                                                                        |
| 6 (R/W1C)          | FLTTRIPB   | Fault Trip Status for Channel B. The PWM_STAT.FLTTRIPB bit indicates whether the PWMChannel B fault has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit descrip- tion.                |
| 6 (R/W1C)          | FLTTRIPB   | 0 Channel B Fault Trip Status is "not tripped"                                                                                                                                                           |
| 6 (R/W1C)          | FLTTRIPB   | 1 Channel A Fault Trip Status is "tripped"                                                                                                                                                               |

Table 19-71: PWM\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | SRTRIPA    | Self-Restart Trip Status for Channel A. The PWM_STAT.SRTRIPA bit indicates whether the PWMChannel A self-restart has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit de- scription. |
| 5 (R/NW)           | SRTRIPA    | 0 Channel A Self-Restart Trip Status is "not tripped"                                                                                                                                                  |
| 5 (R/NW)           | SRTRIPA    | 1 Channel A Self-Restart Trip Status is "tripped"                                                                                                                                                      |
| 4 (R/W1C)          | FLTTRIPA   | Fault Trip Status for Channel A. The PWM_STAT.FLTTRIPA bit indicates whether the PWMChannel A fault has been tripped. For more information, see the PWM_TRIPCFG.MODE0A bit descrip- tion.              |
| 4 (R/W1C)          | FLTTRIPA   | 0 Channel A Fault Trip Status is "not tripped"                                                                                                                                                         |
| 4 (R/W1C)          | FLTTRIPA   | 1 Channel A Fault Trip Status is "tripped"                                                                                                                                                             |
| 3 (R/NW)           | RAWTRIP1   | Raw Trip 1 Status. The PWM_STAT.RAWTRIP1 bit indicates the raw input level for the PWMTRIP1 input.                                                                                                     |
| 3 (R/NW)           | RAWTRIP1   | 0 TRIP1 Level is Low                                                                                                                                                                                   |
| 3 (R/NW)           | RAWTRIP1   | 1 TRIP1 Level is High                                                                                                                                                                                  |
| 2 (R/NW)           | RAWTRIP0   | Raw Trip 0 Status. The PWM_STAT.RAWTRIP0 bit indicates the raw input level for the PWMTRIP0 input.                                                                                                     |
| 2 (R/NW)           | RAWTRIP0   | 0 TRIP0 Level is Low                                                                                                                                                                                   |
| 2 (R/NW)           | RAWTRIP0   | 1 TRIP0 Level is High                                                                                                                                                                                  |
| 1 (R/W1C)          | TRIP1      | Status bit set when TRIP1 is active low. The PWM_STAT.TRIP1 bit indicates whether the PWMTRIP1 fault has been trip- ped with an active-low input.                                                      |
| 1 (R/W1C)          | TRIP1      | 0 TRIP1 status is "not tripped"                                                                                                                                                                        |
| 1 (R/W1C)          | TRIP1      | 1 TRIP1 status is "tripped" (active low)                                                                                                                                                               |
| 0 (R/W1C)          | TRIP0      | Status bit set when TRIP0 is active low. The PWM_STAT.TRIP0 bit indicates whether the PWMTRIP0 fault has been trip- ped with an active-low input.                                                      |
| 0 (R/W1C)          | TRIP0      | 0 TRIP0 status is "not tripped"                                                                                                                                                                        |
| 0 (R/W1C)          | TRIP0      | 1 TRIP0 status is "tripped" (active low)                                                                                                                                                               |

## Sync Pulse Width Register

The PWM\_SYNC\_WID register selects the pulse width for the external sync pulse available on the PWM\_SYNC pin. The relation between the PWM\_SYNC\_WID register value and the pulse width (T PWM\_SYNC ) is give by the formula:

PWM\_SYNC\_WID = (TPWM\_SYNC / tCK) -1

For more information about applying the sync pulse width, see the PWM Functional Description section. Note that if the pulse width is changed in between sync pulses, the PWM applies the changed width on the next internal sync pulse. If, while the sync pulse is active, the chosen timer reaches its period boundary, the changed pulse width takes effect on that period boundary.

Figure 19-94: PWM\_SYNC\_WID Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000093_551efd5881cc9f7879ff34265b64269713cb67ae3cdbc1603456dd3372255706.png)

Table 19-72: PWM\_SYNC\_WID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/W)          | VALUE      | Sync Pulse Width. The PWM_SYNC_WID.VALUE bits select the pulse width for the external sync pulse available on the PWM_SYNC pin. |

## Timer 0 Period Register

The PWM\_TM0 register controls the switch period T SP of the PWMTMR0 timer. The PWM\_TM0 value is in units of t CK (the period of the peripheral clock) and the and is given by the formula:

PWM\_TM0= (TSP) / 2 x tCK

The value written to the register is effectively the number of t CK clock increments in half the period of the respective timer. For more information about applying the switch period, see the PWM Functional Description section. Note that PWM\_TM0 values of 0 and 1 are not defined and must not be used when the PWM is enabled.

Figure 19-95: PWM\_TM0 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000094_8ffbbd00729390e65a5ac8802cc2eccb206614ee81129bb78923121ca64aed91.png)

Table 19-73: PWM\_TM0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                         |
|--------------------|------------|-----------------------------------------------------------------|
| 15:0               | VALUE      | Timer PWMTMR0 Period Value.                                     |
| (R/W)              |            | The PWM_TM0.VALUE bits select the period for the PWMTMR0 timer. |

## Timer 1 Period Register

The PWM\_TM1 register controls the switch period (T SP of the PWMTMR1 timer. The PWM\_TM1 value is in units of t CK (the period of the peripheral clock) and the and is given by the formula:

PWM\_TM1= (TSP) / 2 x tCK

The value written to the register is effectively the number of t CK clock increments in half the period of the respective timer. For more information about applying the switch period, see the PWM Functional Description section. Note that PWM\_TM1 values of 0 and 1 are not defined and must not be used when the PWM is enabled.

Figure 19-96: PWM\_TM1 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000095_a87f2c3301da411e1166158a6c49c6b3490c1d9d4be3013ba707364e44c2ef89.png)

Table 19-74: PWM\_TM1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                         |
|--------------------|------------|-----------------------------------------------------------------|
| 15:0               | VALUE      | Timer PWMTMR1 Period Value.                                     |
| (R/W)              |            | The PWM_TM1.VALUE bits select the period for the PWMTMR1 timer. |

## Timer 2 Period Register

The PWM\_TM2 register controls the switch period (T SP of the PWMTMR2 timer. The PWM\_TM2 value is in units of t CK (the period of the peripheral clock) and the and is given by the formula:

PWM\_TM1= (TSP) / 2 x tCK

The value written to the register is effectively the number of t CK clock increments in half the period of the respective timer. For more information about applying the switch period, see the PWM Functional Description section. Note that PWM\_TM2 values of 0 and 1 are not defined and must not be used when the PWM is enabled.

Figure 19-97: PWM\_TM2 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000096_73f9d3ea917605229aa670a91255468845f782591e8beb1f94a247da194d6f56.png)

Table 19-75: PWM\_TM2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                         |
|--------------------|------------|-----------------------------------------------------------------|
| 15:0               | VALUE      | Timer PWMTMR2 Period Value.                                     |
| (R/W)              |            | The PWM_TM2.VALUE bits select the period for the PWMTMR2 timer. |

## Timer 3 Period Register

The PWM\_TM3 register controls the switch period (T SP of the PWMTMR3 timer. The PWM\_TM3 value is in units of t CK (the period of the peripheral clock) and the and is given by the formula:

PWM\_TM3= (TSP) / 2 x tCK

The value written to the register is effectively the number of t CK clock increments in half the period of the respective timer. For more information about applying the switch period, see the PWM Functional Description section. Note that PWM\_TM3 values of 0 and 1 are not defined and must not be used when the PWM is enabled.

Figure 19-98: PWM\_TM3 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000097_4c8597d52c6684230d1dc792acafe1bd48bb9f3a0ec7069ba3e700a3e2f89fdf.png)

Table 19-76: PWM\_TM3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                         |
|--------------------|------------|-----------------------------------------------------------------|
| 15:0               | VALUE      | Timer PWMTMR3 Period Value.                                     |
| (R/W)              |            | The PWM_TM3.VALUE bits select the period for the PWMTMR3 timer. |

## Timer 4 Period Register

The PWM\_TM4 register controls the switch period (T SP of the PWMTMR4 timer. The PWM\_TM4 value is in units of t CK (the period of the peripheral clock) and the and is given by the formula:

PWM\_TM4= (TSP) / 2 x tCK

The value written to the register is effectively the number of t CK clock increments in half the period of the respective timer. For more information about applying the switch period, see the PWM Functional Description section. Note that PWM\_TM4 values of 0 and 1 are not defined and must not be used when the PWM is enabled.

Figure 19-99: PWM\_TM4 Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000098_300e97bef01180a03fcfbcca192da1d83e47dd74dec6c226c7de5fb75df9ffac.png)

Table 19-77: PWM\_TM4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                         |
|--------------------|------------|-----------------------------------------------------------------|
| 15:0               | VALUE      | Timer PWMTMR4 Period Value.                                     |
| (R/W)              |            | The PWM_TM4.VALUE bits select the period for the PWMTMR4 timer. |

## Trip Configuration Register

The PWM\_TRIPCFG register configures Channel A, B, C, and D trip operation for trip inputs TRIP0 and TRIP1.

Figure 19-100: PWM\_TRIPCFG Register Diagram

![Image](22_Pulse-Width_Modulator_(PWM)_artifacts/image_000099_e5017103225671bd053b5c3af04758f39e7f45fa8e8f25bce6f74e72d513c19a.png)

Table 19-78: PWM\_TRIPCFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27 (R/W)           | MODE1D     | Mode of TRIP1 for Channel D. The PWM_TRIPCFG.MODE1D bit selects the trip mode of TRIP1 for Channel D. For more information, see the PWM_TRIPCFG.MODE0A bit description. 0 Fault Trip on TRIP1 Input 1 Self Restart on TRIP1 Input |
| 26 (R/W)           | EN1D       | Enable TRIP1 for Channel D. The PWM_TRIPCFG.EN1D bit enables TRIP1 as a trip source for Channel D. 0 Disable TRIP1 for ChannelD 1 Enable TRIP1 for ChannelD                                                                       |

Table 19-78: PWM\_TRIPCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | MODE0D     | Mode of TRIP0 for Channel D. The PWM_TRIPCFG.MODE0D bit selects the trip mode of TRIP0 for Channel D. For more information, see the PWM_TRIPCFG.MODE0A bit description.                                                           |
| 24 (R/W)           | EN0D       | Enable TRIP0 for Channel D. The PWM_TRIPCFG.EN0D bit enables TRIP0 as a trip source for Channel D. 0 Disable TRIP0 for ChannelD                                                                                                   |
| 19 (R/W)           | MODE1C     | Mode of TRIP1 for Channel C. The PWM_TRIPCFG.MODE1C bit selects the trip mode of TRIP1 for Channel C. For more information, see the PWM_TRIPCFG.MODE0A bit description. 0 Fault Trip on TRIP1 Input                               |
| 18 (R/W)           | EN1C       | Enable TRIP1 as a trip source for Channel C. The PWM_TRIPCFG.EN1C bit enables TRIP1 as a trip source for Channel C.                                                                                                               |
| 17 (R/W)           | MODE0C     | 0 Disable TRIP1 for Channel C 1 Enable TRIP1 for Channel C                                                                                                                                                                        |
|                    |            | Mode of TRIP0 for Channel C. The PWM_TRIPCFG.MODE0C bit selects the trip mode of TRIP0 for Channel C. For more information, see the PWM_TRIPCFG.MODE0A bit description. 0 Fault Trip on TRIP0 Input 1 Self Restart on TRIP0 Input |
| 16 (R/W)           | EN0C       | Enable TRIP0 for Channel C. The PWM_TRIPCFG.EN0C bit enables TRIP0 as a trip source for Channel C. 0 Disable TRIP0 for Channel C 1 Enable TRIP0 for Channel C                                                                     |
| 11 (R/W)           | MODE1B     | Mode of TRIP1 for Channel B. The PWM_TRIPCFG.MODE1B bit selects the trip mode of TRIP1 for Channel B. For more information, see the PWM_TRIPCFG.MODE0A bit description. 0 Fault Trip on TRIP1 Input                               |

Table 19-78: PWM\_TRIPCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | EN1B       | Enable TRIP1 for Channel B. The PWM_TRIPCFG.EN1B bit enables TRIP1 as a trip source for Channel B. 0 Disable TRIP1 for Channel B                                                                                                 |
| 9 (R/W)            | MODE0B     | Mode of TRIP0 for Channel B. The PWM_TRIPCFG.MODE0B bit selects the trip mode of TRIP0 for Channel B. For more information, see the PWM_TRIPCFG.MODE0A bit description.                                                          |
| 8 (R/W)            | EN0B       | Enable TRIP0 as a trip source for Channel B. The PWM_TRIPCFG.EN0B bit enables TRIP0 as a trip source for Channel B. 0 Disable TRIP0 for Channel B                                                                                |
| 3 (R/W)            | MODE1A     | 1 Enable TRIP0 for Channel B Mode of TRIP1 for Channel A. The PWM_TRIPCFG.MODE1A bit selects the trip mode of TRIP1 for Channel A. For more information, see the PWM_TRIPCFG.MODE0A bit description. 0 Fault Trip on TRIP1 Input |
| 2 (R/W)            | EN1A       | 1 Self Restart on TRIP1 Input Enable TRIP1 as a trip source for Channel A.                                                                                                                                                       |
|                    |            | The PWM_TRIPCFG.EN1A bit enables TRIP1 as a trip source for Channel A. 0 Disable TRIP1 for Channel A                                                                                                                             |

Table 19-78: PWM\_TRIPCFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MODE0A     | Mode of TRIP0 for Channel A. The PWM_TRIPCFG.MODE0A bit selects the trip mode of TRIP0 for Channel A. In fault-trip mode ( PWM_TRIPCFG.MODE0A =0), after the input is tripped, the trip status appears in the corresponding channels fault-trip status bit (for example, PWM_STAT.FLTTRIPA ), and the PWMimmediately shuts down outputs of that channel. After a fault trip occurs, when the trip condition is no longer active, the pro- cessor may cause channel outputs to resume by completing a write-1-to-clear the corre- sponding fault-trip status bit. The raw (input level) trip input state is available from the PWM_STAT.RAWTRIP0 and PWM_STAT.RAWTRIP0 bits. In self-restart mode ( PWM_TRIPCFG.MODE0A =1), after the input is tripped, the trip status appears in the corresponding channels self-restart status bit (for example, PWM_STAT.SRTRIPA ), and the PWMimmediately shuts down outputs of that channel. On the next timer period boundary (of the PWMTMRx used by that chan- nel), if the trip condition is not active, the PWMclears the status and restarts the channels output. | Mode of TRIP0 for Channel A. The PWM_TRIPCFG.MODE0A bit selects the trip mode of TRIP0 for Channel A. In fault-trip mode ( PWM_TRIPCFG.MODE0A =0), after the input is tripped, the trip status appears in the corresponding channels fault-trip status bit (for example, PWM_STAT.FLTTRIPA ), and the PWMimmediately shuts down outputs of that channel. After a fault trip occurs, when the trip condition is no longer active, the pro- cessor may cause channel outputs to resume by completing a write-1-to-clear the corre- sponding fault-trip status bit. The raw (input level) trip input state is available from the PWM_STAT.RAWTRIP0 and PWM_STAT.RAWTRIP0 bits. In self-restart mode ( PWM_TRIPCFG.MODE0A =1), after the input is tripped, the trip status appears in the corresponding channels self-restart status bit (for example, PWM_STAT.SRTRIPA ), and the PWMimmediately shuts down outputs of that channel. On the next timer period boundary (of the PWMTMRx used by that chan- nel), if the trip condition is not active, the PWMclears the status and restarts the channels output. |
| 1 (R/W)            | MODE0A     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Fault Trip on TRIP0 Input                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/W)            | MODE0A     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Self Restart on TRIP0 Input                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | EN0A       | Enable TRIP0 as a trip source for Channel A.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Enable TRIP0 as a trip source for Channel A.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | EN0A       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Disable TRIP0 for Channel A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | EN0A       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Enable TRIP0 for Channel A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |