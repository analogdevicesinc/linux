## 21   Watchdog Timer (WDOG)

The processor has a 32-bit watchdog timer that can be used to verify system reliability by generating an event to the processor core when the watchdog expires before software updates it.

For the ADSP-SC58x processors, a system typically assigns WDG0 to Core ID=0 and WDG1 to Cores ID=1 and ID=2. The system clock (SCLK0\_0) clocks the watchdog timers.

## WDOG Features

The watchdog timer has the following features:

- Programmable 32-bit watchdog count value
- 8-bit disable bit pattern
- General-purpose core event generation

The watchdog timer can supervise system software stability by periodically reloading it to prevent expiration of the downward-counting timer (such that the count never becomes 0). When used in this fashion, an expiring timer can indicate the system software is not running normally.

Expiration of the WDOG counter generates a general-purpose interrupt, which can be used in a variety of ways:

- as an interrupt vector sent through the System Event Controller (SEC) to the core to be serviced by a handler function, providing full software control of device resources (for example, GPIO management and reset control).
- as a fault condition through the SEC to provide hardware-automated:
- signalling of the fault condition on external pins to the system,
- system reset requests to the Reset Control Unit (RCU), and/or
- trigger outputs (SEC0\_FAULT trigger master) through the Trigger Routing Unit (TRU) to initiate activities in a variety of potential trigger slaves (for example, GPIO control).

To facilitate debugging, the watchdog timer does not decrement (even if enabled) when the processor is in emulation mode.

## WDOG Functional Description

When enabled, the 32-bit watchdog timer counts downward every SCLK0\_0 cycle. If it reaches zero, the watchdog expiration event is generated, which can be used in many ways.

To start the watchdog timer:

1. Program the watchdog timeout period (in SCLK0\_0 cycles) in the WDOG\_CNT register. With the watchdog disabled, this write also preloads the WDOG\_STAT register.
2. Enable the watchdog timer by writing any value other than 0xAD to the WDOG\_CTL.WDEN field.

Once the watchdog is enabled, writes to the WDOG\_CNT register are ignored. The counter begins decrementing, and the current counter value can be read from the 32-bit WDOG\_STAT register at any time.

To prevent the counter from expiring, software must "kick" the watchdog by writing any value to the WDOG\_STAT register while the current count is non-zero. While the value written is irrelevant and ignored, this action resets the current counter in the WDOG\_STAT register to the programmed WDOG\_CNT value, and decrementing continues. The internal counter continues decrementing until it reaches zero, at which point the expiration event is generated, and the WDOG\_CTL.WDRO rollover bit is set.

Watchdog operation continues in this manner unless disabled by explicitly writing 0xAD to the WDOG\_CTL.WDEN field.

The watchdog expiration event can be used in a variety of ways, as listed below.

- The watchdog expiration event itself is one of numerous interrupt sources that is managed by the System Event Controller. Like other peripheral sources, this event can be used to cause a vector to a handler function that executes based on interrupt priority.
- The watchdog expiration event can be used to initiate automated hardware response through the SEC Fault Interface (SFI).

For this, the WDOG expiry has to be configured as the fault source in the SEC. Then the response to the WDOG expiry can be set to any of the below:

- Signalling through the external fault pin.
- System reset
- Trigger outputs to numerous potential trigger slaves.

For further details on how watchdog expiration event can be used with the SFI, see Configuring the WDOG Expiry Event to Issue a System Reset.

## ADSP-SC58x WDOG Register List

The Watchdog Timer unit (WDOG) provides a software-based watchdog timer that can improve system reliability by generating an event to the processor core if the watchdog expires before being updated by software. A set of registers governs WDOG operations. For more information on WDOG functionality, see the WDOG register descriptions.

Table 21-1: ADSP-SC58x WDOG Register List

| Name      | Description                    |
|-----------|--------------------------------|
| WDOG_CNT  | Count Register                 |
| WDOG_CTL  | Control Register               |
| WDOG_STAT | Watchdog Timer Status Register |

## ADSP-SC58x WDOG Interrupt List

Table 21-2: ADSP-SC58x WDOG Interrupt List

|   Interrupt ID | Name      | Description      | Sensitivity   | DMA Channel   |
|----------------|-----------|------------------|---------------|---------------|
|              3 | WDOG0_EXP | WDOG0 Expiration | Level         |               |
|              4 | WDOG1_EXP | WDOG1 Expiration | Level         |               |

## WDOG Block Diagram

The Watchdog Timer Block Diagram figure shows the detailed block diagram for the watchdog timer.

Figure 21-1: Watchdog Timer Block Diagram

![Image](24_Watchdog_Timer_(WDOG)_artifacts/image_000000_aba53a4e5bdced757d63d79398f437474103731eaeac4403f1b7ce2934d7a0ba.png)

## Internal Interface

The system clock (SCLK0\_0) clocks the watchdog timer. The registers are accessed through the 16-bit peripheral MMR access bus. 32-bit read/write operations always access the 32-bit WDOG\_CNT and WDOG\_STAT registers. Hardware ensures that those accesses are atomic. When the counter expires, the WDOG expiration event is generated.

## External Interface

The watchdog timer does not directly interact with any external pins.

## ADSP-SC58x WDOG Register Descriptions

Watchdog Timer Unit (WDOG) contains the following registers.

Table 21-3: ADSP-SC58x WDOG Register List

| Name      | Description                    |
|-----------|--------------------------------|
| WDOG_CNT  | Count Register                 |
| WDOG_CTL  | Control Register               |
| WDOG_STAT | Watchdog Timer Status Register |

## Count Register

The WDOG\_CNT register holds the programmable, unsigned count value. A valid write to this register also pre-loads the WDOG counter. For added safety, the WDOG\_CNT register can be updated only when the WDOG timer is disabled. A write to the WDOG\_CNT register while the timer is enabled does not modify the contents of this register. This register must be accessed with 32-bit read/writes only.

Figure 21-2: WDOG\_CNT Register Diagram

![Image](24_Watchdog_Timer_(WDOG)_artifacts/image_000001_7c295d33f818eed91eb2314ed368fa07b98b7f150ca619b6a79e3afbc1315753.png)

Table 21-4: WDOG\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 31:0               | VALUE      | Count Value.                                                               |
| (R/W)              |            | The WDOG_CNT.VALUE bit field holds the programmable, unsigned count value. |

## Control Register

The WDOG\_CTL register controls the watchdog timer. This register supports enabling/disabling the watchdog timer and supports checking the timer rollover status. Note that when the processor is in emulation mode, the watchdog timer counter will not decrement even if it is enabled.

Figure 21-3: WDOG\_CTL Register Diagram

![Image](24_Watchdog_Timer_(WDOG)_artifacts/image_000002_4f1fe02516c91c9d78b8a731d6542b85846f64a5dd1184a0485452ee1d10b86b.png)

Table 21-5: WDOG\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | WDRO       | Watch Dog Rollover. Software can determine whether the timer has rolled over by interrogating the WDOG_CTL.WDRO status bit. This is a sticky bit that is set whenever the watch dog timer count reaches 0 and cleared only by disabling the watch dog timer and then writing a 1 to the bit. | Watch Dog Rollover. Software can determine whether the timer has rolled over by interrogating the WDOG_CTL.WDRO status bit. This is a sticky bit that is set whenever the watch dog timer count reaches 0 and cleared only by disabling the watch dog timer and then writing a 1 to the bit. |
| 11:4 (R/W)         | WDEN       | Watch Dog Enable. The WDOG_CTL.WDEN field is used to enable and disable the watchdog timer. Writ- ing any value other than the disable value into this field enables the watchdog timer. This multi-bit disable key minimizes the chance of inadvertently disabling the watch- dog timer.    | Watch Dog Enable. The WDOG_CTL.WDEN field is used to enable and disable the watchdog timer. Writ- ing any value other than the disable value into this field enables the watchdog timer. This multi-bit disable key minimizes the chance of inadvertently disabling the watch- dog timer.    |
| 11:4 (R/W)         | WDEN       | 173                                                                                                                                                                                                                                                                                          | Counter Disabled. All other values mean that the coun- ter is enabled.                                                                                                                                                                                                                       |

## Watchdog Timer Status Register

The WDOG\_STAT register contains the current count value of the watchdog timer. Reads of this register return the current count value. When the watchdog timer is enabled, the WDOG\_STAT register is decremented by 1 on each SCLK0\_0 cycle. When the count value reaches 0, the watchdog timer stops counting, and the expiry event is generated. The WDOG\_STAT register is a 32-bit unsigned system MMR that must be accessed with 32-bit reads and writes.

Values cannot be stored directly in this register but are instead copied from the WDOG\_CNT register. This copy process can happen in two ways:

- While the watchdog timer is disabled, writing the WDOG\_CNT register pre-loads the WDOG\_STAT register.
- While the watchdog timer is enabled, writing the WDOG\_STAT register loads it with the value in the WDOG\_CNT register.
- While the watchdog timer is disabled, writing to the WDOG\_STAT register also reloads it with the value in the WDOG\_CNT register.

When the processor executes a write (of an arbitrary value) to the WDOG\_STAT register, the value in the WDOG\_CNT register is copied into the WDOG\_STAT register. Typically, software sets the value of WDOG\_CNT at initialization, then periodically writes to the WDOG\_STAT register before the watchdog timer expires. This reloads the watchdog timer with the value from WDOG\_CNT and prevents generation of the expiry event.

If the program does not reload the counter before SCLK0\_0 x count register cycles, an expiry event is generated, and the WDOG\_CTL.WDRO bit is set. When this happens, the counter stops decrementing and remains at zero. If the counter is enabled with a zero loaded to the counter, the WDOG\_CTL.WDRO bit is set immediately and the counter remains at zero and does not decrement.

Figure 21-4: WDOG\_STAT Register Diagram

![Image](24_Watchdog_Timer_(WDOG)_artifacts/image_000003_628c0b1c3388b727c66490a63b1db7e3006750cd33a949321ef3087477adc1ec.png)

Table 21-6: WDOG\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Current Count Value (Status). The WDOG_STAT.VALUE bit field contains the current count value of the watchdog timer. |