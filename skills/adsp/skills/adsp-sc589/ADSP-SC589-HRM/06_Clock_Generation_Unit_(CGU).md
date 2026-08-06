## 3   Clock Generation Unit (CGU)

The Clock Generation Unit (CGU) includes the phase locked loop (PLL) and the PLL control unit (PCU). The PLL generates a master clock that runs at a frequency that is a multiple of the CLKIN input clock frequency. The PCU divides down the master clock to generate various system clocks and synchronization signals.

## CGU Features

The CGU module supports the following features:

- Provides smooth transitions from the current clock condition to a new condition with PLL logic and executes the changes to clocks due to register programming
- Provides PLL and clock domain status reporting for event management
- Supports the capability to bypass the PLL for power savings
- Manages power dynamically through software, allowing the dynamic control of the core clock frequency (f CCLK) of the processor
- Controls clock gating of core and system clocks

NOTE: For more information about processor-specific CGU features, see the processor data sheet.

## CGU Functional Description

The CGU generates all on-chip clocks and synchronization signals based on the programmed PLL multiplication factor and dividers. The CGU provides the following functionality.

## Change the PLL clock frequency

The CGU allows programs to change the PLL clock frequency by writing new values to bits in the control register. Any time the PLL relocks, the CGU aligns all core and system clocks.

## Change other clock frequencies

The CGU allows programs to change the CCLKn, SYSCLK, SCLKn, DCLK, and OCLK frequencies by writing values to the CGU\_DIV register. Any time the clock frequency is changed, the OCLK, CCLKn, SYSCLK, DCLK and SCLKn clocks exit the frequency change sequence aligned.

## Perform clock alignment

The CGU can align all clocks by writing to the CGU\_DIV register. This function aligns all PLL-based clocks.

For more information on these functions, see the CGU Programming Model section.

## ADSP-SC58x CGU Register List

The clock generation unit (CGU) includes the phase locked loop (PLL) and the PLL control unit (PCU). The PLL generates a clock, running at a frequency that is a multiple of the CLKIN input clock's frequency. The CGU also generates all on-chip clocks and synchronization signals. The PCU permits application software control of the PLL's operation. A set of registers govern CGU operations. For more information on CGU functionality, see the CGU register descriptions.

Table 3-1: ADSP-SC58x CGU Register List

| Name          | Description                                      |
|---------------|--------------------------------------------------|
| CGU_CCBF_DIS  | Core Clock Buffer Disable Register               |
| CGU_CCBF_STAT | Core Clock Buffer Status Register                |
| CGU_CLKOUTSEL | CLKOUT Select Register                           |
| CGU_CTL       | Control Register                                 |
| CGU_DIV       | Clocks Divisor Register                          |
| CGU_OSCWDCTL  | Oscillator Watchdog Register                     |
| CGU_PLLCTL    | PLL Control Register                             |
| CGU_REVID     | Revision ID Register                             |
| CGU_SCBF_DIS  | System Clock Buffer Disable Register             |
| CGU_SCBF_STAT | System Clock Buffer Status Register              |
| CGU_STAT      | Status Register                                  |
| CGU_TSCOUNT0  | Time Stamp Counter 32 LSB Register               |
| CGU_TSCOUNT1  | Time Stamp Counter 32 MSB Register               |
| CGU_TSCTL     | Time Stamp Control Register                      |
| CGU_TSVALUE0  | Time Stamp Counter Initial 32 LSB Value Register |
| CGU_TSVALUE1  | Time Stamp Counter Initial MSB Value Register    |

## ADSP-SC58x CGU Interrupt List

Table 3-2: ADSP-SC58x CGU Interrupt List

|   Interrupt ID | Name     | Description   | Sensitivity   | DMA Channel   |
|----------------|----------|---------------|---------------|---------------|
|              1 | CGU0_EVT | CGU0 Event    | Edge          |               |
|              2 | CGU1_EVT | CGU1 Event    | Edge          |               |

## ADSP-SC58x CGU Trigger List

Table 3-3: ADSP-SC58x CGU Trigger List Masters

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|            1 | CGU0_EVT | CGU0 Event    | Edge          |
|            2 | CGU1_EVT | CGU1 Event    | Edge          |

Table 3-4: ADSP-SC58x CGU Trigger List Slaves

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## CGU Definitions

## DPM

The dynamic power management (DPM) works with the CGU to provide flexible power dissipation modes for the processor.

## PCU

The PLL control unit (PCU) in the CGU controls PLL operations. All the MMR registers of the CGU are implemented in this unit.

## PLL

The phase-locked loop (PLL) operates within the CGU.

## RCU

The reset control unit (RCU) provides input to the CGU to manage clocks during processor reset.

## CDU

The clock distribution unit distributes the clocks from the CGU to different clock domains

## CGU

The clock generation unit (CGU) is comprised of the PLL and PCU. The CGU generates the clocks listed in the Clock Descriptions table.

Table 3-5: Clock Descriptions

| Clock    | Description              |
|----------|--------------------------|
| CCLK0_0  | CCLK0 derived from CGU0  |
| CCLK1_0  | CCLK1 derived from CGU0  |
| SYSCLK_0 | SYSCLK derived from CGU0 |
| SCLK0_0  | SCLK0 derived from CGU0  |
| SCLK1_0  | SCLK1 derived from CGU0  |
| DCLK_0   | DCLK derived from CGU0   |
| OCLK_0   | OCLK derived from CGU0   |
| CCLK0_1  | CCLK0 derived from CGU1  |
| CCLK1_1  | CCLK1 derived from CGU1  |
| SYSCLK_1 | SYSCLK derived from CGU1 |
| SCLK0_1  | SCLK0 derived from CGU1  |
| SCLK1_1  | SCLK1 derived from CGU1  |
| DCLK_1   | DCLK derived from CGU1   |
| OCLK_1   | OCLK derived from CGU1   |

## CGU PLL Block Diagram

The CGU PLL Block Diagram provides a top-level block diagram of the phase locked loop (PLL). The main blocks of the PLL are the phase/frequency detector (PFD), the charge pump, the loop filter, and the voltage controlled oscillator (VCO). The VCO multiplies the SYS\_CLKINx input to a higher frequency.

Figure 3-1: CGU PLL Block Diagram (x=0 CGU0, x=1 CGU1)

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000000_4df626bb89e070a57f9ecf867828c2bc0dcd954515674f1476da52bc7ada528e.png)

The SYS\_CLKOUT Generation figure is a conceptual representation of the CLKOUT module. Different clocks that originate from the CGU blocks are available on the SYS\_CLKOUT output pin. The selection of the clock output on the SYS\_CLKOUT pin is controlled by the CGU\_CLKOUTSEL.CLKOUTSEL bit field.

Additional configuration options are configured using the Clock Distribution Unit. See CDU Clock Configuration Options.

Figure 3-2: SYS\_CLKOUT Generation

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000001_2ab7d423a76d3b980194dc534c2d04c0e47a2ee7b3ec659f5be57de633d88f79.png)

NOTE: The processor supports two PLLs (CGU1-0), see the CDU Block Diagram figure in the Clock Distribution (CDU) chapter. The configuration of CGU0 is mandatory because it provides the clock to the chip infrastructure (Fabric, L2 cache, L2 system, SEC and GIC) and to some high speed peripherals.

## CGU Operating Modes

The CGU does not have configurable operating modes, but CGU operations affect the operating modes of other modules. Some CGU operation issues that affect operation of other modules include the following:

- The PLL of the CGU operates in either normal mode (CGU clock divisors applied) or bypass mode (CGU PLL is bypassed and clock divisors ignored).
- The SCB uses the CGU for clock synchronization across clock domains. For more information, see System Crossbars (SCB).
- The DPM uses the CGU for clock management as power state transitions occur. For more information, see the Dynamic Power Management (DPM) chapter.
- The CGU uses clock gating control to obtain flexible low-power modes.

## CGU Power-up Sequence

See the product data sheet for exact power-up requirements. The processor is configured to come up in clock bypass mode. The programs is required to configure full speed clocks and safety monitors. CLKIN0 and all supplies should be stable before the SYS\_HWRST signal is deasserted. CLKIN1 should be stable before operation of OCU safety unit.

## CGU Event Control

The CGU generates an event or error for several different reasons.

## CGU Events

After a frequency change, a CGU event indicates that the PLL has locked and clocks are synchronized. If a core was idled while changing frequencies, the CGU can use an event interrupt to break the core idle. While in active mode, a CGU event indicates that the PLL has locked.

## CGU Errors

A CGU error occurs under following conditions:

- A write access to the CGU\_DIV register triggers an alignment sequence while the PLL is locked and is still aligning the clocks.

The CGU\_STAT.WDIVERR bit state indicates this error. If this error occurs, clear the CGU\_STAT.WDIVERR bit and rewrite the desired values to the CGU\_DIV register.

- A change to the CGU\_DIV register occurs while the PLL is locked and is still aligning the clocks

The CGU\_STAT.WDIVERR bit state indicates this error. If this error occurs, clear the CGU\_STAT.WDIVERR bit and rewrite the desired values to the CGU\_DIV register.

- A write access to the CGU\_CTL.DF bit field occurs or a write access to the CGU\_CTL.MSEL bit field occurs while the PLL is locking.

The CGU\_STAT.WDFMSERR bit state indicates this error. If this error occurs, wait until the PLL has finished locking, clear the error, and rewrite the desired value change.

- A clock divisor value error occurs when the CCLK divisor is greater than the SYSCLK divisor. For example, the CGU\_DIV.CSEL is greater than CGU\_DIV.SYSSEL .

The CGU\_STAT.WDIVERR bit state indicates this error. If this error occurs, clear it. The CGU writes new values to the CGU\_DIV.CSEL bit field, so the field is less than or equal to the CGU\_DIV.SYSSEL bit field value.

The CGU monitors changes to the following fields:

- CCLK Divisor -CGU\_DIV.CSEL
- SYSCLK Divisor -CGU\_DIV.SYSSEL

## CGU Generated Bus Errors

The CGU generates a bus error when a read or write transaction is attempted to an unused address within the CGU address range. It also generates a bus error when a misaligned access is made to a CGU register. In addition to the bus error, the CGU\_STAT.ADDRERR bit is set. If a write to a write-protected CGU register is attempted, the CGU generates a bus error. In addition, the CGU\_STAT.LWERR bit is set.

## Oscillator Watchdog

The oscillator watchdog detects the absence of input clock transitions and provides a fault warning through the SYS\_FAULT pin. To detect harmonic or subharmonic crystal oscillator behavior, the watchdog (under programmable control) also detects and reports input oscillator frequencies above and below the specified limits. Use an internal asynchronous, local 1-MHz oscillator combined with a series of programmable counters for this detection. Set the CGU\_OSCWDCTL.MONDIS bit and clear the CGU\_OSCWDCTL.FAULTEN bit in the control register to optionally disable all the input clock monitor and fault detection functions.

Set the CGU\_OSCWDCTL.HODEN bit to enable harmonic oscillation detection. The CGU uses the CGU\_OSCWDCTL.HODF bit field to indicate the desired lower fail limit for the harmonic oscillation detection in MHz. The upper limit is always twice the lower limit. The HODF Settings for Different Input Clock Frequencies table shows an example of the CGU\_OSCWDCTL.HODF bit settings for different input clock frequencies.

Table 3-6: HODF Settings for Different Input Clock Frequencies

|   CGU_OSCWDCTL. HODF [5:0] |   Subharmonic Fre- quency (MHz) |   Nom. Lower Fail Limit (MHz) |   Input Clock Fre- quency (MHz) |   Nom. Upper Fail Limit (MHz) |   Second Harmonic Frequency (MHz) |
|----------------------------|---------------------------------|-------------------------------|---------------------------------|-------------------------------|-----------------------------------|
|                         14 |                              10 |                            14 |                              20 |                            28 |                                40 |
|                         21 |                              15 |                            21 |                              30 |                            42 |                                60 |

The CGU uses the CGU\_OSCWDCTL.BOUF asynchronous control bit field to indicate the desired upper fail limit for the bad oscillation detection. Set the CGU\_OSCWDCTL.BOUEN bit to enable upper-limit bad oscillation detection. A bad oscillation detection condition signals a fault before any processor operations occur. This detection occurs (even in bypass mode) whenever a clock frequency exceeds its specifications.

The CGU\_OSCWDCTL.BOUF =0 operation starts with a target of 32 MHz and each additional LSB increases the frequency test limit by 2 MHz. For example:

Target Upper Frequency Limit = CGU\_OSCWDCTL.BOUF × 2 MHz + 32 MHz

The CGU\_STAT.OSCWDSTATFC status bits indicate the nature of the fault. The Fault Map table shows the fault values.

Table 3-7: Fault Map

|   CGU_STAT.OSCWDSTATFC Bitfield Values | Fault Type                      |
|----------------------------------------|---------------------------------|
|                                      0 | No Fault                        |
|                                      1 | No Input Clock                  |
|                                      2 | Subharmonic CLKIN               |
|                                      3 | Harmonic CLKIN                  |
|                                      4 | No AUX_CLK                      |
|                                      5 | CLKIN > Upper Freq Limit (BOUF) |
|                                      6 | Reserved                        |
|                                      7 | Multiple Limit Faults           |

There is a priority to the faults given in the case of multiple fault errors. The highest priority is given to No Input clock followed by No AUX\_CLK. The other three fault cases share the lowest priority. Multiple limit faults are asserted if more than one type of subharmonic CLKIN, harmonic CLKIN, or BOUF faults are observed.

NOTE: All the CGU\_STAT.OSCWDSTATFC faults other than the absence of AUX\_CLK (for example, CGU\_STAT.OSCWDSTATFC =4) are not reliable and used for debug only.

NOTE: Program and enable the OSCWDOG to match the actual crystal, before bringing the PLL out of bypass.

## CGU Programming Model

The programming model for the CGU involves the various mode configuration techniques.

## Configuring CGU Modes

Use the following procedures to configure the clocks and PLL.

NOTE: The program needs to perform the following sequence only once, after coming out of reset, inside the application, before changing clocks. This sequence clears the CGU\_STAT.CLKSALGN bit:

```
*pREG_CGU0_PLLCTL |= BITM_CGU_PLLCTL_PLLBPCL; // come out of bypass and enter Full ON while( (pADI_CGU0 ->STAT & 0xF) != 0x5 ) { } // poll // now clocks are running with hardware default divisors. // now program can change frequencies If desired the program can put the PLL again into bypass.
```

## Changing Clock Frequencies

Applications change clock frequencies in two ways. The first way is modifying the PLL multiplication value by writing to the CGU\_CTL register and the second is modifying the clock dividers by writing to the CGU\_DIV register. Both actions have different implications even if the frequencies of the final clock are the same. Write accesses to change the CGU\_CTL.DF or CGU\_CTL.MSEL bit fields while the PLL is locking set the CGU\_STAT.WDFMSERR error bit. The CGU\_STAT.WDIVERR error bit is set when one of following accesses is attempted while the PLL is locked, but still aligning the clocks:

- A write access to the CGU\_DIV register to trigger an alignment sequence
- A write access to the CGU\_DIV register to change the CGU\_DIV.CSEL , CGU\_DIV.SYSSEL , CGU\_DIV.S0SEL , CGU\_DIV.S1SEL , or CGU\_DIV.DSEL bits

Read-after-write accesses to these registers return the new value, even if the frequency of the clock change is still inprogress.

Modifying the PLL multiplier requires the PLL to relock. Once the PLL locks, the CGU synchronizes the clocks. Changes to the CGU\_CTL.DF or CGU\_CTL.MSEL bit field result in bypassing the PLL. By setting the CGU\_CTL.WFI bit, programs force the PLL to wait for all the cores to return to their idle or reset states before the frequency changes. If necessary, clear the CGU\_DIV.UPDT bit to avoid multiple clock alignment sequences. If the CGU\_DIV register is not updated, the CGU uses the current values to determine the frequencies of the clock. It is the programs responsibility to guarantee that the new CGU\_CTL.DF or CGU\_CTL.MSEL and CGU\_DIV combinations are legal.

## Changing the PLL Clock Frequency

To change the phase-locked loop clock ( PLLCLK ) frequency, write new values to the CGU\_CTL.MSEL field or CGU\_CTL.DF field. Any time the PLL relocks, all core and system clocks are aligned.

1. Read CGU\_STAT register and verify that: a. The CGU\_STAT.PLLEN bit =1 (PLL enabled) b. The CGU\_STAT.PLOCK bit =1 (PLL is not locking) c. The CGU\_STAT.CLKSALGN bit =0 (clocks aligned)
2. Write the desired values to the clock divisor select fields of the CGU\_DIV register with the CGU\_DIV.UPDT bit =0.
3. Write the desired values to the CGU\_CTL.DF and CGU\_CTL.MSEL fields.

- a. To change the PLL frequency while the cores are idle, write to the CGU\_CTL register with the CGU\_CTL.WFI bit =1.
- b. To change the PLL frequency while the cores are active, write to the CGU\_CTL register with the CGU\_CTL.WFI bit =0.

This sequence performs these actions:

1. Updates the corresponding CGU registers
2. Bypasses the PLL
3. Makes the PLL lock to the new values in the CGU\_CTL.MSEL or CGU\_CTL.DF fields
4. Changes the clock frequencies
5. Exits the PLL bypass with all clocks aligned

When exiting the PLL bypass state, a CGU event occurs.

The CGU\_STAT register exits this sequence with the CGU\_STAT.PLLEN bit =1, the CGU\_STAT.PLOCK bit =1, the CGU\_STAT.PLLBP bit =0, and the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.PLOCK bit, CGU\_STAT.PLLBP bit, and CGU\_STAT.CLKSALGN bit to discover when the PLL is locked and the clocks are aligned.

Changing the frequency of the PLL is allowed while the PLL is bypassed. But, the new PLLCLK frequency is not used until the PLL is no longer bypassed.

## Changing the CCLKn or SYSCLK Frequency Without Modifying the PLLCLK Frequency

To change the clock frequencies, write new values to CGU\_DIV.CSEL or CGU\_DIV.SYSSEL bits. The frequency change occurs only when the PLL is not bypassed. Any time the CCLKn or SYSCLK clock frequencies are changed, they exit the frequency change sequence aligned.

1. Read the CGU\_STAT register to verify that the CGU\_STAT.CLKSALGN bit =0 (clocks aligned).
2. Write the desired CGU\_DIV.CSEL , CGU\_DIV.SYSSEL , and CGU\_DIV.OSEL bit field values with the CGU\_DIV.UPDT bit = 1.

ADDITIONAL INFORMATION: This write updates the CGU\_DIV register, changes the SCLKn and SYSCLK frequencies, and aligns the clocks. When the clocks are aligned, a CGU event occurs.

The CGU\_STAT register exits this sequence with the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.CLKSALGN bit to discover when the clocks are aligned. Any write attempt to change the CGU\_DIV.S0SEL or CGU\_DIV.S1SEL bit fields while CGU\_STAT.CLKSALGN bit =1 (clocks alignment in progress) triggers an MMR access bus error and the CGU\_DIV register is not modified.

Programming the SYSCLK frequency to a higher value than CCLKn also triggers an MMR access bus error and the CGU\_DIV register is not modified.

Writing to the CGU\_DIV register is allowed while the processor is in active (PLL bypassed) mode. But, the effect of the write is visible only after the transition to full-on (PLL not bypassed) mode.

Accessing the DDR memory while changing the SYSCLK frequency is not supported and can have unpredictable results.

## Changing the OCLK Frequency

To change the OCLK clock frequency, write a new CGU\_DIV.OSEL bit value. Any time the OCLK clock frequency is changed, the OCLK , CCLKn , SYSCLK , and SCLKn clocks exit the frequency change sequence aligned.

1. Read the CGU\_STAT register to verify that the CGU\_STAT.CLKSALGN bit =0 (clocks aligned).
2. Write the desired CGU\_DIV.OSEL value with the CGU\_DIV.UPDT bit =1. ADDITIONAL INFORMATION: This write updates the CGU\_DIV register, changes the OCLK frequency, and aligns all clocks except OCLK .

The CGU\_STAT register exits this sequence with the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.CLKSALGN bit to discover when the clocks are aligned. Any write attempt to change the CGU\_DIV.DSEL field while the CGU\_STAT.CLKSALGN bit =1 (clock alignment in progress) triggers an MMR access bus error and the CGU\_DIV register is not modified. When the clocks are aligned, a CGU event occurs.

Writing to the CGU\_DIV.OSEL bit field is allowed while the processor is in active (PLL bypassed) mode. But, the effect of the write is visible only after the transition to full-on (PLL not bypassed) mode.

## Aligning All Clocks

To align the clocks, write 1 to the CGU\_DIV.ALGN bit. The frequency can be changed, if necessary. The clocks aligned include:

- CCLKn
- SYSCLK
- SCLKn
- DCLK
- OCLK
1. Read the CGU\_STAT register to verify that CGU\_STAT.CLKSALGN bit =0 (clocks aligned).
2. Write 1 to the CGU\_DIV.ALGN bit. All other fields can change.

ADDITIONAL INFORMATION: This write does not alter the CGU\_DIV register unless one of the clock-select fields is modified. When the clocks are aligned, a CGU event occurs.

The CGU\_STAT register exits this sequence with the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.CLKSALGN bit to discover when the clocks are aligned. Any write to the CGU\_DIV register intended to align clocks or to change a clock select field while the CGU\_STAT.CLKSALGN bit =1 (clocks alignment in progress) triggers an MMR access bus error. And, the CGU\_DIV register is not modified.

Writing 1 to the CGU\_DIV.ALGN bit has no effect while the processor is in active (PLL bypassed) mode.

The CGU does not support accessing the DDR memory while changing the SYSCLK or DCLK frequencies. This type of access can have unpredictable results.

## Shutting Off CCLKn From Another Master

CCLKn can be shut off to save power when it is not in use.

1. Disable interrupts to core n.
2. Set the RCU\_SIDIS.SI[n] bit to disable the interfaces of core n in order to:
- a. Stop DMA accesses to its L1.
- b. Stop accesses to memory to core 0.
- c. Stop accesses to MMRs.
3. Test the RCU\_SISTAT.SI[n] bit to detect when accesses to core n have been disabled and all the pending transactions have completed.
4. Set the CGU\_CCBF\_DIS.CCBF0 bit to disable the CCLKn buffer.
5. Check the CGU\_CCBF\_STAT.CCBF0 bit.

If the CGU\_CCBF\_STAT.CCBF0 bit is set, continue.

## Reenable CCLKn From Another Master

1. Clear the CGU\_CCBF\_DIS.CCBF0 to enable CCLKn .
2. Check the CGU\_CCBF\_STAT.CCBF0 bit.
- a. If the CGU\_CCBF\_DIS.CCBF0 bit is cleared, continue.
3. Clear the RCU\_SIDIS.SI[n] bit. The core deasserts its acknowledge signal in response to the RCU\_SYSRST0 signal. This operation clears the RCU\_SISTAT.SI[n] bit.

## Valid Clock Multiplier Settings

Processor operations depend on valid settings in the CGU\_CTL and CGU\_DIV registers. These registers control the clock multiplier and divisor values. Set these registers such that the minimum and maximum clocks specified in the data sheet are not violated. All other clock specifications in the data sheet must also be adhered to for correct operation of the processor.

NOTE: The frequency of any processor core clock to the SYSCLK is either 1:1 or 2:1 only.

## PLL Bypass and PLL Disable

Writing 1 to the CGU\_PLLCTL.PLLBPST bit tells the PLL to apply OSC\_CLKIN clock to CCLK , SYSCLK , SCLK0 , SCLK1 , DCLK (PLL Bypass), and OCLK outputs. Writing 1 to the CGU\_PLLCTL.PLLBPCL bit tells the PLL to exit its PLL Bypass state and make all output clocks align and transition to their programmed frequencies.

The PLL can be disabled by clearing CGU\_PLLCTL.PLLEN bit while in the bypass state. If necessary, clock buffers can be disabled. CCLK0 and CCLK1 clocks can be disabled or enabled by writing 1 or 0 to the corresponding bit in the CGU\_CCBF\_DIS register. T o know which Core clock buffers are enabled or disabled, software reads the CGU\_CCBF\_STAT register. T o know which Core clock buffers were disabled since the last read, software reads the CGU\_CCBF\_STAT register. The SCLK0 , SCLK1 , DCLK and OCLK clocks can be disabled or enabled by writing 1 or 0 to the corresponding bit in the CGU\_SCBF\_DIS register. Software cannot disable SYSCLK .

## ADSP-SC5xx Specific Information

The processor has two system crystal oscillators and two system CGU units to provide the clocks to the system. Both of the CGUs come up in bypass mode out of reset.

CGU0 is the main CGU which provides the SYSCLK (SYSCLK\_0) , SCLK0 (SCLK0\_0) , and SCLK1 (SCKL1\_0) to the system buses, infrastructure, and most of the peripherals. The rest of the clock outputs from the two CGUs can be routed to a specific peripheral and the cores in the system. For more details, refer to the Clock Distribution Unit (CDU) chapter.

NOTE: Frequency ratios of Core clock to SYSCLK are either 2:1 or 1:1.

The processor supports no other frequency ratios. Program the divider/CDU values for the CLKO0 , CLKO1 , and CLKO2 carefully in relation to the divider programming for the SYSCLK .

## ADSP-SC58x CGU Register Descriptions

Clock Generation Unit (CGU) contains the following registers.

Table 3-8: ADSP-SC58x CGU Register List

| Name          | Description                        |
|---------------|------------------------------------|
| CGU_CCBF_DIS  | Core Clock Buffer Disable Register |
| CGU_CCBF_STAT | Core Clock Buffer Status Register  |
| CGU_CLKOUTSEL | CLKOUT Select Register             |
| CGU_CTL       | Control Register                   |
| CGU_DIV       | Clocks Divisor Register            |
| CGU_OSCWDCTL  | Oscillator Watchdog Register       |
| CGU_PLLCTL    | PLL Control Register               |

Table 3-8: ADSP-SC58x CGU Register List (Continued)

| Name          | Description                                      |
|---------------|--------------------------------------------------|
| CGU_REVID     | Revision ID Register                             |
| CGU_SCBF_DIS  | System Clock Buffer Disable Register             |
| CGU_SCBF_STAT | System Clock Buffer Status Register              |
| CGU_STAT      | Status Register                                  |
| CGU_TSCOUNT0  | Time Stamp Counter 32 LSB Register               |
| CGU_TSCOUNT1  | Time Stamp Counter 32 MSB Register               |
| CGU_TSCTL     | Time Stamp Control Register                      |
| CGU_TSVALUE0  | Time Stamp Counter Initial 32 LSB Value Register |
| CGU_TSVALUE1  | Time Stamp Counter Initial MSB Value Register    |

## Core Clock Buffer Disable Register

The CGU\_CCBF\_DIS register controls each core's clock buffer to determine if the CCLK is enabled.

Figure 3-3: CGU\_CCBF\_DIS Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000002_cb153e287aaa1e211c4aced198d7205f15a2314ab9e1011a7474fa5c0c9f184c.png)

Table 3-9: CGU\_CCBF\_DIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If set (=1) the CGU_CCBF_DIS.LOCK bit locks the CGU_CCBF_DIS register. 0 Unlock register 1 Lock register                |
| 1 (R/W)            | CCBF1      | Core Clock Buffer 1. The CGU_CCBF_DIS.CCBF1 bit enables (=0) or disables (=1) CCLK1s buffer.                                  |
| 0 (R/W)            | CCBF0      | Core Clock Buffer 0. The CGU_CCBF_DIS.CCBF0 bit enables (=0) or disables (=1) CCLK0s buffer. 0 Enable buffer 1 Disable buffer |

## Core Clock Buffer Status Register

The CGU\_CCBF\_STAT register shows which core clock buffer(s) are disabled. For example clearing the CGU\_CCBF\_DIS.CCBF0 bit clears the CGU\_CCBF\_STAT.CCBF0 bit after a number of cycles. To guarantee that the correct value is read, this register should be read twice and the second result used.

Figure 3-4: CGU\_CCBF\_STAT Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000003_4dc1a033cf0c20c5efac1f96b8e3bf7c075ea3cfda97a865bab95e2115835347.png)

Table 3-10: CGU\_CCBF\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | CCBF1      | Core Clock Buffer 1. The CGU_CCBF_STAT.CCBF1 bit reports the status of the CGU_CCBF_DIS.CCBF1 bit where 0 = enabled and 1 = disabled. 0 Enabled            |
| 0 (R/NW)           | CCBF0      | Core Clock Buffer 0. The CGU_CCBF_STAT.CCBF0 bit reports the status of the CGU_CCBF_DIS.CCBF0 bit where 0 = enabled and 1 = disabled. 0 Enabled 1 Disabled |

## CLKOUT Select Register

The CGU\_CLKOUTSEL selects the signal that the CGU drives through the CLKOUT multiplexer. Also, this register selects the divisor for the USBCLK output.

Figure 3-5: CGU\_CLKOUTSEL Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000004_31a8f1fcef426f44149cc4a86775055d561dda1e87bef72b810c1681c94bb5d9.png)

Table 3-11: CGU\_CLKOUTSEL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the CGU_CLKOUTSEL.LOCK bit is set, the CGU_CLKOUTSEL register is read only (locked).                                                                            |
| 31 (R/W)           | LOCK       | 0 Unlock                                                                                                                                                                                                                       |
| 31 (R/W)           | LOCK       | 1 Lock                                                                                                                                                                                                                         |
| 21:16 (R/W)        | USBCLKSEL  | USBCLK Select. The CGU_CLKOUTSEL.USBCLKSEL selects the divisor in the USBCLK equation: USBCLK frequency = (USB PLL frequency) / ( CGU_CLKOUTSEL.USBCLKSEL +1 ) Where the value of CGU_CLKOUTSEL.USBCLKSEL is between 0 and 63. |
| 21:16 (R/W)        | USBCLKSEL  | 0 USBCLKSEL = 0                                                                                                                                                                                                                |
| 21:16 (R/W)        | USBCLKSEL  | 63 USBCLKSEL = 63                                                                                                                                                                                                              |
| 4:0 (R/W)          | CLKOUTSEL  | CLKOUT Select. The CGU_CLKOUTSEL.CLKOUTSEL selects the signal that the CGU drives through the CLKOUT pin multiplexer.                                                                                                          |
| 4:0 (R/W)          | CLKOUTSEL  | 0 CLKIN0                                                                                                                                                                                                                       |
| 4:0 (R/W)          | CLKOUTSEL  | 1 CLKIN1                                                                                                                                                                                                                       |
| 4:0 (R/W)          | CLKOUTSEL  | 2 CGU_0.SYSCLK                                                                                                                                                                                                                 |
| 4:0 (R/W)          | CLKOUTSEL  | 3 CLKO0                                                                                                                                                                                                                        |

Table 3-11: CGU\_CLKOUTSEL Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            | 4 CLKO2                   |
|           |            | 5 CLKO3                   |
|           |            | 6 CLKO5                   |
|           |            | 7 CLKO7                   |
|           |            | 8 CLKO8                   |
|           |            | 9 Reserved                |
|           |            | 10 Reserved               |
|           |            | 11 Reserved               |
|           |            | 12 Reserved               |
|           |            | 13 Reserved               |
|           |            | 14 Reserved               |
|           |            | 15 Reserved               |
|           |            | 16 Reserved               |
|           |            | 17 Reserved               |
|           |            | 18 Reserved               |
|           |            | 19-31 Reserved            |

## Control Register

The CGU\_CTL controls the clock generation divisors for SYS\_CLKIN and the PLL. Read after write accesses to the CGU\_CTL register returns the new value even if the clock's frequency change is still in progress.

Figure 3-6: CGU\_CTL Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000005_52824150a8853a59d97683ddd8c29a27d9233447a9aec94bfb9cae47480ee5e7.png)

Table 3-12: CGU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the CGU_CTL.LOCK bit is set, the CGU_CTL register is read only (locked).                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 30 (R/W)           | WFI        | Wait For Idle. Modifying the PLL multiplier requires the PLL to re-lock and once the PLL locks, clocks have to be synchronized. Changes to the CGU_CTL.MSEL and the CGU_CTL.DF bit values results in bypassing the PLL. The CGU_CTL.WFI bit forces the PLL to wait for all processor cores to be in an idle or reset state before changing frequencies as a result of changes to the CGU_CTL.MSEL or CGU_CTL.DF bits. Write accesses to the CGU_CTL to change the CGU_CTL.DF or CGU_CTL.MSEL bit values while the PLL is locking sets the CGU_STAT.WDFMSERR bit. 0 Update Immediately for Idle |
| 30 (R/W)           |            | 1 Wait                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 30 (R/W)           |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 3-12: CGU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:8 (R/W)         | MSEL       | Multiplier Select. The CGU_CTL.MSEL bit field selects the multiplier in the PLLCLK equation: PLLCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL Where the value of MSEL is between 1 and 127. | Multiplier Select. The CGU_CTL.MSEL bit field selects the multiplier in the PLLCLK equation: PLLCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL Where the value of MSEL is between 1 and 127. |
| 0 (R/W)            | DF         | Divide Frequency. The CGU_CTL.DF bit selects whether or not the CLKIN input is divided by two be- fore being passed to the PLL.                                                                     | Divide Frequency. The CGU_CTL.DF bit selects whether or not the CLKIN input is divided by two be- fore being passed to the PLL.                                                                     |
| 0 (R/W)            | DF         | 0                                                                                                                                                                                                   | Pass OSC_CLKIN to PLL                                                                                                                                                                               |
| 0 (R/W)            | DF         | 1                                                                                                                                                                                                   | Pass OSC_CLKIN/2 to PLL                                                                                                                                                                             |

## Clocks Divisor Register

The CGU\_DIV register controls clock divisors for core clocks, system clocks, external (off core) memory clocks, and output clock. Read after write accesses to the CGU\_DIV register returns the new value even if the clock's frequency change is still in progress.

Figure 3-7: CGU\_DIV Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000006_0e6325f8450149847a8f6f515ec70ef81c0ef9757f23718f9c71ab6840f20381.png)

Table 3-13: CGU\_DIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the CGU_DIV.LOCK bit is set, the CGU_DIV register is read only (locked). 0 Unlock 1 Lock                                                                                                                    |
| 30 (R/W)           | UPDT       | Update Clock Divisors. The CGU_DIV.UPDT controls whether the CGU drives new CGU_DIV.CSEL , CGU_DIV.SYSSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , CGU_DIV.DSEL , and CGU_DIV.OSEL values to PLL after CGU_DIV register update. 0 No PLL Update 1 Drive Updated SEL Values to PLL |

Table 3-13: CGU\_DIV Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R0/W)          | ALGN       | Align. The CGU_DIV.ALGN directs the CGU to align the PLL-based clocks. The divisor se- lections ( CGU_DIV.CSEL , CGU_DIV.SYSSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , CGU_DIV.DSEL , and/or CGU_DIV.OSEL ) do not have to change. |
| 29 (R0/W)          | ALGN       | 0 No Action                                                                                                                                                                                                                   |
| 29 (R0/W)          | ALGN       | 1 Align PLL Clocks                                                                                                                                                                                                            |
| 28:22 (R/W)        | OSEL       | OCLK Divisor. The CGU_DIV.OSEL selects the divisor in the OCLK equation: OCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL / CGU_DIV.OSEL Where the value of CGU_DIV.OSEL is between 1 and 127.                          |
| 28:22 (R/W)        | OSEL       | 0 OSEL = 128                                                                                                                                                                                                                  |
| 28:22 (R/W)        | OSEL       | 1-127 OSEL = 1 to 127                                                                                                                                                                                                         |
| 20:16 (R/W)        | DSEL       | DCLK Divisor. The CGU_DIV.DSEL selects the divisor in the DCLK equation: DCLK frequency = (SYS_CLKIN frequency/(DF+1)) MSEL/ CGU_DIV.DSEL Where the value of CGU_DIV.DSEL is between 1 and 31.                                |
| 20:16 (R/W)        | DSEL       | 0 DSEL = 32                                                                                                                                                                                                                   |
| 20:16 (R/W)        | DSEL       | 1-31 DSEL = 1 to 31                                                                                                                                                                                                           |
| 15:13 (R/W)        | S1SEL      | SCLK 1 Divisor. The CGU_DIV.S1SEL selects the divisor in the SCLK1 equation: SCLK1 frequency = (SYSCLK frequency) / CGU_DIV.S1SEL Where the value of CGU_DIV.S1SEL is between 1 and 7. S1SEL = 8                              |
| 15:13 (R/W)        | S1SEL      | 0                                                                                                                                                                                                                             |
| 15:13 (R/W)        | S1SEL      | 1-7 S1SEL = 1 to 7                                                                                                                                                                                                            |
| 12:8 (R/W)         | SYSSEL     | SYSCLK Divisor. The CGU_DIV.SYSSEL selects the divisor in the SYSCLK equation: SYSCLK frequency = (SYS_CLKIN frequency/(DF+1)) MSEL/ CGU_DIV.SYSSEL Where the value of CGU_DIV.SYSSEL is between 1 and 31. SYSSEL = 32        |
| 12:8 (R/W)         | SYSSEL     | 0                                                                                                                                                                                                                             |
| 12:8 (R/W)         | SYSSEL     | 1-31 SYSSEL = 1 to 31                                                                                                                                                                                                         |

Table 3-13: CGU\_DIV Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:5 (R/W)          | S0SEL      | SCLK 0 Divisor. The CGU_DIV.S0SEL selects the divisor in the SCLK0 equation: SCLK0 frequency = (SYSCLK frequency) / CGU_DIV.S0SEL Where the value of CGU_DIV.S0SEL is between 1 and 7. 0 S0SEL = 8                 |
| 4:0 (R/W)          | CSEL       | CCLK Divisor. The CGU_DIV.CSEL selects the divisor in the CCLK equation: CCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL / CGU_DIV.CSEL Where the value of CGU_DIV.CSEL is between 1 and 31. 0 CSEL = 32 31 |
| 4:0 (R/W)          |            | 1-31 CSEL= 1 to                                                                                                                                                                                                    |
| 4:0 (R/W)          |            |                                                                                                                                                                                                                    |

## Oscillator Watchdog Register

The CGU\_OSCWDCTL register configures the CGU to allow the detection of the absence of input clock transitions and provides a fault warning via the SYS\_FAULT pin. The CGU\_OSCWDCTL register also detects and reports input oscillator frequencies above and below specified limits, in order to specifically detect harmonic or sub-harmonic crystal oscillator behavior. This detection is achieved by using an internal asynchronous, local 1 MHz oscillator combined with a series of programmable counters.

Figure 3-8: CGU\_OSCWDCTL Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000007_d805696aa02857fcd4c2dfc4fd0bb70e2ef137fb557070c6096f2a28f29ab9e5.png)

Table 3-14: CGU\_OSCWDCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                           |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK        | Lock.                                                                                                                                             |
| 23 (R/W)           | FAULTPINDIS | Fault Pin disabled. The CGU_OSCWDCTL.FAULTPINDIS bit disables pin fault detection.                                                                |
| 15 (R/W)           | MONDIS      | Oscillator Watchdog Monitor functions disabled. The CGU_OSCWDCTL.MONDIS bit disables all the input clock monitor and fault de- tection functions. |
| 14 (R/W)           | FAULTEN     | Fault enabled. The CGU_OSCWDCTL.FAULTEN bit enables fault detection.                                                                              |
| 13 (R/W)           | BOUEN       | Bad Oscillator Upper Frequency limit detection enabled. The CGU_OSCWDCTL.BOUEN bit enables upper limit bad oscillation detection.                 |

Table 3-14: CGU\_OSCWDCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:8 (R/W)         | BOUF       | Bad Oscillator Upper Frequency limit. The CGU_OSCWDCTL.BOUF bits indicate the desired upper fail limit for the bad os- cillation detection.                     |
| 7 (R/W)            | CNGEN      | Clock not Good enabled. The CGU_OSCWDCTL.CNGEN bit enables the detection of an oscillator watchdog clock fault.                                                 |
| 6 (R/W)            | HODEN      | Harmonic Oscillation Detection enabled. The CGU_OSCWDCTL.HODEN bit enables harmonic oscillation detection.                                                      |
| 5:0 (R/W)          | HODF       | Watchdog lower frequency limit. The CGU_OSCWDCTL.HODF bit field is used to indicate the desired lower fail limit for the harmonic oscillation detection in MHz. |

## PLL Control Register

The CGU\_PLLCTL register contains bits that enable and disable the PLL as well as control its function.

Figure 3-9: CGU\_PLLCTL Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000008_f4a8e7b97cadfb433a5f99bdcd14af57e88f9eb23f695b26f0af777f358e021a.png)

Table 3-15: CGU\_PLLCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. Setting (=1) the CGU_PLLCTL.LOCK bit locks access to the CGU_PLLCTL register. 0 Unlock register                      |
| 3 (R/W)            | PLLEN      | PLL Enable. Setting (=1) the CGU_PLLCTL.PLLEN bit enables the PLL.                                                         |
| 2 (R/W)            | PLLDIS     | PLL Disable. Setting (=1) the CGU_PLLCTL.PLLDIS bit disables the PLL. 0 No action                                          |
| 1 (R/W)            | PLLBPCL    | PLL Bypass Clear. Setting (=1) the CGU_PLLCTL.PLLBPCL bit takes the PLL out of bypass mode. 0 No action 1 Exit bypass mode |

Table 3-15: CGU\_PLLCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | PLLBPST    | PLL Bypass Set. Setting (=1) the CGU_PLLCTL.PLLBPST bit bypasses the PLL and all the clocks run on CLKIN. |
| 0 (R/W)            | PLLBPST    | 0 Use PLL                                                                                                 |
| 0 (R/W)            | PLLBPST    | 1 Bypass PLL                                                                                              |

## Revision ID Register

The CGU\_REVID register reports the version of the CGU.

Figure 3-10: CGU\_REVID Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000009_ff4f23d4094a65e4e9aaaf1347bc94fa53fdf5af66c213b5e8dc939c67b818e2.png)

Table 3-16: CGU\_REVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | MAJOR      | Major Version.            |
| 3:0 (R/NW)         | REV        | Incremental Version ID.   |

## System Clock Buffer Disable Register

The CGU\_SCBF\_DIS register controls each system's clock buffer to determine if the SCLKn buffer is enabled.

Figure 3-11: CGU\_SCBF\_DIS Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000010_638fd7a014e5fa3e8e159849309d145469abdfced2efa6a76ffc98875d0170cc.png)

Table 3-17: CGU\_SCBF\_DIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. The CGU_SCBF_DIS.LOCK bit allows writes to the CGU_SCBF_DIS register when cleared (=0) or blocks writes if set (=1) and the SPU_CTL.GLCK bit is set. |
| 3 (R/W)            | OUTCLKBF   | OCLK Buffer. The CGU_SCBF_DIS.OUTCLKBF bit enables (=0, default) or disables (=1) OCLKs buffer.                                                            |
| 2 (R/W)            | DCLKBF     | DCLK Buffer. The CGU_SCBF_DIS.DCLKBF bit enables (=0, default) or disables (=1) DCLKs buffer. 0 Enable buffer                                              |
| 2 (R/W)            |            |                                                                                                                                                            |

Table 3-17: CGU\_SCBF\_DIS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | SCLK1BF    | SCLK1 Buffer. The CGU_SCBF_DIS.SCLK1BF bit enables (=0, default) or disables (=1) SCLK1s buffer. 0 Enable buffer                  |
| 0 (R/W)            | SCLK0BF    | SCLK0 Buffer. The CGU_SCBF_DIS.SCLK0BF bit enables (=0, default) or disables (=1) SCLK0s buffer. 0 Enable buffer 1 Disable buffer |

## System Clock Buffer Status Register

The CGU\_SCBF\_STAT register shows which system clock buffer(s) are disabled. For example clearing the CGU\_CCBF\_DIS.CCBF0 bit clears the CGU\_SCBF\_STAT.SCLK0BF bit after a number of cycles. To guarantee that the correct value is read, this register should be read twice and the second result used.

Figure 3-12: CGU\_SCBF\_STAT Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000011_194d4ba602480dfd938834d1ffd9c03016d60842e14308cd24f738a8063293eb.png)

Table 3-18: CGU\_SCBF\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | OCLKBF     | OCLK Buffer. The CGU_SCBF_STAT.OCLKBF bit reports the status of the CGU_SCBF_DIS.OUTCLKBF bit where 0 = enabled and 1 = disabled.            |
| 2 (R/NW)           | DCLKBF     | DCLK1 Buffer. The CGU_SCBF_STAT.DCLKBF bit reports the status of the CGU_SCBF_DIS.DCLKBF bit where 0 = enabled and 1 = disabled.             |
| 1 (R/NW)           | SCLK1BF    | SCLK1 Buffer. The CGU_SCBF_STAT.SCLK1BF bit reports the status of the CGU_SCBF_DIS.SCLK1BF bit where 0 = enabled and 1 = disabled. 0 Enabled |
| 1 (R/NW)           |            |                                                                                                                                              |

Table 3-18: CGU\_SCBF\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | SCLK0BF    | SCLK0 Buffer. The CGU_SCBF_STAT.SCLK0BF bit reports the status of the CGU_SCBF_DIS.SCLK0BF bit where 0 = enabled and 1 = disabled. |
| 0 (R/NW)           | SCLK0BF    | 0 Enabled                                                                                                                          |
| 0 (R/NW)           | SCLK0BF    | 1 Disabled                                                                                                                         |

## Status Register

The CGU\_STAT register reflects the PLL status and errors detected during the PLL configuration. This register may be cleared asynchronously by a reset signal from the RCU module. All bits---except those defined as W1C (write-1to-clear)---are read only.

Figure 3-13: CGU\_STAT Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000012_90ae8fc8df26fd245724c6063c33c165ee438d50d5617ff5f1c02475091e0858.png)

Table 3-19: CGU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W1C)         | PCFGERR    | PLL Configuration Error. If the CGU_PLLCTL.PLLBPST and the CGU_PLLCTL.PLLBPCL bits are set (=1) simultaneously or the CGU_PLLCTL.PLLDIS bit was set (=1) in full-on mode or while trying to enter full-on mode ( CGU_PLLCTL.PLLBPCL =1), the CGU_STAT.PCFGERR bit triggers the bus error. |
| 21 (R/W1C)         | PCFGERR    | 0 No Error                                                                                                                                                                                                                                                                                |
| 21 (R/W1C)         | PCFGERR    | 1 Configuration Error                                                                                                                                                                                                                                                                     |

Table 3-19: CGU\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W1C)         | WDIVERR     | Write to DIV Error. The CGU_STAT.WDIVERR bit indicates a write access to the CGU_DIV register (to trigger an alignment sequence or to change the CGU_DIV.CSEL , CGU_DIV.SYSSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , or CGU_DIV.DSEL bit values) while the PLL is locked, but still aligning the clocks. Read after write accesses to the CGU_STAT and CGU_DIV registers return the new value even if the clock frequency change is still in progress. |
| 20 (R/W1C)         | WDIVERR     | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 20 (R/W1C)         | WDIVERR     | 1 Write DIV Error                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 19 (R/W1C)         | WDFMSERR    | Write to DF or MSEL Error. The CGU_STAT.WDFMSERR bit indicates a write access to the CGU_CTL register to change the CGU_CTL.DF or CGU_CTL.MSEL bit values while the PLL is locking.                                                                                                                                                                                                                                                                |
| 19 (R/W1C)         | WDFMSERR    | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 19 (R/W1C)         | WDFMSERR    | 1 Write DF/MSEL Error                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 17 (R/W1C)         | LWERR       | Lock Write Error. The CGU_STAT.LWERR bit indicates an attempt to write to write-protected (locked) CGU registers. The CGU issues a bus error for this condition.                                                                                                                                                                                                                                                                                   |
| 17 (R/W1C)         | LWERR       | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 17 (R/W1C)         | LWERR       | 1 Lock Write Error                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 16 (R/W1C)         | ADDRERR     | Address Error. The CGU_STAT.ADDRERR bit indicates an attempt to make a read or write access to unimplemented addresses or accesses are non-aligned. The CGU issues a bus error for this condition.                                                                                                                                                                                                                                                 |
| 16 (R/W1C)         | ADDRERR     | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 16 (R/W1C)         | ADDRERR     | 1 Address Error                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 15 (R/NW)          | OSCWDSTATF  | Oscillator Watchdog Status Fault. The CGU_STAT.OSCWDSTATF bit indicates a fault in the oscillator watchdog (CGU's OSC_WDSTAT[1:0]) input pins.                                                                                                                                                                                                                                                                                                     |
| 15 (R/NW)          | OSCWDSTATF  | 0 No Fault                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 15 (R/NW)          | OSCWDSTATF  | 1 Fault                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14:12 (R/NW)       | OSCWDSTATFC | Oscillator Watchdog Status Fault Code. The CGU_STAT.OSCWDSTATFC bit field indicates the nature of the fault in the os- cillator watchdog (CGU's OSC_WDSTAT[1:0]) input pins.                                                                                                                                                                                                                                                                       |
| 14:12 (R/NW)       | OSCWDSTATFC | 0 No Fault                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/NW)       | OSCWDSTATFC | 1 No Input Clock                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 3-19: CGU\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Subharmonic CLKIN                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Harmonic CLKIN                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | No AUX_CLK                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | CLKIN > Upper Frequency Limit (BOUF)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Multiple Limit Faults                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 3 (R/NW)           | CLKSALGN   | Clock Alignment. The CGU_STAT.CLKSALGN bit indicates whether a clock alignment sequence is in progress. This bit is set when clocks alignment is required by changes to CGU_DIV.CSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , CGU_DIV.DSEL , or CGU_DIV.OSEL . The CGU_STAT.CLKSALGN bit is cleared when clocks are aligned. Note that (after a PLL frequency change in active state) the CGU_STAT.CLKSALGN bit may indicate that clocks are not aligned even though the clocks are aligned (all clocks are aligned and running at CLKIN frequency). | Clock Alignment. The CGU_STAT.CLKSALGN bit indicates whether a clock alignment sequence is in progress. This bit is set when clocks alignment is required by changes to CGU_DIV.CSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , CGU_DIV.DSEL , or CGU_DIV.OSEL . The CGU_STAT.CLKSALGN bit is cleared when clocks are aligned. Note that (after a PLL frequency change in active state) the CGU_STAT.CLKSALGN bit may indicate that clocks are not aligned even though the clocks are aligned (all clocks are aligned and running at CLKIN frequency). |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Clocks are Aligned                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Clocks not Aligned (alignment in progress)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 2 (R/NW)           | PLOCK      | PLL Lock. The CGU_STAT.PLOCK bit indicates whether the PLL is locked. This bit is set when the PLL locks (PLL lock counter end-of-count). The CGU_STAT.PLOCK bit is cleared when requested PLL frequency change (for PLL reset, PLL disable-to-enable transition, or a change to the CGU_CTL.MSEL or CGU_CTL.DF values) is in prog- ress.                                                                                                                                                                                                     | PLL Lock. The CGU_STAT.PLOCK bit indicates whether the PLL is locked. This bit is set when the PLL locks (PLL lock counter end-of-count). The CGU_STAT.PLOCK bit is cleared when requested PLL frequency change (for PLL reset, PLL disable-to-enable transition, or a change to the CGU_CTL.MSEL or CGU_CTL.DF values) is in prog- ress.                                                                                                                                                                                                     |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | PLL not Locked (PLL frequency change in progress)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | PLL Locked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/NW)           | PLLBP      | PLL Bypass. The CGU_STAT.PLLBP bit indicates whether the PLL is bypassed. The default val- ue for the bit is determined by the bypass strap pin.                                                                                                                                                                                                                                                                                                                                                                                              | PLL Bypass. The CGU_STAT.PLLBP bit indicates whether the PLL is bypassed. The default val- ue for the bit is determined by the bypass strap pin.                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | PLL not Bypassed                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | PLL Bypassed                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 0                  | PLLEN      | PLL Enable.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | PLL Enable.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| (R/NW)             |            | The CGU_STAT.PLLEN bit indicates whether the PLL is enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | The CGU_STAT.PLLEN bit indicates whether the PLL is enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Disabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Enabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

## Time Stamp Counter 32 LSB Register

The CGU\_TSCOUNT0 register address is used to read the CoreSight time stamp counter LSB 32-bit (bits [31:0]) value.

Figure 3-14: CGU\_TSCOUNT0 Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000013_153c3f37588756d5f88d883634102c77e4d13bc851dee69c052af37cc1f8313c.png)

Table 3-20: CGU\_TSCOUNT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | VALUE      | TSCOUNT0 Value.                                                        |
| (R/NW)             |            | The CGU_TSCOUNT0.VALUE bit field holds the time stamp counter 32 LSBs. |

## Time Stamp Counter 32 MSB Register

The CGU\_TSCOUNT1 register address is used to read the CoreSight time stamp counter MSB 32-bit (bits [63:32]) value.

Figure 3-15: CGU\_TSCOUNT1 Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000014_ef45cca0279ccaeac5dad10f694003027de3cbe79d7b53421f839b8df4ea837f.png)

Table 3-21: CGU\_TSCOUNT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | VALUE      | Timestamp Counter 32 MSB.                                              |
| (R/NW)             |            | The CGU_TSCOUNT1.VALUE bit field holds the time stamp counter 32 MSBs. |

## Time Stamp Control Register

The CGU\_TSCTL register controls the operation of the CoreSight time stamp counter.

Figure 3-16: CGU\_TSCTL Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000015_7592c2292480dad6223d9bb74c562cb91ce4075f29062840e3316a2b1b212981.png)

Table 3-22: CGU\_TSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. Setting the CGU_TSCTL.LOCK bit locks this register. 0 Unlock                                                                                                          |
| 7:4 (R/W)          | TSDIV      | Counter's Clock Divider. The CGU_TSCTL.TSDIV bit field divides SYSCLK by 2 TSDIV . 0-15 Divides SYSCLK by 2 TSDIV                                                           |
| 1 (R/W)            | LOAD       | Load Counter. Writing one to the CGU_TSCTL.LOAD bit causes CoreSight time stamp counter to be loaded from the CGU_TSVALUE0 and CGU_TSVALUE1 registers. 0 Always read as "0" |
| 0 (R/W)            | EN         | Counter Enable. The CGU_TSCTL.EN bit enables or disables the CoreSight time stamp counter. 0 Counter Disabled                                                               |

## Time Stamp Counter Initial 32 LSB Value Register

The CGU\_TSVALUE0 register holds the least significant bits (bits [31:0]) value that is initially loaded to the CoreSight time stamp counter.

Figure 3-17: CGU\_TSVALUE0 Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000016_4f385e866d1e8a9d30c85094ed6ef6fb5c5f70397065a3ce927bb39b6ba4809d.png)

Table 3-23: CGU\_TSVALUE0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Counter's 32 LSB Initial Value. The CGU_TSVALUE0.VALUE bit field holds the LSBs value that is initially loaded to the CoreSight time stamp counter. |

## Time Stamp Counter Initial MSB Value Register

The CGU\_TSVALUE1 register holds the most significant bits (bits [63:32]) value that is initially loaded to the CoreSight time stamp counter.

Figure 3-18: CGU\_TSVALUE1 Register Diagram

![Image](06_Clock_Generation_Unit_(CGU)_artifacts/image_000017_7f8a974afe06a5f9f9985e816a4ff31220694bc975309abbbf942e75482afddc.png)

Table 3-24: CGU\_TSVALUE1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Counter's Initial 32 MSB Value. The CGU_TSVALUE1.VALUE bit field holds the MSBs value that is initially loaded to the CoreSight time stamp counter. |