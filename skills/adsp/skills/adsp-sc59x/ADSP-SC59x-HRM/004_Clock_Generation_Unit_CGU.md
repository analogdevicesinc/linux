# Clock Generation Unit (CGU)

<!-- source: 004_Clock_Generation_Unit_CGU.pdf | original pages 134–185 -->

## 2   Clock Generation Unit (CGU)

The Clock Generation Unit (CGU) includes the phase locked loop (PLL) and the PLL control unit (PCU). The PLL generates a requester clock that runs at a frequency that is a multiple of the CLKIN input clock frequency. The PCU divides down the requester clock to generate various system clocks and synchronization signals.

## CGU Features

The CGU module supports the following features:

- Provides smooth transitions from the current clock condition to a new condition with PLL logic and executes the changes to clocks due to register programming.
- Provides PLL and clock domain status reporting for event management.
- Supports the capability to bypass the PLL for power savings.
- Software controlled dynamic power management allows control of the core clock frequency (f CCLK ).
- Controls clock gating of the core and system clocks.

NOTE: For more information about processor-specific CGU features, see the processor data sheet.

## CGU Functional Description

The CGU generates all on-chip clocks and synchronization signals based on the programmed PLL multiplication factor and dividers. The CGU provides the following functionality.

## Change the PLL Clock Frequency

The CGU allows programs to change the PLL clock frequency by writing new values to bits in the control register. Any time the PLL relocks, the CGU aligns all core and system clocks.

## Change Other Clock Frequencies

The CGU allows programs to change the CCLKn, SYSCLK, SCLKn, DCLK, and OCLK frequencies by writing values to the CGU\_DIV register. Any time the clock frequency is changed, the OCLK, CCLKn, SYSCLK, DCLK, and SCLKn clocks exit the frequency change sequence aligned.

## Perform Clock Alignment

The CGU can align all clocks by writing to the CGU\_DIV register. This function aligns all PLL-based clocks.

For more information on these functions, see the CGU Programming Model section.

## ADSP-SC59x CGU Register List

The clock generation unit (CGU) includes the phase locked loop (PLL) and the PLL control unit (PCU). The PLL generates a clock, running at a frequency that is a multiple of the CLKIN input clock's frequency. The CGU also generates all on-chip clocks and synchronization signals. The PCU permits application software control of the PLL's operation. A set of registers govern CGU operations. For more information on CGU functionality, see the CGU register descriptions.

Table 2-1: ADSP-SC59x CGU Register List

| Name          | Description                                      |
|---------------|--------------------------------------------------|
| CGU_CCBF_DIS  | Core Clock Buffer Disable Register               |
| CGU_CCBF_STAT | Core Clock Buffer Status Register                |
| CGU_CLKOUTSEL | CLKOUT Select Register                           |
| CGU_CTL       | Control Register                                 |
| CGU_DIV       | Clocks Divisor Register                          |
| CGU_DIVEX     | DIV Register Extension                           |
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

## ADSP-SC59x CGU Interrupt List

Table 2-2: ADSP-SC59x CGU Interrupt List

|   Interrupt ID | Name     | Description   | Sensitivity   | DMA Channel   |
|----------------|----------|---------------|---------------|---------------|
|              1 | CGU0_EVT | CGU0 Event    | Edge          |               |
|              2 | CGU1_EVT | CGU1 Event    | Edge          |               |

## ADSP-SC59x CGU Trigger List

Table 2-3: ADSP-SC59x CGU Trigger List Generators

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|            1 | CGU0_EVT | CGU0 Event    | Edge          |
|            2 | CGU1_EVT | CGU1 Event    | Edge          |

Table 2-4: ADSP-SC59x CGU Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## CGU Definitions

## DPM

The Dynamic Power Management module (DPM) works with the CGU to provide flexible power dissipation modes for the processor.

## PCU

The PLL control unit (PCU) in the CGU controls PLL operations. The MMR registers of the CGU are implemented in the PCU.

## PLL

The phase-locked loop (PLL) operates within the CGU.

## RCU

The reset control unit (RCU) provides input to the CGU to manage clocks during processor reset.

## CDU

The clock distribution unit distributes the clocks from the CGU to different clock domains

## CGU

The clock generation unit (CGU) is comprised of the PLL and PCU. The CGU generates the clocks listed in the Clock Descriptions table.

Table 2-5: Clock Descriptions

| Clock    | Description                |
|----------|----------------------------|
| CCLK0_0  | CCLK0 derived from CGU0    |
| CCLK1_0  | CCLK1 derived from CGU0    |
| CCLK2_0  | CCLK2 derived from CGU0 *1 |
| SYSCLK_0 | SYSCLK derived from CGU0   |
| SCLK0_0  | SCLK0 derived from CGU0    |
| SCLK1_0  | SCLK1 derived from CGU0    |
| DCLK_0   | DCLK derived from CGU0     |
| OCLK_0   | OCLK derived from CGU0     |
| CCLK0_1  | CCLK0 derived from CGU1    |
| CCLK2_1  | CCLK2 derived from CGU1 *1 |
| SYSCLK_1 | SYSCLK derived from CGU1   |
| SCLK0_1  | SCLK0 derived from CGU1    |
| SCLK1_1  | SCLK1 derived from CGU1    |
| DCLK_1   | DCLK derived from CGU1     |
| OCLK_1   | OCLK derived from CGU1     |
| DCLK_2   | DCLK derived from CGU2     |

## CGU PLL Block Diagram

The CGU PLL Block Diagram provides a top-level block diagram of the phase locked loop (PLL). The main blocks of the PLL are the phase/frequency detector (PFD), the charge pump, the loop filter and the voltage controlled oscillator (VCO). The VCO multiplies the SYS\_CLKIN0 or SYS\_CLKIN1 , input to a higher frequency.

Additional configuration options are configured using the Clock Distribution Unit.

<!-- image -->

* S0SELEXEN AND SOSELEX (EXTENDED SCLK0\_1) APPLY TO CGU1 ONLY

Figure 2-1: CGU1 PLL Block Diagram

<!-- image -->

Figure 2-2: CGU2 PLL Block Diagram

## CLKOUT Selections

The SYS\_CLKOUT Generation figure is a conceptual representation of the CLKOUT module. Different clocks that originate from the CGU blocks are available on the SYS\_CLKOUT output pin. The selection of the clock output on the SYS\_CLKOUT pin is controlled by the CGU\_CLKOUTSEL.CLKOUTSEL bit field. Most of the clock frequencies are divided with a fixed divider value, for example CGU0\_SYSCLK0 has a fixed divider of 8 for the CLKOUT pin.

<!-- image -->

* SHOULD NOT EXCEED 125MHz

Figure 2-3: SYS\_CLKOUT Generation

The BMODE pin affects the CLKOUT pins after a hardware reset is deasserted.

BMODE= 0 - When a hardware reset is deasserted, CGU0\_OCLK/PLLCLK0 is selected to come out of the CLKOUT pin. The default value of OSEL and MSEL is the same. This functionality enables the user to verify PLL functionality by checking that the CLKOUT period is identical to CLKIN period. While programming all values of the CGU\_CLKOUTSEL.CLKOUTSEL bit field is possible, changing it to 0 (SYS\_CLKIN) is not possible until and unless a non-zero value is first programmed to the CGU\_CLKOUTSEL.CLKOUTSEL bit field.

BMODE = (non-zero) - When a hardware reset is deasserted, SYS\_CLKIN is selected by default.

## CGU Operating Modes

The CGU does not have configurable operating modes, but CGU operations affect the operating modes of other modules. Some CGU operation issues that affect operation of other modules include the following:

- The PLL of the CGU operates in either normal mode (CGU clock divisors applied) or bypass mode (CGU PLL is bypassed and clock divisors ignored).
- The SCB uses the CGU for clock synchronization across clock domains. For more information, see System Crossbars (SCB).
- The DPM uses the CGU for clock management as power state transitions occur. For more information, see the Dynamic Power Management (DPM) chapter.
- The CGU uses clock gating control to obtain flexible low-power modes.

## CGU Power-up Sequence

See the product data sheet for exact power-up requirements. The processor is configured to come up in clock bypass mode. The programs is required to configure full speed clocks and safety monitors. SYS\_CLKIN0 or SYS\_CLKIN1 , and all supplies should be stable before the SYS\_HWRST signal is deasserted.

## CGU Event Control

The CGU generates an event or error for several different reasons. Events and errors are described in the following sections.

## Events

After a frequency change, a CGU event indicates that the PLL has locked, and clocks are synchronized. If a core was idled while changing frequencies, the CGU can use an event interrupt to break the core idle. While in active mode, a CGU event indicates that the PLL has locked.

## CGU Errors

A CGU error occurs under following conditions:

- A write access to the CGU\_DIV / CGU\_DIVEX registers trigger an alignment sequence while the PLL is locked and is still aligning the clocks.

The CGU\_STAT.WDIVERR bit state indicates this error. If this error occurs, clear the CGU\_STAT.WDIVERR bit and rewrite the desired values to the CGU\_DIV / CGU\_DIVEX registers.

- A change to the CGU\_DIV / CGU\_DIVEX registers occur while the PLL is locked and is still aligning the clocks.

The CGU\_STAT.WDIVERR bit state indicates this error. If this error occurs, clear the CGU\_STAT.WDIVERR bit and rewrite the desired values to the CGU\_DIV / CGU\_DIVEX registers.

- A write access to the CGU\_CTL.DF bit field occurs or a write access to the CGU\_CTL.MSEL bit field occurs while the PLL is locking.

The CGU\_STAT.WDFMSERR bit state indicates this error. If this error occurs, wait until the PLL has finished locking, cleared the error, and rewritten the desired value change.

- A write access to CGU\_DIV.S1SEL when CGU\_CTL.S1SELEXEN is enabled or a write access to CGU\_DIVEX.S1SELEX when CGU\_CTL.S1SELEXEN is disabled.

A write access to CGU\_DIV.S0SEL when CGU\_CTL.S0SELEXEN is enabled or a write access to CGU\_DIVEX.S0SELEX when CGU\_CTL.S0SELEXEN is disabled.

The CGU\_STAT.WDIVERR bit state indicates this error. If this error occurs, clear the CGU\_STAT.WDIVERR bit and rewrite the desired values to the CGU\_DIV and CGU\_DIVEX registers.

The CGU monitors changes to the following fields:

- SYSCLK Divisor -CGU\_DIV.SYSSEL

## CGU Generated Bus Errors

The CGU generates a bus error when a read or write transaction to an unused address within the CGU address range is attempted. It also generates a bus error when a misaligned access is made to a CGU register. In addition to the bus error, the CGU\_STAT.ADDRERR bit is set. If a write to a write-protected CGU register is attempted, the CGU generates a bus error. In addition, the CGU\_STAT.LWERR bit is set.

## Oscillator Watchdog

The oscillator watchdog detects the absence of input clock transitions and provides a fault warning through the SYS\_FAULT pin. To detect harmonic or subharmonic crystal oscillator behavior, the watchdog (under programmable control) also detects and reports input oscillator frequencies above and below the specified limits. Use an internal asynchronous, local 1 MHz oscillator combined with a series of programmable counters for this detection. Set the CGU\_OSCWDCTL.MONDIS bit and clear the CGU\_OSCWDCTL.FAULTEN bit to optionally disable all the input clock monitor and fault detection functions.

Set the CGU\_OSCWDCTL.HODEN bit to enable harmonic oscillation detection. The CGU uses the CGU\_OSCWDCTL.HODF bit field to indicate the desired lower fail limit (in MHz) for the harmonic oscillation detection. The upper limit is always twice the lower limit.

The HODF Settings for Different Input Clock Frequencies table shows the recommended values of the CGU\_OSCWDCTL.HODF bit settings for different input clock frequencies.

Table 2-6: HODF Settings for Different Input Clock Frequencies

|   CGU_OSCWDCTL. HODF [5:0] |   Subharmonic Fre- quency (MHz) |   Nom. Lower Fail Limit (MHz) |   Input Clock Fre- quency (MHz) |   Nom. Upper Fail Limit (MHz) |   Second Harmonic Frequency (MHz) |
|----------------------------|---------------------------------|-------------------------------|---------------------------------|-------------------------------|-----------------------------------|
|                         16 |                            10   |                            16 |                              20 |                            32 |                                40 |
|                         16 |                            12.5 |                            16 |                              25 |                            32 |                                50 |
|                         20 |                            15   |                            20 |                              30 |                            40 |                                60 |

NOTE: Use a HODF value of 16 for frequency ranges greater than 20 MHz and less than 25 MHz. For frequency ranges greater than 25 MHz and less than 30 MHz, use a HODF value of 20.

The CGU uses the CGU\_OSCWDCTL.BOUF (Bad Oscillator Upper Frequency limit) asynchronous control bit field to indicate the desired upper fail limit for the bad oscillation detection. Set the CGU\_OSCWDCTL.BOUEN bit to enable upper-limit bad oscillation detection. A bad oscillation detection condition signals a fault before any processor operations occur. This detection occurs (even in bypass mode) whenever a clock frequency exceeds the specifications.

The CGU\_OSCWDCTL.BOUF =0 operation starts with a target of 32 MHz and each additional LSB increases the frequency test limit by 2 MHz. For example:

Target Upper Frequency Limit = CGU\_OSCWDCTL.BOUF × 2 MHz + 32 MHz

The CGU\_STAT.OSCWDSTATFC status bits indicate the nature of the fault.

The Fault Map table shows the fault values.

Table 2-7: Fault Map

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

Program and enable the OSCWDOG to match the actual crystal, before bringing the PLL out of bypass.

## CGU Programming Model

The programming model for the CGU involves the various mode configuration techniques.

## Configuring CGU Modes

Use the following procedures to configure the clocks and PLL.

NOTE: The program needs to clear the CGU\_STAT.CLKSALGN bit before changing clocks. The following sequence is executed once, inside the application, after coming out of reset.

```
*pREG_CGU0_PLLCTL |= BITM_CGU_PLLCTL_PLLBPCL; // come out of bypass and enter Full ON while( (pADI_CGU0 ->STAT & 0xF) != 0x5 ) { } // poll // now clocks are running with hardware default divisors. // now program can change frequencies If desired the program can put the PLL again into bypass.
```

## Changing the Clock Frequencies

Applications change clock frequencies in two ways. The first way is modifying the PLL multiplication value by writing to the CGU\_CTL register and the second is modifying the clock dividers by writing to the CGU\_DIV register. Both actions have different implications even if the frequencies of the final clock are the same. Write accesses to change the CGU\_CTL.DF or CGU\_CTL.MSEL bit fields while the PLL is locking set the CGU\_STAT.WDFMSERR error bit. The CGU\_STAT.WDIVERR error bit is set when one of following accesses is attempted while the PLL is locked, but still aligning the clocks:

- A write access to the CGU\_DIV / CGU\_DIVEX to trigger an alignment sequence.
- A write access to the CGU\_DIV / CGU\_DIVEX to change the CGU\_DIV.CSEL /, CGU\_DIV.SYSSEL , CGU\_DIV.S0SEL , CGU\_DIV.S1SEL , or CGU\_DIV.DSEL / CGU\_DIVEX.S1SELEX , CGU\_DIVEX.S0SELEX bits.

Read-after-write accesses to these registers return the new value, even if the frequency of the clock change is still inprogress.

Modifying the PLL multiplier requires the PLL to relock. Once the PLL locks, the CGU synchronizes the clocks. Changes to the CGU\_CTL.DF or CGU\_CTL.MSEL bit field result in bypassing the PLL. If necessary, clear the CGU\_DIV.UPDT bit to avoid multiple clock alignment sequences. If the CGU\_DIV register is not updated, the CGU uses the current values to determine the frequencies of the clock. It is the programs responsibility to guarantee that the new CGU\_CTL.DF or CGU\_CTL.MSEL and CGU\_DIV combinations are legal.

## Changing the PLL Clock Frequency

To change the phase-locked loop clock ( PLLCLK ) frequency, write new values to the CGU\_CTL.MSEL field or CGU\_CTL.DF field. Any time the PLL relocks, all core and system clocks are aligned.

1. Read CGU\_STAT register and verify that:
2. Write the desired values to the clock divisor select fields of the CGU\_DIV / CGU\_DIVEX register with the CGU\_DIV.UPDT bit =0.
3. Write the desired values to the CGU\_CTL.DF and CGU\_CTL.MSEL fields.

```
a. The CGU_STAT.PLLEN bit =1 (PLL enabled) b. The CGU_STAT.PLOCK bit =1 (PLL is not locking) c. The CGU_STAT.CLKSALGN bit =0 (clocks aligned)
```

This sequence performs these actions:

1. Updates the corresponding CGU registers.
2. Bypasses the PLL.
3. Makes the PLL lock to the new values in the CGU\_CTL.MSEL or CGU\_CTL.DF fields.
4. Changes the clock frequencies.

5. Exits the PLL bypass with all clocks aligned.

When exiting the PLL bypass state, a CGU event occurs.

The CGU\_STAT register exits this sequence with the CGU\_STAT.PLLEN bit =1, the CGU\_STAT.PLOCK bit =1, the CGU\_STAT.PLLBP bit =0, and the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.PLOCK bit, CGU\_STAT.PLLBP bit, and CGU\_STAT.CLKSALGN bit to discover when the PLL is locked and the clocks are aligned.

Changing the frequency of the PLL is allowed while the PLL is bypassed. In this case the new PLLCLK frequency is not used until the PLL is no longer bypassed.

## Changing the CCLKn or SYSCLK Frequency Without Modifying the PLLCLK Frequency

To change the clock frequencies, write new values to CGU\_DIV.CSEL or CGU\_DIV.SYSSEL bits. The frequency change occurs only when the PLL is not bypassed. Any time the CCLKn or SYSCLK clock frequencies are changed, they exit the frequency change sequence aligned.

1. Read the CGU\_STAT register to verify that the CGU\_STAT.CLKSALGN bit =0 (clocks aligned).
2. Write the desired CGU\_DIV.CSEL , CGU\_DIV.SYSSEL , and CGU\_DIV.OSEL bitfield values with the CGU\_DIV.UPDT bit = 1 and/or, write to the CGU\_DIVEX.S1SELEX field. This write updates the CGU\_DIV register, changes the SCLKn and SYSCLK frequencies, and aligns the clocks. When the clocks are aligned, a CGU event occurs.

The CGU\_STAT register exits this sequence with the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.CLKSALGN bit to discover when the clocks are aligned. Any write attempt to change the CGU\_DIV.S0SEL or CGU\_DIV.S1SEL bit fields while CGU\_STAT.CLKSALGN bit =1 (clocks alignment in progress) triggers an MMR access bus error and the CGU\_DIV register is not modified.

Also, any write attempt to change the CGU\_DIVEX.S1SELEX field while the CGU\_STAT.CLKSALGN bit =1 (clocks alignment in progress) triggers an MMR access bus error and the CGU\_DIVEX register is not modified.

Programming the SYSCLK frequency to a higher value than CCLKn also triggers an MMR access bus error and the CGU\_DIV / CGU\_DIVEX register is not modified.

Writing to the CGU\_DIV / CGU\_DIVEX register is allowed while the processor is in active (PLL bypassed) mode. In this case the effect of the write is visible only after the transition to full-on (PLL not bypassed) mode.

Accessing the DDR memory while changing the SYSCLK frequency is not supported and can have unpredictable results.

## Changing the OCLK Frequency

To change the OCLK clock frequency, write a new CGU\_DIV.OSEL bit value. Any time the OCLK clock frequency is changed, the OCLK, CCLKn, SYSCLK, and SCLKn clocks exit the frequency change sequence aligned.

1. Read the CGU\_STAT register to verify that the CGU\_STAT.CLKSALGN bit =0 (clocks aligned).
2. Write the desired CGU\_DIV.OSEL value with the CGU\_DIV.UPDT bit =1. This write updates the CGU\_DIV register, changes the OCLK frequency, and aligns all clocks except OCLK.

The CGU\_STAT register exits this sequence with the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.CLKSALGN bit to discover when the clocks are aligned. Any write attempt to change the CGU\_DIV.DSEL field while the CGU\_STAT.CLKSALGN bit =1 (clock alignment in progress) triggers an MMR access bus error and the CGU\_DIV / CGU\_DIVEX is not modified. When the clocks are aligned, a CGU event occurs.

Writing to the CGU\_DIV.OSEL bit field is allowed while the processor is in active (PLL bypassed) mode. In this case the effect of the write is visible only after the transition to full-on (PLL not bypassed) mode.

## Selecting SCLK1\_X and SCLK0\_X Source

Programming the CGU\_DIVEX enables the PLL clock as a source (bypassing SYSCLK0\_0 that continues to source other peripherals). When CGU\_DIVEX is enabled, the PLL clock is divided from CGU\_DIVEX register value.

## Changing SCLK1\_X or SCLK0\_X Frequencies

Complete the following steps to change the SCLK0 or SCLK1 frequency.

1. Read the CGU\_STAT register. Verify that CGU\_STAT.CLKSALGN = 0 (Clocks aligned).
2. Read the CGU\_CTL register. Verify that the CGU\_DIVEX.S1SELEX or CGU\_DIVEX.S0SELEX bit is set as needed. (These bits can only be modified when the PLL is bypassed).
3. Read the CGU\_STAT register. Verify that the PLL is not bypassed. (If it is bypassed, see PLL Bypass and PLL Disable topic.)
4. Read the CGU\_DIV register. Verify that the CGU\_DIV.UPDT bit is set.
5. Write the desired CGU\_DIVEX.S1SELEX or CGU\_DIVEX.S0SELEX value to the CGU\_DIVEX registers .

## Aligning All Clocks

To align the clocks, write 1 to the CGU\_DIV.ALGN bit. The frequency can also be changed, if necessary. The clocks aligned include:

- CCLKn
- SYSCLK
- SCLKn
- DCLK
- OCLK

1. Read the CGU\_STAT register to verify that CGU\_STAT.CLKSALGN bit =0 (clocks aligned).
2. Write 1 to the CGU\_DIV.ALGN bit. All other fields can change.

ADDITIONAL INFORMATION: This write does not alter the CGU\_DIV / CGU\_DIVEX register unless one of the clock-select fields is modified. When the clocks are aligned, a CGU event occurs.

The CGU\_STAT register exits this sequence with the CGU\_STAT.CLKSALGN bit =0. Poll the CGU\_STAT.CLKSALGN bit to discover when the clocks are aligned. Any write to the CGU\_DIV / CGU\_DIVEX register intended to align clocks or to change a clock select field while the CGU\_STAT.CLKSALGN bit =1 (clocks alignment in progress) triggers an MMR access bus error. In this case, the CGU\_DIV / CGU\_DIVEX register is not modified.

Writing 1 to the CGU\_DIV.ALGN bit has no effect while the processor is in active (PLL bypassed) mode.

## Shutting Off CCLKn from Another Controller

CCLKn can be shut off to save power when it is not in use.

1. Disable interrupts to core n.
2. Set the RCU\_SIDIS.SI[n] bit to disable the interfaces of core n to:
- a. Stop DMA accesses to its L1
- b. Stop memory accesses to core 0
- c. Stop accesses to MMRs
3. Test the RCU\_SISTAT.SI[n] bit to detect when accesses to core n have been disabled and all the pending transactions have completed.
4. Set the CGU\_CCBF\_DIS.CCBF0 bit to disable the CCLKn buffer.
5. Check the CGU\_CCBF\_STAT.CCBF0 bit.

If the CGU\_CCBF\_STAT.CCBF0 bit is set, continue.

## Reenable CCLKn From Another Controller

1. Clear the CGU\_CCBF\_DIS.CCBF0 to enable CCLKn.
2. Check the CGU\_CCBF\_STAT.CCBF0 bit.
- a. If the CGU\_CCBF\_DIS.CCBF0 bit is cleared, continue
3. Clear the RCU\_SIDIS.SI[n] bit. The core deasserts its acknowledge signal in response to the RCU\_SYSRST0 signal. This operation clears the RCU\_SISTAT.SI[n] bit.

## Valid Clock Multiplier Settings

Processor operations depend on valid settings in the CGU\_CTL and CGU\_DIV registers. These registers control the clock multiplier and divisor values. Set these registers such that the minimum and maximum clocks specified in the data sheet are not violated. All other clock specifications in the data sheet must also be adhered to for correct operation of the processor.

## PLL Bypass and PLL Disable

Writing 1 to the CGU\_PLLCTL.PLLBPST bit tells the PLL to apply OSC\_CLKIN clock to CCLK, SYSCLK, SCLK0, SCLK1, DCLK (PLL Bypass), and OCLK outputs. Writing 1 to the CGU\_PLLCTL.PLLBPCL bit tells the PLL to exit its PLL Bypass state and make all output clocks align and transition to their programmed frequencies.

The PLL can be disabled by clearing the CGU\_PLLCTL.PLLEN bit while in the bypass state. If necessary, clock buffers can be disabled.

CCLK0 clock can be disabled or enabled by writing 1 or 0 to the corresponding bit in the CGU\_CCBF\_DIS register.

To determine which core clock buffers were disabled since the last read, software reads the CGU\_CCBF\_STAT register. The SCLK0, SCLK1, DCLK and OCLK clocks can be disabled or enabled by writing 1 or 0 to the corresponding bit in the CGU\_SCBF\_DIS register. Software cannot disable SYSCLK.

## DCLK from CGU2

PLL relocking with MSEL update:

1. Clear the MISCREG\_PLL2\_CONTROL.BYPASSB , MISCREG\_PLL2\_CONTROL.DSEL\_DIV\_CHG and MISCREG\_PLL2\_CONTROL.MSELDF\_CHG bits.
2. Wait 20 core clock cycles for the PLL of the third SYSCLK to reflect CLKIN.
3. Update the MISCREG\_PLL2\_CONTROL.MSEL value and set the MISCREG\_PLL2\_CONTROL.MSELDF\_CHG bit.
4. Wait 768 core clock cycles for the MSELDF\_CHG pulse.
5. Wait 2048 CLKIN cycles for the PLL to lock.
6. Set the MISCREG\_PLL2\_CONTROL.BYPASSB bit.
7. Set the MISCREG\_PLL2\_CONTROL.BYPASSB bit

## DCLK divider update:

1. Clear the MISCREG\_PLL2\_CONTROL.BYPASSB , MISCREG\_PLL2\_CONTROL.DSEL\_DIV\_CHG , and MISCREG\_PLL2\_CONTROL.MSELDF\_CHG bits.
2. Update the MISCREG\_PLL2\_CONTROL.DSEL value and set the MISCREG\_PLL2\_CONTROL.DSEL\_DIV\_CHG bit.

3. Wait 768 core clock cycles for the MSELDF\_CHG pulse.
4. Set the MISCREG\_PLL2\_CONTROL.BYPASSB control bit.
5. Set the MISCREG\_PLL2\_CONTROL.DDRCLK\_FROM\_3RDPLL bit to connect the third PLL DCLK to DMC.

This bit determines whether the DCLK is sourced from the third PLL.

## ADSP-SC59x CGU Register Descriptions

Clock Generation Unit (CGU) contains the following registers.

Table 2-8: ADSP-SC59x CGU Register List

| Name          | Description                                      |
|---------------|--------------------------------------------------|
| CGU_CCBF_DIS  | Core Clock Buffer Disable Register               |
| CGU_CCBF_STAT | Core Clock Buffer Status Register                |
| CGU_CLKOUTSEL | CLKOUT Select Register                           |
| CGU_CTL       | Control Register                                 |
| CGU_DIV       | Clocks Divisor Register                          |
| CGU_DIVEX     | DIV Register Extension                           |
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

## Core Clock Buffer Disable Register

The CGU\_CCBF\_DIS register controls each core's clock buffer to determine if the CCLK is enabled.

Figure 2-4: CGU\_CCBF\_DIS Register Diagram

<!-- image -->

Table 2-9: CGU\_CCBF\_DIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If set (=1) the CGU_CCBF_DIS.LOCK bit locks the CGU_CCBF_DIS register. 0 Unlock register                                |
| 0 (R/W)            | CCBF0      | Core Clock Buffer 0. The CGU_CCBF_DIS.CCBF0 bit enables (=0) or disables (=1) CCLK0s buffer. 0 Enable buffer 1 Disable buffer |

## Core Clock Buffer Status Register

The CGU\_CCBF\_STAT register shows which core clock buffer(s) are disabled. For example clearing the CGU\_CCBF\_DIS.CCBF0 bit clears the CGU\_CCBF\_STAT.CCBF0 bit after a number of cycles. To guarantee that the correct value is read, this register should be read twice and the second result used.

Figure 2-5: CGU\_CCBF\_STAT Register Diagram

<!-- image -->

Table 2-10: CGU\_CCBF\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | CCBF0      | Core Clock Buffer 0. The CGU_CCBF_STAT.CCBF0 bit reports the status of the CGU_CCBF_DIS.CCBF0 bit where 0 = enabled and 1 = disabled. 0 Enabled |
| 0 (R/NW)           | CCBF0      | 1 Disabled                                                                                                                                      |
| 0 (R/NW)           | CCBF0      |                                                                                                                                                 |
| 0 (R/NW)           | CCBF0      |                                                                                                                                                 |

## CLKOUT Select Register

The CGU\_CLKOUTSEL selects the signal that the CGU drives through the CLKOUT multiplexer.

Figure 2-6: CGU\_CLKOUTSEL Register Diagram

<!-- image -->

Table 2-11: CGU\_CLKOUTSEL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the CGU_CLKOUTSEL.LOCK bit is set, the CGU_CLKOUTSEL register is read only (locked). |
| 31 (R/W)           | LOCK       | 0 Unlock                                                                                                                                            |
| 31 (R/W)           | LOCK       | 1 Lock                                                                                                                                              |
| 4:0 (R/W)          | CLKOUTSEL  | CLKOUT Select. The CGU_CLKOUTSEL.CLKOUTSEL selects the signal that the CGU drives through the CLKOUT pin multiplexer.                               |
| 4:0 (R/W)          | CLKOUTSEL  | 0 CLKIN0                                                                                                                                            |
| 4:0 (R/W)          | CLKOUTSEL  | 1 CLKIN1                                                                                                                                            |
| 4:0 (R/W)          | CLKOUTSEL  | 2 PLLTOP0.SYSCLKO                                                                                                                                   |
| 4:0 (R/W)          | CLKOUTSEL  | 3 PLLTOP0.CCLK0                                                                                                                                     |
| 4:0 (R/W)          | CLKOUTSEL  | 4 CDU.CLKO3 for DDR                                                                                                                                 |
| 4:0 (R/W)          | CLKOUTSEL  | 5 CDU.CLKO4 for CAN                                                                                                                                 |
| 4:0 (R/W)          | CLKOUTSEL  | 6 CDU.CLKO7 for EMAC                                                                                                                                |
| 4:0 (R/W)          | CLKOUTSEL  | 7 CDU.CLKO8 for LP                                                                                                                                  |
| 4:0 (R/W)          | CLKOUTSEL  | 8 PLLTOP1.SYSCLKO                                                                                                                                   |
| 4:0 (R/W)          | CLKOUTSEL  | 9 PLLTOP1.OCLK                                                                                                                                      |
| 4:0 (R/W)          | CLKOUTSEL  | 10 PLLTOP0.OCLK                                                                                                                                     |

Table 2-11: CGU\_CLKOUTSEL Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration |                        |
|-----------|------------|---------------------------|------------------------|
| (Access)  |            |                           |                        |
|           |            |                        11 | Reserved               |
|           |            |                        12 | Reserved               |
|           |            |                        13 | Reserved               |
|           |            |                        14 | PLLTOP0.SCLK1          |
|           |            |                        15 | Reserved               |
|           |            |                        16 | PLLTOP0.SCLK0          |
|           |            |                        17 | Reserved               |
|           |            |                        18 | Reserved               |
|           |            |                        19 | CDU.CLKO2 for ARM Core |
|           |            |                        20 | GND                    |

## Control Register

The CGU\_CTL controls the clock generation divisors for SYS\_CLKIN and the PLL. Read after write accesses to the CGU\_CTL register returns the new value even if the clock's frequency change is still in progress.

Figure 2-7: CGU\_CTL Register Diagram

<!-- image -->

Table 2-12: CGU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the CGU_CTL.LOCK bit is set, the CGU_CTL register is read only (locked).                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 30 (R/W)           | WFI        | Wait For Idle. Modifying the PLL multiplier requires the PLL to re-lock and once the PLL locks, clocks have to be synchronized. Changes to the CGU_CTL.MSEL and the CGU_CTL.DF bit values results in bypassing the PLL. The CGU_CTL.WFI bit forces the PLL to wait for all processor cores to be in an idle or reset state before changing frequencies as a result of changes to the CGU_CTL.MSEL or CGU_CTL.DF bits. Write accesses to the CGU_CTL to change the CGU_CTL.DF or CGU_CTL.MSEL bit values while the PLL is locking sets the CGU_STAT.WDFMSERR bit. 0 Update Immediately for Idle |
| 30 (R/W)           |            | 1 Wait                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 30 (R/W)           |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 2-12: CGU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | S1SELEXEN  | SCLK1 Extension Divider Enable. The CGU_CTL.S1SELEXEN directs the CGU to select S1SELEX instead of S1SEL. This bit can only be updated when the PLL is bypassed.                                    |
| 16 (R/W)           | S0SELEXEN  | SCLK0 Extension Divider Enable. The CGU_CTL.S0SELEXEN directs the CGU to select S0SELEX instead of S0SEL. This bit can only be updated when the PLL is bypassed.                                    |
| 14:8 (R/W)         | MSEL       | Multiplier Select. The CGU_CTL.MSEL bit field selects the multiplier in the PLLCLK equation: PLLCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL Where the value of MSEL is between 1 and 127. |
| 0 (R/W)            | DF         | Divide Frequency. The CGU_CTL.DF bit selects whether or not the CLKIN input is divided by two be- fore being passed to the PLL. 0 Pass OSC_CLKIN to PLL                                             |

## Clocks Divisor Register

The CGU\_DIV register controls clock divisors for core clocks, system clocks, external (off core) memory clocks, and output clock. Read after write accesses to the CGU\_DIV register returns the new value even if the clock's frequency change is still in progress.

Figure 2-8: CGU\_DIV Register Diagram

<!-- image -->

Table 2-13: CGU\_DIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the CGU_DIV.LOCK bit is set, the CGU0_DIV and CGU0_DIVEX registers are read only (locked). 0 Unlock                                                                                                         |
| 30 (R/W)           | UPDT       | Update Clock Divisors. The CGU_DIV.UPDT controls whether the CGU drives new CGU_DIV.CSEL , CGU_DIV.SYSSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , CGU_DIV.DSEL , and CGU_DIV.OSEL values to PLL after CGU_DIV register update. 0 No PLL Update 1 Drive Updated SEL Values to PLL |

Table 2-13: CGU\_DIV Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R0/W)          | ALGN       | Align. The CGU_DIV.ALGN directs the CGU to align the PLL-based clocks. The divisor se- lections ( CGU_DIV.CSEL , CGU_DIV.SYSSEL , CGU_DIV.S0SEL , CGU_DIV.S1SEL , CGU_DIV.DSEL , and/or CGU_DIV.OSEL ) do not have to change. |
| 29 (R0/W)          | ALGN       | 0 No Action                                                                                                                                                                                                                   |
| 29 (R0/W)          | ALGN       | 1 Align PLL Clocks                                                                                                                                                                                                            |
| 28:22 (R/W)        | OSEL       | OCLK Divisor. The CGU_DIV.OSEL selects the divisor in the OCLK equation: OCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL / CGU_DIV.OSEL Where the value of CGU_DIV.OSEL is between 1 and 127.                          |
| 28:22 (R/W)        | OSEL       | 0 OSEL = 128                                                                                                                                                                                                                  |
| 28:22 (R/W)        | OSEL       | 1-127 OSEL = 1 to 127                                                                                                                                                                                                         |
| 20:16 (R/W)        | DSEL       | XCLK Divisor. The CGU_DIV.DSEL selects the divisor in the XCLK equation: XCLK frequency = (SYS_CLKIN frequency/(DF+1)) MSEL/ CGU_DIV.DSEL Where the value of CGU_DIV.DSEL is between 1 and 31.                                |
| 20:16 (R/W)        | DSEL       | 0 DSEL = 32                                                                                                                                                                                                                   |
| 20:16 (R/W)        | DSEL       | 1-31 DSEL = 1 to 31                                                                                                                                                                                                           |
| 15:13 (R/W)        | S1SEL      | SCLK 1 Divisor. The CGU_DIV.S1SEL selects the divisor in the SCLK1 equation: SCLK1 frequency = (SYSCLK frequency) / CGU_DIV.S1SEL Where the value of CGU_DIV.S1SEL is between 1 and 7. S1SEL = 8                              |
| 15:13 (R/W)        | S1SEL      | 0                                                                                                                                                                                                                             |
| 15:13 (R/W)        | S1SEL      | 1-7 S1SEL = 1 to 7                                                                                                                                                                                                            |
| 12:8 (R/W)         | SYSSEL     | SYSCLK Divisor. The CGU_DIV.SYSSEL selects the divisor in the SYSCLK equation: SYSCLK frequency = (SYS_CLKIN frequency/(DF+1)) MSEL/ CGU_DIV.SYSSEL Where the value of CGU_DIV.SYSSEL is between 1 and 31. SYSSEL = 32        |
| 12:8 (R/W)         | SYSSEL     | 0                                                                                                                                                                                                                             |
| 12:8 (R/W)         | SYSSEL     | 1-31 SYSSEL = 1 to 31                                                                                                                                                                                                         |

Table 2-13: CGU\_DIV Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:5 (R/W)          | S0SEL      | SCLK 0 Divisor. The CGU_DIV.S0SEL selects the divisor in the SCLK0 equation: SCLK0 frequency = (SYSCLK frequency) / CGU_DIV.S0SEL Where the value of CGU_DIV.S0SEL is between 1 and 7. 0 S0SEL = 8                                |
| 4:0 (R/W)          | CSEL       | CCLK Divisor. The CGU_DIV.CSEL bit field selects the divisor in the CCLK equation: CCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL / CGU_DIV.CSEL Where the value of CGU_DIV.CSEL is between 1 and 31. 0 CSEL = 32 1 to 31 |
| 4:0 (R/W)          | CSEL       | 1-31 CSEL=                                                                                                                                                                                                                        |
| 4:0 (R/W)          | CSEL       |                                                                                                                                                                                                                                   |

## DIV Register Extension

The CGU\_DIVEX register contains bitfields for programming the divisor values of the system clock [SCLK0 and SCLK1] in extended divisor mode.

Figure 2-9: CGU\_DIVEX Register Diagram

<!-- image -->

Table 2-14: CGU\_DIVEX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | S1SELEX    | SCLK 1 Extension Divisor. The CGU_DIVEX.S1SELEX selects the divisor in the SCLK1EX equation: SCLK1EX frequency = (PLLCLK frequency) / CGU_DIVEX.S1SELEX Where the value of CGU_DIVEX.S1SELEX is between 1 and 255. 0 S1SELEX = 256        |
| 7:0 (R/W)          | S0SELEX    | SCLK 0 Extension Divisor. The CGU_DIVEX.S0SELEX selects the divisor in the SCLK0EX equation: SCLK0EX frequency = (PLLCLK frequency) / CGU_DIVEX.S0SELEX Where the value of CGU_DIVEX.S0SELEX is between 1 and 255. 0 S1SELEX = 256 to 255 |
| 7:0 (R/W)          | S0SELEX    | 1-255 S1SELEX = 1                                                                                                                                                                                                                         |
| 7:0 (R/W)          | S0SELEX    |                                                                                                                                                                                                                                           |

## Oscillator Watchdog Register

The CGU\_OSCWDCTL register configures the CGU to allow the detection of the absence of input clock transitions and provides a fault warning via the SYS\_FAULT pin. The CGU\_OSCWDCTL register also detects and reports input oscillator frequencies above and below specified limits, in order to specifically detect harmonic or sub-harmonic crystal oscillator behavior. This detection is achieved by using an internal asynchronous, local 1 MHz oscillator combined with a series of programmable counters.

Figure 2-10: CGU\_OSCWDCTL Register Diagram

<!-- image -->

Table 2-15: CGU\_OSCWDCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                           |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK        | Lock.                                                                                                                                             |
| 23 (R/W)           | FAULTPINDIS | Fault Pin disabled. The CGU_OSCWDCTL.FAULTPINDIS bit disables pin fault detection.                                                                |
| 15 (R/W)           | MONDIS      | Oscillator Watchdog Monitor functions disabled. The CGU_OSCWDCTL.MONDIS bit disables all the input clock monitor and fault de- tection functions. |
| 14 (R/W)           | FAULTEN     | Fault enabled. The CGU_OSCWDCTL.FAULTEN bit enables fault detection.                                                                              |
| 13 (R/W)           | BOUEN       | Bad Oscillator Upper Frequency limit detection enabled. The CGU_OSCWDCTL.BOUEN bit enables upper limit bad oscillation detection.                 |

Table 2-15: CGU\_OSCWDCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:8 (R/W)         | BOUF       | Bad Oscillator Upper Frequency limit. The CGU_OSCWDCTL.BOUF bits indicate the desired upper fail limit for the bad os- cillation detection.                     |
| 7 (R/W)            | CNGEN      | Clock not Good enabled. The CGU_OSCWDCTL.CNGEN bit enables the detection of an oscillator watchdog clock fault.                                                 |
| 6 (R/W)            | HODEN      | Harmonic Oscillation Detection enabled. The CGU_OSCWDCTL.HODEN bit enables harmonic oscillation detection.                                                      |
| 5:0 (R/W)          | HODF       | Watchdog lower frequency limit. The CGU_OSCWDCTL.HODF bit field is used to indicate the desired lower fail limit for the harmonic oscillation detection in MHz. |

## PLL Control Register

The CGU\_PLLCTL register contains bits that enable and disable the PLL as well as control its function.

Figure 2-11: CGU\_PLLCTL Register Diagram

<!-- image -->

Table 2-16: CGU\_PLLCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. Setting (=1) the CGU_PLLCTL.LOCK bit locks access to the CGU_PLLCTL register.                                                                                         |
| 3 (R/W)            | PLLEN      | PLL Enable. Setting (=1) the CGU_PLLCTL.PLLEN bit enables the PLL. Check the CGU_STAT.PLLEN bit to verify that the action is complete. 0 No action                          |
| 2 (R/W)            | PLLDIS     | PLL Disable. Setting (=1) the CGU_PLLCTL.PLLDIS bit disables the PLL. Check the CGU_STAT.PLLEN bit to verify that the action is complete.                                   |
| 1 (R/W)            | PLLBPCL    | PLL Bypass Clear. Setting (=1) the CGU_PLLCTL.PLLBPCL bit takes the PLL out of bypass mode. Check the CGU_STAT.PLLBP bit to verify that the action is complete. 0 No action |

Table 2-16: CGU\_PLLCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | PLLBPST    | PLL Bypass Set. Setting (=1) the CGU_PLLCTL.PLLBPST bit bypasses the PLL and all the clocks run on CLKIN. Check the CGU_STAT.PLLBP bit to verify that the action is com- plete. |
| 0 (R/W)            | PLLBPST    | 0 Use PLL                                                                                                                                                                       |
| 0 (R/W)            | PLLBPST    | 1 Bypass PLL                                                                                                                                                                    |

## Revision ID Register

The CGU\_REVID register reports the version of the CGU.

Figure 2-12: CGU\_REVID Register Diagram

<!-- image -->

Table 2-17: CGU\_REVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | MAJOR      | Major Version.            |
| 3:0 (R/NW)         | REV        | Incremental Version ID.   |

## System Clock Buffer Disable Register

The CGU\_SCBF\_DIS register controls each system's clock buffer to determine if the SCLKn buffer is enabled.

Figure 2-13: CGU\_SCBF\_DIS Register Diagram

<!-- image -->

Table 2-18: CGU\_SCBF\_DIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. The CGU_SCBF_DIS.LOCK bit allows writes to the CGU_SCBF_DIS register when cleared (=0) or blocks writes if set (=1) and the SPU_CTL.GLCK bit is set. |
| 3 (R/W)            | OUTCLKBF   | OCLK Buffer. The CGU_SCBF_DIS.OUTCLKBF bit enables (=0, default) or disables (=1) OCLKs buffer.                                                            |
| 2 (R/W)            | DCLKBF     | XCLK Buffer. The CGU_SCBF_DIS.DCLKBF bit enables (=0, default) or disables (=1) XCLKs buffer. 0 Enable buffer                                              |
| 2 (R/W)            |            |                                                                                                                                                            |

Table 2-18: CGU\_SCBF\_DIS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | SCLK1BF    | SCLK1 Buffer. The CGU_SCBF_DIS.SCLK1BF bit enables (=0, default) or disables (=1) SCLK1s buffer. 0 Enable buffer |
| 0 (R/W)            | SCLK0BF    | SCLK0 Buffer. The CGU_SCBF_DIS.SCLK0BF bit enables (=0, default) or disables (=1) SCLK0s buffer. 0 Enable buffer |

## System Clock Buffer Status Register

The CGU\_SCBF\_STAT register shows which system clock buffer(s) are disabled. For example clearing the CGU\_CCBF\_DIS.CCBF0 bit clears the CGU\_SCBF\_STAT.SCLK0BF bit after a number of cycles. To guarantee that the correct value is read, this register should be read twice and the second result used.

Figure 2-14: CGU\_SCBF\_STAT Register Diagram

<!-- image -->

Table 2-19: CGU\_SCBF\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | OCLKBF     | OCLK Buffer. The CGU_SCBF_STAT.OCLKBF bit reports the status of the CGU_SCBF_DIS.OUTCLKBF bit where 0 = enabled and 1 = disabled.            |
| 2 (R/NW)           | DCLKBF     | XCLK1 Buffer. The CGU_SCBF_STAT.DCLKBF bit reports the status of the CGU_SCBF_DIS.DCLKBF bit where 0 = enabled and 1 = disabled.             |
| 1 (R/NW)           | SCLK1BF    | SCLK1 Buffer. The CGU_SCBF_STAT.SCLK1BF bit reports the status of the CGU_SCBF_DIS.SCLK1BF bit where 0 = enabled and 1 = disabled. 0 Enabled |
| 1 (R/NW)           |            |                                                                                                                                              |

Table 2-19: CGU\_SCBF\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | SCLK0BF    | SCLK0 Buffer. The CGU_SCBF_STAT.SCLK0BF bit reports the status of the CGU_SCBF_DIS.SCLK0BF bit where 0 = enabled and 1 = disabled. |
| 0 (R/NW)           | SCLK0BF    | 0 Enabled                                                                                                                          |
| 0 (R/NW)           | SCLK0BF    | 1 Disabled                                                                                                                         |

## Status Register

The CGU\_STAT register reflects the PLL status and errors detected during the PLL configuration. This register may be cleared asynchronously by a reset signal from the RCU module. All bits---except those defined as W1C (write-1to-clear)---are read only.

Figure 2-15: CGU\_STAT Register Diagram

<!-- image -->

Table 2-20: CGU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W1C)         | PCFGERR    | PLL Configuration Error. If the CGU_PLLCTL.PLLBPST and the CGU_PLLCTL.PLLBPCL bits are set (=1) simultaneously or the CGU_PLLCTL.PLLDIS bit was set (=1) in full-on mode or while trying to enter full-on mode ( CGU_PLLCTL.PLLBPCL =1), the CGU_STAT.PCFGERR bit triggers the bus error. |
| 21 (R/W1C)         | PCFGERR    | 0 No Error                                                                                                                                                                                                                                                                                |
| 21 (R/W1C)         | PCFGERR    | 1 Configuration Error                                                                                                                                                                                                                                                                     |

Table 2-20: CGU\_STAT Register Fields (Continued)

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

Table 2-20: CGU\_STAT Register Fields (Continued)

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

Figure 2-16: CGU\_TSCOUNT0 Register Diagram

<!-- image -->

Table 2-21: CGU\_TSCOUNT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | VALUE      | TSCOUNT0 Value.                                                        |
| (R/NW)             |            | The CGU_TSCOUNT0.VALUE bit field holds the time stamp counter 32 LSBs. |

## Time Stamp Counter 32 MSB Register

The CGU\_TSCOUNT1 register address is used to read the CoreSight time stamp counter MSB 32-bit (bits [63:32]) value.

Figure 2-17: CGU\_TSCOUNT1 Register Diagram

<!-- image -->

Table 2-22: CGU\_TSCOUNT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | VALUE      | Timestamp Counter 32 MSB.                                              |
| (R/NW)             |            | The CGU_TSCOUNT1.VALUE bit field holds the time stamp counter 32 MSBs. |

## Time Stamp Control Register

The CGU\_TSCTL register controls the operation of the CoreSight time stamp counter.

Figure 2-18: CGU\_TSCTL Register Diagram

<!-- image -->

Table 2-23: CGU\_TSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. Setting the CGU_TSCTL.LOCK bit locks this register. 0 Unlock                                                                                                          |
| 7:4 (R/W)          | TSDIV      | Counter's Clock Divider. The CGU_TSCTL.TSDIV bit field divides SYSCLK by 2 TSDIV . 0-15 Divides SYSCLK by 2 TSDIV                                                           |
| 1 (R/W)            | LOAD       | Load Counter. Writing one to the CGU_TSCTL.LOAD bit causes CoreSight time stamp counter to be loaded from the CGU_TSVALUE0 and CGU_TSVALUE1 registers. 0 Always read as "0" |
| 0 (R/W)            | EN         | Counter Enable. The CGU_TSCTL.EN bit enables or disables the CoreSight time stamp counter. 0 Counter Disabled                                                               |

## Time Stamp Counter Initial 32 LSB Value Register

The CGU\_TSVALUE0 register holds the least significant bits (bits [31:0]) value that is initially loaded to the CoreSight time stamp counter.

Figure 2-19: CGU\_TSVALUE0 Register Diagram

<!-- image -->

Table 2-24: CGU\_TSVALUE0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Counter's 32 LSB Initial Value. The CGU_TSVALUE0.VALUE bit field holds the LSBs value that is initially loaded to the CoreSight time stamp counter. |

## Time Stamp Counter Initial MSB Value Register

The CGU\_TSVALUE1 register holds the most significant bits (bits [63:32]) value that is initially loaded to the CoreSight time stamp counter.

Figure 2-20: CGU\_TSVALUE1 Register Diagram

<!-- image -->

Table 2-25: CGU\_TSVALUE1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Counter's Initial 32 MSB Value. The CGU_TSVALUE1.VALUE bit field holds the MSBs value that is initially loaded to the CoreSight time stamp counter. |

## ADSP-SC59x MISCREG Register Descriptions

Misc registers for module are for integration purpose (MISCREG) contains the following registers.

Table 2-26: ADSP-SC59x MISCREG Register List

| Name                         | Description                                   |
|------------------------------|-----------------------------------------------|
| MISCREG_CAN_SYSCTL           | CANFD Low Power Selection Mode                |
| MISCREG_ECO_REG9             | Error Register                                |
| MISCREG_PLL2_CONTROL         | PLL2 Control Register                         |
| MISCREG_SH0_PFB_RANGE_SELECT | SH0 Prefetch Range Selection Register         |
| MISCREG_SH1_PFB_RANGE_SELECT | SH1 Prefetch Range Selection Register         |
| MISCREG_SHARC_BRIDGE_REMAP   | SPORT Direct Interrupt Enable for SH0 and SH1 |

## CANFD Low Power Selection Mode

The MISCREG\_CAN\_SYSCTL register contains the control bits for the different low power modes supported in the CANFD module.

Figure 2-21: MISCREG\_CAN\_SYSCTL Register Diagram

<!-- image -->

Table 2-27: MISCREG\_CAN\_SYSCTL Register Fields

| Bit No. (Access)   | Bit Name               | Description/Enumeration                                                               |
|--------------------|------------------------|---------------------------------------------------------------------------------------|
| 9 (R/W)            | CAN1_IPG_STOP_CLK_ EXT | external ClockGating enable for StopMode for CAN1.                                    |
| 8 (R/W)            | CAN0_IPG_STOP_CLK_ EXT | external ClockGating enable for StopMode for CAN0.                                    |
| 7 (R/W)            | CAN1_IPG_SRST          | Soft reset to can_node_1 ; 1: set can1_ipg_srst , 0: clear can1_ipg_srst.             |
| 6 (R/W)            | CAN1_IPG_DEBUG         | Debug mode request to can_node_1 ; 1: set can1_ipg_debug , 0: clear can1_ipg_de- bug. |
| 5 (R/W)            | CAN1_IPG_STOP          | Stop mode request to can_node_1 ; 1: set can1_ipg_stop , 0: clear can1_ipg_stop.      |
| 4 (R/W)            | CAN1_IPG_DOZE          | Doze mode request to can_node_1 ; 1: set can1_ipg_doze , 0: clear can1_ipg_doze.      |

Table 2-27: MISCREG\_CAN\_SYSCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                               |
|--------------------|----------------|---------------------------------------------------------------------------------------|
| 3 (R/W)            | CAN0_IPG_SRST  | Soft reset to can_node_0 ; 1: set can0_ipg_soft_reset , 0: clear can0_ipg_soft_reset. |
| 2 (R/W)            | CAN0_IPG_DEBUG | Debug mode request to can_node_0 ; 1: set can0_ipg_debug , 0: clear can0_ipg_de- bug. |
| 1 (R/W)            | CAN0_IPG_STOP  | Stop mode request to can_node_0 ; 1: set can0_ipg_stop , 0: clear can0_ipg_stop.      |
| 0 (R/W)            | CAN0_IPG_DOZE  | Doze mode request to can_node_0 ; 1: set can0_ipg_doze , 0: clear can0_ipg_doze.      |

## Error Register

The MISCREG\_ECO\_REG9 register contains eight Tx channels, eight Rx channels, and DMA errors for both EMAC0 and EMAC1.

Figure 2-22: MISCREG\_ECO\_REG9 Register Diagram

<!-- image -->

Table 2-28: MISCREG\_ECO\_REG9 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:18              | ECO_VALUE9 | .                         |
| (R/W)              |            |                           |

Table 2-28: MISCREG\_ECO\_REG9 Register Fields (Continued)

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                               |
|--------------------|-------------------|-----------------------------------------------------------------------|
| 17 (R/W1)          | EMAC1_DMA_RX_ERR  | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 16 (R/W1)          | EMAC1_DMA_TX_ERR  | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 15 (R/W1)          | EMAC0_DMA7_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 14 (R/W1)          | EMAC0_DMA6_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 13 (R/W1)          | EMAC0_DMA5_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 12 (R/W1)          | EMAC0_DMA4_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 11 (R/W1)          | EMAC0_DMA3_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 10 (R/W1)          | EMAC0_DMA2_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 9 (R/W1)           | EMAC0_DMA1_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 8 (R/W1)           | EMAC0_DMA0_RX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 7 (R/W1)           | EMAC0_DMA7_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 6 (R/W1)           | EMAC0_DMA6_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 5 (R/W1)           | EMAC0_DMA5_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 4 (R/W1)           | EMAC0_DMA4_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 3 (R/W1)           | EMAC0_DMA3_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 2 (R/W1)           | EMAC0_DMA2_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |

Table 2-28: MISCREG\_ECO\_REG9 Register Fields (Continued)

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                               |
|--------------------|-------------------|-----------------------------------------------------------------------|
| 1 (R/W1)           | EMAC0_DMA1_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |
| 0 (R/W1)           | EMAC0_DMA0_TX_ERR | DMAreceived a 2nd start (stop) trigger before a stop (start) trigger. |

## PLL2 Control Register

Figure 2-23: MISCREG\_PLL2\_CONTROL Register Diagram

<!-- image -->

Table 2-29: MISCREG\_PLL2\_CONTROL Register Fields

| Bit No. (Access)   | Bit Name            | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|---------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | PLLEN               | Control bit for 3rd PLL's pllen input. Internal signal from Dynamic Power Management Unit (DPM) which enables/disables the PLL system (e.g. in Active mode, during BYPASS.)                                                              |
| 17 (R/W)           | DCLKEN              | Control bit for 3rd PLL's dclken input. Enables DCLK clock buffer                                                                                                                                                                        |
| 16:12 (R/W)        | DSEL                | Control bits for 3rd PLL's dsel input. Selects DCLK divide ratio                                                                                                                                                                         |
| 11 (R/W)           | DDRCLK_FROM_3RDPL L | 3rd PLL DCLK Output is connected to DMCblock when this bit is set. When cleared, DDR clock is CLKO[3] output of CDU. 3rd PLL DCLK Output is connected to DMCblock when this bit is set. When cleared, DDR clock is CLKO[3] output of CDU |
| 10:4 (R/W)         | MSEL                | Control bits for 3rd PLL's msel input. The MISCREG_PLL2_CONTROL.MSEL selects the multiplier in the PLLCLK equation: PLLCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL * 2 Where the value of MSEL is between 1 and 127.           |

Table 2-29: MISCREG\_PLL2\_CONTROL Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                     |
|--------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | DSEL_DIV_CHG | Control bit for 3rd PLL's dsel_chg and div_chg inputs. Signal from PCU indicating a DDR clock frequency change                                              |
| 1 (R/W)            | MSELDF_CHG   | Control bit for 3rd PLL's msel_chg input. Signal from PLL Control Unit indicating a frequency change resulting from a change in either the DF or MSEL bits. |
| 0 (R/W)            | BYPASSB      | Control bit for 3rd PLL's bypassb input. Active-low control bit to bypass the PLL which forces CLKIN on all output clocks                                   |

## SH0 Prefetch Range Selection Register

In the MISCREG\_SH0\_PFB\_RANGE\_SELECT register, each bit field controls if the corresponding address range will be prefetched. When an address range is set it is prefetched and when it is zero there is no prefetch.

The MISCREG\_SH0\_PFB\_RANGE\_SELECT [31:16] bits control the range selection for the instruction cache, and MISCREG\_SH0\_PFB\_RANGE\_SELECT [15:0] bits control the range selection for the data cache.

Figure 2-24: MISCREG\_SH0\_PFB\_RANGE\_SELECT Register Diagram

<!-- image -->

Table 2-30: MISCREG\_SH0\_PFB\_RANGE\_SELECT Register Fields

| Bit No. (Access)   | Bit Name             | Description/Enumeration                                         |
|--------------------|----------------------|-----------------------------------------------------------------|
| 31:16 (R/W)        | IPORT_RANGE_SELECT   | IPORT Prefetch Range Select. Prefetch Range Selection for IPORT |
| 15:0 (R/W)         | DPORT_RANGE_SE- LECT | DPORT Prefetch Range Select. Prefetch Range Selection for DPORT |

## SH1 Prefetch Range Selection Register

Each bit field controls if the corresponding address range will be prefetched. When an address range is set it is prefetched an when it is zero there is no prefetch.

The MISCREG\_SH1\_PFB\_RANGE\_SELECT [31:16] bits control the range selection for the instruction cache, and MISCREG\_SH1\_PFB\_RANGE\_SELECT [15:0] bits control the range selection for the data cache.

Figure 2-25: MISCREG\_SH1\_PFB\_RANGE\_SELECT Register Diagram

<!-- image -->

Table 2-31: MISCREG\_SH1\_PFB\_RANGE\_SELECT Register Fields

| Bit No. (Access)   | Bit Name                 | Description/Enumeration                                             |
|--------------------|--------------------------|---------------------------------------------------------------------|
| 31:16 (R/W)        | SH1_IPORT_RANGE_SE- LECT | SH1 IPORT Prefetch Range Select. Prefetch Range Selection for IPORT |
| 15:0 (R/W)         | SH1_DPORT_RANGE_SE LECT  | SH1 DPORT Prefetch Range Select. Prefetch Range Selection for DPORT |

## SPORT Direct Interrupt Enable for SH0 and SH1

The bits in the MISCREG\_SHARC\_BRIDGE\_REMAP register control the direct connection of SPORT interrupts to the SHARC core.

Figure 2-26: MISCREG\_SHARC\_BRIDGE\_REMAP Register Diagram

<!-- image -->

Table 2-32: MISCREG\_SHARC\_BRIDGE\_REMAP Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                            |
|--------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | SH1_DRT_INTR_EN | SPORT Direct Interrupt Enable for SH1. If the MISCREG_SHARC_BRIDGE_REMAP.SH1_DRT_INTR_EN bit is set a di- rect connection is enabled for SH1. Do not set the MISCREG_SHARC_BRIDGE_REMAP.SH1_DRT_INTR_EN bit when IIR1 of the SH1 is in use. If this bit is zero the direct connection is disabled. |
| 16 (R/W)           | SH0_DRT_INTR_EN | SPORT Direct Interrupt Enable for SH0. If the MISCREG_SHARC_BRIDGE_REMAP.SH0_DRT_INTR_EN bit is set a di- rect connection is enabled for SH0. Do not set the MISCREG_SHARC_BRIDGE_REMAP.SH0_DRT_INTR_EN bit when IIR1 of the SH0 is in use. If this bit is zero the direct connection is disabled. |