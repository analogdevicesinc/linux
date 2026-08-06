# Reset Control Unit (RCU)

<!-- source: 008_Reset_Control_Unit_RCU.pdf | original pages 185–210 -->

## 6   Reset Control Unit (RCU)

Reset is the initial state of the processor at power-on or the run-time state of any core, as controlled by another core in the device using the RCU or as a result of a hardware or software triggered event. In this state, all control registers are set to their default values and functional units are idle. Exiting a full system hardware reset results with only the host core being released from reset and ready to boot (Core 0 on the devices ADSP-SC5x, Core 1 on the ADSP-215xx devices).

The following table shows the cores that are released from reset. Exiting a Core n only reset starts with this Core n being ready to execute the code from the software vector registers ( RCU\_SVECT0 , RCU\_SVECT2 ).

Additional information on reset and booting can be found in the Boot ROM and Booting the Processor chapter.

| Product    | Core Released from Reset by RCU   |
|------------|-----------------------------------|
| ADSP-SC59x | Core 0                            |
| ADSP-2159x | Core 1                            |

The Reset Control Unit (RCU) controls how all the functional units enter and exit reset. Differences in functional requirements and clocking constraints define how reset signals are generated. Programs must guarantee that none of the reset functions puts the system into an undefined state or causes resources to stall. This functionality is important when only one of the cores is reset (programs must ensure that there is no pending system activity involving the core that is being reset). While core resets and software system resets are controlled directly in the RCU, hardware resets can come from the TRU, SEC, or CGU Oscillator Watchdog.

## RCU Features

The RCU module supports the following features:

- Hardware reset through the SYS\_HWRST pin
- Software system reset through the RCU control ( RCU\_CTL ) register
- Hardware system reset through:
- TRU module
- SEC module (System Fault Unit )

- A clock not good reset state (safe state of chip under reset) from the Oscillator Watchdog.
- Core reset through RCU Core Reset Output ( RCU\_CRCTL ) register

## RCU Functional Description

This section provides information on the function of RCU module.

## Hardware reset using SYS\_HWRST pin

Asserting the SYS\_HWRST pin resets all functional units, except the real time clock (RTC) (if present).

## Hardware reset through RCU

The RCU can perform a full system reset which can be initiated through hardware blocks like the SEC, the TRU, and the oscillator watchdog.

## Software reset using RCU registers

Setting the RCU\_STAT.SWRST bit issues a software reset for all system units except the RTC module and RCU\_BCODE , RCU\_CRCTL and RCU\_STAT registers.

## Core reset RCU registers

A core can be individually reset by software, or by setting the RCU\_CRCTL.CR[n] bit.

## ADSP-2159x\_SC592\_SC594 RCU Register List

The Reset Control Unit (RCU) controls how all the functional units in the processor enter and exit Reset. Differences in functional requirements and clocking constraints exist (units in different clock domains have to enter reset asynchronously, but units exit reset in a deterministic way), and these differences define how reset signals are generated. Reset signals propagate through all functional units asynchronously. For more information on RCU functionality, see the RCU register descriptions.

Table 6-1: ADSP-2159x\_SC592\_SC594 RCU Register List

| Name        | Description                         |
|-------------|-------------------------------------|
| RCU_BCODE   | Boot Code Register                  |
| RCU_CRCTL   | Core Reset Outputs Control Register |
| RCU_CRSTAT  | Core Reset Outputs Status Register  |
| RCU_CTL     | Control Register                    |
| RCU_MSG     | Message Register                    |
| RCU_MSG_CLR | Message Clear Bits Register         |

Table 6-1: ADSP-2159x\_SC592\_SC594 RCU Register List (Continued)

| Name          | Description                          |
|---------------|--------------------------------------|
| RCU_MSG_SET   | Message Set Bits Register            |
| RCU_SRRQSTAT  | System Reset Request Status Register |
| RCU_STAT      | Status Register                      |
| RCU_SVECT0    | Software Vector Register 0           |
| RCU_SVECT1    | Software Vector Register 1           |
| RCU_SVECT2    | Software Vector Register 2           |
| RCU_SVECT_LCK | SVECT Lock Register                  |

## ADSP-2159x\_SC592\_SC594 RCU Trigger List

Table 6-2: ADSP-2159x\_SC592\_SC594 RCU Trigger List Generators

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

Table 6-3: ADSP-2159x\_SC592\_SC594 RCU Trigger List Receivers

|   Trigger ID | Name         | Description               | Sensitivity   |
|--------------|--------------|---------------------------|---------------|
|           50 | RCU0_SYSRST0 | RCU0 System Reset Slave 0 | Pulse         |
|           51 | RCU0_SYSRST1 | RCU0 System Reset Slave 1 | Pulse         |

## RCU Definitions

To make the best use of the RCU, it is useful to understand the terms in this section.

The target or source defines the following are types of resets.

## Hardware Reset (by target)

All functional units except a small subsection of debug interfaces are set to their default states. State is lost in all nonvolatile storage.

## System Reset (by target)

All functional units except the RCU, flash interface, and debug are set to their default states.

## Core n Only Reset (by target)

Affects Core n only. The system software must guarantee that a bus requester cannot access the core in reset state.

## Hardware Reset (by source)

The SYS\_HWRST input signal is asserted active (pulled low).

## System Reset (by source)

Software can trigger the reset by writing to the RCU\_CTL register or by another functional unit such as the TRU or any of the generic reset inputs.

## RCU Architectural Concepts

To understand the architecture of the RCU, it is important to consider the reset sources and how differing resets affect the functional units of the processor.

The RCU provides the hardware that controls how all the functional units enter and exit reset. Differences in functional requirements and clocking constraints define how reset signals are generated. For example, units in different clock domains must enter reset asynchronously but exit reset in a deterministic way.

The program must guarantee that none of the reset functions put the system in an undefined state or cause resources to stall. This functionality is important when only one of the cores is reset. The program must guarantee that there is no pending system activity involving Core n before it is reset. For example, there must be no pending transactions to core 0 when the core 0 is reset and vice-versa.

The RCU Reset Sources table defines how reset sources affect the different functional units.

Table 6-4: RCU Reset Sources

| Reset Source                                                              | Reset Type      | Affected Functional Units                                                                                      |
|---------------------------------------------------------------------------|-----------------|----------------------------------------------------------------------------------------------------------------|
| SYS_HWRST pin assertion                                                   | Hardware Reset  | All functional units, except RTC (if present)                                                                  |
| SYSCLK clock domain system reset by the Fault Unit (FMU) or TRU completer | System Reset    | All functional units, except: • RTC (if present) • RCU_STAT • RCU_BCODE • the units on the VDDEXT power domain |
| RCU_CTL.SYSRST bit set (software triggered reset)                         | System Reset    | All functional units, except: • RTC (if present) • RCU_STAT • RCU_BCODE • the units on the VDDEXT power domain |
| RCU_CRCTL.CR[n] bit set (software triggered reset)                        | Core Only Reset | Core n only , for (2 ≥ n ≥ 0)                                                                                  |

## RCU Status and Error Signals

The RCU\_STAT register reflects status and error information. There are three kinds of errors that can occur in the RCU. The reset out error is triggered when RSTOUT is both asserted and deasserted at the same time. The lock write error occurs if an attempt is made to write a lock RCU register. The address error occurs if a read-only register is written to or if an attempt is made to a reserved address within the RCU MMR address range.

## Resetting the Arm Core through Another Core/System Requester

The RCU allows reset of a given core n using another core or system requester. Core 0 can be individually reset by software, either setting any of CR0 bit in the RCU\_CRCTL register. Cores that reset themselves cannot guarantee that all the system transactions to or from it have completed. Although a core n reset can be triggered by core n itself, it is recommended that another core or system requester trigger it. Core n can be reset to restore its functionality when it cannot execute software.

The following steps show the suggested programming sequence to reset core n only.

1. Clear the RCU\_CRSTAT.CR[n] bit.
2. Disable interrupts to core n.
3. Set the RCU\_CRCTL.CR[n] bit to reset core n.
4. Poll the RCU\_CRSTAT.CR[n] bit until core n is in reset.
5. Clear the RCU\_CRCTL.CR[n] bit to take core n out of reset.
6. Poll the RCU\_CRSTAT.CR[n] bit until core n is out of reset.

## Resetting a SHARC+ Core Through Another Core

Resetting a SHARC+ core involves a software handshake between the requester core which issues a reset to the SHARC+ core. The handshake is done using a core interrupt along with message passing through a variable in shared L2 memory RAM. Each SHARC+ core needs two bits for handshaking.

- Core Reset Request bit (CRR). This bit is set by the requesting core to indicate to the SHARC+ core that a core reset needs to be done. When the CRR bit is set the SHARC+ core should disable all interrupts, stop all system and memory accesses, and enter the IDLE state.
- IDLE acknowledgement (IDLE). Once the SHARC+ core is ready for reset, it sets this bit before entering the IDLE state to inform the requesting core that it is ready for reset.

Two bits are needed for SHARC0 core, and two bits are needed for SHARC1 core. These bits are:

- CCR0-SHARC0 core reset request bit in shared variable

- IDLE0-SHARC0 IDLE acknowledgement bit in shared variable
- CCR1-SHARC1 core reset request bit in shared variable
- IDLE1-SHARC1 IDLE acknowledgement bit in shared variable

Use the following programming to reset the SHARC+ core.

1. The requesting core checks the IDLE status bits in shared variable for the corresponding SHARC+ core ( RCU\_MSG.C1IDLE and RCU\_MSG.C0IDLE ).
2. When one of the core idle bits is set, then the program jumps to step 10. Otherwise the process continues to step 3.
3. The requesting core sets the CCRx bit in message register and raises the SOFT0 software interrupt through the SEC to the SHARC+ core (see Programming Examples for more information).
4. The SHARC+ core goes to into the ISR and checks the 'CCRx' status bit in RCU\_MSG register to ensure that software interrupt is for a core reset.
5. ADDITIONAL INFORMATION: The SOFT0 software interrupt handler can be used for other non-reset/ general purposes as well. ADDITIONAL INFORMATION: The ISR should be in L1 memory.
5. The core should disable all interrupts before entering IDLE for reset.
6. The core sets the appropriate idle status bit ( RCU\_MSG.C1IDLE and RCU\_MSG.C0IDLE ).
7. Core enters IDLE.
8. The requester polls for the IDLEx status bit
9. The requester keeps a timeout option where if within N number of cycles the SHARC core doesn't respond then a system reset is initiated.

ADDITIONAL INFORMATION: N can be decided by the user depending on timing criticality of the application.

10. Once the IDLEx status bit is found set, the requesting core initiates a core reset though the RCU.
11. Clear the RCU\_CRSTAT.CR[n] bit.
12. Set the bit to disable the interfaces for core n, to stop DMA accesses to its L1, to stop accesses to memory for core n, and stop accesses to MMRs.
13. Test the bit to detect when accesses to core n have been disabled and all the pending transactions have completed.
14. Set the RCU\_CRCTL.CR[n] bit to reset core n.
15. Poll the RCU\_CRSTAT.CR[n] bit until core n is in reset.

16. Once the core is in reset, clear the bit to re-enable the core interfaces.
17. Clear the RCU\_CRCTL.CR[n] bit to take core n out of reset.
18. Poll the RCU\_CRSTAT.CR[n] bit until core n is out of reset.

When a core is servicing a higher priority interrupt and gets stuck, it may not respond to the SEC.

## ADSP-2159x\_SC592\_SC594 RCU Register Descriptions

Reset Control Unit (RCU) contains the following registers.

Table 6-5: ADSP-2159x\_SC592\_SC594 RCU Register List

| Name          | Description                          |
|---------------|--------------------------------------|
| RCU_BCODE     | Boot Code Register                   |
| RCU_CRCTL     | Core Reset Outputs Control Register  |
| RCU_CRSTAT    | Core Reset Outputs Status Register   |
| RCU_CTL       | Control Register                     |
| RCU_MSG       | Message Register                     |
| RCU_MSG_CLR   | Message Clear Bits Register          |
| RCU_MSG_SET   | Message Set Bits Register            |
| RCU_SRRQSTAT  | System Reset Request Status Register |
| RCU_STAT      | Status Register                      |
| RCU_SVECT0    | Software Vector Register 0           |
| RCU_SVECT1    | Software Vector Register 1           |
| RCU_SVECT2    | Software Vector Register 2           |
| RCU_SVECT_LCK | SVECT Lock Register                  |

## Boot Code Register

The RCU\_BCODE register can be used to determine if and how core boots. This register is set to its default values by RESET.

Figure 6-1: RCU\_BCODE Register Diagram

<!-- image -->

Table 6-6: RCU\_BCODE Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                              |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK        | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_BCODE.LOCK bit is set, the RCU_BCODE register is read only (locked). 0 Unlock |
| 13 (R/W)           | IDLEONENTRY | Idle On Entry. The RCU_BCODE.IDLEONENTRY bit configures the RCU to enter the idle state at startup. 0 Do not enter idle state 1 Enter idle state     |

Table 6-6: RCU\_BCODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | NOL2CONFIG | No L2 Configuration. The RCU_BCODE.NOL2CONFIG bit configures the RCU to not perform the L2 memory configuration.            |
| 12 (R/W)           | NOL2CONFIG | 0 Configure L2 memory                                                                                                       |
| 12 (R/W)           | NOL2CONFIG | 1 Do not configure L2 memory                                                                                                |
| 10 (R/W)           | NOHOOK     | No Hook. The RCU_BCODE.NOHOOK bit configures the RCU to not perform the hook rou- tine.                                     |
| 10 (R/W)           | NOHOOK     | 0 Perform hook routine                                                                                                      |
| 10 (R/W)           | NOHOOK     | 1 Do not perform hook routine                                                                                               |
| 9 (R/W)            | NOPREBOOT  | No Preboot. The RCU_BCODE.NOPREBOOT bit configures the RCU to not perform the custom- er preboot routine.                   |
| 9 (R/W)            | NOPREBOOT  | 0 Perform preboot                                                                                                           |
| 9 (R/W)            | NOPREBOOT  | 1 Do not perform preboot                                                                                                    |
| 6 (R/W)            | NOFAULTS   | No Faults. The RCU_BCODE.NOFAULTS bit configures the RCU to not perform fault initiali- zation.                             |
| 6 (R/W)            | NOFAULTS   | 0 Perform fault initialization                                                                                              |
| 6 (R/W)            | NOFAULTS   | 1 Do not perform fault initialization                                                                                       |
| 5 (R/W)            | NOCACHE    | No Cache. The RCU_BCODE.NOCACHE bit configures the RCU to not perform a cache initiali- zation and to not enable the cache. |
| 5 (R/W)            | NOCACHE    | 0 Enable and initialize cache                                                                                               |
| 5 (R/W)            | NOCACHE    | 1 Do not initialize or enable cache                                                                                         |
| 4 (R/W)            | NOMEMINIT  | No Memory Initialization. The RCU_BCODE.NOMEMINIT bit configures the RCU to not perform a memory initialization.            |
| 4 (R/W)            | NOMEMINIT  | 0 Perform memory initialization                                                                                             |
| 4 (R/W)            | NOMEMINIT  | 1 Do not perform memory initialization                                                                                      |

Table 6-6: RCU\_BCODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | HBTOVW     | Execute Wakeup. The RCU_BCODE.HBTOVW bit configures the RCU to execute a wakeup.                                            |
| 2 (R/W)            | HALT       | Halt. The RCU_BCODE.HALT bit configures the RCU to execute the no boot routine.                                             |
| 1 (R/W)            | NOVECTINIT | 1 Execute routine No Vector Initialize. The RCU_BCODE.NOVECTINIT bit configures the RCU to not vector to the appli- cation. |
| 0 (R/W)            | NOKERNEL   | 1 Do not vector No Boot Kernel. The RCU_BCODE.NOKERNEL bit configures the RCU to not execute the boot ker- nel.             |
|                    |            | 0 Execute boot kernel                                                                                                       |

## Core Reset Outputs Control Register

The RCU core reset control n registers ( RCU\_CRCTL ) include a lock bit ( RCU\_CRCTL.LOCK ) and a core reset bit ( RCU\_CRCTL.CR[n] ) for each core reset signal on the product.

Figure 6-2: RCU\_CRCTL Register Diagram

<!-- image -->

Table 6-7: RCU\_CRCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CRCTL.LOCK bit is set, the RCU_CRCTL register is read only (locked).                                                                                                            | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CRCTL.LOCK bit is set, the RCU_CRCTL register is read only (locked).                                                                                                            |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                                                                      | Unlock                                                                                                                                                                                                                                                 |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                                                                      | Lock                                                                                                                                                                                                                                                   |
| 3:0 (R/W)          | CR[n]      | Core Reset Outputs. The RCU_CRCTL.CR[n] bits control CRES[1:0] core reset signals. The RCU_CRES[n] signals can be individually controlled. They are reset to their default value by a hard reset or a system reset. For each RCU_CRES[n], the selected | Core Reset Outputs. The RCU_CRCTL.CR[n] bits control CRES[1:0] core reset signals. The RCU_CRES[n] signals can be individually controlled. They are reset to their default value by a hard reset or a system reset. For each RCU_CRES[n], the selected |
| 3:0 (R/W)          | CR[n]      | 0                                                                                                                                                                                                                                                      | RCU_CRES[3:0] Deasserted                                                                                                                                                                                                                               |
| 3:0 (R/W)          | CR[n]      | 1                                                                                                                                                                                                                                                      | Reserved                                                                                                                                                                                                                                               |
| 3:0 (R/W)          | CR[n]      | 2                                                                                                                                                                                                                                                      | RCU_CRES[1] SHARC0 reset control bit asserted                                                                                                                                                                                                          |
| 3:0 (R/W)          | CR[n]      | 4                                                                                                                                                                                                                                                      | RCU_CRES[2] SHARC1 reset control bit asserted                                                                                                                                                                                                          |
| 3:0 (R/W)          | CR[n]      | 8                                                                                                                                                                                                                                                      | RCU_CRES[3] DEBUG reset control bit asserted                                                                                                                                                                                                           |
| 3:0 (R/W)          | CR[n]      | 15                                                                                                                                                                                                                                                     | RCU_CRES[3:0] Asserted                                                                                                                                                                                                                                 |

## Core Reset Outputs Status Register

The RCU core reset status register ( RCU\_CRSTAT ) contains status bits, indicating which core reset signals have been asserted.

Figure 6-3: RCU\_CRSTAT Register Diagram

<!-- image -->

Table 6-8: RCU\_CRSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W1C)        | CR[n]      | Core Reset Outputs. The RCU_CRSTAT.CR[n] bits indicate which cores have been reset since the last time the bit was cleared. Bits masked by CORE_DISABLE_MASK[15:0] are perma- nently disabled and the corresponding CR bits set. CR bits are sticky, they need to be cleared by software. | Core Reset Outputs. The RCU_CRSTAT.CR[n] bits indicate which cores have been reset since the last time the bit was cleared. Bits masked by CORE_DISABLE_MASK[15:0] are perma- nently disabled and the corresponding CR bits set. CR bits are sticky, they need to be cleared by software. |
|                    |            | 0                                                                                                                                                                                                                                                                                         | RCU_CRES[1:0] deasserted. CR[n] corresponds to RCU_CRES[n].                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                         | RCU_CRES[0] ARM A5 reset control bit asserted                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                         | RCU_CRES[1] SHARC0 reset control bit asserted                                                                                                                                                                                                                                             |
|                    |            | 4                                                                                                                                                                                                                                                                                         | RCU_CRES[2] SHARC1 reset control bit asserted                                                                                                                                                                                                                                             |
|                    |            | 8                                                                                                                                                                                                                                                                                         | RCU_CRES[3] DEBUG reset control bit asserted                                                                                                                                                                                                                                              |
|                    |            | 15                                                                                                                                                                                                                                                                                        | RCU_CRES[3:0] were asserted since the last time bits were cleared. CR[n] corresponds to RCU_CRES[n].                                                                                                                                                                                      |

## Control Register

The RCU control register ( RCU\_CTL ) provides a register lock, enables for the core and system reset requests inputs and control for the Reset Output pin.

Figure 6-4: RCU\_CTL Register Diagram

<!-- image -->

Table 6-9: RCU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CTL.LOCK bit is set, the RCU_CTL register is read only (locked). This bit is cleared by a hard reset or any system reset event. 0 Unlock                        |
| 10 (R/W)           | CRSTMSKSEL | Core Reset System Reset Mask Select. The RCU_CTL.CRSTMSKSEL bit selects the core reset system reset mask. This bit is cleared by a hard reset.                                                                                         |
| 9 (R/W)            | CRSTREQEN  | Core Reset Request Enabled. The RCU_CTL.CRSTREQEN bit controls whether the SYSCLK domain source(s) of reset is/are enabled to reset the core(s) when asserted. This bit is cleared by hard reset or any system reset event. 0 Disabled |
| 9 (R/W)            | CRSTREQEN  | 1 Enabled                                                                                                                                                                                                                              |
| 9 (R/W)            | CRSTREQEN  |                                                                                                                                                                                                                                        |

Table 6-9: RCU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | SRSTREQEN  | System Reset Request Enabled. The RCU_CTL.SRSTREQEN bit controls whether the SYSCLK domain sources of reset are enabled to do a system reset when asserted. This bit is cleared by a hard reset. |
| 2 (R0/W)           | RSTOUTDSRT | Reset Out Deassert. The RCU_CTL.RSTOUTDSRT bit controls the deassertion of the system reset pin. 0 No Action                                                                                     |
| 1 (R0/W)           | RSTOUTASRT | 1 Deassert RSTOUT Reset Out Assert. The RCU_CTL.RSTOUTASRT bit controls assertion of the system reset pin. 0 No Action                                                                           |
| 0 (R0/W)           | SYSRST     | System Reset. The RCU_CTL.SYSRST bit provides reset for all system units. 0 No Action                                                                                                            |

## Message Register

The RCU\_MSG is a general-purpose register. It is intended to provide flexibility for Boot ROM code and to pass predefined variables to the debugger. Please see the Booting chapter for product-specific details.

Figure 6-5: RCU\_MSG Register Diagram

<!-- image -->

Table 6-10: RCU\_MSG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | CALLBACK   | Callback Call Flag. The RCU_MSG.CALLBACK bit indicates that a flag has been set by the boot code prior to a callback call. |
| 30 (R/W)           | CALLBACK   | 0 Flag not set                                                                                                             |
| 30 (R/W)           | CALLBACK   | 1 Flag set                                                                                                                 |

Table 6-10: RCU\_MSG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | CALLINIT   | Call Initcode Flag. The RCU_MSG.CALLINIT bit indicates that a flag has been set by the boot code prior to an initcode call.      |
| 29 (R/W)           | CALLINIT   | 0 Flag not set                                                                                                                   |
| 29 (R/W)           | CALLINIT   | 1 Flag set                                                                                                                       |
| 28 (R/W)           | CALLAPP    | Call Application Flag. The RCU_MSG.CALLAPP bit indicates that a flag has been set by the boot code prior to an application call. |
| 28 (R/W)           | CALLAPP    | 0 Flag not set                                                                                                                   |
| 28 (R/W)           | CALLAPP    | 1 Flag set                                                                                                                       |
| 26 (R/W)           | HALTONCALL | Halt on Callback Call.                                                                                                           |
| 26 (R/W)           | HALTONCALL | 0 Do not generate exception                                                                                                      |
| 26 (R/W)           | HALTONCALL | 1 Generate exception                                                                                                             |
| 25 (R/W)           | HALTONINIT | Halt on Initcode Call. The RCU_MSG.HALTONINIT bit generates an emulation exception prior to an init-                             |
| 25 (R/W)           | HALTONINIT | 0 Do not generate exception                                                                                                      |
| 25 (R/W)           | HALTONINIT | 1 Generate exception                                                                                                             |
| 24 (R/W)           | HALTONAPP  | Halt on Application Call. The RCU_MSG.HALTONAPP bit generates an emulation exception prior to an appli- cation call.             |
| 24 (R/W)           | HALTONAPP  | 0 Do not generate exception                                                                                                      |
| 24 (R/W)           | HALTONAPP  | 1 Generate exception                                                                                                             |
| 23 (R/W)           | L3INIT     | L3 Initialized. The RCU_MSG.L3INIT bit indicates that the L3 resource is initialized. not initialized                            |
| 23 (R/W)           | L3INIT     | 0 Resource                                                                                                                       |
| 23 (R/W)           | L3INIT     | 1 Resource initialized                                                                                                           |
| 22 (R/W)           | L2INIT     | L2 Initialized. The RCU_MSG.L2INIT bit indicates that the L2 resource is initialized.                                            |
| 22 (R/W)           | L2INIT     | 0 Resource not initialized                                                                                                       |
| 22 (R/W)           | L2INIT     | 1 Resource initialized                                                                                                           |

Table 6-10: RCU\_MSG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | SECINIT    | SEC Initialized. The RCU_MSG.SECINIT bit is used by tools for initialization of the SEC.                                                                |
| 20 (R/W)           | C2ACTIVATE | Core 2 Activated. The RCU_MSG.C2ACTIVATE bit is used by tools for activation of Core 2.                                                                 |
| 19 (R/W)           | C1ACTIVATE | Core 1 Activated. The RCU_MSG.C1ACTIVATE bit is used by tools for activation of Core 1.                                                                 |
| 18 (R/W)           | C2L1INIT   | Core 2 L1 Initialized. The RCU_MSG.C2L1INIT bit indicates that the core 2 L1 resource is initialized.                                                   |
| 17 (R/W)           | C1L1INIT   | Core 1 L1 Initialized. The RCU_MSG.C1L1INIT bit indicates that the core 1 L1 resource is initialized. 0 Resource not initialized 1 Resource initialized |
| 16 (R/W)           | C0L1INIT   | Core 0 L1 Initialized. The RCU_MSG.C0L1INIT bit indicates that the core 0 L1 resource is initialized. 0 Resource not initialized                        |
| 10 (R/W)           | C2IDLE     | Core 2 Idle. The RCU_MSG.C2IDLE bit indicates that core 2 is in a safe idle state in ROM.                                                               |
| 9 (R/W)            | C1IDLE     | Core 1 Idle. The RCU_MSG.C1IDLE bit indicates that core 1 is in a safe idle state in ROM.                                                               |
| 8 (R/W)            | C0IDLE     | Core 0 Idle. The RCU_MSG.C0IDLE bit indicates that core 0 is in a safe idle state in ROM.                                                               |
| 7:0 (R/W)          | ERRCODE    | ROMError Code. The RCU_MSG.ERRCODE bit indicates the error code of the ROM. It is valid only when in the error handler.                                 |

## Message Clear Bits Register

The RCU\_MSG\_CLR register is used to clear bits in RCU\_MSG register. Reading this register returns 0x00000000.

Figure 6-6: RCU\_MSG\_CLR Register Diagram

<!-- image -->

Table 6-11: RCU\_MSG\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                   |
|--------------------|------------|-------------------------------------------|
| 31:0               | CLR        | Clear MSG Register Bits.                  |
| (R0/W1C)           |            | The RCU_MSG_CLR.CLR bit resets MSG bit n. |

## Message Set Bits Register

The RCU\_MSG\_SET register is used to set bits in RCU\_MSG register. Reading this register returns 0x00000000.

Figure 6-7: RCU\_MSG\_SET Register Diagram

<!-- image -->

Table 6-12: RCU\_MSG\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 31:0               | SET        | Set Message Bits.                       |
| (R0/W1S)           |            | The RCU_MSG_SET.SET bit sets MSG bit n. |

## System Reset Request Status Register

The RCU system reset request status register ( RCU\_SRRQSTAT ) contains status bits, indicating which system reset request input triggered a system reset. This register is set to its default values by a hard reset.

Figure 6-8: RCU\_SRRQSTAT Register Diagram

<!-- image -->

Table 6-13: RCU\_SRRQSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W1C)        | SRRQ       | System Reset Triggered by System Reset Request [n]. The RCU_SRRQSTAT.SRRQ bits are set by the assertion of the corresponding sys- tem reset request input and deasserted by writing "1" to the bit. The RCU_SRRQSTAT register is cleared by a hard reset. | System Reset Triggered by System Reset Request [n]. The RCU_SRRQSTAT.SRRQ bits are set by the assertion of the corresponding sys- tem reset request input and deasserted by writing "1" to the bit. The RCU_SRRQSTAT register is cleared by a hard reset. |
|                    |            | 1                                                                                                                                                                                                                                                         | RCU_SRRQ[0], System Reset Request from SEC as- serted                                                                                                                                                                                                     |
|                    |            | 2                                                                                                                                                                                                                                                         | RCU_SRRQ[1], System Reset Request from TRGS_RCU0_SYSRST0 asserted                                                                                                                                                                                         |
|                    |            | 4                                                                                                                                                                                                                                                         | RCU_SRRQ[2], System Reset Request from TRGS_RCU0_SYSRST1 asserted                                                                                                                                                                                         |
|                    |            | 8                                                                                                                                                                                                                                                         | RCU_SRRQ[3], System Reset Request from CTI3 as- serted                                                                                                                                                                                                    |

## Status Register

The RCU status register ( RCU\_STAT ) contains status bits for all RCU reset sources, reset status, and boot mode inputs. Status bits for reset sources are sticky and can cleared by software. Error status bits are cleared by any reset event.

Figure 6-9: RCU\_STAT Register Diagram

<!-- image -->

Table 6-14: RCU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W1C)         | RSTOUTERR  | Reset Out Error. The RCU_STAT.RSTOUTERR bit indicates (if set) that a write attempted to set the RCU_CTL.RSTOUTASRT and RCU_CTL.RSTOUTDSRT simultaneously. This condition triggers a bus error. 0 No Error                                                         |
| 17 (R/W1C)         | LWERR      | Lock Write Error. The RCU_STAT.LWERR bit indicates (when set) there was an attempted write to an RCU register while the RCU_CTL.LOCK bit was set and the global lock bit is enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear 0 No Error |
|                    |            | 1 Error Occurred                                                                                                                                                                                                                                                   |

Table 6-14: RCU\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W1C)         | ADDRERR    | Address Error. The RCU_STAT.ADDRERR bit indicates that the RCU generated an address error. This status bit is sticky; write-1-to-clear it.                                                                                                                                        |
| 16 (R/W1C)         | ADDRERR    | 0 No Error                                                                                                                                                                                                                                                                        |
| 16 (R/W1C)         | ADDRERR    | 1 Error Occurred                                                                                                                                                                                                                                                                  |
| 11:8 (R/NW)        | BMODE      | Boot Mode. The RCU_STAT.BMODE bits indicate the input on the boot mode pins.                                                                                                                                                                                                      |
| 5 (R/NW)           | RSTOUT     | Reset Out Status. The RCU_STAT.RSTOUT bit indicates the assertion status of the system reset pin.                                                                                                                                                                                 |
| 5 (R/NW)           | RSTOUT     | 0 RSTOUT Deasserted                                                                                                                                                                                                                                                               |
| 5 (R/NW)           | RSTOUT     | 1 RSTOUT Asserted                                                                                                                                                                                                                                                                 |
| 3 (R/W1C)          | SWRST      | Software Reset. The RCU_STAT.SWRST bit indicates that a system reset (which was triggered by software) has occurred since the last time a hardware reset occurred or since the RCU_STAT.SWRST bit was cleared by software.                                                        |
| 3 (R/W1C)          | SWRST      | 0 Inactive                                                                                                                                                                                                                                                                        |
| 3 (R/W1C)          | SWRST      | 1 Reset Occurred                                                                                                                                                                                                                                                                  |
| 2 (R/W1C)          | SSRST      | System Source Reset. The RCU_STAT.SSRST bit indicates that a system reset triggered by hardware in the system clock domain, clock A domain, or clock B domain has occurred since the last time a hardware reset occurred or since the RCU_STAT.SSRST bit was cleared by software. |
| 2 (R/W1C)          | SSRST      | 0 Inactive                                                                                                                                                                                                                                                                        |
| 2 (R/W1C)          | SSRST      | 1 Reset Occurred                                                                                                                                                                                                                                                                  |
| 0 (R/W1C)          | HWRST      | Hardware Reset. The RCU_STAT.HWRST bit indicates that a hardware reset has occurred.                                                                                                                                                                                              |
| 0 (R/W1C)          | HWRST      | 0 Inactive                                                                                                                                                                                                                                                                        |
| 0 (R/W1C)          | HWRST      | 1 Reset Occurred                                                                                                                                                                                                                                                                  |

## Software Vector Register 0

The RCU\_SVECT0 register contains the default location of the first instruction to execute after a reset.

Figure 6-10: RCU\_SVECT0 Register Diagram

<!-- image -->

Table 6-15: RCU\_SVECT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Core 0 Reset Vector.                                                      |
| (R/W)              |            | Core0 jumps to the address in the RCU_SVECT0.VALUE bit field after reset. |

## Software Vector Register 1

The RCU\_SVECT1 register contains the default location of the first instruction to execute after a reset.

Figure 6-11: RCU\_SVECT1 Register Diagram

<!-- image -->

Table 6-16: RCU\_SVECT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Reset Vector.                                                             |
| (R/W)              |            | Core1 jumps to the address in the RCU_SVECT1.VALUE bit field after reset. |

## Software Vector Register 2

The RCU\_SVECT2 register contains the default location of the first instruction to execute after a reset.

Figure 6-12: RCU\_SVECT2 Register Diagram

<!-- image -->

Table 6-17: RCU\_SVECT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Reset Vector.                                                             |
| (R/W)              |            | Core2 jumps to the address in the RCU_SVECT2.VALUE bit field after reset. |

## SVECT Lock Register

The RCU software vector lock register ( RCU\_SVECT\_LCK ) provides a register lock and software vector n enable bits for each processor core on the product. This register is set to its default values by a hard reset or any system reset event.

Figure 6-13: RCU\_SVECT\_LCK Register Diagram

<!-- image -->

Table 6-18: RCU\_SVECT\_LCK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SVECT_LCK.LOCK bit is set, the RCU_SVECT_LCK register is read only (locked). 0 Unlock       |
| 2:0 (R/W)          | SVECT[n]   | Lock SVECTn Registers. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SVECT_LCK.SVECT[n] bit is set, the SVECT registers are read only (locked). |