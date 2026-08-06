## 6   Reset Control Unit (RCU)

Reset is the initial state of the whole processor at power-on or the run-time state of any core, as controlled by another core in the device via the RCU or as a a result of a hardware or software triggered event. In this state, all control registers are set to their default values and functional units are idle. Exiting a full system hardware reset results with only the host core being released from reset and ready to boot (Core 0 on ADSP-SC58x devices, Core 1 on ADSP-215xx devices). The following table shows the cores that are released from reset.

Additional information on reset and booting can be found in Boot ROM and Booting the Processor.

| Product    | Core Released from Reset by RCU   |
|------------|-----------------------------------|
| ADSP-SC58x | Core ID = 0                       |
| ADSP-2158x | Core ID = 1                       |

The Reset Control Unit (RCU) controls how all the functional units enter and exit reset. Differences in functional requirements and clocking constraints define how reset signals are generated. Programs must guarantee that none of the reset functions puts the system into an undefined state or causes resources to stall. This functionality is important when only one of the cores is reset (programs must ensure that there is no pending system activity involving the core that is being reset). While core resets and software system resets are controlled directly in the RCU, hardware resets can come from the TRU, the SEC, or the CGU Oscillator Watchdog.

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

## ADSP-SC58x RCU Register List

The Reset Control Unit (RCU) controls how all the functional units in the processor enter and exit Reset. Differences in functional requirements and clocking constraints exist (units in different clock domains have to enter reset asynchronously, but units exit reset in a deterministic way), and these differences define how reset signals are generated. Reset signals propagate through all functional units asynchronously. For more information on RCU functionality, see the RCU register descriptions.

Table 6-1: ADSP-SC58x RCU Register List

| Name        | Description                         |
|-------------|-------------------------------------|
| RCU_BCODE   | Boot Code Register                  |
| RCU_CRCTL   | Core Reset Outputs Control Register |
| RCU_CRSTAT  | Core Reset Outputs Status Register  |
| RCU_CTL     | Control Register                    |
| RCU_MSG     | Message Register                    |
| RCU_MSG_CLR | Message Clear Bits Register         |
| RCU_MSG_SET | Message Set Bits Register           |
| RCU_SIDIS   | System Interface Disable Register   |
| RCU_SISTAT  | System Interface Status Register    |

Table 6-1: ADSP-SC58x RCU Register List (Continued)

| Name          | Description                |
|---------------|----------------------------|
| RCU_STAT      | Status Register            |
| RCU_SVECT0    | Software Vector Register 0 |
| RCU_SVECT1    | Software Vector Register 1 |
| RCU_SVECT2    | Software Vector Register 2 |
| RCU_SVECT_LCK | SVECT Lock Register        |

## ADSP-SC58x RCU Trigger List

Table 6-2: ADSP-SC58x RCU Trigger List Masters

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

Table 6-3: ADSP-SC58x RCU Trigger List Slaves

|   Trigger ID | Name         | Description         | Sensitivity   |
|--------------|--------------|---------------------|---------------|
|           50 | RCU0_SYSRST0 | RCU0 System Reset 0 | Pulse         |
|           51 | RCU0_SYSRST1 | RCU0 System Reset 1 | Pulse         |

## RCU Definitions

To make the best use of the RCU, it is useful to understand the terms in this section.

The target or source defines the following are types of resets.

## Hardware Reset (by target)

All functional units except a small subsection of debug interfaces are set to their default states. State is lost in all nonvolatile storage.

## System Reset (by target)

All functional units except the RCU, flash interface, and debug are set to their default states.

## Core n Only Reset (by target)

Affects Core n only. The system software must guarantee that a bus master cannot access the core in reset state.

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

| Reset Source                                                       | Reset Type      | Affected Functional Units                                                                                             |
|--------------------------------------------------------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------|
| SYS_HWRST pin assertion                                            | Hardware Reset  | All functional units, except RTC (if present)                                                                         |
| SYSCLK clock domain system reset by fault unit (FMU) or TRU master | System Reset    | All functional units, except: • RTC (if present), • RCU_STAT, • RCU_BCODE, and • the units on the VDDEXT power domain |
| RCU_CTL.SYSRST bit set (software triggered reset)                  | System Reset    | All functional units, except: • RTC (if present), • RCU_STAT, • RCU_BCODE, and • the units on the VDDEXT power domain |
| RCU_CRCTL.CR[n] bit set (software triggered reset)                 | Core Only Reset | Core n only, for (2 ≥ n ≥ 0)                                                                                          |

## RCU Status and Error Signals

The RCU\_STAT register reflects status and error information. There are three kinds of errors that can occur in the RCU. The reset out error is triggered when RSTOUT is both asserted and deasserted at the same time. The lock

write error occurs if an attempt is made to write a lock RCU register. The address error occurs if a read-only register is written to or if an attempt is made to a reserved address within the RCU MMR address range.

## Resetting the ARM Core through Another Core or System Master

The RCU allows reset of a given core n using another core or system master. Core 0 can be individually reset by software, either setting any of CR0 bit in the RCU\_CRCTL register. Cores that reset themselves cannot guarantee that all the system transactions to or from it have completed. Although a core n reset can be triggered by core n itself, it is recommended that another core or system master trigger it. Core n can be reset to restore its functionality when it cannot execute software.

The following steps show the suggested programming sequence to reset core n only.

1. Clear the RCU\_CRSTAT.CR[n] bit.
2. Disable interrupts to core n
3. Set the RCU\_SIDIS.SI[n] bit to disable the interfaces for core n, to stop DMA accesses to its L1, to stop accesses to memory for core n, and stop accesses to MMRs.
4. Test the RCU\_SISTAT.SI[n] bit to detect when accesses to core n have been disabled and all the pending transactions have completed.
5. Set the RCU\_CRCTL.CR[n] bit to reset core n.
6. Poll the RCU\_CRSTAT.CR[n] bit until core n is in reset.
7. Once the core is in reset, clear the RCU\_SIDIS.SI[n] bit to reenable the core interfaces.
8. Clear the RCU\_CRCTL.CR[n] bit to take core n out of reset.
9. Poll the RCU\_CRSTAT.CR[n] bit until core n is out of reset.

## Resetting a SHARC+ Core Through Another Core

Resetting a SHARC+ core involves a software handshake between the Master core which issues a reset to the SHARC+ core. The handshake is done using a core interrupt along with message passing through a variable in shared L2 memory RAM. Each SHARC+ core needs two bits for handshaking.

- Core Reset Request bit (CRR). This bit is set by the master core to indicate to the SHARC+ core that a core reset needs to be done. If the CRR bit is set the SHARC+ core should disable all interrupts, stop all system and memory accesses and enter the IDLE state.
- IDLE acknowledgement (IDLE). Once the SHARC+ core is ready for reset, it sets this bit before entering the IDLE state to inform the master core that it is ready for reset.

Two bits are needed for SHARC0 core and two bits are needed for SHARC1 core. These bits are:

- CCR0 - SHARC0 core reset request bit in shared variable
- IDLE0 - SHARC0 IDLE acknowledgement bit in shared variable
- CCR1 - SHARC1 core reset request bit in shared variable
- IDLE1 - SHARC1 IDLE acknowledgement bit in shared variable

Use the following programming to reset the SHARC+ core.

1. The master core checks the IDLE status bits in shared variable for the corresponding SHARC+ core ( RCU\_MSG.C1IDLE and RCU\_MSG.C0IDLE ).
2. If one of the core idle bits is set then the program jumps to step 10. Otherwise the process continues to step 3.
3. The master core sets the CCRx bit in message register and raises the SOFT0 software interrupt through the SEC to the SHARC+ core (see Programming Examples for more information).
4. The SHARC+ core goes to into the ISR, and checks the 'CCRx' status bit in RCU\_MSG register to ensure that software interrupt is for core reset.
5. ADDITIONAL INFORMATION: The SOFT0 software interrupt handler can be used for other non-reset/ general purposes as well.
6. ADDITIONAL INFORMATION: The ISR should be in L1 memory.
5. The core should disable all interrupts before entering IDLE for reset.
6. The core sets the appropriate idle status bit ( RCU\_MSG.C1IDLE and RCU\_MSG.C0IDLE ).
7. Core enters IDLE.
8. Master core polls for the IDLEx status bit
9. The master core keeps a timeout option where if within N number of cycles the SHARC core doesn't respond then a system reset is initiated.
12. ADDITIONAL INFORMATION: N can be decided by the user depending on timing criticality of the application.
10. Once the IDLEx status bit is found set, the master core initiates a core reset though the RCU.
11. Clear the RCU\_CRSTAT.CR[n] bit.
12. Set the RCU\_SIDIS.SI[n] bit to disable the interfaces for core n, to stop DMA accesses to its L1, to stop accesses to memory for core n, and stop accesses to MMRs.
13. Test the RCU\_SISTAT.SI[n] bit to detect when accesses to core n have been disabled and all the pending transactions have completed.
14. Set the RCU\_CRCTL.CR[n] bit to reset core n.

15. Poll the RCU\_CRSTAT.CR[n] bit until core n is in reset.
16. Once the core is in reset, clear the RCU\_SIDIS.SI[n] bit to re-enable the core interfaces.
17. Clear the RCU\_CRCTL.CR[n] bit to take core n out of reset.
18. Poll the RCU\_CRSTAT.CR[n] bit until core n is out of reset.

If a core is servicing a higher priority interrupt and gets stuck then it may not respond to the SEC.

## ADSP-SC58x RCU Register Descriptions

Reset Control Unit (RCU) contains the following registers.

Table 6-5: ADSP-SC58x RCU Register List

| Name          | Description                         |
|---------------|-------------------------------------|
| RCU_BCODE     | Boot Code Register                  |
| RCU_CRCTL     | Core Reset Outputs Control Register |
| RCU_CRSTAT    | Core Reset Outputs Status Register  |
| RCU_CTL       | Control Register                    |
| RCU_MSG       | Message Register                    |
| RCU_MSG_CLR   | Message Clear Bits Register         |
| RCU_MSG_SET   | Message Set Bits Register           |
| RCU_SIDIS     | System Interface Disable Register   |
| RCU_SISTAT    | System Interface Status Register    |
| RCU_STAT      | Status Register                     |
| RCU_SVECT0    | Software Vector Register 0          |
| RCU_SVECT1    | Software Vector Register 1          |
| RCU_SVECT2    | Software Vector Register 2          |
| RCU_SVECT_LCK | SVECT Lock Register                 |

## Boot Code Register

The RCU\_BCODE register can be used to determine if and how core boots. This register is set to its default values by RESET.

Figure 6-1: RCU\_BCODE Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000000_152f91d32792d9c4b9803eaccd4cf5ed370b5c2ec54b449f9c162276fd68177e.png)

Table 6-6: RCU\_BCODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_BCODE.LOCK bit is set, the RCU_BCODE register is read only (locked). 0 Unlock 1 Lock |
| 18 (R/W)           | NOCORE2    | No Core 2 Present. The RCU_BCODE.NOCORE2 bit indicates the presence of core 2. 0 Core does not exist 1 Core exists                                          |

Table 6-6: RCU\_BCODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                          |                                                                                                                  |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | NOCORE1     | No Core 1 Present. The RCU_BCODE.NOCORE1 bit indicates the presence of core 1.                                   | No Core 1 Present. The RCU_BCODE.NOCORE1 bit indicates the presence of core 1.                                   |
| 17 (R/W)           | NOCORE1     | 0                                                                                                                | Core does not exist                                                                                              |
| 17 (R/W)           | NOCORE1     | 1                                                                                                                | Core exists                                                                                                      |
| 16 (R/W)           | NOCORE0     | No Core 0 Present. The RCU_BCODE.NOCORE0 bit indicates the presence of core 0.                                   | No Core 0 Present. The RCU_BCODE.NOCORE0 bit indicates the presence of core 0.                                   |
| 16 (R/W)           | NOCORE0     | 0                                                                                                                | Core does not exist                                                                                              |
| 16 (R/W)           | NOCORE0     | 1                                                                                                                | Core exists                                                                                                      |
| 13 (R/W)           | IDLEONENTRY | Idle On Entry. The RCU_BCODE.IDLEONENTRY bit configures the RCU to enter the idle state at startup.              | Idle On Entry. The RCU_BCODE.IDLEONENTRY bit configures the RCU to enter the idle state at startup.              |
| 13 (R/W)           | IDLEONENTRY | 0                                                                                                                | Do not enter idle state                                                                                          |
| 13 (R/W)           | IDLEONENTRY | 1                                                                                                                | Enter idle state                                                                                                 |
| 12 (R/W)           | NOL2CONFIG  | No L2 Configuration. The RCU_BCODE.NOL2CONFIG bit configures the RCU to not perform the L2 memory configuration. | No L2 Configuration. The RCU_BCODE.NOL2CONFIG bit configures the RCU to not perform the L2 memory configuration. |
| 12 (R/W)           | NOL2CONFIG  | 0                                                                                                                | Configure L2 memory                                                                                              |
| 12 (R/W)           | NOL2CONFIG  | 1                                                                                                                | Do not configure L2 memory                                                                                       |
| 10 (R/W)           | NOHOOK      | No Hook. The RCU_BCODE.NOHOOK bit configures the RCU to not perform the hook rou- tine.                          | No Hook. The RCU_BCODE.NOHOOK bit configures the RCU to not perform the hook rou- tine.                          |
| 10 (R/W)           | NOHOOK      | 0                                                                                                                | Perform hook routine                                                                                             |
| 10 (R/W)           | NOHOOK      | 1                                                                                                                | Do not perform hook routine                                                                                      |
| 9 (R/W)            | NOPREBOOT   | No Preboot. The RCU_BCODE.NOPREBOOT bit configures the RCU to not perform the custom- er preboot routine.        | No Preboot. The RCU_BCODE.NOPREBOOT bit configures the RCU to not perform the custom- er preboot routine.        |
| 9 (R/W)            | NOPREBOOT   | 0                                                                                                                | Perform preboot                                                                                                  |
| 9 (R/W)            | NOPREBOOT   | 1                                                                                                                | Do not perform preboot                                                                                           |
| 6 (R/W)            | NOFAULTS    | No Faults. The RCU_BCODE.NOFAULTS bit configures the RCU to not perform fault initiali- zation.                  | No Faults. The RCU_BCODE.NOFAULTS bit configures the RCU to not perform fault initiali- zation.                  |
| 6 (R/W)            | NOFAULTS    | 0                                                                                                                | Perform fault initialization                                                                                     |
| 6 (R/W)            | NOFAULTS    | 1                                                                                                                | Do not perform fault initialization                                                                              |

Table 6-6: RCU\_BCODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | NOCACHE    | No Cache. The RCU_BCODE.NOCACHE bit configures the RCU to not perform a cache initiali- zation and to not enable the cache. |
| 5 (R/W)            | NOCACHE    | 0 Enable and initialize cache                                                                                               |
| 5 (R/W)            | NOCACHE    | 1 Do not initialize or enable cache                                                                                         |
| 4 (R/W)            | NOMEMINIT  | No Memory Initialization. The RCU_BCODE.NOMEMINIT bit configures the RCU to not perform a memory initialization.            |
| 4 (R/W)            | NOMEMINIT  | 0 Perform memory initialization                                                                                             |
| 4 (R/W)            | NOMEMINIT  | 1 Do not perform memory initialization                                                                                      |
| 3 (R/W)            | HBTOVW     | Execute Wakeup. The RCU_BCODE.HBTOVW bit configures the RCU to execute a wakeup.                                            |
| 3 (R/W)            | HBTOVW     | 0 Do not wakeup                                                                                                             |
| 2 (R/W)            | HALT       | Halt. The RCU_BCODE.HALT bit configures the RCU to execute the no boot routine.                                             |
| 2 (R/W)            | HALT       | 0 Do not execute routine                                                                                                    |
| 1 (R/W)            | NOVECTINIT | No Vector Initialize. The RCU_BCODE.NOVECTINIT bit configures the RCU to not vector to the appli- cation.                   |
| 1 (R/W)            | NOVECTINIT | 0 Vector                                                                                                                    |
| 1 (R/W)            | NOVECTINIT | 1 Do not vector                                                                                                             |
| 0 (R/W)            | NOKERNEL   | No Boot Kernel. The RCU_BCODE.NOKERNEL bit configures the RCU to not execute the boot ker- nel.                             |
| 0 (R/W)            | NOKERNEL   | 0 Execute boot kernel                                                                                                       |
| 0 (R/W)            | NOKERNEL   | 1 Do not execute boot kernel                                                                                                |

## Core Reset Outputs Control Register

The RCU core reset control n registers ( RCU\_CRCTL ) include a lock bit ( RCU\_CRCTL.LOCK ) and a core reset bit ( RCU\_CRCTL.CR[n] ) for each core reset signal on the product.

Figure 6-2: RCU\_CRCTL Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000001_94e1c2abcfcb7c12b656bd16279e2e61a77916f5a59d4bd8f869bb603b007368.png)

Table 6-7: RCU\_CRCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CRCTL.LOCK bit is set, the RCU_CRCTL register is read only (locked). 0 Unlock                                                                                                                                                                                      |
| 2:0 (R/W)          | CR[n]      | Core Reset Outputs. The RCU_CRCTL.CR[n] bits control CRES[1:0] core reset signals. The RCU_CRES[n] signals can be individually controlled. They are reset to their default value by a hard reset or a system reset. For each RCU_CRES[n], the selected RCU0_CRMSKi[n] bit is cleared. 0 RCU_CRES[2:0] Deasserted 7 RCU_CRES[2:0] Asserted |

## Core Reset Outputs Status Register

The RCU core reset status register ( RCU\_CRSTAT ) contains status bits, indicating which core reset signals have been asserted.

Figure 6-3: RCU\_CRSTAT Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000002_77df3392860b3b65686c4b61025e74c368edf84ecbd37ae0c213addded7eed3f.png)

Table 6-8: RCU\_CRSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W1C)        | CR[n]      | Core Reset Outputs. The RCU_CRSTAT.CR[n] bits indicate which cores have been reset since the last time the bit was cleared. Bits masked by CORE_DISABLE_MASK[15:0] are perma- nently disabled and the corresponding CR bits set. CR bits are sticky, they need to be cleared by software. | Core Reset Outputs. The RCU_CRSTAT.CR[n] bits indicate which cores have been reset since the last time the bit was cleared. Bits masked by CORE_DISABLE_MASK[15:0] are perma- nently disabled and the corresponding CR bits set. CR bits are sticky, they need to be cleared by software. |
|                    |            | 0                                                                                                                                                                                                                                                                                         | RCU_CRES[1:0] deasserted. CR[n] corresponds to RCU_CRES[n].                                                                                                                                                                                                                               |
|                    |            | 7                                                                                                                                                                                                                                                                                         | RCU_CRES[2:0] were asserted since the last time bits were cleared. CR[n] corresponds to RCU_CRES[n].                                                                                                                                                                                      |

## Control Register

The RCU control register ( RCU\_CTL ) provides a register lock, enables for the core and system reset requests inputs and control for the Reset Output pin.

Figure 6-4: RCU\_CTL Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000003_1e98e864e768529fcf58b6b4cb193202ab60e39132cedcebb7ce34db46b1fdc3.png)

Table 6-9: RCU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CTL.LOCK bit is set, the RCU_CTL register is read only (locked). This bit is cleared by a hard reset or any system reset event.                      |
| 9 (R/W)            | CRSTREQEN  | Core Reset Request Enabled. The RCU_CTL.CRSTREQEN bit controls whether the SYSCLK domain source(s) of reset is/are enabled to reset the core(s) when asserted. This bit is cleared by hard reset or any system reset event. |
| 8 (R/W)            | SRSTREQEN  | System Reset Request Enabled. The RCU_CTL.SRSTREQEN bit controls whether the SYSCLK domain sources of reset are enabled to do a system reset when asserted. This bit is cleared by a hard reset.                            |
| 8 (R/W)            |            | 0 Disabled                                                                                                                                                                                                                  |

Table 6-9: RCU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 2 (R0/W)           | RSTOUTDSRT | Reset Out Deassert. The RCU_CTL.RSTOUTDSRT bit controls the deassertion of the system reset pin. 0 No Action         |
| 1 (R0/W)           | RSTOUTASRT | Reset Out Assert. The RCU_CTL.RSTOUTASRT bit controls assertion of the system reset pin. 0 No Action 1 Assert RSTOUT |
| 0 (R0/W)           | SYSRST     | System Reset. The RCU_CTL.SYSRST bit provides reset for all system units. 0 No Action 1 System Reset                 |

## Message Register

The RCU\_MSG is a general-purpose register. It is intended to provide flexibility for Boot ROM code and to pass predefined variables to the debugger. Please see the Booting chapter for product-specific details.

Figure 6-5: RCU\_MSG Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000004_c4acdce6bb6f0e5304b5e67d3e60e794900d91f112de7681dde0500cf47eda69.png)

Table 6-10: RCU\_MSG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CALLERR    | Call Error Flag. The RCU_MSG.CALLERR bit indicates that a flag has been set by the boot code prior to an error call. |
| 31 (R/W)           | CALLERR    | 0 Flag not set                                                                                                       |
| 31 (R/W)           | CALLERR    | 1 Flag set                                                                                                           |

Table 6-10: RCU\_MSG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | CALLBACK   | Callback Call Flag. The RCU_MSG.CALLBACK bit indicates that a flag has been set by the boot code prior to a callback call.       |
| 30 (R/W)           | CALLBACK   | 0 Flag not set                                                                                                                   |
| 30 (R/W)           | CALLBACK   | 1 Flag set                                                                                                                       |
| 29 (R/W)           | CALLINIT   | Call Initcode Flag. The RCU_MSG.CALLINIT bit indicates that a flag has been set by the boot code prior to an initcode call.      |
| 29 (R/W)           | CALLINIT   | 0 Flag not set                                                                                                                   |
| 29 (R/W)           | CALLINIT   | 1 Flag set                                                                                                                       |
| 28 (R/W)           | CALLAPP    | Call Application Flag. The RCU_MSG.CALLAPP bit indicates that a flag has been set by the boot code prior to an application call. |
| 28 (R/W)           | CALLAPP    | 0 Flag not set                                                                                                                   |
| 28 (R/W)           | CALLAPP    | 1 Flag set                                                                                                                       |
| 27 (R/W)           | HALTONERR  | Halt on Error Call. The RCU_MSG.HALTONERR bit generates an emulation exception prior to an error call.                           |
| 27 (R/W)           | HALTONERR  | 0 Do not generate exception                                                                                                      |
| 27 (R/W)           | HALTONERR  | 1 Generate exception                                                                                                             |
| 26 (R/W)           | HALTONCALL | Halt on Callback Call. The RCU_MSG.HALTONCALL bit generates an emulation exception prior to a call- back call.                   |
| 26 (R/W)           | HALTONCALL | 0 Do not generate exception                                                                                                      |
| 26 (R/W)           | HALTONCALL | 1 Generate exception                                                                                                             |
| 25 (R/W)           | HALTONINIT | Halt on Initcode Call. The RCU_MSG.HALTONINIT bit generates an emulation exception prior to an init- code call.                  |
| 25 (R/W)           | HALTONINIT | 0 Do not generate exception                                                                                                      |
| 25 (R/W)           | HALTONINIT | 1 Generate exception                                                                                                             |

Table 6-10: RCU\_MSG Register Fields (Continued)

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                                        |
|--------------------|-------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | HALTONAPP         | Halt on Application Call. The RCU_MSG.HALTONAPP bit generates an emulation exception prior to an appli- cation call.                                                           |
| 22 (R/W)           | L2INIT            | L2 Initialized. The RCU_MSG.L2INIT bit indicates that the L2 resource is initialized. 0 Resource not initialized                                                               |
| 21 (R/W)           | SECINIT           | SEC Initialized. The RCU_MSG.SECINIT bit is used by tools for initialization of the SEC.                                                                                       |
| (R/W) 19 (R/W)     | C1ACTIVATE        | The RCU_MSG.C2ACTIVATE bit is used by tools for activation of Core 2. Core 1 Activated.                                                                                        |
| 18                 |                   | The RCU_MSG.C1ACTIVATE bit is used by tools for activation of Core 1. Core 2 L1 Initialized.                                                                                   |
| (R/W)              | C2L1INIT          | The RCU_MSG.C2L1INIT bit indicates that the core 2 L1 resource is initialized.                                                                                                 |
| 17 (R/W) 16        | C1L1INIT C0L1INIT | Core 1 L1 Initialized. The RCU_MSG.C1L1INIT bit indicates that the core 1 L1 resource is initialized. 0 Resource not initialized 1 Resource initialized Core 0 L1 Initialized. |
| 10                 | C2IDLE            | 1 Resource initialized Core 2 Idle. The RCU_MSG.C2IDLE bit indicates that core 2 is in a safe idle state in ROM.                                                               |
| (R/W)              |                   | Core 1 Idle.                                                                                                                                                                   |
| 9 (R/W)            | C1IDLE            | The RCU_MSG.C1IDLE bit indicates that core 1 is in a safe idle state in ROM.                                                                                                   |
| 8 (R/W)            | C0IDLE            | Core 0 Idle. The RCU_MSG.C0IDLE bit indicates that core 0 is in a safe idle state in ROM.                                                                                      |

Table 6-10: RCU\_MSG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | ERRCODE    | ROMError Code. The RCU_MSG.ERRCODE bit indicates the error code of the ROM. It is valid only when in the error handler. |

## Message Clear Bits Register

The RCU\_MSG\_CLR register is used to clear bits in RCU\_MSG register. Reading this register returns 0x00000000.

Figure 6-6: RCU\_MSG\_CLR Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000005_5edcc585b1f9cce803d9b85fd8a6999166b1dac3a8a8a5a3b845e399a484b00e.png)

Table 6-11: RCU\_MSG\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                   |
|--------------------|------------|-------------------------------------------|
| 31:0               | CLR        | Clear MSG Register Bits.                  |
| (R0/W1C)           |            | The RCU_MSG_CLR.CLR bit resets MSG bit n. |

## Message Set Bits Register

The RCU\_MSG\_SET register is used to set bits in RCU\_MSG register. Reading this register returns 0x00000000.

Figure 6-7: RCU\_MSG\_SET Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000006_a860408c2a1879d828735fa9945344b77c21ab1466069775d87fc69cca1a8568.png)

Table 6-12: RCU\_MSG\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 31:0               | SET        | Set Message Bits.                       |
| (R0/W1S)           |            | The RCU_MSG_SET.SET bit sets MSG bit n. |

## System Interface Disable Register

The RCU system interface disable register ( RCU\_SIDIS ) lets the RCU assert a system interface disable request to functional units in the processor. This register is set to its default values by a hard reset or any system reset event. For information on mapping between RCU\_SIDIS bits and functional units, see the RCU functional description.

Figure 6-8: RCU\_SIDIS Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000007_624b7fdb22c01b506a9f4136a148158646def4ed45d11b27ac00cca8fefd852f.png)

Table 6-13: RCU\_SIDIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SIDIS.LOCK bit is set, the RCU_SIDIS register is read only (locked). 0 Unlock                                                                                                                |
| 1:0 (R/W)          | SI[n]      | System Interface Disable Request [1:0]. Each RCU_SIDIS.SI[n] bit corresponds to a functional unit in the processor that supports the system interface disable request-acknowledge protocol. 0 RCU_SI_DISABLE_REQ[1:0] deasserted 3 RCU_SI_DISABLE_REQ[1:0] asserted |

## System Interface Status Register

The RCU system interface status register ( RCU\_SISTAT ) indicates whether a functional unit has or has not acknowledged an RCU unit disable request.

Figure 6-9: RCU\_SISTAT Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000008_89ffdcc7132ce8a9f27c21a6b52ca587056672bf3ad4274f5dc636f1334df8c3.png)

Table 6-14: RCU\_SISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1:0 (R/NW)         | SI[n]      | System Interface Disable Acknowledge [1:0]. The RCU_SISTAT.SI[n] bit indicates whether a functional unit has or has not ac- knowledged an RCU unit disable request. |
| 1:0 (R/NW)         | SI[n]      | 0 No Acknowledge                                                                                                                                                    |
| 1:0 (R/NW)         | SI[n]      | 3 SI_DISABLE_ACK[1:0] asserted                                                                                                                                      |

## Status Register

The RCU status register ( RCU\_STAT ) contains status bits for all RCU reset sources, reset status, and boot mode inputs. Status bits for reset sources are sticky and can cleared by software. Error status bits are cleared by any reset event.

Figure 6-10: RCU\_STAT Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000009_b4503191605a9c1a0fc062ffcc5deedd370987a05a5e6c2042d6b318494c9266.png)

Table 6-15: RCU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W1C)         | RSTOUTERR  | Reset Out Error. The RCU_STAT.RSTOUTERR bit indicates (if set) that a write attempted to set the RCU_CTL.RSTOUTASRT and RCU_CTL.RSTOUTDSRT simultaneously. This condition triggers a bus error. 0 No Error                                                         |
| 17 (R/W1C)         | LWERR      | Lock Write Error. The RCU_STAT.LWERR bit indicates (when set) there was an attempted write to an RCU register while the RCU_CTL.LOCK bit was set and the global lock bit is enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear 0 No Error |
|                    |            | 1 Error Occurred                                                                                                                                                                                                                                                   |

Table 6-15: RCU\_STAT Register Fields (Continued)

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

Figure 6-11: RCU\_SVECT0 Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000010_bde83224fa11ed855fe0e16c06dc0bd02e355118d865900e85a814d80b6f25ce.png)

Table 6-16: RCU\_SVECT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Core 0 Reset Vector. The RCU_SVECT0.VALUE bit field contains the default location of the first instruc- tion to execute after a reset. |
| (R/W)              |            |                                                                                                                                        |

## Software Vector Register 1

Figure 6-12: RCU\_SVECT1 Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000011_99c2326c10ef25f5693e63aa54c5856b15759830f0e42ae74c3714cab3fc5041.png)

Table 6-17: RCU\_SVECT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Reset Vector.             |
| (R/W)              |            |                           |

## Software Vector Register 2

Figure 6-13: RCU\_SVECT2 Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000012_d5c01ab7badaf6d850efd6f503e35d0a98dc01dfa97add9b3a7af8d40aa69967.png)

Table 6-18: RCU\_SVECT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Reset Vector.             |
| (R/W)              |            |                           |

## SVECT Lock Register

The RCU software vector lock register ( RCU\_SVECT\_LCK ) provides a register lock and software vector n enable bits for each processor core on the product. This register is set to its default values by a hard reset or any system reset event.

Figure 6-14: RCU\_SVECT\_LCK Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000013_89efd5dd738cafad4a2cc34f45bb9203f93395bbd791d8060ee0a2dbc15461c6.png)

Table 6-19: RCU\_SVECT\_LCK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SVECT_LCK.LOCK bit is set, the RCU_SVECT_LCK register is read only (locked). 0 Unlock                           |
| 2:0 (R/W)          | SVECT[n]   | Lock SVECTn Registers. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SVECT_LCK.SVECT[n] bit is set, the RCU_SVECT0 and RCU_SVECT1 registers are read only (locked). |