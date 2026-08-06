## 6   Reset Control Unit (RCU)

Reset is the initial state of the processor at power-on or the run-time state of any core, as controlled by another core in the device using the RCU or as a result of a hardware or software triggered event. In this state, all control registers are set to their default values and functional units are idle. Exiting a full system hardware reset results with only the host core being released from reset and ready to boot (core 1(SHARC-FX) on the ADSP-2184x/SC84x processors).

The following table shows the cores that are released from reset. Exiting core n only reset starts with this core n being ready to execute the code from the software vector register ( RCU\_SVECT2 ).

Additional information on reset and booting can be found in the Boot ROM and Booting the Processor chapter.

| Product    | Core Released from Reset by RCU   |
|------------|-----------------------------------|
| ADSP-2184x | Core 1                            |
| ADSP-SC84x | Core 0                            |

The Reset Control Unit (RCU) controls how all the functional units enter and exit reset. Differences in functional requirements and clocking constraints define how reset signals are generated. Programs must guarantee that none of the reset functions puts the system into an undefined state or causes resources to stall. This functionality is important when only one of the cores is reset (programs must ensure that there is no pending system activity involving the core that is being reset). While core resets and software system resets are controlled directly in the RCU, hardware resets can come from the TRU or SEC.

## RCU Features

The RCU module supports the following features:

- Hardware reset through the SYS\_HWRST pin
- Software system reset through the RCU control ( RCU\_CTL ) register
- Hardware system reset through:
- TRU module
- SEC module (System Fault Unit )
- Core reset through RCU Core Reset Output ( RCU\_CRCTL ) register

## RCU Functional Description

This section provides information on the function of RCU module.

## Software reset using RCU registers

Setting the RCU\_STAT.SWRST bit issues a software reset for all system units except the RTC module and RCU\_BCODE , RCU\_CRCTL and RCU\_STAT registers.

## Core reset RCU registers

A core can be individually reset by software, or by setting the RCU\_CRCTL.CR[n] bit.

## ADSP-2184x RCU Register List

The Reset Control Unit (RCU) controls how all the functional units in the processor enter and exit Reset. Differences in functional requirements and clocking constraints exist (units in different clock domains have to enter reset asynchronously, but units exit reset in a deterministic way), and these differences define how reset signals are generated. Reset signals propagate through all functional units asynchronously. For more information on RCU functionality, see the RCU register descriptions.

Table 6-1: ADSP-2184x RCU Register List

| Name          | Description                          |
|---------------|--------------------------------------|
| RCU_BCODE     | Boot Code Register                   |
| RCU_CRCTL     | Core Reset Outputs Control Register  |
| RCU_CRSTAT    | Core Reset Outputs Status Register   |
| RCU_CTL       | Control Register                     |
| RCU_MSG       | Message Register                     |
| RCU_MSG_CLR   | Message Clear Bits Register          |
| RCU_MSG_SET   | Message Set Bits Register            |
| RCU_SIDIS     | System Interface Disable Register    |
| RCU_SISTAT    | System Interface Status Register     |
| RCU_SRRQSTAT  | System Reset Request Status Register |
| RCU_STAT      | Status Register                      |
| RCU_SVECT0    | Software Vector Register 0           |
| RCU_SVECT1    | Software Vector Register 1           |
| RCU_SVECT2    | Software Vector Register 2           |
| RCU_SVECT_LCK | SVECT Lock Register                  |

## ADSP-2184x RCU Trigger List

Table 6-2: ADSP-2184x RCU Trigger List Generators

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

Table 6-3: ADSP-2184x RCU Trigger List Receivers

|   Trigger ID | Name         | Description                  | Sensitivity   |
|--------------|--------------|------------------------------|---------------|
|          147 | RCU0_SYSRST0 | RCU0 System Reset Receiver 0 | Pulse         |
|          148 | RCU0_SYSRST1 | RCU0 System Reset Receiver 1 | Pulse         |

## RCU Definitions

To make the best use of the RCU, it is useful to understand the terms in this section.

The target or source defines the following are types of resets.

## Hardware Reset (by target)

All functional units except a small subsection of debug interfaces are set to their default states. State is lost in all non-volatile storage.

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
| SYSCLK clock domain system reset by the Fault Unit (FMU) or TRU completer | System Reset    | All functional units, except: • RTC (if present) • RCU_STAT • RCU_BCODE • the units on the VDDEXT power domain |
| RCU_CTL.SYSRST bit set (software triggered reset)                         | System Reset    | All functional units, except: • RTC (if present) • RCU_STAT • RCU_BCODE • the units on the VDDEXT power domain |
| RCU_CRCTL.CR[n] bit set (software triggered reset)                        | Core Only Reset | Core n only                                                                                                    |

## RCU Status and Error Signals

The RCU\_STAT register reflects status and error information. There are three kinds of errors that can occur in the RCU. The reset out error is triggered when RSTOUT is both asserted and deasserted at the same time. The lock write error occurs if an attempt is made to write a lock RCU register. The address error occurs if a read-only register is written to or if an attempt is made to a reserved address within the RCU MMR address range.

## Core n Reset In

Core n can be requested to stop its IO activity and acknowledge to the requester when the task is completed.

To reset core n:

1. Clear the RCU\_CRSTAT.CR[n] bit.
2. Disable interrupts to core n.
3. Set (=1 ) RCU\_CRCTL.CR[n] to reset core n.

4. Check the RCU\_CRSTAT.CR[n] bit. If core n is in reset, continue.

## Core n Reset Out

To take core n out of reset:

1. Clear the RCU\_CRCTL.CR[n] bit.
2. Check the RCU\_CRSTAT.CR[n] bit. If core n is in out of reset, continue.

## ADSP-2184x RCU Register Descriptions

Reset Control Unit (RCU) contains the following registers.

Table 6-5: ADSP-2184x RCU Register List

| Name          | Description                          |
|---------------|--------------------------------------|
| RCU_BCODE     | Boot Code Register                   |
| RCU_CRCTL     | Core Reset Outputs Control Register  |
| RCU_CRSTAT    | Core Reset Outputs Status Register   |
| RCU_CTL       | Control Register                     |
| RCU_MSG       | Message Register                     |
| RCU_MSG_CLR   | Message Clear Bits Register          |
| RCU_MSG_SET   | Message Set Bits Register            |
| RCU_SIDIS     | System Interface Disable Register    |
| RCU_SISTAT    | System Interface Status Register     |
| RCU_SRRQSTAT  | System Reset Request Status Register |
| RCU_STAT      | Status Register                      |
| RCU_SVECT0    | Software Vector Register 0           |
| RCU_SVECT1    | Software Vector Register 1           |
| RCU_SVECT2    | Software Vector Register 2           |
| RCU_SVECT_LCK | SVECT Lock Register                  |

## Boot Code Register

The RCU\_BCODE register can be used to determine if and how core boots. This register is set to its default values by RESET.

Figure 6-1: RCU\_BCODE Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000000_c4eebfcea2288b52be72060be40522da1c8279702908785c1dbdb0c5a5265ac3.png)

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
| 0 (R/W)            | NOKERNEL   | No Boot Kernel. The RCU_BCODE.NOKERNEL bit configures the RCU to not execute the boot kernel.                               |

## Core Reset Outputs Control Register

The RCU core reset control n registers ( RCU\_CRCTL ) include a lock bit ( RCU\_CRCTL.LOCK ) and a core reset bit ( RCU\_CRCTL.CR[n] ) for each core reset signal on the product.

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000001_94c56d62d77ac51ec5e371a6dc4e686f8f5a05d15108843ceca421c7c9d19c1b.png)

Lock

Figure 6-2: RCU\_CRCTL Register Diagram

Table 6-7: RCU\_CRCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                | Description/Enumeration                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CRCTL.LOCK bit is set, the RCU_CRCTL register is read only (locked).                            | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CRCTL.LOCK bit is set, the RCU_CRCTL register is read only (locked).                            |
|                    |            | 0                                                                                                                                                                      | Unlock                                                                                                                                                                 |
|                    |            | 1                                                                                                                                                                      | Lock                                                                                                                                                                   |
| 3:0 (R/W)          | CR[n]      | Core Reset Outputs. The RCU_CRCTL.CR[n] bits control CRES[1:0] core reset signals. The RCU_CRES[n] signals can be individually controlled. They are reset to their de- | Core Reset Outputs. The RCU_CRCTL.CR[n] bits control CRES[1:0] core reset signals. The RCU_CRES[n] signals can be individually controlled. They are reset to their de- |
|                    |            | 0                                                                                                                                                                      | RCU_CRES[3:0] Deasserted                                                                                                                                               |
|                    |            | 1                                                                                                                                                                      | Reserved                                                                                                                                                               |
|                    |            | 2                                                                                                                                                                      | RCU_CRES[1] SHARC0 reset control bit asserted                                                                                                                          |
|                    |            | 4                                                                                                                                                                      | RCU_CRES[2] A55 reset control bit asserted                                                                                                                             |
|                    |            | 8                                                                                                                                                                      | RCU_CRES[3] DEBUG reset control bit asserted                                                                                                                           |
|                    |            | 15                                                                                                                                                                     | RCU_CRES[3:0] Asserted                                                                                                                                                 |

## Core Reset Outputs Status Register

The RCU core reset status register ( RCU\_CRSTAT ) contains status bits, indicating which core reset signals have been asserted.

Figure 6-3: RCU\_CRSTAT Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000002_8570f1af184ba7673659e681ccc517d49d3d960b504f05bc594127141a4ccc89.png)

Table 6-8: RCU\_CRSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W1C)        | CR[n]      | Core Reset Outputs. The RCU_CRSTAT.CR[n] bits indicate which cores have been reset since the last time the bit was cleared. Bits masked by CORE_DISABLE_MASK[15:0] are perma- nently disabled and the corresponding CR bits set. CR bits are sticky, they need to be cleared by software. | Core Reset Outputs. The RCU_CRSTAT.CR[n] bits indicate which cores have been reset since the last time the bit was cleared. Bits masked by CORE_DISABLE_MASK[15:0] are perma- nently disabled and the corresponding CR bits set. CR bits are sticky, they need to be cleared by software. |
|                    |            | 0                                                                                                                                                                                                                                                                                         | RCU_CRES[1:0] deasserted. CR[n] corresponds to RCU_CRES[n].                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                         | RCU_CRES[0] ARM A5 reset control bit asserted                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                         | RCU_CRES[1] SHARC0 reset control bit asserted                                                                                                                                                                                                                                             |
|                    |            | 4                                                                                                                                                                                                                                                                                         | RCU_CRES[2] A55 reset control bit asserted                                                                                                                                                                                                                                                |
|                    |            | 8                                                                                                                                                                                                                                                                                         | RCU_CRES[3] DEBUG reset control bit asserted                                                                                                                                                                                                                                              |
|                    |            | 15                                                                                                                                                                                                                                                                                        | RCU_CRES[3:0] were asserted since the last time bits were cleared. CR[n] corresponds to RCU_CRES[n].                                                                                                                                                                                      |

## Control Register

The RCU control register ( RCU\_CTL ) provides a register lock, enables for the core and system reset requests inputs and control for the Reset Output pin.

Figure 6-4: RCU\_CTL Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000003_047a1156fe93b0d467de63ab23886a617155a2948681bcebe1468a6b48a3e015.png)

Table 6-9: RCU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_CTL.LOCK bit is set, the RCU_CTL register is read only (locked). This bit is cleared by a hard reset or any system reset event. 0 Unlock                        |
| 10 (R/W)           | CRSTMSKSEL | Core Reset System Reset Mask Select. The RCU_CTL.CRSTMSKSEL bit selects the core reset system reset mask. This bit is cleared by a hard reset.                                                                                         |
| 9 (R/W)            | CRSTREQEN  | Core Reset Request Enabled. The RCU_CTL.CRSTREQEN bit controls whether the SYSCLK domain source(s) of reset is/are enabled to reset the core(s) when asserted. This bit is cleared by hard reset or any system reset event. 0 Disabled |
| 9 (R/W)            | CRSTREQEN  | 1 Enabled                                                                                                                                                                                                                              |

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

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000004_7bea58256cde890f8cdf044730b55ed03360c86db31dd1222c59bc05d91d06e2.png)

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
| 26 (R/W)           | HALTONCALL | Halt on Callback Call. The RCU_MSG.HALTONCALL bit generates an emulation exception prior to a call- back call.                   |
| 26 (R/W)           | HALTONCALL | 0 Do not generate exception                                                                                                      |
| 25 (R/W)           | HALTONINIT | Halt on Initcode Call. The RCU_MSG.HALTONINIT bit generates an emulation exception prior to an initcode call.                    |
| 25 (R/W)           | HALTONINIT | 0 Do not generate exception                                                                                                      |
| 25 (R/W)           | HALTONINIT | 1 Generate exception                                                                                                             |
| 24 (R/W)           | HALTONAPP  | Halt on Application Call. The RCU_MSG.HALTONAPP bit generates an emulation exception prior to an appli- cation call.             |
| 24 (R/W)           | HALTONAPP  | 0 Do not generate exception                                                                                                      |
| 24 (R/W)           | HALTONAPP  | 1 Generate exception                                                                                                             |
| 23 (R/W)           | L3INIT     | L3 Initialized. The RCU_MSG.L3INIT bit indicates that the L3 resource is initialized. not initialized                            |
| 23 (R/W)           | L3INIT     | 0 Resource                                                                                                                       |
| 22 (R/W)           | L2INIT     | L2 Initialized. The RCU_MSG.L2INIT bit indicates that the L2 resource is initialized.                                            |
| 22 (R/W)           | L2INIT     | 0 Resource not initialized                                                                                                       |
| 22 (R/W)           | L2INIT     | 1 Resource initialized                                                                                                           |

Table 6-10: RCU\_MSG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | C2ACTIVATE | Core 2 Activated. The RCU_MSG.C2ACTIVATE bit is used by tools for activation of Core 2.                                                                 |
| 19 (R/W)           | C1ACTIVATE | Core 1 Activated. The RCU_MSG.C1ACTIVATE bit is used by tools for activation of Core 1.                                                                 |
| 18 (R/W)           | C2L1INIT   | Core 2 L1 Initialized. The RCU_MSG.C2L1INIT bit indicates that the core 2 L1 resource is initialized.                                                   |
| 17 (R/W)           | C1L1INIT   | Core 1 L1 Initialized. The RCU_MSG.C1L1INIT bit indicates that the core 1 L1 resource is initialized. 0 Resource not initialized                        |
| 16 (R/W)           | C0L1INIT   | Core 0 L1 Initialized. The RCU_MSG.C0L1INIT bit indicates that the core 0 L1 resource is initialized. 0 Resource not initialized 1 Resource initialized |
| 10 (R/W)           | C2IDLE     | Core 2 Idle. The RCU_MSG.C2IDLE bit indicates that core 2 is in a safe idle state in ROM.                                                               |
| 9 (R/W)            | C1IDLE     | Core 1 Idle. The RCU_MSG.C1IDLE bit indicates that core 1 is in a safe idle state in ROM.                                                               |
| 8 (R/W)            | C0IDLE     | Core 0 Idle. The RCU_MSG.C0IDLE bit indicates that core 0 is in a safe idle state in ROM.                                                               |
| 7:0 (R/W)          | ERRCODE    | ROMError Code. The RCU_MSG.ERRCODE bit indicates the error code of the ROM. It is valid only when in the error handler.                                 |

## Message Clear Bits Register

The RCU\_MSG\_CLR register is used to clear bits in RCU\_MSG register. Reading this register returns 0x00000000.

Figure 6-6: RCU\_MSG\_CLR Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000005_bf99f3ec8a598a496217497801114fe14b18f0dfd7f6452e4bbd93ee3b9d7f00.png)

Table 6-11: RCU\_MSG\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                   |
|--------------------|------------|-------------------------------------------|
| 31:0               | CLR        | Clear MSG Register Bits.                  |
| (R0/W1C)           |            | The RCU_MSG_CLR.CLR bit resets MSG bit n. |

## Message Set Bits Register

The RCU\_MSG\_SET register is used to set bits in RCU\_MSG register. Reading this register returns 0x00000000.

Figure 6-7: RCU\_MSG\_SET Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000006_8589d5c3edc1e2c5f4200d3cb6ed809cc909f89195e89c35dbc89c3e4ff909a3.png)

Table 6-12: RCU\_MSG\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 31:0               | SET        | Set Message Bits.                       |
| (R0/W1S)           |            | The RCU_MSG_SET.SET bit sets MSG bit n. |

## System Interface Disable Register

The RCU system interface disable register ( RCU\_SIDIS ) lets the RCU assert a system interface disable request to functional units in the processor. This register is set to its default values by a hard reset or any system reset event. For information on mapping between RCU\_SIDIS bits and functional units, see the RCU functional description.

Figure 6-8: RCU\_SIDIS Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000007_b11667e906adceb726661e03a6354d40c013c7571f387e7eb4450898c9da3dbe.png)

Table 6-13: RCU\_SIDIS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SIDIS.LOCK bit is set, the RCU_SIDIS register is read only (locked).                                                 |
| 2:0 (R/W)          | SI[n]      | System Interface Disable Request [2:0]. Each RCU_SIDIS.SI[n] bit corresponds to a functional unit in the processor that supports the system interface disable request-acknowledge protocol. |
|                    |            | 0 RCU_SI_DISABLE_REQ[2:0] deasserted                                                                                                                                                        |
|                    |            | 1 RCU_SI_DISABLE_REQ[0] system interface disable request to SHARC0 asserted                                                                                                                 |
|                    |            | 2 RCU_SI_DISABLE_REQ[1] system interface disable request to A55 asserted                                                                                                                    |
|                    |            | 4                                                                                                                                                                                           |
|                    |            | Reserved                                                                                                                                                                                    |
|                    |            | 7 RCU_SI_DISABLE_REQ[2:0] asserted                                                                                                                                                          |

## System Interface Status Register

The RCU system interface status register ( RCU\_SISTAT ) indicates whether a functional unit has or has not acknowledged an RCU unit disable request.

Figure 6-9: RCU\_SISTAT Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000008_736a50eb090452ed76e88726f1f53fa1c78e38d120de9c66a5cf46dceb393f64.png)

Table 6-14: RCU\_SISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/NW)         | SI[n]      | System Interface Disable Acknowledge [2:0]. The RCU_SISTAT.SI[n] bit indicates whether a functional unit has or has not acknowledged an RCU unit disable request. | System Interface Disable Acknowledge [2:0]. The RCU_SISTAT.SI[n] bit indicates whether a functional unit has or has not acknowledged an RCU unit disable request. |
|                    |            | 0                                                                                                                                                                 | No Acknowledge                                                                                                                                                    |
|                    |            | 1                                                                                                                                                                 | RCU_SI_DISABLE_ACK[0] system interface disable acknowledge from SHARC0 asserted                                                                                   |
|                    |            | 2                                                                                                                                                                 | RCU_SI_DISABLE_ACK[1] system interface disable acknowledge from A55 asserted                                                                                      |
|                    |            | 4                                                                                                                                                                 | Reserved                                                                                                                                                          |
|                    |            | 7                                                                                                                                                                 | SI_DISABLE_ACK[2:0] asserted                                                                                                                                      |

## System Reset Request Status Register

The RCU system reset request status register ( RCU\_SRRQSTAT ) contains status bits, indicating which system reset request input triggered a system reset. This register is set to its default values by a hard reset.

Figure 6-10: RCU\_SRRQSTAT Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000009_7cf9c87f862cd18bb2b136cac8d3106cfe22d3bc16f66dfbf2444861d76e3f41.png)

Table 6-15: RCU\_SRRQSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W1C)        | SRRQ       | System Reset Triggered by System Reset Request [n]. The RCU_SRRQSTAT.SRRQ bits are set by the assertion of the correspond- ing system reset request input and deasserted by writing "1" to the bit. The RCU_SRRQSTAT register is cleared by a hard reset. | System Reset Triggered by System Reset Request [n]. The RCU_SRRQSTAT.SRRQ bits are set by the assertion of the correspond- ing system reset request input and deasserted by writing "1" to the bit. The RCU_SRRQSTAT register is cleared by a hard reset. |
|                    |            | 1                                                                                                                                                                                                                                                         | RCU_SRRQ[0], System Reset Request from SEC as- serted                                                                                                                                                                                                     |
|                    |            | 2                                                                                                                                                                                                                                                         | RCU_SRRQ[1], System Reset Request from TRGS_RCU0_SYSRST0 asserted                                                                                                                                                                                         |
|                    |            | 4                                                                                                                                                                                                                                                         | RCU_SRRQ[2], System Reset Request from TRGS_RCU0_SYSRST1 asserted                                                                                                                                                                                         |
|                    |            | 8                                                                                                                                                                                                                                                         | RCU_SRRQ[3], System Reset Request from CTI3 as- serted                                                                                                                                                                                                    |

## Status Register

The RCU status register ( RCU\_STAT ) contains status bits for all RCU reset sources, reset status, and boot mode inputs. Status bits for reset sources are sticky and can cleared by software. Error status bits are cleared by any reset event.

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000010_58ddd7fd6f9becacbff2ab26b3f6b62933a1e2896539de041222ede69e9fedf9.png)

Lock Write Error

Figure 6-11: RCU\_STAT Register Diagram

Table 6-16: RCU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W1C)         | RSTOUTERR  | Reset Out Error. The RCU_STAT.RSTOUTERR bit indicates (if set) that a write attempted to set the RCU_CTL.RSTOUTASRT and RCU_CTL.RSTOUTDSRT simultaneously. This condition triggers a bus error. 0 No Error                                                         |
| 17 (R/W1C)         | LWERR      | Lock Write Error. The RCU_STAT.LWERR bit indicates (when set) there was an attempted write to an RCU register while the RCU_CTL.LOCK bit was set and the global lock bit is enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear 0 No Error |
| 17 (R/W1C)         | LWERR      | 1 Error Occurred                                                                                                                                                                                                                                                   |
| 17 (R/W1C)         | LWERR      |                                                                                                                                                                                                                                                                    |

Table 6-16: RCU\_STAT Register Fields (Continued)

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

Figure 6-12: RCU\_SVECT0 Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000011_8bce858a2d1bf4aff5f4793cea133c8e87a7ddc9ba6cf418e96ea5708d8e05d1.png)

Table 6-17: RCU\_SVECT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Core 0 Reset Vector.                                                      |
| (R/W)              |            | Core0 jumps to the address in the RCU_SVECT0.VALUE bit field after reset. |

## Software Vector Register 1

The RCU\_SVECT1 register contains the default location of the first instruction to execute after a reset.

Figure 6-13: RCU\_SVECT1 Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000012_e864ef73689fa7bc3ebebdc328ca2f8796193bb7c43ca5b70c625f85d8be2591.png)

Table 6-18: RCU\_SVECT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Reset Vector.                                                             |
| (R/W)              |            | Core1 jumps to the address in the RCU_SVECT1.VALUE bit field after reset. |

## Software Vector Register 2

The RCU\_SVECT2 register contains the default location of the first instruction to execute after a reset.

Figure 6-14: RCU\_SVECT2 Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000013_552d838cd7e177515a827d4d9ddd2db3ef79d33f6c7afef52a3fdecf4c69c8e5.png)

Table 6-19: RCU\_SVECT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 31:0               | VALUE      | Reset Vector.                                                             |
| (R/W)              |            | Core2 jumps to the address in the RCU_SVECT2.VALUE bit field after reset. |

## SVECT Lock Register

The RCU software vector lock register ( RCU\_SVECT\_LCK ) provides a register lock and software vector n enable bits for each processor core on the product. This register is set to its default values by a hard reset or any system reset event.

Figure 6-15: RCU\_SVECT\_LCK Register Diagram

![Image](09_Reset_Control_Unit_(RCU)_artifacts/image_000014_ac56c5a60bbf460449e848a51b2291890a6c1c426620cfd545697a47f996a39b.png)

Table 6-20: RCU\_SVECT\_LCK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SVECT_LCK.LOCK bit is set, the RCU_SVECT_LCK register is read only (locked). 0 Unlock       |
| 2:0 (R/W)          | SVECT[n]   | Lock SVECTn Registers. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the RCU_SVECT_LCK.SVECT[n] bit is set, the SVECT registers are read only (locked). |