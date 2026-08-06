# System Debug and Trace Unit (DBG)

<!-- source: 325_System_Debug_and_Trace_Unit_DBG.pdf | original pages 3924–4041 -->

## 51   System Debug and Trace Unit (DBG)

The system debug and trace unit is based on Arm Core Sight technology. CoreSight is a set of architecture specifications defining debug and trace architecture. The processor uses CoreSight infrastructure to provide industry standard debug and trace capabilities.

## http://infocenter.arm.com/help/

The applicable documentation for more details about the ArmCoreSight feature includes:

- CoreSight PFT Architecture Specification, ARM IHI 0035B (PFT)
- System Trace Macrocell, Programmers' Model Architecture Specification, ARM IHI 0054A (STM)
- CoreSight Trace Memory Controller, ARM DDI0461B (TMC)
- CoreSight Components Technical Reference Manual, ARM DDI 0314H (TPIU)
- Embedded Cross Trigger Technical Reference Manual, ARM DDI 0291A
- ETM Architecture Specification, ARM IHI0014Q

## DBG Features

The debug unit has the following features.

- System JTAG TAP controller for system debug features, boundary scan, and public JTAG features
- A debug interface to core, and other system resources
- Direct and run-time access to the memory system and system MMRs
- Direct control over system reset
- Support for debug immediately after reset (boot debug)
- Group halt (debug event immediately halts all specified endpoints)
- Real-time on-chip visibility is made available to all developers, including software developers

## Debug Functional Description

The following sections provide functional descriptions of the DBG unit.

## ADSP-SC59x CSPFT Register List

The CoreSight (TM) Program Flow Trace unit (CSPFT) provides debug features. A set of registers governs CSPFT operations. For more information on CSPFT functionality, see the CSPFT register descriptions.

Table 51-1: ADSP-SC59x CSPFT Register List

| Name               | Description                             |
|--------------------|-----------------------------------------|
| CSPFT_ACTR[n]      | Address Comparator Access Type Register |
| CSPFT_ACVR[n]      | Address Comparator Value Register       |
| CSPFT_AUTHSTATUS   | Authentication Status Register          |
| CSPFT_CCER         | Configuration Code Extension Register   |
| CSPFT_CID0         | Component ID0 Register                  |
| CSPFT_CID1         | Component ID1 Register                  |
| CSPFT_CID2         | Component ID2 Register                  |
| CSPFT_CID3         | Component ID3 Register                  |
| CSPFT_CIDCMR       | Context ID Comparator Mask Register     |
| CSPFT_CIDCVR[n]    | Context ID Comparator Value             |
| CSPFT_CLAIMCLR     | Claim Tag Clear Register                |
| CSPFT_CLAIMSET     | Claim Tag Set Register                  |
| CSPFT_CNTENR[n]    | Counter Enable Event Register           |
| CSPFT_CNTRLDEVR[n] | Counter Reload Event Register           |
| CSPFT_CNTRLDVR[n]  | Counter Reload Value Register           |
| CSPFT_CNTVR[n]     | Counter Value Register                  |
| CSPFT_CTL          | Main Control Register                   |
| CSPFT_DEVTYPE      | Device Type Identifier Register         |
| CSPFT_EXTOUTEVR[n] | External Output Event Register          |
| CSPFT_HWFEAT       | Hardware Feature Register               |
| CSPFT_LAR          | Lock Access Register                    |
| CSPFT_LSR          | Lock Status Register                    |
| CSPFT_PID0         | Peripheral ID0 Register                 |
| CSPFT_PID1         | Peripheral ID1 Register                 |
| CSPFT_PID2         | Peripheral ID2 Register                 |

Table 51-1: ADSP-SC59x CSPFT Register List (Continued)

| Name           | Description                        |
|----------------|------------------------------------|
| CSPFT_PID3     | Peripheral ID3 Register            |
| CSPFT_PID4     | Peripheral ID4 Register            |
| CSPFT_STAT     | Status Register                    |
| CSPFT_SYNCFR   | Synchronization Frequency Register |
| CSPFT_TRACEIDR | CoreSight Trace ID Register        |

## ADSP-SC59x TAPC Register List

The Test Access Port Controller (TAPC) provides access to debug features. A set of registers governs TAPC operations. For more information on TAPC functionality, see the TAPC register descriptions.

Table 51-2: ADSP-SC59x TAPC Register List

| Name              | Description                       |
|-------------------|-----------------------------------|
| TAPC_DBGCTL       | Debug Control Register            |
| TAPC_IDCODE       | IDCODE Register                   |
| TAPC_SDBGKEY0     | Secure Debug Key 0 Register       |
| TAPC_SDBGKEY1     | Secure Debug Key 1 Register       |
| TAPC_SDBGKEY2     | Secure Debug Key 2 Register       |
| TAPC_SDBGKEY3     | Secure Debug Key 3 Register       |
| TAPC_SDBGKEY_CTL  | Secure Debug Key Control Register |
| TAPC_SDBGKEY_STAT | Secure Debug Key Status Register  |
| TAPC_USERCODE     | USERCODE Register                 |

## DBG Block Diagram

The DBG block diagram is shown below.

Figure 51-1: Coresight in ADSP-SC598 Family

<!-- image -->

## DBG Definitions

The following terms are useful when working with the debug features of the processor and programming tools.

## Test Access Port Controller (TAPC)

Provides IDCODE and SDBGKEY features.

## Debug Access Port (DAP)

Core Sight Interface providing a single port for two debug options: JTAG-DP (JTAG Debug Port), SW-DP (Serial Wire Debug Port)

## Program Trace Macrocell (PTM)

Provides DSP () Core Trace.

## Standard Trace Macrocell (STM)

Provides capability to trace up to 32 hardware events and supports 32 software stimulus for data transfer between the user and emulator.

## Embedded Cross Trigger (ECT)

The ECTs are responsible managing events and triggers as follows.

- CTI (Cross Trigger Interface) is a CoreSight component for enabling cross triggering of events across a system. From the view of the ECT, it is responsible for combining and mapping trigger requests.
- CTM (Cross Trigger Matrix) is a CoreSight component for connecting multiple CTIs. From the view of the ECT, it is responsible for connecting CTIs and distribution of events.

NOTE: An embedded cross trigger is not the same as the requester and completer trigger in the trigger routing unit.

## Trace Capture Devices

The trace capture devices capture and format the trace data. There are two trace capture devices in the system:

- ETF - Embedded Trace FIFO - Provides a buffer for burst trace data.
- ETR - Embedded Trace Router - Provides interface for trace data to be stored in system memories.

## Trace Port Interface Unit (TPIU)

The TPIU acts as a bridge between the on-chip trace data, with separate IDs, to a data stream. It encapsulates IDs, when required, that the Trace Port Analyzer (TPA) captures.

## Test Access Port Controller (TAPC)

TAPC is an independent component daisy-chained to the DAP in the JTAG scan path. The TAPC component provides the product IDCODE (Chip ID) and a security feature.

The TAPC component provides security features to the chip using a debug key match features. This feature permits only those components which have the SDBGKEY to connect and debug the chip. Initially, a 128-bits user key is programmed in the SDBGKEY registers ( TAPC\_SDBGKEY0 , TAPC\_SDBGKEY1 , TAPC\_SDBGKEY2 , TAPC\_SDBGKEY3 ) in TAPC by a secure requester. The TAPC\_SDBGKEY\_CTL.VALID bit is set.

Then, a user key is entered through the emulator for a match to the initially programmed-entered user key. On a successful match, the chip connects to the emulator. It can be debugged on a failed user key match. Further attempts at keys matching are disabled. JTAG reset or system reset is required to reenable the user key match logic.

## Embedded Trace Macrocell (ETM)

ETM is the standard trace support provided for the Arm processor. For details of programming and functionality, refer to the Arm documentation.

## Debug Access Ports

The Debug Access Port (DAP) is an implementation of an Arm Debug Interface version 6 (ADIv6) comprising a number of components supplied in a single configuration. All the supplied components fit into the various architectural components for Debug Ports (DPs). The components are used to access the DAP from an external debugger and APs, to access on-chip system resources.

## Trace Unit

The trace module provides instruction, data tracing, and system activity tracing for the processor. The program trace module on the processor is similar to the embedded trace macrocell module provided by the Arm processor. System trace is provided using the system trace macrocell as part of the CoreSight debug and trace interface. The trace module uses an interface based on the AMBA Trace Bus (ATB) standard to output its trace data. The trace data can be

either exported to an off-chip trace port analyzer or captured on an on-chip buffer. The PFT and STM modules capture information on the processor both before and after a specific event. The modules add no burden to the processor performance when it runs at full speed.

- Programmable Flow Trace (CSPFT)
- System Trace Module (STM)

## Programmable Flow Trace (CSPFT)

When tracing processor execution, trace information can be generated for every instruction the processor executes. This information would be easy to interpret, but would require a prohibitively high trace bandwidth to get the trace data off the chip. With program flow tracing, only branch points are traced. The debugger uses the source code to infer the rest of the executed code.

Certain instructions in the program and events are identified as waypoints. A waypoint is a point where instruction execution involves a change of program flow. The CSPFT only traces those waypoints. These waypoints are:

- All indirect branches
- All direct branches
- Exceptions or interrupts
- Emulator debug entry and exit

When a waypoint occurs, trace data is generated to describe it. From this data and the source code, a trace decompressor can determine what instructions were executed and recreate the instruction flow. To allow the decompressor to calculate where it is in the source code, conditional instructions are marked as waypoints, regardless of whether they pass or fail their condition test. Events like interrupts or debug entry and exit can be promoted from nonwaypoint instructions to waypoints to trace the interrupted program flow.

Tracing a waypoint implies the execution of all instructions from the target address of the previous waypoint up to the current waypoint. Non-waypoint instructions are not explicitly traced but the debugger must infer them using the source code. The concept of an instruction block is used throughout this manual and refers to the contiguous block of instructions between two waypoints.

The programming model and function is a subset of the PTM (Programmable Trace Module) of Arm.

## System Trace Module (STM)

The STM is a trace source that is integrated into a CoreSight system, and is designed primarily for high-bandwidth trace of instrumentation embedded into software. The STM enables tracing of system activity from various sources:

- Instrumented software, using memory-mapped stimulus ports
- Hardware events

The STM supports the following features:

- Multiple software requesters writing software instrumentation independently. Each requester can use multiple stimulus ports.

- Time stamping of the system activity. The time stamp is a global time stamp which can be shared with other trace sources in the system to enable correlation of activity from multiple trace sources.
- Indication that specific events have occurred, such as a particular hardware event or a piece of software instrumentation. These events are known as triggers and can be indicated in the trace stream, or through signals to other system components.

## Embedded Cross Trigger (ECT)

ECT provides an interface to the CoreSight debug system enabling the subsystems to interact (cross trigger) with each other. ECT provides a mechanism to forward debug events from one connected subsystem to another connected subsystem. The different subsystems connected to the ECT depend on the processor design. For example, in a multiprocessor system, the interface can be connected to each of the cores and one to the trace subsystem. For a uniprocessor system, the interface can include just the core and trace subsystem connection.

- CTI cross trigger interface. A CoreSight component for enabling cross triggering of events across a system.
- CTM cross trigger matrix. A CoreSight component for connecting multiple cross trigger interfaces.

The main function of the ECT (CTI and CTM) is to pass debug events from one connected subsystem to another connected subsystem.

For example, the ECT can communicate debug state information from the core to trace subsystem for a single processor system or to another core in a multiprocessor-based system. Program execution on both the subsystem can be stopped at the same time.

The Trigger Flow figure shows a simple debug trigger flow sequence. On each CTI, there are four channel, eight input, and eight output debug triggers. All the eight inputs and outputs can be mapped to a single channel or different channels based on the debug trigger to channel mapping. When a trigger input occurs, it creates a channel event. The channel event causes all the output debug triggers to be triggered. The embedded cross trigger depends on the debug trigger it connects to.

Figure 51-2: Trigger Flow

<!-- image -->

Figure 51-3: ECT Integration

<!-- image -->

## CTI Debug Trigger Tables

The System CTI Trigger Connection tables show the debug trigger that connects to each CTI.

Table 51-3: System CTI Trigger Connection

| CTI Port      | Direction   | System Signal                                                       | CTI Signal Description   |
|---------------|-------------|---------------------------------------------------------------------|--------------------------|
| EVENTIN[7:3]  | Input       | From TRU                                                            | Trigger In               |
| EVENTIN[2]    | Input       | RCU_SYSRSTB                                                         | Trigger In               |
| EVENTIN[1:0]  | Input       | From TRU                                                            | Trigger In               |
| EVENTOUT[13]  | Output      | To A55 core sample request input                                    | Trigger Out              |
| EVENTOUT[12]  | Output      | To A55 core PMU snapshot request input                              | Trigger Out              |
| EVENTOUT[11]  | Output      | To DAP DP event status input                                        | Trigger Out              |
| EVENTOUT[10]  | Output      | To system time tsgen restart input                                  | Trigger Out              |
| EVENTOUT[9]   | Output      | To system time tsgen halt input                                     | Trigger Out              |
| EVENTOUT[8]   | Output      | To Coresight tsgen restart input                                    | Trigger Out              |
| EVENTOUT[7]   | Output      | To TRU and as DBGRESTART to CTITRIGOU- TACK[1] and CTITRIGOUTACK[0] | Trigger Out              |
| EVENTOUT[6:3] | Output      | To TRU                                                              | Trigger Out              |
| EVENTOUT[2]   | Output      | To RCU as system reset request                                      | Trigger Out              |
| EVENTOUT[1]   | Output      | To TRU and system peripheral halt                                   | Trigger Out              |
| EVENTOUT[0]   | Output      | To TRU and system fabric halt                                       | Trigger Out              |

Table 51-4: ADSP-SC598-Core 1/2 CTI Trace Connection

| CTI Port          | Direction   | SHARC-XI Signal   | CTI Signal Description   |
|-------------------|-------------|-------------------|--------------------------|
| CTITRIGIN[7]      | Input       | Tied low (Unused) | Trigger in               |
| CTITRIGIN[6]      | Input       | PTM TRIGGER       | Trigger in               |
| CTITRIGIN[5:2]    | Input       | PTM EXTOUT[3:0    | Trigger in               |
| CTITRIGIN[1]      | Input       | Tied low (Unused) | Trigger in               |
| CTITRIGIN[0]      | Input       | DBGTRIGGER        | Trigger in               |
| CTITRIGINACK[7]   | Output      | Unused            | Trigger in acknowledge   |
| CTITRIGINACK[6]   | Output      | PTM TRIGGER ACK   | Trigger in acknowledge   |
| CTITRIGINACK[5:1] | Output      | Unused            | Trigger in acknowledge   |
| CTITRIGINACK[0]   | Output      | DBGTRIGGER ACK    | Trigger in acknowledge   |
| CTITRIGOUT[7]     | Output      | DBGRESTART        | Trigger out              |
| CTITRIGOUT[6]     | Output      | SEC               | Trigger out              |
| CTITRIGOUT[5]     | Output      | Unused            | Trigger out              |
| CTITRIGOUT[4:1]   | Output      | PTM EXTIN[3:0]    | Trigger out              |
| CTITRIGOUT[0]     | Output      | EDBGRQ            | Trigger out              |

Table 51-4: ADSP-SC598-Core 1/2 CTI Trace Connection (Continued)

| CTI Port           | Direction   | SHARC-XI Signal   | CTI Signal Description   |
|--------------------|-------------|-------------------|--------------------------|
| CTITRIGOUTACK[7]   | Input       | DBGRESTARTED      | Trigger out acknowledge  |
| CTITRIGOUTACK[6:0] | Input       | Tied Low          |                          |

Table 51-5: ADSP-SC598 CTI Trace Connection

| CTI Port      | Direction   | Trace Block Signal      | CTI Signal Description   |
|---------------|-------------|-------------------------|--------------------------|
| EVENTIN[10]   | Input       | STM ASYNCOUT            | Trigger in               |
| EVENTIN[9]    | Input       | STM TRIGOUTHETE         | Trigger in               |
| EVENTIN[8]    | Input       | STM TRIGOUTSW           | Trigger in               |
| EVENTIN[7]    | Input       | STM TRIGOUTSPTE         | Trigger in               |
| EVENTIN[6]    | Input       | TPIU FLUSHCOMP          | Trigger in               |
| EVENTIN[5]    | Input       | ETR FLUSHCOMP           | Trigger in               |
| EVENTIN[4]    | Input       | ETR ACQCOMP             | Trigger in               |
| EVENTIN[3]    | Input       | ETR FULL                | Trigger in               |
| EVENTIN[2]    | Input       | ETF FLUSHCOMP           | Trigger in               |
| EVENTIN[1]    | Input       | ETF ACQCOMP             | Trigger in               |
| EVENTIN[0]    | Input       | ETF FULL                | Trigger in               |
| EVENTOUT[7:6] | Output      | trigout_cti_to_stm[1:0] | Trigger out              |
| EVENTOUT[5]   | Output      | TPIU FLUSHIN            | Trigger out              |
| EVENTOUT[4]   | Output      | TPIU TRIGIN             | Trigger out              |
| EVENTOUT[3]   | Output      | ETR FLUSHIN             | Trigger out              |
| EVENTOUT[2]   | Output      | ETR TRIGIN              | Trigger out              |
| EVENTOUT[1]   | Output      | ETF FLUSHIN             | Trigger out              |
| EVENTOUT[0]   | Output      | ETF TRIGIN              | Trigger out              |

Table 51-6: SC59x Arm Core CTI Connection

| CTI Port       | Input           | CTI Port        | Output                |
|----------------|-----------------|-----------------|-----------------------|
| CTITRIGIN[7:4] | EXT_EXTOUT[3:0] | CTITRIGOUT[7:4] | ETM EXTIN[3:0]        |
| CTITRIGIN[3]   | Unused          | CTITRIGOUT[3]   | Unused                |
| CTITRIGIN[2]   | Unused          | CTITRIGOUT[2 ]  | GIC Interrupt Request |
| CTITRIGIN[1]   | PMU IRQ         | CTITRIGOUT[1]   | DBGRESTART            |
| CTITRIGIN[0]   | DBGTRIGGER      | CTITRIGOUT[0]   | EDBGRQ                |

Table 51-7: Trigger Descriptions

| Signal Name           | Description                                                                                                                                                                                                                                                |
|-----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| STM ASYNCOUT          | Alignment synchronization output. This signal is asserted for one clock cycle when an ASYNC-VERSION-FREQ sequence is completely output on the ATB, and can be used for cross-triggering.                                                                   |
| STM TRIGOUTHETE       | Trigger output. This signal is asserted for one clock cycle when a trigger event is detected on match using STMHETER.                                                                                                                                      |
| STM TRIGOUTSW         | Trigger output. This signal is asserted for one clock cycle when a trigger event is generated on writes to a TRIG location in the extended stimulus port registers.                                                                                        |
| STM TRIGOUTSPTE       | Trigger output. This signal is asserted for one clock cycle when a trigger event is detected on match using STMSPTER.                                                                                                                                      |
| ETR / ETF FULL        | This output indicates the value of the full bit in the ETR/ETF status register. A full bit indicates the amount of data in ETF/ETR. An output signal indicating when the circular buffer or FIFO is full, or within a program- mable amount of being full. |
| ETR / ETF ACQCOMP     | This output indicates the value of the FtEmpty bit in the ETR/ETF status register. The bit it is set when trace capture has stopped. An output signal indicating when trace capture has stopped, usually following a trigger condition.                    |
| ETR/ETF/ TPIU TRIGIN  | This input can cause a trigger event (Start /Stop Trigger). An input signal indicating when a trigger condition has occurred.                                                                                                                              |
| ETR/ETF /TPIU FLUSHIN | This input can cause a Trace flush. An input signal indicating a flush request                                                                                                                                                                             |
| PTM/ETM TRIGGER       | Trigger Input. Trigger event specifies the conditions that must be met to generate a trigger.                                                                                                                                                              |
| PTM/ETM EXTOUT[3:0]   | PTM Output.                                                                                                                                                                                                                                                |
| PTM/ETM EXTIN[3:0]    | PTM Input                                                                                                                                                                                                                                                  |
| SEC                   | CTI Interrupt                                                                                                                                                                                                                                              |
| DBGRESTART            | This is an output from the CTI to a processor core or to system to return from debug mode.                                                                                                                                                                 |
| DBGTRIGGER            | This is a processor core output signal indicating that the core has moved to debug mode. If the CTIs are setup for synchronous halt, it will generate EDBGRQ to everyone else.                                                                             |
| EDBGRQ                | This is an output from CTI and an input (as debug halt) to a processor core and to the system in general. It can assert as a result of another core going to emulation space (DBGTRIGGER) or by setting the corresponding bit in the CTI.                  |
| SYS_DBGRESTART        | This is an output from the CTI to system to return from debug mode.                                                                                                                                                                                        |
| To TRU                | TRU Controller Event                                                                                                                                                                                                                                       |
| From TRU              | TRU Target Event                                                                                                                                                                                                                                           |

## ADSP-SC59x CSPFT Register Descriptions

Program Flow Trace (CSPFT) contains the following registers.

Table 51-8: ADSP-SC59x CSPFT Register List

| Name               | Description                             |
|--------------------|-----------------------------------------|
| CSPFT_ACTR[n]      | Address Comparator Access Type Register |
| CSPFT_ACVR[n]      | Address Comparator Value Register       |
| CSPFT_AUTHSTATUS   | Authentication Status Register          |
| CSPFT_CCER         | Configuration Code Extension Register   |
| CSPFT_CID0         | Component ID0 Register                  |
| CSPFT_CID1         | Component ID1 Register                  |
| CSPFT_CID2         | Component ID2 Register                  |
| CSPFT_CID3         | Component ID3 Register                  |
| CSPFT_CIDCMR       | Context ID Comparator Mask Register     |
| CSPFT_CIDCVR[n]    | Context ID Comparator Value             |
| CSPFT_CLAIMCLR     | Claim Tag Clear Register                |
| CSPFT_CLAIMSET     | Claim Tag Set Register                  |
| CSPFT_CNTENR[n]    | Counter Enable Event Register           |
| CSPFT_CNTRLDEVR[n] | Counter Reload Event Register           |
| CSPFT_CNTRLDVR[n]  | Counter Reload Value Register           |
| CSPFT_CNTVR[n]     | Counter Value Register                  |
| CSPFT_CTL          | Main Control Register                   |
| CSPFT_DEVTYPE      | Device Type Identifier Register         |
| CSPFT_EXTOUTEVR[n] | External Output Event Register          |
| CSPFT_HWFEAT       | Hardware Feature Register               |
| CSPFT_LAR          | Lock Access Register                    |
| CSPFT_LSR          | Lock Status Register                    |
| CSPFT_PID0         | Peripheral ID0 Register                 |
| CSPFT_PID1         | Peripheral ID1 Register                 |
| CSPFT_PID2         | Peripheral ID2 Register                 |
| CSPFT_PID3         | Peripheral ID3 Register                 |
| CSPFT_PID4         | Peripheral ID4 Register                 |
| CSPFT_STAT         | Status Register                         |
| CSPFT_SYNCFR       | Synchronization Frequency Register      |

Table 51-8: ADSP-SC59x CSPFT Register List (Continued)

| Name           | Description                 |
|----------------|-----------------------------|
| CSPFT_TRACEIDR | CoreSight Trace ID Register |

## Address Comparator Access Type Register

The CSPFT\_ACTR[n] register specifies whether the context ID needs to match.

Figure 51-4: CSPFT\_ACTR[n] Register Diagram

<!-- image -->

Table 51-9: CSPFT\_ACTR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                            | Description/Enumeration                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|
| 9:8                | CIDCTRL    | Context ID Comparator Control. The CSPFT_ACTR[n].CIDCTRL contains the ID comparator control value. | Context ID Comparator Control. The CSPFT_ACTR[n].CIDCTRL contains the ID comparator control value. |
| (R/W)              |            | 0                                                                                                  | Ignore Context ID                                                                                  |
|                    |            | 1                                                                                                  | Match if Context ID Comparator 0 Matches                                                           |
|                    |            | 2                                                                                                  | Match if Context ID Comparator 1 Matches                                                           |
|                    |            | 3                                                                                                  | Match if Context ID Comparator 2 Matches                                                           |

## Address Comparator Value Register

The CSPFT\_ACVR[n] register holds an address for comparison.

Figure 51-5: CSPFT\_ACVR[n] Register Diagram

<!-- image -->

Table 51-10: CSPFT\_ACVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 31:0               | ADDR       | Address for Comparison.                                                    |
| (R/W)              |            | The CSPFT_ACVR[n].ADDR bit field contains the address used for comparison. |

## Authentication Status Register

The CSPFT\_AUTHSTATUS register reports the level of tracing currently permitted based on the DBGEN signal.

Figure 51-6: CSPFT\_AUTHSTATUS Register Diagram

<!-- image -->

Table 51-11: CSPFT\_AUTHSTATUS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | ONE        | Always reads as one.                                                                                                                                                                                                                                                                                                                                                                         |
| 6 (R/NW)           | DBGEN      | Debug Enabled. The CSPFT_AUTHSTATUS.DBGEN bit indicates that invasive debug is enabled. Normally, NIDEN is used in conjunction with a signal that enables invasive debug, DBGEN. Non-invasive debug is disabled only if both NIDEN and DBGEN signals are LOW. In a PTM, typically these signals are ORed together and the result is used to determine whether non-invasive debug is enabled. |

## Configuration Code Extension Register

The CSPFT\_CCER register holds extra feature information. (See CSPFT\_HWFEAT .)

Figure 51-7: CSPFT\_CCER Register Diagram

<!-- image -->

Table 51-12: CSPFT\_CCER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/NW)          | VEI        | Virtualization Extensions Implemented. The CSPFT_CCER.VEI bit indicates if the virtualization extensions are implement- ed 0 Not Implemented |
| 23 (R/NW)          | RSI        | Return Stack Implemented. The CSPFT_CCER.RSI bit indicates if a return stack is implemented. 0 Not Implemented                               |
| 22 (R/NW)          | TSI        | Time Stamping Implemented. The CSPFT_CCER.TSI bit indicates if time stamping is implemented. 0 Disabled 1 Enabled                            |

## Component ID0 Register

The CSPFT\_CID0 register holds sections of the CoreSight Component ID for CSPFT.

Figure 51-8: CSPFT\_CID0 Register Diagram

<!-- image -->

Table 51-13: CSPFT\_CID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID0.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Component ID1 Register

The CSPFT\_CID1 register holds sections of the CoreSight Component ID for CSPFT.

Figure 51-9: CSPFT\_CID1 Register Diagram

<!-- image -->

Table 51-14: CSPFT\_CID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID1.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Component ID2 Register

The CSPFT\_CID2 register holds sections of the CoreSight Component ID for CSPFT.

Figure 51-10: CSPFT\_CID2 Register Diagram

<!-- image -->

Table 51-15: CSPFT\_CID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID2.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Component ID3 Register

The CSPFT\_CID3 register holds sections of the CoreSight Component ID for CSPFT.

Figure 51-11: CSPFT\_CID3 Register Diagram

<!-- image -->

Table 51-16: CSPFT\_CID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID3.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Context ID Comparator Mask Register

The CSPFT\_CIDCMR register holds a 32-bit mask for use for all context ID comparisons.

Figure 51-12: CSPFT\_CIDCMR Register Diagram

<!-- image -->

Table 51-17: CSPFT\_CIDCMR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | MASK       | Context ID Mask Value. The CSPFT_CIDCMR.MASK bit field holds a 32-bit mask for use in all context ID comparisons. |

## Context ID Comparator Value

The CSPFT\_CIDCVR[n] register holds a context ID value for comparison.

Figure 51-13: CSPFT\_CIDCVR[n] Register Diagram

<!-- image -->

Table 51-18: CSPFT\_CIDCVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Context ID Comparator Value Register. The CSPFT_CIDCVR[n].VALUE bit field holds a context ID value for compari- son. |

## Claim Tag Clear Register

The CSPFT\_CLAIMCLR register is used to clear bits in the claim tag or get the current value of the claim tag.

Figure 51-14: CSPFT\_CLAIMCLR Register Diagram

<!-- image -->

Table 51-19: CSPFT\_CLAIMCLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------|
| 3:0 (R/W1C)        | TAGS       | Tags. A read of the CSPFT_CLAIMCLR.TAGS bit field returns the current value, a write clears bits. |

## Claim Tag Set Register

The CSPFT\_CLAIMSET register is used to set bits in the claim tag and find the number of bits supported by the claim tag.

Figure 51-15: CSPFT\_CLAIMSET Register Diagram

<!-- image -->

Table 51-20: CSPFT\_CLAIMSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R1/W1S)       | TAGS       | Supported Tags. The CSPFT_CLAIMSET.TAGS bit field sets bits in the claim tag and finds the number of bits supported by the claim tag. |

## Counter Enable Event Register

The CSPFT\_CNTENR[n] register describes the event that enables the corresponding counter.

Figure 51-16: CSPFT\_CNTENR[n] Register Diagram

<!-- image -->

Table 51-21: CSPFT\_CNTENR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:14 (R/W)        | FUNC       | Function. The CSPFT_CNTENR[n].FUNC bit field specifies the logical operation that com- bines the two resources that define the event.                                                                                               | Function. The CSPFT_CNTENR[n].FUNC bit field specifies the logical operation that com- bines the two resources that define the event.                                                                                               |
|                    |            | 0                                                                                                                                                                                                                                   | A                                                                                                                                                                                                                                   |
|                    |            | 1                                                                                                                                                                                                                                   | NOT(A)                                                                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                   | A AND B                                                                                                                                                                                                                             |
|                    |            | 3                                                                                                                                                                                                                                   | NOT(A) AND B                                                                                                                                                                                                                        |
|                    |            | 4                                                                                                                                                                                                                                   | NOT(A) AND NOT(B)                                                                                                                                                                                                                   |
|                    |            | 5                                                                                                                                                                                                                                   | A OR B                                                                                                                                                                                                                              |
|                    |            | 6                                                                                                                                                                                                                                   | NOT(A) OR B                                                                                                                                                                                                                         |
|                    |            | 7                                                                                                                                                                                                                                   | NOT(A) OR NOT(B)                                                                                                                                                                                                                    |
| 13:7 (R/W)         | RESB       | Resource B. The CSPFT_CNTENR[n].RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTENR[n].FUNC field (See CSPFT_CNTENR[n].RESA for list of possible values). | Resource B. The CSPFT_CNTENR[n].RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTENR[n].FUNC field (See CSPFT_CNTENR[n].RESA for list of possible values). |
| 6:0 (R/W)          | RESA       | Resource A. The CSPFT_CNTENR[n].RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTENR[n].FUNC field.                                                        | Resource A. The CSPFT_CNTENR[n].RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTENR[n].FUNC field.                                                        |

Table 51-21: CSPFT\_CNTENR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |    | Description/Enumeration   |
|--------------------|------------|----|---------------------------|
|                    |            |  0 | Single Addr Comparator 0  |
|                    |            |  1 | Single Addr Comparator 1  |
|                    |            |  2 | Single Addr Comparator 2  |
|                    |            |  3 | Single Addr Comparator 3  |
|                    |            |  4 | Single Addr Comparator 4  |
|                    |            |  5 | Single Addr Comparator 5  |
|                    |            |  6 | Single Addr Comparator 6  |
|                    |            |  7 | Single Addr Comparator 7  |
|                    |            |  8 | Single Addr Comparator 8  |
|                    |            |  9 | Single Addr Comparator 9  |
|                    |            | 10 | Single Addr Comparator 10 |
|                    |            | 11 | Single Addr Comparator 11 |
|                    |            | 12 | Single Addr Comparator 12 |
|                    |            | 13 | Single Addr Comparator 13 |
|                    |            | 14 | Single Addr Comparator 14 |
|                    |            | 15 | Single Addr Comparator 15 |
|                    |            | 16 | Addr Range Comparator 0   |
|                    |            | 17 | Addr Range Comparator 1   |
|                    |            | 18 | Addr Range Comparator 2   |
|                    |            | 19 | Addr Range Comparator 3   |
|                    |            | 20 | Addr Range Comparator 4   |
|                    |            | 21 | Addr Range Comparator 5   |
|                    |            | 22 | Addr Range Comparator 6   |
|                    |            | 23 | Addr Range Comparator 7   |
|                    |            | 64 | Counter 0 at Zero         |
|                    |            | 65 | Counter 1 at Zero         |
|                    |            | 66 | Counter 2 at Zero         |
|                    |            | 67 | Counter 3 at Zero         |
|                    |            | 88 | Context ID Comparator 0   |
|                    |            | 89 | Context ID Comparator 1   |

Table 51-21: CSPFT\_CNTENR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration                |
|--------------------|------------|---------------------------|----------------------------------------|
|                    |            |                        95 | TraceEnable Start/Stop Resource 0 or 1 |
|                    |            |                        96 | External Inputs 0                      |
|                    |            |                        97 | External Inputs 1                      |
|                    |            |                        98 | External Inputs 2                      |
|                    |            |                        99 | External Inputs 3                      |
|                    |            |                       110 | Trace Prohibited                       |
|                    |            |                       111 | Always TRUE                            |

## Counter Reload Event Register

The CSPFT\_CNTRLDEVR[n] register defines the event that causes the corresponding counter to be reloaded with the value held in the corresponding CSPFT\_CNTRLDVR[n] register.

Figure 51-17: CSPFT\_CNTRLDEVR[n] Register Diagram

<!-- image -->

Table 51-22: CSPFT\_CNTRLDEVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:14 (R/W)        | FUNC       | Function. The CSPFT_CNTRLDEVR[n].FUNC bit field specifies the logical operation that combines the two resources that define the event. 0 A 1 NOT(A) 2 A AND B 3 NOT(A) AND B 4 NOT(A) AND NOT(B) 5 A OR B                                    |
| 16:14 (R/W)        | FUNC       | 6 NOT(A) OR B                                                                                                                                                                                                                                |
| 16:14 (R/W)        | FUNC       | 7 NOT(A) OR NOT(B)                                                                                                                                                                                                                           |
| 13:7 (R/W)         | RESB       | Resource B. The CSPFT_CNTRLDEVR[n].RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTRLDEVR[n].FUNC field (See CSPFT_CNTRLDEVR[n].RESA for list of possible values). |
| 16:14 (R/W)        | FUNC       |                                                                                                                                                                                                                                              |
| 16:14 (R/W)        | FUNC       |                                                                                                                                                                                                                                              |
| 16:14 (R/W)        | FUNC       |                                                                                                                                                                                                                                              |
| 16:14 (R/W)        | FUNC       |                                                                                                                                                                                                                                              |
| 16:14 (R/W)        | FUNC       |                                                                                                                                                                                                                                              |

Table 51-22: CSPFT\_CNTRLDEVR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | RESA       | Resource A. The CSPFT_CNTRLDEVR[n].RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTRLDEVR[n].FUNC field. 0 Single Addr Comparator 0 1 Single Addr Comparator 1 2 Single Addr Comparator 2 3 Single Addr Comparator 3 4 Single Addr Comparator 4 5 Single Addr Comparator 5 6 Single Addr Comparator 6 7 Single Addr Comparator 7 8 Single Addr Comparator 8 9 Single Addr Comparator 9 10 Single Addr Comparator 10 11 Single Addr Comparator 11 12 Single Addr Comparator 12 13 Single Addr Comparator 13 14 Single Addr Comparator 14 15 Single Addr Comparator 15 16 Addr Range Comparator 0 17 Addr Range Comparator 1 18 Addr Range Comparator 2 19 Addr Range Comparator 3 20 Addr Range Comparator 4 21 Addr Range Comparator 5 22 Addr Range Comparator 6 23 Addr Range Comparator 7 |

Table 51-22: CSPFT\_CNTRLDEVR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration |                                        |
|--------------------|------------|---------------------------|----------------------------------------|
|                    |            |                        67 | Counter 3 at zero                      |
|                    |            |                        88 | Context ID comparator 0                |
|                    |            |                        89 | Context ID comparator 1                |
|                    |            |                        90 | Context ID comparator 2                |
|                    |            |                        95 | TraceEnable start/stop resource 0 or 1 |
|                    |            |                        96 | External Inputs 0                      |
|                    |            |                        97 | External Inputs 1                      |
|                    |            |                        98 | External Inputs 2                      |
|                    |            |                        99 | External Inputs 3                      |
|                    |            |                       110 | Trace prohibited                       |
|                    |            |                       111 | Always TRUE                            |

## Counter Reload Value Register

The CSPFT\_CNTRLDVR[n] register specifies the starting value of the corresponding counter.

Figure 51-18: CSPFT\_CNTRLDVR[n] Register Diagram

<!-- image -->

Table 51-23: CSPFT\_CNTRLDVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Counter Initial Value. The CSPFT_CNTRLDVR[n].VALUE bit field specifies the starting value of the cor- responding counter. |

## Counter Value Register

The CSPFT\_CNTVR[n] register holds the current value of the corresponding counter.

Figure 51-19: CSPFT\_CNTVR[n] Register Diagram

<!-- image -->

Table 51-24: CSPFT\_CNTVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Current Counter Value. The CSPFT_CNTVR[n].VALUE bit field specifies the current value of the corre- sponding counter. |

## Main Control Register

The CSPFT\_CTL register controls general operation of the PTM, such as whether tracing is enabled.

Figure 51-20: CSPFT\_CTL Register Diagram

<!-- image -->

Table 51-25: CSPFT\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | RSENA      | Return Stack Enable. When the CSPFT_CTL.RSENA bit is set, the first indirect branch back to an address generates a branch without exception packet, and subsequent branches back to the same address generate E Atoms. This compresses inner loops of HWloops and code that indirectly branches back. |
| 15:14 (R/W)        | CIDSZ      | Context ID Size. The CSPFT_CTL.CIDSZ bit field specifies the byte size to trace. Only the bytes specified are traced, even if the new Context ID value is larger than this.                                                                                                                           |
|                    |            | 0 No Context ID Tracing                                                                                                                                                                                                                                                                               |
|                    |            | 1 One byte Traced                                                                                                                                                                                                                                                                                     |
|                    |            | 2 Two Bytes Traced                                                                                                                                                                                                                                                                                    |
|                    |            | 3 Three Bytes Traced                                                                                                                                                                                                                                                                                  |

Table 51-25: CSPFT\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | PB         | Programming Bit. To program the CSPFT, use the following procedure. 1. Set the CSPFT_CTL.PB bit to disable all trace functionality. 2. Poll the CSPFT_STAT.PB bit waiting for it to be 1 (FIFO drained, trace halted). 3. Program the trace registers, counter and other registers, as required. 4. Set this bit to 0. 5. Poll the CSPFT_STAT.PB bit until it reads 0 (trace status reset, trace restarted). When the CSPFT_CTL.PB bit is set, the FIFO is drained and no more trace is pro- duced. All counters are held in their present state and the external outputs are forced low. After the FIFO is drained, the CSPFT_STAT.PB is set to reflect that the part is ready to program. When this bit is cleared, the trace status is cleared and trace is restarted. 0 Trace Enabled |
| 8 (R/W)            | BBRAN      | Branch Broadcast. Set the CSPFT_CTL.BBRAN bit to 1 to enable branch broadcasting. Branch broad- casting traces the address of direct branch instructions rather than producing E atoms.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Device Type Identifier Register

The CSPFT\_DEVTYPE register is read-only. It provides a debugger with information about the component when the part number field is not recognized. The debugger can then report this information.

Figure 51-21: CSPFT\_DEVTYPE Register Diagram

<!-- image -->

Table 51-26: CSPFT\_DEVTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 7:4                | STYPE      | Sub Type = DSP.             |
| 3:0 (R/NW)         | TYPE       | Device Type = Trace Source. |

## External Output Event Register

The CSPFT\_EXTOUTEVR[n] register defines the event that controls the corresponding EXTOUT external output signal.

Figure 51-22: CSPFT\_EXTOUTEVR[n] Register Diagram

<!-- image -->

Table 51-27: CSPFT\_EXTOUTEVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:14 (R/W)        | FUNC       | Function. The CSPFT_EXTOUTEVR[n].FUNC bit field specifies the logical operation that combines the two resources that define the event.                                                                                                       | Function. The CSPFT_EXTOUTEVR[n].FUNC bit field specifies the logical operation that combines the two resources that define the event.                                                                                                       |
| 16:14 (R/W)        | FUNC       | 0                                                                                                                                                                                                                                            | A                                                                                                                                                                                                                                            |
| 16:14 (R/W)        | FUNC       | 1                                                                                                                                                                                                                                            | NOT(A)                                                                                                                                                                                                                                       |
| 16:14 (R/W)        | FUNC       | 2                                                                                                                                                                                                                                            | A AND B                                                                                                                                                                                                                                      |
| 16:14 (R/W)        | FUNC       | 3                                                                                                                                                                                                                                            | NOT(A) AND B                                                                                                                                                                                                                                 |
| 16:14 (R/W)        | FUNC       | 4                                                                                                                                                                                                                                            | NOT(A) AND NOT(B)                                                                                                                                                                                                                            |
| 16:14 (R/W)        | FUNC       | 5                                                                                                                                                                                                                                            | A OR B                                                                                                                                                                                                                                       |
| 16:14 (R/W)        | FUNC       | 6                                                                                                                                                                                                                                            | NOT(A) OR B                                                                                                                                                                                                                                  |
| 16:14 (R/W)        | FUNC       | 7                                                                                                                                                                                                                                            | NOT(A) OR NOT(B)                                                                                                                                                                                                                             |
| 13:7 (R/W)         | RESB       | Resource B. The CSPFT_EXTOUTEVR[n].RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_EXTOUTEVR[n].FUNC field (See CSPFT_EXTOUTEVR[n].RESA for list of possible values). | Resource B. The CSPFT_EXTOUTEVR[n].RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_EXTOUTEVR[n].FUNC field (See CSPFT_EXTOUTEVR[n].RESA for list of possible values). |

Table 51-27: CSPFT\_EXTOUTEVR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | RESA       | Resource A. The CSPFT_EXTOUTEVR[n].RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_EXTOUTEVR[n].FUNC field. 0 Single Addr Comparator 0 1 Single Addr Comparator 1 2 Single Addr Comparator 2 3 Single Addr Comparator 3 4 Single Addr Comparator 4 5 Single Addr Comparator 5 6 Single Addr Comparator 6 7 Single Addr Comparator 7 8 Single Addr Comparator 8 9 Single Addr Comparator 9 10 Single Addr Comparator 10 11 Single Addr Comparator 11 12 Single Addr Comparator 12 13 Single Addr Comparator 13 14 Single Addr Comparator 14 15 Single Addr Comparator 15 16 Addr Range Comparator 0 17 Addr Range Comparator 1 18 Addr Range Comparator 2 19 Addr Range Comparator 3 20 Addr Range Comparator 4 21 Addr Range Comparator 5 22 Addr Range Comparator 6 23 Addr Range Comparator 7 |

Table 51-27: CSPFT\_EXTOUTEVR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration |                                        |
|--------------------|------------|---------------------------|----------------------------------------|
|                    |            |                        67 | Counter 3 at Zero                      |
|                    |            |                        88 | Context ID Comparator 0                |
|                    |            |                        89 | Context ID Comparator 1                |
|                    |            |                        90 | Context ID Comparator 2                |
|                    |            |                        95 | TraceEnable Start/Stop Resource 0 or 1 |
|                    |            |                        96 | External Inputs 0                      |
|                    |            |                        97 | External Inputs 1                      |
|                    |            |                        98 | External Inputs 2                      |
|                    |            |                        99 | External Inputs 3                      |
|                    |            |                       110 | Trace Prohibited                       |
|                    |            |                       111 | Always TRUE                            |

## Hardware Feature Register

The CSPFT\_HWFEAT register enables software to read the implementation defined configuration of the PTM, giving the number of each type of hardware resource. Each field indicates the number of instances of a particular resource, zero indicates that there are no implemented resources of that type.

Figure 51-23: CSPFT\_HWFEAT Register Diagram

<!-- image -->

Table 51-28: CSPFT\_HWFEAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/NW)       | NCIDC      | Number of Context ID Comparators. The CSPFT_HWFEAT.NCIDC bit field identifies the number of context ID compa- rators.                                                                                                                                                                                                                                                                                                                                               |
| 22:20 (R/NW)       | NEXO       | Number of External Outputs. The CSPFT_HWFEAT.NEXO bit field identifies the number of external outputs (up to four).                                                                                                                                                                                                                                                                                                                                                 |
| 19:17 (R/NW)       | NEXI       | Number of External Inputs. The CSPFT_HWFEAT.NEXI bit field identifies the number of external inputs (up to four).                                                                                                                                                                                                                                                                                                                                                   |
| 15:13 (R/NW)       | NCNTR      | Number of Counters. The CSPFT_HWFEAT.NCNTR bit field identifies the number of counters (up to four) that are configured using the counter registers.                                                                                                                                                                                                                                                                                                                |
| 3:0 (R/NW)         | NACMP      | Number of Pairs of Address Comparators. The CSPFT_HWFEAT.NACMP bit field identifies the number of pairs of address comparators as address range comparators (ARCs). In this case, two adjacent address comparators form the ARC, so you can use address comparators 1 and 2 to define the first ARC. An ARC matches when any instruction in the specified range is committed for execu- tion, regardless of whether the instruction passes its condition code test. |

Table 51-28: CSPFT\_HWFEAT Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|-----------|------------|---------------------------|--------------------------------|
| (Access)  |            |                           |                                |
|           |            |                         1 | 1 Pair of Address Comparators  |
|           |            |                         2 | 2 Pairs of Address Comparators |
|           |            |                         3 | 3 Pairs of Address Comparators |
|           |            |                         4 | 4 Pairs of Address Comparators |
|           |            |                         5 | 5 Pairs of Address Comparators |
|           |            |                         6 | 6 Pairs of Address Comparators |
|           |            |                         7 | 7 Pairs of Address Comparators |
|           |            |                         8 | 8 Pairs of Address Comparators |

## Lock Access Register

The CSPFT\_LAR register is used to provide lock and unlock access to all other CSPFT registers.

Figure 51-24: CSPFT\_LAR Register Diagram

<!-- image -->

Table 51-29: CSPFT\_LAR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 31:0 (RX/W)        | VALUE      | Lock Access. Write 0xC5ACCE55 to the CSPFT_LAR.VALUE bit field to unlock. Write any oth- er value to lock. |

## Lock Status Register

The CSPFT\_LSR register is used to detect if the lock registers are implemented and if they are currently locked.

Figure 51-25: CSPFT\_LSR Register Diagram

<!-- image -->

Table 51-30: CSPFT\_LSR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | LOCKED     | Lock Status. The CSPFT_LSR.LOCKED bit indicates whether the PFT is locked. 0 Writes are permitted 1 Locked. Writes are ignored                                                                                                                                              |
| 0 (R/NW)           | LOCKEN     | Locking Supported. The CSPFT_LSR.LOCKEN bit indicates whether the lock registers are implemented for this interface. 0 Locking is Not Required. This access is from an inter- face that ignores the lock registers. 1 Locking is Required. This access is from an interface |

## Peripheral ID0 Register

The CSPFT\_PID0 register holds peripheral identification information.

Figure 51-26: CSPFT\_PID0 Register Diagram

<!-- image -->

Table 51-31: CSPFT\_PID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 7:0                | PARTNUM    | Part Number.                                                                 |
| (R/NW)             |            | The CSPFT_PID0.PARTNUM bit field holds the peripheral identification number. |

## Peripheral ID1 Register

The CSPFT\_PID1 register holds peripheral identification information.

Figure 51-27: CSPFT\_PID1 Register Diagram

<!-- image -->

Table 51-32: CSPFT\_PID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 7:4                | JEP106     | JEDEC JEP106 Manufacturer ID code.                                           |
| (R/NW)             |            |                                                                              |
| 3:0                | PARTNUM    | Part Number.                                                                 |
| (R/NW)             |            | The CSPFT_PID1.PARTNUM bit field holds the peripheral identification number. |

## Peripheral ID2 Register

The CSPFT\_PID2 register holds peripheral identification information.

Figure 51-28: CSPFT\_PID2 Register Diagram

<!-- image -->

Table 51-33: CSPFT\_PID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | REV        | Peripheral Revision.                                                                                        |
| 3 (R/NW)           | JEDECASGN  | A JEDEC Assigned Value is Used. The CSPFT_PID2.JEDECASGN bit indicates that a JEDEC assigned value is used. |
| 2:0 (R/NW)         | JEP106     | JEDEC JEP106 Manufacturer ID code.                                                                          |

## Peripheral ID3 Register

The CSPFT\_PID3 register holds peripheral identification information.

Figure 51-29: CSPFT\_PID3 Register Diagram

<!-- image -->

Table 51-34: CSPFT\_PID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration           |
|--------------------|------------|-----------------------------------|
| 7:4 (R/NW)         | REVAND     | Field to mark metal fix revision. |
| 3:0 (R/NW)         | CUSTMOD    | Customer Modified.                |

## Peripheral ID4 Register

The CSPFT\_PID4 register holds peripheral identification information.

Figure 51-30: CSPFT\_PID4 Register Diagram

<!-- image -->

Table 51-35: CSPFT\_PID4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | SIZE       | Number of 4K Blocks. The CSPFT_PID4.SIZE bit field contains the size of the component in 4K chunks minus 1 (for example 0=4K). |
| 3:0 (R/NW)         | JEOP106CC  | JEOP106 continuation code (number of leading 0x7Fs).                                                                           |

## Status Register

The CSPFT\_STAT register provides information about the current status of the trace and trigger logic.

Figure 51-31: CSPFT\_STAT Register Diagram

<!-- image -->

Table 51-36: CSPFT\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | TRIG       | Trigger Bit. The CSPFT_STAT.TRIG bit is set when the trigger occurs, and prevents the trigger from being output until the CSPFT is next programmed. This bit is reset when the CSPFT_CTL.PB bit transitions from 1 to 0.                                                                                               |
| 2 (R/W)            | TSS        | Trace Start/Stop Bit Status. The CSPFT_STAT.TSS bit holds the current status of the trace start/stop resource. If = 1, it indicates that a trace start address has been matched, without a corresponding trace stop address match. This bit =0 when trace is restarted (the CSPFT_CTL.PB bit transitions from 1 to 0). |
| 1 (R/NW)           | PB         | Prog Bit Status. The CSPFT_STAT.PB bit indicates the current effective value of the CSPFT_CTL.PB bit. The program must wait for this bit to =1 before programming the CSPFT. (See the CSPFT_CTL.PB bit description).                                                                                                   |
| 0 (R/NW)           | OF         | Overflow. If the CSPFT_STAT.OF bit is =1, there is an overflow. This bit is cleared =0 when the trace is restarted ( CSPFT_CTL.PB transitions from 1 to 0)                                                                                                                                                             |

## Synchronization Frequency Register

The CSPFT\_SYNCFR register holds the trace synchronization frequency value.

Figure 51-32: CSPFT\_SYNCFR Register Diagram

<!-- image -->

Table 51-37: CSPFT\_SYNCFR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | SFREQ      | Synchronization frequency. The CSPFT_SYNCFR.SFREQ bit field is the number of 128-byte blocks of trace da- ta after which you want to drop an address synchronization packet. If the circular buf- fer size is 16k, ensure that there are a few A-syncs in the buffer, so setting this to 16 means that every 2k there is an A-Sync packet. |

## CoreSight Trace ID Register

The CSPFT\_TRACEIDR register defines the 7-bit trace ID, for output to the trace bus.

Figure 51-33: CSPFT\_TRACEIDR Register Diagram

<!-- image -->

Table 51-38: CSPFT\_TRACEIDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | TID        | Trace ID. Defines the 7-bit trace ID, for output to the trace bus. Used in systems where multiple trace sources are present and tracing simultaneously. For example, when outputs trace onto the AMBA 3 Advanced Trace Bus, a unique ID is required for each trace source. |

## ADSP-SC59x TAPC Register Descriptions

TAPC (TAPC) contains the following registers.

Table 51-39: ADSP-SC59x TAPC Register List

| Name              | Description                       |
|-------------------|-----------------------------------|
| TAPC_DBGCTL       | Debug Control Register            |
| TAPC_IDCODE       | IDCODE Register                   |
| TAPC_SDBGKEY0     | Secure Debug Key 0 Register       |
| TAPC_SDBGKEY1     | Secure Debug Key 1 Register       |
| TAPC_SDBGKEY2     | Secure Debug Key 2 Register       |
| TAPC_SDBGKEY3     | Secure Debug Key 3 Register       |
| TAPC_SDBGKEY_CTL  | Secure Debug Key Control Register |
| TAPC_SDBGKEY_STAT | Secure Debug Key Status Register  |
| TAPC_USERCODE     | USERCODE Register                 |

## Debug Control Register

The TAPC\_DBGCTL register creates authentication signals to all CoreSight components in the system such as DBGEN, NIDEN, SPNIDEN and SPIDEN.

Figure 51-34: TAPC\_DBGCTL Register Diagram

<!-- image -->

Table 51-40: TAPC\_DBGCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration             |
|--------------------|-------------|-------------------------------------|
| 15 (R/W)           | SPIDENTRACE | SPIDEN for Coresight Trace Modules. |
| 14 (R/W)           | NIDENTRACE  | NIDEN for Coresight Trace Modules.  |
| 13 (R/W)           | DBGENTRACE  | DBGEN for Coresight Trace Modules.  |
| 12 (R/W)           | NIDENCTISYS | NIDEN for System CTI.               |
| 11 (R/W)           | DBGENCTISYS | DBGEN for System CTI.               |

Table 51-40: TAPC\_DBGCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 10 (R/W)           | SPIDENSTM  | SPIDEN for STM.           |
| 9 (R/W)            | SPNIDENSTM | SPNIDEN for STM.          |
| 8 (R/W)            | NIDENSTM   | NIDEN for STM.            |
| 7 (R/W)            | DBGENSTM   | DBGEN for STM.            |
| 6 (R/W)            | DBGENC2    | DBGEN for Core 2.         |
| 5 (R/W)            | DBGENC1    | DBGEN for Core 1.         |
| 4 (R/W)            | SPIDENC0   | SPIDEN for Core 0.        |
| 3 (R/W)            | SPNIDENC0  | SPNIDEN for Core 0.       |
| 2 (R/W)            | NIDENC0    | NIDEN for Core 0.         |
| 1 (R/W)            | DBGENC0    | DBGEN for Core 0.         |
| 0 (R/W)            | SPIDENDAP  | SPIDEN for DAP.           |

## IDCODE Register

The TAPC\_IDCODE register holds the IDCODE. The bit field is defined as follows.

IDCODE[31:28] = 0x1* - REVID

IDCODE[27:12] = 0x280B - JTAG ID

IDCODE[11:1] = 0x65 - Manufacturer ID

IDCODE[0] = 0x1

Figure 51-35: TAPC\_IDCODE Register Diagram

<!-- image -->

Table 51-41: TAPC\_IDCODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 31:28 (R/NW)       | REVID      | Silicon Revision. The TAPC_IDCODE.REVID bit field holds the silicon revision. See the processor anomaly list for details. |
| 27:12 (R/NW)       | JTAGID     | JTAG ID.                                                                                                                  |
| 11:1 (R/NW)        | MNFID      | Manufacturer ID.                                                                                                          |
| 0 (R/NW)           | LSB        | IDCODE LSB.                                                                                                               |

## Secure Debug Key 0 Register

The TAPC\_SDBGKEY0 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. A debug key of 128 bits needs to be written into the Secure Debug Key registers ( TAPC\_SDBGKEY0 , TAPC\_SDBGKEY1 , TAPC\_SDBGKEY2 , TAPC\_SDBGKEY3 ) in the TAPC through the peripheral bus interface.

These registers hold the value of the key against which a matching key provided by the debug user is compared to enable a debug session. The task of writing these registers is performed (initially) by boot ROM code which copies a customer-selected key from the Flash memory info block to these registers.

An SDBGKEY value of all 0's is always an invalid key, a value of all 1's match the default value of the Secure Debug Key Compare registers and requires no entry in these registers. It is recommended programs have a significant number of 0's and 1's in a pseudo-random pattern throughout the 128-bit code for maximum protection.

Figure 51-36: TAPC\_SDBGKEY0 Register Diagram

<!-- image -->

Table 51-42: TAPC\_SDBGKEY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | SDBGKEY. The TAPC_SDBGKEY0.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |

## Secure Debug Key 1 Register

The TAPC\_SDBGKEY1 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. See the TAPC\_SDBGKEY0 register description for more information.

Figure 51-37: TAPC\_SDBGKEY1 Register Diagram

<!-- image -->

Table 51-43: TAPC\_SDBGKEY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | SDBGKEY. The TAPC_SDBGKEY1.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |
| (R/W)              |            |                                                                                                                                                                      |

## Secure Debug Key 2 Register

The TAPC\_SDBGKEY2 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. See the TAPC\_SDBGKEY0 register description for more information.

Figure 51-38: TAPC\_SDBGKEY2 Register Diagram

<!-- image -->

Table 51-44: TAPC\_SDBGKEY2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | SDBGKEY. The TAPC_SDBGKEY2.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |
| (R/W)              |            |                                                                                                                                                                      |

## Secure Debug Key 3 Register

The TAPC\_SDBGKEY3 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. See the TAPC\_SDBGKEY0 register description for more information.

Figure 51-39: TAPC\_SDBGKEY3 Register Diagram

<!-- image -->

Table 51-45: TAPC\_SDBGKEY3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | SDBGKEY. The TAPC_SDBGKEY3.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |
| (R/W)              |            |                                                                                                                                                                      |

## Secure Debug Key Control Register

The TAPC\_SDBGKEY\_CTL register contains a bit to configure configure validity for the secure debug keys.

Figure 51-40: TAPC\_SDBGKEY\_CTL Register Diagram

<!-- image -->

Table 51-46: TAPC\_SDBGKEY\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 0                  | VALID      | SDBGKEY Valid bit.        |
| (R/W)              |            |                           |

## Secure Debug Key Status Register

The TAPC\_SDBGKEY\_STAT register contains the PASS/FAIL status of the secure debug key match.

Figure 51-41: TAPC\_SDBGKEY\_STAT Register Diagram

<!-- image -->

Table 51-47: TAPC\_SDBGKEY\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/NW)           | FAIL       | SDBGKEY MATCH FAILED.     |
| 0 (R/NW)           | PASS       | SDBGKEY MATCH PASSED.     |

## USERCODE Register

The TAPC\_USERCODE register stores the 32-bit USERCODE value for the SoC debug unit.

Figure 51-42: TAPC\_USERCODE Register Diagram

<!-- image -->

Table 51-48: TAPC\_USERCODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | USERCODE[31:0].           |
| (R/NW)             |            |                           |

## ADSP-SC59x CSCTI Register Descriptions

CSCTI\_Register\_Definitions (CSCTI) contains the following registers.

Table 51-49: ADSP-SC59x CSCTI Register List

| Name              | Description                            |
|-------------------|----------------------------------------|
| CSCTI_ASICCTL     | External Multiplexor Control Register  |
| CSCTI_AUTHSTATUS  | Authentication Status                  |
| CSCTI_CLAIMCLR    | Claim Tag Clear Register               |
| CSCTI_CLAIMSET    | Claim Tag Set Register                 |
| CSCTI_COMPID0     | Component ID0                          |
| CSCTI_COMPID1     | Component ID1                          |
| CSCTI_COMPID2     | Component ID2                          |
| CSCTI_COMPID3     | Component ID3                          |
| CSCTI_CTIAPPCLEAR | CTI Application Trigger Clear Register |
| CSCTI_CTIAPPPULSE | CTI Application Pulse Register         |
| CSCTI_CTIAPPSET   | CTI Application Trigger Set Register   |

Table 51-49: ADSP-SC59x CSCTI Register List (Continued)

| Name                   | Description                              |
|------------------------|------------------------------------------|
| CSCTI_CTICHINSTATUS    | CTI Channel In Status Register           |
| CSCTI_CTICHOUTSTATUS   | CTI Channel Out Status Register          |
| CSCTI_CTICONTROL       | CTI Control Register                     |
| CSCTI_CTIGATE          | Enable CTI Channel Gate Register         |
| CSCTI_CTIINEN0         | CTI Trigger 0 to Channel Enable Register |
| CSCTI_CTIINEN1         | CTI Trigger 1 to Channel Enable Register |
| CSCTI_CTIINEN2         | CTI Trigger 2 to Channel Enable Register |
| CSCTI_CTIINEN3         | CTI Trigger 3 to Channel Enable Register |
| CSCTI_CTIINEN4         | CTI Trigger 4 to Channel Enable Register |
| CSCTI_CTIINEN5         | CTI Trigger 5 to Channel Enable Register |
| CSCTI_CTIINEN6         | CTI Trigger 6 to Channel Enable Register |
| CSCTI_CTIINEN7         | CTI Trigger 7 to Channel Enable Register |
| CSCTI_CTIINTACK        | CTI Interrupt Acknowledge Register       |
| CSCTI_CTIOUTEN0        | CTI Channel to Trigger 0 Enable Register |
| CSCTI_CTIOUTEN1        | CTI Channel to Trigger 1 Enable Register |
| CSCTI_CTIOUTEN2        | CTI Channel to Trigger 2 Enable Register |
| CSCTI_CTIOUTEN3        | CTI Channel to Trigger 3 Enable Register |
| CSCTI_CTIOUTEN4        | CTI Channel to Trigger 4 Enable Register |
| CSCTI_CTIOUTEN5        | CTI Channel to Trigger 5 Enable Register |
| CSCTI_CTIOUTEN6        | CTI Channel to Trigger 6 Enable Register |
| CSCTI_CTIOUTEN7        | CTI Channel to Trigger 7 Enable Register |
| CSCTI_CTITRIGINSTATUS  | CTI Trigger In Status Register           |
| CSCTI_CTITRIGOUTSTATUS | CTI Trigger Out Status Register          |
| CSCTI_DEVID            | Device ID                                |
| CSCTI_DEVTYPE          | Device Type                              |
| CSCTI_ITCHIN           | Integration Test Channel Input           |
| CSCTI_ITCHINACK        | ITCHINACK                                |
| CSCTI_ITCHOUT          | Integration Test Channel Output          |
| CSCTI_ITCHOUTACK       | ITCHOUTACK                               |
| CSCTI_ITCTRL           | Integration Mode Control Register        |
| CSCTI_ITTRIGIN         | Integration Test Trigger Input           |

Table 51-49: ADSP-SC59x CSCTI Register List (Continued)

| Name               | Description                     |
|--------------------|---------------------------------|
| CSCTI_ITTRIGINACK  | ITTRIGINACK                     |
| CSCTI_ITTRIGOUT    | Integration Test Trigger Output |
| CSCTI_ITTRIGOUTACK | ITTRIGOUTACK                    |
| CSCTI_LAR          | Lock Access Register            |
| CSCTI_LSR          | Lock Status Register            |
| CSCTI_PERIPHID0    | Peripheral ID0                  |
| CSCTI_PERIPHID1    | Peripheral ID1                  |
| CSCTI_PERIPHID2    | Peripheral ID2                  |
| CSCTI_PERIPHID3    | Peripheral ID3                  |
| CSCTI_PERIPHID4    | Peripheral ID4                  |
| CSCTI_PERIPHID5    | Peripheral ID5                  |
| CSCTI_PERIPHID6    | Peripheral ID6                  |
| CSCTI_PERIPHID7    | Peripheral ID7                  |

## External Multiplexor Control Register

The CSCTI\_ASICCTL register is for I/O port control.

Figure 51-43: CSCTI\_ASICCTL Register Diagram

<!-- image -->

Table 51-50: CSCTI\_ASICCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | ASICCTL    | ASIC Control. Set and clear external output signal. 0: Clear output bit to 0. 1: Set output bit to 1. |

## Authentication Status

The CSCTI\_AUTHSTATUS register reports the current status of the authentication control signals.

Figure 51-44: CSCTI\_AUTHSTATUS Register Diagram

<!-- image -->

Table 51-51: CSCTI\_AUTHSTATUS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 3 (R/NW)           | NDBG_EN    | Non-invasive Debug Enabled Status. |
| 2 (R/NW)           | NDBG_CTL   | Non-invasive Debug Controlled.     |
| 1 (R/NW)           | IDBG_EN    | Invasive Debug Enable Status.      |
| 0 (R/NW)           | IDBG_CTL   | Invasive Debug Controlled.         |

## Claim Tag Clear Register

The CSCTI\_CLAIMCLR register forms one half of the claim tag value. On writes, this location enables individual bits to be cleared. On reads, it returns the current claim tag value.

Figure 51-45: CSCTI\_CLAIMCLR Register Diagram

<!-- image -->

Table 51-52: CSCTI\_CLAIMCLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W1C)        | CLAIMCLR   | Claim Tag Clear Register Bits. A bit-programmable register bank that clears the claim tag value. It is zero at reset. It is used by software agents to signal to each other ownership of the hardware. It has no direct effect on the hardware itself. |

## Claim Tag Set Register

The CSCTI\_CLAIMSET register forms one half of the claim tag value. On writes, this location enables individual bits to be set. On reads, it returns the number of bits that can be set.

Figure 51-46: CSCTI\_CLAIMSET Register Diagram

<!-- image -->

Table 51-53: CSCTI\_CLAIMSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 3:0                | CLAIMSET   | Claim Tag Set Register Bits.                                                                                            |
| (R1/W)             |            | A bit-programmable register bank that sets the claim tag value. A read returns a logic 1 for all implemented locations. |

## Component ID0

The CSCTI\_COMPID0 register is part of the set of component identification registers.

Figure 51-47: CSCTI\_COMPID0 Register Diagram

<!-- image -->

Table 51-54: CSCTI\_COMPID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | ID0        | Component ID0 Field.      |
| (R/NW)             |            | Preamble. Returns 0x0D.   |

## Component ID1

The CSCTI\_COMPID1 register is part of the set of component identification registers.

Figure 51-48: CSCTI\_COMPID1 Register Diagram

<!-- image -->

Table 51-55: CSCTI\_COMPID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | ID1        | Component ID1 Field. Bits[7:4] Component class. Returns 0x9, indicating this is a CoreSight component. Bits[3:0] Preamble. Returns 0x0. |

## Component ID2

The CSCTI\_COMPID2 register is part of the set of component identification registers.

Figure 51-49: CSCTI\_COMPID2 Register Diagram

<!-- image -->

Table 51-56: CSCTI\_COMPID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | ID2        | Component ID2 Field.      |
| (R/NW)             |            | Preamble. Returns 0x05.   |

## Component ID3

The CSCTI\_COMPID3 register is part of the set of component identification registers.

Figure 51-50: CSCTI\_COMPID3 Register Diagram

<!-- image -->

Table 51-57: CSCTI\_COMPID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | ID3        | Component ID3 Field.      |
| (R/NW)             |            | Preamble. Returns 0xB1.   |

## CTI Application Trigger Clear Register

The CSCTI\_CTIAPPCLEAR register allows software to clear any channel output. Software can use this register to clear a channel event in place of a hardware source on a trigger input. This register must not be used in a system where all events are sent as single-cycle pulses. It is only retained for compatibility with older systems and software. The register is implemented before the CTIGATE register. Therefore, for the channel event to propagate outside the CTI, the corresponding CTIGATE bit must be set to 1.

Figure 51-51: CSCTI\_CTIAPPCLEAR Register Diagram

<!-- image -->

Table 51-58: CSCTI\_CTIAPPCLEAR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R0/W)         | APPCLEAR   | Channel Clear. The CSCTI_CTIAPPCLEAR.APPCLEAR bit is for clearing the corresponding in- ternal channel flag. 0: No effect. 1: Clears the channel output. |

## CTI Application Pulse Register

The CSCTI\_CTIAPPPULSE register allows software to pulse any channel output. This register can be used by software to pulse a channel event in place of a hardware source on a trigger input. The register is implemented before the CSCTI\_CTIGATE register so, for the channel event to propagate outside the CTI, it is necessary for the corresponding bit to be 1.

Figure 51-52: CSCTI\_CTIAPPPULSE Register Diagram

<!-- image -->

Table 51-59: CSCTI\_CTIAPPPULSE Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                  |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------|
| 3:0 (R0/W)         | BIT_0APPULSE | Channel Event Pulse. Pulses the channel outputs. 0: No effect. 1: Pulse channel event for one clk cycle. |

## CTI Application Trigger Set Register

The CSCTI\_CTIAPPSET register allows software to set any channel output. Software can use this register to generate a channel event in place of a hardware source on a trigger input. This register must not be used in a system where all events are sent as single-cycle pulses. It is only retained for compatibility with older systems and software. The register is implemented before the CSCTI\_CTIGATE register. Therefore, for the channel event to propagate outside the CTI, the corresponding CTIGATE bit must be set to 1.

Figure 51-53: CSCTI\_CTIAPPSET Register Diagram

<!-- image -->

Table 51-60: CSCTI\_CTIAPPSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | APPSET     | Set Channel Event. Sets the corresponding internal channel flag: 0: Read: application channel is inactive. Write: has no effect. 1: Read: application channel is active. Write: sets the channel output. |

## CTI Channel In Status Register

If the channel input is driven by a source that generates single cycle pulses, the CSCTI\_CTICHINSTATUS register is generally read as 0.

Figure 51-54: CSCTI\_CTICHINSTATUS Register Diagram

<!-- image -->

Table 51-61: CSCTI\_CTICHINSTATUS Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/NW)         | CTCHINSTATUS | Channel Input Status. Channel input status. 0: One bit per channel input. 0 means that the input is LOW. 1: One bit per channel input. 1 means that the input is HIGH. |

## CTI Channel Out Status Register

The CSCTI\_CTICHOUTSTATUS register only has meaning if the trigger source drives static levels, rather than pulses.

Figure 51-55: CSCTI\_CTICHOUTSTATUS Register Diagram

<!-- image -->

Table 51-62: CSCTI\_CTICHOUTSTATUS Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                   |
|--------------------|---------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/NW)         | CTCHOUTSTATUS | Channel Out Status. Channel output status. 0: One bit per channel output. 0 means that the output is LOW. 1: One bit per channel output. 1 means that the output is HIGH. |

## CTI Control Register

The CSCTI\_CTICONTROL register enables and disables the CTI.

Figure 51-56: CSCTI\_CTICONTROL Register Diagram

<!-- image -->

Table 51-63: CSCTI\_CTICONTROL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 0                  | GLBEN      | Global CTI Enable.        |
| (R/W)              | GLBEN      | 0 Disable                 |
| (R/W)              | GLBEN      | 1 Enable                  |

## Enable CTI Channel Gate Register

The CSCTI\_CTIGATE register enables the propagation of channel events out of the CTI, one bit per channel.

Figure 51-57: CSCTI\_CTIGATE Register Diagram

<!-- image -->

Table 51-64: CSCTI\_CTIGATE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------|
| 3 (R/W)            | CTIGATEEN3 | Enable CTICHOUT3. 0: Disable channel 3 from propagating. 1: Enable channel 3 propagation. |
| 2 (R/W)            | CTIGATEEN2 | Enable CTICHOUT2. 0: Disable channel 2 from propagating. 1: Enable channel 2 propagation. |
| 1 (R/W)            | CTIGATEEN1 | Enable CTICHOUT1. 0: Disable channel 1 from propagating. 1: Enable channel 1 propagation. |
| 0 (R/W)            | CTIGATEEN0 | Enable CTICHOUT0. 0: Disable channel 0 from propagating. 1: Enable channel 0 propagation. |

## CTI Trigger 0 to Channel Enable Register

The CSCTI\_CTIINEN0 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

Figure 51-58: CSCTI\_CTIINEN0 Register Diagram

<!-- image -->

Table 51-65: CSCTI\_CTIINEN0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 0 events are ignored by the corresponding channel. 1: When an event is received on event_in[0], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 1 to Channel Enable Register

The CSCTI\_CTIINEN1 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-59: CSCTI\_CTIINEN1 Register Diagram

<!-- image -->

Table 51-66: CSCTI\_CTIINEN1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 1 events are ignored by the corresponding channel. 1: When an event is received on event_in[1], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 2 to Channel Enable Register

The CSCTI\_CTIINEN2 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-60: CSCTI\_CTIINEN2 Register Diagram

<!-- image -->

Table 51-67: CSCTI\_CTIINEN2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 2 events are ignored by the corresponding channel. 1: When an event is received on event_in[2], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 3 to Channel Enable Register

The CSCTI\_CTIINEN3 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-61: CSCTI\_CTIINEN3 Register Diagram

<!-- image -->

Table 51-68: CSCTI\_CTIINEN3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 3 events are ignored by the corresponding channel. 1: When an event is received on event_in[3], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 4 to Channel Enable Register

The CSCTI\_CTIINEN4 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-62: CSCTI\_CTIINEN4 Register Diagram

<!-- image -->

Table 51-69: CSCTI\_CTIINEN4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 4 events are ignored by the corresponding channel. 1: When an event is received on event_in[4], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 5 to Channel Enable Register

The CSCTI\_CTIINEN5 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-63: CSCTI\_CTIINEN5 Register Diagram

<!-- image -->

Table 51-70: CSCTI\_CTIINEN5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 5 events are ignored by the corresponding channel. 1: When an event is received on event_in[5], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 6 to Channel Enable Register

The CSCTI\_CTIINEN6 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-64: CSCTI\_CTIINEN6 Register Diagram

<!-- image -->

Table 51-71: CSCTI\_CTIINEN6 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 6 events are ignored by the corresponding channel. 1: When an event is received on event_in[6], generate an event on the channel corre- sponding to this bit. |

## CTI Trigger 7 to Channel Enable Register

The CSCTI\_CTIINEN7 register maps trigger inputs to channels in the cross trigger system. The CTIINEN registers are bit maps that allow the trigger input to be mapped to any channel output, including none (0x0) and all (0xF). There is one register per trigger input, so it is possible to map any combination of trigger inputs to any channel outputs.

CTIINEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-65: CSCTI\_CTIINEN7 Register Diagram

<!-- image -->

Table 51-72: CSCTI\_CTIINEN7 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGINEN   | Channel Trigger Input Enable. Trigger input to channel mapping. 0: Input trigger 7 events are ignored by the corresponding channel. 1: When an event is received on event_in[7], generate an event on the channel corre- sponding to this bit. |

## CTI Interrupt Acknowledge Register

The CSCTI\_CTIINTACK register acknowledges trigger outputs.

It is a bit map that allows selective clearing of trigger output events. If the SW\_HANDSHAKE parameter for a trigger output is set, indicating that the output latches HIGH when an event is sent to that output, then the output remains HIGH until the corresponding INTACK bit is written to a 1. A write of a bit to 0 has no effect. This allows different software threads to be responsible for clearing different trigger outputs without needing to perform a readmodify-write operation to find which bits are set.

Figure 51-66: CSCTI\_CTIINTACK Register Diagram

<!-- image -->

Table 51-73: CSCTI\_CTIINTACK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R0/W)         | INTACK     | Interrupt Acknowledge. The CSCTI_CTIINTACK.INTACK bit field acknowledges the corresponding event output. It is write only. |

## CTI Channel to Trigger 0 Enable Register

The CSCTI\_CTIOUTEN0 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

Figure 51-67: CSCTI\_CTIOUTEN0 Register Diagram

<!-- image -->

Table 51-74: CSCTI\_CTIOUTEN0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger0. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[0]. |

## CTI Channel to Trigger 1 Enable Register

The CSCTI\_CTIOUTEN1 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-68: CSCTI\_CTIOUTEN1 Register Diagram

<!-- image -->

Table 51-75: CSCTI\_CTIOUTEN1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger1. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[1]. |

## CTI Channel to Trigger 2 Enable Register

The CSCTI\_CTIOUTEN2 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-69: CSCTI\_CTIOUTEN2 Register Diagram

<!-- image -->

Table 51-76: CSCTI\_CTIOUTEN2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger2. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[2]. |

## CTI Channel to Trigger 3 Enable Register

The CSCTI\_CTIOUTEN3 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-70: CSCTI\_CTIOUTEN3 Register Diagram

<!-- image -->

Table 51-77: CSCTI\_CTIOUTEN3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger3. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[3]. |

## CTI Channel to Trigger 4 Enable Register

The CSCTI\_CTIOUTEN4 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-71: CSCTI\_CTIOUTEN4 Register Diagram

<!-- image -->

Table 51-78: CSCTI\_CTIOUTEN4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger4. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[4]. |

## CTI Channel to Trigger 5 Enable Register

The CSCTI\_CTIOUTEN5 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-72: CSCTI\_CTIOUTEN5 Register Diagram

<!-- image -->

Table 51-79: CSCTI\_CTIOUTEN5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger5. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[5]. |

## CTI Channel to Trigger 6 Enable Register

The CSCTI\_CTIOUTEN6 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-73: CSCTI\_CTIOUTEN6 Register Diagram

<!-- image -->

Table 51-80: CSCTI\_CTIOUTEN6 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger6. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[6]. |

## CTI Channel to Trigger 7 Enable Register

The CSCTI\_CTIOUTEN7 register maps channels in the cross trigger system to trigger outputs. The CTIOUTEN registers are bit maps that allow any channel input to be mapped to the trigger output, including none (0x0) and all (0xF). There is one register per trigger output so it is possible to map any channel input to any combination of trigger outputs.

CTIOUTEN1-31 registers are IMPLEMENTATION-DEFINED. If they are not implemented, the locations are RO and return 0. If they are implemented, they are RW and return a reset value of 0.

Figure 51-74: CSCTI\_CTIOUTEN7 Register Diagram

<!-- image -->

Table 51-81: CSCTI\_CTIOUTEN7 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | TRIGOUTEN  | Channel Trigger Output Enable. Channel to trigger output mapping. 0: The corresponding channel is ignored by the output trigger7. 1: When an event occurs on the channel corresponding to this bit, generate an event on event_out[7]. |

## CTI Trigger In Status Register

If the event\_in input is driven by a source that generates single cycle pulses, the CSCTI\_CTITRIGINSTATUS register is generally read as 0.

Figure 51-75: CSCTI\_CTITRIGINSTATUS Register Diagram

<!-- image -->

Table 51-82: CSCTI\_CTITRIGINSTATUS Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                             |
|--------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | TRIGINSTATUS | Trigger In Status. Trigger input status. 0: One bit per trigger input. 0 means that the input is LOW. 1: One bit per trigger input. 1 means that the input is HIGH. |

## CTI Trigger Out Status Register

The CSCTI\_CTITRIGOUTSTATUS register only has meaning if the trigger source drives static levels, rather than pulses.

Figure 51-76: CSCTI\_CTITRIGOUTSTATUS Register Diagram

<!-- image -->

Table 51-83: CSCTI\_CTITRIGOUTSTATUS Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                      |
|--------------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | TRIGOUTSTATUS | Trigger Output Status. Trigger output status. 0: One bit per trigger output. 0 means that the output is LOW. 1: One bit per trigger output. 1 means that the output is HIGH. |

## Device ID

The CSCTI\_DEVID register is implementation defined for each part number and designer. The register indicates the capabilities of the component.

Figure 51-77: CSCTI\_DEVID Register Diagram

<!-- image -->

Table 51-84: CSCTI\_DEVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:0 (R/NW)        | DEVID      | Device ID Field. Bits[19:16] Indicates the maximum number of triggers - the maximum of the two pa- rameters, NUM_EVENT_SLAVES and NUM_EVENT_MASTERS. Bits[15:8] Indicates the maximum number of triggers - the maximum of the two pa- rameters, NUM_EVENT_SLAVES and NUM_EVENT_MASTERS. Bits[7:5] Reserved bit or field with SBZP behavior Bits[4:0] Indicates the value of the EXT_MUX_NUM parameter, which determines if there is any external multiplexing on the trigger inputs and outputs. 0 indicates no multiplexing. |

## Device Type

A debugger can use the CSCTI\_DEVTYPE register to get information about a component that has an unrecognized part number.

Figure 51-78: CSCTI\_DEVTYPE Register Diagram

<!-- image -->

Table 51-85: CSCTI\_DEVTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | DEVTYPE    | Device Type Field. Bits[7:4] Minor classification. Returns 0x1, indicating this component is a Trigger-Ma- trix. Bits[3:0] Major classification. Returns 0x4, indicating this component performs De- bug Control. |

## Integration Test Channel Input

The CSCTI\_ITCHIN register is used to view channel events. The integration test register includes a latch that is set when a pulse is received on a channel input. When read, a register bit reads as 1 if the channel has received a pulse since it was last read. The act of reading the register automatically clears the 1 to a 0. When performing integration testing it is therefore important to coordinate the setting of event latches and reading/clearing them.

Figure 51-79: CSCTI\_ITCHIN Register Diagram

<!-- image -->

Table 51-86: CSCTI\_ITCHIN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                        |
|--------------------|------------|------------------------------------------------|
| 3:0                | CTCHIN     | Read the values of the CTCHIN inputs.          |
| (R/NW)             |            | Reads the latched value of the channel inputs. |

## ITCHINACK

Figure 51-80: CSCTI\_ITCHINACK Register Diagram

<!-- image -->

Table 51-87: CSCTI\_ITCHINACK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                 |
|--------------------|------------|-----------------------------------------|
| 3:0 (R0/W)         | CTCHINACK  | Set the value of the CTCHINACK outputs. |

## Integration Test Channel Output

The CSCTI\_ITCHOUT register is used to generate channel events. Writing to the register creates a single pulse on the output. The CSCTI\_ITCHOUT register is self-clearing.

Figure 51-81: CSCTI\_ITCHOUT Register Diagram

<!-- image -->

Table 51-88: CSCTI\_ITCHOUT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 3:0 (R0/W)         | CTCHOUT    | Set CTCHOUT Output Value. Pulses the channel outputs. 0: No effect. 1: Pulse channel event for one clk cycle. |

## ITCHOUTACK

Figure 51-82: CSCTI\_ITCHOUTACK Register Diagram

<!-- image -->

Table 51-89: CSCTI\_ITCHOUTACK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                   |
|--------------------|------------|-------------------------------------------|
| 3:0                | CTCHOUTACK | Read the values of the CTCHOUTACK inputs. |
| (R/NW)             |            |                                           |

## Integration Mode Control Register

The CSCTI\_ITCTRL register is used to enable topology detection.

Figure 51-83: CSCTI\_ITCTRL Register Diagram

<!-- image -->

Table 51-90: CSCTI\_ITCTRL Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                         |
|--------------------|------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | INTEGRATION_MODE | Integration Mode Enable. When set, the component enters integration mode, enabling topology detection or in- tegration testing to be performed. |

## Integration Test Trigger Input

The CSCTI\_ITTRIGIN register is used to view trigger events. The integration test register includes a latch that is set when a pulse is received on a trigger input. When read, a register bit reads as 1 if the trigger input has received a pulse since it was last read. The act of reading the register automatically clears the 1 to a 0. When performing integration testing it is therefore important to coordinate the setting of event latches and reading/clearing them.

Figure 51-84: CSCTI\_ITTRIGIN Register Diagram

<!-- image -->

Table 51-91: CSCTI\_ITTRIGIN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                        |
|--------------------|------------|------------------------------------------------|
| 7:0                | CTTRIGIN   | Read CTTRIGIN Input Values.                    |
| (R/NW)             |            | Reads the latched value of the trigger inputs. |

## ITTRIGINACK

Figure 51-85: CSCTI\_ITTRIGINACK Register Diagram

<!-- image -->

Table 51-92: CSCTI\_ITTRIGINACK Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                   |
|--------------------|-------------|-------------------------------------------|
| 7:0                | CTTRIGINACK | Set the value of the CTTRIGINACK outputs. |
| (R0/W)             |             |                                           |

## Integration Test Trigger Output

The CSCTI\_ITTRIGOUT register is used to generate trigger events.

Figure 51-86: CSCTI\_ITTRIGOUT Register Diagram

<!-- image -->

Table 51-93: CSCTI\_ITTRIGOUT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R0/W)         | CTTRIGOUT  | Set the value of the CTTRIGOUT outputs. Set/clear trigger output signal. Reads return the value in the register if SW_HAND- SHAKE=1, otherwise 0 is returned if SW_HANDSHAKE=0. Writes: 0: Clears the trigger output if SW_HANDSHAKE=1, no effect if SW_HAND- SHAKE=0. 1: Sets the trigger output if SW_HANDSHAKE=1, pulses trigger output if SW_HANDSHAKE=0. |

## ITTRIGOUTACK

Figure 51-87: CSCTI\_ITTRIGOUTACK Register Diagram

<!-- image -->

Table 51-94: CSCTI\_ITTRIGOUTACK Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                     |
|--------------------|--------------|---------------------------------------------|
| 7:0                | CTTRIGOUTACK | Read the values of the CTTRIGOUTACK inputs. |
| (R/NW)             |              |                                             |

## Lock Access Register

Figure 51-88: CSCTI\_LAR Register Diagram

<!-- image -->

Table 51-95: CSCTI\_LAR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | WACODE     | Write Access Code.        |
| (R0/W)             |            |                           |

## Lock Status Register

Figure 51-89: CSCTI\_LSR Register Diagram

<!-- image -->

Table 51-96: CSCTI\_LSR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/NW)           | ACCESS     | Access Blocked to Device. |
| 0 (R/NW)           | LOCK       | Lock Exists for Device.   |

## Peripheral ID0

The CSCTI\_PERIPHID0 register is part of the set of peripheral identification registers.

Figure 51-90: CSCTI\_PERIPHID0 Register Diagram

<!-- image -->

Table 51-97: CSCTI\_PERIPHID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0                | PID0       | Peripheral ID0 Field. Part number, bits[7:0]. Taken together with PIDR1.PART_1 it indicates the compo- nent. The Part Number is selected by the designer of the component. |
| (R/NW)             |            |                                                                                                                                                                            |

## Peripheral ID1

The CSCTI\_PERIPHID1 register is part of the set of peripheral identification registers.

Figure 51-91: CSCTI\_PERIPHID1 Register Diagram

<!-- image -->

Table 51-98: CSCTI\_PERIPHID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | PID1       | Peripheral ID1 Field. Bits[7:4] JEP106 identification code, bits[3:0]. Together, with PIDR4.DES_2 and PIDR2.DES_1, they indicate the designer of the component and not the implementer, except where the two are the same. Bits[3:0] Part number, bits[11:8]. Taken together with PIDR0.PART_0 it indicates the component. The Part Number is selected by the designer of the component. |

## Peripheral ID2

The CSCTI\_PERIPHID2 register is part of the set of peripheral identification registers.

Figure 51-92: CSCTI\_PERIPHID2 Register Diagram

<!-- image -->

Table 51-99: CSCTI\_PERIPHID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | PID2       | Peripheral ID2 Field. Bits[7:4] Revision. It is an incremental value starting at 0x0 for the first design of a component. See the Component list in Chapter 1 for information on the RTL revision of the component. Bit[3] 1 - Always set. Indicates that a JEDEC assigned value is used. Bits[2:0] JEP106 identification code, bits[6:4]. Together, with PIDR4.DES_2 and PIDR1.DES_0, they indicate the designer of the component and not the implementer, except where the two are the same. |

## Peripheral ID3

The CSCTI\_PERIPHID3 register is part of the set of peripheral identification registers.

Figure 51-93: CSCTI\_PERIPHID3 Register Diagram

<!-- image -->

Table 51-100: CSCTI\_PERIPHID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | PID3       | Peripheral ID3 Field. Bits[7:4] This field indicates minor errata fixes specific to this design, for example met- al fixes after implementation. In most cases this field is 0x0. Bits[3:0] Customer Modified. Where the component is reusable IP , this value indicates if the customer has modified the behavior of the component. In most cases this field is 0x0. |

## Peripheral ID4

The CSCTI\_PERIPHID4 register is part of the set of peripheral identification registers.

Figure 51-94: CSCTI\_PERIPHID4 Register Diagram

<!-- image -->

Table 51-101: CSCTI\_PERIPHID4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | PID4       | Peripheral ID4 Field. Bits[7:4] Indicates the memory size that is used by this component. Returns 0 indicat- ing that the component uses an UNKNOWNnumber of 4KB blocks. Using the SIZE field to indicate the size of the component is deprecated. Bits[3:0] JEP106 continuation code. Together, with PIDR2.DES_1 and PIDR1.DES_0, they indicate the designer of the component and not the implementer, except where the two are the same. |

## Peripheral ID5

The CSCTI\_PERIPHID5 register is part of the set of peripheral identification registers.

Figure 51-95: CSCTI\_PERIPHID5 Register Diagram

<!-- image -->

Table 51-102: CSCTI\_PERIPHID5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | PID5       | Peripheral ID5 Field.     |
| (R/NW)             |            |                           |

## Peripheral ID6

The CSCTI\_PERIPHID6 register is part of the set of peripheral identification registers.

Figure 51-96: CSCTI\_PERIPHID6 Register Diagram

<!-- image -->

Table 51-103: CSCTI\_PERIPHID6 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | PID6       | Peripheral ID6 Field.     |
| (R/NW)             |            |                           |

## Peripheral ID7

The CSCTI\_PERIPHID7 register is part of the set of peripheral identification registers.

Figure 51-97: CSCTI\_PERIPHID7 Register Diagram

<!-- image -->

Table 51-104: CSCTI\_PERIPHID7 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | PID7       | Peripheral ID7 Field.     |
| (R/NW)             |            |                           |