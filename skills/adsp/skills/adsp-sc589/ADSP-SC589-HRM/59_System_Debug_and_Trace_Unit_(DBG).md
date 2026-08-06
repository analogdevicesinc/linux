## 56   System Debug and Trace Unit (DBG)

The system debug and trace unit is based on ARM Core Sight technology. CoreSight™ is a set of architecture specifications defining debug and trace architecture. The processor uses CoreSight infrastructure to provide industry standard debug and trace capabilities.

## http://infocenter.arm.com/help/

The applicable documentation for more details about the ARM CoreSight feature includes:

- CoreSight PFT Architecture Specification , ARM IHI 0035B (PFT)
- System Trace Macrocell, Programmers' Model Architecture Specification, ARM IHI 0054A (STM)
- CoreSight Trace Memory Controller, ARM DDI0461B (TMC)
- CoreSight Components Technical Reference Manual, ARM DDI 0314H (TPIU)
- Embedded Cross Trigger Technical Reference Manual, ARM DDI 0291A
- ETM Architecture Specification, ARM IHI0014Q
- ETM for A5 Technical Reference Manual, DDI0435C

## DBG Features

The system debug and trace unit contains the following features.

- System JTAG TAP controller for system debug features, boundary scan, and public JTAG features
- A debug interface to cores, and other system resources
- Direct and run-time access to the memory system and system MMRs
- Direct control over system reset
- Support for debug immediately after reset (boot debug)
- Group halt (debug event immediately halts all specified endpoints)
- Real-time on-chip visibility is made available to all developers, including software developers

## DBG Functional Description

The following sections provide functional descriptions of the DBG unit.

## ADSP-SC58x CSPFT Register List

Table 56-1: ADSP-SC58x CSPFT Register List

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

Table 56-1: ADSP-SC58x CSPFT Register List (Continued)

| Name           | Description                             |
|----------------|-----------------------------------------|
| CSPFT_STAT     | Status Register                         |
| CSPFT_SYNCFR   | Synchronization Frequency Register      |
| CSPFT_TECTL    | TraceEnable Control Register            |
| CSPFT_TEEVENT  | TraceEnable Event Register              |
| CSPFT_TRACEIDR | CoreSight Trace ID Register             |
| CSPFT_TRIGGER  | Trigger Event Register                  |
| CSPFT_TSSCTL   | TraceEnable Start/Stop Control Register |

## ADSP-SC58x TAPC Register List

The Test Access Port Controller (TAPC) provides access to debug features. A set of registers governs TAPC operations. For more information on TAPC functionality, see the TAPC register descriptions.

Table 56-2: ADSP-SC58x TAPC Register List

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

## ADSP-SC58x TAPC Register List

The Test Access Port Controller (TAPC) provides access to debug features. A set of registers governs TAPC operations. For more information on TAPC functionality, see the TAPC register descriptions.

Table 56-3: ADSP-SC58x TAPC Register List

| Name          | Description                 |
|---------------|-----------------------------|
| TAPC_DBGCTL   | Debug Control Register      |
| TAPC_IDCODE   | IDCODE Register             |
| TAPC_SDBGKEY0 | Secure Debug Key 0 Register |

Table 56-3: ADSP-SC58x TAPC Register List (Continued)

| Name              | Description                       |
|-------------------|-----------------------------------|
| TAPC_SDBGKEY1     | Secure Debug Key 1 Register       |
| TAPC_SDBGKEY2     | Secure Debug Key 2 Register       |
| TAPC_SDBGKEY3     | Secure Debug Key 3 Register       |
| TAPC_SDBGKEY_CTL  | Secure Debug Key Control Register |
| TAPC_SDBGKEY_STAT | Secure Debug Key Status Register  |
| TAPC_USERCODE     | USERCODE Register                 |

## ADSP-SC58x STM Trigger List

Table 56-4: ADSP-SC58x STM Trigger List Masters

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

Table 56-5: ADSP-SC58x STM Trigger List Slaves

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|           52 | STM0_EVT0  | STM0 STM0 Event 0  | Edge          |
|           53 | STM0_EVT1  | STM0 STM0 Event 1  | Edge          |
|           54 | STM0_EVT2  | STM0 STM0 Event 2  | Edge          |
|           55 | STM0_EVT3  | STM0 STM0 Event 3  | Edge          |
|           56 | STM0_EVT4  | STM0 STM0 Event 4  | Edge          |
|           57 | STM0_EVT5  | STM0 STM0 Event 5  | Edge          |
|           58 | STM0_EVT6  | STM0 STM0 Event 6  | Edge          |
|           59 | STM0_EVT7  | STM0 STM0 Event 7  | Edge          |
|           60 | STM0_EVT8  | STM0 STM0 Event 8  | Edge          |
|           61 | STM0_EVT9  | STM0 STM0 Event 9  | Edge          |
|           62 | STM0_EVT10 | STM0 STM0 Event 10 | Edge          |
|           63 | STM0_EVT11 | STM0 STM0 Event 11 | Edge          |
|           64 | STM0_EVT12 | STM0 STM0 Event 12 | Edge          |
|           65 | STM0_EVT13 | STM0 STM0 Event 13 | Edge          |
|           66 | STM0_EVT14 | STM0 STM0 Event 14 | Edge          |
|           67 | STM0_EVT15 | STM0 STM0 Event 15 | Edge          |
|           68 | STM0_EVT16 | STM0 STM0 Event 16 | Edge          |

Table 56-5: ADSP-SC58x STM Trigger List Slaves (Continued)

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|           69 | STM0_EVT17 | STM0 STM0 Event 17 | Edge          |
|           70 | STM0_EVT18 | STM0 STM0 Event 18 | Edge          |
|           71 | STM0_EVT19 | STM0 STM0 Event 19 | Edge          |
|           72 | STM0_EVT20 | STM0 STM0 Event 20 | Edge          |
|           73 | STM0_EVT21 | STM0 STM0 Event 21 | Edge          |
|           74 | STM0_EVT22 | STM0 STM0 Event 22 | Edge          |
|           75 | STM0_EVT23 | STM0 STM0 Event 23 | Edge          |
|           76 | STM0_EVT24 | STM0 STM0 Event 24 | Edge          |
|           77 | STM0_EVT25 | STM0 STM0 Event 25 | Edge          |
|           78 | STM0_EVT26 | STM0 STM0 Event 26 | Edge          |
|           79 | STM0_EVT27 | STM0 STM0 Event 27 | Edge          |
|           80 | STM0_EVT28 | STM0 STM0 Event 28 | Edge          |
|           81 | STM0_EVT29 | STM0 STM0 Event 29 | Edge          |
|           82 | STM0_EVT30 | STM0 STM0 Event 30 | Edge          |
|           83 | STM0_EVT31 | STM0 STM0 Event 31 | Edge          |

## ADSP-SC58x CTI Trigger List

Table 56-6: ADSP-SC58x CTI Trigger List Masters

|   Trigger ID | Name      | Description                            | Sensitivity   |
|--------------|-----------|----------------------------------------|---------------|
|           84 | CTI3_MST0 | CTI3 SYSCTI (CTI3) System Halt Slave 0 | Edge          |
|           85 | CTI3_MST1 | CTI3 SYSCTI (CTI3) System Halt Slave 1 | Edge          |
|           86 | CTI3_MST2 | CTI3 SYSCTI (CTI3) System Halt Slave 2 | Edge          |
|           87 | CTI3_MST3 | CTI3 SYSCTI (CTI3) System Halt Slave 3 | Edge          |
|           88 | CTI3_MST4 | CTI3 SYSCTI (CTI3) System Halt Slave 4 | Edge          |
|           89 | CTI3_MST5 | CTI3 SYSCTI (CTI3) System Halt Slave 5 | Edge          |
|           90 | CTI3_MST6 | CTI3 SYSCTI (CTI3) System Halt Slave 6 | Edge          |
|           91 | CTI3_MST7 | CTI3 SYSCTI (CTI3) System Halt Slave 7 | Edge          |

Table 56-7: ADSP-SC58x CTI Trigger List Slaves

|   Trigger ID | Name      | Description                      | Sensitivity   |
|--------------|-----------|----------------------------------|---------------|
|          110 | CTI3_SLV0 | CTI3 SYSCTI System Halt Master 0 |               |

Table 56-7: ADSP-SC58x CTI Trigger List Slaves (Continued)

|   Trigger ID | Name      | Description                      | Sensitivity   |
|--------------|-----------|----------------------------------|---------------|
|          111 | CTI3_SLV1 | CTI3 SYSCTI System Halt Master 1 |               |
|          112 | CTI3_SLV2 | CTI3 SYSCTI System Halt Master 2 |               |
|          113 | CTI3_SLV3 | CTI3 SYSCTI System Halt Master 3 |               |
|          114 | CTI3_SLV4 | CTI3 SYSCTI System Halt Master 4 |               |
|          115 | CTI3_SLV5 | CTI3 SYSCTI System Halt Master 5 |               |
|          116 | CTI3_SLV6 | CTI3 SYSCTI System Halt Master 6 |               |
|          117 | CTI3_SLV7 | CTI3 SYSCTI System Halt Master 7 |               |

## ADSP-SC58x CTI Interrupt List

Table 56-8: ADSP-SC58x CTI Interrupt List

|   Interrupt ID | Name      | Description           | Sensitivity   | DMA Channel   |
|----------------|-----------|-----------------------|---------------|---------------|
|            127 | CTI1_EVT0 | CTI1 Core 1 CTI Event |               |               |
|            128 | CTI2_EVT0 | CTI2 Core 2 CTI Event |               |               |
|            252 | CTI0_EVT0 | CTI0 Core 0 CTI Event |               |               |

## DBG Block Diagram

The block diagram is shown below.

Figure 56-1: ADSP-SC5xx Block Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000000_ad4621b834985edc47ad0276b5f268ef7868efa2607c03476bef615af3022af3.png)

## DBG Definitions

The following terms are useful when working with the debug features of the processor and programming tools.

## Test Access Port Controller (ADI TAPC)

Provides IDCODE and SDBGKEY features.

## Debug Access Port (DAP)

Core Sight Interface providing a single port for two debug options: JTAG-DP (JTAP Debug Port), SW-DP (Serial Wire Debug Port)

## Embedded Trace Macrocell (ETM)

Provides ARM Core Trace.

## Program Trace Macrocell (PTM)

Provides DSP (SHARC + )Core Trace.

## Standard Trace Macrocell (STM)

Provides capability to trace up to 32 hardware events and supports 32 software stimulus for data transfer between the user and emulator.

## Embedded Cross Trigger (ECT)

The ECTs are responsible managing events and triggers as follows.

- CTI (Cross Trigger Interface) is a CoreSight component for enabling cross triggering of events across a system. From the view of the ECT, it is responsible for combining and mapping trigger requests.
- CTM (Cross Trigger Matrix) is a CoreSight component for connecting multiple CTIs. From the view of the ECT, it is responsible for connecting CTIs and distribution of events.

NOTE: An embedded cross trigger is not the same as the master and slave trigger in the trigger routing unit.

## Trace Capture Devices

The trace capture devices capture and format the trace data. There are two trace capture devices in the system:

- ETF - Embedded Trace Funnel - Provides a buffer for burst trace data.
- ETR - Embedded Trace Router - Provides interface for trace data to be stored in system memories.

## Trace Port Interface Unit (TPIU)

The TPIU acts as a bridge between the on-chip trace data, with separate IDs, to a data stream. It encapsulates IDs, when required, that the Trace Port Analyzer (TPA) captures.

## Serial Wire Output (SWO)

SWO is a trace data drain that acts as a bridge between the on-chip trace data and a data stream that the T race Port Analyzer (TPA) captures.

## Test Access Port Controller (TAPC)

TAPC is an independent component daisy-chained to the DAP in the JTAG scan path. The TAPC component provides the product IDCODE (Chip ID) and a security feature.

The TAPC component provides security features to the chip using a debug key match features. This feature permits only those components which have the SDBGKEY to connect and debug the chip. Initially, a 128-bits user key is programmed in the SDBGKEY registers ( TAPC\_SDBGKEY0 , TAPC\_SDBGKEY1 , TAPC\_SDBGKEY2 , TAPC\_SDBGKEY3 ) in TAPC by a secure master. The TAPC\_SDBGKEY\_CTL.VALID bit is set.

Then, a user key is entered through the emulator for a match to the initially programmed-entered user key. On a successful match, the chip connects to the emulator. It can be debugged on a failed user key match. Further attempts at keys matching are disabled. JTAG reset or system reset is required to reenable the user key match logic.

## Embedded Trace Macrocell (ETM)

ETM is the standard trace support provided for the A5 processor. For details of programming and functionality, refer to the ARM documentation.

## Debug Access Ports

The Debug Access Port (DAP) is an implementation of an ARM Debug Interface version 5 (ADIv5) comprising a number of components supplied in a single configuration. All the supplied components fit into the various architectural components for Debug Ports (DPs). The components are used to access the DAP from an external debugger and APs, to access on-chip system resources.

The DAP provides access to all debug and trace capabilities through a single external interface. DAP has some additions to support BSCA and IDCODE features that Core Sight DAP does not support. The DAP provides a combined single debug interface port called SWJ\_DP that includes:

- JTAG Debug Port (JTAG-DP). The JTAG-DP is based on the IEEE 1149.1 Test Access Port (TAP) and Boundary Scan Architecture
- Serial Wire Debug Port (SW-DP). The serial wire debug port provides a bidirectional serial connection to the ARM debug interface

The SWJ-DP is a combined JTAG-DP and SW-DP that allows connections to either a Serial Wire Debug (SWD) or JTAG probe to a target. It is the standard CoreSight debug port, and enables access either to the JTAG-DP or SW-DP blocks. The JTAG-DP is selected by default.

## Trace Unit

The trace module provides instruction, data tracing, and system activity tracing for the processor. The program trace module on the processor is similar to the embedded trace macrocell module provided by the ARM processor. System trace is provided using the system trace macrocell as part of the CoreSight debug and trace interface. The trace module uses an interface based on the AMBA Trace Bus (ATB) standard to output its trace data. The trace data can be either exported to an off-chip trace port analyzer or captured on an on-chip buffer. The PFT and STM modules capture information on the processor both before and after a specific event. The modules add no burden to the processor performance when it runs at full speed.

- Programmable Flow Trace (CSPFT)
- System Trace Module (STM)
- Embedded Trace Macrocell (ETM)

## Programmable Flow Trace (CSPFT)

When tracing processor execution, trace information can be generated for every instruction the processor executes. This information would be easy to interpret, but would require a prohibitively high trace bandwidth to get the trace data off the chip. With program flow tracing, only branch points are traced. The debugger uses the source code to infer the rest of the executed code.

Certain instructions in the program and events are identified as waypoints. A waypoint is a point where instruction execution involves a change of program flow. The CSPFT only traces those waypoints. These waypoints are:

- All indirect branches
- All direct branches
- Exceptions or interrupts
- Emulator debug entry and exit

When a waypoint occurs, trace data is generated to describe it. From this data and the source code, a trace decompressor can determine what instructions were executed and recreate the instruction flow. To allow the decompressor to calculate where it is in the source code, conditional instructions are marked as waypoints, regardless of whether they pass or fail their condition test. Events like interrupts or debug entry and exit can be promoted from nonwaypoint instructions to waypoints to trace the interrupted program flow.

Tracing a waypoint implies the execution of all instructions from the target address of the previous waypoint up to the current waypoint. Non-waypoint instructions are not explicitly traced but the debugger must infer them using the source code. The concept of an instruction block is used throughout this manual and refers to the contiguous block of instructions between two waypoints.

The programming model and function is a subset of the PTM (Programmable Trace Module) of ARM.

## System Trace Module (STM)

The STM is a trace source that is integrated into a CoreSight system, and is designed primarily for high-bandwidth trace of instrumentation embedded into software. The STM enables tracing of system activity from various sources:

- Instrumented software, using memory-mapped stimulus ports
- Hardware events

The STM supports the following features:

- Multiple software masters writing software instrumentation independently. Each master can use multiple stimulus ports.
- Time stamping of the system activity. The time stamp is a global time stamp which can be shared with other trace sources in the system to enable correlation of activity from multiple trace sources.
- Indication that specific events have occurred, such as a particular hardware event or a piece of software instrumentation. These events are known as triggers and can be indicated in the trace stream, or through signals to other system components.

Thirty-two hardware event resources are connected as output from the TRU which allows monitoring all of the hardware events that can generate a trigger.

## Embedded Cross Trigger (ECT)

ECT provides an interface to the CoreSight debug system enabling the subsystems to interact (cross trigger) with each other. ECT provides a mechanism to forward debug events from one connected subsystem to another connected subsystem. The different subsystems connected to the ECT depend on the processor design. For example, in a multiprocessor system, the interface can be connected to each of the cores and one to the trace subsystem. For a uniprocessor system, the interface can include just the core and trace subsystem connection.

- CTI Cross trigger interface. A CoreSight component for enabling cross triggering of events across a system.
- CTM Cross trigger matrix. A CoreSight component for connecting multiple cross trigger interfaces.

The main function of the ECT (CTI and CTM) is to pass debug events from one connected subsystem to another connected subsystem.

For example, the ECT can communicate debug state information from the core to trace subsystem for a single processor system or to another core in a multiprocessor-based system. Program execution on both the subsystem can be stopped at the same time.

The Trigger Flow figure shows a simple debug trigger flow sequence. On each CTI, there are four channel, eight input, and eight output debug triggers. All the eight inputs and outputs can be mapped to a single channel or different channels based on the debug trigger to channel mapping. When a trigger input occurs, it creates a channel event. The channel event causes all the output debug triggers to be triggered. The embedded cross trigger depends on the debug trigger it connects to.

Figure 56-2: Trigger Flow

Refer to the ECT Integration figure. There are 5 CTIs in the system that connect to the core, trace, and system module, respectively. The CTIs all interconnect through the CTM. This configuration allows the core to trigger debug events on the trace, on the system and on the core itself. CTI0 handles all the ETM and core0 debug triggers. CTI1 handles all the PFT and core1 debug triggers. CTI2 handles all the PFT and core2 debug triggers. CTI3 handles all the system debug triggers. CTI4 handles all the trace components debug triggers.

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000001_99affd096cce0ad847fd1c1a98876e2a5bfcd98c02fe8075516fa36c3f2dacc5.png)

Figure 56-3: ECT Integration

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000002_70a23edfa3166dfc4068e0ca449708ea7e9b9406ec7b9a220a39433e3054b344.png)

## CTI Debug Trigger Tables

The System CTI Trigger Connection tables show the debug trigger that connects to each CTI.

Table 56-9: System CTI Trigger Connection

| CTI Port        | Input    | CTI Port        | Output                                |
|-----------------|----------|-----------------|---------------------------------------|
| CTITRIGIN[7]    | From TRU | CTITRIGOUT[7]   | To TRU and also as SYS_DBGRESTART     |
| CTITRIGIN [6:2] | From TRU | CTITRIGOUT[6:2] | To TRU                                |
| CTITRIGIN [1]   | From TRU | CTITRIGOUT[1]   | To TRU and also as peripheral halt    |
| CTITRIGIN[0]    | From TRU | CTITRIGOUT[0]   | To TRU and also as system fabric halt |

Table 56-10: Trace CTI Trigger Connection

| CTI Port     | Input           | CTI Port       | Output       |
|--------------|-----------------|----------------|--------------|
| CTITRIGIN[7] | STM ASYNCOUT    | CTITRIGOUT[7]  | Unused       |
| CTITRIGIN[6] | STM TRIGOUTHETE | CTITRIGOUT [6] | TPIU FLUSHIN |
| CTITRIGIN[5] | STM TRIGOUTSW   | CTITRIGOUT [5] | TPIU TRIGIN  |
| CTITRIGIN[4] | STM TRIGOUTSPTE | CTITRIGOUT [4] | ETR FLUSHIN  |
| CTITRIGIN[3] | ETR FULL        | CTITRIGOUT [3] | ETR TRIGIN   |
| CTITRIGIN[2] | ETR ACQCOMP     | CTITRIGOUT [2] | ETF FLUSHIN  |
| CTITRIGIN[1] | ETF FULL        | CTITRIGOUT [1] | ETF TRIGIN   |
| CTITRIGIN[0] | ETF ACQCOMP     | CTITRIGOUT [0] | Unused       |

Table 56-11: SC5xx- Core 1/2 CTI Trace Connection

| CTI Port       | Input           | CTI Port        | Output         |
|----------------|-----------------|-----------------|----------------|
| CTITRIGIN[7]   | Tied Low        | CTITRIGOUT[7]   | DBGRESTART     |
| CTITRIGIN[6]   | PTM TRIGGER     | CTITRIGOUT[6]   | SEC            |
| CTITRIGIN[5:2] | PTM EXTOUT[3:0] | CTITRIGOUT[5:2] | PTM EXTIN[3:0] |
| CTITRIGIN[1]   | Tied Low        | CTITRIGOUT[1]   | Unused         |
| CTITRIGIN[0]   | DBGTRIGGER      | CTITRIGOUT[0]   | EDBGRQ         |

Table 56-12: SC5xx ARM Core CTI Connection

| CTI Port     | Input       | CTI Port      | Output       |
|--------------|-------------|---------------|--------------|
| CTITRIGIN[7] | Tied Low    | CTITRIGOUT[7] | DBGRESTART   |
| CTITRIGIN[6] | ETM TRIGGER | CTITRIGOUT[6] | SEC          |
| CTITRIGIN[5] | COMMRX      | CTITRIGOUT[5] | Unused       |
| CTITRIGIN[4] | COMMTX      | CTITRIGOUT[4] | ETM EXTIN[3] |

Table 56-12: SC5xx ARM Core CTI Connection (Continued)

| CTI Port       | Input          | CTI Port         | Output         |
|----------------|----------------|------------------|----------------|
| CTITRIGIN[3:2] | ETMEXTOUT[1:0] | CTITRIGOUT[3:2 ] | ETM EXTIN[2:1] |
| CTITRIGIN[1]   | PMU IRQ        | CTITRIGOUT[1]    | ETM EXTIN[0]   |
| CTITRIGIN[0]   | DBGTRIGGER     | CTITRIGOUT[0]    | EDBGRQ         |

Table 56-13: Trigger Descriptions

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

Table 56-13: Trigger Descriptions (Continued)

| Signal Name    | Description                                                                                                                                                                                                                               |
|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| EDBGRQ         | This is an output from CTI and an input (as debug halt) to a processor core and to the system in general. It can assert as a result of another core going to emulation space (DBGTRIGGER) or by setting the corresponding bit in the CTI. |
| SYS_DBGRESTART | This is an output from the CTI to system to return from debug mode.                                                                                                                                                                       |
| To TRU         | TRU Master Event                                                                                                                                                                                                                          |
| From TRU       | TRU Slave Event                                                                                                                                                                                                                           |
| COMMRX         | The COMMRXoutput signals enable interrupt-driven communications over the DTR                                                                                                                                                              |
| COMMTX         | The COMMTXoutput signals enable interrupt-driven communications over the DTR                                                                                                                                                              |
| PMU IRQ        | Performance Monitor Unit on ARM Interrupt Signal                                                                                                                                                                                          |

## ADSP-SC58x CSPFT Register Descriptions

Program Flow Trace (CSPFT) contains the following registers.

Table 56-14: ADSP-SC58x CSPFT Register List

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

Table 56-14: ADSP-SC58x CSPFT Register List (Continued)

| Name               | Description                             |
|--------------------|-----------------------------------------|
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
| CSPFT_TECTL        | TraceEnable Control Register            |
| CSPFT_TEEVENT      | TraceEnable Event Register              |
| CSPFT_TRACEIDR     | CoreSight Trace ID Register             |
| CSPFT_TRIGGER      | Trigger Event Register                  |
| CSPFT_TSSCTL       | TraceEnable Start/Stop Control Register |

## Address Comparator Access Type Register

The CSPFT\_ACTR[n] register specifies whether the context ID needs to match.

Figure 56-4: CSPFT\_ACTR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000003_18116735e64542e1c2f1d46b48b12a2575c306e3e7e81602ca68a0b927e42b5e.png)

Table 56-15: CSPFT\_ACTR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                            | Description/Enumeration                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|
| 9:8                | CIDCTRL    | Context ID Comparator Control. The CSPFT_ACTR[n].CIDCTRL contains the ID comparator control value. | Context ID Comparator Control. The CSPFT_ACTR[n].CIDCTRL contains the ID comparator control value. |
| (R/W)              |            | 0                                                                                                  | Ignore Context ID                                                                                  |
|                    |            | 1                                                                                                  | Match if Context ID Comparator 0 Matches                                                           |
|                    |            | 2                                                                                                  | Match if Context ID Comparator 1 Matches                                                           |
|                    |            | 3                                                                                                  | Match if Context ID Comparator 2 Matches                                                           |

## Address Comparator Value Register

The CSPFT\_ACVR[n] register holds an address for comparison.

Figure 56-5: CSPFT\_ACVR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000004_a49b042f8f5ac82394827058e2b91af52b0027654525a5196366fea9d4820424.png)

Table 56-16: CSPFT\_ACVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 31:0               | ADDR       | Address for Comparison.                                                    |
| (R/W)              |            | The CSPFT_ACVR[n].ADDR bit field contains the address used for comparison. |

## Authentication Status Register

The CSPFT\_AUTHSTATUS register reports the level of tracing currently permitted based on the DBGEN signal.

Figure 56-6: CSPFT\_AUTHSTATUS Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000005_22017d80ffa714a3eb165b1a8c005ad7e992e234beb5917fa6d2edd2d2c30430.png)

Table 56-17: CSPFT\_AUTHSTATUS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | ONE        | Always reads as one.                                                                                                                                                                                                                                                                                                                                                                         |
| 6 (R/NW)           | DBGEN      | Debug Enabled. The CSPFT_AUTHSTATUS.DBGEN bit indicates that invasive debug is enabled. Normally, NIDEN is used in conjunction with a signal that enables invasive debug, DBGEN. Non-invasive debug is disabled only if both NIDEN and DBGEN signals are LOW. In a PTM, typically these signals are ORed together and the result is used to determine whether non-invasive debug is enabled. |

## Configuration Code Extension Register

The CSPFT\_CCER register holds extra feature information. (See CSPFT\_HWFEAT .)

Figure 56-7: CSPFT\_CCER Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000006_6a991147ebb7b8762fcacffc6fdab61451efdb392223eabacaec9e72947a91ae.png)

Table 56-18: CSPFT\_CCER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/NW)          | VEI        | Virtualization Extensions Implemented. The CSPFT_CCER.VEI bit indicates if the virtualization extensions are implement- ed 0 Not Implemented |
| 23 (R/NW)          | RSI        | Return Stack Implemented. The CSPFT_CCER.RSI bit indicates if a return stack is implemented. 0 Not Implemented                               |
| 22 (R/NW)          | TSI        | Time Stamping Implemented. The CSPFT_CCER.TSI bit indicates if time stamping is implemented. 0 Disabled 1 Enabled                            |

## Component ID0 Register

The CSPFT\_CID0 register holds sections of the CoreSight Component ID for CSPFT.

Figure 56-8: CSPFT\_CID0 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000007_7a588f83050f020d3737c915411aede175605ff5554789b98b5654ff854ccf32.png)

Table 56-19: CSPFT\_CID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID0.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Component ID1 Register

The CSPFT\_CID1 register holds sections of the CoreSight Component ID for CSPFT.

Figure 56-9: CSPFT\_CID1 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000008_56cf3f97c5dc508afc83b848435019f641759c7db3308fc782692c9bed2b1f23.png)

Table 56-20: CSPFT\_CID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0                | COMPID     | Component ID. The CSPFT_CID1.COMPID bit field identifies this component as a CoreSight com- ponent. |
| (R/NW)             |            |                                                                                                     |

## Component ID2 Register

The CSPFT\_CID2 register holds sections of the CoreSight Component ID for CSPFT.

Figure 56-10: CSPFT\_CID2 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000009_b63d915f774aea436aeffb71ca3e4e3c8be16bf74d099c183c74524c8aad7f40.png)

Table 56-21: CSPFT\_CID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID2.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Component ID3 Register

The CSPFT\_CID3 register holds sections of the CoreSight Component ID for CSPFT.

Figure 56-11: CSPFT\_CID3 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000010_7f7afdc65937f9b80ab95b284e79c09affc71ceaf447a515c13cd19f3e0c6138.png)

Table 56-22: CSPFT\_CID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | COMPID     | Component ID. The CSPFT_CID3.COMPID bit field identifies this component as a CoreSight com- ponent. |

## Context ID Comparator Mask Register

The CSPFT\_CIDCMR register holds a 32-bit mask for use for all context ID comparisons.

Figure 56-12: CSPFT\_CIDCMR Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000011_291945eec0c2c7492c69147e11c68e68896fe732adbef517c546698d40f4476c.png)

Table 56-23: CSPFT\_CIDCMR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | MASK       | Context ID Mask Value. The CSPFT_CIDCMR.MASK bit field holds a 32-bit mask for use in all context ID comparisons. |

## Context ID Comparator Value

The CSPFT\_CIDCVR[n] register holds a context ID value for comparison.

Figure 56-13: CSPFT\_CIDCVR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000012_10e2529edcd89199bd5db835700ac6ebb8b3b129267ca0a61864187bf5435ee9.png)

Table 56-24: CSPFT\_CIDCVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Context ID Comparator Value Register. The CSPFT_CIDCVR[n].VALUE bit field holds a context ID value for compari- son. |

## Claim Tag Clear Register

The CSPFT\_CLAIMCLR register is used to clear bits in the claim tag or get the current value of the claim tag.

Figure 56-14: CSPFT\_CLAIMCLR Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000013_db5651b1af985cde37d1c7c5651d0a479a396271b27f87e915bf58e4216a5209.png)

Table 56-25: CSPFT\_CLAIMCLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------|
| 3:0 (R/W1C)        | TAGS       | Tags. A read of the CSPFT_CLAIMCLR.TAGS bit field returns the current value, a write clears bits. |

## Claim Tag Set Register

The CSPFT\_CLAIMSET register is used to set bits in the claim tag and find the number of bits supported by the claim tag.

Figure 56-15: CSPFT\_CLAIMSET Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000014_87f27503e6b8dc49fe403b9666b99d68c7e751f1c346037965f186157af702c7.png)

Table 56-26: CSPFT\_CLAIMSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R1/W1S)       | TAGS       | Supported Tags. The CSPFT_CLAIMSET.TAGS bit field sets bits in the claim tag and finds the number of bits supported by the claim tag. |

## Counter Enable Event Register

The CSPFT\_CNTENR[n] register describes the event that enables the corresponding counter.

Figure 56-16: CSPFT\_CNTENR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000015_8e0b977b39b8a4b0dcfe0eb459aadb2c9d6c78c4f8ba896d6a4c20194deeb54b.png)

Table 56-27: CSPFT\_CNTENR[n] Register Fields

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

Table 56-27: CSPFT\_CNTENR[n] Register Fields (Continued)

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

Table 56-27: CSPFT\_CNTENR[n] Register Fields (Continued)

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

Figure 56-17: CSPFT\_CNTRLDEVR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000016_c1fcd2815f093d00f3370bda9cc668d8b9e79bcba0a260b1537af2898e21b1c0.png)

Table 56-28: CSPFT\_CNTRLDEVR[n] Register Fields

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

Table 56-28: CSPFT\_CNTRLDEVR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | RESA       | Resource A. The CSPFT_CNTRLDEVR[n].RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_CNTRLDEVR[n].FUNC field. 0 Single Addr Comparator 0 1 Single Addr Comparator 1 2 Single Addr Comparator 2 3 Single Addr Comparator 3 4 Single Addr Comparator 4 5 Single Addr Comparator 5 6 Single Addr Comparator 6 7 Single Addr Comparator 7 8 Single Addr Comparator 8 9 Single Addr Comparator 9 10 Single Addr Comparator 10 11 Single Addr Comparator 11 12 Single Addr Comparator 12 13 Single Addr Comparator 13 14 Single Addr Comparator 14 15 Single Addr Comparator 15 16 Addr Range Comparator 0 17 Addr Range Comparator 1 18 Addr Range Comparator 2 19 Addr Range Comparator 3 20 Addr Range Comparator 4 21 Addr Range Comparator 5 22 Addr Range Comparator 6 |

Table 56-28: CSPFT\_CNTRLDEVR[n] Register Fields (Continued)

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

Figure 56-18: CSPFT\_CNTRLDVR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000017_5b3c59ef530b87184b6ab5aeb11438fe7556944f9c885ee7482cc4271c7b9f93.png)

Table 56-29: CSPFT\_CNTRLDVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Counter Initial Value. The CSPFT_CNTRLDVR[n].VALUE bit field specifies the starting value of the cor- responding counter. |

## Counter Value Register

The CSPFT\_CNTVR[n] register holds the current value of the corresponding counter.

Figure 56-19: CSPFT\_CNTVR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000018_761c76f8d379153eed9d64864790ae687a02c3dcd88a0b8e5d3dc6c83939b4f6.png)

Table 56-30: CSPFT\_CNTVR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Current Counter Value. The CSPFT_CNTVR[n].VALUE bit field specifies the current value of the corre- sponding counter. |

## Main Control Register

The CSPFT\_CTL register controls general operation of the PTM, such as whether tracing is enabled.

Figure 56-20: CSPFT\_CTL Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000019_ae6ae8a732a5ba72890ba140af8b847be3b72234dc6abdffc47309672915d469.png)

Table 56-31: CSPFT\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | RSENA      | Return Stack Enable. When the CSPFT_CTL.RSENA bit is set, the first indirect branch back to an address generates a branch without exception packet, and subsequent branches back to the same address generate E Atoms. This compresses inner loops of HWloops and code that indirectly branches back. |
| 15:14 (R/W)        | CIDSZ      | Context ID Size. The CSPFT_CTL.CIDSZ bit field specifies the byte size to trace. Only the bytes specified are traced, even if the new Context ID value is larger than this.                                                                                                                           |
|                    |            | 0 No Context ID Tracing                                                                                                                                                                                                                                                                               |
|                    |            | 1 One byte Traced                                                                                                                                                                                                                                                                                     |
|                    |            | 2 Two Bytes Traced                                                                                                                                                                                                                                                                                    |
|                    |            | 3 Three Bytes Traced                                                                                                                                                                                                                                                                                  |

Table 56-31: CSPFT\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | PB         | Programming Bit. To program the CSPFT, use the following procedure. 1. Set the CSPFT_CTL.PB bit to disable all trace functionality. 2. Poll the CSPFT_STAT.PB bit waiting for it to be 1 (FIFO drained, trace halted). 3. Program the trace registers, counter and other registers, as required. 4. Set this bit to 0. 5. Poll the CSPFT_STAT.PB bit until it reads 0 (trace status reset, trace restarted). When the CSPFT_CTL.PB bit is set, the FIFO is drained and no more trace is pro- duced. All counters are held in their present state and the external outputs are forced low. After the FIFO is drained, the CSPFT_STAT.PB is set to reflect that the part is ready to program. When this bit is cleared, the trace status is cleared and trace is restarted. 0 Trace Enabled |
| 8 (R/W)            | BBRAN      | Branch Broadcast. Set the CSPFT_CTL.BBRAN bit to 1 to enable branch broadcasting. Branch broad- casting traces the address of direct branch instructions rather than producing E atoms.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Device Type Identifier Register

The CSPFT\_DEVTYPE register is read-only. It provides a debugger with information about the component when the part number field is not recognized. The debugger can then report this information.

Figure 56-21: CSPFT\_DEVTYPE Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000020_6e33b71a1de474970359d6978268e3620dfd3fffe098d9e52f078f525000628e.png)

Table 56-32: CSPFT\_DEVTYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 7:4                | STYPE      | Sub Type = DSP.             |
| 3:0 (R/NW)         | TYPE       | Device Type = Trace Source. |

## External Output Event Register

The CSPFT\_EXTOUTEVR[n] register defines the event that controls the corresponding EXTOUT external output signal.

Figure 56-22: CSPFT\_EXTOUTEVR[n] Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000021_9532b5918a191cb97fb6d47cca32b865807e3983086ee6cba5704851b130b536.png)

Table 56-33: CSPFT\_EXTOUTEVR[n] Register Fields

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

Table 56-33: CSPFT\_EXTOUTEVR[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | RESA       | Resource A. The CSPFT_EXTOUTEVR[n].RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_EXTOUTEVR[n].FUNC field. 0 Single Addr Comparator 0 1 Single Addr Comparator 1 2 Single Addr Comparator 2 3 Single Addr Comparator 3 4 Single Addr Comparator 4 5 Single Addr Comparator 5 6 Single Addr Comparator 6 7 Single Addr Comparator 7 8 Single Addr Comparator 8 9 Single Addr Comparator 9 10 Single Addr Comparator 10 11 Single Addr Comparator 11 12 Single Addr Comparator 12 13 Single Addr Comparator 13 14 Single Addr Comparator 14 15 Single Addr Comparator 15 16 Addr Range Comparator 0 17 Addr Range Comparator 1 18 Addr Range Comparator 2 19 Addr Range Comparator 3 20 Addr Range Comparator 4 21 Addr Range Comparator 5 22 Addr Range Comparator 6 |

Table 56-33: CSPFT\_EXTOUTEVR[n] Register Fields (Continued)

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

Figure 56-23: CSPFT\_HWFEAT Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000022_700311888efa69783f91d6c24c0874dd98625e8554895ccb1c654ff5047f5fac.png)

Table 56-34: CSPFT\_HWFEAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/NW)       | NCIDC      | Number of Context ID Comparators. The CSPFT_HWFEAT.NCIDC bit field identifies the number of context ID compa- rators.                                                                                                                                                                                                                                                                                                                                               |
| 22:20 (R/NW)       | NEXO       | Number of External Outputs. The CSPFT_HWFEAT.NEXO bit field identifies the number of external outputs (up to four).                                                                                                                                                                                                                                                                                                                                                 |
| 19:17 (R/NW)       | NEXI       | Number of External Inputs. The CSPFT_HWFEAT.NEXI bit field identifies the number of external inputs (up to four).                                                                                                                                                                                                                                                                                                                                                   |
| 15:13 (R/NW)       | NCNTR      | Number of Counters. The CSPFT_HWFEAT.NCNTR bit field identifies the number of counters (up to four) that are configured using the counter registers.                                                                                                                                                                                                                                                                                                                |
| 3:0 (R/NW)         | NACMP      | Number of Pairs of Address Comparators. The CSPFT_HWFEAT.NACMP bit field identifies the number of pairs of address comparators as address range comparators (ARCs). In this case, two adjacent address comparators form the ARC, so you can use address comparators 1 and 2 to define the first ARC. An ARC matches when any instruction in the specified range is committed for execu- tion, regardless of whether the instruction passes its condition code test. |

Table 56-34: CSPFT\_HWFEAT Register Fields (Continued)

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

Figure 56-24: CSPFT\_LAR Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000023_90d54ff1dacd7efd1678952adf897ce3b9dafe41b761f7b121d28da25573a62e.png)

Table 56-35: CSPFT\_LAR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 31:0 (RX/W)        | VALUE      | Lock Access. Write 0xC5ACCE55 to the CSPFT_LAR.VALUE bit field to unlock. Write any oth- er value to lock. |

## Lock Status Register

The CSPFT\_LSR register is used to detect if the lock registers are implemented and if they are currently locked.

Figure 56-25: CSPFT\_LSR Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000024_f750d8d8a0b8dd0dd61a4ca63d3e95b4c6242a3b1207570696e0e400485dfa7e.png)

Table 56-36: CSPFT\_LSR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | LOCKED     | Lock Status. The CSPFT_LSR.LOCKED bit indicates whether the PFT is locked. 0 Writes are permitted 1 Locked. Writes are ignored                                                                                                                                              |
| 0 (R/NW)           | LOCKEN     | Locking Supported. The CSPFT_LSR.LOCKEN bit indicates whether the lock registers are implemented for this interface. 0 Locking is Not Required. This access is from an inter- face that ignores the lock registers. 1 Locking is Required. This access is from an interface |

## Peripheral ID0 Register

The CSPFT\_PID0 register holds peripheral identification information.

Figure 56-26: CSPFT\_PID0 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000025_86c78d2e06472b395c54f4d3d3ae22d38fbfaf541b02fcd7db5f40880d18b751.png)

Table 56-37: CSPFT\_PID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 7:0                | PARTNUM    | Part Number.                                                                 |
| (R/NW)             |            | The CSPFT_PID0.PARTNUM bit field holds the peripheral identification number. |

## Peripheral ID1 Register

The CSPFT\_PID1 register holds peripheral identification information.

Figure 56-27: CSPFT\_PID1 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000026_0ce2b0139774a2eb5a126c4162dfe08bb9dd8076fdf44dfbcd84c5f331150e3b.png)

Table 56-38: CSPFT\_PID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 7:4                | JEP106     | JEDEC JEP106 Manufacturer ID code.                                           |
| (R/NW)             |            |                                                                              |
| 3:0                | PARTNUM    | Part Number.                                                                 |
| (R/NW)             |            | The CSPFT_PID1.PARTNUM bit field holds the peripheral identification number. |

## Peripheral ID2 Register

The CSPFT\_PID2 register holds peripheral identification information.

Figure 56-28: CSPFT\_PID2 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000027_9935662969c3181cf3fda56b5802ffaa09a2048a82c852f7c93911f483f2a996.png)

Table 56-39: CSPFT\_PID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | REV        | Peripheral Revision.                                                                                        |
| 3 (R/NW)           | JEDECASGN  | A JEDEC Assigned Value is Used. The CSPFT_PID2.JEDECASGN bit indicates that a JEDEC assigned value is used. |
| 2:0 (R/NW)         | JEP106     | JEDEC JEP106 Manufacturer ID code.                                                                          |

## Peripheral ID3 Register

The CSPFT\_PID3 register holds peripheral identification information.

Figure 56-29: CSPFT\_PID3 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000028_709d9e9abb68888789ef446545432c3ecadcf0f4e5fcebe4e4fee0c2cede119d.png)

Table 56-40: CSPFT\_PID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration           |
|--------------------|------------|-----------------------------------|
| 7:4 (R/NW)         | REVAND     | Field to mark metal fix revision. |
| 3:0 (R/NW)         | CUSTMOD    | Customer Modified.                |

## Peripheral ID4 Register

The CSPFT\_PID4 register holds peripheral identification information.

Figure 56-30: CSPFT\_PID4 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000029_b57222cd78cc243487f181d66efbfe72dce615e110dd319da8cddd8d5ee1dc11.png)

Table 56-41: CSPFT\_PID4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | SIZE       | Number of 4K Blocks. The CSPFT_PID4.SIZE bit field contains the size of the component in 4K chunks minus 1 (for example 0=4K). |
| 3:0 (R/NW)         | JEOP106CC  | JEOP106 continuation code (number of leading 0x7Fs).                                                                           |

## Status Register

The CSPFT\_STAT register provides information about the current status of the trace and trigger logic.

Figure 56-31: CSPFT\_STAT Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000030_51fd74a34a96b61165ed264e601913abcd90332ba596f1e5785a57cf4a243e95.png)

Table 56-42: CSPFT\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | TRIG       | Trigger Bit. The CSPFT_STAT.TRIG bit is set when the trigger occurs, and prevents the trigger from being output until the CSPFT is next programmed. This bit is reset when the CSPFT_CTL.PB bit transitions from 1 to 0.                                                                                               |
| 2 (R/W)            | TSS        | Trace Start/Stop Bit Status. The CSPFT_STAT.TSS bit holds the current status of the trace start/stop resource. If = 1, it indicates that a trace start address has been matched, without a corresponding trace stop address match. This bit =0 when trace is restarted (the CSPFT_CTL.PB bit transitions from 1 to 0). |
| 1 (R/NW)           | PB         | Prog Bit Status. The CSPFT_STAT.PB bit indicates the current effective value of the CSPFT_CTL.PB bit. The program must wait for this bit to =1 before programming the CSPFT. (See the CSPFT_CTL.PB bit description).                                                                                                   |
| 0 (R/NW)           | OF         | Overflow. If the CSPFT_STAT.OF bit is =1, there is an overflow. This bit is cleared =0 when the trace is restarted ( CSPFT_CTL.PB transitions from 1 to 0)                                                                                                                                                             |

## Synchronization Frequency Register

The CSPFT\_SYNCFR register holds the trace synchronization frequency value.

Figure 56-32: CSPFT\_SYNCFR Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000031_049b8d163affd37f73639519175a9dacaf60ebaff1eeb6cc6602b1ef89e1eacf.png)

Table 56-43: CSPFT\_SYNCFR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | SFREQ      | Synchronization frequency. The CSPFT_SYNCFR.SFREQ bit field is the number of 128-byte blocks of trace da- ta after which you want to drop an address synchronization packet. If the circular buf- fer size is 16k, ensure that there are a few A-syncs in the buffer, so setting this to 16 means that every 2k there is an A-Sync packet. |

## TraceEnable Control Register

The CSPFT\_TECTL register controls the start stop logic, whether resources specified are used for include or exclude, and specifies the address range comparators to use.

Figure 56-33: CSPFT\_TECTL Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000032_6e7f9ceb9b9ad7f587153f5bdbe69acdcf493dd0d11d230fcd22f5f3eb04b29d.png)

Table 56-44: CSPFT\_TECTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | TSSCENA    | Trace Start and Stop Control Enable. 0 Tracing is not affected by the trace start/stop logic                                                                         |
| 24 (R/W)           | EXCL       | Include and Exclude Control. 0 Include. The specified address range comparators indi- cate the regions where tracing can occur. When outside                         |
| 7:0 (R/W)          | ARCS       | Address Range Comparator Select Bits. When a bit in the CSPFT_TECTL.ARCS bit field is set to 1, it selects an address range comparator, for include/exclude control. |

## TraceEnable Event Register

The CSPFT\_TEEVENT register defines the T raceEnable enabling event.

Figure 56-34: CSPFT\_TEEVENT Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000033_b426c807681b07b6af2916b3f5aac06211d631b77112fbab77cfc29ed7af8f48.png)

Table 56-45: CSPFT\_TEEVENT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:14 (R/W)        | FUNC       | Function. The CSPFT_TEEVENT.FUNC bit field specifies the logical operation that combines the two resources that define the event.                                                                                              | Function. The CSPFT_TEEVENT.FUNC bit field specifies the logical operation that combines the two resources that define the event.                                                                                              |
| 16:14 (R/W)        | FUNC       | 0                                                                                                                                                                                                                              | A                                                                                                                                                                                                                              |
| 16:14 (R/W)        | FUNC       | 1                                                                                                                                                                                                                              | NOT(A)                                                                                                                                                                                                                         |
| 16:14 (R/W)        | FUNC       | 2                                                                                                                                                                                                                              | A AND B                                                                                                                                                                                                                        |
| 16:14 (R/W)        | FUNC       | 3                                                                                                                                                                                                                              | NOT(A) AND B                                                                                                                                                                                                                   |
| 16:14 (R/W)        | FUNC       | 4                                                                                                                                                                                                                              | NOT(A) AND NOT(B)                                                                                                                                                                                                              |
| 16:14 (R/W)        | FUNC       | 5                                                                                                                                                                                                                              | A OR B                                                                                                                                                                                                                         |
| 16:14 (R/W)        | FUNC       | 6                                                                                                                                                                                                                              | NOT(A) OR B                                                                                                                                                                                                                    |
| 16:14 (R/W)        | FUNC       | 7                                                                                                                                                                                                                              | NOT(A) OR NOT(B)                                                                                                                                                                                                               |
| 13:7 (R/W)         | RESB       | Resource B. The CSPFT_TEEVENT.RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_TEEVENT.FUNC field. (See CSPFT_TEEVENT.RESA for list of possible values). | Resource B. The CSPFT_TEEVENT.RESB bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_TEEVENT.FUNC field. (See CSPFT_TEEVENT.RESA for list of possible values). |
| 6:0 (R/W)          | RESA       | Resource A. The CSPFT_TEEVENT.RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_TEEVENT.FUNC field.                                                       | Resource A. The CSPFT_TEEVENT.RESA bit field specifies one of the two resources that can be combined by the logical operation specified in the CSPFT_TEEVENT.FUNC field.                                                       |
| 6:0 (R/W)          | RESA       | 0                                                                                                                                                                                                                              | Single Addr Comparator 0                                                                                                                                                                                                       |

Table 56-45: CSPFT\_TEEVENT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |    | Description/Enumeration   |
|--------------------|------------|----|---------------------------|
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
|                    |            | 90 | Context ID Comparator 2   |

Table 56-45: CSPFT\_TEEVENT Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration                |
|-----------|------------|---------------------------|----------------------------------------|
| (Access)  |            |                           |                                        |
|           |            |                        95 | TraceEnable Start/Stop Resource 0 or 1 |
|           |            |                        96 | External Inputs 0                      |
|           |            |                        97 | External Inputs 1                      |
|           |            |                        98 | External Inputs 2                      |
|           |            |                        99 | External Inputs 3                      |
|           |            |                       110 | Trace Prohibited                       |
|           |            |                       111 | Always TRUE                            |

## CoreSight Trace ID Register

The CSPFT\_TRACEIDR register defines the 7-bit trace ID, for output to the trace bus.

Figure 56-35: CSPFT\_TRACEIDR Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000034_13c5dd70ef17a28d717e04175671355007c00c97f1e9f74e883cc7a424dca25d.png)

Table 56-46: CSPFT\_TRACEIDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | TID        | Trace ID. Defines the 7-bit trace ID, for output to the trace bus. Used in systems where multiple trace sources are present and tracing simultaneously. For example, when outputs trace onto the AMBA 3 Advanced Trace Bus, a unique ID is required for each trace source. |

## Trigger Event Register

The CSPFT\_TRIGGER register defines the event that controls the trigger. This event creates the trigger output signal that is in the ATCLK domain.

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000035_d3495fe8edc83f8d7d8d3f1f0d991309687d414b3c92a85d0781b882edc68f82.png)

Function

Figure 56-36: CSPFT\_TRIGGER Register Diagram

Table 56-47: CSPFT\_TRIGGER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:14 (R/W)        | FUNC       | Function. Specifies the logical operation that combines the two resources that define the event                                                                                        |
| 13:7 (R/W)         | RESB       | 7 NOT(A) OR NOT(B) Resource B. Specifies one of the two resources that can be combined by the logical operation fied in the CSPFT_TRIGGER.FUNC field. (See CSPFT_TRIGGER.RESA for list |
| 6:0 (R/W)          | RESA       | speci- of possible values.) Resource A. Specifies one of the two resources that can be combined by the logical operation                                                               |
|                    |            | speci- fied in the CSPFT_TRIGGER.FUNC field                                                                                                                                            |

Table 56-47: CSPFT\_TRIGGER Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |    | Description/Enumeration   |
|--------------------|------------|----|---------------------------|
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
|                    |            | 90 | Context ID Comparator 2   |

Table 56-47: CSPFT\_TRIGGER Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration                |
|-----------|------------|---------------------------|----------------------------------------|
| (Access)  |            |                           |                                        |
|           |            |                        95 | TraceEnable Start/Stop Resource 0 or 1 |
|           |            |                        96 | External Inputs 0                      |
|           |            |                        97 | External Inputs 1                      |
|           |            |                        98 | External Inputs 2                      |
|           |            |                        99 | External Inputs 3                      |
|           |            |                       110 | Trace Prohibited                       |
|           |            |                       111 | Always TRUE                            |

## TraceEnable Start/Stop Control Register

The CSPFT\_TSSCTL register specifies the single address comparators that hold the trace start and stop addresses.

Figure 56-37: CSPFT\_TSSCTL Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000036_1a31bc98356a867fecaf88e2dc36f4ae969f7c15aefc1737fafda8d1e646a3f5.png)

Table 56-48: CSPFT\_TSSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | STOP       | Stop Address Comparator Select Bits. When a bit is set to 1, it selects a single address comparator as a stop address for the TraceEnable start/stop block. |
|                    |            | 0 disabled                                                                                                                                                  |
|                    |            | 1 Address Comparator 1                                                                                                                                      |
|                    |            | 2 Address Comparator 2                                                                                                                                      |
|                    |            | 4 Address Comparator 3                                                                                                                                      |
|                    |            | 8 Address Comparator 4                                                                                                                                      |
|                    |            | 16 Address Comparator 5                                                                                                                                     |
|                    |            | 32 Address Comparator 6                                                                                                                                     |
|                    |            | 64 Address Comparator 7                                                                                                                                     |
|                    |            | 128 Address Comparator 8                                                                                                                                    |
|                    |            | 256 Address Comparator 9                                                                                                                                    |
|                    |            | 512 Address Comparator 10                                                                                                                                   |
|                    |            | 1024 Address Comparator 11                                                                                                                                  |
|                    |            | 2048 Address Comparator 12                                                                                                                                  |
|                    |            | 4096 Address Comparator 13                                                                                                                                  |
|                    |            | 8192 Address Comparator 14                                                                                                                                  |
|                    |            | 16384 Address Comparator 15                                                                                                                                 |

Table 56-48: CSPFT\_TSSCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 32768                                                                                                                                                         | Address Comparator 16                                                                                                                                         |
| 15:0 (R/W)         | START      | Start Address Comparator Select Bits. When a bit is set to 1, it selects a single address comparator as a start address for the TraceEnable start/stop block. | Start Address Comparator Select Bits. When a bit is set to 1, it selects a single address comparator as a start address for the TraceEnable start/stop block. |
|                    |            | 0                                                                                                                                                             | Disabled                                                                                                                                                      |
|                    |            | 1                                                                                                                                                             | Address Comparator 1                                                                                                                                          |
|                    |            | 2                                                                                                                                                             | Address Comparator 2                                                                                                                                          |
|                    |            | 4                                                                                                                                                             | Address Comparator 3                                                                                                                                          |
|                    |            | 8                                                                                                                                                             | Address Comparator 4                                                                                                                                          |
|                    |            | 16                                                                                                                                                            | Address Comparator 5                                                                                                                                          |
|                    |            | 32                                                                                                                                                            | Address Comparator 6                                                                                                                                          |
|                    |            | 64                                                                                                                                                            | Address Comparator 7                                                                                                                                          |
|                    |            | 128                                                                                                                                                           | Address Comparator 8                                                                                                                                          |
|                    |            | 256                                                                                                                                                           | Address Comparator 9                                                                                                                                          |
|                    |            | 512                                                                                                                                                           | Address Comparator 10                                                                                                                                         |
|                    |            | 1024                                                                                                                                                          | Address Comparator 11                                                                                                                                         |
|                    |            | 2048                                                                                                                                                          | Address Comparator 12                                                                                                                                         |
|                    |            | 4096                                                                                                                                                          | Address Comparator 13                                                                                                                                         |
|                    |            | 8192                                                                                                                                                          | Address Comparator 14                                                                                                                                         |
|                    |            | 16384                                                                                                                                                         | Address Comparator 15                                                                                                                                         |
|                    |            | 32768                                                                                                                                                         | Address Comparator 16                                                                                                                                         |

## ADSP-SC58x TAPC Register Descriptions

TAPC (TAPC) contains the following registers.

Table 56-49: ADSP-SC58x TAPC Register List

| Name          | Description                 |
|---------------|-----------------------------|
| TAPC_DBGCTL   | Debug Control Register      |
| TAPC_IDCODE   | IDCODE Register             |
| TAPC_SDBGKEY0 | Secure Debug Key 0 Register |
| TAPC_SDBGKEY1 | Secure Debug Key 1 Register |

Table 56-49: ADSP-SC58x TAPC Register List (Continued)

| Name              | Description                       |
|-------------------|-----------------------------------|
| TAPC_SDBGKEY2     | Secure Debug Key 2 Register       |
| TAPC_SDBGKEY3     | Secure Debug Key 3 Register       |
| TAPC_SDBGKEY_CTL  | Secure Debug Key Control Register |
| TAPC_SDBGKEY_STAT | Secure Debug Key Status Register  |
| TAPC_USERCODE     | USERCODE Register                 |

## Debug Control Register

Figure 56-38: TAPC\_DBGCTL Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000037_c7184282bd276469d208d3601596df508da892e7317c8bef73b6fac731b4ead4.png)

Table 56-50: TAPC\_DBGCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration             |
|--------------------|-------------|-------------------------------------|
| 15 (R/W)           | SPIDENTRACE | SPIDEN for Coresight trace modules. |
| 14 (R/W)           | NIDENTRACE  | NIDEN for Coresight trace modules.  |
| 13 (R/W)           | DBGENTRACE  | DBGEN for Coresight trace modules.  |
| 12 (R/W)           | NIDENCTISYS | NIDEN for System CTI.               |
| 11 (R/W)           | DBGENCTISYS | DBGEN for System CTI.               |
| 10 (R/W)           | SPIDENSTM   | SPIDEN for STM.                     |

Table 56-50: TAPC\_DBGCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
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

Figure 56-39: TAPC\_IDCODE Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000038_8892f924079b0382ca70d078ba7290fbd9fd7ba341d98cbb4c2ac72448764a34.png)

Table 56-51: TAPC\_IDCODE Register Fields

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

Figure 56-40: TAPC\_SDBGKEY0 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000039_51e202ebd107af91aeba963298126b0442bb2c351ef6db2f2a2968ab4c6341ac.png)

Table 56-52: TAPC\_SDBGKEY0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | SDBGKEY. The TAPC_SDBGKEY0.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |

## Secure Debug Key 1 Register

The TAPC\_SDBGKEY1 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. See the TAPC\_SDBGKEY0 register description for more information.

Figure 56-41: TAPC\_SDBGKEY1 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000040_8eef975830a4abf9f62c0849ff8517c359daabeee7de0fa6ddfac25d1e4193d6.png)

Table 56-53: TAPC\_SDBGKEY1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | SDBGKEY. The TAPC_SDBGKEY1.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |
| (R/W)              |            |                                                                                                                                                                      |

## Secure Debug Key 2 Register

The TAPC\_SDBGKEY2 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. See the TAPC\_SDBGKEY0 register description for more information.

Figure 56-42: TAPC\_SDBGKEY2 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000041_69a163dbf4cd567282970e3344b70bbc001437edf7b03f231ed1d1647e469c06.png)

Table 56-54: TAPC\_SDBGKEY2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | SDBGKEY. The TAPC_SDBGKEY2.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |
| (R/W)              |            |                                                                                                                                                                      |

## Secure Debug Key 3 Register

The TAPC\_SDBGKEY3 register allows a locked part to unlock debug access through the JTAG or SWD interfaces. See the TAPC\_SDBGKEY0 register description for more information.

Figure 56-43: TAPC\_SDBGKEY3 Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000042_922d023494e8d0d45c5ac08b3774204b2a93ea35fc2c0f69e2c0bf8d6867233c.png)

Table 56-55: TAPC\_SDBGKEY3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | SDBGKEY. The TAPC_SDBGKEY3.VALUE bit field holds the value of the key against which a matching key provided by the debug user is compared to enable a debug session. |
| (R/W)              |            |                                                                                                                                                                      |

## Secure Debug Key Control Register

Figure 56-44: TAPC\_SDBGKEY\_CTL Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000043_f1e030c26b2acfa41fb561c358713ca0c8ebb9dd687ff012981cc1111cb248f5.png)

Table 56-56: TAPC\_SDBGKEY\_CTL Register Fields

|   Bit No. (Access) | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
|                  0 | VALID      | SDBGKEY Valid bit.        |

## Secure Debug Key Status Register

Figure 56-45: TAPC\_SDBGKEY\_STAT Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000044_4f162df0aeb7909c788f026ffd00ba7a4c154e681ab4a2921c6dbb35f0e9c961.png)

Table 56-57: TAPC\_SDBGKEY\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/NW)           | FAIL       | SDBGKEY MATCH FAILED.     |
| 0 (R/NW)           | PASS       | SDBGKEY MATCH PASSED.     |

## USERCODE Register

The TAPC\_USERCODE register

Figure 56-46: TAPC\_USERCODE Register Diagram

![Image](59_System_Debug_and_Trace_Unit_(DBG)_artifacts/image_000045_4c0a506f3da2c139fde0405a6cd57101d6c223aab54af094ada6fe529404979f.png)

Table 56-58: TAPC\_USERCODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | USERCODE[31:0].           |
| (R/NW)             |            |                           |