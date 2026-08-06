## 55   System Watchpoint Unit (SWU)

The system watchpoint unit (SWU) is a single module used for transaction monitoring. The SWU is attached to each system slave through the system crossbar interface and provides ports for all address channel signals for the system crossbar. The SWU does not have ports for the read/write data channel signals or the low-power interface signals.

Each SWU contains four match groups of registers with associated hardware. These four SWU match groups operate independently, but share common event (interrupt and trigger) outputs. Each match group can monitor either the write or read address channel and can operate in either watchpoint mode or bandwidth mode.

## SWU Features

The system watchpoint unit has the following features.

- Four independent match groups for each SWU
- Each match group can operate in either bandwidth mode or watchpoint mode
- In addition to the read and write address channels, the SWU monitors read and write ID, VALID and READY signals for data channels.

## SWU Functional Description

This section describes the function of the SWU match block, interface block, and MMR block.

## ADSP-SC58x SWU Register List

The System Watchpoint Unit (SWU) provides debug and development support through flexible transaction level and bandwidth monitoring and associated event triggering. The SWU can generate events based on monitoring transactions at the system slaves through watchpoint-match groups. The SWU also provides watchpoint event status reporting, a global lock, and processor reset capability. A set of registers governs SWU operations. For more information on SWU functionality, see the SWU register descriptions.

Table 55-1: ADSP-SC58x SWU Register List

| Name        | Description                  |
|-------------|------------------------------|
| SWU_CNT[n]  | Count Register n             |
| SWU_CTL[n]  | Control Register n           |
| SWU_CUR[n]  | Current Register n           |
| SWU_GCTL    | Global Control Register      |
| SWU_GSTAT   | Global Status Register       |
| SWU_HIST[n] | Bandwidth History Register n |
| SWU_ID[n]   | ID Register n                |
| SWU_LA[n]   | Lower Address Register n     |
| SWU_TARG[n] | Target Register n            |
| SWU_UA[n]   | Upper Address Register n     |

## ADSP-SC58x SWU Interrupt List

Table 55-2: ADSP-SC58x SWU Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|            225 | SWU0_EVT  | SWU0 Event    | None          |               |
|            226 | SWU2_EVT  | SWU2 Event    | None          |               |
|            227 | SWU1_EVT  | SWU1 Event    | None          |               |
|            228 | SWU4_EVT  | SWU4 Event    | None          |               |
|            229 | SWU3_EVT  | SWU3 Event    | None          |               |
|            230 | SWU6_EVT  | SWU6 Event    | None          |               |
|            231 | SWU5_EVT  | SWU5 Event    | None          |               |
|            232 | SWU7_EVT  | SWU7 Event    | None          |               |
|            233 | SWU8_EVT  | SWU8 Event    | None          |               |
|            234 | SWU9_EVT  | SWU9 Event    | None          |               |
|            235 | SWU10_EVT | SWU10 Event   | None          |               |
|            236 | SWU11_EVT | SWU11 Event   | None          |               |
|            237 | SWU12_EVT | SWU12 Event   | None          |               |
|            238 | SWU13_EVT | SWU13 Event   | None          |               |
|            239 | SWU14_EVT | SWU14 Event   | None          |               |
|            240 | SWU15_EVT | SWU15 Event   | None          |               |

## ADSP-SC58x SWU Trigger List

Table 55-3: ADSP-SC58x SWU Trigger List Masters

|   Trigger ID | Name      | Description   | Sensitivity   |
|--------------|-----------|---------------|---------------|
|           99 | SWU0_EVT  | SWU0 Event    | None          |
|          100 | SWU2_EVT  | SWU2 Event    | None          |
|          101 | SWU1_EVT  | SWU1 Event    | None          |
|          102 | SWU4_EVT  | SWU4 Event    | None          |
|          103 | SWU3_EVT  | SWU3 Event    | None          |
|          104 | SWU6_EVT  | SWU6 Event    | None          |
|          105 | SWU5_EVT  | SWU5 Event    | None          |
|          106 | SWU7_EVT  | SWU7 Event    | None          |
|          107 | SWU8_EVT  | SWU8 Event    | None          |
|          108 | SWU9_EVT  | SWU9 Event    | None          |
|          109 | SWU10_EVT | SWU10 Event   | None          |
|          110 | SWU11_EVT | SWU11 Event   | None          |
|          111 | SWU12_EVT | SWU12 Event   | None          |
|          112 | SWU13_EVT | SWU13 Event   | None          |
|          113 | SWU14_EVT | SWU14 Event   | None          |
|          114 | SWU15_EVT | SWU15 Event   | None          |
|          115 | SWU0_DBG  | SWU0 Debug    | Edge          |
|          116 | SWU2_DBG  | SWU2 Debug    | Edge          |
|          117 | SWU1_DBG  | SWU1 Debug    | Edge          |
|          118 | SWU4_DBG  | SWU4 Debug    | Edge          |
|          119 | SWU3_DBG  | SWU3 Debug    | Edge          |
|          120 | SWU6_DBG  | SWU6 Debug    | Edge          |
|          121 | SWU5_DBG  | SWU5 Debug    | Edge          |
|          122 | SWU7_DBG  | SWU7 Debug    | Edge          |
|          123 | SWU8_DBG  | SWU8 Debug    | Edge          |
|          124 | SWU9_DBG  | SWU9 Debug    | Edge          |
|          125 | SWU10_DBG | SWU10 Debug   | Edge          |
|          126 | SWU11_DBG | SWU11 Debug   | Edge          |
|          127 | SWU12_DBG | SWU12 Debug   | Edge          |
|          128 | SWU13_DBG | SWU13 Debug   | Edge          |

Table 55-3: ADSP-SC58x SWU Trigger List Masters (Continued)

|   Trigger ID | Name      | Description   | Sensitivity   |
|--------------|-----------|---------------|---------------|
|          129 | SWU14_DBG | SWU14 Debug   | Edge          |
|          130 | SWU15_DBG | SWU15 Debug   | Edge          |

Table 55-4: ADSP-SC58x SWU Trigger List Slaves

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|          119 | SWU0_EN  | SWU0 Enable   | Pulse         |
|          120 | SWU2_EN  | SWU2 Enable   | Pulse         |
|          121 | SWU1_EN  | SWU1 Enable   | Pulse         |
|          122 | SWU4_EN  | SWU4 Enable   | Pulse         |
|          123 | SWU3_EN  | SWU3 Enable   | Pulse         |
|          124 | SWU6_EN  | SWU6 Enable   | Pulse         |
|          125 | SWU5_EN  | SWU5 Enable   | Pulse         |
|          126 | SWU7_EN  | SWU7 Enable   | Pulse         |
|          127 | SWU8_EN  | SWU8 Enable   | Pulse         |
|          128 | SWU9_EN  | SWU9 Enable   | Pulse         |
|          129 | SWU10_EN | SWU10 Enable  | Pulse         |
|          130 | SWU11_EN | SWU11 Enable  | Pulse         |
|          131 | SWU12_EN | SWU12 Enable  | Pulse         |
|          132 | SWU13_EN | SWU13 Enable  | Pulse         |
|          133 | SWU14_EN | SWU14 Enable  | Pulse         |
|          134 | SWU15_EN | SWU15 Enable  | Pulse         |

## SWU Definitions

The following definitions are helpful when using the SWU module.

## Watchpoint Mode

Mode in which transactions are recognized on an exact match. Actions can be configured to be taken after a specified number of matches have occurred.

## Bandwidth Mode

Mode in which transactions are recognized and counted inside sampling window.

## SWU Architectural Concepts

The information in this section provides basic module design concepts.

## SWU-to-SCB Interface

The SWU system crossbar interface block latches all transactions on the system crossbar read and write address channels when the SWU\_GCTL.EN register enable bit is set.

## SWU Block Diagram

The System Watchpoint Unit Top-Level Block Diagram figure shows the SWU block diagram.

Figure 55-1: System Watchpoint Unit Top-Level Block Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000000_0d68537ecdc19bd4069799993ab5e6f253a9dd656574da15d380c9838c952682.png)

## SCB Interface Block

The SWU system crossbar (SCB) latches all transactions on the SCB read and write address channels when the SWU\_GCTL.EN bit is set.

## MMR Interface Block

The SWU MMR block contains the peripheral bus interface and the SWU MMR registers. It also merges all interrupt requests and events from each match block into common outputs.

## SWU Operating Modes

There are two operating modes supported by the SWU: bandwidth mode and watchpoint mode.

## Bandwidth Mode

In bandwidth mode, the SWU module counts transactions which match the properties specified in the SWU\_CTL[n] register during a sampling window determined by the respective SWU\_CNT[n] register. At the end of the sampling window, the SWU stores results in the SWU\_HIST[n] register. If the sampled bandwidth falls outside a programmed range, then the programmed action occurs.

## Watchpoint Mode

In watchpoint mode, if the SWU\_CTL[n].CNTEN bit is set, the SWU module decrements the SWU\_CUR[n] register for each match, until it equals zero, at which point any programmed actions occur. The SWU\_CUR[n] register is then reloaded from the SWU\_CNT[n] register (if the SWU\_CTL[n].CNTRPTEN bit is set), and the cycle repeats. If the SWU\_CTL[n].CNTRPTEN bit is not set, any programmed actions happen on every match.

## Match Block

There are four match blocks for each SWU. Each SWU match block can monitor either the read or write address channel, selected by the SWU\_CTL[n].DIR bit. The SWU match block can operate in either watchpoint or bandwidth mode, as selected by the SWU\_CTL[n].BWEN bit.

In either mode, the SWU match block can be programmed to match based on address (exact, inclusive or exclusive range), ID (with masking), security, and lock type. All enabled matches are AND'ed together to determine a match.

## Scaling

Scaling allows the SWU to count more transactions by scaling the number of transactions and the number of clock cycles in bandwidth window (CNTn register) by 10,100 or 1000. This functionality is applicable only in bandwidth mode ( SWU\_CTL[n].BWEN ==1).

Consider a case where the SWU\_TARG[n].BWMAX bit field is programmed to 2. In the absence of any scaling, bandwidth overflow occurs when the SWU\_CUR[n].CURBW bit field value &gt; the SWU\_TARG[n].BWMAX bit field value (2). With scaling set to 1:100 and after 275 transactions, the SWU\_CUR[n].CURBW value is still 2 and equal to SWU\_TARG[n].BWMAX . This event triggers a bandwidth overflow as the actual number of transactions is greater than 200 (2 × 100). The code can be rerun with a smaller scaling selected to analyze the cause for the overflow.

The counter increments after every group of N transactions with scaling enabled, where N is either 10,100 or 1000. (For N = 10, 0-9 transactions == 0 scaled transaction, 10-19 transactions == 1 scaled transaction, 20-29 transactions == 2 scaled transactions, and so on).

Fractional counts with scaling enabled are discarded and not rolled over from one bandwidth window to the next. For example, consider a case where scaling by 1000 is configured and the first window has 1200 transactions. The second window has 2800 transactions. The bandwidth for the first window is read as 1 and the second as 2. The 200 transactions from the first window do not get carried forward to the next window.

Do not use scaling by 10 with the SWU\_CTL[n].BLENINC bit enabled if any of the masters accessing the slave can launch a transaction of burst length 16.

## SWU Event Control

The SWU can generate the following events when a match occurs and when the event is enabled by configuring the proper bits in the control register.

1. Trace Message

2. Trigger
3. Interrupt request
4. Debug

## SWU Interrupts

All interrupt requests and events from each match block are merged into common outputs.

## SWU Status and Errors

SWU status and errors are reported in the SWU\_GSTAT register. The SWU records an address error when a write or read attempt is made to the MMR address space of the SWU and the register does not exist. This error is the only one the SWU records. The register contains bits that perform the following functions.

- Indicate whether a particular match group sampled a transaction that is below a minimum target or above a maximum target in bandwidth mode.
- Indicate whether a watchpoint match occurred for each match group.
- Indicate whether an interrupt request was triggered due to a match event from one of the match groups.

## Triggers

The SWU can be either a trigger master or a trigger slave depending on the trigger routing unit (TRU) configuration. As a trigger master, programs must set the SWU\_CTL[n].TRGEN bit so that when a match condition is met, a trigger event is generated. Each SWU in the system can also be a trigger slave when mapped as one in the TRU.

When the SWU is a slave, a trigger event activates the SWU by automatically setting the SWU\_GCTL.EN bit. Since the SWU can be automatically enabled through a trigger event, programs must pre-configure the SWU before enabling the TRU. Furthermore, although a trigger event can enable the SWU as a slave, to disable the SWU, programs must manually clear the SWU\_GCTL.EN bit.

## SWU Programming Model

Program the appropriate registers to use the SWU. Each control register configures aspects such as:

- The direction of monitoring (reads or writes)
- Whether SWU uses bandwidth mode or watchpoint mode
- The setup of events that are triggered when a condition is met while monitoring using the SWU

Configure supplemental registers such as the lower ( SWU\_LA[n] ) and upper ( SWU\_UA[n] ) address boundaries before enabling the SWU.

Once the SWU has been enabled and the monitoring conditions are met, events are generated when configured.

The global status register ( SWU\_GSTAT ) can be read to observe the status of the units.

The SWU Logical Flow diagram shows the logical program flow of the SWU.

Figure 55-2: SWU Logical Flow

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000001_df3331818313bf6c74b58ed1ff25115f3b8653a5ea8fee7fca30e88db142802a.png)

## SWU Mode Configuration

The following sections show the steps for configuring SWU bandwidth mode and watchpoint mode.

## Configuring the SWU for Bandwidth Mode

In bandwidth mode, the SWU counts transactions which match during a sampling window. At the end of the sampling window, the SWU stores the results. An action can be taken if the sampled bandwidth goes above or falls below a programmed range.

1. Configure the SWU\_CTL[n].DIR bit to test the match on writes or reads.
2. Configure the SWU\_CTL[n].ACMPM bits to address comparisons, exact match, matches inside a range or matches outside a range.
3. If ID comparison is desired, set the SWU\_CTL[n].IDCMPEN bit.
4. Set the SWU\_CTL[n].BLENINC bit to increment by burst length or clear it to increment by 1.

5. Configure the SWU\_CTL[n].MAXACT and SWU\_CTL[n].MINACT bits to enable actions taken when the bandwidth goes above the maximum, or falls below the minimum, respectively.
6. Set the SWU\_CTL[n].BWEN =1 to enable bandwidth mode.
7. Program the lower address register, SWU\_LA[n] , and upper address register, SWU\_UA[n] , to define the memory range for comparison.
8. If ID comparison is enabled, program the ID register, SWU\_ID[n] .
9. Program the count register, SWU\_CNT[n] , with the number of clock cycles for which the SWU counts the number of matches.
10. If the SWU is set to respond when the bandwidth measurement underflows or overflows, program the min and max values into the SWU\_TARG[n] register.
11. Enable the SWU

The SWU counts the number of matches in a pre-defined number of clock cycles as programmed. As an option, it can define lower and upper limits. If the matches fall outside the limits, an action can be taken.

## Configuring the SWU for Watchpoint Mode

In watchpoint mode, the SWU can trigger a programmed action after every match or after a number of matches. This sequence can be automatically reset.

1. Set the SWU\_CTL[n].DIR bit to test the match on writes or reads.
2. Configure the SWU\_CTL[n].ACMPM bits for address comparisons, exact match, matches inside a range or matches outside a range.
3. If ID comparison is desired, set the SWU\_CTL[n].IDCMPEN .
4. Set the SWU\_CTL[n].CNTEN bit to enable the events to be triggered when the count decrements to zero.
5. If needed, set the SWU\_CTL[n].CNTRPTEN bit to automatically reload the counter after it has decremented to zero to start another match sequence.
6. Clear the SWU\_CTL[n].BWEN = 0 to configure watchpoint mode.
7. Configure the lower address register, SWU\_LA[n] , and upper address register, SWU\_UA[n] , to define the memory range for comparison.
8. If ID comparison is enabled, configure the ID register, SWU\_ID[n] .
9. Configure the count register, SWU\_CNT[n] , to determine how many matches occur before the watchpoint group responds.
10. Enable the SWU.

The SWU detects and counts down the number of match occurrences. When the counter expires, an action is taken.

## ADSP-SC58x SWU Register Descriptions

System Watchpoint Unit (SWU) contains the following registers.

Table 55-5: ADSP-SC58x SWU Register List

| Name        | Description                  |
|-------------|------------------------------|
| SWU_CNT[n]  | Count Register n             |
| SWU_CTL[n]  | Control Register n           |
| SWU_CUR[n]  | Current Register n           |
| SWU_GCTL    | Global Control Register      |
| SWU_GSTAT   | Global Status Register       |
| SWU_HIST[n] | Bandwidth History Register n |
| SWU_ID[n]   | ID Register n                |
| SWU_LA[n]   | Lower Address Register n     |
| SWU_TARG[n] | Target Register n            |
| SWU_UA[n]   | Upper Address Register n     |

## Count Register n

The SWU count registers ( SWU\_CNT[n] ) contain a 16-bit count field ( SWU\_CNT[n].COUNT ) whose usage differs depending on the mode of the watchpoint group. In bandwidth mode, the SWU\_CNT[n].COUNT field value defines the number of clock cycles in a bandwidth period. In watchpoint mode, when the cycle count is enabled, the SWU\_CNT[n].COUNT field value determines how many matches occur before the watchpoint group takes action.

Figure 55-3: SWU\_CNT[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000002_9d02d7eb7c4fcdb43f9957c51915e8d16dead748da9f7998e6405a9e7c2381f3.png)

Table 55-6: SWU\_CNT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | COUNT      | Count. The SWU_CNT[n].COUNT field value defines the number of clock cycles in a band- width period. In watchpoint mode, when the cycle count is enabled, the SWU_CNT[n].COUNT field value determines how many matches occur before the watchpoint group takes action. |

## Control Register n

The SWU control registers ( SWU\_CTL[n] ) contain watchpoint attribute controls for all four watchpoint groups. These controls include enabling watchpoints, selecting the transaction direction for match, selecting address comparison mode, enabling ID comparison, enabling security comparison, enabling locked comparison, enabling cycle count, enabling count repeat, enabling debug events, enabling interrupts, enabling triggers, enabling trace messages, enabling bandwidth mode, selecting the burst length increment, and enabling bandwidth underflow and overflow detection.

Figure 55-4: SWU\_CTL[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000003_9e8a84d441d3455e8018c980699a4412f5a99af6128299e1802d50edc60ac8b1.png)

Table 55-7: SWU\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | DATACNTMODE  | Data Channel Monitor. The SWU_CTL[n].DATACNTMODE bit determines whether an address channel or a data channel is monitored. Note that in data channel only ID,READY and VALID signals are monitored and hence other comparisons (Address, Lock, Secure) will be ig- nored even if enabled. SWU_CTL[n].STALLCNTMODE and SWU_CTL[n].DIR can be used in conjunction with this bit |
| 24 (R/W)           | DATACNTMODE  | 0 Monitor address channel                                                                                                                                                                                                                                                                                                                                                     |
| 24 (R/W)           | DATACNTMODE  | 1 Monitor data channel                                                                                                                                                                                                                                                                                                                                                        |
| 23:21 (R/W)        | SCALE        | Scale the number of transactions. The SWU_CTL[n].SCALE bit field allows a program to count more transactions by scaling the number of transactions and also the number of clock cycles in the band- width window ( SWU_CNT[n] register) by 10,100 or 1000. This is applicable only in bandwidth mode ( SWU_CTL[n].BWEN ==1).                                                  |
| 23:21 (R/W)        | SCALE        | 0 No scaling                                                                                                                                                                                                                                                                                                                                                                  |
| 23:21 (R/W)        | SCALE        | 1 1:10                                                                                                                                                                                                                                                                                                                                                                        |
| 23:21 (R/W)        | SCALE        | 2 1:100                                                                                                                                                                                                                                                                                                                                                                       |
| 23:21 (R/W)        | SCALE        | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                    |
| 23:21 (R/W)        | SCALE        | 4 1:1000                                                                                                                                                                                                                                                                                                                                                                      |
| 23:21 (R/W)        | SCALE        | 5-7 Reserved                                                                                                                                                                                                                                                                                                                                                                  |
| 20 (R/W)           | STALLCNTMODE | Stall Count Mode. The SWU_CTL[n].STALLCNTMODE bit determines whether the number of stalls are counted or whether the number of transactions are counted. This feature is only valid in bandwidth mode.                                                                                                                                                                        |
| 20 (R/W)           | STALLCNTMODE | 0 Count number of transactions                                                                                                                                                                                                                                                                                                                                                |
| 20 (R/W)           | STALLCNTMODE | 1 Count number of stalls                                                                                                                                                                                                                                                                                                                                                      |
| 19 (R/W)           | MAXACT       | Action for Bandwidth Above Maximum. Each SWU_CTL[n].MAXACT bit determines whether a watchpoint group takes ac- tion on bandwidth overflow. This feature is only valid in bandwidth mode.                                                                                                                                                                                      |
| 19 (R/W)           | MAXACT       | 0 No Action                                                                                                                                                                                                                                                                                                                                                                   |
| 19 (R/W)           | MAXACT       | 1 Take Action                                                                                                                                                                                                                                                                                                                                                                 |
| 18 (R/W)           | MINACT       | Action for Bandwidth Below Minimum. Each SWU_CTL[n].MINACT bit determines whether a watchpoint group takes ac- tion on bandwidth underflow. This feature is only valid in bandwidth mode.                                                                                                                                                                                     |
| 18 (R/W)           | MINACT       | 0 No Action                                                                                                                                                                                                                                                                                                                                                                   |
| 18 (R/W)           | MINACT       | 1 Take Action                                                                                                                                                                                                                                                                                                                                                                 |

Table 55-7: SWU\_CTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | BLENINC    | Increment Bandwidth Count by Burst Length. Each SWU_CTL[n].BLENINC bit controls how a watchpoint group's bandwidth count is incremented in the SWU_CUR[n] register's SWU_CUR[n].CURBW field. If the SWU_CTL[n].BLENINC bit is cleared (= 0), the SWU increments the band- width count by 1 for each matching transaction. If the SWU_CTL[n].BLENINC bit is set (=1), the SWU increments the bandwidth count by the burst length of the trans- action for each matching transaction. This feature is only valid for bandwidth mode ( SWU_CTL[n].BWEN bit == 1). Note that if the address range match is enabled ( SWU_CTL[n].ACMPM bits) and if any address of a burst falls within the address range, the SWU_CUR[n].CURBW field is incremented by the burst length even if some of the burst address fall outside of the | Increment Bandwidth Count by Burst Length. Each SWU_CTL[n].BLENINC bit controls how a watchpoint group's bandwidth count is incremented in the SWU_CUR[n] register's SWU_CUR[n].CURBW field. If the SWU_CTL[n].BLENINC bit is cleared (= 0), the SWU increments the band- width count by 1 for each matching transaction. If the SWU_CTL[n].BLENINC bit is set (=1), the SWU increments the bandwidth count by the burst length of the trans- action for each matching transaction. This feature is only valid for bandwidth mode ( SWU_CTL[n].BWEN bit == 1). Note that if the address range match is enabled ( SWU_CTL[n].ACMPM bits) and if any address of a burst falls within the address range, the SWU_CUR[n].CURBW field is incremented by the burst length even if some of the burst address fall outside of the |
| 17 (R/W)           | BLENINC    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Increment by 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 16 (R/W)           | BWEN       | Bandwidth Mode Enable. Each SWU_CTL[n].BWEN bit controls whether a watchpoint group operates in watchpoint mode or bandwidth mode. In watchpoint mode, the SWU_CTL[n].CNTEN and (optionally) SWU_CTL[n].CNTRPTEN registers con- trol usage of the cycle count for watchpoint group operations. In bandwidth mode, the SWU_CTL[n].BLENINC , SWU_TARG[n] , and SWU_HIST[n] registers control usage of watchpoint matches for watchpoint group operations.                                                                                                                                                                                                                                                                                                                                                                   | Bandwidth Mode Enable. Each SWU_CTL[n].BWEN bit controls whether a watchpoint group operates in watchpoint mode or bandwidth mode. In watchpoint mode, the SWU_CTL[n].CNTEN and (optionally) SWU_CTL[n].CNTRPTEN registers con- trol usage of the cycle count for watchpoint group operations. In bandwidth mode, the SWU_CTL[n].BLENINC , SWU_TARG[n] , and SWU_HIST[n] registers control usage of watchpoint matches for watchpoint group operations.                                                                                                                                                                                                                                                                                                                                                                   |
| 16 (R/W)           | BWEN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Watchpoint Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 16 (R/W)           | BWEN       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Bandwidth Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14 (R/W)           | TRGEN      | Trigger Enable.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Trigger Enable.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 14 (R/W)           | TRGEN      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 14 (R/W)           | TRGEN      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 13 (R/W)           | INTEN      | Interrupt Enable. Each SWU_CTL[n].INTEN bit controls whether a match for a watchpoint group generates an interrupt. This feature is valid in both bandwidth and watchpoint modes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Interrupt Enable. Each SWU_CTL[n].INTEN bit controls whether a match for a watchpoint group generates an interrupt. This feature is valid in both bandwidth and watchpoint modes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 13 (R/W)           | INTEN      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 13 (R/W)           | INTEN      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 55-7: SWU\_CTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | DBGEN      | Debug Event Enable. Each SWU_CTL[n].DBGEN bit controls debug event comparison for a watchpoint group, permitting matches based on debug status.                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 12 (R/W)           | DBGEN      | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 9 (R/W)            | CNTRPTEN   | Count Repeat Enable. Each SWU_CTL[n].CNTRPTEN bit controls whether the watchpoint group's cycle count is reloaded and repeated after cycle countdown. If the SWU_CTL[n] register's SWU_CTL[n].CNTRPTEN bit is set, the SWU_CUR[n] register's SWU_CUR[n].CURCNT field is reloaded from SWU_CNT[n] register's SWU_CNT[n].COUNT field, and the countdown starts again. If SWU_CTL[n].CNTRPTEN bit is cleared, the expired count remains zero, and no further events are signalled. (See the SWU_CTL[n].CNTEN bit description for infor- mation regarding the countdown setup.) |
| 8                  | CNTEN      | Count Enable.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 8                  | CNTEN      | the end of the countdown.) 0 Disable 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 6 (R/W)            | LCMPEN     | Locked Comparison Enable. Each SWU_CTL[n].LCMPEN bit controls locked comparison operation of an SWU watchpoint group, permitting matches based on lock status.                                                                                                                                                                                                                                                                                                                                                                                                              |
| 6 (R/W)            | LCMPEN     | 0 Match on all transaction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 5 (R/W)            | SCMPEN     | Secure Comparison Enable. Each SWU_CTL[n].SCMPEN bit controls secure transaction comparison of an SWU watchpoint group, permitting matches based on transaction security.                                                                                                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W)            | SCMPEN     | 0 Match on all transaction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 5 (R/W)            | SCMPEN     | operation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |            | 1 Match only secure transactions                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 55-7: SWU\_CTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | IDCMPEN    | ID Comparison Enable. Each SWU_CTL[n].IDCMPEN bit controls the ID comparison operation of an SWU watchpoint group. The ID match is based on comparison with the value in the SWU_ID[n] register.                                                                                                                                                                                                                                       |
| 3:2 (R/W)          | ACMPM      | Address Comparison Mode. Each set of SWU_CTL[n].ACMPM bits control the address comparison operation of an SWU watchpoint group. The address within range for comparison is defined as ( SWU_LA[n] register <= address < SWU_UA[n] register). The address outside range for comparison is defined as (address < SWU_LA[n] ) or ( SWU_UA[n] <= address).                                                                                 |
| 1 (R/W)            | DIR        | 3 Match on address outside range Transaction Direction for Match. Each SWU_CTL[n].DIR bit determines whether the SWU check reads or writes for watchpoint matches.                                                                                                                                                                                                                                                                     |
| 0 (R/W)            | EN         | Enable Watchpoint. Each SWU_CTL[n].EN bit controls the operation of one SWU watchpoint group. Clearing the SWU_CTL[n].EN bit halts the execution of watchpoint or bandwidth tracking operations in the watchpoint group without resetting status or configuration registers. Setting the SWU_CTL[n].EN bit enables the SWU watchpoint group to begin or resume operation with the current configuration and status. 0 Disable 1 Enable |

## Current Register n

The SWU current register ( SWU\_CUR[n] ) operation varies depending whether the watchpoint group is in bandwidth mode or watchpoint mode. In both modes, the watchpoint count begins when the SWU loads the register's SWU\_CUR[n].CURCNT field from the SWU\_CNT[n] register's SWU\_CNT[n].COUNT field when the watchpoint count is enabled ( SWU\_CTL[n] register, SWU\_CTL[n].CNTEN bit =1).

In bandwidth mode, the current count field ( SWU\_CUR[n].CURCNT ) contains the cycle count remaining within the current watchpoint period. The SWU decrements this value every cycle until the count reaches zero. At that point, the SWU reloads the SWU\_CUR[n].CURCNT field from SWU\_CNT[n] register's SWU\_CNT[n].COUNT field. In bandwidth mode, the current bandwidth field ( SWU\_CUR[n].CURBW ) contains the count of watchpoint matches (bandwidth) accumulated in the current watchpoint period.

In watchpoint mode, the current count field ( SWU\_CUR[n].CURCNT ) contains the watchpoint match count remaining within the current watchpoint period. The SWU decrements this value with every watchpoint match until the count reaches zero. At that point, the SWU reloads the SWU\_CUR[n].CURCNT field from SWU\_CNT[n] register's SWU\_CNT[n].COUNT field if the SWU\_CTL[n] register's SWU\_CTL[n].CNTRPTEN bit is set (=1). In watchpoint mode, the current bandwidth field ( SWU\_CUR[n].CURBW ) is undefined.

Figure 55-5: SWU\_CUR[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000004_15bf980fb37478c1ad7084e857d574fcb22fe97f69636240a2143e695aa311a7.png)

Table 55-8: SWU\_CUR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:16 (R/NW)       | CURBW      | Current Bandwidth.        |
| 15:0 (R/NW)        | CURCNT     | Current Count.            |

## Global Control Register

The SWU global control register ( SWU\_GCTL ) provides SWU reset and enable.

Figure 55-6: SWU\_GCTL Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000005_bacad16a044f5b254655210aa3bb5be1fa6fc4a978e9d52d250179b889f32ab8.png)

Table 55-9: SWU\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R0/W)           | RST        | Global Reset. The SWU_GCTL.RST ) is write-1-action/read zero and controls the SWU operational state. Setting SWU_GCTL.RST resets all SWU registers to their default values and halts all SWU operations. 0 No Action 1 Reset                                                                                                                            |
| 0 (R/W)            | EN         | Global Enable. The SWU_GCTL.EN controls the SWU operational state. Clearing SWU_GCTL.EN halts the execution of all watchpoint and bandwidth tracking operations without reset- ting status registers or associated signals. Setting SWU_GCTL.EN enables the SWU to begin/resume operation with the current configuration and status. 0 Disable 1 Enable |

## Global Status Register

The SWU global status register ( SWU\_GSTAT ) contains status bits for all four watchpoint groups.

Figure 55-7: SWU\_GSTAT Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000006_af2352dc290001d04cce435884a0dbb772443ec76aa7e112a0c30706d4f0fa9d.png)

Table 55-10: SWU\_GSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W1C)         | ADDRERR    | Address Error Status. The SWU_GSTAT.ADDRERR indicates that the SWU generated an address error. This status bit is sticky; write-1-to-clear it. |
| 30 (R/W1C)         | ADDRERR    | 0 Inactive                                                                                                                                     |
| 30 (R/W1C)         | ADDRERR    | 1 Active                                                                                                                                       |

Table 55-10: SWU\_GSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | OVRBW3     | Group 3 Bandwidth Above Maximum Target. See SWU_GSTAT.OVRBW0 description. 0 Group 3 was not above maximum bandwidth                                                                                                                                                                                                                                                                                                |
| 14 (R/W1C)         | UNDRBW3    | Group 3 Bandwidth Below Minimum Target. See SWU_GSTAT.UNDRBW0 description. 0 Group 3 was not below minimum bandwitdth                                                                                                                                                                                                                                                                                              |
| 13 (R/W1C)         | OVRBW2     | Group 2 Bandwidth Above Maximum Target. See SWU_GSTAT.OVRBW0 description. 0 Group 2 was not above maximum bandwidth                                                                                                                                                                                                                                                                                                |
| 12 (R/W1C)         | UNDRBW2    | Group 2 Bandwidth Below Minimum Target. See SWU_GSTAT.UNDRBW0 description. 0 Group 2 was not below minimum                                                                                                                                                                                                                                                                                                         |
| 11 (R/W1C)         | OVRBW1     | bandwidth 1 Group 2 was below minimum bandwidth Group 1 Bandwidth Above Maximum Target.                                                                                                                                                                                                                                                                                                                            |
| 10 (R/W1C)         |            | See SWU_GSTAT.OVRBW0 description. 0 Group 1 was not above maximum bandwidth 1 Group 1 was above maximum bandwidth                                                                                                                                                                                                                                                                                                  |
|                    | UNDRBW1    | Group 1 Bandwidth Below Minimum Target. See SWU_GSTAT.UNDRBW0 description. 0 Group 1 was not below minimum bandwidth 1 Group 1 was below minimum bandwidth                                                                                                                                                                                                                                                         |
| 9 (R/W1C)          | OVRBW0     | Group 0 Bandwidth Above Maximum Target. The SWU_GSTAT.OVRBW0 - SWU_GSTAT.OVRBW3 -- Group 0 through 3 watch- point bandwidth over maximum target bits. Each maximum bandwidth bit indicate (for each group)s that the measured bandwidth over the period defined by the SWU_CNT[n] register was over the maximum target. This status bit is sticky; write-1- to-clear it. 0 Group 0 was not above maximum bandwidth |

Table 55-10: SWU\_GSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8                  | UNDRBW0    | Group 0 Bandwidth Below Minimum Target.                                                                                                                                                                                                                                                                                         |
| 7 (R/W1C)          | INT3       | Group 3 Interrupt Status. See SWU_GSTAT.INT0 description. 0 No Interrupt                                                                                                                                                                                                                                                        |
| 6 (R/W1C)          | INT2       | Group 2 Interrupt Status. See SWU_GSTAT.INT0 description. 0 No Interrupt                                                                                                                                                                                                                                                        |
| 5 (R/W1C)          | INT1       | 1 Interrupt Occurred Group 1 Interrupt Status.                                                                                                                                                                                                                                                                                  |
|                    |            | See SWU_GSTAT.INT0 description. 0 No Interrupt 1 Interrupt Occurred                                                                                                                                                                                                                                                             |
| 4 (R/W1C)          | INT0       | Group 0 Interrupt Status. The SWU_GSTAT.INT0 - SWU_GSTAT.INT3 -- Group 0 through 3 interrupt bits. Each interrupt bit indicates (for each group) whether a watchpoint group is con- tributing to the SWU's interrupt output. This status bit is sticky; write-1-to-clear it. 0 No interrupt 1 Interrupt Occurred Group 3 Match. |
| 3 (R/W1C)          | MTCH3      | See SWU_GSTAT.MTCH0 description. 0 No Match                                                                                                                                                                                                                                                                                     |

Table 55-10: SWU\_GSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | MTCH2      | Group 2 Match. See SWU_GSTAT.MTCH0 description. 0 No match                                                                                                                                                                                                                                                                                         |
| 1 (R/W1C)          | MTCH1      | Group 1 Match. See SWU_GSTAT.MTCH0 description. 0 No match                                                                                                                                                                                                                                                                                         |
| 0 (R/W1C)          | MTCH0      | Group 0 Match. The SWU_GSTAT.MTCH0 - SWU_GSTAT.MTCH3 -- Group 0 through 3 match bits. Each match bit indicates (for each group) whether a watchpoint match has occur- red in a SWU watchpoint group, as controlled by the group's related watchpoint con- trol register ( SWU_CTL[n] ). This status bit is sticky; write-1-to-clear it. 0 No match |
| 0 (R/W1C)          | 1          | Group 0 Watchpoint Match                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W1C)          |            |                                                                                                                                                                                                                                                                                                                                                    |

## Bandwidth History Register n

The SWU bandwidth history registers ( SWU\_HIST[n] ) contain data copied from a watchpoint group's current bandwidth value ( SWU\_CUR[n] register, SWU\_CUR[n].CURBW bits) at the end of the last two watchpoint periods. At the end of each watchpoint period, the SWU copies the previous bandwidth value from the SWU\_HIST[n].BWHIST0 field to the SWU\_HIST[n].BWHIST1 field and copies the new bandwidth value from the SWU\_CUR[n].CURBW field to the SWU\_HIST[n].BWHIST0 field.

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000007_6799f4585123198f468a7838159f15a443bd89427d5f5d01d61e4f8a4cd2fbee.png)

Bandwidth from Window Before Last

Figure 55-8: SWU\_HIST[n] Register Diagram

Table 55-11: SWU\_HIST[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:16 (R/NW)       | BWHIST1    | Bandwidth from Window Before Last. |
| 15:0 (R/NW)        | BWHIST0    | Bandwidth from Last Window.        |

## ID Register n

The SWU ID registers ( SWU\_ID[n] ) contain a 16-bit ID field ( SWU\_ID[n].ID ) and a 16-bit ID mask field ( SWU\_ID[n].IDMASK ) that watchpoint groups use for ID comparison. The ID on the bus is AND'ed with the SWU\_ID[n].IDMASK field, then the watchpoint group compares the result against the SWU\_ID[n].ID field.

Figure 55-9: SWU\_ID[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000008_54935d9980bc45be8657ec63b94c42cdc24b98f6bce529eb5a91a7107bb021ed.png)

Table 55-12: SWU\_ID[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 31:16 (R/W)        | IDMASK     | Identity Mask (for Or with ID). |
| 15:0 (R/W)         | ID         | Identity.                       |

## Lower Address Register n

The SWU lower address registers ( SWU\_LA[n] ) contain each watchpoint group's lower address for address match comparison. In exact match on SWU\_LA[n] address mode ( SWU\_CTL[n].ACMPM bits =01), the watchpoint group uses only this address for match comparison.

Figure 55-10: SWU\_LA[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000009_4bd5b06175dc9173c09fb95acee239f0d52b35cbe62cf3fa8b9802d95de8d78c.png)

Table 55-13: SWU\_LA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ADDR       | Lower Address.            |
| (R/W)              |            |                           |

## Target Register n

The SWU target registers ( SWU\_TARG[n] ) contain a minimum value field ( SWU\_TARG[n].BWMIN ) and maximum value field ( SWU\_TARG[n].BWMAX ) of bandwidth targets used by watchpoint groups in bandwidth mode. When the bandwidth period expires, if the current bandwidth value ( SWU\_CUR[n] register, SWU\_CUR[n].CURBW bits) is below the minimum target or above the maximum target, the watchpoint group takes action as enabled by the SWU\_CTL[n] register's SWU\_CTL[n].MINACT or SWU\_CTL[n].MAXACT bits.

In bandwidth mode, note that the watchpoint group increments its count of either data bus transactions or address bus transactions (bursts) as selected by the SWU\_CTL[n].BLENINC bit. Keep this mode selection in mind when programming the bandwidth target values.

Figure 55-11: SWU\_TARG[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000010_4453039ba9cc6aff9d0330e75602e3b41c0e0615491eeb0ddca06a4a41fe278b.png)

Table 55-14: SWU\_TARG[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:16              | BWMAX      | Maximum Bandwidth Target. |
| (R/W) 15:0         | BWMIN      | Minimum Bandwidth Target. |

## Upper Address Register n

The SWU upper address registers ( SWU\_UA[n] ) contain each watchpoint group's upper address for address match comparison. In exact match on SWU\_LA[n] address mode ( SWU\_CTL[n].ACMPM bits =01), the SWU\_UA[n] is not used for match comparison.

Figure 55-12: SWU\_UA[n] Register Diagram

![Image](58_System_Watchpoint_Unit_(SWU)_artifacts/image_000011_0cc671391b84476670bd32ad26ce4db46b700063c78d22896618fb14ccd1391e.png)

Table 55-15: SWU\_UA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ADDR       | Upper Address.            |