# Memory Error Protection Unit (MEPU)

<!-- source: 056_Memory_Error_Protection_Unit_MEPU.pdf | original pages 3335–3369 -->

## 49   Memory Error Protection Unit (MEPU)

Memory Error Protection Unit (MEPU) handles single-bit and double-bit memory error detection and correction across the memories in the chip and controls routing of their interrupts/triggers.

It consists of following blocks:

- Memory Error Controller (MEC)
- Parity Controller (PCTL)

## Memory Error Controller (MEC)

The Memory Error Controller (MEC) manages memory parity, ECC errors and warning inputs from various cores and peripherals and sends out interrupt and trigger outputs. It is a generic N x M multiplexer and interrupt controller for error or warning inputs to interrupt and trigger outputs. It has control features such as enable and disable and interrupt masking, and status signaling for input/outputs. For more information on MEC functionality, see the MEC registers description.

## MEC Features

The MEC unit has the following features:

- Interrupt controller for memory parity, ECC errors and warnings
- N x M multiplexer for error inputs to interrupt and trigger outputs
- Peripheral bus completer interface for register read and write
- Control and status registers
- Control for enabling or disabling each input error status and masking its interrupt
- Control for enabling or disabling each output error interrupt
- Input error status bits are sticky and write-1-to-clear
- Output error interrupt status bits are read only
- Lock feature for control registers write. Status registers capture lock write error.

- Peripheral and component identification registers
- Capability to retain parity, ECC error and warning status even after any reset for fault source debug
- Fully-configurable design

## Parity Controller (PCTL)

The Parity Controller (PCTL) handles parity encoding and decoding to and from memory data for single bit memory error detection.

## PCTL Features

The PCTL unit has the following features:

- Generic memory parity controller
- Generation of write parity bits from write data bits
- Concatenation of parity bits with data bits for memory write data
- Generation of write enable mask for parity bits to memory
- Extraction of read data bits from memory read data
- Detection of read parity error from memory read data
- Even parity logic for parity generation and error detection
- Provision for parity bit interleaving
- Single parity error output per memory instance
- Memory initialization control logic
- Fully-configurable design

## MEC Functional Description

The MEC works with the PCTL to detect and correct memory error across the memories in the chip. It also controls the routing of the generated interrupts and triggers.

## ADSP-2159x\_SC592\_SC594 MEC Register List

The Memory Error Controller (MEC) manages memory parity/ inputs from the core and peripherals and sends interrupt/trigger outputs. It is a generic N x M multiplexer and interrupt controller for error to interrupt/trigger outputs. It has control features such as enable/disable and interrupt masking, and status signaling for inputs/outputs. For more information on MEC functionality, see the MEC register descriptions.

Table 49-1: ADSP-2159x\_SC592\_SC594 MEC Register List

| Name                | Description                                            |
|---------------------|--------------------------------------------------------|
| MEC_CID0            | Component ID0 Register                                 |
| MEC_CID1            | Component ID1 Register                                 |
| MEC_CID2            | Component ID2 Register                                 |
| MEC_CID3            | Component ID3 Register                                 |
| MEC_CLR             | Clear Register                                         |
| MEC_ECCERR_CTL[n]   | ECC Error Control Register                             |
| MEC_ECCERR_IMASK[n] | ECC Error Interrupt Mask Register                      |
| MEC_ECCERR_STAT[n]  | ECC Error Status Register                              |
| MEC_EEIRQ_GCTL[n]   | ECC Error Interrupt Request Global Control Register    |
| MEC_EEIRQ_GSTAT[n]  | ECC Error Interrupt Request Global Status Register     |
| MEC_PEIRQ_GCTL[n]   | Parity Error Interrupt Request Global Control Register |
| MEC_PEIRQ_GSTAT[n]  | Parity Error Interrupt Request Global Status Register  |
| MEC_PERR_CTL0       | Parity Error Control Register                          |
| MEC_PERR_CTL1       | Parity Error Control Register                          |
| MEC_PERR_IMASK0     | Parity Error Interrupt Mask Register                   |
| MEC_PERR_IMASK1     | Parity Error Interrupt Mask Register                   |
| MEC_PERR_STAT0      | Parity Error Status Register                           |
| MEC_PERR_STAT1      | Parity Error Status Register                           |
| MEC_PID0            | Peripheral ID0 Register                                |
| MEC_PID1            | Peripheral ID1 Register                                |
| MEC_PID2            | Peripheral ID2 Register                                |
| MEC_PID3            | Peripheral ID3 Register                                |
| MEC_PID4            | Peripheral ID4 Register                                |

## ADSP-2159x\_SC592\_SC594 MEC Trigger List

Table 49-2: ADSP-2159x\_SC592\_SC594 MEC Trigger List Generators

|   Trigger ID | Name        | Description                         | Sensitivity   |
|--------------|-------------|-------------------------------------|---------------|
|           58 | MEC0_EEIRQ0 | MEC0 ECC Error Interrupt Request    | Level         |
|           59 | MEC0_PEIRQ0 | MEC0 Parity Error Interrupt Request | Level         |
|           60 | MEC0_PEIRQ1 | MEC0 Parity Error Interrupt Request | Level         |
|           61 | MEC0_PEIRQ2 | MEC0 Parity Error Interrupt Request | Level         |

Table 49-2: ADSP-2159x\_SC592\_SC594 MEC Trigger List Generators (Continued)

|   Trigger ID | Name        | Description                         | Sensitivity   |
|--------------|-------------|-------------------------------------|---------------|
|           62 | MEC0_PEIRQ3 | MEC0 Parity Error Interrupt Request | Level         |
|           63 | MEC1_EEIRQ0 | MEC1 ECC Error Interrupt Request    | Level         |
|           64 | MEC1_PEIRQ0 | MEC1 Parity Error Interrupt Request | Level         |
|           65 | MEC1_PEIRQ1 | MEC1 Parity Error Interrupt Request | Level         |
|           66 | MEC1_PEIRQ2 | MEC1 Parity Error Interrupt Request | Level         |
|           67 | MEC1_PEIRQ3 | MEC1 Parity Error Interrupt Request | Level         |
|           68 | MEC2_EEIRQ0 | MEC2 ECC Error Interrupt Request    | Level         |
|           69 | MEC2_PEIRQ0 | MEC2 Parity Error Interrupt Request | Level         |
|           70 | MEC2_PEIRQ1 | MEC2 Parity Error Interrupt Request | Level         |
|           71 | MEC2_PEIRQ2 | MEC2 Parity Error Interrupt Request | Level         |
|           72 | MEC2_PEIRQ3 | MEC2 Parity Error Interrupt Request | Level         |

Table 49-3: ADSP-2159x\_SC592\_SC594 MEC Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

## MEPU Block Diagram

The MEPU Block Diagram shows the functional blocks within the Memory Error Protection Unit (MEPU) unit.

Figure 49-1: MEPU Block Diagram

<!-- image -->

## MEC Block Diagram

The MEC Interface Block Diagram shows the functional blocks within the MEC module.

Figure 49-2: MEC Unit Block Diagram

<!-- image -->

## PCTL Block Diagram

The PCTL Block Diagram shows the functional blocks within the PCTL module.

Figure 49-3: PCTL Block Diagram

<!-- image -->

## MECx Input and Output Block Diagram

The MECx Input and Output Block Diagram shows the input and output routing to various cores.

Figure 49-4: MECx Input and Output Block Diagram

<!-- image -->

## MEC Architectural Concepts

The MEC unit communicates with the Cortex-A5 core directly through the subsystem's SYS bus crossbar. It services MMR addresses associated with the MEC function unit and translates the addresses into specific transcendental MEC functions. The results are read from common result registers shared by all the functions in single-precision floating-point format. The block stalls all read accesses until the result is ready.

## MEC Configuration

The following tables describe the range of values for configuration and the mapping of parity errors.

Table 49-4: Mapping of Input Parity Errors (PERR)

|   PERR ID | Name         | Description                                      |
|-----------|--------------|--------------------------------------------------|
|         0 | PERR_C0_L1CR | Core0 (ARM) L1 Cache RAM Parity Error            |
|         1 | PERR_C0_L2CR | Core0 (ARM) L2 Cache RAM Parity Error            |
|         2 | PERR_C1_L1R  | Core1 (SHARC0) L1 RAM Parity Error               |
|         3 | PERR_C1_L1CR | Core1 (SHARC0) L1 Cache RAM Parity Error         |
|         4 | PERR_C1_BPR  | Core1 (SHARC0) Branch Predictor RAM Parity Error |
|         5 | PERR_C2_L1R  | Core2 (SHARC1) L1 RAM Parity Error               |

Table 49-4: Mapping of Input Parity Errors (PERR) (Continued)

|   PERR ID | Name           | Description                                      |
|-----------|----------------|--------------------------------------------------|
|         6 | PERR_C2_L1CR   | Core2 (SHARC1) L1 Cache RAM Parity Error         |
|         7 | PERR_C2_BPR    | Core2 (SHARC1) Branch Predictor RAM Parity Error |
|         8 | PERR_ASRC0_FR  | ASRC0 FIFO RAM Parity Error                      |
|         9 | PERR_ASRC1_FR  | ASRC1 FIFO RAM Parity Error                      |
|        10 | PERR_ASRC2_FR  | ASRC2 FIFO RAM Parity Error                      |
|        11 | PERR_ASRC3_FR  | ASRC3 FIFO RAM Parity Error                      |
|        12 | PERR_IIR0_R    | IIR0 RAM Parity Error                            |
|        13 | PERR_FIR0_R    | FIR0 RAM Parity Error                            |
|        14 | PERR_USB0_FR   | USB0 FIFO RAM Parity Error                       |
|        15 | PERR_CAN0_MBR  | CAN0 Mailbox RAM Parity Error                    |
|        16 | PERR_CAN0_AMR  | CAN0 Acceptance Mask RAM Parity Error            |
|        17 | PERR_CAN1_MBR  | CAN1 Mailbox RAM Parity Error                    |
|        18 | PERR_CAN1_AMR  | CAN1 Acceptance Mask RAM Parity Error            |
|        19 | PERR_TRNG0_DBR | TRNG0 Data Buffer RAM Parity Error               |
|        20 | PERR_PKA0_DR   | PKA0 Data RAM Parity Error                       |
|        21 | PERR_SPE0_BR   | SPE0 Buffer RAM Parity Error                     |
|        22 | PERR_SPE0_AR   | SPE0 ARC4 RAM Parity Error                       |
|        23 | PERR_EMAC0_TFR | EMAC0 Transmit FIFO RAM Parity Error             |
|        24 | PERR_EMAC0_RFR | EMAC0 Receive FIFO RAM Parity Error              |
|        25 | PERR_MSI0_FR   | MSI0 FIFO RAM Parity Error                       |
|        26 | PERR_MLB0_DBR  | MLB0 Data Buffer RAM Parity Error                |
|        27 | PERR_MLB0_CTR  | MLB0 Channel Table RAM Parity Error              |
|        28 | PERR_TMC0_TDR  | TMC0 Trace Data RAM Parity Error                 |

Table 49-5: Mapping of Output Parity Errors (PERR)

|   PERR ID | Name      | Description                              |
|-----------|-----------|------------------------------------------|
|         0 | PEIRQ_C0  | Core0 (ARM) Parity Error Interrupt       |
|         1 | PEIRQ_C1  | Core1 (SHARC0) Parity Error Interrupt    |
|         2 | PEIRQ_C2  | Core2 (SHARC1) Parity Error Interrupt    |
|         3 | PEIRQ_SYS | System Peripheral Parity Error Interrupt |

Table 49-6: Mapping of Input ECC Errors (ECCERR)

|   PERR ID | Name          | Description      |
|-----------|---------------|------------------|
|         0 | ECCERR_L2CTL0 | L2CTL0 ECC Error |

Table 49-7: Mapping of Output ECC Errors (EEIRQ)

|   PERR ID | Name         | Description                |
|-----------|--------------|----------------------------|
|         0 | EWIRQ_L2CTL0 | L2CTL0 ECC Error Interrupt |

## PCTL Integration

There is one PCTL instance per port per memory instance (for example, one PCTL instance for single port memory and two PCTL instances for dual port memories).

Memory initialization control logic supported by PCTL is used only for instances attached to Arm L1 cache memories.

An additional softwer trigger requester and initialization trigger completer is provided in TRU for starting memory initialization. A pulse from this trigger completer starts initialization of Arm L1 cache memories. When all the locations of a particular memory instance get initialized, the PCTL generates a memory initialization done signal for the corresponding memory instance. These signals from all PCTL instances corresponding to Arm L1 cache memories are AND'ed to generate a single memory initialization done output which is connected as an interrupt to SEC/GIC and a trigger requester to TRU.

## ADSP-2159x\_SC592\_SC594 MEC Register Descriptions

Memory Error Controller (MEC) contains the following registers.

Table 49-8: ADSP-2159x\_SC592\_SC594 MEC Register List

| Name                | Description                                         |
|---------------------|-----------------------------------------------------|
| MEC_CID0            | Component ID0 Register                              |
| MEC_CID1            | Component ID1 Register                              |
| MEC_CID2            | Component ID2 Register                              |
| MEC_CID3            | Component ID3 Register                              |
| MEC_CLR             | Clear Register                                      |
| MEC_ECCERR_CTL[n]   | ECC Error Control Register                          |
| MEC_ECCERR_IMASK[n] | ECC Error Interrupt Mask Register                   |
| MEC_ECCERR_STAT[n]  | ECC Error Status Register                           |
| MEC_EEIRQ_GCTL[n]   | ECC Error Interrupt Request Global Control Register |
| MEC_EEIRQ_GSTAT[n]  | ECC Error Interrupt Request Global Status Register  |

Table 49-8: ADSP-2159x\_SC592\_SC594 MEC Register List (Continued)

| Name               | Description                                            |
|--------------------|--------------------------------------------------------|
| MEC_PEIRQ_GCTL[n]  | Parity Error Interrupt Request Global Control Register |
| MEC_PEIRQ_GSTAT[n] | Parity Error Interrupt Request Global Status Register  |
| MEC_PERR_CTL0      | Parity Error Control Register                          |
| MEC_PERR_CTL1      | Parity Error Control Register                          |
| MEC_PERR_IMASK0    | Parity Error Interrupt Mask Register                   |
| MEC_PERR_IMASK1    | Parity Error Interrupt Mask Register                   |
| MEC_PERR_STAT0     | Parity Error Status Register                           |
| MEC_PERR_STAT1     | Parity Error Status Register                           |
| MEC_PID0           | Peripheral ID0 Register                                |
| MEC_PID1           | Peripheral ID1 Register                                |
| MEC_PID2           | Peripheral ID2 Register                                |
| MEC_PID3           | Peripheral ID3 Register                                |
| MEC_PID4           | Peripheral ID4 Register                                |

## Component ID0 Register

Figure 49-5: MEC\_CID0 Register Diagram

<!-- image -->

Table 49-9: MEC\_CID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID0.PREAMBLE field indicates the component ID preamble. |

## Component ID1 Register

Figure 49-6: MEC\_CID1 Register Diagram

<!-- image -->

Table 49-10: MEC\_CID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | COMPCLASS  | Component Class. The MEC_CID1.COMPCLASS field indicates the component class. Dedicated debug blocks (core debug access port, Program Trace, etc.) should identify as CoreSight (i.e. 0x9) and implement the full compliment of CoreSight registers including DEVTYPE. All other ADI components should identify as System (i.e. 0xF) components. See Core- Sight Architecture Specification for details. 9 CoreSight |
| 3:0 (R/NW)         | PREAMBLE   | Component ID Preamble. The MEC_CID1.PREAMBLE field indicates the component ID preamble.                                                                                                                                                                                                                                                                                                                             |

## Component ID2 Register

Figure 49-7: MEC\_CID2 Register Diagram

<!-- image -->

Table 49-11: MEC\_CID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID2.PREAMBLE field indicates the component ID preamble. |

## Component ID3 Register

Figure 49-8: MEC\_CID3 Register Diagram

<!-- image -->

Table 49-12: MEC\_CID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID3.PREAMBLE field indicates the component ID preamble. |

## Clear Register

Figure 49-9: MEC\_CLR Register Diagram

<!-- image -->

Table 49-13: MEC\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 0                  | CLRSTAT    | Clear Status.                                                            |
| (R0/W)             |            | Writing 1 to the MEC_CLR.CLRSTAT bit clears all status registers of MEC. |

## ECC Error Control Register

The MEC\_ECCERR\_CTL[n] register bits control enable/disable for ECC error inputs from various cores/peripherals and decide whether their status will be reflected in ECC error status register bits.

Figure 49-10: MEC\_ECCERR\_CTL[n] Register Diagram

<!-- image -->

Table 49-14: MEC\_ECCERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                        | Description/Enumeration                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_CTL[n].LOCK bit is enabled, the MEC_ECCERR_CTL[n] regis- ter is read only. | ECC Error Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_CTL[n].LOCK bit is enabled, the MEC_ECCERR_CTL[n] regis- ter is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                              | Unlock                                                                                                                                                                         |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                              | Lock                                                                                                                                                                           |
| 2:0 (R/W)          | VALUE      | ECC Error Control.                                                                                                                                                             | ECC Error Control.                                                                                                                                                             |
| 2:0 (R/W)          | VALUE      | 1                                                                                                                                                                              | L2 CTL ECC Error                                                                                                                                                               |
| 2:0 (R/W)          | VALUE      | 2                                                                                                                                                                              | CAN0 ECC Error                                                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 4                                                                                                                                                                              | CAN1 ECC Error                                                                                                                                                                 |

## ECC Error Interrupt Mask Register

The MEC\_ECCERR\_IMASK[n] register bits control interrupt masks for ECC error inputs from various cores/ peripherals.

Figure 49-11: MEC\_ECCERR\_IMASK[n] Register Diagram

<!-- image -->

Table 49-15: MEC\_ECCERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Interrupt Mask Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_IMASK[n].LOCK bit is enabled, the MEC_ECCERR_IMASK[n] register is read only. |
| 31 (R/W)           | LOCK       | 0 Unlock                                                                                                                                                                                |
| 2:0 (R/W)          | VALUE      | ECC Error Interrupt Mask. The MEC_ECCERR_IMASK[n].VALUE bit indicates when the ECC error inter- rupt is masked (=1) or unmasked (=0). 1 L2 CTL ECC Error 2 CAN0 ECC Error               |
| 2:0 (R/W)          | VALUE      | 4 CAN1 ECC Error                                                                                                                                                                        |
| 2:0 (R/W)          | VALUE      |                                                                                                                                                                                         |
| 2:0 (R/W)          | VALUE      |                                                                                                                                                                                         |

## ECC Error Status Register

The MEC\_ECCERR\_STAT[n] register bits reflect status for ECC error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding ECC error status.

Figure 49-12: MEC\_ECCERR\_STAT[n] Register Diagram

<!-- image -->

Table 49-16: MEC\_ECCERR\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | ECC Error Control or Interrupt Mask Register Lock Write Error. The MEC_ECCERR_STAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write 1 to clear it. 0 No Error |
| 2:0 (R/W1C)        | VALUE      | ECC Error Status. '1' indicates error and '0' indicates no error. Sticky bit, write '1' to clear. 1 L2 CTL ECC Error 2 CAN0 ECC Error 4 CAN1 ECC Error                                                                                                                                                               |

## ECC Error Interrupt Request Global Control Register

The MEC\_EEIRQ\_GCTL[n] register bits control enable/disable of ECC error interrupt/trigger outputs.

Figure 49-13: MEC\_EEIRQ\_GCTL[n] Register Diagram

<!-- image -->

Table 49-17: MEC\_EEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Global Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_EEIRQ_GCTL[n].LOCK bit is enabled, the MEC_EEIRQ_GCTL[n] regis- ter is read only. |
| 0 (R/W)            | VALUE      | ECC Error Global Control. '1' indicates enable and '0' indicates disable.                                                                                                             |

## ECC Error Interrupt Request Global Status Register

The MEC\_EEIRQ\_GSTAT[n] register bits reflect the status of the ECC error interrupt/trigger outputs.

<!-- image -->

Write Error

Figure 49-14: MEC\_EEIRQ\_GSTAT[n] Register Diagram

Table 49-18: MEC\_EEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | ECC Error Global Control Register Lock Write Error. The MEC_EEIRQ_GSTAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write 1 to clear it. 0 No Error |
| 0 (R/NW)           | VALUE      | ECC Error Global Status. '1' indicates assertion of interrupt/trigger and '0' indicates no interrupt/trigger.                                                                                                                                                                                             |

## Parity Error Interrupt Request Global Control Register

The MEC\_PEIRQ\_GCTL[n] register bits control enable/disable of parity error interrupt/trigger outputs.

Figure 49-15: MEC\_PEIRQ\_GCTL[n] Register Diagram

<!-- image -->

Table 49-19: MEC\_PEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Parity Error Global Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PEIRQ_GCTL[n].LOCK bit is enabled, the MEC_PEIRQ_GCTL[n] regis- ter is read only. |
| 3:0 (R/W)          | VALUE      | Parity Error Global Control. The MEC_PEIRQ_GCTL[n].VALUE bit field indicates whether parity error control is enabled (=1) or disabled (=0).                                              |

## Parity Error Interrupt Request Global Status Register

The MEC\_PEIRQ\_GSTAT[n] register bits reflect status of parity error interrupt/trigger outputs.

Figure 49-16: MEC\_PEIRQ\_GSTAT[n] Register Diagram

<!-- image -->

Table 49-20: MEC\_PEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Global Control Register Lock Write Error. When set (=1), the MEC_PEIRQ_GSTAT[n].LWERR bit indicates an attempted write to an MEC register while the MEC_PEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write 1 to clear it. |
| 3:0 (R/NW)         | VALUE      | Parity Error Global Status. The MEC_PEIRQ_GSTAT[n].VALUE field indicates the parity error global status. 1 indicates the assertion of the interrupt/trigger; 0 indicates no interrupt/trigger.                                                                                                                        |

## Parity Error Control Register

The MEC\_PERR\_CTL0 register bits control enable/disable for parity error inputs from various cores/peripherals and decide whether their status will be reflected in parity error status register bits.

Figure 49-17: MEC\_PERR\_CTL0 Register Diagram

<!-- image -->

Table 49-21: MEC\_PERR\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL0.LOCK bit is enabled, the MEC_PERR_CTL0 register is read on- ly. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL0.LOCK bit is enabled, the MEC_PERR_CTL0 register is read on- ly. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                           | Unlock                                                                                                                                      |
| 31 (R/W)           | LOCK       | 1                                                                                                                                           | Lock                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | Parity Error Control. The MEC_PERR_CTL0.VALUE bit indicates whether parity error control is enabled (=1) or disabled (=0).                  | Parity Error Control. The MEC_PERR_CTL0.VALUE bit indicates whether parity error control is enabled (=1) or disabled (=0).                  |
| 30:0 (R/W)         | VALUE      | 1                                                                                                                                           | Core 0 L1 Cache RAM Parity Error                                                                                                            |
| 30:0 (R/W)         | VALUE      | 2                                                                                                                                           | Core 0 L2 Cache RAM Parity Error                                                                                                            |
| 30:0 (R/W)         | VALUE      | 4                                                                                                                                           | Core 1 L1 RAM Parity Error                                                                                                                  |
| 30:0 (R/W)         | VALUE      | 8                                                                                                                                           | Core 1 L1 Cache Parity Error                                                                                                                |
| 30:0 (R/W)         | VALUE      | 16                                                                                                                                          | Core 1 Branch Predictor Parity Error                                                                                                        |
| 30:0 (R/W)         | VALUE      | 32                                                                                                                                          | Core 2 L1 RAM Parity Error                                                                                                                  |
| 30:0 (R/W)         | VALUE      | 64                                                                                                                                          | Core 2 L1 Cache Parity Error                                                                                                                |
| 30:0 (R/W)         | VALUE      | 128                                                                                                                                         | Core 2 Branch Predictor Parity Error                                                                                                        |
| 30:0 (R/W)         | VALUE      | 256                                                                                                                                         | ASRC0 FIFO Parity Error                                                                                                                     |
| 30:0 (R/W)         | VALUE      | 512                                                                                                                                         | ASRC1 FIFO Parity Error                                                                                                                     |

Table 49-21: MEC\_PERR\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
|                    |            |                      1024 | ASRC2 FIFO Parity Error        |
|                    |            |                      2048 | ASRC3 FIFO Parity Error        |
|                    |            |                      4096 | ASRC4 FIFO Parity Error        |
|                    |            |                      8192 | ASRC5 FIFO Parity Error        |
|                    |            |                     16384 | ASRC6 FIFO Parity Error        |
|                    |            |                     32768 | ASRC7 FIFO Parity Error        |
|                    |            |                     65536 | SHARC 0 IIR0 RAM Parity Error  |
|                    |            |                    131072 | SHARC 0 IIR1 RAM Parity Error  |
|                    |            |                    262144 | SHARC 0 IIR2 RAM Parity Error  |
|                    |            |                    524288 | SHARC 0 IIR3 RAM Parity Error  |
|                    |            |                   1048576 | SHARC 0 FIR0 RAM Parity Error  |
|                    |            |                   2097152 | SHARC 1 IIR0 RAM Parity Error  |
|                    |            |                   4194304 | SHARC 1 IIR1 RAM Parity Error  |
|                    |            |                   8388608 | SHARC 1 IIR2 RAM Parity Error  |
|                    |            |                  16777216 | SHARC 1 IIR3 RAM Parity Error  |
|                    |            |                  33554432 | SHARC 1 FIR0 RAM Parity Error  |
|                    |            |                  67108864 | USB0 FIFO RAM Parity Error     |
|                    |            |                 134217728 | TRNG0 Data Buffer Parity Error |
|                    |            |                 268435456 | PKA0 Data RAM Parity Error     |
|                    |            |                 536870912 | SPE0 Buffer RAM Parity Error   |
|                    |            |                1073741824 | SPE0 ARC4 RAM Parity Error     |

## Parity Error Control Register

MEC\_PERR\_CTL1 register bits control enable/disable for parity error inputs from various cores/peripherals and decide whether their status will be reflected in parity error status register bits.

Figure 49-18: MEC\_PERR\_CTL1 Register Diagram

<!-- image -->

Table 49-22: MEC\_PERR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL1.LOCK bit is enabled, the MEC_PERR_CTL1 register is read on- ly. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL1.LOCK bit is enabled, the MEC_PERR_CTL1 register is read on- ly. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                           | Unlock                                                                                                                                      |
| 31 (R/W)           | LOCK       | 1                                                                                                                                           | Lock                                                                                                                                        |
| 6:0 (R/W)          | VALUE1     | Parity Error Control. The MEC_PERR_CTL1.VALUE1 field indicates the parity error inputs. 1 indicates enable and 0 indicates disable.         | Parity Error Control. The MEC_PERR_CTL1.VALUE1 field indicates the parity error inputs. 1 indicates enable and 0 indicates disable.         |
| 6:0 (R/W)          | VALUE1     | 1                                                                                                                                           | EMAC0 Transmit FIFO RAM Parity Error                                                                                                        |
| 6:0 (R/W)          | VALUE1     | 2                                                                                                                                           | EMAC0 Receive FIFO RAM Parity Error                                                                                                         |
| 6:0 (R/W)          | VALUE1     | 4                                                                                                                                           | MLB0 Data Buffer RAM Parity Error                                                                                                           |
| 6:0 (R/W)          | VALUE1     | 8                                                                                                                                           | MLB0 Channel Table RAM Parity Error                                                                                                         |
| 6:0 (R/W)          | VALUE1     | 16                                                                                                                                          | TMC0 Trace Data RAM Parity Error                                                                                                            |
| 6:0 (R/W)          | VALUE1     | 32                                                                                                                                          | EMAC1 Transmit FIFO RAM Parity Error                                                                                                        |
| 6:0 (R/W)          | VALUE1     | 64                                                                                                                                          | EMAC1 Receive FIFO RAM Parity Error                                                                                                         |

## Parity Error Interrupt Mask Register

The MEC\_PERR\_IMASK0 register bits control interrupt masks for parity error inputs from various cores/peripherals.

Figure 49-19: MEC\_PERR\_IMASK0 Register Diagram

<!-- image -->

Table 49-23: MEC\_PERR\_IMASK0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Parity Error Interrupt Mask0 Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK0.LOCK bit is enabled, the MEC_PERR_IMASK0 register is read only. | Parity Error Interrupt Mask0 Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK0.LOCK bit is enabled, the MEC_PERR_IMASK0 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                   | Unlock                                                                                                                                                                              |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                   | Lock                                                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | Parity Error Interrupt Mask. The MEC_PERR_IMASK0.VALUE bit field, when set (=1), indicates an interrupt masked and, when cleared (=0), indicates an interrupt unmasked.             | Parity Error Interrupt Mask. The MEC_PERR_IMASK0.VALUE bit field, when set (=1), indicates an interrupt masked and, when cleared (=0), indicates an interrupt unmasked.             |
| 30:0 (R/W)         | VALUE      | 1                                                                                                                                                                                   | Core 0 L1 Cache RAM Parity Error                                                                                                                                                    |
| 30:0 (R/W)         | VALUE      | 2                                                                                                                                                                                   | Core 0 L1 CACHE Parity Error                                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | 4                                                                                                                                                                                   | Core 1 L1 RAM Parity Error                                                                                                                                                          |
| 30:0 (R/W)         | VALUE      | 8                                                                                                                                                                                   | Core 1 L1 Cache Parity Error                                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | 16                                                                                                                                                                                  | Core 1 Branch Predictor Parity Error                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | 32                                                                                                                                                                                  | Core 2 L1 RAM Parity Error                                                                                                                                                          |
| 30:0 (R/W)         | VALUE      | 64                                                                                                                                                                                  | Core 2 L1 Cache Parity Error                                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | 128                                                                                                                                                                                 | Core 2 Branch Predictor Parity Error                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | 256                                                                                                                                                                                 | ASRC0 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 512                                                                                                                                                                                 | ASRC1 FIFO Parity Error                                                                                                                                                             |

Table 49-23: MEC\_PERR\_IMASK0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
|                    |            |                      1024 | ASRC2 FIFO Parity Error        |
|                    |            |                      2048 | ASRC3 FIFO Parity Error        |
|                    |            |                      4096 | ASRC4 FIFO Parity Error        |
|                    |            |                      8192 | ASRC5 FIFO Parity Error        |
|                    |            |                     16384 | ASRC6 FIFO Parity Error        |
|                    |            |                     32768 | ASRC7 FIFO Parity Error        |
|                    |            |                     65536 | SHARC 0 IIR0 RAM Parity Error  |
|                    |            |                    131072 | SHARC 0 IIR1 RAM Parity Error  |
|                    |            |                    262144 | SHARC 0 IIR2 RAM Parity Error  |
|                    |            |                    524288 | SHARC 0 IIR3 RAM Parity Error  |
|                    |            |                   1048576 | SHARC 0 FIR0 RAM Parity Error  |
|                    |            |                   2097152 | SHARC 1 IIR0 RAM Parity Error  |
|                    |            |                   4194304 | SHARC 1 IIR1 RAM Parity Error  |
|                    |            |                   8388608 | SHARC 1 IIR2 RAM Parity Error  |
|                    |            |                  16777216 | SHARC 1 IIR3 RAM Parity Error  |
|                    |            |                  33554432 | SHARC1 FIR0 RAM Parity Error   |
|                    |            |                  67108864 | USB0 FIFO RAM Parity Error     |
|                    |            |                 134217728 | TRNG0 Data Buffer Parity Error |
|                    |            |                 268435456 | PKA0 Data RAM Parity Error     |
|                    |            |                 536870912 | SPE0 Buffer RAM Parity Error   |
|                    |            |                1073741824 | SPE0 ARC4 RAM Parity Error     |

## Parity Error Interrupt Mask Register

MEC\_PERR\_IMASK1 register bits control interrupt masks for parity error inputs from various cores/peripherals.

Figure 49-20: MEC\_PERR\_IMASK1 Register Diagram

<!-- image -->

Table 49-24: MEC\_PERR\_IMASK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK1.LOCK bit is enabled, the MEC_PERR_IMASK1 register is read only.                                                                             |
| 6:0 (R/W)          | VALUE      | 1 Lock Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked. 1 EMAC0 Transmit FIFO RAM Parity Error 2 EMAC0 Receive FIFO RAM Parity Error 4 MLB0 Data Buffer RAM Parity Error |

## Parity Error Status Register

The MEC\_PERR\_STAT0 register bits reflect status for parity error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding parity error status.

Figure 49-21: MEC\_PERR\_STAT0 Register Diagram

<!-- image -->

Table 49-25: MEC\_PERR\_STAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT0.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL0.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT0.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL0.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                         | No Error                                                                                                                                                                                                                                                                                                                  |
| 31 (R/W1C)         | LWERR      | 1                                                                                                                                                                                                                                                                                                                         | Error Occurred                                                                                                                                                                                                                                                                                                            |
| 30:0 (R/W1C)       | VALUE      | Parity Error Status. When set (=1), the MEC_PERR_STAT0.VALUE bit indicates the parity error status. When cleared (=0), the bit indicates no error. This is a sticky bit; write 1 to clear.                                                                                                                                | Parity Error Status. When set (=1), the MEC_PERR_STAT0.VALUE bit indicates the parity error status. When cleared (=0), the bit indicates no error. This is a sticky bit; write 1 to clear.                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 1                                                                                                                                                                                                                                                                                                                         | Core 0 L1 Cache RAM Parity Error                                                                                                                                                                                                                                                                                          |
| 30:0 (R/W1C)       | VALUE      | 2                                                                                                                                                                                                                                                                                                                         | Core 0 L2 Cache RAM Parity Error                                                                                                                                                                                                                                                                                          |
| 30:0 (R/W1C)       | VALUE      | 4                                                                                                                                                                                                                                                                                                                         | Core 1 L1 RAM Parity Error                                                                                                                                                                                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 8                                                                                                                                                                                                                                                                                                                         | Core 1 L1 Cache Parity Error                                                                                                                                                                                                                                                                                              |
| 30:0 (R/W1C)       | VALUE      | 16                                                                                                                                                                                                                                                                                                                        | Core 1 Branch Predictor Parity Error                                                                                                                                                                                                                                                                                      |
| 30:0 (R/W1C)       | VALUE      | 32                                                                                                                                                                                                                                                                                                                        | Core 2 L1 RAM Parity Error                                                                                                                                                                                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 64                                                                                                                                                                                                                                                                                                                        | Core 2 L1 Cache Parity Error                                                                                                                                                                                                                                                                                              |
| 30:0 (R/W1C)       | VALUE      | 128                                                                                                                                                                                                                                                                                                                       | Core 2 Branch Predictor Parity Error                                                                                                                                                                                                                                                                                      |
| 30:0 (R/W1C)       | VALUE      | 256                                                                                                                                                                                                                                                                                                                       | ASRC0 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |

Table 49-25: MEC\_PERR\_STAT0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
|                    |            |                       512 | ASRC1 FIFO Parity Error        |
|                    |            |                      1024 | ASRC2 FIFO Parity Error        |
|                    |            |                      2048 | ASRC3 FIFO Parity Error        |
|                    |            |                      4096 | ASRC4 FIFO Parity Error        |
|                    |            |                      8192 | ASRC5 FIFO Parity Error        |
|                    |            |                     16384 | ASRC6 FIFO Parity Error        |
|                    |            |                     32768 | ASRC7 FIFO Parity Error        |
|                    |            |                     65536 | SHARC 0 IIR0 RAM Parity Error  |
|                    |            |                    131072 | SHARC 0 IIR1 RAM Parity Error  |
|                    |            |                    262144 | SHARC 0 IIR2 RAM Parity Error  |
|                    |            |                    524288 | SHARC 0 IIR3 RAM Parity Error  |
|                    |            |                   1048576 | SHARC 0 FIR0 RAM Parity Error  |
|                    |            |                   2097152 | SHARC 1 IIR0 RAM Parity Error  |
|                    |            |                   4194304 | SHARC 1 IIR1 RAM Parity Error  |
|                    |            |                   8388608 | SHARC 1 IIR2 RAM Parity Error  |
|                    |            |                  16777216 | SHARC 1 IIR3 RAM Parity Error  |
|                    |            |                  33554432 | SHARC 1 FIR0 RAM Parity Error  |
|                    |            |                  67108864 | USB0 FIFO RAM Parity Error     |
|                    |            |                 134217728 | TRNG0 Data Buffer Parity Error |
|                    |            |                 268435456 | PKA0 Data RAM Parity Error     |
|                    |            |                 536870912 | SPE0 Buffer RAM Parity Error   |
|                    |            |                1073741824 | SPE0 ARC4 RAM Parity Error     |

## Parity Error Status Register

MEC\_PERR\_STAT1 register bits reflect status for parity error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding parity error status.

Figure 49-22: MEC\_PERR\_STAT1 Register Diagram

<!-- image -->

Table 49-26: MEC\_PERR\_STAT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT1.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL1.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT1.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL1.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                         | No Error                                                                                                                                                                                                                                                                                                                  |
| 6:0 (R/W1C)        | VALUE      | Parity Error Status bit. 1 indicates error and 0 indicates no error. Sticky bit, write 1 to clear.                                                                                                                                                                                                                        | Parity Error Status bit. 1 indicates error and 0 indicates no error. Sticky bit, write 1 to clear.                                                                                                                                                                                                                        |
| 6:0 (R/W1C)        | VALUE      | 1                                                                                                                                                                                                                                                                                                                         | EMAC0 Transmit FIFO RAM Parity Error                                                                                                                                                                                                                                                                                      |
| 6:0 (R/W1C)        | VALUE      | 2                                                                                                                                                                                                                                                                                                                         | EMAC0 Receive FIFO RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 6:0 (R/W1C)        | VALUE      | 4                                                                                                                                                                                                                                                                                                                         | MLB0 Data Buffer RAM Parity Error                                                                                                                                                                                                                                                                                         |
| 6:0 (R/W1C)        | VALUE      | 8                                                                                                                                                                                                                                                                                                                         | MLB0 Channel Table RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 6:0 (R/W1C)        | VALUE      | 16                                                                                                                                                                                                                                                                                                                        | TMC0 Trace Data RAM Parity Error                                                                                                                                                                                                                                                                                          |
| 6:0 (R/W1C)        | VALUE      | 32                                                                                                                                                                                                                                                                                                                        | EMAC1 Transmit FIFO RAM Parity Error                                                                                                                                                                                                                                                                                      |
| 6:0 (R/W1C)        | VALUE      | 64                                                                                                                                                                                                                                                                                                                        | EMAC1 Receive FIFO RAM Parity Error                                                                                                                                                                                                                                                                                       |

## Peripheral ID0 Register

Figure 49-23: MEC\_PID0 Register Diagram

<!-- image -->

Table 49-27: MEC\_PID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 7:0                | PARTNUM    | Part Number.                                                                        |
| (R/NW)             |            | The MEC_PID0.PARTNUM field indicates the part number for component identifi- cation |

## Peripheral ID1 Register

Figure 49-24: MEC\_PID1 Register Diagram

<!-- image -->

Table 49-28: MEC\_PID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | JEP106IC   | JEDEC JEP106 Identity. The MEC_PID1.JEP106IC field indicates JEDEC JEP106 identity (manufacturer ID) code. |
| 3:0 (R/NW)         | PARTNUM    | Part Number. The MEC_PID1.PARTNUM field indicates the part number for component identifi- cation           |

## Peripheral ID2 Register

Figure 49-25: MEC\_PID2 Register Diagram

<!-- image -->

Table 49-29: MEC\_PID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | REV        | Peripheral Revision.                                                                                       |
| 3 (R/NW)           | JEDECASGN  | JEDEC Assigned Value.                                                                                      |
| 2:0 (R/NW)         | JEP106IC   | JEDEC JEP106 Identity. The MEC_PID2.JEP106IC field indicates JEDEC JEP106 identity (manufacturer ID) code. |

## Peripheral ID3 Register

Figure 49-26: MEC\_PID3 Register Diagram

<!-- image -->

Table 49-30: MEC\_PID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4 (R/NW)         | REVAND     | Metal Fix Revision.       |
| 3:0 (R/NW)         | CUSTMOD    | Customer Modified.        |

## Peripheral ID4 Register

Figure 49-27: MEC\_PID4 Register Diagram

<!-- image -->

Table 49-31: MEC\_PID4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | SIZE       | Number of 4k Blocks. The MEC_PID4.SIZE field indicates the size of component 4k chunks minus 1 (for example, 0=4k).          |
| 3:0 (R/NW)         | JEP106CC   | JEDEC Continuation Code. The MEC_PID4.JEP106CC field indicates the JEDEC JEP106 continuation code (number of leading 0x7Fs). |