# Memory Error Protection Unit (MEPU)

<!-- source: 324_Memory_Error_Protection_Unit_MEPU.pdf | original pages 3884–3923 -->

## 50   Memory Error Protection Unit (MEPU)

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

## ADSP-SC59x MEC Register List

The Memory Error Controller (MEC) manages memory parity/ inputs from the core and peripherals and sends interrupt/trigger outputs. It is a generic N x M multiplexer and interrupt controller for error to interrupt/trigger outputs. It has control features such as enable/disable and interrupt masking, and status signaling for inputs/outputs. For more information on MEC functionality, see the MEC register descriptions.

Table 50-1: ADSP-SC59x MEC Register List

| Name                 | Description                                                      |
|----------------------|------------------------------------------------------------------|
| MEC_A55EIRQ_GCTL[n]  | A55 Core Memory Errors Interrupt Request Global Control Register |
| MEC_A55EIRQ_GSTAT[n] | A55 Error Interrupt Request Global Status Register               |
| MEC_A55EIRQ_STAT[n]  | A55 Error Status Register                                        |
| MEC_A55ERR_CTL[n]    | A55 Error Control Register                                       |
| MEC_A55ERR_IMASK[n]  | A55 Error Interrupt Mask Register                                |
| MEC_CID0             | Component ID0 Register                                           |
| MEC_CID1             | Component ID1 Register                                           |
| MEC_CID2             | Component ID2 Register                                           |
| MEC_CID3             | Component ID3 Register                                           |
| MEC_CLR              | Clear Register                                                   |
| MEC_ECCERR_CTL[n]    | ECC Error Control Register                                       |
| MEC_ECCERR_IMASK[n]  | ECC Error Interrupt Mask Register                                |
| MEC_ECCERR_STAT[n]   | ECC Error Status Register                                        |
| MEC_EEIRQ_GCTL[n]    | ECC Error Interrupt Request Global Control Register              |
| MEC_EEIRQ_GSTAT[n]   | ECC Error Interrupt Request Global Status Register               |
| MEC_PEIRQ_GCTL[n]    | Parity Error Interrupt Request Global Control Register           |
| MEC_PEIRQ_GSTAT[n]   | Parity Error Interrupt Request Global Status Register            |
| MEC_PERR_CTL0        | Parity Error Control Register                                    |
| MEC_PERR_CTL1        | Parity Error Control Register                                    |
| MEC_PERR_IMASK0      | Parity Error Interrupt Mask Register                             |
| MEC_PERR_IMASK1      | Parity Error Interrupt Mask Register                             |
| MEC_PERR_STAT0       | Parity Error Status Register                                     |
| MEC_PERR_STAT1       | Parity Error Status Register                                     |
| MEC_PID0             | Peripheral ID0 Register                                          |
| MEC_PID1             | Peripheral ID1 Register                                          |
| MEC_PID2             | Peripheral ID2 Register                                          |
| MEC_PID3             | Peripheral ID3 Register                                          |
| MEC_PID4             | Peripheral ID4 Register                                          |

## MEPU Block Diagram

The MEPU Block Diagram shows the functional blocks within the Memory Error Protection Unit (MEPU) unit.

Figure 50-1: MEPU Unit Block Diagram

<!-- image -->

## MEC Block Diagram

The MEC Interface Block Diagram shows the functional blocks within the MEC module.

Figure 50-2: MEC Unit Block Diagram

<!-- image -->

## PCTL Block Diagram

The PCTL Block Diagram shows the functional blocks within the PCTL module.

Figure 50-3: PCTL Block Diagram

<!-- image -->

## MECx Input and Output Block Diagram

The MECx Input and Output Block Diagram shows the input and output routing to various cores.

Figure 50-4: MECx Input and Output Block Diagram

<!-- image -->

## MEC Architectural Concepts

The MEC unit communicates with the Cortex-A55 core directly through the subsystem's SYS bus crossbar. It services MMR addresses associated with the MEC function unit and translates the addresses into specific transcendental MEC functions. The results are read from common result registers shared by all the functions in single-precision floating-point format. The block stalls all read accesses until the result is ready.

## MEC Configuration

The following tables describe the range of values for configuration and the mapping of parity errors.

Table 50-2: Mapping of Input Parity Errors (PERR)

|   PERR ID | Name           | Description                                      |
|-----------|----------------|--------------------------------------------------|
|         0 | Reserved       | Not used                                         |
|         1 | Reserved       | Not used                                         |
|         2 | PERR_C1_L1R    | Core1 (SHARC0) L1 RAM Parity Error               |
|         3 | PERR_C1_L1CR   | Core1 (SHARC0) L1 Cache RAM Parity Error         |
|         4 | PERR_C1_BPR    | Core1 (SHARC0) Branch Predictor RAM Parity Error |
|         5 | PERR_C2_L1R    | Core2 (SHARC1) L1 RAM Parity Error               |
|         6 | PERR_C2_L1CR   | Core2 (SHARC1) L1 Cache RAM Parity Error         |
|         7 | PERR_C2_BPR    | Core2 (SHARC1) Branch Predictor RAM Parity Error |
|         8 | PERR_ASRC0_FR  | ASRC0 FIFO RAM Parity Error                      |
|         9 | PERR_ASRC1_FR  | ASRC1 FIFO RAM Parity Error                      |
|        10 | PERR_ASRC2_FR  | ASRC2 FIFO RAM Parity Error                      |
|        11 | PERR_ASRC3_FR  | ASRC3 FIFO RAM Parity Error                      |
|        12 | PERR_ASRC4_FR  | ASRC3 FIFO RAM Parity Error                      |
|        13 | PERR_ASRC5_FR  | ASRC3 FIFO RAM Parity Error                      |
|        14 | PERR_ASRC6_FR  | ASRC3 FIFO RAM Parity Error                      |
|        15 | PERR_ASRC7_FR  | ASRC3 FIFO RAM Parity Error                      |
|        16 | PERR_C0_IIR0_R | SH0 IIR0 RAM Parity Error                        |
|        17 | PERR_C0_IIR1_R | SH0 IIR1 RAM Parity Error                        |
|        18 | PERR_C0_IIR2_R | SH0 IIR2 RAM Parity Error                        |
|        19 | PERR_C0_IIR3_R | SH0 IIR3 RAM Parity Error                        |
|        20 | PERR_C0_FIR0_R | FIR0 RAM Parity Error                            |
|        21 | PERR_C1_IIR0_R | SH0 IIR0 RAM Parity Error                        |
|        22 | PERR_C1_IIR1_R | SH0 IIR1 RAM Parity Error                        |
|        23 | PERR_C1_IIR2_R | SH0 IIR2 RAM Parity Error                        |

Table 50-2: Mapping of Input Parity Errors (PERR) (Continued)

|   PERR ID | Name            | Description                          |
|-----------|-----------------|--------------------------------------|
|        24 | PERR_C1_IIR3_R  | SH0 IIR3 RAM Parity Error            |
|        25 | PERR_C1_FIR0_R  | FIR0 RAM Parity Error                |
|        26 | PERR_USB0_FR    | USB0 FIFO RAM Parity Error           |
|        27 | PERR_TRNG0_DBR  | TRNG0 Data Buffer RAM Parity Error   |
|        28 | PERR_PKA0_DR    | PKA0 Data RAM Parity Error           |
|        29 | PERR_SPE0_BR    | SPE0 Buffer RAM Parity Error         |
|        30 | PERR_SPE0_AR    | SPE0 ARC4 RAM Parity Error           |
|        31 | PERR_EMAC0_TFR  | EMAC0 Transmit FIFO RAM Parity Error |
|        32 | PERR_EMAC0_RFR  | EMAC0 Receive FIFO RAM Parity Error  |
|        33 | PERR_MLB0_DBR   | MLB0 Data Buffer RAM Parity Error    |
|        34 | PERR_MLB0_CTR   | MLB0 Channel Table RAM Parity Error  |
|        35 | PERR_TMC0_TDR   | TMC0 Trace Data RAM Parity Error     |
|        36 | PERR_EMAC1_TFR  | EMAC1 Transmit FIFO RAM Parity Error |
|        37 | PERR_EMAC1_RFR  | EMAC1 Receive FIFO RAM Parity Error  |
|        38 | PERR_EMAC0_TMI  | EMAC0 TMI FIFO RAM Parity Error      |
|        39 | PERR_ EMAC0_EST | EMAC0 EST FIFO RAM Parity Error      |
|        40 | PERR_EMSI0      | EMSI0 FIFO RAM Parity Error          |

Table 50-3: Mapping of Output Parity Errors (PERR)

|   PERR ID | Name      | Description                              |
|-----------|-----------|------------------------------------------|
|         0 | PEIRQ_C0  | Core0 (Arm) Parity Error Interrupt       |
|         1 | PEIRQ_C1  | Core1 (SHARC0) Parity Error Interrupt    |
|         2 | PEIRQ_C2  | Core2 (SHARC1) Parity Error Interrupt    |
|         3 | PEIRQ_SYS | System Peripheral Parity Error Interrupt |

Table 50-4: Mapping of Input ECC Errors (ECCERR)

|   PERR ID | Name          | Description      |
|-----------|---------------|------------------|
|         0 | ECCERR_L2CTL0 | L2CTL0 ECC Error |
|         1 | ECCERR_CAN0   | CAN0 ECC Error   |
|         2 | ECCERR_CAN1   | CAN1 ECC Error   |

Table 50-5: Mapping of Output ECC Errors (EEIRQ)

|   PERR ID | Name   | Description         |
|-----------|--------|---------------------|
|         0 | EEIRQ  | ECC Error Interrupt |

Table 50-6: Mapping of Input A55 Core Memory Errors (A55ERR)

|   PERR ID | Name         | Description     |
|-----------|--------------|-----------------|
|         0 | A55_SCU_ERR  | A55 SCU Error   |
|         1 | A55_L1L2_ERR | A55 L1,L2 Error |

Table 50-7: Mapping of Output A55 Core Memory Errors (A55EIRQ)

|   PERR ID | Name    | Description                     |
|-----------|---------|---------------------------------|
|         0 | A55EIRQ | A55 Core Memory Error Interrupt |

## PCTL Integration

There is one PCTL instance per port per memory instance (for example, one PCTL instance for single port memory and two PCTL instances for dual port memories).

For A55 core memories initialization is done by built in hardware. So software initialization is not required.

## ADSP-SC59x MEC Register Descriptions

Memory Error Controller (MEC) contains the following registers.

Table 50-8: ADSP-SC59x MEC Register List

| Name                 | Description                                                      |
|----------------------|------------------------------------------------------------------|
| MEC_A55EIRQ_GCTL[n]  | A55 Core Memory Errors Interrupt Request Global Control Register |
| MEC_A55EIRQ_GSTAT[n] | A55 Error Interrupt Request Global Status Register               |
| MEC_A55EIRQ_STAT[n]  | A55 Error Status Register                                        |
| MEC_A55ERR_CTL[n]    | A55 Error Control Register                                       |
| MEC_A55ERR_IMASK[n]  | A55 Error Interrupt Mask Register                                |
| MEC_CID0             | Component ID0 Register                                           |
| MEC_CID1             | Component ID1 Register                                           |
| MEC_CID2             | Component ID2 Register                                           |
| MEC_CID3             | Component ID3 Register                                           |
| MEC_CLR              | Clear Register                                                   |
| MEC_ECCERR_CTL[n]    | ECC Error Control Register                                       |
| MEC_ECCERR_IMASK[n]  | ECC Error Interrupt Mask Register                                |

Table 50-8: ADSP-SC59x MEC Register List (Continued)

| Name               | Description                                            |
|--------------------|--------------------------------------------------------|
| MEC_ECCERR_STAT[n] | ECC Error Status Register                              |
| MEC_EEIRQ_GCTL[n]  | ECC Error Interrupt Request Global Control Register    |
| MEC_EEIRQ_GSTAT[n] | ECC Error Interrupt Request Global Status Register     |
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

## A55 Core Memory Errors Interrupt Request Global Control Register

The MEC\_A55EIRQ\_GCTL[n] register bits control the core memory error interrupt and trigger outputs.

Figure 50-5: MEC\_A55EIRQ\_GCTL[n] Register Diagram

<!-- image -->

Table 50-9: MEC\_A55EIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55EIRQ_GCTL[n].LOCK bit is enabled, the MEC_A55EIRQ_GCTL[n] register is read only. 0 Unlock |
| 0 (R/W)            | VALUE      | A55 Memory Error Global Control Enable.. The MEC_A55EIRQ_GCTL[n].VALUE bit enables the A55 memory error global control. 0 Disable                              |

## A55 Error Interrupt Request Global Status Register

The MEC\_A55EIRQ\_GSTAT[n] register bits reflect the status of the core memory error interrupt and trigger outputs.

Figure 50-6: MEC\_A55EIRQ\_GSTAT[n] Register Diagram

<!-- image -->

Table 50-10: MEC\_A55EIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | A55 Memory Error Global Control Register Lock Write Error. When set (=1), the MEC_A55EIRQ_GSTAT[n].LWERR bit indicates that there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 0 (R/W)            | VALUE      | A55 Error Global Status. When set (=1), the MEC_A55EIRQ_GSTAT[n].VALUE bit indicates the assertion of an interrupt/trigger. When cleared (=0), there is no interrupt or trigger. 0 No Interrupt/Trigger 1 Interrupt/Trigger Asserted                                                                                                             |

## A55 Error Status Register

The MEC\_A55EIRQ\_STAT[n] register bits reflect the status of the core memory error inputs.

Writing '1' to these bits clears the corresponding error status. (W1C)

Figure 50-7: MEC\_A55EIRQ\_STAT[n] Register Diagram

<!-- image -->

Table 50-11: MEC\_A55EIRQ\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | A55 Memory Error Global Control Register Lock Write Error. The MEC_A55EIRQ_STAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | A55 Memory Error Global Control Register Lock Write Error. The MEC_A55EIRQ_STAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 1:0 (R/W)          | VALUE      | Error Status. The MEC_A55EIRQ_STAT[n].VALUE field indicates the status of an interrupt/ trigger. When set (=1), an interrupt or trigger is asserted. When cleared (=0), there is no interrupt or trigger.                                                                                                                     | Error Status. The MEC_A55EIRQ_STAT[n].VALUE field indicates the status of an interrupt/ trigger. When set (=1), an interrupt or trigger is asserted. When cleared (=0), there is no interrupt or trigger.                                                                                                                     |
| 1:0 (R/W)          | VALUE      | 1                                                                                                                                                                                                                                                                                                                             | A55 SCU Error                                                                                                                                                                                                                                                                                                                 |
| 1:0 (R/W)          | VALUE      | 2                                                                                                                                                                                                                                                                                                                             | A55 L1,L2 Error                                                                                                                                                                                                                                                                                                               |

## A55 Error Control Register

The MEC\_A55ERR\_CTL[n] register bits control the core memory error inputs and whether their status will be reflected in the register.

Figure 50-8: MEC\_A55ERR\_CTL[n] Register Diagram

<!-- image -->

Table 50-12: MEC\_A55ERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | A55 Memory Error Control Register Lock Bit. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55ERR_CTL[n].LOCK bit is enabled, the MEC_A55ERR_CTL[n] regis- ter is read only.              |
| 1:0 (R/W)          | VALUE      | A55 Memory Error Control. The MEC_A55ERR_CTL[n].VALUE field enables A55 memory error control. When set (=1), control is enabled. When cleared (=0), control is disabled. 1 A55 SCU Err 2 A55 L1,L2 Err |

## A55 Error Interrupt Mask Register

The MEC\_A55ERR\_IMASK[n] register bits control interrupt masks for memory error inputs from the core.

Figure 50-9: MEC\_A55ERR\_IMASK[n] Register Diagram

<!-- image -->

Table 50-13: MEC\_A55ERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | A55 Memory Error Control Register Lock Bit. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55ERR_IMASK[n].LOCK bit is enabled, the MEC_A55ERR_IMASK[n] register is read only. |
| 1:0 (R/W)          | VALUE      | A55 Memory Error Interrupt Mask. The MEC_A55ERR_IMASK[n].VALUE field indicates whether an interrupt is masked (when set (=1)) or unmasked (=0). 1 A55 SCU Err 2 A55 L1,L2 Err               |

## Component ID0 Register

The MEC\_CID0 register is a component ID register.

Figure 50-10: MEC\_CID0 Register Diagram

<!-- image -->

Table 50-14: MEC\_CID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID0.PREAMBLE field indicates the component ID preamble. |

## Component ID1 Register

The MEC\_CID1 register is a component ID register.

Figure 50-11: MEC\_CID1 Register Diagram

<!-- image -->

Table 50-15: MEC\_CID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | COMPCLASS  | Component Class. The MEC_CID1.COMPCLASS field indicates the component class. Dedicated debug blocks (core debug access port, Program Trace, etc.) should identify as CoreSight (i.e. 0x9) and implement the full compliment of CoreSight registers including DEVTYPE. All other ADI components should identify as System (i.e. 0xF) components. See Core- Sight Architecture Specification for details. 9 CoreSight 15 System |
| 3:0 (R/NW)         | PREAMBLE   | Component ID Preamble. The MEC_CID1.PREAMBLE field indicates the component ID preamble.                                                                                                                                                                                                                                                                                                                                       |

## Component ID2 Register

The MEC\_CID2 register is a component ID register.

Figure 50-12: MEC\_CID2 Register Diagram

<!-- image -->

Table 50-16: MEC\_CID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID2.PREAMBLE field indicates the component ID preamble. |

## Component ID3 Register

The MEC\_CID3 register is a component ID register.

Figure 50-13: MEC\_CID3 Register Diagram

<!-- image -->

Table 50-17: MEC\_CID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID3.PREAMBLE field indicates the component ID preamble. |

## Clear Register

The MEC\_CLR register clears all the MEC status registers. Before enabling any of the MEC control registers, write to this register in the software.

Figure 50-14: MEC\_CLR Register Diagram

<!-- image -->

Table 50-18: MEC\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 0                  | CLRSTAT    | Clear Status.                                                              |
| (R0/W)             |            | Write of '1' to the MEC_CLR.CLRSTAT bit clears all status registers of MEC |

## ECC Error Control Register

MEC\_ECCERR\_CTL[n] register bits control enable/disable for ECC error inputs from various cores/peripherals and decide whether their status will be reflected in ECC error status register bits.

Figure 50-15: MEC\_ECCERR\_CTL[n] Register Diagram

<!-- image -->

Table 50-19: MEC\_ECCERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_CTL[n].LOCK bit is enabled, the MEC_ECCERR_CTL[n] regis- ter is read only. 0 Unlock |
| 5:0 (R/W)          | VALUE      | ECC Error Control. The MEC_ECCERR_CTL[n].VALUE bit indicates whether ECC control is enabled (=1) or disabled (=0). 1 L2 CTL ECC Error 2 CAN0 ECC Error 4 CAN1 ECC Error                 |

## ECC Error Interrupt Mask Register

MEC\_ECCERR\_IMASK[n] register bits control interrupt masks for ECC error inputs from various cores/peripherals.

Figure 50-16: MEC\_ECCERR\_IMASK[n] Register Diagram

<!-- image -->

Table 50-20: MEC\_ECCERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Interrupt Mask Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_IMASK[n].LOCK bit is enabled, the MEC_ECCERR_IMASK[n] register is read only. |
| 31 (R/W)           | LOCK       | 0 Unlock                                                                                                                                                                                |
| 5:0 (R/W)          | VALUE      | ECC Error Interrupt Mask. The MEC_ECCERR_IMASK[n].VALUE bit indicates when the ECC error inter- rupt is masked (=1) or unmasked (=0). ECC Error                                         |
| 5:0 (R/W)          | VALUE      | 1 L2 CTL                                                                                                                                                                                |
| 5:0 (R/W)          | VALUE      | 2 CAN0 ECC Error                                                                                                                                                                        |
| 5:0 (R/W)          | VALUE      | 4 CAN1 ECC Error                                                                                                                                                                        |
| 5:0 (R/W)          | VALUE      | 8 Reserved                                                                                                                                                                              |

## ECC Error Status Register

MEC\_ECCERR\_STAT[n] register bits reflect status for ECC error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding ECC error status.

Figure 50-17: MEC\_ECCERR\_STAT[n] Register Diagram

<!-- image -->

Table 50-21: MEC\_ECCERR\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | ECC Error Control or Interrupt Mask Register Lock Write Error. The MEC_ECCERR_STAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_ECCERR_CTL[n].LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. | ECC Error Control or Interrupt Mask Register Lock Write Error. The MEC_ECCERR_STAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_ECCERR_CTL[n].LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                                     | No Error                                                                                                                                                                                                                                                                                                                              |
| 5:0 (R/W1C)        | VALUE      | ECC Error Status. '1' indicates error and '0' indicates no error. Sticky bit, write '1' to clear.                                                                                                                                                                                                                                     | ECC Error Status. '1' indicates error and '0' indicates no error. Sticky bit, write '1' to clear.                                                                                                                                                                                                                                     |
| 5:0 (R/W1C)        | VALUE      | 1                                                                                                                                                                                                                                                                                                                                     | L2 CTL ECC Error                                                                                                                                                                                                                                                                                                                      |
| 5:0 (R/W1C)        | VALUE      | 2                                                                                                                                                                                                                                                                                                                                     | CAN0 ECC Error                                                                                                                                                                                                                                                                                                                        |
| 5:0 (R/W1C)        | VALUE      | 4                                                                                                                                                                                                                                                                                                                                     | CAN1 ECC Error                                                                                                                                                                                                                                                                                                                        |
| 5:0 (R/W1C)        | VALUE      | 8                                                                                                                                                                                                                                                                                                                                     | Reserved                                                                                                                                                                                                                                                                                                                              |

## ECC Error Interrupt Request Global Control Register

MEC\_EEIRQ\_GCTL[n] register bits control enable/disable of ECC error interrupt/trigger outputs.

Figure 50-18: MEC\_EEIRQ\_GCTL[n] Register Diagram

<!-- image -->

Table 50-22: MEC\_EEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Global Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_EEIRQ_GCTL[n].LOCK bit is enabled, the MEC_EEIRQ_GCTL[n] regis- ter is read only. |
| 0 (R/W)            | VALUE      | ECC Error Global Control. '1' indicates enable and '0' indicates disable.                                                                                                             |

## ECC Error Interrupt Request Global Status Register

MEC\_EEIRQ\_GSTAT[n] register bits reflect status of ECC error interrupt/trigger outputs.

Figure 50-19: MEC\_EEIRQ\_GSTAT[n] Register Diagram

<!-- image -->

Table 50-23: MEC\_EEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | ECC Error Global Control Register Lock Write Error. The MEC_EEIRQ_GSTAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 0 (R/NW)           | VALUE      | ECC Error Global Status. '1' indicates assertion of interrupt/trigger and '0' indicates no interrupt/trigger.                                                                                                                                                                                                                         |

## Parity Error Interrupt Request Global Control Register

MEC\_PEIRQ\_GCTL[n] register bits control enable/disable of parity error interrupt/trigger outputs.

Figure 50-20: MEC\_PEIRQ\_GCTL[n] Register Diagram

<!-- image -->

Table 50-24: MEC\_PEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Parity Error Global Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PEIRQ_GCTL[n].LOCK bit is enabled, the MEC_PEIRQ_GCTL[n] regis- ter is read only. |
| 3:0 (R/W)          | VALUE      | Parity Error Global Control. The MEC_PEIRQ_GCTL[n].VALUE field indicates parity error global control. '1' indicates enable and '0' indicates disable.                                    |

## Parity Error Interrupt Request Global Status Register

MEC\_PEIRQ\_GSTAT[n] register bits reflect status of parity error interrupt/trigger outputs.

Figure 50-21: MEC\_PEIRQ\_GSTAT[n] Register Diagram

<!-- image -->

Table 50-25: MEC\_PEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Global Control Register Lock Write Error. The MEC_PEIRQ_GSTAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_PEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 3:0 (R/NW)         | VALUE      | Parity Error Global Status. The MEC_PEIRQ_GSTAT[n].VALUE field indicates the parity error global status. '1' indicates assertion of interrupt/trigger, '0' indicates no interrupt/trigger.                                                                                                                                               |

## Parity Error Control Register

MEC\_PERR\_CTL0 register bits control enable/disable for parity error inputs from various cores/peripherals and decide whether their status will be reflected in parity error status register bits.

Figure 50-22: MEC\_PERR\_CTL0 Register Diagram

<!-- image -->

Table 50-26: MEC\_PERR\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL0.LOCK bit is enabled, the MEC_PERR_CTL0 register is read on- ly. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL0.LOCK bit is enabled, the MEC_PERR_CTL0 register is read on- ly. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                           | Unlock                                                                                                                                      |
| 31 (R/W)           | LOCK       | 1                                                                                                                                           | Lock                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | Parity Error Control. 1 indicates enable and 0 indicates disable.                                                                           | Parity Error Control. 1 indicates enable and 0 indicates disable.                                                                           |
| 30:0 (R/W)         | VALUE      | 4                                                                                                                                           | Core 1 L1 RAM Parity Error                                                                                                                  |
| 30:0 (R/W)         | VALUE      | 8                                                                                                                                           | Core 1 L1 Cache Parity Error                                                                                                                |
| 30:0 (R/W)         | VALUE      | 16                                                                                                                                          | Core 1 Branch Predictor Parity Error                                                                                                        |
| 30:0 (R/W)         | VALUE      | 32                                                                                                                                          | Core 2 L1 RAM Parity Error                                                                                                                  |
| 30:0 (R/W)         | VALUE      | 64                                                                                                                                          | Core 2 L1 Cache Parity Error                                                                                                                |
| 30:0 (R/W)         | VALUE      | 128                                                                                                                                         | Core 2 Branch Predictor Parity Error                                                                                                        |
| 30:0 (R/W)         | VALUE      | 256                                                                                                                                         | ASRC0 FIFO Parity Error                                                                                                                     |
| 30:0 (R/W)         | VALUE      | 512                                                                                                                                         | ASRC1 FIFO Parity Error                                                                                                                     |
| 30:0 (R/W)         | VALUE      | 1024                                                                                                                                        | ASRC2 FIFO Parity Error                                                                                                                     |
| 30:0 (R/W)         | VALUE      | 2048                                                                                                                                        | ASRC3 FIFO Parity Error                                                                                                                     |
| 30:0 (R/W)         | VALUE      | 4096                                                                                                                                        | ASRC4 FIFO Parity Error                                                                                                                     |

Table 50-26: MEC\_PERR\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
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

Figure 50-23: MEC\_PERR\_CTL1 Register Diagram

<!-- image -->

Table 50-27: MEC\_PERR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL1.LOCK bit is enabled, the MEC_PERR_CTL1 register is read on- ly. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL1.LOCK bit is enabled, the MEC_PERR_CTL1 register is read on- ly. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                           | Unlock                                                                                                                                      |
| 31 (R/W)           | LOCK       | 1                                                                                                                                           | Lock                                                                                                                                        |
| 9:0 (R/W)          | VALUE1     | Parity Error Control. The MEC_PERR_CTL1.VALUE1 field indicates the parity error inputs. 1 indicates enable and 0 indicates disable.         | Parity Error Control. The MEC_PERR_CTL1.VALUE1 field indicates the parity error inputs. 1 indicates enable and 0 indicates disable.         |
| 9:0 (R/W)          | VALUE1     | 1                                                                                                                                           | EMAC0 Transmit FIFO RAM Parity Error                                                                                                        |
| 9:0 (R/W)          | VALUE1     | 2                                                                                                                                           | EMAC0 Receive FIFO RAM Parity Error                                                                                                         |
| 9:0 (R/W)          | VALUE1     | 4                                                                                                                                           | MLB0 Data Buffer RAM Parity Error                                                                                                           |
| 9:0 (R/W)          | VALUE1     | 8                                                                                                                                           | MLB0 Channel Table RAM Parity Error                                                                                                         |
| 9:0 (R/W)          | VALUE1     | 16                                                                                                                                          | TMC0 Trace Data RAM Parity Error                                                                                                            |
| 9:0 (R/W)          | VALUE1     | 32                                                                                                                                          | EMAC1 Transmit FIFO RAM Parity Error                                                                                                        |
| 9:0 (R/W)          | VALUE1     | 64                                                                                                                                          | EMAC1 Receive FIFO RAM Parity Error                                                                                                         |
| 9:0 (R/W)          | VALUE1     | 128                                                                                                                                         | EMAC0 TMI FIFO RAM Parity Error                                                                                                             |
| 9:0 (R/W)          | VALUE1     | 256                                                                                                                                         | EMAC0 EST FIFO RAM Parity Error                                                                                                             |
| 9:0 (R/W)          | VALUE1     | 512                                                                                                                                         | EMSI0 FIFO RAM Parity Error                                                                                                                 |

## Parity Error Interrupt Mask Register

MEC\_PERR\_IMASK0 register bits control interrupt masks for parity error inputs from various cores/peripherals.

Figure 50-24: MEC\_PERR\_IMASK0 Register Diagram

<!-- image -->

Table 50-28: MEC\_PERR\_IMASK0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Parity Error Interrupt Mask0 Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK0.LOCK bit is enabled, the MEC_PERR_IMASK0 register is read only. | Parity Error Interrupt Mask0 Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK0.LOCK bit is enabled, the MEC_PERR_IMASK0 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                   | Unlock                                                                                                                                                                              |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                   | Lock                                                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked.                                                                                   | Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked.                                                                                   |
| 30:0 (R/W)         | VALUE      | 4                                                                                                                                                                                   | Core 1 L1 RAM Parity Error                                                                                                                                                          |
| 30:0 (R/W)         | VALUE      | 8                                                                                                                                                                                   | Core 1 L1 Cache Parity Error                                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | 16                                                                                                                                                                                  | Core 1 Branch Predictor Parity Error                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | 32                                                                                                                                                                                  | Core 2 L1 RAM Parity Error                                                                                                                                                          |
| 30:0 (R/W)         | VALUE      | 64                                                                                                                                                                                  | Core 2 L1 Cache Parity Error                                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | 128                                                                                                                                                                                 | Core 2 Branch Predictor Parity Error                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | 256                                                                                                                                                                                 | ASRC0 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 512                                                                                                                                                                                 | ASRC1 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 1024                                                                                                                                                                                | ASRC2 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 2048                                                                                                                                                                                | ASRC3 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 4096                                                                                                                                                                                | ASRC4 FIFO Parity Error                                                                                                                                                             |

Table 50-28: MEC\_PERR\_IMASK0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
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

Figure 50-25: MEC\_PERR\_IMASK1 Register Diagram

<!-- image -->

Table 50-29: MEC\_PERR\_IMASK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK1.LOCK bit is enabled, the MEC_PERR_IMASK1 register is read only. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK1.LOCK bit is enabled, the MEC_PERR_IMASK1 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                             | Unlock                                                                                                                                        |
| 31 (R/W)           | LOCK       | 1                                                                                                                                             | Lock                                                                                                                                          |
| 9:0 (R/W)          | VALUE      | Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked.                                             | Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked.                                             |
| 9:0 (R/W)          | VALUE      | 1                                                                                                                                             | EMAC0 Transmit FIFO RAM Parity Error                                                                                                          |
| 9:0 (R/W)          | VALUE      | 2                                                                                                                                             | EMAC0 Receive FIFO RAM Parity Error                                                                                                           |
| 9:0 (R/W)          | VALUE      | 4                                                                                                                                             | MLB0 Data Buffer RAM Parity Error                                                                                                             |
| 9:0 (R/W)          | VALUE      | 8                                                                                                                                             | MLB0 Channel Table RAM Parity Error                                                                                                           |
| 9:0 (R/W)          | VALUE      | 16                                                                                                                                            | TMC0 Trace Data RAM Parity Error                                                                                                              |
| 9:0 (R/W)          | VALUE      | 32                                                                                                                                            | EMAC1 Transmit FIFO RAM Parity Error                                                                                                          |
| 9:0 (R/W)          | VALUE      | 64                                                                                                                                            | EMAC1 Receive FIFO RAM Parity Error                                                                                                           |
| 9:0 (R/W)          | VALUE      | 128                                                                                                                                           | EMAC0 TMI FIFO RAM Parity Error                                                                                                               |
| 9:0 (R/W)          | VALUE      | 256                                                                                                                                           | EMAC0 EST FIFO RAM Parity Error                                                                                                               |
| 9:0 (R/W)          | VALUE      | 512                                                                                                                                           | EMSI0 FIFO RAM Parity Error                                                                                                                   |

## Parity Error Status Register

MEC\_PERR\_STAT0 register bits reflect status for parity error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding parity error status.

Figure 50-26: MEC\_PERR\_STAT0 Register Diagram

<!-- image -->

Table 50-30: MEC\_PERR\_STAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT0.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL0.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT0.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL0.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                         | No Error                                                                                                                                                                                                                                                                                                                  |
| 31 (R/W1C)         | LWERR      | 1                                                                                                                                                                                                                                                                                                                         | Error Occurred                                                                                                                                                                                                                                                                                                            |
| 30:0 (R/W1C)       | VALUE      | Parity Error Status bit. '1' indicates error and '0' indicates no error. Sticky bit, write '1' to clear.                                                                                                                                                                                                                  | Parity Error Status bit. '1' indicates error and '0' indicates no error. Sticky bit, write '1' to clear.                                                                                                                                                                                                                  |
| 30:0 (R/W1C)       | VALUE      | 4                                                                                                                                                                                                                                                                                                                         | Core 1 L1 RAM Parity Error                                                                                                                                                                                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 8                                                                                                                                                                                                                                                                                                                         | Core 1 L1 Cache Parity Error                                                                                                                                                                                                                                                                                              |
| 30:0 (R/W1C)       | VALUE      | 16                                                                                                                                                                                                                                                                                                                        | Core 1 Branch Predictor Parity Error                                                                                                                                                                                                                                                                                      |
| 30:0 (R/W1C)       | VALUE      | 32                                                                                                                                                                                                                                                                                                                        | Core 2 L1 RAM Parity Error                                                                                                                                                                                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 64                                                                                                                                                                                                                                                                                                                        | Core 2 L1 Cache Parity Error                                                                                                                                                                                                                                                                                              |
| 30:0 (R/W1C)       | VALUE      | 128                                                                                                                                                                                                                                                                                                                       | Core 2 Branch Predictor Parity Error                                                                                                                                                                                                                                                                                      |
| 30:0 (R/W1C)       | VALUE      | 256                                                                                                                                                                                                                                                                                                                       | ASRC0 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 512                                                                                                                                                                                                                                                                                                                       | ASRC1 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 1024                                                                                                                                                                                                                                                                                                                      | ASRC2 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 2048                                                                                                                                                                                                                                                                                                                      | ASRC3 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |

Table 50-30: MEC\_PERR\_STAT0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
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

Figure 50-27: MEC\_PERR\_STAT1 Register Diagram

<!-- image -->

Table 50-31: MEC\_PERR\_STAT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT1.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL1.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT1.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL1.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                         | No Error                                                                                                                                                                                                                                                                                                                  |
| 31 (R/W1C)         | LWERR      | 1                                                                                                                                                                                                                                                                                                                         | Error Occurred                                                                                                                                                                                                                                                                                                            |
| 9:0 (R/W1C)        | VALUE      | Parity Error Status bit. 1 indicates error and 0 indicates no error. Sticky bit, write 1 to clear.                                                                                                                                                                                                                        | Parity Error Status bit. 1 indicates error and 0 indicates no error. Sticky bit, write 1 to clear.                                                                                                                                                                                                                        |
| 9:0 (R/W1C)        | VALUE      | 1                                                                                                                                                                                                                                                                                                                         | EMAC0 Transmit FIFO RAM Parity Error                                                                                                                                                                                                                                                                                      |
| 9:0 (R/W1C)        | VALUE      | 2                                                                                                                                                                                                                                                                                                                         | EMAC0 Receive FIFO RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 9:0 (R/W1C)        | VALUE      | 4                                                                                                                                                                                                                                                                                                                         | MLB0 Data Buffer RAM Parity Error                                                                                                                                                                                                                                                                                         |
| 9:0 (R/W1C)        | VALUE      | 8                                                                                                                                                                                                                                                                                                                         | MLB0 Channel Table RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 9:0 (R/W1C)        | VALUE      | 16                                                                                                                                                                                                                                                                                                                        | TMC0 Trace Data RAM Parity Error                                                                                                                                                                                                                                                                                          |
| 9:0 (R/W1C)        | VALUE      | 32                                                                                                                                                                                                                                                                                                                        | EMAC1 Transmit FIFO RAM Parity Error                                                                                                                                                                                                                                                                                      |
| 9:0 (R/W1C)        | VALUE      | 64                                                                                                                                                                                                                                                                                                                        | EMAC1 Receive FIFO RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 9:0 (R/W1C)        | VALUE      | 128                                                                                                                                                                                                                                                                                                                       | EMAC0 TMI FIFO RAM Parity Error                                                                                                                                                                                                                                                                                           |
| 9:0 (R/W1C)        | VALUE      | 256                                                                                                                                                                                                                                                                                                                       | EMAC0 EST FIFO RAM Parity Error                                                                                                                                                                                                                                                                                           |
| 9:0 (R/W1C)        | VALUE      | 512                                                                                                                                                                                                                                                                                                                       | EMSI0 FIFO RAM Parity Error                                                                                                                                                                                                                                                                                               |

## Peripheral ID0 Register

The MEC\_PID0 register is a peripheral ID register.

Figure 50-28: MEC\_PID0 Register Diagram

<!-- image -->

Table 50-32: MEC\_PID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                   |
|--------------------|------------|-------------------------------------------|
| 7:0                | PARTNUM    | Part Number for component identification. |

## Peripheral ID1 Register

The MEC\_PID1 register is a peripheral ID register.

Figure 50-29: MEC\_PID1 Register Diagram

<!-- image -->

Table 50-33: MEC\_PID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                       |
|--------------------|------------|-----------------------------------------------|
| 7:4                | JEP106IC   | JEDEC JEP106 identity (Manufacturer ID) code. |
| 3:0 (R/NW)         | PARTNUM    | Part Number for component identification.     |

## Peripheral ID2 Register

The MEC\_PID2 register is a peripheral ID register.

Figure 50-30: MEC\_PID2 Register Diagram

<!-- image -->

Table 50-34: MEC\_PID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                       |
|--------------------|------------|-----------------------------------------------|
| 7:4 (R/NW)         | REV        | Peripheral Revision.                          |
| 3 (R/NW)           | JEDECASGN  | JEDEC Assigned Value.                         |
| 2:0 (R/NW)         | JEP106IC   | JEDEC JEP106 identity (Manufacturer ID) code. |

## Peripheral ID3 Register

The MEC\_PID3 register is a peripheral ID register.

Figure 50-31: MEC\_PID3 Register Diagram

<!-- image -->

Table 50-35: MEC\_PID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | REVAND     | Metal fix revision.       |
| 3:0 (R/NW)         | CUSTMOD    | Customer Modified.        |

## Peripheral ID4 Register

The MEC\_PID4 register is a peripheral ID register.

Figure 50-32: MEC\_PID4 Register Diagram

<!-- image -->

Table 50-36: MEC\_PID4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 7:4 (R/NW)         | SIZE       | Number of 4k Blocks. Size of component 4k chunks minus 1 (i.e. 0=4k) |
| 3:0 (R/NW)         | JEP106CC   | JEDEC JEP106 continuation code (number of leading 0x7Fs).            |