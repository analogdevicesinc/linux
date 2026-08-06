## 47   Memory Error Protection Unit (MEPU)

Memory Error Protection Unit (MEPU) handles single-bit and double-bit memory error detection and correction across the memories in the chip and controls routing of their interrupts/triggers.

It consists of following blocks:

- Memory Error Controller (MEC)
- Parity Controller (PCTL)
- ECC Controller (ECTL)

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

## ADSP-2184x MEC Register List

The Memory Error Controller (MEC) manages memory parity/ inputs from the core and peripherals and sends interrupt/trigger outputs. It is a generic N x M multiplexer and interrupt controller for error to interrupt/trigger outputs. It has control features such as enable/disable and interrupt masking, and status signaling for inputs/outputs. For more information on MEC functionality, see the MEC register descriptions.

Table 47-1: ADSP-2184x MEC Register List

| Name                   | Description                                                      |
|------------------------|------------------------------------------------------------------|
| MEC_A55C0EIRQ_GCTL[n]  | A55 Core Memory Errors Interrupt Request Global Control Register |
| MEC_A55C0EIRQ_GSTAT[n] | A55 Error Interrupt Request Global Status Register               |
| MEC_A55C0ERR_CTL[n]    | A55 Error Control Register                                       |
| MEC_A55C0ERR_IMASK[n]  | A55 Error Interrupt Mask Register                                |
| MEC_A55C0ERR_STAT[n]   | A55 Error Status Register                                        |
| MEC_A55C2EIRQ_GCTL[n]  | A55 Core Memory Errors Interrupt Request Global Control Register |
| MEC_A55C2EIRQ_GSTAT[n] | A55 Error Interrupt Request Global Status Register               |
| MEC_A55C2ERR_CTL[n]    | A55 Error Control Register                                       |
| MEC_A55C2ERR_IMASK[n]  | A55 Error Interrupt Mask Register                                |
| MEC_A55C2ERR_STAT[n]   | A55 Error Status Register                                        |
| MEC_CID0               | Component ID0 Register                                           |
| MEC_CID1               | Component ID1 Register                                           |
| MEC_CID2               | Component ID2 Register                                           |
| MEC_CID3               | Component ID3 Register                                           |
| MEC_CLR                | Clear Register                                                   |
| MEC_CMEIRQ_GCTL[n]     | Core Mem ECC Error Interrupt Request Global Control Register     |
| MEC_CMEIRQ_GSTAT[n]    | Core Memory ECC Error Interrupt Request Global Status Register   |
| MEC_CMERR_CTL[n]       | Core Memory ECC Error Control Register                           |
| MEC_CMERR_IMASK[n]     | Core Memory ECC Error Interrupt Mask Register                    |
| MEC_CMERR_STAT[n]      | Core Memory ECC Error Status Register                            |
| MEC_ECCERR_CTL[n]      | ECC Error Control Register                                       |
| MEC_ECCERR_IMASK[n]    | ECC Error Interrupt Mask Register                                |
| MEC_ECCERR_STAT[n]     | ECC Error Status Register                                        |
| MEC_EEIRQ_GCTL[n]      | ECC Error Interrupt Request Global Control Register              |
| MEC_EEIRQ_GSTAT[n]     | ECC Error Interrupt Request Global Status Register               |
| MEC_PEIRQ_GCTL[n]      | Parity Error Interrupt Request Global Control Register           |
| MEC_PEIRQ_GSTAT[n]     | Parity Error Interrupt Request Global Status Register            |
| MEC_PERR_CTL0          | Parity Error Control Register                                    |
| MEC_PERR_CTL1          | Parity Error Control Register                                    |
| MEC_PERR_IMASK0        | Parity Error Interrupt Mask Register                             |
| MEC_PERR_IMASK1        | Parity Error Interrupt Mask Register                             |

Table 47-1: ADSP-2184x MEC Register List (Continued)

| Name           | Description                  |
|----------------|------------------------------|
| MEC_PERR_STAT0 | Parity Error Status Register |
| MEC_PERR_STAT1 | Parity Error Status Register |
| MEC_PID0       | Peripheral ID0 Register      |
| MEC_PID1       | Peripheral ID1 Register      |
| MEC_PID2       | Peripheral ID2 Register      |
| MEC_PID3       | Peripheral ID3 Register      |
| MEC_PID4       | Peripheral ID4 Register      |

## ADSP-2184x MEC Trigger List

Table 47-2: ADSP-2184x MEC Trigger List Generators

|   Trigger ID | Name        | Description                                          | Sensitivity   |
|--------------|-------------|------------------------------------------------------|---------------|
|           85 | MEC1_EEIRQ0 | MEC1 ECC Error Interrupt Request                     | Level         |
|           86 | MEC1_MEIRQ2 | MEC1 Memory Error Interrupt Request                  | Level         |
|           87 | MEC1_PEIRQ0 | MEC1 Parity Error Interrupt Request                  | Level         |
|           88 | MEC2_EEIRQ0 | MEC2 ECC Error Interrupt/Trigger Request Bus         | Level         |
|           89 | MEC2_MEIRQ2 | MEC2 Memory Error Interrupt Request                  | Level         |
|           90 | MEC2_PEIRQ0 | MEC2 Parity Error Interrupt/Trigger Request Bus      | Level         |
|           91 | MEC1_MEIRQ1 | MEC1 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           92 | MEC2_MEIRQ1 | MEC2 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           93 | MEC1_MEIRQ0 | MEC1 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           94 | MEC2_MEIRQ0 | MEC2 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           95 | MEC0_EEIRQ0 | MEC0 ECC Error Interrupt/Trigger Request Bus         | Level         |
|           96 | MEC0_MEIRQ2 | MEC0 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           97 | MEC0_PEIRQ0 | MEC0 Parity Error Interrupt Request                  | Level         |
|           98 | MEC0_MEIRQ1 | MEC0 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           99 | MEC0_MEIRQ0 | MEC0 Core Memory Error Interrupt/Trigger Request Bus | Level         |

Table 47-3: ADSP-2184x MEC Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## ADSP-2184x MEC Interrupt List

Table 47-4: ADSP-2184x MEC Interrupt List

|   Interrupt ID | Name         | Description                                                             | Sensitivity   | DMA Channel   |
|----------------|--------------|-------------------------------------------------------------------------|---------------|---------------|
|            183 | MEC1_EEIRQ0  | MEC1 ECC Error Interrupt Request                                        | Level         |               |
|            184 | MEC1_EWIRQ0  | MEC1 ECC Warning to Interrupt Re- quest (Reserved)                      | Level         |               |
|            185 | MEC1_MEIRQ1  | MEC1 Parity Error Interrupt Request                                     | Level         |               |
|            186 | MEC1_MFIRQ00 | MEC1 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            187 | MEC1_MEIRQ0  | MEC1 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            188 | MEC1_MFIRQ20 | MEC1 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            189 | MEC1_MEIRQ2  | MEC1 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            190 | MEC1_PEIRQ0  | MEC1 Parity Error Interrupt/Trigger Re- quest Bus                       | Level         |               |
|            191 | MEC2_EEIRQ0  | MEC2 ECC Error Interrupt/Trigger Re- quest Bus                          | Level         |               |
|            192 | MEC2_EWIRQ0  | MEC2 ECC Warning Interrupt/Trigger Request Bus (Reserved)               | Level         |               |
|            193 | MEC2_MWIRQ1  | MEC2 Core Memory ECC Warning In- terrupt/Trigger Request Bus (Reserved) | Level         |               |
|            194 | MEC2_MEIRQ1  | MEC2 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            195 | MEC2_MFIRQ00 | MEC2 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            196 | MEC2_MEIRQ0  | MEC2 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            197 | MEC2_MFIRQ20 | MEC2 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            198 | MEC2_MEIRQ2  | MEC2 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |

Table 47-4: ADSP-2184x MEC Interrupt List (Continued)

|   Interrupt ID | Name         | Description                                                             | Sensitivity   | DMA Channel   |
|----------------|--------------|-------------------------------------------------------------------------|---------------|---------------|
|            199 | MEC2_PEIRQ0  | MEC2 Parity Error Interrupt/Trigger Re- quest Bus                       | Level         |               |
|            200 | MEC1_MWIRQ1  | MEC1 MEC1 Core 1 Mem Warning to Core 1 (Reserved)                       | Level         |               |
|            201 | MEC0_EEIRQ0  | MEC0 ECC Error Interrupt/Trigger Re- quest Bus                          | Level         |               |
|            202 | MEC0_EWIRQ0  | MEC0 ECC Warning Interrupt/Trigger Request Bus (Reserved)               | Level         |               |
|            203 | MEC0_MWIRQ1  | MEC0 Core Memory ECC Warning In- terrupt/Trigger Request Bus (Reserved) | Level         |               |
|            204 | MEC0_MEIRQ1  | MEC0 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            205 | MEC0_MFIRQ00 | MEC0 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            206 | MEC0_MEIRQ0  | MEC0 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            207 | MEC0_MFIRQ20 | MEC0 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            208 | MEC0_MEIRQ2  | MEC0 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            209 | MEC0_PEIRQ0  | MEC0 Parity Error Interrupt/Trigger Re- quest Bus                       | Level         |               |

## MEPU Block Diagram

The MEPU Block Diagram shows the functional blocks within the Memory Error Protection Unit (MEPU) unit.

Figure 47-1: MEPU Unit Block Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000000_98c22f86b5fcbc924af86a3ea48b1d55ac0a3c818c212c42c05eb95a232629f5.png)

## MEC Block Diagram

The MEC Interface Block Diagram shows the functional blocks within the MEC module.

Figure 47-2: MEC Unit Block Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000001_8ed952662fec75b6873695d3a8ba965c1c3be264d2be7f86d9c0e5ad34601f7c.png)

## PCTL Block Diagram

The PCTL Block Diagram shows the functional blocks within the PCTL module.

Figure 47-3: PCTL Block Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000002_d692c37c6387f01b2a30de748e324dc20ae8677c1332c36708ddea4f6ec22b68.png)

## MECx Input and Output Block Diagram

The MECx Input and Output Block Diagram shows the input and output routing to various cores.

Figure 47-4: MECx Input and Output Block Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000003_6b143e1f886524f36737c0479e7e466cafe1e8dd481ed8963beb503a3dca09be.png)

## MEC Architectural Concepts

The MEC unit communicates with the Cortex-A55 core directly through the subsystem's SYS bus crossbar. It services MMR addresses associated with the MEC function unit and translates the addresses into specific transcendental MEC functions. The results are read from common result registers shared by all the functions in single-precision floating-point format. The block stalls all read accesses until the result is ready.

## MEC Configuration

The following tables describe the range of values for configuration and the mapping of parity errors.

Table 47-5: Mapping of Input Parity Errors (PERR)

|   PERR ID | Name         | Description                                      |
|-----------|--------------|--------------------------------------------------|
|         0 | Reserved     | Reserved                                         |
|         1 | Reserved     | Reserved                                         |
|         2 | PERR_C1_L1R  | Core1 (SHARC0) L1 RAM Parity Error               |
|         3 | PERR_C1_L1CR | Core1 (SHARC0) L1 Cache RAM Parity Error         |
|         4 | PERR_C1_BPR  | Core1 (SHARC0) Branch Predictor RAM Parity Error |

Table 47-5: Mapping of Input Parity Errors (PERR) (Continued)

|   PERR ID | Name           | Description                          |
|-----------|----------------|--------------------------------------|
|         5 | Reserved       | Reserved                             |
|         6 | Reserved       | Reserved                             |
|         7 | Reserved       | Reserved                             |
|         8 | PERR_ASRC0_FR  | ASRC0 FIFO RAM Parity Error          |
|         9 | PERR_ASRC1_FR  | ASRC1 FIFO RAM Parity Error          |
|        10 | PERR_ASRC2_FR  | ASRC2 FIFO RAM Parity Error          |
|        11 | PERR_ASRC3_FR  | ASRC3 FIFO RAM Parity Error          |
|        12 | PERR_ASRC4_FR  | ASRC4 FIFO RAM Parity Error          |
|        13 | PERR_ASRC5_FR  | ASRC5 FIFO RAM Parity Error          |
|        14 | PERR_ASRC6_FR  | ASRC6 FIFO RAM Parity Error          |
|        15 | PERR_ASRC7_FR  | ASRC7 FIFO RAM Parity Error          |
|        16 | PERR_C0_IIR0_R | SH0 IIR0 RAM Parity Error            |
|        17 | PERR_C0_IIR1_R | SH0 IIR1 RAM Parity Error            |
|        18 | PERR_C0_IIR2_R | SH0 IIR2 RAM Parity Error            |
|        19 | PERR_C0_IIR3_R | SH0 IIR3 RAM Parity Error            |
|        20 | PERR_C0_FIR0_R | FIR0 RAM Parity Error                |
|        21 | Reserved       | Reserved                             |
|        22 | Reserved       | Reserved                             |
|        23 | Reserved       | Reserved                             |
|        24 | Reserved       | Reserved                             |
|        25 | PERR_C0_FIR1_R | FIR1 RAM Parity Error                |
|        26 | Reserved       | Reserved                             |
|        27 | PERR_TRNG0_DBR | TRNG0 Data Buffer RAM Parity Error   |
|        28 | PERR_PKA0_DR   | PKA0 Data RAM Parity Error           |
|        29 | PERR_SPE0_BR   | SPE0 Buffer RAM Parity Error         |
|        30 | PERR_SPE0_AR   | SPE0 ARC4 RAM Parity Error           |
|        31 | PERR_EMAC0_TFR | EMAC0 Transmit FIFO RAM Parity Error |
|        32 | PERR_EMAC0_RFR | EMAC0 Receive FIFO RAM Parity Error  |
|        33 | PERR_MLB0_DBR  | MLB0 Data Buffer RAM Parity Error    |
|        34 | PERR_MLB0_CTR  | MLB0 Channel Table RAM Parity Error  |
|        35 | PERR_TMC0_TDR  | TMC0 Trace Data RAM Parity Error     |

Table 47-5: Mapping of Input Parity Errors (PERR) (Continued)

|   PERR ID | Name             | Description                  |
|-----------|------------------|------------------------------|
|        36 | Reserved         | Reserved                     |
|        37 | Reserved         | Reserved                     |
|        38 | PERR_EMAC0_TMI   | EMAC0 TMI Parity Error       |
|        39 | PERR_EMAC0_TST   | EMAC0 TST Parity Error       |
|        40 | PERR_MSHC        | MSHC Parrity Error           |
|        41 | PERR_ASRC8_FR    | ASRC8 FIFO RAM Parity Error  |
|        42 | PERR_ASRC9_FR    | ASRC9 FIFO RAM Parity Error  |
|        43 | PERR_ASRC10_FR   | ASRC10 FIFO RAM Parity Error |
|        44 | PERR_ASRC11_FR   | ASRC11 FIFO RAM Parity Error |
|        45 | PERR_ASRC12_FR   | ASRC12 FIFO RAM Parity Error |
|        47 | PERR_ASRC13_FR   | ASRC13 FIFO RAM Parity Error |
|        47 | PERR_ASRC14_FR   | ASRC14 FIFO RAM Parity Error |
|        48 | PERR_ASRC15_FR   | ASRC15 FIFO RAM Parity Error |
|        49 | PERR_XSPI0_FLASH | XSPI0 Flash Ram Parity Error |
|        50 | PERR_XSPI0_CTRL  | XSPI0 CTRL Ram Parity Error  |
|        51 | PERR_XSPI1_FLASH | XSPI1 Flash Ram Parity Error |
|        52 | PERR_XSPI1_CTRL  | XSPI1 CTRL Ram Parity Error  |
|        53 | Reserved         | Reserved                     |
|        54 | Reserved         | Reserved                     |
|        55 | PERR_LPDDR_ICCM  | LPDDR ICCM Parity Error      |
|        56 | PERR_LPDDR_DCCM  | LPDDR DCCMParity Error       |

Table 47-6: Mapping of Output Parity Errors (PERR)

|   PERR ID | Name      | Description                              |
|-----------|-----------|------------------------------------------|
|         0 | PEIRQ_SYS | System Peripheral Parity Error Interrupt |

Table 47-7: Mapping of Input ECC Errors (ECCERR)

|   PERR ID | Name          | Description      |
|-----------|---------------|------------------|
|         0 | ECCERR_L2CTL0 | L2CTL0 ECC Error |
|         1 | ECCERR_CAN0   | CAN0 ECC Error   |
|         2 | ECCERR_CAN1   | CAN1 ECC Error   |
|         3 | GIC ERR       | GIC Error        |

Table 47-7: Mapping of Input ECC Errors (ECCERR) (Continued)

|   PERR ID | Name          | Description      |
|-----------|---------------|------------------|
|         4 | ECCERR_L2CTL1 | L2CTL1 ECC Error |

Table 47-8: Mapping of Output ECC Errors (EEIRQ)

|   PERR ID | Name   | Description         |
|-----------|--------|---------------------|
|         0 | EWIRQ  | ECC Error Interrupt |

Table 47-9: Mapping of Input Core Memory ECC Errors (CMERR)

|   CMERR ID | Name                | Description                                   |
|------------|---------------------|-----------------------------------------------|
|          7 | ECCERR_DRAM         | Core1 DRAM ECC Error Uncorrectable            |
|          6 | ECCERR_IRAM         | Core1 iRAM ECC Error Uncorrectable            |
|          4 | ECCERR_DCACHE_DIRTY | Core1 dCache ECC Error Dirty Uncorrectable    |
|          3 | ECCERR_ICACHE       | Core1 iCache ECC Error Uncorrectable          |
|          2 | ECCERR_DP_CLEAN     | Core1 dPref RAM ECC Error Clean Uncorrectable |
|          1 | ECCERR_DP_DIRTY     | Core1 dPref RAM ECC Error Dirty Uncorrectable |
|          0 | ECCERR_IP_CLEAN     | Core1 iPref RAM ECC Error Clean Uncorrectable |

Table 47-10: Mapping of Input A55 Core n Memory Errors (A55CnERR); n=0, 2

|   CMERR ID | Name         | Description   |
|------------|--------------|---------------|
|          0 | A55 SCU ERR  | A55 SCU ERR   |
|          1 | A55_L1L2_ERR | A55_L1L2_ERR  |

Table 47-11: Mapping of Output A55 Core n Memory Errors (A55CnEIRQ); n=0, 2

|   CMERR IRQ ID | Name      | Description                       |
|----------------|-----------|-----------------------------------|
|              0 | A55CnEIRQ | A55 Core n Memory Error Interrupt |

## Error Status and Interrupts

There are two categories of memory errors from the SHARC-FX core:

- Recoverable errors - includes single bit ECC errors and parity and dual bit ECC errors in buffers that can be dropped or refilled from the source.
- Unrecoverable errors - includes the remaining dual bit ECC errors

As the errors occur, this information is relayed over PFaultInfo and sent out to the SEC module via the MPEU module. From the SEC, these signals can be sent out to an external fault pin.

There is an additional signal (PFatalError) that is asserted when the core is stuck in a double/triple exception. The signal directly goes to the SEC module.

The following table lists the recoverable ECC errors in the SHARC-FX core.

Table 47-12: Recoverable Core Memory ECC Errors

|   SL Number | Fault Category   | Type Fault PFaultInfo[50:32]       | Description                         |
|-------------|------------------|------------------------------------|-------------------------------------|
|           0 |                  | ECC error correctable 32           | 1-bit ECC error                     |
|           1 |                  | ECC error correctable 34           | 1-bit ECC error                     |
|           2 |                  | ECC error correctable 36           | 1-bit ECC error                     |
|           3 |                  | ECC error 37                       | Clean refill                        |
|           4 |                  | L1S Cache refill 40                | Refill from L1V cache               |
|           6 |                  | ICACHE ECC error correctable 41    | 1-bit ECC error                     |
|           7 |                  | ECC error refill 42                | 2-bit ECC error, refill from memory |
|           8 | DPREFETCH RAM    | DPREF RAM ECC error correctable 44 | 1-bit ECC error                     |
|           9 | DPREF RAM        | ECC error clean drop 45            | Clean drop                          |
|          10 | IPREFETCH RAM    | IPREF RAM ECC error correctable 48 | 1-bit ECC error                     |

Table 47-13: Unrecoverable Core Memory ECC Errors

|   SL Number | Fault Category   | Type Fault PFaultInfo[50:32]                 | Description                                     |
|-------------|------------------|----------------------------------------------|-------------------------------------------------|
|           1 |                  | DRAM ECC error uncorrectable 33              | 2-bit ECC error                                 |
|           2 |                  | IRAM ECC error uncorrectable 35              | 2-bit ECC error                                 |
|           3 |                  | DCACHE ECC error clean uncorrecta- ble 38    | 2-bit ECC error in unmodified cache line        |
|           4 |                  | ECC error dirty uncorrectable 39             | 2-bit ECC error in unmodified cache line        |
|           5 |                  | ICACHE ECC error uncorrectable 43            | 2-bit ECC error, cannot be refilled from memory |
|           6 | DPREFETCH RAM    | DPREF RAM ECC error clean uncor- rectable 46 | 2-bit ECC error in unmodified cache line        |
|           7 | DPREF RAM        | ECC error dirty uncorrectable 47             |                                                 |
|           8 | IPREFETCH RAM    | IPREF RAM ECC error uncorrectable 50         |                                                 |

NOTE: In the SH-FX configuration chosen for EHP , the following fault signals are not implemented and tied to 0 in the design and, hence, never occur.

- IPREF RAM ECC error clean drop PFaultInfo[49]
- DCACHE ECC error clean refill PFaultInfo [37]
- DPREF RAM ECC error clean drop PFaultInfo [45]
- DPREF ECC error clean uncorrectable PFaultInfo [46]

## Requester Port Bus Interrupts

Bus error responses from the requester ports of the SHARC-FX core are detected and routed to the SEC to raise faults. Although the SHARC-FX core can detect and raise exceptions, when there is a bus error response, there is good chance of data/instruction corruption. Therefore, the core exception handling dependency has been removed. The SEC is directly notified using interrupts.

## Instruction Bus Interrupt

Any instruction read error is detected on the instruction requester port; the information is forwarded to the SEC via the core1 instruction read interrupt.

## Data Bus Interrupts

Any data read or write error is detected on the data requester port; the information is forwarded to the SEC via the core1 data read interrupt and core1 data write interrupt.

## iDMA Bus Interrupts

Any data read or write error that is detected on the iDMA port is forwarded to the SEC via the core1 iDMA read interrupt and core1 iDMA write interrupt.

## PCTL Integration

There is one PCTL instance per port per memory instance (for example, one PCTL instance for single port memory and two PCTL instances for dual port memories).

Memory initialization control logic supported by PCTL is used only for instances attached to Arm L1 cache memories.

An additional software trigger requester and initialization trigger completer is provided in TRU for starting memory initialization. A pulse from this trigger completer starts initialization of Arm L1 cache memories. When all the locations of a particular memory instance get initialized, the PCTL generates a memory initialization done signal for the corresponding memory instance. These signals from all PCTL instances corresponding to Arm L1 cache memories are ANDed to generate a single memory initialization done output which is connected as an interrupt to SEC/GIC and a trigger requester to TRU.

## ADSP-2184x MEC Register Descriptions

Memory Error Controller (MEC) contains the following registers.

Table 47-14: ADSP-2184x MEC Register List

| Name                   | Description                                                      |
|------------------------|------------------------------------------------------------------|
| MEC_A55C0EIRQ_GCTL[n]  | A55 Core Memory Errors Interrupt Request Global Control Register |
| MEC_A55C0EIRQ_GSTAT[n] | A55 Error Interrupt Request Global Status Register               |
| MEC_A55C0ERR_CTL[n]    | A55 Error Control Register                                       |
| MEC_A55C0ERR_IMASK[n]  | A55 Error Interrupt Mask Register                                |

Table 47-14: ADSP-2184x MEC Register List (Continued)

| Name                   | Description                                                      |
|------------------------|------------------------------------------------------------------|
| MEC_A55C0ERR_STAT[n]   | A55 Error Status Register                                        |
| MEC_A55C2EIRQ_GCTL[n]  | A55 Core Memory Errors Interrupt Request Global Control Register |
| MEC_A55C2EIRQ_GSTAT[n] | A55 Error Interrupt Request Global Status Register               |
| MEC_A55C2ERR_CTL[n]    | A55 Error Control Register                                       |
| MEC_A55C2ERR_IMASK[n]  | A55 Error Interrupt Mask Register                                |
| MEC_A55C2ERR_STAT[n]   | A55 Error Status Register                                        |
| MEC_CID0               | Component ID0 Register                                           |
| MEC_CID1               | Component ID1 Register                                           |
| MEC_CID2               | Component ID2 Register                                           |
| MEC_CID3               | Component ID3 Register                                           |
| MEC_CLR                | Clear Register                                                   |
| MEC_CMEIRQ_GCTL[n]     | Core Mem ECC Error Interrupt Request Global Control Register     |
| MEC_CMEIRQ_GSTAT[n]    | Core Memory ECC Error Interrupt Request Global Status Register   |
| MEC_CMERR_CTL[n]       | Core Memory ECC Error Control Register                           |
| MEC_CMERR_IMASK[n]     | Core Memory ECC Error Interrupt Mask Register                    |
| MEC_CMERR_STAT[n]      | Core Memory ECC Error Status Register                            |
| MEC_ECCERR_CTL[n]      | ECC Error Control Register                                       |
| MEC_ECCERR_IMASK[n]    | ECC Error Interrupt Mask Register                                |
| MEC_ECCERR_STAT[n]     | ECC Error Status Register                                        |
| MEC_EEIRQ_GCTL[n]      | ECC Error Interrupt Request Global Control Register              |
| MEC_EEIRQ_GSTAT[n]     | ECC Error Interrupt Request Global Status Register               |
| MEC_PEIRQ_GCTL[n]      | Parity Error Interrupt Request Global Control Register           |
| MEC_PEIRQ_GSTAT[n]     | Parity Error Interrupt Request Global Status Register            |
| MEC_PERR_CTL0          | Parity Error Control Register                                    |
| MEC_PERR_CTL1          | Parity Error Control Register                                    |
| MEC_PERR_IMASK0        | Parity Error Interrupt Mask Register                             |
| MEC_PERR_IMASK1        | Parity Error Interrupt Mask Register                             |
| MEC_PERR_STAT0         | Parity Error Status Register                                     |
| MEC_PERR_STAT1         | Parity Error Status Register                                     |
| MEC_PID0               | Peripheral ID0 Register                                          |
| MEC_PID1               | Peripheral ID1 Register                                          |

Table 47-14: ADSP-2184x MEC Register List (Continued)

| Name     | Description             |
|----------|-------------------------|
| MEC_PID2 | Peripheral ID2 Register |
| MEC_PID3 | Peripheral ID3 Register |
| MEC_PID4 | Peripheral ID4 Register |

## A55 Core Memory Errors Interrupt Request Global Control Register

Figure 47-5: MEC\_A55C0EIRQ\_GCTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000004_281608dcb0a9f4a0c1a7db80b1d7d80439484c59cd09abbc3a6007d3e125bfe8.png)

Table 47-15: MEC\_A55C0EIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55C0EIRQ_GCTL[n].LOCK bit is enabled, the MEC_A55C0EIRQ_GCTL[n] register is read only. 0 Unlock |
| 0 (R/W)            | VALUE      | A55 Memory Error Global Control Enable. The MEC_A55C0EIRQ_GCTL[n].VALUE bit enables the A55 memory error glob- al control. 0 Disable                               |

## A55 Error Interrupt Request Global Status Register

Figure 47-6: MEC\_A55C0EIRQ\_GSTAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000005_34f22372bc1c0f92bbf23a83eb51c86e30042d32192cef5b2a88afb249e46419.png)

Table 47-16: MEC\_A55C0EIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | A55 Memory Error Global Control Register Lock Write Error. When set (=1), the MEC_A55C0EIRQ_GSTAT[n].LWERR bit indicates that there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 0 (R/W)            | VALUE      | A55 Error Global Status. When set (=1), the MEC_A55C0EIRQ_GSTAT[n].VALUE bit indicates the asser- tion of an interrupt/trigger. When cleared (=0), there is no interrupt or trigger. 0 No Interrupt/Trigger 1 Interrupt/Trigger Asserted                                                                                                           |

## A55 Error Control Register

Figure 47-7: MEC\_A55C0ERR\_CTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000006_f06c0d1d54780c002a9c17011f54c932687073734847398a3940ca653b9f95f0.png)

Table 47-17: MEC\_A55C0ERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | A55 Memory Error Control Register Lock Bit. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55C0ERR_CTL[n].LOCK bit is enabled, the MEC_A55C0ERR_CTL[n] register is read only.              |
| 1:0 (R/W)          | VALUE      | A55 Memory Error Control. The MEC_A55C0ERR_CTL[n].VALUE field enables A55 memory error control. When set (=1), control is enabled. When cleared (=0), control is disabled. 1 A55 SCU Err 2 A55 L1,L2 Err |

## A55 Error Interrupt Mask Register

Figure 47-8: MEC\_A55C0ERR\_IMASK[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000007_7f172ca3817efe1fcb72e0bda88aa7c963e8ff2c972d638b835cf477eaa96402.png)

Table 47-18: MEC\_A55C0ERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | A55 Memory Error Control Register Lock Bit. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55C0ERR_IMASK[n].LOCK bit is enabled, the MEC_A55C0ERR_IMASK[n] register is read only. |
| 1:0 (R/W)          | VALUE      | A55 Memory Error Interrupt Mask. The MEC_A55C0ERR_IMASK[n].VALUE field indicates whether an interrupt is masked (when set (=1)) or unmasked (=0). 1 A55 SCU Err 2 A55 L1,L2 Err                 |

## A55 Error Status Register

Figure 47-9: MEC\_A55C0ERR\_STAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000008_d61b798daa69db89a70edf1364dd5adeccb693e583d48098602396741ccf81b7.png)

Table 47-19: MEC\_A55C0ERR\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | A55 Memory Error Global Control Register Lock Write Error. The MEC_A55C0ERR_STAT[n].LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | A55 Memory Error Global Control Register Lock Write Error. The MEC_A55C0ERR_STAT[n].LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 1:0 (R/W)          | VALUE      | Error Status. The MEC_A55C0ERR_STAT[n].VALUE field indicates the status of an inter- rupt/trigger. When set (=1), an interrupt or trigger is asserted. When cleared (=0), there is no interrupt or trigger.                                                                                                                  | Error Status. The MEC_A55C0ERR_STAT[n].VALUE field indicates the status of an inter- rupt/trigger. When set (=1), an interrupt or trigger is asserted. When cleared (=0), there is no interrupt or trigger.                                                                                                                  |
| 1:0 (R/W)          | VALUE      | 1                                                                                                                                                                                                                                                                                                                            | A55 SCU Error                                                                                                                                                                                                                                                                                                                |
| 1:0 (R/W)          | VALUE      | 2                                                                                                                                                                                                                                                                                                                            | A55 L1,L2 Error                                                                                                                                                                                                                                                                                                              |

## A55 Core Memory Errors Interrupt Request Global Control Register

Figure 47-10: MEC\_A55C2EIRQ\_GCTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000009_281608dcb0a9f4a0c1a7db80b1d7d80439484c59cd09abbc3a6007d3e125bfe8.png)

Table 47-20: MEC\_A55C2EIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55C2EIRQ_GCTL[n].LOCK bit is enabled, the MEC_A55C2EIRQ_GCTL[n] register is read only. 0 Unlock |
| 0 (R/W)            | VALUE      | A55 Memory Error Global Control Enable. The MEC_A55C2EIRQ_GCTL[n].VALUE bit enables the A55 memory error glob- al control. 0 Disable                               |

## A55 Error Interrupt Request Global Status Register

Figure 47-11: MEC\_A55C2EIRQ\_GSTAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000010_34f22372bc1c0f92bbf23a83eb51c86e30042d32192cef5b2a88afb249e46419.png)

Table 47-21: MEC\_A55C2EIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | A55 Memory Error Global Control Register Lock Write Error. When set (=1), the MEC_A55C2EIRQ_GSTAT[n].LWERR bit indicates that there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 0 (R/W)            | VALUE      | A55 Error Global Status. When set (=1), the MEC_A55C2EIRQ_GSTAT[n].VALUE bit indicates the asser- tion of an interrupt/trigger. When cleared (=0), there is no interrupt or trigger. 0 No Interrupt/Trigger 1 Interrupt/Trigger Asserted                                                                                                           |

## A55 Error Control Register

Figure 47-12: MEC\_A55C2ERR\_CTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000011_f06c0d1d54780c002a9c17011f54c932687073734847398a3940ca653b9f95f0.png)

Table 47-22: MEC\_A55C2ERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | A55 Memory Error Control Register Lock Bit. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55C2ERR_CTL[n].LOCK bit is enabled, the MEC_A55C2ERR_CTL[n] register is read only.              |
| 1:0 (R/W)          | VALUE      | A55 Memory Error Control. The MEC_A55C2ERR_CTL[n].VALUE field enables A55 memory error control. When set (=1), control is enabled. When cleared (=0), control is disabled. 1 A55 SCU Err 2 A55 L1,L2 Err |

## A55 Error Interrupt Mask Register

Figure 47-13: MEC\_A55C2ERR\_IMASK[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000012_7e66581d5f7a0816dd617dd73e4ed4c4817f535263e6b462809f231e5fcbc83d.png)

Table 47-23: MEC\_A55C2ERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | A55 Memory Error Control Register Lock Bit. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_A55C2ERR_IMASK[n].LOCK bit is enabled, the MEC_A55C2ERR_IMASK[n] register is read only. |
| 1:0 (R/W)          | VALUE      | A55 Memory Error Interrupt Mask. The MEC_A55C2ERR_IMASK[n].VALUE field indicates whether an interrupt is masked (when set (=1)) or unmasked (=0). 1 A55 SCU Err 2 A55 L1,L2 Err                 |

## A55 Error Status Register

Figure 47-14: MEC\_A55C2ERR\_STAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000013_d61b798daa69db89a70edf1364dd5adeccb693e583d48098602396741ccf81b7.png)

Table 47-24: MEC\_A55C2ERR\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | A55 Memory Error Global Control Register Lock Write Error. The MEC_A55C2ERR_STAT[n].LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | A55 Memory Error Global Control Register Lock Write Error. The MEC_A55C2ERR_STAT[n].LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 1:0 (R/W)          | VALUE      | Error Status. The MEC_A55C2ERR_STAT[n].VALUE field indicates the status of an inter- rupt/trigger. When set (=1), an interrupt or trigger is asserted. When cleared (=0), there is no interrupt or trigger.                                                                                                                  | Error Status. The MEC_A55C2ERR_STAT[n].VALUE field indicates the status of an inter- rupt/trigger. When set (=1), an interrupt or trigger is asserted. When cleared (=0), there is no interrupt or trigger.                                                                                                                  |
| 1:0 (R/W)          | VALUE      | 1                                                                                                                                                                                                                                                                                                                            | A55 SCU Error                                                                                                                                                                                                                                                                                                                |
| 1:0 (R/W)          | VALUE      | 2                                                                                                                                                                                                                                                                                                                            | A55 L1,L2 Error                                                                                                                                                                                                                                                                                                              |

## Component ID0 Register

Figure 47-15: MEC\_CID0 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000014_f9d45f5056356384dd1abe0c5d23097205a63b41258f808c3e428a309551749f.png)

Table 47-25: MEC\_CID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID0.PREAMBLE field indicates the component ID preamble. |

## Component ID1 Register

Figure 47-16: MEC\_CID1 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000015_69b43fd3084eba57188cbce31680256ba625bd70052a5c7f7e5ca75ed0452484.png)

Table 47-26: MEC\_CID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | COMPCLASS  | Component Class. The MEC_CID1.COMPCLASS field indicates the component class. Dedicated debug blocks (core debug access port, Program Trace, etc.) should identify as CoreSight (i.e. 0x9) and implement the full compliment of CoreSight registers including DEVTYPE. All other ADI components should identify as System (i.e. 0xF) components. See CoreSight Architecture Specification for details. 9 CoreSight |
| 3:0 (R/NW)         | PREAMBLE   | Component ID Preamble. The MEC_CID1.PREAMBLE field indicates the component ID preamble.                                                                                                                                                                                                                                                                                                                           |

## Component ID2 Register

Figure 47-17: MEC\_CID2 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000016_535a5860afb6bf0faaf4ac31ce3ffc73110029e329c426d4394b54cf9c4d0356.png)

Table 47-27: MEC\_CID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID2.PREAMBLE field indicates the component ID preamble. |

## Component ID3 Register

Figure 47-18: MEC\_CID3 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000017_5f9db30a16f5f434c2e75aa3da47353200405a9f61a64e0260c72f066e22d9ef.png)

Table 47-28: MEC\_CID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                          |
|--------------------|------------|------------------------------------------------------------------|
| 7:0                | PREAMBLE   | Component ID Preamble.                                           |
| (R/NW)             |            | The MEC_CID3.PREAMBLE field indicates the component ID preamble. |

## Clear Register

Figure 47-19: MEC\_CLR Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000018_fc26f1607eaa9ae1319dea6075caf6ea9de5828c11f3ba29faadc2026ca947f4.png)

Table 47-29: MEC\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 0                  | CLRSTAT    | Clear Status.                                                            |
| (R0/W)             |            | Writing 1 to the MEC_CLR.CLRSTAT bit clears all status registers of MEC. |

## Core Mem ECC Error Interrupt Request Global Control Register

Figure 47-20: MEC\_CMEIRQ\_GCTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000019_ba063ec14b22ec17c192a966c12ddf8872e727bc6e782f53b2d2c99f6e75e9db.png)

Table 47-30: MEC\_CMEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 31                 | LOCK       | Lock.                                 |
| (R/W)              | VALUE      | 0 Unlock 1 Lock Global Control Value. |
| 2:0 (R/W)          |            |                                       |

## Core Memory ECC Error Interrupt Request Global Status Register

Figure 47-21: MEC\_CMEIRQ\_GSTAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000020_9cad4ecf8894deb6b4dca9bb39dd571626130f482d3c64506702488f625e578b.png)

Table 47-31: MEC\_CMEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Lock Write Error. The MEC_CMEIRQ_GSTAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK = 1). This status bit is sticky; write-1-to-clear it. 0 No Error 1 Error Occurred |
| 2:0 (R/W)          | VALUE      | Global Control. '1' indicates enable and '0' indicates disable.                                                                                                                                                                                                                                                   |

## Core Memory ECC Error Control Register

Figure 47-22: MEC\_CMERR\_CTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000021_c7f28664f48ba4f6d9a123e61de05acc67cba3276401ccda0b1f77de270c91ca.png)

Table 47-32: MEC\_CMERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. The bit indicates the core memory error control register lock status. 0 Unlock |
| 7:0 (R/W)          | VALUE      | Error. '1' indicates enable and '0' indicates disable                                |
|                    |            | 1 iPref RAM ECC error clean-uncorrectable                                            |
|                    |            | 2 dPref RAM ECC error dirty-uncorrectable                                            |
|                    |            | 4 Reserved                                                                           |
|                    |            | 8 iCache ECC error uncorrectable                                                     |
|                    |            | 16 dCache ECC error dirty-uncorrectable                                              |
|                    |            | 32 dCache ECC error clean-uncorrectable                                              |
|                    |            | 64 iRAM ECC error uncorrectable                                                      |
|                    |            | 128 dRAM ECC error uncorrectable                                                     |

## Core Memory ECC Error Interrupt Mask Register

Figure 47-23: MEC\_CMERR\_IMASK[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000022_7a4003c9a8c5e15b946a071f60e19688fb9da9deb73915b28fc43b4cbea9fa93.png)

Table 47-33: MEC\_CMERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31                 | LOCK       | Core Memory Error Interrupt Mask Register Lock.                                                                                                                                                       |
| 7:0 (R/W)          | VALUE      | Core Memory Error. '1' indicates enable and '0' indicates disable. 1 iPref RAM ECC error clean-uncorrectable                                                                                          |
| 7:0 (R/W)          |            | 2 dPref RAM ECC error dirty-uncorrectable 4 Reserved 8 iCache ECC error uncorrectable 16 dCache ECC error dirty-uncorrectable 32 dCache ECC error clean-uncorrectable 64 iRAM ECC error uncorrectable |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |
| 7:0 (R/W)          |            |                                                                                                                                                                                                       |

## Core Memory ECC Error Status Register

Figure 47-24: MEC\_CMERR\_STAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000023_726883bf6d6aa75bb74845de16ddf754d169a52e8c5ec844fdba6df756491076.png)

Table 47-34: MEC\_CMERR\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Core Memory Error Control or Interrupt Mask Register Lock Write Error. The MEC_CMERR_STAT[n].LWERR bit indicates (when set) there was an attempt- ed write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Core Memory Error Control or Interrupt Mask Register Lock Write Error. The MEC_CMERR_STAT[n].LWERR bit indicates (when set) there was an attempt- ed write to an MEC register while the MEC_EEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                                       | No Error                                                                                                                                                                                                                                                                                                                                |
| 31 (R/W1C)         | LWERR      | 1                                                                                                                                                                                                                                                                                                                                       | Error Occurred                                                                                                                                                                                                                                                                                                                          |
| 7:0 (R/W1C)        | VALUE      | Core Memory Error Status.                                                                                                                                                                                                                                                                                                               | Core Memory Error Status.                                                                                                                                                                                                                                                                                                               |
| 7:0 (R/W1C)        | VALUE      | 1                                                                                                                                                                                                                                                                                                                                       | iPref RAM ECC error clean-uncorrectable                                                                                                                                                                                                                                                                                                 |
| 7:0 (R/W1C)        | VALUE      | 2                                                                                                                                                                                                                                                                                                                                       | dPref RAM ECC error dirty-uncorrectable                                                                                                                                                                                                                                                                                                 |
| 7:0 (R/W1C)        | VALUE      | 4                                                                                                                                                                                                                                                                                                                                       | Reserved                                                                                                                                                                                                                                                                                                                                |
| 7:0 (R/W1C)        | VALUE      | 8                                                                                                                                                                                                                                                                                                                                       | iCache ECC error uncorrectable                                                                                                                                                                                                                                                                                                          |
| 7:0 (R/W1C)        | VALUE      | 16                                                                                                                                                                                                                                                                                                                                      | dCache ECC error dirty-uncorrectable                                                                                                                                                                                                                                                                                                    |
| 7:0 (R/W1C)        | VALUE      | 32                                                                                                                                                                                                                                                                                                                                      | dCache ECC error clean-uncorrectable                                                                                                                                                                                                                                                                                                    |
| 7:0 (R/W1C)        | VALUE      | 64                                                                                                                                                                                                                                                                                                                                      | iRAM ECC error uncorrectable                                                                                                                                                                                                                                                                                                            |
| 7:0 (R/W1C)        | VALUE      | 128                                                                                                                                                                                                                                                                                                                                     | dRAM ECC error uncorrectable                                                                                                                                                                                                                                                                                                            |

## ECC Error Control Register

The MEC\_ECCERR\_CTL[n] register bits control enable/disable for ECC error inputs from various cores/peripherals and decide whether their status will be reflected in ECC error status register bits.

Figure 47-25: MEC\_ECCERR\_CTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000024_c1274d1a956dfb6951f480239ce7181bcda0b3870ddcf474a6eea1a3405112dc.png)

Table 47-35: MEC\_ECCERR\_CTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_CTL[n].LOCK bit is enabled, the MEC_ECCERR_CTL[n] regis- ter is read only. 0 Unlock |
| 5:0 (R/W)          | VALUE      | ECC Error Control. The MEC_ECCERR_CTL[n].VALUE bit indicates whether ECC control is enabled (=1) or disabled (=0).                                                                      |

## ECC Error Interrupt Mask Register

The MEC\_ECCERR\_IMASK[n] register bits control interrupt masks for ECC error inputs from various cores/ peripherals.

Figure 47-26: MEC\_ECCERR\_IMASK[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000025_f5a7c9dedd35f4919a3bb0fc00354570ceeaa5802f4a0c83a4ec2beb1b5640c4.png)

Table 47-36: MEC\_ECCERR\_IMASK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Interrupt Mask Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_ECCERR_IMASK[n].LOCK bit is enabled, the MEC_ECCERR_IMASK[n] register is read only. 0 Unlock |
| 5:0 (R/W)          | VALUE      | ECC Error Interrupt Mask. The MEC_ECCERR_IMASK[n].VALUE bit indicates when the ECC error inter- rupt is masked (=1) or unmasked (=0).                                                            |

## ECC Error Status Register

The MEC\_ECCERR\_STAT[n] register bits reflect status for ECC error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding ECC error status.

Figure 47-27: MEC\_ECCERR\_STAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000026_5ce3a966489678689179e213039f6eb023f068364108186e38d54b2744fedcd9.png)

Table 47-37: MEC\_ECCERR\_STAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | ECC Error Control or Interrupt Mask Register Lock Write Error. The MEC_ECCERR_STAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write 1 to clear it. |
| 5:0 (R/W1C)        | VALUE      | ECC Error Status. '1' indicates error and '0' indicates no error. Sticky bit, write '1' to clear. 1 L2 CTL ECC Error                                                                                                                                                                                      |

## ECC Error Interrupt Request Global Control Register

The MEC\_EEIRQ\_GCTL[n] register bits control enable/disable of ECC error interrupt/trigger outputs.

Figure 47-28: MEC\_EEIRQ\_GCTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000027_6cb9a9669725b97ef3a928b0b3bcdd530f143b2224d9ec6947cfb9f877d9c52c.png)

Table 47-38: MEC\_EEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | ECC Error Global Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_EEIRQ_GCTL[n].LOCK bit is enabled, the MEC_EEIRQ_GCTL[n] regis- ter is read only. |
| 0 (R/W)            | VALUE      | ECC Error Global Control. '1' indicates enable and '0' indicates disable.                                                                                                             |

## ECC Error Interrupt Request Global Status Register

The MEC\_EEIRQ\_GSTAT[n] register bits reflect the status of the ECC error interrupt/trigger outputs.

Figure 47-29: MEC\_EEIRQ\_GSTAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000028_d26f0e28e02f9be89d0a3e2e85d5de1dabd4521183a7630f7851bcf8a303d766.png)

Table 47-39: MEC\_EEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | ECC Error Global Control Register Lock Write Error. The MEC_EEIRQ_GSTAT[n].LWERR bit indicates (when set) there was an at- tempted write to an MEC register while the bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write 1 to clear it. |
| 0 (R/NW)           | VALUE      | ECC Error Global Status. '1' indicates assertion of interrupt/trigger and '0' indicates no interrupt/trigger.                                                                                                                                                                                  |

## Parity Error Interrupt Request Global Control Register

The MEC\_PEIRQ\_GCTL[n] register bits control enable/disable of parity error interrupt/trigger outputs.

Figure 47-30: MEC\_PEIRQ\_GCTL[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000029_193546763aa10b374cc8fcc5ef5290db4edd51b073da077a4a93726fe9d47924.png)

Table 47-40: MEC\_PEIRQ\_GCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Parity Error Global Control Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PEIRQ_GCTL[n].LOCK bit is enabled, the MEC_PEIRQ_GCTL[n] regis- ter is read only. 0 Unlock |
| 0 (R/W)            | VALUE      | Parity Error Global Control. The MEC_PEIRQ_GCTL[n].VALUE bit field indicates whether parity error control is enabled (=1) or disabled (=0).                                                       |

## Parity Error Interrupt Request Global Status Register

The MEC\_PEIRQ\_GSTAT[n] register bits reflect status of parity error interrupt/trigger outputs.

Figure 47-31: MEC\_PEIRQ\_GSTAT[n] Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000030_f7b33b1c3ec9a98845684a2f4b7d2706030feaf3a3de06f759f4c8bd9a269daf.png)

Table 47-41: MEC\_PEIRQ\_GSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Global Control Register Lock Write Error. When set (=1), the MEC_PEIRQ_GSTAT[n].LWERR bit indicates an attempted write to an MEC register while the MEC_PEIRQ_GCTL[n].LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write 1 to clear it. |
| 0 (R/NW)           | VALUE      | Parity Error Global Status. The MEC_PEIRQ_GSTAT[n].VALUE field indicates the parity error global status. 1 indicates the assertion of the interrupt/trigger; 0 indicates no interrupt/trigger.                                                                                                                        |

## Parity Error Control Register

The MEC\_PERR\_CTL0 register bits control enable/disable for parity error inputs from various cores/peripherals and decide whether their status will be reflected in parity error status register bits.

Figure 47-32: MEC\_PERR\_CTL0 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000031_edb735339fb2daf0ff5bc3844963ce693b456cfc9467b21cf27ca4c11f62c63e.png)

Table 47-42: MEC\_PERR\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL0.LOCK bit is enabled, the MEC_PERR_CTL0 register is read only. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL0.LOCK bit is enabled, the MEC_PERR_CTL0 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                         | Unlock                                                                                                                                    |
| 31 (R/W)           | LOCK       | 1                                                                                                                                         | Lock                                                                                                                                      |
| 30:0 (R/W)         | VALUE      | Parity Error Control. The MEC_PERR_CTL0.VALUE bit indicates whether parity error control is enabled (=1) or disabled (=0).                | Parity Error Control. The MEC_PERR_CTL0.VALUE bit indicates whether parity error control is enabled (=1) or disabled (=0).                |
| 30:0 (R/W)         | VALUE      | 4                                                                                                                                         | Core 1 L1 RAM Parity Error                                                                                                                |
| 30:0 (R/W)         | VALUE      | 8                                                                                                                                         | Core 1 L1 Cache Parity Error                                                                                                              |
| 30:0 (R/W)         | VALUE      | 16                                                                                                                                        | Core 1 Branch Predictor Parity Error                                                                                                      |
| 30:0 (R/W)         | VALUE      | 256                                                                                                                                       | ASRC0 FIFO Parity Error                                                                                                                   |
| 30:0 (R/W)         | VALUE      | 512                                                                                                                                       | ASRC1 FIFO Parity Error                                                                                                                   |
| 30:0 (R/W)         | VALUE      | 1024                                                                                                                                      | ASRC2 FIFO Parity Error                                                                                                                   |
| 30:0 (R/W)         | VALUE      | 2048                                                                                                                                      | ASRC3 FIFO Parity Error                                                                                                                   |
| 30:0 (R/W)         | VALUE      | 4096                                                                                                                                      | ASRC4 FIFO Parity Error                                                                                                                   |
| 30:0 (R/W)         | VALUE      | 8192                                                                                                                                      | ASRC5 FIFO Parity Error                                                                                                                   |
| 30:0 (R/W)         | VALUE      | 16384                                                                                                                                     | ASRC6 FIFO Parity Error                                                                                                                   |

Table 47-42: MEC\_PERR\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
|                    |            |                     32768 | ASRC7 FIFO Parity Error        |
|                    |            |                     65536 | SHARC 0 IIR0 RAM Parity Error  |
|                    |            |                    131072 | SHARC 0 IIR1 RAM Parity Error  |
|                    |            |                    262144 | SHARC 0 IIR2 RAM Parity Error  |
|                    |            |                    524288 | SHARC 0 IIR3 RAM Parity Error  |
|                    |            |                   1048576 | SHARC 0 FIR0 RAM Parity Error  |
|                    |            |                  33554432 | SHARC0 FIR1 RAM Parity Error   |
|                    |            |                 134217728 | TRNG0 Data Buffer Parity Error |
|                    |            |                 268435456 | PKA0 Data RAM Parity Error     |
|                    |            |                 536870912 | SPE0 Buffer RAM Parity Error   |
|                    |            |                1073741824 | SPE0 ARC4 RAM Parity Error     |

## Parity Error Control Register

MEC\_PERR\_CTL1 register bits control enable/disable for parity error inputs from various cores/peripherals and decide whether their status will be reflected in parity error status register bits.

Figure 47-33: MEC\_PERR\_CTL1 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000032_1734e942bb777f5cb20b815bbaeabfbe079687953f355c846fbc8c475e0ab7a2.png)

Table 47-43: MEC\_PERR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL1.LOCK bit is enabled, the MEC_PERR_CTL1 register is read only. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_CTL1.LOCK bit is enabled, the MEC_PERR_CTL1 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                         | Unlock                                                                                                                                    |
| 31 (R/W)           | LOCK       | 1                                                                                                                                         | Lock                                                                                                                                      |
| 25:0 (R/W)         | VALUE1     | Parity Error Control. The MEC_PERR_CTL1.VALUE1 field indicates the parity error inputs. 1 indicates enable and 0 indicates disable.       | Parity Error Control. The MEC_PERR_CTL1.VALUE1 field indicates the parity error inputs. 1 indicates enable and 0 indicates disable.       |
| 25:0 (R/W)         | VALUE1     | 1                                                                                                                                         | EMAC0 Transmit FIFO RAM Parity Error                                                                                                      |
| 25:0 (R/W)         | VALUE1     | 2                                                                                                                                         | EMAC0 Receive FIFO RAM Parity Error                                                                                                       |
| 25:0 (R/W)         | VALUE1     | 4                                                                                                                                         | MLB0 Data Buffer RAM Parity Error                                                                                                         |
| 25:0 (R/W)         | VALUE1     | 8                                                                                                                                         | MLB0 Channel Table RAM Parity Error                                                                                                       |
| 25:0 (R/W)         | VALUE1     | 16                                                                                                                                        | TMC0 Trace Data RAM Parity Error                                                                                                          |
| 25:0 (R/W)         | VALUE1     | 128                                                                                                                                       | EMAC0 TMI FIFO RAM Parity Error                                                                                                           |
| 25:0 (R/W)         | VALUE1     | 256                                                                                                                                       | EMAC0 EST FIFO RAM Parity Error                                                                                                           |
| 25:0 (R/W)         | VALUE1     | 512                                                                                                                                       | EMSI0 FIFO RAM Parity Error                                                                                                               |
| 25:0 (R/W)         | VALUE1     | 1024                                                                                                                                      | ASRC8 FIFO RAM Parity Error                                                                                                               |
| 25:0 (R/W)         | VALUE1     | 2048                                                                                                                                      | ASRC9 FIFO RAM Parity Error                                                                                                               |

Table 47-43: MEC\_PERR\_CTL1 Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration       |
|-----------|------------|---------------------------|-------------------------------|
| (Access)  |            |                           |                               |
|           |            |                      4096 | ASRC10 FIFO RAM Parity Error  |
|           |            |                      8192 | ASRC11 FIFO RAM Parity Error  |
|           |            |                     16384 | ASRC12 FIFO RAM Parity Error  |
|           |            |                     32768 | ASRC13 FIFO RAM Parity Error  |
|           |            |                     65536 | ASRC14 FIFO RAM Parity Error  |
|           |            |                    131072 | ASRC15 FIFO RAM Parity Error  |
|           |            |                    262144 | XSPI Flash RAM Parity Error   |
|           |            |                    524288 | XSPI CTRL RAM Parity Error    |
|           |            |                   1048576 | XSPI 1 Flash RAM Parity Error |
|           |            |                   2097152 | XSPI 1 CTRL RAM Parity Error  |
|           |            |                  16777216 | LPDDR ICCM Parity Error       |
|           |            |                  33554432 | LPDDR DCCMParity Error        |

## Parity Error Interrupt Mask Register

The MEC\_PERR\_IMASK0 register bits control interrupt masks for parity error inputs from various cores/peripherals.

Figure 47-34: MEC\_PERR\_IMASK0 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000033_89a6102ace181ec8295f8e36cea8c18dc63110698b5b904cce93bfa996e6bbaf.png)

Table 47-44: MEC\_PERR\_IMASK0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Parity Error Interrupt Mask0 Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK0.LOCK bit is enabled, the MEC_PERR_IMASK0 register is read only. | Parity Error Interrupt Mask0 Register Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK0.LOCK bit is enabled, the MEC_PERR_IMASK0 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                   | Unlock                                                                                                                                                                              |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                   | Lock                                                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | Parity Error Interrupt Mask. The MEC_PERR_IMASK0.VALUE bit field, when set (=1), indicates an interrupt masked and, when cleared (=0), indicates an interrupt unmasked.             | Parity Error Interrupt Mask. The MEC_PERR_IMASK0.VALUE bit field, when set (=1), indicates an interrupt masked and, when cleared (=0), indicates an interrupt unmasked.             |
| 30:0 (R/W)         | VALUE      | 4                                                                                                                                                                                   | Core 1 L1 RAM Parity Error                                                                                                                                                          |
| 30:0 (R/W)         | VALUE      | 8                                                                                                                                                                                   | Core 1 L1 Cache Parity Error                                                                                                                                                        |
| 30:0 (R/W)         | VALUE      | 16                                                                                                                                                                                  | Core 1 Branch Predictor Parity Error                                                                                                                                                |
| 30:0 (R/W)         | VALUE      | 256                                                                                                                                                                                 | ASRC0 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 512                                                                                                                                                                                 | ASRC1 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 1024                                                                                                                                                                                | ASRC2 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 2048                                                                                                                                                                                | ASRC3 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 4096                                                                                                                                                                                | ASRC4 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 8192                                                                                                                                                                                | ASRC5 FIFO Parity Error                                                                                                                                                             |
| 30:0 (R/W)         | VALUE      | 16384                                                                                                                                                                               | ASRC6 FIFO Parity Error                                                                                                                                                             |

Table 47-44: MEC\_PERR\_IMASK0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|--------------------|------------|---------------------------|--------------------------------|
|                    |            |                     32768 | ASRC7 FIFO Parity Error        |
|                    |            |                     65536 | SHARC 0 IIR0 RAM Parity Error  |
|                    |            |                    131072 | SHARC 0 IIR1 RAM Parity Error  |
|                    |            |                    262144 | SHARC 0 IIR2 RAM Parity Error  |
|                    |            |                    524288 | SHARC 0 IIR3 RAM Parity Error  |
|                    |            |                   1048576 | SHARC 0 FIR0 RAM Parity Error  |
|                    |            |                  33554432 | SHARC0 FIR1 RAM Parity Error   |
|                    |            |                 134217728 | TRNG0 Data Buffer Parity Error |
|                    |            |                 268435456 | PKA0 Data RAM Parity Error     |
|                    |            |                 536870912 | SPE0 Buffer RAM Parity Error   |
|                    |            |                1073741824 | SPE0 ARC4 RAM Parity Error     |

## Parity Error Interrupt Mask Register

MEC\_PERR\_IMASK1 register bits control interrupt masks for parity error inputs from various cores/peripherals.

Figure 47-35: MEC\_PERR\_IMASK1 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000034_0099d567b20de39789f3fc6c619a03f5b140c5389b64818f97fc618bda92d562.png)

Table 47-45: MEC\_PERR\_IMASK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK1.LOCK bit is enabled, the MEC_PERR_IMASK1 register is read only. | Lock. If the global lock is enabled (SPU_CTL.GLCK =1) and the MEC_PERR_IMASK1.LOCK bit is enabled, the MEC_PERR_IMASK1 register is read only. |
| 31 (R/W)           | LOCK       | 0                                                                                                                                             | Unlock                                                                                                                                        |
| 31 (R/W)           | LOCK       | 1                                                                                                                                             | Lock                                                                                                                                          |
| 25:0 (R/W)         | VALUE      | Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked.                                             | Parity Error Interrupt Mask. '1' indicates interrupt masked and '0' indicates interrupt unmasked.                                             |
| 25:0 (R/W)         | VALUE      | 1                                                                                                                                             | EMAC0 Transmit FIFO RAM Parity Error                                                                                                          |
| 25:0 (R/W)         | VALUE      | 2                                                                                                                                             | EMAC0 Receive FIFO RAM Parity Error                                                                                                           |
| 25:0 (R/W)         | VALUE      | 4                                                                                                                                             | MLB0 Data Buffer RAM Parity Error                                                                                                             |
| 25:0 (R/W)         | VALUE      | 8                                                                                                                                             | MLB0 Channel Table RAM Parity Error                                                                                                           |
| 25:0 (R/W)         | VALUE      | 16                                                                                                                                            | TMC0 Trace Data RAM Parity Error                                                                                                              |
| 25:0 (R/W)         | VALUE      | 128                                                                                                                                           | EMAC0 TMI FIFO RAM Parity Error                                                                                                               |
| 25:0 (R/W)         | VALUE      | 256                                                                                                                                           | EMAC0 EST FIFO RAM Parity Error                                                                                                               |
| 25:0 (R/W)         | VALUE      | 512                                                                                                                                           | EMSI0 FIFO RAM Parity Error                                                                                                                   |
| 25:0 (R/W)         | VALUE      | 1024                                                                                                                                          | ASRC8 FIFO RAM Parity Error                                                                                                                   |
| 25:0 (R/W)         | VALUE      | 2048                                                                                                                                          | ASRC9 FIFO RAM Parity Error                                                                                                                   |
| 25:0 (R/W)         | VALUE      | 4096                                                                                                                                          | ASRC10 FIFO RAM Parity Error                                                                                                                  |
| 25:0 (R/W)         | VALUE      | 8192                                                                                                                                          | ASRC11 FIFO RAM Parity Error                                                                                                                  |

Table 47-45: MEC\_PERR\_IMASK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration       |
|--------------------|------------|---------------------------|-------------------------------|
|                    |            |                     32768 | ASRC13 FIFO RAM Parity Error  |
|                    |            |                     65536 | ASRC14 FIFO RAM Parity Error  |
|                    |            |                    131072 | ASRC15 FIFO RAM Parity Error  |
|                    |            |                    262144 | XSPI Flash RAM Parity Error   |
|                    |            |                    524288 | XSPI CTRL RAM Parity Error    |
|                    |            |                   1048576 | XSPI 1 Flash RAM Parity Error |
|                    |            |                   2097152 | XSPI 1 CTRL RAM Parity Error  |
|                    |            |                  16777216 | LPDDR ICCM Parity Error       |
|                    |            |                  33554432 | LPDDR DCCMParity Error        |

## Parity Error Status Register

The MEC\_PERR\_STAT0 register bits reflect status for parity error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding parity error status.

Figure 47-36: MEC\_PERR\_STAT0 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000035_08312504a54fa8173e036175adb40be8bded5bd39afb5f437fc14225c21d8362.png)

Table 47-46: MEC\_PERR\_STAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT0.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL0.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT0.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL0.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                         | No Error                                                                                                                                                                                                                                                                                                                  |
| 31 (R/W1C)         | LWERR      | 1                                                                                                                                                                                                                                                                                                                         | Error Occurred                                                                                                                                                                                                                                                                                                            |
| 30:0 (R/W1C)       | VALUE      | Parity Error Status. When set (=1), the MEC_PERR_STAT0.VALUE bit indicates the parity error status. When cleared (=0), the bit indicates no error. This is a sticky bit; write 1 to clear.                                                                                                                                | Parity Error Status. When set (=1), the MEC_PERR_STAT0.VALUE bit indicates the parity error status. When cleared (=0), the bit indicates no error. This is a sticky bit; write 1 to clear.                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 4                                                                                                                                                                                                                                                                                                                         | Core 1 L1 RAM Parity Error                                                                                                                                                                                                                                                                                                |
| 30:0 (R/W1C)       | VALUE      | 8                                                                                                                                                                                                                                                                                                                         | Core 1 L1 Cache Parity Error                                                                                                                                                                                                                                                                                              |
| 30:0 (R/W1C)       | VALUE      | 16                                                                                                                                                                                                                                                                                                                        | Core 1 Branch Predictor Parity Error                                                                                                                                                                                                                                                                                      |
| 30:0 (R/W1C)       | VALUE      | 256                                                                                                                                                                                                                                                                                                                       | ASRC0 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 512                                                                                                                                                                                                                                                                                                                       | ASRC1 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 1024                                                                                                                                                                                                                                                                                                                      | ASRC2 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 2048                                                                                                                                                                                                                                                                                                                      | ASRC3 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 4096                                                                                                                                                                                                                                                                                                                      | ASRC4 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |
| 30:0 (R/W1C)       | VALUE      | 8192                                                                                                                                                                                                                                                                                                                      | ASRC5 FIFO Parity Error                                                                                                                                                                                                                                                                                                   |

Table 47-46: MEC\_PERR\_STAT0 Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration        |
|-----------|------------|---------------------------|--------------------------------|
| (Access)  |            |                           |                                |
|           |            |                     16384 | ASRC6 FIFO Parity Error        |
|           |            |                     32768 | ASRC7 FIFO Parity Error        |
|           |            |                     65536 | SHARC 0 IIR0 RAM Parity Error  |
|           |            |                    131072 | SHARC 0 IIR1 RAM Parity Error  |
|           |            |                    262144 | SHARC 0 IIR2 RAM Parity Error  |
|           |            |                    524288 | SHARC 0 IIR3 RAM Parity Error  |
|           |            |                   1048576 | SHARC 0 FIR0 RAM Parity Error  |
|           |            |                  33554432 | SHARC 0 FIR1 RAM Parity Error  |
|           |            |                 134217728 | TRNG0 Data Buffer Parity Error |
|           |            |                 268435456 | PKA0 Data RAM Parity Error     |
|           |            |                 536870912 | SPE0 Buffer RAM Parity Error   |
|           |            |                1073741824 | SPE0 ARC4 RAM Parity Error     |

## Parity Error Status Register

MEC\_PERR\_STAT1 register bits reflect status for parity error inputs from various cores/peripherals. Writing '1' to these bits clear corresponding parity error status.

Figure 47-37: MEC\_PERR\_STAT1 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000036_957b90e03e018e4d2f831f7f87e098eac8db391a83329a9c98ea845c8a3939b3.png)

Table 47-47: MEC\_PERR\_STAT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT1.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL1.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. | Parity Error Control or Interrupt Mask Register Lock Write Error. The MEC_PERR_STAT1.LWERR bit indicates (when set) there was an attempted write to an MEC register while the MEC_PERR_CTL1.LOCK bit was set and while the global lock bit was enabled (SPU_CTL.GLCK =1). This status bit is sticky; write-1-to-clear it. |
| 31 (R/W1C)         | LWERR      | 0                                                                                                                                                                                                                                                                                                                         | No Error                                                                                                                                                                                                                                                                                                                  |
| 31 (R/W1C)         | LWERR      | 1                                                                                                                                                                                                                                                                                                                         | Error Occurred                                                                                                                                                                                                                                                                                                            |
| 25:0 (R/W1C)       | VALUE      | Parity Error Status bit. 1 indicates error and 0 indicates no error. Sticky bit, write 1 to clear.                                                                                                                                                                                                                        | Parity Error Status bit. 1 indicates error and 0 indicates no error. Sticky bit, write 1 to clear.                                                                                                                                                                                                                        |
| 25:0 (R/W1C)       | VALUE      | 1                                                                                                                                                                                                                                                                                                                         | EMAC0 Transmit FIFO RAM Parity Error                                                                                                                                                                                                                                                                                      |
| 25:0 (R/W1C)       | VALUE      | 2                                                                                                                                                                                                                                                                                                                         | EMAC0 Receive FIFO RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 25:0 (R/W1C)       | VALUE      | 4                                                                                                                                                                                                                                                                                                                         | MLB0 Data Buffer RAM Parity Error                                                                                                                                                                                                                                                                                         |
| 25:0 (R/W1C)       | VALUE      | 8                                                                                                                                                                                                                                                                                                                         | MLB0 Channel Table RAM Parity Error                                                                                                                                                                                                                                                                                       |
| 25:0 (R/W1C)       | VALUE      | 16                                                                                                                                                                                                                                                                                                                        | TMC0 Trace Data RAM Parity Error                                                                                                                                                                                                                                                                                          |
| 25:0 (R/W1C)       | VALUE      | 128                                                                                                                                                                                                                                                                                                                       | EMAC0 TMI FIFO RAM Parity Error                                                                                                                                                                                                                                                                                           |
| 25:0 (R/W1C)       | VALUE      | 256                                                                                                                                                                                                                                                                                                                       | EMAC0 EST FIFO RAM Parity Error                                                                                                                                                                                                                                                                                           |
| 25:0 (R/W1C)       | VALUE      | 512                                                                                                                                                                                                                                                                                                                       | EMSI0 FIFO RAM Parity Error                                                                                                                                                                                                                                                                                               |
| 25:0 (R/W1C)       | VALUE      | 1024                                                                                                                                                                                                                                                                                                                      | ASRC8 FIFO RAM Parity Error                                                                                                                                                                                                                                                                                               |
| 25:0 (R/W1C)       | VALUE      | 2048                                                                                                                                                                                                                                                                                                                      | ASRC9 FIFO RAM Parity Error                                                                                                                                                                                                                                                                                               |

Table 47-47: MEC\_PERR\_STAT1 Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration | Description/Enumeration       |
|-----------|------------|---------------------------|-------------------------------|
| (Access)  |            |                           |                               |
|           |            |                      4096 | ASRC10 FIFO RAM Parity Error  |
|           |            |                      8192 | ASRC11 FIFO RAM Parity Error  |
|           |            |                     16384 | ASRC12 FIFO RAM Parity Error  |
|           |            |                     32768 | ASRC13 FIFO RAM Parity Error  |
|           |            |                     65536 | ASRC14 FIFO RAM Parity Error  |
|           |            |                    131072 | ASRC15 FIFO RAM Parity Error  |
|           |            |                    262144 | XSPI Flash RAM Parity Error   |
|           |            |                    524288 | XSPI CTRL RAM Parity Error    |
|           |            |                   1048576 | XSPI 1 Flash RAM Parity Error |
|           |            |                   2097152 | XSPI 1 CTRL RAM Parity Error  |
|           |            |                  16777216 | LPDDR ICCM Parity Error       |
|           |            |                  33554432 | LPDDR DCCMParity Error        |

## Peripheral ID0 Register

Figure 47-38: MEC\_PID0 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000037_e1cc9efb4b131ad4458cbd05b6e42a1c3f90455eddcdd5fb3b4fa183640f58bc.png)

Table 47-48: MEC\_PID0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 7:0                | PARTNUM    | Part Number.                                                                        |
| (R/NW)             |            | The MEC_PID0.PARTNUM field indicates the part number for component identifi- cation |

## Peripheral ID1 Register

Figure 47-39: MEC\_PID1 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000038_9fb6e520a2d2161c0c6c1e792018e6175239043fb2539e33606f9d8121c6b01a.png)

Table 47-49: MEC\_PID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | JEP106IC   | JEDEC JEP106 Identity. The MEC_PID1.JEP106IC field indicates JEDEC JEP106 identity (manufacturer ID) code. |
| 3:0 (R/NW)         | PARTNUM    | Part Number. The MEC_PID1.PARTNUM field indicates the part number for component identifi- cation           |

## Peripheral ID2 Register

Figure 47-40: MEC\_PID2 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000039_0334b319f73f26f828f2d2d6828665770065f17334d7bc5bded60a265d4ff7c6.png)

Table 47-50: MEC\_PID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | REV        | Peripheral Revision.                                                                                       |
| 3 (R/NW)           | JEDECASGN  | JEDEC Assigned Value.                                                                                      |
| 2:0 (R/NW)         | JEP106IC   | JEDEC JEP106 Identity. The MEC_PID2.JEP106IC field indicates JEDEC JEP106 identity (manufacturer ID) code. |

## Peripheral ID3 Register

Figure 47-41: MEC\_PID3 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000040_6bc7c26a344c2396519141e88be7fe5d1f4ae5ac065d79be4b3f85a37469c77c.png)

Table 47-51: MEC\_PID3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4 (R/NW)         | REVAND     | Metal Fix Revision.       |
| 3:0 (R/NW)         | CUSTMOD    | Customer Modified.        |

## Peripheral ID4 Register

Figure 47-42: MEC\_PID4 Register Diagram

![Image](50_Memory_Error_Protection_Unit_(MEPU)_artifacts/image_000041_d9d540b448798b0e5d6e04d83dd5af25c1430b31b82e3582366dd940f1c7e1d8.png)

Table 47-52: MEC\_PID4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/NW)         | SIZE       | Number of 4k Blocks. The MEC_PID4.SIZE field indicates the size of component 4k chunks minus 1 (for example, 0=4k).          |
| 3:0 (R/NW)         | JEP106CC   | JEDEC Continuation Code. The MEC_PID4.JEP106CC field indicates the JEDEC JEP106 continuation code (number of leading 0x7Fs). |