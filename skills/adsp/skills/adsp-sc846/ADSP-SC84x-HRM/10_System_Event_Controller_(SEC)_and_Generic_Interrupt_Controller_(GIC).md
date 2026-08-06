## 7   System Event Controller (SEC) and Generic Interrupt Controller (GIC)

System event management is the responsibility of the system event controller (SEC). The SEC manages the configuration of all system event sources. The SEC also manages the propagation of system events to all connected cores and the system fault interface.

All the peripheral interrupts are routed using a single SEC interrupt to the desired core. The SEC allows programmability of the peripheral interrupt's priority, supporting up to 256 priority levels that are arbitrated within the SEC itself. The SEC also allows these interrupts to be grouped and masked by priority level and provides the flexibility to choose which core(s) the interrupt is routed to.

The SEC also supports self-nesting of interrupts, which is required when sharing a single interrupt request to an individual core, as this allows for a higher-priority peripheral interrupt to be passed to the core while it is currently servicing a lower-priority peripheral interrupt. For more information, refer to 'Self-Nesting Mode for System Event Controller Interrupt (SECI)' in the SHARC+ Core Programming Reference .

For more information about the Arm GIC, visit the Arm Information Center.

## SEC Features

The following list describes the system event controller features.

- Comprehensive system event source management including interrupt enable, fault enable, priority, core mapping, and source grouping.
- Fault management including fault action configuration, timeout, external indication, and system reset.
- Determinism where all system events have the same propagation delay and provide unique identification of a specific system event source.
- Distributed programming model where each system event source control and all status fields are independent of all others.
- Completer control port which provides access to all SEC registers for configuration, status, and interrupt or fault service model.

- Global locking supports a register level protection model to prevent writes to 'locked' registers.

## SEC Functional Description

The following sections provide a functional description of the SEC.

Figure 7-1: SEC Interrupt Signal Flow

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000000_0afe33faaf061aff1accc644a0e9ceed664e7d985f68efcbb07b4fb0f8e85452.png)

## ADSP-2184x SEC Register List

The System Event Controller (SEC) manages the system fault sources, including control features such as enable/disable and active/pending source status. For more information on SEC functionality, see the SEC register descriptions.

Table 7-1: ADSP-2184x SEC Register List

| Name          | Description                       |
|---------------|-----------------------------------|
| SEC_CACT[n]   | SCI Active Register n             |
| SEC_CCTL[n]   | SCI Control Register n            |
| SEC_CGMSK[n]  | SCI Group Mask Register n         |
| SEC_CPLVL[n]  | SCI Priority Level Register n     |
| SEC_CPMSK[n]  | SCI Priority Mask Register n      |
| SEC_CPND[n]   | Core Pending Register n           |
| SEC_CSID[n]   | SCI Source ID Register n          |
| SEC_CSTAT[n]  | SCI Status Register n             |
| SEC_END       | Global End Register               |
| SEC_FCOPP     | Fault COP Period Register         |
| SEC_FCOPP_CUR | Fault COP Period Current Register |

Table 7-1: ADSP-2184x SEC Register List (Continued)

| Name           | Description                               |
|----------------|-------------------------------------------|
| SEC_FCTL       | Fault Control Register                    |
| SEC_FDLY       | Fault Delay Register                      |
| SEC_FDLY_CUR   | Fault Delay Current Register              |
| SEC_FEND       | Fault End Register                        |
| SEC_FSID       | Fault Source ID Register                  |
| SEC_FSRDLY     | Fault System Reset Delay Register         |
| SEC_FSRDLY_CUR | Fault System Reset Delay Current Register |
| SEC_FSTAT      | Fault Status Register                     |
| SEC_GCTL       | Global Control Register                   |
| SEC_GSTAT      | Global Status Register                    |
| SEC_RAISE      | Global Raise Register                     |
| SEC_SCTL[n]    | Source Control Register n                 |
| SEC_SSTAT[n]   | Source Status Register n                  |

## ADSP-2184x SEC Interrupt List

Table 7-2: ADSP-2184x SEC Interrupt List

|   Interrupt ID | Name     | Description   | Sensitivity   | DMA Channel   |
|----------------|----------|---------------|---------------|---------------|
|              0 | SEC0_ERR | SEC0 Error    | Level         |               |

## ADSP-2184x SEC Trigger List

Table 7-3: ADSP-2184x SEC Trigger List Generators

|   Trigger ID | Name       | Description   | Sensitivity   |
|--------------|------------|---------------|---------------|
|          116 | SEC0_FAULT | SEC0 Fault    | Edge          |

Table 7-4: ADSP-2184x SEC Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## ADSP-2184x /ADSP-SC84x Interrupt List

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List

|   Interrupt ID | Name                   | Description                                 | Sensitivity   |   DMA Channel |
|----------------|------------------------|---------------------------------------------|---------------|---------------|
|              0 | SEC0_ERR               | SEC0 Error                                  | Level         |               |
|              1 | CANFD0_WU              | CANFD0 Wakeup Interrupt                     | None          |               |
|              2 | CANFD0_IRQ             | CANFD0 CAN Interrupt                        | None          |               |
|              3 | CANFD0_MSG             | CANFD0 Message Receive/Transmit In- terrupt | None          |               |
|              4 | CANFD1_WU              | CANFD1 Wakeup Interrupt                     | None          |               |
|              5 | CANFD1_IRQ             | CANFD1 CAN Interrupt                        | None          |               |
|              6 | CANFD1_MSG             | CANFD1 Message Receive/Transmit In- terrupt | None          |               |
|              7 | CGU0_EVT               | CGU0 Event                                  | Edge          |               |
|              8 | CGU1_EVT               | CGU1 Event                                  | Edge          |               |
|              9 | CNT0_STAT              | CNT0 Status                                 | Level         |               |
|             10 | C1_DATA_READ_ERR       | Core 1 Data Read Interrupt                  |               |               |
|             11 | C1_DATA_WRITE_ERR      | Core 1 Data Write Interrupt                 |               |               |
|             12 | C1_INST_READ_ERR       | Core 1 Instruction Read Interrupt           |               |               |
|             13 | C0_PMUIRQ              | Performance Monitoring Unit Interrupt       |               |               |
|             14 | LPDDR_LPDDR_DFI_ERR_2  | LPDDR LPDDR4 dfi0 error internal            | None          |               |
|             15 | LPDDR_LPDDR_SCRUB_IN T | LPDDR LPDDR4 scrub interrupt                | None          |               |
|             16 | GIC_PMU_INT            | GIC Performance monitor interrupt           |               |               |
|             21 | FX_PFATAL_ERR          | FX Fatal Error Interrupt                    |               |               |
|             32 | C1_IDMA_READ_ERR       | Core1 iDMA Read Interrupt                   | None          |               |
|             33 | C1_IDMA_WRITE_ERR      | Core 1 iDMA Write Interrupt                 | None          |               |
|             34 | CRC0_ERR               | CRC0 Error                                  | Level         |               |
|             35 | CRC1_ERR               | CRC1 Error                                  | Level         |               |
|             36 | MDMA0_SRC              | Memory DMAStream 0 Source Channel           |               |             8 |
|             37 | MDMA0_DST              | Memory DMAStream 0 Destination Channel      |               |             9 |
|             38 | MDMA1_SRC              | Memory DMAStream 1 Source Channel           |               |            18 |
|             39 | MDMA1_DST              | Memory DMAStream 1 Destination Channel      |               |            19 |
|             40 | CRC0_DCNTEXP           | CRC0 Datacount Expiration                   | Level         |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name           | Description                                     | Sensitivity   |   DMA Channel |
|----------------|----------------|-------------------------------------------------|---------------|---------------|
|             41 | CRC1_DCNTEXP   | CRC1 Datacount Expiration                       | Level         |               |
|             42 | CRC2_ERR       | CRC2 Error                                      | Level         |               |
|             43 | CRC3_ERR       | CRC3 Error                                      | Level         |               |
|             44 | MDMA4_SRC      | Memory DMAStream 0 Source Channel               |               |            45 |
|             45 | MDMA4_DST      | Memory DMAStream 0 Destination Channel          |               |            46 |
|             46 | MDMA5_SRC      | Memory DMAStream 1 Source Channel               |               |            47 |
|             47 | MDMA5_DST      | Memory DMAStream 1 Destination Channel          |               |            48 |
|             48 | CRC2_DCNTEXP   | CRC2 Datacount Expiration                       | Level         |               |
|             49 | CRC3_DCNTEXP   | CRC3 Datacount Expiration                       | Level         |               |
|             50 | MDMA0_SRC_ERR  | Enhanced BWMDMA0Channel 0 (Source) Error        |               |               |
|             51 | MDMA0_DST_ERR  | Enhanced BWMDMA0Channel 1 (Des- tination) Error |               |               |
|             52 | MDMA1_SRC_ERR  | Standard BWMDMA1Channel 0 (Source) Error        |               |               |
|             53 | MDMA1_DST_ERR  | Standard BWMDMA1Channel 1 (Desti- nation) Error |               |               |
|             54 | MDMA4_SRC_ERR  | Standard BWMDMA4Channel 0 (Source) Error        |               |               |
|             55 | MDMA4_DST_ERR  | Standard BWMDMA4Channel 1 (Desti- nation) Error |               |               |
|             56 | MDMA5_SRC_ERR  | Standard BWMDMA5Channel 0 (Source) Error        |               |               |
|             57 | MDMA5_DST_ERR  | Standard BWMDMA5Channel 1 (Desti- nation) Error |               |               |
|             58 | CSETR1_BUFINTR | CSETR1 CoreSight Trace ETR                      |               |               |
|             59 | CTI1_EVT0      | CTI1 Core 1 CTI Event                           | Level         |               |
|             64 | DAI0_IRQH      | DAI0 High Priority Interrupt                    | None          |               |
|             65 | DAI1_IRQH      | DAI1 High Priority Interrupt                    | None          |               |
|             66 | DAI0_IRQL      | DAI0 Low Priority Interrupt                     | None          |               |
|             67 | DAI1_IRQL      | DAI1 Low Priority Interrupt                     | None          |               |
|             68 | EMAC0_DMA2_RX  | EMAC0 DMAn Rx Channel Interrupt                 | None          |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name                    | Description                         | Sensitivity   | DMA Channel   |
|----------------|-------------------------|-------------------------------------|---------------|---------------|
|             70 | EMAC0_DMA3_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             71 | EMAC0_DMA4_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             72 | EMAC0_DMA5_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             73 | EMAC0_DMA6_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             74 | EMAC0_DMA7_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             75 | EMAC0_DMA0_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             82 | EMAC0_DMA1_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             84 | EMAC0_DMA3_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             86 | EMAC0_DMA4_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             87 | EMAC0_DMA5_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             88 | EMAC0_DMA6_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             89 | EMAC0_DMA7_RX           | EMAC0 DMAn Rx Channel Interrupt     | None          |               |
|             94 | EMAC0_STAT              | EMAC0 Status                        | None          |               |
|             95 | EMAC0_PWR               | EMAC0 Power Interrupt               | None          |               |
|             96 | EMAC0_DMA0_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             97 | EMAC0_DMA1_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             98 | EMAC0_DMA2_TX           | EMAC0 DMAn Tx Channel Interrupt     | None          |               |
|             99 | EMAC0_MAC               | EMAC0 MAC Interrupt                 | None          |               |
|            106 | EMDMA0_DONE             | EMDMA0 Transfer Done                | Edge          |               |
|            107 | EMDMA1_DONE             | EMDMA1 Transfer Done                | Edge          |               |
|            108 | FIR0_DMA                | FIR0 Core1DMA                       | Edge          |               |
|            109 | FIR0_STAT               | FIR0 Core 1 Status                  | Edge          |               |
|            110 | FIR1_DMA                | FIR1 Core1DMA                       | Edge          |               |
|            111 | FIR1_STAT               | FIR1 Core 1 Status                  | Edge          |               |
|            112 | FIR0_BUS_ERR            | FIR0 Core 1 FIR Bus Error           | Level         |               |
|            113 | FIR1_BUS_ERR            | FIR1 Core 1 FIR Bus Error           | Level         |               |
|            114 | FRACNPLL0_FRAC_LOCK     | FRACNPLL0 Frac Pll Lock Interrupt   | Edge          |               |
|            115 | FRACNPLL1_FRAC_LOCK     | FRACNPLL1 Frac Pll Lock Interrupt   | Edge          |               |
|            116 | FRACNPLL0_FRAC_UN- LOCK | FRACNPLL0 Frac Pll Unlock Interrupt | Edge          |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name                    | Description                                         | Sensitivity   | DMA Channel   |
|----------------|-------------------------|-----------------------------------------------------|---------------|---------------|
|            117 | FRACNPLL1_FRAC_UN- LOCK | FRACNPLL1 Frac Pll Unlock Interrupt                 | Edge          |               |
|            118 | HADC0_EVT               | HADC0 Event                                         | Edge          |               |
|            119 | HSM_HSM_INTR_0          | HSM Interrupt 0                                     | None          |               |
|            120 | HSM_HSM_INTR_1          | HSM Interrupt 1                                     | None          |               |
|            121 | HSM_HSM_INTR_2          | HSM Interrupt 2                                     | None          |               |
|            122 | HSM_HSM_INTR_3          | HSM Interrupt 3                                     | None          |               |
|            123 | HSM_HSM_INTR_4          | HSM Interrupt 4                                     | None          |               |
|            124 | HSM_HSM_INTR_5          | HSM Interrupt 5                                     | None          |               |
|            125 | HSM_HSM_INTR_6          | HSM Interrupt 6                                     | None          |               |
|            126 | HSM_HSM_INTR_7          | HSM Interrupt 7                                     | None          |               |
|            127 | HSM_HSM_MISC_INTR_0     | HSM Halt State Indication Interrupt                 | None          |               |
|            128 | HSM_HSM_MISC_INTR_1     | HSM Two Bit Error Indication Interrupt from HSM OTP | None          |               |
|            129 | HSM_HSM_MISC_INTR_2     | HSM Misc Interrupt 2                                | None          |               |
|            130 | HSM_HSM_MISC_INTR_3     | HSM Misc Interrupt 3                                | None          |               |
|            131 | IIR0_DMA                | IIR0 Core1DMA                                       | Edge          |               |
|            132 | IIR0_STAT               | IIR0 Core 1 Status                                  | Edge          |               |
|            133 | IIR1_DMA                | IIR1 Core1DMA                                       | Edge          |               |
|            134 | IIR1_STAT               | IIR1 Core 1 Status                                  | Edge          |               |
|            135 | IIR2_DMA                | IIR2 Core1DMA                                       | Edge          |               |
|            136 | IIR2_STAT               | IIR2 Core 1 Status                                  | Edge          |               |
|            137 | IIR3_DMA                | IIR3 Core1DMA                                       | Edge          |               |
|            138 | IIR3_STAT               | IIR3 Core 1 Status                                  | Edge          |               |
|            139 | IIR0_BUS_ERR            | IIR0 Core 1 IIR Bus Error                           | Level         |               |
|            140 | IIR1_BUS_ERR            | IIR1 Core 1 IIR Bus Error                           | Level         |               |
|            141 | IIR2_BUS_ERR            | IIR2 Core 1 IIR Bus Error                           | Level         |               |
|            142 | IIR3_BUS_ERR            | IIR3 Core 1 IIR Bus Error                           | Level         |               |
|            143 | L2CTL0_ECC_ERR          | L2CTL0 ECC Error                                    | Level         |               |
|            145 | L2CTL0_EVT              | L2CTL0 Scrub/Initialization Done                    | Level         |               |
|            146 | L2CTL1_ECC_ERR          | L2CTL1 ECC Error                                    | Level         |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name                          | Description                                 | Sensitivity   |   DMA Channel |
|----------------|-------------------------------|---------------------------------------------|---------------|---------------|
|            148 | L2CTL1_EVT                    | L2CTL1 Scrub/Initialization Done            | Level         |               |
|            152 | LP0_DMA                       | LP0 DMAChannel                              |               |            30 |
|            153 | LP0_STAT                      | LP0 Status                                  |               |               |
|            154 | LP1_DMA                       | LP1 DMAChannel                              |               |            36 |
|            155 | LP1_STAT                      | LP1 Status                                  |               |               |
|            156 | LP0_DMA_ERR                   | LP0 DMAData Error                           |               |               |
|            157 | LP1_DMA_ERR                   | LP1 DMAData Error                           |               |               |
|            158 | LPDDR_LPDDR_ECC_CERR          | LPDDR LPDDR4 ecc corrected err intr         | None          |               |
|            159 | LPDDR_LPDDR_ECC_CFLT          | LPDDR LPDDR4 ecc corrected err intr fault   | None          |               |
|            160 | LPDDR_LPDDR_ECC_UCER R        | LPDDR LPDDR4 ecc uncorrected err intr       | None          |               |
|            161 | LPDDR_LPDDR_ECC_UCFL T        | LPDDR LPDDR4 ecc uncorrected err intr fault | None          |               |
|            163 | LPDDR_LPDDR_DRT_TMP_ LMT_INTR | LPDDR LPDDR4 derate temp limit intr         | None          |               |
|            164 | LPDDR_LPDDR_DRT_TMP_ LMT_FLT  | LPDDR LPDDR4 derate temp limit intr fault   | None          |               |
|            165 | LPDDR_LPDDR_DFI_ERR           | LPDDR LPDDR4 dfi0 error PHY                 | None          |               |
|            166 | LPDDR_LPDDR_DDRPHY_I NT_N     | LPDDR LPDDR4 dwc ddrphy int                 | None          |               |
|            167 | MDMA2_SRC                     | Enhanced BWMDMA2Channel 0 (Source)          |               |            39 |
|            168 | MDMA2_DST                     | Enhanced BWMDMA2Channel 1 (Des- tination)   |               |            40 |
|            169 | MDMA3_SRC                     | Max BWMDMA3Channel 0 (Source)               |               |            43 |
|            170 | MDMA3_DST                     | Max BWMDMA3Channel 1 (Destina- tion)        |               |            44 |
|            171 | MDMA7_SRC                     | Max BWMDMA7Channel 0 (Source)               |               |            51 |
|            172 | MDMA7_DST                     | Max BWMDMA7Channel 1 (Destina- tion)        |               |            52 |
|            173 | MDMA6_SRC                     | Enhanced BWMDMA6Channel 0 (Source)          |               |            49 |
|            174 | MDMA6_DST                     | Enhanced BWMDMA6Channel 1 (Des- tination)   |               |            50 |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name          | Description                                                             | Sensitivity   | DMA Channel   |
|----------------|---------------|-------------------------------------------------------------------------|---------------|---------------|
|            175 | MDMA2_SRC_ERR | Standard BWMDMA2Channel 0 (Source) Error                                |               |               |
|            176 | MDMA2_DST_ERR | Standard BWMDMA2Channel 1 (Desti- nation) Error                         |               |               |
|            177 | MDMA3_SRC_ERR | Standard BWMDMA3Channel 0 (Source) Error                                |               |               |
|            178 | MDMA3_DST_ERR | Standard BWMDMA3Channel 1 (Desti- nation) Error                         |               |               |
|            179 | MDMA6_SRC_ERR | Standard BWMDMA6Channel 0 (Source) Error                                |               |               |
|            180 | MDMA6_DST_ERR | Standard BWMDMA6Channel 1 (Desti- nation) Error                         |               |               |
|            181 | MDMA7_SRC_ERR | Standard BWMDMA7Channel 0 (Source) Error                                |               |               |
|            182 | MDMA7_DST_ERR | Standard BWMDMA7Channel 1 (Desti- nation) Error                         |               |               |
|            183 | MEC1_EEIRQ0   | MEC1 ECC Error Interrupt Request                                        | Level         |               |
|            184 | MEC1_EWIRQ0   | MEC1 ECC Warning to Interrupt Re- quest (Reserved)                      | Level         |               |
|            185 | MEC1_MEIRQ1   | MEC1 Parity Error Interrupt Request                                     | Level         |               |
|            186 | MEC1_MFIRQ00  | MEC1 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            187 | MEC1_MEIRQ0   | MEC1 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            188 | MEC1_MFIRQ20  | MEC1 A55 Core Memory Fault Inter- rupt/Trigger Request Bus              | Level         |               |
|            189 | MEC1_MEIRQ2   | MEC1 Core Memory Error Inter- rupt/Trigger Request Bus                  | Level         |               |
|            190 | MEC1_PEIRQ0   | MEC1 Parity Error Interrupt/Trigger Re- quest Bus                       | Level         |               |
|            191 | MEC2_EEIRQ0   | MEC2 ECC Error Interrupt/Trigger Re- quest Bus                          | Level         |               |
|            192 | MEC2_EWIRQ0   | MEC2 ECC Warning Interrupt/Trigger Request Bus (Reserved)               | Level         |               |
|            193 | MEC2_MWIRQ1   | MEC2 Core Memory ECC Warning In- terrupt/Trigger Request Bus (Reserved) | Level         |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name            | Description                                                              | Sensitivity   | DMA Channel   |
|----------------|-----------------|--------------------------------------------------------------------------|---------------|---------------|
|            194 | MEC2_MEIRQ1     | MEC2 Core Memory Error Inter- rupt/Trigger Request Bus                   | Level         |               |
|            195 | MEC2_MFIRQ00    | MEC2 A55 Core Memory Fault Inter- rupt/Trigger Request Bus               | Level         |               |
|            196 | MEC2_MEIRQ0     | MEC2 Core Memory Error Inter- rupt/Trigger Request Bus                   | Level         |               |
|            197 | MEC2_MFIRQ20    | MEC2 A55 Core Memory Fault Inter- rupt/Trigger Request Bus               | Level         |               |
|            198 | MEC2_MEIRQ2     | MEC2 Core Memory Error Inter- rupt/Trigger Request Bus                   | Level         |               |
|            199 | MEC2_PEIRQ0     | MEC2 Parity Error Interrupt/Trigger Re- quest Bus                        | Level         |               |
|            200 | MEC1_MWIRQ1     | MEC1 MEC1 Core 1 Mem Warning to Core 1 (Reserved)                        | Level         |               |
|            201 | MEC0_EEIRQ0     | MEC0 ECC Error Interrupt/Trigger Re- quest Bus                           | Level         |               |
|            202 | MEC0_EWIRQ0     | MEC0 ECC Warning Interrupt/Trigger Request Bus (Reserved)                | Level         |               |
|            203 | MEC0_MWIRQ1     | MEC0 Core Memory ECC Warning In- terrupt/Trigger Request Bus (Reserved)  | Level         |               |
|            204 | MEC0_MEIRQ1     | MEC0 Core Memory Error Inter- rupt/Trigger Request Bus                   | Level         |               |
|            205 | MEC0_MFIRQ00    | MEC0 A55 Core Memory Fault Inter- rupt/Trigger Request Bus               | Level         |               |
|            206 | MEC0_MEIRQ0     | MEC0 Core Memory Error Inter- rupt/Trigger Request Bus                   | Level         |               |
|            207 | MEC0_MFIRQ20    | MEC0 A55 Core Memory Fault Inter- rupt/Trigger Request Bus               | Level         |               |
|            208 | MEC0_MEIRQ2     | MEC0 Core Memory Error Inter- rupt/Trigger Request Bus                   | Level         |               |
|            209 | MEC0_PEIRQ0     | MEC0 Parity Error Interrupt/Trigger Re- quest Bus                        | Level         |               |
|            210 | MREPAIR0_MEMREP | MREPAIR0 Memory Repair and BISR Register Clear Done Interrupt (Reserved) | None          |               |
|            211 | MLB0_INT0       | MLB0 Interrupt 0                                                         |               |               |
|            212 | MLB0_INT1       | MLB0 Interrupt 1                                                         |               |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name          | Description                     | Sensitivity   |   DMA Channel |
|----------------|---------------|---------------------------------|---------------|---------------|
|            213 | MLB0_STAT     | MLB0 Status                     |               |               |
|            214 | EMSI0_STAT    | EMSI0 Mshc Interrupt            | None          |               |
|            215 | EMSI0_WAKEUP  | EMSI0 MSHC wake interrupt       | None          |               |
|            216 | OTPC0_ERR     | OTPC0 Dual-bit Error            | Level         |               |
|            217 | PINT0_BLOCK   | PINT0 Pin Interrupt Block 0     | Level         |               |
|            218 | PINT1_BLOCK   | PINT1 Pin Interrupt Block 1     | Level         |               |
|            219 | PINT2_BLOCK   | PINT2 Pin Interrupt Block 2     | Level         |               |
|            220 | PINT3_BLOCK   | PINT3 Pin Interrupt Block 3     | Level         |               |
|            221 | PINT4_BLOCK   | PINT4 Pin Interrupt Block 4     | Level         |               |
|            222 | PINT5_BLOCK   | PINT5 Pin Interrupt Block 5     | Level         |               |
|            223 | PINT6_BLOCK   | PINT6 Pin Interrupt Block 6     | Level         |               |
|            224 | PINT7_BLOCK   | PINT7 Pin Interrupt Block 7     | Level         |               |
|            225 | PKIC0_IRQ     | PKIC0 Interrupt                 | Level         |               |
|            226 | PKTE0_IRQ     | PKTE0 Interrupt                 | Level         |               |
|            227 | PWM0_TRIP     | PWM0 Trip                       | Level         |               |
|            228 | PWM0_SYNC     | PWM0 PWMTMRGrouped              | Edge          |               |
|            229 | SMPU_AGGR_INT | SMPU Aggregated Interrupt/Event |               |               |
|            230 | SPI0_TXDMA    | SPI0 TX DMAChannel              | Level         |            22 |
|            231 | SPI0_RXDMA    | SPI0 RX DMAChannel              | Level         |            23 |
|            232 | SPI0_STAT     | SPI0 Status                     | Level         |               |
|            233 | SPI0_ERR      | SPI0 Error                      | Level         |               |
|            234 | SPI1_TXDMA    | SPI1 TX DMAChannel              | Level         |            24 |
|            235 | SPI1_RXDMA    | SPI1 RX DMAChannel              | Level         |            25 |
|            236 | SPI1_STAT     | SPI1 Status                     | Level         |               |
|            237 | SPI1_ERR      | SPI1 Error                      | Level         |               |
|            238 | SPI2_TXDMA    | SPI2 TX DMAChannel              | Level         |            26 |
|            239 | SPI2_RXDMA    | SPI2 RX DMAChannel              | Level         |            27 |
|            240 | SPI2_STAT     | SPI2 Status                     | Level         |               |
|            241 | SPI2_ERR      | SPI2 Error                      | Level         |               |
|            242 | SPI5_TXDMA    | SPI5 TX DMAChannel              | Level         |            59 |
|            243 | SPI5_RXDMA    | SPI5 RX DMAChannel              | Level         |            60 |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name           | Description              | Sensitivity   |   DMA Channel |
|----------------|----------------|--------------------------|---------------|---------------|
|            244 | SPI5_STAT      | SPI5 Status              | Level         |               |
|            245 | SPI5_ERR       | SPI5 Error               | Level         |               |
|            246 | SPI0_TXDMA_ERR | SPI0 TX DMAChannel Error | Level         |               |
|            247 | SPI0_RXDMA_ERR | SPI0 RX DMAChannel Error | Level         |               |
|            248 | SPI1_TXDMA_ERR | SPI1 TX DMAChannel Error | Level         |               |
|            249 | SPI1_RXDMA_ERR | SPI1 RX DMAChannel Error | Level         |               |
|            250 | SPI2_TXDMA_ERR | SPI2 TX DMAChannel Error | Level         |               |
|            251 | SPI2_RXDMA_ERR | SPI2 RX DMAChannel Error | Level         |               |
|            252 | SPI5_TXDMA_ERR | SPI5 TX DMAError         | Level         |               |
|            253 | SPI5_RXDMA_ERR | SPI5 RX DMAError         | Level         |               |
|            254 | SPORT0_A_DMA   | SPORT0 ChannelADMA       | Level         |             0 |
|            255 | SPORT0_A_STAT  | SPORT0 Channel A Status  | Level         |               |
|            256 | SPORT0_B_DMA   | SPORT0 ChannelBDMA       | Level         |             1 |
|            257 | SPORT0_B_STAT  | SPORT0 Channel B Status  | Level         |               |
|            258 | SPORT1_A_DMA   | SPORT1 ChannelADMA       | Level         |             2 |
|            259 | SPORT1_A_STAT  | SPORT1 Channel A Status  | Level         |               |
|            260 | SPORT1_B_DMA   | SPORT1 ChannelBDMA       | Level         |             3 |
|            261 | SPORT1_B_STAT  | SPORT1 Channel B Status  | Level         |               |
|            262 | SPORT2_A_DMA   | SPORT2 ChannelADMA       | Level         |             4 |
|            263 | SPORT2_A_STAT  | SPORT2 Channel A Status  | Level         |               |
|            264 | SPORT2_B_DMA   | SPORT2 ChannelBDMA       | Level         |             5 |
|            265 | SPORT2_B_STAT  | SPORT2 Channel B Status  | Level         |               |
|            266 | SPORT3_A_DMA   | SPORT3 ChannelADMA       | Level         |             6 |
|            267 | SPORT3_A_STAT  | SPORT3 Channel A Status  | Level         |               |
|            268 | SPORT3_B_DMA   | SPORT3 ChannelBDMA       | Level         |             7 |
|            269 | SPORT3_B_STAT  | SPORT3 Channel B Status  | Level         |               |
|            270 | SPORT4_A_DMA   | SPORT4 ChannelADMA       | Level         |            10 |
|            271 | SPORT4_A_STAT  | SPORT4 Channel A Status  | Level         |               |
|            272 | SPORT4_B_DMA   | SPORT4 ChannelBDMA       | Level         |            11 |
|            273 | SPORT4_B_STAT  | SPORT4 Channel B Status  | Level         |               |
|            274 | SPORT5_A_DMA   | SPORT5 ChannelADMA       | Level         |            12 |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name                | Description                   | Sensitivity   |   DMA Channel |
|----------------|---------------------|-------------------------------|---------------|---------------|
|            275 | SPORT5_A_STAT       | SPORT5 Channel A Status       | Level         |               |
|            276 | SPORT5_B_DMA        | SPORT5 ChannelBDMA            | Level         |            13 |
|            277 | SPORT5_B_STAT       | SPORT5 Channel B Status       | Level         |               |
|            278 | SPORT6_A_DMA        | SPORT6 ChannelADMA            | Level         |            14 |
|            279 | SPORT6_A_STAT       | SPORT6 Channel A Status       | Level         |               |
|            280 | SPORT6_B_DMA        | SPORT6 ChannelBDMA            | Level         |            15 |
|            281 | SPORT6_B_STAT       | SPORT6 Channel B Status       | Level         |               |
|            282 | SPORT7_A_DMA        | SPORT7 ChannelADMA            | Level         |            16 |
|            283 | SPORT7_A_STAT       | SPORT7 Channel A Status       | Level         |               |
|            284 | SPORT7_B_DMA        | SPORT7 ChannelBDMA            | Level         |            17 |
|            285 | SPORT7_B_STAT       | SPORT7 Channel B Status       | Level         |               |
|            286 | DAI0_GBL_SPORT_INT0 | DAI0 Global SPORT Interrupt 0 | None          |               |
|            287 | DAI0_GBL_SPORT_INT1 | DAI0 Global SPORT Interrupt 1 | None          |               |
|            288 | DAI1_GBL_SPORT_INT0 | DAI1 Global SPORT Interrupt 0 | None          |               |
|            289 | DAI1_GBL_SPORT_INT1 | DAI1 Global SPORT Interrupt 1 | None          |               |
|            290 | SPORT0_A_DMA_ERR    | SPORT0 Channel A DMAError     | Level         |               |
|            291 | SPORT0_B_DMA_ERR    | SPORT0 Channel B DMAError     | Level         |               |
|            292 | SPORT1_A_DMA_ERR    | SPORT1 Channel A DMAError     | Level         |               |
|            293 | SPORT1_B_DMA_ERR    | SPORT1 Channel B DMAError     | Level         |               |
|            294 | SPORT2_A_DMA_ERR    | SPORT2 Channel A DMAError     | Level         |               |
|            295 | SPORT2_B_DMA_ERR    | SPORT2 Channel B DMAError     | Level         |               |
|            296 | SPORT3_A_DMA_ERR    | SPORT3 Channel A DMAError     | Level         |               |
|            297 | SPORT3_B_DMA_ERR    | SPORT3 Channel B DMAError     | Level         |               |
|            298 | SPORT4_A_DMA_ERR    | SPORT4 Channel A DMAError     | Level         |               |
|            299 | SPORT4_B_DMA_ERR    | SPORT4 Channel B DMAError     | Level         |               |
|            300 | SPORT5_A_DMA_ERR    | SPORT5 Channel A DMAError     | Level         |               |
|            301 | SPORT5_B_DMA_ERR    | SPORT5 Channel B DMAError     | Level         |               |
|            302 | SPORT6_A_DMA_ERR    | SPORT6 Channel A DMAError     | Level         |               |
|            303 | SPORT6_B_DMA_ERR    | SPORT6 Channel B DMAError     | Level         |               |
|            304 | SPORT7_A_DMA_ERR    | SPORT7 Channel A DMAError     | Level         |               |
|            305 | SPORT7_B_DMA_ERR    | SPORT7 Channel B DMAError     | Level         |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name         | Description                       | Sensitivity   | DMA Channel   |
|----------------|--------------|-----------------------------------|---------------|---------------|
|            306 | SPU0_INT     | SPU0 Interrupt                    | Level         |               |
|            307 | SWU0_EVT     | SWU0 Event SMC                    | None          |               |
|            308 | SWU1_EVT     | SWU1 Event L2 Memory DMAPort 0    | None          |               |
|            309 | SWU2_EVT     | SWU2 Event L2 Memory Core Port 0  | None          |               |
|            310 | SWU7_EVT     | SWU7 Event SHARC0 Target Port 1   | None          |               |
|            311 | SWU8_EVT     | SWU8 Event SHARC0 Target Port 2   | None          |               |
|            312 | SWU11_EVT    | SWU11 Event SMMR                  | None          |               |
|            313 | SWU12_EVT    | SWU12 Event SPI2/XSPI             | None          |               |
|            314 | SWU13_EVT    | SWU13 Event DMC0                  | None          |               |
|            315 | SWU3_EVT     | SWU3 Event CL2_1                  | None          |               |
|            316 | SWU4_EVT     | SWU4 Event DL2_1                  | None          |               |
|            317 | SWU5_EVT     | SWU5 Event CL2_2                  | None          |               |
|            318 | SOFT8_INT    | Software Interrupt 8              |               |               |
|            319 | SOFT9_INT    | Software Interrupt 9              |               |               |
|            320 | SOFT0_INT    | Software Interrupt 0              |               |               |
|            321 | SOFT1_INT    | Software Interrupt 1              |               |               |
|            322 | SOFT2_INT    | Software Interrupt 2              |               |               |
|            323 | SOFT3_INT    | Software Interrupt 3              |               |               |
|            324 | SOFT4_INT    | Software Interrupt 4              |               |               |
|            325 | SOFT5_INT    | Software Interrupt 5              |               |               |
|            326 | SOFT6_INT    | Software Interrupt 6              |               |               |
|            327 | SOFT7_INT    | Software Interrupt 7              |               |               |
|            328 | TAPC_KEYFAIL | TAPC Test/User Key Fail Interrupt | Edge          |               |
|            329 | TIMER0_TMR00 | TIMER0 Timer 0                    | Level         |               |
|            330 | TIMER0_TMR01 | TIMER0 Timer 1                    | Level         |               |
|            331 | TIMER0_TMR02 | TIMER0 Timer 2                    | Level         |               |
|            332 | TIMER0_TMR03 | TIMER0 Timer 3                    | Level         |               |
|            333 | TIMER0_TMR04 | TIMER0 Timer 4                    | Level         |               |
|            334 | TIMER0_TMR05 | TIMER0 Timer 5                    | Level         |               |
|            335 | TIMER0_TMR06 | TIMER0 Timer 6                    | Level         |               |
|            336 | TIMER0_TMR07 | TIMER0 Timer 7                    | Level         |               |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name           | Description               | Sensitivity   |   DMA Channel |
|----------------|----------------|---------------------------|---------------|---------------|
|            337 | TIMER0_TMR08   | TIMER0 Timer 8            | Level         |               |
|            338 | TIMER0_TMR09   | TIMER0 Timer 9            | Level         |               |
|            339 | TIMER0_TMR10   | TIMER0 Timer 10           | Level         |               |
|            340 | TIMER0_TMR11   | TIMER0 Timer 11           | Level         |               |
|            341 | TIMER0_TMR12   | TIMER0 Timer 12           | Level         |               |
|            342 | TIMER0_TMR13   | TIMER0 Timer 13           | Level         |               |
|            343 | TIMER0_TMR14   | TIMER0 Timer 14           | Level         |               |
|            344 | TIMER0_TMR15   | TIMER0 Timer 15           | Level         |               |
|            345 | TIMER0_STAT    | TIMER0 Status             | Level         |               |
|            346 | HADC0_FIFO_EVT | HADC0 FIFO Event          | Edge          |               |
|            347 | TMU0_FAULT     | TMU0 Fault                |               |               |
|            348 | TMU0_ALERT     | TMU0 Alert                |               |               |
|            349 | TRU0_RCV0      | TRU0 Interrupt 0 - Core 0 | Edge          |               |
|            350 | TRU0_RCV1      | TRU0 Interrupt 1 - Core 0 | Edge          |               |
|            351 | TRU0_RCV2      | TRU0 Interrupt 2 - Core 0 | Edge          |               |
|            352 | TRU0_RCV3      | TRU0 Interrupt 3 - Core 0 | Edge          |               |
|            353 | TRU0_RCV4      | TRU0 Interrupt 4          | Edge          |               |
|            354 | TRU0_RCV5      | TRU0 Interrupt 5          | Edge          |               |
|            355 | TRU0_RCV6      | TRU0 Interrupt 6          | Edge          |               |
|            356 | TRU0_RCV7      | TRU0 Interrupt 7          | Edge          |               |
|            357 | TRU0_RCV8      | TRU0 Interrupt 8          | Edge          |               |
|            358 | TRU0_RCV9      | TRU0 Interrupt 9          | Edge          |               |
|            359 | TRU0_RCV10     | TRU0 Interrupt 10         | Edge          |               |
|            360 | TRU0_RCV11     | TRU0 Interrupt 11         | Edge          |               |
|            361 | TWI0_DATA      | TWI0 Data Interrupt       | Level         |               |
|            362 | TWI1_DATA      | TWI1 Data Interrupt       | Level         |               |
|            363 | TWI2_DATA      | TWI2 Data Interrupt       | Level         |               |
|            364 | TWI3_DATA      | TWI3 Data Interrupt       | Level         |               |
|            365 | TWI4_DATA      | TWI4 Data Interrupt       | Level         |               |
|            366 | TWI5_DATA      | TWI5 Data Interrupt       | Level         |               |
|            367 | UART0_TXDMA    | UART0 TransmitDMA         | Level         |            20 |

Table 7-5: ADSP-2184x /ADSP-SC84x Interrupt List (Continued)

|   Interrupt ID | Name            | Description             | Sensitivity   |   DMA Channel |
|----------------|-----------------|-------------------------|---------------|---------------|
|            368 | UART0_RXDMA     | UART0 ReceiveDMA        | Level         |            21 |
|            369 | UART0_STAT      | UART0 Status            | Level         |               |
|            370 | UART1_TXDMA     | UART1 TransmitDMA       | Level         |            34 |
|            371 | UART1_RXDMA     | UART1 ReceiveDMA        | Level         |            35 |
|            372 | UART1_STAT      | UART1 Status            | Level         |               |
|            373 | UART2_TXDMA     | UART2 TransmitDMA       | Level         |            37 |
|            374 | UART2_RXDMA     | UART2 ReceiveDMA        | Level         |            38 |
|            375 | UART2_STAT      | UART2 Status            | Level         |               |
|            376 | UART0_TXDMA_ERR | UART0 Transmit DMAError | Level         |               |
|            377 | UART0_RXDMA_ERR | UART0 Receive DMAError  | Level         |               |
|            378 | UART1_TXDMA_ERR | UART1 Transmit DMAError | Level         |               |
|            379 | UART1_RXDMA_ERR | UART1 Receive DMAError  | Level         |               |
|            380 | UART2_TXDMA_ERR | UART2 Transmit DMAError | Level         |               |
|            381 | UART2_RXDMA_ERR | UART2 Receive DMAError  | Level         |               |
|            382 | WDOG0_EXP       | WDOG0 Expiration        | Level         |               |
|            383 | WDOG1_EXP       | WDOG1 Expiration        | Level         |               |
|            384 | WDOG2_EXP       | WDOG2 Expiration        | Level         |               |
|            385 | WDOG3_EXP       | WDOG3 Expiration        | Level         |               |
|            386 | XSPI0_IRQ       | XSPI0 Interrupt Request | None          |               |
|            387 | XSPI1_IRQ       | XSPI1 Interrupt Request | None          |               |

## SEC Definitions

The event controller uses the following definitions.

## SCI

SEC core interface, core interface subblock of the SEC

## SEA

SEC system event aggregator generates an aggregate request to the SEC based on the active events at a given time.

## SID (Identification, unique)

Source numeric identifier for each system source connected to the SEC.

## SFI

SEC Fault Interface, fault management subblock of the SEC.

## SPR

SEC prioritizer determines the highest priority pending interrupt and the highest priority active interrupt. The SPR provides these interrupts in the appropriate registers of the SCI for the priority and nesting model of the SCI.

## SSI

SEC source interface, system event source control, and status subblock of the SEC.

## System Events

System source indications including interrupts and faults.

## System Source

Point of origin of system event.

## SEC System Event Aggregator (SEA)

System event sources request event service from the SEC. To reduce the number of required SSI channels in the SEC, the System Event Aggregator (SEA) acts as an intermediate aggregator between the source and the SEC. All unique system events have dedicated inputs to the SEA. The SEA generates an aggregate request to the SEC based on which events are active at a given time.

To determine the active event for a given SEC ID, read the SEC\_SSTAT[n].CHID bit field. The Combined SEC and GIC Interrupt List table shows five SEAs connected to SEC ID 251-255. All the sources grouped over the SEA are error events for a set of peripherals like the FIR-IIR, SPI-UART-LP-EPPI, MDMA, SPORT, and SWU. An error event from a peripheral is only expected when that peripheral is enabled. To mask SEA events from sources, disable the source peripheral conditions. There are no registers to mask events from the SEA, but there is a provision to mask the complete group output at the SEC.

Events are handled with a fixed priority order, on a first-come, first-served basis. If multiple events corresponding to the same SEC ID are active at the same time, the priority level is order dependent with the lower SEA IDs having the higher priority. The SEA events are level-based, so they remain high until the events are cleared at the source ensuring no event is missed. There is no support for nesting in a group since they all have same the SEC ID. An SEA event must be handled completely before addressing the next event.

## SEC Block Diagram

The SEC Block Diagram shows the event management architecture.

System sources connect to the SEC through the SSI. Each core has a dedicated SCI. The SFI provides fault action connections to the rest of the system.

Figure 7-2: SEC Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000001_407127976b3af3f55300a938130d66115af0f80638b6c7512adbbbcdb473f96c.png)

As shown in the figure, the SEC has three blocks for event management. The SFI monitors and manages any fault event triggered from various fault input sources. System interrupt sources are routed to the SFI through SEC source interface (SSI).

## SEC Fault Interface (SFI)

The SFI manages fault events and associated actions. The fault management support provided in the SEC helps satisfy the safety requirements of various applications. The SSI provides the highest priority pending source that is enabled as a fault. The SFI captures this value and enables a countdown, and once the countdown expires, takes the prescribed action.

Fault actions which can be configured, as shown in SFI Block Diagram , include

- Trigger Output
- System Reset
- Fault Output
- Computer Operating Properly (COP) mode
- Fault Mode

Figure 7-3: SFI Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000002_79a37bac136f5779b28c4d0dbe37f40349290bc30d2b5cc07a20676ba9b0428d.png)

## Fault Management

System sources can be enabled as fault sources in the SEC\_SCTL[n] register. When a source enabled as a fault moves to pending, it is forwarded to the SFI as a fault indication. The pending bit ( SEC\_FSTAT.PND ) indicates a source has signaled a fault assertion but it has not yet triggered the event actions (if delay is enabled). The SEC fault interface sets the SEC\_FSTAT.PND bit when the fault source ID register ( SEC\_FSID ) is updated on assertion of a fault source input. The system source pending triggers a fault pending and after a programmable delay the fault moves to active. Event actions then execute if appropriate action is not taken by the core. The SEC\_FSTAT.ACT bit indicates that the SEC has received a fault source input, the delay has expired, and the fault actions are enabled.

The SEC\_FSTAT.NPND bit indicates if one or more sources have signaled a fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC\_FSTAT.NPND bit when the fault interface detects assertion of any enabled fault source input, while either the SEC\_FSTAT.PND or SEC\_FSTAT.ACT bits are set. The SEC clears the SEC\_FSTAT.NPND bit when there are no fault sources waiting.

A fault indication from an external device can also be detected on sampling the fault signals. When a fault is detected the SEC\_FSTAT.ACT and SEC\_FSID.FEXT bits are set. The assertion of either signal results in a fault input detection.

The SEC\_FEND register receives a fault end indication from the core. The core writes the SID of the fault to the SEC\_FEND register. If the SID matches the value in the SEC\_FSID register, the SEC\_FSTAT.PND and SEC\_FSTAT.ACT bits are cleared.

## SEC Core Interface (SCI)

The SCI manages communication between the corresponding core and the SEC. The SEC prioritizer (SPR) of the SCI receives pending, active, and priority information from the SSI for each system event source assigned to this SCI. The SPR determines the highest priority pending system event and the SCI determines whether it propagates to the core. The SCI maintains the coherency for the system event service model implemented on the connected core.

Figure 7-4: SCI Overview Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000003_51747033405e0de0314e7dc33933819680e771be4c1d4e4059f883dec188bc3b.png)

## SEC Source Interface (SSI)

The SSI manages all of the system event sources. It maintains the status of each source in the corresponding SEC\_SSTAT[n] register. The corresponding SEC\_SCTL[n] register manages the control of each source. A pending and enabled event passes its indication and priority to the SCI to which it is assigned for further processing.

Figure 7-5: SSI Overview Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000004_d2bff1b826c132654b802d56836865e1f7061555ab18d6ab494e9658e065c9c8.png)

## SEC Architectural Concepts

The following sections describe SEC architectural features.

## System Interrupt Groups

System sources can be assigned to groups using the SEC\_SCTL[n].GRP bit field. Source groups allow fast context switching for system interrupts at each SCI. The SEC\_CGMSK[n] register allows quick masking of interrupt groups of unlimited size with a single write operation.

## System Interrupt Flow

An enabled and asserted system interrupt source is latched at the SSI and routed to the appropriate SCI based on the core target select ( SEC\_SCTL[n].CTG ) bit field setting. The SEC priority ordering determines the highest priority pending system interrupt and the SCI updates the SEC\_CPND[n].SID and SEC\_CACT[n].PRIO bit

field values. The SCI compares the SEC\_CPND[n] register value against the highest priority active source in the SEC\_CACT[n] register).

The priority level register ( SEC\_CPLVL[n] ) determines how many of the MSBs the SEC uses in the comparison. The priority mask register ( SEC\_CPMSK[n] ) and the group mask register ( SEC\_CGMSK[n] ) determines which pending interrupt sources participate. If the SEC\_CPND[n] register value is a higher priority (lower value) than the priority of the SEC\_CACT[n] register from the comparison based on the SEC\_CPLVL[n] register, the system interrupt output is asserted. The source ID register ( SEC\_CSID[n] ) is updated with the SEC\_CPND[n].SID bit field value and forwarded to the connected core.

After the core provides an interrupt acknowledgment, the interrupt source is active, until the SEC completes interrupt service with a write to the SEC\_END.SID bit field with the same value. Note the following:

- Interrupt acknowledgement occurs with an MMR write of the SEC\_CSID[n] register or the core version of the SEC\_CSID[n] register.
- Interrupt active status indication is SEC\_SSTAT[n].ACT ==1.

The following sequence shows the example flow for a single interrupt.

1. The SEC compares the SEC\_CPND[n] register value to the SEC\_CACT[n] register value. If the interrupt in the SEC\_CPND[n] register is higher priority, continue.
2. The SEC copies the SEC\_CPND[n] register value to the SEC\_CSID[n] register and asserts the interrupt signal.
3. The core reads the SEC\_CSID[n] register (or core version).
4. The core writes to the SEC\_CSID[n] register (or core version, asserts the acknowledge signal).
5. The SEC deasserts the interrupt signal and clears the SEC\_SSTAT[n].PND bit and sets the SEC\_SSTAT[n].ACT bit of the source going active.
6. The core writes the SEC\_CSID[n] of the active interrupt to the SEC\_END register.
7. The SEC clears the SEC\_SSTAT[n].ACT bit of the source being ended.

The following sequence shows the example flow for interrupt nesting where interrupt A is a lower priority and occurs earlier than interrupt B.

1. The SEC compares the SEC\_CPND[n] (A) register value to the SEC\_CACT[n] register and if the interrupt in the SEC\_CPND[n] register is a higher priority, continue.
2. The SEC copies SEC\_CPND[n] (A) register to the SEC\_CSID[n] register and asserts the interrupt signal.
3. The core reads the SEC\_CSID[n] (A) register (or core version).
4. The core writes to the SEC\_CSID[n] register (or core version, asserts the acknowledge signal).
5. The SEC deasserts the INT signal and clears the SEC\_SSTAT[n].PND bit and sets the SEC\_SSTAT[n].ACT bit of the source (A) going active.

6. The SEC compares the SEC\_CPND[n] (B) register value to the SEC\_CACT[n] (A) register value. If the SEC\_CACT[n] (B) register value is a higher priority, continue.
7. The SEC copies the SEC\_CPND[n] (B) register value to SEC\_CSID[n] register and asserts the interrupt signal.
8. The core reads the SEC\_CSID[n] (B) register (or core version).
9. The core writes to the SEC\_CSID[n] register (or core version, asserts the acknowledge signal).
10. The SEC deasserts the INT signal and clears the SEC\_SSTAT[n].PND bit and sets the SEC\_SSTAT[n].ACT bit of the source (B) going active.
11. The core writes the SEC\_CSID[n] of the active interrupt (B) to the SEC\_END register.
12. The SEC clears the SEC\_SSTAT[n].ACT bit of the source (B) being ended.
13. The core writes the SEC\_CSID[n] of the active interrupt (A) to the SEC\_END register.
14. The SEC clears the SEC\_SSTAT[n].ACT bit of the source (A) being ended.

## System Interrupt Priorities

Each system interrupt source has its own programmable priority level which is configured using the SEC\_SCTL[n].PRIO bit field. The SCI evaluates the priority of all pending sources to determine the source of the highest priority pending system interrupt for forwarding to the attached core. If more than one source of the pending system interrupt has the same priority setting, the SCI chooses the one with the lowest SID. For example, if SID 0, SID 1, and SID 2 are all pending and have the same priority setting, the SCI chooses SID 0 as the highest-priority source.

## SEC Error

The processor includes an SEC error ( SEC\_GSTAT.ERR ) as a source input to the SEC to allow for handling the error as an interrupt or fault.

## SEC Programming Model

Implementing a system interrupt service model using the SEC requires, at a minimum:

- Proper configuration of a system interrupt source (for example a peripheral or DMA)
- A core interrupt or event service model

The core must be configured for response to system interrupts from the SEC. The SEC must be configured to enable and map the system interrupt source to the correct SCI and to forward interrupts to the connected core.

The system interrupt source must be configured to generate interrupt assertions. Alternatively, the processor can use software triggering for interrupt assertion. Software driven interrupts are generated by writing the source ID of the interrupt to be triggered to the SEC\_RAISE register.

## Programming Concepts

The following list provides the basic programming concepts necessary for configuring the SEC.

- Configuring an SSI as a system interrupt for a specific core.
- Configuring an SCI to provide system interrupts to the connected core (See Configuring a System Source to Interrupt a Core).
- Configuring an SSI as a system fault (See Configuring a System Source as a Fault).
- Configuring the SFI to manage system faults.

## Programming Examples

This section provides example programming tasks that are typical for SEC usage.

## Fault Management Interface Programming Model

The SFI interface can be programmed to manage fault events from system sources and associated actions such as issuing a system reset when watchdog expiration event occurs.

1. Set the SEC\_GCTL.EN bit to enable the SEC.
2. Write to the SEC\_FCTL register to configure specific fault actions.
- Trigger Output. Set the SEC\_FCTL.TOEN bit for the SEC to produce trigger outputs when a fault becomes active. The SEC\_FCTL.TES bit can be programmed to select the event that directs the SEC to assert trigger output when a fault is pending or active. Configure receivers for SEC fault trigger generator output.
4. NOTE: If the SEC\_FCTL.TOEN and or the SEC\_FCTL.TES bits =1 (T rigger Output Enabled and Trigger on Fault Pending), an external fault (if enabled by the SEC\_FCTL.FIEN bit) will not issue a trigger since Fault Pending is bypassed for external faults.
- System Reset. The Reset Control Unit (RCU) controls how the functional units enter and exit reset. Configure the RCU\_CTL.SRSTREQEN bit. This bit controls whether the sources of reset are enabled to perform a system reset. To issue a system reset request when a fault becomes active, set the SEC\_FCTL.SREN bit. The SEC fault system reset delay register ( SEC\_FSRDLY ) can be programmed for the delay, if required, from a fault becoming active to system reset request assertion.
- Fault Output. This configuration allows the SEC to indicate the fault status based on the SEC\_FCTL.CMS bit configuration.
- Computer Operating Normally (COP) mode. To configure fault output for COP mode, set the SEC\_FCTL.FOEN bit to enable fault output. Set the SEC\_FCTL.CMS bit to select COP mode to toggle the fault pin when no fault is active. Program the SEC\_FCOPP period register with a desired width value for the COP toggled output pin.

- Fault mode. Set the SEC\_FCTL.FOEN bit to enable fault output. The SEC\_FCTL.CMS bit should be set to Fault mode to toggle the fault pin when a fault is active.
3. If required, program the Fault Input to sample fault inputs from external devices on fault pins. Configure the SEC\_FCTL.FIEN bit to enable the SEC to sample a fault input from an external device.
- ADDITIONAL INFORMATION: The SEC\_FCTL.FIEN bit should be set only while the SEC\_FCTL.EN
- bit is low. If the SEC\_FCTL.EN bit is already high and the SEC\_FCTL.FIEN bit needs to be set, the SEC\_FCTL.EN bit should be cleared first. Fault input can only be enabled when Fault mode is selected by the SEC\_FCTL.CMS bit.
4. Program the required fault delay to the SEC\_FDLY.COUNT bit field if a delay between fault source assertion and the fault response is required.
5. Configure the SEC\_FCTL register to enable the SEC. ADDITIONAL INFORMATION: The SEC\_FCTL.EN bit should be set only while the SEC\_FSTAT.ACT bit is low.
6. Write to the control register of a specific source register using the SEC\_SCTL[n] register to enable the source as a fault.

## Configuring a System Source to Interrupt a Core

To configure a system source to interrupt a core, the SEC itself must be enabled with the source interface (SSI) and core interface (SCI) properly initialized. Specifically, the SCI must be set up to accept interrupt signaling from the SEC and pass them to the specified core, and the SSI must properly enable each of the peripheral interrupt sources to generate interrupt signals and optionally define a priority scheme that overrides the default priority settings. In summary:

1. Write to the SEC\_GCTL register to enable the SEC.
2. Write to the appropriate SCI SEC\_CCTL[n] register to enable SEC interrupts to be sent to that core.
3. Write to the appropriate SSI SEC\_SCTL[n] register to enable that peripheral as an interrupt source and to set the core target field to map the source to the desired SCI.
4. (Optional) By default, all the SEC interrupts are grouped as a single priority level, so passing of peripheral interrupt requests from the SEC is based solely on the default enumerated source ID. By programming the SEC\_CPLVL[n].PLVL register, interrupt sources can be grouped into priority levels within the SEC such that arbitration is first performed by source ID within a grouped priority level before proceeding to the next priority level, thus providing the flexibility to have lower-priority interrupt sources considered before higher-priority sources.

ADDITIONAL INFORMATION: The SEC\_CPMSK[n] and SEC\_CGMSK[n] registers must also can be programmed to mask the interrupts based on the customized levels and grouping.

## Core/SEC Handshake Requirements to Ensure Proper Interrupt Handling

A specific handshake with the SEC is required to handle interrupts associated with an individual core. The handshake ensures that nested interrupts are properly tracked and that new peripheral interrupts being raised within the SEC are either passed immediately to the core or held off and queued within the SEC for later servicing.

Use the following procedure to write a custom dispatcher inside the Interrupt Service Routine. Note that the core needs to read and acknowledge the SEC\_CSID[n] register by writing the same value. The core must also write to the SEC\_END register after the ISR execution completes.

1. Read the SEC\_CSID[n] register to obtain the source ID of the peripheral interrupt request.
2. Write the read value back to the SEC\_CSID[n] register to send the acknowledge signal to the SEC that the core has accepted and begun processing the interrupt request.
3. Execute the actual ISR (typically a call to a specific handler function from a look-up table based on the peripheral source ID). Write to the SEC\_GCTL register to enable the SEC.
4. Write the value of the SEC\_CSID[n] register of the active interrupt (read in step 1 above) to the SEC\_END register to signal to the SEC that the interrupt has now been serviced.
5. Return from interrupt.

This procedure allows a higher-priority interrupt raised by the SEC to be serviced by the core after step 2. The SEC knows what it passed to the core because of the write to the SEC\_CSID[n] register. After the core acknowledges that write, the SEC knows whether newly raised peripheral interrupts are a higher priority than the highest-priority interrupt currently being processed by the core.

- When a higher priority, the SEC pushes the current SEC\_CSID[n] value to an internal stack, writes the new SEC\_CSID[n] value, and asserts a new SEC interrupt request.
- When a lower priority, the SEC queues the interrupt until the core writes to the SEC\_END register with the source ID of the higher-priority interrupt, confirming that it was fully processed.

At this point the SEC\_CSID[n] value is popped from the internal stack and any pending peripheral interrupt requests are arbitrated before the SEC writes the new SEC\_CSID[n] value and asserts a new interrupt request. At the same time the core self-nests the latched SEC interrupt requests as needed. When a higher-priority interrupt is presented to the core the write to the SEC\_END register in the SEC handler epilog code guarantees that each nested level has the required handshake to signal to the SEC block that each individual source ID interrupt request is fully serviced. See the SHARC+ Core Programming Reference for more details about SEC handler code.

## Configuring a System Source as a Fault

Use the following procedure configure a system source as a fault.

1. Write to the SEC\_GCTL register to enable the SEC.
2. Write to the SEC\_FCTL register to configure specific fault actions.
3. Write to the SEC\_FDLY bit field to specify fault delay.

4. Write to the control register of a specific source to enable the source as a fault.

## Configuring the WDOG Expiry Event to Issue a System Reset

Use the following procedure to configure the WDOG timer to issue a system reset.

1. Configure the SEC\_GCTL register to enable the SEC.
2. Configure the SEC\_FCTL register to choose the Fault response mode. In the following code example, the system reset is issued.
3. Configure the SEC\_SCTL[n].SEN and SEC\_SCTL[n].FEN bits in the registers to determine how the fault source is handled. To configure the WDOG as the fault source, program the register. The program can configure any interrupt as the fault source by programming the corresponding register.

```
*pREG_SEC0_GCTL = BITM_SEC_GCTL_EN;
```

```
*pREG_RCU0_CTL |= BITM_RCU_CTL_SRSTREQEN; *pREG_SEC0_FCTL |= BITM_SEC_FCTL_SREN;
```

```
*pREG_SEC0_SCTL3 = BITM_SEC_SCTL_FEN|BITM_SEC_SCTL_SEN;
```

ADDITIONAL INFORMATION: The SEC ID corresponding to WDOG0 is 3, as indicated in the ADSP-2184x Interrupt List .

4. Write to the enable bit.

```
*pREG_SEC0_FCTL |= BITM_SEC_FCTL_EN;
```

## SEC Programming Restrictions

Setting the SEC\_FCTL.EN bit while the SEC\_FSTAT.ACT bit is high can result in unpredictable behavior. To avoid this issue, set the SEC\_FCTL.EN bit while the SEC\_FSTAT.ACT bit is low. The SEC\_FSTAT.ACT bit is only set when the SEC\_FCTL.EN bit is high. Therefore, the problem can only occur if the SEC\_FCTL.EN bit transitions from 1 to 0 and then to 1 again.

Writing to SEC\_FEND to end a fault with both the SEC\_FCTL.FOEN bit and the SEC\_FCTL.FIEN bit set can result in erroneous external fault detection. If this operation (ending a fault) and configuration (fault input and fault output enabled) are required by the application, clear the SEC\_FCTL.FOEN bit prior to writing to SEC\_FEND . The recommended sequence for ending a fault with the SEC\_FCTL.FIEN or SEC\_FCTL.FOEN ==1 is as follows:

1. Clear the SEC\_FCTL.FOEN bit. 2. Write to the SEC\_FEND register. 3. Set the SEC\_FCTL.FOEN bit.

## ADSP-2184x SEC Register Descriptions

System Event Controller (SEC) contains the following registers.

Table 7-6: ADSP-2184x SEC Register List

| Name           | Description                               |
|----------------|-------------------------------------------|
| SEC_CACT[n]    | SCI Active Register n                     |
| SEC_CCTL[n]    | SCI Control Register n                    |
| SEC_CGMSK[n]   | SCI Group Mask Register n                 |
| SEC_CPLVL[n]   | SCI Priority Level Register n             |
| SEC_CPMSK[n]   | SCI Priority Mask Register n              |
| SEC_CPND[n]    | Core Pending Register n                   |
| SEC_CSID[n]    | SCI Source ID Register n                  |
| SEC_CSTAT[n]   | SCI Status Register n                     |
| SEC_END        | Global End Register                       |
| SEC_FCOPP      | Fault COP Period Register                 |
| SEC_FCOPP_CUR  | Fault COP Period Current Register         |
| SEC_FCTL       | Fault Control Register                    |
| SEC_FDLY       | Fault Delay Register                      |
| SEC_FDLY_CUR   | Fault Delay Current Register              |
| SEC_FEND       | Fault End Register                        |
| SEC_FSID       | Fault Source ID Register                  |
| SEC_FSRDLY     | Fault System Reset Delay Register         |
| SEC_FSRDLY_CUR | Fault System Reset Delay Current Register |
| SEC_FSTAT      | Fault Status Register                     |
| SEC_GCTL       | Global Control Register                   |
| SEC_GSTAT      | Global Status Register                    |
| SEC_RAISE      | Global Raise Register                     |
| SEC_SCTL[n]    | Source Control Register n                 |
| SEC_SSTAT[n]   | Source Status Register n                  |

## SCI Active Register n

The SEC SCI active interrupt register ( SEC\_CACT[n] ) contains the source ID and priority of the highest priority active interrupt detected by the SEC prioritizer.

Figure 7-6: SEC\_CACT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000005_a7a7bfe6b527ab753d748c78959c0542544adf6da7ba664fce4fa7f8b942899f.png)

Table 7-7: SEC\_CACT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | PRIO       | Highest Active IRQ Priority. The SEC_CACT[n].PRIO indicates the priority value of the highest priority active interrupt for core n.   |
| 7:0 (R/NW)         | SID        | Highest Active IRQ Source ID. The SEC_CACT[n].SID identifies the source ID value of the highest priority active interrupt for core n. |

## SCI Control Register n

The SEC control register ( SEC\_CCTL[n] ) contains SCI control bits for all system sources.

Figure 7-7: SEC\_CCTL[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000006_e720f93dc50d573ae1b6f7600f5fc0a8ebe5f94371bbfda30950d6221e7fa665.png)

Table 7-8: SEC\_CCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CCTL[n].LOCK bit is enabled, the SEC_CCTL[n] register is read only.                                         |
| 16 (R/W)           | NMIEN      | NMI Enable. The SEC_CCTL[n].NMIEN bit controls NMI propagation to the core. When the SEC_CCTL[n].NMIEN bit is enabled, the SCI allows NMIs to propagate to the core for servicing. |
| 12 (R0/W)          | WFI        | Wait For Idle. When set, the SEC_CCTL[n].WFI bit forces the SCI to wait for indication of core idle before the SCI resumes activity. 0 No Action                                   |

Table 7-8: SEC\_CCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R0/W)           | RESET      | Reset. When set, the SEC_CCTL[n].RESET bit resets all SCI registers to their default values. 0 No Action                                                                                                                                                                                                                                                         |
| 0 (R/W)            | EN         | 1 Reset Enable. The SEC_CCTL[n].EN bit controls operation of the SCI. Clearing the SEC_CCTL[n].EN bit halts the execution of the SCI without resetting status reg- isters. (The INT signal to a core is not affected.) Setting the SEC_CCTL[n].EN bit enables the SCI to begin or resume operation with the current configuration and status. 0 Disable 1 Enable |

## SCI Group Mask Register n

The SEC SCI group mask register ( SEC\_CGMSK[n] ) contains selections for a group mask, an ungroup mask, and a register lock. This register contains the system interrupt group masks for the connected core. The core uses the SEC\_CGMSK[n].UGRP and SEC\_CGMSK[n].GRP fields to mask (disable) interrupts from the specified groups.

Figure 7-8: SEC\_CGMSK[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000007_0114d379c4b5bc7b3f594ed82840581f76028e379f976e0ba4d0aad8a693c575.png)

Table 7-9: SEC\_CGMSK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CGMSK[n].LOCK bit is enabled, the SEC_CGMSK[n] register is read only.                                                      | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CGMSK[n].LOCK bit is enabled, the SEC_CGMSK[n] register is read only.                                                      |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                 | Unlock                                                                                                                                                                                            |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                 | Lock                                                                                                                                                                                              |
| 8 (R/W)            | UGRP       | Ungrouped Mask. The SEC_CGMSK[n].UGRP bit masks interrupts (if set) for the ungrouped inter- rupt sources for core n.                                                                             | Ungrouped Mask. The SEC_CGMSK[n].UGRP bit masks interrupts (if set) for the ungrouped inter- rupt sources for core n.                                                                             |
| 8 (R/W)            | UGRP       | 0                                                                                                                                                                                                 | Unmask Ungrouped Sources                                                                                                                                                                          |
| 8 (R/W)            | UGRP       | 1                                                                                                                                                                                                 | Mask Ungrouped Sources                                                                                                                                                                            |
| 3:0 (R/W)          | GRP        | Grouped Mask. The SEC_CGMSK[n].GRP field selects a group of interrupt sources to mask for core n. (For more information about interrupt source groups, see the SEC_SCTL[n] register description.) | Grouped Mask. The SEC_CGMSK[n].GRP field selects a group of interrupt sources to mask for core n. (For more information about interrupt source groups, see the SEC_SCTL[n] register description.) |
| 3:0 (R/W)          | GRP        | 0                                                                                                                                                                                                 | No groups masked                                                                                                                                                                                  |
| 3:0 (R/W)          | GRP        | 1                                                                                                                                                                                                 | Mask group 0                                                                                                                                                                                      |
| 3:0 (R/W)          | GRP        | 2                                                                                                                                                                                                 | Mask group 1                                                                                                                                                                                      |
| 3:0 (R/W)          | GRP        | 3                                                                                                                                                                                                 | Mask groups 0, 1                                                                                                                                                                                  |

Table 7-9: SEC\_CGMSK[n] Register Fields (Continued)

| Bit No.   | Bit Name   |   Description/Enumeration |                        |
|-----------|------------|---------------------------|------------------------|
| (Access)  |            |                           |                        |
|           |            |                         4 | Mask group 2           |
|           |            |                         5 | Mask groups 0, 2       |
|           |            |                         6 | Mask groups 1, 2       |
|           |            |                         7 | Mask groups 0, 1, 2    |
|           |            |                         8 | Mask group 3           |
|           |            |                         9 | Mask groups 0, 3       |
|           |            |                        10 | Mask groups 1, 3       |
|           |            |                        11 | Mask groups 0, 1, 3    |
|           |            |                        12 | Mask groups 2, 3       |
|           |            |                        13 | Mask groups 0, 2, 3    |
|           |            |                        14 | Mask groups 1, 2, 3    |
|           |            |                        15 | Mask groups 0, 1, 2, 3 |

## SCI Priority Level Register n

The SEC SCI priority level register ( SEC\_CPLVL[n] ) contains selections for priority levels and a register lock. This register is used to divide the total number of priority levels into sub-levels. The sub-level priority resolution provides the tie breaker for simultaneously pending interrupts assigned to the same level.

Figure 7-9: SEC\_CPLVL[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000008_63dc82b5643ee01f38576046faebad875ba128266d18c5de31e3d52d0221d2c8.png)

Table 7-10: SEC\_CPLVL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPLVL[n].LOCK bit is enabled, the SEC_CPLVL[n] register is read only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPLVL[n].LOCK bit is enabled, the SEC_CPLVL[n] register is read only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Unlock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Lock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 2:0 (R/W)          | PLVL       | Priority Levels. The SEC_CPLVL[n].PLVL field serves to divide the total number of interrupt pri- ority levels into sub-levels. The sub-level priority resolution provides the tie breaker for simultaneously pending interrupts assigned to the same interrupt level. The sub-level priority value specifies the number of MSBs (minus 1) designated to interrupt levels, while the remaining LSBs are designated for sub-level specification. For example, if the SEC_CPLVL[n].PLVL field is set to two, the result is four priority levels are specified, because only the two MSBs are used for preemption evalu- ation. The remaining bits of the priority setting are used for sub-level prioritization. | Priority Levels. The SEC_CPLVL[n].PLVL field serves to divide the total number of interrupt pri- ority levels into sub-levels. The sub-level priority resolution provides the tie breaker for simultaneously pending interrupts assigned to the same interrupt level. The sub-level priority value specifies the number of MSBs (minus 1) designated to interrupt levels, while the remaining LSBs are designated for sub-level specification. For example, if the SEC_CPLVL[n].PLVL field is set to two, the result is four priority levels are specified, because only the two MSBs are used for preemption evalu- ation. The remaining bits of the priority setting are used for sub-level prioritization. |
| 2:0 (R/W)          | PLVL       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | 1 MSBs (2 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 2:0 (R/W)          | PLVL       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | 2 MSBs (4 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 2:0 (R/W)          | PLVL       | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | 3 MSBs (8 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 2:0 (R/W)          | PLVL       | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | 4 MSBs (16 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 2:0 (R/W)          | PLVL       | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | 5 MSBs (32 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 2:0 (R/W)          | PLVL       | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | 6 MSBs (64 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Table 7-10: SEC\_CPLVL[n] Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration        |
|-----------|------------|--------------------------------|
| (Access)  |            |                                |
|           |            | 6 7 MSBs (128 priority levels) |
|           |            | 7 8 MSBs (256 priority levels) |

## SCI Priority Mask Register n

The SEC SCI priority mask register ( SEC\_CPMSK[n] ) contains the SCI priority mask for core n and includes a register lock.

Figure 7-10: SEC\_CPMSK[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000009_9f5bb9f208c63647c71618d67088903bf6cfc27b9fc69b3c1990b60c770a82fe.png)

Table 7-11: SEC\_CPMSK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPMSK[n].LOCK bit is enabled, the SEC_CPMSK[n] register is read only. 0 Unlock                                                                               |
| 7:0 (R/W)          | PRIO       | IRQ Priority Mask. The SEC_CPMSK[n].PRIO contains the system interrupt priority mask for core n. The core uses the SEC_CPMSK[n].PRIO field to mask (block) interrupts below the specified level. 0 Priority level 0 (highest) 1-254 |

## Core Pending Register n

The SCI pending interrupt register ( SEC\_CPND[n] ) contains the source ID and priority of the highest priority pending interrupt detected by the SEC prioritizer.

Figure 7-11: SEC\_CPND[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000010_136f7df8e51caf62187c3ee832c8237f53cae3c2fd6f72580641dfc99c1e24b7.png)

Table 7-12: SEC\_CPND[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | PRIO       | Highest Pending IRQ Priority. The SEC_CPND[n].PRIO indicates the priority value of the highest priority pend- ing interrupt for core n. |
| 7:0 (R/NW)         | SID        | Highest Pending IRQ Source ID. The SEC_CPND[n].SID identifies the source ID value of the highest priority pending interrupt for core n. |

## SCI Source ID Register n

The SCI source ID register ( SEC\_CSID[n] ) contains the source ID of the interrupt last issued to core n. The SEC\_CSID[n] register value is loaded by the SCI when a system interrupt indication is sent to core n. The SCI does not change the SEC\_CSID[n] until after the interface receives an interrupt acknowledge from core n. Writing to the SEC\_CSID[n] register generates an interrupt acknowledge, but does not update the value in the register.

Figure 7-12: SEC\_CSID[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000011_88ac5e1878e85221121506d1adc0132a72917f9bac6474117731193ab8ac94e9.png)

Table 7-13: SEC\_CSID[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID.                                                                       |
| (R/NW)             |            | The SEC_CSID[n].SID bit is the source ID of the interrupt last issued to core n. |

## SCI Status Register n

The SCI status register ( SEC\_CSTAT[n] ) contains status bits, indicating the operational status of the SCI.

Figure 7-13: SEC\_CSTAT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000012_316a580488b643c1895cdefb0a2e43d85ff6664986049b7d0eedd436b2ba490d.png)

Table 7-14: SEC\_CSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W1C)         | NMI        | Non-Maskable Interrupt. The SEC_CSTAT[n].NMI bit indicates whether an NMI has occurred since the bit was last cleared.                                                                                                                                                                                                                                                                                                          |
| 12 (R/W1C)         | WFI        | Wait For Idle. The SEC_CSTAT[n].WFI bit indicates (if set) that the SCI is temporarily disabled, pending a core idle indication. This bit is set when SEC_CCTL[n].WFI is set.                                                                                                                                                                                                                                                   |
| 10 (R/NW)          | SIDV       | SID Valid. The SEC_CSTAT[n].SIDV bit indicates (if set) that the current value in the SEC_CSID[n] register is valid. The SCI sets the SEC_CSTAT[n].SIDV bit when the updating the SEC_CSID[n] register with a new value. The SEC_CSTAT[n].SIDV bit is cleared when the SEC_CSID[n] register is written. This status indication may be used to extract all pending interrupts in a single inter- rupt service routine. 0 Invalid |
| 10 (R/NW)          |            | 1 Valid                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 10 (R/NW)          |            |                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 10 (R/NW)          |            |                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 7-14: SEC\_CSTAT[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/NW)           | ACTV       | ACT Valid. The SEC_CSTAT[n].ACTV bit indicates (if set) that the current value in the SEC_CACT[n] register is valid. The SCI sets the SEC_CSTAT[n].ACTV bit when updating the SEC_CACT[n] registers with a new value. The SEC_CSTAT[n].ACTV bit is cleared when the SEC_CSID[n] register is written. |
| 9 (R/NW)           | ACTV       | 0 Invalid                                                                                                                                                                                                                                                                                            |
| 8 (R/NW)           | PNDV       | PND Valid. The SEC_CSTAT[n].PNDV bit indicates (if set) that the current value in the SEC_CPND[n] register is valid. The SCI sets the SEC_CSTAT[n].PNDV bit when updating the SEC_CPND[n] register with a new value. The SEC_CSTAT[n].PNDV bit is cleared when the SEC_CSID[n] register is written.  |
| 8 (R/NW)           | PNDV       | 0 Invalid                                                                                                                                                                                                                                                                                            |
| 5:4 (R/NW)         | ERRC       | Error Cause. The SEC_CSTAT[n].ERRC bits are updated on assertion of the SEC_CSTAT[n].ERR bit to indicate the SCI error type. SEC_CSTAT[n].ERRC is only updated on the assertion of SEC_CSTAT[n].ERR . Subsequent errors while SEC_CSTAT[n].ERR is asserted do not update SEC_CSTAT[n].ERRC .         |
| 5:4 (R/NW)         | ERRC       | 0 Reserved                                                                                                                                                                                                                                                                                           |
| 5:4 (R/NW)         | ERRC       | 1 Acknowledge Error. SCI has received an acknowledge without a pending, unacknowledged interrupt present.                                                                                                                                                                                            |
| 5:4 (R/NW)         | ERRC       | 2 Reserved                                                                                                                                                                                                                                                                                           |
| 5:4 (R/NW)         | ERRC       | 3 Reserved                                                                                                                                                                                                                                                                                           |
| 1 (R/W1C)          | ERR        | Error. The SEC_CSTAT[n].ERR bit indicates that an error has occurred in the SCI. When SEC_CSTAT[n].ERR is set, the SCI updates the SEC_CSTAT[n].ERRC field to the value of the corresponding error cause.                                                                                            |
| 1 (R/W1C)          | ERR        | 0 No Error                                                                                                                                                                                                                                                                                           |
| 1 (R/W1C)          | ERR        | 1 Error Occurred                                                                                                                                                                                                                                                                                     |

## Global End Register

The SEC global end register ( SEC\_END ) contains a source ID interrupt service end field ( SEC\_END.SID ). When a core has finished servicing an interrupt, the core writes the SEC\_END.SID field in the SEC\_END register. This write causes the SEC to clear the SEC\_SSTAT[n].ACT bit in the SEC\_SSTAT[n] register of the corresponding interrupt.

Figure 7-14: SEC\_END Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000013_141cd44c1908ee32e17e16eb2c1b6e306753f2eb2618e428adfb3b04cb6b362a.png)

Table 7-15: SEC\_END Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID IRQ to End.                                                         |
| (R/W)              |            | The SEC_END.SID bit field contains the source ID interrupt service end value. |

## Fault COP Period Register

The SEC fault COP period register ( SEC\_FCOPP ) contains the width value (count in (SEC) clock cycles) for the high and low phase of the computer operating properly (COP) toggled output on the COP pin. Note that the actual high/low phase value is the SEC\_FCOPP.COUNT programmed value plus 1.

Figure 7-15: SEC\_FCOPP Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000014_35d33a0c7a002829376afcb7582fc92946459f528c5abb8d290570fdb2783912.png)

Table 7-16: SEC\_FCOPP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | COUNT      | Fault COP Period. The SEC_FCOPP.COUNT bit field is the width value for the high and low phase of the computer operating properly (COP) toggled output on the COP pin. |
| (R/W)              |            |                                                                                                                                                                       |

## Fault COP Period Current Register

The SEC fault COP period current register ( SEC\_FCOPP\_CUR ) contains the active count (in (SEC) clock periods) for the current phase (high or low) of the computer operating properly (COP) toggled output on the COP pin. The SEC loads the SEC\_FCOPP\_CUR register from the SEC\_FCOPP register when the SEC\_FCOPP\_CUR.COUNT field is cleared and the SEC is in COP mode ( SEC\_FCTL.CMS bit =1). The SEC decrements the SEC\_FCOPP\_CUR count each (SEC) clock cycle while SEC\_FCTL.CMS is set and the SEC\_FSTAT.ACT bit is not set.

Figure 7-16: SEC\_FCOPP\_CUR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000015_c8d6e9158671bc232293f0655a781602dcbb84b554c1f43355bf707de9054851.png)

Table 7-17: SEC\_FCOPP\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault COP Period. The SEC_FCOPP_CUR.COUNT bit field is the active count for the current phase (high or low) of the computer operating properly (COP) toggled output on the COP pin. |

## Fault Control Register

The SEC fault control register ( SEC\_FCTL ) contains fault control bits for all SEC channels. This register controls the operation of the System Fault Management Interface (SFI).

Figure 7-17: SEC\_FCTL Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000016_56a7f9da57b6794a661cd1591c24d8668c033171092322c89e6c2e57543730d3.png)

Table 7-18: SEC\_FCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_FCTL.LOCK bit is enabled, the SEC_FCTL register is read only. 0 UnLock                                                                                                                                                                    |
| 13 (R/W)           | TES        | Trigger Event Select. The SEC_FCTL.TES bit selects the event that directs the SEC to assert trigger output. In fault pending mode, the SEC asserts trigger output when a fault is pending. In fault active mode, the SEC asserts trigger output when a fault is active. 0 Fault Active Mode 1 Fault Pending Mode |

Table 7-18: SEC\_FCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | CMS        | COP Mode Select. The SEC_FCTL.CMS selects the SEC mode for handling fault input. In COP mode, the SEC toggles the COP pin to indicate that no fault is active and ceases toggling the pin to indicate that a fault is active. In fault mode, the SEC deasserts the fault pin (=0) and fault_b pin (=1) when no fault is active and asserts the fault pin (=1) and fault_b pin (=0) when a fault is active. Not all processors feature both the fault and fault_b pins. Refer to the product data sheet for details. |
| 12 (R/W)           | CMS        | 0 Fault Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 12 (R/W)           | CMS        | 1 COP Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W)            | FIEN       | Fault Input Enable. The SEC_FCTL.FIEN bit enables the SEC to sample fault input. If SEC_FCTL.FIEN is set (=1), a fault indication from an external device sets the SEC_FSTAT.ACT bit and SEC_FSID.FEXT bit.                                                                                                                                                                                                                                                                                                         |
| 7 (R/W)            | FIEN       | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 7 (R/W)            | FIEN       | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 6 (R/W)            | SREN       | System Reset Enable. The SEC_FCTL.SREN bit enables the SEC to issue a system reset request when a fault becomes active.                                                                                                                                                                                                                                                                                                                                                                                             |
| 6 (R/W)            | SREN       | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | SREN       | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 5 (R/W)            | TOEN       | Trigger Output Enable. The SEC_FCTL.TOEN bit enables the SEC to produce trigger output when a fault becomes active.                                                                                                                                                                                                                                                                                                                                                                                                 |
| 5 (R/W)            | TOEN       | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W)            | TOEN       | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 4 (R/W)            | FOEN       | Fault Output Enable. The SEC_FCTL.FOEN bit enables the SEC to indicate fault status, according to the SEC_FCTL.CMS bit configuration. Disable                                                                                                                                                                                                                                                                                                                                                                       |
| 4 (R/W)            | FOEN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 4 (R/W)            | FOEN       | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 1 (R0/W)           | RESET      | Reset.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 1 (R0/W)           | RESET      | 0 No Action                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 1 (R0/W)           | RESET      | 1 Reset                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 7-18: SEC\_FCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Enable. The SEC_FCTL.EN bit controls the operational state of the SEC. Clearing the SEC_FCTL.EN bit halts the execution of the SEC without resetting status registers. Setting the SEC_FCTL.EN bit enables the SEC to begin or resume operation with the current configuration and status. |
| 0 (R/W)            | EN         | 0 Disable                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | EN         | 1 Enable                                                                                                                                                                                                                                                                                   |

## Fault Delay Register

The SEC fault delay register ( SEC\_FDLY ) contains the number ( SEC\_FDLY.COUNT field) of (SEC) clock periods to delay from fault pending to fault active, when actions are enabled.

Figure 7-18: SEC\_FDLY Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000017_9fb7c91472143c2c5838b7e4251b41d1a94817290e182fdfdbd0757940c7a5b6.png)

Table 7-19: SEC\_FDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | COUNT      | Fault Delay. The SEC_FDLY.COUNT bit field is the number of (SEC) clock periods to delay from fault pending to fault active, when actions are enabled. |

## Fault Delay Current Register

The SEC fault delay current register ( SEC\_FDLY\_CUR ) contains the active count ( SEC\_FDLY\_CUR.COUNT field) in (SEC) clock periods for the delay from fault pending to fault active, when actions are enabled. The count is loaded from the SEC\_FDLY register when a fault becomes pending ( SEC\_FSTAT.PND bit is set). The SEC decrements the value in SEC\_FDLY\_CUR each (SEC) clock cycle while the SEC\_FSTAT.PND bit is set.

Figure 7-19: SEC\_FDLY\_CUR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000018_9e346c5bc5939bc311b755564a72895fe40b8da59849d6687474e19eee358645.png)

Table 7-20: SEC\_FDLY\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | COUNT      | Fault Delay. The SEC_FDLY_CUR.COUNT bit field is the active count in (SEC) clock periods for the delay from fault pending to fault active, when actions are enabled. |
| (R/NW)             |            |                                                                                                                                                                      |

## Fault End Register

The SEC fault end register ( SEC\_FEND ) contains fault source ID and internal/external fields. This register receives fault end indication from a core.

Figure 7-20: SEC\_FEND Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000019_1a216902bcbf989f880308b73f3b603dd8f681187265dfff0f1ff091212d8fc2.png)

Table 7-21: SEC\_FEND Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | FEXT       | Fault External. Setting the SEC_FEND.FEXT bit, when the SEC_FEND.SID field is cleared, clears an active fault from an external source. 0 Fault Internal 1 Fault External                                                                                        |
| 7:0 (R/W)          | SID        | Source ID. The SEC_FEND.SID identifies a fault to be ended as indicated to the SEC by the core. The core loads the SEC_FEND.SID field value. If the SEC_FEND.SID value matches the SEC_FSID.SID value, the SEC_FSTAT.PND bit and SEC_FSTAT.ACT bit are cleared. |

## Fault Source ID Register

The SEC fault source ID register ( SEC\_FSID ) contains a fault source ID and internal/external fields.

NOTE:These bits are not reset by system reset so that a fault that automatically triggers a system reset to avoid a fault may be analyzed after the reset occurs.

Figure 7-21: SEC\_FSID Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000020_483d4943b9d98d76c81b2941ebbc0aababc3eeebb20fff389c14b8d95ff8e32f.png)

Table 7-22: SEC\_FSID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/NW)          | FEXT       | Fault External. The SEC_FSID.FEXT bit indicates that the last active fault was asserted by an exter- nal device. The SEC sets the SEC_FSID.FEXT bit when the SEC_FSTAT.ACT bit is set by the fault input pins. The SEC_FSID.FEXT bit is cleared when the SEC_FSTAT.ACT bit is set by an internal fault or when the external fault is ended. When the SEC_FSID.FEXT bit is set, the SEC_FSID.SID is cleared. 0 Fault Internal |
| 7:0 (R/NW)         | SID        | Source ID. The SEC_FSID.SID identifies the fault assertion detected by the SEC fault inter- face. The SEC loads the SEC_FSID.SID field value when a system fault indication is asserted. The SEC fault interface does not change the SEC_FSID.SID value until the fault is no longer pending or active, as indicated by the SEC_FSTAT.PND bit and SEC_FSTAT.ACT bit being cleared in the SEC_FSTAT register.                 |

## Fault System Reset Delay Register

The SEC fault system reset delay register ( SEC\_FSRDLY ) contains the number ( SEC\_FSRDLY.COUNT field) of (SEC) clock periods for the delay from a fault becoming active to system reset request assertion, if enabled.

Figure 7-22: SEC\_FSRDLY Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000021_a42fe928c7f12e830de9868a0365efa00723ceb2ef3866b1270347decd53b753.png)

Table 7-23: SEC\_FSRDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | COUNT      | Fault System Reset Delay. The SEC_FSRDLY.COUNT bit field is the number of (SEC) clock periods for the delay from a fault becoming active to system reset request assertion. |

## Fault System Reset Delay Current Register

The SEC fault system reset delay current register ( SEC\_FSRDLY\_CUR ) contains the active count ( SEC\_FSRDLY\_CUR.COUNT field) in (SEC) clock periods for the delay from fault active to system reset assertion, if enabled. The count is loaded from the SEC\_FSRDLY register when a fault becomes active ( SEC\_FSTAT.ACT bit is set). The SEC decrements the value in SEC\_FSRDLY\_CUR each (SEC) clock cycle while the SEC\_FSTAT.ACT bit is set.

Figure 7-23: SEC\_FSRDLY\_CUR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000022_12a0c79c10db8f8b552dff70404916630f678deddc78ae9ea0356a202e10e437.png)

Table 7-24: SEC\_FSRDLY\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | COUNT      | Fault System Reset Delay.                                                                                                                |
| (R/NW)             |            | The SEC_FSRDLY_CUR.COUNT bit field is the active count in (SEC) clock periods for the delay from fault active to system reset assertion. |

## Fault Status Register

The SEC fault status register ( SEC\_FSTAT ) indicates the operational status of the SFI.

Figure 7-24: SEC\_FSTAT Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000023_960d5330c33ccc22a3de06a324f8c49c6d804b7dee53163463e46944d6a00435.png)

Table 7-25: SEC\_FSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/NW)          | NPND       | Next Pending Fault. The SEC_FSTAT.NPND bit indicates that one or more sources have signaled fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC_FSTAT.NPND bit when the fault interface de- tects assertion of any enabled fault source input, while either the SEC_FSTAT.PND or SEC_FSTAT.ACT bits are set. The SEC clears the SEC_FSTAT.NPND bit when there are no fault sources waiting. | Next Pending Fault. The SEC_FSTAT.NPND bit indicates that one or more sources have signaled fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC_FSTAT.NPND bit when the fault interface de- tects assertion of any enabled fault source input, while either the SEC_FSTAT.PND or SEC_FSTAT.ACT bits are set. The SEC clears the SEC_FSTAT.NPND bit when there are no fault sources waiting. |
| 9 (R/NW)           | ACT        | Fault Active. The SEC_FSTAT.ACT bit indicates that the SEC has received a fault source input, the current fault delay count (in the SEC_FDLY_CUR register) has expired, and the fault actions are enabled. The SEC also sets the SEC_FSTAT.ACT bit on fault input detection if the SEC_FCTL.FIEN bit is set. The SEC_FSTAT.ACT bit is cleared by writing the ID value of the asserted fault from SEC_FSID register to the SEC_FEND register.                              | Fault Active. The SEC_FSTAT.ACT bit indicates that the SEC has received a fault source input, the current fault delay count (in the SEC_FDLY_CUR register) has expired, and the fault actions are enabled. The SEC also sets the SEC_FSTAT.ACT bit on fault input detection if the SEC_FCTL.FIEN bit is set. The SEC_FSTAT.ACT bit is cleared by writing the ID value of the asserted fault from SEC_FSID register to the SEC_FEND register.                              |
| 9 (R/NW)           | ACT        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | No Fault                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 9 (R/NW)           | ACT        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Active Fault                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 7-25: SEC\_FSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | PND        | Pending Fault. The SEC_FSTAT.PND bit indicates a fault source has signaled a fault assertion to the SEC, but the SEC has not yet triggered the event actions due to the delay selected with the SEC_FDLY register. The SEC fault interface sets the SEC_FSTAT.PND bit when the SEC_FSID is updated on assertion of a fault source input. The SEC_FSTAT.PND bit is only set when the SEC_FSTAT.ACT bit is cleared. The SEC updates the SEC_FSID register with the SID value when the SEC_FSTAT.PND bit is set. The SEC_FSTAT.PND bit is cleared either by the SEC fault interface when the current delay count in the SEC_FDLY_CUR register expires or by writing the SEC_FSID.SID field value (which indicates the ID of the asserted fault) to the SEC_FEND register. 0 Not Pending |
| 5:4 (R/NW)         | ERRC       | Error Cause. When the SEC_FSTAT.ERR bit is asserted, the SEC updates SEC_FSTAT.ERRC field to convey the interrupt source error type. When the error type is source overflow, the status indicates that a source signal assertion occurred or an SEC raise operation was attempted while pending was already set. The source overflow is detected when the source is set for edge only. When the error type is end error, the status indicates that an end was received for a source while the SEC_FSTAT.ACT bit was not set.                                                                                                                                                                                                                                                         |
| 5:4 (R/NW)         | ERRC       | 0 Source Overflow Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 5:4 (R/NW)         | ERRC       | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 5:4 (R/NW)         | ERRC       | 2 End Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 5:4 (R/NW)         | ERRC       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 1 (R/W1C)          | ERR        | Error. The SEC_FSTAT.ERR bit indicates an SEC fault interface error. When SEC_FSTAT.ERR is set, the SEC updates the SEC_FSTAT.ERRC field to indicate the corresponding error cause. When multiple errors occur, the SEC_FSTAT register captures the status for the first error and does not capture subsequent errors until the                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1 (R/W1C)          | ERR        | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 1 (R/W1C)          | ERR        | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

## Global Control Register

The SEC global control register ( SEC\_GCTL ) provides register locking, reset, and enable for the SEC module.

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000024_7eafd75b74043b0276aac2359f6df66b434a0a5d78acdc1b49cecf035527a5c2.png)

Lock

Figure 7-25: SEC\_GCTL Register Diagram

Table 7-26: SEC\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_GCTL.LOCK bit is enabled, the SEC_GCTL register is read only.                                                                                                                                                                                                      |
| 1 (R0/W)           | RESET      | Reset. The SEC_GCTL.RESET bit is write-1-action and triggers a soft reset to all SEC registers.                                                                                                                                                                                                                                           |
| 0 (R/W)            | EN         | Enable. The SEC_GCTL.EN bit is read/write and must be set for the SEC to begin/re- sume SEC operation with the current configuration and status. Clearing the SEC_GCTL.EN bit halts the execution of the SFI and all SCIs. All SSIs remain active, along with all error detection, without resetting status registers. 0 Disable 1 Enable |

## Global Status Register

The SEC global status register ( SEC\_GSTAT ) contains global status bits for the SEC.

Figure 7-26: SEC\_GSTAT Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000025_dbe2252b1816b24a2866271912f76132344fd71fba64125861f933b34cf69b02.png)

Table 7-27: SEC\_GSTAT Register Fields

| Bit No. (Access)   | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | Lock Write Error. The SEC_GSTAT.LWERR bit indicates (when set) there was an attempted write to an SEC register while the SEC_GCTL.LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 30 (R/W1C)         | Address Error. The SEC_GSTAT.ADRERR bit indicates that the SEC generated and address error. This status bit is sticky; write-1-to-clear it.                                                                                                                                     |
| 23:16 (R/NW)       | Source ID for SSI Error. The SEC_GSTAT.SID bits indicate the source ID that generated the last SSI error conveyed in the SEC_GSTAT.ERRC field.                                                                                                                                  |
| 11:8 (R/NW)        | SCI ID for SCI Error. The SEC_GSTAT.SCI bits indicate the number for the specific SCI that generated the last SCI error conveyed in the SEC_GSTAT.ERRC field.                                                                                                                   |

Table 7-27: SEC\_GSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:4                | ERRC       | Error Cause. When the SEC updates the SEC_GSTAT.ERR bit, the SEC updates the SEC_GSTAT.ERRC bits to indicate the error type. Note that for SCI errors, the error status represents an OR of all the errors from each                                                                                                                       | Error Cause. When the SEC updates the SEC_GSTAT.ERR bit, the SEC updates the SEC_GSTAT.ERRC bits to indicate the error type. Note that for SCI errors, the error status represents an OR of all the errors from each                                                                                                                       |
| 1 (R/W1C)          | ERR        | Error. The SEC_GSTAT.ERR bit indicates an error has occurred in the SEC. When the SEC asserts this bit (=1), the SEC updates the SEC_GSTAT.ERRC field to indicate the corresponding error cause. Even if multiple errors occur, only the first error is captured on assertion of this bit. This status bit is sticky; write-1-to-clear it. | Error. The SEC_GSTAT.ERR bit indicates an error has occurred in the SEC. When the SEC asserts this bit (=1), the SEC updates the SEC_GSTAT.ERRC field to indicate the corresponding error cause. Even if multiple errors occur, only the first error is captured on assertion of this bit. This status bit is sticky; write-1-to-clear it. |
|                    |            | 0 1 3 0                                                                                                                                                                                                                                                                                                                                    | SFI Error SCI Error Reserved                                                                                                                                                                                                                                                                                                               |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                          | SSI Error                                                                                                                                                                                                                                                                                                                                  |
|                    |            |                                                                                                                                                                                                                                                                                                                                            | No Error                                                                                                                                                                                                                                                                                                                                   |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                          | Error Occurred                                                                                                                                                                                                                                                                                                                             |

## Global Raise Register

The SEC global raise register ( SEC\_RAISE ) contains a source ID event set-to-pending field ( SEC\_RAISE.SID ). When a source ID value is written to this field, the SEC raises the source's event status to pending.

Figure 7-27: SEC\_RAISE Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000026_a1aaf41e1278d0bdb29011fade0cbd1cd482252fee0731f8032c72820e78bbd9.png)

Table 7-28: SEC\_RAISE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID.                                                                           |
| (R/W)              |            | The SEC_RAISE.SID bit field is the source ID of event that is set to pending status. |

## Source Control Register n

The SEC source control register ( SEC\_SCTL[n] ) contains control bits to configure the SEC event sources. This register controls the configuration of the corresponding SEC event source.

Figure 7-28: SEC\_SCTL[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000027_4d30eafa8292bdd96b84ce384c6fcfaf1cafb0f3f56334c609a7710cbe313e84.png)

Table 7-29: SEC\_SCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_SCTL[n].LOCK bit is enabled, the SEC_SCTL[n] register is read only. 0 Unlock 1 Lock                                                                                                         |
| 27:24 (R/W)        | CTG        | Core Target Select. The SEC_SCTL[n].CTG bits selects the specific SEC core interface to which the interrupt is mapped. Each system interrupt is mapped uniquely to one specific SEC core interface and (as a result) to a specific core. 1 CORE1 SHARC0 2 Reserved |

Table 7-29: SEC\_SCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | GRP        | Group Select. The SEC_SCTL[n].GRP bits each select a specific group for the interrupt. Each                                                                                                                | Group Select. The SEC_SCTL[n].GRP bits each select a specific group for the interrupt. Each                                                                                                                |
| 15:8 (R/W)         | PRIO       | grouped". Priority Level Select. The SEC_SCTL[n].PRIO bits sets the relative priority for an interrupt request. A pending interrupt request forwards its SEC_SCTL[n].PRIO value to the SEC core interface. | grouped". Priority Level Select. The SEC_SCTL[n].PRIO bits sets the relative priority for an interrupt request. A pending interrupt request forwards its SEC_SCTL[n].PRIO value to the SEC core interface. |
| 4 (R/W)            | ERREN      | Error Enable. The SEC_SCTL[n].ERREN bit permits the SEC_SSTAT[n].ERR status bit to be set on error detection. If SEC_SCTL[n].ERREN is cleared, no errors are detect- ed.                                   | Error Enable. The SEC_SCTL[n].ERREN bit permits the SEC_SSTAT[n].ERR status bit to be set on error detection. If SEC_SCTL[n].ERREN is cleared, no errors are detect- ed.                                   |
| 4 (R/W)            | ERREN      | 0                                                                                                                                                                                                          | Disable                                                                                                                                                                                                    |
| 4 (R/W)            | ERREN      | 1                                                                                                                                                                                                          | Enable                                                                                                                                                                                                     |
| 3 (R/W)            | ES         | Edge Select. The SEC_SCTL[n].ES bit selects the operational and sensitivity mode of the SEC                                                                                                                | Edge Select. The SEC_SCTL[n].ES bit selects the operational and sensitivity mode of the SEC                                                                                                                |
| 3 (R/W)            | ES         | 0                                                                                                                                                                                                          | Level Sensitive                                                                                                                                                                                            |
| 3 (R/W)            | ES         | 1                                                                                                                                                                                                          | Edge Sensitive                                                                                                                                                                                             |
| 2 (R/W)            | SEN        | Source (signal) Enable. The SEC_SCTL[n].SEN bit controls whether the system event source input signal may affect the pending status of the source. Clearing the SEC_SCTL[n].SEN                            | Source (signal) Enable. The SEC_SCTL[n].SEN bit controls whether the system event source input signal may affect the pending status of the source. Clearing the SEC_SCTL[n].SEN                            |
| 2 (R/W)            | SEN        | 0                                                                                                                                                                                                          | Disable                                                                                                                                                                                                    |
| 2 (R/W)            | SEN        | 1                                                                                                                                                                                                          | Enable                                                                                                                                                                                                     |

Table 7-29: SEC\_SCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | FEN        | Fault Enable. The SEC_SCTL[n].FEN bit controls whether the SEC may forward an interrupt request to the SEC fault interface as a fault source. This bit does not affect the ability of an interrupt source to set an interrupt as pending. The SEC_SCTL[n].FEN bit only affects whether the pending request may be forwarded to the SEC fault interface. | Fault Enable. The SEC_SCTL[n].FEN bit controls whether the SEC may forward an interrupt request to the SEC fault interface as a fault source. This bit does not affect the ability of an interrupt source to set an interrupt as pending. The SEC_SCTL[n].FEN bit only affects whether the pending request may be forwarded to the SEC fault interface. |
| 1 (R/W)            | FEN        | 0                                                                                                                                                                                                                                                                                                                                                       | Disable                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | IEN        | Interrupt Enable. The SEC_SCTL[n].IEN bit controls whether the SEC may forward an interrupt request to a core for servicing. This bit does not affect the ability of an interrupt source to set an interrupt as pending.                                                                                                                                | Interrupt Enable. The SEC_SCTL[n].IEN bit controls whether the SEC may forward an interrupt request to a core for servicing. This bit does not affect the ability of an interrupt source to set an interrupt as pending.                                                                                                                                |
| 0 (R/W)            | IEN        | 0                                                                                                                                                                                                                                                                                                                                                       | Disable                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | IEN        | 1                                                                                                                                                                                                                                                                                                                                                       | Enable                                                                                                                                                                                                                                                                                                                                                  |

## Source Status Register n

The SEC event source status register ( SEC\_SSTAT[n] ) contains bits indicating the status of the corresponding event source n. An event source may be: pending, active, active and pending, or neither pending nor active.

Figure 7-29: SEC\_SSTAT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000028_3f8656aeb24de76af3e9a4100e0ac18e2d5e959269939bce4f96969f7f0fe5cb.png)

Table 7-30: SEC\_SSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/NW)       | CHID       | Channel ID. The SEC_SSTAT[n].CHID bits indicate the ID of the specific source (from a set of sources sharing one SEC source interface input) that asserted the SEC source interface input. An SEC source interface input may support multiple system sources, in which case the assertion must be qualified by an identifier to determine the channel that generated the assertion. The SEC_SSTAT[n].CHID field provides this value in the form of a numeric reference that is mapped to a specific interrupt source. The prioritization for simultaneously asserted sources is according to ID, with 0 being the highest priority. The SEC_SSTAT[n].CHID is captured when the SEC source interface input is acknowledged. |
| 9 (R/W1C)          | ACT        | Active Source. The SEC_SSTAT[n].ACT bit indicates the source has been accepted by a core for servicing, but the service is not yet complete. An SEC_SSTAT[n].ACT bit is set by the SEC when the specific system interrupt is acknowledged by the core through the SEC core interface. An SEC_SSTAT[n].ACT bit is cleared by the SEC when the core provides interrupt service end indication for the specific system interrupt through the SEC core interface. Active                                                                                                                                                                                                                                                       |
| 9 (R/W1C)          | ACT        | 0 Not                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 9 (R/W1C)          | ACT        | 1 Active                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 7-30: SEC\_SSTAT[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W1C)          | PND        | Pending Source. The SEC_SSTAT[n].PND bit indicates the source has signaled an event re- quest, but the event request has not been (or is not currently being) serviced. A SEC_SSTAT[n].PND bit is set by the SEC on detection of an assertion of the corresponding system source input. A SEC_SSTAT[n].PND bit is cleared by the SEC when the specific system event is acknowledged by the core through the SEC core interface or by a W1C operation. 0 Not Pending                                           |
| 5:4 (R/NW)         | ERRC       | 1 Pending Error Cause. When the SEC_SSTAT[n].ERR bit is asserted, the SEC updates SEC_SSTAT[n].ERRC field to convey the interrupt source error type. When the error type is source overflow, the status indicates that a source signal assertion occurred or an SEC raise operation was attempted while pending was already set. The source overflow is detected when the source is set for edge only. When the error type is end error, the status indicates that an end was received for a source while the |
| 5:4 (R/NW)         |            | 0 Source Overflow Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 5:4 (R/NW)         |            | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 5:4 (R/NW)         |            | 2 End Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 1 (R/W1C)          | ERR        | Error. The SEC_SSTAT[n].ERR bit indicates an error for a specific system inter- rupt source. When the SEC_SSTAT[n].ERR bit is set, the SEC updates the SEC_SSTAT[n].ERRC field to the value of the corresponding error cause. Even if multiple errors occur, only the first error is captured on assertion of the SEC_SSTAT[n].ERR bit. 0 No Error                                                                                                                                                            |
| 1 (R/W1C)          |            | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 1 (R/W1C)          |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

## GIC Overview

The generic interrupt controller (GIC) provides an interface to the dual Cortex A55 core in the processor and collects up to 419 interrupt requests from all processor system sources. In addition, the GIC also supports sixteen software generated interrupts and nine private peripheral interrupts that are internal to the Cortex A55 core and not connected to the SECs. All GIC interrupts are also connected to the SEC in the same order.

Each interrupt can be configured as a normal or a secure interrupt. Software force registers and software priority masking are also supported. The 'Register Descriptions' section in this chapter provides brief descriptions of these Arm-based registers. For complete information refer to the Arm® Generic Interrupt Controller Architecture version 3.0 and 4.0 Architecture Specification .

## GIC Functional Description

The GIC interrupt controller consists of three interfaces;

- Distributor interface
- Redistributor interface
- CPU interface

The distributor and the redistributor interfaces are used to configure interrupts; the CPU interface is used to handle interrupts.

## GIC Distributer Interface

The distributor registers are memory-mapped. They are used to configures SPIs. The distributor block provides a programming interface to perform the following tasks.

- Interrupt prioritization and distribution of SPIs
- Enable and disable SPIs
- Set the priority level of each SPI
- Set each SPI to be level-sensitive or edge-triggered
- Assign each SPI to an interrupt group
- Control the active and pending state of SPIs.

## GIC Redistributer Interface

There is one redistributor per connected core. The redistributors provide a programming interface to perform the following tasks.

- Enable and disable SGIs and PPIs
- Set the priority level of SGIs and PPIs
- Set each PPI to be level-sensitive or edge-triggered
- Assign each SGI and PPI to an interrupt group
- Control the state of SGIs and PPIs.

## GIC CPU Interface

Each core contains a CPU interface that includes system registers that are used during interrupt handling. The CPU interfaces provide a programming interface to perform the following tasks.

- Provide general control and configuration to enable interrupt handling
- Acknowledge an interrupt
- Perform a priority drop and deactivation of interrupts
- Set an interrupt priority mask for the processor
- Define the preemption policy for the processor
- Determine the highest priority pending interrupt for the processor

In Arm CoreLink GICv3, the CPU interface registers are accessed as system registers: ICC\_*\_ELn, where 'n' specifies the Exception level: EL1-EL3.

Refer to the GIC Register Descriptions in this chapter for details on the distributor and the redistributor interface registers. Refer to the Arm® Generic Interrupt Controller Architecture Specification GIC architecture version 3.0 and 4.0 for details on the CPU interface registers.

## GIC Block Diagram

The GIC Block Diagram shows the event management architecture.

Figure 7-30: GIC Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000029_e60b89bede756c911e8906ea29f1301b629246c99c28dfdf92a746a9ae6baf09.png)

## GIC Performance Monitoring Unit (GIC PMU)

The GIC contains a performance monitoring unit (PMU) for counting key GIC events from the distributor. Redistributor events are not tracked by the PMU. The delivery of PPI and SGI interrupts are counted by recording calls to the core interrupt service routine.

The PMU has five counters with snapshot capability and overflow interrupt. For all the GIC events, PMU count configuration and register information, please refer to the Arm® CoreLink™ GIC 600 Generic Interrupt Controller Technical Reference Manual .

## Overflow interrupt

The overflow interrupt is enabled on a per counter basis by enabling the relevant bit of the register. Similarly, the overflow interrupt enable is disabled by corresponding writes to the register. When enabled, the interrupt activates when there is a write to the register for any counter or overflow on any enabled counter.

## Snapshot

Each PMU counter, , has a corresponding snapshot register. On a snapshot event, all five counters are copied to their backup registers so that all consistent data is copied out over a longer period.

The snapshot events are as follows:

- Write of 1 to the GICP\_ bit
- An overflow of an enabled counter when the bit is set