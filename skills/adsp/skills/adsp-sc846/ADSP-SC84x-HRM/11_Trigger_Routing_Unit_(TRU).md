## 8   Trigger Routing Unit (TRU)

The TRU provides system-level sequence control without core intervention. The TRU maps trigger generators to trigger receivers. Receiver endpoints can be configured to respond to triggers in various ways. Multiple TRUs may be provided in a multiprocessor system to create a trigger network. Common applications enabled by the TRU include:

- Automatically triggering the start of a DMA sequence after a sequence from another DMA channel completes
- Software triggering
- Synchronization of concurrent activities

## TRU Features

The TRU supports the following features:

- Automatically triggering the start of a DMA sequence after a sequence from another DMA channel completes. Once a DMA channel completes data transfer, it can act as a trigger generator and signal an internal trigger pulse to the programmed trigger receiver which can also be another DMA channel. The receiver trigger connected to the DMA channel kicks off the DMA transfer automatically. None of this requires core intervention once the initialization is done.
- Software triggers. The best use of triggers is to minimize core intervention. It is also possible to initiate a trigger pulse to a trigger receiver, in the software.
- Synchronization of concurrent activities. A single trigger generator can initiate a trigger pulse to multiple trigger receiver so that several system level activities can be synchronized on an internally or externally generated event.
- Configuration protection through register-level lock bits and global lock indication

## TRU Functional Description

The following sections provide a description of the TRU.

## ADSP-2184x TRU Register List

The Trigger Routing Unit (TRU) provides simple sequence control of distributed modules without the penalties associated with core intervention (for example, interrupt overhead). The TRU receives trigger inputs from all trigger generator inputs (MTI) and the TRU trigger receiver register ( TRU\_GEN ). Based on these inputs, the TRU logic generates trigger outputs that initiate receiver operations in the processor core and peripherals. A set of registers governs TRU operations. For more information on TRU functionality, see the TRU register descriptions.

Table 8-1: ADSP-2184x TRU Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRU_ERRADDR | Error Address Register      |
| TRU_GCTL    | Global Control Register     |
| TRU_GEN     | Generator Trigger Register  |
| TRU_RSR[n]  | Receiver Select Register    |
| TRU_STAT    | Status Information Register |

## ADSP-2184x TRU Interrupt List

Table 8-2: ADSP-2184x TRU Interrupt List

|   Interrupt ID | Name       | Description               | Sensitivity   | DMA Channel   |
|----------------|------------|---------------------------|---------------|---------------|
|            349 | TRU0_RCV0  | TRU0 Interrupt 0 - Core 0 | Edge          |               |
|            350 | TRU0_RCV1  | TRU0 Interrupt 1 - Core 0 | Edge          |               |
|            351 | TRU0_RCV2  | TRU0 Interrupt 2 - Core 0 | Edge          |               |
|            352 | TRU0_RCV3  | TRU0 Interrupt 3 - Core 0 | Edge          |               |
|            353 | TRU0_RCV4  | TRU0 Interrupt 4          | Edge          |               |
|            354 | TRU0_RCV5  | TRU0 Interrupt 5          | Edge          |               |
|            355 | TRU0_RCV6  | TRU0 Interrupt 6          | Edge          |               |
|            356 | TRU0_RCV7  | TRU0 Interrupt 7          | Edge          |               |
|            357 | TRU0_RCV8  | TRU0 Interrupt 8          | Edge          |               |
|            358 | TRU0_RCV9  | TRU0 Interrupt 9          | Edge          |               |
|            359 | TRU0_RCV10 | TRU0 Interrupt 10         | Edge          |               |
|            360 | TRU0_RCV11 | TRU0 Interrupt 11         | Edge          |               |

## ADSP-2184x Trigger List

Table 8-3: ADSP-2184x Trigger List Generators

|   Trigger ID | Name           | Description                                 | Sensitivity   |
|--------------|----------------|---------------------------------------------|---------------|
|            0 |                | Reserved                                    |               |
|            1 | CANFD0_IPD_REQ | CANFD0 DMARequest                           | None          |
|            2 | CANFD1_IPD_REQ | CANFD1 DMARequest                           | None          |
|            3 | CGU0_EVT       | CGU0 Event                                  | Edge          |
|            4 | CGU1_EVT       | CGU1 Event                                  | Edge          |
|            5 | CNT0_STAT      | CNT0 Status                                 | Level         |
|            6 | CNT0_UD        | CNT0 Count Up and Direction                 | Level         |
|            7 | CNT0_DG        | CNT0 Count Down and Gate                    | Level         |
|            8 | CNT0_TO        | CNT0 Output to Timer Block                  | Level         |
|            9 | C1_SID_ACK     | Core 1 System Interface Disable Acknowledge |               |
|           10 | C1_IDMA        | SHARC-FX (Core 1) iDMA Trigger Initiator    |               |
|           11 | MDMA0_SRC      | Enhanced BWDMAChannel 0 Source (CRC IN)     |               |
|           12 | MDMA0_DST      | Enhanced BWDMAChannel 0 Source (CRC OUT)    |               |
|           13 | MDMA1_SRC      | Enhanced BWDMAChannel 0 Source (CRC IN)     |               |
|           14 | MDMA1_DST      | Enhanced BWDMAChannel 0 Source (CRC OUT)    |               |
|           15 | MDMA4_SRC      | Standard BWDMAChannel 0 Source (CRC IN)     |               |
|           16 | MDMA4_DST      | Standard BWDMAChannel 0 Source (CRC OUT)    |               |
|           17 | MDMA5_SRC      | Standard BWDMAChannel 0 Source (CRC IN)     |               |
|           18 | MDMA5_DST      | Standard BWDMAChannel 0 Source (CRC OUT)    |               |
|           19 | ECT_EVT0       | Embedded Cross Trigger Event n              |               |
|           20 | ECT_EVT1       | Embedded Cross Trigger Event n              |               |
|           21 | ECT_EVT2       | Embedded Cross Trigger Event n              |               |
|           22 | ECT_EVT3       | Embedded Cross Trigger Event n              |               |
|           23 | ECT_EVT4       | Embedded Cross Trigger Event n              |               |

Table 8-3: ADSP-2184x Trigger List Generators (Continued)

|   Trigger ID | Name                     | Description                                                   | Sensitivity   |
|--------------|--------------------------|---------------------------------------------------------------|---------------|
|           24 | ECT_EVT5                 | Embedded Cross Trigger Event n                                |               |
|           25 | ECT_EVT6                 | Embedded Cross Trigger Event n                                |               |
|           26 | ECT_EVT7                 | Embedded Cross Trigger Event n                                |               |
|           27 | EMAC0_MCGR_DMA_REQ0      | EMAC0 MCGR DMArequest                                         | Edge          |
|           28 | EMAC0_MCGR_DMA_REQ1      | EMAC0 MCGR DMArequest                                         | Edge          |
|           29 | EMAC0_MCGR_DMA_REQ2      | EMAC0 MCGR DMArequest                                         | Edge          |
|           30 | EMAC0_MCGR_DMA_REQ3      | EMAC0 MCGR DMArequest                                         | Edge          |
|           31 | EMAC0_DMA0_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           32 | EMAC0_DMA1_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           33 | EMAC0_DMA2_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           34 | EMAC0_DMA3_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           35 | EMAC0_DMA4_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           36 | EMAC0_DMA5_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           37 | EMAC0_DMA6_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           38 | EMAC0_DMA7_RX            | EMAC0 DMAn Rx Channel Interrupt                               | None          |
|           51 | EMDMA0_DONE              | EMDMA0 DMADone                                                | Edge          |
|           52 | EMDMA1_DONE              | EMDMA1 DMADone                                                | Edge          |
|           53 | FIR0_DMA                 | FIR0 Core1DMA                                                 | Edge          |
|           54 | FIR1_DMA                 | FIR1 Core1DMA                                                 | Edge          |
|           55 | FRACNPLL0_FRACPLL_T RIGM | FRACNPLL0 Frac Pll Lock Trigger                               | Edge          |
|           56 | FRACNPLL1_FRACPLL_T RIGM | FRACNPLL1 Frac Pll Lock Trigger                               | Edge          |
|           57 | HADC0_EOC                | HADC0 HADC0 End of Conversion                                 | Edge          |
|           58 | HSM_MISC_HSM_ALRM_0      | HSM_MISC Input Interrupt 0 to HSM from TRU Responder Triggers | None          |
|           59 | HSM_MISC_HSM_ALRM_1      | HSM_MISC Input Interrupt 1 to HSM from TRU Responder Triggers | None          |
|           60 | IIR0_DMA                 | IIR0 Core1DMA                                                 | Edge          |
|           61 | IIR1_DMA                 | IIR1 Core1DMA                                                 | Edge          |
|           62 | IIR2_DMA                 | IIR2 Core1DMA                                                 | Edge          |
|           63 | IIR3_DMA                 | IIR3 Core1DMA                                                 | Edge          |

Table 8-3: ADSP-2184x Trigger List Generators (Continued)

|   Trigger ID | Name              | Description                                          | Sensitivity   |
|--------------|-------------------|------------------------------------------------------|---------------|
|           64 | L2CTL0_EVT        | L2CTL0 L2 Memory Event                               | Level         |
|           65 | L2CTL1_EVT        | L2CTL1 L2 Memory Event                               | Level         |
|           67 | LP0_DMA           | LP0 DMAChannel                                       |               |
|           68 | LP1_DMA           | LP1 DMAChannel                                       |               |
|           69 | Cn_EVENTIACK      | Event Input Request Acknowledge                      |               |
|           70 | Cn_EVENTOREQ      | Event Output Request                                 |               |
|           71 | A55_COREP_ACCEPT0 | MISC_A55 A55[0] Preq Accept                          | Edge          |
|           72 | A55_COREP_DENY0   | MISC_A55 A55[0] Preq Deny                            | Edge          |
|           73 | CLUSTERP_ACCEPT   | MISC_A55 Cluster Preq Accept                         | Edge          |
|           74 | CLUSTERP_DENY     | MISC_A55 Cluster Preq Deny                           | Edge          |
|           75 | COREP_ACCEPT2     | MISC_A55 A55[2] Preq Accept                          | Edge          |
|           76 | COREP_DENY2       | MISC_A55 A55[2] Preq Deny                            | Edge          |
|           77 | MDMA2_SRC         | Enh BWDMAChannel 0                                   |               |
|           78 | MDMA2_DST         | Enh BWDMAChannel 1                                   |               |
|           79 | MDMA3_SRC         | Max BWDMAChannel 0                                   |               |
|           80 | MDMA3_DST         | Max BWDMAChannel 1                                   |               |
|           81 | MDMA6_SRC         | Enh BWDMAChannel 0                                   |               |
|           82 | MDMA6_DST         | Enh BWDMAChannel 1                                   |               |
|           83 | MDMA7_SRC         | Max BWDMAChannel 0                                   |               |
|           84 | MDMA7_DST         | Max BWDMAChannel 1                                   |               |
|           85 | MEC1_EEIRQ0       | MEC1 ECC Error Interrupt Request                     | Level         |
|           86 | MEC1_MEIRQ2       | MEC1 Memory Error Interrupt Request                  | Level         |
|           87 | MEC1_PEIRQ0       | MEC1 Parity Error Interrupt Request                  | Level         |
|           88 | MEC2_EEIRQ0       | MEC2 ECC Error Interrupt/Trigger Request Bus         | Level         |
|           89 | MEC2_MEIRQ2       | MEC2 Memory Error Interrupt Request                  | Level         |
|           90 | MEC2_PEIRQ0       | MEC2 Parity Error Interrupt/Trigger Request Bus      | Level         |
|           91 | MEC1_MEIRQ1       | MEC1 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           92 | MEC2_MEIRQ1       | MEC2 Core Memory Error Interrupt/Trigger Request Bus | Level         |

Table 8-3: ADSP-2184x Trigger List Generators (Continued)

|   Trigger ID | Name        | Description                                          | Sensitivity   |
|--------------|-------------|------------------------------------------------------|---------------|
|           93 | MEC1_MEIRQ0 | MEC1 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           94 | MEC2_MEIRQ0 | MEC2 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           95 | MEC0_EEIRQ0 | MEC0 ECC Error Interrupt/Trigger Request Bus         | Level         |
|           96 | MEC0_MEIRQ2 | MEC0 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           97 | MEC0_PEIRQ0 | MEC0 Parity Error Interrupt Request                  | Level         |
|           98 | MEC0_MEIRQ1 | MEC0 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|           99 | MEC0_MEIRQ0 | MEC0 Core Memory Error Interrupt/Trigger Request Bus | Level         |
|          100 |             | Reserved                                             |               |
|          101 |             | Reserved                                             |               |
|          102 |             | Reserved                                             |               |
|          103 |             | Reserved                                             |               |
|          104 |             | Reserved                                             |               |
|          105 |             | Reserved                                             |               |
|          106 |             | Reserved                                             |               |
|          107 |             | Reserved                                             |               |
|          108 | PINT0_BLOCK | PINT0 Pin Interrupt Block                            | Level         |
|          109 | PINT1_BLOCK | PINT1 Pin Interrupt Block                            | Level         |
|          110 | PINT2_BLOCK | PINT2 Pin Interrupt Block                            | Level         |
|          111 | PINT3_BLOCK | PINT3 Pin Interrupt Block                            | Level         |
|          112 | PINT4_BLOCK | PINT4 Pin Interrupt Block                            | Level         |
|          113 | PINT5_BLOCK | PINT5 Pin Interrupt Block                            | Level         |
|          114 | PINT6_BLOCK | PINT6 Pin Interrupt Block                            | Level         |
|          115 | PINT7_BLOCK | PINT7 Pin Interrupt Block                            | Level         |
|          116 | SEC0_FAULT  | SEC0 Fault                                           | Edge          |
|          117 | SPI0_TXDMA  | SPI0 TX DMAChannel                                   | Edge          |
|          118 | SPI0_RXDMA  | SPI0 RX DMAChannel                                   | Edge          |
|          119 | SPI1_TXDMA  | SPI1 TX DMAChannel                                   | Edge          |
|          120 | SPI1_RXDMA  | SPI1 RX DMAChannel                                   | Edge          |

Table 8-3: ADSP-2184x Trigger List Generators (Continued)

|   Trigger ID | Name                   | Description                      | Sensitivity   |
|--------------|------------------------|----------------------------------|---------------|
|          121 | SPI2_TXDMA             | SPI2 TX DMAChannel               | Edge          |
|          122 | SPI2_RXDMA             | SPI2 RX DMAChannel               | Edge          |
|          123 | SPI5_TXDMA             | SPI5 TX DMAChannel               | Edge          |
|          124 | SPI5_RXDMA             | SPI5 RX DMAChannel               | Edge          |
|          125 | SPORT0_A_DMA           | SPORT0 ChannelADMA               | Edge          |
|          126 | SPORT0_B_DMA           | SPORT0 ChannelBDMA               | Edge          |
|          127 | SPORT1_A_DMA           | SPORT1 ChannelADMA               | Edge          |
|          128 | SPORT1_B_DMA           | SPORT1 ChannelBDMA               | Edge          |
|          129 | SPORT2_A_DMA           | SPORT2 ChannelADMA               | Edge          |
|          130 | SPORT2_B_DMA           | SPORT2 ChannelBDMA               | Edge          |
|          131 | SPORT3_A_DMA           | SPORT3 ChannelADMA               | Edge          |
|          132 | SPORT3_B_DMA           | SPORT3 ChannelBDMA               | Edge          |
|          133 | SPORT4_A_DMA           | SPORT4 ChannelADMA               | Edge          |
|          134 | SPORT4_B_DMA           | SPORT4 ChannelBDMA               | Edge          |
|          135 | SPORT5_A_DMA           | SPORT5 ChannelADMA               | Edge          |
|          136 | SPORT5_B_DMA           | SPORT5 ChannelBDMA               | Edge          |
|          137 | SPORT6_A_DMA           | SPORT6 ChannelADMA               | Edge          |
|          138 | SPORT6_B_DMA           | SPORT6 ChannelBDMA               | Edge          |
|          139 | SPORT7_A_DMA           | SPORT7 ChannelADMA               | Edge          |
|          140 | SPORT7_B_DMA           | SPORT7 ChannelBDMA               | Edge          |
|          141 | DAI0_GBL_SPORT_TRG_ O0 | DAI0 SPORT GROUP0 Trigger Output | None          |
|          142 | DAI0_GBL_SPORT_TRG_ O1 | DAI0 SPORT GROUP1 Trigger Output | None          |
|          143 | DAI1_GBL_SPORT_TRG_ O0 | DAI1 SPORT GROUP2 Trigger Output | None          |
|          144 | DAI1_GBL_SPORT_TRG_ O1 | DAI1 SPORT Group3 Trigger Output | None          |
|          145 |                        | Reserved                         |               |
|          146 | SWU1_EVT               | SWU1 Event                       | None          |
|          147 | SWU2_EVT               | SWU2 Event                       | None          |
|          148 | SWU7_EVT               | SWU7 Event                       | None          |

Table 8-3: ADSP-2184x Trigger List Generators (Continued)

|   Trigger ID | Name             | Description                      | Sensitivity   |
|--------------|------------------|----------------------------------|---------------|
|          149 | SWU8_EVT         | SWU8 Event                       | None          |
|          150 | SWU11_EVT        | SWU11 Event                      | None          |
|          151 | SWU12_EVT        | SWU12 Event                      | None          |
|          152 | SWU13_EVT        | SWU13 Event                      | None          |
|          153 | SWU3_EVT         | SWU3 Event                       | None          |
|          154 | SWU4_EVT         | SWU4 Event                       | None          |
|          155 | SWU5_EVT         | SWU5 Event                       | None          |
|          156 |                  | Reserved                         |               |
|          157 | SWU1_DBG         | SWU1 Debug                       | Edge          |
|          158 | SWU2_DBG         | SWU2 Debug                       | Edge          |
|          159 | SWU7_DBG         | SWU7 Debug                       | Edge          |
|          160 | SWU8_DBG         | SWU8 Debug                       | Edge          |
|          161 | SWU11_DBG        | SWU11 Debug                      | Edge          |
|          162 | SWU12_DBG        | SWU12 Debug                      | Edge          |
|          163 | SWU13_DBG        | SWU13 Debug                      | Edge          |
|          164 | SWU3_DBG         | SWU3 Debug                       | Edge          |
|          165 | SWU4_DBG         | SWU4 Debug                       | Edge          |
|          166 | SWU5_DBG         | SWU5 Debug                       | Edge          |
|          167 | SOFT0_MST        | Software-driven Trigger 0        |               |
|          168 | SOFT1_MST        | Software-driven Trigger 1        |               |
|          169 | SOFT2_MST        | Software-driven Trigger 2        |               |
|          170 | SOFT3_MST        | Software-driven Trigger 3        |               |
|          171 | SOFT4_MST        | Software-driven Trigger 4        |               |
|          172 | SOFT5_MST        | Software-driven Trigger 5        |               |
|          173 | TIMER0_TMR00_GEN | TIMER0 Timer 0 Trigger Initiator | Edge          |
|          174 | TIMER0_TMR01_GEN | TIMER0 Timer 1 Trigger Initiator | Edge          |
|          175 | TIMER0_TMR02_GEN | TIMER0 Timer 2 Trigger Initiator | Edge          |
|          176 | TIMER0_TMR03_GEN | TIMER0 Timer 3 Trigger Initiator | Edge          |
|          177 | TIMER0_TMR04_GEN | TIMER0 Timer 4 Trigger Initiator | Edge          |
|          178 | TIMER0_TMR05_GEN | TIMER0 Timer 5 Trigger Initiator | Edge          |
|          179 | TIMER0_TMR06_GEN | TIMER0 Timer 6 Trigger Initiator | Edge          |

Table 8-3: ADSP-2184x Trigger List Generators (Continued)

|   Trigger ID | Name             | Description                       | Sensitivity   |
|--------------|------------------|-----------------------------------|---------------|
|          180 | TIMER0_TMR07_GEN | TIMER0 Timer 7 Trigger Initiator  | Edge          |
|          181 | TIMER0_TMR08_GEN | TIMER0 Timer 8 Trigger Initiator  | Edge          |
|          182 | TIMER0_TMR09_GEN | TIMER0 Timer 9 Trigger Initiator  | Edge          |
|          183 | TIMER0_TMR10_GEN | TIMER0 Timer 10 Trigger Initiator | Edge          |
|          184 | TIMER0_TMR11_GEN | TIMER0 Timer 11 Trigger Initiator | Edge          |
|          185 | TIMER0_TMR12_GEN | TIMER0 Timer 12 Trigger Initiator | Edge          |
|          186 | TIMER0_TMR13_GEN | TIMER0 Timer 13 Trigger Initiator | Edge          |
|          187 | TIMER0_TMR14_GEN | TIMER0 Timer 14 Trigger Initiator | Edge          |
|          188 | TIMER0_TMR15_GEN | TIMER0 Timer 15 Trigger Initiator | Edge          |
|          189 | TMU0_FAULT       | TMU0 TM0 Fault Event              |               |
|          190 | TMU0_ALERT       | TMU0 TM0 Alert Event              |               |
|          191 | UART0_TXDMA      | UART0 TransmitDMA                 | Edge          |
|          192 | UART0_RXDMA      | UART0 ReceiveDMA                  | Edge          |
|          193 | UART1_TXDMA      | UART1 TransmitDMA                 | Edge          |
|          194 | UART1_RXDMA      | UART1 ReceiveDMA                  | Edge          |
|          195 | UART2_TXDMA      | UART2 TransmitDMA                 | Edge          |
|          196 | UART2_RXDMA      | UART2 ReceiveDMA                  | Edge          |
|          197 | WDOG0_EXP        | WDOG0 Expiration                  | Level         |
|          198 | WDOG1_EXP        | WDOG1 Expiration                  | Level         |
|          199 | WDOG2_EXP        | WDOG2 Expiration                  | Level         |
|          200 | WDOG3_EXP        | WDOG3 Expiration                  | Level         |

Table 8-4: ADSP-2184x Trigger List Receivers

|   Trigger ID | Name      | Description                              | Sensitivity   |
|--------------|-----------|------------------------------------------|---------------|
|            0 |           | Reserved                                 |               |
|            1 |           | Reserved                                 |               |
|            2 | CNT0_UD   | CNT0 Count Up and Direction              | Pulse         |
|            3 | CNT0_DG   | CNT0 Count Down and Gate                 | Pulse         |
|           11 | MDMA0_SRC | Enhanced BWDMAChannel 0 Source (CRC IN)  | Pulse         |
|           12 | MDMA0_DST | Enhanced BWDMAChannel 0 Source (CRC OUT) | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name                | Description                              | Sensitivity   |
|--------------|---------------------|------------------------------------------|---------------|
|           13 | MDMA1_SRC           | Enhanced BWDMAChannel 0 Source (CRC IN)  | Pulse         |
|           14 | MDMA1_DST           | Enhanced BWDMAChannel 0 Source (CRC OUT) | Pulse         |
|           15 | MDMA4_SRC           | Standard BWDMAChannel 0 Source (CRC IN)  | Pulse         |
|           16 | MDMA4_DST           | Standard BWDMAChannel 0 Source (CRC OUT) | Pulse         |
|           17 | MDMA5_SRC           | Standard BWDMAChannel 1 Source (CRC IN)  | Pulse         |
|           18 | MDMA5_DST           | Standard BWDMAChannel 1 Source (CRC OUT) | Pulse         |
|           19 | ECT_EVT0            | Embedded Cross Trigger Event n           | Pulse         |
|           20 | ECT_EVT1            | Embedded Cross Trigger Event n           | Pulse         |
|           21 | ECT_EVT2            | Embedded Cross Trigger Event n           | Pulse         |
|           22 | ECT_EVT3            | Embedded Cross Trigger Event n           | Pulse         |
|           23 | ECT_EVT4            | Embedded Cross Trigger Event n           | Pulse         |
|           24 | ECT_EVT5            | Embedded Cross Trigger Event n           | Pulse         |
|           25 | ECT_EVT6            | Embedded Cross Trigger Event n           | Pulse         |
|           26 | EMAC0_DMA0_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           27 | EMAC0_DMA1_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           28 | EMAC0_DMA2_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           29 | EMAC0_DMA3_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           30 | EMAC0_DMA4_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           31 | EMAC0_DMA5_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           32 | EMAC0_DMA6_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           33 | EMAC0_DMA7_RX_Start | EMAC0 DMAn Rx Channel Start Trigger      | Pulse         |
|           34 | EMAC0_DMA0_TX_Start | EMAC0 DMAn Tx Channel Start Trigger      | Pulse         |
|           35 | EMAC0_DMA1_TX_Start | EMAC0 DMAn Tx Channel Start Trigger      | Pulse         |
|           36 | EMAC0_DMA2_TX_Start | EMAC0 DMAn Tx Channel Start Trigger      | Pulse         |
|           37 | EMAC0_DMA3_TX_Start | EMAC0 DMAn Tx Channel Start Trigger      | Pulse         |
|           38 | EMAC0_DMA4_TX_Start | EMAC0 DMAn Tx Channel Start Trigger      | Pulse         |
|           39 | EMAC0_DMA5_TX_Start | EMAC0 DMAn Tx Channel Start Trigger      | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name                     | Description                           | Sensitivity   |
|--------------|--------------------------|---------------------------------------|---------------|
|           40 | EMAC0_DMA6_TX_Start      | EMAC0 DMAn Tx Channel Start Trigger   | Pulse         |
|           41 | EMAC0_DMA7_TX_Start      | EMAC0 DMAn Tx Channel Start Trigger   | Pulse         |
|           42 | EMAC0_DMA0_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           43 | EMAC0_DMA1_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           44 | EMAC0_DMA2_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           45 | EMAC0_DMA3_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           46 | EMAC0_DMA4_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           47 | EMAC0_DMA5_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           48 | EMAC0_DMA6_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           49 | EMAC0_DMA7_RX_STOP       | EMAC0 DMAn Rx Channel Stop Trigger    | Pulse         |
|           50 | EMAC0_DMA0_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           51 | EMAC0_DMA1_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           52 | EMAC0_DMA2_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           53 | EMAC0_DMA3_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           54 | EMAC0_DMA4_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           55 | EMAC0_DMA5_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           56 | EMAC0_DMA6_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           57 | EMAC0_DMA7_TX_STOP       | EMAC0 DMAn Tx Channel Stop Trigger    | Pulse         |
|           58 | EMAC0_MCGR_DMA_ACK0      | EMAC0 MCGR DMAacknowledge             | Pulse         |
|           59 | EMAC0_MCGR_DMA_ACK1      | EMAC0 MCGR DMAacknowledge             | Pulse         |
|           60 | EMAC0_MCGR_DMA_ACK2      | EMAC0 MCGR DMAacknowledge             | Pulse         |
|           61 | EMAC0_MCGR_DMA_ACK3      | EMAC0 MCGR DMAacknowledge             | Pulse         |
|           98 | FIR0_TRGI                | FIR0 Core 1 FIR Wait on Trigger Input | Pulse         |
|           99 | FIR1_TRGI                | FIR1 Core 1 FIR Wait on Trigger Input | Pulse         |
|          100 | FRACNPLL0_FRACPLL_T RIGS | FRACNPLL0 Frac Pll Start Trigger      | Pulse         |
|          101 | FRACNPLL1_FRACPLL_T RIGS | FRACNPLL1 Frac Pll Start Trigger      | Pulse         |
|          102 | IIR0_TRGI                | IIR0 Core 1 IIR Wait on Trigger Input | Pulse         |
|          103 | IIR1_TRGI                | IIR1 Core 1 IIR Wait on Trigger Input | Pulse         |
|          104 | IIR2_TRGI                | IIR2 Core 1 IIR Wait on Trigger Input | Pulse         |
|          105 | IIR3_TRGI                | IIR3 Core 1 IIR Wait on Trigger Input | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name                | Description                                                   | Sensitivity   |
|--------------|---------------------|---------------------------------------------------------------|---------------|
|          106 | HSM_MISC_HSM_ALRM_0 | HSM_MISC Input Interrupt 0 to HSM from TRU Responder Triggers | Pulse         |
|          107 | HSM_MISC_HSM_ALRM_1 | HSM_MISC Input Interrupt 1 to HSM from TRU Responder Triggers | Pulse         |
|          108 | HSM_MISC_HSM_ALRM_2 | HSM_MISC Input Interrupt 2 to HSM from TRU Responder Triggers | Pulse         |
|          109 | HSM_MISC_HSM_ALRM_3 | HSM_MISC Input Interrupt 3 to HSM from TRU Responder Triggers | Pulse         |
|          110 | HSM_MISC_HSM_ALRM_4 | HSM_MISC Input Interrupt 4 to HSM from TRU Responder Triggers | Pulse         |
|          111 | HSM_MISC_HSM_ALRM_5 | HSM_MISC Input Interrupt 5 to HSM from TRU Responder Triggers | Pulse         |
|          112 | HSM_MISC_HSM_ALRM_6 | HSM_MISC Input Interrupt 6 to HSM from TRU Responder Triggers | Pulse         |
|          113 | HSM_MISC_HSM_ALRM_7 | HSM_MISC Input Interrupt 7 to HSM from TRU Responder Triggers | Pulse         |
|          114 | LP0_DMA             | LP0 DMAChannel                                                | Pulse         |
|          115 | LP1_DMA             | LP1 DMAChannel                                                | Pulse         |
|          116 | Cn_EVENTIREQ        | Event Input Request for Core Wake-up From WFE State           | Pulse         |
|          117 | Cn_EVENTOACK        | Event Output Acknowledgment for Core to Wakeup                | Pulse         |
|          120 | MDMA2_SRC           | Enh BWDMAChannel 0                                            | Pulse         |
|          121 | MDMA2_DST           | Enh BWDMAChannel 1                                            | Pulse         |
|          122 | MDMA3_SRC           | Max BWDMAChannel 0                                            | Pulse         |
|          123 | MDMA3_DST           | Max BWDMAChannel 1                                            | Pulse         |
|          124 | MDMA6_SRC           | Enh BWDMAChannel 0                                            | Pulse         |
|          125 | MDMA6_DST           | Enh BWDMAChannel 1                                            | Pulse         |
|          126 | MDMA7_SRC           | Max BWDMAChannel 0                                            | Pulse         |
|          127 | MDMA7_DST           | Max BWDMAChannel 1                                            | Pulse         |
|          128 | PCG0_HWA            | PCG0 PCG-A Hardware trigger control                           | Pulse         |
|          129 | PCG0_HWB            | PCG0 PCG-B Hardware trigger control                           | Pulse         |
|          130 | PCG0_HWC            | PCG0 PCG-C Hardware trigger control                           | Pulse         |
|          131 | PCG0_HWD            | PCG0 PCG-D Hardware trigger control                           | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name         | Description                         | Sensitivity   |
|--------------|--------------|-------------------------------------|---------------|
|          132 | PCG0_HWE     | PCG0 PCG-E Hardware trigger control | Pulse         |
|          133 | PCG0_HWF     | PCG0 PCG-F Hardware trigger control | Pulse         |
|          134 | PCG0_HWG     | PCG0 PCG-G Hardware trigger control | Pulse         |
|          135 | PCG0_HWH     | PCG0 PCG-H Hardware trigger control | Pulse         |
|          136 | PORTA_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          137 | PORTB_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          138 | PORTC_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          139 | PORTD_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          140 | PORTE_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          141 | PORTF_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          142 | PORTG_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          143 | PORTH_TOGGLE | Port Toggle Trigger                 | Pulse         |
|          145 | PWM0_TRIP    | PWM0 PWMTrip1B Trigger In           | Pulse         |
|          146 | PWM0_SYNC    | PWM0 PWMTMRGrouped                  | Pulse         |
|          147 | RCU0_SYSRST0 | RCU0 System Reset Receiver 0        | Pulse         |
|          148 | RCU0_SYSRST1 | RCU0 System Reset Receiver 1        | Pulse         |
|          149 | SPI0_TXDMA   | SPI0 TX DMAChannel                  | Pulse         |
|          150 | SPI0_RXDMA   | SPI0 RX DMAChannel                  | Pulse         |
|          151 | SPI1_TXDMA   | SPI1 TX DMAChannel                  | Pulse         |
|          152 | SPI1_RXDMA   | SPI1 RX DMAChannel                  | Pulse         |
|          153 | SPI5_TXDMA   | SPI5 TX DMAChannel                  | Pulse         |
|          154 | SPI5_RXDMA   | SPI5 RX DMAChannel                  | Pulse         |
|          155 | SPI2_TXDMA   | SPI2 TX DMAChannel                  | Pulse         |
|          156 | SPI2_RXDMA   | SPI2 RX DMAChannel                  | Pulse         |
|          157 | SPORT0_A_DMA | SPORT0 ChannelADMA                  | Pulse         |
|          158 | SPORT0_B_DMA | SPORT0 ChannelBDMA                  | Pulse         |
|          159 | SPORT1_A_DMA | SPORT1 ChannelADMA                  | Pulse         |
|          160 | SPORT1_B_DMA | SPORT1 ChannelBDMA                  | Pulse         |
|          161 | SPORT2_A_DMA | SPORT2 ChannelADMA                  | Pulse         |
|          162 | SPORT2_B_DMA | SPORT2 ChannelBDMA                  | Pulse         |
|          163 | SPORT3_A_DMA | SPORT3 ChannelADMA                  | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name                   | Description                        | Sensitivity   |
|--------------|------------------------|------------------------------------|---------------|
|          164 | SPORT3_B_DMA           | SPORT3 ChannelBDMA                 | Pulse         |
|          165 | SPORT4_A_DMA           | SPORT4 ChannelADMA                 | Pulse         |
|          166 | SPORT4_B_DMA           | SPORT4 ChannelBDMA                 | Pulse         |
|          167 | SPORT5_A_DMA           | SPORT5 ChannelADMA                 | Pulse         |
|          168 | SPORT5_B_DMA           | SPORT5 ChannelBDMA                 | Pulse         |
|          169 | SPORT6_A_DMA           | SPORT6 ChannelADMA                 | Pulse         |
|          170 | SPORT6_B_DMA           | SPORT6 ChannelBDMA                 | Pulse         |
|          171 | SPORT7_A_DMA           | SPORT7 ChannelADMA                 | Pulse         |
|          172 | SPORT7_B_DMA           | SPORT7 ChannelBDMA                 | Pulse         |
|          173 | DAI0_GBL_SPORT_TRG_ I0 | DAI0 SPORT GROUP0 Trigger Input    | Pulse         |
|          174 | DAI0_GBL_SPORT_TRG_ I1 | DAI0 SPORT GROUP1 Trigger Input    | Pulse         |
|          175 | DAI1_GBL_SPORT_TRG_ I0 | DAI1 SPORT GROUP2 Trigger Input    | Pulse         |
|          176 | DAI1_GBL_SPORT_TRG_ I1 | DAI1 SPORT GROUP3 Trigger Input    | Pulse         |
|          177 |                        | Reserved                           |               |
|          178 | SWU1_EN                | SWU1 Enable                        | Pulse         |
|          179 | SWU2_EN                | SWU2 Enable                        | Pulse         |
|          180 | SWU7_EN                | SWU7 Enable                        | Pulse         |
|          181 | SWU8_EN                | SWU8 Enable                        | Pulse         |
|          182 | SWU11_EN               | SWU11 Enable                       | Pulse         |
|          183 | SWU12_EN               | SWU12 Enable                       | Pulse         |
|          184 | SWU13_EN               | SWU13 Enable                       | Pulse         |
|          185 | SWU3_EN                | SWU3 Enable Event CL2_1            | Pulse         |
|          186 | SWU4_EN                | SWU4 Enable Event DL2_1            | Pulse         |
|          187 | SWU5_EN                | SWU5 Enable Event CL2_2            | Pulse         |
|          188 | TIMER0_TMR00_RCV0      | TIMER0 Timer 0 Trigger Responder 0 | Pulse         |
|          189 | TIMER0_TMR00_RCV1      | TIMER0 Timer 0 Trigger Responder 1 | Pulse         |
|          190 | TIMER0_TMR01_RCV0      | TIMER0 Timer 1 Trigger Responder 0 | Pulse         |
|          191 | TIMER0_TMR01_RCV1      | TIMER0 Timer 1 Trigger Responder 1 | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name              | Description                         | Sensitivity   |
|--------------|-------------------|-------------------------------------|---------------|
|          192 | TIMER0_TMR02_RCV0 | TIMER0 Timer 2 Trigger Responder 0  | Pulse         |
|          193 | TIMER0_TMR02_RCV1 | TIMER0 Timer 2 Trigger Responder 1  | Pulse         |
|          194 | TIMER0_TMR03_RCV0 | TIMER0 Timer 3 Trigger Responder 0  | Pulse         |
|          195 | TIMER0_TMR03_RCV1 | TIMER0 Timer 3 Trigger Responder 1  | Pulse         |
|          196 | TIMER0_TMR04_RCV0 | TIMER0 Timer 4 Trigger Responder 0  | Pulse         |
|          197 | TIMER0_TMR04_RCV1 | TIMER0 Timer 4 Trigger Responder 1  | Pulse         |
|          198 | TIMER0_TMR05_RCV0 | TIMER0 Timer 5 Trigger Responder 0  | Pulse         |
|          199 | TIMER0_TMR05_RCV1 | TIMER0 Timer 5 Trigger Responder 1  | Pulse         |
|          200 | TIMER0_TMR06_RCV0 | TIMER0 Timer 6 Trigger Responder 0  | Pulse         |
|          201 | TIMER0_TMR06_RCV1 | TIMER0 Timer 6 Trigger Responder 1  | Pulse         |
|          202 | TIMER0_TMR07_RCV0 | TIMER0 Timer 7 Trigger Responder 0  | Pulse         |
|          203 | TIMER0_TMR07_RCV1 | TIMER0 Timer 7 Trigger Responder 1  | Pulse         |
|          204 | TIMER0_TMR08_RCV0 | TIMER0 Timer 8 Trigger Responder 0  | Pulse         |
|          205 | TIMER0_TMR08_RCV1 | TIMER0 Timer 8 Trigger Responder 1  | Pulse         |
|          206 | TIMER0_TMR09_RCV0 | TIMER0 Timer 9 Trigger Responder 0  | Pulse         |
|          207 | TIMER0_TMR09_RCV1 | TIMER0 Timer 9 Trigger Responder 1  | Pulse         |
|          208 | TIMER0_TMR10_RCV0 | TIMER0 Timer 10 Trigger Responder 0 | Pulse         |
|          209 | TIMER0_TMR10_RCV1 | TIMER0 Timer 10 Trigger Responder 1 | Pulse         |
|          210 | TIMER0_TMR11_RCV0 | TIMER0 Timer 11 Trigger Responder 0 | Pulse         |
|          211 | TIMER0_TMR11_RCV1 | TIMER0 Timer 11 Trigger Responder 1 | Pulse         |
|          212 | TIMER0_TMR12_RCV0 | TIMER0 Timer 12 Trigger Responder 0 | Pulse         |
|          213 | TIMER0_TMR12_RCV1 | TIMER0 Timer 12 Trigger Responder 1 | Pulse         |
|          214 | TIMER0_TMR13_RCV0 | TIMER0 Timer 13 Trigger Responder 0 | Pulse         |
|          215 | TIMER0_TMR13_RCV1 | TIMER0 Timer 13 Trigger Responder 1 | Pulse         |
|          216 | TIMER0_TMR14_RCV0 | TIMER0 Timer 14 Trigger Responder 0 | Pulse         |
|          217 | TIMER0_TMR14_RCV1 | TIMER0 Timer 14 Trigger Responder 1 | Pulse         |
|          218 | TIMER0_TMR15_RCV0 | TIMER0 Timer 15 Trigger Responder 0 | Pulse         |
|          219 | TIMER0_TMR15_RCV1 | TIMER0 Timer 15 Trigger Responder 1 | Pulse         |
|          220 | TRU0_RCV0         | TRU0 Interrupt Request n            | Pulse         |
|          221 | TRU0_RCV1         | TRU0 Interrupt Request n            | Pulse         |
|          222 | TRU0_RCV2         | TRU0 Interrupt Request n            | Pulse         |

Table 8-4: ADSP-2184x Trigger List Receivers (Continued)

|   Trigger ID | Name        | Description              | Sensitivity   |
|--------------|-------------|--------------------------|---------------|
|          223 | TRU0_RCV3   | TRU0 Interrupt Request n | Pulse         |
|          224 | TRU0_RCV4   | TRU0 Interrupt Request 4 | Pulse         |
|          225 | TRU0_RCV5   | TRU0 Interrupt Request 5 | Pulse         |
|          226 | TRU0_RCV6   | TRU0 Interrupt Request 6 | Pulse         |
|          227 | TRU0_RCV7   | TRU0 Interrupt Request 7 | Pulse         |
|          228 | TRU0_RCV8   | TRU0 Interrupt Request n | Pulse         |
|          229 | TRU0_RCV9   | TRU0 Interrupt Request n | Pulse         |
|          230 | TRU0_RCV10  | TRU0 Interrupt Request n | Pulse         |
|          231 | TRU0_RCV11  | TRU0 Interrupt Request n | Pulse         |
|          232 | UART0_TXDMA | UART0 TransmitDMA        | Pulse         |
|          233 | UART0_RXDMA | UART0 ReceiveDMA         | Pulse         |
|          234 | UART1_TXDMA | UART1 TransmitDMA        | Pulse         |
|          235 | UART1_RXDMA | UART1 ReceiveDMA         | Pulse         |
|          236 | UART2_TXDMA | UART2 TransmitDMA        | Pulse         |
|          237 | UART2_RXDMA | UART2 ReceiveDMA         | Pulse         |

## TRU Definitions

The following definitions are helpful when using the TRU module.

## Trigger Generator

A trigger generator is any system module that provides trigger event indication to the TRU. T rigger generator modules define trigger events and conditions for assertion.

## Trigger Generator ID

Trigger generators are assigned a unique numeric ID according to their physical connection to the TRU. Trigger receiver ID 0 is reserved and defined as null.

## Trigger Receiver

A trigger receiver is any system module that receives a trigger event indication from the TRU. T rigger receiver modules define a trigger event response.

## TRU Block Diagram

The trigger generator and the generator trigger register (GTR) generate trigger assertions. Each trigger receiver has a dedicated receiver select register (RSR) that specifies the unique trigger generator from which it receives the trigger indication.

Figure 8-1: TRU Block Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000000_4831f242e1ce03bd6876ea1eb3ae89e664132110ba7d851bfec4f83b18a8ba08.png)

## TRU Architectural Concepts

The TRU supports a simple trigger-in/trigger-out model for modules that comply with the triggering functional model. The TRU is the controller of the trigger system. T rigger outputs from trigger generators are mapped to trigger inputs of trigger receivers through a set of programmable registers ( TRU\_RSR[n] ).

System modules are trigger generator only, trigger receiver only, or trigger generator and trigger receiver.

All of the trigger input and output signals are connected to a trigger routing unit (TRU) which manages the connections of triggers between modules.

In multi-processor systems, multiple TRU units are provided. These TRUs are networked together. Generic Trigger Ports (GTPs) are provided to forward trigger events from one TRU unit to another, forming a pathway from trigger generator to trigger receivers wherever they might lie in the system.

## TRU Programming Model

Implementing sequence control using the TRU requires, at a minimum, proper configuration of a trigger receiver, a trigger generator, and the TRU module itself. The only requirement for the configuration procedure is that the trigger generator is configured and enabled as the last step.

Complete the following other steps:

- Configure the trigger receiver for response to triggers
- Configure the TRU to map the trigger generator to the trigger receiver through the TRU\_RSR[n] registers
- Configure the trigger generator to generate trigger assertions
- Alternatively, use software triggering for trigger assertion. Writing the trigger generator ID to the GTR register generates software triggers.

## Programming Concepts

The following concepts aid in programming the TRU.

- Trigger Sequence Configuration. A simple sequence consists of one trigger generator and one trigger receiver. More complex trigger sequences consist of several trigger receivers functioning as trigger receiver and trigger generator. Additionally, trigger sequences can loopback to the original generator forming a perpetual sequence.
- Software Triggering. Writing a trigger generator ID to the GTR generates a trigger within the TRU from the trigger generator ID specified.
- Synchronization. The TRU can be used to coarsely synchronize events by mapping multiple trigger receivers to the same trigger generator or by generating multiple trigger generator assertions simultaneously through the GTR.
- Configuration Protection. The TRU\_RSR[n].LOCK bit and the TRU\_GCTL.LOCK bit enable register level write-protection when the global lock is asserted in the SPU.

## Programming Examples

The following examples shows the steps to create a single trigger.

## Configuring a Simple Trigger Sequence

The following example shows the steps to create a simple trigger.

1. Enable the Global Lock feature bit on the SPU (set the SPU\_CTL.GLCK bit).
2. Set the TRU\_GCTL.LOCK bit to 1 so that the TRU\_GCTL register becomes read-only.
3. Set the TRU\_GCTL.GENL bit to 1 so that the TRU\_GEN register becomes read-only.

## TRU Event Control

The TRU is a major part of event control solutions. It is the center of the trigger functional model and can extend to support the interrupt and fault management models as well.

## TRU Status and Error Signals

The TRU does not have dedicated status and error output signals other than the MMR interface. Receiver errors are reported to the generator over the standard peripheral bus protocol.

## ADSP-2184x TRU Register Descriptions

Trigger Routing Unit (TRU) contains the following registers.

Table 8-5: ADSP-2184x TRU Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRU_ERRADDR | Error Address Register      |
| TRU_GCTL    | Global Control Register     |
| TRU_GEN     | Generator Trigger Register  |
| TRU_RSR[n]  | Receiver Select Register    |
| TRU_STAT    | Status Information Register |

## Error Address Register

The TRU error address register ( TRU\_ERRADDR ) holds the address from the memory-mapped register access generating an access error of TRU registers.

Figure 8-2: TRU\_ERRADDR Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000001_bc52e3329f4fca787fd79936233fcf86ede4df3c2f1433da7144d072ef0bf9c8.png)

Table 8-6: TRU\_ERRADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | ADDR       | Error Address. The TRU_ERRADDR.ADDR holds the address from the memory-mapped register access generating an access error of TRU registers. These errors occur on access to the TRU_RSR[n] or TRU_GEN registers when these registers are locked or on access to an invalid address. See the TRU_RSR[n] and TRU_GEN register descriptions for more information about locking. The TRU_ERRADDR register holds the address of the first error to occur. In the event of multiple errors occurring, the TRU_ERRADDR register contains the address of the first error. To re-enable the TRU_ERRADDR register for update, both status bits ( TRU_STAT.LWERR and TRU_STAT.ADDRERR ) in the TRU_STAT register must be cleared. |

## Global Control Register

The TRU global control register ( TRU\_GCTL ) provides register locking, TRU reset, and TRU enable.

Figure 8-3: TRU\_GCTL Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000002_1bd2c3531d8b2dc37cf6ad6ce617b741dbc7d88a80ac48e07420d1140cbe95cf.png)

Table 8-7: TRU\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | GCTL Lock Bit. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_GCTL.LOCK bit is enabled, the TRU_GCTL register is read only. |
| 2 (R/W)            | GENL       | GEN Lock Bit. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_GCTL.GENL bit is enabled, the TRU_GEN register is read only.   |
| 1 (R/W)            | RESET      | Soft Reset. The TRU_GCTL.RESET bit is write-1-action and triggers a soft reset to all TRU registers. 0 No Action                              |

Table 8-7: TRU\_GCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Non-MMR Enable. The TRU_GCTL.EN bit is read/write and must be set for the TRU to propagate trigger events. All TRU register read/write operations continue to operate independent of the TRU_GCTL.EN bit. 0 No Trigger Events |

## Generator Trigger Register

The TRU generator trigger register ( TRU\_GEN ) permits trigger generation through software by writing a trigger generator ID value to one of the four fields in the TRU\_GEN register. If the global lock is enabled ( SPU\_CTL.GLCK bit =1) and the TRU\_GCTL.LOCK bit is set, the TRU\_GEN register is read only. Note this register is primarily used for debug to trigger a TRU output

Figure 8-4: TRU\_GEN Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000003_ae9319513f8e87380c3448fcce29e64001c13a0c147ec34913e0316274c2ad54.png)

Table 8-8: TRU\_GEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | GEN3       | Generator Trigger Register 3. The TRU_GEN.GEN3 bit field is the trigger generator ID value for generator 3. 0 No Generator Specified                                 |
| 23:16 (R/W)        | GEN2       | Generator Trigger Register 2. The TRU_GEN.GEN2 bit field is the trigger generator ID value for generator 2. 0 No Generator Specified                                 |
| 15:8 (R/W)         | GEN1       | Generator Trigger Register 1. The TRU_GEN.GEN1 bit field is the trigger generator ID value for generator 1. 0 No Generator Specified                                 |
| 7:0 (R/W)          | GEN0       | Generator Trigger Register 0. The TRU_GEN.GEN0 bit field is the trigger generator ID value for generator 0. 0 No Generator Specified 1-182 Range of Valid Generators |

## Receiver Select Register

The TRU receiver select registers ( TRU\_RSR[n] ) each provide trigger receiver selection and register locking.

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000004_078da84ed1bb5520c50afa951797d738cbc15716f4cdf3fe3ff76b9f5d361721.png)

RSRn Lock

Figure 8-5: TRU\_RSR[n] Register Diagram

Table 8-9: TRU\_RSR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | RSRn Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_RSR[n].LOCK bit is enabled, the TRU_RSR[n] register is read only. 0 Unlock Register                                                                                                                                                                      |
| 7:0 (R/W)          | RSR        | RSRn Receiver Select. The TRU_RSR[n] register selects the trigger generator ID to which the trigger receiver responds. For example, when a TRU_RSR[n] register is set to respond to trigger generator ID n, a trigger that is generated by trigger generator ID n results in a trigger out to the receiver. 0 No Generator Specified |

## Status Information Register

The TRU status register ( TRU\_STAT ) contains the status of TRU\_GEN and TRU\_RSR[n] register writes and status of bus read/write errors.

Figure 8-6: TRU\_STAT Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000005_6f7fea53b8745bec8d978c054b7731e563eaf46eb71b2728bc7fa94fe112cf8f.png)

Table 8-10: TRU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | ADDRERR    | Address Error Status. The TRU_STAT.ADDRERR bit is set when an invalid address is provided for an MMRaccess while the TRU is selected. Writing a one to this bit clears the error indication. The TRU_ERRADDR register also is updated when an address error occurs during an MMRaccess while the TRU is selected. | Address Error Status. The TRU_STAT.ADDRERR bit is set when an invalid address is provided for an MMRaccess while the TRU is selected. Writing a one to this bit clears the error indication. The TRU_ERRADDR register also is updated when an address error occurs during an MMRaccess while the TRU is selected. |
| 1 (R/W1C)          | ADDRERR    | 0                                                                                                                                                                                                                                                                                                                 | No error                                                                                                                                                                                                                                                                                                          |
| 0 (R/W1C)          | LWERR      | Lock Write Error Status. If TRU_STAT.LWERR is set, a lock write error has occurred. Writing a one to this bit clears the error indication.                                                                                                                                                                        | Lock Write Error Status. If TRU_STAT.LWERR is set, a lock write error has occurred. Writing a one to this bit clears the error indication.                                                                                                                                                                        |
| 0 (R/W1C)          | LWERR      | 0                                                                                                                                                                                                                                                                                                                 | No error                                                                                                                                                                                                                                                                                                          |
| 0 (R/W1C)          | LWERR      | 1                                                                                                                                                                                                                                                                                                                 | Error occurred                                                                                                                                                                                                                                                                                                    |