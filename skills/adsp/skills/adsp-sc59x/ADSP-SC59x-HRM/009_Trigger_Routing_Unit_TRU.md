# Trigger Routing Unit (TRU)

<!-- source: 009_Trigger_Routing_Unit_TRU.pdf | original pages 307–332 -->

## 7   Trigger Routing Unit (TRU)

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

## ADSP-SC59x TRU Register List

The Trigger Routing Unit (TRU) provides simple sequence control of distributed modules without the penalties associated with core intervention (for example, interrupt overhead). The TRU receives trigger inputs from all trigger generator inputs (MTI) and the TRU trigger receiver register ( TRU\_MTR ). Based on these inputs, the TRU logic generates trigger outputs that initiate receiver operations in the processor core and peripherals. A set of registers governs TRU operations. For more information on TRU functionality, see the TRU register descriptions.

Table 7-1: ADSP-SC59x TRU Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRU_ERRADDR | Error Address Register      |
| TRU_GCTL    | Global Control Register     |
| TRU_MTR     | Generator Trigger Register  |
| TRU_SSR[n]  | Receiver Select Register    |
| TRU_STAT    | Status Information Register |

## ADSP-SC59x TRU Interrupt List

Table 7-2: ADSP-SC59x TRU Interrupt List

|   Interrupt ID | Name       | Description                | Sensitivity   | DMA Channel   |
|----------------|------------|----------------------------|---------------|---------------|
|            242 | TRU0_SLV4  | TRU0 Interrupt 4 - Core 1  | Edge          |               |
|            243 | TRU0_SLV5  | TRU0 Interrupt 5 - Core 1  | Edge          |               |
|            244 | TRU0_SLV6  | TRU0 Interrupt 6 - Core 1  | Edge          |               |
|            245 | TRU0_SLV7  | TRU0 Interrupt 7 - Core 1  | Edge          |               |
|            246 | TRU0_SLV8  | TRU0 Interrupt 8 - Core 2  | Edge          |               |
|            247 | TRU0_SLV9  | TRU0 Interrupt 9 - Core 2  | Edge          |               |
|            248 | TRU0_SLV10 | TRU0 Interrupt 10 - Core 2 | Edge          |               |
|            249 | TRU0_SLV11 | TRU0 Interrupt 11 - Core 2 | Edge          |               |
|            334 | TRU0_SLV0  | TRU0 Interrupt 0 - Core 0  | Edge          |               |
|            335 | TRU0_SLV1  | TRU0 Interrupt 1 - Core 0  | Edge          |               |
|            336 | TRU0_SLV2  | TRU0 Interrupt 2 - Core 0  | Edge          |               |
|            337 | TRU0_SLV3  | TRU0 Interrupt 3 - Core 0  | Edge          |               |

## ADSP-SC59x Trigger List

Table 7-3: ADSP-SC59x Trigger List Generators

|   Trigger ID | Name            | Description                                  | Sensitivity   |
|--------------|-----------------|----------------------------------------------|---------------|
|            0 |                 | Reserved                                     |               |
|            1 | CGU0_EVT        | CGU0 Event                                   | Edge          |
|            2 | CGU1_EVT        | CGU1 Event                                   | Edge          |
|            3 | CNT0_STAT       | CNT0 Status                                  | Level         |
|            4 | CNT0_UD         | CNT0 CNT0 Count Up and Direction             | Level         |
|            5 | CNT0_DG         | CNT0 CNT0 Count Down and Gate                | Level         |
|            6 | CNT0_TO         | CNT0 CNT0 Output to Timer Block              | Level         |
|            7 | C1_SID_ACK      | Core 1 System Interface Disable Acknowledge  |               |
|            8 | C0_EVENTIACK    | Core 0 Event Input Request Acknowledge       |               |
|            9 | C0_EVENTOREQ    | Core 0 Event Output Request                  |               |
|           10 | C2_SID_ACK      | C2 System Interface Disable Acknowledge      |               |
|           11 | MDMA0_SRC       | Enhanced BWDMAChannel 0 Source (CRC IN)      |               |
|           12 | MDMA0_DST       | Enhanced BWDMAChannel 0 Source (CRC OUT)     |               |
|           13 | MDMA1_SRC       | Enhanced BWDMAChannel 0 Source (CRC IN)      |               |
|           14 | MDMA1_DST       | Enhanced BWDMAChannel 0 Source (CRC OUT)     |               |
|           15 | MDMA4_SRC       | Standard BWDMAChannel 0 Source (CRC IN)      |               |
|           16 | MDMA4_DST       | Standard BWDMAChannel 0 Source (CRC OUT)     |               |
|           17 | MDMA5_SRC       | Standard BWDMAChannel 0 Source (CRC IN)      |               |
|           18 | MDMA5_DST       | Standard BWDMAChannel 0 Source (CRC OUT)     |               |
|           19 | SOC600CTI0_MST0 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 0 | Edge          |
|           20 | SOC600CTI0_MST1 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 1 | Edge          |
|           21 | SOC600CTI0_MST2 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 2 | Edge          |

Table 7-3: ADSP-SC59x Trigger List Generators (Continued)

|   Trigger ID | Name            | Description                                  | Sensitivity   |
|--------------|-----------------|----------------------------------------------|---------------|
|           22 | SOC600CTI0_MST3 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 3 | Edge          |
|           23 | SOC600CTI0_MST4 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 4 | Edge          |
|           24 | SOC600CTI0_MST5 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 5 | Edge          |
|           25 | SOC600CTI0_MST6 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 6 | Edge          |
|           26 | SOC600CTI0_MST7 | SOC600CTI0 SYSCTI (CTI3) System Halt Slave 7 | Edge          |
|           27 | EMDMA0_DONE     | EMDMA0 DMADone                               | Edge          |
|           28 | EMDMA1_DONE     | EMDMA1 DMADone                               | Edge          |
|           29 | EPPI0_CH0_DMA   | EPPI0 Channel0DMA                            | Edge          |
|           30 | EPPI0_CH1_DMA   | EPPI0 Channel1DMA                            | Edge          |
|           31 | C1_FIR0_DMA     | FIR0 Core1DMA                                | Edge          |
|           32 | C2_FIR0_DMA     | FIR1 Core2DMA                                | Edge          |
|           33 | HADC0_EOC       | HADC0 HADC0 End of Conversion                | Edge          |
|           34 | C1_IIR0_DMA     | IIR0 Core1DMA                                | Edge          |
|           35 | C1_IIR1_DMA     | IIR1 Core1DMA                                | Edge          |
|           36 | C1_IIR2_DMA     | IIR2 Core1DMA                                | Edge          |
|           37 | C1_IIR3_DMA     | IIR3 Core1DMA                                | Edge          |
|           38 | C2_IIR0_DMA     | IIR4 Core2DMA                                | Edge          |
|           39 | C2_IIR1_DMA     | IIR5 Core2DMA                                | Edge          |
|           40 | C2_IIR2_DMA     | IIR6 Core2DMA                                | Edge          |
|           41 | C2_IIR3_DMA     | IIR7 Core2DMA                                | Edge          |
|           42 | L2CTL0_EVT      | L2CTL0 L2 Memory Event                       | Level         |
|           43 | LP0_DMA         | LP0 DMAChannel                               |               |
|           44 | LP1_DMA         | LP1 DMAChannel                               |               |
|           45 | MDMA2_SRC       | Enh BWDMAChannel 0                           |               |
|           46 | MDMA2_DST       | Enh BWDMAChannel 1                           |               |
|           47 | MDMA3_SRC       | Max BWDMAChannel 0                           |               |
|           48 | MDMA3_DST       | Max BWDMAChannel 1                           |               |
|           49 | MDMA6_SRC       | Enh BWDMAChannel 0                           |               |

Table 7-3: ADSP-SC59x Trigger List Generators (Continued)

|   Trigger ID | Name        | Description                         | Sensitivity   |
|--------------|-------------|-------------------------------------|---------------|
|           50 | MDMA6_DST   | Enh BWDMAChannel 1                  |               |
|           51 | MDMA7_SRC   | Max BWDMAChannel 0                  |               |
|           52 | MDMA7_DST   | Max BWDMAChannel 1                  |               |
|           53 | MEC0_EEIRQ0 | MEC0 ECC Error Interrupt Request    | Level         |
|           54 | MEC0_MEIRQ0 | MEC0 Parity Error Interrupt Request | Level         |
|           55 | MEC0_PEIRQ1 | MEC0 Parity Error Interrupt Request | Level         |
|           56 | MEC0_PEIRQ2 | MEC0 Parity Error Interrupt Request | Level         |
|           57 | MEC0_PEIRQ3 | MEC0 Parity Error Interrupt Request | Level         |
|           58 | MEC1_EEIRQ0 | MEC1 ECC Error Interrupt Request    | Level         |
|           59 | MEC1_MEIRQ0 | MEC1 Parity Error Interrupt Request | Level         |
|           60 | MEC1_PEIRQ1 | MEC1 Parity Error Interrupt Request | Level         |
|           61 | MEC1_PEIRQ2 | MEC1 Parity Error Interrupt Request | Level         |
|           62 | MEC1_PEIRQ3 | MEC1 Parity Error Interrupt Request | Level         |
|           63 | MEC2_EEIRQ0 | MEC2 ECC Error Interrupt Request    | Level         |
|           64 | MEC2_MEIRQ0 | MEC2 Parity Error Interrupt Request | Level         |
|           65 | MEC2_PEIRQ1 | MEC2 Parity Error Interrupt Request | Level         |
|           66 | MEC2_PEIRQ2 | MEC2 Parity Error Interrupt Request | Level         |
|           67 | MEC2_PEIRQ3 | MEC2 Parity Error Interrupt Request | Level         |
|           68 | PINT0_BLOCK | PINT0 Pin Interrupt Block           | Level         |
|           69 | PINT1_BLOCK | PINT1 Pin Interrupt Block           | Level         |
|           70 | PINT2_BLOCK | PINT2 Pin Interrupt Block           | Level         |
|           71 | PINT3_BLOCK | PINT3 Pin Interrupt Block           | Level         |
|           72 | PINT4_BLOCK | PINT4 Pin Interrupt Block           | Level         |
|           73 | PINT5_BLOCK | PINT5 Pin Interrupt Block           | Level         |
|           74 | PINT6_BLOCK | PINT6 Pin Interrupt Block           | Level         |
|           75 | PINT7_BLOCK | PINT7 Pin Interrupt Block           | Level         |
|           76 | SEC0_FAULT  | SEC0 Fault                          | Edge          |
|           77 | SPI0_TXDMA  | SPI0 TX DMAChannel                  | Edge          |
|           78 | SPI0_RXDMA  | SPI0 RX DMAChannel                  | Edge          |
|           79 | SPI1_TXDMA  | SPI1 TX DMAChannel                  | Edge          |
|           80 | SPI1_RXDMA  | SPI1 RX DMAChannel                  | Edge          |

Table 7-3: ADSP-SC59x Trigger List Generators (Continued)

|   Trigger ID | Name                   | Description                      | Sensitivity   |
|--------------|------------------------|----------------------------------|---------------|
|           81 | SPI3_TXDMA             | SPI3 TX DMAChannel               | Edge          |
|           82 | SPI3_RXDMA             | SPI3 RX DMAChannel               | Edge          |
|           83 | SPI2_TXDMA             | SPI2 TX DMAChannel               | Edge          |
|           84 | SPI2_RXDMA             | SPI2 RX DMAChannel               | Edge          |
|           85 | SPORT0_A_DMA           | SPORT0 ChannelADMA               | Edge          |
|           86 | SPORT0_B_DMA           | SPORT0 ChannelBDMA               | Edge          |
|           87 | SPORT1_A_DMA           | SPORT1 ChannelADMA               | Edge          |
|           88 | SPORT1_B_DMA           | SPORT1 ChannelBDMA               | Edge          |
|           89 | SPORT2_A_DMA           | SPORT2 ChannelADMA               | Edge          |
|           90 | SPORT2_B_DMA           | SPORT2 ChannelBDMA               | Edge          |
|           91 | SPORT3_A_DMA           | SPORT3 ChannelADMA               | Edge          |
|           92 | SPORT3_B_DMA           | SPORT3 ChannelBDMA               | Edge          |
|           93 | SPORT4_A_DMA           | SPORT4 ChannelADMA               | Edge          |
|           94 | SPORT4_B_DMA           | SPORT4 ChannelBDMA               | Edge          |
|           95 | SPORT5_A_DMA           | SPORT5 ChannelADMA               | Edge          |
|           96 | SPORT5_B_DMA           | SPORT5 ChannelBDMA               | Edge          |
|           97 | SPORT6_A_DMA           | SPORT6 ChannelADMA               | Edge          |
|           98 | SPORT6_B_DMA           | SPORT6 ChannelBDMA               | Edge          |
|           99 | SPORT7_A_DMA           | SPORT7 ChannelADMA               | Edge          |
|          100 | SPORT7_B_DMA           | SPORT7 ChannelBDMA               | Edge          |
|          101 | DAI0_GBL_SPORT_TRG_ O0 | DAI0 SPORT GROUP0 Trigger Output | None          |
|          102 | DAI0_GBL_SPORT_TRG_ O1 | DAI0 SPORT GROUP1 Trigger Output | None          |
|          103 | DAI1_GBL_SPORT_TRG_ O0 | DAI1 SPORT GROUP2 Trigger Output | None          |
|          104 | DAI1_GBL_SPORT_TRG_ O1 | DAI1 SPORT Group3 Trigger Output | None          |
|          106 | SWU1_EVT               | SWU1 Event                       | None          |
|          107 | SWU2_EVT               | SWU2 Event                       | None          |
|          108 | SWU7_EVT               | SWU7 Event                       | None          |
|          109 | SWU8_EVT               | SWU8 Event                       | None          |

Table 7-3: ADSP-SC59x Trigger List Generators (Continued)

|   Trigger ID | Name             | Description               | Sensitivity   |
|--------------|------------------|---------------------------|---------------|
|          110 | SWU9_EVT         | SWU9 Event                | None          |
|          111 | SWU10_EVT        | SWU10 Event               | None          |
|          112 | SWU11_EVT        | SWU11 Event               | None          |
|          113 | SWU12_EVT        | SWU12 Event               | None          |
|          114 | SWU13_EVT        | SWU13 Event               | None          |
|          115 | SWU3_EVT         | SWU3 Event                | None          |
|          116 | SWU4_EVT         | SWU4 Event                | None          |
|          117 | SWU5_EVT         | SWU5 Event                | None          |
|          119 | SWU1_DBG         | SWU1 Debug                | Edge          |
|          120 | SWU2_DBG         | SWU2 Debug                | Edge          |
|          121 | SWU7_DBG         | SWU7 Debug                | Edge          |
|          122 | SWU8_DBG         | SWU8 Debug                | Edge          |
|          123 | SWU9_DBG         | SWU9 Debug                | Edge          |
|          124 | SWU10_DBG        | SWU10 Debug               | Edge          |
|          125 | SWU11_DBG        | SWU11 Debug               | Edge          |
|          126 | SWU12_DBG        | SWU12 Debug               | Edge          |
|          127 | SWU13_DBG        | SWU13 Debug               | Edge          |
|          128 | SWU3_DBG         | SWU3 Debug                | Edge          |
|          129 | SWU4_DBG         | SWU4 Debug                | Edge          |
|          130 | SWU5_DBG         | SWU5 Debug                | Edge          |
|          131 | SOFT0_MST        | Software-driven Trigger 0 |               |
|          132 | SOFT1_MST        | Software-driven Trigger 1 |               |
|          133 | SOFT2_MST        | Software-driven Trigger 2 |               |
|          134 | SOFT3_MST        | Software-driven Trigger 3 |               |
|          135 | SOFT4_MST        | Software-driven Trigger 4 |               |
|          136 | SOFT5_MST        | Software-driven Trigger 5 |               |
|          137 | TIMER0_TMR00_MST | TIMER0 Timer 0            | Edge          |
|          138 | TIMER0_TMR01_MST | TIMER0 Timer 1            | Edge          |
|          139 | TIMER0_TMR02_MST | TIMER0 Timer 2            | Edge          |
|          140 | TIMER0_TMR03_MST | TIMER0 Timer 3            | Edge          |
|          141 | TIMER0_TMR04_MST | TIMER0 Timer 4            | Edge          |

Table 7-3: ADSP-SC59x Trigger List Generators (Continued)

|   Trigger ID | Name               | Description                                             | Sensitivity   |
|--------------|--------------------|---------------------------------------------------------|---------------|
|          142 | TIMER0_TMR05_MST   | TIMER0 Timer 5                                          | Edge          |
|          143 | TIMER0_TMR06_MST   | TIMER0 Timer 6                                          | Edge          |
|          144 | TIMER0_TMR07_MST   | TIMER0 Timer 7                                          | Edge          |
|          145 | TIMER0_TMR08_MST   | TIMER0 Timer 8                                          | Edge          |
|          146 | TIMER0_TMR09_MST   | TIMER0 Timer 9                                          | Edge          |
|          147 | TIMER0_TMR10_MST   | TIMER0 Timer 10                                         | Edge          |
|          148 | TIMER0_TMR11_MST   | TIMER0 Timer 11                                         | Edge          |
|          149 | TIMER0_TMR12_MST   | TIMER0 Timer 12                                         | Edge          |
|          150 | TIMER0_TMR13_MST   | TIMER0 Timer 13                                         | Edge          |
|          151 | TIMER0_TMR14_MST   | TIMER0 Timer 14                                         | Edge          |
|          152 | TIMER0_TMR15_MST   | TIMER0 Timer 15                                         | Edge          |
|          153 | TMU0_FAULT         | TMU0 TM0 Fault Event                                    |               |
|          154 | TMU0_ALERT         | TMU0 TM0 Alert Event                                    |               |
|          155 | UART0_TXDMA        | UART0 TransmitDMA                                       | Edge          |
|          156 | UART0_RXDMA        | UART0 ReceiveDMA                                        | Edge          |
|          157 | UART1_TXDMA        | UART1 TransmitDMA                                       | Edge          |
|          158 | UART1_RXDMA        | UART1 ReceiveDMA                                        | Edge          |
|          159 | UART2_TXDMA        | UART2 TransmitDMA                                       | Edge          |
|          160 | UART2_RXDMA        | UART2 ReceiveDMA                                        | Edge          |
|          161 | UART3_TXDMA        | UART3 TransmitDMA                                       | Edge          |
|          162 | UART3_RXDMA        | UART3 ReceiveDMA                                        | Edge          |
|          163 | USBC0_SOF_TGL      | USBC0 USB start of frame generation toggle              | None          |
|          164 | USBC0_SOF_SENT_TGL | USBC0 USB start of frame transmission toggle            | None          |
|          165 | USBC0_DMAREQ       | USBC0 USB Internal DMARequest for Remote Memory Support | None          |
|          166 | USBC0_DMADONE      | USBC0 USB Internal DMADone for Remote Memory Support    | None          |
|          167 | WDOG0_EXP          | WDOG0 Expiration                                        | Level         |
|          168 | WDOG1_EXP          | WDOG1 Expiration                                        | Level         |
|          169 | WDOG2_EXP          | WDOG2 Expiration                                        | Level         |
|          170 | CANFD0_IPD_REQ     | CANFD0 CAN0 DMArequest interrupt                        | None          |
|          171 | CANFD1_IPD_REQ     | CANFD1 CAN1 DMArequest interrupt                        | None          |

Table 7-3: ADSP-SC59x Trigger List Generators (Continued)

|   Trigger ID | Name                      | Description                                     | Sensitivity   |
|--------------|---------------------------|-------------------------------------------------|---------------|
|          172 | PCG0_HWA                  | PCG0 PCG-A Hardware trigger control             | None          |
|          173 | PCG0_HWB                  | PCG0 PCG-B Hardware trigger control             | None          |
|          174 | PCG0_HWC                  | PCG0 PCG-C Hardware trigger control             | None          |
|          175 | PCG0_HWD                  | PCG0 PCG-D Hardware trigger control             | None          |
|          176 | PCG0_HWE                  | PCG0 PCG-E Hardware trigger control             | None          |
|          177 | PCG0_HWF                  | PCG0 PCG-F Hardware trigger control             | None          |
|          178 | PCG0_HWG                  | PCG0 PCG-G Hardware trigger control             | None          |
|          179 | PCG0_HWH                  | PCG0 PCG-H Hardware trigger control             | None          |
|          180 | EMAC0_MCGR_DMA_REQ0       | EMAC0 EMAC0 Media Clock Generation Re- covery 0 | Edge          |
|          181 | EMAC0_MCGR_DMA_REQ1       | EMAC0 EMAC0 Media Clock Generation Re- covery 1 | Edge          |
|          182 | EMAC0_MCGR_DMA_REQ2       | EMAC0 EMAC0 Media Clock Generation Re- covery 2 | Edge          |
|          183 | EMAC0_MCGR_DMA_REQ3       | EMAC0 EMAC0 Media Clock Generation Re- covery 3 | Edge          |
|          184 | EMAC0_DMA0_RX             | EMAC0 EMAC0 DMA0 Rx                             | None          |
|          185 | EMAC0_DMA1_RX             | EMAC0 EMAC0 DMA1 Rx                             | None          |
|          186 | EMAC0_DMA2_RX             | EMAC0 EMAC0 DMA2 Rx                             | None          |
|          187 | EMAC0_DMA3_RX             | EMAC0 EMAC0 DMA3 Rx                             | None          |
|          188 | EMAC0_DMA4_RX             | EMAC0 EMAC0 DMA4 Rx                             | None          |
|          189 | EMAC0_DMA5_RX             | EMAC0 EMAC0 DMA5 Rx                             | None          |
|          190 | EMAC0_DMA6_RX             | EMAC0 EMAC0 DMA6 Rx                             | None          |
|          191 | EMAC0_DMA7_RX             | EMAC0 EMAC0 DMA7 Rx                             | None          |
|          192 | EMAC1_DMA0_RX             | EMAC1 EMAC1 DMA0 Rx                             | None          |
|          193 | A55MISC0_COREP_ ACCEPT0   | A55MISC0 A55 PREQ ACCEPT                        | Edge          |
|          194 | A55MISC0_COREP_ DENY0     | A55MISC0 A55 PREQ DENY                          | Edge          |
|          195 | A55MISC0_CLUSTERP_ ACCEPT | A55MISC0 CLUSTER PREQ ACCEPT                    | Edge          |
|          196 | A55MISC0_CLUSTERP_ DENY   | A55MISC0 CLUSTER PREQ DENY                      | Edge          |

Table 7-4: ADSP-SC59x Trigger List Receivers

|   Trigger ID | Name            | Description                                   | Sensitivity   |
|--------------|-----------------|-----------------------------------------------|---------------|
|            0 | CNT0_UD         | CNT0 CNT0 Count Up and Direction              | Pulse         |
|            1 | CNT0_DG         | CNT0 CNT0 Count Down and Gate                 | Pulse         |
|            2 | C0_EVENTIREQ    | Core 0 Event Input for Wakeup from WFE State  | Pulse         |
|            3 | C0_EVENTOACK    | Core 0 Event Output Acknowledgment for Wakeup | Pulse         |
|            4 | MDMA0_SRC       | Enhanced BWDMAChannel 0 Source (CRC IN)       | Pulse         |
|            5 | MDMA0_DST       | Enhanced BWDMAChannel 0 Source (CRC OUT)      | Pulse         |
|            6 | MDMA1_SRC       | Enhanced BWDMAChannel 0 Source (CRC IN)       | Pulse         |
|            7 | MDMA1_DST       | Enhanced BWDMAChannel 0 Source (CRC OUT)      | Pulse         |
|            8 | MDMA4_SRC       | Standard BWDMAChannel 0 Source (CRC IN)       | Pulse         |
|            9 | MDMA4_DST       | Standard BWDMAChannel 0 Source (CRC OUT)      | Pulse         |
|           10 | MDMA5_SRC       | Standard BWDMAChannel 1 Source (CRC IN)       | Pulse         |
|           11 | MDMA5_DST       | Standard BWDMAChannel 1 Source (CRC OUT)      | Pulse         |
|           12 | SOC600CTI0_SLV0 | SOC600CTI0 SYSCTI System Halt Master 0        | Pulse         |
|           13 | SOC600CTI0_SLV1 | SOC600CTI0 SYSCTI System Halt Master 1        | Pulse         |
|           14 | SOC600CTI0_SLV2 | SOC600CTI0 SYSCTI System Halt Master 2        | Pulse         |
|           15 | SOC600CTI0_SLV3 | SOC600CTI0 SYSCTI System Halt Master 3        | Pulse         |
|           16 | SOC600CTI0_SLV4 | SOC600CTI0 SYSCTI System Halt Master 4        | Pulse         |
|           17 | SOC600CTI0_SLV5 | SOC600CTI0 SYSCTI System Halt Master 5        | Pulse         |
|           18 | SOC600CTI0_SLV6 | SOC600CTI0 SYSCTI System Halt Master 6        | Pulse         |
|           19 | EPPI0_CH0_DMA   | EPPI0 Channel0DMA                             | Pulse         |
|           20 | EPPI0_CH1_DMA   | EPPI0 Channel1DMA                             | Pulse         |
|           21 | C1_FIR0_TRGI    | FIR0 Core 1 FIR Wait on Trigger Input         | Pulse         |
|           22 | C2_FIR0_TRGI    | FIR1 Core 2 FIR Wait on Trigger Input         | Pulse         |
|           23 | C1_IIR0_TRGI    | IIR0 Core 1 IIR Wait on Trigger Input         | Pulse         |
|           24 | C1_IIR1_TRGI    | IIR1 Core 1 IIR Wait on Trigger Input         | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name         | Description                           | Sensitivity   |
|--------------|--------------|---------------------------------------|---------------|
|           25 | C1_IIR2_TRGI | IIR2 Core 1 IIR Wait on Trigger Input | Pulse         |
|           26 | C1_IIR3_TRGI | IIR3 Core 1 IIR Wait on Trigger Input | Pulse         |
|           27 | C2_IIR0_TRGI | IIR4 Core 2 IIR Wait on Trigger Input | Pulse         |
|           28 | C2_IIR1_TRGI | IIR5 Core 2 IIR Wait on Trigger Input | Pulse         |
|           29 | C2_IIR2_TRGI | IIR6 Core 2 IIR Wait on Trigger Input | Pulse         |
|           30 | C2_IIR3_TRGI | IIR7 Core 2 IIR Wait on Trigger Input | Pulse         |
|           31 | LP0_DMA      | LP0 DMAChannel                        | Pulse         |
|           32 | LP1_DMA      | LP1 DMAChannel                        | Pulse         |
|           33 | MDMA2_SRC    | Enh BWDMAChannel 0                    | Pulse         |
|           34 | MDMA2_DST    | Enh BWDMAChannel 1                    | Pulse         |
|           35 | MDMA3_SRC    | Max BWDMAChannel 0                    | Pulse         |
|           36 | MDMA3_DST    | Max BWDMAChannel 1                    | Pulse         |
|           37 | MDMA6_SRC    | Enh BWDMAChannel 0                    | Pulse         |
|           38 | MDMA6_DST    | Enh BWDMAChannel 1                    | Pulse         |
|           39 | MDMA7_SRC    | Max BWDMAChannel 0                    | Pulse         |
|           40 | MDMA7_DST    | Max BWDMAChannel 1                    | Pulse         |
|           41 | PORTA_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           42 | PORTB_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           43 | PORTC_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           44 | PORTD_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           45 | PORTE_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           46 | PORTF_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           47 | PORTG_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           48 | PORTH_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           49 | PORTI_TOGGLE | Port Toggle Trigger                   | Pulse         |
|           50 | RCU0_SYSRST0 | RCU0 System Reset Slave 0             | Pulse         |
|           51 | RCU0_SYSRST1 | RCU0 System Reset Slave 1             | Pulse         |
|           52 | SPI0_TXDMA   | SPI0 TX DMAChannel                    | Pulse         |
|           53 | SPI0_RXDMA   | SPI0 RX DMAChannel                    | Pulse         |
|           54 | SPI1_TXDMA   | SPI1 TX DMAChannel                    | Pulse         |
|           55 | SPI1_RXDMA   | SPI1 RX DMAChannel                    | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name                   | Description                     | Sensitivity   |
|--------------|------------------------|---------------------------------|---------------|
|           56 | SPI3_TXDMA             | SPI3 TX DMAChannel              | Pulse         |
|           57 | SPI3_RXDMA             | SPI3 RX DMAChannel              | Pulse         |
|           58 | SPI2_TXDMA             | SPI2 TX DMAChannel              | Pulse         |
|           59 | SPI2_RXDMA             | SPI2 RX DMAChannel              | Pulse         |
|           60 | SPORT0_A_DMA           | SPORT0 ChannelADMA              | Pulse         |
|           61 | SPORT0_B_DMA           | SPORT0 ChannelBDMA              | Pulse         |
|           62 | SPORT1_A_DMA           | SPORT1 ChannelADMA              | Pulse         |
|           63 | SPORT1_B_DMA           | SPORT1 ChannelBDMA              | Pulse         |
|           64 | SPORT2_A_DMA           | SPORT2 ChannelADMA              | Pulse         |
|           65 | SPORT2_B_DMA           | SPORT2 ChannelBDMA              | Pulse         |
|           66 | SPORT3_A_DMA           | SPORT3 ChannelADMA              | Pulse         |
|           67 | SPORT3_B_DMA           | SPORT3 ChannelBDMA              | Pulse         |
|           68 | SPORT4_A_DMA           | SPORT4 ChannelADMA              | Pulse         |
|           69 | SPORT4_B_DMA           | SPORT4 ChannelBDMA              | Pulse         |
|           70 | SPORT5_A_DMA           | SPORT5 ChannelADMA              | Pulse         |
|           71 | SPORT5_B_DMA           | SPORT5 ChannelBDMA              | Pulse         |
|           72 | SPORT6_A_DMA           | SPORT6 ChannelADMA              | Pulse         |
|           73 | SPORT6_B_DMA           | SPORT6 ChannelBDMA              | Pulse         |
|           74 | SPORT7_A_DMA           | SPORT7 ChannelADMA              | Pulse         |
|           75 | SPORT7_B_DMA           | SPORT7 ChannelBDMA              | Pulse         |
|           76 | DAI0_GBL_SPORT_TRG_ I0 | DAI0 SPORT GROUP0 Trigger Input | Pulse         |
|           77 | DAI0_GBL_SPORT_TRG_ I1 | DAI0 SPORT GROUP1 Trigger Input | Pulse         |
|           78 | DAI1_GBL_SPORT_TRG_ I0 | DAI1 SPORT GROUP2 Trigger Input | Pulse         |
|           79 | DAI1_GBL_SPORT_TRG_ I1 | DAI1 SPORT GROUP3 Trigger Input | Pulse         |
|           80 | STM0_EVT0              | STM0 STM0 Event 0               | Pulse         |
|           81 | STM0_EVT1              | STM0 STM0 Event 1               | Pulse         |
|           82 | STM0_EVT2              | STM0 STM0 Event 2               | Pulse         |
|           83 | STM0_EVT3              | STM0 STM0 Event 3               | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|           84 | STM0_EVT4  | STM0 STM0 Event 4  | Pulse         |
|           85 | STM0_EVT5  | STM0 STM0 Event 5  | Pulse         |
|           86 | STM0_EVT6  | STM0 STM0 Event 6  | Pulse         |
|           87 | STM0_EVT7  | STM0 STM0 Event 7  | Pulse         |
|           88 | STM0_EVT8  | STM0 STM0 Event 8  | Pulse         |
|           89 | STM0_EVT9  | STM0 STM0 Event 9  | Pulse         |
|           90 | STM0_EVT10 | STM0 STM0 Event 10 | Pulse         |
|           91 | STM0_EVT11 | STM0 STM0 Event 11 | Pulse         |
|           92 | STM0_EVT12 | STM0 STM0 Event 12 | Pulse         |
|           93 | STM0_EVT13 | STM0 STM0 Event 13 | Pulse         |
|           94 | STM0_EVT14 | STM0 STM0 Event 14 | Pulse         |
|           95 | STM0_EVT15 | STM0 STM0 Event 15 | Pulse         |
|           96 | STM0_EVT16 | STM0 STM0 Event 16 | Pulse         |
|           97 | STM0_EVT17 | STM0 STM0 Event 17 | Pulse         |
|           98 | STM0_EVT18 | STM0 STM0 Event 18 | Pulse         |
|           99 | STM0_EVT19 | STM0 STM0 Event 19 | Pulse         |
|          100 | STM0_EVT20 | STM0 STM0 Event 20 | Pulse         |
|          101 | STM0_EVT21 | STM0 STM0 Event 21 | Pulse         |
|          102 | STM0_EVT22 | STM0 STM0 Event 22 | Pulse         |
|          103 | STM0_EVT23 | STM0 STM0 Event 23 | Pulse         |
|          104 | STM0_EVT24 | STM0 STM0 Event 24 | Pulse         |
|          105 | STM0_EVT25 | STM0 STM0 Event 25 | Pulse         |
|          106 | STM0_EVT26 | STM0 STM0 Event 26 | Pulse         |
|          107 | STM0_EVT27 | STM0 STM0 Event 27 | Pulse         |
|          108 | STM0_EVT28 | STM0 STM0 Event 28 | Pulse         |
|          109 | STM0_EVT29 | STM0 STM0 Event 29 | Pulse         |
|          110 | STM0_EVT30 | STM0 STM0 Event 30 | Pulse         |
|          111 | STM0_EVT31 | STM0 STM0 Event 31 | Pulse         |
|          113 | SWU1_EN    | SWU1 Enable        | Pulse         |
|          114 | SWU2_EN    | SWU2 Enable        | Pulse         |
|          115 | SWU7_EN    | SWU7 Enable        | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name              | Description             | Sensitivity   |
|--------------|-------------------|-------------------------|---------------|
|          116 | SWU8_EN           | SWU8 Enable             | Pulse         |
|          117 | SWU9_EN           | SWU9 Enable             | Pulse         |
|          118 | SWU10_EN          | SWU10 Enable            | Pulse         |
|          119 | SWU11_EN          | SWU11 Enable            | Pulse         |
|          120 | SWU12_EN          | SWU12 Enable            | Pulse         |
|          121 | SWU13_EN          | SWU13 Enable            | Pulse         |
|          122 | SWU3_EN           | SWU3 Enable Event CL2_1 | Pulse         |
|          123 | SWU4_EN           | SWU4 Enable Event DL2_1 | Pulse         |
|          124 | SWU5_EN           | SWU5 Enable Event CL2_2 | Pulse         |
|          125 | TIMER0_TMR00_SLV0 | TIMER0 Timer 0          | Pulse         |
|          126 | TIMER0_TMR00_SLV1 | TIMER0 Timer 0          | Pulse         |
|          127 | TIMER0_TMR01_SLV0 | TIMER0 Timer 1          | Pulse         |
|          128 | TIMER0_TMR01_SLV1 | TIMER0 Timer 1          | Pulse         |
|          129 | TIMER0_TMR02_SLV0 | TIMER0 Timer 2          | Pulse         |
|          130 | TIMER0_TMR02_SLV1 | TIMER0 Timer 2          | Pulse         |
|          131 | TIMER0_TMR03_SLV0 | TIMER0 Timer 3          | Pulse         |
|          132 | TIMER0_TMR03_SLV1 | TIMER0 Timer 3          | Pulse         |
|          133 | TIMER0_TMR04_SLV0 | TIMER0 Timer 4          | Pulse         |
|          134 | TIMER0_TMR04_SLV1 | TIMER0 Timer 4          | Pulse         |
|          135 | TIMER0_TMR05_SLV0 | TIMER0 Timer 5          | Pulse         |
|          136 | TIMER0_TMR05_SLV1 | TIMER0 Timer 5          | Pulse         |
|          137 | TIMER0_TMR06_SLV0 | TIMER0 Timer 6          | Pulse         |
|          138 | TIMER0_TMR06_SLV1 | TIMER0 Timer 6          | Pulse         |
|          139 | TIMER0_TMR07_SLV0 | TIMER0 Timer 7          | Pulse         |
|          140 | TIMER0_TMR07_SLV1 | TIMER0 Timer 7          | Pulse         |
|          141 | TIMER0_TMR08_SLV0 | TIMER0 Timer 8          | Pulse         |
|          142 | TIMER0_TMR08_SLV1 | TIMER0 Timer 8          | Pulse         |
|          143 | TIMER0_TMR09_SLV0 | TIMER0 Timer 9          | Pulse         |
|          144 | TIMER0_TMR09_SLV1 | TIMER0 Timer 9          | Pulse         |
|          145 | TIMER0_TMR10_SLV0 | TIMER0 Timer 10         | Pulse         |
|          146 | TIMER0_TMR10_SLV1 | TIMER0 Timer 10         | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name              | Description                         | Sensitivity   |
|--------------|-------------------|-------------------------------------|---------------|
|          147 | TIMER0_TMR11_SLV0 | TIMER0 Timer 11                     | Pulse         |
|          148 | TIMER0_TMR11_SLV1 | TIMER0 Timer 11                     | Pulse         |
|          149 | TIMER0_TMR12_SLV0 | TIMER0 Timer 12                     | Pulse         |
|          150 | TIMER0_TMR12_SLV1 | TIMER0 Timer 12                     | Pulse         |
|          151 | TIMER0_TMR13_SLV0 | TIMER0 Timer 13                     | Pulse         |
|          152 | TIMER0_TMR13_SLV1 | TIMER0 Timer 13                     | Pulse         |
|          153 | TIMER0_TMR14_SLV0 | TIMER0 Timer 14                     | Pulse         |
|          154 | TIMER0_TMR14_SLV1 | TIMER0 Timer 14                     | Pulse         |
|          155 | TIMER0_TMR15_SLV0 | TIMER0 Timer 15                     | Pulse         |
|          156 | TIMER0_TMR15_SLV1 | TIMER0 Timer 15                     | Pulse         |
|          157 | TRU0_SLV0         | TRU0 Interrupt Request 0            | Pulse         |
|          158 | TRU0_SLV1         | TRU0 Interrupt Request 1            | Pulse         |
|          159 | TRU0_SLV2         | TRU0 Interrupt Request 2            | Pulse         |
|          160 | TRU0_SLV3         | TRU0 Interrupt Request 3            | Pulse         |
|          161 | TRU0_SLV4         | TRU0 Interrupt Request 4            | Pulse         |
|          162 | TRU0_SLV5         | TRU0 Interrupt Request 5            | Pulse         |
|          163 | TRU0_SLV6         | TRU0 Interrupt Request 6            | Pulse         |
|          164 | TRU0_SLV7         | TRU0 Interrupt Request 7            | Pulse         |
|          165 | TRU0_SLV8         | TRU0 Interrupt Request 8            | Pulse         |
|          166 | TRU0_SLV9         | TRU0 Interrupt Request 9            | Pulse         |
|          167 | TRU0_SLV10        | TRU0 Interrupt Request 10           | Pulse         |
|          168 | TRU0_SLV11        | TRU0 Interrupt Request 11           | Pulse         |
|          169 | UART0_TXDMA       | UART0 TransmitDMA                   | Pulse         |
|          170 | UART0_RXDMA       | UART0 ReceiveDMA                    | Pulse         |
|          171 | UART1_TXDMA       | UART1 TransmitDMA                   | Pulse         |
|          172 | UART1_RXDMA       | UART1 ReceiveDMA                    | Pulse         |
|          173 | UART2_TXDMA       | UART2 TransmitDMA                   | Pulse         |
|          174 | UART2_RXDMA       | UART2 ReceiveDMA                    | Pulse         |
|          175 | UART3_TXDMA       | UART3 TransmitDMA                   | Pulse         |
|          176 | UART3_RXDMA       | UART3 ReceiveDMA                    | Pulse         |
|          177 | PCG0_HWA          | PCG0 PCG-A Hardware trigger control | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name               | Description                         | Sensitivity   |
|--------------|--------------------|-------------------------------------|---------------|
|          178 | PCG0_HWB           | PCG0 PCG-B Hardware trigger control | Pulse         |
|          179 | PCG0_HWC           | PCG0 PCG-C Hardware trigger control | Pulse         |
|          180 | PCG0_HWD           | PCG0 PCG-D Hardware trigger control | Pulse         |
|          181 | PCG0_HWE           | PCG0 PCG-E Hardware trigger control | Pulse         |
|          182 | PCG0_HWF           | PCG0 PCG-F Hardware trigger control | Pulse         |
|          183 | PCG0_HWG           | PCG0 PCG-G Hardware trigger control | Pulse         |
|          184 | PCG0_HWH           | PCG0 PCG-H Hardware trigger control | Pulse         |
|          185 |                    | Reserved                            |               |
|          186 |                    | Reserved                            |               |
|          187 |                    | Reserved                            |               |
|          188 | EMAC0_DMA0_RX      | EMAC0 EMAC0 DMA0 Rx                 | Pulse         |
|          189 | EMAC0_DMA1_RX      | EMAC0 EMAC0 DMA1 Rx                 | Pulse         |
|          190 | EMAC0_DMA2_RX      | EMAC0 EMAC0 DMA2 Rx                 | Pulse         |
|          191 | EMAC0_DMA3_RX      | EMAC0 EMAC0 DMA3 Rx                 | Pulse         |
|          192 | EMAC0_DMA4_RX      | EMAC0 EMAC0 DMA4 Rx                 | Pulse         |
|          193 | EMAC0_DMA5_RX      | EMAC0 EMAC0 DMA5 Rx                 | Pulse         |
|          194 | EMAC0_DMA6_RX      | EMAC0 EMAC0 DMA6 Rx                 | Pulse         |
|          195 | EMAC0_DMA7_RX      | EMAC0 EMAC0 DMA7 Rx                 | Pulse         |
|          196 | EMAC0_DMA0_TX      | EMAC0 EMAC0 DMA0 Tx                 | Pulse         |
|          197 | EMAC0_DMA1_TX      | EMAC0 EMAC0 DMA1 Tx                 | Pulse         |
|          198 | EMAC0_DMA2_TX      | EMAC0 EMAC0 DMA2 Tx                 | Pulse         |
|          199 | EMAC0_DMA3_TX      | EMAC0 EMAC0 DMA3 Tx                 | Pulse         |
|          200 | EMAC0_DMA4_TX      | EMAC0 EMAC0 DMA4 Tx                 | Pulse         |
|          201 | EMAC0_DMA5_TX      | EMAC0 EMAC0 DMA5 Tx                 | Pulse         |
|          202 | EMAC0_DMA6_TX      | EMAC0 EMAC0 DMA6 Tx                 | Pulse         |
|          203 | EMAC0_DMA7_TX      | EMAC0 EMAC0 DMA7 Tx                 | Pulse         |
|          204 | EMAC1_DMA0_RX      | EMAC1 EMAC1 DMA0 Rx                 | Pulse         |
|          205 | EMAC1_DMA0_TX      | EMAC1 EMAC1 DMA0 Tx                 | Pulse         |
|          206 | EMAC0_DMA0_RX_STOP | EMAC0 EMAC0 DMA0 Rx Stop            | Pulse         |
|          207 | EMAC0_DMA1_RX_STOP | EMAC0 EMAC0 DMA1 Rx Stop            | Pulse         |
|          208 | EMAC0_DMA2_RX_STOP | EMAC0 EMAC0 DMA2 Rx Stop            | Pulse         |

Table 7-4: ADSP-SC59x Trigger List Receivers (Continued)

|   Trigger ID | Name                | Description                                         | Sensitivity   |
|--------------|---------------------|-----------------------------------------------------|---------------|
|          209 | EMAC0_DMA3_RX_STOP  | EMAC0 EMAC0 DMA3 Rx Stop                            | Pulse         |
|          210 | EMAC0_DMA4_RX_STOP  | EMAC0 EMAC0 DMA4 Rx Stop                            | Pulse         |
|          211 | EMAC0_DMA5_RX_STOP  | EMAC0 EMAC0 DMA5 Rx Stop                            | Pulse         |
|          212 | EMAC0_DMA6_RX_STOP  | EMAC0 EMAC0 DMA6 Rx Stop                            | Pulse         |
|          213 | EMAC0_DMA7_RX_STOP  | EMAC0 EMAC0 DMA7 Rx Stop                            | Pulse         |
|          214 | EMAC0_DMA0_TX_STOP  | EMAC0 EMAC0 DMA0 Tx Stop                            | Pulse         |
|          215 | EMAC0_DMA1_TX_STOP  | EMAC0 EMAC0 DMA1 Tx Stop                            | Pulse         |
|          216 | EMAC0_DMA2_TX_STOP  | EMAC0 EMAC0 DMA2 Tx Stop                            | Pulse         |
|          217 | EMAC0_DMA3_TX_STOP  | EMAC0 EMAC0 DMA3 Tx Stop                            | Pulse         |
|          218 | EMAC0_DMA4_TX_STOP  | EMAC0 EMAC0 DMA4 Tx Stop                            | Pulse         |
|          219 | EMAC0_DMA5_TX_STOP  | EMAC0 EMAC0 DMA5 Tx Stop                            | Pulse         |
|          220 | EMAC0_DMA6_TX_STOP  | EMAC0 EMAC0 DMA6 Tx Stop                            | Pulse         |
|          221 | EMAC0_DMA7_TX_STOP  | EMAC0 EMAC0 DMA7 Tx Stop                            | Pulse         |
|          222 | EMAC1_DMA0_RX_STOP  | EMAC1 EMAC1 DMA0 Rx Stop                            | Pulse         |
|          223 | EMAC1_DMA0_TX_STOP  | EMAC1 EMAC1 DMA0 Tx Stop                            | Pulse         |
|          224 | EMAC0_MCGR_DMA_ACK0 | EMAC0 EMAC0 Media Clock Generation Re- covery Ack 0 | Pulse         |
|          225 | EMAC0_MCGR_DMA_ACK1 | EMAC0 EMAC0 Media Clock Generation Re- covery Ack 1 | Pulse         |
|          226 | EMAC0_MCGR_DMA_ACK2 | EMAC0 EMAC0 Media Clock Generation Re- covery Ack 2 | Pulse         |
|          227 | EMAC0_MCGR_DMA_ACK3 | EMAC0 EMAC0 Media Clock Generation Re- covery Ack 3 | Pulse         |

## TRU Definitions

The following definitions are helpful when using the TRU module.

## Trigger Generator

A trigger generator is any system module that provides trigger event indication to the TRU. T rigger generator modules define trigger events and conditions for assertion.

## Trigger Generator ID

Trigger generators are assigned a unique numeric ID according to their physical connection to the TRU. Trigger receiver ID 0 is reserved and defined as null.

## Trigger Receiver

A trigger receiver is any system module that receives a trigger event indication from the TRU. T rigger receiver modules define a trigger event response.

NOTE: For peripherals configured as a receiver: when the corresponding peripheral DMA is kept waiting for trigger from a generator, it can receive stale or garbage data. The peripheral receiver begins shifting in data as soon as it is enabled, and it does not wait for the trigger generator. Upon receiving the trigger, this data is moved into the DMA FIFO. Therefore, this sequence can result in the receiver getting unusable data.

## TRU Block Diagram

The trigger generator and the Generator Trigger register (MTR) generate trigger assertions. Each trigger receiver has a dedicated Receiver Select register (SSR) that specifies the unique trigger generator from which it receives the trigger indication.

Figure 7-1: TRU Block Diagram

<!-- image -->

## TRU Architectural Concepts

The TRU supports a simple trigger-in/trigger-out model for modules that comply with the triggering functional model. The TRU is the controller of the trigger system. T rigger outputs from trigger generators are mapped to trigger inputs of trigger receivers through a set of programmable registers ( TRU\_SSR[n] ).

System modules are trigger generator only, trigger receiver only, or trigger generator and trigger receiver.

All of the trigger input and output signals are connected to a trigger routing unit (TRU) which manages the connections of triggers between modules.

In multi-processor systems, multiple TRU units are provided. These TRUs are networked together. Generic Trigger Ports (GTPs) are provided to forward trigger events from one TRU unit to another, forming a pathway from trigger generator to trigger receivers wherever they might lie in the system.

## TRU Programming Model

Implementing sequence control using the TRU requires, at a minimum, proper configuration of a trigger receiver, a trigger generator, and the TRU module itself. The only requirement for the configuration procedure is that the trigger generator is configured and enabled as the last step.

Complete the following other steps:

- Configure the trigger receiver for response to triggers
- Configure the TRU to map the trigger generator to the trigger receiver through the TRU\_SSR[n] registers
- Configure the trigger generator to generate trigger assertions
- Alternatively, use software triggering for trigger assertion. Writing the trigger generator ID to the MTR register generates software triggers.

## Programming Concepts

The following concepts aid in programming the TRU.

- Trigger Sequence Configuration. A simple sequence consists of one trigger generator and one trigger receiver. More complex trigger sequences consist of several trigger receivers functioning as trigger receiver and trigger generator. Additionally, trigger sequences can loopback to the original generator forming a perpetual sequence.
- Software Triggering. Writing a trigger generator ID to the MTR generates a trigger within the TRU from the trigger generator ID specified.
- Synchronization. The TRU can be used to coarsely synchronize events by mapping multiple trigger receivers to the same trigger generator or by generating multiple trigger generator assertions simultaneously through the MTR.
- Configuration Protection. The TRU\_SSR[n].LOCK bit and the TRU\_GCTL.LOCK bit enable register level write-protection when the global lock is asserted in the SPU.

## Programming Examples

The following examples shows the steps to create a single trigger.

## Configuring a Simple Trigger Sequence

The following example shows the steps to create a simple trigger.

1. Write to the TRU\_GCTL register to enable the TRU.
2. Write to the TRU\_SSR[n] register of a specific receiver target to assign it to a specific trigger generator.
3. Enable the trigger receiver to wait for and accept a trigger.
4. Enable the trigger generator to generate a trigger.

## TRU Event Control

The TRU is a major part of event control solutions. It is the center of the trigger functional model and can extend to support the interrupt and fault management models as well.

## TRU Status and Error Signals

The TRU does not have dedicated status and error output signals other than the MMR interface. Receiver errors are reported to the generator over the standard peripheral bus protocol.

## ADSP-SC59x TRU Register Descriptions

Trigger Routing Unit (TRU) contains the following registers.

Table 7-5: ADSP-SC59x TRU Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRU_ERRADDR | Error Address Register      |
| TRU_GCTL    | Global Control Register     |
| TRU_MTR     | Generator Trigger Register  |
| TRU_SSR[n]  | Receiver Select Register    |
| TRU_STAT    | Status Information Register |

## Error Address Register

The TRU error address register ( TRU\_ERRADDR ) holds the address from the memory-mapped register access generating an access error of TRU registers.

Figure 7-2: TRU\_ERRADDR Register Diagram

<!-- image -->

Table 7-6: TRU\_ERRADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | ADDR       | Error Address. The TRU_ERRADDR.ADDR holds the address from the memory-mapped register ac- cess generating an access error of TRU registers. These errors occur on access to the TRU_SSR[n] or TRU_MTR registers when these registers are locked or on access to an invalid address. See the TRU_SSR[n] and TRU_MTR register descriptions for more information about locking. The TRU_ERRADDR register holds the address of the first error to occur. In the event of multiple errors occurring, the TRU_ERRADDR register contains the address of the first error. To re-enable the TRU_ERRADDR register for update, both status bits ( TRU_STAT.LWERR and TRU_STAT.ADDRERR ) in the TRU_STAT register must be cleared. |

## Global Control Register

The TRU global control register ( TRU\_GCTL ) provides register locking, TRU reset, and TRU enable.

Figure 7-3: TRU\_GCTL Register Diagram

<!-- image -->

Table 7-7: TRU\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | GCTL Lock Bit. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_GCTL.LOCK bit is enabled, the TRU_GCTL register is read only. |
| 2 (R/W)            | MTRL       | MTR Lock Bit. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_GCTL.MTRL bit is enabled, the TRU_MTR register is read only.   |
| 1 (R/W)            | RESET      | Soft Reset. The TRU_GCTL.RESET bit is write-1-action and triggers a soft reset to all TRU reg- isters. 0 No action                            |

Table 7-7: TRU\_GCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Non-MMR Enable. The TRU_GCTL.EN bit is read/write and must be set for the TRU to propagate trig- ger events. All TRU register read/write operations continue to operate independent of the TRU_GCTL.EN bit. 0 No trigger events |

## Generator Trigger Register

The TRU generator trigger register ( TRU\_MTR ) permits trigger generation through software by writing a trigger requester ID value to one of the four fields in the TRU\_MTR register. If the global lock is enabled ( SPU\_CTL.GLCK bit =1) and the TRU\_GCTL.LOCK bit is set, the TRU\_MTR register is read only. Note this register is primarily used for debug to trigger a TRU output

Figure 7-4: TRU\_MTR Register Diagram

<!-- image -->

Table 7-8: TRU\_MTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | MTR3       | Generator Trigger Register 3. The TRU_MTR.MTR3 bit field is the trigger generator ID value for generator 3. 0 No generator specified                                 |
| 23:16 (R/W)        | MTR2       | Generator Trigger Register 2. The TRU_MTR.MTR2 bit field is the trigger requester ID value for requester 2.                                                          |
| 15:8 (R/W)         | MTR1       | Generator Trigger Register 1. The TRU_MTR.MTR1 bit field is the trigger generator ID value for generator 1. 0 No generator specified                                 |
| 7:0 (R/W)          | MTR0       | Generator Trigger Register 0. The TRU_MTR.MTR0 bit field is the trigger generator ID value for generator 0. 0 No generator specified 1-197 Range of valid generators |

## Receiver Select Register

The TRU receiver select registers ( TRU\_SSR[n] ) each provide receiver selection and register locking.

Figure 7-5: TRU\_SSR[n] Register Diagram

<!-- image -->

Table 7-9: TRU\_SSR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | SSRn Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_SSR[n].LOCK bit is enabled, the TRU_SSR[n] register is read only. 0 Unlock register                                                                                                                                                                           |
| 7:0 (R/W)          | SSR        | SSRn Completer Select. The TRU_SSR[n] register selects the trigger generator ID to which the trigger re- ceiver responds. For example, when a TRU_SSR[n] register is set to respond to trig- ger generator ID n, a trigger that is generated by trigger generator ID n results in a trigger out to the receiver. 0 No generator specified |

## Status Information Register

The TRU status register ( TRU\_STAT ) contains the status of TRU\_MTR and TRU\_SSR[n] register writes and status of bus read/write errors.

Figure 7-6: TRU\_STAT Register Diagram

<!-- image -->

Table 7-10: TRU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | ADDRERR    | Address Error Status. The TRU_STAT.ADDRERR bit is set when an invalid address is provided for an MMRaccess while the TRU is selected. Writing a one to this bit clears the error indi- cation. The TRU_ERRADDR register also is updated when an address error occurs dur- ing an MMRaccess while the TRU is selected. 0 No error |
| 0 (R/W1C)          | LWERR      | Lock Write Error Status. If TRU_STAT.LWERR is set, a lock write error has occurred. Writing a one to this bit clears the error indication. 0 No error 1 Error occurred                                                                                                                                                           |