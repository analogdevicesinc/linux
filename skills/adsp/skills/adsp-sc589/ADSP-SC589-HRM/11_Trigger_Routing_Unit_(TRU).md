## 8   Trigger Routing Unit (TRU)

The TRU provides system-level sequence control without core intervention. The TRU maps trigger masters (generators of triggers) to trigger slaves (receivers of triggers). Slave endpoints can be configured to respond to triggers in various ways. Multiple TRUs may be provided in a multiprocessor system to create a trigger network. Common applications enabled by the TRU include:

- Automatically triggering the start of a DMA sequence after a sequence from another DMA channel completes
- Software triggering
- Synchronization of concurrent activities

## TRU Features

The TRU supports the following features:

- Automatically triggering the start of a DMA sequence after a sequence from another DMA channel completes. Once a DMA channel completes data transfer, it can act as a Trigger Master and signal an internal trigger pulse to the programmed Trigger Slave which can also be another DMA channel. The Slave Trigger connected to the DMA channel kicks off the DMA transfer automatically. None of this requires core intervention once the initialization is done.
- Software triggers. The best use of triggers is to minimize core intervention. It is also possible to initiate a trigger pulse to a T rigger Slave, in the software.
- Synchronization of concurrent activities. A single T rigger Master can initiate a trigger pulse to multiple T rigger Slaves so that several system level activities can be synchronized on an internally or externally generated event.
- Configuration protection through register-level lock bits and global lock indication

## TRU Functional Description

The following sections provide a description of the TRU.

## ADSP-SC58x TRU Register List

The Trigger Routing Unit (TRU) provides simple sequence control of distributed modules without the penalties associated with core intervention (for example, interrupt overhead). The TRU receives trigger inputs from all master trigger inputs (MTI) and the TRU master trigger register ( TRU\_MTR ). Based on these inputs, the TRU logic generates trigger outputs that initiate slave operations in the processor core and peripherals. A set of registers governs TRU operations. For more information on TRU functionality, see the TRU register descriptions.

Table 8-1: ADSP-SC58x TRU Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRU_ERRADDR | Error Address Register      |
| TRU_GCTL    | Global Control Register     |
| TRU_MTR     | Master Trigger Register     |
| TRU_SSR[n]  | Slave Select Register       |
| TRU_STAT    | Status Information Register |

## ADSP-SC58x TRU Interrupt List

Table 8-2: ADSP-SC58x TRU Interrupt List

|   Interrupt ID | Name       | Description                         | Sensitivity   | DMA Channel   |
|----------------|------------|-------------------------------------|---------------|---------------|
|            136 | TRU0_SLV4  | TRU0 Interrupt 4, Core ID = 1 only  | Edge          |               |
|            137 | TRU0_SLV5  | TRU0 Interrupt 5, Core ID = 1 only  | Edge          |               |
|            138 | TRU0_SLV6  | TRU0 Interrupt 6, Core ID = 1 only  | Edge          |               |
|            139 | TRU0_SLV7  | TRU0 Interrupt 7, Core ID = 1 only  | Edge          |               |
|            140 | TRU0_SLV8  | TRU0 Interrupt 8, Core ID = 2 only  | Edge          |               |
|            141 | TRU0_SLV9  | TRU0 Interrupt 9, Core ID = 2 only  | Edge          |               |
|            142 | TRU0_SLV10 | TRU0 Interrupt 10, Core ID = 2 only | Edge          |               |
|            143 | TRU0_SLV11 | TRU0 Interrupt 11, Core ID = 2 only | Edge          |               |
|            280 | TRU0_SLV0  | TRU0 Interrupt 0, Core ID = 0 only  | Edge          |               |
|            281 | TRU0_SLV1  | TRU0 Interrupt 1, Core ID = 0 only  | Edge          |               |
|            282 | TRU0_SLV2  | TRU0 Interrupt 2, Core ID = 0 only  | Edge          |               |
|            283 | TRU0_SLV3  | TRU0 Interrupt 3, Core ID = 0 only  | Edge          |               |

## ADSP-SC58x Trigger List

Table 8-3: ADSP-SC58x Trigger List Masters

|   Trigger ID | Name            | Description               | Sensitivity   |
|--------------|-----------------|---------------------------|---------------|
|            0 | RESERVED_TRIG0  | NC_RES0                   |               |
|            1 | CGU0_EVT        | CGU0 Event                | Edge          |
|            2 | CGU1_EVT        | CGU1 Event                | Edge          |
|            3 | C0_SNDEVT       | Core0 Send Event          |               |
|            4 | C0_WFI          | Core0 Wait For Interrupt  |               |
|            5 | C0_WFE          | Core0 Wait For Event      |               |
|            6 | TIMER0_TMR0_MST | TIMER0 Timer 0            | Edge          |
|            7 | TIMER0_TMR1_MST | TIMER0 Timer 1            | Edge          |
|            8 | TIMER0_TMR2_MST | TIMER0 Timer 2            | Edge          |
|            9 | TIMER0_TMR3_MST | TIMER0 Timer 3            | Edge          |
|           10 | TIMER0_TMR4_MST | TIMER0 Timer 4            | Edge          |
|           11 | TIMER0_TMR5_MST | TIMER0 Timer 5            | Edge          |
|           12 | TIMER0_TMR6_MST | TIMER0 Timer 6            | Edge          |
|           13 | TIMER0_TMR7_MST | TIMER0 Timer 7            | Edge          |
|           14 | PINT0_BLOCK     | PINT0 Pin Interrupt Block | Level         |
|           15 | PINT1_BLOCK     | PINT1 Pin Interrupt Block | Level         |
|           16 | PINT2_BLOCK     | PINT2 Pin Interrupt Block | Level         |
|           17 | PINT3_BLOCK     | PINT3 Pin Interrupt Block | Level         |
|           18 | PINT4_BLOCK     | PINT4 Pin Interrupt Block | Level         |
|           19 | PINT5_BLOCK     | PINT5 Pin Interrupt Block | Level         |
|           20 | CNT0_STAT       | CNT0 Status               | Level         |
|           21 | PWM0_SYNC       | PWM0 PWMTMRGrouped        | Edge          |
|           22 | PWM1_SYNC       | PWM1 PWMTMRGrouped        | Edge          |
|           23 | PWM2_SYNC       | PWM2 PWMTMRGrouped        | Edge          |
|           24 | SPORT0_A_DMA    | SPORT0 ChannelADMA        | Edge          |
|           25 | SPORT0_B_DMA    | SPORT0 ChannelBDMA        | Edge          |
|           26 | SPORT1_A_DMA    | SPORT1 ChannelADMA        | Edge          |
|           27 | SPORT1_B_DMA    | SPORT1 ChannelBDMA        | Edge          |
|           28 | SPORT2_A_DMA    | SPORT2 ChannelADMA        | Edge          |
|           29 | SPORT2_B_DMA    | SPORT2 ChannelBDMA        | Edge          |

Table 8-3: ADSP-SC58x Trigger List Masters (Continued)

|   Trigger ID | Name           | Description                     | Sensitivity   |
|--------------|----------------|---------------------------------|---------------|
|           30 | SPORT3_A_DMA   | SPORT3 ChannelADMA              | Edge          |
|           31 | SPORT3_B_DMA   | SPORT3 ChannelBDMA              | Edge          |
|           32 | SPORT4_A_DMA   | SPORT4 ChannelADMA              | Edge          |
|           33 | SPORT4_B_DMA   | SPORT4 ChannelBDMA              | Edge          |
|           34 | SPORT5_A_DMA   | SPORT5 ChannelADMA              | Edge          |
|           35 | SPORT5_B_DMA   | SPORT5 ChannelBDMA              | Edge          |
|           36 | SPORT6_A_DMA   | SPORT6 ChannelADMA              | Edge          |
|           37 | SPORT6_B_DMA   | SPORT6 ChannelBDMA              | Edge          |
|           38 | SPORT7_A_DMA   | SPORT7 ChannelADMA              | Edge          |
|           39 | SPORT7_B_DMA   | SPORT7 ChannelBDMA              | Edge          |
|           40 | SPI0_TXDMA     | SPI0 TX DMAChannel              | Edge          |
|           41 | SPI0_RXDMA     | SPI0 RX DMAChannel              | Edge          |
|           42 | SPI1_TXDMA     | SPI1 TX DMAChannel              | Edge          |
|           43 | SPI1_RXDMA     | SPI1 RX DMAChannel              | Edge          |
|           44 | SPI2_TXDMA     | SPI2 TX DMAChannel              | Edge          |
|           45 | SPI2_RXDMA     | SPI2 RX DMAChannel              | Edge          |
|           46 | HAE0_RXDMA_CH0 | HAE0 RX DMAChannel 0            | Level         |
|           47 | HAE0_RXDMA_CH1 | HAE0 RX DMAChannel 1            | Level         |
|           48 | HAE0_TXDMA     | HAE0 TX DMAChannel 0            | Level         |
|           49 | SINC0_P0_OVLD  | SINC0 Pair 0 Overload Indicator | Edge          |
|           50 | SINC0_P1_OVLD  | SINC0 Pair 1 Overload Indicator | Edge          |
|           51 | SINC0_P2_OVLD  | SINC0 Pair 2 Overload Indicator | Edge          |
|           52 | SINC0_P3_OVLD  | SINC0 Pair 3 Overload Indicator | Edge          |
|           53 | SINC0_DATA0    | SINC0 Data Move 0               | Edge          |
|           54 | SINC0_DATA1    | SINC0 Data Move 1               | Edge          |
|           55 | EMAC0_STAT     | EMAC0 Status                    | None          |
|           56 | EMAC1_STAT     | EMAC1 Status                    | None          |
|           57 | FFTA0_TXDMA    | FFTA0 TransmitDMA               |               |
|           58 | FFTA0_RXDMA    | FFTA0 ReceiveDMA                |               |
|           59 | FFTA0_TRIGOUT  | FFTA0 Trigger Out               |               |
|           60 | FIR0_DMA       | FIR0DMA                         | Edge          |

Table 8-3: ADSP-SC58x Trigger List Masters (Continued)

|   Trigger ID | Name          | Description                                    | Sensitivity   |
|--------------|---------------|------------------------------------------------|---------------|
|           61 | IIR0_DMA      | IIR0DMA                                        | Edge          |
|           62 | EPPI0_CH0_DMA | EPPI0 Channel0DMA                              | Edge          |
|           63 | EPPI0_CH1_DMA | EPPI0 Channel1DMA                              | Edge          |
|           64 | LP0_DMA       | LP0 DMAChannel                                 |               |
|           65 | LP1_DMA       | LP1 DMAChannel                                 |               |
|           66 | UART0_TXDMA   | UART0 TransmitDMA                              | Edge          |
|           67 | UART0_RXDMA   | UART0 ReceiveDMA                               | Edge          |
|           68 | UART1_TXDMA   | UART1 TransmitDMA                              | Edge          |
|           69 | UART1_RXDMA   | UART1 ReceiveDMA                               | Edge          |
|           70 | UART2_TXDMA   | UART2 TransmitDMA                              | Edge          |
|           71 | UART2_RXDMA   | UART2 ReceiveDMA                               | Edge          |
|           72 | USB0_DATA     | USB0 DMAStatus/Transfer Complete               | Level         |
|           73 | USB1_DATA     | USB1 DMAStatus/Transfer Complete               | Level         |
|           74 | MDMA0_SRC     | Standard BWMDMAChannel 0 Source (CRC IN)       |               |
|           75 | MDMA0_DST     | Standard BWMDMAChannel 0 Destination (CRC OUT) |               |
|           76 | MDMA1_SRC     | Standard BWMDMAChannel 1 Source (CRC IN)       |               |
|           77 | MDMA1_DST     | Standard BWMDMAChannel 1 Destination (CRC OUT) |               |
|           78 | MDMA2_SRC     | Enh BWMDMAChannel 2 Source                     |               |
|           79 | MDMA2_DST     | Enh BWMDMAChannel 2 Destination                |               |
|           80 | MDMA3_SRC     | Max BWMDMAChannel 3 Source                     |               |
|           81 | MDMA3_DST     | Max BWMDMAChannel 3 Destination                |               |
|           82 | EMDMA0_DONE   | EMDMA0 DMADone                                 | Edge          |
|           83 | EMDMA1_DONE   | EMDMA1 DMADone                                 | Edge          |
|           84 | CTI3_MST0     | CTI3 SYSCTI (CTI3) System Halt Slave 0         | Edge          |
|           85 | CTI3_MST1     | CTI3 SYSCTI (CTI3) System Halt Slave 1         | Edge          |
|           86 | CTI3_MST2     | CTI3 SYSCTI (CTI3) System Halt Slave 2         | Edge          |
|           87 | CTI3_MST3     | CTI3 SYSCTI (CTI3) System Halt Slave 3         | Edge          |
|           88 | CTI3_MST4     | CTI3 SYSCTI (CTI3) System Halt Slave 4         | Edge          |

Table 8-3: ADSP-SC58x Trigger List Masters (Continued)

|   Trigger ID | Name       | Description                            | Sensitivity   |
|--------------|------------|----------------------------------------|---------------|
|           89 | CTI3_MST5  | CTI3 SYSCTI (CTI3) System Halt Slave 5 | Edge          |
|           90 | CTI3_MST6  | CTI3 SYSCTI (CTI3) System Halt Slave 6 | Edge          |
|           91 | CTI3_MST7  | CTI3 SYSCTI (CTI3) System Halt Slave 7 | Edge          |
|           92 | SEC0_FAULT | SEC0 Fault                             | Edge          |
|           93 | SOFT0_MST  | Software-driven Trigger 0              |               |
|           94 | SOFT1_MST  | Software-driven Trigger 1              |               |
|           95 | SOFT2_MST  | Software-driven Trigger 2              |               |
|           96 | SOFT3_MST  | Software-driven Trigger 3              |               |
|           97 | SOFT4_MST  | Software-driven Trigger 4              |               |
|           98 | SOFT5_MST  | Software-driven Trigger 5              |               |
|           99 | SWU0_EVT   | SWU0 Event                             | None          |
|          100 | SWU2_EVT   | SWU2 Event                             | None          |
|          101 | SWU1_EVT   | SWU1 Event                             | None          |
|          102 | SWU4_EVT   | SWU4 Event                             | None          |
|          103 | SWU3_EVT   | SWU3 Event                             | None          |
|          104 | SWU6_EVT   | SWU6 Event                             | None          |
|          105 | SWU5_EVT   | SWU5 Event                             | None          |
|          106 | SWU7_EVT   | SWU7 Event                             | None          |
|          107 | SWU8_EVT   | SWU8 Event                             | None          |
|          108 | SWU9_EVT   | SWU9 Event                             | None          |
|          109 | SWU10_EVT  | SWU10 Event                            | None          |
|          110 | SWU11_EVT  | SWU11 Event                            | None          |
|          111 | SWU12_EVT  | SWU12 Event                            | None          |
|          112 | SWU13_EVT  | SWU13 Event                            | None          |
|          113 | SWU14_EVT  | SWU14 Event                            | None          |
|          114 | SWU15_EVT  | SWU15 Event                            | None          |
|          115 | SWU0_DBG   | SWU0 Debug                             | Edge          |
|          116 | SWU2_DBG   | SWU2 Debug                             | Edge          |
|          117 | SWU1_DBG   | SWU1 Debug                             | Edge          |
|          118 | SWU4_DBG   | SWU4 Debug                             | Edge          |
|          119 | SWU3_DBG   | SWU3 Debug                             | Edge          |

Table 8-3: ADSP-SC58x Trigger List Masters (Continued)

|   Trigger ID | Name              | Description                                | Sensitivity   |
|--------------|-------------------|--------------------------------------------|---------------|
|          120 | SWU6_DBG          | SWU6 Debug                                 | Edge          |
|          121 | SWU5_DBG          | SWU5 Debug                                 | Edge          |
|          122 | SWU7_DBG          | SWU7 Debug                                 | Edge          |
|          123 | SWU8_DBG          | SWU8 Debug                                 | Edge          |
|          124 | SWU9_DBG          | SWU9 Debug                                 | Edge          |
|          125 | SWU10_DBG         | SWU10 Debug                                | Edge          |
|          126 | SWU11_DBG         | SWU11 Debug                                | Edge          |
|          127 | SWU12_DBG         | SWU12 Debug                                | Edge          |
|          128 | SWU13_DBG         | SWU13 Debug                                | Edge          |
|          129 | SWU14_DBG         | SWU14 Debug                                | Edge          |
|          130 | SWU15_DBG         | SWU15 Debug                                | Edge          |
|          131 | TMU0_FAULT        | TMU0 Fault Event                           |               |
|          132 | ACM0_EVT_COMPLETE | ACM0 Event Complete                        |               |
|          133 | HADC0_EOC         | HADC0 End of Conversion                    | Edge          |
|          134 | RTC0_EVT          | RTC0 Event                                 | Level         |
|          135 | PCIE0_DMA         | PCIE0 DMADone                              |               |
|          136 | MSI0_DONE         | MSI0 Transfer Done                         | Level         |
|          137 | C1_SID_ACK        | Core1 System Interface Disable Acknowledge |               |
|          138 | C2_SID_ACK        | Core2 System Interface Disable Acknowledge |               |

Table 8-4: ADSP-SC58x Trigger List Slaves

|   Trigger ID | Name             | Description               | Sensitivity   |
|--------------|------------------|---------------------------|---------------|
|            0 | TIMER0_TMR0_SLV0 | TIMER0 Timer 0            | Pulse         |
|            1 | TIMER0_TMR1_SLV0 | TIMER0 Timer 1            | Pulse         |
|            2 | TIMER0_TMR2_SLV0 | TIMER0 Timer 2            | Pulse         |
|            3 | TIMER0_TMR3_SLV0 | TIMER0 Timer 3            | Pulse         |
|            4 | TIMER0_TMR4_SLV0 | TIMER0 Timer 4            | Pulse         |
|            5 | TIMER0_TMR5_SLV0 | TIMER0 Timer 5            | Pulse         |
|            6 | TIMER0_TMR6_SLV0 | TIMER0 Timer 6            | Pulse         |
|            7 | TIMER0_TMR7_SLV0 | TIMER0 Timer 7            | Pulse         |
|            8 | PWM0_TRIP_TRIG0  | PWM0 Trip Trigger Slave 0 | Pulse         |

Table 8-4: ADSP-SC58x Trigger List Slaves (Continued)

|   Trigger ID | Name            | Description               | Sensitivity   |
|--------------|-----------------|---------------------------|---------------|
|            9 | PWM0_TRIP_TRIG1 | PWM0 Trip Trigger Slave 1 | Pulse         |
|           10 | PWM0_TRIP_TRIG2 | PWM0 Trip Trigger Slave 2 | Pulse         |
|           11 | PWM1_TRIP_TRIG0 | PWM1 Trip Trigger Slave 0 | Pulse         |
|           12 | PWM1_TRIP_TRIG1 | PWM1 Trip Trigger Slave 1 | Pulse         |
|           13 | PWM1_TRIP_TRIG2 | PWM1 Trip Trigger Slave 2 | Pulse         |
|           14 | PWM2_TRIP_TRIG0 | PWM2 Trip Trigger Slave 0 | Pulse         |
|           15 | PWM2_TRIP_TRIG1 | PWM2 Trip Trigger Slave 1 | Pulse         |
|           16 | PWM2_TRIP_TRIG2 | PWM2 Trip Trigger Slave 2 | Pulse         |
|           17 | SPORT0_A_DMA    | SPORT0 ChannelADMA        | Pulse         |
|           18 | SPORT0_B_DMA    | SPORT0 ChannelBDMA        | Pulse         |
|           19 | SPORT1_A_DMA    | SPORT1 ChannelADMA        | Pulse         |
|           20 | SPORT1_B_DMA    | SPORT1 ChannelBDMA        | Pulse         |
|           21 | SPORT2_A_DMA    | SPORT2 ChannelADMA        | Pulse         |
|           22 | SPORT2_B_DMA    | SPORT2 ChannelBDMA        | Pulse         |
|           23 | SPORT3_A_DMA    | SPORT3 ChannelADMA        | Pulse         |
|           24 | SPORT3_B_DMA    | SPORT3 ChannelBDMA        | Pulse         |
|           25 | SPORT4_A_DMA    | SPORT4 ChannelADMA        | Pulse         |
|           26 | SPORT4_B_DMA    | SPORT4 ChannelBDMA        | Pulse         |
|           27 | SPORT5_A_DMA    | SPORT5 ChannelADMA        | Pulse         |
|           28 | SPORT5_B_DMA    | SPORT5 ChannelBDMA        | Pulse         |
|           29 | SPORT6_A_DMA    | SPORT6 ChannelADMA        | Pulse         |
|           30 | SPORT6_B_DMA    | SPORT6 ChannelBDMA        | Pulse         |
|           31 | SPORT7_A_DMA    | SPORT7 ChannelADMA        | Pulse         |
|           32 | SPORT7_B_DMA    | SPORT7 ChannelBDMA        | Pulse         |
|           33 | SPI0_TXDMA      | SPI0 TX DMAChannel        | Pulse         |
|           34 | SPI0_RXDMA      | SPI0 RX DMAChannel        | Pulse         |
|           35 | SPI1_TXDMA      | SPI1 TX DMAChannel        | Pulse         |
|           36 | SPI1_RXDMA      | SPI1 RX DMAChannel        | Pulse         |
|           37 | SPI2_TXDMA      | SPI2 TX DMAChannel        | Pulse         |
|           38 | SPI2_RXDMA      | SPI2 RX DMAChannel        | Pulse         |
|           39 | HAE0_RXDMA_CH0  | HAE0 RX DMAChannel 0      | Pulse         |

Table 8-4: ADSP-SC58x Trigger List Slaves (Continued)

|   Trigger ID | Name           | Description                   | Sensitivity   |
|--------------|----------------|-------------------------------|---------------|
|           40 | HAE0_RXDMA_CH1 | HAE0 RX DMAChannel 1          | Pulse         |
|           41 | HAE0_TXDMA     | HAE0 TX DMAChannel 0          | Pulse         |
|           42 | SINC0_SYNC0    | SINC0 Synchronization Input 0 | Pulse         |
|           43 | SINC0_SYNC1    | SINC0 Synchronization Input 1 | Pulse         |
|           44 | FFTA0_TXDMA    | FFTA0 TransmitDMA             | Pulse         |
|           45 | FFTA0_RXDMA    | FFTA0 ReceiveDMA              | Pulse         |
|           46 | EPPI0_CH0_DMA  | EPPI0 Channel0DMA             | Pulse         |
|           47 | EPPI0_CH1_DMA  | EPPI0 Channel1DMA             | Pulse         |
|           48 | LP0_DMA        | LP0 DMAChannel                | Pulse         |
|           49 | LP1_DMA        | LP1 DMAChannel                | Pulse         |
|           50 | RCU0_SYSRST0   | RCU0 System Reset 0           | Pulse         |
|           51 | RCU0_SYSRST1   | RCU0 System Reset 1           | Pulse         |
|           52 | STM0_EVT0      | STM0 Event 0                  | Pulse         |
|           53 | STM0_EVT1      | STM0 Event 1                  | Pulse         |
|           54 | STM0_EVT2      | STM0 Event 2                  | Pulse         |
|           55 | STM0_EVT3      | STM0 Event 3                  | Pulse         |
|           56 | STM0_EVT4      | STM0 Event 4                  | Pulse         |
|           57 | STM0_EVT5      | STM0 Event 5                  | Pulse         |
|           58 | STM0_EVT6      | STM0 Event 6                  | Pulse         |
|           59 | STM0_EVT7      | STM0 Event 7                  | Pulse         |
|           60 | STM0_EVT8      | STM0 Event 8                  | Pulse         |
|           61 | STM0_EVT9      | STM0 Event 9                  | Pulse         |
|           62 | STM0_EVT10     | STM0 Event 10                 | Pulse         |
|           63 | STM0_EVT11     | STM0 Event 11                 | Pulse         |
|           64 | STM0_EVT12     | STM0 Event 12                 | Pulse         |
|           65 | STM0_EVT13     | STM0 Event 13                 | Pulse         |
|           66 | STM0_EVT14     | STM0 Event 14                 | Pulse         |
|           67 | STM0_EVT15     | STM0 Event 15                 | Pulse         |
|           68 | STM0_EVT16     | STM0 Event 16                 | Pulse         |
|           69 | STM0_EVT17     | STM0 Event 17                 | Pulse         |
|           70 | STM0_EVT18     | STM0 Event 18                 | Pulse         |

Table 8-4: ADSP-SC58x Trigger List Slaves (Continued)

|   Trigger ID | Name        | Description                                 | Sensitivity   |
|--------------|-------------|---------------------------------------------|---------------|
|           71 | STM0_EVT19  | STM0 Event 19                               | Pulse         |
|           72 | STM0_EVT20  | STM0 Event 20                               | Pulse         |
|           73 | STM0_EVT21  | STM0 Event 21                               | Pulse         |
|           74 | STM0_EVT22  | STM0 Event 22                               | Pulse         |
|           75 | STM0_EVT23  | STM0 Event 23                               | Pulse         |
|           76 | STM0_EVT24  | STM0 Event 24                               | Pulse         |
|           77 | STM0_EVT25  | STM0 Event 25                               | Pulse         |
|           78 | STM0_EVT26  | STM0 Event 26                               | Pulse         |
|           79 | STM0_EVT27  | STM0 Event 27                               | Pulse         |
|           80 | STM0_EVT28  | STM0 Event 28                               | Pulse         |
|           81 | STM0_EVT29  | STM0 Event 29                               | Pulse         |
|           82 | STM0_EVT30  | STM0 Event 30                               | Pulse         |
|           83 | STM0_EVT31  | STM0 Event 31                               | Pulse         |
|           84 | TRU0_SLV0   | TRU0 Interrupt Request 0, core ID = 0 only  | Pulse         |
|           85 | TRU0_SLV1   | TRU0 Interrupt Request 1, core ID = 0 only  | Pulse         |
|           86 | TRU0_SLV2   | TRU0 Interrupt Request 2, core ID = 0 only  | Pulse         |
|           87 | TRU0_SLV3   | TRU0 Interrupt Request 3, core ID = 0 only  | Pulse         |
|           88 | TRU0_SLV4   | TRU0 Interrupt Request 4, core ID = 1 only  | Pulse         |
|           89 | TRU0_SLV5   | TRU0 Interrupt Request 5, core ID = 1 only  | Pulse         |
|           90 | TRU0_SLV6   | TRU0 Interrupt Request 6, core ID = 1 only  | Pulse         |
|           91 | TRU0_SLV7   | TRU0 Interrupt Request 7, core ID = 1 only  | Pulse         |
|           92 | TRU0_SLV8   | TRU0 Interrupt Request 8, core ID = 2 only  | Pulse         |
|           93 | TRU0_SLV9   | TRU0 Interrupt Request 9, core ID = 2 only  | Pulse         |
|           94 | TRU0_SLV10  | TRU0 Interrupt Request 10, Core ID = 2 only | Pulse         |
|           95 | TRU0_SLV11  | TRU0 Interrupt Request 11, core ID = 2 only | Pulse         |
|           96 | UART0_TXDMA | UART0 TransmitDMA                           | Pulse         |
|           97 | UART0_RXDMA | UART0 ReceiveDMA                            | Pulse         |
|           98 | UART1_TXDMA | UART1 TransmitDMA                           | Pulse         |
|           99 | UART1_RXDMA | UART1 ReceiveDMA                            | Pulse         |
|          100 | UART2_TXDMA | UART2 TransmitDMA                           | Pulse         |
|          101 | UART2_RXDMA | UART2 ReceiveDMA                            | Pulse         |

Table 8-4: ADSP-SC58x Trigger List Slaves (Continued)

|   Trigger ID | Name      | Description                                    | Sensitivity   |
|--------------|-----------|------------------------------------------------|---------------|
|          102 | MDMA0_SRC | Standard BWMDMAChannel 0 Source (CRC IN)       | Pulse         |
|          103 | MDMA0_DST | Standard BWMDMAChannel 0 Destination (CRC OUT) | Pulse         |
|          104 | MDMA1_SRC | Standard BWMDMAChannel 1 Source (CRC IN)       | Pulse         |
|          105 | MDMA1_DST | Standard BWMDMAChannel 1 Destination (CRC OUT) | Pulse         |
|          106 | MDMA2_SRC | Enh BWMDMASource Channel                       | Pulse         |
|          107 | MDMA2_DST | Enh BWMDMADestination Channel                  | Pulse         |
|          108 | MDMA3_SRC | Max BWMDMASource Channel                       | Pulse         |
|          109 | MDMA3_DST | Max BWMDMADestination Channel                  | Pulse         |
|          110 | CTI3_SLV0 | CTI3 SYSCTI System Halt Master 0               | Pulse         |
|          111 | CTI3_SLV1 | CTI3 SYSCTI System Halt Master 1               | Pulse         |
|          112 | CTI3_SLV2 | CTI3 SYSCTI System Halt Master 2               | Pulse         |
|          113 | CTI3_SLV3 | CTI3 SYSCTI System Halt Master 3               | Pulse         |
|          114 | CTI3_SLV4 | CTI3 SYSCTI System Halt Master 4               | Pulse         |
|          115 | CTI3_SLV5 | CTI3 SYSCTI System Halt Master 5               | Pulse         |
|          116 | CTI3_SLV6 | CTI3 SYSCTI System Halt Master 6               | Pulse         |
|          117 | CTI3_SLV7 | CTI3 SYSCTI System Halt Master 7               | Pulse         |
|          118 | C0_EVT    | C0 Event input for wake-up from WFE state      | Pulse         |
|          119 | SWU0_EN   | SWU0 Enable                                    | Pulse         |
|          120 | SWU2_EN   | SWU2 Enable                                    | Pulse         |
|          121 | SWU1_EN   | SWU1 Enable                                    | Pulse         |
|          122 | SWU4_EN   | SWU4 Enable                                    | Pulse         |
|          123 | SWU3_EN   | SWU3 Enable                                    | Pulse         |
|          124 | SWU6_EN   | SWU6 Enable                                    | Pulse         |
|          125 | SWU5_EN   | SWU5 Enable                                    | Pulse         |
|          126 | SWU7_EN   | SWU7 Enable                                    | Pulse         |
|          127 | SWU8_EN   | SWU8 Enable                                    | Pulse         |
|          128 | SWU9_EN   | SWU9 Enable                                    | Pulse         |
|          129 | SWU10_EN  | SWU10 Enable                                   | Pulse         |

Table 8-4: ADSP-SC58x Trigger List Slaves (Continued)

|   Trigger ID | Name       | Description          | Sensitivity   |
|--------------|------------|----------------------|---------------|
|          130 | SWU11_EN   | SWU11 Enable         | Pulse         |
|          131 | SWU12_EN   | SWU12 Enable         | Pulse         |
|          132 | SWU13_EN   | SWU13 Enable         | Pulse         |
|          133 | SWU14_EN   | SWU14 Enable         | Pulse         |
|          134 | SWU15_EN   | SWU15 Enable         | Pulse         |
|          135 | ACM0_TRIG2 | ACM0 Trigger Input 2 | Pulse         |
|          136 | ACM0_TRIG3 | ACM0 Trigger Input 3 | Pulse         |

## TRU Definitions

The following definitions are helpful when using the TRU module.

## Trigger Master

A trigger master is any system module that provides trigger event indication to the TRU. T rigger master modules define trigger events and conditions for assertion.

## Trigger Master ID

Trigger masters are assigned a unique numeric ID according to their physical connection to the TRU. Trigger master ID 0 is reserved and defined as null.

## Trigger Slave

A trigger slave is any system module that receives a trigger event indication from the TRU. T rigger slave modules define a trigger event response.

## TRU Block Diagram

Trigger masters and the Master Trigger Register (MTR) generate trigger assertions. Each trigger slave has a dedicated Slave Select Register (SSR) that specifies the unique trigger master from which it receives the trigger indication.

Figure 8-1: TRU Block Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000000_3b92770f446d37ff13d1654a6bb7519261755a41df91627c3a208eb98faa53f3.png)

## TRU Architectural Concepts

The TRU supports a simple trigger-in/trigger-out model for modules that comply with the triggering functional model. The TRU is the controller of the trigger system. T rigger outputs from trigger masters are mapped to trigger inputs of trigger slaves through a set of programmable registers ( TRU\_SSR[n] ).

System modules are trigger master only, trigger slave only, or trigger master and trigger slave.

All of the trigger input and output signals are connected to a trigger routing unit (TRU) which manages the connections of triggers between modules.

In multi-processor systems, multiple TRU units are provided. These TRUs are networked together. Generic Trigger Ports (GTPs) are provided to forward trigger events from one TRU unit to another, forming a pathway from trigger masters to trigger slaves wherever they might lie in the system.

## TRU Programming Model

Implementing sequence control using the TRU requires, at a minimum, proper configuration of a trigger slave, a trigger master, and the TRU module itself. The only requirement for the configuration procedure is that the trigger master is configured and enabled as the last step.

Complete the following other steps:

- Configure the trigger slave for response to triggers.
- Configure the TRU to map the trigger master to the trigger slave through the TRU\_SSR[n] registers.
- Configure the trigger master to generate trigger assertions.
- Alternatively, use software triggering for trigger assertion. Writing the trigger master ID to the MTR register generates software triggers.

## Programming Concepts

The following concepts aid in programming the TRU.

- Trigger Sequence Configuration. A simple sequence consists of one trigger master and one trigger slave. More complex trigger sequences consist of several trigger slaves functioning as trigger slave and trigger master. Additionally, trigger sequences can loopback to the original master forming a perpetual sequence.
- Software Triggering. Writing a trigger master ID to the MTR generates a trigger within the TRU from the trigger master ID specified.
- Synchronization. The TRU can be used to coarsely synchronize events by mapping multiple trigger slaves to the same trigger master or by generating multiple trigger master assertions simultaneously through the MTR.
- Configuration Protection. The TRU\_SSR[n].LOCK bit and the TRU\_GCTL.LOCK bit enable register level write-protection when the global lock is asserted in the SPU.

## Programming Examples

The following examples shows the steps to create a single trigger and a Timer period expiry event automatically toggling a GPIO.

## Configuring a Simple Trigger Sequence

The following example shows the steps to create a simple trigger.

1. Write to the TRU\_GCTL register to enable the TRU.
2. Write to the TRU\_SSR[n] register of a specific trigger slave to assign it to a specific trigger master.
3. Enable the trigger slave to wait for and accept a trigger.
4. Enable the trigger master to generate a trigger.

## Toggle a GPIO on Timer Expiry Event

This example shows a case where a Timer period expiry event automatically toggles a GPIO. This is achieved by programming the Slave trigger register for GPIO with the Timer as Master.

1. Enable a specific pin in the PORT F to toggle up on trigger ( )
2. Enable timer to generate trigger up ( TIMER\_TRG\_MSK register)
3. Program the Slave trigger 23 ( TRU\_SSR[n] register) (this toggles the PORTs) with the Timer as the Master.
4. Enable the TRU ( TRU\_GCTL.EN bit).

```
*pREG_PORTF_TRIG_TGL = 0x00000010; *pREG_TIMER1_TRG_MSK &= ~BITM_TIMER_TRG_MSK_TMR03; *pREG_TRU1_SSR23 = TRGM_TIMER1_TMR3_MST; *pREG_TRU1_GCTL = 0x1;
```

## TRU Event Control

The TRU is a major part of event control solutions. It is the center of the trigger functional model and can extend to support the interrupt and fault management models as well.

## TRU Status and Error Signals

The TRU does not have dedicated status and error output signals other than the MMR interface. Slave errors are reported to the master over the standard peripheral bus protocol.

## ADSP-SC58x TRU Register Descriptions

Trigger Routing Unit (TRU) contains the following registers.

Table 8-5: ADSP-SC58x TRU Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRU_ERRADDR | Error Address Register      |
| TRU_GCTL    | Global Control Register     |
| TRU_MTR     | Master Trigger Register     |
| TRU_SSR[n]  | Slave Select Register       |
| TRU_STAT    | Status Information Register |

## Error Address Register

The TRU error address register ( TRU\_ERRADDR ) holds the address from the memory-mapped register access generating an access error of TRU registers.

Figure 8-2: TRU\_ERRADDR Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000001_7759a66384ed26fb09e77b15d9dcdc488345a80f3730d004820858cd00b37256.png)

Table 8-6: TRU\_ERRADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | ADDR       | Error Address. The TRU_ERRADDR.ADDR holds the address from the memory-mapped register ac- cess generating an access error of TRU registers. These errors occur on access to the TRU_SSR[n] or TRU_MTR registers when these registers are locked or on access to an invalid address. See the TRU_SSR[n] and TRU_MTR register descriptions for more information about locking. The TRU_ERRADDR register holds the address of the first error to occur. In the event of multiple errors occurring, the TRU_ERRADDR register contains the address of the first error. To re-enable the TRU_ERRADDR register for update, both status bits ( TRU_STAT.LWERR and TRU_STAT.ADDRERR ) in the TRU_STAT register must be cleared. |

## Global Control Register

The TRU global control register ( TRU\_GCTL ) provides register locking, TRU reset, and TRU enable.

Figure 8-3: TRU\_GCTL Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000002_7988886643aa57bc6c2b7361737fd30dd4f6fbcdc372dbd41dba511c2bea5e30.png)

Table 8-7: TRU\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | GCTL Lock Bit. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_GCTL.LOCK bit is enabled, the TRU_GCTL register is read only. |
| 2 (R/W)            | MTRL       | MTR Lock Bit. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_GCTL.MTRL bit is enabled, the TRU_MTR register is read only.   |
| 1 (R/W)            | RESET      | Soft Reset. The TRU_GCTL.RESET bit is write-1-action and triggers a soft reset to all TRU reg- isters. 0 No action                            |

Table 8-7: TRU\_GCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Non-MMR Enable. The TRU_GCTL.EN bit is read/write and must be set for the TRU to propagate trig- ger events. All TRU register read/write operations continue to operate independent of the TRU_GCTL.EN bit. 0 No trigger events |

## Master Trigger Register

The TRU master trigger register ( TRU\_MTR ) permits trigger generation through software by writing a trigger master ID value to one of the four fields in the TRU\_MTR register. If the global lock is enabled ( SPU\_CTL.GLCK bit =1) and the TRU\_GCTL.LOCK bit is set, the TRU\_MTR register is read only. Note this register is primarily used for debug to trigger a TRU output

Figure 8-4: TRU\_MTR Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000003_276dbcc9f76a17f976eca0ada4cd06ed2f973685483dcbc238e75244441fa885.png)

Table 8-8: TRU\_MTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | MTR3       | Master Trigger Register 3. The TRU_MTR.MTR3 bit field is the trigger master ID value for master 3. 0 No master specified                              |
| 23:16 (R/W)        | MTR2       | Master Trigger Register 2. The TRU_MTR.MTR2 bit field is the trigger master ID value for master 2.                                                    |
| 15:8 (R/W)         | MTR1       | Master Trigger Register 1. The TRU_MTR.MTR1 bit field is the trigger master ID value for master 1. 0 No master specified                              |
| 7:0 (R/W)          | MTR0       | Master Trigger Register 0. The TRU_MTR.MTR0 bit field is the trigger master ID value for master 0. 0 No master specified 1-139 Range of valid masters |

## Slave Select Register

The TRU slave select registers ( TRU\_SSR[n] ) each provide slave selection and register locking.

Figure 8-5: TRU\_SSR[n] Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000004_af9f7e7c2fa7e77c8a2b5fe4bf300a799e2852e4e20a2f58ff7c66e6c71a8dd7.png)

Table 8-9: TRU\_SSR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | SSRn Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the TRU_SSR[n].LOCK bit is enabled, the TRU_SSR[n] register is read only. 0 Unlock register                                                                                                                                                   |
| 7:0 (R/W)          | SSR        | SSRn Slave Select. The TRU_SSR[n] register selects the trigger master ID to which the trigger slave re- sponds. For example, when a TRU_SSR[n] register is set to respond to trigger master ID n, a trigger that is generated by trigger master ID n results in a trigger out to the slave. 0 No master specified |
|                    |            | 1-139 Range of valid masters                                                                                                                                                                                                                                                                                      |

## Status Information Register

The TRU status register ( TRU\_STAT ) contains the status of TRU\_MTR and TRU\_SSR[n] register writes and status of bus read/write errors.

Figure 8-6: TRU\_STAT Register Diagram

![Image](11_Trigger_Routing_Unit_(TRU)_artifacts/image_000005_66bca088eb4e637c63a7fda815442797703f2734c80c6f0a6ae584eabcaf870f.png)

Table 8-10: TRU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | ADDRERR    | Address Error Status. The TRU_STAT.ADDRERR bit is set when an invalid address is provided for an MMRaccess while the TRU is selected. Writing a one to this bit clears the error indi- cation. The TRU_ERRADDR register also is updated when an address error occurs dur- ing an MMRaccess while the TRU is selected. 0 No error |
| 0 (R/W1C)          | LWERR      | Lock Write Error Status. If TRU_STAT.LWERR is set, a lock write error has occurred. Writing a one to this bit clears the error indication. 0 No error 1 Error occurred                                                                                                                                                           |