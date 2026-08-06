# System Event Controller (SEC) and Generic Interrupt Controller (GIC)

<!-- source: 009_System_Event_Controller_SEC_and_Generic_Interrupt_Controller.pdf | original pages 211–292 -->

## 7   System Event Controller (SEC) and Generic Interrupt Controller (GIC)

There are two interrupt controllers-a generic interrupt controller (GIC) for the Arm core and the system event controller (SEC) for the SHARC cores.

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

The SEC/GIC Interrupt Signal Flow figure shows an overview of the interrupt systems.

Figure 7-1: SEC/GIC Interrupt Signal Flow

<!-- image -->

## ADSP-2159x\_SC592\_SC594 SEC Register List

The System Event Controller (SEC) manages the system fault sources, including control features such as enable/ disable and active/pending source status. For more information on SEC functionality, see the SEC register descriptions.

Table 7-1: ADSP-2159x\_SC592\_SC594 SEC Register List

| Name         | Description                   |
|--------------|-------------------------------|
| SEC_CACT[n]  | SCI Active Register n         |
| SEC_CCTL[n]  | SCI Control Register n        |
| SEC_CGMSK[n] | SCI Group Mask Register n     |
| SEC_CPLVL[n] | SCI Priority Level Register n |
| SEC_CPMSK[n] | SCI Priority Mask Register n  |
| SEC_CPND[n]  | Core Pending Register n       |
| SEC_CSID[n]  | SCI Source ID Register n      |
| SEC_CSTAT[n] | SCI Status Register n         |
| SEC_END      | Global End Register           |

Table 7-1: ADSP-2159x\_SC592\_SC594 SEC Register List (Continued)

| Name           | Description                               |
|----------------|-------------------------------------------|
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

## ADSP-2159x\_SC592\_SC594 SEC Interrupt List

Table 7-2: ADSP-2159x\_SC592\_SC594 SEC Interrupt List

|   Interrupt ID | Name     | Description   | Sensitivity   | DMA Channel   |
|----------------|----------|---------------|---------------|---------------|
|              0 | SEC0_ERR | SEC0 Error    | Level         |               |

## ADSP-2159x\_SC592\_SC594 SEC Trigger List

Table 7-3: ADSP-2159x\_SC592\_SC594 SEC Trigger List Generators

|   Trigger ID | Name       | Description   | Sensitivity   |
|--------------|------------|---------------|---------------|
|           81 | SEC0_FAULT | SEC0 Fault    | Edge          |

Table 7-4: ADSP-2159x\_SC592\_SC594 SEC Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

## Combined SEC and GIC Interrupt List

The Combined SEC and GIC Interrupt List table provides a complete list of interrupts supported by the SHARC+ processor and Arm core. Note that the DAI has its own system interrupt controllers. For more information see the DAI System Interrupt Controller (SIC).

Table 7-5: Combined SEC and GIC Interrupt List

| SEC ID   | Interrupt Number SHARC+   | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description       | Interrupt Number Arm   |
|----------|---------------------------|----------|----------|--------------------------|-------------------------------------|------------------------|
| NA       | NA                        | NA       | GIC      | GIC0_SOFT00              | Arm Software Interrupt 0            | 0                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT01              | Arm Software Interrupt 1            | 1                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT02              | Arm Software Interrupt 2            | 2                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT03              | Arm Software Interrupt 3            | 3                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT04              | Arm Software Interrupt 4            | 4                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT05              | Arm Software Interrupt 5            | 5                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT06              | Arm Software Interrupt 6            | 6                      |
| NA       | NA                        | NA       | GIC      | GIC0_SOFT07              | Arm Software Interrupt 7            | 7                      |
| NA       | NA                        | NA       | NA       | NA                       | Reserved                            | 8-31                   |
| 0        | 0                         | NA       | SEC      | SEC0_ERR                 | SEC0 Error                          | 32                     |
| 1        | 1                         | NA       | CGU      | CGU0_EVT                 | CGU0 Event                          | 33                     |
| 2        | 2                         | NA       | CGU      | CGU1_EVT                 | CGU1 Event                          | 34                     |
| 3        | 3                         | NA       | WDOG     | WDOG0_EXP                | WDOG0 Expiration                    | 35                     |
| 4        | 4                         | NA       | WDOG     | WDOG1_EXP                | WDOG1 Expiration                    | 36                     |
| 5        | 5                         | NA       | WDOG     | WDOG2_EXP                | WDOG2 Expiration                    | 37                     |
| 6        | 6                         | NA       | OTPC     | OTPC0_ERR                | OTPC0 Dual bit Error Interrupt      | 38                     |
| 7        | 7                         | NA       | TMU      | TMU0_FAULT               | TMU0 Fault Event                    | 39                     |
| 8        | 8                         | NA       | TMU      | TMU0_ALERT               | TMU0 Alert Event                    | 40                     |
| 9        | 9                         | NA       | TAPC     | TAPC0_KEYFAIL            | Test/User Key Fail Interrupt        | 41                     |
| 10       | 10                        | NA       | L2CTL    | L2CTL0_ECC_ERR           | L2CTL0 ECC Error                    | 42                     |
|          |                           | Reserved |          |                          |                                     | Reserved               |
| 12       | 12                        | NA       | L2CTL    | L2CTL0_EVT               | L2CTL0 Scrub/Initialization Done    | 44                     |
| 13       | 13                        | NA       | MEC      | MEC1_EEIRQ0              | MEC1 L2CTL0/CAN ECC Error to Core 1 | Reserved               |
|          |                           | Reserved |          |                          |                                     | Reserved               |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description                            | Interrupt Number Arm   |
|----------|---------------------------|----------|----------|--------------------------|----------------------------------------------------------|------------------------|
|       15 |                        15 | NA       | MEC      | MEC1_PEIRQ0              | MEC1 Core 0 Parity Error Inter- rupt to Core 1           | Reserved               |
|       16 |                        16 | NA       | MEC      | MEC1_PEIRQ1              | MEC1 Core 1 Parity Error Inter- rupt to Core 1           | Reserved               |
|       17 |                        17 | NA       | MEC      | MEC1_PEIRQ2              | MEC1 Core 2 Parity Error Inter- rupt to Core 1           | Reserved               |
|       18 |                        18 | NA       | MEC      | MEC1_PEIRQ3              | MEC1 Systerm Peripheral Parity Error Interrupt to Core 1 | Reserved               |
|       19 |                        19 | NA       | MEC      | MEC0_EEIRQ0              | Reserved                                                 | 51                     |
|          |                           | Reserved |          |                          |                                                          | Reserved               |
|       21 |                        21 | NA       | MEC      | MEC0_PEIRQ0              | Reserved                                                 | 53                     |
|       22 |                        22 | NA       | MEC      | MEC0_PEIRQ1              | Reserved                                                 | 54                     |
|       23 |                        23 | NA       | MEC      | MEC0_PEIRQ2              | Reserved                                                 | 55                     |
|       24 |                        24 | NA       | MEC      | MEC0_PEIRQ3              | Reserved                                                 | 56                     |
|       25 |                        25 | NA       | MEC      | MEC2_EEIRQ0              | MEC2 L2CTL0/CAN ECC Error to Core 2                      | Reserved               |
|          |                           | Reserved |          |                          |                                                          | Reserved               |
|       27 |                        27 | NA       | MEC      | MEC2_PEIRQ0              | MEC2 Core 0 Parity Error Inter- rupt to Core 2           | Reserved               |
|       28 |                        28 | NA       | MEC      | MEC2_PEIRQ1              | MEC2 Core 1 Parity Error Inter- rupt to Core 2           | Reserved               |
|       29 |                        29 | NA       | MEC      | MEC2_PEIRQ2              | MEC2 Core 2 Parity Error Inter- rupt to Core 2           | Reserved               |
|       30 |                        30 | NA       | MEC      | MEC2_PEIRQ3              | MEC2 Systerm Peripheral Parity Error Interrupt to Core 2 | Reserved               |
|       31 |                        31 | NA       | CORE     | C0_L2CC                  | CORE0 L2CC Interrupt                                     | 63                     |
|       32 |                        32 | NA       | CORE     | C0_PMUIRQ                | Reserved                                                 | 64                     |
|       33 |                        33 | NA       | CORE     | C0_INITDONE              | C0 Memory Initialization Done                            | 65                     |
|       34 |                        34 | NA       | CORE     | C1_IRQ0                  | CORE1 Data Read Interrupt                                | 66                     |
|       35 |                        35 | NA       | CORE     | C1_IRQ1                  | CORE1 Data Write Interrupt                               | 67                     |
|       36 |                        36 | NA       | CORE     | C1_IRQ2                  | CORE1 Instruction Read Interrupt                         | 68                     |
|       37 |                        37 | NA       | SYSTEM   | SOFT8                    | Software-driven Interrupt 8                              | Reserved               |
|       38 |                        38 | NA       | CORE     | C2_IRQ0                  | CORE2 Data Read Interrupt                                | 70                     |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description                             | Interrupt Number Arm   |
|----------|---------------------------|----------|----------|--------------------------|-----------------------------------------------------------|------------------------|
|       39 |                        39 | NA       | CORE     | C2_IRQ1                  | CORE2 Data Write Interrupt                                | 71                     |
|       40 |                        40 | NA       | CORE     | C2_IRQ2                  | CORE2 Instruction Read Interrupt                          | 72                     |
|       41 |                        41 | NA       | SYSTEM   | SOFT9                    | Software-driven Interrupt 9                               | Reserved               |
|       42 |                        42 | NA       | CORE     | CORE1_DS_SLV_ACC_ INTR   | Core 1 Completer Access request, if core is in Deep Sleep | 74                     |
|       43 |                        43 | NA       | CORE     | CORE2_DS_SLV_ACC_ INTR   | Core2 Completer Access request, if core is in Deep Sleep  | 75                     |
|       44 |                        44 | NA       | DAI      | DAI0_IRQH                | DAI0 High Priority Interrupt                              | 76                     |
|       45 |                        45 | NA       | DAI      | DAI1_IRQH                | DAI1 High Priority Interrupt                              | 77                     |
|       46 |                        46 | NA       | DAI      | DAI0_IRQL                | DAI0 Low Priority Interrupt                               | 78                     |
|       47 |                        47 | NA       | DAI      | DAI1_IRQL                | DAI1 Low Priority Interrupt                               | 79                     |
|       48 |                        48 | NA       | TIMER    | TIMER0_TMR0              | TIMER0 Timer 0                                            | 80                     |
|       49 |                        49 | NA       | TIMER    | TIMER0_TMR1              | TIMER0 Timer 1                                            | 81                     |
|       50 |                        50 | NA       | TIMER    | TIMER0_TMR2              | TIMER0 Timer 2                                            | 82                     |
|       51 |                        51 | NA       | TIMER    | TIMER0_TMR3              | TIMER0 Timer 3                                            | 83                     |
|       52 |                        52 | NA       | TIMER    | TIMER0_TMR4              | TIMER0 Timer 4                                            | 84                     |
|       53 |                        53 | NA       | TIMER    | TIMER0_TMR5              | TIMER0 Timer 5                                            | 85                     |
|       54 |                        54 | NA       | TIMER    | TIMER0_TMR6              | TIMER0 Timer 6                                            | 86                     |
|       55 |                        55 | NA       | TIMER    | TIMER0_TMR7              | TIMER0 Timer 7                                            | 87                     |
|       56 |                        56 | NA       | TIMER    | TIMER0_TMR8              | TIMER0 Timer 8                                            | 88                     |
|       57 |                        57 | NA       | TIMER    | TIMER0_TMR9              | TIMER0 Timer 9                                            | 89                     |
|       58 |                        58 | NA       | TIMER    | TIMER0_TMR10             | TIMER0 Timer 10                                           | 90                     |
|       59 |                        59 | NA       | TIMER    | TIMER0_TMR11             | TIMER0 Timer 11                                           | 91                     |
|       60 |                        60 | NA       | TIMER    | TIMER0_TMR12             | TIMER0 Timer 12                                           | 92                     |
|       61 |                        61 | NA       | TIMER    | TIMER0_TMR13             | TIMER0 Timer 13                                           | 93                     |
|       62 |                        62 | NA       | TIMER    | TIMER0_TMR14             | TIMER0 Timer 14                                           | 94                     |
|       63 |                        63 | NA       | TIMER    | TIMER0_TMR15             | TIMER0 Timer 15                                           | 95                     |
|       64 |                        64 | NA       | TIMER    | TIMER0_STAT              | TIMER0 Status                                             | 96                     |
|       65 |                        65 | NA       | PINT     | PINT0_BLOCK              | PINT0 Pin Interrupt Block                                 | 97                     |
|       66 |                        66 | NA       | PINT     | PINT1_BLOCK              | PINT1 Pin Interrupt Block                                 | 98                     |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description   |   Interrupt Number Arm |
|----------|---------------------------|----------|----------|--------------------------|---------------------------------|------------------------|
|       67 |                        67 | NA       | PINT     | PINT2_BLOCK              | PINT2 Pin Interrupt Block       |                     99 |
|       68 |                        68 | NA       | PINT     | PINT3_BLOCK              | PINT3 Pin Interrupt Block       |                    100 |
|       69 |                        69 | NA       | PINT     | PINT4_BLOCK              | PINT4 Pin Interrupt Block       |                    101 |
|       70 |                        70 | NA       | PINT     | PINT5_BLOCK              | PINT5 Pin Interrupt Block       |                    102 |
|       71 |                        71 | NA       | PINT     | PINT6_BLOCK              | PINT6 Pin Interrupt Block       |                    103 |
|       72 |                        72 | NA       | PINT     | PINT7_BLOCK              | PINT7 Pin Interrupt Block       |                    104 |
|       81 |                        81 | NA       | SPORT    | SPORT0_A_DMA             | SPORT0 ChannelADMA              |                    113 |
|       82 |                        82 | NA       | SPORT    | SPORT0_A_STAT            | SPORT0 Channel A Status         |                    114 |
|       83 |                        83 | NA       | SPORT    | SPORT0_B_DMA             | SPORT0 ChannelBDMA              |                    115 |
|       84 |                        84 | NA       | SPORT    | SPORT0_B_STAT            | SPORT0 Channel B Status         |                    116 |
|       85 |                        85 | NA       | SPORT    | SPORT1_A_DMA             | SPORT1 ChannelADMA              |                    117 |
|       86 |                        86 | NA       | SPORT    | SPORT1_A_STAT            | SPORT1 Channel A Status         |                    118 |
|       87 |                        87 | NA       | SPORT    | SPORT1_B_DMA             | SPORT1 ChannelBDMA              |                    119 |
|       88 |                        88 | NA       | SPORT    | SPORT1_B_STAT            | SPORT1 Channel B Status         |                    120 |
|       89 |                        89 | NA       | SPORT    | SPORT2_A_DMA             | SPORT2 ChannelADMA              |                    121 |
|       90 |                        90 | NA       | SPORT    | SPORT2_A_STAT            | SPORT2 Channel A Status         |                    122 |
|       91 |                        91 | NA       | SPORT    | SPORT2_B_DMA             | SPORT2 ChannelBDMA              |                    123 |
|       92 |                        92 | NA       | SPORT    | SPORT2_B_STAT            | SPORT2 Channel B Status         |                    124 |
|       93 |                        93 | NA       | SPORT    | SPORT3_A_DMA             | SPORT3 ChannelADMA              |                    125 |
|       94 |                        94 | NA       | SPORT    | SPORT3_A_STAT            | SPORT3 Channel A Status         |                    126 |
|       95 |                        95 | NA       | SPORT    | SPORT3_B_DMA             | SPORT3 ChannelBDMA              |                    127 |
|       96 |                        96 | NA       | SPORT    | SPORT3_B_STAT            | SPORT3 Channel B Status         |                    128 |
|       97 |                        97 | NA       | SPORT    | SPORT4_A_DMA             | SPORT4 ChannelADMA              |                    129 |
|       98 |                        98 | NA       | SPORT    | SPORT4_A_STAT            | SPORT4 Channel A Status         |                    130 |
|       99 |                        99 | NA       | SPORT    | SPORT4_B_DMA             | SPORT4 ChannelBDMA              |                    131 |
|      100 |                       100 | NA       | SPORT    | SPORT4_B_STAT            | SPORT4 Channel B Status         |                    132 |
|      101 |                       101 | NA       | SPORT    | SPORT5_A_DMA             | SPORT5 ChannelADMA              |                    133 |
|      102 |                       102 | NA       | SPORT    | SPORT5_A_STAT            | SPORT5 Channel A Status         |                    134 |
|      103 |                       103 | NA       | SPORT    | SPORT5_B_DMA             | SPORT5 ChannelBDMA              |                    135 |
|      104 |                       104 | NA       | SPORT    | SPORT5_B_STAT            | SPORT5 Channel B Status         |                    136 |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description   |   Interrupt Number Arm |
|----------|---------------------------|----------|----------|--------------------------|---------------------------------|------------------------|
|      105 |                       105 | NA       | SPORT    | SPORT6_A_DMA             | SPORT6 ChannelADMA              |                    137 |
|      106 |                       106 | NA       | SPORT    | SPORT6_A_STAT            | SPORT6 Channel A Status         |                    138 |
|      107 |                       107 | NA       | SPORT    | SPORT6_B_DMA             | SPORT6 ChannelBDMA              |                    139 |
|      108 |                       108 | NA       | SPORT    | SPORT6_B_STAT            | SPORT6 Channel B Status         |                    140 |
|      109 |                       109 | NA       | SPORT    | SPORT7_A_DMA             | SPORT7 ChannelADMA              |                    141 |
|      110 |                       110 | NA       | SPORT    | SPORT7_A_STAT            | SPORT7 Channel A Status         |                    142 |
|      111 |                       111 | NA       | SPORT    | SPORT7_B_DMA             | SPORT7 ChannelBDMA              |                    143 |
|      112 |                       112 | NA       | SPORT    | SPORT7_B_STAT            | SPORT7 Channel B Status         |                    144 |
|      113 |                       113 | NA       | SPORT    | GLOB- AL_SPORT_INT0_DM A | SPORT DMAGROUP0 interrupt       |                    145 |
|      114 |                       114 | NA       | SPORT    | GLOB- AL_SPORT_INT1_DM A | SPORT DMAGROUP1 interrupt       |                    146 |
|      115 |                       115 | NA       | SPORT    | GLOB- AL_SPORT_INT2_DM A | SPORT DMAGROUP2 interrupt       |                    147 |
|      116 |                       116 | NA       | SPORT    | GLOB- AL_SPORT_INT3_DM A | SPORT DMAGROUP3 interrupt       |                    148 |
|      117 |                       117 | NA       | LP       | LP0_DMA                  | LP0 DMAData                     |                    149 |
|      118 |                       118 | NA       | LP       | LP0_STAT                 | LP0 Status                      |                    150 |
|      119 |                       119 | NA       | LP       | LP1_DMA                  | LP1 DMAData                     |                    151 |
|      120 |                       120 | NA       | LP       | LP1_STAT                 | LP1 Status                      |                    152 |
|      121 |                       121 | NA       | SPI      | SPI0_TXDMA               | SPI0 TX DMAChannel              |                    153 |
|      122 |                       122 | NA       | SPI      | SPI0_RXDMA               | SPI0 RX DMAChannel              |                    154 |
|      123 |                       123 | NA       | SPI      | SPI0_STAT                | SPI0 Status                     |                    155 |
|      124 |                       124 | NA       | SPI      | SPI0_ERR                 | SPI0 Error                      |                    156 |
|      125 |                       125 | NA       | SPI      | SPI1_TXDMA               | SPI1 TX DMAChannel              |                    157 |
|      126 |                       126 | NA       | SPI      | SPI1_RXDMA               | SPI1 RX DMAChannel              |                    158 |
|      127 |                       127 | NA       | SPI      | SPI1_STAT                | SPI1 Status                     |                    159 |
|      128 |                       128 | NA       | SPI      | SPI1_ERR                 | SPI1 Error                      |                    160 |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description   | Interrupt Number Arm   |
|----------|---------------------------|----------|----------|--------------------------|---------------------------------|------------------------|
|      129 |                       129 | NA       | SPI      | SPI2_TXDMA               | SPI2 TX DMAChannel              | 161                    |
|      130 |                       130 | NA       | SPI      | SPI2_RXDMA               | SPI2 RX DMAChannel              | 162                    |
|      131 |                       131 | NA       | SPI      | SPI2_STAT                | SPI2 Status                     | 163                    |
|      132 |                       132 | NA       | SPI      | SPI2_ERR                 | SPI2 Error                      | 164                    |
|      133 |                       133 | NA       | SPI      | SPI3_TXDMA               | SPI3TX DMAChannel               | 165                    |
|      134 |                       134 | NA       | SPI      | SPI3_RXDMA               | SPI3 RX DMAChannel              | 166                    |
|      135 |                       135 | NA       | SPI      | SPI3_STAT                | SPI3 Status                     | 167                    |
|      136 |                       136 | NA       | SPI      | SPI3_ERR                 | SPI3 Error                      | 168                    |
|      137 |                       137 | NA       | OSPI     | OSPI0_IRQ                | OSPI Interrupt request          | 169                    |
|      138 |                       138 | NA       | UART     | UART0_TXDMA              | UART0 TransmitDMA               | 170                    |
|      139 |                       139 | NA       | UART     | UART0_RXDMA              | UART0 ReceiveDMA                | 171                    |
|      140 |                       140 | NA       | UART     | UART0_STAT               | UART0 Status                    | 172                    |
|      141 |                       141 | NA       | UART     | UART1_TXDMA              | UART1 TransmitDMA               | 173                    |
|      142 |                       142 | NA       | UART     | UART1_RXDMA              | UART1 ReceiveDMA                | 174                    |
|      143 |                       143 | NA       | UART     | UART1_STAT               | UART1 Status                    | 175                    |
|      144 |                       144 | NA       | UART     | UART2_TXDMA              | UART2 TransmitDMA               | 176                    |
|      145 |                       145 | NA       | UART     | UART2_RXDMA              | UART2 ReceiveDMA                | 177                    |
|      146 |                       146 | NA       | UART     | UART2_STAT               | UART2 Status                    | 178                    |
|      147 |                       147 | NA       | UART     | UART3_TXDMA              | UART3 TransmitDMA               | 179                    |
|      148 |                       148 | NA       | UART     | UART3_RXDMA              | UART3 ReceiveDMA                | 180                    |
|      149 |                       149 | NA       | UART     | UART3_STAT               | UART3 Status                    | 181                    |
|      150 |                       150 | NA       | TWI      | TWI0_DATA                | TWI0 Data Interrupt             | 182                    |
|      151 |                       151 | NA       | TWI      | TWI1_DATA                | TWI1 Data Interrupt             | 183                    |
|      152 |                       152 | NA       | TWI      | TWI2_DATA                | TWI2 Data Interrupt             | 184                    |
|      153 |                       153 | NA       | TWI      | TWI3_DATA                | TWI3 Data Interrupt             | 185                    |
|      154 |                       154 | NA       | TWI      | TWI4_DATA                | TWI4 Data Interrupt             | 186                    |
|      155 |                       155 | NA       | TWI      | TWI5_DATA                | TWI5 Data Interrupt             | 187                    |
|      156 |                       156 | NA       | CNT      | CNT0_STAT                | CNT0 Status                     | 188                    |
|      157 |                       157 | NA       | CTI      | ECT_C0_EVT               | Reserved                        | 189                    |
|      158 |                       158 | NA       | CTI      | ECT_C1_EVT               | Core 1 CTI Event (CTI1)         | Reserved               |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description        | Interrupt Number Arm   |
|----------|---------------------------|----------|----------|--------------------------|--------------------------------------|------------------------|
|      159 |                       159 | NA       | CTI      | ECT_C2_EVT               | Core 2 CTI Event (CTI2)              | Reserved               |
|      160 |                       160 | NA       | PKIC     | PKIC0_IRQ                | Public Key Interrupt (PKA, TRNG, SL) | 192                    |
|      161 |                       161 | NA       | PKTE     | PKTE0_IRQ                | Security Packet Engine Interrupt     | 193                    |
|      162 |                       162 | NA       | TRU      | TRU0_SLV0                | Reserved                             | 194                    |
|      163 |                       163 | NA       | TRU      | TRU0_SLV1                | Reserved                             | 195                    |
|      164 |                       164 | NA       | TRU      | TRU0_SLV2                | Reserved                             | 196                    |
|      165 |                       165 | NA       | TRU      | TRU0_SLV3                | Reserved                             | 197                    |
|      166 |                       166 | NA       | FIR      | C1_FIR0_DMA              | Core 1 FIR0DMA                       | 198                    |
|      167 |                       167 | NA       | FIR      | C1_FIR0_STAT             | Core 1 FIR0 Status                   | 199                    |
|      168 |                       168 | NA       | IIR      | C1_IIR0_DMA              | Core 1 IIR0DMA                       | 200                    |
|      169 |                       169 | NA       | IIR      | C1_IIR0_STAT             | Core 1 IIR0 Status                   | 201                    |
|      170 |                       170 | NA       | IIR      | C1_IIR1_DMA              | Core 1 IIR1DMA                       | 202                    |
|      171 |                       171 | NA       | IIR      | C1_IIR1_STAT             | Core 1 IIR1 Status                   | 203                    |
|      172 |                       172 | NA       | IIR      | C1_IIR2_DMA              | Core 1 IIR2DMA                       | 204                    |
|      173 |                       173 | NA       | IIR      | C1_IIR2_STAT             | Core 1 IIR2 Status                   | 205                    |
|      174 |                       174 | NA       | IIR      | C1_IIR3_DMA              | Core 1 IIR3DMA                       | 206                    |
|      175 |                       175 | NA       | IIR      | C1_IIR3_STAT             | Core 1 IIR3 Status                   | 207                    |
|      176 |                       176 | NA       | FIR      | C2_FIR0_DMA              | Core 2 FIR0DMA                       | 208                    |
|      177 |                       177 | NA       | FIR      | C2_FIR0_STAT             | Core 2 FIR0 Status                   | 209                    |
|      178 |                       178 | NA       | IIR      | C2_IIR0_DMA              | Core 2 IIR0DMA                       | 210                    |
|      179 |                       179 | NA       | IIR      | C2_IIR0_STAT             | Core 2 IIR0 Status                   | 211                    |
|      180 |                       180 | NA       | IIR      | C2_IIR1_DMA              | Core 2 IIR1DMA                       | 212                    |
|      181 |                       181 | NA       | IIR      | C2_IIR1_STAT             | Core 2 IIR1 Status                   | 213                    |
|      182 |                       182 | NA       | IIR      | C2_IIR2_DMA              | Core 2 IIR2DMA                       | 214                    |
|      183 |                       183 | NA       | IIR      | C2_IIR2_STAT             | Core 2 IIR2 Status                   | 215                    |
|      184 |                       184 | NA       | IIR      | C2_IIR3_DMA              | Core 2 IIR3DMA                       | 216                    |
|      185 |                       185 | NA       | IIR      | C2_IIR3_STAT             | Core 2 IIR3 Status                   | 217                    |
|      186 |                       186 | NA       | HADC     | HADC0_EVT                | HADC0 Interrupt                      | 218                    |
|      187 |                       187 | NA       | MLB      | MLB0_INT0                | MLB0 AHB interrupt 0                 | 219                    |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description    |   Interrupt Number Arm |
|----------|---------------------------|----------|----------|--------------------------|----------------------------------|------------------------|
|      188 |                       188 | NA       | MLB      | MLB0_INT1                | MLB0 AHB Interrupt 1             |                    220 |
|      189 |                       189 | NA       | MLB      | MLB0_STAT                | MLB0 Status                      |                    221 |
|      190 |                       190 | NA       | CRC      | CRC0_ERR                 | CRC0 Error                       |                    222 |
|      191 |                       191 | NA       | CRC      | CRC1_ERR                 | CRC1 Error                       |                    223 |
|      192 |                       192 | NA       | CRC      | MDMA0_SRC                | MDMASource 0 (EnhBW DMA)/CRC0 In |                    224 |
|      193 |                       193 | NA       | CRC      | MDMA0_DST                | MDMADest 0 (Enh BWDMA)/ CRC0 Out |                    225 |
|      194 |                       194 | NA       | CRC      | MDMA1_SRC                | MDMASource 1 (EnhBW DMA)/CRC1 In |                    226 |
|      195 |                       195 | NA       | CRC      | MDMA1_DST                | MDMADest 1 (Enh BWDMA)/ CRC1 Out |                    227 |
|      196 |                       196 | NA       | CRC      | CRC0_DCNTEXP             | CRC0 Datacount expiration        |                    228 |
|      197 |                       197 | NA       | CRC      | CRC1_DCNTEXP             | CRC1 Datacount expiration        |                    229 |
|      198 |                       198 | NA       | CRC      | CRC2_ERR                 | CRC2 Error                       |                    230 |
|      199 |                       199 | NA       | CRC      | CRC3_ERR                 | CRC3 Error                       |                    231 |
|      200 |                       200 | NA       | CRC      | MDMA4_SRC                | MDMASource 4 (EnhBW DMA)/CRC2 In |                    232 |
|      201 |                       201 | NA       | CRC      | MDMA4_DST                | MDMADest 4 (Enh BWDMA)/ CRC2 Out |                    233 |
|      202 |                       202 | NA       | CRC      | MDMA5_SRC                | MDMASource 5 (EnhBW DMA)/CRC3 In |                    234 |
|      203 |                       203 | NA       | CRC      | MDMA5_DST                | MDMADest 5 (Enh BWDMA)/ CRC3 Out |                    235 |
|      204 |                       204 | NA       | CRC      | CRC2_DCNTEXP             | CRC2 Datacount expiration        |                    236 |
|      205 |                       205 | NA       | CRC      | CRC3_DCNTEXP             | CRC3 Datacount expiration        |                    237 |
|      206 |                       206 | NA       | MDMA     | MDMA2_SRC                | Enh BWDMAChannel 0               |                    238 |
|      207 |                       207 | NA       | MDMA     | MDMA2_DST                | Enh BWDMAChannel 1               |                    239 |
|      208 |                       208 | NA       | MDMA     | MDMA3_SRC                | Max BWDMAChannel 0               |                    240 |
|      209 |                       209 | NA       | MDMA     | MDMA3_DST                | Max BWDMAChannel 1               |                    241 |
|      210 |                       210 | NA       | EMDMA    | EMDMA0_DONE              | EMDMA0 DMADone                   |                    242 |
|      211 |                       211 | NA       | EMDMA    | EMDMA1_DONE              | EMDMA1 DMADone                   |                    243 |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module     | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description             |   Interrupt Number Arm |
|----------|---------------------------|----------|------------|--------------------------|-------------------------------------------|------------------------|
|      212 |                       212 | NA       | MDMA       | MDMA7_SRC                | Max BWDMA1Channel 0                       |                    244 |
|      213 |                       213 | NA       | MDMA       | MDMA7_DST                | Max BWDMA1Channel 1                       |                    245 |
|      214 |                       214 | NA       | MDMA       | MDMA6_SRC                | Enh BWDMA1Channel 0                       |                    246 |
|      215 |                       215 | NA       | MDMA       | MDMA6_DST                | Enh BWDMA1Channel 1                       |                    247 |
|      216 |                       216 | NA       | SPU        | SPU0_INT                 | SPU0 Event                                |                    248 |
|      217 |                       217 | NA       | SMPU       | SMPU0_AGGR_INT           | SMPU Aggregated Event                     |                    249 |
|      218 |                       218 | NA       | EPPI       | EPPI0_CH0_DMA            | EPPI0 DMAChannel 0                        |                    250 |
|      219 |                       219 | NA       | EPPI       | EPPI0_CH1_DMA            | EPPI0 DMAChannel 1                        |                    251 |
|      220 |                       220 | NA       | EPPI       | EPPI0_STAT               | EPPI0 Status                              |                    252 |
|      221 |                       221 | NA       | EMAC       | EMAC0_STAT               | EMAC0 Status                              |                    253 |
|      222 |                       222 | NA       | EMAC       | EMAC0_PWR                | EMAC0 Power                               |                    254 |
|      223 |                       223 | NA       | EMAC       | EMAC0_DMA0               | EMAC0 DMA0                                |                    255 |
|      224 |                       224 | NA       | EMAC       | EMAC0_DMA1               | EMAC0 DMA1                                |                    256 |
|      225 |                       225 | NA       | EMAC       | EMAC0_DMA2               | EMAC0 DMA2                                |                    257 |
|      226 |                       226 | NA       | EMAC       | EMAC0_MAC                | EMAC0 MAC                                 |                    258 |
|      227 |                       227 | NA       | EMAC       | EMAC1_STAT               | EMAC1 Status                              |                    259 |
|      228 |                       228 | NA       | EMAC       | EMAC1_PWR                | EMAC1 Power                               |                    260 |
|      229 |                       229 | NA       | EMAC       | EMAC1_DMA                | EMAC1DMA                                  |                    261 |
|      230 |                       230 | NA       | EMAC       | EMAC1_MAC                | EMAC1 MAC                                 |                    262 |
|      231 |                       231 | NA       | CAN        | CAN0_WU_IRQ              | CAN0 wakeup interrupts                    |                    263 |
|      232 |                       232 | NA       | RE- SERVED | RESERVED0                | Reserved                                  |                    264 |
|      233 |                       233 | NA       | CAN        | CAN0_IRQ                 | CAN0 interrupts                           |                    265 |
|      234 |                       234 | NA       | RE- SERVED | RESERVED1                | Reserved                                  |                    266 |
|      235 |                       235 | NA       | CAN        | CAN0_MSG_IRQ             | CAN0 message receive/transmit in- terrupt |                    267 |
|      236 |                       236 | NA       | CAN        | CAN1_WU_IRQ              | CAN1 wakeup interrupts                    |                    268 |
|      237 |                       237 | NA       | RE- SERVED | RESERVED2                | Reserved                                  |                    269 |
|      238 |                       238 | NA       | CAN        | CAN1_IRQ                 | CAN1 interrupts                           |                    270 |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ | SEA ID   | Module     | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description             | Interrupt Number Arm   |
|----------|---------------------------|----------|------------|--------------------------|-------------------------------------------|------------------------|
|      239 |                       239 | NA       | RE- SERVED | RESERVED3                | Reserved                                  | 271                    |
|      240 |                       240 | NA       | CAN        | CAN1_MSG_IRQ             | CAN1 message receive/transmit in- terrupt | 272                    |
|      241 |                       241 | NA       | USB        | USB0_INT                 | USB0 Interrupt                            | 273                    |
|      242 |                       242 | NA       | TRU        | TRU0_SLV4                | TRU0 Interrupt 4                          | Reserved               |
|      243 |                       243 | NA       | TRU        | TRU0_SLV5                | TRU0 Interrupt 5                          | Reserved               |
|      244 |                       244 | NA       | TRU        | TRU0_SLV6                | TRU0 Interrupt 6                          | Reserved               |
|      245 |                       245 | NA       | TRU        | TRU0_SLV7                | TRU0 Interrupt 7                          | Reserved               |
|      246 |                       246 | NA       | TRU        | TRU0_SLV8                | TRU0 Interrupt 8                          | Reserved               |
|      247 |                       247 | NA       | TRU        | TRU0_SLV9                | TRU0 Interrupt 9                          | Reserved               |
|      248 |                       248 | NA       | TRU        | TRU0_SLV10               | TRU0 Interrupt 10                         | Reserved               |
|      249 |                       249 | NA       | TRU        | TRU0_SLV11               | TRU0 Interrupt 11                         | Reserved               |
|          |                           | Reserved |            |                          |                                           | Reserved               |
|      251 |                       251 | 0        | FIR        | C1_FIR0_BUS_ERR          | Core 1 FIR0 bus error                     | 283                    |
|      251 |                       252 | 1        | IIR        | C1_IIR0_BUS_ERR          | Core 1 IIR0 bus error                     | 284                    |
|      251 |                       253 | 2        | IIR        | C1_IIR1_BUS_ERR          | Core 1 IIR1 bus error                     | 285                    |
|      251 |                       254 | 3        | IIR        | C1_IIR2_BUS_ERR          | Core 1 IIR2 bus error                     | 286                    |
|      251 |                       255 | 4        | IIR        | C1_IIR3_BUS_ERR          | Core 1 IIR3 bus error                     | 287                    |
|      251 |                       256 | 5        | FIR        | C2_FIR0_BUS_ERR          | Core 2 FIR0 bus error                     | 288                    |
|      251 |                       257 | 6        | IIR        | C2_IIR0_BUS_ERR          | Core 2 IIR0 bus error                     | 289                    |
|      251 |                       258 | 7        | IIR        | C2_IIR1_BUS_ERR          | Core 2 IIR1 bus error                     | 290                    |
|      251 |                       259 | 8        | IIR        | C2_IIR2_BUS_ERR          | Core 2 IIR2 bus error                     | 291                    |
|      251 |                       260 | 9        | IIR        | C2_IIR3_BUS_ERR          | Core 2 IIR3 bus error                     | 292                    |
|      252 |                       261 | 0        | SPI        | SPI0_TXDMA_ERR           | SPI0 TX DMAChannel Error                  | 293                    |
|      252 |                       262 | 1        | SPI        | SPI0_RXDMA_ERR           | SPI0 RX DMAChannel Error                  | 294                    |
|      252 |                       263 | 2        | SPI        | SPI1_TXDMA_ERR           | SPI1 TX DMAChannel Error                  | 295                    |
|      252 |                       264 | 3        | SPI        | SPI1_RXDMA_ERR           | SPI1 RX DMAChannel Error                  | 296                    |
|      252 |                       265 | 4        | SPI        | SPI2_TXDMA_ERR           | SPI2 TX DMAChannel Error                  | 297                    |
|      252 |                       266 | 5        | SPI        | SPI2_RXDMA_ERR           | SPI2 RX DMAChannel Error                  | 298                    |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ |   SEA ID | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description   |   Interrupt Number Arm |
|----------|---------------------------|----------|----------|--------------------------|---------------------------------|------------------------|
|      252 |                       267 |        6 | SPI      | SPI3_TXDMA_ERR           | SPI3 TX DMAChannel Error        |                    299 |
|      252 |                       268 |        7 | SPI      | SPI3_RXDMA_ERR           | SPI3 RX DMAChannel Error        |                    300 |
|      252 |                       269 |        8 | UART     | UART0_TXDMA_ERR          | UART0 Transmit DMAError         |                    301 |
|      252 |                       270 |        9 | UART     | UART0_RXDMA_ERR          | UART0 Receive DMAError          |                    302 |
|      252 |                       271 |       10 | UART     | UART1_TXDMA_ERR          | UART1 Transmit DMAError         |                    303 |
|      252 |                       272 |       11 | UART     | UART1_RXDMA_ERR          | UART1 Receive DMAError          |                    304 |
|      252 |                       273 |       12 | UART     | UART2_TXDMA_ERR          | UART2 Transmit DMAError         |                    305 |
|      252 |                       274 |       13 | UART     | UART2_RXDMA_ERR          | UART2 Receive DMAError          |                    306 |
|      252 |                       275 |       14 | UART     | UART3_TXDMA_ERR          | UART3 Transmit DMAError         |                    307 |
|      252 |                       276 |       15 | UART     | UART3_RXDMA_ERR          | UART3 Receive DMAError          |                    308 |
|      252 |                       277 |       16 | LP       | LP0_DMA_ERR              | LP0 DMAData Error               |                    309 |
|      252 |                       278 |       17 | LP       | LP1_DMA_ERR              | LP1 DMAData Error               |                    310 |
|      252 |                       279 |       18 | EPPI     | EP- PI0_CH0_DMA_ERR      | EPPI0 DMAChannel 0 Error        |                    311 |
|      252 |                       280 |       19 | EPPI     | EP- PI0_CH1_DMA_ERR      | EPPI0 DMAChannel 1 Error        |                    312 |
|      253 |                       281 |        0 | CRC      | MDMA0_SRC_ERR            | Enh BWSource 0 DMAChannel Error |                    313 |
|      253 |                       282 |        1 | CRC      | MDMA0_DST_ERR            | Enh BWDest 0 DMAChannel Error   |                    314 |
|      253 |                       283 |        2 | CRC      | MDMA1_SRC_ERR            | Enh BWSource 1 DMAChannel Error |                    315 |
|      253 |                       284 |        3 | CRC      | MDMA1_DST_ERR            | Enh BWDest 1 DMAChannel Error   |                    316 |
|      253 |                       285 |        4 | MDMA     | MDMA2_SRC_ERR            | Enh BWDMAChannel 0 Error        |                    317 |
|      253 |                       286 |        5 | MDMA     | MDMA2_DST_ERR            | Enh BWDMAChannel 1 Error        |                    318 |
|      253 |                       287 |        6 | MDMA     | MDMA3_SRC_ERR            | Max BWDMAChannel 0 Error        |                    319 |
|      253 |                       288 |        7 | MDMA     | MDMA3_DST_ERR            | Max BWDMAChannel 1 Error        |                    320 |
|      253 |                       289 |        8 | CRC      | MDMA4_SRC_ERR            | Enh BWSource 4 DMAChannel Error |                    321 |
|      253 |                       290 |        9 | CRC      | MDMA4_DST_ERR            | Enh BWDest 4 DMAChannel Error   |                    322 |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ |   SEA ID | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description   |   Interrupt Number Arm |
|----------|---------------------------|----------|----------|--------------------------|---------------------------------|------------------------|
|      253 |                       291 |       10 | CRC      | MDMA5_SRC_ERR            | Enh BWSource 5 DMAChannel Error |                    323 |
|      253 |                       292 |       11 | CRC      | MDMA5_DST_ERR            | Enh BWDest 5 DMAChannel Error   |                    324 |
|      253 |                       293 |       12 | MDMA     | MDMA6_SRC_ERR            | Enh BWDMAChannel 4 Error        |                    325 |
|      253 |                       294 |       13 | MDMA     | MDMA6_DST_ERR            | Enh BWDMAChannel 5 Error        |                    326 |
|      253 |                       295 |       14 | MDMA     | MDMA7_SRC_ERR            | Max BWDMAChannel 4 Error        |                    327 |
|      253 |                       296 |       15 | MDMA     | MDMA7_DST_ERR            | Max BWDMAChannel 5 Error        |                    328 |
|      254 |                       297 |        0 | SPORT    | SPORT0_A_DMA_ERR         | SPORT0 Channel A DMAError       |                    329 |
|      254 |                       298 |        1 | SPORT    | SPORT0_B_DMA_ERR         | SPORT0 Channel B DMAError       |                    330 |
|      254 |                       299 |        2 | SPORT    | SPORT1_A_DMA_ERR         | SPORT1 Channel A DMAError       |                    331 |
|      254 |                       300 |        3 | SPORT    | SPORT1_B_DMA_ERR         | SPORT1 Channel B DMAError       |                    332 |
|      254 |                       301 |        4 | SPORT    | SPORT2_A_DMA_ERR         | SPORT2 Channel A DMAError       |                    333 |
|      254 |                       302 |        5 | SPORT    | SPORT2_B_DMA_ERR         | SPORT2 Channel B DMAError       |                    334 |
|      254 |                       303 |        6 | SPORT    | SPORT3_A_DMA_ERR         | SPORT3 Channel A DMAError       |                    335 |
|      254 |                       304 |        7 | SPORT    | SPORT3_B_DMA_ERR         | SPORT3 Channel B DMAError       |                    336 |
|      254 |                       305 |        8 | SPORT    | SPORT4_A_DMA_ERR         | SPORT4 Channel A DMAError       |                    337 |
|      254 |                       306 |        9 | SPORT    | SPORT4_B_DMA_ERR         | SPORT4 Channel B DMAError       |                    338 |
|      254 |                       307 |       10 | SPORT    | SPORT5_A_DMA_ERR         | SPORT5 Channel A DMAError       |                    339 |
|      254 |                       308 |       11 | SPORT    | SPORT5_B_DMA_ERR         | SPORT5 Channel B DMAError       |                    340 |
|      254 |                       309 |       12 | SPORT    | SPORT6_A_DMA_ERR         | SPORT6 Channel A DMAError       |                    341 |
|      254 |                       310 |       13 | SPORT    | SPORT6_B_DMA_ERR         | SPORT6 Channel B DMAError       |                    342 |
|      254 |                       311 |       14 | SPORT    | SPORT7_A_DMA_ERR         | SPORT7 Channel A DMAError       |                    343 |
|      254 |                       312 |       15 | SPORT    | SPORT7_B_DMA_ERR         | SPORT7 Channel B DMAError       |                    344 |
|      255 |                       314 |        1 | SWU      | SWU1_EVT                 | SWU1 Event CL2_0                |                    346 |
|      255 |                       315 |        2 | SWU      | SWU2_EVT                 | SWU2 Event DL2_0                |                    347 |
|      255 |                       316 |        3 | SWU      | SWU7_EVT                 | SWU7 Event SHARC0 S1            |                    348 |
|      255 |                       317 |        4 | SWU      | SWU8_EVT                 | SWU8 Event SHARC0 S2            |                    349 |
|      255 |                       318 |        5 | SWU      | SWU9_EVT                 | SWU9 Event SHARC1 S1            |                    350 |
|      255 |                       319 |        6 | SWU      | SWU10_EVT                | SWU10 Event SHARC2 S2           |                    351 |

Table 7-5: Combined SEC and GIC Interrupt List (Continued)

|   SEC ID |   Interrupt Number SHARC+ |   SEA ID | Module   | SEC/GIC Interrupt Name   | SEC/GIC Interrupt Description   |   Interrupt Number Arm |
|----------|---------------------------|----------|----------|--------------------------|---------------------------------|------------------------|
|      255 |                       320 |        7 | SWU      | SWU11_EVT                | SWU11 Event SMMR                |                    352 |
|      255 |                       321 |        8 | SWU      | SWU12_EVT                | SWU12 Event SPI2/OSPI           |                    353 |
|      255 |                       322 |        9 | SWU      | SWU13_EVT                | SWU13 Event DMC0                |                    354 |
|      255 |                       323 |       10 | SWU      | SWU3_EVT                 | SWU3 Event CL2_1                |                    355 |
|      255 |                       324 |       11 | SWU      | SWU4_EVT                 | SWU4 Event DL2_1                |                    356 |
|      255 |                       325 |       12 | SWU      | SWU5_EVT                 | SWU5 Event CL2_2                |                    357 |

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

<!-- image -->

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

<!-- image -->

## Fault Management

System sources can be enabled as fault sources in the SEC\_SCTL[n] register. When a source enabled as a fault moves to pending, it is forwarded to the SFI as a fault indication. The pending bit ( SEC\_FSTAT.PND ) indicates a source has signaled a fault assertion but it has not yet triggered the event actions (if delay is enabled). The SEC fault interface sets the SEC\_FSTAT.PND bit when the fault source ID register ( SEC\_FSID ) is updated on assertion of a fault source input. The system source pending triggers a fault pending and after a programmable delay the fault moves to active. Event actions then execute if appropriate action is not taken by the core. The SEC\_FSTAT.ACT bit indicates that the SEC has received a fault source input, the delay has expired, and the fault actions are enabled.

The SEC\_FSTAT.NPND bit indicates if one or more sources have signaled a fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC\_FSTAT.NPND bit when the fault interface detects assertion of any enabled fault source input, while either the SEC\_FSTAT.PND or SEC\_FSTAT.ACT bits are set. The SEC clears the SEC\_FSTAT.NPND bit when there are no fault sources waiting.

A fault indication from an external device can also be detected on sampling the fault signals. When a fault is detected the SEC\_FSTAT.ACT and SEC\_FSID.FEXT bits are set. The assertion of either signal results in a fault input detection.

The SEC\_FEND register receives a fault end indication from the core. The core writes the SID of the fault to the SEC\_FEND register. If the SID matches the value in the SEC\_FSID register, the SEC\_FSTAT.PND and SEC\_FSTAT.ACT bits are cleared.

## SEC Core Interface (SCI)

The SCI manages communication between the corresponding core and the SEC. The SEC prioritizer (SPR) of the SCI receives pending, active, and priority information from the SSI for each system event source assigned to this SCI. The SPR determines the highest priority pending system event and the SCI determines whether it propagates to the core. The SCI maintains the coherency for the system event service model implemented on the connected core.

Figure 7-4: SCI Overview Block Diagram

<!-- image -->

## SEC Source Interface (SSI)

The SSI manages all of the system event sources. It maintains the status of each source in the corresponding SEC\_SSTAT[n] register. The corresponding SEC\_SCTL[n] register manages the control of each source. A pending and enabled event passes its indication and priority to the SCI to which it is assigned for further processing.

Figure 7-5: SSI Overview Block Diagram

<!-- image -->

## SEC Architectural Concepts

The following sections describe SEC architectural features.

## System Interrupt Acknowledge

A system interrupt acknowledge occurs when the core provides an indication that it has acquired the SID of the interrupt last issued by the SEC. The SEC core interface option allows generation by:

- A completer port write to the SEC\_CSID[n] register.
- The assertion of an input acknowledge signal (the connected core generates the signal).

## System Interrupt Groups

System sources can be assigned to groups using the SEC\_SCTL[n].GRP bit field. Source groups allow fast context switching for system interrupts at each SCI. The SEC\_CGMSK[n] register allows quick masking of interrupt groups of unlimited size with a single write operation.

## System Interrupt Flow

An enabled and asserted system interrupt source is latched at the SSI and routed to the appropriate SCI based on the core target select ( SEC\_SCTL[n].CTG ) bit field setting. The SEC priority ordering determines the highest priority pending system interrupt and the SCI updates the SEC\_CPND[n].SID and SEC\_CACT[n].PRIO bit field values. The SCI compares the SEC\_CPND[n] register value against the highest priority active source in the SEC\_CACT[n] register).

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

Each system interrupt source has its own programmable priority level which is configured using the SEC\_SCTL[n].PRIO bit field. The SCI evaluates the priority of all pending sources to determine the source of the highest priority pending system interrupt for forwarding to the attached core. If more than one source of the pending system interrupt has the same priority setting, the SCI chooses the one with the lowest SID. For example, if SID 0, SID 1, and SID 2 are all pending and have the same priority setting, the SCI chooses SID 0 as the highestpriority source.

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
10. ADDITIONAL INFORMATION: The SEC\_FCTL.FIEN bit should be set only while the SEC\_FCTL.EN bit is low. If the SEC\_FCTL.EN bit is already high and the SEC\_FCTL.FIEN bit needs to be set, the SEC\_FCTL.EN bit should be cleared first. Fault input can only be enabled when Fault mode is selected by the SEC\_FCTL.CMS bit.
4. Program the required fault delay to the SEC\_FDLY.COUNT bit field if a delay between fault source assertion and the fault response is required.
5. Configure the SEC\_FCTL register to enable the SEC.

ADDITIONAL INFORMATION: The SEC\_FCTL.EN bit should be set only while the SEC\_FSTAT.ACT bit is low.

6. Write to the control register of a specific source register using the SEC\_SCTL[n] register to enable the source as a fault.

## Configuring a System Source to Interrupt a Core

To configure a system source to interrupt a core, the SEC itself must be enabled with the source interface (SSI) and core interface (SCI) properly initialized. Specifically, the SCI must be set up to accept interrupt signaling from the SEC and pass them to the specified core, and the SSI must properly enable each of the peripheral interrupt sources to generate interrupt signals and optionally define a priority scheme that overrides the default priority settings. In summary:

1. Write to the SEC\_GCTL register to enable the SEC.
2. Write to the appropriate SCI SEC\_CCTL[n] register to enable SEC interrupts to be sent to that core.
3. Write to the appropriate SSI SEC\_SCTL[n] register to enable that peripheral as an interrupt source and to set the core target field to map the source to the desired SCI.
4. (Optional) By default, all the SEC interrupts are grouped as a single priority level, so passing of peripheral interrupt requests from the SEC is based solely on the default enumerated source ID. By programming the SEC\_CPLVL[n].PLVL register, interrupt sources can be grouped into priority levels within the SEC such that arbitration is first performed by source ID within a grouped priority level before proceeding to the next priority level, thus providing the flexibility to have lower-priority interrupt sources considered before higherpriority sources.

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

```
*pREG_SEC0_GCTL = BITM_SEC_GCTL_EN;
```

```
*pREG_RCU0_CTL |= BITM_RCU_CTL_SRSTREQEN; *pREG_SEC0_FCTL |= BITM_SEC_FCTL_SREN;
```

3. Configure the SEC\_SCTL[n].SEN and SEC\_SCTL[n].FEN bits in the registers to determine how the fault source is handled. To configure the WDOG as the fault source, program the register. The program can configure any interrupt as the fault source by programming the corresponding register.

```
*pREG_SEC0_SCTL3 = BITM_SEC_SCTL_FEN|BITM_SEC_SCTL_SEN;
```

ADDITIONAL INFORMATION: The SEC ID corresponding to WDOG0 is 3, as indicated in the Combined SEC and GIC Interrupt List .

4. Write to the enable bit.

```
*pREG_SEC0_FCTL |= BITM_SEC_FCTL_EN;
```

## SEC Programming Restrictions

Setting the SEC\_FCTL.EN bit while the SEC\_FSTAT.ACT bit is high can result in unpredictable behavior. To avoid this issue, set the SEC\_FCTL.EN bit while the SEC\_FSTAT.ACT bit is low. The SEC\_FSTAT.ACT bit is only set when the SEC\_FCTL.EN bit is high. Therefore, the problem can only occur if the SEC\_FCTL.EN bit transitions from 1 to 0 and then to 1 again.

Writing to SEC\_FEND to end a fault with both the SEC\_FCTL.FOEN bit and the SEC\_FCTL.FIEN bit set can result in erroneous external fault detection. If this operation (ending a fault) and configuration (fault input and fault output enabled) are required by the application, clear the SEC\_FCTL.FOEN bit prior to writing to SEC\_FEND . The recommended sequence for ending a fault with the SEC\_FCTL.FIEN or SEC\_FCTL.FOEN ==1 is as follows:

1. Clear the SEC\_FCTL.FOEN bit.
2. Write to the SEC\_FEND register.
3. Set the SEC\_FCTL.FOEN bit.

## ADSP-2159x\_SC592\_SC594 SEC Register Descriptions

System Event Controller (SEC) contains the following registers.

Table 7-6: ADSP-2159x\_SC592\_SC594 SEC Register List

| Name         | Description                   |
|--------------|-------------------------------|
| SEC_CACT[n]  | SCI Active Register n         |
| SEC_CCTL[n]  | SCI Control Register n        |
| SEC_CGMSK[n] | SCI Group Mask Register n     |
| SEC_CPLVL[n] | SCI Priority Level Register n |
| SEC_CPMSK[n] | SCI Priority Mask Register n  |
| SEC_CPND[n]  | Core Pending Register n       |

Table 7-6: ADSP-2159x\_SC592\_SC594 SEC Register List (Continued)

| Name           | Description                               |
|----------------|-------------------------------------------|
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

<!-- image -->

Table 7-7: SEC\_CACT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | PRIO       | Highest Active IRQ Priority. The SEC_CACT[n].PRIO indicates the priority value of the highest priority active interrupt for core n.   |
| 7:0 (R/NW)         | SID        | Highest Active IRQ Source ID. The SEC_CACT[n].SID identifies the source ID value of the highest priority active interrupt for core n. |

## SCI Control Register n

The SEC control register ( SEC\_CCTL[n] ) contains SCI control bits for all system sources.

Figure 7-7: SEC\_CCTL[n] Register Diagram

<!-- image -->

Table 7-8: SEC\_CCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CCTL[n].LOCK bit is enabled, the SEC_CCTL[n] register is read only.                                         |
| 16 (R/W)           | NMIEN      | NMI Enable. The SEC_CCTL[n].NMIEN bit controls NMI propagation to the core. When the SEC_CCTL[n].NMIEN bit is enabled, the SCI allows NMIs to propagate to the core for servicing. |
| 12 (R0/W)          | WFI        | Wait For Idle. When set, the SEC_CCTL[n].WFI bit forces the SCI to wait for indication of core idle before the SCI resumes activity. 0 No Action                                   |

Table 7-8: SEC\_CCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R0/W)           | RESET      | Reset. When set, the SEC_CCTL[n].RESET bit resets all SCI registers to their default val- ues. 0 No Action 1 Reset                                                                                                                                                                                                                                         |
| 0 (R/W)            | EN         | Enable. The SEC_CCTL[n].EN bit controls operation of the SCI. Clearing the SEC_CCTL[n].EN bit halts the execution of the SCI without resetting status regis- ters. (The INT signal to a core is not affected.) Setting the SEC_CCTL[n].EN bit enables the SCI to begin or resume operation with the current configuration and sta- tus. 0 Disable 1 Enable |

## SCI Group Mask Register n

The SEC SCI group mask register ( SEC\_CGMSK[n] ) contains selections for a group mask, an ungroup mask, and a register lock. This register contains the system interrupt group masks for the connected core. The core uses the SEC\_CGMSK[n].UGRP and SEC\_CGMSK[n].GRP fields to mask (disable) interrupts from the specified groups.

Figure 7-8: SEC\_CGMSK[n] Register Diagram

<!-- image -->

Table 7-9: SEC\_CGMSK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CGMSK[n].LOCK bit is enabled, the SEC_CGMSK[n] register is read only.                                                        | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CGMSK[n].LOCK bit is enabled, the SEC_CGMSK[n] register is read only.                                                        |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                   | Unlock                                                                                                                                                                                              |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                   | Lock                                                                                                                                                                                                |
| 8 (R/W)            | UGRP       | Ungrouped Mask. The SEC_CGMSK[n].UGRP bit masks interrupts (if set) for the ungrouped inter- rupt sources for core n.                                                                               | Ungrouped Mask. The SEC_CGMSK[n].UGRP bit masks interrupts (if set) for the ungrouped inter- rupt sources for core n.                                                                               |
| 8 (R/W)            | UGRP       | 0                                                                                                                                                                                                   | Unmask Ungrouped Sources                                                                                                                                                                            |
| 8 (R/W)            | UGRP       | 1                                                                                                                                                                                                   | Mask Ungrouped Sources                                                                                                                                                                              |
| 3:0 (R/W)          | GRP        | Grouped Mask. The SEC_CGMSK[n].GRP field selects a group of interrupt sources to mask for core n. (For more information about interrupt source groups, see the SEC_SCTL[n] reg- ister description.) | Grouped Mask. The SEC_CGMSK[n].GRP field selects a group of interrupt sources to mask for core n. (For more information about interrupt source groups, see the SEC_SCTL[n] reg- ister description.) |
| 3:0 (R/W)          | GRP        | 0                                                                                                                                                                                                   | No groups masked                                                                                                                                                                                    |
| 3:0 (R/W)          | GRP        | 1                                                                                                                                                                                                   | Mask group 0                                                                                                                                                                                        |
| 3:0 (R/W)          | GRP        | 2                                                                                                                                                                                                   | Mask group 1                                                                                                                                                                                        |
| 3:0 (R/W)          | GRP        | 3                                                                                                                                                                                                   | Mask groups 0, 1                                                                                                                                                                                    |

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

<!-- image -->

Lock

Figure 7-9: SEC\_CPLVL[n] Register Diagram

Table 7-10: SEC\_CPLVL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPLVL[n].LOCK bit is enabled, the SEC_CPLVL[n] register is read only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPLVL[n].LOCK bit is enabled, the SEC_CPLVL[n] register is read only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Unlock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Lock                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 2:0 (R/W)          | PLVL       | Priority Levels. The SEC_CPLVL[n].PLVL field serves to divide the total number of interrupt pri- ority levels into sub-levels. The sub-level priority resolution provides the tie breaker for simultaneously pending interrupts assigned to the same interrupt level. The sub-level priority value specifies the number of MSBs (minus 1) designated to interrupt levels, while the remaining LSBs are designated for sub-level specification. For example, if the SEC_CPLVL[n].PLVL field is set to two, the result is four pri- ority levels are specified, because only the two MSBs are used for preemption evalua- tion. The remaining bits of the priority setting are used for sub-level prioritization. | Priority Levels. The SEC_CPLVL[n].PLVL field serves to divide the total number of interrupt pri- ority levels into sub-levels. The sub-level priority resolution provides the tie breaker for simultaneously pending interrupts assigned to the same interrupt level. The sub-level priority value specifies the number of MSBs (minus 1) designated to interrupt levels, while the remaining LSBs are designated for sub-level specification. For example, if the SEC_CPLVL[n].PLVL field is set to two, the result is four pri- ority levels are specified, because only the two MSBs are used for preemption evalua- tion. The remaining bits of the priority setting are used for sub-level prioritization. |
| 2:0 (R/W)          | PLVL       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 1 MSBs (2 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 2:0 (R/W)          | PLVL       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 2 MSBs (4 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 2:0 (R/W)          | PLVL       | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 3 MSBs (8 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 2:0 (R/W)          | PLVL       | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 4 MSBs (16 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 2:0 (R/W)          | PLVL       | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 5 MSBs (32 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 2:0 (R/W)          | PLVL       | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 6 MSBs (64 priority levels)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 7-10: SEC\_CPLVL[n] Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration        |
|-----------|------------|--------------------------------|
| (Access)  |            |                                |
|           |            | 6 7 MSBs (128 priority levels) |
|           |            | 7 8 MSBs (256 priority levels) |

## SCI Priority Mask Register n

The SEC SCI priority mask register ( SEC\_CPMSK[n] ) contains the SCI priority mask for core n and includes a register lock.

Figure 7-10: SEC\_CPMSK[n] Register Diagram

<!-- image -->

Table 7-11: SEC\_CPMSK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPMSK[n].LOCK bit is enabled, the SEC_CPMSK[n] register is read only. 0 Unlock                                                                               |
| 7:0 (R/W)          | PRIO       | IRQ Priority Mask. The SEC_CPMSK[n].PRIO contains the system interrupt priority mask for core n. The core uses the SEC_CPMSK[n].PRIO field to mask (block) interrupts below the specified level. 0 Priority level 0 (highest) 1-254 |

## Core Pending Register n

The SCI pending interrupt register ( SEC\_CPND[n] ) contains the source ID and priority of the highest priority pending interrupt detected by the SEC prioritizer.

Figure 7-11: SEC\_CPND[n] Register Diagram

<!-- image -->

Table 7-12: SEC\_CPND[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | PRIO       | Highest Pending IRQ Priority. The SEC_CPND[n].PRIO indicates the priority value of the highest priority pend- ing interrupt for core n.   |
| 7:0 (R/NW)         | SID        | Highest Pending IRQ Source ID. The SEC_CPND[n].SID identifies the source ID value of the highest priority pend- ing interrupt for core n. |

## SCI Source ID Register n

The SCI source ID register ( SEC\_CSID[n] ) contains the source ID of the interrupt last issued to core n. The SEC\_CSID[n] register value is loaded by the SCI when a system interrupt indication is sent to core n. The SCI does not change the SEC\_CSID[n] until after the interface receives an interrupt acknowledge from core n. Writing to the SEC\_CSID[n] register generates an interrupt acknowledge, but does not update the value in the register.

Figure 7-12: SEC\_CSID[n] Register Diagram

<!-- image -->

Table 7-13: SEC\_CSID[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID.                                                                       |
| (R/NW)             |            | The SEC_CSID[n].SID bit is the source ID of the interrupt last issued to core n. |

## SCI Status Register n

The SCI status register ( SEC\_CSTAT[n] ) contains status bits, indicating the operational status of the SCI.

Figure 7-13: SEC\_CSTAT[n] Register Diagram

<!-- image -->

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

<!-- image -->

Table 7-15: SEC\_END Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID IRQ to End.                                                         |
| (R/W)              |            | The SEC_END.SID bit field contains the source ID interrupt service end value. |

## Fault COP Period Register

The SEC fault COP period register ( SEC\_FCOPP ) contains the width value (count in (SEC) clock cycles) for the high and low phase of the computer operating properly (COP) toggled output on the COP pin. Note that the actual high/low phase value is the SEC\_FCOPP.COUNT programmed value plus 1.

Figure 7-15: SEC\_FCOPP Register Diagram

<!-- image -->

Table 7-16: SEC\_FCOPP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | COUNT      | Fault COP Period. The SEC_FCOPP.COUNT bit field is the width value for the high and low phase of the computer operating properly (COP) toggled output on the COP pin. |

## Fault COP Period Current Register

The SEC fault COP period current register ( SEC\_FCOPP\_CUR ) contains the active count (in (SEC) clock periods) for the current phase (high or low) of the computer operating properly (COP) toggled output on the COP pin. The SEC loads the SEC\_FCOPP\_CUR register from the SEC\_FCOPP register when the SEC\_FCOPP\_CUR.COUNT

field is cleared and the SEC is in COP mode ( SEC\_FCTL.CMS bit =1). The SEC decrements the SEC\_FCOPP\_CUR count each (SEC) clock cycle while SEC\_FCTL.CMS is set and the SEC\_FSTAT.ACT bit is not set.

Figure 7-16: SEC\_FCOPP\_CUR Register Diagram

<!-- image -->

Table 7-17: SEC\_FCOPP\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault COP Period. The SEC_FCOPP_CUR.COUNT bit field is the active count for the current phase (high or low) of the computer operating properly (COP) toggled output on the COP pin. |

## Fault Control Register

The SEC fault control register ( SEC\_FCTL ) contains fault control bits for all SEC channels. This register controls the operation of the System Fault Management Interface (SFI).

Figure 7-17: SEC\_FCTL Register Diagram

<!-- image -->

Table 7-18: SEC\_FCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_FCTL.LOCK bit is enabled, the SEC_FCTL register is read only. 0 UnLock 1 Lock                                                                                                                                                               |
| 13 (R/W)           | TES        | Trigger Event Select. The SEC_FCTL.TES bit selects the event that directs the SEC to assert trigger out- put. In fault pending mode, the SEC asserts trigger output when a fault is pending. In fault active mode, the SEC asserts trigger output when a fault is active. 0 Fault Active Mode 1 Fault Pending Mode |

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

<!-- image -->

Table 7-19: SEC\_FDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | COUNT      | Fault Delay. The SEC_FDLY.COUNT bit field is the number of (SEC) clock periods to delay from fault pending to fault active, when actions are enabled. |

## Fault Delay Current Register

The SEC fault delay current register ( SEC\_FDLY\_CUR ) contains the active count ( SEC\_FDLY\_CUR.COUNT field) in (SEC) clock periods for the delay from fault pending to fault active, when actions are enabled. The count is loaded from the SEC\_FDLY register when a fault becomes pending ( SEC\_FSTAT.PND bit is set). The SEC decrements the value in SEC\_FDLY\_CUR each (SEC) clock cycle while the SEC\_FSTAT.PND bit is set.

Figure 7-19: SEC\_FDLY\_CUR Register Diagram

<!-- image -->

Table 7-20: SEC\_FDLY\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault Delay. The SEC_FDLY_CUR.COUNT bit field is the active count in (SEC) clock periods for the delay from fault pending to fault active, when actions are enabled. |

## Fault End Register

The SEC fault end register ( SEC\_FEND ) contains fault source ID and internal/external fields. This register receives fault end indication from a core.

Figure 7-20: SEC\_FEND Register Diagram

<!-- image -->

Table 7-21: SEC\_FEND Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | FEXT       | Fault External. Setting the SEC_FEND.FEXT bit, when the SEC_FEND.SID field is cleared, clears an active fault from an external source. 0 Fault Internal 1 Fault External                                                                                        |
| 7:0 (R/W)          | SID        | Source ID. The SEC_FEND.SID identifies a fault to be ended as indicated to the SEC by the core. The core loads the SEC_FEND.SID field value. If the SEC_FEND.SID value matches the SEC_FSID.SID value, the SEC_FSTAT.PND bit and SEC_FSTAT.ACT bit are cleared. |

## Fault Source ID Register

The SEC fault source ID register ( SEC\_FSID ) contains a fault source ID and internal/external fields.

NOTE:These bits are not reset by system reset so that a fault that automatically triggers a system reset to avoid a fault may be analyzed after the reset occurs.

Figure 7-21: SEC\_FSID Register Diagram

<!-- image -->

Table 7-22: SEC\_FSID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/NW)          | FEXT       | Fault External. The SEC_FSID.FEXT bit indicates that the last active fault was asserted by an exter- nal device. The SEC sets the SEC_FSID.FEXT bit when the SEC_FSTAT.ACT bit is set by the fault input pins. The SEC_FSID.FEXT bit is cleared when the SEC_FSTAT.ACT bit is set by an internal fault or when the external fault is ended. When the SEC_FSID.FEXT bit is set, the SEC_FSID.SID is cleared. 0 Fault Internal |
| 7:0 (R/NW)         | SID        | Source ID. The SEC_FSID.SID identifies the fault assertion detected by the SEC fault inter- face. The SEC loads the SEC_FSID.SID field value when a system fault indication is asserted. The SEC fault interface does not change the SEC_FSID.SID value until the fault is no longer pending or active, as indicated by the SEC_FSTAT.PND bit and SEC_FSTAT.ACT bit being cleared in the SEC_FSTAT register.                 |

## Fault System Reset Delay Register

The SEC fault system reset delay register ( SEC\_FSRDLY ) contains the number ( SEC\_FSRDLY.COUNT field) of (SEC) clock periods for the delay from a fault becoming active to system reset request assertion, if enabled.

Figure 7-22: SEC\_FSRDLY Register Diagram

<!-- image -->

Table 7-23: SEC\_FSRDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 31:0               | COUNT      | Fault System Reset Delay. The SEC_FSRDLY.COUNT bit field is the number of (SEC) clock periods for the |
| (R/W)              |            | delay from a fault becoming active to system reset request assertion.                                 |

## Fault System Reset Delay Current Register

The SEC fault system reset delay current register ( SEC\_FSRDLY\_CUR ) contains the active count ( SEC\_FSRDLY\_CUR.COUNT field) in (SEC) clock periods for the delay from fault active to system reset assertion, if enabled. The count is loaded from the SEC\_FSRDLY register when a fault becomes active ( SEC\_FSTAT.ACT bit is set). The SEC decrements the value in SEC\_FSRDLY\_CUR each (SEC) clock cycle while the SEC\_FSTAT.ACT bit is set.

Figure 7-23: SEC\_FSRDLY\_CUR Register Diagram

<!-- image -->

Table 7-24: SEC\_FSRDLY\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault System Reset Delay. The SEC_FSRDLY_CUR.COUNT bit field is the active count in (SEC) clock periods for the delay from fault active to system reset assertion. |

## Fault Status Register

The SEC fault status register ( SEC\_FSTAT ) indicates the operational status of the SFI.

Figure 7-24: SEC\_FSTAT Register Diagram

<!-- image -->

Table 7-25: SEC\_FSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/NW)          | NPND       | Next Pending Fault. The SEC_FSTAT.NPND bit indicates that one or more sources have signaled fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC_FSTAT.NPND bit when the fault interface de- tects assertion of any enabled fault source input, while either the SEC_FSTAT.PND or SEC_FSTAT.ACT bits are set. The SEC clears the SEC_FSTAT.NPND bit when there are no fault sources waiting. | Next Pending Fault. The SEC_FSTAT.NPND bit indicates that one or more sources have signaled fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC_FSTAT.NPND bit when the fault interface de- tects assertion of any enabled fault source input, while either the SEC_FSTAT.PND or SEC_FSTAT.ACT bits are set. The SEC clears the SEC_FSTAT.NPND bit when there are no fault sources waiting. |
| 10 (R/NW)          | NPND       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Not Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
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

Figure 7-25: SEC\_GCTL Register Diagram

<!-- image -->

Table 7-26: SEC\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_GCTL.LOCK bit is enabled, the SEC_GCTL register is read only.                                                                                                                                                                                                    |
| 1 (R0/W)           | RESET      | Reset. The SEC_GCTL.RESET bit is write-1-action and triggers a soft reset to all SEC reg- isters.                                                                                                                                                                                                                                       |
| 0 (R/W)            | EN         | Enable. The SEC_GCTL.EN bit is read/write and must be set for the SEC to begin/resume SEC operation with the current configuration and status. Clearing the SEC_GCTL.EN bit halts the execution of the SFI and all SCIs. All SSIs remain active, along with all error detection, without resetting status registers. 0 Disable 1 Enable |

## Global Status Register

The SEC global status register ( SEC\_GSTAT ) contains global status bits for the SEC.

Figure 7-26: SEC\_GSTAT Register Diagram

<!-- image -->

Table 7-27: SEC\_GSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Lock Write Error. The SEC_GSTAT.LWERR bit indicates (when set) there was an attempted write to an SEC register while the SEC_GCTL.LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 30 (R/W1C)         | ADRERR     | Address Error. The SEC_GSTAT.ADRERR bit indicates that the SEC generated and address error. This status bit is sticky; write-1-to-clear it. 0 No Error 1 Error Occurred                                                                                                         |
| 23:16 (R/NW)       | SID        | Source ID for SSI Error. The SEC_GSTAT.SID bits indicate the source ID that generated the last SSI error conveyed in the SEC_GSTAT.ERRC field.                                                                                                                                  |
| 11:8 (R/NW)        | SCI        | SCI ID for SCI Error. The SEC_GSTAT.SCI bits indicate the number for the specific SCI that generated the last SCI error conveyed in the SEC_GSTAT.ERRC field.                                                                                                                   |

Table 7-27: SEC\_GSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:4                | ERRC       | Error Cause. When the SEC updates the SEC_GSTAT.ERR bit, the SEC updates the SEC_GSTAT.ERRC bits to indicate the error type. Note that for SCI errors, the error status represents an OR of all the errors from each                                                                                                                           | Error Cause. When the SEC updates the SEC_GSTAT.ERR bit, the SEC updates the SEC_GSTAT.ERRC bits to indicate the error type. Note that for SCI errors, the error status represents an OR of all the errors from each                                                                                                                           |
| 1 (R/W1C)          | ERR        | 3 Error. The SEC_GSTAT.ERR bit indicates an error has occurred in the SEC. When the SEC asserts this bit (=1), the SEC updates the SEC_GSTAT.ERRC field to indicate the corresponding error cause. Even if multiple errors occur, only the first error is cap- tured on assertion of this bit. This status bit is sticky; write-1-to-clear it. | 3 Error. The SEC_GSTAT.ERR bit indicates an error has occurred in the SEC. When the SEC asserts this bit (=1), the SEC updates the SEC_GSTAT.ERRC field to indicate the corresponding error cause. Even if multiple errors occur, only the first error is cap- tured on assertion of this bit. This status bit is sticky; write-1-to-clear it. |
|                    |            | 1 0                                                                                                                                                                                                                                                                                                                                            | 0 SFI Error SCI Error                                                                                                                                                                                                                                                                                                                          |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                              | SSI Error                                                                                                                                                                                                                                                                                                                                      |
|                    |            |                                                                                                                                                                                                                                                                                                                                                | Reserved                                                                                                                                                                                                                                                                                                                                       |
|                    |            |                                                                                                                                                                                                                                                                                                                                                | No Error                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                              | Error Occurred                                                                                                                                                                                                                                                                                                                                 |

## Global Raise Register

The SEC global raise register ( SEC\_RAISE ) contains a source ID event set-to-pending field ( SEC\_RAISE.SID ). When a source ID value is written to this field, the SEC raises the source's event status to pending.

Figure 7-27: SEC\_RAISE Register Diagram

<!-- image -->

Table 7-28: SEC\_RAISE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID.                                                                           |
| (R/W)              |            | The SEC_RAISE.SID bit field is the source ID of event that is set to pending status. |

## Source Control Register n

The SEC source control register ( SEC\_SCTL[n] ) contains control bits to configure the SEC event sources. This register controls the configuration of the corresponding SEC event source.

Figure 7-28: SEC\_SCTL[n] Register Diagram

<!-- image -->

Table 7-29: SEC\_SCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_SCTL[n].LOCK bit is enabled, the SEC_SCTL[n] register is read only. 0 Unlock 1 Lock                                                                                                           |
| 27:24 (R/W)        | CTG        | Core Target Select. The SEC_SCTL[n].CTG bits selects the specific SEC core interface to which the in- terrupt is mapped. Each system interrupt is mapped uniquely to one specific SEC core interface and (as a result) to a specific core. 1 CORE1 SHARC0 2 Reserved |

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
| 2 (R/W)            | SEN        | Source (signal) Enable. The SEC_SCTL[n].SEN bit controls whether the system event source input signal may affect the pending status of the source. Clearing the SEC_SCTL[n].SEN bit                        | Source (signal) Enable. The SEC_SCTL[n].SEN bit controls whether the system event source input signal may affect the pending status of the source. Clearing the SEC_SCTL[n].SEN bit                        |
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

<!-- image -->

Table 7-30: SEC\_SSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/NW)       | CHID       | Channel ID. The SEC_SSTAT[n].CHID bits indicate the ID of the specific source (from a set of sources sharing one SEC source interface input) that asserted the SEC source interface input. An SEC source interface input may support multiple system sources, in which case the assertion must be qualified by an identifier to determine the channel that gen- erated the assertion. The SEC_SSTAT[n].CHID field provides this value in the form of a numeric reference that is mapped to a specific interrupt source. The prioriti- zation for simultaneously asserted sources is according to ID, with 0 being the highest priority. The SEC_SSTAT[n].CHID is captured when the SEC source interface in- put is acknowledged. |
| 9 (R/W1C)          | ACT        | Active Source. The SEC_SSTAT[n].ACT bit indicates the source has been accepted by a core for servicing, but the service is not yet complete. An SEC_SSTAT[n].ACT bit is set by the SEC when the specific system interrupt is acknowledged by the core through the SEC core interface. An SEC_SSTAT[n].ACT bit is cleared by the SEC when the core provides interrupt service end indication for the specific system interrupt through the SEC core interface. Active                                                                                                                                                                                                                                                             |
| 9 (R/W1C)          | ACT        | 0 Not                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 9 (R/W1C)          | ACT        | 1 Active                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 7-30: SEC\_SSTAT[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W1C)          | PND        | Pending Source. The SEC_SSTAT[n].PND bit indicates the source has signaled an event request, but the event request has not been (or is not currently being) serviced. A SEC_SSTAT[n].PND bit is set by the SEC on detection of an assertion of the corre- sponding system source input. A SEC_SSTAT[n].PND bit is cleared by the SEC when the specific system event is acknowledged by the core through the SEC core in- terface or by a W1C operation. 0 Not Pending                                                                 |
| 5:4 (R/NW)         | ERRC       | Error Cause. When the SEC_SSTAT[n].ERR bit is asserted, the SEC updates SEC_SSTAT[n].ERRC field to convey the interrupt source error type. When the error type is source overflow, the status indicates that a source signal assertion occurred or an SEC raise operation was attempted while pending was already set. The source overflow is detected when the source is set for edge only. When the error type is end error, the status indicates that an end was received for a source while the SEC_SSTAT[n].ACT bit was not set. |
| 5:4 (R/NW)         | ERRC       | 0 Source Overflow Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 5:4 (R/NW)         | ERRC       | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 5:4 (R/NW)         | ERRC       | 2 End Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 5:4 (R/NW)         | ERRC       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 1 (R/W1C)          | ERR        | Error. The SEC_SSTAT[n].ERR bit indicates an error for a specific system interrupt source. When the SEC_SSTAT[n].ERR bit is set, the SEC updates the SEC_SSTAT[n].ERRC field to the value of the corresponding error cause. Even if multiple errors occur, only the first error is captured on assertion of the                                                                                                                                                                                                                       |
| 1 (R/W1C)          | ERR        | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 1 (R/W1C)          | ERR        | 1 Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

## GIC Overview

The generic interrupt controller (GIC) provides an interface to the uniprocessor Cortex A5 core in the processor and collects up to 270 interrupt requests from all processor system sources. In addition, the GIC also supports eight software generated interrupts that are internal to the Cortex A5 core and not connected to the SECs. All GIC interrupts are also connected to the SEC in the same order.

Each interrupt can be configured as a normal or a secure interrupt. Software force registers and software priority masking are also supported. The "Register Descriptions" section in this chapter provide brief descriptions of these ARM-based registers. For complete information refer to the ARM® Generic Interrupt Controller Architecture version 1.0 Architecture Specification .

## GIC Functional Description

The GIC splits logically into a GICPORT0 (distributor block) and one GICPORT1 (CPU interface blocks).

## General Interrupt Controller Port0 (GIC Distributer)

The distributor block provides a programming interface to perform the following tasks.

- Globally enable the forwarding of interrupts to the CPU interfaces
- Enable or disable each interrupt
- Set the priority level of each interrupt
- Set the target processor list of each interrupt
- Set each peripheral interrupt to be level-sensitive or edge-triggered
- Set each interrupt as either Group 0 or Group 1
- Forward an SGI to one or more target processors

In addition, the Distributor provides:

- Visibility of the state of each interrupt
- A mechanism for software to set or clear the pending state of a peripheral interrupt.

## General Interrupt Controller Port1 (GIC CPU)

GICPORT1 (CPU interface) block performs priority masking and preemption handling for a connected processor in the system. GICPORT1 supports 8 SGIs (software generated interrupts) and 262 SPIs (shared peripheral interrupts).

Each CPU interface provides a programming interface to perform the following tasks.

- Enable signaling of interrupt requests to the processor
- Acknowledge interrupts
- Indicate that interrupt processing is complete
- Set interrupt priority masks for the processor
- Define the preemption policy for the processor
- Determine the highest priority pending interrupt for the processor

## GIC Block Diagram

The GIC Block Diagram shows the event management architecture.

Figure 7-30: GIC Block Diagram

<!-- image -->

## ADSP-2159x\_SC592\_SC594 GICDST Register Descriptions

GIC Distributor Port (GICDST) contains the following registers.

Table 7-31: ADSP-2159x\_SC592\_SC594 GICDST Register List

| Name                   | Description                                            |
|------------------------|--------------------------------------------------------|
| GICDST_EN              | GIC Port 0 Enable                                      |
| GICDST_SGI_PRIO[n]     | Software Generated Interrupt Priority Register         |
| GICDST_SPI_PRIO[n]     | Shared Peripheral Interrupt Priority Register          |
| GICDST_SGI_ACTIVE      | Software Generated Interrupt Active Register           |
| GICDST_SGI_CTL         | Software Generated Interrupt Control Register          |
| GICDST_SGI_PND_CLR     | Software Generated Interrupt Clear-Pending Register    |
| GICDST_SGI_PND_SET     | Software Generated Interrupt Pending Set Register      |
| GICDST_SGI_SECURITY    | Software Generated Interrupt Security Register         |
| GICDST_SPI[n]          | Shared Peripheral Interrupt Register                   |
| GICDST_SPI_ACTIVE[n]   | Shared Peripheral Interrupt Active Register            |
| GICDST_SPI_CFG[n]      | Shared Peripheral Interrupt Configuration Register     |
| GICDST_SPI_EN_CLR[n]   | Shared Peripheral Interrupt Enable Clear Register      |
| GICDST_SPI_EN_SET[n]   | Shared Peripheral Interrupt Enable Set Register        |
| GICDST_SPI_PND_CLR[n]  | Shared Peripheral Interrupt Pending Clear Register     |
| GICDST_SPI_PND_SET[n]  | Shared Peripheral Interrupt Pending Set Register       |
| GICDST_SPI_SECURITY[n] | Shared Peripheral Interrupt Security Register          |
| GICDST_SPI_TRGT[n]     | Shared Peripheral Interrupt Processor Targets Register |

## GIC Port 0 Enable

The GICDST\_EN register enables global monitoring of the peripheral interrupt signals and forwarding pending interrupts to the CPU interfaces.

Figure 7-31: GICDST\_EN Register Diagram

<!-- image -->

Table 7-32: GICDST\_EN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | VALUE      | Global Interrupt Monitor Enable. The GICDST_EN.VALUE bit field enables global monitoring of the peripheral inter- rupt signals and forwarding pending interrupts to the CPU interfaces. |

## Software Generated Interrupt Priority Register

The GICDST\_SGI\_PRIO[n] register provides the 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the corresponding interrupt.

Figure 7-32: GICDST\_SGI\_PRIO[n] Register Diagram

<!-- image -->

Table 7-33: GICDST\_SGI\_PRIO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Software Generated Interrupt Priority. The GICDST_SGI_PRIO[n].VALUE bit field contains the 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the correspond- ing interrupt. |

## Shared Peripheral Interrupt Priority Register

The GICDST\_SPI\_PRIO[n] registers provide an 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the corresponding interrupt.

Figure 7-33: GICDST\_SPI\_PRIO[n] Register Diagram

<!-- image -->

Table 7-34: GICDST\_SPI\_PRIO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 0                  | VALUE      | Priority. The GICDST_SPI_PRIO[n].VALUE bit field stores the priority of the corre- sponding interrupt (byte offset 3 to Byte offset 0). |
| (R/W)              |            |                                                                                                                                         |

## Software Generated Interrupt Active Register

The GICDST\_SGI\_ACTIVE registers provide a Set-active bit for each interrupt that the GIC supports. Writing to a Set-active bit Activates the corresponding interrupt. These registers are used when preserving and restoring GIC state.

Figure 7-34: GICDST\_SGI\_ACTIVE Register Diagram

<!-- image -->

Table 7-35: GICDST\_SGI\_ACTIVE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 15:0               | VALUE      | SGI Active If N.                                                                                            |
| (R/NW)             |            | The GICDST_SGI_ACTIVE.VALUE bit field provides a Set-active bit for each in- terrupt that the GIC supports. |

## Software Generated Interrupt Control Register

The GICDST\_SGI\_CTL register controls the generation of SGIs.It is implementation defined whether this register has any effect when the forwarding of interrupts by Distributor is disabled by the GICD\_CTLR settings.

Figure 7-35: GICDST\_SGI\_CTL Register Diagram

<!-- image -->

Table 7-36: GICDST\_SGI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/W)        | TRGLSTFILT | Target List Filter. The GICDST_SGI_CTL.TRGLSTFILT bit field determines how the distributor must process the requested SGI.                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 25:24 (R/W)        | TRGLSTFILT | 0 Forward the interrupt to the CPU interfaces specified in the CPUTargetList field                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 25:24 (R/W)        | TRGLSTFILT | 1 Forward the interrupt to all CPU interfaces except that of the processor that requested the interrupt                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 25:24 (R/W)        | TRGLSTFILT | 2 Forward the interrupt only to the CPU interface of the processor that requested the interrupt                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 23:16 (R/W)        | CPUTRGTLST | CPU Target list. When the GICDST_SGI_CTL.CPUTRGTLST bit field TargetList Filter = 0b00, de- fines the CPU interfaces to which the Distributor must forward the interrupt. Each bit of the GICDST_SGI_CTL.CPUTRGTLST bit field refers to the corre- sponding CPU interface, for example CPUTargetList[0] corresponds to CPU interface 0. Setting a bit to 1 indicates that the interrupt must be forwarded to the correspond- ing interface. If this field is 0x00 when TargetListFilter is 0b00, the Distributor does not forward the interrupt to any CPU interface. |

Table 7-36: GICDST\_SGI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | SATT       | Security Value of the SGI. The GICDST_SGI_CTL.SATT bit is implemented only if the GIC includes the Se- curity Extensions. This field is writable only by a Secure access. Any Non-secure write to the GICD_SGIR generates an SGI only if the specified SGI is programmed as Group 1, regardless of the value of bit[15] of the write. 0 Forward the SGI specified in the SGIINTID field to a |
| 3:0                | SGIINTID   | The Interrupt ID of the SGI.                                                                                                                                                                                                                                                                                                                                                                 |

## Software Generated Interrupt Clear-Pending Register

The GICDST\_SGI\_PND\_CLR register provides a clear pending bit for each interrupt supported by the GIC. Writing 1 to a clear-pending bit clears the pending status of the corresponding peripheral interrupt. Reading a bit identifies whether the interrupt is pending.

Figure 7-36: GICDST\_SGI\_PND\_CLR Register Diagram

<!-- image -->

Table 7-37: GICDST\_SGI\_PND\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Software Generated Interrupt Clear-Pending. Writing 1 to a clear-pending bit in the GICDST_SGI_PND_CLR.VALUE bit field clears the pending status of the corresponding peripheral interrupt. Reading a bit iden- tifies whether the interrupt is pending. |

## Software Generated Interrupt Pending Set Register

The GICDST\_SGI\_PND\_SET register provides a set-pending bit for each interrupt supported by the GIC.

Figure 7-37: GICDST\_SGI\_PND\_SET Register Diagram

<!-- image -->

Table 7-38: GICDST\_SGI\_PND\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Software Generated Interrupt Set-Pending. Writing 1 to a Set-pending bit in the GICDST_SGI_PND_SET.VALUE bit field sets the status of the corresponding peripheral interrupt to pending. Reading a bit identifies whether the interrupt is pending. |

## Software Generated Interrupt Security Register

The GICDST\_SGI\_SECURITY registers provide a status bit for each interrupt supported by the GIC. Each bit controls whether the corresponding interrupt is in Group 0 or Group 1. Typically, when used with a processor that implements the ARM Security Extensions, Group 0 interrupts are Secure interrupts, and Group 1 interrupts are Non-secure interrupts,

Figure 7-38: GICDST\_SGI\_SECURITY Register Diagram

<!-- image -->

Table 7-39: GICDST\_SGI\_SECURITY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Software Generated Interrupt Security. Each bit in the GICDST_SGI_SECURITY.VALUE bit field controls whether the corresponding interrupt is in Group 0 or Group 1. |

## Shared Peripheral Interrupt Register

The GICDST\_SPI[n] register contains bits that provide the status of the SPI[987:0] inputs.

Figure 7-39: GICDST\_SPI[n] Register Diagram

<!-- image -->

Table 7-40: GICDST\_SPI[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | STAT       | Shared Peripheral Interrupt Status. The GICDST_SPI[n].STAT bit field returns the status of the SPI[987:0] inputs on the distributor where bit [x] = 0 SPI[x] is low and bit [x] = 1 SPI[x] is high. |

## Shared Peripheral Interrupt Active Register

The GICDST\_SPI\_ACTIVE[n] register provides an active bit for each interrupt supported by the GIC.

Figure 7-40: GICDST\_SPI\_ACTIVE[n] Register Diagram

<!-- image -->

Table 7-41: GICDST\_SPI\_ACTIVE[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Active Bits. The GICDST_SPI_ACTIVE[n].VALUE bit field contains an Active bit for each interrupt supported by the GIC. Reading an active bit identifies whether the corre- sponding interrupt is active (=1) or not active (=0). |
| (R/W)              |            |                                                                                                                                                                                                                                 |

## Shared Peripheral Interrupt Configuration Register

The GICDST\_SPI\_CFG[n] register provides a 2-bit Int\_config field for each interrupt supported by the GIC.

Figure 7-41: GICDST\_SPI\_CFG[n] Register Diagram

<!-- image -->

Table 7-42: GICDST\_SPI\_CFG[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Configuration. The GICDST_SPI_CFG[n].VALUE bit field identifies whether the corresponding interrupt is: edge-triggered or level-sensitive handled using the 1-N model or using the N-N model |

## Shared Peripheral Interrupt Enable Clear Register

The GICDST\_SPI\_EN\_CLR[n] register provides a clear-enable bit for each interrupt supported by the GIC.

Figure 7-42: GICDST\_SPI\_EN\_CLR[n] Register Diagram

<!-- image -->

Table 7-43: GICDST\_SPI\_EN\_CLR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Enable Clear Enable. Writing 1 to a GICDST_SPI_EN_CLR[n].VALUE bit disables forwarding of the corresponding interrupt to the CPU interfaces. Reading a bit identifies whether the in- terrupt is enabled. |

## Shared Peripheral Interrupt Enable Set Register

The GICDST\_SPI\_EN\_SET[n] register provides a set-enable bit for each interrupt supported by the GIC.

Figure 7-43: GICDST\_SPI\_EN\_SET[n] Register Diagram

<!-- image -->

Table 7-44: GICDST\_SPI\_EN\_SET[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Enable. Writing 1 to a GICDST_SPI_EN_SET[n].VALUE bit enables forwarding of the corresponding interrupt to the CPU interfaces. Reading a bit identifies whether the in- terrupt is enabled. |

## Shared Peripheral Interrupt Pending Clear Register

The GICDST\_SPI\_PND\_CLR[n] register provides a clear-pending bit for each interrupt supported by the GIC.

Figure 7-44: GICDST\_SPI\_PND\_CLR[n] Register Diagram

<!-- image -->

Table 7-45: GICDST\_SPI\_PND\_CLR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Clear Pending Interrupt. Writing 1 to a GICDST_SPI_PND_CLR[n].VALUE bit clears the pending status of the corresponding peripheral interrupt. Reading a bit identifies whether the inter- rupt is pending. |

## Shared Peripheral Interrupt Pending Set Register

The GICDST\_SPI\_PND\_SET[n] register provides a set-pending bit for each interrupt supported by the GIC.

Figure 7-45: GICDST\_SPI\_PND\_SET[n] Register Diagram

<!-- image -->

Table 7-46: GICDST\_SPI\_PND\_SET[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Set Pending Interrupt. Writing 1 to a GICDST_SPI_PND_SET[n].VALUE bit sets the status of the cor- responding peripheral interrupt to pending. Reading a bit identifies whether the inter- rupt is pending. |

## Shared Peripheral Interrupt Security Register

The GICDST\_SPI\_SECURITY[n] register provides a security status bit for each interrupt supported by the GIC.

<!-- image -->

VALUE[31:16] (R/W)

Interrupt Security Shared Peripheral Interrupt Security

Figure 7-46: GICDST\_SPI\_SECURITY[n] Register Diagram

Table 7-47: GICDST\_SPI\_SECURITY[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Security Interrupt Security. The GICDST_SPI_SECURITY[n].VALUE bits control the security status of the corresponding interrupt. |

## Shared Peripheral Interrupt Processor Targets Register

The GICDST\_SPI\_TRGT[n] register provides an 8-bit CPU targets field for each interrupt supported by the GIC.

Figure 7-47: GICDST\_SPI\_TRGT[n] Register Diagram

<!-- image -->

Table 7-48: GICDST\_SPI\_TRGT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Shared Peripheral Interrupt Processor Targets. The GICDST_SPI_TRGT[n].VALUE bit field stores the list of processors that the interrupt is sent to if it is asserted. |