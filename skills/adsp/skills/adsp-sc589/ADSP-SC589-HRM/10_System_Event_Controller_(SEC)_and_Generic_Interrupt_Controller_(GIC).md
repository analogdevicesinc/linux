## 7   System Event Controller (SEC) and Generic Interrupt Controller (GIC)

There are two interrupt controllers-a generic interrupt controller (GIC) for the ARM core and the system event controller (SEC) for the SHARC cores.

System event management is the responsibility of the system event controller (SEC). The SEC manages the configuration of all system event sources. The SEC also manages the propagation of system events to all connected cores and the system fault interface.

All of the peripheral interrupts are routed using a single SEC interrupt to the desired core. The SEC allows programmability of the peripheral interrupt's priority, supporting up to 256 priority levels that are arbitrated within the SEC itself. The SEC also allows these interrupts to be grouped and masked by priority level and provides the flexibility to choose which core(s) the interrupt is routed to.

The SEC also supports self-nesting of interrupts, which is required when sharing a single interrupt request to an individual core, as this allows for a higher-priority peripheral interrupt to be passed to the core while it is currently servicing a lower-priority peripheral interrupt. For more details please refer to 'Self-Nesting Mode for System Event Controller Interrupt (SECI)' in the SHARC+ Core Programming Reference .

For more information about the ARM GIC, visit the ARM Information Center.

## SEC Features

The following list describes the system event controller features.

- Comprehensive system event source management including interrupt enable, fault enable, priority, core mapping, and source grouping.
- Fault management including fault action configuration, timeout, external indication, and system reset.
- Determinism where all system events have the same propagation delay and provide unique identification of a specific system event source.
- Distributed programming model where each system event source control and all status fields are independent of all others.

- Slave control port which provides access to all SEC registers for configuration, status, and interrupt or fault service model.
- Global locking supports a register level protection model to prevent writes to 'locked' registers.

## SEC Functional Description

The following sections provide a functional description of the SEC.

The SEC/GIC Interrupt Signal Flow figure shows an overview of the interrupt systems.

Figure 7-1: SEC/GIC Interrupt Signal Flow

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000000_70e9a58a023aff8358d3ce4686475fb90059b1d82eda2ff908a712587759d454.png)

## ADSP-SC58x SEC Register List

The System Event Controller (SEC) manages the system fault sources, including control features such as enable/ disable, priority, and active/pending source status. For more information on SEC functionality, see the SEC register descriptions.

Table 7-1: ADSP-SC58x SEC Register List

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

Table 7-1: ADSP-SC58x SEC Register List (Continued)

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

## ADSP-SC58x SEC Interrupt List

Table 7-2: ADSP-SC58x SEC Interrupt List

|   Interrupt ID | Name     | Description   | Sensitivity   | DMA Channel   |
|----------------|----------|---------------|---------------|---------------|
|              0 | SEC0_ERR | SEC0 Error    | Level         |               |

## Combined SEC and GIC Interrupt List

The Combined SEC and GIC Interrupt List table provides a complete list of the processor and Cortex interrupts. Note that the DAI has its own system interrupt controllers. For more information see the DAI System Interrupt Controller (SIC). Note in the table below many interrupts are supported by all cores. In the cases were an interrupt is not supported its ID number (GIC\_ID for ARM Cortex-A5 or SEC\_ID for SHARC+) is reserved.

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List

| Module   | Event/Interrupt                                    | SEC ID   |   GIC ID | SEC/GIC Interrupt Name   |
|----------|----------------------------------------------------|----------|----------|--------------------------|
| GIC      | SW Interrupt 0, core reset, core ID = 0            | N/A      |        0 | GIC_SOFT00               |
| GIC      | SW Interrupt 1, undefined instruction, core ID = 0 | N/A      |        1 | GIC_SOFT01               |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt                               | SEC ID          | GIC ID           | SEC/GIC Interrupt Name   |
|----------|-----------------------------------------------|-----------------|------------------|--------------------------|
| GIC      | SW Interrupt 2, supervisor call, core ID = 0  | N/A             | 2                | GIC_SOFT02               |
| GIC      | SW Interrupt 3, prefetch call, core ID = 0    | N/A             | 3                | GIC_SOFT03               |
| GIC      | SW Interrupt 4, data abort, core ID = 0       | N/A             | 4                | GIC_SOFT04               |
| GIC      | SW Interrupt 5, reserved, core ID = 0         | N/A             | N/A              | GIC_SOFT05               |
| GIC      | SW Interrupt 6, IRQ interrupt, core ID = 0    | N/A             | 6                | GIC_SOFT06               |
| GIC      | SW Interrupt 7, FIQ interrupt, core ID = 0    | N/A             | 7                | GIC_SOFT07               |
|          |                                               | N/A             | 8 - 31 Reserved  |                          |
| SEC      | SEC0 Error                                    | 0               | 32               | SEC0_ERR                 |
| CGU      | CGU0 Event                                    | 1               | 33               | CGU0_EVT                 |
| CGU      | CGU1 Event                                    | 2               | 34               | CGU1_EVT                 |
| WDOG     | WDOG0 Expiration                              | 3               | 35               | WDOG0_EXP                |
| WDOG     | WDOG1 Expiration                              | 4               | 36               | WDOG1_EXP                |
| OTPC     | OTPC0 Dual bit Error                          | 5               | 37               | OTPC0_ERR                |
| TMU      | TMU0 Fault                                    | 6               | 38               | TMU0_FAULT               |
| TAPC     | Test/User Key Fail                            | 7               | 39               | TAPC0_KEYFAIL            |
| L2CTL    | L2CTL0 ECC Error                              | 8               | 40               | L2CTL0_ECC_ERR           |
|          |                                               | 9 - 13 Reserved | 41 - 45 Reserved |                          |
| SMPU     | Core Data Read interrupt, core ID = 1         | 14              | 46               | C1_IRQ0                  |
| SMPU     | Core Data Write interrupt, core ID = 1        | 15              | 47               | C1_IRQ1                  |
| SMPU     | Core Instruction Fetch interrupt, core ID = 1 | 16              | 48               | C1_IRQ2                  |
| SMPU     | Core Idle instruction interrupt, core ID = 1  | 17              | 49               | C1_IDLE                  |
| SMPU     | Core Data Read interrupt, core ID = 2         | 18              | 50               | C2_IRQ0                  |
| SMPU     | Core Data Write interrupt, core ID = 2        | 19              | 51               | C2_IRQ1                  |
| SMPU     | Core Instruction Fetch interrupt, core ID = 2 | 20              | 52               | C2_IRQ2                  |
| SMPU     | Core Idle instruction interrupt, core ID = 2  | 21              | 53               | C2_IDLE                  |
| CORE     | L2 Cache interrupt, core ID = 0               | 22              | 54               | C0_L2CC                  |
| CORE     | L1 Parity interrupt, core ID = 0              | 23              | 55               | C0_L1_PERR               |
| DAI      | DAI0 High Priority                            | 24              | 56               | DAI0_IRQH                |
| DAI      | DAI1 High Priority                            | 25              | 57               | DAI1_IRQH                |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt           |   SEC ID | GIC ID   | SEC/GIC Interrupt Name   |
|----------|---------------------------|----------|----------|--------------------------|
| GPTIMER  | GP TIMER0 Timer 0         |       26 | 58       | TIMER0_TMR0              |
| GPTIMER  | GP TIMER0 Timer 1         |       27 | 59       | TIMER0_TMR1              |
| GPTIMER  | GP TIMER0 Timer 2         |       28 | 60       | TIMER0_TMR2              |
| GPTIMER  | GP TIMER0 Timer 3         |       29 | 61       | TIMER0_TMR3              |
| EPWM     | EPWM0 PWMTMRGrouped       |       30 | 62       | PWM0_SYNC                |
| EPWM     | EPWM0 Trip                |       31 | 63       | PWM0_TRIP                |
| EPWM     | EPWM1 PWMTMRGrouped       |       32 | 64       | PWM1_SYNC                |
| EPWM     | EPWM1 Trip                |       33 | 65       | PWM1_TRIP                |
| EPWM     | EPWM2 PWMTMRGrouped       |       34 | 66       | PWM2_SYNC                |
| EPWM     | EPWM2 Trip                |       35 | 67       | PWM2_TRIP                |
| ACM      | ACM0 Event Miss           |       36 | 68       | ACM0_EVT_MISS            |
| ACM      | ACM0 Event Complete       |       37 | 69       | ACM0_EVT_COMPLETE        |
| PINT     | PINT0 Pin Interrupt Block |       38 | 70       | PINT0_BLOCK              |
| PINT     | PINT1 Pin Interrupt Block |       39 | 71       | PINT1_BLOCK              |
| PINT     | PINT2 Pin Interrupt Block |       40 | 72       | PINT2_BLOCK              |
| PINT     | PINT3 Pin Interrupt Block |       41 | 73       | PINT3_BLOCK              |
| PINT     | PINT4 Pin Interrupt Block |       42 | 74       | PINT4_BLOCK              |
| PINT     | PINT5 Pin Interrupt Block |       43 | 75       | PINT5_BLOCK              |
| SEC      | Software Interrupt 0      |       44 | Reserved | SOFT0                    |
| SEC      | Software Interrupt 1      |       45 | Reserved | SOFT1                    |
| SEC      | Software Interrupt 2      |       46 | Reserved | SOFT2                    |
| SEC      | Software Interrupt 3      |       47 | Reserved | SOFT3                    |
| SEC      | Software Interrupt 4      |       48 | Reserved | SOFT4                    |
| SEC      | Software Interrupt 5      |       49 | Reserved | SOFT5                    |
| SEC      | Software Interrupt 6      |       50 | Reserved | SOFT6                    |
| SEC      | Software Interrupt 7      |       51 | Reserved | SOFT7                    |
| SPORT    | SPORT0 ChannelADMA        |       52 | 84       | SPORT0_A_DMA             |
| SPORT    | SPORT0 Channel A Status   |       53 | 85       | SPORT0_A_STAT            |
| SPORT    | SPORT0 ChannelBDMA        |       54 | 86       | SPORT0_B_DMA             |
| SPORT    | SPORT0 Channel B Status   |       55 | 87       | SPORT0_B_STAT            |
| SPORT    | SPORT1 ChannelADMA        |       56 | 88       | SPORT1_A_DMA             |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt         |   SEC ID |   GIC ID | SEC/GIC Interrupt Name   |
|----------|-------------------------|----------|----------|--------------------------|
| SPORT    | SPORT1 Channel A Status |       57 |       89 | SPORT1_A_STAT            |
| SPORT    | SPORT1 ChannelBDMA      |       58 |       90 | SPORT1_B_DMA             |
| SPORT    | SPORT1 Channel B Status |       59 |       91 | SPORT1_B_STAT            |
| SPORT    | SPORT4 ChannelADMA      |       60 |       92 | SPORT4_A_DMA             |
| SPORT    | SPORT4 Channel A Status |       61 |       93 | SPORT4_A_STAT            |
| SPORT    | SPORT4 ChannelBDMA      |       62 |       94 | SPORT4_B_DMA             |
| SPORT    | SPORT4 Channel B Status |       63 |       95 | SPORT4_B_STAT            |
| SPORT    | SPORT5 ChannelADMA      |       64 |       96 | SPORT5_A_DMA             |
| SPORT    | SPORT5 Channel A Status |       65 |       97 | SPORT5_A_STAT            |
| SPORT    | SPORT5 ChannelBDMA      |       66 |       98 | SPORT5_B_DMA             |
| SPORT    | SPORT5 Channel B Status |       67 |       99 | SPORT5_B_STAT            |
| SPI      | SPI2 TX DMAChannel      |       68 |      100 | SPI2_TXDMA               |
| SPI      | SPI2 RX DMAChannel      |       69 |      101 | SPI2_RXDMA               |
| SPI      | SPI2 Status             |       70 |      102 | SPI2_STAT                |
| SPI      | SPI2 Error              |       71 |      103 | SPI2_ERR                 |
| GPTIMER  | GP TIMER0 Timer 4       |       72 |      104 | GPTIMER0_TMR4            |
| GPTIMER  | GP TIMER0 Timer 5       |       73 |      105 | GPTIMER0_TMR5            |
| GPTIMER  | GP TIMER0 Timer 6       |       74 |      106 | GPTIMER0_TMR6            |
| GPTIMER  | GP TIMER0 Timer 7       |       75 |      107 | GPTIMER0_TMR7            |
| GPTIMER  | GP TIMER0 Status        |       76 |      108 | GPTIMER0_STAT            |
| LP       | LP0 DMAData             |       77 |      109 | LP0_DMA                  |
| LP       | LP0 Status              |       78 |      110 | LP0_STAT                 |
| LP       | LP1 DMAData             |       79 |      111 | LP1_DMA                  |
| LP       | LP1 Status              |       80 |      112 | LP1_STAT                 |
| EPPI     | EPPI0 DMAChannel 0      |       81 |      113 | EPPI0_CH0_DMA            |
| EPPI     | EPPI0 DMAChannel 1      |       82 |      114 | EPPI0_CH1_DMA            |
| EPPI     | EPPI0 Status            |       83 |      115 | EPPI0_STAT               |
| CAN      | CAN0 Receive            |       84 |      116 | CAN0_RX                  |
| CAN      | CAN0 Transmit           |       85 |      117 | CAN0_TX                  |
| CAN      | CAN0 Status             |       86 |      118 | CAN0_STAT                |
| CAN      | CAN1 Receive            |       87 |      119 | CAN1_RX                  |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt         |   SEC ID |   GIC ID | SEC/GIC Interrupt Name   |
|----------|-------------------------|----------|----------|--------------------------|
| CAN      | CAN1 Transmit           |       88 |      120 | CAN1_TX                  |
| CAN      | CAN1 Status             |       89 |      121 | CAN1_STAT                |
| SPORT    | SPORT2 ChannelADMA      |       90 |      122 | SPORT2_A_DMA             |
| SPORT    | SPORT2 Channel A Status |       91 |      123 | SPORT2_A_STAT            |
| SPORT    | SPORT2 ChannelBDMA      |       92 |      124 | SPORT2_B_DMA             |
| SPORT    | SPORT2 Channel B Status |       93 |      125 | SPORT2_B_STAT            |
| SPORT    | SPORT3 ChannelADMA      |       94 |      126 | SPORT3_A_DMA             |
| SPORT    | SPORT3 Channel A Status |       95 |      127 | SPORT3_A_STAT            |
| SPORT    | SPORT3 ChannelBDMA      |       96 |      128 | SPORT3_B_DMA             |
| SPORT    | SPORT3 Channel B Status |       97 |      129 | SPORT3_B_STAT            |
| SPORT    | SPORT6 ChannelADMA      |       98 |      130 | SPORT6_A_DMA             |
| SPORT    | SPORT6 Channel A Status |       99 |      131 | SPORT6_A_STAT            |
| SPORT    | SPORT6 ChannelBDMA      |      100 |      132 | SPORT6_B_DMA             |
| SPORT    | SPORT6 Channel B Status |      101 |      133 | SPORT6_B_STAT            |
| SPORT    | SPORT7 ChannelADMA      |      102 |      134 | SPORT7_A_DMA             |
| SPORT    | SPORT7 Channel A Status |      103 |      135 | SPORT7_A_STAT            |
| SPORT    | SPORT7 ChannelBDMA      |      104 |      136 | SPORT7_B_DMA             |
| SPORT    | SPORT7 Channel B Status |      105 |      137 | SPORT7_B_STAT            |
| SPI      | SPI0 TX DMAChannel      |      106 |      138 | SPI0_TXDMA               |
| SPI      | SPI0 RX DMAChannel      |      107 |      139 | SPI0_RXDMA               |
| SPI      | SPI0 Status             |      108 |      140 | SPI0_STAT                |
| SPI      | SPI0 Error              |      109 |      141 | SPI0_ERR                 |
| SPI      | SPI1 TX DMAChannel      |      110 |      142 | SPI1_TXDMA               |
| SPI      | SPI1 RX DMAChannel      |      111 |      143 | SPI1_RXDMA               |
| SPI      | SPI1 Status             |      112 |      144 | SPI1_STAT                |
| SPI      | SPI1 Error              |      113 |      145 | SPI1_ERR                 |
| UART     | UART0 TransmitDMA       |      114 |      146 | UART0_TXDMA              |
| UART     | UART0 ReceiveDMA        |      115 |      147 | UART0_RXDMA              |
| UART     | UART0 Status            |      116 |      148 | UART0_STAT               |
| UART     | UART1 TransmitDMA       |      117 |      149 | UART1_TXDMA              |
| UART     | UART1 ReceiveDMA        |      118 |      150 | UART1_RXDMA              |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt                      |   SEC ID | GIC ID   | SEC/GIC Interrupt Name   |
|----------|--------------------------------------|----------|----------|--------------------------|
| UART     | UART1 Status                         |      119 | 151      | UART1_STAT               |
| UART     | UART2 TransmitDMA                    |      120 | 152      | UART2_TXDMA              |
| UART     | UART2 ReceiveDMA                     |      121 | 153      | UART2_RXDMA              |
| UART     | UART2 Status                         |      122 | 154      | UART2_STAT               |
| TWI      | TWI0 Data                            |      123 | 155      | TWI0_DATA                |
| TWI      | TWI1 Data                            |      124 | 156      | TWI1_DATA                |
| TWI      | TWI2 Data                            |      125 | 157      | TWI2_DATA                |
| CNT      | CNT0 Status                          |      126 | 158      | CNT0_STAT                |
| CTI      | CTI Event1, Core ID = 1              |      127 | Reserved | ECT_C1_EVT               |
| CTI      | CTI Event2, Core ID = 2              |      128 | Reserved | ECT_C2_EVT               |
| PKIC     | Public Key Interrupt (PKA, TRNG, SL) |      129 | 161      | PKIC0_IRQ                |
| PKTE     | Security Packet Engine               |      130 | 162      | PKTE0_IRQ                |
| MSI      | MSI0 Status                          |      131 | 163      | MSI0_STAT                |
| USB      | USB0 Status/FIFO Data Ready          |      132 | 164      | USB0_STAT                |
| USB      | USB0 DMAStatus/Transfer Complete     |      133 | 165      | USB0_DATA                |
| USB      | USB1 Status/FIFO Data Ready          |      134 | 166      | USB1_STAT                |
| USB      | USB1 DMAStatus/Transfer Complete     |      135 | 167      | USB1_DATA                |
| TRU      | TRU0 Interrupt 4, core ID = 1        |      136 | Reserved | TRU0_INT4                |
| TRU      | TRU0 Interrupt 5, core ID = 1        |      137 | Reserved | TRU0_INT5                |
| TRU      | TRU0 Interrupt 6, core ID = 1        |      138 | Reserved | TRU0_INT6                |
| TRU      | TRU0 Interrupt 7, core ID = 1        |      139 | Reserved | TRU0_INT7                |
| TRU      | TRU0 Interrupt 8, core ID = 2        |      140 | Reserved | TRU0_INT8                |
| TRU      | TRU0 Interrupt 9, core ID = 2        |      141 | Reserved | TRU0_INT9                |
| TRU      | TRU0 Interrupt 10, core ID = 2       |      142 | Reserved | TRU0_INT10               |
| TRU      | TRU0 Interrupt 11, core ID = 2       |      143 | Reserved | TRU0_INT11               |
| SINC     | SINC0 Status                         |      144 | 176      | SINC0_STAT               |
| DAI      | DAI0 Low Priority                    |      145 | 177      | DAI0_IRQL                |
| DAI      | DAI1 Low Priority                    |      146 | 178      | DAI1_IRQL                |
| EMAC     | EMAC0 Status                         |      148 | 180      | EMAC0_STAT               |
| EMAC     | EMAC1 Status                         |      149 | 181      | EMAC1_STAT               |
| FFTA     | FFTA0 TransmitDMA                    |      150 | 182      | FFTA0_TXDMA              |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt                      |   SEC ID |   GIC ID | SEC/GIC Interrupt Name   |
|----------|--------------------------------------|----------|----------|--------------------------|
| FFTA     | FFTA0 ReceiveDMA                     |      151 |      183 | FFTA0_RXDMA              |
| FFTA     | FFTA0 Status                         |      152 |      184 | FFTA0_STAT               |
| FIR      | FIR0DMA                              |      153 |      185 | FIR0_DMA                 |
| FIR      | FIR0 Status                          |      154 |      186 | FIR0_STAT                |
| IIR      | IIR0DMA                              |      155 |      187 | IIR0_DMA                 |
| IIR      | IIR0 Status                          |      156 |      188 | IIR0_STAT                |
| HADC     | HADC0                                |      157 |      189 | HADC0_EVT                |
| HAE      | HAE0 RX DMAChannel 0                 |      158 |      190 | HAE0_RXDMA_CH0           |
| HAE      | HAE0 RX DMAChannel 1                 |      159 |      191 | HAE0_RXDMA_CH1           |
| HAE      | HAE0 TX DMAChannel 0                 |      160 |      192 | HAE0_TXDMA               |
| HAE      | HAE0 Status                          |      161 |      193 | HAE0_STAT                |
| MLB      | MLB0 Interrupt Channel 0 through 31  |      162 |      194 | MLB0_INT0                |
| MLB      | MLB0 Interrupt Channel 32 thorugh 63 |      163 |      195 | MLB0_INT1                |
| MLB      | MLB0 Status                          |      164 |      196 | MLB0_STAT                |
| RTC      | RTC Event                            |      165 |      197 | RTC0_EVT                 |
| MDMA     | Maximum BWDMAChannel 0               |      166 |      198 | MDMA3_SRC                |
| MDMA     | Maximum BWDMAChannel 1               |      167 |      199 | MDMA3_DST                |
| MDMA     | Enhanced BWDMAChannel 0              |      168 |      200 | MDMA2_SRC                |
| MDMA     | Enhanced BWDMAChannel 1              |      169 |      201 | MDMA2_DST                |
| EMDMA    | EMDMA0 DMADone                       |      170 |      202 | EMDMA0_DONE              |
| EMDMA    | EMDMA1 DMADone                       |      171 |      203 | EMDMA1_DONE              |
| CRC      | Standard BWDMA/CRC0 Channel 0        |      172 |      204 | MDMA0_SRC                |
| CRC      | Standard BWDMA/CRC0 Channel 1        |      173 |      205 | MDMA0_DST                |
| CRC      | Standard BWDMA/CRC1 Channel 0        |      174 |      206 | MDMA1_SRC                |
| CRC      | Standard BWDMA/CRC1 Channel 1        |      175 |      207 | MDMA1_DST                |
| CRC      | CRC0 Datacount expiration            |      176 |      208 | CRC0_DCNTEXP             |
| CRC      | CRC1 Datacount expiration            |      177 |      209 | CRC1_DCNTEXP             |
| CRC      | CRC0 Error                           |      178 |      210 | CRC0_ERR                 |
| CRC      | CRC1 Error                           |      179 |      211 | CRC1_ERR                 |
| SPORT    | SPORT0 Channel A DMAError            |      180 |      212 | SPORT0_A_DMA_ERR         |
| SPORT    | SPORT0 Channel B DMAError            |      181 |      213 | SPORT0_B_DMA_ERR         |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt            |   SEC ID |   GIC ID | SEC/GIC Interrupt Name   |
|----------|----------------------------|----------|----------|--------------------------|
| SPORT    | SPORT1 Channel A DMAError  |      182 |      214 | SPORT1_A_DMA_ERR         |
| SPORT    | SPORT1 Channel B DMAError  |      183 |      215 | SPORT1_B_DMA_ERR         |
| SPORT    | SPORT4 Channel A DMAError  |      184 |      216 | SPORT4_A_DMA_ERR         |
| SPORT    | SPORT4 Channel B DMAError  |      185 |      217 | SPORT4_B_DMA_ERR         |
| SPORT    | SPORT5 Channel A DMAError  |      186 |      218 | SPORT5_A_DMA_ERR         |
| SPORT    | SPORT5 Channel B DMAError  |      187 |      219 | SPORT5_B_DMA_ERR         |
| SPI      | SPI2 TX DMAChannel Error   |      188 |      220 | SPI2_TXDMA_ERR           |
| SPI      | SPI2 RX DMAChannel Error   |      189 |      221 | SPI2_RXDMA_ERR           |
| SPORT    | SPORT2 Channel A DMAError  |      190 |      222 | SPORT2_A_DMA_ERR         |
| SPORT    | SPORT2 Channel B DMAError  |      191 |      223 | SPORT2_B_DMA_ERR         |
| SPORT    | SPORT3 Channel A DMAError  |      192 |      224 | SPORT3_A_DMA_ERR         |
| SPORT    | SPORT3 Channel B DMAError  |      193 |      225 | SPORT3_B_DMA_ERR         |
| SPORT    | SPORT6 Channel A DMAError  |      194 |      226 | SPORT6_A_DMA_ERR         |
| SPORT    | SPORT6 Channel B DMAError  |      195 |      227 | SPORT6_B_DMA_ERR         |
| SPORT    | SPORT7 Channel A DMAError  |      196 |      228 | SPORT7_A_DMA_ERR         |
| SPORT    | SPORT7 Channel B DMAError  |      197 |      229 | SPORT7_B_DMA_ERR         |
| SPI      | SPI0 TX DMAChannel Error   |      198 |      230 | SPI0_TXDMA_ERR           |
| SPI      | SPI0 RX DMAChannel Error   |      199 |      231 | SPI0_RXDMA_ERR           |
| SPI      | SPI1 TX DMAChannel Error   |      200 |      232 | SPI1_TXDMA_ERR           |
| SPI      | SPI1 RX DMAChannel Error   |      201 |      233 | SPI1_RXDMA_ERR           |
| UART     | UART0 Transmit DMAError    |      202 |      234 | UART0_TXDMA_ERR          |
| UART     | UART0 Receive DMAError     |      203 |      235 | UART0_RXDMA_ERR          |
| UART     | UART1 Transmit DMAError    |      204 |      236 | UART1_TXDMA_ERR          |
| UART     | UART1 Receive DMAError     |      205 |      237 | UART1_RXDMA_ERR          |
| UART     | UART2 Transmit DMAError    |      206 |      238 | UART2_TXDMA_ERR          |
| UART     | UART2 Receive DMAError     |      207 |      239 | UART2_RXDMA_ERR          |
| LP       | LP0 DMAData Error          |      208 |      240 | LP0_DMA_ERR              |
| LP       | LP1 DMAData Error          |      209 |      241 | LP1_DMA_ERR              |
| FFTA     | FFTA0 Transmit DMAError    |      210 |      242 | FFT0_TXDMA_ERR           |
| FFTA     | FFTA0 Receive DMAError     |      211 |      243 | FFT0_RXDMA_ERR           |
| HAE      | HAE0 RX DMAChannel 0 Error |      212 |      244 | HAE0_RXDMA_CH0_ERR       |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt                       |   SEC ID |   GIC ID | SEC/GIC Interrupt Name   |
|----------|---------------------------------------|----------|----------|--------------------------|
| HAE      | HAE0 RX DMAChannel 1 Error            |      213 |      245 | HAE0_RXDMA_CH1_ERR       |
| HAE      | HAE0 TX DMAChannel 0 Error            |      214 |      246 | HAE0_TXDMA_ERR           |
| EPPI     | EPPI0 DMAChannel 0 Error              |      215 |      247 | EPPI0_CH0_DMA_ERR        |
| EPPI     | EPPI0 DMAChannel 1 Error              |      216 |      248 | EPPI0_CH1_DMA_ERR        |
| MDMA     | Standard BWSource 0 DMAChannel Error  |      217 |      249 | MDMA0_SRC_ERR            |
| MDMA     | Standard BWDest 0 DMAChannel Error    |      218 |      250 | MDMA0_DST_ERR            |
| MDMA     | Standard BWSource 1 DMAChannel Error  |      219 |      251 | MDMA1_SRC_ERR            |
| MDMA     | Standard BWDest 1 DMAChannel Error    |      220 |      252 | MDMA1_DST_ERR            |
| MDMA     | Enhanced BWDMAChannel 0 Error         |      221 |      253 | MDMA2_SRC_ERR            |
| MDMA     | Enhanced BWDMAChannel 1 Error         |      222 |      254 | MDMA2_DST_ERR            |
| MDMA     | Maximum BWDMAChannel 0 Error          |      223 |      255 | MDMA3_SRC_ERR            |
| MDMA     | Maximum BWDMAChannel 1 Error          |      224 |      256 | MDMA3_DST_ERR            |
| SWU      | SWU0 Event SMC                        |      225 |      257 | SWU0_EVT                 |
| SWU      | SWU1 Event, L2 Memory DMAPort 0       |      226 |      258 | SWU1_EVT                 |
| SWU      | SWU2 Event, L2 Memory Core Port 0     |      227 |      259 | SWU2_EVT                 |
| SWU      | SWU3 Event, L2 Memory DMAPort 1       |      228 |      260 | SWU3_EVT                 |
| SWU      | SWU4 Event, L2 Memory Core Port 1     |      229 |      261 | SWU4_EVT                 |
| SWU      | SWU3 Event, L2 Memory DMAPort 2       |      230 |      262 | SWU5_EVT                 |
| SWU      | SWU4 Event, L2 Memory Core Port 2     |      231 |      263 | SWU6_EVT                 |
| SWU      | SWU7 Event, Core ID = 1 Slave Port 1  |      232 |      264 | SWU7_EVT                 |
| SWU      | SWU8 Event, Core ID = 1 Slave Port 2  |      233 |      265 | SWU8_EVT                 |
| SWU      | SWU9 Event, Core ID = 2 Slave Port 1  |      234 |      266 | SWU9_EVT                 |
| SWU      | SWU10 Event, Core ID = 2 Slave Port 2 |      235 |      267 | SWU10_EVT                |
| SWU      | SWU11 Event SMMR                      |      236 |      268 | SWU11_EVT                |
| SWU      | SWU12 Event SPI L3 Memory             |      237 |      269 | SWU12_EVT                |
| SWU      | SWU13 Event DMC0_A L3 Memory          |      238 |      270 | SWU13_EVT                |
| SWU      | SWU14 Event DMC0_B L3 Memory          |      239 |      271 | SWU14_EVT                |
| SWU      | SWU15 Event PCIe L3 Memory            |      240 |      272 | SWU15_EVT                |
| SPU      | SPU0 Event                            |      241 |      273 | SPU0_INT                 |
| SMPU     | SMPU Aggregated Event                 |      242 |      274 | SMPU0_AGGR_INT           |
| PCIE     | PCIe Reset Request                    |      243 |      275 | PCIE0_RESET              |

Table 7-3: ADSP-SC58x Combined SEC and GIC Interrupt List (Continued)

| Module   | Event/Interrupt                               | SEC ID             | GIC ID             | SEC/GIC Interrupt Name   |
|----------|-----------------------------------------------|--------------------|--------------------|--------------------------|
| PCIE     | PCIe Status                                   | 244                | 276                | PCIE0_STAT               |
| PCIE     | PCIe DMACompletion                            | 245                | 277                | PCIE0_DMA                |
|          |                                               | 246 - 247 Reserved | 278 - 279 Reserved |                          |
| TRU      | TRU0 Interrupt 0, core ID = 0                 | Reserved           | 280                | TRU_INT0                 |
| TRU      | TRU0 Interrupt 1, core ID = 0                 | Reserved           | 281                | TRU_INT1                 |
| TRU      | TRU0 Interrupt 2, core ID = 0                 | Reserved           | 282                | TRU_INT2                 |
| TRU      | TRU0 Interrupt 3, core ID = 0                 | Reserved           | 283                | TRU_INT3                 |
| CTI      | CTI Event0, core ID = 0                       | Reserved           | 284                | ECT_C0_EVT               |
| PMU      | Performance Monitoring Interrupt, core ID = 0 | Reserved           | 285                | C0_PMUIRQ                |

## SEC Definitions

The event controller uses the following definitions.

## System Events

System source indications including interrupts and faults.

## System Source

Point of origin of system event.

## SID (Identification, unique)

Source numeric identifier for each system source connected to the SEC.

## SSI

SEC source interface, system event source control, and status subblock of the SEC.

## SCI

SEC core interface, core interface subblock of the SEC

## SPR

SEC prioritizer determines the highest priority pending interrupt and the highest priority active interrupt. The SPR provides these interrupts in the appropriate registers of the SCI for the priority and nesting model of the SCI.

## SFI

SEC Fault Interface, fault management subblock of the SEC.

## SEC Block Diagram

The SEC Block Diagram shows the event management architecture.

System sources connect to the SEC through the SSI. Each core has a dedicated SCI. The SFI provides fault action connections to the rest of the system.

Figure 7-2: SEC Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000001_3f5500a5891a65de004d9a33a81a7dc0f5abc86f1f552fb8b205e3f2d77f1379.png)

## SEC Fault Interface (SFI)

The SFI manages fault events and associated actions. The fault management support provided in the SEC helps satisfy the safety requirements of various applications. The SSI provides the highest priority pending source that is enabled as a fault. The SFI captures this value and enables a countdown, and once the countdown expires, takes the prescribed action.

Fault actions which can be configured, as shown in SFI Block Diagram , include

- Trigger Output
- System Reset
- Fault Output
- Computer Operating Properly (COP) mode
- Fault Mode

Figure 7-3: SFI Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000002_92b50b47075e1502989c0f7788067c7af21d589c4576264b77a693630ac53b8a.png)

## Fault Management

System sources can be enabled as fault sources in the SEC\_SCTL[n] register. When a source enabled as a fault moves to pending, it is forwarded to the SFI as a fault indication. The pending bit ( SEC\_FSTAT.PND ) indicates a source has signaled a fault assertion but it has not yet triggered the event actions (if delay is enabled). The SEC fault interface sets the SEC\_FSTAT.PND bit when the fault source ID register ( SEC\_FSID ) is updated on assertion of a fault source input. The system source pending triggers a fault pending and after a programmable delay the fault moves to active. Event actions then execute if appropriate action is not taken by the core. The SEC\_FSTAT.ACT bit indicates that the SEC has received a fault source input, the delay has expired, and the fault actions are enabled.

The SEC\_FSTAT.NPND bit indicates if one or more sources have signaled a fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC\_FSTAT.NPND bit when the fault interface detects assertion of any enabled fault source input, while either the SEC\_FSTAT.PND or SEC\_FSTAT.ACT bits are set. The SEC clears the SEC\_FSTAT.NPND bit when there are no fault sources waiting.

A fault indication from an external device can also be detected on sampling the fault signals. When a fault is detected the SEC\_FSTAT.ACT and SEC\_FSID.FEXT bits are set. The assertion of either signal results in a fault input detection.

The SEC\_FEND register receives a fault end indication from the core. The core writes the SID of the fault to the SEC\_FEND register. If the SID matches the value in the SEC\_FSID register, the SEC\_FSTAT.PND and SEC\_FSTAT.ACT bits are cleared.

More information can be found in the Fault Management Interface Programming Model section.

## SEC Core Interface (SCI)

The SCI manages communication between the corresponding core and the SEC. The SEC prioritizer (SPR) of the SCI receives pending, active, and priority information from the SSI for each system event source assigned to this SCI. The SPR determines the highest-priority pending system event and the SCI determines whether it propagates

to the core. The SCI maintains the coherency for the system event service model implemented on the connected core.

Figure 7-4: SCI Overview Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000003_66768fb4d68dc1dfffc34cf1ab4590623c9af60a6b3c80d87ec36b078b57eb2e.png)

## SEC Source Interface (SSI)

The SSI manages all of the system event sources. It maintains the status of each source in the corresponding SEC\_SSTAT[n] register. The corresponding SEC\_SCTL[n] register manages the control of each source. A pending and enabled event passes its indication and priority to the SCI to which it is assigned for further processing.

Figure 7-5: SSI Overview Block Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000004_eabdaf20a25b130704ab76d0073eb9c576e5468ee8cf2d9428e9451566359b84.png)

## SEC Architectural Concepts

The following sections describe SEC architectural features.

## System Interrupt Acknowledge

A system interrupt acknowledge occurs when the core provides an indication that it has acquired the SID of the interrupt last issued by the SEC. The SEC core interface option allows generation by:

- A slave port write to the SEC\_CSID[n] register.
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
6. The SEC compares the SEC\_CPND[n] (B) register value to the SEC\_CACT[n] (A) register value. If the SEC\_CACT[n] (A) register value is a higher priority, continue.
7. The SEC copies the SEC\_CPND[n] (B) register value to SEC\_CSID[n] register and asserts the interrupt signal.
8. The core reads the SEC\_CSID[n] (B) register (or core version).
9. The core writes to the SEC\_CSID[n] register (or core version, asserts the acknowledge signal).
10. The SEC deasserts the INT signal and clears the SEC\_SSTAT[n].PND bit and sets the SEC\_SSTAT[n].ACT bit of the source (B) going active.
11. The core writes the SEC\_CSID[n] of the active interrupt (B) to the SEC\_END register.
12. The SEC clears the SEC\_SSTAT[n].ACT bit of the source (B) being ended.
13. The core writes the SEC\_CSID[n] of the active interrupt (A) to the SEC\_END register.
14. The SEC clears the SEC\_SSTAT[n].ACT bit of the source (A) being ended.

## System Interrupt Priorities

Each system interrupt source has its own programmable priority level which is configured using the SEC\_SCTL[n].PRIO bit field. The SCI evaluates the priority of all pending sources to determine the source of the highest-priority pending system interrupt for forwarding to the attached core. If more than one source of the pending system interrupt has the same priority setting, the SCI chooses the one with the lowest SID. For example, if SID 0, SID 1, and SID 2 are all pending and have the same priority setting, the SCI chooses SID 0 as the highestpriority source.

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
- Configuring an SCI to provide system interrupts to the connected core. (See Configuring a System Source to Interrupt a Core.)
- Configuring an SSI as a system fault. (See Configuring a System Source as a Fault.)
- Configuring the SFI to manage system faults.

## Programming Examples

This section provides example programming tasks that are typical for SEC usage.

## Fault Management Interface Programming Model

The SFI interface can be programmed to manage fault events from system sources and associated actions such as issuing a system reset when watchdog expiration event occurs.

1. Set the SEC\_GCTL.EN bit to enable the SEC.
2. Write to the SEC\_FCTL register to configure specific fault actions.
- Trigger Output. Set the SEC\_FCTL.TOEN bit for the SEC to produce trigger outputs when a fault becomes active. The SEC\_FCTL.TES bit can be programmed to select the event that directs the SEC to assert trigger output when a fault is pending or active. Configure slaves for SEC fault trigger master output.
4. NOTE: If the SEC\_FCTL.TOEN and or the SEC\_FCTL.TES bits =1 (T rigger Output Enabled and Trigger on Fault Pending), an external fault (if enabled by the SEC\_FCTL.FIEN bit) will not issue a trigger since Fault Pending is bypassed for external faults.
- System Reset. The Reset Control Unit (RCU) controls how the functional units enter and exit reset. Configure the RCU\_CTL.SRSTREQEN bit. This bit controls whether the sources of reset are enabled to perform a system reset. To issue a system reset request when a fault becomes active, set the SEC\_FCTL.SREN bit. The SEC fault system reset delay register ( SEC\_FSRDLY ) can be programmed for the delay, if required, from a fault becoming active to system reset request assertion.

- Fault Output. This configuration alows the SEC to indicate the fault status based on the SEC\_FCTL.CMS bit configuration.
- Computer Operating Normally (COP) mode. To configure fault output for COP mode, set the SEC\_FCTL.FOEN bit to enable fault output. Set the SEC\_FCTL.CMS bit to select COP mode to toggle the fault pin when no fault is active. Program the SEC\_FCOPP period register with a desired width value for the COP toggled output pin.
- Fault mode. Set the SEC\_FCTL.FOEN bit to enable fault output. The SEC\_FCTL.CMS bit should be set to Fault mode to toggle the fault pin when a fault is active.
3. If required, program the Fault Input to sample fault inputs from external devices on fault pins. Configure the SEC\_FCTL.FIEN bit to enable the SEC to sample a fault input from an external device.
- ADDITIONAL INFORMATION: The SEC\_FCTL.FIEN bit should be set only while the SEC\_FCTL.EN bit is low. If the SEC\_FCTL.EN bit is already high and the SEC\_FCTL.FIEN bit needs to be set, the SEC\_FCTL.EN bit should be cleared first. Fault input can only be enabled when Fault mode is selected by the SEC\_FCTL.CMS bit.
4. Program the required fault delay to the SEC\_FDLY.COUNT bit field if a delay between fault source assertion and the fault response is required.
5. Configure the SEC\_FCTL register to enable the SEC. ADDITIONAL INFORMATION: The SEC\_FCTL.EN bit should be set only while the SEC\_FSTAT.ACT bit is low.
6. Write to the control register of a specific source register using the SEC\_SCTL[n] register to enable the source as a fault.

## Configuring a System Source to Interrupt a Core

To configure a system source to interrupt a core, the SEC itself must be enabled with the source interface (SSI) and core interface (SCI) properly initialized. Specifically, the SCI must be set up to accept interrupt signaling from the SEC and pass them to the specified core, and the SSI must properly enable each of the peripheral interrupt sources to generate interrupt signals and optionally define a priority scheme that overrides the default priority settings. In summary:

1. Write to the SEC\_GCTL register to enable the SEC.
2. Write to the appropriate SCI SEC\_CCTL[n] register to enable SEC interrupts to be sent to that core.
3. Write to the appropriate SSI SEC\_SCTL[n] register to enable that peripheral as an interrupt source and to set the core target field to map the source to the desired SCI.
4. (Optional) By default, all the SEC interrupts are grouped as a single priority level, so passing of peripheral interrupt requests from the SEC is based solely on the default enumerated source ID. By programming the SEC\_CPLVL[n].PLVL register, interrupt sources can be grouped into priority levels within the SEC such that arbitration is first performed by source ID within a grouped priority level before proceeding to the next

priority level, thus providing the flexibility to have lower-priority interrupt sources considered before higherpriority sources.

ADDITIONAL INFORMATION: The SEC\_CPMSK[n] and SEC\_CGMSK[n] registers must also can be programmed to mask the interrupts based on the customized levels and grouping.

## Core/SEC Handshaking Requirements to Ensure Proper Interrupt Handling

Interrupt handling within an individual core requires specific handshaking with the SEC to ensure that nested interrupts are properly tracked and that new peripheral interrupts being raised within the SEC are either passed immediately to the core or held off and queued within the SEC for later servicing. Inside the SEC ISR, the following steps are required:

Use this procedure to write a custom dispatcher inside the Interrupt Service Routine. Note that the core needs to read the SEC\_CSID[n] register and acknowledge it by writing the same value. It should also write to the SEC\_END register after the ISR execution completes.

1. Read the SEC\_CSID[n] register to obtain the source ID of the peripheral interrupt request.
2. Write the read value back to the SEC\_CSID[n] register to send the acknowledge signal to the SEC that the core has accepted and begun processing for the interrupt request.
3. Execute the actual ISR (typically a call to a specific handler function from a look-up table based on the peripheral source ID). Write to the SEC\_GCTL register to enable the SEC.
4. Write the SEC\_CSID[n] of the active interrupt (read in step 1 above) to the SEC\_END register to signal to the SEC that the interrupt has now been serviced.
5. Return from interrupt.

With this implementation in place, a higher-priority interrupt being raised by the SEC can be serviced by the core after step 2. The SEC knows what it passed to the core by virtue of its write to the SEC\_CSID[n] register. After the core acknowledges that write, the SEC knows whether or not newly raised peripheral interrupts are higher priority than the highest-priority interrupt being processed by the core. If the new interrupt is higher priority, it pushes the current SEC\_CSID[n] to an internal stack, writes the new SEC\_CSID[n] value, and asserts a new SEC interrupt request. If it is lower priority, the SEC queues the interrupt until the core writes to the SEC\_END register with the source ID of the higher-priority interrupt, thus confirming that it was fully processed, at which point that SEC\_CSID[n] value is popped from the internal stack and any pending peripheral interrupt requests are arbitrated among before the SEC writes the new SEC\_CSID[n] value and asserts a new interrupt request. Meanwhile, the core self-nests the latched SEC interrupt requests, as needed, when a higher-priority interrupt is presented to it, and the write to the SEC\_END register in the SEC handler epilog code guarantees that each nested level has the required handshaking to signal to the SEC block that each individual source ID interrupt request is fully serviced. Please refer to the SHARC+ Core Programming Reference for more details regarding SEC handler code.

## Configuring a System Source as a Fault

1. Write to the SEC\_GCTL register to enable the SEC.

2. Write to the SEC\_FCTL register to configure specific fault actions.
3. Write to the SEC\_FDLY bit field to specify fault delay.
4. Write to the control register of a specific source to enable the source as a fault.

## Configuring the WDOG Expiry Event to Issue a System Reset

Use the following procedure to configure the WDOG timer to issue a system reset.

1. Configure the SEC\_GCTL register to enable the SEC.
2. Configure the SEC\_FCTL register to choose the Fault response mode. In the following code example, the system reset is issued.

```
ADDITIONAL INFORMATION: *pREG_SEC0_GCTL = BITM_SEC_GCTL_EN;
```

ADDITIONAL INFORMATION:

```
*pREG_RCU0_CTL |= BITM_RCU_CTL_SRSTREQEN; *pREG_SEC0_FCTL |= BITM_SEC_FCTL_SREN;
```

ADDITIONAL INFORMATION: Similarly, the program can also choose to signal the fault pin or choose to issue a trigger via the TRU as a response to the fault source (WDOG expiry is the fault source in this example) by programming the relevant bit fields in the SEC\_FCTL register.

3. Configure the SEC\_SCTL[n].SEN and SEC\_SCTL[n].FEN bits in the Source Control 3 (n=3) register registers to determine how the fault source is handled. To configure the WDOG as the fault source, program the register. The program can configure any interrupt as the fault source by programming the corresponding register.

ADDITIONAL INFORMATION:

```
*pREG_SEC0_SCTL3 = BITM_SEC_SCTL_FEN|BITM_SEC_SCTL_SEN;
```

ADDITIONAL INFORMATION: The SEC ID corresponding to WDOG0 is 3, as indicated in Table 7-3 ADSP-SC58x Combined SEC and GIC Interrupt List .

4. Write to the enable bit.

```
ADDITIONAL INFORMATION: *pREG_SEC0_FCTL |= BITM_SEC_FCTL_EN;
```

## SEC Programming Restrictions

Setting the SEC\_FCTL.EN bit while the SEC\_FSTAT.ACT bit is high can result in unpredictable behavior. To avoid this issue, set the SEC\_FCTL.EN bit while the SEC\_FSTAT.ACT bit is low. The SEC\_FSTAT.ACT bit is only set when the SEC\_FCTL.EN bit is high. Therefore, the problem can only occur if the SEC\_FCTL.EN bit transitions from 1 to 0 and then to 1 again.

Writing to SEC\_FEND to end a fault with both the SEC\_FCTL.FOEN bit and the SEC\_FCTL.FIEN bit set can result in erroneous external fault detection. If this operation (ending a fault) and configuration (fault input and fault output enabled) are required by the application, clear the SEC\_FCTL.FOEN bit prior to writing to SEC\_FEND . The recommended sequence for ending a fault with the SEC\_FCTL.FIEN or SEC\_FCTL.FOEN ==1 is as follows:

1. Clear the SEC\_FCTL.FOEN bit.
2. Write to the SEC\_FEND register.
3. Set the SEC\_FCTL.FOEN bit.

## ADSP-SC58x SEC Register Descriptions

System Event Controller (SEC) contains the following registers.

Table 7-4: ADSP-SC58x SEC Register List

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

Table 7-4: ADSP-SC58x SEC Register List (Continued)

| Name         | Description               |
|--------------|---------------------------|
| SEC_GSTAT    | Global Status Register    |
| SEC_RAISE    | Global Raise Register     |
| SEC_SCTL[n]  | Source Control Register n |
| SEC_SSTAT[n] | Source Status Register n  |

## SCI Active Register n

The SEC SCI active interrupt register ( SEC\_CACT[n] ) contains the source ID and priority of the highest priority active interrupt detected by the SEC prioritizer.

Figure 7-6: SEC\_CACT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000005_012dda8036880769e15b63a18bc08363bd1fb4fa5b6a8b47819332183261fdc4.png)

Table 7-5: SEC\_CACT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | PRIO       | Highest Active IRQ Priority. The SEC_CACT[n].PRIO indicates the priority value of the highest priority active interrupt for core n.   |
| 7:0 (R/NW)         | SID        | Highest Active IRQ Source ID. The SEC_CACT[n].SID identifies the source ID value of the highest priority active interrupt for core n. |

## SCI Control Register n

The SEC control register ( SEC\_CCTL[n] ) contains SCI control bits for all system sources.

Figure 7-7: SEC\_CCTL[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000006_f1fa11ff6c8213b0231728884ac990181f83e0111a78814775b975a44fbebbe7.png)

Table 7-6: SEC\_CCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CCTL[n].LOCK bit is enabled, the SEC_CCTL[n] register is read only.                                         |
| 16 (R/W)           | NMIEN      | NMI Enable. The SEC_CCTL[n].NMIEN bit controls NMI propagation to the core. When the SEC_CCTL[n].NMIEN bit is enabled, the SCI allows NMIs to propagate to the core for servicing. |
| 12 (R0/W)          | WFI        | Wait For Idle. When set, the SEC_CCTL[n].WFI bit forces the SCI to wait for indication of core idle before the SCI resumes activity. 0 No Action                                   |

Table 7-6: SEC\_CCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R0/W)           | RESET      | Reset. When set, the SEC_CCTL[n].RESET bit resets all SCI registers to their default val- ues. 0 No Action 1 Reset                                                                                                                                                                                                                                         |
| 0 (R/W)            | EN         | Enable. The SEC_CCTL[n].EN bit controls operation of the SCI. Clearing the SEC_CCTL[n].EN bit halts the execution of the SCI without resetting status regis- ters. (The INT signal to a core is not affected.) Setting the SEC_CCTL[n].EN bit enables the SCI to begin or resume operation with the current configuration and sta- tus. 0 Disable 1 Enable |

## SCI Group Mask Register n

The SEC SCI group mask register ( SEC\_CGMSK[n] ) contains selections for a group mask, an ungroup mask, and a register lock. This register contains the system interrupt group masks for the connected core. The core uses the SEC\_CGMSK[n].UGRP and SEC\_CGMSK[n].GRP fields to mask (disable) interrupts from the specified groups.

Figure 7-8: SEC\_CGMSK[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000007_95ead4fbe162d3254ded198b1e70b7f77ad1b68de93432177520ea21f2b8b2c1.png)

Table 7-7: SEC\_CGMSK[n] Register Fields

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

Table 7-7: SEC\_CGMSK[n] Register Fields (Continued)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000008_1b33307c36a5a09f423c2b8702dfefd3b203d2d6ff3dba8167ff22f940224d08.png)

Lock

Figure 7-9: SEC\_CPLVL[n] Register Diagram

Table 7-8: SEC\_CPLVL[n] Register Fields

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

Table 7-8: SEC\_CPLVL[n] Register Fields (Continued)

| Bit No.   | Bit Name   | Description/Enumeration        |
|-----------|------------|--------------------------------|
| (Access)  |            |                                |
|           |            | 6 7 MSBs (128 priority levels) |
|           |            | 7 8 MSBs (256 priority levels) |

## SCI Priority Mask Register n

The SEC SCI priority mask register ( SEC\_CPMSK[n] ) contains the SCI priority mask for core n and includes a register lock.

Figure 7-10: SEC\_CPMSK[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000009_c4a385978d24698fa2403cd7b2fa859d416213a676ca619f485a2d5baa10f473.png)

Table 7-9: SEC\_CPMSK[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_CPMSK[n].LOCK bit is enabled, the SEC_CPMSK[n] register is read only. 0 Unlock                                                                               |
| 7:0 (R/W)          | PRIO       | IRQ Priority Mask. The SEC_CPMSK[n].PRIO contains the system interrupt priority mask for core n. The core uses the SEC_CPMSK[n].PRIO field to mask (block) interrupts below the specified level. 0 Priority level 0 (highest) 1-254 |

## Core Pending Register n

The SCI pending interrupt register ( SEC\_CPND[n] ) contains the source ID and priority of the highest priority pending interrupt detected by the SEC prioritizer.

Figure 7-11: SEC\_CPND[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000010_fac3ccb5573b5776775e3989ab9d6ffefe571bf3b16ed3bd861c85d02756b48c.png)

Table 7-10: SEC\_CPND[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | PRIO       | Highest Pending IRQ Priority. The SEC_CPND[n].PRIO indicates the priority value of the highest priority pend- ing interrupt for core n.   |
| 7:0 (R/NW)         | SID        | Highest Pending IRQ Source ID. The SEC_CPND[n].SID identifies the source ID value of the highest priority pend- ing interrupt for core n. |

## SCI Source ID Register n

The SCI source ID register ( SEC\_CSID[n] ) contains the source ID of the interrupt last issued to core n. The SEC\_CSID[n] register value is loaded by the SCI when a system interrupt indication is sent to core n. The SCI does not change the SEC\_CSID[n] until after the interface receives an interrupt acknowledge from core n. Writing to the SEC\_CSID[n] register generates an interrupt acknowledge, but does not update the value in the register.

Figure 7-12: SEC\_CSID[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000011_eba4efa897144d3d7f59347d4035d2c40429131ef72784f3c02727a53c6f7522.png)

Table 7-11: SEC\_CSID[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID.                                                                       |
| (R/NW)             |            | The SEC_CSID[n].SID bit is the source ID of the interrupt last issued to core n. |

## SCI Status Register n

The SCI status register ( SEC\_CSTAT[n] ) contains status bits, indicating the operational status of the SCI.

Figure 7-13: SEC\_CSTAT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000012_c66758d7e13d2e0e27e50f707e11b125096fe7f863c2066c5e21eece17f45f6e.png)

Table 7-12: SEC\_CSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W1C)         | NMI        | Non-Maskable Interrupt. The SEC_CSTAT[n].NMI bit indicates whether an NMI has occurred since the bit was last cleared.                                                                                                                                                                                                                                                                                                          |
| 12 (R/W1C)         | WFI        | Wait For Idle. The SEC_CSTAT[n].WFI bit indicates (if set) that the SCI is temporarily disabled, pending a core idle indication. This bit is set when SEC_CCTL[n].WFI is set.                                                                                                                                                                                                                                                   |
| 10 (R/NW)          | SIDV       | SID Valid. The SEC_CSTAT[n].SIDV bit indicates (if set) that the current value in the SEC_CSID[n] register is valid. The SCI sets the SEC_CSTAT[n].SIDV bit when the updating the SEC_CSID[n] register with a new value. The SEC_CSTAT[n].SIDV bit is cleared when the SEC_CSID[n] register is written. This status indication may be used to extract all pending interrupts in a single inter- rupt service routine. 0 Invalid |
| 10 (R/NW)          |            | 1 Valid                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 10 (R/NW)          |            |                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 10 (R/NW)          |            |                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 7-12: SEC\_CSTAT[n] Register Fields (Continued)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000013_bbc5e6e97f0fbede6fdec8e6cf11b0f2b47b09241ac3f849ff062fd158b2b5c6.png)

Table 7-13: SEC\_END Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID IRQ to End.                                                         |
| (R/W)              |            | The SEC_END.SID bit field contains the source ID interrupt service end value. |

## Fault COP Period Register

The SEC fault COP period register ( SEC\_FCOPP ) contains the width value (count in (SEC) clock cycles) for the high and low phase of the computer operating properly (COP) toggled output on the COP pin. Note that the actual high/low phase value is the SEC\_FCOPP.COUNT programmed value plus 1.

Figure 7-15: SEC\_FCOPP Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000014_5158e54399b684cca4e7d9ed215609fcc76d765138a6337e74f7da096c9447d9.png)

Table 7-14: SEC\_FCOPP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | COUNT      | Fault COP Period. The SEC_FCOPP.COUNT bit field is the width value for the high and low phase of the computer operating properly (COP) toggled output on the COP pin. |

## Fault COP Period Current Register

The SEC fault COP period current register ( SEC\_FCOPP\_CUR ) contains the active count (in (SEC) clock periods) for the current phase (high or low) of the computer operating properly (COP) toggled output on the COP pin. The SEC loads the SEC\_FCOPP\_CUR register from the SEC\_FCOPP register when the SEC\_FCOPP\_CUR.COUNT

field is cleared and the SEC is in COP mode ( SEC\_FCTL.CMS bit =1). The SEC decrements the SEC\_FCOPP\_CUR count each (SEC) clock cycle while SEC\_FCTL.CMS is set and the SEC\_FSTAT.ACT bit is not set.

Figure 7-16: SEC\_FCOPP\_CUR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000015_d02c97ac5ac9501362d4b404d12dc44c9ea86aae8caac78e5183cf8b79d4f328.png)

Table 7-15: SEC\_FCOPP\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault COP Period. The SEC_FCOPP_CUR.COUNT bit field is the active count for the current phase (high or low) of the computer operating properly (COP) toggled output on the COP pin. |

## Fault Control Register

The SEC fault control register ( SEC\_FCTL ) contains fault control bits for all SEC channels. This register controls the operation of the System Fault Management Interface (SFI).

Figure 7-17: SEC\_FCTL Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000016_17b4799425c14401785fa26d0ecebc34004dfbd688ed0bfcb7265997de5a364f.png)

Table 7-16: SEC\_FCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_FCTL.LOCK bit is enabled, the SEC_FCTL register is read only. 0 UnLock 1 Lock                                                                                                                                                               |
| 13 (R/W)           | TES        | Trigger Event Select. The SEC_FCTL.TES bit selects the event that directs the SEC to assert trigger out- put. In fault pending mode, the SEC asserts trigger output when a fault is pending. In fault active mode, the SEC asserts trigger output when a fault is active. 0 Fault Active Mode 1 Fault Pending Mode |

Table 7-16: SEC\_FCTL Register Fields (Continued)

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

Table 7-16: SEC\_FCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Enable. The SEC_FCTL.EN bit controls the operational state of the SEC. Clearing the SEC_FCTL.EN bit halts the execution of the SEC without resetting status registers. Setting the SEC_FCTL.EN bit enables the SEC to begin or resume operation with the current configuration and status. |
| 0 (R/W)            | EN         | 0 Disable                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | EN         | 1 Enable                                                                                                                                                                                                                                                                                   |

## Fault Delay Register

The SEC fault delay register ( SEC\_FDLY ) contains the number ( SEC\_FDLY.COUNT field) of (SEC) clock periods to delay from fault pending to fault active, when actions are enabled.

Figure 7-18: SEC\_FDLY Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000017_337c949a5dcc5ad6bf064ea991bbdd0ea633ef5d07e11b01a8f3b1346f5c8862.png)

Table 7-17: SEC\_FDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | COUNT      | Fault Delay. The SEC_FDLY.COUNT bit field is the number of (SEC) clock periods to delay from fault pending to fault active, when actions are enabled. |

## Fault Delay Current Register

The SEC fault delay current register ( SEC\_FDLY\_CUR ) contains the active count ( SEC\_FDLY\_CUR.COUNT field) in (SEC) clock periods for the delay from fault pending to fault active, when actions are enabled. The count is loaded from the SEC\_FDLY register when a fault becomes pending ( SEC\_FSTAT.PND bit is set). The SEC decrements the value in SEC\_FDLY\_CUR each (SEC) clock cycle while the SEC\_FSTAT.PND bit is set.

Figure 7-19: SEC\_FDLY\_CUR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000018_53cebd6ac46930a5ab4d09093ee8f8dd70c12411ba7ede6929d81c62d2ddebfc.png)

Table 7-18: SEC\_FDLY\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault Delay. The SEC_FDLY_CUR.COUNT bit field is the active count in (SEC) clock periods for the delay from fault pending to fault active, when actions are enabled. |

## Fault End Register

The SEC fault end register ( SEC\_FEND ) contains fault source ID and internal/external fields. This register receives fault end indication from a core.

Figure 7-20: SEC\_FEND Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000019_93b33f0f4030982a4cecda4a5ad177a1920ed733504b7b8944ac3994e17ac048.png)

Table 7-19: SEC\_FEND Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | FEXT       | Fault External. Setting the SEC_FEND.FEXT bit, when the SEC_FEND.SID field is cleared, clears an active fault from an external source. 0 Fault Internal 1 Fault External                                                                                        |
| 7:0 (R/W)          | SID        | Source ID. The SEC_FEND.SID identifies a fault to be ended as indicated to the SEC by the core. The core loads the SEC_FEND.SID field value. If the SEC_FEND.SID value matches the SEC_FSID.SID value, the SEC_FSTAT.PND bit and SEC_FSTAT.ACT bit are cleared. |

## Fault Source ID Register

The SEC fault source ID register ( SEC\_FSID ) contains a fault source ID and internal/external fields.

NOTE:These bits are not reset by system reset so that a fault that automatically triggers a system reset to avoid a fault may be analyzed after the reset occurs.

Figure 7-21: SEC\_FSID Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000020_77247033d174b7680db8ef20762611ec0a8f5e0c4a122ec3e389ec75ea9a364c.png)

Table 7-20: SEC\_FSID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/NW)          | FEXT       | Fault External. The SEC_FSID.FEXT bit indicates that the last active fault was asserted by an exter- nal device. The SEC sets the SEC_FSID.FEXT bit when the SEC_FSTAT.ACT bit is set by the fault input pins. The SEC_FSID.FEXT bit is cleared when the SEC_FSTAT.ACT bit is set by an internal fault or when the external fault is ended. When the SEC_FSID.FEXT bit is set, the SEC_FSID.SID is cleared. 0 Fault Internal |
| 7:0 (R/NW)         | SID        | Source ID. The SEC_FSID.SID identifies the fault assertion detected by the SEC fault inter- face. The SEC loads the SEC_FSID.SID field value when a system fault indication is asserted. The SEC fault interface does not change the SEC_FSID.SID value until the fault is no longer pending or active, as indicated by the SEC_FSTAT.PND bit and SEC_FSTAT.ACT bit being cleared in the SEC_FSTAT register.                 |

## Fault System Reset Delay Register

The SEC fault system reset delay register ( SEC\_FSRDLY ) contains the number ( SEC\_FSRDLY.COUNT field) of (SEC) clock periods for the delay from a fault becoming active to system reset request assertion, if enabled.

Figure 7-22: SEC\_FSRDLY Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000021_f3336963f38ec637697f1ddad752a14013e3f4213ff74d3b34c28364bb5dd638.png)

Table 7-21: SEC\_FSRDLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 31:0               | COUNT      | Fault System Reset Delay. The SEC_FSRDLY.COUNT bit field is the number of (SEC) clock periods for the |
| (R/W)              |            | delay from a fault becoming active to system reset request assertion.                                 |

## Fault System Reset Delay Current Register

The SEC fault system reset delay current register ( SEC\_FSRDLY\_CUR ) contains the active count ( SEC\_FSRDLY\_CUR.COUNT field) in (SEC) clock periods for the delay from fault active to system reset assertion, if enabled. The count is loaded from the SEC\_FSRDLY register when a fault becomes active ( SEC\_FSTAT.ACT bit is set). The SEC decrements the value in SEC\_FSRDLY\_CUR each (SEC) clock cycle while the SEC\_FSTAT.ACT bit is set.

Figure 7-23: SEC\_FSRDLY\_CUR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000022_0aa40406c9530d699eadac5a5ae8ff51efc735ee1ad596d2dbb67fa9437bac31.png)

Table 7-22: SEC\_FSRDLY\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | COUNT      | Fault System Reset Delay. The SEC_FSRDLY_CUR.COUNT bit field is the active count in (SEC) clock periods for the delay from fault active to system reset assertion. |

## Fault Status Register

The SEC fault status register ( SEC\_FSTAT ) indicates the operational status of the SFI.

Figure 7-24: SEC\_FSTAT Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000023_6e48975f3450bc1aa0c33767cd9ed476ffe2d3c69c50ae522802f0d78d0b8901.png)

Table 7-23: SEC\_FSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/NW)          | NPND       | Next Pending Fault. The SEC_FSTAT.NPND bit indicates that one or more sources have signaled fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC_FSTAT.NPND bit when the fault interface de- tects assertion of any enabled fault source input, while either the SEC_FSTAT.PND or SEC_FSTAT.ACT bits are set. The SEC clears the SEC_FSTAT.NPND bit when there are no fault sources waiting. | Next Pending Fault. The SEC_FSTAT.NPND bit indicates that one or more sources have signaled fault assertion, but the input has not yet triggered the fault pending detection in the SEC fault interface. The SEC sets the SEC_FSTAT.NPND bit when the fault interface de- tects assertion of any enabled fault source input, while either the SEC_FSTAT.PND or SEC_FSTAT.ACT bits are set. The SEC clears the SEC_FSTAT.NPND bit when there are no fault sources waiting. |
| 10 (R/NW)          | NPND       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Not Pending                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 9 (R/NW)           | ACT        | Fault Active. The SEC_FSTAT.ACT bit indicates that the SEC has received a fault source input, the current fault delay count (in the SEC_FDLY_CUR register) has expired, and the fault actions are enabled. The SEC also sets the SEC_FSTAT.ACT bit on fault input detection if the SEC_FCTL.FIEN bit is set. The SEC_FSTAT.ACT bit is cleared by writing the ID value of the asserted fault from SEC_FSID register to the SEC_FEND register.                              | Fault Active. The SEC_FSTAT.ACT bit indicates that the SEC has received a fault source input, the current fault delay count (in the SEC_FDLY_CUR register) has expired, and the fault actions are enabled. The SEC also sets the SEC_FSTAT.ACT bit on fault input detection if the SEC_FCTL.FIEN bit is set. The SEC_FSTAT.ACT bit is cleared by writing the ID value of the asserted fault from SEC_FSID register to the SEC_FEND register.                              |
| 9 (R/NW)           | ACT        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | No Fault                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 9 (R/NW)           | ACT        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Active Fault                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 7-23: SEC\_FSTAT Register Fields (Continued)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000024_e28343ac44d2f63a9e1959ca39140251db6ff0dfb1a3a6594a16d2aa936f57ea.png)

Table 7-24: SEC\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_GCTL.LOCK bit is enabled, the SEC_GCTL register is read only.                                                                                                                                                                                                    |
| 1 (R0/W)           | RESET      | Reset. The SEC_GCTL.RESET bit is write-1-action and triggers a soft reset to all SEC reg- isters.                                                                                                                                                                                                                                       |
| 0 (R/W)            | EN         | Enable. The SEC_GCTL.EN bit is read/write and must be set for the SEC to begin/resume SEC operation with the current configuration and status. Clearing the SEC_GCTL.EN bit halts the execution of the SFI and all SCIs. All SSIs remain active, along with all error detection, without resetting status registers. 0 Disable 1 Enable |

## Global Status Register

The SEC global status register ( SEC\_GSTAT ) contains global status bits for the SEC.

Figure 7-26: SEC\_GSTAT Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000025_7b989876ae2f30d202889e279a63b2283dd11159eecb79be5e35b39b6a619e09.png)

Table 7-25: SEC\_GSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Lock Write Error. The SEC_GSTAT.LWERR bit indicates (when set) there was an attempted write to an SEC register while the SEC_GCTL.LOCK bit was set and while the global lock bit was enabled ( SPU_CTL.GLCK bit =1). This status bit is sticky; write-1-to-clear it. 0 No Error |
| 30 (R/W1C)         | ADRERR     | Address Error. The SEC_GSTAT.ADRERR bit indicates that the SEC generated and address error. This status bit is sticky; write-1-to-clear it. 0 No Error 1 Error Occurred                                                                                                         |
| 23:16 (R/NW)       | SID        | Source ID for SSI Error. The SEC_GSTAT.SID bits indicate the source ID that generated the last SSI error conveyed in the SEC_GSTAT.ERRC field.                                                                                                                                  |
| 11:8 (R/NW)        | SCI        | SCI ID for SCI Error. The SEC_GSTAT.SCI bits indicate the number for the specific SCI that generated the last SCI error conveyed in the SEC_GSTAT.ERRC field.                                                                                                                   |

Table 7-25: SEC\_GSTAT Register Fields (Continued)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000026_cacf9622374569a82beca90f358d7787f604f592a7042b61d2c22e778fed376d.png)

Table 7-26: SEC\_RAISE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 7:0                | SID        | Source ID.                                                                           |
| (R/W)              |            | The SEC_RAISE.SID bit field is the source ID of event that is set to pending status. |

## Source Control Register n

The SEC source control register ( SEC\_SCTL[n] ) contains control bits to configure the SEC event sources. This register controls the configuration of the corresponding SEC event source.

Figure 7-28: SEC\_SCTL[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000027_fa0d8fad743d44d1f98cd6a6cbecefc52160d0deb3477bf350be6db2e086e13a.png)

Table 7-27: SEC\_SCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock is enabled ( SPU_CTL.GLCK bit =1) and the SEC_SCTL[n].LOCK bit is enabled, the SEC_SCTL[n] register is read only. 0 Unlock                                                                                        |
| 27:24 (R/W)        | CTG        | Core Target Select. The SEC_SCTL[n].CTG bits selects the specific SEC core interface to which the in- terrupt is mapped. Each system interrupt is mapped uniquely to one specific SEC core interface and (as a result) to a specific core. |

Table 7-27: SEC\_SCTL[n] Register Fields (Continued)

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

Table 7-27: SEC\_SCTL[n] Register Fields (Continued)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000028_6a13d5b02ca6919a9f48ae6f7030ce35a30ae27f5860f17873ceeff918869118.png)

Table 7-28: SEC\_SSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/NW)       | CHID       | Channel ID. The SEC_SSTAT[n].CHID bits indicate the ID of the specific source (from a set of sources sharing one SEC source interface input) that asserted the SEC source interface input. An SEC source interface input may support multiple system sources, in which case the assertion must be qualified by an identifier to determine the channel that gen- erated the assertion. The SEC_SSTAT[n].CHID field provides this value in the form of a numeric reference that is mapped to a specific interrupt source. The prioriti- zation for simultaneously asserted sources is according to ID, with 0 being the highest priority. The SEC_SSTAT[n].CHID is captured when the SEC source interface in- put is acknowledged. |
| 9 (R/W1C)          | ACT        | Active Source. The SEC_SSTAT[n].ACT bit indicates the source has been accepted by a core for servicing, but the service is not yet complete. An SEC_SSTAT[n].ACT bit is set by the SEC when the specific system interrupt is acknowledged by the core through the SEC core interface. An SEC_SSTAT[n].ACT bit is cleared by the SEC when the core provides interrupt service end indication for the specific system interrupt through the SEC core interface. Active                                                                                                                                                                                                                                                             |
| 9 (R/W1C)          | ACT        | 0 Not                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 9 (R/W1C)          | ACT        | 1 Active                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 7-28: SEC\_SSTAT[n] Register Fields (Continued)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000029_ccee027ee6645a8302a6ffba4aa0d2b7ed9c9fd2e797c40f7606bba96d5f7f76.png)

## ADSP-SC58x GICDST Register List

GIC Distributor Port

Table 7-29: ADSP-SC58x GICDST Register List

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

## ADSP-SC58x GICCPU Register List

GIC CPU Port

Table 7-30: ADSP-SC58x GICCPU Register List

| Name                | Description                                  |
|---------------------|----------------------------------------------|
| GICCPU_BIN_PT_ALIAS | Aliased Binary Point Register (ICCABPR)      |
| GICCPU_BIN_PT       | Binary Point Register (ICCBPR)               |
| GICCPU_CTL          | CPU Interface Control Register (ICCICR)      |
| GICCPU_EOI          | End of Interrupt Register (ICCEOIR)          |
| GICCPU_PND_HI       | Highest Pending Interrupt Register (ICCHPIR) |
| GICCPU_INT_ACK      | Interrupt Acknowledge Register (ICCIAR)      |
| GICCPU_PRIO_MSK     | Priority Mask Register (ICCIPMR)             |
| GICCPU_RUN_PRIO     | Running Priority Register (ICCRPR)           |

## ADSP-SC58x GICDST Register Descriptions

GIC Distributor Port (GICDST) contains the following registers.

Table 7-31: ADSP-SC58x GICDST Register List

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000030_8e035e84e49786b11bdb005a2828b46f7ad3a80d3f35cf43d8a689ac5f9600d0.png)

Table 7-32: GICDST\_EN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | VALUE      | Global Interrupt Monitor Enable. The GICDST_EN.VALUE bit field enables global monitoring of the peripheral inter- rupt signals and forwarding pending interrupts to the CPU interfaces. |

## Software Generated Interrupt Priority Register

The GICDST\_SGI\_PRIO[n] register provides the 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the corresponding interrupt.

Figure 7-32: GICDST\_SGI\_PRIO[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000031_2126d72ca86b8c20508230b75cbbee258d390e9815b40a4fc323abda8df1f96e.png)

Table 7-33: GICDST\_SGI\_PRIO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Software Generated Interrupt Priority. The GICDST_SGI_PRIO[n].VALUE bit field contains the 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the correspond- ing interrupt. |

## Shared Peripheral Interrupt Priority Register

The GICDST\_SPI\_PRIO[n] registers provide an 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the corresponding interrupt.

Figure 7-33: GICDST\_SPI\_PRIO[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000032_ea02109e98d20c1bc6628bc2ee57773100ce73edb627cd3e0956909a99c31ff2.png)

Table 7-34: GICDST\_SPI\_PRIO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 0                  | VALUE      | Priority. The GICDST_SPI_PRIO[n].VALUE bit field stores the priority of the corre- sponding interrupt (byte offset 3 to Byte offset 0). |
| (R/W)              |            |                                                                                                                                         |

## Software Generated Interrupt Active Register

The GICDST\_SGI\_ACTIVE registers provide a Set-active bit for each interrupt that the GIC supports. Writing to a Set-active bit Activates the corresponding interrupt. These registers are used when preserving and restoring GIC state.

Figure 7-34: GICDST\_SGI\_ACTIVE Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000033_5dfd8a7f852bb8f21dde460dd6ec7e20d1baa2612f70e4b82d6617b6eb893cc1.png)

Table 7-35: GICDST\_SGI\_ACTIVE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 15:0               | VALUE      | SGI Active If N. The GICDST_SGI_ACTIVE.VALUE bit field provides a Set-active bit for each in- terrupt that the GIC supports. |
| (R/NW)             |            |                                                                                                                              |

## Software Generated Interrupt Control Register

The GICDST\_SGI\_CTL register controls the generation of SGIs.It is implementation defined whether this register has any effect when the forwarding of interrupts by Distributor is disabled by the GICD\_CTLR settings.

Figure 7-35: GICDST\_SGI\_CTL Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000034_c5a801a08a086f6936aa538f44e679a144dc2277eb00745fcee5a8613b6d8448.png)

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

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000035_f0fb17334a93c4deab527eab99682a8563596358c264dfe44e26633938d02f6e.png)

Table 7-37: GICDST\_SGI\_PND\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Software Generated Interrupt Clear-Pending. Writing 1 to a clear-pending bit in the GICDST_SGI_PND_CLR.VALUE bit field clears the pending status of the corresponding peripheral interrupt. Reading a bit iden- tifies whether the interrupt is pending. |

## Software Generated Interrupt Pending Set Register

The GICDST\_SGI\_PND\_SET register provides a set-pending bit for each interrupt supported by the GIC.

Figure 7-37: GICDST\_SGI\_PND\_SET Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000036_6814880bdc6223d9ed162e751bb422a559a27d7e26ee4010e2bac4e8140e8047.png)

Table 7-38: GICDST\_SGI\_PND\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Software Generated Interrupt Set-Pending. Writing 1 to a Set-pending bit in the GICDST_SGI_PND_SET.VALUE bit field sets the status of the corresponding peripheral interrupt to pending. Reading a bit identifies whether the interrupt is pending. |

## Software Generated Interrupt Security Register

The GICDST\_SGI\_SECURITY registers provide a status bit for each interrupt supported by the GIC. Each bit controls whether the corresponding interrupt is in Group 0 or Group 1. Typically, when used with a processor that implements the ARM Security Extensions, Group 0 interrupts are Secure interrupts, and Group 1 interrupts are Non-secure interrupts,

Figure 7-38: GICDST\_SGI\_SECURITY Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000037_d6d15297dee34af8075e2ebbe7197659bd378668b40860e5f93c9e07c3015d2f.png)

Table 7-39: GICDST\_SGI\_SECURITY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 15:0               | VALUE      | Software Generated Interrupt Security.                                                                                     |
| (R/W)              |            | Each bit in the GICDST_SGI_SECURITY.VALUE bit field controls whether the corresponding interrupt is in Group 0 or Group 1. |

## Shared Peripheral Interrupt Register

The GICDST\_SPI[n] register contains bits that provide the status of the SPI[987:0] inputs.

Figure 7-39: GICDST\_SPI[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000038_78c3c4195d5ad82786bfbccc60dbcd01314fe1b35b7723316bf06bb9dcb1adeb.png)

Table 7-40: GICDST\_SPI[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | STAT       | Shared Peripheral Interrupt Status. The GICDST_SPI[n].STAT bit field returns the status of the SPI[987:0] inputs on the distributor where bit [x] = 0 SPI[x] is low and bit [x] = 1 SPI[x] is high. |

## Shared Peripheral Interrupt Active Register

The GICDST\_SPI\_ACTIVE[n] register provides an active bit for each interrupt supported by the GIC.

Figure 7-40: GICDST\_SPI\_ACTIVE[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000039_89093a26e8d2eb79dbf0ae50960254a7952c1fe9f9f055392c249594982f471a.png)

Table 7-41: GICDST\_SPI\_ACTIVE[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Active Bits. The GICDST_SPI_ACTIVE[n].VALUE bit field contains an Active bit for each interrupt supported by the GIC. Reading an active bit identifies whether the corre- sponding interrupt is active (=1) or not active (=0). |

## Shared Peripheral Interrupt Configuration Register

The GICDST\_SPI\_CFG[n] register provides a 2-bit Int\_config field for each interrupt supported by the GIC.

Figure 7-41: GICDST\_SPI\_CFG[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000040_b11ce9ca415534365a2aa6416adec608bf59814d1cdae935608eef1d28939785.png)

Table 7-42: GICDST\_SPI\_CFG[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Configuration. The GICDST_SPI_CFG[n].VALUE bit field identifies whether the corresponding interrupt is: edge-triggered or level-sensitive handled using the 1-N model or using the N-N model |

## Shared Peripheral Interrupt Enable Clear Register

The GICDST\_SPI\_EN\_CLR[n] register provides a clear-enable bit for each interrupt supported by the GIC.

Figure 7-42: GICDST\_SPI\_EN\_CLR[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000041_82661892fd141ca895fb9531b6ca195527fbf1ef28ee915c6574590eb7cb93da.png)

Table 7-43: GICDST\_SPI\_EN\_CLR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Enable Clear Enable. Writing 1 to a GICDST_SPI_EN_CLR[n].VALUE bit disables forwarding of the corresponding interrupt to the CPU interfaces. Reading a bit identifies whether the in- terrupt is enabled. |

## Shared Peripheral Interrupt Enable Set Register

The GICDST\_SPI\_EN\_SET[n] register provides a set-enable bit for each interrupt supported by the GIC.

Figure 7-43: GICDST\_SPI\_EN\_SET[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000042_bb5536ea58e64e8f158ad460f2e79bb915f3e12acecba700fecc1d4b6c641d35.png)

Table 7-44: GICDST\_SPI\_EN\_SET[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Enable. Writing 1 to a GICDST_SPI_EN_SET[n].VALUE bit enables forwarding of the corresponding interrupt to the CPU interfaces. Reading a bit identifies whether the in- terrupt is enabled. |

## Shared Peripheral Interrupt Pending Clear Register

The GICDST\_SPI\_PND\_CLR[n] register provides a clear-pending bit for each interrupt supported by the GIC.

Figure 7-44: GICDST\_SPI\_PND\_CLR[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000043_0fc37d3add6ee4c1402cddc2dee9e53c15b926aca806e566284baf56b42be418.png)

Table 7-45: GICDST\_SPI\_PND\_CLR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Clear Pending Interrupt. Writing 1 to a GICDST_SPI_PND_CLR[n].VALUE bit clears the pending status of the corresponding peripheral interrupt. Reading a bit identifies whether the inter- rupt is pending. |

## Shared Peripheral Interrupt Pending Set Register

The GICDST\_SPI\_PND\_SET[n] register provides a set-pending bit for each interrupt supported by the GIC.

Figure 7-45: GICDST\_SPI\_PND\_SET[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000044_681aa14712d79fa4d736d2beb2fd35b8f6f7cad62b5b50ba3251beed6a73dda3.png)

Table 7-46: GICDST\_SPI\_PND\_SET[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Set Pending Interrupt. Writing 1 to a GICDST_SPI_PND_SET[n].VALUE bit sets the status of the cor- responding peripheral interrupt to pending. Reading a bit identifies whether the inter- rupt is pending. |

## Shared Peripheral Interrupt Security Register

The GICDST\_SPI\_SECURITY[n] register provides a security status bit for each interrupt supported by the GIC.

Figure 7-46: GICDST\_SPI\_SECURITY[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000045_56a97d8af3cd00fe2df87fda885611e871d0ee6c34dd40f5ff16e49075307c6a.png)

Table 7-47: GICDST\_SPI\_SECURITY[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Shared Peripheral Interrupt Security Interrupt Security.                                          |
| (R/W)              |            | The GICDST_SPI_SECURITY[n].VALUE bits control the security status of the corresponding interrupt. |

## Shared Peripheral Interrupt Processor Targets Register

The GICDST\_SPI\_TRGT[n] register provides an 8-bit CPU targets field for each interrupt supported by the GIC.

Figure 7-47: GICDST\_SPI\_TRGT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000046_bd6089fc3a9cf9eea7d748a6836cd59726de1f4ab89455444cde2e14a0237074.png)

Table 7-48: GICDST\_SPI\_TRGT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0                | VALUE      | Shared Peripheral Interrupt Processor Targets. The GICDST_SPI_TRGT[n].VALUE bit field stores the list of processors that the interrupt is sent to if it is asserted. |
| (R/W)              |            |                                                                                                                                                                      |

## ADSP-SC58x GICCPU Register Descriptions

GIC CPU Port (GICCPU) contains the following registers.

Table 7-49: ADSP-SC58x GICCPU Register List

| Name                | Description                                  |
|---------------------|----------------------------------------------|
| GICCPU_BIN_PT_ALIAS | Aliased Binary Point Register (ICCABPR)      |
| GICCPU_BIN_PT       | Binary Point Register (ICCBPR)               |
| GICCPU_CTL          | CPU Interface Control Register (ICCICR)      |
| GICCPU_EOI          | End of Interrupt Register (ICCEOIR)          |
| GICCPU_PND_HI       | Highest Pending Interrupt Register (ICCHPIR) |
| GICCPU_INT_ACK      | Interrupt Acknowledge Register (ICCIAR)      |
| GICCPU_PRIO_MSK     | Priority Mask Register (ICCIPMR)             |
| GICCPU_RUN_PRIO     | Running Priority Register (ICCRPR)           |

## Aliased Binary Point Register (ICCABPR)

The GICCPU\_BIN\_PT\_ALIAS register Provides an alias for the non secure GICCPU\_BIN\_PT register.

Figure 7-48: GICCPU\_BIN\_PT\_ALIAS Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000047_c762332a047b10621bccc2e5060482484f06af05ab274f478c9f1f1f0b364bbd.png)

Table 7-50: GICCPU\_BIN\_PT\_ALIAS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Aliased Priority Value.   |

## Binary Point Register (ICCBPR)

The GICCPU\_BIN\_PT register defines the point at which the priority value fields split into two parts, the group priority field and the sub-priority field. The group priority field is used to determine interrupt preemption.

Figure 7-49: GICCPU\_BIN\_PT Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000048_b1d9f233ce05b161e11de776631dfa65bd98c36e945e2a4810fb36901d2deb1d.png)

Table 7-51: GICCPU\_BIN\_PT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Binary Point Value. The GICCPU_BIN_PT.VALUE bit field defines the point at which the priority val- ue fields split into two parts. |

## CPU Interface Control Register (ICCICR)

The GICCPU\_CTL register enables the signaling of interrupts to the target processors. In a GIC that implements the Security Extensions, provides additional global controls for handling Secure interrupts.

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000049_b31c11b9c3458f8c4da3f5703f77937a12f2e0480963da2c8558f645a0cbb10a.png)

Control N

Figure 7-50: GICCPU\_CTL Register Diagram

Table 7-52: GICCPU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Control N.                |
| (R/W)              |            |                           |

## End of Interrupt Register (ICCEOIR)

A processor writes to the GICCPU\_EOI register to inform the CPU interface that it has completed its interrupt service routine for the specified interrupt.

Figure 7-51: GICCPU\_EOI Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000050_0b509679acd3bd7db04b79e9a79e7a9130f2ca8a9c20088b562e0217f60b9af5.png)

Table 7-53: GICCPU\_EOI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | End of Interrupt. The GICCPU_EOI.VALUE bit field indicates to the CPU interface that it has com- pleted its interrupt service routine for the specified interrupt. |

## Highest Pending Interrupt Register (ICCHPIR)

The GICCPU\_PND\_HI register indicates the Interrupt ID, and processor ID if appropriate, of the pending interrupt with the highest priority on the CPU interface.

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000051_1effd9c134cb0f5a5eafb395774dcbc75dc37183e8e9070e6a683f3bc35b22dc.png)

Hi Pend N

Figure 7-52: GICCPU\_PND\_HI Register Diagram

Table 7-54: GICCPU\_PND\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                      |
|--------------------|------------|----------------------------------------------|
| 31:0               | VALUE      | Hi Pend N.                                   |
| (R/NW)             |            | Highest Pending Interrupt Register (ICCHPIR) |

## Interrupt Acknowledge Register (ICCIAR)

The processor reads the GICCPU\_INT\_ACK register to obtain the interrupt ID of the signaled interrupt. This read acts as an acknowledge for the interrupt.

Figure 7-53: GICCPU\_INT\_ACK Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000052_6d24317b377b269c33366b9a76f8ee6af86febbb4f3b688655471fdce64f1c18.png)

Table 7-55: GICCPU\_INT\_ACK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 31:0 (RC/NW)       | VALUE      | Interrupt Acknowledge ID. The GICCPU_INT_ACK.VALUE bit field contains the interrupt ID of the signaled interrupt. |

## Priority Mask Register (ICCIPMR)

The GICCPU\_PRIO\_MSK register provides an interrupt priority filter. Only interrupts with higher priority than the value in this register can be signaled to the processor.

Figure 7-54: GICCPU\_PRIO\_MSK Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000053_b0e0f67199a5fd03c52d3b2379d825110c5740e07b693299bb3bf8e624e9820a.png)

Table 7-56: GICCPU\_PRIO\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Interrupt Priority Filter Value. The GICCPU_PRIO_MSK.VALUE bit field contains the interrupt priority filter val- ue. |

## Running Priority Register (ICCRPR)

The GICCPU\_RUN\_PRIO register indicates the priority of the highest priority interrupt that is active on the CPU interface.

Figure 7-55: GICCPU\_RUN\_PRIO Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000054_44176b9b113ba05ac69a3d664932a99f0ecb8626b2e264cf9cf1926eeb952544.png)

Table 7-57: GICCPU\_RUN\_PRIO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Run Priority N. The GICCPU_RUN_PRIO.VALUE bit field contains the priority value of the highest priority interrupt that is active on the CPU interface. If there is no active interrupt on the CPU interface, and the GIC implements 8-bit priority fields, a read of this register returns the value 0xFF, corresponding to the low- est possible interrupt priority. If the GIC implements priority fields of less than 8 bits, the read might return the register reset value of 0xFF, or might return a value corre- sponding to the lowest possible interrupt priority. Software cannot determine the num- ber of implemented priority bits from a read of this register. |

## ADSP-SC58x GICDST Register Descriptions

GIC Distributor Port (GICDST) contains the following registers.

Table 7-58: ADSP-SC58x GICDST Register List

| Name               | Description                                         |
|--------------------|-----------------------------------------------------|
| GICDST_EN          | GIC Port 0 Enable                                   |
| GICDST_SGI_PRIO[n] | Software Generated Interrupt Priority Register      |
| GICDST_SPI_PRIO[n] | Shared Peripheral Interrupt Priority Register       |
| GICDST_SGI_ACTIVE  | Software Generated Interrupt Active Register        |
| GICDST_SGI_CTL     | Software Generated Interrupt Control Register       |
| GICDST_SGI_PND_CLR | Software Generated Interrupt Clear-Pending Register |

Table 7-58: ADSP-SC58x GICDST Register List (Continued)

| Name                   | Description                                            |
|------------------------|--------------------------------------------------------|
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

Figure 7-56: GICDST\_EN Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000055_747d9de895a72a92735a25ef096d352f2d8e2eb083b9fc0c0d9f0ea3715235a3.png)

Table 7-59: GICDST\_EN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | VALUE      | Global Interrupt Monitor Enable. The GICDST_EN.VALUE bit field enables global monitoring of the peripheral inter- rupt signals and forwarding pending interrupts to the CPU interfaces. |

## Software Generated Interrupt Priority Register

The GICDST\_SGI\_PRIO[n] register provides the 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the corresponding interrupt.

Figure 7-57: GICDST\_SGI\_PRIO[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000056_e764052f668198279df2eb0581939e0e8cf49d289d4ca22d79a575fd284b93db.png)

Table 7-60: GICDST\_SGI\_PRIO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Software Generated Interrupt Priority. The GICDST_SGI_PRIO[n].VALUE bit field contains the 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the correspond- ing interrupt. |

## Shared Peripheral Interrupt Priority Register

The GICDST\_SPI\_PRIO[n] registers provide an 8-bit priority field for each interrupt supported by the GIC. This field stores the priority of the corresponding interrupt.

Figure 7-58: GICDST\_SPI\_PRIO[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000057_21c4dd58dd52785e2673b934a0561e56510c7d52c13678c2b747f6b200486037.png)

Table 7-61: GICDST\_SPI\_PRIO[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 0                  | VALUE      | Priority. The GICDST_SPI_PRIO[n].VALUE bit field stores the priority of the corre- sponding interrupt (byte offset 3 to Byte offset 0). |
| (R/W)              |            |                                                                                                                                         |

## Software Generated Interrupt Active Register

The GICDST\_SGI\_ACTIVE registers provide a Set-active bit for each interrupt that the GIC supports. Writing to a Set-active bit Activates the corresponding interrupt. These registers are used when preserving and restoring GIC state.

Figure 7-59: GICDST\_SGI\_ACTIVE Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000058_994ba37287d43df25130cae0f5f8856e166af10b87fe7074ed2802f2cf75e980.png)

Table 7-62: GICDST\_SGI\_ACTIVE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 15:0               | VALUE      | SGI Active If N.                                                                                            |
| (R/NW)             |            | The GICDST_SGI_ACTIVE.VALUE bit field provides a Set-active bit for each in- terrupt that the GIC supports. |

## Software Generated Interrupt Control Register

The GICDST\_SGI\_CTL register controls the generation of SGIs.It is implementation defined whether this register has any effect when the forwarding of interrupts by Distributor is disabled by the GICD\_CTLR settings.

Figure 7-60: GICDST\_SGI\_CTL Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000059_74b498d674facc4ef7a8ee163727d053131ef0d02863751222f84e64476232bd.png)

Table 7-63: GICDST\_SGI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/W)        | TRGLSTFILT | Target List Filter. The GICDST_SGI_CTL.TRGLSTFILT bit field determines how the distributor must process the requested SGI.                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 25:24 (R/W)        | TRGLSTFILT | 0 Forward the interrupt to the CPU interfaces specified in the CPUTargetList field                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 25:24 (R/W)        | TRGLSTFILT | 1 Forward the interrupt to all CPU interfaces except that of the processor that requested the interrupt                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 25:24 (R/W)        | TRGLSTFILT | 2 Forward the interrupt only to the CPU interface of the processor that requested the interrupt                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 23:16 (R/W)        | CPUTRGTLST | CPU Target list. When the GICDST_SGI_CTL.CPUTRGTLST bit field TargetList Filter = 0b00, de- fines the CPU interfaces to which the Distributor must forward the interrupt. Each bit of the GICDST_SGI_CTL.CPUTRGTLST bit field refers to the corre- sponding CPU interface, for example CPUTargetList[0] corresponds to CPU interface 0. Setting a bit to 1 indicates that the interrupt must be forwarded to the correspond- ing interface. If this field is 0x00 when TargetListFilter is 0b00, the Distributor does not forward the interrupt to any CPU interface. |

Table 7-63: GICDST\_SGI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | SATT       | Security Value of the SGI. The GICDST_SGI_CTL.SATT bit is implemented only if the GIC includes the Se- curity Extensions. This field is writable only by a Secure access. Any Non-secure write to the GICD_SGIR generates an SGI only if the specified SGI is programmed as Group 1, regardless of the value of bit[15] of the write. 0 Forward the SGI specified in the SGIINTID field to a |
| 3:0                | SGIINTID   | The Interrupt ID of the SGI.                                                                                                                                                                                                                                                                                                                                                                 |

## Software Generated Interrupt Clear-Pending Register

The GICDST\_SGI\_PND\_CLR register provides a clear pending bit for each interrupt supported by the GIC. Writing 1 to a clear-pending bit clears the pending status of the corresponding peripheral interrupt. Reading a bit identifies whether the interrupt is pending.

Figure 7-61: GICDST\_SGI\_PND\_CLR Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000060_5def518f61c1120f81036942ff2edbd08fbdcdc3fd48da70ab8149399c4d9ff8.png)

Table 7-64: GICDST\_SGI\_PND\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Software Generated Interrupt Clear-Pending. Writing 1 to a clear-pending bit in the GICDST_SGI_PND_CLR.VALUE bit field clears the pending status of the corresponding peripheral interrupt. Reading a bit iden- tifies whether the interrupt is pending. |

## Software Generated Interrupt Pending Set Register

The GICDST\_SGI\_PND\_SET register provides a set-pending bit for each interrupt supported by the GIC.

Figure 7-62: GICDST\_SGI\_PND\_SET Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000061_0cfe4f1aba9cc823b61a19f75209e2da973911f73cefb753caebec1b85c81d97.png)

Table 7-65: GICDST\_SGI\_PND\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Software Generated Interrupt Set-Pending. Writing 1 to a Set-pending bit in the GICDST_SGI_PND_SET.VALUE bit field sets the status of the corresponding peripheral interrupt to pending. Reading a bit identifies whether the interrupt is pending. |

## Software Generated Interrupt Security Register

The GICDST\_SGI\_SECURITY registers provide a status bit for each interrupt supported by the GIC. Each bit controls whether the corresponding interrupt is in Group 0 or Group 1. Typically, when used with a processor that implements the ARM Security Extensions, Group 0 interrupts are Secure interrupts, and Group 1 interrupts are Non-secure interrupts,

Figure 7-63: GICDST\_SGI\_SECURITY Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000062_df5a83630300982739ad01fc5a40608d62bbf932421a1ed02c6275494c2d899e.png)

Table 7-66: GICDST\_SGI\_SECURITY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Software Generated Interrupt Security. Each bit in the GICDST_SGI_SECURITY.VALUE bit field controls whether the corresponding interrupt is in Group 0 or Group 1. |

## Shared Peripheral Interrupt Register

The GICDST\_SPI[n] register contains bits that provide the status of the SPI[987:0] inputs.

Figure 7-64: GICDST\_SPI[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000063_de880e72e6051b8391b33362a23c2c6fb652493580c8b4ead16d5faf90d2437f.png)

Table 7-67: GICDST\_SPI[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | STAT       | Shared Peripheral Interrupt Status. The GICDST_SPI[n].STAT bit field returns the status of the SPI[987:0] inputs on the distributor where bit [x] = 0 SPI[x] is low and bit [x] = 1 SPI[x] is high. |

## Shared Peripheral Interrupt Active Register

The GICDST\_SPI\_ACTIVE[n] register provides an active bit for each interrupt supported by the GIC.

Figure 7-65: GICDST\_SPI\_ACTIVE[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000064_8b3f242ce6e7ca76abcd2028f5a0c22f2f954d3b433ded2aa9b74a6cf70178a5.png)

Table 7-68: GICDST\_SPI\_ACTIVE[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Active Bits. The GICDST_SPI_ACTIVE[n].VALUE bit field contains an Active bit for each interrupt supported by the GIC. Reading an active bit identifies whether the corre- sponding interrupt is active (=1) or not active (=0). |
| (R/W)              |            |                                                                                                                                                                                                                                 |

## Shared Peripheral Interrupt Configuration Register

The GICDST\_SPI\_CFG[n] register provides a 2-bit Int\_config field for each interrupt supported by the GIC.

Figure 7-66: GICDST\_SPI\_CFG[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000065_d5185ba50878f5d0bbfbbd345a3dbd2d02efc7702ef3dba6db065d2e59c30abb.png)

Table 7-69: GICDST\_SPI\_CFG[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Configuration. The GICDST_SPI_CFG[n].VALUE bit field identifies whether the corresponding interrupt is: edge-triggered or level-sensitive handled using the 1-N model or using the N-N model |

## Shared Peripheral Interrupt Enable Clear Register

The GICDST\_SPI\_EN\_CLR[n] register provides a clear-enable bit for each interrupt supported by the GIC.

Figure 7-67: GICDST\_SPI\_EN\_CLR[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000066_715284b665416087d7c6b05fc6a06b6f174caccc4b05e3ce44c896e00efd6bf3.png)

Table 7-70: GICDST\_SPI\_EN\_CLR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Enable Clear Enable. Writing 1 to a GICDST_SPI_EN_CLR[n].VALUE bit disables forwarding of the corresponding interrupt to the CPU interfaces. Reading a bit identifies whether the in- terrupt is enabled. |

## Shared Peripheral Interrupt Enable Set Register

The GICDST\_SPI\_EN\_SET[n] register provides a set-enable bit for each interrupt supported by the GIC.

Figure 7-68: GICDST\_SPI\_EN\_SET[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000067_4adb1b0db6d8ec5f19654b3fca61d80684fa2d591110eb61712f18b9a14bd71e.png)

Table 7-71: GICDST\_SPI\_EN\_SET[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Enable. Writing 1 to a GICDST_SPI_EN_SET[n].VALUE bit enables forwarding of the corresponding interrupt to the CPU interfaces. Reading a bit identifies whether the in- terrupt is enabled. |

## Shared Peripheral Interrupt Pending Clear Register

The GICDST\_SPI\_PND\_CLR[n] register provides a clear-pending bit for each interrupt supported by the GIC.

Figure 7-69: GICDST\_SPI\_PND\_CLR[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000068_6696548b80471bb01864f9d0f21819fdbf39c1f4836cb926144e2d7d668c9178.png)

Table 7-72: GICDST\_SPI\_PND\_CLR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Clear Pending Interrupt. Writing 1 to a GICDST_SPI_PND_CLR[n].VALUE bit clears the pending status of the corresponding peripheral interrupt. Reading a bit identifies whether the inter- rupt is pending. |

## Shared Peripheral Interrupt Pending Set Register

The GICDST\_SPI\_PND\_SET[n] register provides a set-pending bit for each interrupt supported by the GIC.

Figure 7-70: GICDST\_SPI\_PND\_SET[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000069_068b396fef8035372ba6d4645c09b59da2e418dc25cf1bee61c97850fda49b27.png)

Table 7-73: GICDST\_SPI\_PND\_SET[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Set Pending Interrupt. Writing 1 to a GICDST_SPI_PND_SET[n].VALUE bit sets the status of the cor- responding peripheral interrupt to pending. Reading a bit identifies whether the inter- rupt is pending. |

## Shared Peripheral Interrupt Security Register

The GICDST\_SPI\_SECURITY[n] register provides a security status bit for each interrupt supported by the GIC.

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000070_f7c1e52f70dbd559eee9cc0d2db58212d6d452b381f7cc08c3a3104a9cdfda9f.png)

VALUE[31:16] (R/W)

Interrupt Security Shared Peripheral Interrupt Security

Figure 7-71: GICDST\_SPI\_SECURITY[n] Register Diagram

Table 7-74: GICDST\_SPI\_SECURITY[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Shared Peripheral Interrupt Security Interrupt Security. The GICDST_SPI_SECURITY[n].VALUE bits control the security status of the corresponding interrupt. |

## Shared Peripheral Interrupt Processor Targets Register

The GICDST\_SPI\_TRGT[n] register provides an 8-bit CPU targets field for each interrupt supported by the GIC.

Figure 7-72: GICDST\_SPI\_TRGT[n] Register Diagram

![Image](10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC)_artifacts/image_000071_18754f2b228b194fd807c272ca781e291b3c43b3d585b8dbc704711cbe77fb01.png)

Table 7-75: GICDST\_SPI\_TRGT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Shared Peripheral Interrupt Processor Targets. The GICDST_SPI_TRGT[n].VALUE bit field stores the list of processors that the interrupt is sent to if it is asserted. |