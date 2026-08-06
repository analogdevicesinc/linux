## 54   System Crossbars (SCB)

A modern system on a chip (SoCs) contains multi-cores, memory controllers, security blocks, and other high speed peripherals. As system integration increases, SoCs need to provide bus connectivity that allows better throughput to reduce performance bottlenecks. While traditional point-to-point connection buses have performed well in smaller systems, there is a need to use advanced switching based bus architectures for efficient handling of data transfer between multiplicity of data sources and sinks in the system. Additionally, mixing various traffic types in the same SoC (for example control, communication over peripherals and computing) while sharing the same bus resources, create different requirements from the Quality of Service (QoS) perspective.

The system crossbars (SCB) are the fundamental building blocks of a switch-fabric style for (on-chip) system bus interconnection. The SCBs connect system bus masters to system bus slaves, providing concurrent data transfer between multiple bus masters and multiple bus slaves. The SCB architecture addresses the challenges described above. The SCB provides sustainable throughput for simultaneous transactions in the system with configurable quality of service for each type of transaction (traffic) as required. A hierarchical model, built from multiple SCBs, provides a power and area efficient system interconnect, which satisfies the performance and flexibility requirements of a specific system.

## SCB Features

The SCBs provide the following features:

- Efficient, pipelined bus transfer protocol for sustained throughput
- Full-duplex bus operation for flexibility and reduced latency
- Concurrent bus transfer support to allow multiple bus masters to access bus slaves simultaneously
- Protection model (privileged or secure) support for selective bus interconnect protection
- Simple priority-based QoS based arbitration model
- Programmable quality of service

## SCB Functional Description

The following sections provide a functional description of the SCB.

- SCB Architectural Concepts

## ADSP-SC58x SCB0 Register List

This describes the System Crossbar fabric.

Table 54-1: ADSP-SC58x SCB0 Register List

| Name             | Description                           |
|------------------|---------------------------------------|
| SCB0_MST[n]_RQOS | Read Quality of Service for Master n  |
| SCB0_MST[n]_WQOS | Write Quality of Service for Master n |

## ADSP-SC58x SCB1 Register List

System Crossbar for DMC Memory Space

Table 54-2: ADSP-SC58x SCB1 Register List

| Name            | Description                     |
|-----------------|---------------------------------|
| SCB1_MST00_SYNC | Mst00 Interface Block Sync Mode |

## ADSP-SC58x SCB3 Register List

System MMR Fabric Registers

Table 54-3: ADSP-SC58x SCB3 Register List

| Name               | Description                          |
|--------------------|--------------------------------------|
| SCB3_DCLK0_WR_TIDE | DCLK0 Interface Block APB WRTidemark |
| SCB3_MST00_SYNC    | SYNC Mode                            |

## SCB Architectural Concepts

This section describes the components of the SCB and the modules connected to it. The basic elements in the SCB are SCB masters, slaves master interfaces, and slave interfaces.

## Masters

The controllers in the system that raise the data request in the form of a read/write transaction on the SCB are called masters. The system bus masters include peripheral Direct Memory Access (DMA) channels. These include the Serial Port (SPORT) DMA, and SPI DMAs, among others. Also included are the Memory-to-Memory DMA channels (MDMA), the L1 code fill block, and the processor cores.

## Slaves

Slaves are SCB connections that are responding to transfer requests. Slaves include MMR registers, memory units, and various peripherals depending upon individual configurations. Each system slave has its own latencies and response times.

## SCB Block Diagrams

The SCB architectural model is illustrated in the SCB Overview figure. This figure shows a high level representation of a basic SCB connecting n Slaves to x Masters. A variable number of masters connect to a variable number of slaves in each SCB. In this example, all SIs connect to all MIs as indicated by the lines connecting them.

Figure 54-1: SCB Overview

![Image](57_System_Crossbars_(SCB)_artifacts/image_000000_9e1e713e4f782fa78b87103a3bbed4dc8741bdc005834c6d8ddb62ee885b5c8f.png)

## Hierarchy Block Diagram

A system interconnect built from multiple SCBs in a hierarchical model is illustrated in the SCB Hierarchy Overview figure. The system master node level SCBs master connects multiple SIs to a single MI, which in turn connects to an SI of the system slave level node SCB.

As discussed above, all the masters in the system are distributed across different SCBs. A given SCB at system master node level connects directly to the system masters. These SCBs connect to SCB0 through its SIs forming a hierarchal structure. While a master has to access any slave, its first access goes through the SCB it is connected to, and then through SCB0, to reach its intended slave. This simplifies the connecting hardware in the basic SCB block by limiting the masters. Care must be taken when sharing masters to allow adequate throughput for their individual data transfer requirements.

In this example, all SIs are connected to all MIs.

Figure 54-2: SCB Hierarchy Overview

![Image](57_System_Crossbars_(SCB)_artifacts/image_000001_3012d6f5aa8979d682176acd64cc837f4270cfad2d7c28d315a5125c686bd3d4.png)

NOTE: For an overall diagram of all SCB interconnections, see the SCB Block Diagram.

## SCB Block Diagram

The following figures show the SCB block diagram. For DMA channel assignments for the peripherals shown in the SCB Block Diagram figure, see the DMA Channel Peripherals Controlled by SCBs table following the figure.

Figure 54-3: SCB Block Diagram

![Image](57_System_Crossbars_(SCB)_artifacts/image_000002_f4864127222497a952455537084047bfd2240b1f6dfc8bbd766f79260be018ba.png)

There are two types of peripherals that use DMA. The first have dedicated DMA channels controlled by the Dedicated DMA Engine (DDE) and have the same operating modes (see DMA Operating Modes) and use the same programming model (DMA Channel Programming Model). The second type is not controlled by the DDE. These

peripherals have their own operating modes and programming models (see the peripheral chapter for this information). The peripheral types are shown in the following table.

Table 54-4: DMA Channel Peripherals Controlled by SCBs

|      | Masters             | DDE DMAChan- nels   | Non DDE DMAChannels                   |
|------|---------------------|---------------------|---------------------------------------|
| SCB1 | SPORT0, HALF A      | DMA0                |                                       |
| SCB1 | SPORT0, HALF B      | DMA1                |                                       |
| SCB1 | SPORT1, HALF A      | DMA2                |                                       |
| SCB1 | SPORT1, HALF B      | DMA3                |                                       |
| SCB1 | SPORT2, HALF A      | DMA4                |                                       |
| SCB1 | SPORT2, HALF B      | DMA5                |                                       |
| SCB1 | SPORT3, HALF A      | DMA6                |                                       |
| SCB1 | SPORT3, HALF B      | DMA7                |                                       |
| SCB1 | Std BWMDMA0CRC, CH0 | DMA8                |                                       |
| SCB1 | Std BWMDMA0CRC, CH1 | DMA9                |                                       |
| SCB2 | SPORT4, HALF A      | DMA10               |                                       |
| SCB2 | SPORT4, HALF B      | DMA11               |                                       |
| SCB2 | SPORT5, HALF A      | DMA12               |                                       |
| SCB2 | SPORT5, HALF B      | DMA13               |                                       |
| SCB2 | SPORT6, HALF A      | DMA14               |                                       |
| SCB2 | SPORT6, HALF B      | DMA15               |                                       |
| SCB2 | SPORT7, HALF B      | DMA16               |                                       |
| SCB2 | SPORT7, HALF B      | DMA17               |                                       |
| SCB2 | Std BWMDMA1CRC, CH0 | DMA18               |                                       |
| SCB2 | Std BWMDMA1CRC, CH1 | DMA19               |                                       |
| SCB3 | UART0, TX           | DMA20               |                                       |
| SCB3 | UART0, RX           | DMA21               |                                       |
| SCB3 | EMAC0 (10/100/1000) | n/a                 | 6 channels (3 receive and 3 transmit) |
| SCB3 | USB0                | n/a                 | 8 channels                            |
| SCB3 | MSI                 | n/a                 | 1 channel                             |
| SCB3 | SINC                | n/a                 | 1 channel                             |
| SCB4 | SPI0, TX            | DMA22               |                                       |
| SCB4 | SPI0, RX            | DMA23               |                                       |
| SCB4 | SPI1, TX            | DMA24               |                                       |

Table 54-4: DMA Channel Peripherals Controlled by SCBs (Continued)

|                   | Masters                  | DDE DMAChan- nels   | Non DDE DMAChannels          |
|-------------------|--------------------------|---------------------|------------------------------|
|                   | SPI1, RX                 | DMA25               |                              |
|                   | SPI2, TX                 | DMA26               |                              |
|                   | SPI2, RX                 | DMA27               |                              |
|                   | PPI, F0                  | DMA28               |                              |
|                   | PPI, F1                  | DMA29               |                              |
| SCB5              | HAE, OUT                 | DMA31               |                              |
|                   | HAE, IN0                 | DMA32               |                              |
|                   | HAE, IN1                 | DMA33               |                              |
|                   | UART1, TX                | DMA34               |                              |
|                   | UART1, RX                | DMA35               |                              |
|                   | FIR0                     | n/a                 | 1 channel                    |
|                   | EMAC1, 10/100            | n/a                 | 2 channels (TX + RX)         |
| SCB6              | UART2, TX                | DMA37               |                              |
|                   | UART2, RX                | DMA38               |                              |
|                   | IIR0                     | n/a                 | 1 channel                    |
|                   | CRYPTO                   | n/a                 | 1 Channel                    |
|                   | USB1                     | n/a                 | 8 channels                   |
| SCB7              | EMDMA0                   | n/a                 | 2 channels                   |
| SCB8              | LP0                      | DMA30               |                              |
|                   | LP1                      | DMA36               |                              |
|                   | MLB0                     | n/a                 | 1 channel (4 TDMslots)       |
| SCB0 Masters only | Enh BWMDMA2, CH0         | DMA39               |                              |
|                   | Enh BWMDMA2, CH0         | DMA40               |                              |
|                   | FFTA (Max BW, MDMA, CH0) | DMA41               |                              |
|                   | FFTA (Max BW, MDMA, CH1) | DMA42               |                              |
|                   | Max BW(MDMA3, CH0)       | DMA43               |                              |
|                   | Max BW(MDMA3, CH0)       | DMA44               |                              |
|                   | PCIe                     | n/a                 | 2 Channels (1 read, 1 write) |

Figure 54-4: SCB Block Diagram - Interconnections

![Image](57_System_Crossbars_(SCB)_artifacts/image_000003_38441a524b6b5494455925a1c098db4c82fc706a53d6b668735ba3bb39c505b1.png)

While this figure is useful just for the overview it provides, it is also useful to observe the following relationships that are highlighted.

- The hierarchy of SCBs manages system bus interconnections, multiplexing, and arbitration among the peripherals on the processor.
- The SCBs connections support DMA channels for some peripherals and support dedicated connections for others (such as USB). The connections also support memory-mapped register access for internal memory (L1 and L2) and for external memory.
- The slave interface of the crossbar (where the masters such as DMA connect to) performs two functions. The first function is arbitration. SCBx handles arbitration. The second function is clock conversion. The programmable QoS registers can be viewed as being associated with the SCBx. For example, the programmable QoS registers for DMA16-17, USB0, GIG-E, SDIO, ACM and SINC can be viewed as residing in SCB3. Whenever a transaction is received at DMA16, the programmed QoS value is associated with that transaction and is arbitrated with the rest of the masters at SCB3.
- Most of the peripherals and their SCBs are in the SCLK0\_0 domain. Crypto is also in the SCLK0\_0 domain. Enhanced bandwidth MDMA, MDMA0 and MDMA1 streams, MLB, DBG, ETR, PCIe and their SCB are in the SYSCLK\_0domain.
- Each peripheral has a latency for access across the SCB. The latency varies with the nature of the peripheral. Also, the number of active peripherals (especially for cases where multiple peripherals are active on a shared

SCB) affects SCB performance. Refer to the L2 System Memory chapter for more details on the L2 memory organization.

MDMA channels are unidirectional. For example, MDMA0 SRC is read-only and MDMA1 is write-only. Access to the MMR space of SCB0 is allowed only in secure mode. The MMR space of SCB0 has registers for programming the QoS of various masters and controlling the clock domain crossing.

The following are definitions of acronyms that appear in the figure:

## DMA0-DMA35

Indicate DMA channels for peripherals supporting DMA transfers.

## SCB0-8

Indicate SCB interfaces, connecting the system bus masters and slaves.

## SCLK0, SCLK1, SYSCLK, LPCLK

Indicate clock domains in which the specific SCBs operate. For more information on clock domains, see the Clock Generation Unit (CGU) chapter and the product data sheet.

## CDC

Indicates the clock domain crossing.

## C0\_D, C1\_D

Indicates the SHARC processor (0 or 1) data bus (DM/PM).

## C0\_I, C1\_I

Indicates the SHARC processor (0 or 1) instruction bus (PM).

## L2CC

Indicates the system L2 cache.

## A5

Indicates the ARM Cortex A5 processor.

## L1C0 S1, L1C0 S2, L1C1 S1, L1C1 S2

Indicates the L1 memory for SHARC+ processor (0 or 1) slave port (1 or 2).

## DMC0, DMC1

Indicates the dynamic memory controller (DMC) interfaces.

## PCIe

Indicates the PCI Express interface.

## SP0 A/B, SP1 A/B

Indicates the serial port interfaces and their full-duplex halves.

## SPI0, SPI1 - RX/TX

Indicates the serial peripheral interfaces ports with receive or transmit paths.

## DBG, ETR

Indicates the Debug Port (DBG) and Embedded Trace Router (ETR) which provide access to debug and trace capabilities.

## SMPU

Indicates the system memory protection unit (SMPU).

## MDMA0, MDMA1

Indicates the memory DMA 0 through 1 interfaces.

## USB

Indicates the universal serial bus (USB) interface.

## SMMR

Indicates the system memory-mapped register interface.

## System Crossbars

The System Crossbars (SCB) are the fundamental building blocks of the system bus interconnect. The SCB (often referred to as the system interconnect fabric), is a collection of inter-connection units connecting system masters to slave memory spaces. The SCB connects one or more master devices to one or more memory-mapped slave devices. Each connected master can be a core that originates an SCB transaction, or a master interface of an upstream SCB cascaded interconnect. Each connected slave can be the final target of an SCB transaction or a slave interface of a downstream cascaded SCB interconnect (forming a hierarchy of SCBs).

Each SCB that has multiple masters and slaves share the total bandwidth of the SCB. (In a M:N configuration where M masters are connected to N slaves through the SCBx.)

The SCB provides separate channels for reads and writes. Read and write accesses through a given SCB do not share bandwidth. All the SCBs are 32 bits wide and run at SCLK speed, and can provide a bandwidth of up to 400 Mbytes per second for reads and writes separately (when SCLK = 100 MHz). Only SCB0, which is the major SCB in the SCB hierarchy, has the multiple paths between multiple master and slave interfaces.

See the SCB Block Diagram.

All other SCBs in the chip connect to SCB0 through different slave interfaces. Other primary masters (DMAs, cores, and so on) in the system are distributed across these small SCBs. For a given SCB, all the master and slaves share the total bandwidth of the SCB. (Only SCB0 is the exception). Since different DMA channels are scattered across different SCBs (SCB1, SCB2 SCB3, and so on), they do not conflict for the bandwidth as long as they are in different SCB and are accessing different slaves. SCB0 allows for concurrent data transfer between multiple bus masters and multiple bus slaves, providing flexibility, and full-duplex operation. For example, the data transfer between SCB4 (one of the MDMA channels), and SMC controller (accessing SRAM memory) can happen in parallel to SCB2 (SPI RX/TX DMA) accessing memory mapped SPI memory with both the transfers occurring at up to 400 MBPS. If system accesses are carefully architected, SCB has a potential of providing sufficient sustained bandwidth in the end system.

Since the SCBs support burst transfers, it is important to configure the requesting master appropriately to make best use of available SCB bandwidth. For a DMA master, choosing the appropriate DMA\_CFG.MSIZE value, is important from both a functional and a performance perspective. The value in the DMA\_CFG.PSIZE bit field determines the width of the peripheral bus in use. It can be configured to 1-byte, 2-bytes, or 4-bytes. The DMA\_CFG.MSIZE value determines the actual size of the SCB bus in use. It also determines the minimum number of bytes which are transferred from or to memory corresponding to a single DMA request or grant. The transfer can be 1-, 2-, 4-, 8-, 16-, or 32-bytes. If the DMA\_CFG.MSIZE value is greater than the SCB bus width, the SCB performs burst transfers according to the width defined in DMA\_CFG.MSIZE . When DMA\_CFG.MSIZE is less than the SCB bus width, bursting is not supported and partial bus use results.

Each of the SCB unit in the fabric consists of N Slave interfaces (MSTn). Each of these interfaces has controls for read quality of service, write quality of service, and functional mode. A subset of these matrices includes controls for IB (Interface Block) sync mode, and bus functional mode. For more details on IB, see the clock domain synchronization section.

## Clock Domain Synchronization

Most of the masters in the system operate at the same clock as the SCB, which is SCLK0\_0. There is no need to synchronize. The M4 core and L1 code fill blocks (used for cache fill from SPI flash memory or SMC memory) operate in the CCLK domain. The SCB provides the option to program the different synchronization schemes for these masters through the sync mode registers ( SCB3\_MST00\_SYNC , and SCB1\_MST00\_SYNC ).

These registers perform clock domain crossing synchronization from CCLK to SCLK. The configuration of these registers depends on the CCLK and SCLK configuration.

## SCB Bus Master IDs

The SCB bus master ID tables indicate which masters are connected to each of the slave ports of SCB0. The tables also indicate the precise value of the ID as seen by the slave. These values are useful for SWU programming.

NOTE: For an overall diagram of all SCB interconnections, see the SCB Block Diagram.

Table 54-5: Bus Master IDs

| SCB   | Masters                     | Hex ID Values                                                                                                       |
|-------|-----------------------------|---------------------------------------------------------------------------------------------------------------------|
| SCB1  | DMA0 (SPORT0, HALF A)       | 0x0, 0x10 (descriptor fetch)                                                                                        |
| SCB1  | DMA1 (SPORT0, HALF B)       | 0x1, 0x11 (descriptor fetch)                                                                                        |
| SCB1  | DMA2 (SPORT1, HALF A)       | 0x2, 0x12 (descriptor fetch)                                                                                        |
| SCB1  | DMA3 (SPORT1, HALF B)       | 0x3, 0x13 (descriptor fetch)                                                                                        |
| SCB1  | DMA4 (SPORT2, HALF A)       | 0x4, 0x14 (descriptor fetch)                                                                                        |
| SCB1  | DMA5 (SPORT2, HALF B)       | 0x5, 0x15 (descriptor fetch)                                                                                        |
| SCB1  | DMA6 (SPORT3, HALF A)       | 0x6, 0x16 (descriptor fetch)                                                                                        |
| SCB1  | DMA7 (SPORT3, HALF B)       | 0x7, 0x17 (descriptor fetch)                                                                                        |
| SCB1  | DMA8 (Std BWMDMA0CRC, CH0)  | 0x8, 0x18 (descriptor fetch)                                                                                        |
| SCB1  | DMA9 (Std BWMDMA0CRC, CH1)  | 0x9, 0x19 (descriptor fetch)                                                                                        |
| SCB2  | DMA10 (SPORT4, HALF A)      | 0x200, 0x210 (descriptor fetch)                                                                                     |
| SCB2  | DMA11 (SPORT4, HALF B)      | 0x201, 0x211 (descriptor fetch)                                                                                     |
| SCB2  | DMA12 (SPORT5, HALF A)      | 0x202, 0x212 (descriptor fetch)                                                                                     |
| SCB2  | DMA13 (SPORT5, HALF B)      | 0x203, 0x213 (descriptor fetch)                                                                                     |
| SCB2  | DMA14 (SPORT6, HALF A)      | 0x204, 0x214 (descriptor fetch)                                                                                     |
| SCB2  | DMA15 (SPORT6, HALF B)      | 0x205, 0x215 (descriptor fetch)                                                                                     |
| SCB2  | DMA16 (SPORT7, HALF A)      | 0x206, 0x216 (descriptor fetch)                                                                                     |
| SCB2  | DMA17 (SPORT7, HALF B)      | 0x207, 0x217 (descriptor fetch)                                                                                     |
| SCB2  | DMA18 (Std BWMDMA1CRC, CH0) | 0x208, 0x218 (descriptor fetch)                                                                                     |
| SCB2  | DMA19 (Std BWMDMA1CRC, CH1) | 0x209, 0x219 (descriptor fetch)                                                                                     |
| SCB3  | DMA20 (UART0 TX) EMAC0      | 0x401,0x411 (descriptor fetch)                                                                                      |
| SCB3  | DMA21 (UART0 RX) USB0       | 0x402, 0x412 (descriptor fetch)                                                                                     |
| SCB3  | EMAC0                       | 0x400,0x410,0x420,0x430,0x440,0x450,0x460,0x470,0 x480,0x490,0x4A0,0x4B0,0x4C0,0x4D0,0x4E0,0x4F0 (descriptor fetch) |
| SCB3  | SINC                        | 0x404, 0x40C (descriptor fetch)                                                                                     |
| SCB3  | SDIO                        | 0x403                                                                                                               |

Table 54-5: Bus Master IDs (Continued)

| SCB   | Masters           | Hex ID Values                                                                                                         |
|-------|-------------------|-----------------------------------------------------------------------------------------------------------------------|
|       | USB0              | 0x405                                                                                                                 |
| SCB4  | DMA22 (SPI0, TX)  | 0x800, 0x810 (descriptor fetch)                                                                                       |
| SCB4  | DMA23 (SPI0, RX)  | 0x801, 0x811 (descriptor fetch)                                                                                       |
| SCB4  | DMA24 (SPI1, RX)  | 0x803, 0x813 (descriptor fetch)                                                                                       |
| SCB4  | DMA25 (SPI1, TX)  | 0x802, 0x812 (descriptor fetch)                                                                                       |
| SCB4  | DMA26 (SPI2, RX)  | 0x805, 0x815 (descriptor fetch)                                                                                       |
| SCB4  | DMA27 (SPI2, TX)  | 0x804, 0x814 (descriptor fetch)                                                                                       |
| SCB4  | DMA28 (PPI, F0)   | 0x806, 0x816 (descriptor fetch)                                                                                       |
| SCB4  | DMA29 (PPI, F1)   | 0x807, 0x817 (descriptor fetch)                                                                                       |
| SCB5  | FIR (CH0)         | 0xA06                                                                                                                 |
| SCB5  | FIR (CH1)         | 0xA07                                                                                                                 |
| SCB5  | DMA31 (HAE, IN0)  | 0xA00, 0xA10 (descriptor fetch)                                                                                       |
| SCB5  | DMA32 (HAE, IN1)  | 0xA01, 0xA11 (descriptor fetch)                                                                                       |
| SCB5  | DMA33 (HAE, OUT)  | 0xA02, 0xA12 (descriptor fetch)                                                                                       |
| SCB5  | DMA29 (UART1, RX) | 0xA04, 0xA14 (descriptor fetch)                                                                                       |
| SCB5  | DMA30 (UART1, TX) | 0xA03, 0xA13 (descriptor fetch)                                                                                       |
| SCB5  | EMAC1             | 0xA05,0xA15,oxA25,oxA35,0xA45,0xA55,0xA65,0xA75 , 0xA85,0xA95,0xAA5,0xAB5,0xAC5,0xAd5,0xAE5,0xA F5 (descriptor fetch) |
| SCB6  | IIR (CH0)         | 0xC06                                                                                                                 |
| SCB6  | IIR (CH1)         | 0xC07                                                                                                                 |
| SCB6  | DMA34 (UART2, RX) | 0xC04, 0xC14 (descriptor fetch)                                                                                       |
| SCB6  | DMA35 (UART2, TX) | 0xC05, 0xC15 (descriptor fetch)                                                                                       |
| SCB6  | CRYPTO            | 0xC03                                                                                                                 |
| SCB6  | USB1              | 0xC00                                                                                                                 |
| SCB7  | EMDMA0 (CH0)      | 0xE00                                                                                                                 |
| SCB7  | EMDMA0 (CH1)      | 0xE01                                                                                                                 |
| SCB7  | EMDMA1 (CH0)      | 0xE02                                                                                                                 |
| SCB7  | EMDMA1 (CH1)      | 0xE03                                                                                                                 |

Table 54-5: Bus Master IDs (Continued)

| SCB    | Masters            | Hex ID Values                                |
|--------|--------------------|----------------------------------------------|
| SCB8   | DMA30 LP0          | 0x601, 0x611 (descriptor fetch)              |
| SCB8   | DMA36 LP1          | 0x602, 0x612 (descriptor fetch)              |
| SCB8   | MLB                | 0x600                                        |
| SCB8   | DBG                | 0x603                                        |
| SCB8   | ETR                | 0x604                                        |
| DSPMEM | DMA39 (MSMDMA CH1) | 0x1001, 0x1011                               |
|        | DMA40 (MSMDMA CH0) | 0x1000, 0x1010                               |
|        | PCIe               | There are 32 values. "x" is either a 0 or 1. |
|        | DMA43 (HSMDMA CH0) | 0x1200, 0x1210                               |
|        | DMA44 (HSMDMA CH1) | 0x1201, 0x1211                               |
|        | DMA41 (FFTA CH0)   | 0x1202, 0x1212                               |
|        | DMA42 (FFTA CH1)   | 0x1203, 0x1213                               |
|        | SH0 (DPORT)        | 0x1400                                       |
|        | SH0 (IPORT)        | 0x1401                                       |
|        | SH1 (DPORT)        | 0x1402                                       |
|        | SH1 (IPORT)        | 0x1403                                       |
|        | PL310 (M0)         | There are 32 values. "x" is either a 0 or 1. |
|        | PL310 (M1)         | There are 32 values. "x" is either a 0 or 1. |

## SCB Programming Model

The following sections provide information for programming the SCB properly.

## Programming SCB Arbitration

Each slave interface has a QoS value (priority) associated with both read and write channels. These values are 4 bits and are located in the SCB0\_MST[n]\_RQOS and SCB0\_MST[n]\_WQOS registers. At the entry point to the infrastructure, all transactions are allocated this programmable local QoS value. The arbitration of the transaction throughout the infrastructure uses this QoS. At any arbitration node, a fixed priority exists for transactions with a different QoS. The highest value has the highest priority.

If there are coincident transactions at an arbitration node with the same QoS that require arbitration, then the network uses a Least Recently Granted (LRG) algorithm. At each switch, the master with the highest QoS acquires access and that switch output takes the QoS value of the winner for that transaction. At the next switch slave interface, the master uses the QoS value of the winner. QoS can have values from 0 (lowest priority) to 15 (highest priority).

For example in the following figure, SCB Arbitration :

1. At SCB1, masters (1, 2, 3) have RQOS values of (6, 4, 2)
2. At SCB2, masters (4, 5, 6) have RQOS values of (12, 13, 1)

Figure 54-5: SCB Arbitration

![Image](57_System_Crossbars_(SCB)_artifacts/image_000004_54ea1e20b4cb3dab58361a8068f6ae491db57605f9947b9f7185b520e5f008d5.png)

In this case, master 1 wins at SCB1, and master 5 wins at SCB2. However, in a perfect competition at SCB0, masters 4 and 5 had the highest overall RQOS values. Masters 4 and 5 would have fought for arbitration directly at SCB0. However, because of the mini-SCBs, master 1, at a much lower RQOS value, is able to win against master 4 and make it all the way to SCB0.

## Programming Clock Domain Crossing Registers

In addition to the QoS registers, the interconnect has a set of registers to program the clock domain crossing (CDC) in the interface blocks. The clock domain crossing can be programmed to be one of the following:

- SYNC 1:1
- SYNC 1:n
- SYNC n:1
- ASYNC
- SYNC m:n

The CDC can be programmed at the interface between DCLK and the SYSCLK by writing appropriate bits in the SCB1\_MST00\_SYNC register (address 0x30200020) for DMC0 and the SCB3\_MST00\_SYNC register (address 0x30300020) for DMC1. Refer to the Register Description section for more details.

To change the clock domain crossing mode, follow the actions described in the Changing Clock Domain Crossing Modes table.

Table 54-6: Changing Clock Domain Crossing Modes

| Original Mode   | Required Mode   | Action                                              |
|-----------------|-----------------|-----------------------------------------------------|
| ASYNC           | Any other mode  | Change the clocks then change the MMRregister       |
| Any mode        | ASYNC           | Change the MMRregister then change clocks to ASYNC. |
| m:n             | 1:1             | Change the clocks, then change the register.        |

Table 54-6: Changing Clock Domain Crossing Modes (Continued)

| Original Mode   | Required Mode   | Action                                       |
|-----------------|-----------------|----------------------------------------------|
| 1:1             | m:n             | Change the register, then change the clocks. |

The other CDCs are not programmable. The default values are:

- SCLK : SYSCLK CDC - 1: n

- CCLK : SYSCLK CDC - m : 1

- SYSCLK : CCLK CDC - 1 : n

- LPCLK : SYSCLK CDC - ASYNC

## SCB Programming Concepts

The SCB arbitration model among master or slave SCBs of the processor is fixed (not programmable). But, each slave does have a quality of service (QoS) programmable feature that affects arbitration.

The arbitration of transactions in SCB is based on the QoS value or the priority of the transaction. All masters with the same priority form a priority group. Arbitration is granted to the highest priority group from which a member is trying to win access, and within that group, to the highest master at that time. When a master wins arbitration, it is relegated to the bottom of its group to ensure that it cannot prevent other masters in its group from accessing the slave.

If you configure all master priorities to different levels, the arbiter implements a fixed priority scheme. This scheme occurs because each master is in a group of its own, and therefore, masters maintain their ordering.

The LRG and fixed priority modes concurrently exist when the master priority value registers are programmed with a combination of identical and unique values.

NOTE: The SCB arbitration hierarchy is fixed (for example, SCB1 master to SCB1 slave). However, multiple master inputs to the same slave permit QoS programming.

The LRG Arbitration Example figure shows three groups with different QoS values. Masters in the same group share a QoS value. The arbitration occurs using an LRG scheme.

Figure 54-6: LRG Arbitration Example

![Image](57_System_Crossbars_(SCB)_artifacts/image_000005_81b25e6617ddb0295e01df5cb88e71b3c507be5f82930fd2c6bcb3cf386db957.png)

The QoS value assigned to a transaction at entry point is carried forward by the transaction as it passes through all arbitration stages in the SCB. QoS for all masters is configured as programmable in system fabric interconnect in ADSP-SC58x.

The priority of the masters fits into three groups:

- Group A - Peripherals without flow control
- Group B - Cores and debug ports
- Group C - Peripherals with flow control or low bandwidth and MDMAs
- Group D - Peripherals with flow control but without bandwidth control

## Group A: Peripherals without Flow Control

These masters are assigned the highest priority. Long delays in servicing requests due to losing arbitration could result in underrun or overrun of FIFOs and loss of data. All masters in this group are assigned the QoS reset value of 13. The Group A masters include the following peripherals:

- 10/100/1000 EMAC
- EPPI
- MLB (synchronous mode does not support flow control)
- Serial ports
- 10/100 EMAC

## Group B: Cores and Debug Ports

Typically, software running on the cores has the highest priority after the peripherals in Group A. These masters are assigned the QoS reset value of 9.

## Group C: Peripherals with Flow Control or Low Bandwidth and MDMAs

This group includes MDMAs and peripherals with flow control or low bandwidth. These masters are assigned the QoS reset value of 5. The Group C masters include the following:

- USB
- SDIO
- Link Ports
- PCIe
- HAE
- SINC
- Standard-BW MDMA
- Enhanced-BW MDMA
- Max-BW MDMA
- FFT-MDMA (Shared Max-BW MDMA)
- SPI
- UART

Although HAE and SINC do not have any flow control, their bandwidth requirement is much less, which places them in this group. MDMAs are given the same QoS as other peripherals with flow control since they can be made to consume lesser bandwidth by using the bandwidth control feature in the DDE.

## Group D: Peripherals with Flow Control but without Bandwidth Control

Peripherals with flow control but without bandwidth control are given the lowest priority. This priority prevents them from flooding the system bandwidth. These peripherals can withstand longer latency times without any data loss, but can potentially keep issuing requests continuously. These masters are assigned the reset value of QoS as 1. The Group D masters include the following peripherals:

- EMDMA (delay line DMA)
- FIR
- IIR
- Crypto

NOTE: Although USB, SDIO, SINC, and other peripherals in Group C do not always have bandwidth control mechanisms like MDMAs, their rate of issuing requests is limited by their low bandwidth requirement. The QoS reset values are spaced out as 1, 5, 9 and 13 instead of 0, 1, 2 and 3. These values allows the software to change the priority of a few masters in the system without having to reprogram the QoS of all the masters.

Table 54-7: QoS Register Table

|   Master IDs | Master          |   read_qos Reset Value |   write_qos Reset Value |
|--------------|-----------------|------------------------|-------------------------|
|            0 | SPORT0_A_DMA    |                     13 |                      13 |
|            1 | SPORT0_B_DMA    |                     13 |                      13 |
|            2 | SPORT1_A_DMA    |                     13 |                      13 |
|            3 | SPORT1_B_DMA    |                     13 |                      13 |
|            4 | SPORT2_A_DMA    |                     13 |                      13 |
|            5 | SPORT2_B_DMA    |                     13 |                      13 |
|            6 | SPORT3_A_DMA    |                     13 |                      13 |
|            7 | SPORT3_B_DMA    |                     13 |                      13 |
|            8 | STD BWMDMA0_SRC |                      5 |                       5 |
|            9 | STD BWMDMA0_DST |                      5 |                       5 |
|           10 | SPORT4_A_DMA    |                     13 |                      13 |
|           11 | SPORT4_B_DMA    |                     13 |                      13 |
|           12 | SPORT5_A_DMA    |                     13 |                      13 |
|           13 | SPORT5_B_DMA    |                     13 |                      13 |
|           14 | SPORT6_A_DMA    |                     13 |                      13 |
|           15 | SPORT6_B_DMA    |                     13 |                      13 |
|           16 | SPORT7_A_DMA    |                     13 |                      13 |
|           17 | SPORT7_B_DMA    |                     13 |                      13 |
|           18 | STD BWMDMA1_SRC |                      5 |                       5 |
|           19 | STD BWMDMA1_DST |                      5 |                       5 |
|           20 | EMAC0           |                     13 |                      13 |
|           21 | USB0            |                      5 |                       5 |
|           22 | UART0_TXDMA     |                      5 |                       5 |
|           23 | UART0_RXDMA     |                      5 |                       5 |
|           24 | MSI0            |                      5 |                       5 |
|           25 | SINC0           |                      5 |                       5 |
|           26 | SPI0_TXDMA      |                      5 |                       5 |

Table 54-7: QoS Register Table (Continued)

|   Master IDs | Master          |   read_qos Reset Value |   write_qos Reset Value |
|--------------|-----------------|------------------------|-------------------------|
|           27 | SPI0_RXDMA      |                      5 |                       5 |
|           28 | SPI1_TXDMA      |                      5 |                       5 |
|           29 | SPI1_RXDMA      |                      5 |                       5 |
|           30 | SPI2_TXDMA      |                      5 |                       5 |
|           31 | SPI2_RXDMA      |                      5 |                       5 |
|           32 | EPPI0_DMA_CH0   |                     13 |                      13 |
|           33 | EPPI0_DMA_CH1   |                     13 |                      13 |
|           34 | FIR0_CH0        |                      1 |                       1 |
|           35 | FIR0_CH1        |                      1 |                       1 |
|           36 | HAE0_RXDMA_CH0  |                      5 |                       5 |
|           37 | HAE0_RXDMA_CH1  |                      5 |                       5 |
|           38 | HAE0_TXDMA      |                      5 |                       5 |
|           39 | UART1_TXDMA     |                      5 |                       5 |
|           40 | UART1_RXDMA     |                      5 |                       5 |
|           41 | EMAC1           |                     13 |                      13 |
|           42 | IIR0_CH0        |                      1 |                       1 |
|           43 | IIR0_CH1        |                      1 |                       1 |
|           44 | USB1            |                      5 |                       5 |
|           47 | PKTE0 (Crypto)  |                      1 |                       1 |
|           48 | UART2_TXDMA     |                      5 |                       5 |
|           49 | UART2_RXDMA     |                      5 |                       5 |
|           50 | EMDMA0_CH0      |                      1 |                       1 |
|           51 | EMDMA0_CH1      |                      1 |                       1 |
|           52 | EMDMA1_CH0      |                      1 |                       1 |
|           53 | EMDMA1_CH1      |                      1 |                       1 |
|           54 | PCIE0           |                      5 |                       5 |
|           55 | ENH BWMDMA2_SRC |                      5 |                       5 |
|           56 | ENH BWMDMA2_DST |                      5 |                       5 |
|           57 | MLB0            |                     13 |                      13 |
|           58 | LP0_DMA         |                      5 |                       5 |
|           59 | LP1_DMA         |                      5 |                       5 |

Table 54-7: QoS Register Table (Continued)

|   Master IDs | Master                                     |   read_qos Reset Value |   write_qos Reset Value |
|--------------|--------------------------------------------|------------------------|-------------------------|
|           60 | CS_DAP                                     |                      9 |                       9 |
|           61 | CS_ETR                                     |                      9 |                       9 |
|           62 | MAX BWMDMA3_SRC                            |                      5 |                       5 |
|           63 | MAX BWMDMA3_DST                            |                      5 |                       5 |
|           64 | MAX BWMDMA(Shared Max-BW MDMA) FFTA0_TXDMA |                      5 |                       5 |
|           65 | MAX BWMDMA(Shared Max-BW MDMA) FFTA0_RXDMA |                      5 |                       5 |
|           66 | C1_DPORT                                   |                      9 |                       9 |
|           67 | C1_IPORT                                   |                      9 |                       9 |
|           68 | C2_DPORT                                   |                      9 |                       9 |
|           69 | C2_IPORT                                   |                      9 |                       9 |
|           70 | C0_L2CC_M0                                 |                      9 |                       9 |
|           71 | C0_L2CC_M1                                 |                      9 |                       9 |

## ADSP-SC58x SCB0 Register Descriptions

System Interconnect Fabric (SCB0) contains the following registers.

Table 54-8: ADSP-SC58x SCB0 Register List

| Name             | Description                           |
|------------------|---------------------------------------|
| SCB0_MST[n]_RQOS | Read Quality of Service for Master n  |
| SCB0_MST[n]_WQOS | Write Quality of Service for Master n |

## Read Quality of Service for Master n

The SCB0\_MST[n]\_RQOS register indicates the read QOS or priority value for the indicated master. This value is used by the SCBs at different levels to arbitrate among different masters requesting read channel accesses. For mapping of master IDs to peripherals, see the SCB Bus Master IDs table.

Figure 54-7: SCB0\_MST[n]\_RQOS Register Diagram

![Image](57_System_Crossbars_(SCB)_artifacts/image_000006_44903a176feb83f22fab62424bacb52e044d6ac3ce2ebe570544df1b30857d4c.png)

Table 54-9: SCB0\_MST[n]\_RQOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | VALUE      | Read Quality of Service. The SCB0_MST[n]_RQOS.VALUE bit field indicates the read QOS or priority val- ue for the indicated master. |

## Write Quality of Service for Master n

The SCB0\_MST[n]\_WQOS register indicates the write QOS or priority value for the indicated master. This value is used by the SCBs at different levels to arbitrate among different masters requesting write channel accesses. For mapping of master IDs to peripherals, see the SCB Bus Master IDs table.

Figure 54-8: SCB0\_MST[n]\_WQOS Register Diagram

![Image](57_System_Crossbars_(SCB)_artifacts/image_000007_15fed64340f70d4efa02a4927054ad1830a3b9f850269c18fbec8828ea99ac84.png)

Table 54-10: SCB0\_MST[n]\_WQOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | VALUE      | Write Quality of Service. The SCB0_MST[n]_WQOS.VALUE bit field indicates the write QOS or priority value for the indicated master. |

## ADSP-SC58x SCB1 Register Descriptions

System Crossbar for DMC Memory Space (SCB1) contains the following registers.

Table 54-11: ADSP-SC58x SCB1 Register List

| Name            | Description                     |
|-----------------|---------------------------------|
| SCB1_MST00_SYNC | Mst00 Interface Block Sync Mode |

## Mst00 Interface Block Sync Mode

The SCB1\_MST00\_SYNC register is used to program the clock domain crossing in the interface blocks.

Figure 54-9: SCB1\_MST00\_SYNC Register Diagram

![Image](57_System_Crossbars_(SCB)_artifacts/image_000008_aaf264f71bec956784f8685178861d5b6339909e0426e65569575938bd4ccee7.png)

Table 54-12: SCB1\_MST00\_SYNC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | VALUE      | SYNC Mode. The SCB1_MST00_SYNC.VALUE bit field is used to configure the SYSCLK- DCLK CDC for DMCfor the following clock domain boundaries. |
| 2:0 (R/W)          | VALUE      | 0 Sync 1:1                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 1 Sync n:1                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 2 Sync 1:n                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 3 Sync m:n                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 4 Async                                                                                                                                    |
| 2:0 (R/W)          | VALUE      | 5 Reserved                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 6 Reserved                                                                                                                                 |
| 2:0 (R/W)          | VALUE      | 7 Reserved                                                                                                                                 |

## ADSP-SC58x SCB3 Register Descriptions

SMMR Fabric (SCB3) contains the following registers.

Table 54-13: ADSP-SC58x SCB3 Register List

| Name               | Description                          |
|--------------------|--------------------------------------|
| SCB3_DCLK0_WR_TIDE | DCLK0 Interface Block APB WRTidemark |
| SCB3_MST00_SYNC    | SYNC Mode                            |

## DCLK0 Interface Block APB WR Tidemark

The SCB3\_DCLK0\_WR\_TIDE register indicates the maximum permitted number of active transactions before the QoS mechanism is activated.

Figure 54-10: SCB3\_DCLK0\_WR\_TIDE Register Diagram

![Image](57_System_Crossbars_(SCB)_artifacts/image_000009_a59f09dcc26c1fef71fb52885ee2f6a9f34e8626507951eef9ecbf67417ce191.png)

Table 54-14: SCB3\_DCLK0\_WR\_TIDE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0                | VALUE      | Wr Tidemark.                                                                                                                                |
| (R/W)              |            | The SCB3_DCLK0_WR_TIDE.VALUE bit field indicates the maximum permitted number of active transactions before the QoS mechanism is activated. |

## SYNC Mode

The SCB3\_MST00\_SYNC register is used to program the clock domain crossing in the interface blocks.

Figure 54-11: SCB3\_MST00\_SYNC Register Diagram

![Image](57_System_Crossbars_(SCB)_artifacts/image_000010_ca56d9c617daf81ceb3309388cd87d82ad7edb1a83bf10bd27a18f5e17763da7.png)

Table 54-15: SCB3\_MST00\_SYNC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | VALUE      | Sync Mode. The SCB3_MST00_SYNC.VALUE bit field is used to program the clock domain crossing in the interface blocks. |
| 2:0 (R/W)          | VALUE      | 0 Sync 1:1                                                                                                           |
| 2:0 (R/W)          | VALUE      | 1 Sync n:1                                                                                                           |
| 2:0 (R/W)          | VALUE      | 2 Sync 1:n                                                                                                           |
| 2:0 (R/W)          | VALUE      | 3 Sync m:n                                                                                                           |
| 2:0 (R/W)          | VALUE      | 4 Async                                                                                                              |
| 2:0 (R/W)          | VALUE      | 5 Reserved                                                                                                           |
| 2:0 (R/W)          | VALUE      | 6 Reserved                                                                                                           |
| 2:0 (R/W)          | VALUE      | 7 Reserved                                                                                                           |