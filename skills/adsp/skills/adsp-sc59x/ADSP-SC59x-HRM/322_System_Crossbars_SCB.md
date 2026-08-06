# System Crossbars (SCB)

<!-- source: 322_System_Crossbars_SCB.pdf | original pages 3630–3856 -->

## 48   System Crossbars (SCB)

A modern system on a chip (SoCs) contains multi-cores, memory controllers, security blocks, and other high speed peripherals. As system integration increases, SoCs need to provide bus connectivity that allows better throughput to reduce performance bottlenecks. While traditional point-to-point connection buses have performed well in smaller systems, there is a need to use advanced switching based bus architectures for efficient handling of data transfer between multiplicity of data sources and sinks in the system. Additionally, mixing various traffic types in the same SoC (for example control, communication over peripherals and computing) while sharing the same bus resources, create different requirements from the Quality of Service (QoS) perspective.

The system crossbars (SCB) are the fundamental building blocks of a switch-fabric style for (on-chip) system bus interconnection. The SCBs connect system bus requesters to system bus completers, providing concurrent data transfer between multiple bus requesters and multiple bus completers. The SCB architecture addresses the challenges described above. The SCB provides sustainable throughput for simultaneous transactions in the system with configurable quality of service for each type of transaction (traffic) as required. A hierarchical model, built from multiple SCBs, provides a power and area efficient system interconnect, which satisfies the performance and flexibility requirements of a specific system.

## SCB Features

The SCBs provide the following features:

- Efficient, pipelined bus transfer protocol for sustained throughput
- Full-duplex bus operation for flexibility and reduced latency
- Concurrent bus transfer support to allow multiple bus requesters to access bus completers simultaneously
- Simple priority-based QoS based arbitration model
- Programmable quality of service

## SCB Functional Description

The following sections provide a functional description of the SCB.

- SCB Architectural Concepts

## SCB Architectural Concepts

This section describes the components of the SCB and the modules connected to it. The basic elements in the SCB are SCB requesters, completers, requester interfaces, and completer interfaces.

## Requesters

The system bus requesters include the peripheral Direct Memory Access (DMA) channels. These include the Serial Port (SPORT) DMA, SPI DMAs, Memory-to-Memory DMA channels (MDMA), the L1 code fill block, and the processor cores.

## Completers

Completers are SCB connections that are responding to transfer requests. Completers include MMR registers, memory units, and various peripherals depending upon individual configurations. Each system completer has its own latencies and response times.

## SCB Block Diagram

The SCB architectural model is shown in the SCB Overview figure. This figure shows a high level representation of a basic SCB connecting n completers to x requesters. A variable number of requesters connect to a variable number of completers in each SCB. In this example, all SIs connect to all MIs as indicated by the lines connecting them.

Figure 48-1: SCB Overview

<!-- image -->

## Hierarchy Block Diagram

A system interconnect built from multiple SCBs in a hierarchical model is illustrated in the SCB Hierarchy Overview figure. The system requester node level SCBs requester connects multiple SIs to a single MI, which in turn connects to an SI of the system completer level node SCB.

As discussed above, all the requesters in the system are distributed across different SCBs. A given SCB at system requester node level connects directly to the system requesters. These SCBs connect to SCB0 through its SIs forming a hierarchal structure. While a requester must access any completer, its first access goes through the SCB it is connected to, and then through SCB0, to reach its intended completer. This simplifies the connecting hardware in the basic SCB block by limiting the requesters. Care must be taken when sharing requesters to allow adequate throughput for their individual data transfer requirements.

In this example, all SIs are connected to all MIs.

Figure 48-2: SCB Hierarchy Overview

## NOTE:

## SCB Block Diagram

The SCB Block Diagram shows the functional blocks of the SCB module.

<!-- image -->

Figure 48-3: SCB Block Diagram

<!-- image -->

Figure 48-4: SCB Interconnections

<!-- image -->

The following are important points related to the system fabric on the ADSP-SC59x processors.

- The hierarchy of SCBs manages the system bus interconnections, multiplexing, and arbitration among the peripherals on the processor.

- The SCBs connections support DMA channels for some peripherals and support dedicated connections for others. The connections also support memory-mapped register access for internal memory (L1 and L2) and for external memory.

The completer interface of the crossbar where requesters such as DMA connect to performs two functions. The first function is arbitration. SCBx handles arbitration. The second function is clock conversion. The programmable QoS registers can be viewed as being associated with the SCBx.

- Most of the peripherals and their SCBs are in the SCLK0 domain. MDMA0-7, EMDMA0/1, Crypto, FIR/IIR accelerators, MLB, DBG, ETR, SHARC+ and Arm cores and their SCBs are in the SYSCLK domain. The link port is in the CLK04 domain. SPI is in the CLK06 domain.
- Each peripheral has a latency for access across the SCB. The latency varies with the nature of the peripheral. Also, the number of active peripherals (especially for cases where multiple peripherals are active on a shared SCB) affects SCB performance. Refer to the L2 System Memory chapter for details on the L2 memory organization.

MDMA3, MDMA4, MDMA6 and MDMA7 channels are unidirectional. For example, MSMDMA0 CH0 is read-only and MSMDMA0 CH1 is write-only. Access to the MMR space of SCB0 is allowed only in secure mode. The MMR space of SCB0 has registers for programming the QoS of various requesters and controlling the clock domain crossing.

- Access to the GPV (Global Programmers View) space is allowed only in secure mode. This space has registers for programming the QoS and CDC relationships and the remapping of various requesters.
- The S2 completer port of SHARCs is intended primarily for MDMA between the L1 of the 2 SHARCs. Hence, only the HSMDMA0 (MDMA3) and HSMDMA1 (MDMA7) is given access to S2 ports while all other requesters are given access to S1 port. This arrangement also means that S1 and S2 port share the same address space.
- SPI2/OSPI0 is low bandwidth completer - SCLK0 with a 32-bit interface. Only MDMA0/1/2/4/5/6 is used to perform MDMA to these completer nodes. HSMDMA0/1 (MDMA4 and 7) are intended for L1, L2 and DDR (these completers have the same bandwidth capability as HSMDMA0/1).
- SPIF is a read-only flash. Peripherals are not able to read out of the SPI flash directly. Only MDMAs, crypto, cores, and debug tools are given access to it.
- LP to SPI2/OSPI0 access is allowed.
- The SPIF completer in the system fabric has remapping options in the SPIF fabric for SPI2 and OSPI0 in the same address space. The address map configuration is programmable using remap registers.

IMPORTANT: The following points are important changes from ADSP-2156x processors:

- All peripherals (including debugger) access L2 memory through the new DL2\_1 subordinate port on the ADSP-SC59x processor. On the ADSP-2156x processor, these accesses were on the DL2\_0 port of L2.

- To reduce the probability of port conflict in a multicore system, the cores are connected on the CL2\_0 and CL2\_1 ports of L2. CL2\_0 has access only to the first 1 MB of L2. CL2\_1 has access to second 1 MB of L2.
- Booting of the Arm core occurs through the CL2\_0 port; booting of the SHARC cores occurs through CL2\_1 (to avoid any conflict during multi-core boot). The respective BOOTROM addresses have been routed accordingly.

The following acronyms are used in the SCB Interconnections figure.

## SCB0-10

Indicate SCB interfaces, connecting the system bus requesters and completers

## SCLK0, SYSCLK, CLK03, CLK04, CLK06

Indicate clock domains in which the specific SCBs operate. For more information on clock domains, see the Clock Generation Unit (CGU) chapter and the product data sheet.

## CDC

Indicates the clock domain crossing

## CL2\_0, CL2\_1, CL2\_2

Indicates a 128-bit L2 completer

## DL2\_0, DL2\_1

Indicates a 128-bit L2 completer

## L1C1\_S1

Indicates the S1 completer of the SHARC1 processor

## L1C1\_S2

Indicates the S2 completer of the SHARC1 processor

## SHARC1\_IPORT

Indicates the SHARC1 processor instruction requester port

## SHARC1\_MMR

Indicates the SHARC1 MMR interface

## A5\_L2CC\_M0/1

Indicates the Cortex A55 requesters (0 and 1)

## Arm Completer

Indicates the Cortex A55 completer

## SHARC Fabric with FIR/IIR Accelerator

The accelerator block on each SHARC core of ADSP-SC59x processor has one FIR and four IIR accelerators. All instances of the FIR and IIR accelerators operate at the core clock (CCLK) frequency. Accelerator requester ports can directly access the respective SHARC L1 memory with reduced latency. The access does not go through the system fabric. The SHARC core can also directly access the MMR registers of its respective accelerator. The accesses between one SHARC core and the accelerators that belong to the other SHARC core still go through the main system fabric.

The SHARC Fabric Connectivity figure shows a block diagram of the SHARC fabric with FIR/IIR accelerators integrated closely with the SHARC core.

Figure 48-5: SHARC Fabric Connectivity

<!-- image -->

## Note the following:

- The SHARC+ core can directly access the MMR space using its accelerators.

- The MMR port of the SHARC+ core is separate from the data port (DPORT). It is a separate requester port for the main system fabric.
- PFB (prefetch buffer) - after the MMR split, the DPORT comes into the SHARC+ core for the PFB connection and returns to the SHARC+ fabric again for the clock domain crossing.
- Accelerators share the same security level as the respective core. The SHARC+ core associated with the accelerator can always access the MMR space of the accelerator. Accesses from the other requesters such as other SHARC+ core/Arm Core/MSMDMA/DBG are allowed provided that the security requirements of the requester are met.

To ensure complete bandwidth utilization and optimal performance of the L2 ports, the controllers are allocated as shown in L2 Port Requester Allocation table.

Table 48-1: L2Port Requester Allocation

| Requesters      | CL2_0   | CL2_1   | CL2_2   | DL2_0   | DL2_1   |
|-----------------|---------|---------|---------|---------|---------|
| SHARC0 (Core 1) | ✓       | ✓       |         |         |         |
| SHARC1 (Core 2) | ✓       | ✓       |         |         |         |
| A55 (Core 0)    | ✓       | ✓       |         |         |         |
| SHARC0 IPORT    |         |         | ✓       |         |         |
| SHARC1 IPORT    |         |         | ✓       |         |         |
| SHARC0_FIR_CH0  |         |         | ✓       |         |         |
| SHARC0_FIR_CH1  |         |         | ✓       |         |         |
| SHARC0_IIR_CH0  | ✓       | ✓       |         |         |         |
| SHARC0_IIR_CH1  | ✓       | ✓       |         |         |         |
| SHARC1_FIR_CH0  |         |         | ✓       |         |         |
| SHARC1_FIR_CH1  |         |         | ✓       |         |         |
| SHARC1_IIR_CH1  | ✓       | ✓       |         |         |         |
| SHARC1_IIR_CH1  | ✓       | ✓       |         |         |         |
| HSMDMA0         |         |         |         | ✓       |         |
| HSMDMA0         |         |         |         | ✓       |         |
| Peripherals     |         |         |         |         | ✓       |

## Note the following:

- To reduce the probability of port conflict in a multicore system, the cores are connected on the CL2\_0 and CL2\_1 ports of L2. CL2\_0 has access only to the first 1 MB of L2. CL2\_1 has access to second 1 MB of L2
- To avoid conflict during multi-core boot, the booting of the Arm core happens through the CL2\_0 port while the booting of the SHARC cores happens through the CL2\_1 port. The respective bootROM addresses are routed accordingly.

The addressable range for each of the ports is shown in the L2 Port Address Allocation table.

Table 48-2: L2 Port Address Allocation

|               | Memory Address Range   | Memory Address Range   | Memory Address Range   | Memory Address Range   |
|---------------|------------------------|------------------------|------------------------|------------------------|
| SMPU Instance | Data                   | Data                   | Instruction            | Instruction            |
|               | Start Address          | End Address            | Start Address          | End Address            |
| CL2_0         | 0x2000_0000            | 0x200F_FFFF            | 0x2020_0000            | 0x2020_FFFF            |
| DL2_0         | 0x2000_0000            | 0x2022_FFFF            |                        |                        |
| CL2_1         | 0x2010_0000            | 0x201F_FFFF            | 0x2021_0000            | 0x2022_FFFF            |
| CL2_2         | 0x2000_0000            | 0x2022_FFFF            |                        |                        |
| DL2_1         | 0x2000_0000            | 0x2022_FFFF            |                        |                        |

To support exclusive accesses to the 5 port L2 memory on the processor, the configuration of SMPUs as shown in the SMPU Access Scheme for L2 Ports figure is used.

Figure 48-6: SMPU Access Scheme for L2 Ports

<!-- image -->

Table 48-3: SCB Controlled DMA Channel Peripherals

|      | Requesters     | DDE DMAChannels   | Non DDE DMAChannels   |
|------|----------------|-------------------|-----------------------|
| SCB1 | SPORT0, HALF A | DMA0              |                       |
|      | SPORT0, HALF B | DMA1              |                       |

Table 48-3: SCB Controlled DMA Channel Peripherals (Continued)

|      | Requesters     | DDE DMAChannels   | Non DDE DMAChannels   |
|------|----------------|-------------------|-----------------------|
|      | SPORT1, HALF A | DMA2              |                       |
|      | SPORT1, HALF B | DMA3              |                       |
|      | SPORT2, HALF A | DMA4              |                       |
|      | SPORT2, HALF B | DMA5              |                       |
|      | SPORT3, HALF A | DMA6              |                       |
|      | SPORT3, HALF B | DMA7              |                       |
| SCB2 | SPORT4, HALF A | DMA10             |                       |
|      | SPORT4, HALF B | DMA11             |                       |
|      | SPORT5, HALF A | DMA12             |                       |
|      | SPORT5, HALF B | DMA13             |                       |
|      | SPORT6, HALF A | DMA14             |                       |
|      | SPORT6, HALF B | DMA15             |                       |
|      | SPORT7, HALF A | DMA16             |                       |
|      | SPORT7, HALF B | DMA17             |                       |
| SCB3 | UART0, TX      | DMA20             |                       |
|      | UART0, RX      | DMA21             |                       |
|      | UART1, TX      | DMA34             |                       |
|      | UART1, RX      | DMA35             |                       |
|      | UART2, TX      | DMA37             |                       |
|      | UART2, RX      | DMA38             |                       |
|      | UART3, TX      | DMA53             |                       |
|      | UART3, RX      | DMA54             |                       |
|      | PPI F0         | DMA28             |                       |
|      | PPI F1         | DMA29             |                       |
| SCB4 | SPI0, TX       | DMA22             |                       |
|      | SPI0, RX       | DMA23             |                       |
|      | SPI1, TX       | DMA24             |                       |
|      | SPI1, RX       | DMA25             |                       |
|      | SPI2, TX       | DMA26             |                       |
|      | SPI2, RX       | DMA27             |                       |
|      | SPI3, TX       | DMA55             |                       |

Table 48-3: SCB Controlled DMA Channel Peripherals (Continued)

|               | Requesters              | DDE DMAChannels   | Non DDE DMAChannels                     |
|---------------|-------------------------|-------------------|-----------------------------------------|
|               | SPI3, RX                | DMA56             |                                         |
| SCB5          | 10/100/1000 EMAC (GIGE) | N/A               | 6 channels (3 receive and 3 trans- mit) |
| SCB5          | 10/100 EMAC             | N/A               | 2 channels (1 receive and 1 trans- mit) |
| SCB6          | LP0                     | DMA30             |                                         |
| SCB6          | LP1                     | DMA36             |                                         |
| SCB7          | CRC0/MDMA0              | DMA8, DMA9        |                                         |
| SCB7          | CRC1/MDMA1              | DMA18, DMA19      |                                         |
| SCB8          | CRC2/MDMA4              | DMA45, DMA46      |                                         |
| SCB8          | CRC3/MDMA5              | DMA47, DMA48      |                                         |
| SCB9          | MDMA2                   | DMA39, DMA40      |                                         |
| SCB9          | MDMA6                   | DMA49, DMA50      |                                         |
| SCB10         | ETR                     | N/A               | N/A                                     |
| SCB10         | CRYPTO                  | N/A               | 2 channels ( 1 read, 1 write)           |
| SCB10         | EMDMA0                  | N/A               | 2 channels ( 1 read, 1 write)           |
| SCB10         | EMDMA1                  | N/A               | 2 channels ( 1 read, 1 write)           |
| HSMDMA_SCB    | MDMA3                   | DMA43, DMA44      |                                         |
| HSMDMA_SCB    | MDMA7                   | DMA51, DMA52      |                                         |
| CORES_SCB     | SHARC0 DPORT            | N/A               | N/A                                     |
| CORES_SCB     | SHARC1 DPORT            | N/A               | N/A                                     |
| CORES_SCB     | ARM_L2CC_M0             | N/A               | N/A                                     |
| CORES_SCB     | ARM_L2CC_M1             | N/A               | N/A                                     |
| CORES_SCB     | SHARC0 IIR CH0, CH1     | N/A               | N/A                                     |
| CORES_SCB     | SHARC1 IIR CH0, CH1     | N/A               | N/A                                     |
| FIR_IPORT_SCB | SHARC0 FIR CH0, CH1     | N/A               | N/A                                     |
| FIR_IPORT_SCB | SHARC1 FIR CH0, CH1     | N/A               | N/A                                     |
| FIR_IPORT_SCB | SHARC0 IPORT            | N/A               | N/A                                     |
| FIR_IPORT_SCB | SHARC1 IPORT            | N/A               | N/A                                     |

Table 48-3: SCB Controlled DMA Channel Peripherals (Continued)

|         | Requesters   | DDE DMAChannels   | Non DDE DMAChannels          |
|---------|--------------|-------------------|------------------------------|
| MMR_SCB | SHARC0MMR    | N/A               | N/A                          |
| MMR_SCB | SHARC1MMR    | N/A               | N/A                          |
| MMR_SCB | A55MMR       | N/A               | N/A                          |
| MMR_SCB | USB0         | N/A               | 16 channels (8 IN and 8 OUT) |
| MMR_SCB | eMSI         | N/A               | 1 Tx/Rx DMAchannel           |

There are two types of peripherals that use DMA. The first have dedicated DMA channels controlled by the Dedicated DMA Engine (DDE) and have the same operating modes (see DMA Operating Modes) and use the same programming model (DMA Channel Programming Model). The second type is not controlled by the DDE module. These peripherals have their own operating modes and programming models (see the peripheral chapter for this information). The peripheral types are shown in the SCB-Controlled DMA Channel Peripherals table.

## System Crossbars

The System Crossbars (SCB) are the fundamental building blocks of the system bus interconnect. The SCB (often referred to as the system interconnect fabric), is a collection of inter-connection units connecting system requesters to completer memory spaces. The SCB connects one or more requester devices to one or more memory-mapped completer devices. Each connected requester can be a core that originates an SCB transaction, or a requester interface of an upstream SCB cascaded interconnect. Each connected completer can be the final completer of an SCB transaction or a completer interface of a downstream cascaded SCB interconnect (forming a hierarchy of SCBs).

Each SCB that has multiple requesters and completers share the total bandwidth of the SCB. (In a M:N configuration where M requesters are connected to N completers through the SCBx.)

The SCB provides separate channels for reads and writes. Read and write accesses through a given SCB do not share bandwidth. Only SCB0, which is the major SCB in the SCB hierarchy, has the multiple paths between multiple requester and completer interfaces.

All other SCBs in the chip connect to SCB0 through different completer interfaces. Other primary requesters (DMAs, cores, and so on) in the system are distributed across these small SCBs. For a given SCB, all the requester and completers share the total bandwidth of the SCB. (Only SCB0 is the exception). Since different DMA channels are scattered across different SCBs (SCB1, SCB2 SCB3, and so on), they do not conflict for the bandwidth when they are in different SCBs and accessing different completers. SCB0 allows for concurrent data transfer between multiple bus requesters and multiple bus completers, providing flexibility, and full-duplex operation.

If system accesses are carefully architected, SCB has a potential of providing sufficient sustained bandwidth in the end system.

Since the SCBs support burst transfers, it is important to configure the requesting requester appropriately to make best use of available SCB bandwidth. For a DMA requester, choosing the appropriate DMA\_CFG.MSIZE value, is important from both a functional and a performance perspective. The value in the DMA\_CFG.PSIZE bit field determines the width of the peripheral bus in use. It can be configured to 1-byte, 2-bytes, or 4-bytes. The

DMA\_CFG.MSIZE value determines the actual size of the SCB bus in use. It also determines the minimum number of bytes which are transferred from or to memory corresponding to a single DMA request or grant. The transfer can be 1-, 2-, 4-, 8-, 16-, or 32-bytes. If the DMA\_CFG.MSIZE value is greater than the SCB bus width, the SCB performs burst transfers according to the width defined in DMA\_CFG.MSIZE . When DMA\_CFG.MSIZE is less than the SCB bus width, bursting is not supported and partial bus use results.

Each of the SCB unit in the fabric consists of N completer interfaces (MSTn). Each of these interfaces has controls for read quality of service, write quality of service, and functional mode. A subset of these matrices includes controls for IB (Interface Block) sync mode, and bus functional mode. For more details on IB, see the clock domain synchronization section.

## SCB Bus Requester IDs

The SCB Bus Requester IDs table indicates which requesters are connected to each of the completer ports of SCB0. The tables also indicate the precise value of the ID as seen by the completer. These values are useful for SWU programming.

NOTE: For an overall diagram of all SCB interconnections, see the SCB Block Diagram.

Table 48-4: Bus Requester IDs

| Requester                  | Hex ID Values   | Binary Values     |
|----------------------------|-----------------|-------------------|
| DMA0 (SPORT0, HALF A)      | 0x0000, 0x0100  | 13'b0000x00000000 |
| DMA1 (SPORT0, HALF B)      | 0x0010, 0x0110  | 13'b0000x00010000 |
| DMA2 (SPORT1, HALF A)      | 0x0020, 0x0120  | 13'b0000x00100000 |
| DMA3 (SPORT1, HALF B)      | 0x0030, 0x0130  | 13'b0000x00110000 |
| DMA4 (SPORT2, HALF A)      | 0x0040, 0x0140  | 13'b0000x01000000 |
| DMA5 (SPORT2, HALF B)      | 0x0050, 0x0150  | 13'b0000x01010000 |
| DMA6 (SPORT3, HALF A)      | 0x0060, 0x0160  | 13'b0000x01100000 |
| DMA7 (SPORT3, HALF B)      | 0x0070, 0x0170  | 13'b0000x01110000 |
| DMA8 (Enh BWMDMA0CRC, CH0) | 0x0031, 0x0131  | 13'b0000x00110001 |
| DMA9 (Enh BWMDMA0CRC, CH1) | 0x0021, 0x0121  | 13'b0000x00100001 |
| MLB                        | 0x0042          | 13'b0000001000010 |
| DMA20 (UART0, TX)          | 0x0003, 0x0103  | 13'b0000x00000011 |
| DMA21 (UART0, RX)          | 0x0043, 0x0143  | 13'b0000x01000011 |
| DMA37 (UART2, TX)          | 0x0033, 0x0133  | 13'b0000x00110011 |
| DMA22 (SPI0, TX)           | 0x0004, 0x0104  | 13'b0000x00000100 |
| DMA23 (SPI0, RX)           | 0x0014, 0x0114  | 13'b0000x00010100 |
| DMA24 (SPI1, TX)           | 0x0024, 0x0124  | 13'b0000x00100100 |
| DMA25 (SPI1, RX)           | 0x0034, 0x0134  | 13'b0000x00110100 |

Table 48-4: Bus Requester IDs (Continued)

| Requester                       | Hex ID Values         | Binary Values     |
|---------------------------------|-----------------------|-------------------|
| DMA26 (SPI2, TX)                | 0x0054, 0x0154        | 13'b0000x01010100 |
| DMA27 (SPI2, RX)                | 0x0044, 0x0144        | 13'b0000x01000100 |
| DMA30 (LP0)                     | 0x0005, 0x005         | 13'b0000x00000101 |
| DMA34 (UART1, TX)               | 0x0013, 0x0113        | 13'b0000x00010011 |
| DMA35 (UART1, RX)               | 0x0023, 0x0123        | 13'b0000x00100011 |
| DMA36 (LP1)                     | 0x0015, 0x0115        | 13'b0000x00010101 |
| CRYPTO                          | 0x0056, 0x0156        | 13'b0000001010110 |
| SH0_FIR_CH0                     | 0x0X07 (X=don't care) | 13'b0xxxx00000111 |
| SH0_FIR_CH1                     | 0x0X17 (X=don't care) | 13'b0xxxx00010111 |
| EMDMA0 (CH0)                    | 0x0006                | 13'b0000000000110 |
| EMDMA0 (CH1)                    | 0x0016                | 13'b0000000010110 |
| EMDMA1 (CH0)                    | 0x0026                | 13'b0000000100110 |
| EMDMA1 (CH1)                    | 0x0036                | 13'b0000000110110 |
| DMA39 (Enh BWMDMA2, CH0)        | 0x0008, 0x0108        | 13'b0000x00001000 |
| DMA40 (Enh BWMDMA2, CH1)        | 0x0018, 0x0118        | 13'b0000x00011000 |
| DBG                             | 0x0048                | 13'b0000001001000 |
| ETR                             | 0x0046                | 13'b0000001000110 |
| DMA18 (Enh BWMDMA1CRC1, CH0)    | 0x0011, 0x0111        | 13'b0000x00010001 |
| DMA19 (Enh BWMDMA1CRC1, CH1)    | 0x0001, 0x0101        | 13'b0000x00000001 |
| DMA38 (UART2, RX)               | 0x0053, 0x0153        | 13'b0000x01010011 |
| SH0 (DPORT)                     | 0x0X09(X=don't care)  | 13'b0xxxx00001001 |
| SH0 (IPORT)                     | 0x0047                | 13'b0000001000111 |
| DMA43 (High Speed MDMA3, CH0)   | 0x000A, 0x010A        | 13'b0000x00001010 |
| DMA44 (High Speed BWMDMA3, CH1) | 0x001A, 0x011A        | 13'b0000x00011010 |
| DMA10 (SPORT4, HALFA)           | 0x000B, 0x010B        | 13'b0000x00001011 |
| DMA11 (SPORT4, HALFB)           | 0x001B, 0x011B        | 13'b0000x00011011 |
| DMA12 (SPORT5, HALFA)           | 0x002B, 0x012B        | 13'b0000x00101011 |
| DMA13 (SPORT5, HALFB)           | 0x003B, 0x013B        | 13'b0000x00111011 |
| DMA14 (SPORT6, HALFA)           | 0x004B, 0x014B        | 13'b0000x01001011 |
| DMA15 (SPORT6, HALFB)           | 0x005B, 0x015B        | 13'b0000x01011011 |
| DMA16 (SPORT7, HALFA)           | 0x006B, 0x016B        | 13'b0000x01101011 |

Table 48-4: Bus Requester IDs (Continued)

| Requester                       | Hex ID Values         | Binary Values     |
|---------------------------------|-----------------------|-------------------|
| DMA17 (SPORT7, HALFB)           | 0x007B, 0x017B        | 13'b0000x01111011 |
| SH0_MMR                         | 0x0X0C (X=don't care) | 13'b0xxxx00001100 |
| DMA53 (UART3, TX)               | 0x0063, 0x0163        | 13'b0000x01100011 |
| DMA54 (UART3, RX)               | 0x0073, 0x0173        | 13'b0000x01110011 |
| DMA55 (SPI3, TX)                | 0x0064, 0x0164        | 13'b0000x01100100 |
| DMA56 (SPI3, RX)                | 0x0074, 0x0174        | 13'b0000x01110100 |
| DMA45 (Enh BWMDMA4CRC, CH0)     | 0x001D, 0x011D        | 13'b0000x00011101 |
| DMA46 (Enh BWMDMA4CRC, CH1)     | 0x000D, 0x010D        | 13'b0000x00001101 |
| DMA47 (Enh BWMDMA5CRC, CH0)     | 0x003D, 0x013D        | 13'b0000x00111101 |
| DMA48 (Enh BWMDMA5CRC, CH1)     | 0x002D, 0x012D        | 13'b0000x00101101 |
| DMA49 (Enh BWMDMA6CRC1, CH0)    | 0x0038, 0x0138        | 13'b0000x00111000 |
| DMA50 (Enh BWMDMA6CRC1, CH1)    | 0x0028, 0x0128        | 13'b0000x00101000 |
| DMA51 (High Speed MDMA3, CH0)   | 0x002A, 0x012A        | 13'b0000x00101010 |
| DMA52 (High Speed BWMDMA3, CH1) | 0x003A, 0x013A        | 13'b0000x00111010 |
| DMA28 (PPI F0)                  | 0x0083, 0x0183        | 13'b0000x10000011 |
| DMA29 (PPI F1)                  | 0x0093, 0x0193        | 13'b0000x10010011 |
| USB0                            | 0x002E                | 13'b0000000101110 |
| SH1 (IPORT)                     | 0x0057                | 13'b0000001010111 |
| SH1 (DPORT)                     | 0x0X19(X=don't care)  | 13'b0xxxx00011001 |
| SH1_MMR                         | 0x0X2C(X=don't care)  | 13'b0xxxx00101100 |
| ARM_L2CC_M0                     | 0x0X29(X=don't care)  | 13'bxxxxx00101001 |
| ARM_L2CC_M1                     | 0x0X39(X=don't care)  | 13'bxxxxx00111001 |
| GIGE                            | 0x0X0F(X=don't care)  | 13'b0xxxx00001111 |
| EMAC                            | 0x0X1F(X=don't care)  | 13'b0xxxx00011111 |
| SH1_FIR_CH0                     | 0x0X27(X=don't care)  | 13'b0xxxx00100111 |
| SH1_FIR_CH1                     | 0x0X37(X=don't care)  | 13'b0xxxx00110111 |
| SH0_IIR_CH0                     | 0x0X49(X=don't care)  | 13'b0xxxx01001001 |
| SH0_IIR_CH1                     | 0x0X59(X=don't care)  | 13'b0xxxx01011001 |
| SH1_IIR_CH0                     | 0x0X69(X=don't care)  | 13'b0xxxx01101001 |
| SH1_IIR_CH1                     | 0x0X79(X=don't care)  | 13'b0xxxx01111001 |

## SCB Programming Model

The following sections provide information for the correct programming of the SCB. The SCB in the ADSP-SC59x processors is based on the ARM CoreLink NIC-400 Network Interconnect. This chapter should be used in conjunction with the ARM CoreLink NIC-400 Network Interconnect Technical Reference Manual. The ARM manual provides more information for programming in the SCB interface.

## Programming SCB Arbitration

Each completer interface has a QoS value (priority) associated with both read and write channels. These values are 4 bits present in the SCB0\_MSTx\_RQOS and SCB0\_MSTx\_WQOS registers. At the entry point to the infrastructure, all transactions are allocated this programmable local QoS value. The arbitration of the transaction throughout the infrastructure uses this QoS. At any arbitration node, a fixed priority exists for transactions with a different QoS. The highest value has the highest priority.

If there are coincident transactions at an arbitration node with the same QoS that require arbitration, then the network uses a Least Recently Granted (LRG) algorithm. At each switch, the requester with the highest QoS acquires access and that switch output takes the QoS value of the winner for that transaction. At the next switch completer interface, the requester uses the QoS value of the winner. QoS can have values from 0 (lowest priority) to 15 (highest priority).

For example in the following figure, SCB Arbitration :

1. At SCB1, requesters (1, 2, 3) have RQOS values of (6, 4, 2)
2. At SCB2, requesters (4, 5, 6) have RQOS values of (12, 13, 1)

Figure 48-7: SCB Arbitration

<!-- image -->

In this case, requester 1 wins at SCB1, and requester 5 wins at SCB2. However, in a perfect competition at SCB0, requesters 4 and 5 had the highest overall RQOS values. requesters 4 and 5 would have fought for arbitration directly at SCB0. However, because of the mini*SCBs, requester 1, at a much lower RQOS value, is able to win against requester 4 and make it all the way to SCB0.

## Programming Clock Domain Crossing Registers

The Clock Domain Crossing Options table shows the various clock domain crossings that are available on the ADSPSC59x processor. The clocking relationships are defined with respect to SYSCLK. The following clock domains are programmable: CLK02:SYSCLK, CLKO3: SYSCLK, CLKO4: SYSCLK and CLKO8: SYSCLK CDC. The registers to configure the CDC mode are present within the GPV space of the respective fabric. For example: To program

the CLKO3: SYSCLK relationship, program the GPV registers within the DMC\_CDC fabric and the MMRG fabric.

Table 48-5: Clock Domain Crossing Options

|                   |                                            | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   |
|-------------------|--------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|
| Functional Clocks | Possible Com- pleters                      | System Fabric                                   | MMRGFabric                                      | MCFabric                                        | SHARC Fabric                                    | Arm Fabric                                      | SPIF Fabric                                     |
| SYSCLK            | System Fabric and Infrastruc- ture Modules | 1:1                                             | 1:1                                             | *1                                              | *                                               | *                                               | *                                               |
| SCLK0             | SCLK0 Clock Doamin                         | Synchronous (m:1)                               | Synchronous (m:1)                               | *                                               | *                                               | *                                               | *                                               |
| CLKO0             | SHARC0 and its Accelerators                | *                                               | *                                               | *                                               | Synchronous (m:1)                               | *                                               | *                                               |
| CLKO1             | SHARC1 and its Accelerators                | *                                               | *                                               | *                                               | Synchronous (m:1)                               | *                                               | *                                               |
| CLKO2             | ARM                                        | *                                               | *                                               | *                                               | *                                               | Programmable                                    | *                                               |
| CLKO3             | DMC                                        | *                                               | Programmable                                    | Programmable                                    | *                                               | *                                               | *                                               |
| CLKO4             | CAN                                        | *                                               | Programmable                                    | *                                               | *                                               | *                                               | *                                               |
| CLKO6             | SPI                                        | Synchronous (m:n) (except SPIF complet- er)     | Synchronous (m:n)                               | *                                               | *                                               | *                                               | Synchronous (m:n)                               |
| CLKO8             | LP                                         | Programmable                                    | Programmable                                    | *                                               | *                                               | *                                               | *                                               |

*1 * Either the interface is not present or CDC is not required

NOTE: Consider the following points in the Clock Domain Crossing Options table:

- For DMC fabric, the sync mode register should be programmed for m:n or async (default). Only for sysclk:CLKO3 = 500:500MHz, sync mode can be 1:1.
- For system MMRG fabric - CLKO8/LP sync mode register should be programmed for m:n or async (default).

Table 48-6: Sync Mode Bit Field Description

|   Sync Mode | Description   |
|-------------|---------------|
|           0 | sync 1:1      |
|           1 | sync n:1      |
|           2 | sync 1:n      |
|           3 | sync m:n      |

Table 48-6: Sync Mode Bit Field Description (Continued)

|   Sync Mode | Description   |
|-------------|---------------|
|           4 | async         |

To change the clock domain crossing mode, follow the actions described in the Changing Clock Domain Crossing Modes table.

Table 48-7: Changing Clock Domain Crossing Modes

| Original Mode   | Required Mode   | Action                                               |
|-----------------|-----------------|------------------------------------------------------|
| ASYNC           | Any other mode  | Change the clocks, then change the MMRregister       |
| Any mode        | ASYNC           | Change the MMRregister, then change clocks to ASYNC. |
| m:n             | 1:1             | Change the clocks, then change the register.         |
| 1:1             | m:n             | Change the register, then change the clocks.         |

## SCB Programming Concepts

The SCB arbitration model among requester or completer SCBs of the processor is fixed (not programmable). But, each completer does have a quality of service (QoS) programmable feature that affects arbitration.

The arbitration of transactions in SCB is based on the QoS value or the priority of the transaction. All requesters with the same priority form a priority group. Arbitration is granted to the highest priority group from which a member is trying to win access, and within that group, to the highest requester at that time. When a requester wins arbitration, it is relegated to the bottom of its group to ensure that it cannot prevent other requesters in its group from accessing the completer.

If you configure all requester priorities to different levels, the arbiter implements a fixed priority scheme. This scheme occurs because each requester is in a group of its own, and therefore, requesters maintain their ordering.

The LRG and fixed priority modes concurrently exist when the requester priority value registers are programmed with a combination of identical and unique values.

NOTE: The SCB arbitration hierarchy is fixed (for example, SCB1 requester to SCB1 completer). However, multiple requester inputs to the same completer permit QoS programming.

The LRG Arbitration Example figure shows three groups with different QoS values. Requesters in the same group share a QoS value. The arbitration occurs using an LRG scheme.

Figure 48-8: LRG Arbitration Example

<!-- image -->

The QoS value assigned to a transaction at entry point is carried forward by the transaction as it passes through all arbitration stages in the SCB. QoS for all requesters is configured as programmable in the system fabric interconnect.

The priority of the requesters fits into three groups:

- Group A - Peripherals with an external interface, without flow control and with real-time processing requirements
- Group B - Cores, peripherals with flow control, and offload engines
- Group C - MDMAs

## Group A: Peripherals with external interface, without flow control and with real-time processing requirements

These requesters are assigned the highest priority. They are latency-critical - a large increase in latency could potentially result in data corruption and catastrophic failure. This high priority SCB group includes requesters connected to SCB1, SCB2, SCB3, and SCB4. Requesters in this group are assigned the QOS reset value of 12.

## Group B: Cores, peripherals with flow control, and offload engines

The core is latency sensitive. Once a core requests data, it typically waits for the data without performing anything else in parallel. An increase in the latency has a direct impact on the performance of the core. This group is assigned medium priority and a QOS reset value of seven. Group B requesters are connected to SCB5, SCB6, CORES\_SCB, FIR\_IPORT\_SCB, and MMR\_SCB.

## Group C: MDMAs

These requesters are assigned the lowest priority. MDMAs are data intensive and do not have any external interface. Requesters in this group are assigned the QOS reset value of 1. The Group C requesters are connected to SCB7, SCB8, SCB9, and SCB10.

Table 48-8: QoS Register Table

|   Requester ID | Requester    |   read_qos Reset Value |   write_qos Reset Value |
|----------------|--------------|------------------------|-------------------------|
|              1 | SPORT0_A_DMA |                     12 |                      12 |
|              2 | SPORT0_B_DMA |                     12 |                      12 |
|              3 | SPORT1_A_DMA |                     12 |                      12 |
|              4 | SPORT1_B_DMA |                     12 |                      12 |
|              5 | SPORT2_A_DMA |                     12 |                      12 |
|              6 | SPORT2_B_DMA |                     12 |                      12 |
|              7 | SPORT3_A_DMA |                     12 |                      12 |
|              8 | SPORT3_B_DMA |                     12 |                      12 |
|              9 | SPORT4_A_DMA |                     12 |                      12 |
|             10 | SPORT4_B_DMA |                     12 |                      12 |
|             11 | SPORT5_A_DMA |                     12 |                      12 |
|             12 | SPORT5_B_DMA |                     12 |                      12 |
|             13 | SPORT6_A_DMA |                     12 |                      12 |
|             14 | SPORT6_B_DMA |                     12 |                      12 |
|             15 | SPORT7_A_DMA |                     12 |                      12 |
|             16 | SPORT7_B_DMA |                     12 |                      12 |
|                | UART0_TX     |                     12 |                      12 |
|                | UART0_RX     |                     12 |                      12 |
|                | UART1_TX     |                     12 |                      12 |
|                | UART1_RX     |                     12 |                      12 |
|                | UART2_TX     |                     12 |                      12 |
|                | UART2_RX     |                     12 |                      12 |
|                | UART3_TX     |                     12 |                      12 |
|                | UART3_RX     |                     12 |                      12 |
|                | PPI_F0       |                     12 |                      12 |
|                | PPI_F1       |                     12 |                      12 |
|                | SPI0RX       |                     12 |                      12 |

Table 48-8: QoS Register Table (Continued)

|   Requester ID | Requester   |   read_qos Reset Value |   write_qos Reset Value |
|----------------|-------------|------------------------|-------------------------|
|                | SPI0TX      |                     12 |                      12 |
|             19 | SPI1TX      |                     12 |                      12 |
|             20 | SPI1RX      |                     12 |                      12 |
|             21 | SPI2TX      |                     12 |                      12 |
|             22 | SPI2RX      |                     12 |                      12 |
|             23 | MLB         |                     12 |                      12 |
|                | GIGE        |                      7 |                       7 |
|                | EMAC        |                      7 |                       7 |
|                | LP0         |                      7 |                       7 |
|                | LP1         |                      7 |                       7 |
|                | USB         |                      7 |                       7 |
|                | SDIO0       |                      7 |                       7 |
|                | SH0_DPORT   |                      7 |                       7 |
|                | SH0_IPORT   |                      7 |                       7 |
|                | SH1_IPORT   |                      7 |                       7 |
|                | ARM_L2CC_M0 |                      7 |                       7 |
|                | ARM_L2CC_M1 |                      7 |                       7 |
|                | SH0_IIR_CH0 |                      7 |                       7 |
|                | SH0_IIR_CH1 |                      7 |                       7 |
|                | SH1_IIR_CH0 |                      7 |                       7 |
|                | SH1_IIR_CH1 |                      7 |                       7 |
|                | SH0_FIR_CH0 |                      7 |                       7 |
|                | SH0_FIR_CH1 |                      7 |                       7 |
|                | SH0_MMR     |                      7 |                       7 |
|                | SH1_MMR     |                      7 |                       7 |
|                | PL310_MMR   |                      7 |                       7 |
|                | CRC0_CH0    |                      1 |                       1 |
|                | CRC0_CH1    |                      1 |                       1 |
|                | CRC1_CH0    |                      1 |                       1 |
|                | CRC1_CH1    |                      1 |                       1 |
|             42 | CRC2_CH0    |                      1 |                       1 |

Table 48-8: QoS Register Table (Continued)

|   Requester ID | Requester   |   read_qos Reset Value |   write_qos Reset Value |
|----------------|-------------|------------------------|-------------------------|
|             43 | CRC2_CH1    |                      1 |                       1 |
|                | CRC3_CH0    |                      1 |                       1 |
|                | CRC3_CH1    |                      1 |                       1 |
|                | MDMA2_CH0   |                      1 |                       1 |
|                | MDMA2_CH1   |                      1 |                       1 |
|                | MDMA3_CH0   |                      1 |                       1 |
|                | MDMA3_CH1   |                      1 |                       1 |
|                | MDMA6_CH0   |                      1 |                       1 |
|                | MDMA6_CH1   |                      1 |                       1 |
|                | MDMA7_CH0   |                      1 |                       1 |
|                | MDMA7_CH1   |                      1 |                       1 |
|                | EMDMA0_CH0  |                      1 |                       1 |
|                | EMDMA0_CH1  |                      1 |                       1 |
|                | EMDMA1_CH0  |                      1 |                       1 |
|                | EMDMA1_CH1  |                      1 |                       1 |
|                | CRYPTO      |                      1 |                       1 |
|                | ETR         |                      1 |                       1 |
|                | DBG         |                      1 |                       1 |

## QoS Programming

Adhere to the following guidelines if software is modifying the reset value of QoS:

- The highest QoS of any requester in a low priority SCB group must be less than the lowest QoS of any requester in a medium priority SCB group
- The highest QoS of any requester in medium priority SCB group must be less than the lowest QoS of any requester in high priority SCB group

Reset values of QoS have been spaced out to allow software to lower or higher the priority of a few requesters without having to modify the priority all the requesters. For example, the QoS of the LP can be reduced from 12 to 11 without violating the guidelines.

## SCB6 Programming

Single core A55 integration supports in-order transactions in SCB6. To support this feature, program SCB6\_A55\_M0\_IB\_FN\_MOD = 0x3 (2'b11).

## ADSP-SC59x SCB0 Register Descriptions

(SCB0) contains the following registers.

Table 48-9: ADSP-SC59x SCB0 Register List

| Name                         | Description                                        |
|------------------------------|----------------------------------------------------|
| SCB0_CRC0_CH0_IB_READ_QOS    | CRC0 Channel 0 Read Quality of Service Register    |
| SCB0_CRC0_CH0_IB_WRITE_QOS   | CRC0 Channel 0 Write Quality of Service Register   |
| SCB0_CRC0_CH1_IB_READ_QOS    | CRC0 Channel 1 Read Quality of Service Register    |
| SCB0_CRC0_CH1_IB_WRITE_QOS   | CRC0 Channel 1 Write Quality of Service Register   |
| SCB0_CRC1_CH0_IB_READ_QOS    | CRC1 Channel 0 Read Quality of Service Register    |
| SCB0_CRC1_CH0_IB_WRITE_QOS   | CRC1 Channel 0 Write Quality of Service Register   |
| SCB0_CRC1_CH1_IB_READ_QOS    | CRC1 Channel 1 Read Quality of Service Register    |
| SCB0_CRC1_CH1_IB_WRITE_QOS   | CRC1 Channel 1 Write Quality of Service Register   |
| SCB0_CRC2_CH0_IB_READ_QOS    | CRC2 Channel 0 Read Quality of Service Register    |
| SCB0_CRC2_CH0_IB_WRITE_QOS   | CRC2 Channel 0 Write Quality of Service Register   |
| SCB0_CRC2_CH1_IB_READ_QOS    | CRC2 Channel 1 Read Quality of Service Register    |
| SCB0_CRC2_CH1_IB_WRITE_QOS   | CRC2 Channel 1 Write Quality of Service Register   |
| SCB0_CRC3_CH0_IB_READ_QOS    | CRC3 Channel 0 Read Quality of Service Register    |
| SCB0_CRC3_CH0_IB_WRITE_QOS   | CRC3 Channel 0 Write Quality of Service Register   |
| SCB0_CRC3_CH1_IB_READ_QOS    | CRC3 Channel 1 Read Quality of Service Register    |
| SCB0_CRC3_CH1_IB_WRITE_QOS   | CRC3 Channel 1 Write Quality of Service Register   |
| SCB0_CRYPTO_IB_READ_QOS      | CRYPTO Read Quality of Service Register            |
| SCB0_CRYPTO_IB_WRITE_QOS     | CRYPTO Write Quality of Service Register           |
| SCB0_DBG_IB_READ_QOS         | DBG Read Quality of Service Register               |
| SCB0_DBG_IB_WRITE_QOS        | DBG Write Quality of Service Register              |
| SCB0_DLDMA0_CH0_IB_READ_QOS  | DLDMA0 Channel 0 Read Quality of Service Register  |
| SCB0_DLDMA0_CH0_IB_WRITE_QOS | DLDMA0 Channel 0 Write Quality of Service Register |
| SCB0_DLDMA0_CH1_IB_READ_QOS  | DLDMA0 Channel 1 Read Quality of Service Register  |
| SCB0_DLDMA0_CH1_IB_WRITE_QOS | DLDMA0 Channel 1 Write Quality of Service Register |
| SCB0_DLDMA1_CH0_IB_READ_QOS  | DLDMA1 Channel 0 Read Quality of Service Register  |
| SCB0_DLDMA1_CH0_IB_WRITE_QOS | DLDMA1 Channel 0 Write Quality of Service Register |
| SCB0_DLDMA1_CH1_IB_READ_QOS  | DLDMA1 Channel 1 Read Quality of Service Register  |
| SCB0_DLDMA1_CH1_IB_WRITE_QOS | DLDMA1 Channel 1 Write Quality of Service Register |
| SCB0_EMAC_IB_READ_QOS        | EMAC Read Quality of Service Register              |

Table 48-9: ADSP-SC59x SCB0 Register List (Continued)

| Name                          | Description                                         |
|-------------------------------|-----------------------------------------------------|
| SCB0_EMAC_IB_WRITE_QOS        | EMAC Write Quality of Service Register              |
| SCB0_ETR_IB_READ_QOS          | ETR Read Quality of Service Register                |
| SCB0_ETR_IB_WRITE_QOS         | ETR Write Quality of Service Register               |
| SCB0_GIGE_IB_READ_QOS         | GIGE Read Quality of Service Register               |
| SCB0_GIGE_IB_WRITE_QOS        | GIGE Write Quality of Service Register              |
| SCB0_HSMDMA1_CH0_IB_READ_QOS  | HSMDMA1 Channel 0 Read Quality of Service Register  |
| SCB0_HSMDMA1_CH0_IB_WRITE_QOS | HSMDMA1 Channel 0 Write Quality of Service Register |
| SCB0_HSMDMA1_CH1_IB_READ_QOS  | HSMDMA1 Channel 1 Read Quality of Service Register  |
| SCB0_HSMDMA1_CH1_IB_WRITE_QOS | HSMDMA1 Channel 1 Write Quality of Service Register |
| SCB0_HSMDMA_CH0_IB_READ_QOS   | HSMDMA Channel 0 Read Quality of Service Register   |
| SCB0_HSMDMA_CH0_IB_WRITE_QOS  | HSMDMA Channel 0 Write Quality of Service Register  |
| SCB0_HSMDMA_CH1_IB_READ_QOS   | HSMDMA Channel 1 Read Quality of Service Register   |
| SCB0_HSMDMA_CH1_IB_WRITE_QOS  | HSMDMA Channel 1 Write Quality of Service Register  |
| SCB0_LP0_READ_QOS             | LP0 Read Quality of Service Register                |
| SCB0_LP0_WRITE_QOS            | LP0 Write Quality of Service Register               |
| SCB0_LP1_READ_QOS             | LP1 Read Quality of Service Register                |
| SCB0_LP1_WRITE_QOS            | LP1 Write Quality of Service Register               |
| SCB0_MLB_READ_QOS             | MLB Read Quality of Service Register                |
| SCB0_MLB_WRITE_QOS            | MLB Write Quality of Service Register               |
| SCB0_MSMDMA1_CH0_IB_READ_QOS  | MSMDMA1 Channel 0 Read Quality of Service Register  |
| SCB0_MSMDMA1_CH0_IB_WRITE_QOS | MSMDMA1 Channel 0 Write Quality of Service Register |
| SCB0_MSMDMA1_CH1_IB_READ_QOS  | MSMDMA1 Channel 1 Read Quality of Service Register  |
| SCB0_MSMDMA1_CH1_IB_WRITE_QOS | MSMDMA1 Channel 1 Write Quality of Service Register |
| SCB0_MSMDMA_CH0_IB_READ_QOS   | MSMDMAChannel 0 Read Quality of Service Register    |
| SCB0_MSMDMA_CH0_IB_WRITE_QOS  | MSMDMAChannel 0 Write Quality of Service Register   |
| SCB0_MSMDMA_CH1_IB_READ_QOS   | MSMDMAChannel 1 Read Quality of Service Register    |
| SCB0_MSMDMA_CH1_IB_WRITE_QOS  | MSMDMAChannel 1 Write Quality of Service Register   |
| SCB0_PL310_M0_IB_READ_QOS     | PL310 M0 Read Quality of Service Register           |
| SCB0_PL310_M0_IB_WRITE_QOS    | PL310 M0 Write Quality of Service Register          |
| SCB0_PL310_M1_IB_READ_QOS     | PL310 M1 Read Quality of Service Register           |
| SCB0_PL310_M1_IB_WRITE_QOS    | PL310 M1 Write Quality of Service Register          |

Table 48-9: ADSP-SC59x SCB0 Register List (Continued)

| Name                        | Description                                         |
|-----------------------------|-----------------------------------------------------|
| SCB0_PL310_MMR_IB_READ_QOS  | PL310 MMRRead Quality of Service Register           |
| SCB0_PL310_MMR_IB_WRITE_QOS | PL310 MMRWrite Quality of Service Register          |
| SCB0_PPI_F0_READ_QOS        | PPI F0 Read Quality of Service Register             |
| SCB0_PPI_F0_WRITE_QOS       | PPI F0 Write Quality of Service Register            |
| SCB0_PPI_F1_READ_QOS        | PPI F1 Read Quality of Service Register             |
| SCB0_PPI_F1_WRITE_QOS       | PPI F1 Write Quality of Service Register            |
| SCB0_SCB6_IB_SYNC_MODE      | SCB6 Synchronization Mode Register                  |
| SCB0_SDIO0_IB_READ_QOS      | SDIO0 Read Quality of Service Register              |
| SCB0_SDIO0_IB_WRITE_QOS     | SDIO0 Write Quality of Service Register             |
| SCB0_SH0_DPORT_READ_QOS     | SH0 DPORT Read Quality of Service Register          |
| SCB0_SH0_DPORT_WRITE_QOS    | SH0 DPORT Write Quality of Service Register         |
| SCB0_SH0_FIR_CH0_READ_QOS   | SH0 FIR Channel 0Read Quality of Service Register   |
| SCB0_SH0_FIR_CH0_WRITE_QOS  | SH0 FIR Channel 0Write Quality of Service Register  |
| SCB0_SH0_FIR_CH1_READ_QOS   | SH0 FIR Channel 1Read Quality of Service Register   |
| SCB0_SH0_FIR_CH1_WRITE_QOS  | SH0 FIR Channel 1Write Quality of Service Register  |
| SCB0_SH0_IIR_CH0_READ_QOS   | SH0 IIR Channel 0 Read Quality of Service Register  |
| SCB0_SH0_IIR_CH0_WRITE_QOS  | SH0 IIR Channel 0 Write Quality of Service Register |
| SCB0_SH0_IIR_CH1_READ_QOS   | SH0 IIR Channel 1 Read Quality of Service Register  |
| SCB0_SH0_IIR_CH1_WRITE_QOS  | SH0 IIR Channel 1 Write Quality of Service Register |
| SCB0_SH0_IPORT_READ_QOS     | SH0 IPORT Read Quality of Service Register          |
| SCB0_SH0_IPORT_WRITE_QOS    | SH0 IPORT Write Quality of Service Register         |
| SCB0_SH0_MMR_IB_READ_QOS    | SH0 MMRRead Quality of Service Register             |
| SCB0_SH0_MMR_IB_WRITE_QOS   | SH0 MMRWrite Quality of Service Register            |
| SCB0_SH1_DPORT_READ_QOS     | SH1 DPORT Read Quality of Service Register          |
| SCB0_SH1_DPORT_WRITE_QOS    | SH1 DPORT Write Quality of Service Register         |
| SCB0_SH1_FIR_CH0_READ_QOS   | SH1 FIR Channel 0 Read Quality of Service Register  |
| SCB0_SH1_FIR_CH0_WRITE_QOS  | SH1 FIR Channel 0 Write Quality of Service Register |
| SCB0_SH1_FIR_CH1_READ_QOS   | SH1 FIR Channel 1 Read Quality of Service Register  |
| SCB0_SH1_FIR_CH1_WRITE_QOS  | SH1 FIR Channel 1 Write Quality of Service Register |
| SCB0_SH1_IIR_CH0_READ_QOS   | SH1 IIR Channel 0 Read Quality of Service Register  |
| SCB0_SH1_IIR_CH0_WRITE_QOS  | SH1 IIR Channel 0 Write Quality of Service Register |

Table 48-9: ADSP-SC59x SCB0 Register List (Continued)

| Name                       | Description                                         |
|----------------------------|-----------------------------------------------------|
| SCB0_SH1_IIR_CH1_READ_QOS  | SH1 IIR Channel 1 Read Quality of Service Register  |
| SCB0_SH1_IIR_CH1_WRITE_QOS | SH1 IIR Channel 1 Write Quality of Service Register |
| SCB0_SH1_IPORT_READ_QOS    | SH1 IPORT Read Quality of Service Register          |
| SCB0_SH1_IPORT_WRITE_QOS   | SH1 IPORT Write Quality of Service Register         |
| SCB0_SH1_MMR_IB_READ_QOS   | SH1 MMRRead Quality of Service Register             |
| SCB0_SH1_MMR_IB_WRITE_QOS  | SH1 MMRWrite Quality of Service Register            |
| SCB0_SP0A_READ_QOS         | SP0A Read Quality of Service Register               |
| SCB0_SP0A_WRITE_QOS        | SP0A Write Quality of Service Register              |
| SCB0_SP0B_READ_QOS         | SP0B Read Quality of Service Register               |
| SCB0_SP0B_WRITE_QOS        | SP0B Write Quality of Service Register              |
| SCB0_SP1A_READ_QOS         | SP1A Read Quality of Service Register               |
| SCB0_SP1A_WRITE_QOS        | SP1A Write Quality of Service Register              |
| SCB0_SP1B_READ_QOS         | SP1B Read Quality of Service Register               |
| SCB0_SP1B_WRITE_QOS        | SP1B Write Quality of Service Register              |
| SCB0_SP2A_READ_QOS         | SP2A Read Quality of Service Register               |
| SCB0_SP2A_WRITE_QOS        | SP2A Write Quality of Service Register              |
| SCB0_SP2B_READ_QOS         | SP1B Read Quality of Service Register               |
| SCB0_SP2B_WRITE_QOS        | SP1B Write Quality of Service Register              |
| SCB0_SP3A_READ_QOS         | SP3A Read Quality of Service Register               |
| SCB0_SP3A_WRITE_QOS        | SP3A Write Quality of Service Register              |
| SCB0_SP3B_READ_QOS         | SP3B Read Quality of Service Register               |
| SCB0_SP3B_WRITE_QOS        | SP3B Write Quality of Service Register              |
| SCB0_SP4A_READ_QOS         | SP4A Read Quality of Service Register               |
| SCB0_SP4A_WRITE_QOS        | SP4A Write Quality of Service Register              |
| SCB0_SP4B_READ_QOS         | SP4B Read Quality of Service Register               |
| SCB0_SP4B_WRITE_QOS        | SP4B Write Quality of Service Register              |
| SCB0_SP5A_READ_QOS         | SP5A Read Quality of Service Register               |
| SCB0_SP5A_WRITE_QOS        | SP5A Write Quality of Service Register              |
| SCB0_SP5B_READ_QOS         | SP5B Read Quality of Service Register               |
| SCB0_SP5B_WRITE_QOS        | SP5B Write Quality of Service Register              |
| SCB0_SP6A_READ_QOS         | SP6A Read Quality of Service Register               |

Table 48-9: ADSP-SC59x SCB0 Register List (Continued)

| Name                    | Description                                |
|-------------------------|--------------------------------------------|
| SCB0_SP6A_WRITE_QOS     | SP6A Write Quality of Service Register     |
| SCB0_SP6B_READ_QOS      | SP6B Read Quality of Service Register      |
| SCB0_SP6B_WRITE_QOS     | SP6B Write Quality of Service Register     |
| SCB0_SP7A_READ_QOS      | SP7A Read Quality of Service Register      |
| SCB0_SP7A_WRITE_QOS     | SP7A Write Quality of Service Register     |
| SCB0_SP7B_READ_QOS      | SP7B Read Quality of Service Register      |
| SCB0_SP7B_WRITE_QOS     | SP7B Write Quality of Service Register     |
| SCB0_SPI0RX_READ_QOS    | SPI0 RX Read Quality of Service Register   |
| SCB0_SPI0RX_WRITE_QOS   | SPI0 RX Write Quality of Service Register  |
| SCB0_SPI0TX_READ_QOS    | SPI0 TX Read Quality of Service Register   |
| SCB0_SPI0TX_WRITE_QOS   | SPI0 TX Write Quality of Service Register  |
| SCB0_SPI1RX_READ_QOS    | SPI1 RX Read Quality of Service Register   |
| SCB0_SPI1RX_WRITE_QOS   | SPI1 RX Write Quality of Service Register  |
| SCB0_SPI1TX_READ_QOS    | SPI1 TX Read Quality of Service Register   |
| SCB0_SPI1TX_WRITE_QOS   | SPI1 TX Write Quality of Service Register  |
| SCB0_SPI2RX_READ_QOS    | SPI2 RX Read Quality of Service Register   |
| SCB0_SPI2RX_WRITE_QOS   | SPI2 RX Write Quality of Service Register  |
| SCB0_SPI2TX_READ_QOS    | SPI2 TX Read Quality of Service Register   |
| SCB0_SPI2TX_WRITE_QOS   | SPI2 TX Write Quality of Service Register  |
| SCB0_SPI3RX_READ_QOS    | SPI3 RX Read Quality of Service Register   |
| SCB0_SPI3RX_WRITE_QOS   | SPI3 RX Write Quality of Service Register  |
| SCB0_SPI3TX_READ_QOS    | SPI3 TX Read Quality of Service Register   |
| SCB0_SPI3TX_WRITE_QOS   | SPI3 TX Write Quality of Service Register  |
| SCB0_UART0_RX_READ_QOS  | UART0 RX Read Quality of Service Register  |
| SCB0_UART0_RX_WRITE_QOS | UART0 RX Write Quality of Service Register |
| SCB0_UART0_TX_READ_QOS  | UART0 TX Read Quality of Service Register  |
| SCB0_UART0_TX_WRITE_QOS | UART0 TX Write Quality of Service Register |
| SCB0_UART1_RX_READ_QOS  | UART1 RX Read Quality of Service Register  |
| SCB0_UART1_RX_WRITE_QOS | UART1 RX Write Quality of Service Register |
| SCB0_UART1_TX_READ_QOS  | UART1 TX Read Quality of Service Register  |
| SCB0_UART1_TX_WRITE_QOS | UART1 TX Write Quality of Service Register |

Table 48-9: ADSP-SC59x SCB0 Register List (Continued)

| Name                    | Description                                |
|-------------------------|--------------------------------------------|
| SCB0_UART2_RX_READ_QOS  | UART2 RX Read Quality of Service Register  |
| SCB0_UART2_RX_WRITE_QOS | UART2 RX Write Quality of Service Register |
| SCB0_UART2_TX_READ_QOS  | UART2 TX Read Quality of Service Register  |
| SCB0_UART2_TX_WRITE_QOS | UART2 TX Write Quality of Service Register |
| SCB0_UART3_RX_READ_QOS  | UART3 RX Read Quality of Service Register  |
| SCB0_UART3_RX_WRITE_QOS | UART3 RX Write Quality of Service Register |
| SCB0_UART3_TX_READ_QOS  | UART3 TX Read Quality of Service Register  |
| SCB0_UART3_TX_WRITE_QOS | UART3 TX Write Quality of Service Register |
| SCB0_USB0_IB_READ_QOS   | USB0 Read Quality of Service Register      |
| SCB0_USB0_IB_WRITE_QOS  | USB0 Write Quality of Service Register     |

## CRC0 Channel 0 Read Quality of Service Register

The SCB0\_CRC0\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-9: SCB0\_CRC0\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-10: SCB0\_CRC0\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC0 Channel 0 Write Quality of Service Register

The SCB0\_CRC0\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-10: SCB0\_CRC0\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-11: SCB0\_CRC0\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC0 Channel 1 Read Quality of Service Register

The SCB0\_CRC0\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-11: SCB0\_CRC0\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-12: SCB0\_CRC0\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC0 Channel 1 Write Quality of Service Register

The SCB0\_CRC0\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-12: SCB0\_CRC0\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-13: SCB0\_CRC0\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC1 Channel 0 Read Quality of Service Register

The SCB0\_CRC1\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-13: SCB0\_CRC1\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-14: SCB0\_CRC1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC1 Channel 0 Write Quality of Service Register

The SCB0\_CRC1\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-14: SCB0\_CRC1\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-15: SCB0\_CRC1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC1 Channel 1 Read Quality of Service Register

The SCB0\_CRC1\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-15: SCB0\_CRC1\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-16: SCB0\_CRC1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC1 Channel 1 Write Quality of Service Register

The SCB0\_CRC1\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-16: SCB0\_CRC1\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-17: SCB0\_CRC1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC2 Channel 0 Read Quality of Service Register

The SCB0\_CRC2\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-17: SCB0\_CRC2\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-18: SCB0\_CRC2\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC2 Channel 0 Write Quality of Service Register

The SCB0\_CRC2\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-18: SCB0\_CRC2\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-19: SCB0\_CRC2\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC2 Channel 1 Read Quality of Service Register

The SCB0\_CRC2\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-19: SCB0\_CRC2\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-20: SCB0\_CRC2\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC2 Channel 1 Write Quality of Service Register

The SCB0\_CRC2\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-20: SCB0\_CRC2\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-21: SCB0\_CRC2\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC3 Channel 0 Read Quality of Service Register

The SCB0\_CRC3\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-21: SCB0\_CRC3\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-22: SCB0\_CRC3\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC3 Channel 0 Write Quality of Service Register

The SCB0\_CRC3\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-22: SCB0\_CRC3\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-23: SCB0\_CRC3\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRC3 Channel 1 Read Quality of Service Register

The SCB0\_CRC3\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-23: SCB0\_CRC3\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-24: SCB0\_CRC3\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRC3 Channel 1 Write Quality of Service Register

The SCB0\_CRC3\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-24: SCB0\_CRC3\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-25: SCB0\_CRC3\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## CRYPTO Read Quality of Service Register

The SCB0\_CRYPTO\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-25: SCB0\_CRYPTO\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-26: SCB0\_CRYPTO\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## CRYPTO Write Quality of Service Register

The SCB0\_CRYPTO\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-26: SCB0\_CRYPTO\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-27: SCB0\_CRYPTO\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## DBG Read Quality of Service Register

The SCB0\_DBG\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-27: SCB0\_DBG\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-28: SCB0\_DBG\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## DBG Write Quality of Service Register

The SCB0\_DBG\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-28: SCB0\_DBG\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-29: SCB0\_DBG\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## DLDMA0 Channel 0 Read Quality of Service Register

The SCB0\_DLDMA0\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-29: SCB0\_DLDMA0\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-30: SCB0\_DLDMA0\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## DLDMA0 Channel 0 Write Quality of Service Register

The SCB0\_DLDMA0\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-30: SCB0\_DLDMA0\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-31: SCB0\_DLDMA0\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## DLDMA0 Channel 1 Read Quality of Service Register

The SCB0\_DLDMA0\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-31: SCB0\_DLDMA0\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-32: SCB0\_DLDMA0\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## DLDMA0 Channel 1 Write Quality of Service Register

The SCB0\_DLDMA0\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-32: SCB0\_DLDMA0\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-33: SCB0\_DLDMA0\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## DLDMA1 Channel 0 Read Quality of Service Register

The SCB0\_DLDMA1\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-33: SCB0\_DLDMA1\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-34: SCB0\_DLDMA1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## DLDMA1 Channel 0 Write Quality of Service Register

The SCB0\_DLDMA1\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-34: SCB0\_DLDMA1\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-35: SCB0\_DLDMA1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## DLDMA1 Channel 1 Read Quality of Service Register

The SCB0\_DLDMA1\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-35: SCB0\_DLDMA1\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-36: SCB0\_DLDMA1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## DLDMA1 Channel 1 Write Quality of Service Register

The SCB0\_DLDMA1\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-36: SCB0\_DLDMA1\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-37: SCB0\_DLDMA1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## EMAC Read Quality of Service Register

The SCB0\_EMAC\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-37: SCB0\_EMAC\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-38: SCB0\_EMAC\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## EMAC Write Quality of Service Register

The SCB0\_EMAC\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-38: SCB0\_EMAC\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-39: SCB0\_EMAC\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## ETR Read Quality of Service Register

The SCB0\_ETR\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-39: SCB0\_ETR\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-40: SCB0\_ETR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## ETR Write Quality of Service Register

The SCB0\_ETR\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-40: SCB0\_ETR\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-41: SCB0\_ETR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## GIGE Read Quality of Service Register

The SCB0\_GIGE\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-41: SCB0\_GIGE\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-42: SCB0\_GIGE\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## GIGE Write Quality of Service Register

The SCB0\_GIGE\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-42: SCB0\_GIGE\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-43: SCB0\_GIGE\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## HSMDMA1 Channel 0 Read Quality of Service Register

The SCB0\_HSMDMA1\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-43: SCB0\_HSMDMA1\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-44: SCB0\_HSMDMA1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## HSMDMA1 Channel 0 Write Quality of Service Register

The SCB0\_HSMDMA1\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-44: SCB0\_HSMDMA1\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-45: SCB0\_HSMDMA1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## HSMDMA1 Channel 1 Read Quality of Service Register

The SCB0\_HSMDMA1\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-45: SCB0\_HSMDMA1\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-46: SCB0\_HSMDMA1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## HSMDMA1 Channel 1 Write Quality of Service Register

The SCB0\_HSMDMA1\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-46: SCB0\_HSMDMA1\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-47: SCB0\_HSMDMA1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## HSMDMA Channel 0 Read Quality of Service Register

The SCB0\_HSMDMA\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-47: SCB0\_HSMDMA\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-48: SCB0\_HSMDMA\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## HSMDMA Channel 0 Write Quality of Service Register

The SCB0\_HSMDMA\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-48: SCB0\_HSMDMA\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-49: SCB0\_HSMDMA\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## HSMDMA Channel 1 Read Quality of Service Register

The SCB0\_HSMDMA\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-49: SCB0\_HSMDMA\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-50: SCB0\_HSMDMA\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## HSMDMA Channel 1 Write Quality of Service Register

The SCB0\_HSMDMA\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-50: SCB0\_HSMDMA\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-51: SCB0\_HSMDMA\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## LP0 Read Quality of Service Register

The SCB0\_LP0\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-51: SCB0\_LP0\_READ\_QOS Register Diagram

<!-- image -->

Table 48-52: SCB0\_LP0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## LP0 Write Quality of Service Register

The SCB0\_LP0\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-52: SCB0\_LP0\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-53: SCB0\_LP0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## LP1 Read Quality of Service Register

The SCB0\_LP1\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-53: SCB0\_LP1\_READ\_QOS Register Diagram

<!-- image -->

Table 48-54: SCB0\_LP1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## LP1 Write Quality of Service Register

The SCB0\_LP1\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-54: SCB0\_LP1\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-55: SCB0\_LP1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## MLB Read Quality of Service Register

The SCB0\_MLB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-55: SCB0\_MLB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-56: SCB0\_MLB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## MLB Write Quality of Service Register

The SCB0\_MLB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-56: SCB0\_MLB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-57: SCB0\_MLB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## MSMDMA1 Channel 0 Read Quality of Service Register

The SCB0\_MSMDMA1\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-57: SCB0\_MSMDMA1\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-58: SCB0\_MSMDMA1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## MSMDMA1 Channel 0 Write Quality of Service Register

The SCB0\_MSMDMA1\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-58: SCB0\_MSMDMA1\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-59: SCB0\_MSMDMA1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## MSMDMA1 Channel 1 Read Quality of Service Register

The SCB0\_MSMDMA1\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-59: SCB0\_MSMDMA1\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-60: SCB0\_MSMDMA1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## MSMDMA1 Channel 1 Write Quality of Service Register

The SCB0\_MSMDMA1\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-60: SCB0\_MSMDMA1\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-61: SCB0\_MSMDMA1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## MSMDMA Channel 0 Read Quality of Service Register

The SCB0\_MSMDMA\_CH0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-61: SCB0\_MSMDMA\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-62: SCB0\_MSMDMA\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## MSMDMA Channel 0 Write Quality of Service Register

The SCB0\_MSMDMA\_CH0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-62: SCB0\_MSMDMA\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-63: SCB0\_MSMDMA\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## MSMDMA Channel 1 Read Quality of Service Register

The SCB0\_MSMDMA\_CH1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-63: SCB0\_MSMDMA\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-64: SCB0\_MSMDMA\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## MSMDMA Channel 1 Write Quality of Service Register

The SCB0\_MSMDMA\_CH1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-64: SCB0\_MSMDMA\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-65: SCB0\_MSMDMA\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## PL310 M0 Read Quality of Service Register

The SCB0\_PL310\_M0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-65: SCB0\_PL310\_M0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-66: SCB0\_PL310\_M0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## PL310 M0 Write Quality of Service Register

The SCB0\_PL310\_M0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-66: SCB0\_PL310\_M0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-67: SCB0\_PL310\_M0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## PL310 M1 Read Quality of Service Register

The SCB0\_PL310\_M1\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-67: SCB0\_PL310\_M1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-68: SCB0\_PL310\_M1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## PL310 M1 Write Quality of Service Register

The SCB0\_PL310\_M1\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-68: SCB0\_PL310\_M1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-69: SCB0\_PL310\_M1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## PL310 MMR Read Quality of Service Register

The SCB0\_PL310\_MMR\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-69: SCB0\_PL310\_MMR\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-70: SCB0\_PL310\_MMR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## PL310 MMR Write Quality of Service Register

The SCB0\_PL310\_MMR\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-70: SCB0\_PL310\_MMR\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-71: SCB0\_PL310\_MMR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## PPI F0 Read Quality of Service Register

The SCB0\_PPI\_F0\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-71: SCB0\_PPI\_F0\_READ\_QOS Register Diagram

<!-- image -->

Table 48-72: SCB0\_PPI\_F0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## PPI F0 Write Quality of Service Register

The SCB0\_PPI\_F0\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-72: SCB0\_PPI\_F0\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-73: SCB0\_PPI\_F0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## PPI F1 Read Quality of Service Register

The SCB0\_PPI\_F1\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-73: SCB0\_PPI\_F1\_READ\_QOS Register Diagram

<!-- image -->

Table 48-74: SCB0\_PPI\_F1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## PPI F1 Write Quality of Service Register

The SCB0\_PPI\_F1\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-74: SCB0\_PPI\_F1\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-75: SCB0\_PPI\_F1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SCB6 Synchronization Mode Register

Figure 48-75: SCB0\_SCB6\_IB\_SYNC\_MODE Register Diagram

<!-- image -->

Table 48-76: SCB0\_SCB6\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | Sync Mode.                |
| (R/W)              |            |                           |

## SDIO0 Read Quality of Service Register

The SCB0\_SDIO0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-76: SCB0\_SDIO0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-77: SCB0\_SDIO0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SDIO0 Write Quality of Service Register

The SCB0\_SDIO0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-77: SCB0\_SDIO0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-78: SCB0\_SDIO0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 DPORT Read Quality of Service Register

The SCB0\_SH0\_DPORT\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-78: SCB0\_SH0\_DPORT\_READ\_QOS Register Diagram

<!-- image -->

Table 48-79: SCB0\_SH0\_DPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 DPORT Write Quality of Service Register

The SCB0\_SH0\_DPORT\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-79: SCB0\_SH0\_DPORT\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-80: SCB0\_SH0\_DPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 FIR Channel 0Read Quality of Service Register

The SCB0\_SH0\_FIR\_CH0\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-80: SCB0\_SH0\_FIR\_CH0\_READ\_QOS Register Diagram

<!-- image -->

Table 48-81: SCB0\_SH0\_FIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 FIR Channel 0Write Quality of Service Register

The SCB0\_SH0\_FIR\_CH0\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-81: SCB0\_SH0\_FIR\_CH0\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-82: SCB0\_SH0\_FIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 FIR Channel 1Read Quality of Service Register

The SCB0\_SH0\_FIR\_CH1\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-82: SCB0\_SH0\_FIR\_CH1\_READ\_QOS Register Diagram

<!-- image -->

Table 48-83: SCB0\_SH0\_FIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 FIR Channel 1Write Quality of Service Register

The SCB0\_SH0\_FIR\_CH1\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-83: SCB0\_SH0\_FIR\_CH1\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-84: SCB0\_SH0\_FIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 IIR Channel 0 Read Quality of Service Register

The SCB0\_SH0\_IIR\_CH0\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-84: SCB0\_SH0\_IIR\_CH0\_READ\_QOS Register Diagram

<!-- image -->

Table 48-85: SCB0\_SH0\_IIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 IIR Channel 0 Write Quality of Service Register

The SCB0\_SH0\_IIR\_CH0\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-85: SCB0\_SH0\_IIR\_CH0\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-86: SCB0\_SH0\_IIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 IIR Channel 1 Read Quality of Service Register

The SCB0\_SH0\_IIR\_CH1\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-86: SCB0\_SH0\_IIR\_CH1\_READ\_QOS Register Diagram

<!-- image -->

Table 48-87: SCB0\_SH0\_IIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 IIR Channel 1 Write Quality of Service Register

The SCB0\_SH0\_IIR\_CH1\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-87: SCB0\_SH0\_IIR\_CH1\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-88: SCB0\_SH0\_IIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 IPORT Read Quality of Service Register

The SCB0\_SH0\_IPORT\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-88: SCB0\_SH0\_IPORT\_READ\_QOS Register Diagram

<!-- image -->

Table 48-89: SCB0\_SH0\_IPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 IPORT Write Quality of Service Register

The SCB0\_SH0\_IPORT\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-89: SCB0\_SH0\_IPORT\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-90: SCB0\_SH0\_IPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH0 MMR Read Quality of Service Register

The SCB0\_SH0\_MMR\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-90: SCB0\_SH0\_MMR\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-91: SCB0\_SH0\_MMR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH0 MMR Write Quality of Service Register

The SCB0\_SH0\_MMR\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-91: SCB0\_SH0\_MMR\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-92: SCB0\_SH0\_MMR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 DPORT Read Quality of Service Register

The SCB0\_SH1\_DPORT\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-92: SCB0\_SH1\_DPORT\_READ\_QOS Register Diagram

<!-- image -->

Table 48-93: SCB0\_SH1\_DPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 DPORT Write Quality of Service Register

The SCB0\_SH1\_DPORT\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-93: SCB0\_SH1\_DPORT\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-94: SCB0\_SH1\_DPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 FIR Channel 0 Read Quality of Service Register

The SCB0\_SH1\_FIR\_CH0\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-94: SCB0\_SH1\_FIR\_CH0\_READ\_QOS Register Diagram

<!-- image -->

Table 48-95: SCB0\_SH1\_FIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 FIR Channel 0 Write Quality of Service Register

The SCB0\_SH1\_FIR\_CH0\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-95: SCB0\_SH1\_FIR\_CH0\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-96: SCB0\_SH1\_FIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 FIR Channel 1 Read Quality of Service Register

The SCB0\_SH1\_FIR\_CH1\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-96: SCB0\_SH1\_FIR\_CH1\_READ\_QOS Register Diagram

<!-- image -->

Table 48-97: SCB0\_SH1\_FIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 FIR Channel 1 Write Quality of Service Register

The SCB0\_SH1\_FIR\_CH1\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-97: SCB0\_SH1\_FIR\_CH1\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-98: SCB0\_SH1\_FIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 IIR Channel 0 Read Quality of Service Register

The SCB0\_SH1\_IIR\_CH0\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-98: SCB0\_SH1\_IIR\_CH0\_READ\_QOS Register Diagram

<!-- image -->

Table 48-99: SCB0\_SH1\_IIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 IIR Channel 0 Write Quality of Service Register

The SCB0\_SH1\_IIR\_CH0\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-99: SCB0\_SH1\_IIR\_CH0\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-100: SCB0\_SH1\_IIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 IIR Channel 1 Read Quality of Service Register

The SCB0\_SH1\_IIR\_CH1\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-100: SCB0\_SH1\_IIR\_CH1\_READ\_QOS Register Diagram

<!-- image -->

Table 48-101: SCB0\_SH1\_IIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 IIR Channel 1 Write Quality of Service Register

The SCB0\_SH1\_IIR\_CH1\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-101: SCB0\_SH1\_IIR\_CH1\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-102: SCB0\_SH1\_IIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 IPORT Read Quality of Service Register

The SCB0\_SH1\_IPORT\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-102: SCB0\_SH1\_IPORT\_READ\_QOS Register Diagram

<!-- image -->

Table 48-103: SCB0\_SH1\_IPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 IPORT Write Quality of Service Register

The SCB0\_SH1\_IPORT\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-103: SCB0\_SH1\_IPORT\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-104: SCB0\_SH1\_IPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SH1 MMR Read Quality of Service Register

The SCB0\_SH1\_MMR\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-104: SCB0\_SH1\_MMR\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-105: SCB0\_SH1\_MMR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SH1 MMR Write Quality of Service Register

The SCB0\_SH1\_MMR\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-105: SCB0\_SH1\_MMR\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-106: SCB0\_SH1\_MMR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP0A Read Quality of Service Register

The SCB0\_SP0A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-106: SCB0\_SP0A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-107: SCB0\_SP0A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP0A Write Quality of Service Register

The SCB0\_SP0A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-107: SCB0\_SP0A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-108: SCB0\_SP0A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP0B Read Quality of Service Register

The SCB0\_SP0B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-108: SCB0\_SP0B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-109: SCB0\_SP0B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP0B Write Quality of Service Register

The SCB0\_SP0B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-109: SCB0\_SP0B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-110: SCB0\_SP0B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP1A Read Quality of Service Register

The SCB0\_SP1A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-110: SCB0\_SP1A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-111: SCB0\_SP1A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP1A Write Quality of Service Register

The SCB0\_SP1A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-111: SCB0\_SP1A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-112: SCB0\_SP1A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP1B Read Quality of Service Register

The SCB0\_SP1B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-112: SCB0\_SP1B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-113: SCB0\_SP1B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP1B Write Quality of Service Register

The SCB0\_SP1B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-113: SCB0\_SP1B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-114: SCB0\_SP1B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP2A Read Quality of Service Register

The SCB0\_SP2A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-114: SCB0\_SP2A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-115: SCB0\_SP2A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP2A Write Quality of Service Register

The SCB0\_SP2A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-115: SCB0\_SP2A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-116: SCB0\_SP2A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP1B Read Quality of Service Register

The SCB0\_SP2B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-116: SCB0\_SP2B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-117: SCB0\_SP2B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP1B Write Quality of Service Register

The SCB0\_SP2B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-117: SCB0\_SP2B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-118: SCB0\_SP2B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP3A Read Quality of Service Register

The SCB0\_SP3A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-118: SCB0\_SP3A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-119: SCB0\_SP3A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP3A Write Quality of Service Register

The SCB0\_SP3A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-119: SCB0\_SP3A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-120: SCB0\_SP3A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP3B Read Quality of Service Register

The SCB0\_SP3B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-120: SCB0\_SP3B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-121: SCB0\_SP3B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP3B Write Quality of Service Register

The SCB0\_SP3B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-121: SCB0\_SP3B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-122: SCB0\_SP3B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP4A Read Quality of Service Register

The SCB0\_SP4A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-122: SCB0\_SP4A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-123: SCB0\_SP4A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP4A Write Quality of Service Register

The SCB0\_SP4A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-123: SCB0\_SP4A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-124: SCB0\_SP4A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP4B Read Quality of Service Register

The SCB0\_SP4B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-124: SCB0\_SP4B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-125: SCB0\_SP4B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP4B Write Quality of Service Register

The SCB0\_SP4B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-125: SCB0\_SP4B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-126: SCB0\_SP4B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP5A Read Quality of Service Register

The SCB0\_SP5A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-126: SCB0\_SP5A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-127: SCB0\_SP5A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP5A Write Quality of Service Register

The SCB0\_SP5A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-127: SCB0\_SP5A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-128: SCB0\_SP5A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP5B Read Quality of Service Register

The SCB0\_SP5B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-128: SCB0\_SP5B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-129: SCB0\_SP5B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP5B Write Quality of Service Register

The SCB0\_SP5B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-129: SCB0\_SP5B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-130: SCB0\_SP5B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP6A Read Quality of Service Register

The SCB0\_SP6A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-130: SCB0\_SP6A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-131: SCB0\_SP6A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP6A Write Quality of Service Register

The SCB0\_SP6A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-131: SCB0\_SP6A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-132: SCB0\_SP6A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP6B Read Quality of Service Register

The SCB0\_SP6B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-132: SCB0\_SP6B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-133: SCB0\_SP6B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP6B Write Quality of Service Register

The SCB0\_SP6B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-133: SCB0\_SP6B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-134: SCB0\_SP6B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP7A Read Quality of Service Register

The SCB0\_SP7A\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-134: SCB0\_SP7A\_READ\_QOS Register Diagram

<!-- image -->

Table 48-135: SCB0\_SP7A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP7A Write Quality of Service Register

The SCB0\_SP7A\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-135: SCB0\_SP7A\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-136: SCB0\_SP7A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SP7B Read Quality of Service Register

The SCB0\_SP7B\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-136: SCB0\_SP7B\_READ\_QOS Register Diagram

<!-- image -->

Table 48-137: SCB0\_SP7B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SP7B Write Quality of Service Register

The SCB0\_SP7B\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-137: SCB0\_SP7B\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-138: SCB0\_SP7B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI0 RX Read Quality of Service Register

The SCB0\_SPI0RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-138: SCB0\_SPI0RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-139: SCB0\_SPI0RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI0 RX Write Quality of Service Register

The SCB0\_SPI0RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-139: SCB0\_SPI0RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-140: SCB0\_SPI0RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI0 TX Read Quality of Service Register

The SCB0\_SPI0TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-140: SCB0\_SPI0TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-141: SCB0\_SPI0TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI0 TX Write Quality of Service Register

The SCB0\_SPI0TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-141: SCB0\_SPI0TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-142: SCB0\_SPI0TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI1 RX Read Quality of Service Register

The SCB0\_SPI1RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-142: SCB0\_SPI1RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-143: SCB0\_SPI1RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI1 RX Write Quality of Service Register

The SCB0\_SPI1RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-143: SCB0\_SPI1RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-144: SCB0\_SPI1RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI1 TX Read Quality of Service Register

The SCB0\_SPI1TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-144: SCB0\_SPI1TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-145: SCB0\_SPI1TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI1 TX Write Quality of Service Register

The SCB0\_SPI1TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-145: SCB0\_SPI1TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-146: SCB0\_SPI1TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI2 RX Read Quality of Service Register

The SCB0\_SPI2RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-146: SCB0\_SPI2RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-147: SCB0\_SPI2RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI2 RX Write Quality of Service Register

The SCB0\_SPI2RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-147: SCB0\_SPI2RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-148: SCB0\_SPI2RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI2 TX Read Quality of Service Register

The SCB0\_SPI2TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-148: SCB0\_SPI2TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-149: SCB0\_SPI2TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI2 TX Write Quality of Service Register

The SCB0\_SPI2TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-149: SCB0\_SPI2TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-150: SCB0\_SPI2TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI3 RX Read Quality of Service Register

The SCB0\_SPI3RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-150: SCB0\_SPI3RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-151: SCB0\_SPI3RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI3 RX Write Quality of Service Register

The SCB0\_SPI3RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-151: SCB0\_SPI3RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-152: SCB0\_SPI3RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## SPI3 TX Read Quality of Service Register

The SCB0\_SPI3TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-152: SCB0\_SPI3TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-153: SCB0\_SPI3TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## SPI3 TX Write Quality of Service Register

The SCB0\_SPI3TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-153: SCB0\_SPI3TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-154: SCB0\_SPI3TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART0 RX Read Quality of Service Register

The SCB0\_UART0\_RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-154: SCB0\_UART0\_RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-155: SCB0\_UART0\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART0 RX Write Quality of Service Register

The SCB0\_UART0\_RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-155: SCB0\_UART0\_RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-156: SCB0\_UART0\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART0 TX Read Quality of Service Register

The SCB0\_UART0\_TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-156: SCB0\_UART0\_TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-157: SCB0\_UART0\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART0 TX Write Quality of Service Register

The SCB0\_UART0\_TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-157: SCB0\_UART0\_TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-158: SCB0\_UART0\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART1 RX Read Quality of Service Register

The SCB0\_UART1\_RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-158: SCB0\_UART1\_RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-159: SCB0\_UART1\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART1 RX Write Quality of Service Register

The SCB0\_UART1\_RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-159: SCB0\_UART1\_RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-160: SCB0\_UART1\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART1 TX Read Quality of Service Register

The SCB0\_UART1\_TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-160: SCB0\_UART1\_TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-161: SCB0\_UART1\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART1 TX Write Quality of Service Register

The SCB0\_UART1\_TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-161: SCB0\_UART1\_TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-162: SCB0\_UART1\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART2 RX Read Quality of Service Register

The SCB0\_UART2\_RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-162: SCB0\_UART2\_RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-163: SCB0\_UART2\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART2 RX Write Quality of Service Register

The SCB0\_UART2\_RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-163: SCB0\_UART2\_RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-164: SCB0\_UART2\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART2 TX Read Quality of Service Register

The SCB0\_UART2\_TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-164: SCB0\_UART2\_TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-165: SCB0\_UART2\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART2 TX Write Quality of Service Register

The SCB0\_UART2\_TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-165: SCB0\_UART2\_TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-166: SCB0\_UART2\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART3 RX Read Quality of Service Register

The SCB0\_UART3\_RX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-166: SCB0\_UART3\_RX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-167: SCB0\_UART3\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART3 RX Write Quality of Service Register

The SCB0\_UART3\_RX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-167: SCB0\_UART3\_RX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-168: SCB0\_UART3\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## UART3 TX Read Quality of Service Register

The SCB0\_UART3\_TX\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-168: SCB0\_UART3\_TX\_READ\_QOS Register Diagram

<!-- image -->

Table 48-169: SCB0\_UART3\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## UART3 TX Write Quality of Service Register

The SCB0\_UART3\_TX\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-169: SCB0\_UART3\_TX\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-170: SCB0\_UART3\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## USB0 Read Quality of Service Register

The SCB0\_USB0\_IB\_READ\_QOS register indicates the read QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting read channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-170: SCB0\_USB0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-171: SCB0\_USB0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## USB0 Write Quality of Service Register

The SCB0\_USB0\_IB\_WRITE\_QOS register indicates the write QOS or priority value for the indicated bus requester. This value is used by the SCBs at different levels to arbitrate among the ports requesting write channel access. For the mapping of requester IDs to peripherals, see the SCB Bus Requester IDs table in the System Crossbars chapter.

Figure 48-171: SCB0\_USB0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-172: SCB0\_USB0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## ADSP-SC59x SCB1 Register Descriptions

(SCB1) contains the following registers.

Table 48-173: ADSP-SC59x SCB1 Register List

| Name                  | Description                                     |
|-----------------------|-------------------------------------------------|
| SCB1_MST_IB_SYNC_MODE | DMCFabric (CLK03) Synchronization Mode Register |

## DMC Fabric (CLK03) Synchronization Mode Register

Figure 48-172: SCB1\_MST\_IB\_SYNC\_MODE Register Diagram

<!-- image -->

Table 48-174: SCB1\_MST\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | SYNC_MODE  | Synchronization Mode. The SCB1_MST_IB_SYNC_MODE.SYNC_MODE bit field configures the SYSCLK- DCLK clock domain boundary (CDC) for the DMCfabric. 1:1 |
| 2:0 (R/W)          | SYNC_MODE  | 0 Sync                                                                                                                                             |
| 2:0 (R/W)          | SYNC_MODE  | 1 Sync n:1                                                                                                                                         |
| 2:0 (R/W)          | SYNC_MODE  | 2 Sync 1:n                                                                                                                                         |
| 2:0 (R/W)          | SYNC_MODE  | 3 Sync m:n                                                                                                                                         |
| 2:0 (R/W)          | SYNC_MODE  | 4 Async                                                                                                                                            |

## ADSP-SC59x SCB3 Register Descriptions

(SCB3) contains the following registers.

Table 48-175: ADSP-SC59x SCB3 Register List

| Name                        | Description                                          |
|-----------------------------|------------------------------------------------------|
| SCB3_APB_CLKO3_IB_SYNC_MODE | DDR MMRGFabric (CLK03) Synchronization Mode Register |
| SCB3_APB_CLKO4_IB_SYNC_MODE | DDR MMRGFabric (CLK04) Synchronization Mode Register |
| SCB3_APB_CLKO8_IB_SYNC_MODE | DDR MMRGFabric (CLK08) Synchronization Mode Register |

## DDR MMRG Fabric (CLK03) Synchronization Mode Register

Figure 48-173: SCB3\_APB\_CLKO3\_IB\_SYNC\_MODE Register Diagram

<!-- image -->

Table 48-176: SCB3\_APB\_CLKO3\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | SYNC_MODE  | Synchronization Mode. The SCB3_APB_CLKO3_IB_SYNC_MODE.SYNC_MODE bit field configures the SYSCLK-DCLK clock domain boundary (CDC) for the MMRGfabric. |
| 2:0 (R/W)          | SYNC_MODE  | 0 Sync 1:1                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 1 Sync n:1                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 2 Sync 1:n                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 3 Sync m:n                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 4 Async                                                                                                                                              |

## DDR MMRG Fabric (CLK04) Synchronization Mode Register

Figure 48-174: SCB3\_APB\_CLKO4\_IB\_SYNC\_MODE Register Diagram

<!-- image -->

Table 48-177: SCB3\_APB\_CLKO4\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | SYNC_MODE  | Synchronization Mode. The SCB3_APB_CLKO4_IB_SYNC_MODE.SYNC_MODE bit field configures the SYSCLK-DCLK clock domain boundary (CDC) for the MMRGfabric. |
| 2:0 (R/W)          | SYNC_MODE  | 0 Sync 1:1                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 1 Sync n:1                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 2 Sync 1:n                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 3 Sync m:n                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 4 Async                                                                                                                                              |

## DDR MMRG Fabric (CLK08) Synchronization Mode Register

Figure 48-175: SCB3\_APB\_CLKO8\_IB\_SYNC\_MODE Register Diagram

<!-- image -->

Table 48-178: SCB3\_APB\_CLKO8\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | SYNC_MODE  | Synchronization Mode. The SCB3_APB_CLKO8_IB_SYNC_MODE.SYNC_MODE bit field configures the SYSCLK-DCLK clock domain boundary (CDC) for the MMRGfabric. |
| 2:0 (R/W)          | SYNC_MODE  | 0 Sync 1:1                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 1 Sync n:1                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 2 Sync 1:n                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 3 Sync m:n                                                                                                                                           |
| 2:0 (R/W)          | SYNC_MODE  | 4 Async                                                                                                                                              |

## ADSP-SC59x SCB4 Register Descriptions

(SCB4) contains the following registers.

Table 48-179: ADSP-SC59x SCB4 Register List

| Name                             | Description                                     |
|----------------------------------|-------------------------------------------------|
| SCB4_FABRIC_ACC_MMR_IB_READ_QOS  | Fabric Acc MMRRead Quality of Service Register  |
| SCB4_FABRIC_ACC_MMR_IB_WRITE_QOS | Fabric Acc MMRWrite Quality of Service Register |
| SCB4_FABRIC_S1PORT_IB_READ_QOS   | Fabric S1PORT Read Quality of Service Register  |
| SCB4_FABRIC_S1PORT_IB_WRITE_QOS  | Fabric S1PORT Write Quality of Service Register |
| SCB4_FABRIC_S2PORT_IB_READ_QOS   | Fabric S2PORT Read Quality of Service Register  |
| SCB4_FABRIC_S2PORT_IB_WRITE_QOS  | Fabric S2PORT Write Quality of Service Register |
| SCB4_FIR_CH0_IB_READ_QOS         | FIR Channel 0 Read Quality of Service Register  |
| SCB4_FIR_CH0_IB_WRITE_QOS        | FIR Channel 0 Write Quality of Service Register |

Table 48-179: ADSP-SC59x SCB4 Register List (Continued)

| Name                       | Description                                      |
|----------------------------|--------------------------------------------------|
| SCB4_FIR_CH1_IB_READ_QOS   | FIR Channel 1 Read Quality of Service Register   |
| SCB4_FIR_CH1_IB_WRITE_QOS  | FIR Channel 1 Write Quality of Service Register  |
| SCB4_IIR0_CH0_IB_READ_QOS  | IIR0 Channel 0 Read Quality of Service Register  |
| SCB4_IIR0_CH0_IB_WRITE_QOS | IIR0 Channel 0 Write Quality of Service Register |
| SCB4_IIR0_CH1_IB_READ_QOS  | IIR0 Channel 1 Read Quality of Service Register  |
| SCB4_IIR0_CH1_IB_WRITE_QOS | IIR0 Channel 1 Write Quality of Service Register |
| SCB4_IIR1_CH0_IB_READ_QOS  | IIR1 Channel 0 Read Quality of Service Register  |
| SCB4_IIR1_CH0_IB_WRITE_QOS | IIR1 Channel 0 Write Quality of Service Register |
| SCB4_IIR1_CH1_IB_READ_QOS  | IIR1 Channel 1 Read Quality of Service Register  |
| SCB4_IIR1_CH1_IB_WRITE_QOS | IIR1 Channel 1 Write Quality of Service Register |
| SCB4_IIR2_CH0_IB_READ_QOS  | IIR2 Channel 0 Read Quality of Service Register  |
| SCB4_IIR2_CH0_IB_WRITE_QOS | IIR2 Channel 0 Write Quality of Service Register |
| SCB4_IIR2_CH1_IB_READ_QOS  | IIR2 Channel 1 Read Quality of Service Register  |
| SCB4_IIR2_CH1_IB_WRITE_QOS | IIR2 Channel 1 Write Quality of Service Register |
| SCB4_IIR3_CH0_IB_READ_QOS  | IIR3 Channel 0 Read Quality of Service Register  |
| SCB4_IIR3_CH0_IB_WRITE_QOS | IIR3 Channel 0 Write Quality of Service Register |
| SCB4_IIR3_CH1_IB_READ_QOS  | IIR3 Channel 1 Read Quality of Service Register  |
| SCB4_IIR3_CH1_IB_WRITE_QOS | IIR3 Channel 1 Write Quality of Service Register |
| SCB4_SHARC_DPORT_READ_QOS  | DPORT Read Quality of Service Register           |
| SCB4_SHARC_DPORT_WRITE_QOS | DPORT Write Quality of Service Register          |

## Fabric Acc MMR Read Quality of Service Register

Figure 48-176: SCB4\_FABRIC\_ACC\_MMR\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-180: SCB4\_FABRIC\_ACC\_MMR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## Fabric Acc MMR Write Quality of Service Register

Figure 48-177: SCB4\_FABRIC\_ACC\_MMR\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-181: SCB4\_FABRIC\_ACC\_MMR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## Fabric S1PORT Read Quality of Service Register

Figure 48-178: SCB4\_FABRIC\_S1PORT\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-182: SCB4\_FABRIC\_S1PORT\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## Fabric S1PORT Write Quality of Service Register

Figure 48-179: SCB4\_FABRIC\_S1PORT\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-183: SCB4\_FABRIC\_S1PORT\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## Fabric S2PORT Read Quality of Service Register

Figure 48-180: SCB4\_FABRIC\_S2PORT\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-184: SCB4\_FABRIC\_S2PORT\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## Fabric S2PORT Write Quality of Service Register

Figure 48-181: SCB4\_FABRIC\_S2PORT\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-185: SCB4\_FABRIC\_S2PORT\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## FIR Channel 0 Read Quality of Service Register

Figure 48-182: SCB4\_FIR\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-186: SCB4\_FIR\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## FIR Channel 0 Write Quality of Service Register

Figure 48-183: SCB4\_FIR\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-187: SCB4\_FIR\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## FIR Channel 1 Read Quality of Service Register

Figure 48-184: SCB4\_FIR\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-188: SCB4\_FIR\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## FIR Channel 1 Write Quality of Service Register

Figure 48-185: SCB4\_FIR\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-189: SCB4\_FIR\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR0 Channel 0 Read Quality of Service Register

Figure 48-186: SCB4\_IIR0\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-190: SCB4\_IIR0\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR0 Channel 0 Write Quality of Service Register

Figure 48-187: SCB4\_IIR0\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-191: SCB4\_IIR0\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR0 Channel 1 Read Quality of Service Register

Figure 48-188: SCB4\_IIR0\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-192: SCB4\_IIR0\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR0 Channel 1 Write Quality of Service Register

Figure 48-189: SCB4\_IIR0\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-193: SCB4\_IIR0\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR1 Channel 0 Read Quality of Service Register

Figure 48-190: SCB4\_IIR1\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-194: SCB4\_IIR1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR1 Channel 0 Write Quality of Service Register

Figure 48-191: SCB4\_IIR1\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-195: SCB4\_IIR1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR1 Channel 1 Read Quality of Service Register

Figure 48-192: SCB4\_IIR1\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-196: SCB4\_IIR1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR1 Channel 1 Write Quality of Service Register

Figure 48-193: SCB4\_IIR1\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-197: SCB4\_IIR1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR2 Channel 0 Read Quality of Service Register

Figure 48-194: SCB4\_IIR2\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-198: SCB4\_IIR2\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR2 Channel 0 Write Quality of Service Register

Figure 48-195: SCB4\_IIR2\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-199: SCB4\_IIR2\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR2 Channel 1 Read Quality of Service Register

Figure 48-196: SCB4\_IIR2\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-200: SCB4\_IIR2\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR2 Channel 1 Write Quality of Service Register

Figure 48-197: SCB4\_IIR2\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-201: SCB4\_IIR2\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR3 Channel 0 Read Quality of Service Register

Figure 48-198: SCB4\_IIR3\_CH0\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-202: SCB4\_IIR3\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR3 Channel 0 Write Quality of Service Register

Figure 48-199: SCB4\_IIR3\_CH0\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-203: SCB4\_IIR3\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## IIR3 Channel 1 Read Quality of Service Register

Figure 48-200: SCB4\_IIR3\_CH1\_IB\_READ\_QOS Register Diagram

<!-- image -->

Table 48-204: SCB4\_IIR3\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## IIR3 Channel 1 Write Quality of Service Register

Figure 48-201: SCB4\_IIR3\_CH1\_IB\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-205: SCB4\_IIR3\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## DPORT Read Quality of Service Register

Figure 48-202: SCB4\_SHARC\_DPORT\_READ\_QOS Register Diagram

<!-- image -->

Table 48-206: SCB4\_SHARC\_DPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | Ar Qos.                   |
| (R/W)              |            |                           |

## DPORT Write Quality of Service Register

Figure 48-203: SCB4\_SHARC\_DPORT\_WRITE\_QOS Register Diagram

<!-- image -->

Table 48-207: SCB4\_SHARC\_DPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | Aw Qos.                   |
| (R/W)              |            |                           |

## ADSP-SC59x SCB5 Register Descriptions

System Crossbars (SCB5) contains the following registers.

Table 48-208: ADSP-SC59x SCB5 Register List

| Name                 | Description                                 |
|----------------------|---------------------------------------------|
| SCB5_SPI2_OSPI_REMAP | SPI2/OSPI Memory Map Address Remap Register |

## SPI2/OSPI Memory Map Address Remap Register

Figure 48-204: SCB5\_SPI2\_OSPI\_REMAP Register Diagram

<!-- image -->

Table 48-209: SCB5\_SPI2\_OSPI\_REMAP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------|
| 1:0                | REMAP      | .                                                                                                         |
| (RX/W)             |            | 0 Selects map for SPI2 only 0x6000_0000 - 0x7FFF_FFFF - SPIF                                              |
|                    |            | 1 Selects map for OSPI only 0x6000_0000 - 0x7FFF_FFFF - OSPI                                              |
|                    |            | 2 Selects map for SPI2/OSPI split sharing 0x6000_0000 - 602F_FFFF - SPIF 0x6030_0000 - 0x7FFF_FFFF - OSPI |

## ADSP-SC59x SCB6 Register Descriptions

(SCB6) contains the following registers.

Table 48-210: ADSP-SC59x SCB6 Register List

| Name                  | Description      |
|-----------------------|------------------|
| SCB6_A55_M0_IB_FN_MOD | A55 M0 Ib.fn Mod |

## A55 M0 Ib.fn Mod

Figure 48-205: SCB6\_A55\_M0\_IB\_FN\_MOD Register Diagram

<!-- image -->

Table 48-211: SCB6\_A55\_M0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | Fn Mod.                   |
| (R/W)              |            |                           |