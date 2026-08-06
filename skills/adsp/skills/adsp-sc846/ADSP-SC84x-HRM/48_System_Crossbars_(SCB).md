## 45   System Crossbars (SCB)

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

Figure 45-1: SCB Overview

![Image](48_System_Crossbars_(SCB)_artifacts/image_000000_0701d6eafccac823de0fc58e18bc031a22b13234d921f859d76f5f8cc15df665.png)

## Hierarchy Block Diagram

A system interconnect built from multiple SCBs in a hierarchical model is illustrated in the SCB Hierarchy Overview figure. The system requester node level SCBs requester connects multiple SIs to a single MI, which in turn connects to an SI of the system completer level node SCB.

As discussed above, all the requesters in the system are distributed across different SCBs. A given SCB at system requester node level connects directly to the system requesters. These SCBs connect to SCB0 through its SIs forming a hierarchical structure. While a requester must access any completer, its first access goes through the SCB it is connected to, and then through SCB0, to reach its intended completer. This simplifies the connecting hardware in the basic SCB block by limiting the requesters. Care must be taken when sharing requesters to allow adequate throughput for their individual data transfer requirements.

In this example, all SIs are connected to all MIs.

Figure 45-2: SCB Hierarchy Overview

NOTE: For an overall diagram of all SCB interconnections, see the SCB Block Diagram.

## SCB Block Diagram

The SCB Block Diagram shows the functional blocks of the SCB module.

![Image](48_System_Crossbars_(SCB)_artifacts/image_000001_3cba6838b66af1041cc8da5c422f4c26758e7d9d15c3832aab1ff7c7aa7957f3.png)

Figure 45-3: SCB Block Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000002_4bec85c4ffe7a9f4b3c15c9d9f01794f6d579ad0c7751279b92cf01097dee5e6.png)

The following are important points related to the system fabric on the ADSP-SC84x processors.

- The hierarchy of SCBs manages the system bus interconnections, multiplexing, and arbitration among the peripherals on the processor.
- The SCBs connections support DMA channels for some peripherals and support dedicated connections for others. The connections also support memory-mapped register access for internal memory (L1 and L2) and for external memory.

The completer interface of the crossbar where requesters such as DMA connect to performs two functions. The first function is arbitration. SCBx handles arbitration. The second function is clock conversion. The programmable QoS registers can be viewed as being associated with the SCBx.

- Most of the peripherals and the SCBs are in the SCLK0 domain. MDMA0-7, EMDMA0/1, Crypto, FIR/IIR accelerators, MLB, DBG, ETR, SHARCFX and Arm cores and their SCBs are in the SYSCLK domain. The link port is in the CLK08 domain. SPI is in the CLK06 domain.
- MSMDMA0, MSMDMA1, HSMDMA0 and HSMDMA1 channels are unidirectional. For example, MSMDMA0 CH0 is read only and MSMDMA0 CH1 is write only.
- Access to the GPV (Global Programmers View) space is allowed only in secure mode. This space has registers for programming the QoS and CDC relationships and the remapping of various requesters.
- The S2 completer port of SHARC0 is intended primarily for MDMA. Hence, only the HSMDMA0 (MDMA3) and HSMDMA1 (MDMA7) is given access to S2 ports while all other requesters are given access to S1 port. This arrangement also means that S1 and S2 port share the same address space.
- LP to SPI2/xSPI access is allowed.

IMPORTANT: The following points are important changes from ADSP-2159x processors:

- SHARC-FX DPORT accesses L2 RAM and ROM over CL2\_0 and CL2\_1 are through a dedicated requester interface (SHARC0 DPORT L2); accesses to the rest of the system are over SHARC0 DPORT L2.
- Other new requesters are iDMA (from FX core), A55 data requester ports (a55\_m0s\_axi, a55\_m1\_axi), A55 MMR requester port (a55\_mmr), SPI5(TX), SPI5(RX), xSPI requester port (xspi\_m, xspi\_m1), HSM, and eMSI.
- All requesters that can access DMC have been given access to SPI/xSPI.
- The SPIF completer in the system fabric has remapping options in the SPIF fabric for SPI[n] and xSPI in the same address space. The address map configuration is programmable using the remap registers.
- SH0\_FIR\_CH0 is moved to the HSMDMA SCB, and access to L2 is through dl2\_0.

Figure 45-4: SCB Interconnections

The following acronyms are used in the SCB Interconnections figure.

## SCB0-10

Indicate SCB interfaces, connecting the system bus requesters and completers

## SCLK0, SYSCLK, CLK02, CLK06, CLK08

Indicate clock domains in which the specific SCBs operate. For more information on clock domains, see the Clock Generation Unit (CGU) chapter and the product data sheet.

![Image](48_System_Crossbars_(SCB)_artifacts/image_000003_7268572e593cb1ec71ef146fe500302092d3bdb596eb24a0baa993c15f2e78a5.png)

## iDMA

Indicates the integrated DMA in the SHARC-FX core

## xSPI

## A55

Indicates the ARM Coretex A55 microprocessor

## Hardware Security Module CMRT-641

Indicates a 128-bit L2 completer

## DL2\_0, DL2\_1

Indicates a 128-bit L2 completer

## L1C0\_S1

Indicates the S1 completer of the SHARC0 processor

## L1C0\_S2

Indicates the S2 completer of the SHARC0 processor

## SHARC0\_DPORT

Indicates the SHARC0 processor instruction requester port

## SHARC0\_IPORT

Indicates the SHARC0 processor instruction requester port

## SHARC0\_MMR

Indicates the SHARC0 processor MMR interface

## MSMDMA

Indicates medium speed memory to memory DMA

Indicates the SPI hyperbus

## HSMDMA

Indicates high speed memory to memory DMA

## SHARC0, SHARC, SHARCFX

Indicates the SHARC-FX core (used interchangeably)

## QoS

Indicates quality of service

## SHARC Fabric with FIR/IIR Accelerator

The accelerator block on the SHARC core of the processor has two FIR and four IIR accelerators. All instances of the FIR and IIR accelerators operate at the core clock (CCLK) frequency. Accelerator requester ports can directly access the SHARC L1 memory with reduced latency. The access does not go through the system fabric. The SHARC core can also directly access the MMR registers of its accelerator.

The SHARC Fabric Connectivity figure shows a block diagram of the SHARC fabric with FIR/IIR accelerators integrated closely with the SHARC core.

Figure 45-5: SHARC Fabric Connectivity

![Image](48_System_Crossbars_(SCB)_artifacts/image_000004_553b41744083573d66dd89038a77969ccef7594ca7ecb105040af89f6a4ee317.png)

## Port list and Parameters

Requester Ports (AXI-M)

- Data-Requester

Inst-Requester

- iDMA

Completer Ports (AXI-S)

- DRAM-Completer
- IRAM-Completer

To ensure complete bandwidth utilization and optimal performance of the L2 ports, the controllers are allocated as shown in L2 Port Requester Allocation table.

Table 45-1: L2 Port Requester Allocation

| Requesters     | CL2_0   | CL2_1   | CL2_2   | DL2_0   | DL2_1   |
|----------------|---------|---------|---------|---------|---------|
| SHARC0 DPORT   | ✓       | ✓       |         |         |         |
| SHARC0 iDMA    | ✓       | ✓       |         |         |         |
| SHARC0 IPORT   |         |         | ✓       |         |         |
| A55 M0 AXI     |         | ✓       |         |         |         |
| A55 M1 AXI     | ✓       |         |         |         |         |
| HSM            | ✓       | ✓       |         |         |         |
| SHARC0_FIR_CH0 |         |         |         | ✓       |         |
| SHARC0_FIR_CH1 |         |         | ✓       |         |         |
| SHARC0_IIR_CH0 |         |         | ✓       |         |         |
| SHARC0_IIR_CH1 |         |         | ✓       |         |         |
| HSMDMA0        |         |         |         | ✓       |         |
| HSMDMA1        |         |         |         | ✓       |         |
| Peripherals    |         |         |         |         | ✓       |

## Note the following:

- To reduce the probability of port conflict in a multicore system, FX, A55, HSM, and iDMA are connected on the CL2\_0 and CL2\_1 ports of L2. CL2\_0 has access only to the first 2MB of L2. CL2\_1 has access to second 2MB of L2
- For these requesters, access to the initial 2MB of L2 RAM and ROM goes through the CL2\_0 port; the later 2MB of L2 RAM goes through the CL2\_1 port
- SHARC-FX IPORT and Acceelerators (FIRs and IIRs) are connected over CL2\_2 port.
- To increase FIR B/w from L2, FIR\_CH0 is connected to DL2\_0.

The addressable range for each of the ports is shown in the L2 Port Address Allocation table. SMPU 2 and 4 support exclusive access over CL2\_0 and CL2\_1 respectively, between SHARC FX DPORT and A55.

Table 45-2: L2 Port Address Allocation

| SMPU Instance   | Memory Address Range   | Memory Address Range   | EX- Acc   | Comments   |
|-----------------|------------------------|------------------------|-----------|------------|
|                 | Start Address          | End Address            |           |            |
| CL2_0           | 0x2020_0000            | 0x2022_FFFF            | Yes       | ROM        |
|                 | 0x2040_0000            | 0x205F_FFFF            | Yes       | RAM        |
| DL2_0           | 0x2000_0000            | 0x2022_FFFF            | No        | ROM        |
|                 | 0x2040_0000            | 0x207F_FFFF            | No        | RAM        |
| CL2_1           | 0x2060_0000            | 0x207F_FFFF            | Yes       | RAM        |
| CL2_2           | 0x2020_0000            | 0x2022_FFFF            | No        | ROM        |
|                 | 0x2040_0000            | 0x207F_FFFF            | No        | RAM        |
| DL2_1           | 0x2020_0000            | 0x2022_FFFF            | No        | ROM        |
|                 | 0x2040_0000            | 0x207F_FFFF            | No        | RAM        |

Table 45-3: SCB Controlled DMA Channel Peripherals

|      | Requesters     | DDE DMAChannels   | Non DDE DMAChannels   |
|------|----------------|-------------------|-----------------------|
| SCB1 | SPORT0, HALF A | DMA0              |                       |
|      | SPORT0, HALF B | DMA1              |                       |
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

Table 45-3: SCB Controlled DMA Channel Peripherals (Continued)

|            | Requesters        | DDE DMAChannels   | Non DDE DMAChannels           |
|------------|-------------------|-------------------|-------------------------------|
|            | UART0, RX         | DMA21             |                               |
|            | UART1, TX         | DMA34             |                               |
|            | UART1, RX         | DMA35             |                               |
|            | UART2, TX         | DMA37             |                               |
|            | UART2, RX         | DMA38             |                               |
| SCB4       | SPI0, TX          | DMA22             |                               |
|            | SPI0, RX          | DMA23             |                               |
|            | SPI1, TX          | DMA24             |                               |
|            | SPI1, RX          | DMA25             |                               |
|            | SPI2, TX          | DMA26             |                               |
|            | SPI2, RX          | DMA27             |                               |
|            | SPI5, TX          | DMA59             |                               |
|            | SPI5, RX          | DMA60             |                               |
| SCB6       | LP0               | DMA30             |                               |
|            | LP1               | DMA36             |                               |
| SCB7       | CRC0/MDMA0        | DMA8, DMA9        |                               |
|            | CRC1/MDMA1        | DMA18, DMA19      |                               |
| SCB8       | CRC2/MDMA4        | DMA45, DMA46      |                               |
|            | CRC3/MDMA5        | DMA47, DMA48      |                               |
| SCB9       | MDMA2             | DMA39, DMA40      |                               |
|            | MDMA6             | DMA49, DMA50      |                               |
| SCB10      | ETR               | N/A               | N/A                           |
|            | CRYPTO            | N/A               | 2 channels ( 1 read, 1 write) |
|            | EMDMA0            | N/A               | 2 channels ( 1 read, 1 write) |
|            | EMDMA1            | N/A               | 2 channels ( 1 read, 1 write) |
| HSMDMA_SCB | MDMA3             | DMA43, DMA44      |                               |
|            | MDMA7             | DMA51, DMA52      |                               |
|            | SH0 FIR CH0       |                   |                               |
| CORES_SCB  | A55 M0 Controller | N/A               | N/A                           |
|            | A55 M1 Controller | N/A               | N/A                           |
|            | iDMA              | N/A               | TBD                           |

Table 45-3: SCB Controlled DMA Channel Peripherals (Continued)

|               | Requesters   | DDE DMAChannels   | Non DDE DMAChannels   |
|---------------|--------------|-------------------|-----------------------|
|               | SH0 DPORT    | N/A               | N/A                   |
|               | SH0 DPORT L2 | N/A               | N/A                   |
|               | HSM          | N/A               | N/A                   |
| FIR_IPORT_SCB | SH0 FIR CH1  | N/A               | CH1                   |
| FIR_IPORT_SCB | SH0 IIR CH0  | N/A               | CH0                   |
| FIR_IPORT_SCB | SH0 IIR CH1  | N/A               | CH1                   |
| FIR_IPORT_SCB | SH0 IPORT    | N/A               | N/A                   |
| MMR_SCB       | SHARC0MMR    | N/A               | N/A                   |
| MMR_SCB       | A55_MRR      | N/A               | N/A                   |

There are two types of peripherals that use DMA. The first have dedicated DMA channels controlled by the Dedicated DMA Engine (DDE) and have the same operating modes (see DMA Operating Modes) and use the same programming model (DMA Channel Programming Model). The second type is not controlled by the DDE module. These peripherals have their own operating modes and programming models (see the peripheral chapter for this information). The peripheral types are shown in the SCB-Controlled DMA Channel Peripherals table.

## Address Remapping for Backwards Compatibility

To maintain backwards compatibility of the address map to the 21xxx processors, address remapping must occur at the integration level of the SHARC-FX core in SHARC + subsystems.

## Address Translation for IPORT to L2 (Boot) ROM 0 and 1

The SHARC-FX reset vector 0x0010\_0000 is mapped to the L2 (Boot) ROM. Native addresses issued by the IPORT are remapped/translated to reach the L2 ROM as shown in the following table.

Table 45-4: IPORT to L2 ROM Address Translation

| Memory          | Native Address Map   | Native Address Map   | Converted Address Map   | Converted Address Map   |
|-----------------|----------------------|----------------------|-------------------------|-------------------------|
|                 | Start                | End                  | Start                   | End                     |
| Boot ROM0(64kB  | 0010_0000            | 0010_FFFF            | 2020_0000               | 2020_FFFF               |
| Boot ROM1(64kB) | 0011_0000            | 0011_FFFF            | 2021_0000               | 2021_FFFF               |

## S1/S2 to IRAM/DRAM Mapping

Other requesters in a system can access L1 over the bus completer port of the core. The SHARC+ core has two bus completer ports: S1 and S2. The system requester can access the entire L1 over either S1 or S2 port (depending on the grouping). In comparison, the SHARC-FX core has two bus completer ports, I-completer and D-completer, that allow and limit access to IRAM and DRAM, respectively. A bridge has been implemented to allow all system

requesters to access both IRAM and DRAM over I-completer and D-completer. The L1 Access Bridge Routing table shows the routing source to destination through the bridge. The X Bridge to Map Fabric figure shows the S1/S2 and accelerator access to IRAM and DRAM through the bridge.

NOTE: Accelerator accesses are also routed through the bridge.

Table 45-5: L1 Access Bridge Routing

| Source   | Range (SYS_ADDR)   | Range (SYS_ADDR)   | Destination      |
|----------|--------------------|--------------------|------------------|
| S1       | 0x28240000         | 0x282BFFFF         | DRAM/D-Completer |
| S1       | 0x282C0000         | 0x282CFFFF         | IRAM/I-Completer |
| S2       | 0x28240000         | 0x282BFFFF         | DRAM/D-Completer |
| S2       | 0x282C0000         | 0x282CFFFF         | IRAM/I-Completer |

![Image](48_System_Crossbars_(SCB)_artifacts/image_000005_6952fe906ac617280a08b7e7fd53911eb11e9ce83ccb155ca716e9f8ec9187ff.png)

SHARC FABRIC

Figure 45-6: X-Bridge to Map Fabric

## SHARC-FX Bus Completer Port Remapping

The SHARC-FX core accepts only its native address map on its bus completer port. To maintain SHARC+ backwards compatibility of the L1 bus completer address or multi-processor address, an address remapping or offset adjustment occurs on the incoming addresses to I-completer and D-completer before being forwarded to the SHARC-FX core. The Bus Completer Port Address Remapping table shows the remapping of addresses.

Table 45-6: Bus Completer Port Address Remapping

| Destination      | SYS_ADDR/MP_ADDR   | SYS_ADDR/MP_ADDR   | Mapped Native ADDR   | Mapped Native ADDR   |
|------------------|--------------------|--------------------|----------------------|----------------------|
| DRAM/D-Completer | 0x28240000         | 0x282BFFFF         | 0x2F780000           | 0x2F7FFFFF           |
| IRAM/I-Completer | 0x282C0000         | 0x282CFFFF         | 0x2F800000           | 0x2F80FFFF           |

- NOTE: · Byte and half word accesses to IRAM are not allowed by either the core or DMAs (over I-completer). Only 32-, 64- and 128-bit accesses are allowed. If an 8- or 16-bit access is attempted:
- the core hits an exception
- DMAs over I-completer do not receive an error response
- There are no restrictions for DRAM; 8-, 16-, 32-, 64- and128-bit accesses are allowed.
- Unaligned accesses to IRAM are not allowed; the address must always be 32-bit aligned. Unaligned 64- and 128-bit accesses around the 32-bit boundary are valid. The address does not need to be 64or 28-bit aligned.
- Unaligned accesses to DRAM are allowed for any size.

## Port Security

The SHARC-FX core has three requester ports: the instruction requester port, the data requester port, and iDMA. These ports interface with the rest of the system. The SHARC-FX core does not assign any security privileges on accesses from its requester ports. It uses SPU-based security privilege control.

## Bus Completer Port Security

The SHARC-FX core has two bus completer ports: IRAM and DRAM. Requesters in the system use these ports to access the IRAM and DRAM of the SHARC-FX processor. These ports interface with the rest of the system. The security privileges of incoming accesses from other requesters in the system are overwritten or passed. The bus completer port security is based on the security privilege of the core (core SPU settings). As secure accesses pass the first stage and enter the second stage, the addresses of incoming accesses are remapped from the system address to the allowed address range supported by SHARC-FX core. If the accesses are not secure, the SHARC-FX processor generates an error response due to an invalid address.

## System Crossbars

The System Crossbars (SCB) are the fundamental building blocks of the system bus interconnect. The SCB (often referred to as the system interconnect fabric), is a collection of inter-connection units connecting system requesters to completer memory spaces. The SCB connects one or more requester devices to one or more memory-mapped completer devices. Each connected requester can be a core that originates an SCB transaction, or a requester interface of an upstream SCB cascaded interconnect. Each connected completer can be the final completer of an SCB transaction or a completer interface of a downstream cascaded SCB interconnect (forming a hierarchy of SCBs).

Each SCB that has multiple requesters and completers share the total bandwidth of the SCB. (In a M:N configuration where M requesters are connected to N completers through the SCBx.)

The SCB provides separate channels for reads and writes. Read and write accesses through a given SCB do not share bandwidth. Only SCB0, which is the major SCB in the SCB hierarchy, has the multiple paths between multiple requester and completer interfaces.

All other SCBs in the chip connect to SCB0 through different completer interfaces. Other primary requesters (DMAs, cores, and so on) in the system are distributed across these small SCBs. For a given SCB, all the requester and completers share the total bandwidth of the SCB. (Only SCB0 is the exception). Since different DMA channels are scattered across different SCBs (SCB1, SCB2 SCB3, and so on), they do not conflict for the bandwidth when they are in different SCBs and accessing different completers. SCB0 allows for concurrent data transfer between multiple bus requesters and multiple bus completers, providing flexibility, and full-duplex operation.

If system accesses are carefully architected, SCB has a potential of providing sufficient sustained bandwidth in the end system.

Since the SCBs support burst transfers, it is important to configure the requesting requester appropriately to make best use of available SCB bandwidth. For a DMA requester, choosing the appropriate DMA\_CFG.MSIZE value, is important from both a functional and a performance perspective. The value in the DMA\_CFG.PSIZE bit field determines the width of the peripheral bus in use. It can be configured to 1-byte, 2-bytes, or 4-bytes. The DMA\_CFG.MSIZE value determines the actual size of the SCB bus in use. It also determines the minimum number of bytes which are transferred from or to memory corresponding to a single DMA request or grant. The transfer can be 1-, 2-, 4-, 8-, 16-, or 32-bytes. If the DMA\_CFG.MSIZE value is greater than the SCB bus width, the SCB performs burst transfers according to the width defined in DMA\_CFG.MSIZE . When DMA\_CFG.MSIZE is less than the SCB bus width, bursting is not supported and partial bus use results.

Each of the SCB unit in the fabric consists of N completer interfaces (MSTn). Each of these interfaces has controls for read quality of service, write quality of service, and functional mode. A subset of these matrices includes controls for IB (Interface Block) sync mode, and bus functional mode. For more details on IB, see the clock domain synchronization section.

## SCB Bus Requester IDs

The SCB Bus Requester IDs table indicates which requesters are connected to each of the completer ports of SCB0. The tables also indicate the precise value of the ID as seen by the completer. These values are useful for SWU programming.

NOTE: For an overall diagram of all SCB interconnections, see the SCB Block Diagram.

Table 45-7: Bus Requester IDs

| Requester             | Hex ID Values   | Binary Values     |
|-----------------------|-----------------|-------------------|
| DMA0 (SPORT0, HALF A) | 0x0000, 0x0100  | 13'b0000x00000000 |
| DMA1 (SPORT0, HALF B) | 0x0010, 0x0090  | 13'b00000x0010000 |

Table 45-7: Bus Requester IDs (Continued)

| Requester                  | Hex ID Values   | Binary Values     |
|----------------------------|-----------------|-------------------|
| DMA2 (SPORT1, HALF A)      | 0x0020, 0x00A0  | 13'b00000x0100000 |
| DMA3 (SPORT1, HALF B)      | 0x0030, 0x00B0  | 13'b00000x0110000 |
| DMA4 (SPORT2, HALF A)      | 0x0040, 0x00C0  | 13'b00000x1000000 |
| DMA5 (SPORT2, HALF B)      | 0x0050, 0x00D0  | 13'b00000x1010000 |
| DMA6 (SPORT3, HALF A)      | 0x0060, 0x00E0  | 13'b00000x1100000 |
| DMA7 (SPORT3, HALF B)      | 0x0070, 0x00F0  | 13'b00000x1110000 |
| DMA8 (Enh BWMDMA0CRC, CH0) | 0x0031, 0x00B1  | 13'b00000x0110001 |
| DMA9 (Enh BWMDMA0CRC, CH1) | 0x0040, 0x00C1  | 13'b00000x0100001 |
| MLB                        | 0x0042          | 13'b0000001000010 |
| DMA20 (UART0, TX)          | 0x0003, 0x0083  | 13'b00000x0000011 |
| DMA21 (UART0, RX)          | 0x0043, 0x00C3  | 13'b00000x1000011 |
| DMA37 (UART2, TX)          | 0x0033, 0x00B3  | 13'b00000x0110011 |
| DMA22 (SPI0, TX)           | 0x0004, 0x0084  | 13'b00000x0000100 |
| DMA23 (SPI0, RX)           | 0x0014, 0x0094  | 13'b00000x0010100 |
| DMA24 (SPI1, TX)           | 0x0024, 0x0A4   | 13'b00000x0100100 |
| DMA25 (SPI1, RX)           | 0x0034, 0x0B4   | 13'b00000x0110100 |
| DMA26 (SPI2, TX)           | 0x0054, 0x00D4  | 13'b00000x1010100 |
| DMA27 (SPI2, RX)           | 0x0044, 0x00C4  | 13'b00000x1000100 |
| DMA30 (LP0)                | 0x0005, 0x0085  | 13'b00000x0000101 |
| DMA34 (UART1, TX)          | 0x0013, 0x0093  | 13'b00000x0010011 |
| DMA35 (UART1, RX)          | 0x0023, 0x00A3  | 13'b00000x0100011 |
| DMA36 (LP1)                | 0x0015, 0x0095  | 13'b00000x0010101 |
| CRYPTO                     | 0x0056          | 13'b0000001010110 |
| SH0_FIR_CH0                |                 | 13'b00xxxx0000111 |
| SH0_FIR_CH1                |                 | 13'b00xxxx1011010 |
| EMDMA0 (CH0)               | 0x0006          | 13'b0000000000110 |
| EMDMA0 (CH1)               | 0x0016          | 13'b0000000010110 |
| EMDMA1 (CH0)               | 0x0026          | 13'b0000000100110 |
| EMDMA1 (CH1)               | 0x0036          | 13'b0000000110110 |
| DMA39 (Enh BWMDMA2, CH0)   | 0x0008, 0x0088  | 13'b00000x0001000 |
| DMA40 (Enh BWMDMA2, CH1)   | 0x0018, 0x098   | 13'b00000x0011000 |

Table 45-7: Bus Requester IDs (Continued)

| Requester                       | Hex ID Values   | Binary Values     |
|---------------------------------|-----------------|-------------------|
| DBG                             | 0x0048          | 13'b0000001001000 |
| ETR                             | 0x0046          | 13'b0000001000110 |
| DMA18 (Enh BWMDMA1CRC1, CH0)    | 0x0011, 0x0091  | 13'b00000x0010001 |
| DMA19 (Enh BWMDMA1CRC1, CH1)    | 0x0001, 0x0081  | 13'b00000x0000001 |
| DMA38 (UART2, RX)               | 0x0053, 0x00D3  | 13'b00000x1010011 |
| SH0 (DPORT)                     |                 | 13'bxxxxxx0001001 |
| SH0 (IPORT)                     |                 | 13'b00xxxx0100111 |
| DMA43 (High Speed MDMA3, CH0)   | 0x000A, 0x008A  | 13'b00000x0001010 |
| DMA44 (High Speed BWMDMA3, CH1) | 0x001A, 0x009A  | 13'b00000x0011010 |
| DMA10 (SPORT4, HALFA)           | 0x000B, 0x008B  | 13'b00000x0001011 |
| DMA11 (SPORT4, HALFB)           | 0x001B, 0x009B  | 13'b00000x0011011 |
| DMA12 (SPORT5, HALFA)           | 0x002B, 0x00AB  | 13'b00000x0101011 |
| DMA13 (SPORT5, HALFB)           | 0x003B, 0x00BB  | 13'b00000x0111011 |
| DMA14 (SPORT6, HALFA)           | 0x004B, 0x00CB  | 13'b00000x1001011 |
| DMA15 (SPORT6, HALFB)           | 0x005B, 0x00DB  | 13'b00000x1011011 |
| DMA16 (SPORT7, HALFA)           | 0x006B, 0x00EB  | 13'b00000x1101011 |
| DMA17 (SPORT7, HALFB)           | 0x007B, 0x00FB  | 13'b00000x1111011 |
| SH0_MMR                         |                 | 13'bxxxxxx0001100 |
| DMA45 (Enh BWMDMA4CRC, CH0)     | 0x001D, 0x009D  | 13'b00000x0011101 |
| DMA46 (Enh BWMDMA4CRC, CH1)     | 0x000D, 0x008D  | 13'b00000x0001101 |
| DMA47 (Enh BWMDMA5CRC, CH0)     | 0x003D, 0x0BD   | 13'b00000x0111101 |
| DMA48 (Enh BWMDMA5CRC, CH1)     | 0x002D, 0x0AD   | 13'b00000x0101101 |
| DMA49 (Enh BWMDMA6CRC1, CH0)    | 0x003D, 0x00BD  | 13'b00000x0111000 |
| DMA50 (Enh BWMDMA6CRC1, CH1)    | 0x0028 0x00A8   | 13'b00000x0101000 |
| DMA51 (High Speed MDMA3, CH0)   | 0x002A, 0x00AA  | 13'b00000x0101010 |
| DMA52 (High Speed BWMDMA3, CH1) | 0x003A, 0x00BA  | 13'b00000x0111010 |
| SH0_IIR_CH0                     |                 | 13'b00xxxx1000111 |
| SH0_IIR_CH1                     |                 | 13'b00xxxx0110111 |
| iDMA                            |                 | 13'b00xxxx0011001 |
| XSPI M0                         | 0x0052          | 13'b0000001010010 |
| A55 M1 AXI                      | 0x0029          | 13'b0xxxxx0101001 |

Table 45-7: Bus Requester IDs (Continued)

| Requester        | Hex ID Values   | Binary Values     |
|------------------|-----------------|-------------------|
| GIGE0            |                 | 13'b00xxxx0011110 |
| DMA57 (SPI5, TX) | 0x0064, 0x00E4  | 13'b00000x1100100 |
| DMA60 (SPI5, RX) | 0x0074, 0x00F4  | 13'b00000x1110100 |
| SH0_DPORT_L2     |                 | 13'bxxxxxx0111001 |
| GIGE1            |                 | 13'b00xxxx0101110 |
| XSPI M1          |                 | 13'b0000001100010 |
| EMMC             |                 | 13'b00000x0111110 |
| HSM              |                 | 13'b00xxxx1001001 |
| A55 M0 AXI       |                 | 13'b0xxxxx1011001 |
| SH0_FIR_CH0      |                 | 13'b00xxxx1011010 |
| A55_MMR          |                 | 13'b0xxxxx1101001 |

## SCB Programming Model

The following sections provide information for programming the SCB properly.

## Programming SCB Arbitration

Each completer interface has a QoS value (priority) associated with both read and write channels. These values are 4 bits present in the SCB0\_MSTx\_RQOS and SCB0\_MSTx\_WQOS registers. At the entry point to the infrastructure, all transactions are allocated this programmable local QoS value. The arbitration of the transaction throughout the infrastructure uses this QoS. At any arbitration node, a fixed priority exists for transactions with a different QoS. The highest value has the highest priority.

If there are coincident transactions at an arbitration node with the same QoS that require arbitration, then the network uses a Least Recently Granted (LRG) algorithm. At each switch, the requester with the highest QoS acquires access and that switch output takes the QoS value of the winner for that transaction. At the next switch completer interface, the requester uses the QoS value of the winner. QoS can have values from 0 (lowest priority) to 15 (highest priority).

For example in the following figure, SCB Arbitration :

1. At SCB1, requesters (1, 2, 3) have RQOS values of (6, 4, 2)
2. At SCB2, requesters (4, 5, 6) have RQOS values of (12, 13, 1)

Figure 45-7: SCB Arbitration

In this case, requester 1 wins at SCB1, and requester 5 wins at SCB2. However, in a perfect competition at SCB0, requesters 4 and 5 had the highest overall RQOS values. requesters 4 and 5 would have fought for arbitration directly at SCB0. However, because of the mini*SCBs, requester 1, at a much lower RQOS value, is able to win against requester 4 and make it all the way to SCB0.

## Programming Clock Domain Crossing Registers

The Clock Domain Crossing Options table shows the various clock domain crossings that are available on the processor. The clocking relationships are defined with respect to SYSCLK. The followng clock domains are programmable: CLKO3: SYSCLK, CLKO4: SYSCLK and CLKO8: SYSCLK CDC. The registers to configure the CDC mode are present within the GPV space of the respective fabric. For example: To program the CLKO3: SYSCLK relationship, program the GPV registers within the DMC\_CDC fabric and the MMRG fabric.

Table 45-8: Clock Domain Crossing Options

|                   |                                            | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   | Clocking Relationships with respect to SYSCLK   |
|-------------------|--------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|
| Functional Clocks | Possible Com- pleters                      | System Fabric                                   | MMRGFabric                                      | DMCFabric                                       | SHARC Fabric                                    | Arm Fabric                                      | SPIF Fabric                                     |
| SYSCLK            | System Fabric and Infrastruc- ture Modules | 1:1                                             | 1:1                                             | *1                                              | *1                                              | *1                                              | *1                                              |
| SCLK0             | SCLK0 Clock Doamin                         | Synchronous (m:1)                               | Synchronous (m:1)                               | *1                                              | *1                                              | *1                                              | *1                                              |
| CLKO0             | SHARC0 and its Accelerators                | *1                                              | *1                                              | *1                                              | Programmable                                    | *1                                              | *1                                              |
| CLKO1             | A55                                        | *1                                              | *1                                              | *1                                              | *1                                              | Programmable                                    | *1                                              |
| CLKO3             | DMC                                        | *1                                              | Programmable                                    | Programmable                                    | *1                                              | *1                                              | *1                                              |
| CLKO4             | CAN                                        | *1                                              | Programmable                                    | *1                                              | *1                                              | *1                                              | *1                                              |
| CLKO6             | SPI                                        | Synchronous (m:n) (except SPIF complet- er)     | Synchronous (m:n)                               | *1                                              | *1                                              | *1                                              | Synchronous (m:n)                               |
| CLKO8             | LP                                         | Programmable                                    | Programmable                                    | *1                                              | *1                                              | *1                                              | *1                                              |

![Image](48_System_Crossbars_(SCB)_artifacts/image_000006_d1225b3ed3126a5e7d0377a80157adf4cc57aefafbf8f4244ab6758141e05692.png)

NOTE: Consider the following points in the Clock Domain Crossing Options table:

- For DMC fabric, the sync mode register should be programmed for m:n or async (default). Only for sysclk:CLKO3 = 500:500MHz, sync mode can be 1:1.
- For system MMRG fabric - CLKO8/LP sync mode register should be programmed for m:n or async (default).

Table 45-9: Sync Mode Bit Field Description

|   Sync Mode | Description   |
|-------------|---------------|
|           0 | sync 1:1      |
|           1 | sync n:1      |
|           2 | sync 1:n      |
|           3 | sync m:n      |
|           4 | async         |

To change the clock domain crossing mode, follow the actions described in the Changing Clock Domain Crossing Modes table.

Table 45-10: Changing Clock Domain Crossing Modes

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

Figure 45-8: LRG Arbitration Example

![Image](48_System_Crossbars_(SCB)_artifacts/image_000007_a68e0a8710670fc4927c7d1f38c78ce6ef2e9d8524c72251c4bf2b8a9477ffb7.png)

The QoS value assigned to a transaction at entry point is carried forward by the transaction as it passes through all arbitration stages in the SCB. QoS for all requesters is configured as programmable in the system fabric interconnect.

The priority of the requesters fits into three groups:

- Group A - Peripherals with an external interface, without flow control and with real-time processing requirements
- Group B - Cores, peripherals with flow control, and offload engines
- Group C - MDMAs

## Group A: Peripherals with external interface, without flow control and with real-time processing requirements

These requesters are assigned the highest priority. They are latency-critical - a large increase in latency could potentially result in data corruption and catastrophic failure. This high priority SCB group includes requesters connected to SCB1, SCB2, SCB3, SCB4, MLB and XSPI M. Requesters in this group are assigned the QOS reset value of 12.

## Group B: Cores, peripherals with flow control, and offload engines

Core is latency-sensitive. Once a core requests data, it typically waits for the data without performing anything else in parallel. An increase in the latency has a direct impact on the performance of the core. This group is assigned medium priority and a QOS reset value of 7. Group B requesters are connected to SCB6, GIGE, CORES\_SCB, FIR\_IPORT\_SCB, and MMR\_SCB.

## Group C: MDMAs

These requesters are assigned the lowest priority. MDMAs are data intensive and do not have any external interface. Requesters in this group are assigned the QOS reset value of 1. The Group C requesters are connected to SCB7, SCB8, SCB9, and SCB10.

Table 45-11: QoS Register Table

| Requester    |   read_qos Reset Value |   write_qos Reset Value |
|--------------|------------------------|-------------------------|
| SPORT0_A_DMA |                     12 |                      12 |
| SPORT0_B_DMA |                     12 |                      12 |
| SPORT1_A_DMA |                     12 |                      12 |
| SPORT1_B_DMA |                     12 |                      12 |
| SPORT2_A_DMA |                     12 |                      12 |
| SPORT2_B_DMA |                     12 |                      12 |
| SPORT3_A_DMA |                     12 |                      12 |
| SPORT3_B_DMA |                     12 |                      12 |
| SPORT4_A_DMA |                     12 |                      12 |
| SPORT4_B_DMA |                     12 |                      12 |
| SPORT5_A_DMA |                     12 |                      12 |
| SPORT5_B_DMA |                     12 |                      12 |
| SPORT6_A_DMA |                     12 |                      12 |
| SPORT6_B_DMA |                     12 |                      12 |
| SPORT7_A_DMA |                     12 |                      12 |
| SPORT7_B_DMA |                     12 |                      12 |
| UART0_TX     |                     12 |                      12 |
| UART0_RX     |                     12 |                      12 |
| UART1_TX     |                     12 |                      12 |
| UART1_RX     |                     12 |                      12 |
| UART2_TX     |                     12 |                      12 |
| UART2_RX     |                     12 |                      12 |

Table 45-11: QoS Register Table (Continued)

| Requester    |   read_qos Reset Value |   write_qos Reset Value |
|--------------|------------------------|-------------------------|
| SPI0RX       |                     12 |                      12 |
| SPI0TX       |                     12 |                      12 |
| SPI1TX       |                     12 |                      12 |
| SPI1RX       |                     12 |                      12 |
| SPI2TX       |                     12 |                      12 |
| SPI2RX       |                     12 |                      12 |
| SPI5TX       |                     12 |                      12 |
| SPI5RX       |                     12 |                      12 |
| MLB          |                     12 |                      12 |
| xSPI0        |                     12 |                      12 |
| xSPI1        |                     12 |                      12 |
| LP0          |                      7 |                       7 |
| LP1          |                      7 |                       7 |
| GIGE         |                      7 |                       7 |
| EMMC         |                      7 |                       7 |
| SH0_DPORT    |                      7 |                       7 |
| SH0_DPORT_L2 |                      7 |                       7 |
| iDMA         |                      7 |                       7 |
| SH0_IPORT    |                      7 |                       7 |
| SH0_IIR_CH0  |                      7 |                       7 |
| SH0_IIR_CH1  |                      7 |                       7 |
| SH0_FIR_CH0  |                      7 |                       7 |
| SH0_FIR_CH1  |                      7 |                       7 |
| SH0_MMR      |                      7 |                       7 |
| A55MMR       |                      7 |                       7 |
| A55 M0 AXI   |                      7 |                       7 |
| A55 M1 AXI   |                      7 |                       7 |
| HSM          |                      7 |                       7 |
| CRC0_CH0     |                      1 |                       1 |
| CRC0_CH1     |                      1 |                       1 |
| CRC1_CH0     |                      1 |                       1 |

Table 45-11: QoS Register Table (Continued)

| Requester   |   read_qos Reset Value |   write_qos Reset Value |
|-------------|------------------------|-------------------------|
| CRC1_CH1    |                      1 |                       1 |
| CRC2_CH0    |                      1 |                       1 |
| CRC2_CH1    |                      1 |                       1 |
| CRC3_CH0    |                      1 |                       1 |
| CRC3_CH1    |                      1 |                       1 |
| MDMA2_CH0   |                      1 |                       1 |
| MDMA2_CH1   |                      1 |                       1 |
| MDMA3_CH0   |                      1 |                       1 |
| MDMA3_CH1   |                      1 |                       1 |
| MDMA6_CH0   |                      1 |                       1 |
| MDMA6_CH1   |                      1 |                       1 |
| MDMA7_CH0   |                      1 |                       1 |
| MDMA7_CH1   |                      1 |                       1 |
| EMDMA0_CH0  |                      1 |                       1 |
| EMDMA0_CH1  |                      1 |                       1 |
| EMDMA1_CH0  |                      1 |                       1 |
| EMDMA1_CH1  |                      1 |                       1 |
| CRYPTO      |                      1 |                       1 |
| ETR         |                      1 |                       1 |
| DBG         |                      1 |                       1 |

## QoS Programming

Adhere to the following guidelines if software is modifying the reset value of QoS:

- The highest QoS of any requester in a low priority SCB group must be less than the lowest QoS of any requester in a medium priority SCB group
- The highest QoS of any requester in medium priority SCB group must be less than the lowest QoS of any requester in high priority SCB group

Reset values of QoS have been spaced out to allow software to lower or higher the priority of a few requesters without having to modify the priority all the requesters. For example, the QoS of the SPI can be reduced from 12 to 11 without violating the two guidelines outlined above.

## ADSP-2184x SCB0 Register Descriptions

Your module description, here. (SCB0) contains the following registers.

Table 45-12: ADSP-2184x SCB0 Register List

| Name                       | Description   |
|----------------------------|---------------|
| SCB0_HSM_IB_FN_MOD         |               |
| SCB0_HSM_IB_FN_MOD2        |               |
| SCB0_HSM_IB_READ_QOS       |               |
| SCB0_HSM_IB_WRITE_QOS      |               |
| SCB0_CL2_1_FN_MOD_ISS_BM   |               |
| SCB0_CL2_2_FN_MOD_ISS_BM   |               |
| SCB0_COMP_ID_0             |               |
| SCB0_COMP_ID_1             |               |
| SCB0_COMP_ID_2             |               |
| SCB0_COMP_ID_3             |               |
| SCB0_CRC0_CH0_IB_FN_MOD    |               |
| SCB0_CRC0_CH0_IB_FN_MOD2   |               |
| SCB0_CRC0_CH0_IB_READ_QOS  |               |
| SCB0_CRC0_CH0_IB_WRITE_QOS |               |
| SCB0_CRC0_CH1_IB_FN_MOD    |               |
| SCB0_CRC0_CH1_IB_FN_MOD2   |               |
| SCB0_CRC0_CH1_IB_READ_QOS  |               |
| SCB0_CRC0_CH1_IB_WRITE_QOS |               |
| SCB0_CRC1_CH0_IB_FN_MOD    |               |
| SCB0_CRC1_CH0_IB_FN_MOD2   |               |
| SCB0_CRC1_CH0_IB_READ_QOS  |               |
| SCB0_CRC1_CH0_IB_WRITE_QOS |               |
| SCB0_CRC1_CH1_IB_FN_MOD    |               |
| SCB0_CRC1_CH1_IB_FN_MOD2   |               |
| SCB0_CRC1_CH1_IB_READ_QOS  |               |
| SCB0_CRC1_CH1_IB_WRITE_QOS |               |
| SCB0_CRC2_CH0_IB_FN_MOD    |               |
| SCB0_CRC2_CH0_IB_FN_MOD2   |               |
| SCB0_CRC2_CH0_IB_READ_QOS  |               |
| SCB0_CRC2_CH0_IB_WRITE_QOS |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                         | Description   |
|------------------------------|---------------|
| SCB0_CRC2_CH1_IB_FN_MOD      |               |
| SCB0_CRC2_CH1_IB_FN_MOD2     |               |
| SCB0_CRC2_CH1_IB_READ_QOS    |               |
| SCB0_CRC2_CH1_IB_WRITE_QOS   |               |
| SCB0_CRC3_CH0_IB_FN_MOD      |               |
| SCB0_CRC3_CH0_IB_FN_MOD2     |               |
| SCB0_CRC3_CH0_IB_READ_QOS    |               |
| SCB0_CRC3_CH0_IB_WRITE_QOS   |               |
| SCB0_CRC3_CH1_IB_FN_MOD      |               |
| SCB0_CRC3_CH1_IB_FN_MOD2     |               |
| SCB0_CRC3_CH1_IB_READ_QOS    |               |
| SCB0_CRC3_CH1_IB_WRITE_QOS   |               |
| SCB0_CRYPTO_IB_FN_MOD        |               |
| SCB0_CRYPTO_IB_FN_MOD2       |               |
| SCB0_CRYPTO_IB_READ_QOS      |               |
| SCB0_CRYPTO_IB_WRITE_QOS     |               |
| SCB0_DBG_IB_FN_MOD           |               |
| SCB0_DBG_IB_FN_MOD2          |               |
| SCB0_DBG_IB_READ_QOS         |               |
| SCB0_DBG_IB_WRITE_QOS        |               |
| SCB0_DL2_0_FN_MOD_ISS_BM     |               |
| SCB0_DL2_1_FN_MOD_ISS_BM     |               |
| SCB0_DLDMA0_CH0_IB_FN_MOD    |               |
| SCB0_DLDMA0_CH0_IB_FN_MOD2   |               |
| SCB0_DLDMA0_CH0_IB_READ_QOS  |               |
| SCB0_DLDMA0_CH0_IB_WRITE_QOS |               |
| SCB0_DLDMA0_CH1_IB_FN_MOD    |               |
| SCB0_DLDMA0_CH1_IB_FN_MOD2   |               |
| SCB0_DLDMA0_CH1_IB_READ_QOS  |               |
| SCB0_DLDMA0_CH1_IB_WRITE_QOS |               |
| SCB0_DLDMA1_CH0_IB_FN_MOD    |               |
| SCB0_DLDMA1_CH0_IB_FN_MOD2   |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                          | Description   |
|-------------------------------|---------------|
| SCB0_DLDMA1_CH0_IB_READ_QOS   |               |
| SCB0_DLDMA1_CH0_IB_WRITE_QOS  |               |
| SCB0_DLDMA1_CH1_IB_FN_MOD     |               |
| SCB0_DLDMA1_CH1_IB_FN_MOD2    |               |
| SCB0_DLDMA1_CH1_IB_READ_QOS   |               |
| SCB0_DLDMA1_CH1_IB_WRITE_QOS  |               |
| SCB0_DMC0_FN_MOD_ISS_BM       |               |
| SCB0_EMMC_IB_FN_MOD           |               |
| SCB0_EMMC_IB_FN_MOD2          |               |
| SCB0_EMMC_IB_READ_QOS         |               |
| SCB0_EMMC_IB_WRITE_QOS        |               |
| SCB0_ETR_IB_FN_MOD            |               |
| SCB0_ETR_IB_FN_MOD2           |               |
| SCB0_ETR_IB_READ_QOS          |               |
| SCB0_ETR_IB_WRITE_QOS         |               |
| SCB0_GIGE1_IB_FN_MOD          |               |
| SCB0_GIGE1_IB_FN_MOD2         |               |
| SCB0_GIGE1_IB_READ_QOS        |               |
| SCB0_GIGE1_IB_WRITE_QOS       |               |
| SCB0_GIGE_IB_FN_MOD           |               |
| SCB0_GIGE_IB_FN_MOD2          |               |
| SCB0_GIGE_IB_READ_QOS         |               |
| SCB0_GIGE_IB_WRITE_QOS        |               |
| SCB0_HSMDMA1_CH0_IB_FN_MOD    |               |
| SCB0_HSMDMA1_CH0_IB_FN_MOD2   |               |
| SCB0_HSMDMA1_CH0_IB_READ_QOS  |               |
| SCB0_HSMDMA1_CH0_IB_WRITE_QOS |               |
| SCB0_HSMDMA1_CH1_IB_FN_MOD    |               |
| SCB0_HSMDMA1_CH1_IB_FN_MOD2   |               |
| SCB0_HSMDMA1_CH1_IB_READ_QOS  |               |
| SCB0_HSMDMA1_CH1_IB_WRITE_QOS |               |
| SCB0_HSMDMA_CH0_IB_FN_MOD     |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                             | Description   |
|----------------------------------|---------------|
| SCB0_HSMDMA_CH0_IB_FN_MOD2       |               |
| SCB0_HSMDMA_CH0_IB_READ_QOS      |               |
| SCB0_HSMDMA_CH0_IB_WRITE_QOS     |               |
| SCB0_HSMDMA_CH1_IB_FN_MOD        |               |
| SCB0_HSMDMA_CH1_IB_FN_MOD2       |               |
| SCB0_HSMDMA_CH1_IB_READ_QOS      |               |
| SCB0_HSMDMA_CH1_IB_WRITE_QOS     |               |
| SCB0_IDMA_FN_MOD                 |               |
| SCB0_IDMA_READ_QOS               |               |
| SCB0_IDMA_WRITE_QOS              |               |
| SCB0_LP0_FN_MOD                  |               |
| SCB0_LP0_READ_QOS                |               |
| SCB0_LP0_WRITE_QOS               |               |
| SCB0_LP1_FN_MOD                  |               |
| SCB0_LP1_READ_QOS                |               |
| SCB0_LP1_WRITE_QOS               |               |
| SCB0_M85_M2_AXI_FN_MOD           |               |
| SCB0_M85_M2_AXI_READ_QOS         |               |
| SCB0_M85_M2_AXI_WRITE_QOS        |               |
| SCB0_M85_M_AXI_FN_MOD            |               |
| SCB0_M85_M_AXI_READ_QOS          |               |
| SCB0_M85_M_AXI_WRITE_QOS         |               |
| SCB0_M85_S2_AXI_IB_FN_MOD        |               |
| SCB0_M85_S2_AXI_IB_FN_MOD_ISS_BM |               |
| SCB0_M85_S_AXI_IB_FN_MOD         |               |
| SCB0_M85_S_AXI_IB_FN_MOD_ISS_BM  |               |
| SCB0_MLB_FN_MOD                  |               |
| SCB0_MLB_READ_QOS                |               |
| SCB0_MLB_WRITE_QOS               |               |
| SCB0_MSMDMA1_CH0_IB_FN_MOD       |               |
| SCB0_MSMDMA1_CH0_IB_FN_MOD2      |               |
| SCB0_MSMDMA1_CH0_IB_READ_QOS     |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                          | Description   |
|-------------------------------|---------------|
| SCB0_MSMDMA1_CH0_IB_WRITE_QOS |               |
| SCB0_MSMDMA1_CH1_IB_FN_MOD    |               |
| SCB0_MSMDMA1_CH1_IB_FN_MOD2   |               |
| SCB0_MSMDMA1_CH1_IB_READ_QOS  |               |
| SCB0_MSMDMA1_CH1_IB_WRITE_QOS |               |
| SCB0_MSMDMA_CH0_IB_FN_MOD     |               |
| SCB0_MSMDMA_CH0_IB_FN_MOD2    |               |
| SCB0_MSMDMA_CH0_IB_READ_QOS   |               |
| SCB0_MSMDMA_CH0_IB_WRITE_QOS  |               |
| SCB0_MSMDMA_CH1_IB_FN_MOD     |               |
| SCB0_MSMDMA_CH1_IB_FN_MOD2    |               |
| SCB0_MSMDMA_CH1_IB_READ_QOS   |               |
| SCB0_MSMDMA_CH1_IB_WRITE_QOS  |               |
| SCB0_OTP_IB_FN_MOD            |               |
| SCB0_OTP_IB_FN_MOD_ISS_BM     |               |
| SCB0_PERIPH_ID_0              |               |
| SCB0_PERIPH_ID_1              |               |
| SCB0_PERIPH_ID_2              |               |
| SCB0_PERIPH_ID_3              |               |
| SCB0_PERIPH_ID_4              |               |
| SCB0_PERIPH_ID_5              |               |
| SCB0_PERIPH_ID_6              |               |
| SCB0_PERIPH_ID_7              |               |
| SCB0_SCB6_IB_FN_MOD           |               |
| SCB0_SCB6_IB_FN_MOD2          |               |
| SCB0_SCB6_IB_FN_MOD_ISS_BM    |               |
| SCB0_SCB6_IB_SYNC_MODE        |               |
| SCB0_SH0_DPORT_L2_FN_MOD      |               |
| SCB0_SH0_DPORT_L2_READ_QOS    |               |
| SCB0_SH0_DPORT_L2_WRITE_QOS   |               |
| SCB0_SH0_DPORT_FN_MOD         |               |
| SCB0_SH0_DPORT_READ_QOS       |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                          | Description   |
|-------------------------------|---------------|
| SCB0_SH0_DPORT_WRITE_QOS      |               |
| SCB0_SH0_FIR_CH0_FN_MOD       |               |
| SCB0_SH0_FIR_CH0_READ_QOS     |               |
| SCB0_SH0_FIR_CH0_WRITE_QOS    |               |
| SCB0_SH0_FIR_CH1_FN_MOD       |               |
| SCB0_SH0_FIR_CH1_READ_QOS     |               |
| SCB0_SH0_FIR_CH1_WRITE_QOS    |               |
| SCB0_SH0_IIR_CH0_FN_MOD       |               |
| SCB0_SH0_IIR_CH0_READ_QOS     |               |
| SCB0_SH0_IIR_CH0_WRITE_QOS    |               |
| SCB0_SH0_IIR_CH1_FN_MOD       |               |
| SCB0_SH0_IIR_CH1_READ_QOS     |               |
| SCB0_SH0_IIR_CH1_WRITE_QOS    |               |
| SCB0_SH0_IPORT_FN_MOD         |               |
| SCB0_SH0_IPORT_READ_QOS       |               |
| SCB0_SH0_IPORT_WRITE_QOS      |               |
| SCB0_SH0_MMR_IB_FN_MOD        |               |
| SCB0_SH0_MMR_IB_FN_MOD2       |               |
| SCB0_SH0_MMR_IB_READ_QOS      |               |
| SCB0_SH0_MMR_IB_WRITE_QOS     |               |
| SCB0_SH0_S1PORT_FN_MOD_ISS_BM |               |
| SCB0_SH0_S2PORT_FN_MOD_ISS_BM |               |
| SCB0_SMC_IB_FN_MOD            |               |
| SCB0_SMC_IB_FN_MOD_ISS_BM     |               |
| SCB0_SMMR_FN_MOD_ISS_BM       |               |
| SCB0_SP0A_FN_MOD              |               |
| SCB0_SP0A_READ_QOS            |               |
| SCB0_SP0A_WRITE_QOS           |               |
| SCB0_SP0B_FN_MOD              |               |
| SCB0_SP0B_READ_QOS            |               |
| SCB0_SP0B_WRITE_QOS           |               |
| SCB0_SP1A_FN_MOD              |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                | Description   |
|---------------------|---------------|
| SCB0_SP1A_READ_QOS  |               |
| SCB0_SP1A_WRITE_QOS |               |
| SCB0_SP1B_FN_MOD    |               |
| SCB0_SP1B_READ_QOS  |               |
| SCB0_SP1B_WRITE_QOS |               |
| SCB0_SP2A_FN_MOD    |               |
| SCB0_SP2A_READ_QOS  |               |
| SCB0_SP2A_WRITE_QOS |               |
| SCB0_SP2B_FN_MOD    |               |
| SCB0_SP2B_READ_QOS  |               |
| SCB0_SP2B_WRITE_QOS |               |
| SCB0_SP3A_FN_MOD    |               |
| SCB0_SP3A_READ_QOS  |               |
| SCB0_SP3A_WRITE_QOS |               |
| SCB0_SP3B_FN_MOD    |               |
| SCB0_SP3B_READ_QOS  |               |
| SCB0_SP3B_WRITE_QOS |               |
| SCB0_SP4A_FN_MOD    |               |
| SCB0_SP4A_READ_QOS  |               |
| SCB0_SP4A_WRITE_QOS |               |
| SCB0_SP4B_FN_MOD    |               |
| SCB0_SP4B_READ_QOS  |               |
| SCB0_SP4B_WRITE_QOS |               |
| SCB0_SP5A_FN_MOD    |               |
| SCB0_SP5A_READ_QOS  |               |
| SCB0_SP5A_WRITE_QOS |               |
| SCB0_SP5B_FN_MOD    |               |
| SCB0_SP5B_READ_QOS  |               |
| SCB0_SP5B_WRITE_QOS |               |
| SCB0_SP6A_FN_MOD    |               |
| SCB0_SP6A_READ_QOS  |               |
| SCB0_SP6A_WRITE_QOS |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                  | Description   |
|-----------------------|---------------|
| SCB0_SP6B_FN_MOD      |               |
| SCB0_SP6B_READ_QOS    |               |
| SCB0_SP6B_WRITE_QOS   |               |
| SCB0_SP7A_FN_MOD      |               |
| SCB0_SP7A_READ_QOS    |               |
| SCB0_SP7A_WRITE_QOS   |               |
| SCB0_SP7B_FN_MOD      |               |
| SCB0_SP7B_READ_QOS    |               |
| SCB0_SP7B_WRITE_QOS   |               |
| SCB0_SPI0RX_FN_MOD    |               |
| SCB0_SPI0RX_READ_QOS  |               |
| SCB0_SPI0RX_WRITE_QOS |               |
| SCB0_SPI0TX_FN_MOD    |               |
| SCB0_SPI0TX_READ_QOS  |               |
| SCB0_SPI0TX_WRITE_QOS |               |
| SCB0_SPI1RX_FN_MOD    |               |
| SCB0_SPI1RX_READ_QOS  |               |
| SCB0_SPI1RX_WRITE_QOS |               |
| SCB0_SPI1TX_FN_MOD    |               |
| SCB0_SPI1TX_READ_QOS  |               |
| SCB0_SPI1TX_WRITE_QOS |               |
| SCB0_SPI2RX_FN_MOD    |               |
| SCB0_SPI2RX_READ_QOS  |               |
| SCB0_SPI2RX_WRITE_QOS |               |
| SCB0_SPI2TX_FN_MOD    |               |
| SCB0_SPI2TX_READ_QOS  |               |
| SCB0_SPI2TX_WRITE_QOS |               |
| SCB0_SPI5RX_FN_MOD    |               |
| SCB0_SPI5RX_READ_QOS  |               |
| SCB0_SPI5RX_WRITE_QOS |               |
| SCB0_SPI5TX_FN_MOD    |               |
| SCB0_SPI5TX_READ_QOS  |               |

Table 45-12: ADSP-2184x SCB0 Register List (Continued)

| Name                    | Description   |
|-------------------------|---------------|
| SCB0_SPI5TX_WRITE_QOS   |               |
| SCB0_SPIF_FN_MOD_ISS_BM |               |
| SCB0_UART0_RX_FN_MOD    |               |
| SCB0_UART0_RX_READ_QOS  |               |
| SCB0_UART0_RX_WRITE_QOS |               |
| SCB0_UART0_TX_FN_MOD    |               |
| SCB0_UART0_TX_READ_QOS  |               |
| SCB0_UART0_TX_WRITE_QOS |               |
| SCB0_UART1_RX_FN_MOD    |               |
| SCB0_UART1_RX_READ_QOS  |               |
| SCB0_UART1_RX_WRITE_QOS |               |
| SCB0_UART1_TX_FN_MOD    |               |
| SCB0_UART1_TX_READ_QOS  |               |
| SCB0_UART1_TX_WRITE_QOS |               |
| SCB0_UART2_RX_FN_MOD    |               |
| SCB0_UART2_RX_READ_QOS  |               |
| SCB0_UART2_RX_WRITE_QOS |               |
| SCB0_UART2_TX_FN_MOD    |               |
| SCB0_UART2_TX_READ_QOS  |               |
| SCB0_UART2_TX_WRITE_QOS |               |
| SCB0_XSPI_M1_IB_FN_MOD  |               |
| SCB0_XSPI_M1_IB_FN_MOD2 |               |
| SCB0_XSPI_M_IB_FN_MOD   |               |
| SCB0_XSPI_M_IB_FN_MOD2  |               |

Figure 45-9: SCB0\_HSM\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000008_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-13: SCB0\_HSM\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-10: SCB0\_HSM\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000009_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-14: SCB0\_HSM\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-11: SCB0\_HSM\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000010_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-15: SCB0\_HSM\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-12: SCB0\_HSM\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000011_af82241f7125e5739c3ffc5a3a90b67b58fd522473d5a16fff8db13d2e6b6b63.png)

Table 45-16: SCB0\_HSM\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-13: SCB0\_CL2\_1\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000012_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-17: SCB0\_CL2\_1\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-14: SCB0\_CL2\_2\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000013_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-18: SCB0\_CL2\_2\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-15: SCB0\_COMP\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000014_fa7b5c4ab13190a531852390dbbf0eb914d043d23d39766462def973e03a61ef.png)

Table 45-19: SCB0\_COMP\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_0  | .                         |
| (R/NW)             |            |                           |

Figure 45-16: SCB0\_COMP\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000015_0c6358edd6bb0b55252666e64f84d693d96f49320192cdcaa21cf3af93d0047a.png)

Table 45-20: SCB0\_COMP\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_1  | .                         |

Figure 45-17: SCB0\_COMP\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000016_a88b8da2d9b8531f931923a33b2ab653413ae28b2dfbdb4448705e127f1ce435.png)

Table 45-21: SCB0\_COMP\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_2  | .                         |

Figure 45-18: SCB0\_COMP\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000017_53cde00cc5ee17124fb968b9465a049344dacdd7044ae0fb14336394d266e952.png)

Table 45-22: SCB0\_COMP\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_3  | .                         |
| (R/NW)             |            |                           |

Figure 45-19: SCB0\_CRC0\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000018_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-23: SCB0\_CRC0\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-20: SCB0\_CRC0\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000019_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-24: SCB0\_CRC0\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-21: SCB0\_CRC0\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000020_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-25: SCB0\_CRC0\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-22: SCB0\_CRC0\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000021_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-26: SCB0\_CRC0\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-23: SCB0\_CRC0\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000022_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-27: SCB0\_CRC0\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-24: SCB0\_CRC0\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000023_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-28: SCB0\_CRC0\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-25: SCB0\_CRC0\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000024_e3bcc278f0034810d43a716b1701c6f7f8fa01fbb086310385df58bbff3af57b.png)

Table 45-29: SCB0\_CRC0\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-26: SCB0\_CRC0\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000025_ce769fbbec284a1f5c4fc834bda7478e3333536ab43d73b2d1bd1ba054b22aa9.png)

Table 45-30: SCB0\_CRC0\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-27: SCB0\_CRC1\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000026_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-31: SCB0\_CRC1\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-28: SCB0\_CRC1\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000027_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-32: SCB0\_CRC1\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-29: SCB0\_CRC1\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000028_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-33: SCB0\_CRC1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-30: SCB0\_CRC1\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000029_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-34: SCB0\_CRC1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-31: SCB0\_CRC1\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000030_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-35: SCB0\_CRC1\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-32: SCB0\_CRC1\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000031_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-36: SCB0\_CRC1\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-33: SCB0\_CRC1\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000032_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-37: SCB0\_CRC1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-34: SCB0\_CRC1\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000033_ce769fbbec284a1f5c4fc834bda7478e3333536ab43d73b2d1bd1ba054b22aa9.png)

Table 45-38: SCB0\_CRC1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-35: SCB0\_CRC2\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000034_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-39: SCB0\_CRC2\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-36: SCB0\_CRC2\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000035_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-40: SCB0\_CRC2\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-37: SCB0\_CRC2\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000036_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-41: SCB0\_CRC2\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-38: SCB0\_CRC2\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000037_ce769fbbec284a1f5c4fc834bda7478e3333536ab43d73b2d1bd1ba054b22aa9.png)

Table 45-42: SCB0\_CRC2\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-39: SCB0\_CRC2\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000038_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-43: SCB0\_CRC2\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-40: SCB0\_CRC2\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000039_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-44: SCB0\_CRC2\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-41: SCB0\_CRC2\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000040_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-45: SCB0\_CRC2\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-42: SCB0\_CRC2\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000041_ce769fbbec284a1f5c4fc834bda7478e3333536ab43d73b2d1bd1ba054b22aa9.png)

Table 45-46: SCB0\_CRC2\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-43: SCB0\_CRC3\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000042_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-47: SCB0\_CRC3\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-44: SCB0\_CRC3\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000043_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-48: SCB0\_CRC3\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-45: SCB0\_CRC3\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000044_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-49: SCB0\_CRC3\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-46: SCB0\_CRC3\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000045_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-50: SCB0\_CRC3\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-47: SCB0\_CRC3\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000046_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-51: SCB0\_CRC3\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-48: SCB0\_CRC3\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000047_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-52: SCB0\_CRC3\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-49: SCB0\_CRC3\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000048_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-53: SCB0\_CRC3\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-50: SCB0\_CRC3\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000049_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-54: SCB0\_CRC3\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-51: SCB0\_CRYPTO\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000050_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-55: SCB0\_CRYPTO\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-52: SCB0\_CRYPTO\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000051_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-56: SCB0\_CRYPTO\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-53: SCB0\_CRYPTO\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000052_e3bcc278f0034810d43a716b1701c6f7f8fa01fbb086310385df58bbff3af57b.png)

Table 45-57: SCB0\_CRYPTO\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-54: SCB0\_CRYPTO\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000053_7c4cc60a26239c77980383fa9fb62ae4d3f8cc54e5298250eafd90f13b4bc6aa.png)

Table 45-58: SCB0\_CRYPTO\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-55: SCB0\_DBG\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000054_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-59: SCB0\_DBG\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-56: SCB0\_DBG\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000055_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-60: SCB0\_DBG\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-57: SCB0\_DBG\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000056_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-61: SCB0\_DBG\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-58: SCB0\_DBG\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000057_7c4cc60a26239c77980383fa9fb62ae4d3f8cc54e5298250eafd90f13b4bc6aa.png)

Table 45-62: SCB0\_DBG\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-59: SCB0\_DL2\_0\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000058_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-63: SCB0\_DL2\_0\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-60: SCB0\_DL2\_1\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000059_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-64: SCB0\_DL2\_1\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-61: SCB0\_DLDMA0\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000060_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-65: SCB0\_DLDMA0\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-62: SCB0\_DLDMA0\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000061_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-66: SCB0\_DLDMA0\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-63: SCB0\_DLDMA0\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000062_54171ea2f97d7a812c2881a00e86175d2e88a055ca2f2f3623072bd1fba9ce3b.png)

Table 45-67: SCB0\_DLDMA0\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-64: SCB0\_DLDMA0\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000063_7c4cc60a26239c77980383fa9fb62ae4d3f8cc54e5298250eafd90f13b4bc6aa.png)

Table 45-68: SCB0\_DLDMA0\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-65: SCB0\_DLDMA0\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000064_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-69: SCB0\_DLDMA0\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-66: SCB0\_DLDMA0\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000065_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-70: SCB0\_DLDMA0\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-67: SCB0\_DLDMA0\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000066_e0aa0e1dcb6d9a1a8e346f3a7a98805edede9c553236b3ac354fc62bd7cc9f01.png)

Table 45-71: SCB0\_DLDMA0\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-68: SCB0\_DLDMA0\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000067_7c4cc60a26239c77980383fa9fb62ae4d3f8cc54e5298250eafd90f13b4bc6aa.png)

Table 45-72: SCB0\_DLDMA0\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-69: SCB0\_DLDMA1\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000068_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-73: SCB0\_DLDMA1\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-70: SCB0\_DLDMA1\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000069_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-74: SCB0\_DLDMA1\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-71: SCB0\_DLDMA1\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000070_e0aa0e1dcb6d9a1a8e346f3a7a98805edede9c553236b3ac354fc62bd7cc9f01.png)

Table 45-75: SCB0\_DLDMA1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-72: SCB0\_DLDMA1\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000071_ce769fbbec284a1f5c4fc834bda7478e3333536ab43d73b2d1bd1ba054b22aa9.png)

Table 45-76: SCB0\_DLDMA1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-73: SCB0\_DLDMA1\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000072_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-77: SCB0\_DLDMA1\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-74: SCB0\_DLDMA1\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000073_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-78: SCB0\_DLDMA1\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-75: SCB0\_DLDMA1\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000074_e0aa0e1dcb6d9a1a8e346f3a7a98805edede9c553236b3ac354fc62bd7cc9f01.png)

Table 45-79: SCB0\_DLDMA1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-76: SCB0\_DLDMA1\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000075_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-80: SCB0\_DLDMA1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-77: SCB0\_DMC0\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000076_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-81: SCB0\_DMC0\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-78: SCB0\_EMMC\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000077_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-82: SCB0\_EMMC\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-79: SCB0\_EMMC\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000078_b4c15724249e3ea71d43e64170c3a9f0f2999e4c788b9471d0f52701cb720ce5.png)

Table 45-83: SCB0\_EMMC\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-80: SCB0\_EMMC\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000079_1aaa44c58a1b173dd77c14f0b923167b44976d1941de7dd84e0e11ebfb055fbb.png)

Table 45-84: SCB0\_EMMC\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-81: SCB0\_EMMC\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000080_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-85: SCB0\_EMMC\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-82: SCB0\_ETR\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000081_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-86: SCB0\_ETR\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-83: SCB0\_ETR\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000082_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-87: SCB0\_ETR\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-84: SCB0\_ETR\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000083_5fe3d5f5d106de4b7c4fb879662eb70c8544b1e66a84f1f5779dc2602cc333cf.png)

Table 45-88: SCB0\_ETR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-85: SCB0\_ETR\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000084_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-89: SCB0\_ETR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-86: SCB0\_GIGE1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000085_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-90: SCB0\_GIGE1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-87: SCB0\_GIGE1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000086_b4c15724249e3ea71d43e64170c3a9f0f2999e4c788b9471d0f52701cb720ce5.png)

Table 45-91: SCB0\_GIGE1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-88: SCB0\_GIGE1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000087_028d51cfd0c8ff106a940192e92d6d99d5f6b5d6cca6abceee2a9d9b8e69682b.png)

Table 45-92: SCB0\_GIGE1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-89: SCB0\_GIGE1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000088_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-93: SCB0\_GIGE1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-90: SCB0\_GIGE\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000089_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-94: SCB0\_GIGE\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-91: SCB0\_GIGE\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000090_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-95: SCB0\_GIGE\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-92: SCB0\_GIGE\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000091_1aaa44c58a1b173dd77c14f0b923167b44976d1941de7dd84e0e11ebfb055fbb.png)

Table 45-96: SCB0\_GIGE\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-93: SCB0\_GIGE\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000092_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-97: SCB0\_GIGE\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-94: SCB0\_HSMDMA1\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000093_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-98: SCB0\_HSMDMA1\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-95: SCB0\_HSMDMA1\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000094_b9e219e68e22c327cd432539120bbcc7bc6c60c299c19487716d6651430a3f9a.png)

Table 45-99: SCB0\_HSMDMA1\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-96: SCB0\_HSMDMA1\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000095_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-100: SCB0\_HSMDMA1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-97: SCB0\_HSMDMA1\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000096_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-101: SCB0\_HSMDMA1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-98: SCB0\_HSMDMA1\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000097_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-102: SCB0\_HSMDMA1\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-99: SCB0\_HSMDMA1\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000098_dbb60d39ce5407ff1267746c2fed6a38f7bc1c79fb5b06d8199452e9b8d03c50.png)

Table 45-103: SCB0\_HSMDMA1\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-100: SCB0\_HSMDMA1\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000099_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-104: SCB0\_HSMDMA1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-101: SCB0\_HSMDMA1\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000100_72d4cc4999e2444f5eb7e209234c71f17494b0a218b85498d7deed126f012040.png)

Table 45-105: SCB0\_HSMDMA1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-102: SCB0\_HSMDMA\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000101_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-106: SCB0\_HSMDMA\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-103: SCB0\_HSMDMA\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000102_0343c99aeb8656582a6084d6339bb12c5839e895a33bfb86927d8e0d468372be.png)

Table 45-107: SCB0\_HSMDMA\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-104: SCB0\_HSMDMA\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000103_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-108: SCB0\_HSMDMA\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-105: SCB0\_HSMDMA\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000104_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-109: SCB0\_HSMDMA\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-106: SCB0\_HSMDMA\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000105_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-110: SCB0\_HSMDMA\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-107: SCB0\_HSMDMA\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000106_32f1a33d0624d7588b4ea17d8e6b9b4a913741219578151cec78e5ba3535332d.png)

Table 45-111: SCB0\_HSMDMA\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-108: SCB0\_HSMDMA\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000107_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-112: SCB0\_HSMDMA\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-109: SCB0\_HSMDMA\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000108_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-113: SCB0\_HSMDMA\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-110: SCB0\_IDMA\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000109_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-114: SCB0\_IDMA\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-111: SCB0\_IDMA\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000110_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-115: SCB0\_IDMA\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-112: SCB0\_IDMA\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000111_2d48b6eb52ec59c5dd7f0f28dec3da21fb00334cb3fd1132ff08df6bea801c93.png)

Table 45-116: SCB0\_IDMA\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-113: SCB0\_LP0\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000112_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-117: SCB0\_LP0\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-114: SCB0\_LP0\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000113_5849ce93c423e057ee63a548bd410a87435a49b4479835c3e1085dca865a6083.png)

Table 45-118: SCB0\_LP0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-115: SCB0\_LP0\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000114_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-119: SCB0\_LP0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-116: SCB0\_LP1\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000115_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-120: SCB0\_LP1\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-117: SCB0\_LP1\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000116_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-121: SCB0\_LP1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-118: SCB0\_LP1\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000117_e9df2466b65193d3bfb785ca0112c4b3b9e85f48e4dc7659c01301bf01a419a7.png)

Table 45-122: SCB0\_LP1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-119: SCB0\_M85\_M2\_AXI\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000118_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-123: SCB0\_M85\_M2\_AXI\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-120: SCB0\_M85\_M2\_AXI\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000119_1aaa44c58a1b173dd77c14f0b923167b44976d1941de7dd84e0e11ebfb055fbb.png)

Table 45-124: SCB0\_M85\_M2\_AXI\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-121: SCB0\_M85\_M2\_AXI\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000120_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-125: SCB0\_M85\_M2\_AXI\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-122: SCB0\_M85\_M\_AXI\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000121_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-126: SCB0\_M85\_M\_AXI\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-123: SCB0\_M85\_M\_AXI\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000122_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-127: SCB0\_M85\_M\_AXI\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-124: SCB0\_M85\_M\_AXI\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000123_110c594bd99796bd699ff192e9a4ee3cc021de7ec875a017b7e2e28ef3b08294.png)

Table 45-128: SCB0\_M85\_M\_AXI\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-125: SCB0\_M85\_S2\_AXI\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000124_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-129: SCB0\_M85\_S2\_AXI\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-126: SCB0\_M85\_S2\_AXI\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000125_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-130: SCB0\_M85\_S2\_AXI\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-127: SCB0\_M85\_S\_AXI\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000126_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-131: SCB0\_M85\_S\_AXI\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-128: SCB0\_M85\_S\_AXI\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000127_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-132: SCB0\_M85\_S\_AXI\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-129: SCB0\_MLB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000128_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-133: SCB0\_MLB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-130: SCB0\_MLB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000129_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-134: SCB0\_MLB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-131: SCB0\_MLB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000130_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-135: SCB0\_MLB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-132: SCB0\_MSMDMA1\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000131_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-136: SCB0\_MSMDMA1\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-133: SCB0\_MSMDMA1\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000132_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-137: SCB0\_MSMDMA1\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-134: SCB0\_MSMDMA1\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000133_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-138: SCB0\_MSMDMA1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-135: SCB0\_MSMDMA1\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000134_ce769fbbec284a1f5c4fc834bda7478e3333536ab43d73b2d1bd1ba054b22aa9.png)

Table 45-139: SCB0\_MSMDMA1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-136: SCB0\_MSMDMA1\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000135_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-140: SCB0\_MSMDMA1\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-137: SCB0\_MSMDMA1\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000136_0343c99aeb8656582a6084d6339bb12c5839e895a33bfb86927d8e0d468372be.png)

Table 45-141: SCB0\_MSMDMA1\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-138: SCB0\_MSMDMA1\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000137_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-142: SCB0\_MSMDMA1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-139: SCB0\_MSMDMA1\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000138_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-143: SCB0\_MSMDMA1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-140: SCB0\_MSMDMA\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000139_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-144: SCB0\_MSMDMA\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-141: SCB0\_MSMDMA\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000140_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-145: SCB0\_MSMDMA\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-142: SCB0\_MSMDMA\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000141_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-146: SCB0\_MSMDMA\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-143: SCB0\_MSMDMA\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000142_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-147: SCB0\_MSMDMA\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-144: SCB0\_MSMDMA\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000143_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-148: SCB0\_MSMDMA\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-145: SCB0\_MSMDMA\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000144_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-149: SCB0\_MSMDMA\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-146: SCB0\_MSMDMA\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000145_ef95b8c520f0dd9c11743a1fe88444be92d90b8dd612df76fb9a4c46fcf2eb59.png)

Table 45-150: SCB0\_MSMDMA\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-147: SCB0\_MSMDMA\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000146_d709599c8547db4cc979a5bdb3656e45f509f72f32659a216a817097c5e72fe8.png)

Table 45-151: SCB0\_MSMDMA\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-148: SCB0\_OTP\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000147_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-152: SCB0\_OTP\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-149: SCB0\_OTP\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000148_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-153: SCB0\_OTP\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-150: SCB0\_PERIPH\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000149_6cd4b1233646db80e265e5d346cb6386207e27c8ae425133fdb2eeafcf7b0bbb.png)

Table 45-154: SCB0\_PERIPH\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_0 | .                         |

Figure 45-151: SCB0\_PERIPH\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000150_f617638916c1b4b2e09cf83c31f636ca98240a9abb3494aa2233fba71d62fbf4.png)

Table 45-155: SCB0\_PERIPH\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_1 | .                         |

Figure 45-152: SCB0\_PERIPH\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000151_974cbd7514c7872f7387da38ee73a420334d0f8588f2204444ae196c44b3951c.png)

Table 45-156: SCB0\_PERIPH\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_2 | .                         |

Figure 45-153: SCB0\_PERIPH\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000152_c47cfee77106da96abfd626d6ebc5b78c63750405c4cf286367fa34795e59a4f.png)

Table 45-157: SCB0\_PERIPH\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 7:4 (R/NW)         | REV_AND      | .                         |
| 3:0 (R/NW)         | CUST_MOD_NUM | .                         |

Figure 45-154: SCB0\_PERIPH\_ID\_4 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000153_d420f7e793a3ec845b538d24f74d8cfb6a017e7cc45424ed27d25826e99f79bf.png)

Table 45-158: SCB0\_PERIPH\_ID\_4 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_4 | .                         |

Figure 45-155: SCB0\_PERIPH\_ID\_5 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000154_82276f587d16b38e78abfe1c6f34b17b35f83dec31fb12f524705537d9078773.png)

Table 45-159: SCB0\_PERIPH\_ID\_5 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_5 | .                         |

Figure 45-156: SCB0\_PERIPH\_ID\_6 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000155_47f6a6936d7bb8ed4a961cef0a4f023e9d4834c6a76b979eb5e0b5ed421154f5.png)

Table 45-160: SCB0\_PERIPH\_ID\_6 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_6 | .                         |

Figure 45-157: SCB0\_PERIPH\_ID\_7 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000156_79c4b97cbaef5b50dd3fd9add7272283184a52c9504e75ce3065631e564d5a2e.png)

Table 45-161: SCB0\_PERIPH\_ID\_7 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_7 | .                         |

Figure 45-158: SCB0\_SCB6\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000157_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-162: SCB0\_SCB6\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-159: SCB0\_SCB6\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000158_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-163: SCB0\_SCB6\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-160: SCB0\_SCB6\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000159_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-164: SCB0\_SCB6\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-161: SCB0\_SCB6\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000160_008a96bde380efdfbb1ef41d2acfd61f4fe17f8534ecfe249dcaf7310cfa0333.png)

Table 45-165: SCB0\_SCB6\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-162: SCB0\_SH0\_DPORT\_L2\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000161_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-166: SCB0\_SH0\_DPORT\_L2\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-163: SCB0\_SH0\_DPORT\_L2\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000162_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-167: SCB0\_SH0\_DPORT\_L2\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-164: SCB0\_SH0\_DPORT\_L2\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000163_2d48b6eb52ec59c5dd7f0f28dec3da21fb00334cb3fd1132ff08df6bea801c93.png)

Table 45-168: SCB0\_SH0\_DPORT\_L2\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-165: SCB0\_SH0\_DPORT\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000164_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-169: SCB0\_SH0\_DPORT\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-166: SCB0\_SH0\_DPORT\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000165_1aaa44c58a1b173dd77c14f0b923167b44976d1941de7dd84e0e11ebfb055fbb.png)

Table 45-170: SCB0\_SH0\_DPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-167: SCB0\_SH0\_DPORT\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000166_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-171: SCB0\_SH0\_DPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-168: SCB0\_SH0\_FIR\_CH0\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000167_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-172: SCB0\_SH0\_FIR\_CH0\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-169: SCB0\_SH0\_FIR\_CH0\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000168_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-173: SCB0\_SH0\_FIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-170: SCB0\_SH0\_FIR\_CH0\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000169_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-174: SCB0\_SH0\_FIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-171: SCB0\_SH0\_FIR\_CH1\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000170_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-175: SCB0\_SH0\_FIR\_CH1\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-172: SCB0\_SH0\_FIR\_CH1\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000171_1aaa44c58a1b173dd77c14f0b923167b44976d1941de7dd84e0e11ebfb055fbb.png)

Table 45-176: SCB0\_SH0\_FIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-173: SCB0\_SH0\_FIR\_CH1\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000172_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-177: SCB0\_SH0\_FIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-174: SCB0\_SH0\_IIR\_CH0\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000173_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-178: SCB0\_SH0\_IIR\_CH0\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-175: SCB0\_SH0\_IIR\_CH0\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000174_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-179: SCB0\_SH0\_IIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-176: SCB0\_SH0\_IIR\_CH0\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000175_110c594bd99796bd699ff192e9a4ee3cc021de7ec875a017b7e2e28ef3b08294.png)

Table 45-180: SCB0\_SH0\_IIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-177: SCB0\_SH0\_IIR\_CH1\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000176_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-181: SCB0\_SH0\_IIR\_CH1\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-178: SCB0\_SH0\_IIR\_CH1\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000177_5849ce93c423e057ee63a548bd410a87435a49b4479835c3e1085dca865a6083.png)

Table 45-182: SCB0\_SH0\_IIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-179: SCB0\_SH0\_IIR\_CH1\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000178_57c3bf717878b5cc3f0568804eaa09c92e6bfa9861f2ce28e83804fc08a8a2a7.png)

Table 45-183: SCB0\_SH0\_IIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-180: SCB0\_SH0\_IPORT\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000179_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-184: SCB0\_SH0\_IPORT\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-181: SCB0\_SH0\_IPORT\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000180_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-185: SCB0\_SH0\_IPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-182: SCB0\_SH0\_IPORT\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000181_2d48b6eb52ec59c5dd7f0f28dec3da21fb00334cb3fd1132ff08df6bea801c93.png)

Table 45-186: SCB0\_SH0\_IPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-183: SCB0\_SH0\_MMR\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000182_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-187: SCB0\_SH0\_MMR\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-184: SCB0\_SH0\_MMR\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000183_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-188: SCB0\_SH0\_MMR\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-185: SCB0\_SH0\_MMR\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000184_9fc57c12a90ebd59ea1b7745293cb105e6e8229e7f286f57ccc0d4defdf7b56c.png)

Table 45-189: SCB0\_SH0\_MMR\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-186: SCB0\_SH0\_MMR\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000185_110c594bd99796bd699ff192e9a4ee3cc021de7ec875a017b7e2e28ef3b08294.png)

Table 45-190: SCB0\_SH0\_MMR\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-187: SCB0\_SH0\_S1PORT\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000186_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-191: SCB0\_SH0\_S1PORT\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-188: SCB0\_SH0\_S2PORT\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000187_3bcab2d51a690e90acf28824b61f27c1e77315f92d7fb407bbe6c1285510d966.png)

Table 45-192: SCB0\_SH0\_S2PORT\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-189: SCB0\_SMC\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000188_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-193: SCB0\_SMC\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-190: SCB0\_SMC\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000189_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-194: SCB0\_SMC\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-191: SCB0\_SMMR\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000190_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-195: SCB0\_SMMR\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-192: SCB0\_SP0A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000191_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-196: SCB0\_SP0A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-193: SCB0\_SP0A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000192_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-197: SCB0\_SP0A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-194: SCB0\_SP0A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000193_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-198: SCB0\_SP0A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-195: SCB0\_SP0B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000194_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-199: SCB0\_SP0B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-196: SCB0\_SP0B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000195_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-200: SCB0\_SP0B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-197: SCB0\_SP0B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000196_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-201: SCB0\_SP0B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-198: SCB0\_SP1A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000197_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-202: SCB0\_SP1A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-199: SCB0\_SP1A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000198_251d7ec0b19147ae29313238c5704be64c489868a36b52f7f50287a3d4820688.png)

Table 45-203: SCB0\_SP1A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-200: SCB0\_SP1A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000199_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-204: SCB0\_SP1A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-201: SCB0\_SP1B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000200_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-205: SCB0\_SP1B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-202: SCB0\_SP1B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000201_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-206: SCB0\_SP1B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-203: SCB0\_SP1B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000202_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-207: SCB0\_SP1B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-204: SCB0\_SP2A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000203_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-208: SCB0\_SP2A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-205: SCB0\_SP2A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000204_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-209: SCB0\_SP2A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-206: SCB0\_SP2A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000205_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-210: SCB0\_SP2A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-207: SCB0\_SP2B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000206_ecf3f2553821ba8202830de8cb11d549a3f4022437a8f2c456a45f60b30f1053.png)

Table 45-211: SCB0\_SP2B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-208: SCB0\_SP2B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000207_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-212: SCB0\_SP2B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-209: SCB0\_SP2B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000208_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-213: SCB0\_SP2B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-210: SCB0\_SP3A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000209_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-214: SCB0\_SP3A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-211: SCB0\_SP3A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000210_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-215: SCB0\_SP3A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-212: SCB0\_SP3A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000211_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-216: SCB0\_SP3A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-213: SCB0\_SP3B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000212_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-217: SCB0\_SP3B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-214: SCB0\_SP3B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000213_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-218: SCB0\_SP3B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-215: SCB0\_SP3B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000214_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-219: SCB0\_SP3B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-216: SCB0\_SP4A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000215_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-220: SCB0\_SP4A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-217: SCB0\_SP4A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000216_251d7ec0b19147ae29313238c5704be64c489868a36b52f7f50287a3d4820688.png)

Table 45-221: SCB0\_SP4A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-218: SCB0\_SP4A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000217_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-222: SCB0\_SP4A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-219: SCB0\_SP4B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000218_8c49690cb4e377a8a29b1ee1bbf64fbcfe593292b73cbc5bd513128cc660cd54.png)

Table 45-223: SCB0\_SP4B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-220: SCB0\_SP4B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000219_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-224: SCB0\_SP4B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-221: SCB0\_SP4B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000220_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-225: SCB0\_SP4B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-222: SCB0\_SP5A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000221_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-226: SCB0\_SP5A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-223: SCB0\_SP5A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000222_0af288c1a200c4f89a76f9247580f553fa4606e08833a9b60795266d2fe77188.png)

Table 45-227: SCB0\_SP5A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-224: SCB0\_SP5A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000223_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-228: SCB0\_SP5A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-225: SCB0\_SP5B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000224_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-229: SCB0\_SP5B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-226: SCB0\_SP5B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000225_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-230: SCB0\_SP5B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-227: SCB0\_SP5B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000226_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-231: SCB0\_SP5B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-228: SCB0\_SP6A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000227_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-232: SCB0\_SP6A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-229: SCB0\_SP6A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000228_251d7ec0b19147ae29313238c5704be64c489868a36b52f7f50287a3d4820688.png)

Table 45-233: SCB0\_SP6A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-230: SCB0\_SP6A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000229_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-234: SCB0\_SP6A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-231: SCB0\_SP6B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000230_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-235: SCB0\_SP6B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-232: SCB0\_SP6B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000231_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-236: SCB0\_SP6B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-233: SCB0\_SP6B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000232_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-237: SCB0\_SP6B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-234: SCB0\_SP7A\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000233_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-238: SCB0\_SP7A\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-235: SCB0\_SP7A\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000234_0af288c1a200c4f89a76f9247580f553fa4606e08833a9b60795266d2fe77188.png)

Table 45-239: SCB0\_SP7A\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-236: SCB0\_SP7A\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000235_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-240: SCB0\_SP7A\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-237: SCB0\_SP7B\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000236_8c49690cb4e377a8a29b1ee1bbf64fbcfe593292b73cbc5bd513128cc660cd54.png)

Table 45-241: SCB0\_SP7B\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-238: SCB0\_SP7B\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000237_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-242: SCB0\_SP7B\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-239: SCB0\_SP7B\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000238_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-243: SCB0\_SP7B\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-240: SCB0\_SPI0RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000239_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-244: SCB0\_SPI0RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-241: SCB0\_SPI0RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000240_5262e0bed6da45ff45c21a2bcaed25719b8ad6ad7acda5506ababa89248b9de9.png)

Table 45-245: SCB0\_SPI0RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-242: SCB0\_SPI0RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000241_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-246: SCB0\_SPI0RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-243: SCB0\_SPI0TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000242_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-247: SCB0\_SPI0TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-244: SCB0\_SPI0TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000243_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-248: SCB0\_SPI0TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-245: SCB0\_SPI0TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000244_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-249: SCB0\_SPI0TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-246: SCB0\_SPI1RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000245_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-250: SCB0\_SPI1RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-247: SCB0\_SPI1RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000246_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-251: SCB0\_SPI1RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-248: SCB0\_SPI1RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000247_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-252: SCB0\_SPI1RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-249: SCB0\_SPI1TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000248_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-253: SCB0\_SPI1TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-250: SCB0\_SPI1TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000249_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-254: SCB0\_SPI1TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-251: SCB0\_SPI1TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000250_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-255: SCB0\_SPI1TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-252: SCB0\_SPI2RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000251_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-256: SCB0\_SPI2RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-253: SCB0\_SPI2RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000252_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-257: SCB0\_SPI2RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-254: SCB0\_SPI2RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000253_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-258: SCB0\_SPI2RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-255: SCB0\_SPI2TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000254_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-259: SCB0\_SPI2TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-256: SCB0\_SPI2TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000255_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-260: SCB0\_SPI2TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-257: SCB0\_SPI2TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000256_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-261: SCB0\_SPI2TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-258: SCB0\_SPI5RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000257_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-262: SCB0\_SPI5RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-259: SCB0\_SPI5RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000258_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-263: SCB0\_SPI5RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-260: SCB0\_SPI5RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000259_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-264: SCB0\_SPI5RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-261: SCB0\_SPI5TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000260_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-265: SCB0\_SPI5TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-262: SCB0\_SPI5TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000261_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-266: SCB0\_SPI5TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-263: SCB0\_SPI5TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000262_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-267: SCB0\_SPI5TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-264: SCB0\_SPIF\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000263_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-268: SCB0\_SPIF\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-265: SCB0\_UART0\_RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000264_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-269: SCB0\_UART0\_RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-266: SCB0\_UART0\_RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000265_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-270: SCB0\_UART0\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-267: SCB0\_UART0\_RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000266_bdb7f648f1e747d4f44b8310877eca36ce0f25c08f36dafa1f4df8eb1011748b.png)

Table 45-271: SCB0\_UART0\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-268: SCB0\_UART0\_TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000267_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-272: SCB0\_UART0\_TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-269: SCB0\_UART0\_TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000268_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-273: SCB0\_UART0\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-270: SCB0\_UART0\_TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000269_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-274: SCB0\_UART0\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-271: SCB0\_UART1\_RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000270_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-275: SCB0\_UART1\_RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-272: SCB0\_UART1\_RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000271_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-276: SCB0\_UART1\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-273: SCB0\_UART1\_RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000272_bdb7f648f1e747d4f44b8310877eca36ce0f25c08f36dafa1f4df8eb1011748b.png)

Table 45-277: SCB0\_UART1\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-274: SCB0\_UART1\_TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000273_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-278: SCB0\_UART1\_TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-275: SCB0\_UART1\_TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000274_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-279: SCB0\_UART1\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-276: SCB0\_UART1\_TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000275_3be6346b51de0d490f9eab37bde8308e63eb75b3d82a32ba3684e083b12b396b.png)

Table 45-280: SCB0\_UART1\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-277: SCB0\_UART2\_RX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000276_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-281: SCB0\_UART2\_RX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-278: SCB0\_UART2\_RX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000277_5262e0bed6da45ff45c21a2bcaed25719b8ad6ad7acda5506ababa89248b9de9.png)

Table 45-282: SCB0\_UART2\_RX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-279: SCB0\_UART2\_RX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000278_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-283: SCB0\_UART2\_RX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-280: SCB0\_UART2\_TX\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000279_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-284: SCB0\_UART2\_TX\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-281: SCB0\_UART2\_TX\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000280_5262e0bed6da45ff45c21a2bcaed25719b8ad6ad7acda5506ababa89248b9de9.png)

Table 45-285: SCB0\_UART2\_TX\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-282: SCB0\_UART2\_TX\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000281_3be6346b51de0d490f9eab37bde8308e63eb75b3d82a32ba3684e083b12b396b.png)

Table 45-286: SCB0\_UART2\_TX\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-283: SCB0\_XSPI\_M1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000282_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-287: SCB0\_XSPI\_M1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-284: SCB0\_XSPI\_M1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000283_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-288: SCB0\_XSPI\_M1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-285: SCB0\_XSPI\_M\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000284_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-289: SCB0\_XSPI\_M\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-286: SCB0\_XSPI\_M\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000285_c1bd8692af8601ded35ed5ca2a26247aa70ed356f0848cafa22b186710660b81.png)

Table 45-290: SCB0\_XSPI\_M\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

## ADSP-2184x SCB1 Register Descriptions

Your module description, here. (SCB1) contains the following registers.

Table 45-291: ADSP-2184x SCB1 Register List

| Name                  | Description   |
|-----------------------|---------------|
| SCB1_MST_IB_SYNC_MODE |               |

Figure 45-287: SCB1\_MST\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000286_ebda0bd9d44b85d3cd8f65675ee8a12aa6d7e28444ceaa2e0dd6df5c0b0d19cd.png)

Table 45-292: SCB1\_MST\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

## ADSP-2184x SCB3 Register Descriptions

Your module description, here. (SCB3) contains the following registers.

Table 45-293: ADSP-2184x SCB3 Register List

| Name                              | Description   |
|-----------------------------------|---------------|
| SCB3_APB_CLKO3_IB_FN_MOD_ISS_BM   |               |
| SCB3_APB_CLKO3_IB_SYNC_MODE       |               |
| SCB3_APB_CLKO4_IB_FN_MOD_ISS_BM   |               |
| SCB3_APB_CLKO4_IB_SYNC_MODE       |               |
| SCB3_APB_CLKO6_IB_FN_MOD_ISS_BM   |               |
| SCB3_APB_CLKO8_IB_FN_MOD_ISS_BM   |               |
| SCB3_APB_CLKO8_IB_SYNC_MODE       |               |
| SCB3_APB_CLKPWM_IB_FN_MOD_ISS_BM  |               |
| SCB3_APB_CLKPWM_IB_SYNC_MODE      |               |
| SCB3_APB_SCLK0_0_IB_FN_MOD_ISS_BM |               |
| SCB3_APB_SCLK0_1_IB_FN_MOD_ISS_BM |               |
| SCB3_COMP_ID_0                    |               |
| SCB3_COMP_ID_1                    |               |
| SCB3_COMP_ID_2                    |               |
| SCB3_COMP_ID_3                    |               |
| SCB3_CRYPTO_0_AHB_CNTL            |               |

Table 45-293: ADSP-2184x SCB3 Register List (Continued)

| Name                           | Description   |
|--------------------------------|---------------|
| SCB3_CRYPTO_0_FN_MOD_ISS_BM    |               |
| SCB3_CRYPTO_1_AHB_CNTL         |               |
| SCB3_CRYPTO_1_FN_MOD_ISS_BM    |               |
| SCB3_DAI0_AHB_CNTL             |               |
| SCB3_DAI0_FN_MOD_ISS_BM        |               |
| SCB3_DAI1_AHB_CNTL             |               |
| SCB3_DAI1_FN_MOD_ISS_BM        |               |
| SCB3_DLMDMA0_AHB_CNTL          |               |
| SCB3_DLMDMA0_FN_MOD_ISS_BM     |               |
| SCB3_PERIPH_ID_0               |               |
| SCB3_PERIPH_ID_1               |               |
| SCB3_PERIPH_ID_2               |               |
| SCB3_PERIPH_ID_3               |               |
| SCB3_PERIPH_ID_4               |               |
| SCB3_PERIPH_ID_5               |               |
| SCB3_PERIPH_ID_6               |               |
| SCB3_PERIPH_ID_7               |               |
| SCB3_SMMR_FN_MOD               |               |
| SCB3_SPIF_FABRIC_FN_MOD_ISS_BM |               |

Figure 45-288: SCB3\_APB\_CLKO3\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000287_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-294: SCB3\_APB\_CLKO3\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-289: SCB3\_APB\_CLKO3\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000288_f1d06c3722f7cc41a64fa1174b90cffa9f6efa263462614dc8b3c8eeb27a06d4.png)

Table 45-295: SCB3\_APB\_CLKO3\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-290: SCB3\_APB\_CLKO4\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000289_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-296: SCB3\_APB\_CLKO4\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-291: SCB3\_APB\_CLKO4\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000290_f1d06c3722f7cc41a64fa1174b90cffa9f6efa263462614dc8b3c8eeb27a06d4.png)

Table 45-297: SCB3\_APB\_CLKO4\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-292: SCB3\_APB\_CLKO6\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000291_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-298: SCB3\_APB\_CLKO6\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-293: SCB3\_APB\_CLKO8\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000292_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-299: SCB3\_APB\_CLKO8\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-294: SCB3\_APB\_CLKO8\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000293_8ade5e9c1767fb4c934b968597ba329076430b062dd4bb68a99e2e5dcdae66f7.png)

Table 45-300: SCB3\_APB\_CLKO8\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-295: SCB3\_APB\_CLKPWM\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000294_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-301: SCB3\_APB\_CLKPWM\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-296: SCB3\_APB\_CLKPWM\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000295_1e8a4dd1777ae825ec8a42491440c1adecc6fce467261d87a824af969133c0a6.png)

Table 45-302: SCB3\_APB\_CLKPWM\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-297: SCB3\_APB\_SCLK0\_0\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000296_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-303: SCB3\_APB\_SCLK0\_0\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-298: SCB3\_APB\_SCLK0\_1\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000297_739fa3d2a6c7e46256f29e3d39c06bc77a8e3671d4b793af6c6a997703600802.png)

Table 45-304: SCB3\_APB\_SCLK0\_1\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-299: SCB3\_COMP\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000298_727d77b688abfcef3aca3b3adf15fb7eacbc3a93129741ec784511ab54f31b10.png)

Table 45-305: SCB3\_COMP\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_0  | .                         |
| (R/NW)             |            |                           |

Figure 45-300: SCB3\_COMP\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000299_f577545b86556c5e6282b54ebeee76b9248c0ce75901112ce704c9d74ff2691d.png)

Table 45-306: SCB3\_COMP\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_1  | .                         |

Figure 45-301: SCB3\_COMP\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000300_9c9f43ef0871cd2a16f56a41ac7f9bc2179809303e121dc2741a1a2d4e6adbf5.png)

Table 45-307: SCB3\_COMP\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_2  | .                         |
| (R/NW)             |            |                           |

Figure 45-302: SCB3\_COMP\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000301_5cdefaec630838917a2dae9202aadf694f3e61fe6666257163ef4b3c2506880f.png)

Table 45-308: SCB3\_COMP\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_3  | .                         |
| (R/NW)             |            |                           |

Figure 45-303: SCB3\_CRYPTO\_0\_AHB\_CNTL Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000302_9e6225de416537f0aaf3aced8ab0086c3db8f7099bfc1a3e4b978bbf77f1a7a8.png)

Table 45-309: SCB3\_CRYPTO\_0\_AHB\_CNTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/W)            | FORCE_INCR | .                         |
| 0 (R/W)            | DECERR_EN  | .                         |

Figure 45-304: SCB3\_CRYPTO\_0\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000303_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-310: SCB3\_CRYPTO\_0\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-305: SCB3\_CRYPTO\_1\_AHB\_CNTL Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000304_9e6225de416537f0aaf3aced8ab0086c3db8f7099bfc1a3e4b978bbf77f1a7a8.png)

Table 45-311: SCB3\_CRYPTO\_1\_AHB\_CNTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/W)            | FORCE_INCR | .                         |
| 0 (R/W)            | DECERR_EN  | .                         |

Figure 45-306: SCB3\_CRYPTO\_1\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000305_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-312: SCB3\_CRYPTO\_1\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-307: SCB3\_DAI0\_AHB\_CNTL Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000306_9e6225de416537f0aaf3aced8ab0086c3db8f7099bfc1a3e4b978bbf77f1a7a8.png)

Table 45-313: SCB3\_DAI0\_AHB\_CNTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/W)            | FORCE_INCR | .                         |
| 0 (R/W)            | DECERR_EN  | .                         |

Figure 45-308: SCB3\_DAI0\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000307_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-314: SCB3\_DAI0\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-309: SCB3\_DAI1\_AHB\_CNTL Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000308_9e6225de416537f0aaf3aced8ab0086c3db8f7099bfc1a3e4b978bbf77f1a7a8.png)

Table 45-315: SCB3\_DAI1\_AHB\_CNTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/W)            | FORCE_INCR | .                         |
| 0 (R/W)            | DECERR_EN  | .                         |

Figure 45-310: SCB3\_DAI1\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000309_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-316: SCB3\_DAI1\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-311: SCB3\_DLMDMA0\_AHB\_CNTL Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000310_9e6225de416537f0aaf3aced8ab0086c3db8f7099bfc1a3e4b978bbf77f1a7a8.png)

Table 45-317: SCB3\_DLMDMA0\_AHB\_CNTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1 (R/W)            | FORCE_INCR | .                         |
| 0 (R/W)            | DECERR_EN  | .                         |

Figure 45-312: SCB3\_DLMDMA0\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000311_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-318: SCB3\_DLMDMA0\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-313: SCB3\_PERIPH\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000312_6cd4b1233646db80e265e5d346cb6386207e27c8ae425133fdb2eeafcf7b0bbb.png)

Table 45-319: SCB3\_PERIPH\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_0 | .                         |

Figure 45-314: SCB3\_PERIPH\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000313_f617638916c1b4b2e09cf83c31f636ca98240a9abb3494aa2233fba71d62fbf4.png)

Table 45-320: SCB3\_PERIPH\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_1 | .                         |

Figure 45-315: SCB3\_PERIPH\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000314_974cbd7514c7872f7387da38ee73a420334d0f8588f2204444ae196c44b3951c.png)

Table 45-321: SCB3\_PERIPH\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_2 | .                         |

Figure 45-316: SCB3\_PERIPH\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000315_c47cfee77106da96abfd626d6ebc5b78c63750405c4cf286367fa34795e59a4f.png)

Table 45-322: SCB3\_PERIPH\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 7:4 (R/NW)         | REV_AND      | .                         |
| 3:0 (R/NW)         | CUST_MOD_NUM | .                         |

Figure 45-317: SCB3\_PERIPH\_ID\_4 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000316_d420f7e793a3ec845b538d24f74d8cfb6a017e7cc45424ed27d25826e99f79bf.png)

Table 45-323: SCB3\_PERIPH\_ID\_4 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_4 | .                         |

Figure 45-318: SCB3\_PERIPH\_ID\_5 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000317_75d87f200d7e299012f2a2604da7c8910227061d6795a9bd5cb8266fa19d0281.png)

Table 45-324: SCB3\_PERIPH\_ID\_5 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_5 | .                         |

Figure 45-319: SCB3\_PERIPH\_ID\_6 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000318_47f6a6936d7bb8ed4a961cef0a4f023e9d4834c6a76b979eb5e0b5ed421154f5.png)

Table 45-325: SCB3\_PERIPH\_ID\_6 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_6 | .                         |

Figure 45-320: SCB3\_PERIPH\_ID\_7 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000319_6eb564389c1779ca7a57e7b5f44059087c8add8f0966fcb9abcd433f537ba5a2.png)

Table 45-326: SCB3\_PERIPH\_ID\_7 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_7 | .                         |

Figure 45-321: SCB3\_SMMR\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000320_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-327: SCB3\_SMMR\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-322: SCB3\_SPIF\_FABRIC\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000321_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-328: SCB3\_SPIF\_FABRIC\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

## ADSP-2184x SCB4 Register Descriptions

Your module description, here. (SCB4) contains the following registers.

Table 45-329: ADSP-2184x SCB4 Register List

| Name                                 | Description   |
|--------------------------------------|---------------|
| SCB4_COMP_ID_0                       |               |
| SCB4_COMP_ID_1                       |               |
| SCB4_COMP_ID_2                       |               |
| SCB4_COMP_ID_3                       |               |
| SCB4_FABRIC_FIR_CH0_IB_FN_MOD        |               |
| SCB4_FABRIC_FIR_CH0_IB_FN_MOD_ISS_BM |               |
| SCB4_FABRIC_FIR_CH0_IB_SYNC_MODE     |               |
| SCB4_FABRIC_FIR_CH1_IB_FN_MOD        |               |
| SCB4_FABRIC_FIR_CH1_IB_FN_MOD_ISS_BM |               |
| SCB4_FABRIC_FIR_CH1_IB_SYNC_MODE     |               |
| SCB4_FABRIC_IIR_CH0_IB_FN_MOD        |               |
| SCB4_FABRIC_IIR_CH0_IB_FN_MOD_ISS_BM |               |
| SCB4_FABRIC_IIR_CH0_IB_SYNC_MODE     |               |
| SCB4_FABRIC_IIR_CH1_IB_FN_MOD        |               |
| SCB4_FABRIC_IIR_CH1_IB_FN_MOD_ISS_BM |               |
| SCB4_FABRIC_IIR_CH1_IB_SYNC_MODE     |               |

Table 45-329: ADSP-2184x SCB4 Register List (Continued)

| Name                             | Description   |
|----------------------------------|---------------|
| SCB4_FABRIC_MMR_IB_FN_MOD        |               |
| SCB4_FABRIC_MMR_IB_FN_MOD2       |               |
| SCB4_FABRIC_MMR_IB_FN_MOD_ISS_BM |               |
| SCB4_FABRIC_MMR_IB_SYNC_MODE     |               |
| SCB4_FABRIC_S1PORT_IB_FN_MOD     |               |
| SCB4_FABRIC_S1PORT_IB_READ_QOS   |               |
| SCB4_FABRIC_S1PORT_IB_SYNC_MODE  |               |
| SCB4_FABRIC_S1PORT_IB_WRITE_QOS  |               |
| SCB4_FABRIC_S2PORT_IB_FN_MOD     |               |
| SCB4_FABRIC_S2PORT_IB_READ_QOS   |               |
| SCB4_FABRIC_S2PORT_IB_SYNC_MODE  |               |
| SCB4_FABRIC_S2PORT_IB_WRITE_QOS  |               |
| SCB4_FIR1_CH0_FN_MOD             |               |
| SCB4_FIR1_CH0_READ_QOS           |               |
| SCB4_FIR1_CH0_WRITE_QOS          |               |
| SCB4_FIR1_CH1_FN_MOD             |               |
| SCB4_FIR1_CH1_READ_QOS           |               |
| SCB4_FIR1_CH1_WRITE_QOS          |               |
| SCB4_FIR_CH0_FN_MOD              |               |
| SCB4_FIR_CH0_READ_QOS            |               |
| SCB4_FIR_CH0_WRITE_QOS           |               |
| SCB4_FIR_CH1_FN_MOD              |               |
| SCB4_FIR_CH1_READ_QOS            |               |
| SCB4_FIR_CH1_WRITE_QOS           |               |
| SCB4_IDMA_IB_FN_MOD2             |               |
| SCB4_IDMA_IB_SYNC_MODE           |               |
| SCB4_IIR0_CH0_IB_FN_MOD          |               |
| SCB4_IIR0_CH0_IB_FN_MOD2         |               |
| SCB4_IIR0_CH0_IB_READ_QOS        |               |
| SCB4_IIR0_CH0_IB_WRITE_QOS       |               |
| SCB4_IIR0_CH1_IB_FN_MOD          |               |
| SCB4_IIR0_CH1_IB_FN_MOD2         |               |

Table 45-329: ADSP-2184x SCB4 Register List (Continued)

| Name                       | Description   |
|----------------------------|---------------|
| SCB4_IIR0_CH1_IB_READ_QOS  |               |
| SCB4_IIR0_CH1_IB_WRITE_QOS |               |
| SCB4_IIR1_CH0_IB_FN_MOD    |               |
| SCB4_IIR1_CH0_IB_FN_MOD2   |               |
| SCB4_IIR1_CH0_IB_READ_QOS  |               |
| SCB4_IIR1_CH0_IB_WRITE_QOS |               |
| SCB4_IIR1_CH1_IB_FN_MOD    |               |
| SCB4_IIR1_CH1_IB_FN_MOD2   |               |
| SCB4_IIR1_CH1_IB_READ_QOS  |               |
| SCB4_IIR1_CH1_IB_WRITE_QOS |               |
| SCB4_IIR2_CH0_IB_FN_MOD    |               |
| SCB4_IIR2_CH0_IB_FN_MOD2   |               |
| SCB4_IIR2_CH0_IB_READ_QOS  |               |
| SCB4_IIR2_CH0_IB_WRITE_QOS |               |
| SCB4_IIR2_CH1_IB_FN_MOD    |               |
| SCB4_IIR2_CH1_IB_FN_MOD2   |               |
| SCB4_IIR2_CH1_IB_READ_QOS  |               |
| SCB4_IIR2_CH1_IB_WRITE_QOS |               |
| SCB4_IIR3_CH0_IB_FN_MOD    |               |
| SCB4_IIR3_CH0_IB_FN_MOD2   |               |
| SCB4_IIR3_CH0_IB_READ_QOS  |               |
| SCB4_IIR3_CH0_IB_WRITE_QOS |               |
| SCB4_IIR3_CH1_IB_FN_MOD    |               |
| SCB4_IIR3_CH1_IB_FN_MOD2   |               |
| SCB4_IIR3_CH1_IB_READ_QOS  |               |
| SCB4_IIR3_CH1_IB_WRITE_QOS |               |
| SCB4_PERIPH_ID_0           |               |
| SCB4_PERIPH_ID_1           |               |
| SCB4_PERIPH_ID_2           |               |
| SCB4_PERIPH_ID_3           |               |
| SCB4_PERIPH_ID_4           |               |
| SCB4_PERIPH_ID_5           |               |

Table 45-329: ADSP-2184x SCB4 Register List (Continued)

| Name                                        | Description   |
|---------------------------------------------|---------------|
| SCB4_PERIPH_ID_6                            |               |
| SCB4_PERIPH_ID_7                            |               |
| SCB4_REMAP                                  |               |
| SCB4_SHARC_DPORT_FN_MOD                     |               |
| SCB4_SHARC_DPORT_READ_QOS                   |               |
| SCB4_SHARC_DPORT_TO_SF_L2_IB_FN_MOD         |               |
| SCB4_SHARC_DPORT_TO_SF_L2_IB_FN_MOD_I SS_BM |               |
| SCB4_SHARC_DPORT_TO_SF_L2_IB_SYNC_MOD E     |               |
| SCB4_SHARC_DPORT_TO_SF_IB_FN_MOD            |               |
| SCB4_SHARC_DPORT_TO_SF_IB_FN_MOD_ISS_ BM    |               |
| SCB4_SHARC_DPORT_TO_SF_IB_SYNC_MODE         |               |
| SCB4_SHARC_DPORT_WRITE_QOS                  |               |
| SCB4_SHARC_IPORT_IB_FN_MOD2                 |               |
| SCB4_SHARC_IPORT_IB_SYNC_MODE               |               |

Figure 45-323: SCB4\_COMP\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000322_fecfc0473ec62b9c7e372fd861d57cb6139a8836aa38b8d67e23f4ec8bc897da.png)

Table 45-330: SCB4\_COMP\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_0  | .                         |
| (R/NW)             |            |                           |

Figure 45-324: SCB4\_COMP\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000323_0c6358edd6bb0b55252666e64f84d693d96f49320192cdcaa21cf3af93d0047a.png)

Table 45-331: SCB4\_COMP\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_1  | .                         |

Figure 45-325: SCB4\_COMP\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000324_98f06e00f03d25a081cc2f3fc8e20df0941ab3fce0d09e8a1d142823cf332ab0.png)

Table 45-332: SCB4\_COMP\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_2  | .                         |
| (R/NW)             |            |                           |

Figure 45-326: SCB4\_COMP\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000325_53cde00cc5ee17124fb968b9465a049344dacdd7044ae0fb14336394d266e952.png)

Table 45-333: SCB4\_COMP\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_3  | .                         |
| (R/NW)             |            |                           |

Figure 45-327: SCB4\_FABRIC\_FIR\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000326_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-334: SCB4\_FABRIC\_FIR\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-328: SCB4\_FABRIC\_FIR\_CH0\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000327_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-335: SCB4\_FABRIC\_FIR\_CH0\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-329: SCB4\_FABRIC\_FIR\_CH0\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000328_1e8a4dd1777ae825ec8a42491440c1adecc6fce467261d87a824af969133c0a6.png)

Table 45-336: SCB4\_FABRIC\_FIR\_CH0\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-330: SCB4\_FABRIC\_FIR\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000329_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-337: SCB4\_FABRIC\_FIR\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-331: SCB4\_FABRIC\_FIR\_CH1\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000330_59d5c33487e8b26fea901557c89344cc3ca18512c6e8ded5829dc20e2afdb6ee.png)

Table 45-338: SCB4\_FABRIC\_FIR\_CH1\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-332: SCB4\_FABRIC\_FIR\_CH1\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000331_008a96bde380efdfbb1ef41d2acfd61f4fe17f8534ecfe249dcaf7310cfa0333.png)

Table 45-339: SCB4\_FABRIC\_FIR\_CH1\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-333: SCB4\_FABRIC\_IIR\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000332_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-340: SCB4\_FABRIC\_IIR\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-334: SCB4\_FABRIC\_IIR\_CH0\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000333_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-341: SCB4\_FABRIC\_IIR\_CH0\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-335: SCB4\_FABRIC\_IIR\_CH0\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000334_1e8a4dd1777ae825ec8a42491440c1adecc6fce467261d87a824af969133c0a6.png)

Table 45-342: SCB4\_FABRIC\_IIR\_CH0\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-336: SCB4\_FABRIC\_IIR\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000335_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-343: SCB4\_FABRIC\_IIR\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-337: SCB4\_FABRIC\_IIR\_CH1\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000336_68a5ed2c574ca5332417e4d0d70a45556780b9ee58e0b0a9d23ca05c36cd9a47.png)

Table 45-344: SCB4\_FABRIC\_IIR\_CH1\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-338: SCB4\_FABRIC\_IIR\_CH1\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000337_8ade5e9c1767fb4c934b968597ba329076430b062dd4bb68a99e2e5dcdae66f7.png)

Table 45-345: SCB4\_FABRIC\_IIR\_CH1\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-339: SCB4\_FABRIC\_MMR\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000338_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-346: SCB4\_FABRIC\_MMR\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-340: SCB4\_FABRIC\_MMR\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000339_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-347: SCB4\_FABRIC\_MMR\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-341: SCB4\_FABRIC\_MMR\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000340_59d5c33487e8b26fea901557c89344cc3ca18512c6e8ded5829dc20e2afdb6ee.png)

Table 45-348: SCB4\_FABRIC\_MMR\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-342: SCB4\_FABRIC\_MMR\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000341_f1d06c3722f7cc41a64fa1174b90cffa9f6efa263462614dc8b3c8eeb27a06d4.png)

Table 45-349: SCB4\_FABRIC\_MMR\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-343: SCB4\_FABRIC\_S1PORT\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000342_10f06ec60b77d1f42791d57fcde6949048b4ea969b637ba63a882dea7150748c.png)

Table 45-350: SCB4\_FABRIC\_S1PORT\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-344: SCB4\_FABRIC\_S1PORT\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000343_5262e0bed6da45ff45c21a2bcaed25719b8ad6ad7acda5506ababa89248b9de9.png)

Table 45-351: SCB4\_FABRIC\_S1PORT\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-345: SCB4\_FABRIC\_S1PORT\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000344_1e8a4dd1777ae825ec8a42491440c1adecc6fce467261d87a824af969133c0a6.png)

Table 45-352: SCB4\_FABRIC\_S1PORT\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-346: SCB4\_FABRIC\_S1PORT\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000345_e0f069b6fa3c696905b9e1b7be487908d018401522d6a35f75a998fcaab1c1be.png)

Table 45-353: SCB4\_FABRIC\_S1PORT\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-347: SCB4\_FABRIC\_S2PORT\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000346_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-354: SCB4\_FABRIC\_S2PORT\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-348: SCB4\_FABRIC\_S2PORT\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000347_5262e0bed6da45ff45c21a2bcaed25719b8ad6ad7acda5506ababa89248b9de9.png)

Table 45-355: SCB4\_FABRIC\_S2PORT\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-349: SCB4\_FABRIC\_S2PORT\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000348_2b377d5f8f24ff0ceb02d67aecf28ce2e5119c94673d1451fef0774878c1665b.png)

Table 45-356: SCB4\_FABRIC\_S2PORT\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-350: SCB4\_FABRIC\_S2PORT\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000349_e0f069b6fa3c696905b9e1b7be487908d018401522d6a35f75a998fcaab1c1be.png)

Table 45-357: SCB4\_FABRIC\_S2PORT\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-351: SCB4\_FIR1\_CH0\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000350_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-358: SCB4\_FIR1\_CH0\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-352: SCB4\_FIR1\_CH0\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000351_447685ba079ac80f63c8eca559693d77e80b75912c186a65d6a04df3469dbdb2.png)

Table 45-359: SCB4\_FIR1\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-353: SCB4\_FIR1\_CH0\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000352_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-360: SCB4\_FIR1\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-354: SCB4\_FIR1\_CH1\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000353_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-361: SCB4\_FIR1\_CH1\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-355: SCB4\_FIR1\_CH1\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000354_0af288c1a200c4f89a76f9247580f553fa4606e08833a9b60795266d2fe77188.png)

Table 45-362: SCB4\_FIR1\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-356: SCB4\_FIR1\_CH1\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000355_e0f069b6fa3c696905b9e1b7be487908d018401522d6a35f75a998fcaab1c1be.png)

Table 45-363: SCB4\_FIR1\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-357: SCB4\_FIR\_CH0\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000356_a1b2bc29d1556a3ebe11ee53d6449a7d830803ffe31bfafb15d518e54c151290.png)

Table 45-364: SCB4\_FIR\_CH0\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-358: SCB4\_FIR\_CH0\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000357_5262e0bed6da45ff45c21a2bcaed25719b8ad6ad7acda5506ababa89248b9de9.png)

Table 45-365: SCB4\_FIR\_CH0\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-359: SCB4\_FIR\_CH0\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000358_19148bf40115d7ec79ae8ab7a12406d8743466c8943563dfea6100bd9bb14cef.png)

Table 45-366: SCB4\_FIR\_CH0\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-360: SCB4\_FIR\_CH1\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000359_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-367: SCB4\_FIR\_CH1\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-361: SCB4\_FIR\_CH1\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000360_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-368: SCB4\_FIR\_CH1\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-362: SCB4\_FIR\_CH1\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000361_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-369: SCB4\_FIR\_CH1\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-363: SCB4\_IDMA\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000362_5625c7df2eaa854b5b3f338445835352bd011b45cfcb845ba6acf4eb5705958c.png)

Table 45-370: SCB4\_IDMA\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-364: SCB4\_IDMA\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000363_008a96bde380efdfbb1ef41d2acfd61f4fe17f8534ecfe249dcaf7310cfa0333.png)

Table 45-371: SCB4\_IDMA\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-365: SCB4\_IIR0\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000364_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-372: SCB4\_IIR0\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-366: SCB4\_IIR0\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000365_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-373: SCB4\_IIR0\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-367: SCB4\_IIR0\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000366_251d7ec0b19147ae29313238c5704be64c489868a36b52f7f50287a3d4820688.png)

Table 45-374: SCB4\_IIR0\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-368: SCB4\_IIR0\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000367_3be6346b51de0d490f9eab37bde8308e63eb75b3d82a32ba3684e083b12b396b.png)

Table 45-375: SCB4\_IIR0\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-369: SCB4\_IIR0\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000368_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-376: SCB4\_IIR0\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-370: SCB4\_IIR0\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000369_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-377: SCB4\_IIR0\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-371: SCB4\_IIR0\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000370_bd421545c05a06db8aa6e7b502e9a23b52798461d4c3f6a90c3b12bb739fe9e2.png)

Table 45-378: SCB4\_IIR0\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-372: SCB4\_IIR0\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000371_e0f069b6fa3c696905b9e1b7be487908d018401522d6a35f75a998fcaab1c1be.png)

Table 45-379: SCB4\_IIR0\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-373: SCB4\_IIR1\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000372_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-380: SCB4\_IIR1\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-374: SCB4\_IIR1\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000373_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-381: SCB4\_IIR1\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-375: SCB4\_IIR1\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000374_0af288c1a200c4f89a76f9247580f553fa4606e08833a9b60795266d2fe77188.png)

Table 45-382: SCB4\_IIR1\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-376: SCB4\_IIR1\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000375_f265f216d17acc5c0e7b49331d470949f9308786a47a7c58f3729bfe34dc02a6.png)

Table 45-383: SCB4\_IIR1\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-377: SCB4\_IIR1\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000376_a1b2bc29d1556a3ebe11ee53d6449a7d830803ffe31bfafb15d518e54c151290.png)

Table 45-384: SCB4\_IIR1\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-378: SCB4\_IIR1\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000377_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-385: SCB4\_IIR1\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-379: SCB4\_IIR1\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000378_0af288c1a200c4f89a76f9247580f553fa4606e08833a9b60795266d2fe77188.png)

Table 45-386: SCB4\_IIR1\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-380: SCB4\_IIR1\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000379_3be6346b51de0d490f9eab37bde8308e63eb75b3d82a32ba3684e083b12b396b.png)

Table 45-387: SCB4\_IIR1\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-381: SCB4\_IIR2\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000380_1e0bff04549acf0dacdca375647cd594b805481ee849a0447c16dc97975d7435.png)

Table 45-388: SCB4\_IIR2\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-382: SCB4\_IIR2\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000381_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-389: SCB4\_IIR2\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-383: SCB4\_IIR2\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000382_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-390: SCB4\_IIR2\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-384: SCB4\_IIR2\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000383_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-391: SCB4\_IIR2\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-385: SCB4\_IIR2\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000384_ade8e1c2b117ddc9ed1ede1eef004eaac282bb1862763ca57d57af9dda411b54.png)

Table 45-392: SCB4\_IIR2\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-386: SCB4\_IIR2\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000385_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-393: SCB4\_IIR2\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-387: SCB4\_IIR2\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000386_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-394: SCB4\_IIR2\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-388: SCB4\_IIR2\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000387_3be6346b51de0d490f9eab37bde8308e63eb75b3d82a32ba3684e083b12b396b.png)

Table 45-395: SCB4\_IIR2\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-389: SCB4\_IIR3\_CH0\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000388_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-396: SCB4\_IIR3\_CH0\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-390: SCB4\_IIR3\_CH0\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000389_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-397: SCB4\_IIR3\_CH0\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-391: SCB4\_IIR3\_CH0\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000390_bd421545c05a06db8aa6e7b502e9a23b52798461d4c3f6a90c3b12bb739fe9e2.png)

Table 45-398: SCB4\_IIR3\_CH0\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-392: SCB4\_IIR3\_CH0\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000391_3be6346b51de0d490f9eab37bde8308e63eb75b3d82a32ba3684e083b12b396b.png)

Table 45-399: SCB4\_IIR3\_CH0\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-393: SCB4\_IIR3\_CH1\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000392_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-400: SCB4\_IIR3\_CH1\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-394: SCB4\_IIR3\_CH1\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000393_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-401: SCB4\_IIR3\_CH1\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-395: SCB4\_IIR3\_CH1\_IB\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000394_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-402: SCB4\_IIR3\_CH1\_IB\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-396: SCB4\_IIR3\_CH1\_IB\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000395_e0f069b6fa3c696905b9e1b7be487908d018401522d6a35f75a998fcaab1c1be.png)

Table 45-403: SCB4\_IIR3\_CH1\_IB\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-397: SCB4\_PERIPH\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000396_6cd4b1233646db80e265e5d346cb6386207e27c8ae425133fdb2eeafcf7b0bbb.png)

Table 45-404: SCB4\_PERIPH\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_0 | .                         |

Figure 45-398: SCB4\_PERIPH\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000397_f617638916c1b4b2e09cf83c31f636ca98240a9abb3494aa2233fba71d62fbf4.png)

Table 45-405: SCB4\_PERIPH\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_1 | .                         |

Figure 45-399: SCB4\_PERIPH\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000398_974cbd7514c7872f7387da38ee73a420334d0f8588f2204444ae196c44b3951c.png)

Table 45-406: SCB4\_PERIPH\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_2 | .                         |

Figure 45-400: SCB4\_PERIPH\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000399_dc7bb22ac0006956ac6dab579f980a7c613421813ca345a25a6b5f6e27bc79b1.png)

Table 45-407: SCB4\_PERIPH\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 7:4 (R/NW)         | REV_AND      | .                         |
| 3:0 (R/NW)         | CUST_MOD_NUM | .                         |

Figure 45-401: SCB4\_PERIPH\_ID\_4 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000400_d420f7e793a3ec845b538d24f74d8cfb6a017e7cc45424ed27d25826e99f79bf.png)

Table 45-408: SCB4\_PERIPH\_ID\_4 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_4 | .                         |

Figure 45-402: SCB4\_PERIPH\_ID\_5 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000401_75d87f200d7e299012f2a2604da7c8910227061d6795a9bd5cb8266fa19d0281.png)

Table 45-409: SCB4\_PERIPH\_ID\_5 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_5 | .                         |

Figure 45-403: SCB4\_PERIPH\_ID\_6 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000402_47f6a6936d7bb8ed4a961cef0a4f023e9d4834c6a76b979eb5e0b5ed421154f5.png)

Table 45-410: SCB4\_PERIPH\_ID\_6 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_6 | .                         |

Figure 45-404: SCB4\_PERIPH\_ID\_7 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000403_79c4b97cbaef5b50dd3fd9add7272283184a52c9504e75ce3065631e564d5a2e.png)

Table 45-411: SCB4\_PERIPH\_ID\_7 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_7 | .                         |

Figure 45-405: SCB4\_REMAP Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000404_db0c142443bb6e199052a495bd5108c0e6a470d70b0b6ec351b96bb853251ba4.png)

Table 45-412: SCB4\_REMAP Register Fields

|   Bit No. (Access) | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
|                  0 | REMAP      | .                         |

Figure 45-406: SCB4\_SHARC\_DPORT\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000405_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-413: SCB4\_SHARC\_DPORT\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-407: SCB4\_SHARC\_DPORT\_READ\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000406_389f1a2d848e53263dbfbae48549cacdbdccebb2e20351876017c93fec8d1038.png)

Table 45-414: SCB4\_SHARC\_DPORT\_READ\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AR_QOS     | .                         |
| (R/W)              |            |                           |

Figure 45-408: SCB4\_SHARC\_DPORT\_TO\_SF\_L2\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000407_7a50faa4c63d3931cdda960f60e5c65d0838e26f4247517f989f75de1a55a344.png)

Table 45-415: SCB4\_SHARC\_DPORT\_TO\_SF\_L2\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-409: SCB4\_SHARC\_DPORT\_TO\_SF\_L2\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000408_9bd549365e9ddc3e0aa3fa783a5b18ded4a0ad7b729b960c365d73a332067319.png)

Table 45-416: SCB4\_SHARC\_DPORT\_TO\_SF\_L2\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-410: SCB4\_SHARC\_DPORT\_TO\_SF\_L2\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000409_f1d06c3722f7cc41a64fa1174b90cffa9f6efa263462614dc8b3c8eeb27a06d4.png)

Table 45-417: SCB4\_SHARC\_DPORT\_TO\_SF\_L2\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-411: SCB4\_SHARC\_DPORT\_TO\_SF\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000410_0f68f68dabd4191b358a6c5d884c0cde24e05ae63365176e276fff87c127c10c.png)

Table 45-418: SCB4\_SHARC\_DPORT\_TO\_SF\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-412: SCB4\_SHARC\_DPORT\_TO\_SF\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000411_59d5c33487e8b26fea901557c89344cc3ca18512c6e8ded5829dc20e2afdb6ee.png)

Table 45-419: SCB4\_SHARC\_DPORT\_TO\_SF\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-413: SCB4\_SHARC\_DPORT\_TO\_SF\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000412_529764fd4e319cd55377c21d679483ce7362b10889b224adee8d211e46be1d84.png)

Table 45-420: SCB4\_SHARC\_DPORT\_TO\_SF\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

Figure 45-414: SCB4\_SHARC\_DPORT\_WRITE\_QOS Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000413_0c2cf2d7374549835d7b36f245a960fbca8435e943040f79294f84a8bc4f8f27.png)

Table 45-421: SCB4\_SHARC\_DPORT\_WRITE\_QOS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 3:0                | AW_QOS     | .                         |

Figure 45-415: SCB4\_SHARC\_IPORT\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000414_7db9dfadac2bae4b8aa48a7fc4d87427135f1c33024b6daeea0c7e0833f678d5.png)

Table 45-422: SCB4\_SHARC\_IPORT\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-416: SCB4\_SHARC\_IPORT\_IB\_SYNC\_MODE Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000415_7715f1c4d6cf4f809a07b182969fdf551db8410b294b8131dec6ed2936cf1bf2.png)

Table 45-423: SCB4\_SHARC\_IPORT\_IB\_SYNC\_MODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 2:0                | SYNC_MODE  | .                         |

## ADSP-2184x INST1 Register Descriptions

Your module description, here. (INST1) contains the following registers.

Table 45-424: ADSP-2184x INST1 Register List

| Name                          | Description   |
|-------------------------------|---------------|
| INST1_COMP_ID_0               |               |
| INST1_COMP_ID_1               |               |
| INST1_COMP_ID_2               |               |
| INST1_COMP_ID_3               |               |
| INST1_OSPI_S_IB_FN_MOD        |               |
| INST1_OSPI_S_IB_FN_MOD2       |               |
| INST1_OSPI_S_IB_FN_MOD_ISS_BM |               |
| INST1_OSPI_S_IB_FN_MOD_LB     |               |
| INST1_PERIPH_ID_0             |               |
| INST1_PERIPH_ID_1             |               |
| INST1_PERIPH_ID_2             |               |
| INST1_PERIPH_ID_3             |               |
| INST1_PERIPH_ID_4             |               |
| INST1_PERIPH_ID_5             |               |
| INST1_PERIPH_ID_6             |               |
| INST1_PERIPH_ID_7             |               |

Table 45-424: ADSP-2184x INST1 Register List (Continued)

| Name                          | Description   |
|-------------------------------|---------------|
| INST1_REMAP                   |               |
| INST1_SPI2_S_IB_FN_MOD        |               |
| INST1_SPI2_S_IB_FN_MOD2       |               |
| INST1_SPI2_S_IB_FN_MOD_ISS_BM |               |
| INST1_SPIF_M_FN_MOD           |               |

Figure 45-417: INST1\_COMP\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000416_727d77b688abfcef3aca3b3adf15fb7eacbc3a93129741ec784511ab54f31b10.png)

Table 45-425: INST1\_COMP\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_0  | .                         |
| (R/NW)             |            |                           |

Figure 45-418: INST1\_COMP\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000417_0eb6c590a6392d304d7dbf816bf67393fbe0f7b6132a68660a21f800215dc29a.png)

Table 45-426: INST1\_COMP\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_1  | .                         |

Figure 45-419: INST1\_COMP\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000418_c38df1b5c9dd0d4d408936f10a3e2c04ba7fd16fc743b9245e37689c00387df4.png)

Table 45-427: INST1\_COMP\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_2  | .                         |
| (R/NW)             |            |                           |

Figure 45-420: INST1\_COMP\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000419_5cdefaec630838917a2dae9202aadf694f3e61fe6666257163ef4b3c2506880f.png)

Table 45-428: INST1\_COMP\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | COMP_ID_3  | .                         |
| (R/NW)             |            |                           |

Figure 45-421: INST1\_OSPI\_S\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000420_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-429: INST1\_OSPI\_S\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-422: INST1\_OSPI\_S\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000421_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-430: INST1\_OSPI\_S\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-423: INST1\_OSPI\_S\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000422_b63b9a57dea336dac36bcfe2344b0d28a6ccf077dab4d7ebea847c7d1b2adc4b.png)

Table 45-431: INST1\_OSPI\_S\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-424: INST1\_OSPI\_S\_IB\_FN\_MOD\_LB Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000423_9c2608838a8de77fa8e02436f804e783ebe62b7ff15cac8fa5b468ebcffb3022.png)

Table 45-432: INST1\_OSPI\_S\_IB\_FN\_MOD\_LB Register Fields

|   Bit No. (Access) | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
|                  0 | FN_MOD_LB  | .                         |

Figure 45-425: INST1\_PERIPH\_ID\_0 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000424_6cd4b1233646db80e265e5d346cb6386207e27c8ae425133fdb2eeafcf7b0bbb.png)

Table 45-433: INST1\_PERIPH\_ID\_0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_0 | .                         |

Figure 45-426: INST1\_PERIPH\_ID\_1 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000425_f37507244c912d617bc43dd980a285857136cb8f457c5fb39681a22bc39fa507.png)

Table 45-434: INST1\_PERIPH\_ID\_1 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_1 | .                         |

Figure 45-427: INST1\_PERIPH\_ID\_2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000426_974cbd7514c7872f7387da38ee73a420334d0f8588f2204444ae196c44b3951c.png)

Table 45-435: INST1\_PERIPH\_ID\_2 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_2 | .                         |

Figure 45-428: INST1\_PERIPH\_ID\_3 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000427_dc7bb22ac0006956ac6dab579f980a7c613421813ca345a25a6b5f6e27bc79b1.png)

Table 45-436: INST1\_PERIPH\_ID\_3 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 7:4 (R/NW)         | REV_AND      | .                         |
| 3:0 (R/NW)         | CUST_MOD_NUM | .                         |

Figure 45-429: INST1\_PERIPH\_ID\_4 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000428_d420f7e793a3ec845b538d24f74d8cfb6a017e7cc45424ed27d25826e99f79bf.png)

Table 45-437: INST1\_PERIPH\_ID\_4 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_4 | .                         |

Figure 45-430: INST1\_PERIPH\_ID\_5 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000429_75d87f200d7e299012f2a2604da7c8910227061d6795a9bd5cb8266fa19d0281.png)

Table 45-438: INST1\_PERIPH\_ID\_5 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_5 | .                         |

Figure 45-431: INST1\_PERIPH\_ID\_6 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000430_47f6a6936d7bb8ed4a961cef0a4f023e9d4834c6a76b979eb5e0b5ed421154f5.png)

Table 45-439: INST1\_PERIPH\_ID\_6 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_6 | .                         |

Figure 45-432: INST1\_PERIPH\_ID\_7 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000431_79c4b97cbaef5b50dd3fd9add7272283184a52c9504e75ce3065631e564d5a2e.png)

Table 45-440: INST1\_PERIPH\_ID\_7 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration   |
|--------------------|-------------|---------------------------|
| 7:0                | PERIPH_ID_7 | .                         |

Figure 45-433: INST1\_REMAP Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000432_c9fa813a5ad152909ad99dc9e00e83058607f67a8f0f1cccf7bb3c38ab2ee74d.png)

Table 45-441: INST1\_REMAP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 4:0                | REMAP      | .                         |

Figure 45-434: INST1\_SPI2\_S\_IB\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000433_49deabd64930254637e0fce56add23df1e9999e47e357bd8d4068b029f81fbf3.png)

Table 45-442: INST1\_SPI2\_S\_IB\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |

Figure 45-435: INST1\_SPI2\_S\_IB\_FN\_MOD2 Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000434_9d8759b18ac4ededeccc02fad0b6585be817d0230dd99c595834ff5a6433c890.png)

Table 45-443: INST1\_SPI2\_S\_IB\_FN\_MOD2 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration   |
|--------------------|--------------|---------------------------|
| 0 (R/W)            | BYPASS_MERGE | .                         |

Figure 45-436: INST1\_SPI2\_S\_IB\_FN\_MOD\_ISS\_BM Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000435_545f1b28cde223e2d8dd26e318e7a6333d0a401593a2068a9acb8fc129df490b.png)

Table 45-444: INST1\_SPI2\_S\_IB\_FN\_MOD\_ISS\_BM Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration   |
|--------------------|---------------|---------------------------|
| 1:0                | FN_MOD_ISS_BM | .                         |

Figure 45-437: INST1\_SPIF\_M\_FN\_MOD Register Diagram

![Image](48_System_Crossbars_(SCB)_artifacts/image_000436_ab92a2b18bf8fd42cde868dd9ae24d4ec4ea7c83d0b493f56f9e2a24709df194.png)

Table 45-445: INST1\_SPIF\_M\_FN\_MOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | FN_MOD     | .                         |
| (R/W)              |            |                           |