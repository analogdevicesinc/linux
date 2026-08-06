![Image](adsp-sc84x_-PrB_artifacts/image_000000_0294ee5eaa9cf8e6e27f6d7aa933ea1e68e2b721373c4c1ec47bb508f7ad3807.png)

## Preliminary Technical Data

## SYSTEM FEATURES

SHARC-FX high performance  floating-point core 256-bit vector size

Peak core performance at 1.2 GHz: 28.8 GFLOPS, 9.6 GMAC (32-bit float), 19.2 GMAC (16-bit fixed)

64/512 kB L1 instruction/data RAM with ECC protection

32/256 kB L1 instruction/data cache with ECC protection

Dual Arm Cortex-A55 cores

Up to 1200 MHz/3360 DMIPS with advanced SIMD and floating-point support per core

32 kB L1 instruction cache with parity/32 kB L1 data cache with ECC per core

256 kB L2 cache with ECC per core

Shared Snoop Control Unit (SCU) and 0.5 MB L3 cache with ECC

Up to 32 Mb (4 MB) on-chip L2 SRAM with ECC protection Level 3 (L3) 16/32-bit interface to LPDDR4 SDRAM devices

## High Performance SHARC-FX DSP Core

## With Arm-Based Connectivity/Security

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## On-board accelerators

2 FIR engines (up to 1.2 GHz, 4 taps per cycle each) 4 IIR engines (up to 1.2 GHz, 6 cycles per biquad each) Security

Hardware Security Module (HSM) supporting EVITA-FULL Cryptographic hardware accelerators and fast secure boot Key peripherals

Gigabit Ethernet, Dual xSPI/CAN-FD, eMMC, HADC, I 2 C, ASRC/SPORT

## PACKAGE

18 mm × 18 mm, 484-ball BGA\_ED (0.8 mm pitch), RoHS compliant, with LPDDR4 interface

## APPLICATIONS

Automotive: audio for head units and amplifiers, ANC/RNC, digital cockpit, ICC, AEC/Mic beamforming Consumer: speakers, sound bars, AVRs, conferencing systems, mixing consoles, microphone arrays

Figure 1. ADSP-SC846 (Full-Featured Model) Processor Block Diagram

![Image](adsp-sc84x_-PrB_artifacts/image_000001_e15420ac8876b97d643243a56ab29f602bad53783d0f496debfda5173e9e267a.png)

SHARC is a registered trademark of Analog Devices, Inc.

Information  furnished  by  Analog  Devices  is  believed  to  be  accurate  and  reliable. However,  no  responsibility  is  assumed  by  Analog  Devices  for  its  use,  nor  for  any infringements of patents or other rights of third parties that may result from its use. Specifications subject to change without notice. No license is granted by implication or otherwise under any patent or patent rights of Analog Devices. Trademarks and registered trademarks are the property of their respective owners.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## TABLE OF CONTENTS

| System Features . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                       | . . 1   |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| Package . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                           | . . 1   |
| Applications . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                  | . . 1   |
| Table of Contents . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                         | . 2     |
| Revision History . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                        | . . 2   |
| General Description . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                               | . . 3   |
| SHARC-FX Processor Core . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                                                   | . . 5   |
| Dual ARMCortex-A55 Processor (ADSP-SC84x Only) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                | . . 6   |
| System Infrastructure . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                                     | . . 7   |
| System Memory Map . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                                         | . . 7   |
| Security Features . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                               | 10      |
| Security Features Disclaimer . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                                                | 11      |
| REVISION HISTORY                                                                                                                                                                |         |
| Removed 324-ball package and associated models. Replaced Arm Cortex-M85 processor with Arm Cortex-A55 processor.                                                                |         |
| Changes to ADSP-SC846 (Full-Featured Model) Processor Block Diagram . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . | . . 1   |
| Changes to System Memory Map . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                                                      | . . 7   |
| Changes to ADSP-2184x/ADSP-SC84x Memory Map . . . . .                                                                                                                           | . . 9   |
| Added                                                                                                                                                                           |         |
| GPIO Multiplexing for 484-Ball BGA_ED Package . . . . . . . .                                                                                                                   | 21      |
| Added Outline Dimensions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .                                                                            | 25      |

Safety Features ....................................................  11

Processor Peripherals  ...........................................  12

System Acceleration .............................................  17

System Design  ....................................................  17

System Debug .....................................................  19

Development Tools ..............................................  19

Additional Information  ........................................  20

Related Signal Chains  ...........................................  20

GPIO Multiplexing for 484-Ball BGA\_ED Package ......... 21

Outline Dimensions ................................................  25

Surface-Mount Design ..........................................  25

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## GENERAL DESCRIPTION

The ADSP-2184x/ADSP-SC84x digital signal processors (DSPs) are members of the SHARC ® -FX family of products. The SHARC-FX core uses a single-instruction, multiple-data (SIMD) vector floating-point architecture and can issue up to four instructions per cycle in most combinations. The SHARCFX core inside the ADSP-2184x/ADSP-SC84x processors offers processing speeds of up to 1.2 GHz coupled with up to 4 MB of L2 memory for low latency applications. For applications seeking enhanced connectivity options such as Ethernet, the ADSPSC84x includes high performance dual Arm ® Cortex ® -A55 cores in addition to the SHARC-FX core. All members of the SHARC-FX family have on-board IIR and FIR accelerators, as well as an efficient auto-vectorizing compiler for C/C++ programming.

The SHARC-FX core supports scalar and vector operations on all data types in vectors up to 256 bits, including integer, fixedpoint, floating-point, complex 16-bit/32-bit fixed-point and complex 32-bit/64-bit floating-point.

Table 1. DSP Core Comparison: SHARC+ vs. SHARC-FX

| Feature                          | SHARC+                                                         | SHARC-FX                                                                           |
|----------------------------------|----------------------------------------------------------------|------------------------------------------------------------------------------------|
| Float32 Operations Per Cycle     | 2 multiply and 2 add                                           | 8 fused multiply-add and 8 add                                                     |
| 1K Complex FFT Benchmark         | 11,000 cycles                                                  | 2,500 cycles                                                                       |
| Instruction Format               | One to four operations, 16 to 48 bits                          | One to four operations, 16 to 128 bits                                             |
| L1 Memory                        | 640 kB (4-bank shared by data RAM, instruction RAM, and cache) | 512 kB data RAM, 256 kB data cache, 64 kB instruction RAM, 32 kB instruction cache |
| Float64 Operations Per Cycle     | 2 every 6 cycles                                               | 4 each cycle                                                                       |
| Single Sample Biquad Performance | 2 channels and 2 stages every 8 cycles                         | 8 channels and 2 stages every 8 cycles                                             |
| Logf/expf Scalar Benchmark       | 50 cycles                                                      | 4 cycles                                                                           |

Eight float32 multiply/accumulate operations are allowed per cycle, with no constraints on alignment. The SHARC-FX core also features large register sets (32 data registers), thus reducing the need for stack save and restore. The peripherals and system architecture of the ADSP-2184x/ADSP-SC84x processors are compatible with previous SHARC processors, allowing for easy application porting.

By integrating a set of industry-leading system peripherals and memory, this family of processors is the platform of choice for applications that require leading-edge signal processing in one integrated package. These applications span a wide array of markets, including automotive, professional audio, and industrial-based applications that require high floating-point performance.

Table 1 provides a comparison between the SHARC+ and SHARC-FX core.

Table 2 provides comparison information for features that vary across the standard processors.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Table 2. Processor Features

|                                                 | DSP Only        | DSP Only        | Enhanced Connectivity   | Enhanced Connectivity   |
|-------------------------------------------------|-----------------|-----------------|-------------------------|-------------------------|
| Processor Feature 1                             | ADSP-21844      | ADSP-21846      | ADSP-SC844              | ADSP-SC846              |
| SHARC-FX DSP Core (MHz, Maximum) 2              | 600, 800        | 1000, 1200      | 600, 800                | 1000, 1200              |
| L1 D-RAM/ I-RAM (kB)                            | 512/64          | 512/64          | 512/64                  | 512/64                  |
| L1 D-Cache/I-Cache (kB)                         | 256/32          | 256/32          | 256/32                  | 256/32                  |
| Arm Cortex-A55 Dual Core Cluster (MHz, Maximum) | N/A             | N/A             | 1200                    | 1200                    |
| L1 D-Cache/I-Cache (kB)                         | N/A             | N/A             | 32/32                   | 32/32                   |
| L2 Cache (kB)                                   | N/A             | N/A             | 256                     | 256                     |
| L3 Cache (kB)                                   | N/A             | N/A             | 512                     | 512                     |
| System Memory                                   |                 |                 |                         |                         |
| L2 SRAM (kB)                                    | 2048            | 4096            | 2048                    | 4096                    |
| LPDDR4 Controller (16/32-Bit)                   | 1               | 1               | 1                       | 1                       |
| Hardware Accelerators                           |                 |                 |                         |                         |
| HSM                                             | Yes             | Yes             | Yes                     | Yes                     |
| FIR/IIR                                         | 2/4             | 2/4             | 2/4                     | 2/4                     |
| Security Crypto Engine                          | Yes             | Yes             | Yes                     | Yes                     |
| DAI (Includes SRU and DRU) 3                    | 2               | 2               | 2                       | 2                       |
| Full SPORTs                                     | 8/4             | 8/4             | 8/4                     | 8/4                     |
| S/PDIF Receive/Transmit                         | 2/1             | 2/1             | 2/1                     | 2/1                     |
| ASRCs                                           | 16/8            | 16/8            | 16/8                    | 16/8                    |
| PCGs                                            | 8/4             | 8/4             | 8/4                     | 8/4                     |
| 4-Channel PDMMICInput                           | 2/1             | 2/1             | 2/1                     | 2/1                     |
| Buffers                                         | 40/20           | 40/20           | 40/20                   | 40/20                   |
| Multiplexed Peripherals                         |                 |                 |                         |                         |
| Media Local Bus 3-Pin 4                         | 1               | 1               | 1                       | 1                       |
| Link Port (4-bit)                               | 2               | 2               | 2                       | 2                       |
| General-Purpose Counter                         | 1               | 1               | 1                       | 1                       |
| Watchdog Timer                                  | 4               | 4               | 4                       | 4                       |
| I 2 C (TWI)                                     | 6               | 6               | 6                       | 6                       |
| General-Purpose Timer                           | 16              | 16              | 16                      | 16                      |
| xSPI with Octal and HyperBus Support            | 2               | 2               | 2                       | 2                       |
| Quad-Data Bit SPI                               | 2               | 2               | 2                       | 2                       |
| Dual-Data Bit SPI                               | 2               | 2               | 2                       | 2                       |
| UART                                            | 3               | 3               | 3                       | 3                       |
| 10/100/1000 EMAC Std/AVB + Timer IEEE 1588      | N/A             | N/A             | 1                       | 1                       |
| CAN FD 4                                        | 2               | 2               | 2                       | 2                       |
| ePWM                                            | 8 outputs       | 8 outputs       | 8 outputs               | 8 outputs               |
| HADC(12-Bit)                                    | 8-channel       | 8-channel       | 8-channel               | 8-channel               |
| GPIO Ports                                      | Port A toH      | Port A toH      | Port A toH              | Port A toH              |
| GPIO + DAI Pins                                 | 122 + 40        | 122 + 40        | 122 + 40                | 122 + 40                |
| Package Options                                 | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED         | 484-ball BGA_ED         |

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## SHARC-FX PROCESSOR CORE

SHARC-FX is a DSP core (Figure 2) developed jointly by Analog Devices and Cadence, leveraging Cadence's Xtensa ® core technology. It is a follow-on to the SHARC and SHARC+ DSP cores from Analog Devices. The SHARC-FX core is not ISA compatible with the SHARC and SHARC+ DSP cores.

The following sections describe the key features of the SHARCFX core inside the ADSP-2184x/ADSP-SC84x processors.

## 4-Way VLIW

The 4-way VLIW issues one to four operations per cycle in an instruction that is 16 to 128 bits wide. In every cycle, a load or store or scalar ALU op, a second load or second ALU op, a vector ALU op, or a vector ALU or multiply/accumulate op can be executed.

## 256-Bit SIMD

This wide SIMD unit can be broken up into lanes that are 8, 16, 32, or 64 bits wide. For example, the unit can execute eight 32bit multiply-accumulates per cycle. Each lane of a vector is enabled or disabled by a boolean register for conditional operations. An entire vector is permuted for small table lookups. Support is also added for fast histograms.

## DSP Features

The SHARC-FX core supports loop counters, circular addressing, and fixed-point arithmetic with 20-, 40-, and 80-bit accumulators, binary-point shifts and rounding, and saturation.

## Fixed- and Floating-Point Data Types

The SHARC-FX core handles all major data types:

- 8-, 16-, 32-, and 64-bit integers
- 8-, 16-, and 32-bit fixed-point, both real and complex
- 32- and 64-bit floating-point, both real and complex.

## L1 Data Cache and RAM

The SHARC-FX core includes a 256 kB data cache, a 512 kB data RAM, a 32 kB instruction cache, and a 64 kB instruction RAM. The caches are write-back or write-through, use 64B lines and are four-way associative with LRU replacement. All memories are protected with ECC. Figure 4 shows the ADSP2184x/ADSP-SC84x memory map.

## Memory Protection Unit (MPU)

Although memory is physically addressed, an MPU of the ADSP-2184x/ADSP-SC84x controls the access and the caching behavior for up to 32 variable-sized address ranges.

![Image](adsp-sc84x_-PrB_artifacts/image_000002_6a3f4c23964cf1894f6762bb2fce30beb5c970745a9239986d6833feaed3464d.png)

SYSTEM FABRIC

Figure 2. SHARC-FX Processor Block Diagram

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## Advanced Arithmetic

The SHARC-FX core supports high accuracy, floating-point arithmetic via a fused multiply-add and also has seeds for accurate square root and reciprocal. The processor core executes efficient but imprecise checking for floating-point errors and can also break on fixed-point saturation. The log2f and exp2f operations are executed in one cycle, and these operations are used for single-cycle reciprocal, square-root, divide, and power at less accuracy. The compiler recognizes vector versions of standard functions like sinf and sqrtf operations and can automatically vectorize loops containing them.

## Built-In Interrupt Controller

A 221-input vectored interrupt controller is available to the ADSP-2184x/ADSP-SC84x processor. Each input can be active high or low, edge or level, and can have one of 15 priority levels.

## Built-In Direct Memory Access Engine (iDMA)

A fast, low-latency DMA controller for moving blocks to and from the Data RAM is available.

## CoreSight Debugging

The ADSP-2184x/ADSP-SC84x uses the Arm ® CoreSight TM SoC-600 debug standard, with breakpoints, watchpoints, and PC tracing. It can also count a wide range of events such as cache misses and pipeline stalls for performance analysis. When used with a third-party emulator, the emulator must support CoreSight SoC-600.

## DUAL ARM CORTEX-A55 PROCESSOR (ADSP-SC84x ONLY)

ADSP-SC84x processors include high performance dual Cortex-A55 processors. The Arm Cortex-A55 processor (Figure 3) is a mid-range, low-power core that implements the Armv8-A architecture with support for the Armv8.1-A extension, the Armv8.2-A extension, and the reliability, availability, and serviceability (RAS) extension. The core is implemented inside the DynamIQ shared unit (DSU) as a little core.

Each Arm Cortex-A55 core includes the following features:

- Core Features
- Full implementation of the Arm8.2-A A64, A32, and T32 instruction sets
- Both AArch32 and AArch64 execution states at all exception levels (EL0 to EL3)
- In-order pipeline with direct and indirect branch prediction
- Separate L1 data and instruction side memory systems with a memory management unit (MMU)
- Support for Arm TrustZone ® technology
- Extension-data engine unit that implements the advanced SIMD and floating-point architecture support
- Extension-cryptographic extension

## Preliminary Technical Data

- Generic interrupt controller (GIC) CPU interface to connect to an external distributor
- Generic timers interface that supports a 64-bit count input from an external system counter
- Cache Features
- L1 instruction cache unit (32 KB), L1 data cache unit (32 KB), and unified private L2 cache unit (256 KB)
- L1 and L2 cache protection in the form of error correction code (ECC) or parity on all RAM instances
- Debug Features
- Reliability, availability, and serviceability (RAS) extension
- Armv8.2-A debug logic
- Performance monitoring unit (PMU)
- Embedded trace macrocell (ETM) that supports instruction trace only
- Shared Features between dual Cortex-A55 processors
- L3 cache (512 KB)
- Snoop Control Unit (SCU)

Figure 3. Arm Cortex-A55 Processor Block Diagram

![Image](adsp-sc84x_-PrB_artifacts/image_000003_74dbd0c950b0d3983f6b1b0b8e7040ed5aa2eb078b7ae91b81b2c9fa20cc4a82.png)

## Generic Interrupt Controller (GIC), GIC-600

The generic interrupt controller (GIC) is a centralized resource for supporting and managing interrupts. The GIC consists of three interfaces-the distributor interface (GICD), the redistributor interface (GICR), and the central processing unit (CPU) interface. The distributor and the redistributor interfaces configure interrupts. The CPU interface handles interrupts.

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## GIC Distributor Interface (GICD)

The distributor performs interrupt prioritization and distribution of shared peripheral interrupts (SPIs) and software generated interrupts (SGIs) to the redistributors and CPU interfaces that are connected to the processors in the system. The distributor provides the routing configuration for SPIs and holds all the associated routing and priority information for private peripheral interrupts (SPIs).

## GIC Redistributor Interface (GICR)

The redistributor provides the configuration settings for SGIs and PPIs. The redistributor holds the control, prioritization, and pending information for all SGIs and PPIs. It also presents the pending interrupt with the highest priority to the CPU interface.

## GIC CPU Interface

The GIC CPU interface block performs priority masking and preemption handling for a connected processor in the system. The GIC supports 16 SGIs, 9 PPIs, and 354 SPIs.

## GIC Performance Monitoring Unit (GIC PMU)

The GIC contains a performance monitoring unit (PMU) for counting key GIC events from the distributor. Redistributor events are not tracked by the PMU. The delivery of PPI and SGI interrupts are counted by recording calls to the core interrupt service routine. The PMU has five counters with snapshot capability and overflow interrupt.

## Cryptographic Extension

The Cortex-A55 core cryptographic extension supports the Armv8-A cryptographic extension. The cryptographic extension adds new A64, A32, and T32 instructions to advanced SIMD that accelerate:

- Advanced encryption standard (AES) encryption and decryption
- Secure hash algorithm (SHA) functions SHA-1, SHA-224, and SHA-256
- Finite field arithmetic used in algorithms such as Galois/Counter mode and elliptic curve cryptography

## SYSTEM MEMORY MAP

## Table 3. SHARC-FX L1 DRAM and IRAM Space

| Memory                    | SHARC-FX Private Addressing Space   | ArmCortex-A55 Addressing Space   | System Addressing Space   |
|---------------------------|-------------------------------------|----------------------------------|---------------------------|
| L1 Data RAM(512 KB)       | 0x2F780000-0x2F7FFFFF               | 0x28240000-0x282BFFFF            | 0x28240000-0x282BFFFF     |
| L1 Instruction RAM(64 KB) | 0x2F800000-0x2F80FFFF               | 0x282C0000-0x282CFFFF            | 0x282C0000-0x282CFFFF     |

## SYSTEM INFRASTRUCTURE

The following sections describe the system infrastructure of the ADSP-2184x/ADSP-SC84x processors.

## System L2 Memory

A system L2 SRAM memory of up to 32 Mb (4 MB) is available to the SHARC-FX core, both Arm Cortex-A55 cores, and the system DMA channels (see Table 4). The L2 SRAM block is subdivided into up to 16 banks to support concurrent access to the L2 memory ports. Memory accesses to the L2 memory space are multicycle accesses by the SHARC-FX core.

The memory space is used for various situations including:

- Accelerator and peripheral sources and destination memory to avoid accessing data in the external memory
- A location for DMA descriptors
- Storage for additional data for the SHARC-FX core to avoid external memory latencies and reduce external memory bandwidth
- Storage for data cached by the SHARC-FX core
- Inter-core communication and passing of data between the SHARC -FX and Arm Cortex-A55 cores of the processor

See the System Memory Protection Unit (SMPU) section for options in limiting access by specific cores and DMA requesters.

## One Time Programmable Memory (OTP)

The processors feature 4 kB of one time programmable (OTP) memory that is memory-map accessible. This memory can be programmed with custom keys and supports secure boot and secure operation.

## I/O Memory Space

Mapped I/Os include SPI1/SPI2/xSPI0/xSPI1 memory address space (see Table 5).

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Table 4. SHARC-FX and Arm Cortex-A55 L2 Memory Addressing Map

| Memory         | SHARC-FX Addressing   | ArmCortex-A55 Addressing   | System Addressing     |
|----------------|-----------------------|----------------------------|-----------------------|
| L2 BootROM0    | 0x20200000-0x2020FFFF | 0x20200000-0x2020FFFF      | 0x20200000-0x2020FFFF |
| L2 BootROM1    | 0x20210000-0x20217FFF | 0x20210000-0x20217FFF      | 0x20210000-0x20217FFF |
|                | 0x20218000-0x2021FFFF | 0x00010000-0x00017FFF      | 0x20218000-0x2021FFFF |
| L2 BootROM2    | 0x20220000-0x2022FFFF | 0x00000000-0x0000FFFF      | 0x20220000-0x2022FFFF |
| L2 RAM(2 MB) 1 | 0x20400000-0x205FFFFF | 0x20400000-0x205FFFFF      | 0x20400000-0x205FFFFF |
| L2 RAM(2 MB) 1 | 0x20600000-0x207FFFFF | 0x20600000-0x207FFFFF      | 0x20600000-0x207FFFFF |

Table 5. Memory Map of Mapped I/Os

| Memory                          | SHARC-FX Addressing   | ArmCortex-A55 Addressing   | System Addressing     |
|---------------------------------|-----------------------|----------------------------|-----------------------|
| xSPI1 Memory (256 MB)           | 0x50000000-0x5FFFFFFF | 0x50000000-0x5FFFFFFF      | 0x50000000-0x5FFFFFFF |
| SPI1/SPI2/xSPI0 Memory (512 MB) | 0x60000000-0x7FFFFFFF | 0x60000000-0x7FFFFFFF      | 0x60000000-0x7FFFFFFF |

## Table 6. DMC Memory Map

|            | SHARC-FX Addressing   | ArmCortex-A55 Addressing   | System Addressing     |
|------------|-----------------------|----------------------------|-----------------------|
| DMC0(1 GB) | 0x80000000-0xBFFFFFFF | 0x80000000-0xBFFFFFFF      | 0x80000000-0xBFFFFFFF |

## System Crossbars (SCBs)

The system crossbars (SCBs) are the fundamental building blocks of a switch fabric style for on-chip system bus interconnection. The SCBs connect system bus requesters to system bus completers, providing concurrent data transfer between multiple bus requesters and multiple bus completers. A hierarchical model-built from multiple SCBs-provides a power and area efficient system interconnection.

The SCBs provide the following features:

- Highly efficient, pipelined bus transfer protocol for sustained throughput
- Full-duplex bus operation for flexibility and reduced latency
- Concurrent bus transfer support to allow multiple bus requesters to access bus completers simultaneously
- Protection model (privileged/secure) support for selective bus interconnect protection

## Direct Memory Access (DMA)

The processor uses direct memory access (DMA) to transfer data within memory spaces or between a memory space and a peripheral. The processor can specify data transfer operations and return to normal processing while the fully integrated DMA controller carries out the data transfers independent of processor activity.

DMA transfers can occur between memory and a peripheral or between one memory and another memory. Each memory to memory DMA stream uses two channels: the source channel and the destination channel.

All DMA channels can transport data to and from all on-chip and off-chip memories. Programs can use two types of DMA transfers: descriptor-based or register-based. Register-based DMA allows the processors to program DMA control registers directly to initiate a DMA transfer. On completion, the DMA control registers automatically update with original setup values for continuous transfer. Descriptor-based DMA transfers require a set of parameters stored within memory to initiate a DMA sequence. Descriptor-based DMA transfers allow multiple DMA sequences to be chained together by programming a DMA channel to set up and start another DMA transfer automatically after the current sequence completes.

The DMA engine supports the following DMA operations:

- A single linear buffer that stops on completion
- A linear buffer with negative, positive, or zero stride length
- A circular autorefreshing buffer that interrupts when each buffer becomes full
- A similar circular buffer that interrupts on fractional buffers, such as at the halfway point
- A 1D DMA using a set of identical ping pong buffers defined by a linked ring of two-word descriptor sets, each containing a link pointer and an address
- A 1D DMA using a linked list of four-word descriptor sets containing a link pointer, an address, a length, and a configuration
- A 2D DMA using an array of one-word descriptor sets, specifying only the base DMA address
- A 2D DMA using a linked list of multiword descriptor sets, specifying all configurable parameters

## Preliminary Technical Data

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## Memory Direct Memory Access (MDMA)

![Image](adsp-sc84x_-PrB_artifacts/image_000004_00831ddfa0104a4d1594a23cd55fc9333280b8c2a65877d97c8b36eab43d5dde.png)

ADDRESS SPACE

Figure 4. ADSP-2184x/ADSP-SC84x Memory Map

The processor supports a total of eight MDMA channels for various memory direct memory access (MDMA) operations, including,

- Enhanced bandwidth MDMA channels with cyclic redundancy check (CRC) protection (32-bit bus width, run on SYSCLK)
- Enhanced bandwidth MDMA channel (32-bit bus width, runs on SYSCLK)
- Maximum bandwidth MDMA channel (64-bit bus width, runs on SYSCLK)

## Extended Memory DMA

Extended memory DMA supports various operating modes, such as delay line (which allows processor reads and writes to external delay line buffers and to the external memory), with limited core interaction and scatter/gather DMA (writes to and from noncontiguous memory blocks).

## Cyclic Redundancy Check (CRC) Protection

The cyclic redundancy check (CRC) protection modules allow system software to calculate the signature of code, data, or both in memory, the content of memory-mapped registers, or periodic communication message objects. Dedicated hardware circuitry compares the signature with precalculated values and triggers appropriate fault events.

For example, the system software initiates the signature calculation of the entire memory contents every 100 ms and compares this with expected, precalculated values. If a mismatch occurs, a fault condition is generated through the processor core or the trigger routing unit.

The CRC is a hardware module based on a CRC32 engine that computes the CRC value of the 32-bit data-words presented to it. The source channel of the memory to memory DMA (in memory scan mode) provides data. The data can be optionally forwarded to the destination channel (memory transfer mode). The main features of the CRC peripheral are as follows:

- Memory scan mode
- Memory transfer mode
- Data verify mode
- Data fill mode
- User-programmable CRC32 polynomial
- Bit and byte mirroring option (endianness)
- Fault and error interrupt mechanisms
- 1D and 2D fill block to initialize an array with constants
- 32-bit CRC signature of a block of memory or an MMR block

SHARC-FX

ARM CORTEX A-55

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## Event Handling

The processor provides event handling that supports both nesting and prioritization. Nesting allows multiple event service routines to be active simultaneously. Prioritization ensures that servicing a higher priority event takes precedence over servicing a lower priority event.

The processor provides support for four different types of events:

- An emulation event causes the processors to enter emulation mode, allowing command and control of the processors through the JTAG interface.
- A reset event resets the processors.
- An exception event is caused by errors during execution. Synchronous exceptions, such as illegal instruction or data misalignment, do not permit the instruction to complete. Imprecise exceptions, such as floating-point or saturation errors or some error correcting code (ECC) errors, are reported after the instruction has completed. Exceptions cause the program counter to change to the value held in the EPC special register and are not affected by the interrupt priority level or status.
- An interrupt event occurs asynchronously to program flow. The interrupts are caused by input signals, timers, and other peripherals, as well as by an explicit software instruction.

## System Event Controller (SEC)

The SHARC-FX core event controller receives some interrupts from the system event controller (SEC), but most go directly to its built-in interrupt controller. The SEC features include the following:

- Comprehensive system event source management, including interrupt enable, fault enable, priority, core mapping, and source grouping
- A distributed programming model where each system event source control and all status fields are independent of each other
- Determinism where all system events have the same propagation delay and provide unique identification of a specific system event source
- A completer control port that provides access to all SEC registers for configuration, status, and interrupt and fault services
- Global locking that supports a register level protection model to prevent writes to locked registers
- Fault management including fault action configuration, time out, external indication, and system reset

## Preliminary Technical Data

## Trigger Routing Unit (TRU)

The trigger routing unit (TRU) provides system level sequence control without core intervention. The TRU maps trigger generators to trigger receivers. Trigger receivers can be configured to respond to triggers in various ways. Common applications enabled by the TRU include,

- Automatically triggering the start of a DMA sequence after a sequence from another DMA channel completes
- Software triggering
- Synchronization of concurrent activities

## SECURITY FEATURES

The following sections describe the security features of the ADSP-2184x/ADSP-SC84x processors.

## Arm TrustZone

The ADSP-SC84x processors provide TrustZone technology that is integrated into the Arm Cortex-A55 processors. The TrustZone technology enables a secure state that is extended throughout the system fabric.

## Cryptographic Hardware Accelerators

The ADSP-2184x/ADSP-SC84x processors support standardsbased hardware accelerated encryption, decryption, authentication, and true random number generation.

Support for the hardware accelerated cryptographic ciphers includes the following:

- AES in ECB, CBC, ICM, and CTR modes with 128-bit, 192-bit, and 256-bit keys
- DES in ECB and CBC mode with 56-bit key
- 3DES in ECB and CBC mode with 3x 56-bit key
- ARC4 in stateful, stateless mode, up to 128-bit key

Support for the hardware accelerated hash functions includes the following:

- SHA-1
- SHA-2 with 224-bit and 256-bit digests
- HMAC transforms for SHA-1 and SHA-2
- MD5

Public key accelerator (PKA) is available to offload computation intensive public key cryptography operations.

Both a hardware-based nondeterministic random number generator and pseudorandom number generator are available.

Secure boot is also available with 224-bit and 256-bit elliptic curve digital signatures ensuring integrity and authenticity of the boot stream. Optionally, ensuring confidentiality through AES-128 encryption is available.

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Password protected secure debug is also available to allow only trusted users to access the system with debug tools.

## CAUTION

![Image](adsp-sc84x_-PrB_artifacts/image_000005_ed39aef0263741c45fc15f2fb57af76b967d463c43911bf058f5c88c336cfd69.png)

This product includes security features that can be used  to  protect  embedded  nonvolatile  memory contents  and  prevent  execution  of  unauthorized code.  When  security  is  enabled  on  this  device (either  by  the  ordering  party  or  the  subsequent receiving parties), the ability of Analog Devices to conduct  failure  analysis  on  returned  devices  is limited. Contact Analog Devices for details on the failure analysis limitations for this device.

## Hardware Security Module (HSM)

The hardware security module is a standalone security core with root of trust functionality. The HSM combines a secure 32-bit RISC-V CPU, dedicated secure memories, and local nonvolatile memory with cryptographic hardware engines, such as true random number generator (TRNG), secure hash and HMAC engines, a symmetric cipher accelerator, a DPA-resistant asymmetric cipher accelerator, DPA-resistant key derivation, glitch detection, and a hardware firewall.

## System Protection Unit (SPU)

The system protection unit (SPU) guards against accidental or unwanted access to an MMR space of the peripheral by providing a write protection mechanism. The user can choose and configure the protected peripherals, as well as configure which of the system MMR requesters the peripherals are guarded against.

The SPU is also part of the security infrastructure. Along with providing write protection functionality, the SPU is employed to define which resources in the system are secure or nonsecure as well as block access to secure resources from nonsecure requesters.

## System Memory Protection Unit (SMPU)

The system memory protection unit (SMPU) provides memory protection against read and/or write transactions to defined regions of memory. There are SMPU units in the ADSP2184x/ADSP-SC84x processors for each memory space, except for SHARC-FX L1.

The SMPU is also part of the security infrastructure. It allows the user to protect against arbitrary read and/or write transactions and allows regions of memory to be defined as secure and prevent nonsecure requesters from accessing those memory regions.

## SECURITY FEATURES DISCLAIMER

Analog Devices does not guarantee that the Security Features described herein provide absolute security. ACCORDINGLY, ANALOG DEVICES HEREBY DISCLAIMS ANY AND ALL EXPRESS AND IMPLIED WARRANTIES THAT THE SECURITY FEATURES CANNOT BE BREACHED, COMPROMISED, OR OTHERWISE CIRCUMVENTED AND IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR

ANY LOSS, DAMAGE, DESTRUCTION, OR RELEASE OF DATA, INFORMATION, PHYSICAL PROPERTY, OR INTELLECTUAL PROPERTY.

## SAFETY FEATURES

The ADSP-2184x/ADSP-SC84x processors are designed to provide robust and fail-safe operation. The following safety primitives are provided by the processors.

## Error Correcting Code (ECC) Protected L1 Memories

The SRAMs and caches in the SHARC-FX L1 memory space and the DTCM memories and caches in the Arm Cortex-A55 cores are protected by SECDEC. Single-bit errors are automatically corrected and written back. Multiple-bit errors are retried several times, and then cause exceptions. The cache tags and BTB on the SHARC-FX are protected the same way.

## Error Correcting Code (ECC) Protected L2 Memories

Error correcting code (ECC) corrects single event upsets. A single error correct/double error detect (SEC/DED) code protects the L2 memory. By default, ECC is enabled, but it can be disabled on a per bank basis. Single-bit errors correct transparently. If enabled, dual-bit errors can issue a system event or fault. ECC protection is fully transparent to the user, even if L2 memory is read or written by 8-bit or 16-bit entities.

## Parity and ECC Protected Peripheral Memories

Parity protection is added to the following peripheral memories:

- ASRC
- IIR
- FIR
- CRYPTO
- EMAC
- MLB
- TRACE

CAN FD memory is ECC protected.

## Cyclic Redundancy Check (CRC) Protected Memories

Whereas parity bit and ECC protection mainly protect against random soft errors in L1 and L2 memory cells, the CRC engines can protect against systematic errors (pointer errors) and static content (instruction code) of L1, L2, and even Level 3 (L3) memories (DDR3L). The processors feature two CRC engines that are embedded in the memory to memory DMA controllers.

CRC checksums can be calculated or compared automatically during memory transfers. Alternatively, single or multiple memory regions can be continuously scrubbed by a single DMA work unit as per DMA descriptor chain instructions. The CRC engine also protects data loaded during the boot process.

## Signal Watchdogs

The 16 general-purpose (GP) timers feature modes to monitor off-chip signals. The watchdog period mode monitors whether external signals toggle with a period within an expected range.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## Preliminary Technical Data

The watchdog width mode monitors whether the pulse widths of external signals are within an expected range. Both modes help detect undesired toggling or lack of toggling of system level signals.

## System Event Controller (SEC)

Besides system events, the system event controller (SEC) further supports fault management, including fault action configuration as timeout, internal indication by system interrupt, or external indication through the SYS\_FAULT pin and system reset.

## Memory Error Controller (MEC)

The memory error controller (MEC) manages memory parity/ECC errors and warnings from the cores and peripherals and sends out interrupts and triggers.

## PROCESSOR PERIPHERALS

The following sections describe the peripherals of the ADSP2184x/ADSP-SC84x processors.

## Dynamic Memory Controller (DMC)

The 16-bit/32-bit, two-channel, dual-rank dynamic memory controller (DMC) interfaces to LPDDR4 (JESD209-4D), supporting up to 8 Gb along with inline ECC and PHY training support. See Table 6 for the DMC memory map.

The DMC can be used in 32-bit mode by combining both the channels or in 16-bit mode through the first channel using halfbus width support. (Independent/parallel usage of both the channels in 16-bit mode is not supported.)

There is an additional prefetch buffer (PFB) added to improve the performance for read accesses. When enabled on DDR, the prefetch buffer supports the optional predictive prefetch that improves performance for nonlinear access patterns. For noncontinuous (nonlinear) accesses (resulting in MISS), the prefetch buffer feature makes decisions on whether fetch and/or prefetch requests from the PFB to memory launch or give direct access for such read requests. The decision is based on the history of the read transactions received.

## Digital Audio Interface (DAI)

The processors support two identical digital audio interface (DAI) units. The DAI can connect various peripherals to any of the DAI pins.

The application code makes these connections using the signal routing unit (SRU), shown in Figure 1.

The SRU is a matrix routing unit (or group of multiplexers) that enables the peripherals provided by each DAI instance to interconnect under software control. This functionality allows easy use of the DAI associated peripherals for a wider variety of applications by using a larger set of algorithms than is possible with nonconfigurable signal paths.

The DAI includes the peripherals described in the following sections (SPORTs, ASRC, S/PDIF, PCG, and PDM). DAI Pin Buffer 20 and DAI Pin Buffer 19 can change the polarity of the input signals.

The DAI\_PINx pin buffers can also be used as GPIO pins. DAI input signals allow the triggering of interrupts on the rising edge, falling edge, or both.

See the Digital Audio Interface (DAI) chapter of the ADSP2184x/ADSP-SC84x SHARC-FX Processor Hardware Reference (TBD) for complete information on the use of the DAIs and SRUs.

## DAI Routing Unit (DRU)

The DAI routing unit (DRU) provides flexibility when routing signals across the two DAI units. All DAI0 SRU source signals are available as source signals for the DAI1 SRU, and all DAI1 SRU source signals are available as source signals for the DAI0 SRU.

## Serial Port (SPORT)

The processors feature eight synchronous serial ports (SPORTs), providing an inexpensive interface to a wide variety of digital and mixed-signal peripheral devices. These devices include Analog Devices AD19xx and ADAU19xx families of audio codecs, analog-to-digital converters (ADCs) and digitalto-analog converters (DACs). Two data lines, a clock, and a frame sync comprise a SPORT half. The data lines can be programmed to either transmit or receive data, and each data line has a dedicated DMA channel.

An individual SPORT module consists of two independently configurable SPORT halves with identical functionality. Two bidirectional data lines-primary (0) and secondary (1)-are available per SPORT half and are configurable as either transmitters or receivers. Therefore, each SPORT half permits two unidirectional streams into or out of the same SPORT. This bidirectional functionality provides greater flexibility for serial communications. For full-duplex configuration, one half SPORT provides two transmit data signals, and the other half SPORT provides two receive data signals. The frame sync and clock are shared.

Serial ports operate in the following six modes:

- Standard DSP serial mode
- Multichannel time division multiplexing (TDM) mode
- I 2 S mode
- Packed I 2 S mode
- Left justified mode
- Right justified mode

## Asynchronous Sample Rate Converter (ASRC)

The asynchronous sample rate converter (ASRC) contains 16 ASRC blocks. The ASRC provides up to 140 dB signal-tonoise ratio (SNR). The ASRC block performs synchronous or asynchronous sample rate conversion across independent stereo channels, without using internal processor resources. The ASRC blocks can also be configured to operate together to convert multichannel audio data without phase mismatches. Finally, the ASRC can clean up audio data from jittery clock sources such as the S/PDIF receiver.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## S/PDIF-Compatible Digital Audio Receiver/Transmitter

The Sony/Philips Digital Interface Format (S/PDIF) is a standard audio data transfer format that allows the transfer of digital audio signals from one device to another. There are two S/PDIF transmit/receive blocks on the processor. The digital audio interface carries three types of information: audio data, nonaudio data (compressed data), and timing information.

The S/PDIF interface supports one stereo channel or compressed audio streams. The S/PDIF transmitter and receiver are AES3 compliant and support the sample rate from 24 kHz to 192 kHz. The S/PDIF receiver supports professional jitter standards.

The S/PDIF receiver/transmitter has no separate DMA channels. It receives audio data in serial format and converts it into a biphase encoded signal. The serial data input to the receiver/ transmitter can be formatted as left justified, I 2 S, or right justified with word widths of 16, 18, 20, or 24 bits. The serial data, clock, and frame sync inputs to the S/PDIF receiver/transmitter are routed through the SRU. They can come from various sources, such as the SPORTs, external pins, and the precision clock generators (PCGs), and are controlled by the SRU control registers.

## Precision Clock Generators (PCG)

The precision clock generators (PCG) consist of eight units located in the two DAI blocks. The PCG can generate a pair of signals (clock and frame sync) derived from a clock input signal (CLKIN, SCLK0, or DAI pin buffer). Both units are identical in functionality and operate independently of each other. The two signals generated by each unit are normally used as a serial bit clock/frame sync pair.

## Pulse Density Modulation (PDM) Microphone Interface

The pulse density modulation (PDM) interface is used to convert digital PDM microphone data to I 2 S/TDM format. The microphone data in I 2 S/TDM format is then routed internally to the serial port/ASRC or externally via the DAI pins. The PDM microphone inputs include an internal decimation filter. Up to eight PDM microphones can be connected to the two dedicated digital microphone interfaces (one per DAI). Each PDM interface consists of one clock line and two data lines. Two microphones can share a single data line and be used along with a clock line to create a dual-input microphone port. Two dualinput lines can share a single clock line to support four microphone inputs.

## Universal Asynchronous Receiver/Transmitter (UART) Ports

The processors provide four full-duplex universal asynchronous receiver/transmitter (UART) ports, fully compatible with PC standard UARTs. Each UART port provides a simplified UART interface to other peripherals or hosts, supporting full-duplex, DMA supported, asynchronous transfers of serial data. A UART port includes support for five to eight data bits as well as no parity, even parity, or odd parity.

Optionally, an additional address bit can be transferred to interrupt only addressed nodes in multidrop bus (MDB) systems. A frame is terminated by a configurable number of stop bits.

The UART ports support automatic hardware flow control through the clear to send (CTS) input and request to send (RTS) output with programmable assertion first in, first out (FIFO) levels.

To help support the Local Interconnect Network (LIN) protocols, a special command causes the transmitter to queue a break command of programmable bit length into the transmit buffer. Similarly, the number of stop bits can be extended by a programmable interframe space.

## Serial Peripheral Interface (SPI) Ports

The processors have four industry-standard SPI-compatible ports that allow the processors to communicate with multiple SPI-compatible devices.

The baseline SPI peripheral is a synchronous, 4-wire interface consisting of two data pins, one device select pin, and a gated clock pin. The two data pins allow full-duplex operation to other SPI-compatible devices. An extra two (optional) data pins are provided to support quad-SPI operation on two instances. Enhanced modes of operation, such as flow control, fast mode, and dual-I/O mode (DIOM), are also supported. DMA mode allows for transferring several words with minimal CPU interaction.

With a range of configurable options, the SPI ports provide a glueless hardware interface with other SPI-compatible devices in controller mode, target mode, and multicontroller environments. The SPI peripheral includes programmable baud rates, clock phase, and clock polarity. The peripheral can operate in a multicontroller environment by interfacing with several other devices, acting as either a controller device or a target device. In a multicontroller environment, the SPI peripheral uses opendrain outputs to avoid data bus contention. The flow control features enable slow target devices to interface with fast controller devices by providing an SPI ready pin (SPI\_RDY), which flexibly controls the transfers.

The baud rate and clock phase and polarities of the SPI port are programmable. The port has integrated DMA channels for both transmit and receive data streams.

## xSPI with Octal and HyperBus Support

The octal serial peripheral interface port (xSPI/HyperBus) provides an increased external memory data bus width (up to eight bits in parallel). The xSPI port supports dual data rate (DDR) modes of operation, which enable the transfer of up to 16 bits of data each clock cycle. The xSPI port provides overall data throughput and performance improvement, including faster boot time.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Features of the xSPI/HyperBus port include:

- Support for single-, dual-, quad-, or octal-I/O transfers
- Can be interfaced with octal flash, octal RAM, HyperFlash, and HyperRAM devices
- Can be interfaced with legacy flash devices including quad and SPINAND
- Auto command engine and minicontroller
- Built-in DMA support for high speed transfers
- Multi-threading support
- Support for execute in place (XIP): continuous mode
- Programmable page and block sizes
- Programmable write protected regions
- Programmable memory timing
- Support for DDR commands
- Support of PHY mode of operation for high-speed transfers
- Support of DQS for increased robustness of data sampling at higher speeds

## Link Port (LP)

Two 4-bit wide link ports (LPs) can connect to the link ports of other DSPs or peripherals. Link ports are bidirectional and have two or four data lines, an acknowledge line, and a clock line. The data lines can operate in dual data rate (DDR) mode.

Link ports can operate in reduced pin mode, thereby reducing the number of pins required to interface between two processors. For example, two processors can be connected using the link port in 2-bit DDR mode.

## Enhance Mobile Storage Interface (eMSI)

The enhanced mobile storage interface (eMSI) controller acts as the host interface for embedded multimedia cards (eMMC)/ secure digital memory (SD) cards. The eMSI controller has the following features:

- Support for a single eMMC device
- Support for 1-bit, 4-bit, and 8-bit eMMC modes
- Support for 1.8 V I/O eMMC protocols, including eMMC5.1
- 21-signal external interface with clock, command, data strobe, and up to 8 data lines and additional side-band signals
- Integrated DMA controller
- Integrated PHY for reliable operation across frequencies
- Support for HS200 and H400 speed modes for eMMC and SDR104 for SD cards
- Large 8 KB FIFO for seamless transfers to high speed
- Card interface clock generation in the clock distribution unit (CDU)

## Preliminary Technical Data

## Ethernet Media Access Controller (EMAC)

The processor features an ethernet media access controller (EMAC): 10/100/1000 AVB Ethernet with precision time protocol (IEEE 1588).

The processors can directly connect to a network through embedded fast EMAC that supports 10Base-T (10 Mb/sec), 100Base-T (100 Mb/sec) and 1000Base-T (1 Gb/sec) operations.

Some standard features of the EMAC are as follows:

- Support of MII/RMII/RGMII protocols for external PHYs
- Full-duplex and half-duplex modes
- Media access management (in half-duplex operation)
- Flow control
- Station management, including the generation of MDC/MDIO frames for read/write access to PHY registers

Some advanced features of the EMAC include the following:

- Automatic checksum computation of IP header and IP payload fields of receive frames
- Independent 32-bit descriptor driven receive and transmit DMA channels
- Frame status delivery to memory through DMA, including frame completion semaphores for efficient buffer queue management in software
- Transmit DMA support for separate descriptors for MAC header and payload fields to eliminate buffer copy operations
- Convenient frame alignment modes
- 47 MAC management statistics counters with selectable clear on read behavior and programmable interrupts on half maximum value
- Advanced power management
- Magic packet detection and wakeup frame filtering
- Support for 802.3Q tagged VLAN frames
- Programmable MDC clock rate and preamble suppression

## Audio Video Bridging (AVB) Support

The 10/100/1000 EMAC supports the following audio video bridging (AVB) features:

- Separate channels or queues for AV data transfer in 100 Mbps and 1000 Mbps modes
- IEEE 802.1-Qav specified credit-based shaper (CBS) algorithm for the additional transmit channels
- Configuring up to seven additional channels on the transmit and receive paths for AV traffic. Channel 0 is available by default and carries the legacy best effort Ethernet traffic on the transmit side.
- Separate DMA, transmit and receive FIFO for AVB latency class
- Programmable control to route received VLAN tagged nonAV packets to channels or queues

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## Precision Time Protocol (PTP) IEEE 1588 Support

The IEEE 1588 standard is a precision clock synchronization protocol for networked measurement and control systems. The processors include hardware support for IEEE 1588 with an integrated precision time protocol synchronization engine (PTP\_TSYNC).

This engine provides hardware assisted time stamping to improve the accuracy of clock synchronization between PTP nodes. The main features of the engine include the following:

- Support for both IEEE 1588-2002 and IEEE 1588-2008 protocol standards
- Hardware assisted time stamping capable of up to 12.5 ns resolution
- Lock adjustment
- Automatic detection of IPv4 and IPv6 packets, as well as PTP messages
- Multiple input clock sources (SCLK0, RGMII, RMII, MII clock, and external clock)
- Programmable pulse per second (PPS) output
- Auxiliary snapshot to time stamp external events

## Controller Area Network with Flexible Data-Rate (CAN FD)

There are two controller area network (CAN) modules. A CAN controller implements the CAN with flexible data-rate (CAN FD) and the CAN 2.0B protocol supporting both standard and extended message frames and long payloads up to 64 bytes, transferred at rates of up to 8 Mbps. This protocol is an asynchronous communications protocol used in both industrial and automotive control systems. The CAN protocol is well suited for control applications due to the capability to communicate reliably over a network. This is because the protocol incorporates CRC checking, message error tracking, and fault node confinement.

The CAN FD controller offers the following features:

- Flexible mailboxes configurable to store 0 to 8, 16, 32, or 64 bytes
- Dedicated receiver masks for each mailbox
- Flexible message buffers up to 64 buffers of 8 bytes length each, configurable as receive or transmit
- Programmable transmission priority scheme
- Transceiver delay compensation when transmitting CAN FD messages at faster data rates
- Memory read accesses error detection and correction

An additional crystal is not required to supply the CAN clock because it is derived from a system clock through a programmable divider.

## Pulse Width Modulator (PWM) Units

The pulse width modulator (PWM) module is a flexible and programmable waveform generator. With minimal CPU intervention, the PWM generates complex waveforms for motor control, pulse coded modulation (PCM), DAC conversions, power switching, and power conversion. The PWM module has three PWM pairs with the following features:

- 16-bit center-based PWM generation unit
- Programmable PWM pulse width
- Single update mode with an option for asymmetric duty
- Programmable dead time and switching frequency
- Programmable dead time per channel
- Twos-complement implementation which permits smooth transition to full on and full off states
- Dedicated asynchronous PWM shutdown signal (PWM0\_TRIP)
- Support for synchronization using SYNC signal (PWM0\_SYNC)

## Timers

The processors include several timers that are described in the following sections.

## General-Purpose (GP) Timers (TIMER)

There is one general-purpose (GP) timer unit, providing 16 GP programmable timers. Each timer has an external pin that can be configured as PWM or timer output, as an input to clock the timer, or as a mechanism for measuring pulse widths and periods of external events. These timers can be synchronized to an external clock input on the TM\_TMR[n] pins, an external TM\_CLK input pin, or to the internal SCLK0.

These timer units can be used in conjunction with the UARTs to measure the width of the pulses in the data stream to provide a software autobaud detect function for the respective serial channels.

The GP timers can generate interrupts to the processor core, providing periodic events for synchronization to either the system clock or to external signals. Timer events can also trigger other peripherals via the TRU (for instance, to signal a fault). Each timer can also be started and stopped by any trigger generator without core intervention.

## Watchdog Timer (WDT)

Four on-chip software watchdog timers (WDT) are used by the SHARC-FX core. A software watchdog can improve system availability by forcing the processors to a known state, via a general-purpose interrupt, or a fault, if the timer expires before being reset by software.

The programmer initializes the count value of the timer, enables the appropriate interrupt, then enables the timer. Thereafter, the software must reload the counter before it counts down to zero from the programmed value, protecting the system from remaining in an unknown state where software that normally resets the timer stops running due to an external noise condition or software error.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## General-Purpose Counters (CNT)

A 32-bit general-purpose counter (CNT) is provided that can operate in general-purpose up/down count modes and can sense 2-bit quadrature or binary codes as typically emitted by industrial drives or manual thumbwheels. Count direction is controlled by a level-sensitive input pin or by two edge detectors.

A third counter input can provide flexible zero marker support and can input the push button signal of thumbwheel devices. All three CNT0 pins have a programmable debouncing circuit.

Internal signals forwarded to a GP timer enable the timer to measure the intervals between count events. Boundary registers enable auto-zero operation or simple system warning by interrupts when programmed count values are exceeded.

## Housekeeping Analog-to-Digital Converter (HADC)

The housekeeping analog-to-digital converter (HADC) provides a general-purpose, multichannel, successive approximation ADC. The following baseline HADC features apply to all models:

- 12-bit ADC core with built in sample and hold
- Throughput rates up to 1 MSPS
- Single external reference with analog inputs between 0 V and 1.8 V
- Selectable ADC clock frequency including the ability to program a prescaler
- Adaptable conversion type; allows single or continuous conversion with option of autoscan
- Four single-ended input channels
- Autosequencing capability with up to four autoconversions in a single session. Each conversion can be programmed to select one to four input channels.
- Four data registers (individually addressable) to store conversion values

For the ADSP-2184x/ADSP-SC84x processors, 16 data registers (individually addressable) extend the baseline features listed above by storing conversion values.

## Media Local Bus (MediaLB)

The automotive model has a Microchip MediaLB (MLB) device interface that allows the processors to function as a media local bus device. It includes support for 3-pin media local bus protocols. The MLB 3-pin configuration supports speeds up to 1024 × FS. The MLB also supports up to 64 logical channels with up to 468 bytes of data per MLB frame.

The MLB interface supports MOST25, MOST50, and MOST150 data rates and operates in device mode only.

## 2-Wire Controller Interface (TWI)

The processors include six 2-wire interface (TWI) modules that provide a simple exchange method of control data between multiple devices. The TWI module is compatible with the widely used I 2 C bus standard. The TWI module offers the capabilities

## Preliminary Technical Data

of simultaneous controller and target operation and support for both 7-bit addressing and multimedia data arbitration. The TWI interface utilizes two pins for transferring clock (TWI\_SCL) and data (TWI\_SDA) and supports the protocol at speeds up to 400 kbps. The TWI interface pins are compatible with 1.8 V logic levels.

Additionally, the TWI module is fully compatible with serial camera control bus (SCCB) functionality for easier control of various CMOS camera sensor devices.

## General-Purpose I/O (GPIO)

Each general-purpose port pin can be individually controlled by manipulating the port control, status, and interrupt registers:

- The GPIO direction control register specifies the direction of each individual GPIO pin as input or output.
- GPIO control and status registers have a write-one-tomodify mechanism that allows any combination of individual GPIO pins to be modified in a single instruction, without affecting the level of any other GPIO pins.
- GPIO interrupt mask registers allow each individual GPIO pin to function as an interrupt to the processors. GPIO pins defined as inputs can be configured to generate hardware interrupts, whereas output pins can be triggered by software interrupts.
- GPIO interrupt sensitivity registers specify whether individual pins are level or edge sensitive and specify, if edge sensitive, whether the rising edge or both the rising and falling edges of the signal are significant.

## Pin Interrupts

Every port pin on the processors can request interrupts in either an edge sensitive or a level sensitive manner with programmable polarity. Interrupt functionality is decoupled from GPIO operation. Three system level interrupt channels (PINT0-PINT2) are reserved for this purpose. Each of these interrupt channels can manage up to 32 interrupt pins. The assignment from pin to interrupt is not performed on a pin by pin basis. Rather, groups of eight pins (half ports) are flexibly assigned to interrupt channels.

Every pin interrupt channel features a special set of 32-bit memory-mapped registers that enable half port assignment and interrupt management. This functionality includes masking, identification, and clearing of requests. These registers also enable access to the respective pin states and use of the interrupt latches, regardless of whether the interrupt is masked. Most control registers feature multiple MMR address entries to write one to set or write one to clear them individually.

## Fractional PLL (Frac-N PLL)

The processors generate precise and low-jitter audio clock frequency using the fractional PLL module. The audio clock can be synchronized with an input reference clock.

Frac-N PLLs can be used to generate clock with frequency values which are fractional multiples of CLKIN frequency. For example, an audio controller clock of frequency 24.576 MHz can be generated with CLKIN frequency of 25 MHz.

## Preliminary Technical Data

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Frac-N PLL consists of a digital filter/controller, also called DPLL, which can be combined with Frac-N PLL to create a dual-loop PLL. In this mode, DPLL provides fractional and integer parts of divider values to the Frac-N PLL block. Output clock of Frac-N PLL is fed back to DPLL for phase-alignment with the external reference clock. For example, 1 KHz PPS output from the EMAC block can be used as reference to generate 24.576 MHz audio clock phase-aligned to the PPS output.

The reference clock of the DPLL can be fed via any one of the sources-any DAI pin, GPIO pin PA\_15, or ETH\_PTPPPS[03]. The output of the Frac-N PLL can be routed externally to the chip via a DAI pin or routed internally to a DAI peripheral (for example, SPORT, ASRC, and so on).

One of the areas of application of Frac-N PLL can be networked applications requiring time synchronization, thereby removing the need for external PLL.

## SYSTEM ACCELERATION

The following sections describe the system acceleration blocks of the ADSP-2184x/ADSP-SC84x processors.

## Finite Impulse Response (FIR) Accelerator

The finite impulse response (FIR) accelerator consists of a 1024 word coefficient memory, a 1024 word deep delay line for the data, and four multiplier-accumulator (MAC) units. A controller manages the accelerator. The FIR accelerator runs at the SHARC-FX core clock frequency. The FIR accelerator can access all memory spaces and can run concurrently with the other accelerators on the processor.

Note that there are two FIR accelerators.

## Infinite Impulse Response (IIR) Accelerator

The infinite impulse response (IIR) accelerator consists of a 1440 word coefficient memory for storage of biquad coefficients, a data memory for storing the intermediate data, and one MAC unit. A controller manages the accelerator. The IIR accelerator runs at the SHARC-FX core clock frequency. The IIR accelerator can access all memory spaces and run concurrently with the other accelerators on the processor.

Note that there are four IIR accelerators. Two of the IIR accelerators support coefficient slewing mechanism in the hardware.

## SYSTEM DESIGN

The following sections provide an introduction to system design features and power supply issues.

## Clock Management

The processors provide three operating modes, each with a different performance and power profile. Control of clocking to each of the processor peripherals reduces power consumption.

## Reset Control Unit (RCU)

Reset is the initial state of the whole processor, or the core, and is the result of a hardware or software triggered event. In this state, all control registers are set to default values and functional units are idle. Exiting a full system reset begins with the core ready to boot.

The reset control unit (RCU) controls how all the functional units enter and exit reset. Differences in functional requirements and clocking constraints define how reset signals are generated. Programs must guarantee that none of the reset functions put the system into an undefined state or cause resources to stall. This requirement is particularly important when the core resets (programs must ensure that there is no pending system activity involving the core when it is reset).

From a system perspective, reset is defined by both the reset target and the reset source.

The reset target is defined as the following:

- System reset-all functional units except the RCU are set to default states.
- Hardware reset-all functional units are set to default states without exception. History is lost.
- Core only reset-affects the core only. When in reset state, the core is not accessible by any bus requester.

The reset source is defined as the following:

- System reset-can be triggered by software (writing to the RCU\_CTL register) or by another functional unit, such as the dynamic power management (DPM) unit or any of the SEC, TRU, or emulator inputs.
- Hardware reset-the SYS\_HWRST input signal asserts active (pulled down).
- Trigger request (peripheral).

## Clock Generation Unit (CGU)

The ADSP-2184x/ADSP-SC84x processors support two independent PLLs. Each PLL is part of a clock generation unit (CGU).

Frequencies generated by each CGU are derived from a common multiplier with different divider values available for each output.

The CGU generates all on-chip clocks and synchronization signals. Multiplication factors are programmed to define the PLLCLK frequency.

Programmable values divide the PLLCLK frequency to generate the core clock (CCLK), the system clocks, the LPDDR4 clock (DCLK), and the output clock (OCLK). For more information on clocking, see the ADSP-2184x/ADSP-SC84x SHARC-FX Processor Hardware Reference (TBD).

Writing to the CGU control registers does not affect the behavior of the PLL immediately. Registers are first programmed with a new value and the PLL logic executes the changes to ensure smooth transitions from the current conditions to the new conditions.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## System Crystal Oscillator

The processor can be clocked by an external crystal (see Figure 5), a sine wave input, or a buffered, shaped clock derived from an external clock oscillator. If using an external clock, it must be compatible with the VIHCLKIN and VILCLKIN specifications and must not be halted, changed, or operated below the specified frequency during normal operation (see the Preliminary Operating Conditions (TBD) section). This signal is connected to the SYS\_CLKIN0 pin of the processor. When using an external clock, the SYS\_XTAL0 pin must be left unconnected. Alternatively, because the processor includes an on-chip oscillator circuit, an external crystal can be used.

![Image](adsp-sc84x_-PrB_artifacts/image_000006_b81c477d31517bad71366f88a0cb0c90564f76036a24aacea9feff2aafaf1444.png)

NOTE: VALUES MARKED WITH * MUST BE CUSTOMIZED, DEPENDING ON THE CRYSTAL AND LAYOUT. ANALYZE CAREFULLY. VALID FREQUENCY RANGE IS 20 MHz TO 30 MHz FOR SYS\_CLKIN0.

Figure 5. External Crystal Connection

For fundamental frequency operation, use the circuit shown in Figure 5. A parallel resonant, fundamental frequency, microprocessor grade crystal is connected across the SYS\_CLKIN0 pin and the SYS\_XTAL0 pin.

The two capacitors and the series resistor, shown in Figure 5, fine tune phase and amplitude of the sine frequency. The capacitor and resistor values shown in Figure 5 are typical values only. The capacitor values are dependent upon the load capacitance recommendations of the crystal manufacturer and the physical layout of the printed circuit board (PCB). The resistor value depends on the drive level specified by the crystal manufacturer. The user must verify the customized values based on careful investigations on multiple devices over the required temperature range.

## Clock Distribution Unit (CDU)

The two clock generation units each provide outputs that feed a clock distribution unit (CDU). The clock outputs CLKO0-CLKO12 are connected to various targets. For more information, refer to the ADSP-2184x/ADSP-SC84x SHARCFX Processor Hardware Reference (TBD).

## Preliminary Technical Data

## Clock Out/External Clock

The SYS\_CLKOUT output pin has programmable options to output divided down versions of the on-chip clocks. By default, the SYS\_CLKOUT pin drives a buffered version of the SYS\_ CLKIN0 input. Refer to the ADSP-2184x/ADSP-SC84x SHARC-FX Processor Hardware Reference (TBD) to change the default mapping of clocks.

## Booting

The processors have several mechanisms for automatically loading internal and external memory after a reset. The boot mode is defined by the SYS\_BMODE[n] input pins. There are two categories of boot modes. In flash boot modes, the processors actively load data from serial memories. In external host boot modes, the processors receive data over a serial interface from an external host device.

The boot modes are shown in Table 7. These modes are implemented by the SYS\_BMODE[n] bits of the reset configuration register and are sampled during power-on resets and software initiated resets.

Table 7. Boot Modes

| SYS_BMODE[3:0] Setting   | BootMode                      |
|--------------------------|-------------------------------|
| 0000                     | No boot                       |
| 0001                     | SPI flash (SPI1)              |
| 0010                     | SPI host (SPI2)               |
| 0011                     | UART host (UART0)             |
| 0100                     | Extended link port host (LP0) |
| 0101                     | xSPI flash (xSPI0)            |
| 0110                     | eMMCdevice                    |
| 0111                     | xSPI flash (xSPI1)            |
| 1000                     | Link port host (LP0)          |
| 1001-1111                | Reserved                      |

## Thermal Monitoring Unit (TMU)

The thermal monitoring unit (TMU) provides on-chip temperature measurement for applications that require substantial power consumption. The TMU is integrated into the processor die and digital infrastructure using an MMR-based system access to measure the die temperature variations in real-time.

TMU features include the following:

- On-chip temperature sensing
- Programmable over temperature and under temperature limits
- Programmable conversion rate
- Averaging feature available

## Power Supplies

The processors have separate power supply connections for

- Internal (VDD\_INT)
- External (VDD\_EXT)

## Preliminary Technical Data

- HADC/TMU (VDD\_ANA)
- DMC (VDD\_DMC)
- PLL (VDD\_PLL)
- Fractional PLL (FPLLANA\_VDDHV)

All power supplies must meet the specifications provided in the Preliminary Operating Conditions (TBD) section. All external supply pins must be connected to the same power supply.

## Power Management

As shown in Table 8, the processors support six different power domains, which maximizes flexibility while maintaining compliance with industry standards and conventions.

The power dissipated by a processor is largely a function of the clock frequency and the square of the operating voltage. For example, reducing the clock frequency by 25% results in a 25% reduction in dynamic power dissipation.

## Table 8. Power Domains

| Power Domain                           | V DD Range   |
|----------------------------------------|--------------|
| All Internal Logic                     | V DD_INT     |
| LPDDR4                                 | V DD_DMC     |
| HADC/TMU                               | V DD_ANA     |
| SYSCLKIN0                              | V DD_EXT     |
| PLL0/1                                 | V DD_PLL     |
| All Other I/O (Includes SYS, JTAG, and | V DD_EXT     |
| Ports Pins)                            |              |

## Power-On Reset Requirements

SYS\_XTAL0 oscillations (SYS\_CLKIN0) start when power is applied to the VDD\_EXT pins. The rising edge of SYS\_HWRST initiates the PLL locking sequence. The rising edge of SYS\_HWRST must occur after all voltage supplies and SYS\_CLKIN0 oscillations are valid. For further details and information, see the Power-Up Reset Timing (TBD) section.

## Target Board JTAG Emulator Connector

The Analog Devices DSP tools product line of JTAG emulators uses the IEEE 1149.1 JTAG test access port of the processors to monitor and control the target board processor during emulation. The Analog Devices DSP tools product line of JTAG emulators provides emulation at full processor speed, allowing inspection and modification of memory, registers, and processor stacks. The processor JTAG interface ensures the emulator does not affect target system loading or timing.

For information on JTAG emulator operation, see the appropriate emulator hardware user's guide at SHARC Processors Software and Tools.

## SYSTEM DEBUG

The processors include various features that allow easy system debug. These are described in the following sections.

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## System Watchpoint Unit (SWU)

The system watchpoint unit (SWU) is a single module that connects to a single system bus and provides transaction monitoring. One SWU is attached to the bus going to each system completer. The SWU provides ports for all system bus address channel signals. Each SWU contains four match groups of registers with associated hardware. These four SWU match groups operate independently but share common event (for example, interrupt and trigger) outputs.

## Debug Access Port (DAP)

The debug access port (DAP) provides IEEE 1149.1 JTAG interface support through the JTAG debug. The DAP provides an optional instrumentation trace for both the core and system. It provides a trace stream that conforms to MIPI System Trace Protocol version 2 (STPv2) .

## DEVELOPMENT TOOLS

Analog Devices supports its processors with a complete line of software and hardware development tools, including an integrated development environment, evaluation products, emulators, and a variety of software add-ins.

## Integrated Development Environments (IDEs)

For C/C++ software writing and editing, code generation, and debug support, Analog Devices offers the CrossCore ® Embedded Studio (CCES) integrated development environment (IDE). CCES contains support/help information for porting existing SHARC+ applications to the SHARC-FX platform.

CCES is based on the Eclipse framework. Supporting most Analog Devices processor families, CCES is the IDE of choice for processors, including multicore devices. CCES is available for Windows and Linux platforms.

CCES supports available middleware such as real-time operating systems, algorithmic software modules, and evaluation hardware board support packages (BSP). For more information, visit www.analog.com/cces.

## EZ-KIT Evaluation System

For processor evaluation, Analog Devices provides EZ-KIT ® evaluation systems, which are comprised of a System on Module (SOM) board and a SOM carrier board.

The SOM boards (EV-SC846-SOM or EV-21846-SOM) are small and low-cost, featuring the audio processor, SDRAM and QSPI flash memories, FTDI USB-to-UART, and USB power. SOM boards also include a JTAG debug connection such that they can be used standalone for debug/development using either the ADZS-ICE-2000, ADZS-ICE-1500, or ADZS-ICE-1000 in-circuit emulator (ICE).

The SOM carrier board (EV-SOMCRR2-EZKIT) comes with a power supply and features high-speed connectors for the SOM, a comprehensive set of peripherals, and an on-board emulator. In addition, the EV-SOMCRR2-EZKIT carrier board features two SOM connectors, allowing support for dual-processor evaluation. The USB controller on the carrier board connects to the USB port of the user's PC, enabling CCES to emulate the

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

## Preliminary Technical Data

on-board processor in-circuit. This permits users to download, execute, and debug programs, as well as in-circuit program the on-board flash memory device to store user-specific boot code, thus enabling standalone operation.

Each EZ-KIT purchased includes an evaluation license for CCES. The CCES evaluation license type restricts CCES features to specific evaluation systems. With the full CCES license type (sold separately), engineers can develop software for any of the CCES-supported evaluation boards (including the SOM when used standalone or when connected to a different carrier board) or any custom system designed around supported Analog Devices processors. The full CCES license type also enables higher-performance debug capabilities via JTAG using an ICE.

For further information, see:

- www.analog.com/cces
- www.analog.com/EV-SC846-SOM (TBD)
- www.analog.com/EV-21846-SOM (TBD)
- www.analog.com/EV-SOMCRR2-EZKIT (TBD)

## Software Add-Ins for CCES

Analog Devices offers software add-ins which seamlessly integrate with CCES to extend the capabilities and reduce development time. Add-ins include BSPs for evaluation hardware, various middleware packages, FreeRTOS configuration, and algorithmic modules. Documentation, help, configuration dialogs, and coding examples present in these add-ins are viewable through the CCES IDE upon add-in installation.

## Board Support Packages (BSPs) for Evaluation Hardware

Software support for the EZ-KIT evaluation systems is provided by software add-ins called board support packages (BSPs). The BSPs contain the required drivers, pertinent release notes, and select example code for the given evaluation hardware. A download link for a specific BSP is located on the web page for the associated SOM product.

## Middleware Packages

Analog Devices offers FreeRTOS real-time operating system for its SHARC-FX and Arm Cortex-A cores. A port of Yocto Linux is provided for the Arm Cortex-A cores. For more information, see the Operating Systems and Middleware page.

## Algorithmic Modules

To speed development, Analog Devices offers add-ins that perform popular audio and video processing algorithms. These are available for use with CCES. For more information, visit the Software page in the Resource Library.

## Graphical Programming Using SigmaStudio+

®

SigmaStudio + is a next generation graphical programming and tuning tool for audio signal processing for the ADSP2184x/ADSP-SC84x processors. The tool supports in excess of 250 optimized audio algorithms such as filters, dynamic processors, and mixers. Individual algorithms can be dragged onto a canvas and then interconnected to form an audio signal chain which can then be downloaded onto the processor with the click of a button. All modules within the audio signal chain can be tuned in run time. The tool has a modern user interface that provides a rich user experience and supports several advanced features such as system design and scripting support.

The tool also supports custom module creation by which custom IPs/algorithms may be seamlessly integrated into the SigmaStudio+ environment, thereby allowing them to be used as modules within the audio signal chain. For more information, visit SigmaStudio+.

## Designing an Emulator-Compatible DSP Board (Target)

For embedded system test and debug, Analog Devices provides a family of emulators. On each JTAG DSP, Analog Devices supplies an IEEE 1149.1 JTAG test access port (TAP). In-circuit emulation is facilitated by use of this JTAG interface. The emulator accesses the internal features of the processor via the TAP, allowing the developer to load code, set breakpoints, and view variables, memory, and registers.

The processor must be halted to send data and commands, but after an operation is completed by the emulator, the DSP system is set to run at full speed with no impact on system timing. The emulators require the target board to include a header that supports connection of the JTAG port of the DSP to the emulator.

For details on target board design issues including mechanical layout, single processor connections, signal buffering, signal termination, and emulator pod logic, see Analog Devices JTAG Emulation Technical Reference (EE-68).

## ADDITIONAL INFORMATION

This data sheet provides a general overview of the ADSP2184x/ADSP-SC84x architecture and functionality. For detailed information on the core architecture and instruction set, refer to the SHARC-FX core documentation.

## RELATED SIGNAL CHAINS

A signal chain is a series of signal conditioning electronic components that receive input (data acquired from sampling either real-time phenomena or from stored data) in tandem, with the output of one portion of the chain supplying input to the next. Signal chains are often used in signal processing applications to gather and process data or to apply system controls based on analysis of real-time phenomena.

Analog Devices eases signal processing system development by providing signal processing components that are designed to work together. See Reference Designs.

The application signal chains page at Circuits from the Lab ® provides the following:

- Graphical circuit block diagram presentation of signal chains for a variety of circuit types and applications
- Drill down links for components in each chain to selection guides and application information
- Reference designs applying best practice design techniques

## GPIO MULTIPLEXING FOR 484-BALL BGA\_ED PACKAGE

Table 9 through Table 16 identify the pin functions that are multiplexed on the GPIO pins of the 484-ball BGA\_ED package for the ADSP-2184x/ADSP-SC84x processors.

Table 9. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port A 1

| SignalName   | Multiplexed Function 0   | Multiplexed Function 1              | Multiplexed Function 2   | Multiplexed Function 3   | Multiplexed Function Input Tap   |
|--------------|--------------------------|-------------------------------------|--------------------------|--------------------------|----------------------------------|
| PA_00        | UART0_TX                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FPLL2_DBG_LOCK           | SMC0_D00                 |                                  |
| PA_01        | UART0_RX                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FPLL3_DBG_LOCK           | SMC0_D01                 | TM0_ACI0                         |
| PA_02        | UART0_RTS                | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FLG0                     | SMC0_D02                 | ETH0_PTPAUXIN1                   |
| PA_03        | UART0_CTS                | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FLG1                     | SMC0_D03                 | ETH0_PTPAUXIN2                   |
| PA_04        | UART1_RX                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FPLL0_DBG_LOCK           | SMC0_D04                 | TM0_ACI1                         |
| PA_05        | UART1_TX                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FPLL1_DBG_LOCK           | SMC0_D05                 |                                  |
| PA_06        | UART1_RTS                | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FLG2                     | SMC0_D06                 | TM0_ACI10                        |
| PA_07        | UART1_CTS                | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | FLG3                     | SMC0_D07                 | TM0_ACI11                        |
| PA_08        | TWI0_SCL                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | PWM0_AH                  | SMC0_D08                 |                                  |
| PA_09        | TWI0_SDA                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | PWM0_AL                  | SMC0_D09                 |                                  |
| PA_10        | TWI1_SCL                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | PWM0_BH                  | SMC0_D10                 |                                  |
| PA_11        | TWI1_SDA                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | PWM0_BL                  | SMC0_D11                 |                                  |
| PA_12        | TWI2_SCL                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | PWM0_CH                  | SMC0_D12                 |                                  |
| PA_13        | TWI2_SDA                 | SPI5_RDY eMSI0_HOST_REG_VOLT_STABLE | PWM0_CL                  | SMC0_D13                 |                                  |
| PA_14        | TWI3_SCL                 | TM0_TMR9                            | HADC0_EOC_OUT            | SMC0_D14                 |                                  |
| PA_15        | TWI3_SDA                 | TM0_TMR10                           | SPI5_SEL4                | SMC0_D15                 |                                  |

Table 10. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port B 1

| SignalName   | Multiplexed Function 0   | Multiplexed Function 1   | Multiplexed Function 2   | Multiplexed Function 3   | Multiplexed Function Input Tap   |
|--------------|--------------------------|--------------------------|--------------------------|--------------------------|----------------------------------|
| PB_00        | TWI4_SCL                 | TM0_TMR14                | SPI0_SEL3                | SMC0_AMS0                |                                  |
| PB_01        | TWI4_SDA                 | TM0_TMR15                | SPI0_SEL4                | SMC0_ARE                 |                                  |
| PB_02        | TWI5_SCL                 |                          | SPI5_SEL2                | SMC0_AWE                 |                                  |
| PB_03        | TWI5_SDA                 |                          | SPI5_SEL3                | SMC0_ADDR1               |                                  |
| PB_04        | MLB0_DAT                 |                          | TM0_TMR11                | SMC0_ADDR2               |                                  |
| PB_05        | MLB0_SIG                 |                          | TM0_TMR12                | SMC0_ADDR3               |                                  |
| PB_06        | MLB0_CLK                 |                          | TM0_TMR13                | SMC0_ADDR4               | TM0_CLK                          |
| PB_07        | TM0_TMR0                 |                          | SPI2_SEL4                | SMC0_ADDR5               | FPLL2_REF_CLK                    |
| PB_08        | TM0_TMR1                 |                          | SPI1_SEL4                | SMC0_ADDR6               | FPLL3_REF_CLK                    |
| PB_09        | TM0_TMR2                 |                          | eMSI0_WP                 | SMC0_ADDR7               | EHT0_PTPAUXIN3                   |
| PB_10        | TM0_TMR3                 |                          | SPI1_D2                  |                          |                                  |
| PB_11        | TM0_TMR4                 |                          | SPI1_D3                  |                          |                                  |
| PB_12        | TM0_TMR5                 | SPI5_CLK                 | PWM0_DH                  | TRACE0_D04               | CNT0_UD                          |
| PB_13        | TM0_TMR6                 | SPI5_CITO                | PWM0_DL                  | TRACE0_D05               | CNT0_ZM                          |
| PB_14        | TM0_TMR7                 | SPI5_COTI                | PWM0_SYNC                | TRACE0_D06               | CNT0_DG                          |
| PB_15        | TM0_TMR8                 | SPI5_SEL1                | PWM0_TRIP                | TRACE0_D07               | SPI5_TS                          |

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Table 11. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port C 1

| SignalName                                                  | Multiplexed Function 0                                                                                           | Multiplexed Function 1    | Multiplexed Function 2                | Multiplexed Function 3   | Multiplexed Function Input Tap         |
|-------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|---------------------------|---------------------------------------|--------------------------|----------------------------------------|
| PC_00 PC_01 PC_02 PC_03 PC_04 PC_05 PC_06 PC_07 PC_08 PC_09 | SPI2_CITO / SPI2_D1 SPI2_COTI / SPI2_D0 SPI2_D2 SPI2_D3 SPI2_CLK SPI2_SEL1 SPI2_SEL2 SPI2_SEL3 SPI2_RDY SPI0_CLK | ETH0_PTPPPS2 ETH0_PTPPPS3 | eMSI0_SD_VDD1_SEL0 eMSI0_SD_VDD1_SEL1 |                          | TM0_ACLK1 SP12_TS TM0_ACLK13 TM0_ACLK2 |

## Table 12. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port D 1

| SignalName                                                              | Multiplexed Function 0                                                                                                                 | Multiplexed Function 1   | Multiplexed Function 2                                       | Multiplexed Function 3   | Multiplexed Function Input Tap   |
|-------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------|--------------------------|--------------------------------------------------------------|--------------------------|----------------------------------|
| PD_00 PD_01 PD_02 PD_03 PD_04 PD_05 PD_06 PD_07 PD_08 PD_09 PD_10 PD_11 | SPI1_CITO SPI1_COTI SPI1_SEL1 SPI1_RDY SPI1_SEL2 SPI1_SEL3 CAN0_RX CAN0_TX CAN1_RX CAN1_TX xSPI0_CITO / xSPI0_D1 xSPI0_COTI / xSPI0_D0 | eMSI0_UHS1_SWVOLT_EN     | TRACE0_D00 TRACE0_D01 TRACE0_D02 TRACE0_D03 eMSI0_SD_VDD1_ON |                          | SPI1_TS TM0_ACI2 TM0_ACI4        |

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Table 13. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port E 1

| SignalName                                                                                      | Multiplexed Function 0                                                                                                                                                                | Multiplexed Function 1   | Multiplexed Function 2   | Multiplexed Function 3   | Multiplexed Function Input Tap     |
|-------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------|--------------------------|--------------------------|------------------------------------|
| PE_00 PE_01 PE_02 PE_03 PE_04 PE_05 PE_06 PE_07 PE_08 PE_09 PE_10 PE_11 PE_12 PE_13 PE_14 PE_15 | xSPI0_D4 xSPI0_D5 xSPI0_D6 xSPI0_D7 xSPI0_DQS_RWDS xSPI0_SEL1 xSPI0_SEL2 xSPI1_CITO / xSPI1_D1 xSPI1_COTI / xSPI1_D0 xSPI1_D2 xSPI1_D3 xSPI1_CLK xSPI1_CLK xSPI1_D4 xSPI1_D5 xSPI1_D6 | eMSI0_LED_CNTRL          |                          |                          | TM0_ACI12 TM0_ACLK14 FPLL2_REF_CLK |

## Table 14. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port F 1

| SignalName   | Multiplexed Function 0   | Multiplexed Function 1   | Multiplexed Function 2   | Multiplexed Function 3   | Multiplexed Function Input Tap   |
|--------------|--------------------------|--------------------------|--------------------------|--------------------------|----------------------------------|
| PF_00        | xSPI1_D7                 |                          |                          |                          |                                  |
| PF_01        | xSPI1_DQS_RWDS           |                          |                          |                          |                                  |
| PF_02        | xSPI1_SEL1               |                          |                          |                          |                                  |
| PF_03        | xSPI1_SEL2               | eMSI0_CD                 |                          |                          | TM0_ACLK15                       |
| PF_04        | LP0_CLK                  |                          |                          |                          | FPLL3_REF_CLK                    |
| PF_05        | LP0_ACK                  |                          |                          |                          |                                  |
| PF_06        | LP0_D0                   |                          |                          |                          | TM0_ACI13                        |
| PF_07        | LP0_D1                   |                          |                          |                          |                                  |
| PF_08        | LP0_D2                   |                          |                          |                          |                                  |
| PF_09        | LP0_D3                   |                          |                          |                          |                                  |
| PF_10        | LP1_CLK                  |                          |                          |                          |                                  |
| PF_11        | LP1_ACK                  |                          |                          |                          |                                  |
| PF_12        | LP1_D0                   |                          |                          |                          |                                  |
| PF_13        | LP1_D1                   |                          |                          |                          |                                  |
| PF_14        | LP1_D2                   |                          |                          |                          |                                  |
| PF_15        | LP1_D3                   |                          |                          |                          |                                  |

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846

Table 15. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port G 1

| SignalName                                                                    | Multiplexed Function 0                                                                                                                                | Multiplexed Function 1   | Multiplexed Function 2   | Multiplexed Function 3   | Multiplexed Function Input Tap   |
|-------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------|--------------------------|--------------------------|----------------------------------|
| PG_00 PG_01 PG_02 PG_03 PG_04 PG_05 PG_06 PG_07 PG_08 PG_09 PG_10 PG_11 PG_12 | ETH0_MDC ETH0_MDIO ETH0_RXD0 ETH0_RXD1 ETH0_RXCLK_REFCLK ETH0_RXCTL_RXDV ETH0_TXD0 ETH0_TXD1 ETH0_RXD2 ETH0_RXD3 ETH0_TXCTL_TXEN ETH0_TXCLK ETH0_TXD2 | eMSI0_CD                 |                          |                          | TM0_ACLK10                       |

## Table 16. ADSP-2184x/ADSP-SC84x Signal Multiplexing for Port H 1

| SignalName              | Multiplexed Function 0                              | Multiplexed Function 1   | Multiplexed Function 2   | Multiplexed Function 3   | Multiplexed Function Input Tap   |
|-------------------------|-----------------------------------------------------|--------------------------|--------------------------|--------------------------|----------------------------------|
| PH_00 PH_01 PH_02 PH_03 | ETH0_COL ETH0_PHY_INT ETH0_PTPCLKIN0 ETH0_PTPAUXIN0 |                          |                          |                          | TM0_ACLK11 TM0_ACLK12            |

Table 17 shows the internal timer signal routing. This table applies to the 484-ball BGA\_ED package.

Table 17. ADSP-2184x/ADSP-SC84x Internal Timer Signal Routing

| Timer Input Signal   | Internal Source   |
|----------------------|-------------------|
| TM0_ACLK0            | SYS_CLKIN0        |
| TM0_ACI5             | DAI0_PB04         |
| TM0_ACLK5            | DAI0_PB03         |
| TM0_ACI6             | DAI1_PB04         |
| TM0_ACLK6            | DAI1_PB03         |
| TM0_ACI7             | CNT0_TO           |
| TM0_ACLK7            | SYS_CLKIN0        |

Table 17. ADSP-2184x/ADSP-SC84x Internal Timer Signal Routing (Continued)

| Timer Input Signal   | Internal Source   |
|----------------------|-------------------|
| TM0_ACI8             | DAI0_PB06         |
| TM0_ACLK8            | DAI0_PB05         |
| TM0_ACI9             | DAI1_PB06         |
| TM0_ACLK9            | DAI1_PB05         |
| TM0_ACI14            | DAI0 Group C      |
| TM0_ACI15            | DAI1 Group C      |

## Preliminary Technical Data

## OUTLINE DIMENSIONS

Dimensions for the 18 mm × 18 mm 484-ball BGA\_ED package in Figure 6 are shown in millimeters.

![Image](adsp-sc84x_-PrB_artifacts/image_000007_94e5f9d1ea46416bf886ff12c52d01b7cce6ceedd8c25b6c8257341b138d5f15.png)

DETAIL A

Figure 6. 484-Ball Ball Grid Array, Thermally Enhanced [BGA\_ED]

(BP-484-1)

Dimensions shown in millimeters

## SURFACE-MOUNT DESIGN

Table 18 is provided as an aid to PCB design. For industry-standard design recommendations, refer to IPC-7351, Generic Requirements for Surface-Mount Design and Land Pattern Standard .

Table 18. BGA Data for Use with Surface-Mount Design

| Package   | Package Ball Attach Type   | Package Solder Mask Opening   | Package Ball Pad Size   |
|-----------|----------------------------|-------------------------------|-------------------------|
| BP-484-1  | Solder Mask Defined        | 0.4 mmDiameter                | 0.5 mmDiameter          |

I 2 C refers to a communications protocol originally developed by Philips Semiconductors (now NXP Semiconductors).

© 2025  Analog  Devices,  Inc.  All  rights  reserved.  Trademarks  and registered trademarks are the property of their respective owners.

![Image](adsp-sc84x_-PrB_artifacts/image_000008_0d1cfc4a3865952abea7c87ca8a45f6cea6e8a189f17c0df778ef1c142b19161.png)

## ADSP-21844/ADSP-21846/ADSP-SC844/ADSP-SC846