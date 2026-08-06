# Arm Cortex-A5 Subsystem

<!-- source: 004_Arm_Cortex-A5_Subsystem.pdf | original pages 109–117 -->

## 2   Arm Cortex-A5 Subsystem

The ADSP-SC592/4 processor includes an Arm ®  Cortex-A5 ®  core. The Arm Cortex-A5 processor is the smallest, lowest cost and lowest power Armv7 application processor. The A5 sub-system in the ADSP-SC592/4 processor includes a Floating-Point Unit, NEON Media Processing Engine, Generic Interrupt Controller and a Level 2 Cache Controller. The A5 also includes support for a L1-Cache sub-system and a full-fledged Memory Management Unit. The A5 implements the Armv7 architecture and runs 32-bit Arm instructions, 16-bit and 32-bit Thumb instructions, and 8-bit Java byte-codes in Jazelle state.

This document describes the Arm Cortex-A5 core and memory architecture used on the ADSP-SC592/4 processor, but does not provide detailed programming information for the Arm processor. For more information about programming the Arm processor, visit the Arm Information Center:

- http://infocenter.arm.com.

The applicable documentation for programming the Arm Cortex-A5 processor include:

- Cortex-A5 Technical Reference Manual, Revision: r0p1
- Cortex-A Series Programmer's Guide, Revision: r0p1
- Cortex-A5 NEON Media Processing Engine Technical Reference Manual, Revision: r0p1
- Cortex-A5 Floating-Point Unit Technical Reference Manual, Revision: r0p1
- CoreLink Level 2 Cache Controller L2C-310 Technical Reference Manual, Revision: r3p3
- PrimeCell Generic Interrupt Controller (PL390) Technical Reference Manual, Revision: r0p0

## Cortex A5 Features

The Cortex-A5 subsystem has the following features.

- Thumb / Arm Instruction support
- L1- Instruction Cache and L1-Data Cache
- Floating Point Unit (FPU)
- NEON Media Processing Engine (NEON)

- Generic Interrupt Controller (GIC)
- Level 2 Cache Controller (L2CC)
- Memory Management Unit (MMU)

## Functional Description

The following sections provide information on the function of the subsystem.

## A5 Block Diagram

The following figure shows the primary blocks of the Cortex A5 subsystem. The performance increases with accesses to lower levels of memory as follows:

1. Level 1 - cache on-chip, separate data/code (highest)
2. Level 2 - cache on-chip, unified
3. Level 3 - memory external, (lowest)

Figure 2-1: A5 Subsystem Block Diagram

<!-- image -->

## Control Coprocessor (CP15)

The system control coprocessor, CP15, controls and provides status information for the functions implemented in the processor. The main functions of the system control coprocessor are:

- Overall system control and configuration
- MMU configuration and management
- Cache configuration and management
- System performance monitoring

All system architecture functions are controlled by reading or writing a general purpose processor register (Rt) from or to a set of registers (CRn) located within co-processor 15. The Op1, Op2, and CRm fields of the instruction can also be used to select registers or operations.

- MRC p15, Op1, Rt, CRn, CRm, Op2; read a CP15 register into an Arm register
- MCR p15, Op1, Rt, CRn, CRm, Op2; write a CP15 register from an Arm register

## L1 Cache

The Cortex-A5 processor has separate instruction and data caches that run at Arm core clock speed. The caches have the following features:

- L1 data cache size 32 KB
- L1 instruction cache size 32 KB
- Each cache can be disabled independently, using the system control coprocessor
- Cache replacement policy is pseudo random
- Data cache is 4-way set-associative
- Instruction cache is 2-way set-associative
- The cache line length is eight words.
- On a cache miss, critical word first filling of the cache is performed.

## Prefetch Unit (PFU)

The PFU implements a two-level prediction mechanism, comprising the following:

- A 256 entry branch pattern history table
- A four-entry BTAC
- A four-entry return stack

## Memory Management Unit (MMU)

The Arm MMU is responsible for translating addresses of code and data from the virtual view of memory to the physical addresses in the real system. The translation is carried out by the MMU hardware and is transparent to the application. In addition, the MMU controls such things as memory access permissions, memory ordering and cache policies for each region of memory.

The MMU enables tasks or applications to be written in a way that requires them to have no knowledge of the physical memory map of the system, or about other programs that might be running simultaneously. This enables programs to use the same virtual memory address space. It also lets programs work with a contiguous virtual memory map, even if the physical memory is fragmented. This virtual address space is separate from the actual physical map of memory in the system. Applications are written, compiled and linked to run in the virtual memory space. Virtual addresses are those used by you, and the compiler and linker, when placing code in memory. Physical addresses are those used by the actual hardware system.

The first level MMU uses a Harvard design with separate micro TLB structures in the PFU for instruction fetches and in the DPU for data read and write requests. A miss in the micro TLB results in a request to the main unified TLB shared between the data and instruction sides of the memory system. The TLB consists of a 128-entry two-way set-associative RAM based structure. The TLB page-walk mechanism supports page descriptors held in the L1 data cache. The caching of page descriptors is configured globally for each translation table base register, TTBRx, in the system coprocessor, CP15.

Page table entries support:

- 16 MB super sections
- 1 MB sections
- 64 KB large pages
- 4 KB small pages
- NOTE: Virtual memory translation tables are typically created by operating systems, and are often dynamically managed by the memory management layer. However, even a bare metal system can enable the MMU. For this, a flat mapping technique is used, where all virtual memory addresses shall be programmed exactly same as the physical memory addresses in the system.
- NOTE: In order to use L1 data cache, the application has to enable the MMU, via coprocessor 15 in the Arm core. After the MMU and L1 data cache are enabled via CP15: SCTLR, the application can disable/enable cache for individual pages/sections.

## L2 Cache

The Level 2 Cache Controller in the processor is a CoreLink Level 2 Cache Controller (L2C-310) from Arm and is clocked at SYSCLK speed. The addition of an on-chip secondary cache, also referred to as a level 2 or L2 cache, is a recognized method of improving the performance of Arm-based systems when significant memory traffic is generated by the processor. By definition a secondary cache assumes the presence of a Level 1 or primary cache, closely coupled or internal to the processor. The cache controller is a unified, physically addressed, physically tagged cache. It includes the following features:

- 256 KB total size
- Lockdown by Line/Way/Requester
- Fixed line length of 32 bytes, eight words or 256 bits

- Direct mapped to 8-way associativity (fixed)
- Prefetching capability
- Event monitoring
- Software option to enable exclusive cache configuration
- Additional Buffers:
- Line Fill Buffers (LFBs)
- Line Read Buffers (LRBs)
- Eviction Buffers (EBs)
- Store Buffers (STBs)
- TrustZone support, with the following features:
- Non-Secure (NS) tag bit added in tag RAM and used for lookup in the same way as an address bit. The NS-tag bit is added in all buffers.
- NS bit in Tag RAM used to determine security level of evictions to L3.
- Restrictions for NS accesses for control, configuration, and maintenance registers to restrict access to secure data.
- Parity Support

NOTE: The L2CC address filtering registers should not be programmed by the user. Not retaining the reset values can give unpredictable results.

## Sharing L2 Cache with SHARC+ Cores

The L2CC (PL310) supports two requester and two completer ports. The SHARC+ core can access the L2 cache without bank conflict versus the Cortex A5 core by programming the L2CC registers. The cache access is restricted to the address range: from CMMR\_L2CC\_START [31:0] to CMMR\_L2CC\_END [31:0]. For more information, see the SHARC+ Core Programming Reference .

NOTE: There is no guarantee for the data coherency between the A5 and SHARC+ cores.

- NOTE: Programs should perform L2 cache write-back invalidation before changing the value of CMMR\_L2CC\_START and CMMR\_L2CC\_END .

## Floating-Point Unit (FPU)

The Cortex-A5 FPU is a VFPv4-D16 implementation of the Armv7 floating-point architecture. It provides low cost high performance floating-point computation. The FPU supports all addressing modes and operations described in the Arm Architecture Reference Manual .

The features in the FPU are as follows.

- Support for single-precision and double-precision floating-point formats
- Support for conversion between half-precision and single-precision
- Support for Fused Multiply Accumulate (FMA) operations
- Normalized and denormalized data are all handled in hardware
- Trapless operation enabling fast execution

## NeON

The Cortex-A5 NEON MPE extends the Cortex-A5 functionality to provide support for the Arm v7 Advanced SIMD v2 and Vector Floating-Point v4 (VFPv4) instruction sets. The Cortex-A5 NEON MPE supports all addressing modes and data-processing operations described in the Arm Architecture Reference Manual .

The Cortex-A5 NEON MPE features are:

- SIMD and scalar single-precision floating-point computation
- scalar double-precision floating-point computation
- SIMD and scalar half-precision floating-point conversion
- SIMD 8, 16, 32, and 64-bit signed and unsigned integer computation
- 8 or 16-bit polynomial computation for single-bit coefficients
- Structured data load capabilities
- Large, shared register file, addressable as:
- 32 32-bit S (single) registers
- 32 64-bit D (double) registers
- 16 128-bit Q (quad) registers
- The operations include:
- Addition and subtraction
- Multiplication with optional accumulation
- Maximum or minimum value driven lane selection operations
- Inverse square root approximation
- Comprehensive data structure load instructions, including register-bank-resident table lookup

See the Arm Architecture Reference Manual for details of the extension register set.

## Generic Interrupt Controller (GIC)

The GIC is an Arm architecture compliant System-on-Chip (SoC) peripheral. It is a high-performance, area-optimized interrupt controller. The GIC implements the Arm generic interrupt controller architecture. The GIC takes interrupts asserted at the system level and signals them to each connected processor as appropriate. The GIC has the following features.

- Registers for managing interrupt sources, interrupt behavior, and interrupt routing to one or more processors
- Support for the Arm architecture security extensions
- Support for enabling, disabling, and generating processor interrupts from hardware (peripheral) interrupt sources
- Support for generating software interrupts
- Support for interrupt masking and prioritization

Refer to the GIC Overview topic for more information on the processor-specific configuration of GIC.

## A5 Configurations

The following are the Cortex A5 and processor configurations.

## A5 Configurations

Table 2-1: A5 Core Configuration

| Core Feature           | Comment     |
|------------------------|-------------|
| JAZELLE Support        | Implemented |
| NEON Engine            | Implemented |
| FPU                    | Implemented |
| Instruction cache size | 32 KB       |
| Data cache size        | 32 KB       |

## A5 Configuration Signals

Table 2-2: A5 Configuration Signals

| Configuration                               | A5 TRM Signal Name   | Comment                                       |
|---------------------------------------------|----------------------|-----------------------------------------------|
| Default Exception Handler Endianness        | CFGEND               | Little Endian                                 |
| CPU ID field                                | CLUSTERID[3:0]       | 4'b0000                                       |
| Disable Write access to some CP15 registers | CP15SDISABLE         | Not Enabled                                   |
| Default exception handling state            | TEINIT               | ARM Mode                                      |
| Exception vectors' location at reset        | VINITHI              | start exception vectors at address 0x00000000 |

Table 2-2: A5 Configuration Signals (Continued)

| Configuration                                                              | A5 TRM Signal Name   | Comment           |
|----------------------------------------------------------------------------|----------------------|-------------------|
| Disable invalidate entire data cache, instruc- tion cache and TLB at reset | L1RSTDISABLE         | Disabled          |
| Enable the RAM interface clamps                                            | CPURAMCLAMP          | clamps not active |

## A5 Power Modes

Table 2-3: ARM Core Power Modes

| Mode          | Comment       |
|---------------|---------------|
| Run mode      | Supported     |
| Standby mode  | Supported     |
| Dormant mode  | Not Supported |
| Shutdown mode | Not Supported |

## L2CC Configuration Signals

Table 2-4: L2CC Configuration Signals

| Configuration                                                  | L2CC TRM Signal Name   | Comment       |
|----------------------------------------------------------------|------------------------|---------------|
| Associativity                                                  | ASSOCIATIVITY          | 8-Way         |
| Cache controller cache ID                                      | CACHEID[5:0]           | 0             |
| Address filtering Enable out of reset                          | CFGADDRFILTEN          | Enabled       |
| Address filtering End Address out of reset                     | CFGADDRFILTEND[11:0]   | 0xFFF         |
| Address filtering Start Address out of reset                   | CFGADDRFILTSTART[11:0] | 0x201         |
| Endian mode for accessing configuration registers out of reset | CFGBIGEND              | Little-endian |
| Base address for accessing configuration registers             | REGFILEBASE[19:0]      | 0x10000       |
| Size of ways                                                   | WAYSIZE[2:0]           | 32 KB         |

## L2CC Power Down Modes

Table 2-5: L2CC Power Down Modes

| Mode                 | Comment       |
|----------------------|---------------|
| Run mode             | Supported     |
| Dynamic Clock Gating | Supported     |
| Standby mode         | Not Supported |
| Dormant mode         | Not Supported |

Table 2-5: L2CC Power Down Modes (Continued)

| Mode          | Comment       |
|---------------|---------------|
| Shutdown mode | Not Supported |

## L2CC Configuration

## Table 2-6: L2CC Configuration

| Feature                  | Comment   |
|--------------------------|-----------|
| Cache way size           | 32 KB     |
| Associativity            | 8 Ways    |
| Default RAM latencies    | 2 cycles  |
| DATA RAM banking         | Disabled  |
| Completer port 1 present | Enabled   |
| Requester port 1 present | Enabled   |
| Parity logic             | Enabled   |
| Lock down by requester   | Enabled   |
| Lock down by line        | Enabled   |
| Address filtering        | Enabled   |
| Speculative reading      | Disabled  |