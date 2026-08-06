# ADSP-SC59x Documentation Index

> Comprehensive navigation guide for Analog Devices ADSP-SC595/SC596/SC598 SHARC+ processor documentation.
> Processors: Dual SHARC+ DSP cores + ARM Cortex-A55, up to 1.2 GHz, 32/40/64-bit floating-point.

---

## Document Inventory

| # | Document Set | Files | Description |
|---|-------------|-------|-------------|
| 1 | [Data Sheet (DS)](#1-data-sheet-ds) | 1 | Electrical specs, pinout, timing, features overview |
| 2 | [Hardware Reference Manual (HRM)](#2-hardware-reference-manual-hrm) | 57 | Complete peripheral and system block documentation |
| 3 | [Programming Reference Manual (PRM)](#3-programming-reference-manual-prm) | 1 | SHARC+ core ISA, registers, computation units |
| 4 | [Engineer-to-Engineer Notes (EE)](#4-engineer-to-engineer-notes-ee) | 14 | Application notes: boot, power, board design, optimization |
| 5 | [Silicon Anomaly List](#5-silicon-anomaly-list) | 1 | 18 known errata for Rev 0.0 silicon |
| 6 | [Board & Tools](#6-board--tools-documentation) | 4 | EZ-KIT SOM manual, schematic, emulator guide, package |
| 7 | [SVD Register Descriptions](#7-svd-register-descriptions) | 2 | CMSIS-SVD peripheral register maps for SC598 and SC598W |

**Total: 81 markdown files + 2 SVD files**

---

## Quick-Find by Topic

| Topic | Where to Look |
|-------|--------------|
| **Pin assignments / BGA ballmap** | DS `adsp-sc596-adsp-sc598.md` (Tables 9-10, lines 1350+) |
| **Electrical specifications / timing** | DS `adsp-sc596-adsp-sc598.md` (lines 2200-4100) |
| **Register addresses (full list)** | HRM `326_ADSP-SC59x_Register_List_pages.md` |
| **SHARC+ instruction set** | PRM `sc58x-2158x-prm.md` (Chapters 13-27) |
| **SHARC+ register definitions** | PRM `sc58x-2158x-prm.md` (Chapters 29-35) |
| **Boot process / boot modes** | HRM `321_Boot_ROM_and_Booting_the_Processor.md` |
| **Boot time estimation** | EE `ee432.md` (SC59x general), `ee438v01.md` (SC598 specific) |
| **Clock/PLL configuration** | HRM `004_Clock_Generation_Unit_CGU.md`, `005_Clock_Distribution_Unit_CDU.md` |
| **DDR3 memory controller** | HRM `011_Dynamic_Memory_Controller_DMC.md` |
| **DDR3 board layout** | EE `ee434.md` (SC59x), `ee-441.md` (SC596/SC598) |
| **DDR3 programming/init code** | EE `ee-443.md` |
| **Power estimation** | EE `ee433.md` (SC59x), `ee440v01.md` (SC596/SC598) |
| **Thermal design** | EE `ee451v01.md` |
| **DMA** | HRM `310_Direct_Memory_Access_DMA.md`, `311_Extended_Memory_DMA_EMDMA.md` |
| **Audio interfaces (SPORT/I2S/TDM)** | HRM `306_Serial_Port_SPORT.md` |
| **Audio sample rate conversion** | HRM `308_Asynchronous_Sample_Rate_Converter_ASRC.md` |
| **S/PDIF** | HRM `309_SonyPhilips_Digital_Interface_SPDIF.md` |
| **PDM microphones** | HRM `022_Pulse_Density_Modulation_PDM_Microphone_Interface.md` |
| **Precision clocks (audio)** | HRM `307_Precision_Clock_Generators_PCG.md` |
| **Ethernet (EMAC)** | HRM `032`-`037` + `304` (6 files covering features, programming, registers) |
| **CAN FD** | HRM `018_Controller_Area_Network_Flexible_Data_Rate_CANFD.md` |
| **SPI** | HRM `024_Serial_Peripheral_Interface_SPI.md` |
| **OSPI (Octal SPI)** | HRM `025_Octal_Serial_Peripheral_Interface_OSPI.md` |
| **OSPI PHY training** | EE `ee-437.md` |
| **UART** | HRM `026_Universal_Asynchronous_ReceiverTransmitter_UART.md` |
| **I2C / TWI** | HRM `031_Two-Wire_Interface_TWI.md` |
| **USB 2.0 OTG** | HRM `023_Universal_Serial_Bus_Controller_USBC.md` |
| **eMMC / SD card (eMSI)** | HRM `020_Enhanced_Mobile_Storage_Interface_eMSI.md` |
| **eMSI optimization & anomalies** | EE `ee444v01.md` |
| **GPIO / pin mux** | HRM `015_General-Purpose_Ports_PORT.md` |
| **Timers / PWM** | HRM `028_General-Purpose_Timer_TIMER.md` |
| **Watchdog** | HRM `019_Watchdog_Timer_WDOG.md` |
| **Counters / encoders** | HRM `029_General-Purpose_Counter_CNT.md` |
| **Parallel interface (EPPI)** | HRM `027_Enhanced_Parallel_Peripheral_Interface_EPPI.md` |
| **Link port** | HRM `021_Link_Port_LP.md` |
| **MediaLB / MOST** | HRM `030_Media_Local_Bus_MLB.md` |
| **FIR accelerator** | HRM `319_FIR_Accelerator_FIR.md` |
| **IIR accelerator** | HRM `320_IIR_Accelerator_IIR.md` |
| **FIR/IIR usage & performance** | EE `ee436v02.md` |
| **Interrupts (SEC/GIC)** | HRM `008_System_Event_Controller_SEC_and_Generic_Interrupt_Controller.md` |
| **Trigger routing** | HRM `009_Trigger_Routing_Unit_TRU.md` |
| **L2 memory** | HRM `010_L2_System_Memory.md` |
| **DDR prefetch buffer** | HRM `012_DDR_Pre-fetch_Buffer_DDRPFB.md` |
| **OTP memory** | HRM `013_One-Time_Programmable_Memory_Controller_OTPC.md` |
| **Memory protection** | HRM `014_System_Memory_Protection_Unit_SMPU.md` |
| **Security (crypto, keys, TRNG)** | HRM `313`-`318` (6 files) |
| **System protection (SPU)** | HRM `314_System_Protection_Unit_SPU.md` |
| **System crossbars / bus** | HRM `322_System_Crossbars_SCB.md` |
| **System optimization** | EE `ee445v01.md` |
| **CRC** | HRM `312_Cyclic_Redundancy_Check_CRC.md` |
| **Power management** | HRM `006_Dynamic_Power_Management_DPM.md` |
| **Reset control** | HRM `007_Reset_Control_Unit_RCU.md` |
| **Safe power-down** | EE `ee448v01.md` |
| **Thermal monitoring** | HRM `016_Thermal_Monitoring_Unit_TMU.md` |
| **Housekeeping ADC** | HRM `017_Housekeeping_ADC_HADC.md` |
| **Debug / trace** | HRM `325_System_Debug_and_Trace_Unit_DBG.md` |
| **Watchpoint unit** | HRM `323_System_Watchpoint_Unit_SWU.md` |
| **Memory error protection** | HRM `324_Memory_Error_Protection_Unit_MEPU.md` |
| **ARM Cortex-A55 subsystem** | HRM `003_Arm_Cortex-A55_Subsystem.md` |
| **Migration from ADSP-2156x** | EE `ee-430.md` |
| **Silicon errata / anomalies** | Anomaly `adsp-sc595-sc596-sc598-anomaly.md` |
| **SVD register map (SC598)** | `ADSP-SC598.svd` (CMSIS-SVD, 93 peripherals) |
| **SVD register map (SC598W)** | `ADSP-SC598W.svd` (CMSIS-SVD, 95 peripherals) |
| **EZ-KIT SOM board** | `ev-sc598-som_manual.md`, `ev-sc598-som-schematic.md` |
| **ICE-1500 emulator** | `mb_ice1500_emu_ug.md` |
| **BGA package drawing** | `bp-400-3.md` |

---

## 1. Data Sheet (DS)

**File:** `ADSP-SC59x-DS/adsp-sc596-adsp-sc598.md` (4,759 lines)

Covers SC596 (single SHARC+, 1 GHz) and SC598 (dual SHARC+, up to 1 GHz + ARM A55 at 1.2 GHz).

| Section | Content | Lines (approx) |
|---------|---------|----------------|
| System Features | Feature overview, processor comparison table | 3-189 |
| ARM Cortex-A55 | Core features, caches, GIC-600 | 189-255 |
| SHARC+ Core | Architecture, SIMD, DAGs, BTB, ISA/VISA | 256-396 |
| System Infrastructure | Memory map, L2, OTP, crossbars | 400-501 |
| DMA & CRC | DMA subsystem, memory protection via CRC | 502-554 |
| Security | TrustZone, crypto accelerators, SPU, SMPU | 587-639 |
| Safety | ECC, parity, CRC scrubbing, watchdogs, MEC | 643-689 |
| Peripherals | All peripheral descriptions (see Quick-Find) | 697-1050+ |
| System Design | Clocking, reset, power domains, JTAG | 1100-1300 |
| Signal Descriptions | Pin-by-pin signal tables, GPIO mux tables | 1350-2000 |
| Electrical Specs | Operating conditions, currents, power dissipation | 2200-2900 |
| Timing Specs | All interface timing parameters | 2900-4100 |
| Output Drive | Drive strength tables | 4220-4330 |
| Package | 400-ball BGA_ED assignments, outline, ordering | 4408-4759 |

---

## 2. Hardware Reference Manual (HRM)

**Directory:** `ADSP-SC59x-HRM/` (57 files)

### 2.1 Front Matter
| File | Content |
|------|---------|
| `000_Notices.md` | Copyright, disclaimers, trademarks |
| `001_Contents.md` | Original table of contents with page references |
| `002_Preface.md` | Audience, conventions, support resources |

### 2.2 Core & System Infrastructure
| File | Chapter | Topic |
|------|---------|-------|
| `003_Arm_Cortex-A55_Subsystem.md` | 1 | ARM A55 core: AArch32/64, SIMD, crypto, MMU, caches, GIC, PMU |
| `004_Clock_Generation_Unit_CGU.md` | 2 | PLL, clock outputs (CCLK, SYSCLK, SCLK, DCLK, OCLK), watchdog |
| `005_Clock_Distribution_Unit_CDU.md` | 3 | Clock mux routing, 14 output clocks |
| `006_Dynamic_Power_Management_DPM.md` | 4 | Power states, peripheral clock gating |
| `007_Reset_Control_Unit_RCU.md` | 5 | HW/SW reset, boot vectors, core handshake |
| `008_System_Event_Controller_SEC_and_Generic_Interrupt_Controller.md` | 6 | SEC (SHARC+), GIC (ARM), fault mgmt, interrupt lists |
| `009_Trigger_Routing_Unit_TRU.md` | 7 | DMA-to-DMA triggers, SW triggers, synchronization |

### 2.3 Memory Subsystem
| File | Chapter | Topic |
|------|---------|-------|
| `010_L2_System_Memory.md` | 8 | 2 MB SRAM, 8 banks, ECC, L2CTL, scrub |
| `011_Dynamic_Memory_Controller_DMC.md` | 9 | DDR3/DDR3L, 16-bit interface, timing, refresh, perf monitoring |
| `012_DDR_Pre-fetch_Buffer_DDRPFB.md` | 10 | Smart prefetch, direct access, return-zero |
| `013_One-Time_Programmable_Memory_Controller_OTPC.md` | 11 | OTP with ECC, secure keys, boot config, anti-rollback |
| `014_System_Memory_Protection_Unit_SMPU.md` | 12 | 8 regions/instance, R/W/secure protection, bus errors |

### 2.4 GPIO, Monitoring & Misc
| File | Chapter | Topic |
|------|---------|-------|
| `015_General-Purpose_Ports_PORT.md` | 13 | GPIO, pin mux (PORT_FER/PORT_MUX), PINT interrupts |
| `016_Thermal_Monitoring_Unit_TMU.md` | 14 | 12-bit temp sensor, alert/fault thresholds |
| `017_Housekeeping_ADC_HADC.md` | 15 | 12-bit ADC, 8 channels, 1 MSPS, auto-scan |
| `019_Watchdog_Timer_WDOG.md` | 17 | 32-bit WDT, window mode, fault interface |

### 2.5 Communication Interfaces
| File | Chapter | Topic |
|------|---------|-------|
| `018_Controller_Area_Network_Flexible_Data_Rate_CANFD.md` | 16 | CAN 2.0B + CAN FD, 28 mailboxes, Rx FIFO, 8 Mbps |
| `020_Enhanced_Mobile_Storage_Interface_eMSI.md` | 18 | eMMC 5.1 / SD, SDMA/ADMA, command queuing |
| `021_Link_Port_LP.md` | 19 | 8-bit parallel, point-to-point, DDR modes |
| `022_Pulse_Density_Modulation_PDM_Microphone_Interface.md` | 20 | 4 PDM inputs, decimation, I2S/TDM output |
| `023_Universal_Serial_Bus_Controller_USBC.md` | 21 | USB 2.0 OTG, HS/FS/LS, 12 endpoints, ULPI |
| `024_Serial_Peripheral_Interface_SPI.md` | 22 | SPI full-duplex, Quad SPI, 8/16/32-bit |
| `025_Octal_Serial_Peripheral_Interface_OSPI.md` | 23 | OSPI: 1/2/4/8-bit, STR/DTR, XIP, STIG, PHY mode |
| `026_Universal_Asynchronous_ReceiverTransmitter_UART.md` | 24 | UART, IrDA SIR, HW flow control, DMA |
| `027_Enhanced_Parallel_Peripheral_Interface_EPPI.md` | 25 | 8-24 bit parallel, ITU-656, RGB conversion |
| `029_General-Purpose_Counter_CNT.md` | 27 | Quadrature/binary encoder, 32-bit counter |
| `030_Media_Local_Bus_MLB.md` | 28 | MOST25/50/150, 64 channels, LVDS |
| `031_Two-Wire_Interface_TWI.md` | 29 | I2C (100K/400K), SCCB, multi-controller |

### 2.6 Ethernet (6 files)
| File | Chapter | Topic |
|------|---------|-------|
| `032_..._EMAC_Features.md` | 30a | 10/100/1000, RMII/RGMII, VLAN, PTP, TSN, AVB |
| `033_..._Functional_Descri.md` | 30b | Tx/Rx paths, MTL, DMA descriptors |
| `034_..._Event_Control.md` | 30c | Interrupt sources, event masking |
| `035_..._Programming_Model.md` | 30d | Init sequence, descriptor ring setup |
| `036_..._Programming_Conce.md` | 30e | PTP config, VLAN filtering, power management |
| `037_..._Regist.md` + `304_..._Regist.md` | 30f-g | Register definitions (split across 2 files) |

### 2.7 Audio Subsystem
| File | Chapter | Topic |
|------|---------|-------|
| `306_Serial_Port_SPORT.md` | 31 | 8 SPORTs, I2S/TDM/DSP modes, full-duplex, DMA |
| `307_Precision_Clock_Generators_PCG.md` | 32 | 8 PCGs, clock + frame sync generation |
| `308_Asynchronous_Sample_Rate_Converter_ASRC.md` | 33 | 8 ASRCs, 140 dB SNR, jitter cleanup |
| `309_SonyPhilips_Digital_Interface_SPDIF.md` | 34 | S/PDIF Tx/Rx, AES3, 24-192 kHz |

### 2.8 DMA
| File | Chapter | Topic |
|------|---------|-------|
| `310_Direct_Memory_Access_DMA.md` | 35 | Peripheral DMA, descriptor chains, scatter-gather |
| `311_Extended_Memory_DMA_EMDMA.md` | 36 | Memory-to-memory EMDMA, 2D transfers |

### 2.9 Timers
| File | Chapter | Topic |
|------|---------|-------|
| `028_General-Purpose_Timer_TIMER.md` | 26 | 16 timers, PWM, capture, counter, windowed WDT |

### 2.10 Security
| File | Chapter | Topic |
|------|---------|-------|
| `312_Cyclic_Redundancy_Check_CRC.md` | 37 | CRC accelerator, memory scrubbing |
| `313_System_Security.md` | 38 | Secure boot, TrustZone integration, key management |
| `314_System_Protection_Unit_SPU.md` | 39 | MMR write protection |
| `315_Security_Packet_Engine_PKTE.md` | 40 | AES/DES/SHA/HMAC/MD5 hardware acceleration |
| `316_Public_Key_Accelerator_PKA.md` | 41 | RSA/ECC public key operations |
| `317_Public_Key_Interrupt_Controller_PKIC.md` | 42 | PKA interrupt management |
| `318_True_Random_Number_Generator_TRNG.md` | 43 | Hardware TRNG |

### 2.11 Accelerators
| File | Chapter | Topic |
|------|---------|-------|
| `319_FIR_Accelerator_FIR.md` | 44 | 1024-tap, 4 MACs, concurrent operation |
| `320_IIR_Accelerator_IIR.md` | 45 | Biquad filter, 1440-word coefficient memory |

### 2.12 System & Debug
| File | Chapter | Topic |
|------|---------|-------|
| `321_Boot_ROM_and_Booting_the_Processor.md` | 46 | Boot modes (SPI, OSPI, eMMC, LP, UART), secure boot |
| `322_System_Crossbars_SCB.md` | 47 | Bus fabric, arbitration, QoS |
| `323_System_Watchpoint_Unit_SWU.md` | 48 | Transaction monitoring, 4 match groups |
| `324_Memory_Error_Protection_Unit_MEPU.md` | 49 | Error detection/reporting |
| `325_System_Debug_and_Trace_Unit_DBG.md` | 50 | JTAG, CoreSight trace, STPv2 |
| `326_ADSP-SC59x_Register_List_pages.md` | 51 | Complete register address list |

---

## 3. Programming Reference Manual (PRM)

**File:** `ADSP-SC59x-PRM/sc58x-2158x-prm.md` (22,334 lines)

SHARC+ Core Programming Reference, Rev 1.5 (June 2023). Covers ADSP-SC5xx and ADSP-215xx.

| Chapter | Topic | Lines (approx) |
|---------|-------|----------------|
| **Architecture (Ch 1-12)** | | |
| 1 - Introduction | Core overview, block diagram, dual PEs, DAGs, CEC, BTB | 1061-1408 |
| 2 - Register File | Data regs, complementary pairs, CMMR, operating modes | 1409-1768 |
| 3 - Processing Elements | ALU, Multiplier, Barrel Shifter, SIMD, 64-bit ops | 1769-3056 |
| 4 - Program Sequencer | Pipeline (11-stage), IAB, branches, BTB, delayed branches | 3057-5306 |
| 5 - Timer | Core timer, exceptions | 5307-5362 |
| 6 - Data Address Generators | DAG1/DAG2, addressing modes, circular buffers, bit-reverse | 5363-6176 |
| 7 - L1 Memory Interface | Super Harvard architecture, memory blocks, arbitration | 6177-6899 |
| 8 - L1 Cache Controller | I-cache, D-cache, hit/miss, coherency, locking, invalidation | 6900-7287 |
| 9 - Safety/Security/Multi-core | Parity, SYNC, semaphores, low-power idle/sleep/shutdown | 7288-7508 |
| 10 - Debug Interface | Breakpoints, emulation, cycle counting, statistical profiling | 7509-7767 |
| 11 - Performance Monitor | PFM counters | 7768-7813 |
| 12 - Program Trace Macrocell | CoreSight PTM, address comparators, trace security | 7814-7889 |
| **Instruction Set (Ch 13-27)** | | |
| 13 - ISA Overview | ISA/VISA instruction types and opcodes | 7890-7964 |
| 14 - Group I | Conditional compute + move/modify | 7965-9500~ |
| 15 - Group II | Conditional program flow control | ~9500-10500 |
| 16 - Group III | Immediate data move | ~10500-11500 |
| 17 - Group IV | Miscellaneous instructions | ~11500-12353 |
| 18 - Computation Opcodes | All compute operation encodings | 12354-12919 |
| 19-25 | ALU/Multiplier/Shifter fixed & float ops, multifunction | 12920-16192 |
| 26-27 | Immediate/constant and register opcode encodings | 16193-17342 |
| **Reference (Ch 28-36)** | | |
| 28 - Numeric Formats | 32/40/64-bit fixed & float formats | 17343-17447 |
| 29-34 | Register descriptions (REGF, CMMR, BTB, DBG, L1C, PFM) | 17448-20294 |
| 35 - Register List | Complete tabular register reference | 20295-20591 |
| 36 - Glossary | ~400 lines of terminology | 20592-21155+ |

---

## 4. Engineer-to-Engineer Notes (EE)

**Directory:** `ADSP-SC59x-EE-Notes/` (14 files)

### By Topic Area

#### Migration
| File | Title | Lines |
|------|-------|-------|
| `ee-430.md` | Migrating from ADSP-2156x to ADSP-SC59x | 193 |

#### Boot Time
| File | Title | Lines |
|------|-------|-------|
| `ee432.md` | Boot Time Estimation for SC59x/2159x (SPI, OSPI, LP, UART) | 497 |
| `ee438v01.md` | Boot Time Estimation for SC598 (adds eMMC, SD card) | 718 |

#### Power & Thermal
| File | Title | Lines |
|------|-------|-------|
| `ee433.md` | Power Estimation for SC59x/2159x (A5 core variants) | 878 |
| `ee440v01.md` | Power Estimation for SC596/SC598 (A55 core variants) | 798 |
| `ee448v01.md` | Safe Power-down Mechanisms (failure detection, mitigations) | 289 |
| `ee451v01.md` | Thermal Guidelines for SC596/SC598 (ECXML model, measurement) | 188 |

#### Board Design (DDR3 / DMC)
| File | Title | Lines |
|------|-------|-------|
| `ee434.md` | DMC Board Design Guidelines for SC59x/2159x | 309 |
| `ee-441.md` | DMC Board Design Guidelines for SC596/SC598 | 326 |
| `ee-443.md` | DMC Programming Guidelines for SC595/SC596/SC598 (init code) | 385 |

#### Accelerators & Performance
| File | Title | Lines |
|------|-------|-------|
| `ee436v02.md` | FIR/IIR Accelerator Usage & Performance (benchmarks, coding) | 928 |
| `ee445v01.md` | System Optimization (bus architecture, DMA, memory throughput) | 898 |

#### Interfaces
| File | Title | Lines |
|------|-------|-------|
| `ee-437.md` | OSPI PHY Configuration and Training | 236 |
| `ee444v01.md` | eMSI Optimization for SC598 (anomaly workarounds, eMMC/SD) | 243 |

---

## 5. Silicon Anomaly List

**File:** `ADSP-SC59x-Anomaly/adsp-sc595-sc596-sc598-anomaly.md` (515 lines)

18 anomalies for Rev 0.0 silicon, grouped by subsystem:

| ID | Subsystem | Summary |
|----|-----------|---------|
| 20000002 | CPU Pipeline | Data forwarding Rn/Sn to DAG may fail with stalls |
| 20000003 | SPU/SMPU | Non-secure MMR access to upper half may error |
| 20000031 | GP Timer | First interrupt one edge late in EXTCLK mode |
| 20000062 | SPI | Single write to SPI_SLVSEL has no effect (write twice) |
| 20000069 | CPU Pipeline | PCSTK/MODE1STK loads fail if next instr is L2/L3 access |
| 20000072 | CPU Pipeline | Float compute targeting F0 can cause stalls |
| 20000096 | CPU Pipeline | Type 18a USTAT instructions fail after specific sequences |
| 20000103 | SPDIF | Unreliable Rx clock pulse width above 96 kHz |
| 20000114 | FIR Accel | Circular buffering fails in burst-16 mode |
| 20000117 | DMC | PHY calibration issue (76-step workaround provided) |
| 20000118 | FIR Accel | Wrong output for tap length > 1024 in multi-iteration |
| 20000119 | eMSI | END bit error during data transfers (clock gating) |
| 20000120 | eMSI/Boot | eMMC boot failure due to clock gating (related to 20000119) |
| 20000121 | eMSI/Boot | eMMC device identification may fail during boot |
| 20000123 | Boot | Boot failure with ignore block in page mode |
| 20000124 | DMC/Boot | DMC init routine not usable in boot ROM |
| 20000126 | eMSI | Incorrect EMSI_CAP2 register bit fields |
| 20000128 | FIR/IIR Accel | Core write to IRPTL can clear pending accelerator interrupts |

---

## 6. Board & Tools Documentation

| File | Lines | Content |
|------|-------|---------|
| `ev-sc598-som_manual.md` | 697 | EV-SC598-SOM evaluation board manual (Rev 1.0, July 2021) |
| `ev-sc598-som-schematic.md` | 11,004 | Full schematic for EV-SC598-SOM board |
| `mb_ice1500_emu_ug.md` | 408 | ICE-1500 JTAG Emulator User Guide (Rev 1.0, Nov 2024) |
| `bp-400-3.md` | 4 | 400-ball BGA package drawing reference |

---

## 7. SVD Register Descriptions

**Files:** `ADSP-SC598.svd` (522,441 lines, 22 MB) and `ADSP-SC598W.svd` (528,847 lines, 22 MB)

ARM CMSIS-SVD (System View Description) files providing machine-readable peripheral register maps. Version 3.0.2.412, dated May 2025.

| File | Device | Peripherals | Description |
|------|--------|-------------|-------------|
| `ADSP-SC598.svd` | ADSP-SC598 | 93 | Standard SC598 register definitions |
| `ADSP-SC598W.svd` | ADSP-SC598W | 95 | SC598W variant register definitions |

Each peripheral entry includes:
- Base address and address block size
- Register names, offsets, sizes, and reset values
- Bit field definitions with offsets, widths, access types, and value constraints

**When to use:** These files are very large (500K+ lines). Use targeted searches (grep for peripheral or register names) rather than reading entire files. Prefer the HRM register chapters for narrative documentation; use SVD files for exact bit field definitions, reset values, and address calculations.

---

## Cross-Reference: SC596 vs SC598

| Feature | SC596 | SC598 |
|---------|-------|-------|
| SHARC+ cores | 1 | 2 |
| SHARC+ clock | 1000 MHz | 812.5-1000 MHz |
| ARM Cortex-A55 | 1200 MHz | 1200 MHz |
| L1 SRAM per core | 1 MB | 1 MB (x2) |
| L2 shared SRAM | 2 MB | 2 MB |
| DDR3/DDR3L | Yes (16-bit) | Yes (16-bit) |
| FIR accelerators | 1 | 2 |
| IIR accelerators | 4 | 8 |
| SPORTs | 8 | 8 |
| DAI blocks | 2 | 2 |
| ASRCs | 8 | 8 |
| EMACs | 2 (GbE + 10/100) | 2 (GbE + 10/100) |
| CAN FD | 2 | 2 |
| UARTs | 4 | 4 |
| SPI | 4 (incl. QSPI) | 4 (incl. QSPI) |
| OSPI | 1 | 1 |
| USB 2.0 OTG | 1 | 1 |
| TWI/I2C | 6 | 6 |
| eMSI | 1 | 1 |
| Package | 400-ball BGA_ED | 400-ball BGA_ED |

---

## Power Domains

| Domain | Supply | Purpose |
|--------|--------|---------|
| VDD_INT | Internal | Core logic, SHARC+, ARM, L2, peripherals |
| VDD_EXT | External I/O | All I/O pads |
| VDD_REF | Reference | Clock references, PLL |
| VDD_ANA | Analog | HADC, TMU |
| VDD_DMC | DDR | DDR3 controller and PHY |
| VDD_PLL | PLL | Phase-locked loops |

---

## File Path Reference

```
ADSP-SC59x-DS/adsp-sc596-adsp-sc598.md          # Data Sheet
ADSP-SC59x-HRM/000_Notices.md ... 326_*.md       # Hardware Reference (57 files)
ADSP-SC59x-PRM/sc58x-2158x-prm.md               # Programming Reference
ADSP-SC59x-EE-Notes/ee-430.md ... ee451v01.md    # EE Notes (14 files)
ADSP-SC59x-Anomaly/adsp-sc595-sc596-sc598-anomaly.md  # Anomaly List
ADSP-SC598.svd                                    # SVD Register Map (SC598)
ADSP-SC598W.svd                                   # SVD Register Map (SC598W)
ev-sc598-som_manual.md                            # EZ-KIT SOM Manual
ev-sc598-som-schematic.md                         # EZ-KIT SOM Schematic
mb_ice1500_emu_ug.md                              # ICE-1500 Emulator Guide
bp-400-3.md                                       # BGA Package Drawing
```
