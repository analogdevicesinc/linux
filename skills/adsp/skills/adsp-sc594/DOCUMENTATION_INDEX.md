# ADSP-SC594 Documentation Index

> Comprehensive navigation guide for Analog Devices ADSP-SC592/SC594 SHARC+ processor documentation.
> Processors: Dual SHARC+ DSP cores (SC594) or single SHARC+ (SC592) + ARM Cortex-A5, up to 1 GHz, 32/40/64-bit floating-point.

---

## Document Inventory

| # | Document Set | Files | Description |
|---|-------------|-------|-------------|
| 1 | [Data Sheet (DS)](#1-data-sheet-ds) | 1 | Electrical specs, pinout, timing, features overview |
| 2 | [Hardware Reference Manual (HRM)](#2-hardware-reference-manual-hrm) | 59 | Complete peripheral and system block documentation |
| 3 | [Programming Reference Manual (PRM)](#3-programming-reference-manual-prm) | 1 | SHARC+ core ISA, registers, computation units |
| 4 | [Engineer-to-Engineer Notes (EE)](#4-engineer-to-engineer-notes-ee) | 11 | Application notes: boot, power, board design, optimization |
| 5 | [Silicon Anomaly List](#5-silicon-anomaly-list) | 1 | 18 known errata for Rev 0.0 silicon |
| 6 | [Board & Tools](#6-board--tools-documentation) | 3 | EZ-KIT SOM manual, schematic, emulator guide |

**Total: 76 markdown files**

---

## Quick-Find by Topic

| Topic | Where to Look |
|-------|--------------|
| **Pin assignments / BGA ballmap** | DS `adsp-21593-21594-adsp-sc592-sc594.md` (lines 4607+, HPC & LPC BGA) |
| **Electrical specifications / timing** | DS `adsp-21593-21594-adsp-sc592-sc594.md` (lines 2860-4607) |
| **Register addresses (full list)** | HRM `058_ADSP-2159x_SC592_SC594_Register_List_pages.md` |
| **SHARC+ instruction set** | PRM `sc58x-2158x-prm.md` (Chapters 13-27) |
| **SHARC+ register definitions** | PRM `sc58x-2158x-prm.md` (Chapters 29-35) |
| **Boot process / boot modes** | HRM `053_Boot_ROM_and_Booting_the_Processor.md` |
| **Boot time estimation** | EE `ee432.md` |
| **Boot tips & tricks (ROM API, OTP)** | EE `ee447v01.md` |
| **Clock/PLL configuration** | HRM `005_Clock_Generation_Unit_CGU.md`, `006_Clock_Distribution_Unit_CDU.md` |
| **DDR3 memory controller** | HRM `012_Dynamic_Memory_Controller_DMC.md` |
| **DDR3 board layout** | EE `ee434.md` |
| **Power estimation** | EE `ee433.md` |
| **Thermal design** | EE `ee449v01.md` |
| **TMU peak temperature estimation** | EE `ee458v01.md` |
| **DMA** | HRM `042_Direct_Memory_Access_DMA.md`, `043_Extended_Memory_DMA_EMDMA.md` |
| **Audio interfaces (SPORT/I2S/TDM)** | HRM `038_Serial_Port_SPORT.md` |
| **Audio signal routing (DAI/SRU)** | HRM `037_Digital_Audio_Interface_DAI.md` |
| **Audio sample rate conversion** | HRM `040_Asynchronous_Sample_Rate_Converter_ASRC.md` |
| **S/PDIF** | HRM `041_SonyPhilips_Digital_Interface_SPDIF.md` |
| **PDM microphones** | HRM `021_Pulse_Density_Modulation_PDM_Microphone_Interface.md` |
| **Precision clocks (audio)** | HRM `039_Precision_Clock_Generators_PCG.md` |
| **Ethernet (EMAC)** | HRM `031`-`036` (6 files covering features, programming, registers) |
| **CAN FD** | HRM `018_Controller_Area_Network_Flexible_Data_Rate_CANFD.md` |
| **SPI** | HRM `023_Serial_Peripheral_Interface_SPI.md` |
| **OSPI (Octal SPI)** | HRM `024_Octal_Serial_Peripheral_Interface_OSPI.md` |
| **OSPI PHY training** | EE `ee-437.md` |
| **UART** | HRM `025_Universal_Asynchronous_ReceiverTransmitter_UART.md` |
| **I2C / TWI** | HRM `030_Two-Wire_Interface_TWI.md` |
| **USB 2.0 OTG** | HRM `022_Universal_Serial_Bus_Controller_USBC.md` |
| **GPIO / pin mux** | HRM `015_General-Purpose_Ports_PORT.md` |
| **Timers / PWM** | HRM `027_General-Purpose_Timer_TIMER.md` |
| **Watchdog** | HRM `019_Watchdog_Timer_WDOG.md` |
| **Counters / encoders** | HRM `028_General-Purpose_Counter_CNT.md` |
| **Parallel interface (EPPI)** | HRM `026_Enhanced_Parallel_Peripheral_Interface_EPPI.md` |
| **Link port** | HRM `020_Link_Port_LP.md` |
| **MediaLB / MOST** | HRM `029_Media_Local_Bus_MLB.md` |
| **FIR accelerator** | HRM `051_FIR_Accelerator_FIR.md` |
| **IIR accelerator** | HRM `052_IIR_Accelerator_IIR.md` |
| **FIR/IIR usage & performance** | EE `ee436v02.md` |
| **Interrupts (SEC/GIC)** | HRM `009_System_Event_Controller_SEC_and_Generic_Interrupt_Controller.md` |
| **Trigger routing** | HRM `010_Trigger_Routing_Unit_TRU.md` |
| **L2 memory** | HRM `011_L2_System_Memory.md` |
| **OTP memory** | HRM `013_One-Time_Programmable_Memory_Controller_OTPC.md` |
| **Memory protection** | HRM `014_System_Memory_Protection_Unit_SMPU.md` |
| **Security (crypto, keys, TRNG)** | HRM `045`-`050` (6 files) |
| **System protection (SPU)** | HRM `046_System_Protection_Unit_SPU.md` |
| **System crossbars / bus** | HRM `054_System_Crossbars_SCB.md` |
| **System optimization** | EE `ee445v01.md` |
| **CRC** | HRM `044_Cyclic_Redundancy_Check_CRC.md` |
| **Power management** | HRM `007_Dynamic_Power_Management_DPM.md` |
| **Reset control** | HRM `008_Reset_Control_Unit_RCU.md` |
| **Safe power-down** | EE `ee448v01.md` |
| **Thermal monitoring** | HRM `016_Thermal_Monitoring_Unit_TMU.md` |
| **Housekeeping ADC** | HRM `017_Housekeeping_ADC_HADC.md` |
| **Debug / trace** | HRM `057_System_Debug_and_Trace_Unit_DBG.md` |
| **Watchpoint unit** | HRM `055_System_Watchpoint_Unit_SWU.md` |
| **Memory error protection** | HRM `056_Memory_Error_Protection_Unit_MEPU.md` |
| **ARM Cortex-A5 subsystem** | HRM `004_Arm_Cortex-A5_Subsystem.md` |
| **System overview / block diagram** | HRM `003_Introduction.md` |
| **Migration from ADSP-2156x** | EE `ee-430.md` |
| **Silicon errata / anomalies** | Anomaly `adsp-21591_21593_21594_adsp-sc591_sc592_sc594-anomaly.md` |
| **EZ-KIT SOM board** | `ev-sc594-som_manual.md`, `ev-sc594-som-schematic.md` |
| **ICE-1500 emulator** | `mb_ice1500_emu_ug.md` |

---

## 1. Data Sheet (DS)

**File:** `ADSP-SC594-DS/adsp-21593-21594-adsp-sc592-sc594.md` (5,278 lines)

Covers four processors in one combined datasheet: ADSP-21593, ADSP-21594 (no ARM), ADSP-SC592 (single SHARC+), and ADSP-SC594 (dual SHARC+). All share the 400-ball BGA_ED package (HPC variant for SC592/SC594/21594, LPC variant for 21593).

| Section | Content | Lines (approx) |
|---------|---------|----------------|
| System Features | Feature overview, processor comparison table (Table 1) | 1-220 |
| ARM Cortex-A5 | Core features, GIC (PL390), L2 cache (PL310) | 221-265 |
| SHARC+ Core | Architecture, SIMD, DAGs, BTB, ISA/VISA | 265-435 |
| System Infrastructure | Memory map, L2, OTP, crossbars, DMA, SEC, TRU | 434-625 |
| Security | TrustZone, crypto accelerators, SPU, SMPU | 621-730 |
| Safety | Parity on L1, ECC on L2, CRC, watchdogs, MEC | 677-730 |
| Peripherals | All peripheral descriptions (see Quick-Find) | 730-1030 |
| System Design | Clocking, reset, power domains, booting, JTAG | 1028-1277 |
| Signal Descriptions (HPC BGA) | Pin-by-pin signal tables, GPIO mux tables | 1277-2117 |
| Signal Descriptions (LPC BGA) | 400-ball LPC BGA pin tables | 2117-2860 |
| Electrical Specs | Operating conditions, currents, power dissipation | 2860-3256 |
| Timing Specs | All interface timing parameters | 3256-4607 |
| Output Drive | Drive strength tables, test conditions | 4447-4567 |
| Package | HPC/LPC BGA ball assignments, outline, ordering guide | 4607-5278 |

---

## 2. Hardware Reference Manual (HRM)

**Directory:** `ADSP-SC594-HRM/` (59 files)

### 2.1 Front Matter
| File | Content |
|------|---------|
| `000_Notices.md` | Copyright, disclaimers, trademarks |
| `001_Contents.md` | Original table of contents with page references |
| `002_Preface.md` | Audience, conventions, support resources (Rev 0.7, 2024) |

### 2.2 Core & System Infrastructure
| File | Chapter | Topic |
|------|---------|-------|
| `003_Introduction.md` | 1 | System-level overview, functional block diagram for SC594 |
| `004_Arm_Cortex-A5_Subsystem.md` | 2 | ARM Cortex-A5 (Armv7): FPU, NEON, MMU, L1/L2 cache, GIC |
| `005_Clock_Generation_Unit_CGU.md` | 3 | PLL, clock outputs (CCLK, SYSCLK, SCLK, DCLK, OCLK) |
| `006_Clock_Distribution_Unit_CDU.md` | 4 | Clock mux routing from CGU sources to on-chip destinations |
| `007_Dynamic_Power_Management_DPM.md` | 5 | Power states, peripheral clock gating |
| `008_Reset_Control_Unit_RCU.md` | 6 | HW/SW reset, boot vectors, core handshake |
| `009_System_Event_Controller_SEC_and_Generic_Interrupt_Controller.md` | 7 | SEC (SHARC+, 256 priority levels), GIC (ARM), fault mgmt |
| `010_Trigger_Routing_Unit_TRU.md` | 8 | DMA-to-DMA triggers, SW triggers, synchronization |

### 2.3 Memory Subsystem
| File | Chapter | Topic |
|------|---------|-------|
| `011_L2_System_Memory.md` | 9 | 2 MB SRAM, 8 banks, ECC, 192 KB boot ROM |
| `012_Dynamic_Memory_Controller_DMC.md` | 10 | DDR3/DDR3L, 16-bit interface, read/write leveling |
| `013_One-Time_Programmable_Memory_Controller_OTPC.md` | 11 | OTP with Hamming ECC, secure keys, boot config |
| `014_System_Memory_Protection_Unit_SMPU.md` | 12 | Region-based R/W/secure protection for L2, DDR, OTP, SPI |

### 2.4 GPIO, Monitoring & Misc
| File | Chapter | Topic |
|------|---------|-------|
| `015_General-Purpose_Ports_PORT.md` | 13 | GPIO, pin mux (PORT_FER/PORT_MUX), PINT interrupts |
| `016_Thermal_Monitoring_Unit_TMU.md` | 14 | 12-bit temp sensor, alert/fault thresholds |
| `017_Housekeeping_ADC_HADC.md` | 15 | 12-bit SAR ADC, 8 channels, 1 MSPS, auto-scan |
| `019_Watchdog_Timer_WDOG.md` | 17 | 32-bit WDT, countdown, SEC interrupt or HW reset |

### 2.5 Communication Interfaces
| File | Chapter | Topic |
|------|---------|-------|
| `018_Controller_Area_Network_Flexible_Data_Rate_CANFD.md` | 16 | CAN 2.0B + CAN FD, 28 mailboxes, 64-byte payloads, 8 Mbps |
| `020_Link_Port_LP.md` | 18 | 8-bit parallel, point-to-point, DDR/non-DDR modes |
| `022_Universal_Serial_Bus_Controller_USBC.md` | 20 | USB 2.0 OTG, HS/FS/LS, scatter/gather DMA, ULPI |
| `023_Serial_Peripheral_Interface_SPI.md` | 21 | SPI full-duplex, Quad mode, memory-mapped (SPI2) |
| `024_Octal_Serial_Peripheral_Interface_OSPI.md` | 22 | OSPI: 1/2/4/8-bit, STR/DTR, XIP, STIG, PHY mode |
| `025_Universal_Asynchronous_ReceiverTransmitter_UART.md` | 23 | UART, IrDA SIR, LIN break, HW flow control, DMA |
| `026_Enhanced_Parallel_Peripheral_Interface_EPPI.md` | 24 | 8-24 bit parallel, ITU-656, SMPTE, TFT LCD |
| `029_Media_Local_Bus_MLB.md` | 27 | MOST25/50/150, 64 channels, LVDS |
| `030_Two-Wire_Interface_TWI.md` | 28 | I2C (100K/400K), SCCB camera bus support |

### 2.6 Ethernet (6 files)
| File | Chapter | Topic |
|------|---------|-------|
| `031_..._EMAC_Features.md` | 29a | 10/100/1000, RMII/RGMII, VLAN, PTP, AVB |
| `032_..._Functional_Descri.md` | 29b | Tx/Rx paths, MTL, DMA descriptors, AV channels |
| `033_..._Event_Control.md` | 29c | LPI (EEE) timers, interrupt routing via SEC |
| `034_..._Programming_Model.md` | 29d | Init sequence, descriptor ring setup, PHYINT GPIO |
| `035_..._Programming_Conce.md` | 29e | Frame structure, PPS output, IEEE 802.3 statistics |
| `036_..._ADSP-2159x_SC592_SC594.md` | 29f | SC592/SC594-specific EMAC register address map and bit-fields |

### 2.7 Audio Subsystem
| File | Chapter | Topic |
|------|---------|-------|
| `021_Pulse_Density_Modulation_PDM_Microphone_Interface.md` | 19 | 4 PDM inputs, decimation, I2S/TDM output, 4-192 kHz |
| `037_Digital_Audio_Interface_DAI.md` | 30 | SRU-based signal routing between SPORT, PCG, ASRC, SPDIF, DAI pins |
| `038_Serial_Port_SPORT.md` | 31 | 8 SPORTs (4 per DAI), I2S/TDM/DSP modes, full-duplex, DMA |
| `039_Precision_Clock_Generators_PCG.md` | 32 | 4 PCGs per DAI, bit clock + frame sync generation |
| `040_Asynchronous_Sample_Rate_Converter_ASRC.md` | 33 | 4 stereo ASRCs per DAI, 128-140 dB SNR, up to 192 kHz |
| `041_SonyPhilips_Digital_Interface_SPDIF.md` | 34 | S/PDIF Tx/Rx, AES3, DTS detection, on-chip PLL |

### 2.8 DMA
| File | Chapter | Topic |
|------|---------|-------|
| `042_Direct_Memory_Access_DMA.md` | 35 | Peripheral DMA, descriptor chains, 1D/2D/3D, triggers |
| `043_Extended_Memory_DMA_EMDMA.md` | 36 | Memory-to-memory EMDMA, delay lines, scatter-gather |

### 2.9 Timers
| File | Chapter | Topic |
|------|---------|-------|
| `027_General-Purpose_Timer_TIMER.md` | 25 | PWM, capture, external event, windowed WDT, autobaud |
| `028_General-Purpose_Counter_CNT.md` | 26 | Quadrature/binary encoder, boundary compare |

### 2.10 Security
| File | Chapter | Topic |
|------|---------|-------|
| `044_Cyclic_Redundancy_Check_CRC.md` | 37 | CRC32 accelerator, DMA-coupled memory scan/fill |
| `045_System_Security.md` | 38 | Secure boot, TrustZone, OTP, SPU, SMPU, crypto overview |
| `046_System_Protection_Unit_SPU.md` | 39 | MMR write protection, security privilege enforcement |
| `047_Security_Packet_Engine_PKTE.md` | 40 | AES/ARC4/SHA/HMAC hardware crypto offload |
| `048_Public_Key_Accelerator_PKA.md` | 41 | RSA/ECC public key operations, modular exponentiation |
| `049_Public_Key_Interrupt_Controller_PKIC.md` | 42 | PKA/TRNG interrupt aggregation |
| `050_True_Random_Number_Generator_TRNG.md` | 43 | Hardware TRNG, FRO oscillators, ANSI X9.31 post-processing |

### 2.11 Accelerators
| File | Chapter | Topic |
|------|---------|-------|
| `051_FIR_Accelerator_FIR.md` | 44 | 1024-tap, 4 MACs, multi-rate decimation/interpolation |
| `052_IIR_Accelerator_IIR.md` | 45 | Biquad filter, up to 64 cascaded stages, 32-ch TDM, ACM mode |

### 2.12 System & Debug
| File | Chapter | Topic |
|------|---------|-------|
| `053_Boot_ROM_and_Booting_the_Processor.md` | 46 | Boot modes (SPI, OSPI, LP, UART), secure boot, OTP |
| `054_System_Crossbars_SCB.md` | 47 | Bus fabric, QoS arbitration |
| `055_System_Watchpoint_Unit_SWU.md` | 48 | Transaction monitoring, 4 match groups per SWU |
| `056_Memory_Error_Protection_Unit_MEPU.md` | 49 | Centralized ECC/parity error collection (MEC), parity control |
| `057_System_Debug_and_Trace_Unit_DBG.md` | 50 | CoreSight JTAG, ETM, STM, TMC, TPIU, cross-trigger |
| `058_ADSP-2159x_SC592_SC594_Register_List_pages.md` | App. A | Complete MMR address list |

---

## 3. Programming Reference Manual (PRM)

**File:** `ADSP-SC594-PRM/sc58x-2158x-prm.md` (22,333 lines)

SHARC+ Core Programming Reference, Rev 1.5 (June 2023). Covers ADSP-SC5xx and ADSP-215xx — the same PRM shared across the SC594 and SC59x families.

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

**Directory:** `ADSP-SC594-EE-Notes/` (11 files)

### By Topic Area

#### Migration
| File | Title | Lines |
|------|-------|-------|
| `ee-430.md` | Migrating from ADSP-2156x to ADSP-SC59x | 193 |

#### Boot
| File | Title | Lines |
|------|-------|-------|
| `ee432.md` | Boot Time Estimation for SC59x/2159x (SPI, OSPI, LP, UART) | 507 |
| `ee447v01.md` | Boot Tips & Tricks: Boot Stream Generation, OTP, ROM API, Secure Boot | 648 |

#### Power & Thermal
| File | Title | Lines |
|------|-------|-------|
| `ee433.md` | Power Estimation for ADSP-2159x/SC59x (voltage, frequency, temperature) | 876 |
| `ee448v01.md` | Safe Power-down Mechanisms (failure detection, mitigations) | 287 |
| `ee449v01.md` | Thermal Guidelines for ADSP-21593/21594/SC592/SC594 (junction temp, dissipation) | 204 |
| `ee458v01.md` | Estimating Peak Junction Temperature Using TMU Sensors (calibration, MAXTEMP) | 76 |

#### Board Design (DDR3 / DMC)
| File | Title | Lines |
|------|-------|-------|
| `ee434.md` | DMC Board Design Guidelines for SC59x/2159x (trace routing, impedance) | 311 |

#### Accelerators & Performance
| File | Title | Lines |
|------|-------|-------|
| `ee436v02.md` | FIR/IIR Accelerator Usage & Performance (up to 20 GFLOPs, programming) | 928 |
| `ee445v01.md` | System Optimization (bus architecture, crossbars, DMA, bandwidth) | 898 |

#### Interfaces
| File | Title | Lines |
|------|-------|-------|
| `ee-437.md` | OSPI PHY Configuration and Training (DQS/non-DQS, SSLD driver) | 236 |

---

## 5. Silicon Anomaly List

**File:** `ADSP-SC594-Anomaly/adsp-21591_21593_21594_adsp-sc591_sc592_sc594-anomaly.md` (500 lines)

18 anomalies for Rev 0.0 silicon (Revision F, April 2024), covering ADSP-21591/21593/21594 and ADSP-SC591/SC592/SC594:

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
| 20000110 | Boot/Security | Secure image auth bypassed during autodetect on POR when locked |
| 20000113 | Boot/UART | UART3 slave boot not functional with autobaud enabled |
| 20000114 | FIR Accel | Circular buffering fails in burst-16 mode |
| 20000115 | LPC/SYS_FAULT | SYS_FAULT signals active out of reset on LPC parts |
| 20000117 | DMC | PHY calibration issue (workaround provided) |
| 20000118 | FIR Accel | Wrong output for tap length > 1024 in multi-iteration |
| 20000123 | Boot | Boot failure with ignore block in page mode |
| 20000124 | DMC/Boot | DMC init routine not usable in boot ROM |
| 20000128 | FIR/IIR Accel | Core write to IRPTL can clear pending accelerator interrupts |
| 20000129 | OTP | OTP Program API fails with leaky bit |

---

## 6. Board & Tools Documentation

| File | Lines | Content |
|------|-------|---------|
| `ev-sc594-som_manual.md` | 720 | EV-SC594-SOM evaluation board manual (Rev 1.0, April 2021) |
| `ev-sc594-som-schematic.md` | 5,339 | Full schematic for EV-SC594-SOM board |
| `mb_ice1500_emu_ug.md` | 408 | ICE-1500 JTAG Emulator User Guide (Rev 1.0, Nov 2024) |

---

## Cross-Reference: SC592 vs SC594

| Feature | ADSP-SC592 | ADSP-SC594 |
|---------|-----------|-----------|
| SHARC+ cores | 1 | 2 |
| SHARC+ clock | up to 1000 MHz | up to 800/1000 MHz |
| ARM Cortex-A5 | up to 1000 MHz | up to 800/1000 MHz |
| L1 SRAM per core | 640 kB | 640 kB (x2) |
| ARM L1 cache (I/D) | 32/32 kB | 32/32 kB |
| ARM L2 cache | 256 kB | 256 kB |
| L2 shared SRAM | 2 MB | 2 MB |
| DDR3/DDR3L | Yes (16-bit) | Yes (16-bit) |
| SPORTs | 8 (4 per DAI) | 8 (4 per DAI) |
| DAI blocks | 2 | 2 |
| ASRCs | 8 (4 per DAI) | 8 (4 per DAI) |
| EMACs | 1 (10/100/1000 AVB) | 1 (10/100/1000 AVB) |
| CAN FD | 2 | 2 |
| UARTs | 4 | 4 |
| SPI | 2 (Dual-data) | 2 (Dual-data) |
| OSPI | 1 | 1 |
| USB 2.0 OTG | 1 | 1 |
| TWI/I2C | 6 | 6 |
| HADC channels | 8 | 8 |
| GPIO Ports | Port A-I | Port A-I |
| Package | 400-ball HPC BGA_ED | 400-ball HPC BGA_ED |

---

## Key Differences from ADSP-SC59x (SC595/SC596/SC598)

| Feature | ADSP-SC592/SC594 | ADSP-SC595/SC596/SC598 |
|---------|-----------------|----------------------|
| ARM core | Cortex-A5 (Armv7) | Cortex-A55 (Armv8) |
| Audio interface naming | DAI (Digital Audio Interface) | SHAI (SHARC Audio Interface) |
| HRM Introduction chapter | `003_Introduction.md` (system overview) | `003_Arm_Cortex-A55_Subsystem.md` (direct to A55) |
| eMSI (eMMC/SD) | Not present | Yes (eMMC 5.1 / SD) |
| DDR prefetch buffer | Not present | Yes (`012_DDR_Pre-fetch_Buffer_DDRPFB.md`) |
| EMAC count | 1 | 2 (GbE + 10/100) |
| Package variants | HPC BGA + LPC BGA (21593) | Single BGA_ED |

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

All paths relative to `/home/michael/skills/adsp_sc594/`:

```
ADSP-SC594-DS/adsp-21593-21594-adsp-sc592-sc594.md          # Data Sheet
ADSP-SC594-HRM/000_Notices.md ... 058_*.md                   # Hardware Reference (59 files)
ADSP-SC594-PRM/sc58x-2158x-prm.md                           # Programming Reference
ADSP-SC594-EE-Notes/ee-430.md ... ee458v01.md                # EE Notes (11 files)
ADSP-SC594-Anomaly/adsp-21591_21593_21594_adsp-sc591_sc592_sc594-anomaly.md  # Anomaly List
ev-sc594-som_manual.md                                        # EZ-KIT SOM Manual
ev-sc594-som-schematic.md                                     # EZ-KIT SOM Schematic
mb_ice1500_emu_ug.md                                          # ICE-1500 Emulator Guide
```
