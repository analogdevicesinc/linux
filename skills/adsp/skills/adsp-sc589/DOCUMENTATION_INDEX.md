# ADSP-SC589 Documentation Index

> Comprehensive navigation guide for Analog Devices ADSP-SC582/SC583/SC584/SC587/SC589 SHARC+ processor documentation (also covers ADSP-21583/21584/21587 no-ARM variants).
> Processors: Dual SHARC+ DSP cores (SC587/SC589) or single SHARC+ (SC582/SC584) + ARM Cortex-A5 (SC58x), up to 500 MHz, 32/40/64-bit floating-point.

---

## Document Inventory

| # | Document Set | Files | Description |
|---|-------------|-------|-------------|
| 1 | [Data Sheet (DS)](#1-data-sheet-ds) | 1 | Electrical specs, pinout, timing, features overview for all SC58x/2158x |
| 2 | [Hardware Reference Manual (HRM)](#2-hardware-reference-manual-hrm) | 60 | Complete peripheral and system block documentation |
| 3 | [Engineer-to-Engineer Notes (EE)](#3-engineer-to-engineer-notes-ee) | 5 | Application notes: migration, power, cache, dual-SHARC, Linux loader |
| 4 | [Silicon Anomaly List](#4-silicon-anomaly-list) | 1 | 77 known errata for Rev 0.1/1.0/1.2 silicon |

**Total: 67 markdown files**

---

## Quick-Find by Topic

| Topic | Where to Look |
|-------|--------------|
| **Pin assignments / BGA ballmap** | DS `adsp-sc582_583_584_587_589_adsp-21583_584_587.md` (lines 3700+, 349-ball and 529-ball BGA) |
| **Electrical specifications / timing** | DS (lines 2800-3700, electrical; lines 3700-5200, timing) |
| **Register addresses (full list)** | HRM `60_ADSP-SC58x_Register_List.md` (9,917 lines) |
| **SHARC+ instruction set / ISA** | Use `adsp-sc594` skill: `ADSP-SC594-PRM/sc58x-2158x-prm.md` (covers SC58x family) |
| **Boot process / boot modes** | HRM `56_Boot_ROM_and_Booting_the_Processor.md` |
| **Clock/PLL configuration** | HRM `06_Clock_Generation_Unit_(CGU).md`, `07_Clock_Distribution_Unit_(CDU).md` |
| **DDR3 memory controller** | HRM `13_Dynamic_Memory_Controller_(DMC).md` |
| **Static memory controller** | HRM `14_Static_Memory_Controller_(SMC).md` |
| **Power management** | HRM `08_Dynamic_Power_Management_(DPM).md` |
| **Power estimation** | EE `EE392v01.md` |
| **Reset control** | HRM `09_Reset_Control_Unit_(RCU).md` |
| **DMA** | HRM `41_Direct_Memory_Access_(DMA).md`, `42_Extended_Memory_DMA_(EMDMA).md` |
| **Audio interfaces (SPORT/I2S/TDM)** | HRM `37_Serial_Port_(SPORT).md` |
| **Audio signal routing (DAI/SRU)** | HRM `36_Digital_Audio_Interface_(DAI).md` |
| **Audio sample rate conversion** | HRM `39_Asynchronous_Sample_Rate_Converter_(ASRC).md` |
| **S/PDIF** | HRM `40_SonyPhilips_Digital_Interface_(SPDIF).md` |
| **Precision clocks (audio)** | HRM `38_Precision_Clock_Generators_(PCG).md` |
| **SINC filter** | HRM `35_Sinus_Cardinalis_(SINC)_Filter.md` |
| **Ethernet (EMAC)** | HRM `34_Ethernet_Media_Access_Controller_(EMAC).md` |
| **PCIe (SC589 only)** | HRM `32_PCI_Express.md` |
| **CAN** | HRM `28_Controller_Area_Network_(CAN).md` |
| **SPI** | HRM `19_Serial_Peripheral_Interface_(SPI).md` |
| **UART** | HRM `20_Universal_Asynchronous_ReceiverTransmitter_(UART).md` |
| **I2C / TWI** | HRM `33_Two-Wire_Interface_(TWI).md` |
| **USB 2.0** | HRM `30_Universal_Serial_Bus_(USB).md` |
| **eMMC / mobile storage (MSI)** | HRM `29_Mobile_Storage_Interface_(MSI).md` |
| **MLB / MOST** | HRM `31_Media_Local_Bus_(MLB).md` |
| **GPIO / pin mux** | HRM `17_General-Purpose_Ports_(PORT).md` |
| **Timers / PWM** | HRM `22_Pulse-Width_Modulator_(PWM).md`, `23_General-Purpose_Timer_(TIMER).md` |
| **Watchdog** | HRM `24_Watchdog_Timer_(WDOG).md` |
| **Real-time clock** | HRM `25_Real_Time_Clock_(RTC).md` |
| **Counters / encoders** | HRM `26_General-Purpose_Counter_(CNT).md` |
| **Parallel interface (EPPI)** | HRM `21_Enhanced_Parallel_Peripheral_Interface_(EPPI).md` |
| **Link port** | HRM `18_Link_Port_(LP).md` |
| **ADC control module** | HRM `27_ADC_Control_Module_(ACM).md` |
| **FIR accelerator** | HRM `54_FIR_Accelerator_(FIR).md` |
| **IIR accelerator** | HRM `55_IIR_Accelerator_(IIR).md` |
| **FFT accelerator** | HRM `53_FFT_Accelerator_(FFTA).md` |
| **Harmonic Analysis Engine (HAE)** | HRM `52_Harmonic_Analysis_Engine_(HAE).md` |
| **Cache usage** | EE `EE-400.md` |
| **Interrupts (SEC/GIC)** | HRM `10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC).md` |
| **Trigger routing** | HRM `11_Trigger_Routing_Unit_(TRU).md` |
| **L2 memory** | HRM `12_L2_System_Memory.md` |
| **OTP memory** | HRM `15_One-Time_Programmable_Memory_Controller_(OTPC).md` |
| **Memory protection** | HRM `16_System_Memory_Protection_Unit_(SMPU).md` |
| **Security (crypto, keys, TRNG)** | HRM `45_System_Security.md`, `46`-`50` (5 files) |
| **System protection (SPU)** | HRM `46_System_Protection_Unit_(SPU).md` |
| **Security packet engine** | HRM `47_Security_Packet_Engine_(PKTE).md` |
| **CRC** | HRM `43_Cyclic_Redundancy_Check_(CRC).md` |
| **Housekeeping ADC** | HRM `44_Housekeeping_ADC_(HADC).md` |
| **Thermal monitoring** | HRM `51_Thermal_Monitoring_Unit_(TMU).md` |
| **System crossbars / bus** | HRM `57_System_Crossbars_(SCB).md` |
| **Watchpoint unit** | HRM `58_System_Watchpoint_Unit_(SWU).md` |
| **Debug / trace** | HRM `59_System_Debug_and_Trace_Unit_(DBG).md` |
| **ARM Cortex-A5 subsystem** | HRM `05_ARM_Cortex-A5_Sub-System.md` |
| **System overview / block diagram** | HRM `04_Introduction.md` |
| **Migration from legacy SHARC** | EE `EE375v01.md` |
| **Dual-SHARC communication** | EE `EE377v02.md` |
| **Linux SHARC+ loader** | EE `EE399v03.md` |
| **Silicon errata / anomalies** | Anomaly `adsp-2158x_adsp-sc58x-sharc-anomaly.md` |

---

## 1. Data Sheet (DS)

**File:** `ADSP-SC589-DS/adsp-sc582_583_584_587_589_adsp-21583_584_587.md` (6,148 lines)

Covers the full SC58x/2158x family: SC582, SC583, SC584, SC587, SC589 (with ARM Cortex-A5) and ADSP-21583, 21584, 21587 (DSP-only, no ARM). Uses 349-ball (SC58x small) and 529-ball (SC589) BGA packages.

| Section | Content | Lines (approx) |
|---------|---------|----------------|
| System Features | Feature overview, processor comparison table | 1-300 |
| ARM Cortex-A5 | Core features, cache, GIC | 300-450 |
| SHARC+ Core | Architecture, SIMD, DAGs, 64-bit float | 450-700 |
| System Infrastructure | Memory map, L2, OTP, DMA, SEC/TRU, crossbars | 700-1000 |
| Security | TrustZone, SPU, SMPU, crypto, PKA, TRNG | 1000-1200 |
| Safety | Parity, ECC, CRC, watchdogs | 1150-1300 |
| Peripherals | All peripheral descriptions | 1300-1700 |
| System Design | Clocking, reset, power, booting, JTAG | 1700-2000 |
| Signal Descriptions (349-ball) | Pin tables for SC582/583/584/587, 21583/21584/21587 | 2000-2800 |
| Electrical Specs | Operating conditions, supply currents | 2800-3200 |
| Timing Specs | All interface timing parameters | 3200-4700 |
| Signal Descriptions (529-ball) | Pin tables for SC589 529-ball BGA | 4700-5400 |
| Package | BGA ball assignments, outline, ordering guide | 5400-6148 |

---

## 2. Hardware Reference Manual (HRM)

**Directory:** `ADSP-SC589-HRM/` (60 files)

### 2.1 Front Matter
| File | Content |
|------|---------|
| `01_Notices.md` | Copyright, disclaimers, trademarks |
| `02_Contents.md` | Original table of contents (3,477 lines) |
| `03_Preface.md` | Audience, conventions, support resources |

### 2.2 Core & System Infrastructure
| File | Chapter | Topic |
|------|---------|-------|
| `04_Introduction.md` | 1 | System-level overview, functional block diagram for SC589 |
| `05_ARM_Cortex-A5_Sub-System.md` | 2 | ARM Cortex-A5 (Armv7): FPU, NEON, MMU, L1/L2 cache, GIC |
| `06_Clock_Generation_Unit_(CGU).md` | 3 | PLL, clock outputs (CCLK, SYSCLK, SCLK, DCLK, OCLK) |
| `07_Clock_Distribution_Unit_(CDU).md` | 4 | Clock mux routing from CGU sources to on-chip destinations |
| `08_Dynamic_Power_Management_(DPM).md` | 5 | Power states, peripheral clock gating |
| `09_Reset_Control_Unit_(RCU).md` | 6 | HW/SW reset, boot vectors, core handshake |
| `10_System_Event_Controller_(SEC)_and_Generic_Interrupt_Controller_(GIC).md` | 7 | SEC (SHARC+), GIC (ARM), fault management (2,030 lines) |
| `11_Trigger_Routing_Unit_(TRU).md` | 8 | DMA-to-DMA triggers, SW triggers, synchronization |

### 2.3 Memory Subsystem
| File | Chapter | Topic |
|------|---------|-------|
| `12_L2_System_Memory.md` | 9 | L2 SRAM, ECC, boot ROM |
| `13_Dynamic_Memory_Controller_(DMC).md` | 10 | DDR3/DDR2/LPDDR, read/write leveling |
| `14_Static_Memory_Controller_(SMC).md` | 11 | External static memory, async SRAM, NOR flash |
| `15_One-Time_Programmable_Memory_Controller_(OTPC).md` | 12 | OTP with Hamming ECC, secure keys, boot config |
| `16_System_Memory_Protection_Unit_(SMPU).md` | 13 | Region-based R/W/secure protection |

### 2.4 GPIO & Interfaces
| File | Chapter | Topic |
|------|---------|-------|
| `17_General-Purpose_Ports_(PORT).md` | 14 | GPIO, pin mux (PORT_FER/PORT_MUX), PINT interrupts (1,876 lines) |
| `18_Link_Port_(LP).md` | 15 | 8-bit parallel link port, point-to-point, DDR/non-DDR |

### 2.5 Communication Interfaces
| File | Chapter | Topic |
|------|---------|-------|
| `19_Serial_Peripheral_Interface_(SPI).md` | 16 | SPI full-duplex, Quad mode, memory-mapped (SPI2) |
| `20_Universal_Asynchronous_ReceiverTransmitter_(UART).md` | 17 | UART, IrDA, LIN break, HW flow control, DMA |
| `21_Enhanced_Parallel_Peripheral_Interface_(EPPI).md` | 18 | 8-24 bit parallel, ITU-656, SMPTE, TFT LCD (1,953 lines) |
| `22_Pulse-Width_Modulator_(PWM).md` | 19 | PWM control (2,666 lines) |
| `23_General-Purpose_Timer_(TIMER).md` | 20 | PWM/capture/external event, autobaud |
| `24_Watchdog_Timer_(WDOG).md` | 21 | 32-bit WDT, countdown, SEC interrupt or HW reset |
| `25_Real_Time_Clock_(RTC).md` | 22 | Real-time clock, alarm, stopwatch |
| `26_General-Purpose_Counter_(CNT).md` | 23 | Quadrature/binary encoder, boundary compare |
| `27_ADC_Control_Module_(ACM).md` | 24 | ADC control module |
| `28_Controller_Area_Network_(CAN).md` | 25 | CAN 2.0B, mailboxes |
| `29_Mobile_Storage_Interface_(MSI).md` | 26 | eMMC/SD interface (2,285 lines) |
| `30_Universal_Serial_Bus_(USB).md` | 27 | USB 2.0 OTG, HS/FS/LS, scatter/gather DMA, ULPI (4,029 lines) |
| `31_Media_Local_Bus_(MLB).md` | 28 | MOST25/50/150, 64 channels, LVDS |
| `32_PCI_Express.md` | 29 | PCIe interface (SC589 only, 6,074 lines) |
| `33_Two-Wire_Interface_(TWI).md` | 30 | I2C (100K/400K), SCCB camera bus |
| `34_Ethernet_Media_Access_Controller_(EMAC).md` | 31 | 10/100/1000 Ethernet EMAC (7,121 lines) |

### 2.6 Audio Subsystem
| File | Chapter | Topic |
|------|---------|-------|
| `35_Sinus_Cardinalis_(SINC)_Filter.md` | 32 | SINC decimation filter for PDM microphones |
| `36_Digital_Audio_Interface_(DAI).md` | 33 | SRU-based signal routing between SPORT, PCG, ASRC, SPDIF, DAI pins (2,362 lines) |
| `37_Serial_Port_(SPORT).md` | 34 | 8 SPORTs, I2S/TDM/DSP modes, full-duplex, DMA (1,932 lines) |
| `38_Precision_Clock_Generators_(PCG).md` | 35 | PCGs, bit clock + frame sync generation |
| `39_Asynchronous_Sample_Rate_Converter_(ASRC).md` | 36 | Stereo ASRCs, 128-140 dB SNR |
| `40_SonyPhilips_Digital_Interface_(SPDIF).md` | 37 | S/PDIF Tx/Rx, AES3, DTS detection, on-chip PLL |

### 2.7 DMA & Utility
| File | Chapter | Topic |
|------|---------|-------|
| `41_Direct_Memory_Access_(DMA).md` | 38 | Peripheral DMA, descriptor chains, 1D/2D/3D, triggers |
| `42_Extended_Memory_DMA_(EMDMA).md` | 39 | Memory-to-memory EMDMA, delay lines, scatter-gather |
| `43_Cyclic_Redundancy_Check_(CRC).md` | 40 | CRC32 accelerator, DMA-coupled |
| `44_Housekeeping_ADC_(HADC).md` | 41 | 12-bit SAR ADC housekeeping channels |

### 2.8 Security
| File | Chapter | Topic |
|------|---------|-------|
| `45_System_Security.md` | 42 | Secure boot, TrustZone, OTP, SPU, SMPU, crypto overview |
| `46_System_Protection_Unit_(SPU).md` | 43 | MMR write protection, security privilege enforcement |
| `47_Security_Packet_Engine_(PKTE).md` | 44 | AES/ARC4/SHA/HMAC hardware crypto offload (2,699 lines) |
| `48_Public_Key_Accelerator_(PKA).md` | 45 | RSA/ECC public key operations |
| `49_Public_Key_Interrupt_Controller_(PKIC).md` | 46 | PKA/TRNG interrupt aggregation |
| `50_True_Random_Number_Generator_(TRNG).md` | 47 | Hardware TRNG, FRO oscillators |

### 2.9 Accelerators
| File | Chapter | Topic |
|------|---------|-------|
| `51_Thermal_Monitoring_Unit_(TMU).md` | 48 | Thermal sensor, alert/fault thresholds |
| `52_Harmonic_Analysis_Engine_(HAE).md` | 49 | HAE spectral analysis accelerator |
| `53_FFT_Accelerator_(FFTA).md` | 50 | FFT accelerator |
| `54_FIR_Accelerator_(FIR).md` | 51 | FIR accelerator, multi-rate decimation/interpolation |
| `55_IIR_Accelerator_(IIR).md` | 52 | Biquad IIR accelerator, cascaded stages, TDM |

### 2.10 System & Debug
| File | Chapter | Topic |
|------|---------|-------|
| `56_Boot_ROM_and_Booting_the_Processor.md` | 53 | Boot modes (SPI, LP, UART, MSI, PCIe), secure boot, OTP (4,495 lines) |
| `57_System_Crossbars_(SCB).md` | 54 | Bus fabric, QoS arbitration |
| `58_System_Watchpoint_Unit_(SWU).md` | 55 | Transaction monitoring, match groups |
| `59_System_Debug_and_Trace_Unit_(DBG).md` | 56 | CoreSight JTAG, ETM, STM, cross-trigger |
| `60_ADSP-SC58x_Register_List.md` | App. A | Complete MMR address list (9,917 lines) |

---

## 3. Engineer-to-Engineer Notes (EE)

**Directory:** `ADSP-SC589-EE-Notes/` (5 files)

| File | Title | Lines |
|------|-------|-------|
| `EE375v01.md` | Migrating Legacy SHARC to ADSP-SC58x/2158x SHARC+ Processors | 623 |
| `EE377v02.md` | Using MCAPI/MDMA for ADSP-SC58x Dual-SHARC Audio Talkthrough | 272 |
| `EE392v01.md` | Estimating Power for ADSP-SC58x/2158x SHARC+ Processors | 951 |
| `EE399v03.md` | The Linux Run-Time SHARC+ Loader on the ADSP-SC57x/ADSP-SC58x Processors | 277 |
| `EE-400.md` | Using Cache on ADSP-SC5xx/ADSP-215xx SHARC+ Processors | 499 |

### By Topic

| Topic | File |
|-------|------|
| Migration from legacy SHARC (pipeline, registers, memory, multi-core) | `EE375v01.md` |
| Dual-SHARC communication (MCAPI messaging, MDMA transfers, audio talkthrough) | `EE377v02.md` |
| Power estimation (voltage, frequency, temperature, activity factors) | `EE392v01.md` |
| Linux runtime SHARC+ loader (AMP with ARM Linux host, shared memory) | `EE399v03.md` |
| Cache usage (L1 I/D cache config, coherency, locking, optimization) | `EE-400.md` |

---

## 4. Silicon Anomaly List

**File:** `ADSP-SC589-Anomaly/adsp-2158x_adsp-sc58x-sharc-anomaly.md` (1,814 lines)

77 anomalies for silicon Revisions 0.1, 1.0, and 1.2, covering ADSP-SC582/583/584/587/589 and ADSP-21583/21584/21587.

| ID | Subsystem | Affects Rev | Summary |
|----|-----------|-------------|---------|
| 20000002 | CPU Pipeline | 0.1, 1.0, 1.2 | Data forwarding Rn/Sn to DAG may fail with stalls |
| 20000003 | SPU/SMPU | 0.1, 1.0, 1.2 | SPU and SMPU MMR transactions may cause errors |
| 20000031 | GP Timer | 0.1, 1.0, 1.2 | First interrupt one edge late in EXTCLK mode |
| 20000037 | DMC | 0.1, 1.0, 1.2 | DMC read state machine may be incorrect after init |
| 20000038 | Boot/ARM | 0.1, 1.0, 1.2 | ADI_ROM_BOOT_CONFIG errorReturn incorrect for ARM-hosted boot |
| 20000039 | OTP | 0.1, 1.0, 1.2 | OTP API does not report OTP errors |
| 20000043 | Boot/Security | 0.1, 1.0, 1.2 | Key unwrapping on SHARC+ fails when using ROMAPI |
| 20000044 | Boot | 0.1, 1.0, 1.2 | Ignore blocks not supported in page mode for non-secure slave boot |
| 20000048 | CGU | 0.1, 1.0, 1.2 | CGU0 lock write error bit can be erroneously set |
| 20000050 | SPORT | 0.1, 1.0, 1.2 | SPORT erroneously drives data pins during inactive channels in multichannel mode |
| 20000051 | SPI Boot | 0.1, 1.0, 1.2 | Secure SPI master boot only supported from SPI2 |
| 20000052 | SPI Boot | 0.1, 1.0, 1.2 | SPI master boot fails when block payload > 65,532 bytes |
| 20000053 | PKTE | 0.1, 1.0, 1.2 | Reading PKTE registers may return incorrect data during packet processing |
| 20000062 | SPI | 0.1, 1.0, 1.2 | Writes to SPI_SLVSEL register do not take effect (write twice) |
| 20000067 | DMC | 0.1, 1.0, 1.2 | DMC clock may violate JEDEC timing in self-refresh mode |
| 20000069 | CPU Pipeline | 0.1, 1.0, 1.2 | PCSTK/MODE1STK loads fail if next instruction is L2/L3 access |
| 20000070 | PCIe | 0.1 | Special programming for PCIe PHY TXDEEMPH/TXSWING registers (SC589 only) |
| 20000072 | CPU Pipeline | 0.1, 1.0, 1.2 | Float computes targeting F0 register cause pipeline stalls |
| 20000073 | DMC | 1.0, 1.2 | DDR frequency limited to 300 MHz when using OTP for DMC programming |
| 20000074 | Link Port | 0.1, 1.0, 1.2 | Link port DMA peripheral interrupt not supported |
| 20000075 | Link Port | 0.1, 1.0, 1.2 | Link port cannot trigger TRU slaves with CGU1 sources |
| 20000079 | MLB | 0.1, 1.0, 1.2 | MLB at 3072x Fs and 4096x Fs is not functional |
| 20000080 | SPI Boot | 0.1, 1.0, 1.2 | Quad-SPI master boot modes not functional |
| 20000082 | ARM Cortex-A5 | 0.1, 1.0, 1.2 | Unaligned half-word reads of non-cacheable memory return incomplete data |
| 20000087 | CPU Pipeline | 0.1, 1.0, 1.2 | Computes targeting F0 register cause pipeline stalls |
| 20000090 | DMC | 0.1, 1.0, 1.2 | Single-ended clock/DQS may violate JEDEC Vix and VSWING specs |
| 20000091 | DMC | 0.1, 1.0, 1.2 | Accesses to DMC_CPHY_CTL do not function as expected |
| 20000094 | SPDIF | 1.0, 1.2 | SPDIF receiver output clock is unreliable |
| 20000096 | CPU Pipeline | 0.1, 1.0, 1.2 | Type 18a USTAT instructions fail following specific code sequences |
| 20000101 | SMPU | 0.1, 1.0, 1.2 | SMPU hang when exclusive read arrives while non-exclusive write is pending |
| 20000108 | Security | 0.1, 1.0, 1.2 | Factory serial number cannot be read when device is locked |
| 20000123 | Boot | 0.1, 1.0, 1.2 | Boot failure with ignore block in page mode |

**Note:** Only the most impactful anomalies affecting SC589 Rev 1.2 (current) are shown above. 77 anomalies total (many are Rev 0.1 only). Read the full anomaly file for all entries and detailed workarounds.

---

## SC58x Processor Variant Comparison

| Feature | SC582 | SC583 | SC584 | SC587 | SC589 | 21583/21584/21587 |
|---------|-------|-------|-------|-------|-------|-------------------|
| SHARC+ cores | 1 | 2 | 1 | 2 | 2 | 2 (no ARM) |
| ARM Cortex-A5 | Yes | Yes | Yes | Yes | Yes | No |
| PCIe | No | No | No | No | Yes | No |
| Package | 349-ball | 349-ball | 349-ball | 349-ball | 529-ball | 349-ball |

All variants include: DAI + SRU audio routing, SPORT, ASRC, S/PDIF, PCG, DDR3 DMC, DMA, Ethernet EMAC, SPI, UART, TWI/I2C, CAN, USB, MSI, EPPI, MLB, FIR/IIR accelerators, PKTE/PKA/TRNG security.

---

## Key Differences from ADSP-SC59x (SC595/SC596/SC598)

| Feature | ADSP-SC58x/SC589 | ADSP-SC595/SC596/SC598 |
|---------|-----------------|----------------------|
| ARM core | Cortex-A5 (Armv7) | Cortex-A55 (Armv8) |
| Audio naming | DAI (Digital Audio Interface) | SHAI (SHARC Audio Interface) |
| PCIe | SC589 only | Not present |
| FFT/HAE accelerators | Yes | Not present |
| SINC filter | Yes | Not present |
| eMMC/SD | MSI | eMSI |
| SHARC+ clock | up to 500 MHz | up to 1 GHz (SC598) |

---

## File Path Reference

All paths relative to the skill root:

```
ADSP-SC589-DS/adsp-sc582_583_584_587_589_adsp-21583_584_587.md   # Data Sheet
ADSP-SC589-HRM/01_Notices.md ... 60_ADSP-SC58x_Register_List.md  # HRM (60 files)
ADSP-SC589-EE-Notes/EE375v01.md ... EE-400.md                    # EE Notes (5 files)
ADSP-SC589-Anomaly/adsp-2158x_adsp-sc58x-sharc-anomaly.md        # Anomaly List
```
