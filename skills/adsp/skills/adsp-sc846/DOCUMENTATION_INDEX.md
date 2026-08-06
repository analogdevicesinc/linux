# ADSP-SC84x Documentation Index

> Navigation guide for Analog Devices ADSP-21844/21846/SC844/SC846 SHARC-FX processor documentation.
> Processors: SHARC-FX DSP core (256-bit SIMD, up to 1.2 GHz) + optional dual ARM Cortex-A55.

---

## Document Inventory

| # | Document Set | Files | Description |
|---|-------------|-------|-------------|
| 1 | [Data Sheet (DS)](#1-data-sheet-ds) | 2 | Preliminary technical data: features, pinout, memory map, peripherals overview. Prefer PrD unless comparing older PrB text. |
| 2 | [Silicon Anomaly List](#2-silicon-anomaly-list) | 1 | Known Rev 0.1 silicon anomalies and workarounds |
| 3 | [Hardware Reference Manual (HRM)](#3-hardware-reference-manual-hrm) | 46 | Complete peripheral and system block documentation |
| 4 | [SVD Register Descriptions](#4-svd-register-descriptions) | 2 | CMSIS-SVD peripheral register maps for SC84x and SC84xW |

**Total: 49 markdown files + 2 SVD files**

---

## Quick-Find by Topic

| Topic | Where to Look |
|-------|--------------|
| **Pin assignments / BGA ballmap** | DS `adsp-2184x-adsp-sc84x.md` (Table 9, lines 1192-1901) |
| **GPIO multiplexing / pin mux** | DS `adsp-2184x-adsp-sc84x.md` (Tables 9-17, lines 1192-1903); HRM `13_General-Purpose_Ports_(PORT).md` |
| **Memory map** | DS `adsp-2184x-adsp-sc84x.md` (Tables 3-6, lines 328-383) |
| **Register addresses (full list)** | HRM `46_ADSP-2184x_Register_List.md` |
| **SVD register map (SC84x)** | `ADSP-SC84x.svd` (CMSIS-SVD, 174 peripherals) |
| **SVD register map (SC84xW)** | `ADSP-SC84xW.svd` (CMSIS-SVD, 176 peripherals) |
| **Boot process / boot modes** | HRM `44_Boot_ROM_and_Booting_the_Processor.md`; DS `adsp-2184x-adsp-sc84x.md` Table 7 (lines 1016-1033); anomaly DSPSI-36/DSPSI-42 |
| **Clock/PLL configuration** | HRM `04_Clock_Generation_Unit_(CGU).md`, `05_Clock_Distribution_Unit_(CDU).md` |
| **Fractional PLL (Frac-N)** | HRM `06_Fractional_PLL_(Frac‑N_PLL).md` |
| **Power management** | HRM `07_Dynamic_Power_Management_(DPM).md` |
| **Reset control** | HRM `08_Reset_Control_Unit_(RCU).md` |
| **Interrupts (SEC)** | HRM `09_System_Event_Controller_(SEC).md` |
| **Trigger routing** | HRM `10_Trigger_Routing_Unit_(TRU).md` |
| **L2 memory** | HRM `11_L2_System_Memory.md` |
| **Memory protection (SMPU)** | HRM `12_System_Memory_Protection_Unit_(SMPU).md` |
| **GPIO / pin mux** | HRM `13_General-Purpose_Ports_(PORT).md` |
| **Thermal monitoring** | HRM `14_Thermal_Monitoring_Unit_(TMU).md` |
| **Housekeeping ADC** | HRM `15_Housekeeping_ADC_(HADC).md` |
| **CAN FD** | HRM `16_Controller_Area_Network_Flexible_Data_Rate_(CANFD).md` |
| **Watchdog** | HRM `17_Watchdog_Timer_(WDOG).md` |
| **Link port** | HRM `18_Link_Port_(LP).md` |
| **PDM microphones** | HRM `19_Pulse_Density_Modulation_(PDM)_Microphone_Interface.md` |
| **SPI** | HRM `20_Serial_Peripheral_Interface_(SPI).md`; anomaly DSPSI-26 |
| **xSPI (Octal/HyperBus)** | HRM `21_Extended_Serial_Peripheral_Interface_(xSPI).md` |
| **UART** | HRM `22_Universal_Asynchronous_ReceiverTransmitter_(UART).md` |
| **Timers** | HRM `23_General-Purpose_Timer_(TIMER).md` |
| **PWM** | HRM `24_Pulse-Width_Modulator_(PWM).md` |
| **Counters / encoders** | HRM `25_General-Purpose_Counter_(CNT).md` |
| **MediaLB / MOST** | HRM `26_Media_Local_Bus_(MLB).md` |
| **I2C / TWI** | HRM `27_Two-Wire_Interface_(TWI).md` |
| **Ethernet (EMAC)** | HRM `28_Ethernet_Media_Access_Controller_(EMAC).md` |
| **Digital Audio Interface (DAI/SRU/DRU)** | HRM `29_Digital_Audio_Interface_(DAI).md` |
| **Audio interfaces (SPORT/I2S/TDM)** | HRM `30_Serial_Port_(SPORT).md` |
| **Precision clocks (audio)** | HRM `31_Precision_Clock_Generators_(PCG).md` |
| **Audio sample rate conversion** | HRM `32_Asynchronous_Sample_Rate_Converter_(ASRC).md` |
| **S/PDIF** | HRM `33_SonyPhilips_Digital_Interface_(SPDIF).md`; anomaly DSPSI-27 |
| **DMA** | HRM `34_Direct_Memory_Access_(DMA).md`, `35_Extended_Memory_DMA_(EMDMA).md` |
| **CRC** | HRM `36_Cyclic_Redundancy_Check_(CRC).md` |
| **System protection (SPU)** | HRM `37_System_Protection_Unit_(SPU).md` |
| **Security (crypto)** | HRM `38_Security_Packet_Engine_(PKTE).md` |
| **Public key accelerator** | HRM `39_Public_Key_Accelerator_(PKA).md` |
| **Public key interrupt controller** | HRM `40_Public_Key_Interrupt_Controller_(PKIC).md` |
| **Random number generator** | HRM `41_True_Random_Number_Generator_(TRNG).md` |
| **FIR accelerator** | HRM `42_FIR_Accelerator_(FIR).md` |
| **IIR accelerator** | HRM `43_IIR_Accelerator_(IIR).md` |
| **Memory error protection** | HRM `45_Memory_Error_Protection_Unit_(MEPU).md`; anomaly DSPSI-33 |
| **SPU / SMPU** | HRM `37_System_Protection_Unit_(SPU).md`, `12_System_Memory_Protection_Unit_(SMPU).md`; anomaly DSPSI-23 |
| **USB** | DS `adsp-2184x-adsp-sc84x.md` USB overview (lines 877-883); anomaly DSPSI-41 |
| **eMSI / eMMC boot** | DS `adsp-2184x-adsp-sc84x.md` eMSI overview (lines 736-747); anomaly DSPSI-42 |
| **Power supplies / domains** | DS `adsp-2184x-adsp-sc84x.md` (Table 8, lines 1065-1088) |
| **Processor variants comparison** | DS `adsp-2184x-adsp-sc84x.md` (Table 2, lines 152-197) |
| **Package / outline dimensions** | DS `adsp-2184x-adsp-sc84x.md` (lines 2748-2754) |

---

## 1. Data Sheet (DS)

**Preferred file:** `ADSP-SC84x-DS/adsp-2184x-adsp-sc84x.md` (PrD, 2,802 lines)

**Legacy file:** `ADSP-SC84x-DS/adsp-sc84x_-PrB.md` (PrB, 1,316 lines) -- retained for comparison only.

Preliminary technical data for ADSP-21844/21846/SC844/SC846. Covers all four variants. Use the PrD file for current specifications.

| Section | Content | Lines (approx) |
|---------|---------|----------------|
| System Features | Feature overview, block diagram | 5-55 |
| Table of Contents / Revision History | TOC, revision notes | 57-118 |
| General Description | Architecture overview, SHARC-FX description | 120-150 |
| Processor Features | Variant comparison table (Table 2) | 152-197 |
| SHARC-FX Core | 4-way VLIW, 256-bit SIMD, DSP features, cache, MPU | 199-257 |
| Dual ARM Cortex-A55 | Core features, caches, GIC-600, crypto extension | 261-320 |
| System Memory Map | L1/L2/I/O/DMC memory maps (Tables 3-6) | 328-383 |
| System Infrastructure | L2 SRAM, OTP, SCBs, DMA, MDMA, CRC | 337-443 |
| Event Handling / SEC / TRU | Interrupt model, SEC, trigger routing | 465-489 |
| Security Features | TrustZone, crypto, HSM, SPU, SMPU | 497-559 |
| Safety Features | ECC, parity, CRC, signal watchdogs, MEC | 565-611 |
| Processor Peripherals | All peripheral descriptions | 615-912 |
| System Acceleration | FIR/IIR accelerators | 915-929 |
| System Design | Clock, reset, CGU, CDU, crystal, booting | 931-1033 |
| Thermal / Power | TMU, power supplies, power domains | 1043-1088 |
| Debug | JTAG, SWU, DAP, CoreSight | 1089-1138 |
| Development Tools | CCES, EZ-KIT, SigmaStudio+ | 1140-1190 |
| GPIO Multiplexing | Pin mux tables (Tables 9-17) | 1192-1903 |
| Electrical / Timing Specs | Operating conditions, clock, reset, DAI/SPORT/DDR timing | 1905-2738 |
| Outline Dimensions / Ordering | 484-ball BGA_ED package, ordering guide | 2748-2802 |

---

## 2. Silicon Anomaly List

**File:** `ADSP-SC84x-Anomaly/adsp-21844-21846-adsp-sc844-sc846-anomaly.md` (212 lines)

Silicon anomaly list for ADSP-21844/21846/SC844/SC846 Rev 0.1 (Anomaly List Rev A, 06/23/2026, Data Sheet PrD). Always check this file when working with affected subsystems.

| ID | Affected Area | Summary | Lines |
|----|---------------|---------|-------|
| DSPSI-23 | SPU/SMPU | Transactions on SPU and SMPU MMR regions may cause errors | 56-73 |
| DSPSI-25 | GP Timer | First interrupt/trigger one edge late in EXTCLK mode | 75-95 |
| DSPSI-26 | SPI | Writes to `SPI_SLVSEL` do not take effect | 103-115 |
| DSPSI-27 | S/PDIF | Receiver clock output pulse width unreliable above 96 kHz | 117-131 |
| DSPSI-33 | SHARC-FX L1 IRAM / MEPU | Correctable L1 IRAM errors may generate memory error exception | 133-150 |
| DSPSI-36 | Boot ROM | CRC protection with page mode may cause boot error | 154-166 |
| DSPSI-41 | USB/GPIO | USB controller and associated USB/GPIO pins are not functional | 168-180 |
| DSPSI-42 | eMSI/eMMC Boot | eMMC user-area boot may fail due to missing CMD7 during cleanup | 182-212 |

---

## 3. Hardware Reference Manual (HRM)

**Directory:** `ADSP-SC84x-HRM/` (46 files)

### 2.1 Front Matter
| File | Content | Lines |
|------|---------|-------|
| `01_Notices.md` | Copyright, disclaimers, trademarks | 16 |
| `02_Contents.md` | Original table of contents | 2,979 |
| `03_Preface.md` | Audience, conventions, support resources | 45 |

### 2.2 Core & System Infrastructure
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `04_Clock_Generation_Unit_(CGU).md` | 1 | PLL, clock outputs (CCLK, SYSCLK, SCLK, DCLK, OCLK) | 791 |
| `05_Clock_Distribution_Unit_(CDU).md` | 2 | Clock mux routing, output clocks | 256 |
| `06_Fractional_PLL_(Frac‑N_PLL).md` | 3 | Frac-N PLL, DPLL, audio clock generation | 796 |
| `07_Dynamic_Power_Management_(DPM).md` | 4 | Power states, peripheral clock gating | 298 |
| `08_Reset_Control_Unit_(RCU).md` | 5 | HW/SW reset, boot vectors, core handshake | 482 |
| `09_System_Event_Controller_(SEC).md` | 6 | SEC, fault management, interrupt lists | 1,312 |
| `10_Trigger_Routing_Unit_(TRU).md` | 7 | DMA-to-DMA triggers, SW triggers, synchronization | 616 |

### 2.3 Memory Subsystem
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `11_L2_System_Memory.md` | 8 | Up to 4 MB SRAM, up to 16 banks, ECC | 975 |
| `12_System_Memory_Protection_Unit_(SMPU).md` | 9 | Memory region protection, R/W/secure | 737 |

### 2.4 GPIO, Monitoring & Misc
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `13_General-Purpose_Ports_(PORT).md` | 10 | GPIO, pin mux, PINT interrupts | 3,690 |
| `14_Thermal_Monitoring_Unit_(TMU).md` | 11 | Temperature sensor, alert/fault thresholds | 441 |
| `15_Housekeeping_ADC_(HADC).md` | 12 | 12-bit ADC, up to 16 channels, 1 MSPS | 351 |
| `17_Watchdog_Timer_(WDOG).md` | 14 | 4 software watchdog timers | 193 |

### 2.5 Communication Interfaces
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `16_Controller_Area_Network_Flexible_Data_Rate_(CANFD).md` | 13 | CAN 2.0B + CAN FD, up to 64 bytes, 8 Mbps | 4,460 |
| `18_Link_Port_(LP).md` | 15 | 4-bit parallel, DDR modes, reduced pin mode | 505 |
| `19_Pulse_Density_Modulation_(PDM)_Microphone_Interface.md` | 16 | PDM inputs, decimation, I2S/TDM output | 375 |
| `20_Serial_Peripheral_Interface_(SPI).md` | 17 | SPI full-duplex, Quad SPI, flow control | 1,513 |
| `21_Extended_Serial_Peripheral_Interface_(xSPI).md` | 18 | xSPI: octal, HyperBus, DDR, XIP, PHY mode | 3,943 |
| `22_Universal_Asynchronous_ReceiverTransmitter_(UART).md` | 19 | UART, HW flow control, LIN, DMA | 1,148 |
| `25_General-Purpose_Counter_(CNT).md` | 22 | Quadrature/binary encoder, 32-bit counter | 681 |
| `26_Media_Local_Bus_(MLB).md` | 23 | MOST25/50/150, up to 64 channels | 1,280 |
| `27_Two-Wire_Interface_(TWI).md` | 24 | I2C (100K/400K), SCCB, multi-controller | 847 |

### 2.6 Ethernet
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `28_Ethernet_Media_Access_Controller_(EMAC).md` | 25 | 10/100/1000, RMII/RGMII, AVB, PTP IEEE 1588, VLAN | 13,830 |

### 2.7 Audio Subsystem
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `29_Digital_Audio_Interface_(DAI).md` | 26 | 2 DAI units, SRU matrix routing, DRU cross-routing | 3,680 |
| `30_Serial_Port_(SPORT).md` | 27 | 8 SPORTs, I2S/TDM/DSP modes, full-duplex, DMA | 2,014 |
| `31_Precision_Clock_Generators_(PCG).md` | 28 | 8 PCGs, clock + frame sync generation | 1,034 |
| `32_Asynchronous_Sample_Rate_Converter_(ASRC).md` | 29 | 16 ASRCs, 140 dB SNR, jitter cleanup | 725 |
| `33_SonyPhilips_Digital_Interface_(SPDIF).md` | 30 | S/PDIF Tx/Rx, AES3, 24-192 kHz | 993 |

### 2.8 Timers & PWM
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `23_General-Purpose_Timer_(TIMER).md` | 20 | 16 GP timers, capture, counter, watchdog modes | 1,159 |
| `24_Pulse-Width_Modulator_(PWM).md` | 21 | 3 PWM pairs, center-based, dead time, sync | 2,350 |

### 2.9 DMA
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `34_Direct_Memory_Access_(DMA).md` | 31 | Peripheral DMA, descriptor chains, scatter-gather | 1,610 |
| `35_Extended_Memory_DMA_(EMDMA).md` | 32 | Memory-to-memory EMDMA, delay line, scatter/gather | 685 |

### 2.10 Security
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `36_Cyclic_Redundancy_Check_(CRC).md` | 33 | CRC accelerator, memory scrubbing | 929 |
| `37_System_Protection_Unit_(SPU).md` | 34 | MMR write protection, secure/nonsecure | 558 |
| `38_Security_Packet_Engine_(PKTE).md` | 35 | AES/DES/SHA/HMAC/MD5 hardware acceleration | 2,784 |
| `39_Public_Key_Accelerator_(PKA).md` | 36 | RSA/ECC public key operations | 697 |
| `40_Public_Key_Interrupt_Controller_(PKIC).md` | 37 | PKA interrupt management | 231 |
| `41_True_Random_Number_Generator_(TRNG).md` | 38 | Hardware TRNG | 752 |

### 2.11 Accelerators
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `42_FIR_Accelerator_(FIR).md` | 39 | 1024-tap, 4 MACs, concurrent operation (2 instances) | 1,156 |
| `43_IIR_Accelerator_(IIR).md` | 40 | Biquad filter, 1440-word coefficient memory (4 instances, 2 with slew) | 1,271 |

### 2.12 System & Debug
| File | Chapter | Topic | Lines |
|------|---------|-------|-------|
| `44_Boot_ROM_and_Booting_the_Processor.md` | 41 | Boot modes (SPI, xSPI, eMMC, LP, UART), secure boot | 4,988 |
| `45_Memory_Error_Protection_Unit_(MEPU).md` | 42 | Error detection/reporting, parity/ECC management | 1,350 |
| `46_ADSP-2184x_Register_List.md` | 43 | Complete register address list | 44,491 |

---

## 4. SVD Register Descriptions

**Files:** `ADSP-SC84x.svd` (1,278,168 lines, 51 MB) and `ADSP-SC84xW.svd` (1,284,552 lines, 51 MB)

ARM CMSIS-SVD (System View Description) files providing machine-readable peripheral register maps.

| File | Device | Peripherals | Description |
|------|--------|-------------|-------------|
| `ADSP-SC84x.svd` | ADSP-SC84x | 174 | Standard SC84x register definitions |
| `ADSP-SC84xW.svd` | ADSP-SC84xW | 176 | SC84xW variant register definitions |

Each peripheral entry includes:
- Base address and address block size
- Register names, offsets, sizes, and reset values
- Bit field definitions with offsets, widths, access types, and value constraints

**When to use:** These files are very large (1.2M+ lines). Use targeted searches (grep for peripheral or register names) rather than reading entire files. Prefer the HRM register chapters for narrative documentation; use SVD files for exact bit field definitions, reset values, and address calculations.

---

## Cross-Reference: Processor Variants

| Feature | ADSP-21844 | ADSP-21846 | ADSP-SC844 | ADSP-SC846 |
|---------|-----------|-----------|-----------|-----------|
| Type | DSP Only | DSP Only | Enhanced Connectivity | Enhanced Connectivity |
| SHARC-FX clock (MHz) | 600, 800 | 1000, 1200 | 600, 800 | 1000, 1200 |
| ARM Cortex-A55 | N/A | N/A | Dual, 1200 MHz | Dual, 1200 MHz |
| L1 D-RAM / I-RAM (kB) | 512 / 64 | 512 / 64 | 512 / 64 | 512 / 64 |
| L1 D-Cache / I-Cache (kB) | 256 / 32 | 256 / 32 | 256 / 32 | 256 / 32 |
| A55 L2 Cache (kB) | N/A | N/A | 256 | 256 |
| A55 L3 Cache (kB) | N/A | N/A | 512 | 512 |
| L2 SRAM (kB) | 2048 | 4096 | 2048 | 4096 |
| LPDDR4 Controller | 16/32-bit | 16/32-bit | 16/32-bit | 16/32-bit |
| FIR / IIR accelerators | 2 / 4 | 2 / 4 | 2 / 4 | 2 / 4 |
| SPORTs | 8 | 8 | 8 | 8 |
| DAI blocks | 2 | 2 | 2 | 2 |
| ASRCs | 16 | 16 | 16 | 16 |
| S/PDIF Rx/Tx | 2/1 | 2/1 | 2/1 | 2/1 |
| PCGs | 8 | 8 | 8 | 8 |
| GbE EMAC (AVB + PTP) | N/A | N/A | 1 | 1 |
| CAN FD | 2 | 2 | 2 | 2 |
| UARTs | 3 | 3 | 3 | 3 |
| SPI (incl. Quad) | 4 | 4 | 4 | 4 |
| xSPI (Octal/HyperBus) | 2 | 2 | 2 | 2 |
| TWI/I2C | 6 | 6 | 6 | 6 |
| ePWM outputs | 8 | 8 | 8 | 8 |
| GP Timers | 16 | 16 | 16 | 16 |
| Link Ports | 2 | 2 | 2 | 2 |
| Package | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED |

---

## Power Domains

| Domain | Supply | Purpose |
|--------|--------|---------|
| VDD_INT | Internal | Core logic, SHARC-FX, ARM, L2, peripherals |
| VDD_EXT | External I/O | All I/O pads, SYS_CLKIN0 |
| VDD_ANA | Analog | HADC, TMU |
| VDD_DMC | DDR | LPDDR4 controller and PHY |
| VDD_PLL | PLL | PLL0/PLL1 |
| FPLLANA_VDDHV | Frac PLL | Fractional PLL analog |

---

## File Path Reference

```
ADSP-SC84x-DS/adsp-2184x-adsp-sc84x.md          # Data Sheet (Preliminary PrD, preferred)
ADSP-SC84x-DS/adsp-sc84x_-PrB.md                # Data Sheet (Preliminary PrB, legacy)
ADSP-SC84x-Anomaly/adsp-21844-21846-adsp-sc844-sc846-anomaly.md  # Silicon anomalies
ADSP-SC84x-HRM/01_Notices.md ... 46_*.md         # Hardware Reference (46 files)
ADSP-SC84x.svd                                    # SVD Register Map (SC84x)
ADSP-SC84xW.svd                                   # SVD Register Map (SC84xW)
```
