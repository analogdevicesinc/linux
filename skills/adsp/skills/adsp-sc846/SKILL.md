---
name: adsp-sc846
description: >
  ADSP-SC84x (SC844/SC846/21844/21846) SHARC-FX DSP processor development assistant.
  This skill should be used when the user asks about firmware development,
  peripheral configuration, register programming, SHARC-FX vector DSP architecture,
  DMA setup, audio subsystem (DAI, SRU, DRU, SPORT, I2S, TDM, ASRC, SPDIF, PDM, PCG),
  Ethernet EMAC, SPI, xSPI, UART, I2C/TWI, CAN FD, LPDDR4 memory controller (DMC),
  clock/PLL/CGU/CDU/Frac-N PLL configuration, boot process, power management,
  pin mux/GPIO, PWM, FIR/IIR accelerators, HSM, security (TrustZone, crypto),
  debug/JTAG/CoreSight, system crossbar optimization, board design,
  EZ-KIT SOM, ARM Cortex-A55 dual-core subsystem, or any topic related to
  Analog Devices SHARC-FX SC84x processors.
  Make sure to use this skill whenever the user mentions ADSP, SC844, SC846,
  21844, 21846, SHARC-FX, 2184x, SC84x, or any SC84x peripheral by name.
---

# ADSP-SC84x Development Assistant

You have access to the documentation set for the Analog Devices ADSP-21844/21846/SC844/SC846 SHARC-FX processor family: Preliminary Data Sheet, Silicon Anomaly List, Hardware Reference Manual (46 chapters), and CMSIS-SVD register maps. All documentation is in markdown format at the project root.

**Note:** This is preliminary documentation. Prefer the PrD Data Sheet (`ADSP-SC84x-DS/adsp-2184x-adsp-sc84x.md`) for current specifications; the older PrB conversion remains available only for comparison.

## Documentation Navigation Protocol

**Step 1 -- Always start by reading the index.**
Read `DOCUMENTATION_INDEX.md` at the project root. It contains a topic-to-file mapping table that tells you exactly which file(s) and line ranges to consult for any topic.

**Step 2 -- Read targeted sections, not entire files.**
Use the Read tool with `offset` and `limit` parameters for large files:
- **Data Sheet** (`ADSP-SC84x-DS/adsp-2184x-adsp-sc84x.md`): 2,802 lines. Can be read selectively or in full for broad queries.
- **Silicon Anomaly List** (`ADSP-SC84x-Anomaly/adsp-21844-21846-adsp-sc844-sc846-anomaly.md`): 212 lines. Read this when working with affected subsystems.
- **HRM files**: Most are under 2,000 lines and can be read in full. The Ethernet chapter (28) is 13,830 lines and the Register List (46) is 44,491 lines -- read these selectively.

**Step 3 -- Cross-reference when needed.**
For any peripheral topic, consider consulting multiple sources:
- **HRM** chapter: Functional description, programming model, registers
- **Data Sheet**: Feature overview, signal descriptions, pin mux tables, power domains
- **SVD files**: Exact register bit field definitions, offsets, widths, reset values, and access types
- **Anomaly list**: Known Rev 0.1 errata and workarounds; check for SPU/SMPU, GP Timer, SPI, S/PDIF, L1 IRAM/MEPU, boot ROM, USB, and eMSI/eMMC boot topics

**SVD file usage notes:**
The SVD files (`ADSP-SC84x.svd`, `ADSP-SC84xW.svd`) are very large (1.2M+ lines each). Never read them in full. Instead, use the Grep tool to search for specific peripheral or register names (e.g., grep for `<name>SPI0</name>` or `<name>SPI_CTL</name>`), then read only the relevant section with offset/limit. Use SVD files when you need machine-precise register field layouts (bit offsets, widths, reset values) for code generation or register verification. Prefer the HRM for narrative explanations of how registers work.

## Key File Paths

| Document | Path |
|----------|------|
| Data Sheet (Preliminary PrD, preferred) | `ADSP-SC84x-DS/adsp-2184x-adsp-sc84x.md` |
| Data Sheet (Preliminary PrB, legacy) | `ADSP-SC84x-DS/adsp-sc84x_-PrB.md` |
| Silicon Anomaly List | `ADSP-SC84x-Anomaly/adsp-21844-21846-adsp-sc844-sc846-anomaly.md` |
| HRM (46 chapters) | `ADSP-SC84x-HRM/04_*.md` through `46_*.md` |
| Register Address List | `ADSP-SC84x-HRM/46_ADSP-2184x_Register_List.md` |
| SVD Register Map (SC84x) | `ADSP-SC84x.svd` (CMSIS-SVD, 1.3M lines, 174 peripherals) |
| SVD Register Map (SC84xW) | `ADSP-SC84xW.svd` (CMSIS-SVD, 1.3M lines, 176 peripherals) |
| Navigation Index | `DOCUMENTATION_INDEX.md` |

## Task-Specific Routing

### Peripheral Configuration
1. Find the HRM chapter for that peripheral (use the index)
2. Check `46_ADSP-2184x_Register_List.md` for register addresses
3. Check the Silicon Anomaly List if the topic involves SPU/SMPU, GP Timer, SPI, S/PDIF, L1 IRAM/MEPU, boot ROM, USB, or eMSI/eMMC boot
4. For exact bit field definitions, grep the SVD files (`ADSP-SC84x.svd` or `ADSP-SC84xW.svd`) for the peripheral/register name
5. Check DS pin mux tables (Tables 9-17, lines 1192-1903 in the PrD data sheet)

### Boot Configuration
- HRM: `44_Boot_ROM_and_Booting_the_Processor.md`
- DS: Boot modes table (Table 7, lines 1016-1033)
- Anomalies: DSPSI-36 (CRC protection with page mode), DSPSI-42 (eMSI/eMMC user-area boot cleanup)

### LPDDR4 / DMC Setup
- DS: DMC memory map (Table 6, lines 383-387), DMC description (lines 619-625)
- HRM: No dedicated DMC chapter in current HRM set -- use SVD for register details

### Audio Subsystem (DAI / SRU / DRU)
- HRM: `29_Digital_Audio_Interface_(DAI).md`, `30_Serial_Port_(SPORT).md`, `31_Precision_Clock_Generators_(PCG).md`, `32_Asynchronous_Sample_Rate_Converter_(ASRC).md`, `33_SonyPhilips_Digital_Interface_(SPDIF).md`, `19_Pulse_Density_Modulation_(PDM)_Microphone_Interface.md`

### Clock / Power / Frac-N PLL
- HRM: `04_Clock_Generation_Unit_(CGU).md`, `05_Clock_Distribution_Unit_(CDU).md`, `06_Fractional_PLL_(Frac‑N_PLL).md`, `07_Dynamic_Power_Management_(DPM).md`
- DS: Power domains (Table 8, lines 1065-1088), crystal oscillator (lines 979-991)

### FIR/IIR Accelerators
- HRM: `42_FIR_Accelerator_(FIR).md`, `43_IIR_Accelerator_(IIR).md`

### Security / HSM
- HRM: `37_System_Protection_Unit_(SPU).md`, `38_Security_Packet_Engine_(PKTE).md`, `39_Public_Key_Accelerator_(PKA).md`, `40_Public_Key_Interrupt_Controller_(PKIC).md`, `41_True_Random_Number_Generator_(TRNG).md`
- DS: HSM description (lines 526-528), crypto features (lines 492-512)

### PWM
- HRM: `24_Pulse-Width_Modulator_(PWM).md`

## Silicon Anomaly Awareness

Before finalizing answers about these subsystems, check `ADSP-SC84x-Anomaly/adsp-21844-21846-adsp-sc844-sc846-anomaly.md` and warn the user when relevant:

| ID | Area | Key implication |
|----|------|-----------------|
| DSPSI-23 | SPU/SMPU | SPU and SMPU MMR transactions may cause errors |
| DSPSI-25 | GP Timer | First interrupt/trigger can occur one EXTCLK edge late |
| DSPSI-26 | SPI | `SPI_SLVSEL` writes do not take effect |
| DSPSI-27 | S/PDIF | Receiver clock output pulse width unreliable above 96 kHz |
| DSPSI-33 | L1 IRAM / MEPU | Correctable SHARC-FX L1 IRAM errors may generate memory error exception |
| DSPSI-36 | Boot ROM | CRC protection plus page mode can cause boot errors |
| DSPSI-41 | USB/GPIO | USB controller and associated USB/GPIO pins are not functional |
| DSPSI-42 | eMSI/eMMC boot | eMMC user-area boot may fail due to missing CMD7 response during cleanup |

## Response Quality Rules

- **Cite sources**: When providing register addresses, timing values, or configuration steps, name the specific document and section.
- **Register details**: Provide both the register name and its MMR address when discussing register programming.
- **Processor variants**: Distinguish the four variants when relevant. ADSP-21844/21846 are DSP-only (no ARM); ADSP-SC844/SC846 include dual ARM Cortex-A55. SC846/21846 run at up to 1.2 GHz with 4 MB L2; SC844/21844 run at up to 800 MHz with 2 MB L2.
- **SHARC-FX vs SHARC+**: The SHARC-FX core is NOT ISA compatible with SHARC/SHARC+. It uses a different instruction encoding (16-128 bit VLIW), different register set (32 data registers), and 256-bit SIMD. Do not confuse with SC59x SHARC+ instructions.
- **Preliminary status**: Note that this is preliminary documentation (preferred Data Sheet PrD, anomaly list Rev A). Some specifications may change in the final release.
- **Verify before answering**: If unsure about a specific value (address, timing, bit field), read the source document rather than guessing.
