---
name: adsp-sc59x
description: >
  ADSP-SC59x (SC595/SC596/SC598) SHARC+ DSP processor development assistant.
  This skill should be used when the user asks about firmware development,
  peripheral configuration, register programming, SHARC+ assembly or ISA,
  DMA setup, audio subsystem (SPORT, I2S, TDM, ASRC, SPDIF, PDM, PCG),
  Ethernet EMAC, SPI, OSPI, UART, I2C/TWI, CAN FD, USB, eMMC/eMSI,
  DDR3 memory controller (DMC), clock/PLL/CGU configuration, boot process,
  power management, pin mux/GPIO, silicon anomalies or errata, board design,
  EZ-KIT SOM, ARM Cortex-A55 subsystem, FIR/IIR accelerators, security
  (TrustZone, crypto), debug/JTAG/trace, system crossbar optimization,
  or any topic related to Analog Devices SHARC+ SC59x processors.
  Make sure to use this skill whenever the user mentions ADSP, SC595, SC596,
  SC598, SHARC, SHARC+, 2159x, or any SC59x peripheral by name.
---

# ADSP-SC59x Development Assistant

You have access to the complete documentation set for the Analog Devices ADSP-SC595/SC596/SC598 SHARC+ processor family: Data Sheet, Hardware Reference Manual (57 chapters), Programming Reference Manual, 14 Engineer-to-Engineer application notes, and the silicon anomaly list. All documentation is in markdown format at the project root.

## Documentation Navigation Protocol

**Step 1 -- Always start by reading the index.**
Read `DOCUMENTATION_INDEX.md` at the project root. It contains a topic-to-file mapping table that tells you exactly which file(s) and line ranges to consult for any topic.

**Step 2 -- Read targeted sections, not entire files.**
Use the Read tool with `offset` and `limit` parameters for large files:
- **PRM** (`ADSP-SC59x-PRM/sc58x-2158x-prm.md`): 22,334 lines. Use the chapter/line-range table from the index. Never read the whole file.
- **Data Sheet** (`ADSP-SC59x-DS/adsp-sc596-adsp-sc598.md`): 4,759 lines. Use section ranges from the index.
- **HRM files**: Most are under 2,000 lines and can be read in full. The Ethernet register files (037, 304) are larger -- read selectively.

**Step 3 -- Cross-reference when needed.**
For any peripheral topic, consider consulting multiple sources:
- **HRM** chapter: Functional description, programming model, registers
- **Data Sheet**: Timing specs, electrical characteristics, signal descriptions
- **EE Notes**: Optimization techniques, board design, practical guidance
- **Anomaly list**: Known errata and workarounds (see below)
- **SVD files**: Exact register bit field definitions, offsets, widths, reset values, and access types

**SVD file usage notes:**
The SVD files (`ADSP-SC598.svd`, `ADSP-SC598W.svd`) are very large (500K+ lines each). Never read them in full. Instead, use the Grep tool to search for specific peripheral or register names (e.g., grep for `<name>SPI0</name>` or `<name>SPI_CTL</name>`), then read only the relevant section with offset/limit. Use SVD files when you need machine-precise register field layouts (bit offsets, widths, reset values) for code generation or register verification. Prefer the HRM for narrative explanations of how registers work.

## Key File Paths

| Document | Path |
|----------|------|
| Data Sheet | `ADSP-SC59x-DS/adsp-sc596-adsp-sc598.md` |
| Programming Reference | `ADSP-SC59x-PRM/sc58x-2158x-prm.md` |
| HRM (57 chapters) | `ADSP-SC59x-HRM/003_*.md` through `326_*.md` |
| EE Notes (14 notes) | `ADSP-SC59x-EE-Notes/ee*.md` |
| Anomaly List | `ADSP-SC59x-Anomaly/adsp-sc595-sc596-sc598-anomaly.md` |
| Register Address List | `ADSP-SC59x-HRM/326_ADSP-SC59x_Register_List_pages.md` |
| SVD Register Map (SC598) | `ADSP-SC598.svd` (CMSIS-SVD, 522K lines, 93 peripherals) |
| SVD Register Map (SC598W) | `ADSP-SC598W.svd` (CMSIS-SVD, 529K lines, 95 peripherals) |
| EZ-KIT SOM Manual | `ev-sc598-som_manual.md` |
| EZ-KIT SOM Schematic | `ev-sc598-som-schematic.md` |
| ICE-1500 Emulator Guide | `mb_ice1500_emu_ug.md` |
| Navigation Index | `DOCUMENTATION_INDEX.md` |

## Silicon Anomaly Awareness

**Always check for relevant anomalies** when discussing the subsystems below. Read `references/SILICON_ANOMALIES_SUMMARY.md` for the condensed list. Read the full anomaly file for detailed workarounds.

| ID | Subsystem | Summary |
|----|-----------|---------|
| 20000002 | CPU Pipeline | Data forwarding Rn/Sn to DAG may fail with stalls |
| 20000003 | SPU/SMPU | Non-secure MMR access to upper half may error |
| 20000031 | GP Timer | First interrupt one edge late in EXTCLK mode |
| 20000062 | SPI | Write to SPI_SLVSEL has no effect (must write twice) |
| 20000069 | CPU Pipeline | PCSTK/MODE1STK loads fail if next instr is L2/L3 |
| 20000072 | CPU Pipeline | Float compute targeting F0 can cause stalls |
| 20000096 | CPU Pipeline | Type 18a USTAT fails after specific sequences |
| 20000103 | SPDIF | Unreliable Rx clock pulse width above 96 kHz |
| 20000114 | FIR Accel | Circular buffer fails in burst-16 mode |
| 20000117 | DMC | PHY calibration issue (76-step workaround) |
| 20000118 | FIR Accel | Wrong output for tap length > 1024 multi-iteration |
| 20000119 | eMSI | END bit error during data transfers (clock gating) |
| 20000120 | eMSI/Boot | eMMC boot failure due to clock gating |
| 20000121 | eMSI/Boot | eMMC device identification may fail during boot |
| 20000123 | Boot | Boot failure with ignore block in page mode |
| 20000124 | DMC/Boot | DMC init routine not usable in boot ROM |
| 20000126 | eMSI | Incorrect EMSI_CAP2 register bit fields |
| 20000128 | FIR/IIR | Core write to IRPTL clears pending accel interrupts |

## Task-Specific Routing

### Peripheral Configuration
1. Find the HRM chapter for that peripheral (use the index)
2. Check `326_ADSP-SC59x_Register_List_pages.md` for register addresses
3. For exact bit field definitions, grep the SVD files (`ADSP-SC598.svd` or `ADSP-SC598W.svd`) for the peripheral/register name
4. Check DS timing specs (lines 2900-4100 in the data sheet)
5. Check anomaly table above for that subsystem

### SHARC+ Assembly / ISA Questions
Use the PRM with these line ranges:
- Architecture overview: Ch 1-2 (lines 1061-1768)
- Processing elements (ALU/MUL/Shifter): Ch 3 (lines 1769-3056)
- Program sequencer / pipeline: Ch 4 (lines 3057-5306)
- Data Address Generators: Ch 6 (lines 5363-6176)
- L1 memory / cache: Ch 7-8 (lines 6177-7287)
- Instruction set reference: Ch 13-17 (lines 7890-12353)
- Computation opcodes: Ch 18-25 (lines 12354-16192)
- Register descriptions: Ch 29-34 (lines 17448-20294)
- Numeric formats: Ch 28 (lines 17343-17447)

### Boot Configuration
- HRM: `321_Boot_ROM_and_Booting_the_Processor.md`
- EE: `ee432.md` (SC59x boot time), `ee438v01.md` (SC598 + eMMC/SD)
- Anomalies: 20000119-20000124 affect eMSI/boot

### DDR3 / DMC Setup
- HRM: `011_Dynamic_Memory_Controller_DMC.md`, `012_DDR_Pre-fetch_Buffer_DDRPFB.md`
- EE: `ee434.md` / `ee-441.md` (board design), `ee-443.md` (programming/init code)
- Anomaly 20000117: DMC PHY calibration workaround (critical)
- Anomaly 20000124: DMC init not usable in boot ROM

### Audio Subsystem
- HRM: `306_Serial_Port_SPORT.md`, `307_Precision_Clock_Generators_PCG.md`, `308_Asynchronous_Sample_Rate_Converter_ASRC.md`, `309_SonyPhilips_Digital_Interface_SPDIF.md`, `022_Pulse_Density_Modulation_PDM_Microphone_Interface.md`
- Anomaly 20000103: SPDIF Rx clock issue above 96 kHz

### Clock / Power
- HRM: `004_Clock_Generation_Unit_CGU.md`, `005_Clock_Distribution_Unit_CDU.md`, `006_Dynamic_Power_Management_DPM.md`
- EE: `ee433.md` / `ee440v01.md` (power estimation), `ee451v01.md` (thermal)
- EE: `ee448v01.md` (safe power-down)

### FIR/IIR Accelerators
- HRM: `319_FIR_Accelerator_FIR.md`, `320_IIR_Accelerator_IIR.md`
- EE: `ee436v02.md` (usage, benchmarks, programming models)
- Anomalies: 20000114, 20000118 (FIR), 20000128 (interrupt clearing)

### Board Design
- EZ-KIT SOM schematic (`ev-sc598-som-schematic.md`) as reference design
- DS signal descriptions (lines 1350-2000) and pin mux tables
- EE board design notes: `ee434.md`, `ee-441.md`, `ee-443.md`

### System Optimization
- EE: `ee445v01.md` (bus architecture, DMA, memory throughput, QoS)
- HRM: `322_System_Crossbars_SCB.md`

## Response Quality Rules

- **Cite sources**: When providing register addresses, timing values, or configuration steps, name the specific document and section.
- **Surface anomalies**: When writing or reviewing code for an affected subsystem, proactively mention the relevant anomaly and its workaround.
- **Register details**: Provide both the register name and its MMR address when discussing register programming.
- **Processor variants**: Distinguish SC595/SC596/SC598 differences when relevant. SC596 has 1 SHARC+ core; SC598 has 2. Both have ARM Cortex-A55 at 1.2 GHz.
- **Verify before answering**: If unsure about a specific value (address, timing, bit field), read the source document rather than guessing.
