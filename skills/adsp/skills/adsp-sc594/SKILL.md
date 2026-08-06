---
name: adsp-sc594
description: >
  ADSP-SC592/SC594 SHARC+ DSP processor development assistant (also covers
  ADSP-21593/21594 no-ARM variants). This skill should be used when the user
  asks about firmware development, peripheral configuration, register
  programming, SHARC+ assembly or ISA, DMA setup, audio subsystem (SPORT, I2S,
  TDM, ASRC, SPDIF, PDM, PCG, DAI/SRU), Ethernet EMAC, SPI, OSPI, UART,
  I2C/TWI, CAN FD, USB, DDR3 memory controller (DMC), clock/PLL/CGU
  configuration, boot process, power management, pin mux/GPIO, silicon
  anomalies or errata, board design, EZ-KIT SOM, ARM Cortex-A5 subsystem,
  FIR/IIR accelerators, security (TrustZone, crypto), debug/JTAG/trace,
  system crossbar optimization, or any topic related to Analog Devices
  SHARC+ SC592/SC594 processors. Make sure to use this skill whenever the
  user mentions ADSP, SC591, SC592, SC594, 21591, 21593, 21594, SHARC, SHARC+,
  2159x, or any SC59x-family peripheral by name.
---

# ADSP-SC592/SC594 Development Assistant

You have access to the complete documentation set for the Analog Devices ADSP-SC592/SC594 SHARC+ processor family (also covering ADSP-21593/21594 no-ARM variants): Data Sheet, Hardware Reference Manual (59 chapters), Programming Reference Manual, 11 Engineer-to-Engineer application notes, and the silicon anomaly list. All documentation is in markdown format at the project root.

## Documentation Navigation Protocol

**Step 1 -- Always start by reading the index.**
Read `DOCUMENTATION_INDEX.md` at the project root. It contains a topic-to-file mapping table that tells you exactly which file(s) and line ranges to consult for any topic.

**Step 2 -- Read targeted sections, not entire files.**
Use the Read tool with `offset` and `limit` parameters for large files:
- **PRM** (`ADSP-SC594-PRM/sc58x-2158x-prm.md`): 22,333 lines. Use the chapter/line-range table from the index. Never read the whole file.
- **Data Sheet** (`ADSP-SC594-DS/adsp-21593-21594-adsp-sc592-sc594.md`): 5,278 lines. Use section ranges from the index.
- **HRM files**: Most are under 2,000 lines and can be read in full. The Ethernet register file (036) is larger -- read selectively.

**Step 3 -- Cross-reference when needed.**
For any peripheral topic, consider consulting multiple sources:
- **HRM** chapter: Functional description, programming model, registers
- **Data Sheet**: Timing specs, electrical characteristics, signal descriptions
- **EE Notes**: Optimization techniques, board design, practical guidance
- **Anomaly list**: Known errata and workarounds (see below)

## Key File Paths

| Document | Path |
|----------|------|
| Data Sheet | `ADSP-SC594-DS/adsp-21593-21594-adsp-sc592-sc594.md` |
| Programming Reference | `ADSP-SC594-PRM/sc58x-2158x-prm.md` |
| HRM (59 chapters) | `ADSP-SC594-HRM/000_*.md` through `058_*.md` |
| EE Notes (11 notes) | `ADSP-SC594-EE-Notes/ee*.md` |
| Anomaly List | `ADSP-SC594-Anomaly/adsp-21591_21593_21594_adsp-sc591_sc592_sc594-anomaly.md` |
| Register Address List | `ADSP-SC594-HRM/058_ADSP-2159x_SC592_SC594_Register_List_pages.md` |
| EZ-KIT SOM Manual | `ev-sc594-som_manual.md` |
| EZ-KIT SOM Schematic | `ev-sc594-som-schematic.md` |
| ICE-1500 Emulator Guide | `mb_ice1500_emu_ug.md` |
| Navigation Index | `DOCUMENTATION_INDEX.md` |

## Silicon Anomaly Awareness

**Always check for relevant anomalies** when discussing the subsystems below. Read the full anomaly file (`ADSP-SC594-Anomaly/adsp-21591_21593_21594_adsp-sc591_sc592_sc594-anomaly.md`) for detailed workarounds.

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
| 20000110 | Boot/Security | Secure image auth bypassed during autodetect on POR when locked |
| 20000113 | Boot/UART | UART3 slave boot fails with autobaud enabled |
| 20000114 | FIR Accel | Circular buffer fails in burst-16 mode |
| 20000115 | LPC/SYS_FAULT | SYS_FAULT signals active out of reset on LPC parts |
| 20000117 | DMC | PHY calibration issue (workaround required) |
| 20000118 | FIR Accel | Wrong output for tap length > 1024 multi-iteration |
| 20000123 | Boot | Boot failure with ignore block in page mode |
| 20000124 | DMC/Boot | DMC init routine not usable in boot ROM |
| 20000128 | FIR/IIR | Core write to IRPTL clears pending accel interrupts |
| 20000129 | OTP | OTP Program API fails in presence of a leaky bit |

## Task-Specific Routing

### Peripheral Configuration
1. Find the HRM chapter for that peripheral (use the index)
2. Check `058_ADSP-2159x_SC592_SC594_Register_List_pages.md` for register addresses
3. Check DS timing specs (lines 3256-4607 in the data sheet)
4. Check anomaly table above for that subsystem

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
- HRM: `053_Boot_ROM_and_Booting_the_Processor.md`
- EE: `ee432.md` (boot time estimation), `ee447v01.md` (boot tips, ROM API, OTP, secure boot)
- Anomalies: 20000110 (secure boot), 20000113 (UART slave boot), 20000123, 20000124

### DDR3 / DMC Setup
- HRM: `012_Dynamic_Memory_Controller_DMC.md`
- EE: `ee434.md` (board design guidelines)
- Anomaly 20000117: DMC PHY calibration workaround (critical)
- Anomaly 20000124: DMC init not usable in boot ROM

### Audio Subsystem
- HRM: `037_Digital_Audio_Interface_DAI.md` (SRU signal routing), `038_Serial_Port_SPORT.md`, `039_Precision_Clock_Generators_PCG.md`, `040_Asynchronous_Sample_Rate_Converter_ASRC.md`, `041_SonyPhilips_Digital_Interface_SPDIF.md`, `021_Pulse_Density_Modulation_PDM_Microphone_Interface.md`
- Anomaly 20000103: SPDIF Rx clock issue above 96 kHz

### Clock / Power / Thermal
- HRM: `005_Clock_Generation_Unit_CGU.md`, `006_Clock_Distribution_Unit_CDU.md`, `007_Dynamic_Power_Management_DPM.md`
- EE: `ee433.md` (power estimation), `ee449v01.md` (thermal guidelines)
- EE: `ee448v01.md` (safe power-down), `ee458v01.md` (TMU peak junction temp)

### FIR/IIR Accelerators
- HRM: `051_FIR_Accelerator_FIR.md`, `052_IIR_Accelerator_IIR.md`
- EE: `ee436v02.md` (usage, benchmarks, programming models)
- Anomalies: 20000114, 20000118 (FIR), 20000128 (interrupt clearing)

### OSPI
- HRM: `024_Octal_Serial_Peripheral_Interface_OSPI.md`
- EE: `ee-437.md` (PHY configuration and training)

### Board Design
- EV-SC594-SOM schematic (`ev-sc594-som-schematic.md`) as reference design
- DS signal descriptions (lines 1277-2860) and pin mux tables
- EE board design notes: `ee434.md`

### System Optimization
- EE: `ee445v01.md` (bus architecture, crossbars, DMA, memory throughput)
- HRM: `054_System_Crossbars_SCB.md`

### Migration
- EE: `ee-430.md` (migrating from ADSP-2156x to ADSP-SC59x/2159x)

## Response Quality Rules

- **Cite sources**: When providing register addresses, timing values, or configuration steps, name the specific document and section.
- **Surface anomalies**: When writing or reviewing code for an affected subsystem, proactively mention the relevant anomaly and its workaround.
- **Register details**: Provide both the register name and its MMR address when discussing register programming.
- **Processor variants**: Distinguish SC592/SC594/21593/21594 differences when relevant. SC592 has 1 SHARC+ core; SC594, 21593, and 21594 have 2 SHARC+ cores. SC592/SC594 include an ARM Cortex-A5 (Armv7); 21593/21594 do not. ADSP-21593 uses the LPC BGA package with a reduced peripheral set; all others use HPC BGA.
- **ARM distinction**: This family uses **ARM Cortex-A5 (Armv7)**, NOT the Cortex-A55 (Armv8) found in SC595/SC596/SC598. Do not confuse the two.
- **Audio interface naming**: This family calls it **DAI** (Digital Audio Interface), not SHAI as in SC59x.
- **Verify before answering**: If unsure about a specific value (address, timing, bit field), read the source document rather than guessing.
