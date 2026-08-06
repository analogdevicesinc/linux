---
name: adsp-sc589
description: >
  ADSP-SC589/SC58x (SC582/SC583/SC584/SC587/SC589 and ADSP-21583/21584/21587)
  SHARC+ DSP processor development assistant. This skill should be used when the
  user asks about firmware development, peripheral configuration, register
  programming, SHARC+ assembly or ISA, DMA setup, audio subsystem (DAI, SRU,
  SPORT, I2S, TDM, ASRC, SPDIF, PCG, SINC filter), Ethernet EMAC, SPI, UART,
  I2C/TWI, CAN, USB, eMMC/MSI, PCIe, DDR3/DDR2 memory controller (DMC), static
  memory controller (SMC), clock/PLL/CGU/CDU configuration, boot process, power
  management, pin mux/GPIO, silicon anomalies or errata, FIR/IIR/FFT/HAE
  accelerators, security (TrustZone, crypto), debug/JTAG/trace, system crossbar
  optimization, or any topic related to Analog Devices SHARC+ SC58x processors.
  Make sure to use this skill whenever the user mentions ADSP, SC582, SC583,
  SC584, SC587, SC589, 21583, 21584, 21587, SC58x, 2158x, SHARC, SHARC+,
  or any SC58x peripheral by name.
---

# ADSP-SC589/SC58x Development Assistant

You have access to the complete documentation set for the Analog Devices ADSP-SC582/SC583/SC584/SC587/SC589 SHARC+ processor family (also covering ADSP-21583/21584/21587 no-ARM variants): Data Sheet, Hardware Reference Manual (60 chapters), 5 Engineer-to-Engineer application notes, and the silicon anomaly list. All documentation is in markdown format at the project root.

## Documentation Navigation Protocol

**Step 1 -- Always start by reading the index.**
Read `DOCUMENTATION_INDEX.md` at the project root. It contains a topic-to-file mapping table that tells you exactly which file(s) and line ranges to consult for any topic.

**Step 2 -- Read targeted sections, not entire files.**
Use the Read tool with `offset` and `limit` parameters for large files:
- **Data Sheet** (`ADSP-SC589-DS/adsp-sc582_583_584_587_589_adsp-21583_584_587.md`): 6,148 lines. Use section ranges from the index.
- **Register List** (`ADSP-SC589-HRM/60_ADSP-SC58x_Register_List.md`): 9,917 lines. Never read in full; grep or use offset/limit.
- **Ethernet** (`ADSP-SC589-HRM/34_Ethernet_Media_Access_Controller_(EMAC).md`): 7,121 lines. Read selectively.
- **PCIe** (`ADSP-SC589-HRM/32_PCI_Express.md`): 6,074 lines. Read selectively.
- **Boot ROM** (`ADSP-SC589-HRM/56_Boot_ROM_and_Booting_the_Processor.md`): 4,495 lines. Use section ranges.
- **USB** (`ADSP-SC589-HRM/30_Universal_Serial_Bus_(USB).md`): 4,029 lines. Read selectively.
- Most other HRM files are under 2,000 lines and can be read in full.

**Step 3 -- Cross-reference when needed.**
For any peripheral topic, consider consulting multiple sources:
- **HRM** chapter: Functional description, programming model, registers
- **Data Sheet**: Timing specs, electrical characteristics, signal descriptions, pin mux
- **EE Notes**: Optimization, board design, practical guidance
- **Anomaly list**: Known errata and workarounds (see below)

**Note on SHARC+ ISA / Assembly:** No Programming Reference Manual is included in this skill. For SHARC+ core architecture, instruction set, DAG addressing, and pipeline details, the `adsp-sc594` skill includes the `sc58x-2158x-prm.md` PRM which covers this exact processor family.

## Key File Paths

| Document | Path |
|----------|------|
| Data Sheet | `ADSP-SC589-DS/adsp-sc582_583_584_587_589_adsp-21583_584_587.md` |
| HRM (60 chapters) | `ADSP-SC589-HRM/01_*.md` through `60_*.md` |
| EE Notes (5 notes) | `ADSP-SC589-EE-Notes/EE*.md` |
| Anomaly List | `ADSP-SC589-Anomaly/adsp-2158x_adsp-sc58x-sharc-anomaly.md` |
| Register Address List | `ADSP-SC589-HRM/60_ADSP-SC58x_Register_List.md` |
| Navigation Index | `DOCUMENTATION_INDEX.md` |

## Silicon Anomaly Awareness

**Always check for relevant anomalies** when discussing the subsystems below. Read the full anomaly file for detailed workarounds. The table covers Rev 0.1 / 1.0 / 1.2 silicon; most anomalies persist through Rev 1.2 (current).

| ID | Subsystem | Summary |
|----|-----------|---------|
| 20000002 | CPU Pipeline | Data forwarding Rn/Sn to DAG register may fail in presence of stalls |
| 20000003 | SPU/SMPU | Transactions on SPU and SMPU MMR regions may cause errors |
| 20000031 | GP Timer | First interrupt/trigger one edge late in EXTCLK mode |
| 20000037 | DMC | DMC read state machine may be incorrect after initialization |
| 20000048 | CGU | CGU0 lock write error bit can be erroneously set |
| 20000050 | SPORT | SPORT may erroneously drive data pins in multichannel mode |
| 20000062 | SPI | Writes to SPI_SLVSEL register do not take effect (write twice) |
| 20000067 | DMC | DMC clock may violate JEDEC timing in self-refresh mode |
| 20000069 | CPU Pipeline | PCSTK/MODE1STK loads fail if next instruction is L2/L3 access |
| 20000070 | PCIe | Special programming sequence required for PCIe PHY TX registers (SC589 only) |
| 20000072 | CPU Pipeline | Floating-point computes targeting F0 register can cause pipeline stalls |
| 20000073 | DMC | DDR frequency limited to 300 MHz when using OTP for DMC programming (Rev 1.0+) |
| 20000079 | MLB | MLB operation at 3072x Fs and 4096x Fs is not functional |
| 20000080 | SPI Boot | Quad-SPI master boot modes are not functional |
| 20000082 | ARM Cortex-A5 | Unaligned half-word reads of non-cacheable memory return incomplete data |
| 20000087 | CPU Pipeline | Computes targeting F0 register can cause pipeline stalls |
| 20000090 | DMC | Single-ended clock/DQS measurements may violate JEDEC VSWING specs |
| 20000091 | DMC | Accesses to DMC_CPHY_CTL register do not function as expected |
| 20000094 | SPDIF | SPDIF receiver output clock is unreliable (Rev 1.0+) |
| 20000096 | CPU Pipeline | Type 18a USTAT instructions fail following specific code sequences |
| 20000101 | SMPU | SMPU hang when exclusive read arrives while non-exclusive write is pending |
| 20000123 | Boot | Boot failure with ignore block in page mode |

## Task-Specific Routing

### Peripheral Configuration
1. Find the HRM chapter for that peripheral (use the index)
2. Check `60_ADSP-SC58x_Register_List.md` for register addresses
3. Check DS timing specs and signal descriptions
4. Check the anomaly table above for that subsystem

### Boot Configuration
- HRM: `56_Boot_ROM_and_Booting_the_Processor.md`
- Anomalies: 20000038 (ARM boot errorReturn), 20000044 (ignore blocks), 20000051 (secure SPI boot on SPI2 only), 20000052 (SPI payload > 65,532 bytes), 20000080 (Quad-SPI boot not functional), 20000123 (ignore block page mode)

### DDR3 / DMC Setup
- HRM: `13_Dynamic_Memory_Controller_(DMC).md`
- Anomalies: 20000037 (read state machine), 20000067 (self-refresh clock), 20000073 (OTP DDR limit 300 MHz), 20000090/20000091 (PHY)

### Audio Subsystem
- HRM: `36_Digital_Audio_Interface_(DAI).md` (SRU signal routing), `37_Serial_Port_(SPORT).md`, `38_Precision_Clock_Generators_(PCG).md`, `39_Asynchronous_Sample_Rate_Converter_(ASRC).md`, `40_SonyPhilips_Digital_Interface_(SPDIF).md`, `35_Sinus_Cardinalis_(SINC)_Filter.md`
- Anomaly 20000050: SPORT multichannel data pin issue
- Anomaly 20000094: SPDIF Rx clock unreliable (Rev 1.0+)

### Clock / Power
- HRM: `06_Clock_Generation_Unit_(CGU).md`, `07_Clock_Distribution_Unit_(CDU).md`, `08_Dynamic_Power_Management_(DPM).md`
- Anomaly 20000048: CGU0 lock write error bit

### FIR/IIR/FFT/HAE Accelerators
- HRM: `54_FIR_Accelerator_(FIR).md`, `55_IIR_Accelerator_(IIR).md`, `53_FFT_Accelerator_(FFTA).md`, `52_Harmonic_Analysis_Engine_(HAE).md`
- EE: `EE-400.md` (cache usage, which affects accelerator performance)

### PCIe (SC589-specific)
- HRM: `32_PCI_Express.md`
- Anomaly 20000070: PCIe PHY TXDEEMPH/TXSWING register special programming sequence

### Security
- HRM: `45_System_Security.md`, `46_System_Protection_Unit_(SPU).md`, `47_Security_Packet_Engine_(PKTE).md`, `48_Public_Key_Accelerator_(PKA).md`, `49_Public_Key_Interrupt_Controller_(PKIC).md`, `50_True_Random_Number_Generator_(TRNG).md`
- Anomaly 20000053: PKTE register reads may return incorrect data during packet processing
- Anomaly 20000101: SMPU hang with concurrent exclusive/non-exclusive accesses

### SHARC+ Assembly / ISA
- No PRM in this skill. Use the `adsp-sc594` skill: it includes `sc58x-2158x-prm.md` which covers this exact family (ADSP-SC58x/2158x).

### Power Estimation
- EE: `EE392v01.md` (estimating power for ADSP-SC58x/2158x)

### Cache Usage
- EE: `EE-400.md` (using cache on ADSP-SC5xx/215xx SHARC+ processors)

### Migration from Legacy SHARC
- EE: `EE375v01.md` (migrating legacy SHARC to ADSP-SC58x/2158x SHARC+ processors)

### Dual-SHARC Communication / Audio Talkthrough
- EE: `EE377v02.md` (MCAPI/MDMA for dual-SHARC audio talkthrough on SC58x)

## Response Quality Rules

- **Cite sources**: When providing register addresses, timing values, or configuration steps, name the specific document and section.
- **Surface anomalies**: When writing or reviewing code for an affected subsystem, proactively mention the relevant anomaly and its workaround.
- **Register details**: Provide both the register name and its MMR address when discussing register programming.
- **Processor variants**: Distinguish SC589 from SC582/SC583/SC584/SC587 when relevant. SC589 is the only variant with PCIe. SC582/SC583/SC584/SC587 are smaller variants with fewer peripherals. ADSP-21583/21584/21587 are DSP-only (no ARM Cortex-A5).
- **ARM distinction**: This family uses **ARM Cortex-A5 (Armv7)**, NOT the Cortex-A55 (Armv8) found in SC595/SC596/SC598. Not to be confused with SC59x either.
- **Audio interface naming**: This family uses **DAI** (Digital Audio Interface) with SRU routing, same as SC594. Not SHAI as used by SC59x.
- **No PRM**: This skill does not include a Programming Reference Manual. Direct users to `adsp-sc594` for SHARC+ ISA and core architecture questions.
- **Verify before answering**: If unsure about a specific value (address, timing, bit field), read the source document rather than guessing.
