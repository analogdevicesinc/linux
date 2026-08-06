# ADSP-SC59x Skill for Claude Code

A Claude Code skill that turns Claude into a knowledgeable development assistant for **Analog Devices ADSP-SC595/SC596/SC598 SHARC+ processors** -- dual-core DSPs with an ARM Cortex-A55, running up to 1.2 GHz, targeting high-performance audio, industrial, and automotive applications.

## What's Inside

The skill bundles the complete official documentation set, converted to markdown for fast in-context access:

| Document | Contents |
|----------|----------|
| **Data Sheet** | Electrical specs, timing, pinout, 400-ball BGA assignments, feature comparison |
| **Hardware Reference Manual** (57 chapters) | Every peripheral and system block: SPORT, SPI, OSPI, UART, I2C, CAN FD, USB, Ethernet (GbE + 10/100), eMMC/SD, DMA, clocks/PLL, GPIO, timers, watchdog, DDR3 controller, L2 memory, audio (ASRC, S/PDIF, PDM, PCG), security (TrustZone, crypto, TRNG), FIR/IIR accelerators, boot ROM, system crossbars, debug/trace, and more |
| **Programming Reference Manual** | SHARC+ core architecture, 11-stage pipeline, dual processing elements (ALU, multiplier, barrel shifter), ISA/VISA instruction set, DAGs, caches, register definitions, 64-bit floating-point, SIMD |
| **14 Engineer-to-Engineer Notes** | Boot time estimation, power estimation, thermal guidelines, DDR3 board design & programming, FIR/IIR accelerator benchmarks, OSPI PHY training, eMSI optimization, system bus optimization, safe power-down, migration from ADSP-2156x |
| **Silicon Anomaly List** | All 18 known errata for Rev 0.0 silicon with workarounds |
| **Board & Tools** | EV-SC598-SOM evaluation board manual + full schematic, ICE-1500 emulator guide, BGA package drawing |
| **SVD Register Maps** | CMSIS-SVD peripheral register definitions for SC598 (93 peripherals) and SC598W (95 peripherals) -- bit fields, offsets, reset values, access types |

Total: **83 files, ~203 MB** of processor documentation.

## How It Works

The skill uses a **two-phase lookup pattern** to efficiently navigate the large documentation set without flooding the context window:

1. **Index lookup** -- Claude first reads `DOCUMENTATION_INDEX.md`, a master topic-to-file map covering ~60 topics with exact file paths and line ranges
2. **Targeted reading** -- Claude reads only the relevant sections of the right file(s), using offset/limit parameters for large documents like the 22K-line Programming Reference

The skill also embeds a **silicon anomaly awareness table** directly in its instructions, so Claude proactively warns you about known errata when you're working with affected subsystems (SPI slave select bug, DMC PHY calibration, FIR accelerator edge cases, eMSI/boot issues, etc.).

## What You Can Ask

Pretty much anything about SC59x development:

- *"How do I configure SPORT0 for I2S stereo input on the SC598?"*
- *"Write the CGU init code to set CCLK to 1 GHz and SCLK0 to 250 MHz"*
- *"What's the DDR3 trace length matching requirement for the DMC?"*
- *"Show me the register setup for CAN FD at 5 Mbps data rate"*
- *"Are there any silicon errata I need to worry about for the FIR accelerator?"*
- *"How does the SHARC+ barrel shifter bit-reverse addressing work?"*
- *"What's the boot sequence when booting from OSPI flash?"*
- *"Help me estimate power consumption for my audio application"*
- *"Review my SPI init code for potential issues"*

Claude will cite specific documents and sections, provide register names with MMR addresses, and flag relevant anomalies automatically.

## Installation

The skill is installed as a **personal skill** (available across all projects):

```
~/.claude/skills/adsp-sc59x/
├── SKILL.md                          # Skill instructions
├── DOCUMENTATION_INDEX.md            # Master navigation index
├── references/
│   └── SILICON_ANOMALIES_SUMMARY.md  # Condensed errata reference
├── ADSP-SC59x-DS/                    # Data Sheet
├── ADSP-SC59x-HRM/                   # Hardware Reference Manual (57 files)
├── ADSP-SC59x-PRM/                   # Programming Reference Manual
├── ADSP-SC59x-EE-Notes/              # Engineer-to-Engineer Notes (14 files)
├── ADSP-SC59x-Anomaly/               # Silicon Anomaly List
├── ADSP-SC598.svd                    # SVD Register Map (SC598, 93 peripherals)
├── ADSP-SC598W.svd                   # SVD Register Map (SC598W, 95 peripherals)
├── ev-sc598-som_manual.md            # EZ-KIT SOM board manual
├── ev-sc598-som-schematic.md         # EZ-KIT SOM schematic
├── mb_ice1500_emu_ug.md              # ICE-1500 emulator guide
└── bp-400-3.md                       # BGA package drawing
```

## Usage

The skill **auto-triggers** whenever you mention ADSP, SC595, SC596, SC598, SHARC+, or any SC59x peripheral by name. You can also invoke it explicitly:

```
/adsp-sc59x
```

## Processor Overview

| Feature | SC596 | SC598 |
|---------|-------|-------|
| SHARC+ cores | 1 @ 1 GHz | 2 @ 812.5-1000 MHz |
| ARM Cortex-A55 | 1.2 GHz | 1.2 GHz |
| L1 SRAM | 1 MB | 2 x 1 MB |
| L2 shared SRAM | 2 MB | 2 MB |
| DDR3/DDR3L | 16-bit | 16-bit |
| Package | 400-ball BGA | 400-ball BGA |

Both processors include: 8 SPORTs, 2 DAI blocks, 8 ASRCs, 2 S/PDIF, 8 PCGs, 2 EMAC (GbE + 10/100 with IEEE 1588/TSN/AVB), 2 CAN FD, 4 UART, 6 TWI/I2C, 4 SPI (incl. QSPI), OSPI, USB 2.0 OTG, eMSI (eMMC/SD), EPPI, 16 GP timers, FIR/IIR hardware accelerators, hardware crypto (AES/SHA/RSA/ECC/TRNG), and TrustZone security.
