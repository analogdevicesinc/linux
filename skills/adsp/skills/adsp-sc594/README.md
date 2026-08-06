# ADSP-SC594 Skill for Claude Code

A Claude Code skill that turns Claude into a knowledgeable development assistant for **Analog Devices ADSP-SC592/SC594 SHARC+ processors** (and the no-ARM ADSP-21593/21594 variants) -- dual-SHARC+ DSPs with an ARM Cortex-A5, running up to 1 GHz, targeting high-performance audio, industrial, and automotive applications.

## What's Inside

The skill bundles the complete official documentation set, converted to markdown for fast in-context access:

| Document | Contents |
|----------|----------|
| **Data Sheet** | Electrical specs, timing, pinout, 400-ball HPC/LPC BGA assignments, feature comparison across all four parts (21593, 21594, SC592, SC594) |
| **Hardware Reference Manual** (59 chapters) | Every peripheral and system block: SPORT, SPI, OSPI, UART, I2C, CAN FD, USB, Ethernet (GbE + 10/100), DMA, clocks/PLL, GPIO, timers, watchdog, DDR3 controller, L2 memory, audio (DAI/SRU, ASRC, S/PDIF, PDM, PCG), security (TrustZone, crypto, TRNG), FIR/IIR accelerators, boot ROM, system crossbars, debug/trace, and more |
| **Programming Reference Manual** | SHARC+ core architecture, 11-stage pipeline, dual processing elements (ALU, multiplier, barrel shifter), ISA/VISA instruction set, DAGs, caches, register definitions, 64-bit floating-point, SIMD |
| **11 Engineer-to-Engineer Notes** | Boot time estimation, boot tips & ROM API, power estimation, thermal guidelines, TMU junction temperature estimation, DDR3 board design, FIR/IIR accelerator benchmarks, OSPI PHY training, system bus optimization, safe power-down, migration from ADSP-2156x |
| **Silicon Anomaly List** | All 18 known errata for Rev 0.0 silicon with workarounds |
| **Board & Tools** | EV-SC594-SOM evaluation board manual + full schematic, ICE-1500 emulator guide |

Total: **76 files** of processor documentation.

## How It Works

The skill uses a **two-phase lookup pattern** to efficiently navigate the large documentation set without flooding the context window:

1. **Index lookup** -- Claude first reads `DOCUMENTATION_INDEX.md`, a master topic-to-file map covering ~60 topics with exact file paths and line ranges
2. **Targeted reading** -- Claude reads only the relevant sections of the right file(s), using offset/limit parameters for large documents like the 22K-line Programming Reference

The skill also embeds a **silicon anomaly awareness table** directly in its instructions, so Claude proactively warns you about known errata when you're working with affected subsystems (SPI slave select bug, DMC PHY calibration, FIR accelerator edge cases, secure boot autodetect, OTP leaky bit, etc.).

## What You Can Ask

Pretty much anything about SC592/SC594 development:

- *"How do I configure SPORT0 for I2S stereo input on the SC594?"*
- *"Write the CGU init code to set CCLK to 1 GHz and SCLK0 to 250 MHz"*
- *"What's the DDR3 trace length matching requirement for the DMC?"*
- *"Show me the register setup for CAN FD at 5 Mbps data rate"*
- *"Are there any silicon errata I need to worry about for the FIR accelerator?"*
- *"How does the SHARC+ barrel shifter bit-reverse addressing work?"*
- *"What's the boot sequence when booting from OSPI flash?"*
- *"Help me estimate power consumption for my audio application"*
- *"How do I route a SPORT clock through the SRU in the DAI?"*
- *"Review my SPI init code for potential issues"*

Claude will cite specific documents and sections, provide register names with MMR addresses, and flag relevant anomalies automatically.

## Installation

The skill is installed as a **personal skill** (available across all projects):

```
~/.claude/skills/adsp-sc594/
├── SKILL.md                          # Skill instructions
├── DOCUMENTATION_INDEX.md            # Master navigation index
├── ADSP-SC594-DS/                    # Data Sheet
├── ADSP-SC594-HRM/                   # Hardware Reference Manual (59 files)
├── ADSP-SC594-PRM/                   # Programming Reference Manual
├── ADSP-SC594-EE-Notes/              # Engineer-to-Engineer Notes (11 files)
├── ADSP-SC594-Anomaly/               # Silicon Anomaly List
├── ev-sc594-som_manual.md            # EZ-KIT SOM board manual
├── ev-sc594-som-schematic.md         # EZ-KIT SOM schematic
└── mb_ice1500_emu_ug.md              # ICE-1500 emulator guide
```

## Usage

The skill **auto-triggers** whenever you mention ADSP, SC591, SC592, SC594, 21593, 21594, SHARC+, 2159x, or any SC592/SC594 peripheral by name. You can also invoke it explicitly:

```
/adsp-sc594
```

## Processor Overview

| Feature | ADSP-SC592 | ADSP-SC594 | ADSP-21593 | ADSP-21594 |
|---------|-----------|-----------|-----------|-----------|
| SHARC+ cores | 1 @ 1 GHz | 2 @ 800-1000 MHz | 2 @ 800-1000 MHz | 2 @ 1 GHz |
| ARM Cortex-A5 | 1 GHz | 800-1000 MHz | — | — |
| L1 SRAM | 640 kB | 2 x 640 kB | 2 x 640 kB | 2 x 640 kB |
| L2 shared SRAM | 2 MB | 2 MB | 2 MB | 2 MB |
| DDR3/DDR3L | 16-bit | 16-bit | 16-bit | 16-bit |
| Package | 400-ball HPC BGA | 400-ball HPC BGA | 400-ball LPC BGA | 400-ball HPC BGA |

All four processors include: 8 SPORTs (4 per DAI), 2 DAI blocks, 8 ASRCs, 2 S/PDIF, 8 PCGs, 1 EMAC (GbE + 10/100 with IEEE 1588/AVB), 2 CAN FD, 4 UART, 6 TWI/I2C, 2 Dual-data SPI, OSPI, USB 2.0 OTG, EPPI, MLB (MOST), 16 GP timers, FIR/IIR hardware accelerators, hardware crypto (AES/SHA/RSA/ECC/TRNG), and TrustZone security.

### Key Distinction from ADSP-SC59x (SC595/SC596/SC598)

This family uses the **ARM Cortex-A5 (Armv7)** -- **not** the Cortex-A55 (Armv8) found in SC595/SC596/SC598. The audio subsystem is named **DAI** (Digital Audio Interface) rather than SHAI. There is no eMMC/SD (eMSI) interface and no DDR prefetch buffer on this family.
