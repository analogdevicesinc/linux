# ADSP-SC589 Skill for Claude Code

A Claude Code skill that turns Claude into a knowledgeable development assistant for **Analog Devices ADSP-SC589/SC58x SHARC+ processors** -- dual-SHARC+ DSPs with an ARM Cortex-A5, targeting high-performance audio, industrial, and embedded applications. Also covers the related SC582/SC583/SC584/SC587 variants and ADSP-21583/21584/21587 DSP-only parts.

## What's Inside

The skill bundles the official documentation set, converted to markdown for fast in-context access:

| Document | Contents |
|----------|----------|
| **Data Sheet** | Electrical specs, timing, pinout, 349-ball and 529-ball BGA assignments, feature comparison across SC582/583/584/587/589 and 21583/21584/21587 |
| **Hardware Reference Manual** (60 chapters) | Every peripheral and system block: SPORT, SPI, UART, I2C, CAN, USB, Ethernet (EMAC), PCIe (SC589 only), DMA, clocks/PLL, GPIO, timers, PWM, RTC, watchdog, DDR3 controller, static memory controller, audio (DAI/SRU, ASRC, S/PDIF, SINC, PCG), eMMC (MSI), security (TrustZone, crypto, TRNG), FIR/IIR/FFT/HAE accelerators, boot ROM, system crossbars, debug/trace, and more |
| **5 Engineer-to-Engineer Notes** | Migration from legacy SHARC, dual-SHARC audio talkthrough (MCAPI/MDMA), power estimation, Linux SHARC+ loader, cache usage guide |
| **Silicon Anomaly List** | All 77 known errata for Rev 0.1/1.0/1.2 silicon with workarounds |

Total: **67 files** of processor documentation.

## How It Works

The skill uses a **two-phase lookup pattern** to efficiently navigate the large documentation set without flooding the context window:

1. **Index lookup** -- Claude first reads `DOCUMENTATION_INDEX.md`, a master topic-to-file map covering ~60 topics with exact file paths and line ranges
2. **Targeted reading** -- Claude reads only the relevant sections of the right file(s), using offset/limit parameters for large documents like the 9,917-line register list or the 7,121-line Ethernet chapter

The skill also embeds a **silicon anomaly awareness table** directly in its instructions, so Claude proactively warns you about known errata when you're working with affected subsystems (SPI slave select bug, DMC init state machine, SPORT multichannel data pins, F0 pipeline stalls, SPDIF Rx clock, etc.).

## What You Can Ask

Pretty much anything about SC589/SC58x development:

- *"How do I configure SPORT0 for I2S stereo input on the SC589?"*
- *"Write the CGU init code for the ADSP-SC589"*
- *"Are there silicon errata I should know about for the SPI peripheral?"*
- *"How do I set up the DMC for DDR3 on the SC589?"*
- *"Show me the register setup for the DAI/SRU to route a SPORT clock"*
- *"What's the boot sequence when booting from SPI flash?"*
- *"How do I configure the FIR accelerator for a 256-tap filter?"*
- *"What does the PCIe anomaly 20000070 require?"*
- *"How do I use the MCAPI for dual-SHARC communication?"*
- *"Help me estimate power consumption for my SC589 design"*

Claude will cite specific documents and sections, provide register names with MMR addresses, and flag relevant anomalies automatically.

## Installation

The skill is installed as a **personal skill** (available across all projects):

```
~/.claude/skills/adsp-sc589/
├── SKILL.md                          # Skill instructions
├── DOCUMENTATION_INDEX.md            # Master navigation index
├── ADSP-SC589-DS/                    # Data Sheet
├── ADSP-SC589-HRM/                   # Hardware Reference Manual (60 files)
├── ADSP-SC589-EE-Notes/              # Engineer-to-Engineer Notes (5 files)
└── ADSP-SC589-Anomaly/               # Silicon Anomaly List
```

## Usage

The skill **auto-triggers** whenever you mention ADSP, SC582, SC583, SC584, SC587, SC589, 21583, 21584, 21587, SC58x, 2158x, SHARC+, or any SC58x peripheral by name. You can also invoke it explicitly:

```
/adsp-sc589
```

## Processor Overview

| Feature | SC582 | SC583 | SC584 | SC587 | SC589 |
|---------|-------|-------|-------|-------|-------|
| SHARC+ cores | 1 | 2 | 1 | 2 | 2 |
| ARM Cortex-A5 | Yes | Yes | Yes | Yes | Yes |
| PCIe | — | — | — | — | Yes |
| Package | 349-ball BGA | 349-ball BGA | 349-ball BGA | 349-ball BGA | 529-ball BGA |

ADSP-21583/21584/21587 are pin-compatible DSP-only variants (dual SHARC+, no ARM Cortex-A5).

All SC58x processors include: DAI + SRU audio routing, 4-8 SPORTs, ASRC, S/PDIF, PCG, SINC filter, DDR3 DMC, DMA/EMDMA, Ethernet EMAC, SPI, UART, TWI/I2C, CAN, USB 2.0, MSI (eMMC/SD), EPPI, MLB, FIR/IIR/FFT/HAE accelerators, hardware crypto (AES/SHA/RSA/ECC/TRNG), and TrustZone security.

### Key Distinctions

- **ARM Cortex-A5 (Armv7)**: This family uses Cortex-A5, NOT the Cortex-A55 (Armv8) found in SC595/SC596/SC598.
- **DAI audio naming**: Audio signal routing is called **DAI** (Digital Audio Interface), same as SC594, not SHAI.
- **No PRM in this skill**: For SHARC+ core ISA and assembly, use the `adsp-sc594` skill which includes the `sc58x-2158x-prm.md` Programming Reference covering this exact processor family.
