# ADSP-SC84x Skill for Claude Code

A Claude Code skill that turns Claude into a knowledgeable development assistant for **Analog Devices ADSP-21844/21846/SC844/SC846 SHARC-FX processors** -- high performance DSPs with a 256-bit SIMD vector core running up to 1.2 GHz (28.8 GFLOPS), optional dual ARM Cortex-A55, targeting automotive audio, ANC/RNC, digital cockpit, conferencing, and professional audio applications.

## What's Inside

The skill bundles the available official documentation set, converted to markdown for fast in-context access:

| Document | Contents |
|----------|----------|
| **Data Sheet** (Preliminary PrD) | Feature overview, processor variant comparison, memory map, pin mux tables, power domains, package outline, electrical/timing specifications, ordering guide |
| **Silicon Anomaly List** (Rev A) | Rev 0.1 errata and workarounds for SPU/SMPU, GP Timer, SPI, S/PDIF, L1 IRAM/MEPU, Boot ROM, USB, and eMSI/eMMC boot |
| **Hardware Reference Manual** (46 chapters) | Every peripheral and system block: CGU, CDU, Frac-N PLL, DPM, RCU, SEC, TRU, L2 memory, SMPU, GPIO, TMU, HADC, CAN FD, WDOG, Link Port, PDM, SPI, xSPI (Octal/HyperBus), UART, Timers, PWM, Counter, MLB, TWI, EMAC (GbE + AVB + PTP), DAI/SRU/DRU, SPORT, PCG, ASRC, S/PDIF, DMA, EMDMA, CRC, SPU, crypto (PKTE/PKA/PKIC/TRNG), FIR/IIR accelerators, Boot ROM, MEPU, and complete register list |
| **SVD Register Maps** | CMSIS-SVD peripheral register definitions for SC84x (174 peripherals) and SC84xW (176 peripherals) -- bit fields, offsets, reset values, access types |

Total: **51 files, ~208 MB** of processor documentation.

## How It Works

The skill uses a **two-phase lookup pattern** to efficiently navigate the documentation set without flooding the context window:

1. **Index lookup** -- Claude first reads `DOCUMENTATION_INDEX.md`, a master topic-to-file map covering ~50 topics with exact file paths and line ranges
2. **Targeted reading** -- Claude reads only the relevant sections of the right file(s), using offset/limit parameters for large documents like the 44K-line Register List

## What You Can Ask

Pretty much anything about SC84x development:

- *"How do I configure SPORT0 for I2S stereo input on the SC846?"*
- *"Write the CGU init code to set CCLK to 1.2 GHz"*
- *"How does the Frac-N PLL generate a 24.576 MHz audio clock from a 25 MHz crystal?"*
- *"Show me the DAI SRU routing for connecting PCG0 to SPORT0"*
- *"What are the LPDDR4 memory map address ranges?"*
- *"How do I configure xSPI for octal DDR mode with XIP?"*
- *"What are the differences between ADSP-21846 and ADSP-SC846?"*
- *"Help me set up CAN FD at 8 Mbps data rate"*
- *"How does the PWM module generate center-aligned waveforms?"*
- *"Review my SPI init code for potential issues"*

Claude will cite specific documents and sections, provide register names with MMR addresses, and flag relevant silicon anomalies when your question touches affected subsystems.

## Installation

The skill is installed as a **personal skill** (available across all projects):

```
~/.claude/skills/adsp-sc846/
├── SKILL.md                          # Skill instructions
├── DOCUMENTATION_INDEX.md            # Master navigation index
├── ADSP-SC84x-DS/                    # Data Sheet (Preliminary PrD; PrB retained for comparison)
├── ADSP-SC84x-Anomaly/               # Silicon Anomaly List
├── ADSP-SC84x-HRM/                   # Hardware Reference Manual (46 files)
├── ADSP-SC84x.svd                    # SVD Register Map (SC84x, 174 peripherals)
└── ADSP-SC84xW.svd                   # SVD Register Map (SC84xW, 176 peripherals)
```

## Usage

The skill **auto-triggers** whenever you mention ADSP, SC844, SC846, 21844, 21846, SHARC-FX, or any SC84x peripheral by name. You can also invoke it explicitly:

```
/adsp-sc846
```

## Processor Overview

| Feature | ADSP-21844 | ADSP-21846 | ADSP-SC844 | ADSP-SC846 |
|---------|-----------|-----------|-----------|-----------|
| Type | DSP Only | DSP Only | DSP + ARM | DSP + ARM |
| SHARC-FX core | 600/800 MHz | 1000/1200 MHz | 600/800 MHz | 1000/1200 MHz |
| ARM Cortex-A55 | -- | -- | Dual, 1200 MHz | Dual, 1200 MHz |
| Peak GFLOPS | 9.6/12.8 | 16/19.2 | 9.6/12.8 | 16/19.2 |
| L1 D-RAM / I-RAM | 512 / 64 kB | 512 / 64 kB | 512 / 64 kB | 512 / 64 kB |
| L1 D-Cache / I-Cache | 256 / 32 kB | 256 / 32 kB | 256 / 32 kB | 256 / 32 kB |
| L2 SRAM | 2 MB | 4 MB | 2 MB | 4 MB |
| LPDDR4 | 16/32-bit | 16/32-bit | 16/32-bit | 16/32-bit |
| GbE EMAC (AVB+PTP) | -- | -- | 1 | 1 |
| Package | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED |

All processors include: 8 SPORTs, 2 DAI blocks (SRU+DRU), 16 ASRCs, 2 S/PDIF Rx + 1 Tx, 8 PCGs, 2 CAN FD, 3 UART, 6 TWI/I2C, 4 SPI (incl. Quad), 2 xSPI (Octal/HyperBus), 2 Link Ports, 16 GP timers, 8 ePWM outputs, FIR/IIR hardware accelerators, HSM, hardware crypto (AES/SHA/RSA/ECC/TRNG), Frac-N PLL, and 484-ball BGA_ED package.
