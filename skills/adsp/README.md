# ADSP Claude Code Skills

Claude Code skills that turn Claude into a knowledgeable development assistant for **Analog Devices SHARC+ and SHARC-FX DSP processors**. Three skills ship in this repo, covering distinct device families:

| Skill | Processors | DSP core | ARM core | Directory |
|-------|-----------|----------|----------|-----------|
| `adsp-sc59x` | ADSP-SC595 / SC596 / SC598 | SHARC+ @ 1 GHz | Cortex-A55 @ 1.2 GHz | [`skills/adsp-sc59x/`](skills/adsp-sc59x/) |
| `adsp-sc594` | ADSP-SC592 / SC594 (and ADSP-21593 / 21594, no ARM) | SHARC+ @ 1 GHz | Cortex-A5 @ 1 GHz | [`skills/adsp-sc594/`](skills/adsp-sc594/) |
| `adsp-sc846` | ADSP-SC844 / SC846 (and ADSP-21844 / 21846, no ARM) | SHARC-FX @ 1.2 GHz | Dual Cortex-A55 @ 1.2 GHz | [`skills/adsp-sc846/`](skills/adsp-sc846/) |

Each skill bundles the available official documentation set (Data Sheet, Hardware Reference Manual, SVD register maps, and where available: Programming Reference Manual, Engineer-to-Engineer application notes, silicon anomaly list, EZ-KIT SOM board docs, and emulator guide), converted from PDF to markdown for fast in-context access by Claude. The SC84x skill includes the PrD data sheet and Rev A silicon anomaly list.

## Quick Start

```bash
git clone <your-repo-url> adsp-claude-skills
cd adsp-claude-skills
chmod +x install.sh
./install.sh
```

That installs all skills to `~/.claude/skills/`. Claude Code picks them up on the next session.

### Install one at a time

```bash
./install.sh sc59x            # just the SC595/SC596/SC598 skill
./install.sh sc594            # just the SC592/SC594 skill
./install.sh sc846            # just the SC844/SC846 skill
```

### Select the harness to install

By default the installer targets Claude Code. Pass `-a [harness]` to
install into a different agent harness's skills directory. Supported
harnesses:

- `claude` (default) → `~/.claude/skills/`
- `pi` → `~/.pi/agent/skills/`

```bash
./install.sh -a pi            # all skills
./install.sh -a pi sc594      # just one
```

`-a [harness]` composes with every other flag (`--force`, `--uninstall`,
skill selectors).

**pi:** run `/reload` to pick the skills up immediately instead of
restarting. You can also skip a second copy entirely and point pi at an
existing Claude install by adding the directory to its `settings.json`:

```json
{
  "skills": ["~/.claude/skills"]
}
```

### Update / overwrite an existing install

```bash
./install.sh --force          # all
./install.sh --force sc594    # just one
```

### Uninstall

```bash
./install.sh --uninstall           # remove all
./install.sh --uninstall sc594     # remove one
```

### Custom install location

The installer honors a per-harness override if set — `CLAUDE_SKILLS_DIR`
(claude, defaults to `~/.claude/skills`) or `PI_SKILLS_DIR` (pi,
defaults to `~/.pi/agent/skills`):

```bash
CLAUDE_SKILLS_DIR=/tmp/test-skills ./install.sh
PI_SKILLS_DIR=/tmp/test-skills ./install.sh -a pi
```

## How the Skills Work

Each skill uses a **two-phase lookup pattern** to navigate a large documentation set without flooding the context window:

1. **Index lookup** — Claude first reads `DOCUMENTATION_INDEX.md`, a master topic-to-file map with exact file paths and line ranges for ~60 topics
2. **Targeted reading** — Claude reads only the relevant sections of the right file(s), using `offset`/`limit` for large documents (the 22K-line Programming Reference Manual in particular)

Each skill also embeds a **silicon anomaly awareness table** directly in its instructions, so Claude proactively warns about known errata when you're working with affected subsystems.

## What You Can Ask

Pretty much anything about SC59x, SC592/SC594, or SC84x development — firmware, peripherals, register programming, SHARC+/SHARC-FX assembly, DMA, audio (DAI/SRU/SPORT/I2S/TDM/ASRC/SPDIF/PDM), Ethernet, SPI/xSPI/OSPI/UART/I2C/CAN FD/USB, DDR3/LPDDR4 setup, boot, power, pin mux, PWM, board design. Claude will cite specific documents and sections, provide register names with MMR addresses, and flag relevant anomalies automatically.

See each skill's README for topic-by-topic examples:
- [`skills/adsp-sc59x/README.md`](skills/adsp-sc59x/README.md)
- [`skills/adsp-sc594/README.md`](skills/adsp-sc594/README.md)
- [`skills/adsp-sc846/README.md`](skills/adsp-sc846/README.md)

## Repository Layout

```
adsp-claude-skills/
├── README.md                        # This file
├── LICENSE                          # MIT — applies to skill scaffolding & install tooling
├── install.sh                       # Installer (install / --force / --uninstall)
├── .gitignore
├── skills/
│   ├── adsp-sc59x/                  # SC595/SC596/SC598 skill (81 files, ~159 MB)
│   │   ├── SKILL.md
│   │   ├── README.md
│   │   ├── DOCUMENTATION_INDEX.md
│   │   ├── ADSP-SC59x-DS/
│   │   ├── ADSP-SC59x-HRM/          # 57 chapters
│   │   ├── ADSP-SC59x-PRM/
│   │   ├── ADSP-SC59x-EE-Notes/     # 14 application notes
│   │   ├── ADSP-SC59x-Anomaly/
│   │   ├── ev-sc598-som_manual.md
│   │   ├── ev-sc598-som-schematic.md
│   │   ├── mb_ice1500_emu_ug.md
│   │   └── bp-400-3.md
│   ├── adsp-sc594/                  # SC592/SC594 skill (76 files, ~63 MB)
│   │   ├── SKILL.md
│   │   ├── README.md
│   │   ├── DOCUMENTATION_INDEX.md
│   │   ├── ADSP-SC594-DS/
│   │   ├── ADSP-SC594-HRM/          # 59 chapters
│   │   ├── ADSP-SC594-PRM/
│   │   ├── ADSP-SC594-EE-Notes/     # 11 application notes
│   │   ├── ADSP-SC594-Anomaly/
│   │   ├── ev-sc594-som_manual.md
│   │   ├── ev-sc594-som-schematic.md
│   │   └── mb_ice1500_emu_ug.md
│   └── adsp-sc846/                  # SC844/SC846 skill (51 files, ~208 MB)
│       ├── SKILL.md
│       ├── README.md
│       ├── DOCUMENTATION_INDEX.md
│       ├── ADSP-SC84x-DS/           # Preliminary data sheet (PrD preferred, PrB legacy)
│       ├── ADSP-SC84x-Anomaly/      # Silicon anomaly list
│       ├── ADSP-SC84x-HRM/          # 46 chapters
│       ├── ADSP-SC84x.svd           # SVD register map (174 peripherals)
│       └── ADSP-SC84xW.svd          # SVD register map (176 peripherals)
└── tools/                           # Build tooling (not installed, for regeneration only)
    ├── pdf_to_md_chunked.py         # PDF → markdown converter (docling-based)
    ├── merge_chapter_splits.sh      # Merge multi-part HRM chapter PDFs
    ├── merge_chapter_splits_md.sh   # Merge multi-part HRM chapter markdown
    ├── docling.md                   # Notes on docling usage
    └── source_urls.txt              # Canonical URLs for all source PDFs on analog.com
```

The `pdf/` subdirectories inside each skill's HRM folder contain the raw source PDFs from Analog Devices. They are **gitignored** — the markdown is the source of truth for Claude, and the PDFs can be re-fetched from `tools/source_urls.txt` if regeneration is needed.

## Regenerating the Markdown

If Analog Devices publishes a new revision, you can rebuild a skill from fresh PDFs:

1. Fetch PDFs listed in `tools/source_urls.txt` into the appropriate `ADSP-*-HRM/pdf/` (or other) directories
2. Run `tools/pdf_to_md_chunked.py` to convert via [docling](https://github.com/docling-project/docling)
3. If the HRM is split into multi-part chapter PDFs, run `tools/merge_chapter_splits.sh` (or the `_md` variant) first
4. Update `DOCUMENTATION_INDEX.md` line ranges and the anomaly table in `SKILL.md` if content changed

## Licensing

- **Skill scaffolding, installer, and tools in this repo**: MIT license (see `LICENSE`)
- **Documentation content under `skills/*/ADSP-*/`, board manuals, schematics, and emulator guides**: © Analog Devices, Inc. Redistribution terms are governed by Analog Devices. This repository is intended for **private use only** — do not make it public without verifying ADI's redistribution policy.

## Processor Overview

### adsp-sc59x (SC595/SC596/SC598)

| Feature | SC596 | SC598 |
|---------|-------|-------|
| SHARC+ cores | 1 @ 1 GHz | 2 @ 812.5-1000 MHz |
| ARM Cortex-A55 | 1.2 GHz | 1.2 GHz |
| L1 SRAM | 1 MB | 2 × 1 MB |
| L2 shared SRAM | 2 MB | 2 MB |
| DDR3/DDR3L | 16-bit | 16-bit |
| eMMC/SD (eMSI) | Yes | Yes |
| Package | 400-ball BGA | 400-ball BGA |

### adsp-sc594 (SC592/SC594 + 21593/21594)

| Feature | SC592 | SC594 | 21593 | 21594 |
|---------|-------|-------|-------|-------|
| SHARC+ cores | 1 @ 1 GHz | 2 @ 800-1000 MHz | 2 @ 800-1000 MHz | 2 @ 1 GHz |
| ARM Cortex-A5 | 1 GHz | 800-1000 MHz | — | — |
| L1 SRAM | 640 kB | 2 × 640 kB | 2 × 640 kB | 2 × 640 kB |
| L2 shared SRAM | 2 MB | 2 MB | 2 MB | 2 MB |
| Package | HPC BGA | HPC BGA | LPC BGA | HPC BGA |

### adsp-sc846 (SC844/SC846 + 21844/21846)

| Feature | ADSP-21844 | ADSP-21846 | ADSP-SC844 | ADSP-SC846 |
|---------|-----------|-----------|-----------|-----------|
| SHARC-FX core | 600/800 MHz | 1000/1200 MHz | 600/800 MHz | 1000/1200 MHz |
| ARM Cortex-A55 | -- | -- | Dual, 1200 MHz | Dual, 1200 MHz |
| L1 D-RAM / I-RAM | 512 / 64 kB | 512 / 64 kB | 512 / 64 kB | 512 / 64 kB |
| L2 SRAM | 2 MB | 4 MB | 2 MB | 4 MB |
| LPDDR4 | 16/32-bit | 16/32-bit | 16/32-bit | 16/32-bit |
| Package | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED | 484-ball BGA_ED |

All three families share common peripherals: SPORT, ASRC, S/PDIF, PCG, PDM, EMAC, CAN FD, UART, SPI, TWI/I2C, GP timers, FIR/IIR accelerators, hardware crypto, TrustZone. The SC84x family adds SHARC-FX (256-bit SIMD), xSPI (Octal/HyperBus), PWM, Frac-N PLL, HSM, DAI routing (SRU/DRU), and LPDDR4 support. See each skill's `README.md` for full comparison.
