# ADI Linux tree

ADI maintains Linux support for three categories:

- AMD/Xilinx and Raspberry Pi
    - ADI devices and peripherals that are integrated with AMD/Xilinx FPGAs as
      well as Raspberry Pi boards
    - Open pull requests against `main` (soon to be migrated to `xlnx-main`)
    - Changes are ported to branches are prefixed with `rpi-`
- ADI Digital Signal Processors (ADSP)
    - See [Digital Signal Processors
      (ADSP)](https://analogdevicesinc.github.io/documentation/products/adsp/index.html)
      in the System Level Documentation for more information
    - Branches are prefixed with `adsp-` and include a mainline LTS kernel
      version (e.g. `adsp-6.12.0-y`, `adsp-6.12.38-y`). Open pull requests
      against the latest branch with a release tag
- ADI RadioVerse Open Radio Acess Network (O-RAN)
    - The ADRV906x 5G base station on a chip
    - Migration to this repository is currently in progress

The Xilinx branches are based on
[linux-xlnx](https://github.com/Xilinx/linux-xlnx) releases, while all other
categories use [stable mainline LTS releases](https://www.kernel.org/).

## CI/CD

This orphan branch is used to maintain Github reusable workflows to be used by
development branches. Since development branches are designed to contain
changes for upstream projects it is better to keep ADI specific changes (like
CI/CD workflows) as minimal as possible on those branches. This also reduces
the need to synchronize CI/CD changes between development branches.
