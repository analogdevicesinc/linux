# Linux kernel - variant from Analog Devices, Inc.

## Table of contents

1. [Description](#description)
2. [Release branches](#release-branches)
3. [Rebased branches](#rebased-branches)
4. [Raspberry Pi branches](#raspberry-pi-branches)
5. [Intel/Altera branches](#intelaltera-branches)

## Description

The Linux kernel in this repository is the [Linux kernel from Xilinx](https://github.com/Xilinx/linux-xlnx) together with drivers & patches applied from Analog Devices.

Details about the drivers that are of interest [and supported] by this repository can be found on the [Analog Devices wiki](https://wiki.analog.com/resources/tools-software/linux-drivers-all). This readme focuses on details specific to how this code is structured/organized, how it was derived, etc.

For the current master, the last point in git history where things were merged from Xilinx is commit [dd100de5395b65494a285a37a74ccceab4f39520](https://github.com/analogdevicesinc/linux/commit/dd100de5395b65494a285a37a74ccceab4f39520). That commit (being a working branched) was merged into master via commit [d1873cb62263704a93d904420daf5941f38599b4](https://github.com/analogdevicesinc/linux/commit/d1873cb62263704a93d904420daf5941f38599b4).

For legacy reasons, the [xcomm_zynq](https://github.com/analogdevicesinc/linux/tree/xcomm_zynq) branch is still available and should be in-sync with current master. That branch used to be the old master branch.

The current version of upstream Linux that Xilinx merged into their tree is from [Linux 4.14](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tag/?h=v4.14). No other patches seem to have been added after this (by Xilinx).

## Release branches

All release branches have the [YEAR]\_R[NUM] format. There are some special release branches sometimes (like 2014\_R2\_altera), because it wasn't always possible to match the Linux repo between Xilinx & Intel/Altera.

Each release is matched with a release from the [HDL repo](https://github.com/analogdevicesinc/hdl). The branching name/model for the HDL repo differs a bit from the one in this repo, but the matching should be obvious.
Therefore, each kernel in the release branches is be built using the toolchains from a specific version of Vivado & Quartus.
A matching of these can be found [on the wiki](https://wiki.analog.com/resources/fpga/docs/releases).
Release branches can be built using other GCC toolchains, but in the official SD-card images provided, they will use the toolchains from Vivado/Quartus.

## Rebased branches

Starting with branch [adi-4.9.0](https://github.com/analogdevicesinc/linux/tree/adi-4.9.0) there are rebased branches.
They're typically rebased branches from Xilinx with the ADI patches on top.

For [adi-4.9.0](https://github.com/analogdevicesinc/linux/tree/adi-4.9.0) the base was branch [xlnx_rebase_v4.9](https://github.com/Xilinx/linux-xlnx/tree/xlnx_rebase_v4.9) at commit [d45e196f59364e9f5eafe46027a7d2af349083974](https://github.com/analogdevicesinc/linux/commit/d45e196f59364e9f5eafe46027a7d2af349083974) in the ADI repo and commit [45e196f59364e9f5eafe46027a7d2af349083974](https://github.com/Xilinx/linux-xlnx/commit/45e196f59364e9f5eafe46027a7d2af349083974) in the Xilinx repo. All ADI patches & drivers up to a specific point in time were cherry-picked to that branch from master.
Note that since the `adi-4.9.0` branch is the first rebased branch, it's not particularly the best rebase that could have been done, but it should provide some patches that are somewhat reasonable to take and apply on top of an upstream 4.9 kernel [after some polishing].

The current master branch has an equivalent [adi-4.14.0](https://github.com/analogdevicesinc/linux/tree/adi-4.14.0).
The common base/commit for this branch is commit https://github.com/analogdevicesinc/linux/commit/ad4cd988ba86ab0fb306d57f244b7eaa6cce79a4 from the [xlnx_rebase_v4.14](https://github.com/Xilinx/linux-xlnx/tree/xlnx_rebase_v4.14). Note that the [same hash is present](https://github.com/xilinx/linux-xlnx/commit/ad4cd988ba86ab0fb306d57f244b7eaa6cce79a4) in the Xilinx. As such, the `adi-4.14.0` is an incremental improvement for rebased branches in this repo.

## Raspberry Pi branches

These provide a kernel that is good to run on a Raspberry Pi board. Some of the drivers that are developed internally are prototyped on top of this board.

The typical ones are [rpi-4.0.y](https://github.com/analogdevicesinc/linux/tree/rpi-4.0.y), [rpi-4.9.y](https://github.com/analogdevicesinc/linux/tree/rpi-4.9.y) and [rpi-4.14.y](https://github.com/analogdevicesinc/linux/tree/rpi-4.14.y). The first 2 are upstream RPi branches merged with the ADI master branch from this repo. The 3rd is a rebased version of upstream rpi-4.14.y at some point in time.

These branches are a bit hard to maintain. They require constant conflict resolution whenever rebasing, so they rarely get updated.
As such, [rpi-4.14.y](https://github.com/analogdevicesinc/linux/tree/rpi-4.14.y) will probably be the last RPi branch. The idea is that with the next stable Linux kernel (4.19) support for RPi boards should be good enough in the upstream kernel, so that there won't be a need to keep these special branches.

## Intel/Altera branches

Because the kernel versions that Intel/Altera were usually not in sync with Xilinx's, `altera-*` branches were created.
These are [altera_4.0](https://github.com/analogdevicesinc/linux/tree/altera_4.0), [altera_4.4](https://github.com/analogdevicesinc/linux/tree/altera_4.4), [altera_4.6](https://github.com/analogdevicesinc/linux/tree/altera_4.6), [altera_4.9](https://github.com/analogdevicesinc/linux/tree/altera_4.9).

These branches are derived from the [Intel/Altera linux kernel repo](https://github.com/altera-opensource/linux-socfpga), together with some merged versions of old master branches.

The hope is that with the upcoming Linux 4.19, these branches would stop existing, since Intel/Altera seems to keep in sync their kernel version with more recent [non-LTS kernels]. Typically the releases/references that are provided for these boards should already be in the mainline kernel, so these branches should no longer be needed.

