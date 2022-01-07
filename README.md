# Linux kernel - variant from Analog Devices, Inc.

## Table of contents

1. [Description](#description)
2. [How to build](#how-to-build)
3. [Release branches](#release-branches)
4. [Rebased branches](#rebased-branches)
5. [Raspberry Pi branches](#raspberry-pi-branches)
6. [Intel/Altera branches](#intelaltera-branches)

## Description

The Linux kernel in this repository is the [Linux kernel from Xilinx](https://github.com/Xilinx/linux-xlnx) together with drivers & patches applied from Analog Devices.

Details about the drivers that are of interest [and supported] by this repository can be found on the [Analog Devices wiki](https://wiki.analog.com/resources/tools-software/linux-drivers-all). This readme focuses on details specific to how this code is structured/organized, how it was derived, etc.

The current master is based on [xilinx v2021.1](https://github.com/Xilinx/linux-xlnx/tree/xilinx-v2021.1). For details about the merge see commit [67d89797e6b7](https://github.com/analogdevicesinc/linux/commit/67d89797e6b7e313f93f3683f7dd0479895ee9b0) ("Merge tag 'xilinx-v2021.1' of https://github.com/Xilinx/linux-xlnx.git"). In this Xilinx release, the current version of upstream Linux is [Linux 5.10](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tag/?h=v5.10).

For legacy reasons, the [xcomm_zynq](https://github.com/analogdevicesinc/linux/tree/xcomm_zynq) branch is still available and should be in-sync with current master. That branch used to be the old master branch.

## How to build

For build instructions [check the wiki](https://wiki.analog.com/resources/tools-software/linux-drivers-all#building_the_adi_linux_kernel).

## Release branches

All release branches have the [YEAR]\_R[NUM] format. There are some special release branches sometimes (like 2014\_R2\_altera), because it wasn't always possible to match the Linux repo between Xilinx & Intel/Altera.

Each release is matched with a release from the [HDL repo](https://github.com/analogdevicesinc/hdl). The branching name/model for the HDL repo differs a bit from the one in this repo, but the matching should be obvious.
Therefore, each kernel in the release branches is be built using the toolchains from a specific version of Vivado & Quartus.
A matching of these can be found [on the wiki](https://wiki.analog.com/resources/fpga/docs/releases).
Release branches can be built using other GCC toolchains, but in the official SD-card images provided, they will use the toolchains from Vivado/Quartus.

## Rebased branches

Starting with branch [adi-4.9.0](https://github.com/analogdevicesinc/linux/tree/adi-4.9.0) there are rebased branches.
They're typically rebased branches from Xilinx with the ADI patches on top so that it's easier to identify patches that are not yet upstreamed.

For [adi-4.9.0](https://github.com/analogdevicesinc/linux/tree/adi-4.9.0) the base was branch [xlnx_rebase_v4.9](https://github.com/Xilinx/linux-xlnx/tree/xlnx_rebase_v4.9) at commit [d45e196f59364e9f5eafe46027a7d2af349083974](https://github.com/analogdevicesinc/linux/commit/d45e196f59364e9f5eafe46027a7d2af349083974) in the ADI repo and commit [45e196f59364e9f5eafe46027a7d2af349083974](https://github.com/Xilinx/linux-xlnx/commit/45e196f59364e9f5eafe46027a7d2af349083974) in the Xilinx repo. All ADI patches & drivers up to a specific point in time were cherry-picked to that branch from master.
Note that since the `adi-4.9.0` branch is the first rebased branch, it's not particularly the best rebase that could have been done, but it should provide some patches that are somewhat reasonable to take and apply on top of an upstream 4.9 kernel [after some polishing].

The latest rebased branch depends on the current linux version supported in master. At the time of writing it is 5.10 so that [adi-5.10.0](https://github.com/analogdevicesinc/linux/tree/adi-5.10.0) is the latest. Also note that a diff between the latest rebased branch and master (`git diff master adi-5.10.0`) must be NULL.

## Raspberry Pi branches

These provide a kernel that is good to run on a Raspberry Pi board. All the drivers present in the master branch should be automatically cherry-picked into the latest rpi branch.

As in the rebased branches, the latest rpi branch should be in accordance with the current kernel version supported in master. At the time of writing, the kernel version in master is 5.10 so that the correspondent latest rpi branch is [rpi-5.10.y](https://github.com/analogdevicesinc/linux/tree/rpi-5.10.y).

## Intel/Altera branches

Because the kernel versions that Intel/Altera were usually not in sync with Xilinx's, `altera-*` branches were created.
These are [altera_4.0](https://github.com/analogdevicesinc/linux/tree/altera_4.0), [altera_4.4](https://github.com/analogdevicesinc/linux/tree/altera_4.4), [altera_4.6](https://github.com/analogdevicesinc/linux/tree/altera_4.6), [altera_4.9](https://github.com/analogdevicesinc/linux/tree/altera_4.9).

These branches are derived from the [Intel/Altera linux kernel repo](https://github.com/altera-opensource/linux-socfpga), together with some merged versions of old master branches.

The hope is that with the upcoming Linux 4.19, these branches would stop existing, since Intel/Altera seems to keep in sync their kernel version with more recent [non-LTS kernels]. Typically the releases/references that are provided for these boards should already be in the mainline kernel, so these branches should no longer be needed.

