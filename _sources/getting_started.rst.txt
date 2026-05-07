.. _getting_started:

Getting Started
===============

Description
-----------

The Linux kernel in this repository is the
`Linux kernel from Xilinx <https://github.com/Xilinx/linux-xlnx>`__
together with drivers & patches applied from Analog Devices.

The driver documentation that are supported by this repository
can be found at :ref:`drivers`.
This section focuses on details specific to how this code is
structured/organized, how it was derived, etc.

The current main is based on `xilinx v2025.1 <https://github.com/Xilinx/linux-xlnx/tree/xilinx-v2025.1>`__.
For details about the merge see commit
:git+linux:`3b1f15dc5c4d <commit/3b1f15dc5c4de92663059705d6f1cb6fc87d4470+>`
(Merge tag ``xilinx-v2025.1`` of https://github.com/Xilinx/linux-xlnx.git).
In this Xilinx release, the current version of upstream Linux is
`Linux 6.12 <https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tag/?h=v6.12>`__.

How to build
------------

For generic instructions on how to build the Linux Kernel, see
:external+upstream:doc:`admin-guide/quickly-build-trimmed-linux`.

For specific targets broadly used by us:

- :external+documentation:ref:`linux-kernel zynq`
- :external+documentation:ref:`linux-kernel zynqmp`
- :external+documentation:ref:`linux-kernel microblaze`
- `Build Raspberry Pi Linux Kernel <https://www.raspberrypi.com/documentation/computers/linux_kernel.html>`__

From the instructions above, you adjust the defconfig, devicetrees of interest,
for example, for ``bcm2712_defconfig`` with all ADI's drivers use
``adi_bcm2712_defconfig``.

.. tip::

   Instead of building all ADI drivers, learn about how to use ``menuconfig`` at
   :external+upstream:doc:`kbuild/kconfig` to enable only the drivers of interest.

The CI/CD scripts at :git+linux:`ci/build.sh` and
:git+linux:`.github/workflows/build.yml` provide detailed steps for all
supported targets. Checkout :ref:`ci` for how to build with CI/CD pipelines.

.. collapsible:: Download built artifacts from our CI/CD

   Built artifacts from our CI/CD are stored at
   `adi/linux (S3) <https://broadcasts.cloudsmith.com/adi/linux>`__,
   you can batch download by git sha or ref:

   .. shell::

      $ cd $(mktemp -d)
      /tmp/tmp.XXX
      $ mkdir ci
      $ curl https://raw.githubusercontent.com/analogdevicesinc/linux/refs/heads/ci/ci/lib.sh \
          -o ci/lib.sh
      $ curl https://raw.githubusercontent.com/analogdevicesinc/linux/refs/heads/ci/ci/download_artifacts.sh \
          -o ci/download_artifacts.sh

      $ source ci/download_artifacts.sh
      $ download_artifacts 494d15a4f
        [step]: Get artifacts
        ...

If you are looking for building or using our Linux distribution **Kuiper**,
please see :external+kuiper:doc:`index` and its
:external+kuiper:ref:`build-flow`.

Release branches
----------------

All release branches have the ``[YEAR]_R[NUM]`` format. There are some special
release branches sometimes (like ``2014_R2_altera``), because it wasn't always
possible to match the Linux repo between Xilinx & Intel/Altera.

Each release is matched with a release from the :git+hdl:`HDL repo </>`. The
branching name/model for the HDL repo differs a bit from the one in this repo,
but the matching should be obvious. Therefore, each kernel in the release
branches is being built using the toolchains from a specific version of Vivado &
Quartus. A matching of these can be found at :external+hdl:ref:`releases`.
Release branches can be built using other GCC toolchains, but in the official
SD-card images provided, they will use the toolchains from Vivado/Quartus.

Rebased branches
----------------

Starting with :git+linux:`adi-4.9.0:` there are rebased branches. They're
typically rebased branches from Xilinx with the ADI patches on top so that it's
easier to identify patches that are not yet upstreamed.

For :git+linux:`adi-4.9.0:` the base was branch
`xlnx_rebase_v4.9 <https://github.com/Xilinx/linux-xlnx/tree/xlnx_rebase_v4.9>`__
at commit
:git+linux:`e5c22c2179cf <main:commit/e5c22c2179cfbec584d2c540d40a0c3d7a20770c+>`
in the ADI repo and commit
`45e196f59364 <https://github.com/Xilinx/linux-xlnx/commit/45e196f59364e9f5eafe46027a7d2af349083974>`__
in the Xilinx repo. All ADI patches & drivers up to a specific point in time
were cherry-picked to that branch from master. Note that since the
``adi-4.9.0`` branch is the first rebased branch, it's not particularly the
best rebase that could have been done, but it should provide some patches that
are somewhat reasonable to take and apply on top of an upstream 4.9 kernel
after some polishing.

The latest rebased branch depends on the current linux version supported in
master. Also note that a diff between the latest rebased branch and `xlnx-main`
(e.g., ``git diff xlnx-main adi-6.12.0``) must be NULL.

Raspberry Pi branches
---------------------

These provide a kernel that is good to run on a Raspberry Pi board. All the
drivers present in the master branch should be automatically cherry-picked into
the latest rpi branch.

As in the rebased branches, the latest rpi branch should be in accordance with
the current kernel version supported in master.

Intel/Altera branches
---------------------

Because the kernel versions that Intel/Altera were usually not in sync with
Xilinx's, ``altera-*`` branches were created:

- :git+linux:`altera_4.0 <altera_4.0:>`
- :git+linux:`altera_4.4 <altera_4.4:>`
- :git+linux:`altera_4.6 <altera_4.6:>`
- :git+linux:`altera_4.9 <altera_4.9:>`
- :git+linux:`altera_4.14 <altera_4.14:>`

These branches are derived from the
`Intel/Altera linux kernel repo <https://github.com/altera-opensource/linux-socfpga>`__,
together with some merged versions of old master branches.

These branches would stop existing, since Intel/Altera seems to keep in sync
their kernel version with more recent non-LTS kernels. Typically, the
releases/references that are provided for these boards should already be in the
mainline kernel, so these branches should no longer be needed.
