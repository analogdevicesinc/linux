Contribution guidelines
=======================

This page serves the purpose of providing pointers and a brief introduction to
contributing to the Linux Kernel, using the tools available as well as wrappers
maintained by Analog Devices Inc.

.. tip::

   When using :git-linux:`/`, the :doc:`ci` system automates most checks inside
   a container (:git-linux:`ci:container`). See :ref:`interactive-run` for
   interactive usage.

Code checkers
-------------

There are many checkers to catch issues before submitting changes to the Kernel
mailing lists. :git-linux:`ci:ci/build.sh` combines all of the checkers so that
they can be run with one command using a standard configuration. The checkers
supported by build.sh are as follows.

Checkpatch
~~~~~~~~~~

:external+upstream:doc:`dev-tools/checkpatch`
(:git-linux:`scripts/checkpatch.pl`) is a perl script which checks for trivial
style violations in patches and optionally corrects them. Checkpatch can also
be run on file contexts and without the kernel tree.

It is the bare-minimum tool before submitting any patch series.

Usage:

.. code:: bash

   ./scripts/checkpatch.pl [OPTION]... [FILE]...

.. important::

   You must always adjust the style to match the guidelines of the
   :ref:`subsystem` you are collaborating to.

To run it as a low-budget `lsp server <https://en.wikipedia.org/wiki/Language_Server_Protocol>`__, do:

.. code:: bash

   while true; do \
     scripts/checkpatch.pl --color=always drivers/power/supply/max77928_charger.c  \
                           --strict \
                           --ignore FILE_PATH_CHANGES \
                           --ignore LONG_LINE \
                           --ignore LONG_LINE_STRING \
                           --ignore LONG_LINE_COMMENT \
                           --ignore PARENTHESIS_ALIGNMENT \
                           --ignore CAMELCASE \
                           --ignore UNDOCUMENTED_DT_STRING \
                           --strict \ > /tmp/output ; clear ; cat /tmp/output ; \
   done

Sparse and smatch
~~~~~~~~~~~~~~~~~

:external+upstream:doc:`dev-tools/sparse` is a semantic checker for C programs;
it can be used to find a number of potential problems with kernel code.
Usage:

.. shell::

   ~/linux
   $make C=1

And `Smatch <https://smatch.sourceforge.net/>`__ is a static analysis tool for
C mostly for the linux kernel.
Usage:

.. shell::

   ~/linux
   $make C=1 CHECK="smatch -p=kernel"

Further reading: `Finding locking bugs with Smatch <https://lwn.net/Articles/1023646/>`__

GCC fanalyzer
~~~~~~~~~~~~~

GCC's
`-fanalyzer <https://gcc.gnu.org/onlinedocs/gcc/Static-Analyzer-Options.html#index-analyzer>`__
enables an static analysis of program flow which looks for "interesting"
interprocedural paths through the code, and issues warnings for problems found
on them.

Since it is a flag, it must be appended to the compile command, either to the
`KCFLAGS`:

.. shell::

   ~/linux
   $make KCFLAGS=" -fanalyzer"

To analyze a single file, generate compile commands with
`./scripts/clang-tools/gen_compile_commands.py`, extract the compile command
for the ``.c`` file and append ``-fanalyzer``.

Clang static analyzer
~~~~~~~~~~~~~~~~~~~~~

`Clang static analyzer <https://clang-analyzer.llvm.org/>`__  is a source code
analysis tool that finds bugs in C, C++, and Objective-C programs.

Since it is a flag, it must be appended to the compile command, either to the
`KCFLAGS`:

.. shell::

   ~/linux
   $make LLVM=1 KCFLAGS=" --analyze -Xanalyzer"

To analyze a single file, generate compile commands with
`./scripts/clang-tools/gen_compile_commands.py`, extract the compile command
for the ``.c`` file and append ``--analyze -Xanalyzer``.

Devicetree
----------

The "Open Firmware Device Tree", or simply
:external+upstream:doc:`Devicetree <devicetree/usage-model>` (DT), is a data
structure and language for describing hardware. More specifically, it is a
description of hardware that is readable by an operating system so that the
operating system doesn’t need to hard code details of the machine.

Even though some devicetrees are provided with the Linux Kernel, in general,
a custom devicetree will need to be written to describe a specific board or
device, using the protopytes provided by the
:git-linux:`Documentation/devicetree/bindings/**/*.yaml <Documentation/devicetree/bindings>` files.

When submitting dt-bindings, you must check:

.. shell::

   ~/linux
   $make dt_binding_check CONFIG_DTC=y DT_CHECKER_FLAGS=-m  DT_SCHEMA_FILES="./path/to/.yaml"

For warnings and erros and resolve accordingly.

.. _b4:

B4
--

:external+b4:doc:`B4 <index>` is a tool created to make it easier for project
developers and maintainers to use a distributed development workflow that
relies on patches and distribution lists for code contributions and review.

Take some time to try it out, and understand how it simplies many tasks.
B4 tools is not currently leveraged by continuous integration, and you
must run it locally.

The section that you will most interested in is the
:external+b4:doc:`contributor/overview`, where the contributor workflow is
extensively detailed, as well as the tools to ease it, such as
:external+b4:doc:`b4 prep <contributor/prep>`,
:external+b4:doc:`b4 send <contributor/send>`, and
:external+b4:doc:`b4 trailers <contributor/trailers>`.

.. _subsystem:

Subsytems
---------

The Linux kernel is organized into subsystems—logical divisions around
functionality such as core APIs (memory, scheduling, locking, timers), driver
interfaces (networking, storage, input, etc.), and various device-oriented
modules (IIO, USB, SPI, etc.). Each subsystem encapsulates its own APIs,
conventions, and lifecycle, helping maintain modularity and clarity. For an
up-to-date map of these subsystems and their interfaces, see
:external+upstream:doc:`subsystem-apis`.

When developing for a particular subsystem, look for the appropriate git tree
in the :git-linux:`MAINTAINERS` file to work on. Development branches may be
force pushed. It is reasonable to base you work on top of the current latest
tag, such as `v6.19-rc1
<https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?h=v6.19-rc1&id=8f0b4cce4481fb22653697cced8d0d04027cb1e8>`__
or near it, this avoids unnecessary merge commits when pulling changes.

For some subsystems, CI/CD is automatically applied to developemnt branches as
described at :ref:`ci`.

IIO Subsytem
~~~~~~~~~~~~

The :external+upstream:doc:`Industrial I/O subsystem <driver-api/iio/intro>` is
intended to provide support for devices that in some senses are analog to
digital or digital to analog converters (ADCs, DACs). Devices that fall into
this category are: ADCs, accelerometers, gyros, IMUs, capacitance to digital
converters, pressure sensors, light and proximity sensors, temperature sensors,
magnetometers, DACs, DDS (Direct Digital Synthesis), variable/programmable gain
amplifiers (VGA, PGA). These devices typically would be connected via SPI or
I2C.

The overall aim is to fill the gap between the somewhat similar hwmon and input
subsystems. Hwmon is very much directed at low sample rate sensors used in
applications such as fan speed control and temperature measurement.

To continuous capture data based on a trigger source,
:external+upstream:doc:`iio/iio_devbuf` are used.
