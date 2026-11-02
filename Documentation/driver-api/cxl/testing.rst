.. SPDX-License-Identifier: GPL-2.0

===========
Testing CXL
===========

CXL testing uses two complementary frameworks: cxl_test exercises the driver
stack against an emulated CXL topology, while KUnit tests self-contained logic
in isolation. Changes should preserve existing test coverage and extend the
framework appropriate to the behavior being changed.

cxl_test
========

The cxl_test modules, built from tools/testing/cxl/, emulate a CXL topology with
host bridges, switches, memory devices, and mailboxes. This allows the CXL driver
stack to be tested without hardware. The tests are maintained in the ndctl repo
at https://github.com/pmem/ndctl, under test/cxl-<name>.sh.

Use cxl_test for behavior that depends on a live topology: port and endpoint
enumeration, region creation and assembly, decoder commit, and anything that
crosses the sysfs ABI.

Build the cxl_test modules from the top of the kernel tree::

  make M=tools/testing/cxl

After installing the modules for the running kernel, run the CXL suite from an
ndctl build::

  meson test -C build --suite cxl

For more on building and running the ndctl unit tests, see:

- the Unit Tests section of the ndctl README,
  https://github.com/pmem/ndctl?tab=readme-ov-file#unit-tests
- the ndctl test runner,
  https://github.com/pmem/ndctl-test-runner?tab=readme-ov-file#ndctl-test-runner


KUnit
=====

KUnit tests run self-contained logic in the kernel without any CXL topology.
KUnit is suited to logic that can be exercised with controlled test inputs,
including encode/decode helpers, address translation and interleave calculations,
parsers of device- or firmware-supplied data, and mailbox command handling
through a stub transport.

To run all of the CXL KUnit tests, from the top of the kernel tree::

  ./tools/testing/kunit/kunit.py run --kunitconfig=drivers/cxl

This builds a test kernel in .kunit/ using drivers/cxl/.kunitconfig and runs it
under User Mode Linux (UML). The running kernel is not modified, and no
installation is required. The test kernel runs the tests, reports the results,
and exits.

Use --arch=<arch> to run the tests under QEMU instead.
Use --arch=help to list supported architectures.

Running during development
--------------------------

kunit.py builds in .kunit/, but kbuild does not allow an out-of-tree build when
the source tree already contains an in-tree build. If you build the kernel
in-tree, use a separate worktree for KUnit. The worktree contains the committed
sources at the selected revision::

  git worktree add ../linux-kunit HEAD
  cd ../linux-kunit
  ./tools/testing/kunit/kunit.py run --kunitconfig=drivers/cxl

To run a single suite, name it::

  ./tools/testing/kunit/kunit.py run --kunitconfig=drivers/cxl cxl_core_mbox

Enable KASAN when testing memory safety error paths. It can detect an out-of-bounds
access even when the resulting value does not cause a test assertion to fail. Run
under QEMU, for example x86_64::

  ./tools/testing/kunit/kunit.py run --kunitconfig=drivers/cxl \
      --arch=x86_64 --kconfig_add CONFIG_KASAN=y

Maintaining KUnit tests
-----------------------

Use loop bounds and table sizes from the code under test, such as ARRAY_SIZE()
of a lookup table, so tests adapt as the code evolves. Derive expected results
from the specification, not the implementation, so tests can detect unintended
behavior changes.

When a patch changes behavior covered by a test, update the test in the same
patch. For example, add newly supported encodings to a valid-value table or
adjust the expected errno for a new return code.

A failing test may indicate a regression or a test that needs updating. Run the
CXL KUnit tests throughout development, not just before posting, to catch either
case while making changes.

Adding a KUnit test
-------------------

A CXL KUnit test follows these rules:

- Place the test in a tests/ directory beside the code under test, in a
  file named <name>_kunit.c.

- Derive the module and suite names from the path below drivers/cxl/, excluding
  tests/. For example, drivers/cxl/core/tests/port_kunit.c produces module
  cxl_core_port_kunit and suite cxl_core_port, while drivers/cxl/tests/port_kunit.c
  produces module cxl_port_kunit and suite cxl_port.

- Build new suites under CONFIG_CXL_KUNIT_TEST rather than introducing a separate
  Kconfig option for each suite. Individual suites can be selected by name with
  kunit.py or by loading their module.

- If the code under test depends on another configuration option, such as
  CONFIG_CXL_REGION, make the test build conditional on that option in
  tests/Makefile and enable it in drivers/cxl/.kunitconfig.

- Use exported interfaces where possible, importing the CXL symbol namespace as
  needed.

For an example of testing mailbox command handling with a stub transport, see
drivers/cxl/core/tests/mbox_kunit.c.

When to add a test
==================

New or modified self-contained logic should be accompanied by KUnit coverage where
practical. Extend an existing suite when possible, or introduce a new one for
functionality that is not yet covered.

Bug fixes should include regression coverage that reproduces the problem without
the fix.

Changes to topology-dependent behavior should be covered by cxl_test tests in ndctl.
KUnit coverage complements, but does not replace, cxl_test.
