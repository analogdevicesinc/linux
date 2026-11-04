.. SPDX-License-Identifier: GPL-2.0

======================================
Kernel API Specification Framework
======================================

:Author: Sasha Levin <sashal@kernel.org>

.. contents:: Table of Contents
   :depth: 3
   :local:

Introduction
============

The Kernel API Specification Framework (KAPI) describes kernel APIs in a
machine-readable form. The descriptions are written as kerneldoc annotations
next to the implementation and compiled into the kernel. They can be used to
check system call arguments at runtime and can be read back through debugfs or
the ``kapi`` tool.

Purpose and Goals
-----------------

The framework aims to:

1. **Improve API Documentation**: Provide structured, inline documentation that
   lives alongside the code and is maintained as part of the development process.

2. **Enable Runtime Validation**: Optionally validate API usage at runtime to catch
   common programming errors during development and testing.

3. **Support Tooling**: Export API specifications in machine-readable formats for
   use by static analyzers, documentation generators, and development tools. See
   `The kapi Tool`_.

4. **Formalize Contracts**: Explicitly document API contracts including parameter
   constraints, execution contexts, locking requirements, and side effects.

Architecture Overview
=====================

Components
----------

The framework consists of several key components:

1. **Core Framework** (``kernel/api/kernel_api_spec.c``)

   - API specification registration and storage
   - Runtime validation engine
   - Specification lookup and querying

2. **DebugFS Interface** (``kernel/api/kapi_debugfs.c``)

   - Runtime introspection via ``/sys/kernel/debug/kapi/``
   - Per-API detailed specification output
   - List of all registered API specifications

3. **kapi Tool** (``tools/kapi/``)

   - Userspace utility for extracting specifications
   - Multiple input sources (source, binary, debugfs)
   - Multiple output formats (plain, JSON, RST)
   - Testing and validation utilities

Data Model
----------

The framework uses a hierarchical data model::

    kernel_api_spec
    ├── Basic Information
    │   ├── name (API function name)
    │   ├── version (specification version)
    │   └── description (human-readable description)
    │
    ├── Parameters (up to 16)
    │   └── kapi_param_spec
    │       ├── name
    │       ├── type (int, pointer, fd, path, etc.)
    │       ├── flags (in, out, inout, optional, etc.)
    │       ├── constraints (range, mask, enum values)
    │       └── description
    │
    ├── Return Value
    │   └── kapi_return_spec
    │       ├── type
    │       ├── success conditions
    │       └── validation rules
    │
    ├── Error Conditions (up to 32)
    │   └── kapi_error_spec
    │       ├── error code
    │       ├── name
    │       ├── condition
    │       └── description
    │
    ├── Execution Context
    │   ├── allowed contexts (process, interrupt, etc.)
    │   ├── locking requirements
    │   └── preemption/interrupt state
    │
    └── Side Effects
        ├── memory allocation
        ├── state changes
        └── signal handling

Usage Guide
===========

Basic API Specification
-----------------------

API specifications are written as KAPI-annotated kerneldoc comments directly in
the source file, immediately preceding the function implementation. With
``CONFIG_KAPI_SPEC`` enabled, Kbuild runs ``kernel-doc -apispec`` on each
built-in C file that has a ``contexts:`` (or ``context-flags:``) line plus at
least one of ``api-type:``, ``param:``, ``error:``, ``capability:``,
``signal:``, ``lock:``, ``state-trans:``, ``constraint:``, ``side-effect:`` or
``long-desc:``, and compiles the generated header into that file. The ``kapi``
tool reads the same annotations, or the specifications in a built kernel, for
use outside the kernel build.

The following is an excerpt of the ``sys_read`` specification in
``fs/read_write.c``, trimmed for length:

.. code-block:: c

    /**
     * sys_read - Read data from a file descriptor
     * @fd: File descriptor to read from
     * @buf: User-space buffer to read data into
     * @count: Maximum number of bytes to read
     *
     * long-desc: Attempts to read up to count bytes from file descriptor fd into
     *   the buffer starting at buf. ...
     *
     * contexts: process, sleepable
     *
     * param: fd
     *   type: fd, input
     *   constraint-type: range(0, INT_MAX)
     *
     * param: buf
     *   type: user_ptr, output
     *   constraint-type: buffer(2)
     *
     * param: count
     *   type: uint, input
     *
     * return:
     *   type: int
     *   check-type: range
     *   success: >= 0
     *   desc: On success, returns the number of bytes read (non-negative). ...
     *
     * error: EBADF, Bad file descriptor
     *   desc: fd is not a valid file descriptor, or fd was not opened for
     *     reading. ...
     */
    SYSCALL_DEFINE3(read, unsigned int, fd, char __user *, buf, size_t, count)

DSL reference:

* ``contexts:`` — comma-separated list of call contexts.  Accepted tokens:
  ``process``, ``softirq``, ``hardirq``, ``nmi``, ``atomic``, ``sleepable``,
  ``preempt_disabled``, ``irq_disabled``.  ``context-flags:`` with
  ``|``-joined ``KAPI_CTX_*`` constants is equivalent.
* ``type:`` — parameter type plus direction/qualifier flags on a single
  line.  Type aliases (case-insensitive): ``int``, ``uint``, ``ptr``,
  ``fd``, ``path``, ``user_ptr`` (or ``uptr``), ``struct``, ``union``,
  ``enum``, ``func_ptr``, ``array``, ``custom``.  Flag aliases:
  ``input``, ``output``, ``inout``, ``user``, ``optional``, ``const``,
  ``volatile``, ``dma``, ``aligned``.
* ``constraint-type:`` — a ``KAPI_CONSTRAINT_*`` enum token or a
  function-call expression.  ``range(lo, hi)``, ``mask(expr)``,
  ``enum(v1, v2, …)``, ``buffer(size_param_idx)``, ``alignment(N)``,
  ``user_string``, ``user_path``, ``user_ptr``, ``power_of_two``,
  ``page_aligned``, ``nonzero``.  ``user_string`` takes its length limits
  from ``range:``.  The function-call
  form populates the matching aux fields
  (``range:`` / ``valid-mask:`` / ``size-param:``).
* ``arch-mask:`` — extends a ``mask(...)`` constraint with bits that
  are valid only on a named architecture.  The form is
  ``arch-mask: <arch> = <bits-expr>`` and may appear multiple times.
  The named arch must be one of the known short names
  (``alpha``, ``arc``, ``arm``, ``arm64``, ``csky``, ``hexagon``,
  ``loongarch``, ``m68k``, ``microblaze``, ``mips``, ``nios2``,
  ``openrisc``, ``parisc``, ``powerpc``, ``riscv``, ``s390``, ``sh``,
  ``sparc``, ``um``, ``x86``, ``xtensa``); the generator translates it
  to the matching ``CONFIG_*`` symbol and emits the bits inside an
  ``#ifdef`` so a single generated apispec.h compiles on every
  architecture and folds in the arch-specific bits at compile time.
  Use this for PROT/MAP bits whose UAPI symbols are defined only on
  the matching arch.
* ``lock: … type:`` accepts ``mutex``, ``spinlock``, ``rwlock``,
  ``seqlock``, ``rcu``, ``semaphore``, ``custom`` or ``KAPI_LOCK_*``.
* ``signal: … direction:`` accepts ``receive``, ``send``, ``handle``,
  ``block``, ``ignore`` (bitmask, joinable with ``|`` or ``,``).
* ``signal: … action:`` accepts ``default``, ``terminate``, ``coredump``,
  ``stop``, ``continue``, ``custom``, ``return``, ``restart``,
  ``queue``, ``discard``, ``transform``.
* ``signal: … timing:`` accepts ``before``, ``during``, ``after``.
* ``capability: … type:`` accepts ``bypass_check``, ``increase_limit``,
  ``override_restriction``, ``grant_permission``, ``modify_behavior``,
  ``access_resource``, ``perform_operation``.
* ``side-effect:`` accepts the snake_case effect names
  (``alloc_memory``, ``free_memory``, ``modify_state``, ``signal_send``,
  ``file_position``, ``lock_acquire``, ``lock_release``,
  ``resource_create``, ``resource_destroy``, ``schedule``, ``hardware``,
  ``network``, ``filesystem``, ``process_state``, ``irreversible``)
  joined with ``|`` — for example ``side-effect: resource_create | alloc_memory``.
* ``return: … type:`` reuses the ``type:`` aliases above.
* ``return: … check-type:`` accepts ``exact``, ``range``,
  ``fd``, ``no_return``.  ``success:``
  gives the success value for ``exact`` (an integer, optionally
  written ``= N``) and the lower bound for ``range`` (``>= N``); the
  other check types do not use it.  The ``type:`` of a return block
  is also kept as written (for example ``int``) in the
  human-readable ``type_name`` of ``struct kapi_return_spec``.
* ``error:`` takes a ``NAME, one-line summary`` header followed
  by optional indented ``desc:`` / ``condition:`` subfields.
* ``lock:`` and ``signal:`` take an indented ``desc:`` subfield
  for the long-form description; ``signal:`` also accepts
  ``number:`` (the signal constant, for example ``SIGPIPE``),
  ``errno:``, ``priority:``, ``restartable:``, ``interruptible:``,
  and ``queue:`` subfields.
* ``state-trans:`` takes ``from:``, ``to:``, ``object:``,
  optional ``condition:``, and ``desc:`` subfields.  The condition is
  stored in its own field, apart from the description.
* ``long-desc:`` is a free-form multi-line prose block that
  populates ``long_description`` in the spec.  ``notes:`` is a
  free-form block of the same kind.  In both, a blank line starts a
  new paragraph, a line starting with ``- `` stays on its own line so
  bullet lists survive, and any other wrapped line is joined to the
  previous one with a space.  ``examples:`` keeps one example per
  line, preserving the relative indentation of nested code.  The
  line breaks are stored as ``\n`` in the generated strings, which
  have no length limit.
* A subfield line inside a block starts with one of the subfield
  names of that block followed by ``:``.  Every other line, even one
  with a colon in the middle of a sentence, continues the previous
  subfield.
* ``param-count:`` is optional; the parser counts ``param:`` blocks and
  warns when an explicit count disagrees.

System Call Specification
-------------------------

System calls are documented inline in the implementation file (e.g., ``fs/open.c``)
using KAPI-annotated kerneldoc comments. When ``CONFIG_KAPI_RUNTIME_CHECKS`` is
enabled, the ``SYSCALL_DEFINEx`` macros automatically look up the specification
and validate parameters before and after the syscall executes.

Runtime Validation
==================

Enabling Validation
-------------------

Runtime validation is controlled by kernel configuration:

1. Enable ``CONFIG_KAPI_SPEC`` to build the framework
2. Enable ``CONFIG_KAPI_RUNTIME_CHECKS`` for runtime validation
3. Optionally enable ``CONFIG_KAPI_SPEC_DEBUGFS`` for debugfs interface

Validation Behavior
-------------------

When ``CONFIG_KAPI_RUNTIME_CHECKS`` is enabled, every system call that has a
specification is validated in its ``SYSCALL_DEFINEx()`` wrapper: the arguments
are checked against the parameter constraints before the handler runs, and the
return value is checked against the return specification afterwards. Violations
are reported via ``pr_warn_ratelimited`` to avoid flooding the kernel log. On
the return side only a successful ``fd`` return that is not a valid file
descriptor is reported. Any other value that does not satisfy the success check
is treated as an error and accepted, and error codes that the specification
does not list are only logged at debug level.
The execution context recorded in a specification is not checked at runtime.
The option is available on x86 and on architectures that use the generic
``__SYSCALL_DEFINEx()``.

.. warning::

   Userspace errno is affected when this option is on. For syscalls that
   violate their parameter specification, KAPI short-circuits the call and
   returns ``-EINVAL`` from the validator **before** the real handler runs.
   That errno can differ from what the real handler would have produced for
   the same condition (for example, ``-ENOMEM`` from an allocation path or
   ``-EFAULT`` from a deeper copy-in). ``CONFIG_KAPI_RUNTIME_CHECKS`` is a
   debug-only option; do not enable it on production kernels or in
   userspace-visible test environments where error-code fidelity matters.

Custom Validators
-----------------

``KAPI_CONSTRAINT_CUSTOM`` calls the ``validate`` function of the parameter
specification. Kerneldoc annotations cannot set it, so it is only available to a
``struct kapi_param_spec`` that is filled in by hand:

.. code-block:: c

    static bool validate_buffer_size(s64 value)
    {
        size_t size = (size_t)value;

        return size > 0 && size <= MAX_BUFFER_SIZE;
    }

    /* In the parameter definition: */
    .constraint_type = KAPI_CONSTRAINT_CUSTOM,
    .validate = validate_buffer_size,

Tracepoints
-----------

When ``CONFIG_KAPI_RUNTIME_CHECKS`` is enabled, the syscall validation path emits
two ftrace tracepoints (in the ``kapi`` trace system) for every syscall that has a
specification:

- ``kapi_syscall_enter`` -- fired before parameter validation, recording the spec
  name, the raw syscall argument values, and -- when the spec provides parameter
  metadata -- a rendered ``name=value`` list: signed integer and file descriptor
  values are shown in decimal, all other values in hex, and an unnamed parameter
  as ``arg``.
- ``kapi_syscall_exit`` -- fired after the handler returns, or in place of the
  handler when parameter validation rejects the call (the handler is skipped and
  ``-EINVAL`` is returned). Records the spec name, the return value, and
  ``spec_match``: 0 when the parameters were rejected or a successful ``fd`` return
  was not a valid file descriptor, and 1 otherwise. Error codes missing from the
  spec do not clear it.

Unlike the ``pr_warn_ratelimited`` violation reports, the tracepoints capture every
spec'd call rather than only violations, and can be filtered with the usual ftrace
facilities. They require ``CONFIG_TRACEPOINTS`` and stay dormant
until enabled::

    # echo 1 > /sys/kernel/tracing/events/kapi/enable
    # cat /sys/kernel/tracing/trace
     ...  kapi_syscall_enter: sys_read(fd=3, buf=0x7ffd46780b58, count=0x340)
     ...  kapi_syscall_exit: sys_read = 832 spec_match=1
     ...  kapi_syscall_enter: sys_open(filename=0x480300, flags=268435456, mode=0x0)
     ...  kapi_syscall_exit: sys_open = -22 spec_match=0

DebugFS Interface
=================

The debugfs interface provides runtime access to API specifications:

Directory Structure
-------------------

::

    /sys/kernel/debug/kapi/
    ├── list                     # Overview of all registered API specs
    ├── specs/                   # Per-API specification files
    │   ├── sys_open             # Human-readable spec for sys_open
    │   ├── sys_close            # Human-readable spec for sys_close
    │   ├── sys_read             # Human-readable spec for sys_read
    │   ├── sys_write            # Human-readable spec for sys_write
    │   └── sys_madvise          # Human-readable spec for sys_madvise
    └── specs-json/              # Machine-readable counterpart of specs/
        ├── sys_open             # JSON spec for sys_open
        └── ...

Usage Examples
--------------

List all available API specifications::

    $ cat /sys/kernel/debug/kapi/list
    Available Kernel API Specifications
    ===================================

    sys_open - Open or create a file
    sys_close - Close a file descriptor
    sys_read - Read data from a file descriptor
    sys_write - Write data to a file descriptor
    sys_madvise - Give advice about use of memory

    Total: 5 specifications

Query specific API::

    $ cat /sys/kernel/debug/kapi/specs/sys_open
    Kernel API Specification
    ========================

    Name: sys_open
    Version: 1
    Description: Open or create a file
    ...

The ``specs-json/`` files carry the complete specification: for each
parameter the type class, flags and the constraint (type, range, valid
mask, enumerated values, alignment, size and the index of the parameter
holding a buffer's size), the return check, errors, locks, signals,
signal masks, side effects, state transitions, capabilities, additional
constraints and structure specifications. Masks and flag words are
hex strings, enumerations are lower-case tokens such as
``"constraint_type": "buffer"``, and ``size_param_idx`` is a 0-based
index into ``parameters`` (``null`` when unused). The ``kapi`` tool
reads these files with ``--debugfs`` and reports the same data as
``--vmlinux``.

Performance Considerations
==========================

Memory Overhead
---------------

Each compiled spec is 26400 bytes (``readelf -sW vmlinux | grep
__kapi_spec_``), dominated by the fixed-bound arrays
``struct_specs[8]`` (11584 bytes), ``signal_masks[32]`` (4864),
``signals[32]`` (3328) and ``params[16]`` (1920). With
the five syscall specs in this series, ``.kapi_specs`` and the backing
``.rodata`` objects occupy ~132 KB. Building with ``CONFIG_KAPI_SPEC=n``
emits no code or data from the framework.

Runtime Overhead
----------------

When ``CONFIG_KAPI_RUNTIME_CHECKS`` is enabled, each validated
call pays for a parameter walk plus the per-constraint check
(range/mask/enum/align/user-ptr/user-path/user-string).
The cost depends on the parameter count and the constraints involved;
profile before enabling on workloads where syscall latency matters.
``CONFIG_KAPI_RUNTIME_CHECKS=n`` compiles the validators away
entirely.

The kapi Tool
=============

Overview
--------

The ``kapi`` tool is a userspace utility that extracts and displays kernel API
specifications from multiple sources. It provides a unified interface to access
API documentation whether from compiled kernels, source code, or runtime systems.

Installation
------------

Build the tool from the kernel source tree::

    $ cd tools/kapi
    $ cargo build --release

    # Optional: Install system-wide
    $ cargo install --path .

The tool requires Rust and Cargo to build. The binary will be available at
``tools/kapi/target/release/kapi``.

Command-Line Usage
------------------

Basic syntax::

    kapi [OPTIONS] [API_NAME]

Options:

- ``--vmlinux <PATH>``: Extract from compiled kernel binary
- ``--source <PATH>``: Extract from kernel source code
- ``--debugfs <PATH>``: Extract from debugfs (default: /sys/kernel/debug)
- ``-f, --format <FORMAT>``: Output format (plain, json, rst)
- ``-h, --help``: Display help information
- ``-V, --version``: Display version information

Input Modes
-----------

**1. Source Code Mode**

Extract specifications directly from kernel source::

    # Scan entire kernel source tree
    $ kapi --source /path/to/linux

    # Extract from specific file
    $ kapi --source fs/open.c

    # Get details for specific API
    $ kapi --source /path/to/linux sys_close

**2. Vmlinux Mode**

Extract from compiled kernel with debug symbols::

    # List all APIs in vmlinux
    $ kapi --vmlinux ./vmlinux

    # Get specific syscall details
    $ kapi --vmlinux ./vmlinux sys_read

**3. Debugfs Mode**

Extract from running kernel via debugfs::

    # Use default debugfs path
    $ kapi

    # Use custom debugfs mount
    $ kapi --debugfs /mnt/debugfs

    # Get specific API from running kernel
    $ kapi sys_write

Output Formats
--------------

The samples below are shortened; ``...`` marks omitted output.

**Plain Text Format** (default)::

    $ kapi --source . sys_read

    Detailed information for sys_read:
    ==================================
    Description: Read data from a file descriptor

    Detailed Description:
      Attempts to read up to count bytes from file descriptor fd into the buffer starting at buf. ...

    Execution Context:
      - KAPI_CTX_PROCESS
      - KAPI_CTX_SLEEPABLE

    Parameters (3):
      [0] fd (unsigned int fd)
          File descriptor to read from
          Flags: IN
          ...

**JSON Format**::

    $ kapi --source . --format json sys_read
    {
      "api_details": {
        "name": "sys_read",
        "description": "Read data from a file descriptor",
        "long_description": "Attempts to read up to count bytes from file descriptor fd into the buffer starting at buf. ...",
        "context_flags": [
          "KAPI_CTX_PROCESS",
          "KAPI_CTX_SLEEPABLE"
        ],
        ...
      }
    }

**ReStructuredText Format**::

    $ kapi --source . --format rst sys_read

    sys_read
    ========

    **Read data from a file descriptor**

    Attempts to read up to count bytes from file descriptor fd into the buffer starting at buf. ...

Usage Examples
--------------

**Generate complete API documentation**::

    # Export all kernel APIs to JSON
    $ kapi --source /path/to/linux --format json > kernel-apis.json

    # Generate RST documentation for all syscalls
    $ kapi --vmlinux ./vmlinux --format rst > syscalls.rst

    # List APIs from specific subsystem
    $ kapi --source fs/

**Integration with other tools**::

    # List the names of all APIs
    $ kapi --format json | jq -r '.apis[].name'

    # Generate markdown documentation
    $ kapi --format rst sys_madvise | pandoc -f rst -t markdown

**Debugging and analysis**::

    # Check if specific API exists
    $ kapi --source . my_custom_api || echo "API not found"

Implementation Details
----------------------

The tool extracts API specifications from three sources:

1. **Source Code**: Parses KAPI-annotated kerneldoc comments in C files, using
   the same selection rule as Kbuild; regular expressions only locate the
   ``SYSCALL_DEFINEx()`` or function that follows each comment
2. **Vmlinux**: Reads the ``.kapi_specs`` ELF section from compiled kernels
3. **Debugfs**: Reads from ``/sys/kernel/debug/kapi/`` filesystem interface

The tool supports all KAPI specification types:

- System calls (kerneldoc annotations)
- Kernel functions (kerneldoc annotations with KAPI tags)

Troubleshooting
===============

Common Issues
-------------

**Specification Not Found**

A syscall that is missing from ``/sys/kernel/debug/kapi/list`` has no
specification. Ensure the KAPI-annotated kerneldoc comment is in the same
translation unit as the function implementation, is named ``sys_<name>`` for
``SYSCALL_DEFINEx(<name>, ...)``, and has a ``contexts:`` line plus one more
KAPI section as described above.

**Validation Failures**::

    kapi: Parameter fd: invalid file descriptor -1

    Solution: Check parameter constraints or adjust specification if
    the constraint is incorrect.

Debug Options
-------------

Violations are logged with ``pr_warn_ratelimited()``. Error codes that a
specification does not list are logged with ``pr_debug()``; with
``CONFIG_DYNAMIC_DEBUG`` they can be enabled with::

    echo 'file kernel_api_spec.c +p' > /sys/kernel/debug/dynamic_debug/control

Contributing
============

Submitting Specifications
-------------------------

1. Add specifications to the same file as the API implementation
2. Follow existing patterns and naming conventions
3. Test with CONFIG_KAPI_RUNTIME_CHECKS enabled
4. Verify debugfs output is correct
5. Run scripts/checkpatch.pl on your changes
