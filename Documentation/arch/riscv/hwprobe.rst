.. SPDX-License-Identifier: GPL-2.0

RISC-V Hardware Probing Interface
---------------------------------

The RISC-V hardware probing interface is based around a single syscall, which
is defined in <asm/hwprobe.h>::

    struct riscv_hwprobe {
        __s64 key;
        __u64 value;
    };

    long sys_riscv_hwprobe(struct riscv_hwprobe *pairs, size_t pair_count,
                           size_t cpusetsize, cpu_set_t *cpus,
                           unsigned int flags);

The arguments are split into three groups: an array of key-value pairs, a CPU
set, and some flags. The key-value pairs are supplied with a count. Userspace
must prepopulate the key field for each element, and the kernel will fill in the
value if the key is recognized. If a key is unknown to the kernel, its key field
will be cleared to -1, and its value set to 0. The CPU set is defined by
CPU_SET(3) with size ``cpusetsize`` bytes. For value-like keys (eg. vendor,
arch, impl), the returned value will only be valid if all CPUs in the given set
have the same value. Otherwise -1 will be returned. For boolean-like keys, the
value returned will be a logical AND of the values for the specified CPUs.
Usermode can supply NULL for ``cpus`` and 0 for ``cpusetsize`` as a shortcut for
all online CPUs. The currently supported flags are:

* :c:macro:`RISCV_HWPROBE_WHICH_CPUS`: This flag basically reverses the behavior
  of sys_riscv_hwprobe().  Instead of populating the values of keys for a given
  set of CPUs, the values of each key are given and the set of CPUs is reduced
  by sys_riscv_hwprobe() to only those which match each of the key-value pairs.
  How matching is done depends on the key type.  For value-like keys, matching
  means to be the exact same as the value.  For boolean-like keys, matching
  means the result of a logical AND of the pair's value with the CPU's value is
  exactly the same as the pair's value.  Additionally, when ``cpus`` is an empty
  set, then it is initialized to all online CPUs which fit within it, i.e. the
  CPU set returned is the reduction of all the online CPUs which can be
  represented with a CPU set of size ``cpusetsize``.

All other flags are reserved for future compatibility and must be zero.

On success 0 is returned, on failure a negative error code is returned.

The following keys are defined:

* :c:macro:`RISCV_HWPROBE_KEY_MVENDORID`: Contains the value of ``mvendorid``,
  as defined by the RISC-V privileged architecture specification.

* :c:macro:`RISCV_HWPROBE_KEY_MARCHID`: Contains the value of ``marchid``, as
  defined by the RISC-V privileged architecture specification.

* :c:macro:`RISCV_HWPROBE_KEY_MIMPLID`: Contains the value of ``mimplid``, as
  defined by the RISC-V privileged architecture specification.

* :c:macro:`RISCV_HWPROBE_KEY_BASE_BEHAVIOR`: A bitmask containing the base
  user-visible behavior that this kernel supports.  The following base user ABIs
  are defined:

  * :c:macro:`RISCV_HWPROBE_BASE_BEHAVIOR_IMA`: Support for rv32ima or
    rv64ima, as defined by version 2.2 of the user ISA and version 1.10 of the
    privileged ISA, with the following known exceptions (more exceptions may be
    added, but only if it can be demonstrated that the user ABI is not broken):

    * The ``fence.i`` instruction cannot be directly executed by userspace
      programs (it may still be executed in userspace via a
      kernel-controlled mechanism such as the vDSO).

* :c:macro:`RISCV_HWPROBE_KEY_IMA_EXT_0`: A bitmask containing the extensions
  that are compatible with the :c:macro:`RISCV_HWPROBE_BASE_BEHAVIOR_IMA`:
  base system behavior.

  * :c:macro:`RISCV_HWPROBE_IMA_FD`: The F and D extensions are supported, as
    defined by commit cd20cee ("FMIN/FMAX now implement
    minimumNumber/maximumNumber, not minNum/maxNum") of the RISC-V ISA manual.

  * :c:macro:`RISCV_HWPROBE_IMA_C`: The C extension is supported, as defined
    by version 2.2 of the RISC-V ISA manual.

  * :c:macro:`RISCV_HWPROBE_IMA_V`: The V extension is supported, as defined by
    version 1.0 of the RISC-V Vector extension manual.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBA`: The Zba address generation extension is
       supported, as defined in version 1.0 of the Bit-Manipulation ISA
       extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBB`: The Zbb extension is supported, as defined
       in version 1.0 of the Bit-Manipulation ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBS`: The Zbs extension is supported, as defined
       in version 1.0 of the Bit-Manipulation ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZICBOZ`: The Zicboz extension is supported, as
       ratified in commit 3dd606f ("Create cmobase-v1.0.pdf") of riscv-CMOs.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBC` The Zbc extension is supported, as defined
       in version 1.0 of the Bit-Manipulation ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBKB` The Zbkb extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBKC` The Zbkc extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZBKX` The Zbkx extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZKND` The Zknd extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZKNE` The Zkne extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZKNH` The Zknh extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZKSED` The Zksed extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZKSH` The Zksh extension is supported, as
       defined in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZKT` The Zkt extension is supported, as defined
       in version 1.0 of the Scalar Crypto ISA extensions.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVBB`: The Zvbb extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVBC`: The Zvbc extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKB`: The Zvkb extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKG`: The Zvkg extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKNED`: The Zvkned extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKNHA`: The Zvknha extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKNHB`: The Zvknhb extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKSED`: The Zvksed extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKSH`: The Zvksh extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVKT`: The Zvkt extension is supported as
       defined in version 1.0 of the RISC-V Cryptography Extensions Volume II.

  * :c:macro:`RISCV_HWPROBE_EXT_ZFH`: The Zfh extension version 1.0 is supported
       as defined in the RISC-V ISA manual.

  * :c:macro:`RISCV_HWPROBE_EXT_ZFHMIN`: The Zfhmin extension version 1.0 is
       supported as defined in the RISC-V ISA manual.

  * :c:macro:`RISCV_HWPROBE_EXT_ZIHINTNTL`: The Zihintntl extension version 1.0
       is supported as defined in the RISC-V ISA manual.

  * :c:macro:`RISCV_HWPROBE_EXT_ZVFH`: The Zvfh extension is supported as
       defined in the RISC-V Vector manual starting from commit e2ccd0548d6c
       ("Remove draft warnings from Zvfh[min]").

  * :c:macro:`RISCV_HWPROBE_EXT_ZVFHMIN`: The Zvfhmin extension is supported as
       defined in the RISC-V Vector manual starting from commit e2ccd0548d6c
       ("Remove draft warnings from Zvfh[min]").

  * :c:macro:`RISCV_HWPROBE_EXT_ZFA`: The Zfa extension is supported as
       defined in the RISC-V ISA manual starting from commit 056b6ff467c7
       ("Zfa is ratified").

  * :c:macro:`RISCV_HWPROBE_EXT_ZTSO`: The Ztso extension is supported as
       defined in the RISC-V ISA manual starting from commit 5618fb5a216b
       ("Ztso is now ratified.")

  * :c:macro:`RISCV_HWPROBE_EXT_ZACAS`: The Zacas extension is supported as
       defined in the Atomic Compare-and-Swap (CAS) instructions manual starting
       from commit 5059e0ca641c ("update to ratified").

  * :c:macro:`RISCV_HWPROBE_EXT_ZICOND`: The Zicond extension is supported as
       defined in the RISC-V Integer Conditional (Zicond) operations extension
       manual starting from commit 95cf1f9 ("Add changes requested by Ved
       during signoff")

  * :c:macro:`RISCV_HWPROBE_EXT_ZIHINTPAUSE`: The Zihintpause extension is
       supported as defined in the RISC-V ISA manual starting from commit
       d8ab5c78c207 ("Zihintpause is ratified").

* :c:macro:`RISCV_HWPROBE_KEY_CPUPERF_0`: A bitmask that contains performance
  information about the selected set of processors.

  * :c:macro:`RISCV_HWPROBE_MISALIGNED_UNKNOWN`: The performance of misaligned
    accesses is unknown.

  * :c:macro:`RISCV_HWPROBE_MISALIGNED_EMULATED`: Misaligned accesses are
    emulated via software, either in or below the kernel.  These accesses are
    always extremely slow.

  * :c:macro:`RISCV_HWPROBE_MISALIGNED_SLOW`: Misaligned accesses are slower
    than equivalent byte accesses.  Misaligned accesses may be supported
    directly in hardware, or trapped and emulated by software.

  * :c:macro:`RISCV_HWPROBE_MISALIGNED_FAST`: Misaligned accesses are faster
    than equivalent byte accesses.

  * :c:macro:`RISCV_HWPROBE_MISALIGNED_UNSUPPORTED`: Misaligned accesses are
    not supported at all and will generate a misaligned address fault.

* :c:macro:`RISCV_HWPROBE_KEY_ZICBOZ_BLOCK_SIZE`: An unsigned int which
  represents the size of the Zicboz block in bytes.
