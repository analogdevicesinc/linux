.. SPDX-License-Identifier: GPL-2.0

===============================================
Calling the ADSP-SC5xx boot ROM API from Linux
===============================================

The ADSP-SC5xx boot ROM exports a table of helper functions (the "jump
table"), for example to read, program and lock the OTP (One-Time
Programmable) memory. Drivers call them through the architecture-neutral
interface in ``linux/soc/adi/adsp-rom.h``:

.. code-block:: c

   #include <linux/soc/adi/adsp-rom.h>

   rom = devm_adsp_rom_get(dev);
   ret = adsp_rom_call(rom, ROM_FUNC_ADDR, arg0, arg1, 0, 0);

This document describes its ADSP-SC58x (Cortex-A5, 32-bit ARM)
implementation, ``drivers/soc/adi/mach-sc5xx/rom-sc58x.c``, enabled by
``CONFIG_ADI_ADSP_ROM``, which drivers select. The regions the ROM functions access (the ROM itself and,
for the OTP functions, the memory-mapped OTP array and the OTP controller
registers) are part of the implementation, chosen by the machine's
compatible string, so drivers do not need to know them.

The problem
===========

The boot ROM was written to run before the MMU (Memory Management Unit) is
turned on. Its code contains hard-coded *physical* addresses: of its own
data, of the OTP array and of peripheral registers (MMRs,
Memory-Mapped Registers). With the MMU on, every address the CPU uses is a
*virtual* address that Linux's translation tables map wherever Linux
chooses. Jumping into the ROM from Linux would send those addresses to the
wrong place, or fault.

The fix is an *identity* (1:1) mapping, where virtual address equals
physical address, for every region the ROM touches. The ROM then works as
if the MMU were off.

The regions are kept as small as 1 MiB sections allow, so that a ROM
access anywhere else oopses instead of silently reaching memory that
belongs to someone else. On the SC58x the OTP functions only use the
caller's stack, the OTP array at 0x24000000 and the OTP controller
registers at 0x31011000, so only the 1 MiB sections holding the ROM, the
OTP array and the OTPC are mapped. The ROM's own boot-time variables and
stacks in L2 SRAM (0x200be000-0x200bffff) are not: the OTP functions do
not use them, while Linux (e.g. rpmsg buffers) and the SHARC+ cores do
use that memory. A ROM function that needs more must have its region
added explicitly, and its users must keep that memory reserved.

How a call works
================

For each call, ``adsp_rom_call()`` runs the following inside
``stop_machine()``, i.e. on one CPU with interrupts disabled and every
other online CPU spinning with interrupts disabled:

1. Fill a private first-level translation table: the kernel half is
   copied from the table the CPU is running on *right now*, the user half
   is left empty.
2. Add 1 MiB section entries mapping the SoC's ROM regions 1:1.
3. Clean the table to memory (the table walker may not see the caches),
   switch TTBR0 (Translation Table Base Register 0) to the private table,
   and invalidate the TLB (Translation Lookaside Buffer, the MMU's cache
   of translations) and the branch predictor. This is the sequence Linux
   itself uses to switch to its identity map (``setup_mm_for_reboot()``).
   The instruction cache is physically tagged and needs no maintenance.
4. Call the ROM function, then switch back to the original table and
   invalidate the TLB and branch predictor again.

The process that called the driver keeps its own page tables untouched.
Only the calling CPU ever loads the private table, so all maintenance is
local to it.

Why the table is rebuilt for every call
---------------------------------------

Most first-level entries are *pointers* to second-level tables, which stay
shared with the live table. A copy taken once and reused later can point
at a second-level table that has since been freed (for example after an
``iounmap()`` of a section mapping, or when a process's page tables are
freed). Copying with every CPU stopped, right before use, rules this out.
The user half is not copied at all, so it never points at a process's page
tables, and a ROM access outside the listed regions faults cleanly: the
kernel oopses naming the address. Kernel mappings added to ``init_mm``
after the copy are still found: a kernel-address translation fault copies
the entry into the table in TTBR0, i.e. the private one.

Section attributes
==================

Every identity entry is:

* kernel read/write, no user access (``PMD_SECT_AP_WRITE``);
* in the kernel *domain*. With ``CONFIG_CPU_SW_DOMAIN_PAN`` the user
  domain, where these low addresses normally live, is inaccessible from
  kernel code;
* *not global* (``PMD_SECT_nG``): TLB entries are tagged with the current
  ASID (Address Space IDentifier), so a stale entry, e.g. after a crash
  inside the ROM, can only ever match the calling process;
* *execute never* (``PMD_SECT_XN``), except the ROM itself.

Linux on ARMv7 runs with TEX remapping (``SCTLR.TRE = 1``). The TEX[0], C
and B bits then are an index into the PRRR/NMRR registers set up in
``arch/arm/mm/proc-v7-2level.S``, not the classic ARM memory types:

=====  ======================  ======================================
index  memory type             used for
=====  ======================  ======================================
0b000  strongly-ordered        ROM code, MMRs, memory-mapped OTP
=====  ======================  ======================================

The memory-mapped OTP must not be cacheable either: it does not support
the burst transfers of cache line fills (HRM, OTPC chapter).

``devm_adsp_rom_get()`` refuses to work if ``SCTLR.TRE`` is clear.

The MMRs must be strongly-ordered, exactly as the ROM sees them with the
MMU off. An early version of the OTP driver mapped them with index 0b001,
taking it for Device memory. With TEX remapping that is Normal memory,
where stores may be merged and reordered: the three adjacent stores that
start an OTP program operation never started it, and the ROM waited
forever, while single loads (OTP reads) kept working.

Requirements and assumptions
============================

* Classic (non-LPAE) two-level page tables with a single table (TTBR0
  only). The Cortex-A5 does not implement LPAE.
* A little-endian kernel. The ROM is little-endian code and runs with the
  kernel's data endianness, so ``CONFIG_ADI_ADSP_ROM`` is not available
  with ``CONFIG_CPU_BIG_ENDIAN``.
* All regions are 1 MiB aligned, a multiple of 1 MiB in size and below
  ``TASK_SIZE``.
* Linux runs in the Secure world, so the ROM's bus accesses are secure, as
  it expects (ADI's U-Boot does not switch SC5xx to the Non-secure world).
* Pointer arguments point to kernel memory mapped at all times, e.g. from
  ``kmalloc()``. Never pass stack buffers: with ``CONFIG_VMAP_STACK`` the
  stack lives in vmalloc space, and on SMP ``stop_machine()`` runs the call
  on the stopper thread's stack.
* The ROM function signatures are those of ADI's ROM headers (CCES
  ``cdefSC58x_rom.h`` and friends); the ROM is Thumb code, so function
  addresses have bit 0 set.
* Nothing else (SHARC cores, secure firmware, a debugger) uses the same
  hardware concurrently. ``stop_machine()`` only stops the ARM cores
  running Linux; callers must check for the other cores themselves (the
  OTP driver requires the SHARC+ cores to be held in reset or idle
  before it programs the OTP, see silicon anomaly 20000084).
* The ROM functions use no floating-point or SIMD registers (none appear
  in the SC58x ARM boot ROM), so no FPU state needs saving.

Side effects
============

Every call keeps interrupts disabled on all CPUs for its whole duration.
Callers should keep each call short (the OTP driver programs at most a
few 16-bit half-words per call). If the ROM faults, the kernel oopses with the calling CPU still on
the private table and interrupts disabled; treat the system as unreliable
and reboot.
