.. SPDX-License-Identifier: GPL-2.0

=====================
ADSP OTP NVMEM driver
=====================

``drivers/nvmem/adsp-otp*`` gives access to the customer area of the OTP
(One-Time Programmable) memory of ADI ADSP processors, and to the factory
serial number; currently the ADSP-SC58x. OTP bits can be changed from 0 to
1 once and never back. On the SC58x all accesses go through the boot ROM
API (see :doc:`sc5xx-rom`), which implements the redundancy and ECC (Error
Correcting Code) scheme of the OTP array.

Acronyms: OTPC is the OTP Controller; PMC is the OTPC's programming
controller, which burns the fuses; RCU is the Reset Control Unit; FSN is
the Factory Serial Number; HUK, EK and DTCP are the Hardware Unique Key,
Endorsement Key and Digital Transmission Content Protection keys stored
in OTP.

Device tree
===========

See ``Documentation/devicetree/bindings/nvmem/adi,adsp-otp.yaml``.
The SoC specific data (field layout, how the OTP is accessed, conditions
for locking) is selected by the compatible string. The ``adi,rcu`` phandle
points to the Reset Control Unit, used to check that the SHARC+ cores are
held in reset or idle before programming (see `Other cores`_).

Driver structure
================

The driver is split so that other ADSP families can be added without
touching the user interface or the programming rules:

=========================  ================================================
``adsp-otp.c``             Generic core: the NVMEM device, the write rules
                           and read-back verification, the ``lock`` and
                           ``lock_state`` attributes, locking and buffer
                           wiping.
``adsp-otp-<soc>.c``       Per-SoC data (``struct adsp_otp_soc_data``): the
                           field table, the guarded half-words, the
                           backend's parameters, and ``lock_precheck()``,
                           the SoC's conditions for locking.
``adsp-otp-rom.c``         Boot ROM backend: the platform driver for the
                           SoCs whose OTP is reached through the ROM API
                           (its ``of_match_table`` maps each compatible to
                           the SoC data), and its ``struct adsp_otp_ops``:
                           reads, programs and locks through the ROM API,
                           checks the OTPC error bits and the RCU. Its
                           comment lists what it assumes about the
                           hardware: the OTPC register map and encodings
                           are the same on the SC57x, SC58x, SC59x, SC598
                           and SC84x, but it only handles the 32-bit ROM
                           interface of the Cortex-A5 parts.
``adsp-otp.h``             The interfaces between them.
=========================  ================================================

The compatible string selects both the backend, whose platform driver
matches it, and the SoC data, the match data. The backend's probe hands its
operations to the core (``adsp_otp_probe()``), which does the rest; there
is no global list of backends. A backend reports the lock state from the
hardware (``lock_state()``), so that a pending lock survives rebinding the
driver. A SoC whose OTP is reached differently, e.g. through secure
firmware, needs a new backend, with its own platform driver and
compatibles; one with the same ROM API only needs new data and an entry in
the ROM backend's ``of_match_table``.

Intended use
============

The driver is meant for manufacturing images. ``CONFIG_NVMEM_ADI_ADSP_OTP``
lets root read and program every customer OTP field, keys included, and
permanently lock the part for secure boot. Programming and locking are
irreversible. The device tree ``read-only`` property disables both. The
SC589-MINI device tree sets it; a manufacturing image removes it, e.g.
from U-Boot before booting::

    fdt addr ${fdt_addr_r}
    fdt rm /scb/otp@31011000 read-only

Reading
=======

Each OTP device has an NVMEM device ``adsp-otp<N>``, numbered in probe
order; on the SC58x there is one, ``adsp-otp0``. The NVMEM device (root
only) presents the customer OTP area, followed by
the factory serial number. Byte offsets are those of CCES
``cdefSC58x_rom_otpLayout.h`` (``ROM_OTP_<field>``) and data is each
field's 32-bit words in little-endian byte order (the driver requires a
little-endian kernel). There is one NVMEM cell per field::

    hexdump -C /sys/bus/nvmem/devices/adsp-otp0/nvmem
    hexdump -C /sys/bus/nvmem/devices/adsp-otp0/cells/gp1@300,0

Reads may start at any byte offset and be of any length. Reserved areas
read as zero. Fields are read as the ROM's ``otp_get()`` returns them,
except:

* ``antiroll_nv_cntr`` (0x2c0) is read raw. ``otp_get()`` would return the
  decoded count instead of the 16 words the offsets promise.
* ``fsn`` (0x380, 16 bytes, read-only) is outside the customer area. A
  locked part cannot read it (silicon anomaly 20000108), so it reads as
  zero once the part is locked.

On a locked part the hardware may deny access to the key fields. On a
part in the "unlocked" state (``OTPC_SECU_STATE`` PARTLOCK = 2) the SC58x
boot ROM returns zeros for all customer fields, and reports success, so
reads show zeros (the raw reads above do the same); programming is
refused.

Writing
=======

The OTP is programmed in 16-bit half-words, each with its own ECC, so each
half-word can be programmed exactly once. Writes must be 2-byte aligned
and a multiple of 2 bytes long. For every half-word of a write, comparing
the requested value with the current one:

=========================  ================================================
requested 0                skipped: the hardware ignores 0, OTP bits
                           cannot be cleared
equal to the current       skipped
current is 0 (blank)       programmed
anything else              ``EEXIST``
=========================  ================================================

A write can thus cover a single half-word, part of a field, several fields
or the whole area, and a write that is repeated after succeeding changes
nothing. For example, to program customer public key 0 (64 bytes at
offset 0x200) or half-word 7 of gp1 (offset 0x300 + 14)::

    dd if=pubkey0.bin of=/sys/bus/nvmem/devices/adsp-otp0/nvmem bs=64 seek=8
    printf '\xbe\xef' | dd of=/sys/bus/nvmem/devices/adsp-otp0/nvmem \
       bs=2 seek=391

The data is bytes in OTP address order: the second command puts 0xbe at
0x30e and 0xef at 0x30f, which is how ``hexdump -C`` of the NVMEM device
then shows them.

``bootModeDisable`` is the byte at offset 0x340 (bit assignments in the
HRM, "Boot Mode Disable"), which is where the SC58x ROM stores it and where
its boot code reads it. The HRM's OTP layout table shows byte 0x343
instead, which is where the SC59x ROM keeps it.

Each ``write()`` is checked as a whole before anything is programmed:
write multi-word values such as keys in a single ``write()`` (e.g. ``dd
bs=<size>``), so that a rejected request (below) cannot leave one partly
programmed. This does not cover failures while programming (``EIO``,
below), which can leave part of a field programmed.

Guarded half-words
------------------

Some half-words inside writable fields are refused (``EACCES``) even when
blank:

* ``boot_info`` word 15 (0x2bc-0x2bf) holds ``otpTiming``, the OTP read
  timing override (``ADI_ROM_OTP_BOOT_INFO.otpReadTiming`` in CCES). The
  ROM's ``otp_cfg()``, run on every boot and by this driver before every
  programming operation, loads it into ``OTPC_TIMING`` when its high byte
  is non-zero. A wrong value can make the OTP unreadable, and the boot ROM
  then stops booting. It can only be written with the module parameter
  ``allow_unsafe_fields=1`` (``nvmem_adsp_otp.allow_unsafe_fields=1`` on the
  kernel command line when built in), which takes effect at load time
  only. Leave the word 0 in boot_info images otherwise.
* The half-word after ``stageID`` (0x37a) is reserved and never written.

Errors
------

All rules are checked for all fields of a request before anything is
programmed. A write fails, with nothing programmed, with:

==========  ===============================================================
``EROFS``   The device is read-only (device tree ``read-only``), the part
            is locked or "unlocked", or its lock is programmed (it takes
            effect at the next reset). With ``read-only`` the NVMEM file
            is not writable at all, so ``open()`` already fails with
            ``EACCES``.
``EBUSY``   A SHARC+ core is running (see `Other cores`_).
``EINVAL``  Not 2-byte aligned, an odd length, or a value field
            (``bootModeDisable``, 8 bits) is out of range.
``EACCES``  A non-zero half-word in a reserved area, in a guarded
            half-word (see above), or in a field that is not writable:
            ``antiroll_nv_cntr`` is a counter the ROM advances from a
            count value, not raw data, and ``fsn`` is ADI's.
``EEXIST``  A half-word conflicts with its current content (see above).
==========  ===============================================================

Fields are then programmed one at a time, at most four half-words per ROM
call (see `How the ROM programs OTP`_), and each field is read back and
compared afterwards. A failure while programming fails the write with
``EIO``: the PMC not idle, an error bit in ``OTPC_STAT`` or
``OTPC_PMC_STAT``, the ROM reporting an error, or a difference on
read-back, for which the log names the half-words concerned (never their
values: fields may hold keys). ``EBUSY`` can also occur here, if a SHARC+
core is started while a write is in progress.

``EIO`` does not mean that nothing was programmed. Fields programmed
before the failing one stay programmed, and the failing field itself may
be partly programmed, a single key included: some of its half-words may
already be burned. After ``EIO``, read the area back to find out what it
holds before deciding how to proceed; half-words still blank can be
programmed by a later write, others cannot be changed.

Silicon and ROM anomalies
=========================

20000039, OTP API does not report OTP errors
    The ROM's results do not reflect the OTPC error bits. The driver checks
    ``OTPC_STAT.ADDRERR`` after every ROM call, and ``OTPC_PMC_STAT[5:4]``
    after every programming call, as the ROM's own ``pmc_error()`` does. A
    bit that was already set before a call cannot be attributed to it; the
    bits are read-only, so the driver only warns about it once.

20000084, simultaneous OTP accesses by multiple cores can cause a core hang
    See `Other cores`_.

20000108, factory serial number cannot be read when the device is locked
    Copy ``fsn`` into ``gp1`` (or other OTP that a locked part can read)
    before locking; ``lock`` warns if ``gp1`` does not hold it.

20000129, ROM OTP programming fails in presence of a leaky bit
    According to CCES ``sys/SC5xx_anomaly_macros_rtl.h`` (all ADSP-SC58x
    parts), the ROM's OTP programming functions can fail to program a
    16-bit half-word correctly "in presence of a leaky bit"; the half-word
    then reads back as 0. No workaround is published, hence the read back
    after every field.

Other cores
===========

The SHARC+ cores can reach the OTP too, through their own copy of the ROM
OTP functions, and simultaneous OTP accesses by several cores can hang a
core (anomaly 20000084). The driver therefore only programs the OTP, and
locks the part, while each SHARC+ core is either

* held in reset (its bits in ``RCU0_CRCTL`` and ``RCU0_CRSTAT`` both set,
  as after power-up), or
* idle, as reported by its ``C<n>IDLE`` bit in ``RCU0_MSG``. That is the
  HRM's core reset handshake: the core's software sets the bit when asked
  to stop. The ADI remoteproc driver stops a core this way, without
  putting it into reset.

This is checked before each write and again before every programming ROM
call; otherwise the driver fails with ``EBUSY``. Stop the cores first,
e.g.::

    echo stop > /sys/class/remoteproc/remoteproc0/state
    echo stop > /sys/class/remoteproc/remoteproc1/state

and check that remoteproc did not log a timeout waiting for the core to
idle: a core that does not take part in the handshake keeps running.

Reads are not restricted, as a hang during a read damages nothing, but
they can still hang the system if a SHARC+ core reads the OTP at the same
time.

Locking
=======

::

    cat /sys/bus/platform/devices/31011000.otp/lock_state
    echo permanently-lock > /sys/bus/platform/devices/31011000.otp/lock

See ``Documentation/ABI/testing/sysfs-driver-adsp-otp``. NVMEM has no lock
operation, and locking is a single, irreversible, whole-device action
rather than data: a write-only attribute demanding an exact confirmation
string cannot be triggered by a stray ``dd``.

Once locked, after the next reset, the boot ROM only accepts secure boot
streams, and secure boot cannot be disabled again. A standard boot stream,
or a secure one that fails authentication, stops the boot with the core
idle and the fault pin active (EE-366, "Locking the Part"; HRM, "Secure
Boot"). Streams are authenticated with one customer public key: like the
ROM, the first one not invalidated in ``boot_info`` (``public_key0``
unless ``pubkey0Inv`` is set, then ``public_key1`` unless ``pubkey1Inv``
is set; EE-384, "Public and Encryption (Private) Key in OTP Space"). The
ROM does not skip a blank key. The driver therefore refuses to lock
(``EPERM``) unless that key is programmed. It does not check that the key
is a valid ECDSA key, nor that it matches the signed images.

Debug access to a locked part requires the debugger to present the
``secure_emu_key`` (EE-384); the driver warns when locking with that
field blank. It also warns when ``gp1`` does not hold the factory serial
number (anomaly 20000108).

``EINVAL``, ``EPERM``, ``EROFS`` and ``EBUSY`` from ``lock`` mean that
nothing was programmed. ``EIO`` does not: it can come from a failure while
or after the lock is programmed, so the part may already be locked. After
``EIO``, read ``lock_state`` before proceeding; ``lock-pending`` means the
part will be locked from the next reset on.

Concurrency
===========

* Each OTP device has its own mutex, which serializes every backend call,
  every check-then-program sequence and the lock operation on that device
  (the NVMEM core does not serialize ``reg_read``/``reg_write``). Without
  it two writers could both find a field blank and both program it, or a
  write could slip in after the lock decision.
* Data to be written is copied into a private buffer first, so the checks
  and the ROM call see the same bytes.
* Several OTP devices can bind, each with its own state, lock, NVMEM device
  and sysfs attributes. A per-device lock is only enough if each device
  has its own hardware, so a backend must keep two devices from driving the
  same OTP. The ROM backend does: the SC58x ROM functions take no instance
  argument and only drive OTPC0 at 0x31011000, so a device is only accepted
  for that OTPC (``ENODEV`` otherwise), and mapping its registers claims
  them, so a second device for it fails to bind (``EBUSY``). ROM calls are
  also serialized across all devices by ``stop_machine()``.
* Buffers that held key material are wiped after every operation, and
  diagnostics never log OTP values.
* A pending lock is read from the OTP, not remembered by the driver, so
  unbinding, rebinding or reloading the driver cannot re-enable
  programming before the reset that makes the lock effective.
* The SHARC+ cores must be held in reset while programming (see
  `Other cores`_). Other bus masters (secure firmware, a debugger) must
  not use the OTPC while this driver does; this is not checked.

How the ROM programs OTP
========================

The OTP module integrates the OTP array with the PMC, a charge pump and a
voltage regulator; programming needs no external supply. On the SC589 the
ROM functions involved are:

==================  ======  ===============================================
``otp_cfg()``       0x3e46  Set up the OTPC (PMC mode, program and read
                            timing, including the ``otpTiming`` override).
                            Preboot runs it on every boot unless the part
                            is "unlocked" (PARTLOCK = 2).
``otp_pgm()``       0x25d0  Calls ``otp_pgm_data()`` for each non-null
                            field of ``struct otp_data``, and always
                            ``otp_pgm_cntr32()`` for ``antiroll_nv_cntr``.
``otp_pgm_data()``  0x3c84  Leaves read mode, programs the data at the
                            field's offset and its redundant copies at
                            +0x800, +0x1000 and +0x1800, returns to read
                            mode.
==================  ======  ===============================================

Per non-zero half-word the ROM writes the address and data registers,
starts the PMC and waits for "done" or an error, *without a timeout*. A
PMC that never completes would hang the ROM, with interrupts disabled on
all CPUs. The driver therefore, before every ``otp_pgm()`` or
``otp_lock()``, checks the SHARC+ cores, calls ``otp_cfg()`` (the hardware
reference manual allows this "to ensure the OTPC is configured
correctly") and only proceeds if the PMC is idle (``PMC_STAT[7:6] =
0b10``), the ROM's own precondition. This cannot detect a PMC that accepts
a command and never finishes it. The OTPC and PMC setup is logged at debug
level around each programming operation (enable with dynamic debug, module
``nvmem_adsp_otp``).

Every half-word takes four PMC operations (the data and its three copies),
all with interrupts disabled, so the driver passes the ROM at most four
half-words per ``otp_pgm()`` call; the ROM skips the half-words that are 0.

``otp_pgm()`` always hands ``antiroll_nv_cntr`` to ``otp_pgm_cntr32()``,
which programs that count and fails if it is lower than the current one.
Passing 0 would make every ``otp_pgm()`` on a part whose counter is not 0
report a failure, although the requested field was programmed. The driver
reads the current count with ``otp_get()`` and passes it back, so that the
counter is left alone.

The PMC registers (OTPC offset 0x80 and up) are only accessible while
``OTPC_CTL.READMODE = 0``; in read mode, the normal state, any access to
them is a bus error, from the kernel, from user space or from a JTAG
debugger alike. The driver clears ``READMODE`` around its own PMC accesses,
like the ROM does. OTPC offsets 0x30-0x73 are a hole where any access
aborts.

Assumptions
===========

* ROM function signatures from CCES ``cdefSC58x_rom.h``, checked against
  ADI's ROM source::

      bool otp_get(OTPCMD cmd, uint32_t data[]);
      bool otp_get_data(uint32_t data[], uint16_t offset,
                        uint16_t sizeIn32bitWords);
      bool otp_pgm(otp_data *data);
      bool otp_lock(void);
      bool otp_cfg(void);

  ``otp_get_data()`` takes a byte offset into the OTP and a size in 32-bit
  words. ``struct otp_data`` is SoC specific; its SC58x layout is checked
  with ``static_assert()``.
* Programming always uses the field-based ``otp_pgm()``, as the hardware
  reference manual recommends, never the offset-based ``otp_pgm_data()``.
* Linux runs as a secure bus requester (see :doc:`sc5xx-rom`). This is not
  checked: ``SPU_SECURECHK`` lies in the upper half of the SPU's register
  space, where non-secure accesses fault (anomaly 20000003), so it cannot
  report a non-secure requester cleanly.

Tested on an open SC589: reading all fields, and programming gp1 one
half-word at a time. ``otp_lock()`` uses the same ROM programming path but
has not been run on hardware.

References
==========

ADSP-SC58x/ADSP-2158x Hardware Reference Manual (OTPC, RCU, SPU, security
and boot ROM chapters); ADSP-SC58x/ADSP-2158x silicon anomaly list;
EE-366, Secure Booting Guide for Blackfin+ and SHARC+ Processors; EE-384,
Tips and Tricks Using the ADSP-SC58x/ADSP-2158x Processor Boot ROM; CCES
3.1.0 ``defSC58x_rom_jumptable.h``, ``cdefSC58x_rom.h``,
``cdefSC58x_rom_otpLayout.h``, ``sys/ADSP_SC589.h``,
``sys/SC5xx_anomaly_macros_rtl.h``.
