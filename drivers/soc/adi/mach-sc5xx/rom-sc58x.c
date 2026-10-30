// SPDX-License-Identifier: GPL-2.0
/*
 * Call ADI ADSP-SC58x boot ROM API functions from Linux: the ADSP-SC58x
 * (Cortex-A5, 32-bit ARM) implementation of linux/soc/adi/adsp-rom.h.
 *
 * The boot ROM was written to run with the MMU (Memory Management Unit)
 * off and uses hard-coded physical addresses. Each call therefore runs,
 * inside stop_machine(), on a private first-level translation table that
 * holds the current kernel mappings plus 1:1 ("identity") sections for the
 * regions the ROM touches. See Documentation/arch/arm/adi/sc5xx-rom.rst
 * for the design and its assumptions.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bits.h>
#include <linux/cacheflush.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/pgtable.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/soc/adi/adsp-rom.h>
#include <linux/stop_machine.h>
#include <linux/string.h>
#include <asm/cp15.h>
#include <asm/domain.h>
#include <asm/outercache.h>
#include <asm/proc-fns.h>
#include <asm/tlbflush.h>

/*
 * First-level section descriptor attributes (ARMv7 short-descriptor
 * format). nG ("not global") tags TLB entries with the caller's ASID, so a
 * stale entry can never be hit by another process. The kernel domain is
 * required because, with CONFIG_CPU_SW_DOMAIN_PAN, the user domain (where
 * these low addresses normally live) is inaccessible from the kernel.
 *
 * Linux runs with TEX remap enabled (SCTLR.TRE = 1), so TEX[0]:C:B is an
 * index into PRRR/NMRR (arch/arm/mm/proc-v7-2level.S), not the classic
 * encoding: index 0b000 is strongly-ordered.
 */
#define SECT_BASE	(PMD_TYPE_SECT | PMD_SECT_AP_WRITE | PMD_SECT_nG | \
			 PMD_DOMAIN(DOMAIN_KERNEL))
#define SECT_TRE_SO	0

#define SCTLR_TRE	BIT(28)

/*
 * The private first-level translation table (TTB), in the hardware view:
 * 4096 entries of 4 bytes, one per 1 MiB, 16 KiB aligned. Linux's pgd_t
 * groups the same memory into 2048 entries of 8 bytes; this code does not
 * use it.
 */
#define TTB_ORDER	2
#define TTB_SIZE	(PAGE_SIZE << TTB_ORDER)
#define TTB_USER_ENTRIES	(TASK_SIZE >> SECTION_SHIFT)

/* TTBR0 bits 6:0 are table walk attributes, not part of the address */
#define TTBR0_ATTR_MASK	0x7f

/**
 * enum sc58x_rom_mem - memory type of an identity-mapped region
 * @SC58X_ROM_MEM_CODE: the boot ROM itself: strongly-ordered, executable
 * @SC58X_ROM_MEM_MMIO: memory-mapped registers (MMRs) or memory-mapped
 *	OTP: strongly-ordered, never executable
 */
enum sc58x_rom_mem {
	SC58X_ROM_MEM_CODE,
	SC58X_ROM_MEM_MMIO,
};

/**
 * struct sc58x_rom_region - a physical region the ROM accesses
 * @base: physical address, 1 MiB aligned; mapped at the same virtual address
 * @size: size in bytes, a multiple of 1 MiB
 * @type: memory type
 * @name: description, for diagnostics
 */
struct sc58x_rom_region {
	u32 base;
	u32 size;
	enum sc58x_rom_mem type;
	const char *name;
};

/*
 * The regions each SoC's ROM functions access, kept as small as the 1 MiB
 * sections allow: a ROM access outside them causes a kernel oops that
 * names the address, rather than silently reaching whatever is there.
 *
 * SC58x: the OTP functions (otp_get(), otp_get_data(), otp_pgm(),
 * otp_lock(), otp_cfg() and their helpers) only use the caller's stack,
 * the OTP array (0x24000000, with its redundant copies up to 0x24001fff)
 * and the OTPC registers (0x31011000-0x310110ff). They use no ROM data in
 * L2 SRAM: the ROM's boot-time variables and stacks at 0x200be000-
 * 0x200bffff are not mapped, as Linux and the SHARC+ cores use that
 * memory (ADSP-SC589 ARM boot ROM listing).
 */
static const struct sc58x_rom_region sc58x_rom_regions[] = {
	{ 0x00000000, SZ_1M, SC58X_ROM_MEM_CODE, "boot ROM" },
	{ 0x24000000, SZ_1M, SC58X_ROM_MEM_MMIO, "OTP array" },
	{ 0x31000000, SZ_1M, SC58X_ROM_MEM_MMIO, "OTPC MMRs" },
};

struct adsp_rom {
	u32 *ttb;			/* private table, filled per call */
	unsigned int nsect;
	u32 sect[];			/* section descriptors to install */
};

struct sc58x_rom_args {
	struct adsp_rom *rom;
	unsigned long fn;
	unsigned long arg[4];
	unsigned long ret;
};

typedef unsigned long (*sc58x_rom_fn_t)(unsigned long arg0,
					unsigned long arg1,
					unsigned long arg2,
					unsigned long arg3);

static u32 sc58x_read_ttbr0(void)
{
	u32 val;

	asm volatile("mrc p15, 0, %0, c2, c0, 0" : "=r" (val));
	return val;
}

static void sc58x_write_ttbr0(u32 val)
{
	asm volatile("mcr p15, 0, %0, c2, c0, 0" : : "r" (val) : "memory");
	isb();
}

/*
 * Switch this CPU to another first-level table. Both tables map some (low)
 * addresses differently under the same ASID, so drop the TLB and branch
 * predictor contents after the switch, as setup_mm_for_reboot() does for
 * the identity map. The instruction cache needs no maintenance: it is
 * physically tagged on ARMv7 implementations other than ASID-tagged VIVT
 * ones (cf. check_and_switch_context()). Only this CPU ever loads the
 * private table, so local maintenance is sufficient. The new table must
 * already be in memory (see sc58x_rom_fill_table()).
 */
static void sc58x_switch_ttbr0(u32 ttbr0)
{
	sc58x_write_ttbr0(ttbr0);	/* includes isb */
	local_flush_tlb_all();
	local_flush_bp_all();
	isb();
}

/*
 * Build the private table. Called with all other CPUs stopped and
 * interrupts disabled, immediately before use, so the copied kernel
 * first-level entries cannot go stale. The user half is left empty.
 */
static void sc58x_rom_fill_table(struct adsp_rom *rom)
{
	u32 *ttb = rom->ttb;
	unsigned int i;

	memset(ttb, 0, TTB_USER_ENTRIES * sizeof(u32));
	memcpy(ttb + TTB_USER_ENTRIES, (u32 *)cpu_get_pgd() + TTB_USER_ENTRIES,
	       TTB_SIZE - TTB_USER_ENTRIES * sizeof(u32));

	for (i = 0; i < rom->nsect; i++)
		ttb[rom->sect[i] >> SECTION_SHIFT] = rom->sect[i];

	/* The table walker may not snoop the caches: clean to memory */
	__cpuc_flush_dcache_area(ttb, TTB_SIZE);
	outer_clean_range(virt_to_phys(ttb), virt_to_phys(ttb) + TTB_SIZE);
}

/*
 * Runs via stop_machine(): one CPU, interrupts off, other CPUs parked.
 * __nocfi: the boot ROM has no kCFI type information, so the indirect
 * call into it must not be checked (as for EFI runtime services).
 */
static int __nocfi sc58x_rom_call_fn(void *data)
{
	struct sc58x_rom_args *args = data;
	sc58x_rom_fn_t fn = (sc58x_rom_fn_t)args->fn;
	u32 old_ttbr0;

	/* Written and cleaned to memory before the switch */
	sc58x_rom_fill_table(args->rom);

	old_ttbr0 = sc58x_read_ttbr0();
	sc58x_switch_ttbr0(virt_to_phys(args->rom->ttb) |
			   (old_ttbr0 & TTBR0_ATTR_MASK));

	/* Bit 0 of the address selects Thumb; BLX honours it */
	args->ret = fn(args->arg[0], args->arg[1], args->arg[2],
		       args->arg[3]);

	sc58x_switch_ttbr0(old_ttbr0);

	return 0;
}

/**
 * adsp_rom_call() - call a boot ROM function
 * @rom: handle from devm_adsp_rom_get()
 * @fn: ROM address of the function (bit 0 set for Thumb code)
 * @arg0: first argument (r0)
 * @arg1: second argument (r1)
 * @arg2: third argument (r2)
 * @arg3: fourth argument (r3)
 *
 * Unused arguments are ignored by the ROM. Pointer arguments must point to
 * kernel memory that is mapped at all times, e.g. from kmalloc(); never
 * to the stack. The call runs with interrupts disabled on all CPUs, and
 * may sleep while waiting for the other CPUs, so it must be called from
 * process context. Concurrent calls are serialized by stop_machine().
 *
 * Return: the value the ROM function returned in r0.
 */
unsigned long adsp_rom_call(struct adsp_rom *rom, unsigned long fn,
			    unsigned long arg0, unsigned long arg1,
			    unsigned long arg2, unsigned long arg3)
{
	struct sc58x_rom_args args = {
		.rom = rom,
		.fn = fn,
		.arg = { arg0, arg1, arg2, arg3 },
	};

	stop_machine(sc58x_rom_call_fn, &args, NULL);

	return args.ret;
}
EXPORT_SYMBOL_GPL(adsp_rom_call);

static u32 sc58x_rom_sect_attr(enum sc58x_rom_mem type)
{
	switch (type) {
	case SC58X_ROM_MEM_CODE:
		return SECT_BASE | SECT_TRE_SO;
	case SC58X_ROM_MEM_MMIO:
		return SECT_BASE | SECT_TRE_SO | PMD_SECT_XN;
	}

	return 0;
}

/**
 * devm_adsp_rom_get() - prepare for calling boot ROM functions
 * @dev: device that owns the handle
 *
 * The regions the ROM accesses, mapped 1:1 during calls, are those of
 * sc58x_rom_regions[]. Only for machines compatible with "adi,sc58x".
 *
 * Return: a handle for adsp_rom_call(), or an ERR_PTR().
 */
struct adsp_rom *devm_adsp_rom_get(struct device *dev)
{
	const struct sc58x_rom_region *regions = sc58x_rom_regions;
	unsigned int i, nregions = ARRAY_SIZE(sc58x_rom_regions);
	unsigned int nsect = 0, next = 0;
	struct adsp_rom *rom;
	u32 offset;

	/* The section attributes above assume TEX remapping */
	if (!(get_cr() & SCTLR_TRE)) {
		dev_err(dev, "SCTLR.TRE is clear, cannot call the boot ROM\n");
		return ERR_PTR(-ENODEV);
	}

	if (!of_machine_is_compatible("adi,sc58x")) {
		dev_err(dev, "boot ROM calls not supported on this SoC\n");
		return ERR_PTR(-ENODEV);
	}

	for (i = 0; i < nregions; i++) {
		const struct sc58x_rom_region *region = &regions[i];

		if (!IS_ALIGNED(region->base, SECTION_SIZE) || !region->size ||
		    !IS_ALIGNED(region->size, SECTION_SIZE) ||
		    region->base > TASK_SIZE ||
		    region->size > TASK_SIZE - region->base) {
			dev_err(dev, "invalid ROM region %s\n", region->name);
			return ERR_PTR(-EINVAL);
		}
		nsect += region->size >> SECTION_SHIFT;
	}

	rom = devm_kzalloc(dev, struct_size(rom, sect, nsect), GFP_KERNEL);
	if (!rom)
		return ERR_PTR(-ENOMEM);

	rom->ttb = (u32 *)devm_get_free_pages(dev, GFP_KERNEL, TTB_ORDER);
	if (!rom->ttb)
		return ERR_PTR(-ENOMEM);

	rom->nsect = nsect;
	for (i = 0; i < nregions; i++) {
		const struct sc58x_rom_region *region = &regions[i];

		dev_dbg(dev, "ROM identity map %#010x-%#010x: %s\n",
			region->base, region->base + region->size - 1,
			region->name);
		for (offset = 0; offset < region->size; offset += SECTION_SIZE)
			rom->sect[next++] = (region->base + offset) |
					    sc58x_rom_sect_attr(region->type);
	}

	return rom;
}
EXPORT_SYMBOL_GPL(devm_adsp_rom_get);
