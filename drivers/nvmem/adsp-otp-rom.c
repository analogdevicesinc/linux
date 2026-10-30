// SPDX-License-Identifier: GPL-2.0
/*
 * ADI ADSP OTP driver: boot ROM backend.
 *
 * Reads, programs and locks the OTP through the on-chip boot ROM API
 * (otp_get(), otp_get_data(), otp_pgm(), otp_lock(), otp_cfg(); see
 * linux/soc/adi/adsp-rom.h), and checks the OTP controller (OTPC) and the
 * Reset Control Unit (RCU) around doing so. The SoC data supplies the ROM
 * function addresses and register details (struct adsp_otp_rom_data) and,
 * per field, how the ROM handles it (struct adsp_otp_rom_field).
 *
 * Assumptions, checked against the CCES register headers of the SC57x,
 * SC58x, SC59x, SC598 and SC84x and against the SC58x, SC59x and SC84x
 * boot ROMs:
 *
 *  - The OTPC register map and the fields used here (OTPC_CTL.READMODE,
 *    OTPC_SECU_STATE.PARTLOCK: 0 open, 1 locked, 2 "unlocked", and the
 *    PMC state in PMC_STAT[7:6], idle = 0b10) are the same on all of
 *    them. That PMC registers fault while READMODE is set was only seen
 *    on the SC589.
 *  - The ROM is called with 32-bit arguments and a 32-bit otp_data
 *    (pointers included): the Cortex-A5 parts only. The AArch64 ROMs of
 *    the SC598 and SC84x use 64-bit pointers and are not supported here.
 *  - The word otp_lock() programs, and its value, are SoC data
 *    (SC58x: 1 at 0x48c; SC84x: 0x30000 at 0x424).
 *  - On an "unlocked" part (PARTLOCK = 2) the ROM's otp_get() returns
 *    zeros for customer fields, reporting success; raw reads here do the
 *    same.
 *  - The ROM's boolean results do not reliably report OTPC errors
 *    (silicon anomaly 20000039), so the error bits of OTPC_STAT and
 *    PMC_STAT are checked here after every call.
 *  - The ROM functions take no OTPC instance: they drive the one OTPC at
 *    the SoC's fixed address (adsp_otp_rom_data.otpc_base). A device is
 *    only accepted for that OTPC, and claiming its registers keeps a
 *    second device for it from binding, so each OTPC has one device and
 *    one lock. ROM calls of all devices are serialized by stop_machine().
 *  - Linux runs as a secure bus requester (see sc5xx-rom.rst). This is
 *    not checked: SPU_SECURECHK, the register meant for it, lies in the
 *    upper half of the SPU's MMR space, where non-secure accesses fault
 *    (anomaly 20000003), so it could never report "non-secure" cleanly.
 *  - A little-endian kernel, like the ROM (see the Kconfig dependency).
 *
 * Acronyms: PMC = the OTPC's programming controller; MMR = Memory-Mapped
 * Register.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/adi/adsp-rom.h>
#include <linux/string.h>

#include "adsp-otp.h"

/* OTPC registers. 0x30-0x73 is a hole: any access to it aborts. */
#define ADSP_OTPC_CTL			0x00
#define ADSP_OTPC_CTL_READMODE		BIT(0)
#define ADSP_OTPC_STAT			0x04
#define ADSP_OTPC_TIMING		0x08
#define ADSP_OTPC_SECU_STATE		0x2c
#define ADSP_OTPC_SECU_PARTLOCK		GENMASK(1, 0)
#define ADSP_OTPC_SECU_OPEN		0
#define ADSP_OTPC_SECU_LOCKED		1
#define ADSP_OTPC_SECU_UNLOCKED		2
/* PMC registers: only accessible while OTPC_CTL.READMODE = 0 */
#define ADSP_OTPC_PMC_STAT		0x84
#define ADSP_OTPC_PMC_STAT_STATE	GENMASK(7, 6)
#define ADSP_OTPC_PMC_STATE_IDLE	2
#define ADSP_OTPC_PMC_MODE(idx)		(0x88 + 4 * (idx))
#define ADSP_OTPC_PMC_PGM_TMG(idx)	(0x98 + 4 * (idx))

/*
 * Half-words programmed per otp_pgm() call. Each takes four PMC
 * operations (the data and its three redundant copies), all with
 * interrupts disabled on every CPU, so a large field (e.g. 80 half-words
 * of DTCP_key_ecc) is split into several calls.
 */
#define ADSP_OTP_ROM_PGM_CHUNK		4

/**
 * struct adsp_otp_rom - backend state
 * @data: the SoC's ROM parameters
 * @rom: handle for adsp_rom_call()
 * @otpc: OTPC registers
 * @rcu: RCU registers
 * @pgm: the ROM's struct otp_data, the argument of otp_pgm()
 * @chunk: one field's worth of half-words for one otp_pgm() call
 * @scratch: one field's worth of words, e.g. for the lock word
 */
struct adsp_otp_rom {
	const struct adsp_otp_rom_data *data;
	struct adsp_rom *rom;
	void __iomem *otpc;
	struct regmap *rcu;
	u8 *pgm;
	u32 *chunk;
	u32 *scratch;
};

static struct adsp_otp_rom *to_rom(struct adsp_otp *otp)
{
	return otp->backend;
}

static bool adsp_otp_rom_call(struct adsp_otp *otp, unsigned long fn,
			      unsigned long arg0, unsigned long arg1,
			      unsigned long arg2)
{
	/* The ROM returns bool; only the low byte is defined */
	return adsp_rom_call(to_rom(otp)->rom, fn, arg0, arg1, arg2, 0) & 0xff;
}

static u32 adsp_otp_rom_stat(struct adsp_otp *otp)
{
	return readl(to_rom(otp)->otpc + ADSP_OTPC_STAT);
}

/*
 * Anomaly 20000039: the ROM does not report OTPC errors. Fail if an error
 * bit is set after the call that was clear before it. One that was
 * already set cannot be attributed to this call, and the bits are
 * read-only, so it is only reported.
 */
static int adsp_otp_rom_check_err(struct adsp_otp *otp, const char *reg,
				  u32 before, u32 after, u32 mask,
				  const char *what)
{
	if (after & mask & ~before) {
		dev_err(otp->dev, "%s: %s reports an error (%08x)\n", what, reg,
			after);
		return -EIO;
	}
	if (after & mask)
		dev_warn_once(otp->dev, "%s error bits stuck at %08x, cannot check them\n",
			      reg, after);

	return 0;
}

/* A ROM call that reads the OTP (or configures the OTPC): check OTPC_STAT */
static int adsp_otp_rom_read_call(struct adsp_otp *otp, const char *what,
				  unsigned long fn, unsigned long arg0,
				  unsigned long arg1, unsigned long arg2)
{
	const struct adsp_otp_rom_data *data = to_rom(otp)->data;
	u32 before = adsp_otp_rom_stat(otp);
	bool ok;
	int ret;

	ok = adsp_otp_rom_call(otp, fn, arg0, arg1, arg2);
	ret = adsp_otp_rom_check_err(otp, "OTPC_STAT", before,
				     adsp_otp_rom_stat(otp), data->stat_err,
				     what);
	if (!ok) {
		dev_err(otp->dev, "%s: the boot ROM reports a failure\n", what);
		return -EIO;
	}

	return ret;
}

static u32 adsp_otp_rom_partlock(struct adsp_otp *otp)
{
	return FIELD_GET(ADSP_OTPC_SECU_PARTLOCK,
			 readl(to_rom(otp)->otpc + ADSP_OTPC_SECU_STATE));
}

static int adsp_otp_rom_read_field(struct adsp_otp *otp,
				   const struct adsp_otp_field *field, u32 *buf)
{
	const struct adsp_otp_rom_data *data = to_rom(otp)->data;
	u32 partlock = adsp_otp_rom_partlock(otp);

	/*
	 * Like the ROM's otp_get() on an "unlocked" part, and, for fields
	 * that a locked part cannot read, without touching the OTP.
	 */
	if ((field->rom.open_only && partlock != ADSP_OTPC_SECU_OPEN) ||
	    (field->rom.get == ADSP_OTP_ROM_GET_RAW &&
	     partlock == ADSP_OTPC_SECU_UNLOCKED)) {
		memset(buf, 0, adsp_otp_field_bytes(field));
		return 0;
	}

	if (field->rom.get == ADSP_OTP_ROM_GET_RAW)
		return adsp_otp_rom_read_call(otp, field->name,
					      data->otp_get_data,
					      (unsigned long)buf,
					      field->offset, field->words);

	return adsp_otp_rom_read_call(otp, field->name, data->otp_get,
				      field->rom.cmd, (unsigned long)buf, 0);
}

/*
 * Read PMC_STAT; log the OTPC/PMC setup at debug level. The PMC registers
 * fault in read mode, so leave it around the accesses like the ROM does.
 */
static u32 adsp_otp_rom_pmc_stat(struct adsp_otp *otp)
{
	void __iomem *regs = to_rom(otp)->otpc;
	u32 ctl, stat;

	ctl = readl(regs + ADSP_OTPC_CTL);
	writel(ctl & ~ADSP_OTPC_CTL_READMODE, regs + ADSP_OTPC_CTL);
	readl(regs + ADSP_OTPC_CTL);	/* posted write must land first */

	stat = readl(regs + ADSP_OTPC_PMC_STAT);
	dev_dbg(otp->dev,
		"OTPC_STAT %08x TIMING %08x PMC_STAT %08x MODE0-3 %08x %08x %08x %08x PGM_TMG0-2 %08x %08x %08x\n",
		readl(regs + ADSP_OTPC_STAT), readl(regs + ADSP_OTPC_TIMING),
		stat,
		readl(regs + ADSP_OTPC_PMC_MODE(0)),
		readl(regs + ADSP_OTPC_PMC_MODE(1)),
		readl(regs + ADSP_OTPC_PMC_MODE(2)),
		readl(regs + ADSP_OTPC_PMC_MODE(3)),
		readl(regs + ADSP_OTPC_PMC_PGM_TMG(0)),
		readl(regs + ADSP_OTPC_PMC_PGM_TMG(1)),
		readl(regs + ADSP_OTPC_PMC_PGM_TMG(2)));

	writel(ctl, regs + ADSP_OTPC_CTL);
	readl(regs + ADSP_OTPC_CTL);

	return stat;
}

/*
 * Anomaly 20000084: simultaneous OTP accesses by several cores can hang a
 * core. Only program while every other core (the SHARC+ cores) is either
 * held in reset (core reset control and status both set) or idle, as its
 * software reports through the RCU message register when stopped (the
 * HRM's core reset handshake, used by the ADI remoteproc driver's stop).
 * This is checked again right before every programming ROM call; a core
 * started between that check and the call is not caught.
 */
static int adsp_otp_rom_check_program(struct adsp_otp *otp)
{
	const struct adsp_otp_rom_data *data = to_rom(otp)->data;
	struct regmap *rcu = to_rom(otp)->rcu;
	unsigned int crctl, crstat, msg;
	u32 held, idle;
	int ret;

	ret = regmap_read(rcu, data->rcu_crctl, &crctl);
	if (!ret)
		ret = regmap_read(rcu, data->rcu_crstat, &crstat);
	if (!ret)
		ret = regmap_read(rcu, data->rcu_msg, &msg);
	if (ret)
		return ret;

	held = crctl & crstat;
	idle = msg >> data->rcu_idle_shift;
	if (((held | idle) & data->rcu_cores) != data->rcu_cores) {
		dev_err(otp->dev,
			"other cores are running (RCU CRCTL %08x CRSTAT %08x MSG %08x): stop them before programming the OTP (anomaly 20000084)\n",
			crctl, crstat, msg);
		return -EBUSY;
	}

	return 0;
}

/*
 * A ROM call that programs the OTP: otp_pgm() or otp_lock().
 *
 * Before it: require the other cores in reset, reconfigure the OTPC with
 * otp_cfg() and require an idle PMC. The ROM waits for the PMC without a
 * timeout, with interrupts off, so this is the last point to fail
 * cleanly. After it: check the error bits of OTPC_STAT and PMC_STAT.
 */
static int adsp_otp_rom_pgm_call(struct adsp_otp *otp, const char *what,
				 unsigned long fn, unsigned long arg0)
{
	const struct adsp_otp_rom_data *data = to_rom(otp)->data;
	u32 pmc_before, pmc_after, stat_before, stat_after;
	bool ok;
	int ret;

	ret = adsp_otp_rom_check_program(otp);
	if (ret)
		return ret;

	ret = adsp_otp_rom_read_call(otp, what, data->otp_cfg, 0, 0, 0);
	if (ret)
		return ret;

	pmc_before = adsp_otp_rom_pmc_stat(otp);
	if (FIELD_GET(ADSP_OTPC_PMC_STAT_STATE, pmc_before) !=
	    ADSP_OTPC_PMC_STATE_IDLE) {
		dev_err(otp->dev, "%s: PMC not idle (PMC_STAT %08x)\n", what,
			pmc_before);
		return -EIO;
	}

	stat_before = adsp_otp_rom_stat(otp);
	ok = adsp_otp_rom_call(otp, fn, arg0, 0, 0);
	stat_after = adsp_otp_rom_stat(otp);
	pmc_after = adsp_otp_rom_pmc_stat(otp);

	ret = adsp_otp_rom_check_err(otp, "OTPC_STAT", stat_before,
				     stat_after, data->stat_err, what);
	if (!ret)
		ret = adsp_otp_rom_check_err(otp, "PMC_STAT", pmc_before,
					     pmc_after, data->pmc_err, what);
	if (!ok) {
		dev_err(otp->dev, "%s: the boot ROM reports a failure\n", what);
		return -EIO;
	}

	return ret;
}

/*
 * otp_pgm() always also handles the SoC's counter: it programs the count
 * given in otp_data, fails if that is below the current count, and does
 * nothing if it is equal. Get the current count, read with the ROM's own
 * decoding, so that otp_pgm() leaves the counter alone and reports the
 * result of programming the field.
 */
static int adsp_otp_rom_get_cntr(struct adsp_otp *otp, u8 *cntr)
{
	struct adsp_otp_rom *rom = to_rom(otp);
	int ret;

	*cntr = 0;
	if (!rom->data->cntr_cmd)
		return 0;

	memset(rom->scratch, 0, otp->max_words * sizeof(u32));
	ret = adsp_otp_rom_read_call(otp, "counter", rom->data->otp_get,
				     rom->data->cntr_cmd,
				     (unsigned long)rom->scratch, 0);
	if (ret)
		return ret;

	if (rom->scratch[0] > rom->data->cntr_max) {
		dev_err(otp->dev, "counter reads %u, more than %u\n",
			rom->scratch[0], rom->data->cntr_max);
		return -EIO;
	}
	*cntr = rom->scratch[0];

	return 0;
}

/* One otp_pgm() call: @buf for @field, nothing else (but the counter) */
static int adsp_otp_rom_pgm_one(struct adsp_otp *otp,
				const struct adsp_otp_field *field,
				const u32 *buf, u8 cntr)
{
	struct adsp_otp_rom *rom = to_rom(otp);
	u32 *member = (u32 *)(rom->pgm + field->rom.pgm_off);
	int ret;

	/* Null pointers and zero values are left alone by the ROM */
	memset(rom->pgm, 0, rom->data->pgm_size);
	if (field->rom.pgm == ADSP_OTP_ROM_PGM_VAL)
		*member = buf[0];
	else
		*member = (u32)(uintptr_t)buf;
	if (rom->data->cntr_cmd)
		rom->pgm[rom->data->cntr_pgm_off] = cntr;

	ret = adsp_otp_rom_pgm_call(otp, field->name, rom->data->otp_pgm,
				    (unsigned long)rom->pgm);

	/* A value field holds the data itself */
	memzero_explicit(rom->pgm, rom->data->pgm_size);
	return ret;
}

static int adsp_otp_rom_program_field(struct adsp_otp *otp,
				      const struct adsp_otp_field *field,
				      const u32 *halves)
{
	struct adsp_otp_rom *rom = to_rom(otp);
	unsigned int word, half, n = 0;
	u8 cntr;
	int ret;

	ret = adsp_otp_rom_get_cntr(otp, &cntr);
	if (ret)
		return ret;

	if (field->rom.pgm == ADSP_OTP_ROM_PGM_VAL)
		return adsp_otp_rom_pgm_one(otp, field, halves, cntr);

	/*
	 * The ROM skips half-words that are 0, so the field can be
	 * programmed a few half-words per call, to bound the time spent
	 * with interrupts disabled.
	 */
	memset(rom->chunk, 0, adsp_otp_field_bytes(field));
	for (word = 0; word < field->words && !ret; word++) {
		for (half = 0; half < 2 && !ret; half++) {
			u32 val = halves[word] & (0xffffU << (16 * half));

			if (!val)
				continue;
			rom->chunk[word] |= val;
			if (++n < ADSP_OTP_ROM_PGM_CHUNK)
				continue;

			ret = adsp_otp_rom_pgm_one(otp, field, rom->chunk, cntr);
			memset(rom->chunk, 0, adsp_otp_field_bytes(field));
			n = 0;
		}
	}
	if (!ret && n)
		ret = adsp_otp_rom_pgm_one(otp, field, rom->chunk, cntr);

	memzero_explicit(rom->chunk, adsp_otp_field_bytes(field));
	return ret;
}

/*
 * OTPC_SECU_STATE only changes at the next reset after otp_lock(), so a
 * pending lock is recognised from the lock word itself, read from the OTP:
 * pending once all the bits otp_lock() sets read back set.
 */
static int adsp_otp_rom_lock_state(struct adsp_otp *otp)
{
	struct adsp_otp_rom *rom = to_rom(otp);
	u32 partlock;
	int ret;

	partlock = adsp_otp_rom_partlock(otp);
	switch (partlock) {
	case ADSP_OTPC_SECU_OPEN:
		break;
	case ADSP_OTPC_SECU_LOCKED:
		return ADSP_OTP_LOCKED;
	case ADSP_OTPC_SECU_UNLOCKED:
		return ADSP_OTP_UNLOCKED;
	default:
		dev_err(otp->dev, "unknown PARTLOCK state %u\n", partlock);
		return -EIO;
	}

	rom->scratch[0] = 0;
	ret = adsp_otp_rom_read_call(otp, "lock word", rom->data->otp_get_data,
				     (unsigned long)rom->scratch,
				     rom->data->lock_offset, 1);
	if (ret)
		return ret;

	if ((rom->scratch[0] & rom->data->lock_mask) == rom->data->lock_mask)
		return ADSP_OTP_LOCK_PENDING;

	return ADSP_OTP_OPEN;
}

static int adsp_otp_rom_lock(struct adsp_otp *otp)
{
	return adsp_otp_rom_pgm_call(otp, "otp_lock", to_rom(otp)->data->otp_lock,
				     0);
}

/* Every writable field needs a slot in the ROM's otp_data */
static int adsp_otp_rom_validate(struct adsp_otp *otp)
{
	const struct adsp_otp_rom_data *data = otp->soc->backend_data;
	unsigned int i;

	if (data->cntr_cmd && data->cntr_pgm_off >= data->pgm_size) {
		dev_err(otp->dev, "invalid ROM counter data\n");
		return -EINVAL;
	}

	for (i = 0; i < otp->soc->nfields; i++) {
		const struct adsp_otp_field *field = &otp->soc->fields[i];

		if (field->writable &&
		    (field->rom.pgm == ADSP_OTP_ROM_PGM_NONE ||
		     field->rom.pgm_off + sizeof(u32) > data->pgm_size)) {
			dev_err(otp->dev, "invalid ROM data for field %s\n",
				field->name);
			return -EINVAL;
		}
	}

	return 0;
}

static int adsp_otp_rom_probe(struct adsp_otp *otp,
			      struct platform_device *pdev)
{
	struct device *dev = otp->dev;
	struct adsp_otp_rom *rom;
	struct resource *res;
	int ret;

	rom = devm_kzalloc(dev, sizeof(*rom), GFP_KERNEL);
	if (!rom)
		return -ENOMEM;
	rom->data = otp->soc->backend_data;
	otp->backend = rom;

	ret = adsp_otp_rom_validate(otp);
	if (ret)
		return ret;

	/*
	 * The ROM only drives the OTPC at otpc_base. Mapping claims the
	 * registers, so a second device for the same OTPC fails here.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	if (res->start != rom->data->otpc_base)
		return dev_err_probe(dev, -ENODEV,
				     "the boot ROM only drives the OTPC at %#x, not %pa\n",
				     rom->data->otpc_base, &res->start);
	rom->otpc = devm_ioremap_resource(dev, res);
	if (IS_ERR(rom->otpc))
		return PTR_ERR(rom->otpc);

	rom->rcu = syscon_regmap_lookup_by_phandle(dev->of_node, "adi,rcu");
	if (IS_ERR(rom->rcu))
		return dev_err_probe(dev, PTR_ERR(rom->rcu),
				     "cannot get the RCU\n");

	rom->rom = devm_adsp_rom_get(dev);
	if (IS_ERR(rom->rom))
		return dev_err_probe(dev, PTR_ERR(rom->rom),
				     "cannot call the boot ROM\n");

	/* Handed to the ROM: kmalloc()ed, never on the stack */
	rom->pgm = devm_kzalloc(dev, rom->data->pgm_size, GFP_KERNEL);
	rom->chunk = devm_kcalloc(dev, otp->max_words, sizeof(u32),
				  GFP_KERNEL);
	rom->scratch = devm_kcalloc(dev, otp->max_words, sizeof(u32),
				    GFP_KERNEL);
	if (!rom->pgm || !rom->chunk || !rom->scratch)
		return -ENOMEM;

	if (adsp_otp_rom_stat(otp) & rom->data->stat_err)
		dev_warn(dev, "OTPC_STAT reports an earlier error (%08x)\n",
			 adsp_otp_rom_stat(otp));

	return 0;
}

static const struct adsp_otp_ops adsp_otp_rom_ops = {
	.probe = adsp_otp_rom_probe,
	.check_program = adsp_otp_rom_check_program,
	.lock_state = adsp_otp_rom_lock_state,
	.read_field = adsp_otp_rom_read_field,
	.program_field = adsp_otp_rom_program_field,
	.lock = adsp_otp_rom_lock,
};

static int adsp_otp_rom_platform_probe(struct platform_device *pdev)
{
	return adsp_otp_probe(pdev, &adsp_otp_rom_ops);
}

/* The SoCs whose OTP this backend drives, with their SoC data */
static const struct of_device_id adsp_otp_rom_of_match[] = {
	{ .compatible = "adi,adsp-sc589-otp", .data = &adsp_otp_sc589_data },
	{ }
};
MODULE_DEVICE_TABLE(of, adsp_otp_rom_of_match);

static struct platform_driver adsp_otp_rom_driver = {
	.probe = adsp_otp_rom_platform_probe,
	.driver = {
		.name = "adsp-otp",
		.of_match_table = adsp_otp_rom_of_match,
		.dev_groups = adsp_otp_groups,
	},
};
module_platform_driver(adsp_otp_rom_driver);
