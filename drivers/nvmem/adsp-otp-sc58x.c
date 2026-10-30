// SPDX-License-Identifier: GPL-2.0
/*
 * ADI ADSP OTP driver: ADSP-SC58x data and lock conditions.
 *
 * Sources: CCES 3.1.0 defSC58x_rom_jumptable.h, cdefSC58x_rom.h,
 * cdefSC58x_rom_otpLayout.h; the ADSP-SC58x/ADSP-2158x Hardware Reference
 * Manual and silicon anomaly list; EE-366 and EE-384; the SC58x boot ROM
 * (addresses below) and its source and OTP layout generator
 * (genGriffinSrc.py).
 *
 * Acronyms: HUK = Hardware Unique Key; EK = Endorsement Key; DTCP =
 * Digital Transmission Content Protection.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "adsp-otp.h"

/*
 * struct otp_data from cdefSC58x_rom_otpLayout.h, the argument of
 * otp_pgm(): a pointer per field, except antiroll_nv_cntr and
 * bootModeDisable which are values. "Any pointers left null will not be
 * written, any data values of 0 will not be written." The ROM is 32-bit
 * ARM: pointers are u32 here so that the layout is checkable everywhere.
 * bootModeDisable is "uint32_t bootModeDisable:8" in the header; it
 * occupies the low byte of the u32 at the same offset.
 */
struct adsp_sc58x_rom_otp_data {
	u32 huk;
	u32 DTCP_key_ecc;
	u32 DTCP_key_cont;
	u32 DTCP_key_dev;
	u32 pvt_128key0;
	u32 pvt_128key1;
	u32 pvt_128key2;
	u32 pvt_128key3;
	u32 ek;
	u32 secure_emu_key;
	u32 public_key0;
	u32 public_key1;
	u32 boot_info;
	u8 antiroll_nv_cntr;
	u32 gp1;
	u32 bootModeDisable;
	u32 preboot_ddr_cfg;
	u32 stageID;
};

static_assert(offsetof(struct adsp_sc58x_rom_otp_data, antiroll_nv_cntr) == 52);
static_assert(offsetof(struct adsp_sc58x_rom_otp_data, gp1) == 56);
static_assert(offsetof(struct adsp_sc58x_rom_otp_data, bootModeDisable) == 60);
static_assert(offsetof(struct adsp_sc58x_rom_otp_data, stageID) == 68);
static_assert(sizeof(struct adsp_sc58x_rom_otp_data) == 72);

#define SC58X_PTR(_name, _off, _words, _cmd)				\
	{ .name = #_name, .offset = _off, .words = _words,		\
	  .writable = true,						\
	  .rom = { .cmd = _cmd, .get = ADSP_OTP_ROM_GET_CMD,		\
		   .pgm = ADSP_OTP_ROM_PGM_PTR,				\
		   .pgm_off = offsetof(struct adsp_sc58x_rom_otp_data,	\
				       _name) } }

/*
 * Offsets as ROM_OTP_<name> in cdefSC58x_rom_otpLayout.h: the customer
 * OTP area (0x000-0x37f) and the factory serial number.
 */
static const struct adsp_otp_field adsp_sc58x_otp_fields[] = {
	/* Key material: huk .. secure_emu_key */
	SC58X_PTR(huk,             0x000,  8,  1),
	SC58X_PTR(DTCP_key_ecc,    0x020, 40,  2),
	SC58X_PTR(DTCP_key_cont,   0x0c0, 10,  3),
	SC58X_PTR(DTCP_key_dev,    0x0e8, 32,  4),
	SC58X_PTR(pvt_128key0,     0x168,  4,  5),
	SC58X_PTR(pvt_128key1,     0x178,  4,  6),
	SC58X_PTR(pvt_128key2,     0x188,  4,  7),
	SC58X_PTR(pvt_128key3,     0x198,  4,  8),
	SC58X_PTR(ek,              0x1a8,  8,  9),
	SC58X_PTR(secure_emu_key,  0x1c8,  4, 10),
	/* 0x1d8-0x1ff: reserved */
	SC58X_PTR(public_key0,     0x200, 16, 11),
	SC58X_PTR(public_key1,     0x240, 16, 12),
	SC58X_PTR(boot_info,       0x280, 16, 13),
	/*
	 * A counter the ROM advances from a count value: read-only here.
	 * otp_get() returns the decoded count (otp_get_cntr32()), so read
	 * the raw words, which is what the NVMEM offsets promise.
	 */
	{ .name = "antiroll_nv_cntr", .offset = 0x2c0, .words = 16,
	  .rom = { .get = ADSP_OTP_ROM_GET_RAW,
		   .pgm = ADSP_OTP_ROM_PGM_NONE } },
	SC58X_PTR(gp1,             0x300, 16, 16),
	/*
	 * The SC58x ROM keeps the 8-bit value in the low byte of this word
	 * (otp_pgm() at 0x26b0, otp_get() at 0x24ec), and its boot code
	 * reads it there through otp_get(), not in byte 0x343 as the HRM's
	 * OTP layout table (and ADI's layout generator) shows. 0x343 is
	 * where the SC59x ROM keeps it.
	 */
	{ .name = "bootModeDisable", .offset = 0x340, .words = 1,
	  .writable = true, .val_mask = GENMASK(7, 0),
	  .rom = { .cmd = 17, .get = ADSP_OTP_ROM_GET_CMD,
		   .pgm = ADSP_OTP_ROM_PGM_VAL,
		   .pgm_off = offsetof(struct adsp_sc58x_rom_otp_data,
				       bootModeDisable) } },
	SC58X_PTR(preboot_ddr_cfg, 0x344, 12, 18),
	SC58X_PTR(stageID,         0x374,  2, 19),
	/* 0x37c-0x37f: reserved */
	/*
	 * Factory serial number, in the ADI area: read-only. A locked part
	 * cannot read it (anomaly 20000108), so it reads as zero then.
	 */
	{ .name = "fsn", .offset = 0x380, .words = 4,
	  .rom = { .get = ADSP_OTP_ROM_GET_RAW, .open_only = true,
		   .pgm = ADSP_OTP_ROM_PGM_NONE } },
};

/*
 * Half-words of writable fields that writes must leave alone.
 *
 * boot_info word 15 holds otpTiming, the OTP read timing override:
 * otp_cfg(), run on every boot and by this driver before every
 * programming operation, loads its low byte into OTPC_TIMING when its
 * high byte is non-zero. A wrong value can leave the OTP unreadable, and
 * the ROM then stops booting. The SC58x ROM reads the override from the
 * low half of the word (otp_get() of command 14), ADI's layout from the
 * high half; the whole word is guarded. CCES exposes it as
 * ADI_ROM_OTP_BOOT_INFO.otpReadTiming.
 *
 * stageID is 48 bits; the half-word after it is reserved.
 */
static const struct adsp_otp_guard adsp_sc58x_otp_guards[] = {
	{ 0x2bc, 4, true, "boot_info word 15, otpTiming (OTP read timing override)" },
	{ 0x37a, 2, false, "reserved half-word of stageID" },
};

#define SC58X_PUBLIC_KEY0	0x200
#define SC58X_PUBLIC_KEY1	0x240
#define SC58X_BOOT_INFO		0x280
#define SC58X_SECURE_EMU_KEY	0x1c8
#define SC58X_GP1		0x300
#define SC58X_FSN		0x380

/* boot_info words 8..12 hold ADI_ROM_OTP_BOOT_CFG (cdefSC58x_rom.h) */
#define SC58X_KEYINV_WORD	10
#define SC58X_PUBKEY0_INV	BIT(0)		/* pubkey0Inv */
#define SC58X_PUBKEY1_INV	BIT(16)		/* pubkey1Inv */

/* True if @field is blank (all 0) */
static int adsp_sc58x_field_blank(struct adsp_otp *otp,
				  const struct adsp_otp_field *field,
				  bool *blank)
{
	int ret = adsp_otp_read_field(otp, field);

	if (!ret)
		*blank = !memchr_inv(otp->rbuf, 0, adsp_otp_field_bytes(field));
	return ret;
}

/*
 * Anomaly 20000108: a locked part cannot read its factory serial number.
 * The workaround is to store it in OTP that remains readable, e.g. gp1,
 * before locking. Warn unless gp1 holds it, at any word offset.
 */
static int adsp_sc58x_check_fsn(struct adsp_otp *otp)
{
	const struct adsp_otp_field *fsn_field, *gp1;
	u32 fsn[4];
	unsigned int i;
	int ret;

	fsn_field = adsp_otp_field_at(otp, SC58X_FSN);
	gp1 = adsp_otp_field_at(otp, SC58X_GP1);
	if (WARN_ON(!fsn_field || !gp1 ||
		    adsp_otp_field_bytes(fsn_field) != sizeof(fsn)))
		return -EINVAL;

	ret = adsp_otp_read_field(otp, fsn_field);
	if (ret)
		return ret;
	memcpy(fsn, otp->rbuf, sizeof(fsn));
	if (!memchr_inv(fsn, 0, sizeof(fsn))) {
		dev_warn(otp->dev, "fsn reads blank\n");
		return 0;
	}

	ret = adsp_otp_read_field(otp, gp1);
	if (ret)
		return ret;
	for (i = 0; i + ARRAY_SIZE(fsn) <= gp1->words; i++)
		if (!memcmp(&otp->rbuf[i], fsn, sizeof(fsn)))
			return 0;

	dev_warn(otp->dev, "fsn is not stored in gp1: a locked part cannot read it (anomaly 20000108)\n");
	return 0;
}

/*
 * A locked part only boots secure boot streams, authenticated with the
 * public key selected by boot_info; any other stream, or a failed
 * authentication, idles the core (EE-366, "Locking the Part"). Like the
 * ROM, the key is the first one not invalidated, whether programmed or
 * not (EE-384, "Public and Encryption (Private) Key in OTP Space"; SC58x
 * ROM at 0x355a). Refuse to lock unless that key is programmed, and warn
 * if the secure emulation key, which debug access then needs, is blank.
 */
static int adsp_sc58x_lock_precheck(struct adsp_otp *otp)
{
	const struct adsp_otp_field *boot_info, *key, *emu_key;
	bool blank;
	u32 keyinv;
	int ret;

	boot_info = adsp_otp_field_at(otp, SC58X_BOOT_INFO);
	emu_key = adsp_otp_field_at(otp, SC58X_SECURE_EMU_KEY);
	if (WARN_ON(!boot_info || !emu_key))
		return -EINVAL;

	ret = adsp_otp_read_field(otp, boot_info);
	if (ret)
		return ret;
	keyinv = otp->rbuf[SC58X_KEYINV_WORD];

	if (!(keyinv & SC58X_PUBKEY0_INV)) {
		key = adsp_otp_field_at(otp, SC58X_PUBLIC_KEY0);
	} else if (!(keyinv & SC58X_PUBKEY1_INV)) {
		key = adsp_otp_field_at(otp, SC58X_PUBLIC_KEY1);
	} else {
		dev_err(otp->dev, "refusing to lock: all public keys are invalidated\n");
		return -EPERM;
	}
	if (WARN_ON(!key))
		return -EINVAL;

	ret = adsp_sc58x_field_blank(otp, key, &blank);
	if (ret)
		return ret;
	if (blank) {
		dev_err(otp->dev, "refusing to lock: %s, the key secure boot would use, is blank\n",
			key->name);
		return -EPERM;
	}

	ret = adsp_sc58x_field_blank(otp, emu_key, &blank);
	if (ret)
		return ret;
	if (blank)
		dev_warn(otp->dev, "%s is blank: no debug access once locked\n",
			 emu_key->name);

	ret = adsp_sc58x_check_fsn(otp);
	if (ret)
		return ret;

	dev_warn(otp->dev, "secure boot key: %s\n", key->name);
	return 0;
}

static const struct adsp_otp_rom_data adsp_sc589_rom_data = {
	.otpc_base	= 0x31011000,	/* OTPC0 */
	.otp_get	= 0x00000151,	/* FUNC_ROM_OTP_GET */
	.otp_get_data	= 0x00000149,	/* FUNC_ROM_OTP_GET_DATA */
	.otp_pgm	= 0x00000139,	/* FUNC_ROM_OTPPGM */
	.otp_lock	= 0x00000141,	/* FUNC_ROM_OTP_LOCK */
	.otp_cfg	= 0x00000169,	/* FUNC_ROM_OTP_CFG */
	.pgm_size	= sizeof(struct adsp_sc58x_rom_otp_data),
	/* HRM OTP layout "lock"; otp_lock() at 0x3cd4 programs 1 there */
	.lock_offset	= 0x48c,
	.lock_mask	= BIT(0),
	/*
	 * otp_pgm() always passes antiroll_nv_cntr to otp_pgm_cntr32(),
	 * which fails if it is below the current count (0x2c0, at most 32)
	 */
	.cntr_cmd	= 15,
	.cntr_pgm_off	= offsetof(struct adsp_sc58x_rom_otp_data,
				   antiroll_nv_cntr),
	.cntr_max	= 32,
	/* As the ROM's pmc_error(): OTPC_STAT.ADDRERR and PMC_STAT[5:4] */
	.stat_err	= GENMASK(14, 13),
	.pmc_err	= GENMASK(5, 4),
	/*
	 * RCU0_CRCTL/RCU0_CRSTAT bits 1 and 2: SHARC+ cores 1 and 2 held in
	 * reset; RCU0_MSG bits 9 and 10 (C1IDLE, C2IDLE): idle
	 */
	.rcu_crctl	= 0x08,
	.rcu_crstat	= 0x0c,
	.rcu_msg	= 0x60,
	.rcu_cores	= GENMASK(2, 1),
	.rcu_idle_shift	= 8,
};

const struct adsp_otp_soc_data adsp_otp_sc589_data = {
	.fields		= adsp_sc58x_otp_fields,
	.nfields	= ARRAY_SIZE(adsp_sc58x_otp_fields),
	.size		= 0x390,	/* end of fsn */
	.guards		= adsp_sc58x_otp_guards,
	.nguards	= ARRAY_SIZE(adsp_sc58x_otp_guards),
	.backend_data	= &adsp_sc589_rom_data,
	.lock_precheck	= adsp_sc58x_lock_precheck,
};
