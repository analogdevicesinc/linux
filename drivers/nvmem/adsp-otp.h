/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADI ADSP OTP driver: interfaces between the generic core (adsp-otp.c),
 * the per-SoC data (adsp-otp-<soc>.c) and the access backends
 * (adsp-otp-rom.c). See Documentation/arch/arm/adi/adsp-otp.rst.
 *
 * A backend is a platform driver: its of_match_table lists the compatibles
 * it drives, each with the SoC data as match data, and its probe hands its
 * (private) adsp_otp_ops to adsp_otp_probe(). The compatible thus selects
 * both the SoC data and the backend.
 *
 * Copyright 2026 Analog Devices Inc.
 */
#ifndef __NVMEM_ADSP_OTP_H
#define __NVMEM_ADSP_OTP_H

#include <linux/mutex.h>
#include <linux/types.h>

struct adsp_otp;
struct attribute_group;
struct device;
struct platform_device;

/* struct adsp_otp_rom_field.get */
enum adsp_otp_rom_get {
	ADSP_OTP_ROM_GET_CMD,	/* otp_get(cmd): the ROM's view of the field */
	ADSP_OTP_ROM_GET_RAW,	/* otp_get_data(): the raw OTP words */
};

/* struct adsp_otp_rom_field.pgm */
enum adsp_otp_rom_pgm {
	ADSP_OTP_ROM_PGM_NONE,	/* not programmable through the ROM */
	ADSP_OTP_ROM_PGM_PTR,	/* pointer to the data in otp_data */
	ADSP_OTP_ROM_PGM_VAL,	/* 32-bit value in otp_data */
};

/**
 * struct adsp_otp_rom_field - how the boot ROM API handles a field
 * @cmd: OTPCMD value that makes the ROM's otp_get() read this field
 *	(ADSP_OTP_ROM_GET_CMD only)
 * @get: how the field is read (ADSP_OTP_ROM_GET_*)
 * @pgm: how otp_pgm() is told to program this field (ADSP_OTP_ROM_PGM_*)
 * @pgm_off: byte offset of the field's member in the ROM's otp_data
 * @open_only: only readable on an open part; reads as zero once the part
 *	is locked (e.g. the factory serial number, anomaly 20000108)
 */
struct adsp_otp_rom_field {
	u8 cmd;
	u8 get;
	u8 pgm;
	u8 pgm_off;
	bool open_only;
};

/**
 * struct adsp_otp_field - one field of the OTP area exposed by the driver
 * @name: field name, also the NVMEM cell name
 * @offset: byte offset in the OTP area
 * @words: size in 32-bit words
 * @writable: may be programmed through this driver
 * @val_mask: if non-zero, the field is a single word whose value may only
 *	use these bits
 * @rom: how the boot ROM backend accesses the field
 */
struct adsp_otp_field {
	const char *name;
	u16 offset;
	u8 words;
	bool writable;
	u32 val_mask;
	struct adsp_otp_rom_field rom;
};

/**
 * struct adsp_otp_guard - half-words that writes must leave alone
 * @offset: byte offset in the OTP area, 2-byte aligned
 * @bytes: size in bytes, a multiple of 2
 * @unsafe: may be written when the allow_unsafe_fields module parameter is
 *	set; otherwise never writable through this driver
 * @what: description, for diagnostics
 *
 * For half-words inside writable fields that the ROM or the hardware
 * treat specially, e.g. an override of the OTP read timing.
 */
struct adsp_otp_guard {
	u16 offset;
	u16 bytes;
	bool unsafe;
	const char *what;
};

/* What adsp_otp_ops.lock_state() reports */
enum adsp_otp_lock_state {
	ADSP_OTP_OPEN,		/* not locked */
	ADSP_OTP_LOCK_PENDING,	/* lock programmed, effective at reset */
	ADSP_OTP_LOCKED,	/* locked */
	ADSP_OTP_UNLOCKED,	/* "unlocked" part (e.g. failure analysis) */
};

/**
 * struct adsp_otp_ops - how a backend reaches the OTP
 * @probe: set up the backend (map registers, get the ROM, ...) and store
 *	its state in adsp_otp.backend
 * @check_program: whether the OTP may be programmed now (0 or -errno),
 *	e.g. that no other core can access it
 * @lock_state: an enum adsp_otp_lock_state, or -errno. Must reflect the
 *	hardware, so that a pending lock survives rebinding the driver
 * @read_field: read all of @field into @buf (adsp_otp_field_bytes() bytes)
 * @program_field: program the non-zero half-words of @halves into @field;
 *	all others are left alone
 * @lock: lock the part
 *
 * All but @probe are called with the device's mutex (adsp_otp.lock) held,
 * so a backend needs no locking of its own for state of that device; state
 * shared between devices (e.g. hardware that several devices reach) needs
 * the backend's own care. Buffers passed in are kmalloc()ed.
 */
struct adsp_otp_ops {
	int (*probe)(struct adsp_otp *otp, struct platform_device *pdev);
	int (*check_program)(struct adsp_otp *otp);
	int (*lock_state)(struct adsp_otp *otp);
	int (*read_field)(struct adsp_otp *otp,
			  const struct adsp_otp_field *field, u32 *buf);
	int (*program_field)(struct adsp_otp *otp,
			     const struct adsp_otp_field *field,
			     const u32 *halves);
	int (*lock)(struct adsp_otp *otp);
};

/**
 * struct adsp_otp_soc_data - SoC specific parameters
 * @fields: OTP fields, sorted by offset, not overlapping
 * @nfields: number of @fields
 * @size: size of the NVMEM device: end of the last field
 * @guards: half-words inside writable fields that writes must avoid
 * @nguards: number of @guards
 * @backend_data: parameters for the backend that drives this SoC's OTP,
 *	e.g. ROM function addresses
 * @lock_precheck: optional: the SoC's conditions for locking the part
 *	(0 or -errno), e.g. that secure boot has a key to use afterwards
 */
struct adsp_otp_soc_data {
	const struct adsp_otp_field *fields;
	unsigned int nfields;
	unsigned int size;
	const struct adsp_otp_guard *guards;
	unsigned int nguards;
	const void *backend_data;
	int (*lock_precheck)(struct adsp_otp *otp);
};

/**
 * struct adsp_otp - driver state, one per OTP device
 * @dev: the device
 * @soc: SoC specific parameters
 * @ops: the backend's operations, from adsp_otp_probe()
 * @backend: the backend's state, set by adsp_otp_ops.probe()
 * @lock: serializes all access to this OTP: every backend call, every
 *	check-then-program sequence and the lock operation. The NVMEM core
 *	does not serialize reg_read()/reg_write().
 * @max_words: size of the largest field, in 32-bit words
 * @rbuf: one field, as read by adsp_otp_read_field()
 * @stage: data to program, indexed like the OTP area
 * @delta: half-words of one field still to program
 * @read_only: DT "read-only": no programming, no locking
 */
struct adsp_otp {
	struct device *dev;
	const struct adsp_otp_soc_data *soc;
	const struct adsp_otp_ops *ops;
	void *backend;
	struct mutex lock;		/* serializes all access to this OTP */
	unsigned int max_words;
	u32 *rbuf;
	u32 *stage;
	u32 *delta;
	bool read_only;
};

static inline unsigned int
adsp_otp_field_bytes(const struct adsp_otp_field *field)
{
	return field->words * sizeof(u32);
}

/* Core helpers for SoC code (lock_precheck); call with otp->lock held */
const struct adsp_otp_field *adsp_otp_field_at(struct adsp_otp *otp,
					       unsigned int pos);
int adsp_otp_read_field(struct adsp_otp *otp,
			const struct adsp_otp_field *field);

/* Core entry points for backends (their platform drivers) */
int adsp_otp_probe(struct platform_device *pdev,
		   const struct adsp_otp_ops *ops);
extern const struct attribute_group *adsp_otp_groups[];

/**
 * struct adsp_otp_rom_data - parameters of the boot ROM backend
 * @otpc_base: physical address of the OTPC that the ROM functions drive;
 *	they take no instance argument, so this is the only one they can
 * @otp_get: ROM address of bool otp_get(OTPCMD cmd, uint32_t data[])
 * @otp_get_data: ROM address of bool otp_get_data(uint32_t data[],
 *	uint16_t offset, uint16_t size): raw read of @size 32-bit words at
 *	byte offset @offset
 * @otp_pgm: ROM address of bool otp_pgm(otp_data *data)
 * @otp_lock: ROM address of bool otp_lock(void)
 * @otp_cfg: ROM address of bool otp_cfg(void)
 * @pgm_size: size of the ROM's struct otp_data
 * @lock_offset: OTP offset of the word that otp_lock() programs (outside
 *	the customer area)
 * @lock_mask: the bits otp_lock() sets in that word; the lock is pending
 *	or effective when all of them read back set
 * @cntr_cmd: if non-zero, otp_pgm() always also "programs" a counter
 *	from a u8 count in otp_data, and fails if that count is below the
 *	current one. This is the OTPCMD that reads the current count, which
 *	the backend passes back so that otp_pgm() leaves the counter alone
 * @cntr_pgm_off: byte offset of that u8 count in the ROM's otp_data
 * @cntr_max: the largest count the ROM accepts
 * @stat_err: error bits of OTPC_STAT (silicon anomaly 20000039: the ROM
 *	API does not report them)
 * @pmc_err: error bits of OTPC_PMC_STAT
 * @rcu_crctl: offset of the core reset control register in the RCU
 * @rcu_crstat: offset of the core reset status register in the RCU
 * @rcu_msg: offset of the RCU message register
 * @rcu_cores: the other cores, as bits of the core reset registers, that
 *	must be held in reset or idle while the OTP is programmed
 * @rcu_idle_shift: shift from a core's bit in @rcu_cores to its idle flag
 *	in the RCU message register
 */
struct adsp_otp_rom_data {
	u32 otpc_base;
	unsigned long otp_get;
	unsigned long otp_get_data;
	unsigned long otp_pgm;
	unsigned long otp_lock;
	unsigned long otp_cfg;
	unsigned int pgm_size;
	u16 lock_offset;
	u32 lock_mask;
	u8 cntr_cmd;
	u8 cntr_pgm_off;
	u8 cntr_max;
	u32 stat_err;
	u32 pmc_err;
	unsigned int rcu_crctl;
	unsigned int rcu_crstat;
	unsigned int rcu_msg;
	u32 rcu_cores;
	u8 rcu_idle_shift;
};

/* SoCs */
extern const struct adsp_otp_soc_data adsp_otp_sc589_data;

#endif /* __NVMEM_ADSP_OTP_H */
