// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#include "dpu95.h"

#define STATICCONTROL		0x8
#define  SHDTOKSEL_MASK		0x70
#define  SHDTOKSEL_SW		(0x4 << 0x4)
#define  SHDTOKSEL_SW_PRIM	(0x5 << 0x4)
#define  SHDTOKSEL_SW_SEC	(0x6 << 0x4)
#define  SHDLDSEL_MASK		0xe
#define  SHDLDSEL_SW		(0x4 << 0x1)
#define  SHDLDSEL_SW_PRIM	(0x5 << 0x1)
#define  SHDLDSEL_SW_SEC	(0x6 << 0x1)

#define CONTROLTRIGGER		0xc
#define  SEQUENCE_COMPLETE	BIT(1)
#define  SHDTOKGEN		BIT(0)

#define MODECONTROL		0x10

#define ALPHACONTROL		0x14
#define  ALPHAMASKENABLE	BIT(0)

#define BLENDCONTROL		0x18
#define  PRIM_C_BLD_FUNC_SHIFT	0
#define  SEC_C_BLD_FUNC_SHIFT	4
#define  PRIM_A_BLD_FUNC_SHIFT	8
#define  SEC_A_BLD_FUNC_SHIFT	12
#define  BLENDALPHA(n)		((n) << 16)

#define TIMEOUT_FEEDBACK	0x1c
#define LOCKUP_CLEAR		0x20
#define DELAY_COUNTER_EN	0x24
#define DELAY_COUNTER_PRIM	0x28
#define DELAY_COUNTER_SEC	0x2c
#define ERROR_COUNTER_PRIM	0x30
#define ERROR_COUNTER_SEC	0x34
#define SOURCE_STATUS		0x38
#define SOURCE_STATUS_CLEAR	0x3c
#define PRIMCONTROLWORD		0x40
#define SECCONTROLWORD		0x44

enum db_blendcontrol_mode {
	DB_BLENDCONTROL_ZERO,
	DB_BLENDCONTROL_ONE,
	DB_BLENDCONTROL_PRIM_ALPHA,
	DB_BLENDCONTROL_ONE_MINUS_PRIM_ALPHA,
	DB_BLENDCONTROL_SEC_ALPHA,
	DB_BLENDCONTROL_ONE_MINUS_SEC_ALPHA,
	DB_BLENDCONTROL_CONST_ALPHA,
	DB_BLENDCONTROL_ONE_MINUA_CONST_ALPHA,
};

struct dpu95_domainblend {
	void __iomem *base;
	int id;
	unsigned int index;
	struct dpu95_soc *dpu;
};

static inline u32 dpu95_db_read(struct dpu95_domainblend *db,
				unsigned int offset)
{
	return readl(db->base + offset);
}

static inline void dpu95_db_write(struct dpu95_domainblend *db,
				  unsigned int offset, u32 value)
{
	writel(value, db->base + offset);
}

static inline void dpu95_db_write_mask(struct dpu95_domainblend *db,
				       unsigned int offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = dpu95_db_read(db, offset);
	tmp &= ~mask;
	dpu95_db_write(db, offset, tmp | value);
}

static void dpu95_db_enable_shden(struct dpu95_domainblend *db)
{
	dpu95_db_write_mask(db, STATICCONTROL, SHDEN, SHDEN);
}

static void dpu95_db_shtoksel_init(struct dpu95_domainblend *db)
{
	dpu95_db_write_mask(db, STATICCONTROL, SHDTOKSEL_MASK, SHDTOKSEL_SW);
}

static void dpu95_db_shdldsel_init(struct dpu95_domainblend *db)
{
	dpu95_db_write_mask(db, STATICCONTROL, SHDLDSEL_MASK, SHDLDSEL_SW);
}

void dpu95_db_shdtokgen(struct dpu95_domainblend *db)
{
	dpu95_db_write(db, CONTROLTRIGGER, SHDTOKGEN);
}

void dpu95_db_modecontrol(struct dpu95_domainblend *db,
			  enum dpu95_db_modecontrol m)
{
	dpu95_db_write(db, MODECONTROL, m);
}

void dpu95_db_alphamaskmode_enable(struct dpu95_domainblend *db)
{
	dpu95_db_write_mask(db, ALPHACONTROL, ALPHAMASKENABLE, ALPHAMASKENABLE);
}

void dpu95_db_alphamaskmode_disable(struct dpu95_domainblend *db)
{
	dpu95_db_write_mask(db, ALPHACONTROL, ALPHAMASKENABLE, 0);
}

static void dpu95_db_blendcontrol_init(struct dpu95_domainblend *db)
{
	dpu95_db_write(db, BLENDCONTROL,
		       (DB_BLENDCONTROL_ZERO << SEC_A_BLD_FUNC_SHIFT) |
		       (DB_BLENDCONTROL_ZERO << PRIM_A_BLD_FUNC_SHIFT) |
		       (DB_BLENDCONTROL_ONE  << SEC_C_BLD_FUNC_SHIFT) |
		       (DB_BLENDCONTROL_ZERO << PRIM_C_BLD_FUNC_SHIFT));
}

struct dpu95_domainblend *dpu95_db_get(struct dpu95_soc *dpu, int id)
{
	struct dpu95_domainblend *db;
	int i;

	for (i = 0; i < ARRAY_SIZE(dpu->db); i++) {
		db = dpu->db[i];
		if (db->id == id)
			break;
	}

	if (i == ARRAY_SIZE(dpu->db))
		return ERR_PTR(-EINVAL);

	return db;
}
EXPORT_SYMBOL_GPL(dpu95_db_get);

void dpu95_db_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	struct dpu95_domainblend *db = dpu->db[index];

	dpu95_db_enable_shden(db);
	dpu95_db_shtoksel_init(db);
	dpu95_db_shdldsel_init(db);
	dpu95_db_modecontrol(db, DB_MODE_PRIMARY);
	dpu95_db_alphamaskmode_disable(db);
	dpu95_db_blendcontrol_init(db);
}

int dpu95_db_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long unused, unsigned long base)
{
	struct dpu95_domainblend *db;

	db = devm_kzalloc(dpu->dev, sizeof(*db), GFP_KERNEL);
	if (!db)
		return -ENOMEM;

	dpu->db[index] = db;

	db->base = devm_ioremap(dpu->dev, base, SZ_128);
	if (!db->base)
		return -ENOMEM;

	db->dpu = dpu;
	db->id = id;
	db->index = index;

	return 0;
}
