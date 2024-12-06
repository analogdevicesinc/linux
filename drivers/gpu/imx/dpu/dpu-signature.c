/*
 * Copyright 2019,2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include "dpu-prv.h"

#define STATICCONTROL		0x8
#define SHDLDSEL		BIT(4)
#define LOCAL			0
#define GLOBAL			BIT(4)
#define PANICCOLOR		0xC
#define EVALCONTRL(n)		(0x10 + (n) * 0x24)
#define ENGLOBALPANIC		BIT(17)
#define ENLOCALPANIC		BIT(16)
#define ALPHAINV		BIT(9)
#define ALPHAMASK		BIT(8)
#define ENCRC			BIT(1)
#define ENEVALWIN		BIT(0)
#define EVALUPPERLEFT(n)	(0x14 + (n) * 0x24)
#define EVALLOWERRIGHT(n)	(0x18 + (n) * 0x24)
#define YEVAL(y)		(((y) & 0x3FFF) << 16)
#define XEVAL(x)		((x) & 0x3FFF)
#define SIGCRCREDREF(n)		(0x1C + (n) * 0x24)
#define SIGCRCGREENREF(n)	(0x20 + (n) * 0x24)
#define SIGCRCBLUEREF(n)	(0x24 + (n) * 0x24)
#define SIGCRCRED(n)		(0x28 + (n) * 0x24)
#define SIGCRCGREEN(n)		(0x2C + (n) * 0x24)
#define SIGCRCBLUE(n)		(0x30 + (n) * 0x24)
#define SHADOWLOAD		0x130
#define SHDLDREQ(n)		BIT(n)
#define CONTINUOUSMODE		0x134
#define ENCONT			BIT(0)
#define SOFTWAREKICK		0x138
#define KICK			BIT(0)
#define STATUS			0x13C
#define STSSIGIDLE		BIT(20)
#define STSSIGVALID		BIT(16)
#define STSSIGERROR(n)		BIT(n)
#define STSSIGERROR_MASK	0xFF

struct dpu_signature {
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
};

static inline u32 dpu_sig_read(struct dpu_signature *sig, unsigned int offset)
{
	return readl(sig->base + offset);
}

static inline void dpu_sig_write(struct dpu_signature *sig,
				 unsigned int offset, u32 value)
{
	writel(value, sig->base + offset);
}

void signature_shden(struct dpu_signature *sig, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_sig_write(sig, STATICCONTROL, val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_shden);

void signature_shdldsel_local(struct dpu_signature *sig)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, STATICCONTROL);
	val &= ~GLOBAL;
	dpu_sig_write(sig, STATICCONTROL, val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_shdldsel_local);

void signature_shdldsel_global(struct dpu_signature *sig)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, STATICCONTROL);
	dpu_sig_write(sig, STATICCONTROL, val | GLOBAL);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_shdldsel_global);

void
signature_global_panic(struct dpu_signature *sig, unsigned int win, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, EVALCONTRL(win));
	if (enable)
		val |= ENGLOBALPANIC;
	else
		val &= ~ENGLOBALPANIC;
	dpu_sig_write(sig, EVALCONTRL(win), val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_global_panic);

void
signature_local_panic(struct dpu_signature *sig, unsigned int win, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, EVALCONTRL(win));
	if (enable)
		val |= ENLOCALPANIC;
	else
		val &= ~ENLOCALPANIC;
	dpu_sig_write(sig, EVALCONTRL(win), val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_local_panic);

void
signature_alpha_mask(struct dpu_signature *sig, unsigned int win, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, EVALCONTRL(win));
	if (enable)
		val |= ALPHAMASK;
	else
		val &= ~ALPHAMASK;
	dpu_sig_write(sig, EVALCONTRL(win), val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_alpha_mask);

void signature_crc(struct dpu_signature *sig, unsigned int win, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, EVALCONTRL(win));
	if (enable)
		val |= ENCRC;
	else
		val &= ~ENCRC;
	dpu_sig_write(sig, EVALCONTRL(win), val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_crc);

void
signature_eval_win(struct dpu_signature *sig, unsigned int win, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, EVALCONTRL(win));
	if (enable)
		val |= ENEVALWIN;
	else
		val &= ~ENEVALWIN;
	dpu_sig_write(sig, EVALCONTRL(win), val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_eval_win);

void signature_win(struct dpu_signature *sig, unsigned int win,
		   int xul, int yul, int xlr, int ylr)
{
	mutex_lock(&sig->mutex);
	dpu_sig_write(sig, EVALUPPERLEFT(win),  XEVAL(xul)   | YEVAL(yul));
	dpu_sig_write(sig, EVALLOWERRIGHT(win), XEVAL(--xlr) | YEVAL(--ylr));
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_win);

void signature_crc_value(struct dpu_signature *sig, unsigned int win,
			 u32 *red, u32 *green, u32 *blue)
{
	mutex_lock(&sig->mutex);
	*red   = dpu_sig_read(sig, SIGCRCRED(win));
	*green = dpu_sig_read(sig, SIGCRCGREEN(win));
	*blue  = dpu_sig_read(sig, SIGCRCBLUE(win));
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_crc_value);

void signature_shdldreq(struct dpu_signature *sig, u8 win_mask)
{
	mutex_lock(&sig->mutex);
	dpu_sig_write(sig, SHADOWLOAD, win_mask);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_shdldreq);

void signature_continuous_mode(struct dpu_signature *sig, bool enable)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, CONTINUOUSMODE);
	if (enable)
		val |= ENCONT;
	else
		val &= ~ENCONT;
	dpu_sig_write(sig, CONTINUOUSMODE, val);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_continuous_mode);

void signature_kick(struct dpu_signature *sig)
{
	mutex_lock(&sig->mutex);
	dpu_sig_write(sig, SOFTWAREKICK, KICK);
	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(signature_kick);

bool signature_is_idle(struct dpu_signature *sig)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, STATUS);
	mutex_unlock(&sig->mutex);

	return !!(val & STSSIGIDLE);
}
EXPORT_SYMBOL_GPL(signature_is_idle);

void signature_wait_for_idle(struct dpu_signature *sig)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(100);
	bool idle;

	do {
		idle = signature_is_idle(sig);
	} while (!idle && time_before(jiffies, timeout));

	if (idle)
		dev_dbg(sig->dpu->dev, "Signature%d is idle\n", sig->id);
	else
		dev_err(sig->dpu->dev,
			"failed to wait for Signature%d idle\n", sig->id);
}
EXPORT_SYMBOL_GPL(signature_wait_for_idle);

bool signature_is_valid(struct dpu_signature *sig)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, STATUS);
	mutex_unlock(&sig->mutex);

	return !!(val & STSSIGVALID);
}
EXPORT_SYMBOL_GPL(signature_is_valid);

bool signature_is_error(struct dpu_signature *sig, u8 *err_win_mask)
{
	u32 val;

	mutex_lock(&sig->mutex);
	val = dpu_sig_read(sig, STATUS);
	mutex_unlock(&sig->mutex);

	*err_win_mask = val & STSSIGERROR_MASK;

	return !!(*err_win_mask);
}
EXPORT_SYMBOL_GPL(signature_is_error);

struct dpu_signature *dpu_sig_get(struct dpu_soc *dpu, int id)
{
	struct dpu_signature *sig;
	int i;

	for (i = 0; i < ARRAY_SIZE(sig_ids); i++)
		if (sig_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(sig_ids))
		return ERR_PTR(-EINVAL);

	sig = dpu->sig_priv[i];

	mutex_lock(&sig->mutex);

	if (sig->inuse) {
		mutex_unlock(&sig->mutex);
		return ERR_PTR(-EBUSY);
	}

	sig->inuse = true;

	mutex_unlock(&sig->mutex);

	return sig;
}
EXPORT_SYMBOL_GPL(dpu_sig_get);

void dpu_sig_put(struct dpu_signature *sig)
{
	mutex_lock(&sig->mutex);

	sig->inuse = false;

	mutex_unlock(&sig->mutex);
}
EXPORT_SYMBOL_GPL(dpu_sig_put);

struct dpu_signature *dpu_aux_sig_peek(struct dpu_signature *sig)
{
	return sig->dpu->sig_priv[sig->id ^ 1];
}
EXPORT_SYMBOL_GPL(dpu_aux_sig_peek);

void _dpu_sig_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_signature *sig;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(sig_ids); i++)
		if (sig_ids[i] == id)
			break;

	if (WARN_ON(i == ARRAY_SIZE(sig_ids)))
		return;

	sig = dpu->sig_priv[i];

	signature_shden(sig, true);
	signature_shdldsel_local(sig);
	for (j = 0; j < MAX_DPU_SIGNATURE_WIN_NUM; j++) {
		signature_global_panic(sig, j, false);
		signature_local_panic(sig, j, false);
		signature_alpha_mask(sig, j, false);
		signature_crc(sig, j, false);
		signature_eval_win(sig, j, false);
		signature_continuous_mode(sig, false);
	}
}

int dpu_sig_init(struct dpu_soc *dpu, unsigned int id,
			unsigned long unused, unsigned long base)
{
	struct dpu_signature *sig;

	sig = devm_kzalloc(dpu->dev, sizeof(*sig), GFP_KERNEL);
	if (!sig)
		return -ENOMEM;

	dpu->sig_priv[id] = sig;

	sig->base = devm_ioremap(dpu->dev, base, SZ_512);
	if (!sig->base)
		return -ENOMEM;

	sig->dpu = dpu;
	sig->id = id;
	mutex_init(&sig->mutex);

	_dpu_sig_init(dpu, id);

	return 0;
}
