// SPDX-License-Identifier: GPL-2.0
/*
 * ADI ADSP OTP (One-Time Programmable memory) driver: generic core.
 *
 * Exposes the customer OTP area of ADI ADSP processors (plus, read-only,
 * the factory serial number) as an NVMEM device whose byte offsets match
 * the SoC's OTP layout, and adds a sysfs attribute to lock the part for
 * secure boot. The core implements the user interface and the rules for
 * programming; the per-SoC data (adsp-otp-<soc>.c) describes the fields,
 * the guarded half-words and the conditions for locking, and a backend
 * (e.g. adsp-otp-rom.c) reaches the OTP. See
 * Documentation/arch/arm/adi/adsp-otp.rst for the usage and the rules,
 * and Documentation/ABI/testing/sysfs-driver-adsp-otp for the lock
 * interface.
 *
 * The driver is meant for manufacturing images: it lets root read and
 * program every customer field, keys included, and permanently lock the
 * part for secure boot.
 *
 * The NVMEM data is the OTP's 32-bit words in little-endian byte order,
 * and the core handles it as native u32 (e.g. to find the half-words of
 * a write), so the driver depends on a little-endian kernel.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitmap.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "adsp-otp.h"

/* Exact string to write to the 'lock' attribute */
#define ADSP_OTP_LOCK_CONFIRM		"permanently-lock"

/*
 * Writing the half-words of adsp_otp_soc_data.guards marked unsafe, e.g.
 * the OTP read timing override, can leave a part that no longer reads its
 * OTP and no longer boots. Only at load time (or on the kernel command
 * line), never by default.
 */
static bool allow_unsafe_fields;
module_param(allow_unsafe_fields, bool, 0444);
MODULE_PARM_DESC(allow_unsafe_fields,
		 "Allow writing OTP half-words that can make the part unbootable (default: no)");

/* The field containing byte @pos, or NULL for a reserved gap */
const struct adsp_otp_field *adsp_otp_field_at(struct adsp_otp *otp,
					       unsigned int pos)
{
	unsigned int i;

	for (i = 0; i < otp->soc->nfields; i++) {
		const struct adsp_otp_field *field = &otp->soc->fields[i];

		if (pos >= field->offset &&
		    pos < field->offset + adsp_otp_field_bytes(field))
			return field;
	}

	return NULL;
}

/* Read one whole field into otp->rbuf */
int adsp_otp_read_field(struct adsp_otp *otp,
			const struct adsp_otp_field *field)
{
	lockdep_assert_held(&otp->lock);

	memset(otp->rbuf, 0, otp->max_words * sizeof(u32));

	return otp->ops->read_field(otp, field, otp->rbuf);
}

/* Wipe buffers that may hold key material */
static void adsp_otp_wipe(struct adsp_otp *otp)
{
	memzero_explicit(otp->rbuf, otp->max_words * sizeof(u32));
	memzero_explicit(otp->stage, otp->soc->size);
	memzero_explicit(otp->delta, otp->max_words * sizeof(u32));
}

static int adsp_otp_lock_state(struct adsp_otp *otp)
{
	lockdep_assert_held(&otp->lock);

	return otp->ops->lock_state(otp);
}

/*
 * Programming (and locking) needs a device not marked read-only in the DT,
 * an open part whose lock is not programmed yet, and the backend's go-ahead
 * (e.g. no other core able to access the OTP).
 */
static int adsp_otp_check_writable(struct adsp_otp *otp)
{
	int ret;

	lockdep_assert_held(&otp->lock);

	if (otp->read_only) {
		dev_err(otp->dev, "device is read-only\n");
		return -EROFS;
	}

	ret = adsp_otp_lock_state(otp);
	if (ret < 0)
		return ret;
	if (ret == ADSP_OTP_LOCKED) {
		dev_err(otp->dev, "part is locked\n");
		return -EROFS;
	}
	if (ret == ADSP_OTP_UNLOCKED) {
		dev_err(otp->dev, "part is in the unlocked state\n");
		return -EROFS;
	}
	if (ret == ADSP_OTP_LOCK_PENDING) {
		dev_err(otp->dev, "lock programmed, effective at the next reset\n");
		return -EROFS;
	}

	return otp->ops->check_program(otp);
}

/* Start of the first field after byte @pos, or the end of the area */
static unsigned int adsp_otp_next_field(struct adsp_otp *otp,
					unsigned int pos)
{
	unsigned int i;

	for (i = 0; i < otp->soc->nfields; i++)
		if (otp->soc->fields[i].offset > pos)
			return otp->soc->fields[i].offset;

	return otp->soc->size;
}

static bool adsp_otp_range_ok(struct adsp_otp *otp, unsigned int offset,
			      size_t bytes)
{
	return bytes && offset < otp->soc->size &&
	       bytes <= otp->soc->size - offset;
}

/* ------------------------------------------------------------------------
 * NVMEM read
 */
static int adsp_otp_reg_read(void *priv, unsigned int offset, void *val,
			     size_t bytes)
{
	struct adsp_otp *otp = priv;
	unsigned int pos = offset, end = offset + bytes;
	u8 *out = val;
	int ret = 0;	/* a read of reserved gaps only touches no field */

	if (!adsp_otp_range_ok(otp, offset, bytes))
		return -EINVAL;

	mutex_lock(&otp->lock);

	while (pos < end) {
		const struct adsp_otp_field *field;
		unsigned int stop;

		field = adsp_otp_field_at(otp, pos);
		if (!field) {
			/* Reserved gap: not part of any field */
			stop = min(end, adsp_otp_next_field(otp, pos));
			memset(out + (pos - offset), 0, stop - pos);
			pos = stop;
			continue;
		}

		ret = adsp_otp_read_field(otp, field);
		if (ret)
			goto out;

		stop = min(end, field->offset + adsp_otp_field_bytes(field));
		memcpy(out + (pos - offset),
		       (u8 *)otp->rbuf + (pos - field->offset), stop - pos);
		pos = stop;
	}

out:
	adsp_otp_wipe(otp);
	mutex_unlock(&otp->lock);

	if (ret)
		memzero_explicit(val, bytes);	/* no partial data on error */
	return ret;
}

/* ------------------------------------------------------------------------
 * NVMEM write
 */

/*
 * Name the half-words that did not read back as written (e.g. ROM anomaly
 * 20000129). Never log OTP values: fields may hold key material.
 */
static void adsp_otp_report_mismatch(struct adsp_otp *otp,
				     const struct adsp_otp_field *field,
				     const u32 *want, const u32 *got)
{
	unsigned int word, half;

	for (word = 0; word < field->words; word++) {
		for (half = 0; half < 2; half++) {
			if ((u16)(want[word] >> (16 * half)) ==
			    (u16)(got[word] >> (16 * half)))
				continue;
			dev_err(otp->dev,
				"%s word %u half %u does not read back as programmed\n",
				field->name, word, half);
		}
	}
}

/*
 * Each 16-bit half-word has its own ECC and can be programmed once. Per
 * half-word of @want: 0 or equal to the current value is skipped (the
 * hardware ignores 0), otherwise the current value must be blank (0), or
 * the result is -EEXIST with *@bad set to the half-word index. Returns the
 * number of half-words to program and, if @delta is given, fills it with
 * just those.
 */
static int adsp_otp_delta(const u32 *cur, const u32 *want, u32 *delta,
			  unsigned int words, unsigned int *bad)
{
	unsigned int word, half;
	int halves = 0;

	for (word = 0; word < words; word++) {
		if (delta)
			delta[word] = 0;
		for (half = 0; half < 2; half++) {
			u16 cur_half = cur[word] >> (16 * half);
			u16 want_half = want[word] >> (16 * half);

			if (!want_half || cur_half == want_half)
				continue;
			if (cur_half) {
				*bad = 2 * word + half;
				return -EEXIST;
			}
			if (delta)
				delta[word] |= (u32)want_half << (16 * half);
			halves++;
		}
	}

	return halves;
}

/* Program the non-zero half-words of one field in otp->stage, verify */
static int adsp_otp_program_field(struct adsp_otp *otp,
				  const struct adsp_otp_field *field)
{
	u32 *want = otp->stage + field->offset / 4;
	unsigned int bad, i;
	int ret, halves;

	/* Hand the backend only the half-words that still need programming */
	ret = adsp_otp_read_field(otp, field);
	if (ret)
		return ret;
	halves = adsp_otp_delta(otp->rbuf, want, otp->delta, field->words,
				&bad);
	if (halves <= 0)
		return halves;

	/* Expected content afterwards: the current one plus the new halves */
	for (i = 0; i < field->words; i++)
		want[i] = otp->rbuf[i] | otp->delta[i];

	dev_info(otp->dev, "programming %s (%d half-words)\n", field->name,
		 halves);

	ret = otp->ops->program_field(otp, field, otp->delta);
	if (ret)
		return ret;

	/* A backend's success report is not enough (e.g. anomaly 20000129) */
	ret = adsp_otp_read_field(otp, field);
	if (ret)
		return ret;
	if (memcmp(otp->rbuf, want, adsp_otp_field_bytes(field))) {
		adsp_otp_report_mismatch(otp, field, want, otp->rbuf);
		return -EIO;
	}

	return 0;
}

/* The guard containing byte @pos, or NULL */
static const struct adsp_otp_guard *adsp_otp_guard_at(struct adsp_otp *otp,
						      unsigned int pos)
{
	unsigned int i;

	for (i = 0; i < otp->soc->nguards; i++) {
		const struct adsp_otp_guard *guard = &otp->soc->guards[i];

		if (pos >= guard->offset && pos < guard->offset + guard->bytes)
			return guard;
	}

	return NULL;
}

/*
 * Pass 1: every non-zero half-word of the request must lie in a writable
 * field, outside the guarded half-words (unless allowed). Marks those
 * fields in @todo.
 */
static int adsp_otp_check_request(struct adsp_otp *otp, unsigned int offset,
				  unsigned int end, unsigned long *todo)
{
	const struct adsp_otp_soc_data *soc = otp->soc;
	const struct adsp_otp_field *field;
	const struct adsp_otp_guard *guard;
	unsigned int pos, i;

	for (pos = offset; pos < end; pos += 2) {
		if (!(u16)(otp->stage[pos / 4] >> (8 * (pos % 4))))
			continue;

		field = adsp_otp_field_at(otp, pos);
		if (!field || !field->writable) {
			dev_err(otp->dev, "0x%03x (%s) is not writable\n", pos,
				field ? field->name : "reserved");
			return -EACCES;
		}

		guard = adsp_otp_guard_at(otp, pos);
		if (guard && (!guard->unsafe || !allow_unsafe_fields)) {
			dev_err(otp->dev, "0x%03x (%s) is protected%s\n", pos,
				guard->what, guard->unsafe ?
				", see the allow_unsafe_fields parameter" : "");
			return -EACCES;
		}
		if (guard)
			dev_warn(otp->dev, "writing 0x%03x (%s)\n", pos,
				 guard->what);

		__set_bit(field - soc->fields, todo);
	}

	for_each_set_bit(i, todo, soc->nfields) {
		field = &soc->fields[i];
		if (field->val_mask &&
		    otp->stage[field->offset / 4] & ~field->val_mask) {
			dev_err(otp->dev, "%s: value out of range\n",
				field->name);
			return -EINVAL;
		}
	}

	return 0;
}

static int adsp_otp_reg_write(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct adsp_otp *otp = priv;
	const struct adsp_otp_soc_data *soc = otp->soc;
	unsigned int end = offset + bytes;
	unsigned long *todo;
	unsigned int i;
	int ret;

	/* Whole 16-bit half-words only (see config.word_size in probe) */
	if (!adsp_otp_range_ok(otp, offset, bytes) ||
	    !IS_ALIGNED(offset, 2) || !IS_ALIGNED(bytes, 2))
		return -EINVAL;

	todo = bitmap_zalloc(soc->nfields, GFP_KERNEL);
	if (!todo)
		return -ENOMEM;

	mutex_lock(&otp->lock);

	ret = adsp_otp_check_writable(otp);
	if (ret)
		goto out;

	/* Private copy: checks and backend see the same bytes */
	memcpy((u8 *)otp->stage + offset, val, bytes);

	ret = adsp_otp_check_request(otp, offset, end, todo);
	if (ret)
		goto out;

	/*
	 * Pass 2: check every field against its current content before
	 * programming any, so an invalid request burns nothing.
	 */
	for_each_set_bit(i, todo, soc->nfields) {
		const struct adsp_otp_field *field = &soc->fields[i];
		const u32 *want = otp->stage + field->offset / 4;
		unsigned int bad;
		int halves;

		ret = adsp_otp_read_field(otp, field);
		if (ret)
			goto out;

		halves = adsp_otp_delta(otp->rbuf, want, NULL, field->words,
					&bad);
		if (halves < 0) {
			/* No values: the field may hold key material */
			dev_err(otp->dev,
				"%s word %u half %u is already programmed with a different value\n",
				field->name, bad / 2, bad % 2);
			ret = -EEXIST;
			goto out;
		}
		if (!halves)
			__clear_bit(i, todo);	/* already holds the data */
	}

	/* Pass 3: program, one field at a time, each verified */
	for_each_set_bit(i, todo, soc->nfields) {
		ret = adsp_otp_program_field(otp, &soc->fields[i]);
		if (ret) {
			dev_err(otp->dev, "stopped at %s; earlier fields stay programmed\n",
				soc->fields[i].name);
			goto out;
		}
	}

out:
	adsp_otp_wipe(otp);
	mutex_unlock(&otp->lock);
	bitmap_free(todo);
	return ret;
}

/* ------------------------------------------------------------------------
 * Locking: sysfs attributes 'lock' and 'lock_state'
 */
static ssize_t lock_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	static const char * const names[] = {
		[ADSP_OTP_OPEN] = "open",
		[ADSP_OTP_LOCK_PENDING] = "lock-pending",
		[ADSP_OTP_LOCKED] = "locked",
		[ADSP_OTP_UNLOCKED] = "unlocked",
	};
	struct adsp_otp *otp = dev_get_drvdata(dev);
	int state;

	mutex_lock(&otp->lock);
	state = adsp_otp_lock_state(otp);
	adsp_otp_wipe(otp);
	mutex_unlock(&otp->lock);
	if (state < 0)
		return state;
	if (WARN_ON(state >= ARRAY_SIZE(names)))
		return -EIO;

	return sysfs_emit(buf, "%s\n", names[state]);
}
static DEVICE_ATTR_RO(lock_state);

static ssize_t lock_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct adsp_otp *otp = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, ADSP_OTP_LOCK_CONFIRM))
		return -EINVAL;

	mutex_lock(&otp->lock);

	ret = adsp_otp_check_writable(otp);
	if (ret)
		goto out;

	/* The SoC's conditions, e.g. a key for secure boot to use */
	if (otp->soc->lock_precheck) {
		ret = otp->soc->lock_precheck(otp);
		if (ret)
			goto out;
	}

	dev_warn(dev, "permanently locking the part\n");

	ret = otp->ops->lock(otp);
	if (!ret) {
		/* Like every programming operation, verify by reading back */
		ret = adsp_otp_lock_state(otp);
		if (ret >= 0 && ret != ADSP_OTP_LOCK_PENDING) {
			dev_err(dev, "lock does not read back as programmed\n");
			ret = -EIO;
		}
	}
	if (ret < 0) {
		/* The backend may have failed before or after programming */
		dev_err(dev, "locking failed; the lock may be programmed, read lock_state\n");
		goto out;
	}
	ret = 0;
	dev_warn(dev, "OTP lock programmed; takes effect at the next reset\n");

out:
	adsp_otp_wipe(otp);
	mutex_unlock(&otp->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(lock);

static struct attribute *adsp_otp_attrs[] = {
	&dev_attr_lock_state.attr,
	&dev_attr_lock.attr,
	NULL
};

static const struct attribute_group adsp_otp_group = {
	.attrs = adsp_otp_attrs,
};

/* For the backends' platform_driver.driver.dev_groups */
const struct attribute_group *adsp_otp_groups[] = {
	&adsp_otp_group,
	NULL
};

/* ------------------------------------------------------------------------
 * Probe
 */

/*
 * Check the SoC's tables: fields sorted, not overlapping, 32-bit aligned
 * and inside the OTP area, value fields one word long, and guards whole
 * half-words inside the OTP area. Catches mistakes in the SoC tables.
 */
static int adsp_otp_validate_fields(struct adsp_otp *otp)
{
	const struct adsp_otp_soc_data *soc = otp->soc;
	unsigned int i, end = 0;

	for (i = 0; i < soc->nfields; i++) {
		const struct adsp_otp_field *field = &soc->fields[i];

		if (field->offset < end || !IS_ALIGNED(field->offset, 4) ||
		    !field->words ||
		    field->offset + adsp_otp_field_bytes(field) > soc->size ||
		    (field->val_mask && field->words != 1)) {
			dev_err(otp->dev, "invalid field table entry %s\n",
				field->name);
			return -EINVAL;
		}
		end = field->offset + adsp_otp_field_bytes(field);
	}

	for (i = 0; i < soc->nguards; i++) {
		const struct adsp_otp_guard *guard = &soc->guards[i];

		if (!IS_ALIGNED(guard->offset, 2) ||
		    !IS_ALIGNED(guard->bytes, 2) || !guard->bytes ||
		    guard->offset + guard->bytes > soc->size) {
			dev_err(otp->dev, "invalid guard table entry %s\n",
				guard->what);
			return -EINVAL;
		}
	}

	return 0;
}

/* Size of the largest field, in 32-bit words: sizes the buffers */
static unsigned int adsp_otp_max_words(const struct adsp_otp_soc_data *soc)
{
	unsigned int i, max_words = 0;

	for (i = 0; i < soc->nfields; i++)
		max_words = max_t(unsigned int, max_words,
				  soc->fields[i].words);

	return max_words;
}

/**
 * adsp_otp_probe() - probe an OTP device, for a backend's platform driver
 * @pdev: the device; its match data is the SoC's struct adsp_otp_soc_data
 * @ops: the backend's operations
 *
 * Each device has its own state and lock. Devices backed by the same
 * hardware must be prevented by the backend (e.g. by claiming the
 * hardware's registers).
 *
 * Return: 0 or -errno.
 */
int adsp_otp_probe(struct platform_device *pdev,
		   const struct adsp_otp_ops *ops)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = { };
	struct nvmem_cell_info *cells;
	struct nvmem_device *nvmem;
	struct adsp_otp *otp;
	unsigned int i;
	int ret;

	otp = devm_kzalloc(dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;
	otp->dev = dev;
	otp->ops = ops;
	otp->read_only = device_property_present(dev, "read-only");
	otp->soc = device_get_match_data(dev);
	if (!otp->soc)
		return -ENODEV;
	platform_set_drvdata(pdev, otp);

	ret = devm_mutex_init(dev, &otp->lock);
	if (ret)
		return ret;

	ret = adsp_otp_validate_fields(otp);
	if (ret)
		return ret;
	otp->max_words = adsp_otp_max_words(otp->soc);

	ret = otp->ops->probe(otp, pdev);
	if (ret)
		return ret;

	/* Handed to the backend: kmalloc()ed, never on the stack */
	otp->rbuf = devm_kcalloc(dev, otp->max_words, sizeof(u32), GFP_KERNEL);
	otp->delta = devm_kcalloc(dev, otp->max_words, sizeof(u32), GFP_KERNEL);
	otp->stage = devm_kzalloc(dev, otp->soc->size, GFP_KERNEL);
	if (!otp->rbuf || !otp->delta || !otp->stage)
		return -ENOMEM;

	/* One cell per field, named after it */
	cells = devm_kcalloc(dev, otp->soc->nfields, sizeof(*cells),
			     GFP_KERNEL);
	if (!cells)
		return -ENOMEM;
	for (i = 0; i < otp->soc->nfields; i++) {
		cells[i].name = otp->soc->fields[i].name;
		cells[i].offset = otp->soc->fields[i].offset;
		cells[i].bytes = adsp_otp_field_bytes(&otp->soc->fields[i]);
	}

	config.dev = dev;
	/* adsp-otp0, adsp-otp1, ...: one NVMEM device per OTP device */
	config.name = "adsp-otp";
	config.id = NVMEM_DEVID_AUTO;
	config.owner = THIS_MODULE;
	config.type = NVMEM_TYPE_OTP;
	/* The NVMEM core also honours the DT "read-only" property */
	config.root_only = true;	/* holds keys */
	/*
	 * Writes must be whole 16-bit half-words, the programming unit;
	 * adsp_otp_reg_write() rejects odd offsets and lengths itself. Keep
	 * word_size and stride at 1: the core would otherwise round write
	 * lengths down to word_size without telling reg_write(), and reject
	 * reads at offsets that are not a multiple of the stride. Reads may
	 * be of any offset and length.
	 */
	config.word_size = 1;
	config.stride = 1;
	config.size = otp->soc->size;
	config.cells = cells;
	config.ncells = otp->soc->nfields;
	config.reg_read = adsp_otp_reg_read;
	config.reg_write = adsp_otp_reg_write;
	config.priv = otp;

	nvmem = devm_nvmem_register(dev, &config);
	return PTR_ERR_OR_ZERO(nvmem);
}

MODULE_AUTHOR("Sam Winchenbach <sam.winchenbach@analog.com>");
MODULE_DESCRIPTION("ADI ADSP OTP driver");
MODULE_LICENSE("GPL");
