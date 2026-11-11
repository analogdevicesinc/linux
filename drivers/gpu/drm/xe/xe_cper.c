// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <linux/bcd.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/time.h>
#include <linux/timekeeping.h>

#include "xe_cper.h"
#include "xe_cper_types.h"
#include "xe_device.h"
#include "xe_printk.h"
#include "xe_ras.h"
#include "xe_ras_types.h"
#include "xe_trace_cper.h"

static bool is_sigid(u32 sigid)
{
	return sigid != INTEL_SIGID_INVALID && sigid != U32_MAX;
}

static bool is_hw_sigid(enum xe_sigid sigid)
{
	return in_range(sigid, INTEL_SIGID_GPU_XE_HARDWARE_START, INTEL_SIGID_BATCH);
}

/* CPER platform_id GUIDs (RFC 9562 UUIDv5) for each PCI device ID. */
static const struct xe_device_guid_entry xe_device_guids[] = {
	{ 0x674C, GUID_INIT(0x9046afe5, 0x9041, 0x5124,
			    0x86, 0x14, 0x92, 0x55, 0x0d, 0x9e, 0x9d, 0xa6) },
	{ 0x674D, GUID_INIT(0xf95911b8, 0x6564, 0x5905,
			    0xa6, 0xa7, 0xb9, 0xbf, 0x54, 0xab, 0x6c, 0x27) },
	{ 0x674E, GUID_INIT(0xb8d24106, 0x0108, 0x59f1,
			    0x8f, 0xfc, 0x9f, 0xc4, 0xa9, 0xc9, 0xb7, 0xb0) },
	{ 0x674F, GUID_INIT(0xdd8dd88e, 0xef2f, 0x59b1,
			    0x99, 0x15, 0xdb, 0x6f, 0x1d, 0xc9, 0xb4, 0x5d) },
	{ 0x6750, GUID_INIT(0x1dd66c44, 0x87d5, 0x524a,
			    0xad, 0x6e, 0xfc, 0x18, 0x9d, 0x42, 0x6f, 0x1a) },
	{ 0x6751, GUID_INIT(0x17d0dd04, 0x2fad, 0x5400,
			    0x99, 0xc8, 0x9b, 0x45, 0x51, 0x3b, 0x67, 0xe3) },
	{ 0x6752, GUID_INIT(0xb778ead8, 0x0009, 0x59a7,
			    0xa1, 0x3a, 0x78, 0xce, 0x4d, 0xdd, 0xae, 0xf5) },
};

static const guid_t *lookup_device_guid(struct xe_device *xe)
{
	struct pci_dev *pdev = to_pci_dev(xe->drm.dev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(xe_device_guids); i++)
		if (xe_device_guids[i].device_id == pdev->device)
			return &xe_device_guids[i].guid;

	xe_printk(xe, warn_once, "CPER: no platform GUID for device %04x\n", pdev->device);

	return NULL;
}

static u64 cper_timestamp_now(void)
{
	struct tm tm;
	u64 ts = 0;
	u8 *p = (u8 *)&ts;
	int year;

	time64_to_tm(ktime_get_real_seconds(), 0, &tm);

	year = tm.tm_year + 1900;

	p[0] = bin2bcd(tm.tm_sec);
	p[1] = bin2bcd(tm.tm_min);
	p[2] = bin2bcd(tm.tm_hour);
	p[3] = 0x0;
	p[4] = bin2bcd(tm.tm_mday);
	p[5] = bin2bcd(tm.tm_mon + 1);
	p[6] = bin2bcd(year % 100);
	p[7] = bin2bcd(year / 100);

	return ts;
}

static void cper_init_intel_hdr(struct xe_device *xe, const struct xe_ras_error_class *counter,
				u64 first_timestamp, u32 sig_id, u32 error_count,
				struct xe_cper_intel_hdr *ihdr)
{
	ihdr->counter = *counter;
	ihdr->validation_bits |= XE_CPER_VALID_LOCATION;
	ihdr->sig_id = U32_MAX;

	if (first_timestamp) {
		ihdr->first_timestamp = first_timestamp;
		ihdr->validation_bits |= XE_CPER_VALID_FIRST_TIMESTAMP;
	}

	if (is_sigid(sig_id)) {
		ihdr->sig_id = sig_id;
		ihdr->validation_bits |= XE_CPER_VALID_SIG_ID;
	}

	ihdr->error_count = error_count;

	strscpy(ihdr->pci_bdf, pci_name(to_pci_dev(xe->drm.dev)), sizeof(ihdr->pci_bdf));
	ihdr->validation_bits |= XE_CPER_VALID_PCI_BDF;

#ifdef MODULE
	if (THIS_MODULE->srcversion) {
		strscpy(ihdr->drv_version, THIS_MODULE->srcversion, sizeof(ihdr->drv_version));
		ihdr->validation_bits |= XE_CPER_VALID_DRV_VERSION;
	}
#endif
}

static void cper_init_record_hdr(struct xe_device *xe, u8 severity,
				 const guid_t *notification_type,
				 struct cper_record_header *rhdr,
				 u32 total_len)
{
	const guid_t *device_guid;

	/* Assemble the CPER record header (UEFI Appendix N.2.1) */
	memcpy(rhdr->signature, CPER_SIG_RECORD, CPER_SIG_SIZE);
	rhdr->revision          = CPER_RECORD_REV;
	rhdr->signature_end     = CPER_SIG_END;
	rhdr->section_count     = 1;
	rhdr->error_severity    = severity;
	rhdr->validation_bits   = CPER_VALID_TIMESTAMP;
	rhdr->record_length     = total_len;
	rhdr->timestamp         = cper_timestamp_now();
	rhdr->creator_id        = INTEL_CPER_CREATOR_XEKMD;
	rhdr->notification_type = *notification_type;
	rhdr->record_id         = cper_next_record_id();

	device_guid = lookup_device_guid(xe);
	if (device_guid) {
		rhdr->platform_id      = *device_guid;
		rhdr->validation_bits |= CPER_VALID_PLATFORM_ID;
	}
}

static void cper_init_section_desc(struct xe_device *xe, u8 severity,
				   struct cper_section_descriptor *sdesc,
				   u32 einfo_len)
{
	sdesc->section_offset   = sizeof(struct cper_record_header) +
				  sizeof(struct cper_section_descriptor);
	sdesc->section_length   = sizeof(struct xe_cper_intel_hdr) + einfo_len;
	sdesc->revision         = CPER_SEC_REV;
	sdesc->validation_bits  = 0;
	sdesc->section_type     = INTEL_CPER_SECTION_ACCEL_GENERIC;
	sdesc->section_severity = severity;

	if (xe->info.has_device_uid) {
		memcpy(&sdesc->fru_id, &xe->device_uid, sizeof(xe->device_uid));
		sdesc->validation_bits |= CPER_SEC_VALID_FRU_ID;
	}
}

static void cper_record_emit(struct xe_device *xe, u8 severity,
			     const guid_t *notification_type,
			     const struct xe_cper_intel_hdr *ihdr,
			     const void *einfo, u32 einfo_len)
{
	struct cper_section_descriptor *sdesc;
	struct xe_cper_nonstd_record *rec;
	struct cper_record_header *rhdr;
	u32 total_len;

	total_len = sizeof(struct xe_cper_nonstd_record) + einfo_len;
	rec = kzalloc(total_len, GFP_KERNEL);
	if (!rec)
		return;

	rhdr = &rec->record_hdr;
	sdesc = &rec->section_desc;

	/* Assemble the CPER record header (UEFI Appendix N.2.1) */
	cper_init_record_hdr(xe, severity, notification_type, rhdr, total_len);

	/* Assemble the section descriptor (UEFI Appendix N.2.2) */
	cper_init_section_desc(xe, severity, sdesc, einfo_len);

	/* Copy the Intel-specific section header */
	rec->intel_hdr = *ihdr;

	/* Append optional variable-length error info */
	if (einfo && einfo_len)
		memcpy((u8 *)rec + sizeof(*rec), einfo, einfo_len);

	trace_xe_error_cper(xe, &rhdr->platform_id, &sdesc->fru_id, severity,
			    &rec->intel_hdr, total_len, (u8 *)rec);

	kfree(rec);
}

/**
 * xe_cper_emit_counter() - Emit CPER records for an already fetched error counter
 * @xe: The &xe_device
 * @cper_sev: CPER severity
 * @sigid: Error signature identifier
 * @counter: RAS error class
 * @response: Counter response returned by firmware for @counter
 *
 * Build and emit a CPER record from @response.
 */
void xe_cper_emit_counter(struct xe_device *xe, int cper_sev, enum xe_sigid sigid,
			  const struct xe_ras_error_class *counter,
			  const struct xe_ras_get_counter_response *response)
{
	struct xe_cper_intel_hdr ihdr = {};

	xe_assert(xe, is_hw_sigid(sigid));

	cper_init_intel_hdr(xe, counter, response->timestamp,
			    sigid, response->value, &ihdr);
	cper_record_emit(xe, cper_sev, &INTEL_CPER_NOTIFY_GPU_ERROR, &ihdr, NULL, 0);
}

static void cper_emit_hw_err(struct xe_device *xe, int cper_sev, enum xe_sigid sigid,
			     const struct xe_ras_error_class *counter)
{
	struct xe_ras_get_counter_response response = {};

	/* Fetching the counter sends a sysctrl command */
	might_sleep();
	xe_device_assert_mem_access(xe);

	if (xe_ras_get_counter_response(xe, counter, &response)) {
		xe_err(xe, "CPER: failed to get counter, skipping record\n");
		return;
	}

	xe_cper_emit_counter(xe, cper_sev, sigid, counter, &response);
}

/**
 * xe_cper_emit() - Emit a CPER record for a SIGID log entry
 * @xe: The &xe_device
 * @cper_sev: CPER severity
 * @sigid: Error signature identifier
 * @component: Component identifier
 * @location: Location details of the @component
 * @data: Pointer to the additional details, or ERR_PTR, or NULL if not applicable
 * @len: Length of the @data in bytes, or 0 if not applicable
 * @vaf: Formatted log message
 *
 * Emit a CPER record for a SIGID log entry
 *
 * Example trace output::
 *
 *   xe_error_cper: dev=0000:03:00.0 severity=2
 *     platform_id=9046afe5-9041-5124-8614-92550d9e9da6
 *     fru_id=91befe38-2f61-3d7b-0000-000000000000
 *     intel_hdr_raw=
 *       01 02 00 18 00 00 00 01 00 00 00 00 64 77 51 14 ...
 *     cper_len=1772
 *     cper_raw=
 *       43 50 45 52 00 01 ff ff ff ff 01 00 02 00 00 00 ...
 */
void xe_cper_emit(struct xe_device *xe, int cper_sev, enum xe_sigid sigid,
		  u32 component, u32 location, const void *data, size_t len,
		  struct va_format *vaf)
{
	if (is_hw_sigid(sigid) && !IS_ERR_OR_NULL(data)) {
		cper_emit_hw_err(xe, cper_sev, sigid, data);
		return;
	}

	/* TODO software CPER */
}

static ssize_t cper_on_query_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct xe_device *xe = kdev_to_xe_device(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(xe->cper.on_query));
}

static ssize_t cper_on_query_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct xe_device *xe = kdev_to_xe_device(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	WRITE_ONCE(xe->cper.on_query, enable);

	return count;
}
static DEVICE_ATTR_RW(cper_on_query);

static struct attribute *cper_attrs[] = {
	&dev_attr_cper_on_query.attr,
	NULL
};

/* See Documentation/ABI/testing/sysfs-driver-intel-xe-cper */
static const struct attribute_group cper_group = {
	.attrs = cper_attrs,
};

/**
 * xe_cper_sysfs_init() - Create the CPER sysfs attributes
 * @xe: The &xe_device
 *
 * Return: 0 on success, negative error code on failure.
 */
int xe_cper_sysfs_init(struct xe_device *xe)
{
	return devm_device_add_group(xe->drm.dev, &cper_group);
}
