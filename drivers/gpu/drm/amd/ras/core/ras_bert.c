// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ras.h"

#if defined(CONFIG_X86_MCE_AMD) && defined(CONFIG_ACPI_APEI)
struct ras_bert_hest_generic_status {
	u32 block_status;
	u32 raw_data_offset;
	u32 raw_data_length;
	u32 data_length;
	u32 error_severity;
};

struct ras_bert_hest_generic_data {
	u8 section_type[16];
	u32 error_severity;
	u16 revision;
	u8 validation_bits;
	u8 flags;
	u32 error_data_length;
	u8 fru_id[16];
	u8 fru_text[20];
};

struct ras_bert_hest_generic_data_v300 {
	u8 section_type[16];
	u32 error_severity;
	u16 revision;
	u8 validation_bits;
	u8 flags;
	u32 error_data_length;
	u8 fru_id[16];
	u8 fru_text[20];
	u64 time_stamp;
};

static inline int ras_bert_get_version(struct ras_bert_hest_generic_data *gdata)
{
	return gdata->revision >> 8;
}

static inline void *ras_bert_get_payload(struct ras_bert_hest_generic_data *gdata)
{
	if (ras_bert_get_version(gdata) >= 3)
		return (void *)(((struct ras_bert_hest_generic_data_v300 *)(gdata)) + 1);

	return gdata + 1;
}

static inline int ras_bert_get_size(struct ras_bert_hest_generic_data *gdata)
{
	if (ras_bert_get_version(gdata) >= 3)
		return sizeof(struct ras_bert_hest_generic_data_v300);

	return sizeof(struct ras_bert_hest_generic_data);
}

static inline void *ras_bert_get_next(struct ras_bert_hest_generic_data *gdata)
{
	return (void *)gdata + ras_bert_get_size(gdata) + gdata->error_data_length;
}

static int ras_bert_estatus_check(const struct ras_bert_hest_generic_status *estatus)
{
	struct ras_bert_hest_generic_data *gdata;
	unsigned int data_len, record_size;

	if (estatus->data_length &&
	    estatus->data_length < sizeof(struct ras_bert_hest_generic_data))
		return -EINVAL;
	if (estatus->raw_data_length &&
	    estatus->raw_data_offset < sizeof(*estatus) + estatus->data_length)
		return -EINVAL;

	data_len = estatus->data_length;
	gdata = (struct ras_bert_hest_generic_data *)(estatus + 1);
	while ((void *)gdata - (void *)(estatus + 1) < estatus->data_length) {
		if (ras_bert_get_size(gdata) > data_len)
			return -EINVAL;

		record_size = ras_bert_get_size(gdata) + gdata->error_data_length;
		if (record_size > data_len)
			return -EINVAL;

		data_len -= record_size;
		gdata = ras_bert_get_next(gdata);
	}
	if (data_len)
		return -EINVAL;

	return 0;
}

typedef int (*ras_bert_section_parser)(struct ras_core_context *ras_core,
				       struct ras_bert_hest_generic_data *gdata);

struct ras_bert_section_dispatch {
	const u8 *section_type;
	ras_bert_section_parser parser;
};

static int ras_bert_record_section(struct ras_core_context *ras_core,
				   struct ras_bert_hest_generic_data *gdata)
{
	struct ras_boot_err_ctx ctx = { 0 };
	u16 data_size;

	memcpy(ctx.section_type, gdata->section_type,
	       min(sizeof(ctx.section_type), sizeof(gdata->section_type)));
	ctx.error_severity = gdata->error_severity;
	ctx.reg_ctx_type = CPER_CTX_TYPE__BOOT;
	data_size = min_t(u32, gdata->error_data_length, sizeof(ctx.regs));
	ctx.reg_arr_size = data_size;
	memcpy(ctx.regs, ras_bert_get_payload(gdata), data_size);

	ras_log_ring_add_log_event(ras_core, RAS_LOG_EVENT_BOOT, &ctx, sizeof(ctx), NULL);

	return 0;
}

static const struct ras_cper_guid ras_bert_guid_crashdump = GPU__CRASHDUMP;

static const struct ras_bert_section_dispatch ras_bert_section_dispatch[] = {
	{ ras_bert_guid_crashdump.b, ras_bert_record_section },
};

static const struct ras_bert_section_dispatch *
ras_bert_find_section_parser(const u8 *section_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ras_bert_section_dispatch); i++) {
		if (!memcmp(section_type, ras_bert_section_dispatch[i].section_type,
			    CPER_UUID_MAX_SIZE))
			return &ras_bert_section_dispatch[i];
	}

	return NULL;
}

int ras_bert_process_records(struct ras_core_context *ras_core,
			     const void *bert, u32 bert_len)
{
	struct ras_bert_hest_generic_status *estatus;
	struct ras_bert_hest_generic_data *gdata;
	u32 estatus_len;
	int remain;
	int ret;

	if (!ras_core || !bert || bert_len < sizeof(struct ras_bert_hest_generic_status))
		return -EINVAL;

	estatus = (struct ras_bert_hest_generic_status *)bert;
	remain = bert_len;

	while (remain >= sizeof(struct ras_bert_hest_generic_status)) {
		estatus_len = estatus->raw_data_length ?
			      estatus->raw_data_offset + estatus->raw_data_length :
			      sizeof(*estatus) + estatus->data_length;
		if (remain < estatus_len) {
			RAS_DEV_ERR(ras_core->dev, "truncated status block (length: %u).\n",
				    estatus_len);
			return -EINVAL;
		}
		if (!estatus_len)
			return -EINVAL;

		/*
		 * The generic APEI BERT path may have already consumed the boot
		 * error region and cleared block_status. AMDGPU still needs to
		 * parse the cached BERT payload, so do not use block_status to
		 * decide whether this status block contains records.
		 */
		if (ras_bert_estatus_check(estatus)) {
			RAS_DEV_ERR(ras_core->dev, "invalid error record.\n");
			return -EINVAL;
		}

		gdata = (struct ras_bert_hest_generic_data *)(estatus + 1);
		while ((void *)gdata - (void *)(estatus + 1) < estatus->data_length) {
			const struct ras_bert_section_dispatch *parser;

			parser = ras_bert_find_section_parser(gdata->section_type);
			if (parser) {
				ret = parser->parser(ras_core, gdata);
				if (ret)
					return ret;
			} else {
				RAS_DEV_INFO(ras_core->dev, "skip section: %pUl\n",
					     gdata->section_type);
			}

			gdata = ras_bert_get_next(gdata);
		}
		estatus = (struct ras_bert_hest_generic_status *)((u8 *)estatus + estatus_len);
		remain -= estatus_len;
	}

	return 0;
}
#endif
