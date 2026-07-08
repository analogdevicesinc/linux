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
int ras_bert_process_records(struct ras_core_context *ras_core,
			     const void *bert, u32 bert_len)
{
	struct acpi_hest_generic_status *estatus;
	struct acpi_hest_generic_data *gdata;
	u32 estatus_len;
	int remain;

	if (!ras_core || !bert || bert_len < sizeof(struct acpi_hest_generic_status))
		return -EINVAL;

	estatus = (struct acpi_hest_generic_status *)bert;
	remain = bert_len;

	while (remain >= sizeof(struct acpi_hest_generic_status)) {
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
		if (cper_estatus_check(estatus)) {
			RAS_DEV_ERR(ras_core->dev, "invalid error record.\n");
			return -EINVAL;
		}

		gdata = (struct acpi_hest_generic_data *)(estatus + 1);
		while ((void *)gdata - (void *)(estatus + 1) < estatus->data_length) {
			RAS_DEV_INFO(ras_core->dev, "unknown section: %pUl\n",
				     gdata->section_type);

			gdata = acpi_hest_get_next(gdata);
		}
		estatus = (struct acpi_hest_generic_status *)((u8 *)estatus + estatus_len);
		remain -= estatus_len;
	}

	return 0;
}
#endif
