// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
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

#include "amdgpu.h"
#include "amdgpu_ras_bert.h"

#if defined(CONFIG_X86_MCE_AMD) && defined(CONFIG_ACPI_APEI)
#include <linux/acpi.h>
#include <linux/mutex.h>

#include "amdgpu_ras_mgr.h"

static DEFINE_MUTEX(boot_err_lock);
static bool boot_err_polled, boot_err_written[MAX_GPU_INSTANCE];
static int boot_err_poll_result;
static u8 *boot_err_raw_data;
static u32 boot_err_raw_data_len;

int amdgpu_ras_bert_process_boot_errors(struct amdgpu_device *adev)
{
	struct amdgpu_ras_mgr *ras_mgr = amdgpu_ras_mgr_get_context(adev);
	struct acpi_bert_region *boot_error_region;
	struct acpi_table_header *table;
	struct acpi_table_bert *bert_tab;
	unsigned int region_len;
	acpi_status status;
	u32 socket_id;
	int ret;

	if (!ras_mgr || !ras_mgr->ras_core)
		return -EPERM;

	mutex_lock(&boot_err_lock);

	if (!boot_err_polled) {
		boot_err_polled = true;
		boot_err_poll_result = 0;
		ret = 0;

		if (acpi_disabled)
			goto out_unlock;

		status = acpi_get_table(ACPI_SIG_BERT, 0, &table);
		if (status == AE_NOT_FOUND)
			goto out_unlock;

		if (ACPI_FAILURE(status)) {
			RAS_DEV_ERR(adev, "get table failed, %s.\n", acpi_format_exception(status));
			boot_err_poll_result = -EINVAL;
			goto out_unlock;
		}
		bert_tab = (struct acpi_table_bert *)table;

		if (bert_tab->header.length < sizeof(struct acpi_table_bert) ||
		    bert_tab->region_length < sizeof(struct acpi_bert_region)) {
			RAS_DEV_ERR(adev, "table invalid.\n");
			boot_err_poll_result = -EINVAL;
			goto out_put_bert_tab;
		}

		region_len = bert_tab->region_length;
		boot_error_region = acpi_os_map_memory(bert_tab->address, region_len);
		if (boot_error_region) {
			boot_err_raw_data = kmemdup(boot_error_region, region_len, GFP_KERNEL);
			acpi_os_unmap_memory(boot_error_region, region_len);
			if (!boot_err_raw_data) {
				RAS_DEV_ERR(adev, "failed to cache BERT raw data.\n");
				boot_err_poll_result = -ENOMEM;
			} else {
				boot_err_raw_data_len = region_len;
			}
		} else {
			RAS_DEV_ERR(adev, "failed to map BERT region.\n");
			boot_err_poll_result = -ENOMEM;
		}

out_put_bert_tab:
		acpi_put_table(table);
	}

	if (boot_err_poll_result) {
		ret = boot_err_poll_result;
		goto out_unlock;
	}

	if (!boot_err_raw_data || !boot_err_raw_data_len ||
	    !(adev && (adev->gmc.xgmi.connected_to_cpu || adev->gmc.is_app_apu))) {
		ret = 0;
		goto out_unlock;
	}

	if (!adev->smuio.funcs || !adev->smuio.funcs->get_socket_id) {
		RAS_DEV_WARN(adev, "No interface to obtain current device socket ID!\n");
		ret = 0;
		goto out_unlock;
	}

	socket_id = adev->smuio.funcs->get_socket_id(adev);
	if (socket_id >= MAX_GPU_INSTANCE) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (boot_err_written[socket_id]) {
		ret = 0;
		goto out_unlock;
	}

	ret = ras_bert_process_records(ras_mgr->ras_core,
				       boot_err_raw_data, boot_err_raw_data_len);
	if (!ret)
		boot_err_written[socket_id] = true;

out_unlock:
	mutex_unlock(&boot_err_lock);

	return ret;
}

void amdgpu_ras_bert_reset_boot_errors(void)
{
	mutex_lock(&boot_err_lock);
	memset(boot_err_written, 0, sizeof(boot_err_written));
	kfree(boot_err_raw_data);
	boot_err_raw_data = NULL;
	boot_err_raw_data_len = 0;
	boot_err_poll_result = 0;
	boot_err_polled = false;
	mutex_unlock(&boot_err_lock);
}
#endif
