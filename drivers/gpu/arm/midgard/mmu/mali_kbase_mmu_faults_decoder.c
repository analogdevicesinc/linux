// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2024 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

/**
 * DOC: Base kernel MMU faults decoder.
 */

#include <mmu/mali_kbase_mmu_faults_decoder.h>
#include <mmu/mali_kbase_mmu_faults_decoder_luts.h>
#if MALI_USE_CSF
#include <mmu/backend/mali_kbase_mmu_faults_decoder_luts_csf.h>
#else
#include <mmu/backend/mali_kbase_mmu_faults_decoder_luts_jm.h>
#endif

#include <hw_access/mali_kbase_hw_access_regmap.h>
#include <mali_kbase.h>

unsigned int fault_source_id_internal_requester_get(struct kbase_device *kbdev,
						    unsigned int source_id)
{
	if (kbdev->gpu_props.gpu_id.product_model < GPU_ID_MODEL_MAKE(14, 0))
		return ((source_id >> 4) & 0xF);
	else
		return (source_id & 0x3F);
}

static inline const char *source_id_enc_core_type_get_str(struct kbase_device *kbdev,
							  unsigned int source_id)
{
	if (kbdev->gpu_props.gpu_id.product_model < GPU_ID_MODEL_MAKE(14, 0))
		return decode_fault_source_core_id_t_core_type(
			FAULT_SOURCE_ID_CORE_ID_GET(source_id), kbdev->gpu_props.gpu_id.arch_id);
	else
		return decode_fault_source_core_type_t_name(
			FAULT_SOURCE_ID_CORE_TYPE_GET(source_id), kbdev->gpu_props.gpu_id.arch_id);
}
const char *fault_source_id_internal_requester_get_str(struct kbase_device *kbdev,
						       unsigned int source_id,
						       unsigned int access_type)
{
	unsigned int ir = fault_source_id_internal_requester_get(kbdev, source_id);
	bool older_source_id_fmt =
		(kbdev->gpu_props.gpu_id.product_model < GPU_ID_MODEL_MAKE(14, 0));
	unsigned int utlb_id = 0;

	if (older_source_id_fmt)
		utlb_id = FAULT_SOURCE_ID_UTLB_ID_GET(source_id);

	if (!strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "shader")) {
		if (utlb_id == 0) {
			if (access_type == AS_FAULTSTATUS_ACCESS_TYPE_READ)
				return decode_fault_source_shader_r_t(
					ir, kbdev->gpu_props.gpu_id.arch_id);
			else
				return decode_fault_source_shader_w_t(
					ir, kbdev->gpu_props.gpu_id.arch_id);
		} else
			return "Load/store cache";
	} else if (!strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "tiler")) {
#if MALI_USE_CSF
		if (utlb_id == 0) {
			if (access_type == AS_FAULTSTATUS_ACCESS_TYPE_READ)
				return decode_fault_source_tiler_r_t(
					ir, kbdev->gpu_props.gpu_id.arch_id);
			else
				return decode_fault_source_tiler_w_t(
					ir, kbdev->gpu_props.gpu_id.arch_id);
		} else
			return "The polygon list writer. No further details.";
#else
		return (utlb_id == 0) ? "Anything other than the polygon list writer" :
					      "The polygon list writer";
#endif
	}
#if MALI_USE_CSF
	else if (!strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "csf")) {
		if (access_type == AS_FAULTSTATUS_ACCESS_TYPE_READ)
			return decode_fault_source_csf_r_t(ir, kbdev->gpu_props.gpu_id.arch_id);
		else
			return decode_fault_source_csf_w_t(ir, kbdev->gpu_props.gpu_id.arch_id);
	}
#else
	else if (!strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "jm"))
		return decode_fault_source_jm_t(ir, kbdev->gpu_props.gpu_id.arch_id);
#endif
	else if (!strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "I2c") ||
		 !strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "memsys") ||
		 !strcmp(source_id_enc_core_type_get_str(kbdev, source_id), "mmu")) {
		return "Not used";
	}

	return "unknown";
}

const char *fault_source_id_core_type_description_get(struct kbase_device *kbdev,
						      unsigned int source_id)
{
	if (kbdev->gpu_props.gpu_id.product_model < GPU_ID_MODEL_MAKE(14, 0)) {
		return decode_fault_source_core_id_t_desc(FAULT_SOURCE_ID_CORE_ID_GET(source_id),
							  kbdev->gpu_props.gpu_id.arch_id);
	} else {
		return decode_fault_source_core_type_t_desc(
			FAULT_SOURCE_ID_CORE_TYPE_GET(source_id), kbdev->gpu_props.gpu_id.arch_id);
	}
}
