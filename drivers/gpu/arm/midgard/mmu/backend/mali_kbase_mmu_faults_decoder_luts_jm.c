// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2024 ARM Limited. All rights reserved.
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
 * DOC: Base kernel MMU faults decoder for Job Manager GPUs.
 */

#include <mmu/backend/mali_kbase_mmu_faults_decoder_luts_jm.h>

#define GPU_ID_ARCH_ID_MAJOR_GET(gpu_id) ((gpu_id >> 16) & 0xFF)
#define GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id) (gpu_id & 0xFFFF)
#define NELEMS(s) (sizeof(s) / sizeof((s)[0]))

struct decode_lut_element {
	u16 arch_minor_rev;
	u16 key;
	const char *text;
};

static const char *decode_lut_element_lookup(u16 arch_minor_rev, u16 key,
					     struct decode_lut_element *decode_element_lut,
					     unsigned int lut_len)
{
	struct decode_lut_element *p;

	for (p = decode_element_lut; p < decode_element_lut + lut_len; p++) {
		if (p->key == key &&
		    (p->arch_minor_rev == 0xffff || p->arch_minor_rev == arch_minor_rev))
			break;
	}
	if (p < decode_element_lut + lut_len)
		return p->text;
	else
		return "unknown";
}

/* Auto-generated code: DO NOT MODIFY! */

static struct decode_lut_element lut_fault_source_jm_t_major_9[] = {
	{ 0xFFFF, 0, "js" },
	{ 0xFFFF, 1, "pcm" },
};

const char *decode_fault_source_jm_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_jm_t_major_9,
						NELEMS(lut_fault_source_jm_t_major_9));
		break;
	}
	return ret;
}
