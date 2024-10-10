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
 * DOC: Base kernel MMU faults decoder for CSF GPUs.
 */

#include <mmu/backend/mali_kbase_mmu_faults_decoder_luts_csf.h>

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

static struct decode_lut_element lut_fault_source_csf_r_t_major_10[] = {
	{ 0xFFFF, 0, "pref0" },
	{ 0xFFFF, 4, "iter0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_r_t_major_11[] = {
	{ 0xFFFF, 0, "pref0" },
	{ 0xFFFF, 4, "iter0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_r_t_major_12[] = {
	{ 0xFFFF, 0, "pref0" },
	{ 0xFFFF, 4, "iter0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_r_t_major_13[] = {
	{ 0xFFFF, 0, "pref0" },
	{ 0xFFFF, 4, "iter0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_r_t_major_14[] = {
	{ 0xFFFF, 0, "pref0_jasid0" }, { 0xFFFF, 4, "iter0_jasid0" }, { 0xFFFF, 12, "lsu_jasid0" },
	{ 0xFFFF, 13, "mcu_jasid0" },  { 0xFFFF, 16, "pref0_other" }, { 0xFFFF, 20, "iter0_other" },
	{ 0xFFFF, 28, "lsu_other" },   { 0xFFFF, 29, "mcu_other" },
};

static struct decode_lut_element lut_fault_source_csf_w_t_major_10[] = {
	{ 0xFFFF, 8, "pcb0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_w_t_major_11[] = {
	{ 0xFFFF, 8, "pcb0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_w_t_major_12[] = {
	{ 0xFFFF, 8, "pcb0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_w_t_major_13[] = {
	{ 0xFFFF, 8, "pcb0" },
	{ 0xFFFF, 12, "lsu" },
	{ 0xFFFF, 13, "mcu" },
};

static struct decode_lut_element lut_fault_source_csf_w_t_major_14[] = {
	{ 0xFFFF, 8, "pcb0_jasid0" }, { 0xFFFF, 12, "lsu_jasid0" }, { 0xFFFF, 13, "mcu_jasid0" },
	{ 0xFFFF, 24, "pcb0_other" }, { 0xFFFF, 28, "lsu_other" },  { 0xFFFF, 29, "mcu_other" },
};


const char *decode_fault_source_csf_r_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_r_t_major_10,
						NELEMS(lut_fault_source_csf_r_t_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_r_t_major_11,
						NELEMS(lut_fault_source_csf_r_t_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_r_t_major_12,
						NELEMS(lut_fault_source_csf_r_t_major_12));
		break;
	case 13:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_r_t_major_13,
						NELEMS(lut_fault_source_csf_r_t_major_13));
		break;
	case 14:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_r_t_major_14,
						NELEMS(lut_fault_source_csf_r_t_major_14));
		break;
	}
	return ret;
}

const char *decode_fault_source_csf_w_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_w_t_major_10,
						NELEMS(lut_fault_source_csf_w_t_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_w_t_major_11,
						NELEMS(lut_fault_source_csf_w_t_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_w_t_major_12,
						NELEMS(lut_fault_source_csf_w_t_major_12));
		break;
	case 13:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_w_t_major_13,
						NELEMS(lut_fault_source_csf_w_t_major_13));
		break;
	case 14:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_csf_w_t_major_14,
						NELEMS(lut_fault_source_csf_w_t_major_14));
		break;
	}
	return ret;
}
