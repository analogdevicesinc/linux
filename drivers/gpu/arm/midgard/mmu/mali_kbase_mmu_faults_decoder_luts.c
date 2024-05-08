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
 * DOC: Base kernel MMU faults decoder.
 */

#include <mmu/mali_kbase_mmu_faults_decoder_luts.h>

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

static struct decode_lut_element lut_fault_source_core_type_t_name_major_9[] = {
	{ 0xFFFF, 0, "shader" }, { 0xFFFF, 1, "l2c" }, { 0xFFFF, 2, "tiler" },
	{ 0xFFFF, 3, "mmu" },	 { 0xFFFF, 4, "jm" },  { 0xFFFF, 5, "pmb" },
};

static struct decode_lut_element lut_fault_source_core_type_t_desc_major_9[] = {
	{ 0xFFFF, 0, "Shader core" }, { 0xFFFF, 1, "Level 2 cache" },
	{ 0xFFFF, 2, "Tiler" },	      { 0xFFFF, 3, "MMU" },
	{ 0xFFFF, 4, "Job Manager" }, { 0xFFFF, 5, "Performance Monitor Block" },
};

static struct decode_lut_element lut_fault_source_core_type_t_name_major_10[] = {
	{ 0xFFFF, 0, "shader" }, { 0xFFFF, 1, "l2c" }, { 0xFFFF, 2, "tiler" },
	{ 0xFFFF, 3, "mmu" },	 { 0xFFFF, 4, "csf" }, { 0xFFFF, 5, "memsys" },
};

static struct decode_lut_element lut_fault_source_core_type_t_desc_major_10[] = {
	{ 0xFFFF, 0, "Shader core" }, { 0xFFFF, 1, "Level 2 cache" },
	{ 0xFFFF, 2, "Tiler" },	      { 0xFFFF, 3, "MMU" },
	{ 0xFFFF, 4, "CSF" },	      { 0xFFFF, 5, "Memory system" },
};

static struct decode_lut_element lut_fault_source_core_type_t_name_major_11[] = {
	{ 0xFFFF, 0, "shader" }, { 0xFFFF, 1, "l2c" }, { 0xFFFF, 2, "tiler" },
	{ 0xFFFF, 3, "mmu" },	 { 0xFFFF, 4, "csf" }, { 0xFFFF, 5, "memsys" },
};

static struct decode_lut_element lut_fault_source_core_type_t_desc_major_11[] = {
	{ 0xFFFF, 0, "Shader core" }, { 0xFFFF, 1, "Level 2 cache" },
	{ 0xFFFF, 2, "Tiler" },	      { 0xFFFF, 3, "MMU" },
	{ 0xFFFF, 4, "CSF" },	      { 0xFFFF, 5, "Memory system" },
};

static struct decode_lut_element lut_fault_source_core_type_t_name_major_12[] = {
	{ 0xFFFF, 0, "shader" }, { 0xFFFF, 1, "l2c" }, { 0xFFFF, 2, "tiler" },
	{ 0xFFFF, 3, "mmu" },	 { 0xFFFF, 4, "csf" }, { 0xFFFF, 5, "memsys" },
};

static struct decode_lut_element lut_fault_source_core_type_t_desc_major_12[] = {
	{ 0xFFFF, 0, "Shader core" }, { 0xFFFF, 1, "Level 2 cache" },
	{ 0xFFFF, 2, "Tiler" },	      { 0xFFFF, 3, "MMU" },
	{ 0xFFFF, 4, "CSF" },	      { 0xFFFF, 5, "Memory system" },
};

static struct decode_lut_element lut_fault_source_core_id_t_desc_major_9[] = {
	{ 0xFFFF, 0, "Shader core 0" },
	{ 0xFFFF, 1, "Shader core 1" },
	{ 0xFFFF, 2, "Shader core 2" },
	{ 0xFFFF, 3, "Shader core 3" },
	{ 0xFFFF, 4, "Shader core 4" },
	{ 0xFFFF, 5, "Shader core 5" },
	{ 0xFFFF, 6, "Shader core 6" },
	{ 0xFFFF, 7, "Shader core 7" },
	{ 0xFFFF, 8, "Shader core 8" },
	{ 0xFFFF, 9, "Shader core 9" },
	{ 0xFFFF, 10, "Shader core 10" },
	{ 0xFFFF, 11, "Shader core 11" },
	{ 0xFFFF, 12, "Shader core 12" },
	{ 0xFFFF, 13, "Shader core 13" },
	{ 0xFFFF, 14, "Shader core 14" },
	{ 0xFFFF, 15, "Shader core 15" },
	{ 0xFFFF, 16, "Shader core 16" },
	{ 0xFFFF, 17, "Shader core 17" },
	{ 0xFFFF, 18, "Shader core 18" },
	{ 0xFFFF, 19, "Shader core 19" },
	{ 0xFFFF, 20, "Shader core 20" },
	{ 0xFFFF, 21, "Shader core 21" },
	{ 0xFFFF, 22, "Shader core 22" },
	{ 0xFFFF, 23, "Shader core 23" },
	{ 0xFFFF, 24, "Shader core 24" },
	{ 0xFFFF, 25, "Shader core 25" },
	{ 0xFFFF, 26, "Shader core 26" },
	{ 0xFFFF, 27, "Shader core 27" },
	{ 0xFFFF, 28, "Shader core 28" },
	{ 0xFFFF, 29, "Shader core 29" },
	{ 0xFFFF, 30, "Shader core 30" },
	{ 0xFFFF, 31, "Shader core 31" },
	{ 0xFFFF, 41, "L2 Slice 3" },
	{ 0xFFFF, 43, "L2 Slice 2" },
	{ 0xFFFF, 45, "L2 Slice 1" },
	{ 0xFFFF, 46, "PMB" },
	{ 0xFFFF, 47, "L2 Slice 0" },
	{ 0xFFFF, 51, "Tiler" },
	{ 0xFFFF, 55, "MMU" },
	{ 0xFFFF, 62, "Job Manager" },
};

static struct decode_lut_element lut_fault_source_core_id_t_core_type_major_9[] = {
	{ 0xFFFF, 0, "shader" },  { 0xFFFF, 1, "shader" },  { 0xFFFF, 2, "shader" },
	{ 0xFFFF, 3, "shader" },  { 0xFFFF, 4, "shader" },  { 0xFFFF, 5, "shader" },
	{ 0xFFFF, 6, "shader" },  { 0xFFFF, 7, "shader" },  { 0xFFFF, 8, "shader" },
	{ 0xFFFF, 9, "shader" },  { 0xFFFF, 10, "shader" }, { 0xFFFF, 11, "shader" },
	{ 0xFFFF, 12, "shader" }, { 0xFFFF, 13, "shader" }, { 0xFFFF, 14, "shader" },
	{ 0xFFFF, 15, "shader" }, { 0xFFFF, 16, "shader" }, { 0xFFFF, 17, "shader" },
	{ 0xFFFF, 18, "shader" }, { 0xFFFF, 19, "shader" }, { 0xFFFF, 20, "shader" },
	{ 0xFFFF, 21, "shader" }, { 0xFFFF, 22, "shader" }, { 0xFFFF, 23, "shader" },
	{ 0xFFFF, 24, "shader" }, { 0xFFFF, 25, "shader" }, { 0xFFFF, 26, "shader" },
	{ 0xFFFF, 27, "shader" }, { 0xFFFF, 28, "shader" }, { 0xFFFF, 29, "shader" },
	{ 0xFFFF, 30, "shader" }, { 0xFFFF, 31, "shader" }, { 0xFFFF, 41, "l2c" },
	{ 0xFFFF, 43, "l2c" },	  { 0xFFFF, 45, "l2c" },    { 0xFFFF, 46, "pmb" },
	{ 0xFFFF, 47, "l2c" },	  { 0xFFFF, 51, "tiler" },  { 0xFFFF, 55, "mmu" },
	{ 0xFFFF, 62, "jm" },
};

static struct decode_lut_element lut_fault_source_core_id_t_desc_major_10[] = {
	{ 0xFFFF, 0, "Shader core 0" },
	{ 0xFFFF, 1, "Shader core 1" },
	{ 0xFFFF, 2, "Shader core 2" },
	{ 0xFFFF, 3, "Shader core 3" },
	{ 0xFFFF, 4, "Shader core 4" },
	{ 0xFFFF, 5, "Shader core 5" },
	{ 0xFFFF, 6, "Shader core 6" },
	{ 0xFFFF, 7, "Shader core 7" },
	{ 0xFFFF, 8, "Shader core 8" },
	{ 0xFFFF, 9, "Shader core 9" },
	{ 0xFFFF, 10, "Shader core 10" },
	{ 0xFFFF, 11, "Shader core 11" },
	{ 0xFFFF, 12, "Shader core 12" },
	{ 0xFFFF, 13, "Shader core 13" },
	{ 0xFFFF, 14, "Shader core 14" },
	{ 0xFFFF, 15, "Shader core 15" },
	{ 0xFFFF, 16, "Shader core 16" },
	{ 0xFFFF, 17, "Shader core 17" },
	{ 0xFFFF, 18, "Shader core 18" },
	{ 0xFFFF, 19, "Shader core 19" },
	{ 0xFFFF, 20, "Shader core 20" },
	{ 0xFFFF, 21, "Shader core 21" },
	{ 0xFFFF, 22, "Shader core 22" },
	{ 0xFFFF, 23, "Shader core 23" },
	{ 0xFFFF, 24, "Shader core 24" },
	{ 0xFFFF, 25, "Shader core 25" },
	{ 0xFFFF, 26, "Shader core 26" },
	{ 0xFFFF, 27, "Shader core 27" },
	{ 0xFFFF, 28, "Shader core 28" },
	{ 0xFFFF, 29, "Shader core 29" },
	{ 0xFFFF, 30, "Shader core 30" },
	{ 0xFFFF, 31, "Shader core 31" },
	{ 0xFFFF, 41, "L2 Slice 3" },
	{ 0xFFFF, 43, "L2 Slice 2" },
	{ 0xFFFF, 45, "L2 Slice 1" },
	{ 0xFFFF, 47, "L2 Slice 0" },
	{ 0xFFFF, 51, "Tiler" },
	{ 0xFFFF, 55, "MMU" },
	{ 0xFFFF, 33, "L2 Slice 7" },
	{ 0xFFFF, 35, "L2 Slice 6" },
	{ 0xFFFF, 37, "L2 Slice 5" },
	{ 0xFFFF, 39, "L2 Slice 4" },
	{ 0xFFFF, 48, "Memory system, undefined" },
	{ 0xFFFF, 62, "Command Stream Frontend" },
};

static struct decode_lut_element lut_fault_source_core_id_t_core_type_major_10[] = {
	{ 0xFFFF, 0, "shader" },  { 0xFFFF, 1, "shader" },  { 0xFFFF, 2, "shader" },
	{ 0xFFFF, 3, "shader" },  { 0xFFFF, 4, "shader" },  { 0xFFFF, 5, "shader" },
	{ 0xFFFF, 6, "shader" },  { 0xFFFF, 7, "shader" },  { 0xFFFF, 8, "shader" },
	{ 0xFFFF, 9, "shader" },  { 0xFFFF, 10, "shader" }, { 0xFFFF, 11, "shader" },
	{ 0xFFFF, 12, "shader" }, { 0xFFFF, 13, "shader" }, { 0xFFFF, 14, "shader" },
	{ 0xFFFF, 15, "shader" }, { 0xFFFF, 16, "shader" }, { 0xFFFF, 17, "shader" },
	{ 0xFFFF, 18, "shader" }, { 0xFFFF, 19, "shader" }, { 0xFFFF, 20, "shader" },
	{ 0xFFFF, 21, "shader" }, { 0xFFFF, 22, "shader" }, { 0xFFFF, 23, "shader" },
	{ 0xFFFF, 24, "shader" }, { 0xFFFF, 25, "shader" }, { 0xFFFF, 26, "shader" },
	{ 0xFFFF, 27, "shader" }, { 0xFFFF, 28, "shader" }, { 0xFFFF, 29, "shader" },
	{ 0xFFFF, 30, "shader" }, { 0xFFFF, 31, "shader" }, { 0xFFFF, 41, "l2c" },
	{ 0xFFFF, 43, "l2c" },	  { 0xFFFF, 45, "l2c" },    { 0xFFFF, 47, "l2c" },
	{ 0xFFFF, 51, "tiler" },  { 0xFFFF, 55, "mmu" },    { 0xFFFF, 33, "l2c" },
	{ 0xFFFF, 35, "l2c" },	  { 0xFFFF, 37, "l2c" },    { 0xFFFF, 39, "l2c" },
	{ 0xFFFF, 48, "memsys" }, { 0xFFFF, 62, "csf" },
};

static struct decode_lut_element lut_fault_source_core_id_t_desc_major_11[] = {
	{ 0xFFFF, 0, "Shader core 0" },
	{ 0xFFFF, 1, "Shader core 1" },
	{ 0xFFFF, 2, "Shader core 2" },
	{ 0xFFFF, 3, "Shader core 3" },
	{ 0xFFFF, 4, "Shader core 4" },
	{ 0xFFFF, 5, "Shader core 5" },
	{ 0xFFFF, 6, "Shader core 6" },
	{ 0xFFFF, 7, "Shader core 7" },
	{ 0xFFFF, 8, "Shader core 8" },
	{ 0xFFFF, 9, "Shader core 9" },
	{ 0xFFFF, 10, "Shader core 10" },
	{ 0xFFFF, 11, "Shader core 11" },
	{ 0xFFFF, 12, "Shader core 12" },
	{ 0xFFFF, 13, "Shader core 13" },
	{ 0xFFFF, 14, "Shader core 14" },
	{ 0xFFFF, 15, "Shader core 15" },
	{ 0xFFFF, 16, "Shader core 16" },
	{ 0xFFFF, 17, "Shader core 17" },
	{ 0xFFFF, 18, "Shader core 18" },
	{ 0xFFFF, 19, "Shader core 19" },
	{ 0xFFFF, 20, "Shader core 20" },
	{ 0xFFFF, 21, "Shader core 21" },
	{ 0xFFFF, 22, "Shader core 22" },
	{ 0xFFFF, 23, "Shader core 23" },
	{ 0xFFFF, 24, "Shader core 24" },
	{ 0xFFFF, 25, "Shader core 25" },
	{ 0xFFFF, 26, "Shader core 26" },
	{ 0xFFFF, 27, "Shader core 27" },
	{ 0xFFFF, 28, "Shader core 28" },
	{ 0xFFFF, 29, "Shader core 29" },
	{ 0xFFFF, 30, "Shader core 30" },
	{ 0xFFFF, 31, "Shader core 31" },
	{ 0xFFFF, 41, "L2 Slice 3" },
	{ 0xFFFF, 43, "L2 Slice 2" },
	{ 0xFFFF, 45, "L2 Slice 1" },
	{ 0xFFFF, 47, "L2 Slice 0" },
	{ 0xFFFF, 51, "Tiler" },
	{ 0xFFFF, 55, "MMU" },
	{ 0xFFFF, 33, "L2 Slice 7" },
	{ 0xFFFF, 35, "L2 Slice 6" },
	{ 0xFFFF, 37, "L2 Slice 5" },
	{ 0xFFFF, 39, "L2 Slice 4" },
	{ 0xFFFF, 48, "Memory system, undefined" },
	{ 0xFFFF, 62, "Command Stream Frontend" },
};

static struct decode_lut_element lut_fault_source_core_id_t_core_type_major_11[] = {
	{ 0xFFFF, 0, "shader" },  { 0xFFFF, 1, "shader" },  { 0xFFFF, 2, "shader" },
	{ 0xFFFF, 3, "shader" },  { 0xFFFF, 4, "shader" },  { 0xFFFF, 5, "shader" },
	{ 0xFFFF, 6, "shader" },  { 0xFFFF, 7, "shader" },  { 0xFFFF, 8, "shader" },
	{ 0xFFFF, 9, "shader" },  { 0xFFFF, 10, "shader" }, { 0xFFFF, 11, "shader" },
	{ 0xFFFF, 12, "shader" }, { 0xFFFF, 13, "shader" }, { 0xFFFF, 14, "shader" },
	{ 0xFFFF, 15, "shader" }, { 0xFFFF, 16, "shader" }, { 0xFFFF, 17, "shader" },
	{ 0xFFFF, 18, "shader" }, { 0xFFFF, 19, "shader" }, { 0xFFFF, 20, "shader" },
	{ 0xFFFF, 21, "shader" }, { 0xFFFF, 22, "shader" }, { 0xFFFF, 23, "shader" },
	{ 0xFFFF, 24, "shader" }, { 0xFFFF, 25, "shader" }, { 0xFFFF, 26, "shader" },
	{ 0xFFFF, 27, "shader" }, { 0xFFFF, 28, "shader" }, { 0xFFFF, 29, "shader" },
	{ 0xFFFF, 30, "shader" }, { 0xFFFF, 31, "shader" }, { 0xFFFF, 41, "l2c" },
	{ 0xFFFF, 43, "l2c" },	  { 0xFFFF, 45, "l2c" },    { 0xFFFF, 47, "l2c" },
	{ 0xFFFF, 51, "tiler" },  { 0xFFFF, 55, "mmu" },    { 0xFFFF, 33, "l2c" },
	{ 0xFFFF, 35, "l2c" },	  { 0xFFFF, 37, "l2c" },    { 0xFFFF, 39, "l2c" },
	{ 0xFFFF, 48, "memsys" }, { 0xFFFF, 62, "csf" },
};

static struct decode_lut_element lut_fault_source_core_id_t_desc_major_12[] = {
	{ 0xFFFF, 0, "Shader core 0" },
	{ 0xFFFF, 1, "Shader core 1" },
	{ 0xFFFF, 2, "Shader core 2" },
	{ 0xFFFF, 3, "Shader core 3" },
	{ 0xFFFF, 4, "Shader core 4" },
	{ 0xFFFF, 5, "Shader core 5" },
	{ 0xFFFF, 6, "Shader core 6" },
	{ 0xFFFF, 7, "Shader core 7" },
	{ 0xFFFF, 8, "Shader core 8" },
	{ 0xFFFF, 9, "Shader core 9" },
	{ 0xFFFF, 10, "Shader core 10" },
	{ 0xFFFF, 11, "Shader core 11" },
	{ 0xFFFF, 12, "Shader core 12" },
	{ 0xFFFF, 13, "Shader core 13" },
	{ 0xFFFF, 14, "Shader core 14" },
	{ 0xFFFF, 15, "Shader core 15" },
	{ 0xFFFF, 16, "Shader core 16" },
	{ 0xFFFF, 17, "Shader core 17" },
	{ 0xFFFF, 18, "Shader core 18" },
	{ 0xFFFF, 19, "Shader core 19" },
	{ 0xFFFF, 20, "Shader core 20" },
	{ 0xFFFF, 21, "Shader core 21" },
	{ 0xFFFF, 22, "Shader core 22" },
	{ 0xFFFF, 23, "Shader core 23" },
	{ 0xFFFF, 24, "Shader core 24" },
	{ 0xFFFF, 25, "Shader core 25" },
	{ 0xFFFF, 26, "Shader core 26" },
	{ 0xFFFF, 27, "Shader core 27" },
	{ 0xFFFF, 28, "Shader core 28" },
	{ 0xFFFF, 29, "Shader core 29" },
	{ 0xFFFF, 30, "Shader core 30" },
	{ 0xFFFF, 31, "Shader core 31" },
	{ 0xFFFF, 41, "L2 Slice 3" },
	{ 0xFFFF, 43, "L2 Slice 2" },
	{ 0xFFFF, 45, "L2 Slice 1" },
	{ 0xFFFF, 47, "L2 Slice 0" },
	{ 0xFFFF, 51, "Tiler" },
	{ 0xFFFF, 55, "MMU" },
	{ 0xFFFF, 33, "L2 Slice 7" },
	{ 0xFFFF, 35, "L2 Slice 6" },
	{ 0xFFFF, 37, "L2 Slice 5" },
	{ 0xFFFF, 39, "L2 Slice 4" },
	{ 0xFFFF, 48, "Memory system, undefined" },
	{ 0xFFFF, 62, "Command Stream Frontend" },
};

static struct decode_lut_element lut_fault_source_core_id_t_core_type_major_12[] = {
	{ 0xFFFF, 0, "shader" },  { 0xFFFF, 1, "shader" },  { 0xFFFF, 2, "shader" },
	{ 0xFFFF, 3, "shader" },  { 0xFFFF, 4, "shader" },  { 0xFFFF, 5, "shader" },
	{ 0xFFFF, 6, "shader" },  { 0xFFFF, 7, "shader" },  { 0xFFFF, 8, "shader" },
	{ 0xFFFF, 9, "shader" },  { 0xFFFF, 10, "shader" }, { 0xFFFF, 11, "shader" },
	{ 0xFFFF, 12, "shader" }, { 0xFFFF, 13, "shader" }, { 0xFFFF, 14, "shader" },
	{ 0xFFFF, 15, "shader" }, { 0xFFFF, 16, "shader" }, { 0xFFFF, 17, "shader" },
	{ 0xFFFF, 18, "shader" }, { 0xFFFF, 19, "shader" }, { 0xFFFF, 20, "shader" },
	{ 0xFFFF, 21, "shader" }, { 0xFFFF, 22, "shader" }, { 0xFFFF, 23, "shader" },
	{ 0xFFFF, 24, "shader" }, { 0xFFFF, 25, "shader" }, { 0xFFFF, 26, "shader" },
	{ 0xFFFF, 27, "shader" }, { 0xFFFF, 28, "shader" }, { 0xFFFF, 29, "shader" },
	{ 0xFFFF, 30, "shader" }, { 0xFFFF, 31, "shader" }, { 0xFFFF, 41, "l2c" },
	{ 0xFFFF, 43, "l2c" },	  { 0xFFFF, 45, "l2c" },    { 0xFFFF, 47, "l2c" },
	{ 0xFFFF, 51, "tiler" },  { 0xFFFF, 55, "mmu" },    { 0xFFFF, 33, "l2c" },
	{ 0xFFFF, 35, "l2c" },	  { 0xFFFF, 37, "l2c" },    { 0xFFFF, 39, "l2c" },
	{ 0xFFFF, 48, "memsys" }, { 0xFFFF, 62, "csf" },
};

static struct decode_lut_element lut_fault_source_shader_r_t_major_9[] = {
	{ 0xFFFF, 0, "ic" },	{ 0xFFFF, 1, "adc" },	{ 0xFFFF, 4, "scm" },
	{ 0xFFFF, 5, "vl" },	{ 0xFFFF, 6, "plr" },	{ 0xFFFF, 7, "fsdc" },
	{ 0xFFFF, 8, "lsc" },	{ 0xFFFF, 9, "cse" },	{ 0xFFFF, 10, "tb" },
	{ 0xFFFF, 11, "tmdi" }, { 0xFFFF, 12, "tmu0" }, { 0xFFFF, 13, "tmu1" },
	{ 0xFFFF, 14, "tma0" }, { 0xFFFF, 15, "tma1" },
};

static struct decode_lut_element lut_fault_source_shader_r_t_major_10[] = {
	{ 0xFFFF, 4, "scm" },	{ 0xFFFF, 5, "vl" },	{ 0xFFFF, 6, "plr" },
	{ 0xFFFF, 7, "fsdc" },	{ 0xFFFF, 8, "lsc" },	{ 0xFFFF, 9, "cse" },
	{ 0xFFFF, 10, "tb" },	{ 0xFFFF, 11, "tmdi" }, { 0xFFFF, 12, "tmu0" },
	{ 0xFFFF, 13, "tmu1" }, { 0xFFFF, 14, "tma0" }, { 0xFFFF, 15, "tma1" },
	{ 0xFFFF, 0, "ic0" },	{ 0xFFFF, 1, "ic1" },	{ 0xFFFF, 2, "adc" },
};

static struct decode_lut_element lut_fault_source_shader_r_t_major_11[] = {
	{ 0xFFFF, 4, "scm" },	{ 0xFFFF, 5, "vl" },	{ 0xFFFF, 6, "plr" },
	{ 0xFFFF, 7, "fsdc" },	{ 0xFFFF, 8, "lsc" },	{ 0xFFFF, 9, "cse" },
	{ 0xFFFF, 10, "tb" },	{ 0xFFFF, 11, "tmdi" }, { 0xFFFF, 12, "tmu0" },
	{ 0xFFFF, 13, "tmu1" }, { 0xFFFF, 14, "tma0" }, { 0xFFFF, 15, "tma1" },
	{ 0xFFFF, 0, "ic0" },	{ 0xFFFF, 1, "ic1" },	{ 0xFFFF, 2, "adc" },
};

static struct decode_lut_element lut_fault_source_shader_r_t_major_12[] = {
	{ 0xFFFF, 4, "scm" },	{ 0xFFFF, 6, "plr" },	{ 0xFFFF, 7, "fsdc" },
	{ 0xFFFF, 8, "lsc" },	{ 0xFFFF, 9, "cse" },	{ 0xFFFF, 10, "tb" },
	{ 0xFFFF, 11, "tmdi" }, { 0xFFFF, 12, "tmu0" }, { 0xFFFF, 13, "tmu1" },
	{ 0xFFFF, 14, "tma0" }, { 0xFFFF, 15, "tma1" }, { 0xFFFF, 0, "ic0" },
	{ 0xFFFF, 1, "ic1" },	{ 0xFFFF, 2, "adc" },	{ 0xFFFF, 3, "rtas" },
};

static struct decode_lut_element lut_fault_source_shader_w_t_major_9[] = {
	{ 0xFFFF, 0, "pcb" },
	{ 0xFFFF, 8, "lsc" },
	{ 0xFFFF, 10, "tb" },
};

static struct decode_lut_element lut_fault_source_shader_w_t_major_10[] = {
	{ 0xFFFF, 0, "pcb" },  { 0xFFFF, 8, "lsc" },  { 0xFFFF, 12, "tb0" },
	{ 0xFFFF, 13, "tb1" }, { 0xFFFF, 14, "tb2" }, { 0xFFFF, 15, "tb3" },
};

static struct decode_lut_element lut_fault_source_shader_w_t_major_11[] = {
	{ 0xFFFF, 0, "pcb" },  { 0xFFFF, 8, "lsc" },  { 0xFFFF, 12, "tb0" },
	{ 0xFFFF, 13, "tb1" }, { 0xFFFF, 14, "tb2" }, { 0xFFFF, 15, "tb3" },
};

static struct decode_lut_element lut_fault_source_shader_w_t_major_12[] = {
	{ 0xFFFF, 0, "pcb" },  { 0xFFFF, 8, "lsc" },  { 0xFFFF, 12, "tb0" },
	{ 0xFFFF, 13, "tb1" }, { 0xFFFF, 14, "tb2" }, { 0xFFFF, 15, "tb3" },
};

static struct decode_lut_element lut_fault_source_tiler_r_t_major_10[] = {
	{ 0xFFFF, 0, "pf" },
	{ 0xFFFF, 1, "pcache" },
	{ 0xFFFF, 2, "tcu" },
	{ 0xFFFF, 3, "idx" },
};

static struct decode_lut_element lut_fault_source_tiler_r_t_major_11[] = {
	{ 0xFFFF, 0, "pf" },
	{ 0xFFFF, 1, "pcache" },
	{ 0xFFFF, 2, "tcu" },
	{ 0xFFFF, 3, "idx" },
};

static struct decode_lut_element lut_fault_source_tiler_r_t_major_12[] = {
	{ 0xFFFF, 0, "pf" },
	{ 0xFFFF, 1, "pcache" },
	{ 0xFFFF, 2, "tcu" },
	{ 0xFFFF, 3, "idx" },
};

static struct decode_lut_element lut_fault_source_tiler_w_t_major_10[] = {
	{ 0xFFFF, 1, "pcache_wb" },
	{ 0xFFFF, 2, "tcu_pcb" },
};

static struct decode_lut_element lut_fault_source_tiler_w_t_major_11[] = {
	{ 0xFFFF, 1, "pcache_wb" },
	{ 0xFFFF, 2, "tcu_pcb" },
};

static struct decode_lut_element lut_fault_source_tiler_w_t_major_12[] = {
	{ 0xFFFF, 1, "pcache_wb" },
	{ 0xFFFF, 2, "tcu_pcb" },
};


const char *decode_fault_source_core_type_t_name(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_name_major_9,
						NELEMS(lut_fault_source_core_type_t_name_major_9));
		break;
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_name_major_10,
						NELEMS(lut_fault_source_core_type_t_name_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_name_major_11,
						NELEMS(lut_fault_source_core_type_t_name_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_name_major_12,
						NELEMS(lut_fault_source_core_type_t_name_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_core_type_t_desc(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_desc_major_9,
						NELEMS(lut_fault_source_core_type_t_desc_major_9));
		break;
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_desc_major_10,
						NELEMS(lut_fault_source_core_type_t_desc_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_desc_major_11,
						NELEMS(lut_fault_source_core_type_t_desc_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_type_t_desc_major_12,
						NELEMS(lut_fault_source_core_type_t_desc_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_core_id_t_desc(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_id_t_desc_major_9,
						NELEMS(lut_fault_source_core_id_t_desc_major_9));
		break;
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_id_t_desc_major_10,
						NELEMS(lut_fault_source_core_id_t_desc_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_id_t_desc_major_11,
						NELEMS(lut_fault_source_core_id_t_desc_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx,
						lut_fault_source_core_id_t_desc_major_12,
						NELEMS(lut_fault_source_core_id_t_desc_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_core_id_t_core_type(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(
			min_rev, idx, lut_fault_source_core_id_t_core_type_major_9,
			NELEMS(lut_fault_source_core_id_t_core_type_major_9));
		break;
	case 10:
		ret = decode_lut_element_lookup(
			min_rev, idx, lut_fault_source_core_id_t_core_type_major_10,
			NELEMS(lut_fault_source_core_id_t_core_type_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(
			min_rev, idx, lut_fault_source_core_id_t_core_type_major_11,
			NELEMS(lut_fault_source_core_id_t_core_type_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(
			min_rev, idx, lut_fault_source_core_id_t_core_type_major_12,
			NELEMS(lut_fault_source_core_id_t_core_type_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_shader_r_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_r_t_major_9,
						NELEMS(lut_fault_source_shader_r_t_major_9));
		break;
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_r_t_major_10,
						NELEMS(lut_fault_source_shader_r_t_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_r_t_major_11,
						NELEMS(lut_fault_source_shader_r_t_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_r_t_major_12,
						NELEMS(lut_fault_source_shader_r_t_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_shader_w_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 9:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_w_t_major_9,
						NELEMS(lut_fault_source_shader_w_t_major_9));
		break;
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_w_t_major_10,
						NELEMS(lut_fault_source_shader_w_t_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_w_t_major_11,
						NELEMS(lut_fault_source_shader_w_t_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_shader_w_t_major_12,
						NELEMS(lut_fault_source_shader_w_t_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_tiler_r_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_tiler_r_t_major_10,
						NELEMS(lut_fault_source_tiler_r_t_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_tiler_r_t_major_11,
						NELEMS(lut_fault_source_tiler_r_t_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_tiler_r_t_major_12,
						NELEMS(lut_fault_source_tiler_r_t_major_12));
		break;
	}
	return ret;
}

const char *decode_fault_source_tiler_w_t(u16 idx, u32 gpu_id)
{
	u16 min_rev = GPU_ID_ARCH_ID_MINOR_AND_REV_GET(gpu_id);
	const char *ret = "unknown";

	switch (GPU_ID_ARCH_ID_MAJOR_GET(gpu_id)) {
	case 10:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_tiler_w_t_major_10,
						NELEMS(lut_fault_source_tiler_w_t_major_10));
		break;
	case 11:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_tiler_w_t_major_11,
						NELEMS(lut_fault_source_tiler_w_t_major_11));
		break;
	case 12:
		ret = decode_lut_element_lookup(min_rev, idx, lut_fault_source_tiler_w_t_major_12,
						NELEMS(lut_fault_source_tiler_w_t_major_12));
		break;
	}
	return ret;
}
