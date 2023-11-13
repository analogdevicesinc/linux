/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2015-2023 ARM Limited. All rights reserved.
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

#ifndef _UAPI_KBASE_GPU_ID_H_
#define _UAPI_KBASE_GPU_ID_H_

#if defined(__linux)
#include <linux/types.h>
#endif

#define GPU_ID2_VERSION_STATUS_SHIFT 0
#define GPU_ID2_VERSION_MINOR_SHIFT 4
#define GPU_ID2_VERSION_MAJOR_SHIFT 12
#define GPU_ID2_PRODUCT_MAJOR_SHIFT 16
#define GPU_ID2_ARCH_REV_SHIFT 20
#define GPU_ID2_ARCH_MINOR_SHIFT 24
#define GPU_ID2_ARCH_MAJOR_SHIFT 28
#define GPU_ID2_VERSION_STATUS (0xFu << GPU_ID2_VERSION_STATUS_SHIFT)
#define GPU_ID2_VERSION_MINOR (0xFFu << GPU_ID2_VERSION_MINOR_SHIFT)
#define GPU_ID2_VERSION_MAJOR (0xFu << GPU_ID2_VERSION_MAJOR_SHIFT)
#define GPU_ID2_PRODUCT_MAJOR (0xFu << GPU_ID2_PRODUCT_MAJOR_SHIFT)
#define GPU_ID2_ARCH_REV (0xFu << GPU_ID2_ARCH_REV_SHIFT)
#define GPU_ID2_ARCH_MINOR (0xFu << GPU_ID2_ARCH_MINOR_SHIFT)
#define GPU_ID2_ARCH_MAJOR (0xFu << GPU_ID2_ARCH_MAJOR_SHIFT)
#define GPU_ID2_PRODUCT_MODEL (GPU_ID2_ARCH_MAJOR | GPU_ID2_PRODUCT_MAJOR)
#define GPU_ID2_VERSION (GPU_ID2_VERSION_MAJOR | GPU_ID2_VERSION_MINOR | GPU_ID2_VERSION_STATUS)

#define GPU_ID2_ARCH_REV_GET(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_ARCH_REV) >> GPU_ID2_ARCH_REV_SHIFT)
#define GPU_ID2_ARCH_MINOR_GET(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_ARCH_MINOR) >> GPU_ID2_ARCH_MINOR_SHIFT)
#define GPU_ID2_ARCH_MAJOR_GET(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_ARCH_MAJOR) >> GPU_ID2_ARCH_MAJOR_SHIFT)
#define GPU_ID2_VERSION_MINOR_GET(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_VERSION_MINOR) >> GPU_ID2_VERSION_MINOR_SHIFT)
#define GPU_ID2_VERSION_MAJOR_GET(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_VERSION_MAJOR) >> GPU_ID2_VERSION_MAJOR_SHIFT)
#define GPU_ID2_PRODUCT_MAJOR_GET(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_PRODUCT_MAJOR) >> GPU_ID2_PRODUCT_MAJOR_SHIFT)
/* Helper macro to construct a value consisting of arch major and revision
 * using the value of gpu_id.
 */
#define GPU_ID2_ARCH_MAJOR_REV_REG(gpu_id) \
	((((__u32)gpu_id) & GPU_ID2_ARCH_MAJOR) | (((__u32)gpu_id) & GPU_ID2_ARCH_REV))

/* Helper macro to create a partial GPU_ID (new format) that defines
 * a arch major and revision.
 */
#define GPU_ID2_ARCH_MAJOR_REV_MAKE(arch_major, arch_rev)    \
	((((__u32)arch_major) << GPU_ID2_ARCH_MAJOR_SHIFT) | \
	 (((__u32)arch_rev) << GPU_ID2_ARCH_REV_SHIFT))

/* Helper macro to create a partial GPU_ID (new format) that defines
 * a product ignoring its version.
 */
#define GPU_ID2_PRODUCT_MAKE(arch_major, arch_minor, arch_rev, product_major) \
	((((__u32)arch_major) << GPU_ID2_ARCH_MAJOR_SHIFT) |                  \
	 (((__u32)arch_minor) << GPU_ID2_ARCH_MINOR_SHIFT) |                  \
	 (((__u32)arch_rev) << GPU_ID2_ARCH_REV_SHIFT) |                      \
	 (((__u32)product_major) << GPU_ID2_PRODUCT_MAJOR_SHIFT))

/* Helper macro to create a partial GPU_ID (new format) that specifies the
 * revision (major, minor, status) of a product
 */
#define GPU_ID2_VERSION_MAKE(version_major, version_minor, version_status) \
	((((__u32)version_major) << GPU_ID2_VERSION_MAJOR_SHIFT) |         \
	 (((__u32)version_minor) << GPU_ID2_VERSION_MINOR_SHIFT) |         \
	 (((__u32)version_status) << GPU_ID2_VERSION_STATUS_SHIFT))

/* Helper macro to create a complete GPU_ID (new format) */
#define GPU_ID2_MAKE(arch_major, arch_minor, arch_rev, product_major, version_major, \
		     version_minor, version_status)                                  \
	(GPU_ID2_PRODUCT_MAKE(arch_major, arch_minor, arch_rev, product_major) |     \
	 GPU_ID2_VERSION_MAKE(version_major, version_minor, version_status))

/* Helper macro to create a partial GPU_ID (new format) that identifies
 * a particular GPU model by its arch_major and product_major.
 */
#define GPU_ID2_MODEL_MAKE(arch_major, product_major)        \
	((((__u32)arch_major) << GPU_ID2_ARCH_MAJOR_SHIFT) | \
	 (((__u32)product_major) << GPU_ID2_PRODUCT_MAJOR_SHIFT))

/* Strip off the non-relevant bits from a product_id value and make it suitable
 * for comparison against the GPU_ID2_PRODUCT_xxx values which identify a GPU
 * model.
 */
#define GPU_ID2_MODEL_MATCH_VALUE(product_id) \
	((((__u32)product_id) << GPU_ID2_PRODUCT_MAJOR_SHIFT) & GPU_ID2_PRODUCT_MODEL)

#define GPU_ID2_PRODUCT_TMIX GPU_ID2_MODEL_MAKE(6, 0)
#define GPU_ID2_PRODUCT_THEX GPU_ID2_MODEL_MAKE(6, 1)
#define GPU_ID2_PRODUCT_TSIX GPU_ID2_MODEL_MAKE(7, 0)
#define GPU_ID2_PRODUCT_TDVX GPU_ID2_MODEL_MAKE(7, 3)
#define GPU_ID2_PRODUCT_TNOX GPU_ID2_MODEL_MAKE(7, 1)
#define GPU_ID2_PRODUCT_TGOX GPU_ID2_MODEL_MAKE(7, 2)
#define GPU_ID2_PRODUCT_TTRX GPU_ID2_MODEL_MAKE(9, 0)
#define GPU_ID2_PRODUCT_TNAX GPU_ID2_MODEL_MAKE(9, 1)
#define GPU_ID2_PRODUCT_TBEX GPU_ID2_MODEL_MAKE(9, 2)
#define GPU_ID2_PRODUCT_LBEX GPU_ID2_MODEL_MAKE(9, 4)
#define GPU_ID2_PRODUCT_TBAX GPU_ID2_MODEL_MAKE(9, 5)
#define GPU_ID2_PRODUCT_TODX GPU_ID2_MODEL_MAKE(10, 2)
#define GPU_ID2_PRODUCT_TGRX GPU_ID2_MODEL_MAKE(10, 3)
#define GPU_ID2_PRODUCT_TVAX GPU_ID2_MODEL_MAKE(10, 4)
#define GPU_ID2_PRODUCT_LODX GPU_ID2_MODEL_MAKE(10, 7)
#define GPU_ID2_PRODUCT_TTUX GPU_ID2_MODEL_MAKE(11, 2)
#define GPU_ID2_PRODUCT_LTUX GPU_ID2_MODEL_MAKE(11, 3)
#define GPU_ID2_PRODUCT_TTIX GPU_ID2_MODEL_MAKE(12, 0)
#define GPU_ID2_PRODUCT_LTIX GPU_ID2_MODEL_MAKE(12, 1)
#define GPU_ID2_PRODUCT_TKRX GPU_ID2_MODEL_MAKE(13, 0)
#define GPU_ID2_PRODUCT_LKRX GPU_ID2_MODEL_MAKE(13, 1)



#define GPU_ID_U8_COMP(val3, val2, val1, val0) \
	((((__u32)val3) << 24U) | (((__u32)val2) << 16U) | (((__u32)val1) << 8U) | ((__u32)val0))
#define GPU_ID_U8_COMP_SHIFT(comp, idx) (((__u32)comp) >> (idx * 8U))
#define GPU_ID_U8_COMP_GET(comp, idx) (GPU_ID_U8_COMP_SHIFT(comp, idx) & 0xFF)

#define GPU_ID_PRODUCT_ID_MAKE(arch_major, arch_minor, arch_rev, product_major) \
	GPU_ID_U8_COMP(arch_major, arch_minor, arch_rev, product_major)
#define GPU_ID_MODEL_MAKE(arch_major, product_major) GPU_ID_U8_COMP(arch_major, 0, 0, product_major)
#define GPU_ID_VERSION_MAKE(version_major, version_minor, version_status) \
	GPU_ID_U8_COMP(0, version_major, version_minor, version_status)
#define GPU_ID_ARCH_MAKE(arch_major, arch_minor, arch_rev) \
	GPU_ID_U8_COMP(0, arch_major, arch_minor, arch_rev)

/* Convert ID created from GPU_ID_PRODUCT_ID_MAKE() to match the format of
 * GPU_ID_MODEL_MAKE()
 */
#define GPU_ID_MODEL_MATCH_VALUE(product_id) (((__u32)product_id) & GPU_ID_MODEL_MAKE(0xFF, 0xFF))

#define GPU_ID_VERSION_ID_MAJOR_MINOR_GET(version_id) GPU_ID_U8_COMP_SHIFT(version_id, 1)
#define GPU_ID_VERSION_ID_STATUS_GET(version_id) GPU_ID_U8_COMP_GET(version_id, 0)
#define GPU_ID_VERSION_ID_MINOR_GET(version_id) GPU_ID_U8_COMP_GET(version_id, 1)
#define GPU_ID_VERSION_ID_MAJOR_GET(version_id) GPU_ID_U8_COMP_GET(version_id, 2)

#define GPU_ID_PRODUCT_TMIX GPU_ID_MODEL_MAKE(6, 0)
#define GPU_ID_PRODUCT_THEX GPU_ID_MODEL_MAKE(6, 1)
#define GPU_ID_PRODUCT_TSIX GPU_ID_MODEL_MAKE(7, 0)
#define GPU_ID_PRODUCT_TDVX GPU_ID_MODEL_MAKE(7, 3)
#define GPU_ID_PRODUCT_TNOX GPU_ID_MODEL_MAKE(7, 1)
#define GPU_ID_PRODUCT_TGOX GPU_ID_MODEL_MAKE(7, 2)
#define GPU_ID_PRODUCT_TTRX GPU_ID_MODEL_MAKE(9, 0)
#define GPU_ID_PRODUCT_TNAX GPU_ID_MODEL_MAKE(9, 1)
#define GPU_ID_PRODUCT_TBEX GPU_ID_MODEL_MAKE(9, 2)
#define GPU_ID_PRODUCT_LBEX GPU_ID_MODEL_MAKE(9, 4)
#define GPU_ID_PRODUCT_TBAX GPU_ID_MODEL_MAKE(9, 5)
#define GPU_ID_PRODUCT_TODX GPU_ID_MODEL_MAKE(10, 2)
#define GPU_ID_PRODUCT_TGRX GPU_ID_MODEL_MAKE(10, 3)
#define GPU_ID_PRODUCT_TVAX GPU_ID_MODEL_MAKE(10, 4)
#define GPU_ID_PRODUCT_LODX GPU_ID_MODEL_MAKE(10, 7)
#define GPU_ID_PRODUCT_TTUX GPU_ID_MODEL_MAKE(11, 2)
#define GPU_ID_PRODUCT_LTUX GPU_ID_MODEL_MAKE(11, 3)
#define GPU_ID_PRODUCT_TTIX GPU_ID_MODEL_MAKE(12, 0)
#define GPU_ID_PRODUCT_LTIX GPU_ID_MODEL_MAKE(12, 1)
#define GPU_ID_PRODUCT_TKRX GPU_ID_MODEL_MAKE(13, 0)
#define GPU_ID_PRODUCT_LKRX GPU_ID_MODEL_MAKE(13, 1)

#endif /* _UAPI_KBASE_GPU_ID_H_ */
