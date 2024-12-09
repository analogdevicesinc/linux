/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
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

#ifndef _KBASE_IOCTL_HELPERS_H_
#define _KBASE_IOCTL_HELPERS_H_

#include <uapi/gpu/arm/midgard/mali_kbase_ioctl.h>

/* Macro for IOCTLs that don't have IOCTL struct */
#define KBASE_HANDLE_IOCTL(cmd, function, arg)                                         \
	do {                                                                           \
		int ret;                                                               \
		BUILD_BUG_ON(_IOC_DIR(cmd) != _IOC_NONE);                              \
		dev_dbg(arg->kbdev->dev, "Enter ioctl %s\n", #function);               \
		ret = function(arg);                                                   \
		dev_dbg(arg->kbdev->dev, "Return %d from ioctl %s\n", ret, #function); \
		return ret;                                                            \
	} while (0)

/* Macro for IOCTLs that have input IOCTL struct */
#define KBASE_HANDLE_IOCTL_IN(cmd, function, type, arg)                                \
	do {                                                                           \
		type param = {0};                                                            \
		int ret, err;                                                          \
		dev_dbg(arg->kbdev->dev, "Enter ioctl %s\n", #function);               \
		BUILD_BUG_ON(_IOC_DIR(cmd) != _IOC_WRITE);                             \
		BUILD_BUG_ON(sizeof(param) != _IOC_SIZE(cmd));                         \
		err = copy_from_user(&param, uarg, sizeof(param));                     \
		if (unlikely(err))                                                     \
			return -EFAULT;                                                \
		err = check_padding_##cmd(&param);                                     \
		if (unlikely(err))                                                     \
			return -EINVAL;                                                \
		ret = function(arg, &param);                                           \
		dev_dbg(arg->kbdev->dev, "Return %d from ioctl %s\n", ret, #function); \
		return ret;                                                            \
	} while (0)

/* Macro for IOCTLs that have output IOCTL struct */
#define KBASE_HANDLE_IOCTL_OUT(cmd, function, type, arg)                               \
	do {                                                                           \
		type param;                                                            \
		int ret, err;                                                          \
		dev_dbg(arg->kbdev->dev, "Enter ioctl %s\n", #function);               \
		BUILD_BUG_ON(_IOC_DIR(cmd) != _IOC_READ);                              \
		BUILD_BUG_ON(sizeof(param) != _IOC_SIZE(cmd));                         \
		memset(&param, 0, sizeof(param));                                      \
		ret = function(arg, &param);                                           \
		err = copy_to_user(uarg, &param, sizeof(param));                       \
		if (unlikely(err))                                                     \
			return -EFAULT;                                                \
		dev_dbg(arg->kbdev->dev, "Return %d from ioctl %s\n", ret, #function); \
		return ret;                                                            \
	} while (0)

/* Macro for IOCTLs that have input and output IOCTL struct */
#define KBASE_HANDLE_IOCTL_INOUT(cmd, function, type, arg)                             \
	do {                                                                           \
		type param = {0};                                                            \
		int ret, err;                                                          \
		dev_dbg(arg->kbdev->dev, "Enter ioctl %s\n", #function);               \
		BUILD_BUG_ON(_IOC_DIR(cmd) != (_IOC_WRITE | _IOC_READ));               \
		BUILD_BUG_ON(sizeof(param) != _IOC_SIZE(cmd));                         \
		err = copy_from_user(&param, uarg, sizeof(param));                     \
		if (unlikely(err))                                                     \
			return -EFAULT;                                                \
		err = check_padding_##cmd(&param);                                     \
		if (unlikely(err))                                                     \
			return -EINVAL;                                                \
		ret = function(arg, &param);                                           \
		err = copy_to_user(uarg, &param, sizeof(param));                       \
		if (unlikely(err))                                                     \
			return -EFAULT;                                                \
		dev_dbg(arg->kbdev->dev, "Return %d from ioctl %s\n", ret, #function); \
		return ret;                                                            \
	} while (0)

/* Inline functions to check padding bytes in the input IOCTL struct.
 * Return 0 if all padding bytes are zero, non-zero otherwise.
 */
static inline int check_padding_KBASE_IOCTL_VERSION_CHECK(struct kbase_ioctl_version_check *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_VERSION_CHECK_RESERVED(struct kbase_ioctl_version_check *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_SET_FLAGS(struct kbase_ioctl_set_flags *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_GET_GPUPROPS(struct kbase_ioctl_get_gpuprops *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_ALLOC(union kbase_ioctl_mem_alloc *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_QUERY(union kbase_ioctl_mem_query *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_FREE(struct kbase_ioctl_mem_free *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_HWCNT_READER_SETUP(struct kbase_ioctl_hwcnt_reader_setup *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_HWCNT_SET(struct kbase_ioctl_hwcnt_values *p)
{
	return p->padding;
}

static inline int check_padding_KBASE_IOCTL_GET_DDK_VERSION(struct kbase_ioctl_get_ddk_version *p)
{
	return p->padding;
}

static inline int check_padding_KBASE_IOCTL_MEM_JIT_INIT(struct kbase_ioctl_mem_jit_init *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->padding); i++) {
		if (p->padding[i])
			return -1;
	}

	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_SYNC(struct kbase_ioctl_mem_sync *p)
{
	size_t i;

	/*
	 * Checking p->padding is deferred till the support window for backward-compatibility ends.
	 * GPUCORE-42000 will add the checking.
	 *
	 * To avoid the situation with old version of base which might not set padding bytes as 0,
	 * padding bytes are set as zero here on behalf on user space.
	 */
	for (i = 0; i < ARRAY_SIZE(p->padding); i++)
		p->padding[i] = 0;

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_MEM_FIND_CPU_OFFSET(union kbase_ioctl_mem_find_cpu_offset *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_TLSTREAM_ACQUIRE(struct kbase_ioctl_tlstream_acquire *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_COMMIT(struct kbase_ioctl_mem_commit *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_ALIAS(union kbase_ioctl_mem_alias *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_IMPORT(union kbase_ioctl_mem_import *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_FLAGS_CHANGE(struct kbase_ioctl_mem_flags_change *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_STREAM_CREATE(struct kbase_ioctl_stream_create *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_FENCE_VALIDATE(struct kbase_ioctl_fence_validate *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_PROFILE_ADD(struct kbase_ioctl_mem_profile_add *p)
{
	return p->padding;
}

static inline int
check_padding_KBASE_IOCTL_STICKY_RESOURCE_MAP(struct kbase_ioctl_sticky_resource_map *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_STICKY_RESOURCE_UNMAP(struct kbase_ioctl_sticky_resource_unmap *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_FIND_GPU_START_AND_OFFSET(
	union kbase_ioctl_mem_find_gpu_start_and_offset *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_EXEC_INIT(struct kbase_ioctl_mem_exec_init *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_GET_CPU_GPU_TIMEINFO(union kbase_ioctl_get_cpu_gpu_timeinfo *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.paddings); i++) {
		if (p->in.paddings[i])
			return -1;
	}

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CONTEXT_PRIORITY_CHECK(struct kbase_ioctl_context_priority_check *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_SET_LIMITED_CORE_COUNT(struct kbase_ioctl_set_limited_core_count *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_KINSTR_PRFCNT_ENUM_INFO(struct kbase_ioctl_kinstr_prfcnt_enum_info *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_KINSTR_PRFCNT_SETUP(union kbase_ioctl_kinstr_prfcnt_setup *p)
{
	return 0;
}

#if MALI_UNIT_TEST
#endif /* MALI_UNIT_TEST */

#if MALI_USE_CSF

static inline int
check_padding_KBASE_IOCTL_CS_QUEUE_REGISTER(struct kbase_ioctl_cs_queue_register *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->padding); i++) {
		if (p->padding[i])
			return -1;
	}

	return 0;
}

static inline int check_padding_KBASE_IOCTL_CS_QUEUE_KICK(struct kbase_ioctl_cs_queue_kick *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_CS_QUEUE_BIND(union kbase_ioctl_cs_queue_bind *p)
{
	size_t i;

	/*
	 * Checking p->padding is deferred till the support window for backward-compatibility ends.
	 * GPUCORE-42000 will add the checking.
	 *
	 * To avoid the situation with old version of base which might not set padding bytes as 0,
	 * padding bytes are set as zero here on behalf on user space.
	 */
	for (i = 0; i < ARRAY_SIZE(p->in.padding); i++)
		p->in.padding[i] = 0;

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_QUEUE_REGISTER_EX(struct kbase_ioctl_cs_queue_register_ex *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->padding); i++) {
		if (p->padding[i])
			return -1;
	}

	for (i = 0; i < ARRAY_SIZE(p->ex_padding); i++) {
		if (p->ex_padding[i])
			return -1;
	}

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_QUEUE_TERMINATE(struct kbase_ioctl_cs_queue_terminate *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_6(union kbase_ioctl_cs_queue_group_create_1_6 *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.padding); i++) {
		if (p->in.padding[i])
			return -1;
	}

	return 0;
}

static inline int check_padding_KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_18(
	union kbase_ioctl_cs_queue_group_create_1_18 *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.padding); i++) {
		if (p->in.padding[i])
			return -1;
	}

	return 0;
}

static inline int check_padding_KBASE_IOCTL_CS_QUEUE_GROUP_CREATE_1_35(
	union kbase_ioctl_cs_queue_group_create_1_35 *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.padding); i++) {
		if (p->in.padding[i])
			return -1;
	}

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_QUEUE_GROUP_CREATE(union kbase_ioctl_cs_queue_group_create *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.padding); i++) {
		if (p->in.padding[i])
			return -1;
	}

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_QUEUE_GROUP_TERMINATE(struct kbase_ioctl_cs_queue_group_term *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->padding); i++) {
		if (p->padding[i])
			return -1;
	}

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_KCPU_QUEUE_DELETE(struct kbase_ioctl_kcpu_queue_delete *p)
{
	size_t i;

	/*
	 * Checking p->padding is deferred till the support window for backward-compatibility ends.
	 * GPUCORE-42000 will add the checking.
	 *
	 * To avoid the situation with old version of base which might not set padding bytes as 0,
	 * padding bytes are set as zero here on behalf on user space.
	 */
	for (i = 0; i < ARRAY_SIZE(p->padding); i++)
		p->padding[i] = 0;

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_KCPU_QUEUE_ENQUEUE(struct kbase_ioctl_kcpu_queue_enqueue *p)
{
	size_t i;

	/*
	 * Checking p->padding is deferred till the support window for backward-compatibility ends.
	 * GPUCORE-42000 will add the checking.
	 *
	 * To avoid the situation with old version of base which might not set padding bytes as 0,
	 * padding bytes are set as zero here on behalf on user space.
	 */
	for (i = 0; i < ARRAY_SIZE(p->padding); i++)
		p->padding[i] = 0;

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_TILER_HEAP_INIT(union kbase_ioctl_cs_tiler_heap_init *p)
{
	return p->in.padding;
}

static inline int
check_padding_KBASE_IOCTL_CS_TILER_HEAP_INIT_1_13(union kbase_ioctl_cs_tiler_heap_init_1_13 *p)
{
	return p->in.padding;
}

static inline int
check_padding_KBASE_IOCTL_CS_TILER_HEAP_TERM(struct kbase_ioctl_cs_tiler_heap_term *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_CS_GET_GLB_IFACE(union kbase_ioctl_cs_get_glb_iface *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_CPU_QUEUE_DUMP(struct kbase_ioctl_cs_cpu_queue_info *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_MEM_ALLOC_EX(union kbase_ioctl_mem_alloc_ex *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.extra); i++) {
		if (p->in.extra[i])
			return -1;
	}

	return 0;
}

static inline int check_padding_KBASE_IOCTL_READ_USER_PAGE(union kbase_ioctl_read_user_page *p)
{
	return p->in.padding;
}

static inline int
check_padding_KBASE_IOCTL_QUEUE_GROUP_CLEAR_FAULTS(struct kbase_ioctl_queue_group_clear_faults *p)
{
	size_t i;

	/*
	 * Checking p->padding is deferred till the support window for backward-compatibility ends.
	 * GPUCORE-42000 will add the checking.
	 *
	 * To avoid the situation with old version of base which might not set padding bytes as 0,
	 * padding bytes are set as zero here on behalf on user space.
	 */
	for (i = 0; i < ARRAY_SIZE(p->padding); i++)
		p->padding[i] = 0;

	return 0;
}

static inline int
check_padding_KBASE_IOCTL_CS_TILER_HEAP_SIZE(union kbase_ioctl_cs_tiler_heap_size *p)
{
	return 0;
}

#else /* MALI_USE_CSF */

static inline int check_padding_KBASE_IOCTL_JOB_SUBMIT(struct kbase_ioctl_job_submit *p)
{
	return 0;
}

static inline int
check_padding_KBASE_IOCTL_SOFT_EVENT_UPDATE(struct kbase_ioctl_soft_event_update *p)
{
	return 0;
}

static inline int check_padding_KBASE_IOCTL_KINSTR_JM_FD(union kbase_kinstr_jm_fd *p)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(p->in.padding); i++) {
		if (p->in.padding[i])
			return -1;
	}

	return 0;
}

#endif /* !MALI_USE_CSF */

#endif /* _KBASE_IOCTL_HELPERS_H_ */
