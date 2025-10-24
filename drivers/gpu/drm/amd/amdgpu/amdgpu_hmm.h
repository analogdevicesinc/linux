/*
 * Copyright 2017 Advanced Micro Devices, Inc.
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
 * Authors: Christian König
 */
#ifndef __AMDGPU_MN_H__
#define __AMDGPU_MN_H__

#include <linux/types.h>
#include <linux/hmm.h>
#include <linux/rwsem.h>
#include <linux/workqueue.h>
#include <linux/interval_tree.h>
#include <linux/mmu_notifier.h>

struct amdgpu_hmm_range {
	struct hmm_range hmm_range;
	struct amdgpu_bo *bo;
};

int amdgpu_hmm_range_get_pages(struct mmu_interval_notifier *notifier,
			       uint64_t start, uint64_t npages, bool readonly,
			       void *owner,
			       struct amdgpu_hmm_range *range);

#if defined(CONFIG_HMM_MIRROR)
bool amdgpu_hmm_range_valid(struct amdgpu_hmm_range *range);
struct amdgpu_hmm_range *amdgpu_hmm_range_alloc(struct amdgpu_bo *bo);
void amdgpu_hmm_range_free(struct amdgpu_hmm_range *range);
int amdgpu_hmm_register(struct amdgpu_bo *bo, unsigned long addr);
void amdgpu_hmm_unregister(struct amdgpu_bo *bo);
#else
static inline int amdgpu_hmm_register(struct amdgpu_bo *bo, unsigned long addr)
{
	DRM_WARN_ONCE("HMM_MIRROR kernel config option is not enabled, "
		      "add CONFIG_ZONE_DEVICE=y in config file to fix this\n");
	return -ENODEV;
}

static inline void amdgpu_hmm_unregister(struct amdgpu_bo *bo) {}

static inline bool amdgpu_hmm_range_valid(struct amdgpu_hmm_range *range)
{
	return false;
}

static inline struct amdgpu_hmm_range *amdgpu_hmm_range_alloc(struct amdgpu_bo *bo)
{
	return NULL;
}

static inline void amdgpu_hmm_range_free(struct amdgpu_hmm_range *range) {}
#endif

#endif
