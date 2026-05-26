/*
 * Copyright 2016 Advanced Micro Devices, Inc.
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
#ifndef __AMDGPU_VM_INTERNAL_H__
#define __AMDGPU_VM_INTERNAL_H__

#include <linux/types.h>
#include <linux/list.h>
#include "amdgpu_vm.h"

struct amdgpu_device;
struct amdgpu_vm;
struct amdgpu_job;
struct amdgpu_sync;
struct amdgpu_bo_vm;
struct amdgpu_vm_bo_base;
struct dma_fence;

/**
 * struct amdgpu_vm_update_params
 *
 * Encapsulate some VM table update parameters to reduce
 * the number of function parameters
 *
 */
struct amdgpu_vm_update_params {

	/**
	 * @adev: amdgpu device we do this update for
	 */
	struct amdgpu_device *adev;

	/**
	 * @vm: optional amdgpu_vm we do this update for
	 */
	struct amdgpu_vm *vm;

	/**
	 * @immediate: if changes should be made immediately
	 */
	bool immediate;

	/**
	 * @unlocked: true if the root BO is not locked
	 */
	bool unlocked;

	/**
	 * @pages_addr:
	 *
	 * DMA addresses to use for mapping
	 */
	dma_addr_t *pages_addr;

	/**
	 * @job: job to used for hw submission
	 */
	struct amdgpu_job *job;

	/**
	 * @num_dw_left: number of dw left for the IB
	 */
	unsigned int num_dw_left;

	/**
	 * @needs_flush: true whenever we need to invalidate the TLB
	 */
	bool needs_flush;

	/**
	 * @override_pte: true for memory that is not uncached and gmc override function is
	 * implemented to allow MTYPE to be overridden for NUMA local memory.
	 */
	bool override_pte;

	/**
	 * @tlb_flush_waitlist: temporary storage for BOs until tlb_flush
	 */
	struct list_head tlb_flush_waitlist;
};

struct amdgpu_vm_update_funcs {
	int (*map_table)(struct amdgpu_bo_vm *bo);
	int (*prepare)(struct amdgpu_vm_update_params *p,
		       struct amdgpu_sync *sync, u64 k_job_id);
	int (*update)(struct amdgpu_vm_update_params *p,
		      struct amdgpu_bo_vm *bo, uint64_t pe, uint64_t addr,
		      unsigned count, uint32_t incr, uint64_t flags);
	int (*commit)(struct amdgpu_vm_update_params *p,
		      struct dma_fence **fence);
};

int amdgpu_vm_pt_clear(struct amdgpu_device *adev, struct amdgpu_vm *vm,
		       struct amdgpu_bo_vm *vmbo, bool immediate);
int amdgpu_vm_pt_create(struct amdgpu_device *adev, struct amdgpu_vm *vm,
			int level, bool immediate, struct amdgpu_bo_vm **vmbo,
			int32_t xcp_id);
void amdgpu_vm_pt_free_root(struct amdgpu_device *adev, struct amdgpu_vm *vm);
int amdgpu_vm_pde_update(struct amdgpu_vm_update_params *params,
			 struct amdgpu_vm_bo_base *entry);
int amdgpu_vm_ptes_update(struct amdgpu_vm_update_params *params,
			  uint64_t start, uint64_t end,
			  uint64_t dst, uint64_t flags);
void amdgpu_vm_pt_free_work(struct work_struct *work);
void amdgpu_vm_pt_free_list(struct amdgpu_device *adev,
			    struct amdgpu_vm_update_params *params);
int amdgpu_vm_pt_map_tables(struct amdgpu_device *adev, struct amdgpu_vm *vm);

/*
 * vm eviction_lock can be taken in MMU notifiers. Make sure no reclaim-FS
 * happens while holding this lock anywhere to prevent deadlocks when
 * an MMU notifier runs in reclaim-FS context.
 */
static inline void amdgpu_vm_eviction_lock(struct amdgpu_vm *vm)
{
	mutex_lock(&vm->eviction_lock);
	vm->saved_flags = memalloc_noreclaim_save();
}

static inline bool amdgpu_vm_eviction_trylock(struct amdgpu_vm *vm)
{
	if (mutex_trylock(&vm->eviction_lock)) {
		vm->saved_flags = memalloc_noreclaim_save();
		return true;
	}
	return false;
}

static inline void amdgpu_vm_eviction_unlock(struct amdgpu_vm *vm)
{
	memalloc_noreclaim_restore(vm->saved_flags);
	mutex_unlock(&vm->eviction_lock);
}

#endif
