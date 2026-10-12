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
 */
#include "amdgpu_ids.h"

#include <linux/xarray.h>
#include <linux/dma-fence-array.h>


#include "amdgpu.h"
#include "amdgpu_trace.h"

/*
 * PASID manager
 *
 * PASIDs are global address space identifiers that can be shared
 * between the GPU, an IOMMU and the driver. VMs on different devices
 * may use the same PASID if they share the same address
 * space. Therefore PASIDs are allocated using IDR cyclic allocator
 * (similar to kernel PID allocation) which naturally delays reuse.
 * VMs are looked up from the PASID per amdgpu_device.
 */

static DEFINE_XARRAY_FLAGS(amdgpu_pasid_xa, XA_FLAGS_LOCK_IRQ | XA_FLAGS_ALLOC1);
static u32 amdgpu_pasid_xa_next;

/* Helper to free pasid from a fence callback */
struct amdgpu_pasid_cb {
	struct dma_fence_cb cb;
	u32 pasid;
};

/**
 * amdgpu_pasid_alloc - Allocate a PASID
 * @bits: Number of PASID bits supported by the hardware
 * @fpriv: DRM file private to associate with the PASID
 *
 * Uses the kernel's XArray cyclic allocator.
 * Allocates sequentially with automatic wrap-around.
 *
 * Returns:
 * Allocated PASID on success or a negative error code on failure.
 */
int amdgpu_pasid_alloc(unsigned int bits, struct amdgpu_fpriv *fpriv)
{
	u32 pasid;
	int r;

	if (bits == 0)
		return -EINVAL;

	r = xa_alloc_cyclic_irq(&amdgpu_pasid_xa, &pasid, fpriv,
				XA_LIMIT(1, (1U << bits) - 1),
				&amdgpu_pasid_xa_next, GFP_KERNEL);
	if (r < 0)
		return r;

	trace_amdgpu_pasid_allocated(pasid);
	return pasid;
}

/**
 * amdgpu_pasid_free - Free a PASID
 * @pasid: PASID to free
 *
 * Called in IRQ context.
 */
void amdgpu_pasid_free(u32 pasid)
{
	unsigned long flags;

	trace_amdgpu_pasid_freed(pasid);

	xa_lock_irqsave(&amdgpu_pasid_xa, flags);
	__xa_erase(&amdgpu_pasid_xa, pasid);
	xa_unlock_irqrestore(&amdgpu_pasid_xa, flags);
}

static void amdgpu_pasid_free_cb(struct dma_fence *fence,
				 struct dma_fence_cb *_cb)
{
	struct amdgpu_pasid_cb *cb =
		container_of(_cb, struct amdgpu_pasid_cb, cb);

	amdgpu_pasid_free(cb->pasid);
	dma_fence_put(fence);
	kfree(cb);
}

/**
 * amdgpu_pasid_clear_owner - Clear the owner associated with a PASID
 * @pasid: PASID whose owner should be cleared
 *
 * Replace the stored owner with NULL while keeping the PASID allocated.
 *
 * This is used by the delayed PASID free path so that future PASID
 * lookups cannot resolve a stale DRM file-private object while the PASID
 * is still waiting for outstanding fences before being released.
 */
static void amdgpu_pasid_clear_owner(u32 pasid)
{
	unsigned long flags;

	if (!pasid)
		return;

	xa_lock_irqsave(&amdgpu_pasid_xa, flags);
	__xa_store(&amdgpu_pasid_xa, pasid, NULL, GFP_ATOMIC);
	xa_unlock_irqrestore(&amdgpu_pasid_xa, flags);
}

/**
 * amdgpu_pasid_lock - acquire the global PASID xarray lock
 * @flags: storage for interrupt state
 *
 * Acquire the global PASID xarray lock with interrupts disabled.
 * The saved interrupt state must be passed to
 * amdgpu_pasid_unlock().
 */
void amdgpu_pasid_lock(unsigned long *flags)
{
	xa_lock_irqsave(&amdgpu_pasid_xa, *flags);
}

/**
 * amdgpu_pasid_unlock - release the global PASID xarray lock
 * @flags: interrupt state returned by amdgpu_pasid_lock()
 *
 * Release the global PASID xarray lock and restore the previous
 * interrupt state.
 */
void amdgpu_pasid_unlock(unsigned long flags)
{
	xa_unlock_irqrestore(&amdgpu_pasid_xa, flags);
}

/**
 * amdgpu_pasid_get_fpriv_locked - get the DRM owner of a PASID
 * @pasid: PASID to resolve
 *
 * The caller must hold the PASID XArray lock.
 *
 * Returns:
 * Pointer to the owning DRM file private, or %NULL if the PASID has no
 * current owner.
 *
 * The returned pointer is only valid while the PASID lock remains held
 * and must not be retained after calling amdgpu_pasid_unlock().
 */
struct amdgpu_fpriv *amdgpu_pasid_get_fpriv_locked(u32 pasid)
{
	return xa_load(&amdgpu_pasid_xa, pasid);
}

/**
 * amdgpu_pasid_drain_irq - wait for pending KFD interrupts of a PASID
 * @adev: amdgpu device the drain fence is sent to
 * @pasid: PASID whose interrupts should be drained
 * @ih_fence: IH entry injected as the drain fence
 * @timeout: maximum time to wait, in jiffies
 *
 * Injects @ih_fence behind the interrupts already queued for KFD and waits
 * until amdgpu_pasid_drain_irq_done() is called for @pasid.
 *
 * The caller must hold a reference to the DRM file that owns @pasid, so
 * that its amdgpu_fpriv stays valid after the PASID lock is dropped.
 *
 * Returns:
 * 0 on success or when no fence could be sent, -ENOENT if @pasid has no
 * owner, -ETIMEDOUT on timeout or -ERESTARTSYS if interrupted.
 */
int amdgpu_pasid_drain_irq(struct amdgpu_device *adev, u32 pasid,
			   u32 *ih_fence, unsigned long timeout)
{
	struct amdgpu_fpriv *fpriv;
	unsigned long flags;
	long r;

	amdgpu_pasid_lock(&flags);
	fpriv = amdgpu_pasid_get_fpriv_locked(pasid);
	if (fpriv)
		reinit_completion(&fpriv->irq_drain_done);
	amdgpu_pasid_unlock(flags);
	if (!fpriv)
		return -ENOENT;

	if (amdgpu_amdkfd_send_close_event_drain_irq(adev, ih_fence))
		return 0;

	r = wait_for_completion_interruptible_timeout(&fpriv->irq_drain_done,
						      timeout);
	if (!r)
		return -ETIMEDOUT;

	return r < 0 ? r : 0;
}

/**
 * amdgpu_pasid_drain_irq_done - signal that a KFD interrupt drain completed
 * @pasid: PASID carried by the drain fence
 *
 * Called by KFD when it processes the drain fence injected by
 * amdgpu_pasid_drain_irq().
 */
void amdgpu_pasid_drain_irq_done(u32 pasid)
{
	struct amdgpu_fpriv *fpriv;
	unsigned long flags;

	amdgpu_pasid_lock(&flags);
	fpriv = amdgpu_pasid_get_fpriv_locked(pasid);
	if (fpriv)
		complete(&fpriv->irq_drain_done);
	amdgpu_pasid_unlock(flags);
}

/**
 * amdgpu_pasid_free_delayed - free pasid when fences signal
 *
 * @resv: reservation object with the fences to wait for
 * @pasid: pasid to free
 *
 * Free the pasid only after all the fences in resv are signaled.
 */
void amdgpu_pasid_free_delayed(struct dma_resv *resv,
			       u32 pasid)
{
	struct amdgpu_pasid_cb *cb;
	struct dma_fence *fence;
	int r;

	amdgpu_pasid_clear_owner(pasid);

	r = dma_resv_get_singleton(resv, DMA_RESV_USAGE_BOOKKEEP, &fence);
	if (r)
		goto fallback;

	if (!fence) {
		amdgpu_pasid_free(pasid);
		return;
	}

	cb = kmalloc_obj(*cb);
	if (!cb) {
		/* Last resort when we are OOM */
		dma_fence_wait(fence, false);
		dma_fence_put(fence);
		amdgpu_pasid_free(pasid);
	} else {
		cb->pasid = pasid;
		if (dma_fence_add_callback(fence, &cb->cb,
					   amdgpu_pasid_free_cb))
			amdgpu_pasid_free_cb(fence, &cb->cb);
	}

	return;

fallback:
	/* Not enough memory for the delayed delete, as last resort
	 * block for all the fences to complete.
	 */
	dma_resv_wait_timeout(resv, DMA_RESV_USAGE_BOOKKEEP,
			      false, MAX_SCHEDULE_TIMEOUT);
	amdgpu_pasid_free(pasid);
}

/*
 * VMID manager
 *
 * VMIDs are a per VMHUB identifier for page tables handling.
 */

/**
 * amdgpu_vmid_had_gpu_reset - check if reset occurred since last use
 *
 * @adev: amdgpu_device pointer
 * @id: VMID structure
 *
 * Check if GPU reset occurred since last use of the VMID.
 */
bool amdgpu_vmid_had_gpu_reset(struct amdgpu_device *adev,
			       struct amdgpu_vmid *id)
{
	return id->current_gpu_reset_count !=
		atomic_read(&adev->gpu_reset_counter);
}

/* Check if we need to switch to another set of resources */
static bool amdgpu_vmid_gds_switch_needed(struct amdgpu_vmid *id,
					  struct amdgpu_job *job)
{
	return id->gds_base != job->gds_base ||
		id->gds_size != job->gds_size ||
		id->gws_base != job->gws_base ||
		id->gws_size != job->gws_size ||
		id->oa_base != job->oa_base ||
		id->oa_size != job->oa_size;
}

/* Check if the id is compatible with the job */
static bool amdgpu_vmid_compatible(struct amdgpu_vmid *id,
				   struct amdgpu_job *job)
{
	return  id->pd_gpu_addr == job->vm_pd_addr &&
		!amdgpu_vmid_gds_switch_needed(id, job);
}

/**
 * amdgpu_vmid_grab_idle - grab idle VMID
 *
 * @ring: ring we want to submit job to
 * @idle: resulting idle VMID
 * @fence: fence to wait for if no id could be grabbed
 *
 * Try to find an idle VMID, if none is idle add a fence to wait to the sync
 * object. Returns -ENOMEM when we are out of memory.
 */
static int amdgpu_vmid_grab_idle(struct amdgpu_ring *ring,
				 struct amdgpu_vmid **idle,
				 struct dma_fence **fence)
{
	struct amdgpu_device *adev = ring->adev;
	unsigned vmhub = ring->vm_hub;
	struct amdgpu_kq_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr[vmhub];

	/* If anybody is waiting for a VMID let everybody wait for fairness */
	if (!dma_fence_is_signaled(ring->vmid_wait)) {
		*fence = dma_fence_get(ring->vmid_wait);
		return 0;
	}

	/* Check if we have an idle VMID */
	list_for_each_entry_reverse((*idle), &id_mgr->ids_lru, list) {
		/* Don't use per engine and per process VMID at the same time */
		struct amdgpu_ring *r = adev->vm_manager.concurrent_flush ?
			NULL : ring;

		*fence = amdgpu_sync_peek_fence(&(*idle)->active, r);
		if (!(*fence))
			return 0;
	}

	/*
	 * If we can't find a idle VMID to use, wait on a fence from the least
	 * recently used in the hope that it will be available soon.
	 */
	*idle = NULL;
	dma_fence_put(ring->vmid_wait);
	ring->vmid_wait = dma_fence_get(*fence);

	/* This is the reference we return */
	dma_fence_get(*fence);
	return 0;
}

/**
 * amdgpu_vmid_grab_reserved - try to assign reserved VMID
 *
 * @vm: vm to allocate id for
 * @ring: ring we want to submit job to
 * @job: job who wants to use the VMID
 * @id: resulting VMID
 * @fence: fence to wait for if no id could be grabbed
 *
 * Try to assign a reserved VMID.
 */
static int amdgpu_vmid_grab_reserved(struct amdgpu_vm *vm,
				     struct amdgpu_ring *ring,
				     struct amdgpu_job *job,
				     struct amdgpu_vmid **id,
				     struct dma_fence **fence)
{
	struct amdgpu_device *adev = ring->adev;
	unsigned vmhub = ring->vm_hub;
	uint64_t fence_context = adev->fence_context + ring->idx;
	bool needs_flush = vm->use_cpu_for_update;
	uint64_t updates = amdgpu_vm_tlb_seq(vm);
	int r;

	*id = vm->reserved_vmid[vmhub];
	if ((*id)->owner != vm->immediate.fence_context ||
	    !amdgpu_vmid_compatible(*id, job) ||
	    (*id)->flushed_updates < updates ||
	    !(*id)->last_flush ||
	    ((*id)->last_flush->context != fence_context &&
	     !dma_fence_is_signaled((*id)->last_flush)))
		needs_flush = true;

	if ((*id)->owner != vm->immediate.fence_context ||
	    (!adev->vm_manager.concurrent_flush && needs_flush)) {
		struct dma_fence *tmp;

		/* Don't use per engine and per process VMID at the
		 * same time
		 */
		if (adev->vm_manager.concurrent_flush)
			ring = NULL;

		/* to prevent one context starved by another context */
		(*id)->pd_gpu_addr = 0;
		tmp = amdgpu_sync_peek_fence(&(*id)->active, ring);
		if (tmp) {
			*id = NULL;
			*fence = dma_fence_get(tmp);
			return 0;
		}
	}

	/* Good we can use this VMID. Remember this submission as
	* user of the VMID.
	*/
	r = amdgpu_sync_fence(&(*id)->active, &job->base.s_fence->finished,
			      GFP_ATOMIC);
	if (r)
		return r;

	job->vm_needs_flush = needs_flush;
	return 0;
}

/**
 * amdgpu_vmid_grab_used - try to reuse a VMID
 *
 * @vm: vm to allocate id for
 * @ring: ring we want to submit job to
 * @job: job who wants to use the VMID
 * @id: resulting VMID
 *
 * Try to reuse a VMID for this submission.
 */
static int amdgpu_vmid_grab_used(struct amdgpu_vm *vm,
				 struct amdgpu_ring *ring,
				 struct amdgpu_job *job,
				 struct amdgpu_vmid **id)
{
	struct amdgpu_device *adev = ring->adev;
	unsigned vmhub = ring->vm_hub;
	struct amdgpu_kq_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr[vmhub];
	uint64_t fence_context = adev->fence_context + ring->idx;
	uint64_t updates = amdgpu_vm_tlb_seq(vm);
	int r;

	job->vm_needs_flush = vm->use_cpu_for_update;

	/* Check if we can use a VMID already assigned to this VM */
	list_for_each_entry_reverse((*id), &id_mgr->ids_lru, list) {
		bool needs_flush = vm->use_cpu_for_update;

		/* Check all the prerequisites to using this VMID */
		if ((*id)->owner != vm->immediate.fence_context)
			continue;

		if (!amdgpu_vmid_compatible(*id, job))
			continue;

		if (!(*id)->last_flush ||
		    ((*id)->last_flush->context != fence_context &&
		     !dma_fence_is_signaled((*id)->last_flush)))
			needs_flush = true;

		if ((*id)->flushed_updates < updates)
			needs_flush = true;

		if (needs_flush && !adev->vm_manager.concurrent_flush)
			continue;

		/* Good, we can use this VMID. Remember this submission as
		 * user of the VMID.
		 */
		r = amdgpu_sync_fence(&(*id)->active,
				      &job->base.s_fence->finished,
				      GFP_ATOMIC);
		if (r)
			return r;

		job->vm_needs_flush |= needs_flush;
		return 0;
	}

	*id = NULL;
	return 0;
}

/**
 * amdgpu_vmid_grab - allocate the next free VMID
 *
 * @vm: vm to allocate id for
 * @ring: ring we want to submit job to
 * @job: job who wants to use the VMID
 * @fence: fence to wait for if no id could be grabbed
 *
 * Allocate an id for the vm, adding fences to the sync obj as necessary.
 */
int amdgpu_vmid_grab(struct amdgpu_vm *vm, struct amdgpu_ring *ring,
		     struct amdgpu_job *job, struct dma_fence **fence)
{
	struct amdgpu_device *adev = ring->adev;
	unsigned vmhub = ring->vm_hub;
	struct amdgpu_kq_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr[vmhub];
	struct amdgpu_vmid *idle = NULL;
	struct amdgpu_vmid *id = NULL;
	int r = 0;

	mutex_lock(&id_mgr->lock);
	r = amdgpu_vmid_grab_idle(ring, &idle, fence);
	if (r || !idle)
		goto error;

	if (amdgpu_vmid_uses_reserved(vm, vmhub)) {
		r = amdgpu_vmid_grab_reserved(vm, ring, job, &id, fence);
		if (r || !id)
			goto error;
	} else {
		r = amdgpu_vmid_grab_used(vm, ring, job, &id);
		if (r)
			goto error;

		if (!id) {
			/* Still no ID to use? Then use the idle one found earlier */
			id = idle;

			/* Remember this submission as user of the VMID */
			r = amdgpu_sync_fence(&id->active,
					      &job->base.s_fence->finished,
					      GFP_ATOMIC);
			if (r)
				goto error;

			job->vm_needs_flush = true;
		}

		list_move_tail(&id->list, &id_mgr->ids_lru);
	}

	job->gds_switch_needed = amdgpu_vmid_gds_switch_needed(id, job);
	if (job->vm_needs_flush) {
		id->flushed_updates = amdgpu_vm_tlb_seq(vm);
		dma_fence_put(id->last_flush);
		id->last_flush = NULL;
	}
	job->vmid = id - id_mgr->ids;
	job->pasid = vm->pasid;

	id->gds_base = job->gds_base;
	id->gds_size = job->gds_size;
	id->gws_base = job->gws_base;
	id->gws_size = job->gws_size;
	id->oa_base = job->oa_base;
	id->oa_size = job->oa_size;
	id->pd_gpu_addr = job->vm_pd_addr;
	id->owner = vm->immediate.fence_context;

	trace_amdgpu_vm_grab_id(vm, ring, job);

error:
	mutex_unlock(&id_mgr->lock);
	return r;
}

/*
 * amdgpu_vmid_uses_reserved - check if a VM will use a reserved VMID
 * @vm: the VM to check
 * @vmhub: the VMHUB which will be used
 *
 * Returns: True if the VM will use a reserved VMID.
 */
bool amdgpu_vmid_uses_reserved(struct amdgpu_vm *vm, unsigned int vmhub)
{
	return vm->reserved_vmid[vmhub];
}

/*
 * amdgpu_vmid_alloc_reserved - reserve a specific VMID for this vm
 * @adev: amdgpu device structure
 * @vm: the VM to reserve an ID for
 * @vmhub: the VMHUB which should be used
 *
 * Mostly used to have a reserved VMID for debugging and SPM.
 *
 * Returns: 0 for success, -ENOENT if an ID is already reserved.
 */
int amdgpu_vmid_alloc_reserved(struct amdgpu_device *adev, struct amdgpu_vm *vm,
			       unsigned vmhub)
{
	struct amdgpu_kq_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr[vmhub];
	struct amdgpu_vmid *id;
	int r = 0;

	mutex_lock(&id_mgr->lock);
	if (vm->reserved_vmid[vmhub])
		goto unlock;
	if (id_mgr->reserved_vmid) {
		r = -ENOENT;
		goto unlock;
	}
	if (!adev->vm_manager.spm_vmid) {
		r = -ENOENT;
		goto unlock;
	}

	id = &id_mgr->ids[adev->vm_manager.spm_vmid];
	vm->reserved_vmid[vmhub] = id;
	id_mgr->reserved_vmid = true;
	mutex_unlock(&id_mgr->lock);

	return 0;
unlock:
	mutex_unlock(&id_mgr->lock);
	return r;
}

/*
 * amdgpu_vmid_free_reserved - free up a reserved VMID again
 * @adev: amdgpu device structure
 * @vm: the VM with the reserved ID
 * @vmhub: the VMHUB which should be used
 */
void amdgpu_vmid_free_reserved(struct amdgpu_device *adev, struct amdgpu_vm *vm,
			       unsigned vmhub)
{
	struct amdgpu_kq_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr[vmhub];

	mutex_lock(&id_mgr->lock);
	if (vm->reserved_vmid[vmhub]) {
		vm->reserved_vmid[vmhub] = NULL;
		id_mgr->reserved_vmid = false;
	}
	mutex_unlock(&id_mgr->lock);
}

/**
 * amdgpu_vmid_reset - reset VMID to zero
 *
 * @adev: amdgpu device structure
 * @vmhub: vmhub type
 * @vmid: vmid number to use
 *
 * Reset saved GDW, GWS and OA to force switch on next flush.
 */
void amdgpu_vmid_reset(struct amdgpu_device *adev, unsigned vmhub,
		       unsigned vmid)
{
	struct amdgpu_kq_vmid_mgr *id_mgr = &adev->vm_manager.id_mgr[vmhub];
	struct amdgpu_vmid *id = &id_mgr->ids[vmid];

	mutex_lock(&id_mgr->lock);
	id->owner = 0;
	id->gds_base = 0;
	id->gds_size = 0;
	id->gws_base = 0;
	id->gws_size = 0;
	id->oa_base = 0;
	id->oa_size = 0;
	mutex_unlock(&id_mgr->lock);
}

/**
 * amdgpu_vmid_reset_all - reset VMID to zero
 *
 * @adev: amdgpu device structure
 *
 * Reset VMID to force flush on next use
 */
void amdgpu_vmid_reset_all(struct amdgpu_device *adev)
{
	unsigned i, j;

	for (i = 0; i < AMDGPU_MAX_VMHUBS; ++i) {
		struct amdgpu_kq_vmid_mgr *id_mgr =
			&adev->vm_manager.id_mgr[i];
		for_each_set_bit(j, id_mgr->vmid_mask, AMDGPU_NUM_VMID)
			amdgpu_vmid_reset(adev, i, j);
	}

	/*
	 * The SPM VMID is not part of the normal kernel pool, reset it
	 * anyway in case it is used with kernel queues.
	 */
	if (adev->vm_manager.spm_vmid)
		for (i = AMDGPU_GFXHUB_START; i < AMDGPU_MMHUB0_START; ++i)
			amdgpu_vmid_reset(adev, i, adev->vm_manager.spm_vmid);
}

/**
 * amdgpu_vmid_mgr_set_vmid_mask - set the kernel- and user-queue VMID
 * pools for both hubs
 *
 * @adev: amdgpu device structure
 * @vmid_kq_mask_gfxhub: kernel-queue VMID pool mask for GFXHUB
 * @vmid_kq_mask_mmhub: kernel-queue VMID pool mask for MMHUB
 *
 * Sets both GFXHUB and MMHUB kernel-queue VMID pools
 * via id_mgr[].ids_lru, and KFD/MES user-queue masks too.
 */
void amdgpu_vmid_mgr_set_vmid_mask(struct amdgpu_device *adev,
				   unsigned long vmid_kq_mask_gfxhub,
				   unsigned long vmid_kq_mask_mmhub)
{
	unsigned long uq_mask_gfxhub, uq_mask_mmhub;
	unsigned int i;

	BUILD_BUG_ON(AMDGPU_NUM_VMID > BITS_PER_LONG);

	/*
	 * Derive the uq masks from the kq masks as given, then carve each
	 * hub's dedicated out-of-pool VMID out of both of that hub's pools.
	 */
	uq_mask_gfxhub = GENMASK(AMDGPU_NUM_VMID - 1, 1) & ~vmid_kq_mask_gfxhub;
	uq_mask_mmhub = 0xFF00;

	/*
	 * SPM uses a dedicated vmid that is reserved from gfxhub.
	 * Unmask the reserved vmids from both user and kernel queues
	 */
	if (adev->vm_manager.spm_vmid) {
		vmid_kq_mask_gfxhub &= ~BIT(adev->vm_manager.spm_vmid);
		uq_mask_gfxhub &= ~BIT(adev->vm_manager.spm_vmid);
	}

	/*
	 * NPA uses a dedicated vmid that is reserved from mmhub.
	 * Unmask the reserved VMIDs from both user and kernel queues
	 */
	if (adev->vm_manager.npa_vmid) {
		vmid_kq_mask_mmhub &= ~BIT(adev->vm_manager.npa_vmid);
		uq_mask_mmhub &= ~BIT(adev->vm_manager.npa_vmid);
	}

	adev->vm_manager.vmid_uq_mask_gfxhub = uq_mask_gfxhub;
	adev->vm_manager.vmid_uq_mask_mmhub = uq_mask_mmhub;

	for (i = 0; i < AMDGPU_MAX_VMHUBS; i++) {
		bool is_mmhub = AMDGPU_IS_MMHUB0(i) || AMDGPU_IS_MMHUB1(i);

		bitmap_copy(adev->vm_manager.id_mgr[i].vmid_mask,
			    is_mmhub ? &vmid_kq_mask_mmhub : &vmid_kq_mask_gfxhub,
			    AMDGPU_NUM_VMID);
	}
}

/**
 * amdgpu_vmid_mgr_init - init the VMID manager
 *
 * @adev: amdgpu_device pointer
 *
 * Initialize the VM manager structures
 */
void amdgpu_vmid_mgr_init(struct amdgpu_device *adev)
{
	unsigned i, j;

	for (i = 0; i < AMDGPU_MAX_VMHUBS; ++i) {
		struct amdgpu_kq_vmid_mgr *id_mgr =
			&adev->vm_manager.id_mgr[i];

		mutex_init(&id_mgr->lock);
		INIT_LIST_HEAD(&id_mgr->ids_lru);

		/* skip over VMID 0, since it is the system VM */
		for_each_set_bit(j, id_mgr->vmid_mask, AMDGPU_NUM_VMID) {
			amdgpu_vmid_reset(adev, i, j);
			amdgpu_sync_create(&id_mgr->ids[j].active);
			list_add_tail(&id_mgr->ids[j].list, &id_mgr->ids_lru);
		}
	}

	/*
	 * The SPM VMID is not part of the normal kernel pool, create its
	 * sync anyway in case it is used with kernel queues.
	 */
	if (adev->vm_manager.spm_vmid) {
		unsigned int spm_vmid = adev->vm_manager.spm_vmid;

		for (i = AMDGPU_GFXHUB_START; i < AMDGPU_MMHUB0_START; ++i)
			amdgpu_sync_create(&adev->vm_manager.id_mgr[i].ids[spm_vmid].active);
	}
}

/**
 * amdgpu_vmid_mgr_fini - cleanup VM manager
 *
 * @adev: amdgpu_device pointer
 *
 * Cleanup the VM manager and free resources.
 */
void amdgpu_vmid_mgr_fini(struct amdgpu_device *adev)
{
	unsigned i, j;

	for (i = 0; i < AMDGPU_MAX_VMHUBS; ++i) {
		struct amdgpu_kq_vmid_mgr *id_mgr =
			&adev->vm_manager.id_mgr[i];

		mutex_destroy(&id_mgr->lock);

		for (j = 0; j < AMDGPU_NUM_VMID; ++j) {
			struct amdgpu_vmid *id = &id_mgr->ids[j];

			amdgpu_sync_free(&id->active);
			dma_fence_put(id->last_flush);
			dma_fence_put(id->pasid_mapping);
		}
	}
}

/**
 * amdgpu_pasid_mgr_cleanup - cleanup PASID manager
 *
 * Cleanup the IDR allocator.
 */
void amdgpu_pasid_mgr_cleanup(void)
{
	xa_destroy(&amdgpu_pasid_xa);
}
