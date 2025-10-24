/*
 * Copyright 2008 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 * Copyright 2009 Jerome Glisse.
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
 * Authors: Dave Airlie
 *          Alex Deucher
 *          Jerome Glisse
 *          Christian König
 */
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <drm/amdgpu_drm.h>
#include "amdgpu.h"
#include "atom.h"

/*
 * Rings
 * Most engines on the GPU are fed via ring buffers.  Ring
 * buffers are areas of GPU accessible memory that the host
 * writes commands into and the GPU reads commands out of.
 * There is a rptr (read pointer) that determines where the
 * GPU is currently reading, and a wptr (write pointer)
 * which determines where the host has written.  When the
 * pointers are equal, the ring is idle.  When the host
 * writes commands to the ring buffer, it increments the
 * wptr.  The GPU then starts fetching commands and executes
 * them until the pointers are equal again.
 */

/**
 * amdgpu_ring_max_ibs - Return max IBs that fit in a single submission.
 *
 * @type: ring type for which to return the limit.
 */
unsigned int amdgpu_ring_max_ibs(enum amdgpu_ring_type type)
{
	switch (type) {
	case AMDGPU_RING_TYPE_GFX:
		/* Need to keep at least 192 on GFX7+ for old radv. */
		return 192;
	case AMDGPU_RING_TYPE_COMPUTE:
		return 125;
	case AMDGPU_RING_TYPE_VCN_JPEG:
		return 16;
	default:
		return 49;
	}
}

/**
 * amdgpu_ring_alloc - allocate space on the ring buffer
 *
 * @ring: amdgpu_ring structure holding ring information
 * @ndw: number of dwords to allocate in the ring buffer
 *
 * Allocate @ndw dwords in the ring buffer (all asics).
 * Returns 0 on success, error on failure.
 */
int amdgpu_ring_alloc(struct amdgpu_ring *ring, unsigned int ndw)
{
	/* Align requested size with padding so unlock_commit can
	 * pad safely */
	ndw = (ndw + ring->funcs->align_mask) & ~ring->funcs->align_mask;

	/* Make sure we aren't trying to allocate more space
	 * than the maximum for one submission
	 */
	if (WARN_ON_ONCE(ndw > ring->max_dw))
		return -ENOMEM;

	ring->count_dw = ndw;
	ring->wptr_old = ring->wptr;

	if (ring->funcs->begin_use)
		ring->funcs->begin_use(ring);

	return 0;
}

/**
 * amdgpu_ring_alloc_reemit - allocate space on the ring buffer for reemit
 *
 * @ring: amdgpu_ring structure holding ring information
 * @ndw: number of dwords to allocate in the ring buffer
 *
 * Allocate @ndw dwords in the ring buffer (all asics).
 * doesn't check the max_dw limit as we may be reemitting
 * several submissions.
 */
static void amdgpu_ring_alloc_reemit(struct amdgpu_ring *ring, unsigned int ndw)
{
	/* Align requested size with padding so unlock_commit can
	 * pad safely */
	ndw = (ndw + ring->funcs->align_mask) & ~ring->funcs->align_mask;

	ring->count_dw = ndw;
	ring->wptr_old = ring->wptr;

	if (ring->funcs->begin_use)
		ring->funcs->begin_use(ring);
}

/** amdgpu_ring_insert_nop - insert NOP packets
 *
 * @ring: amdgpu_ring structure holding ring information
 * @count: the number of NOP packets to insert
 *
 * This is the generic insert_nop function for rings except SDMA
 */
void amdgpu_ring_insert_nop(struct amdgpu_ring *ring, uint32_t count)
{
	uint32_t occupied, chunk1, chunk2;

	occupied = ring->wptr & ring->buf_mask;
	chunk1 = ring->buf_mask + 1 - occupied;
	chunk1 = (chunk1 >= count) ? count : chunk1;
	chunk2 = count - chunk1;

	if (chunk1)
		memset32(&ring->ring[occupied], ring->funcs->nop, chunk1);

	if (chunk2)
		memset32(ring->ring, ring->funcs->nop, chunk2);

	ring->wptr += count;
	ring->wptr &= ring->ptr_mask;
	ring->count_dw -= count;
}

/**
 * amdgpu_ring_generic_pad_ib - pad IB with NOP packets
 *
 * @ring: amdgpu_ring structure holding ring information
 * @ib: IB to add NOP packets to
 *
 * This is the generic pad_ib function for rings except SDMA
 */
void amdgpu_ring_generic_pad_ib(struct amdgpu_ring *ring, struct amdgpu_ib *ib)
{
	u32 align_mask = ring->funcs->align_mask;
	u32 count = ib->length_dw & align_mask;

	if (count) {
		count = align_mask + 1 - count;

		memset32(&ib->ptr[ib->length_dw], ring->funcs->nop, count);

		ib->length_dw += count;
	}
}

/**
 * amdgpu_ring_commit - tell the GPU to execute the new
 * commands on the ring buffer
 *
 * @ring: amdgpu_ring structure holding ring information
 *
 * Update the wptr (write pointer) to tell the GPU to
 * execute new commands on the ring buffer (all asics).
 */
void amdgpu_ring_commit(struct amdgpu_ring *ring)
{
	uint32_t count;

	if (ring->count_dw < 0)
		DRM_ERROR("amdgpu: writing more dwords to the ring than expected!\n");

	/* We pad to match fetch size */
	count = ring->funcs->align_mask + 1 -
		(ring->wptr & ring->funcs->align_mask);
	count &= ring->funcs->align_mask;

	if (count != 0)
		ring->funcs->insert_nop(ring, count);

	mb();
	amdgpu_ring_set_wptr(ring);

	if (ring->funcs->end_use)
		ring->funcs->end_use(ring);
}

/**
 * amdgpu_ring_undo - reset the wptr
 *
 * @ring: amdgpu_ring structure holding ring information
 *
 * Reset the driver's copy of the wptr (all asics).
 */
void amdgpu_ring_undo(struct amdgpu_ring *ring)
{
	ring->wptr = ring->wptr_old;

	if (ring->funcs->end_use)
		ring->funcs->end_use(ring);
}

#define amdgpu_ring_get_gpu_addr(ring, offset)				\
	 (ring->adev->wb.gpu_addr + offset * 4)

#define amdgpu_ring_get_cpu_addr(ring, offset)				\
	 (&ring->adev->wb.wb[offset])

/**
 * amdgpu_ring_init - init driver ring struct.
 *
 * @adev: amdgpu_device pointer
 * @ring: amdgpu_ring structure holding ring information
 * @max_dw: maximum number of dw for ring alloc
 * @irq_src: interrupt source to use for this ring
 * @irq_type: interrupt type to use for this ring
 * @hw_prio: ring priority (NORMAL/HIGH)
 * @sched_score: optional score atomic shared with other schedulers
 *
 * Initialize the driver information for the selected ring (all asics).
 * Returns 0 on success, error on failure.
 */
int amdgpu_ring_init(struct amdgpu_device *adev, struct amdgpu_ring *ring,
		     unsigned int max_dw, struct amdgpu_irq_src *irq_src,
		     unsigned int irq_type, unsigned int hw_prio,
		     atomic_t *sched_score)
{
	int r;
	int sched_hw_submission = amdgpu_sched_hw_submission;
	u32 *num_sched;
	u32 hw_ip;
	unsigned int max_ibs_dw;

	/* Set the hw submission limit higher for KIQ because
	 * it's used for a number of gfx/compute tasks by both
	 * KFD and KGD which may have outstanding fences and
	 * it doesn't really use the gpu scheduler anyway;
	 * KIQ tasks get submitted directly to the ring.
	 */
	if (ring->funcs->type == AMDGPU_RING_TYPE_KIQ)
		sched_hw_submission = max(sched_hw_submission, 256);
	if (ring->funcs->type == AMDGPU_RING_TYPE_MES)
		sched_hw_submission = 8;
	else if (ring == &adev->sdma.instance[0].page)
		sched_hw_submission = 256;

	if (ring->adev == NULL) {
		if (adev->num_rings >= AMDGPU_MAX_RINGS)
			return -EINVAL;

		ring->adev = adev;
		ring->num_hw_submission = sched_hw_submission;
		ring->sched_score = sched_score;
		ring->vmid_wait = dma_fence_get_stub();

		ring->idx = adev->num_rings++;
		adev->rings[ring->idx] = ring;

		r = amdgpu_fence_driver_init_ring(ring);
		if (r)
			return r;
	}

	r = amdgpu_device_wb_get(adev, &ring->rptr_offs);
	if (r) {
		dev_err(adev->dev, "(%d) ring rptr_offs wb alloc failed\n", r);
		return r;
	}

	r = amdgpu_device_wb_get(adev, &ring->wptr_offs);
	if (r) {
		dev_err(adev->dev, "(%d) ring wptr_offs wb alloc failed\n", r);
		return r;
	}

	r = amdgpu_device_wb_get(adev, &ring->fence_offs);
	if (r) {
		dev_err(adev->dev, "(%d) ring fence_offs wb alloc failed\n", r);
		return r;
	}

	r = amdgpu_device_wb_get(adev, &ring->trail_fence_offs);
	if (r) {
		dev_err(adev->dev, "(%d) ring trail_fence_offs wb alloc failed\n", r);
		return r;
	}

	r = amdgpu_device_wb_get(adev, &ring->cond_exe_offs);
	if (r) {
		dev_err(adev->dev, "(%d) ring cond_exec_polling wb alloc failed\n", r);
		return r;
	}

	ring->fence_gpu_addr =
		amdgpu_ring_get_gpu_addr(ring, ring->fence_offs);
	ring->fence_cpu_addr =
		amdgpu_ring_get_cpu_addr(ring, ring->fence_offs);

	ring->rptr_gpu_addr =
		amdgpu_ring_get_gpu_addr(ring, ring->rptr_offs);
	ring->rptr_cpu_addr =
		amdgpu_ring_get_cpu_addr(ring, ring->rptr_offs);

	ring->wptr_gpu_addr =
		amdgpu_ring_get_gpu_addr(ring, ring->wptr_offs);
	ring->wptr_cpu_addr =
		amdgpu_ring_get_cpu_addr(ring, ring->wptr_offs);

	ring->trail_fence_gpu_addr =
		amdgpu_ring_get_gpu_addr(ring, ring->trail_fence_offs);
	ring->trail_fence_cpu_addr =
		amdgpu_ring_get_cpu_addr(ring, ring->trail_fence_offs);

	ring->cond_exe_gpu_addr =
		amdgpu_ring_get_gpu_addr(ring, ring->cond_exe_offs);
	ring->cond_exe_cpu_addr =
		amdgpu_ring_get_cpu_addr(ring, ring->cond_exe_offs);

	/* always set cond_exec_polling to CONTINUE */
	*ring->cond_exe_cpu_addr = 1;

	if (ring->funcs->type != AMDGPU_RING_TYPE_CPER) {
		r = amdgpu_fence_driver_start_ring(ring, irq_src, irq_type);
		if (r) {
			dev_err(adev->dev, "failed initializing fences (%d).\n", r);
			return r;
		}

		max_ibs_dw = ring->funcs->emit_frame_size +
			     amdgpu_ring_max_ibs(ring->funcs->type) * ring->funcs->emit_ib_size;
		max_ibs_dw = (max_ibs_dw + ring->funcs->align_mask) & ~ring->funcs->align_mask;

		if (WARN_ON(max_ibs_dw > max_dw))
			max_dw = max_ibs_dw;

		ring->ring_size = roundup_pow_of_two(max_dw * 4 * sched_hw_submission);
	} else {
		ring->ring_size = roundup_pow_of_two(max_dw * 4);
		ring->count_dw = (ring->ring_size - 4) >> 2;
		/* ring buffer is empty now */
		ring->wptr = *ring->rptr_cpu_addr = 0;
	}

	ring->buf_mask = (ring->ring_size / 4) - 1;
	ring->ptr_mask = ring->funcs->support_64bit_ptrs ?
		0xffffffffffffffff : ring->buf_mask;
	/*  Initialize cached_rptr to 0 */
	ring->cached_rptr = 0;

	if (!ring->ring_backup) {
		ring->ring_backup = kvzalloc(ring->ring_size, GFP_KERNEL);
		if (!ring->ring_backup)
			return -ENOMEM;
	}

	/* Allocate ring buffer */
	if (ring->ring_obj == NULL) {
		r = amdgpu_bo_create_kernel(adev, ring->ring_size + ring->funcs->extra_bytes,
					    PAGE_SIZE,
					    AMDGPU_GEM_DOMAIN_GTT,
					    &ring->ring_obj,
					    &ring->gpu_addr,
					    (void **)&ring->ring);
		if (r) {
			dev_err(adev->dev, "(%d) ring create failed\n", r);
			kvfree(ring->ring_backup);
			return r;
		}
		amdgpu_ring_clear_ring(ring);
	}

	ring->max_dw = max_dw;
	ring->hw_prio = hw_prio;

	if (!ring->no_scheduler && ring->funcs->type < AMDGPU_HW_IP_NUM) {
		hw_ip = ring->funcs->type;
		num_sched = &adev->gpu_sched[hw_ip][hw_prio].num_scheds;
		adev->gpu_sched[hw_ip][hw_prio].sched[(*num_sched)++] =
			&ring->sched;
	}

	return 0;
}

/**
 * amdgpu_ring_fini - tear down the driver ring struct.
 *
 * @ring: amdgpu_ring structure holding ring information
 *
 * Tear down the driver information for the selected ring (all asics).
 */
void amdgpu_ring_fini(struct amdgpu_ring *ring)
{

	/* Not to finish a ring which is not initialized */
	if (!(ring->adev) || !(ring->adev->rings[ring->idx]))
		return;

	ring->sched.ready = false;

	amdgpu_device_wb_free(ring->adev, ring->rptr_offs);
	amdgpu_device_wb_free(ring->adev, ring->wptr_offs);

	amdgpu_device_wb_free(ring->adev, ring->cond_exe_offs);
	amdgpu_device_wb_free(ring->adev, ring->fence_offs);

	amdgpu_bo_free_kernel(&ring->ring_obj,
			      &ring->gpu_addr,
			      (void **)&ring->ring);
	kvfree(ring->ring_backup);
	ring->ring_backup = NULL;

	dma_fence_put(ring->vmid_wait);
	ring->vmid_wait = NULL;
	ring->me = 0;
}

/**
 * amdgpu_ring_emit_reg_write_reg_wait_helper - ring helper
 *
 * @ring: ring to write to
 * @reg0: register to write
 * @reg1: register to wait on
 * @ref: reference value to write/wait on
 * @mask: mask to wait on
 *
 * Helper for rings that don't support write and wait in a
 * single oneshot packet.
 */
void amdgpu_ring_emit_reg_write_reg_wait_helper(struct amdgpu_ring *ring,
						uint32_t reg0, uint32_t reg1,
						uint32_t ref, uint32_t mask)
{
	amdgpu_ring_emit_wreg(ring, reg0, ref);
	amdgpu_ring_emit_reg_wait(ring, reg1, mask, mask);
}

/**
 * amdgpu_ring_soft_recovery - try to soft recover a ring lockup
 *
 * @ring: ring to try the recovery on
 * @vmid: VMID we try to get going again
 * @fence: timedout fence
 *
 * Tries to get a ring proceeding again when it is stuck.
 */
bool amdgpu_ring_soft_recovery(struct amdgpu_ring *ring, unsigned int vmid,
			       struct dma_fence *fence)
{
	unsigned long flags;
	ktime_t deadline;
	bool ret;

	if (unlikely(ring->adev->debug_disable_soft_recovery))
		return false;

	deadline = ktime_add_us(ktime_get(), 10000);

	if (amdgpu_sriov_vf(ring->adev) || !ring->funcs->soft_recovery || !fence)
		return false;

	spin_lock_irqsave(fence->lock, flags);
	if (!dma_fence_is_signaled_locked(fence))
		dma_fence_set_error(fence, -ENODATA);
	spin_unlock_irqrestore(fence->lock, flags);

	while (!dma_fence_is_signaled(fence) &&
	       ktime_to_ns(ktime_sub(deadline, ktime_get())) > 0)
		ring->funcs->soft_recovery(ring, vmid);

	ret = dma_fence_is_signaled(fence);
	/* increment the counter only if soft reset worked */
	if (ret)
		atomic_inc(&ring->adev->gpu_reset_counter);

	return ret;
}

/*
 * Debugfs info
 */
#if defined(CONFIG_DEBUG_FS)

/* Layout of file is 12 bytes consisting of
 * - rptr
 * - wptr
 * - driver's copy of wptr
 *
 * followed by n-words of ring data
 */
static ssize_t amdgpu_debugfs_ring_read(struct file *f, char __user *buf,
					size_t size, loff_t *pos)
{
	struct amdgpu_ring *ring = file_inode(f)->i_private;
	uint32_t value, result, early[3];
	uint64_t p;
	loff_t i;
	int r;

	if (*pos & 3 || size & 3)
		return -EINVAL;

	result = 0;

	if (*pos < 12) {
		if (ring->funcs->type == AMDGPU_RING_TYPE_CPER)
			mutex_lock(&ring->adev->cper.ring_lock);

		early[0] = amdgpu_ring_get_rptr(ring) & ring->buf_mask;
		early[1] = amdgpu_ring_get_wptr(ring) & ring->buf_mask;
		early[2] = ring->wptr & ring->buf_mask;
		for (i = *pos / 4; i < 3 && size; i++) {
			r = put_user(early[i], (uint32_t *)buf);
			if (r) {
				result = r;
				goto out;
			}
			buf += 4;
			result += 4;
			size -= 4;
			*pos += 4;
		}
	}

	if (ring->funcs->type != AMDGPU_RING_TYPE_CPER) {
		while (size) {
			if (*pos >= (ring->ring_size + 12))
				return result;

			value = ring->ring[(*pos - 12)/4];
			r = put_user(value, (uint32_t *)buf);
			if (r)
				return r;
			buf += 4;
			result += 4;
			size -= 4;
			*pos += 4;
		}
	} else {
		p = early[0];
		if (early[0] <= early[1])
			size = (early[1] - early[0]);
		else
			size = ring->ring_size - (early[0] - early[1]);

		while (size) {
			if (p == early[1])
				goto out;

			value = ring->ring[p];
			r = put_user(value, (uint32_t *)buf);
			if (r) {
				result = r;
				goto out;
			}

			buf += 4;
			result += 4;
			size--;
			p++;
			p &= ring->ptr_mask;
		}
	}

out:
	if (ring->funcs->type == AMDGPU_RING_TYPE_CPER)
		mutex_unlock(&ring->adev->cper.ring_lock);

	return result;
}

static ssize_t amdgpu_debugfs_virt_ring_read(struct file *f, char __user *buf,
	size_t size, loff_t *pos)
{
	struct amdgpu_ring *ring = file_inode(f)->i_private;

	if (*pos & 3 || size & 3)
		return -EINVAL;

	if (ring->funcs->type == AMDGPU_RING_TYPE_CPER)
		amdgpu_virt_req_ras_cper_dump(ring->adev, false);

	return amdgpu_debugfs_ring_read(f, buf, size, pos);
}

static const struct file_operations amdgpu_debugfs_ring_fops = {
	.owner = THIS_MODULE,
	.read = amdgpu_debugfs_ring_read,
	.llseek = default_llseek
};

static const struct file_operations amdgpu_debugfs_virt_ring_fops = {
	.owner = THIS_MODULE,
	.read = amdgpu_debugfs_virt_ring_read,
	.llseek = default_llseek
};

static ssize_t amdgpu_debugfs_mqd_read(struct file *f, char __user *buf,
				       size_t size, loff_t *pos)
{
	struct amdgpu_ring *ring = file_inode(f)->i_private;
	ssize_t bytes = min_t(ssize_t, ring->mqd_size - *pos, size);
	void *from = ((u8 *)ring->mqd_ptr) + *pos;

	if (*pos > ring->mqd_size)
		return 0;

	if (copy_to_user(buf, from, bytes))
		return -EFAULT;

	*pos += bytes;
	return bytes;
}

static const struct file_operations amdgpu_debugfs_mqd_fops = {
	.owner = THIS_MODULE,
	.read = amdgpu_debugfs_mqd_read,
	.llseek = default_llseek
};

static int amdgpu_debugfs_ring_error(void *data, u64 val)
{
	struct amdgpu_ring *ring = data;

	amdgpu_fence_driver_set_error(ring, val);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE_SIGNED(amdgpu_debugfs_error_fops, NULL,
				amdgpu_debugfs_ring_error, "%lld\n");

#endif

void amdgpu_debugfs_ring_init(struct amdgpu_device *adev,
			      struct amdgpu_ring *ring)
{
#if defined(CONFIG_DEBUG_FS)
	struct drm_minor *minor = adev_to_drm(adev)->primary;
	struct dentry *root = minor->debugfs_root;
	char name[32];

	sprintf(name, "amdgpu_ring_%s", ring->name);
	if (amdgpu_sriov_vf(adev))
		debugfs_create_file_size(name, S_IFREG | 0444, root, ring,
					 &amdgpu_debugfs_virt_ring_fops,
					 ring->ring_size + 12);
	else
		debugfs_create_file_size(name, S_IFREG | 0444, root, ring,
					 &amdgpu_debugfs_ring_fops,
					 ring->ring_size + 12);

	if (ring->mqd_obj) {
		sprintf(name, "amdgpu_mqd_%s", ring->name);
		debugfs_create_file_size(name, S_IFREG | 0444, root, ring,
					 &amdgpu_debugfs_mqd_fops,
					 ring->mqd_size);
	}

	sprintf(name, "amdgpu_error_%s", ring->name);
	debugfs_create_file(name, 0200, root, ring,
			    &amdgpu_debugfs_error_fops);

#endif
}

/**
 * amdgpu_ring_test_helper - tests ring and set sched readiness status
 *
 * @ring: ring to try the recovery on
 *
 * Tests ring and set sched readiness status
 *
 * Returns 0 on success, error on failure.
 */
int amdgpu_ring_test_helper(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	int r;

	r = amdgpu_ring_test_ring(ring);
	if (r)
		DRM_DEV_ERROR(adev->dev, "ring %s test failed (%d)\n",
			      ring->name, r);
	else
		DRM_DEV_DEBUG(adev->dev, "ring test on %s succeeded\n",
			      ring->name);

	ring->sched.ready = !r;

	return r;
}

static void amdgpu_ring_to_mqd_prop(struct amdgpu_ring *ring,
				    struct amdgpu_mqd_prop *prop)
{
	struct amdgpu_device *adev = ring->adev;
	bool is_high_prio_compute = ring->funcs->type == AMDGPU_RING_TYPE_COMPUTE &&
				    amdgpu_gfx_is_high_priority_compute_queue(adev, ring);
	bool is_high_prio_gfx = ring->funcs->type == AMDGPU_RING_TYPE_GFX &&
				amdgpu_gfx_is_high_priority_graphics_queue(adev, ring);

	memset(prop, 0, sizeof(*prop));

	prop->mqd_gpu_addr = ring->mqd_gpu_addr;
	prop->hqd_base_gpu_addr = ring->gpu_addr;
	prop->rptr_gpu_addr = ring->rptr_gpu_addr;
	prop->wptr_gpu_addr = ring->wptr_gpu_addr;
	prop->queue_size = ring->ring_size;
	prop->eop_gpu_addr = ring->eop_gpu_addr;
	prop->use_doorbell = ring->use_doorbell;
	prop->doorbell_index = ring->doorbell_index;
	prop->kernel_queue = true;

	/* map_queues packet doesn't need activate the queue,
	 * so only kiq need set this field.
	 */
	prop->hqd_active = ring->funcs->type == AMDGPU_RING_TYPE_KIQ;

	prop->allow_tunneling = is_high_prio_compute;
	if (is_high_prio_compute || is_high_prio_gfx) {
		prop->hqd_pipe_priority = AMDGPU_GFX_PIPE_PRIO_HIGH;
		prop->hqd_queue_priority = AMDGPU_GFX_QUEUE_PRIORITY_MAXIMUM;
	}
}

int amdgpu_ring_init_mqd(struct amdgpu_ring *ring)
{
	struct amdgpu_device *adev = ring->adev;
	struct amdgpu_mqd *mqd_mgr;
	struct amdgpu_mqd_prop prop;

	amdgpu_ring_to_mqd_prop(ring, &prop);

	ring->wptr = 0;

	if (ring->funcs->type == AMDGPU_RING_TYPE_KIQ)
		mqd_mgr = &adev->mqds[AMDGPU_HW_IP_COMPUTE];
	else
		mqd_mgr = &adev->mqds[ring->funcs->type];

	return mqd_mgr->init_mqd(adev, ring->mqd_ptr, &prop);
}

void amdgpu_ring_ib_begin(struct amdgpu_ring *ring)
{
	if (ring->is_sw_ring)
		amdgpu_sw_ring_ib_begin(ring);
}

void amdgpu_ring_ib_end(struct amdgpu_ring *ring)
{
	if (ring->is_sw_ring)
		amdgpu_sw_ring_ib_end(ring);
}

void amdgpu_ring_ib_on_emit_cntl(struct amdgpu_ring *ring)
{
	if (ring->is_sw_ring)
		amdgpu_sw_ring_ib_mark_offset(ring, AMDGPU_MUX_OFFSET_TYPE_CONTROL);
}

void amdgpu_ring_ib_on_emit_ce(struct amdgpu_ring *ring)
{
	if (ring->is_sw_ring)
		amdgpu_sw_ring_ib_mark_offset(ring, AMDGPU_MUX_OFFSET_TYPE_CE);
}

void amdgpu_ring_ib_on_emit_de(struct amdgpu_ring *ring)
{
	if (ring->is_sw_ring)
		amdgpu_sw_ring_ib_mark_offset(ring, AMDGPU_MUX_OFFSET_TYPE_DE);
}

bool amdgpu_ring_sched_ready(struct amdgpu_ring *ring)
{
	if (!ring)
		return false;

	if (ring->no_scheduler || !drm_sched_wqueue_ready(&ring->sched))
		return false;

	return true;
}

void amdgpu_ring_reset_helper_begin(struct amdgpu_ring *ring,
				    struct amdgpu_fence *guilty_fence)
{
	/* Stop the scheduler to prevent anybody else from touching the ring buffer. */
	drm_sched_wqueue_stop(&ring->sched);
	/* back up the non-guilty commands */
	amdgpu_ring_backup_unprocessed_commands(ring, guilty_fence);
}

int amdgpu_ring_reset_helper_end(struct amdgpu_ring *ring,
				 struct amdgpu_fence *guilty_fence)
{
	unsigned int i;
	int r;

	/* verify that the ring is functional */
	r = amdgpu_ring_test_ring(ring);
	if (r)
		return r;

	/* signal the guilty fence and set an error on all fences from the context */
	if (guilty_fence)
		amdgpu_fence_driver_guilty_force_completion(guilty_fence);
	/* Re-emit the non-guilty commands */
	if (ring->ring_backup_entries_to_copy) {
		amdgpu_ring_alloc_reemit(ring, ring->ring_backup_entries_to_copy);
		for (i = 0; i < ring->ring_backup_entries_to_copy; i++)
			amdgpu_ring_write(ring, ring->ring_backup[i]);
		amdgpu_ring_commit(ring);
	}
	/* Start the scheduler again */
	drm_sched_wqueue_start(&ring->sched);
	return 0;
}

bool amdgpu_ring_is_reset_type_supported(struct amdgpu_ring *ring,
					 u32 reset_type)
{
	switch (ring->funcs->type) {
	case AMDGPU_RING_TYPE_GFX:
		if (ring->adev->gfx.gfx_supported_reset & reset_type)
			return true;
		break;
	case AMDGPU_RING_TYPE_COMPUTE:
		if (ring->adev->gfx.compute_supported_reset & reset_type)
			return true;
		break;
	case AMDGPU_RING_TYPE_SDMA:
		if (ring->adev->sdma.supported_reset & reset_type)
			return true;
		break;
	case AMDGPU_RING_TYPE_VCN_DEC:
	case AMDGPU_RING_TYPE_VCN_ENC:
		if (ring->adev->vcn.supported_reset & reset_type)
			return true;
		break;
	case AMDGPU_RING_TYPE_VCN_JPEG:
		if (ring->adev->jpeg.supported_reset & reset_type)
			return true;
		break;
	default:
		break;
	}
	return false;
}
