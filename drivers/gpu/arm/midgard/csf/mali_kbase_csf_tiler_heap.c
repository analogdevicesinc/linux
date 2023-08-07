// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2022 ARM Limited. All rights reserved.
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

#include <tl/mali_kbase_tracepoints.h>

#include "mali_kbase_csf_tiler_heap.h"
#include "mali_kbase_csf_tiler_heap_def.h"
#include "mali_kbase_csf_heap_context_alloc.h"

/**
 * encode_chunk_ptr - Encode the address and size of a chunk as an integer.
 *
 * @chunk_size: Size of a tiler heap chunk, in bytes.
 * @chunk_addr: GPU virtual address of the same tiler heap chunk.
 *
 * The size and address of the next chunk in a list are packed into a single
 * 64-bit value for storage in a chunk's header. This function returns that
 * value.
 *
 * Return: Next chunk pointer suitable for writing into a chunk header.
 */
static u64 encode_chunk_ptr(u32 const chunk_size, u64 const chunk_addr)
{
	u64 encoded_size, encoded_addr;

	WARN_ON(chunk_size & ~CHUNK_SIZE_MASK);
	WARN_ON(chunk_addr & ~CHUNK_ADDR_MASK);

	encoded_size =
		(u64)(chunk_size >> CHUNK_HDR_NEXT_SIZE_ENCODE_SHIFT) <<
		CHUNK_HDR_NEXT_SIZE_POS;

	encoded_addr =
		(chunk_addr >> CHUNK_HDR_NEXT_ADDR_ENCODE_SHIFT) <<
		CHUNK_HDR_NEXT_ADDR_POS;

	return (encoded_size & CHUNK_HDR_NEXT_SIZE_MASK) |
		(encoded_addr & CHUNK_HDR_NEXT_ADDR_MASK);
}

/**
 * get_last_chunk - Get the last chunk of a tiler heap
 *
 * @heap:  Pointer to the tiler heap.
 *
 * Return: The address of the most recently-linked chunk, or NULL if none.
 */
static struct kbase_csf_tiler_heap_chunk *get_last_chunk(
	struct kbase_csf_tiler_heap *const heap)
{
	if (list_empty(&heap->chunks_list))
		return NULL;

	return list_last_entry(&heap->chunks_list,
		struct kbase_csf_tiler_heap_chunk, link);
}

/**
 * link_chunk - Link a chunk into a tiler heap
 *
 * @heap:  Pointer to the tiler heap.
 * @chunk: Pointer to the heap chunk to be linked.
 *
 * Unless the @chunk is the first in the kernel's list of chunks belonging to
 * a given tiler heap, this function stores the size and address of the @chunk
 * in the header of the preceding chunk. This requires the GPU memory region
 * containing the header to be mapped temporarily, which can fail.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int link_chunk(struct kbase_csf_tiler_heap *const heap,
	struct kbase_csf_tiler_heap_chunk *const chunk)
{
	struct kbase_csf_tiler_heap_chunk *const prev = get_last_chunk(heap);

	if (prev) {
		struct kbase_context *const kctx = heap->kctx;
		struct kbase_vmap_struct map;
		u64 *const prev_hdr = kbase_vmap_prot(kctx, prev->gpu_va,
			sizeof(*prev_hdr), KBASE_REG_CPU_WR, &map);

		if (unlikely(!prev_hdr)) {
			dev_err(kctx->kbdev->dev,
				"Failed to map tiler heap chunk 0x%llX\n",
				prev->gpu_va);
			return -ENOMEM;
		}

		*prev_hdr = encode_chunk_ptr(heap->chunk_size, chunk->gpu_va);
		kbase_vunmap(kctx, &map);

		dev_dbg(kctx->kbdev->dev,
			"Linked tiler heap chunks, 0x%llX -> 0x%llX\n",
			prev->gpu_va, chunk->gpu_va);
	}

	return 0;
}

/**
 * init_chunk - Initialize and link a tiler heap chunk
 *
 * @heap:  Pointer to the tiler heap.
 * @chunk: Pointer to the heap chunk to be initialized and linked.
 * @link_with_prev: Flag to indicate if the new chunk needs to be linked with
 *                  the previously allocated chunk.
 *
 * Zero-initialize a new chunk's header (including its pointer to the next
 * chunk, which doesn't exist yet) and then update the previous chunk's
 * header to link the new chunk into the chunk list.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int init_chunk(struct kbase_csf_tiler_heap *const heap,
	struct kbase_csf_tiler_heap_chunk *const chunk, bool link_with_prev)
{
	struct kbase_vmap_struct map;
	struct u64 *chunk_hdr = NULL;
	struct kbase_context *const kctx = heap->kctx;

	if (unlikely(chunk->gpu_va & ~CHUNK_ADDR_MASK)) {
		dev_err(kctx->kbdev->dev,
			"Tiler heap chunk address is unusable\n");
		return -EINVAL;
	}

	chunk_hdr = kbase_vmap_prot(kctx,
		chunk->gpu_va, CHUNK_HDR_SIZE, KBASE_REG_CPU_WR, &map);

	if (unlikely(!chunk_hdr)) {
		dev_err(kctx->kbdev->dev,
			"Failed to map a tiler heap chunk header\n");
		return -ENOMEM;
	}

	memset(chunk_hdr, 0, CHUNK_HDR_SIZE);
	kbase_vunmap(kctx, &map);

	if (link_with_prev)
		return link_chunk(heap, chunk);
	else
		return 0;
}

/**
 * create_chunk - Create a tiler heap chunk
 *
 * @heap: Pointer to the tiler heap for which to allocate memory.
 * @link_with_prev: Flag to indicate if the chunk to be allocated needs to be
 *                  linked with the previously allocated chunk.
 *
 * This function allocates a chunk of memory for a tiler heap and adds it to
 * the end of the list of chunks associated with that heap. The size of the
 * chunk is not a parameter because it is configured per-heap not per-chunk.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int create_chunk(struct kbase_csf_tiler_heap *const heap,
			bool link_with_prev)
{
	int err = 0;
	struct kbase_context *const kctx = heap->kctx;
	u64 nr_pages = PFN_UP(heap->chunk_size);
	u64 flags = BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR | BASE_MEM_PROT_CPU_WR |
		    BASEP_MEM_NO_USER_FREE | BASE_MEM_COHERENT_LOCAL | BASE_MEM_PROT_CPU_RD;
	struct kbase_csf_tiler_heap_chunk *chunk = NULL;

	/* Calls to this function are inherently synchronous, with respect to
	 * MMU operations.
	 */
	const enum kbase_caller_mmu_sync_info mmu_sync_info = CALLER_MMU_SYNC;

	flags |= kbase_mem_group_id_set(kctx->jit_group_id);

	chunk = kzalloc(sizeof(*chunk), GFP_KERNEL);
	if (unlikely(!chunk)) {
		dev_err(kctx->kbdev->dev,
			"No kernel memory for a new tiler heap chunk\n");
		return -ENOMEM;
	}

	/* Allocate GPU memory for the new chunk. */
	INIT_LIST_HEAD(&chunk->link);
	chunk->region =
		kbase_mem_alloc(kctx, nr_pages, nr_pages, 0, &flags, &chunk->gpu_va, mmu_sync_info);

	if (unlikely(!chunk->region)) {
		dev_err(kctx->kbdev->dev,
			"Failed to allocate a tiler heap chunk\n");
		err = -ENOMEM;
	} else {
		err = init_chunk(heap, chunk, link_with_prev);
		if (unlikely(err)) {
			kbase_gpu_vm_lock(kctx);
			chunk->region->flags &= ~KBASE_REG_NO_USER_FREE;
			kbase_mem_free_region(kctx, chunk->region);
			kbase_gpu_vm_unlock(kctx);
		}
	}

	if (unlikely(err)) {
		kfree(chunk);
	} else {
		list_add_tail(&chunk->link, &heap->chunks_list);
		heap->chunk_count++;

		dev_dbg(kctx->kbdev->dev, "Created tiler heap chunk 0x%llX\n",
			chunk->gpu_va);
	}

	return err;
}

static void mark_free_mem_bypassing_pool(struct kbase_va_region *reg)
{
	if (WARN_ON(reg->gpu_alloc == NULL))
		return;

	reg->gpu_alloc->evicted = reg->gpu_alloc->nents;
	kbase_mem_evictable_mark_reclaim(reg->gpu_alloc);
}

/**
 * delete_chunk - Delete a tiler heap chunk
 *
 * @heap:  Pointer to the tiler heap for which @chunk was allocated.
 * @chunk: Pointer to a chunk to be deleted.
 * @reclaim: Indicating the deletion is from shrinking reclaim or not.
 *
 * This function frees a tiler heap chunk previously allocated by @create_chunk
 * and removes it from the list of chunks associated with the heap.
 *
 * WARNING: The deleted chunk is not unlinked from the list of chunks used by
 *          the GPU, therefore it is only safe to use this function when
 *          deleting a heap, or under reclaim operations when the relevant CSGS
 *          are off-slots for the given kctx.
 */
static void delete_chunk(struct kbase_csf_tiler_heap *const heap,
			 struct kbase_csf_tiler_heap_chunk *const chunk, bool reclaim)
{
	struct kbase_context *const kctx = heap->kctx;

	kbase_gpu_vm_lock(kctx);
	chunk->region->flags &= ~KBASE_REG_NO_USER_FREE;
	if (reclaim)
		mark_free_mem_bypassing_pool(chunk->region);
	kbase_mem_free_region(kctx, chunk->region);
	kbase_gpu_vm_unlock(kctx);
	list_del(&chunk->link);
	heap->chunk_count--;
	kfree(chunk);
}

/**
 * delete_all_chunks - Delete all chunks belonging to a tiler heap
 *
 * @heap: Pointer to a tiler heap.
 *
 * This function empties the list of chunks associated with a tiler heap by
 * freeing all chunks previously allocated by @create_chunk.
 */
static void delete_all_chunks(struct kbase_csf_tiler_heap *heap)
{
	struct list_head *entry = NULL, *tmp = NULL;

	list_for_each_safe(entry, tmp, &heap->chunks_list) {
		struct kbase_csf_tiler_heap_chunk *chunk = list_entry(
			entry, struct kbase_csf_tiler_heap_chunk, link);

		delete_chunk(heap, chunk, false);
	}
}

/**
 * create_initial_chunks - Create the initial list of chunks for a tiler heap
 *
 * @heap:    Pointer to the tiler heap for which to allocate memory.
 * @nchunks: Number of chunks to create.
 *
 * This function allocates a given number of chunks for a tiler heap and
 * adds them to the list of chunks associated with that heap.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int create_initial_chunks(struct kbase_csf_tiler_heap *const heap,
	u32 const nchunks)
{
	int err = 0;
	u32 i;

	for (i = 0; (i < nchunks) && likely(!err); i++)
		err = create_chunk(heap, true);

	if (unlikely(err))
		delete_all_chunks(heap);

	return err;
}

/**
 * delete_heap - Delete a tiler heap
 *
 * @heap: Pointer to a tiler heap to be deleted.
 *
 * This function frees any chunks allocated for a tiler heap previously
 * initialized by @kbase_csf_tiler_heap_init and removes it from the list of
 * heaps associated with the kbase context. The heap context structure used by
 * the firmware is also freed.
 */
static void delete_heap(struct kbase_csf_tiler_heap *heap)
{
	struct kbase_context *const kctx = heap->kctx;

	dev_dbg(kctx->kbdev->dev, "Deleting tiler heap 0x%llX\n", heap->gpu_va);

	lockdep_assert_held(&kctx->csf.tiler_heaps.lock);

	delete_all_chunks(heap);

	/* We could optimize context destruction by not freeing leaked heap
	 * contexts but it doesn't seem worth the extra complexity.
	 */
	kbase_csf_heap_context_allocator_free(&kctx->csf.tiler_heaps.ctx_alloc,
		heap->gpu_va);

	list_del(&heap->link);
	atomic_sub(PFN_UP(heap->chunk_size), &kctx->csf.tiler_heaps.est_count_pages);

	WARN_ON(heap->chunk_count);
	KBASE_TLSTREAM_AUX_TILER_HEAP_STATS(kctx->kbdev, kctx->id,
		heap->heap_id, 0, 0, heap->max_chunks, heap->chunk_size, 0,
		heap->target_in_flight, 0);

	if (heap->buf_desc_va) {
		kbase_gpu_vm_lock(kctx);
		heap->buf_desc_reg->flags &= ~KBASE_REG_NO_USER_FREE;
		kbase_gpu_vm_unlock(kctx);
	}

	kfree(heap);
}

/**
 * find_tiler_heap - Find a tiler heap from the address of its heap context
 *
 * @kctx:        Pointer to the kbase context to search for a tiler heap.
 * @heap_gpu_va: GPU virtual address of a heap context structure.
 *
 * Each tiler heap managed by the kernel has an associated heap context
 * structure used by the firmware. This function finds a tiler heap object from
 * the GPU virtual address of its associated heap context. The heap context
 * should have been allocated by @kbase_csf_heap_context_allocator_alloc in the
 * same @kctx.
 *
 * Return: pointer to the tiler heap object, or NULL if not found.
 */
static struct kbase_csf_tiler_heap *find_tiler_heap(
	struct kbase_context *const kctx, u64 const heap_gpu_va)
{
	struct kbase_csf_tiler_heap *heap = NULL;

	lockdep_assert_held(&kctx->csf.tiler_heaps.lock);

	list_for_each_entry(heap, &kctx->csf.tiler_heaps.list, link) {
		if (heap_gpu_va == heap->gpu_va)
			return heap;
	}

	dev_dbg(kctx->kbdev->dev, "Tiler heap 0x%llX was not found\n",
		heap_gpu_va);

	return NULL;
}

int kbase_csf_tiler_heap_context_init(struct kbase_context *const kctx)
{
	int err = kbase_csf_heap_context_allocator_init(
		&kctx->csf.tiler_heaps.ctx_alloc, kctx);

	if (unlikely(err))
		return err;

	INIT_LIST_HEAD(&kctx->csf.tiler_heaps.list);
	mutex_init(&kctx->csf.tiler_heaps.lock);
	atomic_set(&kctx->csf.tiler_heaps.est_count_pages, 0);

	dev_dbg(kctx->kbdev->dev, "Initialized a context for tiler heaps\n");

	return 0;
}

void kbase_csf_tiler_heap_context_term(struct kbase_context *const kctx)
{
	struct list_head *entry = NULL, *tmp = NULL;

	dev_dbg(kctx->kbdev->dev, "Terminating a context for tiler heaps\n");

	mutex_lock(&kctx->csf.tiler_heaps.lock);

	list_for_each_safe(entry, tmp, &kctx->csf.tiler_heaps.list) {
		struct kbase_csf_tiler_heap *heap = list_entry(
			entry, struct kbase_csf_tiler_heap, link);
		delete_heap(heap);
	}

	WARN_ON(atomic_read(&kctx->csf.tiler_heaps.est_count_pages) != 0);
	mutex_unlock(&kctx->csf.tiler_heaps.lock);
	mutex_destroy(&kctx->csf.tiler_heaps.lock);

	kbase_csf_heap_context_allocator_term(&kctx->csf.tiler_heaps.ctx_alloc);
}

int kbase_csf_tiler_heap_init(struct kbase_context *const kctx, u32 const chunk_size,
			      u32 const initial_chunks, u32 const max_chunks,
			      u16 const target_in_flight, u64 const buf_desc_va,
			      u64 *const heap_gpu_va, u64 *const first_chunk_va)
{
	int err = 0;
	struct kbase_csf_tiler_heap *heap = NULL;
	struct kbase_csf_heap_context_allocator *const ctx_alloc =
		&kctx->csf.tiler_heaps.ctx_alloc;
	struct kbase_va_region *reg = NULL;

	dev_dbg(kctx->kbdev->dev,
		"Creating a tiler heap with %u chunks (limit: %u) of size %u, buf_desc_va: 0x%llx",
		initial_chunks, max_chunks, chunk_size, buf_desc_va);

	if (!kbase_mem_allow_alloc(kctx))
		return -EINVAL;

	if (chunk_size == 0)
		return -EINVAL;

	if (chunk_size & ~CHUNK_SIZE_MASK)
		return -EINVAL;

	if (initial_chunks == 0)
		return -EINVAL;

	if (initial_chunks > max_chunks)
		return -EINVAL;

	if (target_in_flight == 0)
		return -EINVAL;

	/* Check on the buffer descriptor virtual Address */
	if (buf_desc_va) {
		kbase_gpu_vm_lock(kctx);
		reg = kbase_region_tracker_find_region_enclosing_address(kctx, buf_desc_va);
		if (kbase_is_region_invalid_or_free(reg) || !(reg->flags & KBASE_REG_CPU_RD) ||
		    (reg->gpu_alloc->type != KBASE_MEM_TYPE_NATIVE)) {
			kbase_gpu_vm_unlock(kctx);
			return -EINVAL;
		}

		reg->flags |= KBASE_REG_NO_USER_FREE;
		kbase_gpu_vm_unlock(kctx);
	}

	heap = kzalloc(sizeof(*heap), GFP_KERNEL);
	if (unlikely(!heap)) {
		dev_err(kctx->kbdev->dev, "No kernel memory for a new tiler heap");
		err = -ENOMEM;
		goto err_out;
	}

	heap->kctx = kctx;
	heap->chunk_size = chunk_size;
	heap->max_chunks = max_chunks;
	heap->target_in_flight = target_in_flight;
	heap->buf_desc_va = buf_desc_va;
	heap->buf_desc_reg = reg;
	heap->desc_chk_flags = 0;
	heap->desc_chk_cnt = 0;
	INIT_LIST_HEAD(&heap->chunks_list);

	heap->gpu_va = kbase_csf_heap_context_allocator_alloc(ctx_alloc);

	if (unlikely(!heap->gpu_va)) {
		dev_dbg(kctx->kbdev->dev,
			"Failed to allocate a tiler heap context");
		err = -ENOMEM;
	} else {
		err = create_initial_chunks(heap, initial_chunks);
		if (unlikely(err))
			kbase_csf_heap_context_allocator_free(ctx_alloc, heap->gpu_va);
	}

	if (likely(!err)) {
		struct kbase_csf_tiler_heap_chunk const *chunk = list_first_entry(
			&heap->chunks_list, struct kbase_csf_tiler_heap_chunk, link);

		*heap_gpu_va = heap->gpu_va;
		*first_chunk_va = chunk->gpu_va;

		mutex_lock(&kctx->csf.tiler_heaps.lock);
		kctx->csf.tiler_heaps.nr_of_heaps++;
		heap->heap_id = kctx->csf.tiler_heaps.nr_of_heaps;
		list_add(&heap->link, &kctx->csf.tiler_heaps.list);

		KBASE_TLSTREAM_AUX_TILER_HEAP_STATS(
			kctx->kbdev, kctx->id, heap->heap_id,
			PFN_UP(heap->chunk_size * heap->max_chunks),
			PFN_UP(heap->chunk_size * heap->chunk_count), heap->max_chunks,
			heap->chunk_size, heap->chunk_count, heap->target_in_flight, 0);

#if defined(CONFIG_MALI_VECTOR_DUMP)
		list_for_each_entry(chunk, &heap->chunks_list, link) {
			KBASE_TLSTREAM_JD_TILER_HEAP_CHUNK_ALLOC(
				kctx->kbdev, kctx->id, heap->heap_id, chunk->gpu_va);
		}
#endif
		kctx->running_total_tiler_heap_nr_chunks += heap->chunk_count;
		kctx->running_total_tiler_heap_memory += (u64)heap->chunk_size * heap->chunk_count;
		if (kctx->running_total_tiler_heap_memory > kctx->peak_total_tiler_heap_memory)
			kctx->peak_total_tiler_heap_memory = kctx->running_total_tiler_heap_memory;

		/* Assuming at least one chunk reclaimable per heap on (estimated) count */
		atomic_add(PFN_UP(heap->chunk_size), &kctx->csf.tiler_heaps.est_count_pages);
		dev_dbg(kctx->kbdev->dev,
			"Created tiler heap 0x%llX, buffer descriptor 0x%llX, ctx_%d_%d",
			heap->gpu_va, buf_desc_va, kctx->tgid, kctx->id);
		mutex_unlock(&kctx->csf.tiler_heaps.lock);

		return 0;
	}

err_out:
	kfree(heap);
	if (buf_desc_va) {
		kbase_gpu_vm_lock(kctx);
		reg->flags &= ~KBASE_REG_NO_USER_FREE;
		kbase_gpu_vm_unlock(kctx);
	}
	return err;
}

int kbase_csf_tiler_heap_term(struct kbase_context *const kctx,
	u64 const heap_gpu_va)
{
	int err = 0;
	struct kbase_csf_tiler_heap *heap = NULL;
	u32 chunk_count = 0;
	u64 heap_size = 0;

	mutex_lock(&kctx->csf.tiler_heaps.lock);

	heap = find_tiler_heap(kctx, heap_gpu_va);
	if (likely(heap)) {
		chunk_count = heap->chunk_count;
		heap_size = heap->chunk_size * chunk_count;
		delete_heap(heap);
	} else
		err = -EINVAL;

	if (likely(kctx->running_total_tiler_heap_memory >= heap_size))
		kctx->running_total_tiler_heap_memory -= heap_size;
	else
		dev_warn(kctx->kbdev->dev,
			 "Running total tiler heap memory lower than expected!");
	if (likely(kctx->running_total_tiler_heap_nr_chunks >= chunk_count))
		kctx->running_total_tiler_heap_nr_chunks -= chunk_count;
	else
		dev_warn(kctx->kbdev->dev,
			 "Running total tiler chunk count lower than expected!");
	if (!err)
		dev_dbg(kctx->kbdev->dev,
			"Terminated tiler heap 0x%llX, buffer descriptor 0x%llX, ctx_%d_%d",
			heap->gpu_va, heap->buf_desc_va, kctx->tgid, kctx->id);
	mutex_unlock(&kctx->csf.tiler_heaps.lock);
	return err;
}

/**
 * alloc_new_chunk - Allocate a new chunk for the tiler heap.
 *
 * @heap:               Pointer to the tiler heap.
 * @nr_in_flight:       Number of render passes that are in-flight, must not be zero.
 * @pending_frag_count: Number of render passes in-flight with completed vertex/tiler stage.
 *                      The minimum value is zero but it must be less or equal to
 *                      the total number of render passes in flight
 * @new_chunk_ptr:      Where to store the GPU virtual address & size of the new
 *                      chunk allocated for the heap.
 *
 * This function will allocate a new chunk for the chunked tiler heap depending
 * on the settings provided by userspace when the heap was created and the
 * heap's statistics (like number of render passes in-flight).
 *
 * Return: 0 if a new chunk was allocated otherwise an appropriate negative
 *         error code.
 */
static int alloc_new_chunk(struct kbase_csf_tiler_heap *heap,
		u32 nr_in_flight, u32 pending_frag_count, u64 *new_chunk_ptr)
{
	int err = -ENOMEM;

	lockdep_assert_held(&heap->kctx->csf.tiler_heaps.lock);

	if (WARN_ON(!nr_in_flight) ||
		WARN_ON(pending_frag_count > nr_in_flight))
		return -EINVAL;

	if (nr_in_flight <= heap->target_in_flight) {
		if (heap->chunk_count < heap->max_chunks) {
			/* Not exceeded the target number of render passes yet so be
			 * generous with memory.
			 */
			err = create_chunk(heap, false);

			if (likely(!err)) {
				struct kbase_csf_tiler_heap_chunk *new_chunk =
								get_last_chunk(heap);
				if (!WARN_ON(!new_chunk)) {
					*new_chunk_ptr =
						encode_chunk_ptr(heap->chunk_size,
								 new_chunk->gpu_va);
					return 0;
				}
			}
		} else if (pending_frag_count > 0) {
			err = -EBUSY;
		} else {
			err = -ENOMEM;
		}
	} else {
		/* Reached target number of render passes in flight.
		 * Wait for some of them to finish
		 */
		err = -EBUSY;
	}

	return err;
}

int kbase_csf_tiler_heap_alloc_new_chunk(struct kbase_context *kctx,
	u64 gpu_heap_va, u32 nr_in_flight, u32 pending_frag_count, u64 *new_chunk_ptr)
{
	struct kbase_csf_tiler_heap *heap;
	int err = -EINVAL;

	mutex_lock(&kctx->csf.tiler_heaps.lock);

	heap = find_tiler_heap(kctx, gpu_heap_va);

	if (likely(heap)) {
		err = alloc_new_chunk(heap, nr_in_flight, pending_frag_count,
			new_chunk_ptr);
		if (likely(!err)) {
			/* update total and peak tiler heap memory record */
			kctx->running_total_tiler_heap_nr_chunks++;
			kctx->running_total_tiler_heap_memory += heap->chunk_size;

			if (kctx->running_total_tiler_heap_memory >
			    kctx->peak_total_tiler_heap_memory)
				kctx->peak_total_tiler_heap_memory =
					kctx->running_total_tiler_heap_memory;
		}

		KBASE_TLSTREAM_AUX_TILER_HEAP_STATS(
			kctx->kbdev, kctx->id, heap->heap_id,
			PFN_UP(heap->chunk_size * heap->max_chunks),
			PFN_UP(heap->chunk_size * heap->chunk_count),
			heap->max_chunks, heap->chunk_size, heap->chunk_count,
			heap->target_in_flight, nr_in_flight);
	}

	mutex_unlock(&kctx->csf.tiler_heaps.lock);

	return err;
}

static bool delete_chunk_from_gpu_va(struct kbase_csf_tiler_heap *heap, u64 chunk_gpu_va,
				     u64 *hdr_val)
{
	struct kbase_context *kctx = heap->kctx;
	struct kbase_csf_tiler_heap_chunk *chunk;

	list_for_each_entry(chunk, &heap->chunks_list, link) {
		struct kbase_vmap_struct map;
		u64 *chunk_hdr;

		if (chunk->gpu_va != chunk_gpu_va)
			continue;
		/* Found it, extract next chunk header before delete it */
		chunk_hdr = kbase_vmap_prot(kctx, chunk_gpu_va, sizeof(*chunk_hdr),
					    KBASE_REG_CPU_RD, &map);

		if (unlikely(!chunk_hdr)) {
			dev_warn(
				kctx->kbdev->dev,
				"Failed to map tiler heap(0x%llX) chunk(0x%llX) for reclaim extract next header",
				heap->gpu_va, chunk_gpu_va);
			return false;
		}

		*hdr_val = *chunk_hdr;
		kbase_vunmap(kctx, &map);

		dev_dbg(kctx->kbdev->dev,
			"Scan reclaim delete chunk(0x%llx) in heap(0x%llx), header value(0x%llX)",
			chunk_gpu_va, heap->gpu_va, *hdr_val);
		delete_chunk(heap, chunk, true);

		return true;
	}

	dev_warn(kctx->kbdev->dev,
		 "Failed to find tiler heap(0x%llX) chunk(0x%llX) for reclaim-delete", heap->gpu_va,
		 chunk_gpu_va);
	return false;
}

static bool heap_buffer_decsriptor_checked(struct kbase_csf_tiler_heap *const heap)
{
	return heap->desc_chk_flags & HEAP_BUF_DESCRIPTOR_CHECKED;
}

static void sanity_check_gpu_buffer_heap(struct kbase_csf_tiler_heap *heap,
					 struct kbase_csf_gpu_buffer_heap *desc)
{
	u64 ptr_addr = desc->pointer & CHUNK_ADDR_MASK;

	lockdep_assert_held(&heap->kctx->csf.tiler_heaps.lock);

	if (ptr_addr) {
		struct kbase_csf_tiler_heap_chunk *chunk;

		/* desc->pointer must be a chunk in the given heap */
		list_for_each_entry(chunk, &heap->chunks_list, link) {
			if (chunk->gpu_va == ptr_addr) {
				dev_dbg(heap->kctx->kbdev->dev,
					"Buffer descriptor 0x%llX sanity check ok, HW reclaim allowed",
					heap->buf_desc_va);

				heap->desc_chk_flags = HEAP_BUF_DESCRIPTOR_CHECKED;
				return;
			}
		}
	}
	/* If there is no match, defer the check to next time */
	dev_dbg(heap->kctx->kbdev->dev, "Buffer descriptor 0x%llX runtime sanity check deferred",
		heap->buf_desc_va);
}

static bool can_read_hw_gpu_buffer_heap(struct kbase_csf_tiler_heap *heap, u64 *ptr_u64)
{
	struct kbase_context *kctx = heap->kctx;
	bool checked = false;

	lockdep_assert_held(&kctx->csf.tiler_heaps.lock);

	/* Initialize the descriptor pointer value to 0 */
	*ptr_u64 = 0;

	if (heap_buffer_decsriptor_checked(heap))
		return true;

	/* The BufferDescriptor on heap is a hint on creation, do a sanity check at runtime */
	if (heap->buf_desc_va) {
		struct kbase_vmap_struct map;
		struct kbase_csf_gpu_buffer_heap *desc = kbase_vmap_prot(
			kctx, heap->buf_desc_va, sizeof(*desc), KBASE_REG_CPU_RD, &map);

		if (unlikely(!desc)) {
			dev_warn_once(kctx->kbdev->dev,
				      "Sanity check: buffer descriptor 0x%llX map failed",
				      heap->buf_desc_va);
			goto out;
		}

		sanity_check_gpu_buffer_heap(heap, desc);
		checked = heap_buffer_decsriptor_checked(heap);
		if (checked)
			*ptr_u64 = desc->pointer & CHUNK_ADDR_MASK;

		kbase_vunmap(kctx, &map);
	}

out:
	return checked;
}

static u32 delete_hoarded_chunks(struct kbase_csf_tiler_heap *heap)
{
	u32 freed = 0;
	u64 gpu_va = 0;
	struct kbase_context *kctx = heap->kctx;

	lockdep_assert_held(&kctx->csf.tiler_heaps.lock);

	if (can_read_hw_gpu_buffer_heap(heap, &gpu_va)) {
		u64 chunk_hdr_val;
		u64 *hw_hdr;
		struct kbase_vmap_struct map;

		if (!gpu_va) {
			struct kbase_csf_gpu_buffer_heap *desc = kbase_vmap_prot(
				kctx, heap->buf_desc_va, sizeof(*desc), KBASE_REG_CPU_RD, &map);

			if (unlikely(!desc)) {
				dev_warn(
					kctx->kbdev->dev,
					"Failed to map Buffer descriptor 0x%llX for HW reclaim scan",
					heap->buf_desc_va);
				goto out;
			}

			gpu_va = desc->pointer & CHUNK_ADDR_MASK;
			kbase_vunmap(kctx, &map);

			if (!gpu_va) {
				dev_dbg(kctx->kbdev->dev,
					"Buffer descriptor 0x%llX has no chunks (NULL) for reclaim scan",
					heap->buf_desc_va);
				goto out;
			}
		}

		/* Map the HW chunk header here with RD/WR for likely update */
		hw_hdr = kbase_vmap_prot(kctx, gpu_va, sizeof(*hw_hdr),
					 KBASE_REG_CPU_RD | KBASE_REG_CPU_WR, &map);
		if (unlikely(!hw_hdr)) {
			dev_warn(kctx->kbdev->dev,
				 "Failed to map HW chnker header 0x%llX for HW reclaim scan",
				 gpu_va);
			goto out;
		}

		/* Move onto the next chunk relevant information */
		chunk_hdr_val = *hw_hdr;
		gpu_va = chunk_hdr_val & CHUNK_ADDR_MASK;

		while (gpu_va && heap->chunk_count > HEAP_SHRINK_STOP_LIMIT) {
			bool success = delete_chunk_from_gpu_va(heap, gpu_va, &chunk_hdr_val);

			if (!success)
				break;

			freed++;
			/* On success, chunk_hdr_val is updated, extract the next chunk address */
			gpu_va = chunk_hdr_val & CHUNK_ADDR_MASK;
		}

		/* Update the existing hardware chunk header, after reclaim deletion of chunks */
		*hw_hdr = chunk_hdr_val;
		kbase_vunmap(kctx, &map);
		dev_dbg(heap->kctx->kbdev->dev,
			"HW reclaim scan freed chunks: %u, set hw_hdr[0]: 0x%llX", freed,
			chunk_hdr_val);
	} else
		dev_dbg(kctx->kbdev->dev,
			"Skip HW reclaim scan, (disabled: buffer descriptor 0x%llX)",
			heap->buf_desc_va);

out:
	return freed;
}

static u64 delete_unused_chunk_pages(struct kbase_csf_tiler_heap *heap)
{
	u32 freed_chunks = 0;
	u64 freed_pages = 0;
	u64 gpu_va;
	u64 chunk_hdr_val;
	struct kbase_context *kctx = heap->kctx;
	unsigned long prot = KBASE_REG_CPU_RD | KBASE_REG_CPU_WR;
	struct kbase_vmap_struct map;
	u64 *ctx_ptr;

	lockdep_assert_held(&kctx->csf.tiler_heaps.lock);

	ctx_ptr = kbase_vmap_prot(kctx, heap->gpu_va, sizeof(*ctx_ptr), prot, &map);
	if (unlikely(!ctx_ptr)) {
		dev_dbg(kctx->kbdev->dev,
			"Failed to map tiler heap context 0x%llX for reclaim_scan", heap->gpu_va);
		goto out;
	}

	/* Extract the first chunk address from the context's free_list_head */
	chunk_hdr_val = *ctx_ptr;
	gpu_va = chunk_hdr_val & CHUNK_ADDR_MASK;

	while (gpu_va) {
		u64 hdr_val;
		bool success = delete_chunk_from_gpu_va(heap, gpu_va, &hdr_val);

		if (!success)
			break;

		freed_chunks++;
		chunk_hdr_val = hdr_val;
		/* extract the next chunk address */
		gpu_va = chunk_hdr_val & CHUNK_ADDR_MASK;
	}

	/* Update the post-scan deletion to context header */
	*ctx_ptr = chunk_hdr_val;
	kbase_vunmap(kctx, &map);

	/* Try to scan the HW hoarded list of unused chunks */
	freed_chunks += delete_hoarded_chunks(heap);
	freed_pages = freed_chunks * PFN_UP(heap->chunk_size);
	dev_dbg(heap->kctx->kbdev->dev,
		"Scan reclaim freed chunks/pages %u/%llu, set heap-ctx_u64[0]: 0x%llX",
		freed_chunks, freed_pages, chunk_hdr_val);

	/* Update context tiler heaps memory usage */
	kctx->running_total_tiler_heap_memory -= freed_pages << PAGE_SHIFT;
	kctx->running_total_tiler_heap_nr_chunks -= freed_chunks;
out:
	return freed_pages;
}

static u32 scan_kctx_unused_heap_pages_cb(struct kbase_context *kctx, u32 to_free)
{
	u64 freed = 0;
	struct kbase_csf_tiler_heap *heap;

	mutex_lock(&kctx->csf.tiler_heaps.lock);

	list_for_each_entry(heap, &kctx->csf.tiler_heaps.list, link) {
		freed += delete_unused_chunk_pages(heap);
		/* If freed enough, then stop here */
		if (freed >= to_free)
			break;
	}

	mutex_unlock(&kctx->csf.tiler_heaps.lock);
	/* The scan is surely not more than 4-G pages, but for logic flow limit it */
	if (WARN_ON(unlikely(freed > U32_MAX)))
		return U32_MAX;
	else
		return (u32)freed;
}

static u64 count_unused_heap_pages(struct kbase_csf_tiler_heap *heap)
{
	u32 chunk_cnt = 0;
	u64 page_cnt = 0;

	lockdep_assert_held(&heap->kctx->csf.tiler_heaps.lock);

	/* Here the count is basically an informed estimate, avoiding the costly mapping/unmaping
	 * in the chunk list walk. The downside is that the number is a less reliable guide for
	 * later on scan (free) calls on this heap for what actually is freeable.
	 */
	if (heap->chunk_count > HEAP_SHRINK_STOP_LIMIT) {
		chunk_cnt = heap->chunk_count - HEAP_SHRINK_STOP_LIMIT;
		page_cnt = chunk_cnt * PFN_UP(heap->chunk_size);
	}

	dev_dbg(heap->kctx->kbdev->dev,
		"Reclaim count chunks/pages %u/%llu (estimated), heap_va: 0x%llX", chunk_cnt,
		page_cnt, heap->gpu_va);

	return page_cnt;
}

static u32 count_kctx_unused_heap_pages_cb(struct kbase_context *kctx)
{
	u64 page_cnt = 0;
	struct kbase_csf_tiler_heap *heap;

	mutex_lock(&kctx->csf.tiler_heaps.lock);

	list_for_each_entry(heap, &kctx->csf.tiler_heaps.list, link)
		page_cnt += count_unused_heap_pages(heap);

	mutex_unlock(&kctx->csf.tiler_heaps.lock);

	/* The count is surely not more than 4-G pages, but for logic flow limit it */
	if (WARN_ON(unlikely(page_cnt > U32_MAX)))
		return U32_MAX;
	else
		return (u32)page_cnt;
}

static unsigned long kbase_csf_tiler_heap_reclaim_count_objects(struct shrinker *s,
								struct shrink_control *sc)
{
	struct kbase_device *kbdev = container_of(s, struct kbase_device, csf.tiler_heap_reclaim);
	struct kbase_csf_tiler_heap_shrink_control shrink_ctrl = {
		.sc = sc,
		.count_cb = count_kctx_unused_heap_pages_cb,
		.scan_cb = scan_kctx_unused_heap_pages_cb,
	};

	return kbase_csf_scheduler_count_free_heap_pages(kbdev, &shrink_ctrl);
}

static unsigned long kbase_csf_tiler_heap_reclaim_scan_objects(struct shrinker *s,
							       struct shrink_control *sc)
{
	struct kbase_device *kbdev = container_of(s, struct kbase_device, csf.tiler_heap_reclaim);
	struct kbase_csf_tiler_heap_shrink_control shrink_ctrl = {
		.sc = sc,
		.count_cb = count_kctx_unused_heap_pages_cb,
		.scan_cb = scan_kctx_unused_heap_pages_cb,
	};

	return kbase_csf_scheduler_scan_free_heap_pages(kbdev, &shrink_ctrl);
}

void kbase_csf_tiler_heap_register_shrinker(struct kbase_device *kbdev)
{
	struct shrinker *reclaim = &kbdev->csf.tiler_heap_reclaim;

	reclaim->count_objects = kbase_csf_tiler_heap_reclaim_count_objects;
	reclaim->scan_objects = kbase_csf_tiler_heap_reclaim_scan_objects;
	reclaim->seeks = HEAP_SHRINKER_SEEKS;
	reclaim->batch = HEAP_SHRINKER_BATCH;

	register_shrinker(reclaim);
}

void kbase_csf_tiler_heap_unregister_shrinker(struct kbase_device *kbdev)
{
	unregister_shrinker(&kbdev->csf.tiler_heap_reclaim);
}
