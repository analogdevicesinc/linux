/* SPDX-License-Identifier: GPL-2.0 */
/* Implementation of a segmented queue of bio_vec[].
 *
 * Copyright (C) 2026 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 */

#ifndef _LINUX_BVECQ_H
#define _LINUX_BVECQ_H

#include <linux/bvec.h>

/*
 * The type of memory retention used by the elements in bvecq->bv[] and how to
 * clean it up.
 */
enum bvecq_mem {
	BVECQ_MEM_EXTERNAL,	/* Externally retained memory - no freeing */
	BVECQ_MEM_PAGECACHE,	/* Ref'd pagecache pages - must put */
	BVECQ_MEM_GUP,		/* Pinned memory from get_user_pages() - unpin */
	BVECQ_MEM_ALLOCED,	/* Memory alloc'd by bvecq - can be freed/pooled */
} __mode(byte);

/*
 * Segmented bio_vec queue.
 *
 * These can be linked together to form messages of indefinite length and
 * iterated over with an ITER_BVECQ iterator.  The list is non-circular; next
 * and prev are NULL at the ends.
 *
 * The bv pointer points to the bio_vec array; this may be __bv if allocated
 * together.  The caller is responsible for determining whether or not this is
 * the case as the array pointed to by bv may be follow on directly from the
 * bvecq by accident of allocation (ie. ->bv == ->__bv is *not* sufficient to
 * determine this).
 */
struct bvecq {
	struct bvecq	*next;		/* Next bvec in the list or NULL */
	struct bvecq	*prev;		/* Prev bvec in the list or NULL */
	refcount_t	ref;
	u32		priv;		/* Private data */
	u16		nr_slots;	/* Number of elements in bv[] used */
	u16		max_slots;	/* Number of elements allocated in bv[] */
	enum bvecq_mem	mem_type:3;	/* What sort of memory and how to free it */
	bool		inline_bv:1;	/* T if __bv[] is being used */
	bool		from_pool:1;	/* T if bvecq from mempool */
	struct bio_vec	*bv;		/* Pointer to array of page fragments */
	struct bio_vec	__bv[];		/* Default array (if ->inline_bv) */
};

/* Number of slots in a 512-byte mempool-backed bvecq. */
#define BVECQ_POOL_SLOTS ((512 - sizeof(struct bvecq)) / sizeof(struct bio_vec))

/* Number of slots in a 4K bvecq. */
#define BVECQ_4KB_SLOTS  ((4096 - sizeof(struct bvecq)) / sizeof(struct bio_vec))

void bvecq_dump(const struct bvecq *bq);
struct bvecq *bvecq_alloc_one(size_t nr_slots, gfp_t gfp, bool for_writeback);
struct bvecq *bvecq_alloc_chain(size_t nr_slots, gfp_t gfp, bool for_writeback);
struct bvecq *bvecq_alloc_buffer2(size_t size, unsigned int pre_slots, gfp_t gfp,
				  bool for_writeback);
void bvecq_put(struct bvecq *bq);
int bvecq_expand_buffer(struct bvecq **_buffer, size_t *_cur_size, size_t size, gfp_t gfp);

/**
 * bvecq_alloc_buffer - Allocate a bvecq chain and populate with buffers
 * @size: Target size of the buffer (can be 0 for an empty buffer)
 * @gfp: The allocation constraints.
 * @for_writeback: True if allocating for writeback
 *
 * Wrapper around %bvecq_alloc_buffer2().
 */
static inline struct bvecq *bvecq_alloc_buffer(size_t size, gfp_t gfp, bool for_writeback)
{
	return bvecq_alloc_buffer2(size, 0, gfp, for_writeback);
}

/**
 * bvecq_get - Get a ref on a bvecq
 * @bq: The bvecq to get a ref on
 */
static inline struct bvecq *bvecq_get(struct bvecq *bq)
{
	refcount_inc(&bq->ref);
	return bq;
}

/**
 * bvecq_is_full - Determine if a bvecq is full
 * @bvecq: The object to query
 *
 * Return: true if full; false if not.
 */
static inline bool bvecq_is_full(const struct bvecq *bvecq)
{
	return bvecq->nr_slots >= bvecq->max_slots;
}

/**
 * bvecq_filled_to - Release filled slots with release barrier
 * @bvecq: The object modified
 * @to: The latest slot filled + 1
 */
static inline void bvecq_filled_to(struct bvecq *bvecq, unsigned int to)
{
	/* Set the slot counter after filling the slot */
	smp_store_release(&bvecq->nr_slots, to);
}

/**
 * bvecq_nr_slots_acquire - Get the number of filled slots with acquire barrier
 * @bvecq: The object to query
 *
 * Return: The number of filled slots
 */
static inline unsigned int bvecq_nr_slots_acquire(const struct bvecq *bvecq)
{
	/* Read the slot counter before looking at the slot */
	return smp_load_acquire(&bvecq->nr_slots);
}

/**
 * bvecq_acquire_slot - Determine if a slot is valid with acquire barrier
 * @bvecq: The object to query
 * @slot: The next slot
 *
 * Return: true if valid; false if might not be valid
 */
static inline bool bvecq_acquire_slot(const struct bvecq *bvecq, unsigned int slot)
{
	/* Read the slot counter before looking at the slot */
	return slot < bvecq_nr_slots_acquire(bvecq);
}

/**
 * bvecq_append - Get the next bvecq with appropriate barrier
 * @to: The bvecq to append to
 * @add: The bvecq to append
 *
 * Attach a new bvecq to a chain using an appropriate barrier to protect the
 * write.
 *
 * [!] Note that this function transfers the caller's ref to the chain.
 */
static inline void bvecq_append(struct bvecq *to, struct bvecq *add)
{
	add->prev = to;

	/* Make sure the initialisation is stored before the next pointer. */
	smp_store_release(&to->next, add);
}

/**
 * bvecq_next - Get the next bvecq with appropriate barrier
 * @bq: The bvecq to start from
 *
 * Return the next bvecq in a chain, using an appropriate barrier to protect
 * the access.
 */
static inline struct bvecq *bvecq_next(const struct bvecq *bq)
{
	/* Read the contents of the next node after the pointer to it. */
	return smp_load_acquire(&bq->next);
}

#endif /* _LINUX_BVECQ_H */
