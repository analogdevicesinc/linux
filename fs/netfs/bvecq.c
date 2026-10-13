// SPDX-License-Identifier: GPL-2.0-only
/* Buffering helpers for bvec queues
 *
 * Copyright (C) 2026 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 */

#include <linux/bvecq.h>
#include "internal.h"

void bvecq_dump(const struct bvecq *bq)
{
	int b = 0;

	for (; bq; bq = bvecq_next(bq), b++) {
		int skipz = 0;

		pr_notice("BQ[%u] %u/%u\n", b, bq->nr_slots, bq->max_slots);
		for (int s = 0; s < bq->nr_slots; s++) {
			const struct bio_vec *bv = &bq->bv[s];

			if (!bv->bv_page && !bv->bv_len && skipz < 2) {
				skipz = 1;
				continue;
			}
			if (skipz == 1)
				pr_notice("BQ[%u:00-%02u] ...\n", b, s - 1);
			skipz = 2;
			pr_notice("BQ[%u:%02u] %10lx %04x %04x %u\n",
				  b, s,
				  bv->bv_page ? page_to_pfn(bv->bv_page) : 0,
				  bv->bv_offset, bv->bv_len,
				  bv->bv_page ? page_count(bv->bv_page) : 0);
		}
	}
}
EXPORT_SYMBOL(bvecq_dump);

/**
 * bvecq_alloc_one - Allocate a single bvecq node with unpopulated slots
 * @nr_slots: Number of slots to allocate
 * @gfp: The allocation constraints.
 * @for_writeback: True if allocating for writeback
 *
 * Allocate a single bvecq node and initialise the header.  The allocation is
 * rounded up to the size of the smallest slab granule that will accommodate it
 * and all the remaining space is set as an inline slot array with max_slots
 * set to the number of slots.  The slot array is not initialised.
 *
 * If @for_writeback is set, then the emergency mempool may be used for
 * allocation.  If it does allocate from that pool, the maximum number of slots
 * available will be BVECQ_POOL_SLOTS which may be less than requested.  Also,
 * if @for_writeback is set, the function will not fail - though it may have to
 * wait.
 *
 * Return: The node pointer or NULL on allocation failure.
 */
struct bvecq *bvecq_alloc_one(size_t nr_slots, gfp_t gfp, bool for_writeback)
{
	struct bvecq *bq;
	const size_t pool_size = struct_size_t(struct bvecq, __bv, BVECQ_POOL_SLOTS);
	size_t size;
	bool from_pool = false;

	size = kmalloc_size_roundup(struct_size(bq, __bv, nr_slots));
	gfp &= ~(GFP_ZONEMASK | __GFP_THISNODE);

	if (for_writeback) {
		if (size != pool_size) {
			gfp_t gfp_temp = gfp;

			gfp_temp |= __GFP_NOMEMALLOC | __GFP_NORETRY | __GFP_NOWARN;
			gfp_temp &= ~(__GFP_DIRECT_RECLAIM | __GFP_IO);
			bq = kmalloc(size, gfp_temp);
			if (bq)
				goto success;
		}

		bq = mempool_alloc(&netfs_bvecq_pool, gfp);
		if (!bq)
			return bq;
		from_pool = true;
		size = pool_size;
	} else {
		bq = kmalloc(size, gfp);
		if (!bq)
			return bq;
	}

success:
	*bq = (struct bvecq) {
		.ref		= REFCOUNT_INIT(1),
		.bv		= bq->__bv,
		.inline_bv	= true,
		.max_slots	= (size - sizeof(*bq)) / sizeof(bq->__bv[0]),
		.from_pool	= from_pool,
	};
	netfs_stat(&netfs_n_bvecq);
	return bq;
}
EXPORT_SYMBOL(bvecq_alloc_one);

/**
 * bvecq_alloc_chain - Allocate an unpopulated bvecq chain
 * @nr_slots: Number of slots to allocate
 * @gfp: The allocation constraints.
 * @for_writeback: True if allocating for writeback
 *
 * Allocate a chain of bvecq nodes providing at least the requested cumulative
 * number of slots.  Each node is a maximum of 4KiB in size.
 *
 * Return: The first node pointer or NULL on allocation failure.
 */
struct bvecq *bvecq_alloc_chain(size_t nr_slots, gfp_t gfp, bool for_writeback)
{
	struct bvecq *head = NULL, *tail = NULL;

	_enter("%zu", nr_slots);

	for (;;) {
		struct bvecq *bq;

		bq = bvecq_alloc_one(min(nr_slots, BVECQ_4KB_SLOTS), gfp, for_writeback);
		if (!bq)
			goto oom;

		if (tail)
			bvecq_append(tail, bq);
		else
			head = bq;
		tail = bq;
		if (tail->max_slots >= nr_slots)
			break;
		nr_slots -= tail->max_slots;
	}

	return head;
oom:
	bvecq_put(head);
	return NULL;
}
EXPORT_SYMBOL(bvecq_alloc_chain);

/**
 * bvecq_alloc_buffer2 - Allocate a bvecq chain and populate with buffers
 * @size: Target size of the buffer (can be 0 for an empty buffer)
 * @pre_slots: Number of preamble slots to set aside
 * @gfp: The allocation constraints.
 * @for_writeback: True if allocating for writeback
 *
 * Allocate a chain of bvecq nodes and populate the slots with sufficient pages
 * to provide at least the requested amount of space, leaving the first
 * @pre_slots slots unset.  The pre-slots must all fit into the the first
 * bvecq.
 *
 * The pages allocated may be compound pages larger than PAGE_SIZE and thus
 * occupy fewer slots.  The pages have their refcounts set to 1 and can be
 * passed to MSG_SPLICE_PAGES.
 *
 * Return: The first node pointer or NULL on allocation failure.
 */
struct bvecq *bvecq_alloc_buffer2(size_t size, unsigned int pre_slots, gfp_t gfp,
				  bool for_writeback)
{
	struct bvecq *head = NULL, *p = NULL;
	size_t nr_per_bq = BVECQ_POOL_SLOTS;
	size_t count = pre_slots + DIV_ROUND_UP(size, PAGE_SIZE);

	_enter("%zx,%zx,%u", size, count, pre_slots);

	if (WARN_ON_ONCE(pre_slots > nr_per_bq))
		return NULL;

	head = bvecq_alloc_chain(count, gfp, for_writeback);
	if (!head)
		return NULL;

	p = head;
	do {
		struct page **pages;
		size_t unused, want, got, slot;

		if (!count)
			break;
		if (WARN_ON_ONCE(!p))
			goto oom;

		if (p->nr_slots == 0) {
			/* Need to clear pre slots and pages[], so just clear all. */
			memset(p->bv, 0, p->max_slots * sizeof(p->bv[0]));
			p->mem_type = BVECQ_MEM_ALLOCED;
			p->nr_slots = pre_slots;
			count -= pre_slots;
			pre_slots = 0;
			if (!count)
				break;
		}

		if (p->nr_slots >= p->max_slots) {
			p = p->next;
			continue;
		}
		unused = p->max_slots - p->nr_slots;

		pages = (struct page **)&p->bv[p->max_slots];
		pages -= unused;

		want = min(count, unused);
		got = alloc_pages_bulk(gfp, want, pages);
		if (!got)
			goto oom;

		slot = p->nr_slots;
		for (int i = 0; i < got; i++)
			bvec_set_page(&p->bv[slot++], pages[i], PAGE_SIZE, 0);

		bvecq_filled_to(p, slot);
		count -= got;
	} while (count > 0);

	return head;
oom:
	bvecq_put(head);
	return NULL;
}
EXPORT_SYMBOL(bvecq_alloc_buffer2);

/*
 * Free the page pointed to by a slot as necessary.
 */
static void bvecq_free_slot(struct bvecq *bq, unsigned int slot)
{
	struct page *page = bq->bv[slot].bv_page;

	if (!page)
		return;

	switch (bq->mem_type) {
	case BVECQ_MEM_EXTERNAL:
		break;
	case BVECQ_MEM_PAGECACHE:
		put_page(page);
		break;
	case BVECQ_MEM_GUP:
		unpin_user_page(page);
		break;
	case BVECQ_MEM_ALLOCED:
		__free_pages(page, compound_order(page));
		break;
	default:
		WARN_ON_ONCE(1);
		break;
	}
}

/**
 * bvecq_put - Put a ref on a bvec queue
 * @bq: The start of the folio queue to free
 *
 * Put the ref(s) on the nodes in a bvec queue, freeing up the node and the
 * page fragments it points to as the refcounts become zero.
 */
void bvecq_put(struct bvecq *bq)
{
	struct bvecq *next;

	for (; bq; bq = next) {
		if (!refcount_dec_and_test(&bq->ref))
			break;
		for (int slot = 0; slot < bq->nr_slots; slot++)
			bvecq_free_slot(bq, slot);
		next = bq->next;
		netfs_stat_d(&netfs_n_bvecq);
		if (bq->from_pool)
			mempool_free(bq, &netfs_bvecq_pool);
		else
			kfree(bq);
	}
}
EXPORT_SYMBOL(bvecq_put);

/**
 * bvecq_expand_buffer - Allocate buffer space into a bvec queue
 * @_buffer: Pointer to the bvecq chain to expand (may point to a NULL; updated).
 * @_cur_size: Current size of the buffer (updated).
 * @size: Target size of the buffer.
 * @gfp: The allocation constraints.
 *
 * Append extra pages to a buffer to increase its capacity to the @size
 * specified.  If the current tail has space, but is not of the
 * BVECQ_MEM_ALLOCED memory type, a separate bvecq will be allocated to hold
 * the new memory.
 */
int bvecq_expand_buffer(struct bvecq **_buffer, size_t *_cur_size, size_t size, gfp_t gfp)
{
	struct bvecq *tail = *_buffer;

	size = round_up(size, PAGE_SIZE);
	if (tail)
		while (tail->next)
			tail = tail->next;

	while (*_cur_size < size) {
		struct page *page;
		size_t need = size - *_cur_size;
		gfp_t gfp_add;
		int order = 0;

		if (!tail || bvecq_is_full(tail) || tail->mem_type != BVECQ_MEM_ALLOCED) {
			struct bvecq *p;

			p = bvecq_alloc_one(BVECQ_POOL_SLOTS, gfp, false);
			if (!p)
				return -ENOMEM;
			if (tail)
				bvecq_append(tail, p);
			else
				*_buffer = p;
			tail = p;
			p->mem_type = BVECQ_MEM_ALLOCED;
		}

		if (need > PAGE_SIZE)
			order = umin(ilog2(need) - PAGE_SHIFT, MAX_PAGECACHE_ORDER);

		gfp_add = 0;
		if (order > 0)
			gfp_add |= __GFP_NORETRY | __GFP_NOWARN;
		page = alloc_pages(gfp | __GFP_COMP | gfp_add, order);
		if (!page && order > 0) {
			page = alloc_pages(gfp | __GFP_COMP, 0);
			order = 0;
		}
		if (!page)
			return -ENOMEM;

		bvec_set_page(&tail->bv[tail->nr_slots], page, PAGE_SIZE << order, 0);
		*_cur_size += PAGE_SIZE << order;
		bvecq_filled_to(tail, tail->nr_slots + 1);
	}

	return 0;
}
EXPORT_SYMBOL(bvecq_expand_buffer);
