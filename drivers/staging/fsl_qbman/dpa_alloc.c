/* Copyright 2009-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dpa_sys.h"
#include <linux/fsl_qman.h>
#include <linux/fsl_bman.h>

/* Qman and Bman APIs are front-ends to the common code; */

static DECLARE_DPA_ALLOC(bpalloc); /* BPID allocator */
static DECLARE_DPA_ALLOC(fqalloc); /* FQID allocator */
static DECLARE_DPA_ALLOC(qpalloc); /* pool-channel allocator */
static DECLARE_DPA_ALLOC(cgralloc); /* CGR ID allocator */
static DECLARE_DPA_ALLOC(ceetm0_challoc); /* CEETM Channel ID allocator */
static DECLARE_DPA_ALLOC(ceetm0_lfqidalloc); /* CEETM LFQID allocator */
static DECLARE_DPA_ALLOC(ceetm1_challoc); /* CEETM Channel ID allocator */
static DECLARE_DPA_ALLOC(ceetm1_lfqidalloc); /* CEETM LFQID allocator */

/* This is a sort-of-conditional dpa_alloc_free() routine. Eg. when releasing
 * FQIDs (probably from user-space), it can filter out those that aren't in the
 * OOS state (better to leak a h/w resource than to crash). This function
 * returns the number of invalid IDs that were not released. */
static u32 release_id_range(struct dpa_alloc *alloc, u32 id, u32 count,
			     int (*is_valid)(u32 id))
{
	int valid_mode = 0;
	u32 loop = id, total_invalid = 0;
	while (loop < (id + count)) {
		int isvalid = is_valid ? is_valid(loop) : 1;
		if (!valid_mode) {
			/* We're looking for a valid ID to terminate an invalid
			 * range */
			if (isvalid) {
				/* We finished a range of invalid IDs, a valid
				 * range is now underway */
				valid_mode = 1;
				count -= (loop - id);
				id = loop;
			} else
				total_invalid++;
		} else {
			/* We're looking for an invalid ID to terminate a
			 * valid range */
			if (!isvalid) {
				/* Release the range of valid IDs, an unvalid
				 * range is now underway */
				if (loop > id)
					dpa_alloc_free(alloc, id, loop - id);
				valid_mode = 0;
			}
		}
		loop++;
	}
	/* Release any unterminated range of valid IDs */
	if (valid_mode && count)
		dpa_alloc_free(alloc, id, count);
	return total_invalid;
}

/* BPID allocator front-end */

int bman_alloc_bpid_range(u32 *result, u32 count, u32 align, int partial)
{
	return dpa_alloc_new(&bpalloc, result, count, align, partial);
}
EXPORT_SYMBOL(bman_alloc_bpid_range);

static int bp_cleanup(u32 bpid)
{
	return bman_shutdown_pool(bpid) == 0;
}
void bman_release_bpid_range(u32 bpid, u32 count)
{
	u32 total_invalid = release_id_range(&bpalloc, bpid, count, bp_cleanup);
	if (total_invalid)
		pr_err("BPID range [%d..%d] (%d) had %d leaks\n",
			bpid, bpid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(bman_release_bpid_range);

void bman_seed_bpid_range(u32 bpid, u32 count)
{
	dpa_alloc_seed(&bpalloc, bpid, count);
}
EXPORT_SYMBOL(bman_seed_bpid_range);

int bman_reserve_bpid_range(u32 bpid, u32 count)
{
	return dpa_alloc_reserve(&bpalloc, bpid, count);
}
EXPORT_SYMBOL(bman_reserve_bpid_range);


/* FQID allocator front-end */

int qman_alloc_fqid_range(u32 *result, u32 count, u32 align, int partial)
{
	return dpa_alloc_new(&fqalloc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_fqid_range);

static int fq_cleanup(u32 fqid)
{
	return qman_shutdown_fq(fqid) == 0;
}
void qman_release_fqid_range(u32 fqid, u32 count)
{
	u32 total_invalid = release_id_range(&fqalloc, fqid, count, fq_cleanup);
	if (total_invalid)
		pr_err("FQID range [%d..%d] (%d) had %d leaks\n",
			fqid, fqid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(qman_release_fqid_range);

int qman_reserve_fqid_range(u32 fqid, u32 count)
{
	return dpa_alloc_reserve(&fqalloc, fqid, count);
}
EXPORT_SYMBOL(qman_reserve_fqid_range);

void qman_seed_fqid_range(u32 fqid, u32 count)
{
	dpa_alloc_seed(&fqalloc, fqid, count);
}
EXPORT_SYMBOL(qman_seed_fqid_range);

/* Pool-channel allocator front-end */

int qman_alloc_pool_range(u32 *result, u32 count, u32 align, int partial)
{
	return dpa_alloc_new(&qpalloc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_pool_range);

static int qpool_cleanup(u32 qp)
{
	/* We query all FQDs starting from
	 * FQID 1 until we get an "invalid FQID" error, looking for non-OOS FQDs
	 * whose destination channel is the pool-channel being released.
	 * When a non-OOS FQD is found we attempt to clean it up */
	struct qman_fq fq = {
		.fqid = 1
	};
	int err;
	do {
		struct qm_mcr_queryfq_np np;
		err = qman_query_fq_np(&fq, &np);
		if (err)
			/* FQID range exceeded, found no problems */
			return 1;
		if ((np.state & QM_MCR_NP_STATE_MASK) != QM_MCR_NP_STATE_OOS) {
			struct qm_fqd fqd;
			err = qman_query_fq(&fq, &fqd);
			BUG_ON(err);
			if (fqd.dest.channel == qp) {
				/* The channel is the FQ's target, clean it */
				if (qman_shutdown_fq(fq.fqid) != 0)
					/* Couldn't shut down the FQ
					   so the pool must be leaked */
					return 0;
			}
		}
		/* Move to the next FQID */
		fq.fqid++;
	} while (1);
}
void qman_release_pool_range(u32 qp, u32 count)
{
	u32 total_invalid = release_id_range(&qpalloc, qp,
					     count, qpool_cleanup);
	if (total_invalid) {
		/* Pool channels are almost always used individually */
		if (count == 1)
			pr_err("Pool channel 0x%x had %d leaks\n",
				qp, total_invalid);
		else
			pr_err("Pool channels [%d..%d] (%d) had %d leaks\n",
				qp, qp + count - 1, count, total_invalid);
	}
}
EXPORT_SYMBOL(qman_release_pool_range);


void qman_seed_pool_range(u32 poolid, u32 count)
{
	dpa_alloc_seed(&qpalloc, poolid, count);

}
EXPORT_SYMBOL(qman_seed_pool_range);

int qman_reserve_pool_range(u32 poolid, u32 count)
{
	return dpa_alloc_reserve(&qpalloc, poolid, count);
}
EXPORT_SYMBOL(qman_reserve_pool_range);


/* CGR ID allocator front-end */

int qman_alloc_cgrid_range(u32 *result, u32 count, u32 align, int partial)
{
	return dpa_alloc_new(&cgralloc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_cgrid_range);

static int cqr_cleanup(u32 cgrid)
{
	/* We query all FQDs starting from
	 * FQID 1 until we get an "invalid FQID" error, looking for non-OOS FQDs
	 * whose CGR is the CGR being released.
	 */
	struct qman_fq fq = {
		.fqid = 1
	};
	int err;
	do {
		struct qm_mcr_queryfq_np np;
		err = qman_query_fq_np(&fq, &np);
		if (err)
			/* FQID range exceeded, found no problems */
			return 1;
		if ((np.state & QM_MCR_NP_STATE_MASK) != QM_MCR_NP_STATE_OOS) {
			struct qm_fqd fqd;
			err = qman_query_fq(&fq, &fqd);
			BUG_ON(err);
			if ((fqd.fq_ctrl & QM_FQCTRL_CGE) &&
			    (fqd.cgid == cgrid)) {
				pr_err("CRGID 0x%x is being used by FQID 0x%x,"
				       " CGR will be leaked\n",
				       cgrid, fq.fqid);
				return 1;
			}
		}
		/* Move to the next FQID */
		fq.fqid++;
	} while (1);
}

void qman_release_cgrid_range(u32 cgrid, u32 count)
{
	u32 total_invalid = release_id_range(&cgralloc, cgrid,
					     count, cqr_cleanup);
	if (total_invalid)
		pr_err("CGRID range [%d..%d] (%d) had %d leaks\n",
			cgrid, cgrid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(qman_release_cgrid_range);

void qman_seed_cgrid_range(u32 cgrid, u32 count)
{
	dpa_alloc_seed(&cgralloc, cgrid, count);

}
EXPORT_SYMBOL(qman_seed_cgrid_range);

/* CEETM CHANNEL ID allocator front-end */
int qman_alloc_ceetm0_channel_range(u32 *result, u32 count, u32 align,
								 int partial)
{
	return dpa_alloc_new(&ceetm0_challoc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_ceetm0_channel_range);

int qman_alloc_ceetm1_channel_range(u32 *result, u32 count, u32 align,
								 int partial)
{
	return dpa_alloc_new(&ceetm1_challoc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_ceetm1_channel_range);

void qman_release_ceetm0_channel_range(u32 channelid, u32 count)
{
	u32 total_invalid;

	total_invalid = release_id_range(&ceetm0_challoc, channelid, count,
									 NULL);
	if (total_invalid)
		pr_err("CEETM channel range [%d..%d] (%d) had %d leaks\n",
			channelid, channelid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(qman_release_ceetm0_channel_range);

void qman_seed_ceetm0_channel_range(u32 channelid, u32 count)
{
	dpa_alloc_seed(&ceetm0_challoc, channelid, count);

}
EXPORT_SYMBOL(qman_seed_ceetm0_channel_range);

void qman_release_ceetm1_channel_range(u32 channelid, u32 count)
{
	u32 total_invalid;
	total_invalid = release_id_range(&ceetm1_challoc, channelid, count,
									 NULL);
	if (total_invalid)
		pr_err("CEETM channel range [%d..%d] (%d) had %d leaks\n",
			channelid, channelid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(qman_release_ceetm1_channel_range);

void qman_seed_ceetm1_channel_range(u32 channelid, u32 count)
{
	dpa_alloc_seed(&ceetm1_challoc, channelid, count);

}
EXPORT_SYMBOL(qman_seed_ceetm1_channel_range);

/* CEETM LFQID allocator front-end */
int qman_alloc_ceetm0_lfqid_range(u32 *result, u32 count, u32 align,
								 int partial)
{
	return dpa_alloc_new(&ceetm0_lfqidalloc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_ceetm0_lfqid_range);

int qman_alloc_ceetm1_lfqid_range(u32 *result, u32 count, u32 align,
								 int partial)
{
	return dpa_alloc_new(&ceetm1_lfqidalloc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_ceetm1_lfqid_range);

void qman_release_ceetm0_lfqid_range(u32 lfqid, u32 count)
{
	u32 total_invalid;

	total_invalid = release_id_range(&ceetm0_lfqidalloc, lfqid, count,
									NULL);
	if (total_invalid)
		pr_err("CEETM LFQID range [0x%x..0x%x] (%d) had %d leaks\n",
			lfqid, lfqid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(qman_release_ceetm0_lfqid_range);

void qman_seed_ceetm0_lfqid_range(u32 lfqid, u32 count)
{
	dpa_alloc_seed(&ceetm0_lfqidalloc, lfqid, count);

}
EXPORT_SYMBOL(qman_seed_ceetm0_lfqid_range);

void qman_release_ceetm1_lfqid_range(u32 lfqid, u32 count)
{
	u32 total_invalid;

	total_invalid = release_id_range(&ceetm1_lfqidalloc, lfqid, count,
									NULL);
	if (total_invalid)
		pr_err("CEETM LFQID range [0x%x..0x%x] (%d) had %d leaks\n",
			lfqid, lfqid + count - 1, count, total_invalid);
}
EXPORT_SYMBOL(qman_release_ceetm1_lfqid_range);

void qman_seed_ceetm1_lfqid_range(u32 lfqid, u32 count)
{
	dpa_alloc_seed(&ceetm1_lfqidalloc, lfqid, count);

}
EXPORT_SYMBOL(qman_seed_ceetm1_lfqid_range);


/* Everything else is the common backend to all the allocators */

/* The allocator is a (possibly-empty) list of these; */
struct alloc_node {
	struct list_head list;
	u32 base;
	u32 num;
	/* refcount and is_alloced are only set
	   when the node is in the used list */
	unsigned int refcount;
	int is_alloced;
};

/* #define DPA_ALLOC_DEBUG */

#ifdef DPA_ALLOC_DEBUG
#define DPRINT pr_info
static void DUMP(struct dpa_alloc *alloc)
{
	int off = 0;
	char buf[256];
	struct alloc_node *p;
	pr_info("Free Nodes\n");
	list_for_each_entry(p, &alloc->free, list) {
		if (off < 255)
			off += snprintf(buf + off, 255-off, "{%d,%d}",
				p->base, p->base + p->num - 1);
	}
	pr_info("%s\n", buf);

	off = 0;
	pr_info("Used Nodes\n");
	list_for_each_entry(p, &alloc->used, list) {
		if (off < 255)
			off += snprintf(buf + off, 255-off, "{%d,%d}",
				p->base, p->base + p->num - 1);
	}
	pr_info("%s\n", buf);



}
#else
#define DPRINT(x...)
#define DUMP(a)
#endif

int dpa_alloc_new(struct dpa_alloc *alloc, u32 *result, u32 count, u32 align,
		  int partial)
{
	struct alloc_node *i = NULL, *next_best = NULL, *used_node = NULL;
	u32 base, next_best_base = 0, num = 0, next_best_num = 0;
	struct alloc_node *margin_left, *margin_right;

	*result = (u32)-1;
	DPRINT("alloc_range(%d,%d,%d)\n", count, align, partial);
	DUMP(alloc);
	/* If 'align' is 0, it should behave as though it was 1 */
	if (!align)
		align = 1;
	margin_left = kmalloc(sizeof(*margin_left), GFP_KERNEL);
	if (!margin_left)
		goto err;
	margin_right = kmalloc(sizeof(*margin_right), GFP_KERNEL);
	if (!margin_right) {
		kfree(margin_left);
		goto err;
	}
	spin_lock_irq(&alloc->lock);
	list_for_each_entry(i, &alloc->free, list) {
		base = (i->base + align - 1) / align;
		base *= align;
		if ((base - i->base) >= i->num)
			/* alignment is impossible, regardless of count */
			continue;
		num = i->num - (base - i->base);
		if (num >= count) {
			/* this one will do nicely */
			num = count;
			goto done;
		}
		if (num > next_best_num) {
			next_best = i;
			next_best_base = base;
			next_best_num = num;
		}
	}
	if (partial && next_best) {
		i = next_best;
		base = next_best_base;
		num = next_best_num;
	} else
		i = NULL;
done:
	if (i) {
		if (base != i->base) {
			margin_left->base = i->base;
			margin_left->num = base - i->base;
			list_add_tail(&margin_left->list, &i->list);
		} else
			kfree(margin_left);
		if ((base + num) < (i->base + i->num)) {
			margin_right->base = base + num;
			margin_right->num = (i->base + i->num) -
						(base + num);
			list_add(&margin_right->list, &i->list);
		} else
			kfree(margin_right);
		list_del(&i->list);
		kfree(i);
		*result = base;
	} else {
		spin_unlock_irq(&alloc->lock);
		kfree(margin_left);
		kfree(margin_right);
	}

err:
	DPRINT("returning %d\n", i ? num : -ENOMEM);
	DUMP(alloc);
	if (!i)
		return -ENOMEM;

	/* Add the allocation to the used list with a refcount of 1 */
	used_node = kmalloc(sizeof(*used_node), GFP_KERNEL);
	if (!used_node) {
		spin_unlock_irq(&alloc->lock);
		return -ENOMEM;
	}
	used_node->base = *result;
	used_node->num = num;
	used_node->refcount = 1;
	used_node->is_alloced = 1;
	list_add_tail(&used_node->list, &alloc->used);
	spin_unlock_irq(&alloc->lock);
	return (int)num;
}

/* Allocate the list node using GFP_ATOMIC, because we *really* want to avoid
 * forcing error-handling on to users in the deallocation path. */
static void _dpa_alloc_free(struct dpa_alloc *alloc, u32 base_id, u32 count)
{
	struct alloc_node *i, *node = kmalloc(sizeof(*node), GFP_ATOMIC);
	BUG_ON(!node);
	DPRINT("release_range(%d,%d)\n", base_id, count);
	DUMP(alloc);
	BUG_ON(!count);
	spin_lock_irq(&alloc->lock);


	node->base = base_id;
	node->num = count;
	list_for_each_entry(i, &alloc->free, list) {
		if (i->base >= node->base) {
			/* BUG_ON(any overlapping) */
			BUG_ON(i->base < (node->base + node->num));
			list_add_tail(&node->list, &i->list);
			goto done;
		}
	}
	list_add_tail(&node->list, &alloc->free);
done:
	/* Merge to the left */
	i = list_entry(node->list.prev, struct alloc_node, list);
	if (node->list.prev != &alloc->free) {
		BUG_ON((i->base + i->num) > node->base);
		if ((i->base + i->num) == node->base) {
			node->base = i->base;
			node->num += i->num;
			list_del(&i->list);
			kfree(i);
		}
	}
	/* Merge to the right */
	i = list_entry(node->list.next, struct alloc_node, list);
	if (node->list.next != &alloc->free) {
		BUG_ON((node->base + node->num) > i->base);
		if ((node->base + node->num) == i->base) {
			node->num += i->num;
			list_del(&i->list);
			kfree(i);
		}
	}
	spin_unlock_irq(&alloc->lock);
	DUMP(alloc);
}


void dpa_alloc_free(struct dpa_alloc *alloc, u32 base_id, u32 count)
{
	struct alloc_node *i = NULL;
	spin_lock_irq(&alloc->lock);

	/* First find the node in the used list and decrement its ref count */
	list_for_each_entry(i, &alloc->used, list) {
		if (i->base == base_id && i->num == count) {
			--i->refcount;
			if (i->refcount == 0) {
				list_del(&i->list);
				spin_unlock_irq(&alloc->lock);
				if (i->is_alloced)
					_dpa_alloc_free(alloc, base_id, count);
				kfree(i);
				return;
			}
			spin_unlock_irq(&alloc->lock);
			return;
		}
	}
	/* Couldn't find the allocation */
	pr_err("Attempt to free ID 0x%x COUNT %d that wasn't alloc'd or reserved\n",
	       base_id, count);
	spin_unlock_irq(&alloc->lock);
}

void dpa_alloc_seed(struct dpa_alloc *alloc, u32 base_id, u32 count)
{
	/* Same as free but no previous allocation checking is needed */
	_dpa_alloc_free(alloc, base_id, count);
}


int dpa_alloc_reserve(struct dpa_alloc *alloc, u32 base, u32 num)
{
	struct alloc_node *i = NULL, *used_node;

	DPRINT("alloc_reserve(%d,%d)\n", base, num);
	DUMP(alloc);

	spin_lock_irq(&alloc->lock);

	/* Check for the node in the used list.
	   If found, increase it's refcount */
	list_for_each_entry(i, &alloc->used, list) {
		if ((i->base == base) && (i->num == num)) {
			++i->refcount;
			spin_unlock_irq(&alloc->lock);
			return 0;
		}
		if ((base >= i->base) && (base < (i->base + i->num))) {
			/* This is an attempt to reserve a region that was
			   already reserved or alloced with a different
			   base or num */
			pr_err("Cannot reserve %d - %d, it overlaps with"
			       " existing reservation from %d - %d\n",
			       base, base + num - 1, i->base,
			       i->base + i->num - 1);
			spin_unlock_irq(&alloc->lock);
			return -1;
		}
	}
	/* Check to make sure this ID isn't in the free list */
	list_for_each_entry(i, &alloc->free, list) {
		if ((base >= i->base) && (base < (i->base + i->num))) {
			/* yep, the reservation is within this node */
			pr_err("Cannot reserve %d - %d, it overlaps with"
			       " free range %d - %d and must be alloced\n",
			       base, base + num - 1,
			       i->base, i->base + i->num - 1);
			spin_unlock_irq(&alloc->lock);
			return -1;
		}
	}
	/* Add the allocation to the used list with a refcount of 1 */
	used_node = kmalloc(sizeof(*used_node), GFP_KERNEL);
	if (!used_node) {
		spin_unlock_irq(&alloc->lock);
		return -ENOMEM;

	}
	used_node->base = base;
	used_node->num = num;
	used_node->refcount = 1;
	used_node->is_alloced = 0;
	list_add_tail(&used_node->list, &alloc->used);
	spin_unlock_irq(&alloc->lock);
	return 0;
}


int dpa_alloc_pop(struct dpa_alloc *alloc, u32 *result, u32 *count)
{
	struct alloc_node *i = NULL;
	DPRINT("alloc_pop()\n");
	DUMP(alloc);
	spin_lock_irq(&alloc->lock);
	if (!list_empty(&alloc->free)) {
		i = list_entry(alloc->free.next, struct alloc_node, list);
		list_del(&i->list);
	}
	spin_unlock_irq(&alloc->lock);
	DPRINT("returning %d\n", i ? 0 : -ENOMEM);
	DUMP(alloc);
	if (!i)
		return -ENOMEM;
	*result = i->base;
	*count = i->num;
	kfree(i);
	return 0;
}

int dpa_alloc_check(struct dpa_alloc *list_head, u32 item)
{
	struct alloc_node *i = NULL;
	int res = 0;
	DPRINT("alloc_check()\n");
	spin_lock_irq(&list_head->lock);

	list_for_each_entry(i, &list_head->free, list) {
		if ((item >= i->base) && (item < (i->base + i->num))) {
			res = 1;
			break;
		}
	}
	spin_unlock_irq(&list_head->lock);
	return res;
}
