/* Copyright 2008-2011 Freescale Semiconductor, Inc.
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

#include "qman_private.h"

/* ----------------- */
/* --- FQID Pool --- */

struct qman_fqid_pool {
	/* Base and size of the FQID range */
	u32 fqid_base;
	u32 total;
	/* Number of FQIDs currently "allocated" */
	u32 used;
	/* Allocation optimisation. When 'used<total', it is the index of an
	 * available FQID. Otherwise there are no available FQIDs, and this
	 * will be set when the next deallocation occurs. */
	u32 next;
	/* A bit-field representation of the FQID range. */
	unsigned long *bits;
};

#define QLONG_BYTES	sizeof(unsigned long)
#define QLONG_BITS	(QLONG_BYTES * 8)
/* Number of 'longs' required for the given number of bits */
#define QNUM_LONGS(b)	(((b) + QLONG_BITS - 1) / QLONG_BITS)
/* Shorthand for the number of bytes of same (kmalloc, memset, etc) */
#define QNUM_BYTES(b)	(QNUM_LONGS(b) * QLONG_BYTES)
/* And in bits */
#define QNUM_BITS(b)	(QNUM_LONGS(b) * QLONG_BITS)

struct qman_fqid_pool *qman_fqid_pool_create(u32 fqid_start, u32 num)
{
	struct qman_fqid_pool *pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	unsigned int i;

	BUG_ON(!num);
	if (!pool)
		return NULL;
	pool->fqid_base = fqid_start;
	pool->total = num;
	pool->used = 0;
	pool->next = 0;
	pool->bits = kzalloc(QNUM_BYTES(num), GFP_KERNEL);
	if (!pool->bits) {
		kfree(pool);
		return NULL;
	}
	/* If num is not an even multiple of QLONG_BITS (or even 8, for
	 * byte-oriented searching) then we fill the trailing bits with 1, to
	 * make them look allocated (permanently). */
	for (i = num + 1; i < QNUM_BITS(num); i++)
		set_bit(i, pool->bits);
	return pool;
}
EXPORT_SYMBOL(qman_fqid_pool_create);

int qman_fqid_pool_destroy(struct qman_fqid_pool *pool)
{
	int ret = pool->used;
	kfree(pool->bits);
	kfree(pool);
	return ret;
}
EXPORT_SYMBOL(qman_fqid_pool_destroy);

int qman_fqid_pool_alloc(struct qman_fqid_pool *pool, u32 *fqid)
{
	int ret;
	if (pool->used == pool->total)
		return -ENOMEM;
	*fqid = pool->fqid_base + pool->next;
	ret = test_and_set_bit(pool->next, pool->bits);
	BUG_ON(ret);
	if (++pool->used == pool->total)
		return 0;
	pool->next = find_next_zero_bit(pool->bits, pool->total, pool->next);
	if (pool->next >= pool->total)
		pool->next = find_first_zero_bit(pool->bits, pool->total);
	BUG_ON(pool->next >= pool->total);
	return 0;
}
EXPORT_SYMBOL(qman_fqid_pool_alloc);

void qman_fqid_pool_free(struct qman_fqid_pool *pool, u32 fqid)
{
	int ret;

	fqid -= pool->fqid_base;
	ret = test_and_clear_bit(fqid, pool->bits);
	BUG_ON(!ret);
	if (pool->used-- == pool->total)
		pool->next = fqid;
}
EXPORT_SYMBOL(qman_fqid_pool_free);

u32 qman_fqid_pool_used(struct qman_fqid_pool *pool)
{
	return pool->used;
}
EXPORT_SYMBOL(qman_fqid_pool_used);
