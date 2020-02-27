/* Copyright 2008-2012 Freescale Semiconductor, Inc.
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

#ifndef FSL_BMAN_H
#define FSL_BMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Last updated for v00.79 of the BG */

/* Portal processing (interrupt) sources */
#define BM_PIRQ_RCRI	0x00000002	/* RCR Ring (below threshold) */
#define BM_PIRQ_BSCN	0x00000001	/* Buffer depletion State Change */

/* This wrapper represents a bit-array for the depletion state of the 64 Bman
 * buffer pools. */
struct bman_depletion {
	u32 __state[2];
};
#define BMAN_DEPLETION_EMPTY { { 0x00000000, 0x00000000 } }
#define BMAN_DEPLETION_FULL { { 0xffffffff, 0xffffffff } }
#define __bmdep_word(x) ((x) >> 5)
#define __bmdep_shift(x) ((x) & 0x1f)
#define __bmdep_bit(x) (0x80000000 >> __bmdep_shift(x))
static inline void bman_depletion_init(struct bman_depletion *c)
{
	c->__state[0] = c->__state[1] = 0;
}
static inline void bman_depletion_fill(struct bman_depletion *c)
{
	c->__state[0] = c->__state[1] = ~0;
}
static inline int bman_depletion_get(const struct bman_depletion *c, u8 bpid)
{
	return c->__state[__bmdep_word(bpid)] & __bmdep_bit(bpid);
}
static inline void bman_depletion_set(struct bman_depletion *c, u8 bpid)
{
	c->__state[__bmdep_word(bpid)] |= __bmdep_bit(bpid);
}
static inline void bman_depletion_unset(struct bman_depletion *c, u8 bpid)
{
	c->__state[__bmdep_word(bpid)] &= ~__bmdep_bit(bpid);
}

/* ------------------------------------------------------- */
/* --- Bman data structures (and associated constants) --- */

/* Represents s/w corenet portal mapped data structures */
struct bm_rcr_entry;	/* RCR (Release Command Ring) entries */
struct bm_mc_command;	/* MC (Management Command) command */
struct bm_mc_result;	/* MC result */

/* Code-reduction, define a wrapper for 48-bit buffers. In cases where a buffer
 * pool id specific to this buffer is needed (BM_RCR_VERB_CMD_BPID_MULTI,
 * BM_MCC_VERB_ACQUIRE), the 'bpid' field is used. */
struct bm_buffer {
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 __reserved1;
			u8 bpid;
			u16 hi; /* High 16-bits of 48-bit address */
			u32 lo; /* Low 32-bits of 48-bit address */
#else
			u32 lo;
			u16 hi;
			u8 bpid;
			u8 __reserved;
#endif
		};
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u64 __notaddress:16;
			u64 addr:48;
#else
			u64 addr:48;
			u64 __notaddress:16;
#endif
		};
		u64 opaque;
	};
} __aligned(8);
static inline u64 bm_buffer_get64(const struct bm_buffer *buf)
{
	return buf->addr;
}
static inline dma_addr_t bm_buf_addr(const struct bm_buffer *buf)
{
	return (dma_addr_t)buf->addr;
}
/* Macro, so we compile better if 'v' isn't always 64-bit */
#define bm_buffer_set64(buf, v) \
	do { \
		struct bm_buffer *__buf931 = (buf); \
		__buf931->hi = upper_32_bits(v); \
		__buf931->lo = lower_32_bits(v); \
	} while (0)

/* See 1.5.3.5.4: "Release Command" */
struct bm_rcr_entry {
	union {
		struct {
			u8 __dont_write_directly__verb;
			u8 bpid; /* used with BM_RCR_VERB_CMD_BPID_SINGLE */
			u8 __reserved1[62];
		};
		struct bm_buffer bufs[8];
	};
} __packed;
#define BM_RCR_VERB_VBIT		0x80
#define BM_RCR_VERB_CMD_MASK		0x70	/* one of two values; */
#define BM_RCR_VERB_CMD_BPID_SINGLE	0x20
#define BM_RCR_VERB_CMD_BPID_MULTI	0x30
#define BM_RCR_VERB_BUFCOUNT_MASK	0x0f	/* values 1..8 */

/* See 1.5.3.1: "Acquire Command" */
/* See 1.5.3.2: "Query Command" */
struct bm_mcc_acquire {
	u8 bpid;
	u8 __reserved1[62];
} __packed;
struct bm_mcc_query {
	u8 __reserved2[63];
} __packed;
struct bm_mc_command {
	u8 __dont_write_directly__verb;
	union {
		struct bm_mcc_acquire acquire;
		struct bm_mcc_query query;
	};
} __packed;
#define BM_MCC_VERB_VBIT		0x80
#define BM_MCC_VERB_CMD_MASK		0x70	/* where the verb contains; */
#define BM_MCC_VERB_CMD_ACQUIRE		0x10
#define BM_MCC_VERB_CMD_QUERY		0x40
#define BM_MCC_VERB_ACQUIRE_BUFCOUNT	0x0f	/* values 1..8 go here */

/* See 1.5.3.3: "Acquire Response" */
/* See 1.5.3.4: "Query Response" */
struct bm_pool_state {
	u8 __reserved1[32];
	/* "availability state" and "depletion state" */
	struct {
		u8 __reserved1[8];
		/* Access using bman_depletion_***() */
		struct bman_depletion state;
	} as, ds;
};
struct bm_mc_result {
	union {
		struct {
			u8 verb;
			u8 __reserved1[63];
		};
		union {
			struct {
				u8 __reserved1;
				u8 bpid;
				u8 __reserved2[62];
			};
			struct bm_buffer bufs[8];
		} acquire;
		struct bm_pool_state query;
	};
} __packed;
#define BM_MCR_VERB_VBIT		0x80
#define BM_MCR_VERB_CMD_MASK		BM_MCC_VERB_CMD_MASK
#define BM_MCR_VERB_CMD_ACQUIRE		BM_MCC_VERB_CMD_ACQUIRE
#define BM_MCR_VERB_CMD_QUERY		BM_MCC_VERB_CMD_QUERY
#define BM_MCR_VERB_CMD_ERR_INVALID	0x60
#define BM_MCR_VERB_CMD_ERR_ECC		0x70
#define BM_MCR_VERB_ACQUIRE_BUFCOUNT	BM_MCC_VERB_ACQUIRE_BUFCOUNT /* 0..8 */
/* Determine the "availability state" of pool 'p' from a query result 'r' */
#define BM_MCR_QUERY_AVAILABILITY(r, p)	\
		bman_depletion_get(&r->query.as.state, p)
/* Determine the "depletion state" of pool 'p' from a query result 'r' */
#define BM_MCR_QUERY_DEPLETION(r, p)	\
		bman_depletion_get(&r->query.ds.state, p)

/*******************************************************************/
/* Managed (aka "shared" or "mux/demux") portal, high-level i/face */
/*******************************************************************/

	/* Portal and Buffer Pools */
	/* ----------------------- */
/* Represents a managed portal */
struct bman_portal;

/* This object type represents Bman buffer pools. */
struct bman_pool;

struct bman_portal_config {
	/* This is used for any "core-affine" portals, ie. default portals
	 * associated to the corresponding cpu. -1 implies that there is no core
	 * affinity configured. */
	int cpu;
	/* portal interrupt line */
	int irq;
	/* the unique index of this portal */
	u32 index;
	/* Is this portal shared? (If so, it has coarser locking and demuxes
	 * processing on behalf of other CPUs.) */
	int is_shared;
	/* These are the buffer pool IDs that may be used via this portal. */
	struct bman_depletion mask;
};

/* This callback type is used when handling pool depletion entry/exit. The
 * 'cb_ctx' value is the opaque value associated with the pool object in
 * bman_new_pool(). 'depleted' is non-zero on depletion-entry, and zero on
 * depletion-exit. */
typedef void (*bman_cb_depletion)(struct bman_portal *bm,
			struct bman_pool *pool, void *cb_ctx, int depleted);

/* This struct specifies parameters for a bman_pool object. */
struct bman_pool_params {
	/* index of the buffer pool to encapsulate (0-63), ignored if
	 * BMAN_POOL_FLAG_DYNAMIC_BPID is set. */
	u32 bpid;
	/* bit-mask of BMAN_POOL_FLAG_*** options */
	u32 flags;
	/* depletion-entry/exit callback, if BMAN_POOL_FLAG_DEPLETION is set */
	bman_cb_depletion cb;
	/* opaque user value passed as a parameter to 'cb' */
	void *cb_ctx;
	/* depletion-entry/exit thresholds, if BMAN_POOL_FLAG_THRESH is set. NB:
	 * this is only allowed if BMAN_POOL_FLAG_DYNAMIC_BPID is used *and*
	 * when run in the control plane (which controls Bman CCSR). This array
	 * matches the definition of bm_pool_set(). */
	u32 thresholds[4];
};

/* Flags to bman_new_pool() */
#define BMAN_POOL_FLAG_NO_RELEASE    0x00000001 /* can't release to pool */
#define BMAN_POOL_FLAG_ONLY_RELEASE  0x00000002 /* can only release to pool */
#define BMAN_POOL_FLAG_DEPLETION     0x00000004 /* track depletion entry/exit */
#define BMAN_POOL_FLAG_DYNAMIC_BPID  0x00000008 /* (de)allocate bpid */
#define BMAN_POOL_FLAG_THRESH        0x00000010 /* set depletion thresholds */
#define BMAN_POOL_FLAG_STOCKPILE     0x00000020 /* stockpile to reduce hw ops */

/* Flags to bman_release() */
#ifdef CONFIG_FSL_DPA_CAN_WAIT
#define BMAN_RELEASE_FLAG_WAIT       0x00000001 /* wait if RCR is full */
#define BMAN_RELEASE_FLAG_WAIT_INT   0x00000002 /* if we wait, interruptible? */
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
#define BMAN_RELEASE_FLAG_WAIT_SYNC  0x00000004 /* if wait, until consumed? */
#endif
#endif
#define BMAN_RELEASE_FLAG_NOW        0x00000008 /* issue immediate release */

/* Flags to bman_acquire() */
#define BMAN_ACQUIRE_FLAG_STOCKPILE  0x00000001 /* no hw op, stockpile only */

	/* Portal Management */
	/* ----------------- */
/**
 * bman_get_portal_config - get portal configuration settings
 *
 * This returns a read-only view of the current cpu's affine portal settings.
 */
const struct bman_portal_config *bman_get_portal_config(void);

/**
 * bman_irqsource_get - return the portal work that is interrupt-driven
 *
 * Returns a bitmask of BM_PIRQ_**I processing sources that are currently
 * enabled for interrupt handling on the current cpu's affine portal. These
 * sources will trigger the portal interrupt and the interrupt handler (or a
 * tasklet/bottom-half it defers to) will perform the corresponding processing
 * work. The bman_poll_***() functions will only process sources that are not in
 * this bitmask. If the current CPU is sharing a portal hosted on another CPU,
 * this always returns zero.
 */
u32 bman_irqsource_get(void);

/**
 * bman_irqsource_add - add processing sources to be interrupt-driven
 * @bits: bitmask of BM_PIRQ_**I processing sources
 *
 * Adds processing sources that should be interrupt-driven (rather than
 * processed via bman_poll_***() functions). Returns zero for success, or
 * -EINVAL if the current CPU is sharing a portal hosted on another CPU. */
int bman_irqsource_add(u32 bits);

/**
 * bman_irqsource_remove - remove processing sources from being interrupt-driven
 * @bits: bitmask of BM_PIRQ_**I processing sources
 *
 * Removes processing sources from being interrupt-driven, so that they will
 * instead be processed via bman_poll_***() functions. Returns zero for success,
 * or -EINVAL if the current CPU is sharing a portal hosted on another CPU. */
int bman_irqsource_remove(u32 bits);

/**
 * bman_affine_cpus - return a mask of cpus that have affine portals
 */
const cpumask_t *bman_affine_cpus(void);

/**
 * bman_poll_slow - process anything that isn't interrupt-driven.
 *
 * This function does any portal processing that isn't interrupt-driven. If the
 * current CPU is sharing a portal hosted on another CPU, this function will
 * return -EINVAL, otherwise the return value is a bitmask of BM_PIRQ_* sources
 * indicating what interrupt sources were actually processed by the call.
 *
 * NB, unlike the legacy wrapper bman_poll(), this function will
 * deterministically check for the presence of portal processing work and do it,
 * which implies some latency even if there's nothing to do. The bman_poll()
 * wrapper on the other hand (like the qman_poll() wrapper) attenuates this by
 * checking for (and doing) portal processing infrequently. Ie. such that
 * qman_poll() and bman_poll() can be called from core-processing loops. Use
 * bman_poll_slow() when you yourself are deciding when to incur the overhead of
 * processing.
 */
u32 bman_poll_slow(void);

/**
 * bman_poll - process anything that isn't interrupt-driven.
 *
 * Dispatcher logic on a cpu can use this to trigger any maintenance of the
 * affine portal. This function does whatever processing is not triggered by
 * interrupts. This is a legacy wrapper that can be used in core-processing
 * loops but mitigates the performance overhead of portal processing by
 * adaptively bypassing true portal processing most of the time. (Processing is
 * done once every 10 calls if the previous processing revealed that work needed
 * to be done, or once very 1000 calls if the previous processing revealed no
 * work needed doing.) If you wish to control this yourself, call
 * bman_poll_slow() instead, which always checks for portal processing work.
 */
void bman_poll(void);

/**
 * bman_rcr_is_empty - Determine if portal's RCR is empty
 *
 * For use in situations where a cpu-affine caller needs to determine when all
 * releases for the local portal have been processed by Bman but can't use the
 * BMAN_RELEASE_FLAG_WAIT_SYNC flag to do this from the final bman_release().
 * The function forces tracking of RCR consumption (which normally doesn't
 * happen until release processing needs to find space to put new release
 * commands), and returns zero if the ring still has unprocessed entries,
 * non-zero if it is empty.
 */
int bman_rcr_is_empty(void);

/**
 * bman_alloc_bpid_range - Allocate a contiguous range of BPIDs
 * @result: is set by the API to the base BPID of the allocated range
 * @count: the number of BPIDs required
 * @align: required alignment of the allocated range
 * @partial: non-zero if the API can return fewer than @count BPIDs
 *
 * Returns the number of buffer pools allocated, or a negative error code. If
 * @partial is non zero, the allocation request may return a smaller range of
 * BPs than requested (though alignment will be as requested). If @partial is
 * zero, the return value will either be 'count' or negative.
 */
int bman_alloc_bpid_range(u32 *result, u32 count, u32 align, int partial);
static inline int bman_alloc_bpid(u32 *result)
{
	int ret = bman_alloc_bpid_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}

/**
 * bman_release_bpid_range - Release the specified range of buffer pool IDs
 * @bpid: the base BPID of the range to deallocate
 * @count: the number of BPIDs in the range
 *
 * This function can also be used to seed the allocator with ranges of BPIDs
 * that it can subsequently allocate from.
 */
void bman_release_bpid_range(u32 bpid, unsigned int count);
static inline void bman_release_bpid(u32 bpid)
{
	bman_release_bpid_range(bpid, 1);
}

int bman_reserve_bpid_range(u32 bpid, unsigned int count);
static inline int bman_reserve_bpid(u32 bpid)
{
	return bman_reserve_bpid_range(bpid, 1);
}

void bman_seed_bpid_range(u32 bpid, unsigned int count);


int bman_shutdown_pool(u32 bpid);

	/* Pool management */
	/* --------------- */
/**
 * bman_new_pool - Allocates a Buffer Pool object
 * @params: parameters specifying the buffer pool ID and behaviour
 *
 * Creates a pool object for the given @params. A portal and the depletion
 * callback field of @params are only used if the BMAN_POOL_FLAG_DEPLETION flag
 * is set. NB, the fields from @params are copied into the new pool object, so
 * the structure provided by the caller can be released or reused after the
 * function returns.
 */
struct bman_pool *bman_new_pool(const struct bman_pool_params *params);

/**
 * bman_free_pool - Deallocates a Buffer Pool object
 * @pool: the pool object to release
 *
 */
void bman_free_pool(struct bman_pool *pool);

/**
 * bman_get_params - Returns a pool object's parameters.
 * @pool: the pool object
 *
 * The returned pointer refers to state within the pool object so must not be
 * modified and can no longer be read once the pool object is destroyed.
 */
const struct bman_pool_params *bman_get_params(const struct bman_pool *pool);

/**
 * bman_release - Release buffer(s) to the buffer pool
 * @pool: the buffer pool object to release to
 * @bufs: an array of buffers to release
 * @num: the number of buffers in @bufs (1-8)
 * @flags: bit-mask of BMAN_RELEASE_FLAG_*** options
 *
 * Adds the given buffers to RCR entries. If the portal @p was created with the
 * "COMPACT" flag, then it will be using a compaction algorithm to improve
 * utilisation of RCR. As such, these buffers may join an existing ring entry
 * and/or it may not be issued right away so as to allow future releases to join
 * the same ring entry. Use the BMAN_RELEASE_FLAG_NOW flag to override this
 * behaviour by committing the RCR entry (or entries) right away. If the RCR
 * ring is full, the function will return -EBUSY unless BMAN_RELEASE_FLAG_WAIT
 * is selected, in which case it will sleep waiting for space to become
 * available in RCR. If the function receives a signal before such time (and
 * BMAN_RELEASE_FLAG_WAIT_INT is set), the function returns -EINTR. Otherwise,
 * it returns zero.
 */
int bman_release(struct bman_pool *pool, const struct bm_buffer *bufs, u8 num,
			u32 flags);

/**
 * bman_acquire - Acquire buffer(s) from a buffer pool
 * @pool: the buffer pool object to acquire from
 * @bufs: array for storing the acquired buffers
 * @num: the number of buffers desired (@bufs is at least this big)
 *
 * Issues an "Acquire" command via the portal's management command interface.
 * The return value will be the number of buffers obtained from the pool, or a
 * negative error code if a h/w error or pool starvation was encountered. In
 * the latter case, the content of @bufs is undefined.
 */
int bman_acquire(struct bman_pool *pool, struct bm_buffer *bufs, u8 num,
			u32 flags);

/**
 * bman_flush_stockpile - Flush stockpile buffer(s) to the buffer pool
 * @pool: the buffer pool object the stockpile belongs
 * @flags: bit-mask of BMAN_RELEASE_FLAG_*** options
 *
 * Adds stockpile buffers to RCR entries until the stockpile is empty.
 * The return value will be a negative error code if a h/w error occurred.
 * If BMAN_RELEASE_FLAG_NOW flag is passed and RCR ring is full,
 * -EAGAIN will be returned.
 */
int bman_flush_stockpile(struct bman_pool *pool, u32 flags);

/**
 * bman_query_pools - Query all buffer pool states
 * @state: storage for the queried availability and depletion states
 */
int bman_query_pools(struct bm_pool_state *state);

#ifdef CONFIG_FSL_BMAN_CONFIG
/**
 * bman_query_free_buffers - Query how many free buffers are in buffer pool
 * @pool: the buffer pool object to query
 *
 * Return the number of the free buffers
 */
u32 bman_query_free_buffers(struct bman_pool *pool);

/**
 * bman_update_pool_thresholds - Change the buffer pool's depletion thresholds
 * @pool: the buffer pool object to which the thresholds will be set
 * @thresholds: the new thresholds
 */
int bman_update_pool_thresholds(struct bman_pool *pool, const u32 *thresholds);
#endif

/**
 * The below bman_p_***() variant might be called in a situation that the cpu
 * which the portal affine to is not online yet.
 * @bman_portal specifies which portal the API will use.
*/
int bman_p_irqsource_add(struct bman_portal *p, __maybe_unused u32 bits);
#ifdef __cplusplus
}
#endif

#endif /* FSL_BMAN_H */
