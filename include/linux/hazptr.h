// SPDX-License-Identifier: LGPL-2.1-or-later
//
// SPDX-FileCopyrightText: 2024 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>

#ifndef _LINUX_HAZPTR_H
#define _LINUX_HAZPTR_H

/*
 * hazptr: Hazard Pointers
 *
 * This API provides existence guarantees of objects through hazard
 * pointers.
 *
 * Its main benefit over RCU is that it allows fast reclaim of
 * HP-protected pointers without needing to wait for a grace period.
 *
 * References:
 *
 * [1]: M. M. Michael, "Hazard pointers: safe memory reclamation for
 *      lock-free objects," in IEEE Transactions on Parallel and
 *      Distributed Systems, vol. 15, no. 6, pp. 491-504, June 2004
 */

#include <linux/percpu.h>
#include <linux/types.h>
#include <linux/cleanup.h>
#include <linux/sched.h>

/* 4 slots (each sizeof(hazptr_slot_item)) fit in a single 64-byte cache line. */
#define NR_HAZPTR_PERCPU_SLOTS	4
#define HAZPTR_WILDCARD		((void *) 0x1UL)

/*
 * Hazard pointer slot.
 */
struct hazptr_slot {
	void *addr;
};

struct hazptr_overflow_list;

struct hazptr_backup_slot {
	struct hlist_node overflow_node;
	struct hazptr_slot slot;
	/* Overflow list where the backup slot is added. */
	struct hazptr_overflow_list *overflow_list;
};

struct hazptr_ctx {
	struct hazptr_slot *slot;
	/* Backup slot in case all per-CPU slots are used. */
	struct hazptr_backup_slot backup_slot;
	struct hlist_node preempt_node;
};

struct hazptr_slot_ctx {
	struct hazptr_ctx *ctx;
};

struct hazptr_slot_item {
	struct hazptr_slot slot;
	struct hazptr_slot_ctx ctx;
};

struct hazptr_percpu_slots {
	struct hazptr_slot_item items[NR_HAZPTR_PERCPU_SLOTS];
} ____cacheline_aligned;

DECLARE_PER_CPU(struct hazptr_percpu_slots, hazptr_percpu_slots);

void *__hazptr_acquire(struct hazptr_ctx *ctx, void * const *addr_p);

/*
 * hazptr_synchronize: Wait until @addr is released from all slots.
 *
 * Wait to observe that each slot contains a value that differs from
 * @addr before returning.
 * Should be called from preemptible context.
 */
void hazptr_synchronize(void *addr);

/*
 * hazptr_chain_backup_slot: Chain backup slot into overflow list.
 *
 * Set backup slot address to @addr, and chain it into the overflow
 * list.
 */
struct hazptr_slot *hazptr_chain_backup_slot(struct hazptr_ctx *ctx);

/*
 * hazptr_unchain_backup_slot: Unchain backup slot from overflow list.
 */
void hazptr_unchain_backup_slot(struct hazptr_ctx *ctx);

static inline
bool hazptr_slot_is_backup(struct hazptr_ctx *ctx, struct hazptr_slot *slot)
{
	return slot == &ctx->backup_slot.slot;
}

/* Internal helper. */
static inline
void hazptr_promote_to_backup_slot(struct hazptr_ctx *ctx, struct hazptr_slot *slot)
{
	struct hazptr_slot *backup_slot;

	backup_slot = hazptr_chain_backup_slot(ctx);
	/*
	 * Move hazard pointer from the per-CPU slot to the
	 * backup slot. This requires hazard pointer
	 * synchronize to iterate on per-CPU slots with
	 * load-acquire before iterating on the overflow list.
	 */
	WRITE_ONCE(backup_slot->addr, slot->addr);
	/*
	 * store-release orders store to backup slot addr before
	 * store to per-CPU slot addr.
	 */
	smp_store_release(&slot->addr, NULL);
	/* Use the backup slot for context. */
	ctx->slot = backup_slot;
}

static inline
void hazptr_detach(struct hazptr_ctx *ctx)
{
	struct hazptr_slot *slot;

	guard(preempt)();
	slot = ctx->slot;
	if (unlikely(hazptr_slot_is_backup(ctx, slot)))
		return;
	hazptr_promote_to_backup_slot(ctx, slot);
}

static inline
void hazptr_note_context_switch(void)
{
	struct hazptr_percpu_slots *percpu_slots = this_cpu_ptr(&hazptr_percpu_slots);
	unsigned int idx;

	for (idx = 0; idx < NR_HAZPTR_PERCPU_SLOTS; idx++) {
		struct hazptr_slot_item *item = &percpu_slots->items[idx];
		struct hazptr_slot *slot = &item->slot;

		if (!slot->addr)
			continue;
		hazptr_promote_to_backup_slot(item->ctx.ctx, slot);
	}
}

/*
 * hazptr_acquire: Load pointer at address and protect with hazard pointer.
 *
 * Load @addr_p, and protect the loaded pointer with hazard pointer.
 * When using hazptr_acquire from interrupt handlers, the acquired slots
 * need to be released before returning from the interrupt handler.
 *
 * Returns a non-NULL protected address if the loaded pointer is non-NULL.
 * Returns NULL if the loaded pointer is NULL.
 *
 * On success the protected hazptr slot is stored in @ctx->slot.
 */
static inline
void *hazptr_acquire(struct hazptr_ctx *ctx, void * const *addr_p)
{
	struct hazptr_percpu_slots *percpu_slots;
	struct hazptr_slot_item *slot_item;
	struct hazptr_slot *slot;
	void *addr;

	guard(preempt)();
	percpu_slots = this_cpu_ptr(&hazptr_percpu_slots);
	slot_item = &percpu_slots->items[0];
	slot = &slot_item->slot;
	if (unlikely(slot->addr))
		return __hazptr_acquire(ctx, addr_p);
	WRITE_ONCE(slot->addr, HAZPTR_WILDCARD);	/* Store B */

	/* Memory ordering: Store B before Load A. */
	smp_mb();

	/*
	 * Load @addr_p after storing wildcard to the hazard pointer slot.
	 */
	addr = READ_ONCE(*addr_p);	/* Load A */

	/*
	 * We don't care about ordering of Store C. It will simply
	 * replace the wildcard by a more specific address. If addr is
	 * NULL, we simply store NULL into the slot.
	 */
	WRITE_ONCE(slot->addr, addr);	/* Store C */
	slot_item->ctx.ctx = ctx;
	ctx->slot = slot;
	return addr;
}

/* Release the protected hazard pointer from @slot. */
static inline
void hazptr_release(struct hazptr_ctx *ctx, void *addr)
{
	struct hazptr_slot *slot;

	if (!addr)
		return;
	guard(preempt)();
	slot = ctx->slot;
	smp_store_release(&slot->addr, NULL);
	if (unlikely(hazptr_slot_is_backup(ctx, slot)))
		hazptr_unchain_backup_slot(ctx);
}

void hazptr_init(void);

#endif /* _LINUX_HAZPTR_H */
