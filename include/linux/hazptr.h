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

/* The current hazard pointer wildcard. */
extern void *hazptr_wildcard;

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
#ifdef CONFIG_HAZPTR_DEBUG
	bool detach_task, detach_cpu;	/* Whether the ctx has been detached from task/cpu. */
	int acquire_pid, acquire_cpu;	/* Note the task and cpu number at acquire. */
	unsigned long acquire_caller;	/* Acquire instruction pointer. */
#endif
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

/**
 * hazptr_synchronize: Wait for release from hazard-pointer protection
 *
 * @addr: The address to be released from hazard-pointer protection
 *
 * Wait for the specified @addr to be released from protection from all
 * hazard pointers.  The caller should make @addr inaccessible to all
 * hazard-pointer readers before invoking this function.
 *
 * Must be called from preemptible context.
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

/**
 * hazptr_detach - Allow a hazard pointer to be released in some other context
 *
 * @ctx: The hazard-pointer context to be detached.
 *
 * By default, a given hazptr_acquire() and the corresponding
 * hazptr_release() must run in a single execution context, for example,
 * the context of a single task or a single interrupt handler.  When you
 * have acquired a hazard pointer in one context and need to release it
 * in another, you must invoke hazptr_detach() on that hazard pointer's
 * context.  It is permissible to invoke hazptr_detach() multiple times
 * on the same @ctx while it is protecting the same pointer, however,
 * the first invocation absolutely must be in the same context that did
 * the hazptr_acquire(), and must take place after the return from that
 * hazptr_acquire().
 *
 * For example, if a hazard pointer is acquired by a task and released
 * by a timer handler, that task would need to pass the hazard pointer's
 * context to hazptr_detach() after return from the hazptr_acquire() and
 * before arming the timer (or at least before the handler had a chance
 * to access that hazard-pointer context).
 */
static inline
void hazptr_detach(struct hazptr_ctx *ctx)
{
	struct hazptr_slot *slot;

	guard(preempt)();
#ifdef CONFIG_HAZPTR_DEBUG
	ctx->detach_task = ctx->detach_cpu = true;
#endif
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
#ifdef CONFIG_HAZPTR_DEBUG
		item->ctx.ctx->detach_cpu = true;
#endif
		hazptr_promote_to_backup_slot(item->ctx.ctx, slot);
	}
}

/**
 * hazptr_acquire - Load pointer at address and protect with hazard pointer.
 *
 * @ctx: The hazard-pointer context to be passed to hazptr_release().
 * @addr_p: Pointer to the pointer that is to be hazard-pointer protected.
 *
 * Load @addr_p, and protect the loaded pointer with hazard pointer.
 * This protection is roughly similar to (but way faster than) that of a
 * reference counter, and ends with a later call to hazptr_release().
 *
 * This protection is unconditional, and has limitations similar to
 * that of unconditional reference-counter acquisition.  In particular,
 * although holding a hazard pointer prevents a hazard-pointer-protected
 * object from being freed, it does not prevent that object from being
 * removed from a linked data structure, and does not prevent other
 * hazard-pointer-protected objects referenced by this object from being
 * both removed and freed.  At which point, invoking hazptr_acquire()
 * on these dangling pointers would be a bug.  On the other hand, use of
 * hazptr_acquire() is safe for immortal pointers to objects that do not
 * themselves contain pointers to hazard-pointer-protected objects.
 * Other (more complex) use cases are also possible.
 *
 * By default, the call to hazptr_release() must be running in the same
 * execution context as the corresponding hazptr_acquire(), for example,
 * within the same task or interrupt handler.  When it is necessary to
 * instead call hazptr_release() from some other context, pass @ctx to
 * hazptr_detach() in the original context after invoking hazptr_acquire()
 * but before making the hazard pointer available to that other context.
 *
 * It is not permissible to invoke hazptr_acquire() twice on the same @ctx
 * without an intervening hazptr_release().
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
#ifdef CONFIG_HAZPTR_DEBUG
	ctx->detach_cpu = ctx->detach_task = false;
	ctx->acquire_pid = current->pid;
	ctx->acquire_cpu = smp_processor_id();
	ctx->acquire_caller = _THIS_IP_;
#endif
	if (unlikely(slot->addr))
		return __hazptr_acquire(ctx, addr_p);
	WRITE_ONCE(slot->addr, READ_ONCE(hazptr_wildcard));	/* Store B */

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

#ifdef CONFIG_HAZPTR_DEBUG
/* Called with preemption disabled. */
static inline
void hazptr_release_debug(struct hazptr_ctx *ctx, void *addr)
{
	int pid = current->pid, cpu = smp_processor_id();
	bool warn_remote_cpu = !ctx->detach_cpu && ctx->acquire_cpu != cpu,
	     warn_remote_task = !ctx->detach_task && ctx->acquire_pid != pid;

	WARN_ONCE(warn_remote_cpu || warn_remote_task,
		"Hazard Pointer (addr=%p) released on remote %s without %s. Acquire: caller=%pS, pid=%d, cpu=%d. Release: pid=%d, cpu=%d.",
		addr,
		warn_remote_task ? "task" : "cpu",
		warn_remote_task ? "being detached from task" : "context switch",
		(void *) ctx->acquire_caller, ctx->acquire_pid, ctx->acquire_cpu, pid, cpu);
}
#else
static inline void hazptr_release_debug(struct hazptr_ctx *ctx, void *addr) { }
#endif

/**
 * hazptr_release - Release the specified hazard pointer
 *
 * @ctx: The hazard-pointer context that was passed to hazptr_acquire().
 * @addr_p: The pointer that is to be hazard-pointer unprotected.
 *
 * Release the protected hazard pointer recorded in @ctx.
 *
 * By default, hazptr_release() must execute in the same execution context
 * that invoked the corresponding hazptr_acquire(), for example, within the
 * same task or the same interrupt handler.  However, if this restriction
 * is problematic for your use case, please see hazptr_detach().
 *
 * It is permissible (though unwise from a maintainability viewpoint)
 * to invoke hazptr_release() twice on the same @ctx without an intervening
 * hazptr_acquire().
 */
static inline
void hazptr_release(struct hazptr_ctx *ctx, void *addr)
{
	struct hazptr_slot *slot;

	if (!addr)
		return;
	guard(preempt)();
	hazptr_release_debug(ctx, addr);
	slot = ctx->slot;
	smp_store_release(&slot->addr, NULL);
	if (unlikely(hazptr_slot_is_backup(ctx, slot)))
		hazptr_unchain_backup_slot(ctx);
}

void hazptr_init(void);

#endif /* _LINUX_HAZPTR_H */
