// SPDX-License-Identifier: LGPL-2.1-or-later
//
// SPDX-FileCopyrightText: 2024 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>

/*
 * hazptr: Hazard Pointers
 */

#include <linux/hazptr.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/export.h>

/*
 * The current hazard pointer wildcard. Flips between 1UL and 2UL to guarantee
 * hazptr_synchronize forward progress even with a steady stream of readers.
 * This wildcard value is used by acquire to temporarily tag the per-CPU slots.
 * This also affects the overflow list selection: the current list used by
 * readers is array[(unsigned long) hazptr_wildcard - 1].
 */
static DEFINE_MUTEX(hazptr_wildcard_lock);	/* Protect the wildcard flip. */
void *hazptr_wildcard = (void *) 1UL;
EXPORT_SYMBOL_GPL(hazptr_wildcard);

struct hazptr_overflow_list {
	raw_spinlock_t lock;		/* Lock protecting overflow list and list generation. */
	struct hlist_head head;		/* Overflow list head. */
	uint64_t gen;			/* Overflow list generation. */
};

/*
 * Flip between two lists to guarantee list scan forward progress even
 * with frequent generation counter increments. The list additions are
 * always done on a different list than the one used for scan. The scan
 * successively iterates on both lists. Therefore, only list removals
 * can cause the iteration to retry, and the number of removals is
 * limited to the number of list elements.
 */
struct hazptr_overflow_list_flip {
	struct hazptr_overflow_list array[2];
};

static DEFINE_PER_CPU(struct hazptr_overflow_list_flip, percpu_overflow_list_flip);

DEFINE_PER_CPU(struct hazptr_percpu_slots, hazptr_percpu_slots);
EXPORT_PER_CPU_SYMBOL_GPL(hazptr_percpu_slots);

static
void *flip_wildcard(void *wildcard)
{
	return ((unsigned long) wildcard == 1UL) ? (void *) 2UL : (void *) 1UL;
}

static
bool is_wildcard(void *addr)
{
	if ((unsigned long) addr == 1UL || (unsigned long) addr == 2UL)
		return true;
	return false;
}

static
struct hazptr_slot *hazptr_get_free_percpu_slot(struct hazptr_ctx *ctx)
{
	struct hazptr_percpu_slots *percpu_slots = this_cpu_ptr(&hazptr_percpu_slots);
	unsigned int idx;

	for (idx = 0; idx < NR_HAZPTR_PERCPU_SLOTS; idx++) {
		struct hazptr_slot_item *item = &percpu_slots->items[idx];
		struct hazptr_slot *slot = &item->slot;

		if (!slot->addr) {
			item->ctx.ctx = ctx;
			return slot;
		}
	}
	/* All slots are in use. */
	return NULL;
}

/*
 * Hazard pointer acquire slow path.
 * Called with preemption disabled.
 */
void *__hazptr_acquire(struct hazptr_ctx *ctx, void * const *addr_p)
{
	struct hazptr_slot *slot = hazptr_get_free_percpu_slot(ctx);
	void *addr;

	/*
	 * If all the per-CPU slots are already in use, fallback
	 * to the backup slot.
	 */
	if (unlikely(!slot))
		slot = hazptr_chain_backup_slot(ctx);
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
	ctx->slot = slot;
	if (!addr && hazptr_slot_is_backup(ctx, slot))
		hazptr_unchain_backup_slot(ctx);
	return addr;
}
EXPORT_SYMBOL_GPL(__hazptr_acquire);

/*
 * Perform piecewise iteration on overflow list waiting until "addr" is
 * not present. Raw spinlock is released and taken between each list
 * item and busy loop iteration. The overflow list generation is checked
 * each time the lock is taken to validate that the list has not changed
 * before resuming iteration or busy wait. If the generation has
 * changed, retry the entire list traversal.
 */
static
void hazptr_synchronize_overflow_list(struct hazptr_overflow_list *overflow_list, void *addr)
{
	struct hazptr_backup_slot *backup_slot;
	uint64_t snapshot_gen;
	unsigned long flags;

	raw_spin_lock_irqsave(&overflow_list->lock, flags);
retry:
	snapshot_gen = overflow_list->gen;
	hlist_for_each_entry(backup_slot, &overflow_list->head, overflow_node) {
		/* Busy-wait if node is found. */
		for (;;) {
			void *load_addr = smp_load_acquire(&backup_slot->slot.addr);	/* Load B */

			/* We don't expect wildcards in overflow list. */
			WARN_ON_ONCE(is_wildcard(load_addr));
			if (load_addr != addr)
				break;
			raw_spin_unlock_irqrestore(&overflow_list->lock, flags);
			cpu_relax();
			raw_spin_lock_irqsave(&overflow_list->lock, flags);
			if (overflow_list->gen != snapshot_gen)
				goto retry;
		}
		raw_spin_unlock_irqrestore(&overflow_list->lock, flags);
		/*
		 * Release raw spinlock, validate generation after
		 * re-acquiring the lock.
		 */
		raw_spin_lock_irqsave(&overflow_list->lock, flags);
		if (overflow_list->gen != snapshot_gen)
			goto retry;
	}
	raw_spin_unlock_irqrestore(&overflow_list->lock, flags);
}

static
void hazptr_synchronize_cpu_slots(int cpu, void *addr, void *scan_wildcard)
{
	struct hazptr_percpu_slots *percpu_slots = per_cpu_ptr(&hazptr_percpu_slots, cpu);
	unsigned int idx;

	for (idx = 0; idx < NR_HAZPTR_PERCPU_SLOTS; idx++) {
		struct hazptr_slot_item *item = &percpu_slots->items[idx];

		/* Busy-wait if node is found. */
		smp_cond_load_acquire(&item->slot.addr, VAL != addr && VAL != scan_wildcard); /* Load B */
	}
}

static
void hazptr_scan_period(void *addr, void *scan_wildcard)
{
	unsigned int scan_idx = (unsigned long) scan_wildcard - 1;
	int cpu;

	/* Scan all CPUs slots. */
	for_each_possible_cpu(cpu) {
		struct hazptr_overflow_list_flip *overflow_list_flip = per_cpu_ptr(&percpu_overflow_list_flip, cpu);

		/*
		 * Scan CPU slots.
		 * Forward progress against recurring wildcards is guaranteed
		 * by scanning for one wildcard while new elements use the
		 * other wildcard value (1UL vs 2UL).
		 * Forward progress against recurring single hazard pointer
		 * values is guaranteed by the fact that a hazard pointer
		 * is not reclaimed nor reused until the scan for that hazard
		 * pointer completes, which prevents a steady flow of readers
		 * to acquire that same hazard pointer value.
		 */
		hazptr_synchronize_cpu_slots(cpu, addr, scan_wildcard);

		/*
		 * Scan backup slots in percpu overflow lists.
		 * Forward progress is guaranteed by scanning one list
		 * while new elements are added into the other list.
		 */
		hazptr_synchronize_overflow_list(&overflow_list_flip->array[scan_idx], addr);
	}
}

/*
 * hazptr_synchronize: Wait until @addr is released from all slots.
 *
 * Wait to observe that each slot contains a value that differs from
 * @addr before returning.
 * Should be called from preemptible context.
 */
void hazptr_synchronize(void *addr)
{
	void *scan_wildcard;

	/*
	 * Busy-wait should only be done from preemptible context.
	 */
	lockdep_assert_preemption_enabled();

	/*
	 * Store A precedes hazptr_scan(): it unpublishes addr (sets it to
	 * NULL or to a different value), and thus hides it from hazard
	 * pointer readers.
	 */
	if (!addr)
		return;
	/* Memory ordering: Store A before Load B. */
	smp_mb();

	guard(mutex)(&hazptr_wildcard_lock);
	scan_wildcard = flip_wildcard(hazptr_wildcard);
	hazptr_scan_period(addr, scan_wildcard);
	WRITE_ONCE(hazptr_wildcard, scan_wildcard);	/* Flip the current wildcard. */
	hazptr_scan_period(addr, flip_wildcard(scan_wildcard));
}
EXPORT_SYMBOL_GPL(hazptr_synchronize);

struct hazptr_slot *hazptr_chain_backup_slot(struct hazptr_ctx *ctx)
{
	struct hazptr_overflow_list_flip *overflow_list_flip = this_cpu_ptr(&percpu_overflow_list_flip);
	unsigned int list_idx = (unsigned long) READ_ONCE(hazptr_wildcard) - 1;
	struct hazptr_overflow_list *overflow_list = &overflow_list_flip->array[list_idx];
	struct hazptr_slot *slot = &ctx->backup_slot.slot;

	slot->addr = NULL;
	guard(raw_spinlock_irqsave)(&overflow_list->lock);
	overflow_list->gen++;
	hlist_add_head(&ctx->backup_slot.overflow_node, &overflow_list->head);
	ctx->backup_slot.overflow_list = overflow_list;
	return slot;
}
EXPORT_SYMBOL_GPL(hazptr_chain_backup_slot);

void hazptr_unchain_backup_slot(struct hazptr_ctx *ctx)
{
	struct hazptr_overflow_list *overflow_list = ctx->backup_slot.overflow_list;

	guard(raw_spinlock_irqsave)(&overflow_list->lock);
	overflow_list->gen++;
	hlist_del(&ctx->backup_slot.overflow_node);
}
EXPORT_SYMBOL_GPL(hazptr_unchain_backup_slot);

void __init hazptr_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct hazptr_overflow_list_flip *overflow_list_flip = per_cpu_ptr(&percpu_overflow_list_flip, cpu);

		for (int i = 0; i < 2; i++) {
			raw_spin_lock_init(&overflow_list_flip->array[i].lock);
			INIT_HLIST_HEAD(&overflow_list_flip->array[i].head);
		}
	}
}
