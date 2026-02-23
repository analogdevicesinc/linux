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
	struct mutex lock;		/* Mutex protecting add_idx from concurrent updates. */
	unsigned int add_idx;		/* Index of current flip-list to add to. */
	struct hazptr_overflow_list array[2];
};

static DEFINE_PER_CPU(struct hazptr_overflow_list_flip, percpu_overflow_list_flip);

DEFINE_PER_CPU(struct hazptr_percpu_slots, hazptr_percpu_slots);
EXPORT_PER_CPU_SYMBOL_GPL(hazptr_percpu_slots);

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

			if (load_addr != addr && load_addr != HAZPTR_WILDCARD)
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
void hazptr_synchronize_cpu_slots(int cpu, void *addr)
{
	struct hazptr_percpu_slots *percpu_slots = per_cpu_ptr(&hazptr_percpu_slots, cpu);
	unsigned int idx;

	for (idx = 0; idx < NR_HAZPTR_PERCPU_SLOTS; idx++) {
		struct hazptr_slot_item *item = &percpu_slots->items[idx];

		/* Busy-wait if node is found. */
		smp_cond_load_acquire(&item->slot.addr, VAL != addr && VAL != HAZPTR_WILDCARD); /* Load B */
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
	int cpu;

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
	/* Scan all CPUs slots. */
	for_each_possible_cpu(cpu) {
		struct hazptr_overflow_list_flip *overflow_list_flip = per_cpu_ptr(&percpu_overflow_list_flip, cpu);
		unsigned int scan_idx;

		/* Scan CPU slots. */
		hazptr_synchronize_cpu_slots(cpu, addr);

		/*
		 * Scan backup slots in percpu overflow lists.
		 * Forward progress is guaranteed by scanning one list
		 * while new elements are added into the other list.
		 */
		guard(mutex)(&overflow_list_flip->lock);
		scan_idx = overflow_list_flip->add_idx ^ 1;
		hazptr_synchronize_overflow_list(&overflow_list_flip->array[scan_idx], addr);
		/* Flip current list. */
		WRITE_ONCE(overflow_list_flip->add_idx, scan_idx);
		hazptr_synchronize_overflow_list(&overflow_list_flip->array[scan_idx ^ 1], addr);
	}
}
EXPORT_SYMBOL_GPL(hazptr_synchronize);

struct hazptr_slot *hazptr_chain_backup_slot(struct hazptr_ctx *ctx)
{
	struct hazptr_overflow_list_flip *overflow_list_flip = this_cpu_ptr(&percpu_overflow_list_flip);
	unsigned int list_idx = READ_ONCE(overflow_list_flip->add_idx);
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

		mutex_init(&overflow_list_flip->lock);
		for (int i = 0; i < 2; i++) {
			raw_spin_lock_init(&overflow_list_flip->array[i].lock);
			INIT_HLIST_HEAD(&overflow_list_flip->array[i].head);
		}
	}
}
