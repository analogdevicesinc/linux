// SPDX-License-Identifier: GPL-2.0
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/osq_lock.h>

/*
 * An MCS like lock especially tailored for optimistic spinning for sleeping
 * lock implementations (mutex, rwsem, etc).
 *
 * Using a single mcs node per CPU is safe because sleeping locks should not be
 * called from interrupt context and we have preemption disabled while
 * spinning.
 */

struct optimistic_spin_node {
	struct optimistic_spin_node *next;
	int prev_cpu; /* CPU number offset by 1 */
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct optimistic_spin_node, osq_node);

/*
 * We use the value 0 to represent "no CPU", thus the encoded value
 * will be the CPU number incremented by 1.
 */
static inline int encode_cpu(int cpu_nr)
{
	return cpu_nr + 1;
}

static inline struct optimistic_spin_node *decode_cpu(int encoded_cpu_val)
{
	int cpu_nr = encoded_cpu_val - 1;

	return per_cpu_ptr(&osq_node, cpu_nr);
}

/*
 * Get a stable @node->next pointer, either for unlock() or unqueue() purposes.
 * Can return NULL in case we were the last queued and we updated @lock instead.
 *
 * If osq_lock() is being cancelled there must be a previous node
 * and 'old_cpu' is its CPU #.
 * For osq_unlock() there is never a previous node and old_cpu is
 * set to OSQ_UNLOCKED_VAL.
 */
static inline struct optimistic_spin_node *
osq_wait_next(struct optimistic_spin_queue *lock,
	      struct optimistic_spin_node *node,
	      int old_cpu)
{
	int curr = encode_cpu(smp_processor_id());

	for (;;) {
		if (atomic_read(&lock->tail) == curr &&
		    atomic_cmpxchg_release(&lock->tail, curr, old_cpu) == curr) {
			/*
			 * We were the last queued, we moved @lock back. @prev
			 * will now observe @lock and will complete its
			 * unlock()/unqueue().
			 */
			return NULL;
		}

		/*
		 * We must xchg() the @node->next value, because if we were to
		 * leave it in, a concurrent unlock()/unqueue() from
		 * @node->next might complete Step-A and think its @prev is
		 * still valid.
		 *
		 * If the concurrent unlock()/unqueue() wins the race, we'll
		 * wait for either @lock to point to us, through its Step-B, or
		 * wait for a new @node->next from its Step-C.
		 */
		if (node->next) {
			struct optimistic_spin_node *next;

			next = xchg(&node->next, NULL);
			if (next)
				return next;
		}

		cpu_relax();
	}
}

/*
 * Set prev->next; this is the counterpart of osq_wait_next() in that
 * by setting ->next the wait is terminated and progress is resumed.
 */
static inline void osq_link_next(struct optimistic_spin_node *prev,
				 struct optimistic_spin_node *next)
{
	/*
	 * Suppose:
	 *
	 *                   tail
	 *                    |
	 *                    V
	 *   CPU1 -> CPU2 -> CPU3:
	 *   n: 2    n: 3    n: nil
	 *   p: nil  p: 1    p: 2
	 *
	 * And this is CPU2 doing the self-unqueue concurrent against CPU1s
	 * unlock()/unqueue(). Since 'prev->next == NULL' and CPU1 will be
	 * stuck in osq_wait_next() until the below store of 'prev->next'.
	 *
	 * CPU2				CPU1
	 *
	 * next->prev = prev;		osq_wait_next()
	 * WMB				MB
	 * prev->next = next;		next->prev = prev
	 *
	 * Without the WMB it would be possible to have conflicting stores
	 * like:
	 *
	 * CPU2				CPU1
	 *
	 * prev->next = next // CPU1.n = 3
	 *				next = osq_wait_next() // = 3
	 *				next->prev = prev // CPU3.p = nil
	 * next->prev = prev // CPU3.p = 1
	 *
	 * Which would result in list corruption.
	 */
	smp_store_release(&prev->next, next);
}

bool osq_lock(struct optimistic_spin_queue *lock)
{
	struct optimistic_spin_node *node = this_cpu_ptr(&osq_node);
	struct optimistic_spin_node *prev, *next;
	int curr = encode_cpu(smp_processor_id());
	int prev_cpu;

	node->next = NULL;

	/*
	 * We need both ACQUIRE (pairs with corresponding RELEASE in
	 * unlock() uncontended, or fastpath) and RELEASE (to publish
	 * the node fields we just initialised) semantics when updating
	 * the lock tail.
	 */
	prev_cpu = atomic_xchg(&lock->tail, curr);
	if (prev_cpu == OSQ_UNLOCKED_VAL)
		return true;

	prev = decode_cpu(prev_cpu);
	node->prev_cpu = prev_cpu;
	osq_link_next(prev, node);

	/*
	 * Normally @prev is untouchable after the above store; because at that
	 * moment unlock can proceed and wipe the node element from stack.
	 *
	 * However, since our nodes are static per-cpu storage, we're
	 * guaranteed their existence -- this allows us to apply
	 * cmpxchg in an attempt to undo our queueing.
	 */

	/*
	 * Wait to acquire the lock or cancellation. Note that need_resched()
	 * will come with an IPI, which will wake smp_cond_load_relaxed() if it
	 * is implemented with a monitor-wait. vcpu_is_preempted() relies on
	 * polling, be careful.
	 */
	prev_cpu = smp_cond_load_relaxed(&node->prev_cpu, !VAL || need_resched() ||
					 vcpu_is_preempted(VAL - 1));

	/* unqueue */
	/*
	 * Step - A  -- stabilize @prev
	 *
	 * Loop until either node->prev_cpu is zero (lock acquired) or we
	 * atomically change prev->next from node to NULL (stopping prev
	 * handing on the lock).
	 * Note that 'prev' can unlink itself concurrently with this
	 * test so that prev/prev_ptr can be stale, but since it
	 * is per-cpu data the memory can always be read.
	 */

	for (;;) {
		if (!prev_cpu) {
			smp_acquire__after_ctrl_dep();
			return true;
		}

		prev = decode_cpu(prev_cpu);

		if (data_race(prev->next) == node &&
		    cmpxchg(&prev->next, node, NULL) == node)
			break;

		/*
		 * 'prev' must have unlinked (or be in the process of unlinking)
		 * itself from the list.
		 */
		cpu_relax();
		prev_cpu = READ_ONCE(node->prev_cpu);
	}

	/*
	 * Step - B -- stabilize @next
	 *
	 * Similar to unlock(), wait for @node->next or move @lock from @node
	 * back to @prev.
	 */

	next = osq_wait_next(lock, node, prev_cpu);
	if (!next)
		return false;

	/*
	 * Step - C -- unlink
	 *
	 * @prev is stable because its still waiting for a new @prev->next
	 * pointer, @next is stable because our @node->next pointer is NULL and
	 * it will wait in Step-A.
	 */

	WRITE_ONCE(next->prev_cpu, prev_cpu);
	osq_link_next(prev, next);

	return false;
}

void osq_unlock(struct optimistic_spin_queue *lock)
{
	struct optimistic_spin_node *node, *next;
	int curr = encode_cpu(smp_processor_id());

	/*
	 * Fast path for the uncontended case.
	 */
	if (atomic_try_cmpxchg_release(&lock->tail, &curr, OSQ_UNLOCKED_VAL))
		return;

	/*
	 * Second most likely case.
	 */
	node = this_cpu_ptr(&osq_node);
	next = xchg(&node->next, NULL);
	if (next) {
		WRITE_ONCE(next->prev_cpu, 0);
		return;
	}

	next = osq_wait_next(lock, node, OSQ_UNLOCKED_VAL);
	if (next)
		WRITE_ONCE(next->prev_cpu, 0);
}
