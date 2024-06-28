// SPDX-License-Identifier: GPL-2.0

#include "io_uring.h"
#include "napi.h"

#ifdef CONFIG_NET_RX_BUSY_POLL

/* Timeout for cleanout of stale entries. */
#define NAPI_TIMEOUT		(60 * SEC_CONVERSION)

struct io_napi_entry {
	unsigned int		napi_id;
	struct list_head	list;

	unsigned long		timeout;
	struct hlist_node	node;

	struct rcu_head		rcu;
};

static struct io_napi_entry *io_napi_hash_find(struct hlist_head *hash_list,
					       unsigned int napi_id)
{
	struct io_napi_entry *e;

	hlist_for_each_entry_rcu(e, hash_list, node) {
		if (e->napi_id != napi_id)
			continue;
		e->timeout = jiffies + NAPI_TIMEOUT;
		return e;
	}

	return NULL;
}

void __io_napi_add(struct io_ring_ctx *ctx, struct socket *sock)
{
	struct hlist_head *hash_list;
	unsigned int napi_id;
	struct sock *sk;
	struct io_napi_entry *e;

	sk = sock->sk;
	if (!sk)
		return;

	napi_id = READ_ONCE(sk->sk_napi_id);

	/* Non-NAPI IDs can be rejected. */
	if (napi_id < MIN_NAPI_ID)
		return;

	hash_list = &ctx->napi_ht[hash_min(napi_id, HASH_BITS(ctx->napi_ht))];

	rcu_read_lock();
	e = io_napi_hash_find(hash_list, napi_id);
	if (e) {
		e->timeout = jiffies + NAPI_TIMEOUT;
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	e = kmalloc(sizeof(*e), GFP_NOWAIT);
	if (!e)
		return;

	e->napi_id = napi_id;
	e->timeout = jiffies + NAPI_TIMEOUT;

	spin_lock(&ctx->napi_lock);
	if (unlikely(io_napi_hash_find(hash_list, napi_id))) {
		spin_unlock(&ctx->napi_lock);
		kfree(e);
		return;
	}

	hlist_add_tail_rcu(&e->node, hash_list);
	list_add_tail(&e->list, &ctx->napi_list);
	spin_unlock(&ctx->napi_lock);
}

static void __io_napi_remove_stale(struct io_ring_ctx *ctx)
{
	struct io_napi_entry *e;
	unsigned int i;

	spin_lock(&ctx->napi_lock);
	hash_for_each(ctx->napi_ht, i, e, node) {
		if (time_after(jiffies, e->timeout)) {
			list_del(&e->list);
			hash_del_rcu(&e->node);
			kfree_rcu(e, rcu);
		}
	}
	spin_unlock(&ctx->napi_lock);
}

static inline void io_napi_remove_stale(struct io_ring_ctx *ctx, bool is_stale)
{
	if (is_stale)
		__io_napi_remove_stale(ctx);
}

static inline bool io_napi_busy_loop_timeout(unsigned long start_time,
					     unsigned long bp_usec)
{
	if (bp_usec) {
		unsigned long end_time = start_time + bp_usec;
		unsigned long now = busy_loop_current_time();

		return time_after(now, end_time);
	}

	return true;
}

static bool io_napi_busy_loop_should_end(void *data,
					 unsigned long start_time)
{
	struct io_wait_queue *iowq = data;

	if (signal_pending(current))
		return true;
	if (io_should_wake(iowq) || io_has_work(iowq->ctx))
		return true;
	if (io_napi_busy_loop_timeout(start_time, iowq->napi_busy_poll_to))
		return true;

	return false;
}

static bool __io_napi_do_busy_loop(struct io_ring_ctx *ctx,
				   void *loop_end_arg)
{
	struct io_napi_entry *e;
	bool (*loop_end)(void *, unsigned long) = NULL;
	bool is_stale = false;

	if (loop_end_arg)
		loop_end = io_napi_busy_loop_should_end;

	list_for_each_entry_rcu(e, &ctx->napi_list, list) {
		napi_busy_loop_rcu(e->napi_id, loop_end, loop_end_arg,
				   ctx->napi_prefer_busy_poll, BUSY_POLL_BUDGET);

		if (time_after(jiffies, e->timeout))
			is_stale = true;
	}

	return is_stale;
}

static void io_napi_blocking_busy_loop(struct io_ring_ctx *ctx,
				       struct io_wait_queue *iowq)
{
	unsigned long start_time = busy_loop_current_time();
	void *loop_end_arg = NULL;
	bool is_stale = false;

	/* Singular lists use a different napi loop end check function and are
	 * only executed once.
	 */
	if (list_is_singular(&ctx->napi_list))
		loop_end_arg = iowq;

	rcu_read_lock();
	do {
		is_stale = __io_napi_do_busy_loop(ctx, loop_end_arg);
	} while (!io_napi_busy_loop_should_end(iowq, start_time) && !loop_end_arg);
	rcu_read_unlock();

	io_napi_remove_stale(ctx, is_stale);
}

/*
 * io_napi_init() - Init napi settings
 * @ctx: pointer to io-uring context structure
 *
 * Init napi settings in the io-uring context.
 */
void io_napi_init(struct io_ring_ctx *ctx)
{
	INIT_LIST_HEAD(&ctx->napi_list);
	spin_lock_init(&ctx->napi_lock);
	ctx->napi_prefer_busy_poll = false;
	ctx->napi_busy_poll_to = READ_ONCE(sysctl_net_busy_poll);
}

/*
 * io_napi_free() - Deallocate napi
 * @ctx: pointer to io-uring context structure
 *
 * Free the napi list and the hash table in the io-uring context.
 */
void io_napi_free(struct io_ring_ctx *ctx)
{
	struct io_napi_entry *e;
	LIST_HEAD(napi_list);
	unsigned int i;

	spin_lock(&ctx->napi_lock);
	hash_for_each(ctx->napi_ht, i, e, node) {
		hash_del_rcu(&e->node);
		kfree_rcu(e, rcu);
	}
	spin_unlock(&ctx->napi_lock);
}

/*
 * io_napi_register() - Register napi with io-uring
 * @ctx: pointer to io-uring context structure
 * @arg: pointer to io_uring_napi structure
 *
 * Register napi in the io-uring context.
 */
int io_register_napi(struct io_ring_ctx *ctx, void __user *arg)
{
	const struct io_uring_napi curr = {
		.busy_poll_to 	  = ctx->napi_busy_poll_to,
		.prefer_busy_poll = ctx->napi_prefer_busy_poll
	};
	struct io_uring_napi napi;

	if (copy_from_user(&napi, arg, sizeof(napi)))
		return -EFAULT;
	if (napi.pad[0] || napi.pad[1] || napi.pad[2] || napi.resv)
		return -EINVAL;

	if (copy_to_user(arg, &curr, sizeof(curr)))
		return -EFAULT;

	WRITE_ONCE(ctx->napi_busy_poll_to, napi.busy_poll_to);
	WRITE_ONCE(ctx->napi_prefer_busy_poll, !!napi.prefer_busy_poll);
	WRITE_ONCE(ctx->napi_enabled, true);
	return 0;
}

/*
 * io_napi_unregister() - Unregister napi with io-uring
 * @ctx: pointer to io-uring context structure
 * @arg: pointer to io_uring_napi structure
 *
 * Unregister napi. If arg has been specified copy the busy poll timeout and
 * prefer busy poll setting to the passed in structure.
 */
int io_unregister_napi(struct io_ring_ctx *ctx, void __user *arg)
{
	const struct io_uring_napi curr = {
		.busy_poll_to 	  = ctx->napi_busy_poll_to,
		.prefer_busy_poll = ctx->napi_prefer_busy_poll
	};

	if (arg && copy_to_user(arg, &curr, sizeof(curr)))
		return -EFAULT;

	WRITE_ONCE(ctx->napi_busy_poll_to, 0);
	WRITE_ONCE(ctx->napi_prefer_busy_poll, false);
	WRITE_ONCE(ctx->napi_enabled, false);
	return 0;
}

/*
 * __io_napi_adjust_timeout() - adjust busy loop timeout
 * @ctx: pointer to io-uring context structure
 * @iowq: pointer to io wait queue
 * @ts: pointer to timespec or NULL
 *
 * Adjust the busy loop timeout according to timespec and busy poll timeout.
 * If the specified NAPI timeout is bigger than the wait timeout, then adjust
 * the NAPI timeout accordingly.
 */
void __io_napi_adjust_timeout(struct io_ring_ctx *ctx, struct io_wait_queue *iowq,
			      struct timespec64 *ts)
{
	unsigned int poll_to = READ_ONCE(ctx->napi_busy_poll_to);

	if (ts) {
		struct timespec64 poll_to_ts;

		poll_to_ts = ns_to_timespec64(1000 * (s64)poll_to);
		if (timespec64_compare(ts, &poll_to_ts) < 0) {
			s64 poll_to_ns = timespec64_to_ns(ts);
			if (poll_to_ns > 0) {
				u64 val = poll_to_ns + 999;
				do_div(val, (s64) 1000);
				poll_to = val;
			}
		}
	}

	iowq->napi_busy_poll_to = poll_to;
}

/*
 * __io_napi_busy_loop() - execute busy poll loop
 * @ctx: pointer to io-uring context structure
 * @iowq: pointer to io wait queue
 *
 * Execute the busy poll loop and merge the spliced off list.
 */
void __io_napi_busy_loop(struct io_ring_ctx *ctx, struct io_wait_queue *iowq)
{
	iowq->napi_prefer_busy_poll = READ_ONCE(ctx->napi_prefer_busy_poll);

	if (!(ctx->flags & IORING_SETUP_SQPOLL) && ctx->napi_enabled)
		io_napi_blocking_busy_loop(ctx, iowq);
}

/*
 * io_napi_sqpoll_busy_poll() - busy poll loop for sqpoll
 * @ctx: pointer to io-uring context structure
 *
 * Splice of the napi list and execute the napi busy poll loop.
 */
int io_napi_sqpoll_busy_poll(struct io_ring_ctx *ctx)
{
	LIST_HEAD(napi_list);
	bool is_stale = false;

	if (!READ_ONCE(ctx->napi_busy_poll_to))
		return 0;
	if (list_empty_careful(&ctx->napi_list))
		return 0;

	rcu_read_lock();
	is_stale = __io_napi_do_busy_loop(ctx, NULL);
	rcu_read_unlock();

	io_napi_remove_stale(ctx, is_stale);
	return 1;
}

#endif
