// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/nospec.h>
#include <linux/io_uring.h>

#include <uapi/linux/io_uring.h>

#include "filetable.h"
#include "sqpoll.h"
#include "fdinfo.h"
#include "io_uring.h"
#include "cancel.h"
#include "rsrc.h"
#include "opdef.h"

#ifdef CONFIG_NET_RX_BUSY_POLL
static __cold void common_tracking_show_fdinfo(struct io_ring_ctx *ctx,
					       struct seq_file *m,
					       const char *tracking_strategy)
{
	seq_puts(m, "NAPI:\tenabled\n");
	seq_printf(m, "napi tracking:\t%s\n", tracking_strategy);
	seq_printf(m, "napi_busy_poll_dt:\t%llu\n", ctx->napi_busy_poll_dt);
	if (ctx->napi_prefer_busy_poll)
		seq_puts(m, "napi_prefer_busy_poll:\ttrue\n");
	else
		seq_puts(m, "napi_prefer_busy_poll:\tfalse\n");
}

static __cold void napi_show_fdinfo(struct io_ring_ctx *ctx,
				    struct seq_file *m)
{
	unsigned int mode = READ_ONCE(ctx->napi_track_mode);

	switch (mode) {
	case IO_URING_NAPI_TRACKING_INACTIVE:
		seq_puts(m, "NAPI:\tdisabled\n");
		break;
	case IO_URING_NAPI_TRACKING_DYNAMIC:
		common_tracking_show_fdinfo(ctx, m, "dynamic");
		break;
	case IO_URING_NAPI_TRACKING_STATIC:
		common_tracking_show_fdinfo(ctx, m, "static");
		break;
	default:
		seq_printf(m, "NAPI:\tunknown mode (%u)\n", mode);
	}
}
#else
static inline void napi_show_fdinfo(struct io_ring_ctx *ctx,
				    struct seq_file *m)
{
}
#endif

static void __io_uring_show_fdinfo(struct io_ring_ctx *ctx, struct seq_file *m)
{
	struct io_overflow_cqe *ocqe;
	struct io_rings *r = ctx->rings;
	unsigned int sq_mask = ctx->sq_entries - 1, cq_mask = ctx->cq_entries - 1;
	unsigned int sq_head = READ_ONCE(r->sq.head);
	unsigned int sq_tail = READ_ONCE(r->sq.tail);
	unsigned int cq_head = READ_ONCE(r->cq.head);
	unsigned int cq_tail = READ_ONCE(r->cq.tail);
	unsigned int sq_shift = 0;
	unsigned int sq_entries;
	int sq_pid = -1, sq_cpu = -1;
	u64 sq_total_time = 0, sq_work_time = 0;
	unsigned int i;

	if (ctx->flags & IORING_SETUP_SQE128)
		sq_shift = 1;

	/*
	 * we may get imprecise sqe and cqe info if uring is actively running
	 * since we get cached_sq_head and cached_cq_tail without uring_lock
	 * and sq_tail and cq_head are changed by userspace. But it's ok since
	 * we usually use these info when it is stuck.
	 */
	seq_printf(m, "SqMask:\t0x%x\n", sq_mask);
	seq_printf(m, "SqHead:\t%u\n", sq_head);
	seq_printf(m, "SqTail:\t%u\n", sq_tail);
	seq_printf(m, "CachedSqHead:\t%u\n", data_race(ctx->cached_sq_head));
	seq_printf(m, "CqMask:\t0x%x\n", cq_mask);
	seq_printf(m, "CqHead:\t%u\n", cq_head);
	seq_printf(m, "CqTail:\t%u\n", cq_tail);
	seq_printf(m, "CachedCqTail:\t%u\n", data_race(ctx->cached_cq_tail));
	seq_printf(m, "SQEs:\t%u\n", sq_tail - sq_head);
	sq_entries = min(sq_tail - sq_head, ctx->sq_entries);
	for (i = 0; i < sq_entries; i++) {
		unsigned int entry = i + sq_head;
		struct io_uring_sqe *sqe;
		unsigned int sq_idx;
		bool sqe128 = false;
		u8 opcode;

		if (ctx->flags & IORING_SETUP_NO_SQARRAY)
			sq_idx = entry & sq_mask;
		else
			sq_idx = READ_ONCE(ctx->sq_array[entry & sq_mask]);
		if (sq_idx > sq_mask)
			continue;

		sqe = &ctx->sq_sqes[sq_idx << sq_shift];
		opcode = READ_ONCE(sqe->opcode);
		if (opcode >= IORING_OP_LAST)
			continue;
		opcode = array_index_nospec(opcode, IORING_OP_LAST);
		if (sq_shift) {
			sqe128 = true;
		} else if (io_issue_defs[opcode].is_128) {
			if (!(ctx->flags & IORING_SETUP_SQE_MIXED)) {
				seq_printf(m,
					"%5u: invalid sqe, 128B entry on non-mixed sq\n",
					sq_idx);
				break;
			}
			if ((++sq_head & sq_mask) == 0) {
				seq_printf(m,
					"%5u: corrupted sqe, wrapping 128B entry\n",
					sq_idx);
				break;
			}
			sqe128 = true;
		}
		seq_printf(m, "%5u: opcode:%s, fd:%d, flags:%x, off:%llu, "
			      "addr:0x%llx, rw_flags:0x%x, buf_index:%d "
			      "user_data:%llu",
			   sq_idx, io_uring_get_opcode(opcode), sqe->fd,
			   sqe->flags, (unsigned long long) sqe->off,
			   (unsigned long long) sqe->addr, sqe->rw_flags,
			   sqe->buf_index, sqe->user_data);
		if (sqe128) {
			u64 *sqeb = (void *) (sqe + 1);
			int size = sizeof(struct io_uring_sqe) / sizeof(u64);
			int j;

			for (j = 0; j < size; j++) {
				seq_printf(m, ", e%d:0x%llx", j,
						(unsigned long long) *sqeb);
				sqeb++;
			}
		}
		seq_printf(m, "\n");
	}
	seq_printf(m, "CQEs:\t%u\n", cq_tail - cq_head);
	while (cq_head < cq_tail) {
		struct io_uring_cqe *cqe;
		bool cqe32 = false;

		cqe = &r->cqes[(cq_head & cq_mask)];
		if (cqe->flags & IORING_CQE_F_32 || ctx->flags & IORING_SETUP_CQE32)
			cqe32 = true;
		seq_printf(m, "%5u: user_data:%llu, res:%d, flags:%x",
			   cq_head & cq_mask, cqe->user_data, cqe->res,
			   cqe->flags);
		if (cqe32)
			seq_printf(m, ", extra1:%llu, extra2:%llu\n",
					cqe->big_cqe[0], cqe->big_cqe[1]);
		seq_printf(m, "\n");
		cq_head++;
		if (cqe32)
			cq_head++;
	}

	if (ctx->flags & IORING_SETUP_SQPOLL) {
		struct io_sq_data *sq = ctx->sq_data;
		struct task_struct *tsk;

		rcu_read_lock();
		tsk = rcu_dereference(sq->thread);
		/*
		 * sq->thread might be NULL if we raced with the sqpoll
		 * thread termination.
		 */
		if (tsk) {
			u64 usec;

			get_task_struct(tsk);
			rcu_read_unlock();
			usec = io_sq_cpu_usec(tsk);
			put_task_struct(tsk);
			sq_pid = sq->task_pid;
			sq_cpu = sq->sq_cpu;
			sq_total_time = usec;
			sq_work_time = sq->work_time;
		} else {
			rcu_read_unlock();
		}
	}

	seq_printf(m, "SqThread:\t%d\n", sq_pid);
	seq_printf(m, "SqThreadCpu:\t%d\n", sq_cpu);
	seq_printf(m, "SqTotalTime:\t%llu\n", sq_total_time);
	seq_printf(m, "SqWorkTime:\t%llu\n", sq_work_time);
	seq_printf(m, "UserFiles:\t%u\n", ctx->file_table.data.nr);
	for (i = 0; i < ctx->file_table.data.nr; i++) {
		struct file *f = NULL;

		if (ctx->file_table.data.nodes[i])
			f = io_slot_file(ctx->file_table.data.nodes[i]);
		if (f) {
			seq_printf(m, "%5u: ", i);
			seq_file_path(m, f, " \t\n\\");
			seq_puts(m, "\n");
		}
	}
	seq_printf(m, "UserBufs:\t%u\n", ctx->buf_table.nr);
	for (i = 0; i < ctx->buf_table.nr; i++) {
		struct io_mapped_ubuf *buf = NULL;

		if (ctx->buf_table.nodes[i])
			buf = ctx->buf_table.nodes[i]->buf;
		if (buf)
			seq_printf(m, "%5u: 0x%llx/%u\n", i, buf->ubuf, buf->len);
		else
			seq_printf(m, "%5u: <none>\n", i);
	}

	seq_puts(m, "PollList:\n");
	for (i = 0; i < (1U << ctx->cancel_table.hash_bits); i++) {
		struct io_hash_bucket *hb = &ctx->cancel_table.hbs[i];
		struct io_kiocb *req;

		hlist_for_each_entry(req, &hb->list, hash_node)
			seq_printf(m, "  op=%d, task_works=%d\n", req->opcode,
					task_work_pending(req->tctx->task));
	}

	seq_puts(m, "CqOverflowList:\n");
	spin_lock(&ctx->completion_lock);
	list_for_each_entry(ocqe, &ctx->cq_overflow_list, list) {
		struct io_uring_cqe *cqe = &ocqe->cqe;

		seq_printf(m, "  user_data=%llu, res=%d, flags=%x\n",
			   cqe->user_data, cqe->res, cqe->flags);

	}
	spin_unlock(&ctx->completion_lock);
	napi_show_fdinfo(ctx, m);
}

static void io_uring_dump_req(const char *prefix, struct io_kiocb *req)
{
	pr_warn("%s: op=%s, flags=0x%llx, user_data=%llu, refs=%d, poll_refs=%d, task=%d, cancel_seq=%d/%d, tw=%ps\n",
		prefix, io_uring_get_opcode(req->opcode),
		(unsigned long long)req->flags, req->cqe.user_data,
		atomic_read(&req->refs), atomic_read(&req->poll_refs),
		req->tctx ? req->tctx->task->pid : -1,
		req->work.cancel_seq, req->cancel_seq_set,
		req->io_task_work.func);
}

static void io_uring_dump_llist(const char *name, struct llist_head *list)
{
	struct io_kiocb *req;

	if (llist_empty(list))
		return;

	pr_warn("  %s:\n", name);
	llist_for_each_entry(req, list->first, io_task_work.node)
		io_uring_dump_req("    req", req);
}

static void io_uring_dump_task_works(struct task_struct *task)
{
	struct callback_head *cb;

	for (cb = READ_ONCE(task->task_works); cb; cb = cb->next)
		pr_warn("      task_work: %ps\n", cb->func);
}

/*
 * Dump pending requests and ring state for debugging stuck rings.
 * Called with uring_lock held.
 */
__cold void io_uring_dump_reqs(struct io_ring_ctx *ctx, const char *prefix)
{
	struct io_defer_entry *de;
	struct io_overflow_cqe *ocqe;
	struct io_hash_bucket *hb;
	struct io_kiocb *req;
	unsigned int i, count;

	lockdep_assert_held(&ctx->uring_lock);

	pr_warn("%s: ring %p, flags=0x%x\n", prefix, ctx, ctx->flags);

	/* Ring state */
	if (ctx->rings) {
		struct io_rings *r = ctx->rings;

		pr_warn("  SQ: head=%u, tail=%u (cached_head=%u)\n",
			READ_ONCE(r->sq.head), READ_ONCE(r->sq.tail),
			ctx->cached_sq_head);
		pr_warn("  CQ: head=%u, tail=%u (cached_tail=%u)\n",
			READ_ONCE(r->cq.head), READ_ONCE(r->cq.tail),
			ctx->cached_cq_tail);
	}

	/* Allocated requests */
	pr_warn("  nr_req_allocated: %u\n", ctx->nr_req_allocated);
	pr_warn("  cancel_seq: %d\n", atomic_read(&ctx->cancel_seq));

	/* Poll requests in cancel_table */
	pr_warn("  PollList:\n");
	for (i = 0; i < (1U << ctx->cancel_table.hash_bits); i++) {
		hb = &ctx->cancel_table.hbs[i];
		hlist_for_each_entry(req, &hb->list, hash_node) {
			struct task_struct *task = req->tctx ? req->tctx->task : NULL;

			io_uring_dump_req("    poll", req);
			if (task && task_work_pending(task))
				io_uring_dump_task_works(task);
		}
	}

	/* Local task_work (DEFER_TASKRUN) */
	io_uring_dump_llist("work_llist", &ctx->work_llist);
	io_uring_dump_llist("retry_llist", &ctx->retry_llist);
	io_uring_dump_llist("fallback_llist", &ctx->fallback_llist);

	/* Deferred requests */
	if (!list_empty(&ctx->defer_list)) {
		pr_warn("  defer_list:\n");
		list_for_each_entry(de, &ctx->defer_list, list)
			io_uring_dump_req("    req", de->req);
	}

	/* Overflow */
	count = 0;
	spin_lock(&ctx->completion_lock);
	list_for_each_entry(ocqe, &ctx->cq_overflow_list, list)
		count++;
	spin_unlock(&ctx->completion_lock);
	if (count)
		pr_warn("  cq_overflow_list: %u CQEs\n", count);
}

/*
 * Caller holds a reference to the file already, we don't need to do
 * anything else to get an extra reference.
 */
__cold void io_uring_show_fdinfo(struct seq_file *m, struct file *file)
{
	struct io_ring_ctx *ctx = file->private_data;

	/*
	 * Avoid ABBA deadlock between the seq lock and the io_uring mutex,
	 * since fdinfo case grabs it in the opposite direction of normal use
	 * cases.
	 */
	if (mutex_trylock(&ctx->uring_lock)) {
		__io_uring_show_fdinfo(ctx, m);
		mutex_unlock(&ctx->uring_lock);
	}
}
