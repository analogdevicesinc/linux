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

#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include "qman_test.h"

/* Algorithm:
 *
 * Each cpu will have HP_PER_CPU "handlers" set up, each of which incorporates
 * an rx/tx pair of FQ objects (both of which are stashed on dequeue). The
 * organisation of FQIDs is such that the HP_PER_CPU*NUM_CPUS handlers will
 * shuttle a "hot potato" frame around them such that every forwarding action
 * moves it from one cpu to another. (The use of more than one handler per cpu
 * is to allow enough handlers/FQs to truly test the significance of caching -
 * ie. when cache-expiries are occurring.)
 *
 * The "hot potato" frame content will be HP_NUM_WORDS*4 bytes in size, and the
 * first and last words of the frame data will undergo a transformation step on
 * each forwarding action. To achieve this, each handler will be assigned a
 * 32-bit "mixer", that is produced using a 32-bit LFSR. When a frame is
 * received by a handler, the mixer of the expected sender is XOR'd into all
 * words of the entire frame, which is then validated against the original
 * values. Then, before forwarding, the entire frame is XOR'd with the mixer of
 * the current handler. Apart from validating that the frame is taking the
 * expected path, this also provides some quasi-realistic overheads to each
 * forwarding action - dereferencing *all* the frame data, computation, and
 * conditional branching. There is a "special" handler designated to act as the
 * instigator of the test by creating an enqueuing the "hot potato" frame, and
 * to determine when the test has completed by counting HP_LOOPS iterations.
 *
 * Init phases:
 *
 * 1. prepare each cpu's 'hp_cpu' struct using on_each_cpu(,,1) and link them
 *    into 'hp_cpu_list'. Specifically, set processor_id, allocate HP_PER_CPU
 *    handlers and link-list them (but do no other handler setup).
 *
 * 2. scan over 'hp_cpu_list' HP_PER_CPU times, the first time sets each
 *    hp_cpu's 'iterator' to point to its first handler. With each loop,
 *    allocate rx/tx FQIDs and mixer values to the hp_cpu's iterator handler
 *    and advance the iterator for the next loop. This includes a final fixup,
 *    which connects the last handler to the first (and which is why phase 2
 *    and 3 are separate).
 *
 * 3. scan over 'hp_cpu_list' HP_PER_CPU times, the first time sets each
 *    hp_cpu's 'iterator' to point to its first handler. With each loop,
 *    initialise FQ objects and advance the iterator for the next loop.
 *    Moreover, do this initialisation on the cpu it applies to so that Rx FQ
 *    initialisation targets the correct cpu.
 */

/* helper to run something on all cpus (can't use on_each_cpu(), as that invokes
 * the fn from irq context, which is too restrictive). */
struct bstrap {
	void (*fn)(void);
	atomic_t started;
};
static int bstrap_fn(void *__bstrap)
{
	struct bstrap *bstrap = __bstrap;
	atomic_inc(&bstrap->started);
	bstrap->fn();
	while (!kthread_should_stop())
		msleep(1);
	return 0;
}
static int on_all_cpus(void (*fn)(void))
{
	int cpu;
	for_each_cpu(cpu, cpu_online_mask) {
		struct bstrap bstrap = {
			.fn = fn,
			.started = ATOMIC_INIT(0)
		};
		struct task_struct *k = kthread_create(bstrap_fn, &bstrap,
			"hotpotato%d", cpu);
		int ret;
		if (IS_ERR(k))
			return -ENOMEM;
		kthread_bind(k, cpu);
		wake_up_process(k);
		/* If we call kthread_stop() before the "wake up" has had an
		 * effect, then the thread may exit with -EINTR without ever
		 * running the function. So poll until it's started before
		 * requesting it to stop. */
		while (!atomic_read(&bstrap.started))
			msleep(10);
		ret = kthread_stop(k);
		if (ret)
			return ret;
	}
	return 0;
}

struct hp_handler {

	/* The following data is stashed when 'rx' is dequeued; */
	/* -------------- */
	/* The Rx FQ, dequeues of which will stash the entire hp_handler */
	struct qman_fq rx;
	/* The Tx FQ we should forward to */
	struct qman_fq tx;
	/* The value we XOR post-dequeue, prior to validating */
	u32 rx_mixer;
	/* The value we XOR pre-enqueue, after validating */
	u32 tx_mixer;
	/* what the hotpotato address should be on dequeue */
	dma_addr_t addr;
	u32 *frame_ptr;

	/* The following data isn't (necessarily) stashed on dequeue; */
	/* -------------- */
	u32 fqid_rx, fqid_tx;
	/* list node for linking us into 'hp_cpu' */
	struct list_head node;
	/* Just to check ... */
	unsigned int processor_id;
} ____cacheline_aligned;

struct hp_cpu {
	/* identify the cpu we run on; */
	unsigned int processor_id;
	/* root node for the per-cpu list of handlers */
	struct list_head handlers;
	/* list node for linking us into 'hp_cpu_list' */
	struct list_head node;
	/* when repeatedly scanning 'hp_list', each time linking the n'th
	 * handlers together, this is used as per-cpu iterator state */
	struct hp_handler *iterator;
};

/* Each cpu has one of these */
static DEFINE_PER_CPU(struct hp_cpu, hp_cpus);

/* links together the hp_cpu structs, in first-come first-serve order. */
static LIST_HEAD(hp_cpu_list);
static spinlock_t hp_lock = __SPIN_LOCK_UNLOCKED(hp_lock);

static unsigned int hp_cpu_list_length;

/* the "special" handler, that starts and terminates the test. */
static struct hp_handler *special_handler;
static int loop_counter;

/* handlers are allocated out of this, so they're properly aligned. */
static struct kmem_cache *hp_handler_slab;

/* this is the frame data */
static void *__frame_ptr;
static u32 *frame_ptr;
static dma_addr_t frame_dma;

/* the main function waits on this */
static DECLARE_WAIT_QUEUE_HEAD(queue);

#define HP_PER_CPU	2
#define HP_LOOPS	8
/* 80 bytes, like a small ethernet frame, and bleeds into a second cacheline */
#define HP_NUM_WORDS	80
/* First word of the LFSR-based frame data */
#define HP_FIRST_WORD	0xabbaf00d

static inline u32 do_lfsr(u32 prev)
{
	return (prev >> 1) ^ (-(prev & 1u) & 0xd0000001u);
}

static void allocate_frame_data(void)
{
	u32 lfsr = HP_FIRST_WORD;
	int loop;
	struct platform_device *pdev = platform_device_alloc("foobar", -1);
	if (!pdev)
		panic("platform_device_alloc() failed");
	if (platform_device_add(pdev))
		panic("platform_device_add() failed");
	__frame_ptr = kmalloc(4 * HP_NUM_WORDS, GFP_KERNEL);
	if (!__frame_ptr)
		panic("kmalloc() failed");
	frame_ptr = (void *)(((unsigned long)__frame_ptr + 63) &
				~(unsigned long)63);
	for (loop = 0; loop < HP_NUM_WORDS; loop++) {
		frame_ptr[loop] = lfsr;
		lfsr = do_lfsr(lfsr);
	}
	frame_dma = dma_map_single(&pdev->dev, frame_ptr, 4 * HP_NUM_WORDS,
					DMA_BIDIRECTIONAL);
	platform_device_del(pdev);
	platform_device_put(pdev);
}

static void deallocate_frame_data(void)
{
	kfree(__frame_ptr);
}

static inline void process_frame_data(struct hp_handler *handler,
				const struct qm_fd *fd)
{
	u32 *p = handler->frame_ptr;
	u32 lfsr = HP_FIRST_WORD;
	int loop;
	if (qm_fd_addr_get64(fd) != (handler->addr & 0xffffffffff)) {
		pr_err("Got 0x%llx expected 0x%llx\n",
		       qm_fd_addr_get64(fd), handler->addr);
		panic("bad frame address");
	}
	for (loop = 0; loop < HP_NUM_WORDS; loop++, p++) {
		*p ^= handler->rx_mixer;
		if (*p != lfsr)
			panic("corrupt frame data");
		*p ^= handler->tx_mixer;
		lfsr = do_lfsr(lfsr);
	}
}

static enum qman_cb_dqrr_result normal_dqrr(struct qman_portal *portal,
					struct qman_fq *fq,
					const struct qm_dqrr_entry *dqrr)
{
	struct hp_handler *handler = (struct hp_handler *)fq;

	process_frame_data(handler, &dqrr->fd);
	if (qman_enqueue(&handler->tx, &dqrr->fd, 0))
		panic("qman_enqueue() failed");
	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result special_dqrr(struct qman_portal *portal,
					struct qman_fq *fq,
					const struct qm_dqrr_entry *dqrr)
{
	struct hp_handler *handler = (struct hp_handler *)fq;

	process_frame_data(handler, &dqrr->fd);
	if (++loop_counter < HP_LOOPS) {
		if (qman_enqueue(&handler->tx, &dqrr->fd, 0))
			panic("qman_enqueue() failed");
	} else {
		pr_info("Received final (%dth) frame\n", loop_counter);
		wake_up(&queue);
	}
	return qman_cb_dqrr_consume;
}

static void create_per_cpu_handlers(void)
{
	struct hp_handler *handler;
	int loop;
	struct hp_cpu *hp_cpu = &get_cpu_var(hp_cpus);

	hp_cpu->processor_id = smp_processor_id();
	spin_lock(&hp_lock);
	list_add_tail(&hp_cpu->node, &hp_cpu_list);
	hp_cpu_list_length++;
	spin_unlock(&hp_lock);
	INIT_LIST_HEAD(&hp_cpu->handlers);
	for (loop = 0; loop < HP_PER_CPU; loop++) {
		handler = kmem_cache_alloc(hp_handler_slab, GFP_KERNEL);
		if (!handler)
			panic("kmem_cache_alloc() failed");
		handler->processor_id = hp_cpu->processor_id;
		handler->addr = frame_dma;
		handler->frame_ptr = frame_ptr;
		list_add_tail(&handler->node, &hp_cpu->handlers);
	}
	put_cpu_var(hp_cpus);
}

static void destroy_per_cpu_handlers(void)
{
	struct list_head *loop, *tmp;
	struct hp_cpu *hp_cpu = &get_cpu_var(hp_cpus);

	spin_lock(&hp_lock);
	list_del(&hp_cpu->node);
	spin_unlock(&hp_lock);
	list_for_each_safe(loop, tmp, &hp_cpu->handlers) {
		u32 flags;
		struct hp_handler *handler = list_entry(loop, struct hp_handler,
							node);
		if (qman_retire_fq(&handler->rx, &flags))
			panic("qman_retire_fq(rx) failed");
		BUG_ON(flags & QMAN_FQ_STATE_BLOCKOOS);
		if (qman_oos_fq(&handler->rx))
			panic("qman_oos_fq(rx) failed");
		qman_destroy_fq(&handler->rx, 0);
		qman_destroy_fq(&handler->tx, 0);
		qman_release_fqid(handler->fqid_rx);
		list_del(&handler->node);
		kmem_cache_free(hp_handler_slab, handler);
	}
	put_cpu_var(hp_cpus);
}

static inline u8 num_cachelines(u32 offset)
{
	u8 res = (offset + (L1_CACHE_BYTES - 1))
			 / (L1_CACHE_BYTES);
	if (res > 3)
		return 3;
	return res;
}
#define STASH_DATA_CL \
	num_cachelines(HP_NUM_WORDS * 4)
#define STASH_CTX_CL \
	num_cachelines(offsetof(struct hp_handler, fqid_rx))

static void init_handler(void *__handler)
{
	struct qm_mcc_initfq opts;
	struct hp_handler *handler = __handler;
	BUG_ON(handler->processor_id != smp_processor_id());
	/* Set up rx */
	memset(&handler->rx, 0, sizeof(handler->rx));
	if (handler == special_handler)
		handler->rx.cb.dqrr = special_dqrr;
	else
		handler->rx.cb.dqrr = normal_dqrr;
	if (qman_create_fq(handler->fqid_rx, 0, &handler->rx))
		panic("qman_create_fq(rx) failed");
	memset(&opts, 0, sizeof(opts));
	opts.we_mask = QM_INITFQ_WE_FQCTRL | QM_INITFQ_WE_CONTEXTA;
	opts.fqd.fq_ctrl = QM_FQCTRL_CTXASTASHING;
	opts.fqd.context_a.stashing.data_cl = STASH_DATA_CL;
	opts.fqd.context_a.stashing.context_cl = STASH_CTX_CL;
	if (qman_init_fq(&handler->rx, QMAN_INITFQ_FLAG_SCHED |
				QMAN_INITFQ_FLAG_LOCAL, &opts))
		panic("qman_init_fq(rx) failed");
	/* Set up tx */
	memset(&handler->tx, 0, sizeof(handler->tx));
	if (qman_create_fq(handler->fqid_tx, QMAN_FQ_FLAG_NO_MODIFY,
				&handler->tx))
		panic("qman_create_fq(tx) failed");
}

static void init_phase2(void)
{
	int loop;
	u32 fqid = 0;
	u32 lfsr = 0xdeadbeef;
	struct hp_cpu *hp_cpu;
	struct hp_handler *handler;

	for (loop = 0; loop < HP_PER_CPU; loop++) {
		list_for_each_entry(hp_cpu, &hp_cpu_list, node) {
			int ret;
			if (!loop)
				hp_cpu->iterator = list_first_entry(
						&hp_cpu->handlers,
						struct hp_handler, node);
			else
				hp_cpu->iterator = list_entry(
						hp_cpu->iterator->node.next,
						struct hp_handler, node);
			/* Rx FQID is the previous handler's Tx FQID */
			hp_cpu->iterator->fqid_rx = fqid;
			/* Allocate new FQID for Tx */
			ret = qman_alloc_fqid(&fqid);
			if (ret)
				panic("qman_alloc_fqid() failed");
			hp_cpu->iterator->fqid_tx = fqid;
			/* Rx mixer is the previous handler's Tx mixer */
			hp_cpu->iterator->rx_mixer = lfsr;
			/* Get new mixer for Tx */
			lfsr = do_lfsr(lfsr);
			hp_cpu->iterator->tx_mixer = lfsr;
		}
	}
	/* Fix up the first handler (fqid_rx==0, rx_mixer=0xdeadbeef) */
	hp_cpu = list_first_entry(&hp_cpu_list, struct hp_cpu, node);
	handler = list_first_entry(&hp_cpu->handlers, struct hp_handler, node);
	BUG_ON((handler->fqid_rx != 0) || (handler->rx_mixer != 0xdeadbeef));
	handler->fqid_rx = fqid;
	handler->rx_mixer = lfsr;
	/* and tag it as our "special" handler */
	special_handler = handler;
}

static void init_phase3(void)
{
	int loop;
	struct hp_cpu *hp_cpu;

	for (loop = 0; loop < HP_PER_CPU; loop++) {
		list_for_each_entry(hp_cpu, &hp_cpu_list, node) {
			if (!loop)
				hp_cpu->iterator = list_first_entry(
						&hp_cpu->handlers,
						struct hp_handler, node);
			else
				hp_cpu->iterator = list_entry(
						hp_cpu->iterator->node.next,
						struct hp_handler, node);
			preempt_disable();
			if (hp_cpu->processor_id == smp_processor_id())
				init_handler(hp_cpu->iterator);
			else
				smp_call_function_single(hp_cpu->processor_id,
					init_handler, hp_cpu->iterator, 1);
			preempt_enable();
		}
	}
}

static void send_first_frame(void *ignore)
{
	u32 *p = special_handler->frame_ptr;
	u32 lfsr = HP_FIRST_WORD;
	int loop;
	struct qm_fd fd;

	BUG_ON(special_handler->processor_id != smp_processor_id());
	memset(&fd, 0, sizeof(fd));
	qm_fd_addr_set64(&fd, special_handler->addr);
	fd.format = qm_fd_contig_big;
	fd.length29 = HP_NUM_WORDS * 4;
	for (loop = 0; loop < HP_NUM_WORDS; loop++, p++) {
		if (*p != lfsr)
			panic("corrupt frame data");
		*p ^= special_handler->tx_mixer;
		lfsr = do_lfsr(lfsr);
	}
	pr_info("Sending first frame\n");
	if (qman_enqueue(&special_handler->tx, &fd, 0))
		panic("qman_enqueue() failed");
}

void qman_test_hotpotato(void)
{
	if (cpumask_weight(cpu_online_mask) < 2) {
		pr_info("qman_test_hotpotato, skip - only 1 CPU\n");
		return;
	}

	pr_info("qman_test_hotpotato starting\n");

	hp_cpu_list_length = 0;
	loop_counter = 0;
	hp_handler_slab = kmem_cache_create("hp_handler_slab",
			sizeof(struct hp_handler), L1_CACHE_BYTES,
			SLAB_HWCACHE_ALIGN, NULL);
	if (!hp_handler_slab)
		panic("kmem_cache_create() failed");

	allocate_frame_data();

	/* Init phase 1 */
	pr_info("Creating %d handlers per cpu...\n", HP_PER_CPU);
	if (on_all_cpus(create_per_cpu_handlers))
		panic("on_each_cpu() failed");
	pr_info("Number of cpus: %d, total of %d handlers\n",
		hp_cpu_list_length, hp_cpu_list_length * HP_PER_CPU);

	init_phase2();

	init_phase3();

	preempt_disable();
	if (special_handler->processor_id == smp_processor_id())
		send_first_frame(NULL);
	else
		smp_call_function_single(special_handler->processor_id,
			send_first_frame, NULL, 1);
	preempt_enable();

	wait_event(queue, loop_counter == HP_LOOPS);
	deallocate_frame_data();
	if (on_all_cpus(destroy_per_cpu_handlers))
		panic("on_each_cpu() failed");
	kmem_cache_destroy(hp_handler_slab);
	pr_info("qman_test_hotpotato finished\n");
}
