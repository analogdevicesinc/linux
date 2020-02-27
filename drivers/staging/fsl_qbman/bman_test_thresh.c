/* Copyright 2010-2011 Freescale Semiconductor, Inc.
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

#include "bman_test.h"

/* Test constants */
#define TEST_NUMBUFS	129728
#define TEST_EXIT	129536
#define TEST_ENTRY	129024

struct affine_test_data {
	struct task_struct *t;
	int cpu;
	int expect_affinity;
	int drain;
	int num_enter;
	int num_exit;
	struct list_head node;
	struct completion wakethread;
	struct completion wakeparent;
};

static void cb_depletion(struct bman_portal *portal,
			struct bman_pool *pool,
			void *opaque,
			int depleted)
{
	struct affine_test_data *data = opaque;
	int c = smp_processor_id();
	pr_info("cb_depletion: bpid=%d, depleted=%d, cpu=%d, original=%d\n",
		bman_get_params(pool)->bpid, !!depleted, c, data->cpu);
	/* We should be executing on the CPU of the thread that owns the pool if
	 * and that CPU has an affine portal (ie. it isn't slaved). */
	BUG_ON((c != data->cpu) && data->expect_affinity);
	BUG_ON((c == data->cpu) && !data->expect_affinity);
	if (depleted)
		data->num_enter++;
	else
		data->num_exit++;
}

/* Params used to set up a pool, this also dynamically allocates a BPID */
static const struct bman_pool_params params_nocb = {
	.flags = BMAN_POOL_FLAG_DYNAMIC_BPID | BMAN_POOL_FLAG_THRESH,
	.thresholds = { TEST_ENTRY, TEST_EXIT, 0, 0 }
};

/* Params used to set up each cpu's pool with callbacks enabled */
static struct bman_pool_params params_cb = {
	.bpid = 0, /* will be replaced to match pool_nocb */
	.flags = BMAN_POOL_FLAG_DEPLETION,
	.cb = cb_depletion
};

static struct bman_pool *pool_nocb;
static LIST_HEAD(threads);

static int affine_test(void *__data)
{
	struct bman_pool *pool;
	struct affine_test_data *data = __data;
	struct bman_pool_params my_params = params_cb;

	pr_info("thread %d: starting\n", data->cpu);
	/* create the pool */
	my_params.cb_ctx = data;
	pool = bman_new_pool(&my_params);
	BUG_ON(!pool);
	complete(&data->wakeparent);
	wait_for_completion(&data->wakethread);
	init_completion(&data->wakethread);

	/* if we're the drainer, we get signalled for that */
	if (data->drain) {
		struct bm_buffer buf;
		int ret;
		pr_info("thread %d: draining...\n", data->cpu);
		do {
			ret = bman_acquire(pool, &buf, 1, 0);
		} while (ret > 0);
		pr_info("thread %d: draining done.\n", data->cpu);
		complete(&data->wakeparent);
		wait_for_completion(&data->wakethread);
		init_completion(&data->wakethread);
	}

	/* cleanup */
	bman_free_pool(pool);
	while (!kthread_should_stop())
		cpu_relax();
	pr_info("thread %d: exiting\n", data->cpu);
	return 0;
}

static struct affine_test_data *start_affine_test(int cpu, int drain)
{
	struct affine_test_data *data = kmalloc(sizeof(*data), GFP_KERNEL);

	if (!data)
		return NULL;
	data->cpu = cpu;
	data->expect_affinity = cpumask_test_cpu(cpu, bman_affine_cpus());
	data->drain = drain;
	data->num_enter = 0;
	data->num_exit = 0;
	init_completion(&data->wakethread);
	init_completion(&data->wakeparent);
	list_add_tail(&data->node, &threads);
	data->t = kthread_create(affine_test, data, "threshtest%d", cpu);
	BUG_ON(IS_ERR(data->t));
	kthread_bind(data->t, cpu);
	wake_up_process(data->t);
	return data;
}

void bman_test_thresh(void)
{
	int loop = TEST_NUMBUFS;
	int ret, num_cpus = 0;
	struct affine_test_data *data, *drainer = NULL;

	pr_info("bman_test_thresh: start\n");

	/* allocate a BPID and seed it */
	pool_nocb = bman_new_pool(&params_nocb);
	BUG_ON(!pool_nocb);
	while (loop--) {
		struct bm_buffer buf;
		bm_buffer_set64(&buf, 0x0badbeef + loop);
		ret = bman_release(pool_nocb, &buf, 1,
					BMAN_RELEASE_FLAG_WAIT);
		BUG_ON(ret);
	}
	while (!bman_rcr_is_empty())
		cpu_relax();
	pr_info("bman_test_thresh: buffers are in\n");

	/* create threads and wait for them to create pools */
	params_cb.bpid = bman_get_params(pool_nocb)->bpid;
	for_each_cpu(loop, cpu_online_mask) {
		data = start_affine_test(loop, drainer ? 0 : 1);
		BUG_ON(!data);
		if (!drainer)
			drainer = data;
		num_cpus++;
		wait_for_completion(&data->wakeparent);
	}

	/* signal the drainer to start draining */
	complete(&drainer->wakethread);
	wait_for_completion(&drainer->wakeparent);
	init_completion(&drainer->wakeparent);

	/* tear down */
	list_for_each_entry_safe(data, drainer, &threads, node) {
		complete(&data->wakethread);
		ret = kthread_stop(data->t);
		BUG_ON(ret);
		list_del(&data->node);
		/* check that we get the expected callbacks (and no others) */
		BUG_ON(data->num_enter != 1);
		BUG_ON(data->num_exit != 0);
		kfree(data);
	}
	bman_free_pool(pool_nocb);

	pr_info("bman_test_thresh: done\n");
}
