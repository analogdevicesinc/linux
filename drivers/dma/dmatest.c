/*
 * DMA Engine test module
 *
 * Copyright (C) 2007 Atmel Corporation
 * Copyright (C) 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/freezer.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/sched/task.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>

static unsigned int test_buf_size = 16384;
module_param(test_buf_size, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(test_buf_size, "Size of the memcpy test buffer");

static char test_channel[20];
module_param_string(channel, test_channel, sizeof(test_channel),
		S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(channel, "Bus ID of the channel to test (default: any)");

static char test_device[32];
module_param_string(device, test_device, sizeof(test_device),
		S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(device, "Bus ID of the DMA Engine to test (default: any)");

static unsigned int threads_per_chan = 1;
module_param(threads_per_chan, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(threads_per_chan,
		"Number of threads to start per channel (default: 1)");

static unsigned int max_channels;
module_param(max_channels, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_channels,
		"Maximum number of channels to use (default: all)");

static unsigned int iterations;
module_param(iterations, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(iterations,
		"Iterations before stopping test (default: infinite)");

static int param_set_dmatest_type(const char *val,
				  const struct kernel_param *kp);
static int param_get_dmatest_type(char *buffer,
				  const struct kernel_param *kp);
#define param_check_dmatest_type(a, b)
const struct kernel_param_ops param_ops_dmatest_type = {
	.set = param_set_dmatest_type,
	.get = param_get_dmatest_type,
};

static unsigned int dmatest = BIT(DMA_MEMCPY);
module_param(dmatest, dmatest_type, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dmatest,
		"dmatest type (default: memcpy)");

static unsigned int xor_sources = 3;
module_param(xor_sources, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(xor_sources,
		"Number of xor source buffers (default: 3)");

static unsigned int pq_sources = 3;
module_param(pq_sources, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pq_sources,
		"Number of p+q source buffers (default: 3)");

static int timeout = 3000;
module_param(timeout, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timeout, "Transfer Timeout in msec (default: 3000), "
		 "Pass -1 for infinite timeout");

static bool noverify;
module_param(noverify, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(noverify, "Disable data verification (default: verify)");

static bool norandom;
module_param(norandom, bool, 0644);
MODULE_PARM_DESC(norandom, "Disable random offset setup (default: random)");

static bool verbose;
module_param(verbose, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(verbose, "Enable \"success\" result messages (default: off)");

/**
 * struct dmatest_params - test parameters.
 * @buf_size:		size of the memcpy test buffer
 * @channel:		bus ID of the channel to test
 * @device:		bus ID of the DMA Engine to test
 * @threads_per_chan:	number of threads to start per channel
 * @max_channels:	maximum number of channels to use
 * @iterations:		iterations before stopping test
 * @xor_sources:	number of xor source buffers
 * @pq_sources:		number of p+q source buffers
 * @timeout:		transfer timeout in msec, -1 for infinite timeout
 */
struct dmatest_params {
	unsigned int	buf_size;
	char		channel[20];
	char		device[32];
	u8		align;
	unsigned int	threads_per_chan;
	unsigned int	max_channels;
	unsigned int	iterations;
	unsigned int	xor_sources;
	unsigned int	pq_sources;
	int		timeout;
	bool		noverify;
	bool		norandom;
};

/**
 * struct dmatest_info - test information.
 * @params:		test parameters
 * @lock:		access protection to the fields of this structure
 */
static struct dmatest_info {
	/* Test parameters */
	struct dmatest_params	params;

	/* Internal state */
	struct list_head	channels;
	unsigned int		nr_channels;
	struct mutex		lock;
	bool			did_init;
} test_info = {
	.channels = LIST_HEAD_INIT(test_info.channels),
	.lock = __MUTEX_INITIALIZER(test_info.lock),
};

static int dmatest_run_set(const char *val, const struct kernel_param *kp);
static int dmatest_run_get(char *val, const struct kernel_param *kp);
static const struct kernel_param_ops run_ops = {
	.set = dmatest_run_set,
	.get = dmatest_run_get,
};
static bool dmatest_run;
module_param_cb(run, &run_ops, &dmatest_run, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(run, "Run the test (default: false)");

/* Maximum amount of mismatched bytes in buffer to print */
#define MAX_ERROR_COUNT		32

/*
 * Initialization patterns. All bytes in the source buffer has bit 7
 * set, all bytes in the destination buffer has bit 7 cleared.
 *
 * Bit 6 is set for all bytes which are to be copied by the DMA
 * engine. Bit 5 is set for all bytes which are to be overwritten by
 * the DMA engine.
 *
 * The remaining bits are the inverse of a counter which increments by
 * one for each byte address.
 */
#define PATTERN_SRC		0x80
#define PATTERN_DST		0x00
#define PATTERN_COPY		0x40
#define PATTERN_OVERWRITE	0x20
#define PATTERN_COUNT_MASK	0x1f
#define PATTERN_MEMSET_IDX	0x01

/* poor man's completion - we want to use wait_event_freezable() on it */
struct dmatest_done {
	bool			done;
	wait_queue_head_t	*wait;
};

struct dmatest_data {
	u8		**raw;
	u8		**aligned;
	dma_addr_t	*dma_addrs;
	unsigned int	cnt;
	unsigned int	off;
	struct device	*dev;
	struct dmaengine_unmap_data *um;
	enum dma_data_direction	dir;
};

struct dmatest_xfer {
	struct dma_async_tx_descriptor	*tx;
	struct dma_chan			*chan;
	dma_cookie_t			cookie;
};

struct dmatest_thread {
	struct list_head	node;
	struct dmatest_info	*info;
	struct task_struct	*task;
	struct dma_chan		*chan;
	struct dmatest_data	src;
	struct dmatest_data	dst;
	struct dmatest_xfer	xfer;
	enum dma_transaction_type type;
	wait_queue_head_t done_wait;
	struct dmatest_done test_done;
	bool			done;
};

struct dmatest_chan {
	struct list_head	node;
	struct dma_chan		*chan;
	struct list_head	threads;
};

static DECLARE_WAIT_QUEUE_HEAD(thread_wait);
static bool wait;

const static char *transaction_type_names[DMA_TX_TYPE_END] = {
	[DMA_MEMCPY]     = "memcpy",
	[DMA_XOR]        = "xor",
	[DMA_PQ]         = "pq",
	[DMA_MEMSET]     = "memset",
};

static int find_string(const char *name, const char *names[], int names_cnt)
{
	int i;
	if (!name || !names || names_cnt <= 0)
		return -EINVAL;
	for (i = 0; i < names_cnt; i++) {
		if (!names[i])
			continue;
		if (!strcmp(names[i], name))
			return i;
	}
	return -ENOENT;
}

static bool is_threaded_test_run(struct dmatest_info *info)
{
	struct dmatest_chan *dtc;

	list_for_each_entry(dtc, &info->channels, node) {
		struct dmatest_thread *thread;

		list_for_each_entry(thread, &dtc->threads, node) {
			if (!thread->done)
				return true;
		}
	}

	return false;
}

static int dmatest_wait_get(char *val, const struct kernel_param *kp)
{
	struct dmatest_info *info = &test_info;
	struct dmatest_params *params = &info->params;

	if (params->iterations)
		wait_event(thread_wait, !is_threaded_test_run(info));
	wait = true;
	return param_get_bool(val, kp);
}

static const struct kernel_param_ops wait_ops = {
	.get = dmatest_wait_get,
	.set = param_set_bool,
};
module_param_cb(wait, &wait_ops, &wait, S_IRUGO);
MODULE_PARM_DESC(wait, "Wait for tests to complete (default: false)");

static bool dmatest_match_channel(struct dmatest_params *params,
		struct dma_chan *chan)
{
	if (params->channel[0] == '\0')
		return true;
	return strcmp(dma_chan_name(chan), params->channel) == 0;
}

static bool dmatest_match_device(struct dmatest_params *params,
		struct dma_device *device)
{
	if (params->device[0] == '\0')
		return true;
	return strcmp(dev_name(device->dev), params->device) == 0;
}

static unsigned long dmatest_random(void)
{
	unsigned long buf;

	prandom_bytes(&buf, sizeof(buf));
	return buf;
}

static inline u8 gen_inv_idx(u8 index, bool is_memset)
{
	u8 val = is_memset ? PATTERN_MEMSET_IDX : index;

	return ~val & PATTERN_COUNT_MASK;
}

static inline u8 gen_src_value(u8 index, bool is_memset)
{
	return PATTERN_SRC | gen_inv_idx(index, is_memset);
}

static inline u8 gen_dst_value(u8 index, bool is_memset)
{
	return PATTERN_DST | gen_inv_idx(index, is_memset);
}

static void dmatest_init_srcs(u8 **bufs, unsigned int start, unsigned int len,
		unsigned int buf_size, bool is_memset)
{
	unsigned int i;
	u8 *buf;

	for (; (buf = *bufs); bufs++) {
		for (i = 0; i < start; i++)
			buf[i] = gen_src_value(i, is_memset);
		for ( ; i < start + len; i++)
			buf[i] = gen_src_value(i, is_memset) | PATTERN_COPY;
		for ( ; i < buf_size; i++)
			buf[i] = gen_src_value(i, is_memset);
		buf++;
	}
}

static void dmatest_init_dsts(u8 **bufs, unsigned int start, unsigned int len,
		unsigned int buf_size, bool is_memset)
{
	unsigned int i;
	u8 *buf;

	for (; (buf = *bufs); bufs++) {
		for (i = 0; i < start; i++)
			buf[i] = gen_dst_value(i, is_memset);
		for ( ; i < start + len; i++)
			buf[i] = gen_dst_value(i, is_memset) |
						PATTERN_OVERWRITE;
		for ( ; i < buf_size; i++)
			buf[i] = gen_dst_value(i, is_memset);
	}
}

static void dmatest_mismatch(u8 actual, u8 pattern, unsigned int index,
		unsigned int counter, bool is_srcbuf, bool is_memset)
{
	u8		diff = actual ^ pattern;
	u8		expected = pattern | gen_inv_idx(counter, is_memset);
	const char	*thread_name = current->comm;

	if (is_srcbuf)
		pr_warn("%s: srcbuf[0x%x] overwritten! Expected %02x, got %02x\n",
			thread_name, index, expected, actual);
	else if ((pattern & PATTERN_COPY)
			&& (diff & (PATTERN_COPY | PATTERN_OVERWRITE)))
		pr_warn("%s: dstbuf[0x%x] not copied! Expected %02x, got %02x\n",
			thread_name, index, expected, actual);
	else if (diff & PATTERN_SRC)
		pr_warn("%s: dstbuf[0x%x] was copied! Expected %02x, got %02x\n",
			thread_name, index, expected, actual);
	else
		pr_warn("%s: dstbuf[0x%x] mismatch! Expected %02x, got %02x\n",
			thread_name, index, expected, actual);
}

static unsigned int dmatest_verify(u8 **bufs, unsigned int start,
		unsigned int end, unsigned int counter, u8 pattern,
		bool is_srcbuf, bool is_memset)
{
	unsigned int i;
	unsigned int error_count = 0;
	u8 actual;
	u8 expected;
	u8 *buf;
	unsigned int counter_orig = counter;

	for (; (buf = *bufs); bufs++) {
		counter = counter_orig;
		for (i = start; i < end; i++) {
			actual = buf[i];
			expected = pattern | gen_inv_idx(counter, is_memset);
			if (actual != expected) {
				if (error_count < MAX_ERROR_COUNT)
					dmatest_mismatch(actual, pattern, i,
							 counter, is_srcbuf,
							 is_memset);
				error_count++;
			}
			counter++;
		}
	}

	if (error_count > MAX_ERROR_COUNT)
		pr_warn("%s: %u errors suppressed\n",
			current->comm, error_count - MAX_ERROR_COUNT);

	return error_count;
}


static void dmatest_callback(void *arg)
{
	struct dmatest_done *done = arg;
	struct dmatest_thread *thread =
		container_of(done, struct dmatest_thread, test_done);
	if (!thread->done) {
		done->done = true;
		wake_up_all(done->wait);
	} else {
		/*
		 * If thread->done, it means that this callback occurred
		 * after the parent thread has cleaned up. This can
		 * happen in the case that driver doesn't implement
		 * the terminate_all() functionality and a dma operation
		 * did not occur within the timeout period
		 */
		WARN(1, "dmatest: Kernel memory may be corrupted!!\n");
	}
}

static unsigned int min_odd(unsigned int x, unsigned int y)
{
	unsigned int val = min(x, y);

	return val % 2 ? val : val - 1;
}

static void result(const char *err, unsigned int n, unsigned int src_off,
		   unsigned int dst_off, unsigned int len, unsigned long data)
{
	pr_info("%s: result #%u: '%s' with src_off=0x%x dst_off=0x%x len=0x%x (%lu)\n",
		current->comm, n, err, src_off, dst_off, len, data);
}

static void dbg_result(const char *err, unsigned int n, unsigned int src_off,
		       unsigned int dst_off, unsigned int len,
		       unsigned long data)
{
	pr_debug("%s: result #%u: '%s' with src_off=0x%x dst_off=0x%x len=0x%x (%lu)\n",
		 current->comm, n, err, src_off, dst_off, len, data);
}

#define verbose_result(err, n, src_off, dst_off, len, data) ({	\
	if (verbose)						\
		result(err, n, src_off, dst_off, len, data);	\
	else							\
		dbg_result(err, n, src_off, dst_off, len, data);\
})

static unsigned long long dmatest_persec(s64 runtime, unsigned int val)
{
	unsigned long long per_sec = 1000000;

	if (runtime <= 0)
		return 0;

	/* drop precision until runtime is 32-bits */
	while (runtime > UINT_MAX) {
		runtime >>= 1;
		per_sec <<= 1;
	}

	per_sec *= val;
	do_div(per_sec, runtime);
	return per_sec;
}

static unsigned long long dmatest_KBs(s64 runtime, unsigned long long len)
{
	return dmatest_persec(runtime, len >> 10);
}

static int dmatest_init_sizes(struct dmatest_thread *t)
{
	struct dma_device *dev = t->chan->device;
	struct dmatest_params *p = &t->info->params;
	struct dmatest_data *src = &t->src;
	struct dmatest_data *dst = &t->dst;

	switch (t->type) {
	case DMA_MEMCPY:
		p->align = dev->copy_align;
		src->cnt = dst->cnt = 1;
		return 0;
	case DMA_XOR:
		/* force odd to ensure dst = src */
		src->cnt = min_odd(p->xor_sources | 1, dev->max_xor);
		dst->cnt = 1;
		p->align = dev->xor_align;
		return 0;
	case DMA_PQ:
		/* force odd to ensure dst = src */
		src->cnt = min_odd(p->pq_sources | 1, dma_maxpq(dev, 0));
		dst->cnt = 2;
		p->align = dev->pq_align;
		return 0;
	case DMA_MEMSET:
		p->align = dev->fill_align;
		src->cnt = dst->cnt = 1;
		return 0;
	default:
		return -EINVAL;
	}
}

static void __dmatest_free_test_data(struct dmatest_data *d, unsigned int cnt)
{
	unsigned int i;

	for (i = 0; i < cnt; i++)
		kfree(d->raw[i]);

	kfree(d->aligned);
	kfree(d->raw);
	kfree(d->dma_addrs);
}

static void dmatest_free_test_data(struct dmatest_data *d)
{
	__dmatest_free_test_data(d, d->cnt);
}

static int dmatest_alloc_test_data(struct dmatest_data *d,
		unsigned int buf_size, u8 align)
{
	unsigned int i = 0;

	d->raw = kcalloc(d->cnt + 1, sizeof(u8 *), GFP_KERNEL);
	if (!d->raw)
		return -ENOMEM;

	d->aligned = kcalloc(d->cnt + 1, sizeof(u8 *), GFP_KERNEL);
	if (!d->aligned)
		goto err;

	d->dma_addrs = kcalloc(d->cnt + 1, sizeof(*d->dma_addrs),
			       GFP_KERNEL);
	if (!d->dma_addrs)
		goto err;

	for (i = 0; i < d->cnt; i++) {
		d->raw[i] = kmalloc(buf_size + align, GFP_KERNEL);
		if (!d->raw[i])
			goto err;

		/* align to alignment restriction */
		if (align)
			d->aligned[i] = PTR_ALIGN(d->raw[i], align);
		else
			d->aligned[i] = d->raw[i];
	}

	return 0;
err:
	__dmatest_free_test_data(d, i);
	return -ENOMEM;
}

static int dmatest_map_data(struct device *dev, struct dmatest_data *d,
		unsigned int buf_size,
		unsigned int len)
{
	struct dmaengine_unmap_data *um;
	unsigned int i;
	int ret;
	u8 *cnt;

	um = dmaengine_get_unmap_data(dev, d->cnt, GFP_KERNEL);
	if (!um)
		return -ENOMEM;

	switch (d->dir) {
	case DMA_TO_DEVICE:
		cnt = &um->to_cnt;
		break;
	case DMA_BIDIRECTIONAL:
		cnt = &um->bidi_cnt;
		break;
	case DMA_FROM_DEVICE:
		cnt = &um->from_cnt;
		break;
	default:
		return -EINVAL;
	}

	um->len = buf_size;
	for (i = 0; i < d->cnt; i++) {
		void *buf = d->aligned[i];
		struct page *pg = virt_to_page(buf);
		unsigned long pg_off = offset_in_page(buf);

		um->addr[i] = dma_map_page(dev, pg, pg_off,
					     um->len, d->dir);
		ret = dma_mapping_error(dev, um->addr[i]);
		if (ret) {
			dmaengine_unmap_put(um);
			return ret;
		}
		d->dma_addrs[i] = um->addr[i] + d->off;
		(*cnt)++;
	}
	d->um = um;
	d->dev = dev;

	return 0;
}

static void dmatest_unmap_data(struct dmatest_data *s,
		struct dmatest_data *d)
{
	if (s->um)
		dmaengine_unmap_put(s->um);
	if (d->um)
		dmaengine_unmap_put(d->um);

}

static int dmatest_prep_tx_desc(struct dmatest_thread *t,
		unsigned int len,
		u8 *pq_coefs)
{
	struct dma_chan *chan = t->chan;
	struct dma_device *dev = chan->device;
	struct dmatest_xfer *x = &t->xfer;
	struct dmatest_data *src = &t->src;
	struct dmatest_data *dst = &t->dst;
	enum dma_ctrl_flags flags;
	dma_addr_t *srcs = src->dma_addrs;
	dma_addr_t *dsts = dst->dma_addrs;

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	switch (t->type) {
	case DMA_MEMCPY:
		x->tx = dev->device_prep_dma_memcpy(chan,
						    dsts[0],
						    srcs[0], len, flags);
		break;
	case DMA_XOR:
		x->tx = dev->device_prep_dma_xor(chan,
						 dsts[0],
						 srcs, src->cnt,
						 len, flags);
		break;
	case DMA_PQ:
		x->tx = dev->device_prep_dma_pq(chan, dsts, srcs,
						src->cnt, pq_coefs,
						len, flags);
		break;
	case DMA_MEMSET:
		x->tx = dev->device_prep_dma_memset(chan,
						  dsts[0],
						  *(src->aligned[0]),
						  len, flags);
		break;
	default:
		return -EINVAL;
	}
	if (!x->tx)
		return -1;
	x->chan = chan;

	return 0;
}

static int dmatest_run_transaction(struct dmatest_xfer *x,
		struct dmatest_done *done)
{
	dma_cookie_t cookie;

	x->tx->callback = dmatest_callback;
	x->tx->callback_param = done;
	cookie = x->tx->tx_submit(x->tx);

	if (dma_submit_error(cookie))
		return -1;
	x->cookie = cookie;

	dma_async_issue_pending(x->chan);
	return 0;
}

/*
 * This function repeatedly tests DMA transfers of various lengths and
 * offsets for a given operation type until it is told to exit by
 * kthread_stop(). There may be multiple threads running this function
 * in parallel for a single channel, and there may be multiple channels
 * being tested in parallel.
 *
 * Before each test, the source and destination buffer is initialized
 * with a known pattern. This pattern is different depending on
 * whether it's in an area which is supposed to be copied or
 * overwritten, and different in the source and destination buffers.
 * So if the DMA engine doesn't copy exactly what we tell it to copy,
 * we'll notice.
 */
static int dmatest_func(void *data)
{
	struct dmatest_thread	*thread = data;
	struct dmatest_done	*done = &thread->test_done;
	struct dmatest_info	*info;
	struct dmatest_params	*params;
	struct dma_chan		*chan;
	struct dma_device	*dev;
	unsigned int		error_count;
	unsigned int		failed_tests = 0;
	unsigned int		total_tests = 0;
	enum dma_status		status;
	u8			*pq_coefs = NULL;
	int			ret;
	unsigned int 		buf_size;
	struct dmatest_data	*src;
	struct dmatest_data	*dst;
	int			i;
	ktime_t			ktime, start, diff;
	ktime_t			filltime = 0;
	ktime_t			comparetime = 0;
	s64			runtime = 0;
	unsigned long long	total_len = 0;
	u8			align;
	bool			is_memset;

	set_freezable();

	ret = -ENOMEM;

	smp_rmb();

	chan = thread->chan;
	if (dmatest_init_sizes(thread) < 0)
		goto err_thread_type;

	info = thread->info;
	params = &info->params;
	dev = chan->device;
	src = &thread->src;
	dst = &thread->dst;
	is_memset = (thread->type == DMA_MEMSET);
	align = params->align;

	if (thread->type == DMA_PQ) {
		pq_coefs = kmalloc(params->pq_sources + 1, GFP_KERNEL);
		if (!pq_coefs)
			goto err_thread_type;

		for (i = 0; i < src->cnt; i++)
			pq_coefs[i] = 1;
	}

	/* Check if buffer count fits into map count variable (u8) */
	if ((src->cnt + dst->cnt) >= 255) {
		pr_err("too many buffers (%d of 255 supported)\n",
		       src->cnt + dst->cnt);
		goto err_thread_type;
	}

	buf_size = params->buf_size;
	if (1 << align > buf_size) {
		pr_err("%u-byte buffer too small for %d-byte alignment\n",
		       buf_size, 1 << align);
		goto err_thread_type;
	}

	if (dmatest_alloc_test_data(src, buf_size, align) < 0)
		goto err_src;

	if (dmatest_alloc_test_data(dst, buf_size, align) < 0)
		goto err_dst;

	set_user_nice(current, 10);

	ktime = ktime_get();
	while (!kthread_should_stop()
	       && !(params->iterations && total_tests >= params->iterations)) {
		unsigned int len;

		total_tests++;

		if (params->norandom)
			len = buf_size;
		else
			len = dmatest_random() % buf_size + 1;

		len = (len >> align) << align;
		if (!len)
			len = 1 << align;

		total_len += len;

		if (params->norandom) {
			src->off = 0;
			dst->off = 0;
		} else {
			src->off = dmatest_random() % (buf_size - len + 1);
			dst->off = dmatest_random() % (buf_size - len + 1);

			src->off = (src->off >> align) << align;
			dst->off = (dst->off >> align) << align;
		}

		if (!params->noverify) {
			start = ktime_get();
			dmatest_init_srcs(src->aligned, src->off, len,
					  buf_size, is_memset);
			dmatest_init_dsts(dst->aligned, dst->off, len,
					  buf_size, is_memset);

			diff = ktime_sub(ktime_get(), start);
			filltime = ktime_add(filltime, diff);
		}
		src->dir = DMA_TO_DEVICE;
		/* map with DMA_BIDIRECTIONAL to force writeback/invalidate */
		dst->dir = DMA_BIDIRECTIONAL;

		ret = dmatest_map_data(dev->dev, src, buf_size, len);
		if (ret < 0) {
			result("src mapping error", total_tests,
			       src->off, dst->off, len, ret);
			failed_tests++;
			continue;
		}

		ret = dmatest_map_data(dev->dev, dst, buf_size, len);
		if (ret < 0) {
			result("dst mapping error", total_tests,
			       src->off, dst->off, len, ret);
			failed_tests++;
			continue;
		}

		ret = dmatest_prep_tx_desc(thread, len, pq_coefs);
		if (ret < 0) {
			dmatest_unmap_data(src, dst);
			result("prep error", total_tests, src->off,
			       dst->off, len, ret);
			msleep(100);
			failed_tests++;
			continue;
		}

		done->done = false;
		ret = dmatest_run_transaction(&thread->xfer, done);
		if (ret < 0) {
			dmatest_unmap_data(src, dst);
			result("submit error", total_tests, src->off,
			       dst->off, len, ret);
			msleep(100);
			failed_tests++;
			continue;
		}

		wait_event_freezable_timeout(thread->done_wait, done->done,
					     msecs_to_jiffies(params->timeout));

		status = dma_async_is_tx_complete(chan, thread->xfer.cookie,
						  NULL, NULL);

		dmatest_unmap_data(src, dst);

		if (!done->done) {
			result("test timed out", total_tests, src->off, dst->off,
			       len, 0);
			failed_tests++;
			continue;
		} else if (status != DMA_COMPLETE) {
			result(status == DMA_ERROR ?
			       "completion error status" :
			       "completion busy status", total_tests, src->off,
			       dst->off, len, ret);
			failed_tests++;
			continue;
		}

		if (params->noverify) {
			verbose_result("test passed", total_tests, src->off,
				       dst->off, len, 0);
			continue;
		}

		start = ktime_get();
		pr_debug("%s: verifying source buffer...\n", current->comm);
		error_count = dmatest_verify(src->aligned, 0, src->off,
				0, PATTERN_SRC, true, is_memset);
		error_count += dmatest_verify(src->aligned, src->off,
				src->off + len, src->off,
				PATTERN_SRC | PATTERN_COPY, true, is_memset);
		error_count += dmatest_verify(src->aligned, src->off + len,
				buf_size, src->off + len,
				PATTERN_SRC, true, is_memset);

		pr_debug("%s: verifying dest buffer...\n", current->comm);
		error_count += dmatest_verify(dst->aligned, 0, dst->off,
				0, PATTERN_DST, false, is_memset);

		error_count += dmatest_verify(dst->aligned, dst->off,
				dst->off + len, src->off,
				PATTERN_SRC | PATTERN_COPY, false, is_memset);

		error_count += dmatest_verify(dst->aligned, dst->off + len,
				buf_size, dst->off + len,
				PATTERN_DST, false, is_memset);

		diff = ktime_sub(ktime_get(), start);
		comparetime = ktime_add(comparetime, diff);

		if (error_count) {
			result("data error", total_tests, src->off, dst->off,
			       len, error_count);
			failed_tests++;
		} else {
			verbose_result("test passed", total_tests, src->off,
				       dst->off, len, 0);
		}
	}
	ktime = ktime_sub(ktime_get(), ktime);
	ktime = ktime_sub(ktime, comparetime);
	ktime = ktime_sub(ktime, filltime);
	runtime = ktime_to_us(ktime);

	ret = 0;
	dmatest_free_test_data(dst);
err_dst:
	dmatest_free_test_data(src);
err_src:
	kfree(pq_coefs);
err_thread_type:
	pr_info("%s: summary %u tests, %u failures %llu iops %llu KB/s (%d)\n",
		current->comm, total_tests, failed_tests,
		dmatest_persec(runtime, total_tests),
		dmatest_KBs(runtime, total_len), ret);

	/* terminate all transfers on specified channels */
	if (ret || failed_tests)
		dmaengine_terminate_sync(chan);

	thread->done = true;
	wake_up(&thread_wait);

	return ret;
}

static void dmatest_cleanup_channel(struct dmatest_chan *dtc)
{
	struct dmatest_thread	*thread;
	struct dmatest_thread	*_thread;
	int			ret;

	list_for_each_entry_safe(thread, _thread, &dtc->threads, node) {
		ret = kthread_stop(thread->task);
		pr_debug("thread %s exited with status %d\n",
			 thread->task->comm, ret);
		list_del(&thread->node);
		put_task_struct(thread->task);
		kfree(thread);
	}

	/* terminate all transfers on specified channels */
	dmaengine_terminate_sync(dtc->chan);

	kfree(dtc);
}

static int dmatest_add_threads(struct dmatest_info *info,
		struct dmatest_chan *dtc, enum dma_transaction_type type)
{
	struct dmatest_params *params = &info->params;
	struct dmatest_thread *thread;
	struct dma_chan *chan = dtc->chan;
	const char *op = transaction_type_names[type];
	unsigned int i;

	for (i = 0; i < params->threads_per_chan; i++) {
		thread = kzalloc(sizeof(struct dmatest_thread), GFP_KERNEL);
		if (!thread) {
			pr_warn("No memory for %s-%s%u\n",
				dma_chan_name(chan), op, i);
			break;
		}
		thread->info = info;
		thread->chan = dtc->chan;
		thread->type = type;
		thread->test_done.wait = &thread->done_wait;
		init_waitqueue_head(&thread->done_wait);
		smp_wmb();
		thread->task = kthread_create(dmatest_func, thread, "%s-%s%u",
				dma_chan_name(chan), op, i);
		if (IS_ERR(thread->task)) {
			pr_warn("Failed to create thread %s-%s%u\n",
				dma_chan_name(chan), op, i);
			kfree(thread);
			break;
		}

		/* srcbuf and dstbuf are allocated by the thread itself */
		get_task_struct(thread->task);
		list_add_tail(&thread->node, &dtc->threads);
		wake_up_process(thread->task);
	}

	return i;
}

static int dmatest_add_channel(struct dmatest_info *info,
		struct dma_chan *chan,
		enum dma_transaction_type type)
{
	struct dma_device	*dma_dev = chan->device;
	struct dmatest_chan	*dtc;
	unsigned int		thread_count = 0;
	int cnt;

	/* Test operation is undefined in this test (yet) */
        if (!transaction_type_names[type])
		return 0;

	if (!dma_has_cap(type, dma_dev->cap_mask))
		return 0;

	dtc = kmalloc(sizeof(struct dmatest_chan), GFP_KERNEL);
	if (!dtc) {
		pr_warn("No memory for %s\n", dma_chan_name(chan));
		return -ENOMEM;
	}

	dtc->chan = chan;
	INIT_LIST_HEAD(&dtc->threads);

	cnt = dmatest_add_threads(info, dtc, type);
	thread_count += cnt > 0 ? cnt : 0;

	pr_info("Started %u threads using %s\n",
		thread_count, dma_chan_name(chan));

	list_add_tail(&dtc->node, &info->channels);
	info->nr_channels++;

	return 0;
}

static bool filter(struct dma_chan *chan, void *param)
{
	struct dmatest_params *params = param;

	if (!dmatest_match_channel(params, chan) ||
	    !dmatest_match_device(params, chan->device))
		return false;
	else
		return true;
}

static void request_channels(struct dmatest_info *info,
			     enum dma_transaction_type type)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(type, mask);
	for (;;) {
		struct dmatest_params *params = &info->params;
		struct dma_chan *chan;

		chan = dma_request_channel(mask, filter, params);
		if (chan) {
			if (dmatest_add_channel(info, chan, type)) {
				dma_release_channel(chan);
				break; /* add_channel failed, punt */
			}
		} else
			break; /* no more channels available */
		if (params->max_channels &&
		    info->nr_channels >= params->max_channels)
			break; /* we have all we need */
	}
}

static void run_threaded_test(struct dmatest_info *info)
{
	struct dmatest_params *params = &info->params;
	unsigned int i;

	/* Copy test parameters */
	params->buf_size = test_buf_size;
	strlcpy(params->channel, strim(test_channel), sizeof(params->channel));
	strlcpy(params->device, strim(test_device), sizeof(params->device));
	params->threads_per_chan = threads_per_chan;
	params->max_channels = max_channels;
	params->iterations = iterations;
	params->xor_sources = xor_sources;
	params->pq_sources = pq_sources;
	params->timeout = timeout;
	params->noverify = noverify;
	params->norandom = norandom;

	for (i = 0; i < DMA_TX_TYPE_END; i++) {
		if (!transaction_type_names[i])
			continue;
		/* skip if this test hasn't been selected */
		if (!(BIT(i) & dmatest))
			continue;
		request_channels(info, i);
	}
}

static void stop_threaded_test(struct dmatest_info *info)
{
	struct dmatest_chan *dtc, *_dtc;
	struct dma_chan *chan;

	list_for_each_entry_safe(dtc, _dtc, &info->channels, node) {
		list_del(&dtc->node);
		chan = dtc->chan;
		dmatest_cleanup_channel(dtc);
		pr_debug("dropped channel %s\n", dma_chan_name(chan));
		dma_release_channel(chan);
	}

	info->nr_channels = 0;
}

static void restart_threaded_test(struct dmatest_info *info, bool run)
{
	/* we might be called early to set run=, defer running until all
	 * parameters have been evaluated
	 */
	if (!info->did_init)
		return;

	/* Stop any running test first */
	stop_threaded_test(info);

	/* Run test with new parameters */
	run_threaded_test(info);
}

static int dmatest_run_get(char *val, const struct kernel_param *kp)
{
	struct dmatest_info *info = &test_info;

	mutex_lock(&info->lock);
	if (is_threaded_test_run(info)) {
		dmatest_run = true;
	} else {
		stop_threaded_test(info);
		dmatest_run = false;
	}
	mutex_unlock(&info->lock);

	return param_get_bool(val, kp);
}

static int dmatest_run_set(const char *val, const struct kernel_param *kp)
{
	struct dmatest_info *info = &test_info;
	int ret;

	mutex_lock(&info->lock);
	ret = param_set_bool(val, kp);
	if (ret) {
		mutex_unlock(&info->lock);
		return ret;
	}

	if (is_threaded_test_run(info))
		ret = -EBUSY;
	else if (dmatest_run)
		restart_threaded_test(info, dmatest_run);

	mutex_unlock(&info->lock);

	return ret;
}

static int param_set_dmatest_type(const char *val,
				  const struct kernel_param *kp)
{
	unsigned int mask;
	char val_buf[32], *p;
	int ret, en = 1;

	strlcpy(val_buf, val, sizeof(val_buf));

	if ((p = strchr(val_buf, '='))) {
		*p = 0;
		p++;
		kstrtoint(p, 10, &en);
	}

	ret = find_string(strim(val_buf), transaction_type_names,
			  ARRAY_SIZE(transaction_type_names));
	if (ret < 0)
		return ret;
	mask = BIT(ret);
	if (en)
		dmatest |= mask;
	else
		dmatest &= ~mask;
	return 0;
}

static int param_get_dmatest_type(char *buffer,
				  const struct kernel_param *kp)
{
	int i, ret, off = 0;
	for (i = 0; i < ARRAY_SIZE(transaction_type_names); i++) {
		const char *n = transaction_type_names[i];
		if (!n)
			continue;
		ret = scnprintf(&buffer[off], PAGE_SIZE - off, "%16s - %s\n",
			n, (dmatest & BIT(i)) ? "on" : "off");
		if (ret < 0)
			return ret;
		off += ret;
	}
	return off;
}

static int __init dmatest_init(void)
{
	struct dmatest_info *info = &test_info;
	struct dmatest_params *params = &info->params;

	if (dmatest_run) {
		mutex_lock(&info->lock);
		run_threaded_test(info);
		mutex_unlock(&info->lock);
	}

	if (params->iterations && wait)
		wait_event(thread_wait, !is_threaded_test_run(info));

	/* module parameters are stable, inittime tests are started,
	 * let userspace take over 'run' control
	 */
	info->did_init = true;

	return 0;
}
/* when compiled-in wait for drivers to load first */
late_initcall(dmatest_init);

static void __exit dmatest_exit(void)
{
	struct dmatest_info *info = &test_info;

	mutex_lock(&info->lock);
	stop_threaded_test(info);
	mutex_unlock(&info->lock);
}
module_exit(dmatest_exit);

MODULE_AUTHOR("Haavard Skinnemoen (Atmel)");
MODULE_LICENSE("GPL v2");
