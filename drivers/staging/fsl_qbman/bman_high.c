/* Copyright 2008-2012 Freescale Semiconductor, Inc.
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

#include "bman_low.h"

/* Compilation constants */
#define RCR_THRESH	2	/* reread h/w CI when running out of space */
#define IRQNAME		"BMan portal %d"
#define MAX_IRQNAME	16	/* big enough for "BMan portal %d" */

struct bman_portal {
	struct bm_portal p;
	/* 2-element array. pools[0] is mask, pools[1] is snapshot. */
	struct bman_depletion *pools;
	int thresh_set;
	unsigned long irq_sources;
	u32 slowpoll;	/* only used when interrupts are off */
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	struct bman_pool *rcri_owned; /* only 1 release WAIT_SYNC at a time */
#endif
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	raw_spinlock_t sharing_lock; /* only used if is_shared */
	int is_shared;
	struct bman_portal *sharing_redirect;
#endif
	/* When the cpu-affine portal is activated, this is non-NULL */
	const struct bm_portal_config *config;
	/* This is needed for power management */
	struct platform_device *pdev;
	/* 64-entry hash-table of pool objects that are tracking depletion
	 * entry/exit (ie. BMAN_POOL_FLAG_DEPLETION). This isn't fast-path, so
	 * we're not fussy about cache-misses and so forth - whereas the above
	 * members should all fit in one cacheline.
	 * BTW, with 64 entries in the hash table and 64 buffer pools to track,
	 * you'll never guess the hash-function ... */
	struct bman_pool *cb[64];
	char irqname[MAX_IRQNAME];
	/* Track if the portal was alloced by the driver */
	u8 alloced;
	/* power management data */
	u32 save_isdr;
};

/* For an explanation of the locking, redirection, or affine-portal logic,
 * please consult the Qman driver for details. This is the same, only simpler
 * (no fiddly Qman-specific bits.) */
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
#define PORTAL_IRQ_LOCK(p, irqflags) \
	do { \
		if ((p)->is_shared) \
			raw_spin_lock_irqsave(&(p)->sharing_lock, irqflags); \
		else \
			local_irq_save(irqflags); \
	} while (0)
#define PORTAL_IRQ_UNLOCK(p, irqflags) \
	do { \
		if ((p)->is_shared) \
			raw_spin_unlock_irqrestore(&(p)->sharing_lock, \
						   irqflags); \
		else \
			local_irq_restore(irqflags); \
	} while (0)
#else
#define PORTAL_IRQ_LOCK(p, irqflags) local_irq_save(irqflags)
#define PORTAL_IRQ_UNLOCK(p, irqflags) local_irq_restore(irqflags)
#endif

static cpumask_t affine_mask;
static DEFINE_SPINLOCK(affine_mask_lock);
static DEFINE_PER_CPU(struct bman_portal, bman_affine_portal);
static inline struct bman_portal *get_raw_affine_portal(void)
{
	return &get_cpu_var(bman_affine_portal);
}
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
static inline struct bman_portal *get_affine_portal(void)
{
	struct bman_portal *p = get_raw_affine_portal();
	if (p->sharing_redirect)
		return p->sharing_redirect;
	return p;
}
#else
#define get_affine_portal() get_raw_affine_portal()
#endif
static inline void put_affine_portal(void)
{
	put_cpu_var(bman_affine_portal);
}
static inline struct bman_portal *get_poll_portal(void)
{
	return &get_cpu_var(bman_affine_portal);
}
#define put_poll_portal()

/* GOTCHA: this object type refers to a pool, it isn't *the* pool. There may be
 * more than one such object per Bman buffer pool, eg. if different users of the
 * pool are operating via different portals. */
struct bman_pool {
	struct bman_pool_params params;
	/* Used for hash-table admin when using depletion notifications. */
	struct bman_portal *portal;
	struct bman_pool *next;
	/* stockpile state - NULL unless BMAN_POOL_FLAG_STOCKPILE is set */
	struct bm_buffer *sp;
	unsigned int sp_fill;
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_t in_use;
#endif
};

/* (De)Registration of depletion notification callbacks */
static void depletion_link(struct bman_portal *portal, struct bman_pool *pool)
{
	__maybe_unused unsigned long irqflags;
	pool->portal = portal;
	PORTAL_IRQ_LOCK(portal, irqflags);
	pool->next = portal->cb[pool->params.bpid];
	portal->cb[pool->params.bpid] = pool;
	if (!pool->next)
		/* First object for that bpid on this portal, enable the BSCN
		 * mask bit. */
		bm_isr_bscn_mask(&portal->p, pool->params.bpid, 1);
	PORTAL_IRQ_UNLOCK(portal, irqflags);
}
static void depletion_unlink(struct bman_pool *pool)
{
	struct bman_pool *it, *last = NULL;
	struct bman_pool **base = &pool->portal->cb[pool->params.bpid];
	__maybe_unused unsigned long irqflags;
	PORTAL_IRQ_LOCK(pool->portal, irqflags);
	it = *base;	/* <-- gotcha, don't do this prior to the irq_save */
	while (it != pool) {
		last = it;
		it = it->next;
	}
	if (!last)
		*base = pool->next;
	else
		last->next = pool->next;
	if (!last && !pool->next) {
		/* Last object for that bpid on this portal, disable the BSCN
		 * mask bit. */
		bm_isr_bscn_mask(&pool->portal->p, pool->params.bpid, 0);
		/* And "forget" that we last saw this pool as depleted */
		bman_depletion_unset(&pool->portal->pools[1],
					pool->params.bpid);
	}
	PORTAL_IRQ_UNLOCK(pool->portal, irqflags);
}

/* In the case that the application's core loop calls qman_poll() and
 * bman_poll(), we ought to balance how often we incur the overheads of the
 * slow-path poll. We'll use two decrementer sources. The idle decrementer
 * constant is used when the last slow-poll detected no work to do, and the busy
 * decrementer constant when the last slow-poll had work to do. */
#define SLOW_POLL_IDLE 1000
#define SLOW_POLL_BUSY 10
static u32 __poll_portal_slow(struct bman_portal *p, u32 is);

/* Portal interrupt handler */
static irqreturn_t portal_isr(__always_unused int irq, void *ptr)
{
	struct bman_portal *p = ptr;
	u32 clear = p->irq_sources;
	u32 is = bm_isr_status_read(&p->p) & p->irq_sources;
	clear |= __poll_portal_slow(p, is);
	bm_isr_status_clear(&p->p, clear);
	return IRQ_HANDLED;
}

#ifdef CONFIG_SUSPEND
static int _bman_portal_suspend_noirq(struct device *dev)
{
	struct bman_portal *p = (struct bman_portal *)dev->platform_data;
#ifdef CONFIG_PM_DEBUG
	struct platform_device *pdev = to_platform_device(dev);
#endif
	p->save_isdr = bm_isr_disable_read(&p->p);
	bm_isr_disable_write(&p->p, 0xffffffff);
	bm_isr_status_clear(&p->p, 0xffffffff);
#ifdef CONFIG_PM_DEBUG
	pr_info("Suspend for %s\n", pdev->name);
#endif
	return 0;
}

static int _bman_portal_resume_noirq(struct device *dev)
{
	struct bman_portal *p = (struct bman_portal *)dev->platform_data;

	/* restore isdr */
	bm_isr_disable_write(&p->p, p->save_isdr);
	return 0;
}
#else
#define _bman_portal_suspend_noirq NULL
#define _bman_portal_resume_noirq NULL
#endif

struct dev_pm_domain bman_portal_device_pm_domain = {
	.ops = {
		USE_PLATFORM_PM_SLEEP_OPS
		.suspend_noirq = _bman_portal_suspend_noirq,
		.resume_noirq = _bman_portal_resume_noirq,
	}
};

struct bman_portal *bman_create_portal(
				       struct bman_portal *portal,
				       const struct bm_portal_config *config)
{
	struct bm_portal *__p;
	const struct bman_depletion *pools = &config->public_cfg.mask;
	int ret;
	u8 bpid = 0;
	char buf[16];

	if (!portal) {
		portal = kmalloc(sizeof(*portal), GFP_KERNEL);
		if (!portal)
			return portal;
		portal->alloced = 1;
	} else
		portal->alloced = 0;

	__p = &portal->p;

	/* prep the low-level portal struct with the mapped addresses from the
	 * config, everything that follows depends on it and "config" is more
	 * for (de)reference... */
	__p->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	__p->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];
	if (bm_rcr_init(__p, bm_rcr_pvb, bm_rcr_cce)) {
		pr_err("Bman RCR initialisation failed\n");
		goto fail_rcr;
	}
	if (bm_mc_init(__p)) {
		pr_err("Bman MC initialisation failed\n");
		goto fail_mc;
	}
	if (bm_isr_init(__p)) {
		pr_err("Bman ISR initialisation failed\n");
		goto fail_isr;
	}
	portal->pools = kmalloc(2 * sizeof(*pools), GFP_KERNEL);
	if (!portal->pools)
		goto fail_pools;
	portal->pools[0] = *pools;
	bman_depletion_init(portal->pools + 1);
	while (bpid < bman_pool_max) {
		/* Default to all BPIDs disabled, we enable as required at
		 * run-time. */
		bm_isr_bscn_mask(__p, bpid, 0);
		bpid++;
	}
	portal->slowpoll = 0;
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	portal->rcri_owned = NULL;
#endif
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	raw_spin_lock_init(&portal->sharing_lock);
	portal->is_shared = config->public_cfg.is_shared;
	portal->sharing_redirect = NULL;
#endif
	sprintf(buf, "bportal-%u", config->public_cfg.index);
	portal->pdev = platform_device_alloc(buf, -1);
	if (!portal->pdev)
		goto fail_devalloc;
	portal->pdev->dev.pm_domain = &bman_portal_device_pm_domain;
	portal->pdev->dev.platform_data = portal;
	ret = platform_device_add(portal->pdev);
	if (ret)
		goto fail_devadd;
	memset(&portal->cb, 0, sizeof(portal->cb));
	/* Write-to-clear any stale interrupt status bits */
	bm_isr_disable_write(__p, 0xffffffff);
	portal->irq_sources = 0;
	bm_isr_enable_write(__p, portal->irq_sources);
	bm_isr_status_clear(__p, 0xffffffff);
	snprintf(portal->irqname, MAX_IRQNAME, IRQNAME, config->public_cfg.cpu);
	if (request_irq(config->public_cfg.irq, portal_isr, 0, portal->irqname,
			portal)) {
		pr_err("request_irq() failed\n");
		goto fail_irq;
	}
	if ((config->public_cfg.cpu != -1) &&
			irq_can_set_affinity(config->public_cfg.irq) &&
			irq_set_affinity(config->public_cfg.irq,
				cpumask_of(config->public_cfg.cpu))) {
		pr_err("irq_set_affinity() failed %s\n", portal->irqname);
		goto fail_affinity;
	}

	/* Need RCR to be empty before continuing */
	ret = bm_rcr_get_fill(__p);
	if (ret) {
		pr_err("Bman RCR unclean\n");
		goto fail_rcr_empty;
	}
	/* Success */
	portal->config = config;

	bm_isr_disable_write(__p, 0);
	bm_isr_uninhibit(__p);
	return portal;
fail_rcr_empty:
fail_affinity:
	free_irq(config->public_cfg.irq, portal);
fail_irq:
	platform_device_del(portal->pdev);
fail_devadd:
	platform_device_put(portal->pdev);
fail_devalloc:
	kfree(portal->pools);
fail_pools:
	bm_isr_finish(__p);
fail_isr:
	bm_mc_finish(__p);
fail_mc:
	bm_rcr_finish(__p);
fail_rcr:
	if (portal->alloced)
		kfree(portal);
	return NULL;
}

struct bman_portal *bman_create_affine_portal(
			const struct bm_portal_config *config)
{
	struct bman_portal *portal;

	portal = &per_cpu(bman_affine_portal, config->public_cfg.cpu);
	portal = bman_create_portal(portal, config);
	if (portal) {
		spin_lock(&affine_mask_lock);
		cpumask_set_cpu(config->public_cfg.cpu, &affine_mask);
		spin_unlock(&affine_mask_lock);
	}
	return portal;
}


struct bman_portal *bman_create_affine_slave(struct bman_portal *redirect,
								int cpu)
{
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	struct bman_portal *p;
	p = &per_cpu(bman_affine_portal, cpu);
	BUG_ON(p->config);
	BUG_ON(p->is_shared);
	BUG_ON(!redirect->config->public_cfg.is_shared);
	p->irq_sources = 0;
	p->sharing_redirect = redirect;
	return p;
#else
	BUG();
	return NULL;
#endif
}

void bman_destroy_portal(struct bman_portal *bm)
{
	const struct bm_portal_config *pcfg;
	pcfg = bm->config;
	bm_rcr_cce_update(&bm->p);
	bm_rcr_cce_update(&bm->p);

	free_irq(pcfg->public_cfg.irq, bm);

	kfree(bm->pools);
	bm_isr_finish(&bm->p);
	bm_mc_finish(&bm->p);
	bm_rcr_finish(&bm->p);
	bm->config = NULL;
	if (bm->alloced)
		kfree(bm);
}

const struct bm_portal_config *bman_destroy_affine_portal(void)
{
	struct bman_portal *bm = get_raw_affine_portal();
	const struct bm_portal_config *pcfg;
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (bm->sharing_redirect) {
		bm->sharing_redirect = NULL;
		put_affine_portal();
		return NULL;
	}
	bm->is_shared = 0;
#endif
	pcfg = bm->config;
	bman_destroy_portal(bm);
	spin_lock(&affine_mask_lock);
	cpumask_clear_cpu(pcfg->public_cfg.cpu, &affine_mask);
	spin_unlock(&affine_mask_lock);
	put_affine_portal();
	return pcfg;
}

/* When release logic waits on available RCR space, we need a global waitqueue
 * in the case of "affine" use (as the waits wake on different cpus which means
 * different portals - so we can't wait on any per-portal waitqueue). */
static DECLARE_WAIT_QUEUE_HEAD(affine_queue);

static u32 __poll_portal_slow(struct bman_portal *p, u32 is)
{
	struct bman_depletion tmp;
	u32 ret = is;

	/* There is a gotcha to be aware of. If we do the query before clearing
	 * the status register, we may miss state changes that occur between the
	 * two. If we write to clear the status register before the query, the
	 * cache-enabled query command may overtake the status register write
	 * unless we use a heavyweight sync (which we don't want). Instead, we
	 * write-to-clear the status register then *read it back* before doing
	 * the query, hence the odd while loop with the 'is' accumulation. */
	if (is & BM_PIRQ_BSCN) {
		struct bm_mc_result *mcr;
		__maybe_unused unsigned long irqflags;
		unsigned int i, j;
		u32 __is;
		bm_isr_status_clear(&p->p, BM_PIRQ_BSCN);
		while ((__is = bm_isr_status_read(&p->p)) & BM_PIRQ_BSCN) {
			is |= __is;
			bm_isr_status_clear(&p->p, BM_PIRQ_BSCN);
		}
		is &= ~BM_PIRQ_BSCN;
		PORTAL_IRQ_LOCK(p, irqflags);
		bm_mc_start(&p->p);
		bm_mc_commit(&p->p, BM_MCC_VERB_CMD_QUERY);
		while (!(mcr = bm_mc_result(&p->p)))
			cpu_relax();
		tmp = mcr->query.ds.state;
		tmp.__state[0] = be32_to_cpu(tmp.__state[0]);
		tmp.__state[1] = be32_to_cpu(tmp.__state[1]);
		PORTAL_IRQ_UNLOCK(p, irqflags);
		for (i = 0; i < 2; i++) {
			int idx = i * 32;
			/* tmp is a mask of currently-depleted pools.
			 * pools[0] is mask of those we care about.
			 * pools[1] is our previous view (we only want to
			 * be told about changes). */
			tmp.__state[i] &= p->pools[0].__state[i];
			if (tmp.__state[i] == p->pools[1].__state[i])
				/* fast-path, nothing to see, move along */
				continue;
			for (j = 0; j <= 31; j++, idx++) {
				struct bman_pool *pool = p->cb[idx];
				int b4 = bman_depletion_get(&p->pools[1], idx);
				int af = bman_depletion_get(&tmp, idx);
				if (b4 == af)
					continue;
				while (pool) {
					pool->params.cb(p, pool,
						pool->params.cb_ctx, af);
					pool = pool->next;
				}
			}
		}
		p->pools[1] = tmp;
	}

	if (is & BM_PIRQ_RCRI) {
		__maybe_unused unsigned long irqflags;
		PORTAL_IRQ_LOCK(p, irqflags);
		bm_rcr_cce_update(&p->p);
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
		/* If waiting for sync, we only cancel the interrupt threshold
		 * when the ring utilisation hits zero. */
		if (p->rcri_owned) {
			if (!bm_rcr_get_fill(&p->p)) {
				p->rcri_owned = NULL;
				bm_rcr_set_ithresh(&p->p, 0);
			}
		} else
#endif
		bm_rcr_set_ithresh(&p->p, 0);
		PORTAL_IRQ_UNLOCK(p, irqflags);
		wake_up(&affine_queue);
		bm_isr_status_clear(&p->p, BM_PIRQ_RCRI);
		is &= ~BM_PIRQ_RCRI;
	}

	/* There should be no status register bits left undefined */
	DPA_ASSERT(!is);
	return ret;
}

const struct bman_portal_config *bman_get_portal_config(void)
{
	struct bman_portal *p = get_affine_portal();
	const struct bman_portal_config *ret = &p->config->public_cfg;
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(bman_get_portal_config);

u32 bman_irqsource_get(void)
{
	struct bman_portal *p = get_raw_affine_portal();
	u32 ret = p->irq_sources & BM_PIRQ_VISIBLE;
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(bman_irqsource_get);

int bman_p_irqsource_add(struct bman_portal *p, __maybe_unused u32 bits)
{
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (p->sharing_redirect)
		return -EINVAL;
	else
#endif
	{
		__maybe_unused unsigned long irqflags;
		PORTAL_IRQ_LOCK(p, irqflags);
		set_bits(bits & BM_PIRQ_VISIBLE, &p->irq_sources);
		bm_isr_enable_write(&p->p, p->irq_sources);
		PORTAL_IRQ_UNLOCK(p, irqflags);
	}
	return 0;
}
EXPORT_SYMBOL(bman_p_irqsource_add);

int bman_irqsource_add(__maybe_unused u32 bits)
{
	struct bman_portal *p = get_raw_affine_portal();
	int ret = 0;
	ret = bman_p_irqsource_add(p, bits);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(bman_irqsource_add);

int bman_irqsource_remove(u32 bits)
{
	struct bman_portal *p = get_raw_affine_portal();
	__maybe_unused unsigned long irqflags;
	u32 ier;
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (p->sharing_redirect) {
		put_affine_portal();
		return -EINVAL;
	}
#endif
	/* Our interrupt handler only processes+clears status register bits that
	 * are in p->irq_sources. As we're trimming that mask, if one of them
	 * were to assert in the status register just before we remove it from
	 * the enable register, there would be an interrupt-storm when we
	 * release the IRQ lock. So we wait for the enable register update to
	 * take effect in h/w (by reading it back) and then clear all other bits
	 * in the status register. Ie. we clear them from ISR once it's certain
	 * IER won't allow them to reassert. */
	PORTAL_IRQ_LOCK(p, irqflags);
	bits &= BM_PIRQ_VISIBLE;
	clear_bits(bits, &p->irq_sources);
	bm_isr_enable_write(&p->p, p->irq_sources);
	ier = bm_isr_enable_read(&p->p);
	/* Using "~ier" (rather than "bits" or "~p->irq_sources") creates a
	 * data-dependency, ie. to protect against re-ordering. */
	bm_isr_status_clear(&p->p, ~ier);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return 0;
}
EXPORT_SYMBOL(bman_irqsource_remove);

const cpumask_t *bman_affine_cpus(void)
{
	return &affine_mask;
}
EXPORT_SYMBOL(bman_affine_cpus);

u32 bman_poll_slow(void)
{
	struct bman_portal *p = get_poll_portal();
	u32 ret;
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (unlikely(p->sharing_redirect))
		ret = (u32)-1;
	else
#endif
	{
		u32 is = bm_isr_status_read(&p->p) & ~p->irq_sources;
		ret = __poll_portal_slow(p, is);
		bm_isr_status_clear(&p->p, ret);
	}
	put_poll_portal();
	return ret;
}
EXPORT_SYMBOL(bman_poll_slow);

/* Legacy wrapper */
void bman_poll(void)
{
	struct bman_portal *p = get_poll_portal();
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (unlikely(p->sharing_redirect))
		goto done;
#endif
	if (!(p->slowpoll--)) {
		u32 is = bm_isr_status_read(&p->p) & ~p->irq_sources;
		u32 active = __poll_portal_slow(p, is);
		if (active)
			p->slowpoll = SLOW_POLL_BUSY;
		else
			p->slowpoll = SLOW_POLL_IDLE;
	}
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
done:
#endif
	put_poll_portal();
}
EXPORT_SYMBOL(bman_poll);

static const u32 zero_thresholds[4] = {0, 0, 0, 0};

struct bman_pool *bman_new_pool(const struct bman_pool_params *params)
{
	struct bman_pool *pool = NULL;
	u32 bpid;

	if (params->flags & BMAN_POOL_FLAG_DYNAMIC_BPID) {
		int ret = bman_alloc_bpid(&bpid);
		if (ret)
			return NULL;
	} else {
		if (params->bpid >= bman_pool_max)
			return NULL;
		bpid = params->bpid;
	}
#ifdef CONFIG_FSL_BMAN_CONFIG
	if (params->flags & BMAN_POOL_FLAG_THRESH) {
		int ret = bm_pool_set(bpid, params->thresholds);
		if (ret)
			goto err;
	}
#else
	if (params->flags & BMAN_POOL_FLAG_THRESH)
		goto err;
#endif
	pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		goto err;
	pool->sp = NULL;
	pool->sp_fill = 0;
	pool->params = *params;
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_set(&pool->in_use, 1);
#endif
	if (params->flags & BMAN_POOL_FLAG_DYNAMIC_BPID)
		pool->params.bpid = bpid;
	if (params->flags & BMAN_POOL_FLAG_STOCKPILE) {
		pool->sp = kmalloc(sizeof(struct bm_buffer) * BMAN_STOCKPILE_SZ,
					GFP_KERNEL);
		if (!pool->sp)
			goto err;
	}
	if (pool->params.flags & BMAN_POOL_FLAG_DEPLETION) {
		struct bman_portal *p = get_affine_portal();
		if (!p->pools || !bman_depletion_get(&p->pools[0], bpid)) {
			pr_err("Depletion events disabled for bpid %d\n", bpid);
			goto err;
		}
		depletion_link(p, pool);
		put_affine_portal();
	}
	return pool;
err:
#ifdef CONFIG_FSL_BMAN_CONFIG
	if (params->flags & BMAN_POOL_FLAG_THRESH)
		bm_pool_set(bpid, zero_thresholds);
#endif
	if (params->flags & BMAN_POOL_FLAG_DYNAMIC_BPID)
		bman_release_bpid(bpid);
	if (pool) {
		kfree(pool->sp);
		kfree(pool);
	}
	return NULL;
}
EXPORT_SYMBOL(bman_new_pool);

void bman_free_pool(struct bman_pool *pool)
{
#ifdef CONFIG_FSL_BMAN_CONFIG
	if (pool->params.flags & BMAN_POOL_FLAG_THRESH)
		bm_pool_set(pool->params.bpid, zero_thresholds);
#endif
	if (pool->params.flags & BMAN_POOL_FLAG_DEPLETION)
		depletion_unlink(pool);
	if (pool->params.flags & BMAN_POOL_FLAG_STOCKPILE) {
		if (pool->sp_fill)
			pr_err("Stockpile not flushed, has %u in bpid %u.\n",
				pool->sp_fill, pool->params.bpid);
		kfree(pool->sp);
		pool->sp = NULL;
		pool->params.flags ^= BMAN_POOL_FLAG_STOCKPILE;
	}
	if (pool->params.flags & BMAN_POOL_FLAG_DYNAMIC_BPID)
		bman_release_bpid(pool->params.bpid);
	kfree(pool);
}
EXPORT_SYMBOL(bman_free_pool);

const struct bman_pool_params *bman_get_params(const struct bman_pool *pool)
{
	return &pool->params;
}
EXPORT_SYMBOL(bman_get_params);

static noinline void update_rcr_ci(struct bman_portal *p, u8 avail)
{
	if (avail)
		bm_rcr_cce_prefetch(&p->p);
	else
		bm_rcr_cce_update(&p->p);
}

int bman_rcr_is_empty(void)
{
	__maybe_unused unsigned long irqflags;
	struct bman_portal *p = get_affine_portal();
	u8 avail;

	PORTAL_IRQ_LOCK(p, irqflags);
	update_rcr_ci(p, 0);
	avail = bm_rcr_get_fill(&p->p);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return avail == 0;
}
EXPORT_SYMBOL(bman_rcr_is_empty);

static inline struct bm_rcr_entry *try_rel_start(struct bman_portal **p,
#ifdef CONFIG_FSL_DPA_CAN_WAIT
					__maybe_unused struct bman_pool *pool,
#endif
					__maybe_unused unsigned long *irqflags,
					__maybe_unused u32 flags)
{
	struct bm_rcr_entry *r;
	u8 avail;

	*p = get_affine_portal();
	PORTAL_IRQ_LOCK(*p, (*irqflags));
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
			(flags & BMAN_RELEASE_FLAG_WAIT_SYNC))) {
		if ((*p)->rcri_owned) {
			PORTAL_IRQ_UNLOCK(*p, (*irqflags));
			put_affine_portal();
			return NULL;
		}
		(*p)->rcri_owned = pool;
	}
#endif
	avail = bm_rcr_get_avail(&(*p)->p);
	if (avail < 2)
		update_rcr_ci(*p, avail);
	r = bm_rcr_start(&(*p)->p);
	if (unlikely(!r)) {
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
		if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
				(flags & BMAN_RELEASE_FLAG_WAIT_SYNC)))
			(*p)->rcri_owned = NULL;
#endif
		PORTAL_IRQ_UNLOCK(*p, (*irqflags));
		put_affine_portal();
	}
	return r;
}

#ifdef CONFIG_FSL_DPA_CAN_WAIT
static noinline struct bm_rcr_entry *__wait_rel_start(struct bman_portal **p,
					struct bman_pool *pool,
					__maybe_unused unsigned long *irqflags,
					u32 flags)
{
	struct bm_rcr_entry *rcr = try_rel_start(p, pool, irqflags, flags);
	if (!rcr)
		bm_rcr_set_ithresh(&(*p)->p, 1);
	return rcr;
}

static noinline struct bm_rcr_entry *wait_rel_start(struct bman_portal **p,
					struct bman_pool *pool,
					__maybe_unused unsigned long *irqflags,
					u32 flags)
{
	struct bm_rcr_entry *rcr;
#ifndef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	pool = NULL;
#endif
	if (flags & BMAN_RELEASE_FLAG_WAIT_INT)
		/* NB: return NULL if signal occurs before completion. Signal
		 * can occur during return. Caller must check for signal */
		wait_event_interruptible(affine_queue,
			(rcr = __wait_rel_start(p, pool, irqflags, flags)));
	else
		wait_event(affine_queue,
			(rcr = __wait_rel_start(p, pool, irqflags, flags)));
	return rcr;
}
#endif

static inline int __bman_release(struct bman_pool *pool,
			const struct bm_buffer *bufs, u8 num, u32 flags)
{
	struct bman_portal *p;
	struct bm_rcr_entry *r;
	__maybe_unused unsigned long irqflags;
	u32 i = num - 1;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & BMAN_RELEASE_FLAG_WAIT)
		r = wait_rel_start(&p, pool, &irqflags, flags);
	else
		r = try_rel_start(&p, pool, &irqflags, flags);
#else
	r = try_rel_start(&p, &irqflags, flags);
#endif
	if (!r)
		return -EBUSY;
	/* We can copy all but the first entry, as this can trigger badness
	 * with the valid-bit. Use the overlay to mask the verb byte. */
	r->bufs[0].opaque =
		((cpu_to_be64((bufs[0].opaque |
			      ((u64)pool->params.bpid<<48))
			      & 0x00ffffffffffffff)));
	if (i) {
		for (i = 1; i < num; i++)
			r->bufs[i].opaque =
				cpu_to_be64(bufs[i].opaque);
	}

	bm_rcr_pvb_commit(&p->p, BM_RCR_VERB_CMD_BPID_SINGLE |
			(num & BM_RCR_VERB_BUFCOUNT_MASK));
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	/* if we wish to sync we need to set the threshold after h/w sees the
	 * new ring entry. As we're mixing cache-enabled and cache-inhibited
	 * accesses, this requires a heavy-weight sync. */
	if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
			(flags & BMAN_RELEASE_FLAG_WAIT_SYNC))) {
		hwsync();
		bm_rcr_set_ithresh(&p->p, 1);
	}
#endif
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
			(flags & BMAN_RELEASE_FLAG_WAIT_SYNC))) {
		if (flags & BMAN_RELEASE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->rcri_owned != pool));
		else
			wait_event(affine_queue, (p->rcri_owned != pool));
	}
#endif
	return 0;
}

int bman_release(struct bman_pool *pool, const struct bm_buffer *bufs, u8 num,
			u32 flags)
{
	int ret;
#ifdef CONFIG_FSL_DPA_CHECKING
	if (!num || (num > 8))
		return -EINVAL;
	if (pool->params.flags & BMAN_POOL_FLAG_NO_RELEASE)
		return -EINVAL;
#endif
	/* Without stockpile, this API is a pass-through to the h/w operation */
	if (!(pool->params.flags & BMAN_POOL_FLAG_STOCKPILE))
		return __bman_release(pool, bufs, num, flags);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (!atomic_dec_and_test(&pool->in_use)) {
		pr_crit("Parallel attempts to enter bman_released() detected.");
		panic("only one instance of bman_released/acquired allowed");
	}
#endif
	/* Two movements of buffers are possible, and can occur in either order.
	 * A: moving buffers from the caller to the stockpile.
	 * B: moving buffers from the stockpile to hardware.
	 * Order 1: if there is already enough space in the stockpile for A
	 * then we want to do A first, and only do B if we trigger the
	 * stockpile-high threshold.
	 * Order 2: if there is not enough space in the stockpile for A, then
	 * we want to do B first, then do A if B had succeeded. However in this
	 * case B is dependent on how many buffers the user needs to release,
	 * not the stockpile-high threshold.
	 * Due to the different handling of B between the two cases, putting A
	 * and B in a while() loop would require quite obscure logic, so handle
	 * the different sequences explicitly. */
	if ((pool->sp_fill + num) <= BMAN_STOCKPILE_SZ) {
		/* Order 1: do A */
		copy_words(pool->sp + pool->sp_fill, bufs,
			   sizeof(struct bm_buffer) * num);
		pool->sp_fill += num;
		/* do B relative to STOCKPILE_HIGH */
		while (pool->sp_fill >= BMAN_STOCKPILE_HIGH) {
			ret = __bman_release(pool,
					     pool->sp + (pool->sp_fill - 8), 8,
					     flags);
			if (ret >= 0)
				pool->sp_fill -= 8;
		}
	} else {
		/* Order 2: do B relative to 'num' */
		do {
			ret = __bman_release(pool,
					     pool->sp + (pool->sp_fill - 8), 8,
					     flags);
			if (ret < 0)
				/* failure */
				goto release_done;
			pool->sp_fill -= 8;
		} while ((pool->sp_fill + num) > BMAN_STOCKPILE_SZ);
		/* do A */
		copy_words(pool->sp + pool->sp_fill, bufs,
			   sizeof(struct bm_buffer) * num);
		pool->sp_fill += num;
	}
	/* success */
	ret = 0;
release_done:
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_inc(&pool->in_use);
#endif
	return ret;
}
EXPORT_SYMBOL(bman_release);

static inline int __bman_acquire(struct bman_pool *pool, struct bm_buffer *bufs,
					u8 num)
{
	struct bman_portal *p = get_affine_portal();
	struct bm_mc_command *mcc;
	struct bm_mc_result *mcr;
	__maybe_unused unsigned long irqflags;
	int ret, i;

	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = bm_mc_start(&p->p);
	mcc->acquire.bpid = pool->params.bpid;
	bm_mc_commit(&p->p, BM_MCC_VERB_CMD_ACQUIRE |
			(num & BM_MCC_VERB_ACQUIRE_BUFCOUNT));
	while (!(mcr = bm_mc_result(&p->p)))
		cpu_relax();
	ret = mcr->verb & BM_MCR_VERB_ACQUIRE_BUFCOUNT;
	if (bufs) {
		for (i = 0; i < num; i++)
			bufs[i].opaque =
				be64_to_cpu(mcr->acquire.bufs[i].opaque);
	}
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (ret != num)
		ret = -ENOMEM;
	return ret;
}

int bman_acquire(struct bman_pool *pool, struct bm_buffer *bufs, u8 num,
			u32 flags)
{
	int ret;
#ifdef CONFIG_FSL_DPA_CHECKING
	if (!num || (num > 8))
		return -EINVAL;
	if (pool->params.flags & BMAN_POOL_FLAG_ONLY_RELEASE)
		return -EINVAL;
#endif
	/* Without stockpile, this API is a pass-through to the h/w operation */
	if (!(pool->params.flags & BMAN_POOL_FLAG_STOCKPILE))
		return __bman_acquire(pool, bufs, num);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (!atomic_dec_and_test(&pool->in_use)) {
		pr_crit("Parallel attempts to enter bman_acquire() detected.");
		panic("only one instance of bman_released/acquired allowed");
	}
#endif
	/* Two movements of buffers are possible, and can occur in either order.
	 * A: moving buffers from stockpile to the caller.
	 * B: moving buffers from hardware to the stockpile.
	 * Order 1: if there are already enough buffers in the stockpile for A
	 * then we want to do A first, and only do B if we trigger the
	 * stockpile-low threshold.
	 * Order 2: if there are not enough buffers in the stockpile for A,
	 * then we want to do B first, then do A if B had succeeded. However in
	 * this case B is dependent on how many buffers the user needs, not the
	 * stockpile-low threshold.
	 * Due to the different handling of B between the two cases, putting A
	 * and B in a while() loop would require quite obscure logic, so handle
	 * the different sequences explicitly. */
	if (num <= pool->sp_fill) {
		/* Order 1: do A */
		copy_words(bufs, pool->sp + (pool->sp_fill - num),
			   sizeof(struct bm_buffer) * num);
		pool->sp_fill -= num;
		/* do B relative to STOCKPILE_LOW */
		while (pool->sp_fill <= BMAN_STOCKPILE_LOW) {
			ret = __bman_acquire(pool, pool->sp + pool->sp_fill, 8);
			if (ret < 0)
				ret = __bman_acquire(pool,
						pool->sp + pool->sp_fill, 1);
			if (ret < 0)
				break;
			pool->sp_fill += ret;
		}
	} else {
		/* Order 2: do B relative to 'num' */
		do {
			ret = __bman_acquire(pool, pool->sp + pool->sp_fill, 8);
			if (ret < 0)
				ret = __bman_acquire(pool,
						pool->sp + pool->sp_fill, 1);
			if (ret < 0)
				/* failure */
				goto acquire_done;
			pool->sp_fill += ret;
		} while (pool->sp_fill < num);
		/* do A */
		copy_words(bufs, pool->sp + (pool->sp_fill - num),
			   sizeof(struct bm_buffer) * num);
		pool->sp_fill -= num;
	}
	/* success */
	ret = num;
acquire_done:
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_inc(&pool->in_use);
#endif
	return ret;
}
EXPORT_SYMBOL(bman_acquire);

int bman_flush_stockpile(struct bman_pool *pool, u32 flags)
{
	u8 num;
	int ret;

	while (pool->sp_fill) {
		num = ((pool->sp_fill > 8) ? 8 : pool->sp_fill);
		ret = __bman_release(pool, pool->sp + (pool->sp_fill - num),
				     num, flags);
		if (ret)
			return ret;
		pool->sp_fill -= num;
	}
	return 0;
}
EXPORT_SYMBOL(bman_flush_stockpile);

int bman_query_pools(struct bm_pool_state *state)
{
	struct bman_portal *p = get_affine_portal();
	struct bm_mc_result *mcr;
	__maybe_unused unsigned long irqflags;

	PORTAL_IRQ_LOCK(p, irqflags);
	bm_mc_start(&p->p);
	bm_mc_commit(&p->p, BM_MCC_VERB_CMD_QUERY);
	while (!(mcr = bm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & BM_MCR_VERB_CMD_MASK) == BM_MCR_VERB_CMD_QUERY);
	*state = mcr->query;
	state->as.state.__state[0] = be32_to_cpu(state->as.state.__state[0]);
	state->as.state.__state[1] = be32_to_cpu(state->as.state.__state[1]);
	state->ds.state.__state[0] = be32_to_cpu(state->ds.state.__state[0]);
	state->ds.state.__state[1] = be32_to_cpu(state->ds.state.__state[1]);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return 0;
}
EXPORT_SYMBOL(bman_query_pools);

#ifdef CONFIG_FSL_BMAN_CONFIG
u32 bman_query_free_buffers(struct bman_pool *pool)
{
	return bm_pool_free_buffers(pool->params.bpid);
}
EXPORT_SYMBOL(bman_query_free_buffers);

int bman_update_pool_thresholds(struct bman_pool *pool, const u32 *thresholds)
{
	u32 bpid;

	bpid = bman_get_params(pool)->bpid;

	return bm_pool_set(bpid, thresholds);
}
EXPORT_SYMBOL(bman_update_pool_thresholds);
#endif

int bman_shutdown_pool(u32 bpid)
{
	struct bman_portal *p = get_affine_portal();
	__maybe_unused unsigned long irqflags;
	int ret;

	PORTAL_IRQ_LOCK(p, irqflags);
	ret = bm_shutdown_pool(&p->p, bpid);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(bman_shutdown_pool);

const struct bm_portal_config *bman_get_bm_portal_config(
						struct bman_portal *portal)
{
	return portal->sharing_redirect ? NULL : portal->config;
}
