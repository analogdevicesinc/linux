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

#include "qman_low.h"

/* Compilation constants */
#define DQRR_MAXFILL	15
#define EQCR_ITHRESH	4	/* if EQCR congests, interrupt threshold */
#define IRQNAME		"QMan portal %d"
#define MAX_IRQNAME	16	/* big enough for "QMan portal %d" */

/* Divide 'n' by 'd', rounding down if 'r' is negative, rounding up if it's
 * positive, and rounding to the closest value if it's zero. NB, this macro
 * implicitly upgrades parameters to unsigned 64-bit, so feed it with types
 * that are compatible with this. NB, these arguments should not be expressions
 * unless it is safe for them to be evaluated multiple times. Eg. do not pass
 * in "some_value++" as a parameter to the macro! */
#define ROUNDING(n, d, r) \
	(((r) < 0) ? div64_u64((n), (d)) : \
	(((r) > 0) ? div64_u64(((n) + (d) - 1), (d)) : \
	div64_u64(((n) + ((d) / 2)), (d))))

/* Lock/unlock frame queues, subject to the "LOCKED" flag. This is about
 * inter-processor locking only. Note, FQLOCK() is always called either under a
 * local_irq_save() or from interrupt context - hence there's no need for irq
 * protection (and indeed, attempting to nest irq-protection doesn't work, as
 * the "irq en/disable" machinery isn't recursive...). */
#define FQLOCK(fq) \
	do { \
		struct qman_fq *__fq478 = (fq); \
		if (fq_isset(__fq478, QMAN_FQ_FLAG_LOCKED)) \
			spin_lock(&__fq478->fqlock); \
	} while (0)
#define FQUNLOCK(fq) \
	do { \
		struct qman_fq *__fq478 = (fq); \
		if (fq_isset(__fq478, QMAN_FQ_FLAG_LOCKED)) \
			spin_unlock(&__fq478->fqlock); \
	} while (0)

static inline void fq_set(struct qman_fq *fq, u32 mask)
{
	set_bits(mask, &fq->flags);
}
static inline void fq_clear(struct qman_fq *fq, u32 mask)
{
	clear_bits(mask, &fq->flags);
}
static inline int fq_isset(struct qman_fq *fq, u32 mask)
{
	return fq->flags & mask;
}
static inline int fq_isclear(struct qman_fq *fq, u32 mask)
{
	return !(fq->flags & mask);
}

struct qman_portal {
	struct qm_portal p;
	unsigned long bits; /* PORTAL_BITS_*** - dynamic, strictly internal */
	unsigned long irq_sources;
	u32 use_eqcr_ci_stashing;
	u32 slowpoll;	/* only used when interrupts are off */
	struct qman_fq *vdqcr_owned; /* only 1 volatile dequeue at a time */
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	struct qman_fq *eqci_owned; /* only 1 enqueue WAIT_SYNC at a time */
#endif
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	raw_spinlock_t sharing_lock; /* only used if is_shared */
	int is_shared;
	struct qman_portal *sharing_redirect;
#endif
	u32 sdqcr;
	int dqrr_disable_ref;
	/* A portal-specific handler for DCP ERNs. If this is NULL, the global
	 * handler is called instead. */
	qman_cb_dc_ern cb_dc_ern;
	/* When the cpu-affine portal is activated, this is non-NULL */
	const struct qm_portal_config *config;
	/* This is needed for providing a non-NULL device to dma_map_***() */
	struct platform_device *pdev;
	struct dpa_rbtree retire_table;
	char irqname[MAX_IRQNAME];
	/* 2-element array. cgrs[0] is mask, cgrs[1] is snapshot. */
	struct qman_cgrs *cgrs;
	/* linked-list of CSCN handlers. */
	struct list_head cgr_cbs;
	/* list lock */
	spinlock_t cgr_lock;
	/* 2-element array. ccgrs[0] is mask, ccgrs[1] is snapshot. */
	struct qman_ccgrs *ccgrs[QMAN_CEETM_MAX];
	/* 256-element array, each is a linked-list of CCSCN handlers. */
	struct list_head ccgr_cbs[QMAN_CEETM_MAX];
	/* list lock */
	spinlock_t ccgr_lock;
	/* track if memory was allocated by the driver */
	u8 alloced;
	/* power management data */
	u32 save_isdr;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	/* Keep a shadow copy of the DQRR on LE systems as the SW needs to
	 * do byte swaps of DQRR read only memory.  First entry must be aligned
	 * to 2 ** 10 to ensure DQRR index calculations based shadow copy
	 * address (6 bits for address shift + 4 bits for the DQRR size).
	 */
	struct qm_dqrr_entry shadow_dqrr[QM_DQRR_SIZE] __aligned(1024);
#endif
};

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

/* Global handler for DCP ERNs. Used when the portal receiving the message does
 * not have a portal-specific handler. */
static qman_cb_dc_ern cb_dc_ern;

static cpumask_t affine_mask;
static DEFINE_SPINLOCK(affine_mask_lock);
static u16 affine_channels[NR_CPUS];
static DEFINE_PER_CPU(struct qman_portal, qman_affine_portal);
void *affine_portals[NR_CPUS];

/* "raw" gets the cpu-local struct whether it's a redirect or not. */
static inline struct qman_portal *get_raw_affine_portal(void)
{
	return &get_cpu_var(qman_affine_portal);
}
/* For ops that can redirect, this obtains the portal to use */
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
static inline struct qman_portal *get_affine_portal(void)
{
	struct qman_portal *p = get_raw_affine_portal();
	if (p->sharing_redirect)
		return p->sharing_redirect;
	return p;
}
#else
#define get_affine_portal() get_raw_affine_portal()
#endif
/* For every "get", there must be a "put" */
static inline void put_affine_portal(void)
{
	put_cpu_var(qman_affine_portal);
}
/* Exception: poll functions assume the caller is cpu-affine and in no risk of
 * re-entrance, which are the two reasons we usually use the get/put_cpu_var()
 * semantic - ie. to disable pre-emption. Some use-cases expect the execution
 * context to remain as non-atomic during poll-triggered callbacks as it was
 * when the poll API was first called (eg. NAPI), so we go out of our way in
 * this case to not disable pre-emption. */
static inline struct qman_portal *get_poll_portal(void)
{
	return &get_cpu_var(qman_affine_portal);
}
#define put_poll_portal()

/* This gives a FQID->FQ lookup to cover the fact that we can't directly demux
 * retirement notifications (the fact they are sometimes h/w-consumed means that
 * contextB isn't always a s/w demux - and as we can't know which case it is
 * when looking at the notification, we have to use the slow lookup for all of
 * them). NB, it's possible to have multiple FQ objects refer to the same FQID
 * (though at most one of them should be the consumer), so this table isn't for
 * all FQs - FQs are added when retirement commands are issued, and removed when
 * they complete, which also massively reduces the size of this table. */
IMPLEMENT_DPA_RBTREE(fqtree, struct qman_fq, node, fqid);

/* This is what everything can wait on, even if it migrates to a different cpu
 * to the one whose affine portal it is waiting on. */
static DECLARE_WAIT_QUEUE_HEAD(affine_queue);

static inline int table_push_fq(struct qman_portal *p, struct qman_fq *fq)
{
	int ret = fqtree_push(&p->retire_table, fq);
	if (ret)
		pr_err("ERROR: double FQ-retirement %d\n", fq->fqid);
	return ret;
}

static inline void table_del_fq(struct qman_portal *p, struct qman_fq *fq)
{
	fqtree_del(&p->retire_table, fq);
}

static inline struct qman_fq *table_find_fq(struct qman_portal *p, u32 fqid)
{
	return fqtree_find(&p->retire_table, fqid);
}

#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
static void **qman_fq_lookup_table;
static size_t qman_fq_lookup_table_size;

int qman_setup_fq_lookup_table(size_t num_entries)
{
	num_entries++;
	/* Allocate 1 more entry since the first entry is not used */
	qman_fq_lookup_table = vzalloc((num_entries * sizeof(void *)));
	if (!qman_fq_lookup_table) {
		pr_err("QMan: Could not allocate fq lookup table\n");
		return -ENOMEM;
	}
	qman_fq_lookup_table_size = num_entries;
	pr_info("QMan: Allocated lookup table at %p, entry count %lu\n",
			qman_fq_lookup_table,
			(unsigned long)qman_fq_lookup_table_size);
	return 0;
}

/* global structure that maintains fq object mapping */
static DEFINE_SPINLOCK(fq_hash_table_lock);

static int find_empty_fq_table_entry(u32 *entry, struct qman_fq *fq)
{
	u32 i;

	spin_lock(&fq_hash_table_lock);
	/* Can't use index zero because this has special meaning
	 * in context_b field. */
	for (i = 1; i < qman_fq_lookup_table_size; i++) {
		if (qman_fq_lookup_table[i] == NULL) {
			*entry = i;
			qman_fq_lookup_table[i] = fq;
			spin_unlock(&fq_hash_table_lock);
			return 0;
		}
	}
	spin_unlock(&fq_hash_table_lock);
	return -ENOMEM;
}

static void clear_fq_table_entry(u32 entry)
{
	spin_lock(&fq_hash_table_lock);
	BUG_ON(entry >= qman_fq_lookup_table_size);
	qman_fq_lookup_table[entry] = NULL;
	spin_unlock(&fq_hash_table_lock);
}

static inline struct qman_fq *get_fq_table_entry(u32 entry)
{
	BUG_ON(entry >= qman_fq_lookup_table_size);
	return qman_fq_lookup_table[entry];
}
#endif

static inline void cpu_to_hw_fqd(struct qm_fqd *fqd)
{
	/* Byteswap the FQD to HW format */
	fqd->fq_ctrl = cpu_to_be16(fqd->fq_ctrl);
	fqd->dest_wq = cpu_to_be16(fqd->dest_wq);
	fqd->ics_cred = cpu_to_be16(fqd->ics_cred);
	fqd->context_b = cpu_to_be32(fqd->context_b);
	fqd->context_a.opaque = cpu_to_be64(fqd->context_a.opaque);
}

static inline void hw_fqd_to_cpu(struct qm_fqd *fqd)
{
	/* Byteswap the FQD to CPU format */
	fqd->fq_ctrl = be16_to_cpu(fqd->fq_ctrl);
	fqd->dest_wq = be16_to_cpu(fqd->dest_wq);
	fqd->ics_cred = be16_to_cpu(fqd->ics_cred);
	fqd->context_b = be32_to_cpu(fqd->context_b);
	fqd->context_a.opaque = be64_to_cpu(fqd->context_a.opaque);
}

/* Swap a 40 bit address */
static inline u64 cpu_to_be40(u64 in)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return in;
#else
	u64 out = 0;
	u8 *p = (u8 *) &out;
	p[0] = in >> 32;
	p[1] = in >> 24;
	p[2] = in >> 16;
	p[3] = in >> 8;
	p[4] = in >> 0;
	return out;
#endif
}
static inline u64 be40_to_cpu(u64 in)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return in;
#else
	u64 out = 0;
	u8 *pout = (u8 *) &out;
	u8 *pin = (u8 *) &in;
	pout[0] = pin[4];
	pout[1] = pin[3];
	pout[2] = pin[2];
	pout[3] = pin[1];
	pout[4] = pin[0];
	return out;
#endif
}

/* Swap a 24 bit value */
static inline u32 cpu_to_be24(u32 in)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return in;
#else
	u32 out = 0;
	u8 *p = (u8 *) &out;
	p[0] = in >> 16;
	p[1] = in >> 8;
	p[2] = in >> 0;
	return out;
#endif
}

static inline u32 be24_to_cpu(u32 in)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return in;
#else
	u32 out = 0;
	u8 *pout = (u8 *) &out;
	u8 *pin = (u8 *) &in;
	pout[0] = pin[2];
	pout[1] = pin[1];
	pout[2] = pin[0];
	return out;
#endif
}

static inline u64 be48_to_cpu(u64 in)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return in;
#else
	u64 out = 0;
	u8 *pout = (u8 *) &out;
	u8 *pin = (u8 *) &in;

	pout[0] = pin[5];
	pout[1] = pin[4];
	pout[2] = pin[3];
	pout[3] = pin[2];
	pout[4] = pin[1];
	pout[5] = pin[0];
	return out;
#endif
}
static inline void cpu_to_hw_fd(struct qm_fd *fd)
{
	fd->opaque_addr = cpu_to_be64(fd->opaque_addr);
	fd->status = cpu_to_be32(fd->status);
	fd->opaque = cpu_to_be32(fd->opaque);
}

static inline void hw_fd_to_cpu(struct qm_fd *fd)
{
	fd->opaque_addr = be64_to_cpu(fd->opaque_addr);
	fd->status = be32_to_cpu(fd->status);
	fd->opaque = be32_to_cpu(fd->opaque);
}

static inline void hw_cq_query_to_cpu(struct qm_mcr_ceetm_cq_query *cq_query)
{
	cq_query->ccgid = be16_to_cpu(cq_query->ccgid);
	cq_query->state = be16_to_cpu(cq_query->state);
	cq_query->pfdr_hptr = be24_to_cpu(cq_query->pfdr_hptr);
	cq_query->pfdr_tptr = be24_to_cpu(cq_query->pfdr_tptr);
	cq_query->od1_xsfdr = be16_to_cpu(cq_query->od1_xsfdr);
	cq_query->od2_xsfdr = be16_to_cpu(cq_query->od2_xsfdr);
	cq_query->od3_xsfdr = be16_to_cpu(cq_query->od3_xsfdr);
	cq_query->od4_xsfdr = be16_to_cpu(cq_query->od4_xsfdr);
	cq_query->od5_xsfdr = be16_to_cpu(cq_query->od5_xsfdr);
	cq_query->od6_xsfdr = be16_to_cpu(cq_query->od6_xsfdr);
	cq_query->ra1_xsfdr = be16_to_cpu(cq_query->ra1_xsfdr);
	cq_query->ra2_xsfdr = be16_to_cpu(cq_query->ra2_xsfdr);
	cq_query->frm_cnt = be24_to_cpu(cq_query->frm_cnt);
}

static inline void hw_ccgr_query_to_cpu(struct qm_mcr_ceetm_ccgr_query *ccgr_q)
{
	int i;

	ccgr_q->cm_query.cs_thres.hword =
		be16_to_cpu(ccgr_q->cm_query.cs_thres.hword);
	ccgr_q->cm_query.cs_thres_x.hword =
		be16_to_cpu(ccgr_q->cm_query.cs_thres_x.hword);
	ccgr_q->cm_query.td_thres.hword =
		be16_to_cpu(ccgr_q->cm_query.td_thres.hword);
	ccgr_q->cm_query.wr_parm_g.word =
	       be32_to_cpu(ccgr_q->cm_query.wr_parm_g.word);
	ccgr_q->cm_query.wr_parm_y.word =
		be32_to_cpu(ccgr_q->cm_query.wr_parm_y.word);
	ccgr_q->cm_query.wr_parm_r.word =
		be32_to_cpu(ccgr_q->cm_query.wr_parm_r.word);
	ccgr_q->cm_query.cscn_targ_dcp =
		be16_to_cpu(ccgr_q->cm_query.cscn_targ_dcp);
	ccgr_q->cm_query.i_cnt = be40_to_cpu(ccgr_q->cm_query.i_cnt);
	ccgr_q->cm_query.a_cnt = be40_to_cpu(ccgr_q->cm_query.a_cnt);
	for (i = 0; i < ARRAY_SIZE(ccgr_q->cm_query.cscn_targ_swp); i++)
		ccgr_q->cm_query.cscn_targ_swp[i] =
			be32_to_cpu(ccgr_q->cm_query.cscn_targ_swp[i]);
}

/* In the case that slow- and fast-path handling are both done by qman_poll()
 * (ie. because there is no interrupt handling), we ought to balance how often
 * we do the fast-path poll versus the slow-path poll. We'll use two decrementer
 * sources, so we call the fast poll 'n' times before calling the slow poll
 * once. The idle decrementer constant is used when the last slow-poll detected
 * no work to do, and the busy decrementer constant when the last slow-poll had
 * work to do. */
#define SLOW_POLL_IDLE   1000
#define SLOW_POLL_BUSY   10
static u32 __poll_portal_slow(struct qman_portal *p, u32 is);
static inline unsigned int __poll_portal_fast(struct qman_portal *p,
					unsigned int poll_limit);

/* Portal interrupt handler */
static irqreturn_t portal_isr(__always_unused int irq, void *ptr)
{
	struct qman_portal *p = ptr;
	/*
	 * The CSCI/CCSCI source is cleared inside __poll_portal_slow(), because
	 * it could race against a Query Congestion State command also given
	 * as part of the handling of this interrupt source. We mustn't
	 * clear it a second time in this top-level function.
	 */
	u32 clear = QM_DQAVAIL_MASK | (p->irq_sources &
		~(QM_PIRQ_CSCI | QM_PIRQ_CCSCI));
	u32 is = qm_isr_status_read(&p->p) & p->irq_sources;
	/* DQRR-handling if it's interrupt-driven */
	if (is & QM_PIRQ_DQRI)
		__poll_portal_fast(p, CONFIG_FSL_QMAN_POLL_LIMIT);
	/* Handling of anything else that's interrupt-driven */
	clear |= __poll_portal_slow(p, is);
	qm_isr_status_clear(&p->p, clear);
	return IRQ_HANDLED;
}

/* This inner version is used privately by qman_create_affine_portal(), as well
 * as by the exported qman_stop_dequeues(). */
static inline void qman_stop_dequeues_ex(struct qman_portal *p)
{
	unsigned long irqflags __maybe_unused;
	PORTAL_IRQ_LOCK(p, irqflags);
	if (!(p->dqrr_disable_ref++))
		qm_dqrr_set_maxfill(&p->p, 0);
	PORTAL_IRQ_UNLOCK(p, irqflags);
}

static int drain_mr_fqrni(struct qm_portal *p)
{
	const struct qm_mr_entry *msg;
loop:
	msg = qm_mr_current(p);
	if (!msg) {
		/* if MR was full and h/w had other FQRNI entries to produce, we
		 * need to allow it time to produce those entries once the
		 * existing entries are consumed. A worst-case situation
		 * (fully-loaded system) means h/w sequencers may have to do 3-4
		 * other things before servicing the portal's MR pump, each of
		 * which (if slow) may take ~50 qman cycles (which is ~200
		 * processor cycles). So rounding up and then multiplying this
		 * worst-case estimate by a factor of 10, just to be
		 * ultra-paranoid, goes as high as 10,000 cycles. NB, we consume
		 * one entry at a time, so h/w has an opportunity to produce new
		 * entries well before the ring has been fully consumed, so
		 * we're being *really* paranoid here. */
		u64 now, then = mfatb();
		do {
			now = mfatb();
		} while ((then + 10000) > now);
		msg = qm_mr_current(p);
		if (!msg)
			return 0;
	}
	if ((msg->verb & QM_MR_VERB_TYPE_MASK) != QM_MR_VERB_FQRNI) {
		/* We aren't draining anything but FQRNIs */
		pr_err("QMan found verb 0x%x in MR\n", msg->verb);
		return -1;
	}
	qm_mr_next(p);
	qm_mr_cci_consume(p, 1);
	goto loop;
}

#ifdef CONFIG_SUSPEND
static int _qman_portal_suspend_noirq(struct device *dev)
{
	struct qman_portal *p = (struct qman_portal *)dev->platform_data;
#ifdef CONFIG_PM_DEBUG
	struct platform_device *pdev = to_platform_device(dev);
#endif

	p->save_isdr = qm_isr_disable_read(&p->p);
	qm_isr_disable_write(&p->p, 0xffffffff);
	qm_isr_status_clear(&p->p, 0xffffffff);
#ifdef CONFIG_PM_DEBUG
	pr_info("Suspend for %s\n", pdev->name);
#endif
	return 0;
}

static int _qman_portal_resume_noirq(struct device *dev)
{
	struct qman_portal *p = (struct qman_portal *)dev->platform_data;

	/* restore isdr */
	qm_isr_disable_write(&p->p, p->save_isdr);
	return 0;
}
#else
#define _qman_portal_suspend_noirq NULL
#define _qman_portal_resume_noirq NULL
#endif

struct dev_pm_domain qman_portal_device_pm_domain = {
	.ops = {
		USE_PLATFORM_PM_SLEEP_OPS
		.suspend_noirq = _qman_portal_suspend_noirq,
		.resume_noirq = _qman_portal_resume_noirq,
	}
};

struct qman_portal *qman_create_portal(
			struct qman_portal *portal,
			const struct qm_portal_config *config,
			const struct qman_cgrs *cgrs)
{
	struct qm_portal *__p;
	char buf[16];
	int ret;
	u32 isdr;
	struct platform_device_info pdev_info;

	if (!portal) {
		portal = kmalloc(sizeof(*portal), GFP_KERNEL);
		if (!portal)
			return portal;
		portal->alloced = 1;
	} else
		portal->alloced = 0;

	__p = &portal->p;

#if (defined CONFIG_PPC || defined CONFIG_PPC64) && defined CONFIG_FSL_PAMU
        /* PAMU is required for stashing */
        portal->use_eqcr_ci_stashing = ((qman_ip_rev >= QMAN_REV30) ?
					1 : 0);
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	portal->use_eqcr_ci_stashing = 1;
#else
        portal->use_eqcr_ci_stashing = 0;
#endif

	/* prep the low-level portal struct with the mapped addresses from the
	 * config, everything that follows depends on it and "config" is more
	 * for (de)reference... */
	__p->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	__p->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];
	/*
	 * If CI-stashing is used, the current defaults use a threshold of 3,
	 * and stash with high-than-DQRR priority.
	 */
	if (qm_eqcr_init(__p, qm_eqcr_pvb,
			portal->use_eqcr_ci_stashing ? 3 : 0, 1)) {
		pr_err("Qman EQCR initialisation failed\n");
		goto fail_eqcr;
	}
	if (qm_dqrr_init(__p, config, qm_dqrr_dpush, qm_dqrr_pvb,
			qm_dqrr_cdc, DQRR_MAXFILL)) {
		pr_err("Qman DQRR initialisation failed\n");
		goto fail_dqrr;
	}
	if (qm_mr_init(__p, qm_mr_pvb, qm_mr_cci)) {
		pr_err("Qman MR initialisation failed\n");
		goto fail_mr;
	}
	if (qm_mc_init(__p)) {
		pr_err("Qman MC initialisation failed\n");
		goto fail_mc;
	}
	if (qm_isr_init(__p)) {
		pr_err("Qman ISR initialisation failed\n");
		goto fail_isr;
	}
	/* static interrupt-gating controls */
	qm_dqrr_set_ithresh(__p, CONFIG_FSL_QMAN_PIRQ_DQRR_ITHRESH);
	qm_mr_set_ithresh(__p, CONFIG_FSL_QMAN_PIRQ_MR_ITHRESH);
	qm_isr_set_iperiod(__p, CONFIG_FSL_QMAN_PIRQ_IPERIOD);
	portal->cgrs = kmalloc(2 * sizeof(*cgrs), GFP_KERNEL);
	if (!portal->cgrs)
		goto fail_cgrs;
	/* initial snapshot is no-depletion */
	qman_cgrs_init(&portal->cgrs[1]);
	if (cgrs)
		portal->cgrs[0] = *cgrs;
	else
		/* if the given mask is NULL, assume all CGRs can be seen */
		qman_cgrs_fill(&portal->cgrs[0]);
	INIT_LIST_HEAD(&portal->cgr_cbs);
	spin_lock_init(&portal->cgr_lock);
	if (num_ceetms) {
		for (ret = 0; ret < num_ceetms; ret++) {
			portal->ccgrs[ret] = kmalloc(2 *
				sizeof(struct qman_ccgrs), GFP_KERNEL);
			if (!portal->ccgrs[ret])
				goto fail_ccgrs;
			qman_ccgrs_init(&portal->ccgrs[ret][1]);
			qman_ccgrs_fill(&portal->ccgrs[ret][0]);
			INIT_LIST_HEAD(&portal->ccgr_cbs[ret]);
		}
	}
	spin_lock_init(&portal->ccgr_lock);
	portal->bits = 0;
	portal->slowpoll = 0;
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	portal->eqci_owned = NULL;
#endif
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	raw_spin_lock_init(&portal->sharing_lock);
	portal->is_shared = config->public_cfg.is_shared;
	portal->sharing_redirect = NULL;
#endif
	portal->sdqcr = QM_SDQCR_SOURCE_CHANNELS | QM_SDQCR_COUNT_UPTO3 |
			QM_SDQCR_DEDICATED_PRECEDENCE | QM_SDQCR_TYPE_PRIO_QOS |
			QM_SDQCR_TOKEN_SET(0xab) | QM_SDQCR_CHANNELS_DEDICATED;
	portal->dqrr_disable_ref = 0;
	portal->cb_dc_ern = NULL;
	sprintf(buf, "qportal-%d", config->public_cfg.channel);

	memset(&pdev_info, 0, sizeof(pdev_info));
	pdev_info.name = buf;
	pdev_info.id = PLATFORM_DEVID_NONE;
	pdev_info.dma_mask = DMA_BIT_MASK(40);

	portal->pdev = platform_device_register_full(&pdev_info);
	if (!portal->pdev) {
		pr_err("qman_portal - platform_device_alloc() failed\n");
		goto fail_devregister;
	}

	arch_setup_dma_ops(&portal->pdev->dev, 0, 0, NULL, true);

	portal->pdev->dev.pm_domain = &qman_portal_device_pm_domain;
	portal->pdev->dev.platform_data = portal;
	dpa_rbtree_init(&portal->retire_table);
	isdr = 0xffffffff;
	qm_isr_disable_write(__p, isdr);
	portal->irq_sources = 0;
	qm_isr_enable_write(__p, portal->irq_sources);
	qm_isr_status_clear(__p, 0xffffffff);
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
		pr_err("irq_set_affinity() failed\n");
		goto fail_affinity;
	}

	/* Need EQCR to be empty before continuing */
	isdr ^= QM_PIRQ_EQCI;
	qm_isr_disable_write(__p, isdr);
	ret = qm_eqcr_get_fill(__p);
	if (ret) {
		pr_err("Qman EQCR unclean\n");
		goto fail_eqcr_empty;
	}
	isdr ^= (QM_PIRQ_DQRI | QM_PIRQ_MRI);
	qm_isr_disable_write(__p, isdr);
	if (qm_dqrr_current(__p) != NULL) {
		pr_err("Qman DQRR unclean\n");
		qm_dqrr_cdc_consume_n(__p, 0xffff);
	}
	if (qm_mr_current(__p) != NULL) {
		/* special handling, drain just in case it's a few FQRNIs */
		if (drain_mr_fqrni(__p)) {
			const struct qm_mr_entry *e = qm_mr_current(__p);
			/*
			 * Message ring cannot be empty no need to check
			 * qm_mr_current returned successfully
			 */
			pr_err("Qman MR unclean, MR VERB 0x%x, rc 0x%x\n, addr 0x%x",
				e->verb, e->ern.rc, e->ern.fd.addr_lo);
			goto fail_dqrr_mr_empty;
		}
	}
	/* Success */
	portal->config = config;
	qm_isr_disable_write(__p, 0);
	qm_isr_uninhibit(__p);
	/* Write a sane SDQCR */
	qm_dqrr_sdqcr_set(__p, portal->sdqcr);
	return portal;
fail_dqrr_mr_empty:
fail_eqcr_empty:
fail_affinity:
	free_irq(config->public_cfg.irq, portal);
fail_irq:
	platform_device_unregister(portal->pdev);
fail_devregister:
	if (num_ceetms)
		for (ret = 0; ret < num_ceetms; ret++)
			kfree(portal->ccgrs[ret]);
fail_ccgrs:
	kfree(portal->cgrs);
fail_cgrs:
	qm_isr_finish(__p);
fail_isr:
	qm_mc_finish(__p);
fail_mc:
	qm_mr_finish(__p);
fail_mr:
	qm_dqrr_finish(__p);
fail_dqrr:
	qm_eqcr_finish(__p);
fail_eqcr:
	if (portal->alloced)
		kfree(portal);
	return NULL;
}

struct qman_portal *qman_create_affine_portal(
			const struct qm_portal_config *config,
			const struct qman_cgrs *cgrs)
{
	struct qman_portal *res;
	struct qman_portal *portal;

	portal = &per_cpu(qman_affine_portal, config->public_cfg.cpu);
	res = qman_create_portal(portal, config, cgrs);
	if (res) {
		spin_lock(&affine_mask_lock);
		cpumask_set_cpu(config->public_cfg.cpu, &affine_mask);
		affine_channels[config->public_cfg.cpu] =
			config->public_cfg.channel;
		affine_portals[config->public_cfg.cpu] = portal;
		spin_unlock(&affine_mask_lock);
	}
	return res;
}

/* These checks are BUG_ON()s because the driver is already supposed to avoid
 * these cases. */
struct qman_portal *qman_create_affine_slave(struct qman_portal *redirect,
								int cpu)
{
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	struct qman_portal *p;
	p = &per_cpu(qman_affine_portal, cpu);
	/* Check that we don't already have our own portal */
	BUG_ON(p->config);
	/* Check that we aren't already slaving to another portal */
	BUG_ON(p->is_shared);
	/* Check that 'redirect' is prepared to have us */
	BUG_ON(!redirect->config->public_cfg.is_shared);
	/* These are the only elements to initialise when redirecting */
	p->irq_sources = 0;
	p->sharing_redirect = redirect;
	affine_portals[cpu] = p;
	return p;
#else
	BUG();
	return NULL;
#endif
}

void qman_destroy_portal(struct qman_portal *qm)
{
	const struct qm_portal_config *pcfg;
	int i;

	/* Stop dequeues on the portal */
	qm_dqrr_sdqcr_set(&qm->p, 0);

	/* NB we do this to "quiesce" EQCR. If we add enqueue-completions or
	 * something related to QM_PIRQ_EQCI, this may need fixing.
	 * Also, due to the prefetching model used for CI updates in the enqueue
	 * path, this update will only invalidate the CI cacheline *after*
	 * working on it, so we need to call this twice to ensure a full update
	 * irrespective of where the enqueue processing was at when the teardown
	 * began. */
	qm_eqcr_cce_update(&qm->p);
	qm_eqcr_cce_update(&qm->p);
	pcfg = qm->config;

	free_irq(pcfg->public_cfg.irq, qm);

	kfree(qm->cgrs);
	if (num_ceetms)
		for (i = 0; i < num_ceetms; i++)
			kfree(qm->ccgrs[i]);
	qm_isr_finish(&qm->p);
	qm_mc_finish(&qm->p);
	qm_mr_finish(&qm->p);
	qm_dqrr_finish(&qm->p);
	qm_eqcr_finish(&qm->p);

	platform_device_unregister(qm->pdev);

	qm->config = NULL;
	if (qm->alloced)
		kfree(qm);
}

const struct qm_portal_config *qman_destroy_affine_portal(void)
{
	/* We don't want to redirect if we're a slave, use "raw" */
	struct qman_portal *qm = get_raw_affine_portal();
	const struct qm_portal_config *pcfg;
	int cpu;
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (qm->sharing_redirect) {
		qm->sharing_redirect = NULL;
		put_affine_portal();
		return NULL;
	}
	qm->is_shared = 0;
#endif
	pcfg = qm->config;
	cpu = pcfg->public_cfg.cpu;

	qman_destroy_portal(qm);

	spin_lock(&affine_mask_lock);
	cpumask_clear_cpu(cpu, &affine_mask);
	spin_unlock(&affine_mask_lock);
	put_affine_portal();
	return pcfg;
}

const struct qman_portal_config *qman_p_get_portal_config(struct qman_portal *p)
{
	return &p->config->public_cfg;
}
EXPORT_SYMBOL(qman_p_get_portal_config);

const struct qman_portal_config *qman_get_portal_config(void)
{
	struct qman_portal *p = get_affine_portal();
	const struct qman_portal_config *ret = qman_p_get_portal_config(p);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_get_portal_config);

/* Inline helper to reduce nesting in __poll_portal_slow() */
static inline void fq_state_change(struct qman_portal *p, struct qman_fq *fq,
				const struct qm_mr_entry *msg, u8 verb)
{
	FQLOCK(fq);
	switch (verb) {
	case QM_MR_VERB_FQRL:
		DPA_ASSERT(fq_isset(fq, QMAN_FQ_STATE_ORL));
		fq_clear(fq, QMAN_FQ_STATE_ORL);
		table_del_fq(p, fq);
		break;
	case QM_MR_VERB_FQRN:
		DPA_ASSERT((fq->state == qman_fq_state_parked) ||
			(fq->state == qman_fq_state_sched));
		DPA_ASSERT(fq_isset(fq, QMAN_FQ_STATE_CHANGING));
		fq_clear(fq, QMAN_FQ_STATE_CHANGING);
		if (msg->fq.fqs & QM_MR_FQS_NOTEMPTY)
			fq_set(fq, QMAN_FQ_STATE_NE);
		if (msg->fq.fqs & QM_MR_FQS_ORLPRESENT)
			fq_set(fq, QMAN_FQ_STATE_ORL);
		else
			table_del_fq(p, fq);
		fq->state = qman_fq_state_retired;
		break;
	case QM_MR_VERB_FQPN:
		DPA_ASSERT(fq->state == qman_fq_state_sched);
		DPA_ASSERT(fq_isclear(fq, QMAN_FQ_STATE_CHANGING));
		fq->state = qman_fq_state_parked;
	}
	FQUNLOCK(fq);
}

static u32 __poll_portal_slow(struct qman_portal *p, u32 is)
{
	const struct qm_mr_entry *msg;
	struct qm_mr_entry swapped_msg;
	int k;

	if (is & QM_PIRQ_CSCI) {
		struct qman_cgrs rr, c;
		struct qm_mc_result *mcr;
		struct qman_cgr *cgr;
		unsigned long irqflags __maybe_unused;

		spin_lock_irqsave(&p->cgr_lock, irqflags);
		/*
		 * The CSCI bit must be cleared _before_ issuing the
		 * Query Congestion State command, to ensure that a long
		 * CGR State Change callback cannot miss an intervening
		 * state change.
		 */
		qm_isr_status_clear(&p->p, QM_PIRQ_CSCI);
		qm_mc_start(&p->p);
		qm_mc_commit(&p->p, QM_MCC_VERB_QUERYCONGESTION);
		while (!(mcr = qm_mc_result(&p->p)))
			cpu_relax();
		for (k = 0; k < 8; k++)
			mcr->querycongestion.state.__state[k] = be32_to_cpu(
					mcr->querycongestion.state.__state[k]);
		/* mask out the ones I'm not interested in */
		qman_cgrs_and(&rr, (const struct qman_cgrs *)
			&mcr->querycongestion.state, &p->cgrs[0]);
		/* check previous snapshot for delta, enter/exit congestion */
		qman_cgrs_xor(&c, &rr, &p->cgrs[1]);
		/* update snapshot */
		qman_cgrs_cp(&p->cgrs[1], &rr);
		/* Invoke callback */
		list_for_each_entry(cgr, &p->cgr_cbs, node)
			if (cgr->cb && qman_cgrs_get(&c, cgr->cgrid))
				cgr->cb(p, cgr, qman_cgrs_get(&rr, cgr->cgrid));
		spin_unlock_irqrestore(&p->cgr_lock, irqflags);
	}
	if (is & QM_PIRQ_CCSCI) {
		struct qman_ccgrs rr, c, congestion_result;
		struct qm_mc_result *mcr;
		struct qm_mc_command *mcc;
		struct qm_ceetm_ccg *ccg;
		unsigned long irqflags __maybe_unused;
		int i, j;

		spin_lock_irqsave(&p->ccgr_lock, irqflags);
		/*
		 * The CCSCI bit must be cleared _before_ issuing the
		 * Query Congestion State command, to ensure that a long
		 * CCGR State Change callback cannot miss an intervening
		 * state change.
		 */
		qm_isr_status_clear(&p->p, QM_PIRQ_CCSCI);

		for (i = 0; i < num_ceetms; i++) {
			for (j = 0; j < 2; j++) {
				mcc = qm_mc_start(&p->p);
				mcc->ccgr_query.ccgrid = cpu_to_be16(
					CEETM_QUERY_CONGESTION_STATE | j);
				mcc->ccgr_query.dcpid = i;
				qm_mc_commit(&p->p, QM_CEETM_VERB_CCGR_QUERY);
				while (!(mcr = qm_mc_result(&p->p)))
					cpu_relax();
				for (k = 0; k < 8; k++)
					mcr->ccgr_query.congestion_state.state.
						__state[k] = be32_to_cpu(
						mcr->ccgr_query.
						congestion_state.state.
						__state[k]);
				congestion_result.q[j] =
					mcr->ccgr_query.congestion_state.state;
			}
			/* mask out the ones I'm not interested in */
			qman_ccgrs_and(&rr, &congestion_result,
							&p->ccgrs[i][0]);
			/*
			 * check previous snapshot for delta, enter/exit
			 * congestion.
			 */
			qman_ccgrs_xor(&c, &rr, &p->ccgrs[i][1]);
			/* update snapshot */
			qman_ccgrs_cp(&p->ccgrs[i][1], &rr);
			/* Invoke callback */
			list_for_each_entry(ccg, &p->ccgr_cbs[i], cb_node)
				if (ccg->cb && qman_ccgrs_get(&c,
					(ccg->parent->idx << 4) | ccg->idx))
					ccg->cb(ccg, ccg->cb_ctx,
						qman_ccgrs_get(&rr,
							(ccg->parent->idx << 4)
							| ccg->idx));
		}
		spin_unlock_irqrestore(&p->ccgr_lock, irqflags);
	}

#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (is & QM_PIRQ_EQCI) {
		unsigned long irqflags;
		PORTAL_IRQ_LOCK(p, irqflags);
		p->eqci_owned = NULL;
		PORTAL_IRQ_UNLOCK(p, irqflags);
		wake_up(&affine_queue);
	}
#endif

	if (is & QM_PIRQ_EQRI) {
		unsigned long irqflags __maybe_unused;
		PORTAL_IRQ_LOCK(p, irqflags);
		qm_eqcr_cce_update(&p->p);
		qm_eqcr_set_ithresh(&p->p, 0);
		PORTAL_IRQ_UNLOCK(p, irqflags);
		wake_up(&affine_queue);
	}

	if (is & QM_PIRQ_MRI) {
		struct qman_fq *fq;
		u8 verb, num = 0;
mr_loop:
		qm_mr_pvb_update(&p->p);
		msg = qm_mr_current(&p->p);
		if (!msg)
			goto mr_done;
		swapped_msg = *msg;
		hw_fd_to_cpu(&swapped_msg.ern.fd);
		verb = msg->verb & QM_MR_VERB_TYPE_MASK;
		/* The message is a software ERN iff the 0x20 bit is set */
		if (verb & 0x20) {
			switch (verb) {
			case QM_MR_VERB_FQRNI:
				/* nada, we drop FQRNIs on the floor */
				break;
			case QM_MR_VERB_FQRN:
			case QM_MR_VERB_FQRL:
				/* Lookup in the retirement table */
				fq = table_find_fq(p, be32_to_cpu(msg->fq.fqid));
				BUG_ON(!fq);
				fq_state_change(p, fq, &swapped_msg, verb);
				if (fq->cb.fqs)
					fq->cb.fqs(p, fq, &swapped_msg);
				break;
			case QM_MR_VERB_FQPN:
				/* Parked */
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
				fq = get_fq_table_entry(
					be32_to_cpu(msg->fq.contextB));
#else
				fq = (void *)(uintptr_t)
					be32_to_cpu(msg->fq.contextB);
#endif
				fq_state_change(p, fq, msg, verb);
				if (fq->cb.fqs)
					fq->cb.fqs(p, fq, &swapped_msg);
				break;
			case QM_MR_VERB_DC_ERN:
				/* DCP ERN */
				if (p->cb_dc_ern)
					p->cb_dc_ern(p, msg);
				else if (cb_dc_ern)
					cb_dc_ern(p, msg);
				else {
					static int warn_once;
					if (!warn_once) {
						pr_crit("Leaking DCP ERNs!\n");
						warn_once = 1;
					}
				}
				break;
			default:
				pr_crit("Invalid MR verb 0x%02x\n", verb);
			}
		} else {
			/* Its a software ERN */
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
			fq = get_fq_table_entry(be32_to_cpu(msg->ern.tag));
#else
			fq = (void *)(uintptr_t)be32_to_cpu(msg->ern.tag);
#endif
			fq->cb.ern(p, fq, &swapped_msg);
		}
		num++;
		qm_mr_next(&p->p);
		goto mr_loop;
mr_done:
		qm_mr_cci_consume(&p->p, num);
	}
	/*
	 * QM_PIRQ_CSCI/CCSCI has already been cleared, as part of its specific
	 * processing. If that interrupt source has meanwhile been re-asserted,
	 * we mustn't clear it here (or in the top-level interrupt handler).
	 */
	return is & (QM_PIRQ_EQCI | QM_PIRQ_EQRI | QM_PIRQ_MRI);
}

/* remove some slowish-path stuff from the "fast path" and make sure it isn't
 * inlined. */
static noinline void clear_vdqcr(struct qman_portal *p, struct qman_fq *fq)
{
	p->vdqcr_owned = NULL;
	FQLOCK(fq);
	fq_clear(fq, QMAN_FQ_STATE_VDQCR);
	FQUNLOCK(fq);
	wake_up(&affine_queue);
}

/* Copy a DQRR entry ensuring reads reach QBMan in order */
static inline void safe_copy_dqrr(struct qm_dqrr_entry *dst,
				  const struct qm_dqrr_entry *src)
{
	int i = 0;
	const u64 *s64 = (u64*)src;
	u64 *d64 = (u64*)dst;

	/* DQRR only has 32 bytes of valid data so only need to
	 * copy 4 - 64 bit values */
	*d64 = *s64;
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	{
		u32 res, zero = 0;
		/* Create a dependancy after copying first bytes ensures no wrap
		   transaction generated to QBMan */
		/* Logical AND the value pointed to by s64 with 0x0 and
		   store the result in res */
		asm volatile("and %[result], %[in1], %[in2]"
			     : [result] "=r" (res)
			     : [in1] "r" (zero), [in2] "r" (*s64)
			     : "memory");
		/* Add res to s64 - this creates a dependancy on the result of
		   reading the value of s64 before the next read. The side
		   effect of this is that the core must stall until the first
		   aligned read is complete therefore preventing a WRAP
		   transaction to be seen by the QBMan */
		asm volatile("add %[result], %[in1], %[in2]"
			     : [result] "=r" (s64)
			     : [in1] "r" (res), [in2] "r" (s64)
			     : "memory");
	}
#endif
	/* Copy the last 3 64 bit parts */
	d64++; s64++;
	for (;i<3; i++)
		*d64++ = *s64++;
}

/* Look: no locks, no irq_save()s, no preempt_disable()s! :-) The only states
 * that would conflict with other things if they ran at the same time on the
 * same cpu are;
 *
 *   (i) setting/clearing vdqcr_owned, and
 *  (ii) clearing the NE (Not Empty) flag.
 *
 * Both are safe. Because;
 *
 *   (i) this clearing can only occur after qman_volatile_dequeue() has set the
 *       vdqcr_owned field (which it does before setting VDQCR), and
 *       qman_volatile_dequeue() blocks interrupts and preemption while this is
 *       done so that we can't interfere.
 *  (ii) the NE flag is only cleared after qman_retire_fq() has set it, and as
 *       with (i) that API prevents us from interfering until it's safe.
 *
 * The good thing is that qman_volatile_dequeue() and qman_retire_fq() run far
 * less frequently (ie. per-FQ) than __poll_portal_fast() does, so the nett
 * advantage comes from this function not having to "lock" anything at all.
 *
 * Note also that the callbacks are invoked at points which are safe against the
 * above potential conflicts, but that this function itself is not re-entrant
 * (this is because the function tracks one end of each FIFO in the portal and
 * we do *not* want to lock that). So the consequence is that it is safe for
 * user callbacks to call into any Qman API *except* qman_poll() (as that's the
 * sole API that could be invoking the callback through this function).
 */
static inline unsigned int __poll_portal_fast(struct qman_portal *p,
					unsigned int poll_limit)
{
	const struct qm_dqrr_entry *dq;
	struct qman_fq *fq;
	enum qman_cb_dqrr_result res;
	unsigned int limit = 0;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	struct qm_dqrr_entry *shadow;
	const struct qm_dqrr_entry *orig_dq;
#endif
loop:
	qm_dqrr_pvb_update(&p->p);
	dq = qm_dqrr_current(&p->p);
	if (!dq)
		goto done;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	/* If running on an LE system the fields of the
	   dequeue entry must be swapped.  Because the
	   QMan HW will ignore writes the DQRR entry is
	   copied and the index stored within the copy */
	shadow = &p->shadow_dqrr[DQRR_PTR2IDX(dq)];
	/* Use safe copy here to avoid WRAP transaction */
	safe_copy_dqrr(shadow, dq);
	orig_dq = dq;
	dq = shadow;
	shadow->fqid = be32_to_cpu(shadow->fqid);
	shadow->contextB = be32_to_cpu(shadow->contextB);
	shadow->seqnum = be16_to_cpu(shadow->seqnum);
	hw_fd_to_cpu(&shadow->fd);
#endif
	if (dq->stat & QM_DQRR_STAT_UNSCHEDULED) {
		/* VDQCR: don't trust contextB as the FQ may have been
		 * configured for h/w consumption and we're draining it
		 * post-retirement. */
		fq = p->vdqcr_owned;
		/* We only set QMAN_FQ_STATE_NE when retiring, so we only need
		 * to check for clearing it when doing volatile dequeues. It's
		 * one less thing to check in the critical path (SDQCR). */
		if (dq->stat & QM_DQRR_STAT_FQ_EMPTY)
			fq_clear(fq, QMAN_FQ_STATE_NE);
		/* this is duplicated from the SDQCR code, but we have stuff to
		 * do before *and* after this callback, and we don't want
		 * multiple if()s in the critical path (SDQCR). */
		res = fq->cb.dqrr(p, fq, dq);
		if (res == qman_cb_dqrr_stop)
			goto done;
		/* Check for VDQCR completion */
		if (dq->stat & QM_DQRR_STAT_DQCR_EXPIRED)
			clear_vdqcr(p, fq);
	} else {
		/* SDQCR: contextB points to the FQ */
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
		fq = get_fq_table_entry(dq->contextB);
#else
		fq = (void *)(uintptr_t)dq->contextB;
#endif
		/* Now let the callback do its stuff */
		res = fq->cb.dqrr(p, fq, dq);

		/* The callback can request that we exit without consuming this
		 * entry nor advancing; */
		if (res == qman_cb_dqrr_stop)
			goto done;
	}
	/* Interpret 'dq' from a driver perspective. */
	/* Parking isn't possible unless HELDACTIVE was set. NB,
	 * FORCEELIGIBLE implies HELDACTIVE, so we only need to
	 * check for HELDACTIVE to cover both. */
	DPA_ASSERT((dq->stat & QM_DQRR_STAT_FQ_HELDACTIVE) ||
		(res != qman_cb_dqrr_park));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	if (res != qman_cb_dqrr_defer)
		qm_dqrr_cdc_consume_1ptr(&p->p, orig_dq,
					 (res == qman_cb_dqrr_park));
#else
	/* Defer just means "skip it, I'll consume it myself later on" */
	if (res != qman_cb_dqrr_defer)
		qm_dqrr_cdc_consume_1ptr(&p->p, dq, (res == qman_cb_dqrr_park));
#endif
	/* Move forward */
	qm_dqrr_next(&p->p);
	/* Entry processed and consumed, increment our counter. The callback can
	 * request that we exit after consuming the entry, and we also exit if
	 * we reach our processing limit, so loop back only if neither of these
	 * conditions is met. */
	if ((++limit < poll_limit) && (res != qman_cb_dqrr_consume_stop))
		goto loop;
done:
	return limit;
}

u32 qman_irqsource_get(void)
{
	/* "irqsource" and "poll" APIs mustn't redirect when sharing, they
	 * should shut the user out if they are not the primary CPU hosting the
	 * portal. That's why we use the "raw" interface. */
	struct qman_portal *p = get_raw_affine_portal();
	u32 ret = p->irq_sources & QM_PIRQ_VISIBLE;
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_irqsource_get);

int qman_p_irqsource_add(struct qman_portal *p, u32 bits __maybe_unused)
{
	__maybe_unused unsigned long irqflags;
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (p->sharing_redirect)
		return -EINVAL;
	else
#endif
	{
		bits = bits & QM_PIRQ_VISIBLE;
		PORTAL_IRQ_LOCK(p, irqflags);

		/* Clear any previously remaining interrupt conditions in
		 * QCSP_ISR. This prevents raising a false interrupt when
		 * interrupt conditions are enabled in QCSP_IER.
		 */
		qm_isr_status_clear(&p->p, bits);
		set_bits(bits, &p->irq_sources);
		qm_isr_enable_write(&p->p, p->irq_sources);
		PORTAL_IRQ_UNLOCK(p, irqflags);
	}
	return 0;
}
EXPORT_SYMBOL(qman_p_irqsource_add);

int qman_irqsource_add(u32 bits __maybe_unused)
{
	struct qman_portal *p = get_raw_affine_portal();
	int ret;
	ret = qman_p_irqsource_add(p, bits);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_irqsource_add);

int qman_p_irqsource_remove(struct qman_portal *p, u32 bits)
{
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
	bits &= QM_PIRQ_VISIBLE;
	clear_bits(bits, &p->irq_sources);
	qm_isr_enable_write(&p->p, p->irq_sources);

	ier = qm_isr_enable_read(&p->p);
	/* Using "~ier" (rather than "bits" or "~p->irq_sources") creates a
	 * data-dependency, ie. to protect against re-ordering. */
	qm_isr_status_clear(&p->p, ~ier);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	return 0;
}
EXPORT_SYMBOL(qman_p_irqsource_remove);

int qman_irqsource_remove(u32 bits)
{
	struct qman_portal *p = get_raw_affine_portal();
	int ret;
	ret = qman_p_irqsource_remove(p, bits);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_irqsource_remove);

const cpumask_t *qman_affine_cpus(void)
{
	return &affine_mask;
}
EXPORT_SYMBOL(qman_affine_cpus);

u16 qman_affine_channel(int cpu)
{
	if (cpu < 0) {
		struct qman_portal *portal = get_raw_affine_portal();
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
		BUG_ON(portal->sharing_redirect);
#endif
		cpu = portal->config->public_cfg.cpu;
		put_affine_portal();
	}
	BUG_ON(!cpumask_test_cpu(cpu, &affine_mask));
	return affine_channels[cpu];
}
EXPORT_SYMBOL(qman_affine_channel);

void *qman_get_affine_portal(int cpu)
{
	return affine_portals[cpu];
}
EXPORT_SYMBOL(qman_get_affine_portal);

int qman_p_poll_dqrr(struct qman_portal *p, unsigned int limit)
{
	int ret;

#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (unlikely(p->sharing_redirect))
		ret = -EINVAL;
	else
#endif
	{
		BUG_ON(p->irq_sources & QM_PIRQ_DQRI);
		ret = __poll_portal_fast(p, limit);
	}
	return ret;
}
EXPORT_SYMBOL(qman_p_poll_dqrr);

int qman_poll_dqrr(unsigned int limit)
{
	struct qman_portal *p = get_poll_portal();
	int ret;
	ret = qman_p_poll_dqrr(p, limit);
	put_poll_portal();
	return ret;
}
EXPORT_SYMBOL(qman_poll_dqrr);

u32 qman_p_poll_slow(struct qman_portal *p)
{
	u32 ret;
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (unlikely(p->sharing_redirect))
		ret = (u32)-1;
	else
#endif
	{
		u32 is = qm_isr_status_read(&p->p) & ~p->irq_sources;
		ret = __poll_portal_slow(p, is);
		qm_isr_status_clear(&p->p, ret);
	}
	return ret;
}
EXPORT_SYMBOL(qman_p_poll_slow);

u32 qman_poll_slow(void)
{
	struct qman_portal *p = get_poll_portal();
	u32 ret;
	ret = qman_p_poll_slow(p);
	put_poll_portal();
	return ret;
}
EXPORT_SYMBOL(qman_poll_slow);

/* Legacy wrapper */
void qman_p_poll(struct qman_portal *p)
{
#ifdef CONFIG_FSL_DPA_PORTAL_SHARE
	if (unlikely(p->sharing_redirect))
		return;
#endif
	if ((~p->irq_sources) & QM_PIRQ_SLOW) {
		if (!(p->slowpoll--)) {
			u32 is = qm_isr_status_read(&p->p) & ~p->irq_sources;
			u32 active = __poll_portal_slow(p, is);
			if (active) {
				qm_isr_status_clear(&p->p, active);
				p->slowpoll = SLOW_POLL_BUSY;
			} else
				p->slowpoll = SLOW_POLL_IDLE;
		}
	}
	if ((~p->irq_sources) & QM_PIRQ_DQRI)
		__poll_portal_fast(p, CONFIG_FSL_QMAN_POLL_LIMIT);
}
EXPORT_SYMBOL(qman_p_poll);

void qman_poll(void)
{
	struct qman_portal *p = get_poll_portal();
	qman_p_poll(p);
	put_poll_portal();
}
EXPORT_SYMBOL(qman_poll);

void qman_p_stop_dequeues(struct qman_portal *p)
{
	qman_stop_dequeues_ex(p);
}
EXPORT_SYMBOL(qman_p_stop_dequeues);

void qman_stop_dequeues(void)
{
	struct qman_portal *p = get_affine_portal();
	qman_p_stop_dequeues(p);
	put_affine_portal();
}
EXPORT_SYMBOL(qman_stop_dequeues);

void qman_p_start_dequeues(struct qman_portal *p)
{
	unsigned long irqflags __maybe_unused;
	PORTAL_IRQ_LOCK(p, irqflags);
	DPA_ASSERT(p->dqrr_disable_ref > 0);
	if (!(--p->dqrr_disable_ref))
		qm_dqrr_set_maxfill(&p->p, DQRR_MAXFILL);
	PORTAL_IRQ_UNLOCK(p, irqflags);
}
EXPORT_SYMBOL(qman_p_start_dequeues);

void qman_start_dequeues(void)
{
	struct qman_portal *p = get_affine_portal();
	qman_p_start_dequeues(p);
	put_affine_portal();
}
EXPORT_SYMBOL(qman_start_dequeues);

void qman_p_static_dequeue_add(struct qman_portal *p, u32 pools)
{
	unsigned long irqflags __maybe_unused;
	PORTAL_IRQ_LOCK(p, irqflags);
	pools &= p->config->public_cfg.pools;
	p->sdqcr |= pools;
	qm_dqrr_sdqcr_set(&p->p, p->sdqcr);
	PORTAL_IRQ_UNLOCK(p, irqflags);
}
EXPORT_SYMBOL(qman_p_static_dequeue_add);

void qman_static_dequeue_add(u32 pools)
{
	struct qman_portal *p = get_affine_portal();
	qman_p_static_dequeue_add(p, pools);
	put_affine_portal();
}
EXPORT_SYMBOL(qman_static_dequeue_add);

void qman_p_static_dequeue_del(struct qman_portal *p, u32 pools)
{
	unsigned long irqflags __maybe_unused;
	PORTAL_IRQ_LOCK(p, irqflags);
	pools &= p->config->public_cfg.pools;
	p->sdqcr &= ~pools;
	qm_dqrr_sdqcr_set(&p->p, p->sdqcr);
	PORTAL_IRQ_UNLOCK(p, irqflags);
}
EXPORT_SYMBOL(qman_p_static_dequeue_del);

void qman_static_dequeue_del(u32 pools)
{
	struct qman_portal *p = get_affine_portal();
	qman_p_static_dequeue_del(p, pools);
	put_affine_portal();
}
EXPORT_SYMBOL(qman_static_dequeue_del);

u32 qman_p_static_dequeue_get(struct qman_portal *p)
{
	return p->sdqcr;
}
EXPORT_SYMBOL(qman_p_static_dequeue_get);

u32 qman_static_dequeue_get(void)
{
	struct qman_portal *p = get_affine_portal();
	u32 ret = qman_p_static_dequeue_get(p);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_static_dequeue_get);

void qman_p_dca(struct qman_portal *p, struct qm_dqrr_entry *dq,
						int park_request)
{
	qm_dqrr_cdc_consume_1ptr(&p->p, dq, park_request);
}
EXPORT_SYMBOL(qman_p_dca);

void qman_dca(struct qm_dqrr_entry *dq, int park_request)
{
	struct qman_portal *p = get_affine_portal();
	qman_p_dca(p, dq, park_request);
	put_affine_portal();
}
EXPORT_SYMBOL(qman_dca);

/*******************/
/* Frame queue API */
/*******************/

static const char *mcr_result_str(u8 result)
{
	switch (result) {
	case QM_MCR_RESULT_NULL:
		return "QM_MCR_RESULT_NULL";
	case QM_MCR_RESULT_OK:
		return "QM_MCR_RESULT_OK";
	case QM_MCR_RESULT_ERR_FQID:
		return "QM_MCR_RESULT_ERR_FQID";
	case QM_MCR_RESULT_ERR_FQSTATE:
		return "QM_MCR_RESULT_ERR_FQSTATE";
	case QM_MCR_RESULT_ERR_NOTEMPTY:
		return "QM_MCR_RESULT_ERR_NOTEMPTY";
	case QM_MCR_RESULT_PENDING:
		return "QM_MCR_RESULT_PENDING";
	case QM_MCR_RESULT_ERR_BADCOMMAND:
		return "QM_MCR_RESULT_ERR_BADCOMMAND";
	}
	return "<unknown MCR result>";
}

int qman_create_fq(u32 fqid, u32 flags, struct qman_fq *fq)
{
	struct qm_fqd fqd;
	struct qm_mcr_queryfq_np np;
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;

	if (flags & QMAN_FQ_FLAG_DYNAMIC_FQID) {
		int ret = qman_alloc_fqid(&fqid);
		if (ret)
			return ret;
	}
	spin_lock_init(&fq->fqlock);
	fq->fqid = fqid;
	fq->flags = flags;
	fq->state = qman_fq_state_oos;
	fq->cgr_groupid = 0;
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	if (unlikely(find_empty_fq_table_entry(&fq->key, fq)))
		return -ENOMEM;
#endif
	if (!(flags & QMAN_FQ_FLAG_AS_IS) || (flags & QMAN_FQ_FLAG_NO_MODIFY))
		return 0;
	/* Everything else is AS_IS support */
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = qm_mc_start(&p->p);
	mcc->queryfq.fqid = cpu_to_be32(fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_QUERYFQ);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCC_VERB_QUERYFQ);
	if (mcr->result != QM_MCR_RESULT_OK) {
		pr_err("QUERYFQ failed: %s\n", mcr_result_str(mcr->result));
		goto err;
	}
	fqd = mcr->queryfq.fqd;
	hw_fqd_to_cpu(&fqd);
	mcc = qm_mc_start(&p->p);
	mcc->queryfq_np.fqid = cpu_to_be32(fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_QUERYFQ_NP);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCC_VERB_QUERYFQ_NP);
	if (mcr->result != QM_MCR_RESULT_OK) {
		pr_err("QUERYFQ_NP failed: %s\n", mcr_result_str(mcr->result));
		goto err;
	}
	np = mcr->queryfq_np;
	/* Phew, have queryfq and queryfq_np results, stitch together
	 * the FQ object from those. */
	fq->cgr_groupid = fqd.cgid;
	switch (np.state & QM_MCR_NP_STATE_MASK) {
	case QM_MCR_NP_STATE_OOS:
		break;
	case QM_MCR_NP_STATE_RETIRED:
		fq->state = qman_fq_state_retired;
		if (np.frm_cnt)
			fq_set(fq, QMAN_FQ_STATE_NE);
		break;
	case QM_MCR_NP_STATE_TEN_SCHED:
	case QM_MCR_NP_STATE_TRU_SCHED:
	case QM_MCR_NP_STATE_ACTIVE:
		fq->state = qman_fq_state_sched;
		if (np.state & QM_MCR_NP_STATE_R)
			fq_set(fq, QMAN_FQ_STATE_CHANGING);
		break;
	case QM_MCR_NP_STATE_PARKED:
		fq->state = qman_fq_state_parked;
		break;
	default:
		DPA_ASSERT(NULL == "invalid FQ state");
	}
	if (fqd.fq_ctrl & QM_FQCTRL_CGE)
		fq->state |= QMAN_FQ_STATE_CGR_EN;
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return 0;
err:
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (flags & QMAN_FQ_FLAG_DYNAMIC_FQID)
		qman_release_fqid(fqid);
	return -EIO;
}
EXPORT_SYMBOL(qman_create_fq);

void qman_destroy_fq(struct qman_fq *fq, u32 flags __maybe_unused)
{

	/* We don't need to lock the FQ as it is a pre-condition that the FQ be
	 * quiesced. Instead, run some checks. */
	switch (fq->state) {
	case qman_fq_state_parked:
		DPA_ASSERT(flags & QMAN_FQ_DESTROY_PARKED);
	case qman_fq_state_oos:
		if (fq_isset(fq, QMAN_FQ_FLAG_DYNAMIC_FQID))
			qman_release_fqid(fq->fqid);
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
		clear_fq_table_entry(fq->key);
#endif
		return;
	default:
		break;
	}
	DPA_ASSERT(NULL == "qman_free_fq() on unquiesced FQ!");
}
EXPORT_SYMBOL(qman_destroy_fq);

u32 qman_fq_fqid(struct qman_fq *fq)
{
	return fq->fqid;
}
EXPORT_SYMBOL(qman_fq_fqid);

void qman_fq_state(struct qman_fq *fq, enum qman_fq_state *state, u32 *flags)
{
	if (state)
		*state = fq->state;
	if (flags)
		*flags = fq->flags;
}
EXPORT_SYMBOL(qman_fq_state);

int qman_init_fq(struct qman_fq *fq, u32 flags, struct qm_mcc_initfq *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res, myverb = (flags & QMAN_INITFQ_FLAG_SCHED) ?
		QM_MCC_VERB_INITFQ_SCHED : QM_MCC_VERB_INITFQ_PARKED;

	if ((fq->state != qman_fq_state_oos) &&
			(fq->state != qman_fq_state_parked))
		return -EINVAL;
#ifdef CONFIG_FSL_DPA_CHECKING
	if (unlikely(fq_isset(fq, QMAN_FQ_FLAG_NO_MODIFY)))
		return -EINVAL;
#endif
	if (opts && (opts->we_mask & QM_INITFQ_WE_OAC)) {
		/* And can't be set at the same time as TDTHRESH */
		if (opts->we_mask & QM_INITFQ_WE_TDTHRESH)
			return -EINVAL;
	}
	/* Issue an INITFQ_[PARKED|SCHED] management command */
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	FQLOCK(fq);
	if (unlikely((fq_isset(fq, QMAN_FQ_STATE_CHANGING)) ||
			((fq->state != qman_fq_state_oos) &&
				(fq->state != qman_fq_state_parked)))) {
		FQUNLOCK(fq);
		PORTAL_IRQ_UNLOCK(p, irqflags);
		put_affine_portal();
		return -EBUSY;
	}
	mcc = qm_mc_start(&p->p);
	if (opts)
		mcc->initfq = *opts;
	mcc->initfq.fqid = cpu_to_be32(fq->fqid);
	mcc->initfq.count = 0;

	/* If the FQ does *not* have the TO_DCPORTAL flag, contextB is set as a
	 * demux pointer. Otherwise, the caller-provided value is allowed to
	 * stand, don't overwrite it. */
	if (fq_isclear(fq, QMAN_FQ_FLAG_TO_DCPORTAL)) {
		dma_addr_t phys_fq;
		mcc->initfq.we_mask |= QM_INITFQ_WE_CONTEXTB;
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
		mcc->initfq.fqd.context_b = fq->key;
#else
		mcc->initfq.fqd.context_b = (u32)(uintptr_t)fq;
#endif
		/* and the physical address - NB, if the user wasn't trying to
		 * set CONTEXTA, clear the stashing settings. */
		if (!(mcc->initfq.we_mask & QM_INITFQ_WE_CONTEXTA)) {
			mcc->initfq.we_mask |= QM_INITFQ_WE_CONTEXTA;
			memset(&mcc->initfq.fqd.context_a, 0,
				sizeof(mcc->initfq.fqd.context_a));
		} else {
			phys_fq = dma_map_single(&p->pdev->dev, fq, sizeof(*fq),
						DMA_TO_DEVICE);
			if (dma_mapping_error(&p->pdev->dev, phys_fq)) {
				dev_err(&p->pdev->dev,
					"dma_map_single failed for fqid: %u\n",
					fq->fqid);
				FQUNLOCK(fq);
				PORTAL_IRQ_UNLOCK(p, irqflags);
				put_affine_portal();
				return -EIO;
			}

			qm_fqd_stashing_set64(&mcc->initfq.fqd, phys_fq);
		}
	}
	if (flags & QMAN_INITFQ_FLAG_LOCAL) {
		mcc->initfq.fqd.dest.channel = p->config->public_cfg.channel;
		if (!(mcc->initfq.we_mask & QM_INITFQ_WE_DESTWQ)) {
			mcc->initfq.we_mask |= QM_INITFQ_WE_DESTWQ;
			mcc->initfq.fqd.dest.wq = 4;
		}
	}
	mcc->initfq.we_mask = cpu_to_be16(mcc->initfq.we_mask);
	cpu_to_hw_fqd(&mcc->initfq.fqd);
	qm_mc_commit(&p->p, myverb);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == myverb);
	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		FQUNLOCK(fq);
		PORTAL_IRQ_UNLOCK(p, irqflags);
		put_affine_portal();
		return -EIO;
	}
	if (opts) {
		if (opts->we_mask & QM_INITFQ_WE_FQCTRL) {
			if (opts->fqd.fq_ctrl & QM_FQCTRL_CGE)
				fq_set(fq, QMAN_FQ_STATE_CGR_EN);
			else
				fq_clear(fq, QMAN_FQ_STATE_CGR_EN);
		}
		if (opts->we_mask & QM_INITFQ_WE_CGID)
			fq->cgr_groupid = opts->fqd.cgid;
	}
	fq->state = (flags & QMAN_INITFQ_FLAG_SCHED) ?
			qman_fq_state_sched : qman_fq_state_parked;
	FQUNLOCK(fq);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return 0;
}
EXPORT_SYMBOL(qman_init_fq);

int qman_schedule_fq(struct qman_fq *fq)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	int ret = 0;
	u8 res;

	if (fq->state != qman_fq_state_parked)
		return -EINVAL;
#ifdef CONFIG_FSL_DPA_CHECKING
	if (unlikely(fq_isset(fq, QMAN_FQ_FLAG_NO_MODIFY)))
		return -EINVAL;
#endif
	/* Issue a ALTERFQ_SCHED management command */
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	FQLOCK(fq);
	if (unlikely((fq_isset(fq, QMAN_FQ_STATE_CHANGING)) ||
			(fq->state != qman_fq_state_parked))) {
		ret = -EBUSY;
		goto out;
	}
	mcc = qm_mc_start(&p->p);
	mcc->alterfq.fqid = cpu_to_be32(fq->fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_ALTER_SCHED);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_ALTER_SCHED);
	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		ret = -EIO;
		goto out;
	}
	fq->state = qman_fq_state_sched;
out:
	FQUNLOCK(fq);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_schedule_fq);

int qman_retire_fq(struct qman_fq *fq, u32 *flags)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	int rval;
	u8 res;

	if ((fq->state != qman_fq_state_parked) &&
			(fq->state != qman_fq_state_sched))
		return -EINVAL;
#ifdef CONFIG_FSL_DPA_CHECKING
	if (unlikely(fq_isset(fq, QMAN_FQ_FLAG_NO_MODIFY)))
		return -EINVAL;
#endif
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	FQLOCK(fq);
	if (unlikely((fq_isset(fq, QMAN_FQ_STATE_CHANGING)) ||
			(fq->state == qman_fq_state_retired) ||
				(fq->state == qman_fq_state_oos))) {
		rval = -EBUSY;
		goto out;
	}
	rval = table_push_fq(p, fq);
	if (rval)
		goto out;
	mcc = qm_mc_start(&p->p);
	mcc->alterfq.fqid = cpu_to_be32(fq->fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_ALTER_RETIRE);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_ALTER_RETIRE);
	res = mcr->result;
	/* "Elegant" would be to treat OK/PENDING the same way; set CHANGING,
	 * and defer the flags until FQRNI or FQRN (respectively) show up. But
	 * "Friendly" is to process OK immediately, and not set CHANGING. We do
	 * friendly, otherwise the caller doesn't necessarily have a fully
	 * "retired" FQ on return even if the retirement was immediate. However
	 * this does mean some code duplication between here and
	 * fq_state_change(). */
	if (likely(res == QM_MCR_RESULT_OK)) {
		rval = 0;
		/* Process 'fq' right away, we'll ignore FQRNI */
		if (mcr->alterfq.fqs & QM_MCR_FQS_NOTEMPTY)
			fq_set(fq, QMAN_FQ_STATE_NE);
		if (mcr->alterfq.fqs & QM_MCR_FQS_ORLPRESENT)
			fq_set(fq, QMAN_FQ_STATE_ORL);
		else
			table_del_fq(p, fq);
		if (flags)
			*flags = fq->flags;
		fq->state = qman_fq_state_retired;
		if (fq->cb.fqs) {
			/* Another issue with supporting "immediate" retirement
			 * is that we're forced to drop FQRNIs, because by the
			 * time they're seen it may already be "too late" (the
			 * fq may have been OOS'd and free()'d already). But if
			 * the upper layer wants a callback whether it's
			 * immediate or not, we have to fake a "MR" entry to
			 * look like an FQRNI... */
			struct qm_mr_entry msg;
			msg.verb = QM_MR_VERB_FQRNI;
			msg.fq.fqs = mcr->alterfq.fqs;
			msg.fq.fqid = fq->fqid;
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
			msg.fq.contextB = fq->key;
#else
			msg.fq.contextB = (u32)(uintptr_t)fq;
#endif
			fq->cb.fqs(p, fq, &msg);
		}
	} else if (res == QM_MCR_RESULT_PENDING) {
		rval = 1;
		fq_set(fq, QMAN_FQ_STATE_CHANGING);
	} else {
		rval = -EIO;
		table_del_fq(p, fq);
	}
out:
	FQUNLOCK(fq);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return rval;
}
EXPORT_SYMBOL(qman_retire_fq);

int qman_oos_fq(struct qman_fq *fq)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	int ret = 0;
	u8 res;

	if (fq->state != qman_fq_state_retired)
		return -EINVAL;
#ifdef CONFIG_FSL_DPA_CHECKING
	if (unlikely(fq_isset(fq, QMAN_FQ_FLAG_NO_MODIFY)))
		return -EINVAL;
#endif
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	FQLOCK(fq);
	if (unlikely((fq_isset(fq, QMAN_FQ_STATE_BLOCKOOS)) ||
			(fq->state != qman_fq_state_retired))) {
		ret = -EBUSY;
		goto out;
	}
	mcc = qm_mc_start(&p->p);
	mcc->alterfq.fqid = cpu_to_be32(fq->fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_ALTER_OOS);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_ALTER_OOS);
	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		ret = -EIO;
		goto out;
	}
	fq->state = qman_fq_state_oos;
out:
	FQUNLOCK(fq);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_oos_fq);

int qman_fq_flow_control(struct qman_fq *fq, int xon)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	int ret = 0;
	u8 res;
	u8 myverb;

	if ((fq->state == qman_fq_state_oos) ||
		(fq->state == qman_fq_state_retired) ||
		(fq->state == qman_fq_state_parked))
		return -EINVAL;

#ifdef CONFIG_FSL_DPA_CHECKING
	if (unlikely(fq_isset(fq, QMAN_FQ_FLAG_NO_MODIFY)))
		return -EINVAL;
#endif
	/* Issue a ALTER_FQXON or ALTER_FQXOFF management command */
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	FQLOCK(fq);
	if (unlikely((fq_isset(fq, QMAN_FQ_STATE_CHANGING)) ||
			(fq->state == qman_fq_state_parked) ||
			(fq->state == qman_fq_state_oos) ||
			(fq->state == qman_fq_state_retired))) {
		ret = -EBUSY;
		goto out;
	}
	mcc = qm_mc_start(&p->p);
	mcc->alterfq.fqid = fq->fqid;
	mcc->alterfq.count = 0;
	myverb = xon ? QM_MCC_VERB_ALTER_FQXON : QM_MCC_VERB_ALTER_FQXOFF;

	qm_mc_commit(&p->p, myverb);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == myverb);

	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		ret = -EIO;
		goto out;
	}
out:
	FQUNLOCK(fq);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_fq_flow_control);

int qman_query_fq(struct qman_fq *fq, struct qm_fqd *fqd)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;

	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = qm_mc_start(&p->p);
	mcc->queryfq.fqid = cpu_to_be32(fq->fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_QUERYFQ);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_QUERYFQ);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK)
		*fqd = mcr->queryfq.fqd;
	hw_fqd_to_cpu(fqd);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK)
		return -EIO;
	return 0;
}
EXPORT_SYMBOL(qman_query_fq);

int qman_query_fq_np(struct qman_fq *fq, struct qm_mcr_queryfq_np *np)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;

	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = qm_mc_start(&p->p);
	mcc->queryfq.fqid = cpu_to_be32(fq->fqid);
	qm_mc_commit(&p->p, QM_MCC_VERB_QUERYFQ_NP);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_QUERYFQ_NP);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK) {
		*np = mcr->queryfq_np;
		np->fqd_link = be24_to_cpu(np->fqd_link);
		np->odp_seq = be16_to_cpu(np->odp_seq);
		np->orp_nesn = be16_to_cpu(np->orp_nesn);
		np->orp_ea_hseq  = be16_to_cpu(np->orp_ea_hseq);
		np->orp_ea_tseq  = be16_to_cpu(np->orp_ea_tseq);
		np->orp_ea_hptr = be24_to_cpu(np->orp_ea_hptr);
		np->orp_ea_tptr = be24_to_cpu(np->orp_ea_tptr);
		np->pfdr_hptr = be24_to_cpu(np->pfdr_hptr);
		np->pfdr_tptr = be24_to_cpu(np->pfdr_tptr);
		np->ics_surp = be16_to_cpu(np->ics_surp);
		np->byte_cnt = be32_to_cpu(np->byte_cnt);
		np->frm_cnt = be24_to_cpu(np->frm_cnt);
		np->ra1_sfdr = be16_to_cpu(np->ra1_sfdr);
		np->ra2_sfdr = be16_to_cpu(np->ra2_sfdr);
		np->od1_sfdr = be16_to_cpu(np->od1_sfdr);
		np->od2_sfdr = be16_to_cpu(np->od2_sfdr);
		np->od3_sfdr = be16_to_cpu(np->od3_sfdr);
	}
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res == QM_MCR_RESULT_ERR_FQID)
		return -ERANGE;
	else if (res != QM_MCR_RESULT_OK)
		return -EIO;
	return 0;
}
EXPORT_SYMBOL(qman_query_fq_np);

int qman_query_wq(u8 query_dedicated, struct qm_mcr_querywq *wq)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res, myverb;

	PORTAL_IRQ_LOCK(p, irqflags);
	myverb = (query_dedicated) ? QM_MCR_VERB_QUERYWQ_DEDICATED :
				 QM_MCR_VERB_QUERYWQ;
	mcc = qm_mc_start(&p->p);
	mcc->querywq.channel.id = cpu_to_be16(wq->channel.id);
	qm_mc_commit(&p->p, myverb);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == myverb);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK) {
		int i, array_len;
		wq->channel.id = be16_to_cpu(mcr->querywq.channel.id);
		array_len = ARRAY_SIZE(mcr->querywq.wq_len);
		for (i = 0; i < array_len; i++)
			wq->wq_len[i] = be32_to_cpu(mcr->querywq.wq_len[i]);
	}
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK) {
		pr_err("QUERYWQ failed: %s\n", mcr_result_str(res));
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(qman_query_wq);

int qman_testwrite_cgr(struct qman_cgr *cgr, u64 i_bcnt,
			struct qm_mcr_cgrtestwrite *result)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;

	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = qm_mc_start(&p->p);
	mcc->cgrtestwrite.cgid = cgr->cgrid;
	mcc->cgrtestwrite.i_bcnt_hi = (u8)(i_bcnt >> 32);
	mcc->cgrtestwrite.i_bcnt_lo = (u32)i_bcnt;
	qm_mc_commit(&p->p, QM_MCC_VERB_CGRTESTWRITE);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCC_VERB_CGRTESTWRITE);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK)
		*result = mcr->cgrtestwrite;
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CGR TEST WRITE failed: %s\n", mcr_result_str(res));
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(qman_testwrite_cgr);

int qman_query_cgr(struct qman_cgr *cgr, struct qm_mcr_querycgr *cgrd)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;
	int i;

	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = qm_mc_start(&p->p);
	mcc->querycgr.cgid = cgr->cgrid;
	qm_mc_commit(&p->p, QM_MCC_VERB_QUERYCGR);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCC_VERB_QUERYCGR);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK)
		*cgrd = mcr->querycgr;
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK) {
		pr_err("QUERY_CGR failed: %s\n", mcr_result_str(res));
		return -EIO;
	}
	cgrd->cgr.wr_parm_g.word =
		be32_to_cpu(cgrd->cgr.wr_parm_g.word);
	cgrd->cgr.wr_parm_y.word =
		be32_to_cpu(cgrd->cgr.wr_parm_y.word);
	cgrd->cgr.wr_parm_r.word =
		be32_to_cpu(cgrd->cgr.wr_parm_r.word);
	cgrd->cgr.cscn_targ =  be32_to_cpu(cgrd->cgr.cscn_targ);
	cgrd->cgr.__cs_thres = be16_to_cpu(cgrd->cgr.__cs_thres);
	for (i = 0; i < ARRAY_SIZE(cgrd->cscn_targ_swp); i++)
			be32_to_cpus(&cgrd->cscn_targ_swp[i]);
	return 0;
}
EXPORT_SYMBOL(qman_query_cgr);

int qman_query_congestion(struct qm_mcr_querycongestion *congestion)
{
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;
	int i;

	PORTAL_IRQ_LOCK(p, irqflags);
	qm_mc_start(&p->p);
	qm_mc_commit(&p->p, QM_MCC_VERB_QUERYCONGESTION);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
			QM_MCC_VERB_QUERYCONGESTION);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK)
		memcpy_fromio(congestion, &mcr->querycongestion,
			      sizeof(*congestion));
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK) {
		pr_err("QUERY_CONGESTION failed: %s\n", mcr_result_str(res));
		return -EIO;
	}

	for (i = 0; i < ARRAY_SIZE(congestion->state.__state); i++)
			be32_to_cpus(&congestion->state.__state[i]);
	return 0;
}
EXPORT_SYMBOL(qman_query_congestion);

/* internal function used as a wait_event() expression */
static int set_p_vdqcr(struct qman_portal *p, struct qman_fq *fq, u32 vdqcr)
{
	unsigned long irqflags __maybe_unused;
	int ret = -EBUSY;
	PORTAL_IRQ_LOCK(p, irqflags);
	if (!p->vdqcr_owned) {
		FQLOCK(fq);
		if (fq_isset(fq, QMAN_FQ_STATE_VDQCR))
			goto escape;
		fq_set(fq, QMAN_FQ_STATE_VDQCR);
		FQUNLOCK(fq);
		p->vdqcr_owned = fq;
		ret = 0;
	}
escape:
	PORTAL_IRQ_UNLOCK(p, irqflags);
	if (!ret)
		qm_dqrr_vdqcr_set(&p->p, vdqcr);
	return ret;
}

static int set_vdqcr(struct qman_portal **p, struct qman_fq *fq, u32 vdqcr)
{
	int ret;
	*p = get_affine_portal();
	ret = set_p_vdqcr(*p, fq, vdqcr);
	put_affine_portal();
	return ret;
}

#ifdef CONFIG_FSL_DPA_CAN_WAIT
static int wait_p_vdqcr_start(struct qman_portal *p, struct qman_fq *fq,
				u32 vdqcr, u32 flags)
{
	int ret = 0;
	if (flags & QMAN_VOLATILE_FLAG_WAIT_INT)
		ret = wait_event_interruptible(affine_queue,
				!(ret = set_p_vdqcr(p, fq, vdqcr)));
	else
		wait_event(affine_queue, !(ret = set_p_vdqcr(p, fq, vdqcr)));
	return ret;
}

static int wait_vdqcr_start(struct qman_portal **p, struct qman_fq *fq,
				u32 vdqcr, u32 flags)
{
	int ret = 0;
	if (flags & QMAN_VOLATILE_FLAG_WAIT_INT)
		ret = wait_event_interruptible(affine_queue,
				!(ret = set_vdqcr(p, fq, vdqcr)));
	else
		wait_event(affine_queue, !(ret = set_vdqcr(p, fq, vdqcr)));
	return ret;
}
#endif

int qman_p_volatile_dequeue(struct qman_portal *p, struct qman_fq *fq,
					u32 flags __maybe_unused, u32 vdqcr)
{
	int ret;

	if ((fq->state != qman_fq_state_parked) &&
			(fq->state != qman_fq_state_retired))
		return -EINVAL;
	if (vdqcr & QM_VDQCR_FQID_MASK)
		return -EINVAL;
	if (fq_isset(fq, QMAN_FQ_STATE_VDQCR))
		return -EBUSY;
	vdqcr = (vdqcr & ~QM_VDQCR_FQID_MASK) | fq->fqid;
#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_VOLATILE_FLAG_WAIT)
		ret = wait_p_vdqcr_start(p, fq, vdqcr, flags);
	else
#endif
		ret = set_p_vdqcr(p, fq, vdqcr);
	if (ret)
		return ret;
	/* VDQCR is set */
#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_VOLATILE_FLAG_FINISH) {
		if (flags & QMAN_VOLATILE_FLAG_WAIT_INT)
			/* NB: don't propagate any error - the caller wouldn't
			 * know whether the VDQCR was issued or not. A signal
			 * could arrive after returning anyway, so the caller
			 * can check signal_pending() if that's an issue. */
			wait_event_interruptible(affine_queue,
				!fq_isset(fq, QMAN_FQ_STATE_VDQCR));
		else
			wait_event(affine_queue,
				!fq_isset(fq, QMAN_FQ_STATE_VDQCR));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_p_volatile_dequeue);

int qman_volatile_dequeue(struct qman_fq *fq, u32 flags __maybe_unused,
				u32 vdqcr)
{
	struct qman_portal *p;
	int ret;

	if ((fq->state != qman_fq_state_parked) &&
			(fq->state != qman_fq_state_retired))
		return -EINVAL;
	if (vdqcr & QM_VDQCR_FQID_MASK)
		return -EINVAL;
	if (fq_isset(fq, QMAN_FQ_STATE_VDQCR))
		return -EBUSY;
	vdqcr = (vdqcr & ~QM_VDQCR_FQID_MASK) | fq->fqid;
#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_VOLATILE_FLAG_WAIT)
		ret = wait_vdqcr_start(&p, fq, vdqcr, flags);
	else
#endif
		ret = set_vdqcr(&p, fq, vdqcr);
	if (ret)
		return ret;
	/* VDQCR is set */
#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_VOLATILE_FLAG_FINISH) {
		if (flags & QMAN_VOLATILE_FLAG_WAIT_INT)
			/* NB: don't propagate any error - the caller wouldn't
			 * know whether the VDQCR was issued or not. A signal
			 * could arrive after returning anyway, so the caller
			 * can check signal_pending() if that's an issue. */
			wait_event_interruptible(affine_queue,
				!fq_isset(fq, QMAN_FQ_STATE_VDQCR));
		else
			wait_event(affine_queue,
				!fq_isset(fq, QMAN_FQ_STATE_VDQCR));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_volatile_dequeue);

static noinline void update_eqcr_ci(struct qman_portal *p, u8 avail)
{
	if (avail)
		qm_eqcr_cce_prefetch(&p->p);
	else
		qm_eqcr_cce_update(&p->p);
}

int qman_eqcr_is_empty(void)
{
	unsigned long irqflags __maybe_unused;
	struct qman_portal *p = get_affine_portal();
	u8 avail;

	PORTAL_IRQ_LOCK(p, irqflags);
	update_eqcr_ci(p, 0);
	avail = qm_eqcr_get_fill(&p->p);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return avail == 0;
}
EXPORT_SYMBOL(qman_eqcr_is_empty);

void qman_set_dc_ern(qman_cb_dc_ern handler, int affine)
{
	if (affine) {
		unsigned long irqflags __maybe_unused;
		struct qman_portal *p = get_affine_portal();
		PORTAL_IRQ_LOCK(p, irqflags);
		p->cb_dc_ern = handler;
		PORTAL_IRQ_UNLOCK(p, irqflags);
		put_affine_portal();
	} else
		cb_dc_ern = handler;
}
EXPORT_SYMBOL(qman_set_dc_ern);

static inline struct qm_eqcr_entry *try_p_eq_start(struct qman_portal *p,
					unsigned long *irqflags __maybe_unused,
					struct qman_fq *fq,
					const struct qm_fd *fd,
					u32 flags)
{
	struct qm_eqcr_entry *eq;
	u8 avail;
	PORTAL_IRQ_LOCK(p, (*irqflags));
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (p->eqci_owned) {
			PORTAL_IRQ_UNLOCK(p, (*irqflags));
			return NULL;
		}
		p->eqci_owned = fq;
	}
#endif
	if (p->use_eqcr_ci_stashing) {
		/*
		 * The stashing case is easy, only update if we need to in
		 * order to try and liberate ring entries.
		 */
		eq = qm_eqcr_start_stash(&p->p);
	} else {
		/*
		 * The non-stashing case is harder, need to prefetch ahead of
		 * time.
		 */
		avail = qm_eqcr_get_avail(&p->p);
		if (avail < 2)
			update_eqcr_ci(p, avail);
		eq = qm_eqcr_start_no_stash(&p->p);
	}

	if (unlikely(!eq)) {
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
		if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
				(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC)))
			p->eqci_owned = NULL;
#endif
		PORTAL_IRQ_UNLOCK(p, (*irqflags));
		return NULL;
	}
	if (flags & QMAN_ENQUEUE_FLAG_DCA)
		eq->dca = QM_EQCR_DCA_ENABLE |
			((flags & QMAN_ENQUEUE_FLAG_DCA_PARK) ?
					QM_EQCR_DCA_PARK : 0) |
			((flags >> 8) & QM_EQCR_DCA_IDXMASK);
	eq->fqid = cpu_to_be32(fq->fqid);
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	eq->tag = cpu_to_be32(fq->key);
#else
	eq->tag = cpu_to_be32((u32)(uintptr_t)fq);
#endif
	eq->fd = *fd;
	cpu_to_hw_fd(&eq->fd);
	return eq;
}

static inline struct qm_eqcr_entry *try_eq_start(struct qman_portal **p,
					unsigned long *irqflags __maybe_unused,
					struct qman_fq *fq,
					const struct qm_fd *fd,
					u32 flags)
{
	struct qm_eqcr_entry *eq;
	*p = get_affine_portal();
	eq = try_p_eq_start(*p, irqflags, fq, fd, flags);
	if (!eq)
		put_affine_portal();
	return eq;
}

#ifdef CONFIG_FSL_DPA_CAN_WAIT
static noinline struct qm_eqcr_entry *__wait_eq_start(struct qman_portal **p,
					unsigned long *irqflags __maybe_unused,
					struct qman_fq *fq,
					const struct qm_fd *fd,
					u32 flags)
{
	struct qm_eqcr_entry *eq = try_eq_start(p, irqflags, fq, fd, flags);
	if (!eq)
		qm_eqcr_set_ithresh(&(*p)->p, EQCR_ITHRESH);
	return eq;
}
static noinline struct qm_eqcr_entry *wait_eq_start(struct qman_portal **p,
					unsigned long *irqflags __maybe_unused,
					struct qman_fq *fq,
					const struct qm_fd *fd,
					u32 flags)
{
	struct qm_eqcr_entry *eq;
	if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
		/* NB: return NULL if signal occurs before completion. Signal
		 * can occur during return. Caller must check for signal */
		wait_event_interruptible(affine_queue,
			(eq = __wait_eq_start(p, irqflags, fq, fd, flags)));
	else
		wait_event(affine_queue,
			(eq = __wait_eq_start(p, irqflags, fq, fd, flags)));
	return eq;
}
static noinline struct qm_eqcr_entry *__wait_p_eq_start(struct qman_portal *p,
					unsigned long *irqflags __maybe_unused,
					struct qman_fq *fq,
					const struct qm_fd *fd,
					u32 flags)
{
	struct qm_eqcr_entry *eq = try_p_eq_start(p, irqflags, fq, fd, flags);
	if (!eq)
		qm_eqcr_set_ithresh(&p->p, EQCR_ITHRESH);
	return eq;
}
static noinline struct qm_eqcr_entry *wait_p_eq_start(struct qman_portal *p,
					unsigned long *irqflags __maybe_unused,
					struct qman_fq *fq,
					const struct qm_fd *fd,
					u32 flags)
{
	struct qm_eqcr_entry *eq;
	if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
		/* NB: return NULL if signal occurs before completion. Signal
		 * can occur during return. Caller must check for signal */
		wait_event_interruptible(affine_queue,
			(eq = __wait_p_eq_start(p, irqflags, fq, fd, flags)));
	else
		wait_event(affine_queue,
			(eq = __wait_p_eq_start(p, irqflags, fq, fd, flags)));
	return eq;
}
#endif

int qman_p_enqueue(struct qman_portal *p, struct qman_fq *fq,
				const struct qm_fd *fd, u32 flags)
{
	struct qm_eqcr_entry *eq;
	unsigned long irqflags __maybe_unused;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_ENQUEUE_FLAG_WAIT)
		eq = wait_p_eq_start(p, &irqflags, fq, fd, flags);
	else
#endif
	eq = try_p_eq_start(p, &irqflags, fq, fd, flags);
	if (!eq)
		return -EBUSY;
	/* Note: QM_EQCR_VERB_INTERRUPT == QMAN_ENQUEUE_FLAG_WAIT_SYNC */
	qm_eqcr_pvb_commit(&p->p, QM_EQCR_VERB_CMD_ENQUEUE |
		(flags & (QM_EQCR_VERB_COLOUR_MASK | QM_EQCR_VERB_INTERRUPT)));
	/* Factor the below out, it's used from qman_enqueue_orp() too */
	PORTAL_IRQ_UNLOCK(p, irqflags);
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->eqci_owned != fq));
		else
			wait_event(affine_queue, (p->eqci_owned != fq));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_p_enqueue);

int qman_enqueue(struct qman_fq *fq, const struct qm_fd *fd, u32 flags)
{
	struct qman_portal *p;
	struct qm_eqcr_entry *eq;
	unsigned long irqflags __maybe_unused;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_ENQUEUE_FLAG_WAIT)
		eq = wait_eq_start(&p, &irqflags, fq, fd, flags);
	else
#endif
	eq = try_eq_start(&p, &irqflags, fq, fd, flags);
	if (!eq)
		return -EBUSY;
	/* Note: QM_EQCR_VERB_INTERRUPT == QMAN_ENQUEUE_FLAG_WAIT_SYNC */
	qm_eqcr_pvb_commit(&p->p, QM_EQCR_VERB_CMD_ENQUEUE |
		(flags & (QM_EQCR_VERB_COLOUR_MASK | QM_EQCR_VERB_INTERRUPT)));
	/* Factor the below out, it's used from qman_enqueue_orp() too */
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->eqci_owned != fq));
		else
			wait_event(affine_queue, (p->eqci_owned != fq));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_enqueue);

int qman_p_enqueue_orp(struct qman_portal *p, struct qman_fq *fq,
				const struct qm_fd *fd, u32 flags,
				struct qman_fq *orp, u16 orp_seqnum)
{
	struct qm_eqcr_entry *eq;
	unsigned long irqflags __maybe_unused;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_ENQUEUE_FLAG_WAIT)
		eq = wait_p_eq_start(p, &irqflags, fq, fd, flags);
	else
#endif
	eq = try_p_eq_start(p, &irqflags, fq, fd, flags);
	if (!eq)
		return -EBUSY;
	/* Process ORP-specifics here */
	if (flags & QMAN_ENQUEUE_FLAG_NLIS)
		orp_seqnum |= QM_EQCR_SEQNUM_NLIS;
	else {
		orp_seqnum &= ~QM_EQCR_SEQNUM_NLIS;
		if (flags & QMAN_ENQUEUE_FLAG_NESN)
			orp_seqnum |= QM_EQCR_SEQNUM_NESN;
		else
			/* No need to check 4 QMAN_ENQUEUE_FLAG_HOLE */
			orp_seqnum &= ~QM_EQCR_SEQNUM_NESN;
	}
	eq->seqnum = cpu_to_be16(orp_seqnum);
	eq->orp = cpu_to_be32(orp->fqid);
	/* Note: QM_EQCR_VERB_INTERRUPT == QMAN_ENQUEUE_FLAG_WAIT_SYNC */
	qm_eqcr_pvb_commit(&p->p, QM_EQCR_VERB_ORP |
		((flags & (QMAN_ENQUEUE_FLAG_HOLE | QMAN_ENQUEUE_FLAG_NESN)) ?
				0 : QM_EQCR_VERB_CMD_ENQUEUE) |
		(flags & (QM_EQCR_VERB_COLOUR_MASK | QM_EQCR_VERB_INTERRUPT)));
	PORTAL_IRQ_UNLOCK(p, irqflags);
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->eqci_owned != fq));
		else
			wait_event(affine_queue, (p->eqci_owned != fq));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_p_enqueue_orp);

int qman_enqueue_orp(struct qman_fq *fq, const struct qm_fd *fd, u32 flags,
			struct qman_fq *orp, u16 orp_seqnum)
{
	struct qman_portal *p;
	struct qm_eqcr_entry *eq;
	unsigned long irqflags __maybe_unused;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_ENQUEUE_FLAG_WAIT)
		eq = wait_eq_start(&p, &irqflags, fq, fd, flags);
	else
#endif
	eq = try_eq_start(&p, &irqflags, fq, fd, flags);
	if (!eq)
		return -EBUSY;
	/* Process ORP-specifics here */
	if (flags & QMAN_ENQUEUE_FLAG_NLIS)
		orp_seqnum |= QM_EQCR_SEQNUM_NLIS;
	else {
		orp_seqnum &= ~QM_EQCR_SEQNUM_NLIS;
		if (flags & QMAN_ENQUEUE_FLAG_NESN)
			orp_seqnum |= QM_EQCR_SEQNUM_NESN;
		else
			/* No need to check 4 QMAN_ENQUEUE_FLAG_HOLE */
			orp_seqnum &= ~QM_EQCR_SEQNUM_NESN;
	}
	eq->seqnum = cpu_to_be16(orp_seqnum);
	eq->orp = cpu_to_be32(orp->fqid);
	/* Note: QM_EQCR_VERB_INTERRUPT == QMAN_ENQUEUE_FLAG_WAIT_SYNC */
	qm_eqcr_pvb_commit(&p->p, QM_EQCR_VERB_ORP |
		((flags & (QMAN_ENQUEUE_FLAG_HOLE | QMAN_ENQUEUE_FLAG_NESN)) ?
				0 : QM_EQCR_VERB_CMD_ENQUEUE) |
		(flags & (QM_EQCR_VERB_COLOUR_MASK | QM_EQCR_VERB_INTERRUPT)));
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->eqci_owned != fq));
		else
			wait_event(affine_queue, (p->eqci_owned != fq));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_enqueue_orp);

int qman_p_enqueue_precommit(struct qman_portal *p, struct qman_fq *fq,
				const struct qm_fd *fd, u32 flags,
				qman_cb_precommit cb, void *cb_arg)
{
	struct qm_eqcr_entry *eq;
	unsigned long irqflags __maybe_unused;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_ENQUEUE_FLAG_WAIT)
		eq = wait_p_eq_start(p, &irqflags, fq, fd, flags);
	else
#endif
	eq = try_p_eq_start(p, &irqflags, fq, fd, flags);
	if (!eq)
		return -EBUSY;
	/* invoke user supplied callback function before writing commit verb */
	if (cb(cb_arg)) {
		PORTAL_IRQ_UNLOCK(p, irqflags);
		return -EINVAL;
	}
	/* Note: QM_EQCR_VERB_INTERRUPT == QMAN_ENQUEUE_FLAG_WAIT_SYNC */
	qm_eqcr_pvb_commit(&p->p, QM_EQCR_VERB_CMD_ENQUEUE |
		(flags & (QM_EQCR_VERB_COLOUR_MASK | QM_EQCR_VERB_INTERRUPT)));
	/* Factor the below out, it's used from qman_enqueue_orp() too */
	PORTAL_IRQ_UNLOCK(p, irqflags);
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->eqci_owned != fq));
		else
			wait_event(affine_queue, (p->eqci_owned != fq));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_p_enqueue_precommit);

int qman_enqueue_precommit(struct qman_fq *fq, const struct qm_fd *fd,
		u32 flags, qman_cb_precommit cb, void *cb_arg)
{
	struct qman_portal *p;
	struct qm_eqcr_entry *eq;
	unsigned long irqflags __maybe_unused;

#ifdef CONFIG_FSL_DPA_CAN_WAIT
	if (flags & QMAN_ENQUEUE_FLAG_WAIT)
		eq = wait_eq_start(&p, &irqflags, fq, fd, flags);
	else
#endif
	eq = try_eq_start(&p, &irqflags, fq, fd, flags);
	if (!eq)
		return -EBUSY;
	/* invoke user supplied callback function before writing commit verb */
	if (cb(cb_arg)) {
		PORTAL_IRQ_UNLOCK(p, irqflags);
		put_affine_portal();
		return -EINVAL;
	}
	/* Note: QM_EQCR_VERB_INTERRUPT == QMAN_ENQUEUE_FLAG_WAIT_SYNC */
	qm_eqcr_pvb_commit(&p->p, QM_EQCR_VERB_CMD_ENQUEUE |
		(flags & (QM_EQCR_VERB_COLOUR_MASK | QM_EQCR_VERB_INTERRUPT)));
	/* Factor the below out, it's used from qman_enqueue_orp() too */
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
	if (unlikely((flags & QMAN_ENQUEUE_FLAG_WAIT) &&
			(flags & QMAN_ENQUEUE_FLAG_WAIT_SYNC))) {
		if (flags & QMAN_ENQUEUE_FLAG_WAIT_INT)
			/* NB: return success even if signal occurs before
			 * condition is true. pvb_commit guarantees success */
			wait_event_interruptible(affine_queue,
					(p->eqci_owned != fq));
		else
			wait_event(affine_queue, (p->eqci_owned != fq));
	}
#endif
	return 0;
}
EXPORT_SYMBOL(qman_enqueue_precommit);

int qman_modify_cgr(struct qman_cgr *cgr, u32 flags,
			struct qm_mcc_initcgr *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;
	u8 verb = QM_MCC_VERB_MODIFYCGR;

	PORTAL_IRQ_LOCK(p, irqflags);
	mcc = qm_mc_start(&p->p);
	if (opts)
		mcc->initcgr = *opts;
	mcc->initcgr.we_mask = cpu_to_be16(mcc->initcgr.we_mask);
	mcc->initcgr.cgr.wr_parm_g.word =
		cpu_to_be32(mcc->initcgr.cgr.wr_parm_g.word);
	mcc->initcgr.cgr.wr_parm_y.word =
		cpu_to_be32(mcc->initcgr.cgr.wr_parm_y.word);
	mcc->initcgr.cgr.wr_parm_r.word =
		cpu_to_be32(mcc->initcgr.cgr.wr_parm_r.word);
	mcc->initcgr.cgr.cscn_targ =  cpu_to_be32(mcc->initcgr.cgr.cscn_targ);
	mcc->initcgr.cgr.__cs_thres = cpu_to_be16(mcc->initcgr.cgr.__cs_thres);

	mcc->initcgr.cgid = cgr->cgrid;
	if (flags & QMAN_CGR_FLAG_USE_INIT)
		verb = QM_MCC_VERB_INITCGR;
	qm_mc_commit(&p->p, verb);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == verb);
	res = mcr->result;
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return (res == QM_MCR_RESULT_OK) ? 0 : -EIO;
}
EXPORT_SYMBOL(qman_modify_cgr);

#define TARG_MASK(n) (0x80000000 >> (n->config->public_cfg.channel - \
					QM_CHANNEL_SWPORTAL0))
#define TARG_DCP_MASK(n) (0x80000000 >> (10 + n))
#define PORTAL_IDX(n) (n->config->public_cfg.channel - QM_CHANNEL_SWPORTAL0)

static u8 qman_cgr_cpus[__CGR_NUM];

int qman_create_cgr(struct qman_cgr *cgr, u32 flags,
			struct qm_mcc_initcgr *opts)
{
	unsigned long irqflags __maybe_unused;
	struct qm_mcr_querycgr cgr_state;
	struct qm_mcc_initcgr local_opts;
	int ret;
	struct qman_portal *p;

	/* We have to check that the provided CGRID is within the limits of the
	 * data-structures, for obvious reasons. However we'll let h/w take
	 * care of determining whether it's within the limits of what exists on
	 * the SoC. */
	if (cgr->cgrid >= __CGR_NUM)
		return -EINVAL;

	preempt_disable();
	p = get_affine_portal();
	qman_cgr_cpus[cgr->cgrid] = smp_processor_id();
	preempt_enable();

	memset(&local_opts, 0, sizeof(struct qm_mcc_initcgr));
	cgr->chan = p->config->public_cfg.channel;
	spin_lock_irqsave(&p->cgr_lock, irqflags);

	/* if no opts specified, just add it to the list */
	if (!opts)
		goto add_list;

	ret = qman_query_cgr(cgr, &cgr_state);
	if (ret)
		goto release_lock;
	if (opts)
		local_opts = *opts;
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30)
		local_opts.cgr.cscn_targ_upd_ctrl =
			QM_CGR_TARG_UDP_CTRL_WRITE_BIT | PORTAL_IDX(p);
	else
		/* Overwrite TARG */
		local_opts.cgr.cscn_targ = cgr_state.cgr.cscn_targ |
							TARG_MASK(p);
	local_opts.we_mask |= QM_CGR_WE_CSCN_TARG;

	/* send init if flags indicate so */
	if (opts && (flags & QMAN_CGR_FLAG_USE_INIT))
		ret = qman_modify_cgr(cgr, QMAN_CGR_FLAG_USE_INIT, &local_opts);
	else
		ret = qman_modify_cgr(cgr, 0, &local_opts);
	if (ret)
		goto release_lock;
add_list:
	list_add(&cgr->node, &p->cgr_cbs);

	/* Determine if newly added object requires its callback to be called */
	ret = qman_query_cgr(cgr, &cgr_state);
	if (ret) {
		/* we can't go back, so proceed and return success, but screen
		 * and wail to the log file */
		pr_crit("CGR HW state partially modified\n");
		ret = 0;
		goto release_lock;
	}
	if (cgr->cb && cgr_state.cgr.cscn_en && qman_cgrs_get(&p->cgrs[1],
							cgr->cgrid))
		cgr->cb(p, cgr, 1);
release_lock:
	spin_unlock_irqrestore(&p->cgr_lock, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_create_cgr);

int qman_create_cgr_to_dcp(struct qman_cgr *cgr, u32 flags, u16 dcp_portal,
					struct qm_mcc_initcgr *opts)
{
	unsigned long irqflags __maybe_unused;
	struct qm_mcc_initcgr local_opts;
	struct qm_mcr_querycgr cgr_state;
	int ret;

	if ((qman_ip_rev & 0xFF00) < QMAN_REV30) {
		pr_warn("This QMan version doesn't support to send CSCN to DCP portal\n");
		return -EINVAL;
	}
	/* We have to check that the provided CGRID is within the limits of the
	 * data-structures, for obvious reasons. However we'll let h/w take
	 * care of determining whether it's within the limits of what exists on
	 * the SoC.
	 */
	if (cgr->cgrid >= __CGR_NUM)
		return -EINVAL;

	ret = qman_query_cgr(cgr, &cgr_state);
	if (ret)
		return ret;

	memset(&local_opts, 0, sizeof(struct qm_mcc_initcgr));
	if (opts)
		local_opts = *opts;

	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30)
		local_opts.cgr.cscn_targ_upd_ctrl =
				QM_CGR_TARG_UDP_CTRL_WRITE_BIT |
				QM_CGR_TARG_UDP_CTRL_DCP | dcp_portal;
	else
		local_opts.cgr.cscn_targ = cgr_state.cgr.cscn_targ |
					TARG_DCP_MASK(dcp_portal);
	local_opts.we_mask |= QM_CGR_WE_CSCN_TARG;

	/* send init if flags indicate so */
	if (opts && (flags & QMAN_CGR_FLAG_USE_INIT))
		ret = qman_modify_cgr(cgr, QMAN_CGR_FLAG_USE_INIT,
							&local_opts);
	else
		ret = qman_modify_cgr(cgr, 0, &local_opts);

	return ret;
}
EXPORT_SYMBOL(qman_create_cgr_to_dcp);

int qman_delete_cgr(struct qman_cgr *cgr)
{
	unsigned long irqflags __maybe_unused;
	struct qm_mcr_querycgr cgr_state;
	struct qm_mcc_initcgr local_opts;
	int ret = 0;
	struct qman_cgr *i;
	struct qman_portal *p = get_affine_portal();

	if (cgr->chan != p->config->public_cfg.channel) {
		pr_crit("Attempting to delete cgr from different portal "
			"than it was create: create 0x%x, delete 0x%x\n",
			cgr->chan, p->config->public_cfg.channel);
		ret = -EINVAL;
		goto put_portal;
	}
	memset(&local_opts, 0, sizeof(struct qm_mcc_initcgr));
	spin_lock_irqsave(&p->cgr_lock, irqflags);
	list_del(&cgr->node);
	/*
	 * If there are no other CGR objects for this CGRID in the list, update
	 * CSCN_TARG accordingly
	 */
	list_for_each_entry(i, &p->cgr_cbs, node)
		if ((i->cgrid == cgr->cgrid) && i->cb)
			goto release_lock;
	ret = qman_query_cgr(cgr, &cgr_state);
	if (ret)  {
		/* add back to the list */
		list_add(&cgr->node, &p->cgr_cbs);
		goto release_lock;
	}
	/* Overwrite TARG */
	local_opts.we_mask = QM_CGR_WE_CSCN_TARG;
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30)
		local_opts.cgr.cscn_targ_upd_ctrl = PORTAL_IDX(p);
	else
		local_opts.cgr.cscn_targ = cgr_state.cgr.cscn_targ &
							 ~(TARG_MASK(p));
	ret = qman_modify_cgr(cgr, 0, &local_opts);
	if (ret)
		/* add back to the list */
		list_add(&cgr->node, &p->cgr_cbs);
release_lock:
	spin_unlock_irqrestore(&p->cgr_lock, irqflags);
put_portal:
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_delete_cgr);

struct cgr_comp {
	struct qman_cgr *cgr;
	struct completion completion;
};

static void qman_delete_cgr_smp_call(void *p)
{
	qman_delete_cgr((struct qman_cgr *)p);
}

void qman_delete_cgr_safe(struct qman_cgr *cgr)
{
	preempt_disable();
	if (qman_cgr_cpus[cgr->cgrid] != smp_processor_id()) {
		smp_call_function_single(qman_cgr_cpus[cgr->cgrid],
					 qman_delete_cgr_smp_call, cgr, true);
		preempt_enable();
		return;
	}
	qman_delete_cgr(cgr);
	preempt_enable();
}
EXPORT_SYMBOL(qman_delete_cgr_safe);

int qm_get_clock(u64 *clock_hz)
{
	if (!qman_clk) {
		pr_warn("Qman clock speed is unknown\n");
		return  -EINVAL;
	}
	*clock_hz = (u64)qman_clk;
	return 0;
}
EXPORT_SYMBOL(qm_get_clock);

int qm_set_clock(u64 clock_hz)
{
	if (qman_clk)
		return -1;
	qman_clk = (u32)clock_hz;
		return 0;
}
EXPORT_SYMBOL(qm_set_clock);

/* CEETM management command */
static int qman_ceetm_configure_lfqmt(struct qm_mcc_ceetm_lfqmt_config *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->lfqmt_config = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_LFQMT_CONFIG);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
					 QM_CEETM_VERB_LFQMT_CONFIG);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CONFIGURE LFQMT failed\n");
		return -EIO;
	}
	return 0;
}

int qman_ceetm_query_lfqmt(int lfqid,
			   struct qm_mcr_ceetm_lfqmt_query *lfqmt_query)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->lfqmt_query.lfqid = lfqid;
	qm_mc_commit(&p->p, QM_CEETM_VERB_LFQMT_QUERY);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_LFQMT_QUERY);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK)
		*lfqmt_query = mcr->lfqmt_query;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: QUERY LFQMT failed\n");
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_query_lfqmt);

static int qman_ceetm_configure_cq(struct qm_mcc_ceetm_cq_config *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->cq_config = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_CQ_CONFIG);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	res = mcr->result;
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_CQ_CONFIG);

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CONFIGURE CQ failed\n");
		return -EIO;
	}
	return 0;
}

int qman_ceetm_query_cq(unsigned int cqid, unsigned int dcpid,
				struct qm_mcr_ceetm_cq_query *cq_query)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->cq_query.cqid = cpu_to_be16(cqid);
	mcc->cq_query.dcpid = dcpid;
	qm_mc_commit(&p->p, QM_CEETM_VERB_CQ_QUERY);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_CQ_QUERY);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK) {
		*cq_query = mcr->cq_query;
		hw_cq_query_to_cpu(cq_query);
	}

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: QUERY CQ failed\n");
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_query_cq);

static int qman_ceetm_configure_dct(struct qm_mcc_ceetm_dct_config *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->dct_config = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_DCT_CONFIG);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_DCT_CONFIG);
	res = mcr->result;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CONFIGURE DCT failed\n");
		return -EIO;
	}
	return 0;
}

static int qman_ceetm_query_dct(struct qm_mcc_ceetm_dct_query *opts,
			 struct qm_mcr_ceetm_dct_query *dct_query)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p = get_affine_portal();
	unsigned long irqflags __maybe_unused;
	u8 res;

	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->dct_query = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_DCT_QUERY);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_DCT_QUERY);
	res = mcr->result;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: QUERY DCT failed\n");
		return -EIO;
	}

	*dct_query = mcr->dct_query;
	return 0;
}

static int qman_ceetm_configure_class_scheduler(
			struct qm_mcc_ceetm_class_scheduler_config *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->csch_config = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
					QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG);
	res = mcr->result;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CONFIGURE CLASS SCHEDULER failed\n");
		return -EIO;
	}
	return 0;
}

static int qman_ceetm_query_class_scheduler(struct qm_ceetm_channel *channel,
			struct qm_mcr_ceetm_class_scheduler_query *query)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->csch_query.cqcid = cpu_to_be16(channel->idx);
	mcc->csch_query.dcpid = channel->dcp_idx;
	qm_mc_commit(&p->p, QM_CEETM_VERB_CLASS_SCHEDULER_QUERY);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
				QM_CEETM_VERB_CLASS_SCHEDULER_QUERY);
	res = mcr->result;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: QUERY CLASS SCHEDULER failed\n");
		return -EIO;
	}
	*query = mcr->csch_query;
	return 0;
}

static int qman_ceetm_configure_mapping_shaper_tcfc(
		struct qm_mcc_ceetm_mapping_shaper_tcfc_config *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->mst_config = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
				QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG);
	res = mcr->result;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CONFIGURE CHANNEL MAPPING failed\n");
		return -EIO;
	}
	return 0;
}

static int qman_ceetm_query_mapping_shaper_tcfc(
		struct qm_mcc_ceetm_mapping_shaper_tcfc_query *opts,
		struct qm_mcr_ceetm_mapping_shaper_tcfc_query *response)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->mst_query = *opts;
	qm_mc_commit(&p->p, QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
				QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY);
	res = mcr->result;

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: QUERY CHANNEL MAPPING failed\n");
		return -EIO;
	}

	*response = mcr->mst_query;
	return 0;
}

static int qman_ceetm_configure_ccgr(struct qm_mcc_ceetm_ccgr_config *opts)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->ccgr_config = *opts;

	qm_mc_commit(&p->p, QM_CEETM_VERB_CCGR_CONFIG);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_CCGR_CONFIG);

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CONFIGURE CCGR failed\n");
		return -EIO;
	}
	return 0;
}

int qman_ceetm_query_ccgr(struct qm_mcc_ceetm_ccgr_query *ccgr_query,
				struct qm_mcr_ceetm_ccgr_query *response)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->ccgr_query.ccgrid = cpu_to_be16(ccgr_query->ccgrid);
	mcc->ccgr_query.dcpid = ccgr_query->dcpid;
	qm_mc_commit(&p->p, QM_CEETM_VERB_CCGR_QUERY);

	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_CEETM_VERB_CCGR_QUERY);
	res = mcr->result;
	if (res == QM_MCR_RESULT_OK) {
		*response = mcr->ccgr_query;
		hw_ccgr_query_to_cpu(response);
	}

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: QUERY CCGR failed\n");
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_query_ccgr);

static int qman_ceetm_cq_peek_pop_xsfdrread(struct qm_ceetm_cq *cq,
			u8 command_type, u16 xsfdr,
			struct qm_mcr_ceetm_cq_peek_pop_xsfdrread *cq_ppxr)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	switch (command_type) {
	case 0:
	case 1:
		mcc->cq_ppxr.cqid = (cq->parent->idx << 4) | cq->idx;
		break;
	case 2:
		mcc->cq_ppxr.xsfdr = xsfdr;
		break;
	default:
		break;
	}
	mcc->cq_ppxr.ct = command_type;
	mcc->cq_ppxr.dcpid = cq->parent->dcp_idx;
	qm_mc_commit(&p->p, QM_CEETM_VERB_CQ_PEEK_POP_XFDRREAD);
	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
				QM_CEETM_VERB_CQ_PEEK_POP_XFDRREAD);

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: CQ PEEK/POP/XSFDR READ failed\n");
		return -EIO;
	}
	*cq_ppxr = mcr->cq_ppxr;
	return 0;
}

static int qman_ceetm_query_statistics(u16 cid,
			enum qm_dc_portal dcp_idx,
			u16 command_type,
			struct qm_mcr_ceetm_statistics_query *query_result)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->stats_query_write.cid = cid;
	mcc->stats_query_write.dcpid = dcp_idx;
	mcc->stats_query_write.ct = command_type;
	qm_mc_commit(&p->p, QM_CEETM_VERB_STATISTICS_QUERY_WRITE);

	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
					 QM_CEETM_VERB_STATISTICS_QUERY_WRITE);

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: STATISTICS QUERY failed\n");
		return -EIO;
	}
	*query_result = mcr->stats_query;
	return 0;
}

int qman_ceetm_query_write_statistics(u16 cid, enum qm_dc_portal dcp_idx,
				      u16 command_type, u64 frame_count,
				      u64 byte_count)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	mcc->stats_query_write.cid = cid;
	mcc->stats_query_write.dcpid = dcp_idx;
	mcc->stats_query_write.ct = command_type;
	mcc->stats_query_write.frm_cnt = frame_count;
	mcc->stats_query_write.byte_cnt = byte_count;
	qm_mc_commit(&p->p, QM_CEETM_VERB_STATISTICS_QUERY_WRITE);

	while (!(mcr = qm_mc_result(&p->p)))
		cpu_relax();
	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
					 QM_CEETM_VERB_STATISTICS_QUERY_WRITE);

	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();

	res = mcr->result;
	if (res != QM_MCR_RESULT_OK) {
		pr_err("CEETM: STATISTICS WRITE failed\n");
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_query_write_statistics);

int qman_ceetm_bps2tokenrate(u64 bps, struct qm_ceetm_rate *token_rate,
							int rounding)
{
	u16 pres;
	u64 temp;
	u64 qman_freq;
	int ret;

	/* Read PRES from CEET_CFG_PRES register */
	ret = qman_ceetm_get_prescaler(&pres);
	if (ret)
		return -EINVAL;

	ret = qm_get_clock(&qman_freq);
	if (ret)
		return -EINVAL;

	/* token-rate = bytes-per-second * update-reference-period
	 *
	 * Where token-rate is N/8192 for a integer N, and
	 * update-reference-period is (2^22)/(PRES*QHz), where PRES
	 * is the prescalar value and QHz is the QMan clock frequency.
	 * So:
	 *
	 * token-rate = (byte-per-second*2^22)/PRES*QHZ)
	 *
	 * Converting to bits-per-second gives;
	 *
	 *	token-rate = (bps*2^19) / (PRES*QHZ)
	 *	N = (bps*2^32) / (PRES*QHz)
	 *
	 * And to avoid 64-bit overflow if 'bps' is larger than 4Gbps
	 * (yet minimise rounding error if 'bps' is small), we reorganise
	 * the formula to use two 16-bit shifts rather than 1 32-bit shift.
	 *      N = (((bps*2^16)/PRES)*2^16)/QHz
	 */
	temp = ROUNDING((bps << 16), pres, rounding);
	temp = ROUNDING((temp << 16), qman_freq, rounding);
	token_rate->whole = temp >> 13;
	token_rate->fraction = temp & (((u64)1 << 13) - 1);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_bps2tokenrate);

int qman_ceetm_tokenrate2bps(const struct qm_ceetm_rate *token_rate, u64 *bps,
							int rounding)
{
	u16 pres;
	u64 temp;
	u64 qman_freq;
	int ret;

	/* Read PRES from CEET_CFG_PRES register */
	ret = qman_ceetm_get_prescaler(&pres);
	if (ret)
		return -EINVAL;

	ret = qm_get_clock(&qman_freq);
	if (ret)
		return -EINVAL;

	/* bytes-per-second = token-rate / update-reference-period
	 *
	 * where "token-rate" is N/8192 for an integer N, and
	 * "update-reference-period" is (2^22)/(PRES*QHz), where PRES is
	 * the prescalar value and QHz is the QMan clock frequency. So;
	 *
	 * bytes-per-second = (N/8192) / (4194304/PRES*QHz)
	 *                  = N*PRES*QHz / (4194304*8192)
	 *                  = N*PRES*QHz / (2^35)
	 *
	 * Converting to bits-per-second gives;
	 *
	 *             bps = N*PRES*QHZ / (2^32)
	 *
	 * Note, the numerator has a maximum width of 72 bits! So to
	 * avoid 64-bit overflow errors, we calculate PRES*QHZ (maximum
	 * width 48 bits) divided by 2^9 (reducing to maximum 39 bits), before
	 * multiplying by N (goes to maximum of 63 bits).
	 *
	 *             temp = PRES*QHZ / (2^16)
	 *             kbps = temp*N / (2^16)
	 */
	temp = ROUNDING(qman_freq * pres, (u64)1 << 16 , rounding);
	temp *= ((token_rate->whole << 13) + token_rate->fraction);
	*bps = ROUNDING(temp, (u64)(1) << 16, rounding);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_tokenrate2bps);

int qman_ceetm_sp_claim(struct qm_ceetm_sp **sp, enum qm_dc_portal dcp_idx,
						unsigned int sp_idx)
{
	struct qm_ceetm_sp *p;

	DPA_ASSERT((dcp_idx ==  qm_dc_portal_fman0) ||
			(dcp_idx == qm_dc_portal_fman1));

	if ((sp_idx < qman_ceetms[dcp_idx].sp_range[0]) ||
		(sp_idx >= (qman_ceetms[dcp_idx].sp_range[0] +
		qman_ceetms[dcp_idx].sp_range[1]))) {
		pr_err("Sub-portal index doesn't exist\n");
		return -EINVAL;
	}

	list_for_each_entry(p, &qman_ceetms[dcp_idx].sub_portals, node) {
		if ((p->idx == sp_idx) && (p->is_claimed == 0)) {
			p->is_claimed = 1;
			*sp = p;
			return 0;
		}
	}
	pr_err("The sub-portal#%d is not available!\n", sp_idx);
	return -ENODEV;
}
EXPORT_SYMBOL(qman_ceetm_sp_claim);

int qman_ceetm_sp_release(struct qm_ceetm_sp *sp)
{
	struct qm_ceetm_sp *p;

	if (sp->lni && sp->lni->is_claimed == 1) {
		pr_err("The dependency of sub-portal has not been released!\n");
		return -EBUSY;
	}

	list_for_each_entry(p, &qman_ceetms[sp->dcp_idx].sub_portals, node) {
		if (p->idx == sp->idx) {
			p->is_claimed = 0;
			p->lni = NULL;
		}
	}
	/* Disable CEETM mode of this sub-portal */
	qman_sp_disable_ceetm_mode(sp->dcp_idx, sp->idx);

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_sp_release);

int qman_ceetm_lni_claim(struct qm_ceetm_lni **lni, enum qm_dc_portal dcp_idx,
							unsigned int lni_idx)
{
	struct qm_ceetm_lni *p;

	if ((lni_idx < qman_ceetms[dcp_idx].lni_range[0]) ||
		(lni_idx >= (qman_ceetms[dcp_idx].lni_range[0] +
		qman_ceetms[dcp_idx].lni_range[1]))) {
		pr_err("The lni index is out of range\n");
		return -EINVAL;
	}

	list_for_each_entry(p, &qman_ceetms[dcp_idx].lnis, node) {
		if ((p->idx == lni_idx) && (p->is_claimed == 0)) {
			*lni = p;
			p->is_claimed = 1;
			return 0;
		}
	}

	pr_err("The LNI#%d is not available!\n", lni_idx);
	return -EINVAL;
}
EXPORT_SYMBOL(qman_ceetm_lni_claim);

int qman_ceetm_lni_release(struct qm_ceetm_lni *lni)
{
	struct qm_ceetm_lni *p;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	if (!list_empty(&lni->channels)) {
		pr_err("The LNI dependencies are not released!\n");
		return -EBUSY;
	}

	list_for_each_entry(p, &qman_ceetms[lni->dcp_idx].lnis, node) {
		if (p->idx == lni->idx) {
			p->shaper_enable = 0;
			p->shaper_couple = 0;
			p->cr_token_rate.whole = 0;
			p->cr_token_rate.fraction = 0;
			p->er_token_rate.whole = 0;
			p->er_token_rate.fraction = 0;
			p->cr_token_bucket_limit = 0;
			p->er_token_bucket_limit = 0;
			p->is_claimed = 0;
		}
	}
	config_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	config_opts.dcpid = lni->dcp_idx;
	memset(&config_opts.shaper_config, 0,
				sizeof(config_opts.shaper_config));
	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_lni_release);

int qman_ceetm_sp_set_lni(struct qm_ceetm_sp *sp, struct qm_ceetm_lni *lni)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_SP_MAPPING | sp->idx);
	config_opts.dcpid = sp->dcp_idx;
	config_opts.sp_mapping.map_lni_id = lni->idx;
	sp->lni = lni;

	if (qman_ceetm_configure_mapping_shaper_tcfc(&config_opts))
		return -EINVAL;

	/* Enable CEETM mode for this sub-portal */
	return qman_sp_enable_ceetm_mode(sp->dcp_idx, sp->idx);
}
EXPORT_SYMBOL(qman_ceetm_sp_set_lni);

int qman_ceetm_sp_get_lni(struct qm_ceetm_sp *sp, unsigned int *lni_idx)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_SP_MAPPING | sp->idx);
	query_opts.dcpid = sp->dcp_idx;
	if (qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result)) {
		pr_err("Can't get SP <-> LNI mapping\n");
		return -EINVAL;
	}
	*lni_idx = query_result.sp_mapping_query.map_lni_id;
	sp->lni->idx = query_result.sp_mapping_query.map_lni_id;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_sp_get_lni);

int qman_ceetm_lni_enable_shaper(struct qm_ceetm_lni *lni, int coupled,
								int oal)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	if (lni->shaper_enable) {
		pr_err("The shaper has already been enabled\n");
		return -EINVAL;
	}
	lni->shaper_enable = 1;
	lni->shaper_couple = coupled;
	lni->oal = oal;

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	config_opts.dcpid = lni->dcp_idx;
	config_opts.shaper_config.cpl = coupled;
	config_opts.shaper_config.oal = oal;
	config_opts.shaper_config.crtcr = cpu_to_be24((lni->cr_token_rate.whole
					<< 13) | lni->cr_token_rate.fraction);
	config_opts.shaper_config.ertcr = cpu_to_be24((lni->er_token_rate.whole
					<< 13) | lni->er_token_rate.fraction);
	config_opts.shaper_config.crtbl =
					cpu_to_be16(lni->cr_token_bucket_limit);
	config_opts.shaper_config.ertbl =
					cpu_to_be16(lni->er_token_bucket_limit);
	config_opts.shaper_config.mps = 60;

	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_lni_enable_shaper);

int qman_ceetm_lni_disable_shaper(struct qm_ceetm_lni *lni)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	if (!lni->shaper_enable) {
		pr_err("The shaper has been disabled\n");
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	config_opts.dcpid = lni->dcp_idx;
	config_opts.shaper_config.cpl = lni->shaper_couple;
	config_opts.shaper_config.oal = lni->oal;
	config_opts.shaper_config.crtbl =
					cpu_to_be16(lni->cr_token_bucket_limit);
	config_opts.shaper_config.ertbl =
					cpu_to_be16(lni->er_token_bucket_limit);
	/* Set CR/ER rate with all 1's to configure an infinite rate, thus
	 * disable the shaping.
	 */
	config_opts.shaper_config.crtcr = 0xFFFFFF;
	config_opts.shaper_config.ertcr = 0xFFFFFF;
	config_opts.shaper_config.mps = 60;
	lni->shaper_enable = 0;
	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_lni_disable_shaper);

int qman_ceetm_lni_is_shaper_enabled(struct qm_ceetm_lni *lni)
{
	return lni->shaper_enable;
}
EXPORT_SYMBOL(qman_ceetm_lni_is_shaper_enabled);

int qman_ceetm_lni_set_commit_rate(struct qm_ceetm_lni *lni,
				const struct qm_ceetm_rate *token_rate,
				u16 token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	lni->cr_token_rate.whole = token_rate->whole;
	lni->cr_token_rate.fraction = token_rate->fraction;
	lni->cr_token_bucket_limit = token_limit;
	if (!lni->shaper_enable)
		return 0;
	query_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	query_opts.dcpid = lni->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts,
						   &query_result);
	if (ret) {
		pr_err("Fail to get current LNI shaper setting\n");
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	config_opts.dcpid = lni->dcp_idx;
	config_opts.shaper_config.crtcr = cpu_to_be24((token_rate->whole << 13)
						      | (token_rate->fraction));
	config_opts.shaper_config.crtbl = cpu_to_be16(token_limit);
	config_opts.shaper_config.cpl = query_result.shaper_query.cpl;
	config_opts.shaper_config.oal = query_result.shaper_query.oal;
	config_opts.shaper_config.ertcr = query_result.shaper_query.ertcr;
	config_opts.shaper_config.ertbl = query_result.shaper_query.ertbl;
	config_opts.shaper_config.mps = query_result.shaper_query.mps;
	return	qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_lni_set_commit_rate);

int qman_ceetm_lni_set_commit_rate_bps(struct qm_ceetm_lni *lni,
				       u64 bps,
				       u16 token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_bps2tokenrate(bps, &token_rate, 0);
	if (ret) {
		pr_err("Can not convert bps to token rate\n");
		return -EINVAL;
	}

	return qman_ceetm_lni_set_commit_rate(lni, &token_rate, token_limit);
}
EXPORT_SYMBOL(qman_ceetm_lni_set_commit_rate_bps);

int qman_ceetm_lni_get_commit_rate(struct qm_ceetm_lni *lni,
				struct qm_ceetm_rate *token_rate,
				u16 *token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	query_opts.dcpid = lni->dcp_idx;

	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret) {
		pr_err("The LNI CR rate or limit is not set\n");
		return -EINVAL;
	}
	token_rate->whole = be24_to_cpu(query_result.shaper_query.crtcr) >> 13;
	token_rate->fraction = be24_to_cpu(query_result.shaper_query.crtcr) &
				 0x1FFF;
	*token_limit = be16_to_cpu(query_result.shaper_query.crtbl);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_lni_get_commit_rate);

int qman_ceetm_lni_get_commit_rate_bps(struct qm_ceetm_lni *lni,
				       u64 *bps, u16 *token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_lni_get_commit_rate(lni, &token_rate, token_limit);
	if (ret) {
		pr_err("The LNI CR rate or limit is not available\n");
		return -EINVAL;
	}

	return qman_ceetm_tokenrate2bps(&token_rate, bps, 0);
}
EXPORT_SYMBOL(qman_ceetm_lni_get_commit_rate_bps);

int qman_ceetm_lni_set_excess_rate(struct qm_ceetm_lni *lni,
					const struct qm_ceetm_rate *token_rate,
					u16 token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	lni->er_token_rate.whole = token_rate->whole;
	lni->er_token_rate.fraction = token_rate->fraction;
	lni->er_token_bucket_limit = token_limit;
	if (!lni->shaper_enable)
		return 0;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	query_opts.dcpid = lni->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts,
						   &query_result);
	if (ret) {
		pr_err("Fail to get current LNI shaper setting\n");
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	config_opts.dcpid = lni->dcp_idx;
	config_opts.shaper_config.ertcr = cpu_to_be24(
		(token_rate->whole << 13) | (token_rate->fraction));
	config_opts.shaper_config.ertbl = cpu_to_be16(token_limit);
	config_opts.shaper_config.cpl = query_result.shaper_query.cpl;
	config_opts.shaper_config.oal = query_result.shaper_query.oal;
	config_opts.shaper_config.crtcr = query_result.shaper_query.crtcr;
	config_opts.shaper_config.crtbl = query_result.shaper_query.crtbl;
	config_opts.shaper_config.mps = query_result.shaper_query.mps;
	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_lni_set_excess_rate);

int qman_ceetm_lni_set_excess_rate_bps(struct qm_ceetm_lni *lni,
				       u64 bps,
				       u16 token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_bps2tokenrate(bps, &token_rate, 0);
	if (ret) {
		pr_err("Can not convert bps to token rate\n");
		return -EINVAL;
	}
	return qman_ceetm_lni_set_excess_rate(lni, &token_rate, token_limit);
}
EXPORT_SYMBOL(qman_ceetm_lni_set_excess_rate_bps);

int qman_ceetm_lni_get_excess_rate(struct qm_ceetm_lni *lni,
					struct qm_ceetm_rate *token_rate,
					u16 *token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	query_opts.dcpid = lni->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret) {
		pr_err("The LNI ER rate or limit is not set\n");
		return -EINVAL;
	}
	token_rate->whole = be24_to_cpu(query_result.shaper_query.ertcr) >> 13;
	token_rate->fraction = be24_to_cpu(query_result.shaper_query.ertcr) &
								 0x1FFF;
	*token_limit = be16_to_cpu(query_result.shaper_query.ertbl);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_lni_get_excess_rate);

int qman_ceetm_lni_get_excess_rate_bps(struct qm_ceetm_lni *lni,
				       u64 *bps, u16 *token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_lni_get_excess_rate(lni, &token_rate, token_limit);
	if (ret) {
		pr_err("The LNI ER rate or limit is not available\n");
		return -EINVAL;
	}

	return qman_ceetm_tokenrate2bps(&token_rate, bps, 0);
}
EXPORT_SYMBOL(qman_ceetm_lni_get_excess_rate_bps);

#define QMAN_CEETM_LNITCFCC_CQ_LEVEL_SHIFT(n) ((15 - n) * 4)
#define QMAN_CEETM_LNITCFCC_ENABLE 0x8
int qman_ceetm_lni_set_tcfcc(struct qm_ceetm_lni *lni,
				unsigned int cq_level,
				int traffic_class)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	u64 lnitcfcc;

	if ((cq_level > 15) | (traffic_class > 7)) {
		pr_err("The CQ or traffic class id is out of range\n");
		return -EINVAL;
	}

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_TCFC | lni->idx);
	query_opts.dcpid = lni->dcp_idx;
	if (qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result)) {
		pr_err("Fail to query tcfcc\n");
		return -EINVAL;
	}

	lnitcfcc = be64_to_cpu(query_result.tcfc_query.lnitcfcc);
	if (traffic_class == -1) {
		/* disable tcfc for this CQ */
		lnitcfcc &= ~((u64)QMAN_CEETM_LNITCFCC_ENABLE <<
				QMAN_CEETM_LNITCFCC_CQ_LEVEL_SHIFT(cq_level));
	} else {
		lnitcfcc &= ~((u64)0xF <<
				QMAN_CEETM_LNITCFCC_CQ_LEVEL_SHIFT(cq_level));
		lnitcfcc |= ((u64)(QMAN_CEETM_LNITCFCC_ENABLE |
				traffic_class)) <<
				QMAN_CEETM_LNITCFCC_CQ_LEVEL_SHIFT(cq_level);
	}
	config_opts.tcfc_config.lnitcfcc = cpu_to_be64(lnitcfcc);
	config_opts.cid = cpu_to_be16(CEETM_COMMAND_TCFC | lni->idx);
	config_opts.dcpid = lni->dcp_idx;
	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_lni_set_tcfcc);

#define QMAN_CEETM_LNITCFCC_TC_MASK 0x7
int qman_ceetm_lni_get_tcfcc(struct qm_ceetm_lni *lni, unsigned int cq_level,
						int *traffic_class)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;
	u8 lnitcfcc;

	if (cq_level > 15) {
		pr_err("the CQ level is out of range\n");
		return -EINVAL;
	}

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_TCFC | lni->idx);
	query_opts.dcpid = lni->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret)
		return ret;
	lnitcfcc = (u8)be64_to_cpu((query_result.tcfc_query.lnitcfcc) >>
				QMAN_CEETM_LNITCFCC_CQ_LEVEL_SHIFT(cq_level));
	if (lnitcfcc & QMAN_CEETM_LNITCFCC_ENABLE)
		*traffic_class = lnitcfcc & QMAN_CEETM_LNITCFCC_TC_MASK;
	else
		*traffic_class = -1;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_lni_get_tcfcc);

int qman_ceetm_channel_claim(struct qm_ceetm_channel **channel,
				struct qm_ceetm_lni *lni)
{
	struct qm_ceetm_channel *p;
	u32 channel_idx;
	int ret = 0;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	if (lni->dcp_idx == qm_dc_portal_fman0) {
		ret = qman_alloc_ceetm0_channel(&channel_idx);
	} else if (lni->dcp_idx == qm_dc_portal_fman1) {
		ret = qman_alloc_ceetm1_channel(&channel_idx);
	} else {
		pr_err("dcp_idx %u does not correspond to a known fman in this driver\n",
			lni->dcp_idx);
		return -EINVAL;
	}

	if (ret) {
		pr_err("The is no channel available for LNI#%d\n", lni->idx);
		return -ENODEV;
	}

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->idx = channel_idx;
	p->dcp_idx = lni->dcp_idx;
	p->lni_idx = lni->idx;
	list_add_tail(&p->node, &lni->channels);
	INIT_LIST_HEAD(&p->class_queues);
	INIT_LIST_HEAD(&p->ccgs);
	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_MAPPING |
						channel_idx);
	config_opts.dcpid = lni->dcp_idx;
	config_opts.channel_mapping.map_lni_id = lni->idx;
	config_opts.channel_mapping.map_shaped = 0;
	if (qman_ceetm_configure_mapping_shaper_tcfc(&config_opts)) {
		pr_err("Can't map channel#%d for LNI#%d\n",
						channel_idx, lni->idx);
		return -EINVAL;
	}
	*channel = p;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_claim);

int qman_ceetm_channel_release(struct qm_ceetm_channel *channel)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;
	if (!list_empty(&channel->class_queues)) {
		pr_err("CEETM channel#%d has class queue unreleased!\n",
						channel->idx);
		return -EBUSY;
	}
	if (!list_empty(&channel->ccgs)) {
		pr_err("CEETM channel#%d has ccg unreleased!\n",
						channel->idx);
		return -EBUSY;
	}

	/* channel->dcp_idx corresponds to known fman validation */
	if ((channel->dcp_idx != qm_dc_portal_fman0) &&
	    (channel->dcp_idx != qm_dc_portal_fman1)) {
		pr_err("dcp_idx %u does not correspond to a known fman in this driver\n",
			channel->dcp_idx);
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
				      channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	memset(&config_opts.shaper_config, 0,
				sizeof(config_opts.shaper_config));
	if (qman_ceetm_configure_mapping_shaper_tcfc(&config_opts)) {
		pr_err("Can't reset channel shapping parameters\n");
		return -EINVAL;
	}

	if (channel->dcp_idx == qm_dc_portal_fman0) {
		qman_release_ceetm0_channelid(channel->idx);
	} else if (channel->dcp_idx == qm_dc_portal_fman1) {
		qman_release_ceetm1_channelid(channel->idx);
	} else {
		pr_err("dcp_idx %u does not correspond to a known fman in this driver\n",
			channel->dcp_idx);
		return -EINVAL;
	}
	list_del(&channel->node);
	kfree(channel);

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_release);

int qman_ceetm_channel_enable_shaper(struct qm_ceetm_channel *channel,
								int coupled)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	if (channel->shaper_enable == 1) {
		pr_err("This channel shaper has been enabled!\n");
		return -EINVAL;
	}

	channel->shaper_enable = 1;
	channel->shaper_couple = coupled;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_MAPPING |
						channel->idx);
	query_opts.dcpid = channel->dcp_idx;

	if (qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result)) {
		pr_err("Can't query channel mapping\n");
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_MAPPING |
						channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	config_opts.channel_mapping.map_lni_id =
				query_result.channel_mapping_query.map_lni_id;
	config_opts.channel_mapping.map_shaped = 1;
	if (qman_ceetm_configure_mapping_shaper_tcfc(&config_opts)) {
		pr_err("Can't enable shaper for channel #%d\n",	channel->idx);
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
						channel->idx);
	config_opts.shaper_config.cpl = coupled;
	config_opts.shaper_config.crtcr =
				cpu_to_be24((channel->cr_token_rate.whole
				<< 13) |
				channel->cr_token_rate.fraction);
	config_opts.shaper_config.ertcr =
				cpu_to_be24(channel->er_token_rate.whole
				<< 13 |
				channel->er_token_rate.fraction);
	config_opts.shaper_config.crtbl =
				cpu_to_be16(channel->cr_token_bucket_limit);
	config_opts.shaper_config.ertbl =
				cpu_to_be16(channel->er_token_bucket_limit);

	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_channel_enable_shaper);

int qman_ceetm_channel_disable_shaper(struct qm_ceetm_channel *channel)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;


	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_MAPPING |
						channel->idx);
	query_opts.dcpid = channel->dcp_idx;

	if (qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result)) {
		pr_err("Can't query channel mapping\n");
		return -EINVAL;
	}

	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_MAPPING |
						channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	config_opts.channel_mapping.map_shaped = 0;
	config_opts.channel_mapping.map_lni_id =
				query_result.channel_mapping_query.map_lni_id;

	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_channel_disable_shaper);

int qman_ceetm_channel_is_shaper_enabled(struct qm_ceetm_channel *channel)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_MAPPING |
						channel->idx);
	query_opts.dcpid = channel->dcp_idx;

	if (qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result)) {
		pr_err("Can't query channel mapping\n");
		return -EINVAL;
	}

	return query_result.channel_mapping_query.map_shaped;
}
EXPORT_SYMBOL(qman_ceetm_channel_is_shaper_enabled);

int qman_ceetm_channel_set_commit_rate(struct qm_ceetm_channel *channel,
				const struct qm_ceetm_rate *token_rate,
				u16 token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
						channel->idx);
	query_opts.dcpid = channel->dcp_idx;

	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret) {
		pr_err("Fail to get the current channel shaper setting\n");
		return -EINVAL;
	}

	channel->cr_token_rate.whole = token_rate->whole;
	channel->cr_token_rate.fraction = token_rate->fraction;
	channel->cr_token_bucket_limit = token_limit;
	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
							channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	config_opts.shaper_config.crtcr = cpu_to_be24((token_rate->whole
					<< 13) | (token_rate->fraction));
	config_opts.shaper_config.crtbl = cpu_to_be16(token_limit);
	config_opts.shaper_config.cpl = query_result.shaper_query.cpl;
	config_opts.shaper_config.ertcr = query_result.shaper_query.ertcr;
	config_opts.shaper_config.ertbl = query_result.shaper_query.ertbl;
	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_channel_set_commit_rate);

int qman_ceetm_channel_set_commit_rate_bps(struct qm_ceetm_channel *channel,
					   u64 bps, u16 token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_bps2tokenrate(bps, &token_rate, 0);
	if (ret) {
		pr_err("Can not convert bps to token rate\n");
		return -EINVAL;
	}
	return qman_ceetm_channel_set_commit_rate(channel, &token_rate,
						  token_limit);
}
EXPORT_SYMBOL(qman_ceetm_channel_set_commit_rate_bps);

int qman_ceetm_channel_get_commit_rate(struct qm_ceetm_channel *channel,
				struct qm_ceetm_rate *token_rate,
				u16 *token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
						channel->idx);
	query_opts.dcpid = channel->dcp_idx;

	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret | !query_result.shaper_query.crtcr |
			 !query_result.shaper_query.crtbl) {
		pr_err("The channel commit rate or limit is not set\n");
		return -EINVAL;
	}
	token_rate->whole = be24_to_cpu(query_result.shaper_query.crtcr) >> 13;
	token_rate->fraction = be24_to_cpu(query_result.shaper_query.crtcr) &
							0x1FFF;
	*token_limit = be16_to_cpu(query_result.shaper_query.crtbl);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_get_commit_rate);

int qman_ceetm_channel_get_commit_rate_bps(struct qm_ceetm_channel *channel,
					   u64 *bps, u16 *token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_channel_get_commit_rate(channel, &token_rate,
						 token_limit);
	if (ret) {
		pr_err("The channel CR rate or limit is not available\n");
		return -EINVAL;
	}

	return qman_ceetm_tokenrate2bps(&token_rate, bps, 0);
}
EXPORT_SYMBOL(qman_ceetm_channel_get_commit_rate_bps);

int qman_ceetm_channel_set_excess_rate(struct qm_ceetm_channel *channel,
					const struct qm_ceetm_rate *token_rate,
					u16 token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
							channel->idx);
	query_opts.dcpid = channel->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret) {
		pr_err("Fail to get the current channel shaper setting\n");
		return -EINVAL;
	}

	channel->er_token_rate.whole = token_rate->whole;
	channel->er_token_rate.fraction = token_rate->fraction;
	channel->er_token_bucket_limit = token_limit;
	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
							channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	config_opts.shaper_config.ertcr = cpu_to_be24(
			 (token_rate->whole << 13) | (token_rate->fraction));
	config_opts.shaper_config.ertbl = cpu_to_be16(token_limit);
	config_opts.shaper_config.cpl = query_result.shaper_query.cpl;
	config_opts.shaper_config.crtcr = query_result.shaper_query.crtcr;
	config_opts.shaper_config.crtbl = query_result.shaper_query.crtbl;
	return qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_channel_set_excess_rate);

int qman_ceetm_channel_set_excess_rate_bps(struct qm_ceetm_channel *channel,
					   u64 bps, u16 token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_bps2tokenrate(bps, &token_rate, 0);
	if (ret) {
		pr_err("Can not convert bps to token rate\n");
		return -EINVAL;
	}
	return qman_ceetm_channel_set_excess_rate(channel, &token_rate,
						  token_limit);
}
EXPORT_SYMBOL(qman_ceetm_channel_set_excess_rate_bps);

int qman_ceetm_channel_get_excess_rate(struct qm_ceetm_channel *channel,
					struct qm_ceetm_rate *token_rate,
					u16 *token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
							channel->idx);
	query_opts.dcpid = channel->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret | !query_result.shaper_query.ertcr |
			 !query_result.shaper_query.ertbl) {
		pr_err("The channel excess rate or limit is not set\n");
		return -EINVAL;
	}
	token_rate->whole = be24_to_cpu(query_result.shaper_query.ertcr) >> 13;
	token_rate->fraction = be24_to_cpu(query_result.shaper_query.ertcr) &
								0x1FFF;
	*token_limit = be16_to_cpu(query_result.shaper_query.ertbl);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_get_excess_rate);

int qman_ceetm_channel_get_excess_rate_bps(struct qm_ceetm_channel *channel,
					   u64 *bps, u16 *token_limit)
{
	struct qm_ceetm_rate token_rate;
	int ret;

	ret = qman_ceetm_channel_get_excess_rate(channel, &token_rate,
						 token_limit);
	if (ret) {
		pr_err("The channel ER rate or limit is not available\n");
		return -EINVAL;
	}

	return qman_ceetm_tokenrate2bps(&token_rate, bps, 0);
}
EXPORT_SYMBOL(qman_ceetm_channel_get_excess_rate_bps);

int qman_ceetm_channel_set_weight(struct qm_ceetm_channel *channel,
						u16 token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config config_opts;

	if (channel->shaper_enable) {
		pr_err("This channel is a shaped one\n");
		return -EINVAL;
	}

	channel->cr_token_bucket_limit = token_limit;
	config_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
						channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	config_opts.shaper_config.crtbl = cpu_to_be16(token_limit);
	return	qman_ceetm_configure_mapping_shaper_tcfc(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_channel_set_weight);

int qman_ceetm_channel_get_weight(struct qm_ceetm_channel *channel,
					u16 *token_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query query_opts;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query query_result;
	int ret;

	query_opts.cid = cpu_to_be16(CEETM_COMMAND_CHANNEL_SHAPER |
						channel->idx);
	query_opts.dcpid = channel->dcp_idx;
	ret = qman_ceetm_query_mapping_shaper_tcfc(&query_opts, &query_result);
	if (ret | !query_result.shaper_query.crtbl) {
		pr_err("This unshaped channel's uFQ wight is unavailable\n");
		return -EINVAL;
	}
	*token_limit = be16_to_cpu(query_result.shaper_query.crtbl);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_get_weight);

int qman_ceetm_channel_set_group(struct qm_ceetm_channel *channel, int group_b,
				unsigned int prio_a, unsigned int prio_b)
{
	struct qm_mcc_ceetm_class_scheduler_config config_opts;
	struct qm_mcr_ceetm_class_scheduler_query query_result;
	int i;

	if (prio_a > 7) {
		pr_err("The priority of group A is out of range\n");
		return -EINVAL;
	}
	if (group_b && (prio_b > 7)) {
		pr_err("The priority of group B is out of range\n");
		return -EINVAL;
	}

	if (qman_ceetm_query_class_scheduler(channel, &query_result)) {
		pr_err("Can't query channel#%d's scheduler!\n", channel->idx);
		return -EINVAL;
	}

	config_opts.cqcid = cpu_to_be16(channel->idx);
	config_opts.dcpid = channel->dcp_idx;
	config_opts.gpc_combine_flag = !group_b;
	config_opts.gpc_prio_a = prio_a;
	config_opts.gpc_prio_b = prio_b;

	for (i = 0; i < 8; i++)
		config_opts.w[i] = query_result.w[i];
	config_opts.crem = query_result.crem;
	config_opts.erem = query_result.erem;

	return qman_ceetm_configure_class_scheduler(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_channel_set_group);

int qman_ceetm_channel_get_group(struct qm_ceetm_channel *channel, int *group_b,
				unsigned int *prio_a, unsigned int *prio_b)
{
	struct qm_mcr_ceetm_class_scheduler_query query_result;

	if (qman_ceetm_query_class_scheduler(channel, &query_result)) {
		pr_err("Can't query channel#%d's scheduler!\n", channel->idx);
		return -EINVAL;
	}
	*group_b = !query_result.gpc_combine_flag;
	*prio_a = query_result.gpc_prio_a;
	*prio_b = query_result.gpc_prio_b;

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_get_group);

#define GROUP_A_ELIGIBILITY_SET		(1 << 8)
#define GROUP_B_ELIGIBILITY_SET		(1 << 9)
#define CQ_ELIGIBILITY_SET(n)		(1 << (7 - n))
int qman_ceetm_channel_set_group_cr_eligibility(struct qm_ceetm_channel
				*channel, int group_b, int cre)
{
	struct qm_mcc_ceetm_class_scheduler_config csch_config;
	struct qm_mcr_ceetm_class_scheduler_query csch_query;
	int i;

	if (qman_ceetm_query_class_scheduler(channel, &csch_query)) {
		pr_err("Cannot get the channel %d scheduler setting.\n",
						channel->idx);
		return -EINVAL;
	}
	csch_config.cqcid = cpu_to_be16(channel->idx);
	csch_config.dcpid = channel->dcp_idx;
	csch_config.gpc_combine_flag = csch_query.gpc_combine_flag;
	csch_config.gpc_prio_a = csch_query.gpc_prio_a;
	csch_config.gpc_prio_b = csch_query.gpc_prio_b;

	for (i = 0; i < 8; i++)
		csch_config.w[i] = csch_query.w[i];
	csch_config.erem = csch_query.erem;
	if (group_b)
		csch_config.crem = (be16_to_cpu(csch_query.crem)
					& ~GROUP_B_ELIGIBILITY_SET)
					| (cre ? GROUP_B_ELIGIBILITY_SET : 0);
	else
		csch_config.crem = (be16_to_cpu(csch_query.crem)
					& ~GROUP_A_ELIGIBILITY_SET)
					| (cre ? GROUP_A_ELIGIBILITY_SET : 0);

	csch_config.crem = cpu_to_be16(csch_config.crem);

	if (qman_ceetm_configure_class_scheduler(&csch_config)) {
		pr_err("Cannot config channel %d's scheduler with "
			"group_%c's cr eligibility\n", channel->idx,
			group_b ? 'b' : 'a');
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_set_group_cr_eligibility);

int qman_ceetm_channel_set_group_er_eligibility(struct qm_ceetm_channel
				*channel, int group_b, int ere)
{
	struct qm_mcc_ceetm_class_scheduler_config csch_config;
	struct qm_mcr_ceetm_class_scheduler_query csch_query;
	int i;

	if (qman_ceetm_query_class_scheduler(channel, &csch_query)) {
		pr_err("Cannot get the channel %d scheduler setting.\n",
						channel->idx);
		return -EINVAL;
	}
	csch_config.cqcid = cpu_to_be16(channel->idx);
	csch_config.dcpid = channel->dcp_idx;
	csch_config.gpc_combine_flag = csch_query.gpc_combine_flag;
	csch_config.gpc_prio_a = csch_query.gpc_prio_a;
	csch_config.gpc_prio_b = csch_query.gpc_prio_b;

	for (i = 0; i < 8; i++)
		csch_config.w[i] = csch_query.w[i];
	csch_config.crem = csch_query.crem;
	if (group_b)
		csch_config.erem = (be16_to_cpu(csch_query.erem)
					& ~GROUP_B_ELIGIBILITY_SET)
					| (ere ? GROUP_B_ELIGIBILITY_SET : 0);
	else
		csch_config.erem = (be16_to_cpu(csch_query.erem)
					& ~GROUP_A_ELIGIBILITY_SET)
					| (ere ? GROUP_A_ELIGIBILITY_SET : 0);

	csch_config.erem = cpu_to_be16(csch_config.erem);

	if (qman_ceetm_configure_class_scheduler(&csch_config)) {
		pr_err("Cannot config channel %d's scheduler with "
			"group_%c's er eligibility\n", channel->idx,
			group_b ? 'b' : 'a');
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_set_group_er_eligibility);

int qman_ceetm_channel_set_cq_cr_eligibility(struct qm_ceetm_channel *channel,
						unsigned int idx, int cre)
{
	struct qm_mcc_ceetm_class_scheduler_config csch_config;
	struct qm_mcr_ceetm_class_scheduler_query csch_query;
	int i;

	if (idx > 7) {
		pr_err("CQ index is out of range\n");
		return -EINVAL;
	}
	if (qman_ceetm_query_class_scheduler(channel, &csch_query)) {
		pr_err("Cannot get the channel %d scheduler setting.\n",
						channel->idx);
		return -EINVAL;
	}
	csch_config.cqcid = cpu_to_be16(channel->idx);
	csch_config.dcpid = channel->dcp_idx;
	csch_config.gpc_combine_flag = csch_query.gpc_combine_flag;
	csch_config.gpc_prio_a = csch_query.gpc_prio_a;
	csch_config.gpc_prio_b = csch_query.gpc_prio_b;
	for (i = 0; i < 8; i++)
		csch_config.w[i] = csch_query.w[i];
	csch_config.erem = csch_query.erem;
	csch_config.crem = (be16_to_cpu(csch_query.crem)
					& ~CQ_ELIGIBILITY_SET(idx)) |
					(cre ? CQ_ELIGIBILITY_SET(idx) : 0);
	csch_config.crem = cpu_to_be16(csch_config.crem);
	if (qman_ceetm_configure_class_scheduler(&csch_config)) {
		pr_err("Cannot config channel scheduler to set "
			"cr eligibility mask for CQ#%d\n", idx);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_set_cq_cr_eligibility);

int qman_ceetm_channel_set_cq_er_eligibility(struct qm_ceetm_channel *channel,
						unsigned int idx, int ere)
{
	struct qm_mcc_ceetm_class_scheduler_config csch_config;
	struct qm_mcr_ceetm_class_scheduler_query csch_query;
	int i;

	if (idx > 7) {
		pr_err("CQ index is out of range\n");
		return -EINVAL;
	}
	if (qman_ceetm_query_class_scheduler(channel, &csch_query)) {
		pr_err("Cannot get the channel %d scheduler setting.\n",
						channel->idx);
		return -EINVAL;
	}
	csch_config.cqcid = cpu_to_be16(channel->idx);
	csch_config.dcpid = channel->dcp_idx;
	csch_config.gpc_combine_flag = csch_query.gpc_combine_flag;
	csch_config.gpc_prio_a = csch_query.gpc_prio_a;
	csch_config.gpc_prio_b = csch_query.gpc_prio_b;
	for (i = 0; i < 8; i++)
		csch_config.w[i] = csch_query.w[i];
	csch_config.crem = csch_query.crem;
	csch_config.erem = (be16_to_cpu(csch_query.erem)
					& ~CQ_ELIGIBILITY_SET(idx)) |
					(ere ? CQ_ELIGIBILITY_SET(idx) : 0);
	csch_config.erem = cpu_to_be16(csch_config.erem);
	if (qman_ceetm_configure_class_scheduler(&csch_config)) {
		pr_err("Cannot config channel scheduler to set "
			"er eligibility mask for CQ#%d\n", idx);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_channel_set_cq_er_eligibility);

int qman_ceetm_cq_claim(struct qm_ceetm_cq **cq,
		struct qm_ceetm_channel *channel, unsigned int idx,
		struct qm_ceetm_ccg *ccg)
{
	struct qm_ceetm_cq *p;
	struct qm_mcc_ceetm_cq_config cq_config;

	if (idx > 7) {
		pr_err("The independent class queue id is out of range\n");
		return -EINVAL;
	}

	list_for_each_entry(p, &channel->class_queues, node) {
		if (p->idx == idx) {
			pr_err("The CQ#%d has been claimed!\n", idx);
			return -EINVAL;
		}
	}

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		pr_err("Can't allocate memory for CQ#%d!\n", idx);
		return -ENOMEM;
	}

	list_add_tail(&p->node, &channel->class_queues);
	p->idx = idx;
	p->is_claimed = 1;
	p->parent = channel;
	INIT_LIST_HEAD(&p->bound_lfqids);

	if (ccg) {
		cq_config.cqid = cpu_to_be16((channel->idx << 4) | idx);
		cq_config.dcpid = channel->dcp_idx;
		cq_config.ccgid = cpu_to_be16(ccg->idx);
		if (qman_ceetm_configure_cq(&cq_config)) {
			pr_err("Can't configure the CQ#%d with CCGRID#%d\n",
						 idx, ccg->idx);
			list_del(&p->node);
			kfree(p);
			return -EINVAL;
		}
	}

	*cq = p;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cq_claim);

int qman_ceetm_cq_claim_A(struct qm_ceetm_cq **cq,
		struct qm_ceetm_channel *channel, unsigned int idx,
		struct qm_ceetm_ccg *ccg)
{
	struct qm_ceetm_cq *p;
	struct qm_mcc_ceetm_cq_config cq_config;

	if ((idx < 8) || (idx > 15)) {
		pr_err("This grouped class queue id is out of range\n");
		return -EINVAL;
	}

	list_for_each_entry(p, &channel->class_queues, node) {
		if (p->idx == idx) {
			pr_err("The CQ#%d has been claimed!\n", idx);
			return -EINVAL;
		}
	}

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		pr_err("Can't allocate memory for CQ#%d!\n", idx);
		return -ENOMEM;
	}

	list_add_tail(&p->node, &channel->class_queues);
	p->idx = idx;
	p->is_claimed = 1;
	p->parent = channel;
	INIT_LIST_HEAD(&p->bound_lfqids);

	if (ccg) {
		cq_config.cqid = cpu_to_be16((channel->idx << 4) | idx);
		cq_config.dcpid = channel->dcp_idx;
		cq_config.ccgid = cpu_to_be16(ccg->idx);
		if (qman_ceetm_configure_cq(&cq_config)) {
			pr_err("Can't configure the CQ#%d with CCGRID#%d\n",
						 idx, ccg->idx);
			list_del(&p->node);
			kfree(p);
			return -EINVAL;
		}
	}
	*cq = p;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cq_claim_A);

int qman_ceetm_cq_claim_B(struct qm_ceetm_cq **cq,
		struct qm_ceetm_channel *channel, unsigned int idx,
		struct qm_ceetm_ccg *ccg)
{
	struct qm_ceetm_cq *p;
	struct qm_mcc_ceetm_cq_config cq_config;

	if ((idx < 12) || (idx > 15)) {
		pr_err("This grouped class queue id is out of range\n");
		return -EINVAL;
	}

	list_for_each_entry(p, &channel->class_queues, node) {
		if (p->idx == idx) {
			pr_err("The CQ#%d has been claimed!\n", idx);
			return -EINVAL;
		}
	}

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		pr_err("Can't allocate memory for CQ#%d!\n", idx);
		return -ENOMEM;
	}

	list_add_tail(&p->node, &channel->class_queues);
	p->idx = idx;
	p->is_claimed = 1;
	p->parent = channel;
	INIT_LIST_HEAD(&p->bound_lfqids);

	if (ccg) {
		cq_config.cqid = cpu_to_be16((channel->idx << 4) | idx);
		cq_config.dcpid = channel->dcp_idx;
		cq_config.ccgid = cpu_to_be16(ccg->idx);
		if (qman_ceetm_configure_cq(&cq_config)) {
			pr_err("Can't configure the CQ#%d with CCGRID#%d\n",
					 idx, ccg->idx);
			list_del(&p->node);
			kfree(p);
			return -EINVAL;
		}
	}
	*cq = p;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cq_claim_B);

int qman_ceetm_cq_release(struct qm_ceetm_cq *cq)
{
	if (!list_empty(&cq->bound_lfqids)) {
		pr_err("The CQ#%d has unreleased LFQID\n", cq->idx);
		return -EBUSY;
	}
	list_del(&cq->node);
	qman_ceetm_drain_cq(cq);
	kfree(cq);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cq_release);

int qman_ceetm_set_queue_weight(struct qm_ceetm_cq *cq,
				struct qm_ceetm_weight_code *weight_code)
{
	struct qm_mcc_ceetm_class_scheduler_config config_opts;
	struct qm_mcr_ceetm_class_scheduler_query query_result;
	int i;

	if (cq->idx < 8) {
		pr_err("Can not set weight for ungrouped class queue\n");
		return -EINVAL;
	}

	if (qman_ceetm_query_class_scheduler(cq->parent, &query_result)) {
		pr_err("Can't query channel#%d's scheduler!\n",
						cq->parent->idx);
		return -EINVAL;
	}

	config_opts.cqcid = cpu_to_be16(cq->parent->idx);
	config_opts.dcpid = cq->parent->dcp_idx;
	config_opts.crem = query_result.crem;
	config_opts.erem = query_result.erem;
	config_opts.gpc_combine_flag = query_result.gpc_combine_flag;
	config_opts.gpc_prio_a = query_result.gpc_prio_a;
	config_opts.gpc_prio_b = query_result.gpc_prio_b;

	for (i = 0; i < 8; i++)
		config_opts.w[i] = query_result.w[i];
	config_opts.w[cq->idx - 8] = ((weight_code->y << 3) |
						(weight_code->x & 0x7));
	return qman_ceetm_configure_class_scheduler(&config_opts);
}
EXPORT_SYMBOL(qman_ceetm_set_queue_weight);

int qman_ceetm_get_queue_weight(struct qm_ceetm_cq *cq,
				struct qm_ceetm_weight_code *weight_code)
{
	struct qm_mcr_ceetm_class_scheduler_query query_result;

	if (cq->idx < 8) {
		pr_err("Can not get weight for ungrouped class queue\n");
		return -EINVAL;
	}

	if (qman_ceetm_query_class_scheduler(cq->parent,
						&query_result)) {
		pr_err("Can't get the weight code for CQ#%d!\n", cq->idx);
		return -EINVAL;
	}
	weight_code->y = query_result.w[cq->idx - 8] >> 3;
	weight_code->x = query_result.w[cq->idx - 8] & 0x7;

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_get_queue_weight);

/* The WBFS code is represent as {x,y}, the effect wieght can be calculated as:
 *	effective weight = 2^x / (1 - (y/64))
 *			 = 2^(x+6) / (64 - y)
 */
static void reduce_fraction(u32 *n, u32 *d)
{
	u32 factor = 2;
	u32 lesser = (*n < *d) ? *n : *d;
	/* If factor exceeds the square-root of the lesser of *n and *d,
	 * then there's no point continuing. Proof: if there was a factor
	 * bigger than the square root, that would imply there exists
	 * another factor smaller than the square-root with which it
	 * multiplies to give 'lesser' - but that's a contradiction
	 * because the other factor would have already been found and
	 * divided out.
	 */
	while ((factor * factor) <= lesser) {
		/* If 'factor' is a factor of *n and *d, divide them both
		 * by 'factor' as many times as possible.
		 */
		while (!(*n % factor) && !(*d % factor)) {
			*n /= factor;
			*d /= factor;
			lesser /= factor;
		}
		if (factor == 2)
			factor = 3;
		else
			factor += 2;
	}
}

int qman_ceetm_wbfs2ratio(struct qm_ceetm_weight_code *weight_code,
				u32 *numerator,
				u32 *denominator)
{
	*numerator = (u32) 1 << (weight_code->x + 6);
	*denominator = 64 - weight_code->y;
	reduce_fraction(numerator, denominator);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_wbfs2ratio);

/* For a given x, the weight is between 2^x (inclusive) and 2^(x+1) (exclusive).
 * So find 'x' by range, and then estimate 'y' using:
 *		64 - y	= 2^(x + 6) / weight
 *			= 2^(x + 6) / (n/d)
 *			= d * 2^(x+6) / n
 *		y = 64 - (d * 2^(x+6) / n)
 */
int qman_ceetm_ratio2wbfs(u32 numerator,
				u32 denominator,
				struct qm_ceetm_weight_code *weight_code,
				int rounding)
{
	unsigned int y, x = 0;
	/* search incrementing 'x' until:
	 * weight < 2^(x+1)
	 *    n/d < 2^(x+1)
	 *	n < d * 2^(x+1)
	 */
	while ((x < 8) && (numerator >= (denominator << (x + 1))))
		x++;
	if (x >= 8)
		return -ERANGE;
	/* because of the subtraction, use '-rounding' */
	y = 64 - ROUNDING(denominator << (x + 6), numerator, -rounding);
	if (y >= 32)
		return -ERANGE;
	weight_code->x = x;
	weight_code->y = y;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_ratio2wbfs);

int qman_ceetm_set_queue_weight_in_ratio(struct qm_ceetm_cq *cq, u32 ratio)
{
	struct qm_ceetm_weight_code weight_code;

	if (qman_ceetm_ratio2wbfs(ratio, 100, &weight_code, 0)) {
		pr_err("Cannot get wbfs code for cq %x\n", cq->idx);
		return -EINVAL;
	}
	return qman_ceetm_set_queue_weight(cq, &weight_code);
}
EXPORT_SYMBOL(qman_ceetm_set_queue_weight_in_ratio);

int qman_ceetm_get_queue_weight_in_ratio(struct qm_ceetm_cq *cq, u32 *ratio)
{
	struct qm_ceetm_weight_code weight_code;
	u32 n, d;

	if (qman_ceetm_get_queue_weight(cq, &weight_code)) {
		pr_err("Cannot query the weight code for cq%x\n", cq->idx);
		return -EINVAL;
	}

	if (qman_ceetm_wbfs2ratio(&weight_code, &n, &d)) {
		pr_err("Cannot get the ratio with wbfs code\n");
		return -EINVAL;
	}

	*ratio = (n * 100) / d;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_get_queue_weight_in_ratio);

int qman_ceetm_cq_get_dequeue_statistics(struct qm_ceetm_cq *cq, u32 flags,
					u64 *frame_count, u64 *byte_count)
{
	struct qm_mcr_ceetm_statistics_query result;
	u16 cid, command_type;
	enum qm_dc_portal dcp_idx;
	int ret;

	cid = cpu_to_be16((cq->parent->idx << 4) | cq->idx);
	dcp_idx = cq->parent->dcp_idx;
	if (flags == QMAN_CEETM_FLAG_CLEAR_STATISTICS_COUNTER)
		command_type = CEETM_QUERY_DEQUEUE_CLEAR_STATISTICS;
	else
		command_type = CEETM_QUERY_DEQUEUE_STATISTICS;

	ret = qman_ceetm_query_statistics(cid, dcp_idx, command_type, &result);
	if (ret) {
		pr_err("Can't query the statistics of CQ#%d!\n", cq->idx);
		return -EINVAL;
	}

	*frame_count = be40_to_cpu(result.frm_cnt);
	*byte_count = be48_to_cpu(result.byte_cnt);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cq_get_dequeue_statistics);

int qman_ceetm_drain_cq(struct qm_ceetm_cq *cq)
{
	struct qm_mcr_ceetm_cq_peek_pop_xsfdrread ppxr;
	int ret;

	do {
		ret = qman_ceetm_cq_peek_pop_xsfdrread(cq, 1, 0, &ppxr);
		if (ret) {
			pr_err("Failed to pop frame from CQ\n");
			return -EINVAL;
		}
	} while (!(ppxr.stat & 0x2));

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_drain_cq);

#define CEETM_LFQMT_LFQID_MSB 0xF00000
#define CEETM_LFQMT_LFQID_LSB 0x000FFF
int qman_ceetm_lfq_claim(struct qm_ceetm_lfq **lfq,
					struct qm_ceetm_cq *cq)
{
	struct qm_ceetm_lfq *p;
	u32 lfqid;
	int ret = 0;
	struct qm_mcc_ceetm_lfqmt_config lfqmt_config;

	if (cq->parent->dcp_idx == qm_dc_portal_fman0) {
		ret = qman_alloc_ceetm0_lfqid(&lfqid);
	} else if (cq->parent->dcp_idx == qm_dc_portal_fman1) {
		ret = qman_alloc_ceetm1_lfqid(&lfqid);
	} else {
		pr_err("dcp_idx %u does not correspond to a known fman in this driver\n",
			cq->parent->dcp_idx);
		return -EINVAL;
	}

	if (ret) {
		pr_err("There is no lfqid avalaible for CQ#%d!\n", cq->idx);
		return -ENODEV;
	}
	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->idx = lfqid;
	p->dctidx = (u16)(lfqid & CEETM_LFQMT_LFQID_LSB);
	p->parent = cq->parent;
	list_add_tail(&p->node, &cq->bound_lfqids);

	lfqmt_config.lfqid = cpu_to_be24(CEETM_LFQMT_LFQID_MSB |
				(cq->parent->dcp_idx << 16) |
				(lfqid & CEETM_LFQMT_LFQID_LSB));
	lfqmt_config.cqid = cpu_to_be16((cq->parent->idx << 4) | (cq->idx));
	lfqmt_config.dctidx = cpu_to_be16(p->dctidx);
	if (qman_ceetm_configure_lfqmt(&lfqmt_config)) {
		pr_err("Can't configure LFQMT for LFQID#%d @ CQ#%d\n",
				lfqid, cq->idx);
		list_del(&p->node);
		kfree(p);
		return -EINVAL;
	}
	*lfq = p;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_lfq_claim);

int qman_ceetm_lfq_release(struct qm_ceetm_lfq *lfq)
{
	if (lfq->parent->dcp_idx == qm_dc_portal_fman0) {
		qman_release_ceetm0_lfqid(lfq->idx);
	} else if (lfq->parent->dcp_idx == qm_dc_portal_fman1) {
		qman_release_ceetm1_lfqid(lfq->idx);
	} else {
		pr_err("dcp_idx %u does not correspond to a known fman in this driver\n",
			lfq->parent->dcp_idx);
		return -EINVAL;
	}
	list_del(&lfq->node);
	kfree(lfq);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_lfq_release);

int qman_ceetm_lfq_set_context(struct qm_ceetm_lfq *lfq, u64 context_a,
							u32 context_b)
{
	struct qm_mcc_ceetm_dct_config dct_config;
	lfq->context_a = context_a;
	lfq->context_b = context_b;
	dct_config.dctidx = cpu_to_be16((u16)lfq->dctidx);
	dct_config.dcpid = lfq->parent->dcp_idx;
	dct_config.context_b = cpu_to_be32(context_b);
	dct_config.context_a = cpu_to_be64(context_a);

	return qman_ceetm_configure_dct(&dct_config);
}
EXPORT_SYMBOL(qman_ceetm_lfq_set_context);

int qman_ceetm_lfq_get_context(struct qm_ceetm_lfq *lfq, u64 *context_a,
							u32 *context_b)
{
	struct qm_mcc_ceetm_dct_query dct_query;
	struct qm_mcr_ceetm_dct_query query_result;

	dct_query.dctidx = cpu_to_be16(lfq->dctidx);
	dct_query.dcpid = lfq->parent->dcp_idx;
	if (qman_ceetm_query_dct(&dct_query, &query_result)) {
		pr_err("Can't query LFQID#%d's context!\n", lfq->idx);
		return -EINVAL;
	}
	*context_a = be64_to_cpu(query_result.context_a);
	*context_b = be32_to_cpu(query_result.context_b);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_lfq_get_context);

int qman_ceetm_create_fq(struct qm_ceetm_lfq *lfq, struct qman_fq *fq)
{
	spin_lock_init(&fq->fqlock);
	fq->fqid = lfq->idx;
	fq->flags = QMAN_FQ_FLAG_NO_MODIFY;
	if (lfq->ern)
		fq->cb.ern = lfq->ern;
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	if (unlikely(find_empty_fq_table_entry(&fq->key, fq)))
		return -ENOMEM;
#endif
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_create_fq);

#define MAX_CCG_IDX 0x000F
int qman_ceetm_ccg_claim(struct qm_ceetm_ccg **ccg,
				struct qm_ceetm_channel *channel,
				unsigned int idx,
				void (*cscn)(struct qm_ceetm_ccg *,
					void *cb_ctx,
					int congested),
				void *cb_ctx)
{
	struct qm_ceetm_ccg *p;

	if (idx > MAX_CCG_IDX) {
		pr_err("The given ccg index is out of range\n");
		return -EINVAL;
	}

	list_for_each_entry(p, &channel->ccgs, node) {
		if (p->idx == idx) {
			pr_err("The CCG#%d has been claimed\n", idx);
			return -EINVAL;
		}
	}

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		pr_err("Can't allocate memory for CCG#%d!\n", idx);
		return -ENOMEM;
	}

	list_add_tail(&p->node, &channel->ccgs);

	p->idx = idx;
	p->parent = channel;
	p->cb = cscn;
	p->cb_ctx = cb_ctx;
	INIT_LIST_HEAD(&p->cb_node);

	*ccg = p;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_ccg_claim);

int qman_ceetm_ccg_release(struct qm_ceetm_ccg *ccg)
{
	unsigned long irqflags __maybe_unused;
	struct qm_mcc_ceetm_ccgr_config config_opts;
	int ret = 0;
	struct qman_portal *p = get_affine_portal();

	memset(&config_opts, 0, sizeof(struct qm_mcc_ceetm_ccgr_config));
	spin_lock_irqsave(&p->ccgr_lock, irqflags);
	if (!list_empty(&ccg->cb_node))
		list_del(&ccg->cb_node);
	config_opts.ccgrid = cpu_to_be16(CEETM_CCGR_CM_CONFIGURE |
				(ccg->parent->idx << 4) | ccg->idx);
	config_opts.dcpid = ccg->parent->dcp_idx;
	config_opts.we_mask = cpu_to_be16(QM_CCGR_WE_CSCN_TUPD);
	config_opts.cm_config.cscn_tupd = cpu_to_be16(PORTAL_IDX(p));
	ret = qman_ceetm_configure_ccgr(&config_opts);
	spin_unlock_irqrestore(&p->ccgr_lock, irqflags);
	put_affine_portal();

	list_del(&ccg->node);
	kfree(ccg);
	return ret;
}
EXPORT_SYMBOL(qman_ceetm_ccg_release);

int qman_ceetm_ccg_set(struct qm_ceetm_ccg *ccg, u16 we_mask,
				const struct qm_ceetm_ccg_params *params)
{
	struct qm_mcc_ceetm_ccgr_config config_opts;
	unsigned long irqflags __maybe_unused;
	int ret;
	struct qman_portal *p;

	if (((ccg->parent->idx << 4) | ccg->idx) >= (2 * __CGR_NUM))
		return -EINVAL;

	p = get_affine_portal();

	memset(&config_opts, 0, sizeof(struct qm_mcc_ceetm_ccgr_config));
	spin_lock_irqsave(&p->ccgr_lock, irqflags);

	config_opts.ccgrid = cpu_to_be16(CEETM_CCGR_CM_CONFIGURE |
				(ccg->parent->idx << 4) | ccg->idx);
	config_opts.dcpid = ccg->parent->dcp_idx;
	config_opts.we_mask = we_mask;
	if (we_mask & QM_CCGR_WE_CSCN_EN) {
		config_opts.we_mask |= QM_CCGR_WE_CSCN_TUPD;
		config_opts.cm_config.cscn_tupd = cpu_to_be16(
			QM_CGR_TARG_UDP_CTRL_WRITE_BIT | PORTAL_IDX(p));
	}
	config_opts.we_mask = cpu_to_be16(config_opts.we_mask);
	config_opts.cm_config.ctl_wr_en_g = params->wr_en_g;
	config_opts.cm_config.ctl_wr_en_y = params->wr_en_y;
	config_opts.cm_config.ctl_wr_en_r = params->wr_en_r;
	config_opts.cm_config.ctl_td_en = params->td_en;
	config_opts.cm_config.ctl_td_mode = params->td_mode;
	config_opts.cm_config.ctl_cscn_en = params->cscn_en;
	config_opts.cm_config.ctl_mode = params->mode;
	config_opts.cm_config.oal = params->oal;
	config_opts.cm_config.cs_thres.hword =
					cpu_to_be16(params->cs_thres_in.hword);
	config_opts.cm_config.cs_thres_x.hword =
					cpu_to_be16(params->cs_thres_out.hword);
	config_opts.cm_config.td_thres.hword =
					cpu_to_be16(params->td_thres.hword);
	config_opts.cm_config.wr_parm_g.word =
					cpu_to_be32(params->wr_parm_g.word);
	config_opts.cm_config.wr_parm_y.word =
					cpu_to_be32(params->wr_parm_y.word);
	config_opts.cm_config.wr_parm_r.word =
					cpu_to_be32(params->wr_parm_r.word);
	ret = qman_ceetm_configure_ccgr(&config_opts);
	if (ret) {
		pr_err("Configure CCGR CM failed!\n");
		goto release_lock;
	}

	if (we_mask & QM_CCGR_WE_CSCN_EN)
		if (list_empty(&ccg->cb_node))
			list_add(&ccg->cb_node,
				&p->ccgr_cbs[ccg->parent->dcp_idx]);
release_lock:
	spin_unlock_irqrestore(&p->ccgr_lock, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(qman_ceetm_ccg_set);

#define CEETM_CCGR_CTL_MASK 0x01
int qman_ceetm_ccg_get(struct qm_ceetm_ccg *ccg,
				struct qm_ceetm_ccg_params *params)
{
	struct qm_mcc_ceetm_ccgr_query query_opts;
	struct qm_mcr_ceetm_ccgr_query query_result;

	query_opts.ccgrid = cpu_to_be16(CEETM_CCGR_CM_QUERY |
				(ccg->parent->idx << 4) | ccg->idx);
	query_opts.dcpid = ccg->parent->dcp_idx;

	if (qman_ceetm_query_ccgr(&query_opts, &query_result)) {
		pr_err("Can't query CCGR#%d\n", ccg->idx);
		return -EINVAL;
	}

	params->wr_parm_r.word = query_result.cm_query.wr_parm_r.word;
	params->wr_parm_y.word = query_result.cm_query.wr_parm_y.word;
	params->wr_parm_g.word = query_result.cm_query.wr_parm_g.word;
	params->td_thres.hword = query_result.cm_query.td_thres.hword;
	params->cs_thres_out.hword = query_result.cm_query.cs_thres_x.hword;
	params->cs_thres_in.hword = query_result.cm_query.cs_thres.hword;
	params->oal = query_result.cm_query.oal;
	params->wr_en_g = query_result.cm_query.ctl_wr_en_g;
	params->wr_en_y = query_result.cm_query.ctl_wr_en_y;
	params->wr_en_r = query_result.cm_query.ctl_wr_en_r;
	params->td_en = query_result.cm_query.ctl_td_en;
	params->td_mode = query_result.cm_query.ctl_td_mode;
	params->cscn_en = query_result.cm_query.ctl_cscn_en;
	params->mode = query_result.cm_query.ctl_mode;

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_ccg_get);

int qman_ceetm_ccg_get_reject_statistics(struct qm_ceetm_ccg *ccg, u32 flags,
					u64 *frame_count, u64 *byte_count)
{
	struct qm_mcr_ceetm_statistics_query result;
	u16 cid, command_type;
	enum qm_dc_portal dcp_idx;
	int ret;

	cid = cpu_to_be16((ccg->parent->idx << 4) | ccg->idx);
	dcp_idx = ccg->parent->dcp_idx;
	if (flags == QMAN_CEETM_FLAG_CLEAR_STATISTICS_COUNTER)
		command_type = CEETM_QUERY_REJECT_CLEAR_STATISTICS;
	else
		command_type = CEETM_QUERY_REJECT_STATISTICS;

	ret = qman_ceetm_query_statistics(cid, dcp_idx, command_type, &result);
	if (ret) {
		pr_err("Can't query the statistics of CCG#%d!\n", ccg->idx);
		return -EINVAL;
	}

	*frame_count = be40_to_cpu(result.frm_cnt);
	*byte_count = be48_to_cpu(result.byte_cnt);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_ccg_get_reject_statistics);

int qman_ceetm_cscn_swp_get(struct qm_ceetm_ccg *ccg,
					u16 swp_idx,
					unsigned int *cscn_enabled)
{
	struct qm_mcc_ceetm_ccgr_query query_opts;
	struct qm_mcr_ceetm_ccgr_query query_result;
	int i;

	DPA_ASSERT(swp_idx < 127);
	query_opts.ccgrid = cpu_to_be16(CEETM_CCGR_CM_QUERY |
				(ccg->parent->idx << 4) | ccg->idx);
	query_opts.dcpid = ccg->parent->dcp_idx;

	if (qman_ceetm_query_ccgr(&query_opts, &query_result)) {
		pr_err("Can't query CCGR#%d\n", ccg->idx);
		return -EINVAL;
	}

	i = swp_idx / 32;
	i = 3 - i;
	*cscn_enabled = query_result.cm_query.cscn_targ_swp[i] >>
							(31 - swp_idx % 32);

	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cscn_swp_get);

int qman_ceetm_cscn_dcp_set(struct qm_ceetm_ccg *ccg,
				u16 dcp_idx,
				u8 vcgid,
				unsigned int cscn_enabled,
				u16 we_mask,
				const struct qm_ceetm_ccg_params *params)
{
	struct qm_mcc_ceetm_ccgr_config config_opts;
	int ret;

	config_opts.ccgrid = cpu_to_be16(CEETM_CCGR_CM_CONFIGURE |
				(ccg->parent->idx << 4) | ccg->idx);
	config_opts.dcpid = ccg->parent->dcp_idx;
	config_opts.we_mask = cpu_to_be16(we_mask | QM_CCGR_WE_CSCN_TUPD |
						 QM_CCGR_WE_CDV);
	config_opts.cm_config.cdv = vcgid;
	config_opts.cm_config.cscn_tupd = cpu_to_be16((cscn_enabled << 15) |
					QM_CGR_TARG_UDP_CTRL_DCP | dcp_idx);
	config_opts.cm_config.ctl_wr_en_g = params->wr_en_g;
	config_opts.cm_config.ctl_wr_en_y = params->wr_en_y;
	config_opts.cm_config.ctl_wr_en_r = params->wr_en_r;
	config_opts.cm_config.ctl_td_en = params->td_en;
	config_opts.cm_config.ctl_td_mode = params->td_mode;
	config_opts.cm_config.ctl_cscn_en = params->cscn_en;
	config_opts.cm_config.ctl_mode = params->mode;
	config_opts.cm_config.cs_thres.hword =
					cpu_to_be16(params->cs_thres_in.hword);
	config_opts.cm_config.cs_thres_x.hword =
					cpu_to_be16(params->cs_thres_out.hword);
	config_opts.cm_config.td_thres.hword =
					cpu_to_be16(params->td_thres.hword);
	config_opts.cm_config.wr_parm_g.word =
					cpu_to_be32(params->wr_parm_g.word);
	config_opts.cm_config.wr_parm_y.word =
					cpu_to_be32(params->wr_parm_y.word);
	config_opts.cm_config.wr_parm_r.word =
					cpu_to_be32(params->wr_parm_r.word);

	ret = qman_ceetm_configure_ccgr(&config_opts);
	if (ret) {
		pr_err("Configure CSCN_TARG_DCP failed!\n");
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cscn_dcp_set);

int qman_ceetm_cscn_dcp_get(struct qm_ceetm_ccg *ccg,
				u16 dcp_idx,
				u8 *vcgid,
				unsigned int *cscn_enabled)
{
	struct qm_mcc_ceetm_ccgr_query query_opts;
	struct qm_mcr_ceetm_ccgr_query query_result;

	query_opts.ccgrid = cpu_to_be16(CEETM_CCGR_CM_QUERY |
				(ccg->parent->idx << 4) | ccg->idx);
	query_opts.dcpid = ccg->parent->dcp_idx;

	if (qman_ceetm_query_ccgr(&query_opts, &query_result)) {
		pr_err("Can't query CCGR#%d\n", ccg->idx);
		return -EINVAL;
	}

	*vcgid = query_result.cm_query.cdv;
	*cscn_enabled = (query_result.cm_query.cscn_targ_dcp >> dcp_idx) & 0x1;
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_cscn_dcp_get);

int qman_ceetm_querycongestion(struct __qm_mcr_querycongestion *ccg_state,
							unsigned int dcp_idx)
{
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	u8 res;
	int i, j;

	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);

	mcc = qm_mc_start(&p->p);
	for (i = 0; i < 2; i++) {
		mcc->ccgr_query.ccgrid =
				cpu_to_be16(CEETM_QUERY_CONGESTION_STATE | i);
		mcc->ccgr_query.dcpid = dcp_idx;
		qm_mc_commit(&p->p, QM_CEETM_VERB_CCGR_QUERY);

		while (!(mcr = qm_mc_result(&p->p)))
			cpu_relax();
		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
						QM_CEETM_VERB_CCGR_QUERY);
		res = mcr->result;
		if (res == QM_MCR_RESULT_OK) {
			for (j = 0; j < 8; j++)
				mcr->ccgr_query.congestion_state.state.
				__state[j] = be32_to_cpu(mcr->ccgr_query.
					congestion_state.state.__state[j]);
			*(ccg_state + i) =
				mcr->ccgr_query.congestion_state.state;
		} else {
			pr_err("QUERY CEETM CONGESTION STATE failed\n");
			PORTAL_IRQ_UNLOCK(p, irqflags);
			return -EIO;
		}
	}
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return 0;
}

int qman_set_wpm(int wpm_enable)
{
	return qm_set_wpm(wpm_enable);
}
EXPORT_SYMBOL(qman_set_wpm);

int qman_get_wpm(int *wpm_enable)
{
	return qm_get_wpm(wpm_enable);
}
EXPORT_SYMBOL(qman_get_wpm);

int qman_shutdown_fq(u32 fqid)
{
	struct qman_portal *p;
	unsigned long irqflags __maybe_unused;
	int ret;
	struct qm_portal *low_p;
	p = get_affine_portal();
	PORTAL_IRQ_LOCK(p, irqflags);
	low_p = &p->p;
	ret = qm_shutdown_fq(&low_p, 1, fqid);
	PORTAL_IRQ_UNLOCK(p, irqflags);
	put_affine_portal();
	return ret;
}

const struct qm_portal_config *qman_get_qm_portal_config(
						struct qman_portal *portal)
{
	return portal->sharing_redirect ? NULL : portal->config;
}
