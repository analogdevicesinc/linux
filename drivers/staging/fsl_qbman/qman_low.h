/* Copyright 2008-2011 Freescale Semiconductor, Inc.
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

#include <linux/iopoll.h>

#include "qman_private.h"

struct qm_portal *qm_get_portal_for_channel(u16 channel);
int drain_mr_fqrni(struct qm_portal *p);

#define QM_MC_DELAY_US		1
#define QM_MC_TIMEOUT_US	10000
#define FQRN_WAIT_TIMEOUT	msecs_to_jiffies(1000)

/***************************/
/* Portal register assists */
/***************************/

/* Cache-inhibited register offsets */
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64)

#define QM_REG_EQCR_PI_CINH	0x0000
#define QM_REG_EQCR_CI_CINH	0x0004
#define QM_REG_EQCR_ITR		0x0008
#define QM_REG_DQRR_PI_CINH	0x0040
#define QM_REG_DQRR_CI_CINH	0x0044
#define QM_REG_DQRR_ITR		0x0048
#define QM_REG_DQRR_DCAP	0x0050
#define QM_REG_DQRR_SDQCR	0x0054
#define QM_REG_DQRR_VDQCR	0x0058
#define QM_REG_DQRR_PDQCR	0x005c
#define QM_REG_MR_PI_CINH	0x0080
#define QM_REG_MR_CI_CINH	0x0084
#define QM_REG_MR_ITR		0x0088
#define QM_REG_CFG		0x0100
#define QM_REG_ISR		0x0e00
#define QM_REG_IIR              0x0e0c
#define QM_REG_ITPR		0x0e14

/* Cache-enabled register offsets */
#define QM_CL_EQCR		0x0000
#define QM_CL_DQRR		0x1000
#define QM_CL_MR		0x2000
#define QM_CL_EQCR_PI_CENA	0x3000
#define QM_CL_EQCR_CI_CENA	0x3100
#define QM_CL_DQRR_PI_CENA	0x3200
#define QM_CL_DQRR_CI_CENA	0x3300
#define QM_CL_MR_PI_CENA	0x3400
#define QM_CL_MR_CI_CENA	0x3500
#define QM_CL_CR		0x3800
#define QM_CL_RR0		0x3900
#define QM_CL_RR1		0x3940

#endif

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)

#define QM_REG_EQCR_PI_CINH	0x3000
#define QM_REG_EQCR_CI_CINH	0x3040
#define QM_REG_EQCR_ITR		0x3080
#define QM_REG_DQRR_PI_CINH	0x3100
#define QM_REG_DQRR_CI_CINH	0x3140
#define QM_REG_DQRR_ITR		0x3180
#define QM_REG_DQRR_DCAP	0x31C0
#define QM_REG_DQRR_SDQCR	0x3200
#define QM_REG_DQRR_VDQCR	0x3240
#define QM_REG_DQRR_PDQCR	0x3280
#define QM_REG_MR_PI_CINH	0x3300
#define QM_REG_MR_CI_CINH	0x3340
#define QM_REG_MR_ITR		0x3380
#define QM_REG_CFG		0x3500
#define QM_REG_ISR		0x3600
#define QM_REG_IIR              0x36C0
#define QM_REG_ITPR		0x3740

/* Cache-enabled register offsets */
#define QM_CL_EQCR		0x0000
#define QM_CL_DQRR		0x1000
#define QM_CL_MR		0x2000
#define QM_CL_EQCR_PI_CENA	0x3000
#define QM_CL_EQCR_CI_CENA	0x3040
#define QM_CL_DQRR_PI_CENA	0x3100
#define QM_CL_DQRR_CI_CENA	0x3140
#define QM_CL_MR_PI_CENA	0x3300
#define QM_CL_MR_CI_CENA	0x3340
#define QM_CL_CR		0x3800
#define QM_CL_RR0		0x3900
#define QM_CL_RR1		0x3940

#endif


/* BTW, the drivers (and h/w programming model) already obtain the required
 * synchronisation for portal accesses via lwsync(), hwsync(), and
 * data-dependencies. Use of barrier()s or other order-preserving primitives
 * simply degrade performance. Hence the use of the __raw_*() interfaces, which
 * simply ensure that the compiler treats the portal registers as volatile (ie.
 * non-coherent). */

/* Cache-inhibited register access. */
#define __qm_in(qm, o)		be32_to_cpu(__raw_readl((qm)->addr_ci  + (o)))
#define __qm_out(qm, o, val)	__raw_writel((cpu_to_be32(val)), \
					     (qm)->addr_ci + (o));
#define qm_in(reg)		__qm_in(&portal->addr, QM_REG_##reg)
#define qm_out(reg, val)	__qm_out(&portal->addr, QM_REG_##reg, val)

/* Cache-enabled (index) register access */
#define __qm_cl_touch_ro(qm, o) dcbt_ro((qm)->addr_ce + (o))
#define __qm_cl_touch_rw(qm, o) dcbt_rw((qm)->addr_ce + (o))
#define __qm_cl_in(qm, o)	be32_to_cpu(__raw_readl((qm)->addr_ce + (o)))
#define __qm_cl_out(qm, o, val) \
	do { \
		u32 *__tmpclout = (qm)->addr_ce + (o); \
		__raw_writel(cpu_to_be32(val), __tmpclout); \
		dcbf(__tmpclout); \
	} while (0)
#define __qm_cl_invalidate(qm, o) dcbi((qm)->addr_ce + (o))
#define qm_cl_touch_ro(reg) __qm_cl_touch_ro(&portal->addr, QM_CL_##reg##_CENA)
#define qm_cl_touch_rw(reg) __qm_cl_touch_rw(&portal->addr, QM_CL_##reg##_CENA)
#define qm_cl_in(reg)	    __qm_cl_in(&portal->addr, QM_CL_##reg##_CENA)
#define qm_cl_out(reg, val) __qm_cl_out(&portal->addr, QM_CL_##reg##_CENA, val)
#define qm_cl_invalidate(reg)\
	__qm_cl_invalidate(&portal->addr, QM_CL_##reg##_CENA)

/* Cache-enabled ring access */
#define qm_cl(base, idx)	((void *)base + ((idx) << 6))

/* Cyclic helper for rings. FIXME: once we are able to do fine-grain perf
 * analysis, look at using the "extra" bit in the ring index registers to avoid
 * cyclic issues. */
static inline u8 qm_cyc_diff(u8 ringsize, u8 first, u8 last)
{
	/* 'first' is included, 'last' is excluded */
	if (first <= last)
		return last - first;
	return ringsize + last - first;
}

/* Portal modes.
 *   Enum types;
 *     pmode == production mode
 *     cmode == consumption mode,
 *     dmode == h/w dequeue mode.
 *   Enum values use 3 letter codes. First letter matches the portal mode,
 *   remaining two letters indicate;
 *     ci == cache-inhibited portal register
 *     ce == cache-enabled portal register
 *     vb == in-band valid-bit (cache-enabled)
 *     dc == DCA (Discrete Consumption Acknowledgement), DQRR-only
 *   As for "enum qm_dqrr_dmode", it should be self-explanatory.
 */
enum qm_eqcr_pmode {		/* matches QCSP_CFG::EPM */
	qm_eqcr_pci = 0,	/* PI index, cache-inhibited */
	qm_eqcr_pce = 1,	/* PI index, cache-enabled */
	qm_eqcr_pvb = 2		/* valid-bit */
};
enum qm_dqrr_dmode {		/* matches QCSP_CFG::DP */
	qm_dqrr_dpush = 0,	/* SDQCR  + VDQCR */
	qm_dqrr_dpull = 1	/* PDQCR */
};
enum qm_dqrr_pmode {		/* s/w-only */
	qm_dqrr_pci,		/* reads DQRR_PI_CINH */
	qm_dqrr_pce,		/* reads DQRR_PI_CENA */
	qm_dqrr_pvb		/* reads valid-bit */
};
enum qm_dqrr_cmode {		/* matches QCSP_CFG::DCM */
	qm_dqrr_cci = 0,	/* CI index, cache-inhibited */
	qm_dqrr_cce = 1,	/* CI index, cache-enabled */
	qm_dqrr_cdc = 2		/* Discrete Consumption Acknowledgement */
};
enum qm_mr_pmode {		/* s/w-only */
	qm_mr_pci,		/* reads MR_PI_CINH */
	qm_mr_pce,		/* reads MR_PI_CENA */
	qm_mr_pvb		/* reads valid-bit */
};
enum qm_mr_cmode {		/* matches QCSP_CFG::MM */
	qm_mr_cci = 0,		/* CI index, cache-inhibited */
	qm_mr_cce = 1		/* CI index, cache-enabled */
};


/* ------------------------- */
/* --- Portal structures --- */

#define QM_EQCR_SIZE		8
#define QM_DQRR_SIZE		16
#define QM_MR_SIZE		8

struct qm_eqcr {
	struct qm_eqcr_entry *ring, *cursor;
	u8 ci, available, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	u32 busy;
	enum qm_eqcr_pmode pmode;
#endif
};

struct qm_dqrr {
	const struct qm_dqrr_entry *ring, *cursor;
	u8 pi, ci, fill, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum qm_dqrr_dmode dmode;
	enum qm_dqrr_pmode pmode;
	enum qm_dqrr_cmode cmode;
#endif
};

struct qm_mr {
	const struct qm_mr_entry *ring, *cursor;
	u8 pi, ci, fill, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum qm_mr_pmode pmode;
	enum qm_mr_cmode cmode;
#endif
};

struct qm_mc {
	struct qm_mc_command *cr;
	struct qm_mc_result *rr;
	u8 rridx, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum {
		/* Can be _mc_start()ed */
		qman_mc_idle,
		/* Can be _mc_commit()ed or _mc_abort()ed */
		qman_mc_user,
		/* Can only be _mc_retry()ed */
		qman_mc_hw
	} state;
#endif
};

#define QM_PORTAL_ALIGNMENT ____cacheline_aligned

struct qm_addr {
	void __iomem *addr_ce;	/* cache-enabled */
	void __iomem *addr_ci;	/* cache-inhibited */
};

struct qm_portal {
	/* In the non-CONFIG_FSL_DPA_CHECKING case, the following stuff up to
	 * and including 'mc' fits within a cacheline (yay!). The 'config' part
	 * is setup-only, so isn't a cause for a concern. In other words, don't
	 * rearrange this structure on a whim, there be dragons ... */
	struct qm_addr addr;
	struct qm_eqcr eqcr;
	struct qm_dqrr dqrr;
	struct qm_mr mr;
	struct qm_mc mc;
} QM_PORTAL_ALIGNMENT;

u32 qm_portal_get_sdqcr(struct qm_portal *p);

/* ---------------- */
/* --- EQCR API --- */

/* Bit-wise logic to wrap a ring pointer by clearing the "carry bit" */
#define EQCR_CARRYCLEAR(p) \
	(void *)((unsigned long)(p) & (~(unsigned long)(QM_EQCR_SIZE << 6)))

/* Bit-wise logic to convert a ring pointer to a ring index */
static inline u8 EQCR_PTR2IDX(struct qm_eqcr_entry *e)
{
	return ((uintptr_t)e >> 6) & (QM_EQCR_SIZE - 1);
}

/* Increment the 'cursor' ring pointer, taking 'vbit' into account */
static inline void EQCR_INC(struct qm_eqcr *eqcr)
{
	/* NB: this is odd-looking, but experiments show that it generates fast
	 * code with essentially no branching overheads. We increment to the
	 * next EQCR pointer and handle overflow and 'vbit'. */
	struct qm_eqcr_entry *partial = eqcr->cursor + 1;
	eqcr->cursor = EQCR_CARRYCLEAR(partial);
	if (partial != eqcr->cursor)
		eqcr->vbit ^= QM_EQCR_VERB_VBIT;
}

static inline int qm_eqcr_init(struct qm_portal *portal,
				enum qm_eqcr_pmode pmode,
				unsigned int eq_stash_thresh,
				int eq_stash_prio)
{
	/* This use of 'register', as well as all other occurrences, is because
	 * it has been observed to generate much faster code with gcc than is
	 * otherwise the case. */
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u32 cfg;
	u8 pi;

	eqcr->ring = portal->addr.addr_ce + QM_CL_EQCR;
	eqcr->ci = qm_in(EQCR_CI_CINH) & (QM_EQCR_SIZE - 1);
	qm_cl_invalidate(EQCR_CI);
	pi = qm_in(EQCR_PI_CINH) & (QM_EQCR_SIZE - 1);
	eqcr->cursor = eqcr->ring + pi;
	eqcr->vbit = (qm_in(EQCR_PI_CINH) & QM_EQCR_SIZE) ?
			QM_EQCR_VERB_VBIT : 0;
	eqcr->available = QM_EQCR_SIZE - 1 -
			qm_cyc_diff(QM_EQCR_SIZE, eqcr->ci, pi);
	eqcr->ithresh = qm_in(EQCR_ITR);
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
	eqcr->pmode = pmode;
#endif
	cfg = (qm_in(CFG) & 0x00ffffff) |
		(eq_stash_thresh << 28) | /* QCSP_CFG: EST */
		(eq_stash_prio << 26)	| /* QCSP_CFG: EP */
		((pmode & 0x3) << 24);	/* QCSP_CFG::EPM */
	qm_out(CFG, cfg);
	return 0;
}

static inline unsigned int qm_eqcr_get_ci_stashing(struct qm_portal *portal)
{
	return (qm_in(CFG) >> 28) & 0x7;
}

static inline void qm_eqcr_finish(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 pi, ci;
	u32 cfg;

	/*
	 * Disable EQCI stashing because the QMan only
	 * presents the value it previously stashed to
	 * maintain coherency.  Setting the stash threshold
	 * to 1 then 0 ensures that QMan has resyncronized
	 * its internal copy so that the portal is clean
	 * when it is reinitialized in the future
	 */
	cfg = (qm_in(CFG) & 0x0fffffff) |
		(1 << 28); /* QCSP_CFG: EST */
	qm_out(CFG, cfg);
	cfg &= 0x0fffffff; /* stash threshold = 0 */
	qm_out(CFG, cfg);

	pi = qm_in(EQCR_PI_CINH) & (QM_EQCR_SIZE - 1);
	ci = qm_in(EQCR_CI_CINH) & (QM_EQCR_SIZE - 1);

	/* Refresh EQCR CI cache value */
	qm_cl_invalidate(EQCR_CI);
	eqcr->ci = qm_cl_in(EQCR_CI) & (QM_EQCR_SIZE - 1);

	DPA_ASSERT(!eqcr->busy);
	if (pi != EQCR_PTR2IDX(eqcr->cursor))
		pr_crit("losing uncommited EQCR entries\n");
	if (ci != eqcr->ci)
		pr_crit("missing existing EQCR completions\n");
	if (eqcr->ci != EQCR_PTR2IDX(eqcr->cursor))
		pr_crit("EQCR destroyed unquiesced\n");
}

static inline struct qm_eqcr_entry *qm_eqcr_start_no_stash(struct qm_portal
								 *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	DPA_ASSERT(!eqcr->busy);
	if (!eqcr->available)
		return NULL;


#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 1;
#endif
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64)
	dcbz_64(eqcr->cursor);
#endif
	return eqcr->cursor;
}

static inline struct qm_eqcr_entry *qm_eqcr_start_stash(struct qm_portal
								*portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 diff, old_ci;

	DPA_ASSERT(!eqcr->busy);
	if (!eqcr->available) {
		old_ci = eqcr->ci;
		eqcr->ci = qm_cl_in(EQCR_CI) & (QM_EQCR_SIZE - 1);
		diff = qm_cyc_diff(QM_EQCR_SIZE, old_ci, eqcr->ci);
		eqcr->available += diff;
		if (!diff)
			return NULL;
	}
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 1;
#endif
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64)
	dcbz_64(eqcr->cursor);
#endif
	return eqcr->cursor;
}

static inline void qm_eqcr_abort(struct qm_portal *portal)
{
	__maybe_unused register struct qm_eqcr *eqcr = &portal->eqcr;
	DPA_ASSERT(eqcr->busy);
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline struct qm_eqcr_entry *qm_eqcr_pend_and_next(
					struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	DPA_ASSERT(eqcr->busy);
	DPA_ASSERT(eqcr->pmode != qm_eqcr_pvb);
	if (eqcr->available == 1)
		return NULL;
	eqcr->cursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	dcbf(eqcr->cursor);
	EQCR_INC(eqcr);
	eqcr->available--;
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64)
	dcbz_64(eqcr->cursor);
#endif
	return eqcr->cursor;
}

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define EQCR_COMMIT_CHECKS(eqcr) \
do { \
	DPA_ASSERT(eqcr->busy); \
	DPA_ASSERT(eqcr->cursor->orp == (eqcr->cursor->orp & 0xffffff00)); \
	DPA_ASSERT(eqcr->cursor->fqid == (eqcr->cursor->fqid & 0xffffff00)); \
} while (0)
#else
#define EQCR_COMMIT_CHECKS(eqcr) \
do { \
	DPA_ASSERT(eqcr->busy); \
	DPA_ASSERT(eqcr->cursor->orp == (eqcr->cursor->orp & \
					cpu_to_be32(0x00ffffff))); \
	DPA_ASSERT(eqcr->cursor->fqid == (eqcr->cursor->fqid & \
					cpu_to_be32(0x00ffffff))); \
} while (0)
#endif

static inline void qm_eqcr_pci_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	EQCR_COMMIT_CHECKS(eqcr);
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pci);
	eqcr->cursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	EQCR_INC(eqcr);
	eqcr->available--;
	dcbf(eqcr->cursor);
	hwsync();
	qm_out(EQCR_PI_CINH, EQCR_PTR2IDX(eqcr->cursor));
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline void qm_eqcr_pce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_eqcr *eqcr = &portal->eqcr;
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pce);
	qm_cl_invalidate(EQCR_PI);
	qm_cl_touch_rw(EQCR_PI);
}

static inline void qm_eqcr_pce_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	EQCR_COMMIT_CHECKS(eqcr);
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pce);
	eqcr->cursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	EQCR_INC(eqcr);
	eqcr->available--;
	dcbf(eqcr->cursor);
	lwsync();
	qm_cl_out(EQCR_PI, EQCR_PTR2IDX(eqcr->cursor));
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline void qm_eqcr_pvb_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	struct qm_eqcr_entry *eqcursor;
	EQCR_COMMIT_CHECKS(eqcr);
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pvb);
	lwsync();
	eqcursor = eqcr->cursor;
	eqcursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	dcbf(eqcursor);
	EQCR_INC(eqcr);
	eqcr->available--;
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline u8 qm_eqcr_cci_update(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 diff, old_ci = eqcr->ci;
	eqcr->ci = qm_in(EQCR_CI_CINH) & (QM_EQCR_SIZE - 1);
	diff = qm_cyc_diff(QM_EQCR_SIZE, old_ci, eqcr->ci);
	eqcr->available += diff;
	return diff;
}

static inline void qm_eqcr_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_eqcr *eqcr = &portal->eqcr;
	qm_cl_touch_ro(EQCR_CI);
}

static inline u8 qm_eqcr_cce_update(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 diff, old_ci = eqcr->ci;
	eqcr->ci = qm_cl_in(EQCR_CI) & (QM_EQCR_SIZE - 1);
	qm_cl_invalidate(EQCR_CI);
	diff = qm_cyc_diff(QM_EQCR_SIZE, old_ci, eqcr->ci);
	eqcr->available += diff;
	return diff;
}

static inline u8 qm_eqcr_get_ithresh(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	return eqcr->ithresh;
}

static inline void qm_eqcr_set_ithresh(struct qm_portal *portal, u8 ithresh)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	eqcr->ithresh = ithresh;
	qm_out(EQCR_ITR, ithresh);
}

static inline u8 qm_eqcr_get_avail(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	return eqcr->available;
}

static inline u8 qm_eqcr_get_fill(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	return QM_EQCR_SIZE - 1 - eqcr->available;
}


/* ---------------- */
/* --- DQRR API --- */

/* FIXME: many possible improvements;
 * - look at changing the API to use pointer rather than index parameters now
 *   that 'cursor' is a pointer,
 * - consider moving other parameters to pointer if it could help (ci)
 */

#define DQRR_CARRYCLEAR(p) \
	(void *)((unsigned long)(p) & (~(unsigned long)(QM_DQRR_SIZE << 6)))

static inline u8 DQRR_PTR2IDX(const struct qm_dqrr_entry *e)
{
	return ((uintptr_t)e >> 6) & (QM_DQRR_SIZE - 1);
}

static inline const struct qm_dqrr_entry *DQRR_INC(
						const struct qm_dqrr_entry *e)
{
	return DQRR_CARRYCLEAR(e + 1);
}

static inline void qm_dqrr_set_maxfill(struct qm_portal *portal, u8 mf)
{
	qm_out(CFG, (qm_in(CFG) & 0xff0fffff) |
		((mf & (QM_DQRR_SIZE - 1)) << 20));
}

static inline void qm_dqrr_cci_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cci);
	dqrr->ci = (dqrr->ci + num) & (QM_DQRR_SIZE - 1);
	qm_out(DQRR_CI_CINH, dqrr->ci);
}

static inline void qm_dqrr_cce_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cce);
	dqrr->ci = (dqrr->ci + num) & (QM_DQRR_SIZE - 1);
	qm_cl_out(DQRR_CI, dqrr->ci);
}

static inline void qm_dqrr_cdc_consume_n(struct qm_portal *portal, u16 bitmask)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	qm_out(DQRR_DCAP, (1 << 8) |		/* DQRR_DCAP::S */
		((u32)bitmask << 16));		/* DQRR_DCAP::DCAP_CI */
	dqrr->ci = qm_in(DQRR_CI_CINH) & (QM_DQRR_SIZE - 1);
	dqrr->fill = qm_cyc_diff(QM_DQRR_SIZE, dqrr->ci, dqrr->pi);
}

static inline int qm_dqrr_init(struct qm_portal *portal,
				const struct qm_portal_config *config,
				enum qm_dqrr_dmode dmode,
				__maybe_unused enum qm_dqrr_pmode pmode,
				enum qm_dqrr_cmode cmode, u8 max_fill)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	u32 cfg;

	/* Make sure the DQRR will be idle when we enable */
	qm_out(DQRR_SDQCR, 0);
	qm_out(DQRR_VDQCR, 0);
	qm_out(DQRR_PDQCR, 0);
	dqrr->ring = portal->addr.addr_ce + QM_CL_DQRR;
	dqrr->pi = qm_in(DQRR_PI_CINH) & (QM_DQRR_SIZE - 1);
	dqrr->ci = qm_in(DQRR_CI_CINH) & (QM_DQRR_SIZE - 1);
	dqrr->cursor = dqrr->ring + dqrr->ci;
	dqrr->fill = qm_cyc_diff(QM_DQRR_SIZE, dqrr->ci, dqrr->pi);
	dqrr->vbit = (qm_in(DQRR_PI_CINH) & QM_DQRR_SIZE) ?
			QM_DQRR_VERB_VBIT : 0;
	dqrr->ithresh = qm_in(DQRR_ITR);

	/* Free up pending DQRR entries if any as per current DCM */
	if (dqrr->fill) {
		enum qm_dqrr_cmode dcm = (qm_in(CFG) >> 16) & 3;

#ifdef CONFIG_FSL_DPA_CHECKING
		dqrr->cmode = dcm;
#endif
		switch (dcm) {
		case qm_dqrr_cci:
			qm_dqrr_cci_consume(portal, dqrr->fill);
			break;
		case qm_dqrr_cce:
			qm_dqrr_cce_consume(portal, dqrr->fill);
			break;
		case qm_dqrr_cdc:
			qm_dqrr_cdc_consume_n(portal, (1<<QM_DQRR_SIZE) - 1);
			break;
		default:
			DPA_ASSERT(0);
		}
	}

#ifdef CONFIG_FSL_DPA_CHECKING
	dqrr->dmode = dmode;
	dqrr->pmode = pmode;
	dqrr->cmode = cmode;
#endif
	/* Invalidate every ring entry before beginning */
	for (cfg = 0; cfg < QM_DQRR_SIZE; cfg++)
		dcbi(qm_cl(dqrr->ring, cfg));
	cfg = (qm_in(CFG) & 0xff000f00) |
		((max_fill & (QM_DQRR_SIZE - 1)) << 20) | /* DQRR_MF */
		((dmode & 1) << 18) |			/* DP */
		((cmode & 3) << 16) |			/* DCM */
		0xa0 |					/* RE+SE */
		(0 ? 0x40 : 0) |			/* Ignore RP */
		(0 ? 0x10 : 0);				/* Ignore SP */
	qm_out(CFG, cfg);
	qm_dqrr_set_maxfill(portal, max_fill);

	/* Recalculate cursor as we may have consumed frames */
	dqrr->cursor = dqrr->ring + dqrr->ci;
	return 0;
}

static inline void qm_dqrr_finish(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
#ifdef CONFIG_FSL_DPA_CHECKING
	if ((dqrr->cmode != qm_dqrr_cdc) &&
			(dqrr->ci != DQRR_PTR2IDX(dqrr->cursor)))
		pr_crit("Ignoring completed DQRR entries\n");
#endif
}

static inline const struct qm_dqrr_entry *qm_dqrr_current(
						struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	if (!dqrr->fill)
		return NULL;
	return dqrr->cursor;
}

static inline u8 qm_dqrr_cursor(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	return DQRR_PTR2IDX(dqrr->cursor);
}

static inline u8 qm_dqrr_next(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->fill);
	dqrr->cursor = DQRR_INC(dqrr->cursor);
	return --dqrr->fill;
}

static inline u8 qm_dqrr_pci_update(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	u8 diff, old_pi = dqrr->pi;
	DPA_ASSERT(dqrr->pmode == qm_dqrr_pci);
	dqrr->pi = qm_in(DQRR_PI_CINH) & (QM_DQRR_SIZE - 1);
	diff = qm_cyc_diff(QM_DQRR_SIZE, old_pi, dqrr->pi);
	dqrr->fill += diff;
	return diff;
}

static inline void qm_dqrr_pce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->pmode == qm_dqrr_pce);
	qm_cl_invalidate(DQRR_PI);
	qm_cl_touch_ro(DQRR_PI);
}

static inline u8 qm_dqrr_pce_update(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	u8 diff, old_pi = dqrr->pi;
	DPA_ASSERT(dqrr->pmode == qm_dqrr_pce);
	dqrr->pi = qm_cl_in(DQRR_PI) & (QM_DQRR_SIZE - 1);
	diff = qm_cyc_diff(QM_DQRR_SIZE, old_pi, dqrr->pi);
	dqrr->fill += diff;
	return diff;
}

static inline void qm_dqrr_pvb_update(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	const struct qm_dqrr_entry *res = qm_cl(dqrr->ring, dqrr->pi);
	DPA_ASSERT(dqrr->pmode == qm_dqrr_pvb);
#if (defined CONFIG_PPC || defined CONFIG_PPC64) && !defined CONFIG_FSL_PAMU
	/*
	 * On PowerPC platforms if PAMU is not available we need to
	 * manually invalidate the cache. When PAMU is available the
	 * cache is updated by stashing operations generated by QMan
	 */
	dcbi(res);
	dcbt_ro(res);
#endif

	/* when accessing 'verb', use __raw_readb() to ensure that compiler
	 * inlining doesn't try to optimise out "excess reads". */
	if ((__raw_readb(&res->verb) & QM_DQRR_VERB_VBIT) == dqrr->vbit) {
		dqrr->pi = (dqrr->pi + 1) & (QM_DQRR_SIZE - 1);
		if (!dqrr->pi)
			dqrr->vbit ^= QM_DQRR_VERB_VBIT;
		dqrr->fill++;
	}
}


static inline void qm_dqrr_cci_consume_to_current(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cci);
	dqrr->ci = DQRR_PTR2IDX(dqrr->cursor);
	qm_out(DQRR_CI_CINH, dqrr->ci);
}

static inline void qm_dqrr_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cce);
	qm_cl_invalidate(DQRR_CI);
	qm_cl_touch_rw(DQRR_CI);
}

static inline void qm_dqrr_cce_consume_to_current(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cce);
	dqrr->ci = DQRR_PTR2IDX(dqrr->cursor);
	qm_cl_out(DQRR_CI, dqrr->ci);
}

static inline void qm_dqrr_cdc_consume_1(struct qm_portal *portal, u8 idx,
					int park)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	DPA_ASSERT(idx < QM_DQRR_SIZE);
	qm_out(DQRR_DCAP, (0 << 8) |	/* S */
		((park ? 1 : 0) << 6) |	/* PK */
		idx);			/* DCAP_CI */
}

static inline void qm_dqrr_cdc_consume_1ptr(struct qm_portal *portal,
					const struct qm_dqrr_entry *dq,
					int park)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	u8 idx = DQRR_PTR2IDX(dq);
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	DPA_ASSERT((dqrr->ring + idx) == dq);
	DPA_ASSERT(idx < QM_DQRR_SIZE);
	qm_out(DQRR_DCAP, (0 << 8) |		/* DQRR_DCAP::S */
		((park ? 1 : 0) << 6) |		/* DQRR_DCAP::PK */
		idx);				/* DQRR_DCAP::DCAP_CI */
}

static inline u8 qm_dqrr_cdc_cci(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	return qm_in(DQRR_CI_CINH) & (QM_DQRR_SIZE - 1);
}

static inline void qm_dqrr_cdc_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	qm_cl_invalidate(DQRR_CI);
	qm_cl_touch_ro(DQRR_CI);
}

static inline u8 qm_dqrr_cdc_cce(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	return qm_cl_in(DQRR_CI) & (QM_DQRR_SIZE - 1);
}

static inline u8 qm_dqrr_get_ci(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode != qm_dqrr_cdc);
	return dqrr->ci;
}

static inline void qm_dqrr_park(struct qm_portal *portal, u8 idx)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode != qm_dqrr_cdc);
	qm_out(DQRR_DCAP, (0 << 8) |		/* S */
		(1 << 6) |			/* PK */
		(idx & (QM_DQRR_SIZE - 1)));	/* DCAP_CI */
}

static inline void qm_dqrr_park_current(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	DPA_ASSERT(dqrr->cmode != qm_dqrr_cdc);
	qm_out(DQRR_DCAP, (0 << 8) |		/* S */
		(1 << 6) |			/* PK */
		DQRR_PTR2IDX(dqrr->cursor));	/* DCAP_CI */
}

static inline void qm_dqrr_sdqcr_set(struct qm_portal *portal, u32 sdqcr)
{
	qm_out(DQRR_SDQCR, sdqcr);
}

static inline u32 qm_dqrr_sdqcr_get(struct qm_portal *portal)
{
	return qm_in(DQRR_SDQCR);
}

static inline void qm_dqrr_vdqcr_set(struct qm_portal *portal, u32 vdqcr)
{
	qm_out(DQRR_VDQCR, vdqcr);
}

static inline u32 qm_dqrr_vdqcr_get(struct qm_portal *portal)
{
	return qm_in(DQRR_VDQCR);
}

static inline void qm_dqrr_pdqcr_set(struct qm_portal *portal, u32 pdqcr)
{
	qm_out(DQRR_PDQCR, pdqcr);
}

static inline u32 qm_dqrr_pdqcr_get(struct qm_portal *portal)
{
	return qm_in(DQRR_PDQCR);
}

static inline u8 qm_dqrr_get_ithresh(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	return dqrr->ithresh;
}

static inline void qm_dqrr_set_ithresh(struct qm_portal *portal, u8 ithresh)
{
	qm_out(DQRR_ITR, ithresh);
}

static inline u8 qm_dqrr_get_maxfill(struct qm_portal *portal)
{
	return (qm_in(CFG) & 0x00f00000) >> 20;
}


/* -------------- */
/* --- MR API --- */

#define MR_CARRYCLEAR(p) \
	(void *)((unsigned long)(p) & (~(unsigned long)(QM_MR_SIZE << 6)))

static inline u8 MR_PTR2IDX(const struct qm_mr_entry *e)
{
	return ((uintptr_t)e >> 6) & (QM_MR_SIZE - 1);
}

static inline const struct qm_mr_entry *MR_INC(const struct qm_mr_entry *e)
{
	return MR_CARRYCLEAR(e + 1);
}

static inline int qm_mr_init(struct qm_portal *portal, enum qm_mr_pmode pmode,
		enum qm_mr_cmode cmode)
{
	register struct qm_mr *mr = &portal->mr;
	u32 cfg;

	mr->ring = portal->addr.addr_ce + QM_CL_MR;
	mr->pi = qm_in(MR_PI_CINH) & (QM_MR_SIZE - 1);
	mr->ci = qm_in(MR_CI_CINH) & (QM_MR_SIZE - 1);
	mr->cursor = mr->ring + mr->ci;
	mr->fill = qm_cyc_diff(QM_MR_SIZE, mr->ci, mr->pi);
	mr->vbit = (qm_in(MR_PI_CINH) & QM_MR_SIZE) ? QM_MR_VERB_VBIT : 0;
	mr->ithresh = qm_in(MR_ITR);
#ifdef CONFIG_FSL_DPA_CHECKING
	mr->pmode = pmode;
	mr->cmode = cmode;
#endif
	cfg = (qm_in(CFG) & 0xfffff0ff) |
		((cmode & 1) << 8);		/* QCSP_CFG:MM */
	qm_out(CFG, cfg);
	return 0;
}

static inline void qm_mr_finish(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	if (mr->ci != MR_PTR2IDX(mr->cursor))
		pr_crit("Ignoring completed MR entries\n");
}

static inline const struct qm_mr_entry *qm_mr_current(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	if (!mr->fill)
		return NULL;
	return mr->cursor;
}

static inline u8 qm_mr_cursor(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	return MR_PTR2IDX(mr->cursor);
}

static inline u8 qm_mr_next(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->fill);
	mr->cursor = MR_INC(mr->cursor);
	return --mr->fill;
}

static inline u8 qm_mr_pci_update(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	u8 diff, old_pi = mr->pi;
	DPA_ASSERT(mr->pmode == qm_mr_pci);
	mr->pi = qm_in(MR_PI_CINH);
	diff = qm_cyc_diff(QM_MR_SIZE, old_pi, mr->pi);
	mr->fill += diff;
	return diff;
}

static inline void qm_mr_pce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->pmode == qm_mr_pce);
	qm_cl_invalidate(MR_PI);
	qm_cl_touch_ro(MR_PI);
}

static inline u8 qm_mr_pce_update(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	u8 diff, old_pi = mr->pi;
	DPA_ASSERT(mr->pmode == qm_mr_pce);
	mr->pi = qm_cl_in(MR_PI) & (QM_MR_SIZE - 1);
	diff = qm_cyc_diff(QM_MR_SIZE, old_pi, mr->pi);
	mr->fill += diff;
	return diff;
}

static inline void qm_mr_pvb_update(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	const struct qm_mr_entry *res = qm_cl(mr->ring, mr->pi);
	DPA_ASSERT(mr->pmode == qm_mr_pvb);
	/* when accessing 'verb', use __raw_readb() to ensure that compiler
	 * inlining doesn't try to optimise out "excess reads". */
	if ((__raw_readb(&res->verb) & QM_MR_VERB_VBIT) == mr->vbit) {
		mr->pi = (mr->pi + 1) & (QM_MR_SIZE - 1);
		if (!mr->pi)
			mr->vbit ^= QM_MR_VERB_VBIT;
		mr->fill++;
		res = MR_INC(res);
	}
	dcbit_ro(res);
}

static inline void qm_mr_cci_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->cmode == qm_mr_cci);
	mr->ci = (mr->ci + num) & (QM_MR_SIZE - 1);
	qm_out(MR_CI_CINH, mr->ci);
}

static inline void qm_mr_cci_consume_to_current(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->cmode == qm_mr_cci);
	mr->ci = MR_PTR2IDX(mr->cursor);
	qm_out(MR_CI_CINH, mr->ci);
}

static inline void qm_mr_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->cmode == qm_mr_cce);
	qm_cl_invalidate(MR_CI);
	qm_cl_touch_rw(MR_CI);
}

static inline void qm_mr_cce_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->cmode == qm_mr_cce);
	mr->ci = (mr->ci + num) & (QM_MR_SIZE - 1);
	qm_cl_out(MR_CI, mr->ci);
}

static inline void qm_mr_cce_consume_to_current(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	DPA_ASSERT(mr->cmode == qm_mr_cce);
	mr->ci = MR_PTR2IDX(mr->cursor);
	qm_cl_out(MR_CI, mr->ci);
}

static inline u8 qm_mr_get_ci(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	return mr->ci;
}

static inline u8 qm_mr_get_ithresh(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	return mr->ithresh;
}

static inline void qm_mr_set_ithresh(struct qm_portal *portal, u8 ithresh)
{
	qm_out(MR_ITR, ithresh);
}


/* ------------------------------ */
/* --- Management command API --- */

static inline int qm_mc_init(struct qm_portal *portal)
{
	u8 rr0, rr1;
	register struct qm_mc *mc = &portal->mc;

	mc->cr = portal->addr.addr_ce + QM_CL_CR;
	mc->rr = portal->addr.addr_ce + QM_CL_RR0;

	/*
	 * The expected valid bit polarity for the next CR command is 0
	 * if RR1 contains a valid response, and is 1 if RR0 contains a
	 * valid response. If both RR contain all 0, this indicates either
	 * that no command has been executed since reset (in which case the
	 * expected valid bit polarity is 1)
	 */
	rr0 = __raw_readb(&mc->rr->verb);
	rr1 = __raw_readb(&(mc->rr+1)->verb);
	if ((rr0 == 0 && rr1 == 0) || rr0 != 0)
		mc->rridx = 1;
	else
		mc->rridx = 0;

	mc->vbit = mc->rridx ? QM_MCC_VERB_VBIT : 0;
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_idle;
#endif
	return 0;
}

static inline void qm_mc_finish(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mc *mc = &portal->mc;
	DPA_ASSERT(mc->state == qman_mc_idle);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (mc->state != qman_mc_idle)
		pr_crit("Losing incomplete MC command\n");
#endif
}

static inline struct qm_mc_command *qm_mc_start(struct qm_portal *portal)
{
	register struct qm_mc *mc = &portal->mc;
	DPA_ASSERT(mc->state == qman_mc_idle);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_user;
#endif
#if defined(CONFIG_PPC32) || defined(CONFIG_PPC64)
	dcbz_64(mc->cr);
#endif
	return mc->cr;
}

static inline void qm_mc_abort(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mc *mc = &portal->mc;
	DPA_ASSERT(mc->state == qman_mc_user);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_idle;
#endif
}

static inline void qm_mc_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_mc *mc = &portal->mc;
	struct qm_mc_result *rr = mc->rr + mc->rridx;
	DPA_ASSERT(mc->state == qman_mc_user);
	lwsync();
	mc->cr->__dont_write_directly__verb = myverb | mc->vbit;
	dcbf(mc->cr);
	dcbit_ro(rr);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_hw;
#endif
}

static inline struct qm_mc_result *qm_mc_result(struct qm_portal *portal)
{
	register struct qm_mc *mc = &portal->mc;
	struct qm_mc_result *rr = mc->rr + mc->rridx;
	DPA_ASSERT(mc->state == qman_mc_hw);
	/* The inactive response register's verb byte always returns zero until
	 * its command is submitted and completed. This includes the valid-bit,
	 * in case you were wondering... */
	if (!__raw_readb(&rr->verb)) {
		dcbit_ro(rr);
		return NULL;
	}
	mc->rridx ^= 1;
	mc->vbit ^= QM_MCC_VERB_VBIT;
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_idle;
#endif
	return rr;
}


/* ------------------------------------- */
/* --- Portal interrupt register API --- */

static inline int qm_isr_init(__always_unused struct qm_portal *portal)
{
	return 0;
}

static inline void qm_isr_finish(__always_unused struct qm_portal *portal)
{
}

static inline void qm_isr_set_iperiod(struct qm_portal *portal, u16 iperiod)
{
	qm_out(ITPR, iperiod);
}

static inline u32 __qm_isr_read(struct qm_portal *portal, enum qm_isr_reg n)
{
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	return __qm_in(&portal->addr, QM_REG_ISR + (n << 6));
#else
	return __qm_in(&portal->addr, QM_REG_ISR + (n << 2));
#endif
}

static inline void __qm_isr_write(struct qm_portal *portal, enum qm_isr_reg n,
					u32 val)
{
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	__qm_out(&portal->addr, QM_REG_ISR + (n << 6), val);
#else
	__qm_out(&portal->addr, QM_REG_ISR + (n << 2), val);
#endif
}

static inline int _qm_mr_consume_and_match_verb(struct qm_portal *p, int v)
{
	const struct qm_mr_entry *msg;
	int found = 0;

	qm_mr_pvb_update(p);
	msg = qm_mr_current(p);
	while (msg) {
		if ((msg->verb & QM_MR_VERB_TYPE_MASK) == v)
			found = 1;
		qm_mr_next(p);
		qm_mr_cci_consume_to_current(p);
		qm_mr_pvb_update(p);
		msg = qm_mr_current(p);
	}
	return found;
}

static inline int _qm_dqrr_consume_and_match(struct qm_portal *p, u32 fqid,
					     int s, bool wait)
{
	const struct qm_dqrr_entry *dqrr;
	int found = 0;

	do {
		qm_dqrr_pvb_update(p);
		dqrr = qm_dqrr_current(p);
		if (!dqrr)
			cpu_relax();
	} while (wait && !dqrr);

	while (dqrr) {
		if (qm_fqid_get(dqrr) == fqid && (dqrr->stat & s))
			found = 1;
		qm_dqrr_cdc_consume_1ptr(p, dqrr, 0);
		qm_dqrr_pvb_update(p);
		qm_dqrr_next(p);
		dqrr = qm_dqrr_current(p);
	}
	return found;
}

#define qm_mr_drain(p, V) \
	_qm_mr_consume_and_match_verb(p, QM_MR_VERB_##V)

#define qm_dqrr_drain(p, f, S) \
	_qm_dqrr_consume_and_match(p, f, QM_DQRR_STAT_##S, false)

#define qm_dqrr_drain_wait(p, f, S) \
	_qm_dqrr_consume_and_match(p, f, QM_DQRR_STAT_##S, true)

#define qm_dqrr_drain_nomatch(p) \
	_qm_dqrr_consume_and_match(p, 0, 0, false)

/* Cleanup FQs */
static inline int qm_shutdown_fq(struct qm_portal **portal, int portal_count,
				 u32 fqid)
{
	struct qm_portal *channel_portal;
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	int orl_empty, drain = 0;
	bool drained = false;
	unsigned long start;
	u32 old_sdqcr;
	bool timeout;
	u8 state;
	u32 result;
	u32 channel;
	u16 dest_wq;
	int err;

	/* Determine the state of the FQID */
	mcc = qm_mc_start(portal[0]);
	mcc->queryfq_np.fqid = cpu_to_be32(fqid);
	qm_mc_commit(portal[0], QM_MCC_VERB_QUERYFQ_NP);

	err = read_poll_timeout_atomic(qm_mc_result, mcr, mcr, QM_MC_DELAY_US,
				       QM_MC_TIMEOUT_US, false, portal[0]);
	if (err) {
		pr_err("QMan: Failed to query FQ non-programmable fields for FQID %u: %pe\n",
		       fqid, ERR_PTR(err));
		return err;
	}

	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_QUERYFQ_NP);
	state = mcr->queryfq_np.state & QM_MCR_NP_STATE_MASK;
	if (state == QM_MCR_NP_STATE_OOS)
		return 0; /* Already OOS, no need to do anymore checks */

	/* Query which channel the FQ is using */
	mcc = qm_mc_start(portal[0]);
	mcc->queryfq.fqid = cpu_to_be32(fqid);
	qm_mc_commit(portal[0], QM_MCC_VERB_QUERYFQ);

	err = read_poll_timeout_atomic(qm_mc_result, mcr, mcr, QM_MC_DELAY_US,
				       QM_MC_TIMEOUT_US, false, portal[0]);
	if (err) {
		pr_err("QMan: Failed to query FQ programmable fields for FQID %u: %pe\n",
		       fqid, ERR_PTR(err));
		return err;
	}

	DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) == QM_MCR_VERB_QUERYFQ);

	/* Need to store these since the MCR gets reused */
	dest_wq = be16_to_cpu(mcr->queryfq.fqd.dest_wq);
	channel = dest_wq>>3;

	if (channel < qm_channel_pool1) {
		channel_portal = qm_get_portal_for_channel(channel);
		if (channel_portal == NULL) {
			pr_err("QMan: Can't find portal for dedicated channel 0x%x\n",
			       channel);
			return -EIO;
		}
	} else {
		channel_portal = portal[0];
	}

	switch (state) {
	case QM_MCR_NP_STATE_TEN_SCHED:
	case QM_MCR_NP_STATE_TRU_SCHED:
	case QM_MCR_NP_STATE_ACTIVE:
	case QM_MCR_NP_STATE_PARKED:
		orl_empty = 0;
		mcc = qm_mc_start(channel_portal);
		mcc->alterfq.fqid = cpu_to_be32(fqid);
		qm_mc_commit(channel_portal, QM_MCC_VERB_ALTER_RETIRE);

		err = read_poll_timeout_atomic(qm_mc_result, mcr, mcr,
					       QM_MC_DELAY_US, QM_MC_TIMEOUT_US,
					       false, channel_portal);
		if (err) {
			pr_err("QMan: Failed to retire FQID %u: %pe\n", fqid,
			       ERR_PTR(err));
			return err;
		}

		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
			   QM_MCR_VERB_ALTER_RETIRE);
		result = mcr->result; /* Make a copy as we reuse MCR below */

		if (result == QM_MCR_RESULT_OK)
			drain_mr_fqrni(channel_portal);

		if (result == QM_MCR_RESULT_PENDING) {
			/* Need to wait for the FQRN in the message ring, which
			   will only occur once the FQ has been drained.  In
			   order for the FQ to drain the portal needs to be set
			   to dequeue from the channel the FQ is scheduled on */
			int found_fqrn = 0;

			/* Flag that we need to drain FQ */
			drain = 1;

			if (channel >= (qm_channel_pool1 + 15)) {
				pr_info("Cannot recover FQ 0x%x, it is "
					"scheduled on channel 0x%x",
					fqid, channel);
				return -EBUSY;
			}
			/* Set the sdqcr to drain this channel */
			if (channel < qm_channel_pool1) {
				qm_dqrr_sdqcr_set(channel_portal,
						  QM_SDQCR_TYPE_ACTIVE |
						  QM_SDQCR_CHANNELS_DEDICATED);
			} else {
				qm_dqrr_sdqcr_set(channel_portal,
						  QM_SDQCR_TYPE_ACTIVE |
						  QM_SDQCR_CHANNELS_POOL_CONV(channel));
			}
			start = jiffies;
			timeout = false;
			while (!found_fqrn && !timeout) {
				/* Keep draining DQRR while checking the MR*/
				qm_dqrr_drain_nomatch(channel_portal);
				/* Process message ring too */
				found_fqrn = qm_mr_drain(channel_portal, FQRN);
				timeout = jiffies > start + FQRN_WAIT_TIMEOUT;
				cpu_relax();
			}
			if (timeout) {
				pr_err("QMan: timed out waiting for retire notification on FQID %u\n",
				       fqid);
			}

			/* Restore SDQCR */
			old_sdqcr = qm_portal_get_sdqcr(channel_portal);
			qm_dqrr_sdqcr_set(channel_portal, old_sdqcr);
		}
		if (result != QM_MCR_RESULT_OK &&
		    result !=  QM_MCR_RESULT_PENDING) {
			/* error */
			pr_err("qman_retire_fq failed on FQ 0x%x, result=0x%x\n",
			       fqid, result);
			return -1;
		}
		if (!(mcr->alterfq.fqs & QM_MCR_FQS_ORLPRESENT)) {
			/* ORL had no entries, no need to wait until the
			   ERNs come in */
			orl_empty = 1;
		}
		/* Retirement succeeded, check to see if FQ needs
		   to be drained */
		if (drain || mcr->alterfq.fqs & QM_MCR_FQS_NOTEMPTY) {
			u32 vdqcr = fqid | QM_VDQCR_NUMFRAMES_TILLEMPTY;
			bool fq_empty;

			/* FQ is Not Empty, drain using volatile DQ commands */
			qm_dqrr_vdqcr_set(portal[0], vdqcr);
			/*
			 * Wait for a dequeue and process the dequeues,
			 * making sure to empty the ring completely
			 */
			do {
				fq_empty = qm_dqrr_drain_wait(portal[0], fqid, DQCR_EXPIRED);
			} while (!fq_empty);
		}

		/* Wait for the ORL to have been completely drained */
		while (orl_empty == 0) {
			orl_empty = qm_mr_drain(portal[0], FQRL);
			cpu_relax();
		}

		drained = true;
		fallthrough;
	case QM_MCR_NP_STATE_RETIRED:
		/* Send OOS Command */
		mcc = qm_mc_start(portal[0]);
		mcc->alterfq.fqid = cpu_to_be32(fqid);
		qm_mc_commit(portal[0], QM_MCC_VERB_ALTER_OOS);

		err = read_poll_timeout_atomic(qm_mc_result, mcr, mcr,
					       QM_MC_DELAY_US, QM_MC_TIMEOUT_US,
					       false, portal[0]);
		if (err) {
			pr_err("QMan: Failed to put FQID %u out of service: %pe\n",
			       fqid, ERR_PTR(err));
			return err;
		}

		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK) ==
			   QM_MCR_VERB_ALTER_OOS);
		if (mcr->result != QM_MCR_RESULT_OK) {
			pr_err("OOS%s failed on FQID 0x%x\n",
			       drained ? " after drain" : "", fqid);
			return -1;
		}
		return 0;
	}
	return -1;
}
