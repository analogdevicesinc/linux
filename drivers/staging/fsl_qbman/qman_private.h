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

#include "dpa_sys.h"
#include <linux/fsl_qman.h>
#include <linux/iommu.h>

#if defined(CONFIG_FSL_PAMU)
#include <asm/fsl_pamu_stash.h>
#endif

#if !defined(CONFIG_FSL_QMAN_FQ_LOOKUP) && defined(CONFIG_PPC64)
#error "_PPC64 requires _FSL_QMAN_FQ_LOOKUP"
#endif

#define QBMAN_ANY_PORTAL_IDX 0xffffffff
	/* ----------------- */
	/* Congestion Groups */
	/* ----------------- */
/* This wrapper represents a bit-array for the state of the 256 Qman congestion
 * groups. Is also used as a *mask* for congestion groups, eg. so we ignore
 * those that don't concern us. We harness the structure and accessor details
 * already used in the management command to query congestion groups. */
struct qman_cgrs {
	struct __qm_mcr_querycongestion q;
};
static inline void qman_cgrs_init(struct qman_cgrs *c)
{
	memset(c, 0, sizeof(*c));
}
static inline void qman_cgrs_fill(struct qman_cgrs *c)
{
	memset(c, 0xff, sizeof(*c));
}
static inline int qman_cgrs_get(struct qman_cgrs *c, int num)
{
	return QM_MCR_QUERYCONGESTION(&c->q, num);
}
static inline void qman_cgrs_set(struct qman_cgrs *c, int num)
{
	c->q.__state[__CGR_WORD(num)] |= (0x80000000 >> __CGR_SHIFT(num));
}
static inline void qman_cgrs_unset(struct qman_cgrs *c, int num)
{
	c->q.__state[__CGR_WORD(num)] &= ~(0x80000000 >> __CGR_SHIFT(num));
}
static inline int qman_cgrs_next(struct qman_cgrs *c, int num)
{
	while ((++num < __CGR_NUM) && !qman_cgrs_get(c, num))
		;
	return num;
}
static inline void qman_cgrs_cp(struct qman_cgrs *dest,
				const struct qman_cgrs *src)
{
	*dest = *src;
}
static inline void qman_cgrs_and(struct qman_cgrs *dest,
			const struct qman_cgrs *a, const struct qman_cgrs *b)
{
	int ret;
	u32 *_d = dest->q.__state;
	const u32 *_a = a->q.__state;
	const u32 *_b = b->q.__state;
	for (ret = 0; ret < 8; ret++)
		*(_d++) = *(_a++) & *(_b++);
}
static inline void qman_cgrs_xor(struct qman_cgrs *dest,
			const struct qman_cgrs *a, const struct qman_cgrs *b)
{
	int ret;
	u32 *_d = dest->q.__state;
	const u32 *_a = a->q.__state;
	const u32 *_b = b->q.__state;
	for (ret = 0; ret < 8; ret++)
		*(_d++) = *(_a++) ^ *(_b++);
}

	/* ----------------------- */
	/* CEETM Congestion Groups */
	/* ----------------------- */
/* This wrapper represents a bit-array for the state of the 512 Qman CEETM
 * congestion groups.
 */
struct qman_ccgrs {
	struct __qm_mcr_querycongestion q[2];
};
static inline void qman_ccgrs_init(struct qman_ccgrs *c)
{
	memset(c, 0, sizeof(*c));
}
static inline void qman_ccgrs_fill(struct qman_ccgrs *c)
{
	memset(c, 0xff, sizeof(*c));
}
static inline int qman_ccgrs_get(struct qman_ccgrs *c, int num)
{
	if (num < __CGR_NUM)
		return QM_MCR_QUERYCONGESTION(&c->q[0], num);
	else
		return QM_MCR_QUERYCONGESTION(&c->q[1], (num - __CGR_NUM));
}
static inline int qman_ccgrs_next(struct qman_ccgrs *c, int num)
{
	while ((++num < __CGR_NUM) && !qman_ccgrs_get(c, num))
		;
	return num;
}
static inline void qman_ccgrs_cp(struct qman_ccgrs *dest,
					const struct qman_ccgrs *src)
{
	*dest = *src;
}
static inline void qman_ccgrs_and(struct qman_ccgrs *dest,
			const struct qman_ccgrs *a, const struct qman_ccgrs *b)
{
	int ret, i;
	u32 *_d;
	const u32 *_a, *_b;
	for (i = 0; i < 2; i++) {
		_d = dest->q[i].__state;
		_a = a->q[i].__state;
		_b = b->q[i].__state;
		for (ret = 0; ret < 8; ret++)
			*(_d++) = *(_a++) & *(_b++);
	}
}
static inline void qman_ccgrs_xor(struct qman_ccgrs *dest,
			const struct qman_ccgrs *a, const struct qman_ccgrs *b)
{
	int ret, i;
	u32 *_d;
	const u32 *_a, *_b;
	for (i = 0; i < 2; i++) {
		_d = dest->q[i].__state;
		_a = a->q[i].__state;
		_b = b->q[i].__state;
		for (ret = 0; ret < 8; ret++)
			*(_d++) = *(_a++) ^ *(_b++);
	}
}

/* used by CCSR and portal interrupt code */
enum qm_isr_reg {
	qm_isr_status = 0,
	qm_isr_enable = 1,
	qm_isr_disable = 2,
	qm_isr_inhibit = 3
};

struct qm_portal_config {
	/* Corenet portal addresses;
	 * [0]==cache-enabled, [1]==cache-inhibited. */
	__iomem void *addr_virt[2];
	struct resource addr_phys[2];
	struct device dev;
	struct iommu_domain *iommu_domain;
	/* Allow these to be joined in lists */
	struct list_head list;
	/* User-visible portal configuration settings */
	struct qman_portal_config public_cfg;
	/* power management saved data */
	u32 saved_isdr;
};

/* Revision info (for errata and feature handling) */
#define QMAN_REV11 0x0101
#define QMAN_REV12 0x0102
#define QMAN_REV20 0x0200
#define QMAN_REV30 0x0300
#define QMAN_REV31 0x0301
#define QMAN_REV32 0x0302

/* QMan REV_2 register contains the Cfg option */
#define QMAN_REV_CFG_0 0x0
#define QMAN_REV_CFG_1 0x1
#define QMAN_REV_CFG_2 0x2
#define QMAN_REV_CFG_3 0x3

extern u16 qman_ip_rev; /* 0 if uninitialised, otherwise QMAN_REVx */
extern u8 qman_ip_cfg;
extern u32 qman_clk;
extern u16 qman_portal_max;

#ifdef CONFIG_FSL_QMAN_CONFIG
/* Hooks from qman_driver.c to qman_config.c */
int qman_init_ccsr(struct device_node *node);
void qman_liodn_fixup(u16 channel);
int qman_set_sdest(u16 channel, unsigned int cpu_idx);
size_t get_qman_fqd_size(void);
#else
static inline size_t get_qman_fqd_size(void)
{
	return (PAGE_SIZE << CONFIG_FSL_QMAN_FQD_SZ);
}
#endif

int qm_set_wpm(int wpm);
int qm_get_wpm(int *wpm);

/* Hooks from qman_driver.c in to qman_high.c */
struct qman_portal *qman_create_portal(
			struct qman_portal *portal,
			const struct qm_portal_config *config,
			const struct qman_cgrs *cgrs);

struct qman_portal *qman_create_affine_portal(
			const struct qm_portal_config *config,
			const struct qman_cgrs *cgrs);
struct qman_portal *qman_create_affine_slave(struct qman_portal *redirect,
								int cpu);
const struct qm_portal_config *qman_destroy_affine_portal(void);
void qman_destroy_portal(struct qman_portal *qm);

/* Hooks from fsl_usdpaa.c to qman_driver.c */
struct qm_portal_config *qm_get_unused_portal(void);
struct qm_portal_config *qm_get_unused_portal_idx(uint32_t idx);

void qm_put_unused_portal(struct qm_portal_config *pcfg);
void qm_set_liodns(struct qm_portal_config *pcfg);

/* This CGR feature is supported by h/w and required by unit-tests and the
 * debugfs hooks, so is implemented in the driver. However it allows an explicit
 * corruption of h/w fields by s/w that are usually incorruptible (because the
 * counters are usually maintained entirely within h/w). As such, we declare
 * this API internally. */
int qman_testwrite_cgr(struct qman_cgr *cgr, u64 i_bcnt,
	struct qm_mcr_cgrtestwrite *result);

#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
/* If the fq object pointer is greater than the size of context_b field,
 * than a lookup table is required. */
int qman_setup_fq_lookup_table(size_t num_entries);
#endif


/*************************************************/
/*   QMan s/w corenet portal, low-level i/face   */
/*************************************************/

/* Note: most functions are only used by the high-level interface, so are
 * inlined from qman_low.h. The stuff below is for use by other parts of the
 * driver. */

/* For qm_dqrr_sdqcr_set(); Choose one SOURCE. Choose one COUNT. Choose one
 * dequeue TYPE. Choose TOKEN (8-bit).
 * If SOURCE == CHANNELS,
 *   Choose CHANNELS_DEDICATED and/or CHANNELS_POOL(n).
 *   You can choose DEDICATED_PRECEDENCE if the portal channel should have
 *   priority.
 * If SOURCE == SPECIFICWQ,
 *     Either select the work-queue ID with SPECIFICWQ_WQ(), or select the
 *     channel (SPECIFICWQ_DEDICATED or SPECIFICWQ_POOL()) and specify the
 *     work-queue priority (0-7) with SPECIFICWQ_WQ() - either way, you get the
 *     same value.
 */
#define QM_SDQCR_SOURCE_CHANNELS	0x0
#define QM_SDQCR_SOURCE_SPECIFICWQ	0x40000000
#define QM_SDQCR_COUNT_EXACT1		0x0
#define QM_SDQCR_COUNT_UPTO3		0x20000000
#define QM_SDQCR_DEDICATED_PRECEDENCE	0x10000000
#define QM_SDQCR_TYPE_MASK		0x03000000
#define QM_SDQCR_TYPE_NULL		0x0
#define QM_SDQCR_TYPE_PRIO_QOS		0x01000000
#define QM_SDQCR_TYPE_ACTIVE_QOS	0x02000000
#define QM_SDQCR_TYPE_ACTIVE		0x03000000
#define QM_SDQCR_TOKEN_MASK		0x00ff0000
#define QM_SDQCR_TOKEN_SET(v)		(((v) & 0xff) << 16)
#define QM_SDQCR_TOKEN_GET(v)		(((v) >> 16) & 0xff)
#define QM_SDQCR_CHANNELS_DEDICATED	0x00008000
#define QM_SDQCR_SPECIFICWQ_MASK	0x000000f7
#define QM_SDQCR_SPECIFICWQ_DEDICATED	0x00000000
#define QM_SDQCR_SPECIFICWQ_POOL(n)	((n) << 4)
#define QM_SDQCR_SPECIFICWQ_WQ(n)	(n)

/* For qm_dqrr_vdqcr_set(): use FQID(n) to fill in the frame queue ID */
#define QM_VDQCR_FQID_MASK		0x00ffffff
#define QM_VDQCR_FQID(n)		((n) & QM_VDQCR_FQID_MASK)

/* For qm_dqrr_pdqcr_set(); Choose one MODE. Choose one COUNT.
 * If MODE==SCHEDULED
 *   Choose SCHEDULED_CHANNELS or SCHEDULED_SPECIFICWQ. Choose one dequeue TYPE.
 *   If CHANNELS,
 *     Choose CHANNELS_DEDICATED and/or CHANNELS_POOL() channels.
 *     You can choose DEDICATED_PRECEDENCE if the portal channel should have
 *     priority.
 *   If SPECIFICWQ,
 *     Either select the work-queue ID with SPECIFICWQ_WQ(), or select the
 *     channel (SPECIFICWQ_DEDICATED or SPECIFICWQ_POOL()) and specify the
 *     work-queue priority (0-7) with SPECIFICWQ_WQ() - either way, you get the
 *     same value.
 * If MODE==UNSCHEDULED
 *     Choose FQID().
 */
#define QM_PDQCR_MODE_SCHEDULED		0x0
#define QM_PDQCR_MODE_UNSCHEDULED	0x80000000
#define QM_PDQCR_SCHEDULED_CHANNELS	0x0
#define QM_PDQCR_SCHEDULED_SPECIFICWQ	0x40000000
#define QM_PDQCR_COUNT_EXACT1		0x0
#define QM_PDQCR_COUNT_UPTO3		0x20000000
#define QM_PDQCR_DEDICATED_PRECEDENCE	0x10000000
#define QM_PDQCR_TYPE_MASK		0x03000000
#define QM_PDQCR_TYPE_NULL		0x0
#define QM_PDQCR_TYPE_PRIO_QOS		0x01000000
#define QM_PDQCR_TYPE_ACTIVE_QOS	0x02000000
#define QM_PDQCR_TYPE_ACTIVE		0x03000000
#define QM_PDQCR_CHANNELS_DEDICATED	0x00008000
#define QM_PDQCR_CHANNELS_POOL(n)	(0x00008000 >> (n))
#define QM_PDQCR_SPECIFICWQ_MASK	0x000000f7
#define QM_PDQCR_SPECIFICWQ_DEDICATED	0x00000000
#define QM_PDQCR_SPECIFICWQ_POOL(n)	((n) << 4)
#define QM_PDQCR_SPECIFICWQ_WQ(n)	(n)
#define QM_PDQCR_FQID(n)		((n) & 0xffffff)

/* Used by all portal interrupt registers except 'inhibit'
 * Channels with frame availability
 */
#define QM_PIRQ_DQAVAIL	0x0000ffff

/* The DQAVAIL interrupt fields break down into these bits; */
#define QM_DQAVAIL_PORTAL	0x8000		/* Portal channel */
#define QM_DQAVAIL_POOL(n)	(0x8000 >> (n))	/* Pool channel, n==[1..15] */
#define QM_DQAVAIL_MASK		0xffff
/* This mask contains all the "irqsource" bits visible to API users */
#define QM_PIRQ_VISIBLE	(QM_PIRQ_SLOW | QM_PIRQ_DQRI)

/* These are qm_<reg>_<verb>(). So for example, qm_disable_write() means "write
 * the disable register" rather than "disable the ability to write". */
#define qm_isr_status_read(qm)		__qm_isr_read(qm, qm_isr_status)
#define qm_isr_status_clear(qm, m)	__qm_isr_write(qm, qm_isr_status, m)
#define qm_isr_enable_read(qm)		__qm_isr_read(qm, qm_isr_enable)
#define qm_isr_enable_write(qm, v)	__qm_isr_write(qm, qm_isr_enable, v)
#define qm_isr_disable_read(qm)		__qm_isr_read(qm, qm_isr_disable)
#define qm_isr_disable_write(qm, v)	__qm_isr_write(qm, qm_isr_disable, v)
/* TODO: unfortunate name-clash here, reword? */
#define qm_isr_inhibit(qm)		__qm_isr_write(qm, qm_isr_inhibit, 1)
#define qm_isr_uninhibit(qm)		__qm_isr_write(qm, qm_isr_inhibit, 0)

#ifdef CONFIG_FSL_QMAN_CONFIG
int qman_have_ccsr(void);
#else
#define qman_have_ccsr	0
#endif

__init int qman_init(void);
__init int qman_resource_init(void);

/* CEETM related */
#define QMAN_CEETM_MAX	2
extern u8 num_ceetms;
extern struct qm_ceetm qman_ceetms[QMAN_CEETM_MAX];
int qman_sp_enable_ceetm_mode(enum qm_dc_portal portal, u16 sub_portal);
int qman_sp_disable_ceetm_mode(enum qm_dc_portal portal, u16 sub_portal);
int qman_ceetm_set_prescaler(enum qm_dc_portal portal);
int qman_ceetm_get_prescaler(u16 *pres);
int qman_ceetm_query_cq(unsigned int cqid, unsigned int dcpid,
			struct qm_mcr_ceetm_cq_query *cq_query);
int qman_ceetm_query_ccgr(struct qm_mcc_ceetm_ccgr_query *ccgr_query,
				struct qm_mcr_ceetm_ccgr_query *response);
int qman_ceetm_get_xsfdr(enum qm_dc_portal portal, unsigned int *num);

extern void *affine_portals[NR_CPUS];
const struct qm_portal_config *qman_get_qm_portal_config(
						struct qman_portal *portal);

/* power management */
#ifdef CONFIG_SUSPEND
void suspend_unused_qportal(void);
void resume_unused_qportal(void);
#endif
