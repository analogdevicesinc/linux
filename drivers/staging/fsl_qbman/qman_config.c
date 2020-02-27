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

#include <asm/cacheflush.h>
#include "qman_private.h"
#include <linux/highmem.h>
#include <linux/of_reserved_mem.h>

/* Last updated for v00.800 of the BG */

/* Register offsets */
#define REG_QCSP_LIO_CFG(n)	(0x0000 + ((n) * 0x10))
#define REG_QCSP_IO_CFG(n)	(0x0004 + ((n) * 0x10))
#define REG_QCSP_DD_CFG(n)	(0x000c + ((n) * 0x10))
#define REG_DD_CFG		0x0200
#define REG_DCP_CFG(n)		(0x0300 + ((n) * 0x10))
#define REG_DCP_DD_CFG(n)	(0x0304 + ((n) * 0x10))
#define REG_DCP_DLM_AVG(n)	(0x030c + ((n) * 0x10))
#define REG_PFDR_FPC		0x0400
#define REG_PFDR_FP_HEAD	0x0404
#define REG_PFDR_FP_TAIL	0x0408
#define REG_PFDR_FP_LWIT	0x0410
#define REG_PFDR_CFG		0x0414
#define REG_SFDR_CFG		0x0500
#define REG_SFDR_IN_USE		0x0504
#define REG_WQ_CS_CFG(n)	(0x0600 + ((n) * 0x04))
#define REG_WQ_DEF_ENC_WQID	0x0630
#define REG_WQ_SC_DD_CFG(n)	(0x640 + ((n) * 0x04))
#define REG_WQ_PC_DD_CFG(n)	(0x680 + ((n) * 0x04))
#define REG_WQ_DC0_DD_CFG(n)	(0x6c0 + ((n) * 0x04))
#define REG_WQ_DC1_DD_CFG(n)	(0x700 + ((n) * 0x04))
#define REG_WQ_DCn_DD_CFG(n)	(0x6c0 + ((n) * 0x40)) /* n=2,3 */
#define REG_CM_CFG		0x0800
#define REG_ECSR		0x0a00
#define REG_ECIR		0x0a04
#define REG_EADR		0x0a08
#define REG_ECIR2		0x0a0c
#define REG_EDATA(n)		(0x0a10 + ((n) * 0x04))
#define REG_SBEC(n)		(0x0a80 + ((n) * 0x04))
#define REG_MCR			0x0b00
#define REG_MCP(n)		(0x0b04 + ((n) * 0x04))
#define REG_MISC_CFG		0x0be0
#define REG_HID_CFG		0x0bf0
#define REG_IDLE_STAT		0x0bf4
#define REG_IP_REV_1		0x0bf8
#define REG_IP_REV_2		0x0bfc
#define REG_FQD_BARE		0x0c00
#define REG_PFDR_BARE		0x0c20
#define REG_offset_BAR		0x0004	/* relative to REG_[FQD|PFDR]_BARE */
#define REG_offset_AR		0x0010	/* relative to REG_[FQD|PFDR]_BARE */
#define REG_QCSP_BARE		0x0c80
#define REG_QCSP_BAR		0x0c84
#define REG_CI_SCHED_CFG	0x0d00
#define REG_SRCIDR		0x0d04
#define REG_LIODNR		0x0d08
#define REG_CI_RLM_AVG		0x0d14
#define REG_ERR_ISR		0x0e00	/* + "enum qm_isr_reg" */
#define REG_REV3_QCSP_LIO_CFG(n)	(0x1000 + ((n) * 0x10))
#define REG_REV3_QCSP_IO_CFG(n)	(0x1004 + ((n) * 0x10))
#define REG_REV3_QCSP_DD_CFG(n)	(0x100c + ((n) * 0x10))
#define REG_CEETM_CFG_IDX	0x900
#define REG_CEETM_CFG_PRES	0x904
#define REG_CEETM_XSFDR_IN_USE	0x908

/* Assists for QMAN_MCR */
#define MCR_INIT_PFDR		0x01000000
#define MCR_get_rslt(v)		(u8)((v) >> 24)
#define MCR_rslt_idle(r)	(!rslt || (rslt >= 0xf0))
#define MCR_rslt_ok(r)		(rslt == 0xf0)
#define MCR_rslt_eaccess(r)	(rslt == 0xf8)
#define MCR_rslt_inval(r)	(rslt == 0xff)

struct qman;

/* Follows WQ_CS_CFG0-5 */
enum qm_wq_class {
	qm_wq_portal = 0,
	qm_wq_pool = 1,
	qm_wq_fman0 = 2,
	qm_wq_fman1 = 3,
	qm_wq_caam = 4,
	qm_wq_pme = 5,
	qm_wq_first = qm_wq_portal,
	qm_wq_last = qm_wq_pme
};

/* Follows FQD_[BARE|BAR|AR] and PFDR_[BARE|BAR|AR] */
enum qm_memory {
	qm_memory_fqd,
	qm_memory_pfdr
};

/* Used by all error interrupt registers except 'inhibit' */
#define QM_EIRQ_CIDE	0x20000000	/* Corenet Initiator Data Error */
#define QM_EIRQ_CTDE	0x10000000	/* Corenet Target Data Error */
#define QM_EIRQ_CITT	0x08000000	/* Corenet Invalid Target Transaction */
#define QM_EIRQ_PLWI	0x04000000	/* PFDR Low Watermark */
#define QM_EIRQ_MBEI	0x02000000	/* Multi-bit ECC Error */
#define QM_EIRQ_SBEI	0x01000000	/* Single-bit ECC Error */
#define QM_EIRQ_PEBI	0x00800000	/* PFDR Enqueues Blocked Interrupt */
#define QM_EIRQ_IFSI	0x00020000	/* Invalid FQ Flow Control State */
#define QM_EIRQ_ICVI	0x00010000	/* Invalid Command Verb */
#define QM_EIRQ_IDDI	0x00000800	/* Invalid Dequeue (Direct-connect) */
#define QM_EIRQ_IDFI	0x00000400	/* Invalid Dequeue FQ */
#define QM_EIRQ_IDSI	0x00000200	/* Invalid Dequeue Source */
#define QM_EIRQ_IDQI	0x00000100	/* Invalid Dequeue Queue */
#define QM_EIRQ_IECE	0x00000010	/* Invalid Enqueue Configuration */
#define QM_EIRQ_IEOI	0x00000008	/* Invalid Enqueue Overflow */
#define QM_EIRQ_IESI	0x00000004	/* Invalid Enqueue State */
#define QM_EIRQ_IECI	0x00000002	/* Invalid Enqueue Channel */
#define QM_EIRQ_IEQI	0x00000001	/* Invalid Enqueue Queue */

/* QMAN_ECIR valid error bit */
#define PORTAL_ECSR_ERR	(QM_EIRQ_IEQI | QM_EIRQ_IESI | QM_EIRQ_IEOI | \
				QM_EIRQ_IDQI | QM_EIRQ_IDSI | QM_EIRQ_IDFI | \
				QM_EIRQ_IDDI | QM_EIRQ_ICVI | QM_EIRQ_IFSI)
#define FQID_ECSR_ERR	(QM_EIRQ_IEQI | QM_EIRQ_IECI | QM_EIRQ_IESI | \
			QM_EIRQ_IEOI | QM_EIRQ_IDQI | QM_EIRQ_IDFI | \
			QM_EIRQ_IFSI)

union qman_ecir {
	u32 ecir_raw;
	struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		u32 __reserved:2;
		u32 portal_type:1;
		u32 portal_num:5;
		u32 fqid:24;
#else
		u32 fqid:24;
		u32 portal_num:5;
		u32 portal_type:1;
		u32 __reserved:2;
#endif
	} __packed info;
};

union qman_ecir2 {
	u32 ecir2_raw;
	struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		u32 portal_type:1;
		u32 __reserved:21;
		u32 portal_num:10;
#else
		u32 portal_num:10;
		u32 __reserved:21;
		u32 portal_type:1;
#endif
	} __packed info;
};

union qman_eadr {
	u32 eadr_raw;
	struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		u32 __reserved1:4;
		u32 memid:4;
		u32 __reserved2:12;
		u32 eadr:12;
#else
		u32 eadr:12;
		u32 __reserved2:12;
		u32 memid:4;
		u32 __reserved1:4;
#endif
	} __packed info;
	struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		u32 __reserved1:3;
		u32 memid:5;
		u32 __reserved:8;
		u32 eadr:16;
#else
		u32 eadr:16;
		u32 __reserved:8;
		u32 memid:5;
		u32 __reserved1:3;
#endif
	} __packed info_rev3;
};

struct qman_hwerr_txt {
	u32 mask;
	const char *txt;
};

#define QMAN_HWE_TXT(a, b) { .mask = QM_EIRQ_##a, .txt = b }

static const struct qman_hwerr_txt qman_hwerr_txts[] = {
	QMAN_HWE_TXT(CIDE, "Corenet Initiator Data Error"),
	QMAN_HWE_TXT(CTDE, "Corenet Target Data Error"),
	QMAN_HWE_TXT(CITT, "Corenet Invalid Target Transaction"),
	QMAN_HWE_TXT(PLWI, "PFDR Low Watermark"),
	QMAN_HWE_TXT(MBEI, "Multi-bit ECC Error"),
	QMAN_HWE_TXT(SBEI, "Single-bit ECC Error"),
	QMAN_HWE_TXT(PEBI, "PFDR Enqueues Blocked Interrupt"),
	QMAN_HWE_TXT(ICVI, "Invalid Command Verb"),
	QMAN_HWE_TXT(IFSI, "Invalid Flow Control State"),
	QMAN_HWE_TXT(IDDI, "Invalid Dequeue (Direct-connect)"),
	QMAN_HWE_TXT(IDFI, "Invalid Dequeue FQ"),
	QMAN_HWE_TXT(IDSI, "Invalid Dequeue Source"),
	QMAN_HWE_TXT(IDQI, "Invalid Dequeue Queue"),
	QMAN_HWE_TXT(IECE, "Invalid Enqueue Configuration"),
	QMAN_HWE_TXT(IEOI, "Invalid Enqueue Overflow"),
	QMAN_HWE_TXT(IESI, "Invalid Enqueue State"),
	QMAN_HWE_TXT(IECI, "Invalid Enqueue Channel"),
	QMAN_HWE_TXT(IEQI, "Invalid Enqueue Queue")
};
#define QMAN_HWE_COUNT (sizeof(qman_hwerr_txts)/sizeof(struct qman_hwerr_txt))

struct qman_error_info_mdata {
	u16 addr_mask;
	u16 bits;
	const char *txt;
};

#define QMAN_ERR_MDATA(a, b, c) { .addr_mask = a, .bits = b, .txt = c}
static const struct qman_error_info_mdata error_mdata[] = {
	QMAN_ERR_MDATA(0x01FF, 24, "FQD cache tag memory 0"),
	QMAN_ERR_MDATA(0x01FF, 24, "FQD cache tag memory 1"),
	QMAN_ERR_MDATA(0x01FF, 24, "FQD cache tag memory 2"),
	QMAN_ERR_MDATA(0x01FF, 24, "FQD cache tag memory 3"),
	QMAN_ERR_MDATA(0x0FFF, 512, "FQD cache memory"),
	QMAN_ERR_MDATA(0x07FF, 128, "SFDR memory"),
	QMAN_ERR_MDATA(0x01FF, 72, "WQ context memory"),
	QMAN_ERR_MDATA(0x00FF, 240, "CGR memory"),
	QMAN_ERR_MDATA(0x00FF, 302, "Internal Order Restoration List memory"),
	QMAN_ERR_MDATA(0x7FFF, 256, "SW portal ring memory"),
	QMAN_ERR_MDATA(0x07FF, 181, "CEETM class queue descriptor memory"),
	QMAN_ERR_MDATA(0x0FFF, 140, "CEETM extended SFDR memory"),
	QMAN_ERR_MDATA(0x0FFF, 25, "CEETM logical FQ mapping memory"),
	QMAN_ERR_MDATA(0x0FFF, 96, "CEETM dequeue context memory"),
	QMAN_ERR_MDATA(0x07FF, 396, "CEETM ccgr memory"),
	QMAN_ERR_MDATA(0x00FF, 146, "CEETM CQ channel shaping memory"),
	QMAN_ERR_MDATA(0x007F, 256, "CEETM CQ channel scheduling memory"),
	QMAN_ERR_MDATA(0x01FF, 88, "CEETM dequeue statistics memory"),
};
#define QMAN_ERR_MDATA_COUNT \
	(sizeof(error_mdata)/sizeof(struct qman_error_info_mdata))

/* Add this in Kconfig */
#define QMAN_ERRS_TO_UNENABLE (QM_EIRQ_PLWI | QM_EIRQ_PEBI)

/**
 * qm_err_isr_<reg>_<verb> - Manipulate global interrupt registers
 * @v: for accessors that write values, this is the 32-bit value
 *
 * Manipulates QMAN_ERR_ISR, QMAN_ERR_IER, QMAN_ERR_ISDR, QMAN_ERR_IIR. All
 * manipulations except qm_err_isr_[un]inhibit() use 32-bit masks composed of
 * the QM_EIRQ_*** definitions. Note that "qm_err_isr_enable_write" means
 * "write the enable register" rather than "enable the write register"!
 */
#define qm_err_isr_status_read(qm)	\
		__qm_err_isr_read(qm, qm_isr_status)
#define qm_err_isr_status_clear(qm, m)	\
		__qm_err_isr_write(qm, qm_isr_status, m)
#define qm_err_isr_enable_read(qm)	\
		__qm_err_isr_read(qm, qm_isr_enable)
#define qm_err_isr_enable_write(qm, v)	\
		__qm_err_isr_write(qm, qm_isr_enable, v)
#define qm_err_isr_disable_read(qm)	\
		__qm_err_isr_read(qm, qm_isr_disable)
#define qm_err_isr_disable_write(qm, v)	\
		__qm_err_isr_write(qm, qm_isr_disable, v)
#define qm_err_isr_inhibit(qm)		\
		__qm_err_isr_write(qm, qm_isr_inhibit, 1)
#define qm_err_isr_uninhibit(qm)	\
		__qm_err_isr_write(qm, qm_isr_inhibit, 0)

/*
 * TODO: unimplemented registers
 *
 * Keeping a list here of Qman registers I have not yet covered;
 * QCSP_DD_IHRSR, QCSP_DD_IHRFR, QCSP_DD_HASR,
 * DCP_DD_IHRSR, DCP_DD_IHRFR, DCP_DD_HASR, CM_CFG,
 * QMAN_EECC, QMAN_SBET, QMAN_EINJ, QMAN_SBEC0-12
 */

/* Encapsulate "struct qman *" as a cast of the register space address. */

static struct qman *qm_create(void *regs)
{
	return (struct qman *)regs;
}

static inline u32 __qm_in(struct qman *qm, u32 offset)
{
	return in_be32((void *)qm + offset);
}
static inline void __qm_out(struct qman *qm, u32 offset, u32 val)
{
	out_be32((void *)qm + offset, val);
}
#define qm_in(reg)		__qm_in(qm, REG_##reg)
#define qm_out(reg, val)	__qm_out(qm, REG_##reg, val)

static u32 __qm_err_isr_read(struct qman *qm, enum qm_isr_reg n)
{
	return __qm_in(qm, REG_ERR_ISR + (n << 2));
}

static void __qm_err_isr_write(struct qman *qm, enum qm_isr_reg n, u32 val)
{
	__qm_out(qm, REG_ERR_ISR + (n << 2), val);
}

static void qm_set_dc(struct qman *qm, enum qm_dc_portal portal,
			int ed, u8 sernd)
{
	DPA_ASSERT(!ed || (portal == qm_dc_portal_fman0) ||
			(portal == qm_dc_portal_fman1));
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30)
		qm_out(DCP_CFG(portal), (ed ? 0x1000 : 0) | (sernd & 0x3ff));
	else
		qm_out(DCP_CFG(portal), (ed ? 0x100 : 0) | (sernd & 0x1f));
}

static void qm_set_wq_scheduling(struct qman *qm, enum qm_wq_class wq_class,
			u8 cs_elev, u8 csw2, u8 csw3, u8 csw4, u8 csw5,
			u8 csw6, u8 csw7)
{
	qm_out(WQ_CS_CFG(wq_class), ((cs_elev & 0xff) << 24) |
		((csw2 & 0x7) << 20) | ((csw3 & 0x7) << 16) |
		((csw4 & 0x7) << 12) | ((csw5 & 0x7) << 8) |
		((csw6 & 0x7) << 4) | (csw7 & 0x7));
}

static void qm_set_hid(struct qman *qm)
{
	qm_out(HID_CFG, 0);
}

static void qm_set_corenet_initiator(struct qman *qm)
{
	qm_out(CI_SCHED_CFG,
		0x80000000 | /* write srcciv enable */
		(CONFIG_FSL_QMAN_CI_SCHED_CFG_SRCCIV << 24) |
		(CONFIG_FSL_QMAN_CI_SCHED_CFG_SRQ_W << 8) |
		(CONFIG_FSL_QMAN_CI_SCHED_CFG_RW_W << 4) |
		CONFIG_FSL_QMAN_CI_SCHED_CFG_BMAN_W);
}

static void qm_get_version(struct qman *qm, u16 *id, u8 *major, u8 *minor,
			u8 *cfg)
{
	u32 v = qm_in(IP_REV_1);
	u32 v2 = qm_in(IP_REV_2);
	*id = (v >> 16);
	*major = (v >> 8) & 0xff;
	*minor = v & 0xff;
	*cfg = v2 & 0xff;
}

static void qm_set_memory(struct qman *qm, enum qm_memory memory, u64 ba,
			int enable, int prio, int stash, u32 size)
{
	u32 offset = (memory == qm_memory_fqd) ? REG_FQD_BARE : REG_PFDR_BARE;
	u32 exp = ilog2(size);
	/* choke if size isn't within range */
	DPA_ASSERT((size >= 4096) && (size <= 1073741824) &&
			is_power_of_2(size));
	/* choke if 'ba' has lower-alignment than 'size' */
	DPA_ASSERT(!(ba & (size - 1)));
	__qm_out(qm, offset, upper_32_bits(ba));
	__qm_out(qm, offset + REG_offset_BAR, lower_32_bits(ba));
	__qm_out(qm, offset + REG_offset_AR,
		(enable ? 0x80000000 : 0) |
		(prio ? 0x40000000 : 0) |
		(stash ? 0x20000000 : 0) |
		(exp - 1));
}

static void qm_set_pfdr_threshold(struct qman *qm, u32 th, u8 k)
{
	qm_out(PFDR_FP_LWIT, th & 0xffffff);
	qm_out(PFDR_CFG, k);
}

static void qm_set_sfdr_threshold(struct qman *qm, u16 th)
{
	qm_out(SFDR_CFG, th & 0x3ff);
}

static int qm_init_pfdr(struct qman *qm, u32 pfdr_start, u32 num)
{
	u8 rslt = MCR_get_rslt(qm_in(MCR));

	DPA_ASSERT(pfdr_start && !(pfdr_start & 7) && !(num & 7) && num);
	/* Make sure the command interface is 'idle' */
	if (!MCR_rslt_idle(rslt))
		panic("QMAN_MCR isn't idle");

	/* Write the MCR command params then the verb */
	qm_out(MCP(0), pfdr_start);
	/* TODO: remove this - it's a workaround for a model bug that is
	 * corrected in more recent versions. We use the workaround until
	 * everyone has upgraded. */
	qm_out(MCP(1), (pfdr_start + num - 16));
	lwsync();
	qm_out(MCR, MCR_INIT_PFDR);
	/* Poll for the result */
	do {
		rslt = MCR_get_rslt(qm_in(MCR));
	} while (!MCR_rslt_idle(rslt));
	if (MCR_rslt_ok(rslt))
		return 0;
	if (MCR_rslt_eaccess(rslt))
		return -EACCES;
	if (MCR_rslt_inval(rslt))
		return -EINVAL;
	pr_crit("Unexpected result from MCR_INIT_PFDR: %02x\n", rslt);
	return -ENOSYS;
}

/*****************/
/* Config driver */
/*****************/

#define DEFAULT_FQD_SZ	(PAGE_SIZE << CONFIG_FSL_QMAN_FQD_SZ)
#define DEFAULT_PFDR_SZ	(PAGE_SIZE << CONFIG_FSL_QMAN_PFDR_SZ)

/* We support only one of these */
static struct qman *qm;
static struct device_node *qm_node;

/* And this state belongs to 'qm'. It is set during fsl_qman_init(), but used
 * during qman_init_ccsr(). */
static dma_addr_t fqd_a, pfdr_a;
static size_t fqd_sz = DEFAULT_FQD_SZ, pfdr_sz = DEFAULT_PFDR_SZ;

static int qman_fqd(struct reserved_mem *rmem)
{
	fqd_a = rmem->base;
	fqd_sz = rmem->size;

	WARN_ON(!(fqd_a && fqd_sz));

	return 0;
}
RESERVEDMEM_OF_DECLARE(qman_fqd, "fsl,qman-fqd", qman_fqd);

static int qman_pfdr(struct reserved_mem *rmem)
{
	pfdr_a = rmem->base;
	pfdr_sz = rmem->size;

	WARN_ON(!(pfdr_a && pfdr_sz));

	return 0;
}
RESERVEDMEM_OF_DECLARE(qman_fbpr, "fsl,qman-pfdr", qman_pfdr);

size_t get_qman_fqd_size()
{
	return fqd_sz;
}

/* Parse the <name> property to extract the memory location and size and
 * memblock_reserve() it. If it isn't supplied, memblock_alloc() the default
 * size. Also flush this memory range from data cache so that QMAN originated
 * transactions for this memory region could be marked non-coherent.
 */
static __init int parse_mem_property(struct device_node *node, const char *name,
				dma_addr_t *addr, size_t *sz, int zero)
{
	int ret;

	/* If using a "zero-pma", don't try to zero it, even if you asked */
	if (zero && of_find_property(node, "zero-pma", &ret)) {
		pr_info("  it's a 'zero-pma', not zeroing from s/w\n");
		zero = 0;
	}

	if (zero) {
		/* map as cacheable, non-guarded */
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
		void __iomem *tmpp = ioremap_cache(*addr, *sz);
#else
		void __iomem *tmpp = ioremap(*addr, *sz);
#endif

		if (!tmpp)
			return -ENOMEM;
		memset_io(tmpp, 0, *sz);
		flush_dcache_range((unsigned long)tmpp,
				   (unsigned long)tmpp + *sz);
		iounmap(tmpp);
	}

	return 0;
}

/* TODO:
 * - there is obviously no handling of errors,
 * - the calls to qm_set_memory() hard-code the priority and CPC-stashing for
 *   both memory resources to zero.
 */
static int __init fsl_qman_init(struct device_node *node)
{
	struct resource res;
	resource_size_t len;
	u32 __iomem *regs;
	const char *s;
	int ret, standby = 0;
	u16 id;
	u8 major, minor, cfg;
	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		pr_err("Can't get %s property '%s'\n", node->full_name, "reg");
		return ret;
	}
	s = of_get_property(node, "fsl,hv-claimable", &ret);
	if (s && !strcmp(s, "standby"))
		standby = 1;
	if (!standby) {
		ret = parse_mem_property(node, "fsl,qman-fqd",
					&fqd_a, &fqd_sz, 1);
		pr_info("qman-fqd addr %pad size 0x%zx\n", &fqd_a, fqd_sz);
		BUG_ON(ret);
		ret = parse_mem_property(node, "fsl,qman-pfdr",
					&pfdr_a, &pfdr_sz, 0);
		pr_info("qman-pfdr addr %pad size 0x%zx\n", &pfdr_a, pfdr_sz);
		BUG_ON(ret);
	}
	/* Global configuration */
	len = resource_size(&res);
	if (len != (unsigned long)len)
		return -EINVAL;
	regs = ioremap(res.start, (unsigned long)len);
	qm = qm_create(regs);
	qm_node = node;
	qm_get_version(qm, &id, &major, &minor, &cfg);
	pr_info("Qman ver:%04x,%02x,%02x,%02x\n", id, major, minor, cfg);
	if (!qman_ip_rev) {
		if ((major == 1) && (minor == 0)) {
			pr_err("QMAN rev1.0 on P4080 rev1 is not supported!\n");
			iounmap(regs);
			return -ENODEV;
		} else if ((major == 1) && (minor == 1))
			qman_ip_rev = QMAN_REV11;
		else if	((major == 1) && (minor == 2))
			qman_ip_rev = QMAN_REV12;
		else if ((major == 2) && (minor == 0))
			qman_ip_rev = QMAN_REV20;
		else if ((major == 3) && (minor == 0))
			qman_ip_rev = QMAN_REV30;
		else if ((major == 3) && (minor == 1))
			qman_ip_rev = QMAN_REV31;
		else if ((major == 3) && (minor == 2))
			qman_ip_rev = QMAN_REV32;
		else {
			pr_warn("unknown Qman version, default to rev1.1\n");
			qman_ip_rev = QMAN_REV11;
		}
		qman_ip_cfg = cfg;
	}

	if (standby) {
		pr_info("  -> in standby mode\n");
		return 0;
	}
	return 0;
}

int qman_have_ccsr(void)
{
	return qm ? 1 : 0;
}

__init int qman_init_early(void)
{
	struct device_node *dn;
	int ret;

	for_each_compatible_node(dn, NULL, "fsl,qman") {
		if (qm)
			pr_err("%s: only one 'fsl,qman' allowed\n",
				dn->full_name);
		else {
			if (!of_device_is_available(dn))
				continue;

			ret = fsl_qman_init(dn);
			BUG_ON(ret);
		}
	}
	return 0;
}
postcore_initcall_sync(qman_init_early);

static void log_edata_bits(u32 bit_count)
{
	u32 i, j, mask = 0xffffffff;

	pr_warn("Qman ErrInt, EDATA:\n");
	i = bit_count/32;
	if (bit_count%32) {
		i++;
		mask = ~(mask << bit_count%32);
	}
	j = 16-i;
	pr_warn("  0x%08x\n", qm_in(EDATA(j)) & mask);
	j++;
	for (; j < 16; j++)
		pr_warn("  0x%08x\n", qm_in(EDATA(j)));
}

static void log_additional_error_info(u32 isr_val, u32 ecsr_val)
{
	union qman_ecir ecir_val;
	union qman_eadr eadr_val;

	ecir_val.ecir_raw = qm_in(ECIR);
	/* Is portal info valid */
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30) {
		union qman_ecir2 ecir2_val;
		ecir2_val.ecir2_raw = qm_in(ECIR2);
		if (ecsr_val & PORTAL_ECSR_ERR) {
			pr_warn("Qman ErrInt: %s id %d\n",
				(ecir2_val.info.portal_type) ?
				"DCP" : "SWP", ecir2_val.info.portal_num);
		}
		if (ecsr_val & (FQID_ECSR_ERR | QM_EIRQ_IECE)) {
			pr_warn("Qman ErrInt: ecir.fqid 0x%x\n",
				ecir_val.info.fqid);
		}
		if (ecsr_val & (QM_EIRQ_SBEI|QM_EIRQ_MBEI)) {
			eadr_val.eadr_raw = qm_in(EADR);
			pr_warn("Qman ErrInt: EADR Memory: %s, 0x%x\n",
				error_mdata[eadr_val.info_rev3.memid].txt,
				error_mdata[eadr_val.info_rev3.memid].addr_mask
					& eadr_val.info_rev3.eadr);
			log_edata_bits(
				error_mdata[eadr_val.info_rev3.memid].bits);
		}
	} else {
		if (ecsr_val & PORTAL_ECSR_ERR) {
			pr_warn("Qman ErrInt: %s id %d\n",
				(ecir_val.info.portal_type) ?
				"DCP" : "SWP", ecir_val.info.portal_num);
		}
		if (ecsr_val & FQID_ECSR_ERR) {
			pr_warn("Qman ErrInt: ecir.fqid 0x%x\n",
				ecir_val.info.fqid);
		}
		if (ecsr_val & (QM_EIRQ_SBEI|QM_EIRQ_MBEI)) {
			eadr_val.eadr_raw = qm_in(EADR);
			pr_warn("Qman ErrInt: EADR Memory: %s, 0x%x\n",
				error_mdata[eadr_val.info.memid].txt,
				error_mdata[eadr_val.info.memid].addr_mask
					& eadr_val.info.eadr);
			log_edata_bits(error_mdata[eadr_val.info.memid].bits);
		}
	}
}

/* Qman interrupt handler */
static irqreturn_t qman_isr(int irq, void *ptr)
{
	u32 isr_val, ier_val, ecsr_val, isr_mask, i;

	ier_val = qm_err_isr_enable_read(qm);
	isr_val = qm_err_isr_status_read(qm);
	ecsr_val = qm_in(ECSR);
	isr_mask = isr_val & ier_val;

	if (!isr_mask)
		return IRQ_NONE;
	for (i = 0; i < QMAN_HWE_COUNT; i++) {
		if (qman_hwerr_txts[i].mask & isr_mask) {
			pr_warn("Qman ErrInt: %s\n", qman_hwerr_txts[i].txt);
			if (qman_hwerr_txts[i].mask & ecsr_val) {
				log_additional_error_info(isr_mask, ecsr_val);
				/* Re-arm error capture registers */
				qm_out(ECSR, ecsr_val);
			}
			if (qman_hwerr_txts[i].mask & QMAN_ERRS_TO_UNENABLE) {
				pr_devel("Qman un-enabling error 0x%x\n",
					qman_hwerr_txts[i].mask);
				ier_val &= ~qman_hwerr_txts[i].mask;
				qm_err_isr_enable_write(qm, ier_val);
			}
		}
	}
	qm_err_isr_status_clear(qm, isr_val);
	return IRQ_HANDLED;
}

static int __bind_irq(void)
{
	int ret, err_irq;

	err_irq = of_irq_to_resource(qm_node, 0, NULL);
	if (err_irq == 0) {
		pr_info("Can't get %s property '%s'\n", qm_node->full_name,
			"interrupts");
		return -ENODEV;
	}
	ret = request_irq(err_irq, qman_isr, IRQF_SHARED, "qman-err", qm_node);
	if (ret)  {
		pr_err("request_irq() failed %d for '%s'\n", ret,
			qm_node->full_name);
		return -ENODEV;
	}
	/* Write-to-clear any stale bits, (eg. starvation being asserted prior
	 * to resource allocation during driver init). */
	qm_err_isr_status_clear(qm, 0xffffffff);
	/* Enable Error Interrupts */
	qm_err_isr_enable_write(qm, 0xffffffff);
	return 0;
}

int qman_init_ccsr(struct device_node *node)
{
	int ret;
	if (!qman_have_ccsr())
		return 0;
	if (node != qm_node)
		return -EINVAL;
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	/* TEMP for LS1043 : should be done in uboot */
	qm_out(QCSP_BARE, 0x5);
	qm_out(QCSP_BAR, 0x0);
#endif
	/* FQD memory */
	qm_set_memory(qm, qm_memory_fqd, fqd_a, 1, 0, 0, fqd_sz);
	/* PFDR memory */
	qm_set_memory(qm, qm_memory_pfdr, pfdr_a, 1, 0, 0, pfdr_sz);
	qm_init_pfdr(qm, 8, pfdr_sz / 64 - 8);
	/* thresholds */
	qm_set_pfdr_threshold(qm, 512, 64);
	qm_set_sfdr_threshold(qm, 128);
	/* clear stale PEBI bit from interrupt status register */
	qm_err_isr_status_clear(qm, QM_EIRQ_PEBI);
	/* corenet initiator settings */
	qm_set_corenet_initiator(qm);
	/* HID settings */
	qm_set_hid(qm);
	/* Set scheduling weights to defaults */
	for (ret = qm_wq_first; ret <= qm_wq_last; ret++)
		qm_set_wq_scheduling(qm, ret, 0, 0, 0, 0, 0, 0, 0);
	/* We are not prepared to accept ERNs for hardware enqueues */
	qm_set_dc(qm, qm_dc_portal_fman0, 1, 0);
	qm_set_dc(qm, qm_dc_portal_fman1, 1, 0);
	/* Initialise Error Interrupt Handler */
	ret = __bind_irq();
	if (ret)
		return ret;
	return 0;
}

#define LIO_CFG_LIODN_MASK 0x0fff0000
void qman_liodn_fixup(u16 channel)
{
	static int done;
	static u32 liodn_offset;
	u32 before, after;
	int idx = channel - QM_CHANNEL_SWPORTAL0;

	if (!qman_have_ccsr())
		return;
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30)
		before = qm_in(REV3_QCSP_LIO_CFG(idx));
	else
		before = qm_in(QCSP_LIO_CFG(idx));
	if (!done) {
		liodn_offset = before & LIO_CFG_LIODN_MASK;
		done = 1;
		return;
	}
	after = (before & (~LIO_CFG_LIODN_MASK)) | liodn_offset;
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30)
		qm_out(REV3_QCSP_LIO_CFG(idx), after);
	else
		qm_out(QCSP_LIO_CFG(idx), after);
}

#define IO_CFG_SDEST_MASK 0x00ff0000
int qman_set_sdest(u16 channel, unsigned int cpu_idx)
{
	int idx = channel - QM_CHANNEL_SWPORTAL0;
	u32 before, after;

	if (!qman_have_ccsr())
		return -ENODEV;
	if ((qman_ip_rev & 0xFFFF) == QMAN_REV31) {
		/* LS1043A - only one L2 cache */
		cpu_idx = 0;
	}

	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30) {
		before = qm_in(REV3_QCSP_IO_CFG(idx));
		/* Each pair of vcpu share the same SRQ(SDEST) */
		cpu_idx /= 2;
		after = (before & (~IO_CFG_SDEST_MASK)) | (cpu_idx << 16);
		qm_out(REV3_QCSP_IO_CFG(idx), after);
	} else {
		before = qm_in(QCSP_IO_CFG(idx));
		after = (before & (~IO_CFG_SDEST_MASK)) | (cpu_idx << 16);
		qm_out(QCSP_IO_CFG(idx), after);
	}
	return 0;
}

#define MISC_CFG_WPM_MASK 0x00000002
int qm_set_wpm(int wpm)
{
	u32 before;
	u32 after;

	if (!qman_have_ccsr())
		return -ENODEV;

	before = qm_in(MISC_CFG);
	after = (before & (~MISC_CFG_WPM_MASK)) | (wpm << 1);
	qm_out(MISC_CFG, after);
	return 0;
}

int qm_get_wpm(int *wpm)
{
	u32 before;

	if (!qman_have_ccsr())
		return -ENODEV;

	before = qm_in(MISC_CFG);
	*wpm = (before & MISC_CFG_WPM_MASK) >> 1;
	return 0;
}

/* CEETM_CFG_PRES register has PRES field which is calculated by:
 *    PRES = (2^22 / credit update reference period) * QMan clock period
 *         = (2^22 * 10^9)/ CONFIG_QMAN_CEETM_UPDATE_PERIOD) / qman_clk
 */

int qman_ceetm_set_prescaler(enum qm_dc_portal portal)
{
	u64 temp;
	u16 pres;

	if (!qman_have_ccsr())
		return -ENODEV;

	temp = 0x400000 * 100;
	do_div(temp, CONFIG_QMAN_CEETM_UPDATE_PERIOD);
	temp *= 10000000;
	do_div(temp, qman_clk);
	pres = (u16) temp;
	qm_out(CEETM_CFG_IDX, portal);
	qm_out(CEETM_CFG_PRES, pres);
	return 0;
}

int qman_ceetm_get_prescaler(u16 *pres)
{
	if (!qman_have_ccsr())
		return -ENODEV;
	*pres = (u16)qm_in(CEETM_CFG_PRES);
	return 0;
}

#define DCP_CFG_CEETME_MASK 0xFFFF0000
#define QM_SP_ENABLE_CEETM(n) (0x80000000 >> (n))
int qman_sp_enable_ceetm_mode(enum qm_dc_portal portal, u16 sub_portal)
{
	u32 dcp_cfg;

	if (!qman_have_ccsr())
		return -ENODEV;

	dcp_cfg = qm_in(DCP_CFG(portal));
	dcp_cfg |= QM_SP_ENABLE_CEETM(sub_portal);
	qm_out(DCP_CFG(portal), dcp_cfg);
	return 0;
}

int qman_sp_disable_ceetm_mode(enum qm_dc_portal portal, u16 sub_portal)
{
	u32 dcp_cfg;

	if (!qman_have_ccsr())
		return -ENODEV;
	dcp_cfg = qm_in(DCP_CFG(portal));
	dcp_cfg &= ~(QM_SP_ENABLE_CEETM(sub_portal));
	qm_out(DCP_CFG(portal), dcp_cfg);
	return 0;
}

int qman_ceetm_get_xsfdr(enum qm_dc_portal portal, unsigned int *num)
{
	if (!qman_have_ccsr())
		return -ENODEV;
	*num = qm_in(CEETM_XSFDR_IN_USE);
	return 0;
}
EXPORT_SYMBOL(qman_ceetm_get_xsfdr);

#ifdef CONFIG_SYSFS

#define DRV_NAME	"fsl-qman"
#define DCP_MAX_ID	3
#define DCP_MIN_ID	0

static ssize_t show_pfdr_fpc(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", qm_in(PFDR_FPC));
};

static ssize_t show_dlm_avg(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	u32 data;
	int i;

	if (!sscanf(dev_attr->attr.name, "dcp%d_dlm_avg", &i))
		return -EINVAL;
	if (i < DCP_MIN_ID || i > DCP_MAX_ID)
		return -EINVAL;
	data = qm_in(DCP_DLM_AVG(i));
	return snprintf(buf, PAGE_SIZE, "%d.%08d\n", data>>8,
			(data & 0x000000ff)*390625);
};

static ssize_t set_dlm_avg(struct device *dev,
	struct device_attribute *dev_attr, const char *buf, size_t count)
{
	unsigned long val;
	int i;

	if (!sscanf(dev_attr->attr.name, "dcp%d_dlm_avg", &i))
		return -EINVAL;
	if (i < DCP_MIN_ID || i > DCP_MAX_ID)
		return -EINVAL;
	if (kstrtoul(buf, 0, &val)) {
		dev_dbg(dev, "invalid input %s\n", buf);
		return -EINVAL;
	}
	qm_out(DCP_DLM_AVG(i), val);
	return count;
};

static ssize_t show_pfdr_cfg(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", qm_in(PFDR_CFG));
};

static ssize_t set_pfdr_cfg(struct device *dev,
	struct device_attribute *dev_attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val)) {
		dev_dbg(dev, "invalid input %s\n", buf);
		return -EINVAL;
	}
	qm_out(PFDR_CFG, val);
	return count;
};

static ssize_t show_sfdr_in_use(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", qm_in(SFDR_IN_USE));
};

static ssize_t show_idle_stat(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", qm_in(IDLE_STAT));
};

static ssize_t show_ci_rlm_avg(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	u32 data = qm_in(CI_RLM_AVG);
	return snprintf(buf, PAGE_SIZE, "%d.%08d\n", data>>8,
			(data & 0x000000ff)*390625);
};

static ssize_t set_ci_rlm_avg(struct device *dev,
	struct device_attribute *dev_attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val)) {
		dev_dbg(dev, "invalid input %s\n", buf);
		return -EINVAL;
	}
	qm_out(CI_RLM_AVG, val);
	return count;
};

static ssize_t show_err_isr(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%08x\n", qm_in(ERR_ISR));
};

#define SBEC_MAX_ID	14
#define SBEC_MIN_ID	0

static ssize_t show_sbec(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	int i;

	if (!sscanf(dev_attr->attr.name, "sbec_%d", &i))
		return -EINVAL;
	if (i < SBEC_MIN_ID || i > SBEC_MAX_ID)
		return -EINVAL;
	return snprintf(buf, PAGE_SIZE, "%u\n", qm_in(SBEC(i)));
};

static DEVICE_ATTR(pfdr_fpc, S_IRUSR, show_pfdr_fpc, NULL);
static DEVICE_ATTR(pfdr_cfg, S_IRUSR, show_pfdr_cfg, set_pfdr_cfg);
static DEVICE_ATTR(idle_stat, S_IRUSR, show_idle_stat, NULL);
static DEVICE_ATTR(ci_rlm_avg, (S_IRUSR|S_IWUSR),
		show_ci_rlm_avg, set_ci_rlm_avg);
static DEVICE_ATTR(err_isr, S_IRUSR, show_err_isr, NULL);
static DEVICE_ATTR(sfdr_in_use, S_IRUSR, show_sfdr_in_use, NULL);

static DEVICE_ATTR(dcp0_dlm_avg, (S_IRUSR|S_IWUSR), show_dlm_avg, set_dlm_avg);
static DEVICE_ATTR(dcp1_dlm_avg, (S_IRUSR|S_IWUSR), show_dlm_avg, set_dlm_avg);
static DEVICE_ATTR(dcp2_dlm_avg, (S_IRUSR|S_IWUSR), show_dlm_avg, set_dlm_avg);
static DEVICE_ATTR(dcp3_dlm_avg, (S_IRUSR|S_IWUSR), show_dlm_avg, set_dlm_avg);

static DEVICE_ATTR(sbec_0, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_1, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_2, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_3, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_4, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_5, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_6, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_7, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_8, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_9, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_10, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_11, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_12, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_13, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_14, S_IRUSR, show_sbec, NULL);

static struct attribute *qman_dev_attributes[] = {
	&dev_attr_pfdr_fpc.attr,
	&dev_attr_pfdr_cfg.attr,
	&dev_attr_idle_stat.attr,
	&dev_attr_ci_rlm_avg.attr,
	&dev_attr_err_isr.attr,
	&dev_attr_dcp0_dlm_avg.attr,
	&dev_attr_dcp1_dlm_avg.attr,
	&dev_attr_dcp2_dlm_avg.attr,
	&dev_attr_dcp3_dlm_avg.attr,
	/* sfdr_in_use will be added if necessary */
	NULL
};

static struct attribute *qman_dev_ecr_attributes[] = {
	&dev_attr_sbec_0.attr,
	&dev_attr_sbec_1.attr,
	&dev_attr_sbec_2.attr,
	&dev_attr_sbec_3.attr,
	&dev_attr_sbec_4.attr,
	&dev_attr_sbec_5.attr,
	&dev_attr_sbec_6.attr,
	&dev_attr_sbec_7.attr,
	&dev_attr_sbec_8.attr,
	&dev_attr_sbec_9.attr,
	&dev_attr_sbec_10.attr,
	&dev_attr_sbec_11.attr,
	&dev_attr_sbec_12.attr,
	&dev_attr_sbec_13.attr,
	&dev_attr_sbec_14.attr,
	NULL
};

/* root level */
static const struct attribute_group qman_dev_attr_grp = {
	.name = NULL,
	.attrs = qman_dev_attributes
};
static const struct attribute_group qman_dev_ecr_grp = {
	.name = "error_capture",
	.attrs = qman_dev_ecr_attributes
};

static int of_fsl_qman_remove(struct platform_device *ofdev)
{
	sysfs_remove_group(&ofdev->dev.kobj, &qman_dev_attr_grp);
	return 0;
};

static int of_fsl_qman_probe(struct platform_device *ofdev)
{
	int ret;

	ret = sysfs_create_group(&ofdev->dev.kobj, &qman_dev_attr_grp);
	if (ret)
		goto done;
	ret = sysfs_add_file_to_group(&ofdev->dev.kobj,
		&dev_attr_sfdr_in_use.attr, qman_dev_attr_grp.name);
	if (ret)
		goto del_group_0;
	ret = sysfs_create_group(&ofdev->dev.kobj, &qman_dev_ecr_grp);
	if (ret)
		goto del_group_0;

	goto done;

del_group_0:
	sysfs_remove_group(&ofdev->dev.kobj, &qman_dev_attr_grp);
done:
	if (ret)
		dev_err(&ofdev->dev,
				"Cannot create dev attributes ret=%d\n", ret);
	return ret;
};

static struct of_device_id of_fsl_qman_ids[] = {
	{
		.compatible = "fsl,qman",
	},
	{}
};
MODULE_DEVICE_TABLE(of, of_fsl_qman_ids);

#ifdef CONFIG_SUSPEND

static u32 saved_isdr;
static int qman_pm_suspend_noirq(struct device *dev)
{
	uint32_t idle_state;

	suspend_unused_qportal();
	/* save isdr, disable all, clear isr */
	saved_isdr = qm_err_isr_disable_read(qm);
	qm_err_isr_disable_write(qm, 0xffffffff);
	qm_err_isr_status_clear(qm, 0xffffffff);
	idle_state = qm_in(IDLE_STAT);
	if (!(idle_state & 0x1)) {
		pr_err("Qman not idle 0x%x aborting\n", idle_state);
		qm_err_isr_disable_write(qm, saved_isdr);
		resume_unused_qportal();
		return -EBUSY;
	}
#ifdef CONFIG_PM_DEBUG
	pr_info("Qman suspend code, IDLE_STAT = 0x%x\n", idle_state);
#endif
	return 0;
}

static int qman_pm_resume_noirq(struct device *dev)
{
	/* restore isdr */
	qm_err_isr_disable_write(qm, saved_isdr);
	resume_unused_qportal();
	return 0;
}
#else
#define qman_pm_suspend_noirq	NULL
#define qman_pm_resume_noirq	NULL
#endif

static const struct dev_pm_ops qman_pm_ops = {
	.suspend_noirq = qman_pm_suspend_noirq,
	.resume_noirq = qman_pm_resume_noirq,
};

static struct platform_driver of_fsl_qman_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRV_NAME,
		.of_match_table = of_fsl_qman_ids,
		.pm = &qman_pm_ops,
	},
	.probe = of_fsl_qman_probe,
	.remove      = of_fsl_qman_remove,
};

static int qman_ctrl_init(void)
{
	return platform_driver_register(&of_fsl_qman_driver);
}

static void qman_ctrl_exit(void)
{
	platform_driver_unregister(&of_fsl_qman_driver);
}

module_init(qman_ctrl_init);
module_exit(qman_ctrl_exit);

#endif /* CONFIG_SYSFS */
