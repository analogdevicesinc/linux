/* Copyright 2008-2012 Freescale Semiconductor Inc.
 * Copyright 2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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

#ifndef __DPA_H
#define __DPA_H

#include <linux/netdevice.h>
#include <linux/fsl_qman.h>	/* struct qman_fq */

#include "fm_ext.h"
#include "dpaa_eth_trace.h"

extern int dpa_rx_extra_headroom;
extern int dpa_max_frm;
extern int dpa_num_cpus;

#define dpa_get_rx_extra_headroom() dpa_rx_extra_headroom
#define dpa_get_max_frm() dpa_max_frm

#define dpa_get_max_mtu()	\
	(dpa_get_max_frm() - (VLAN_ETH_HLEN + ETH_FCS_LEN))

#define __hot

/* Simple enum of FQ types - used for array indexing */
enum port_type {RX, TX};

/* TODO: This structure should be renamed & moved to the FMD wrapper */
struct dpa_buffer_layout_s {
	uint16_t	priv_data_size;
	bool		parse_results;
	bool		time_stamp;
	bool		hash_results;
	uint8_t		manip_extra_space;
	uint16_t	data_align;
};

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define DPA_BUG_ON(cond)	BUG_ON(cond)
#else
#define DPA_BUG_ON(cond)
#endif

#define DPA_TX_PRIV_DATA_SIZE	16
#define DPA_PARSE_RESULTS_SIZE sizeof(fm_prs_result_t)
#define DPA_TIME_STAMP_SIZE 8
#define DPA_HASH_RESULTS_SIZE 8
#define DPA_RX_PRIV_DATA_SIZE   (DPA_TX_PRIV_DATA_SIZE + \
					dpa_get_rx_extra_headroom())

#define FM_FD_STAT_RX_ERRORS						\
	(FM_PORT_FRM_ERR_DMA | FM_PORT_FRM_ERR_PHYSICAL	| \
	 FM_PORT_FRM_ERR_SIZE | FM_PORT_FRM_ERR_CLS_DISCARD | \
	 FM_PORT_FRM_ERR_EXTRACTION | FM_PORT_FRM_ERR_NO_SCHEME	| \
	 FM_PORT_FRM_ERR_ILL_PLCR | FM_PORT_FRM_ERR_PRS_TIMEOUT	| \
	 FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT | FM_PORT_FRM_ERR_PRS_HDR_ERR)

#define FM_FD_STAT_TX_ERRORS \
	(FM_PORT_FRM_ERR_UNSUPPORTED_FORMAT | \
	 FM_PORT_FRM_ERR_LENGTH | FM_PORT_FRM_ERR_DMA)

#ifndef CONFIG_FSL_DPAA_ETH_JUMBO_FRAME
/* The raw buffer size must be cacheline aligned.
 * Normally we use 2K buffers.
 */
#define DPA_BP_RAW_SIZE		2048
#else
/* For jumbo frame optimizations, use buffers large enough to accommodate
 * 9.6K frames, FD maximum offset, skb sh_info overhead and some extra
 * space to account for further alignments.
 */
#define DPA_MAX_FRM_SIZE	9600
#ifndef FM_ERRATUM_A050385
#define DPA_BP_RAW_SIZE \
	((DPA_MAX_FRM_SIZE + DPA_MAX_FD_OFFSET + \
	  sizeof(struct skb_shared_info) + 128) & ~(SMP_CACHE_BYTES - 1))
#else /* FM_ERRATUM_A050385 */
#define DPA_BP_RAW_SIZE ((unlikely(fm_has_errata_a050385())) ? 2048 : \
	((DPA_MAX_FRM_SIZE + DPA_MAX_FD_OFFSET + \
	  sizeof(struct skb_shared_info) + 128) & ~(SMP_CACHE_BYTES - 1)))
#endif /* FM_ERRATUM_A050385 */
#endif /* CONFIG_FSL_DPAA_ETH_JUMBO_FRAME */

/* This is what FMan is ever allowed to use.
 * FMan-DMA requires 16-byte alignment for Rx buffers, but SKB_DATA_ALIGN is
 * even stronger (SMP_CACHE_BYTES-aligned), so we just get away with that,
 * via SKB_WITH_OVERHEAD(). We can't rely on netdev_alloc_frag() giving us
 * half-page-aligned buffers (can we?), so we reserve some more space
 * for start-of-buffer alignment.
 */
#define dpa_bp_size(buffer_layout)	(SKB_WITH_OVERHEAD(DPA_BP_RAW_SIZE) - \
						SMP_CACHE_BYTES)
/* We must ensure that skb_shinfo is always cacheline-aligned. */
#define DPA_SKB_SIZE(size)	((size) & ~(SMP_CACHE_BYTES - 1))

/* Maximum size of a buffer for which recycling is allowed.
 * We need an upper limit such that forwarded skbs that get reallocated on Tx
 * aren't allowed to grow unboundedly. On the other hand, we need to make sure
 * that skbs allocated by us will not fail to be recycled due to their size.
 *
 * For a requested size, the kernel allocator provides the next power of two
 * sized block, which the stack will use as is, regardless of the actual size
 * it required; since we must accommodate at most 9.6K buffers (L2 maximum
 * supported frame size), set the recycling upper limit to 16K.
 */
#define DPA_RECYCLE_MAX_SIZE	16384

#if defined(CONFIG_FSL_SDK_FMAN_TEST)
/*TODO: temporary for fman pcd testing */
#define FMAN_PCD_TESTS_MAX_NUM_RANGES	20
#endif

#define DPAA_ETH_FQ_DELTA	0x10000

#define DPAA_ETH_PCD_FQ_BASE(device_addr) \
	(((device_addr) & 0x1fffff) >> 6)

#define DPAA_ETH_PCD_FQ_HI_PRIO_BASE(device_addr) \
	(DPAA_ETH_FQ_DELTA + DPAA_ETH_PCD_FQ_BASE(device_addr))

/* Largest value that the FQD's OAL field can hold.
 * This is DPAA-1.x specific.
 * TODO: This rather belongs in fsl_qman.h
 */
#define FSL_QMAN_MAX_OAL	127

/* Maximum offset value for a contig or sg FD (represented on 9 bits) */
#define DPA_MAX_FD_OFFSET	((1 << 9) - 1)

/* Default alignment for start of data in an Rx FD */
#define DPA_FD_DATA_ALIGNMENT  16

/* Values for the L3R field of the FM Parse Results
 */
/* L3 Type field: First IP Present IPv4 */
#define FM_L3_PARSE_RESULT_IPV4	0x8000
/* L3 Type field: First IP Present IPv6 */
#define FM_L3_PARSE_RESULT_IPV6	0x4000

/* Values for the L4R field of the FM Parse Results
 * See $8.8.4.7.20 - L4 HXS - L4 Results from DPAA-Rev2 Reference Manual.
 */
/* L4 Type field: UDP */
#define FM_L4_PARSE_RESULT_UDP	0x40
/* L4 Type field: TCP */
#define FM_L4_PARSE_RESULT_TCP	0x20
/* FD status field indicating whether the FM Parser has attempted to validate
 * the L4 csum of the frame.
 * Note that having this bit set doesn't necessarily imply that the checksum
 * is valid. One would have to check the parse results to find that out.
 */
#define FM_FD_STAT_L4CV		0x00000004


#define FM_FD_STAT_ERR_PHYSICAL	FM_PORT_FRM_ERR_PHYSICAL

/* Check if the parsed frame was found to be a TCP segment.
 *
 * @parse_result_ptr must be of type (fm_prs_result_t *).
 */
#define fm_l4_frame_is_tcp(parse_result_ptr) \
	((parse_result_ptr)->l4r & FM_L4_PARSE_RESULT_TCP)

/* number of Tx queues to FMan */
#ifdef CONFIG_FMAN_PFC
#define DPAA_ETH_TX_QUEUES	(NR_CPUS * CONFIG_FMAN_PFC_COS_COUNT)
#else
#define DPAA_ETH_TX_QUEUES	NR_CPUS
#endif

#define DPAA_ETH_RX_QUEUES	128

/* Convenience macros for storing/retrieving the skb back-pointers. They must
 * accommodate both recycling and confirmation paths - i.e. cases when the buf
 * was allocated by ourselves, respectively by the stack. In the former case,
 * we could store the skb at negative offset; in the latter case, we can't,
 * so we'll use 0 as offset.
 *
 * NB: @off is an offset from a (struct sk_buff **) pointer!
 */
#define DPA_WRITE_SKB_PTR(skb, skbh, addr, off) \
{ \
	skbh = (struct sk_buff **)addr; \
	*(skbh + (off)) = skb; \
}
#define DPA_READ_SKB_PTR(skb, skbh, addr, off) \
{ \
	skbh = (struct sk_buff **)addr; \
	skb = *(skbh + (off)); \
}

#ifdef CONFIG_PM
/* Magic Packet wakeup */
#define DPAA_WOL_MAGIC		0x00000001
#endif

#if defined(CONFIG_FSL_SDK_FMAN_TEST)
struct pcd_range {
	uint32_t			 base;
	uint32_t			 count;
};
#endif

/* More detailed FQ types - used for fine-grained WQ assignments */
enum dpa_fq_type {
	FQ_TYPE_RX_DEFAULT = 1, /* Rx Default FQs */
	FQ_TYPE_RX_ERROR,       /* Rx Error FQs */
	FQ_TYPE_RX_PCD,         /* User-defined PCDs */
	FQ_TYPE_TX,             /* "Real" Tx FQs */
	FQ_TYPE_TX_CONFIRM,     /* Tx default Conf FQ (actually an Rx FQ) */
	FQ_TYPE_TX_CONF_MQ,     /* Tx conf FQs (one for each Tx FQ) */
	FQ_TYPE_TX_ERROR,       /* Tx Error FQs (these are actually Rx FQs) */
	FQ_TYPE_RX_PCD_HI_PRIO, /* User-defined high-priority PCDs */
};

struct dpa_fq {
	struct qman_fq		 fq_base;
	struct list_head	 list;
	struct net_device	*net_dev;
	bool			 init;
	uint32_t fqid;
	uint32_t flags;
	uint16_t channel;
	uint8_t wq;
	enum dpa_fq_type fq_type;
};

struct dpa_fq_cbs_t {
	struct qman_fq rx_defq;
	struct qman_fq tx_defq;
	struct qman_fq rx_errq;
	struct qman_fq tx_errq;
	struct qman_fq egress_ern;
};

struct fqid_cell {
	uint32_t start;
	uint32_t count;
};

struct dpa_bp {
	struct bman_pool		*pool;
	uint8_t				bpid;
	struct device			*dev;
	union {
		/* The buffer pools used for the private ports are initialized
		 * with target_count buffers for each CPU; at runtime the
		 * number of buffers per CPU is constantly brought back to this
		 * level
		 */
		int target_count;
		/* The configured value for the number of buffers in the pool,
		 * used for shared port buffer pools
		 */
		int config_count;
	};
	size_t				size;
	bool				seed_pool;
	/* physical address of the contiguous memory used by the pool to store
	 * the buffers
	 */
	dma_addr_t			paddr;
	/* virtual address of the contiguous memory used by the pool to store
	 * the buffers
	 */
	void __iomem			*vaddr;
	atomic_t refs;
	/* some bpools need to be emptied before freeing; this cb is used
	 * for freeing of individual buffers taken from the pool
	 */
	void (*free_buf_cb)(void *addr);
};

struct dpa_rx_errors {
	u64 dme;		/* DMA Error */
	u64 fpe;		/* Frame Physical Error */
	u64 fse;		/* Frame Size Error */
	u64 phe;		/* Header Error */
	u64 cse;		/* Checksum Validation Error */
};

/* Counters for QMan ERN frames - one counter per rejection code */
struct dpa_ern_cnt {
	u64 cg_tdrop;		/* Congestion group taildrop */
	u64 wred;		/* WRED congestion */
	u64 err_cond;		/* Error condition */
	u64 early_window;	/* Order restoration, frame too early */
	u64 late_window;	/* Order restoration, frame too late */
	u64 fq_tdrop;		/* FQ taildrop */
	u64 fq_retired;		/* FQ is retired */
	u64 orp_zero;		/* ORP disabled */
};

struct dpa_napi_portal {
	struct napi_struct napi;
	struct qman_portal *p;
};

struct dpa_percpu_priv_s {
	struct net_device *net_dev;
	struct dpa_napi_portal *np;
	u64 in_interrupt;
	u64 tx_returned;
	u64 tx_confirm;
	/* fragmented (non-linear) skbuffs received from the stack */
	u64 tx_frag_skbuffs;
	/* number of S/G frames received */
	u64 rx_sg;

	struct rtnl_link_stats64 stats;
	struct dpa_rx_errors rx_errors;
	struct dpa_ern_cnt ern_cnt;
};

struct dpa_priv_s {
	struct dpa_percpu_priv_s	__percpu *percpu_priv;
	struct dpa_bp *dpa_bp;
	/* current number of buffers in the bpool allotted to this CPU */
	int __percpu *percpu_count;
	/* Store here the needed Tx headroom for convenience and speed
	 * (even though it can be computed based on the fields of buf_layout)
	 */
	uint16_t tx_headroom;
	struct net_device *net_dev;
	struct mac_device	*mac_dev;
	struct qman_fq		*egress_fqs[DPAA_ETH_TX_QUEUES];
	struct qman_fq		*conf_fqs[DPAA_ETH_TX_QUEUES];

	size_t bp_count;

	uint16_t		 channel;	/* "fsl,qman-channel-id" */
	struct list_head	 dpa_fq_list;

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	struct dentry		*debugfs_loop_file;
#endif

	uint32_t		 msg_enable;	/* net_device message level */
#ifdef CONFIG_FSL_DPAA_1588
	struct dpa_ptp_tsu	 *tsu;
#endif

#if defined(CONFIG_FSL_SDK_FMAN_TEST)
/* TODO: this is temporary until pcd support is implemented in dpaa */
	int			priv_pcd_num_ranges;
	struct pcd_range	priv_pcd_ranges[FMAN_PCD_TESTS_MAX_NUM_RANGES];
#endif

	struct {
		/**
		 * All egress queues to a given net device belong to one
		 * (and the same) congestion group.
		 */
		struct qman_cgr cgr;
		/* If congested, when it began. Used for performance stats. */
		u32 congestion_start_jiffies;
		/* Number of jiffies the Tx port was congested. */
		u32 congested_jiffies;
		/**
		 * Counter for the number of times the CGR
		 * entered congestion state
		 */
		u32 cgr_congested_count;
	} cgr_data;
	/* Use a per-port CGR for ingress traffic. */
	bool use_ingress_cgr;
	struct qman_cgr ingress_cgr;

#ifdef CONFIG_FSL_DPAA_TS
	bool ts_tx_en; /* Tx timestamping enabled */
	bool ts_rx_en; /* Rx timestamping enabled */
#endif /* CONFIG_FSL_DPAA_TS */

	struct dpa_buffer_layout_s *buf_layout;
	uint16_t rx_headroom;
	char if_type[30];

	void *peer;
#ifdef CONFIG_PM
	u32 wol;
#endif
#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	int loop_id;
	int loop_to;
#endif
#ifdef CONFIG_FSL_DPAA_CEETM
	bool ceetm_en; /* CEETM QoS enabled */
#endif
};

struct fm_port_fqs {
	struct dpa_fq *tx_defq;
	struct dpa_fq *tx_errq;
	struct dpa_fq *rx_defq;
	struct dpa_fq *rx_errq;
};


#ifdef CONFIG_FSL_DPAA_DBG_LOOP
extern struct net_device *dpa_loop_netdevs[20];
#endif

int dpaa_eth_refill_bpools(struct dpa_bp *dpa_bp, int *count_ptr);
void __hot _dpa_rx(struct net_device *net_dev,
		struct qman_portal *portal,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid,
		int *count_ptr);
int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev);
int __hot dpa_tx_extended(struct sk_buff *skb, struct net_device *net_dev,
		struct qman_fq *egress_fq, struct qman_fq *conf_fq);
struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
				   const struct qm_fd *fd);
void __hot _dpa_process_parse_results(const fm_prs_result_t *parse_results,
				      const struct qm_fd *fd,
				      struct sk_buff *skb,
				      int *use_gro);
#ifndef CONFIG_FSL_DPAA_TS
bool dpa_skb_is_recyclable(struct sk_buff *skb);
bool dpa_buf_is_recyclable(struct sk_buff *skb,
			   uint32_t min_size,
			   uint16_t min_offset,
			   unsigned char **new_buf_start);
#endif
int __hot skb_to_contig_fd(struct dpa_priv_s *priv,
			   struct sk_buff *skb, struct qm_fd *fd,
			   int *count_ptr, int *offset);
int __hot skb_to_sg_fd(struct dpa_priv_s *priv,
		       struct sk_buff *skb, struct qm_fd *fd);
int __cold __attribute__((nonnull))
	_dpa_fq_free(struct device *dev, struct qman_fq *fq);

/* Turn on HW checksum computation for this outgoing frame.
 * If the current protocol is not something we support in this regard
 * (or if the stack has already computed the SW checksum), we do nothing.
 *
 * Returns 0 if all goes well (or HW csum doesn't apply), and a negative value
 * otherwise.
 *
 * Note that this function may modify the fd->cmd field and the skb data buffer
 * (the Parse Results area).
 */
int dpa_enable_tx_csum(struct dpa_priv_s *priv,
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results);

static inline int dpaa_eth_napi_schedule(struct dpa_percpu_priv_s *percpu_priv,
			struct qman_portal *portal)
{
	/* In case of threaded ISR for RT enable kernel,
	 * in_irq() does not return appropriate value, so use
	 * in_serving_softirq to distinguish softirq or irq context.
	 */
	if (unlikely(in_irq() || !in_serving_softirq())) {
		/* Disable QMan IRQ and invoke NAPI */
		int ret = qman_p_irqsource_remove(portal, QM_PIRQ_DQRI);
		if (likely(!ret)) {
			const struct qman_portal_config *pc =
					qman_p_get_portal_config(portal);
			struct dpa_napi_portal *np =
					&percpu_priv->np[pc->index];

			np->p = portal;
			napi_schedule(&np->napi);
			percpu_priv->in_interrupt++;
			return 1;
		}
	}
	return 0;
}

static inline ssize_t __must_check __attribute__((nonnull))
dpa_fd_length(const struct qm_fd *fd)
{
	return fd->length20;
}

static inline ssize_t __must_check __attribute__((nonnull))
dpa_fd_offset(const struct qm_fd *fd)
{
	return fd->offset;
}

static inline uint16_t dpa_get_headroom(struct dpa_buffer_layout_s *bl)
{
	uint16_t headroom;
	/* The frame headroom must accommodate:
	 * - the driver private data area
	 * - parse results, hash results, timestamp if selected
	 * - manip extra space
	 * If either hash results or time stamp are selected, both will
	 * be copied to/from the frame headroom, as TS is located between PR and
	 * HR in the IC and IC copy size has a granularity of 16bytes
	 * (see description of FMBM_RICP and FMBM_TICP registers in DPAARM)
	 *
	 * Also make sure the headroom is a multiple of data_align bytes
	 */
	headroom = (uint16_t)(bl->priv_data_size +
		   (bl->parse_results ? DPA_PARSE_RESULTS_SIZE : 0) +
		   (bl->hash_results || bl->time_stamp ?
		    DPA_TIME_STAMP_SIZE + DPA_HASH_RESULTS_SIZE : 0) +
		   bl->manip_extra_space);

	return bl->data_align ? ALIGN(headroom, bl->data_align) : headroom;
}

int fm_mac_dump_regs(struct mac_device *h_dev, char *buf, int n);
int fm_mac_dump_rx_stats(struct mac_device *h_dev, char *buf, int n);
int fm_mac_dump_tx_stats(struct mac_device *h_dev, char *buf, int n);

void dpaa_eth_sysfs_remove(struct device *dev);
void dpaa_eth_sysfs_init(struct device *dev);
int dpaa_eth_poll(struct napi_struct *napi, int budget);

void dpa_private_napi_del(struct net_device *net_dev);

/* Equivalent to a memset(0), but works faster */
static inline void clear_fd(struct qm_fd *fd)
{
	fd->opaque_addr = 0;
	fd->opaque = 0;
	fd->cmd = 0;
}

static inline int _dpa_tx_fq_to_id(const struct dpa_priv_s *priv,
		struct qman_fq *tx_fq)
{
	int i;

	for (i = 0; i < DPAA_ETH_TX_QUEUES; i++)
		if (priv->egress_fqs[i] == tx_fq)
			return i;

	return -EINVAL;
}

static inline int __hot dpa_xmit(struct dpa_priv_s *priv,
			struct rtnl_link_stats64 *percpu_stats,
			struct qm_fd *fd, struct qman_fq *egress_fq,
			struct qman_fq *conf_fq)
{
	int err, i;

	if (fd->bpid == 0xff)
		fd->cmd |= qman_fq_fqid(conf_fq);

	/* Trace this Tx fd */
	trace_dpa_tx_fd(priv->net_dev, egress_fq, fd);

	for (i = 0; i < 100000; i++) {
		err = qman_enqueue(egress_fq, fd, 0);
		if (err != -EBUSY)
			break;
	}

	if (unlikely(err < 0)) {
		/* TODO differentiate b/w -EBUSY (EQCR full) and other codes? */
		percpu_stats->tx_errors++;
		percpu_stats->tx_fifo_errors++;
		return err;
	}

	percpu_stats->tx_packets++;
	percpu_stats->tx_bytes += dpa_fd_length(fd);

	return 0;
}

/* Use multiple WQs for FQ assignment:
 *	- Tx Confirmation queues go to WQ1.
 *	- Rx Default, Tx and PCD queues go to WQ3 (no differentiation between
 *	  Rx and Tx traffic, or between Rx Default and Rx PCD frames).
 *	- Rx Error and Tx Error queues go to WQ2 (giving them a better chance
 *	  to be scheduled, in case there are many more FQs in WQ3).
 * This ensures that Tx-confirmed buffers are timely released. In particular,
 * it avoids congestion on the Tx Confirm FQs, which can pile up PFDRs if they
 * are greatly outnumbered by other FQs in the system (usually PCDs), while
 * dequeue scheduling is round-robin.
 */
static inline void _dpa_assign_wq(struct dpa_fq *fq)
{
	switch (fq->fq_type) {
	case FQ_TYPE_TX_CONFIRM:
	case FQ_TYPE_TX_CONF_MQ:
		fq->wq = 1;
		break;
	case FQ_TYPE_RX_DEFAULT:
	case FQ_TYPE_TX:
		fq->wq = 3;
		break;
	case FQ_TYPE_RX_ERROR:
	case FQ_TYPE_TX_ERROR:
	case FQ_TYPE_RX_PCD_HI_PRIO:
		fq->wq = 2;
		break;
	case FQ_TYPE_RX_PCD:
		fq->wq = 5;
		break;
	default:
		WARN(1, "Invalid FQ type %d for FQID %d!\n",
		       fq->fq_type, fq->fqid);
	}
}

#ifdef CONFIG_FMAN_PFC
/* Use in lieu of skb_get_queue_mapping() */
#define dpa_get_queue_mapping(skb) \
	(((skb)->priority < CONFIG_FMAN_PFC_COS_COUNT) ? \
		((skb)->priority * dpa_num_cpus + smp_processor_id()) : \
		((CONFIG_FMAN_PFC_COS_COUNT - 1) * \
			dpa_num_cpus + smp_processor_id()));
#else
#define dpa_get_queue_mapping(skb) skb_get_queue_mapping(skb)
#endif

static inline void _dpa_bp_free_pf(void *addr)
{
	put_page(virt_to_head_page(addr));
}

/* LS1043A SoC has a HW issue regarding FMan DMA transactions; The issue
 * manifests itself at high traffic rates when frames cross 4K memory
 * boundaries, when they are not aligned to 16 bytes or when they have
 * Scatter/Gather fragments; For the moment, we use a SW workaround that
 * realigns frames to 256 bytes. Scatter/Gather frames aren't supported
 * on egress.
 */

#ifdef FM_ERRATUM_A050385
#define CROSS_4K(start, size) \
	(((uintptr_t)(start) + (size)) > \
	 (((uintptr_t)(start) + 0x1000) & ~0xFFF))
/* The headroom needs to accommodate our private data (64 bytes) but
 * we reserve 256 bytes instead to guarantee 256 data alignment.
 */
#define DPAA_A050385_HEADROOM	256
#endif  /* FM_ERRATUM_A050385 */

#endif	/* __DPA_H */
