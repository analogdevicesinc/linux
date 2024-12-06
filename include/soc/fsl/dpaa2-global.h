/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2016 NXP
 *
 */
#ifndef __FSL_DPAA2_GLOBAL_H
#define __FSL_DPAA2_GLOBAL_H

#include <linux/types.h>
#include <linux/cpumask.h>
#include "dpaa2-fd.h"

struct dpaa2_dq {
	union {
		struct common {
			u8 verb;
			u8 reserved[63];
		} common;
		struct dq {
			u8 verb;
			u8 stat;
			__le16 seqnum;
			__le16 oprid;
			u8 reserved;
			u8 tok;
			__le32 fqid;
			u32 reserved2;
			__le32 fq_byte_cnt;
			__le32 fq_frm_cnt;
			__le64 fqd_ctx;
			u8 fd[32];
		} dq;
		struct scn {
			u8 verb;
			u8 stat;
			u8 state;
			u8 reserved;
			__le32 rid_tok;
			__le64 ctx;
		} scn;
	};
};

/* Parsing frame dequeue results */
/* FQ empty */
#define DPAA2_DQ_STAT_FQEMPTY       0x80
/* FQ held active */
#define DPAA2_DQ_STAT_HELDACTIVE    0x40
/* FQ force eligible */
#define DPAA2_DQ_STAT_FORCEELIGIBLE 0x20
/* valid frame */
#define DPAA2_DQ_STAT_VALIDFRAME    0x10
/* FQ ODP enable */
#define DPAA2_DQ_STAT_ODPVALID      0x04
/* volatile dequeue */
#define DPAA2_DQ_STAT_VOLATILE      0x02
/* volatile dequeue command is expired */
#define DPAA2_DQ_STAT_EXPIRED       0x01

#define DQ_FQID_MASK		0x00FFFFFF
#define DQ_FRAME_COUNT_MASK	0x00FFFFFF

/**
 * dpaa2_dq_flags() - Get the stat field of dequeue response
 * @dq: the dequeue result.
 */
static inline u32 dpaa2_dq_flags(const struct dpaa2_dq *dq)
{
	return dq->dq.stat;
}

/**
 * dpaa2_dq_is_pull() - Check whether the dq response is from a pull
 *                      command.
 * @dq: the dequeue result
 *
 * Return 1 for volatile(pull) dequeue, 0 for static dequeue.
 */
static inline int dpaa2_dq_is_pull(const struct dpaa2_dq *dq)
{
	return (int)(dpaa2_dq_flags(dq) & DPAA2_DQ_STAT_VOLATILE);
}

/**
 * dpaa2_dq_is_pull_complete() - Check whether the pull command is completed.
 * @dq: the dequeue result
 *
 * Return boolean.
 */
static inline bool dpaa2_dq_is_pull_complete(const struct dpaa2_dq *dq)
{
	return !!(dpaa2_dq_flags(dq) & DPAA2_DQ_STAT_EXPIRED);
}

/**
 * dpaa2_dq_seqnum() - Get the seqnum field in dequeue response
 * @dq: the dequeue result
 *
 * seqnum is valid only if VALIDFRAME flag is TRUE
 *
 * Return seqnum.
 */
static inline u16 dpaa2_dq_seqnum(const struct dpaa2_dq *dq)
{
	return le16_to_cpu(dq->dq.seqnum);
}

/**
 * dpaa2_dq_odpid() - Get the odpid field in dequeue response
 * @dq: the dequeue result
 *
 * odpid is valid only if ODPVALID flag is TRUE.
 *
 * Return odpid.
 */
static inline u16 dpaa2_dq_odpid(const struct dpaa2_dq *dq)
{
	return le16_to_cpu(dq->dq.oprid);
}

/**
 * dpaa2_dq_fqid() - Get the fqid in dequeue response
 * @dq: the dequeue result
 *
 * Return fqid.
 */
static inline u32 dpaa2_dq_fqid(const struct dpaa2_dq *dq)
{
	return le32_to_cpu(dq->dq.fqid) & DQ_FQID_MASK;
}

/**
 * dpaa2_dq_byte_count() - Get the byte count in dequeue response
 * @dq: the dequeue result
 *
 * Return the byte count remaining in the FQ.
 */
static inline u32 dpaa2_dq_byte_count(const struct dpaa2_dq *dq)
{
	return le32_to_cpu(dq->dq.fq_byte_cnt);
}

/**
 * dpaa2_dq_frame_count() - Get the frame count in dequeue response
 * @dq: the dequeue result
 *
 * Return the frame count remaining in the FQ.
 */
static inline u32 dpaa2_dq_frame_count(const struct dpaa2_dq *dq)
{
	return le32_to_cpu(dq->dq.fq_frm_cnt) & DQ_FRAME_COUNT_MASK;
}

/**
 * dpaa2_dq_fd_ctx() - Get the frame queue context in dequeue response
 * @dq: the dequeue result
 *
 * Return the frame queue context.
 */
static inline u64 dpaa2_dq_fqd_ctx(const struct dpaa2_dq *dq)
{
	return le64_to_cpu(dq->dq.fqd_ctx);
}

/**
 * dpaa2_dq_fd() - Get the frame descriptor in dequeue response
 * @dq: the dequeue result
 *
 * Return the frame descriptor.
 */
static inline const struct dpaa2_fd *dpaa2_dq_fd(const struct dpaa2_dq *dq)
{
	return (const struct dpaa2_fd *)&dq->dq.fd[0];
}

#define DPAA2_CSCN_SIZE		sizeof(struct dpaa2_dq)
#define DPAA2_CSCN_ALIGN	16
#define DPAA2_CSCN_STATE_CG	BIT(0)

/**
 * dpaa2_cscn_state_congested() - Check congestion state
 * @cscn: congestion SCN (delivered to WQ or memory)
 *
i * Return true is congested.
 */
static inline bool dpaa2_cscn_state_congested(struct dpaa2_dq *cscn)
{
	return !!(cscn->scn.state & DPAA2_CSCN_STATE_CG);
}

/* Data Path Order Restoration API
 * Contains initialization APIs and runtime APIs for the Order Restoration
 */

/** Order Restoration properties */

/**
 * Create a new Order Point Record option
 */
#define OPR_OPT_CREATE 0x1
/**
 * Retire an existing Order Point Record option
 */
#define OPR_OPT_RETIRE 0x2

/**
 * struct opr_cfg - Structure representing OPR configuration
 * @oprrws: Order point record (OPR) restoration window size (0 to 5)
 *			0 - Window size is 32 frames.
 *			1 - Window size is 64 frames.
 *			2 - Window size is 128 frames.
 *			3 - Window size is 256 frames.
 *			4 - Window size is 512 frames.
 *			5 - Window size is 1024 frames.
 * @oa: OPR auto advance NESN window size (0 disabled, 1 enabled)
 * @olws: OPR acceptable late arrival window size (0 to 3)
 *			0 - Disabled. Late arrivals are always rejected.
 *			1 - Window size is 32 frames.
 *			2 - Window size is the same as the OPR restoration
 *				window size configured in the OPRRWS field.
 *			3 - Window size is 8192 frames. Late arrivals are
 *				always accepted.
 * @oeane: Order restoration list (ORL) resource exhaustion
 *			advance NESN enable (0 disabled, 1 enabled)
 * @oloe: OPR loose ordering enable (0 disabled, 1 enabled)
 */
struct opr_cfg {
	u8 oprrws;
	u8 oa;
	u8 olws;
	u8 oeane;
	u8 oloe;
};

/**
 * struct opr_qry - Structure representing OPR configuration
 * @enable: Enabled state
 * @rip: Retirement In Progress
 * @ndsn: Next dispensed sequence number
 * @nesn: Next expected sequence number
 * @ea_hseq: Early arrival head sequence number
 * @hseq_nlis: HSEQ not last in sequence
 * @ea_tseq: Early arrival tail sequence number
 * @tseq_nlis: TSEQ not last in sequence
 * @ea_tptr: Early arrival tail pointer
 * @ea_hptr: Early arrival head pointer
 * @opr_id: Order Point Record ID
 * @opr_vid: Order Point Record Virtual ID
 */
struct opr_qry {
	char enable;
	char rip;
	u16 ndsn;
	u16 nesn;
	u16 ea_hseq;
	char hseq_nlis;
	u16 ea_tseq;
	char tseq_nlis;
	u16 ea_tptr;
	u16 ea_hptr;
	u16 opr_id;
	u16 opr_vid;
};

#endif /* __FSL_DPAA2_GLOBAL_H */
