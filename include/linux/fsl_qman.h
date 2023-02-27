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

#ifndef FSL_QMAN_H
#define FSL_QMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Last updated for v00.800 of the BG */

/* Hardware constants */
#define QM_CHANNEL_SWPORTAL0 0
#define QMAN_CHANNEL_POOL1 0x21
#define QMAN_CHANNEL_CAAM 0x80
#define QMAN_CHANNEL_PME 0xa0
#define QMAN_CHANNEL_POOL1_REV3 0x401
#define QMAN_CHANNEL_CAAM_REV3 0x840
#define QMAN_CHANNEL_PME_REV3 0x860
#define QMAN_CHANNEL_DCE 0x8a0
#define QMAN_CHANNEL_DCE_QMANREV312 0x880
extern u16 qm_channel_pool1;
extern u16 qm_channel_caam;
extern u16 qm_channel_pme;
extern u16 qm_channel_dce;
enum qm_dc_portal {
	qm_dc_portal_fman0 = 0,
	qm_dc_portal_fman1 = 1,
	qm_dc_portal_caam = 2,
	qm_dc_portal_pme = 3,
	qm_dc_portal_rman = 4,
	qm_dc_portal_dce = 5
};

/* Portal processing (interrupt) sources */
#define QM_PIRQ_CCSCI	0x00200000	/* CEETM Congestion State Change */
#define QM_PIRQ_CSCI	0x00100000	/* Congestion State Change */
#define QM_PIRQ_EQCI	0x00080000	/* Enqueue Command Committed */
#define QM_PIRQ_EQRI	0x00040000	/* EQCR Ring (below threshold) */
#define QM_PIRQ_DQRI	0x00020000	/* DQRR Ring (non-empty) */
#define QM_PIRQ_MRI	0x00010000	/* MR Ring (non-empty) */
/* This mask contains all the interrupt sources that need handling except DQRI,
 * ie. that if present should trigger slow-path processing. */
#define QM_PIRQ_SLOW	(QM_PIRQ_CSCI | QM_PIRQ_EQCI | QM_PIRQ_EQRI | \
			QM_PIRQ_MRI | QM_PIRQ_CCSCI)

/* --- Clock speed --- */
/* A qman driver instance may or may not know the current qman clock speed.
 * However, certain CEETM calculations may not be possible if this is not known.
 * The 'set' function will only succeed (return zero) if the driver did not
 * already know the clock speed. Likewise, the 'get' function will only succeed
 * if the driver does know the clock speed (either because it knew when booting,
 * or was told via 'set'). In cases where software is running on a driver
 * instance that does not know the clock speed (eg. on a hypervised data-plane),
 * and the user can obtain the current qman clock speed by other means (eg. from
 * a message sent from the control-plane), then the 'set' function can be used
 * to enable rate-calculations in a driver where it would otherwise not be
 * possible. */
int qm_get_clock(u64 *clock_hz);
int qm_set_clock(u64 clock_hz);

/* For qman_static_dequeue_*** APIs */
#define QM_SDQCR_CHANNELS_POOL_MASK	0x00007fff
/* for n in [1,15] */
#define QM_SDQCR_CHANNELS_POOL(n)	(0x00008000 >> (n))
/* for conversion from n of qm_channel */
static inline u32 QM_SDQCR_CHANNELS_POOL_CONV(u16 channel)
{
	return QM_SDQCR_CHANNELS_POOL(channel + 1 - qm_channel_pool1);
}

/* For qman_volatile_dequeue(); Choose one PRECEDENCE. EXACT is optional. Use
 * NUMFRAMES(n) (6-bit) or NUMFRAMES_TILLEMPTY to fill in the frame-count. Use
 * FQID(n) to fill in the frame queue ID. */
#define QM_VDQCR_PRECEDENCE_VDQCR	0x0
#define QM_VDQCR_PRECEDENCE_SDQCR	0x80000000
#define QM_VDQCR_EXACT			0x40000000
#define QM_VDQCR_NUMFRAMES_MASK		0x3f000000
#define QM_VDQCR_NUMFRAMES_SET(n)	(((n) & 0x3f) << 24)
#define QM_VDQCR_NUMFRAMES_GET(n)	(((n) >> 24) & 0x3f)
#define QM_VDQCR_NUMFRAMES_TILLEMPTY	QM_VDQCR_NUMFRAMES_SET(0)


/* ------------------------------------------------------- */
/* --- Qman data structures (and associated constants) --- */

/* Represents s/w corenet portal mapped data structures */
struct qm_eqcr_entry;	/* EQCR (EnQueue Command Ring) entries */
struct qm_dqrr_entry;	/* DQRR (DeQueue Response Ring) entries */
struct qm_mr_entry;	/* MR (Message Ring) entries */
struct qm_mc_command;	/* MC (Management Command) command */
struct qm_mc_result;	/* MC result */

/* See David Lapp's "Frame formats" document, "dpateam", Jan 07, 2008 */
#define QM_FD_FORMAT_SG		0x4
#define QM_FD_FORMAT_LONG	0x2
#define QM_FD_FORMAT_COMPOUND	0x1
enum qm_fd_format {
	/* 'contig' implies a contiguous buffer, whereas 'sg' implies a
	 * scatter-gather table. 'big' implies a 29-bit length with no offset
	 * field, otherwise length is 20-bit and offset is 9-bit. 'compound'
	 * implies a s/g-like table, where each entry itself represents a frame
	 * (contiguous or scatter-gather) and the 29-bit "length" is
	 * interpreted purely for congestion calculations, ie. a "congestion
	 * weight". */
	qm_fd_contig = 0,
	qm_fd_contig_big = QM_FD_FORMAT_LONG,
	qm_fd_sg = QM_FD_FORMAT_SG,
	qm_fd_sg_big = QM_FD_FORMAT_SG | QM_FD_FORMAT_LONG,
	qm_fd_compound = QM_FD_FORMAT_COMPOUND
};

/* Capitalised versions are un-typed but can be used in static expressions */
#define QM_FD_CONTIG	0
#define QM_FD_CONTIG_BIG QM_FD_FORMAT_LONG
#define QM_FD_SG	QM_FD_FORMAT_SG
#define QM_FD_SG_BIG	(QM_FD_FORMAT_SG | QM_FD_FORMAT_LONG)
#define QM_FD_COMPOUND	QM_FD_FORMAT_COMPOUND

/* See 1.5.1.1: "Frame Descriptor (FD)" */
struct qm_fd {
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 dd:2;	/* dynamic debug */
			u8 liodn_offset:6;
			u8 bpid:8;	/* Buffer Pool ID */
			u8 eliodn_offset:4;
			u8 __reserved:4;
			u8 addr_hi;	/* high 8-bits of 40-bit address */
			u32 addr_lo;	/* low 32-bits of 40-bit address */
#else
			u32 addr_lo;    /* low 32-bits of 40-bit address */
			u8 addr_hi;     /* high 8-bits of 40-bit address */
			u8 __reserved:4;
			u8 eliodn_offset:4;
			u8 bpid:8;      /* Buffer Pool ID */
			u8 liodn_offset:6;
			u8 dd:2;        /* dynamic debug */
#endif
		};
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u64 __notaddress:24;
			u64 addr:40;
#else
			u64 addr:40;
			u64 __notaddress:24;
#endif
		};
		u64 opaque_addr;
	};
	/* The 'format' field indicates the interpretation of the remaining 29
	 * bits of the 32-bit word. For packing reasons, it is duplicated in the
	 * other union elements. Note, union'd structs are difficult to use with
	 * static initialisation under gcc, in which case use the "opaque" form
	 * with one of the macros. */
	union {
		/* For easier/faster copying of this part of the fd (eg. from a
		 * DQRR entry to an EQCR entry) copy 'opaque' */
		u32 opaque;
		/* If 'format' is _contig or _sg, 20b length and 9b offset */
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			enum qm_fd_format format:3;
			u16 offset:9;
			u32 length20:20;
#else
			u32 length20:20;
			u16 offset:9;
			enum qm_fd_format format:3;
#endif
		};
		/* If 'format' is _contig_big or _sg_big, 29b length */
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			enum qm_fd_format _format1:3;
			u32 length29:29;
#else
			u32 length29:29;
			enum qm_fd_format _format1:3;
#endif
		};
		/* If 'format' is _compound, 29b "congestion weight" */
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			enum qm_fd_format _format2:3;
			u32 cong_weight:29;
#else
			u32 cong_weight:29;
			enum qm_fd_format _format2:3;
#endif
		};
	};
	union {
		u32 cmd;
		u32 status;
	};
} __aligned(8);
#define QM_FD_DD_NULL		0x00
#define QM_FD_PID_MASK		0x3f
static inline u64 qm_fd_addr_get64(const struct qm_fd *fd)
{
	return fd->addr;
}

static inline dma_addr_t qm_fd_addr(const struct qm_fd *fd)
{
	return (dma_addr_t)fd->addr;
}
/* Macro, so we compile better if 'v' isn't always 64-bit */
#define qm_fd_addr_set64(fd, v) \
	do { \
		struct qm_fd *__fd931 = (fd); \
		__fd931->addr = v; \
	} while (0)

/* For static initialisation of FDs (which is complicated by the use of unions
 * in "struct qm_fd"), use the following macros. Note that;
 * - 'dd', 'pid' and 'bpid' are ignored because there's no static initialisation
 *   use-case),
 * - use capitalised QM_FD_*** formats for static initialisation.
 */
#define QM_FD_FMT_20(cmd, addr_hi, addr_lo, fmt, off, len) \
	{ 0, 0, 0, 0, 0, addr_hi, addr_lo, \
	{ (((fmt)&0x7) << 29) | (((off)&0x1ff) << 20) | ((len)&0xfffff) }, \
	{ cmd } }
#define QM_FD_FMT_29(cmd, addr_hi, addr_lo, fmt, len) \
	{ 0, 0, 0, 0, 0, addr_hi, addr_lo, \
	{ (((fmt)&0x7) << 29) | ((len)&0x1fffffff) }, \
	{ cmd } }

/* See 2.2.1.3 Multi-Core Datapath Acceleration Architecture */
#define QM_SG_OFFSET_MASK 0x1FFF
struct qm_sg_entry {
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 __reserved1[3];
			u8 addr_hi;	/* high 8-bits of 40-bit address */
			u32 addr_lo;	/* low 32-bits of 40-bit address */
#else
			u32 addr_lo;	/* low 32-bits of 40-bit address */
			u8 addr_hi;	/* high 8-bits of 40-bit address */
			u8 __reserved1[3];
#endif
		};
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u64 __notaddress:24;
			u64 addr:40;
#else
			u64 addr:40;
			u64 __notaddress:24;
#endif
		};
		u64 opaque;
	};
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u32 extension:1;	/* Extension bit */
			u32 final:1;		/* Final bit */
			u32 length:30;
#else
			u32 length:30;
			u32 final:1;            /* Final bit */
			u32 extension:1;        /* Extension bit */
#endif
		};
		u32 sgt_efl;
	};
	u8 __reserved2;
	u8 bpid;
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u16 __reserved3:3;
			u16 offset:13;
#else
			u16 offset:13;
			u16 __reserved3:3;
#endif
		};
		u16 opaque_offset;
	};
} __packed;
union qm_sg_efl {
	struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		u32 extension:1;	/* Extension bit */
		u32 final:1;		/* Final bit */
		u32 length:30;
#else
		u32 length:30;
		u32 final:1;            /* Final bit */
		u32 extension:1;        /* Extension bit */
#endif
	};
	u32 efl;
};
static inline dma_addr_t qm_sg_addr(const struct qm_sg_entry *sg)
{
	return (dma_addr_t)be64_to_cpu(sg->opaque) & 0xffffffffffULL;
}
static inline u8 qm_sg_entry_get_ext(const struct qm_sg_entry *sg)
{
	union qm_sg_efl u;

	u.efl = be32_to_cpu(sg->sgt_efl);
	return u.extension;
}
static inline u8 qm_sg_entry_get_final(const struct qm_sg_entry *sg)
{
	union qm_sg_efl u;

	u.efl = be32_to_cpu(sg->sgt_efl);
	return u.final;
}
static inline u32 qm_sg_entry_get_len(const struct qm_sg_entry *sg)
{
	union qm_sg_efl u;

	u.efl = be32_to_cpu(sg->sgt_efl);
	return u.length;
}
static inline u8 qm_sg_entry_get_bpid(const struct qm_sg_entry *sg)
{
	return sg->bpid;
}
static inline u16 qm_sg_entry_get_offset(const struct qm_sg_entry *sg)
{
	u32 opaque_offset = be16_to_cpu(sg->opaque_offset);

	return opaque_offset & 0x1fff;
}

/* Macro, so we compile better if 'v' isn't always 64-bit */
#define qm_sg_entry_set64(sg, v) \
	do { \
		struct qm_sg_entry *__sg931 = (sg); \
		__sg931->opaque = cpu_to_be64(v); \
	} while (0)
#define qm_sg_entry_set_ext(sg, v) \
	do { \
		union qm_sg_efl __u932; \
		__u932.efl = be32_to_cpu((sg)->sgt_efl); \
		__u932.extension = v; \
		(sg)->sgt_efl = cpu_to_be32(__u932.efl); \
	} while (0)
#define qm_sg_entry_set_final(sg, v) \
	do { \
		union qm_sg_efl __u933; \
		__u933.efl = be32_to_cpu((sg)->sgt_efl); \
		__u933.final = v; \
		(sg)->sgt_efl = cpu_to_be32(__u933.efl); \
	} while (0)
#define qm_sg_entry_set_len(sg, v) \
	do { \
		union qm_sg_efl __u934; \
		__u934.efl = be32_to_cpu((sg)->sgt_efl); \
		__u934.length = v; \
		(sg)->sgt_efl = cpu_to_be32(__u934.efl); \
	} while (0)
#define qm_sg_entry_set_bpid(sg, v) \
	do { \
		struct qm_sg_entry *__u935 = (sg); \
		__u935->bpid = v; \
	} while (0)
#define qm_sg_entry_set_offset(sg, v) \
	do { \
		struct qm_sg_entry *__u936 = (sg); \
		__u936->opaque_offset = cpu_to_be16(v); \
	} while (0)

/* See 1.5.8.1: "Enqueue Command" */
struct qm_eqcr_entry {
	u8 __dont_write_directly__verb;
	u8 dca;
	u16 seqnum;
	u32 orp;	/* 24-bit */
	u32 fqid;	/* 24-bit */
	u32 tag;
	struct qm_fd fd;
	u8 __reserved3[32];
} __packed __aligned(8);
#define QM_EQCR_VERB_VBIT		0x80
#define QM_EQCR_VERB_CMD_MASK		0x61	/* but only one value; */
#define QM_EQCR_VERB_CMD_ENQUEUE	0x01
#define QM_EQCR_VERB_COLOUR_MASK	0x18	/* 4 possible values; */
#define QM_EQCR_VERB_COLOUR_GREEN	0x00
#define QM_EQCR_VERB_COLOUR_YELLOW	0x08
#define QM_EQCR_VERB_COLOUR_RED		0x10
#define QM_EQCR_VERB_COLOUR_OVERRIDE	0x18
#define QM_EQCR_VERB_INTERRUPT		0x04	/* on command consumption */
#define QM_EQCR_VERB_ORP		0x02	/* enable order restoration */
#define QM_EQCR_DCA_ENABLE		0x80
#define QM_EQCR_DCA_PARK		0x40
#define QM_EQCR_DCA_IDXMASK		0x0f	/* "DQRR::idx" goes here */
#define QM_EQCR_SEQNUM_NESN		0x8000	/* Advance NESN */
#define QM_EQCR_SEQNUM_NLIS		0x4000	/* More fragments to come */
#define QM_EQCR_SEQNUM_SEQMASK		0x3fff	/* sequence number goes here */
#define QM_EQCR_FQID_NULL		0	/* eg. for an ORP seqnum hole */

/* See 1.5.8.2: "Frame Dequeue Response" */
struct qm_dqrr_entry {
	u8 verb;
	u8 stat;
	u16 seqnum;	/* 15-bit */
	u8 tok;
	u8 __reserved2[3];
	u32 fqid;	/* 24-bit */
	u32 contextB;
	struct qm_fd fd;
	u8 __reserved4[32];
};
#define QM_DQRR_VERB_VBIT		0x80
#define QM_DQRR_VERB_MASK		0x7f	/* where the verb contains; */
#define QM_DQRR_VERB_FRAME_DEQUEUE	0x60	/* "this format" */
#define QM_DQRR_STAT_FQ_EMPTY		0x80	/* FQ empty */
#define QM_DQRR_STAT_FQ_HELDACTIVE	0x40	/* FQ held active */
#define QM_DQRR_STAT_FQ_FORCEELIGIBLE	0x20	/* FQ was force-eligible'd */
#define QM_DQRR_STAT_FD_VALID		0x10	/* has a non-NULL FD */
#define QM_DQRR_STAT_UNSCHEDULED	0x02	/* Unscheduled dequeue */
#define QM_DQRR_STAT_DQCR_EXPIRED	0x01	/* VDQCR or PDQCR expired*/

/* See 1.5.8.3: "ERN Message Response" */
/* See 1.5.8.4: "FQ State Change Notification" */
struct qm_mr_entry {
	union {
		struct {
			u8 verb;
			u8 __reserved[31];
		};
		struct {
			u8 verb;
			u8 dca;
			u16 seqnum;
			u8 rc;		/* Rejection Code */
			u32 orp:24;
			u32 fqid;	/* 24-bit */
			u32 tag;
			struct qm_fd fd;
		} __packed __aligned(32) ern;
		struct {
			u8 verb;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 colour:2;	/* See QM_MR_DCERN_COLOUR_* */
			u8 __reserved1:3;
			enum qm_dc_portal portal:3;
#else
			enum qm_dc_portal portal:3;
			u8 __reserved1:3;
			u8 colour:2;	/* See QM_MR_DCERN_COLOUR_* */
#endif
			u16 __reserved2;
			u8 rc;		/* Rejection Code */
			u32 __reserved3:24;
			u32 fqid;	/* 24-bit */
			u32 tag;
			struct qm_fd fd;
		} __packed __aligned(32) dcern;
		struct {
			u8 verb;
			u8 fqs;		/* Frame Queue Status */
			u8 __reserved1[6];
			u32 fqid;	/* 24-bit */
			u32 contextB;
			u8 __reserved2[16];
		} __packed fq;		/* FQRN/FQRNI/FQRL/FQPN */
	};
	u8 __reserved2[32];
} __packed;
#define QM_MR_VERB_VBIT			0x80
/* The "ern" VERB bits match QM_EQCR_VERB_*** so aren't reproduced here. ERNs
 * originating from direct-connect portals ("dcern") use 0x20 as a verb which
 * would be invalid as a s/w enqueue verb. A s/w ERN can be distinguished from
 * the other MR types by noting if the 0x20 bit is unset. */
#define QM_MR_VERB_TYPE_MASK		0x27
#define QM_MR_VERB_DC_ERN		0x20
#define QM_MR_VERB_FQRN			0x21
#define QM_MR_VERB_FQRNI		0x22
#define QM_MR_VERB_FQRL			0x23
#define QM_MR_VERB_FQPN			0x24
#define QM_MR_RC_MASK			0xf0	/* contains one of; */
#define QM_MR_RC_CGR_TAILDROP		0x00
#define QM_MR_RC_WRED			0x10
#define QM_MR_RC_ERROR			0x20
#define QM_MR_RC_ORPWINDOW_EARLY	0x30
#define QM_MR_RC_ORPWINDOW_LATE		0x40
#define QM_MR_RC_FQ_TAILDROP		0x50
#define QM_MR_RC_ORPWINDOW_RETIRED	0x60
#define QM_MR_RC_ORP_ZERO		0x70
#define QM_MR_FQS_ORLPRESENT		0x02	/* ORL fragments to come */
#define QM_MR_FQS_NOTEMPTY		0x01	/* FQ has enqueued frames */
#define QM_MR_DCERN_COLOUR_GREEN	0x00
#define QM_MR_DCERN_COLOUR_YELLOW	0x01
#define QM_MR_DCERN_COLOUR_RED		0x02
#define QM_MR_DCERN_COLOUR_OVERRIDE	0x03

/* An identical structure of FQD fields is present in the "Init FQ" command and
 * the "Query FQ" result, it's suctioned out into the "struct qm_fqd" type.
 * Within that, the 'stashing' and 'taildrop' pieces are also factored out, the
 * latter has two inlines to assist with converting to/from the mant+exp
 * representation. */
struct qm_fqd_stashing {
	/* See QM_STASHING_EXCL_<...> */
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u8 exclusive;
	u8 __reserved1:2;
	/* Numbers of cachelines */
	u8 annotation_cl:2;
	u8 data_cl:2;
	u8 context_cl:2;
#else
	u8 context_cl:2;
	u8 data_cl:2;
	u8 annotation_cl:2;
	u8 __reserved1:2;
	u8 exclusive;
#endif
} __packed;
struct qm_fqd_taildrop {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u16 __reserved1:3;
	u16 mant:8;
	u16 exp:5;
#else
	u16 exp:5;
	u16 mant:8;
	u16 __reserved1:3;
#endif
} __packed;
struct qm_fqd_oac {
	/* See QM_OAC_<...> */
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u8 oac:2; /* "Overhead Accounting Control" */
	u8 __reserved1:6;
#else
	u8 __reserved1:6;
	u8 oac:2; /* "Overhead Accounting Control" */
#endif
	/* Two's-complement value (-128 to +127) */
	signed char oal; /* "Overhead Accounting Length" */
} __packed;
struct qm_fqd {
	union {
		u8 orpc;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 __reserved1:2;
			u8 orprws:3;
			u8 oa:1;
			u8 olws:2;
#else
			u8 olws:2;
			u8 oa:1;
			u8 orprws:3;
			u8 __reserved1:2;
#endif
		} __packed;
	};
	u8 cgid;
	u16 fq_ctrl;	/* See QM_FQCTRL_<...> */
	union {
		u16 dest_wq;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u16 channel:13; /* qm_channel */
			u16 wq:3;
#else
			u16 wq:3;
			u16 channel:13; /* qm_channel */
#endif
		} __packed dest;
	};
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u16 __reserved2:1;
	u16 ics_cred:15;
#else
	u16 __reserved2:1;
	u16 ics_cred:15;
#endif
	/* For "Initialize Frame Queue" commands, the write-enable mask
	 * determines whether 'td' or 'oac_init' is observed. For query
	 * commands, this field is always 'td', and 'oac_query' (below) reflects
	 * the Overhead ACcounting values. */
	union {
		struct qm_fqd_taildrop td;
		struct qm_fqd_oac oac_init;
	};
	u32 context_b;
	union {
		/* Treat it as 64-bit opaque */
		u64 opaque;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u32 hi;
			u32 lo;
#else
			u32 lo;
			u32 hi;
#endif
		};
		/* Treat it as s/w portal stashing config */
		/* See 1.5.6.7.1: "FQD Context_A field used for [...] */
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			struct qm_fqd_stashing stashing;
			/* 48-bit address of FQ context to
			 * stash, must be cacheline-aligned */
			u16 context_hi;
			u32 context_lo;
#else
			u32 context_lo;
			u16 context_hi;
			struct qm_fqd_stashing stashing;
#endif
		} __packed;
	} context_a;
	struct qm_fqd_oac oac_query;
} __packed;
/* 64-bit converters for context_hi/lo */
static inline u64 qm_fqd_stashing_get64(const struct qm_fqd *fqd)
{
	return ((u64)fqd->context_a.context_hi << 32) |
		(u64)fqd->context_a.context_lo;
}
static inline dma_addr_t qm_fqd_stashing_addr(const struct qm_fqd *fqd)
{
	return (dma_addr_t)qm_fqd_stashing_get64(fqd);
}
static inline u64 qm_fqd_context_a_get64(const struct qm_fqd *fqd)
{
	return ((u64)fqd->context_a.hi << 32) |
		(u64)fqd->context_a.lo;
}
/* Macro, so we compile better when 'v' isn't necessarily 64-bit */
#define qm_fqd_stashing_set64(fqd, v) \
	do { \
		struct qm_fqd *__fqd931 = (fqd); \
		__fqd931->context_a.context_hi = upper_32_bits(v); \
		__fqd931->context_a.context_lo = lower_32_bits(v); \
	} while (0)
#define qm_fqd_context_a_set64(fqd, v) \
	do { \
		struct qm_fqd *__fqd931 = (fqd); \
		__fqd931->context_a.hi = upper_32_bits(v); \
		__fqd931->context_a.lo = lower_32_bits(v); \
	} while (0)
/* convert a threshold value into mant+exp representation */
static inline int qm_fqd_taildrop_set(struct qm_fqd_taildrop *td, u32 val,
					int roundup)
{
	u32 e = 0;
	int oddbit = 0;
	if (val > 0xe0000000)
		return -ERANGE;
	while (val > 0xff) {
		oddbit = val & 1;
		val >>= 1;
		e++;
		if (roundup && oddbit)
			val++;
	}
	td->exp = e;
	td->mant = val;
	return 0;
}
/* and the other direction */
static inline u32 qm_fqd_taildrop_get(const struct qm_fqd_taildrop *td)
{
	return (u32)td->mant << td->exp;
}

/* See 1.5.2.2: "Frame Queue Descriptor (FQD)" */
/* Frame Queue Descriptor (FQD) field 'fq_ctrl' uses these constants */
#define QM_FQCTRL_MASK		0x07ff	/* 'fq_ctrl' flags; */
#define QM_FQCTRL_CGE		0x0400	/* Congestion Group Enable */
#define QM_FQCTRL_TDE		0x0200	/* Tail-Drop Enable */
#define QM_FQCTRL_ORP		0x0100	/* ORP Enable */
#define QM_FQCTRL_CTXASTASHING	0x0080	/* Context-A stashing */
#define QM_FQCTRL_CPCSTASH	0x0040	/* CPC Stash Enable */
#define QM_FQCTRL_FORCESFDR	0x0008	/* High-priority SFDRs */
#define QM_FQCTRL_AVOIDBLOCK	0x0004	/* Don't block active */
#define QM_FQCTRL_HOLDACTIVE	0x0002	/* Hold active in portal */
#define QM_FQCTRL_PREFERINCACHE	0x0001	/* Aggressively cache FQD */
#define QM_FQCTRL_LOCKINCACHE	QM_FQCTRL_PREFERINCACHE /* older naming */

/* See 1.5.6.7.1: "FQD Context_A field used for [...] */
/* Frame Queue Descriptor (FQD) field 'CONTEXT_A' uses these constants */
#define QM_STASHING_EXCL_ANNOTATION	0x04
#define QM_STASHING_EXCL_DATA		0x02
#define QM_STASHING_EXCL_CTX		0x01

/* See 1.5.5.3: "Intra Class Scheduling" */
/* FQD field 'OAC' (Overhead ACcounting) uses these constants */
#define QM_OAC_ICS		0x2 /* Accounting for Intra-Class Scheduling */
#define QM_OAC_CG		0x1 /* Accounting for Congestion Groups */

/* See 1.5.8.4: "FQ State Change Notification" */
/* This struct represents the 32-bit "WR_PARM_[GYR]" parameters in CGR fields
 * and associated commands/responses. The WRED parameters are calculated from
 * these fields as follows;
 *   MaxTH = MA * (2 ^ Mn)
 *   Slope = SA / (2 ^ Sn)
 *    MaxP = 4 * (Pn + 1)
 */
struct qm_cgr_wr_parm {
	union {
		u32 word;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u32 MA:8;
			u32 Mn:5;
			u32 SA:7; /* must be between 64-127 */
			u32 Sn:6;
			u32 Pn:6;
#else
			u32 Pn:6;
			u32 Sn:6;
			u32 SA:7; /* must be between 64-127 */
			u32 Mn:5;
			u32 MA:8;
#endif
		} __packed;
	};
} __packed;
/* This struct represents the 13-bit "CS_THRES" CGR field. In the corresponding
 * management commands, this is padded to a 16-bit structure field, so that's
 * how we represent it here. The congestion state threshold is calculated from
 * these fields as follows;
 *   CS threshold = TA * (2 ^ Tn)
 */
struct qm_cgr_cs_thres {
	union {
		u16 hword;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u16 __reserved:3;
			u16 TA:8;
			u16 Tn:5;
#else
			u16 Tn:5;
			u16 TA:8;
			u16 __reserved:3;
#endif
		} __packed;
	};
} __packed;
/* This identical structure of CGR fields is present in the "Init/Modify CGR"
 * commands and the "Query CGR" result. It's suctioned out here into its own
 * struct. */
struct __qm_mc_cgr {
	struct qm_cgr_wr_parm wr_parm_g;
	struct qm_cgr_wr_parm wr_parm_y;
	struct qm_cgr_wr_parm wr_parm_r;
	u8 wr_en_g;	/* boolean, use QM_CGR_EN */
	u8 wr_en_y;	/* boolean, use QM_CGR_EN */
	u8 wr_en_r;	/* boolean, use QM_CGR_EN */
	u8 cscn_en;	/* boolean, use QM_CGR_EN */
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u16 cscn_targ_upd_ctrl; /* use QM_CSCN_TARG_UDP_ */
			u16 cscn_targ_dcp_low;  /* CSCN_TARG_DCP low-16bits */
#else
			u16 cscn_targ_dcp_low;  /* CSCN_TARG_DCP low-16bits */
			u16 cscn_targ_upd_ctrl; /* use QM_CSCN_TARG_UDP_ */
#endif
		};
		u32 cscn_targ;	/* use QM_CGR_TARG_* */
	};
	u8 cstd_en;	/* boolean, use QM_CGR_EN */
	u8 cs;		/* boolean, only used in query response */
	union {
		/* use qm_cgr_cs_thres_set64() */
		struct qm_cgr_cs_thres cs_thres;
		u16 __cs_thres;
	};
	u8 mode;	/* QMAN_CGR_MODE_FRAME not supported in rev1.0 */
} __packed;
#define QM_CGR_EN		0x01 /* For wr_en_*, cscn_en, cstd_en */
#define QM_CGR_TARG_UDP_CTRL_WRITE_BIT	0x8000 /* value written to portal bit*/
#define QM_CGR_TARG_UDP_CTRL_DCP	0x4000 /* 0: SWP, 1: DCP */
#define QM_CGR_TARG_PORTAL(n)	(0x80000000 >> (n)) /* s/w portal, 0-9 */
#define QM_CGR_TARG_FMAN0	0x00200000 /* direct-connect portal: fman0 */
#define QM_CGR_TARG_FMAN1	0x00100000 /*                      : fman1 */
/* Convert CGR thresholds to/from "cs_thres" format */
static inline u64 qm_cgr_cs_thres_get64(const struct qm_cgr_cs_thres *th)
{
	return (u64)th->TA << th->Tn;
}
static inline int qm_cgr_cs_thres_set64(struct qm_cgr_cs_thres *th, u64 val,
					int roundup)
{
	u32 e = 0;
	int oddbit = 0;
	while (val > 0xff) {
		oddbit = val & 1;
		val >>= 1;
		e++;
		if (roundup && oddbit)
			val++;
	}
	th->Tn = e;
	th->TA = val;
	return 0;
}

/* See 1.5.8.5.1: "Initialize FQ" */
/* See 1.5.8.5.2: "Query FQ" */
/* See 1.5.8.5.3: "Query FQ Non-Programmable Fields" */
/* See 1.5.8.5.4: "Alter FQ State Commands " */
/* See 1.5.8.6.1: "Initialize/Modify CGR" */
/* See 1.5.8.6.2: "CGR Test Write" */
/* See 1.5.8.6.3: "Query CGR" */
/* See 1.5.8.6.4: "Query Congestion Group State" */
struct qm_mcc_initfq {
	u8 __reserved1;
	u16 we_mask;	/* Write Enable Mask */
	u32 fqid;	/* 24-bit */
	u16 count;	/* Initialises 'count+1' FQDs */
	struct qm_fqd fqd; /* the FQD fields go here */
	u8 __reserved3[30];
} __packed;
struct qm_mcc_queryfq {
	u8 __reserved1[3];
	u32 fqid;	/* 24-bit */
	u8 __reserved2[56];
} __packed;
struct qm_mcc_queryfq_np {
	u8 __reserved1[3];
	u32 fqid;	/* 24-bit */
	u8 __reserved2[56];
} __packed;
struct qm_mcc_alterfq {
	u8 __reserved1[3];
	u32 fqid;	/* 24-bit */
	u8 __reserved2;
	u8 count;	/* number of consecutive FQID */
	u8 __reserved3[10];
	u32 context_b;	/* frame queue context b */
	u8 __reserved4[40];
} __packed;
struct qm_mcc_initcgr {
	u8 __reserved1;
	u16 we_mask;	/* Write Enable Mask */
	struct __qm_mc_cgr cgr;	/* CGR fields */
	u8 __reserved2[2];
	u8 cgid;
	u8 __reserved4[32];
} __packed;
struct qm_mcc_cgrtestwrite {
	u8 __reserved1[2];
	u8 i_bcnt_hi:8;/* high 8-bits of 40-bit "Instant" */
	u32 i_bcnt_lo;	/* low 32-bits of 40-bit */
	u8 __reserved2[23];
	u8 cgid;
	u8 __reserved3[32];
} __packed;
struct qm_mcc_querycgr {
	u8 __reserved1[30];
	u8 cgid;
	u8 __reserved2[32];
} __packed;
struct qm_mcc_querycongestion {
	u8 __reserved[63];
} __packed;
struct qm_mcc_querywq {
	u8 __reserved;
	/* select channel if verb != QUERYWQ_DEDICATED */
	union {
		u16 channel_wq; /* ignores wq (3 lsbits) */
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u16 id:13; /* qm_channel */
			u16 __reserved1:3;
#else
			u16 __reserved1:3;
			u16 id:13; /* qm_channel */
#endif
		} __packed channel;
	};
	u8 __reserved2[60];
} __packed;

struct qm_mcc_ceetm_lfqmt_config {
	u8 __reserved1[4];
	u32 lfqid:24;
	u8 __reserved2[2];
	u16 cqid;
	u8 __reserved3[2];
	u16 dctidx;
	u8 __reserved4[48];
} __packed;

struct qm_mcc_ceetm_lfqmt_query {
	u8 __reserved1[4];
	u32 lfqid:24;
	u8 __reserved2[56];
} __packed;

struct qm_mcc_ceetm_cq_config {
	u8 __reserved1;
	u16 cqid;
	u8 dcpid;
	u8 __reserved2;
	u16 ccgid;
	u8 __reserved3[56];
} __packed;

struct qm_mcc_ceetm_cq_query {
	u8 __reserved1;
	u16 cqid;
	u8 dcpid;
	u8 __reserved2[59];
} __packed;

struct qm_mcc_ceetm_dct_config {
	u8 __reserved1;
	u16 dctidx;
	u8 dcpid;
	u8 __reserved2[15];
	u32 context_b;
	u64 context_a;
	u8 __reserved3[32];
} __packed;

struct qm_mcc_ceetm_dct_query {
	u8 __reserved1;
	u16 dctidx;
	u8 dcpid;
	u8 __reserved2[59];
} __packed;

struct qm_mcc_ceetm_class_scheduler_config {
	u8 __reserved1;
	u16 cqcid;
	u8 dcpid;
	u8 __reserved2[6];
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u8 gpc_reserved:1;
	u8 gpc_combine_flag:1;
	u8 gpc_prio_b:3;
	u8 gpc_prio_a:3;
#else
	u8 gpc_prio_a:3;
	u8 gpc_prio_b:3;
	u8 gpc_combine_flag:1;
	u8 gpc_reserved:1;
#endif
	u16 crem;
	u16 erem;
	u8 w[8];
	u8 __reserved3[40];
} __packed;

struct qm_mcc_ceetm_class_scheduler_query {
	u8 __reserved1;
	u16 cqcid;
	u8 dcpid;
	u8 __reserved2[59];
} __packed;

#define CEETM_COMMAND_CHANNEL_MAPPING	(0 << 12)
#define CEETM_COMMAND_SP_MAPPING	(1 << 12)
#define CEETM_COMMAND_CHANNEL_SHAPER	(2 << 12)
#define CEETM_COMMAND_LNI_SHAPER	(3 << 12)
#define CEETM_COMMAND_TCFC		(4 << 12)

#define CEETM_CCGRID_MASK	0x01FF
#define CEETM_CCGR_CM_CONFIGURE	(0 << 14)
#define CEETM_CCGR_DN_CONFIGURE	(1 << 14)
#define CEETM_CCGR_TEST_WRITE	(2 << 14)
#define CEETM_CCGR_CM_QUERY	(0 << 14)
#define CEETM_CCGR_DN_QUERY	(1 << 14)
#define CEETM_CCGR_DN_QUERY_FLUSH	(2 << 14)
#define CEETM_QUERY_CONGESTION_STATE (3 << 14)

struct qm_mcc_ceetm_mapping_shaper_tcfc_config {
	u8 __reserved1;
	u16 cid;
	u8 dcpid;
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 map_shaped:1;
			u8 map_reserved:4;
			u8 map_lni_id:3;
#else
			u8 map_lni_id:3;
			u8 map_reserved:4;
			u8 map_shaped:1;
#endif
			u8 __reserved2[58];
		} __packed channel_mapping;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 map_reserved:5;
			u8 map_lni_id:3;
#else
			u8 map_lni_id:3;
			u8 map_reserved:5;
#endif
			u8 __reserved2[58];
		} __packed sp_mapping;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 cpl:1;
			u8 cpl_reserved:2;
			u8 oal:5;
#else
			u8 oal:5;
			u8 cpl_reserved:2;
			u8 cpl:1;
#endif
			u32 crtcr:24;
			u32 ertcr:24;
			u16 crtbl;
			u16 ertbl;
			/* MPS will be hardcoded by the driver to either 0
			 * or 60. See A-010383 for details. */
			u8 mps;
			u8 __reserved2[47];
		} __packed shaper_config;
		struct {
			u8 __reserved2[11];
			u64 lnitcfcc;
			u8 __reserved3[40];
		} __packed tcfc_config;
	};
} __packed;

struct qm_mcc_ceetm_mapping_shaper_tcfc_query {
	u8 __reserved1;
	u16 cid;
	u8 dcpid;
	u8 __reserved2[59];
} __packed;

struct qm_mcc_ceetm_ccgr_config {
	u8 __reserved1;
	u16 ccgrid;
	u8 dcpid;
	u8 __reserved2;
	u16 we_mask;
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 ctl_reserved:1;
			u8 ctl_wr_en_g:1;
			u8 ctl_wr_en_y:1;
			u8 ctl_wr_en_r:1;
			u8 ctl_td_en:1;
			u8 ctl_td_mode:1;
			u8 ctl_cscn_en:1;
			u8 ctl_mode:1;
#else
			u8 ctl_mode:1;
			u8 ctl_cscn_en:1;
			u8 ctl_td_mode:1;
			u8 ctl_td_en:1;
			u8 ctl_wr_en_r:1;
			u8 ctl_wr_en_y:1;
			u8 ctl_wr_en_g:1;
			u8 ctl_reserved:1;
#endif
			u8 cdv;
			u16 cscn_tupd;
			u8 oal;
			u8 __reserved3;
			struct qm_cgr_cs_thres cs_thres;
			struct qm_cgr_cs_thres cs_thres_x;
			struct qm_cgr_cs_thres td_thres;
			struct qm_cgr_wr_parm wr_parm_g;
			struct qm_cgr_wr_parm wr_parm_y;
			struct qm_cgr_wr_parm wr_parm_r;
		} __packed cm_config;
		struct {
			u8 dnc;
			u8 dn0;
			u8 dn1;
			u64 dnba:40;
			u8 __reserved3[2];
			u16 dnth_0;
			u8 __reserved4[2];
			u16 dnth_1;
			u8 __reserved5[8];
		} __packed dn_config;
		struct {
			u8 __reserved3[3];
			u64 i_cnt:40;
			u8 __reserved4[16];
		} __packed test_write;
	};
	u8 __reserved5[32];
} __packed;

struct qm_mcc_ceetm_ccgr_query {
	u8 __reserved1;
	u16 ccgrid;
	u8 dcpid;
	u8 __reserved2[59];
} __packed;

struct qm_mcc_ceetm_cq_peek_pop_xsfdrread {
	u8 __reserved1;
	u16 cqid;
	u8 dcpid;
	u8 ct;
	u16 xsfdr;
	u8 __reserved2[56];
} __packed;

#define CEETM_QUERY_DEQUEUE_STATISTICS 0x00
#define CEETM_QUERY_DEQUEUE_CLEAR_STATISTICS 0x01
#define CEETM_WRITE_DEQUEUE_STATISTICS 0x02
#define CEETM_QUERY_REJECT_STATISTICS 0x03
#define CEETM_QUERY_REJECT_CLEAR_STATISTICS 0x04
#define CEETM_WRITE_REJECT_STATISTICS 0x05
struct qm_mcc_ceetm_statistics_query_write {
	u8 __reserved1;
	u16 cid;
	u8 dcpid;
	u8 ct;
	u8 __reserved2[13];
	u64 frm_cnt:40;
	u8 __reserved3[2];
	u64 byte_cnt:48;
	u8 __reserved[32];
} __packed;

struct qm_mc_command {
	u8 __dont_write_directly__verb;
	union {
		struct qm_mcc_initfq initfq;
		struct qm_mcc_queryfq queryfq;
		struct qm_mcc_queryfq_np queryfq_np;
		struct qm_mcc_alterfq alterfq;
		struct qm_mcc_initcgr initcgr;
		struct qm_mcc_cgrtestwrite cgrtestwrite;
		struct qm_mcc_querycgr querycgr;
		struct qm_mcc_querycongestion querycongestion;
		struct qm_mcc_querywq querywq;
		struct qm_mcc_ceetm_lfqmt_config lfqmt_config;
		struct qm_mcc_ceetm_lfqmt_query lfqmt_query;
		struct qm_mcc_ceetm_cq_config cq_config;
		struct qm_mcc_ceetm_cq_query cq_query;
		struct qm_mcc_ceetm_dct_config dct_config;
		struct qm_mcc_ceetm_dct_query dct_query;
		struct qm_mcc_ceetm_class_scheduler_config csch_config;
		struct qm_mcc_ceetm_class_scheduler_query csch_query;
		struct qm_mcc_ceetm_mapping_shaper_tcfc_config mst_config;
		struct qm_mcc_ceetm_mapping_shaper_tcfc_query mst_query;
		struct qm_mcc_ceetm_ccgr_config ccgr_config;
		struct qm_mcc_ceetm_ccgr_query ccgr_query;
		struct qm_mcc_ceetm_cq_peek_pop_xsfdrread cq_ppxr;
		struct qm_mcc_ceetm_statistics_query_write stats_query_write;
	};
} __packed;
#define QM_MCC_VERB_VBIT		0x80
#define QM_MCC_VERB_MASK		0x7f	/* where the verb contains; */
#define QM_MCC_VERB_INITFQ_PARKED	0x40
#define QM_MCC_VERB_INITFQ_SCHED	0x41
#define QM_MCC_VERB_QUERYFQ		0x44
#define QM_MCC_VERB_QUERYFQ_NP		0x45	/* "non-programmable" fields */
#define QM_MCC_VERB_QUERYWQ		0x46
#define QM_MCC_VERB_QUERYWQ_DEDICATED	0x47
#define QM_MCC_VERB_ALTER_SCHED		0x48	/* Schedule FQ */
#define QM_MCC_VERB_ALTER_FE		0x49	/* Force Eligible FQ */
#define QM_MCC_VERB_ALTER_RETIRE	0x4a	/* Retire FQ */
#define QM_MCC_VERB_ALTER_OOS		0x4b	/* Take FQ out of service */
#define QM_MCC_VERB_ALTER_FQXON		0x4d	/* FQ XON */
#define QM_MCC_VERB_ALTER_FQXOFF	0x4e	/* FQ XOFF */
#define QM_MCC_VERB_INITCGR		0x50
#define QM_MCC_VERB_MODIFYCGR		0x51
#define QM_MCC_VERB_CGRTESTWRITE	0x52
#define QM_MCC_VERB_QUERYCGR		0x58
#define QM_MCC_VERB_QUERYCONGESTION	0x59
/* INITFQ-specific flags */
#define QM_INITFQ_WE_MASK		0x01ff	/* 'Write Enable' flags; */
#define QM_INITFQ_WE_OAC		0x0100
#define QM_INITFQ_WE_ORPC		0x0080
#define QM_INITFQ_WE_CGID		0x0040
#define QM_INITFQ_WE_FQCTRL		0x0020
#define QM_INITFQ_WE_DESTWQ		0x0010
#define QM_INITFQ_WE_ICSCRED		0x0008
#define QM_INITFQ_WE_TDTHRESH		0x0004
#define QM_INITFQ_WE_CONTEXTB		0x0002
#define QM_INITFQ_WE_CONTEXTA		0x0001
/* INITCGR/MODIFYCGR-specific flags */
#define QM_CGR_WE_MASK			0x07ff	/* 'Write Enable Mask'; */
#define QM_CGR_WE_WR_PARM_G		0x0400
#define QM_CGR_WE_WR_PARM_Y		0x0200
#define QM_CGR_WE_WR_PARM_R		0x0100
#define QM_CGR_WE_WR_EN_G		0x0080
#define QM_CGR_WE_WR_EN_Y		0x0040
#define QM_CGR_WE_WR_EN_R		0x0020
#define QM_CGR_WE_CSCN_EN		0x0010
#define QM_CGR_WE_CSCN_TARG		0x0008
#define QM_CGR_WE_CSTD_EN		0x0004
#define QM_CGR_WE_CS_THRES		0x0002
#define QM_CGR_WE_MODE			0x0001

/* See 1.5.9.7 CEETM Management Commands */
#define QM_CEETM_VERB_LFQMT_CONFIG	0x70
#define QM_CEETM_VERB_LFQMT_QUERY	0x71
#define QM_CEETM_VERB_CQ_CONFIG		0x72
#define QM_CEETM_VERB_CQ_QUERY		0x73
#define QM_CEETM_VERB_DCT_CONFIG	0x74
#define QM_CEETM_VERB_DCT_QUERY		0x75
#define QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG		0x76
#define QM_CEETM_VERB_CLASS_SCHEDULER_QUERY		0x77
#define QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG	0x78
#define QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY		0x79
#define QM_CEETM_VERB_CCGR_CONFIG			0x7A
#define QM_CEETM_VERB_CCGR_QUERY			0x7B
#define QM_CEETM_VERB_CQ_PEEK_POP_XFDRREAD		0x7C
#define QM_CEETM_VERB_STATISTICS_QUERY_WRITE		0x7D

/* See 1.5.8.5.1: "Initialize FQ" */
/* See 1.5.8.5.2: "Query FQ" */
/* See 1.5.8.5.3: "Query FQ Non-Programmable Fields" */
/* See 1.5.8.5.4: "Alter FQ State Commands " */
/* See 1.5.8.6.1: "Initialize/Modify CGR" */
/* See 1.5.8.6.2: "CGR Test Write" */
/* See 1.5.8.6.3: "Query CGR" */
/* See 1.5.8.6.4: "Query Congestion Group State" */
struct qm_mcr_initfq {
	u8 verb;
	u8 result;
	u8 __reserved1[62];
} __packed;
struct qm_mcr_queryfq {
	u8 verb;
	u8 result;
	u8 __reserved1[8];
	struct qm_fqd fqd;	/* the FQD fields are here */
	u8 __reserved2[30];
} __packed;
struct qm_mcr_queryfq_np {
	u8 verb;
	u8 result;
	u8 __reserved1;
	u8 state;	/* QM_MCR_NP_STATE_*** */
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u8 __reserved2;
	u32 fqd_link:24;
	u16 __reserved3:2;
	u16 odp_seq:14;
	u16 __reserved4:2;
	u16 orp_nesn:14;
	u16 __reserved5:1;
	u16 orp_ea_hseq:15;
	u16 __reserved6:1;
	u16 orp_ea_tseq:15;
	u8 __reserved7;
	u32 orp_ea_hptr:24;
	u8 __reserved8;
	u32 orp_ea_tptr:24;
	u8 __reserved9;
	u32 pfdr_hptr:24;
	u8 __reserved10;
	u32 pfdr_tptr:24;
	u8 __reserved11[5];
	u8 __reserved12:7;
	u8 is:1;
	u16 ics_surp;
	u32 byte_cnt;
	u8 __reserved13;
	u32 frm_cnt:24;
	u32 __reserved14;
	u16 ra1_sfdr;	/* QM_MCR_NP_RA1_*** */
	u16 ra2_sfdr;	/* QM_MCR_NP_RA2_*** */
	u16 __reserved15;
	u16 od1_sfdr;	/* QM_MCR_NP_OD1_*** */
	u16 od2_sfdr;	/* QM_MCR_NP_OD2_*** */
	u16 od3_sfdr;	/* QM_MCR_NP_OD3_*** */
#else
	u8 __reserved2;
	u32 fqd_link:24;

	u16 odp_seq:14;
	u16 __reserved3:2;

	u16 orp_nesn:14;
	u16 __reserved4:2;

	u16 orp_ea_hseq:15;
	u16 __reserved5:1;

	u16 orp_ea_tseq:15;
	u16 __reserved6:1;

	u8 __reserved7;
	u32 orp_ea_hptr:24;

	u8 __reserved8;
	u32 orp_ea_tptr:24;

	u8 __reserved9;
	u32 pfdr_hptr:24;

	u8 __reserved10;
	u32 pfdr_tptr:24;

	u8 __reserved11[5];
	u8 is:1;
	u8 __reserved12:7;
	u16 ics_surp;
	u32 byte_cnt;
	u8 __reserved13;
	u32 frm_cnt:24;
	u32 __reserved14;
	u16 ra1_sfdr;	/* QM_MCR_NP_RA1_*** */
	u16 ra2_sfdr;	/* QM_MCR_NP_RA2_*** */
	u16 __reserved15;
	u16 od1_sfdr;	/* QM_MCR_NP_OD1_*** */
	u16 od2_sfdr;	/* QM_MCR_NP_OD2_*** */
	u16 od3_sfdr;	/* QM_MCR_NP_OD3_*** */
#endif
} __packed;


struct qm_mcr_alterfq {
	u8 verb;
	u8 result;
	u8 fqs;		/* Frame Queue Status */
	u8 __reserved1[61];
} __packed;
struct qm_mcr_initcgr {
	u8 verb;
	u8 result;
	u8 __reserved1[62];
} __packed;
struct qm_mcr_cgrtestwrite {
	u8 verb;
	u8 result;
	u16 __reserved1;
	struct __qm_mc_cgr cgr; /* CGR fields */
	u8 __reserved2[3];
	u32 __reserved3:24;
	u32 i_bcnt_hi:8;/* high 8-bits of 40-bit "Instant" */
	u32 i_bcnt_lo;	/* low 32-bits of 40-bit */
	u32 __reserved4:24;
	u32 a_bcnt_hi:8;/* high 8-bits of 40-bit "Average" */
	u32 a_bcnt_lo;	/* low 32-bits of 40-bit */
	u16 lgt;	/* Last Group Tick */
	u16 wr_prob_g;
	u16 wr_prob_y;
	u16 wr_prob_r;
	u8 __reserved5[8];
} __packed;
struct qm_mcr_querycgr {
	u8 verb;
	u8 result;
	u16 __reserved1;
	struct __qm_mc_cgr cgr; /* CGR fields */
	u8 __reserved2[3];
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u32 __reserved3:24;
			u32 i_bcnt_hi:8;/* high 8-bits of 40-bit "Instant" */
			u32 i_bcnt_lo;	/* low 32-bits of 40-bit */
#else
			u32 i_bcnt_lo;	/* low 32-bits of 40-bit */
			u32 i_bcnt_hi:8;/* high 8-bits of 40-bit "Instant" */
			u32 __reserved3:24;
#endif
		};
		u64 i_bcnt;
	};
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u32 __reserved4:24;
			u32 a_bcnt_hi:8;/* high 8-bits of 40-bit "Average" */
			u32 a_bcnt_lo;	/* low 32-bits of 40-bit */
#else
			u32 a_bcnt_lo;	/* low 32-bits of 40-bit */
			u32 a_bcnt_hi:8;/* high 8-bits of 40-bit "Average" */
			u32 __reserved4:24;
#endif
		};
		u64 a_bcnt;
	};
	union {
		u32 cscn_targ_swp[4];
		u8 __reserved5[16];
	};
} __packed;
static inline u64 qm_mcr_querycgr_i_get64(const struct qm_mcr_querycgr *q)
{
	return be64_to_cpu(q->i_bcnt);
}
static inline u64 qm_mcr_querycgr_a_get64(const struct qm_mcr_querycgr *q)
{
	return be64_to_cpu(q->a_bcnt);
}
static inline u64 qm_mcr_cgrtestwrite_i_get64(
					const struct qm_mcr_cgrtestwrite *q)
{
	return be64_to_cpu(((u64)q->i_bcnt_hi << 32) | (u64)q->i_bcnt_lo);
}
static inline u64 qm_mcr_cgrtestwrite_a_get64(
					const struct qm_mcr_cgrtestwrite *q)
{
	return be64_to_cpu(((u64)q->a_bcnt_hi << 32) | (u64)q->a_bcnt_lo);
}
/* Macro, so we compile better if 'v' isn't always 64-bit */
#define qm_mcr_querycgr_i_set64(q, v) \
	do { \
		struct qm_mcr_querycgr *__q931 = (fd); \
		__q931->i_bcnt_hi = upper_32_bits(v); \
		__q931->i_bcnt_lo = lower_32_bits(v); \
	} while (0)
#define qm_mcr_querycgr_a_set64(q, v) \
	do { \
		struct qm_mcr_querycgr *__q931 = (fd); \
		__q931->a_bcnt_hi = upper_32_bits(v); \
		__q931->a_bcnt_lo = lower_32_bits(v); \
	} while (0)
struct __qm_mcr_querycongestion {
	u32 __state[8];
};
struct qm_mcr_querycongestion {
	u8 verb;
	u8 result;
	u8 __reserved[30];
	/* Access this struct using QM_MCR_QUERYCONGESTION() */
	struct __qm_mcr_querycongestion state;
} __packed;
struct qm_mcr_querywq {
	u8 verb;
	u8 result;
	union {
		u16 channel_wq; /* ignores wq (3 lsbits) */
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u16 id:13; /* qm_channel */
			u16 __reserved:3;
#else
			u16 __reserved:3;
			u16 id:13; /* qm_channel */
#endif
		} __packed channel;
	};
	u8 __reserved[28];
	u32 wq_len[8];
} __packed;

/* QMAN CEETM Management Command Response */
struct qm_mcr_ceetm_lfqmt_config {
	u8 verb;
	u8 result;
	u8 __reserved1[62];
} __packed;
struct qm_mcr_ceetm_lfqmt_query {
	u8 verb;
	u8 result;
	u8 __reserved1[8];
	u16 cqid;
	u8 __reserved2[2];
	u16 dctidx;
	u8 __reserved3[2];
	u16 ccgid;
	u8 __reserved4[44];
} __packed;

struct qm_mcr_ceetm_cq_config {
	u8 verb;
	u8 result;
	u8 __reserved1[62];
} __packed;

struct qm_mcr_ceetm_cq_query {
	u8 verb;
	u8 result;
	u8 __reserved1[4];
	u16 ccgid;
	u16 state;
	u32 pfdr_hptr:24;
	u32 pfdr_tptr:24;
	u16 od1_xsfdr;
	u16 od2_xsfdr;
	u16 od3_xsfdr;
	u16 od4_xsfdr;
	u16 od5_xsfdr;
	u16 od6_xsfdr;
	u16 ra1_xsfdr;
	u16 ra2_xsfdr;
	u8 __reserved2;
	u32 frm_cnt:24;
	u8 __reserved333[28];
} __packed;

struct qm_mcr_ceetm_dct_config {
	u8 verb;
	u8 result;
	u8 __reserved1[62];
} __packed;

struct qm_mcr_ceetm_dct_query {
	u8 verb;
	u8 result;
	u8 __reserved1[18];
	u32 context_b;
	u64 context_a;
	u8 __reserved2[32];
} __packed;

struct qm_mcr_ceetm_class_scheduler_config {
	u8 verb;
	u8 result;
	u8 __reserved1[62];
} __packed;

struct qm_mcr_ceetm_class_scheduler_query {
	u8 verb;
	u8 result;
	u8 __reserved1[9];
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u8 gpc_reserved:1;
	u8 gpc_combine_flag:1;
	u8 gpc_prio_b:3;
	u8 gpc_prio_a:3;
#else
	u8 gpc_prio_a:3;
	u8 gpc_prio_b:3;
	u8 gpc_combine_flag:1;
	u8 gpc_reserved:1;
#endif
	u16 crem;
	u16 erem;
	u8 w[8];
	u8 __reserved2[5];
	u32 wbfslist:24;
	u32 d8;
	u32 d9;
	u32 d10;
	u32 d11;
	u32 d12;
	u32 d13;
	u32 d14;
	u32 d15;
} __packed;

struct qm_mcr_ceetm_mapping_shaper_tcfc_config {
	u8 verb;
	u8 result;
	u16 cid;
	u8 __reserved2[60];
} __packed;

struct qm_mcr_ceetm_mapping_shaper_tcfc_query {
	u8 verb;
	u8 result;
	u16 cid;
	u8 __reserved1;
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 map_shaped:1;
			u8 map_reserved:4;
			u8 map_lni_id:3;
#else
			u8 map_lni_id:3;
			u8 map_reserved:4;
			u8 map_shaped:1;
#endif
			u8 __reserved2[58];
		} __packed channel_mapping_query;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 map_reserved:5;
			u8 map_lni_id:3;
#else
			u8 map_lni_id:3;
			u8 map_reserved:5;
#endif
			u8 __reserved2[58];
		} __packed sp_mapping_query;
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 cpl:1;
			u8 cpl_reserved:2;
			u8 oal:5;
#else
			u8 oal:5;
			u8 cpl_reserved:2;
			u8 cpl:1;
#endif
			u32 crtcr:24;
			u32 ertcr:24;
			u16 crtbl;
			u16 ertbl;
			u8 mps;
			u8 __reserved2[15];
			u32 crat;
			u32 erat;
			u8 __reserved3[24];
		} __packed shaper_query;
		struct {
			u8 __reserved1[11];
			u64 lnitcfcc;
			u8 __reserved3[40];
		} __packed tcfc_query;
	};
} __packed;

struct qm_mcr_ceetm_ccgr_config {
	u8 verb;
	u8 result;
	u8 __reserved1[46];
	union {
		u8 __reserved2[8];
		struct {
			u16 timestamp;
			u16 wr_porb_g;
			u16 wr_prob_y;
			u16 wr_prob_r;
		} __packed test_write;
	};
	u8 __reserved3[8];
} __packed;

struct qm_mcr_ceetm_ccgr_query {
	u8 verb;
	u8 result;
	u8 __reserved1[6];
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			u8 ctl_reserved:1;
			u8 ctl_wr_en_g:1;
			u8 ctl_wr_en_y:1;
			u8 ctl_wr_en_r:1;
			u8 ctl_td_en:1;
			u8 ctl_td_mode:1;
			u8 ctl_cscn_en:1;
			u8 ctl_mode:1;
#else
			u8 ctl_mode:1;
			u8 ctl_cscn_en:1;
			u8 ctl_td_mode:1;
			u8 ctl_td_en:1;
			u8 ctl_wr_en_r:1;
			u8 ctl_wr_en_y:1;
			u8 ctl_wr_en_g:1;
			u8 ctl_reserved:1;
#endif
			u8 cdv;
			u8 __reserved2[2];
			u8 oal;
			u8 __reserved3;
			struct qm_cgr_cs_thres cs_thres;
			struct qm_cgr_cs_thres cs_thres_x;
			struct qm_cgr_cs_thres td_thres;
			struct qm_cgr_wr_parm wr_parm_g;
			struct qm_cgr_wr_parm wr_parm_y;
			struct qm_cgr_wr_parm wr_parm_r;
			u16 cscn_targ_dcp;
			u8 dcp_lsn;
			u64 i_cnt:40;
			u8 __reserved4[3];
			u64 a_cnt:40;
			u32 cscn_targ_swp[4];
		} __packed cm_query;
		struct {
			u8 dnc;
			u8 dn0;
			u8 dn1;
			u64 dnba:40;
			u8 __reserved2[2];
			u16 dnth_0;
			u8 __reserved3[2];
			u16 dnth_1;
			u8 __reserved4[10];
			u16 dnacc_0;
			u8 __reserved5[2];
			u16 dnacc_1;
			u8 __reserved6[24];
		} __packed dn_query;
		struct {
			u8 __reserved2[24];
			struct  __qm_mcr_querycongestion state;
		} __packed congestion_state;

	};
} __packed;

struct qm_mcr_ceetm_cq_peek_pop_xsfdrread {
	u8 verb;
	u8 result;
	u8 stat;
	u8 __reserved1[11];
	u16 dctidx;
	struct qm_fd fd;
	u8 __reserved2[32];
} __packed __aligned(8);

struct qm_mcr_ceetm_statistics_query {
	u8 verb;
	u8 result;
	u8 __reserved1[17];
	u64 frm_cnt:40;
	u8 __reserved2[2];
	u64 byte_cnt:48;
	u8 __reserved3[32];
} __packed;

struct qm_mc_result {
	union {
		struct {
			u8 verb;
			u8 result;
			u8 __reserved1[62];
		};
		struct qm_mcr_initfq initfq;
		struct qm_mcr_queryfq queryfq;
		struct qm_mcr_queryfq_np queryfq_np;
		struct qm_mcr_alterfq alterfq;
		struct qm_mcr_initcgr initcgr;
		struct qm_mcr_cgrtestwrite cgrtestwrite;
		struct qm_mcr_querycgr querycgr;
		struct qm_mcr_querycongestion querycongestion;
		struct qm_mcr_querywq querywq;
		struct qm_mcr_ceetm_lfqmt_config lfqmt_config;
		struct qm_mcr_ceetm_lfqmt_query lfqmt_query;
		struct qm_mcr_ceetm_cq_config cq_config;
		struct qm_mcr_ceetm_cq_query cq_query;
		struct qm_mcr_ceetm_dct_config dct_config;
		struct qm_mcr_ceetm_dct_query dct_query;
		struct qm_mcr_ceetm_class_scheduler_config csch_config;
		struct qm_mcr_ceetm_class_scheduler_query csch_query;
		struct qm_mcr_ceetm_mapping_shaper_tcfc_config mst_config;
		struct qm_mcr_ceetm_mapping_shaper_tcfc_query mst_query;
		struct qm_mcr_ceetm_ccgr_config ccgr_config;
		struct qm_mcr_ceetm_ccgr_query ccgr_query;
		struct qm_mcr_ceetm_cq_peek_pop_xsfdrread cq_ppxr;
		struct qm_mcr_ceetm_statistics_query stats_query;
	};
} __packed;

#define QM_MCR_VERB_RRID		0x80
#define QM_MCR_VERB_MASK		QM_MCC_VERB_MASK
#define QM_MCR_VERB_INITFQ_PARKED	QM_MCC_VERB_INITFQ_PARKED
#define QM_MCR_VERB_INITFQ_SCHED	QM_MCC_VERB_INITFQ_SCHED
#define QM_MCR_VERB_QUERYFQ		QM_MCC_VERB_QUERYFQ
#define QM_MCR_VERB_QUERYFQ_NP		QM_MCC_VERB_QUERYFQ_NP
#define QM_MCR_VERB_QUERYWQ		QM_MCC_VERB_QUERYWQ
#define QM_MCR_VERB_QUERYWQ_DEDICATED	QM_MCC_VERB_QUERYWQ_DEDICATED
#define QM_MCR_VERB_ALTER_SCHED		QM_MCC_VERB_ALTER_SCHED
#define QM_MCR_VERB_ALTER_FE		QM_MCC_VERB_ALTER_FE
#define QM_MCR_VERB_ALTER_RETIRE	QM_MCC_VERB_ALTER_RETIRE
#define QM_MCR_VERB_ALTER_OOS		QM_MCC_VERB_ALTER_OOS
#define QM_MCR_RESULT_NULL		0x00
#define QM_MCR_RESULT_OK		0xf0
#define QM_MCR_RESULT_ERR_FQID		0xf1
#define QM_MCR_RESULT_ERR_FQSTATE	0xf2
#define QM_MCR_RESULT_ERR_NOTEMPTY	0xf3	/* OOS fails if FQ is !empty */
#define QM_MCR_RESULT_ERR_BADCHANNEL	0xf4
#define QM_MCR_RESULT_PENDING		0xf8
#define QM_MCR_RESULT_ERR_BADCOMMAND	0xff
#define QM_MCR_NP_STATE_FE		0x10
#define QM_MCR_NP_STATE_R		0x08
#define QM_MCR_NP_STATE_MASK		0x07	/* Reads FQD::STATE; */
#define QM_MCR_NP_STATE_OOS		0x00
#define QM_MCR_NP_STATE_RETIRED		0x01
#define QM_MCR_NP_STATE_TEN_SCHED	0x02
#define QM_MCR_NP_STATE_TRU_SCHED	0x03
#define QM_MCR_NP_STATE_PARKED		0x04
#define QM_MCR_NP_STATE_ACTIVE		0x05
#define QM_MCR_NP_PTR_MASK		0x07ff	/* for RA[12] & OD[123] */
#define QM_MCR_NP_RA1_NRA(v)		(((v) >> 14) & 0x3)	/* FQD::NRA */
#define QM_MCR_NP_RA2_IT(v)		(((v) >> 14) & 0x1)	/* FQD::IT */
#define QM_MCR_NP_OD1_NOD(v)		(((v) >> 14) & 0x3)	/* FQD::NOD */
#define QM_MCR_NP_OD3_NPC(v)		(((v) >> 14) & 0x3)	/* FQD::NPC */
#define QM_MCR_FQS_ORLPRESENT		0x02	/* ORL fragments to come */
#define QM_MCR_FQS_NOTEMPTY		0x01	/* FQ has enqueued frames */
/* This extracts the state for congestion group 'n' from a query response.
 * Eg.
 *   u8 cgr = [...];
 *   struct qm_mc_result *res = [...];
 *   printf("congestion group %d congestion state: %d\n", cgr,
 *       QM_MCR_QUERYCONGESTION(&res->querycongestion.state, cgr));
 */
#define __CGR_WORD(num)		(num >> 5)
#define __CGR_SHIFT(num)	(num & 0x1f)
#define __CGR_NUM		(sizeof(struct __qm_mcr_querycongestion) << 3)
static inline int QM_MCR_QUERYCONGESTION(struct __qm_mcr_querycongestion *p,
					u8 cgr)
{
	return p->__state[__CGR_WORD(cgr)] & (0x80000000 >> __CGR_SHIFT(cgr));
}


/*********************/
/* Utility interface */
/*********************/

/* Represents an allocator over a range of FQIDs. NB, accesses are not locked,
 * spinlock them yourself if needed. */
struct qman_fqid_pool;

/* Create/destroy a FQID pool, num must be a multiple of 32. NB, _destroy()
 * always succeeds, but returns non-zero if there were "leaked" FQID
 * allocations. */
struct qman_fqid_pool *qman_fqid_pool_create(u32 fqid_start, u32 num);
int qman_fqid_pool_destroy(struct qman_fqid_pool *pool);
/* Alloc/free a FQID from the range. _alloc() returns zero for success. */
int qman_fqid_pool_alloc(struct qman_fqid_pool *pool, u32 *fqid);
void qman_fqid_pool_free(struct qman_fqid_pool *pool, u32 fqid);
u32 qman_fqid_pool_used(struct qman_fqid_pool *pool);

/*******************************************************************/
/* Managed (aka "shared" or "mux/demux") portal, high-level i/face */
/*******************************************************************/

	/* Portal and Frame Queues */
	/* ----------------------- */
/* Represents a managed portal */
struct qman_portal;

/* This object type represents Qman frame queue descriptors (FQD), it is
 * cacheline-aligned, and initialised by qman_create_fq(). The structure is
 * defined further down. */
struct qman_fq;

/* This object type represents a Qman congestion group, it is defined further
 * down. */
struct qman_cgr;

struct qman_portal_config {
	/* If the caller enables DQRR stashing (and thus wishes to operate the
	 * portal from only one cpu), this is the logical CPU that the portal
	 * will stash to. Whether stashing is enabled or not, this setting is
	 * also used for any "core-affine" portals, ie. default portals
	 * associated to the corresponding cpu. -1 implies that there is no core
	 * affinity configured. */
	int cpu;
	/* portal interrupt line */
	int irq;
	/* the unique index of this portal */
	u32 index;
	/* Is this portal shared? (If so, it has coarser locking and demuxes
	 * processing on behalf of other CPUs.) */
	int is_shared;
	/* The portal's dedicated channel id, use this value for initialising
	 * frame queues to target this portal when scheduled. */
	u16 channel;
	/* A mask of which pool channels this portal has dequeue access to
	 * (using QM_SDQCR_CHANNELS_POOL(n) for the bitmask) */
	u32 pools;
};

/* This enum, and the callback type that returns it, are used when handling
 * dequeued frames via DQRR. Note that for "null" callbacks registered with the
 * portal object (for handling dequeues that do not demux because contextB is
 * NULL), the return value *MUST* be qman_cb_dqrr_consume. */
enum qman_cb_dqrr_result {
	/* DQRR entry can be consumed */
	qman_cb_dqrr_consume,
	/* Like _consume, but requests parking - FQ must be held-active */
	qman_cb_dqrr_park,
	/* Does not consume, for DCA mode only. This allows out-of-order
	 * consumes by explicit calls to qman_dca() and/or the use of implicit
	 * DCA via EQCR entries. */
	qman_cb_dqrr_defer,
	/* Stop processing without consuming this ring entry. Exits the current
	 * qman_poll_dqrr() or interrupt-handling, as appropriate. If within an
	 * interrupt handler, the callback would typically call
	 * qman_irqsource_remove(QM_PIRQ_DQRI) before returning this value,
	 * otherwise the interrupt will reassert immediately. */
	qman_cb_dqrr_stop,
	/* Like qman_cb_dqrr_stop, but consumes the current entry. */
	qman_cb_dqrr_consume_stop
};
typedef enum qman_cb_dqrr_result (*qman_cb_dqrr)(struct qman_portal *qm,
					struct qman_fq *fq,
					const struct qm_dqrr_entry *dqrr);

/* This callback type is used when handling ERNs, FQRNs and FQRLs via MR. They
 * are always consumed after the callback returns. */
typedef void (*qman_cb_mr)(struct qman_portal *qm, struct qman_fq *fq,
				const struct qm_mr_entry *msg);

/* This callback type is used when handling DCP ERNs */
typedef void (*qman_cb_dc_ern)(struct qman_portal *qm,
				const struct qm_mr_entry *msg);

/* s/w-visible states. Ie. tentatively scheduled + truly scheduled + active +
 * held-active + held-suspended are just "sched". Things like "retired" will not
 * be assumed until it is complete (ie. QMAN_FQ_STATE_CHANGING is set until
 * then, to indicate it's completing and to gate attempts to retry the retire
 * command). Note, park commands do not set QMAN_FQ_STATE_CHANGING because it's
 * technically impossible in the case of enqueue DCAs (which refer to DQRR ring
 * index rather than the FQ that ring entry corresponds to), so repeated park
 * commands are allowed (if you're silly enough to try) but won't change FQ
 * state, and the resulting park notifications move FQs from "sched" to
 * "parked". */
enum qman_fq_state {
	qman_fq_state_oos,
	qman_fq_state_parked,
	qman_fq_state_sched,
	qman_fq_state_retired
};

/* Frame queue objects (struct qman_fq) are stored within memory passed to
 * qman_create_fq(), as this allows stashing of caller-provided demux callback
 * pointers at no extra cost to stashing of (driver-internal) FQ state. If the
 * caller wishes to add per-FQ state and have it benefit from dequeue-stashing,
 * they should;
 *
 * (a) extend the qman_fq structure with their state; eg.
 *
 *     // myfq is allocated and driver_fq callbacks filled in;
 *     struct my_fq {
 *         struct qman_fq base;
 *         int an_extra_field;
 *         [ ... add other fields to be associated with each FQ ...]
 *     } *myfq = some_my_fq_allocator();
 *     struct qman_fq *fq = qman_create_fq(fqid, flags, &myfq->base);
 *
 *     // in a dequeue callback, access extra fields from 'fq' via a cast;
 *     struct my_fq *myfq = (struct my_fq *)fq;
 *     do_something_with(myfq->an_extra_field);
 *     [...]
 *
 * (b) when and if configuring the FQ for context stashing, specify how ever
 *     many cachelines are required to stash 'struct my_fq', to accelerate not
 *     only the Qman driver but the callback as well.
 */

struct qman_fq_cb {
	qman_cb_dqrr dqrr;      /* for dequeued frames */
	qman_cb_mr ern;         /* for s/w ERNs */
	qman_cb_mr fqs;         /* frame-queue state changes*/
};

struct qman_fq {
	/* Caller of qman_create_fq() provides these demux callbacks */
	struct qman_fq_cb cb;
	/* These are internal to the driver, don't touch. In particular, they
	 * may change, be removed, or extended (so you shouldn't rely on
	 * sizeof(qman_fq) being a constant). */
	spinlock_t fqlock;
	u32 fqid;
	volatile unsigned long flags;
	enum qman_fq_state state;
	int cgr_groupid;
	struct rb_node node;
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	u32 key;
#endif
};

/* This callback type is used when handling congestion group entry/exit.
 * 'congested' is non-zero on congestion-entry, and zero on congestion-exit. */
typedef void (*qman_cb_cgr)(struct qman_portal *qm,
			struct qman_cgr *cgr, int congested);

struct qman_cgr {
	/* Set these prior to qman_create_cgr() */
	u32 cgrid; /* 0..255, but u32 to allow specials like -1, 256, etc.*/
	qman_cb_cgr cb;
	/* These are private to the driver */
	u16 chan; /* portal channel this object is created on */
	struct list_head node;
};

/* Flags to qman_create_fq() */
#define QMAN_FQ_FLAG_NO_ENQUEUE      0x00000001 /* can't enqueue */
#define QMAN_FQ_FLAG_NO_MODIFY       0x00000002 /* can only enqueue */
#define QMAN_FQ_FLAG_TO_DCPORTAL     0x00000004 /* consumed by CAAM/PME/Fman */
#define QMAN_FQ_FLAG_LOCKED          0x00000008 /* multi-core locking */
#define QMAN_FQ_FLAG_AS_IS           0x00000010 /* query h/w state */
#define QMAN_FQ_FLAG_DYNAMIC_FQID    0x00000020 /* (de)allocate fqid */

/* Flags to qman_destroy_fq() */
#define QMAN_FQ_DESTROY_PARKED       0x00000001 /* FQ can be parked or OOS */

/* Flags from qman_fq_state() */
#define QMAN_FQ_STATE_CHANGING       0x80000000 /* 'state' is changing */
#define QMAN_FQ_STATE_NE             0x40000000 /* retired FQ isn't empty */
#define QMAN_FQ_STATE_ORL            0x20000000 /* retired FQ has ORL */
#define QMAN_FQ_STATE_BLOCKOOS       0xe0000000 /* if any are set, no OOS */
#define QMAN_FQ_STATE_CGR_EN         0x10000000 /* CGR enabled */
#define QMAN_FQ_STATE_VDQCR          0x08000000 /* being volatile dequeued */

/* Flags to qman_init_fq() */
#define QMAN_INITFQ_FLAG_SCHED       0x00000001 /* schedule rather than park */
#define QMAN_INITFQ_FLAG_LOCAL       0x00000004 /* set dest portal */

/* Flags to qman_volatile_dequeue() */
#ifdef CONFIG_FSL_DPA_CAN_WAIT
#define QMAN_VOLATILE_FLAG_WAIT      0x00000001 /* wait if VDQCR is in use */
#define QMAN_VOLATILE_FLAG_WAIT_INT  0x00000002 /* if wait, interruptible? */
#define QMAN_VOLATILE_FLAG_FINISH    0x00000004 /* wait till VDQCR completes */
#endif

/* Flags to qman_enqueue(). NB, the strange numbering is to align with hardware,
 * bit-wise. (NB: the PME API is sensitive to these precise numberings too, so
 * any change here should be audited in PME.) */
#ifdef CONFIG_FSL_DPA_CAN_WAIT
#define QMAN_ENQUEUE_FLAG_WAIT       0x00010000 /* wait if EQCR is full */
#define QMAN_ENQUEUE_FLAG_WAIT_INT   0x00020000 /* if wait, interruptible? */
#ifdef CONFIG_FSL_DPA_CAN_WAIT_SYNC
#define QMAN_ENQUEUE_FLAG_WAIT_SYNC  0x00000004 /* if wait, until consumed? */
#endif
#endif
#define QMAN_ENQUEUE_FLAG_WATCH_CGR  0x00080000 /* watch congestion state */
#define QMAN_ENQUEUE_FLAG_DCA        0x00008000 /* perform enqueue-DCA */
#define QMAN_ENQUEUE_FLAG_DCA_PARK   0x00004000 /* If DCA, requests park */
#define QMAN_ENQUEUE_FLAG_DCA_PTR(p)		/* If DCA, p is DQRR entry */ \
		(((u32)(p) << 2) & 0x00000f00)
#define QMAN_ENQUEUE_FLAG_C_GREEN    0x00000000 /* choose one C_*** flag */
#define QMAN_ENQUEUE_FLAG_C_YELLOW   0x00000008
#define QMAN_ENQUEUE_FLAG_C_RED      0x00000010
#define QMAN_ENQUEUE_FLAG_C_OVERRIDE 0x00000018
/* For the ORP-specific qman_enqueue_orp() variant;
 * - this flag indicates "Not Last In Sequence", ie. all but the final fragment
 *   of a frame. */
#define QMAN_ENQUEUE_FLAG_NLIS       0x01000000
/* - this flag performs no enqueue but fills in an ORP sequence number that
 *   would otherwise block it (eg. if a frame has been dropped). */
#define QMAN_ENQUEUE_FLAG_HOLE       0x02000000
/* - this flag performs no enqueue but advances NESN to the given sequence
 *   number. */
#define QMAN_ENQUEUE_FLAG_NESN       0x04000000

/* Flags to qman_modify_cgr() */
#define QMAN_CGR_FLAG_USE_INIT       0x00000001
#define QMAN_CGR_MODE_FRAME          0x00000001

	/* Portal Management */
	/* ----------------- */
/**
 * qman_get_portal_config - get portal configuration settings
 *
 * This returns a read-only view of the current cpu's affine portal settings.
 */
const struct qman_portal_config *qman_get_portal_config(void);

/**
 * qman_irqsource_get - return the portal work that is interrupt-driven
 *
 * Returns a bitmask of QM_PIRQ_**I processing sources that are currently
 * enabled for interrupt handling on the current cpu's affine portal. These
 * sources will trigger the portal interrupt and the interrupt handler (or a
 * tasklet/bottom-half it defers to) will perform the corresponding processing
 * work. The qman_poll_***() functions will only process sources that are not in
 * this bitmask. If the current CPU is sharing a portal hosted on another CPU,
 * this always returns zero.
 */
u32 qman_irqsource_get(void);

/**
 * qman_irqsource_add - add processing sources to be interrupt-driven
 * @bits: bitmask of QM_PIRQ_**I processing sources
 *
 * Adds processing sources that should be interrupt-driven (rather than
 * processed via qman_poll_***() functions). Returns zero for success, or
 * -EINVAL if the current CPU is sharing a portal hosted on another CPU.
 */
int qman_irqsource_add(u32 bits);

/**
 * qman_irqsource_remove - remove processing sources from being interrupt-driven
 * @bits: bitmask of QM_PIRQ_**I processing sources
 *
 * Removes processing sources from being interrupt-driven, so that they will
 * instead be processed via qman_poll_***() functions. Returns zero for success,
 * or -EINVAL if the current CPU is sharing a portal hosted on another CPU.
 */
int qman_irqsource_remove(u32 bits);

/**
 * qman_affine_cpus - return a mask of cpus that have affine portals
 */
const cpumask_t *qman_affine_cpus(void);

/**
 * qman_affine_channel - return the channel ID of an portal
 * @cpu: the cpu whose affine portal is the subject of the query
 *
 * If @cpu is -1, the affine portal for the current CPU will be used. It is a
 * bug to call this function for any value of @cpu (other than -1) that is not a
 * member of the mask returned from qman_affine_cpus().
 */
u16 qman_affine_channel(int cpu);

/**
 * qman_get_affine_portal - return the portal pointer affine to cpu
 * @cpu: the cpu whose affine portal is the subject of the query
 *
 */
void *qman_get_affine_portal(int cpu);

/**
 * qman_poll_dqrr - process DQRR (fast-path) entries
 * @limit: the maximum number of DQRR entries to process
 *
 * Use of this function requires that DQRR processing not be interrupt-driven.
 * Ie. the value returned by qman_irqsource_get() should not include
 * QM_PIRQ_DQRI. If the current CPU is sharing a portal hosted on another CPU,
 * this function will return -EINVAL, otherwise the return value is >=0 and
 * represents the number of DQRR entries processed.
 */
int qman_poll_dqrr(unsigned int limit);

/**
 * qman_poll_slow - process anything (except DQRR) that isn't interrupt-driven.
 *
 * This function does any portal processing that isn't interrupt-driven. If the
 * current CPU is sharing a portal hosted on another CPU, this function will
 * return (u32)-1, otherwise the return value is a bitmask of QM_PIRQ_* sources
 * indicating what interrupt sources were actually processed by the call.
 */
u32 qman_poll_slow(void);

/**
 * qman_poll - legacy wrapper for qman_poll_dqrr() and qman_poll_slow()
 *
 * Dispatcher logic on a cpu can use this to trigger any maintenance of the
 * affine portal. There are two classes of portal processing in question;
 * fast-path (which involves demuxing dequeue ring (DQRR) entries and tracking
 * enqueue ring (EQCR) consumption), and slow-path (which involves EQCR
 * thresholds, congestion state changes, etc). This function does whatever
 * processing is not triggered by interrupts.
 *
 * Note, if DQRR and some slow-path processing are poll-driven (rather than
 * interrupt-driven) then this function uses a heuristic to determine how often
 * to run slow-path processing - as slow-path processing introduces at least a
 * minimum latency each time it is run, whereas fast-path (DQRR) processing is
 * close to zero-cost if there is no work to be done. Applications can tune this
 * behaviour themselves by using qman_poll_dqrr() and qman_poll_slow() directly
 * rather than going via this wrapper.
 */
void qman_poll(void);

/**
 * qman_stop_dequeues - Stop h/w dequeuing to the s/w portal
 *
 * Disables DQRR processing of the portal. This is reference-counted, so
 * qman_start_dequeues() must be called as many times as qman_stop_dequeues() to
 * truly re-enable dequeuing.
 */
void qman_stop_dequeues(void);

/**
 * qman_start_dequeues - (Re)start h/w dequeuing to the s/w portal
 *
 * Enables DQRR processing of the portal. This is reference-counted, so
 * qman_start_dequeues() must be called as many times as qman_stop_dequeues() to
 * truly re-enable dequeuing.
 */
void qman_start_dequeues(void);

/**
 * qman_static_dequeue_add - Add pool channels to the portal SDQCR
 * @pools: bit-mask of pool channels, using QM_SDQCR_CHANNELS_POOL(n)
 *
 * Adds a set of pool channels to the portal's static dequeue command register
 * (SDQCR). The requested pools are limited to those the portal has dequeue
 * access to.
 */
void qman_static_dequeue_add(u32 pools);

/**
 * qman_static_dequeue_del - Remove pool channels from the portal SDQCR
 * @pools: bit-mask of pool channels, using QM_SDQCR_CHANNELS_POOL(n)
 *
 * Removes a set of pool channels from the portal's static dequeue command
 * register (SDQCR). The requested pools are limited to those the portal has
 * dequeue access to.
 */
void qman_static_dequeue_del(u32 pools);

/**
 * qman_static_dequeue_get - return the portal's current SDQCR
 *
 * Returns the portal's current static dequeue command register (SDQCR). The
 * entire register is returned, so if only the currently-enabled pool channels
 * are desired, mask the return value with QM_SDQCR_CHANNELS_POOL_MASK.
 */
u32 qman_static_dequeue_get(void);

/**
 * qman_dca - Perform a Discrete Consumption Acknowledgement
 * @dq: the DQRR entry to be consumed
 * @park_request: indicates whether the held-active @fq should be parked
 *
 * Only allowed in DCA-mode portals, for DQRR entries whose handler callback had
 * previously returned 'qman_cb_dqrr_defer'. NB, as with the other APIs, this
 * does not take a 'portal' argument but implies the core affine portal from the
 * cpu that is currently executing the function. For reasons of locking, this
 * function must be called from the same CPU as that which processed the DQRR
 * entry in the first place.
 */
void qman_dca(struct qm_dqrr_entry *dq, int park_request);

/**
 * qman_eqcr_is_empty - Determine if portal's EQCR is empty
 *
 * For use in situations where a cpu-affine caller needs to determine when all
 * enqueues for the local portal have been processed by Qman but can't use the
 * QMAN_ENQUEUE_FLAG_WAIT_SYNC flag to do this from the final qman_enqueue().
 * The function forces tracking of EQCR consumption (which normally doesn't
 * happen until enqueue processing needs to find space to put new enqueue
 * commands), and returns zero if the ring still has unprocessed entries,
 * non-zero if it is empty.
 */
int qman_eqcr_is_empty(void);

/**
 * qman_set_dc_ern - Set the handler for DCP enqueue rejection notifications
 * @handler: callback for processing DCP ERNs
 * @affine: whether this handler is specific to the locally affine portal
 *
 * If a hardware block's interface to Qman (ie. its direct-connect portal, or
 * DCP) is configured not to receive enqueue rejections, then any enqueues
 * through that DCP that are rejected will be sent to a given software portal.
 * If @affine is non-zero, then this handler will only be used for DCP ERNs
 * received on the portal affine to the current CPU. If multiple CPUs share a
 * portal and they all call this function, they will be setting the handler for
 * the same portal! If @affine is zero, then this handler will be global to all
 * portals handled by this instance of the driver. Only those portals that do
 * not have their own affine handler will use the global handler.
 */
void qman_set_dc_ern(qman_cb_dc_ern handler, int affine);

	/* FQ management */
	/* ------------- */
/**
 * qman_create_fq - Allocates a FQ
 * @fqid: the index of the FQD to encapsulate, must be "Out of Service"
 * @flags: bit-mask of QMAN_FQ_FLAG_*** options
 * @fq: memory for storing the 'fq', with callbacks filled in
 *
 * Creates a frame queue object for the given @fqid, unless the
 * QMAN_FQ_FLAG_DYNAMIC_FQID flag is set in @flags, in which case a FQID is
 * dynamically allocated (or the function fails if none are available). Once
 * created, the caller should not touch the memory at 'fq' except as extended to
 * adjacent memory for user-defined fields (see the definition of "struct
 * qman_fq" for more info). NO_MODIFY is only intended for enqueuing to
 * pre-existing frame-queues that aren't to be otherwise interfered with, it
 * prevents all other modifications to the frame queue. The TO_DCPORTAL flag
 * causes the driver to honour any contextB modifications requested in the
 * qm_init_fq() API, as this indicates the frame queue will be consumed by a
 * direct-connect portal (PME, CAAM, or Fman). When frame queues are consumed by
 * software portals, the contextB field is controlled by the driver and can't be
 * modified by the caller. If the AS_IS flag is specified, management commands
 * will be used on portal @p to query state for frame queue @fqid and construct
 * a frame queue object based on that, rather than assuming/requiring that it be
 * Out of Service.
 */
int qman_create_fq(u32 fqid, u32 flags, struct qman_fq *fq);

/**
 * qman_destroy_fq - Deallocates a FQ
 * @fq: the frame queue object to release
 * @flags: bit-mask of QMAN_FQ_FREE_*** options
 *
 * The memory for this frame queue object ('fq' provided in qman_create_fq()) is
 * not deallocated but the caller regains ownership, to do with as desired. The
 * FQ must be in the 'out-of-service' state unless the QMAN_FQ_FREE_PARKED flag
 * is specified, in which case it may also be in the 'parked' state.
 */
void qman_destroy_fq(struct qman_fq *fq, u32 flags);

/**
 * qman_fq_fqid - Queries the frame queue ID of a FQ object
 * @fq: the frame queue object to query
 */
u32 qman_fq_fqid(struct qman_fq *fq);

/**
 * qman_fq_state - Queries the state of a FQ object
 * @fq: the frame queue object to query
 * @state: pointer to state enum to return the FQ scheduling state
 * @flags: pointer to state flags to receive QMAN_FQ_STATE_*** bitmask
 *
 * Queries the state of the FQ object, without performing any h/w commands.
 * This captures the state, as seen by the driver, at the time the function
 * executes.
 */
void qman_fq_state(struct qman_fq *fq, enum qman_fq_state *state, u32 *flags);

/**
 * qman_init_fq - Initialises FQ fields, leaves the FQ "parked" or "scheduled"
 * @fq: the frame queue object to modify, must be 'parked' or new.
 * @flags: bit-mask of QMAN_INITFQ_FLAG_*** options
 * @opts: the FQ-modification settings, as defined in the low-level API
 *
 * The @opts parameter comes from the low-level portal API. Select
 * QMAN_INITFQ_FLAG_SCHED in @flags to cause the frame queue to be scheduled
 * rather than parked. NB, @opts can be NULL.
 *
 * Note that some fields and options within @opts may be ignored or overwritten
 * by the driver;
 * 1. the 'count' and 'fqid' fields are always ignored (this operation only
 * affects one frame queue: @fq).
 * 2. the QM_INITFQ_WE_CONTEXTB option of the 'we_mask' field and the associated
 * 'fqd' structure's 'context_b' field are sometimes overwritten;
 *   - if @fq was not created with QMAN_FQ_FLAG_TO_DCPORTAL, then context_b is
 *     initialised to a value used by the driver for demux.
 *   - if context_b is initialised for demux, so is context_a in case stashing
 *     is requested (see item 4).
 * (So caller control of context_b is only possible for TO_DCPORTAL frame queue
 * objects.)
 * 3. if @flags contains QMAN_INITFQ_FLAG_LOCAL, the 'fqd' structure's
 * 'dest::channel' field will be overwritten to match the portal used to issue
 * the command. If the WE_DESTWQ write-enable bit had already been set by the
 * caller, the channel workqueue will be left as-is, otherwise the write-enable
 * bit is set and the workqueue is set to a default of 4. If the "LOCAL" flag
 * isn't set, the destination channel/workqueue fields and the write-enable bit
 * are left as-is.
 * 4. if the driver overwrites context_a/b for demux, then if
 * QM_INITFQ_WE_CONTEXTA is set, the driver will only overwrite
 * context_a.address fields and will leave the stashing fields provided by the
 * user alone, otherwise it will zero out the context_a.stashing fields.
 */
int qman_init_fq(struct qman_fq *fq, u32 flags, struct qm_mcc_initfq *opts);

/**
 * qman_schedule_fq - Schedules a FQ
 * @fq: the frame queue object to schedule, must be 'parked'
 *
 * Schedules the frame queue, which must be Parked, which takes it to
 * Tentatively-Scheduled or Truly-Scheduled depending on its fill-level.
 */
int qman_schedule_fq(struct qman_fq *fq);

/**
 * qman_retire_fq - Retires a FQ
 * @fq: the frame queue object to retire
 * @flags: FQ flags (as per qman_fq_state) if retirement completes immediately
 *
 * Retires the frame queue. This returns zero if it succeeds immediately, +1 if
 * the retirement was started asynchronously, otherwise it returns negative for
 * failure. When this function returns zero, @flags is set to indicate whether
 * the retired FQ is empty and/or whether it has any ORL fragments (to show up
 * as ERNs). Otherwise the corresponding flags will be known when a subsequent
 * FQRN message shows up on the portal's message ring.
 *
 * NB, if the retirement is asynchronous (the FQ was in the Truly Scheduled or
 * Active state), the completion will be via the message ring as a FQRN - but
 * the corresponding callback may occur before this function returns!! Ie. the
 * caller should be prepared to accept the callback as the function is called,
 * not only once it has returned.
 */
int qman_retire_fq(struct qman_fq *fq, u32 *flags);

/**
 * qman_oos_fq - Puts a FQ "out of service"
 * @fq: the frame queue object to be put out-of-service, must be 'retired'
 *
 * The frame queue must be retired and empty, and if any order restoration list
 * was released as ERNs at the time of retirement, they must all be consumed.
 */
int qman_oos_fq(struct qman_fq *fq);

/**
 * qman_fq_flow_control - Set the XON/XOFF state of a FQ
 * @fq: the frame queue object to be set to XON/XOFF state, must not be 'oos',
 * or 'retired' or 'parked' state
 * @xon: boolean to set fq in XON or XOFF state
 *
 * The frame should be in Tentatively Scheduled state or Truly Schedule sate,
 * otherwise the IFSI interrupt will be asserted.
 */
int qman_fq_flow_control(struct qman_fq *fq, int xon);

/**
 * qman_query_fq - Queries FQD fields (via h/w query command)
 * @fq: the frame queue object to be queried
 * @fqd: storage for the queried FQD fields
 */
int qman_query_fq(struct qman_fq *fq, struct qm_fqd *fqd);

/**
 * qman_query_fq_np - Queries non-programmable FQD fields
 * @fq: the frame queue object to be queried
 * @np: storage for the queried FQD fields
 */
int qman_query_fq_np(struct qman_fq *fq, struct qm_mcr_queryfq_np *np);

/**
 * qman_query_wq - Queries work queue lengths
 * @query_dedicated: If non-zero, query length of WQs in the channel dedicated
 *		to this software portal. Otherwise, query length of WQs in a
 *		channel  specified in wq.
 * @wq: storage for the queried WQs lengths. Also specified the channel to
 *	to query if query_dedicated is zero.
 */
int qman_query_wq(u8 query_dedicated, struct qm_mcr_querywq *wq);

/**
 * qman_volatile_dequeue - Issue a volatile dequeue command
 * @fq: the frame queue object to dequeue from
 * @flags: a bit-mask of QMAN_VOLATILE_FLAG_*** options
 * @vdqcr: bit mask of QM_VDQCR_*** options, as per qm_dqrr_vdqcr_set()
 *
 * Attempts to lock access to the portal's VDQCR volatile dequeue functionality.
 * The function will block and sleep if QMAN_VOLATILE_FLAG_WAIT is specified and
 * the VDQCR is already in use, otherwise returns non-zero for failure. If
 * QMAN_VOLATILE_FLAG_FINISH is specified, the function will only return once
 * the VDQCR command has finished executing (ie. once the callback for the last
 * DQRR entry resulting from the VDQCR command has been called). If not using
 * the FINISH flag, completion can be determined either by detecting the
 * presence of the QM_DQRR_STAT_UNSCHEDULED and QM_DQRR_STAT_DQCR_EXPIRED bits
 * in the "stat" field of the "struct qm_dqrr_entry" passed to the FQ's dequeue
 * callback, or by waiting for the QMAN_FQ_STATE_VDQCR bit to disappear from the
 * "flags" retrieved from qman_fq_state().
 */
int qman_volatile_dequeue(struct qman_fq *fq, u32 flags, u32 vdqcr);

/**
 * qman_enqueue - Enqueue a frame to a frame queue
 * @fq: the frame queue object to enqueue to
 * @fd: a descriptor of the frame to be enqueued
 * @flags: bit-mask of QMAN_ENQUEUE_FLAG_*** options
 *
 * Fills an entry in the EQCR of portal @qm to enqueue the frame described by
 * @fd. The descriptor details are copied from @fd to the EQCR entry, the 'pid'
 * field is ignored. The return value is non-zero on error, such as ring full
 * (and FLAG_WAIT not specified), congestion avoidance (FLAG_WATCH_CGR
 * specified), etc. If the ring is full and FLAG_WAIT is specified, this
 * function will block. If FLAG_INTERRUPT is set, the EQCI bit of the portal
 * interrupt will assert when Qman consumes the EQCR entry (subject to "status
 * disable", "enable", and "inhibit" registers). If FLAG_DCA is set, Qman will
 * perform an implied "discrete consumption acknowledgement" on the dequeue
 * ring's (DQRR) entry, at the ring index specified by the FLAG_DCA_IDX(x)
 * macro. (As an alternative to issuing explicit DCA actions on DQRR entries,
 * this implicit DCA can delay the release of a "held active" frame queue
 * corresponding to a DQRR entry until Qman consumes the EQCR entry - providing
 * order-preservation semantics in packet-forwarding scenarios.) If FLAG_DCA is
 * set, then FLAG_DCA_PARK can also be set to imply that the DQRR consumption
 * acknowledgement should "park request" the "held active" frame queue. Ie.
 * when the portal eventually releases that frame queue, it will be left in the
 * Parked state rather than Tentatively Scheduled or Truly Scheduled. If the
 * portal is watching congestion groups, the QMAN_ENQUEUE_FLAG_WATCH_CGR flag
 * is requested, and the FQ is a member of a congestion group, then this
 * function returns -EAGAIN if the congestion group is currently congested.
 * Note, this does not eliminate ERNs, as the async interface means we can be
 * sending enqueue commands to an un-congested FQ that becomes congested before
 * the enqueue commands are processed, but it does minimise needless thrashing
 * of an already busy hardware resource by throttling many of the to-be-dropped
 * enqueues "at the source".
 */
int qman_enqueue(struct qman_fq *fq, const struct qm_fd *fd, u32 flags);

typedef int (*qman_cb_precommit) (void *arg);
/**
 * qman_enqueue_precommit - Enqueue a frame to a frame queue and call cb
 * @fq: the frame queue object to enqueue to
 * @fd: a descriptor of the frame to be enqueued
 * @flags: bit-mask of QMAN_ENQUEUE_FLAG_*** options
 * @cb: user supplied callback function to invoke before writing commit verb.
 * @cb_arg: callback function argument
 *
 * This is similar to qman_enqueue except that it will invoke a user supplied
 * callback function just before writng the commit verb. This is useful
 * when the user want to do something *just before* enqueuing the request and
 * the enqueue can't fail.
 */
int qman_enqueue_precommit(struct qman_fq *fq, const struct qm_fd *fd,
		u32 flags, qman_cb_precommit cb, void *cb_arg);

/**
 * qman_enqueue_orp - Enqueue a frame to a frame queue using an ORP
 * @fq: the frame queue object to enqueue to
 * @fd: a descriptor of the frame to be enqueued
 * @flags: bit-mask of QMAN_ENQUEUE_FLAG_*** options
 * @orp: the frame queue object used as an order restoration point.
 * @orp_seqnum: the sequence number of this frame in the order restoration path
 *
 * Similar to qman_enqueue(), but with the addition of an Order Restoration
 * Point (@orp) and corresponding sequence number (@orp_seqnum) for this
 * enqueue operation to employ order restoration. Each frame queue object acts
 * as an Order Definition Point (ODP) by providing each frame dequeued from it
 * with an incrementing sequence number, this value is generally ignored unless
 * that sequence of dequeued frames will need order restoration later. Each
 * frame queue object also encapsulates an Order Restoration Point (ORP), which
 * is a re-assembly context for re-ordering frames relative to their sequence
 * numbers as they are enqueued. The ORP does not have to be within the frame
 * queue that receives the enqueued frame, in fact it is usually the frame
 * queue from which the frames were originally dequeued. For the purposes of
 * order restoration, multiple frames (or "fragments") can be enqueued for a
 * single sequence number by setting the QMAN_ENQUEUE_FLAG_NLIS flag for all
 * enqueues except the final fragment of a given sequence number. Ordering
 * between sequence numbers is guaranteed, even if fragments of different
 * sequence numbers are interlaced with one another. Fragments of the same
 * sequence number will retain the order in which they are enqueued. If no
 * enqueue is to performed, QMAN_ENQUEUE_FLAG_HOLE indicates that the given
 * sequence number is to be "skipped" by the ORP logic (eg. if a frame has been
 * dropped from a sequence), or QMAN_ENQUEUE_FLAG_NESN indicates that the given
 * sequence number should become the ORP's "Next Expected Sequence Number".
 *
 * Side note: a frame queue object can be used purely as an ORP, without
 * carrying any frames at all. Care should be taken not to deallocate a frame
 * queue object that is being actively used as an ORP, as a future allocation
 * of the frame queue object may start using the internal ORP before the
 * previous use has finished.
 */
int qman_enqueue_orp(struct qman_fq *fq, const struct qm_fd *fd, u32 flags,
			struct qman_fq *orp, u16 orp_seqnum);

/**
 * qman_alloc_fqid_range - Allocate a contiguous range of FQIDs
 * @result: is set by the API to the base FQID of the allocated range
 * @count: the number of FQIDs required
 * @align: required alignment of the allocated range
 * @partial: non-zero if the API can return fewer than @count FQIDs
 *
 * Returns the number of frame queues allocated, or a negative error code. If
 * @partial is non zero, the allocation request may return a smaller range of
 * FQs than requested (though alignment will be as requested). If @partial is
 * zero, the return value will either be 'count' or negative.
 */
int qman_alloc_fqid_range(u32 *result, u32 count, u32 align, int partial);
static inline int qman_alloc_fqid(u32 *result)
{
	int ret = qman_alloc_fqid_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}

/**
 * qman_release_fqid_range - Release the specified range of frame queue IDs
 * @fqid: the base FQID of the range to deallocate
 * @count: the number of FQIDs in the range
 *
 * This function can also be used to seed the allocator with ranges of FQIDs
 * that it can subsequently allocate from.
 */
void qman_release_fqid_range(u32 fqid, unsigned int count);
static inline void qman_release_fqid(u32 fqid)
{
	qman_release_fqid_range(fqid, 1);
}

void qman_seed_fqid_range(u32 fqid, unsigned int count);


int qman_shutdown_fq(u32 fqid);

/**
 * qman_reserve_fqid_range - Reserve the specified range of frame queue IDs
 * @fqid: the base FQID of the range to deallocate
 * @count: the number of FQIDs in the range
 */
int qman_reserve_fqid_range(u32 fqid, unsigned int count);
static inline int qman_reserve_fqid(u32 fqid)
{
	return qman_reserve_fqid_range(fqid, 1);
}

	/* Pool-channel management */
	/* ----------------------- */
/**
 * qman_alloc_pool_range - Allocate a contiguous range of pool-channel IDs
 * @result: is set by the API to the base pool-channel ID of the allocated range
 * @count: the number of pool-channel IDs required
 * @align: required alignment of the allocated range
 * @partial: non-zero if the API can return fewer than @count
 *
 * Returns the number of pool-channel IDs allocated, or a negative error code.
 * If @partial is non zero, the allocation request may return a smaller range of
 * than requested (though alignment will be as requested). If @partial is zero,
 * the return value will either be 'count' or negative.
 */
int qman_alloc_pool_range(u32 *result, u32 count, u32 align, int partial);
static inline int qman_alloc_pool(u32 *result)
{
	int ret = qman_alloc_pool_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}

/**
 * qman_release_pool_range - Release the specified range of pool-channel IDs
 * @id: the base pool-channel ID of the range to deallocate
 * @count: the number of pool-channel IDs in the range
 */
void qman_release_pool_range(u32 id, unsigned int count);
static inline void qman_release_pool(u32 id)
{
	qman_release_pool_range(id, 1);
}

/**
 * qman_reserve_pool_range - Reserve the specified range of pool-channel IDs
 * @id: the base pool-channel ID of the range to reserve
 * @count: the number of pool-channel IDs in the range
 */
int qman_reserve_pool_range(u32 id, unsigned int count);
static inline int qman_reserve_pool(u32 id)
{
	return qman_reserve_pool_range(id, 1);
}

void qman_seed_pool_range(u32 id, unsigned int count);

	/* CGR management */
	/* -------------- */
/**
 * qman_create_cgr - Register a congestion group object
 * @cgr: the 'cgr' object, with fields filled in
 * @flags: QMAN_CGR_FLAG_* values
 * @opts: optional state of CGR settings
 *
 * Registers this object to receiving congestion entry/exit callbacks on the
 * portal affine to the cpu portal on which this API is executed. If opts is
 * NULL then only the callback (cgr->cb) function is registered. If @flags
 * contains QMAN_CGR_FLAG_USE_INIT, then an init hw command (which will reset
 * any unspecified parameters) will be used rather than a modify hw hardware
 * (which only modifies the specified parameters).
 */
int qman_create_cgr(struct qman_cgr *cgr, u32 flags,
			struct qm_mcc_initcgr *opts);

/**
 * qman_create_cgr_to_dcp - Register a congestion group object to DCP portal
 * @cgr: the 'cgr' object, with fields filled in
 * @flags: QMAN_CGR_FLAG_* values
 * @dcp_portal: the DCP portal to which the cgr object is registered.
 * @opts: optional state of CGR settings
 *
 */
int qman_create_cgr_to_dcp(struct qman_cgr *cgr, u32 flags, u16 dcp_portal,
				struct qm_mcc_initcgr *opts);

/**
 * qman_delete_cgr - Deregisters a congestion group object
 * @cgr: the 'cgr' object to deregister
 *
 * "Unplugs" this CGR object from the portal affine to the cpu on which this API
 * is executed. This must be excuted on the same affine portal on which it was
 * created.
 */
int qman_delete_cgr(struct qman_cgr *cgr);

/**
 * qman_delete_cgr_safe - Deregisters a congestion group object from any CPU
 * @cgr: the 'cgr' object to deregister
 *
 * This will select the proper CPU and run there qman_delete_cgr().
 */
void qman_delete_cgr_safe(struct qman_cgr *cgr);

/**
 * qman_modify_cgr - Modify CGR fields
 * @cgr: the 'cgr' object to modify
 * @flags: QMAN_CGR_FLAG_* values
 * @opts: the CGR-modification settings
 *
 * The @opts parameter comes from the low-level portal API, and can be NULL.
 * Note that some fields and options within @opts may be ignored or overwritten
 * by the driver, in particular the 'cgrid' field is ignored (this operation
 * only affects the given CGR object). If @flags contains
 * QMAN_CGR_FLAG_USE_INIT, then an init hw command (which will reset any
 * unspecified parameters) will be used rather than a modify hw hardware (which
 * only modifies the specified parameters).
 */
int qman_modify_cgr(struct qman_cgr *cgr, u32 flags,
			struct qm_mcc_initcgr *opts);

/**
* qman_query_cgr - Queries CGR fields
* @cgr: the 'cgr' object to query
* @result: storage for the queried congestion group record
*/
int qman_query_cgr(struct qman_cgr *cgr, struct qm_mcr_querycgr *result);

/**
 * qman_query_congestion - Queries the state of all congestion groups
 * @congestion: storage for the queried state of all congestion groups
 */
int qman_query_congestion(struct qm_mcr_querycongestion *congestion);

/**
 * qman_alloc_cgrid_range - Allocate a contiguous range of CGR IDs
 * @result: is set by the API to the base CGR ID of the allocated range
 * @count: the number of CGR IDs required
 * @align: required alignment of the allocated range
 * @partial: non-zero if the API can return fewer than @count
 *
 * Returns the number of CGR IDs allocated, or a negative error code.
 * If @partial is non zero, the allocation request may return a smaller range of
 * than requested (though alignment will be as requested). If @partial is zero,
 * the return value will either be 'count' or negative.
 */
int qman_alloc_cgrid_range(u32 *result, u32 count, u32 align, int partial);
static inline int qman_alloc_cgrid(u32 *result)
{
	int ret = qman_alloc_cgrid_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}

/**
 * qman_release_cgrid_range - Release the specified range of CGR IDs
 * @id: the base CGR ID of the range to deallocate
 * @count: the number of CGR IDs in the range
 */
void qman_release_cgrid_range(u32 id, unsigned int count);
static inline void qman_release_cgrid(u32 id)
{
	qman_release_cgrid_range(id, 1);
}

/**
 * qman_reserve_cgrid_range - Reserve the specified range of CGR ID
 * @id: the base CGR ID of the range to reserve
 * @count: the number of CGR IDs in the range
 */
int qman_reserve_cgrid_range(u32 id, unsigned int count);
static inline int qman_reserve_cgrid(u32 id)
{
	return qman_reserve_cgrid_range(id, 1);
}

void qman_seed_cgrid_range(u32 id, unsigned int count);


	/* Helpers */
	/* ------- */
/**
 * qman_poll_fq_for_init - Check if an FQ has been initialised from OOS
 * @fqid: the FQID that will be initialised by other s/w
 *
 * In many situations, a FQID is provided for communication between s/w
 * entities, and whilst the consumer is responsible for initialising and
 * scheduling the FQ, the producer(s) generally create a wrapper FQ object using
 * and only call qman_enqueue() (no FQ initialisation, scheduling, etc). Ie;
 *     qman_create_fq(..., QMAN_FQ_FLAG_NO_MODIFY, ...);
 * However, data can not be enqueued to the FQ until it is initialised out of
 * the OOS state - this function polls for that condition. It is particularly
 * useful for users of IPC functions - each endpoint's Rx FQ is the other
 * endpoint's Tx FQ, so each side can initialise and schedule their Rx FQ object
 * and then use this API on the (NO_MODIFY) Tx FQ object in order to
 * synchronise. The function returns zero for success, +1 if the FQ is still in
 * the OOS state, or negative if there was an error.
 */
static inline int qman_poll_fq_for_init(struct qman_fq *fq)
{
	struct qm_mcr_queryfq_np np;
	int err;
	err = qman_query_fq_np(fq, &np);
	if (err)
		return err;
	if ((np.state & QM_MCR_NP_STATE_MASK) == QM_MCR_NP_STATE_OOS)
		return 1;
	return 0;
}

	/* -------------- */
	/* CEETM :: types */
	/* -------------- */
/**
 * Token Rate Structure
 * Shaping rates are based on a "credit" system and a pre-configured h/w
 * internal timer. The following type represents a shaper "rate" parameter as a
 * fractional number of "tokens". Here's how it works. This (fractional) number
 * of tokens is added to the shaper's "credit" every time the h/w timer elapses
 * (up to a limit which is set by another shaper parameter). Every time a frame
 * is enqueued through a shaper, the shaper deducts as many tokens as there are
 * bytes of data in the enqueued frame. A shaper will not allow itself to
 * enqueue any frames if its token count is negative. As such;
 *
 *         The rate at which data is enqueued is limited by the
 *         rate at which tokens are added.
 *
 * Therefore if the user knows the period between these h/w timer updates in
 * seconds, they can calculate the maximum traffic rate of the shaper (in
 * bytes-per-second) from the token rate. And vice versa, they can calculate
 * the token rate to use in order to achieve a given traffic rate.
 */
struct qm_ceetm_rate {
	/* The token rate is; whole + (fraction/8192) */
	u32 whole:11; /* 0..2047 */
	u32 fraction:13; /* 0..8191 */
};

struct qm_ceetm_weight_code {
	/* The weight code is; 5 msbits + 3 lsbits */
	u8 y:5;
	u8 x:3;
};

struct qm_ceetm {
	unsigned int idx;
	struct list_head sub_portals;
	struct list_head lnis;
	unsigned int sp_range[2];
	unsigned int lni_range[2];
};

struct qm_ceetm_sp {
	struct list_head node;
	unsigned int idx;
	unsigned int dcp_idx;
	int is_claimed;
	struct qm_ceetm_lni *lni;
};

/* Logical Network Interface */
struct qm_ceetm_lni {
	struct list_head node;
	unsigned int idx;
	unsigned int dcp_idx;
	int is_claimed;
	struct qm_ceetm_sp *sp;
	struct list_head channels;
	int shaper_enable;
	int shaper_couple;
	int oal;
	struct qm_ceetm_rate cr_token_rate;
	struct qm_ceetm_rate er_token_rate;
	u16 cr_token_bucket_limit;
	u16 er_token_bucket_limit;
};

/* Class Queue Channel */
struct qm_ceetm_channel {
	struct list_head node;
	unsigned int idx;
	unsigned int lni_idx;
	unsigned int dcp_idx;
	struct list_head class_queues;
	struct list_head ccgs;
	u8 shaper_enable;
	u8 shaper_couple;
	struct qm_ceetm_rate cr_token_rate;
	struct qm_ceetm_rate er_token_rate;
	u16 cr_token_bucket_limit;
	u16 er_token_bucket_limit;
};

struct qm_ceetm_ccg;

/* This callback type is used when handling congestion entry/exit. The
 * 'cb_ctx' value is the opaque value associated with ccg object.
 * 'congested' is non-zero on congestion-entry, and zero on congestion-exit.
 */
typedef void (*qman_cb_ccgr)(struct qm_ceetm_ccg *ccg, void *cb_ctx,
							int congested);

/* Class Congestion Group */
struct qm_ceetm_ccg {
	struct qm_ceetm_channel *parent;
	struct list_head node;
	struct list_head cb_node;
	qman_cb_ccgr cb;
	void *cb_ctx;
	unsigned int idx;
};

/* Class Queue */
struct qm_ceetm_cq {
	struct qm_ceetm_channel *parent;
	struct qm_ceetm_ccg *ccg;
	struct list_head node;
	unsigned int idx;
	int is_claimed;
	struct list_head bound_lfqids;
	struct list_head binding_node;
};

/* Logical Frame Queue */
struct qm_ceetm_lfq {
	struct qm_ceetm_channel *parent;
	struct list_head node;
	unsigned int idx;
	unsigned int dctidx;
	u64 context_a;
	u32 context_b;
	qman_cb_mr ern;
};

/**
 * qman_ceetm_bps2tokenrate - Given a desired rate 'bps' measured in bps
 * (ie. bits-per-second), compute the 'token_rate' fraction that best
 * approximates that rate.
 * @bps: the desired shaper rate in bps.
 * @token_rate: the output token rate computed with the given kbps.
 * @rounding: dictates how to round if an exact conversion is not possible; if
 * it is negative then 'token_rate' will round down to the highest value that
 * does not exceed the desired rate, if it is positive then 'token_rate' will
 * round up to the lowest value that is greater than or equal to the desired
 * rate, and if it is zero then it will round to the nearest approximation,
 * whether that be up or down.
 *
 * Return 0 for success, or -EINVAL if prescaler or qman clock is not available.
  */
int qman_ceetm_bps2tokenrate(u64 bps,
				struct qm_ceetm_rate *token_rate,
				int rounding);

/**
 * qman_ceetm_tokenrate2bps - Given a 'token_rate', compute the
 * corresponding number of 'bps'.
 * @token_rate: the input desired token_rate fraction.
 * @bps: the output shaper rate in bps computed with the give token rate.
 * @rounding: has the same semantics as the previous function.
 *
 * Return 0 for success, or -EINVAL if prescaler or qman clock is not available.
 */
int qman_ceetm_tokenrate2bps(const struct qm_ceetm_rate *token_rate,
			      u64 *bps,
			      int rounding);

int qman_alloc_ceetm0_channel_range(u32 *result, u32 count, u32 align,
								int partial);
static inline int qman_alloc_ceetm0_channel(u32 *result)
{
	int ret = qman_alloc_ceetm0_channel_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}
void qman_release_ceetm0_channel_range(u32 channelid, u32 count);
static inline void qman_release_ceetm0_channelid(u32 channelid)
{
	qman_release_ceetm0_channel_range(channelid, 1);
}

int qman_reserve_ceetm0_channel_range(u32 channelid, u32 count);
static inline int qman_reserve_ceetm0_channelid(u32 channelid)
{
	return qman_reserve_ceetm0_channel_range(channelid, 1);
}

void qman_seed_ceetm0_channel_range(u32 channelid, u32 count);


int qman_alloc_ceetm1_channel_range(u32 *result, u32 count, u32 align,
								int partial);
static inline int qman_alloc_ceetm1_channel(u32 *result)
{
	int ret = qman_alloc_ceetm1_channel_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}
void qman_release_ceetm1_channel_range(u32 channelid, u32 count);
static inline void qman_release_ceetm1_channelid(u32 channelid)
{
	qman_release_ceetm1_channel_range(channelid, 1);
}
int qman_reserve_ceetm1_channel_range(u32 channelid, u32 count);
static inline int qman_reserve_ceetm1_channelid(u32 channelid)
{
	return qman_reserve_ceetm1_channel_range(channelid, 1);
}

void qman_seed_ceetm1_channel_range(u32 channelid, u32 count);


int qman_alloc_ceetm0_lfqid_range(u32 *result, u32 count, u32 align,
								int partial);
static inline int qman_alloc_ceetm0_lfqid(u32 *result)
{
	int ret = qman_alloc_ceetm0_lfqid_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}
void qman_release_ceetm0_lfqid_range(u32 lfqid, u32 count);
static inline void qman_release_ceetm0_lfqid(u32 lfqid)
{
	qman_release_ceetm0_lfqid_range(lfqid, 1);
}
int qman_reserve_ceetm0_lfqid_range(u32 lfqid, u32 count);
static inline int qman_reserve_ceetm0_lfqid(u32 lfqid)
{
	return qman_reserve_ceetm0_lfqid_range(lfqid, 1);
}

void qman_seed_ceetm0_lfqid_range(u32 lfqid, u32 count);


int qman_alloc_ceetm1_lfqid_range(u32 *result, u32 count, u32 align,
								int partial);
static inline int qman_alloc_ceetm1_lfqid(u32 *result)
{
	int ret = qman_alloc_ceetm1_lfqid_range(result, 1, 0, 0);
	return (ret > 0) ? 0 : ret;
}
void qman_release_ceetm1_lfqid_range(u32 lfqid, u32 count);
static inline void qman_release_ceetm1_lfqid(u32 lfqid)
{
	qman_release_ceetm1_lfqid_range(lfqid, 1);
}
int qman_reserve_ceetm1_lfqid_range(u32 lfqid, u32 count);
static inline int qman_reserve_ceetm1_lfqid(u32 lfqid)
{
	return qman_reserve_ceetm1_lfqid_range(lfqid, 1);
}

void qman_seed_ceetm1_lfqid_range(u32 lfqid, u32 count);


	/* ----------------------------- */
	/* CEETM :: sub-portals          */
	/* ----------------------------- */

/**
 * qman_ceetm_sp_claim - Claims the given sub-portal, provided it is available
 * to us and configured for traffic-management.
 * @sp: the returned sub-portal object, if successful.
 * @dcp_id: specifies the desired Fman block (and thus the relevant CEETM
 * instance),
 * @sp_idx" is the desired sub-portal index from 0 to 15.
 *
 * Returns zero for success, or -ENODEV if the sub-portal is in use,  or -EINVAL
 * if the sp_idx is out of range.
 *
 * Note that if there are multiple driver domains (eg. a linux kernel versus
 * user-space drivers in USDPAA, or multiple guests running under a hypervisor)
 * then a sub-portal may be accessible by more than one instance of a qman
 * driver and so it may be claimed multiple times. If this is the case, it is
 * up to the system architect to prevent conflicting configuration actions
 * coming from the different driver domains. The qman drivers do not have any
 * behind-the-scenes coordination to prevent this from happening.
 */
int qman_ceetm_sp_claim(struct qm_ceetm_sp **sp,
			enum qm_dc_portal dcp_idx,
			unsigned int sp_idx);

/**
 * qman_ceetm_sp_release - Releases a previously claimed sub-portal.
 * @sp: the sub-portal to be released.
 *
 * Returns 0 for success, or -EBUSY for failure if the dependencies are not
 * released.
 */
int qman_ceetm_sp_release(struct qm_ceetm_sp *sp);

	/* ----------------------------------- */
	/* CEETM :: logical network interfaces */
	/* ----------------------------------- */

/**
 * qman_ceetm_lni_claim - Claims an unclaimed LNI.
 * @lni: the returned LNI object, if successful.
 * @dcp_id: specifies the desired Fman block (and thus the relevant CEETM
 * instance)
 * @lni_idx: is the desired LNI index.
 *
 * Returns zero for success, or -EINVAL on failure, which will happen if the LNI
 * is not available or has already been claimed (and not yet successfully
 * released), or lni_dix is out of range.
 *
 * Note that there may be multiple driver domains (or instances) that need to
 * transmit out the same LNI, so this claim is only guaranteeing exclusivity
 * within the domain of the driver being called. See qman_ceetm_sp_claim() and
 * qman_ceetm_sp_get_lni() for more information.
 */
int qman_ceetm_lni_claim(struct qm_ceetm_lni **lni,
			 enum qm_dc_portal dcp_id,
			 unsigned int lni_idx);

/**
 * qman_ceetm_lni_releaes - Releases a previously claimed LNI.
 * @lni: the lni needs to be released.
 *
 * This will only succeed if all dependent objects have been released.
 * Returns zero for success, or -EBUSY if the dependencies are not released.
 */
int qman_ceetm_lni_release(struct qm_ceetm_lni *lni);

/**
 * qman_ceetm_sp_set_lni
 * qman_ceetm_sp_get_lni - Set/get the LNI that the sub-portal is currently
 * mapped to.
 * @sp: the given sub-portal.
 * @lni(in "set"function): the LNI object which the sp will be mappaed to.
 * @lni_idx(in "get" function): the LNI index which the sp is mapped to.
 *
 * Returns zero for success, or -EINVAL for the "set" function when this sp-lni
 * mapping has been set, or configure mapping command returns error, and
 * -EINVAL for "get" function when this sp-lni mapping is not set or the query
 * mapping command returns error.
 *
 * This may be useful in situations where multiple driver domains have access
 * to the same sub-portals in order to all be able to transmit out the same
 * physical interface (perhaps they're on different IP addresses or VPNs, so
 * Fman is splitting Rx traffic and here we need to converge Tx traffic). In
 * that case, a control-plane is likely to use qman_ceetm_lni_claim() followed
 * by qman_ceetm_sp_set_lni() to configure the sub-portal, and other domains
 * are likely to use qman_ceetm_sp_get_lni() followed by qman_ceetm_lni_claim()
 * in order to determine the LNI that the control-plane had assigned. This is
 * why the "get" returns an index, whereas the "set" takes an (already claimed)
 * LNI object.
 */
int qman_ceetm_sp_set_lni(struct qm_ceetm_sp *sp,
			  struct qm_ceetm_lni *lni);
int qman_ceetm_sp_get_lni(struct qm_ceetm_sp *sp,
			  unsigned int *lni_idx);

/**
 * qman_ceetm_lni_enable_shaper
 * qman_ceetm_lni_disable_shaper - Enables/disables shaping on the LNI.
 * @lni: the given LNI.
 * @coupled: indicates whether CR and ER are coupled.
 * @oal: the overhead accounting length which is added to the actual length of
 * each frame when performing shaper calculations.
 *
 * When the number of (unused) committed-rate tokens reach the committed-rate
 * token limit, 'coupled' indicates whether surplus tokens should be added to
 * the excess-rate token count (up to the excess-rate token limit).
 * When LNI is claimed, the shaper is disabled by default. The enable function
 * will turn on this shaper for this lni.
 * Whenever a claimed LNI is first enabled for shaping, its committed and
 * excess token rates and limits are zero, so will need to be changed to do
 * anything useful. The shaper can subsequently be enabled/disabled without
 * resetting the shaping parameters, but the shaping parameters will be reset
 * when the LNI is released.
 *
 * Returns zero for success, or  errno for "enable" function in the cases as:
 * a) -EINVAL if the shaper is already enabled,
 * b) -EIO if the configure shaper command returns error.
 * For "disable" function, returns:
 * a) -EINVAL if the shaper is has already disabled.
 * b) -EIO if calling configure shaper command returns error.
 */
int qman_ceetm_lni_enable_shaper(struct qm_ceetm_lni *lni, int coupled,
								int oal);
int qman_ceetm_lni_disable_shaper(struct qm_ceetm_lni *lni);

/**
 * qman_ceetm_lni_is_shaper_enabled - Check LNI shaper status
 * @lni: the give LNI
 */
int qman_ceetm_lni_is_shaper_enabled(struct qm_ceetm_lni *lni);

/**
 * qman_ceetm_lni_set_commit_rate
 * qman_ceetm_lni_get_commit_rate
 * qman_ceetm_lni_set_excess_rate
 * qman_ceetm_lni_get_excess_rate - Set/get the shaper CR/ER token rate and
 * token limit for the given LNI.
 * @lni: the given LNI.
 * @token_rate: the desired token rate for "set" fuction, or the token rate of
 * the LNI queried by "get" function.
 * @token_limit: the desired token bucket limit for "set" function, or the token
 * limit of the given LNI queried by "get" function.
 *
 * Returns zero for success. The "set" function returns -EINVAL if the given
 * LNI is unshapped or -EIO if the configure shaper command returns error.
 * The "get" function returns -EINVAL if the token rate or the token limit is
 * not set or the query command returns error.
 */
int qman_ceetm_lni_set_commit_rate(struct qm_ceetm_lni *lni,
				   const struct qm_ceetm_rate *token_rate,
				   u16 token_limit);
int qman_ceetm_lni_get_commit_rate(struct qm_ceetm_lni *lni,
				   struct qm_ceetm_rate *token_rate,
				   u16 *token_limit);
int qman_ceetm_lni_set_excess_rate(struct qm_ceetm_lni *lni,
				   const struct qm_ceetm_rate *token_rate,
				   u16 token_limit);
int qman_ceetm_lni_get_excess_rate(struct qm_ceetm_lni *lni,
				   struct qm_ceetm_rate *token_rate,
				   u16 *token_limit);
/**
 * qman_ceetm_lni_set_commit_rate_bps
 * qman_ceetm_lni_get_commit_rate_bps
 * qman_ceetm_lni_set_excess_rate_bps
 * qman_ceetm_lni_get_excess_rate_bps - Set/get the shaper CR/ER rate
 * and token limit for the given LNI.
 * @lni: the given LNI.
 * @bps: the desired shaping rate in bps for "set" fuction, or the shaping rate
 * of the LNI queried by "get" function.
 * @token_limit: the desired token bucket limit for "set" function, or the token
 * limit of the given LNI queried by "get" function.
 *
 * Returns zero for success. The "set" function returns -EINVAL if the given
 * LNI is unshapped or -EIO if the configure shaper command returns error.
 * The "get" function returns -EINVAL if the token rate or the token limit is
 * not set or the query command returns error.
 */
int qman_ceetm_lni_set_commit_rate_bps(struct qm_ceetm_lni *lni,
				       u64 bps,
				       u16 token_limit);
int qman_ceetm_lni_get_commit_rate_bps(struct qm_ceetm_lni *lni,
				       u64 *bps, u16 *token_limit);
int qman_ceetm_lni_set_excess_rate_bps(struct qm_ceetm_lni *lni,
				       u64 bps,
				       u16 token_limit);
int qman_ceetm_lni_get_excess_rate_bps(struct qm_ceetm_lni *lni,
				       u64 *bps, u16 *token_limit);

/**
 * qman_ceetm_lni_set_tcfcc
 * qman_ceetm_lni_get_tcfcc - Configure/query "Traffic Class Flow Control".
 * @lni: the given LNI.
 * @cq_level: is between 0 and 15, representing individual class queue levels
 * (CQ0 to CQ7 for every channel) and grouped class queue levels (CQ8 to CQ15
 * for every channel).
 * @traffic_class: is between 0 and 7 when associating a given class queue level
 * to a traffic class, or -1 when disabling traffic class flow control for this
 * class queue level.
 *
 * Return zero for success, or -EINVAL if the cq_level or traffic_class is out
 * of range as indicated above, or -EIO if the configure/query tcfcc command
 * returns error.
 *
 * Refer to the section of QMan CEETM traffic class flow control in the
 * Reference Manual.
 */
int qman_ceetm_lni_set_tcfcc(struct qm_ceetm_lni *lni,
			     unsigned int cq_level,
			     int traffic_class);
int qman_ceetm_lni_get_tcfcc(struct qm_ceetm_lni *lni,
			     unsigned int cq_level,
			     int *traffic_class);

	/* ----------------------------- */
	/* CEETM :: class queue channels */
	/* ----------------------------- */

/**
 * qman_ceetm_channel_claim - Claims an unclaimed CQ channel that is mapped to
 * the given LNI.
 * @channel: the returned class queue channel object, if successful.
 * @lni: the LNI that the channel belongs to.
 *
 * Channels are always initially "unshaped".
 *
 * Return zero for success, or -ENODEV if there is no channel available(all 32
 * channels are claimed) or -EINVAL if the channel mapping command returns
 * error.
 */
int qman_ceetm_channel_claim(struct qm_ceetm_channel **channel,
			     struct qm_ceetm_lni *lni);

/**
 * qman_ceetm_channel_release - Releases a previously claimed CQ channel.
 * @channel: the channel needs to be released.
 *
 * Returns zero for success, or -EBUSY if the dependencies are still in use.
 *
 * Note any shaping of the channel will be cleared to leave it in an unshaped
 * state.
 */
int qman_ceetm_channel_release(struct qm_ceetm_channel *channel);

/**
 * qman_ceetm_channel_enable_shaper
 * qman_ceetm_channel_disable_shaper - Enables/disables shaping on the channel.
 * @channel: the given channel.
 * @coupled: indicates whether surplus CR tokens should be added to the
 * excess-rate token count (up to the excess-rate token limit) when the number
 * of (unused) committed-rate tokens reach the committed_rate token limit.
 *
 * Whenever a claimed channel is first enabled for shaping, its committed and
 * excess token rates and limits are zero, so will need to be changed to do
 * anything useful. The shaper can subsequently be enabled/disabled without
 * resetting the shaping parameters, but the shaping parameters will be reset
 * when the channel is released.
 *
 * Return 0 for success, or -EINVAL for failure, in the case that the channel
 * shaper has been enabled/disabled or the management command returns error.
 */
int qman_ceetm_channel_enable_shaper(struct qm_ceetm_channel *channel,
							 int coupled);
int qman_ceetm_channel_disable_shaper(struct qm_ceetm_channel *channel);

/**
 * qman_ceetm_channel_is_shaper_enabled - Check channel shaper status.
 * @channel: the give channel.
 */
int qman_ceetm_channel_is_shaper_enabled(struct qm_ceetm_channel *channel);

/**
 * qman_ceetm_channel_set_commit_rate
 * qman_ceetm_channel_get_commit_rate
 * qman_ceetm_channel_set_excess_rate
 * qman_ceetm_channel_get_excess_rate - Set/get channel CR/ER shaper parameters.
 * @channel: the given channel.
 * @token_rate: the desired token rate for "set" function, or the queried token
 * rate for "get" function.
 * @token_limit: the desired token limit for "set" function, or the queried
 * token limit for "get" function.
 *
 * Return zero for success. The "set" function returns -EINVAL if the channel
 * is unshaped, or -EIO if the configure shapper command returns error. The
 * "get" function returns -EINVAL if token rate of token limit is not set, or
 * the query shaper command returns error.
 */
int qman_ceetm_channel_set_commit_rate(struct qm_ceetm_channel *channel,
				   const struct qm_ceetm_rate *token_rate,
				   u16 token_limit);
int qman_ceetm_channel_get_commit_rate(struct qm_ceetm_channel *channel,
				   struct qm_ceetm_rate *token_rate,
				   u16 *token_limit);
int qman_ceetm_channel_set_excess_rate(struct qm_ceetm_channel *channel,
				   const struct qm_ceetm_rate *token_rate,
				   u16 token_limit);
int qman_ceetm_channel_get_excess_rate(struct qm_ceetm_channel *channel,
				   struct qm_ceetm_rate *token_rate,
				   u16 *token_limit);
/**
 * qman_ceetm_channel_set_commit_rate_bps
 * qman_ceetm_channel_get_commit_rate_bps
 * qman_ceetm_channel_set_excess_rate_bps
 * qman_ceetm_channel_get_excess_rate_bps - Set/get channel CR/ER shaper
 * parameters.
 * @channel: the given channel.
 * @token_rate: the desired shaper rate in bps for "set" function, or the
 * shaper rate in bps for "get" function.
 * @token_limit: the desired token limit for "set" function, or the queried
 * token limit for "get" function.
 *
 * Return zero for success. The "set" function returns -EINVAL if the channel
 * is unshaped, or -EIO if the configure shapper command returns error. The
 * "get" function returns -EINVAL if token rate of token limit is not set, or
 * the query shaper command returns error.
 */
int qman_ceetm_channel_set_commit_rate_bps(struct qm_ceetm_channel *channel,
					   u64 bps, u16 token_limit);
int qman_ceetm_channel_get_commit_rate_bps(struct qm_ceetm_channel *channel,
					   u64 *bps, u16 *token_limit);
int qman_ceetm_channel_set_excess_rate_bps(struct qm_ceetm_channel *channel,
					   u64 bps, u16 token_limit);
int qman_ceetm_channel_get_excess_rate_bps(struct qm_ceetm_channel *channel,
					   u64 *bps, u16 *token_limit);

/**
 * qman_ceetm_channel_set_weight
 * qman_ceetm_channel_get_weight - Set/get the weight for unshaped channel
 * @channel: the given channel.
 * @token_limit: the desired token limit as the weight of the unshaped channel
 * for "set" function, or the queried token limit for "get" function.
 *
 * The algorithm of unshaped fair queuing (uFQ) is used for unshaped channel.
 * It allows the unshaped channels to be included in the CR time eligible list,
 * and thus use the configured CR token limit value as their fair queuing
 * weight.
 *
 * Return zero for success, or -EINVAL if the channel is a shaped channel or
 * the management command returns error.
 */
int qman_ceetm_channel_set_weight(struct qm_ceetm_channel *channel,
				  u16 token_limit);
int qman_ceetm_channel_get_weight(struct qm_ceetm_channel *channel,
				  u16 *token_limit);

/**
 * qman_ceetm_channel_set_group
 * qman_ceetm_channel_get_group - Set/get the grouping of the class scheduler.
 * @channel: the given channel.
 * @group_b: indicates whether there is group B in this channel.
 * @prio_a: the priority of group A.
 * @prio_b: the priority of group B.
 *
 * There are 8 individual class queues (CQ0-CQ7), and 8 grouped class queues
 * (CQ8-CQ15). If 'group_b' is zero, then all the grouped class queues are in
 * group A, otherwise they are split into group A (CQ8-11) and group B
 * (CQ12-C15). The individual class queues and the group(s) are in strict
 * priority order relative to each other. Within the group(s), the scheduling
 * is not strict priority order, but the result of scheduling within a group
 * is in strict priority order relative to the other class queues in the
 * channel. 'prio_a' and 'prio_b' control the priority order of the groups
 * relative to the individual class queues, and take values from 0-7. Eg. if
 * 'group_b' is non-zero, 'prio_a' is 2 and 'prio_b' is 6, then the strict
 * priority order would be;
 *      CQ0, CQ1, CQ2, GROUPA, CQ3, CQ4, CQ5, CQ6, GROUPB, CQ7
 *
 * Return 0 for success. For "set" function, returns -EINVAL if prio_a or
 * prio_b are out of the range 0 - 7 (priority of group A or group B can not
 * be 0, CQ0 is always the highest class queue in this channel.), or -EIO if
 * the configure scheduler command returns error. For "get" function, return
 * -EINVAL if the query scheduler command returns error.
 */
int qman_ceetm_channel_set_group(struct qm_ceetm_channel *channel,
			     int group_b,
			     unsigned int prio_a,
			     unsigned int prio_b);
int qman_ceetm_channel_get_group(struct qm_ceetm_channel *channel,
			     int *group_b,
			     unsigned int *prio_a,
			     unsigned int *prio_b);

/**
 * qman_ceetm_channel_set_group_cr_eligibility
 * qman_ceetm_channel_set_group_er_eligibility - Set channel group eligibility
 * @channel: the given channel object
 * @group_b: indicates whether there is group B in this channel.
 * @cre: the commit rate eligibility, 1 for enable, 0 for disable.
 *
 * Return zero for success, or -EINVAL if eligibility setting fails.
*/
int qman_ceetm_channel_set_group_cr_eligibility(struct qm_ceetm_channel
				*channel, int group_b, int cre);
int qman_ceetm_channel_set_group_er_eligibility(struct qm_ceetm_channel
				*channel, int group_b, int ere);

/**
 * qman_ceetm_channel_set_cq_cr_eligibility
 * qman_ceetm_channel_set_cq_er_eligibility - Set channel cq eligibility
 * @channel: the given channel object
 * @idx: is from 0 to 7 (representing CQ0 to CQ7).
 * @cre: the commit rate eligibility, 1 for enable, 0 for disable.
 *
 * Return zero for success, or -EINVAL if eligibility setting fails.
*/
int qman_ceetm_channel_set_cq_cr_eligibility(struct qm_ceetm_channel *channel,
					unsigned int idx, int cre);
int qman_ceetm_channel_set_cq_er_eligibility(struct qm_ceetm_channel *channel,
					unsigned int idx, int ere);

	/* --------------------- */
	/* CEETM :: class queues */
	/* --------------------- */

/**
 * qman_ceetm_cq_claim - Claims an individual class queue.
 * @cq: the returned class queue object, if successful.
 * @channel: the class queue channel.
 * @idx: is from 0 to 7 (representing CQ0 to CQ7).
 * @ccg: represents the class congestion group that this class queue should be
 * subscribed to, or NULL if no congestion group membership is desired.
 *
 * Returns zero for success, or -EINVAL if @idx is out of range 0 - 7 or
 * if this class queue has been claimed, or configure class queue command
 * returns error, or returns -ENOMEM if allocating CQ memory fails.
 */
int qman_ceetm_cq_claim(struct qm_ceetm_cq **cq,
			struct qm_ceetm_channel *channel,
			unsigned int idx,
			struct qm_ceetm_ccg *ccg);

/**
 * qman_ceetm_cq_claim_A - Claims a class queue group A.
 * @cq: the returned class queue object, if successful.
 * @channel: the class queue channel.
 * @idx: is from 8 to 15 if only group A exits, otherwise, it is from 8 to 11.
 * @ccg: represents the class congestion group that this class queue should be
 * subscribed to, or NULL if no congestion group membership is desired.
 *
 * Return zero for success, or -EINVAL if @idx is out the range or if
 * this class queue has been claimed or configure class queue command returns
 * error, or returns -ENOMEM if allocating CQ memory fails.
 */
int qman_ceetm_cq_claim_A(struct qm_ceetm_cq **cq,
				struct qm_ceetm_channel *channel,
				unsigned int idx,
				struct qm_ceetm_ccg *ccg);

/**
 * qman_ceetm_cq_claim_B - Claims a class queue group B.
 * @cq: the returned class queue object, if successful.
 * @channel: the class queue channel.
 * @idx: is from 0 to 3 (CQ12 to CQ15).
 * @ccg: represents the class congestion group that this class queue should be
 * subscribed to, or NULL if no congestion group membership is desired.
 *
 * Return zero for success, or -EINVAL if @idx is out the range or if
 * this class queue has been claimed or configure class queue command returns
 * error, or returns -ENOMEM if allocating CQ memory fails.
 */
int qman_ceetm_cq_claim_B(struct qm_ceetm_cq **cq,
				struct qm_ceetm_channel *channel,
				unsigned int idx,
				struct qm_ceetm_ccg *ccg);

/**
 * qman_ceetm_cq_release - Releases a previously claimed class queue.
 * @cq: The class queue to be released.
 *
 * Return zero for success, or -EBUSY if the dependent objects (eg. logical
 * FQIDs) have not been released.
 */
int qman_ceetm_cq_release(struct qm_ceetm_cq *cq);

/**
 * qman_ceetm_set_queue_weight
 * qman_ceetm_get_queue_weight - Configure/query the weight of a grouped class
 * queue.
 * @cq: the given class queue.
 * @weight_code: the desired weight code to set for the given class queue for
 * "set" function or the queired weight code for "get" function.
 *
 * Grouped class queues have a default weight code of zero, which corresponds to
 * a scheduler weighting of 1. This function can be used to modify a grouped
 * class queue to another weight, (Use the helpers qman_ceetm_wbfs2ratio()
 * and qman_ceetm_ratio2wbfs() to convert between these 'weight_code' values
 * and the corresponding sharing weight.)
 *
 * Returns zero for success, or -EIO if the configure weight command returns
 * error for "set" function, or -EINVAL if the query command returns
 * error for "get" function.
 * See section "CEETM Weighted Scheduling among Grouped Classes" in Reference
 * Manual for weight and weight code.
 */
int qman_ceetm_set_queue_weight(struct qm_ceetm_cq *cq,
				struct qm_ceetm_weight_code *weight_code);
int qman_ceetm_get_queue_weight(struct qm_ceetm_cq *cq,
				struct qm_ceetm_weight_code *weight_code);

/**
 * qman_ceetm_set_queue_weight_in_ratio
 * qman_ceetm_get_queue_weight_in_ratio - Configure/query the weight of a
 * grouped class queue.
 * @cq: the given class queue.
 * @ratio: the weight in ratio. It should be the real ratio number multiplied
 * by 100 to get rid of fraction.
 *
 * Returns zero for success, or -EIO if the configure weight command returns
 * error for "set" function, or -EINVAL if the query command returns
 * error for "get" function.
 */
int qman_ceetm_set_queue_weight_in_ratio(struct qm_ceetm_cq *cq, u32 ratio);
int qman_ceetm_get_queue_weight_in_ratio(struct qm_ceetm_cq *cq, u32 *ratio);

/* Weights are encoded using a pseudo-exponential scheme. The weight codes 0,
 * 32, 64, [...] correspond to weights of 1, 2, 4, [...]. The weights
 * corresponding to intermediate weight codes are calculated using linear
 * interpolation on the inverted values. Or put another way, the inverse weights
 * for each 32nd weight code are 1, 1/2, 1/4, [...], and so the intervals
 * between these are divided linearly into 32 intermediate values, the inverses
 * of which form the remaining weight codes.
 *
 * The Weighted Bandwidth Fair Scheduling (WBFS) algorithm provides a form of
 * scheduling within a group of class queues (group A or B). Weights are used to
 * normalise the class queues to an underlying BFS algorithm where all class
 * queues are assumed to require "equal bandwidth". So the weights referred to
 * by the weight codes act as divisors on the size of frames being enqueued. Ie.
 * one class queue in a group is assigned a weight of 2 whilst the other class
 * queues in the group keep the default weight of 1, then the WBFS scheduler
 * will effectively treat all frames enqueued on the weight-2 class queue as
 * having half the number of bytes they really have. Ie. if all other things are
 * equal, that class queue would get twice as much bytes-per-second bandwidth as
 * the others. So weights should be chosen to provide bandwidth ratios between
 * members of the same class queue group. These weights have no bearing on
 * behaviour outside that group's WBFS mechanism though.
 */

/**
 * qman_ceetm_wbfs2ratio - Given a weight code ('wbfs'), an accurate fractional
 * representation of the corresponding weight is given (in order to not lose
 * any precision).
 * @weight_code: The given weight code in WBFS.
 * @numerator: the numerator part of the weight computed by the weight code.
 * @denominator: the denominator part of the weight computed by the weight code
 *
 * Returns zero for success or -EINVAL if the given weight code is illegal.
 */
int qman_ceetm_wbfs2ratio(struct qm_ceetm_weight_code *weight_code,
			   u32 *numerator,
			   u32 *denominator);
/**
 * qman_ceetm_ratio2wbfs - Given a weight, find the nearest possible weight code
 * If the user needs to know how close this is, convert the resulting weight
 * code back to a weight and compare.
 * @numerator: numerator part of the given weight.
 * @denominator: denominator part of the given weight.
 * @weight_code: the weight code computed from the given weight.
 *
 * Returns zero for success, or -ERANGE if "numerator/denominator" is outside
 * the range of weights.
 */
int qman_ceetm_ratio2wbfs(u32 numerator,
			   u32 denominator,
			   struct qm_ceetm_weight_code *weight_code,
			   int rounding);

#define QMAN_CEETM_FLAG_CLEAR_STATISTICS_COUNTER	0x1
/**
 * qman_ceetm_cq_get_dequeue_statistics - Get the statistics provided by CEETM
 * CQ counters.
 * @cq: the given CQ object.
 * @flags: indicates whether the statistics counter will be cleared after query.
 * @frame_count: The number of the frames that have been counted since the
 * counter was cleared last time.
 * @byte_count: the number of bytes in all frames that have been counted.
 *
 * Return zero for success or -EINVAL if query statistics command returns error.
 *
 */
int qman_ceetm_cq_get_dequeue_statistics(struct qm_ceetm_cq *cq, u32 flags,
					u64 *frame_count, u64 *byte_count);

/**
 * qman_ceetm_drain_cq - drain the CQ till it is empty.
 * @cq: the give CQ object.
 * Return 0 for success or -EINVAL for unsuccessful command to empty CQ.
 */
int qman_ceetm_drain_cq(struct qm_ceetm_cq *cq);

	/* ---------------------- */
	/* CEETM :: logical FQIDs */
	/* ---------------------- */
/**
 * qman_ceetm_lfq_claim - Claims an unused logical FQID, associates it with
 * the given class queue.
 * @lfq: the returned lfq object, if successful.
 * @cq: the class queue which needs to claim a LFQID.
 *
 * Return zero for success, or -ENODEV if no LFQID is available or -ENOMEM if
 * allocating memory for lfq fails, or -EINVAL if configuring LFQMT fails.
 */
int qman_ceetm_lfq_claim(struct qm_ceetm_lfq **lfq,
				struct qm_ceetm_cq *cq);

/**
 * qman_ceetm_lfq_release - Releases a previously claimed logical FQID.
 * @lfq: the lfq to be released.
 *
 * Return zero for success.
 */
int qman_ceetm_lfq_release(struct qm_ceetm_lfq *lfq);

/**
 * qman_ceetm_lfq_set_context
 * qman_ceetm_lfq_get_context - Set/get the context_a/context_b pair to the
 * "dequeue context table" associated with the logical FQID.
 * @lfq: the given logical FQ object.
 * @context_a: contextA of the dequeue context.
 * @context_b: contextB of the dequeue context.
 *
 * Returns zero for success, or -EINVAL if there is error to set/get the
 * context pair.
 */
int qman_ceetm_lfq_set_context(struct qm_ceetm_lfq *lfq,
				u64 context_a,
				u32 context_b);
int qman_ceetm_lfq_get_context(struct qm_ceetm_lfq *lfq,
				u64 *context_a,
				u32 *context_b);

/**
 * qman_ceetm_create_fq - Initialise a FQ object for the LFQ.
 * @lfq: the given logic fq.
 * @fq: the fq object created for the given logic fq.
 *
 * The FQ object can be used in qman_enqueue() and qman_enqueue_orp() APIs to
 * target a logical FQID (and the class queue it is associated with).
 * Note that this FQ object can only be used for enqueues, and
 * in the case of qman_enqueue_orp() it can not be used as the 'orp' parameter,
 * only as 'fq'. This FQ object can not (and shouldn't) be destroyed, it is only
 * valid as long as the underlying 'lfq' remains claimed. It is the user's
 * responsibility to ensure that the underlying 'lfq' is not released until any
 * enqueues to this FQ object have completed. The only field the user needs to
 * fill in is fq->cb.ern, as that enqueue rejection handler is the callback that
 * could conceivably be called on this FQ object. This API can be called
 * multiple times to create multiple FQ objects referring to the same logical
 * FQID, and any enqueue rejections will respect the callback of the object that
 * issued the enqueue (and will identify the object via the parameter passed to
 * the callback too). There is no 'flags' parameter to this API as there is for
 * qman_create_fq() - the created FQ object behaves as though qman_create_fq()
 * had been called with the single flag QMAN_FQ_FLAG_NO_MODIFY.
 *
 * Returns 0 for success.
 */
int qman_ceetm_create_fq(struct qm_ceetm_lfq *lfq, struct qman_fq *fq);

	/* -------------------------------- */
	/* CEETM :: class congestion groups */
	/* -------------------------------- */

/**
 * qman_ceetm_ccg_claim - Claims an unused CCG.
 * @ccg: the returned CCG object, if successful.
 * @channel: the given class queue channel
 * @cscn: the callback function of this CCG.
 * @cb_ctx: the corresponding context to be used used if state change
 * notifications are later enabled for this CCG.
 *
 * The congestion group is local to the given class queue channel, so only
 * class queues within the channel can be associated with that congestion group.
 * The association of class queues to congestion groups occurs  when the class
 * queues are claimed, see qman_ceetm_cq_claim() and related functions.
 * Congestion groups are in a "zero" state when initially claimed, and they are
 * returned to that state when released.
 *
 * Return zero for success, or -EINVAL if no CCG in the channel is available.
 */
int qman_ceetm_ccg_claim(struct qm_ceetm_ccg **ccg,
			 struct qm_ceetm_channel *channel,
			 unsigned int idx,
			 void (*cscn)(struct qm_ceetm_ccg *,
				       void *cb_ctx,
				       int congested),
			 void *cb_ctx);

/**
 * qman_ceetm_ccg_release - Releases a previously claimed CCG.
 * @ccg: the given ccg.
 *
 * Returns zero for success, or -EBUSY if the given ccg's dependent objects
 * (class queues that are associated with the CCG) have not been released.
 */
int qman_ceetm_ccg_release(struct qm_ceetm_ccg *ccg);

/* This struct is used to specify attributes for a CCG. The 'we_mask' field
 * controls which CCG attributes are to be updated, and the remainder specify
 * the values for those attributes. A CCG counts either frames or the bytes
 * within those frames, but not both ('mode'). A CCG can optionally cause
 * enqueues to be rejected, due to tail-drop or WRED, or both (they are
 * independent options, 'td_en' and 'wr_en_g,wr_en_y,wr_en_r'). Tail-drop can be
 * level-triggered due to a single threshold ('td_thres') or edge-triggered due
 * to a "congestion state", but not both ('td_mode'). Congestion state has
 * distinct entry and exit thresholds ('cs_thres_in' and 'cs_thres_out'), and
 * notifications can be sent to software the CCG goes in to and out of this
 * congested state ('cscn_en'). */
struct qm_ceetm_ccg_params {
	/* Boolean fields together in a single bitfield struct */
	struct {
		/* Whether to count bytes or frames. 1==frames */
		u8 mode:1;
		/* En/disable tail-drop. 1==enable */
		u8 td_en:1;
		/* Tail-drop on congestion-state or threshold. 1=threshold */
		u8 td_mode:1;
		/* Generate congestion state change notifications. 1==enable */
		u8 cscn_en:1;
		/* Enable WRED rejections (per colour). 1==enable */
		u8 wr_en_g:1;
		u8 wr_en_y:1;
		u8 wr_en_r:1;
	} __packed;
	/* Tail-drop threshold. See qm_cgr_thres_[gs]et64(). */
	struct qm_cgr_cs_thres td_thres;
	/* Congestion state thresholds, for entry and exit. */
	struct qm_cgr_cs_thres cs_thres_in;
	struct qm_cgr_cs_thres cs_thres_out;
	/* Overhead accounting length. Per-packet "tax", from -128 to +127 */
	signed char oal;
	/* Congestion state change notification for DCP portal, virtual CCGID*/
	/* WRED parameters. */
	struct qm_cgr_wr_parm wr_parm_g;
	struct qm_cgr_wr_parm wr_parm_y;
	struct qm_cgr_wr_parm wr_parm_r;
};
/* Bits used in 'we_mask' to qman_ceetm_ccg_set(), controls which attributes of
 * the CCGR are to be updated. */
#define QM_CCGR_WE_MODE         0x0001 /* mode (bytes/frames) */
#define QM_CCGR_WE_CS_THRES_IN  0x0002 /* congestion state entry threshold */
#define QM_CCGR_WE_TD_EN        0x0004 /* congestion state tail-drop enable */
#define QM_CCGR_WE_CSCN_TUPD	0x0008 /* CSCN target update */
#define QM_CCGR_WE_CSCN_EN      0x0010 /* congestion notification enable */
#define QM_CCGR_WE_WR_EN_R      0x0020 /* WRED enable - red */
#define QM_CCGR_WE_WR_EN_Y      0x0040 /* WRED enable - yellow */
#define QM_CCGR_WE_WR_EN_G      0x0080 /* WRED enable - green */
#define QM_CCGR_WE_WR_PARM_R    0x0100 /* WRED parameters - red */
#define QM_CCGR_WE_WR_PARM_Y    0x0200 /* WRED parameters - yellow */
#define QM_CCGR_WE_WR_PARM_G    0x0400 /* WRED parameters - green */
#define QM_CCGR_WE_OAL          0x0800 /* overhead accounting length */
#define QM_CCGR_WE_CS_THRES_OUT 0x1000 /* congestion state exit threshold */
#define QM_CCGR_WE_TD_THRES     0x2000 /* tail-drop threshold */
#define QM_CCGR_WE_TD_MODE      0x4000 /* tail-drop mode (state/threshold) */
#define QM_CCGR_WE_CDV		0x8000 /* cdv */

/**
 * qman_ceetm_ccg_set
 * qman_ceetm_ccg_get - Configure/query a subset of CCG attributes.
 * @ccg: the given CCG object.
 * @we_mask: the write enable mask.
 * @params: the parameters setting for this ccg
 *
 * Return 0 for success, or -EIO if configure ccg command returns error for
 * "set" function, or -EINVAL if query ccg command returns error for "get"
 * function.
 */
int qman_ceetm_ccg_set(struct qm_ceetm_ccg *ccg,
			u16 we_mask,
			const struct qm_ceetm_ccg_params *params);
int qman_ceetm_ccg_get(struct qm_ceetm_ccg *ccg,
			struct qm_ceetm_ccg_params *params);

/** qman_ceetm_cscn_swp_set - Add or remove a software portal from the target
 * mask.
 * qman_ceetm_cscn_swp_get - Query whether a given software portal index is
 * in the cscn target mask.
 * @ccg: the give CCG object.
 * @swp_idx: the index of the software portal.
 * @cscn_enabled: 1: Set the swp to be cscn target. 0: remove the swp from
 * the target mask.
 * @we_mask: the write enable mask.
 * @params: the parameters setting for this ccg
 *
 * Return 0 for success, or -EINVAL if command in set/get function fails.
 */
int qman_ceetm_cscn_swp_set(struct qm_ceetm_ccg *ccg,
				u16 swp_idx,
				unsigned int cscn_enabled,
				u16 we_mask,
				const struct qm_ceetm_ccg_params *params);
int qman_ceetm_cscn_swp_get(struct qm_ceetm_ccg *ccg,
				u16 swp_idx,
				unsigned int *cscn_enabled);

/** qman_ceetm_cscn_dcp_set - Add or remove a direct connect portal from the\
 * target mask.
 * qman_ceetm_cscn_dcp_get - Query whether a given direct connect portal index
 * is in the cscn target mask.
 * @ccg: the give CCG object.
 * @dcp_idx: the index of the direct connect portal.
 * @vcgid: congestion state change notification for dcp portal, virtual CGID.
 * @cscn_enabled: 1: Set the dcp to be cscn target. 0: remove the dcp from
 * the target mask.
 * @we_mask: the write enable mask.
 * @params: the parameters setting for this ccg
 *
 * Return 0 for success, or -EINVAL if command in set/get function fails.
  */
int qman_ceetm_cscn_dcp_set(struct qm_ceetm_ccg *ccg,
				u16 dcp_idx,
				u8 vcgid,
				unsigned int cscn_enabled,
				u16 we_mask,
				const struct qm_ceetm_ccg_params *params);
int qman_ceetm_cscn_dcp_get(struct qm_ceetm_ccg *ccg,
				u16 dcp_idx,
				u8 *vcgid,
				unsigned int *cscn_enabled);

/**
 * qman_ceetm_ccg_get_reject_statistics - Get the statistics provided by
 * CEETM CCG counters.
 * @ccg: the given CCG object.
 * @flags: indicates whether the statistics counter will be cleared after query.
 * @frame_count: The number of the frames that have been counted since the
 * counter was cleared last time.
 * @byte_count: the number of bytes in all frames that have been counted.
 *
 * Return zero for success or -EINVAL if query statistics command returns error.
 *
 */
int qman_ceetm_ccg_get_reject_statistics(struct qm_ceetm_ccg *ccg, u32 flags,
					u64 *frame_count, u64 *byte_count);

/**
 * qman_ceetm_query_lfqmt - Query the logical frame queue mapping table
 * @lfqid: Logical Frame Queue ID
 * @lfqmt_query: Results of the query command
 *
 * Returns zero for success or -EIO if the query command returns error.
 *
 */
int qman_ceetm_query_lfqmt(int lfqid,
			   struct qm_mcr_ceetm_lfqmt_query *lfqmt_query);

/**
 * qman_ceetm_query_cq - Queries a CEETM CQ
 * @cqid: the channel ID (first byte) followed by the CQ idx
 * @dcpid: CEETM portal ID
 * @cq_query: storage for the queried CQ fields
 *
 * Returns zero for success or -EIO if the query command returns error.
 *
*/
int qman_ceetm_query_cq(unsigned int cqid, unsigned int dcpid,
			struct qm_mcr_ceetm_cq_query *cq_query);

/**
 * qman_ceetm_query_write_statistics - Query (and optionally write) statistics
 * @cid: Target ID (CQID or CCGRID)
 * @dcp_idx: CEETM portal ID
 * @command_type: One of the following:
 *   0 = Query dequeue statistics. CID carries the CQID to be queried.
 *   1 = Query and clear dequeue statistics. CID carries the CQID to be queried
 *   2 = Write dequeue statistics. CID carries the CQID to be written.
 *   3 = Query reject statistics. CID carries the CCGRID to be queried.
 *   4 = Query and clear reject statistics. CID carries the CCGRID to be queried
 *   5 = Write reject statistics. CID carries the CCGRID to be written
 * @frame_count: Frame count value to be written if this is a write command
 * @byte_count: Bytes count value to be written if this is a write command
 *
 * Returns zero for success or -EIO if the query command returns error.
 */
int qman_ceetm_query_write_statistics(u16 cid, enum qm_dc_portal dcp_idx,
				      u16 command_type, u64 frame_count,
				      u64 byte_count);

/**
 * qman_set_wpm - Set waterfall power management
 *
 * @wpm_enable: boolean, 1 = enable wpm, 0 = disable wpm.
 *
 * Return 0 for success, return -ENODEV if QMan misc_cfg register is not
 * accessible.
 */
int qman_set_wpm(int wpm_enable);

/**
 * qman_get_wpm - Query the waterfall power management setting
 *
 * @wpm_enable: boolean, 1 = enable wpm, 0 = disable wpm.
 *
 * Return 0 for success, return -ENODEV if QMan misc_cfg register is not
 * accessible.
 */
int qman_get_wpm(int *wpm_enable);

/* The below qman_p_***() variants might be called in a migration situation
 * (e.g. cpu hotplug). They are used to continue accessing the portal that
 * execution was affine to prior to migration.
 * @qman_portal specifies which portal the APIs will use.
*/
const struct qman_portal_config *qman_p_get_portal_config(struct qman_portal
									 *p);
int qman_p_irqsource_add(struct qman_portal *p, u32 bits);
int qman_p_irqsource_remove(struct qman_portal *p, u32 bits);
int qman_p_poll_dqrr(struct qman_portal *p, unsigned int limit);
u32 qman_p_poll_slow(struct qman_portal *p);
void qman_p_poll(struct qman_portal *p);
void qman_p_stop_dequeues(struct qman_portal *p);
void qman_p_start_dequeues(struct qman_portal *p);
void qman_p_static_dequeue_add(struct qman_portal *p, u32 pools);
void qman_p_static_dequeue_del(struct qman_portal *p, u32 pools);
u32 qman_p_static_dequeue_get(struct qman_portal *p);
void qman_p_dca(struct qman_portal *p, struct qm_dqrr_entry *dq,
						int park_request);
int qman_p_volatile_dequeue(struct qman_portal *p, struct qman_fq *fq,
				u32 flags __maybe_unused, u32 vdqcr);
int qman_p_enqueue(struct qman_portal *p, struct qman_fq *fq,
					const struct qm_fd *fd, u32 flags);
int qman_p_enqueue_orp(struct qman_portal *p, struct qman_fq *fq,
				const struct qm_fd *fd, u32 flags,
				struct qman_fq *orp, u16 orp_seqnum);
int qman_p_enqueue_precommit(struct qman_portal *p, struct qman_fq *fq,
				const struct qm_fd *fd, u32 flags,
				qman_cb_precommit cb, void *cb_arg);

static inline int qman_is_probed(void) {
	return 1;
}


static inline int qman_portals_probed(void) {
	return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* FSL_QMAN_H */
