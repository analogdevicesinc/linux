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
#include "qman_private.h"

#define MAX_FQID (0x00ffffff)
#define QM_FQD_BLOCK_SIZE     64
#define QM_FQD_AR             (0xC10)

static u32 fqid_max;
static u64 qman_ccsr_start;
static u64 qman_ccsr_size;

static const char * const state_txt[] = {
	"Out of Service",
	"Retired",
	"Tentatively Scheduled",
	"Truly Scheduled",
	"Parked",
	"Active, Active Held or Held Suspended",
	"Unknown State 6",
	"Unknown State 7",
	NULL,
};

static const u8 fqd_states[] = {
	QM_MCR_NP_STATE_OOS, QM_MCR_NP_STATE_RETIRED, QM_MCR_NP_STATE_TEN_SCHED,
	QM_MCR_NP_STATE_TRU_SCHED, QM_MCR_NP_STATE_PARKED,
	QM_MCR_NP_STATE_ACTIVE};

struct mask_to_text {
	u16 mask;
	const char *txt;
};

struct mask_filter_s {
	u16 mask;
	u8 filter;
};

static const struct mask_filter_s mask_filter[] = {
	{QM_FQCTRL_PREFERINCACHE, 0},
	{QM_FQCTRL_PREFERINCACHE, 1},
	{QM_FQCTRL_HOLDACTIVE, 0},
	{QM_FQCTRL_HOLDACTIVE, 1},
	{QM_FQCTRL_AVOIDBLOCK, 0},
	{QM_FQCTRL_AVOIDBLOCK, 1},
	{QM_FQCTRL_FORCESFDR, 0},
	{QM_FQCTRL_FORCESFDR, 1},
	{QM_FQCTRL_CPCSTASH, 0},
	{QM_FQCTRL_CPCSTASH, 1},
	{QM_FQCTRL_CTXASTASHING, 0},
	{QM_FQCTRL_CTXASTASHING, 1},
	{QM_FQCTRL_ORP, 0},
	{QM_FQCTRL_ORP, 1},
	{QM_FQCTRL_TDE, 0},
	{QM_FQCTRL_TDE, 1},
	{QM_FQCTRL_CGE, 0},
	{QM_FQCTRL_CGE, 1}
};

static const struct mask_to_text fq_ctrl_text_list[] = {
	{
		.mask = QM_FQCTRL_PREFERINCACHE,
		.txt = "Prefer in cache",
	},
	{
		.mask = QM_FQCTRL_HOLDACTIVE,
		.txt =  "Hold active in portal",
	},
	{
		.mask = QM_FQCTRL_AVOIDBLOCK,
		.txt = "Avoid Blocking",
	},
	{
		.mask = QM_FQCTRL_FORCESFDR,
		.txt = "High-priority SFDRs",
	},
	{
		.mask = QM_FQCTRL_CPCSTASH,
		.txt = "CPC Stash Enable",
	},
	{
		.mask = QM_FQCTRL_CTXASTASHING,
		.txt =  "Context-A stashing",
	},
	{
		.mask = QM_FQCTRL_ORP,
		.txt =  "ORP Enable",
	},
	{
		.mask = QM_FQCTRL_TDE,
		.txt = "Tail-Drop Enable",
	},
	{
		.mask = QM_FQCTRL_CGE,
		.txt = "Congestion Group Enable",
	},
	{
		.mask = 0,
		.txt = NULL,
	}
};

static const char *get_fqd_ctrl_text(u16 mask)
{
	int i = 0;

	while (fq_ctrl_text_list[i].txt != NULL) {
		if (fq_ctrl_text_list[i].mask == mask)
			return fq_ctrl_text_list[i].txt;
		i++;
	}
	return NULL;
}

static const struct mask_to_text stashing_text_list[] = {
	{
		.mask = QM_STASHING_EXCL_CTX,
		.txt = "FQ Ctx Stash"
	},
	{
		.mask = QM_STASHING_EXCL_DATA,
		.txt =  "Frame Data Stash",
	},
	{
		.mask = QM_STASHING_EXCL_ANNOTATION,
		.txt = "Frame Annotation Stash",
	},
	{
		.mask = 0,
		.txt = NULL,
	},
};

static int user_input_convert(const char __user *user_buf, size_t count,
				unsigned long *val)
{
	char buf[12];

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtoul(buf, 0, val))
		return -EINVAL;
	return 0;
}

struct line_buffer_fq {
	u32 buf[8];
	u32 buf_cnt;
	int line_cnt;
};

static void add_to_line_buffer(struct line_buffer_fq *line_buf, u32 fqid,
			struct seq_file *file)
{
	line_buf->buf[line_buf->buf_cnt] = fqid;
	line_buf->buf_cnt++;
	if (line_buf->buf_cnt == 8) {
		/* Buffer is full, flush it */
		if (line_buf->line_cnt != 0)
			seq_puts(file, ",\n");
		seq_printf(file, "0x%06x,0x%06x,0x%06x,0x%06x,0x%06x,"
			"0x%06x,0x%06x,0x%06x",
			line_buf->buf[0], line_buf->buf[1], line_buf->buf[2],
			line_buf->buf[3], line_buf->buf[4], line_buf->buf[5],
			line_buf->buf[6], line_buf->buf[7]);
		line_buf->buf_cnt = 0;
		line_buf->line_cnt++;
	}
}

static void flush_line_buffer(struct line_buffer_fq *line_buf,
				struct seq_file *file)
{
	if (line_buf->buf_cnt) {
		int y = 0;
		if (line_buf->line_cnt != 0)
			seq_puts(file, ",\n");
		while (y != line_buf->buf_cnt) {
			if (y+1 == line_buf->buf_cnt)
				seq_printf(file, "0x%06x", line_buf->buf[y]);
			else
				seq_printf(file, "0x%06x,", line_buf->buf[y]);
			y++;
		}
		line_buf->line_cnt++;
	}
	if (line_buf->line_cnt)
		seq_putc(file, '\n');
}

static struct dentry *dfs_root; /* debugfs root directory */

/*******************************************************************************
 *  Query Frame Queue Non Programmable Fields
 ******************************************************************************/
struct query_fq_np_fields_data_s {
	u32 fqid;
};
static struct query_fq_np_fields_data_s query_fq_np_fields_data = {
	.fqid = 1,
};

static int query_fq_np_fields_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_queryfq_np np;
	struct qman_fq fq;

	fq.fqid = query_fq_np_fields_data.fqid;
	ret = qman_query_fq_np(&fq, &np);
	if (ret)
		return ret;
	/* Print state */
	seq_printf(file, "Query FQ Non Programmable Fields Result fqid 0x%x\n",
			fq.fqid);
	seq_printf(file, " force eligible pending: %s\n",
		(np.state & QM_MCR_NP_STATE_FE) ? "yes" : "no");
	seq_printf(file, " retirement pending: %s\n",
		(np.state & QM_MCR_NP_STATE_R) ? "yes" : "no");
	seq_printf(file, " state: %s\n",
		state_txt[np.state & QM_MCR_NP_STATE_MASK]);
	seq_printf(file, " fq_link: 0x%x\n", np.fqd_link);
	seq_printf(file, " odp_seq: %u\n", np.odp_seq);
	seq_printf(file, " orp_nesn: %u\n", np.orp_nesn);
	seq_printf(file, " orp_ea_hseq: %u\n", np.orp_ea_hseq);
	seq_printf(file, " orp_ea_tseq: %u\n", np.orp_ea_tseq);
	seq_printf(file, " orp_ea_hptr: 0x%x\n", np.orp_ea_hptr);
	seq_printf(file, " orp_ea_tptr: 0x%x\n", np.orp_ea_tptr);
	seq_printf(file, " pfdr_hptr: 0x%x\n", np.pfdr_hptr);
	seq_printf(file, " pfdr_tptr: 0x%x\n", np.pfdr_tptr);
	seq_printf(file, " is: ics_surp contains a %s\n",
		(np.is) ? "deficit" : "surplus");
	seq_printf(file, " ics_surp: %u\n", np.ics_surp);
	seq_printf(file, " byte_cnt: %u\n", np.byte_cnt);
	seq_printf(file, " frm_cnt: %u\n", np.frm_cnt);
	seq_printf(file, " ra1_sfdr: 0x%x\n", np.ra1_sfdr);
	seq_printf(file, " ra2_sfdr: 0x%x\n", np.ra2_sfdr);
	seq_printf(file, " od1_sfdr: 0x%x\n", np.od1_sfdr);
	seq_printf(file, " od2_sfdr: 0x%x\n", np.od2_sfdr);
	seq_printf(file, " od3_sfdr: 0x%x\n", np.od3_sfdr);
	return 0;
}

static int query_fq_np_fields_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, query_fq_np_fields_show, NULL);
}

static ssize_t query_fq_np_fields_write(struct file *f,
			const char __user *buf, size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > MAX_FQID)
		return -EINVAL;
	query_fq_np_fields_data.fqid = (u32)val;
	return count;
}

static const struct file_operations query_fq_np_fields_fops = {
	.owner          = THIS_MODULE,
	.open		= query_fq_np_fields_open,
	.read           = seq_read,
	.write		= query_fq_np_fields_write,
	.release	= single_release,
};

/*******************************************************************************
 *  Frame Queue Programmable Fields
 ******************************************************************************/
struct query_fq_fields_data_s {
	u32 fqid;
};

static struct query_fq_fields_data_s query_fq_fields_data = {
	.fqid = 1,
};

static int query_fq_fields_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_fqd fqd;
	struct qman_fq fq;
	int i = 0;

	memset(&fqd, 0, sizeof(struct qm_fqd));
	fq.fqid = query_fq_fields_data.fqid;
	ret = qman_query_fq(&fq, &fqd);
	if (ret)
		return ret;
	seq_printf(file, "Query FQ Programmable Fields Result fqid 0x%x\n",
			fq.fqid);
	seq_printf(file, " orprws: %u\n", fqd.orprws);
	seq_printf(file, " oa: %u\n", fqd.oa);
	seq_printf(file, " olws: %u\n", fqd.olws);

	seq_printf(file, " cgid: %u\n", fqd.cgid);

	if ((fqd.fq_ctrl & QM_FQCTRL_MASK) == 0)
		seq_puts(file, " fq_ctrl: None\n");
	else {
		i = 0;
		seq_puts(file, " fq_ctrl:\n");
		while (fq_ctrl_text_list[i].txt != NULL) {
			if ((fqd.fq_ctrl & QM_FQCTRL_MASK) &
					fq_ctrl_text_list[i].mask)
				seq_printf(file, "  %s\n",
					fq_ctrl_text_list[i].txt);
			i++;
		}
	}
	seq_printf(file, " dest_channel: %u\n", fqd.dest.channel);
	seq_printf(file, " dest_wq: %u\n", fqd.dest.wq);
	seq_printf(file, " ics_cred: %u\n", fqd.ics_cred);
	seq_printf(file, " td_mant: %u\n", fqd.td.mant);
	seq_printf(file, " td_exp: %u\n", fqd.td.exp);

	seq_printf(file, " ctx_b: 0x%x\n", fqd.context_b);

	seq_printf(file, " ctx_a: 0x%llx\n", qm_fqd_stashing_get64(&fqd));
	/* Any stashing configured */
	if ((fqd.context_a.stashing.exclusive & 0x7) == 0)
		seq_puts(file, " ctx_a_stash_exclusive: None\n");
	else {
		seq_puts(file, " ctx_a_stash_exclusive:\n");
		i = 0;
		while (stashing_text_list[i].txt != NULL) {
			if ((fqd.fq_ctrl & 0x7) & stashing_text_list[i].mask)
				seq_printf(file, "  %s\n",
					stashing_text_list[i].txt);
			i++;
		}
	}
	seq_printf(file, " ctx_a_stash_annotation_cl: %u\n",
			fqd.context_a.stashing.annotation_cl);
	seq_printf(file, " ctx_a_stash_data_cl: %u\n",
			fqd.context_a.stashing.data_cl);
	seq_printf(file, " ctx_a_stash_context_cl: %u\n",
			fqd.context_a.stashing.context_cl);
	return 0;
}

static int query_fq_fields_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, query_fq_fields_show, NULL);
}

static ssize_t query_fq_fields_write(struct file *f,
			const char __user *buf, size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > MAX_FQID)
		return -EINVAL;
	query_fq_fields_data.fqid = (u32)val;
	return count;
}

static const struct file_operations query_fq_fields_fops = {
	.owner          = THIS_MODULE,
	.open		= query_fq_fields_open,
	.read           = seq_read,
	.write		= query_fq_fields_write,
	.release	= single_release,
};

/*******************************************************************************
 * Query WQ lengths
 ******************************************************************************/
struct query_wq_lengths_data_s {
	union {
		u16 channel_wq; /* ignores wq (3 lsbits) */
		struct {
			u16 id:13; /* qm_channel */
			u16 __reserved:3;
		} __packed channel;
	};
};
static struct query_wq_lengths_data_s query_wq_lengths_data;
static int query_wq_lengths_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_querywq wq;
	int i;

	memset(&wq, 0, sizeof(struct qm_mcr_querywq));
	wq.channel.id = query_wq_lengths_data.channel.id;
	ret = qman_query_wq(0, &wq);
	if (ret)
		return ret;
	seq_printf(file, "Query Result For Channel: 0x%x\n", wq.channel.id);
	for (i = 0; i < 8; i++)
		/* mask out upper 4 bits since they are not part of length */
		seq_printf(file, " wq%d_len : %u\n", i, wq.wq_len[i] & 0x0fff);
	return 0;
}

static int query_wq_lengths_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, query_wq_lengths_show, NULL);
}

static ssize_t query_wq_lengths_write(struct file *f,
			const char __user *buf, size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > 0xfff8)
		return -EINVAL;
	query_wq_lengths_data.channel.id = (u16)val;
	return count;
}

static const struct file_operations query_wq_lengths_fops = {
	.owner          = THIS_MODULE,
	.open		= query_wq_lengths_open,
	.read           = seq_read,
	.write		= query_wq_lengths_write,
	.release	= single_release,
};

/*******************************************************************************
 *  Query CGR
 ******************************************************************************/
struct query_cgr_s {
	u8 cgid;
};
static struct query_cgr_s query_cgr_data;

static int query_cgr_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_querycgr cgrd;
	struct qman_cgr cgr;
	int i, j;
	u32 mask;

	memset(&cgr, 0, sizeof(cgr));
	memset(&cgrd, 0, sizeof(cgrd));
	cgr.cgrid = query_cgr_data.cgid;
	ret = qman_query_cgr(&cgr, &cgrd);
	if (ret)
		return ret;
	seq_printf(file, "Query CGR id 0x%x\n", cgr.cgrid);
	seq_printf(file, " wr_parm_g MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		cgrd.cgr.wr_parm_g.MA, cgrd.cgr.wr_parm_g.Mn,
		cgrd.cgr.wr_parm_g.SA, cgrd.cgr.wr_parm_g.Sn,
		cgrd.cgr.wr_parm_g.Pn);

	seq_printf(file, " wr_parm_y MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		cgrd.cgr.wr_parm_y.MA, cgrd.cgr.wr_parm_y.Mn,
		cgrd.cgr.wr_parm_y.SA, cgrd.cgr.wr_parm_y.Sn,
		cgrd.cgr.wr_parm_y.Pn);

	seq_printf(file, " wr_parm_r MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		cgrd.cgr.wr_parm_r.MA, cgrd.cgr.wr_parm_r.Mn,
		cgrd.cgr.wr_parm_r.SA, cgrd.cgr.wr_parm_r.Sn,
		cgrd.cgr.wr_parm_r.Pn);

	seq_printf(file, " wr_en_g: %u, wr_en_y: %u, we_en_r: %u\n",
		cgrd.cgr.wr_en_g, cgrd.cgr.wr_en_y, cgrd.cgr.wr_en_r);

	seq_printf(file, " cscn_en: %u\n", cgrd.cgr.cscn_en);
	if ((qman_ip_rev & 0xFF00) >= QMAN_REV30) {
		seq_puts(file, " cscn_targ_dcp:\n");
		mask = 0x80000000;
		for (i = 0; i < 32; i++) {
			if (cgrd.cgr.cscn_targ & mask)
				seq_printf(file, "  send CSCN to dcp %u\n",
								(31 - i));
			mask >>= 1;
		}

		seq_puts(file, " cscn_targ_swp:\n");
		for (i = 0; i < 4; i++) {
			mask = 0x80000000;
			for (j = 0; j < 32; j++) {
				if (cgrd.cscn_targ_swp[i] & mask)
					seq_printf(file, "  send CSCN to swp"
						" %u\n", (127 - (i * 32) - j));
				mask >>= 1;
			}
		}
	} else {
		seq_printf(file, " cscn_targ: %u\n", cgrd.cgr.cscn_targ);
	}
	seq_printf(file, " cstd_en: %u\n", cgrd.cgr.cstd_en);
	seq_printf(file, " cs: %u\n", cgrd.cgr.cs);

	seq_printf(file, " cs_thresh_TA: %u, cs_thresh_Tn: %u\n",
		cgrd.cgr.cs_thres.TA, cgrd.cgr.cs_thres.Tn);

	seq_printf(file, " mode: %s\n",
		(cgrd.cgr.mode & QMAN_CGR_MODE_FRAME) ?
		"frame count" : "byte count");
	seq_printf(file, " i_bcnt: %llu\n", qm_mcr_querycgr_i_get64(&cgrd));
	seq_printf(file, " a_bcnt: %llu\n", qm_mcr_querycgr_a_get64(&cgrd));

	return 0;
}

static int query_cgr_open(struct inode *inode, struct file *file)
{
	return single_open(file, query_cgr_show, NULL);
}

static ssize_t query_cgr_write(struct file *f, const char __user *buf,
				size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > 0xff)
		return -EINVAL;
	query_cgr_data.cgid = (u8)val;
	return count;
}

static const struct file_operations query_cgr_fops = {
	.owner          = THIS_MODULE,
	.open		= query_cgr_open,
	.read           = seq_read,
	.write		= query_cgr_write,
	.release	= single_release,
};

/*******************************************************************************
 *  Test Write CGR
 ******************************************************************************/
struct test_write_cgr_s {
	u64 i_bcnt;
	u8 cgid;
};
static struct test_write_cgr_s test_write_cgr_data;

static int testwrite_cgr_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_cgrtestwrite result;
	struct qman_cgr cgr;
	u64 i_bcnt;

	memset(&cgr, 0, sizeof(struct qman_cgr));
	memset(&result, 0, sizeof(struct qm_mcr_cgrtestwrite));
	cgr.cgrid = test_write_cgr_data.cgid;
	i_bcnt = test_write_cgr_data.i_bcnt;
	ret = qman_testwrite_cgr(&cgr, i_bcnt, &result);
	if (ret)
		return ret;
	seq_printf(file, "CGR Test Write CGR id 0x%x\n", cgr.cgrid);
	seq_printf(file, " wr_parm_g MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		result.cgr.wr_parm_g.MA, result.cgr.wr_parm_g.Mn,
		result.cgr.wr_parm_g.SA, result.cgr.wr_parm_g.Sn,
		result.cgr.wr_parm_g.Pn);
	seq_printf(file, " wr_parm_y MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		result.cgr.wr_parm_y.MA, result.cgr.wr_parm_y.Mn,
		result.cgr.wr_parm_y.SA, result.cgr.wr_parm_y.Sn,
		result.cgr.wr_parm_y.Pn);
	seq_printf(file, " wr_parm_r MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		result.cgr.wr_parm_r.MA, result.cgr.wr_parm_r.Mn,
		result.cgr.wr_parm_r.SA, result.cgr.wr_parm_r.Sn,
		result.cgr.wr_parm_r.Pn);
	seq_printf(file, " wr_en_g: %u, wr_en_y: %u, we_en_r: %u\n",
		result.cgr.wr_en_g, result.cgr.wr_en_y, result.cgr.wr_en_r);
	seq_printf(file, " cscn_en: %u\n", result.cgr.cscn_en);
	seq_printf(file, " cscn_targ: %u\n", result.cgr.cscn_targ);
	seq_printf(file, " cstd_en: %u\n", result.cgr.cstd_en);
	seq_printf(file, " cs: %u\n", result.cgr.cs);
	seq_printf(file, " cs_thresh_TA: %u, cs_thresh_Tn: %u\n",
		result.cgr.cs_thres.TA, result.cgr.cs_thres.Tn);

	/* Add Mode for Si 2 */
	seq_printf(file, " mode: %s\n",
		(result.cgr.mode & QMAN_CGR_MODE_FRAME) ?
		"frame count" : "byte count");

	seq_printf(file, " i_bcnt: %llu\n",
		qm_mcr_cgrtestwrite_i_get64(&result));
	seq_printf(file, " a_bcnt: %llu\n",
		qm_mcr_cgrtestwrite_a_get64(&result));
	seq_printf(file, " wr_prob_g: %u\n", result.wr_prob_g);
	seq_printf(file, " wr_prob_y: %u\n", result.wr_prob_y);
	seq_printf(file, " wr_prob_r: %u\n", result.wr_prob_r);
	return 0;
}

static int testwrite_cgr_open(struct inode *inode, struct file *file)
{
	return single_open(file, testwrite_cgr_show, NULL);
}

static const struct file_operations testwrite_cgr_fops = {
	.owner          = THIS_MODULE,
	.open		= testwrite_cgr_open,
	.read           = seq_read,
	.release	= single_release,
};


static int testwrite_cgr_ibcnt_show(struct seq_file *file, void *offset)
{
	seq_printf(file, "i_bcnt: %llu\n", test_write_cgr_data.i_bcnt);
	return 0;
}
static int testwrite_cgr_ibcnt_open(struct inode *inode, struct file *file)
{
	return single_open(file, testwrite_cgr_ibcnt_show, NULL);
}

static ssize_t testwrite_cgr_ibcnt_write(struct file *f, const char __user *buf,
				size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	test_write_cgr_data.i_bcnt = val;
	return count;
}

static const struct file_operations teswrite_cgr_ibcnt_fops = {
	.owner          = THIS_MODULE,
	.open		= testwrite_cgr_ibcnt_open,
	.read           = seq_read,
	.write		= testwrite_cgr_ibcnt_write,
	.release	= single_release,
};

static int testwrite_cgr_cgrid_show(struct seq_file *file, void *offset)
{
	seq_printf(file, "cgrid: %u\n", (u32)test_write_cgr_data.cgid);
	return 0;
}
static int testwrite_cgr_cgrid_open(struct inode *inode, struct file *file)
{
	return single_open(file, testwrite_cgr_cgrid_show, NULL);
}

static ssize_t testwrite_cgr_cgrid_write(struct file *f, const char __user *buf,
				size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > 0xff)
		return -EINVAL;
	test_write_cgr_data.cgid = (u8)val;
	return count;
}

static const struct file_operations teswrite_cgr_cgrid_fops = {
	.owner          = THIS_MODULE,
	.open		= testwrite_cgr_cgrid_open,
	.read           = seq_read,
	.write		= testwrite_cgr_cgrid_write,
	.release	= single_release,
};

/*******************************************************************************
 *  Query Congestion State
 ******************************************************************************/
static int query_congestion_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_querycongestion cs;
	int i, j, in_cong = 0;
	u32 mask;

	memset(&cs, 0, sizeof(struct qm_mcr_querycongestion));
	ret = qman_query_congestion(&cs);
	if (ret)
		return ret;
	seq_puts(file, "Query Congestion Result\n");
	for (i = 0; i < 8; i++) {
		mask = 0x80000000;
		for (j = 0; j < 32; j++) {
			if (cs.state.__state[i] & mask) {
				in_cong = 1;
				seq_printf(file, " cg %u: %s\n", (i*32)+j,
					"in congestion");
			}
			mask >>= 1;
		}
	}
	if (!in_cong)
		seq_puts(file, " All congestion groups not congested.\n");
	return 0;
}

static int query_congestion_open(struct inode *inode, struct file *file)
{
	return single_open(file, query_congestion_show, NULL);
}

static const struct file_operations query_congestion_fops = {
	.owner          = THIS_MODULE,
	.open		= query_congestion_open,
	.read           = seq_read,
	.release	= single_release,
};

/*******************************************************************************
 *  Query CCGR
 ******************************************************************************/
struct query_ccgr_s {
	u32 ccgid;
};
static struct query_ccgr_s query_ccgr_data;

static int query_ccgr_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_ceetm_ccgr_query ccgr_query;
	struct qm_mcc_ceetm_ccgr_query query_opts;
	int i, j;
	u32 mask;

	memset(&ccgr_query, 0, sizeof(struct qm_mcr_ceetm_ccgr_query));
	memset(&query_opts, 0, sizeof(struct qm_mcc_ceetm_ccgr_query));

	if ((qman_ip_rev & 0xFF00) < QMAN_REV30)
		return -EINVAL;

	seq_printf(file, "Query CCGID %x\n", query_ccgr_data.ccgid);
	query_opts.dcpid = ((query_ccgr_data.ccgid & 0xFF000000) >> 24);
	query_opts.ccgrid = query_ccgr_data.ccgid & 0x000001FF;
	ret = qman_ceetm_query_ccgr(&query_opts, &ccgr_query);
	if (ret)
		return ret;
	seq_printf(file, "Query CCGR id %x in DCP %d\n", query_opts.ccgrid,
						query_opts.dcpid);
	seq_printf(file, " wr_parm_g MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		ccgr_query.cm_query.wr_parm_g.MA,
		ccgr_query.cm_query.wr_parm_g.Mn,
		ccgr_query.cm_query.wr_parm_g.SA,
		ccgr_query.cm_query.wr_parm_g.Sn,
		ccgr_query.cm_query.wr_parm_g.Pn);

	seq_printf(file, " wr_parm_y MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		ccgr_query.cm_query.wr_parm_y.MA,
		ccgr_query.cm_query.wr_parm_y.Mn,
		ccgr_query.cm_query.wr_parm_y.SA,
		ccgr_query.cm_query.wr_parm_y.Sn,
		ccgr_query.cm_query.wr_parm_y.Pn);

	seq_printf(file, " wr_parm_r MA: %u, Mn: %u, SA: %u, Sn: %u, Pn: %u\n",
		ccgr_query.cm_query.wr_parm_r.MA,
		ccgr_query.cm_query.wr_parm_r.Mn,
		ccgr_query.cm_query.wr_parm_r.SA,
		ccgr_query.cm_query.wr_parm_r.Sn,
		ccgr_query.cm_query.wr_parm_r.Pn);

	seq_printf(file, " wr_en_g: %u, wr_en_y: %u, we_en_r: %u\n",
		ccgr_query.cm_query.ctl_wr_en_g,
		ccgr_query.cm_query.ctl_wr_en_y,
		ccgr_query.cm_query.ctl_wr_en_r);

	seq_printf(file, " cscn_en: %u\n", ccgr_query.cm_query.ctl_cscn_en);
	seq_puts(file, " cscn_targ_dcp:\n");
	mask = 0x80000000;
	for (i = 0; i < 32; i++) {
		if (ccgr_query.cm_query.cscn_targ_dcp & mask)
			seq_printf(file, "  send CSCN to dcp %u\n", (31 - i));
		mask >>= 1;
	}

	seq_puts(file, " cscn_targ_swp:\n");
	for (i = 0; i < 4; i++) {
		mask = 0x80000000;
		for (j = 0; j < 32; j++) {
			if (ccgr_query.cm_query.cscn_targ_swp[i] & mask)
				seq_printf(file, "  send CSCN to swp"
					"%u\n", (127 - (i * 32) - j));
			mask >>= 1;
		}
	}

	seq_printf(file, " td_en: %u\n", ccgr_query.cm_query.ctl_td_en);

	seq_printf(file, " cs_thresh_in_TA: %u, cs_thresh_in_Tn: %u\n",
			ccgr_query.cm_query.cs_thres.TA,
			ccgr_query.cm_query.cs_thres.Tn);

	seq_printf(file, " cs_thresh_out_TA: %u, cs_thresh_out_Tn: %u\n",
			ccgr_query.cm_query.cs_thres_x.TA,
			ccgr_query.cm_query.cs_thres_x.Tn);

	seq_printf(file, " td_thresh_TA: %u, td_thresh_Tn: %u\n",
			ccgr_query.cm_query.td_thres.TA,
			ccgr_query.cm_query.td_thres.Tn);

	seq_printf(file, " mode: %s\n",
			(ccgr_query.cm_query.ctl_mode &
			QMAN_CGR_MODE_FRAME) ?
			"frame count" : "byte count");
	seq_printf(file, " i_cnt: %llu\n", (u64)ccgr_query.cm_query.i_cnt);
	seq_printf(file, " a_cnt: %llu\n", (u64)ccgr_query.cm_query.a_cnt);

	return 0;
}

static int query_ccgr_open(struct inode *inode, struct file *file)
{
	return single_open(file, query_ccgr_show, NULL);
}

static ssize_t query_ccgr_write(struct file *f, const char __user *buf,
				size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	query_ccgr_data.ccgid = val;
	return count;
}

static const struct file_operations query_ccgr_fops = {
	.owner          = THIS_MODULE,
	.open		= query_ccgr_open,
	.read           = seq_read,
	.write		= query_ccgr_write,
	.release	= single_release,
};
/*******************************************************************************
 *  QMan register
 ******************************************************************************/
struct qman_register_s {
	u32 val;
};
static struct qman_register_s qman_register_data;

static void init_ccsrmempeek(void)
{
	struct device_node *dn;
	const u32 *regaddr_p;

	dn = of_find_compatible_node(NULL, NULL, "fsl,qman");
	if (!dn) {
		pr_info("No fsl,qman node\n");
		return;
	}
	regaddr_p = of_get_address(dn, 0, &qman_ccsr_size, NULL);
	if (!regaddr_p) {
		of_node_put(dn);
		return;
	}
	qman_ccsr_start = of_translate_address(dn, regaddr_p);
	of_node_put(dn);
}
/* This function provides access to QMan ccsr memory map */
static int qman_ccsrmempeek(u32 *val, u32 offset)
{
	void __iomem *addr;
	u64 phys_addr;

	if (!qman_ccsr_start)
		return -EINVAL;

	if (offset > (qman_ccsr_size - sizeof(u32)))
		return -EINVAL;

	phys_addr = qman_ccsr_start + offset;
	addr = ioremap(phys_addr, sizeof(u32));
	if (!addr) {
		pr_err("ccsrmempeek, ioremap failed\n");
		return -EINVAL;
	}
	*val = in_be32(addr);
	iounmap(addr);
	return 0;
}

static int qman_ccsrmempeek_show(struct seq_file *file, void *offset)
{
	u32 b;

	qman_ccsrmempeek(&b, qman_register_data.val);
	seq_printf(file, "QMan register offset = 0x%x\n",
		   qman_register_data.val);
	seq_printf(file, "value = 0x%08x\n", b);

	return 0;
}

static int qman_ccsrmempeek_open(struct inode *inode, struct file *file)
{
	return single_open(file, qman_ccsrmempeek_show, NULL);
}

static ssize_t qman_ccsrmempeek_write(struct file *f, const char __user *buf,
				size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	/* multiple of 4 */
	if (val > (qman_ccsr_size - sizeof(u32))) {
		pr_info("Input 0x%lx > 0x%llx\n",
			val, (qman_ccsr_size - sizeof(u32)));
		return -EINVAL;
	}
	if (val & 0x3) {
		pr_info("Input 0x%lx not multiple of 4\n", val);
		return -EINVAL;
	}
	qman_register_data.val = val;
	return count;
}

static const struct file_operations qman_ccsrmempeek_fops = {
	.owner          = THIS_MODULE,
	.open		= qman_ccsrmempeek_open,
	.read           = seq_read,
	.write		= qman_ccsrmempeek_write,
};

/*******************************************************************************
 *  QMan state
 ******************************************************************************/
static int qman_fqd_state_show(struct seq_file *file, void *offset)
{
	struct qm_mcr_queryfq_np np;
	struct qman_fq fq;
	struct line_buffer_fq line_buf;
	int ret, i;
	u8 *state = file->private;
	u32 qm_fq_state_cnt[ARRAY_SIZE(fqd_states)];

	memset(qm_fq_state_cnt, 0, sizeof(qm_fq_state_cnt));
	memset(&line_buf, 0, sizeof(line_buf));

	seq_printf(file, "List of fq ids in state: %s\n", state_txt[*state]);

	for (i = 1; i < fqid_max; i++) {
		fq.fqid = i;
		ret = qman_query_fq_np(&fq, &np);
		if (ret)
			return ret;
		if (*state == (np.state & QM_MCR_NP_STATE_MASK))
			add_to_line_buffer(&line_buf, fq.fqid, file);
		/* Keep a summary count of all states */
		if ((np.state & QM_MCR_NP_STATE_MASK) < ARRAY_SIZE(fqd_states))
			qm_fq_state_cnt[(np.state & QM_MCR_NP_STATE_MASK)]++;
	}
	flush_line_buffer(&line_buf, file);

	for (i = 0; i < ARRAY_SIZE(fqd_states); i++) {
		seq_printf(file, "%s count = %u\n", state_txt[i],
			   qm_fq_state_cnt[i]);
	}
	return 0;
}

static int qman_fqd_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, qman_fqd_state_show, inode->i_private);
}

static const struct file_operations qman_fqd_state_fops =  {
	.owner          = THIS_MODULE,
	.open		= qman_fqd_state_open,
	.read           = seq_read,
};

static int qman_fqd_ctrl_show(struct seq_file *file, void *offset)
{
	struct qm_fqd fqd;
	struct qman_fq fq;
	u32 fq_en_cnt = 0, fq_di_cnt = 0;
	int ret, i;
	struct mask_filter_s *data = file->private;
	const char *ctrl_txt = get_fqd_ctrl_text(data->mask);
	struct line_buffer_fq line_buf;

	memset(&line_buf, 0, sizeof(line_buf));
	seq_printf(file, "List of fq ids with: %s :%s\n",
		ctrl_txt, (data->filter) ? "enabled" : "disabled");
	for (i = 1; i < fqid_max; i++) {
		fq.fqid = i;
		memset(&fqd, 0, sizeof(struct qm_fqd));
		ret = qman_query_fq(&fq, &fqd);
		if (ret)
			return ret;
		if (data->filter) {
			if (fqd.fq_ctrl & data->mask)
				add_to_line_buffer(&line_buf, fq.fqid, file);
		} else {
			if (!(fqd.fq_ctrl & data->mask))
				add_to_line_buffer(&line_buf, fq.fqid, file);
		}
		if (fqd.fq_ctrl & data->mask)
			fq_en_cnt++;
		else
			fq_di_cnt++;
	}
	flush_line_buffer(&line_buf, file);

	seq_printf(file, "Total FQD with: %s :  enabled = %u\n",
		   ctrl_txt, fq_en_cnt);
	seq_printf(file, "Total FQD with: %s : disabled = %u\n",
		   ctrl_txt, fq_di_cnt);
	return 0;
}

/*******************************************************************************
 *  QMan ctrl CGE, TDE, ORP, CTX, CPC, SFDR, BLOCK, HOLD, CACHE
 ******************************************************************************/
static int qman_fqd_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, qman_fqd_ctrl_show, inode->i_private);
}

static const struct file_operations qman_fqd_ctrl_fops =  {
	.owner          = THIS_MODULE,
	.open		= qman_fqd_ctrl_open,
	.read           = seq_read,
};

/*******************************************************************************
 *  QMan ctrl summary
 ******************************************************************************/
/*******************************************************************************
 *  QMan summary state
 ******************************************************************************/
static int qman_fqd_non_prog_summary_show(struct seq_file *file, void *offset)
{
	struct qm_mcr_queryfq_np np;
	struct qman_fq fq;
	int ret, i;
	u32 qm_fq_state_cnt[ARRAY_SIZE(fqd_states)];

	memset(qm_fq_state_cnt, 0, sizeof(qm_fq_state_cnt));

	for (i = 1; i < fqid_max; i++) {
		fq.fqid = i;
		ret = qman_query_fq_np(&fq, &np);
		if (ret)
			return ret;
		/* Keep a summary count of all states */
		if ((np.state & QM_MCR_NP_STATE_MASK) < ARRAY_SIZE(fqd_states))
			qm_fq_state_cnt[(np.state & QM_MCR_NP_STATE_MASK)]++;
	}

	for (i = 0; i < ARRAY_SIZE(fqd_states); i++) {
		seq_printf(file, "%s count = %u\n", state_txt[i],
			   qm_fq_state_cnt[i]);
	}
	return 0;
}

static int qman_fqd_prog_summary_show(struct seq_file *file, void *offset)
{
	struct qm_fqd fqd;
	struct qman_fq fq;
	int ret, i , j;
	u32 qm_prog_cnt[ARRAY_SIZE(mask_filter)/2];

	memset(qm_prog_cnt, 0, sizeof(qm_prog_cnt));

	for (i = 1; i < fqid_max; i++) {
		memset(&fqd, 0, sizeof(struct qm_fqd));
		fq.fqid = i;
		ret = qman_query_fq(&fq, &fqd);
		if (ret)
			return ret;
		/* Keep a summary count of all states */
		for (j = 0; j < ARRAY_SIZE(mask_filter); j += 2)
			if ((fqd.fq_ctrl & QM_FQCTRL_MASK) &
					mask_filter[j].mask)
				qm_prog_cnt[j/2]++;
	}
	for (i = 0; i < ARRAY_SIZE(mask_filter) / 2; i++) {
		seq_printf(file, "%s count = %u\n",
			get_fqd_ctrl_text(mask_filter[i*2].mask),
			   qm_prog_cnt[i]);
	}
	return 0;
}

static int qman_fqd_summary_show(struct seq_file *file, void *offset)
{
	int ret;

	/* Display summary of non programmable fields */
	ret = qman_fqd_non_prog_summary_show(file, offset);
	if (ret)
		return ret;
	seq_puts(file, "-----------------------------------------\n");
	/* Display programmable fields */
	ret = qman_fqd_prog_summary_show(file, offset);
	if (ret)
		return ret;
	return 0;
}

static int qman_fqd_summary_open(struct inode *inode, struct file *file)
{
	return single_open(file, qman_fqd_summary_show, NULL);
}

static const struct file_operations qman_fqd_summary_fops =  {
	.owner          = THIS_MODULE,
	.open		= qman_fqd_summary_open,
	.read           = seq_read,
};

/*******************************************************************************
 *  QMan destination work queue
 ******************************************************************************/
struct qman_dest_wq_s {
	u16 wq_id;
};
static struct qman_dest_wq_s qman_dest_wq_data = {
	.wq_id = 0,
};

static int qman_fqd_dest_wq_show(struct seq_file *file, void *offset)
{
	struct qm_fqd fqd;
	struct qman_fq fq;
	int ret, i;
	u16 *wq, wq_id = qman_dest_wq_data.wq_id;
	struct line_buffer_fq line_buf;

	memset(&line_buf, 0, sizeof(line_buf));
	/* use vmalloc : need to allocate large memory region and don't
	 * require the memory to be physically contiguous. */
	wq = vzalloc(sizeof(u16) * (0xFFFF+1));
	if (!wq)
		return -ENOMEM;

	seq_printf(file, "List of fq ids with destination work queue id"
			" = 0x%x\n", wq_id);

	for (i = 1; i < fqid_max; i++) {
		fq.fqid = i;
		memset(&fqd, 0, sizeof(struct qm_fqd));
		ret = qman_query_fq(&fq, &fqd);
		if (ret) {
			vfree(wq);
			return ret;
		}
		if (wq_id == fqd.dest_wq)
			add_to_line_buffer(&line_buf, fq.fqid, file);
		wq[fqd.dest_wq]++;
	}
	flush_line_buffer(&line_buf, file);

	seq_puts(file, "Summary of all FQD destination work queue values\n");
	for (i = 0; i < 0xFFFF; i++) {
		if (wq[i])
			seq_printf(file, "Channel: 0x%x WQ: 0x%x WQ_ID: 0x%x, "
				"count = %u\n", i >> 3, i & 0x3, i, wq[i]);
	}
	vfree(wq);
	return 0;
}

static ssize_t qman_fqd_dest_wq_write(struct file *f, const char __user *buf,
				      size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > 0xFFFF)
		return -EINVAL;
	qman_dest_wq_data.wq_id = val;
	return count;
}

static int qman_fqd_dest_wq_open(struct inode *inode, struct file *file)
{
	return single_open(file, qman_fqd_dest_wq_show, NULL);
}

static const struct file_operations qman_fqd_dest_wq_fops =  {
	.owner          = THIS_MODULE,
	.open		= qman_fqd_dest_wq_open,
	.read           = seq_read,
	.write		= qman_fqd_dest_wq_write,
};

/*******************************************************************************
 *  QMan Intra-Class Scheduling Credit
 ******************************************************************************/
static int qman_fqd_cred_show(struct seq_file *file, void *offset)
{
	struct qm_fqd fqd;
	struct qman_fq fq;
	int ret, i;
	u32 fq_cnt = 0;
	struct line_buffer_fq line_buf;

	memset(&line_buf, 0, sizeof(line_buf));
	seq_puts(file, "List of fq ids with Intra-Class Scheduling Credit > 0"
			"\n");

	for (i = 1; i < fqid_max; i++) {
		fq.fqid = i;
		memset(&fqd, 0, sizeof(struct qm_fqd));
		ret = qman_query_fq(&fq, &fqd);
		if (ret)
			return ret;
		if (fqd.ics_cred > 0) {
			add_to_line_buffer(&line_buf, fq.fqid, file);
			fq_cnt++;
		}
	}
	flush_line_buffer(&line_buf, file);

	seq_printf(file, "Total FQD with ics_cred > 0 = %d\n", fq_cnt);
	return 0;
}

static int qman_fqd_cred_open(struct inode *inode, struct file *file)
{
	return single_open(file, qman_fqd_cred_show, NULL);
}

static const struct file_operations qman_fqd_cred_fops =  {
	.owner          = THIS_MODULE,
	.open		= qman_fqd_cred_open,
	.read           = seq_read,
};

/*******************************************************************************
 *  Class Queue Fields
 ******************************************************************************/
struct query_cq_fields_data_s {
	u32 cqid;
};

static struct query_cq_fields_data_s query_cq_fields_data = {
	.cqid = 1,
};

static int query_cq_fields_show(struct seq_file *file, void *offset)
{
	int ret;
	struct qm_mcr_ceetm_cq_query query_result;
	unsigned int cqid;
	unsigned int portal;

	if ((qman_ip_rev & 0xFF00) < QMAN_REV30)
		return -EINVAL;

	cqid = query_cq_fields_data.cqid & 0x00FFFFFF;
	portal = query_cq_fields_data.cqid >> 24;
	if (portal > qm_dc_portal_fman1)
		return -EINVAL;

	ret = qman_ceetm_query_cq(cqid, portal, &query_result);
	if (ret)
		return ret;
	seq_printf(file, "Query CQ Fields Result cqid 0x%x on DCP %d\n",
								cqid, portal);
	seq_printf(file, " ccgid: %u\n", query_result.ccgid);
	seq_printf(file, " state: %u\n", query_result.state);
	seq_printf(file, " pfdr_hptr: %u\n", query_result.pfdr_hptr);
	seq_printf(file, " pfdr_tptr: %u\n", query_result.pfdr_tptr);
	seq_printf(file, " od1_xsfdr: %u\n", query_result.od1_xsfdr);
	seq_printf(file, " od2_xsfdr: %u\n", query_result.od2_xsfdr);
	seq_printf(file, " od3_xsfdr: %u\n", query_result.od3_xsfdr);
	seq_printf(file, " od4_xsfdr: %u\n", query_result.od4_xsfdr);
	seq_printf(file, " od5_xsfdr: %u\n", query_result.od5_xsfdr);
	seq_printf(file, " od6_xsfdr: %u\n", query_result.od6_xsfdr);
	seq_printf(file, " ra1_xsfdr: %u\n", query_result.ra1_xsfdr);
	seq_printf(file, " ra2_xsfdr: %u\n", query_result.ra2_xsfdr);
	seq_printf(file, " frame_count: %u\n", query_result.frm_cnt);

	return 0;
}

static int query_cq_fields_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, query_cq_fields_show, NULL);
}

static ssize_t query_cq_fields_write(struct file *f,
			const char __user *buf, size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	query_cq_fields_data.cqid = (u32)val;
	return count;
}

static const struct file_operations query_cq_fields_fops = {
	.owner          = THIS_MODULE,
	.open		= query_cq_fields_open,
	.read           = seq_read,
	.write		= query_cq_fields_write,
	.release	= single_release,
};

/*******************************************************************************
 *  READ CEETM_XSFDR_IN_USE
 ******************************************************************************/
struct query_ceetm_xsfdr_data_s {
	enum qm_dc_portal dcp_portal;
};

static struct query_ceetm_xsfdr_data_s query_ceetm_xsfdr_data;

static int query_ceetm_xsfdr_show(struct seq_file *file, void *offset)
{
	int ret;
	unsigned int xsfdr_in_use;
	enum qm_dc_portal portal;


	if (qman_ip_rev < QMAN_REV31)
		return -EINVAL;

	portal = query_ceetm_xsfdr_data.dcp_portal;
	ret = qman_ceetm_get_xsfdr(portal, &xsfdr_in_use);
	if (ret) {
		seq_printf(file, "Read CEETM_XSFDR_IN_USE on DCP %d failed\n",
								portal);
		return ret;
	}

	seq_printf(file, "DCP%d: CEETM_XSFDR_IN_USE number is %u\n", portal,
						(xsfdr_in_use & 0x1FFF));
	return 0;
}

static int query_ceetm_xsfdr_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, query_ceetm_xsfdr_show, NULL);
}

static ssize_t query_ceetm_xsfdr_write(struct file *f,
			const char __user *buf, size_t count, loff_t *off)
{
	int ret;
	unsigned long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;
	if (val > qm_dc_portal_fman1)
		return -EINVAL;
	query_ceetm_xsfdr_data.dcp_portal = (u32)val;
	return count;
}

static const struct file_operations query_ceetm_xsfdr_fops = {
	.owner          = THIS_MODULE,
	.open		= query_ceetm_xsfdr_open,
	.read           = seq_read,
	.write		= query_ceetm_xsfdr_write,
	.release	= single_release,
};

/* helper macros used in qman_debugfs_module_init */
#define QMAN_DBGFS_ENTRY(name, mode, parent, data, fops) \
	do { \
		d = debugfs_create_file(name, \
			mode, parent, \
			data, \
			fops); \
		if (d == NULL) { \
			ret = -ENOMEM; \
			goto _return; \
		} \
	} while (0)

/* dfs_root as parent */
#define QMAN_DBGFS_ENTRY_ROOT(name, mode, data, fops) \
	QMAN_DBGFS_ENTRY(name, mode, dfs_root, data, fops)

/* fqd_root as parent */
#define QMAN_DBGFS_ENTRY_FQDROOT(name, mode, data, fops) \
	QMAN_DBGFS_ENTRY(name, mode, fqd_root, data, fops)

/* fqd state */
#define QMAN_DBGFS_ENTRY_FQDSTATE(name, index) \
	QMAN_DBGFS_ENTRY_FQDROOT(name, S_IRUGO, \
	(void *)&mask_filter[index], &qman_fqd_ctrl_fops)

static int __init qman_debugfs_module_init(void)
{
	int ret = 0;
	struct dentry *d, *fqd_root;
	u32 reg;

	fqid_max = 0;
	init_ccsrmempeek();
	if (!qman_ccsr_start) {
		/* No QMan node found in device tree */
		return 0;
	}

	if (!qman_ccsrmempeek(&reg, QM_FQD_AR)) {
		/* extract the size of the FQD window */
		reg = reg & 0x3f;
		/* calculate valid frame queue descriptor range */
		fqid_max = (1 << (reg + 1)) / QM_FQD_BLOCK_SIZE;
	}
	dfs_root = debugfs_create_dir("qman", NULL);
	fqd_root = debugfs_create_dir("fqd", dfs_root);
	if (dfs_root == NULL || fqd_root == NULL) {
		ret = -ENOMEM;
		pr_err("Cannot create qman/fqd debugfs dir\n");
		goto _return;
	}
	if (fqid_max) {
		QMAN_DBGFS_ENTRY_ROOT("ccsrmempeek", S_IRUGO | S_IWUGO,
				NULL, &qman_ccsrmempeek_fops);
	}
	QMAN_DBGFS_ENTRY_ROOT("query_fq_np_fields", S_IRUGO | S_IWUGO,
		&query_fq_np_fields_data, &query_fq_np_fields_fops);

	QMAN_DBGFS_ENTRY_ROOT("query_fq_fields", S_IRUGO | S_IWUGO,
		&query_fq_fields_data, &query_fq_fields_fops);

	QMAN_DBGFS_ENTRY_ROOT("query_wq_lengths", S_IRUGO | S_IWUGO,
		&query_wq_lengths_data, &query_wq_lengths_fops);

	QMAN_DBGFS_ENTRY_ROOT("query_cgr", S_IRUGO | S_IWUGO,
		&query_cgr_data, &query_cgr_fops);

	QMAN_DBGFS_ENTRY_ROOT("query_congestion", S_IRUGO,
		NULL, &query_congestion_fops);

	QMAN_DBGFS_ENTRY_ROOT("testwrite_cgr", S_IRUGO,
		NULL, &testwrite_cgr_fops);

	QMAN_DBGFS_ENTRY_ROOT("testwrite_cgr_cgrid", S_IRUGO | S_IWUGO,
		NULL, &teswrite_cgr_cgrid_fops);

	QMAN_DBGFS_ENTRY_ROOT("testwrite_cgr_ibcnt", S_IRUGO | S_IWUGO,
		NULL, &teswrite_cgr_ibcnt_fops);

	QMAN_DBGFS_ENTRY_ROOT("query_ceetm_ccgr", S_IRUGO | S_IWUGO,
		&query_ccgr_data, &query_ccgr_fops);
	/* Create files with fqd_root as parent */

	QMAN_DBGFS_ENTRY_FQDROOT("stateoos", S_IRUGO,
		(void *)&fqd_states[QM_MCR_NP_STATE_OOS], &qman_fqd_state_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("state_retired", S_IRUGO,
		(void *)&fqd_states[QM_MCR_NP_STATE_RETIRED],
		&qman_fqd_state_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("state_tentatively_sched", S_IRUGO,
		(void *)&fqd_states[QM_MCR_NP_STATE_TEN_SCHED],
		&qman_fqd_state_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("state_truly_sched", S_IRUGO,
		(void *)&fqd_states[QM_MCR_NP_STATE_TRU_SCHED],
		&qman_fqd_state_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("state_parked", S_IRUGO,
		(void *)&fqd_states[QM_MCR_NP_STATE_PARKED],
		&qman_fqd_state_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("state_active", S_IRUGO,
		(void *)&fqd_states[QM_MCR_NP_STATE_ACTIVE],
		&qman_fqd_state_fops);
	QMAN_DBGFS_ENTRY_ROOT("query_cq_fields", S_IRUGO | S_IWUGO,
		&query_cq_fields_data, &query_cq_fields_fops);
	QMAN_DBGFS_ENTRY_ROOT("query_ceetm_xsfdr_in_use", S_IRUGO | S_IWUGO,
		&query_ceetm_xsfdr_data, &query_ceetm_xsfdr_fops);


	QMAN_DBGFS_ENTRY_FQDSTATE("cge_enable", 17);

	QMAN_DBGFS_ENTRY_FQDSTATE("cge_disable", 16);

	QMAN_DBGFS_ENTRY_FQDSTATE("tde_enable", 15);

	QMAN_DBGFS_ENTRY_FQDSTATE("tde_disable", 14);

	QMAN_DBGFS_ENTRY_FQDSTATE("orp_enable", 13);

	QMAN_DBGFS_ENTRY_FQDSTATE("orp_disable", 12);

	QMAN_DBGFS_ENTRY_FQDSTATE("ctx_a_stashing_enable", 11);

	QMAN_DBGFS_ENTRY_FQDSTATE("ctx_a_stashing_disable", 10);

	QMAN_DBGFS_ENTRY_FQDSTATE("cpc_enable", 9);

	QMAN_DBGFS_ENTRY_FQDSTATE("cpc_disable", 8);

	QMAN_DBGFS_ENTRY_FQDSTATE("sfdr_enable", 7);

	QMAN_DBGFS_ENTRY_FQDSTATE("sfdr_disable", 6);

	QMAN_DBGFS_ENTRY_FQDSTATE("avoid_blocking_enable", 5);

	QMAN_DBGFS_ENTRY_FQDSTATE("avoid_blocking_disable", 4);

	QMAN_DBGFS_ENTRY_FQDSTATE("hold_active_enable", 3);

	QMAN_DBGFS_ENTRY_FQDSTATE("hold_active_disable", 2);

	QMAN_DBGFS_ENTRY_FQDSTATE("prefer_in_cache_enable", 1);

	QMAN_DBGFS_ENTRY_FQDSTATE("prefer_in_cache_disable", 0);

	QMAN_DBGFS_ENTRY_FQDROOT("summary", S_IRUGO,
		NULL, &qman_fqd_summary_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("wq", S_IRUGO | S_IWUGO,
		NULL, &qman_fqd_dest_wq_fops);

	QMAN_DBGFS_ENTRY_FQDROOT("cred", S_IRUGO,
		NULL, &qman_fqd_cred_fops);

	return 0;

_return:
	debugfs_remove_recursive(dfs_root);
	return ret;
}

static void __exit qman_debugfs_module_exit(void)
{
	debugfs_remove_recursive(dfs_root);
}

module_init(qman_debugfs_module_init);
module_exit(qman_debugfs_module_exit);
MODULE_LICENSE("Dual BSD/GPL");
