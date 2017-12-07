#ifndef __DCSS_PRV_H__
#define __DCSS_PRV_H__

#include <linux/pm_qos.h>

#define SET 0x04
#define CLR 0x08
#define TGL 0x0C

#define dcss_writel(v, c)   writel((v), (c))
#define dcss_readl(c)	    readl(c)
#define dcss_set(v, c)	    writel((v), (c) + SET)
#define dcss_clr(v, c)	    writel((v), (c) + CLR)
#define dcss_toggle(v, c)   writel((v), (c) + TGL)
#define dcss_update(v, m, c) writel((readl(c) & ~(m)) | (v), (c))

#define DCSS_DBG_REG(reg)	{.name = #reg, .ofs = reg}

struct dcss_debug_reg {
	char *name;
	u32 ofs;
};

enum dcss_ctxld_ctx_type {
	CTX_DB,
	CTX_SB_HP, /* high-priority */
	CTX_SB_LP, /* low-priority  */
};

struct dcss_soc;
struct dcss_devtype;

struct dcss_soc {
	struct device *dev;
	const struct dcss_devtype *devtype;

	u32 start_addr;

	struct dcss_blkctl_priv *blkctl_priv;
	struct dcss_ctxld_priv *ctxld_priv;
	struct dcss_dpr_priv *dpr_priv;
	struct dcss_dtg_priv *dtg_priv;
	struct dcss_ss_priv *ss_priv;
	struct dcss_hdr10_priv *hdr10_priv;
	struct dcss_scaler_priv *scaler_priv;
	struct dcss_dtrc_priv *dtrc_priv;
	struct dcss_dec400d_priv *dec400d_priv;

	struct clk *apb_clk;
	struct clk *axi_clk;
	struct clk *p_clk;
	struct clk *rtrm_clk;
	struct clk *dtrc_clk;

	void (*dcss_disable_callback)(void *data);

	bool bus_freq_req;
	bool clks_on;

	struct pm_qos_request pm_qos_req;
};

/* BLKCTL */
int dcss_blkctl_init(struct dcss_soc *dcss, unsigned long blkctl_base);
void dcss_blkctl_cfg(struct dcss_soc *dcss);
void dcss_blkctl_exit(struct dcss_soc *dcss);

/* CTXLD */
int dcss_ctxld_init(struct dcss_soc *dcss, unsigned long ctxld_base);
void dcss_ctxld_hw_cfg(struct dcss_soc *dcss);
void dcss_ctxld_exit(struct dcss_soc *dcss);
void dcss_ctxld_write(struct dcss_soc *dcss, u32 ctx_id, u32 val, u32 reg_idx);
void dcss_ctxld_update(struct dcss_soc *dcss, u32 ctx_id, u32 val, u32 mask,
		       u32 reg_idx);
void dcss_ctxld_dump(struct seq_file *s, void *data);
int dcss_ctxld_resume(struct dcss_soc *dcss);
int dcss_ctxld_suspend(struct dcss_soc *dcss);

/* DPR */
int dcss_dpr_init(struct dcss_soc *dcss, unsigned long dpr_base);
void dcss_dpr_exit(struct dcss_soc *dcss);

/* DTG */
int dcss_dtg_init(struct dcss_soc *dcss, unsigned long dtg_base);
void dcss_dtg_exit(struct dcss_soc *dcss);
void dcss_dtg_vblank_irq_enable(struct dcss_soc *dcss, bool en);
void dcss_dtg_vblank_irq_clear(struct dcss_soc *dcss);

/* SUBSAM */
int dcss_ss_init(struct dcss_soc *dcss, unsigned long subsam_base);
void dcss_ss_exit(struct dcss_soc *dcss);

/* HDR10 */
int dcss_hdr10_init(struct dcss_soc *dcss, unsigned long hdr10_base);
void dcss_hdr10_exit(struct dcss_soc *dcss);
void dcss_hdr10_cfg(struct dcss_soc *dcss);

/* SCALER */
int dcss_scaler_init(struct dcss_soc *dcss, unsigned long scaler_base);
void dcss_scaler_exit(struct dcss_soc *dcss);

/* DTRC */
int dcss_dtrc_init(struct dcss_soc *dcss, unsigned long dtrc_base);
void dcss_dtrc_exit(struct dcss_soc *dcss);

/* DEC400d */
int dcss_dec400d_init(struct dcss_soc *dcss, unsigned long dec400d_base);
void dcss_dec400d_exit(struct dcss_soc *dcss);

void dcss_blkctl_dump_regs(struct seq_file *s, void *data);
void dcss_dtrc_dump_regs(struct seq_file *s, void *data);
void dcss_dpr_dump_regs(struct seq_file *s, void *data);
void dcss_dtg_dump_regs(struct seq_file *s, void *data);
void dcss_ss_dump_regs(struct seq_file *s, void *data);
void dcss_scaler_dump_regs(struct seq_file *s, void *data);
void dcss_ctxld_dump_regs(struct seq_file *s, void *data);
void dcss_hdr10_dump_regs(struct seq_file *s, void *data);
#endif /* __DCSS_PRV_H__ */
