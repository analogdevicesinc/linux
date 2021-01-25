/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 NXP.
 */

#ifndef __DCSS_PRV_H__
#define __DCSS_PRV_H__

#include <drm/drm_atomic.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <video/videomode.h>

#define SET			0x04
#define CLR			0x08
#define TGL			0x0C

#define dcss_writel(v, c)	writel((v), (c))
#define dcss_readl(c)		readl(c)
#define dcss_set(v, c)		writel((v), (c) + SET)
#define dcss_clr(v, c)		writel((v), (c) + CLR)
#define dcss_toggle(v, c)	writel((v), (c) + TGL)

static inline void dcss_update(u32 v, u32 m, void __iomem *c)
{
	writel((readl(c) & ~(m)) | (v), (c));
}

#define DCSS_DBG_REG(reg)	{.name = #reg, .ofs = reg}

enum {
	DCSS_IMX8MQ = 0,
};

struct dcss_type_data {
	const char *name;
	u32 blkctl_ofs;
	u32 ctxld_ofs;
	u32 rdsrc_ofs;
	u32 wrscl_ofs;
	u32 dtg_ofs;
	u32 scaler_ofs;
	u32 ss_ofs;
	u32 dpr_ofs;
	u32 dtrc_ofs;
	u32 dec400d_ofs;
	u32 hdr10_ofs;
};

struct dcss_debug_reg {
	char *name;
	u32 ofs;
};

enum dcss_ctxld_ctx_type {
	CTX_DB,
	CTX_SB_HP, /* high-priority */
	CTX_SB_LP, /* low-priority  */
};

enum dcss_pixel_pipe_output {
	DCSS_PIPE_OUTPUT_RGB = 0,
	DCSS_PIPE_OUTPUT_YUV444,
	DCSS_PIPE_OUTPUT_YUV422,
	DCSS_PIPE_OUTPUT_YUV420,
};

struct dcss_dev {
	struct device *dev;
	const struct dcss_type_data *devtype;
	struct device_node *of_port;

	u32 start_addr;

	struct dcss_blkctl *blkctl;
	struct dcss_ctxld *ctxld;
	struct dcss_dpr *dpr;
	struct dcss_dtg *dtg;
	struct dcss_ss *ss;
	struct dcss_hdr10 *hdr10;
	struct dcss_scaler *scaler;
	struct dcss_dtrc *dtrc;
	struct dcss_dec400d *dec400d;
	struct dcss_wrscl *wrscl;
	struct dcss_rdsrc *rdsrc;

	struct clk *apb_clk;
	struct clk *axi_clk;
	struct clk *pix_clk;
	struct clk *rtrm_clk;
	struct clk *dtrc_clk;
	struct clk *pll_src_clk;
	struct clk *pll_phy_ref_clk;

	bool hdmi_output;

	void (*disable_callback)(void *data);
	struct completion disable_completion;
};

struct dcss_dev *dcss_drv_dev_to_dcss(struct device *dev);
struct drm_device *dcss_drv_dev_to_drm(struct device *dev);
bool dcss_drv_is_componentized(struct device *dev);
struct dcss_dev *dcss_dev_create(struct device *dev, bool hdmi_output);
void dcss_dev_destroy(struct dcss_dev *dcss);
void dcss_enable_dtg_and_ss(struct dcss_dev *dcss);
void dcss_disable_dtg_and_ss(struct dcss_dev *dcss);

extern const struct dev_pm_ops dcss_dev_pm_ops;

/* BLKCTL */
int dcss_blkctl_init(struct dcss_dev *dcss, unsigned long blkctl_base);
void dcss_blkctl_cfg(struct dcss_blkctl *blkctl);

/* CTXLD */
int dcss_ctxld_init(struct dcss_dev *dcss, unsigned long ctxld_base);
void dcss_ctxld_exit(struct dcss_ctxld *ctxld);
void dcss_ctxld_write(struct dcss_ctxld *ctxld, u32 ctx_id,
		      u32 val, u32 reg_idx);
int dcss_ctxld_resume(struct dcss_ctxld *dcss_ctxld);
int dcss_ctxld_suspend(struct dcss_ctxld *dcss_ctxld);
void dcss_ctxld_write_irqsafe(struct dcss_ctxld *ctlxd, u32 ctx_id, u32 val,
			      u32 reg_ofs);
void dcss_ctxld_kick(struct dcss_ctxld *ctxld);
bool dcss_ctxld_is_flushed(struct dcss_ctxld *ctxld);
int dcss_ctxld_enable(struct dcss_ctxld *ctxld);
void dcss_ctxld_register_completion(struct dcss_ctxld *ctxld,
				    struct completion *dis_completion);
void dcss_ctxld_assert_locked(struct dcss_ctxld *ctxld);
void dcss_ctxld_register_dtrc_cb(struct dcss_ctxld *ctxld,
				 bool (*cb)(void *),
				 void *data);

/* DPR */
int dcss_dpr_init(struct dcss_dev *dcss, unsigned long dpr_base);
void dcss_dpr_exit(struct dcss_dpr *dpr);
void dcss_dpr_write_sysctrl(struct dcss_dpr *dpr);
void dcss_dpr_set_res(struct dcss_dpr *dpr, int ch_num, u32 xres, u32 yres);
void dcss_dpr_addr_set(struct dcss_dpr *dpr, int ch_num, u32 luma_base_addr,
		       u32 chroma_base_addr, u16 pitch);
void dcss_dpr_enable(struct dcss_dpr *dpr, int ch_num, bool en);
void dcss_dpr_format_set(struct dcss_dpr *dpr, int ch_num,
			 const struct drm_format_info *format, u64 modifier);
void dcss_dpr_set_rotation(struct dcss_dpr *dpr, int ch_num, u32 rotation);

/* DTG */
int dcss_dtg_init(struct dcss_dev *dcss, unsigned long dtg_base);
void dcss_dtg_exit(struct dcss_dtg *dtg);
bool dcss_dtg_vblank_irq_valid(struct dcss_dtg *dtg);
void dcss_dtg_vblank_irq_enable(struct dcss_dtg *dtg, bool en);
void dcss_dtg_vblank_irq_clear(struct dcss_dtg *dtg);
void dcss_dtg_sync_set(struct dcss_dtg *dtg, struct videomode *vm);
void dcss_dtg_css_set(struct dcss_dtg *dtg,
		      enum dcss_pixel_pipe_output output_encoding);
void dcss_dtg_enable(struct dcss_dtg *dtg);
void dcss_dtg_shutoff(struct dcss_dtg *dtg);
bool dcss_dtg_is_enabled(struct dcss_dtg *dtg);
void dcss_dtg_ctxld_kick_irq_enable(struct dcss_dtg *dtg, bool en);
bool dcss_dtg_global_alpha_changed(struct dcss_dtg *dtg, int ch_num, int alpha);
void dcss_dtg_plane_alpha_set(struct dcss_dtg *dtg, int ch_num,
			      const struct drm_format_info *format, int alpha);
void dcss_dtg_plane_pos_set(struct dcss_dtg *dtg, int ch_num,
			    int px, int py, int pw, int ph);
void dcss_dtg_ch_enable(struct dcss_dtg *dtg, int ch_num, bool en);

/* SUBSAM */
int dcss_ss_init(struct dcss_dev *dcss, unsigned long subsam_base);
void dcss_ss_exit(struct dcss_ss *ss);
void dcss_ss_enable(struct dcss_ss *ss);
void dcss_ss_shutoff(struct dcss_ss *ss);
void dcss_ss_subsam_set(struct dcss_ss *ss,
			enum dcss_pixel_pipe_output output_encoding);
void dcss_ss_sync_set(struct dcss_ss *ss, struct videomode *vm,
		      bool phsync, bool pvsync);

/* SCALER */
int dcss_scaler_init(struct dcss_dev *dcss, unsigned long scaler_base);
void dcss_scaler_exit(struct dcss_scaler *scl);
void dcss_scaler_set_filter(struct dcss_scaler *scl, int ch_num,
			    enum drm_scaling_filter scaling_filter);
void dcss_scaler_setup(struct dcss_scaler *scl, int ch_num,
		       const struct drm_format_info *format,
		       int src_xres, int src_yres, int dst_xres, int dst_yres,
		       u32 vrefresh_hz);
void dcss_scaler_ch_enable(struct dcss_scaler *scl, int ch_num, bool en);
int dcss_scaler_get_min_max_ratios(struct dcss_scaler *scl, int ch_num,
				   int *min, int *max);
void dcss_scaler_write_sclctrl(struct dcss_scaler *scl);

/* DEC400D */

#define VIV_VIDMEM_METADATA_MAGIC fourcc_code('v', 'i', 'v', 'm')

/* Compressed format now was defined same as dec400d, should be general. */
typedef enum _VIV_COMPRESS_FMT
{
    _VIV_CFMT_ARGB8 = 0,
    _VIV_CFMT_XRGB8,
    _VIV_CFMT_AYUV,
    _VIV_CFMT_UYVY,
    _VIV_CFMT_YUY2,
    _VIV_CFMT_YUV_ONLY,
    _VIV_CFMT_UV_MIX,
    _VIV_CFMT_ARGB4,
    _VIV_CFMT_XRGB4,
    _VIV_CFMT_A1R5G5B5,
    _VIV_CFMT_X1R5G5B5,
    _VIV_CFMT_R5G6B5,
    _VIV_CFMT_Z24S8,
    _VIV_CFMT_Z24,
    _VIV_CFMT_Z16,
    _VIV_CFMT_A2R10G10B10,
    _VIV_CFMT_BAYER,
    _VIV_CFMT_SIGNED_BAYER,
    _VIV_CFMT_VAA16,
    _VIV_CFMT_S8,

    _VIV_CFMT_MAX,
} _VIV_COMPRESS_FMT;

/* Metadata for cross-device fd share with additional (ts) info. */
typedef struct _VIV_VIDMEM_METADATA
{
    uint32_t magic;

    int32_t  ts_fd;
    void *   ts_dma_buf;

    uint32_t fc_enabled;
    uint32_t fc_value;
    uint32_t fc_value_upper;

    uint32_t compressed;
    uint32_t compress_format;
} _VIV_VIDMEM_METADATA;

int dcss_dec400d_init(struct dcss_dev *dcss, unsigned long dec400d_base);
void dcss_dec400d_exit(struct dcss_dec400d *dec400d);
void dcss_dec400d_bypass(struct dcss_dec400d *dec400d);
void dcss_dec400d_shadow_trig(struct dcss_dec400d *dec400d);
void dcss_dec400d_enable(struct dcss_dec400d *dec400d);
void dcss_dec400d_fast_clear_config(struct dcss_dec400d *dec400d,
				    u32 fc_value,
				    bool enable);
void dcss_dec400d_read_config(struct dcss_dec400d *dec400d,
			      u32 read_id,
			      bool compress_en,
			      u32 compress_format);
void dcss_dec400d_addr_set(struct dcss_dec400d *dec400d, u32 baddr, u32 caddr);

/* HDR10 */
enum dcss_hdr10_nonlinearity {
	NL_REC2084,
	NL_REC709,
	NL_BT1886,
	NL_2100HLG,
	NL_SRGB,
};

enum dcss_hdr10_pixel_range {
	PR_LIMITED,
	PR_FULL,
};

enum dcss_hdr10_gamut {
	G_REC2020,
	G_REC709,
	G_REC601_NTSC,
	G_REC601_PAL,
	G_ADOBE_ARGB,
};

struct dcss_hdr10_pipe_cfg {
	bool is_yuv;
	enum dcss_hdr10_nonlinearity nl;
	enum dcss_hdr10_pixel_range pr;
	enum dcss_hdr10_gamut g;
};

int dcss_hdr10_init(struct dcss_dev *dcss, unsigned long hdr10_base);
void dcss_hdr10_exit(struct dcss_hdr10 *hdr10);
bool dcss_hdr10_pipe_cfg_is_supported(struct dcss_hdr10 *hdr10,
				      struct dcss_hdr10_pipe_cfg *ipipe_cfg,
				      struct dcss_hdr10_pipe_cfg *opipe_cfg);
void dcss_hdr10_setup(struct dcss_hdr10 *hdr10, int ch_num,
		      struct dcss_hdr10_pipe_cfg *ipipe_cfg,
		      struct dcss_hdr10_pipe_cfg *opipe_cfg);

/* enums common to both WRSCL and RDSRC */
enum dcss_wrscl_rdsrc_psize {
	PSIZE_64,
	PSIZE_128,
	PSIZE_256,
	PSIZE_512,
	PSIZE_1024,
	PSIZE_2048,
	PSIZE_4096,
};

enum dcss_wrscl_rdsrc_tsize {
	TSIZE_64,
	TSIZE_128,
	TSIZE_256,
	TSIZE_512,
};

enum dcss_wrscl_rdsrc_fifo_size {
	FIFO_512,
	FIFO_1024,
	FIFO_2048,
	FIFO_4096,
};

enum dcss_wrscl_rdsrc_bpp {
	BPP_38, /* 38 bit unpacked components */
	BPP_32_UPCONVERT,
	BPP_32_10BIT_OUTPUT,
	BPP_20, /* 10-bit YUV422 */
	BPP_16, /* 8-bit YUV422 */
};

/* WRSCL */
int dcss_wrscl_init(struct dcss_dev *dcss, unsigned long wrscl_base);
void dcss_wrscl_exit(struct dcss_wrscl *wrscl);
u32 dcss_wrscl_setup(struct dcss_wrscl *wrscl, u32 pix_format, u32 pix_clk_hz,
		     u32 dst_xres, u32 dst_yres);
void dcss_wrscl_enable(struct dcss_wrscl *wrscl);
void dcss_wrscl_disable(struct dcss_wrscl *wrscl);

/* RDSRC */
int dcss_rdsrc_init(struct dcss_dev *dcss, unsigned long rdsrc_base);
void dcss_rdsrc_exit(struct dcss_rdsrc *rdsrc);
void dcss_rdsrc_setup(struct dcss_rdsrc *rdsrc, u32 pix_format, u32 dst_xres,
		      u32 dst_yres, u32 base_addr);
void dcss_rdsrc_enable(struct dcss_rdsrc *rdsrc);
void dcss_rdsrc_disable(struct dcss_rdsrc *rdsrc);

/* DTRC */
int dcss_dtrc_init(struct dcss_dev *dcss, unsigned long dtrc_base);
void dcss_dtrc_exit(struct dcss_dtrc *dtrc);
void dcss_dtrc_bypass(struct dcss_dtrc *dtrc, int ch_num);
void dcss_dtrc_set_format_mod(struct dcss_dtrc *dtrc, int ch_num, u64 modifier);
void dcss_dtrc_addr_set(struct dcss_dtrc *dtrc, int ch_num,
			u32 p1_ba, u32 p2_ba, uint64_t dec_table_ofs);
bool dcss_dtrc_ch_running(struct dcss_dtrc *dtrc, int ch_num);
bool dcss_dtrc_is_running(struct dcss_dtrc *dtrc);
void dcss_dtrc_enable(struct dcss_dtrc *dtrc, int ch_num, bool enable);
void dcss_dtrc_set_res(struct dcss_dtrc *dtrc, int ch_num,
		       struct drm_plane_state *state, u32 *dtrc_w, u32 *dtrc_h);
void dcss_dtrc_switch_banks(struct dcss_dtrc *dtrc);

#endif /* __DCSS_PRV_H__ */
