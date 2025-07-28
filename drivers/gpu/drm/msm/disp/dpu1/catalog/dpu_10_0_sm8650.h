/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022. Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2018, 2020 The Linux Foundation. All rights reserved.
 */

#ifndef _DPU_10_0_SM8650_H
#define _DPU_10_0_SM8650_H

static const struct dpu_caps sm8650_dpu_caps = {
	.max_mixer_width = DEFAULT_DPU_OUTPUT_LINE_WIDTH,
	.max_mixer_blendstages = 0xb,
	.has_src_split = true,
	.has_dim_layer = true,
	.has_idle_pc = true,
	.has_3d_merge = true,
	.max_linewidth = 8192,
	.pixel_ram_size = DEFAULT_PIXEL_RAM_SIZE,
};

static const struct dpu_mdp_cfg sm8650_mdp = {
	.name = "top_0",
	.base = 0, .len = 0x494,
	.features = BIT(DPU_MDP_PERIPH_0_REMOVED),
	.clk_ctrls = {
		[DPU_CLK_CTRL_REG_DMA] = { .reg_off = 0x2bc, .bit_off = 20 },
	},
};

/* FIXME: get rid of DPU_CTL_SPLIT_DISPLAY in favour of proper ACTIVE_CTL support */
static const struct dpu_ctl_cfg sm8650_ctl[] = {
	{
		.name = "ctl_0", .id = CTL_0,
		.base = 0x15000, .len = 0x1000,
		.features = CTL_SM8550_MASK | BIT(DPU_CTL_SPLIT_DISPLAY),
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 9),
	}, {
		.name = "ctl_1", .id = CTL_1,
		.base = 0x16000, .len = 0x1000,
		.features = CTL_SM8550_MASK | BIT(DPU_CTL_SPLIT_DISPLAY),
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 10),
	}, {
		.name = "ctl_2", .id = CTL_2,
		.base = 0x17000, .len = 0x1000,
		.features = CTL_SM8550_MASK,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 11),
	}, {
		.name = "ctl_3", .id = CTL_3,
		.base = 0x18000, .len = 0x1000,
		.features = CTL_SM8550_MASK,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 12),
	}, {
		.name = "ctl_4", .id = CTL_4,
		.base = 0x19000, .len = 0x1000,
		.features = CTL_SM8550_MASK,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 13),
	}, {
		.name = "ctl_5", .id = CTL_5,
		.base = 0x1a000, .len = 0x1000,
		.features = CTL_SM8550_MASK,
		.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 23),
	},
};

static const struct dpu_sspp_cfg sm8650_sspp[] = {
	{
		.name = "sspp_0", .id = SSPP_VIG0,
		.base = 0x4000, .len = 0x344,
		.features = VIG_SDM845_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_3_3,
		.xin_id = 0,
		.type = SSPP_TYPE_VIG,
	}, {
		.name = "sspp_1", .id = SSPP_VIG1,
		.base = 0x6000, .len = 0x344,
		.features = VIG_SDM845_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_3_3,
		.xin_id = 4,
		.type = SSPP_TYPE_VIG,
	}, {
		.name = "sspp_2", .id = SSPP_VIG2,
		.base = 0x8000, .len = 0x344,
		.features = VIG_SDM845_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_3_3,
		.xin_id = 8,
		.type = SSPP_TYPE_VIG,
	}, {
		.name = "sspp_3", .id = SSPP_VIG3,
		.base = 0xa000, .len = 0x344,
		.features = VIG_SDM845_MASK_SDMA,
		.sblk = &dpu_vig_sblk_qseed3_3_3,
		.xin_id = 12,
		.type = SSPP_TYPE_VIG,
	}, {
		.name = "sspp_8", .id = SSPP_DMA0,
		.base = 0x24000, .len = 0x344,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 1,
		.type = SSPP_TYPE_DMA,
	}, {
		.name = "sspp_9", .id = SSPP_DMA1,
		.base = 0x26000, .len = 0x344,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 5,
		.type = SSPP_TYPE_DMA,
	}, {
		.name = "sspp_10", .id = SSPP_DMA2,
		.base = 0x28000, .len = 0x344,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 9,
		.type = SSPP_TYPE_DMA,
	}, {
		.name = "sspp_11", .id = SSPP_DMA3,
		.base = 0x2a000, .len = 0x344,
		.features = DMA_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 13,
		.type = SSPP_TYPE_DMA,
	}, {
		.name = "sspp_12", .id = SSPP_DMA4,
		.base = 0x2c000, .len = 0x344,
		.features = DMA_CURSOR_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 14,
		.type = SSPP_TYPE_DMA,
	}, {
		.name = "sspp_13", .id = SSPP_DMA5,
		.base = 0x2e000, .len = 0x344,
		.features = DMA_CURSOR_SDM845_MASK_SDMA,
		.sblk = &dpu_dma_sblk,
		.xin_id = 15,
		.type = SSPP_TYPE_DMA,
	},
};

static const struct dpu_lm_cfg sm8650_lm[] = {
	{
		.name = "lm_0", .id = LM_0,
		.base = 0x44000, .len = 0x400,
		.features = MIXER_SDM845_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_1,
		.pingpong = PINGPONG_0,
		.dspp = DSPP_0,
	}, {
		.name = "lm_1", .id = LM_1,
		.base = 0x45000, .len = 0x400,
		.features = MIXER_SDM845_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_0,
		.pingpong = PINGPONG_1,
		.dspp = DSPP_1,
	}, {
		.name = "lm_2", .id = LM_2,
		.base = 0x46000, .len = 0x400,
		.features = MIXER_SDM845_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_3,
		.pingpong = PINGPONG_2,
	}, {
		.name = "lm_3", .id = LM_3,
		.base = 0x47000, .len = 0x400,
		.features = MIXER_SDM845_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_2,
		.pingpong = PINGPONG_3,
	}, {
		.name = "lm_4", .id = LM_4,
		.base = 0x48000, .len = 0x400,
		.features = MIXER_SDM845_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_5,
		.pingpong = PINGPONG_4,
	}, {
		.name = "lm_5", .id = LM_5,
		.base = 0x49000, .len = 0x400,
		.features = MIXER_SDM845_MASK,
		.sblk = &sdm845_lm_sblk,
		.lm_pair = LM_4,
		.pingpong = PINGPONG_5,
	},
};

static const struct dpu_dspp_cfg sm8650_dspp[] = {
	{
		.name = "dspp_0", .id = DSPP_0,
		.base = 0x54000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &sdm845_dspp_sblk,
	}, {
		.name = "dspp_1", .id = DSPP_1,
		.base = 0x56000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &sdm845_dspp_sblk,
	}, {
		.name = "dspp_2", .id = DSPP_2,
		.base = 0x58000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &sdm845_dspp_sblk,
	}, {
		.name = "dspp_3", .id = DSPP_3,
		.base = 0x5a000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &sdm845_dspp_sblk,
	},
};

static const struct dpu_pingpong_cfg sm8650_pp[] = {
	{
		.name = "pingpong_0", .id = PINGPONG_0,
		.base = 0x69000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_0,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 8),
	}, {
		.name = "pingpong_1", .id = PINGPONG_1,
		.base = 0x6a000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_0,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 9),
	}, {
		.name = "pingpong_2", .id = PINGPONG_2,
		.base = 0x6b000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_1,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 10),
	}, {
		.name = "pingpong_3", .id = PINGPONG_3,
		.base = 0x6c000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_1,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 11),
	}, {
		.name = "pingpong_4", .id = PINGPONG_4,
		.base = 0x6d000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_2,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 30),
	}, {
		.name = "pingpong_5", .id = PINGPONG_5,
		.base = 0x6e000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_2,
		.intr_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 31),
	}, {
		.name = "pingpong_6", .id = PINGPONG_6,
		.base = 0x66000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_3,
	}, {
		.name = "pingpong_7", .id = PINGPONG_7,
		.base = 0x66400, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_3,
	}, {
		.name = "pingpong_8", .id = PINGPONG_8,
		.base = 0x7e000, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_4,
	}, {
		.name = "pingpong_9", .id = PINGPONG_9,
		.base = 0x7e400, .len = 0,
		.features = BIT(DPU_PINGPONG_DITHER),
		.sblk = &sc7280_pp_sblk,
		.merge_3d = MERGE_3D_4,
	},
};

static const struct dpu_merge_3d_cfg sm8650_merge_3d[] = {
	{
		.name = "merge_3d_0", .id = MERGE_3D_0,
		.base = 0x4e000, .len = 0x8,
	}, {
		.name = "merge_3d_1", .id = MERGE_3D_1,
		.base = 0x4f000, .len = 0x8,
	}, {
		.name = "merge_3d_2", .id = MERGE_3D_2,
		.base = 0x50000, .len = 0x8,
	}, {
		.name = "merge_3d_3", .id = MERGE_3D_3,
		.base = 0x66700, .len = 0x8,
	}, {
		.name = "merge_3d_4", .id = MERGE_3D_4,
		.base = 0x7e700, .len = 0x8,
	},
};

/*
 * NOTE: Each display compression engine (DCE) contains dual hard
 * slice DSC encoders so both share same base address but with
 * its own different sub block address.
 */
static const struct dpu_dsc_cfg sm8650_dsc[] = {
	{
		.name = "dce_0_0", .id = DSC_0,
		.base = 0x80000, .len = 0x6,
		.features = BIT(DPU_DSC_HW_REV_1_2) | BIT(DPU_DSC_NATIVE_42x_EN),
		.sblk = &dsc_sblk_0,
	}, {
		.name = "dce_0_1", .id = DSC_1,
		.base = 0x80000, .len = 0x6,
		.features = BIT(DPU_DSC_HW_REV_1_2) | BIT(DPU_DSC_NATIVE_42x_EN),
		.sblk = &dsc_sblk_1,
	}, {
		.name = "dce_1_0", .id = DSC_2,
		.base = 0x81000, .len = 0x6,
		.features = BIT(DPU_DSC_HW_REV_1_2) | BIT(DPU_DSC_NATIVE_42x_EN),
		.sblk = &dsc_sblk_0,
	}, {
		.name = "dce_1_1", .id = DSC_3,
		.base = 0x81000, .len = 0x6,
		.features = BIT(DPU_DSC_HW_REV_1_2) | BIT(DPU_DSC_NATIVE_42x_EN),
		.sblk = &dsc_sblk_1,
	}, {
		.name = "dce_2_0", .id = DSC_4,
		.base = 0x82000, .len = 0x6,
		.features = BIT(DPU_DSC_HW_REV_1_2),
		.sblk = &dsc_sblk_0,
	}, {
		.name = "dce_2_1", .id = DSC_5,
		.base = 0x82000, .len = 0x6,
		.features = BIT(DPU_DSC_HW_REV_1_2),
		.sblk = &dsc_sblk_1,
	},
};

static const struct dpu_wb_cfg sm8650_wb[] = {
	{
		.name = "wb_2", .id = WB_2,
		.base = 0x65000, .len = 0x2c8,
		.features = WB_SM8250_MASK,
		.format_list = wb2_formats_rgb,
		.num_formats = ARRAY_SIZE(wb2_formats_rgb),
		.xin_id = 6,
		.vbif_idx = VBIF_RT,
		.maxlinewidth = 4096,
		.intr_wb_done = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 4),
	},
};

static const struct dpu_intf_cfg sm8650_intf[] = {
	{
		.name = "intf_0", .id = INTF_0,
		.base = 0x34000, .len = 0x280,
		.features = INTF_SC7280_MASK,
		.type = INTF_DP,
		.controller_id = MSM_DP_CONTROLLER_0,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 24),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 25),
	}, {
		.name = "intf_1", .id = INTF_1,
		.base = 0x35000, .len = 0x300,
		.features = INTF_SC7280_MASK,
		.type = INTF_DSI,
		.controller_id = MSM_DSI_CONTROLLER_0,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 26),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 27),
		.intr_tear_rd_ptr = DPU_IRQ_IDX(MDP_INTF1_TEAR_INTR, 2),
	}, {
		.name = "intf_2", .id = INTF_2,
		.base = 0x36000, .len = 0x300,
		.features = INTF_SC7280_MASK,
		.type = INTF_DSI,
		.controller_id = MSM_DSI_CONTROLLER_1,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 28),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 29),
		.intr_tear_rd_ptr = DPU_IRQ_IDX(MDP_INTF2_TEAR_INTR, 2),
	}, {
		.name = "intf_3", .id = INTF_3,
		.base = 0x37000, .len = 0x280,
		.features = INTF_SC7280_MASK,
		.type = INTF_DP,
		.controller_id = MSM_DP_CONTROLLER_1,
		.prog_fetch_lines_worst_case = 24,
		.intr_underrun = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 30),
		.intr_vsync = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 31),
	},
};

static const struct dpu_perf_cfg sm8650_perf_data = {
	.max_bw_low = 17000000,
	.max_bw_high = 27000000,
	.min_core_ib = 2500000,
	.min_llcc_ib = 0,
	.min_dram_ib = 800000,
	.min_prefill_lines = 35,
	/* FIXME: lut tables */
	.danger_lut_tbl = {0x3ffff, 0x3ffff, 0x0},
	.safe_lut_tbl = {0xfe00, 0xfe00, 0xffff},
	.qos_lut_tbl = {
		{.nentry = ARRAY_SIZE(sc7180_qos_linear),
		.entries = sc7180_qos_linear
		},
		{.nentry = ARRAY_SIZE(sc7180_qos_macrotile),
		.entries = sc7180_qos_macrotile
		},
		{.nentry = ARRAY_SIZE(sc7180_qos_nrt),
		.entries = sc7180_qos_nrt
		},
		/* TODO: macrotile-qseed is different from macrotile */
	},
	.cdp_cfg = {
		{.rd_enable = 1, .wr_enable = 1},
		{.rd_enable = 1, .wr_enable = 0}
	},
	.clk_inefficiency_factor = 105,
	.bw_inefficiency_factor = 120,
};

static const struct dpu_mdss_version sm8650_mdss_ver = {
	.core_major_ver = 10,
	.core_minor_ver = 0,
};

const struct dpu_mdss_cfg dpu_sm8650_cfg = {
	.mdss_ver = &sm8650_mdss_ver,
	.caps = &sm8650_dpu_caps,
	.mdp = &sm8650_mdp,
	.ctl_count = ARRAY_SIZE(sm8650_ctl),
	.ctl = sm8650_ctl,
	.sspp_count = ARRAY_SIZE(sm8650_sspp),
	.sspp = sm8650_sspp,
	.mixer_count = ARRAY_SIZE(sm8650_lm),
	.mixer = sm8650_lm,
	.dspp_count = ARRAY_SIZE(sm8650_dspp),
	.dspp = sm8650_dspp,
	.pingpong_count = ARRAY_SIZE(sm8650_pp),
	.pingpong = sm8650_pp,
	.dsc_count = ARRAY_SIZE(sm8650_dsc),
	.dsc = sm8650_dsc,
	.merge_3d_count = ARRAY_SIZE(sm8650_merge_3d),
	.merge_3d = sm8650_merge_3d,
	.wb_count = ARRAY_SIZE(sm8650_wb),
	.wb = sm8650_wb,
	.intf_count = ARRAY_SIZE(sm8650_intf),
	.intf = sm8650_intf,
	.vbif_count = ARRAY_SIZE(sm8650_vbif),
	.vbif = sm8650_vbif,
	.perf = &sm8650_perf_data,
};

#endif
