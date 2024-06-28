/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved
 */

#ifndef _DPU_HW_WB_H
#define _DPU_HW_WB_H

#include "dpu_hw_catalog.h"
#include "dpu_hw_mdss.h"
#include "dpu_hw_top.h"
#include "dpu_hw_util.h"
#include "dpu_hw_pingpong.h"

struct dpu_hw_wb;

struct dpu_hw_wb_cfg {
	struct dpu_hw_fmt_layout dest;
	enum dpu_intf_mode intf_mode;
	struct drm_rect roi;
	struct drm_rect crop;
};

/**
 *
 * struct dpu_hw_wb_ops : Interface to the wb hw driver functions
 *  Assumption is these functions will be called after clocks are enabled
 *  @setup_outaddress: setup output address from the writeback job
 *  @setup_outformat: setup output format of writeback block from writeback job
 *  @setup_qos_lut:   setup qos LUT for writeback block based on input
 *  @setup_cdp:       setup chroma down prefetch block for writeback block
 *  @setup_clk_force_ctrl: setup clock force control
 *  @bind_pingpong_blk: enable/disable the connection with ping-pong block
 */
struct dpu_hw_wb_ops {
	void (*setup_outaddress)(struct dpu_hw_wb *ctx,
			struct dpu_hw_wb_cfg *wb);

	void (*setup_outformat)(struct dpu_hw_wb *ctx,
			struct dpu_hw_wb_cfg *wb);

	void (*setup_roi)(struct dpu_hw_wb *ctx,
			struct dpu_hw_wb_cfg *wb);

	void (*setup_qos_lut)(struct dpu_hw_wb *ctx,
			struct dpu_hw_qos_cfg *cfg);

	void (*setup_cdp)(struct dpu_hw_wb *ctx,
			  const struct msm_format *fmt,
			  bool enable);

	bool (*setup_clk_force_ctrl)(struct dpu_hw_wb *ctx,
				     bool enable);

	void (*bind_pingpong_blk)(struct dpu_hw_wb *ctx,
				  const enum dpu_pingpong pp);
};

/**
 * struct dpu_hw_wb : WB driver object
 * @hw: block hardware details
 * @idx: hardware index number within type
 * @wb_hw_caps: hardware capabilities
 * @ops: function pointers
 */
struct dpu_hw_wb {
	struct dpu_hw_blk_reg_map hw;

	/* wb path */
	int idx;
	const struct dpu_wb_cfg *caps;

	/* ops */
	struct dpu_hw_wb_ops ops;
};

/**
 * dpu_hw_wb_init() - Initializes the writeback hw driver object.
 * @dev:  Corresponding device for devres management
 * @cfg:  wb_path catalog entry for which driver object is required
 * @addr: mapped register io address of MDP
 * @mdss_rev: dpu core's major and minor versions
 * Return: Error code or allocated dpu_hw_wb context
 */
struct dpu_hw_wb *dpu_hw_wb_init(struct drm_device *dev,
				 const struct dpu_wb_cfg *cfg,
				 void __iomem *addr,
				 const struct dpu_mdss_version *mdss_rev);

#endif /*_DPU_HW_WB_H */
