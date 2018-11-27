/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * DSP Audio DAI header
 *
 * Copyright 2018 NXP
 */

#ifndef __FSL_DSP_CPU_H
#define __FSL_DSP_CPU_H

#define ASRC_CLK_MAX_NUM 4

struct fsl_dsp_audio {
	struct platform_device *pdev;
	struct clk *bus_clk;
	struct clk *m_clk;
	struct clk *asrc_mem_clk;
	struct clk *asrc_ipg_clk;
	struct clk *asrck_clk[ASRC_CLK_MAX_NUM];
};

#endif /*__FSL_DSP_CPU_H*/

