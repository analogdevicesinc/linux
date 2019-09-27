/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */

#ifndef __MXC_MIPI_CSI2_H__
#define __MXC_MIPI_CSI2_H__

#ifdef DEBUG
#define mipi_dbg(fmt, ...)	\
	printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define mipi_dbg(fmt, ...)
#endif

/* driver private data */
struct mipi_csi2_info {
	bool		mipi_en;
	int		ipu_id;
	unsigned int	csi_id;
	unsigned int	v_channel;
	unsigned int	lanes;
	unsigned int	datatype;
	struct clk	*cfg_clk;
	struct clk	*dphy_clk;
	struct clk	*pixel_clk;
	void __iomem	*mipi_csi2_base;
	struct platform_device	*pdev;

	struct mutex mutex_lock;
};

#endif
