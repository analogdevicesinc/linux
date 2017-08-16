/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 */

#ifndef __MACH_S32V234_CLK_H
#define __MACH_S32V234_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include "mc_cgm.h"
#include "mc_me.h"
#include "pll.h"
#include "src.h"
#include "dfs.h"
#include "../clk.h"

struct clk *s32v234_clk_plldig(enum s32v234_plldig_type type, const char *name,
			       const char *parent_name, void __iomem *base,
			       u32 plldv_mfd, u32 plldv_mfn,
			       u32 plldv_rfdphi, u32 plldv_rfdphi1);

struct clk *s32v234_clk_plldig_phi(enum s32v234_plldig_type type,
				   const char *name, const char *parent,
				   void __iomem *base, u32 phi);

struct clk *s32v234_clk_dfs(enum s32v234_plldig_type type, const char *name,
			    const char *parent_name,
			    void __iomem *reg, u8 idx, u32 mfn);
#endif
