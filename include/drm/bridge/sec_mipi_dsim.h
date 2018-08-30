/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SEC_MIPI_DSIM_H__
#define __SEC_MIPI_DSIM_H__

#include <drm/drmP.h>
#include <linux/bsearch.h>

struct sec_mipi_dsim_dphy_timing;

struct sec_mipi_dsim_plat_data {
	uint32_t version;
	uint32_t max_data_lanes;
	uint64_t max_data_rate;
	const struct sec_mipi_dsim_dphy_timing *dphy_timing;
	uint32_t num_dphy_timing;
	int (*dphy_timing_cmp)(const void *key, const void *elt);
	enum drm_mode_status (*mode_valid)(struct drm_connector *connector,
					   struct drm_display_mode *mode);
};

/* DPHY timings structure */
struct sec_mipi_dsim_dphy_timing {
	uint32_t bit_clk;	/* MHz */

	uint32_t clk_prepare;
	uint32_t clk_zero;
	uint32_t clk_post;
	uint32_t clk_trail;

	uint32_t hs_prepare;
	uint32_t hs_zero;
	uint32_t hs_trail;

	uint32_t lpx;
	uint32_t hs_exit;
};

#define DSIM_DPHY_TIMING(bclk, cpre, czero, cpost, ctrail,	\
			 hpre, hzero, htrail, lp, hexit)	\
	.bit_clk	= bclk,					\
	.clk_prepare	= cpre,					\
	.clk_zero	= czero,				\
	.clk_post	= cpost,				\
	.clk_trail	= ctrail,				\
	.hs_prepare	= hpre,					\
	.hs_zero	= hzero,				\
	.hs_trail	= htrail,				\
	.lpx		= lp,					\
	.hs_exit	= hexit

static inline int dphy_timing_default_cmp(const void *key, const void *elt)
{
	const struct sec_mipi_dsim_dphy_timing *_key = key;
	const struct sec_mipi_dsim_dphy_timing *_elt = elt;

	/* find an element whose 'bit_clk' is equal to the
	 * the key's 'bit_clk' value or, the difference
	 * between them is less than 5.
	 */
	if (abs((int)(_elt->bit_clk - _key->bit_clk)) <= 5)
		return 0;

	if (_key->bit_clk < _elt->bit_clk)
		/* search bottom half */
		return 1;
	else
		/* search top half */
		return -1;
}

int sec_mipi_dsim_check_pll_out(void *driver_private,
				const struct drm_display_mode *mode);
int sec_mipi_dsim_bind(struct device *dev, struct device *master, void *data,
		       struct drm_encoder *encoder, struct resource *res,
		       int irq, const struct sec_mipi_dsim_plat_data *pdata);
void sec_mipi_dsim_unbind(struct device *dev, struct device *master, void *data);

void sec_mipi_dsim_suspend(struct device *dev);
void sec_mipi_dsim_resume(struct device *dev);

#endif
