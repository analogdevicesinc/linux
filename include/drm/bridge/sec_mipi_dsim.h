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

struct sec_mipi_dsim_plat_data {
	uint32_t version;
	uint32_t max_data_lanes;
	uint64_t max_data_rate;
	enum drm_mode_status (*mode_valid)(struct drm_connector *connector,
					   struct drm_display_mode *mode);
};

int sec_mipi_dsim_check_pll_out(void *driver_private,
				const struct drm_display_mode *mode);
int sec_mipi_dsim_bind(struct device *dev, struct device *master, void *data,
		       struct drm_encoder *encoder, struct resource *res,
		       int irq, const struct sec_mipi_dsim_plat_data *pdata);
void sec_mipi_dsim_unbind(struct device *dev, struct device *master, void *data);

void sec_mipi_dsim_suspend(struct device *dev);
void sec_mipi_dsim_resume(struct device *dev);

#endif
