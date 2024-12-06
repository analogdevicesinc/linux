/*
 * Copyright 2018,2021 NXP
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

#ifndef __IMX_LCDIF_H__
#define __IMX_LCDIF_H__

struct lcdif_soc;
struct videomode;

struct lcdif_client_platformdata {
	struct device_node *of_node;
};

int  lcdif_vblank_irq_get(struct lcdif_soc *lcdif);
void lcdif_vblank_irq_enable(struct lcdif_soc *lcdif);
void lcdif_vblank_irq_disable(struct lcdif_soc *lcdif);
void lcdif_vblank_irq_clear(struct lcdif_soc *lcdif);

int  lcdif_get_bus_fmt_from_pix_fmt(struct lcdif_soc *lcdif,
				    uint32_t format);
int  lcdif_set_pix_fmt(struct lcdif_soc *lcdif, u32 format);
void lcdif_set_bus_fmt(struct lcdif_soc *lcdif, u32 bus_format);
void lcdif_set_fb_addr(struct lcdif_soc *lcdif, int id, u32 addr, bool use_i80);
void lcdif_set_mode(struct lcdif_soc *lcdif, struct videomode *vmode,
		    bool use_i80);
void lcdif_set_fb_hcrop(struct lcdif_soc *lcdif, u32 src_w,
			u32 fb_w, bool crop);
void lcdif_enable_controller(struct lcdif_soc *lcdif, bool use_i80);
void lcdif_disable_controller(struct lcdif_soc *lcdif, bool use_i80);
void lcdif_dump_registers(struct lcdif_soc *lcdif);
long lcdif_pix_clk_round_rate(struct lcdif_soc *lcdif,
			      unsigned long rate);

#endif
