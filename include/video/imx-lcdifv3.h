/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef __IMX_LCDIFV3_H__
#define __IMX_LCDIFV3_H__

struct lcdifv3_soc;
struct videomode;

struct lcdifv3_client_platformdata {
	struct device_node *of_node;
};

int  lcdifv3_vblank_irq_get(struct lcdifv3_soc *lcdifv3);
void lcdifv3_vblank_irq_enable(struct lcdifv3_soc *lcdifv3);
void lcdifv3_vblank_irq_disable(struct lcdifv3_soc *lcdifv3);
void lcdifv3_vblank_irq_clear(struct lcdifv3_soc *lcdifv3);

int  lcdifv3_get_bus_fmt_from_pix_fmt(struct lcdifv3_soc *lcdifv3,
				    uint32_t format);
int  lcdifv3_set_pix_fmt(struct lcdifv3_soc *lcdifv3, u32 format);
void lcdifv3_set_bus_fmt(struct lcdifv3_soc *lcdifv3, u32 bus_format);
void lcdifv3_set_fb_addr(struct lcdifv3_soc *lcdifv3, int id, u32 addr);
void lcdifv3_set_mode(struct lcdifv3_soc *lcdifv3, struct videomode *vmode);
void lcdifv3_set_fb_hcrop(struct lcdifv3_soc *lcdifv3, u32 src_w,
			u32 fb_w, bool crop);
void lcdifv3_en_shadow_load(struct lcdifv3_soc *lcdifv3);
void lcdifv3_enable_controller(struct lcdifv3_soc *lcdifv3);
void lcdifv3_disable_controller(struct lcdifv3_soc *lcdifv3);
void lcdifv3_dump_registers(struct lcdifv3_soc *lcdifv3);
long lcdifv3_pix_clk_round_rate(struct lcdifv3_soc *lcdifv3,
				unsigned long rate);

#endif
