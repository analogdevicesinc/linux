/*
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef _IMX8_PREFETCH_H_
#define _IMX8_PREFETCH_H_

#define PRG_HANDSHAKE_8LINES		8
#define PRG_HANDSHAKE_4LINES		4
#define AMPHION_STRIPE_WIDTH		8
#define AMPHION_STRIPE_HEIGHT		128
#define AMPHION_UV_STRIPE_HEIGHT	AMPHION_STRIPE_HEIGHT
#define AMPHION_Y_STRIPE_HEIGHT		(2 * AMPHION_STRIPE_HEIGHT)
#define VIVANTE_TILE_WIDTH		4
#define VIVANTE_TILE_HEIGHT		4
#define VIVANTE_SUPER_TILE_WIDTH	64
#define VIVANTE_SUPER_TILE_HEIGHT	64

struct prg;
struct prg *
prg_lookup_by_phandle(struct device *dev, const char *name, int index);
void prg_enable(struct prg *prg);
void prg_disable(struct prg *prg);
void prg_configure(struct prg *prg, unsigned int width, unsigned int height,
		   unsigned int x_offset, unsigned int y_offset,
		   unsigned int stride, unsigned int bits_per_pixel,
		   unsigned long baddr, u32 format, u64 modifier,
		   bool start);
void prg_reg_update(struct prg *prg);
void prg_shadow_enable(struct prg *prg);
bool prg_stride_supported(struct prg *prg, unsigned int stride);
bool prg_stride_double_check(struct prg *prg,
			     unsigned int stride, dma_addr_t baddr);
void prg_set_auxiliary(struct prg *prg);
void prg_set_primary(struct prg *prg);
void prg_set_blit(struct prg *prg);

struct dprc;
struct dprc *
dprc_lookup_by_phandle(struct device *dev, const char *name, int index);
void dprc_enable(struct dprc *dprc);
void dprc_disable(struct dprc *dprc);
void dprc_configure(struct dprc *dprc, unsigned int stream_id,
		    unsigned int width, unsigned int height,
		    unsigned int x_offset, unsigned int y_offset,
		    unsigned int stride, u32 format, u64 modifier,
		    unsigned long baddr, unsigned long uv_baddr,
		    bool start, bool aux_start, bool interlace_frame);
void dprc_reg_update(struct dprc *dprc);
void dprc_first_frame_handle(struct dprc *dprc);
void dprc_irq_handle(struct dprc *dprc);
void dprc_enable_ctrl_done_irq(struct dprc *dprc);
bool dprc_format_supported(struct dprc *dprc, u32 format, u64 modifier);
bool dprc_stride_supported(struct dprc *dprc,
			   unsigned int stride, unsigned int uv_stride,
			   unsigned int width, u32 format);
bool dprc_stride_double_check(struct dprc *dprc,
			      unsigned int stride, unsigned int uv_stride,
			      unsigned int width, u32 format,
			      dma_addr_t baddr, dma_addr_t uv_baddr);

#endif
