/*
 * Copyright (C) 2010-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _REG_BITFIELDS_H
#define _REG_BITFIELDS_H
struct mux_config {
	uint32_t mux0_sel  		: 2;
	uint32_t mux1_sel  		: 2;
	uint32_t mux2_sel  		: 2;
	uint32_t mux3_sel  		: 2;
	uint32_t mux4_sel  		: 2;
	uint32_t mux5_sel  		: 2;
	uint32_t mux6_sel  		: 2;
	uint32_t mux7_sel  		: 2;
	uint32_t mux8_sel  		: 2;
	uint32_t mux9_sel  		: 2;
	uint32_t mux10_sel 		: 2;
	uint32_t mux11_sel 		: 2;
	uint32_t mux12_sel 		: 2;
	uint32_t mux13_sel 		: 2;
	uint32_t mux14_sel 		: 2;
	uint32_t mux15_sel 		: 2;
};

/* legacy engine registers */
struct ps_ctrl {
	uint32_t format			: 6;
	uint32_t wb_swap		: 1;
	uint32_t rsvd0			: 1;
	uint32_t decy			: 2;
	uint32_t decx			: 2;
	uint32_t rsvd1			: 20;
};

struct ps_scale {
	uint32_t xscale			: 15;
	uint32_t rsvd1			: 1;
	uint32_t yscale			: 15;
	uint32_t rsvd2			: 1;
};

struct ps_offset {
	uint32_t xoffset		: 12;
	uint32_t rsvd1			: 4;
	uint32_t yoffset		: 12;
	uint32_t rsvd2			: 4;
};

struct as_ctrl {
	uint32_t rsvd0			: 1;
	uint32_t alpha_ctrl		: 2;
	uint32_t enable_colorkey	: 1;
	uint32_t format			: 4;
	uint32_t alpha			: 8;
	uint32_t rop			: 4;
	uint32_t alpha0_invert		: 1;
	uint32_t alpha1_invert		: 1;
	uint32_t rsvd1			: 10;
};

struct out_ctrl {
	uint32_t format			: 5;
	uint32_t rsvd0			: 3;
	uint32_t interlaced_output	: 2;
	uint32_t rsvd1			: 13;
	uint32_t alpha_output		: 1;
	uint32_t alpha			: 8;
};

struct coordinate {
	uint32_t y			: 14;
	uint32_t rsvd0			: 2;
	uint32_t x			: 14;
	uint32_t rsvd1			: 2;
};

struct pxp_alpha_ctrl {
	uint32_t poter_duff_enable	: 1;
	uint32_t s0_s1_factor_mode	: 2;
	uint32_t s0_global_alpha_mode	: 2;
	uint32_t s0_alpha_mode		: 1;
	uint32_t s0_color_mode		: 1;
	uint32_t rsvd1			: 1;
	uint32_t s1_s0_factor_mode	: 2;
	uint32_t s1_global_alpha_mode	: 2;
	uint32_t s1_alpha_mode		: 1;
	uint32_t s1_color_mode		: 1;
	uint32_t rsvd0			: 2;
	uint32_t s0_global_alpha	: 8;
	uint32_t s1_global_alpha	: 8;
};

/* store engine registers */
struct store_ctrl {
	uint32_t ch_en			: 1;
	uint32_t block_en		: 1;
	uint32_t block_16		: 1;
	uint32_t handshake_en		: 1;
	uint32_t array_en		: 1;
	uint32_t array_line_num 	: 2;
	uint32_t rsvd3			: 1;
	uint32_t store_bypass_en	: 1;
	uint32_t store_memory_en	: 1;
	uint32_t pack_in_sel		: 1;
	uint32_t fill_data_en		: 1;
	uint32_t rsvd2			: 4;
	uint32_t wr_num_bytes		: 2;
	uint32_t rsvd1			: 6;
	uint32_t combine_2channel	: 1;
	uint32_t rsvd0			: 6;
	uint32_t arbit_en		: 1;
};

struct store_size {
	uint32_t out_width		: 16;
	uint32_t out_height		: 16;
};

struct store_pitch {
	uint32_t ch0_out_pitch		: 16;
	uint32_t ch1_out_pitch		: 16;
};

struct store_shift_ctrl {
	uint32_t rsvd2			: 2;
	uint32_t output_active_bpp	: 2;
	uint32_t out_yuv422_1p_en	: 1;
	uint32_t out_yuv422_2p_en	: 1;
	uint32_t rsvd1			: 1;
	uint32_t shift_bypass		: 1;
	uint32_t rsvd0			: 24;
};

struct store_d_shift {
	uint64_t d_shift_width0		: 6;
	uint64_t rsvd3			: 1;
	uint64_t d_shift_flag0		: 1;
	uint64_t d_shift_width1		: 6;
	uint64_t rsvd2			: 1;
	uint64_t d_shift_flag1		: 1;
	uint64_t d_shift_width2		: 6;
	uint64_t rsvd1			: 1;
	uint64_t d_shift_flag2		: 1;
	uint64_t d_shift_width3		: 6;
	uint64_t rsvd0			: 1;
	uint64_t d_shift_flag3		: 1;

	uint64_t d_shift_width4		: 6;
	uint64_t rsvd7			: 1;
	uint64_t d_shift_flag4		: 1;
	uint64_t d_shift_width5		: 6;
	uint64_t rsvd6			: 1;
	uint64_t d_shift_flag5		: 1;
	uint64_t d_shift_width6		: 6;
	uint64_t rsvd5			: 1;
	uint64_t d_shift_flag6		: 1;
	uint64_t d_shift_width7		: 6;
	uint64_t rsvd4			: 1;
	uint64_t d_shift_flag7		: 1;
};

struct store_f_shift {
	uint64_t f_shift_width0		: 6;
	uint64_t rsvd3			: 1;
	uint64_t f_shift_flag0		: 1;
	uint64_t f_shift_width1		: 6;
	uint64_t rsvd2			: 1;
	uint64_t f_shift_flag1		: 1;
	uint64_t f_shift_width2		: 6;
	uint64_t rsvd1			: 1;
	uint64_t f_shift_flag2		: 1;
	uint64_t f_shift_width3		: 6;
	uint64_t rsvd0			: 1;
	uint64_t f_shift_flag3		: 1;

	uint64_t f_shift_width4		: 6;
	uint64_t rsvd7			: 1;
	uint64_t f_shift_flag4		: 1;
	uint64_t f_shift_width5		: 6;
	uint64_t rsvd6			: 1;
	uint64_t f_shift_flag5		: 1;
	uint64_t f_shift_width6		: 6;
	uint64_t rsvd5			: 1;
	uint64_t f_shift_flag6		: 1;
	uint64_t f_shift_width7		: 6;
	uint64_t rsvd4			: 1;
	uint64_t f_shift_flag7		: 1;
};

struct store_d_mask {
	uint64_t d_mask_l		: 32;
	uint64_t d_mask_h		: 32;
};

/* fetch engine registers */
struct fetch_ctrl {
	uint32_t ch_en			: 1;
	uint32_t block_en		: 1;
	uint32_t block_16		: 1;
	uint32_t handshake_en		: 1;
	uint32_t bypass_pixel_en	: 1;
	uint32_t high_byte		: 1;
	uint32_t rsvd4			: 3;
	uint32_t hflip			: 1;
	uint32_t vflip			: 1;
	uint32_t rsvd3			: 1;
	uint32_t rotation_angle		: 2;
	uint32_t rsvd2			: 2;
	uint32_t rd_num_bytes		: 2;
	uint32_t rsvd1			: 6;
	uint32_t handshake_scan_line_num : 2;
	uint32_t rsvd0			: 5;
	uint32_t arbit_en		: 1;
};

struct fetch_active_size_ulc {
	uint32_t active_size_ulc_x	: 16;
	uint32_t active_size_ulc_y	: 16;
};

struct fetch_active_size_lrc {
	uint32_t active_size_lrc_x	: 16;
	uint32_t active_size_lrc_y	: 16;
};

struct fetch_size {
	uint32_t input_total_width	: 16;
	uint32_t input_total_height	: 16;
};

struct fetch_pitch {
	uint32_t ch0_input_pitch	: 16;
	uint32_t ch1_input_pitch	: 16;
};

struct fetch_shift_ctrl {
	uint32_t input_active_bpp	: 2;
	uint32_t rsvd1			: 6;
	uint32_t expand_format		: 3;
	uint32_t expand_en		: 1;
	uint32_t shift_bypass		: 1;
	uint32_t rsvd0			: 19;
};

struct fetch_shift_offset {
	uint32_t offset0		: 5;
	uint32_t rsvd3			: 3;
	uint32_t offset1		: 5;
	uint32_t rsvd2			: 3;
	uint32_t offset2		: 5;
	uint32_t rsvd1			: 3;
	uint32_t offset3		: 5;
	uint32_t rsvd0			: 3;
};

struct fetch_shift_width {
	uint32_t width0			: 4;
	uint32_t width1			: 4;
	uint32_t width2			: 4;
	uint32_t width3			: 4;
	uint32_t rsvd0			: 16;
};
#endif
