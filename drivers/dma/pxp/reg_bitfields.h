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
#endif
