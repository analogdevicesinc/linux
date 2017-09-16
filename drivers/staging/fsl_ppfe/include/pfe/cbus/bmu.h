/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _BMU_H_
#define _BMU_H_

#define BMU_VERSION	0x000
#define BMU_CTRL	0x004
#define BMU_UCAST_CONFIG	0x008
#define BMU_UCAST_BASE_ADDR	0x00c
#define BMU_BUF_SIZE	0x010
#define BMU_BUF_CNT	0x014
#define BMU_THRES	0x018
#define BMU_INT_SRC	0x020
#define BMU_INT_ENABLE	0x024
#define BMU_ALLOC_CTRL	0x030
#define BMU_FREE_CTRL	0x034
#define BMU_FREE_ERR_ADDR	0x038
#define BMU_CURR_BUF_CNT	0x03c
#define BMU_MCAST_CNT	0x040
#define BMU_MCAST_ALLOC_CTRL	0x044
#define BMU_REM_BUF_CNT	0x048
#define BMU_LOW_WATERMARK	0x050
#define BMU_HIGH_WATERMARK	0x054
#define BMU_INT_MEM_ACCESS	0x100

struct BMU_CFG {
	unsigned long baseaddr;
	u32 count;
	u32 size;
	u32 low_watermark;
	u32 high_watermark;
};

#define BMU1_BUF_SIZE	LMEM_BUF_SIZE_LN2
#define BMU2_BUF_SIZE	DDR_BUF_SIZE_LN2

#define BMU2_MCAST_ALLOC_CTRL	(BMU2_BASE_ADDR + BMU_MCAST_ALLOC_CTRL)

#endif /* _BMU_H_ */
