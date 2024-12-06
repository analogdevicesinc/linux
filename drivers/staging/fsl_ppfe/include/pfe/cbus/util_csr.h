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

#ifndef _UTIL_CSR_H_
#define _UTIL_CSR_H_

#define UTIL_VERSION	(UTIL_CSR_BASE_ADDR + 0x000)
#define UTIL_TX_CTRL	(UTIL_CSR_BASE_ADDR + 0x004)
#define UTIL_INQ_PKTPTR	(UTIL_CSR_BASE_ADDR + 0x010)

#define UTIL_HDR_SIZE	(UTIL_CSR_BASE_ADDR + 0x014)

#define UTIL_PE0_QB_DM_ADDR0	(UTIL_CSR_BASE_ADDR + 0x020)
#define UTIL_PE0_QB_DM_ADDR1	(UTIL_CSR_BASE_ADDR + 0x024)
#define UTIL_PE0_RO_DM_ADDR0	(UTIL_CSR_BASE_ADDR + 0x060)
#define UTIL_PE0_RO_DM_ADDR1	(UTIL_CSR_BASE_ADDR + 0x064)

#define UTIL_MEM_ACCESS_ADDR	(UTIL_CSR_BASE_ADDR + 0x100)
#define UTIL_MEM_ACCESS_WDATA	(UTIL_CSR_BASE_ADDR + 0x104)
#define UTIL_MEM_ACCESS_RDATA	(UTIL_CSR_BASE_ADDR + 0x108)

#define UTIL_TM_INQ_ADDR	(UTIL_CSR_BASE_ADDR + 0x114)
#define UTIL_PE_STATUS	(UTIL_CSR_BASE_ADDR + 0x118)

#define UTIL_PE_SYS_CLK_RATIO	(UTIL_CSR_BASE_ADDR + 0x200)
#define UTIL_AFULL_THRES	(UTIL_CSR_BASE_ADDR + 0x204)
#define UTIL_GAP_BETWEEN_READS	(UTIL_CSR_BASE_ADDR + 0x208)
#define UTIL_MAX_BUF_CNT	(UTIL_CSR_BASE_ADDR + 0x20c)
#define UTIL_TSQ_FIFO_THRES	(UTIL_CSR_BASE_ADDR + 0x210)
#define UTIL_TSQ_MAX_CNT	(UTIL_CSR_BASE_ADDR + 0x214)
#define UTIL_IRAM_DATA_0	(UTIL_CSR_BASE_ADDR + 0x218)
#define UTIL_IRAM_DATA_1	(UTIL_CSR_BASE_ADDR + 0x21c)
#define UTIL_IRAM_DATA_2	(UTIL_CSR_BASE_ADDR + 0x220)
#define UTIL_IRAM_DATA_3	(UTIL_CSR_BASE_ADDR + 0x224)

#define UTIL_BUS_ACCESS_ADDR	(UTIL_CSR_BASE_ADDR + 0x228)
#define UTIL_BUS_ACCESS_WDATA	(UTIL_CSR_BASE_ADDR + 0x22c)
#define UTIL_BUS_ACCESS_RDATA	(UTIL_CSR_BASE_ADDR + 0x230)

#define UTIL_INQ_AFULL_THRES	(UTIL_CSR_BASE_ADDR + 0x234)

struct util_cfg {
	u32 pe_sys_clk_ratio;
};

#endif /* _UTIL_CSR_H_ */
