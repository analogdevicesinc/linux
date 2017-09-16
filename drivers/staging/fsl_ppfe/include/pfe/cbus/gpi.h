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

#ifndef _GPI_H_
#define _GPI_H_

#define GPI_VERSION	0x00
#define GPI_CTRL	0x04
#define GPI_RX_CONFIG	0x08
#define GPI_HDR_SIZE	0x0c
#define GPI_BUF_SIZE	0x10
#define GPI_LMEM_ALLOC_ADDR	0x14
#define GPI_LMEM_FREE_ADDR	0x18
#define GPI_DDR_ALLOC_ADDR	0x1c
#define GPI_DDR_FREE_ADDR	0x20
#define GPI_CLASS_ADDR	0x24
#define GPI_DRX_FIFO	0x28
#define GPI_TRX_FIFO	0x2c
#define GPI_INQ_PKTPTR	0x30
#define GPI_DDR_DATA_OFFSET	0x34
#define GPI_LMEM_DATA_OFFSET	0x38
#define GPI_TMLF_TX	0x4c
#define GPI_DTX_ASEQ	0x50
#define GPI_FIFO_STATUS	0x54
#define GPI_FIFO_DEBUG	0x58
#define GPI_TX_PAUSE_TIME	0x5c
#define GPI_LMEM_SEC_BUF_DATA_OFFSET	0x60
#define GPI_DDR_SEC_BUF_DATA_OFFSET	0x64
#define GPI_TOE_CHKSUM_EN	0x68
#define GPI_OVERRUN_DROPCNT	0x6c
#define GPI_CSR_MTIP_PAUSE_REG		0x74
#define GPI_CSR_MTIP_PAUSE_QUANTUM	0x78
#define GPI_CSR_RX_CNT			0x7c
#define GPI_CSR_TX_CNT			0x80
#define GPI_CSR_DEBUG1			0x84
#define GPI_CSR_DEBUG2			0x88

struct gpi_cfg {
	u32 lmem_rtry_cnt;
	u32 tmlf_txthres;
	u32 aseq_len;
	u32 mtip_pause_reg;
};

/* GPI commons defines */
#define GPI_LMEM_BUF_EN	0x1
#define GPI_DDR_BUF_EN	0x1

/* EGPI 1 defines */
#define EGPI1_LMEM_RTRY_CNT	0x40
#define EGPI1_TMLF_TXTHRES	0xBC
#define EGPI1_ASEQ_LEN	0x50

/* EGPI 2 defines */
#define EGPI2_LMEM_RTRY_CNT	0x40
#define EGPI2_TMLF_TXTHRES	0xBC
#define EGPI2_ASEQ_LEN	0x40

/* EGPI 3 defines */
#define EGPI3_LMEM_RTRY_CNT	0x40
#define EGPI3_TMLF_TXTHRES	0xBC
#define EGPI3_ASEQ_LEN	0x40

/* HGPI defines */
#define HGPI_LMEM_RTRY_CNT	0x40
#define HGPI_TMLF_TXTHRES	0xBC
#define HGPI_ASEQ_LEN	0x40

#define EGPI_PAUSE_TIME		0x000007D0
#define EGPI_PAUSE_ENABLE	0x40000000
#endif /* _GPI_H_ */
