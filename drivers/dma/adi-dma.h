/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * SC598 DMA definitions
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __ASM_DMA_H__
#define __ASM_DMA_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/soc/adi/sc59x.h>
#include <asm-generic/dma.h>

/* DMA_CONFIG Masks */
#define DMAEN			0x00000001	/* DMA Channel Enable */
#define WNR			0x00000002	/* Channel Direction (W/R*) */

#define PSIZE_8			0x00000000	/* Transfer Word Size = 16 */
#define PSIZE_16		0x00000010	/* Transfer Word Size = 16 */
#define PSIZE_32		0x00000020	/* Transfer Word Size = 32 */
#define PSIZE_64		0x00000030	/* Transfer Word Size = 32 */

#define WDSIZE_8		0x00000000	/* Transfer Word Size = 8 */
#define WDSIZE_16		0x00000100	/* Transfer Word Size = 16 */
#define WDSIZE_32		0x00000200	/* Transfer Word Size = 32 */
#define WDSIZE_64		0x00000300	/* Transfer Word Size = 32 */
#define WDSIZE_128		0x00000400	/* Transfer Word Size = 32 */
#define WDSIZE_256		0x00000500	/* Transfer Word Size = 32 */

#define DMA2D			0x04000000	/* DMA Mode (2D/1D*) */
#define DMARESTART		0x00000004	/* DMA Buffer Clear SYNC */

#define DI_EN_X			0x00100000	/* Data Interrupt Enable in X count */
#define DI_EN_Y			0x00200000	/* Data Interrupt Enable in Y count */
#define DI_EN_P			0x00300000	/* Data Interrupt Enable in Peripheral */
#define DI_EN			DI_EN_X		/* Data Interrupt Enable */

#define NDSIZE_0		0x00000000	/* Next Descriptor Size = 1 */
#define NDSIZE_1		0x00010000	/* Next Descriptor Size = 2 */
#define NDSIZE_2		0x00020000	/* Next Descriptor Size = 3 */
#define NDSIZE_3		0x00030000	/* Next Descriptor Size = 4 */
#define NDSIZE_4		0x00040000	/* Next Descriptor Size = 5 */
#define NDSIZE_5		0x00050000	/* Next Descriptor Size = 6 */
#define NDSIZE_6		0x00060000	/* Next Descriptor Size = 7 */
#define NDSIZE			0x00070000	/* Next Descriptor Size */
#define NDSIZE_OFFSET		16		/* Next Descriptor Size Offset */

#define DMAFLOW_LIST		0x00004000	/* Descriptor List Mode */
#define DMAFLOW_LARGE		DMAFLOW_LIST
#define DMAFLOW_ARRAY		0x00005000	/* Descriptor Array Mode */
#define DMAFLOW_LIST_DEMAND	0x00006000	/* Descriptor Demand List Mode */
#define DMAFLOW_ARRAY_DEMAND	0x00007000	/* Descriptor Demand Array Mode */

#define DMA_RUN_DFETCH		0x00000100	/* DMA Running Fetch */
#define DMA_RUN			0x00000200	/* DMA Running Trans */
#define DMA_RUN_WAIT_TRIG	0x00000300	/* DMA Running WAIT TRIG */
#define DMA_RUN_WAIT_ACK	0x00000400	/* DMA Running WAIT ACK */
#define DMA_RUN_MASK		0x00000700	/* DMA Running Bits Mask */

#define DMAFLOW			0x000007000	/* Flow Control */
#define DMAFLOW_STOP		0x000000000	/* Stop Mode */
#define DMAFLOW_AUTO		0x000001000	/* Autobuffer Mode */

/* DMA_IRQ_STATUS Masks */
#define DMA_DONE		0x1	/* DMA Completion Interrupt Status */
#define DMA_ERR			0x2	/* DMA Error Interrupt Status */
#define DMA_PIRQ		0x4	/* DMA Peripheral Error Interrupt Status */

#define ADI_DMA_NEXT_DESC		0x00
#define ADI_DMA_ADDRSTART		0x04
#define ADI_DMA_CFG				0x08
#define ADI_DMA_XCNT			0x0c
#define ADI_DMA_XMOD			0x10
#define ADI_DMA_YCNT			0x14
#define ADI_DMA_YMOD			0x18
#define ADI_DMA_DSCPTR_CUR		0x24
#define ADI_DMA_DSCPTR_PRV		0x28
#define ADI_DMA_ADDR_CUR		0x2c
#define ADI_DMA_STAT			0x30
#define ADI_DMA_XCNT_CUR		0x34
#define ADI_DMA_YCNT_CUR		0x38
#define ADI_DMA_BWLCNT			0x40
#define ADI_DMA_BWLCNT_CUR		0x44
#define ADI_DMA_BWMCNT			0x48
#define ADI_DMA_BWMCNT_CUR		0x4c

/*******************************************************************************
 *	DMA API's
 *******************************************************************************/
static inline void set_dma_start_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	writel(lower_32_bits(addr), ioaddr + ADI_DMA_ADDRSTART);
}

static inline void set_dma_next_desc_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	writel(lower_32_bits(addr), ioaddr + ADI_DMA_NEXT_DESC);
}

static inline void set_dma_curr_desc_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	writel(lower_32_bits(addr), ioaddr + ADI_DMA_DSCPTR_CUR);
}

static inline void set_dma_x_count(void __iomem *ioaddr, unsigned long x_count)
{
	writel(x_count, ioaddr + ADI_DMA_XCNT);
}

static inline void set_dma_y_count(void __iomem *ioaddr, unsigned long y_count)
{
	writel(y_count, ioaddr + ADI_DMA_YCNT);
}

static inline void set_dma_x_modify(void __iomem *ioaddr, long x_modify)
{
	writel(x_modify, ioaddr + ADI_DMA_XMOD);
}

static inline void set_dma_y_modify(void __iomem *ioaddr, long y_modify)
{
	writel(y_modify, ioaddr + ADI_DMA_YMOD);
}

static inline void set_dma_config(void __iomem *ioaddr, unsigned long config)
{
	writel(config, ioaddr + ADI_DMA_CFG);
}

static inline void set_dma_curr_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	writel(lower_32_bits(addr), ioaddr + ADI_DMA_ADDR_CUR);
}

static inline unsigned long get_dma_curr_irqstat(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_STAT);
}

static inline unsigned long get_dma_curr_xcount(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_XCNT_CUR);
}

static inline unsigned long get_dma_curr_ycount(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_YCNT_CUR);
}

static inline dma_addr_t get_dma_next_desc_ptr(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_NEXT_DESC);
}

static inline dma_addr_t get_dma_curr_desc_ptr(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_DSCPTR_CUR);
}

static inline unsigned long get_dma_config(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_CFG);
}

static inline unsigned long get_dma_curr_addr(void __iomem *ioaddr)
{
	return readl(ioaddr + ADI_DMA_ADDR_CUR);
}

static inline void clear_dma_irqstat(void __iomem *ioaddr)
{
	writel(DMA_DONE | DMA_ERR | DMA_PIRQ, ioaddr + ADI_DMA_STAT);
}

#endif
