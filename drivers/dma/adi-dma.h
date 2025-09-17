// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ASM_DMA_H__
#define __ASM_DMA_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <asm-generic/dma.h>
#include <linux/io.h>

#define ADI_DMA_NEXT_DESC       0x00
#define ADI_DMA_ADDRSTART       0x04

#define ADI_DMA_CFG             0x08
#define   DMA2D                 BIT(26)         /* DMA Mode (2D/1D*) */
#define   DMA_INT_MSK           GENMASK(21, 20) /* Generate Interrupt Bits Mask */
#define   DI_EN_X               0x00100000      /* Data Interrupt Enable in X count */
#define   DI_EN_Y               0x00200000      /* Data Interrupt Enable in Y count */
#define   DI_EN_P               0x00300000      /* Data Interrupt Enable in Peripheral */
#define   DI_EN                 DI_EN_X         /* Data Interrupt Enable */
#define   NDSIZE                GENMASK(18, 16) /* Next Descriptor */
#define   NDSIZE_0              0x00000000      /* Next Descriptor Size = 1 */
#define   NDSIZE_1              0x00010000      /* Next Descriptor Size = 2 */
#define   NDSIZE_2              0x00020000      /* Next Descriptor Size = 3 */
#define   NDSIZE_3              0x00030000      /* Next Descriptor Size = 4 */
#define   NDSIZE_4              0x00040000      /* Next Descriptor Size = 5 */
#define   NDSIZE_5              0x00050000      /* Next Descriptor Size = 6 */
#define   NDSIZE_6              0x00060000      /* Next Descriptor Size = 7 */
#define   NDSIZE_OFFSET         16              /* Next Descriptor Size Offset */
#define   DMAFLOW               GENMASK(14, 12) /* Flow Control */
#define   DMAFLOW_STOP          0x00000000      /* Stop Mode */
#define   DMAFLOW_AUTO          0x00001000      /* Autobuffer Mode */
#define   DMAFLOW_LIST          0x00004000      /* Descriptor List Mode */
#define   DMAFLOW_LARGE         DMAFLOW_LIST
#define   DMAFLOW_ARRAY         0x00005000      /* Descriptor Array Mode */
#define   DMAFLOW_LIST_DEMAND   0x00006000      /* Descriptor Demand List Mode */
#define   DMAFLOW_ARRAY_DEMAND  0x00007000      /* Descriptor Demand Array Mode */
#define   WDSIZEE_MSK           GENMASK(10, 8)  /* Memory Transfer Word Size Mask */
#define   WDSIZE_8              0x00000000      /* Memory Transfer Word Size = 8 bits */
#define   WDSIZE_16             0x00000100      /* Memory Transfer Word Size = 16 bits */
#define   WDSIZE_32             0x00000200      /* Memory Transfer Word Size = 32 bits */
#define   WDSIZE_64             0x00000300      /* Memory Transfer Word Size = 64 bits */
#define   WDSIZE_128            0x00000400      /* Memory Transfer Word Size = 128 bits */
#define   WDSIZE_256            0x00000500      /* Memory Transfer Word Size = 256 bits */
#define   PSIZE_MSK             GENMASK(6, 4)   /* Peripheral Transfer Word Size Mask */
#define   PSIZE_8               0x00000000      /* Peripheral Transfer Word Size = 8 bits */
#define   PSIZE_16              0x00000010      /* Peripheral Transfer Word Size = 16 bits */
#define   PSIZE_32              0x00000020      /* Peripheral Transfer Word Size = 32 bits */
#define   PSIZE_64              0x00000030      /* Peripheral Transfer Word Size = 64 bits */
#define   DMASYNC               BIT(2)          /* DMA Buffer Clear SYNC */
#define   WNR                   BIT(1)          /* Channel Direction (W/R*) */
#define   DMAEN                 BIT(0)          /* DMA Channel Enable */

#define ADI_DMA_XCNT            0x0c
#define ADI_DMA_XMOD            0x10
#define ADI_DMA_YCNT            0x14
#define ADI_DMA_YMOD            0x18
#define ADI_DMA_DSCPTR_CUR      0x24
#define ADI_DMA_DSCPTR_PRV      0x28
#define ADI_DMA_ADDR_CUR        0x2c

#define ADI_DMA_STAT            0x30
#define   DMA_RUN_MASK          GENMASK(10, 8)  /* DMA Running Bits Mask */
#define   DMA_RUN_DFETCH        0x00000100      /* DMA Running Fetch */
#define   DMA_RUN               0x00000200      /* DMA Running Trans */
#define   DMA_RUN_WAIT_TRIG     0x00000300      /* DMA Running WAIT TRIG */
#define   DMA_RUN_WAIT_ACK      0x00000400      /* DMA Running WAIT ACK */
#define   DMA_PIRQ              BIT(2)          /* DMA Peripheral Error Interrupt Status */
#define   DMA_ERR               BIT(1)          /* DMA Error Interrupt Status */
#define   DMA_DONE              BIT(0)          /* DMA Completion Interrupt Status */

#define ADI_DMA_XCNT_CUR        0x34
#define ADI_DMA_YCNT_CUR        0x38
#define ADI_DMA_BWLCNT          0x40
#define ADI_DMA_BWLCNT_CUR      0x44
#define ADI_DMA_BWMCNT          0x48
#define ADI_DMA_BWMCNT_CUR      0x4c

#define DMA_DESC_FETCH          0x100           /* DMA is fetching descriptors */
#define DMA_DATA_XFER           0x200           /* DMA is in data transfer state */
#define DMA_IDLE_MASK           0x700
#define DMA_IDLE(x)             ((x & DMA_IDLE_MASK) == 0)
#define DMA_FETCHING_DESC(x)    (x & DMA_DESC_FETCH)
#define DMA_XFER_DATA(x)        (x & DMA_DATA_XFER)

#define START_DESC_LIST_XFR     (NDSIZE_4 | DMAFLOW_LIST)

/** DMA API's **/
static inline void set_dma_start_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	iowrite32(lower_32_bits(addr), ioaddr + ADI_DMA_ADDRSTART);
}
static inline void set_dma_next_desc_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	iowrite32(lower_32_bits(addr), ioaddr + ADI_DMA_NEXT_DESC);
}
static inline void set_dma_curr_desc_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	iowrite32(lower_32_bits(addr), ioaddr + ADI_DMA_DSCPTR_CUR);
}
static inline void set_dma_x_count(void __iomem *ioaddr, unsigned long x_count)
{
	iowrite32(x_count, ioaddr + ADI_DMA_XCNT);
}
static inline void set_dma_y_count(void __iomem *ioaddr, unsigned long y_count)
{
	iowrite32(y_count, ioaddr + ADI_DMA_YCNT);
}
static inline void set_dma_x_modify(void __iomem *ioaddr, long x_modify)
{
	iowrite32(x_modify, ioaddr + ADI_DMA_XMOD);
}
static inline void set_dma_y_modify(void __iomem *ioaddr, long y_modify)
{
	iowrite32(y_modify, ioaddr + ADI_DMA_YMOD);
}
static inline void set_dma_config(void __iomem *ioaddr, unsigned long config)
{
	iowrite32(config, ioaddr + ADI_DMA_CFG);
}
static inline void set_dma_curr_addr(void __iomem *ioaddr, dma_addr_t addr)
{
	iowrite32(lower_32_bits(addr), ioaddr + ADI_DMA_ADDR_CUR);
}

static inline unsigned long get_dma_curr_irqstat(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_STAT);
}
static inline unsigned long get_dma_curr_xcount(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_XCNT_CUR);
}
static inline unsigned long get_dma_curr_ycount(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_YCNT_CUR);
}
static inline dma_addr_t get_dma_next_desc_ptr(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_NEXT_DESC);
}
static inline dma_addr_t get_dma_curr_desc_ptr(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_DSCPTR_CUR);
}
static inline unsigned long get_dma_config(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_CFG);
}
static inline unsigned long get_dma_curr_addr(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_ADDR_CUR);
}
static inline unsigned long get_dma_start_addr(void __iomem *ioaddr)
{
	return ioread32(ioaddr + ADI_DMA_ADDRSTART);
}
static inline void clear_dma_irqstat(void __iomem *ioaddr)
{
	iowrite32(DMA_DONE | DMA_ERR | DMA_PIRQ, ioaddr + ADI_DMA_STAT);
}
#endif
