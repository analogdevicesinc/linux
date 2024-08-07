// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/if_ether.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/spinlock.h>
#include <linux/bitfield.h>
#include "adrv906x-ndma.h"

#define NDMA_TX_STAT_AND_CTRL                      0x000
#define   NDMA_DATAPATH_EN                         BIT(0)
#define NDMA_TX_EVENT_EN                           0x004
#define NDMA_TX_EVENT_STAT                         0x008
#define   NDMA_TX_STATUS_FIFO_FULL_EVENT           BIT(4)
#define   NDMA_TX_FRAME_SIZE_ERR_EVENT             BIT(3)
#define   NDMA_TX_STATUS_WRITE_COMPLETE_EVENT      BIT(2)
#define   NDMA_TX_WORKUNIT_COMPLETE_EVENT          BIT(1)
#define   NDMA_TX_WU_HEADER_ERR_EVENT              BIT(0)
#define NDMA_TX_TIMEOUT_VALUE                      0x00c
#define NDMA_TX_FRAME_SIZE                         0x010
#define   NDMA_TX_MIN_FRAME_SIZE                   GENMASK(15, 0)
#define   NDMA_TX_MAX_FRAME_SIZE                   GENMASK(31, 16)

#define NDMA_TX_ERROR_EVENTS    (NDMA_TX_FRAME_SIZE_ERR_EVENT | \
				 NDMA_TX_WU_HEADER_ERR_EVENT)
#define NDMA_TX_STATUS_EVENTS   (NDMA_TX_STATUS_FIFO_FULL_EVENT | \
				 NDMA_TX_STATUS_WRITE_COMPLETE_EVENT | \
				 NDMA_TX_WORKUNIT_COMPLETE_EVENT)

#define NDMA_RX_STAT_AND_CTRL                      0x000
#define   NDMA_RX_DATAPATH_EN                      BIT(0)
#define NDMA_RX_EVENT_EN                           0x004
#define NDMA_RX_EVENT_STAT                         0x008
#define   NDMA_RX_FRAME_SIZE_ERR_EVENT             BIT(4)
#define   NDMA_RX_ERR_EVENT                        BIT(3)
#define   NDMA_RX_STATUS_WR_EVENT                  BIT(2)
#define   NDMA_RX_WORKUNIT_COMPLETE_EVENT          BIT(1)
#define   NDMA_RX_FRAME_DROPPED_ERR_EVENT          BIT(0)
#define NDMA_RX_FRAME_DROPPED_COUNT_MPLANE         0x00c
#define NDMA_RX_FRAME_DROPPED_COUNT_SPLANE         0x010
#define NDMA_RX_FRAME_SIZE                         0x014
#define   NDMA_RX_MIN_FRAME_SIZE                   GENMASK(15, 0)
#define   NDMA_RX_MAX_FRAME_SIZE                   GENMASK(31, 16)
#define NDMA_RX_SYNC_FIFO_MPLANE_THRESHOLD         0x018
#define NDMA_RX_IPV4_FRAME_FIELD_VALUE0            0x100
#define NDMA_RX_IPV4_FRAME_FIELD_VALUE1            0x104
#define NDMA_RX_IPV4_FRAME_FIELD_OFFSET0           0x108
#define NDMA_RX_IPV4_FRAME_FIELD_OFFSET1           0x10c
#define NDMA_RX_IPV4_FRAME_FIELD_MASK0             0x110
#define NDMA_RX_IPV4_FRAME_FIELD_MASK1             0x114
#define NDMA_RX_IPV6_FRAME_FIELD_VALUE0            0x200
#define NDMA_RX_IPV6_FRAME_FIELD_VALUE1            0x204
#define NDMA_RX_IPV6_FRAME_FIELD_OFFSET0           0x208
#define NDMA_RX_IPV6_FRAME_FIELD_OFFSET1           0x20c
#define NDMA_RX_IPV6_FRAME_FIELD_MASK0             0x210
#define NDMA_RX_IPV6_FRAME_FIELD_MASK1             0x214
#define NDMA_RX_ETH_FRAME_FIELD_VALUE              0x300
#define NDMA_RX_ETH_FRAME_FIELD_OFFSET             0x304
#define NDMA_RX_ETH_FRAME_FIELD_MASK               0x308
#define NDMA_RX_SPLANE_FILTER_PTP_MSG_VALUE        0x400
#define NDMA_RX_SPLANE_FILTER_VLAN_TAG_VALUE       0x404
#define NDMA_RX_SPLANE_FILTER_VLAN_TAG_OFFSET      0x408
#define NDMA_RX_SPLANE_FILTER_VLAN_TAG_MASK        0x40c
#define NDMA_RX_SPLANE_FILTER_VLAN_FRAME_OFFSET    0x410
#define NDMA_RX_SPLANE_FILTER_EN                   0x414
#define NDMA_RX_GEN_FILTER_REG_STRIDE              0x100
#define NDMA_RX_GEN_FILTER0_REG_OFFSET             0x500
#define NDMA_RX_GEN_FILTER1_REG_OFFSET             0x600
#define NDMA_RX_GEN_FILTER2_REG_OFFSET             0x700
#define NDMA_RX_GEN_FILTER3_REG_OFFSET             0x800
#define NDMA_RX_GEN_FILTER4_REG_OFFSET             0x900

#define NDMA_RX_CYCLE0_LOWER_VALUE_REG_OFFSET      0x00
#define NDMA_RX_CYCLE0_LOWER_MASK_REG_OFFSET       0x60

#define NDMA_RX_ERROR_EVENTS    (NDMA_RX_FRAME_SIZE_ERR_EVENT | \
				 NDMA_RX_ERR_EVENT | \
				 NDMA_RX_FRAME_DROPPED_ERR_EVENT)
#define NDMA_RX_STATUS_EVENTS   (NDMA_RX_STATUS_WR_EVENT | \
				 NDMA_RX_WORKUNIT_COMPLETE_EVENT)

#define NDMA_INTR_CTRL_TX                          0x00
#define   NDMA_INTR_CTRL_TX_DMA_ERR_EN             BIT(4)
#define   NDMA_INTR_CTRL_TX_DMA_DMADONE_EN         BIT(3)
#define   NDMA_INTR_CTRL_TX_DMA_DONE_EN            BIT(2)
#define   NDMA_INTR_CTRL_TX_ERR_EN                 BIT(0)
#define NDMA_INTR_CTRL_STATUS                      0x10
#define   NDMA_INTR_CTRL_TX_STATUS_DMA_ERR_EN      BIT(4)
#define   NDMA_INTR_CTRL_TX_STATUS_DMA_DMADONE_EN  BIT(3)
#define   NDMA_INTR_CTRL_TX_STATUS_DMA_DONE_EN     BIT(2)
#define   NDMA_INTR_CTRL_TX_STATUS_EN              BIT(1)
#define   NDMA_INTR_CTRL_RX_STATUS_EN              BIT(0)
#define NDMA_INTR_CTRL_RX                          0x20
#define   NDMA_INTR_CTRL_RX_DMA_ERR_EN             BIT(4)
#define   NDMA_INTR_CTRL_RX_DMA_DMADONE_EN         BIT(3)
#define   NDMA_INTR_CTRL_RX_DMA_DONE_EN            BIT(2)
#define   NDMA_INTR_CTRL_RX_ERR_EN                 BIT(0)

#define NDMA_RESET                                 0x00
#define   NDMA_RX0_RST                             BIT(0)
#define   NDMA_RX1_RST                             BIT(1)
#define   NDMA_TX0_RST                             BIT(2)
#define   NDMA_TX1_RST                             BIT(3)
#define   NDMA_TX0_PTP_MODE                        BIT(4)
#define   NDMA_TX1_PTP_MODE                        BIT(5)

#define DMA_NEXT_DESC           0x00
#define DMA_ADDRSTART           0x04

#define DMA_CFG                 0x08
#define   DMA2D                 BIT(26)                 /* DMA Mode (2D/1D*) */
#define   DESCIDCPY             BIT(25)                 /* Descriptor ID Copy Control */
#define   DMA_INT_MSK           GENMASK(21, 20)         /* Generate Interrupt Bits Mask */
#define   DI_EN_X               0x00100000              /* Data Interrupt Enable in X count */
#define   DI_EN_Y               0x00200000              /* Data Interrupt Enable in Y count */
#define   DI_EN_P               0x00300000              /* Data Interrupt Enable in Peripheral */
#define   DI_EN                 DI_EN_X                 /* Data Interrupt Enable */
#define   NDSIZE                GENMASK(18, 16)         /* Next Descriptor */
#define   NDSIZE_0              0x00000000              /* Next Descriptor Size = 1 */
#define   NDSIZE_1              0x00010000              /* Next Descriptor Size = 2 */
#define   NDSIZE_2              0x00020000              /* Next Descriptor Size = 3 */
#define   NDSIZE_3              0x00030000              /* Next Descriptor Size = 4 */
#define   NDSIZE_4              0x00040000              /* Next Descriptor Size = 5 */
#define   NDSIZE_5              0x00050000              /* Next Descriptor Size = 6 */
#define   NDSIZE_6              0x00060000              /* Next Descriptor Size = 7 */
#define   NDSIZE_OFFSET         16                      /* Next Descriptor Size Offset */
#define   DMAFLOW               GENMASK(14, 12)         /* Flow Control */
#define   DMAFLOW_STOP          0x00000000              /* Stop Mode */
#define   DMAFLOW_AUTO          0x00001000              /* Autobuffer Mode */
#define   DMAFLOW_LIST          0x00004000              /* Descriptor List Mode */
#define   DMAFLOW_LARGE         DMAFLOW_LIST
#define   DMAFLOW_ARRAY         0x00005000              /* Descriptor Array Mode */
#define   DMAFLOW_LIST_DEMAND   0x00006000              /* Descriptor Demand List Mode */
#define   DMAFLOW_ARRAY_DEMAND  0x00007000              /* Descriptor Demand Array Mode */
#define   WDSIZE_MSK            GENMASK(10, 8)          /* Memory Transfer Word Size Mask */
#define   WDSIZE_8              0x00000000              /* Memory Transfer Word Size = 8 bits */
#define   WDSIZE_16             0x00000100              /* Memory Transfer Word Size = 16 bits */
#define   WDSIZE_32             0x00000200              /* Memory Transfer Word Size = 32 bits */
#define   WDSIZE_64             0x00000300              /* Memory Transfer Word Size = 64 bits */
#define   WDSIZE_128            0x00000400              /* Memory Transfer Word Size = 128 bits */
#define   WDSIZE_256            0x00000500              /* Memory Transfer Word Size = 256 bits */
#define   PSIZE_MSK             GENMASK(6, 4)           /* Peripheral Transfer Word Size Mask */
#define   PSIZE_8               0x00000000              /* Peripheral Transfer Word Size = 8 bits */
#define   PSIZE_16              0x00000010              /* Peripheral Transfer Word Size = 16 bits */
#define   PSIZE_32              0x00000020              /* Peripheral Transfer Word Size = 32 bits */
#define   PSIZE_64              0x00000030              /* Peripheral Transfer Word Size = 64 bits */
#define   DMASYNC               BIT(2)                  /* DMA Buffer Clear SYNC */
#define   WNR                   BIT(1)                  /* Channel Direction (W/R*) */
#define   DMAEN                 BIT(0)                  /* DMA Channel Enable */
#define DMA_XCNT                0x0c
#define DMA_XMOD                0x10
#define    XMODE_8              0x01
#define    XMODE_16             0x02
#define    XMODE_32             0x04
#define    XMODE_64             0x08
#define    XMODE_182            0x10
#define    XMODE_256            0x20
#define DMA_YCNT                0x14
#define DMA_YMOD                0x18
#define DSCPTR_CUR              0x24
#define DSCPTR_PRV              0x28
#define DMA_ADDR_CUR            0x2c
#define DMA_STAT                0x30
#define   DMA_RUN_MASK          GENMASK(10, 8)          /* DMA Running Bits Mask */
#define   DMA_RUN_DFETCH        0x00000100              /* DMA Running Fetch */
#define   DMA_RUN               0x00000200              /* DMA Running Trans */
#define   DMA_RUN_WAIT_TRIG     0x00000300              /* DMA Running WAIT TRIG */
#define   DMA_RUN_WAIT_ACK      0x00000400              /* DMA Running WAIT ACK */
#define   DMA_PIRQ              BIT(2)                  /* DMA Peripheral Error Interrupt Status */
#define   DMA_ERR               BIT(1)                  /* DMA Error Interrupt Status */
#define   DMA_DONE              BIT(0)                  /* DMA Completion Interrupt Status */
#define DMA_XCNT_CUR            0x34
#define DMA_YCNT_CUR            0x38
#define DMA_BWLCNT              0x40
#define DMA_BWLCNT_CUR          0x44
#define DMA_BWMCNT              0x48
#define DMA_BWMCNT_CUR          0x4c

#define DMA_DESC_FETCH          0x100                   /* DMA is fetching descriptors */
#define DMA_DATA_XFER           0x200                   /* DMA is in data transfer state */
#define DMA_IDLE_MASK           0x700
#define DMA_IDLE(x)             (((x)& DMA_IDLE_MASK) == 0)
#define DMA_FETCHING_DESC(x)    ((x)& DMA_DESC_FETCH)
#define DMA_XFER_DATA(x)        ((x)& DMA_DATA_XFER)

#define TIMEOUT_100_MS          (HZ / 10)

DEFINE_SPINLOCK(ndma_reset_lock);

enum adrv906x_ndma_rx_filter_status {
	NDMA_RX_FILTER_OFF,
	NDMA_RX_FILTER_ON
};

enum adrv906x_ndma_error {
	NDMA_NO_ERROR,
	NDMA_TX_FRAME_SIZE_ERROR,
	NDMA_TX_DATA_HEADER_ERROR,
	NDMA_TX_STATUS_HEADER_ERROR,
	NDMA_TX_TSTAMP_TIMEOUT_ERROR,
	NDMA_TX_SEQNUM_MISMATCH_ERROR,
	NDMA_TX_DMA_TRANS_ERROR,
	NDMA_TX_UNKNOWN_ERROR,
	NDMA_RX_FRAME_SIZE_ERROR,
	NDMA_RX_FRAME_DROPPED_ERROR,
	NDMA_RX_ERROR,
	NDMA_RX_SEQNUM_MISMATCH_ERROR,
	NDMA_RX_DMA_TRANS_ERROR,
	NDMA_RX_UNKNOWN_ERROR,
};

enum adrv906x_ndma_irqs {
	NDMA_TX_DATA_DMA_ERR_IRQ	= BIT(0),
	NDMA_TX_DATA_DMA_DMADONE_IRQ	= BIT(1),
	NDMA_TX_DATA_DMA_DONE_IRQ	= BIT(2),
	NDMA_TX_STATUS_DMA_ERR_IRQ	= BIT(3),
	NDMA_TX_STATUS_DMA_DMADONE_IRQ	= BIT(4),
	NDMA_TX_STATUS_DMA_DONE_IRQ	= BIT(5),
	NDMA_TX_STATUS_IRQ		= BIT(6),
	NDMA_TX_ERR_IRQ			= BIT(7),
	NDMA_RX_ERR_IRQ			= BIT(8),
	NDMA_RX_STATUS_IRQ		= BIT(9),
	NDMA_RX_DMA_ERR_IRQ		= BIT(10),
	NDMA_RX_DMA_DMADONE_IRQ		= BIT(11),
	NDMA_RX_DMA_DONE_IRQ		= BIT(12),
};

enum adrv906x_ndma_rx_filter_id {
	NDMA_RX_IPV4_FILTER = 0,
	NDMA_RX_IPV6_FILTER,
	NDMA_RX_ETH_FILTER,
	NDMA_RX_GENERIC_FILTER_0,
	NDMA_RX_GENERIC_FILTER_1,
	NDMA_RX_GENERIC_FILTER_2,
	NDMA_RX_GENERIC_FILTER_3,
	NDMA_RX_GENERIC_FILTER_4,
	NDMA_RX_FILTER_CNT,
};

static void get_ts_from_status(unsigned char *status, struct timespec64 *ts)
{
	ts->tv_nsec = ((uint32_t)status[4]) | ((uint32_t)status[5] << 8) |
		      ((uint32_t)status[6] << 16) | ((uint32_t)status[7] << 24);

	ts->tv_sec = ((uint64_t)status[8]) | ((uint64_t)status[9] << 8) |
		     ((uint64_t)status[10] << 16) | ((uint64_t)status[11] << 24) |
		     ((uint64_t)status[12] << 32) | ((uint64_t)status[13] << 40);
}

static bool is_timestamp_all_zero(unsigned char *status)
{
	unsigned char str[12] = { 0 };

	return !memcmp(&status[2], str, 12);
}

static void adrv906x_ndma_enable_irqs(struct adrv906x_ndma_dev *ndma_dev,
				      enum adrv906x_ndma_irqs irqs)
{
	unsigned int val;

	val = 0;
	if (irqs & NDMA_TX_DATA_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_ERR_EN;
	if (irqs & NDMA_TX_DATA_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_DATA_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DONE_EN;
	if (irqs & NDMA_TX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_ERR_EN;
	if (val) {
		val |= ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
	}

	val = 0;
	if (irqs & NDMA_TX_STATUS_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_ERR_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DONE_EN;
	if (irqs & NDMA_TX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_EN;
	if (irqs & NDMA_RX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_RX_STATUS_EN;
	if (val) {
		val |= ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
	}

	val = 0;
	if (irqs & NDMA_RX_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_ERR_EN;
	if (irqs & NDMA_RX_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DMADONE_EN;
	if (irqs & NDMA_RX_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DONE_EN;
	if (irqs & NDMA_RX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_ERR_EN;

	if (val) {
		val |= ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
	}
}

static void adrv906x_ndma_disable_irqs(struct adrv906x_ndma_dev *ndma_dev,
				       enum adrv906x_ndma_irqs irqs)
{
	unsigned int val;

	val = 0;
	if (irqs & NDMA_TX_DATA_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_ERR_EN;
	if (irqs & NDMA_TX_DATA_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_DATA_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_DMA_DONE_EN;
	if (irqs & NDMA_TX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_ERR_EN;
	if (val) {
		val = ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX) & ~val;
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
	}

	val = 0;
	if (irqs & NDMA_TX_STATUS_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_ERR_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DMADONE_EN;
	if (irqs & NDMA_TX_STATUS_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_DMA_DONE_EN;
	if (irqs & NDMA_TX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_TX_STATUS_EN;
	if (irqs & NDMA_RX_STATUS_IRQ)
		val |= NDMA_INTR_CTRL_RX_STATUS_EN;
	if (val) {
		val = ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS) & ~val;
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
	}

	val = 0;
	if (irqs & NDMA_RX_DMA_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_ERR_EN;
	if (irqs & NDMA_RX_DMA_DMADONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DMADONE_EN;
	if (irqs & NDMA_RX_DMA_DONE_IRQ)
		val |= NDMA_INTR_CTRL_RX_DMA_DONE_EN;
	if (irqs & NDMA_RX_ERR_IRQ)
		val |= NDMA_INTR_CTRL_RX_ERR_EN;
	if (val) {
		val = ioread32(ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX) & ~val;
		iowrite32(val, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
	}
}

static void adrv906x_ndma_disable_all_irqs(struct adrv906x_ndma_dev *ndma_dev)
{
	iowrite32(0, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_TX);
	iowrite32(0, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_STATUS);
	iowrite32(0, ndma_dev->intr_ctrl + NDMA_INTR_CTRL_RX);
}

static void adrv906x_dma_rx_reset(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(0, ndma_ch->rx_dma_base + DMA_CFG);
	iowrite32(DMA_ERR | DMA_PIRQ | DMA_DONE, ndma_ch->rx_dma_base + DMA_STAT);
}

static void adrv906x_dma_tx_reset(struct adrv906x_ndma_chan *ndma_ch)
{
	if (ndma_ch->tx_dma_base) {
		iowrite32(0, ndma_ch->tx_dma_base + DMA_CFG);
		iowrite32(DMA_ERR | DMA_PIRQ | DMA_DONE, ndma_ch->tx_dma_base + DMA_STAT);
	}
}

static void adrv906x_dma_rx_start(struct adrv906x_ndma_chan *ndma_ch)
{
	dma_addr_t desc_addr;

	desc_addr = ndma_ch->rx_ring_dma + sizeof(struct dma_desc) * ndma_ch->rx_tail;

	iowrite32(0, ndma_ch->rx_dma_base + DMA_CFG);
	iowrite32(desc_addr, ndma_ch->rx_dma_base + DMA_NEXT_DESC);
	iowrite32(ndma_ch->rx_ring[ndma_ch->rx_tail].cfg, ndma_ch->rx_dma_base + DMA_CFG);
}

static void adrv906x_dma_tx_start(struct adrv906x_ndma_chan *ndma_ch)
{
	dma_addr_t desc_addr;

	mod_timer(&ndma_ch->tx_timer, jiffies + TIMEOUT_100_MS);
	desc_addr = ndma_ch->tx_ring_dma + sizeof(struct dma_desc) * ndma_ch->tx_tail;
	iowrite32(0, ndma_ch->tx_dma_base + DMA_CFG);
	iowrite32(desc_addr, ndma_ch->tx_dma_base + DMA_NEXT_DESC);
	iowrite32(ndma_ch->tx_ring[ndma_ch->tx_tail].cfg | DMAFLOW_LIST,
		  ndma_ch->tx_dma_base + DMA_CFG);
}

static irqreturn_t adrv906x_dma_rx_done_irq_handler(int irq, void *ctx)
{
	struct adrv906x_ndma_chan *ndma_ch = ctx;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	unsigned long flags;

	if (napi_schedule_prep(&ndma_ch->napi)) {
		spin_lock_irqsave(&ndma_ch->lock, flags);
		if (ndma_ch->chan_type == NDMA_RX)
			adrv906x_ndma_disable_irqs(ndma_dev, NDMA_RX_DMA_DONE_IRQ);
		else
			adrv906x_ndma_disable_irqs(ndma_dev, NDMA_TX_STATUS_DMA_DONE_IRQ);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);
		__napi_schedule(&ndma_ch->napi);
	}

	return IRQ_HANDLED;
}

static irqreturn_t adrv906x_dma_error_irq_handler(int irq, void *ctx)
{
	struct adrv906x_ndma_chan *ndma_ch = ctx;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;

	if (ndma_ch->rx_dma_error_irq == irq)
		adrv906x_dma_rx_reset(ndma_ch);
	else
		adrv906x_dma_tx_reset(ndma_ch);

	dev_dbg(dev, "%s", __func__);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t adrv906x_dma_error_irq_handler_thread(int irq, void *ctx)
{
	struct adrv906x_ndma_chan *ndma_ch = ctx;
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;
	unsigned long flags;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	if (ndma_ch->chan_type == NDMA_RX) {
		ndma_ch->stats.rx.dma_errors++;
		adrv906x_dma_rx_start(ndma_ch);
	} else {
		if (ndma_ch->rx_dma_error_irq == irq) {
			ndma_ch->stats.tx.status_dma_errors++;
			adrv906x_dma_rx_start(ndma_ch);
		} else {
			ndma_ch->stats.tx.data_dma_errors++;
			adrv906x_dma_tx_start(ndma_ch);
		}
	}
	spin_unlock_irqrestore(&ndma_ch->lock, flags);

	dev_dbg(dev, "%s", __func__);

	return IRQ_HANDLED;
}

static void adrv906x_ndma_chan_enable(struct adrv906x_ndma_chan *ndma_ch)
{
	unsigned int val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX) ? NDMA_RX_STAT_AND_CTRL : NDMA_TX_STAT_AND_CTRL;

	val = ioread32(ndma_ch->ctrl_base + offset);
	val |= NDMA_DATAPATH_EN;
	iowrite32(val, ndma_ch->ctrl_base + offset);
}

static bool adrv906x_ndma_chan_enabled(struct adrv906x_ndma_chan *ndma_ch)
{
	unsigned int val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX) ? NDMA_RX_STAT_AND_CTRL : NDMA_TX_STAT_AND_CTRL;
	val = ioread32(ndma_ch->ctrl_base + offset);
	return val & NDMA_DATAPATH_EN;
}

static void adrv906x_ndma_chan_disable(struct adrv906x_ndma_chan *ndma_ch)
{
	unsigned int val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX) ? NDMA_RX_STAT_AND_CTRL : NDMA_TX_STAT_AND_CTRL;

	val = ioread32(ndma_ch->ctrl_base + offset);
	val &= ~NDMA_DATAPATH_EN;
	iowrite32(val, ndma_ch->ctrl_base + offset);

	/* Reset DMAs just to ensure there will be no active DMA transfer during NDMA reset */
	adrv906x_dma_rx_reset(ndma_ch);
	adrv906x_dma_tx_reset(ndma_ch);
}

static void adrv906x_ndma_set_frame_size(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	unsigned int val;

	val = FIELD_PREP(NDMA_RX_MIN_FRAME_SIZE, NDMA_MIN_FRAME_SIZE_VALUE)
	      | FIELD_PREP(NDMA_RX_MAX_FRAME_SIZE, NDMA_MAX_FRAME_SIZE_VALUE);
	iowrite32(val, rx_chan->ctrl_base + NDMA_RX_FRAME_SIZE);

	val = FIELD_PREP(NDMA_TX_MIN_FRAME_SIZE, NDMA_MIN_FRAME_SIZE_VALUE)
	      | FIELD_PREP(NDMA_TX_MAX_FRAME_SIZE, NDMA_MAX_FRAME_SIZE_VALUE);
	iowrite32(val, tx_chan->ctrl_base + NDMA_TX_FRAME_SIZE);
}

void adrv906x_ndma_set_tx_timeout_value(struct adrv906x_ndma_dev *ndma_dev, u32 val)
{
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;

	iowrite32(val, tx_chan->ctrl_base + NDMA_TX_TIMEOUT_VALUE);
}

void adrv906x_ndma_set_ptp_mode(struct adrv906x_ndma_dev *ndma_dev, u32 mode)
{
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&ndma_reset_lock, flags);
	val = ioread32(reset->reg);
	if (ndma_dev->dev_num == 0) {
		val &= ~NDMA_TX0_PTP_MODE;
		val |= FIELD_PREP(NDMA_TX0_PTP_MODE, mode);
	} else {
		val &= ~NDMA_TX1_PTP_MODE;
		val |= FIELD_PREP(NDMA_TX1_PTP_MODE, mode);
	}
	iowrite32(val, reset->reg);
	spin_unlock_irqrestore(&ndma_reset_lock, flags);
}

static void adrv906x_ndma_config_rx_ipv4_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(0x11450800, ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_VALUE0);
	iowrite32(0x140013F, ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_VALUE1);
	iowrite32(0x24170E0C, ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_OFFSET0);
	iowrite32(0x2A, ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_OFFSET1);
	iowrite32(0x00, ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_MASK0);
	iowrite32(0xF0000, ndma_ch->ctrl_base + NDMA_RX_IPV4_FRAME_FIELD_MASK1);
}

static void adrv906x_ndma_config_rx_ipv6_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(0x110686DD, ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_VALUE0);
	iowrite32(0x140013F, ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_VALUE1);
	iowrite32(0x38140E0C, ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_OFFSET0);
	iowrite32(0x3E, ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_OFFSET1);
	iowrite32(0x00, ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_MASK0);
	iowrite32(0xF0000, ndma_ch->ctrl_base + NDMA_RX_IPV6_FRAME_FIELD_MASK1);
}

static void adrv906x_ndma_config_rx_eth_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(0x88F7, ndma_ch->ctrl_base + NDMA_RX_ETH_FRAME_FIELD_VALUE);
	iowrite32(0xE0C, ndma_ch->ctrl_base + NDMA_RX_ETH_FRAME_FIELD_OFFSET);
	iowrite32(0xF0000, ndma_ch->ctrl_base + NDMA_RX_ETH_FRAME_FIELD_MASK);
}

static void adrv906x_ndma_config_rx_splane_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	iowrite32(0xCB9810, ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_PTP_MSG_VALUE);
	iowrite32(0x88A88100, ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_TAG_VALUE);
	iowrite32(0x0C, ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_TAG_OFFSET);
	iowrite32(0x00, ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_TAG_MASK);
	iowrite32(0x84, ndma_ch->ctrl_base + NDMA_RX_SPLANE_FILTER_VLAN_FRAME_OFFSET);
}

static void adrv906x_ndma_config_rx_ecpri_filter(struct adrv906x_ndma_chan *ndma_ch)
{
	unsigned int val, mask, nbytes;

	/* eCPRI "One-Way delay measurement" message to match:
	 *   byte 12: 0xAE    
	 *   byte 13: 0xFE    
	 *   byte 15: 0x05    
	 *   byte 19: 0x00 or 0x01
	 */
	for (nbytes = 0; nbytes < 96; nbytes += 4) {
		if (nbytes == 12) {
			val = 0x0500FEAE;
			mask = 0x00FF0000;
		} else if (nbytes == 16) {
			val = 0;
			mask = 0x01FFFFFF;
		} else {
			val = 0;
			mask = 0xFFFFFFFF;
		}

		iowrite32(val, ndma_ch->ctrl_base + NDMA_RX_GEN_FILTER0_REG_OFFSET +
			  NDMA_RX_CYCLE0_LOWER_VALUE_REG_OFFSET + nbytes);
		iowrite32(mask, ndma_ch->ctrl_base + NDMA_RX_GEN_FILTER0_REG_OFFSET +
			  NDMA_RX_CYCLE0_LOWER_MASK_REG_OFFSET + nbytes);
	}
}

static void adrv906x_ndma_enable_rx_filter(struct adrv906x_ndma_dev *ndma_dev,
					   unsigned int filter_en_mask,
					   enum adrv906x_ndma_rx_filter_status status)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	unsigned int val;

	val = ioread32(rx_chan->ctrl_base + NDMA_RX_SPLANE_FILTER_EN);

	if (status == NDMA_RX_FILTER_ON)
		val |= filter_en_mask;
	else
		val &= ~filter_en_mask;

	iowrite32(val, rx_chan->ctrl_base + NDMA_RX_SPLANE_FILTER_EN);
}

static void adrv906x_ndma_config_rx_filter(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	unsigned int en_mask;

	adrv906x_ndma_config_rx_ipv4_filter(rx_chan);
	adrv906x_ndma_config_rx_ipv6_filter(rx_chan);
	adrv906x_ndma_config_rx_eth_filter(rx_chan);
	adrv906x_ndma_config_rx_ecpri_filter(rx_chan);
	adrv906x_ndma_config_rx_splane_filter(rx_chan);

	en_mask = BIT(NDMA_RX_IPV4_FILTER)
		  | BIT(NDMA_RX_IPV6_FILTER)
		  | BIT(NDMA_RX_ETH_FILTER)
		  | BIT(NDMA_RX_GENERIC_FILTER_0);

	adrv906x_ndma_enable_rx_filter(ndma_dev, en_mask, NDMA_RX_FILTER_ON);
}

static int adrv906x_ndma_refill_rx(struct adrv906x_ndma_chan *ndma_ch, int budget)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;
	struct sk_buff *skb;
	dma_addr_t addr;
	int done = 0;

	while (ndma_ch->rx_free < NDMA_RING_SIZE && done < budget) {
		skb = napi_alloc_skb(&ndma_ch->napi, NDMA_RX_PKT_BUF_SIZE);
		ndma_ch->rx_buffs[ndma_ch->rx_head] = skb;
		if (!skb)
			break;

		/* Initialize the first byte of work unit header to 0 */
		skb->data[0] = 0;
		addr = dma_map_single(dev, skb->data, NDMA_RX_PKT_BUF_SIZE,
				      DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(dev, addr))) {
			napi_consume_skb(skb, budget);
			break;
		}

		ndma_ch->rx_ring[ndma_ch->rx_head].start = addr;
		ndma_ch->rx_ring[ndma_ch->rx_head].cfg |= DMAEN;
		ndma_ch->rx_head = (ndma_ch->rx_head + 1) % NDMA_RING_SIZE;
		ndma_ch->rx_free++;
		done++;
	}

	return done;
}

static int adrv906x_ndma_init_irqs(struct device_node *node, struct adrv906x_ndma_dev *ndma_dev)
{
	struct device *dev = ndma_dev->dev;
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	int irq, ret;

	/* Init for TX */
	irq = of_irq_get_byname(node, "tx_data_dma_done");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_data_dma_done interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->tx_dma_done_irq = irq;

	irq = of_irq_get_byname(node, "tx_data_dma_error");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_data_dma_error interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->tx_dma_error_irq = irq;

	ret = devm_request_threaded_irq(dev, tx_chan->tx_dma_error_irq,
					adrv906x_dma_error_irq_handler,
					adrv906x_dma_error_irq_handler_thread,
					IRQF_ONESHOT, dev_name(dev), tx_chan);
	if (ret) {
		dev_err(dev, "failed to request tx_dma_error_irq interrupt");
		return ret;
	}

	irq = of_irq_get_byname(node, "tx_status_dma_done");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_status_dma_done interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->rx_dma_done_irq = irq;

	irq = of_irq_get_byname(node, "tx_status_dma_error");
	if (irq <= 0) {
		dev_err(dev, "failed to get tx_status_dma_error interrupt");
		return irq ? irq : -ENOENT;
	}
	tx_chan->rx_dma_error_irq = irq;

	ret = devm_request_irq(dev, tx_chan->rx_dma_done_irq,
			       adrv906x_dma_rx_done_irq_handler, 0, dev_name(dev), tx_chan);
	if (ret) {
		dev_err(dev, "failed to request tx_status_dma_done interrupt");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, tx_chan->rx_dma_error_irq,
					adrv906x_dma_error_irq_handler,
					adrv906x_dma_error_irq_handler_thread,
					IRQF_ONESHOT, dev_name(dev), tx_chan);
	if (ret) {
		dev_err(dev, "failed to request tx_status_dma_error interrupt");
		return ret;
	}

	/* Init for RX */
	irq = of_irq_get_byname(node, "rx_dma_done");
	if (irq <= 0) {
		dev_err(dev, "failed to get rx_dma_done interrupt");
		return irq ? irq : -ENOENT;
	}
	rx_chan->rx_dma_done_irq = irq;

	irq = of_irq_get_byname(node, "rx_dma_error");
	if (irq <= 0) {
		dev_err(dev, "failed to get rx_dma_error interrupt");
		return irq ? irq : -ENOENT;
	}
	rx_chan->rx_dma_error_irq = irq;

	ret = devm_request_irq(dev, rx_chan->rx_dma_done_irq,
			       adrv906x_dma_rx_done_irq_handler, 0, dev_name(dev), rx_chan);
	if (ret) {
		dev_err(dev, "failed to request rx_dma_done interrupt");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, rx_chan->rx_dma_error_irq,
					adrv906x_dma_error_irq_handler,
					adrv906x_dma_error_irq_handler_thread,
					IRQF_ONESHOT, dev_name(dev), rx_chan);
	if (ret) {
		dev_err(dev, "failed to request rx_dma_error interrupt");
		return ret;
	}

	return 0;
}

static int adrv906x_ndma_get_reset_ctrl(struct adrv906x_ndma_dev *ndma_dev,
					struct device_node *ndma_np)
{
	struct device *dev = ndma_dev->dev;
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	struct device_node *reset_np;
	unsigned int ret, reg, len;

	reset_np = of_parse_phandle(ndma_np, "reset-ctrl", 0);
	if (!reset_np) {
		dev_err(dev, "missing reset-ctrl property");
		return -ENODEV;
	}
	ret = of_property_read_u32_index(reset_np, "reg", 0, &reg);
	if (ret) {
		dev_err(dev, "missing reg property of reset-ctrl node");
		return -EINVAL;
	}
	ret = of_property_read_u32_index(reset_np, "reg", 1, &len);
	if (ret) {
		dev_err(dev, "missing reg length of reset-ctrl node");
		return -EINVAL;
	}
	reset->reg = devm_ioremap(dev->parent, reg, len);
	if (!reset->reg) {
		dev_err(dev, "ioremap ndma-rst failed!");
		return -EINVAL;
	}

	if (ndma_dev->dev_num == 0) {
		reset->rx_chan_reset_bit = NDMA_RX0_RST;
		reset->tx_chan_reset_bit = NDMA_TX0_RST;
	} else {
		reset->rx_chan_reset_bit = NDMA_RX1_RST;
		reset->tx_chan_reset_bit = NDMA_TX1_RST;
	}
	return 0;
}

static int adrv906x_ndma_get_intr_ctrl(struct adrv906x_ndma_dev *ndma_dev,
				       struct device_node *ndma_np)
{
	struct device *dev = ndma_dev->dev;
	unsigned int reg, len, ret;
	struct device_node *intr_ctrl;

	intr_ctrl = of_parse_phandle(ndma_np, "interrupt-ctrl", 0);
	if (!intr_ctrl) {
		dev_err(dev, "missing interrupt-ctrl property");
		return -ENODEV;
	}
	ret = of_property_read_u32_index(intr_ctrl, "reg", 0, &reg);
	if (ret) {
		dev_err(dev, "missing interrupt-ctrl node reg addr");
		return -EINVAL;
	}
	ret = of_property_read_u32_index(intr_ctrl, "reg", 1, &len);
	if (ret) {
		dev_err(dev, "missing interrupt-ctrl node reg length");
		return -EINVAL;
	}
	ndma_dev->intr_ctrl = devm_ioremap(dev->parent, reg, len);
	return 0;
}

static void adrv906x_ndma_get_frame_drop_stats(struct work_struct *work)
{
	struct adrv906x_ndma_dev *ndma_dev =
		container_of(work, struct adrv906x_ndma_dev, update_stats.work);
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	union adrv906x_ndma_chan_stats *stats = &rx_chan->stats;

	stats->rx.frame_dropped_splane_errors += ioread32(rx_chan->ctrl_base
							  + NDMA_RX_FRAME_DROPPED_COUNT_SPLANE);
	stats->rx.frame_dropped_mplane_errors += ioread32(rx_chan->ctrl_base
							  + NDMA_RX_FRAME_DROPPED_COUNT_MPLANE);
	stats->rx.frame_dropped_errors = stats->rx.frame_dropped_splane_errors
					 + stats->rx.frame_dropped_mplane_errors;
	iowrite32(0, rx_chan->ctrl_base + NDMA_RX_FRAME_DROPPED_COUNT_SPLANE);
	iowrite32(0, rx_chan->ctrl_base + NDMA_RX_FRAME_DROPPED_COUNT_MPLANE);

	mod_delayed_work(system_long_wq, &ndma_dev->update_stats, msecs_to_jiffies(1000));
}

static void adrv906x_dma_tx_prep_desc_list(struct adrv906x_ndma_chan *ndma_ch)
{
	struct dma_desc *tx_ring = ndma_ch->tx_ring;
	unsigned int desc_indx = ndma_ch->tx_tail;

	while (ndma_ch->tx_frames_waiting) {
		tx_ring[desc_indx].cfg |= DMAFLOW_LIST;
		desc_indx = (desc_indx + 1) % NDMA_RING_SIZE;
		ndma_ch->tx_frames_waiting--;
		ndma_ch->tx_frames_pending++;
	}

	desc_indx = (desc_indx) ? desc_indx - 1 : NDMA_RING_SIZE - 1;
	tx_ring[desc_indx].cfg &= ~DMAFLOW_LIST;
}

static void adrv906x_ndma_tx_timeout(struct timer_list *t)
{
	struct adrv906x_ndma_chan *ndma_ch = from_timer(ndma_ch, t, tx_timer);
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct dma_desc *tx_ring = ndma_ch->tx_ring;
	struct device *dev = ndma_dev->dev;
	struct timespec64 ts = { 0, 0 };
	unsigned long flags;
	struct sk_buff *skb;
	dma_addr_t addr;
	unsigned int size;
	unsigned char port;

	dev_warn(dev, "transmit timed out: tx status not received");

	spin_lock_irqsave(&ndma_ch->lock, flags);

	skb = ndma_ch->tx_buffs[ndma_ch->tx_tail];
	port = FIELD_GET(NDMA_TX_HDR_SOF_PORT_ID, skb->data[0]);
	addr = tx_ring[ndma_ch->tx_tail].start;
	size = tx_ring[ndma_ch->tx_tail].xcnt * tx_ring[ndma_ch->tx_tail].xmod;
	dma_unmap_single(dev, addr, size, DMA_TO_DEVICE);
	ndma_ch->tx_buffs[ndma_ch->tx_tail] = NULL;
	ndma_ch->tx_tail = (ndma_ch->tx_tail + 1) % NDMA_RING_SIZE;
	ndma_ch->status_cb_fn(skb, port, ts, ndma_ch->status_cb_param);
	ndma_ch->tx_frames_pending--;

	if (ndma_ch->tx_frames_pending) {
		mod_timer(&ndma_ch->tx_timer, jiffies + TIMEOUT_100_MS);
	} else if (ndma_ch->tx_frames_waiting) {
		adrv906x_dma_tx_prep_desc_list(ndma_ch);
		adrv906x_dma_tx_start(ndma_ch);
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;

	spin_unlock_irqrestore(&ndma_ch->lock, flags);
}

static int adrv906x_ndma_device_init(struct adrv906x_ndma_dev *ndma_dev, struct device_node *np)
{
	struct device *dev = ndma_dev->dev;
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	unsigned int reg, len;
	int ret;

	/* config TX  */
	ret = of_property_read_u32_index(np, "reg", 0, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 1, &len);
	if (ret)
		return ret;
	tx_chan->ctrl_base = devm_ioremap(dev->parent, reg, len);
	ret = of_property_read_u32_index(np, "reg", 2, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 3, &len);
	if (ret)
		return ret;
	tx_chan->tx_dma_base = devm_ioremap(dev->parent, reg, len);
	ret = of_property_read_u32_index(np, "reg", 4, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 5, &len);
	if (ret)
		return ret;
	tx_chan->rx_dma_base = devm_ioremap(dev->parent, reg, len);

	tx_chan->chan_type = NDMA_TX;
	tx_chan->chan_name = "NDMA_TX";
	tx_chan->parent = ndma_dev;

	/* config RX  */
	ret = of_property_read_u32_index(np, "reg", 6, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 7, &len);
	if (ret)
		return ret;
	rx_chan->ctrl_base = devm_ioremap(dev->parent, reg, len);
	ret = of_property_read_u32_index(np, "reg", 8, &reg);
	if (ret)
		return ret;
	ret = of_property_read_u32_index(np, "reg", 9, &len);
	if (ret)
		return ret;
	rx_chan->rx_dma_base = devm_ioremap(dev->parent, reg, len);
	rx_chan->chan_type = NDMA_RX;
	rx_chan->chan_name = "NDMA_RX";
	rx_chan->parent = ndma_dev;

	ret = adrv906x_ndma_init_irqs(np, ndma_dev);
	if (ret)
		return ret;

	spin_lock_init(&ndma_dev->lock);
	spin_lock_init(&tx_chan->lock);
	spin_lock_init(&rx_chan->lock);

	INIT_DELAYED_WORK(&ndma_dev->update_stats, adrv906x_ndma_get_frame_drop_stats);
	timer_setup(&tx_chan->tx_timer, adrv906x_ndma_tx_timeout, 0);

	return ret;
}

static void adrv906x_ndma_enable_events(struct adrv906x_ndma_chan *ndma_ch, unsigned int events)
{
	unsigned int val, offset;

	offset = (ndma_ch->chan_type == NDMA_RX) ? NDMA_RX_EVENT_EN : NDMA_TX_EVENT_EN;

	val = ioread32(ndma_ch->ctrl_base + offset);
	val |= events;
	iowrite32(val, ndma_ch->ctrl_base + offset);
}

static void adrv906x_ndma_disable_all_event(struct adrv906x_ndma_chan *ndma_ch)
{
	unsigned int offset;

	offset = (ndma_ch->chan_type == NDMA_RX) ? NDMA_RX_EVENT_EN : NDMA_TX_EVENT_EN;

	iowrite32(0, ndma_ch->ctrl_base + offset);
}

static void adrv906x_ndma_add_tx_header(struct sk_buff *skb, unsigned char seq_num,
					unsigned char port, bool hw_tstamp_en, bool dsa_en)
{
	unsigned int frame_len = skb->len;
	unsigned char *hdr;

	hdr = skb_push(skb, NDMA_TX_HDR_SOF_SIZE);

	hdr[0] = FIELD_PREP(NDMA_HDR_TYPE_MASK, NDMA_TX_HDR_TYPE_SOF)
		 | FIELD_PREP(NDMA_TX_HDR_SOF_FR_PTP, hw_tstamp_en)
		 | FIELD_PREP(NDMA_TX_HDR_SOF_PORT_ID, port)
		 | FIELD_PREP(NDMA_TX_HDR_SOF_DSA_EN, dsa_en);
	hdr[1] = seq_num;
	hdr[2] = frame_len & 0xff;
	hdr[3] = (frame_len >> 8) & 0xff;
	hdr[4] = 0;
	hdr[5] = 0;
	hdr[6] = 0;
	hdr[7] = 0;
}

static void adrv906x_ndma_reset(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_reset *reset = &ndma_dev->reset;
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	unsigned int val, reset_bits;
	unsigned long flags;

	spin_lock_irqsave(&ndma_reset_lock, flags);
	reset_bits = reset->rx_chan_reset_bit | reset->tx_chan_reset_bit;
	val = ioread32(reset->reg);
	iowrite32(val & ~reset_bits, reset->reg);
	iowrite32(val | reset_bits, reset->reg);
	spin_unlock_irqrestore(&ndma_reset_lock, flags);

	spin_lock_irqsave(&rx_chan->lock, flags);
	rx_chan->expected_seq_num = 0;
	spin_unlock_irqrestore(&rx_chan->lock, flags);

	spin_lock_irqsave(&tx_chan->lock, flags);
	ndma_dev->tx_chan.expected_seq_num = 1;
	ndma_dev->tx_chan.seq_num = 1;
	spin_unlock_irqrestore(&tx_chan->lock, flags);
}

int adrv906x_ndma_alloc_rings(struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev = ndma_dev->dev;
	void *tx_status_buffs;
	dma_addr_t addr;
	int i;

	/* Allocate and initialize DMA descriptor ring for RX data & status */
	rx_chan->rx_ring = dmam_alloc_coherent(dev,
					       sizeof(struct dma_desc) * NDMA_RING_SIZE,
					       &rx_chan->rx_ring_dma, GFP_KERNEL);
	if (!rx_chan->rx_ring)
		return -ENOMEM;

	for (i = 0; i < NDMA_RING_SIZE; i++) {
		rx_chan->rx_ring[i].cfg = (DESCIDCPY | DI_EN_X | NDSIZE_4 |
					   WDSIZE_64 | PSIZE_64 | WNR | DMAEN);
		rx_chan->rx_ring[i].cfg |=
			(i == NDMA_RING_SIZE - 1) ? DMAFLOW_STOP : DMAFLOW_LIST;
		rx_chan->rx_ring[i].xcnt = NDMA_RX_PKT_BUF_SIZE / XMODE_64;
		rx_chan->rx_ring[i].xmod = XMODE_64;
		rx_chan->rx_ring[i].next = rx_chan->rx_ring_dma +
					   sizeof(struct dma_desc) * ((i + 1) % NDMA_RING_SIZE);
	}

	/* Allocate and initialize DMA descriptor ring for TX data */
	tx_chan->tx_ring = dmam_alloc_coherent(dev,
					       sizeof(struct dma_desc) * NDMA_RING_SIZE,
					       &tx_chan->tx_ring_dma, GFP_KERNEL);
	if (!tx_chan->tx_ring)
		return -ENOMEM;

	for (i = 0; i < NDMA_RING_SIZE; i++) {
		tx_chan->tx_ring[i].cfg = (DESCIDCPY | NDSIZE_4 | DMASYNC | PSIZE_64 | DMAEN);
		tx_chan->tx_ring[i].next = tx_chan->tx_ring_dma +
					   sizeof(struct dma_desc) * ((i + 1) % NDMA_RING_SIZE);
	}

	/* Allocate and initialize DMA descriptor ring and buffers for TX status */
	tx_chan->rx_ring = dmam_alloc_coherent(dev,
					       sizeof(struct dma_desc) * NDMA_RING_SIZE,
					       &tx_chan->rx_ring_dma, GFP_KERNEL);
	if (!tx_chan->rx_ring)
		return -ENOMEM;

	tx_status_buffs = dmam_alloc_coherent(dev, NDMA_TX_HDR_STATUS_SIZE * NDMA_RING_SIZE,
					      &addr, GFP_KERNEL);
	if (!tx_status_buffs)
		return -ENOMEM;

	for (i = 0; i < NDMA_RING_SIZE; i++) {
		tx_chan->rx_buffs[i] = tx_status_buffs + i * NDMA_TX_HDR_STATUS_SIZE;

		tx_chan->rx_ring[i].cfg = (DESCIDCPY | DI_EN_X | NDSIZE_4 |
					   WDSIZE_64 | PSIZE_32 | WNR | DMAEN | DMAFLOW_LIST);
		tx_chan->rx_ring[i].xcnt = NDMA_TX_HDR_STATUS_SIZE / XMODE_64;
		tx_chan->rx_ring[i].xmod = XMODE_64;
		tx_chan->rx_ring[i].next = tx_chan->rx_ring_dma +
					   sizeof(struct dma_desc) * ((i + 1) % NDMA_RING_SIZE);
		tx_chan->rx_ring[i].start = addr + i * NDMA_TX_HDR_STATUS_SIZE;
	}

	return 0;
}

void adrv906x_ndma_open(struct adrv906x_ndma_dev *ndma_dev, ndma_callback tx_cb_fn,
			ndma_callback rx_cb_fn, void *cb_param)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	unsigned long flags0, flags1;

	spin_lock_irqsave(&ndma_dev->lock, flags0);
	if (!ndma_dev->enabled) {
		adrv906x_ndma_disable_all_irqs(ndma_dev);
		adrv906x_ndma_enable_events(rx_chan, NDMA_RX_ERROR_EVENTS);
		adrv906x_ndma_enable_events(tx_chan, NDMA_TX_ERROR_EVENTS);

		memset(&rx_chan->stats, 0, sizeof(union adrv906x_ndma_chan_stats));
		memset(&tx_chan->stats, 0, sizeof(union adrv906x_ndma_chan_stats));

		adrv906x_ndma_config_rx_filter(ndma_dev);
		adrv906x_ndma_set_frame_size(ndma_dev);

		spin_lock_irqsave(&tx_chan->lock, flags1);
		adrv906x_dma_rx_reset(tx_chan);
		adrv906x_dma_tx_reset(tx_chan);
		adrv906x_ndma_chan_enable(tx_chan);
		spin_unlock_irqrestore(&tx_chan->lock, flags1);

		spin_lock_irqsave(&rx_chan->lock, flags1);
		adrv906x_dma_rx_reset(rx_chan);
		adrv906x_dma_tx_reset(rx_chan);
		adrv906x_ndma_chan_enable(rx_chan);
		spin_unlock_irqrestore(&rx_chan->lock, flags1);

		tx_chan->status_cb_fn = tx_cb_fn;
		tx_chan->status_cb_param = cb_param;
		tx_chan->rx_head = 0;
		tx_chan->rx_tail = 0;
		tx_chan->rx_free = 0;
		tx_chan->tx_head = 0;
		tx_chan->tx_tail = 0;
		tx_chan->tx_frames_waiting = 0;
		tx_chan->tx_frames_pending = 0;
		adrv906x_dma_rx_start(tx_chan);
		napi_enable(&tx_chan->napi);

		rx_chan->status_cb_fn = rx_cb_fn;
		rx_chan->status_cb_param = cb_param;
		rx_chan->rx_head = 0;
		rx_chan->rx_tail = 0;
		rx_chan->rx_free = 0;
		adrv906x_ndma_refill_rx(rx_chan, NDMA_RING_SIZE);
		adrv906x_dma_rx_start(rx_chan);
		napi_enable(&rx_chan->napi);

		adrv906x_ndma_enable_irqs(ndma_dev,
					  NDMA_RX_DMA_ERR_IRQ |
					  NDMA_RX_DMA_DONE_IRQ |
					  NDMA_TX_DATA_DMA_ERR_IRQ |
					  NDMA_TX_STATUS_DMA_ERR_IRQ |
					  NDMA_TX_STATUS_DMA_DONE_IRQ);

		ndma_dev->enabled = true;
		kref_init(&ndma_dev->refcount);
	} else {
		kref_get(&ndma_dev->refcount);
	}

	mod_delayed_work(system_long_wq, &ndma_dev->update_stats, msecs_to_jiffies(1000));
	spin_unlock_irqrestore(&ndma_dev->lock, flags0);
}

static void adrv906x_ndma_stop(struct kref *ref)
{
	struct adrv906x_ndma_dev *ndma_dev = container_of(ref, struct adrv906x_ndma_dev, refcount);
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct device *dev = ndma_dev->dev;
	struct timespec64 ts = { 0, 0 };
	struct sk_buff *skb;
	dma_addr_t addr;
	unsigned long flags;
	unsigned int size, num_frames;
	unsigned char port;

	del_timer_sync(&tx_chan->tx_timer);

	spin_lock_irqsave(&ndma_dev->lock, flags);
	adrv906x_ndma_disable_all_irqs(ndma_dev);
	cancel_delayed_work(&ndma_dev->update_stats);
	spin_unlock_irqrestore(&ndma_dev->lock, flags);

	/* Disable ndma RX channel */
	napi_disable(&rx_chan->napi);
	spin_lock_irqsave(&rx_chan->lock, flags);
	adrv906x_ndma_disable_all_event(rx_chan);
	adrv906x_ndma_chan_disable(rx_chan);

	while (rx_chan->rx_free) {
		skb = (struct sk_buff *)rx_chan->rx_buffs[rx_chan->rx_tail];
		addr = rx_chan->rx_ring[rx_chan->rx_tail].start;

		dma_unmap_single(dev, addr, NDMA_RX_PKT_BUF_SIZE, DMA_FROM_DEVICE);
		dev_kfree_skb(skb);

		rx_chan->rx_tail = (rx_chan->rx_tail + 1) % NDMA_RING_SIZE;
		rx_chan->rx_free--;
	}
	spin_unlock_irqrestore(&rx_chan->lock, flags);

	/* Disable ndma TX channel */
	napi_disable(&tx_chan->napi);
	spin_lock_irqsave(&tx_chan->lock, flags);
	adrv906x_ndma_disable_all_event(tx_chan);
	adrv906x_ndma_chan_disable(tx_chan);

	num_frames = tx_chan->tx_frames_waiting + tx_chan->tx_frames_pending;
	while (num_frames--) {
		skb = tx_chan->tx_buffs[tx_chan->tx_tail];
		port = FIELD_GET(NDMA_TX_HDR_SOF_PORT_ID, skb->data[0]);
		addr = tx_chan->tx_ring[tx_chan->tx_tail].start;
		size = tx_chan->tx_ring[tx_chan->tx_tail].xcnt *
		       tx_chan->tx_ring[tx_chan->tx_tail].xmod;
		dma_unmap_single(dev, addr, size, DMA_TO_DEVICE);

		tx_chan->tx_buffs[tx_chan->tx_tail] = NULL;
		tx_chan->tx_tail = (tx_chan->tx_tail + 1) % NDMA_RING_SIZE;
		tx_chan->status_cb_fn(skb, port, ts, tx_chan->status_cb_param);
	}
	tx_chan->tx_frames_pending = 0;
	tx_chan->tx_frames_waiting = 0;
	spin_unlock_irqrestore(&tx_chan->lock, flags);

	adrv906x_ndma_reset(ndma_dev);
	ndma_dev->enabled = false;
}

void adrv906x_ndma_close(struct adrv906x_ndma_dev *ndma_dev)
{
	kref_put(&ndma_dev->refcount, adrv906x_ndma_stop);
}

static int adrv906x_ndma_parse_rx_status_header(struct adrv906x_ndma_chan *ndma_ch,
						unsigned char *status_hdr,
						struct timespec64 *ts, unsigned int *port_id,
						unsigned int *frame_size)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct device *dev = ndma_dev->dev;
	int ret = NDMA_NO_ERROR;
	unsigned int error;

	get_ts_from_status(status_hdr, ts);
	*port_id = FIELD_GET(NDMA_RX_HDR_STATUS_PORT_ID, status_hdr[0]);
	*frame_size = (status_hdr[NDMA_RX_FRAME_LEN_MSB] << 8)
		      | (status_hdr[NDMA_RX_FRAME_LEN_LSB]);

	if (NDMA_RX_HDR_STATUS_FR_ERR & status_hdr[0]) {
		error = ioread32(ndma_ch->ctrl_base + NDMA_RX_EVENT_STAT) & NDMA_RX_ERROR_EVENTS;

		/* Get error type from IRQ status register and update statistics.
		 * Note: More than one error bit in IRQ status register can be set,
		 * so to avoid losing them, we clear only one error bit at a time.
		 */
		if (NDMA_RX_FRAME_SIZE_ERR_EVENT & error) {
			dev_dbg(dev, "%s_%u frame size error",
				ndma_ch->chan_name, ndma_dev->dev_num);
			stats->rx.frame_size_errors++;
			if (NDMA_RX_HDR_STATUS_FR_DROP_ERR & status_hdr[0])
				dev_dbg(dev, "%s_%u partial frame dropped error",
					ndma_ch->chan_name, ndma_dev->dev_num);
			iowrite32(NDMA_RX_FRAME_SIZE_ERR_EVENT, ndma_ch->ctrl_base +
				  NDMA_RX_EVENT_STAT);
			ret = NDMA_RX_FRAME_SIZE_ERR_EVENT;
		} else if (NDMA_RX_ERR_EVENT & error) {
			dev_dbg(dev, "%s_%u mac error(s) signaled by tuser[0]",
				ndma_ch->chan_name, ndma_dev->dev_num);
			stats->rx.frame_errors++;
			iowrite32(NDMA_RX_ERR_EVENT, ndma_ch->ctrl_base +
				  NDMA_RX_EVENT_STAT);
			ret = NDMA_RX_ERR_EVENT;
		} else if (NDMA_RX_FRAME_DROPPED_ERR_EVENT & error) {
			dev_dbg(dev, "%s_%u frame dropped error",
				ndma_ch->chan_name, ndma_dev->dev_num);
			iowrite32(NDMA_RX_FRAME_DROPPED_ERR_EVENT, ndma_ch->ctrl_base +
				  NDMA_RX_EVENT_STAT);
			ret = NDMA_RX_FRAME_DROPPED_ERR_EVENT;
		} else {
			dev_dbg(dev, "%s_%u status wu has error flag set but no interrupt generated",
				ndma_ch->chan_name, ndma_dev->dev_num);
			stats->rx.unknown_errors++;
			ret = NDMA_RX_UNKNOWN_ERROR;
		}
	} else {
		/* If no error, check and update sequence number */
		if (status_hdr[1] != ndma_ch->expected_seq_num) {
			dev_dbg(dev, "%s_%u frame seq number mismatch, exp:0x%x recv:0x%x",
				ndma_ch->chan_name, ndma_dev->dev_num, ndma_ch->expected_seq_num,
				status_hdr[1]);
			stats->rx.seqnumb_mismatch_errors++;
			ndma_ch->expected_seq_num = status_hdr[1];
			ret = NDMA_RX_SEQNUM_MISMATCH_ERROR;
		}
		ndma_ch->expected_seq_num++;
	}

	return ret;
}

void adrv906x_ndma_process_rx_work_unit(struct adrv906x_ndma_chan *ndma_ch,
					struct sk_buff *skb, int budget)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;
	struct timespec64 ts = { 0, 0 };
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	unsigned int port_id = 0, frame_size = 0;
	int ret;

	/* Status WU type */
	if (FIELD_GET(NDMA_HDR_TYPE_MASK, skb->data[0]) == NDMA_RX_HDR_TYPE_STATUS) {
		ret = adrv906x_ndma_parse_rx_status_header(ndma_ch, skb->data, &ts,
							   &port_id, &frame_size);
		if (ret == NDMA_NO_ERROR || ret == NDMA_RX_SEQNUM_MISMATCH_ERROR) {
			if (ndma_ch->skb_rx_data_wu) {
				skb_put(ndma_ch->skb_rx_data_wu, NDMA_RX_HDR_DATA_SIZE +
					frame_size);
				skb_pull(ndma_ch->skb_rx_data_wu, NDMA_RX_HDR_DATA_SIZE);
				ndma_ch->status_cb_fn(ndma_ch->skb_rx_data_wu, port_id, ts,
						      ndma_ch->status_cb_param);
			} else {
				dev_dbg(dev, "%s_%u received status without preceding data wu",
					ndma_ch->chan_name, ndma_dev->dev_num);
			}
		} else {
			/* If error detected, free skb with associated data WU */
			if (ndma_ch->skb_rx_data_wu)
				napi_consume_skb(ndma_ch->skb_rx_data_wu, budget);
		}
		ndma_ch->skb_rx_data_wu = NULL;
		napi_consume_skb(skb, budget); /* free skb with status WU */
		/* Data WU type */
	} else if (FIELD_GET(NDMA_HDR_TYPE_MASK, skb->data[0]) == NDMA_RX_HDR_TYPE_DATA) {
		if (skb->data[0] & NDMA_RX_HDR_TYPE_DATA_SOF) { /* Start of Frame */
			if (ndma_ch->skb_rx_data_wu) {
				dev_dbg(dev, "%s_%u no status received for previous frame",
					ndma_ch->chan_name, ndma_dev->dev_num);
				napi_consume_skb(ndma_ch->skb_rx_data_wu, budget);
			}
			ndma_ch->skb_rx_data_wu = skb;
		} else { /* Subsequent WU type is unsupported */
			napi_consume_skb(skb, budget);
			if (ndma_ch->skb_rx_data_wu)
				napi_consume_skb(ndma_ch->skb_rx_data_wu, budget);
			ndma_ch->skb_rx_data_wu = NULL;
			dev_dbg(dev, "%s_%u unsupported type of received wu",
				ndma_ch->chan_name, ndma_dev->dev_num);
		}
		/* Incorrect WU type */
	} else {
		dev_dbg(dev, "%s_%u incorrect type of received wu",
			ndma_ch->chan_name, ndma_dev->dev_num);
		napi_consume_skb(skb, budget);
		if (ndma_ch->skb_rx_data_wu)
			napi_consume_skb(ndma_ch->skb_rx_data_wu, budget);
		ndma_ch->skb_rx_data_wu = NULL;
	}

	stats->rx.done_work_units++;
}

static int adrv906x_ndma_parse_tx_status_header(struct adrv906x_ndma_chan *ndma_ch,
						unsigned char *status_hdr,
						struct timespec64 *ts)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct device *dev = ndma_dev->dev;
	unsigned int error;
	int ret = NDMA_NO_ERROR;

	if (FIELD_GET(NDMA_HDR_TYPE_MASK, status_hdr[0]) != NDMA_TX_HDR_TYPE_STATUS) {
		dev_dbg(dev, "%s_%u incorrect format of wu status header: 0x%x",
			ndma_ch->chan_name, ndma_dev->dev_num, status_hdr[0]);
		stats->tx.status_header_errors++;
		ret = NDMA_TX_STATUS_HEADER_ERROR;
	} else {
		get_ts_from_status(status_hdr, ts);

		if (status_hdr[1] != ndma_ch->expected_seq_num) {
			dev_dbg(dev, "%s_%u frame seq number mismatch, exp:0x%x recv:0x%x",
				ndma_ch->chan_name, ndma_dev->dev_num, ndma_ch->expected_seq_num,
				status_hdr[1]);
			stats->tx.seqnumb_mismatch_errors++;
			/* seq number mismatch, update it to new value */
			ndma_ch->expected_seq_num = status_hdr[1];
			ret = NDMA_TX_SEQNUM_MISMATCH_ERROR;
		}
		ndma_ch->expected_seq_num++;
		if (ndma_ch->expected_seq_num == 0)
			ndma_ch->expected_seq_num++;

		if (NDMA_TX_HDR_STATUS_FR_ERR & status_hdr[0]) {
			if (adrv906x_ndma_chan_enabled(ndma_ch) &&
			    is_timestamp_all_zero(status_hdr)) {
				dev_dbg(dev, "%s_%u hw timestamp timeout error",
					ndma_ch->chan_name, ndma_dev->dev_num);
				stats->tx.tstamp_timeout_errors++;
				ret = NDMA_TX_TSTAMP_TIMEOUT_ERROR;
			} else {
				/* Get error type from IRQ status register and update statistics.
				 * Note: More than one error bit in IRQ status register can be set,
				 * so to avoid losing them, we clear only one error bit at a time.
				 */
				error = ioread32(ndma_ch->ctrl_base + NDMA_TX_EVENT_STAT) &
					NDMA_TX_ERROR_EVENTS;

				if (NDMA_TX_FRAME_SIZE_ERR_EVENT & error) {
					dev_dbg(dev, "%s_%u frame size error",
						ndma_ch->chan_name, ndma_dev->dev_num);
					stats->tx.frame_size_errors++;
					iowrite32(NDMA_TX_FRAME_SIZE_ERR_EVENT, ndma_ch->ctrl_base +
						  NDMA_TX_EVENT_STAT);
					ret = NDMA_TX_FRAME_SIZE_ERROR;
				} else if (NDMA_TX_WU_HEADER_ERR_EVENT & error) {
					dev_dbg(dev, "%s_%u incorrect format of wu data header",
						ndma_ch->chan_name, ndma_dev->dev_num);
					stats->tx.data_header_errors++;
					iowrite32(NDMA_TX_WU_HEADER_ERR_EVENT, ndma_ch->ctrl_base +
						  NDMA_TX_EVENT_STAT);
					ret = NDMA_TX_DATA_HEADER_ERROR;
				} else {
					dev_dbg(dev, "%s_%u status wu has set error flag, but there is no error flag in status register",
						ndma_ch->chan_name, ndma_dev->dev_num);
					stats->tx.unknown_errors++;
					ret = NDMA_RX_UNKNOWN_ERROR;
				}
			}
		}
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;
	stats->tx.done_work_units++;

	return ret;
}

static void adrv906x_ndma_process_tx_status(struct adrv906x_ndma_chan *ndma_ch,
					    unsigned char *status)
{
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct device *dev = ndma_dev->dev;
	struct dma_desc *tx_ring = ndma_ch->tx_ring;
	struct timespec64 ts = { 0, 0 };
	struct sk_buff *skb;
	dma_addr_t addr;
	unsigned int size;
	unsigned char port;
	int ret;

	ret = adrv906x_ndma_parse_tx_status_header(ndma_ch, status, &ts);
	if (ret)
		/* ndma channel must be re-enabled after error occurrence */
		adrv906x_ndma_chan_enable(ndma_ch);

	skb = ndma_ch->tx_buffs[ndma_ch->tx_tail];
	port = FIELD_GET(NDMA_TX_HDR_SOF_PORT_ID, skb->data[0]);
	addr = tx_ring[ndma_ch->tx_tail].start;
	size = tx_ring[ndma_ch->tx_tail].xcnt * tx_ring[ndma_ch->tx_tail].xmod;
	dma_unmap_single(dev, addr, size, DMA_TO_DEVICE);

	ndma_ch->tx_buffs[ndma_ch->tx_tail] = NULL;
	ndma_ch->tx_tail = (ndma_ch->tx_tail + 1) % NDMA_RING_SIZE;
	ndma_ch->status_cb_fn(skb, port, ts, ndma_ch->status_cb_param);
	ndma_ch->tx_frames_pending--;

	if (!ndma_ch->tx_frames_pending) {
		del_timer(&ndma_ch->tx_timer);

		if (ndma_ch->tx_frames_waiting) {
			adrv906x_dma_tx_prep_desc_list(ndma_ch);
			adrv906x_dma_tx_start(ndma_ch);
		}
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;
}

int adrv906x_ndma_start_xmit(struct adrv906x_ndma_dev *ndma_dev, struct sk_buff *skb,
			     unsigned char port, bool hw_tstamp_en, bool dsa_en)
{
	struct adrv906x_ndma_chan *ndma_ch = &ndma_dev->tx_chan;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	struct device *dev = ndma_dev->dev;
	unsigned long flags;
	unsigned int size;
	dma_addr_t addr;
	u32 wdsize, xmod;
	int ret = 0;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	if (ndma_ch->tx_frames_waiting + ndma_ch->tx_frames_pending >= NDMA_RING_SIZE) {
		ret = -EBUSY;
		goto out;
	}

	if (skb->len < NDMA_MIN_FRAME_SIZE_VALUE)
		skb_put(skb, NDMA_MIN_FRAME_SIZE_VALUE - skb->len);

	adrv906x_ndma_add_tx_header(skb, ndma_ch->seq_num, port, hw_tstamp_en, dsa_en);

	size = ALIGN(skb->len, 8);
	addr = dma_map_single(dev, skb->data, size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, addr))) {
		ret = -EINVAL;
		goto out;
	}

	if (addr % 8 == 0) {
		wdsize = WDSIZE_64;
		xmod = XMODE_64;
	} else if (addr % 4 == 0) {
		wdsize = WDSIZE_32;
		xmod = XMODE_32;
	} else if (addr % 2 == 0) {
		wdsize = WDSIZE_16;
		xmod = XMODE_16;
	} else {
		wdsize = WDSIZE_8;
		xmod = XMODE_8;
	}

	ndma_ch->tx_buffs[ndma_ch->tx_head] = skb;
	ndma_ch->tx_ring[ndma_ch->tx_head].start = addr;
	ndma_ch->tx_ring[ndma_ch->tx_head].xmod = xmod;
	ndma_ch->tx_ring[ndma_ch->tx_head].xcnt = size / xmod;
	ndma_ch->tx_ring[ndma_ch->tx_head].cfg &= ~WDSIZE_MSK;
	ndma_ch->tx_ring[ndma_ch->tx_head].cfg |= wdsize;
	ndma_ch->tx_head = (ndma_ch->tx_head + 1) % NDMA_RING_SIZE;
	ndma_ch->tx_frames_waiting++;
	ndma_ch->seq_num++;
	if (ndma_ch->seq_num == 0)
		ndma_ch->seq_num++;

	if (!ndma_ch->tx_frames_pending) {
		adrv906x_dma_tx_prep_desc_list(ndma_ch);
		adrv906x_dma_tx_start(ndma_ch);
	}

	stats->tx.pending_work_units = ndma_ch->tx_frames_pending;
out:
	spin_unlock_irqrestore(&ndma_ch->lock, flags);
	return ret;
}

static int adrv906x_ndma_tx_status_poll(struct napi_struct *napi, int budget)
{
	struct adrv906x_ndma_chan *ndma_ch = container_of(napi, struct adrv906x_ndma_chan, napi);
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	int count = 0;
	unsigned char *buff;
	dma_addr_t addr, addr_cur, state;
	unsigned long flags;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	while (count < budget) {
		buff = (unsigned char *)ndma_ch->rx_buffs[ndma_ch->rx_tail];
		addr = ndma_ch->rx_ring[ndma_ch->rx_tail].start;

		if (buff[0] == 0)
			break; /* WU not copied */

		/* Clear DMA IRQ status */
		iowrite32(DMA_DONE, ndma_ch->rx_dma_base + DMA_STAT);

		addr_cur = ioread32(ndma_ch->rx_dma_base + DMA_ADDR_CUR);
		state = ioread32(ndma_ch->rx_dma_base + DMA_STAT);
		if (addr_cur >= addr &&
		    addr_cur < addr + NDMA_TX_HDR_STATUS_SIZE &&
		    (DMA_RUN_MASK & state) == DMA_RUN)
			break; /* WU copy in proggress */

		adrv906x_ndma_process_tx_status(ndma_ch, buff);

		buff[0] = 0;
		ndma_ch->rx_tail = (ndma_ch->rx_tail + 1) % NDMA_RING_SIZE;
		count++;
	}
	spin_unlock_irqrestore(&ndma_ch->lock, flags);

	if (count < budget) {
		/* We processed all status available. Tell NAPI it can
		 * stop polling then re-enable rx interrupts.
		 */
		napi_complete_done(napi, count);

		spin_lock_irqsave(&ndma_ch->lock, flags);
		adrv906x_ndma_enable_irqs(ndma_dev, NDMA_TX_STATUS_DMA_DONE_IRQ);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);
	}

	return count;
}

static int adrv906x_ndma_rx_data_and_status_poll(struct napi_struct *napi, int budget)
{
	struct adrv906x_ndma_chan *ndma_ch = container_of(napi, struct adrv906x_ndma_chan, napi);
	struct adrv906x_ndma_dev *ndma_dev = ndma_ch->parent;
	struct device *dev = ndma_dev->dev;
	union adrv906x_ndma_chan_stats *stats = &ndma_ch->stats;
	int count = 0;
	struct sk_buff *skb;
	dma_addr_t addr, addr_cur;
	unsigned int state;
	unsigned long flags;

	spin_lock_irqsave(&ndma_ch->lock, flags);
	while (count < budget) {
		skb = (struct sk_buff *)ndma_ch->rx_buffs[ndma_ch->rx_tail];
		addr = ndma_ch->rx_ring[ndma_ch->rx_tail].start;

		dma_sync_single_for_cpu(dev, addr, NDMA_RX_PKT_BUF_SIZE, DMA_FROM_DEVICE);

		if (skb->data[0] == 0)
			break; /* WU not copied */

		/* Clear DMA IRQ status */
		iowrite32(DMA_DONE, ndma_ch->rx_dma_base + DMA_STAT);

		addr_cur = ioread32(ndma_ch->rx_dma_base + DMA_ADDR_CUR);
		state = ioread32(ndma_ch->rx_dma_base + DMA_STAT);
		if (addr_cur >= addr &&
		    addr_cur < addr + NDMA_RX_PKT_BUF_SIZE &&
		    (DMA_RUN_MASK & state) == DMA_RUN)
			break; /* WU copy in proggress */

		ndma_ch->rx_buffs[ndma_ch->rx_tail] = NULL;
		adrv906x_ndma_process_rx_work_unit(ndma_ch, skb, budget);

		dma_unmap_single(dev, addr, NDMA_RX_PKT_BUF_SIZE, DMA_FROM_DEVICE);

		ndma_ch->rx_tail = (ndma_ch->rx_tail + 1) % NDMA_RING_SIZE;
		ndma_ch->rx_free--;
		count++;
	}

	adrv906x_ndma_refill_rx(ndma_ch, budget);
	if (ndma_ch->rx_tail == 0) /* we reach end of descriptor list */
		adrv906x_dma_rx_start(ndma_ch);

	spin_unlock_irqrestore(&ndma_ch->lock, flags);

	stats->rx.pending_work_units = ndma_ch->rx_free;

	if (count < budget) {
		/* We processed all packets available. Tell NAPI it can
		 * stop polling then re-enable rx interrupts.
		 */
		napi_complete_done(napi, count);

		spin_lock_irqsave(&ndma_ch->lock, flags);
		adrv906x_ndma_enable_irqs(ndma_dev, NDMA_RX_DMA_DONE_IRQ);
		spin_unlock_irqrestore(&ndma_ch->lock, flags);
	}

	return count;
}

int adrv906x_ndma_probe(struct platform_device *pdev, struct net_device *ndev,
			struct device_node *ndma_np, struct adrv906x_ndma_dev *ndma_dev)
{
	struct adrv906x_ndma_chan *rx_chan = &ndma_dev->rx_chan;
	struct adrv906x_ndma_chan *tx_chan = &ndma_dev->tx_chan;
	struct platform_device *ndma_pdev;
	struct device *dev;
	int ret;

	ndma_pdev = of_platform_device_create(ndma_np, NULL, &pdev->dev);
	if (!ndma_pdev) {
		dev_err(dev, "failed to create ndma platform device");
		return -ENODEV;
	}
	dev = &ndma_pdev->dev;
	ndma_dev->dev = dev;

	if (of_property_read_u32(ndma_np, "id", &ndma_dev->dev_num)) {
		dev_err(dev, "failed to retrieve ndma device id from device tree");
		return -EINVAL;
	}

	ret = adrv906x_ndma_device_init(ndma_dev, ndma_np);
	if (ret)
		return ret;
	ret = adrv906x_ndma_get_reset_ctrl(ndma_dev, ndma_np);
	if (ret)
		return ret;
	ret = adrv906x_ndma_get_intr_ctrl(ndma_dev, ndma_np);
	if (ret)
		return ret;
	ret = adrv906x_ndma_alloc_rings(ndma_dev);
	if (ret)
		return ret;

	netif_napi_add(ndev, &rx_chan->napi,
		       adrv906x_ndma_rx_data_and_status_poll, NDMA_NAPI_POLL_WEIGHT);
	netif_napi_add(ndev, &tx_chan->napi,
		       adrv906x_ndma_tx_status_poll, NDMA_NAPI_POLL_WEIGHT);

	return 0;
}

void adrv906x_ndma_remove(struct adrv906x_ndma_dev *ndma_dev)
{
	adrv906x_ndma_chan_disable(&ndma_dev->tx_chan);
	adrv906x_ndma_chan_disable(&ndma_dev->rx_chan);
	of_platform_device_destroy(ndma_dev->dev, NULL);
}
