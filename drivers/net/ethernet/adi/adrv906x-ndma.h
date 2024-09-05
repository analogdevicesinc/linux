// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_NDMA_H__
#define __ADRV906X_NDMA_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>

#define NDMA_MAX_FRAME_SIZE_VALUE 9038
#if IS_ENABLED(CONFIG_MACSEC)
#define NDMA_MIN_FRAME_SIZE_VALUE 42
#else
#define NDMA_MIN_FRAME_SIZE_VALUE (64 - ETH_FCS_LEN)
#endif // IS_ENABLED(CONFIG_MACSEC)

#define NDMA_PTP_MODE_1 0
#define NDMA_PTP_MODE_4 1
#define NDMA_HDR_TYPE_MASK                 GENMASK(1, 0)

/* NDMA TX header */
#define NDMA_TX_HDR_TYPE_SOF               BIT(0)
#define NDMA_TX_HDR_TYPE_SUBSEQ            BIT(1)
#define NDMA_TX_HDR_TYPE_LOOPBACK          GENMASK(1, 0)
#define NDMA_TX_HDR_TYPE_STATUS            GENMASK(1, 0)
#define NDMA_TX_HDR_SOF_FR_PTP             BIT(2)
#define NDMA_TX_HDR_SOF_PORT_ID            BIT(3)
#define NDMA_TX_HDR_SOF_DSA_EN             BIT(4)
#define NDMA_TX_HDR_STATUS_FR_ERR          BIT(2)
#define NDMA_TX_HDR_STATUS_FR_PTP          BIT(3)
#define NDMA_TX_HDR_SOF_SIZE               8
#define NDMA_TX_HDR_SUBSEQ_SIZE            8
#define NDMA_TX_HDR_STATUS_SIZE            16
#define NDMA_TX_HDR_LOOPBACK_SIZE          16

/* NDMA RX header */
#define NDMA_RX_HDR_TYPE_DATA_SOF          BIT(2)
#define NDMA_RX_HDR_TYPE_DATA              BIT(0)
#define NDMA_RX_HDR_TYPE_STATUS            BIT(1)
#define NDMA_RX_HDR_STATUS_FR_ERR          BIT(2)
#define NDMA_RX_HDR_STATUS_PORT_ID         BIT(3)
#define NDMA_RX_HDR_STATUS_FR_DROP_ERR     BIT(4)
#define NDMA_RX_HDR_DATA_SIZE              2
#define NDMA_RX_HDR_STATUS_SIZE            16

#define NDMA_RX_FRAME_LEN_MSB              (NDMA_RX_HDR_STATUS_SIZE - 1)
#define NDMA_RX_FRAME_LEN_LSB              (NDMA_RX_HDR_STATUS_SIZE - 2)
#define NDMA_RX_PKT_BUF_SIZE               ALIGN(1536 + NDMA_RX_HDR_DATA_SIZE + NET_IP_ALIGN, 8)

#define NDMA_NAPI_POLL_WEIGHT              64
#define NDMA_RING_SIZE                     128

/* default timestamp timeout delay */
#define NDMA_TS_TX_DELAY 0x9975

enum adrv906x_ndma_chan_type {
	NDMA_TX,
	NDMA_RX,
	MAX_NDMA_CHANNELS
};

typedef void (*ndma_callback)(struct sk_buff *skb, unsigned int port_id,
			      struct timespec64 ts, void *cb_param);

struct dma_desc {
	u32 next;
	u32 start;
	u32 cfg;
	u32 xcnt;
	u32 xmod;
};

struct adrv906x_ndma_reset {
	void __iomem *reg;
	unsigned int rx_chan_reset_bit;
	unsigned int tx_chan_reset_bit;
};

union adrv906x_ndma_chan_stats {
	struct {
		u64 frame_size_errors;
		u64 data_header_errors;
		u64 status_header_errors;
		u64 tstamp_timeout_errors;
		u64 seqnumb_mismatch_errors;
		u64 unknown_errors;
		u64 pending_work_units;
		u64 done_work_units;
		u64 status_dma_errors;
		u64 data_dma_errors;
	} tx;
	struct {
		u64 frame_errors;
		u64 frame_size_errors;
		u64 frame_dropped_errors;
		u64 frame_dropped_splane_errors;
		u64 frame_dropped_mplane_errors;
		u64 seqnumb_mismatch_errors;
		u64 status_header_errors;
		u64 unknown_errors;
		u64 pending_work_units;
		u64 done_work_units;
		u64 dma_errors;
	} rx;
};

struct adrv906x_ndma_chan {
	struct adrv906x_ndma_dev *parent;
	enum adrv906x_ndma_chan_type chan_type;
	char *chan_name;
	void __iomem *ctrl_base;
	union adrv906x_ndma_chan_stats stats;
	ndma_callback status_cb_fn;
	void *status_cb_param;
	spinlock_t lock; /* protects struct and register access */
	unsigned char expected_seq_num;
	unsigned char seq_num;

	/* TX DMA channel related fields */
	void __iomem *tx_dma_base;
	void *tx_buffs[NDMA_RING_SIZE];
	char tx_loopback_wu[NDMA_TX_HDR_LOOPBACK_SIZE];
	struct dma_desc tx_loopback_desc;
	dma_addr_t tx_loopback_addr;
	int tx_dma_done_irq;
	int tx_dma_error_irq;
	struct dma_desc *tx_ring;
	dma_addr_t tx_ring_dma;
	unsigned int tx_tail;   /* Next entry in tx ring to read */
	unsigned int tx_head;   /* Next entry in tx ring to give a new buffer */
	unsigned int tx_frames_waiting;
	unsigned int tx_frames_pending;
	struct timer_list tx_timer;

	/* RX DMA channel related fields */
	void __iomem *rx_dma_base;
	struct sk_buff *skb_rx_data_wu;
	void *rx_buffs[NDMA_RING_SIZE];
	int rx_dma_done_irq;
	int rx_dma_error_irq;
	struct dma_desc *rx_ring;
	dma_addr_t rx_ring_dma;
	unsigned int rx_tail;   /* Next entry in rx ring to read */
	unsigned int rx_head;   /* Next entry in rx ring to give a new buffer */
	unsigned int rx_free;   /* Number of free RX buffers */
	struct napi_struct napi;
	unsigned int rx_data_fragments;
};

struct adrv906x_ndma_dev {
	struct device *dev;
	unsigned int dev_num;
	struct adrv906x_ndma_chan rx_chan;
	struct adrv906x_ndma_chan tx_chan;
	struct adrv906x_ndma_reset reset;
	void __iomem *intr_ctrl;
	struct delayed_work update_stats;
	bool enabled;
	struct kref refcount;
	spinlock_t lock; /* protects struct and stats access */
	bool loopback_en;
};

int adrv906x_ndma_start_xmit(struct adrv906x_ndma_dev *ndma_dev, struct sk_buff *skb,
			     unsigned char port, bool hw_tstamp_en, bool dsa_en);
int adrv906x_ndma_probe(struct platform_device *pdev, struct net_device *ndev,
			struct device_node *ndma_np, struct adrv906x_ndma_dev *ndma_dev);
void adrv906x_ndma_remove(struct adrv906x_ndma_dev *ndma_dev);
void adrv906x_ndma_set_ptp_mode(struct adrv906x_ndma_dev *ndma_dev, u32 ptp_mode);
void adrv906x_ndma_open(struct adrv906x_ndma_dev *ndma_dev, ndma_callback tx_cb_fn,
			ndma_callback rx_cb_fn, void *rx_cb_param, bool loopback_mode);
void adrv906x_ndma_close(struct adrv906x_ndma_dev *ndma_dev);

#endif /* __ADRV906X_NDMA_H__ */
