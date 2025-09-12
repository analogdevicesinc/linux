/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices SC5XX SPORT driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef _SC5XX_SPORT_H_
#define _SC5XX_SPORT_H_

#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <sound/pcm_params.h>

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)
#include <linux/workqueue.h>
#include <linux/rpmsg.h>
#include "icap/include/icap_application.h"
#endif

#define TDM_MAX_SLOTS 8
#define SHARC_CORES_NUM 2

#define SPORT_CTL_SPENPRI        0x00000001 /* Enable Primary Channel */

#define SPORT_CTL_DTYPE          0x00000006 /* Data type select */
#define SPORT_CTL_RJUSTIFY_ZFILL 0x00000000 /*   MCM mode: Right-just, zero-fill unused MSBs */
#define SPORT_CTL_RJUSTIFY_SFILL 0x00000002 /*   MCM mode: Right-just, sign-extend unused MSBs */
#define SPORT_CTL_USE_U_LAW      0x00000004 /*   MCM mode: Compand using u-law */
#define SPORT_CTL_USE_A_LAW      0x00000006 /*   MCM mode: Compand using A-law */

#define SPORT_CTL_LSBF           0x00000008 /* Serial bit endian select */
#define SPORT_CTL_SLEN           0x000001F0 /* Serial Word length select */
#define SPORT_CTL_PACK           0x00000200 /* 16-bit to 32-bit packing enable */
#define SPORT_CTL_ICLK           0x00000400 /* Internal Clock Select */
#define SPORT_CTL_OPMODE         0x00000800 /* Operation mode */
#define SPORT_CTL_CKRE           0x00001000 /* Clock rising edge select */
#define SPORT_CTL_FSR            0x00002000 /* Frame Sync required */
#define SPORT_CTL_IFS            0x00004000 /* Internal Frame Sync select */
#define SPORT_CTL_DIFS           0x00008000 /* Data-independent frame sync select */
#define SPORT_CTL_LFS            0x00010000 /* Active low frame sync select */
#define SPORT_CTL_LAFS           0x00020000 /* Late Transmit frame select */
#define SPORT_CTL_RJUST          0x00040000 /* Right Justified mode select */
#define SPORT_CTL_FSED           0x00080000 /* External frame sync edge select */
#define SPORT_CTL_TFIEN          0x00100000 /* Transmit finish interrupt enable select */
#define SPORT_CTL_GCLKEN         0x00200000 /* Gated clock mode select */
#define SPORT_CTL_SPENSEC        0x01000000 /* Enable secondary channel */
#define SPORT_CTL_SPTRAN         0x02000000 /* Data direction control */
#define SPORT_CTL_DERRSEC        0x04000000 /* Secondary channel error status */
#define SPORT_CTL_DXSSEC         0x18000000 /* Secondary channel data buffer status */
#define SPORT_CTL_SEC_EMPTY      0x00000000 /* DXSSEC: Empty */
#define SPORT_CTL_SEC_PART_FULL  0x10000000 /* DXSSEC: Partially full */
#define SPORT_CTL_SEC_FULL       0x18000000 /* DXSSEC: Full */
#define SPORT_CTL_DERRPRI        0x20000000 /* Primary channel error status */
#define SPORT_CTL_DXSPRI         0xC0000000 /* Primary channel data buffer status */
#define SPORT_CTL_PRM_EMPTY      0x00000000 /* DXSPRI: Empty */
#define SPORT_CTL_PRM_PART_FULL  0x80000000 /* DXSPRI: Partially full */
#define SPORT_CTL_PRM_FULL       0xC0000000 /* DXSPRI: Full */

#define SPORT_DIV_CLKDIV         0x0000FFFF /* Clock divisor */
#define SPORT_DIV_FSDIV          0xFFFF0000 /* Frame sync divisor */

#define SPORT_MCTL_MCE           0x00000001 /* Multichannel enable */
#define SPORT_MCTL_MCPDE         0x00000004 /* Multichannel data packing select */
#define SPORT_MCTL_MFD           0x000000F0 /* Multichannel frame delay */
#define SPORT_MCTL_WSIZE         0x00007F00 /* Number of multichannel slots */
#define SPORT_MCTL_WOFFSET       0x03FF0000 /* Window offset size */

#define SPORT_CNT_CLKCNT         0x0000FFFF /* Current state of clk div counter */
#define SPORT_CNT_FSDIVCNT       0xFFFF0000 /* Current state of frame div counter */

#define SPORT_ERR_DERRPMSK       0x00000001 /* Primary channel data error interrupt enable */
#define SPORT_ERR_DERRSMSK       0x00000002 /* Secondary channel data error interrupt enable */
#define SPORT_ERR_FSERRMSK       0x00000004 /* Frame sync error interrupt enable */
#define SPORT_ERR_DERRPSTAT      0x00000010 /* Primary channel data error status */
#define SPORT_ERR_DERRSSTAT      0x00000020 /* Secondary channel data error status */
#define SPORT_ERR_FSERRSTAT      0x00000040 /* Frame sync error status */

#define SPORT_MSTAT_CURCHAN      0x000003FF /* Current channel */

#define SPORT_CTL2_FSMUXSEL      0x00000001 /* Frame Sync MUX Select */
#define SPORT_CTL2_CKMUXSEL      0x00000002 /* Clock MUX Select */
#define SPORT_CTL2_LBSEL         0x00000004 /* Loopback Select */

struct sport_register {
	u32 spctl;
	u32 div;
	u32 spmctl;
	u32 spcs0;
	u32 spcs1;
	u32 spcs2;
	u32 spcs3;
	u32 spcnt;
	u32 sperrctl;
	u32 spmstat;
	u32 spctl2;
	u32 txa;
	u32 rxa;
	u32 txb;
	u32 rxb;
	u32 revid;
};

struct sport_device {
	struct platform_device *pdev;
	struct clk *clk;
	const unsigned short *pin_req;
	struct sport_register *tx_regs;
	struct sport_register *rx_regs;
	struct dma_chan *tx_dma_chan;
	struct dma_chan *rx_dma_chan;
	int tx_err_irq;
	int rx_err_irq;

	void (*tx_callback)(void *data);
	void *tx_data;
	void (*rx_callback)(void *data);
	void *rx_data;

	/* cpu address of dma descriptor */
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;
	dma_addr_t tx_buf;
	dma_addr_t rx_buf;
	size_t tx_fragsize;
	size_t rx_fragsize;
	unsigned int tx_frags;
	unsigned int rx_frags;
	u32 tx_count;
	u32 rx_count;
	dma_cookie_t tx_cookie;
	dma_cookie_t rx_cookie;
	size_t tx_totalsize;
	size_t rx_totalsize;
	unsigned int wdsize;

	unsigned int tx_map[TDM_MAX_SLOTS];
	unsigned int rx_map[TDM_MAX_SLOTS];

	struct snd_pcm_substream *tx_substream;
	struct snd_pcm_substream *rx_substream;

	struct snd_pcm_hw_params tx_hw_params;
	struct snd_pcm_hw_params rx_hw_params;

	unsigned int dai_format;

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)

	struct icap_instance icap[SHARC_CORES_NUM];
	spinlock_t icap_spinlock;

	u32 sharc_tx_core;
	u32 sharc_rx_core;

	u32 icap_tx_dev_id;
	u32 icap_rx_dev_id;

	struct work_struct get_sharc1_feature_work;
	struct work_struct get_sharc2_feature_work;

	struct snd_dma_buffer sharc_tx_dma_buf;
	struct snd_dma_buffer sharc_rx_dma_buf;

	size_t sharc_tx_buf_pos;
	size_t sharc_rx_buf_pos;
	spinlock_t sharc_tx_buf_pos_lock;
	spinlock_t sharc_rx_buf_pos_lock;

	u32 tx_alsa_icap_buf_id;
	u32 tx_dma_icap_buf_id;
	u32 rx_alsa_icap_buf_id;
	u32 rx_dma_icap_buf_id;

	struct work_struct send_tx_start_work;
	struct work_struct send_rx_start_work;

	struct work_struct send_tx_stop_work;
	struct work_struct send_rx_stop_work;

	struct wait_queue_head pending_tx_stop_event;
	struct wait_queue_head pending_rx_stop_event;

	u32 pending_tx_stop;
	u32 pending_rx_stop;
#endif
};

struct sport_params {
	u32 spctl;
	u32 div;
	u32 spmctl;
	u32 spcs0;
};

struct sport_device *sport_create(struct platform_device *pdev);
void sport_delete(struct sport_device *sport);
int sport_set_tx_params(struct sport_device *sport,
		struct sport_params *params);
int sport_set_rx_params(struct sport_device *sport,
		struct sport_params *params);
int sport_tx_start(struct sport_device *sport);
int sport_rx_start(struct sport_device *sport);
int sport_tx_stop(struct sport_device *sport);
int sport_rx_stop(struct sport_device *sport);
void sport_tx_dma_callback(void *ptr);
void sport_rx_dma_callback(void *ptr);
void sport_set_tx_callback(struct sport_device *sport,
	void (*tx_callback)(void *), void *tx_data);
void sport_set_rx_callback(struct sport_device *sport,
	void (*rx_callback)(void *), void *rx_data);
int sport_config_tx_dma(struct sport_device *sport, void *buf,
	int fragcount, size_t fragsize, struct snd_pcm_substream *substream);
int sport_config_rx_dma(struct sport_device *sport, void *buf,
	int fragcount, size_t fragsize, struct snd_pcm_substream *substream);
unsigned long sport_curr_offset_tx(struct sport_device *sport);
unsigned long sport_curr_offset_rx(struct sport_device *sport);

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)
int rpmsg_icap_sport_probe(struct rpmsg_device *rpdev);
int rpmsg_icap_sport_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src);
void rpmsg_icap_sport_remove(struct rpmsg_device *rpdev);
#endif

#endif
