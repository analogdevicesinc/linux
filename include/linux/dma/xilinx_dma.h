/*
 * Xilinx DMA Engine drivers support header file
 *
 * Copyright (C) 2010-2014 Xilinx, Inc. All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __DMA_XILINX_DMA_H
#define __DMA_XILINX_DMA_H

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

/* DMA IP masks */
#define XILINX_DMA_IP_DMA	0x00100000	/* A DMA IP */
#define XILINX_DMA_IP_CDMA	0x00200000	/* A Central DMA IP */

/* Device Id in the private structure */
#define XILINX_DMA_DEVICE_ID_SHIFT	28

/* Specific hardware configuration-related constants
 */
#define XILINX_DMA_NO_CHANGE             0xFFFF;

/* DMA IP masks 
 */
#define XILINX_DMA_IP_DMA              0x00100000 /* A DMA IP */
#define XILINX_DMA_IP_CDMA             0x00200000 /* A Central DMA IP */
#define XILINX_DMA_IP_VDMA             0x00400000 /* A Video DMA IP */
#define XILINX_DMA_IP_MASK             0x00700000 /* DMA IP MASK */

/* shared by all Xilinx DMA engines
 */
/* Device configuration structure
 *
 * Xilinx CDMA and Xilinx DMA only use interrupt coalescing and delay counter
 * settings.
 *
 * If used to start/stop parking mode for Xilinx VDMA, vsize must be -1
 * If used to set interrupt coalescing and delay counter only for
 * Xilinx VDMA, hsize must be -1 */
struct xilinx_dma_config {
	enum dma_transfer_direction direction; /* Channel direction */
	int vsize;                         /* Vertical size */
	int hsize;                         /* Horizontal size */
	int stride;                        /* Stride */
	int frm_dly;                       /* Frame delay */
	int gen_lock;                      /* Whether in gen-lock mode */
	int master;                        /* Master that it syncs to */
	int frm_cnt_en;                    /* Enable frame count enable */
	int park;                          /* Whether wants to park */
	int park_frm;                      /* Frame to park on */
	int coalesc;                       /* Interrupt coalescing threshold */
	int delay;                         /* Delay counter */
	int disable_intr;                  /* Whether use interrupts */
	int reset;			   /* Reset Channel */
	int ext_fsync;			   /* External Frame Sync */
};

/* Platform data definition until ARM supports device tree */

struct dma_channel_config {
	char *type;	
	unsigned int lite_mode;       /* cdma only */
	unsigned int include_dre;
	unsigned int genlock_mode;    /* vdma only */
	unsigned int datawidth;
	unsigned int max_burst_len;
};

struct dma_device_config {
	char *type;	
	unsigned int include_sg;
	unsigned int num_fstores;    /* vdma only */
	unsigned int sg_include_stscntrl_strm;  /* dma only */
	unsigned int channel_count;
	struct dma_channel_config *channel_config;
};

/**
 * struct xilinx_vdma_config - VDMA Configuration structure
 * @frm_dly: Frame delay
 * @gen_lock: Whether in gen-lock mode
 * @master: Master that it syncs to
 * @frm_cnt_en: Enable frame count enable
 * @park: Whether wants to park
 * @park_frm: Frame to park on
 * @coalesc: Interrupt coalescing threshold
 * @delay: Delay counter
 * @reset: Reset Channel
 * @ext_fsync: External Frame Sync source
 */
struct xilinx_vdma_config {
	int frm_dly;
	int gen_lock;
	int master;
	int frm_cnt_en;
	int park;
	int park_frm;
	int coalesc;
	int delay;
	int reset;
	int ext_fsync;
};

/**
 * struct xilinx_cdma_config - CDMA Configuration structure
 * @coalesc: Interrupt coalescing threshold
 * @delay: Delay counter
 * @reset: Reset Channel
 */
struct xilinx_cdma_config {
        int coalesc;
        int delay;
        int reset;
};

/**
 * struct xilinx_mcdma_config - DMA Multi channel configuration structure
 * @tdest: Channel to operate on
 * @tid:   Channel configuration
 * @tuser: Tuser configuration
 * @ax_user: ax_user value
 * @ax_cache: ax_cache value
 */
struct xilinx_mcdma_config {
	u8 tdest;
	u8 tid;
	u8 tuser;
	u8 ax_user;
	u8 ax_cache;
};
int xilinx_vdma_channel_set_config(struct dma_chan *dchan,
					struct xilinx_vdma_config *cfg);
int xilinx_cdma_channel_set_config(struct dma_chan *dchan,
                                        struct xilinx_cdma_config *cfg);
int xilinx_dma_channel_mcdma_set_config(struct dma_chan *dchan,
					struct xilinx_mcdma_config *cfg);
#endif
