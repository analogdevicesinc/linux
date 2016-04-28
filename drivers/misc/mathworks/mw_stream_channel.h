/*
 * MathWorks Streaming Channel
 *
 * Copyright 2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MW_STREAM_CHANNEL_H_
#define _MW_STREAM_CHANNEL_H_

#include <linux/dmaengine.h>
#include <linux/dma-contiguous.h>
#include <linux/interrupt.h>
#include <linux/errno.h>

#include "mathworks_ipcore.h"
#include "mwadma_ioctl.h"  /* IOCTL */

enum DESCRIPTOR_STATUS {
    BD_UNALLOC = -1,
    BD_ALLOC = 0,
    BD_MAPPED,
    BD_ISSUED,
    BD_PROCESS,
    BD_PROCESSED,
    BD_MAX_DESCRIPTOR_STATUS
};


#define ERR_RING_OVERFLOW 0x1

enum mwadma_chan_status {
    ready = 0x0,      /* default state on init and reset */
    running = 0x1,
    waiting = 0x2    /* waiting on data for tx */
};

enum mwadma_buffer_block_state {
    MWDMA_ACTIVE = 0x1, 
    MWDMA_PENDING = 0x2,
    MWDMA_READY = 0x3 
};

// BLOCK
struct mwadma_slist {
    struct list_head    list;
    struct list_head    userid;
    dma_addr_t		phys;
    size_t		length;
    struct dma_async_tx_descriptor  *desc;
    dma_cookie_t                    cookie;
    unsigned int                    buffer_index;
    enum mwadma_buffer_block_state state;
    struct mwadma_chan             *qchan;
};

/* structure contains common parmaters for rx/tx.
 * Not all params are sensible for both
 */
struct mwadma_chan;

struct mwadma_dev {
    struct fasync_struct 	*asyncq;
    struct mathworks_ipcore_dev *mw_ipcore_dev;
    /* Transmit & Receive Channels */
    struct mwadma_chan      *rx;
    struct mwadma_chan      *tx;
    unsigned int 	    channel_offset;
};
// QUEUE
struct mwadma_chan {
    struct device		dev;
    struct mwadma_dev 		*mwdev;
    struct kernfs_node	        *irq_kn;
    spinlock_t                  slock;
    struct mutex                lock;
    struct dma_chan             *chan;
    int	                        chan_id;
    size_t                      offset;
    enum dma_ctrl_flags         flags;
    enum dma_transfer_direction direction;
    dma_async_tx_callback       callback;
    char                        *buf;
    dma_addr_t                  phys;
    enum mwadma_chan_status     status;
    unsigned long               length;
    unsigned long               transfer_count;
    long                        transfer_queued;
    struct mwadma_slist         *scatter;
    struct mwadma_slist         *curr;
    struct mwadma_slist         *completed;
    struct mwadma_slist         *prev;
    struct completion		dma_complete;
    struct tasklet_struct       tasklet;
    unsigned int                next_index;
    unsigned int                error;
    ktime_t                     start;
    ktime_t                     stop;
    unsigned int                ring_total;
    unsigned int                bd_bytes;
    struct mwadma_slist         **blocks;
};


/*********************************************************
* API functions
*********************************************************/
#if defined(CONFIG_MWIPCORE_DMA_STREAMING) || defined(CONFIG_MWIPCORE_DMA_STREAMING_MODULE)
extern struct mathworks_ip_ops* mw_stream_channel_get_ops(void);
extern int mw_stream_channels_probe(struct mathworks_ipcore_dev		*mw_ipcore_dev);
#else
static inline struct mathworks_ip_ops* mw_stream_channel_get_ops(void) {
	return NULL;
}
static inline int mw_stream_channels_probe(struct mathworks_ipcore_dev	*mw_ipcore_dev) {
	return -ENODEV;
}
#endif

#endif /* _MW_STREAM_CHANNEL_H_ */
