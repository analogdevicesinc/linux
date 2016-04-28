#ifndef _MWADMA_H_
#define _MWADMA_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/ktime.h>
#include <linux/sysfs.h>
/* Open firmware includes */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/dma-contiguous.h>
#include <linux/dma/xilinx_dma.h>
#include "mwadma_ioctl.h"  /* IOCTL */

#define DRIVER_NAME "mwipcore"
#define MAX_DEVICES 4
#define MAX_CHANNELS 8

#ifdef _DEBUG
#define MW_DBG_text(txt) printk(KERN_INFO DRIVER_NAME txt)
#define MW_DBG_printf(txt,...) printk(KERN_INFO DRIVER_NAME txt,__VA_ARGS__)
#else
#define MW_DBG_printf(txt,...)
#define MW_DBG_text(txt)
#endif

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
    ready = 0x0, /* default state on init and reset */
    running = 0x1,
    waiting = 0x2, /* waiting on data for tx */
    flushable = 0x3 /* final transfer can be flushed, assumes running */
};


#define	IP2DEV(x)	(x->pdev->dev)

struct mwadma_slist {
    struct list_head                list;
    char                            *buf;
    struct dma_async_tx_descriptor  *desc;
    dma_cookie_t                    cookie;
    struct sg_table                 *sg_t;
    int                             status;
    /* i*ring_length for each ring gives you 
     * offset into large shared buffer */
    unsigned int                    buffer_index; 
};

/* structure contains common parmaters for rx/tx. 
 * Not all params are sensible for both
 */
struct mwadma_chan {
    spinlock_t                  slock;
    struct mutex                lock;
    struct dma_chan             *chan;
    int	                        chan_id;
    size_t                      offset;
    enum dma_ctrl_flags         flags;
    enum dma_transfer_direction direction;
    dma_async_tx_callback       callback;
    char                        *buf;
    enum mwadma_chan_status     status;
    unsigned long               length;
    unsigned long               transfer_count;
    long                        transfer_queued;
    struct mwadma_slist         *scatter;
    struct mwadma_slist         *curr;
    struct mwadma_slist         *completed;
    struct mwadma_slist         *prev;
    struct completion		    dma_complete;
    struct tasklet_struct       tasklet;
    unsigned int                next_index;
    unsigned int                error;
    ktime_t                     start;
    ktime_t                     stop;
    unsigned int                ring_total;
    unsigned int                bd_bytes;
    unsigned int                sg_entries;
    unsigned int                buffer_interrupts;
};

struct mwadma_dev {
    const char 		    	*name;
    struct resource 	    *mem;
    void __iomem 		    *regs;
    struct platform_device 	*pdev;
    struct cdev      		cdev;
    dev_t 			        dev_id;
    int 		        	irq;
    struct fasync_struct 	*asyncq;
    unsigned long           signal_rate;
    dma_addr_t              phys;
    char                    *virt;
    size_t                  size;
    /* Transmit & Receive Channels */
    struct mwadma_chan      *rx;
    struct mwadma_chan      *tx;
};

#endif
