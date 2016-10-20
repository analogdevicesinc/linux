/*
 * MathWorks AXI DMA Driver
 *
 * Copyright 2014-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MATHWORKS_IPCORE_H_
#define _MATHWORKS_IPCORE_H_

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

#include <linux/mathworks/mathworks_ip.h>

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

enum mw_stream_mode {
	MWSTREAM_MODE_LEGACY,
	MWSTREAM_MODE_SUBDEV,
	MWSTREAM_MODE_NONE,
};

struct mw_dev_info {
	enum mw_stream_mode	stream_mode;
};

struct mathworks_ipcore_dev {
    struct mathworks_ip_info 		*mw_ip_info;
    u32								rst_reg;
    const struct mw_dev_info		*info;
    void							*private;
};

/*********************************************************
* API functions
*********************************************************/

static inline void mw_ip_reset(struct mathworks_ipcore_dev *mwdev)
{
	mw_ip_write32(mwdev->mw_ip_info, mwdev->rst_reg, 0x1);
}

#endif /* _MATHWORKS_IPCORE_H_ */
