/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright 2023 NXP
 */

#ifndef NEUTRON_DEVICE_H
#define NEUTRON_DEVICE_H

/****************************************************************************/

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/mutex.h>

#ifdef DEBUG
 #define neu_dbg(fmt, arg...) pr_info("neutron: " fmt, ##arg)
#else
 #define neu_dbg(fmt, arg...)
#endif /* DEBUG */

/****************************************************************************
 * Defines
 ***************************************************************************/
// registers Offset, start STATUSERR not RESETCTRL
#define STATUSERR  0x00
#define INTENA     0x04
#define INTCLR     0x08
#define APPCTRL    0x1FC
#define APPSTATUS  0x200
#define BASEDDRL   0x204
#define BASEDDRH   0x208
#define TAIL       0x234
#define HEAD       0x238

#define MBOX0      0x23C
#define MBOX1      0x240
#define MBOX2      0x244
#define MBOX3      0x248
#define MBOX4      0x24C
#define MBOX5      0x250
#define MBOX6      0x254
#define MBOX7      0x258

#define BASEINOUTL  0x27C
#define BASEINOUTH  0x280
#define BASESPILLL  0x284
#define BASESPILLH  0x288

// values for ctrl firmware mbox (first fw mbox)
#define RUN_ACK   0xA3
#define DM_ACK    0xDA3
#define DONE      0xAD0
#define DM_DONE   0xDAD
#define RESET_VAL 0x0

// values received from SoC (first driver mbox)
#define RUN             0x269
#define CLEAR_FW_LOG    0x270
#define GET_FW_LOGLEVEL 0x271
#define KERNELS         0x272
#define RESET           0x23637
#define DM_TEST         0xD37E57

// Power status
#define NEUTRON_POWER_OFF    0
#define NEUTRON_POWER_ON     1
/****************************************************************************
 * Types
 ****************************************************************************/
struct rproc;
struct clk_bulk_data;

/**
 * struct neutron_log_buffer - Neutron log buffer
 */
struct neutron_log_buffer {
	char *start_addr; /* Start of buffer */
	char *end_addr; /* End of buffer */
	char *end_of_data; /* Current end of data */
	char *last_to_console; /* Last data sent to console */
};

/**
 * struct neutron_device - Device structure
 * @dev:			Common device
 * @reg_base:			Register virtual address
 * @logger:			Log file struct
 * @queue:			Neutron inference queue list pointer
 * @mbox:			Neutron mailbox interface pointer
 * @clks:			Neutron clock pointer
 * @rproc:			Neutron remote-proc pointer
 * @power_state:		Neutron device power state
 */
struct neutron_device {
	struct device                  *dev;
	void __iomem                   *reg_base;
	/* protect the neutron device */
	struct mutex                   mutex;
	int                            irq;
	struct clk_bulk_data           *clks;
	unsigned int                   num_clks;
	struct neutron_log_buffer      logger;
	struct neutron_inference_queue *queue;
	struct neutron_mbox            *mbox;
	struct rproc                   *rproc;
	struct cdev                    cdev;
	struct                         class *class;
	dev_t                          devt;
	unsigned int                   power_state;
};

int neutron_dev_init(struct neutron_device *ndev,
		     struct device *dev, int irq,
		     struct class *class, dev_t devt);

void neutron_dev_deinit(struct neutron_device *ndev);

int neutron_rproc_boot(struct neutron_device *ndev, const char *fw_name);
int neutron_rproc_shutdown(struct neutron_device *ndev);
int neutron_hw_reset(struct neutron_device *ndev);

#endif /* NEUTRON_DEVICE_H */

