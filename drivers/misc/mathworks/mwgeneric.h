#ifndef _MWGENERIC_H_
#define _MWGENERIC_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include "mwgeneric_ioctl.h"

#define MWGENERIC_MEMTYPE_NORMAL		0
#define MWGENERIC_MEMTYPE_NOMEM			1
#define MWGENERIC_MEMTYPE_NOMEM_STR		"nomem"

#define MWGENERIC_MAX_DEVTYPE 32
#define	MWGENERIC_DEVNAME_LEN 32


struct mw_dma_info {
	void				*virt;
	dma_addr_t			phys;
	size_t				size;
};

struct mwgeneric_info {
	char			devname[MWGENERIC_DEVNAME_LEN];
	dev_t			devid;
	int				devcnt;
};

struct ipcore_info;

struct mw_generic_ops {
	void (*get_devname) (struct ipcore_info *thisIpCore, char *devname);
	int	(*get_param) (struct ipcore_info *thisIpCore, void *arg);
};

/* Struct types */
struct ipcore_info {
    const char 			        *name;
    struct resource 		    *mem;
    struct device               *dev;
    struct device				*char_device;
	struct cdev 		        cdev;
    dev_t 			            dev_id;
    int 			            irq;
    int							nirq;
    struct fasync_struct 	    *asyncq;

	/*
	 * Bus Specific Ops
	 */
	struct mw_generic_ops		*ops;

	struct mwgeneric_info		*dev_info;

	/*
	 * DMA Virtual and physical address
	 */
    struct mw_dma_info	        dma_info;

/*
 * I2C Controller and EEPROM
 */
    struct i2c_client           *i2c;
};

/*********************************************************
* API functions
*********************************************************/
extern int devm_mwgeneric_register(struct ipcore_info *thisIpcore);

#endif
