/*
 * MathWorks IP Common Functionality
 *
 * Copyright 2013-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MATHWORKS_IP_H_

#define _MATHWORKS_IP_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/mathworks/mathworks_ip_ioctl.h>

#define MATHWORKS_IP_MAX_DEVTYPE 32
#define	MATHWORKS_IP_DEVNAME_LEN 32

/*********************************************************
* Devm Helpers
*********************************************************/
typedef void (*devm_action_fn)(void *);

static inline int devm_add_action_helper(struct device *dev, void (*action)(void *), void *data){
	int status;
	status = devm_add_action(dev, action, data);
	if(status){
		action(data);
		dev_err(dev,"Failed to allocate memory for devm action\n");
	}
	return status;
}

struct mw_dma_info {
	void				*virt;
	dma_addr_t			phys;
	size_t				size;
};

struct mathworks_ip_dev_info {
	char			devname[MATHWORKS_IP_DEVNAME_LEN];
	dev_t			devid;
	int				devcnt;
};

struct mathworks_ip_info;

struct mathworks_ip_ops {
	void (*get_devname) (struct mathworks_ip_info *thisIpCore, char *devname);
	int	(*get_param) (struct mathworks_ip_info *thisIpCore, void *arg);
	struct file_operations *fops;
};

/* Struct types */
struct mathworks_ip_info {
    const char 			        	*name;
    struct resource 		    	*mem;
    void __iomem 		    		*regs;
    struct device               	*dev;
    struct device					*char_device;
	struct cdev 		        	cdev;
    dev_t 			            	dev_id;
    int 			            	irq;
    int								nirq;
    struct fasync_struct 	    	*asyncq;
    struct module 					*module;
	/*
	 * Bus Specific Ops
	 */
	struct mathworks_ip_ops			*ops;

	struct mathworks_ip_dev_info	*dev_info;

	void 							*private;

	/*
	 * DMA Virtual and physical address
	 */
    struct mw_dma_info	        	dma_info;

/*
 * I2C Controller and EEPROM
 */
    struct i2c_client           	*i2c;
};


/*********************************************************
* API structures
*********************************************************/
extern struct file_operations mathworks_ip_common_fops;

/*********************************************************
* API functions
*********************************************************/
extern int devm_mathworks_ip_register(struct mathworks_ip_info *thisIpcore);

extern struct mathworks_ip_info *devm_mathworks_ip_of_init(
		struct platform_device *pdev,
		struct module *module,
		struct mathworks_ip_ops	*ops,
		bool mapRegs);


/*********************************************************
* API functions
*********************************************************/

static inline void mw_ip_write32(struct mathworks_ip_info *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int mw_ip_read32(struct mathworks_ip_info *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

#endif /* _MATHWORKS_IP_H_ */
