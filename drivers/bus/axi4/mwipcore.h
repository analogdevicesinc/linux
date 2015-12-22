#ifndef _MWIPCORE_H_
#define _MWIPCORE_H_

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
/* ### ADI SPECIFIC ### */
#include <linux/spi/spi.h>
/* Open firmware includes */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/kfifo.h>


#ifdef _DEBUG
#define MW_DBG_text(txt) printk(KERN_INFO DRIVER_NAME txt)
#define MW_DBG_printf(txt,...) printk(KERN_INFO DRIVER_NAME txt,__VA_ARGS__)
#else
#define MW_DBG_printf(txt,...)
#define MW_DBG_text(txt)
#endif


struct ipcore_info {
    const char 			*name;
    struct resource 		*mem;
    void __iomem 		*regs;
    struct platform_device 	*pdev;
    struct cdev 		cdev;
    dev_t 			dev_id;
    int 			irq;
    struct fasync_struct 	*asyncq;
    struct mutex 		lock;
/*
 * FIFO and DMA structures
 */ 
/*################ Receive #####################*/   
    unsigned int		rx_length;
    struct dma_chan		*rx_chan;
/*################ Transmit #####################*/   
    unsigned int		tx_length;
    struct dma_chan		*tx_chan;
/*################ SHARED #####################*/   
    int				done_status;
    struct completion		dma_complete;
/*
 * DMA Virtual and physical address
 */
    void			*buf_virt;
    dma_addr_t			buf_phys;
/* 
 *###ADI_ZED_SDR###
 */
    struct device 		*dev_spi;
    unsigned			id;
    unsigned long		adc_clk;
    unsigned char		testmode[2];
    unsigned                    pcore_version;
    unsigned                    have_user_logic;
    unsigned                    max_count;
};


struct ipcore_dma_params {
    struct device_node			*of_node;
    enum dma_transfer_direction	        direction;
    int					chan_id;
};


void mwipcore_reg_write(struct ipcore_info *thisInfo, unsigned reg, unsigned val);

unsigned int mwipcore_reg_read(struct ipcore_info *thisInfo, unsigned reg);


#endif
