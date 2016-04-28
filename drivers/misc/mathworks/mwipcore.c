/*
 * Copyright 2013 MathWorks, Inc.
 * 
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/of_irq.h>
#include <linux/debugfs.h>
#include <linux/init.h>

#include <linux/mm.h>

#include "mwipcore.h"
#include "mwipcore_ioctl.h"

#define DRIVER_NAME "mwipcore"
#define MWIPCORE_INTERRUPT_OFFSET 0x108

/*Device structure for IPCore information*/
dev_t mwipcore_dev_id = 0;
static unsigned int device_num = 0;
static unsigned int cur_minor = 0;
static struct class *mwipcore_class = NULL;
static unsigned int g_mw_intr_offset = 0x108;

static int mwipcore_fasync_impl(int fd, struct file* fp, int mode)
{
    struct ipcore_info *thisIpcore = fp->private_data;
    
    return fasync_helper(fd, fp, mode, &thisIpcore->asyncq);
  
}

static int mwipcore_open(struct inode *inode, struct file *fp)
{
    struct ipcore_info *thisIpcore;
    thisIpcore = container_of(inode->i_cdev, struct ipcore_info, cdev);
    fp->private_data = thisIpcore;
    
    return 0;
}

static int mwipcore_close(struct inode *inode, struct file *fp)
{
    mwipcore_fasync_impl(-1, fp, 0);
    return 0;
}

static irqreturn_t mwipcore_intr_handler(int irq, void * theIpcore)
{
    struct ipcore_info *thisIpcore = (struct ipcore_info*) theIpcore;
    /*  */ 
    writel(1,thisIpcore->regs + g_mw_intr_offset);
    writel(0,thisIpcore->regs + g_mw_intr_offset);
    if (thisIpcore->asyncq)
    {
        kill_fasync(&thisIpcore->asyncq, SIGIO, POLL_IN);
    }
    /*MW_DBG_printf("IRQ Handled:in %s at %d\n", __func__, __LINE__);*/
    return IRQ_HANDLED;

}

static int mwipcore_writeto_devicereg(struct ipcore_info *thisIpcore, struct raw_access_data * user_data)
{
    if ( (user_data == NULL) || (user_data->data_buf == NULL) ) 
    {
        dev_err(&thisIpcore->pdev->dev, "Write to mwipcore:ERROR: buffer address is NULL: 0x%08x\n", (unsigned int)user_data);
        return -EINVAL;
    }
   /* 
    * MW_DBG_printf(":In:%s:data_buf:%p, data_len = %u\n", __func__, user_data->data_buf, user_data->data_length); 
    */
    memcpy_toio((void *)(thisIpcore->regs + user_data->offset_addr), \
	(void *) user_data->data_buf, \
	user_data->data_length * (sizeof(unsigned int)));

    return 0;
}

static int mwipcore_readfrom_devicereg(struct ipcore_info *thisIpcore, struct raw_access_data * user_data)
{
    if ( (user_data== NULL) || (user_data->data_buf == NULL) ) 
    {
        dev_err(&thisIpcore->pdev->dev, "Read from buffer:ERROR: buffer address is NULL: 0x%08x\n", (unsigned int)user_data);
        return -EINVAL;
    }

    /* MW_DBG_printf(":In:%s:data_buf:%p, data_len = %u\n", __func__, user_data->data_buf, user_data->data_length); */
    memcpy_fromio((void *) user_data->data_buf, (void *)(thisIpcore->regs + user_data->offset_addr), user_data->data_length * (sizeof(unsigned int)));

    return 0;
}


static void mwipcore_mmap_open(struct vm_area_struct *vma)
{
    printk(KERN_NOTICE "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwipcore_mmap_close(struct vm_area_struct *vma)
{
    printk(KERN_NOTICE "Simple VMA close.\n");
}

static int mwipcore_mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    struct ipcore_info * thisInfo = vma->vm_private_data;
    struct page *thisPage;
    unsigned long offset;
    offset = (vmf->pgoff - vma->vm_pgoff) << PAGE_SHIFT;
    thisPage = virt_to_page(thisInfo->mem->start + offset);
    get_page(thisPage);
    vmf->page = thisPage;
    return 0;
}
struct vm_operations_struct mwipcore_mmap_ops = {
    .open   = mwipcore_mmap_open,
    .close  = mwipcore_mmap_close,
    .fault = mwipcore_mmap_fault,
}; 

int mwipcore_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct ipcore_info *thisInfo = fp->private_data;
    vma->vm_private_data = thisInfo;
 
    vma->vm_flags |= VM_IO | VM_DONTDUMP | VM_DONTDUMP; // may be redundant with call to remap_pfn_range below
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    
    if (remap_pfn_range(vma, vma->vm_start,
			thisInfo->mem->start >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot))
    {
        return -EAGAIN;
    }
    return 0;
}

static long mwipcore_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    /* struct ipcore_info *thisInfo = fp->private_data; */
    struct ipcore_info *thisInfo = fp->private_data;
    struct raw_access_data data;
    
    if (NULL==thisInfo)
    {
        return -ENODEV;
    }

    switch(cmd)
    {
	case MWIPCORE_REGISTER_READ:
    	/* read register involves 3 steps
	 * read from the user-argument, copy it to data
	 *    data.offset_addr gives the offset of the register
	 *    data.data_length gives the amount of unsigned long chunks to be read
	 * Copy the register values into data.data_buf
         * Copy to user the data again
         */     
		if( copy_from_user(&data, (struct raw_access_data *)arg, sizeof(struct raw_access_data)) )
		{
		    return -EACCES;
		}
		if (mwipcore_readfrom_devicereg(thisInfo, &data))
		{
		    printk(KERN_ERR ":Error while reading from device registers\n");
		    return -EINVAL;
		}
		if( copy_to_user((struct raw_access_data *)arg, &data, sizeof(struct raw_access_data)) )
		{
		    return -EACCES;
		}
		break;
	case MWIPCORE_REGISTER_WRITE:
    	/* write register involves 2 steps
	 * copy from the user-argument, copy data from it
	 *    data.offset_addr gives the offset of the register
	 *    data.data_length gives the amount of unsigned long chunks to be read
	 *    data.data_buf contains the new value 
	 * Write the register values from data.data_buf
         */
       		if( copy_from_user(&data, (struct raw_access_data *)arg, sizeof(struct raw_access_data)) )
		{
		    return -EACCES;
		}
                if(mwipcore_writeto_devicereg(thisInfo, &data))
		{
		    printk(KERN_ERR ":Error while writing to the device-registers\n");
		    return -EINVAL;
		}
		break;
	case MWIPCORE_SET_INTERRUPT_OFFSET:
    	/* Set the interrupt offset 
         */
       		if( copy_from_user(&data, (unsigned int *)arg, sizeof(unsigned int)) )
		{
		    return -EACCES;
		}
                g_mw_intr_offset = arg;
		break;
	default:
		return -EINVAL;
    }
    return 0;
}




struct file_operations mwipcore_cdev_fops = {
    .owner 		= THIS_MODULE,
    .open 		= mwipcore_open,
    .fasync 		= mwipcore_fasync_impl,
    .release 		= mwipcore_close,
    .mmap		= mwipcore_mmap,
    .unlocked_ioctl	= mwipcore_ioctl,
};

static int mwipcore_setup_cdev(struct ipcore_info *thisIpcore, dev_t *dev_id)
{
    int status = 0;
    struct device * thisDevice;
   
   cdev_init(&thisIpcore->cdev, &mwipcore_cdev_fops);
   thisIpcore->cdev.owner = THIS_MODULE;
   thisIpcore->cdev.ops = &mwipcore_cdev_fops;
   *dev_id = MKDEV(MAJOR(mwipcore_dev_id), cur_minor++);
   status = cdev_add(&thisIpcore->cdev, *dev_id, 1);

   if (status < 0) {
       return status;
   } 

   
   thisDevice = device_create(mwipcore_class, NULL, *dev_id, NULL, "%s", thisIpcore->name);
   
   if(IS_ERR(thisDevice)) 
   {
       status = PTR_ERR(thisDevice);
       dev_err(&thisIpcore->pdev->dev, "Error: failed to create device node %s, err %d\n", thisIpcore->name, status);
       cdev_del(&thisIpcore->cdev);
   }
   return status;
}

static void mwipcore_init(struct ipcore_info *thisIpcore)
{
    printk(KERN_INFO DRIVER_NAME ": Initialization done.\n");
}


static const struct of_device_id mwipcore_of_match[] = {
    { .compatible = "mathworks,mwipcore-axi4lite-v1.00",},
    {},

};

MODULE_DEVICE_TABLE(of, mwipcore_of_match);



static int mwipcore_of_probe(struct platform_device *pdev)
{
    int status = 0;
    struct ipcore_info *thisIpcore;
    struct device_node *nodePointer = pdev->dev.of_node;

    thisIpcore = (struct ipcore_info*) kzalloc(sizeof(*thisIpcore), GFP_KERNEL);
    
    if (!thisIpcore)
    {
        status = -ENOMEM;
        goto allocation_error;
    }
    
    thisIpcore->mem = platform_get_resource(pdev, IORESOURCE_MEM,0);

    if(!thisIpcore->mem) 
    {
        status = -ENOENT;
        dev_err(&pdev->dev, "Failed to obtain the resource for platform device\n");
        goto invalid_platform_res;
    }

    printk(KERN_INFO DRIVER_NAME " : Dev memory resource found at %08X %08X. \n", thisIpcore->mem->start, resource_size(thisIpcore->mem));


    thisIpcore->mem = request_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem), pdev->name);

    if (!thisIpcore->mem)
    {
        status = -ENODEV;
        dev_err(&pdev->dev, "Error while request_mem_region call\n");
        goto mem_request_err;
    }


    thisIpcore->regs = ioremap(thisIpcore->mem->start, resource_size(thisIpcore->mem));
    if(!thisIpcore->regs)
    {
        status = -ENODEV;
        dev_err(&pdev->dev, "Failed while ioremap\n"); 
        goto ioremap_failure;
    }

    thisIpcore->pdev = pdev;
    thisIpcore->name = nodePointer->name;
    MW_DBG_printf("IPCore name :%s\n", thisIpcore->name);

    if (nodePointer->data == NULL)
         nodePointer->data = thisIpcore; 

    if(mwipcore_dev_id == 0)
    {
       status = alloc_chrdev_region(&mwipcore_dev_id, 0, 16, DRIVER_NAME);
       if (status)
       {
           dev_err(&pdev->dev, "Character dev. region not allocated: %d\n", status);
           goto chrdev_alloc_err;
       }
       printk(KERN_INFO DRIVER_NAME ": Char dev region registered: major num:%d\n", MAJOR(mwipcore_dev_id));
    }

    if(mwipcore_class == NULL)
    {
        mwipcore_class = class_create(THIS_MODULE, DRIVER_NAME);
        if(IS_ERR(mwipcore_class))
        {
            status = PTR_ERR(mwipcore_class);
            goto class_create_err;
        }
        printk(KERN_INFO DRIVER_NAME ": mwipcore class registration success\n");
    }

    status = mwipcore_setup_cdev(thisIpcore, &(thisIpcore->dev_id));
    if(status)
    {
        dev_err(&pdev->dev, "mwipcore device addition failed: %d\n", status);
        goto dev_add_err;
    }

    thisIpcore->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
    /* It is possible that we have not required any interrupt */
    if (thisIpcore->irq)
    {
        status = request_irq(thisIpcore->irq, mwipcore_intr_handler, IRQF_SHARED, DRIVER_NAME, thisIpcore);
        if(status)
        {
            dev_err(&pdev->dev, "mwipcore interrupt request addition failed.\n");
            goto dev_add_err;
        }
    }

    mwipcore_init(thisIpcore);


    device_num++;
    return status;


dev_add_err:
    if(mwipcore_class){
         class_destroy(mwipcore_class);    
    }
class_create_err:
    unregister_chrdev_region(mwipcore_dev_id, 16);
    mwipcore_dev_id = 0;
chrdev_alloc_err:
    iounmap(thisIpcore->regs);
ioremap_failure:
     release_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem));

mem_request_err:
invalid_platform_res:
    kfree(thisIpcore);
allocation_error:
    return status;
}


static int mwipcore_of_remove(struct platform_device *pdev)
{
    struct ipcore_info *thisIpcore;
    struct device_node *nodePointer =  pdev->dev.of_node;


    if(nodePointer->data == NULL)
    {
        dev_err(&pdev->dev, "MWIPCORE device not found.\n");
        return -ENOSYS;
    }

    thisIpcore = (struct ipcore_info *) (nodePointer->data);


    printk(KERN_INFO DRIVER_NAME "%s : free and release memory\n", nodePointer->name);
    
    if(thisIpcore->regs)
    {
        iounmap(thisIpcore->regs);
    } 

    if(thisIpcore->mem->start)
    {
        release_mem_region(thisIpcore->mem->start, resource_size(thisIpcore->mem));

    }
    nodePointer->data = NULL;
    device_num--;

    
    if(&thisIpcore->cdev)
    {
        printk(KERN_INFO DRIVER_NAME ": Destroy character dev\n");
        device_destroy(mwipcore_class, thisIpcore->dev_id);
        cdev_del(&thisIpcore->cdev);
    }
    cur_minor--;

    if(device_num == 0)
    {
        printk(KERN_INFO DRIVER_NAME ": destroy mwipcore class\n");
        if (mwipcore_class)
        {
             class_destroy(mwipcore_class);
        }

        printk(KERN_INFO DRIVER_NAME " : release device region\n");
        unregister_chrdev_region(mwipcore_dev_id, 16);
        mwipcore_dev_id  = 0;
    }
    return 0;
}



static struct platform_driver mwipcore_driver = {
    .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE, 
		.of_match_table = mwipcore_of_match,
		},
    .probe = mwipcore_of_probe,
    .remove = mwipcore_of_remove,
};

module_platform_driver(mwipcore_driver);


MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": MathWorks AXI4Lite driver");
MODULE_ALIAS(DRIVER_NAME);
