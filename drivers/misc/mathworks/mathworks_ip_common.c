/*
 * MathWorks IP Common Functionality
 *
 * Copyright 2013-2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <linux/mathworks/mathworks_ip.h>

#define DRIVER_NAME "mathworks_ip"

/*Device structure for IPCore information*/
static struct class *mathworks_ip_class = NULL;
static struct mathworks_ip_dev_info dev_table[MATHWORKS_IP_MAX_DEVTYPE] = {{{0}}};

static irqreturn_t mathworks_ip_intr_handler(int irq, void * theIpcore)
{
    struct mathworks_ip_info *thisIpcore = (struct mathworks_ip_info*) theIpcore;

    dev_dbg(thisIpcore->dev, "IRQ %d Handled\n", irq);
    return IRQ_HANDLED;

}

static int mathworks_ip_fasync_impl(int fd, struct file* fp, int mode)
{
    struct mathworks_ip_info *thisIpcore = fp->private_data;
    
    return fasync_helper(fd, fp, mode, &thisIpcore->asyncq);
  
}

static int mathworks_ip_open(struct inode *inode, struct file *fp)
{
    struct mathworks_ip_info *thisIpcore;
    thisIpcore = container_of(inode->i_cdev, struct mathworks_ip_info, cdev);
    fp->private_data = thisIpcore;
    
    return 0;
}

static int mathworks_ip_close(struct inode *inode, struct file *fp)
{
    mathworks_ip_fasync_impl(-1, fp, 0);
    return 0;
}

static int mathworks_ip_dma_alloc(struct mathworks_ip_info *thisIpcore, size_t size) {

	struct mw_dma_info *dinfo = &thisIpcore->dma_info;
	
	if (dinfo->size != 0) {
		dev_err(thisIpcore->dev, "DMA memory already allocated\n");		
		return -EEXIST;
	}
	
	dinfo->virt = dmam_alloc_coherent(thisIpcore->dev, size,
						&dinfo->phys, GFP_KERNEL);
	if(!dinfo->virt){
		dev_err(thisIpcore->dev, "failed to allocate DMA memory\n");		
		return -ENOMEM;
	}
	dinfo->size = size;
	
	return 0;

}

static int	mathworks_ip_dma_info(struct mathworks_ip_info *thisIpcore, void *arg)
{
	
	struct mathworks_ip_dma_info dinfo;
	
	/* Copy the struct from user space */
	if( copy_from_user(&dinfo, (struct mathworks_ip_dma_info *)arg, sizeof(struct mathworks_ip_dma_info)) ) {
		return -EACCES;
	}
	
	/* Populate the struct with information */
	dinfo.size = thisIpcore->dma_info.size;
	dinfo.phys = (void *)((uintptr_t)thisIpcore->dma_info.phys);
	
	/* Copy the struct back to user space */
	if( copy_to_user((struct mathworks_ip_dma_info*)arg, &dinfo, sizeof(struct mathworks_ip_dma_info)) ) {
		return -EACCES;
	}
	
	return 0;

}

static int	mathworks_ip_reg_info(struct mathworks_ip_info *thisIpcore, void *arg)
{
	struct mathworks_ip_reg_info rinfo;

	/* Copy the struct from user space */
	if( copy_from_user(&rinfo, (struct mathworks_ip_reg_info *)arg, sizeof(struct mathworks_ip_reg_info)) ) {
		return -EACCES;
	}

	/* Populate the struct with information */
	rinfo.size = resource_size(thisIpcore->mem);
	rinfo.phys = (void *)((uintptr_t)thisIpcore->mem->start);

	/* Copy the struct back to user space */
	if( copy_to_user((struct mathworks_ip_reg_info*)arg, &rinfo, sizeof(struct mathworks_ip_reg_info)) ) {
		return -EACCES;
	}

	return 0;
}

static int mathworks_ip_get_devinfo(struct mathworks_ip_info *thisIpcore)
{
	int i, devname_len, status;
	char devname[MATHWORKS_IP_DEVNAME_LEN];
	char *tgtDevname;
	struct mathworks_ip_dev_info *thisDev;

	thisIpcore->ops->get_devname(thisIpcore,devname);
	devname_len = strlen(devname);
	for (i = 0; i < MATHWORKS_IP_MAX_DEVTYPE; i++)
	{
		/* Search for the device in the table */
		thisDev = &dev_table[i];
		tgtDevname=thisDev->devname;
		if(*tgtDevname == 0){
			dev_info(thisIpcore->dev, "'%s' device not found, creating\n", devname);
			break;
		}
		if(strncasecmp(tgtDevname,devname,devname_len) == 0)
		{
			dev_info(thisIpcore->dev, "'%s' device found, adding\n", devname);
			thisIpcore->dev_info = thisDev;
			return 0;
		}
	}
	if ((*tgtDevname == 0) && i < MATHWORKS_IP_MAX_DEVTYPE)
	{
		/* Add in a new device to the table */
		strncpy(tgtDevname,devname,devname_len);

		status = alloc_chrdev_region(&thisDev->devid, 0, MATHWORKS_IP_MAX_DEVTYPE, devname);
		if (status)
		{
			dev_err(thisIpcore->dev, "Character dev. region not allocated: %d\n", status);
			return status;
		}
		dev_info(thisIpcore->dev, "Char dev region registered: major num:%d\n", MAJOR(thisDev->devid));
		dev_info(thisIpcore->dev, "'%s' device created\n", devname);
		thisIpcore->dev_info = thisDev;
		return 0;
	}

	/* Not found and table full */
	thisIpcore->dev_info = NULL;
	return -ENOMEM;

}

static void mathworks_ip_mmap_dma_open(struct vm_area_struct *vma)
{
    struct mathworks_ip_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "DMA VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mathworks_ip_mmap_dma_close(struct vm_area_struct *vma)
{
    struct mathworks_ip_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "DMA VMA close.\n");
	
	/* Free the memory DMA */
	dmam_free_coherent(thisIpcore->dev,thisIpcore->dma_info.size,
				thisIpcore->dma_info.virt, thisIpcore->dma_info.phys);
	
	/* Set the size to zero to indicate no memory is allocated */
	thisIpcore->dma_info.size = 0;
}

static void mathworks_ip_mmap_open(struct vm_area_struct *vma)
{
    struct mathworks_ip_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mathworks_ip_mmap_close(struct vm_area_struct *vma)
{
    struct mathworks_ip_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "Simple VMA close.\n");
}


static int mathworks_ip_mmap_fault(struct vm_fault *vmf)
{
    struct vm_area_struct *vma = vmf->vma;
    struct mathworks_ip_info * thisIpcore = vma->vm_private_data;
    struct page *thisPage;
    unsigned long offset;
    offset = (vmf->pgoff - vma->vm_pgoff) << PAGE_SHIFT;
    thisPage = virt_to_page(thisIpcore->mem->start + offset);
    get_page(thisPage);
    vmf->page = thisPage;
    return 0;
}

static struct vm_operations_struct mathworks_ip_mmap_ops = {
    .open   = mathworks_ip_mmap_open,
    .close  = mathworks_ip_mmap_close,
    .fault = mathworks_ip_mmap_fault,
}; 

static struct vm_operations_struct mathworks_ip_mmap_dma_ops = {
    .open   = mathworks_ip_mmap_dma_open,
    .close  = mathworks_ip_mmap_dma_close,
}; 

static int mathworks_ip_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct mathworks_ip_info *thisIpcore = fp->private_data;
    size_t	size = vma->vm_end - vma->vm_start;
	int status = 0;
	vma->vm_private_data = thisIpcore;
	
	dev_info(thisIpcore->dev, "[MMAP] size:%X pgoff: %lx\n", (unsigned int)size, vma->vm_pgoff);
 
	switch(vma->vm_pgoff) {
		case 0: 
            if (!thisIpcore->mem) {
        		return -ENOMEM;
        	}
			/* mmap the MMIO base address */
			vma->vm_flags |= VM_IO | VM_DONTDUMP | VM_DONTDUMP; // may be redundant with call to remap_pfn_range below
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			if (remap_pfn_range(vma, vma->vm_start,
					thisIpcore->mem->start >> PAGE_SHIFT,
					size,
					vma->vm_page_prot))
			{
				return -EAGAIN;
			}
			vma->vm_ops = &mathworks_ip_mmap_ops;
			break;
		default:
			/* mmap the DMA region */
			
			status = mathworks_ip_dma_alloc(thisIpcore, size);
			if (status != 0)
				return status;
						
			if (thisIpcore->dma_info.size == 0 || size != thisIpcore->dma_info.size)
				return -EINVAL;
			/* We want to mmap the whole buffer */
			vma->vm_pgoff = 0; 
			status =  dma_mmap_coherent(thisIpcore->dev,vma,
						thisIpcore->dma_info.virt, thisIpcore->dma_info.phys, size);
			vma->vm_ops = &mathworks_ip_mmap_dma_ops;
			break;
	} 
	//vma->vm_ops->open(vma);
	
	return status;
}

static long mathworks_ip_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    /* struct ipcore_info *thisIpcore = fp->private_data; */
    int status;
	struct mathworks_ip_info *thisIpcore = fp->private_data;
    
    if (NULL==thisIpcore) {
        return -ENODEV;
    }

    switch(cmd) {
	case MATHWORKS_IP_GET_PARAM:
		if (thisIpcore->ops->get_param) {
			status = thisIpcore->ops->get_param(thisIpcore, (void *)arg);
		} else {
			status = -ENODEV;
		}

		break;
		
	case MATHWORKS_IP_DMA_INFO:
		
		status = mathworks_ip_dma_info(thisIpcore, (void *)arg);
		break;
	
	case MATHWORKS_IP_REG_INFO:

		status = mathworks_ip_reg_info(thisIpcore, (void *)arg);
		break;

	default:
		status = -EINVAL;
    }
    return status;
}



static void mathworks_ip_remove_cdev(void *opaque){
	struct mathworks_ip_info *thisIpcore = opaque;

	if(&thisIpcore->cdev)
	{
		dev_info(thisIpcore->dev, "Destroy character dev\n");
		device_destroy(mathworks_ip_class, thisIpcore->dev_id);
		cdev_del(&thisIpcore->cdev);
	}

	if (thisIpcore->dev_info){
		thisIpcore->dev_info->devcnt--;

		if(thisIpcore->dev_info->devcnt == 0)
		{
			dev_info(thisIpcore->dev, "release device region\n");
			unregister_chrdev_region(thisIpcore->dev_info->devid, MATHWORKS_IP_MAX_DEVTYPE);
		}
	}
}

struct file_operations mathworks_ip_common_fops = {
    .owner 		= THIS_MODULE,
    .open 		= mathworks_ip_open,
    .fasync 		= mathworks_ip_fasync_impl,
    .release 		= mathworks_ip_close,
    .mmap		= mathworks_ip_mmap,
    .unlocked_ioctl	= mathworks_ip_ioctl,
};

EXPORT_SYMBOL_GPL(mathworks_ip_common_fops);

static int mathworks_ip_setup_cdev(struct mathworks_ip_info *thisIpcore)
{
    int status = 0;
	struct mathworks_ip_dev_info *dev_entry;
   
	if(mathworks_ip_class == NULL){
		return -EPROBE_DEFER;
	}
	cdev_init(&thisIpcore->cdev, thisIpcore->ops->fops);
	thisIpcore->cdev.owner = thisIpcore->module;
   
	/* Find the device name */
	status = mathworks_ip_get_devinfo(thisIpcore);
	if (status)
	{
		return status;
	}
	dev_entry = thisIpcore->dev_info;

	thisIpcore->dev_id = MKDEV(MAJOR(dev_entry->devid), dev_entry->devcnt);
	status = cdev_add(&thisIpcore->cdev, thisIpcore->dev_id, 1);
	if (status) {
	   goto add_err;
	}

	thisIpcore->char_device = device_create(mathworks_ip_class, thisIpcore->dev, thisIpcore->dev_id, NULL, "%s%d", dev_entry->devname, dev_entry->devcnt++);


	if(IS_ERR(thisIpcore->char_device))
	{
	   status = PTR_ERR(thisIpcore->char_device);
	   dev_err(thisIpcore->dev, "Error: failed to create device node %s, err %d\n", thisIpcore->name, status);
	   goto create_err;
	}

	status = devm_add_action(thisIpcore->dev, mathworks_ip_remove_cdev, thisIpcore);
	if(status){
	   mathworks_ip_remove_cdev(thisIpcore);
	   return status;
	}

	return status;
create_err:
	dev_entry->devcnt--;
	cdev_del(&thisIpcore->cdev);
add_err:
	if(dev_entry->devcnt == 0)
		unregister_chrdev_region(dev_entry->devid, MATHWORKS_IP_MAX_DEVTYPE);
	return status;
}

static void mathworks_ip_unregister(void *opaque){
	struct mathworks_ip_info *thisIpcore = opaque;
	dev_set_drvdata(thisIpcore->dev, NULL);
}

struct mathworks_ip_info *devm_mathworks_ip_of_init(
		struct platform_device *pdev,
		struct module *module,
		struct mathworks_ip_ops	*ops,
		bool mapRegs)
{
	struct mathworks_ip_info *ipDev;
	int status;

	ipDev = (struct mathworks_ip_info*)devm_kzalloc(&pdev->dev, sizeof(struct mathworks_ip_info), GFP_KERNEL);
	if (!ipDev)
		return ERR_PTR(-ENOMEM);

	if( !pdev || !ops || !ops->fops || !ops->get_devname)
		return ERR_PTR(-EINVAL);

	ipDev->module = module;
	ipDev->ops = ops;
	ipDev->dev = &pdev->dev;
	ipDev->name = pdev->dev.of_node->name;
	/* Check for IRQ first, we may have to defer */
	ipDev->irq = platform_get_irq(pdev, 0);
	if (ipDev->irq < 0) {
		switch (ipDev->irq){
			case -EPROBE_DEFER:
				dev_info(&pdev->dev, "Deferring probe for IRQ resources\n");
				return ERR_PTR(-EPROBE_DEFER);
			case -ENXIO:
				ipDev->irq = 0;
				break;
			default :
				return ERR_PTR(ipDev->irq);
		}
	}
	/* Support only linear IRQ ranges */
	if (ipDev->irq){
		/* capture the number of irqs */
		ipDev->nirq = 1;
		do {
			status = platform_get_irq(pdev, ipDev->nirq);
			if (status > 0){
				if (status == ipDev->irq + ipDev->nirq)
					ipDev->nirq++;
				else
					dev_warn(&pdev->dev, "Non-sequential IRQs are not supported\n");
			}
		} while(status > 0);
	}
	ipDev->mem = platform_get_resource(pdev, IORESOURCE_MEM,0);
	if(ipDev->mem)
	{
		dev_info(&pdev->dev, "Dev memory resource found at %p %08lX. \n",
			 (void *)((uintptr_t)ipDev->mem->start),
			 (unsigned long)resource_size(ipDev->mem));
		ipDev->mem  = devm_request_mem_region(&pdev->dev, ipDev->mem->start, resource_size(ipDev->mem), pdev->name);

		if (!ipDev->mem)
		{
			dev_err(&pdev->dev, "Error while request_mem_region call\n");
			return ERR_PTR(-ENODEV);
		}
		if(mapRegs){
			ipDev->regs = devm_ioremap(&pdev->dev, ipDev->mem->start, resource_size(ipDev->mem));
			if(!ipDev->regs) {
				dev_err(&pdev->dev, "Failed to do ioremap\n");
				return ERR_PTR(-ENODEV);
			}
		}
	}

	return ipDev;
}

EXPORT_SYMBOL_GPL(devm_mathworks_ip_of_init);

int devm_mathworks_ip_register(struct mathworks_ip_info *thisIpcore){
	int status;

	status = mathworks_ip_setup_cdev(thisIpcore);
	if(status)
	{
		dev_err(thisIpcore->dev, "mwipcore device addition failed: %d\n", status);
		return status;
	}

	/* It is possible that we have not required any interrupt */
	if (thisIpcore->irq)
	{
		int irq_idx;
		for (irq_idx = 0; irq_idx < thisIpcore->nirq; irq_idx++) {
			status = devm_request_irq(thisIpcore->dev,
					thisIpcore->irq+irq_idx,
					mathworks_ip_intr_handler,
					0,
					thisIpcore->name,
					thisIpcore);
			if(status)
			{
				dev_err(thisIpcore->dev, "interrupt request addition failed.\n");
				return status;
			}
		}

		dev_info(thisIpcore->dev, "Enabled %d irqs from %d\n", thisIpcore->nirq, thisIpcore->irq);
	}

	dev_set_drvdata(thisIpcore->dev, thisIpcore);
	/* Add the release logic */
	status = devm_add_action(thisIpcore->dev, mathworks_ip_unregister, thisIpcore);
	if(status){
		mathworks_ip_unregister(thisIpcore);
		return status;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(devm_mathworks_ip_register);

static int __init mathworks_ip_init(void)
{
	mathworks_ip_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(mathworks_ip_class))
		return PTR_ERR(mathworks_ip_class);
	pr_info("Registered %s class\n", DRIVER_NAME);
	return 0;
}

static void __exit mathworks_ip_exit(void)
{

	class_destroy(mathworks_ip_class);
	mathworks_ip_class = NULL;
}

module_init(mathworks_ip_init);
module_exit(mathworks_ip_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks IP driver framework");
MODULE_ALIAS(DRIVER_NAME);
