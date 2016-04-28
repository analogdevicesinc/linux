#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include "mwgeneric.h"

#define DRIVER_NAME "mwgeneric"

/*Device structure for IPCore information*/
static struct class *mwgeneric_class = NULL;
static struct mwgeneric_info dev_table[MWGENERIC_MAX_DEVTYPE] = {{{0}}};

static irqreturn_t mwgeneric_intr_handler(int irq, void * theIpcore)
{
    struct ipcore_info *thisIpcore = (struct ipcore_info*) theIpcore;

    dev_dbg(thisIpcore->dev, "IRQ %d Handled\n", irq);
    return IRQ_HANDLED;

}

static int mwgeneric_fasync_impl(int fd, struct file* fp, int mode)
{
    struct ipcore_info *thisIpcore = fp->private_data;
    
    return fasync_helper(fd, fp, mode, &thisIpcore->asyncq);
  
}

static int mwgeneric_open(struct inode *inode, struct file *fp)
{
    struct ipcore_info *thisIpcore;
    thisIpcore = container_of(inode->i_cdev, struct ipcore_info, cdev);
    fp->private_data = thisIpcore;
    
    return 0;
}

static int mwgeneric_close(struct inode *inode, struct file *fp)
{
    mwgeneric_fasync_impl(-1, fp, 0);
    return 0;
}

static int mwgeneric_dma_alloc(struct ipcore_info *thisIpcore, size_t size) {

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

static int	mwgeneric_dma_info(struct ipcore_info *thisIpcore, void *arg)
{
	
	struct mwgeneric_dma_info dinfo;
	
	/* Copy the struct from user space */
	if( copy_from_user(&dinfo, (struct mwgeneric_dma_info *)arg, sizeof(struct mwgeneric_dma_info)) ) {
		return -EACCES;
	}
	
	/* Populate the struct with information */
	dinfo.size = thisIpcore->dma_info.size;
	dinfo.phys = (void *)thisIpcore->dma_info.phys;
	
	/* Copy the struct back to user space */
	if( copy_to_user((struct mwgeneric_dma_info*)arg, &dinfo, sizeof(struct mwgeneric_dma_info)) ) {
		return -EACCES;
	}
	
	return 0;

}

static int	mwgeneric_reg_info(struct ipcore_info *thisIpcore, void *arg)
{
	struct mwgeneric_reg_info rinfo;

	/* Copy the struct from user space */
	if( copy_from_user(&rinfo, (struct mwgeneric_reg_info *)arg, sizeof(struct mwgeneric_reg_info)) ) {
		return -EACCES;
	}

	/* Populate the struct with information */
	rinfo.size = resource_size(thisIpcore->mem);
	rinfo.phys = (void *)thisIpcore->mem->start;

	/* Copy the struct back to user space */
	if( copy_to_user((struct mwgeneric_reg_info*)arg, &rinfo, sizeof(struct mwgeneric_reg_info)) ) {
		return -EACCES;
	}

	return 0;
}

static int mwgeneric_get_devinfo(struct ipcore_info *thisIpcore)
{
	int i, devname_len, status;
	char devname[MWGENERIC_DEVNAME_LEN];
	char *tgtDevname;
	struct mwgeneric_info *thisDev;

	thisIpcore->ops->get_devname(thisIpcore,devname);
	devname_len = strlen(devname);
	for (i = 0; i < MWGENERIC_MAX_DEVTYPE; i++)
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
	if ((*tgtDevname == 0) && i < MWGENERIC_MAX_DEVTYPE)
	{
		/* Add in a new device to the table */
		strncpy(tgtDevname,devname,devname_len);

		status = alloc_chrdev_region(&thisDev->devid, 0, MWGENERIC_MAX_DEVTYPE, devname);
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

static void mwgeneric_mmap_dma_open(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "DMA VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwgeneric_mmap_dma_close(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "DMA VMA close.\n");
	
	/* Free the memory DMA */
	dmam_free_coherent(thisIpcore->dev,thisIpcore->dma_info.size,
				thisIpcore->dma_info.virt, thisIpcore->dma_info.phys);
	
	/* Set the size to zero to indicate no memory is allocated */
	thisIpcore->dma_info.size = 0;
}

static void mwgeneric_mmap_open(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    
}

static void mwgeneric_mmap_close(struct vm_area_struct *vma)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
	dev_info(thisIpcore->dev, "Simple VMA close.\n");
}


static int mwgeneric_mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    struct ipcore_info * thisIpcore = vma->vm_private_data;
    struct page *thisPage;
    unsigned long offset;
    offset = (vmf->pgoff - vma->vm_pgoff) << PAGE_SHIFT;
    thisPage = virt_to_page(thisIpcore->mem->start + offset);
    get_page(thisPage);
    vmf->page = thisPage;
    return 0;
}

static struct vm_operations_struct mwgeneric_mmap_ops = {
    .open   = mwgeneric_mmap_open,
    .close  = mwgeneric_mmap_close,
    .fault = mwgeneric_mmap_fault,
}; 

static struct vm_operations_struct mwgeneric_mmap_dma_ops = {
    .open   = mwgeneric_mmap_dma_open,
    .close  = mwgeneric_mmap_dma_close,
}; 

static int mwgeneric_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct ipcore_info *thisIpcore = fp->private_data;
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
			vma->vm_ops = &mwgeneric_mmap_ops;
			break;
		default:
			/* mmap the DMA region */
			
			status = mwgeneric_dma_alloc(thisIpcore, size);
			if (status != 0)
				return status;
						
			if (thisIpcore->dma_info.size == 0 || size != thisIpcore->dma_info.size)
				return -EINVAL;
			/* We want to mmap the whole buffer */
			vma->vm_pgoff = 0; 
			status =  dma_mmap_coherent(thisIpcore->dev,vma,
						thisIpcore->dma_info.virt, thisIpcore->dma_info.phys, size);
			vma->vm_ops = &mwgeneric_mmap_dma_ops;
			break;
	} 
	//vma->vm_ops->open(vma);
	
	return status;
}

static long mwgeneric_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    /* struct ipcore_info *thisIpcore = fp->private_data; */
    int status;
	struct ipcore_info *thisIpcore = fp->private_data;
    
    if (NULL==thisIpcore) {
        return -ENODEV;
    }

    switch(cmd) {
	case MWGENERIC_GET_PARAM:
		if (thisIpcore->ops->get_param) {
			status = thisIpcore->ops->get_param(thisIpcore, (void *)arg);
		} else {
			status = -ENODEV;
		}

		break;
		
	case MWGENERIC_DMA_INFO:
		
		status = mwgeneric_dma_info(thisIpcore, (void *)arg);
		break;
	
	case MWGENERIC_REG_INFO:

		status = mwgeneric_reg_info(thisIpcore, (void *)arg);
		break;

	default:
		status = -EINVAL;
    }
    return status;
}



static void mwgeneric_remove_cdev(void *opaque){
	struct ipcore_info *thisIpcore = opaque;

	if(&thisIpcore->cdev)
	{
		dev_info(thisIpcore->dev, "Destroy character dev\n");
		device_destroy(mwgeneric_class, thisIpcore->dev_id);
		cdev_del(&thisIpcore->cdev);
	}

	if (thisIpcore->dev_info){
		thisIpcore->dev_info->devcnt--;

		if(thisIpcore->dev_info->devcnt == 0)
		{
			dev_info(thisIpcore->dev, "release device region\n");
			unregister_chrdev_region(thisIpcore->dev_info->devid, MWGENERIC_MAX_DEVTYPE);
		}
	}
}

static struct file_operations mwgeneric_cdev_fops = {
    .owner 		= THIS_MODULE,
    .open 		= mwgeneric_open,
    .fasync 		= mwgeneric_fasync_impl,
    .release 		= mwgeneric_close,
    .mmap		= mwgeneric_mmap,
    .unlocked_ioctl	= mwgeneric_ioctl,
};

static int mwgeneric_setup_cdev(struct ipcore_info *thisIpcore)
{
    int status = 0;
	struct mwgeneric_info *dev_entry;
   
	if(mwgeneric_class == NULL){
		return -EPROBE_DEFER;
	}
	cdev_init(&thisIpcore->cdev, &mwgeneric_cdev_fops);
	thisIpcore->cdev.owner = THIS_MODULE;
	thisIpcore->cdev.ops = &mwgeneric_cdev_fops;
   
	/* Find the device name */
	status = mwgeneric_get_devinfo(thisIpcore);
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

	thisIpcore->char_device = device_create(mwgeneric_class, thisIpcore->dev, thisIpcore->dev_id, NULL, "%s%d", dev_entry->devname, dev_entry->devcnt++);

	if(IS_ERR(thisIpcore->char_device))
	{
	   status = PTR_ERR(thisIpcore->char_device);
	   dev_err(thisIpcore->dev, "Error: failed to create device node %s, err %d\n", thisIpcore->name, status);
	   goto create_err;
	}

	status = devm_add_action(thisIpcore->dev, mwgeneric_remove_cdev, thisIpcore);
	if(status){
	   mwgeneric_remove_cdev(thisIpcore);
	   return status;
	}

	return status;
create_err:
	dev_entry->devcnt--;
	cdev_del(&thisIpcore->cdev);
add_err:
	if(dev_entry->devcnt == 0)
		unregister_chrdev_region(dev_entry->devid, MWGENERIC_MAX_DEVTYPE);
	return status;
}

static void mwgeneric_unregister(void *opaque){
	struct ipcore_info *thisIpcore = opaque;
	dev_set_drvdata(thisIpcore->dev, NULL);
}

int devm_mwgeneric_register(struct ipcore_info *thisIpcore){
	int status;

	status = mwgeneric_setup_cdev(thisIpcore);
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
					mwgeneric_intr_handler,
					0,
					thisIpcore->name,
					thisIpcore);
			if(status)
			{
				dev_err(thisIpcore->dev, "mwgeneric interrupt request addition failed.\n");
				return status;
			}
		}

		dev_info(thisIpcore->dev, "Enabled %d irqs from %d\n", thisIpcore->nirq, thisIpcore->irq);
	}

	dev_set_drvdata(thisIpcore->dev, thisIpcore);
	/* Add the release logic */
	status = devm_add_action(thisIpcore->dev, mwgeneric_unregister, thisIpcore);
	if(status){
		mwgeneric_unregister(thisIpcore);
		return status;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(devm_mwgeneric_register);

static int __init mwgeneric_init(void)
{
	mwgeneric_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(mwgeneric_class))
		return PTR_ERR(mwgeneric_class);
	pr_info("Registered mwgeneric class\n");
	return 0;
}

static void __exit mwgeneric_exit(void)
{

	class_destroy(mwgeneric_class);
	mwgeneric_class = NULL;
}

module_init(mwgeneric_init);
module_exit(mwgeneric_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks Generic driver framework");
MODULE_ALIAS(DRIVER_NAME);
