#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/device.h>

MODULE_AUTHOR("XCube, Ben Chaco");
MODULE_DESCRIPTION("Adapteva Epiphany Driver");
MODULE_LICENSE("GPL");

#define DEVICE_NAME             "epiphany"
#define DRIVER_NAME             "epiphany"

/* The physical address of the start of the Epiphany divice memory */
#define EPIPHANY_MEM_START      0x80800000UL

static int epiphany_init(void);
static void epiphany_exit(void);
static int epiphany_open(struct inode *, struct file *);
static int epiphany_release(struct inode *, struct file *);
static int epiphany_mmap(struct file *, struct vm_area_struct *);

static struct file_operations epiphany_fops = {
 .owner   =     THIS_MODULE,
 .open    =     epiphany_open,
 .release =     epiphany_release,
 .mmap    =     epiphany_mmap
};

static int major = 0;
static dev_t dev_no = 0;
struct cdev *epiphany_cdev = 0;
static struct class *class_epiphany;
static struct device *dev_epiphany;

static int epiphany_init(void)
{
    int result = 0;
    void *ptr_err;

    result = alloc_chrdev_region(&dev_no, 0, 1, DRIVER_NAME);
    if ( result < 0 ) {
        printk(KERN_ERR "Failed to register the epiphany driver\n");
        return result;
    }

    major = MAJOR(dev_no);
    dev_no = MKDEV(major,0);

    epiphany_cdev = cdev_alloc();
    epiphany_cdev->ops = &epiphany_fops;
    epiphany_cdev->owner = THIS_MODULE;

    result = cdev_add(epiphany_cdev, dev_no, 1);
    if ( result < 0 ) 
    {
        printk(KERN_INFO "wpiphany_init() - Unable to add character device");
    }

    class_epiphany = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(ptr_err = class_epiphany))
    {
        goto err2;
    }

    dev_epiphany = device_create(class_epiphany, NULL, dev_no, NULL, DRIVER_NAME);
    if (IS_ERR(ptr_err = dev_epiphany))
    {
        goto err;
    }

    return 0;

err:
    class_destroy(class_epiphany);     
err2:
    cdev_del(epiphany_cdev);
    unregister_chrdev_region(dev_no, 1);
        
    return PTR_ERR(ptr_err);
}

static void epiphany_exit(void)
{
    device_destroy(class_epiphany, MKDEV(major, 0));
    class_destroy(class_epiphany);
    cdev_del(epiphany_cdev);
    unregister_chrdev_region(dev_no, 1);
}

static int epiphany_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int epiphany_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
    .access = generic_access_phys
#endif
};

static int epiphany_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long off  = vma->vm_pgoff << PAGE_SHIFT;
    unsigned long pfn  = off >> PAGE_SHIFT;
    unsigned long size = vma->vm_end - vma->vm_start;

    vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    vma->vm_ops = &mmap_mem_ops;

#ifdef EHalUsesOffsetsRatherThanPhysicalAddress
    pfn = (EPIPHANY_MEM_START + off) >> PAGE_SHIFT;
#endif

    if ( io_remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot) ) {
        printk(KERN_ERR "epiphany_mmap - failed to remap the page range\n");
        return -EAGAIN;
    }

    return 0;
}

// Driver Entry Point
module_init(epiphany_init);

// Driver Exit Point
module_exit(epiphany_exit);
