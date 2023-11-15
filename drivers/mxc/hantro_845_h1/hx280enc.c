/*
 * Encoder device driver (kernel module)
 *
 * Copyright (c) 2013-2019, VeriSilicon Inc.
 * Copyright (C) 2012 Google Finland Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
--------------------------------------------------------------------------------
--
--  Abstract : 6280/7280/8270/8290/H1 Encoder device driver (kernel module)
--
------------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_page_range
	SetPageReserved
	ClearPageReserved
*/
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>

#include <linux/moduleparam.h>
/* request_irq(), free_irq() */
#include <linux/interrupt.h>
#include <linux/sched.h>

/* needed for virt_to_phys() */
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>

#include <asm/irq.h>
#include <linux/of_irq.h>
#include <linux/version.h>

/* our own stuff */
#include <linux/hx280enc.h>

#ifndef VSI
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
//static int hantro_h1_major = -1; /* dynamic allocation */
static struct class *hantro_h1_class;
#define DEVICE_NAME		"mxc_hantro_h1"
static struct device *hantro_h1_dev;
static struct clk *hantro_clk_h1;
static struct clk *hantro_clk_h1_bus;
#define IRQF_DISABLED 0x0
#endif

#include <linux/delay.h>
#include <linux/compat.h>

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Hantro 6280/7280/8270/8290/H1 Encoder driver");

/* this is ARM Integrator specific stuff */
#ifndef VSI
#define INTEGRATOR_LOGIC_MODULE0_BASE   0x38320000
#else
#define INTEGRATOR_LOGIC_MODULE0_BASE   0xC0000000
#endif
#define BLK_CTL_BASE        0x38330000 //0x38320000
/*
#define INTEGRATOR_LOGIC_MODULE1_BASE   0xD0000000
#define INTEGRATOR_LOGIC_MODULE2_BASE   0xE0000000
#define INTEGRATOR_LOGIC_MODULE3_BASE   0xF0000000
*/

#define VP_PB_INT_LT                    30
/*
#define INT_EXPINT1                     10
#define INT_EXPINT2                     11
#define INT_EXPINT3                     12
*/
/* these could be module params in the future */

#define ENC_IO_BASE                 INTEGRATOR_LOGIC_MODULE0_BASE
#define ENC_IO_SIZE                 (500 * 4)    /* bytes */

#define ENC_HW_ID1                  0x62800000
#define ENC_HW_ID2                  0x72800000
#define ENC_HW_ID3                  0x82700000
#define ENC_HW_ID4                  0x82900000
#define ENC_HW_ID5                  0x48310000

#define HX280ENC_BUF_SIZE           0

static unsigned long base_port = INTEGRATOR_LOGIC_MODULE0_BASE;
static int irq = VP_PB_INT_LT;

/* for critical data access */
static DEFINE_SPINLOCK(owner_lock);
/* for irq wait */
static DECLARE_WAIT_QUEUE_HEAD(enc_wait_queue);
/* for reserve hw */
static DECLARE_WAIT_QUEUE_HEAD(enc_hw_queue);

/* module_param(name, type, perm) */
module_param(base_port, ulong, 0644);
module_param(irq, int, 0644);

/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hx280enc_major;

/* here's all the must remember stuff */
typedef struct {
	u32 hw_id; //hw id to indicate project
	u32 is_valid; //indicate this core is hantro's core or not
	u32 is_reserved; //indicate this core is occupied by user or not
	struct file *filp;  //indicate which instance is occupying the core
	u32 irq_received; //indicate this core receives irq
	u32 irq_status;
	int irq;
	unsigned long iobaseaddr;
	unsigned int iosize;
	u32 reg_corrupt;
	struct semaphore core_suspend_sem;

	volatile u8 *hwregs;
	struct fasync_struct *async_queue;
	unsigned int mirror_regs[512];
	struct device *dev;
	bool skip_blkctrl;
	u32 statusShowReg;
} hx280enc_t;

/* dynamic allocation? */
static hx280enc_t hx280enc_data;

static int ReserveIO(void);
static void ReleaseIO(void);
static void ResetAsic(hx280enc_t *dev);

#ifdef HX280ENC_DEBUG
static void dump_regs(unsigned long data);
#endif

/* IRQ handler */
static irqreturn_t hx280enc_isr(int irq, void *dev_id);

#ifndef VSI
static int hantro_h1_clk_enable(struct device *dev)
{
	clk_prepare(hantro_clk_h1);
	clk_enable(hantro_clk_h1);
	clk_prepare(hantro_clk_h1_bus);
	clk_enable(hantro_clk_h1_bus);
	return 0;
}

static int hantro_h1_clk_disable(struct device *dev)
{
	if (hantro_clk_h1) {
		clk_disable(hantro_clk_h1);
		clk_unprepare(hantro_clk_h1);
	}
	if (hantro_clk_h1_bus) {
		clk_disable(hantro_clk_h1_bus);
		clk_unprepare(hantro_clk_h1_bus);
	}
	return 0;
}

static int hantro_h1_ctrlblk_reset(struct device *dev)
{
	volatile u8 *iobase;
	u32 val;

	if (hx280enc_data.skip_blkctrl)
		return 0;

	//config H1
	hantro_h1_clk_enable(dev);
	iobase = (volatile u8 *)ioremap(BLK_CTL_BASE, 0x10000);

	val = ioread32(iobase);
	val &= (~0x4);
	iowrite32(val, iobase); // assert soft reset
	udelay(2);

	val = ioread32(iobase);
	val |= 0x4;
	iowrite32(val, iobase); // release soft reset

	val = ioread32(iobase+0x4);
	val |= 0x4;
	iowrite32(val, iobase + 0x4); // enable clock

	iowrite32(0xFFFFFFFF, iobase + 0x14); // H1 fuse encoder enable
	iounmap(iobase);
	hantro_h1_clk_disable(dev);
	return 0;
}

static int hantro_h1_power_on_disirq(hx280enc_t *hx280enc)
{
	pm_runtime_get_sync(hx280enc->dev);
	return 0;
}

#endif

/* the device's mmap method. The VFS has kindly prepared the process's
 * vm_area_struct for us, so we examine this to see what was requested.
 */

static int hx280enc_mmap(struct file *filp, struct vm_area_struct *vm)
{
#ifdef VSI
	int result = -EINVAL;

	result = -EINVAL;
	vma->vm_ops = &hx280enc_vm_ops;
	return result;
#else
	if (vm->vm_pgoff == (hx280enc_data.iobaseaddr >> PAGE_SHIFT)) {
		vm_flags_set(vm, VM_IO);
		vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
		PDEBUG("hx280enc mmap: size=0x%lX, page off=0x%lX\n", (vm->vm_end - vm->vm_start), vm->vm_pgoff);
		return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end - vm->vm_start,
						vm->vm_page_prot) ? -EAGAIN : 0;
	} else {
		pr_err("invalid map offset :0x%lX\n", vm->vm_pgoff);
		return -EINVAL;
	}
#endif
}

static int CheckEncIrq(hx280enc_t *dev)
{
	unsigned long flags;
	int rdy = 0;

	spin_lock_irqsave(&owner_lock, flags);

	if (dev->irq_received) {
		/* reset the wait condition(s) */
		PDEBUG("check irq ready\n");
		dev->irq_received = 0;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);
	//printk("rdy=%d\n",rdy);

	return rdy;
}

unsigned int WaitEncReady(hx280enc_t *dev)
{
	u32 irq_status, is_write1_clr;
	int i;
	long ret;

	PDEBUG("%s\n", __func__);
	ret = wait_event_timeout(enc_wait_queue, CheckEncIrq(dev), msecs_to_jiffies(200));
	if (ret == 0) {
		u32 reg14 = readl(dev->hwregs + 14*4);

		pr_err("%s: wait_event_timeout() timeout !\n", __func__);
		writel(reg14 & (~1), dev->hwregs + 14*4);
		up(&hx280enc_data.core_suspend_sem);
	}

	/* read register to mirror */
	for (i = 0; i < dev->iosize; i += 4)
		dev->mirror_regs[i/4] = readl(dev->hwregs + i);

	dev->mirror_regs[1] = dev->statusShowReg;

	/* clear the status bits */
	is_write1_clr = (dev->mirror_regs[0x4a0/4] & 0x00800000);
	irq_status = dev->mirror_regs[1];
	if (is_write1_clr)
		writel(irq_status, dev->hwregs + 0x04);
	else
		writel(irq_status & (~0xf7d), dev->hwregs + 0x04);
	dev->statusShowReg = readl(dev->hwregs + 0x04);

	return 0;
}

int CheckCoreOccupation(hx280enc_t *dev, struct file *filp)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&owner_lock, flags);
	if (!dev->is_reserved) {
		dev->is_reserved = 1;
		dev->filp = filp;
		ret = 1;
	}
	spin_unlock_irqrestore(&owner_lock, flags);

	return ret;
}

int GetWorkableCore(hx280enc_t *dev, struct file *filp)
{
	int ret = 0;

	PDEBUG("GetWorkableCore\n");

	if (dev->is_valid && CheckCoreOccupation(dev, filp))
		ret = 1;

	return ret;
}

long ReserveEncoder(hx280enc_t *dev, struct file *filp)
{
	/* lock a core that has specified core id*/
	if (wait_event_interruptible(enc_hw_queue, GetWorkableCore(dev, filp) != 0))
		return -ERESTARTSYS;

	return 0;
}

void ReleaseEncoder(hx280enc_t *dev, struct file *filp)
{
	unsigned long flags;

	PDEBUG("ReleaseEncoder\n");

	spin_lock_irqsave(&owner_lock, flags);
	if (dev->is_reserved && dev->filp == filp) {
		dev->filp = NULL;
		dev->is_reserved = 0;
	}

	dev->irq_received = 0;
	dev->irq_status = 0;
	spin_unlock_irqrestore(&owner_lock, flags);

	wake_up_interruptible_all(&enc_hw_queue);
}

static long EncRefreshRegs(hx280enc_t *dev, unsigned int *regs)
{
	long ret;

	ret = copy_to_user(regs, dev->mirror_regs, dev->iosize);
	if (ret) {
		PDEBUG("%s: copy_to_user failed, returned %li\n", __func__, ret);
		return -EFAULT;
	}
	return 0;
}

static int hx280enc_write_regs(struct enc_regs_buffer *regs)
{
	hx280enc_t *dev = &hx280enc_data;
	u32 *reg_buf;
	u32 i;
	int ret;

	if (regs->size > sizeof(dev->mirror_regs) || !regs->size)
		return -EINVAL;
	if (regs->offset >= sizeof(dev->mirror_regs))
		return -EINVAL;
	if ((regs->offset + regs->size) > sizeof(dev->mirror_regs)) {
		pr_err("%s invalid param, offset:%d, size:%d\n",
			__func__, regs->offset, regs->size);
		return -EINVAL;
	}

	reg_buf = &dev->mirror_regs[regs->offset / 4];
	ret = copy_from_user((void *)reg_buf, (void *)regs->regs, regs->size);
	if (ret)
		return ret;

	for (i = 0; i < regs->size / 4; i++) {
		iowrite32(reg_buf[i], (dev->hwregs + regs->offset) + i * 4);
		if ((regs->offset + i * 4) == 4)
			dev->statusShowReg = readl(dev->hwregs + 0x04);
	}

	return ret;
}

static int hx280enc_read_regs(struct enc_regs_buffer *regs)
{
	hx280enc_t *dev = &hx280enc_data;
	u32 *reg_buf;
	u32 i;
	int ret;

	if (regs->size > sizeof(dev->mirror_regs) || !regs->size)
		return -EINVAL;
	if (regs->offset >= sizeof(dev->mirror_regs))
		return -EINVAL;
	if ((regs->offset + regs->size) > sizeof(dev->mirror_regs)) {
		pr_err("%s invalid param, offset:%d, size:%d\n",
			__func__, regs->offset, regs->size);
		return -EINVAL;
	}

	reg_buf = &dev->mirror_regs[regs->offset / 4];

	for (i = 0; i < regs->size / 4; i++)
		reg_buf[i] = ioread32((dev->hwregs + regs->offset) + i * 4);

	ret = copy_to_user((void *)regs->regs, (void *)reg_buf, regs->size);

	return ret;
}

static long hx280enc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	PDEBUG("ioctl cmd 0x%X\n", cmd);
	/*
	* extract the type and number bitfields, and don't encode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/
	if (_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HX280ENC_IOC_MAXNR)
		return -ENOTTY;

	/*
	* the direction is a bitmask, and VERIFY_WRITE catches R/W
	* transfers. `Type' is user-oriented, while
	* access_ok is kernel-oriented, so the concept of "read" and
	* "write" is reversed
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok((void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok((void *) arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (_IOC_NR(cmd))	{
	case _IOC_NR(HX280ENC_IOCGHWOFFSET):
		__put_user(hx280enc_data.iobaseaddr, (unsigned long *) arg);
		break;
	case _IOC_NR(HX280ENC_IOCGHWIOSIZE):
		__put_user(hx280enc_data.iosize, (unsigned int *) arg);
	break;
	case _IOC_NR(HX280ENC_IOCH_ENC_RESERVE): {
		int ret;

		PDEBUG("Reserve ENC Cores\n");
		ret = ReserveEncoder(&hx280enc_data, filp);
		return ret;
	}
	case _IOC_NR(HX280ENC_IOCH_ENC_RELEASE):
		PDEBUG("Release ENC Core\n");
		ReleaseEncoder(&hx280enc_data, filp);
		break;
	case _IOC_NR(HX280ENC_IOCG_CORE_WAIT): {
		unsigned int *regs = (unsigned int *)arg;
		unsigned int ret1, ret2;

		ret1 = WaitEncReady(&hx280enc_data);
		ret2 = EncRefreshRegs(&hx280enc_data, regs);
		if (ret2)
			return ret2;
		if (ret1)
			return ret1;
		break;
	}
	case _IOC_NR(HX280ENC_IOC_WRITE_REGS): {
		struct enc_regs_buffer regs;

		err = copy_from_user(&regs, (void *)arg, sizeof(regs));
		if (err)
			return err;

		err = hx280enc_write_regs(&regs);
		if (err)
			return err;
		break;
	}
	case _IOC_NR(HX280ENC_IOC_READ_REGS): {
		struct enc_regs_buffer regs;

		err = copy_from_user(&regs, (void *)arg, sizeof(regs));
		if (err)
			return err;

		err = hx280enc_read_regs(&regs);
		if (err)
			return err;
		break;
	}
	case _IOC_NR(HX280ENC_IOCG_EN_CORE): {
		u32 reg_value;

		PDEBUG("Enable ENC Core\n");

		if (hx280enc_data.is_reserved == 0)
			return -EPERM;
		if (hx280enc_data.reg_corrupt == 1) {
			hx280enc_data.reg_corrupt = 0;
			return -EAGAIN;
		}

		if (down_timeout(&hx280enc_data.core_suspend_sem, msecs_to_jiffies(10000))) {
			pr_err("en core suspend sem down error id\n");
			return -ERESTARTSYS;
		}

		reg_value = readl(hx280enc_data.hwregs + 14 * 4);
		reg_value |= 0x01;
		writel(reg_value, hx280enc_data.hwregs + 14 * 4);
		// printk("%s reg_value:%d\n", __func__, reg_value);

		break;
	}
	default:
		break;
	}
	return 0;
}

static int hx280enc_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	hx280enc_t *dev = &hx280enc_data;

	filp->private_data = (void *) dev;

#ifndef VSI
	hantro_h1_clk_enable(dev->dev);
	hantro_h1_power_on_disirq(dev);
#endif

	PDEBUG("dev opened\n");
	return result;
}

static int hx280enc_release(struct inode *inode, struct file *filp)
{
	hx280enc_t *dev = (hx280enc_t *) filp->private_data;
	unsigned long flags;
#ifdef HX280ENC_DEBUG
	dump_regs((unsigned long) dev); /* dump the regs */
#endif

	PDEBUG("dev closed\n");
	spin_lock_irqsave(&owner_lock, flags);
	if (dev->is_reserved == 1 && dev->filp == filp) {
		dev->filp = NULL;
		dev->is_reserved = 0;
		dev->irq_received = 0;
		dev->irq_status = 0;
		PDEBUG("release reserved core\n");
	}
	spin_unlock_irqrestore(&owner_lock, flags);
	wake_up_interruptible_all(&enc_hw_queue);

#ifndef VSI
	pm_runtime_put_sync(hantro_h1_dev);
	hantro_h1_clk_disable(hantro_h1_dev);
#endif

	return 0;
}

#ifdef CONFIG_COMPAT
struct enc_regs_buffer_32 {
	__u32 core_id;
	compat_caddr_t regs;
	__u32 offset;
	__u32 size;
	compat_caddr_t reserved;
};

static int get_hantro_enc_regs_buffer32(struct enc_regs_buffer *kp,
					struct enc_regs_buffer_32 __user *up)
{
	u32 tmp1, tmp2;

	if (!access_ok(up, sizeof(struct enc_regs_buffer_32)) ||
	    get_user(kp->core_id, &up->core_id) ||
	    get_user(kp->offset, &up->offset) ||
	    get_user(kp->size, &up->size) ||
	    get_user(tmp1, &up->regs) ||
	    get_user(tmp2, &up->reserved)) {
		return -EFAULT;
	}
	kp->regs = (__force u32 *)compat_ptr(tmp1);
	kp->reserved = (__force u32 *)compat_ptr(tmp2);
	return 0;
}

static bool hantro_h1_is_compat_ptr_ioctl(unsigned int cmd)
{
	bool ret = true;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HX280ENC_IOC_WRITE_REGS):
	case _IOC_NR(HX280ENC_IOC_READ_REGS):
		ret = false;
		break;
	default:
		break;
	}

	return ret;
}

static long hx280enc_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	void __user *up = compat_ptr(arg);
	struct enc_regs_buffer regs;

	if (hantro_h1_is_compat_ptr_ioctl(cmd))
		return compat_ptr_ioctl(filp, cmd, arg);

	err = get_hantro_enc_regs_buffer32(&regs, up);
	if (err)
		return err;

	switch (_IOC_NR(cmd)) {
		case _IOC_NR(HX280ENC_IOC_WRITE_REGS): {
			err = hx280enc_write_regs(&regs);
			if (err)
				return err;
			break;
		}
		case _IOC_NR(HX280ENC_IOC_READ_REGS): {
			err = hx280enc_read_regs(&regs);
			if (err)
				return err;
			break;
		}
	}
	return 0;
}
#endif

/* VFS methods */
static struct file_operations hx280enc_fops = {
	.owner = THIS_MODULE,
	.open = hx280enc_open,
	.release = hx280enc_release,
	.unlocked_ioctl = hx280enc_ioctl,
	.fasync = NULL,
	.mmap = hx280enc_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hx280enc_ioctl32,
#endif
};

#ifndef VSI
static int hx280enc_init(void)
#else
static int __init hx280enc_init(void)
#endif
{
	int result;

	PDEBUG(KERN_INFO "hx280enc: module init - base_port=0x%08lx irq=%i\n",
	base_port, irq);

	hx280enc_data.iobaseaddr = base_port;
	hx280enc_data.iosize = ENC_IO_SIZE;
	hx280enc_data.irq = irq;
	hx280enc_data.async_queue = NULL;
	hx280enc_data.hwregs = NULL;
	sema_init(&hx280enc_data.core_suspend_sem, 1);

	result = register_chrdev(hx280enc_major, "hx280enc", &hx280enc_fops);
	if (result < 0) {
		PDEBUG(KERN_INFO "hx280enc: unable to get major <%d>\n", hx280enc_major);
		return result;
	} else if (result != 0)    /* this is for dynamic major */
		hx280enc_major = result;

	result = ReserveIO();
	if (result < 0)
		goto err;

	ResetAsic(&hx280enc_data);  /* reset hardware */

	/* get the IRQ line */
	if (irq != -1) {
		result = request_irq(irq, hx280enc_isr,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
					SA_INTERRUPT | SA_SHIRQ,
#else
				//IRQF_DISABLED | IRQF_SHARED,
				0,
#endif
					"hx280enc", (void *) &hx280enc_data);
		if (result == -EINVAL) {
			PDEBUG(KERN_ERR "hx280enc: Bad irq number or handler\n");
			ReleaseIO();
			goto err;
		} else if (result == -EBUSY) {
			PDEBUG(KERN_ERR "hx280enc: IRQ <%d> busy, change your config\n",
			hx280enc_data.irq);
			ReleaseIO();
			goto err;
		}
	} else
	PDEBUG(KERN_INFO "hx280enc: IRQ not in use!\n");
	irq_set_status_flags(irq, IRQ_DISABLE_UNLAZY);

	pr_info("hx280enc: module inserted. Major <%d>\n", hx280enc_major);
	return 0;

err:
	unregister_chrdev(hx280enc_major, "hx280enc");
	PDEBUG(KERN_ERR "hx280enc: module not inserted\n");
	return result;
}

#ifndef VSI
static void hx280enc_cleanup(void)
#else
static void __exit hx280enc_cleanup(void)
#endif
{
	writel(0, hx280enc_data.hwregs + 0x38); /* disable HW */
	writel(0, hx280enc_data.hwregs + 0x04); /* clear enc IRQ */

	/* free the encoder IRQ */
	if (hx280enc_data.irq != -1)
		free_irq(hx280enc_data.irq, (void *) &hx280enc_data);

	ReleaseIO();
	unregister_chrdev(hx280enc_major, "hx280enc");

	PDEBUG(KERN_INFO "hx280enc: module removed\n");
}

#ifdef VSI
module_init(hx280enc_init);
module_exit(hx280enc_cleanup);
#endif
static int ReserveIO(void)
{
	long int hwid;

	if (!request_mem_region(hx280enc_data.iobaseaddr, hx280enc_data.iosize, "hx280enc")) {
		PDEBUG(KERN_INFO "hx280enc: failed to reserve HW regs\n");
		return -EBUSY;
	}
	hx280enc_data.hwregs = (volatile u8 *) ioremap(hx280enc_data.iobaseaddr, hx280enc_data.iosize);
	if (hx280enc_data.hwregs == NULL)	{
		PDEBUG(KERN_INFO "hx280enc: failed to ioremap HW regs\n");
		ReleaseIO();
		return -EBUSY;
	}

	hwid = readl(hx280enc_data.hwregs);

	/* check for encoder HW ID */
	if ((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF)) &&
		(((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF)) &&
		(((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID3 >> 16) & 0xFFFF)) &&
		(((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID4 >> 16) & 0xFFFF)) &&
		(((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID5 >> 16) & 0xFFFF))) {
		PDEBUG(KERN_ERR "hx280enc: HW not found at 0x%08lx\n", hx280enc_data.iobaseaddr);
#ifdef HX280ENC_DEBUG
		dump_regs((unsigned long) &hx280enc_data);
#endif
		ReleaseIO();
		return -EBUSY;
	}

	hx280enc_data.hw_id = hwid;
	hx280enc_data.is_valid = 1;

	PDEBUG(KERN_INFO "hx280enc: HW at base <0x%08lx> with ID <0x%08lx>\n", hx280enc_data.iobaseaddr, hwid);
	return 0;
}

static void ReleaseIO(void)
{
	if (hx280enc_data.is_valid == 0)
		return;
	if (hx280enc_data.hwregs)
		iounmap((void *) hx280enc_data.hwregs);
	release_mem_region(hx280enc_data.iobaseaddr, hx280enc_data.iosize);
}

irqreturn_t hx280enc_isr(int irq, void *dev_id)
{
	hx280enc_t *dev = (hx280enc_t *) dev_id;
	u32 irq_status;
	unsigned long flags;
	u32 is_write1_clr;

	spin_lock_irqsave(&owner_lock, flags);
	if (!dev->is_reserved)	{
		spin_unlock_irqrestore(&owner_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&owner_lock, flags);
	irq_status = readl(dev->hwregs + 0x04);

	/* BASE_HWFuse2 = 0x4a0; HWCFGIrqClearSupport = 0x00800000 */
	is_write1_clr = (readl(dev->hwregs + 0x4a0) & 0x00800000);
	if (irq_status & 0x01) {
		/* clear enc IRQ ,sw reset and slice ready interrupt bit */
		if (is_write1_clr)
			writel(irq_status & (0x111), dev->hwregs + 0x04);
		else
			writel(irq_status & (~0x111), dev->hwregs + 0x04);

		dev->statusShowReg = readl(dev->hwregs + 0x04);

		/* Handle slice ready interrupts. The reference implementation
		* doesn't signal slice ready interrupts to EWL.
		* The EWL will poll the slices ready register value. */
		if ((irq_status & 0x1FE) == 0x100) {
			PDEBUG("Slice ready IRQ handled!\n");
			return IRQ_HANDLED;
		}

		if (irq_status & 0x04) {
			spin_lock_irqsave(&owner_lock, flags);
			dev->irq_received = 1;
			dev->irq_status = irq_status & (~0x01);
			spin_unlock_irqrestore(&owner_lock, flags);
			up(&hx280enc_data.core_suspend_sem);
			wake_up_all(&enc_wait_queue);
		} else {
			pr_err("h1 abnormal irq %x\n", irq_status);
		}

		PDEBUG("IRQ handled!\n");
		return IRQ_HANDLED;
	} else {
		PDEBUG("IRQ received, but NOT handled!\n");
		return IRQ_NONE;
	}
}

static void ResetAsic(hx280enc_t *dev)
{
	int i;

	if (dev->is_valid == 0)
		return;

	writel(0, dev->hwregs + 0x38);

	for (i = 4; i < dev->iosize; i += 4)
		writel(0, dev->hwregs + i);
}

#ifdef HX280ENC_DEBUG
static void dump_regs(unsigned long data)
{
	hx280enc_t *dev = (hx280enc_t *) data;
	int i;

	PDEBUG("Reg Dump Start\n");
	for (i = 0; i < dev->iosize; i += 4)
		PDEBUG("\toffset %02X = %08X\n", i, readl(dev->hwregs + i));
	PDEBUG("Reg Dump End\n");
}
#endif



#ifndef VSI
static int hantro_h1_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *temp_class;
	struct resource *res;
	unsigned long reg_base;
	struct device_node *node;

	hantro_h1_dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs_hantro_h1");
	if (!res) {
		pr_err("hantro h1: unable to get vpu base addr\n");
		return -ENODEV;
	}
	reg_base = res->start;
	if ((ulong)reg_base != base_port) {
		pr_err("hantro h1: regbase(0x%lX) not equal to expected value(0x%lX)\n", reg_base, base_port);
		return -ENODEV;
	}

	irq = platform_get_irq_byname(pdev, "irq_hantro_h1");
	if (irq < 0) {
		pr_err("hantro h1: not find valid irq\n");
		return -ENODEV;
	}

	hantro_clk_h1 = clk_get(&pdev->dev, "clk_hantro_h1");
	if (IS_ERR(hantro_clk_h1)) {
		pr_err("hantro h1: get clock failed, %p\n", hantro_clk_h1);
		err = -ENXIO;
		goto error;
	}
	hantro_clk_h1_bus = clk_get(&pdev->dev, "clk_hantro_h1_bus");
	if (IS_ERR(hantro_clk_h1_bus)) {
		pr_err("hantro h1: get bus clock failed, %p\n", hantro_clk_h1_bus);
		err = -ENXIO;
		goto error;
	}

	PDEBUG("hantro: h1 clock: 0x%lX, 0x%lX\n", clk_get_rate(hantro_clk_h1), clk_get_rate(hantro_clk_h1_bus));

	/*
	 * If integrate power-domains into blk-ctrl driver, vpu driver don't
	 * need handle it again.
	 */
	node = of_parse_phandle(pdev->dev.of_node, "power-domains", 0);
	if (!strcmp(node->name, "blk-ctl") || !strcmp(node->name, "blk-ctrl"))
		hx280enc_data.skip_blkctrl = 1;
	else
		hx280enc_data.skip_blkctrl = 0;
	of_node_put(node);

	hantro_h1_clk_enable(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	hantro_h1_ctrlblk_reset(&pdev->dev);

	err = hx280enc_init();
	if (0 != err) {
		pr_err("hantro h1: init failed\n");
		goto error;
	}

	hantro_h1_class = class_create("mxc_hantro_h1");
	if (IS_ERR(hantro_h1_class)) {
		err = PTR_ERR(hantro_h1_class);
		goto error;
	}
	temp_class = device_create(hantro_h1_class, NULL, MKDEV(hx280enc_major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}
	hx280enc_data.dev = &pdev->dev;
	platform_set_drvdata(pdev, &hx280enc_data);

	goto out;

err_out_class:
	device_destroy(hantro_h1_class, MKDEV(hx280enc_major, 0));
	class_destroy(hantro_h1_class);
error:
	pr_err("hantro probe failed\n");
out:
	pm_runtime_put_sync(&pdev->dev);
	hantro_h1_clk_disable(&pdev->dev);
	return err;
}

static int hantro_h1_dev_remove(struct platform_device *pdev)
{
	hantro_h1_clk_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	if (hx280enc_major > 0) {
		device_destroy(hantro_h1_class, MKDEV(hx280enc_major, 0));
		class_destroy(hantro_h1_class);
		hx280enc_cleanup();
		hx280enc_major = 0;
	}
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	hantro_h1_clk_disable(&pdev->dev);

	if (!IS_ERR(hantro_clk_h1))
		clk_put(hantro_clk_h1);
	if (!IS_ERR(hantro_clk_h1_bus))
		clk_put(hantro_clk_h1_bus);

	return 0;
}

static int hx280enc_suspend(struct device *dev)
{
	int ret = 0;
	int i;

	PDEBUG("%s start..\n", __func__);

	if (hx280enc_data.is_reserved == 0)
		return ret;

	ret = down_timeout(&hx280enc_data.core_suspend_sem, msecs_to_jiffies(10000));
	if (ret) {
		pr_err("h280xenc sem down error when store regs\n");
		return ret;
	}
	hx280enc_data.reg_corrupt = 1;
	if (hx280enc_data.irq_status & 0x04) {
		for (i = 0; i < hx280enc_data.iosize; i += 4)
			hx280enc_data.mirror_regs[i/4] = readl(hx280enc_data.hwregs + i);
	}
	up(&hx280enc_data.core_suspend_sem);

	return ret;
}

static int hx280enc_resume(struct device *dev)
{
	int ret = 0;
	int i;

	PDEBUG("%s start..\n", __func__);

	if (hx280enc_data.is_reserved == 0 || hx280enc_data.reg_corrupt == 0)
		return ret;

	if (hx280enc_data.irq_status & 0x04) {
		for (i = 0; i < hx280enc_data.iosize; i += 4)
			writel(hx280enc_data.mirror_regs[i/4], hx280enc_data.hwregs + i);
		hx280enc_data.reg_corrupt = 0;
	}

	return ret;
}

#ifdef CONFIG_PM
static int __maybe_unused hantro_h1_suspend(struct device *dev)
{
	hx280enc_suspend(dev);
	pm_runtime_put_sync_suspend(dev);   //power off
	return 0;
}
static int __maybe_unused hantro_h1_resume(struct device *dev)
{
	hx280enc_t *hx280enc = dev_get_drvdata(dev);

	hantro_h1_power_on_disirq(hx280enc);
	hantro_h1_ctrlblk_reset(dev);
	hx280enc_resume(dev);
	return 0;
}
static int hantro_h1_runtime_suspend(struct device *dev)
{
	//release_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static int hantro_h1_runtime_resume(struct device *dev)
{
	//request_bus_freq(BUS_FREQ_HIGH);
	hantro_h1_ctrlblk_reset(dev);
	return 0;
}

static const struct dev_pm_ops hantro_h1_pm_ops = {
	SET_RUNTIME_PM_OPS(hantro_h1_runtime_suspend, hantro_h1_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantro_h1_suspend, hantro_h1_resume)
};
#endif //CONFIG_PM

static const struct of_device_id hantro_h1_of_match[] = {
	{ .compatible = "nxp,imx8mm-hantro-h1", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, hantro_h1_of_match);


static struct platform_driver mxchantro_h1_driver = {
	.driver = {
	.name = "mxc_hantro_h1",
	.of_match_table = hantro_h1_of_match,
#ifdef CONFIG_PM
	.pm = &hantro_h1_pm_ops,
#endif
	},
	.probe = hantro_h1_probe,
	.remove = hantro_h1_dev_remove,
};

static int __init hantro_h1_init(void)
{
	int ret = platform_driver_register(&mxchantro_h1_driver);

	return ret;
}

static void __exit hantro_h1_exit(void)
{
	platform_driver_unregister(&mxchantro_h1_driver);
}

module_init(hantro_h1_init);
module_exit(hantro_h1_exit);

#endif

