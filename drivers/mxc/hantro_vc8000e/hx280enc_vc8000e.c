/*
 *  H2 Encoder device driver (kernel module)
 *
 *  COPYRIGHT(C) 2014 VERISILICON
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

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

#include <linux/semaphore.h>
#include <linux/spinlock.h>
/* needed for virt_to_phys() */
#include <asm/io.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>

#include <asm/irq.h>

#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/compat.h>

/* our own stuff */
#include "hx280enc.h"

#ifndef VSI
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/delay.h>
#define DEVICE_NAME		"mxc_hantro_vc8000e"
static struct device *hantro_vc8000e_dev;
static struct clk *hantro_clk_vc8000e;
static struct clk *hantro_clk_vc8000e_bus;
#define BLK_CTL_BASE        0x38330000
#endif


/********variables declaration related with race condition**********/

struct semaphore enc_core_sem;
static DECLARE_WAIT_QUEUE_HEAD(hw_queue);
static DEFINE_SPINLOCK(owner_lock);
static DECLARE_WAIT_QUEUE_HEAD(enc_wait_queue);

/*------------------------------------------------------------------------
*****************************PORTING LAYER********************************
-------------------------------------------------------------------------*/
#define RESOURCE_SHARED_INTER_CORES        0        /*0:no resource sharing inter cores 1: existing resource sharing*/
#define CORE_0_IO_ADDR                 0xC0000000   /*customer specify according to own platform*/
#define CORE_0_IO_SIZE                 (500 * 4)    /* bytes */

#define CORE_1_IO_ADDR                 0xD0000000   /*customer specify according to own platform*/
#define CORE_1_IO_SIZE                 (500 * 4)    /* bytes */

#define INT_PIN_CORE_0                    -1        /*IRQ pin of core 0*/
#define INT_PIN_CORE_1                    -1        /*IRQ pin of core 1*/

/*for all cores, the core info should be listed here for subsequent use*/
/*base_addr, iosize, irq, resource_shared*/
CORE_CONFIG core_array[] = {
	{CORE_0_IO_ADDR, CORE_0_IO_SIZE, INT_PIN_CORE_0, RESOURCE_SHARED_INTER_CORES}, //core_0 (VC8000E)
	//{CORE_1_IO_ADDR, CORE_1_IO_SIZE, INT_PIN_CORE_1, RESOURCE_SHARED_INTER_CORES} //core_1 (VC8000EJ)
};

/*------------------------------END-------------------------------------*/

/***************************TYPE AND FUNCTION DECLARATION****************/

/* here's all the must remember stuff */
typedef struct {
	CORE_CONFIG  core_cfg; //config of each core,such as base addr, irq,etc
	u32 hw_id; //hw id to indicate project
	u32 core_id; //core id for driver and sw internal use
	u32 is_valid; //indicate this core is hantro's core or not
	u32 is_reserved; //indicate this core is occupied by user or not
	struct file *filp; //indicate which instance is occupying the core
	u32 irq_received; //indicate this core receives irq
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	volatile u8 *hwregs;
	u32 reg_buf[CORE_0_IO_SIZE/4];
	struct semaphore core_suspend_sem;
	u32 reg_corrupt;
	struct fasync_struct *async_queue;
#ifndef VSI
	struct device *dev;
	struct mutex dev_mutex;
#endif
} hantroenc_t;

static int ReserveIO(void);
static void ReleaseIO(void);
static void ResetAsic(hantroenc_t *dev);
static int CheckCoreOccupation(hantroenc_t *dev, struct file *filp);

#ifdef hantroenc_DEBUG
static void dump_regs(unsigned long data);
#endif

/* IRQ handler */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
static irqreturn_t hantroenc_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
static irqreturn_t hantroenc_isr(int irq, void *dev_id);
#endif

/*********************local variable declaration*****************/
static unsigned long sram_base;
static unsigned int sram_size;
/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hantroenc_major;
static int total_core_num;
/* dynamic allocation*/
static hantroenc_t *hantroenc_data;
//static unsigned int pcie = 0;          /* used in hantro_mmu.c*/
static struct class *hantro_enc_class;
static struct device *hantro_enc_dev;

/******************************************************************************/
#ifndef VSI
static int hantro_vc8000e_clk_enable(struct device *dev)
{
	clk_prepare(hantro_clk_vc8000e);
	clk_enable(hantro_clk_vc8000e);
	clk_prepare(hantro_clk_vc8000e_bus);
	clk_enable(hantro_clk_vc8000e_bus);
	return 0;
}

static int hantro_vc8000e_clk_disable(struct device *dev)
{
	if (hantro_clk_vc8000e) {
		clk_disable(hantro_clk_vc8000e);
		clk_unprepare(hantro_clk_vc8000e);
	}
	if (hantro_clk_vc8000e_bus) {
		clk_disable(hantro_clk_vc8000e_bus);
		clk_unprepare(hantro_clk_vc8000e_bus);
	}
	return 0;
}

static int hantro_vc8000e_ctrlblk_reset(struct device *dev)
{
	volatile u8 *iobase;
	u32 val;

	//config vc8000e
	hantro_vc8000e_clk_enable(dev);
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

	iowrite32(0xFFFFFFFF, iobase + 0x14); // fuse encoder enable
	iounmap(iobase);
	hantro_vc8000e_clk_disable(dev);
	return 0;
}

static int hantro_vc8000e_power_on_disirq(hantroenc_t *hx280enc)
{
	//spin_lock_irq(&owner_lock);
	mutex_lock(&hx280enc->dev_mutex);
	//disable_irq(hx280enc->irq);
	pm_runtime_get_sync(hx280enc->dev);
	//enable_irq(hx280enc->irq);
	mutex_unlock(&hx280enc->dev_mutex);
	//spin_unlock_irq(&owner_lock);
	return 0;
}

static int hantroenc_mmap(struct file *filp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff == (hantroenc_data[0].core_cfg.base_addr >> PAGE_SHIFT)) {
		vm->vm_flags |= VM_IO;
		vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
		PDEBUG("hx280enc mmap: size=0x%lX, page off=0x%lX\n", (vm->vm_end - vm->vm_start), vm->vm_pgoff);
		return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end - vm->vm_start,
						vm->vm_page_prot) ? -EAGAIN : 0;
	} else {
		pr_err("invalid map offset :0x%lX\n", vm->vm_pgoff);
		return -EINVAL;
	}
}

#endif  //VSI

static int CheckEncIrq(hantroenc_t *dev, u32 *core_info, u32 *irq_status)
{
	unsigned long flags;
	int rdy = 0;
	u32 i = 0;
	u8 core_mapping = 0;

	core_mapping = (u8)(*core_info&0xFF);

	while (core_mapping)	{
		if (core_mapping & 0x1)	{
			if (i > total_core_num-1)
				break;

			spin_lock_irqsave(&owner_lock, flags);

			if (dev[i].irq_received)	{
				/* reset the wait condition(s) */
				PDEBUG("check %d irq ready\n", i);
				dev[i].irq_received = 0;
				rdy = 1;
				*core_info = i;
				*irq_status = dev[i].irq_status;
			}

			spin_unlock_irqrestore(&owner_lock, flags);
			break;
		}
		core_mapping = core_mapping>>1;
		i++;
	}

	return rdy;
}

static int WaitEncReady(hantroenc_t *dev, u32 *core_info, u32 *irq_status)
{
	PDEBUG("%s\n", __func__);

	if (wait_event_timeout(enc_wait_queue,
		CheckEncIrq(dev, core_info, irq_status), msecs_to_jiffies(200)) == 0)	{
		pr_err("%s: wait interrupt timeout !\n", __func__);
		return -1;
	}

	return 0;
}

static int CheckCoreOccupation(hantroenc_t *dev, struct file *filp)
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

static int GetWorkableCore(hantroenc_t *dev, u32 *core_info, u32 *core_info_tmp, struct file *filp)
{
	int ret = 0;
	u32 i = 0;
	u32 cores;
	u32 core_id = 0;
	u8 core_mapping = 0;
	u32 required_num = 0;

	cores = *core_info;
	required_num = ((cores >> CORE_INFO_AMOUNT_OFFSET) & 0x7)+1;
	core_mapping = (u8)(cores & 0xFF);

	if (*core_info_tmp == 0)
		*core_info_tmp = required_num << 8;
	else
		required_num = ((*core_info_tmp & 0xF00) >> 8);

	PDEBUG("%s:required_num=%d,core_info=%x\n", __func__, required_num, *core_info);

	if (required_num) {
		/* a valid free Core that has specified core id */
		while (core_mapping)	{
			if (core_mapping & 0x1)	{
				if (i > total_core_num-1)
					break;
				core_id = i;
				if (dev[core_id].is_valid && CheckCoreOccupation(&dev[core_id], filp)) {
					*core_info_tmp = ((((*core_info_tmp & 0xF00) >> 8)-1)<<8)|(*core_info_tmp & 0x0FF);
					*core_info_tmp = *core_info_tmp | (1<<core_id);
					if (((*core_info_tmp & 0xF00) >> 8) == 0) {
						ret = 1;
						*core_info = (*core_info&0xFFFFFF00)|(*core_info_tmp & 0xFF);
						*core_info_tmp = 0;
						required_num = 0;
						break;
					}
				}
			}
			core_mapping = core_mapping >> 1;
			i++;
		}
	} else
		ret = 1;

	PDEBUG("*core_info = %x\n", *core_info);
	return ret;
}

static long ReserveEncoder(hantroenc_t *dev, u32 *core_info, struct file *filp)
{
	u32 core_info_tmp = 0;

	/*If HW resources are shared inter cores, just make sure only one is using the HW*/
	if (dev[0].core_cfg.resouce_shared)	{
		if (down_interruptible(&enc_core_sem))
			return -ERESTARTSYS;
	}

	/* lock a core that has specified core id*/
	if (wait_event_interruptible(hw_queue, GetWorkableCore(dev, core_info, &core_info_tmp, filp) != 0))
		return -ERESTARTSYS;

	return 0;
}

static void ReleaseEncoder(hantroenc_t *dev, u32 *core_info, struct file *filp)
{
	unsigned long flags;
	u32 core_num = 0;
	u32 i = 0, core_id;
	u8 core_mapping = 0;

	core_num = ((*core_info >> CORE_INFO_AMOUNT_OFFSET) & 0x7)+1;

	core_mapping = (u8)(*core_info&0xFF);

	PDEBUG("%s:core_num=%d,core_mapping=%x\n", __func__, core_num, core_mapping);
	/* release specified core id */
	while (core_mapping) {
		if (core_mapping & 0x1)	{
			core_id = i;
			spin_lock_irqsave(&owner_lock, flags);
			if (dev[core_id].is_reserved && dev[core_id].filp == filp) {
				dev[core_id].filp = NULL;
				dev[core_id].is_reserved = 0;
				dev[core_id].irq_received = 0;
				dev[core_id].irq_status = 0;
				dev[core_id].reg_corrupt = 0;
			} else if (dev[core_id].filp != filp)
				pr_err("WARNING: trying to release core reserved by another instance\n");

			spin_unlock_irqrestore(&owner_lock, flags);

			//wake_up_interruptible_all(&hw_queue);
		}
		core_mapping = core_mapping >> 1;
		i++;
	}

	wake_up_interruptible_all(&hw_queue);

	if (dev->core_cfg.resouce_shared)
		up(&enc_core_sem);

}

static int hantroenc_write_regs(unsigned long arg)
{
	struct enc_regs_buffer regs;
	hantroenc_t *dev;
	u32 *reg_buf;
	u32 i;
	int ret;

	ret = copy_from_user(&regs, (void *)arg, sizeof(regs));
	if (ret)
		return ret;
	if (regs.core_id >= total_core_num ||
	    (regs.offset + regs.size) > sizeof(hantroenc_data[regs.core_id].reg_buf)) {
		pr_err("%s invalid param, core_id:%d, offset:%d, size:%d\n",
			__func__, regs.core_id, regs.offset, regs.size);
		return -EINVAL;
	}

	dev = &hantroenc_data[regs.core_id];
	reg_buf = &dev->reg_buf[regs.offset / 4];
	ret = copy_from_user(reg_buf, (void *)regs.regs, regs.size);
	if (ret)
		return ret;

	for (i = 0; i < regs.size / 4; i++)
		iowrite32(reg_buf[i], (dev->hwregs + regs.offset) + i * 4);

	return ret;
}

static int hantroenc_read_regs(unsigned long arg)
{
	struct enc_regs_buffer regs;
	hantroenc_t *dev;
	u32 *reg_buf;
	u32 i;
	int ret;

	ret = copy_from_user(&regs, (void *)arg, sizeof(regs));
	if (ret)
		return ret;
	if (regs.core_id >= total_core_num ||
	    (regs.offset + regs.size) > sizeof(hantroenc_data[regs.core_id].reg_buf)) {
		pr_err("%s invalid param, core_id:%d, offset:%d, size:%d\n",
			__func__, regs.core_id, regs.offset, regs.size);
		return -EINVAL;
	}

	dev = &hantroenc_data[regs.core_id];
	reg_buf = &dev->reg_buf[regs.offset / 4];

	for (i = 0; i < regs.size / 4; i++)
		reg_buf[i] = ioread32((dev->hwregs + regs.offset) + i * 4);

	ret = copy_to_user((void *)regs.regs, reg_buf, regs.size);

	return ret;
}


static long hantroenc_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int err = 0;

	PDEBUG("ioctl cmd 0x%08x\n", cmd);
	/*
	* extract the type and number bitfields, and don't encode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/
	if (_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC
#ifdef HANTROMMU_SUPPORT
		&& _IOC_TYPE(cmd) != HANTRO_IOC_MMU
#endif
	)
		return -ENOTTY;
	if ((_IOC_TYPE(cmd) == HX280ENC_IOC_MAGIC &&
		_IOC_NR(cmd) > HX280ENC_IOC_MAXNR)
#ifdef HANTROMMU_SUPPORT
		|| (_IOC_TYPE(cmd) == HANTRO_IOC_MMU &&
		_IOC_NR(cmd) > HANTRO_IOC_MMU_MAXNR)
#endif
	)
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

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HX280ENC_IOCGHWOFFSET): {
		u32 id;

		__get_user(id, (u32 *)arg);

		if (id >= total_core_num)
			return -EFAULT;

		__put_user(hantroenc_data[id].core_cfg.base_addr, (unsigned long *) arg);
		break;
	}
	case _IOC_NR(HX280ENC_IOCGHWIOSIZE):	{
		u32 id;
		u32 io_size;

		__get_user(id, (u32 *)arg);

		if (id >= total_core_num)
			return -EFAULT;

		io_size = hantroenc_data[id].core_cfg.iosize;
		__put_user(io_size, (u32 *) arg);

		return 0;
	}
	case _IOC_NR(HX280ENC_IOCGSRAMOFFSET):
		__put_user(sram_base, (unsigned long *) arg);
		break;
	case _IOC_NR(HX280ENC_IOCGSRAMEIOSIZE):
		__put_user(sram_size, (unsigned int *) arg);
		break;
	case _IOC_NR(HX280ENC_IOCG_CORE_NUM):
		__put_user(total_core_num, (unsigned int *) arg);
		break;
	case _IOC_NR(HX280ENC_IOCH_ENC_RESERVE): {
		u32 core_info;
		int ret;

		PDEBUG("Reserve ENC Cores\n");
		__get_user(core_info, (u32 *)arg);
		ret = ReserveEncoder(hantroenc_data, &core_info, filp);
		if (ret == 0)
			__put_user(core_info, (u32 *) arg);
		return ret;
	}
	case _IOC_NR(HX280ENC_IOCH_ENC_RELEASE): {
		u32 core_info;

		__get_user(core_info, (u32 *)arg);

		PDEBUG("Release ENC Core\n");

		ReleaseEncoder(hantroenc_data, &core_info, filp);

		break;
	}
	case _IOC_NR(HX280ENC_IOCG_EN_CORE): {
		u32 core_id;
		u32 reg_value;

		__get_user(core_id, (u32 *)arg);
		PDEBUG("Enable ENC Core\n");

		if (hantroenc_data[core_id].is_reserved == 0)
			return -EPERM;

		if (hantroenc_data[core_id].reg_corrupt)	{
			/*need to re-config HW if exception happen between reserve and enable*/
			hantroenc_data[core_id].reg_corrupt = 0;
			return -EAGAIN;
		}

		if (down_interruptible(&hantroenc_data[core_id].core_suspend_sem))
			return -ERESTARTSYS;

		reg_value = (u32)ioread32((void *)(hantroenc_data[core_id].hwregs + 0x14));
		reg_value |= 0x01;
		iowrite32(reg_value, (void *)(hantroenc_data[core_id].hwregs + 0x14));

		break;
	}

	case _IOC_NR(HX280ENC_IOCG_CORE_WAIT): {
		u32 core_info;
		u32 irq_status;
		u32 i;
		u8 core_mapping;

		__get_user(core_info, (u32 *)arg);

		i = 0;
		core_mapping = (u8)(core_info&0xFF);
		while (core_mapping) {
			if (core_mapping & 0x1)	{
				if (i > total_core_num-1)
					return -1;

				if (hantroenc_data[i].is_reserved == 0)
					return -1;
				break;
			}
			core_mapping = core_mapping>>1;
			i++;
		}
		err = WaitEncReady(hantroenc_data, &core_info, &irq_status);
		if (err == 0) {
			__put_user(irq_status, (unsigned int *)arg);
			return core_info;//return core_id
		} else {
			__put_user(0, (unsigned int *)arg);
			return -1;
		}

		break;
	}
	case _IOC_NR(HX280ENC_IOC_WRITE_REGS): {
		err = hantroenc_write_regs(arg);
		if (err)
			return err;
		break;
	}
	case _IOC_NR(HX280ENC_IOC_READ_REGS): {
		err = hantroenc_read_regs(arg);
		if (err)
			return err;
		break;
	}
	default: {
#ifdef HANTROMMU_SUPPORT
	if (_IOC_TYPE(cmd) == HANTRO_IOC_MMU)
		return (MMUIoctl(cmd, filp, arg, hantroenc_data[0].hwregs));
#endif
	}
	}
	return 0;
}

static int hantroenc_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	hantroenc_t *dev = hantroenc_data;

	filp->private_data = (void *) dev;

#ifndef VSI
	hantro_vc8000e_clk_enable(dev->dev);
	hantro_vc8000e_power_on_disirq(dev);
#endif

	PDEBUG("dev opened\n");
	return result;
}
static int hantroenc_release(struct inode *inode, struct file *filp)
{
	hantroenc_t *dev = (hantroenc_t *) filp->private_data;
	u32 core_id = 0;

#ifdef hantroenc_DEBUG
	dump_regs((unsigned long) dev); /* dump the regs */
#endif
	unsigned long flags;

	PDEBUG("dev closed\n");

	for (core_id = 0; core_id < total_core_num; core_id++) {
		spin_lock_irqsave(&owner_lock, flags);
		if (dev[core_id].is_reserved == 1 && dev[core_id].filp == filp) {
			dev[core_id].filp = NULL;
			dev[core_id].is_reserved = 0;
			dev[core_id].irq_received = 0;
			dev[core_id].irq_status = 0;
			PDEBUG("release reserved core\n");
		}
		spin_unlock_irqrestore(&owner_lock, flags);
	}

#ifdef HANTROMMU_SUPPORT
	MMURelease(filp, hantroenc_data[0].hwregs);
#endif

	wake_up_interruptible_all(&hw_queue);

	if (dev->core_cfg.resouce_shared)
		up(&enc_core_sem);

#ifndef VSI
	pm_runtime_put_sync(hantro_vc8000e_dev);
	hantro_vc8000e_clk_disable(hantro_vc8000e_dev);
#endif
	return 0;
}

#ifdef CONFIG_COMPAT
static long hantroenc_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long err = 0;

#define HX280ENC_IOCTL32(err, filp, cmd, arg) { \
	mm_segment_t old_fs = force_uaccess_begin(); \
	err = hantroenc_ioctl(filp, cmd, arg); \
	if (err) \
		return err; \
	force_uaccess_end(old_fs); \
}
#endif

union {
	unsigned long kux;
	unsigned int kui;
} karg;
	void __user *up = compat_ptr(arg);

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HX280ENC_IOCGHWOFFSET): {
		err = get_user(karg.kux, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kux), (s32 __user *)up);
		break;
	}
	case _IOC_NR(HX280ENC_IOCGHWIOSIZE): {
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kui), (s32 __user *)up);
		break;
	}
	case _IOC_NR(HX280ENC_IOCGSRAMOFFSET): {
		err = get_user(karg.kux, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kux), (s32 __user *)up);
		break;
	}
	case _IOC_NR(HX280ENC_IOCGSRAMEIOSIZE):{
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kui), (s32 __user *)up);
		break;
	}
	case _IOC_NR(HX280ENC_IOCG_CORE_NUM): {
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kui), (s32 __user *)up);
		break;
	}
	case _IOC_NR(HX280ENC_IOCH_ENC_RESERVE): {
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kui), (s32 __user *)up);
		break;
	}
	case _IOC_NR(HX280ENC_IOCH_ENC_RELEASE): {
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		break;
	}
	case _IOC_NR(HX280ENC_IOCG_EN_CORE): {
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		break;
	}

	case _IOC_NR(HX280ENC_IOCG_CORE_WAIT): {
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)up);
		break;
	}

	case _IOC_NR(HX280ENC_IOC_WRITE_REGS): {
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)up);
		break;
	}

	case _IOC_NR(HX280ENC_IOC_READ_REGS): {
		HX280ENC_IOCTL32(err, filp, cmd, (unsigned long)up);
		break;
	}

	}
	return 0;
}

/* VFS methods */
static struct file_operations hantroenc_fops = {
	.owner = THIS_MODULE,
	.open = hantroenc_open,
	.release = hantroenc_release,
	.unlocked_ioctl = hantroenc_ioctl,
	.fasync = NULL,
#ifndef VSI
	.mmap = hantroenc_mmap,
#endif
#ifdef CONFIG_COMPAT
	.compat_ioctl = hantroenc_ioctl32,
#endif
};

static int hantro_enc_suspend(struct device *dev, pm_message_t state)
{
	int i, j;
	u32 *reg_buf;

	PDEBUG("%s start..\n", __func__);

	for (i = 0; i < total_core_num; i++) {
		/*if HW is active, need to wait until frame ready interrupt*/
		if ((hantroenc_data[i].is_reserved == 0) || (down_interruptible(&hantroenc_data[i].core_suspend_sem)))
			continue;

		hantroenc_data[i].reg_corrupt = 1;
		if (hantroenc_data[i].irq_status & 0x04) {
			reg_buf = hantroenc_data[i].reg_buf;
			for (j = 0; j < hantroenc_data[i].core_cfg.iosize; j += 4)
				reg_buf[j/4] = ioread32((void *)(hantroenc_data[i].hwregs + j));
		}

		up(&hantroenc_data[i].core_suspend_sem);
	}

	PDEBUG("%s succeed!\n", __func__);
	return 0;
}

static int hantro_enc_resume(struct device *dev)
{
	int i, j;
	u32 *reg_buf;

	PDEBUG("%s start..\n", __func__);

	for (i = 0; i < total_core_num; i++)	{
		if (hantroenc_data[i].is_reserved == 0)
			continue;
		reg_buf = hantroenc_data[i].reg_buf;

		if (hantroenc_data[i].irq_status & 0x04) {
			for (j = 0; j < hantroenc_data[i].core_cfg.iosize; j += 4)
				iowrite32(reg_buf[j/4], (void *)(hantroenc_data[i].hwregs + j));
			hantroenc_data[i].reg_corrupt = 0;
		}
	}

	PDEBUG("%s succeed!\n", __func__);
	return 0;

}

#ifndef VSI
static int hantroenc_init(void)
#else
static int __init hantroenc_init(void)
#endif
{
	int result = 0;
	int i;

	total_core_num = sizeof(core_array)/sizeof(CORE_CONFIG);
	for (i = 0; i < total_core_num; i++)	{
		PDEBUG("hantroenc: module init - core[%d] addr =%px\n", i,
			(char *)core_array[i].base_addr);
	}

	hantroenc_data = (hantroenc_t *)vmalloc(sizeof(hantroenc_t)*total_core_num);
	if (hantroenc_data == NULL)
		goto err1;
	memset(hantroenc_data, 0, sizeof(hantroenc_t)*total_core_num);

	for (i = 0; i < total_core_num; i++) {
		hantroenc_data[i].core_cfg = core_array[i];
		hantroenc_data[i].async_queue = NULL;
		hantroenc_data[i].hwregs = NULL;
		hantroenc_data[i].core_id = i;
		sema_init(&hantroenc_data[i].core_suspend_sem, 1);
	}

	result = register_chrdev(hantroenc_major, "hx280enc", &hantroenc_fops);
	if (result < 0) {
		pr_err("hx280enc: unable to get major <%d>\n",
		hantroenc_major);
		goto err1;
	} else if (result != 0) /* this is for dynamic major */
		hantroenc_major = result;


	hantro_enc_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(hantro_enc_class)) {
		pr_err("can't register device hx280 class\n");
		goto err2;
	}

	hantro_enc_dev = device_create(hantro_enc_class, NULL, MKDEV(hantroenc_major, 0), NULL, DEVICE_NAME);

	result = ReserveIO();
	if (result < 0)
		goto err;

	ResetAsic(hantroenc_data);  /* reset hardware */

	sema_init(&enc_core_sem, 1);

#ifdef HANTROMMU_SUPPORT
	result = MMUInit(hantroenc_data[0].hwregs);
	if (result == MMU_STATUS_NOT_FOUND)
		pr_err("MMU does not exist!\n");
	else if (result != MMU_STATUS_OK) {
		ReleaseIO();
		goto err;
	}
#endif

	/* get the IRQ line */
	for (i = 0; i < total_core_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;
		if (hantroenc_data[i].core_cfg.irq != -1) {
			result = request_irq(hantroenc_data[i].core_cfg.irq, hantroenc_isr,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
				SA_INTERRUPT | SA_SHIRQ,
#else
				IRQF_SHARED,
#endif
				"hx280enc", (void *) &hantroenc_data[i]);
			if (result == -EINVAL) {
				pr_err("hx280enc: Bad irq number or handler\n");
				ReleaseIO();
				goto err;
			} else if (result == -EBUSY) {
				pr_err("hx280enc: IRQ <%d> busy, change your config\n",
				hantroenc_data[i].core_cfg.irq);
				ReleaseIO();
				goto err;
			}
		} else
			pr_err("hx280enc: IRQ not in use!\n");
	}
	pr_info("hx280enc: module inserted. Major <%d>\n", hantroenc_major);

	return 0;

err:
	device_destroy(hantro_enc_class, MKDEV(hantroenc_major, 0));
	class_destroy(hantro_enc_class);
err2:
	unregister_chrdev(hantroenc_major, "hx280enc");
err1:
	if (hantroenc_data != NULL)
		vfree(hantroenc_data);
	pr_err("hx280enc: module not inserted\n");
	return result;
}

#ifndef VSI
static void hantroenc_cleanup(void)
#else
static void __exit hantroenc_cleanup(void)
#endif
{
	int i = 0;

	for (i = 0; i < total_core_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;
		writel(0, hantroenc_data[i].hwregs + 0x14); /* disable HW */
		writel(0, hantroenc_data[i].hwregs + 0x04); /* clear enc IRQ */

		/* free the encoder IRQ */
		if (hantroenc_data[i].core_cfg.irq != -1)
			free_irq(hantroenc_data[i].core_cfg.irq, (void *)&hantroenc_data[i]);
	}

#ifdef HANTROMMU_SUPPORT
	MMUCleanup(hantroenc_data[0].hwregs);
#endif

	ReleaseIO();
	vfree(hantroenc_data);

	device_destroy(hantro_enc_class, MKDEV(hantroenc_major, 0));

	class_destroy(hantro_enc_class);

	unregister_chrdev(hantroenc_major, "hantroenc");

	pr_info("hantroenc: module removed\n");

}

static int ReserveIO(void)
{
	u32 hwid;
	int i;
	u32 found_hw = 0;

	for (i = 0; i < total_core_num; i++) {
		if (!request_mem_region
			(hantroenc_data[i].core_cfg.base_addr, hantroenc_data[i].core_cfg.iosize, "hx280enc")) {
			pr_err("hantroenc: failed to reserve HW regs\n");
			continue;
		}

		hantroenc_data[i].hwregs =
			(volatile u8 *) ioremap(hantroenc_data[i].core_cfg.base_addr,
		hantroenc_data[i].core_cfg.iosize);

		if (hantroenc_data[i].hwregs == NULL) {
			pr_err("hantroenc: failed to ioremap HW regs\n");
			ReleaseIO();
			continue;
		}

		/*read hwid and check validness and store it*/
		hwid = (u32)ioread32((void *)hantroenc_data[i].hwregs);
		PDEBUG("hwid=0x%08x\n", hwid);

		/* check for encoder HW ID */
		if (((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF))) &&
			((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF)))) {
			pr_err("hantroenc: HW not found at %p\n",
			(void *)hantroenc_data[i].core_cfg.base_addr);
#ifdef hantroenc_DEBUG
			dump_regs((unsigned long) &hantroenc_data);
#endif
			//ReleaseIO();
			hantroenc_data[i].is_valid = 0;
			continue;
		}
		hantroenc_data[i].hw_id = hwid;
		hantroenc_data[i].is_valid = 1;
		found_hw = 1;

		pr_info("hantroenc: HW at base <%px> with ID <0x%08x>\n",
			(char *)hantroenc_data[i].core_cfg.base_addr, hwid);

	}

	if (found_hw == 0) {
		pr_err("hantroenc: NO ANY HW found!!\n");
		return -1;
	}

	return 0;
}

static void ReleaseIO(void)
{
	u32 i;

	for (i = 0; i < total_core_num; i++)	{
		//if (hantroenc_data[i].is_valid == 0)
		//   continue;
		if (hantroenc_data[i].hwregs)
			iounmap((void *) hantroenc_data[i].hwregs);
		release_mem_region(hantroenc_data[i].core_cfg.base_addr, hantroenc_data[i].core_cfg.iosize);
	}
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
static irqreturn_t hantroenc_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t hantroenc_isr(int irq, void *dev_id)
#endif
{
	unsigned int handled = 0;
	hantroenc_t *dev = (hantroenc_t *) dev_id;
	u32 irq_status;
	unsigned long flags;

	/*If core is not reserved by any user, but irq is received, just clean it*/
	spin_lock_irqsave(&owner_lock, flags);
	if (!dev->is_reserved) {
		u32 hwId;
		u32 majorId;
		u32 wClr;

		pr_err("hantroenc_isr:received IRQ but core is not reserved!\n");
		irq_status = (u32)ioread32((void *)(dev->hwregs + 0x04));
		if (irq_status & 0x01) {
			/*  Disable HW when buffer over-flow happen
			*  HW behavior changed in over-flow
			*    in-pass, HW cleanup HWIF_ENC_E auto
			*    new version:  ask SW cleanup HWIF_ENC_E when buffer over-flow
			*/
			if (irq_status & 0x20)
				iowrite32(0, (void *)(dev->hwregs + 0x14));

			/* clear all IRQ bits. (hwId >= 0x80006100) means IRQ is cleared by writting 1 */
			hwId = ioread32((void *)dev->hwregs);
			majorId = (hwId & 0x0000FF00) >> 8;
			wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));
			iowrite32(wClr, (void *)(dev->hwregs + 0x04));
		}
		spin_unlock_irqrestore(&owner_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&owner_lock, flags);

	irq_status = (u32)ioread32((void *)(dev->hwregs + 0x04));
	if (irq_status & 0x01) {
		u32 hwId;
		u32 majorId;
		u32 wClr;
		/*  Disable HW when buffer over-flow happen
		*  HW behavior changed in over-flow
		*    in-pass, HW cleanup HWIF_ENC_E auto
		*    new version:  ask SW cleanup HWIF_ENC_E when buffer over-flow
		*/
		if (irq_status & 0x20)
			iowrite32(0, (void *)(dev->hwregs + 0x14));

		/* clear all IRQ bits. (hwId >= 0x80006100) means IRQ is cleared by writting 1 */
		hwId = ioread32((void *)dev->hwregs);
		majorId = (hwId & 0x0000FF00) >> 8;
		wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));
		iowrite32(wClr, (void *)(dev->hwregs + 0x04));

		spin_lock_irqsave(&owner_lock, flags);
		dev->irq_received = 1;
		dev->irq_status = irq_status & (~0x01);
		spin_unlock_irqrestore(&owner_lock, flags);
		if (irq_status & 0x04)  // if frame_rdy IRQ is received, then HW will not be used any more.
			up(&hantroenc_data[dev->core_id].core_suspend_sem);

		wake_up_all(&enc_wait_queue);
		handled++;
	}

	if (!handled) {
		PDEBUG("IRQ received, but not hantro's!\n");
	}
	return IRQ_HANDLED;
}

static void ResetAsic(hantroenc_t *dev)
{
	int i, n;

	for (n = 0; n < total_core_num; n++) {
		if (dev[n].is_valid == 0)
			continue;
		iowrite32(0, (void *)(dev[n].hwregs + 0x14));
		for (i = 4; i < dev[n].core_cfg.iosize; i += 4)
			iowrite32(0, (void *)(dev[n].hwregs + i));
	}
}

#ifdef hantroenc_DEBUG
static void dump_regs(unsigned long data)
{
	hantroenc_t *dev = (hantroenc_t *) data;
	int i;

	PDEBUG("Reg Dump Start\n");
	for (i = 0; i < dev->iosize; i += 4) {
		PDEBUG("\toffset %02X = %08X\n", i, ioread32(dev->hwregs + i));
	}
	PDEBUG("Reg Dump End\n");
}
#endif

#ifndef VSI
static int hantro_vc8000e_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;

	hantro_vc8000e_dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs_hantro_vc8000e");
	if (!res) {
		pr_err("hantro vc8000e: unable to get vpu base addr\n");
		return -ENODEV;
	}

	core_array[0].base_addr = res->start;
	core_array[0].irq = platform_get_irq_byname(pdev, "irq_hantro_vc8000e");
	hantro_clk_vc8000e = clk_get(&pdev->dev, "clk_hantro_vc8000e");
	if (IS_ERR(hantro_clk_vc8000e)) {
		pr_err("hantro vc8000e: get clock failed, %p\n", hantro_clk_vc8000e);
		err = -ENXIO;
		goto error;
	}
	hantro_clk_vc8000e_bus = clk_get(&pdev->dev, "clk_hantro_vc8000e_bus");
	if (IS_ERR(hantro_clk_vc8000e_bus)) {
		pr_err("hantro vc8000e: get bus clock failed, %p\n", hantro_clk_vc8000e_bus);
		err = -ENXIO;
		goto error;
	}

	PDEBUG("hantro: vc8000e clock: 0x%lX, 0x%lX\n", clk_get_rate(hantro_clk_vc8000e), clk_get_rate(hantro_clk_vc8000e_bus));

	hantro_vc8000e_clk_enable(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	hantro_vc8000e_ctrlblk_reset(&pdev->dev);

	err = hantroenc_init();
	if (0 != err) {
		pr_err("hantro vc8000e: init failed\n");
		goto error;
	}

	hantroenc_data->dev = &pdev->dev;
	platform_set_drvdata(pdev, hantroenc_data);
	mutex_init(&hantroenc_data->dev_mutex);

	goto out;

error:
	pr_err("hantro vc8000e probe failed\n");
out:
	pm_runtime_put_sync(&pdev->dev);
	hantro_vc8000e_clk_disable(&pdev->dev);
	return err;
}

static int hantro_vc8000e_dev_remove(struct platform_device *pdev)
{
	hantro_vc8000e_clk_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	if (hantroenc_major > 0) {
		hantroenc_cleanup();
		hantroenc_major = 0;
	}
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	hantro_vc8000e_clk_disable(&pdev->dev);
	if (!IS_ERR(hantro_clk_vc8000e))
		clk_put(hantro_clk_vc8000e);
	if (!IS_ERR(hantro_clk_vc8000e_bus))
		clk_put(hantro_clk_vc8000e_bus);
	return 0;
}

#ifdef CONFIG_PM
static int __maybe_unused hantro_vc8000e_suspend(struct device *dev)
{
	pm_message_t state = {0};

	hantro_enc_suspend(dev, state);
	pm_runtime_put_sync_suspend(dev);   //power off
	return 0;
}
static int __maybe_unused hantro_vc8000e_resume(struct device *dev)
{
	hantroenc_t *hx280enc = dev_get_drvdata(dev);

	hantro_vc8000e_power_on_disirq(hx280enc);
	hantro_vc8000e_ctrlblk_reset(dev);

	hantro_enc_resume(dev);

	return 0;
}
static int hantro_vc8000e_runtime_suspend(struct device *dev)
{
	//release_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static int hantro_vc8000e_runtime_resume(struct device *dev)
{
	//request_bus_freq(BUS_FREQ_HIGH);
	hantro_vc8000e_ctrlblk_reset(dev);
	return 0;
}

static const struct dev_pm_ops hantro_vc8000e_pm_ops = {
	SET_RUNTIME_PM_OPS(hantro_vc8000e_runtime_suspend, hantro_vc8000e_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantro_vc8000e_suspend, hantro_vc8000e_resume)
};
#endif //CONFIG_PM

static const struct of_device_id hantro_vc8000e_of_match[] = {
	{ .compatible = "nxp,imx8mp-hantro-vc8000e", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, hantro_vc8000e_of_match);


static struct platform_driver mxchantro_vc8000e_driver = {
	.driver = {
	.name = "mxc_hantro_vc8000e",
	.of_match_table = hantro_vc8000e_of_match,
#ifdef CONFIG_PM
	.pm = &hantro_vc8000e_pm_ops,
#endif
	},
	.probe = hantro_vc8000e_probe,
	.remove = hantro_vc8000e_dev_remove,
};

static int __init hantro_vc8000e_init(void)
{
	int ret = platform_driver_register(&mxchantro_vc8000e_driver);

	return ret;
}

static void __exit hantro_vc8000e_exit(void)
{
	platform_driver_unregister(&mxchantro_vc8000e_driver);
}

module_init(hantro_vc8000e_init);
module_exit(hantro_vc8000e_exit);

#else  //VSI
module_init(hantroenc_init);
module_exit(hantroenc_cleanup);

#endif  //VSI

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("Hantro Encoder driver");

