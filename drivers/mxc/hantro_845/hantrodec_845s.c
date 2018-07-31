/*****************************************************************************
 *    The GPL License (GPL)
 *
 *    Copyright (c) 2015-2018, VeriSilicon Inc.
 *    Copyright (c) 2011-2014, Google Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 *****************************************************************************/
#include <linux/hantrodec.h>
#include "dwl_defs.h"
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/busfreq-imx.h>

#include <linux/delay.h>

#ifdef CONFIG_DEVICE_THERMAL_XXX
#include <linux/device_cooling.h>
#define HANTRO_REG_THERMAL_NOTIFIER(a) register_devfreq_cooling_notifier(a)
#define HANTRO_UNREG_THERMAL_NOTIFIER(a) unregister_devfreq_cooling_notifier(a)
DEFINE_SPINLOCK(thermal_lock);
/*1:hot, 0: not hot*/
static int thermal_event;
static int thermal_cur;
static int hantro_clock_ratio = 2;
static int hantro_dynamic_clock;
module_param(hantro_clock_ratio, int, 0644);
module_param(hantro_dynamic_clock, int, 0644);
MODULE_PARM_DESC(hantro_clock_ratio, "clock ratio 1/N");
MODULE_PARM_DESC(hantro_dynamic_clock, "enable or disable dynamic clock rate");
#endif

/*hantro G1 regs config including dec and pp*/
#define HANTRO_DEC_ORG_REGS             60
#define HANTRO_PP_ORG_REGS              41

#define HANTRO_DEC_EXT_REGS             27
#define HANTRO_PP_EXT_REGS              9

#define HANTRO_G1_DEC_TOTAL_REGS        (HANTRO_DEC_ORG_REGS + HANTRO_DEC_EXT_REGS)
#define HANTRO_PP_TOTAL_REGS            (HANTRO_PP_ORG_REGS + HANTRO_PP_EXT_REGS)
#define HANTRO_G1_TOTAL_REGS             155 /*G1 total regs*/

#define HANTRO_DEC_ORG_FIRST_REG        0
#define HANTRO_DEC_ORG_LAST_REG         59
#define HANTRO_DEC_EXT_FIRST_REG        119
#define HANTRO_DEC_EXT_LAST_REG         145

#define HANTRO_PP_ORG_FIRST_REG         60
#define HANTRO_PP_ORG_LAST_REG          100
#define HANTRO_PP_EXT_FIRST_REG         146
#define HANTRO_PP_EXT_LAST_REG          154

/*hantro G2 reg config*/
#define HANTRO_G2_DEC_REGS                 265 /*G2 total regs*/

#define HANTRO_G2_DEC_FIRST_REG            0
#define HANTRO_G2_DEC_LAST_REG             (HANTRO_G2_DEC_REGS - 1)

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define DEC_IO_SIZE_MAX             (MAX(HANTRO_G2_DEC_REGS, HANTRO_G1_TOTAL_REGS) * 4)

/********************************************************************
 *                                              PORTING SEGMENT
 * NOTES: customer should modify these configuration if do porting to own platform.
 * Please guarantee the base_addr, io_size,dec_irq belong to same core.
 ********************************************************************/

#define HXDEC_MAX_CORES                 2

/* Logic module base address */
#define SOCLE_LOGIC_0_BASE              0x38300000
#define SOCLE_LOGIC_1_BASE              0x38310000
#define BLK_CTL_BASE                          0x38330000 //0x38320000

#define VEXPRESS_LOGIC_0_BASE           0xFC010000
#define VEXPRESS_LOGIC_1_BASE           0xFC020000

#define DEC_IO_SIZE_0             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */
#define DEC_IO_SIZE_1             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */

#define HANTRO_G1_DEF_CLK		(600000000)
#define HANTRO_G2_DEF_CLK		(600000000)
#define HANTRO_BUS_DEF_CLK	(800000000)
/***********************************************************************/

#define IS_G1(hw_id)    ((hw_id == 0x6731) ? 1:0)

static const int DecHwId[] = {
	0x8190, /* Legacy HW */
	0x8170,
	0x9170,
	0x9190,
	0x6731, /* G1 */
	0x6732 /* G2 */
};

static ulong multicorebase[HXDEC_MAX_CORES] = {
	SOCLE_LOGIC_0_BASE,
	SOCLE_LOGIC_1_BASE
};


static struct class *hantro_class;
#define DEVICE_NAME		"mxc_hantro"

//static struct device *hantro_dev[HXDEC_MAX_CORES];

typedef struct {
	struct clk *dec;
	struct clk *bus;
} hantrodec_clk;

static int hantro_dbg = -1;
module_param(hantro_dbg, int, 0644);
MODULE_PARM_DESC(hantro_dbg, "Debug level (0-1)");
#undef PDEBUG
#define PDEBUG(fmt, arg...)     \
	do {                                      \
		if (hantro_dbg > 0) { \
			dev_info(hantrodec_data[0].dev, fmt, ## arg); \
		} \
	} while (0)


static int hantrodec_major;
static int cores = 2;
/* here's all the must remember stuff */
typedef struct {
	//char *buffer;
	unsigned int iosize;
	volatile u8 *hwregs;
	int irq;
	int hw_id;
	int core_id;
	//int cores;
	//struct fasync_struct *async_queue_dec;
	//struct fasync_struct *async_queue_pp;
	hantrodec_clk clk;
	u32 dec_regs[DEC_IO_SIZE_MAX/4];
	struct semaphore dec_core_sem;
	struct semaphore pp_core_sem;
	struct file *dec_owner;
	struct file *pp_owner;
	u32 cfg;
	u32 timeout;
	atomic_t irq_rx;
	atomic_t irq_tx;
	struct device *dev;
	struct mutex dev_mutex;
} hantrodec_t;

static hantrodec_t hantrodec_data[HXDEC_MAX_CORES]; /* dynamic allocation? */

typedef struct {
	char inst_id;
	char core_id;	//1:g1; 2:g2; 3:unknow
} hantrodec_instance;
static unsigned long instance_mask;
#define MAX_HANTRODEC_INSTANCE 32
static hantrodec_instance hantrodec_ctx[MAX_HANTRODEC_INSTANCE];

static int ReserveIO(int);
static void ReleaseIO(int);

static void ResetAsic(hantrodec_t *dev);

#ifdef HANTRODEC_DEBUG
static void dump_regs(hantrodec_t *dev);
#endif

/* IRQ handler */
static irqreturn_t hantrodec_isr(int irq, void *dev_id);

static int dec_irq;
static int pp_irq;


/* spinlock_t owner_lock = SPIN_LOCK_UNLOCKED; */
static DEFINE_SPINLOCK(owner_lock);

static DECLARE_WAIT_QUEUE_HEAD(dec_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(pp_wait_queue);

static DECLARE_WAIT_QUEUE_HEAD(hw_queue);

#define DWL_CLIENT_TYPE_H264_DEC         1U
#define DWL_CLIENT_TYPE_MPEG4_DEC        2U
#define DWL_CLIENT_TYPE_JPEG_DEC         3U
#define DWL_CLIENT_TYPE_PP               4U
#define DWL_CLIENT_TYPE_VC1_DEC          5U
#define DWL_CLIENT_TYPE_MPEG2_DEC        6U
#define DWL_CLIENT_TYPE_VP6_DEC          7U
#define DWL_CLIENT_TYPE_AVS_DEC          8U
#define DWL_CLIENT_TYPE_RV_DEC           9U
#define DWL_CLIENT_TYPE_VP8_DEC          10U
#define DWL_CLIENT_TYPE_VP9_DEC          11U
#define DWL_CLIENT_TYPE_HEVC_DEC         12U

static int hantro_device_id(struct device *dev)
{
	int id;

	if (strcmp("vpu_g1", dev->of_node->name) == 0) {
		id = 0;
	} else if (strcmp("vpu_g2", dev->of_node->name) == 0) {
		id = 1;
	} else {
		return id = -1;
	}
	return id;
}
static int hantro_clk_enable(hantrodec_clk *clk)
{
	clk_prepare(clk->dec);
	clk_enable(clk->dec);
	clk_prepare(clk->bus);
	clk_enable(clk->bus);
	return 0;
}

static int hantro_clk_disable(hantrodec_clk *clk)
{
	clk_disable(clk->dec);
	clk_unprepare(clk->dec);
	clk_disable(clk->bus);
	clk_unprepare(clk->bus);
	return 0;
}

static int hantro_ctrlblk_reset(hantrodec_t *dev)
{
	volatile u8 *iobase;
	u32 val;

	//config G1/G2
	hantro_clk_enable(&dev->clk);
	iobase = (volatile u8 *)ioremap_nocache(BLK_CTL_BASE, 0x10000);
	if (dev->core_id == 0) {
		val = ioread32(iobase);
		val &= (~0x2);
		iowrite32(val, iobase);  //assert G1 block soft reset  control
		udelay(2);
		val = ioread32(iobase);
		val |= 0x2;
		iowrite32(val, iobase);  //desert G1 block soft reset  control

		val = ioread32(iobase+4);
		val |= 0x2;
		iowrite32(val, iobase+4); //VPUMIX G1 block clock enable control
		iowrite32(0xFFFFFFFF, iobase + 0x8); // all G1 fuse dec enable
		iowrite32(0xFFFFFFFF, iobase + 0xC); // all G1 fuse pp enable
	} else {
		val = ioread32(iobase);
		val &= (~0x1);
		iowrite32(val, iobase);  //assert G2 block soft reset  control
		udelay(2);
		val = ioread32(iobase);
		val |= 0x1;
		iowrite32(val, iobase);  //desert G2 block soft reset  control

		val = ioread32(iobase+4);
		val |= 0x1;
		iowrite32(val, iobase+4); //VPUMIX G2 block clock enable control
		iowrite32(0xFFFFFFFF, iobase + 0x10); // all G2 fuse dec enable
	}
	iounmap(iobase);
	hantro_clk_disable(&dev->clk);
	return 0;
}

static int hantro_power_on_disirq(hantrodec_t *hantrodev)
{
	//spin_lock_irq(&owner_lock);
	mutex_lock(&hantrodev->dev_mutex);
	disable_irq(hantrodev->irq);
	pm_runtime_get_sync(hantrodev->dev);
	enable_irq(hantrodev->irq);
	mutex_unlock(&hantrodev->dev_mutex);
	//spin_unlock_irq(&owner_lock);
	return 0;
}

static int hantro_new_instance(void)
{
	int idx;

	spin_lock(&owner_lock);
	if (instance_mask  == ((1UL << MAX_HANTRODEC_INSTANCE) - 1)) {
		spin_unlock(&owner_lock);
		return -1;
	}
	idx = ffz(instance_mask);
	set_bit(idx, &instance_mask);
	spin_unlock(&owner_lock);
	return idx;
}

static int hantro_free_instance(int idx)
{
	spin_lock(&owner_lock);
	clear_bit(idx, &instance_mask);
	spin_unlock(&owner_lock);

	return 0;
}

#ifdef CONFIG_DEVICE_THERMAL_XXX
static int hantro_thermal_check(struct device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&thermal_lock, flags);
	if (thermal_event == thermal_cur) {
		/*nothing to do and return directly*/
		spin_unlock_irqrestore(&thermal_lock, flags);
		return 0;
	}
	thermal_cur = thermal_event;
	spin_unlock_irqrestore(&thermal_lock, flags);

	if (thermal_cur) {
		int ratio = hantro_clock_ratio;

		pr_debug("hantro: too hot, need to decrease clock, ratio: 1/%d\n", ratio);
		/*clock disable/enable are not required for vpu clock rate operation*/
		clk_set_rate(hantro_clk_g1, HANTRO_G1_DEF_CLK/ratio);
		clk_set_rate(hantro_clk_g2, HANTRO_G2_DEF_CLK/ratio);
		clk_set_rate(hantro_clk_bus, HANTRO_BUS_DEF_CLK/ratio);
	} else {
		pr_debug("hantro: not hot again, will restore default clock\n");
		clk_set_rate(hantro_clk_g1, HANTRO_G1_DEF_CLK);
		clk_set_rate(hantro_clk_g2, HANTRO_G2_DEF_CLK);
		clk_set_rate(hantro_clk_bus, HANTRO_BUS_DEF_CLK);
	}
	pr_info("hantro: event(%d), g1, g2, bus clock: %ld, %ld, %ld\n", thermal_cur,
		clk_get_rate(hantro_clk_g1),	clk_get_rate(hantro_clk_g2), clk_get_rate(hantro_clk_bus));
	return 0;
}

static int hantro_thermal_hot_notify(struct notifier_block *nb, unsigned long event, void *dummy)
{
	unsigned long flags;

	spin_lock_irqsave(&thermal_lock, flags);
	thermal_event = event;		/*event: 1: hot, 0: cool*/
	spin_unlock_irqrestore(&thermal_lock, flags);
	pr_info("hantro receive hot notification event: %ld\n", event);

	return NOTIFY_OK;
}

static struct notifier_block hantro_thermal_hot_notifier = {
	.notifier_call = hantro_thermal_hot_notify,
};
#endif  //CONFIG_DEVICE_THERMAL_XXX

static void ReadCoreConfig(hantrodec_t *dev)
{
	int c = dev->core_id;
	u32 reg, tmp, mask;

	memset(&dev->cfg, 0, sizeof(dev->cfg));

	//for (c = 0; c < dev->cores; c++) {
		/* Decoder configuration */
		if (IS_G1(dev->hw_id)) {
			reg = ioread32(dev->hwregs + HANTRODEC_SYNTH_CFG * 4);

			tmp = (reg >> DWL_H264_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has H264\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has JPEG\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_MPEG4_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has MPEG4\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

			tmp = (reg >> DWL_VC1_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has VC1\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

			tmp = (reg >> DWL_MPEG2_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has MPEG2\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

			tmp = (reg >> DWL_VP6_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has VP6\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

			reg = ioread32(dev->hwregs + HANTRODEC_SYNTH_CFG_2 * 4);

			/* VP7 and WEBP is part of VP8 */
			mask =  (1 << DWL_VP8_E) | (1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
			tmp = (reg & mask);
			if (tmp & (1 << DWL_VP8_E))
				pr_debug("hantrodec: Core[%d] has VP8\n", c);
			if (tmp & (1 << DWL_VP7_E))
				pr_debug("hantrodec: Core[%d] has VP7\n", c);
			if (tmp & (1 << DWL_WEBP_E))
				pr_debug("hantrodec: Core[%d] has WebP\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

			tmp = (reg >> DWL_AVS_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has AVS\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

			tmp = (reg >> DWL_RV_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has RV\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

			/* Post-processor configuration */
			reg = ioread32(dev->hwregs + HANTROPP_SYNTH_CFG * 4);
		} else {
			reg = ioread32(dev->hwregs + HANTRODEC_SYNTH_CFG_2 * 4);

			tmp = (reg >> DWL_HEVC_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has HEVC\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

			tmp = (reg >> DWL_VP9_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has VP9\n", c);
			dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;
		}

		/* Post-processor configuration */
		reg = ioread32(dev->hwregs + HANTRODECPP_SYNTH_CFG * 4);

		tmp = (reg >> DWL_PP_E) & 0x01U;
		if (tmp)
			pr_debug("hantrodec: Core[%d] has PP\n", c);
		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
	//}
}

static int CoreHasFormat(const u32 cfg, u32 format)
{
	return (cfg & (1 << format)) ? 1 : 0;
}

static int GetDecCore(hantrodec_t *dev, struct file *filp)
{
	int success = 0;
	unsigned long flags;

	spin_lock_irqsave(&owner_lock, flags);
	if (dev->dec_owner == NULL) {
		dev->dec_owner = filp;
		success = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return success;
}

static int GetDecCoreAny(long *Core, hantrodec_t *dev, struct file *filp,
			unsigned long format) {
	int success = 0;
	//long c;

	*Core = -1;

	//for (c = 0; c < dev->cores; c++) {
		/* a free Core that has format */
		if (CoreHasFormat(dev->cfg, format) && GetDecCore(dev, filp)) {
			success = 1;
			*Core = dev->core_id;
			//break;
		}
	//}

	return success;
}

static int GetDecCoreID(unsigned long format)
{
	long c;

	int core_id = -1;

	for (c = 0; c < cores; c++) {
		/* a Core that has format */
		if (CoreHasFormat(hantrodec_data[c].cfg, format)) {
			core_id = c;
			break;
		}
	}
	PDEBUG("GetDecCoreID=%d\n", core_id);
	return core_id;
}
#if 0
static int hantrodec_choose_core(int is_g1)
{
	volatile unsigned char *reg = NULL;
	unsigned int blk_base = BLK_CTL_BASE;

	PDEBUG("hantrodec_choose_core\n");
	if (!request_mem_region(blk_base, 0x1000, "blk_ctl"))	{
		pr_err("blk_ctl: failed to reserve HW regs\n");
		return -EBUSY;
	}

	reg = (volatile u8 *) ioremap_nocache(blk_base, 0x1000);

	if (reg == NULL) {
		pr_err("blk_ctl: failed to ioremap HW regs\n");
		if (reg)
			iounmap((void *)reg);
		release_mem_region(blk_base, 0x1000);
		return -EBUSY;
	}

	// G1 use, set to 1; G2 use, set to 0, choose the one you are using
	if (is_g1)
		iowrite32(0x1, reg + 0x14);  // VPUMIX only use G1
	else
		iowrite32(0x0, reg + 0x14); // VPUMIX only use G2

	if (reg)
		iounmap((void *)reg);
	release_mem_region(blk_base, 0x1000);
	PDEBUG("hantrodec_choose_core OK!\n");
	return 0;
}
#endif

static long ReserveDecoder(hantrodec_t *dev, struct file *filp, unsigned long format)
{
	long Core = -1;

	/* reserve a Core */
	if (down_interruptible(&dev->dec_core_sem))
		return -ERESTARTSYS;

#if 1
	if (GetDecCoreAny(&Core, dev, filp, format) == 0) {
		pr_err("Core %d is already been reserved !\n", dev->core_id);
		return -1;
	}
#else
	/* lock a Core that has specific format*/
	if (wait_event_interruptible(hw_queue, GetDecCoreAny(&Core, dev, filp, format) != 0))
		return -ERESTARTSYS;
#endif
#if 0
	if (IS_G1(dev->hw_id)) {
		if (0 == hantrodec_choose_core(1))
			PDEBUG("G1 is reserved\n");
		else
			return -1;
	} else {
		if (0 == hantrodec_choose_core(0))
			PDEBUG("G2 is reserved\n");
		else
			return -1;
	}
#endif
#ifdef CONFIG_DEVICE_THERMAL_XXX
	if (hantro_dynamic_clock)
		hantro_thermal_check(hantro_dev);
#endif

	return Core;
}

static void ReleaseDecoder(hantrodec_t *dev)
{
	u32 status;
	unsigned long flags;

	status = ioread32(dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

	/* make sure HW is disabled */
	if (status & HANTRODEC_DEC_E) {
		pr_info("hantrodec: DEC[%d] still enabled -> reset\n", dev->core_id);

		/* abort decoder */
		status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status, dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);
	}

	spin_lock_irqsave(&owner_lock, flags);

	dev->dec_owner = NULL;

	spin_unlock_irqrestore(&owner_lock, flags);

	up(&dev->dec_core_sem);

	//wake_up_interruptible_all(&hw_queue);

}

static long ReservePostProcessor(hantrodec_t *dev, struct file *filp)
{
	unsigned long flags;

	long Core = 0;

	/* single Core PP only */
	if (down_interruptible(&dev->pp_core_sem))
		return -ERESTARTSYS;

	spin_lock_irqsave(&owner_lock, flags);

	dev->pp_owner = filp;

	spin_unlock_irqrestore(&owner_lock, flags);

	return Core;
}

static void ReleasePostProcessor(hantrodec_t *dev)
{
	unsigned long flags;

	u32 status = ioread32(dev->hwregs + HANTRO_IRQ_STAT_PP_OFF);

	/* make sure HW is disabled */
	if (status & HANTRO_PP_E) {
		pr_info("hantrodec: PP[%d] still enabled -> reset\n", dev->core_id);

		/* disable IRQ */
		status |= HANTRO_PP_IRQ_DISABLE;

		/* disable postprocessor */
		status &= (~HANTRO_PP_E);
		iowrite32(0x10, dev->hwregs + HANTRO_IRQ_STAT_PP_OFF);
	}

	spin_lock_irqsave(&owner_lock, flags);

	dev->pp_owner = NULL;

	spin_unlock_irqrestore(&owner_lock, flags);

	up(&dev->pp_core_sem);
}

#if 0
static long ReserveDecPp(hantrodec_t *dev, struct file *filp, unsigned long format)
{
	/* reserve Core 0, DEC+PP for pipeline */
	unsigned long flags;

	long Core = 0;

	/* check that Core has the requested dec format */
	if (!CoreHasFormat(dev->cfg, format))
		return -EFAULT;

	/* check that Core has PP */
	if (!CoreHasFormat(dev->cfg, DWL_CLIENT_TYPE_PP))
		return -EFAULT;

	/* reserve a Core */
	if (down_interruptible(&dev->dec_core_sem))
		return -ERESTARTSYS;

	/* wait until the Core is available */
	if (wait_event_interruptible(hw_queue,	GetDecCore(dev, filp) != 0)) {
		up(&dev->dec_core_sem);
		return -ERESTARTSYS;
	}

	if (down_interruptible(&dev->pp_core_sem)) {
		ReleaseDecoder(dev);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&owner_lock, flags);
	dev->pp_owner = filp;
	spin_unlock_irqrestore(&owner_lock, flags);

	return Core;
}
#endif
static long DecFlushRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long ret = 0, i;

	//u32 id = Core->id;

	if (IS_G1(dev->hw_id)) {
		/* copy original dec regs to kernal space*/
		ret = copy_from_user(dev->dec_regs, Core->regs, HANTRO_DEC_ORG_REGS*4);
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}
#ifdef USE_64BIT_ENV
		/* copy extended dec regs to kernal space*/
		ret = copy_from_user(dev->dec_regs + HANTRO_DEC_EXT_FIRST_REG,
				Core->regs + HANTRO_DEC_EXT_FIRST_REG, HANTRO_DEC_EXT_REGS * 4);
#endif
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}

		/* write dec regs but the status reg[1] to hardware */
		/* both original and extended regs need to be written */
		for (i = 2; i <= HANTRO_DEC_ORG_LAST_REG; i++)
			iowrite32(dev->dec_regs[i], dev->hwregs + i*4);
#ifdef USE_64BIT_ENV
		for (i = HANTRO_DEC_EXT_FIRST_REG; i <= HANTRO_DEC_EXT_LAST_REG; i++)
			iowrite32(dev->dec_regs[i], dev->hwregs + i*4);
#endif
	} else {
		ret = copy_from_user(dev->dec_regs, Core->regs, HANTRO_G2_DEC_REGS*4);
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}

		/* write all regs but the status reg[1] to hardware */
		for (i = 2; i <= HANTRO_G2_DEC_LAST_REG; i++)
			iowrite32(dev->dec_regs[i], dev->hwregs + i*4);
	}

	/* write the status register, which may start the decoder */
	iowrite32(dev->dec_regs[1], dev->hwregs + 4);

	PDEBUG("flushed registers on Core %d\n", dev->core_id);

	return 0;
}

static long DecRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long ret, i;
	//u32 id = Core->id;

	if (IS_G1(dev->hw_id)) {
		/* user has to know exactly what they are asking for */
		//if(Core->size != (HANTRO_DEC_ORG_REGS * 4))
		//  return -EFAULT;

		/* read all registers from hardware */
		/* both original and extended regs need to be read */
		for (i = 0; i <= HANTRO_DEC_ORG_LAST_REG; i++)
			dev->dec_regs[i] = ioread32(dev->hwregs + i*4);
#ifdef USE_64BIT_ENV
		for (i = HANTRO_DEC_EXT_FIRST_REG; i <= HANTRO_DEC_EXT_LAST_REG; i++)
			dev->dec_regs[i] = ioread32(dev->hwregs + i*4);
#endif

		if (dev->timeout) {
			/* Enable TIMEOUT bits in Reg[1] */
			dev->dec_regs[1] = 0x40100;
			/* Reset HW */
			ResetAsic(dev);
			dev->timeout = 0;
		}

		/* put registers to user space*/
		/* put original registers to user space*/
		ret = copy_to_user(Core->regs, dev->dec_regs, HANTRO_DEC_ORG_REGS*4);
#ifdef USE_64BIT_ENV
		/*put extended registers to user space*/
		ret = copy_to_user(Core->regs + HANTRO_DEC_EXT_FIRST_REG,
				dev->dec_regs + HANTRO_DEC_EXT_FIRST_REG, HANTRO_DEC_EXT_REGS * 4);
#endif
		if (ret) {
			pr_err("copy_to_user failed, returned %li\n", ret);
			return -EFAULT;
		}
	} else {
	/* user has to know exactly what they are asking for */
		if (Core->size != (HANTRO_G2_DEC_REGS * 4))
			return -EFAULT;

		/* read all registers from hardware */
		for (i = 0; i <= HANTRO_G2_DEC_LAST_REG; i++)
			dev->dec_regs[i] = ioread32(dev->hwregs + i*4);

		if (dev->timeout) {
			/* Enable TIMEOUT bits in Reg[1] */
			dev->dec_regs[1] = 0x40100;
			/* Reset HW */
			ResetAsic(dev);
			dev->timeout = 0;
		}

		/* put registers to user space*/
		ret = copy_to_user(Core->regs, dev->dec_regs, HANTRO_G2_DEC_REGS*4);
		if (ret) {
			pr_err("copy_to_user failed, returned %li\n", ret);
			return -EFAULT;
		}
	}
	return 0;
}

static int CheckDecIrq(hantrodec_t *dev)
{
	unsigned long flags;
	int rdy = 0;

	const u32 irq_mask = (1 << dev->core_id);

	spin_lock_irqsave(&owner_lock, flags);

	if (dec_irq & irq_mask) {
		/* reset the wait condition(s) */
		dec_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return rdy;
}

static long WaitDecReadyAndRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	//u32 id = Core->id;
	long ret;

	PDEBUG("wait_event_interruptible DEC[%d]\n", dev->core_id);

	ret = wait_event_interruptible_timeout(dec_wait_queue, CheckDecIrq(dev), msecs_to_jiffies(200));
	if (ret == -ERESTARTSYS) {
		pr_err("DEC[%d]  failed to wait_event_interruptible interrupted\n", dev->core_id);
		return -ERESTARTSYS;
	} else if (ret == 0) {
		pr_err("DEC[%d]  wait_event_interruptible timeout\n", dev->core_id);
		dev->timeout = 1;
	}

	atomic_inc(&dev->irq_tx);

	/* refresh registers */
	return DecRefreshRegs(dev, Core);
}

static long PPFlushRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long ret = 0;
	//u32 id = Core->id;
	u32 i;

	/* copy original dec regs to kernal space*/
	ret = copy_from_user(dev->dec_regs + HANTRO_PP_ORG_FIRST_REG,
			Core->regs + HANTRO_PP_ORG_FIRST_REG, HANTRO_PP_ORG_REGS*4);
#ifdef USE_64BIT_ENV
	/* copy extended dec regs to kernal space*/
	ret = copy_from_user(dev->dec_regs + HANTRO_PP_EXT_FIRST_REG,
			Core->regs + HANTRO_PP_EXT_FIRST_REG, HANTRO_PP_EXT_REGS*4);
#endif
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	/* both original and extended regs need to be written */
	for (i = HANTRO_PP_ORG_FIRST_REG + 1; i <= HANTRO_PP_ORG_LAST_REG; i++)
		iowrite32(dev->dec_regs[i], dev->hwregs + i*4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
		iowrite32(dev->dec_regs[i], dev->hwregs + i*4);
#endif
	/* write the stat reg, which may start the PP */
	iowrite32(dev->dec_regs[HANTRO_PP_ORG_FIRST_REG],
	dev->hwregs + HANTRO_PP_ORG_FIRST_REG * 4);

	return 0;
}

static long PPRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long i, ret;
	//u32 id = Core->id;
#ifdef USE_64BIT_ENV
	/* user has to know exactly what they are asking for */
	if (Core->size != (HANTRO_PP_TOTAL_REGS * 4))
		return -EFAULT;
#else
	/* user has to know exactly what they are asking for */
	if (Core->size != (HANTRO_PP_ORG_REGS * 4))
		return -EFAULT;
#endif

	/* read all registers from hardware */
	/* both original and extended regs need to be read */
	for (i = HANTRO_PP_ORG_FIRST_REG; i <= HANTRO_PP_ORG_LAST_REG; i++)
		dev->dec_regs[i] = ioread32(dev->hwregs + i*4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
		dev->dec_regs[i] = ioread32(dev->hwregs + i*4);
#endif
	/* put registers to user space*/
	/* put original registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_ORG_FIRST_REG,
			dev->dec_regs + HANTRO_PP_ORG_FIRST_REG, HANTRO_PP_ORG_REGS*4);
#ifdef USE_64BIT_ENV
	/* put extended registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_EXT_FIRST_REG,
			dev->dec_regs + HANTRO_PP_EXT_FIRST_REG, HANTRO_PP_EXT_REGS * 4);
#endif
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int CheckPPIrq(hantrodec_t *dev)
{
	unsigned long flags;
	int rdy = 0;

	const u32 irq_mask = (1 << dev->core_id);

	spin_lock_irqsave(&owner_lock, flags);

	if (pp_irq & irq_mask) {
		/* reset the wait condition(s) */
		pp_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return rdy;
}

static long WaitPPReadyAndRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	//u32 id = Core->id;

	PDEBUG("wait_event_interruptible PP[%d]\n", dev->core_id);

	if (wait_event_interruptible(pp_wait_queue, CheckPPIrq(dev))) {
		pr_err("PP[%d]  failed to wait_event_interruptible interrupted\n", dev->core_id);
		return -ERESTARTSYS;
	}

	atomic_inc(&dev->irq_tx);

	/* refresh registers */
	return PPRefreshRegs(dev, Core);
}

static int CheckCoreIrq(const struct file *filp, int *id)
{
	unsigned long flags;
	int rdy = 0, n = 0;
	hantrodec_t *dev;

	do {
		u32 irq_mask;

		dev = &hantrodec_data[n];
		irq_mask = (1 << dev->core_id);

		spin_lock_irqsave(&owner_lock, flags);

		if (dec_irq & irq_mask) {
			if (dev->dec_owner == filp) {
				/* we have an IRQ for our client */

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;

				/* signal ready Core no. for our client */
				*id = dev->core_id;

				rdy = 1;

				spin_unlock_irqrestore(&owner_lock, flags);
				break;
			} else if (dev->dec_owner == NULL) {
				/* zombie IRQ */
				pr_info("IRQ on Core[%d], but no owner!!!\n", n);

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;
			}
		}

		spin_unlock_irqrestore(&owner_lock, flags);

		n++; /* next Core */
	} while (n < cores);

	return rdy;
}

static long WaitCoreReady(const struct file *filp, int *id)
{
	PDEBUG("wait_event_interruptible CORE\n");

	if (wait_event_interruptible(dec_wait_queue, CheckCoreIrq(filp, id))) {
		pr_err("CORE  failed to wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	atomic_inc(&hantrodec_data[*id].irq_tx);

	return 0;
}

/*-------------------------------------------------------------------------
 *Function name   : hantrodec_ioctl
 *Description     : communication method to/from the user space
 *
 *Return type     : long
 *-------------------------------------------------------------------------
 */

static long hantrodec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	long tmp;

	PDEBUG("ioctl cmd 0x%08x\n", cmd);
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HANTRODEC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HANTRODEC_IOC_MAXNR)
		return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_CLI): {
		__u32 id;

		__get_user(id, (__u32 *)arg);
		if (id >= cores)
			return -EFAULT;
		disable_irq(hantrodec_data[id].irq);
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_STI): {
		__u32 id;

		__get_user(id, (__u32 *)arg);
		if (id >= cores)
			return -EFAULT;
		enable_irq(hantrodec_data[id].irq);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET): {
		__u32 id;

		__get_user(id, (__u32 *)arg);
		if (id >= cores)
			return -EFAULT;

		__put_user(multicorebase[id], (unsigned long *) arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE): {
		__u32 id;
		__u32 io_size;

		__get_user(id, (__u32 *)arg);
		if (id >= cores)
			return -EFAULT;
		io_size = hantrodec_data[id].iosize;
		__put_user(io_size, (u32 *) arg);

		return 0;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS): {
		tmp = copy_to_user((u64 *) arg, multicorebase, sizeof(multicorebase));
		if (err) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_CORES):
		__put_user(cores, (unsigned int *) arg);
		PDEBUG("hantrodec_data.cores=%d\n", cores);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		DecFlushRegs(&hantrodec_data[Core.id], &Core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		PPFlushRegs(&hantrodec_data[Core.id], &Core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecRefreshRegs(&hantrodec_data[Core.id], &Core);
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return PPRefreshRegs(&hantrodec_data[Core.id], &Core);
	}
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
		int id;

		PDEBUG("Reserve DEC Core, format = %li\n", arg);
		id = GetDecCoreID(arg);
		if (id < 0) {
			pr_err("invalid format: %ld\n", arg);
			return -EFAULT;
		}
		return ReserveDecoder(&hantrodec_data[id], filp, arg);
	}
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
		if (arg >= cores || hantrodec_data[arg].dec_owner != filp) {
			pr_err("bogus DEC release, Core = %li\n", arg);
			return -EFAULT;
		}

		PDEBUG("Release DEC, Core = %li\n", arg);

		ReleaseDecoder(&hantrodec_data[arg]);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
		return ReservePostProcessor(&hantrodec_data[0], filp);
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE): {
		if (arg != 0 || hantrodec_data[arg].pp_owner != filp) {
			pr_err("bogus PP release %li\n", arg);
			return -EFAULT;
		}

		ReleasePostProcessor(&hantrodec_data[arg]);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT): {
		struct core_desc Core;

		/* get registers from user space */
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return WaitDecReadyAndRefreshRegs(&hantrodec_data[Core.id], &Core);
	}
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT): {
		struct core_desc Core;

		/* get registers from user space */
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return WaitPPReadyAndRefreshRegs(&hantrodec_data[Core.id], &Core);
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT): {
		int id;

		tmp = WaitCoreReady(filp, &id);
		__put_user(id, (int *) arg);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID): {
		u32 id;

		__get_user(id, (u32 *)arg);
		if (id >= cores)
			return -EFAULT;
		id = ioread32(hantrodec_data[id].hwregs);
		__put_user(id, (u32 *) arg);
		return 0;
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
		int id;
		hantrodec_instance *ctx = (hantrodec_instance *)filp->private_data;

		PDEBUG("Get DEC Core_id, format = %li\n", arg);
		id = GetDecCoreID(arg);
		if ((ctx->core_id == 3) && (id >= 0)) {
			if (id == 0) {
				ctx->core_id = 1; //g1
				/*power off g2*/
				pm_runtime_put_sync(hantrodec_data[1].dev);
				hantro_clk_disable(&hantrodec_data[1].clk);
			} else if (id == 1) {
				ctx->core_id = 2; //g2
				/*power off g1*/
				pm_runtime_put_sync(hantrodec_data[0].dev);
				hantro_clk_disable(&hantrodec_data[0].clk);
			}
		}
		return id;
	}
	case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
		PDEBUG("hantrodec: dec_irq     = 0x%08x\n", dec_irq);
		PDEBUG("hantrodec: pp_irq      = 0x%08x\n", pp_irq);

		//PDEBUG("hantrodec: IRQs received/sent2user = %d / %d\n",
		//atomic_read(&irq_rx), atomic_read(&irq_tx));

		for (tmp = 0; tmp < cores; tmp++) {
			PDEBUG("hantrodec: Core %ld IRQs received/sent2user = %d / %d\n", tmp,
				atomic_read(&hantrodec_data[tmp].irq_rx), atomic_read(&hantrodec_data[tmp].irq_tx));

			PDEBUG("hantrodec: dec_core[%li] %s\n",
					tmp, hantrodec_data[tmp].dec_owner == NULL ? "FREE" : "RESERVED");
			PDEBUG("hantrodec: pp_core[%li]  %s\n",
					tmp, hantrodec_data[tmp].pp_owner == NULL ? "FREE" : "RESERVED");
		}
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
struct core_desc_32 {
	__u32 id; /* id of the Core */
	compat_caddr_t regs; /* pointer to user registers */
	__u32 size; /* size of register space */
};

static int get_hantro_core_desc32(struct core_desc *kp, struct core_desc_32 __user *up)
{
	u32 tmp;

	if (!access_ok(VERIFY_READ, up, sizeof(struct core_desc_32)) ||
				get_user(kp->id, &up->id) ||
				get_user(kp->size, &up->size) ||
				get_user(tmp, &up->regs)) {
		return -EFAULT;
	}
	kp->regs = (__force u32 *)compat_ptr(tmp);
	return 0;
}

static int put_hantro_core_desc32(struct core_desc *kp, struct core_desc_32 __user *up)
{
	u32 tmp = (u32)((unsigned long)kp->regs);

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct core_desc_32)) ||
				put_user(kp->id, &up->id) ||
				put_user(kp->size, &up->size) ||
				put_user(tmp, &up->regs)) {
		return -EFAULT;
	}
	return 0;
}
static long hantrodec_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
#define HANTRO_IOCTL32(err, filp, cmd, arg) { \
		mm_segment_t old_fs = get_fs(); \
		set_fs(KERNEL_DS); \
		err = hantrodec_ioctl(filp, cmd, arg); \
		if (err) \
			return err; \
		set_fs(old_fs); \
	}

	union {
		struct core_desc kcore;
		unsigned long kux;
		unsigned int kui;
	} karg;
	void __user *up = compat_ptr(arg);
	long err = 0;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET):
	case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS):
		err = get_user(karg.kux, (s32 __user *)up);
		if (err)
			return err;
		HANTRO_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kux), (s32 __user *)up);
		break;
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE):
	case _IOC_NR(HANTRODEC_IOC_MC_CORES):
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT):
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID):
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		HANTRO_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kui), (s32 __user *)up);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG):
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG):
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT):
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT):
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG):
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG):
		err = get_hantro_core_desc32(&karg.kcore, up);
		if (err)
			return err;
		HANTRO_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_hantro_core_desc32(&karg.kcore, up);
		break;
	default:
		err = hantrodec_ioctl(filp, cmd, (unsigned long)up);
		break;
	}

	return err;
}

#endif //ifdef CONFIG_COMPAT

/*--------------------------------------------------------------------------
 *Function name   : hantrodec_open
 *Description     : open method
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int hantrodec_open(struct inode *inode, struct file *filp)
{
	int i;
	int idx;

	idx = hantro_new_instance();
	if (idx < 0)
		return -ENOMEM;

	PDEBUG("dev opened: id: %d\n", idx);
	hantrodec_ctx[idx].core_id = 3;  //unknow
	hantrodec_ctx[idx].inst_id = idx;
	filp->private_data = (void *)(&hantrodec_ctx[idx]);

	/*not yet know which core id, so power on both g1 and g2 firstly*/
	for (i = 0; i < 2; i++) {
		hantro_clk_enable(&hantrodec_data[i].clk);
		hantro_power_on_disirq(&hantrodec_data[i]);
	}
	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_release
 *Description     : Release driver
 *
 *Return type     : int
 *----------------------------------------------------------------------------
 */
static int hantrodec_release(struct inode *inode, struct file *filp)
{
	int n;
	//hantrodec_t *dev = &hantrodec_data;
	hantrodec_instance *ctx = (hantrodec_instance *)filp->private_data;

	PDEBUG("closing ...\n");
	for (n = 0; n < cores; n++) {
		if (hantrodec_data[n].dec_owner == filp) {
			PDEBUG("releasing dec Core %i lock\n", n);
			ReleaseDecoder(&hantrodec_data[n]);
		}
	}

	for (n = 0; n < 1; n++) {
		if (hantrodec_data[n].pp_owner == filp) {
			PDEBUG("releasing pp Core %i lock\n", n);
			ReleasePostProcessor(&hantrodec_data[n]);
		}
	}

	if (ctx->core_id & 0x1) {
		pm_runtime_put_sync(hantrodec_data[0].dev);
		hantro_clk_disable(&hantrodec_data[0].clk);
	}
	if (ctx->core_id & 0x2) {
		pm_runtime_put_sync(hantrodec_data[1].dev);
		hantro_clk_disable(&hantrodec_data[1].clk);
	}
	hantro_free_instance(ctx->inst_id);

	PDEBUG("closed: id: %d\n", n);
	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantro_mmap
 *Description     : memory map interface for hantro file operation
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int hantro_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff == (multicorebase[0] >> PAGE_SHIFT) || vm->vm_pgoff == (multicorebase[1] >> PAGE_SHIFT)) {
		vm->vm_flags |= VM_IO;
		vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
		PDEBUG("hantro mmap: size=0x%lX, page off=0x%lX\n", (vm->vm_end - vm->vm_start), vm->vm_pgoff);
		return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end - vm->vm_start,
						vm->vm_page_prot) ? -EAGAIN : 0;
	}	else {
		pr_err("invalid map offset :0x%lX\n", vm->vm_pgoff);
		return -EINVAL;
	}
}

/* VFS methods */
static const struct file_operations hantrodec_fops = {
	.owner = THIS_MODULE,
	.open = hantrodec_open,
	.release = hantrodec_release,
	.unlocked_ioctl = hantrodec_ioctl,
	.fasync = NULL,
	.mmap = hantro_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hantrodec_ioctl32,
#endif
};

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_init
 *Description     : Initialize the driver
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int hantrodec_init(struct platform_device *pdev, int id)
{
	int result;
	int irq;
	struct device *temp_class;

	//dec_irq = 0;
	//pp_irq = 0;
	pr_debug("hantrodec: Init multi Core[0] at 0x%16lx\n"
			"                     Core[1] at 0x%16lx\n", multicorebase[0], multicorebase[1]);

	//hantrodec_data.cores = 0;
	//hantrodec_data.iosize[0] = DEC_IO_SIZE_0;
	//hantrodec_data.iosize[1] = DEC_IO_SIZE_1;

	//hantrodec_data.async_queue_dec = NULL;
	//hantrodec_data.async_queue_pp = NULL;

	hantrodec_data[id].iosize = (id == 0) ? DEC_IO_SIZE_0 : DEC_IO_SIZE_1;

	if (!hantrodec_major) {
		dec_irq = 0;
		pp_irq = 0;

		result = register_chrdev(hantrodec_major, "hantrodec", &hantrodec_fops);
		if (result < 0) {
			pr_err("hantrodec: unable to get major %d\n", hantrodec_major);
			goto err;
		} else if (result != 0) { /* this is for dynamic major */
			hantrodec_major = result;
		}

		hantro_class = class_create(THIS_MODULE, "mxc_hantro_845");
		if (IS_ERR(hantro_class)) {
			result = -1;
			goto err;
		}
		temp_class = device_create(hantro_class, NULL, MKDEV(hantrodec_major, 0), NULL, DEVICE_NAME);
		if (IS_ERR(temp_class)) {
			result = -1;
			goto err_out_class;
		}
	}

	result = ReserveIO(id);
	if (result < 0)
		goto err;

	hantrodec_data[id].dec_owner = 0;
	hantrodec_data[id].pp_owner = 0;

	sema_init(&hantrodec_data[id].dec_core_sem, 1);
	sema_init(&hantrodec_data[id].pp_core_sem, 1);

	/* read configuration fo all cores */
	ReadCoreConfig(&hantrodec_data[id]);

	/* reset hardware */
	ResetAsic(&hantrodec_data[id]);

	/* register irq for each core*/
	irq = platform_get_irq_byname(pdev, "irq_hantro");
	if (irq > 0) {
		hantrodec_data[id].irq = irq;
		result = request_irq(irq, hantrodec_isr, IRQF_SHARED,
				"hantrodec", (void *) &hantrodec_data[id]);

		if (result != 0) {
			if (result == -EINVAL)
				pr_err("hantrodec: Bad irq number or handler\n");
			else if (result == -EBUSY) {
				pr_err("hantrodec: IRQ <%d> busy, change your config\n",
				hantrodec_data[id].irq);
			}
			ReleaseIO(id);
			goto err;
		}
	}	else {
		pr_err("hantrodec: IRQ0 not in use!\n");
		goto err;
	}

	hantrodec_data[id].irq_rx.counter = 0;
	hantrodec_data[id].irq_tx.counter = 0;
	irq_set_status_flags(irq, IRQ_DISABLE_UNLAZY);
	pr_info("hantrodec %d : module inserted. Major = %d\n", id, hantrodec_major);

	return 0;

err_out_class:
	device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
	class_destroy(hantro_class);
err:
	pr_err("hantrodec: module not inserted\n");
	unregister_chrdev(hantrodec_major, "hantrodec");
	return result;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_cleanup
 *Description     : clean up
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static void hantrodec_cleanup(int id)
{
	hantrodec_t *dev = &hantrodec_data[id];
	//int n = 0;
	/* reset hardware */
	ResetAsic(dev);

	/* free the IRQ */
	//for (n = 0; n < dev->cores; n++) {
		if (dev->irq != -1)
			free_irq(dev->irq, (void *) dev);
	//}

	ReleaseIO(id);

	//unregister_chrdev(hantrodec_major, "hantrodec");

	PDEBUG("hantrodec: module removed\n");

}

/*---------------------------------------------------------------------------
 *Function name   : CheckHwId
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int CheckHwId(hantrodec_t *dev)
{
	long int hwid;
	//int i;
	size_t num_hw = sizeof(DecHwId) / sizeof(*DecHwId);

	int found = 0;

	//for (i = 0; i < cores; i++) {
		if (dev->hwregs != NULL) {
			hwid = readl(dev->hwregs);
			pr_debug("hantrodec: Core %d HW ID=0x%16lx\n", dev->core_id, hwid);
			hwid = (hwid >> 16) & 0xFFFF; /* product version only */

			while (num_hw--) {
				if (hwid == DecHwId[num_hw]) {
					pr_debug("hantrodec: Supported HW found at 0x%16lx\n",
							multicorebase[dev->core_id]);
					found++;
					dev->hw_id = hwid;
					break;
				}
			}
			if (!found) {
				pr_err("hantrodec: Unknown HW found at 0x%16lx\n",	multicorebase[dev->core_id]);
				return 0;
			}
			found = 0;
			num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
		}
	//}

	return 1;
}

/*---------------------------------------------------------------------------
 *Function name   : ReserveIO
 *Description     : IO reserve
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int ReserveIO(int i)
{
	//int i;

	//for (i = 0; i < HXDEC_MAX_CORES; i++) {
		if (multicorebase[i] != -1) {
			if (!request_mem_region(multicorebase[i], hantrodec_data[i].iosize, "hantrodec0")) {
				pr_err("hantrodec: failed to reserve HW regs, %d, base: 0x%lX, %d\n", i, multicorebase[i], hantrodec_data[i].iosize);
				return -EBUSY;
			}

			hantrodec_data[i].hwregs = (volatile u8 *) ioremap_nocache(multicorebase[i],
			hantrodec_data[i].iosize);

			if (hantrodec_data[i].hwregs == NULL) {
				pr_err("hantrodec: failed to ioremap HW regs\n");
				ReleaseIO(i);
				return -EBUSY;
			}
			//hantrodec_data.cores++;
		}
	//}

	/* check for correct HW */
	if (!CheckHwId(&hantrodec_data[i])) {
		ReleaseIO(i);
		return -EBUSY;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : releaseIO
 *Description     : release
 *
 *Return type     : void
 *---------------------------------------------------------------------------
 */
static void ReleaseIO(int i)
{
	//int i;

	//for (i = 0; i < hantrodec_data.cores; i++) {
		if (hantrodec_data[i].hwregs)
			iounmap((void *) hantrodec_data[i].hwregs);
		release_mem_region(multicorebase[i], hantrodec_data[i].iosize);
	//}
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_isr
 *Description     : interrupt handler
 *
 *Return type     : irqreturn_t
 *---------------------------------------------------------------------------
 */
static irqreturn_t hantrodec_isr(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int handled = 0;
	//int i;
	volatile u8 *hwregs;

	hantrodec_t *dev = (hantrodec_t *) dev_id;
	u32 irq_status_dec;

	spin_lock_irqsave(&owner_lock, flags);

	//for (i = 0; i < cores; i++) {
		hwregs = dev->hwregs;

		/* interrupt status register read */
		irq_status_dec = ioread32(hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

		if (irq_status_dec & HANTRODEC_DEC_IRQ) {
			/* clear dec IRQ */
			irq_status_dec &= (~HANTRODEC_DEC_IRQ);
			iowrite32(irq_status_dec, hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

			PDEBUG("decoder IRQ received! Core %d\n", dev->core_id);

			atomic_inc(&hantrodec_data[dev->core_id].irq_rx);

			dec_irq |= (1 << dev->core_id);

			wake_up_interruptible_all(&dec_wait_queue);
			handled++;
		}
	//}

	spin_unlock_irqrestore(&owner_lock, flags);

	if (!handled)
		pr_info("IRQ received, but not hantrodec's!\n");

	(void)hwregs;
	return IRQ_RETVAL(handled);
}

/*---------------------------------------------------------------------------
 *Function name   : ResetAsic
 *Description     : reset asic
 *
 *Return type     :
 *---------------------------------------------------------------------------
 */
static void ResetAsic(hantrodec_t *dev)
{
	int i;
	u32 status;

	//for (j = 0; j < dev->cores; j++) {
		status = ioread32(dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

		if (status & HANTRODEC_DEC_E) {
			/* abort with IRQ disabled */
			status = HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
			iowrite32(status, dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);
		}

		if (IS_G1(dev->hw_id))
			/* reset PP */
			iowrite32(0, dev->hwregs + HANTRO_IRQ_STAT_PP_OFF);

		for (i = 4; i < dev->iosize; i += 4)
			iowrite32(0, dev->hwregs + i);
	//}
}

/*---------------------------------------------------------------------------
 *Function name   : dump_regs
 *Description     : Dump registers
 *
 *Return type     :
 *---------------------------------------------------------------------------
 */
#ifdef HANTRODEC_DEBUG
static void dump_regs(hantrodec_t *dev)
{
	int i, c;

	PDEBUG("Reg Dump Start\n");
	for (c = 0; c < dev->cores; c++) {
		for (i = 0; i < dev->iosize[c]; i += 4*4) {
			PDEBUG("\toffset %04X: %08X  %08X  %08X  %08X\n", i,
			ioread32(dev->hwregs[c] + i),
			ioread32(dev->hwregs[c] + i + 4),
			ioread32(dev->hwregs[c] + i + 8),
			ioread32(dev->hwregs[c] + i + 12));
		}
	}
	PDEBUG("Reg Dump End\n");
}
#endif

static int hantro_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	unsigned long reg_base;
	int id;

	id = hantro_device_id(&pdev->dev);
	if (id < 0)
		return -ENODEV;

	hantrodec_data[id].dev = &pdev->dev;
	hantrodec_data[id].core_id = id;
	platform_set_drvdata(pdev, &hantrodec_data[id]);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs_hantro");
	if (!res) {
		pr_err("hantro: unable to get vpu base addr\n");
		return -ENODEV;
	}
	reg_base = res->start;
	if ((ulong)reg_base != multicorebase[id]) {
		pr_err("hantrodec %d: regbase(0x%lX) not equal to expected value(0x%lX)\n", id, reg_base, multicorebase[id]);
		return -ENODEV;
	}

	hantrodec_data[id].clk.dec = clk_get(&pdev->dev, "clk_hantro");
	hantrodec_data[id].clk.bus = clk_get(&pdev->dev, "clk_hantro_bus");
	if (IS_ERR(hantrodec_data[id].clk.dec) || IS_ERR(hantrodec_data[id].clk.bus)) {
		pr_err("hantro: get clock failed\n");
		return -ENODEV;
	}
	pr_debug("hantro: dec, bus clock: 0x%lX, 0x%lX\n", clk_get_rate(hantrodec_data[id].clk.dec),
				clk_get_rate(hantrodec_data[id].clk.bus));

#if 0 //eagle for temporary debug on zebu
{
volatile u8 *pd_regs;
int val1, val2;

//set 0x303a00f8 with 0x3fff to power up all the domain
//request_mem_region(0x303a00f8, 0x100,"hx280enc");
pd_regs = (volatile u8 *) ioremap_nocache(0x303a00f8, 0x100);
printk("power up all domain: set pd_regs(%p) with 0x3fff\n", pd_regs);
val1 = readl(pd_regs);
writel(0x3fff, pd_regs);
val2 = readl(pd_regs);
iounmap((void *) pd_regs);
printk("%p : old: 0x%X, new: 0x%X\n", pd_regs, val1, val2);
//release_mem_region(0x303a00f8, 0x100);
printk("power enable done\n");
}
#endif

	hantro_clk_enable(&hantrodec_data[id].clk);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	hantro_ctrlblk_reset(&hantrodec_data[id]);

	err = hantrodec_init(pdev, id);
	if (err != 0) {
		pr_err("hantro: hantrodec_init failed\n");
		goto error;
	}

#ifdef CONFIG_DEVICE_THERMAL_XXX
	HANTRO_REG_THERMAL_NOTIFIER(&hantro_thermal_hot_notifier);
	thermal_event = 0;
	thermal_cur = 0;
	hantro_dynamic_clock = 0;
#endif
	hantrodec_data[id].timeout = 0;
	mutex_init(&hantrodec_data[id].dev_mutex);
	instance_mask = 0;

	goto out;

error:
	pr_err("hantro probe failed\n");
out:
	pm_runtime_put_sync(&pdev->dev);
	hantro_clk_disable(&hantrodec_data[id].clk);
	return err;
}

static int hantro_dev_remove(struct platform_device *pdev)
{
	hantrodec_t *dev = platform_get_drvdata(pdev);

	hantro_clk_enable(&dev->clk);
	pm_runtime_get_sync(&pdev->dev);

	hantrodec_cleanup(dev->core_id);
#if 1 // FIXME: need to identify core id
	if (hantrodec_major > 0) {
		device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
		class_destroy(hantro_class);
		unregister_chrdev(hantrodec_major, "hantrodec");
		hantrodec_major = 0;
	}
#endif
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	hantro_clk_disable(&dev->clk);

#ifdef CONFIG_DEVICE_THERMAL_XXX
	HANTRO_UNREG_THERMAL_NOTIFIER(&hantro_thermal_hot_notifier);
#endif

	return 0;
}

#ifdef CONFIG_PM
static int hantro_suspend(struct device *dev)
{
	pm_runtime_put_sync_suspend(dev);   //power off
	return 0;
}
static int hantro_resume(struct device *dev)
{
	hantrodec_t *hantrodev = dev_get_drvdata(dev);

	hantro_power_on_disirq(hantrodev);
	hantro_ctrlblk_reset(hantrodev);
	return 0;
}
static int hantro_runtime_suspend(struct device *dev)
{
	release_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static int hantro_runtime_resume(struct device *dev)
{
	hantrodec_t *hantrodev = dev_get_drvdata(dev);

	request_bus_freq(BUS_FREQ_HIGH);
	hantro_ctrlblk_reset(hantrodev);
	return 0;
}

static const struct dev_pm_ops hantro_pm_ops = {
	SET_RUNTIME_PM_OPS(hantro_runtime_suspend, hantro_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantro_suspend, hantro_resume)
};
#endif //CONFIG_PM

static const struct of_device_id hantro_of_match[] = {
	{ .compatible = "nxp,imx8mm-hantro", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, vpu_of_match);


static struct platform_driver mxchantro_driver = {
	.driver = {
	.name = "mxc_hantro_845",
	.of_match_table = hantro_of_match,
#ifdef CONFIG_PM
	.pm = &hantro_pm_ops,
#endif
	},
	.probe = hantro_dev_probe,
	.remove = hantro_dev_remove,
};

static int __init hantro_init(void)
{
	int ret = platform_driver_register(&mxchantro_driver);

	return ret;
}

static void __exit hantro_exit(void)
{
	//clk_put(hantro_clk);
	platform_driver_unregister(&mxchantro_driver);
}

module_init(hantro_init);
module_exit(hantro_exit);

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Driver module for Hantro Decoder/Post-Processor");

