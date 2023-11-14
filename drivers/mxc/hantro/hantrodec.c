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
#include <linux/compat.h>
#include <linux/busfreq-imx.h>

#ifdef CONFIG_DEVFREQ_THERMAL
#include <linux/thermal.h>
DEFINE_SPINLOCK(thermal_lock);
/*1:hot, 0: not hot*/
#define HANTRO_COOLING_MAX_STATE 1
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
#define BLK_CTL_BASE                          0x38320000

#define VEXPRESS_LOGIC_0_BASE           0xFC010000
#define VEXPRESS_LOGIC_1_BASE           0xFC020000

#define DEC_IO_SIZE_0             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */
#define DEC_IO_SIZE_1             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */

#define HANTRO_G1_DEF_CLK		(600000000)
#define HANTRO_G2_DEF_CLK		(600000000)
#define HANTRO_BUS_DEF_CLK	(800000000)
#define HANTRO_CLK_VOL_THR	(600000000)
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

ulong multicorebase[HXDEC_MAX_CORES] = {
	SOCLE_LOGIC_0_BASE,
	SOCLE_LOGIC_1_BASE
};


static struct class *hantro_class;
#define DEVICE_NAME		"mxc_hantro"

static struct device *hantro_dev;
static struct clk *hantro_clk_g1;
static struct clk *hantro_clk_g2;
static struct clk *hantro_clk_bus;
static struct regulator *hantro_regulator;

static int hantro_dbg = -1;
module_param(hantro_dbg, int, 0644);
MODULE_PARM_DESC(hantro_dbg, "Debug level (0-1)");
#undef PDEBUG
#define PDEBUG(fmt, arg...)     \
	do {                                      \
		if (hantro_dbg > 0) { \
			dev_info(hantro_dev, fmt, ## arg); \
		} \
	} while (0)


static int hantrodec_major; /* dynamic allocation */

/* here's all the must remember stuff */
typedef struct {
	char *buffer;
	unsigned int iosize[HXDEC_MAX_CORES];
	volatile u8 *hwregs[HXDEC_MAX_CORES];
	int irq[HXDEC_MAX_CORES];
	int hw_id[HXDEC_MAX_CORES];
	int cores;
	struct fasync_struct *async_queue_dec;
	struct fasync_struct *async_queue_pp;
	struct thermal_cooling_device *cooling;
	bool skip_blkctrl;
} hantrodec_t;

static hantrodec_t hantrodec_data; /* dynamic allocation? */

static int ReserveIO(void);
static void ReleaseIO(void);

static void ResetAsic(hantrodec_t *dev);

#ifdef HANTRODEC_DEBUG
static void dump_regs(hantrodec_t *dev);
#endif

/* IRQ handler */
static irqreturn_t hantrodec_isr(int irq, void *dev_id);

static u32 dec_regs[HXDEC_MAX_CORES][DEC_IO_SIZE_MAX/4];
struct semaphore dec_core_sem;
struct semaphore pp_core_sem;
struct semaphore core_suspend_sem[HXDEC_MAX_CORES];

static int dec_irq;
static int pp_irq;

atomic_t irq_rx = ATOMIC_INIT(0);
atomic_t irq_tx = ATOMIC_INIT(0);

static struct file *dec_owner[HXDEC_MAX_CORES];
static struct file *pp_owner[HXDEC_MAX_CORES];

/* spinlock_t owner_lock = SPIN_LOCK_UNLOCKED; */
DEFINE_SPINLOCK(owner_lock);

DECLARE_WAIT_QUEUE_HEAD(dec_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(pp_wait_queue);

DECLARE_WAIT_QUEUE_HEAD(hw_queue);

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

static u32 cfg[HXDEC_MAX_CORES];
static u32 timeout;

static int hantro_update_voltage(struct device *dev)
{
	unsigned long new_vol, old_vol;
	int ret;
	unsigned long clk1, clk2;

	clk1 = clk_get_rate(hantro_clk_g1);
	clk2 = clk_get_rate(hantro_clk_g2);

	if (!clk1 || !clk2)
		return -1;

	old_vol = regulator_get_voltage(hantro_regulator);
	if ((clk1 >= HANTRO_CLK_VOL_THR) || (clk2 >= HANTRO_CLK_VOL_THR))
		new_vol = 1000000; // 1.0v
	else
		new_vol = 900000; // 0.9v

	if (old_vol != new_vol)	{
		ret = regulator_set_voltage_tol(hantro_regulator, new_vol, 0);
		if (ret)
			pr_err("failed to set hantro voltage: %ld mV\n", new_vol/1000);
		else
			pr_info("update hantro voltage from %ld mV to %ld mV\n", old_vol/1000, new_vol/1000);
	}
	return 0;
}

static int hantro_clk_enable(struct device *dev)
{
	clk_prepare(hantro_clk_g1);
	clk_enable(hantro_clk_g1);
	clk_prepare(hantro_clk_g2);
	clk_enable(hantro_clk_g2);
	clk_prepare(hantro_clk_bus);
	clk_enable(hantro_clk_bus);
	return 0;
}

static int hantro_clk_disable(struct device *dev)
{
	if (hantro_clk_g1) {
		clk_disable(hantro_clk_g1);
		clk_unprepare(hantro_clk_g1);
	}
	if (hantro_clk_g2) {
		clk_disable(hantro_clk_g2);
		clk_unprepare(hantro_clk_g2);
	}
	if (hantro_clk_bus) {
		clk_disable(hantro_clk_bus);
		clk_unprepare(hantro_clk_bus);
	}
	return 0;
}

static int hantro_ctrlblk_reset(struct device *dev)
{
	volatile u8 *iobase;

	if (hantrodec_data.skip_blkctrl)
		return 0;

	//config G1/G2
	hantro_clk_enable(dev);
	iobase = (volatile u8 *)ioremap(BLK_CTL_BASE, 0x10000);
	iowrite32(0x3, iobase);  //VPUMIX G1/G2 block soft reset  control
	iowrite32(0x3, iobase+4); //VPUMIX G1/G2 block clock enable control
	iowrite32(0xFFFFFFFF, iobase + 0x8); // all G1 fuse dec enable
	iowrite32(0xFFFFFFFF, iobase + 0xC); // all G1 fuse pp enable
	iowrite32(0xFFFFFFFF, iobase + 0x10); // all G2 fuse dec enable
	iounmap(iobase);
	hantro_clk_disable(dev);
	return 0;
}

#ifdef CONFIG_DEVFREQ_THERMAL
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
	hantro_update_voltage(dev);
	return 0;
}

static int hantro_cooling_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = HANTRO_COOLING_MAX_STATE;
	return 0;
}

static int hantro_cooling_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = thermal_event;
	return 0;
}

static int hantro_cooling_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	unsigned long flags;

	spin_lock_irqsave(&thermal_lock, flags);
	thermal_event = state;		/*event: 1: hot, 0: cool*/
	spin_unlock_irqrestore(&thermal_lock, flags);
	pr_info("hantro receive cooling set state: %ld\n", state);
	return 0;
}

static struct thermal_cooling_device_ops hantro_cooling_ops = {
	.get_max_state = hantro_cooling_get_max_state,
	.get_cur_state = hantro_cooling_get_cur_state,
	.set_cur_state = hantro_cooling_set_cur_state,
};
#endif  //CONFIG_DEVFREQ_THERMAL

static void ReadCoreConfig(hantrodec_t *dev)
{
	int c;
	u32 reg, tmp, mask;

	memset(cfg, 0, sizeof(cfg));

	for (c = 0; c < dev->cores; c++) {
		/* Decoder configuration */
		if (IS_G1(dev->hw_id[c])) {
			reg = ioread32(dev->hwregs[c] + HANTRODEC_SYNTH_CFG * 4);

			tmp = (reg >> DWL_H264_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has H264\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has JPEG\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_MPEG4_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has MPEG4\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

			tmp = (reg >> DWL_VC1_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has VC1\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

			tmp = (reg >> DWL_MPEG2_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has MPEG2\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

			tmp = (reg >> DWL_VP6_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has VP6\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

			reg = ioread32(dev->hwregs[c] + HANTRODEC_SYNTH_CFG_2 * 4);

			/* VP7 and WEBP is part of VP8 */
			mask =  (1 << DWL_VP8_E) | (1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
			tmp = (reg & mask);
			if (tmp & (1 << DWL_VP8_E))
				pr_debug("hantrodec: Core[%d] has VP8\n", c);
			if (tmp & (1 << DWL_VP7_E))
				pr_debug("hantrodec: Core[%d] has VP7\n", c);
			if (tmp & (1 << DWL_WEBP_E))
				pr_debug("hantrodec: Core[%d] has WebP\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

			tmp = (reg >> DWL_AVS_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has AVS\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

			tmp = (reg >> DWL_RV_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has RV\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

			/* Post-processor configuration */
			//reg = ioread32(dev->hwregs[c] + HANTROPP_SYNTH_CFG * 4);
		} else {
			reg = ioread32(dev->hwregs[c] + HANTRODEC_SYNTH_CFG_2 * 4);

			tmp = (reg >> DWL_HEVC_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has HEVC\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

			tmp = (reg >> DWL_VP9_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: Core[%d] has VP9\n", c);
			cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;
		}

		/* Post-processor configuration */
		reg = ioread32(dev->hwregs[c] + HANTRODECPP_SYNTH_CFG * 4);

		tmp = (reg >> DWL_PP_E) & 0x01U;
		if (tmp)
			pr_debug("hantrodec: Core[%d] has PP\n", c);
		cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
	}
}

static int CoreHasFormat(const u32 *cfg, int Core, u32 format)
{
	return (cfg[Core] & (1 << format)) ? 1 : 0;
}

int GetDecCore(long Core, hantrodec_t *dev, struct file *filp)
{
	int success = 0;
	unsigned long flags;

	spin_lock_irqsave(&owner_lock, flags);
	if (dec_owner[Core] == NULL) {
		dec_owner[Core] = filp;
		success = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return success;
}

int GetDecCoreAny(long *Core, hantrodec_t *dev, struct file *filp,
			unsigned long format)
{
	int success = 0;
	long c;

	*Core = -1;

	for (c = 0; c < dev->cores; c++) {
		/* a free Core that has format */
		if (CoreHasFormat(cfg, c, format) && GetDecCore(c, dev, filp)) {
			success = 1;
			*Core = c;
			break;
		}
	}

	return success;
}
int GetDecCoreID(hantrodec_t *dev, struct file *filp,
			unsigned long format)
{
	long c;

	int core_id = -1;

	for (c = 0; c < dev->cores; c++) {
		/* a Core that has format */
		if (CoreHasFormat(cfg, c, format)) {
			core_id = c;
			break;
		}
	}
	PDEBUG("GetDecCoreID=%d\n", core_id);
	return core_id;
}

static int hantrodec_choose_core(int is_g1)
{
	volatile unsigned char *reg = NULL;
	unsigned int blk_base = BLK_CTL_BASE;

	PDEBUG("hantrodec_choose_core\n");
	if (!request_mem_region(blk_base, 0x1000, "blk_ctl"))	{
		pr_err("blk_ctl: failed to reserve HW regs\n");
		return -EBUSY;
	}

	reg = (volatile u8 *) ioremap(blk_base, 0x1000);

	if (reg == NULL) {
		pr_err("blk_ctl: failed to ioremap HW regs\n");
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


long ReserveDecoder(hantrodec_t *dev, struct file *filp, unsigned long format)
{
	long Core = -1;

	/* reserve a Core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* lock a Core that has specific format*/
	if (wait_event_interruptible(hw_queue, GetDecCoreAny(&Core, dev, filp, format) != 0))
		return -ERESTARTSYS;

	if (Core < 0 || Core >= HXDEC_MAX_CORES)
		return -1;

	if (IS_G1(dev->hw_id[Core])) {
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

#ifdef CONFIG_DEVFREQ_THERMAL
	if (hantro_dynamic_clock)
		hantro_thermal_check(hantro_dev);
#endif

	return Core;
}

void ReleaseDecoder(hantrodec_t *dev, long Core)
{
	u32 status;
	unsigned long flags;

	status = ioread32(dev->hwregs[Core] + HANTRODEC_IRQ_STAT_DEC_OFF);

	/* make sure HW is disabled */
	if (status & HANTRODEC_DEC_E) {
		pr_info("hantrodec: DEC[%li] still enabled -> reset\n", Core);

		/* abort decoder */
		status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status, dev->hwregs[Core] + HANTRODEC_IRQ_STAT_DEC_OFF);
	}

	spin_lock_irqsave(&owner_lock, flags);

	dec_owner[Core] = NULL;

	spin_unlock_irqrestore(&owner_lock, flags);

	up(&dec_core_sem);

	wake_up_interruptible_all(&hw_queue);

}

long ReservePostProcessor(hantrodec_t *dev, struct file *filp)
{
	unsigned long flags;

	long Core = 0;

	/* single Core PP only */
	if (down_interruptible(&pp_core_sem))
		return -ERESTARTSYS;

	spin_lock_irqsave(&owner_lock, flags);

	pp_owner[Core] = filp;

	spin_unlock_irqrestore(&owner_lock, flags);

	return Core;
}

void ReleasePostProcessor(hantrodec_t *dev, long Core)
{
	unsigned long flags;

	u32 status = ioread32(dev->hwregs[Core] + HANTRO_IRQ_STAT_PP_OFF);

	/* make sure HW is disabled */
	if (status & HANTRO_PP_E) {
		pr_info("hantrodec: PP[%li] still enabled -> reset\n", Core);

		/* disable IRQ */
		status |= HANTRO_PP_IRQ_DISABLE;

		/* disable postprocessor */
		status &= (~HANTRO_PP_E);
		iowrite32(0x10, dev->hwregs[Core] + HANTRO_IRQ_STAT_PP_OFF);
	}

	spin_lock_irqsave(&owner_lock, flags);

	pp_owner[Core] = NULL;

	spin_unlock_irqrestore(&owner_lock, flags);

	up(&pp_core_sem);
}

long ReserveDecPp(hantrodec_t *dev, struct file *filp, unsigned long format)
{
	/* reserve Core 0, DEC+PP for pipeline */
	unsigned long flags;

	long Core = 0;

	/* check that Core has the requested dec format */
	if (!CoreHasFormat(cfg, Core, format))
		return -EFAULT;

	/* check that Core has PP */
	if (!CoreHasFormat(cfg, Core, DWL_CLIENT_TYPE_PP))
		return -EFAULT;

	/* reserve a Core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* wait until the Core is available */
	if (wait_event_interruptible(hw_queue,	GetDecCore(Core, dev, filp) != 0)) {
		up(&dec_core_sem);
		return -ERESTARTSYS;
	}

	if (down_interruptible(&pp_core_sem)) {
		ReleaseDecoder(dev, Core);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&owner_lock, flags);
	pp_owner[Core] = filp;
	spin_unlock_irqrestore(&owner_lock, flags);

	return Core;
}

long DecFlushRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long ret = 0, i;

	u32 id = Core->id;

	if (IS_G1(dev->hw_id[id])) {
		/* copy original dec regs to kernal space*/
		ret = copy_from_user(dec_regs[id], Core->regs, HANTRO_DEC_ORG_REGS*4);
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}
#ifdef USE_64BIT_ENV
		/* copy extended dec regs to kernal space*/
		ret = copy_from_user(dec_regs[id] + HANTRO_DEC_EXT_FIRST_REG,
				Core->regs + HANTRO_DEC_EXT_FIRST_REG, HANTRO_DEC_EXT_REGS * 4);
#endif
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}

		/* write dec regs but the status reg[1] to hardware */
		/* both original and extended regs need to be written */
		for (i = 2; i <= HANTRO_DEC_ORG_LAST_REG; i++)
			iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
		for (i = HANTRO_DEC_EXT_FIRST_REG; i <= HANTRO_DEC_EXT_LAST_REG; i++)
			iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#endif
	} else {
		ret = copy_from_user(dec_regs[id], Core->regs, HANTRO_G2_DEC_REGS*4);
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}

		/* write all regs but the status reg[1] to hardware */
		for (i = 2; i <= HANTRO_G2_DEC_LAST_REG; i++)
			iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
	}

	if (dec_regs[id][1] & 0x1) {
		if (down_timeout(&core_suspend_sem[id], msecs_to_jiffies(10000)))
			pr_err("core suspend sem down error id %d\n", id);
	}

	/* write the status register, which may start the decoder */
	iowrite32(dec_regs[id][1], dev->hwregs[id] + 4);

	PDEBUG("flushed registers on Core %d\n", id);

	return 0;
}

long DecRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long ret, i;
	u32 id = Core->id;

	if (IS_G1(dev->hw_id[id])) {
		/* user has to know exactly what they are asking for */
		//if(Core->size != (HANTRO_DEC_ORG_REGS * 4))
		//  return -EFAULT;

		/* read all registers from hardware */
		/* both original and extended regs need to be read */
		for (i = 0; i <= HANTRO_DEC_ORG_LAST_REG; i++)
			dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
		for (i = HANTRO_DEC_EXT_FIRST_REG; i <= HANTRO_DEC_EXT_LAST_REG; i++)
			dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#endif

		if (timeout) {
			/* Enable TIMEOUT bits in Reg[1] */
			dec_regs[id][1] = 0x40100;
			/* Reset HW */
			ResetAsic(dev);
			timeout = 0;
		}

		/* put registers to user space*/
		/* put original registers to user space*/
		ret = copy_to_user(Core->regs, dec_regs[id], HANTRO_DEC_ORG_REGS*4);
#ifdef USE_64BIT_ENV
		/*put extended registers to user space*/
		ret = copy_to_user(Core->regs + HANTRO_DEC_EXT_FIRST_REG,
				dec_regs[id] + HANTRO_DEC_EXT_FIRST_REG, HANTRO_DEC_EXT_REGS * 4);
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
			dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);

		if (timeout) {
			/* Enable TIMEOUT bits in Reg[1] */
			dec_regs[id][1] = 0x40100;
			/* Reset HW */
			ResetAsic(dev);
			timeout = 0;
		}

		/* put registers to user space*/
		ret = copy_to_user(Core->regs, dec_regs[id], HANTRO_G2_DEC_REGS*4);
		if (ret) {
			pr_err("copy_to_user failed, returned %li\n", ret);
			return -EFAULT;
		}
	}
	return 0;
}

static long DecRestoreRegs(hantrodec_t *dev)
{
	long i;

	//G1
	if (dec_owner[0]) {
		for (i = 1; i <= HANTRO_DEC_ORG_LAST_REG; i++)
			iowrite32(dec_regs[0][i], dev->hwregs[0] + i * 4);
	}
	//G2
	if (dec_owner[1]) {
	/* write all regs to hardware */
		for (i = 1; i <= HANTRO_G2_DEC_LAST_REG; i++)
			iowrite32(dec_regs[1][i], dev->hwregs[1] + i * 4);
	}

	return 0;
}

static long DecStoreRegs(hantrodec_t *dev)
{
	long i;
	//G1
	if (dec_owner[0]) {
		if (down_timeout(&core_suspend_sem[0], msecs_to_jiffies(10000))) {
			pr_err("sem down error when store regs, id %d\n", 0);
		} else {
			/* read all registers from hardware */
			/* both original and extended regs need to be read */
			for (i = 0; i <= HANTRO_DEC_ORG_LAST_REG; i++)
				dec_regs[0][i] = ioread32(dev->hwregs[0] + i * 4);

			up(&core_suspend_sem[0]);
		}
	}

	if (dec_owner[1]) {
		//G2
		if (down_timeout(&core_suspend_sem[1], msecs_to_jiffies(10000))) {
			pr_err("sem down error when store regs, id %d\n", 1);
		} else {
			/* read all registers from hardware */
			for (i = 0; i <= HANTRO_G2_DEC_LAST_REG; i++)
				dec_regs[1][i] = ioread32(dev->hwregs[1] + i * 4);

			up(&core_suspend_sem[1]);
		}
	}
	return 0;
}

static int CheckDecIrq(hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;

	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&owner_lock, flags);

	if (dec_irq & irq_mask) {
		/* reset the wait condition(s) */
		dec_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return rdy;
}

long WaitDecReadyAndRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	u32 id = Core->id;
	long ret;

	PDEBUG("wait_event_interruptible DEC[%d]\n", id);

	//ret = wait_event_interruptible_timeout(dec_wait_queue, CheckDecIrq(dev, id), msecs_to_jiffies(200));
	ret = wait_event_timeout(dec_wait_queue, CheckDecIrq(dev, id), msecs_to_jiffies(200));
	if (ret == -ERESTARTSYS) {
		pr_err("DEC[%d]  failed to wait_event interrupted\n", id);
		return -ERESTARTSYS;
	} else if (ret == 0) {
		pr_err("DEC[%d]  wait_event timeout\n", id);
		timeout = 1;
		up(&core_suspend_sem[id]);
	}

	atomic_inc(&irq_tx);

	/* refresh registers */
	return DecRefreshRegs(dev, Core);
}

long PPFlushRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long ret = 0;
	u32 id = Core->id;
	u32 i;

	/* copy original dec regs to kernal space*/
	ret = copy_from_user(dec_regs[id] + HANTRO_PP_ORG_FIRST_REG,
			Core->regs + HANTRO_PP_ORG_FIRST_REG, HANTRO_PP_ORG_REGS*4);
#ifdef USE_64BIT_ENV
	/* copy extended dec regs to kernal space*/
	ret = copy_from_user(dec_regs[id] + HANTRO_PP_EXT_FIRST_REG,
			Core->regs + HANTRO_PP_EXT_FIRST_REG, HANTRO_PP_EXT_REGS*4);
#endif
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	/* both original and extended regs need to be written */
	for (i = HANTRO_PP_ORG_FIRST_REG + 1; i <= HANTRO_PP_ORG_LAST_REG; i++)
		iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
		iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#endif
	/* write the stat reg, which may start the PP */
	iowrite32(dec_regs[id][HANTRO_PP_ORG_FIRST_REG],
	dev->hwregs[id] + HANTRO_PP_ORG_FIRST_REG * 4);

	return 0;
}

long PPRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	long i, ret;
	u32 id = Core->id;
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
		dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
		dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#endif
	/* put registers to user space*/
	/* put original registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_ORG_FIRST_REG,
			dec_regs[id] + HANTRO_PP_ORG_FIRST_REG, HANTRO_PP_ORG_REGS*4);
#ifdef USE_64BIT_ENV
	/* put extended registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_EXT_FIRST_REG,
			dec_regs[id] + HANTRO_PP_EXT_FIRST_REG, HANTRO_PP_EXT_REGS * 4);
#endif
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int CheckPPIrq(hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;

	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&owner_lock, flags);

	if (pp_irq & irq_mask) {
		/* reset the wait condition(s) */
		pp_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return rdy;
}

long WaitPPReadyAndRefreshRegs(hantrodec_t *dev, struct core_desc *Core)
{
	u32 id = Core->id;

	PDEBUG("wait_event_interruptible PP[%d]\n", id);

	if (wait_event_interruptible(pp_wait_queue, CheckPPIrq(dev, id))) {
		pr_err("PP[%d]  failed to wait_event_interruptible interrupted\n", id);
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);

	/* refresh registers */
	return PPRefreshRegs(dev, Core);
}

static int CheckCoreIrq(hantrodec_t *dev, const struct file *filp, int *id)
{
	unsigned long flags;
	int rdy = 0, n = 0;

	do {
		u32 irq_mask = (1 << n);

		spin_lock_irqsave(&owner_lock, flags);

		if (dec_irq & irq_mask) {
			if (dec_owner[n] == filp) {
				/* we have an IRQ for our client */

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;

				/* signal ready Core no. for our client */
				*id = n;

				rdy = 1;

				spin_unlock_irqrestore(&owner_lock, flags);
				break;
			} else if (dec_owner[n] == NULL) {
				/* zombie IRQ */
				pr_info("IRQ on Core[%d], but no owner!!!\n", n);

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;
			}
		}

		spin_unlock_irqrestore(&owner_lock, flags);

		n++; /* next Core */
	} while (n < dev->cores);

	return rdy;
}

long WaitCoreReady(hantrodec_t *dev, const struct file *filp, int *id)
{
	PDEBUG("wait_event_interruptible CORE\n");

	if (wait_event_interruptible(dec_wait_queue, CheckCoreIrq(dev, filp, id))) {
		pr_err("CORE  failed to wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);

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
		err = !access_ok((void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok((void *) arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_CLI): {
		__u32 id;

		__get_user(id, (__u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		disable_irq(hantrodec_data.irq[id]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_STI): {
		__u32 id;

		__get_user(id, (__u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		enable_irq(hantrodec_data.irq[id]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET): {
		__u32 id;

		__get_user(id, (__u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;

		__put_user(multicorebase[id], (unsigned long *) arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE): {
		__u32 id;
		__u32 io_size;

		__get_user(id, (__u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		io_size = hantrodec_data.iosize[id];
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
		__put_user(hantrodec_data.cores, (unsigned int *) arg);
		PDEBUG("hantrodec_data.cores=%d\n", hantrodec_data.cores);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		if (Core.id >= hantrodec_data.cores)
			return -EFAULT;

		DecFlushRegs(&hantrodec_data, &Core);
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

		if (Core.id >= hantrodec_data.cores)
			return -EFAULT;

		PPFlushRegs(&hantrodec_data, &Core);
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

		if (Core.id >= hantrodec_data.cores)
			return -EFAULT;

		return DecRefreshRegs(&hantrodec_data, &Core);
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		if (Core.id >= hantrodec_data.cores)
			return -EFAULT;

		return PPRefreshRegs(&hantrodec_data, &Core);
	}
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
		PDEBUG("Reserve DEC Core, format = %li\n", arg);
		return ReserveDecoder(&hantrodec_data, filp, arg);
	}
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
		if (arg >= hantrodec_data.cores || dec_owner[arg] != filp) {
			pr_err("bogus DEC release, Core = %li\n", arg);
			return -EFAULT;
		}

		PDEBUG("Release DEC, Core = %li\n", arg);

		ReleaseDecoder(&hantrodec_data, arg);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
		return ReservePostProcessor(&hantrodec_data, filp);
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE): {
		if (arg != 0 || pp_owner[arg] != filp) {
			pr_err("bogus PP release %li\n", arg);
			return -EFAULT;
		}

		ReleasePostProcessor(&hantrodec_data, arg);

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

		if (Core.id >= hantrodec_data.cores)
			return -EFAULT;

		return WaitDecReadyAndRefreshRegs(&hantrodec_data, &Core);
	}
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT): {
		struct core_desc Core;

		/* get registers from user space */
		tmp = copy_from_user(&Core, (void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		if (Core.id >= hantrodec_data.cores)
			return -EFAULT;

		return WaitPPReadyAndRefreshRegs(&hantrodec_data, &Core);
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT): {
		int id;

		tmp = WaitCoreReady(&hantrodec_data, filp, &id);
		__put_user(id, (int *) arg);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID): {
		u32 id;

		__get_user(id, (u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		id = ioread32(hantrodec_data.hwregs[id]);
		__put_user(id, (u32 *) arg);
		return 0;
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
		PDEBUG("Get DEC Core_id, format = %li\n", arg);
		return GetDecCoreID(&hantrodec_data, filp, arg);
	}
	case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
		PDEBUG("hantrodec: dec_irq     = 0x%08x\n", dec_irq);
		PDEBUG("hantrodec: pp_irq      = 0x%08x\n", pp_irq);

		PDEBUG("hantrodec: IRQs received/sent2user = %d / %d\n",
		atomic_read(&irq_rx), atomic_read(&irq_tx));

		for (tmp = 0; tmp < hantrodec_data.cores; tmp++) {
			PDEBUG("hantrodec: dec_core[%li] %s\n",
					tmp, dec_owner[tmp] == NULL ? "FREE" : "RESERVED");
			PDEBUG("hantrodec: pp_core[%li]  %s\n",
					tmp, pp_owner[tmp] == NULL ? "FREE" : "RESERVED");
		}
		break;
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

	if (!access_ok(up, sizeof(struct core_desc_32)) ||
				get_user(kp->id, &up->id) ||
				get_user(kp->size, &up->size) ||
				get_user(tmp, &up->regs)) {
		return -EFAULT;
	}
	kp->regs = (__force u32 *)compat_ptr(tmp);
	return 0;
}

static bool hantrodec_is_compat_ptr_ioctl(unsigned int cmd)
{
	bool ret = true;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG):
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG):
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT):
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT):
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG):
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG):
		ret = false;
		break;
	default:
		break;
	}

	return ret;
}

static long hantrodec_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct core_desc Core;
	long err = 0;

	if (hantrodec_is_compat_ptr_ioctl(cmd))
		return compat_ptr_ioctl(filp, cmd, arg);

	err = get_hantro_core_desc32(&Core, up);
	if (err)
		return err;

	if (Core.id >= hantrodec_data.cores)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG):
		err = DecFlushRegs(&hantrodec_data, &Core);
		break;
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG):
		err = PPFlushRegs(&hantrodec_data, &Core);
		break;
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT):
		err = WaitDecReadyAndRefreshRegs(&hantrodec_data, &Core);
		break;
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT):
		err = WaitPPReadyAndRefreshRegs(&hantrodec_data, &Core);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG):
		err = DecRefreshRegs(&hantrodec_data, &Core);
		break;
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG):
		err = PPRefreshRegs(&hantrodec_data, &Core);
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
	PDEBUG("dev opened\n");
	hantro_clk_enable(hantro_dev);
	pm_runtime_get_sync(hantro_dev);
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
	hantrodec_t *dev = &hantrodec_data;

	PDEBUG("closing ...\n");

	for (n = 0; n < dev->cores; n++) {
		if (dec_owner[n] == filp) {
			PDEBUG("releasing dec Core %i lock\n", n);
			ReleaseDecoder(dev, n);
		}
	}

	for (n = 0; n < 1; n++) {
		if (pp_owner[n] == filp) {
			PDEBUG("releasing pp Core %i lock\n", n);
			ReleasePostProcessor(dev, n);
		}
	}

	pm_runtime_put_sync(hantro_dev);
	hantro_clk_disable(hantro_dev);
	PDEBUG("closed\n");
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
		vm_flags_set(vm, VM_IO);
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
int hantrodec_init(struct platform_device *pdev)
{
	int result;
	int irq_0, irq_1;

	dec_irq = 0;
	pp_irq = 0;
	pr_debug("hantrodec: Init multi Core[0] at 0x%16lx\n"
			"                     Core[1] at 0x%16lx\n", multicorebase[0], multicorebase[1]);

	hantrodec_data.cores = 0;
	hantrodec_data.iosize[0] = DEC_IO_SIZE_0;
	hantrodec_data.iosize[1] = DEC_IO_SIZE_1;

	hantrodec_data.async_queue_dec = NULL;
	hantrodec_data.async_queue_pp = NULL;

	result = register_chrdev(hantrodec_major, "hantrodec", &hantrodec_fops);
	if (result < 0) {
		pr_err("hantrodec: unable to get major %d\n", hantrodec_major);
		goto err;
	} else if (result != 0) { /* this is for dynamic major */
		hantrodec_major = result;
	}

	result = ReserveIO();
	if (result < 0)
		goto err;

	memset(dec_owner, 0, sizeof(dec_owner));
	memset(pp_owner, 0, sizeof(pp_owner));

	sema_init(&dec_core_sem, hantrodec_data.cores-1);
	sema_init(&pp_core_sem, 1);
	sema_init(&core_suspend_sem[0], 1);
	sema_init(&core_suspend_sem[1], 1);

	/* read configuration fo all cores */
	ReadCoreConfig(&hantrodec_data);

	/* reset hardware */
	ResetAsic(&hantrodec_data);

	/* register irq for each core*/
	irq_0 = platform_get_irq_byname(pdev, "irq_hantro_g1");
	if (irq_0 > 0) {
		hantrodec_data.irq[0] = irq_0;
		result = request_irq(irq_0, hantrodec_isr, 0,
				"hantrodec", (void *) &hantrodec_data);

		if (result != 0) {
			if (result == -EINVAL)
				pr_err("hantrodec: Bad irq number or handler\n");
			else if (result == -EBUSY) {
				pr_err("hantrodec: IRQ <%d> busy, change your config\n",
				hantrodec_data.irq[0]);
			}
			ReleaseIO();
			goto err;
		}
	}	else {
		pr_err("hantrodec: IRQ0 not in use!\n");
		goto err;
	}

	irq_1 = platform_get_irq_byname(pdev, "irq_hantro_g2");
	if (irq_1 > 0) {
		hantrodec_data.irq[1] = irq_1;
		result = request_irq(irq_1, hantrodec_isr, 0,
				"hantrodec", (void *) &hantrodec_data);

		if (result != 0) {
			if (result == -EINVAL)
				pr_err("hantrodec: Bad irq number or handler\n");
			else if (result == -EBUSY) {
				pr_err("hantrodec: IRQ <%d> busy, change your config\n",
					hantrodec_data.irq[1]);
			}

			ReleaseIO();
			goto err;
		}
	}	else {
		pr_err("hantrodec: IRQ1 not in use!\n");
		goto err;
	}
	pr_info("hantrodec: module inserted. Major = %d\n", hantrodec_major);

	return 0;

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
void hantrodec_cleanup(void)
{
	hantrodec_t *dev = &hantrodec_data;
	int n = 0;
	/* reset hardware */
	ResetAsic(dev);

	/* free the IRQ */
	for (n = 0; n < dev->cores; n++) {
		if (dev->irq[n] != -1)
			free_irq(dev->irq[n], (void *) dev);
	}

	ReleaseIO();

	unregister_chrdev(hantrodec_major, "hantrodec");

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
	int i;
	size_t num_hw = sizeof(DecHwId) / sizeof(*DecHwId);

	int found = 0;

	for (i = 0; i < dev->cores; i++) {
		if (dev->hwregs[i] != NULL) {
			hwid = readl(dev->hwregs[i]);
			pr_debug("hantrodec: Core %d HW ID=0x%16lx\n", i, hwid);
			hwid = (hwid >> 16) & 0xFFFF; /* product version only */

			while (num_hw--) {
				if (hwid == DecHwId[num_hw]) {
					pr_debug("hantrodec: Supported HW found at 0x%16lx\n",
							multicorebase[i]);
					found++;
					dev->hw_id[i] = hwid;
					break;
				}
			}
			if (!found) {
				pr_err("hantrodec: Unknown HW found at 0x%16lx\n",	multicorebase[i]);
				return 0;
			}
			found = 0;
			num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
		}
	}

	return 1;
}

/*---------------------------------------------------------------------------
 *Function name   : ReserveIO
 *Description     : IO reserve
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int ReserveIO(void)
{
	int i;

	for (i = 0; i < HXDEC_MAX_CORES; i++) {
		if (multicorebase[i] != -1) {
			if (!request_mem_region(multicorebase[i], hantrodec_data.iosize[i], "hantrodec0")) {
				pr_err("hantrodec: failed to reserve HW regs\n");
				return -EBUSY;
			}

			hantrodec_data.hwregs[i] = (volatile u8 *) ioremap(multicorebase[i],
			hantrodec_data.iosize[i]);

			if (hantrodec_data.hwregs[i] == NULL) {
				pr_err("hantrodec: failed to ioremap HW regs\n");
				ReleaseIO();
				return -EBUSY;
			}
			hantrodec_data.cores++;
		}
	}

	/* check for correct HW */
	if (!CheckHwId(&hantrodec_data)) {
		ReleaseIO();
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
static void ReleaseIO(void)
{
	int i;

	for (i = 0; i < hantrodec_data.cores; i++) {
		if (hantrodec_data.hwregs[i])
			iounmap((void *) hantrodec_data.hwregs[i]);
		release_mem_region(multicorebase[i], hantrodec_data.iosize[i]);
	}
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_isr
 *Description     : interrupt handler
 *
 *Return type     : irqreturn_t
 *---------------------------------------------------------------------------
 */
irqreturn_t hantrodec_isr(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int handled = 0;
	int i;
	volatile u8 *hwregs;

	hantrodec_t *dev = (hantrodec_t *) dev_id;
	u32 irq_status_dec;

	spin_lock_irqsave(&owner_lock, flags);

	for (i = 0; i < dev->cores; i++) {
		volatile u8 *hwregs = dev->hwregs[i];

		/* interrupt status register read */
		irq_status_dec = ioread32(hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

		if (irq_status_dec & HANTRODEC_DEC_IRQ) {
			/* clear dec IRQ */
			irq_status_dec &= (~HANTRODEC_DEC_IRQ);
			iowrite32(irq_status_dec, hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

			PDEBUG("decoder IRQ received! Core %d\n", i);

			up(&core_suspend_sem[i]);

			atomic_inc(&irq_rx);

			dec_irq |= (1 << i);

			//wake_up_interruptible_all(&dec_wait_queue);
			wake_up_all(&dec_wait_queue);
			handled++;
		}
	}

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
void ResetAsic(hantrodec_t *dev)
{
	int i, j;
	u32 status;

	for (j = 0; j < dev->cores; j++) {
		status = ioread32(dev->hwregs[j] + HANTRODEC_IRQ_STAT_DEC_OFF);

		if (status & HANTRODEC_DEC_E) {
			/* abort with IRQ disabled */
			status = HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
			iowrite32(status, dev->hwregs[j] + HANTRODEC_IRQ_STAT_DEC_OFF);
		}

		if (IS_G1(dev->hw_id[j]))
			/* reset PP */
			iowrite32(0, dev->hwregs[j] + HANTRO_IRQ_STAT_PP_OFF);

		for (i = 4; i < dev->iosize[j]; i += 4)
			iowrite32(0, dev->hwregs[j] + i);
	}
}

/*---------------------------------------------------------------------------
 *Function name   : dump_regs
 *Description     : Dump registers
 *
 *Return type     :
 *---------------------------------------------------------------------------
 */
#ifdef HANTRODEC_DEBUG
void dump_regs(hantrodec_t *dev)
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
	struct device *temp_class;
	struct resource *res;
	unsigned long reg_base;
	struct device_node *node;

	hantro_dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs_hantro");
	if (!res) {
		pr_err("hantro: unable to get vpu base addr\n");
		return -ENODEV;
	}
	reg_base = res->start;
	if ((ulong)reg_base != multicorebase[0]) {
		pr_err("hantrodec: regbase(0x%lX) not equal to expected value(0x%lX)\n", reg_base, multicorebase[0]);
		return -ENODEV;
	}

	hantro_clk_g1 = clk_get(&pdev->dev, "clk_hantro_g1");
	hantro_clk_g2 = clk_get(&pdev->dev, "clk_hantro_g2");
	hantro_clk_bus = clk_get(&pdev->dev, "clk_hantro_bus");
	if (IS_ERR(hantro_clk_g1) || IS_ERR(hantro_clk_g2) || IS_ERR(hantro_clk_bus)) {
		pr_err("hantro: get clock failed\n");
		return -ENODEV;
	}
	pr_debug("hantro: g1, g2, bus clock: 0x%lX, 0x%lX, 0x%lX\n", clk_get_rate(hantro_clk_g1),
				clk_get_rate(hantro_clk_g2), clk_get_rate(hantro_clk_bus));

	/*
	 * If integrate power-domains into blk-ctrl driver, vpu driver don't
	 * need handle it again.
	 */
	node = of_parse_phandle(pdev->dev.of_node, "power-domains", 0);
	if (!node) {
		pr_err("hantro: not get power-domains\n");
		return -ENODEV;
	}
	if (!strcmp(node->name, "blk-ctl") || !strcmp(node->name, "blk-ctrl"))
		hantrodec_data.skip_blkctrl = 1;
	else
		hantrodec_data.skip_blkctrl = 0;
	of_node_put(node);

	hantro_regulator = devm_regulator_get(&pdev->dev, "vpu");
	if (IS_ERR(hantro_regulator)) {
		pr_err("hantro: get regulator failed\n");
		return -ENODEV;
	}
	hantro_update_voltage(&pdev->dev);

	hantro_clk_enable(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pm_runtime_get_sync(&pdev->dev);

	hantro_ctrlblk_reset(&pdev->dev);

	err = hantrodec_init(pdev);
	if (0 != err) {
		pr_err("hantro: hantrodec_init failed\n");
		goto error;
	}

	hantro_class = class_create("mxc_hantro");
	if (IS_ERR(hantro_class)) {
		err = PTR_ERR(hantro_class);
		goto error;
	}
	temp_class = device_create(hantro_class, NULL, MKDEV(hantrodec_major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

#ifdef CONFIG_DEVFREQ_THERMAL
	hantrodec_data.cooling = thermal_of_cooling_device_register(pdev->dev.of_node,
		(char *)dev_name(&pdev->dev), &hantrodec_data, &hantro_cooling_ops);
	if (IS_ERR(hantrodec_data.cooling))
		goto err_out_class;
	thermal_event = 0;
	thermal_cur = 0;
	hantro_dynamic_clock = 0;
#endif
	timeout = 0;
	goto out;

err_out_class:
	device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
	class_destroy(hantro_class);
error:
	pr_err("hantro probe failed\n");
out:
	pm_runtime_put_sync(&pdev->dev);
	hantro_clk_disable(&pdev->dev);
	return err;
}

static int hantro_dev_remove(struct platform_device *pdev)
{
	hantro_clk_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	if (hantrodec_major > 0) {
#ifdef CONFIG_DEVFREQ_THERMAL
		thermal_cooling_device_unregister(hantrodec_data.cooling);
#endif
		device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
		class_destroy(hantro_class);
		hantrodec_cleanup();
		hantrodec_major = 0;
	}
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	hantro_clk_disable(&pdev->dev);

	if (!IS_ERR(hantro_clk_g1))
		clk_put(hantro_clk_g1);
	if (!IS_ERR(hantro_clk_g2))
		clk_put(hantro_clk_g2);
	if (!IS_ERR(hantro_clk_bus))
		clk_put(hantro_clk_bus);

	return 0;
}

#ifdef CONFIG_PM
static int hantro_suspend(struct device *dev)
{
	DecStoreRegs(&hantrodec_data);
	pm_runtime_put_sync_suspend(dev);   //power off
	return 0;
}
static int hantro_resume(struct device *dev)
{
	pm_runtime_get_sync(dev);     //power on
	hantro_ctrlblk_reset(dev);
	DecRestoreRegs(&hantrodec_data);
	return 0;
}
static int hantro_runtime_suspend(struct device *dev)
{
//	release_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static int hantro_runtime_resume(struct device *dev)
{
//	request_bus_freq(BUS_FREQ_HIGH);
	hantro_ctrlblk_reset(dev);
	return 0;
}

static const struct dev_pm_ops hantro_pm_ops = {
	SET_RUNTIME_PM_OPS(hantro_runtime_suspend, hantro_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantro_suspend, hantro_resume)
};
#endif //CONFIG_PM

static const struct of_device_id hantro_of_match[] = {
	{ .compatible = "nxp,imx8mq-hantro", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, hantro_of_match);


static struct platform_driver mxchantro_driver = {
	.driver = {
	.name = "mxc_hantro",
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

#if 0
	if (!IS_ERR(hantro_regulator))
		regulator_put(hantro_regulator);
#endif
	platform_driver_unregister(&mxchantro_driver);
}

module_init(hantro_init);
module_exit(hantro_exit);

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Driver module for Hantro Decoder/Post-Processor");

