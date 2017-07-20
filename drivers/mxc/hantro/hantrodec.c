/*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (c) 2015-2017, VeriSilicon Inc.
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
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************/

#include <linux/hantrodec.h>
#include "dwl_defs.h"

#include <asm/io.h>
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
#define HANTRO_G2_DEC_LAST_REG             HANTRO_G2_DEC_REGS-1

/* Logic module IRQs */
#define HXDEC_NO_IRQ                    -1

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define DEC_IO_SIZE_MAX             (MAX(HANTRO_G2_DEC_REGS, HANTRO_G1_TOTAL_REGS)* 4) 
                                                         
/********************************************************************
*                                              PORTING SEGMENT 
* NOTES: customer should modify these configuration if do porting to own platform.
* Please guarantee the base_addr, io_size,dec_irq belong to same core.
********************************************************************/

#define HXDEC_MAX_CORES                 4
#define MULTI_CORE                     /*this macro defines single core or multicore*/
//#define CLK_CFG                        /*this macro defines use kernel clk cfg or not*/
#ifdef CLK_CFG
#define CLK_ID   "hantrodec_clk" /*this id should conform with platform define*/
#endif

/* Logic module base address */
#define SOCLE_LOGIC_0_BASE              0x38300000
#define SOCLE_LOGIC_1_BASE              0x38310000
#define BLK_CTL_BASE                          0x38320000

#define VEXPRESS_LOGIC_0_BASE           0xFC010000
#define VEXPRESS_LOGIC_1_BASE           0xFC020000

#ifdef USE_64BIT_ENV
#define DEC_IO_SIZE_0             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */
#else
#define DEC_IO_SIZE_0             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */
#endif
#ifdef MULTI_CORE
#ifdef USE_64BIT_ENV
#define DEC_IO_SIZE_1             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */
#else
#define DEC_IO_SIZE_1             ((HANTRO_G2_DEC_REGS) * 4) /* bytes */
#endif

#endif
#define DEC_IRQ_0                 7
#ifdef MULTI_CORE
#define DEC_IRQ_1                 8
#endif

/***********************************************************************/

#define IS_G1(hw_id)    ((hw_id == 0x6731)? 1:0)

static const int DecHwId[] = {
  0x8190,	/* Legacy HW */
  0x8170,
  0x9170,
  0x9190,
  0x6731,	/* G1 */
  0x6732	/* G2 */
};

unsigned long base_port = -1; /*for single core and set when module init*/
volatile unsigned char *reg = NULL;

ulong multicorebase[HXDEC_MAX_CORES] = {
  SOCLE_LOGIC_0_BASE,
  SOCLE_LOGIC_1_BASE,
  -1,
  -1
};

int irq_0 = DEC_IRQ_0;
#ifndef MULTI_CORE
int elements = 1;
#else
int irq_1 = DEC_IRQ_1;
int elements = 2;
#endif

#ifdef CLK_CFG
  struct clk *clk_cfg;
  int is_clk_on;
  struct timer_list timer;
#endif

static struct class *hantro_class;
#define DEVICE_NAME		"mxc_hantro"

static struct device *hantro_dev;
static struct clk *hantro_clk_g1;
static struct clk *hantro_clk_g2;
static struct clk * hantro_clk_bus;


static int hantro_dbg=0;
module_param(hantro_dbg, int, 0644);
MODULE_PARM_DESC(hantro_dbg, "Debug level (0-1)");
#undef PDEBUG
#define PDEBUG(fmt, arg...)     \
    do {                                      \
    if (hantro_dbg > 0){printk(KERN_DEBUG fmt ,## arg);} \
    } while (0)

/* module_param(name, type, perm) */
module_param(base_port, ulong, 0);
module_param(irq_0, int, 0);
module_param_array(multicorebase, ulong, &elements, 0644);

static int hantrodec_major = 0; /* dynamic allocation */
static const struct of_device_id mmp_timer_dt_ids_0[] = {
	{ .compatible = "nxp,imx8dv-g1", },
	{}
};
#ifdef MULTI_CORE
static const struct of_device_id mmp_timer_dt_ids_1[] = {
	{ .compatible = "nxp,imx8dv-g2", },
	{}
};
#endif


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
} hantrodec_t;

static hantrodec_t hantrodec_data; /* dynamic allocation? */

static int ReserveIO(void);
static void ReleaseIO(void);

static void ResetAsic(hantrodec_t * dev);

#ifdef HANTRODEC_DEBUG
static void dump_regs(hantrodec_t *dev);
#endif

/* IRQ handler */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
static irqreturn_t hantrodec_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
static irqreturn_t hantrodec_isr(int irq, void *dev_id);
#endif


static u32 dec_regs[HXDEC_MAX_CORES][DEC_IO_SIZE_MAX/4];
struct semaphore dec_core_sem;
struct semaphore pp_core_sem;

static int dec_irq = 0;
static int pp_irq = 0;

atomic_t irq_rx = ATOMIC_INIT(0);
atomic_t irq_tx = ATOMIC_INIT(0);

static struct file* dec_owner[HXDEC_MAX_CORES];
static struct file* pp_owner[HXDEC_MAX_CORES];

/* spinlock_t owner_lock = SPIN_LOCK_UNLOCKED; */
DEFINE_SPINLOCK(owner_lock);

DECLARE_WAIT_QUEUE_HEAD(dec_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(pp_wait_queue);

DECLARE_WAIT_QUEUE_HEAD(hw_queue);
#ifdef CLK_CFG
DEFINE_SPINLOCK(clk_lock);
#endif

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

static void ReadCoreConfig(hantrodec_t *dev) {
  int c;
  u32 reg, tmp, mask;

  memset(cfg, 0, sizeof(cfg));

  for(c = 0; c < dev->cores; c++) {
    /* Decoder configuration */
    if (IS_G1(dev->hw_id[c])) {
      reg = ioread32(dev->hwregs[c] + HANTRODEC_SYNTH_CFG * 4);

      tmp = (reg >> DWL_H264_E) & 0x3U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has H264\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

      tmp = (reg >> DWL_JPEG_E) & 0x01U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has JPEG\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

      tmp = (reg >> DWL_MPEG4_E) & 0x3U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has MPEG4\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

      tmp = (reg >> DWL_VC1_E) & 0x3U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has VC1\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC: 0;

      tmp = (reg >> DWL_MPEG2_E) & 0x01U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has MPEG2\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

      tmp = (reg >> DWL_VP6_E) & 0x01U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has VP6\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

      reg = ioread32(dev->hwregs[c] + HANTRODEC_SYNTH_CFG_2 * 4);

      /* VP7 and WEBP is part of VP8 */
      mask =  (1 << DWL_VP8_E) | (1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
      tmp = (reg & mask);
      if(tmp & (1 << DWL_VP8_E))
        printk(KERN_DEBUG "hantrodec: Core[%d] has VP8\n", c);
      if(tmp & (1 << DWL_VP7_E))
        printk(KERN_DEBUG "hantrodec: Core[%d] has VP7\n", c);
      if(tmp & (1 << DWL_WEBP_E))
        printk(KERN_DEBUG "hantrodec: Core[%d] has WebP\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

      tmp = (reg >> DWL_AVS_E) & 0x01U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has AVS\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC: 0;

      tmp = (reg >> DWL_RV_E) & 0x03U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has RV\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

      /* Post-processor configuration */
      reg = ioread32(dev->hwregs[c] + HANTROPP_SYNTH_CFG * 4);
    }else {
      reg = ioread32(dev->hwregs[c] + HANTRODEC_SYNTH_CFG_2 * 4);

      tmp = (reg >> DWL_HEVC_E) & 0x3U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has HEVC\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

      tmp = (reg >> DWL_VP9_E) & 0x03U;
      if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has VP9\n", c);
      cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;
    }

    /* Post-processor configuration */
    reg = ioread32(dev->hwregs[c] + HANTRODECPP_SYNTH_CFG * 4);

    tmp = (reg >> DWL_PP_E) & 0x01U;
    if(tmp) printk(KERN_DEBUG "hantrodec: Core[%d] has PP\n", c);
    cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
  }
}

static int CoreHasFormat(const u32 *cfg, int Core, u32 format) {
  return (cfg[Core] & (1 << format)) ? 1 : 0;
}

int GetDecCore(long Core, hantrodec_t *dev, struct file* filp) {
  int success = 0;
  unsigned long flags;

  spin_lock_irqsave(&owner_lock, flags);
  if(dec_owner[Core] == NULL ) {
    dec_owner[Core] = filp;
    success = 1;
  }

  spin_unlock_irqrestore(&owner_lock, flags);

  return success;
}

int GetDecCoreAny(long *Core, hantrodec_t *dev, struct file* filp,
                  unsigned long format) {
  int success = 0;
  long c;

  *Core = -1;

  for(c = 0; c < dev->cores; c++) {
    /* a free Core that has format */
    if(CoreHasFormat(cfg, c, format) && GetDecCore(c, dev, filp)) {
      success = 1;
      *Core = c;
      break;
    }
  }

  return success;
}
int GetDecCoreID(hantrodec_t *dev, struct file* filp,
                  unsigned long format) {
  long c;

  int core_id = -1;

  for(c = 0; c < dev->cores; c++) {
    /* a Core that has format */
    if(CoreHasFormat(cfg, c, format)) {
      core_id = c;
      break;
    }
  }
  PDEBUG("GetDecCoreID=%d\n",core_id);
  return core_id;
}

static int hantrodec_choose_core(int is_g1) {
  volatile unsigned char *reg = NULL;
  unsigned int blk_base = BLK_CTL_BASE;

  PDEBUG("hantrodec_choose_core\n");
  if (!request_mem_region(blk_base, 0x1000, "blk_ctl"))
  {
	  printk(KERN_INFO "blk_ctl: failed to reserve HW regs\n");
	  return -EBUSY;
  }

  reg = (volatile u8 *) ioremap_nocache(blk_base, 0x1000);

  if (reg == NULL )
  {
	  printk(KERN_INFO "blk_ctl: failed to ioremap HW regs\n");
	  if (reg)								   
		  iounmap((void *)reg); 			  
	  release_mem_region(blk_base, 0x1000);  
	  return -EBUSY;
  }

  // G1 use, set to 1; G2 use, set to 0, choose the one you are using
  if (is_g1)
     iowrite32(0x1, reg + 0x14);  // VPUMIX only use G1, user should modify the reg according to platform design
  else
     iowrite32(0x0, reg + 0x14); // VPUMIX only use G2, user should modify the reg according to platform design

  if (reg)
     iounmap((void *)reg); 			  
  release_mem_region(blk_base, 0x1000);   
  PDEBUG("hantrodec_choose_core OK!\n");
  return 0;
}


long ReserveDecoder(hantrodec_t *dev, struct file* filp, unsigned long format) {
  long Core = -1;

  /* reserve a Core */
  if (down_interruptible(&dec_core_sem))
    return -ERESTARTSYS;

  /* lock a Core that has specific format*/
  if(wait_event_interruptible(hw_queue,
                              GetDecCoreAny(&Core, dev, filp, format) != 0 ))
    return -ERESTARTSYS;
#if 1
  if(IS_G1(dev->hw_id[Core]))
  	{
	  if (0 == hantrodec_choose_core(1)) {
	    PDEBUG("G1 is reserved\n");
      }
	  else
	  	return -1;
  	}
  else
  	{
      if (0 == hantrodec_choose_core(0)){
        PDEBUG("G2 is reserved\n");
      }
	  else
	  	return -1;
  	}
#endif
  	
  return Core;
}

void ReleaseDecoder(hantrodec_t *dev, long Core) {
  u32 status;
  unsigned long flags;

  status = ioread32(dev->hwregs[Core] + HANTRODEC_IRQ_STAT_DEC_OFF);

  /* make sure HW is disabled */
  if(status & HANTRODEC_DEC_E) {
    printk(KERN_INFO "hantrodec: DEC[%li] still enabled -> reset\n", Core);

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

long ReservePostProcessor(hantrodec_t *dev, struct file* filp) {
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

void ReleasePostProcessor(hantrodec_t *dev, long Core) {
  unsigned long flags;

  u32 status = ioread32(dev->hwregs[Core] + HANTRO_IRQ_STAT_PP_OFF);

  /* make sure HW is disabled */
  if(status & HANTRO_PP_E) {
    printk(KERN_INFO "hantrodec: PP[%li] still enabled -> reset\n", Core);

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

long ReserveDecPp(hantrodec_t *dev, struct file* filp, unsigned long format) {
  /* reserve Core 0, DEC+PP for pipeline */
  unsigned long flags;

  long Core = 0;

  /* check that Core has the requested dec format */
  if(!CoreHasFormat(cfg, Core, format))
    return -EFAULT;

  /* check that Core has PP */
  if(!CoreHasFormat(cfg, Core, DWL_CLIENT_TYPE_PP))
    return -EFAULT;
  
  /* reserve a Core */
  if (down_interruptible(&dec_core_sem))
    return -ERESTARTSYS;
  
  /* wait until the Core is available */
  if(wait_event_interruptible(hw_queue,
                              GetDecCore(Core, dev, filp) != 0)) {
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

long DecFlushRegs(hantrodec_t *dev, struct core_desc *Core) {
  long ret = 0, i;

  u32 id = Core->id;

  if (IS_G1(dev->hw_id[id])) {
    /* copy original dec regs to kernal space*/
    ret = copy_from_user(dec_regs[id], Core->regs, HANTRO_DEC_ORG_REGS*4);
    if (ret) {
      PDEBUG("copy_from_user failed, returned %li\n", ret);
      return -EFAULT;
    }
#ifdef USE_64BIT_ENV
    /* copy extended dec regs to kernal space*/
    ret = copy_from_user(dec_regs[id] + HANTRO_DEC_EXT_FIRST_REG,
                         Core->regs + HANTRO_DEC_EXT_FIRST_REG,
                         HANTRO_DEC_EXT_REGS*4);
#endif
    if (ret) {
      PDEBUG("copy_from_user failed, returned %li\n", ret);
      return -EFAULT;
    }

    /* write dec regs but the status reg[1] to hardware */
    /* both original and extended regs need to be written */
    for(i = 2; i <= HANTRO_DEC_ORG_LAST_REG; i++)
    {		
      iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
    }
#ifdef USE_64BIT_ENV
    for(i = HANTRO_DEC_EXT_FIRST_REG; i <= HANTRO_DEC_EXT_LAST_REG; i++)
      iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#endif
  } else {
    ret = copy_from_user(dec_regs[id], Core->regs, HANTRO_G2_DEC_REGS*4);
    if (ret) {
      PDEBUG("copy_from_user failed, returned %li\n", ret);
      return -EFAULT;
    }

    /* write all regs but the status reg[1] to hardware */
    for(i = 2; i <= HANTRO_G2_DEC_LAST_REG; i++)
    {
 #if 0
      if(i==2)   
			//dec_regs[id][i] = 0x78777777;
			//dec_regs[id][i] = 0x78778787;
			//c_regs[id][i] = 0x78888888; //64bit
      	{
      	    printk("reg2=%08x\n",dec_regs[id][i]);
			//dec_regs[id][i] = 0xF0F00000;//128bit 0xF0F0F000
      	}
			//dec_regs[id][i] = 0xF0F0000F;//128bit 0xF0F00000 for big endian
	  if(i==58)  
	  	{
	  	    printk("reg58=%08x\n",dec_regs[id][i]);
			//dec_regs[id][i]= 0x210;//128bit
			//dec_regs[id][i]= 0x110;//64bit
	  	}
      if(i==3)  
      	{
      	    printk("reg3=%08x\n",dec_regs[id][i]);
			//dec_regs[id][i] |= 0x00F00000;//128bit
			//dec_regs[id][i]= 0x110;//64bit
      	}
 #endif
      iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);

	}
  }

  /* write the status register, which may start the decoder */
  iowrite32(dec_regs[id][1], dev->hwregs[id] + 4);

  PDEBUG("flushed registers on Core %d\n", id);

  return 0;
}

long DecRefreshRegs(hantrodec_t *dev, struct core_desc *Core) {
  long ret, i;
  u32 id = Core->id;

  if (IS_G1(dev->hw_id[id])) {
#ifdef USE_64BIT_ENV
    /* user has to know exactly what they are asking for */
#if 1 //eagle for compiler
    PDEBUG("DecRefreshRegs: size: %d \n", Core->size);
#else
    if(Core->size != (HANTRO_DEC_TOTAL_REGS * 4))
      return -EFAULT;
#endif
#else
    /* user has to know exactly what they are asking for */
    //if(Core->size != (HANTRO_DEC_ORG_REGS * 4))
    //  return -EFAULT;
#endif
    /* read all registers from hardware */
    /* both original and extended regs need to be read */
    for(i = 0; i <= HANTRO_DEC_ORG_LAST_REG; i++)
      dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
    for(i = HANTRO_DEC_EXT_FIRST_REG; i <= HANTRO_DEC_EXT_LAST_REG; i++)
      dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#endif
    /* put registers to user space*/
    /* put original registers to user space*/
    ret = copy_to_user(Core->regs, dec_regs[id], HANTRO_DEC_ORG_REGS*4);
#ifdef USE_64BIT_ENV
    /*put extended registers to user space*/
    ret = copy_to_user(Core->regs + HANTRO_DEC_EXT_FIRST_REG,
                       dec_regs[id] + HANTRO_DEC_EXT_FIRST_REG,
                       HANTRO_DEC_EXT_REGS * 4);
#endif
    if (ret) {
      PDEBUG("copy_to_user failed, returned %li\n", ret);
      return -EFAULT;
    }
  } else {
    /* user has to know exactly what they are asking for */
    if(Core->size != (HANTRO_G2_DEC_REGS * 4))
      return -EFAULT;

    /* read all registers from hardware */
    for(i = 0; i <= HANTRO_G2_DEC_LAST_REG; i++)
      dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);

    /* put registers to user space*/
    ret = copy_to_user(Core->regs, dec_regs[id], HANTRO_G2_DEC_REGS*4);
    if (ret) {
      PDEBUG("copy_to_user failed, returned %li\n", ret);
      return -EFAULT;
    }
  }
  return 0;
}

static int CheckDecIrq(hantrodec_t *dev, int id) {
  unsigned long flags;
  int rdy = 0;

  const u32 irq_mask = (1 << id);
  
  spin_lock_irqsave(&owner_lock, flags);

  if(dec_irq & irq_mask) {
    /* reset the wait condition(s) */
    dec_irq &= ~irq_mask;
    rdy = 1;
  }

  spin_unlock_irqrestore(&owner_lock, flags);

  return rdy;
}

long WaitDecReadyAndRefreshRegs(hantrodec_t *dev, struct core_desc *Core) {
  u32 id = Core->id;

  PDEBUG("wait_event_interruptible DEC[%d]\n", id);

  if(wait_event_interruptible(dec_wait_queue, CheckDecIrq(dev, id))) {
    PDEBUG("DEC[%d]  wait_event_interruptible interrupted\n", id);
    return -ERESTARTSYS;
  }

  atomic_inc(&irq_tx);

  /* refresh registers */
  return DecRefreshRegs(dev, Core);
}

long PPFlushRegs(hantrodec_t *dev, struct core_desc *Core) {
  long ret = 0;
  u32 id = Core->id;
  u32 i;

  /* copy original dec regs to kernal space*/
  ret = copy_from_user(dec_regs[id] + HANTRO_PP_ORG_FIRST_REG,
                       Core->regs + HANTRO_PP_ORG_FIRST_REG,
                       HANTRO_PP_ORG_REGS*4);
#ifdef USE_64BIT_ENV
  /* copy extended dec regs to kernal space*/
  ret = copy_from_user(dec_regs[id] + HANTRO_PP_EXT_FIRST_REG,
                       Core->regs + HANTRO_PP_EXT_FIRST_REG,
                       HANTRO_PP_EXT_REGS*4);
#endif
  if (ret) {
    PDEBUG("copy_from_user failed, returned %li\n", ret);
    return -EFAULT;
  }

  /* write all regs but the status reg[1] to hardware */
  /* both original and extended regs need to be written */
  for(i = HANTRO_PP_ORG_FIRST_REG + 1; i <= HANTRO_PP_ORG_LAST_REG; i++)
    iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
  for(i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
    iowrite32(dec_regs[id][i], dev->hwregs[id] + i*4);
#endif
  /* write the stat reg, which may start the PP */
  iowrite32(dec_regs[id][HANTRO_PP_ORG_FIRST_REG],
            dev->hwregs[id] + HANTRO_PP_ORG_FIRST_REG * 4);

  return 0;
}

long PPRefreshRegs(hantrodec_t *dev, struct core_desc *Core) {
  long i, ret;
  u32 id = Core->id;
#ifdef USE_64BIT_ENV
  /* user has to know exactly what they are asking for */
  if(Core->size != (HANTRO_PP_TOTAL_REGS * 4))
    return -EFAULT;
#else
  /* user has to know exactly what they are asking for */
  if(Core->size != (HANTRO_PP_ORG_REGS * 4))
    return -EFAULT;
#endif

  /* read all registers from hardware */
  /* both original and extended regs need to be read */
  for(i = HANTRO_PP_ORG_FIRST_REG; i <= HANTRO_PP_ORG_LAST_REG; i++)
    dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#ifdef USE_64BIT_ENV
  for(i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
    dec_regs[id][i] = ioread32(dev->hwregs[id] + i*4);
#endif
  /* put registers to user space*/
  /* put original registers to user space*/
  ret = copy_to_user(Core->regs + HANTRO_PP_ORG_FIRST_REG,
                     dec_regs[id] + HANTRO_PP_ORG_FIRST_REG,
                     HANTRO_PP_ORG_REGS*4);
#ifdef USE_64BIT_ENV
  /* put extended registers to user space*/
  ret = copy_to_user(Core->regs + HANTRO_PP_EXT_FIRST_REG,
                     dec_regs[id] + HANTRO_PP_EXT_FIRST_REG,
                     HANTRO_PP_EXT_REGS * 4);
#endif
  if (ret) {
    PDEBUG("copy_to_user failed, returned %li\n", ret);
    return -EFAULT;
  }

  return 0;
}

static int CheckPPIrq(hantrodec_t *dev, int id) {
  unsigned long flags;
  int rdy = 0;

  const u32 irq_mask = (1 << id);

  spin_lock_irqsave(&owner_lock, flags);

  if(pp_irq & irq_mask) {
    /* reset the wait condition(s) */
    pp_irq &= ~irq_mask;
    rdy = 1;
  }

  spin_unlock_irqrestore(&owner_lock, flags);

  return rdy;
}

long WaitPPReadyAndRefreshRegs(hantrodec_t *dev, struct core_desc *Core) {
  u32 id = Core->id;

  PDEBUG("wait_event_interruptible PP[%d]\n", id);

  if(wait_event_interruptible(pp_wait_queue, CheckPPIrq(dev, id))) {
    PDEBUG("PP[%d]  wait_event_interruptible interrupted\n", id);
    return -ERESTARTSYS;
  }

  atomic_inc(&irq_tx);

  /* refresh registers */
  return PPRefreshRegs(dev, Core);
}

static int CheckCoreIrq(hantrodec_t *dev, const struct file *filp, int *id) {
  unsigned long flags;
  int rdy = 0, n = 0;

  do {
    u32 irq_mask = (1 << n);

    spin_lock_irqsave(&owner_lock, flags);

    if(dec_irq & irq_mask) {
      if (dec_owner[n] == filp) {
        /* we have an IRQ for our client */

        /* reset the wait condition(s) */
        dec_irq &= ~irq_mask;

        /* signal ready Core no. for our client */
        *id = n;

        rdy = 1;
		
        spin_unlock_irqrestore(&owner_lock, flags);
        break;
      } else if(dec_owner[n] == NULL) {
        /* zombie IRQ */
        printk(KERN_INFO "IRQ on Core[%d], but no owner!!!\n", n);

        /* reset the wait condition(s) */
        dec_irq &= ~irq_mask;
      }
    }

    spin_unlock_irqrestore(&owner_lock, flags);

    n++; /* next Core */
  } while(n < dev->cores);

  return rdy;
}

long WaitCoreReady(hantrodec_t *dev, const struct file *filp, int *id) {
  PDEBUG("wait_event_interruptible CORE\n");

  if(wait_event_interruptible(dec_wait_queue, CheckCoreIrq(dev, filp, id))) {
    PDEBUG("CORE  wait_event_interruptible interrupted\n");
    return -ERESTARTSYS;
  }

  atomic_inc(&irq_tx);

  return 0;
}

/*------------------------------------------------------------------------------
 Function name   : hantrodec_ioctl
 Description     : communication method to/from the user space

 Return type     : long
------------------------------------------------------------------------------*/

static long hantrodec_ioctl(struct file *filp, unsigned int cmd,
                            unsigned long arg) {
  int err = 0;
  long tmp;
#ifdef CLK_CFG
  unsigned long flags;
#endif

#ifdef HW_PERFORMANCE
  struct timeval *end_time_arg;
#endif

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
  
#ifdef CLK_CFG
  spin_lock_irqsave(&clk_lock, flags);
  if (clk_cfg!=NULL && !IS_ERR(clk_cfg)&&(is_clk_on==0))
  	{
	  printk("turn on clock by user\n");
	  if (clk_enable(clk_cfg))
	  	{
	  	 spin_unlock_irqrestore(&clk_lock, flags);
		 return -EFAULT;
	  	}
      else
	  	is_clk_on=1;
  	}
  spin_unlock_irqrestore(&clk_lock, flags);
  mod_timer(&timer, jiffies + 10*HZ); /*the interval is 10s*/
#endif

  switch (_IOC_NR(cmd)) {
  case _IOC_NR(HANTRODEC_IOC_CLI):
  	{
	  __u32 id; 
      __get_user(id, (__u32*)arg);
      
      if(id >= hantrodec_data.cores) {
    	  return -EFAULT;
      }
      disable_irq(hantrodec_data.irq[id]);
      break;
  	}
  case _IOC_NR(HANTRODEC_IOC_STI):
  	{
	  __u32 id; 
      __get_user(id, (__u32*)arg);
      
      if(id >= hantrodec_data.cores) {
    	  return -EFAULT;
      }
      enable_irq(hantrodec_data.irq[id]);
      break;
  	}
  case _IOC_NR(HANTRODEC_IOCGHWOFFSET):
  	{
      __u32 id; 
      __get_user(id, (__u32*)arg);
      
      if(id >= hantrodec_data.cores) {
    	  return -EFAULT;
      }
      
      __put_user(multicorebase[id], (unsigned long *) arg);
      break;
  	}
  case _IOC_NR(HANTRODEC_IOCGHWIOSIZE):
  	{
      __u32 id;
      __u32 io_size;
      __get_user(id, (__u32*)arg);
      
      if(id >= hantrodec_data.cores) {
    	  return -EFAULT;
      }
      io_size = hantrodec_data.iosize[id];
      __put_user(io_size, (u32 *) arg);

  	  return 0;
  	}
  case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS): {
    tmp = copy_to_user((u64 *) arg, multicorebase, sizeof(multicorebase));
    if (err) {
      PDEBUG("copy_to_user failed, returned %li\n", tmp);
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
    tmp = copy_from_user(&Core, (void*)arg, sizeof(struct core_desc));
    if (tmp) {
      PDEBUG("copy_from_user failed, returned %li\n", tmp);
      return -EFAULT;
    }

    DecFlushRegs(&hantrodec_data, &Core);
    break;
  }
  case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG): {
    struct core_desc Core;

    /* get registers from user space*/
    tmp = copy_from_user(&Core, (void*)arg, sizeof(struct core_desc));
    if (tmp) {
      PDEBUG("copy_from_user failed, returned %li\n", tmp);
      return -EFAULT;
    }

    PPFlushRegs(&hantrodec_data, &Core);
    break;
  }
  case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
    struct core_desc Core;

    /* get registers from user space*/
    tmp = copy_from_user(&Core, (void*)arg, sizeof(struct core_desc));
    if (tmp) {
      PDEBUG("copy_from_user failed, returned %li\n", tmp);
      return -EFAULT;
    }

    return DecRefreshRegs(&hantrodec_data, &Core);
  }
  case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG): {
    struct core_desc Core;

    /* get registers from user space*/
    tmp = copy_from_user(&Core, (void*)arg, sizeof(struct core_desc));
    if (tmp) {
      PDEBUG("copy_from_user failed, returned %li\n", tmp);
      return -EFAULT;
    }

    return PPRefreshRegs(&hantrodec_data, &Core);
  }
  case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
    PDEBUG("Reserve DEC Core, format = %li\n", arg);
    return ReserveDecoder(&hantrodec_data, filp, arg);
  }
  case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
    if(arg >= hantrodec_data.cores || dec_owner[arg] != filp) {
      PDEBUG("bogus DEC release, Core = %li\n", arg);
      return -EFAULT;
    }

    PDEBUG("Release DEC, Core = %li\n", arg);

    ReleaseDecoder(&hantrodec_data, arg);

    break;
  }
  case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
    return ReservePostProcessor(&hantrodec_data, filp);
  case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE): {
    if(arg != 0 || pp_owner[arg] != filp) {
      PDEBUG("bogus PP release %li\n", arg);
      return -EFAULT;
    }

    ReleasePostProcessor(&hantrodec_data, arg);

    break;
  }
  case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT): {
    struct core_desc Core;

    /* get registers from user space */
    tmp = copy_from_user(&Core, (void*)arg, sizeof(struct core_desc));
    if (tmp) {
      PDEBUG("copy_from_user failed, returned %li\n", tmp);
      return -EFAULT;
    }

    return WaitDecReadyAndRefreshRegs(&hantrodec_data, &Core);
  }
  case _IOC_NR(HANTRODEC_IOCX_PP_WAIT): {
    struct core_desc Core;

    /* get registers from user space */
    tmp = copy_from_user(&Core, (void*)arg, sizeof(struct core_desc));
    if (tmp) {
      PDEBUG("copy_from_user failed, returned %li\n", tmp);
      return -EFAULT;
    }

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
    __get_user(id, (u32*)arg);

    if(id >= hantrodec_data.cores) {
      return -EFAULT;
    }
    id = ioread32(hantrodec_data.hwregs[id]);
    __put_user(id, (u32 *) arg);
	return 0;
  }
  case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
    PDEBUG("Get DEC Core_id, format = %li\n", arg);
	return GetDecCoreID(&hantrodec_data, filp, arg);
  }
  case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
    printk(KERN_INFO "hantrodec: dec_irq     = 0x%08x \n", dec_irq);
    printk(KERN_INFO "hantrodec: pp_irq      = 0x%08x \n", pp_irq);

    printk(KERN_INFO "hantrodec: IRQs received/sent2user = %d / %d \n",
           atomic_read(&irq_rx), atomic_read(&irq_tx));

    for (tmp = 0; tmp < hantrodec_data.cores; tmp++) {
      printk(KERN_INFO "hantrodec: dec_core[%li] %s\n",
             tmp, dec_owner[tmp] == NULL ? "FREE" : "RESERVED");
      printk(KERN_INFO "hantrodec: pp_core[%li]  %s\n",
             tmp, pp_owner[tmp] == NULL ? "FREE" : "RESERVED");
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
        get_user(tmp, &up->regs)){
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
        put_user(tmp, &up->regs)){
            return -EFAULT;
    }
    return 0;
}
static long hantrodec_ioctl32(struct file *filp, unsigned int cmd,unsigned long arg)
{
#define HANTRO_IOCTL32(err,filp,cmd, arg) {\
      mm_segment_t old_fs = get_fs(); \
      set_fs(KERNEL_DS); \
      err = hantrodec_ioctl(filp, cmd, arg);\
      if(err) return err;\
      set_fs(old_fs);\
}
    union {
        struct core_desc kcore;
        unsigned long kux;
        unsigned int kui;
    } karg;
    void __user *up = compat_ptr(arg);
    long err = 0;

    switch(_IOC_NR(cmd)){
        case _IOC_NR(HANTRODEC_IOCGHWOFFSET):
        case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS):
            err = get_user(karg.kux, (s32 __user *)up);
            if(err) return err;
            HANTRO_IOCTL32(err,filp,cmd, (unsigned long)&karg);
            err = put_user(((s32)karg.kux), (s32 __user *)up);
            break;
        case _IOC_NR(HANTRODEC_IOCGHWIOSIZE):
        case _IOC_NR(HANTRODEC_IOC_MC_CORES):
        case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT):
        case _IOC_NR(HANTRODEC_IOX_ASIC_ID):
            err = get_user(karg.kui, (s32 __user *)up);
            if(err) return err;
            HANTRO_IOCTL32(err,filp,cmd, (unsigned long)&karg);
            err = put_user(((s32)karg.kui), (s32 __user *)up);
            break;
        case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG):
        case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG):
        case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT):
        case _IOC_NR(HANTRODEC_IOCX_PP_WAIT):
        case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG):
        case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG):
            err = get_hantro_core_desc32(&karg.kcore, up);
            if(err) return err;
            HANTRO_IOCTL32(err,filp,cmd, (unsigned long)&karg);
            err = put_hantro_core_desc32(&karg.kcore, up);
            break;
        default:
            err = hantrodec_ioctl(filp, cmd, (unsigned long)up);  
            break;
    }

    return err;
}

#endif //ifdef CONFIG_COMPAT

/*------------------------------------------------------------------------------
 Function name   : hantrodec_open
 Description     : open method

 Return type     : int
------------------------------------------------------------------------------*/

static int hantrodec_open(struct inode *inode, struct file *filp) {
  PDEBUG("dev opened\n");
  //pm_runtime_get_sync(hantro_dev);
  return 0;
}

/*------------------------------------------------------------------------------
 Function name   : hantrodec_release
 Description     : Release driver

 Return type     : int
------------------------------------------------------------------------------*/

static int hantrodec_release(struct inode *inode, struct file *filp) {
  int n;
  hantrodec_t *dev = &hantrodec_data;

  PDEBUG("closing ...\n");

  for(n = 0; n < dev->cores; n++) {
    if(dec_owner[n] == filp) {
      PDEBUG("releasing dec Core %i lock\n", n);
      ReleaseDecoder(dev, n);
    }
  }

  for(n = 0; n < 1; n++) {
    if(pp_owner[n] == filp) {
      PDEBUG("releasing pp Core %i lock\n", n);
      ReleasePostProcessor(dev, n);
    }
  }

  //pm_runtime_put_sync_suspend(hantro_dev);
  PDEBUG("closed\n");
  return 0;
}

/*------------------------------------------------------------------------------
 Function name   : hantro_mmap
 Description     : memory map interface for hantro file operation

 Return type     : int
------------------------------------------------------------------------------*/
static int hantro_mmap(struct file *fp, struct vm_area_struct *vm)
{
  if (vm->vm_pgoff==(multicorebase[0] >> PAGE_SHIFT) || vm->vm_pgoff==(multicorebase[1] >> PAGE_SHIFT)){
    vm->vm_flags |= VM_IO;
    vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
    PDEBUG("hantro mmap: size=0x%lX, page off=0x%lX\n",(vm->vm_end - vm->vm_start), vm->vm_pgoff);
    return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff, vm->vm_end - vm->vm_start,
               vm->vm_page_prot) ? -EAGAIN : 0;
  }
  else{
    PDEBUG("invalid map offset :0x%lX \n",vm->vm_pgoff);
    return -EINVAL;
  }
}

#ifdef CLK_CFG
void hantrodec_disable_clk(unsigned long value)
{
  unsigned long flags;
 /*entering this function means decoder is idle over expiry.So disable clk*/
 if (clk_cfg!=NULL && !IS_ERR(clk_cfg))
  {
    spin_lock_irqsave(&clk_lock, flags);
    if (is_clk_on==1)
     {
	  clk_disable(clk_cfg);
      is_clk_on = 0;
	  printk("turned off hantrodec clk\n");
     }
    spin_unlock_irqrestore(&clk_lock, flags);
   }
}
#endif

/* VFS methods */
static struct file_operations hantrodec_fops = {
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

/*------------------------------------------------------------------------------
 Function name   : hantrodec_init
 Description     : Initialize the driver

 Return type     : int
------------------------------------------------------------------------------*/

int hantrodec_init(void) {
  int result, i;

  PDEBUG("module init\n");

  printk(KERN_DEBUG "hantrodec: dec/pp kernel module. \n");
#ifdef VSI
/*This segment is related with customer platform CFG and should be modified for specified use*/
/*---------------------------------------------------------------------------*/
{
  //map PGC_CPU_MAPPING
  unsigned long pgc_base = 0x303A0000;
  //volatile unsigned char *reg = NULL;
  unsigned int val = 0;
  if (!request_mem_region(pgc_base, 0x1000, "pgc_cpu_mapping"))
  {
	  printk(KERN_INFO "pgc_cpu_mapping: failed to reserve HW regs\n");
	  return -EBUSY;
  }

  reg = (volatile u8 *) ioremap_nocache(pgc_base, 0x1000);

  if (reg == NULL )
  {
	  printk(KERN_INFO "pgc_cpu_mapping: failed to ioremap HW regs\n");
	  if (reg)									 
		  iounmap((void *)reg); 			
	  release_mem_region(pgc_base, 0x1000);  
	  return -EBUSY;
  }   
  val =  ioread32(reg + 0xEC);
  printk(KERN_INFO" the initial val = 0x%x\n", val);
  iowrite32(0x0000ffffl, reg+0xEC);
  val =  ioread32(reg + 0xEC);
  printk(KERN_INFO" the changed val = 0x%x\n", val);

  val =  ioread32(reg + 0xF8);
  printk(KERN_INFO" the initial val = 0x%x\n", val);
  val |= 0x100;
  iowrite32(val, reg+0xF8);
  val =  ioread32(reg + 0xF8);
  printk(KERN_INFO" the changed val = 0x%x\n", val);
  
  if (reg)
	  iounmap((void *)reg); 			  
  release_mem_region(pgc_base, 0x1000);   
}
#if 1
{
  //BLK_CTL
  unsigned int blk_base = BLK_CTL_BASE;
  volatile unsigned char *reg = NULL;
  
  if (!request_mem_region(blk_base, 0x1000, "blk_ctl"))
  {
	  printk(KERN_INFO "blk_ctl: failed to reserve HW regs\n");
	  return -EBUSY;
  }

  reg = (volatile u8 *) ioremap_nocache(blk_base, 0x1000);

  if (reg == NULL )
  {
	  printk(KERN_INFO "blk_ctl: failed to ioremap HW regs\n");
	  if (reg)								   
		  iounmap((void *)reg); 			  
	  release_mem_region(blk_base, 0x1000);  
	  return -EBUSY;
  }

  iowrite32(0x3, reg + 0x0); // release all soft reset
  iowrite32(0x3, reg + 0x4); // enable all clock
  iowrite32(0xFFFFFFFF, reg + 0x8); // all G1 fuse dec enable
  iowrite32(0xFFFFFFFF, reg + 0xC); // all G1 fuse pp enable
  iowrite32(0xFFFFFFFF, reg + 0x10); // all G2 fuse dec enable
  // G1 use, set to 1; G2 use, set to 0, choose the one you are using
  //iowrite32(0x1, reg + 0x14);  // VPUMIX only use G1
  //iowrite32(0x0, reg + 0x14); // VPUMIX only use G2

  if (reg)
     iounmap((void *)reg); 			  
  release_mem_region(blk_base, 0x1000);   
}
#endif
/*------------------------------------------------------------*/
#endif
  /* If base_port is set at load, use that for single Core legacy mode */
  if(base_port != -1) {
    multicorebase[0] = base_port;
    elements = 1;
    printk(KERN_INFO "hantrodec: Init single Core at 0x%16lx IRQ=%i\n",
           multicorebase[0], irq_0);
  } else {
    printk(KERN_DEBUG "hantrodec: Init multi Core[0] at 0x%16lx\n"
           "                     Core[1] at 0x%16lx\n"
           "                     Core[2] at 0x%16lx\n"
           "                     Core[3] at 0x%16lx\n"
           "          IRQ_0=%i\n"
           "          IRQ_1=%i\n",
           multicorebase[0], multicorebase[1],
           multicorebase[2], multicorebase[3],
           irq_0,irq_1);
  }

  hantrodec_data.cores = 0;
  
  hantrodec_data.iosize[0] = DEC_IO_SIZE_0;
  hantrodec_data.irq[0] = irq_0;
#ifdef MULTI_CORE
  hantrodec_data.iosize[1] = DEC_IO_SIZE_1;
  hantrodec_data.irq[1] = irq_1;
#endif

  for(i=0; i< HXDEC_MAX_CORES; i++) {
    hantrodec_data.hwregs[i] = 0;
    /* If user gave less Core bases that we have by default,
     * invalidate default bases
     */
    if(elements && i>=elements) {
      multicorebase[i] = -1;
    }
  }

  hantrodec_data.async_queue_dec = NULL;
  hantrodec_data.async_queue_pp = NULL;

  result = register_chrdev(hantrodec_major, "hantrodec", &hantrodec_fops);
  if(result < 0) {
    printk(KERN_INFO "hantrodec: unable to get major %d\n", hantrodec_major);
    goto err;
  } else if(result != 0) { /* this is for dynamic major */
    hantrodec_major = result;
  }

#ifdef CLK_CFG
  clk_cfg = clk_get(NULL, CLK_ID); /*first get clk instance pointer*/
  if (!clk_cfg||IS_ERR(clk_cfg))
  	{
     printk("get handrodec clk failed!\n");
     goto err;
  	}
  if(clk_prepare_enable(clk_cfg)) /*prepare and enable clk*/
  	{
     printk("try to enable handrodec clk failed!\n");
	 goto err;
    }
  is_clk_on =1;
  /*init a timer to disable clk*/
  init_timer(&timer);
  timer.function = &hantrodec_disable_clk;
  timer.expires =  jiffies + 100*HZ; //the expires time is 100s
  add_timer(&timer); 
#endif

  result = ReserveIO();
  if(result < 0) {
    goto err;
  }

  memset(dec_owner, 0, sizeof(dec_owner));
  memset(pp_owner, 0, sizeof(pp_owner));

  sema_init(&dec_core_sem, hantrodec_data.cores-1);
  sema_init(&pp_core_sem, 1);

  /* read configuration fo all cores */
  ReadCoreConfig(&hantrodec_data);

  /* reset hardware */
  ResetAsic(&hantrodec_data);

  /* register irq for each core*/
  if(irq_0 > 0)
    {
#ifdef VSI
		struct device_node *np;
		//memset(&irq_g2,0,sizeof(struct irq_test_type));
		np = of_find_matching_node(NULL, mmp_timer_dt_ids_0);
		if (!np) {
			result = -ENODEV;
			printk("errno 1 = %d\n", result);
			goto err;
		}
		irq_0 = irq_of_parse_and_map(np, 0);
		if (!irq_0) {
			result = -EINVAL;
			printk("errno 2 = %d\n", result);
			goto err;
		}
#endif
		//printk("irq = %d\n", irq_0);
		hantrodec_data.irq[0] = irq_0;
    	//printk(KERN_INFO "init the irq function !!\n");
        result = request_irq(irq_0, hantrodec_isr,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
                         SA_INTERRUPT | SA_SHIRQ,
#else
                         IRQF_SHARED,
#endif
                         "hantrodec", (void *) &hantrodec_data);

		if(result != 0)
        {
            if(result == -EINVAL)
            {
                printk(KERN_ERR "hantrodec: Bad irq number or handler\n");
            }
            else if(result == -EBUSY)
            {
                printk(KERN_ERR "hantrodec: IRQ <%d> busy, change your config\n",
                        hantrodec_data.irq[0]);
            }

            ReleaseIO();
            goto err;
        }
    }
    else
    {
        printk(KERN_INFO "hantrodec: IRQ not in use!\n");
    }
  
#ifdef MULTI_CORE
    if(irq_1 > 0)
    {
#ifdef VSI
		struct device_node *np;
		//memset(&irq_g2,0,sizeof(struct irq_test_type));
		np = of_find_matching_node(NULL, mmp_timer_dt_ids_1);
		if (!np) {
			result = -ENODEV;
			printk("errno 1 = %d\n", result);
			goto err;
		}
		irq_1 = irq_of_parse_and_map(np, 0);
		if (!irq_1) {
			result = -EINVAL;
			printk("errno 2 = %d\n", result);
			goto err;
		}
#endif
		//printk("irq = %d\n", irq_1);
		hantrodec_data.irq[1] = irq_1;
    	//printk(KERN_INFO "init the irq function !!\n");
        result = request_irq(irq_1, hantrodec_isr,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
                         SA_INTERRUPT | SA_SHIRQ,
#else
                         IRQF_SHARED,
#endif
                         "hantrodec", (void *) &hantrodec_data);

		if(result != 0)
        {
            if(result == -EINVAL)
            {
                printk(KERN_ERR "hantrodec: Bad irq number or handler\n");
            }
            else if(result == -EBUSY)
            {
                printk(KERN_ERR "hantrodec: IRQ <%d> busy, change your config\n",
                        hantrodec_data.irq[1]);
            }

            ReleaseIO();
            goto err;
        }
    }
    else
    {
        printk(KERN_INFO "hantrodec: IRQ not in use!\n");
    }
#endif

  printk(KERN_INFO "hantrodec: module inserted. Major = %d\n", hantrodec_major);

  return 0;

err:
  printk(KERN_INFO "hantrodec: module not inserted\n");
  unregister_chrdev(hantrodec_major, "hantrodec");
  return result;
}

/*------------------------------------------------------------------------------
 Function name   : hantrodec_cleanup
 Description     : clean up

 Return type     : int
------------------------------------------------------------------------------*/

void hantrodec_cleanup(void) {
  hantrodec_t *dev = &hantrodec_data;
  int n =0;
  /* reset hardware */
  ResetAsic(dev);

  /* free the IRQ */
  for (n = 0; n < dev->cores; n++)
  	{
	  if(dev->irq[n] != -1) {
	    free_irq(dev->irq[n], (void *) dev);
	  }
  	}

  ReleaseIO();

#ifdef CLK_CFG
  if (clk_cfg!=NULL && !IS_ERR(clk_cfg))
  {
   clk_disable_unprepare(clk_cfg);
   is_clk_on = 0;
   printk("turned off hantrodec clk\n");
  }

  /*delete timer*/
  del_timer(&timer);
#endif

  unregister_chrdev(hantrodec_major, "hantrodec");

  printk(KERN_INFO "hantrodec: module removed\n");
  return;
}

/*------------------------------------------------------------------------------
 Function name   : CheckHwId
 Return type     : int
------------------------------------------------------------------------------*/
static int CheckHwId(hantrodec_t * dev) {
  long int hwid;
  int i;
  size_t num_hw = sizeof(DecHwId) / sizeof(*DecHwId);

  int found = 0;
  
  for (i = 0; i < dev->cores; i++) {
    if (dev->hwregs[i] != NULL ) {
      hwid = readl(dev->hwregs[i]);
      printk(KERN_DEBUG "hantrodec: Core %d HW ID=0x%16lx\n", i, hwid);
      hwid = (hwid >> 16) & 0xFFFF; /* product version only */

      while (num_hw--) {
        if (hwid == DecHwId[num_hw]) {
          printk(KERN_DEBUG "hantrodec: Supported HW found at 0x%16lx\n",
                 multicorebase[i]);
          found++;
		  dev->hw_id[i] = hwid;
          break;
        }
      }
      if (!found) {
        printk(KERN_ERR "hantrodec: Unknown HW found at 0x%16lx\n",
               multicorebase[i]);
        return 0;
      }
      found = 0;
      num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
    }
  }

  return 1;
}

/*------------------------------------------------------------------------------
 Function name   : ReserveIO
 Description     : IO reserve

 Return type     : int
------------------------------------------------------------------------------*/
static int ReserveIO(void) {
  int i;

  for (i = 0; i < HXDEC_MAX_CORES; i++) {
    if (multicorebase[i] != -1) {
      if (!request_mem_region(multicorebase[i], hantrodec_data.iosize[i],
                              "hantrodec0")) {
        printk("hantrodec: failed to reserve HW regs\n");
        return -EBUSY;
      }

      hantrodec_data.hwregs[i] = (volatile u8 *) ioremap_nocache(multicorebase[i],
                                 hantrodec_data.iosize[i]);

      if (hantrodec_data.hwregs[i] == NULL ) {
        printk("hantrodec: failed to ioremap HW regs\n");
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

/*------------------------------------------------------------------------------
 Function name   : releaseIO
 Description     : release

 Return type     : void
------------------------------------------------------------------------------*/

static void ReleaseIO(void) {
  int i;
  for (i = 0; i < hantrodec_data.cores; i++) {
    if (hantrodec_data.hwregs[i])
      iounmap((void *) hantrodec_data.hwregs[i]);
    release_mem_region(multicorebase[i], hantrodec_data.iosize[i]);
  }
}

/*------------------------------------------------------------------------------
 Function name   : hantrodec_isr
 Description     : interrupt handler

 Return type     : irqreturn_t
------------------------------------------------------------------------------*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
irqreturn_t hantrodec_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
irqreturn_t hantrodec_isr(int irq, void *dev_id)
#endif
{
  unsigned long flags;
  unsigned int handled = 0;
  int i;
  volatile u8 *hwregs;

  hantrodec_t *dev = (hantrodec_t *) dev_id;
  u32 irq_status_dec;
  
  spin_lock_irqsave(&owner_lock, flags);

  for(i=0; i<dev->cores; i++) {
    volatile u8 *hwregs = dev->hwregs[i];

    /* interrupt status register read */
    irq_status_dec = ioread32(hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

    if(irq_status_dec & HANTRODEC_DEC_IRQ) {
      /* clear dec IRQ */
      irq_status_dec &= (~HANTRODEC_DEC_IRQ);
      iowrite32(irq_status_dec, hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

      PDEBUG("decoder IRQ received! Core %d\n", i);

      atomic_inc(&irq_rx);

      dec_irq |= (1 << i);

      wake_up_interruptible_all(&dec_wait_queue);
      handled++;
    }
  }

  spin_unlock_irqrestore(&owner_lock, flags);

  if(!handled) {
    PDEBUG("IRQ received, but not hantrodec's!\n");
  }

  (void)hwregs;
  return IRQ_RETVAL(handled);
}

/*------------------------------------------------------------------------------
 Function name   : ResetAsic
 Description     : reset asic

 Return type     :
------------------------------------------------------------------------------*/
void ResetAsic(hantrodec_t * dev) {
  int i, j;
  u32 status;

  for (j = 0; j < dev->cores; j++) {
    status = ioread32(dev->hwregs[j] + HANTRODEC_IRQ_STAT_DEC_OFF);
    
    if( status & HANTRODEC_DEC_E) {
      /* abort with IRQ disabled */
      status = HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
      iowrite32(status, dev->hwregs[j] + HANTRODEC_IRQ_STAT_DEC_OFF);
    }
	
	if (IS_G1(dev->hw_id[j]))
	  /* reset PP */
      iowrite32(0, dev->hwregs[j] + HANTRO_IRQ_STAT_PP_OFF);
    
    for (i = 4; i < dev->iosize[j]; i += 4) {
      iowrite32(0, dev->hwregs[j] + i);
    }
  }
}

/*------------------------------------------------------------------------------
 Function name   : dump_regs
 Description     : Dump registers

 Return type     :
------------------------------------------------------------------------------*/
#ifdef HANTRODEC_DEBUG
void dump_regs(hantrodec_t *dev) {
  int i,c;

  PDEBUG("Reg Dump Start\n");
  for(c = 0; c < dev->cores; c++) {
    for(i = 0; i < dev->iosize[c]; i += 4*4) {
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

#ifdef VSI
module_init( hantrodec_init);
module_exit( hantrodec_cleanup);
#else
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
    clk_disable(hantro_clk_g1);
    clk_unprepare(hantro_clk_g1);
    clk_disable(hantro_clk_g2);
    clk_unprepare(hantro_clk_g2);
    clk_disable(hantro_clk_bus);
    clk_unprepare(hantro_clk_bus);
    return 0;
}

static int hantro_dev_probe(struct platform_device *pdev)
{
    int err = 0;
    struct device *temp_class;
    struct resource *res;
    unsigned long reg_base;
    volatile u8* iobase;

    hantro_dev=&pdev->dev;
    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs_hantro");
    if (!res) {
        printk(KERN_ERR "hantro: unable to get vpu base addr\n");
        return -ENODEV;
    }
    reg_base = res->start;
    if((ulong)reg_base!=multicorebase[0]){
        printk(KERN_ERR "hantrodec: regbase(0x%lX) not equal to expected value(0x%lX)\n",reg_base,multicorebase[0]);
        err=-ENODEV;
        goto error;
    }

    irq_1 = platform_get_irq_byname(pdev, "irq_hantro_g2");
    irq_0 = platform_get_irq_byname(pdev, "irq_hantro_g1");
    if ((irq_0 < 0) ||(irq_1 < 0)) {
        printk(KERN_ERR "hantro: unable to get hantro g1/g2 interrupt\n");
        err = -ENXIO;
        goto error;
    }
    printk(KERN_DEBUG "base port: 0x%lX , g1 irq: %d, g2 irq: %d \n",reg_base,irq_0,irq_1);

    hantro_clk_g1 = clk_get(&pdev->dev, "clk_hantro_g1");
    hantro_clk_g2 = clk_get(&pdev->dev, "clk_hantro_g2");
    hantro_clk_bus = clk_get(&pdev->dev, "clk_hantro_bus");
    if (IS_ERR(hantro_clk_g1)||IS_ERR(hantro_clk_g2)||IS_ERR(hantro_clk_bus)){
        printk(KERN_ERR "hantro: get clock failed\n");
        err = -ENXIO;
        goto error;
    }
    printk(KERN_DEBUG "hantro: g1, g2, bus clock: 0x%lX, 0x%lX, 0x%lX \n",clk_get_rate(hantro_clk_g1),
		clk_get_rate(hantro_clk_g2),clk_get_rate(hantro_clk_bus));
	
    pm_runtime_enable(hantro_dev);
    pm_runtime_get_sync(hantro_dev);
    hantro_clk_enable(&pdev->dev);

    //config G1/G2
    iobase = (volatile u8 *) ioremap_nocache(BLK_CTL_BASE,0x10000);
    iowrite32(0x3,iobase);  //VPUMIX G1/G2 block soft reset  control
    iowrite32(0x3,iobase+4); //VPUMIX G1/G2 block clock enable control
    iowrite32(0xFFFFFFFF, iobase + 0x8); // all G1 fuse dec enable
    iowrite32(0xFFFFFFFF, iobase + 0xC); // all G1 fuse pp enable
    iowrite32(0xFFFFFFFF, iobase + 0x10); // all G2 fuse dec enable
    iounmap(iobase);  

	err=hantrodec_init();
	if(0!=err){
		printk(KERN_ERR "hantro: hantrodec_init failed\n");
		goto error;
	}

	hantro_class = class_create(THIS_MODULE, "mxc_hantro");
	if (IS_ERR(hantro_class)) {
		err = PTR_ERR(hantro_class);
		goto error;
	}
  temp_class = device_create(hantro_class, NULL, MKDEV(hantrodec_major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	goto out;

err_out_class:
	device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
	class_destroy(hantro_class);
error:
	printk(KERN_ERR "hantro probe failed\n");
out:
	return err;
}

static int hantro_dev_remove(struct platform_device *pdev)
{
	if (hantrodec_major > 0) {
		device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
		class_destroy(hantro_class);
		hantrodec_cleanup();
		hantrodec_major = 0;
	}
	hantro_clk_disable(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
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
    pm_runtime_get_sync(dev);     //power on
    return 0;
}
static int hantro_runtime_suspend(struct device *dev)
{
    //release_bus_freq(BUS_FREQ_HIGH);
    return 0;
}

static int hantro_runtime_resume(struct device *dev)
{
    //request_bus_freq(BUS_FREQ_HIGH);
    return 0;
}

static const struct dev_pm_ops hantro_pm_ops = {
    SET_RUNTIME_PM_OPS(hantro_runtime_suspend, hantro_runtime_resume, NULL)
    SET_SYSTEM_SLEEP_PM_OPS(hantro_suspend, hantro_resume)
};
#endif

static const struct of_device_id hantro_of_match[] = {
  { .compatible = "nxp,imx8mq-hantro", },
  {/* sentinel */}
};
MODULE_DEVICE_TABLE(of, vpu_of_match);


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
    //clk_put(hantro_clk);
    platform_driver_unregister(&mxchantro_driver);
    return;
}

module_init( hantro_init);
module_exit( hantro_exit);

#endif

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Driver module for Hantro Decoder/Post-Processor");

