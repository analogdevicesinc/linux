/*
 * Copyright 2017 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_vpu-malone.c
 *
 * @brief VPU system initialization and file operation implementation
 *
 * @ingroup VPU
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/regulator/consumer.h>
#include <linux/page-flags.h>
#include <linux/mm_types.h>
#include <linux/types.h>
#include <linux/memblock.h>
#include <linux/memory.h>
#include <linux/version.h>
#include <asm/page.h>

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>

#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/genalloc.h>
#include <linux/mxc_vpu-malone.h>
#include <linux/of.h>
#include <linux/reset.h>

#include <soc/imx8/sc/svc/pm/api.h>
#include <soc/imx8/sc/ipc.h>

#include "VPU_regdef.h"
#include "pal.h"
#include "mvd.h"
#include "DecKernelLib.h"

struct vpu_priv {
	struct fasync_struct *async_queue;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
};

/* To track the allocated memory buffer */
struct memalloc_record {
	struct list_head list;
	struct vpu_mem_desc mem;
};

static LIST_HEAD(head);

static int vpu_major;
static struct class *vpu_class;
static struct vpu_priv vpu_data;
static u8 open_count;
static struct clk *vpu_clk;
static struct vpu_mem_desc share_mem = { 0 };
static struct vpu_mem_desc vshare_mem = { 0 };

void __iomem *vpu_base;
static u32 phy_vpu_base_addr;
static int vpu_dec_irq;
static int vpu_dec_fiq;

static struct device *vpu_dev;

/* implement the blocking ioctl */
static int irq_status;
static wait_queue_head_t vpu_queue;

static int vpu_clk_usercount;
static atomic_t clk_cnt_from_ioc = ATOMIC_INIT(0);

static sc_ipc_t ipcHndl;

#define ISR_EVENT_QU_SIZE 64

static PAL_QUEUE_ID gIsrEventQu;
static DECODER_KERNEL_LIB_ISR_EVENT_DATA gIsrEventData[ISR_EVENT_QU_SIZE];
static u32 uIsrEventQuWrIdx = 0;
static DECODERLIB_KERNEL_CFG gtKernelCfg;

#define	READ_REG(x)		readl_relaxed(vpu_base + x)
#define	WRITE_REG(val, x)	writel_relaxed(val, vpu_base + x)

static void vpu_reset(void)
{
#if 0
	int ret;

	ret = device_reset(vpu_dev);
#endif
}

/*!
 * Private function to change the power mode of VPU to pm
 * @return status  0 success.
 */
static int setVPUPwr(sc_ipc_t ipcHndl,
                     sc_pm_power_mode_t pm
                     )
{
	int rv = -1;
	sc_err_t sciErr;

	dev_dbg(vpu_dev, "enter %s()\n", __FUNCTION__);
	if (!ipcHndl)
	{
		dev_err(vpu_dev, "--- setVPUPwr no IPC handle\n");
		goto setVPUPwrExit;
	}

	/* Power on or off PID0, DEC, ENC */
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID0, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID0,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	/* FIXME: no need of PID1-7? */
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID1, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID1,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl,SC_R_VPU_PID2, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID2,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID3, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID3,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID4, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID4,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID5, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID5,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID6, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID6,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID7, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID7,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_DEC_0, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_DEC,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_ENC_0, pm);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_set_resource_power_mode(SC_R_VPU_ENC,%d) SCI error! (%d)\n", pm, sciErr);
		goto setVPUPwrExit;
	}

	rv = 0;

setVPUPwrExit:
	return (rv);
}

#if 0
/*!
 * Private function to return the power mode of VPU
 * @return <0 if there were errors.
 */
static sc_pm_power_mode_t getVPUPwr(sc_ipc_t ipcHndl)
{
	sc_pm_power_mode_t rv = -1;
	sc_err_t sciErr;

	if (!ipcHndl)
	{
		dev_err(vpu_dev, "--- getVPUPwr no IPC handle\n");
		return (rv);
	}

	sciErr = sc_pm_get_resource_power_mode(ipcHndl, SC_R_VPU_PID0, &rv);
	if (sciErr != SC_ERR_NONE)
	{
		dev_err(vpu_dev, "--- sc_pm_get_resource_power_mode(SC_R_VPU_PID0) SCI error! (%d)\n", sciErr);
		rv = -1;
	}

	return (rv);
}
#endif

static long vpu_power_get(bool on)
{
	return 0;
}

static void vpu_power_up(bool on)
{
	int err;

	if (on) {
		err = setVPUPwr(ipcHndl, SC_PM_PW_MODE_ON);
		if (err)
			dev_err(vpu_dev, "failed to power on\n");
		pm_runtime_get_sync(vpu_dev);
		/* TODO: before or after clk enable? */
		clk_set_rate(vpu_clk, 600000000);
	} else {
		pm_runtime_put_sync_suspend(vpu_dev);
		err = setVPUPwr(ipcHndl, SC_PM_PW_MODE_OFF);
		if (err)
			dev_err(vpu_dev, "failed to power off\n");
	}
}

#define VM_RESERVED 0

/*!
 * Private function to alloc dma buffer
 * @return status  0 success.
 */
static int vpu_alloc_dma_buffer(struct vpu_mem_desc *mem)
{
	mem->cpu_addr = dma_alloc_coherent(vpu_dev, PAGE_ALIGN(mem->size),
			       &mem->phy_addr,
			       GFP_DMA | GFP_KERNEL);
	dev_dbg(vpu_dev, "[ALLOC] mem alloc cpu_addr = 0x%p\n", mem->cpu_addr);
	if (mem->cpu_addr == NULL) {
		dev_err(vpu_dev, "Physical memory allocation error!\n");
		return -1;
	}
	return 0;
}

/*!
 * Private function to free dma buffer
 */
static void vpu_free_dma_buffer(struct vpu_mem_desc *mem)
{
	if (mem->cpu_addr != NULL) {
		dma_free_coherent(vpu_dev, PAGE_ALIGN(mem->size),
				  mem->cpu_addr, mem->phy_addr);
	}
}

/*!
 * Private function to free buffers
 * @return status  0 success.
 */
static int vpu_free_buffers(void)
{
	struct memalloc_record *rec, *n;
	struct vpu_mem_desc mem;

	list_for_each_entry_safe(rec, n, &head, list) {
		mem = rec->mem;
		if (mem.cpu_addr != NULL) {
			vpu_free_dma_buffer(&mem);
			dev_dbg(vpu_dev, "[FREE] freed paddr=0x%08llX\n", mem.phy_addr);
			/* delete from list */
			list_del(&rec->list);
			kfree(rec);
		}
	}

	return 0;
}

static inline void vpu_worker_callback(struct work_struct *w)
{
	struct vpu_priv *dev = container_of(w, struct vpu_priv,
				work);

	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	irq_status = 1;

	wake_up_interruptible(&vpu_queue);
}

#if 0
/*!
 * @brief vpu interrupt handler
 */
static irqreturn_t vpu_ipi_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *dev = dev_id;

	queue_work(dev->workqueue, &dev->work);

	return IRQ_HANDLED;
}
#endif

/*!
 * @brief vpu fiq handler
 */
static irqreturn_t vpu_dec_fiq_handler(int irq, void *dev_id)
{
	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

  /* NXP using this bit in pal_int_enable() which is FORCE_EXIT, not hanlded within the ISR routing itself */
	WRITE_REG(0x100, DEC_MFD_XREG_SLV_BASE + MFD_SIF + MFD_SIF_INTR_STATUS);

	int_handlers[INT_ID_MALONE_LOW](uMvdKernelIrqPin[0][0]);

	return IRQ_HANDLED;
}

/*!
 * @brief vpu irq handler
 */
static irqreturn_t vpu_dec_irq_handler(int irq, void *dev_id)
{
	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	int_handlers[INT_ID_MALONE_HI](uMvdKernelIrqPin[0][1]);

	return IRQ_HANDLED;
}

static int vpu_isr_event_qu_init(void)
{
	int ret;

	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	ret = pal_qu_create(ISR_EVENT_QU_SIZE,
			NULL,
			&gIsrEventQu);

	return ret;
}

static int vpu_isr_event_qu_send(DECODER_KERNEL_LIB_ISR_EVENT_DATA *pIsrEventData)
{
	int ret;
	uint_addr ulMsg[4];

	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	mutex_lock(&vpu_data.lock);

	/* Need to take care of copying over the data */
	gIsrEventData[uIsrEventQuWrIdx] = *pIsrEventData;

	/* Fill message */
	ulMsg[0] = (uint_addr)&gIsrEventData[uIsrEventQuWrIdx];
	ulMsg[1] = 0;
	ulMsg[2] = 0;
	ulMsg[3] = 0;

	uIsrEventQuWrIdx++;
	if (uIsrEventQuWrIdx == ISR_EVENT_QU_SIZE) uIsrEventQuWrIdx = 0;

	ret = pal_qu_send (gIsrEventQu,
			ulMsg);

	mutex_unlock(&vpu_data.lock);

	dev_dbg(vpu_dev, "leave %s\n", __FUNCTION__);
	return ret;
}

static int vpu_isr_event_qu_receive(DECODER_KERNEL_LIB_ISR_EVENT_DATA *pIsrEventData)
{
	int ret;
	uint_addr uMsg[4];

	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	ret = pal_qu_receive(gIsrEventQu,
			1000,
			uMsg);

	if (ret == MEDIAIP_FW_STATUS_OK)
		*pIsrEventData = *(DECODER_KERNEL_LIB_ISR_EVENT_DATA *)uMsg[0];

	dev_dbg(vpu_dev, "leave %s\n", __FUNCTION__);
	return ret;
}

static int vpu_isr_event_qu_uninit(void)
{
	int ret;

	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	ret = pal_qu_destroy(gIsrEventQu);

	return ret;
}

static void vpu_isr_event_callback(DECODER_KERNEL_LIB_ISR_EVENT_DATA *ptEventData)
{
	int err;

	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	err = vpu_isr_event_qu_send(ptEventData); 
	if (err)
		dev_err(vpu_dev, "failed to send isr event\n");

	dev_dbg(vpu_dev, "leave %s\n", __FUNCTION__);
}

static void vpu_set_kernel_cfg(pDECODERLIB_KERNEL_CFG ptKernelCfg)
{
	ptKernelCfg->uNumMalones = 1;

	ptKernelCfg->uMaloneBaseAddr[0]   = (uint_addr)(vpu_base + 0x180000);
	ptKernelCfg->uMaloneHifOffset[0]  = 0x1C000;

	/* Pass Decoder a PAL index, not an actual GIC position */
	/* The PAL will take care of the rest                   */
	ptKernelCfg->uMaloneIrqPin[0][0x0] = PAL_IRQ_MALONE0_LOW;
	ptKernelCfg->uMaloneIrqPin[0][0x1] = PAL_IRQ_MALONE0_HI;

	ptKernelCfg->uDPVBaseAddr   = 0;
	ptKernelCfg->uDPVIrqPin     = 0;
}

static void vpu_prepare(void)
{
	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	WRITE_REG(0x1, SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	WRITE_REG(0xffffffff, SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_XMEM_RESET_SET);

	WRITE_REG(0xE, SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	WRITE_REG(0x7, SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_SET);

	WRITE_REG(0x1f, DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET);
	WRITE_REG(0xffffffff, DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_SET);

	WRITE_REG(0x102, XMEM_CONTROL);
}

static void vpu_unprepare(void)
{
	dev_dbg(vpu_dev, "enter %s\n", __FUNCTION__);

	WRITE_REG(0x7, SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_CLR);
	WRITE_REG(0xffffffff, DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_CLR);
}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_open(struct inode *inode, struct file *filp)
{
	u32 mu_id;
	int err;

	mutex_lock(&vpu_data.lock);

	if (open_count++ == 0) {
		err = sc_ipc_getMuID(&mu_id);;
		if(err != SC_ERR_NONE)
		{
			dev_err(vpu_dev, "--- sc_ipc_getMuID() cannot obtain mu id SCI error! (%d)\n", err);
			return -EFAULT;
		}

		err = sc_ipc_open(&ipcHndl, mu_id);
		if(err != SC_ERR_NONE)
		{
			dev_err(vpu_dev, "--- sc_ipc_getMuID() cannot open MU channel to SCU error! (%d)\n", err);
			return -EFAULT;
		}
		vpu_power_up(true);
		clk_prepare_enable(vpu_clk);
		vpu_prepare();
		err = vpu_isr_event_qu_init();
		if(err)
		{
			dev_err(vpu_dev, "failed to init isr event qu\n");
			return -EFAULT;
		}
		vpu_set_kernel_cfg(&gtKernelCfg);
		decoder_kernel_lib_init(&gtKernelCfg);
		decoder_kernel_lib_register_isr_callback(0, &vpu_isr_event_callback);

#if 0
		WRITE_REG(0x1000, DEC_MFD_XREG_SLV_BASE + MFD_HIF + 0x014);
		WRITE_REG(0x20, DEC_MFD_XREG_SLV_BASE + MFD_HIF + 0x01C);
		WRITE_REG(0x1000, DEC_MFD_XREG_SLV_BASE + MFD_SIF + MFD_SIF_CTRL_STATUS);
		pal_int_set(PAL_IRQ_MALONE0_LOW);
#endif
	} else {
		dev_err(vpu_dev, "open more than once is forbidden now\n");
	}

	filp->private_data = (void *)(&vpu_data);

	mutex_unlock(&vpu_data.lock);

	return 0;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */

#ifdef VPU_KERNEL_DBG_ISR
int guDbgIsrArray[1024][3] ;
int guDbgIsrArrayWrtIdx = 0 ;
#endif

static long vpu_ioctl(struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = 0;

	switch (cmd) {
	case VPU_IOC_PHYMEM_ALLOC:
		{
			struct memalloc_record *rec;

			rec = kzalloc(sizeof(*rec), GFP_KERNEL);
			if (!rec)
				return -ENOMEM;

			ret = copy_from_user(&(rec->mem),
					     (struct vpu_mem_desc *)arg,
					     sizeof(struct vpu_mem_desc));
			if (ret) {
				kfree(rec);
				return -EFAULT;
			}

			dev_dbg(vpu_dev, "[ALLOC] mem alloc size = 0x%x\n",
				 rec->mem.size);

			ret = vpu_alloc_dma_buffer(&(rec->mem));
			if (ret == -1) {
				kfree(rec);
				dev_err(vpu_dev,
					"Physical memory allocation error!\n");
				break;
			}
			ret = copy_to_user((void __user *)arg, &(rec->mem),
					   sizeof(struct vpu_mem_desc));
			if (ret) {
				kfree(rec);
				ret = -EFAULT;
				break;
			}

			mutex_lock(&vpu_data.lock);
			list_add(&rec->list, &head);
			mutex_unlock(&vpu_data.lock);

			break;
		}
	case VPU_IOC_PHYMEM_FREE:
		{
			struct memalloc_record *rec, *n;
			struct vpu_mem_desc vpu_mem;

			ret = copy_from_user(&vpu_mem,
					     (struct vpu_mem_desc *)arg,
					     sizeof(struct vpu_mem_desc));
			if (ret)
				return -EACCES;

			dev_dbg(vpu_dev, "[FREE] mem freed cpu_addr = 0x%p\n",
				 vpu_mem.cpu_addr);
			if (vpu_mem.cpu_addr != NULL)
				vpu_free_dma_buffer(&vpu_mem);

			mutex_lock(&vpu_data.lock);
			list_for_each_entry_safe(rec, n, &head, list) {
				if (rec->mem.cpu_addr == vpu_mem.cpu_addr) {
					/* delete from list */
					list_del(&rec->list);
					kfree(rec);
					break;
				}
			}
			mutex_unlock(&vpu_data.lock);

			break;
		}
	case VPU_IOC_WAIT4INT:
		{
			DECODER_KERNEL_LIB_ISR_EVENT_DATA IsrEventData;

			ret = vpu_isr_event_qu_receive(&IsrEventData);
			if (ret) 
            {
				dev_info(vpu_dev, "no isr event\n");
				ret = -EFAULT;
				break;
			}
#ifdef VPU_KERNEL_DBG_ISR
      else
      {
        dev_dbg(vpu_dev, "isr events= %x %x %x\n", IsrEventData.uIrqStatus[0],IsrEventData.uIrqStatus[1], IsrEventData.uIrqStatus[2]);
      }

      guDbgIsrArray[guDbgIsrArrayWrtIdx][0] = IsrEventData.uIrqStatus[0];
      guDbgIsrArray[guDbgIsrArrayWrtIdx][1] = IsrEventData.uIrqStatus[1];
      guDbgIsrArray[guDbgIsrArrayWrtIdx][2] = IsrEventData.uIrqStatus[2];      
      guDbgIsrArrayWrtIdx++;
      guDbgIsrArrayWrtIdx = (guDbgIsrArrayWrtIdx==1024) ? 0 : guDbgIsrArrayWrtIdx;
#endif
        
			ret = copy_to_user((void __user *)arg, &IsrEventData, sizeof(DECODER_KERNEL_LIB_ISR_EVENT_DATA));
      
			if (ret)
				ret = -EFAULT;
#if 0
			u_long timeout = (u_long) arg;
			if (!wait_event_interruptible_timeout
			    (vpu_queue, irq_status != 0,
			     msecs_to_jiffies(timeout))) {
				dev_warn(vpu_dev, "VPU blocking: timeout.\n");
				ret = -ETIME;
			} else if (signal_pending(current)) {
				dev_warn(vpu_dev, "Other interrupt received.\n");
				ret = -ERESTARTSYS;
			} else
				irq_status = 0;
#endif
			break;
		}
	case VPU_IOC_CLKGATE_SETTING:
		{
			u32 clkgate_en;

			if (get_user(clkgate_en, (u32 __user *) arg))
				return -EFAULT;

			if (clkgate_en) {
				clk_prepare_enable(vpu_clk);
				atomic_inc(&clk_cnt_from_ioc);
			} else {
				clk_disable_unprepare(vpu_clk);
				atomic_dec(&clk_cnt_from_ioc);
			}

			break;
		}
	case VPU_IOC_GET_SHARE_MEM:
		{
			mutex_lock(&vpu_data.lock);
			if (share_mem.cpu_addr != NULL) {
				ret = copy_to_user((void __user *)arg,
						   &share_mem,
						   sizeof(struct vpu_mem_desc));
				mutex_unlock(&vpu_data.lock);
				break;
			} else {
				if (copy_from_user(&share_mem,
						   (struct vpu_mem_desc *)arg,
						 sizeof(struct vpu_mem_desc))) {
					mutex_unlock(&vpu_data.lock);
					return -EFAULT;
				}
				if (vpu_alloc_dma_buffer(&share_mem) == -1)
					ret = -EFAULT;
				else {
					if (copy_to_user((void __user *)arg,
							 &share_mem,
							 sizeof(struct
								vpu_mem_desc)))
						ret = -EFAULT;
				}
			}
			mutex_unlock(&vpu_data.lock);
			break;
		}
	case VPU_IOC_REQ_VSHARE_MEM:
		{
			mutex_lock(&vpu_data.lock);
			if (vshare_mem.cpu_addr != NULL) {
				ret = copy_to_user((void __user *)arg,
						   &vshare_mem,
						   sizeof(struct vpu_mem_desc));
				mutex_unlock(&vpu_data.lock);
				break;
			} else {
				if (copy_from_user(&vshare_mem,
						   (struct vpu_mem_desc *)arg,
						   sizeof(struct
							  vpu_mem_desc))) {
					mutex_unlock(&vpu_data.lock);
					return -EFAULT;
				}
				/* vmalloc shared memory if not allocated */
				if (!vshare_mem.cpu_addr)
					vshare_mem.cpu_addr =
					    vmalloc_user(vshare_mem.size);
				if (copy_to_user
				     ((void __user *)arg, &vshare_mem,
				     sizeof(struct vpu_mem_desc)))
					ret = -EFAULT;
			}
			mutex_unlock(&vpu_data.lock);
			break;
		}
	case VPU_IOC_SYS_SW_RESET:
		{
			vpu_reset();
			break;
		}
	case VPU_IOC_LOCK_DEV:
		{
			u32 lock_en;

			if (get_user(lock_en, (u32 __user *) arg))
				return -EFAULT;

			if (lock_en)
				mutex_lock(&vpu_data.lock);
			else
				mutex_unlock(&vpu_data.lock);

			break;
		}
	default:
		{
			dev_err(vpu_dev, "No such IOCTL, cmd is %d\n", cmd);
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}

/*!
 * @brief Release function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_release(struct inode *inode, struct file *filp)
{
	int i;
	unsigned long timeout;
	int err;

	mutex_lock(&vpu_data.lock);

	if (open_count > 0 && !(--open_count)) {

		/* Wait for vpu go to idle state */
		clk_prepare_enable(vpu_clk);
		timeout = jiffies + HZ;
		/* checking busy */
		while (0) {
			msleep(1);
			if (time_after(jiffies, timeout)) {
				dev_warn(vpu_dev, "VPU timeout during release\n");
				break;
			}
		}

		/* Clean up interrupt */
#if 0
		cancel_work_sync(&vpu_data.work);
		flush_workqueue(vpu_data.workqueue);
		irq_status = 0;
#endif

		/* reset if busy */
		if (0) {
			vpu_reset();
		}
		clk_disable_unprepare(vpu_clk);

		vpu_free_buffers();

		/* Free shared memory when vpu device is idle */
		vpu_free_dma_buffer(&share_mem);
		share_mem.cpu_addr = NULL;
		vfree(vshare_mem.cpu_addr);
		vshare_mem.cpu_addr = NULL;

		vpu_clk_usercount = atomic_read(&clk_cnt_from_ioc);
		for (i = 0; i < vpu_clk_usercount; i++) {
			clk_disable_unprepare(vpu_clk);
			atomic_dec(&clk_cnt_from_ioc);
		}

		err = vpu_isr_event_qu_uninit();
		if (err)
			dev_err(vpu_dev, "failed to uninit isr event qu\n");
		vpu_unprepare();
		clk_disable_unprepare(vpu_clk);
		vpu_power_up(false);
		sc_ipc_close(ipcHndl);
	}
	mutex_unlock(&vpu_data.lock);

	return 0;
}

/*!
 * @brief fasync function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_priv *dev = (struct vpu_priv *)filp->private_data;
	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

/*!
 * @brief memory map function of harware registers for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_hwregs(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	/*
	 * Since vpu registers have been mapped with ioremap() at probe
	 * which L_PTE_XN is 1, and the same physical address must be
	 * mapped multiple times with same type, so set L_PTE_XN to 1 here.
	 * Otherwise, there may be unexpected result in video codec.
	 */
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = phy_vpu_base_addr >> PAGE_SHIFT;
	dev_dbg(vpu_dev, "size=0x%x, page no.=0x%x\n",
		 (int)(vm->vm_end - vm->vm_start), (int)pfn);
	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map function of memory for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_dma_mem(struct file *fp, struct vm_area_struct *vm)
{
	int request_size;
	request_size = vm->vm_end - vm->vm_start;

	dev_dbg(vpu_dev, "start=0x%x, pgoff=0x%x, size=0x%x\n",
		 (unsigned int)(vm->vm_start), (unsigned int)(vm->vm_pgoff),
		 request_size);

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       request_size, vm->vm_page_prot) ? -EAGAIN : 0;

}

/* !
 * @brief memory map function of vmalloced share memory
 * @return  0 on success or negative error code on error
 */
static int vpu_map_vshare_mem(struct file *fp, struct vm_area_struct *vm)
{
	int ret = -EINVAL;

	ret = remap_vmalloc_range(vm, (void *)(vm->vm_pgoff << PAGE_SHIFT), 0);
	vm->vm_flags |= VM_IO;

	return ret;
}
/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	u64 offset;

	offset = (u64)((s64)vshare_mem.cpu_addr >> PAGE_SHIFT);
    if (vm->vm_pgoff && (vm->vm_pgoff == offset || vm->vm_pgoff == (offset & 0xFFFFFFFF))){
        if(vm->vm_pgoff == (offset & 0xFFFFFFFF))
            vm->vm_pgoff = offset;
		return vpu_map_vshare_mem(fp, vm);
	}else if (vm->vm_pgoff)
		return vpu_map_dma_mem(fp, vm);
	else
		return vpu_map_hwregs(fp, vm);
}

#ifdef CONFIG_COMPAT
struct core_desc_32 {
	__u32 id; /* id of the Core */
	compat_caddr_t regs; /* pointer to user registers */
	__u32 size; /* size of register space */
};
struct core_desc {
  __u32 id; /* id of the Core */
  __u32 *regs; /* pointer to user registers */
  __u32 size; /* size of register space */
};
static int get_vpu_core_desc32(struct core_desc *kp, struct core_desc_32 __user *up)
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

static int put_vpu_core_desc32(struct core_desc *kp, struct core_desc_32 __user *up)
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
static long vpu_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
#define VPU_IOCTL32(err, filp, cmd, arg) { \
		mm_segment_t old_fs = get_fs(); \
		set_fs(KERNEL_DS); \
		err = vpu_ioctl(filp, cmd, arg); \
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
	case _IOC_NR(VPU_IOC_CLKGATE_SETTING):
	case _IOC_NR(VPU_IOC_LOCK_DEV):
		err = get_user(karg.kui, (s32 __user *)up);
		if (err)
			return err;
		VPU_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_user(((s32)karg.kui), (s32 __user *)up);
		break;
	case _IOC_NR(VPU_IOC_PHYMEM_ALLOC):
	case _IOC_NR(VPU_IOC_PHYMEM_FREE):
	case _IOC_NR(VPU_IOC_WAIT4INT):
	case _IOC_NR(VPU_IOC_GET_SHARE_MEM):
	case _IOC_NR(VPU_IOC_REQ_VSHARE_MEM):
		err = get_vpu_core_desc32(&karg.kcore, up);
		if (err)
			return err;
		VPU_IOCTL32(err, filp, cmd, (unsigned long)&karg);
		err = put_vpu_core_desc32(&karg.kcore, up);
		break;
	default:
		err = vpu_ioctl(filp, cmd, (unsigned long)up);
		break;
	}

	return err;
}

#endif //ifdef CONFIG_COMPAT

const struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.unlocked_ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vpu_ioctl32,
#endif
};

/*!
 * This function is called by the driver framework to initialize the vpu device.
 * @param   dev The device structure for the vpu passed in by the framework.
 * @return   0 on success or negative error code on error
 */
static int vpu_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *temp_class;
	struct resource *res;

	vpu_dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vpu_regs");
	if (!res) {
		dev_err(vpu_dev, "vpu: unable to get vpu base addr\n");
		return -ENODEV;
	}
	phy_vpu_base_addr = res->start;
	vpu_base = devm_ioremap_resource(vpu_dev, res);

	vpu_major = register_chrdev(vpu_major, "mxc_vpu_malone", &vpu_fops);
	if (vpu_major < 0) {
		dev_err(vpu_dev, "vpu: unable to get a major for VPU\n");
		err = -EBUSY;
		goto error;
	}

	vpu_class = class_create(THIS_MODULE, "mxc_vpu_malone");
	if (IS_ERR(vpu_class)) {
		err = PTR_ERR(vpu_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(vpu_class, NULL, MKDEV(vpu_major, 0),
				   NULL, "mxc_vpu_malone");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	vpu_clk = clk_get(&pdev->dev, "vpu_clk");
	if (IS_ERR(vpu_clk)) {
		err = -ENOENT;
		goto err_out_class;
	}

	vpu_dec_irq = platform_get_irq_byname(pdev, "dec_irq");
	if (vpu_dec_irq < 0) {
		dev_err(vpu_dev, "dec_irq is not found\n");
		err = -ENXIO;
		goto err_out_class;
	}
	err = devm_request_threaded_irq(vpu_dev, vpu_dec_irq, vpu_dec_irq_handler,
			NULL, 0, "VPU DEC IRQ", (void*)(&vpu_data));
	if (err < 0) {
		dev_err(vpu_dev, "failed to request irq for dec_irq\n");
		goto err_out_class;
	}

	vpu_dec_fiq = platform_get_irq_byname(pdev, "dec_fiq");
	if (vpu_dec_fiq < 0) {
		dev_err(vpu_dev, "dec_fiq is not found\n");
		err = -ENXIO;
		goto err_out_class;
	}
	err = devm_request_threaded_irq(vpu_dev, vpu_dec_fiq, vpu_dec_fiq_handler,
			NULL, 0, "VPU DEC FIQ", (void*)(&vpu_data));
	if (err < 0)
		dev_err(vpu_dev, "failed to request irq for dec_fiq\n");

	if (vpu_power_get(true)) {
		dev_err(vpu_dev, "failed to get vpu power\n");
		goto err_out_class;
	}

	pm_runtime_enable(&pdev->dev);

	vpu_data.workqueue = create_workqueue("vpu_wq");
	INIT_WORK(&vpu_data.work, vpu_worker_callback);
	mutex_init(&vpu_data.lock);
	dev_info(vpu_dev, "VPU initialized\n");

	goto out;

err_out_class:
	device_destroy(vpu_class, MKDEV(vpu_major, 0));
	class_destroy(vpu_class);
err_out_chrdev:
	unregister_chrdev(vpu_major, "mxc_vpu_malone");
error:
	devm_iounmap(vpu_dev, vpu_base);
out:
	return err;
}

static int vpu_dev_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	devm_free_irq(&pdev->dev, vpu_dec_irq, &vpu_data);
	devm_free_irq(&pdev->dev, vpu_dec_fiq, &vpu_data);
	cancel_work_sync(&vpu_data.work);
	flush_workqueue(vpu_data.workqueue);
	destroy_workqueue(vpu_data.workqueue);

	devm_iounmap(&pdev->dev, vpu_base);

	vpu_power_get(false);
	return 0;
}

#ifdef CONFIG_PM
static int vpu_suspend(struct device *dev)
{
	int i;
	unsigned long timeout;

	mutex_lock(&vpu_data.lock);
	if (open_count != 0) {
		/* Wait for vpu go to idle state, suspect vpu cannot be changed
		   to idle state after about 1 sec */
		timeout = jiffies + HZ;
		clk_prepare_enable(vpu_clk);
		/* checking busy */
		while (0) {
			msleep(1);
			if (time_after(jiffies, timeout)) {
				clk_disable_unprepare(vpu_clk);
				mutex_unlock(&vpu_data.lock);
				return -EAGAIN;
			}
		}
		clk_disable_unprepare(vpu_clk);

		/* Make sure clock is disabled before suspend */
		vpu_clk_usercount = atomic_read(&clk_cnt_from_ioc);
		for (i = 0; i < vpu_clk_usercount; i++) {
			clk_disable_unprepare(vpu_clk);
		}

		clk_prepare_enable(vpu_clk);
		/* Save registers */
		clk_disable_unprepare(vpu_clk);

		/* If VPU is working before suspend, disable
		 * regulator to make usecount right. */
		vpu_power_up(false);
	}

	mutex_unlock(&vpu_data.lock);
	return 0;
}

static int vpu_resume(struct device *dev)
{
	int i;

	mutex_lock(&vpu_data.lock);
	if (open_count != 0) {
		/* If VPU is working before suspend, enable
		 * regulator to make usecount right. */
		vpu_power_up(true);

		clk_prepare_enable(vpu_clk);

		/* Restore registers */

		clk_disable_unprepare(vpu_clk);

		/* Recover vpu clock */
		for (i = 0; i < vpu_clk_usercount; i++) {
			clk_prepare_enable(vpu_clk);
		}
	}

	mutex_unlock(&vpu_data.lock);
	return 0;
}

static int vpu_runtime_suspend(struct device *dev)
{
	release_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static int vpu_runtime_resume(struct device *dev)
{
	request_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
};

#else
#define	vpu_suspend	NULL
#define	vpu_resume	NULL
#endif				/* !CONFIG_PM */

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "nxp,imx8qm-vpu", },
	{ .compatible = "nxp,imx8qxp-vpu", },
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, vpu_of_match);

/*! Driver definition
 *
 */
static struct platform_driver mxcvpu_driver = {
	.driver = {
		   .name = "mxc_vpu_malone",
		   .of_match_table = vpu_of_match,
#ifdef CONFIG_PM
		   .pm = &vpu_pm_ops,
#endif
		   },
	.probe = vpu_dev_probe,
	.remove = vpu_dev_remove,
};

static int __init vpu_init(void)
{
	int ret = platform_driver_register(&mxcvpu_driver);

	init_waitqueue_head(&vpu_queue);

	return ret;
}

static void __exit vpu_exit(void)
{
	if (vpu_major > 0) {
		device_destroy(vpu_class, MKDEV(vpu_major, 0));
		class_destroy(vpu_class);
		unregister_chrdev(vpu_major, "mxc_vpu_malone");
		vpu_major = 0;
	}

	/* reset VPU state */
	vpu_power_up(true);
	clk_prepare_enable(vpu_clk);
	vpu_reset();
	clk_disable_unprepare(vpu_clk);
	vpu_power_up(false);

	clk_put(vpu_clk);

	platform_driver_unregister(&mxcvpu_driver);
	return;
}

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("Linux VPU driver for NXP i.MX/MXC");
MODULE_LICENSE("GPL");

module_init(vpu_init);
module_exit(vpu_exit);
