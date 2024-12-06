// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - vpu control device
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/genalloc.h>
#include <linux/vmalloc.h>
#include <linux/thermal.h>
#include <linux/units.h>
#include <linux/pm_opp.h>
#include <linux/freezer.h>

#include "wave6-vpuconfig.h"
#include "wave6-regdefine.h"
#include "wave6-vdi.h"
#include "wave6-vpu-ctrl.h"

#define wave6_wait_event_freezable_timeout(wq_head, condition, timeout)	 \
({                                                                       \
	int wave6_wait_ret = 0;                                          \
	unsigned long _timeout = timeout;                                \
	unsigned long stop;                                              \
	stop = jiffies + _timeout;                                       \
	do {                                                             \
		if (wave6_wait_ret == -ERESTARTSYS && freezing(current)) \
			clear_thread_flag(TIF_SIGPENDING);               \
		_timeout = stop - jiffies;                               \
		if ((long)_timeout <= 0) {                               \
			wave6_wait_ret = -ERESTARTSYS;                   \
			break;                                           \
		}                                                        \
		wave6_wait_ret = wait_event_freezable_timeout(wq_head, condition, _timeout); \
	} while (wave6_wait_ret == -ERESTARTSYS && freezing(current));   \
	wave6_wait_ret;                                                  \
})

static unsigned int debug;
module_param(debug, uint, 0644);

static unsigned int reload_firmware;
module_param(reload_firmware, uint, 0644);

#define dprintk(dev, fmt, arg...)					\
	do {								\
		if (debug)						\
			dev_info(dev, fmt, ## arg);			\
	} while (0)

struct vpu_ctrl_resource {
	const char *fw_name;
	u32 sram_size;
};

#define WAVE6_ENABLE_SW_UART	1

#if WAVE6_ENABLE_SW_UART
#include <linux/debugfs.h>

#define W6_NXP_SW_UART_LOGER                         (W6_REG_BASE + 0x00f0)
#define TRACEBUF_SIZE 131072

static unsigned int enable_fwlog;
module_param(enable_fwlog, uint, 0644);

struct loger_t {
	u32 size;
	u32 wptr;
	u32 rptr;
	u32 anchor;
	u32 count;
	u32 reserved[3];
	char vbase[];
};
#endif

struct vpu_ctrl_buf {
	struct list_head list;
	struct vpu_buf buf;
};

static int wave6_cooling_disable;
module_param(wave6_cooling_disable, int, 0644);
MODULE_PARM_DESC(wave6_cooling_disable, "enable or disable cooling");

struct vpu_ctrl {
	struct device *dev;
	void __iomem *reg_base;
	struct clk_bulk_data *clks;
	int num_clks;
	struct vpu_dma_buf boot_mem;
	u32 state;
	struct mutex ctrl_lock;
	struct wave6_vpu_entity *current_entity;
	struct list_head entities;
	const struct vpu_ctrl_resource *res;
	struct gen_pool *sram_pool;
	struct vpu_dma_buf sram_buf;
	struct list_head buffers;
	bool support_follower;
	wait_queue_head_t  load_fw_wq;
#if WAVE6_ENABLE_SW_UART
	struct vpu_buf loger_buf;
	struct loger_t *loger;
	struct dentry *debugfs;
#endif
	int thermal_event;
	int thermal_max;
	struct thermal_cooling_device *cooling;
	struct dev_pm_domain_list  *pd_list;
	struct device *dev_perf;
	int clk_id;
	unsigned long *freq_table;
};

#define DOMAIN_VPU_PWR  0
#define DOMAIN_VPU_PERF 1

static const struct vpu_ctrl_resource wave633c_ctrl_data = {
	.fw_name = "wave633c_codec_fw.bin",
	.sram_size = 0x18000,
};

#if WAVE6_ENABLE_SW_UART
static void wave6_vpu_ctrl_init_loger(struct vpu_ctrl *ctrl)
{
	ctrl->loger_buf.size = TRACEBUF_SIZE + sizeof(struct loger_t);
	ctrl->loger_buf.vaddr = dma_alloc_coherent(ctrl->dev,
						   ctrl->loger_buf.size,
						   &ctrl->loger_buf.daddr,
						   GFP_KERNEL);
	if (!ctrl->loger_buf.vaddr) {
		ctrl->loger_buf.size = 0;
		return;
	}
	ctrl->loger = ctrl->loger_buf.vaddr;
	ctrl->loger->size = TRACEBUF_SIZE;
}

static void wave6_vpu_ctrl_free_loger(struct vpu_ctrl *ctrl)
{
	ctrl->loger = NULL;
	if (ctrl->loger_buf.vaddr)
		dma_free_coherent(ctrl->dev,
				  ctrl->loger_buf.size,
				  ctrl->loger_buf.vaddr,
				  ctrl->loger_buf.daddr);

	memset(&ctrl->loger_buf, 0, sizeof(ctrl->loger_buf));
}

static void wave6_vpu_ctrl_start_loger(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	if (enable_fwlog)
		entity->write_reg(entity->dev, W6_NXP_SW_UART_LOGER, (u32)ctrl->loger_buf.daddr);
}

static void wave6_vpu_ctrl_stop_loger(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	entity->write_reg(entity->dev, W6_NXP_SW_UART_LOGER, 0);
}

static int wave6_vpu_loger_show(struct seq_file *s, void *data)
{
	struct vpu_ctrl *ctrl = s->private;
	u32 rptr;
	u32 wptr;
	int length;

	if (!ctrl->loger)
		return 0;

	rptr = ctrl->loger->rptr;
	wptr = ctrl->loger->wptr;

	if (rptr == wptr)
		return 0;

	if (rptr < wptr)
		length = wptr - rptr;
	else
		length = ctrl->loger->size - rptr;

	if (s->count + length >= s->size) {
		s->count = s->size;
		return 0;
	}

	if (!seq_write(s, ctrl->loger->vbase + rptr, length)) {
		rptr += length;
		if (rptr == ctrl->loger->size)
			rptr = 0;
		ctrl->loger->rptr = rptr;
	}

	return 0;
}

static int wave6_vpu_loger_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, wave6_vpu_loger_show, inode->i_private);
}

static const struct file_operations wave6_vpu_loger_fops = {
	.owner = THIS_MODULE,
	.open = wave6_vpu_loger_open,
	.release = single_release,
	.read = seq_read,
};

static void wave6_vpu_ctrl_create_debugfs(struct vpu_ctrl *ctrl)
{
	struct dentry *wave6_dbgfs = debugfs_lookup("wave6", NULL);

	if (!wave6_dbgfs)
		wave6_dbgfs = debugfs_create_dir("wave6", NULL);
	if (!wave6_dbgfs)
		return;

	ctrl->debugfs = debugfs_create_file("fwlog",
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    wave6_dbgfs,
					    ctrl,
					    &wave6_vpu_loger_fops);
}

static void wave6_vpu_ctrl_remove_debugfs(struct vpu_ctrl *ctrl)
{
	if (!ctrl->debugfs)
		return;

	debugfs_remove(ctrl->debugfs);
	ctrl->debugfs = NULL;
}
#endif

static void wave6_vpu_ctrl_writel(struct device *dev, u32 addr, u32 data)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	writel(data, ctrl->reg_base + addr);
}

static void byte_swap(unsigned char *data, int len)
{
	u8 temp;
	int i;

	for (i = 0; i < len; i += 2) {
		temp = data[i];
		data[i] = data[i + 1];
		data[i + 1] = temp;
	}
}

static void word_swap(unsigned char *data, int len)
{
	u16 temp;
	u16 *ptr = (u16 *)data;
	int i;
	s32 size = len / sizeof(uint16_t);

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static void dword_swap(unsigned char *data, int len)
{
	u32 temp;
	u32 *ptr = (u32 *)data;
	s32 size = len / sizeof(uint32_t);
	int i;

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

static void lword_swap(unsigned char *data, int len)
{
	u64 temp;
	u64 *ptr = (u64 *)data;
	s32 size = len / sizeof(uint64_t);
	int i;

	for (i = 0; i < size; i += 2) {
		temp = ptr[i];
		ptr[i] = ptr[i + 1];
		ptr[i + 1] = temp;
	}
}

int wave6_convert_endian(unsigned int endian)
{
	switch (endian) {
	case VDI_LITTLE_ENDIAN:
		endian = 0x00;
		break;
	case VDI_BIG_ENDIAN:
		endian = 0x0f;
		break;
	case VDI_32BIT_LITTLE_ENDIAN:
		endian = 0x04;
		break;
	case VDI_32BIT_BIG_ENDIAN:
		endian = 0x03;
		break;
	}

	return (endian & 0x0f);
}
EXPORT_SYMBOL_GPL(wave6_convert_endian);

void wave6_swap_endian(u8 *data, int len, int endian)
{
	int changes;
	int sys_endian;
	bool byte_change, word_change, dword_change, lword_change;

	sys_endian = VDI_128BIT_LITTLE_ENDIAN;

	endian = wave6_convert_endian(endian);
	sys_endian = wave6_convert_endian(sys_endian);
	if (endian == sys_endian)
		return;

	changes = endian ^ sys_endian;
	byte_change = changes & 0x01;
	word_change = ((changes & 0x02) == 0x02);
	dword_change = ((changes & 0x04) == 0x04);
	lword_change = ((changes & 0x08) == 0x08);

	if (byte_change)
		byte_swap(data, len);
	if (word_change)
		word_swap(data, len);
	if (dword_change)
		dword_swap(data, len);
	if (lword_change)
		lword_swap(data, len);
}
EXPORT_SYMBOL_GPL(wave6_swap_endian);

int wave6_alloc_dma(struct device *dev, struct vpu_buf *vb)
{
	void *vaddr;
	dma_addr_t daddr;

	if (!vb || !vb->size)
		return -EINVAL;

	vaddr = dma_alloc_coherent(dev, vb->size, &daddr, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;

	vb->vaddr = vaddr;
	vb->daddr = daddr;
	vb->dev = dev;

	return 0;
}
EXPORT_SYMBOL_GPL(wave6_alloc_dma);

int wave6_write_dma(struct vpu_buf *vb, size_t offset, u8 *data, int len, int endian)
{
	if (!vb)
		return -EINVAL;

	if (!vb->vaddr) {
		dev_err(vb->dev, "%s(): unable to write to unmapped buffer\n", __func__);
		return -EINVAL;
	}

	if (offset > vb->size || len > vb->size || offset + len > vb->size) {
		dev_err(vb->dev, "%s(): buffer too small\n", __func__);
		return -ENOSPC;
	}

	wave6_swap_endian(data, len, endian);
	memcpy(vb->vaddr + offset, data, len);

	return len;
}
EXPORT_SYMBOL_GPL(wave6_write_dma);

void wave6_free_dma(struct vpu_buf *vb)
{
	if (!vb || !vb->size || !vb->vaddr)
		return;

	dma_free_coherent(vb->dev, vb->size, vb->vaddr, vb->daddr);
	memset(vb, 0, sizeof(*vb));
}
EXPORT_SYMBOL_GPL(wave6_free_dma);

static const char *wave6_vpu_ctrl_state_name(u32 state)
{
	switch (state) {
	case WAVE6_VPU_STATE_OFF:
		return "off";
	case WAVE6_VPU_STATE_PREPARE:
		return "prepare";
	case WAVE6_VPU_STATE_ON:
		return "on";
	case WAVE6_VPU_STATE_SLEEP:
		return "sleep";
	default:
		return "unknown";
	}
}

static void wave6_vpu_ctrl_set_state(struct vpu_ctrl *ctrl, u32 state)
{
	dprintk(ctrl->dev, "set state: %s -> %s\n",
		wave6_vpu_ctrl_state_name(ctrl->state), wave6_vpu_ctrl_state_name(state));
	ctrl->state = state;
}

static int wave6_vpu_ctrl_wait_busy(struct wave6_vpu_entity *entity)
{
	u32 val;

	return read_poll_timeout(entity->read_reg, val, val == 0,
				 10, W6_VPU_POLL_TIMEOUT, false,
				 entity->dev, W6_VPU_BUSY_STATUS);
}

static int wave6_vpu_ctrl_check_result(struct wave6_vpu_entity *entity)
{
	if (entity->read_reg(entity->dev, W6_RET_SUCCESS))
		return 0;

	return entity->read_reg(entity->dev, W6_RET_FAIL_REASON);
}

static u32 wave6_vpu_ctrl_get_code_buf_size(struct vpu_ctrl *ctrl)
{
	return min_t(u32, ctrl->boot_mem.size, WAVE6_MAX_CODE_BUF_SIZE);
}

static void wave6_vpu_ctrl_remap_code_buffer(struct vpu_ctrl *ctrl)
{
	dma_addr_t code_base = ctrl->boot_mem.dma_addr;
	u32 i, reg_val, remap_size;

	for (i = 0; i < wave6_vpu_ctrl_get_code_buf_size(ctrl) / W6_REMAP_MAX_SIZE; i++) {
		remap_size = (W6_REMAP_MAX_SIZE >> 12) & 0x1ff;
		reg_val = 0x80000000 |
			  (WAVE6_UPPER_PROC_AXI_ID << 20) |
			  (0 << 16) |
			  (i << 12) |
			  BIT(11) |
			  remap_size;
		wave6_vpu_ctrl_writel(ctrl->dev, W6_VPU_REMAP_CTRL_GB, reg_val);
		wave6_vpu_ctrl_writel(ctrl->dev, W6_VPU_REMAP_VADDR_GB, i * W6_REMAP_MAX_SIZE);
		wave6_vpu_ctrl_writel(ctrl->dev, W6_VPU_REMAP_PADDR_GB,
				      code_base + i * W6_REMAP_MAX_SIZE);
	}
}

static int wave6_vpu_ctrl_init_vpu(struct vpu_ctrl *ctrl)
{
	struct wave6_vpu_entity *entity = ctrl->current_entity;
	int ret;

	dprintk(ctrl->dev, "cold boot vpu\n");
	entity->write_reg(entity->dev, W6_VPU_BUSY_STATUS, 1);
	entity->write_reg(entity->dev, W6_CMD_INIT_VPU_SEC_AXI_BASE_CORE0,
				       ctrl->sram_buf.dma_addr);
	entity->write_reg(entity->dev, W6_CMD_INIT_VPU_SEC_AXI_SIZE_CORE0,
				       ctrl->sram_buf.size);
	wave6_vpu_ctrl_writel(ctrl->dev, W6_COMMAND_GB, W6_INIT_VPU);
	wave6_vpu_ctrl_writel(ctrl->dev, W6_VPU_REMAP_CORE_START_GB, 1);

	ret = wave6_vpu_ctrl_wait_busy(entity);
	if (ret) {
		dev_err(ctrl->dev, "init vpu timeout\n");
		return -EINVAL;
	}

	ret = wave6_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "init vpu fail, reason 0x%x\n", ret);
		return -EIO;
	}

	return 0;
}

static void wave6_vpu_ctrl_on_boot(struct wave6_vpu_entity *entity)
{
	if (!entity->on_boot)
		return;

	if (!entity->booted) {
		entity->on_boot(entity->dev);
		entity->booted = true;
	}
}

static void wave6_vpu_ctrl_clear_firmware_buffers(struct vpu_ctrl *ctrl,
						  struct wave6_vpu_entity *entity)
{
	int ret;

	dprintk(ctrl->dev, "clear firmware work buffers\n");
	entity->write_reg(entity->dev, W6_VPU_BUSY_STATUS, 1);
	entity->write_reg(entity->dev, W6_COMMAND, W6_INIT_WORK_BUF);
	entity->write_reg(entity->dev, W6_VPU_HOST_INT_REQ, 1);

	ret = wave6_vpu_ctrl_wait_busy(entity);
	if (ret) {
		dev_err(ctrl->dev, "set buffer failed\n");
		return;
	}

	ret = wave6_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "set buffer failed, reason 0x%x\n", ret);
		return;
	}
}

int wave6_vpu_ctrl_require_buffer(struct device *dev, struct wave6_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	struct vpu_ctrl_buf *pbuf;
	u32 size;
	int ret = -ENOMEM;

	if (!ctrl || !entity)
		return -EINVAL;

	size = entity->read_reg(entity->dev, W6_CMD_SET_CTRL_WORK_BUF_SIZE);
	if (!size)
		return 0;

	pbuf = vzalloc(sizeof(*pbuf));
	if (!pbuf)
		goto exit;

	pbuf->buf.size = size;
	ret = wave6_alloc_dma(ctrl->dev, &pbuf->buf);
	if (ret) {
		vfree(pbuf);
		goto exit;
	}

	list_add_tail(&pbuf->list, &ctrl->buffers);
	entity->write_reg(entity->dev, W6_CMD_SET_CTRL_WORK_BUF_ADDR, pbuf->buf.daddr);
exit:
	entity->write_reg(entity->dev, W6_CMD_SET_CTRL_WORK_BUF_SIZE, 0);
	return ret;
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_require_buffer);

static void wave6_vpu_ctrl_clear_buffers(struct vpu_ctrl *ctrl)
{
	struct wave6_vpu_entity *entity;
	struct vpu_ctrl_buf *pbuf, *tmp;

	dprintk(ctrl->dev, "clear all buffers\n");
	entity = list_first_entry_or_null(&ctrl->entities,
					  struct wave6_vpu_entity, list);
	if (entity)
		wave6_vpu_ctrl_clear_firmware_buffers(ctrl, entity);

	list_for_each_entry_safe(pbuf, tmp, &ctrl->buffers, list) {
		list_del(&pbuf->list);
		wave6_free_dma(&pbuf->buf);
		vfree(pbuf);
	}
}

static void wave6_vpu_ctrl_boot_done(struct vpu_ctrl *ctrl, int wakeup)
{
	struct wave6_vpu_entity *entity;

	if (ctrl->state == WAVE6_VPU_STATE_ON)
		return;

	dprintk(ctrl->dev, "boot done from %s\n", wakeup ? "wakeup" : "cold boot");

	if (!wakeup)
		wave6_vpu_ctrl_clear_buffers(ctrl);

	list_for_each_entry(entity, &ctrl->entities, list)
		wave6_vpu_ctrl_on_boot(entity);

	wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_ON);
}

static bool wave6_vpu_ctrl_find_entity(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	struct wave6_vpu_entity *tmp;

	list_for_each_entry(tmp, &ctrl->entities, list) {
		if (tmp == entity)
			return true;
	}

	return false;
}

static void wave6_vpu_ctrl_load_firmware(const struct firmware *fw, void *context)
{
	struct vpu_ctrl *ctrl = context;
	struct wave6_vpu_entity *entity = ctrl->current_entity;
	u32 product_code;
	int ret;

	ret = pm_runtime_resume_and_get(ctrl->dev);
	if (ret) {
		dev_err(ctrl->dev, "pm runtime resume fail, ret = %d\n", ret);
		mutex_lock(&ctrl->ctrl_lock);
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		ctrl->current_entity = NULL;
		mutex_unlock(&ctrl->ctrl_lock);
		release_firmware(fw);
		return;
	}

	dprintk(ctrl->dev, "loading firmware\n");

	if (!fw || !fw->data) {
		dev_err(ctrl->dev, "No firmware.\n");
		ret = -EINVAL;
		goto exit;
	}

	if (fw->size + WAVE6_EXTRA_CODE_BUF_SIZE > wave6_vpu_ctrl_get_code_buf_size(ctrl)) {
		dev_err(ctrl->dev, "firmware size (%ld > %zd) is too big\n",
			fw->size, ctrl->boot_mem.size);
		ret = -EINVAL;
		goto exit;
	}

	product_code = entity->read_reg(entity->dev, W6_VPU_RET_PRODUCT_VERSION);
	if (!PRODUCT_CODE_W_SERIES(product_code)) {
		dev_err(ctrl->dev, "unknown product id : %08x\n", product_code);
		ret = -EINVAL;
		goto exit;
	}

	wave6_swap_endian((u8 *)fw->data, fw->size, VDI_128BIT_LITTLE_ENDIAN);
	memcpy(ctrl->boot_mem.vaddr, fw->data, fw->size);

exit:
	mutex_lock(&ctrl->ctrl_lock);
	if (!ret && wave6_vpu_ctrl_find_entity(ctrl, ctrl->current_entity)) {
		wave6_vpu_ctrl_remap_code_buffer(ctrl);
		ret = wave6_vpu_ctrl_init_vpu(ctrl);
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&ctrl->ctrl_lock);

	pm_runtime_put_sync(ctrl->dev);
	release_firmware(fw);

	mutex_lock(&ctrl->ctrl_lock);
	ctrl->current_entity = NULL;
	if (ret)
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
	else
		wave6_vpu_ctrl_boot_done(ctrl, 0);
	mutex_unlock(&ctrl->ctrl_lock);

	wake_up_interruptible_all(&ctrl->load_fw_wq);
}

static int wave6_vpu_ctrl_sleep(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	int ret;

	dprintk(ctrl->dev, "sleep firmware\n");
	entity->write_reg(entity->dev, W6_VPU_BUSY_STATUS, 1);
	entity->write_reg(entity->dev, W6_CMD_INSTANCE_INFO, (0 << 16) | 0);
	entity->write_reg(entity->dev, W6_COMMAND, W6_SLEEP_VPU);
	entity->write_reg(entity->dev, W6_VPU_HOST_INT_REQ, 1);

	ret = wave6_vpu_ctrl_wait_busy(entity);
	if (ret) {
		dev_err(ctrl->dev, "sleep vpu timeout\n");
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		return -EINVAL;
	}

	ret = wave6_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "sleep vpu fail, reason 0x%x\n", ret);
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		return -EIO;
	}

	wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_SLEEP);

	return 0;
}

static int wave6_vpu_ctrl_wakeup(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	int ret;

	dprintk(ctrl->dev, "wakeup firmware\n");
	wave6_vpu_ctrl_remap_code_buffer(ctrl);

	entity->write_reg(entity->dev, W6_VPU_BUSY_STATUS, 1);
	entity->write_reg(entity->dev, W6_CMD_INIT_VPU_SEC_AXI_BASE_CORE0,
				       ctrl->sram_buf.dma_addr);
	entity->write_reg(entity->dev, W6_CMD_INIT_VPU_SEC_AXI_SIZE_CORE0,
				       ctrl->sram_buf.size);
	wave6_vpu_ctrl_writel(ctrl->dev, W6_COMMAND_GB, W6_WAKEUP_VPU);
	wave6_vpu_ctrl_writel(ctrl->dev, W6_VPU_REMAP_CORE_START_GB, 1);

	ret = wave6_vpu_ctrl_wait_busy(entity);
	if (ret) {
		dev_err(ctrl->dev, "wakeup vpu timeout\n");
		return -EINVAL;
	}

	ret = wave6_vpu_ctrl_check_result(entity);
	if (ret) {
		dev_err(ctrl->dev, "wakeup vpu fail, reason 0x%x\n", ret);
		return -EIO;
	}

	wave6_vpu_ctrl_boot_done(ctrl, 1);

	return 0;
}

static int wave6_vpu_ctrl_try_boot(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	int ret;

	if (ctrl->state != WAVE6_VPU_STATE_OFF && ctrl->state != WAVE6_VPU_STATE_SLEEP)
		return 0;

	if (reload_firmware)
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);

	if (entity->read_reg(entity->dev, W6_VCPU_CUR_PC)) {
		dprintk(ctrl->dev, "try boot directly as firmware is running\n");
		wave6_vpu_ctrl_boot_done(ctrl, ctrl->state == WAVE6_VPU_STATE_SLEEP);
		return 0;
	}

	if (ctrl->state == WAVE6_VPU_STATE_SLEEP) {
		ret = wave6_vpu_ctrl_wakeup(ctrl, entity);
		return ret;
	}

	wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_PREPARE);
	ctrl->current_entity = entity;
	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_UEVENT,
				      ctrl->res->fw_name,
				      ctrl->dev, GFP_KERNEL,
				      ctrl,
				      wave6_vpu_ctrl_load_firmware);
	if (ret) {
		dev_err(ctrl->dev, "request firmware %s fail, ret = %d\n", ctrl->res->fw_name, ret);
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		return ret;
	}

	return 0;
}

bool wave6_vpu_ctrl_support_follower(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl)
		return false;

	return ctrl->support_follower;
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_support_follower);

int wave6_vpu_ctrl_resume_and_get(struct device *dev, struct wave6_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	bool boot_flag;
	int ret = 0;

	if (!ctrl)
		return -EINVAL;

	if (!entity || !entity->dev || !entity->read_reg || !entity->write_reg)
		return -EINVAL;

	mutex_lock(&ctrl->ctrl_lock);

	ret = pm_runtime_resume_and_get(ctrl->dev);
	if (ret) {
		dev_err(dev, "pm runtime resume fail, ret = %d\n", ret);
		mutex_unlock(&ctrl->ctrl_lock);
		return ret;
	}

#if WAVE6_ENABLE_SW_UART
	wave6_vpu_ctrl_start_loger(ctrl, entity);
#endif

	entity->booted = false;

	if (ctrl->current_entity)
		boot_flag = false;
	else
		boot_flag = list_empty(&ctrl->entities) ? true : false;

	list_add_tail(&entity->list, &ctrl->entities);
	if (boot_flag)
		ret = wave6_vpu_ctrl_try_boot(ctrl, entity);

	if (ctrl->state == WAVE6_VPU_STATE_ON)
		wave6_vpu_ctrl_on_boot(entity);

	if (ret)
		pm_runtime_put_sync(ctrl->dev);

	mutex_unlock(&ctrl->ctrl_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_resume_and_get);

void wave6_vpu_ctrl_put_sync(struct device *dev, struct wave6_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl)
		return;

	if (entity == ctrl->current_entity)
		wave6_vpu_ctrl_wait_done(dev);

	mutex_lock(&ctrl->ctrl_lock);

	if (!wave6_vpu_ctrl_find_entity(ctrl, entity))
		goto exit;

	list_del_init(&entity->list);
	if (list_empty(&ctrl->entities)) {
		if (!entity->read_reg(entity->dev, W6_VCPU_CUR_PC))
			wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		else
			wave6_vpu_ctrl_sleep(ctrl, entity);
	}

#if WAVE6_ENABLE_SW_UART
	wave6_vpu_ctrl_stop_loger(ctrl, entity);
#endif

	if (!pm_runtime_suspended(ctrl->dev))
		pm_runtime_put_sync(ctrl->dev);
exit:
	mutex_unlock(&ctrl->ctrl_lock);
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_put_sync);

int wave6_vpu_ctrl_wait_done(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	int ret;

	if (!ctrl)
		return -EINVAL;

	if (ctrl->state == WAVE6_VPU_STATE_OFF)
		return -EINVAL;

	if (ctrl->state == WAVE6_VPU_STATE_ON)
		return 0;

	ret = wave6_wait_event_freezable_timeout(ctrl->load_fw_wq,
						 wave6_vpu_ctrl_get_state(dev) ==
						 WAVE6_VPU_STATE_ON,
						 msecs_to_jiffies(W6_BOOT_WAIT_TIMEOUT));
	if (ret == -ERESTARTSYS || ret == 0) {
		dev_err(ctrl->dev, "fail to wait vcpu boot done,ret %d\n", ret);
		mutex_lock(&ctrl->ctrl_lock);
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		mutex_unlock(&ctrl->ctrl_lock);
		return -EINVAL;
	}

	mutex_lock(&ctrl->ctrl_lock);
	wave6_vpu_ctrl_boot_done(ctrl, 0);
	mutex_unlock(&ctrl->ctrl_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_wait_done);

int wave6_vpu_ctrl_get_state(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl)
		return -EINVAL;

	return ctrl->state;
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_get_state);

static void wave6_vpu_ctrl_init_reserved_boot_region(struct vpu_ctrl *ctrl)
{
	if (ctrl->boot_mem.size < WAVE6_CODE_BUF_SIZE) {
		dev_warn(ctrl->dev, "boot memory size (%zu) is too small\n", ctrl->boot_mem.size);
		ctrl->boot_mem.phys_addr = 0;
		ctrl->boot_mem.size = 0;
		memset(&ctrl->boot_mem, 0, sizeof(ctrl->boot_mem));
		return;
	}

	ctrl->boot_mem.vaddr = devm_memremap(ctrl->dev,
					     ctrl->boot_mem.phys_addr,
					     ctrl->boot_mem.size,
					     MEMREMAP_WC);
	if (!ctrl->boot_mem.vaddr) {
		memset(&ctrl->boot_mem, 0, sizeof(ctrl->boot_mem));
		return;
	}

	ctrl->boot_mem.dma_addr = dma_map_resource(ctrl->dev,
						   ctrl->boot_mem.phys_addr,
						   ctrl->boot_mem.size,
						   DMA_BIDIRECTIONAL,
						   0);
	if (!ctrl->boot_mem.dma_addr) {
		memset(&ctrl->boot_mem, 0, sizeof(ctrl->boot_mem));
		return;
	}

	dev_info(ctrl->dev, "boot phys_addr: %pad, dma_addr: %pad, size: 0x%zx\n",
		 &ctrl->boot_mem.phys_addr,
		 &ctrl->boot_mem.dma_addr,
		 ctrl->boot_mem.size);
}

static int wave6_vpu_ctrl_thermal_update(struct device *dev, int state)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	unsigned long new_clock_rate;
	int ret = 0;

	if (wave6_cooling_disable || !ctrl->dev_perf || state > ctrl->thermal_max || !ctrl->cooling)
		return 0;

	new_clock_rate = DIV_ROUND_UP(ctrl->freq_table[state], HZ_PER_KHZ);
	dev_dbg(dev, "receive cooling set state: %d, new clock rate %ld\n",
		state, new_clock_rate);
	ret = dev_pm_genpd_set_performance_state(ctrl->dev_perf, new_clock_rate);
	dev_dbg(dev, "clk set to %lu\n", clk_get_rate(ctrl->clks[ctrl->clk_id].clk));
	if (ret && !((ret == -ENODEV) || (ret == -EOPNOTSUPP))) {
		dev_err(dev, "failed to set perf to %lu (ret = %d)\n", new_clock_rate, ret);
		return ret;
	}

	return 0;
}

static int wave6_cooling_get_max_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	struct vpu_ctrl *ctrl = cdev->devdata;

	*state = ctrl->thermal_max;
	return 0;
}

static int wave6_cooling_get_cur_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	struct vpu_ctrl *ctrl = cdev->devdata;

	*state = ctrl->thermal_event;

	return 0;
}

static int wave6_cooling_set_cur_state(struct thermal_cooling_device *cdev,
	unsigned long state)
{
	struct vpu_ctrl *ctrl = cdev->devdata;
	struct wave6_vpu_entity *entity;

	ctrl->thermal_event = state;

	list_for_each_entry(entity, &ctrl->entities, list) {
		if (entity->pause)
			entity->pause(entity->dev, 0);
	}

	wave6_vpu_ctrl_thermal_update(ctrl->dev, state);

	list_for_each_entry(entity, &ctrl->entities, list) {
		if (entity->pause)
			entity->pause(entity->dev, 1);
	}

	return 0;
}

static struct thermal_cooling_device_ops wave6_cooling_ops = {
	.get_max_state = wave6_cooling_get_max_state,
	.get_cur_state = wave6_cooling_get_cur_state,
	.set_cur_state = wave6_cooling_set_cur_state,
};

static void wave6_cooling_remove(struct vpu_ctrl *ctrl)
{
	int i;

	if (!ctrl->pd_list)
		return;

	thermal_cooling_device_unregister(ctrl->cooling);

	kfree(ctrl->freq_table);
	ctrl->freq_table = NULL;

	for (i = 0; i < ctrl->pd_list->num_pds; i++) {
		struct device *pd_dev = ctrl->pd_list->pd_devs[i];

		if (!pm_runtime_suspended(pd_dev))
			pm_runtime_force_suspend(pd_dev);
	}

	dev_pm_domain_detach_list(ctrl->pd_list);
	ctrl->pd_list = NULL;
	ctrl->dev_perf = NULL;
}

static void wave6_cooling_init(struct vpu_ctrl *ctrl)
{
	struct dev_pm_domain_attach_data pd_data = {
		.pd_names = (const char *[]) {"vpumix", "vpuperf"},
		.num_pd_names = 2,
	};
	int ret;
	int i;
	int num_opps;
	unsigned long freq;

	ctrl->clk_id = -1;
	for (i = 0; i < ctrl->num_clks; i++)
		if (!strcmp("vpu", ctrl->clks[i].id)) {
			ctrl->clk_id = i;
			break;
		}
	if (ctrl->clk_id == -1) {
		dev_err(ctrl->dev, "cooling device unable to get clock\n");
		return;
	}
	ret = dev_pm_domain_attach_list(ctrl->dev, &pd_data, &ctrl->pd_list);
	ctrl->dev_perf = NULL;
	if (ret < 0)
		dev_err(ctrl->dev, "didn't attach perf power domains, ret=%d", ret);
	else if (ret == 2)
		ctrl->dev_perf = ctrl->pd_list->pd_devs[DOMAIN_VPU_PERF];
	dev_dbg(ctrl->dev, "get perf domain ret=%d, perf=%p\n", ret, ctrl->dev_perf);
	if (!ctrl->dev_perf)
		return;

	num_opps = dev_pm_opp_get_opp_count(ctrl->dev_perf);
	if (num_opps <= 0) {
		dev_err(ctrl->dev, "fail to get pm opp count, ret = %d\n", num_opps);
		goto error;
	}
	ctrl->freq_table = kcalloc(num_opps, sizeof(*ctrl->freq_table), GFP_KERNEL);
	if (!ctrl->freq_table)
		goto error;

	for (i = 0, freq = ULONG_MAX; i < num_opps; i++, freq--) {
		struct dev_pm_opp *opp;

		opp = dev_pm_opp_find_freq_floor(ctrl->dev_perf, &freq);
		if (IS_ERR(opp))
			break;
		dev_pm_opp_put(opp);

		dev_dbg(ctrl->dev, "[%d] = %ld\n", i, freq);
		if (freq < 100 * HZ_PER_MHZ)
			break;
		ctrl->freq_table[i] = freq;
		ctrl->thermal_max = i;
	}
	if (!ctrl->thermal_max)
		goto error;

	ctrl->thermal_event = 0;
	ctrl->cooling = thermal_of_cooling_device_register(ctrl->dev->of_node,
		(char *)dev_name(ctrl->dev), ctrl, &wave6_cooling_ops);
	if (IS_ERR(ctrl->cooling)) {
		dev_err(ctrl->dev, "register cooling device failed\n");
		goto error;
	}
	return;

error:
	wave6_cooling_remove(ctrl);
}

static int wave6_vpu_ctrl_probe(struct platform_device *pdev)
{
	struct vpu_ctrl *ctrl;
	struct device_node *np;
	const struct vpu_ctrl_resource *res;
	int ret;

	/* physical addresses limited to 32 bits */
	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

	res = of_device_get_match_data(&pdev->dev);
	if (!res)
		return -ENODEV;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, ctrl);
	ctrl->dev = &pdev->dev;
	ctrl->res = res;
	ctrl->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->reg_base))
		return PTR_ERR(ctrl->reg_base);
	ret = devm_clk_bulk_get_all(&pdev->dev, &ctrl->clks);
	if (ret < 0) {
		dev_warn(&pdev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}

	ctrl->num_clks = ret;

	np = of_parse_phandle(pdev->dev.of_node, "boot", 0);
	if (np) {
		struct resource mem;

		ret = of_address_to_resource(np, 0, &mem);
		of_node_put(np);
		if (!ret) {
			ctrl->boot_mem.phys_addr = mem.start;
			ctrl->boot_mem.size = resource_size(&mem);
			wave6_vpu_ctrl_init_reserved_boot_region(ctrl);
		} else {
			dev_warn(&pdev->dev, "boot resource is not available.\n");
		}
	}

	ctrl->sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram", 0);
	if (ctrl->sram_pool) {
		ctrl->sram_buf.size = ctrl->res->sram_size;
		ctrl->sram_buf.vaddr = gen_pool_dma_alloc(ctrl->sram_pool,
							  ctrl->sram_buf.size,
							  &ctrl->sram_buf.phys_addr);
		if (!ctrl->sram_buf.vaddr)
			ctrl->sram_buf.size = 0;
		else
			ctrl->sram_buf.dma_addr = dma_map_resource(&pdev->dev,
								   ctrl->sram_buf.phys_addr,
								   ctrl->sram_buf.size,
								   DMA_BIDIRECTIONAL,
								   0);

		dev_info(&pdev->dev, "sram 0x%pad, 0x%pad, size 0x%lx\n",
			 &ctrl->sram_buf.phys_addr, &ctrl->sram_buf.dma_addr, ctrl->sram_buf.size);
	}

	mutex_init(&ctrl->ctrl_lock);
	init_waitqueue_head(&ctrl->load_fw_wq);
	INIT_LIST_HEAD(&ctrl->entities);
	INIT_LIST_HEAD(&ctrl->buffers);

#if WAVE6_ENABLE_SW_UART
	wave6_vpu_ctrl_init_loger(ctrl);
	wave6_vpu_ctrl_create_debugfs(ctrl);
#endif

	pm_runtime_enable(&pdev->dev);

	wave6_cooling_init(ctrl);

	if (of_find_property(pdev->dev.of_node, "support-follower", NULL))
		ctrl->support_follower = true;

	return 0;
}

static void wave6_vpu_ctrl_remove(struct platform_device *pdev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(&pdev->dev);

#if WAVE6_ENABLE_SW_UART
	wave6_vpu_ctrl_remove_debugfs(ctrl);
	wave6_vpu_ctrl_free_loger(ctrl);
#endif

	pm_runtime_disable(&pdev->dev);

	wave6_vpu_ctrl_clear_buffers(ctrl);
	wave6_cooling_remove(ctrl);
	if (ctrl->sram_pool && ctrl->sram_buf.vaddr) {
		dma_unmap_resource(&pdev->dev,
				   ctrl->sram_buf.dma_addr,
				   ctrl->sram_buf.size,
				   DMA_BIDIRECTIONAL,
				   0);
		gen_pool_free(ctrl->sram_pool,
			      (unsigned long)ctrl->sram_buf.vaddr,
			      ctrl->sram_buf.size);
	}
	if (ctrl->boot_mem.dma_addr)
		dma_unmap_resource(&pdev->dev,
				   ctrl->boot_mem.dma_addr,
				   ctrl->boot_mem.size,
				   DMA_BIDIRECTIONAL,
				   0);
	mutex_destroy(&ctrl->ctrl_lock);
}

#ifdef CONFIG_PM
static int wave6_vpu_ctrl_runtime_suspend(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ctrl->num_clks, ctrl->clks);
	return 0;
}

static int wave6_vpu_ctrl_runtime_resume(struct device *dev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ctrl->num_clks, ctrl->clks);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int wave6_vpu_ctrl_suspend(struct device *dev)
{
	return 0;
}

static int wave6_vpu_ctrl_resume(struct device *dev)
{
	return 0;
}
#endif
static const struct dev_pm_ops wave6_vpu_ctrl_pm_ops = {
	SET_RUNTIME_PM_OPS(wave6_vpu_ctrl_runtime_suspend, wave6_vpu_ctrl_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(wave6_vpu_ctrl_suspend, wave6_vpu_ctrl_resume)
};

static const struct of_device_id wave6_ctrl_ids[] = {
	{ .compatible = "fsl,cm633c-vpu-ctrl", .data = &wave633c_ctrl_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wave6_ctrl_ids);

static struct platform_driver wave6_vpu_ctrl_driver = {
	.driver = {
		.name = "vpu-ctrl",
		.of_match_table = of_match_ptr(wave6_ctrl_ids),
		.pm = &wave6_vpu_ctrl_pm_ops,
	},
	.probe = wave6_vpu_ctrl_probe,
	.remove = wave6_vpu_ctrl_remove,
};

module_platform_driver(wave6_vpu_ctrl_driver);
MODULE_DESCRIPTION("chips&media VPU WAVE6 CTRL");
MODULE_LICENSE("Dual BSD/GPL");
