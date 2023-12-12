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
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/genalloc.h>

#include "wave6-vpuconfig.h"
#include "wave6-regdefine.h"
#include "wave6-vdi.h"
#include "wave6-vpu-ctrl.h"

#define W6_VCPU_BOOT_TIMEOUT	3000000

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
#if WAVE6_ENABLE_SW_UART
	struct vpu_buf loger_buf;
	struct loger_t *loger;
	struct dentry *debugfs;
#endif
};

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

void wave6_swap_endian(u32 product_code, u8 *data, int len, int endian)
{
	int changes;
	int sys_endian;
	bool byte_change, word_change, dword_change, lword_change;

	WARN_ON(!PRODUCT_CODE_W_SERIES(product_code));
	if (!PRODUCT_CODE_W_SERIES(product_code))
		return;

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

	return read_poll_timeout(entity->read_reg, val, val == 0, 10, W6_VCPU_BOOT_TIMEOUT, false,
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

	entity->write_reg(entity->dev, W6_VPU_BUSY_STATUS, 1);
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

static void wave6_vpu_ctrl_boot_done(struct vpu_ctrl *ctrl)
{
	struct wave6_vpu_entity *entity;

	if (ctrl->state == WAVE6_VPU_STATE_ON)
		return;

	list_for_each_entry(entity, &ctrl->entities, list)
		wave6_vpu_ctrl_on_boot(entity);

	wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_ON);
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
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		ctrl->current_entity = NULL;
		release_firmware(fw);
		return;
	}

	dprintk(ctrl->dev, "loading firmware\n");

	if (!fw || !fw->data) {
		dev_err(ctrl->dev, "No firmware.\n");
		goto error;
	}

	if (fw->size + WAVE6_EXTRA_CODE_BUF_SIZE > wave6_vpu_ctrl_get_code_buf_size(ctrl)) {
		dev_err(ctrl->dev, "firmware size (%ld > %zd) is too big\n",
			fw->size, ctrl->boot_mem.size);
		goto error;
	}

	product_code = entity->read_reg(entity->dev, W6_VPU_RET_PRODUCT_VERSION);
	if (!PRODUCT_CODE_W_SERIES(product_code)) {
		dev_err(ctrl->dev, "unknown product id : %08x\n", product_code);
		goto error;
	}

	wave6_swap_endian(product_code, (u8 *)fw->data, fw->size, VDI_128BIT_LITTLE_ENDIAN);
	memcpy(ctrl->boot_mem.vaddr, fw->data, fw->size);
	wave6_vpu_ctrl_remap_code_buffer(ctrl);
	ret = wave6_vpu_ctrl_init_vpu(ctrl);
	if (ret)
		goto error;

	mutex_lock(&ctrl->ctrl_lock);
	entity->write_reg(entity->dev, W6_VPU_REG_GLOBAL_WR, 0);
	ctrl->current_entity = NULL;
	wave6_vpu_ctrl_boot_done(ctrl);
	mutex_unlock(&ctrl->ctrl_lock);

	pm_runtime_put_sync(ctrl->dev);
	release_firmware(fw);
	return;
error:
	entity->write_reg(entity->dev, W6_VPU_REG_GLOBAL_WR, 0);
	wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
	ctrl->current_entity = NULL;
	pm_runtime_put_sync(ctrl->dev);
	release_firmware(fw);
}

static int wave6_vpu_ctrl_sleep(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	int ret;

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

	wave6_vpu_ctrl_remap_code_buffer(ctrl);

	entity->write_reg(entity->dev, W6_VPU_BUSY_STATUS, 1);
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

	wave6_vpu_ctrl_boot_done(ctrl);

	return 0;
}

static int wave6_vpu_ctrl_try_boot(struct vpu_ctrl *ctrl, struct wave6_vpu_entity *entity)
{
	int ret;

	if (ctrl->state != WAVE6_VPU_STATE_OFF && ctrl->state != WAVE6_VPU_STATE_SLEEP)
		return 0;

	if (reload_firmware)
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);

	if (entity->read_reg(entity->dev, W6_VPU_REG_GLOBAL_WR)) {
		u32 val;

		/* the vcpu may be booted by other vm */
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_PREPARE);
		ret = read_poll_timeout(entity->read_reg, val, val != 0,
					10, W6_VCPU_BOOT_TIMEOUT, false,
					entity->dev, W6_VCPU_CUR_PC);
		if (!ret)
			wave6_vpu_ctrl_boot_done(ctrl);
		else
			wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		return ret;
	}
	entity->write_reg(entity->dev, W6_VPU_REG_GLOBAL_WR, 1);

	if (entity->read_reg(entity->dev, W6_VCPU_CUR_PC)) {
		entity->write_reg(entity->dev, W6_VPU_REG_GLOBAL_WR, 0);
		wave6_vpu_ctrl_boot_done(ctrl);
		return 0;
	}

	if (ctrl->state == WAVE6_VPU_STATE_SLEEP) {
		ret = wave6_vpu_ctrl_wakeup(ctrl, entity);
		entity->write_reg(entity->dev, W6_VPU_REG_GLOBAL_WR, 0);
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
		entity->write_reg(entity->dev, W6_VPU_REG_GLOBAL_WR, 0);
		return ret;
	}

	return 0;
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
		wave6_vpu_ctrl_wait_done(dev, entity);

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

int wave6_vpu_ctrl_wait_done(struct device *dev, struct wave6_vpu_entity *entity)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);
	int ret;
	u32 val;

	if (!ctrl)
		return -EINVAL;

	if (ctrl->state == WAVE6_VPU_STATE_OFF)
		return -EINVAL;

	if (ctrl->state == WAVE6_VPU_STATE_ON)
		return 0;

	ret = read_poll_timeout(wave6_vpu_ctrl_get_state, val, val == WAVE6_VPU_STATE_ON,
				10, W6_VCPU_BOOT_TIMEOUT, false, dev);
	if (ret) {
		dev_err(ctrl->dev, "fail to wait vcpu boot done\n");
		wave6_vpu_ctrl_set_state(ctrl, WAVE6_VPU_STATE_OFF);
		return -EINVAL;
	}

	mutex_lock(&ctrl->ctrl_lock);
	wave6_vpu_ctrl_boot_done(ctrl);
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

void *wave6_vpu_ctrl_get_sram(struct device *dev, dma_addr_t *dma_addr, u32 *size)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(dev);

	if (!ctrl) {
		if (dma_addr)
			*dma_addr = 0;
		if (size)
			*size = 0;
		return NULL;
	}

	if (dma_addr)
		*dma_addr = ctrl->sram_buf.dma_addr;
	if (size)
		*size = ctrl->sram_buf.size;

	return ctrl->sram_buf.vaddr;
}
EXPORT_SYMBOL_GPL(wave6_vpu_ctrl_get_sram);

static void wave6_vpu_ctrl_init_reserved_boot_region(struct vpu_ctrl *ctrl)
{
	if (ctrl->boot_mem.size < SIZE_COMMON) {
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
	INIT_LIST_HEAD(&ctrl->entities);

#if WAVE6_ENABLE_SW_UART
	wave6_vpu_ctrl_init_loger(ctrl);
	wave6_vpu_ctrl_create_debugfs(ctrl);
#endif

	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int wave6_vpu_ctrl_remove(struct platform_device *pdev)
{
	struct vpu_ctrl *ctrl = dev_get_drvdata(&pdev->dev);

#if WAVE6_ENABLE_SW_UART
	wave6_vpu_ctrl_remove_debugfs(ctrl);
	wave6_vpu_ctrl_free_loger(ctrl);
#endif

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

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

	return 0;
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
