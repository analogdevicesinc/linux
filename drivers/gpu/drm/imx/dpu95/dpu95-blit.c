// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023 NXP
 */

#include <drm/drm_drv.h>
#include <drm/drm_ioctl.h>
#include <drm/imx_drm.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "dpu95-blit-registers.h"
#include "dpu95.h"
#include "dpu95-drv.h"

static struct dpu_bliteng *dpu_blit_eng;
static int imx_dpu_num;

static inline u32 dpu95_be_read(struct dpu_bliteng *dpu_be, unsigned int offset)
{
	return readl(dpu_be->base + offset);
}

static inline void dpu95_be_write(struct dpu_bliteng *dpu_be, u32 value,
	unsigned int offset)
{
	writel(value, dpu_be->base + offset);
}

static void dpu95_cs_wait_fifo_space(struct dpu_bliteng *dpu_be)
{
	while ((dpu95_be_read(dpu_be, CMDSEQ_STATUS) &
		CMDSEQ_STATUS_FIFOSPACE_MASK) < CMDSEQ_FIFO_SPACE_THRESHOLD)
		usleep_range(30, 50);
}

static void dpu95_cs_wait_idle(struct dpu_bliteng *dpu_be)
{
	while ((dpu95_be_read(dpu_be, CMDSEQ_STATUS) &
		CMDSEQ_STATUS_IDLE_MASK) == 0x0)
		mdelay(1);
}

static int dpu95_cs_alloc_command_buffer(struct dpu_bliteng *dpu_be)
{
	/* command buffer need 32 bit address */
	dpu_be->buffer_addr_virt =
		alloc_pages_exact(COMMAND_BUFFER_SIZE,
			GFP_KERNEL | GFP_DMA | GFP_DMA32 | __GFP_ZERO);
	if (!dpu_be->buffer_addr_virt) {
		dev_err(dpu_be->dev, "memory alloc failed for dpu command buffer\n");
		return -ENOMEM;
	}

	dpu_be->buffer_addr_phy =
		(u32)virt_to_phys(dpu_be->buffer_addr_virt);

	return 0;
}

static void dpu95_cs_static_setup(struct dpu_bliteng *dpu_be)
{
	dpu95_cs_wait_idle(dpu_be);

	/* LockUnlock and LockUnlockHIF */
	dpu95_be_write(dpu_be, CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__UNLOCK_KEY,
		CMDSEQ_LOCKUNLOCKHIF);
	dpu95_be_write(dpu_be, CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__UNLOCK_KEY,
		CMDSEQ_LOCKUNLOCK);

	/* Control */
	dpu95_be_write(dpu_be, 1 << CMDSEQ_CONTROL_CLEAR_SHIFT,
		CMDSEQ_CONTROL);

	/* BufferAddress and BufferSize */
	dpu95_be_write(dpu_be, dpu_be->buffer_addr_phy, CMDSEQ_BUFFERADDRESS);
	dpu95_be_write(dpu_be, COMMAND_BUFFER_SIZE / WORD_SIZE,
		CMDSEQ_BUFFERSIZE);
}

static u32 *dpu95_bliteng_get_cmd_list(struct dpu_bliteng *dpu_be)
{
	return dpu_be->cmd_list;
}

static void dpu95_bliteng_set_dev(struct dpu_bliteng *dpu_be, struct device *dev)
{
	dpu_be->dev = dev;
}

static int dpu95_be_get(struct dpu_bliteng *dpu_be)
{
	mutex_lock(&dpu_be->mutex);

	return 0;
}

static void dpu95_be_put(struct dpu_bliteng *dpu_be)
{
	mutex_unlock(&dpu_be->mutex);
}

static int dpu95_be_blit(struct dpu_bliteng *dpu_be,
	u32 *cmdlist, u32 cmdnum)
{
	int i;

	if (cmdnum > CMDSEQ_FIFO_SPACE_THRESHOLD) {
		dev_err(dpu_be->dev, "dpu blit cmdnum[%d] should be less than %d !\n",
			cmdnum, CMDSEQ_FIFO_SPACE_THRESHOLD);
		return -EINVAL;
	}
	dpu95_cs_wait_fifo_space(dpu_be);

	for (i = 0; i < cmdnum; i++)
		dpu95_be_write(dpu_be, cmdlist[i], CMDSEQ_HIF);

	return 0;
}

#define STORE9_SEQCOMPLETE_IRQ         2U
#define STORE9_SEQCOMPLETE_IRQ_MASK    (1U<<STORE9_SEQCOMPLETE_IRQ)

static void dpu95_be_wait(struct dpu_bliteng *dpu_be)
{
	dpu95_cs_wait_fifo_space(dpu_be);

	dpu95_be_write(dpu_be, 0x14000001, CMDSEQ_HIF);
	dpu95_be_write(dpu_be, PIXENGCFG_STORE9_TRIGGER, CMDSEQ_HIF);
	dpu95_be_write(dpu_be, 0x10, CMDSEQ_HIF);

	while ((dpu95_be_read(dpu_be, COMCTRL_INTERRUPTSTATUS0) &
		STORE9_SEQCOMPLETE_IRQ_MASK) == 0)
		usleep_range(30, 50);

	dpu95_be_write(dpu_be, STORE9_SEQCOMPLETE_IRQ_MASK,
		COMCTRL_INTERRUPTCLEAR0);
}

static void dpu95_be_init_units(struct dpu_bliteng *dpu_be)
{
	u32 staticcontrol;
	u32 pixengcfg_unit_dynamic;
	u32 store9_static;

	staticcontrol =
	1 << FETCHDECODE9_STATICCONTROL_SHDEN_SHIFT |
	FETCHDECODE9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, FETCHDECODE9_STATICCONTROL);

	staticcontrol =
	1 << FETCHROT9_STATICCONTROL_SHDEN_SHIFT |
	0 << FETCHROT9_STATICCONTROL_BASEADDRESSSELECT_SHIFT |
	FETCHROT9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, FETCHROT9_STATICCONTROL);

	staticcontrol =
	1 << FETCHECO9_STATICCONTROL_SHDEN_SHIFT |
	FETCHECO9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, FETCHECO9_STATICCONTROL);

	staticcontrol =
	1 << HSCALER9_STATICCONTROL_SHDEN_SHIFT |
	HSCALER9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, HSCALER9_STATICCONTROL);

	staticcontrol =
	1 << VSCALER9_STATICCONTROL_SHDEN_SHIFT |
	VSCALER9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, VSCALER9_STATICCONTROL);

	staticcontrol =
	1 << ROP9_STATICCONTROL_SHDEN_SHIFT |
	ROP9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, ROP9_STATICCONTROL);

	staticcontrol =
	1 << MATRIX9_STATICCONTROL_SHDEN_SHIFT |
	MATRIX9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, MATRIX9_STATICCONTROL);

	staticcontrol =
	1 << BLITBLEND9_STATICCONTROL_SHDEN_SHIFT |
	BLITBLEND9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, BLITBLEND9_STATICCONTROL);

	staticcontrol =
	1 << STORE9_STATICCONTROL_SHDEN_SHIFT |
	0 << STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT |
	STORE9_STATICCONTROL_RESET_VALUE;
	dpu95_be_write(dpu_be, staticcontrol, STORE9_STATICCONTROL);

	/* Safety_Pixengcfg Dynamic */
	pixengcfg_unit_dynamic =
	PIXENGCFG_FETCHDECODE9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_FETCHDECODE9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_FETCHROT9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_FETCHROT9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_ROP9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_ROP9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_MATRIX9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_MATRIX9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_HSCALER9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_HSCALER9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_VSCALER9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_VSCALER9_DYNAMIC);

	pixengcfg_unit_dynamic =
	PIXENGCFG_CLKEN__AUTOMATIC << PIXENGCFG_CLKEN_SHIFT |
	PIXENGCFG_BLITBLEND9_DYNAMIC_RESET_VALUE;
	dpu95_be_write(dpu_be, pixengcfg_unit_dynamic,
		PIXENGCFG_BLITBLEND9_DYNAMIC);

	store9_static = dpu95_be_read(dpu_be, PIXENGCFG_STORE9_STATIC);
	store9_static &= ~PIXENGCFG_STORE9_STATIC_STORE9_POWERDOWN_MASK;
	store9_static |= PIXENGCFG_STORE9_STATIC_STORE9_SHDEN_MASK;
	dpu95_be_write(dpu_be, store9_static, PIXENGCFG_STORE9_STATIC);
}

static int dpu95_bliteng_init(struct dpu_bliteng *dpu_bliteng)
{
	struct dpu95_drm_device *dpu_drm =
		container_of(dpu_bliteng, struct dpu95_drm_device, dpu_be);
	struct dpu95_soc *dpu = &dpu_drm->dpu_soc;
	struct platform_device *dpu_pdev = to_platform_device(dpu->dev);
	struct resource *res;
	unsigned long dpu_base;
	void __iomem *base;
	u32 *cmd_list;
	int ret;

	cmd_list = kzalloc(sizeof(*cmd_list) * CMDSEQ_FIFO_SPACE_THRESHOLD,
			GFP_KERNEL);
	if (!cmd_list)
		return -ENOMEM;
	dpu_bliteng->cmd_list = cmd_list;

	res = platform_get_resource(dpu_pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	dpu_base = res->start;

	/* remap with bigger size */
	base = devm_ioremap(dpu->dev, dpu_base, 4096*SZ_1K);
	dpu_bliteng->base = base;
	dpu_bliteng->dpu = dpu;

	mutex_init(&dpu_bliteng->mutex);

	/* Init the uints used by blit engine */
	dpu95_be_init_units(dpu_bliteng);

	/* Init for command sequencer */
	ret = dpu95_cs_alloc_command_buffer(dpu_bliteng);
	if (ret)
		return ret;

	dpu95_cs_static_setup(dpu_bliteng);

	return 0;
}

static void dpu95_bliteng_fini(struct dpu_bliteng *dpu_bliteng)
{
	/* LockUnlock and LockUnlockHIF */
	dpu95_be_write(dpu_bliteng, CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__LOCK_KEY,
		CMDSEQ_LOCKUNLOCKHIF);
	dpu95_be_write(dpu_bliteng, CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__LOCK_KEY,
		CMDSEQ_LOCKUNLOCK);

	kfree(dpu_bliteng->cmd_list);

	if (dpu_bliteng->buffer_addr_virt)
		free_pages_exact(dpu_bliteng->buffer_addr_virt,
				 COMMAND_BUFFER_SIZE);
}

static int imx_drm_dpu95_set_cmdlist_ioctl(struct drm_device *drm_dev, void *data,
					  struct drm_file *file)
{
	struct drm_imx_dpu_set_cmdlist *req;
	u32 cmd_nr, *cmd, *cmd_list;
	void *user_data;
	s32 id = 0;
	struct drm_imx_dpu_frame_info frame_info;
	int ret;

	req = data;
	user_data = (void *)(unsigned long)req->user_data;
	if (copy_from_user(&id, (void __user *)user_data,
		sizeof(id))) {
		return -EFAULT;
	}

	if (id != 0)
		return -EINVAL;

	user_data += sizeof(id);
	if (copy_from_user(&frame_info, (void __user *)user_data,
		sizeof(frame_info))) {
		return -EFAULT;
	}

	ret = dpu95_be_get(dpu_blit_eng);
	ret = pm_runtime_resume_and_get(dpu_blit_eng->dev);
	if (ret < 0) {
		drm_err(drm_dev, "failed to get device RPM: %d\n", ret);
		return ret;
	}

	cmd_nr = req->cmd_nr;
	cmd = (u32 *)(unsigned long)req->cmd;
	cmd_list = dpu95_bliteng_get_cmd_list(dpu_blit_eng);

	if (copy_from_user(cmd_list, (void __user *)cmd,
			sizeof(*cmd) * cmd_nr)) {
		ret = -EFAULT;
		goto err;
	}

	ret = dpu95_be_blit(dpu_blit_eng, cmd_list, cmd_nr);

err:
	dpu95_be_put(dpu_blit_eng);
	pm_runtime_put_autosuspend(dpu_blit_eng->dev);

	return ret;
}

static int imx_drm_dpu95_wait_ioctl(struct drm_device *drm_dev, void *data,
				  struct drm_file *file)
{
	struct drm_imx_dpu_wait *wait;
	void *user_data;
	s32 id = 0;
	int ret;

	wait = data;
	user_data = (void *)(unsigned long)wait->user_data;
	if (copy_from_user(&id, (void __user *)user_data,
		sizeof(id))) {
		return -EFAULT;
	}

	if (id != 0)
		return -EINVAL;

	ret = dpu95_be_get(dpu_blit_eng);
	ret = pm_runtime_resume_and_get(dpu_blit_eng->dev);
	if (ret < 0) {
		drm_err(drm_dev, "failed to get device RPM: %d\n", ret);
		return ret;
	}

	dpu95_be_wait(dpu_blit_eng);

	dpu95_be_put(dpu_blit_eng);
	pm_runtime_put_autosuspend(dpu_blit_eng->dev);

	return ret;
}

static int imx_drm_dpu95_get_param_ioctl(struct drm_device *drm_dev, void *data,
				       struct drm_file *file)
{
	enum drm_imx_dpu_param *param = data;
	int ret;

	switch (*param) {
	case (DRM_IMX_MAX_DPUS):
		ret = imx_dpu_num;
		break;
	default:
		ret = -EINVAL;
		DRM_ERROR("Unknown param![%d]\n", *param);
		break;
	}

	return ret;
}

static int imx_drm_dpu95_sync_dmabuf_ioctl(struct drm_device *drm_dev, void *data,
					  struct drm_file *file)
{
	struct drm_imx_dpu_sync_dmabuf *flush = data;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	int direction;
	int ret = 0;

	if (flush->direction == IMX_DPU_SYNC_TO_BOTH)
		direction = DMA_BIDIRECTIONAL;
	else if (flush->direction == IMX_DPU_SYNC_TO_DEVICE)
		direction = DMA_TO_DEVICE;
	else if (flush->direction == IMX_DPU_SYNC_TO_CPU)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_NONE;

	dmabuf = dma_buf_get(flush->dmabuf_fd);
	if (IS_ERR(dmabuf)) {
		drm_err(drm_dev, "failed to get dmabuf\n");
		return PTR_ERR(dmabuf);
	}
	attachment = dma_buf_attach(dmabuf, drm_dev->dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		drm_err(drm_dev, "failed to attach dmabuf\n");
		goto err_put;
	}
	sgt = dma_buf_map_attachment(attachment, direction);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		drm_err(drm_dev, "failed to get dmabuf sg_table\n");
		goto err_detach;
	}
	dma_buf_unmap_attachment(attachment, sgt, direction);
err_detach:
	dma_buf_detach(dmabuf, attachment);
err_put:
	dma_buf_put(dmabuf);

	return ret;
}

const struct drm_ioctl_desc imx_drm_dpu95_ioctls[4] = {
	DRM_IOCTL_DEF_DRV(IMX_DPU_SET_CMDLIST, imx_drm_dpu95_set_cmdlist_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_WAIT, imx_drm_dpu95_wait_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_GET_PARAM, imx_drm_dpu95_get_param_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_SYNC_DMABUF, imx_drm_dpu95_sync_dmabuf_ioctl,
			DRM_RENDER_ALLOW),
};

int dpu95_bliteng_load(struct dpu95_drm_device *dpu_drm)
{
	struct drm_device *drm = &dpu_drm->base;
	struct dpu_bliteng *dpu_bliteng = &dpu_drm->dpu_be;
	int ret;

	dpu95_bliteng_set_dev(dpu_bliteng, drm->dev);

	ret = dpu95_bliteng_init(dpu_bliteng);
	if (ret)
		return ret;

	dpu_blit_eng = dpu_bliteng;

	imx_dpu_num++;

	return 0;
}

void dpu95_bliteng_unload(struct dpu95_drm_device *dpu_drm)
{
	struct dpu_bliteng *dpu_bliteng = &dpu_drm->dpu_be;

	dpu95_bliteng_fini(dpu_bliteng);

	dpu_blit_eng = NULL;

	imx_dpu_num--;
}

int dpu95_bliteng_runtime_suspend(struct dpu95_drm_device *dpu_drm)
{
	int ret;
	struct dpu_bliteng *dpu_bliteng = &dpu_drm->dpu_be;

	if (!dpu_bliteng)
		return 0;

	ret = dpu95_be_get(dpu_bliteng);

	dpu95_be_wait(dpu_bliteng);

	dpu95_be_put(dpu_bliteng);

	/* LockUnlock and LockUnlockHIF */
	dpu95_be_write(dpu_bliteng, CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__LOCK_KEY,
		CMDSEQ_LOCKUNLOCKHIF);
	dpu95_be_write(dpu_bliteng, CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__LOCK_KEY,
		CMDSEQ_LOCKUNLOCK);

	return 0;
}

int dpu95_bliteng_runtime_resume(struct dpu95_drm_device *dpu_drm)
{
	struct dpu_bliteng *dpu_bliteng = &dpu_drm->dpu_be;

	if (!dpu_bliteng)
		return 0;

	dpu95_be_init_units(dpu_bliteng);

	dpu95_cs_static_setup(dpu_bliteng);

	return 0;
}
