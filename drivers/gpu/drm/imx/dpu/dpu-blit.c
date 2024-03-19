/*
 * Copyright 2017,2021-2023 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <drm/drm_vblank.h>
#include <drm/drm_print.h>
#include <drm/drm_drv.h>
#include <drm/drm_ioctl.h>
#include <drm/imx_drm.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <video/dpu.h>

#include "imx-drm.h"

struct imx_drm_dpu_bliteng {
	struct dpu_bliteng *dpu_be;
	struct list_head list;
};

static DEFINE_MUTEX(imx_drm_dpu_bliteng_lock);
static LIST_HEAD(imx_drm_dpu_bliteng_list);

static int imx_dpu_num;

int dpu_be_get(struct dpu_bliteng *dpu_be);
void dpu_be_put(struct dpu_bliteng *dpu_be);
s32 dpu_bliteng_get_id(struct dpu_bliteng *dpu_be);
void dpu_be_configure_prefetch(struct dpu_bliteng *dpu_be,
                   u32 width, u32 height,
                   u32 x_offset, u32 y_offset,
                   u32 stride, u32 format, u64 modifier,
                   u64 baddr, u64 uv_addr);
u32 *dpu_bliteng_get_cmd_list(struct dpu_bliteng *dpu_be);
void dpu_be_wait(struct dpu_bliteng *dpu_be);
int dpu_bliteng_get_empty_instance(struct dpu_bliteng **dpu_be,
    struct device *dev);
void dpu_bliteng_set_id(struct dpu_bliteng *dpu_be, int id);
void dpu_bliteng_set_dev(struct dpu_bliteng *dpu_be, struct device *dev);
int dpu_bliteng_init(struct dpu_bliteng *dpu_bliteng);
void dpu_bliteng_fini(struct dpu_bliteng *dpu_bliteng);
int dpu_be_blit(struct dpu_bliteng *dpu_be,
    u32 *cmdlist, u32 cmdnum);
int dpu_be_get_fence(struct dpu_bliteng *dpu_be, int dpu_num);
int dpu_be_set_fence(struct dpu_bliteng *dpu_be, int fd);

static struct imx_drm_dpu_bliteng *imx_drm_dpu_bliteng_find_by_id(s32 id)
{
	struct imx_drm_dpu_bliteng *bliteng;

	mutex_lock(&imx_drm_dpu_bliteng_lock);

	list_for_each_entry(bliteng, &imx_drm_dpu_bliteng_list, list) {
		if (id == dpu_bliteng_get_id(bliteng->dpu_be)) {
			mutex_unlock(&imx_drm_dpu_bliteng_lock);
			return bliteng;
		}
	}

	mutex_unlock(&imx_drm_dpu_bliteng_lock);

	return NULL;
}

static int imx_drm_dpu_set_cmdlist_ioctl(struct drm_device *drm_dev, void *data,
					  struct drm_file *file)
{
	struct drm_imx_dpu_set_cmdlist *req;
	struct imx_drm_dpu_bliteng *bliteng;
	struct dpu_bliteng *dpu_be;
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

	if (id != 0 && id != 1)
		return -EINVAL;

	user_data += sizeof(id);
	if (copy_from_user(&frame_info, (void __user *)user_data,
		sizeof(frame_info))) {
		return -EFAULT;
	}

	bliteng = imx_drm_dpu_bliteng_find_by_id(id);
	if (!bliteng) {
		DRM_ERROR("Failed to get dpu_bliteng\n");
		return -ENODEV;
	}

	dpu_be = bliteng->dpu_be;

	ret = dpu_be_get(dpu_be);

	cmd_nr = req->cmd_nr;
	cmd = (u32 *)(unsigned long)req->cmd;
	cmd_list = dpu_bliteng_get_cmd_list(dpu_be);

	if (copy_from_user(cmd_list, (void __user *)cmd,
			sizeof(*cmd) * cmd_nr)) {
		ret = -EFAULT;
		goto err;
	}

	dpu_be_configure_prefetch(dpu_be, frame_info.width, frame_info.height,
				  frame_info.x_offset, frame_info.y_offset,
				  frame_info.stride, frame_info.format,
				  frame_info.modifier, frame_info.baddr,
				  frame_info.uv_addr);

	ret = dpu_be_blit(dpu_be, cmd_list, cmd_nr);

err:
	dpu_be_put(dpu_be);

	return ret;
}

static int imx_drm_dpu_wait_ioctl(struct drm_device *drm_dev, void *data,
				  struct drm_file *file)
{
	struct drm_imx_dpu_wait *wait;
	struct imx_drm_dpu_bliteng *bliteng;
	struct dpu_bliteng *dpu_be;
	void *user_data;
	s32 id = 0;
	int ret;

	wait = data;
	user_data = (void *)(unsigned long)wait->user_data;
	if (copy_from_user(&id, (void __user *)user_data,
		sizeof(id))) {
		return -EFAULT;
	}

	if (id != 0 && id != 1)
		return -EINVAL;

	bliteng = imx_drm_dpu_bliteng_find_by_id(id);
	if (!bliteng) {
		DRM_ERROR("Failed to get dpu_bliteng\n");
		return -ENODEV;
	}

	dpu_be = bliteng->dpu_be;

	ret = dpu_be_get(dpu_be);

	dpu_be_wait(dpu_be);

	dpu_be_put(dpu_be);

	return ret;
}

static int imx_drm_dpu_get_param_ioctl(struct drm_device *drm_dev, void *data,
				       struct drm_file *file)
{
	enum drm_imx_dpu_param *param = data;
	struct imx_drm_dpu_bliteng *bliteng;
	struct dpu_bliteng *dpu_be;
	int ret, id, fd = -1;

	switch (*param) {
	case (DRM_IMX_MAX_DPUS):
		ret = imx_dpu_num;
		break;
	case DRM_IMX_GET_FENCE:
		for (id = 0; id < imx_dpu_num; id++) {
			bliteng = imx_drm_dpu_bliteng_find_by_id(id);
			if (!bliteng) {
				DRM_ERROR("Failed to get dpu_bliteng\n");
				return -ENODEV;
			}

			dpu_be = bliteng->dpu_be;
			ret = dpu_be_get(dpu_be);

			if (fd == -1)
				fd = dpu_be_get_fence(dpu_be, imx_dpu_num);

			dpu_be_set_fence(dpu_be, fd);
			dpu_be_put(dpu_be);
		}
		ret = fd;
		break;
	default:
		ret = -EINVAL;
		DRM_ERROR("Unknown param![%d]\n", *param);
		break;
	}

	return ret;
}

static int imx_drm_dpu_sync_dmabuf_ioctl(struct drm_device *drm_dev, void *data,
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

const struct drm_ioctl_desc imx_drm_dpu_ioctls[4] = {
	DRM_IOCTL_DEF_DRV(IMX_DPU_SET_CMDLIST, imx_drm_dpu_set_cmdlist_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_WAIT, imx_drm_dpu_wait_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_GET_PARAM, imx_drm_dpu_get_param_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_SYNC_DMABUF, imx_drm_dpu_sync_dmabuf_ioctl,
			DRM_RENDER_ALLOW),
};
EXPORT_SYMBOL_GPL(imx_drm_dpu_ioctls);

static int dpu_bliteng_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct imx_drm_dpu_bliteng *bliteng;
	struct dpu_bliteng *dpu_bliteng = NULL;
	int ret;

	bliteng = devm_kzalloc(dev, sizeof(*bliteng), GFP_KERNEL);
	if (!bliteng)
		return -ENOMEM;

	INIT_LIST_HEAD(&bliteng->list);

	ret = dpu_bliteng_get_empty_instance(&dpu_bliteng, dev);
	if (ret)
		return ret;

	dpu_bliteng_set_id(dpu_bliteng, imx_dpu_num);
	dpu_bliteng_set_dev(dpu_bliteng, dev);

	ret = dpu_bliteng_init(dpu_bliteng);
	if (ret)
		return ret;

	mutex_lock(&imx_drm_dpu_bliteng_lock);
	bliteng->dpu_be = dpu_bliteng;
	list_add_tail(&bliteng->list, &imx_drm_dpu_bliteng_list);
	mutex_unlock(&imx_drm_dpu_bliteng_lock);

	dev_set_drvdata(dev, dpu_bliteng);

	imx_dpu_num++;

	return 0;
}

static void dpu_bliteng_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct imx_drm_dpu_bliteng *bliteng;
	struct dpu_bliteng *dpu_bliteng = dev_get_drvdata(dev);
	s32 id = dpu_bliteng_get_id(dpu_bliteng);

	bliteng = imx_drm_dpu_bliteng_find_by_id(id);
	list_del(&bliteng->list);

	dpu_bliteng_fini(dpu_bliteng);
	dev_set_drvdata(dev, NULL);

	imx_dpu_num--;
}

static const struct component_ops dpu_bliteng_ops = {
	.bind = dpu_bliteng_bind,
	.unbind = dpu_bliteng_unbind,
};

static int dpu_bliteng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!dev->platform_data)
		return -EINVAL;

	return component_add(dev, &dpu_bliteng_ops);
}

static int dpu_bliteng_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dpu_bliteng_ops);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dpu_bliteng_suspend(struct device *dev)
{
	struct dpu_bliteng *dpu_bliteng = dev_get_drvdata(dev);
	int ret;

	if (dpu_bliteng == NULL)
		return 0;

	ret = dpu_be_get(dpu_bliteng);

	dpu_be_wait(dpu_bliteng);

	dpu_be_put(dpu_bliteng);

	dpu_bliteng_fini(dpu_bliteng);

	return 0;
}

static int dpu_bliteng_resume(struct device *dev)
{
	struct dpu_bliteng *dpu_bliteng = dev_get_drvdata(dev);

	if (dpu_bliteng != NULL)
		dpu_bliteng_init(dpu_bliteng);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(dpu_bliteng_pm_ops,
			 dpu_bliteng_suspend, dpu_bliteng_resume);

struct platform_driver dpu_bliteng_driver = {
	.driver = {
		.name = "imx-drm-dpu-bliteng",
		.pm = &dpu_bliteng_pm_ops,
	},
	.probe = dpu_bliteng_probe,
	.remove = dpu_bliteng_remove,
};

module_platform_driver(dpu_bliteng_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX DRM DPU BLITENG");
MODULE_IMPORT_NS(DMA_BUF);
