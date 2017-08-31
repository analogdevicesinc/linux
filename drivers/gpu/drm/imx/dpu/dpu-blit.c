/*
 * Copyright 2017 NXP
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

#include <drm/drmP.h>
#include <drm/imx_drm.h>
#include <linux/component.h>
#include <linux/device.h>
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
	int ret;

	req = data;
	user_data = (void *)(unsigned long)req->user_data;
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

retry:
	ret = dpu_be_get(dpu_be);
	if (ret == -EBUSY)
		goto retry;

	cmd_nr = req->cmd_nr;
	cmd = (u32 *)(unsigned long)req->cmd;
	cmd_list = dpu_bliteng_get_cmd_list(dpu_be);

	if (copy_from_user(cmd_list, (void __user *)cmd,
			sizeof(*cmd) * cmd_nr)) {
		ret = -EFAULT;
		goto err;
	}

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

retry:
	ret = dpu_be_get(dpu_be);
	if (ret == -EBUSY)
		goto retry;

	dpu_be_wait(dpu_be);

	dpu_be_put(dpu_be);

	return ret;
}

static int imx_drm_dpu_get_param_ioctl(struct drm_device *drm_dev, void *data,
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

static struct drm_ioctl_desc imx_drm_dpu_ioctls[] = {
	DRM_IOCTL_DEF_DRV(IMX_DPU_SET_CMDLIST, imx_drm_dpu_set_cmdlist_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_WAIT, imx_drm_dpu_wait_ioctl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(IMX_DPU_GET_PARAM, imx_drm_dpu_get_param_ioctl,
			DRM_RENDER_ALLOW),
};

static int dpu_bliteng_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct drm_device *drm = (struct drm_device *)data;
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

	if (drm->driver->num_ioctls == 0) {
		drm->driver->ioctls = imx_drm_dpu_ioctls;
		drm->driver->num_ioctls = ARRAY_SIZE(imx_drm_dpu_ioctls);
	}

	return 0;
}

static void dpu_bliteng_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct drm_device *drm = (struct drm_device *)data;
	struct imx_drm_dpu_bliteng *bliteng;
	struct dpu_bliteng *dpu_bliteng = dev_get_drvdata(dev);
	s32 id = dpu_bliteng_get_id(dpu_bliteng);

	bliteng = imx_drm_dpu_bliteng_find_by_id(id);
	list_del(&bliteng->list);

	dpu_bliteng_fini(dpu_bliteng);

	imx_dpu_num--;

	if (drm->driver->num_ioctls != 0) {
		drm->driver->ioctls = NULL;
		drm->driver->num_ioctls = 0;
	}
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

struct platform_driver dpu_bliteng_driver = {
	.driver = {
		.name = "imx-drm-dpu-bliteng",
	},
	.probe = dpu_bliteng_probe,
	.remove = dpu_bliteng_remove,
};

module_platform_driver(dpu_bliteng_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX DRM DPU BLITENG");
