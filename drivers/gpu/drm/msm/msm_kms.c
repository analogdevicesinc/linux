// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2018, 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include <linux/kthread.h>
#include <linux/sched/mm.h>
#include <uapi/linux/sched/types.h>

#include <drm/drm_aperture.h>
#include <drm/drm_drv.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_vblank.h>

#include "disp/msm_disp_snapshot.h"
#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_kms.h"
#include "msm_mmu.h"

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = msm_framebuffer_create,
	.atomic_check = msm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs mode_config_helper_funcs = {
	.atomic_commit_tail = msm_atomic_commit_tail,
};

static irqreturn_t msm_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	BUG_ON(!kms);

	return kms->funcs->irq(kms);
}

static void msm_irq_preinstall(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	BUG_ON(!kms);

	kms->funcs->irq_preinstall(kms);
}

static int msm_irq_postinstall(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	BUG_ON(!kms);

	if (kms->funcs->irq_postinstall)
		return kms->funcs->irq_postinstall(kms);

	return 0;
}

static int msm_irq_install(struct drm_device *dev, unsigned int irq)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int ret;

	if (irq == IRQ_NOTCONNECTED)
		return -ENOTCONN;

	msm_irq_preinstall(dev);

	ret = request_irq(irq, msm_irq, 0, dev->driver->name, dev);
	if (ret)
		return ret;

	kms->irq_requested = true;

	ret = msm_irq_postinstall(dev);
	if (ret) {
		free_irq(irq, dev);
		return ret;
	}

	return 0;
}

static void msm_irq_uninstall(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	kms->funcs->irq_uninstall(kms);
	if (kms->irq_requested)
		free_irq(kms->irq, dev);
}

struct msm_vblank_work {
	struct work_struct work;
	struct drm_crtc *crtc;
	bool enable;
	struct msm_drm_private *priv;
};

static void vblank_ctrl_worker(struct work_struct *work)
{
	struct msm_vblank_work *vbl_work = container_of(work,
						struct msm_vblank_work, work);
	struct msm_drm_private *priv = vbl_work->priv;
	struct msm_kms *kms = priv->kms;

	if (vbl_work->enable)
		kms->funcs->enable_vblank(kms, vbl_work->crtc);
	else
		kms->funcs->disable_vblank(kms,	vbl_work->crtc);

	kfree(vbl_work);
}

static int vblank_ctrl_queue_work(struct msm_drm_private *priv,
				  struct drm_crtc *crtc, bool enable)
{
	struct msm_vblank_work *vbl_work;

	vbl_work = kzalloc(sizeof(*vbl_work), GFP_ATOMIC);
	if (!vbl_work)
		return -ENOMEM;

	INIT_WORK(&vbl_work->work, vblank_ctrl_worker);

	vbl_work->crtc = crtc;
	vbl_work->enable = enable;
	vbl_work->priv = priv;

	queue_work(priv->wq, &vbl_work->work);

	return 0;
}

int msm_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	if (!kms)
		return -ENXIO;
	drm_dbg_vbl(dev, "crtc=%u\n", crtc->base.id);
	return vblank_ctrl_queue_work(priv, crtc, true);
}

void msm_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	if (!kms)
		return;
	drm_dbg_vbl(dev, "crtc=%u\n", crtc->base.id);
	vblank_ctrl_queue_work(priv, crtc, false);
}

struct msm_gem_address_space *msm_kms_init_aspace(struct drm_device *dev)
{
	struct msm_gem_address_space *aspace;
	struct msm_mmu *mmu;
	struct device *mdp_dev = dev->dev;
	struct device *mdss_dev = mdp_dev->parent;
	struct device *iommu_dev;

	/*
	 * IOMMUs can be a part of MDSS device tree binding, or the
	 * MDP/DPU device.
	 */
	if (device_iommu_mapped(mdp_dev))
		iommu_dev = mdp_dev;
	else
		iommu_dev = mdss_dev;

	mmu = msm_iommu_new(iommu_dev, 0);
	if (IS_ERR(mmu))
		return ERR_CAST(mmu);

	if (!mmu) {
		drm_info(dev, "no IOMMU, fallback to phys contig buffers for scanout\n");
		return NULL;
	}

	aspace = msm_gem_address_space_create(mmu, "mdp_kms",
		0x1000, 0x100000000 - 0x1000);
	if (IS_ERR(aspace)) {
		dev_err(mdp_dev, "aspace create, error %pe\n", aspace);
		mmu->funcs->destroy(mmu);
	}

	return aspace;
}

void msm_drm_kms_uninit(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct drm_device *ddev = priv->dev;
	struct msm_kms *kms = priv->kms;
	int i;

	BUG_ON(!kms);

	/* clean up event worker threads */
	for (i = 0; i < priv->num_crtcs; i++) {
		if (priv->event_thread[i].worker)
			kthread_destroy_worker(priv->event_thread[i].worker);
	}

	drm_kms_helper_poll_fini(ddev);

	msm_disp_snapshot_destroy(ddev);

	pm_runtime_get_sync(dev);
	msm_irq_uninstall(ddev);
	pm_runtime_put_sync(dev);

	if (kms && kms->funcs)
		kms->funcs->destroy(kms);
}

int msm_drm_kms_init(struct device *dev, const struct drm_driver *drv)
{
	struct msm_drm_private *priv = dev_get_drvdata(dev);
	struct drm_device *ddev = priv->dev;
	struct msm_kms *kms = priv->kms;
	struct drm_crtc *crtc;
	int ret;

	/* the fw fb could be anywhere in memory */
	ret = drm_aperture_remove_framebuffers(drv);
	if (ret)
		return ret;

	ret = priv->kms_init(ddev);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to load kms\n");
		priv->kms = NULL;
		return ret;
	}

	/* Enable normalization of plane zpos */
	ddev->mode_config.normalize_zpos = true;

	ddev->mode_config.funcs = &mode_config_funcs;
	ddev->mode_config.helper_private = &mode_config_helper_funcs;

	kms->dev = ddev;
	ret = kms->funcs->hw_init(kms);
	if (ret) {
		DRM_DEV_ERROR(dev, "kms hw init failed: %d\n", ret);
		goto err_msm_uninit;
	}

	drm_helper_move_panel_connectors_to_head(ddev);

	drm_for_each_crtc(crtc, ddev) {
		struct msm_drm_thread *ev_thread;

		/* initialize event thread */
		ev_thread = &priv->event_thread[drm_crtc_index(crtc)];
		ev_thread->dev = ddev;
		ev_thread->worker = kthread_create_worker(0, "crtc_event:%d", crtc->base.id);
		if (IS_ERR(ev_thread->worker)) {
			ret = PTR_ERR(ev_thread->worker);
			DRM_DEV_ERROR(dev, "failed to create crtc_event kthread\n");
			ev_thread->worker = NULL;
			goto err_msm_uninit;
		}

		sched_set_fifo(ev_thread->worker->task);
	}

	ret = drm_vblank_init(ddev, priv->num_crtcs);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to initialize vblank\n");
		goto err_msm_uninit;
	}

	pm_runtime_get_sync(dev);
	ret = msm_irq_install(ddev, kms->irq);
	pm_runtime_put_sync(dev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to install IRQ handler\n");
		goto err_msm_uninit;
	}

	ret = msm_disp_snapshot_init(ddev);
	if (ret)
		DRM_DEV_ERROR(dev, "msm_disp_snapshot_init failed ret = %d\n", ret);

	drm_mode_config_reset(ddev);

	return 0;

err_msm_uninit:
	return ret;
}

int msm_kms_pm_prepare(struct device *dev)
{
	struct msm_drm_private *priv = dev_get_drvdata(dev);
	struct drm_device *ddev = priv ? priv->dev : NULL;

	if (!priv || !priv->kms)
		return 0;

	return drm_mode_config_helper_suspend(ddev);
}

void msm_kms_pm_complete(struct device *dev)
{
	struct msm_drm_private *priv = dev_get_drvdata(dev);
	struct drm_device *ddev = priv ? priv->dev : NULL;

	if (!priv || !priv->kms)
		return;

	drm_mode_config_helper_resume(ddev);
}

void msm_kms_shutdown(struct platform_device *pdev)
{
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct drm_device *drm = priv ? priv->dev : NULL;

	/*
	 * Shutdown the hw if we're far enough along where things might be on.
	 * If we run this too early, we'll end up panicking in any variety of
	 * places. Since we don't register the drm device until late in
	 * msm_drm_init, drm_dev->registered is used as an indicator that the
	 * shutdown will be successful.
	 */
	if (drm && drm->registered && priv->kms)
		drm_atomic_helper_shutdown(drm);
}
