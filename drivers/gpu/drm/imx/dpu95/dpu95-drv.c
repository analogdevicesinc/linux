// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023 NXP
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_module.h>
#include <drm/drm_print.h>

#include "dpu95.h"
#include "dpu95-drv.h"

#define DRIVER_NAME	"imx95-dpu"

DEFINE_DRM_GEM_DMA_FOPS(dpu95_drm_driver_fops);

static struct drm_driver dpu95_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC | DRIVER_RENDER,
	DRM_GEM_DMA_DRIVER_OPS,
	.ioctls                 = imx_drm_dpu95_ioctls,
	.num_ioctls             = ARRAY_SIZE(imx_drm_dpu95_ioctls),
	.fops = &dpu95_drm_driver_fops,
	.name = DRIVER_NAME,
	.desc = "i.MX95 DPU DRM graphics",
	.date = "20230213",
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
};

static int dpu95_load(struct dpu95_drm_device *dpu_drm)
{
	int ret;

	ret = dpu95_core_init(dpu_drm);
	if (ret)
		return ret;

	ret = dpu95_kms_prepare(dpu_drm);
	if (ret)
		return ret;

	ret = dpu95_bliteng_load(dpu_drm);
	if (ret)
		return ret;

	return 0;
}

static void dpu95_unload(struct dpu95_drm_device *dpu_drm)
{
	dpu95_bliteng_unload(dpu_drm);
	dpu95_kms_unprepare(dpu_drm);
}

static int dpu95_probe(struct platform_device *pdev)
{
	struct dpu95_drm_device *dpu_drm;
	struct device *dev = &pdev->dev;
	struct drm_device *drm;
	int ret;

	dpu_drm = devm_drm_dev_alloc(dev, &dpu95_drm_driver,
				     struct dpu95_drm_device, base);
	if (IS_ERR(dpu_drm)) {
		ret = PTR_ERR(dpu_drm);
		DRM_DEV_ERROR(dev, "failed to alloc drm device: %d\n", ret);
		return ret;
	}

	drm = &dpu_drm->base;

	dev_set_drvdata(dev, drm);

	pm_runtime_enable(dev);

	ret = dpu95_load(dpu_drm);
	if (ret)
		goto disable_rpm;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto unload;

	drm_fbdev_dma_setup(drm, 0);

	return 0;
unload:
	dpu95_unload(dpu_drm);
disable_rpm:
	pm_runtime_disable(dev);

	return ret;
}

static int dpu95_remove(struct platform_device *pdev)
{
	struct drm_device *drm = dev_get_drvdata(&pdev->dev);
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(drm);

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);
	dpu95_unload(dpu_drm);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int dpu95_runtime_suspend(struct device *dev)
{
	int ret;
	struct drm_device *drm = dev_get_drvdata(dev);
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(drm);
	struct dpu95_soc *dpu = &dpu_drm->dpu_soc;

	ret = dpu95_bliteng_runtime_suspend(dpu_drm);
	if (ret) {
		dev_err(dev, "failed to runtime suspend blit engine: %d\n", ret);
		return ret;
	}

	clk_disable_unprepare(dpu->clk_ocram);
	clk_disable_unprepare(dpu->clk_apb);
	clk_disable_unprepare(dpu->clk_axi);

	return 0;
}

static int dpu95_runtime_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(drm);
	struct dpu95_soc *dpu = &dpu_drm->dpu_soc;
	int ret;

	ret = clk_prepare_enable(dpu->clk_axi);
	if (ret) {
		dev_err(dev, "failed to enable AXI clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(dpu->clk_apb);
	if (ret) {
		dev_err(dev, "failed to enable APB clock: %d\n", ret);
		clk_disable_unprepare(dpu->clk_axi);
		return ret;
	}

	ret = clk_prepare_enable(dpu->clk_ocram);
	if (ret) {
		dev_err(dev, "failed to enable OCRAM clock: %d\n", ret);
		clk_disable_unprepare(dpu->clk_axi);
		clk_disable_unprepare(dpu->clk_apb);
		return ret;
	}

	ret = dpu95_set_qos(dpu);
	if (ret) {
		clk_disable_unprepare(dpu->clk_ocram);
		clk_disable_unprepare(dpu->clk_apb);
		clk_disable_unprepare(dpu->clk_axi);
		return ret;
	}

	dpu95_irq_hw_init(dpu);

	dpu95_submodules_hw_init(dpu);

	ret = dpu95_bliteng_runtime_resume(dpu_drm);
	if (ret) {
		dev_err(dev, "failed to runtime resume blit engine: %d\n", ret);
		return ret;
	}

	return 0;
}

static int dpu95_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	int ret;

	ret = drm_mode_config_helper_suspend(drm_dev);
	if (ret)
		return ret;

	if (pm_runtime_active(dev))
		dpu95_runtime_suspend(dev);

	return 0;
}

static int dpu95_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	if (pm_runtime_active(dev))
		dpu95_runtime_resume(dev);

	return drm_mode_config_helper_resume(drm_dev);
}

static const struct dev_pm_ops dpu95_pm_ops = {
	RUNTIME_PM_OPS(dpu95_runtime_suspend, dpu95_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(dpu95_suspend, dpu95_resume)
};

static const struct of_device_id dpu95_dt_ids[] = {
	{ .compatible = "nxp,imx95-dpu", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dpu95_dt_ids);

static struct platform_driver dpu95_platform_driver = {
	.probe = dpu95_probe,
	.remove = dpu95_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table	= dpu95_dt_ids,
		.pm = pm_sleep_ptr(&dpu95_pm_ops),
	},
};

drm_module_platform_driver(dpu95_platform_driver);

MODULE_DESCRIPTION("i.MX95 DPU DRM Driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
