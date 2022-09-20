// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020,2021 NXP
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "dcnano-drv.h"
#include "dcnano-reg.h"

#define DRIVER_NAME     "imx-dcnano-drm"

static int legacyfb_depth = 32;
module_param(legacyfb_depth, uint, 0444);

DEFINE_DRM_GEM_DMA_FOPS(dcnano_driver_fops);

static struct drm_driver dcnano_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	DRM_GEM_DMA_DRIVER_OPS,
	.fops			= &dcnano_driver_fops,
	.name			= "imx-dcnano",
	.desc			= "i.MX DCNANO DRM graphics",
	.date			= "20201221",
	.major			= 1,
	.minor			= 0,
	.patchlevel		= 0,
};

static int dcnano_reset(struct dcnano_dev *dcnano)
{
	struct drm_device *drm = &dcnano->base;
	int ret;

	pm_runtime_get_sync(drm->dev);

	ret = reset_control_assert(dcnano->tied_resets);
	if (ret) {
		DRM_DEV_ERROR(drm->dev,
			      "failed to assert tied resets: %d\n", ret);
		goto err;
	}

	/*
	 * 10 microseconds are enough for the 32-cycle(slowest clock)
	 * assertion duration.
	 */
	usleep_range(10, 20);

	ret = reset_control_deassert(dcnano->tied_resets);
	if (ret) {
		DRM_DEV_ERROR(drm->dev,
			      "failed to deassert tied resets: %d\n", ret);
		goto err;
	}

	/*
	 * 40 microseconds are enough for the 128-cycle(slowest clock)
	 * de-assertion duration.
	 */
	usleep_range(40, 50);

err:
	pm_runtime_put_sync(drm->dev);
	return ret;
}

static int dcnano_irq_install(struct drm_device *dev, int irq)
{
	if (irq == IRQ_NOTCONNECTED)
		return -ENOTCONN;

	return request_irq(irq, dcnano_irq_handler, 0, dev->driver->name, dev);
}

static void dcnano_irq_uninstall(struct drm_device *dev)
{
	struct dcnano_dev *dcnano = to_dcnano_dev(dev);

	free_irq(dcnano->irq, dev);
}

static int dcnano_check_chip_info(struct dcnano_dev *dcnano)
{
	struct drm_device *drm = &dcnano->base;
	u32 val;
	int ret = 0;

	pm_runtime_get_sync(drm->dev);

	val = dcnano_read(dcnano, DCNANO_DCCHIPREV);
	if (val != DCCHIPREV) {
		DRM_DEV_ERROR(drm->dev, "invalid chip revision(0x%08x)\n", val);
		ret = -ENODEV;
		goto err;
	}
	DRM_DEV_DEBUG(drm->dev, "chip revision is 0x%08x\n", val);

	val = dcnano_read(dcnano, DCNANO_DCCHIPDATE);
	if (val != DCCHIPDATE) {
		DRM_DEV_ERROR(drm->dev, "invalid chip date(0x%08x)\n", val);
		ret = -ENODEV;
		goto err;
	}
	DRM_DEV_DEBUG(drm->dev, "chip date is 0x%08x\n", val);

	val = dcnano_read(dcnano, DCNANO_DCCHIPPATCHREV);
	if (val != DCCHIPPATCHREV) {
		DRM_DEV_ERROR(drm->dev,
			      "invalid chip patch revision(0x%08x)\n", val);
		ret = -ENODEV;
		goto err;
	}
	DRM_DEV_DEBUG(drm->dev, "chip patch revision is 0x%08x\n", val);
err:
	pm_runtime_put_sync(drm->dev);
	return ret;
}

static int dcnano_probe(struct platform_device *pdev)
{
	struct dcnano_dev *dcnano;
	struct drm_device *drm;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	dcnano = devm_drm_dev_alloc(&pdev->dev, &dcnano_driver,
				    struct dcnano_dev, base);
	if (IS_ERR(dcnano))
		return PTR_ERR(dcnano);

	drm = &dcnano->base;
	dev_set_drvdata(&pdev->dev, dcnano);

	dcnano->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dcnano->mmio_base))
		return PTR_ERR(dcnano->mmio_base);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	dcnano->irq = ret;

	dcnano->axi_clk = devm_clk_get(drm->dev, "axi");
	if (IS_ERR(dcnano->axi_clk)) {
		ret = PTR_ERR(dcnano->axi_clk);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(drm->dev,
				      "failed to get axi clk: %d\n", ret);
		return ret;
	}

	dcnano->ahb_clk = devm_clk_get(drm->dev, "ahb");
	if (IS_ERR(dcnano->ahb_clk)) {
		ret = PTR_ERR(dcnano->ahb_clk);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(drm->dev,
				      "failed to get ahb clk: %d\n", ret);
		return ret;
	}

	dcnano->pixel_clk = devm_clk_get(drm->dev, "pixel");
	if (IS_ERR(dcnano->pixel_clk)) {
		ret = PTR_ERR(dcnano->pixel_clk);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(drm->dev,
				      "failed to get pixel clk: %d\n", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret) {
		DRM_DEV_ERROR(drm->dev,
			      "failed to set dma mask and coherent: %d\n", ret);
		return ret;
	}

	pm_runtime_enable(drm->dev);

	dcnano->tied_resets = devm_reset_control_get(drm->dev, NULL);
	if (IS_ERR(dcnano->tied_resets)) {
		ret = PTR_ERR(dcnano->tied_resets);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(drm->dev,
				      "failed to get tied resets: %d\n", ret);
		goto err_reset_get;
	}

	ret = dcnano_reset(dcnano);
	if (ret)
		goto err_dcnano_reset;

	pm_runtime_get_sync(drm->dev);
	ret = dcnano_irq_install(drm, dcnano->irq);
	pm_runtime_put_sync(drm->dev);

	if (ret < 0) {
		DRM_DEV_ERROR(drm->dev,
			      "failed to install IRQ handler: %d\n", ret);
		goto err_irq_install;
	}

	ret = dcnano_check_chip_info(dcnano);
	if (ret)
		goto err_check_chip_info;

	ret = dcnano_kms_prepare(dcnano);
	if (ret)
		goto err_kms_prepare;

	ret = drm_dev_register(drm, 0);
	if (ret) {
		DRM_DEV_ERROR(drm->dev,
			      "failed to register drm device: %d\n", ret);
		goto err_register;
	}

	if (legacyfb_depth != 16 && legacyfb_depth != 32) {
		DRM_DEV_INFO(drm->dev,
			     "Invalid legacyfb_depth.  Defaulting to 32bpp\n");
		legacyfb_depth = 32;
	}

	drm_fbdev_dma_setup(drm, legacyfb_depth);

	return 0;

err_register:
	drm_kms_helper_poll_fini(drm);
err_kms_prepare:
err_check_chip_info:
	pm_runtime_get_sync(drm->dev);
	dcnano_irq_uninstall(drm);
	pm_runtime_put_sync(drm->dev);
err_irq_install:
err_dcnano_reset:
err_reset_get:
	pm_runtime_disable(drm->dev);
	return ret;
}

static void dcnano_remove(struct platform_device *pdev)
{
	struct dcnano_dev *dcnano = dev_get_drvdata(&pdev->dev);
	struct drm_device *drm = &dcnano->base;

	drm_dev_unregister(drm);

	drm_kms_helper_poll_fini(drm);

	drm_atomic_helper_shutdown(drm);

	pm_runtime_get_sync(drm->dev);
	dcnano_irq_uninstall(drm);
	pm_runtime_put_sync(drm->dev);

	pm_runtime_disable(drm->dev);
}

static int __maybe_unused dcnano_suspend(struct device *dev)
{
	struct dcnano_dev *dcnano = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&dcnano->base);
}

static int __maybe_unused dcnano_resume(struct device *dev)
{
	struct dcnano_dev *dcnano = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&dcnano->base);
}

static int __maybe_unused dcnano_runtime_suspend(struct device *dev)
{
	struct dcnano_dev *dcnano = dev_get_drvdata(dev);

	drm_dbg(&dcnano->base, "runtime suspend\n");

	clk_disable_unprepare(dcnano->pixel_clk);
	clk_disable_unprepare(dcnano->ahb_clk);
	clk_disable_unprepare(dcnano->axi_clk);

	return 0;
}

static int __maybe_unused dcnano_runtime_resume(struct device *dev)
{
	struct dcnano_dev *dcnano = dev_get_drvdata(dev);
	int ret;

	drm_dbg(&dcnano->base, "runtime resume\n");

	ret = clk_prepare_enable(dcnano->axi_clk);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to enable axi clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(dcnano->ahb_clk);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to enable ahb clock: %d\n", ret);
		clk_disable_unprepare(dcnano->axi_clk);
		return ret;
	}

	/*
	 * Pixel clock has to be enabled for like DCNANO in i.MX8ulp,
	 * otherwise registers cannot be accessed.
	 */
	ret = clk_prepare_enable(dcnano->pixel_clk);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to enable pixel clock: %d\n", ret);
		clk_disable_unprepare(dcnano->axi_clk);
		clk_disable_unprepare(dcnano->ahb_clk);
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops dcnano_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dcnano_suspend, dcnano_resume)
	SET_RUNTIME_PM_OPS(dcnano_runtime_suspend, dcnano_runtime_resume, NULL)
};

static const struct of_device_id dcnano_dt_ids[] = {
	{ .compatible = "nxp,imx8ulp-dcnano", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dcnano_dt_ids);

static struct platform_driver dcnano_platform_driver = {
	.probe	= dcnano_probe,
	.remove	= dcnano_remove,
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= dcnano_dt_ids,
		.pm		= &dcnano_pm_ops,
	},
};
module_platform_driver(dcnano_platform_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX DCNANO DRM driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL v2");
