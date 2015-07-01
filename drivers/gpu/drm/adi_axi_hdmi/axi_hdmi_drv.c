/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "axi_hdmi_drv.h"
#include "axi_hdmi_crtc.h"
#include "axi_hdmi_encoder.h"

#define DRIVER_NAME	"axi_hdmi_drm"
#define DRIVER_DESC	"AXI HDMI DRM"
#define DRIVER_DATE	"20120930"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static void axi_hdmi_output_poll_changed(struct drm_device *dev)
{
	struct axi_hdmi_private *private = dev->dev_private;
	drm_fbdev_cma_hotplug_event(private->fbdev);
}

static struct drm_mode_config_funcs axi_hdmi_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = axi_hdmi_output_poll_changed,
};

static void axi_hdmi_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &axi_hdmi_mode_config_funcs;
}

static int axi_hdmi_load(struct drm_device *dev, unsigned long flags)
{
	struct axi_hdmi_private *private = dev_get_drvdata(dev->dev);
	struct drm_encoder *encoder;
	int ret;

	private->drm_dev = dev;

	dev->dev_private = private;

	drm_mode_config_init(dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	axi_hdmi_mode_config_init(dev);

	private->crtc = axi_hdmi_crtc_create(dev);
	if (IS_ERR(private->crtc)) {
		ret = PTR_ERR(private->crtc);
		goto err_crtc;
	}

	encoder = axi_hdmi_encoder_create(dev);
	if (IS_ERR(encoder)) {
	    ret = PTR_ERR(encoder);
	    goto err_crtc;
	}

	private->fbdev = drm_fbdev_cma_init(dev, 32, 1, 1);
	if (IS_ERR(private->fbdev)) {
		DRM_ERROR("failed to initialize drm fbdev\n");
		ret = PTR_ERR(private->fbdev);
		goto err_crtc;
	}

	return 0;

err_crtc:
	drm_mode_config_cleanup(dev);
	return ret;
}

static int axi_hdmi_unload(struct drm_device *dev)
{
	struct axi_hdmi_private *private = dev->dev_private;

	drm_fbdev_cma_fini(private->fbdev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);

	return 0;
}

static void axi_hdmi_lastclose(struct drm_device *dev)
{
	struct axi_hdmi_private *private = dev->dev_private;
	drm_fbdev_cma_restore_mode(private->fbdev);
}

static const struct file_operations axi_hdmi_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= drm_gem_cma_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.release	= drm_release,
};

static struct drm_driver axi_hdmi_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM,
	.load			= axi_hdmi_load,
	.unload			= axi_hdmi_unload,
	.set_busid		= drm_platform_set_busid,
	.lastclose		= axi_hdmi_lastclose,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.fops			= &axi_hdmi_driver_fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
};

static const struct of_device_id adv7511_encoder_of_match[] = {
	{
		.compatible = "adi,axi-hdmi-tx-1.00.a",
		.data = (const void *)AXI_HDMI
	}, {
		.compatible = "adi,axi-hdmi-1.00.a",
		.data = (const void *)AXI_HDMI_LEGACY
	},
	{},
};
MODULE_DEVICE_TABLE(of, adv7511_encoder_of_match);

static int axi_hdmi_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct device_node *np = pdev->dev.of_node;
	struct axi_hdmi_private *private;
	struct device_node *slave_node;
	struct resource *res;

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	private->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(private->base))
		return PTR_ERR(private->base);

	private->hdmi_clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(private->hdmi_clock)) {
		return -EPROBE_DEFER;
	}

	slave_node = of_parse_phandle(np, "encoder-slave", 0);
	if (!slave_node)
		return -EINVAL;

	private->is_rgb = of_property_read_bool(np, "adi,is-rgb");

	id = of_match_node(adv7511_encoder_of_match, np);
	private->version = (unsigned long)id->data;

	if (private->version == AXI_HDMI_LEGACY &&
		of_property_read_bool(np, "adi,embedded-sync"))
		private->version = AXI_HDMI_LEGACY_ES;
	
	private->encoder_slave = of_find_i2c_device_by_node(slave_node);
	of_node_put(slave_node);

	if (!private->encoder_slave || !private->encoder_slave->dev.driver)
		return -EPROBE_DEFER;

	private->dma = dma_request_slave_channel(&pdev->dev, "video");
	if (private->dma == NULL)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, private);

	return drm_platform_init(&axi_hdmi_driver, pdev);
}

static int axi_hdmi_platform_remove(struct platform_device *pdev)
{
	struct axi_hdmi_private *private = platform_get_drvdata(pdev);

	drm_put_dev(private->drm_dev);
	dma_release_channel(private->dma);
	return 0;
}

static struct platform_driver adv7511_encoder_driver = {
	.driver = {
		.name = "axi-hdmi",
		.owner = THIS_MODULE,
		.of_match_table = adv7511_encoder_of_match,
	},
	.probe = axi_hdmi_platform_probe,
	.remove = axi_hdmi_platform_remove,
};
module_platform_driver(adv7511_encoder_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("");
