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
#include <linux/of_graph.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_helper.h>

#include "axi_hdmi_drv.h"

#define DRIVER_NAME	"axi_hdmi_drm"
#define DRIVER_DESC	"AXI HDMI DRM"
#define DRIVER_DATE	"20120930"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static struct drm_mode_config_funcs axi_hdmi_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.output_poll_changed = drm_fb_helper_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void axi_hdmi_mode_config_init(struct drm_device *dev)
{
	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &axi_hdmi_mode_config_funcs;
}

static int axi_hdmi_init(struct drm_driver *ddrv, struct device *dev)
{
	struct axi_hdmi_private *private = dev_get_drvdata(dev);
	struct drm_device *ddev;
	struct drm_encoder *encoder;
	int ret;

	ddev = drm_dev_alloc(ddrv, dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	private->drm_dev = ddev;

	ddev->dev_private = private;

	axi_hdmi_mode_config_init(ddev);

	private->crtc = axi_hdmi_crtc_create(ddev);
	if (IS_ERR(private->crtc)) {
		ret = PTR_ERR(private->crtc);
		goto err_crtc;
	}

	encoder = axi_hdmi_encoder_create(ddev);
	if (IS_ERR(encoder)) {
	    ret = PTR_ERR(encoder);
	    goto err_crtc;
	}

	drm_mode_config_reset(ddev);

	ret = drm_fb_cma_fbdev_init(ddev, 32, 1);
	if (ret) {
		DRM_ERROR("failed to initialize drm fbdev: %d\n", ret);
		goto err_crtc;
	}

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(ddev);

	return drm_dev_register(ddev, 0);

err_crtc:
	drm_mode_config_cleanup(ddev);
	drm_dev_unref(ddev);

	return ret;
}

static void axi_hdmi_unload(struct drm_device *dev)
{
	drm_fb_cma_fbdev_fini(dev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
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
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.unload			= axi_hdmi_unload,
	.lastclose		= drm_fb_helper_lastclose,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
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
	},
	{},
};
MODULE_DEVICE_TABLE(of, adv7511_encoder_of_match);

static int axi_hdmi_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct device_node *np = pdev->dev.of_node;
	struct axi_hdmi_private *private;
	struct device_node *slave_node, *ep_node;
	struct of_endpoint ep;
	struct resource *res;
	int ret;

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

	ep_node = of_graph_get_next_endpoint(np, NULL);
	if (ep_node) {
		ret = of_graph_parse_endpoint(ep_node, &ep);
		if (ret) {
			of_node_put(ep_node);
			return ret;
		}
		if (ep.port != 0 && ep.id != 0) {
			of_node_put(ep_node);
			return -EINVAL;
		}
		slave_node = of_graph_get_remote_port_parent(ep_node);
		of_node_put(ep_node);
	} else {
		slave_node = of_parse_phandle(np, "encoder-slave", 0);
	}

	if (!slave_node)
		return -EINVAL;

	private->is_rgb = of_property_read_bool(np, "adi,is-rgb");

	id = of_match_node(adv7511_encoder_of_match, np);

	private->encoder_slave = of_find_i2c_device_by_node(slave_node);
	of_node_put(slave_node);

	if (!private->encoder_slave || !private->encoder_slave->dev.driver)
		return -EPROBE_DEFER;

	private->dma = dma_request_slave_channel(&pdev->dev, "video");
	if (private->dma == NULL)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, private);

	return axi_hdmi_init(&axi_hdmi_driver, &pdev->dev);
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
