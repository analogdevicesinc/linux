#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_address.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>

#include "analog_drm_drv.h"
#include "analog_drm_crtc.h"
#include "analog_drm_encoder.h"
#include "analog_drm_fbdev.h"
#include "analog_drm_gem.h"

#define DRIVER_NAME	"analog_drm"
#define DRIVER_DESC	"ANALOG DRM"
#define DRIVER_DATE	"20110530"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static struct platform_device *analog_drm_pdev;
static struct i2c_adapter *slave_adapter;
static struct i2c_adapter *ddc_adapter;

static int analog_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct analog_drm_private *private;
	struct of_phandle_args dma_spec;
	struct device_node *of_node;
	int ret;

	ret = of_parse_phandle_with_args(analog_drm_pdev->dev.of_node, "dma-request", "#dma-cells",
					 0, &dma_spec);
	if (ret) {
		return ret;
	}

	private = kzalloc(sizeof(struct analog_drm_private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	dev->dev_private = (void *)private;

	private->dma_params.of_node = dma_spec.np;
	private->dma_params.chan_id = dma_spec.args[0];

	drm_mode_config_init(dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	analog_drm_mode_config_init(dev);

	private->crtc = analog_drm_crtc_create(dev);
	if (IS_ERR(private->crtc)) {
		ret = PTR_ERR(private->crtc);
		goto err_crtc;
	}

	of_node = dev->platformdev->dev.of_node;
	private->base = of_iomap(of_node, 0);
	private->base_clock = of_iomap(of_node, 1);

	private->ddc_adapter = ddc_adapter;
	private->slave_adapter = slave_adapter;

	analog_drm_encoder_create(dev);
	ret = analog_drm_fbdev_init(dev);
	if (ret) {
		DRM_ERROR("failed to initialize drm fbdev\n");
		goto err_drm_device;
	}

	return 0;

err_drm_device:
/*	analog_drm_device_unregister(dev);*/
err_crtc:
	drm_mode_config_cleanup(dev);
	kfree(private);

	return ret;
}

static int analog_drm_unload(struct drm_device *dev)
{
	analog_drm_fbdev_fini(dev);
	/*analog_drm_device_unregister(dev);*/
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	kfree(dev->dev_private);

	return 0;
}

static void analog_drm_lastclose(struct drm_device *dev)
{
	analog_drm_fbdev_restore_mode(dev);
}

static struct vm_operations_struct analog_drm_gem_vm_ops = {
	.fault = analog_drm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations analog_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= analog_drm_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.release	= drm_release,
};

static struct drm_driver analog_drm_driver = {
	.driver_features	= DRIVER_BUS_PLATFORM |
				  DRIVER_MODESET | DRIVER_GEM,
	.load			= analog_drm_load,
	.unload			= analog_drm_unload,
	.lastclose		= analog_drm_lastclose,
	.gem_free_object	= analog_drm_gem_free_object,
	.gem_vm_ops		= &analog_drm_gem_vm_ops,
	.dumb_create		= analog_drm_gem_dumb_create,
	.dumb_map_offset	= analog_drm_gem_dumb_map_offset,
	.dumb_destroy		= analog_drm_gem_dumb_destroy,
	.fops = &analog_drm_driver_fops,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

static void drm_register_work(struct work_struct *work)
{
	drm_platform_init(&analog_drm_driver, analog_drm_pdev);
}

static DECLARE_WORK(register_work, drm_register_work);

static int analog_drm_attach_i2c_adapter(struct device *dev, void *data)
{
	struct device_node *of_node;

	if (dev->type != &i2c_adapter_type)
		return 0;

	if (analog_drm_pdev == NULL || dev->of_node == NULL)
		return 0;

	if (/*ddc_adapter && */slave_adapter)
		return 0;

	of_node = analog_drm_pdev->dev.of_node;


/*	if (dev->of_node == of_parse_phandle(of_node, "ddc_adapter", 0))
		ddc_adapter = to_i2c_adapter(dev);
	else*/ if (dev->of_node == of_parse_phandle(of_node, "slave_adapter", 0))
		slave_adapter = to_i2c_adapter(dev);

	if (/*ddc_adapter && */slave_adapter)
		schedule_work(&register_work);

	return 0;
}

static int i2cdev_notifier_call(struct notifier_block *nb, unsigned long action,
			 void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		analog_drm_attach_i2c_adapter(dev, NULL);
	}

	return 0;
}

static struct notifier_block i2cdev_notifier = {
	.notifier_call = i2cdev_notifier_call,
};

static int analog_drm_platform_probe(struct platform_device *pdev)
{
	analog_drm_pdev = pdev;
	bus_register_notifier(&i2c_bus_type, &i2cdev_notifier);
	i2c_for_each_dev(NULL, analog_drm_attach_i2c_adapter);

	return 0;
}

static int analog_drm_platform_remove(struct platform_device *pdev)
{
	drm_platform_exit(&analog_drm_driver, pdev);
	return 0;
}
static const struct of_device_id adv7511_encoder_of_match[] __devinitconst = {
	{ .compatible = "adi,cf-adv7x11-core-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, adv7511_encoder_of_match);

static struct platform_driver adv7511_encoder_driver = {
	.driver = {
		.name = "cf-adv7x11-core",
		.owner = THIS_MODULE,
		.of_match_table = adv7511_encoder_of_match,
	},
	.probe = analog_drm_platform_probe,
	.remove = __devexit_p(analog_drm_platform_remove),
};
module_platform_driver(adv7511_encoder_driver);
