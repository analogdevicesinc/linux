// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 AXI Backend driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include <linux/iio/frequency/ad9910.h>

enum {
	AD9910_AXI_RAMP_DELAY,
	AD9910_AXI_BURST_COUNT,
	AD9910_AXI_BURST_DELAY,
};

struct ad9910_axi_state {
	struct device *dev;
	struct regmap *regmap;
	/*
	 * lock to protect multiple accesses to the device registers and global
	 * data/variables.
	 */
	struct mutex lock;

	u32 dummy;
};

static int ad9910_axi_enable(struct iio_backend *back)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	dev_warn(st->dev, "Enabling not supported yet\n");
	return -ENOSYS;
}

static void ad9910_axi_disable(struct iio_backend *back)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	dev_warn(st->dev, "Disabling not supported yet\n");
}

static struct iio_buffer *ad9910_axi_request_buffer(struct iio_backend *back,
						    struct iio_dev *indio_dev)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	const char *dma_name;

	if (device_property_read_string(st->dev, "dma-names", &dma_name))
		dma_name = "tx";

	return iio_dmaengine_buffer_setup_ext(st->dev, indio_dev, dma_name,
					      IIO_BUFFER_DIRECTION_OUT);
}

static void ad9910_axi_free_buffer(struct iio_backend *back,
				   struct iio_buffer *buffer)
{
	iio_dmaengine_buffer_teardown(buffer);
}

static int ad9910_axi_ext_info_set(struct iio_backend *back, uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	int ret;

	switch (private) {
	case AD9910_AXI_RAMP_DELAY:
	case AD9910_AXI_BURST_COUNT:
	case AD9910_AXI_BURST_DELAY:
		ret = kstrtou32(buf, 10, &st->dummy);
		if (ret)
			return ret;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret ?: len;
}

static int ad9910_axi_ext_info_get(struct iio_backend *back, uintptr_t private,
				   const struct iio_chan_spec *chan, char *buf)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	switch (private) {
	case AD9910_AXI_RAMP_DELAY:
	case AD9910_AXI_BURST_COUNT:
	case AD9910_AXI_BURST_DELAY:
		return sysfs_emit(buf, "%u\n", st->dummy);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct iio_chan_spec_ext_info ad9910_axi_drg_ext_info[] = {
	IIO_BACKEND_EX_INFO("ramp_delay", IIO_SEPARATE, AD9910_AXI_RAMP_DELAY),
	IIO_BACKEND_EX_INFO("burst_count", IIO_SEPARATE, AD9910_AXI_BURST_COUNT),
	IIO_BACKEND_EX_INFO("burst_delay", IIO_SEPARATE, AD9910_AXI_BURST_DELAY),
	{ }
};

static int ad9910_axi_chan_spec(struct iio_backend *back,
				struct iio_chan_spec *chan)
{
	if (chan->channel == AD9910_CHANNEL_DRG) {
		chan->ext_info = ad9910_axi_drg_ext_info;
	}

	return 0;
}

static int ad9910_axi_set_sample_rate(struct iio_backend *back,
				      unsigned int chan,
				      u64 sample_rate)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	dev_warn(st->dev, "Setting sample rate is not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_reg_access(struct iio_backend *back, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_backend_ops ad9910_axi_iio_back_ops = {
	.enable = ad9910_axi_enable,
	.disable = ad9910_axi_disable,
	.request_buffer = ad9910_axi_request_buffer,
	.free_buffer = ad9910_axi_free_buffer,
	.extend_chan_spec = ad9910_axi_chan_spec,
	.ext_info_set = ad9910_axi_ext_info_set,
	.ext_info_get = ad9910_axi_ext_info_get,
	.set_sample_rate = ad9910_axi_set_sample_rate,
	.debugfs_reg_access = iio_backend_debugfs_ptr(ad9910_axi_reg_access),
};

static int ad9910_axi_profile_set(struct ad9910_backend *back,
				  unsigned int profile)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "Setting profile not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_io_update(struct ad9910_backend *back)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "IO update not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_io_reset(struct ad9910_backend *back)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "IO reset not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_drg_control_mode_set(struct ad9910_backend *back,
					   enum ad9910_drg_control_mode mode)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "set DRG control mode not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_drg_control_mode_get(struct ad9910_backend *back,
					   enum ad9910_drg_control_mode* mode)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "get DRG control mode not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_drg_oper_mode_set(struct ad9910_backend *back,
					enum ad9910_drg_oper_mode mode)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "set DRG operating mode not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_powerdown_set(struct ad9910_backend *back,
				    unsigned int enable)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "Powerdown not supported yet\n");
	return -ENOSYS;
}

static int ad9910_axi_reset(struct ad9910_backend *back, unsigned int enable)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	dev_warn(st->dev, "Reset not supported yet\n");
	return -ENOSYS;
}

static const struct ad9910_backend_ops ad9910_axi_back_ops = {
	.profile_set = ad9910_axi_profile_set,
	.io_update = ad9910_axi_io_update,
	.io_reset = ad9910_axi_io_reset,
	.drg_control_mode_set = ad9910_axi_drg_control_mode_set,
	.drg_control_mode_get = ad9910_axi_drg_control_mode_get,
	.drg_oper_mode_set = ad9910_axi_drg_oper_mode_set,
	.powerdown_set = ad9910_axi_powerdown_set,
	.reset = ad9910_axi_reset,
};

static const struct ad9910_backend_info ad9910_axi_back_info = {
	.base = {
		.name = "ad9910-axi",
		.ops = &ad9910_axi_iio_back_ops,
	},
	.ops = &ad9910_axi_back_ops,
};

static const struct regmap_config ad9910_axi_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int ad9910_axi_probe(struct platform_device *pdev)
{
	struct ad9910_axi_state *st;
	void __iomem *base;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	/* TODO: get fclk */

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->dev = &pdev->dev;
	st->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					   &ad9910_axi_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->regmap),
				     "failed to init register map\n");

	ret = devm_mutex_init(&pdev->dev, &st->lock);
	if (ret)
		return ret;

	return devm_ad9910_backend_register(&pdev->dev, &ad9910_axi_back_info, st);
}

static const struct of_device_id ad9910_axi_of_match[] = {
	{ .compatible = "adi,axi-ad9910" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9910_axi_of_match);

static struct platform_driver ad9910_axi_driver = {
	.driver = {
		.name = "ad9910-axi",
		.of_match_table = ad9910_axi_of_match,
	},
	.probe = ad9910_axi_probe,
};
module_platform_driver(ad9910_axi_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9910 AXI Backend driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_DMAENGINE_BUFFER");
MODULE_IMPORT_NS("IIO_BACKEND");
MODULE_IMPORT_NS("AD9910");
