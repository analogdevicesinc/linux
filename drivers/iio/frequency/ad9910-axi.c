// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 AXI Backend driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/units.h>

#include <linux/iio/backend.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>

#include <linux/iio/frequency/ad9910.h>

/* Register addresses */
#define AD9910_AXI_REG_RESET		0x040
#define AD9910_AXI_REG_DRG		0x084
#define AD9910_AXI_REG_PROFILE		0x088
#define AD9910_AXI_REG_IO_UPDATE	0x08C
#define AD9910_AXI_REG_RAMP_DELAY_BST	0x094
#define AD9910_AXI_REG_RAMP_DELAY_ALR	0x098
#define AD9910_AXI_REG_BURST_DELAY	0x09C
#define AD9910_AXI_REG_BURST_COUNT	0x0A0
#define AD9910_AXI_REG_UPDATE_CTRL	0x104
#define AD9910_AXI_REG_PAR_RATE		0x108
#define AD9910_AXI_REG_DMA_CFG		0x10C

#define AD9910_AXI_REG_RESET_CORE_MSK		BIT(0)
#define AD9910_AXI_REG_RESET_IO_MSK		BIT(1)
#define AD9910_AXI_REG_RESET_DEV_MSK		BIT(2)
#define AD9910_AXI_REG_RESET_PW_DOWN_MSK	BIT(3)

#define AD9910_AXI_REG_DRG_DRCTL_MSK		BIT(2)
#define AD9910_AXI_REG_DRG_TOGGLE_EN_MSK	BIT(3)
#define AD9910_AXI_REG_DRG_OPER_MODE_MSK	GENMASK(5, 4)

#define AD9910_AXI_REG_UPDATE_RATE_MSK		BIT(0)
#define AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK	BIT(1)

#define AD9910_AXI_REG_PROFILE_MSK		GENMASK(2, 0)
#define AD9910_AXI_REG_IO_UPDATE_MSK		BIT(0)
#define AD9910_AXI_REG_DMA_CFG_MSK		GENMASK(1, 0)

#define AD9910_AXI_PD_CLK_DEFAULT_FREQ_HZ	(250 * HZ_PER_MHZ)
#define AD9910_AXI_PAR_RATE_MAX			U32_MAX

#define AD9910_AXI_RAMP_DELAY_OFFSET		2
#define AD9910_AXI_BURST_DELAY_OFFSET		3

enum {
	AD9910_AXI_PP_SAMPLE_RATE,
	AD9910_AXI_DRG_CONTROL_EN,
	AD9910_AXI_DRG_TOGGLE_EN,
	AD9910_AXI_DRG_RAMP_DELAY,
	AD9910_AXI_DRG_BURST_COUNT,
	AD9910_AXI_DRG_BURST_DELAY,
};

struct ad9910_axi_state {
	struct reset_controller_dev rc;
	struct device *dev;
	struct regmap *regmap;
	/*
	 * lock to protect multiple accesses to the device registers and global
	 * data/variables.
	 */
	struct mutex lock;
	u32 pd_clk_freq_hz;
};

#define ad9910_axi_from_rst_ctrl(st)	\
	container_of(st, struct ad9910_axi_state, rc)

static int ad9910_axi_chan_enable(struct iio_backend *back, unsigned int chan)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	switch (chan) {
	case AD9910_CHANNEL_PARALLEL_PORT:
		return regmap_set_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
				       AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_chan_disable(struct iio_backend *back, unsigned int chan)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	switch (chan) {
	case AD9910_CHANNEL_PARALLEL_PORT:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
					 AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK);
	default:
		return -EINVAL;
	}
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
	int val, val2, ret;
	u32 tmp32;
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_AXI_PP_SAMPLE_RATE:
		ret = iio_str_to_fixpoint(buf, MICRO/10, &val, &val2);
		if (ret)
			return ret;

		tmp64 = (u64)val * MICRO + val2;
		if (!tmp64)
			return -EINVAL;

		tmp64 = DIV64_U64_ROUND_CLOSEST((u64)st->pd_clk_freq_hz * MICRO,
						 tmp64);
		tmp32 = clamp(tmp64, 1U, AD9910_AXI_PAR_RATE_MAX);
		ret = regmap_write(st->regmap, AD9910_AXI_REG_PAR_RATE, tmp32 - 1);
		if (ret)
			return ret;

		ret = regmap_set_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
				      AD9910_AXI_REG_UPDATE_RATE_MSK);
		break;
	case AD9910_AXI_DRG_CONTROL_EN:
		ret = kstrtou32(buf, 10, &tmp32);
		if (ret)
			return ret;

		tmp32 = tmp32 ? AD9910_AXI_REG_DRG_DRCTL_MSK : 0;
		ret = regmap_update_bits(st->regmap, AD9910_AXI_REG_DRG,
					 AD9910_AXI_REG_DRG_DRCTL_MSK, tmp32);
		break;
	case AD9910_AXI_DRG_TOGGLE_EN:
		ret = kstrtou32(buf, 10, &tmp32);
		if (ret)
			return ret;

		tmp32 = tmp32 ? AD9910_AXI_REG_DRG_TOGGLE_EN_MSK : 0;
		ret = regmap_update_bits(st->regmap, AD9910_AXI_REG_DRG,
					 AD9910_AXI_REG_DRG_TOGGLE_EN_MSK, tmp32);
		break;
	case AD9910_AXI_DRG_RAMP_DELAY:
		ret = iio_str_to_fixpoint(buf, NANO / 10, &val, &val2);
		if (ret)
			return ret;

		tmp64 = (u64)val * NANO + val2;
		tmp64 = mul_u64_u32_div(tmp64, st->pd_clk_freq_hz, NANO);
		tmp64 = max(tmp64, AD9910_AXI_RAMP_DELAY_OFFSET);
		tmp64 -= AD9910_AXI_RAMP_DELAY_OFFSET;
		tmp32 = min(tmp64, U32_MAX);
		ret = regmap_write(st->regmap, AD9910_AXI_REG_RAMP_DELAY_ALR, tmp32);
		break;
	case AD9910_AXI_DRG_BURST_COUNT:
		ret = kstrtou32(buf, 10, &tmp32);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, AD9910_AXI_REG_BURST_COUNT, tmp32);
		break;
	case AD9910_AXI_DRG_BURST_DELAY:
		ret = iio_str_to_fixpoint(buf, NANO / 10, &val, &val2);
		if (ret)
			return ret;

		tmp64 = (u64)val * NANO + val2;
		tmp64 = mul_u64_u32_div(tmp64, st->pd_clk_freq_hz, NANO);
		tmp64 = max(tmp64, AD9910_AXI_BURST_DELAY_OFFSET);
		tmp64 -= AD9910_AXI_BURST_DELAY_OFFSET;
		tmp32 = min(tmp64, U32_MAX);
		ret = regmap_write(st->regmap, AD9910_AXI_REG_BURST_DELAY, tmp32);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

static int ad9910_axi_ext_info_get(struct iio_backend *back, uintptr_t private,
				   const struct iio_chan_spec *chan, char *buf)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	int vals[2], ret;
	u32 tmp32;
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_AXI_PP_SAMPLE_RATE:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_PAR_RATE, &tmp32);
		if (ret)
			return ret;

		tmp32++;
		vals[0] = st->pd_clk_freq_hz / tmp32;
		vals[1] = div_u64((u64)(st->pd_clk_freq_hz % tmp32) * MICRO, tmp32);

		return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals), vals);
	case AD9910_AXI_DRG_CONTROL_EN:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_DRG, &tmp32);
		if (ret)
			return ret;

		tmp32 = FIELD_GET(AD9910_AXI_REG_DRG_DRCTL_MSK, tmp32);
		return sysfs_emit(buf, "%u\n", tmp32);
	case AD9910_AXI_DRG_TOGGLE_EN:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_DRG, &tmp32);
		if (ret)
			return ret;

		tmp32 = FIELD_GET(AD9910_AXI_REG_DRG_TOGGLE_EN_MSK, tmp32);
		return sysfs_emit(buf, "%u\n", tmp32);
	case AD9910_AXI_DRG_RAMP_DELAY:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_RAMP_DELAY_ALR, &tmp32);
		if (ret)
			return ret;

		tmp64 = (u64)tmp32 + AD9910_AXI_RAMP_DELAY_OFFSET;
		tmp64 = div_u64(tmp64 * NANO, st->pd_clk_freq_hz);
		vals[0] = div_u64_rem(tmp64, NANO, &tmp32);
		vals[1] = tmp32;
		return iio_format_value(buf, IIO_VAL_INT_PLUS_NANO, ARRAY_SIZE(vals), vals);
	case AD9910_AXI_DRG_BURST_COUNT:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_BURST_COUNT, &tmp32);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n", tmp32);
	case AD9910_AXI_DRG_BURST_DELAY:
		ret = regmap_read(st->regmap, AD9910_AXI_REG_BURST_DELAY, &tmp32);
		if (ret)
			return ret;

		tmp64 = (u64)tmp32 + AD9910_AXI_BURST_DELAY_OFFSET;
		tmp64 = div_u64(tmp64 * NANO, st->pd_clk_freq_hz);
		vals[0] = div_u64_rem(tmp64, NANO, &tmp32);
		vals[1] = tmp32;
		return iio_format_value(buf, IIO_VAL_INT_PLUS_NANO, ARRAY_SIZE(vals), vals);
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec_ext_info ad9910_axi_pp_ext_info[] = {
	/*
	 * Even though sampling_frequency is a standard IIO channel attribute,
	 * we define it as an extended attribute here since the backend
	 * channel extension mechanism is designed for extended attributes only.
	 */
	IIO_BACKEND_EX_INFO("sampling_frequency", IIO_SEPARATE, AD9910_AXI_PP_SAMPLE_RATE),
	{ }
};

static const struct iio_chan_spec_ext_info ad9910_axi_drg_ext_info[] = {
	IIO_BACKEND_EX_INFO("control_en", IIO_SEPARATE, AD9910_AXI_DRG_CONTROL_EN),
	IIO_BACKEND_EX_INFO("toggle_en", IIO_SEPARATE, AD9910_AXI_DRG_TOGGLE_EN),
	IIO_BACKEND_EX_INFO("ramp_delay", IIO_SEPARATE, AD9910_AXI_DRG_RAMP_DELAY),
	IIO_BACKEND_EX_INFO("burst_count", IIO_SEPARATE, AD9910_AXI_DRG_BURST_COUNT),
	IIO_BACKEND_EX_INFO("burst_delay", IIO_SEPARATE, AD9910_AXI_DRG_BURST_DELAY),
	{ }
};

static int ad9910_axi_chan_spec(struct iio_backend *back,
				struct iio_chan_spec *chan)
{
	if (chan->channel == AD9910_CHANNEL_DRG) {
		chan->ext_info = ad9910_axi_drg_ext_info;
	} else if (chan->channel == AD9910_CHANNEL_PARALLEL_PORT) {
		chan->ext_info = ad9910_axi_pp_ext_info;
	}

	return 0;
}

static int ad9910_axi_set_sample_rate(struct iio_backend *back,
				      unsigned int chan,
				      u64 sample_rate)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	switch (chan) {
	case AD9910_CHANNEL_PARALLEL_PORT:
		st->pd_clk_freq_hz = sample_rate;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_reg_access(struct iio_backend *back, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad9910_axi_scan_type_get(struct iio_backend *back,
				    const struct iio_chan_spec *chan)
{
	if (chan->channel != AD9910_CHANNEL_PARALLEL_PORT)
		return -EINVAL;

	return AD9910_PP_SCAN_TYPE_FULL;
}

static const struct iio_backend_ops ad9910_axi_iio_back_ops = {
	.chan_enable = ad9910_axi_chan_enable,
	.chan_disable = ad9910_axi_chan_disable,
	.request_buffer = ad9910_axi_request_buffer,
	.free_buffer = ad9910_axi_free_buffer,
	.extend_chan_spec = ad9910_axi_chan_spec,
	.ext_info_set = ad9910_axi_ext_info_set,
	.ext_info_get = ad9910_axi_ext_info_get,
	.set_sample_rate = ad9910_axi_set_sample_rate,
	.debugfs_reg_access = iio_backend_debugfs_ptr(ad9910_axi_reg_access),
	.scan_type_get = ad9910_axi_scan_type_get,
};

static int ad9910_axi_profile_set(struct ad9910_backend *back,
				  unsigned int profile)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, AD9910_AXI_REG_PROFILE,
				  AD9910_AXI_REG_PROFILE_MSK,
				  FIELD_PREP(AD9910_AXI_REG_PROFILE_MSK, profile));
}

static int ad9910_axi_io_update(struct ad9910_backend *back)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	guard(mutex)(&st->lock);

	return regmap_set_bits(st->regmap, AD9910_AXI_REG_IO_UPDATE,
			       AD9910_AXI_REG_IO_UPDATE_MSK);
}

static int ad9910_axi_drg_oper_mode_set(struct ad9910_backend *back,
					enum ad9910_drg_oper_mode mode)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, AD9910_AXI_REG_DRG,
				  AD9910_AXI_REG_DRG_OPER_MODE_MSK,
				  FIELD_PREP(AD9910_AXI_REG_DRG_OPER_MODE_MSK, mode));
}

static int ad9910_axi_powerdown_set(struct ad9910_backend *back,
				    unsigned int enable)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back->iio_back);

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, AD9910_AXI_REG_RESET,
				  AD9910_AXI_REG_RESET_PW_DOWN_MSK,
				  enable ? AD9910_AXI_REG_RESET_PW_DOWN_MSK : 0);
}

static const struct ad9910_backend_ops ad9910_axi_back_ops = {
	.profile_set = ad9910_axi_profile_set,
	.io_update = ad9910_axi_io_update,
	.drg_oper_mode_set = ad9910_axi_drg_oper_mode_set,
	.powerdown_set = ad9910_axi_powerdown_set,
};

static const struct ad9910_backend_info ad9910_axi_back_info = {
	.base = {
		.name = "ad9910-axi",
		.ops = &ad9910_axi_iio_back_ops,
	},
	.ops = &ad9910_axi_back_ops,
};

static int ad9910_axi_reset_assert(struct reset_controller_dev *rc,
				   unsigned long id)
{
	struct ad9910_axi_state *st = ad9910_axi_from_rst_ctrl(rc);

	guard(mutex)(&st->lock);

	switch (id) {
	case AD9910_RESET_CTRL_DEVICE:
		return regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
				       AD9910_AXI_REG_RESET_DEV_MSK);
	case AD9910_RESET_CTRL_IO:
		return regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
				       AD9910_AXI_REG_RESET_IO_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_reset_deassert(struct reset_controller_dev *rc,
				     unsigned long id)
{
	struct ad9910_axi_state *st = ad9910_axi_from_rst_ctrl(rc);

	guard(mutex)(&st->lock);

	switch (id) {
	case AD9910_RESET_CTRL_DEVICE:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
					 AD9910_AXI_REG_RESET_DEV_MSK);
	case AD9910_RESET_CTRL_IO:
		return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
					 AD9910_AXI_REG_RESET_IO_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_reset_status(struct reset_controller_dev *rc,
				   unsigned long id)
{
	struct ad9910_axi_state *st = ad9910_axi_from_rst_ctrl(rc);
	int ret;
	u32 tmp32;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AD9910_AXI_REG_RESET, &tmp32);
	if (ret)
		return ret;

	switch (id)
	{
	case AD9910_RESET_CTRL_DEVICE:
		return FIELD_GET(AD9910_AXI_REG_RESET_DEV_MSK, tmp32);
	case AD9910_RESET_CTRL_IO:
		return FIELD_GET(AD9910_AXI_REG_RESET_IO_MSK, tmp32);
	default:
		return -EINVAL;
	}
}

static const struct reset_control_ops ad9910_axi_reset_ops = {
	.assert = ad9910_axi_reset_assert,
	.deassert = ad9910_axi_reset_deassert,
	.status = ad9910_axi_reset_status,
};

static const struct regmap_config ad9910_axi_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int ad9910_axi_setup(struct ad9910_axi_state *st)
{
	int ret;

	st->pd_clk_freq_hz = AD9910_AXI_PD_CLK_DEFAULT_FREQ_HZ;

	ret = regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
			      AD9910_AXI_REG_RESET_CORE_MSK);
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->regmap, AD9910_AXI_REG_DMA_CFG,
				AD9910_AXI_REG_DMA_CFG_MSK);
	if (ret)
		return ret;

	return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
				 AD9910_AXI_REG_RESET_CORE_MSK);
}

static int ad9910_axi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad9910_axi_state *st;
	void __iomem *base;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->dev = dev;
	st->regmap = devm_regmap_init_mmio(dev, base,
					   &ad9910_axi_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to init register map\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->rc.ops = &ad9910_axi_reset_ops;
	st->rc.owner = THIS_MODULE;
	st->rc.of_node = dev->of_node;
	st->rc.nr_resets = AD9910_RESET_CTRL_MAX;

	ret = devm_reset_controller_register(dev, &st->rc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register reset controller\n");

	ret = ad9910_axi_setup(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup device\n");

	return devm_ad9910_backend_register(dev, &ad9910_axi_back_info, st);
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
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
MODULE_IMPORT_NS(IIO_BACKEND);
MODULE_IMPORT_NS(AD9910);
