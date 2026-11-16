// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 AXI Backend driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include <linux/iio/backend.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>

#include "ad9910.h"

/* Register addresses */
#define AD9910_AXI_REG_RESET		0x040
#define AD9910_AXI_REG_DRG		0x084
#define AD9910_AXI_REG_PROFILE		0x088
#define AD9910_AXI_REG_DRCTL_PERIOD	0x08C
#define AD9910_AXI_REG_DRCTL_WIDTH	0x090
#define AD9910_AXI_REG_DRCTL_DELAY	0x094
#define AD9910_AXI_REG_BURST_COUNT	0x098
#define AD9910_AXI_REG_BURST_DELAY	0x09C
#define AD9910_AXI_REG_RAMP_CFG		0x0A0

#define AD9910_AXI_REG_UPDATE_CTRL	0x104
#define AD9910_AXI_REG_PAR_RATE		0x108
#define AD9910_AXI_REG_F_CFG		0x10C

#define AD9910_AXI_REG_RESET_CORE_MSK		BIT(0)

#define AD9910_AXI_REG_DRG_DRCTL_MSK		BIT(2)
#define AD9910_AXI_REG_DRG_TOGGLE_EN_MSK	BIT(3)

#define AD9910_AXI_REG_RAMP_CFG_ONE_SHOT_MSK	GENMASK(1, 0)
#define AD9910_AXI_REG_BURST_COUNT_MSK		GENMASK(19, 0)

#define AD9910_AXI_REG_UPDATE_RATE_MSK		BIT(0)
#define AD9910_AXI_REG_UPDATE_PAR_IF_EN_MSK	BIT(1)
#define AD9910_AXI_REG_UPDATE_EXT_TRIG_MSK	BIT(2)

#define AD9910_AXI_REG_PROFILE_MSK		GENMASK(2, 0)
#define AD9910_AXI_REG_IO_UPDATE_MSK		BIT(0)
#define AD9910_AXI_REG_F_CFG_MSK		GENMASK(1, 0)

#define AD9910_AXI_F_CFG_AMP			0
#define AD9910_AXI_F_CFG_PHASE			1
#define AD9910_AXI_F_CFG_FREQ			2
#define AD9910_AXI_F_CFG_POLAR			3

#define AD9910_AXI_PD_CLK_MAX_FREQ_HZ		(250ULL * HZ_PER_MHZ)
#define AD9910_AXI_PAR_RATE_MAX			U16_MAX
#define AD9910_AXI_ONE_SHOT_BURST		1

struct ad9910_axi_state {
	struct device *dev;
	struct regmap *regmap;
	/*
	 * lock to protect multiple accesses to the device registers and global
	 * data/variables.
	 */
	struct mutex lock;
	u64 pd_clk_freq_uhz;
	/* last applied DRCTL waveform, kept when the SYNC_CLK rate changes */
	struct pwm_waveform drctl_wf;
};

/* DRCTL waveform in SYNC_CLK cycles, a zero period means disabled */
struct ad9910_axi_drctl_wfhw {
	u32 period;
	u32 width;
	u32 delay;
};

static u32 ad9910_axi_ns_to_cycles(struct ad9910_axi_state *st, u64 ns)
{
	return min(mul_u64_u64_div_u64(ns, st->pd_clk_freq_uhz,
				       1ULL * NANO * MICROHZ_PER_HZ), U32_MAX);
}

static u64 ad9910_axi_cycles_to_ns(struct ad9910_axi_state *st, u32 cycles)
{
	return mul_u64_u64_div_u64_roundup((u64)cycles * NANO, MICROHZ_PER_HZ,
					   st->pd_clk_freq_uhz);
}

static int ad9910_axi_drctl_tohw(struct ad9910_axi_state *st,
				 const struct pwm_waveform *wf,
				 struct ad9910_axi_drctl_wfhw *wfhw)
{
	if (!wf->period_length_ns) {
		*wfhw = (struct ad9910_axi_drctl_wfhw){ };
		return 0;
	}

	wfhw->period = ad9910_axi_ns_to_cycles(st, wf->period_length_ns);
	if (!wfhw->period) {
		/* round up to the shortest period, with no duty and no delay */
		*wfhw = (struct ad9910_axi_drctl_wfhw){ .period = 1 };
		return 1;
	}

	wfhw->width = min(ad9910_axi_ns_to_cycles(st, wf->duty_length_ns),
			  wfhw->period);
	/*
	 * The delay is applied once, before DRCTL starts toggling, which
	 * results in the duty offset of the periodic waveform. A delay of a
	 * whole period is the same as no delay at all.
	 */
	wfhw->delay = ad9910_axi_ns_to_cycles(st, wf->duty_offset_ns);
	if (wfhw->delay >= wfhw->period)
		wfhw->delay = 0;

	return 0;
}

static void ad9910_axi_drctl_fromhw(struct ad9910_axi_state *st,
				    const struct ad9910_axi_drctl_wfhw *wfhw,
				    struct pwm_waveform *wf)
{
	/*
	 * A zero period means the PWM output is disabled, in which case
	 * width and delay are not programmed and carry no meaning.
	 */
	if (!wfhw->period) {
		*wf = (struct pwm_waveform){ };
		return;
	}

	wf->period_length_ns = ad9910_axi_cycles_to_ns(st, wfhw->period);
	wf->duty_length_ns = ad9910_axi_cycles_to_ns(st, wfhw->width);
	wf->duty_offset_ns = ad9910_axi_cycles_to_ns(st, wfhw->delay);
}

static int ad9910_axi_drctl_write(struct ad9910_axi_state *st,
				  const struct ad9910_axi_drctl_wfhw *wfhw)
{
	int ret;

	/*
	 * DRCTL toggling restarts, applying the delay, when enabled or when
	 * its period or width changes. Changing the delay alone does not
	 * restart it, so disable toggling while updating the waveform.
	 */
	ret = regmap_clear_bits(st->regmap, AD9910_AXI_REG_DRG,
				AD9910_AXI_REG_DRG_TOGGLE_EN_MSK);
	if (ret)
		return ret;

	/* A zero period disables the PWM output, so leave toggling off */
	if (!wfhw->period)
		return 0;

	ret = regmap_write(st->regmap, AD9910_AXI_REG_DRCTL_DELAY, wfhw->delay);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD9910_AXI_REG_DRCTL_WIDTH, wfhw->width);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD9910_AXI_REG_DRCTL_PERIOD, wfhw->period);
	if (ret)
		return ret;

	/*
	 * Register values reach the SYNC_CLK domain in snapshots, taken every
	 * 64 AXI clock cycles. Make sure toggling being disabled is captured
	 * by one of them before enabling it again.
	 */
	fsleep(2);

	return regmap_set_bits(st->regmap, AD9910_AXI_REG_DRG,
			       AD9910_AXI_REG_DRG_TOGGLE_EN_MSK);
}

static int ad9910_axi_f_cfg(unsigned int chan)
{
	switch (chan) {
	case AD9910_CHAN_IDX_PARALLEL_AMP:
		return AD9910_AXI_F_CFG_AMP;
	case AD9910_CHAN_IDX_PARALLEL_PHASE:
		return AD9910_AXI_F_CFG_PHASE;
	case AD9910_CHAN_IDX_PARALLEL_FREQ:
		return AD9910_AXI_F_CFG_FREQ;
	/* polar amplitude and phase are a pair sharing the same destination */
	case AD9910_CHAN_IDX_PARALLEL_POLAR_AMP:
	case AD9910_CHAN_IDX_PARALLEL_POLAR_PHASE:
		return AD9910_AXI_F_CFG_POLAR;
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_chan_enable(struct iio_backend *back, unsigned int chan)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	int f_cfg, ret;

	guard(mutex)(&st->lock);

	switch (chan) {
	/*
	 * Profiles are mutually exclusive, as the PROFILE[2:0] pins always
	 * select one of them. Enabling a profile channel selects it, which
	 * implicitly disables the previously selected one, hence there is
	 * nothing to do when disabling a profile channel.
	 */
	case AD9910_CHAN_IDX_PROFILE_0 ... AD9910_CHAN_IDX_PROFILE_7:
		chan -= AD9910_CHAN_IDX_PROFILE_0;
		return regmap_update_bits(st->regmap, AD9910_AXI_REG_PROFILE,
					  AD9910_AXI_REG_PROFILE_MSK,
					  FIELD_PREP(AD9910_AXI_REG_PROFILE_MSK, chan));
	case AD9910_CHAN_IDX_PARALLEL_AMP ... AD9910_CHAN_IDX_PARALLEL_POLAR_PHASE:
		f_cfg = ad9910_axi_f_cfg(chan);
		if (f_cfg < 0)
			return f_cfg;

		ret = regmap_update_bits(st->regmap, AD9910_AXI_REG_F_CFG,
					 AD9910_AXI_REG_F_CFG_MSK,
					 FIELD_PREP(AD9910_AXI_REG_F_CFG_MSK, f_cfg));
		if (ret)
			return ret;

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
	case AD9910_CHAN_IDX_PROFILE_0 ... AD9910_CHAN_IDX_PROFILE_7:
		return 0;
	case AD9910_CHAN_IDX_PARALLEL_AMP ... AD9910_CHAN_IDX_PARALLEL_POLAR_PHASE:
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

static int ad9910_axi_set_sample_rate(struct iio_backend *back,
				      unsigned int chan, u64 sample_rate)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	struct ad9910_axi_drctl_wfhw wfhw;
	u64 div, pd_clk_freq_uhz;
	int ret;

	guard(mutex)(&st->lock);

	switch (chan) {
	case AD9910_CHAN_IDX_PHY:
		if (!sample_rate || sample_rate > 4 * AD9910_AXI_PD_CLK_MAX_FREQ_HZ)
			return -EINVAL;

		pd_clk_freq_uhz = sample_rate * MICROHZ_PER_HZ >> 2;
		if (pd_clk_freq_uhz == st->pd_clk_freq_uhz)
			return 0;

		st->pd_clk_freq_uhz = pd_clk_freq_uhz;
		if (!st->drctl_wf.period_length_ns)
			return 0;

		/*
		 * DRCTL is clocked by SYNC_CLK, reprogram it so that the PWM
		 * output keeps the applied waveform.
		 */
		ad9910_axi_drctl_tohw(st, &st->drctl_wf, &wfhw);
		return ad9910_axi_drctl_write(st, &wfhw);
	case AD9910_CHAN_IDX_PARALLEL_AMP ... AD9910_CHAN_IDX_PARALLEL_POLAR_PHASE:
		if (!sample_rate || sample_rate > AD9910_AXI_PD_CLK_MAX_FREQ_HZ)
			return -EINVAL;

		sample_rate *= MICROHZ_PER_HZ;
		div = DIV64_U64_ROUND_CLOSEST(st->pd_clk_freq_uhz, sample_rate);
		div = clamp(div, 1U, AD9910_AXI_PAR_RATE_MAX);
		ret = regmap_write(st->regmap, AD9910_AXI_REG_PAR_RATE, div - 1);
		if (ret)
			return ret;

		return regmap_set_bits(st->regmap, AD9910_AXI_REG_UPDATE_CTRL,
				       AD9910_AXI_REG_UPDATE_RATE_MSK);
	default:
		return -EINVAL;
	}
}

static int ad9910_axi_read_raw(struct iio_backend *back,
			       struct iio_chan_spec const *chan, int *val,
			       int *val2, long mask)
{
	struct ad9910_axi_state *st = iio_backend_get_priv(back);
	u32 div;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->address) {
		case AD9910_CHAN_IDX_PARALLEL_AMP ... AD9910_CHAN_IDX_PARALLEL_POLAR_PHASE: {
			guard(mutex)(&st->lock);

			ret = regmap_read(st->regmap, AD9910_AXI_REG_PAR_RATE,
					  &div);
			if (ret)
				return ret;

			iio_val_s64_decompose(div64_u64(st->pd_clk_freq_uhz,
							(u64)div + 1),
					      val, val2);
			return IIO_VAL_DECIMAL64_MICRO;
		}
		default:
			return -EINVAL;
		}
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

static const struct iio_backend_ops ad9910_axi_iio_back_ops = {
	.chan_enable = ad9910_axi_chan_enable,
	.chan_disable = ad9910_axi_chan_disable,
	.request_buffer = ad9910_axi_request_buffer,
	.free_buffer = ad9910_axi_free_buffer,
	.set_sample_rate = ad9910_axi_set_sample_rate,
	.read_raw = ad9910_axi_read_raw,
	.debugfs_reg_access = iio_backend_debugfs_ptr(ad9910_axi_reg_access),
};

static const struct iio_backend_info ad9910_axi_back_info = {
	.name = "ad9910-axi",
	.ops = &ad9910_axi_iio_back_ops,
};

static const struct regmap_config ad9910_axi_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int ad9910_axi_pwm_round_waveform_tohw(struct pwm_chip *chip,
					      struct pwm_device *pwm,
					      const struct pwm_waveform *wf,
					      void *wfhw)
{
	struct ad9910_axi_state *st = pwmchip_get_drvdata(chip);

	guard(mutex)(&st->lock);

	return ad9910_axi_drctl_tohw(st, wf, wfhw);
}

static int ad9910_axi_pwm_round_waveform_fromhw(struct pwm_chip *chip,
						struct pwm_device *pwm,
						const void *wfhw,
						struct pwm_waveform *wf)
{
	struct ad9910_axi_state *st = pwmchip_get_drvdata(chip);

	guard(mutex)(&st->lock);

	ad9910_axi_drctl_fromhw(st, wfhw, wf);

	return 0;
}

static int ad9910_axi_pwm_read_waveform(struct pwm_chip *chip,
					struct pwm_device *pwm, void *_wfhw)
{
	struct ad9910_axi_state *st = pwmchip_get_drvdata(chip);
	struct ad9910_axi_drctl_wfhw *wfhw = _wfhw;
	u32 drg;
	int ret;

	*wfhw = (struct ad9910_axi_drctl_wfhw){ };

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AD9910_AXI_REG_DRG, &drg);
	if (ret)
		return ret;

	if (!FIELD_GET(AD9910_AXI_REG_DRG_TOGGLE_EN_MSK, drg))
		return 0;

	ret = regmap_read(st->regmap, AD9910_AXI_REG_DRCTL_PERIOD, &wfhw->period);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD9910_AXI_REG_DRCTL_WIDTH, &wfhw->width);
	if (ret)
		return ret;

	return regmap_read(st->regmap, AD9910_AXI_REG_DRCTL_DELAY, &wfhw->delay);
}

static int ad9910_axi_pwm_write_waveform(struct pwm_chip *chip,
					 struct pwm_device *pwm,
					 const void *_wfhw)
{
	struct ad9910_axi_state *st = pwmchip_get_drvdata(chip);
	const struct ad9910_axi_drctl_wfhw *wfhw = _wfhw;
	int ret;

	guard(mutex)(&st->lock);

	ret = ad9910_axi_drctl_write(st, wfhw);
	if (ret)
		return ret;

	ad9910_axi_drctl_fromhw(st, wfhw, &st->drctl_wf);

	return 0;
}

static const struct pwm_ops ad9910_axi_pwm_ops = {
	.sizeof_wfhw = sizeof(struct ad9910_axi_drctl_wfhw),
	.round_waveform_tohw = ad9910_axi_pwm_round_waveform_tohw,
	.round_waveform_fromhw = ad9910_axi_pwm_round_waveform_fromhw,
	.read_waveform = ad9910_axi_pwm_read_waveform,
	.write_waveform = ad9910_axi_pwm_write_waveform,
};

static int ad9910_axi_setup(struct ad9910_axi_state *st)
{
	int ret;

	st->pd_clk_freq_uhz = AD9910_AXI_PD_CLK_MAX_FREQ_HZ * MICROHZ_PER_HZ;

	ret = regmap_set_bits(st->regmap, AD9910_AXI_REG_RESET,
			      AD9910_AXI_REG_RESET_CORE_MSK);
	if (ret)
		return ret;

	return regmap_clear_bits(st->regmap, AD9910_AXI_REG_RESET,
				 AD9910_AXI_REG_RESET_CORE_MSK);
}

static int ad9910_axi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad9910_axi_state *st;
	struct pwm_chip *chip;
	void __iomem *base;
	struct clk *clk;
	int ret;

	chip = devm_pwmchip_alloc(dev, 1, sizeof(*st));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	st = pwmchip_get_drvdata(chip);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	st->dev = dev;
	st->regmap = devm_regmap_init_mmio(dev, base,
					   &ad9910_axi_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to init register map\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	ret = ad9910_axi_setup(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup device\n");

	chip->ops = &ad9910_axi_pwm_ops;

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "Could not register PWM chip\n");

	return devm_iio_backend_register(dev, &ad9910_axi_back_info, st);
}

static const struct of_device_id ad9910_axi_of_match[] = {
	{ .compatible = "adi,ad9910-axi-backend" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9910_axi_of_match);

static struct platform_driver ad9910_axi_driver = {
	.driver = {
		.name = "ad9910-axi-backend",
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
