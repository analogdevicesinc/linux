// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices ad7606x ADC driver for Parallel Interface
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/fpga/adi-axi-common.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			BIT(0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
/* Enable ADC channel with sign extension in HDL regmap's REG_CHAN_CNTRL register */
#define ADI_ENABLE_FMT_SIGNEX		0x51

#define KHZ				1000
#define SLEEP_TIME			3300
#define SETUP_TIME			275

#define AD7606X_MULTIPLE_CHAN(_idx, _storagebits, _realbits)		\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.indexed = 1,						\
		.channel = _idx,					\
		.scan_index = _idx,					\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = _storagebits,			\
			.realbits = _realbits,				\
			.shift = 0,					\
		},							\
	}

enum ad7606x_pi_id {
	ID_AD7606X_16,
	ID_AD7606X_18,
};

struct ad7606x_pi_info {
	char				*name;
	struct iio_chan_spec		channels[8];
	unsigned int			num_channels;
	unsigned int			resolution;
};

static const struct ad7606x_pi_info ad7606x_pi_infos[] = {
	[ID_AD7606X_16] = {
		.name = "AD7606X-16",
		.resolution = 16,
		.channels = {
			AD7606X_MULTIPLE_CHAN(0, 16, 16),
			AD7606X_MULTIPLE_CHAN(1, 16, 16),
			AD7606X_MULTIPLE_CHAN(2, 16, 16),
			AD7606X_MULTIPLE_CHAN(3, 16, 16),
			AD7606X_MULTIPLE_CHAN(4, 16, 16),
			AD7606X_MULTIPLE_CHAN(5, 16, 16),
			AD7606X_MULTIPLE_CHAN(6, 16, 16),
			AD7606X_MULTIPLE_CHAN(7, 16, 16),
		},
		.num_channels = 8,
	},
	[ID_AD7606X_18] = {
		.name = "AD7606X-18",
		.resolution = 18,
		.channels = {
			AD7606X_MULTIPLE_CHAN(0, 32, 18),
			AD7606X_MULTIPLE_CHAN(1, 32, 18),
			AD7606X_MULTIPLE_CHAN(2, 32, 18),
			AD7606X_MULTIPLE_CHAN(3, 32, 18),
			AD7606X_MULTIPLE_CHAN(4, 32, 18),
			AD7606X_MULTIPLE_CHAN(5, 32, 18),
			AD7606X_MULTIPLE_CHAN(6, 32, 18),
			AD7606X_MULTIPLE_CHAN(7, 32, 18),
		},
		.num_channels = 8,
	},
};

struct ad7606x_pi_state {
	const	struct	ad7606x_pi_info			*device_info;
	struct	iio_info				iio_info;
	struct	gpio_desc				*adc_serpar;
	struct	gpio_desc				*adc_refsel;
	struct	gpio_desc				*adc_reset;
	struct	gpio_desc				*adc_standby;
	struct	gpio_desc				*adc_range;
	struct	gpio_descs				*adc_os;
	struct	pwm_device				*cnvst_n;
	struct	clk					*ref_clk;

	/* protect against device accessse */
	struct	mutex					lock;

	void	__iomem					*regs;
	int						sampling_freq;

	unsigned	int				pcore_version;
	unsigned	long				ref_clk_rate;
};

static inline void axiadc_write(struct ad7606x_pi_state *ad7606x_pi, unsigned int reg, unsigned int val)
{
	iowrite32(val, ad7606x_pi->regs + reg);
}

static inline unsigned int axiadc_read(struct ad7606x_pi_state *ad7606x_pi, unsigned int reg)
{
	return ioread32(ad7606x_pi->regs + reg);
}

static int ad7606x_pi_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);

	axiadc_write(ad7606x_pi, ADI_REG_STATUS, ~0);
	axiadc_write(ad7606x_pi, ADI_REG_DMA_STATUS, ~0);

	return 0;
}

static const struct iio_dma_buffer_ops ad7606x_pi_dma_buffer_ops = {
	.submit = ad7606x_pi_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int ad7606x_pi_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (!dma_name)
		dma_name = "rx";

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
						 &ad7606x_pi_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static int ad7606x_pi_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg, unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);

	mutex_lock(&ad7606x_pi->lock);
	if (!readval)
		axiadc_write(ad7606x_pi, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(ad7606x_pi, reg & 0xFFFF);
	mutex_unlock(&ad7606x_pi->lock);

	return 0;
}

static int ad7606x_pi_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);
	unsigned int i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(ad7606x_pi, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE_FMT_SIGNEX;
		else
			ctrl &= ~ADI_ENABLE_FMT_SIGNEX;

		axiadc_write(ad7606x_pi, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static int ad7606x_pi_set_sampling_freq(struct ad7606x_pi_state *ad7606x_pi, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnvst_n_state;
	int ret;

	target = DIV_ROUND_CLOSEST_ULL(ad7606x_pi->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL,
						  ad7606x_pi->ref_clk_rate);
	cnvst_n_state.period = ref_clk_period_ps * target;
	cnvst_n_state.duty_cycle = ref_clk_period_ps;
	cnvst_n_state.phase = ref_clk_period_ps;
	cnvst_n_state.time_unit = PWM_UNIT_PSEC;
	cnvst_n_state.enabled = true;
	ret = pwm_apply_state(ad7606x_pi->cnvst_n, &cnvst_n_state);
	if (ret)
		return ret;

	ad7606x_pi->sampling_freq = DIV_ROUND_CLOSEST_ULL(ad7606x_pi->ref_clk_rate, target);

	return 0;
}

static int ad7606x_pi_gpio_request(struct platform_device *pdev, struct ad7606x_pi_state *ad7606x_pi)
{
	ad7606x_pi->adc_serpar = devm_gpiod_get_optional(&pdev->dev, "adi,adc_serpar", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_serpar))
		return PTR_ERR(ad7606x_pi->adc_serpar);

	ad7606x_pi->adc_refsel = devm_gpiod_get_optional(&pdev->dev, "adi,adc_refsel", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_refsel))
		return PTR_ERR(ad7606x_pi->adc_refsel);

	ad7606x_pi->adc_reset = devm_gpiod_get_optional(&pdev->dev, "adi,adc_reset", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_reset))
		return PTR_ERR(ad7606x_pi->adc_reset);

	ad7606x_pi->adc_range = devm_gpiod_get_optional(&pdev->dev, "adi,adc_range", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_range))
		return PTR_ERR(ad7606x_pi->adc_range);

	ad7606x_pi->adc_standby = devm_gpiod_get_optional(&pdev->dev, "adi,adc_standby", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x_pi->adc_standby))
		return PTR_ERR(ad7606x_pi->adc_standby);

	ad7606x_pi->adc_os = devm_gpiod_get_array_optional(&pdev->dev, "adi,adc_os", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x_pi->adc_os))
		return PTR_ERR(ad7606x_pi->adc_os);

	return 0;
}

static int ad7606x_pi_full_reset(struct platform_device *pdev, struct ad7606x_pi_state *ad7606x_pi)
{
	int ret;

	ret = gpiod_direction_output(ad7606x_pi->adc_reset, 1);
	if (ret) {
		dev_err(&pdev->dev, "gpiod_direction_output to 1 failed");
		return ret;
	}

	ndelay(SLEEP_TIME);

	ret = gpiod_direction_output(ad7606x_pi->adc_reset, 0);
	if (ret) {
		dev_err(&pdev->dev, "gpiod_direction_output to 0 failed");
		return ret;
	}

	udelay(SETUP_TIME);

	return 0;
}

static int ad7606x_pi_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long info)
{
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);
	/* Scales are computed as 10000/32768 */
	const int scale_10_s_ended = 305176;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad7606x_pi->sampling_freq;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:

		*val = 0;
		*val2 = scale_10_s_ended;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad7606x_pi_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7606x_pi_set_sampling_freq(ad7606x_pi, val);

	default:
		return -EINVAL;
	}
}

static void ad7606x_pi_pwm_disable(void *data)
{
	pwm_disable(data);
}

static void ad7606x_pi_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct iio_info ad7606x_pi_iio_info = {
	.debugfs_reg_access = &ad7606x_pi_reg_access,
	.update_scan_mode = &ad7606x_pi_update_scan_mode,
	.read_raw = ad7606x_pi_read_raw,
	.write_raw = ad7606x_pi_write_raw,
};

static const struct of_device_id ad7606x_pi_of_match[] = {
	{
		.compatible = "ad7606x-pi-16",
		.data = &ad7606x_pi_infos[ID_AD7606X_16]
	},
	{
		.compatible = "ad7606x-pi-18",
		.data = &ad7606x_pi_infos[ID_AD7606X_18]
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad7606x_pi_of_match);

static int ad7606x_pi_probe(struct platform_device *pdev)
{
	struct iio_dev			*indio_dev;
	struct ad7606x_pi_state		*ad7606x_pi;
	struct resource			*mem;
	int				ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ad7606x_pi));
	if (!indio_dev)
		return -ENOMEM;

	ad7606x_pi = iio_priv(indio_dev);

	mutex_init(&ad7606x_pi->lock);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	ad7606x_pi_gpio_request(pdev, ad7606x_pi);

	ad7606x_pi->ref_clk = devm_clk_get(&pdev->dev, "ad7606x_pi_clk");
	if (IS_ERR(ad7606x_pi->ref_clk))
		return PTR_ERR(ad7606x_pi->ref_clk);

	ret = clk_prepare_enable(ad7606x_pi->ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, ad7606x_pi_clk_disable,
				       ad7606x_pi->ref_clk);
	if (ret)
		return ret;

	ad7606x_pi->ref_clk_rate = clk_get_rate(ad7606x_pi->ref_clk);

	ad7606x_pi->cnvst_n = devm_pwm_get(&pdev->dev, "cnvst_n");

	if (IS_ERR(ad7606x_pi->cnvst_n))
		return PTR_ERR(ad7606x_pi->cnvst_n);

	ret = devm_add_action_or_reset(&pdev->dev, ad7606x_pi_pwm_disable,
				       ad7606x_pi->cnvst_n);
	if (ret)
		return ret;

	ad7606x_pi_full_reset(pdev, ad7606x_pi);

	ad7606x_pi->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(ad7606x_pi->regs))
		return PTR_ERR(ad7606x_pi->regs);

	platform_set_drvdata(pdev, indio_dev);

	/* Reset all HDL Cores */
	axiadc_write(ad7606x_pi, ADI_REG_RSTN, 0);
	axiadc_write(ad7606x_pi, ADI_REG_RSTN, ADI_RSTN);

	ad7606x_pi->pcore_version = axiadc_read(ad7606x_pi, ADI_AXI_REG_VERSION);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ad7606x_pi->device_info = device_get_match_data(&pdev->dev);
	if (!ad7606x_pi->device_info)
		return -EINVAL;

	indio_dev->channels = ad7606x_pi->device_info->channels;
	indio_dev->num_channels = ad7606x_pi->device_info->num_channels;

	ad7606x_pi->iio_info = ad7606x_pi_iio_info;
	indio_dev->info = &ad7606x_pi->iio_info;

	ret = ad7606x_pi_configure_ring_stream(indio_dev, "ad7606x-pi-adc-dma");
	if (ret)
		return ret;

	ret = ad7606x_pi_set_sampling_freq(ad7606x_pi, 100 * KHZ);
	if (ret) {
		dev_err(&pdev->dev, "\nAD7606X setup failed\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, probed ADC %s as %s\n",
		 ad7606x_pi->pcore_version,
		 (unsigned long long)mem->start, ad7606x_pi->regs, ad7606x_pi->device_info->name,
		 axiadc_read(ad7606x_pi, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER");

	return 0;
}

static struct platform_driver ad7606x_pi_driver = {
	.probe          = ad7606x_pi_probe,
	.driver         = {
		.name   = "ad7606x_pi",
		.owner = THIS_MODULE,
		.of_match_table = ad7606x_pi_of_match,
	},
};
module_platform_driver(ad7606x_pi_driver);

MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606X Parallel Interface ADC");
MODULE_LICENSE("Dual BSD/GPL");
