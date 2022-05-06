// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices PulSAR ADC family driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/regulator/consumer.h>

#define AD4003_READ_COMMAND	0x54
#define AD4003_WRITE_COMMAND	0x14
#define AD4003_RESERVED_MSK	0xE0
#define AD4003_REG_CONFIG       0x00
#define AD4003_TURBO_MODE       BIT(1)
#define AD4003_HIGH_Z_MODE      BIT(2)

#define AD_PULSAR_CHANNEL(info)                                                 \
        {                                                                       \
                .type = IIO_VOLTAGE,                                            \
                .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) |       \
                                           BIT(IIO_CHAN_INFO_SCALE),            \
		.scan_type = {						        \
			.sign = info.input_type == SINGLE_ENDED ? 'u' : 's',    \
			.storagebits = 32,			                \
			.realbits = info.resolution,				\
		}                                                               \
        }

enum {
        ID_AD7988_5,
        ID_AD7988_1,
        ID_AD7984,
        ID_AD7983,
        ID_AD7982,
        ID_AD7980,
        ID_AD7946,
        ID_AD7942,
        ID_AD7693,
        ID_AD7691,
        ID_AD7690,
        ID_AD7688,
        ID_AD7687,
        ID_AD7686,
        ID_AD7685,
	ID_AD4022,
	ID_AD4021,
	ID_AD4020,
	ID_AD4011,
	ID_AD4007,
        ID_AD4003,
};

enum ad_pulsar_input_type {
        SINGLE_ENDED = 0,
        DIFFERENTIAL
};

struct ad_pulsar_spi_config {
        unsigned int init_val;
        unsigned int init_reg;
        unsigned int reg_write_msk;
        unsigned int reg_read_msk;
};

struct ad_pulsar_chip_info {
        const struct ad_pulsar_spi_config *config;
        enum ad_pulsar_input_type input_type;
        const char *name;
        int resolution;
        int sclk_rate;
        int max_rate;
};

static const struct ad_pulsar_spi_config ad4003_spi_init = {
        .init_reg = AD4003_REG_CONFIG,
        .init_val = AD4003_TURBO_MODE,
        .reg_write_msk = AD4003_WRITE_COMMAND,
        .reg_read_msk = AD4003_READ_COMMAND,
};

static const struct ad_pulsar_chip_info ad_pulsar_chip_infos[] = {
        [ID_AD7988_5] = {
                .name = "ad7988-5",
                .input_type = SINGLE_ENDED,
                .max_rate = 500000,
                .resolution = 16,
                .sclk_rate = 80000000
        },
        [ID_AD7988_1] = {
                .name = "ad7988-1",
                .input_type = SINGLE_ENDED,
                .max_rate = 100000,
                .resolution = 16,
                .sclk_rate = 80000000
        },
        [ID_AD7984] = {
                .name = "ad7984",
                .input_type = DIFFERENTIAL,
                .max_rate = 1333333,
                .resolution = 18,
                .sclk_rate = 80000000
        },
        [ID_AD7983] = {
                .name = "ad7983",
                .input_type = SINGLE_ENDED,
                .max_rate = 1333333,
                .resolution = 16,
                .sclk_rate = 80000000
        },
        [ID_AD7982] = {
                .name = "ad7982",
                .input_type = DIFFERENTIAL,
                .max_rate = 1000000,
                .resolution = 18,
                .sclk_rate = 80000000
        },
        [ID_AD7980] = {
                .name = "ad7980",
                .input_type = SINGLE_ENDED,
                .max_rate = 1000000,
                .resolution = 16,
                .sclk_rate = 80000000
        },
        [ID_AD7946] = {
                .name = "ad7946",
                .input_type = SINGLE_ENDED,
                .max_rate = 500000,
                .resolution = 14,
                .sclk_rate = 40000000
        },
        [ID_AD7942] = {
                .name = "ad7942",
                .input_type = SINGLE_ENDED,
                .max_rate = 250000,
                .resolution = 14,
                .sclk_rate = 40000000
        },
        [ID_AD7693] = {
                .name = "ad7693",
                .input_type = DIFFERENTIAL,
                .max_rate = 500000,
                .resolution = 16,
                .sclk_rate = 40000000
        },
        [ID_AD7691] = {
                .name = "ad7691",
                .input_type = DIFFERENTIAL,
                .max_rate = 250000,
                .resolution = 18,
                .sclk_rate = 40000000
        },
        [ID_AD7690] = {
                .name = "ad7690",
                .input_type = DIFFERENTIAL,
                .max_rate = 400000,
                .resolution = 18,
                .sclk_rate = 40000000
        },
        [ID_AD7688] = {
                .name = "ad7688",
                .input_type = DIFFERENTIAL,
                .max_rate = 500000,
                .resolution = 16,
                .sclk_rate = 40000000
        },
        [ID_AD7687] = {
                .name = "ad7687",
                .input_type = DIFFERENTIAL,
                .max_rate = 250000,
                .resolution = 16,
                .sclk_rate = 40000000
        },
        [ID_AD7686] = {
                .name = "ad7686",
                .input_type = SINGLE_ENDED,
                .max_rate = 500000,
                .resolution = 16,
                .sclk_rate = 40000000
        },
        [ID_AD7685] = {
                .name = "ad7685",
                .input_type = SINGLE_ENDED,
                .max_rate = 250000,
                .resolution = 16,
                .sclk_rate = 40000000
        },
        [ID_AD4022] = {
                .name = "ad4022",
                .input_type = DIFFERENTIAL,
                .max_rate = 500000,
                .resolution = 20,
                .sclk_rate = 80000000,
                .config = &ad4003_spi_init,
        },
        [ID_AD4021] = {
                .name = "ad4021",
                .input_type = DIFFERENTIAL,
                .max_rate = 1000000,
                .resolution = 20,
                .sclk_rate = 80000000,
                .config = &ad4003_spi_init,
        },
        [ID_AD4020] = {
                .name = "ad4020",
                .input_type = DIFFERENTIAL,
                .max_rate = 1800000,
                .resolution = 20,
                .sclk_rate = 80000000,
                .config = &ad4003_spi_init,
        },
        [ID_AD4011] = {
                .name = "ad4011",
                .input_type = DIFFERENTIAL,
                .max_rate = 500000,
                .resolution = 18,
                .sclk_rate = 80000000,
                .config = &ad4003_spi_init,
        },
        [ID_AD4007] = {
                .name = "ad4007",
                .input_type = DIFFERENTIAL,
                .max_rate = 1000000,
                .resolution = 18,
                .sclk_rate = 80000000,
                .config = &ad4003_spi_init,
        },
        [ID_AD4003] = {
                .name = "ad4003",
                .input_type = DIFFERENTIAL,
                .max_rate = 2000000,
                .resolution = 18,
                .sclk_rate = 80000000,
                .config = &ad4003_spi_init,
        }
};

static const struct iio_chan_spec ad_pulsar_iio_channels[] = {
        [ID_AD7988_5] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7988_5]),
        [ID_AD7988_1] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7988_1]),
        [ID_AD7984] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7984]),
        [ID_AD7983] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7983]),
        [ID_AD7982] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7982]),
        [ID_AD7980] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7980]),
        [ID_AD7946] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7946]),
        [ID_AD7942] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7942]),
        [ID_AD7693] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7693]),
        [ID_AD7691] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7691]),
        [ID_AD7690] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7690]),
        [ID_AD7688] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7688]),
        [ID_AD7687] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7687]),
        [ID_AD7686] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7686]),
        [ID_AD7685] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD7685]),
        [ID_AD4022] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD4022]),
        [ID_AD4021] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD4021]),
        [ID_AD4020] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD4020]),
        [ID_AD4011] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD4011]),
        [ID_AD4007] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD4007]),
        [ID_AD4003] = AD_PULSAR_CHANNEL(ad_pulsar_chip_infos[ID_AD4003]),
};

struct ad_pulsar_adc {
        const struct ad_pulsar_chip_info *info;
        struct pwm_device *cnv;
	struct spi_device *spi;
	struct regulator *vref;
        struct clk *ref_clk;
        int spi_speed_hz;
        int samp_freq;
        int device_id;
};

static int ad_pulsar_reg_write(struct ad_pulsar_adc *adc, unsigned int reg,
                               unsigned int val)
{
	struct spi_transfer xfer = {
                .bits_per_word = adc->info->resolution,
                .speed_hz = adc->info->sclk_rate / 4,
                .len = 4,
	};
        unsigned char rx[4], tx[4];

        tx[0] = reg | adc->info->config->reg_write_msk;
        tx[1] = val;
        tx[2] = 0xFF;
        xfer.tx_buf = tx;
        xfer.rx_buf = rx;

	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static int ad_pulsar_reg_read(struct ad_pulsar_adc *adc, unsigned int reg,
                              unsigned int *val)
{
	struct spi_transfer xfer = {
                .bits_per_word = 16,
                .speed_hz = adc->info->sclk_rate / 4,
                .len = 4,
	};
        unsigned char rx[4], tx[4];
        int ret;

        tx[0] = reg | adc->info->config->reg_read_msk;
        xfer.tx_buf = tx;
        xfer.rx_buf = rx;
	ret = spi_sync_transfer(adc->spi, &xfer, 1);
        if (ret < 0)
                return ret;

        *val = rx[0];

        return ret;
}

static int ad_pulsar_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			        unsigned int writeval, unsigned int *readval)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	int ret;

	// mutex_lock(&st->lock);
	if (readval) {
		ret = ad_pulsar_reg_read(adc, reg, readval);
		if (ret < 0)
			goto err_unlock;
	} else {
		ret = ad_pulsar_reg_write(adc, reg, writeval);
	}
err_unlock:
	// mutex_unlock(&st->lock);

	return ret;
}

static int ad_pulsar_set_samp_freq(struct ad_pulsar_adc *adc, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnv_state;
	unsigned long ref_clk_rate;
	int ret;

        freq = clamp(freq, 0, adc->info->max_rate);
	ref_clk_rate = clk_get_rate(adc->ref_clk);
	target = DIV_ROUND_CLOSEST_ULL(ref_clk_rate, freq);
        ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL, ref_clk_rate);
	cnv_state.period = ref_clk_period_ps * target;
	cnv_state.duty_cycle = ref_clk_period_ps;
	cnv_state.offset = ref_clk_period_ps;
	cnv_state.time_unit = PWM_UNIT_PSEC;
	cnv_state.enabled = true;
	ret = pwm_apply_state(adc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	adc->samp_freq = (int)DIV_ROUND_CLOSEST_ULL(ref_clk_rate, target);

        return ret;
}

static int ad_pulsar_read_raw(struct iio_dev *indio_dev,
                              const struct iio_chan_spec *chan,
                              int *val, int *val2, long info)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        int ret;

        switch (info) {
        case IIO_CHAN_INFO_SAMP_FREQ:
                *val = adc->samp_freq;

                return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(adc->vref);
		if (ret < 0)
			return ret;
		*val = ret / 1000;
		*val2 = adc->info->resolution;

		return IIO_VAL_FRACTIONAL_LOG2;
        default:
                return -EINVAL;
        }
}

static int ad_pulsar_write_raw(struct iio_dev *indio_dev,
                               const struct iio_chan_spec *chan,
                               int val, int val2, long info)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);

        switch (info){
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad_pulsar_set_samp_freq(adc, val);
	default:
		return -EINVAL;
        }
}

static void ad_pulsar_reg_disable(void *data)
{
	regulator_disable(data);
}

static void ad_pulsar_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static void ad_pulsar_clk_disable(void *data)
{
        clk_disable_unprepare(data);
}

static int ad_pulsar_buffer_preenable(struct iio_dev *indio_dev)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        unsigned int spi_rx_data;
        unsigned int spi_tx_data = 0xFFFFFFFF;
        struct spi_transfer xfer = {
                .tx_buf = &spi_tx_data,
                .rx_buf = &spi_rx_data,
                .len = 3,
                .bits_per_word = adc->info->resolution,
                .speed_hz = adc->info->sclk_rate,
        };

        struct spi_message msg;
        int ret;

        spi_bus_lock(adc->spi->master);
        spi_message_init_with_transfers(&msg, &xfer, 1);
        ret = spi_engine_offload_load_msg(adc->spi, &msg);
        if (ret < 0)
                return ret;
        spi_engine_offload_enable(adc->spi, true);

        return ret;
}

static int ad_pulsar_buffer_postdisable(struct iio_dev *indio_dev)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);

        spi_engine_offload_enable(adc->spi, false);
        spi_bus_unlock(adc->spi->master);

	return 0;
}

static int ad_pulsar_dma_submit(struct iio_dma_buffer_queue *queue,
                                struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops ad_pulsar_dma_buffer_ops = {
	.submit = ad_pulsar_dma_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad_pulsar_buffer_ops = {
	.preenable = &ad_pulsar_buffer_preenable,
	.postdisable = &ad_pulsar_buffer_postdisable,
};

static const struct iio_info ad_pulsar_iio_info = {
        .read_raw = ad_pulsar_read_raw,
        .write_raw = ad_pulsar_write_raw,
	.debugfs_reg_access = &ad_pulsar_reg_access,
};

static int ad_pulsar_setup(struct iio_dev *indio_dev)
{
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
        int ret;

        ad_pulsar_set_samp_freq(adc, adc->info->max_rate);
        if (adc->info->config->init_val) {
                ret = ad_pulsar_reg_write(adc, adc->info->config->init_reg,
                        adc->info->config->init_val);
                if (ret < 0)
                        return ret;
        }

        return ad_pulsar_set_samp_freq(adc, adc->info->max_rate);
}

static const struct of_device_id ad_pulsar_of_match[] = {
	{ .compatible = "adi,ad7988-5" , .data = (void *)ID_AD7988_5 },
        { .compatible = "adi,ad7988-1" , .data = (void *)ID_AD7988_1 },
        { .compatible = "adi,ad7984"   , .data = (void *)ID_AD7984   },
        { .compatible = "adi,ad7983"   , .data = (void *)ID_AD7983   },
        { .compatible = "adi,ad7982"   , .data = (void *)ID_AD7982   },
        { .compatible = "adi,ad7980"   , .data = (void *)ID_AD7980   },
        { .compatible = "adi,ad7946"   , .data = (void *)ID_AD7946   },
        { .compatible = "adi,ad7942"   , .data = (void *)ID_AD7942   },
        { .compatible = "adi,ad7693"   , .data = (void *)ID_AD7693   },
        { .compatible = "adi,ad7691"   , .data = (void *)ID_AD7691   },
        { .compatible = "adi,ad7690"   , .data = (void *)ID_AD7690   },
        { .compatible = "adi,ad7688"   , .data = (void *)ID_AD7688   },
        { .compatible = "adi,ad7687"   , .data = (void *)ID_AD7687   },
        { .compatible = "adi,ad7686"   , .data = (void *)ID_AD7686   },
        { .compatible = "adi,ad7685"   , .data = (void *)ID_AD7685   },
        { .compatible = "adi,ad4022"   , .data = (void *)ID_AD4022   },
        { .compatible = "adi,ad4021"   , .data = (void *)ID_AD4021   },
        { .compatible = "adi,ad4020"   , .data = (void *)ID_AD4020   },
        { .compatible = "adi,ad4011"   , .data = (void *)ID_AD4011   },
        { .compatible = "adi,ad4007"   , .data = (void *)ID_AD4007   },
        { .compatible = "adi,ad4003"   , .data = (void *)ID_AD4003   },
        { .compatible = "adi,adaq4003" , .data = (void *)ID_AD4003   },
	{ },
};
MODULE_DEVICE_TABLE(of, ad_pulsar_of_match);

static int ad_pulsar_parse_channels(struct iio_dev *indio_dev)
{
	struct device *child, *dev = indio_dev->dev.parent;
        struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	struct iio_chan_spec *iio_channels;
        int num_ch, ret;

	num_ch = device_get_child_node_count(dev);
        iio_channels = devm_kzalloc(indio_dev->dev.parent,
                                    num_ch * sizeof(*iio_channels), GFP_KERNEL);
        device_for_each_child_node(dev, child) {

        }
        return ret;
}

static int ad_pulsar_probe(struct spi_device *spi)
{
        struct ad_pulsar_adc *adc;
	struct iio_buffer *buffer;
        struct iio_dev *indio_dev;
        int device_id;
        int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	adc->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->vref))
		return PTR_ERR(adc->vref);

	ret = regulator_enable(adc->vref);
	if (ret) {
                dev_err(&spi->dev, "Failed to enable VREF regulator");
		return ret;
        }

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_reg_disable,
                                       adc->vref);
	if (ret)
		return ret;

        adc->ref_clk = devm_clk_get(&spi->dev, "ref_clk");
        if (IS_ERR(adc->ref_clk))
                return PTR_ERR(adc->ref_clk);

        ret = clk_prepare_enable(adc->ref_clk);
        if (ret < 0)
                return ret;

        ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_clk_disable,
                                       adc->ref_clk);
        if (ret < 0)
                return ret;

        adc->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(adc->cnv))
		return PTR_ERR(adc->cnv);

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_pwm_diasble,
                                       adc->cnv);
        if (ret < 0)
                return ret;

        device_id = (int)device_get_match_data(&spi->dev);
        adc->device_id = device_id;
        adc->info = &ad_pulsar_chip_infos[device_id];
        indio_dev->name = adc->info->name;
	indio_dev->dev.parent = &spi->dev;
        indio_dev->channels = &ad_pulsar_iio_channels[device_id];
	indio_dev->num_channels = 1;
        indio_dev->info = &ad_pulsar_iio_info;
        indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad_pulsar_buffer_ops;
        buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
					         "rx",
					         &ad_pulsar_dma_buffer_ops,
					         indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

        ret = ad_pulsar_setup(indio_dev);
        if (ret < 0)
                return ret;

        return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad_pulsar_driver = {
	.driver = {
		.name = "pulsar_adc",
		.of_match_table = ad_pulsar_of_match,
	},
	.probe = ad_pulsar_probe,
};
module_spi_driver(ad_pulsar_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices PulSAR ADC family driver");
MODULE_LICENSE("GPL v2");