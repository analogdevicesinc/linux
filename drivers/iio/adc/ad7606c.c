// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices AD7606 ADC family driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/util_macros.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
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
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/regulator/consumer.h>

/* Registers */
#define REG_STATUS (0x01)
#define REG_CONFIG (0x02)
#define REG_CH_RANGE(ch) (0x03 + ((ch & 0x7) >> 1))
#define REG_BANDWITH (0x07)
#define REG_OVERSAMPLING (0x08)
#define REG_CH_GAIN(ch) (0x09 + (ch & 0x7))
#define REG_CH_OFFSET(ch) (0x11 + (ch & 0x7))
#define REG_CH_PHASE(ch) (0x19 + (ch & 0x7))
#define REG_DIGITAL_DIAG_ENABLE (0x21)
#define REG_DIGITAL_DIAG_ERR (0x22)
#define REG_OPEN_DETECT_ENABLE (0x23)
#define REG_OPEN_DETECTED (0x24)
#define REG_DIAGNOSTIC_MUX_CH1_2 (0x28)
#define REG_DIAGNOSTIC_MUX_CH3_4 (0x29)
#define REG_DIAGNOSTIC_MUX_CH5_6 (0x2A)
#define REG_DIAGNOSTIC_MUX_CH7_8 (0x2B)
#define REG_OPEN_DETECT_QUEUE (0x2C)
#define REG_FS_CLK_COUNTER (0x2D)
#define REG_OS_CLK_COUNTER (0x2E)
#define REG_ID (0x2F)

/* Bits */
#define INTERFACE_CHECK_EN BIT(7)
#define RANGE_CH_MASK(ch) (GENMASK(3, 0) << (4 * ((ch) & 0x1)))
#define RANGE_CH_MODE(ch, mode)	((GENMASK(3, 0) & mode) << (4 * ((ch) & 0x1)))

/* Commands */
#define AD7606_READ_CMD(reg) ((1 << 6) | (reg & 0x3F))
#define AD7606_WRITE_CMD(reg) (reg & 0x3F)
#define AD7606_8_DOUT_LINES (BIT(4) | BIT(3))

static const unsigned long ad7606c_available_scan_masks[]  = { 0xFF, 0x00 };

static const unsigned int ad7606c_oversampling_avail[9] = {
	0, 2, 4, 8, 16, 32, 64, 128, 256
};

static const unsigned int ad7606c_oversampling_pwm_rate[9] = {
    500000, 1700000, 3600000, 7600000, 
    15500000, 31000000, 63000000, 126000000, 252000000
};

static const unsigned int ad7606c_scale_avail[5] = {
	19073, 38146, 47683, 76293, 95367
};

enum {
    ID_INVALID,
    ID_AD7606C_16,
    ID_AD7606C_18,
};

struct ad7606_chip_info {
    const char *name;
    int resolution;
    int num_channels;
    int sclk_rate;
    int bits_per_word;
    const unsigned int *oversampling_avail;
    int num_oversamplings;
    const unsigned int *scale_avail;
    int num_scales;
};

struct ad7606_chip_info ad7606_chip_infos[] = {
    [ID_AD7606C_16] = {
        .name = "ad7606c-16",
        .resolution = 16,
        .num_channels = 8,
        .sclk_rate = 60000000,
        .bits_per_word = 16,
        .oversampling_avail = ad7606c_oversampling_avail,
        .num_oversamplings = ARRAY_SIZE(ad7606c_oversampling_avail),
        .scale_avail = ad7606c_scale_avail,
        .num_scales = ARRAY_SIZE(ad7606c_scale_avail),
    },
    [ID_AD7606C_18] = {
        .name = "ad7606c-18",
        .resolution = 18,
        .num_channels = 8,
        .sclk_rate = 60000000,
        .bits_per_word = 16,
        .oversampling_avail = ad7606c_oversampling_avail,
        .num_oversamplings = ARRAY_SIZE(ad7606c_oversampling_avail),
        .scale_avail = ad7606c_scale_avail,
        .num_scales = ARRAY_SIZE(ad7606c_scale_avail),
    },
};

struct ad7606_state {
    const struct ad7606_chip_info *info;
    int device_id;
    unsigned long ref_clk_rate;
    int sampling_frequency;
    struct mutex lock;
    struct pwm_device *cnv;
    struct spi_device *spi;
    struct regulator *vref;
    struct gpio_desc *gpio_reset;
    struct gpio_desc *gpio_range;
    struct gpio_desc *gpio_standby;
    struct gpio_desc *gpio_frstdata;
    struct gpio_descs *gpio_os;
    struct gpio_desc *gpio_serpar;
    uint8_t tx[4];
    uint8_t rx[2];

    /*
     * DMA (thus cache coherency maintenance) requires the
     * transfer buffers to live in their own cache lines.
    */
    uint32_t data[8] ____cacheline_aligned;
};

#define AD7606_CHANNEL(_name, _idx) \
{  \
    .type = IIO_VOLTAGE,  \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
                          BIT(IIO_CHAN_INFO_SCALE) |  \
                          BIT(IIO_CHAN_INFO_HARDWAREGAIN) |  \
                          BIT(IIO_CHAN_INFO_OFFSET) |  \
                          BIT(IIO_CHAN_INFO_PHASE),  \
    .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |  \
                               BIT(IIO_CHAN_INFO_SAMP_FREQ),  \
    .indexed = 1,  \
    .output = 0,  \
    .channel = _idx,  \
    .scan_index = _idx,  \
    .address = _idx, \
    .scan_type = {  \
        .sign = 's',  \
        .endianness = IIO_LE,  \
    },  \
    .extend_name = _name, \
}

static struct iio_chan_spec ad7606c_base_channels[] = {
    AD7606_CHANNEL("v1", 0),
    AD7606_CHANNEL("v2", 1),
    AD7606_CHANNEL("v3", 2),
    AD7606_CHANNEL("v4", 3),
    AD7606_CHANNEL("v5", 4),
    AD7606_CHANNEL("v6", 5),
    AD7606_CHANNEL("v7", 6),
    AD7606_CHANNEL("v8", 7),
};

static const struct of_device_id ad7606_of_match[] = {
    { .compatible = "adi,ad7606c-18", .data = (void*)ID_AD7606C_18 },
    { },
}
MODULE_DEVICE_TABLE(of, ad7606_of_match);

static const struct spi_device_id ad7606_spi_id[] = {
    { "adi,ad7606c", ID_AD7606C_18 },
    { },
}
MODULE_DEVICE_TABLE(spi, ad7606_spi_id);

static ssize_t ad7606_show_avail(char *buf, const unsigned int *vals,
				 unsigned int n, bool micros)
{
	size_t len = 0;
	int i;

	for (i = 0; i < n; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			micros ? "0.%06u " : "%u ", vals[i]);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t ad7606_oversampling_ratio_avail(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7606_state *adc = iio_priv(indio_dev);

	return ad7606_show_avail(buf, adc->info->oversampling_avail,
				 adc->info->num_oversamplings, false);
}

static IIO_DEVICE_ATTR(oversampling_ratio_available, 0444,
		       ad7606_oversampling_ratio_avail, NULL, 0);

static ssize_t in_voltage_scale_available_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7606_state *adc = iio_priv(indio_dev);

	return ad7606_show_avail(buf, adc->info->scale_avail, adc->info->num_scales, true);
}

static IIO_DEVICE_ATTR_RO(in_voltage_scale_available, 0);

static int ad7606_request_gpios(struct ad7606_state *adc) {
    struct device *dev = &adc->spi->dev;
    
    adc->gpio_reset = devm_gpiod_get_optional(dev, "adi,reset", GPIOD_OUT_LOW);
    if (IS_ERR(adc->gpio_reset)) {
        return PTR_ERR(adc->gpio_reset);
    }

    adc->gpio_range = devm_gpiod_get_optional(dev, "adi,range", GPIOD_OUT_HIGH);
    if (IS_ERR(adc->gpio_range)) {
        return PTR_ERR(adc->gpio_range);
    }

    adc->gpio_standby = devm_gpiod_get_optional(dev, "adi,standby", GPIOD_OUT_HIGH);
    if (IS_ERR(adc->gpio_standby)) {
        return PTR_ERR(adc->gpio_standby);
    }

    adc->gpio_frstdata = devm_gpiod_get_optional(dev, "adi,first-data", GPIOD_IN);
    if (IS_ERR(adc->gpio_frstdata)) {
        return PTR_ERR(adc->gpio_frstdata);
    }

    adc->gpio_os = devm_gpiod_get_array_optional(dev, "adi,oversampling-ratio", GPIOD_OUT_HIGH);
    if (IS_ERR(adc->gpio_os)) {
        return PTR_ERR(adc->gpio_os);
    }

    adc->gpio_serpar = devm_gpiod_get_optional(dev, "adi,serpar", GPIOD_OUT_HIGH);
    if (IS_ERR(adc->gpio_serpar)) {
        return PTR_ERR(adc->gpio_serpar);
    }

    return 0;
}

static int ad7606_set_sampling_frequency(struct ad7606_state *adc, 
                    unsigned int frequency) {
    int ret = 0;
    unsigned long long target, ref_clk_period_ps;
    struct pwm_state cnv = {};

    target = DIV_ROUND_CLOSEST_ULL(adc->ref_clk_rate, frequency);
    ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000, adc->ref_clk_rate);
    cnv.period = ref_clk_period_ps * target;
    cnv.duty_cycle = ref_clk_period_ps;
    cnv.phase = ref_clk_period_ps;
    cnv.time_unit = PWM_UNIT_PSEC;
    cnv.enabled = true;
    
    ret = pwm_apply_state(adc->cnv, &cnv);
    if (ret < 0) {
        return ret;
    }

    adc->sampling_frequency = DIV_ROUND_CLOSEST_ULL(adc->ref_clk_rate, target);
    return 0;
}

static int ad7606_reset(struct ad7606_state *adc, bool full_reset) {
    int ret = EINVAL;
    const int sleep_time = (full_reset)? 3300 : 100;
    const int setup_time = (full_reset)? 275000 : 70000;

    if (adc->gpio_reset) {
        if (gpiod_direction_output(adc->gpio_reset, 1)) {
            dev_err(&adc->spi->dev, "gpiod_direction_output to 1 failed");
        }
        ndelay(sleep_time);
        if (gpiod_direction_output(adc->gpio_reset, 0)) {
            dev_err(&adc->spi->dev, "gpiod_direction_output to 0 failed");
        }
        ndelay(setup_time);
        ret = 0;
    }

    return ret;
}

static int ad7606_reg_read(struct ad7606_state *adc, 
                unsigned int reg,
                unsigned int *val) {
    int ret;
    struct spi_message m;
    struct spi_transfer t[] = {
        {
            .tx_buf = &adc->tx[0],
            .len = 2,
            .cs_change = 0,
            .bits_per_word = adc->info->bits_per_word,
            .speed_hz = adc->info->sclk_rate,
        },
        {
            .tx_buf = &adc->tx[2],
            .rx_buf = &adc->rx[0],
            .len = 2,
            .bits_per_word = adc->info->bits_per_word,
            .speed_hz = adc->info->sclk_rate,
        },
    };

    adc->tx[0] = AD7606_READ_CMD(reg);

    spi_message_init_with_transfers(&m, t, ARRAY_SIZE(t));
    ret = spi_sync(adc->spi, &m);
    if (ret < 0) {
        dev_err(&adc->spi->dev, "spi_sync_transfer failed for read: %d", ret);
    }

    *val = adc->rx[0];
    
    return ret;
}

static int ad7606_reg_write(struct ad7606_state *adc, 
                unsigned int reg,
                unsigned int val) {
    int ret;
    struct spi_message m;
    struct spi_transfer t[] = {
        {
            .tx_buf = &adc->tx[0],
            .len = 2,
            .bits_per_word = adc->info->bits_per_word,
            .speed_hz = adc->info->sclk_rate,
        },
        {
            .tx_buf = &adc->tx[2],
            .len = 2,
            .bits_per_word = adc->info->bits_per_word,
            .speed_hz = adc->info->sclk_rate,
        },
    };

    adc->tx[0] = AD7606_WRITE_CMD(reg);
    adc->tx[1] = val & 0xFF;

    spi_message_init_with_transfers(&m, t, ARRAY_SIZE(t));
    ret = spi_sync(adc->spi, &m);
    if (ret < 0) {
        dev_err(&adc->spi->dev, "spi_sync_transfer failed for write: %d", ret);
        return ret;
    }
    
    return ret;
}

static int ad7606_reg_write_mask(struct ad7606_state *adc,
				 unsigned int reg,
				 unsigned long mask,
				 unsigned int val) {
    int readval;
    int ret;

    ret = ad7606_reg_read(adc, reg, &readval);
    if (ret < 0) {
        return ret;
    }

    readval &= ~mask;
    readval |= val;

    return ad7606_reg_write(adc, reg, readval);

}

static int ad7606_reg_access(struct iio_dev *indio_dev, 
                unsigned int reg, 
                unsigned int writeval, 
                unsigned int *readval) {
    struct ad7606_state *adc = iio_priv(indio_dev);
    int ret;

    mutex_lock(&adc->lock);
    if (readval) {
        ret = ad7606_reg_read(adc, reg, readval);
        //dev_info(&adc->spi->dev, "reg_read for register: %u value: %d", reg, *readval);
    }
    else {
        ret = ad7606_reg_write(adc, reg, writeval);
        //dev_info(&adc->spi->dev, "reg_write for register: %u value: %u", reg, writeval);
    }
    
    mutex_unlock(&adc->lock);
    return ret;
}

static int ad7606_read_samples(struct iio_dev *indio_dev) {
    struct ad7606_state *adc = iio_priv(indio_dev);
    int ret = 0;
    int i;
    uint32_t *tx = (uint32_t*)&adc->tx[0];
    struct spi_transfer xfer[8];
    struct spi_message m;

    memset(xfer, 0, sizeof(struct spi_transfer) * ARRAY_SIZE(xfer));
    *tx = 0xff;
    for(i=0; i < adc->info->num_channels; i++) {
        xfer[i].tx_buf = tx;
        xfer[i].rx_buf = &adc->data[i];
        xfer[i].bits_per_word = adc->info->resolution;
        xfer[i].speed_hz = adc->info->sclk_rate;
        xfer[i].len = 4;
    }
    spi_message_init_with_transfers(&m, xfer, ARRAY_SIZE(xfer));

    if (adc->gpio_frstdata) {
        while(!gpiod_get_value(adc->gpio_frstdata)) {
            ndelay(10);
            dev_info(&adc->spi->dev, "Waiting for frstdata to be set");
        }
    }

    ret = spi_sync(adc->spi, &m);
    if (ret < 0) {
        dev_err(&adc->spi->dev, "spi_sync err: %d", ret);
        return ret;
    }

    for(i=0; i < adc->info->num_channels; i++) {
        adc->data[i] &= GENMASK(adc->info->resolution-1, 0);
    }

    // for(i=0; i < adc->info->num_channels; i++) {
    //     dev_info(&adc->spi->dev, "data[%d]=0x%05x", i, adc->data[i]);
    // }

    return ret;
}

static int ad7606_read_raw(struct iio_dev *indio_dev,
                 struct iio_chan_spec const *chan,
                 int *val,
                 int *val2,
                 long mask) {
    struct ad7606_state *adc = iio_priv(indio_dev);
    int ret;
    int range;
    
    mutex_lock(&adc->lock);
    switch (mask) {
    case IIO_CHAN_INFO_SAMP_FREQ:
        *val = adc->sampling_frequency;
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_SCALE:
        ret = ad7606_reg_read(adc, REG_CH_RANGE(chan->address), &range);
        if (ret < 0) {
            break;
        }
        range = (range & RANGE_CH_MASK(chan->address)) >> (4 * (chan->address & 1));
        *val2 = adc->info->scale_avail[range];
        *val = 0;
        ret = IIO_VAL_INT_PLUS_MICRO;
        break;
    case IIO_CHAN_INFO_RAW:
        ret = ad7606_read_samples(indio_dev);
        *val = adc->data[chan->address];
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        ret = ad7606_reg_read(adc, REG_OVERSAMPLING, val);
        if (ret < 0) {
            break;
        }
        *val = adc->info->oversampling_avail[*val];
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_HARDWAREGAIN:
        ret = ad7606_reg_read(adc, REG_CH_GAIN(chan->address), val);
        if (ret < 0) {
            break;
        }
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_OFFSET:
        ret = ad7606_reg_read(adc, REG_CH_OFFSET(chan->address), val);
        if (ret < 0) {
            break;
        }
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_PHASE:
        ret = ad7606_reg_read(adc, REG_CH_PHASE(chan->address), val);
        if (ret < 0) {
            break;
        }
        ret = IIO_VAL_INT;
        break;
    default:
        ret = -EINVAL;
        break;
    }

    mutex_unlock(&adc->lock);
    return ret;
}

static int ad7606_write_raw(struct iio_dev *indio_dev,
                 struct iio_chan_spec const *chan,
                 int val,
                 int val2,
                 long mask) {
    struct ad7606_state *adc = iio_priv(indio_dev);
    int ret = -EINVAL;
    int i;
    int size;

    mutex_lock(&adc->lock);
    switch (mask) {
    case IIO_CHAN_INFO_SAMP_FREQ:
        ret = ad7606_set_sampling_frequency(adc, val);
        break;
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        if (val2) {
            break;
        }
        size = adc->info->num_oversamplings;
        i = find_closest(val, adc->info->oversampling_avail, size);
        ret = ad7606_reg_write(adc, REG_OVERSAMPLING, i);
        if (ret < 0) {
            break;
        }
        ret = ad7606_set_sampling_frequency(adc, ad7606c_oversampling_pwm_rate[i]);
        break;
    case IIO_CHAN_INFO_SCALE:
        size = adc->info->num_scales;
        i = find_closest(val2, adc->info->scale_avail, size);
        ret = ad7606_reg_write_mask(adc,
				     REG_CH_RANGE(chan->address),
				     RANGE_CH_MASK(chan->address),
				     RANGE_CH_MODE(chan->address, i));
        break;
    case IIO_CHAN_INFO_HARDWAREGAIN:
        ret = ad7606_reg_write(adc, REG_CH_GAIN(chan->address), val & GENMASK(5, 0));
        break;
    case IIO_CHAN_INFO_OFFSET:
        ret = ad7606_reg_write(adc, REG_CH_OFFSET(chan->address), val);
        break;
    case IIO_CHAN_INFO_PHASE:
        ret = ad7606_reg_write(adc, REG_CH_PHASE(chan->address), val);
        break;
    }

    mutex_unlock(&adc->lock);
    return ret;
}

static int ad7606_spi_init(struct ad7606_state *adc) {
    int ret;

    switch (adc->device_id) {
    case ID_AD7606C_16:
    case ID_AD7606C_18:
        ret = ad7606_reg_write(adc, REG_CONFIG, AD7606_8_DOUT_LINES);
        if (ret) {
            goto end;
        }
        adc->sampling_frequency = ad7606c_oversampling_pwm_rate[0];
        break;
    default:
        return -EINVAL;
    }

end:
    return ret;
}

void ad7606_reg_disable(void *data) {
    regulator_disable((struct regulator*)data);
}

void ad7606_pwm_disable(void *data) {
    pwm_disable((struct pwm_device*)data);
}

void ad7606_clk_disable(void *data) {
    clk_disable_unprepare((struct clk*)data);
}

static int ad7606_buffer_postenable(struct iio_dev *indio_dev) {
    struct ad7606_state *adc = iio_priv(indio_dev);
    int ret = 0;
    uint32_t tx = -1;
    uint32_t rx;
    struct spi_transfer xfer = {
		.tx_buf = &tx,
        .rx_buf = &rx,
		.len = 4,
		.bits_per_word = adc->info->resolution,
		.speed_hz = adc->info->sclk_rate,
	};
    struct spi_message msg;
    spi_message_init_with_transfers(&msg, &xfer, 1);

    ret = spi_engine_offload_load_msg(adc->spi, &msg);
	if (ret < 0) {
		return ret;
    }

    spi_engine_offload_enable(adc->spi, true);

    return ret;
}

static int ad7606_buffer_postdisable(struct iio_dev *indio_dev) {
    struct ad7606_state *adc = iio_priv(indio_dev);
    int ret = 0;
    spi_engine_offload_enable(adc->spi, false);

    return ret;
}

static int ad7606_dma_submit(struct iio_dma_buffer_queue *queue,
                struct iio_dma_buffer_block *block) {
    return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static struct attribute *ad7606_attributes_os_and_range[] = {
    &iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7606_attribute_group_os_and_range = {
	.attrs = ad7606_attributes_os_and_range,
};

static const struct iio_info ad7606_iio_info = {
    .read_raw = &ad7606_read_raw,
    .write_raw = &ad7606_write_raw,
    .debugfs_reg_access = &ad7606_reg_access,
    .attrs = &ad7606_attribute_group_os_and_range,
};

static const struct iio_dma_buffer_ops ad7606_dma_buffer_ops = {
    .submit = ad7606_dma_submit,
    .abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad7606_buffer_ops = {
    .postenable = &ad7606_buffer_postenable,
    .postdisable = &ad7606_buffer_postdisable,
};

static int ad7606_init_channels(struct iio_dev *indio_dev) {
    struct ad7606_state *state = iio_priv(indio_dev);
    int storagebits, realbits;
    int i;
    
    switch (state->device_id) {
        case ID_AD7606C_16:
            storagebits = 16;
            realbits = 16;
            break;
        case ID_AD7606C_18:
            storagebits = 32;
            realbits = 18;
            break;
        default:
            return -EINVAL;
    }

    for(i=0; i < ARRAY_SIZE(ad7606c_base_channels); i++) {
        ad7606c_base_channels[i].scan_type.storagebits = storagebits;
        ad7606c_base_channels[i].scan_type.realbits = realbits;
    }
    indio_dev->channels = ad7606c_base_channels;
    indio_dev->num_channels = ARRAY_SIZE(ad7606c_base_channels);

    return 0;
}

static int ad7606_probe(struct spi_device *spi) {
    struct ad7606_state *adc;
    struct iio_dev *indio_dev;
    struct clk *ref_clk;
    struct iio_buffer *buffer;
    const void* id;
    int ret;

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
    if (!indio_dev) {
        return -ENOMEM;
    }

    adc = iio_priv(indio_dev);
    adc->spi = spi;

    memset(&adc->tx, 0, ARRAY_SIZE(adc->tx) * sizeof(adc->tx[0]));
    mutex_init(&adc->lock);

    adc->vref = devm_regulator_get(&spi->dev, "vref");
    if (IS_ERR(adc->vref)) {
        return PTR_ERR(adc->vref);
    }
    ret = regulator_enable(adc->vref);
    if (ret) {
        dev_err(&spi->dev, "Failed to enable VREF regulator");
        return ret;
    }
    ret = devm_add_action_or_reset(&spi->dev, ad7606_reg_disable, (void*)adc->vref);
    if (ret < 0) {
        return ret;
    }

    ref_clk = devm_clk_get(&spi->dev, "ref_clk");
    if (IS_ERR(ref_clk)) {
        return PTR_ERR(ref_clk);
    }
    ret = clk_prepare_enable(ref_clk);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to enable ref_clk");
        return ret;
    }
    ret = devm_add_action_or_reset(&spi->dev, ad7606_clk_disable, (void*)ref_clk);
    if (ret < 0) {
        return ret;
    }
    adc->ref_clk_rate = clk_get_rate(ref_clk);

    adc->cnv = devm_pwm_get(&spi->dev, "cnv_gen");
    if (IS_ERR(adc->cnv)) {
        return PTR_ERR(adc->cnv);
    }
    ret = devm_add_action_or_reset(&spi->dev, ad7606_pwm_disable, (void*)adc->cnv);
    if (ret < 0) {
        return ret;
    }

    id = device_get_match_data(&spi->dev);
    if (!id) {
        dev_err(&spi->dev, "Failed to match device info");
        return -EINVAL;
    }

    adc->device_id = (int)id;
    adc->info = &ad7606_chip_infos[adc->device_id];

    indio_dev->name = adc->info->name;
    indio_dev->info = &ad7606_iio_info;
    indio_dev->modes = INDIO_BUFFER_HARDWARE;
    indio_dev->setup_ops = &ad7606_buffer_ops;
    indio_dev->available_scan_masks = ad7606c_available_scan_masks;
    ret = ad7606_init_channels(indio_dev);
    if (ret) {
        dev_err(&adc->spi->dev, "Failed to init channels");
        return ret;
    }

    ret = ad7606_request_gpios(adc); 
    if (ret) {
        dev_err(&adc->spi->dev, "Failed to request gpios");
        return ret;
    }

    ret = ad7606_reset(adc, true); 
    if (ret) {
        dev_err(&adc->spi->dev, "Failed to reset ADC");
        return ret;
    }

    ret = ad7606_spi_init(adc);
    if (ret) {
        dev_err(&spi->dev, "Failed spi init of ADC");
        return ret;
    }

    buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
                         "rx",
                         &ad7606_dma_buffer_ops,
                         indio_dev);
    if (IS_ERR(buffer)) {
        dev_err(&adc->spi->dev, "Failed to alloc dma buffer");
        return PTR_ERR(buffer);
    }

    ret = ad7606_set_sampling_frequency(adc, adc->sampling_frequency);
    if (ret < 0) {
        dev_err(&adc->spi->dev, "Failed to set sampling frequency");
    }

    iio_device_attach_buffer(indio_dev, buffer);

    return devm_iio_device_register(&adc->spi->dev, indio_dev);
}

static struct spi_driver ad7606_driver = {
    .driver = {
        .name = "ad7606_si",
        .of_match_table = ad7606_of_match,
    },
    .probe = &ad7606_probe,
    .id_table = ad7606_spi_id,
};

module_spi_driver(ad7606_driver);

MODULE_AUTHOR("Bogdan Luncan <bogdan.luncan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606C driver");
MODULE_LICENSE("GPL v2");
