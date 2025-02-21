// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD7768 ADC driver
 *
 * Copyright 2018-2025 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>

/* AD7768 registers definition */
#define AD7768_CH_STANDBY			0x00
#define AD7768_CH_MODE				0x01
#define AD7768_POWER_MODE			0x04
#define AD7768_DATA_CONTROL			0x06
#define AD7768_INTERFACE_CFG			0x07

#define AD7768_REG_GPIO_CONTROL			0x0E

/* AD7768_REG_GPIO_CONTROL */
#define AD7768_GPIO_UGPIO_ENABLE		BIT(7)

#define AD7768_GPIO_INPUT(x)			0x00
#define AD7768_GPIO_OUTPUT(x)			BIT(x)

#define AD7768_REG_GPIO_WRITE			0x0F
#define AD7768_REG_GPIO_READ			0x10

/* AD7768_CH_MODE */
#define AD7768_CH_MODE_FILTER_TYPE_MSK		BIT(3)
#define AD7768_CH_MODE_FILTER_TYPE_MODE(x)	(((x) & 0x1) << 3)
#define AD7768_CH_MODE_GET_FILTER_TYPE(x)	(((x) >> 3) & 0x1)
#define AD7768_CH_MODE_DEC_RATE_MSK		GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE_MODE(x)		(((x) & 0x7) << 0)

/* AD7768_POWER_MODE */
#define AD7768_POWER_MODE_POWER_MODE_MSK	GENMASK(5, 4)
#define AD7768_POWER_MODE_POWER_MODE(x)		(((x) & 0x3) << 4)
#define AD7768_POWER_MODE_GET_POWER_MODE(x)	(((x) >> 4) & 0x3)
#define AD7768_POWER_MODE_MCLK_DIV_MSK		GENMASK(1, 0)
#define AD7768_POWER_MODE_MCLK_DIV_MODE(x)	(((x) & 0x3) << 0)
#define ad7768_map_power_mode_to_regval(x)	((x) ? ((x) + 1) : 0)
#define ad7768_map_regval_to_power_mode(x)	((x) ? ((x) - 1) : 0)

/* AD7768_DATA_CONTROL */
#define AD7768_DATA_CONTROL_SPI_RESET_MSK	GENMASK(1, 0)
#define AD7768_DATA_CONTROL_SPI_RESET_1		0x03
#define AD7768_DATA_CONTROL_SPI_RESET_2		0x02
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK	BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC		BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR	0

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK	GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)	(4 - ffs(x))
#define AD7768_MAX_DCLK_DIV			8

#define AD7768_INTERFACE_CFG_CRC_SELECT_MSK	GENMASK(3, 2)
/* only 4 samples CRC calculation support exists */
#define AD7768_INTERFACE_CFG_CRC_SELECT		0x01

#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

#define AD7768_OUTPUT_MODE_TWOS_COMPLEMENT	0x01

#define SAMPLE_SIZE				32
#define MAX_FREQ_PER_MODE			6

#define AD7768_MAX_CHANNEL  8

enum ad7768_power_modes {
	AD7768_LOW_POWER_MODE,
	AD7768_MEDIAN_MODE,
	AD7768_FAST_MODE,
	AD7768_NUM_POWER_MODES
};

struct ad7768_freq_config {
	unsigned int freq;
	unsigned int dec_rate;
};

struct ad7768_avail_freq {
	unsigned int n_freqs;
	struct ad7768_freq_config freq_cfg[MAX_FREQ_PER_MODE];
};

struct ad7768_chip_info {
	unsigned int			id;
	const char *name;
	unsigned			num_channels;
	const struct iio_chan_spec channel[AD7768_MAX_CHANNEL];
};

struct ad7768_state {
	struct spi_device *spi;
	struct mutex lock;
	struct clk *mclk;
	struct gpio_chip gpiochip;
	u64 vref_nv;
	unsigned int datalines;
	unsigned int sampling_freq;
	enum ad7768_power_modes power_mode;
	const struct ad7768_chip_info *chip_info;
	struct ad7768_avail_freq avail_freq[AD7768_NUM_POWER_MODES];
	__be16 d16;
	struct iio_backend *back;

	unsigned int num_en_channels;
};

enum ad7768_device_ids {
	ID_AD7768,
	ID_AD7768_4
};

static const int ad7768_dec_rate[6] = {
	32, 64, 128, 256, 512, 1024
};

static const int ad7768_mclk_div[3] = {
	32, 8, 4
};

static const unsigned int ad7768_available_datalines[] = {
	1, 2, 8
};

static const unsigned int ad7768_4_available_datalines[] = {
	1, 4
};

static int ad7768_spi_reg_read(struct ad7768_state *st, unsigned int addr,
			       unsigned int *val)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d16,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = &st->d16,
			.len = 2,
		},
	};
	int ret;

	st->d16 = cpu_to_be16((AD7768_WR_FLAG_MSK(addr) << 8));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	*val = be16_to_cpu(st->d16);

	return ret;
}

static int ad7768_spi_reg_write(struct ad7768_state *st,
				unsigned int addr,
				unsigned int val)
{
	st->d16 = cpu_to_be16(((addr & 0x7F) << 8) | val);

	return spi_write(st->spi, &st->d16, sizeof(st->d16));
}

static int ad7768_spi_write_mask(struct ad7768_state *st,
				 unsigned int addr,
				 unsigned long int mask,
				 unsigned int val)
{
	unsigned int regval;
	int ret;

	ret = ad7768_spi_reg_read(st, addr, &regval);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return ad7768_spi_reg_write(st, addr, regval);
}

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval) {
		ret = ad7768_spi_reg_read(st, reg, readval);
		if (ret < 0)
			goto exit;
		ret = 0;
	} else {
		ret = ad7768_spi_reg_write(st, reg, writeval);
	}
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_sync(struct ad7768_state *st)
{
	int ret;

	ret = ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC_CLEAR);
	if (ret < 0)
		return ret;

	return ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC);
}

static int ad7768_set_clk_divs(struct ad7768_state *st,
			       unsigned int freq)
{
	unsigned int mclk, dclk, dclk_div, i;
	struct ad7768_freq_config f_cfg;
	unsigned int chan_per_doutx;
	int ret = 0;

	mclk = clk_get_rate(st->mclk);

	chan_per_doutx = st->chip_info->num_channels / st->datalines;

	for (i = 0; i < st->avail_freq[st->power_mode].n_freqs; i++) {
		f_cfg = st->avail_freq[st->power_mode].freq_cfg[i];
		if (freq == f_cfg.freq)
			break;
	}
	if (i == st->avail_freq[st->power_mode].n_freqs)
		return -EINVAL;

	dclk = f_cfg.freq * SAMPLE_SIZE * chan_per_doutx;
	if (dclk > mclk)
		return -EINVAL;

	/* Set dclk_div to the nearest power of 2 less than the original value */
	dclk_div = DIV_ROUND_CLOSEST_ULL(mclk, dclk);
	if (dclk_div > AD7768_MAX_DCLK_DIV)
		dclk_div = AD7768_MAX_DCLK_DIV;
	else if (hweight32(dclk_div) != 1)
		dclk_div = 1 << (fls(dclk_div) - 1);

	ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
			AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
			AD7768_INTERFACE_CFG_DCLK_DIV_MODE(dclk_div));
	if (ret < 0)
		return ret;

	return ad7768_spi_write_mask(st, AD7768_CH_MODE,
				     AD7768_CH_MODE_DEC_RATE_MSK,
				     AD7768_CH_MODE_DEC_RATE_MODE(f_cfg.dec_rate));
}

static int ad7768_set_sampling_freq(struct iio_dev *indio_dev,
				    unsigned int freq)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret = 0;

	if (!freq)
		return -EINVAL;

	mutex_lock(&st->lock);

	ret = ad7768_set_clk_divs(st, freq);
	if (ret < 0)
		goto freq_err;

	st->sampling_freq = freq;

freq_err:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_set_power_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	struct ad7768_avail_freq avail_freq;
	int max_mode_freq;
	unsigned int regval;
	int ret;

	st->power_mode = mode;

	regval = ad7768_map_power_mode_to_regval(mode);
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_POWER_MODE_MSK,
				    AD7768_POWER_MODE_POWER_MODE(regval));
	if (ret < 0)
		return ret;

	/* The values for the powermode correspond for mclk div */
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_MCLK_DIV_MSK,
				    AD7768_POWER_MODE_MCLK_DIV_MODE(regval));
	if (ret < 0)
		return ret;

	/* Set the max freq of the selected power mode */
	avail_freq = st->avail_freq[mode];
	max_mode_freq = avail_freq.freq_cfg[avail_freq.n_freqs - 1].freq;
	ret = ad7768_set_sampling_freq(indio_dev, max_mode_freq);
	if (ret < 0)
		return ret;

	ret = ad7768_sync(st);
	if (ret < 0)
		return ret;

	return ret;
}

static int ad7768_get_power_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int regval, power_mode;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_POWER_MODE, &regval);
	if (ret < 0)
		return ret;

	power_mode = AD7768_POWER_MODE_GET_POWER_MODE(regval);
	st->power_mode = ad7768_map_regval_to_power_mode(power_mode);

	return st->power_mode;
}

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7768_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ad7768_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int c;
	int ret;

	for (c = 0; c < st->chip_info->num_channels; c++) {
		if (test_bit(c, scan_mask))
			ret = iio_backend_chan_enable(st->back, c);
		else
			ret = iio_backend_chan_disable(st->back, c);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct ad7768_chip_info ad7768_chip_info = {
	.id = ID_AD7768,
	.name = "ad7768",
	.num_channels = 8,
};

static const struct ad7768_chip_info ad7768_4_chip_info = {
	.id = ID_AD7768_4,
	.name = "ad7768-4",
	.num_channels = 4,
};

static const struct iio_info ad7768_info = {
	.debugfs_reg_access = ad7768_reg_access,
	.read_raw = ad7768_read_raw,
	.write_raw = ad7768_write_raw,
	.update_scan_mode = ad7768_update_scan_mode,
};

static void ad7768_set_available_sampl_freq(struct ad7768_state *st)
{
	unsigned int mode;
	unsigned int dec;
	unsigned int mclk = clk_get_rate(st->mclk);
	struct ad7768_avail_freq *avail_freq;

	for (mode = 0; mode < AD7768_NUM_POWER_MODES; mode++) {
		avail_freq = &st->avail_freq[mode];
		for (dec = ARRAY_SIZE(ad7768_dec_rate); dec > 0; dec--) {
			struct ad7768_freq_config freq_cfg;

			freq_cfg.dec_rate = dec - 1;
			freq_cfg.freq = mclk / (ad7768_dec_rate[dec - 1] *
					ad7768_mclk_div[mode]);
			avail_freq->freq_cfg[avail_freq->n_freqs++] = freq_cfg;
		}
	}

	/* The max frequency is not supported in one data line configuration */
	if (st->datalines == 1)
		st->avail_freq[AD7768_FAST_MODE].n_freqs--;
}

static int ad7768_input_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_write_mask(st,
				    AD7768_REG_GPIO_CONTROL,
				    BIT(offset),
				    AD7768_GPIO_INPUT(offset));
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_output_gpio(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_write_mask(st,
				    AD7768_REG_GPIO_CONTROL,
				    BIT(offset),
				    AD7768_GPIO_OUTPUT(offset));
	if (ret < 0)
		goto out;

	ret = ad7768_spi_write_mask(st,
				    AD7768_REG_GPIO_WRITE,
				    BIT(offset),
				    (value << offset));
out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &val);
	if (ret < 0)
		goto out;

	if (val & BIT(offset))
		ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_WRITE, &val);
	else
		ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_READ, &val);
	if (ret < 0)
		goto out;

	ret = !!(val & BIT(offset));

out:
	mutex_unlock(&st->lock);

	return ret;
}

static void ad7768_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &val);
	if (ret < 0)
		goto out;

	if (val & BIT(offset))
		ad7768_spi_write_mask(st,
				      AD7768_REG_GPIO_WRITE,
				      BIT(offset),
				      (value << offset));

out:
	mutex_unlock(&st->lock);
}

static int ad7768_gpio_setup(struct ad7768_state *st)
{
	int ret;

	ret = ad7768_spi_reg_write(st,
				   AD7768_REG_GPIO_CONTROL,
				   AD7768_GPIO_UGPIO_ENABLE);
	if (ret < 0)
		return ret;

	st->gpiochip.label = st->chip_info->name;
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 5;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad7768_input_gpio;
	st->gpiochip.direction_output = ad7768_output_gpio;
	st->gpiochip.get = ad7768_get_gpio;
	st->gpiochip.set = ad7768_set_gpio;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static const struct iio_chan_spec ad7768_channel_template = {
	.type = IIO_VOLTAGE,
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.indexed = 1,
	.scan_type = {
		.sign = 's',
		.realbits = 24,
		.storagebits = 32,
	},
};

static int ad7768_parse_config(struct iio_dev *indio_dev,
			       struct device *dev)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	const unsigned int *available_datalines;
	struct iio_chan_spec *chan;
	unsigned int channel;
	unsigned int i, len;
	int chan_idx = 0;
	int ret;

	st->num_en_channels = device_get_child_node_count(dev);

	if (st->num_en_channels > st->chip_info->num_channels)
		return dev_err_probe(dev, -EINVAL, "Too many channels defined\n");

	chan = devm_kcalloc(indio_dev->dev.parent, st->num_en_channels,
			    sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = st->num_en_channels;

	ret = ad7768_spi_reg_write(st, AD7768_CH_STANDBY, 0xFF);
	if (ret < 0)
		return ret;

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to parse reg property of %pfwP\n", child);

		ret = ad7768_spi_write_mask(st,
					    AD7768_CH_STANDBY,
					    BIT(channel),
					    0);
		if (ret < 0)
			return ret;

		chan[chan_idx] = ad7768_channel_template;
		chan[chan_idx].address = chan_idx;
		chan[chan_idx].channel = channel;
		chan[chan_idx].scan_index = chan_idx;
		chan_idx++;
	}

	st->datalines = 1;
	ret = device_property_read_u32(&st->spi->dev, "adi,data-lines-number",
				       &st->datalines);
	if (ret) {
		dev_err(&st->spi->dev, "Missing \"data-lines-number\" property\n");
		return ret;
	}

	switch (st->chip_info->id) {
	case ID_AD7768:
		available_datalines = ad7768_available_datalines;
		len = ARRAY_SIZE(ad7768_available_datalines);
		break;
	case ID_AD7768_4:
		available_datalines = ad7768_4_available_datalines;
		len = ARRAY_SIZE(ad7768_4_available_datalines);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		if (available_datalines[i] == st->datalines)
			return 0;
	}

	return -EINVAL;
}

static int ad7768_probe(struct spi_device *spi)
{
	struct gpio_desc *gpio_reset;
	struct iio_dev *indio_dev;
	struct ad7768_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	mutex_init(&st->lock);

	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return -ENODEV;

	st->mclk = devm_clk_get_enabled(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);

	/* get out of reset state */
	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		fsleep(2);
		gpiod_set_value_cansleep(gpio_reset, 0);
		fsleep(1660);
	}

	ret = ad7768_parse_config(indio_dev, &spi->dev);
	if (ret < 0)
		return ret;

	ad7768_set_available_sampl_freq(st);

	ad7768_set_power_mode(indio_dev, NULL, AD7768_FAST_MODE);
	if (ret < 0)
		return ret;

	ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
				    AD7768_INTERFACE_CFG_CRC_SELECT_MSK,
				    AD7768_INTERFACE_CFG_CRC_SELECT);
	if (ret < 0)
		return ret;

	ret = ad7768_gpio_setup(st);
	if (ret < 0)
		return ret;

	indio_dev->name = st->chip_info->name;
	indio_dev->info = &ad7768_info;

	st->back = devm_iio_backend_get(&spi->dev, NULL);
	if (IS_ERR(st->back))
		return PTR_ERR(st->back);

	ret = devm_iio_backend_request_buffer(&spi->dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(&spi->dev, st->back);
	if (ret)
		return ret;

	ret = iio_backend_set_num_lanes(st->back, st->datalines);
	if (ret)
		return ret;
	ret = iio_backend_crc_enable(st->back);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad7768_of_match[]  = {
	{ .compatible = "adi,ad7768", .data = &ad7768_chip_info },
	{ .compatible = "adi,ad7768-4", .data = &ad7768_4_chip_info },
	{},
};

static const struct spi_device_id ad7768_spi_id[] = {
	{"ad7768", (kernel_ulong_t)&ad7768_chip_info},
	{"ad7768-4", (kernel_ulong_t)&ad7768_4_chip_info},
	{},
};
MODULE_DEVICE_TABLE(spi, ad7768_spi_id);

static struct spi_driver ad7768_driver = {
	.probe = ad7768_probe,
	.driver = {
		.name   = "ad7768",
		.of_match_table = ad7768_of_match,
	},
	.id_table = ad7768_spi_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 ADC driver");
MODULE_LICENSE("GPL");
