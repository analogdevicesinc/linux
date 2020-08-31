// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Linear Technology LTC2308 SPI ADC driver
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define LTC2308_DEVICE_ID		0x00
#define LTC2308_ADC_RESOLUTION		12
#define LTC2308_CFG_SINGLE_ENDED_CH	BIT(11)
#define LTC2308_CFG_SIGN		BIT(10)
#define LTC2308_CFG_CHAN_UNIPOLAR	BIT(7)
#define LTC2308_CFG_SLEEP_MODE_EN	BIT(6)
#define LTC2308_CFG_SLEEP_MODE_DIS	0x00

struct ltc2308_dev {
	struct regulator		*vref;
	struct spi_device		*spi;
	struct mutex			lock;
	struct gpio_desc		*gpio_cnv;
	unsigned short			sleep_mode;
	unsigned short			prev_ch;
	int				vref_voltage_mv;
};

static int ltc2308_read(struct ltc2308_dev *adc, unsigned short ch, int *val)
{
	unsigned short spi_tx_buffer;
	unsigned short spi_rx_buffer;
	struct spi_transfer xfer = {
		.tx_buf = &spi_tx_buffer,
		.rx_buf = &spi_rx_buffer,
		.len = 2,
		.bits_per_word = 16
	};
	int ret;

	spi_tx_buffer = ch;

	gpiod_set_value(adc->gpio_cnv, 1);
	/* According to page 18 of the datasheet, the High time of the
	 *  CONVST pin is tCYC - tWLCONVST = 1590 us
	 */
	usleep_range(2, 10);
	gpiod_set_value(adc->gpio_cnv, 0);
	ret = spi_sync_transfer(adc->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	/* Do a dummy read in order to get the data for the specified channel */
	if (adc->prev_ch != ch) {
		gpiod_set_value(adc->gpio_cnv, 1);
		/* According to page 18 of the datasheet, the High time of the
		 *  CONVST pin is tCYC - tWLCONVST = 1590 us
		 */
		usleep_range(2, 10);
		gpiod_set_value(adc->gpio_cnv, 0);

		ret = spi_sync_transfer(adc->spi, &xfer, 1);
		if (ret < 0)
			return ret;
	}
	*val = spi_rx_buffer;
	if (!(ch & LTC2308_CFG_CHAN_UNIPOLAR))
		*val = sign_extend32(*val, LTC2308_ADC_RESOLUTION - 1);
	adc->prev_ch = ch;

	return 0;
}

static int ltc2308_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ltc2308_dev *adc = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&adc->lock);
		ret = ltc2308_read(adc, chan->address, val);
		mutex_unlock(&adc->lock);
		if (ret < 0)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = adc->vref_voltage_mv;
		*val2 = LTC2308_ADC_RESOLUTION;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int lc2308_set_sleep_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				const unsigned int mode)
{
	struct ltc2308_dev *adc = iio_priv(indio_dev);
	int dummy;
	int ret;

	mutex_lock(&adc->lock);
	/* Dummy read used to enable sleep mode */
	ret = ltc2308_read(adc, adc->prev_ch | mode, &dummy);
	mutex_unlock(&adc->lock);
	if (ret == 0)
		adc->sleep_mode = mode;

	return ret;
}

static int lc2308_get_sleep_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct ltc2308_dev *adc = iio_priv(indio_dev);

	return adc->sleep_mode;
}

static void ltc2308_regulator_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static int ltc2308_setup(struct ltc2308_dev *adc)
{
	adc->gpio_cnv = devm_gpiod_get(&adc->spi->dev, "cnv", GPIOD_OUT_LOW);
	if (IS_ERR(adc->gpio_cnv))
		return PTR_ERR(adc->gpio_cnv);
	gpiod_direction_output(adc->gpio_cnv, 0);

	return 0;
}

static const struct iio_info ltc2308_info = {
	.read_raw = ltc2308_read_raw
};

static const char * const ltc2308_sleep_modes[] = {
	[LTC2308_CFG_SLEEP_MODE_DIS]	= "DISABLED",
	[LTC2308_CFG_SLEEP_MODE_EN]	= "ENABLED" };

static const struct iio_enum ltc2308_sleep_mode_enum = {
	.items = ltc2308_sleep_modes,
	.num_items = ARRAY_SIZE(ltc2308_sleep_modes),
	.set = lc2308_set_sleep_mode,
	.get = lc2308_get_sleep_mode
};

static struct iio_chan_spec_ext_info ltc2308_ext_info[] = {
	IIO_ENUM("sleep_mode",
		 IIO_SHARED_BY_ALL,
		 &ltc2308_sleep_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("sleep_mode",
				  IIO_SHARED_BY_ALL,
				  &ltc2308_sleep_mode_enum),
	{ }
};

#define LTC2308_CHAN(name)						\
	struct iio_chan_spec name = {					\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.address = LTC2308_CFG_CHAN_UNIPOLAR,			\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.ext_info = ltc2308_ext_info				\
	}

static int ltc2308_parse_dt_single_ended_chan(struct fwnode_handle *child,
					      struct iio_chan_spec *chan,
					      struct device *dev)
{
	LTC2308_CHAN(single_ended_chan);
	unsigned int idx;
	int ret;

	single_ended_chan.address |= LTC2308_CFG_SINGLE_ENDED_CH;

	ret = fwnode_property_read_u32(child, "reg", &idx);
	if (ret < 0 || idx > 7) {
		dev_err(dev,"Invalid channel index %d. Index should be less \
			than 8", idx);
		return -EINVAL;
	}

	single_ended_chan.channel = idx;
	single_ended_chan.address |= (idx / 2) << 8 |
				     ((idx % 2) ? LTC2308_CFG_SIGN : 0);

	if (fwnode_property_present(child, "bipolar"))
		single_ended_chan.address &= ~LTC2308_CFG_CHAN_UNIPOLAR;

	*chan = single_ended_chan;

	return 0;
}

static int ltc2308_parse_dt_diff_chan(struct fwnode_handle *child,
				      struct iio_chan_spec *chan,
				      struct device *dev)
{
	LTC2308_CHAN(differential_chan);
	unsigned int diff_ch[2];
	int ret;

	differential_chan.differential = true;
	ret = fwnode_property_read_u32_array(child, "diff-channels", diff_ch,
					     ARRAY_SIZE(diff_ch));
	if (ret < 0)
		return ret;
	if (diff_ch[0] > 7 || diff_ch[1] > 7) {
		dev_err(dev,"Invalid channel index %d. Index should be less \
			than 8", diff_ch[0] > 7 ? diff_ch[0] : diff_ch[1]);

		return -EINVAL;
	}
	else if ((diff_ch[0] % 2 == 0) && (diff_ch[1] != (diff_ch[0] + 1))){
		dev_err(dev,"Invalid differential channel pair (%d,%d) should \
			be (%d,%d)\n", diff_ch[0], diff_ch[1],
			diff_ch[0], diff_ch[0] + 1);

		return -EINVAL;
	}
	else if ((diff_ch[1] % 2 == 0) && (diff_ch[0] != (diff_ch[1] + 1))){
		dev_err(dev,"Invalid differential channel pair (%d,%d) should \
			be (%d,%d)\n", diff_ch[0], diff_ch[1],
			diff_ch[0], diff_ch[0] - 1);

		return -EINVAL;
	}

	differential_chan.channel = diff_ch[0];
	differential_chan.channel2 = diff_ch[1];
	differential_chan.address |= (diff_ch[0] / 2) << 8 |
			((diff_ch[0] < diff_ch[1]) ? 0 : LTC2308_CFG_SIGN);

	if (fwnode_property_present(child, "bipolar"))
		differential_chan.address &= ~LTC2308_CFG_CHAN_UNIPOLAR;

	*chan = differential_chan;

	return 0;
}

static int ltc2308_parse_dt(struct ltc2308_dev *adc)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(adc);
	struct iio_chan_spec *ltc2308_channels;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	unsigned int num_ch;
	int chan_idx = 0;
	int ret;

	fwnode = dev_fwnode(indio_dev->dev.parent);
	num_ch =  device_get_child_node_count(indio_dev->dev.parent);
	if (num_ch == 0)
		return -EINVAL;

	ltc2308_channels = devm_kzalloc(indio_dev->dev.parent,
					num_ch * sizeof(struct iio_chan_spec),
					GFP_KERNEL);

	fwnode_for_each_child_node(fwnode, child) {
		if (fwnode_property_present(child, "reg")) {
			ret = ltc2308_parse_dt_single_ended_chan(child,
				&ltc2308_channels[chan_idx],
				&adc->spi->dev);
			if (ret < 0)
				return ret;
		} else if (fwnode_property_present(child, "diff-channels")) {
			ret = ltc2308_parse_dt_diff_chan(child,
				&ltc2308_channels[chan_idx],
				&adc->spi->dev);
			if (ret < 0)
				return ret;
		} else {
			return -EINVAL;
		}
		chan_idx++;
	}

	indio_dev->num_channels = num_ch;
	indio_dev->channels = ltc2308_channels;

	return 0;
}

static int ltc2308_probe(struct spi_device *spi)
{
	struct ltc2308_dev *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	adc->vref = devm_regulator_get_optional(&spi->dev, "vref");
	if (!IS_ERR(adc->vref)) {
		ret = regulator_enable(adc->vref);
		if (ret) {
			dev_err(&spi->dev, "Failed to enable vref regulator\n");
			return ret;
		}
		ret = regulator_get_voltage(adc->vref);
		if (ret < 0)
			return ret;
		adc->vref_voltage_mv = ret / 1000;
		ret = devm_add_action_or_reset(&spi->dev,
					       ltc2308_regulator_disable,
					       adc->vref);
		if (ret)
			return ret;
	} else {
		/* Internal vref is used */
		adc->vref_voltage_mv = 2500;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ltc2308";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc2308_info;

	ret = ltc2308_parse_dt(adc);
	if (ret < 0)
		return ret;

	ret = ltc2308_setup(adc);
	if (ret < 0) {
		dev_err(&spi->dev, "setup failed\n");
		return ret;
	}
	mutex_init(&adc->lock);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2308_of_match[] = {
	{ .compatible = "adi,ltc2308" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2308_of_match);

static struct spi_driver ltc2308_driver = {
	.driver = {
		.name = "ltc2308",
		.of_match_table = ltc2308_of_match,
	},
	.probe = ltc2308_probe,
};
module_spi_driver(ltc2308_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Linear Technology LTC2308 ADC");
MODULE_LICENSE("Dual BSD/GPL");
