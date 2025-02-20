/*
 * Analog Devices MC-ADC Module
 *
 * Copyright 2024 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include "asm-generic/errno-base.h"
#include "linux/array_size.h"
#include "linux/stddef.h"
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iio/buffer.h>
#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/util_macros.h>

const unsigned int ad7405_dec_rates[] = {
	4096, 2048, 1024, 512, 256, 128, 64, 32,
};

struct ad7405_chip_info {
	char				*name;
	unsigned			num_channels;
	const unsigned long		*scan_masks;
	u64				max_rate;
	u64				min_rate;
	struct iio_chan_spec		channel[3];
	const unsigned long 		*available_masks;
};

struct ad7405_state {
	const struct ad7405_chip_info *chip_info;
	struct iio_backend *back;
	u64 sample_frequency;
	unsigned int sample_frequency_tbl[ARRAY_SIZE(ad7405_dec_rates)];
	u64 ref_frequency;
	struct clk *clk_in;
};


static void ad7405_fill_samp_freq_table(struct ad7405_state *st)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ad7405_dec_rates); i++)
		st->sample_frequency_tbl[i] = DIV_ROUND_CLOSEST_ULL(st->ref_frequency,
								    ad7405_dec_rates[i]);
}

static int ad7405_set_sampling_rate(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int samp_rate)
{
	struct ad7405_state *st = iio_priv(indio_dev);
	unsigned int dec_rate, idx;
	int ret;

	dec_rate = DIV_ROUND_CLOSEST_ULL(st->ref_frequency, samp_rate);
	idx = find_closest_descending(dec_rate, ad7405_dec_rates, 
			   ARRAY_SIZE(ad7405_dec_rates)); 
	dec_rate = ad7405_dec_rates[idx];
	ret = iio_backend_set_decimation_rate(st->back, chan->channel, dec_rate);
	if (ret)
		return ret;

	st->sample_frequency = DIV_ROUND_CLOSEST_ULL(st->ref_frequency, dec_rate);

	return 0;
}


static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	// struct ad7405_state *st = iio_priv(indio_dev);
	// unsigned i, ctrl;

	// for (i = 0; i < indio_dev->masklength; i++) {
	// 	ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

	// 	if (test_bit(i, scan_mask))
	// 		ctrl |= ADI_ENABLE;
	// 	else
	// 		ctrl &= ~ADI_ENABLE;

	// 	axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	// }

	return 0;
}

static int ad7405_read_raw(struct iio_dev *indio_dev, 
			   const struct iio_chan_spec *chan, int *val,
			   int *val2, long info)
{
	struct ad7405_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sample_frequency;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad7405_write_raw(struct iio_dev *indio_dev,
	 struct iio_chan_spec const *chan,
	 int val, int val2, long info)
{
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		return ad7405_set_sampling_rate(indio_dev, chan, val);
	default:
		return -EINVAL;
	}
}

static int ad7405_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	struct ad7405_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = st->sample_frequency_tbl;
		*length = ARRAY_SIZE(st->sample_frequency_tbl);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad7405_info = {
	.read_raw = &ad7405_read_raw,
	.write_raw = &ad7405_write_raw,
	.read_avail = &ad7405_read_avail,
	.update_scan_mode = &axiadc_update_scan_mode,
};

#define AIM_CHAN_NOCALIB(_chan, _bits, _sign) { \
	.type = IIO_VOLTAGE, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.indexed = 1, \
	  .channel = _chan, \
	  .scan_index = _chan, \
	  .scan_type = { \
		.sign = _sign, \
		.realbits = _bits, \
		.storagebits = (_bits) > 16 ? 32 : 16, \
	  }, \
}

static const unsigned long ad7405_channel_masks[] = {
	BIT(0),
	0,
};

static const struct ad7405_chip_info ad7405_chip_info = {
	.name = "ad7405",
	.max_rate = 20000000UL,
	.min_rate = 5000000UL,
	.num_channels = 1,
	.channel = {
		AIM_CHAN_NOCALIB(0, 16, 'u'),
	},
	.available_masks = ad7405_channel_masks,
};

static int ad7405_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct ad7405_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->chip_info = device_get_match_data(dev);
	platform_set_drvdata(pdev, indio_dev);

	st->clk_in = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(st->clk_in))
		return dev_err_probe(&pdev->dev,  PTR_ERR(st->clk_in),
				     "Failed to get and enable CLK_IN\n");

	st->ref_frequency = clk_get_rate(st->clk_in);
	if (st->ref_frequency < st->chip_info->min_rate || 
	    st->ref_frequency > st->chip_info->max_rate)
		return dev_err_probe(dev,  -EINVAL, "CLK_IN rate out of range\n");

	ad7405_fill_samp_freq_table(st);

	indio_dev->dev.parent = dev;
	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad7405_info;

	st->back = devm_iio_backend_get(dev, NULL);
	if (IS_ERR(st->back))
		return dev_err_probe(dev, PTR_ERR(st->back),
				     "failed to get IIO backend\n");

	ret = devm_iio_backend_request_buffer(dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(dev, st->back);
	if (ret)
		return ret;
	
	/* Set the highest sampling rate */
	ret = ad7405_set_sampling_rate(indio_dev, &indio_dev->channels[0],
				       st->chip_info->max_rate);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id ad7405_of_match[] = {
	{ .compatible = "adi,ad7405", .data = &ad7405_chip_info},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, ad7405_of_match);

static struct platform_driver ad7405_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = ad7405_of_match,
	},
	.probe = ad7405_probe,
};

module_platform_driver(ad7405_driver);

MODULE_AUTHOR("Jonathan Santos <jonathan.santos@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7405 ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS("IIO_BACKEND");