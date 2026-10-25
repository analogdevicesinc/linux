// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6397 PMIC AUXADC IIO driver
 *
 * Copyright (c) 2026 Ryan Brue <ryanbrue.dev@gmail.com>
 *
 * Based on drivers/iio/adc/mt6323-auxadc.c
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/time.h>
#include <linux/types.h>

#include <linux/mfd/mt6397/core.h>

#include <dt-bindings/iio/adc/mediatek,mt6397-auxadc.h>

/* The ready bit is in the raw result register, the value in the trimmed one. */
#define MT6397_AUXADC_ADC(n)			(0x0514 + 2 * (n))
#define   MT6397_AUXADC_ADC_RDY			BIT(15)
#define   MT6397_AUXADC_ADC_VAL			GENMASK(9, 0)
/* The trimmed copy of ADC(n), which is ADC11 + n. */
#define MT6397_AUXADC_ADC_TRIM(n)		(0x052a + 2 * (n))

/* The thermistor needs the battery-detect bias and the input buffer on. */
#define MT6397_AUXADC_CON0			0x0542
#define   MT6397_AUXADC_CON0_BUF_PWD_B		BIT(1)
#define   MT6397_AUXADC_CON0_BUF_PWD_ON		BIT(3)
/* Samples accumulated per conversion; the chip's accumulator is left at one. */
#define   MT6397_AUXADC_CON0_SPL_NUM		GENMASK(11, 7)
#define MT6397_CHR_CON7				0x000e
#define   MT6397_CHR_CON7_BATON_TDET_EN		BIT(2)

#define MT6397_AUXADC_BATTEMP_SETTLE_US		(20 * USEC_PER_MSEC)

#define MT6397_AUXADC_CON1			0x0544
#define   MT6397_AUXADC_CON1_CHSEL		GENMASK(10, 7)
#define   MT6397_AUXADC_CON1_START		BIT(0)

/*
 * CHR_CON16 and the two SOURCE_CH0 selects route either BATSNS or ISENSE onto
 * the battery channel. Only ISENSE is used: with a switching charger in the
 * power path BATSNS sits on the system rail rather than on the pack. All five
 * ADCIN bits are written out because they are one enable per analog input
 * and not a select field; the driver touches only the two it needs.
 */
#define MT6397_CHR_CON16			0x0020
#define   MT6397_CHR_CON16_ADCIN_VCHR_EN	BIT(12)
#define   MT6397_CHR_CON16_ADCIN_VSEN_EN	BIT(11)
#define   MT6397_CHR_CON16_ADCIN_VBAT_EN	BIT(10)
#define   MT6397_CHR_CON16_ADCIN_VSEN_EXT_BATON_EN	BIT(9)
#define   MT6397_CHR_CON16_ADCIN_VSEN_MUX_EN	BIT(8)
#define MT6397_AUXADC_CON14			0x055e
#define   MT6397_AUXADC_CON14_CH0_NORM_SEL	BIT(2)
#define   MT6397_AUXADC_CON14_CH0_LBAT_SEL	BIT(0)

#define MT6397_AUXADC_ISENSE_SETTLE_US		(1 * USEC_PER_MSEC)

/* Conversions averaged in software per read. */
#define MT6397_AUXADC_SAMPLES			16

/* Hardware limitation: the result is not ready for this long after START. */
#define MT6397_AUXADC_START_US			30

/* The battery channel is divided down by 4; the thermistor is not divided. */
#define MT6397_AUXADC_ISENSE_DIVIDER		4

/* The PMIC's channel numbers, as the CON1 select field takes them. */
#define MT6397_AUXADC_HWCHAN_BATSNS		0
#define MT6397_AUXADC_HWCHAN_BAT_TEMP		3

struct mt6397_auxadc {
	struct regmap *regmap;
	/*
	 * A read is a multi-register sequence: analog setup, then a channel
	 * select and an edge-triggered start per conversion. Interleaving two
	 * of them steals the select and tears down the setup, so a read holds
	 * this for its whole burst.
	 */
	struct mutex lock;
};

/*
 * A device tree channel ID indexes this array; .address holds the PMIC channel
 * number. The thermistor reports as a voltage because that is all the PMIC
 * measures: the divider across an NTC whose curve belongs to the board.
 */
static const struct iio_chan_spec mt6397_auxadc_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = MT6397_AUXADC_ISENSE,
		.address = MT6397_AUXADC_HWCHAN_BATSNS,
		.datasheet_name = "isense",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	}, {
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = MT6397_AUXADC_BAT_TEMP,
		.address = MT6397_AUXADC_HWCHAN_BAT_TEMP,
		.datasheet_name = "bat_temp",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int mt6397_auxadc_read_once(struct mt6397_auxadc *adc,
				   const struct iio_chan_spec *chan, int *val)
{
	struct regmap *map = adc->regmap;
	unsigned int reg;
	int ret;

	ret = regmap_update_bits(map, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_CHSEL,
				 FIELD_PREP(MT6397_AUXADC_CON1_CHSEL, chan->address));
	if (ret)
		return ret;

	/*
	 * START is edge triggered: lower it before raising it again. No delay
	 * is needed in between -- the ready bit survives the fall and is
	 * cleared by the rise, before the next transaction on this bus can
	 * observe it, so a ready bit seen by the poll can only be this
	 * conversion's.
	 */
	ret = regmap_clear_bits(map, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_START);
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_START);
	if (ret)
		return ret;

	fsleep(MT6397_AUXADC_START_US);

	ret = regmap_read_poll_timeout(map, MT6397_AUXADC_ADC(chan->address),
				       reg, reg & MT6397_AUXADC_ADC_RDY,
				       100, 100 * USEC_PER_MSEC);
	if (ret)
		return ret;

	ret = regmap_read(map, MT6397_AUXADC_ADC_TRIM(chan->address), &reg);
	if (ret)
		return ret;

	*val = FIELD_GET(MT6397_AUXADC_ADC_VAL, reg);

	return 0;
}

static int mt6397_auxadc_battemp_bias_on(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;
	int ret;

	ret = regmap_set_bits(map, MT6397_AUXADC_CON0,
			      MT6397_AUXADC_CON0_BUF_PWD_ON);
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6397_AUXADC_CON0,
			      MT6397_AUXADC_CON0_BUF_PWD_B);
	if (ret)
		return ret;

	return regmap_set_bits(map, MT6397_CHR_CON7,
			       MT6397_CHR_CON7_BATON_TDET_EN);
}

static void mt6397_auxadc_battemp_bias_off(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;

	regmap_clear_bits(map, MT6397_CHR_CON7, MT6397_CHR_CON7_BATON_TDET_EN);
	regmap_clear_bits(map, MT6397_AUXADC_CON0, MT6397_AUXADC_CON0_BUF_PWD_B);
	regmap_clear_bits(map, MT6397_AUXADC_CON0, MT6397_AUXADC_CON0_BUF_PWD_ON);
}

static int mt6397_auxadc_isense_enable(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;
	int ret;

	/*
	 * Each ADCIN bit connects one input to the battery channel. BATSNS is
	 * disconnected before ISENSE is connected, so that the two are never
	 * connected at the same time.
	 */
	ret = regmap_clear_bits(map, MT6397_CHR_CON16, MT6397_CHR_CON16_ADCIN_VBAT_EN);
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6397_CHR_CON16, MT6397_CHR_CON16_ADCIN_VSEN_EN);
	if (ret)
		return ret;

	return regmap_set_bits(map, MT6397_AUXADC_CON14,
			       MT6397_AUXADC_CON14_CH0_NORM_SEL |
			       MT6397_AUXADC_CON14_CH0_LBAT_SEL);
}

static void mt6397_auxadc_isense_disable(struct mt6397_auxadc *adc)
{
	struct regmap *map = adc->regmap;

	regmap_clear_bits(map, MT6397_AUXADC_CON14,
			  MT6397_AUXADC_CON14_CH0_NORM_SEL |
			  MT6397_AUXADC_CON14_CH0_LBAT_SEL);
	regmap_clear_bits(map, MT6397_CHR_CON16,
			  MT6397_CHR_CON16_ADCIN_VSEN_EN |
			  MT6397_CHR_CON16_ADCIN_VBAT_EN);
}

/*
 * One burst on a channel that is already prepared: the caller owns the lock,
 * the analog setup and the teardown.
 */
static int mt6397_auxadc_read_burst(struct mt6397_auxadc *adc,
				    const struct iio_chan_spec *chan, int *val)
{
	unsigned int sum = 0;
	int sample;
	int ret;

	for (unsigned int i = 0; i < MT6397_AUXADC_SAMPLES; i++) {
		ret = mt6397_auxadc_read_once(adc, chan, &sample);
		if (ret)
			return ret;

		sum += sample;
	}

	*val = DIV_ROUND_CLOSEST(sum, MT6397_AUXADC_SAMPLES);

	return 0;
}

static void mt6397_auxadc_stop(struct mt6397_auxadc *adc)
{
	/* Lowering START leaves the converter unarmed between reads. */
	regmap_clear_bits(adc->regmap, MT6397_AUXADC_CON1, MT6397_AUXADC_CON1_START);
}

static int mt6397_auxadc_isense_read(struct mt6397_auxadc *adc,
				     const struct iio_chan_spec *chan, int *val)
{
	int ret;

	ret = mt6397_auxadc_isense_enable(adc);
	if (ret)
		goto out_disable;

	fsleep(MT6397_AUXADC_ISENSE_SETTLE_US);

	ret = mt6397_auxadc_read_burst(adc, chan, val);

out_disable:
	mt6397_auxadc_stop(adc);
	mt6397_auxadc_isense_disable(adc);

	return ret;
}

static int mt6397_auxadc_battemp_read(struct mt6397_auxadc *adc,
				      const struct iio_chan_spec *chan, int *val)
{
	int ret;

	ret = mt6397_auxadc_battemp_bias_on(adc);
	if (ret)
		goto out_bias_off;

	fsleep(MT6397_AUXADC_BATTEMP_SETTLE_US);

	ret = mt6397_auxadc_read_burst(adc, chan, val);

out_bias_off:
	mt6397_auxadc_stop(adc);
	mt6397_auxadc_battemp_bias_off(adc);

	return ret;
}

static int mt6397_auxadc_read_channel(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      int *val)
{
	struct mt6397_auxadc *adc = iio_priv(indio_dev);

	guard(mutex)(&adc->lock);

	if (chan->channel == MT6397_AUXADC_ISENSE)
		return mt6397_auxadc_isense_read(adc, chan, val);

	return mt6397_auxadc_battemp_read(adc, chan, val);
}

static int mt6397_auxadc_read_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int *val, int *val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = mt6397_auxadc_read_channel(indio_dev, chan, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* 1200 mV full range with 10-bit resolution. */
		if (chan->channel == MT6397_AUXADC_ISENSE)
			*val = 1200 * MT6397_AUXADC_ISENSE_DIVIDER;
		else
			*val = 1200;

		*val2 = 10;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}
}

static const struct iio_info mt6397_auxadc_info = {
	.read_raw = mt6397_auxadc_read_raw,
};

static int mt6397_auxadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6397_chip *chip = dev_get_drvdata(dev->parent);
	struct mt6397_auxadc *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->regmap = chip->regmap;

	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	ret = regmap_update_bits(adc->regmap, MT6397_AUXADC_CON0,
				 MT6397_AUXADC_CON0_SPL_NUM,
				 FIELD_PREP(MT6397_AUXADC_CON0_SPL_NUM, 1));
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize auxadc\n");

	indio_dev->name = "mt6397-auxadc";
	indio_dev->info = &mt6397_auxadc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mt6397_auxadc_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt6397_auxadc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id mt6397_auxadc_of_match[] = {
	{ .compatible = "mediatek,mt6397-auxadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6397_auxadc_of_match);

static struct platform_driver mt6397_auxadc_driver = {
	.driver = {
		.name = "mt6397-auxadc",
		.of_match_table = mt6397_auxadc_of_match,
	},
	.probe = mt6397_auxadc_probe,
};
module_platform_driver(mt6397_auxadc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan Brue <ryanbrue.dev@gmail.com>");
MODULE_DESCRIPTION("MediaTek MT6397 PMIC AUXADC Driver");
