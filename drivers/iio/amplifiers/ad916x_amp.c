// SPDX-License-Identifier: GPL-2.0
/*
 * SPI Amplifier Driver for the A916x series
 *
 * Copyright 2019 Analog Devices Inc.
 *
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include "ad916x_amp.h"

#define AD916x_AMP_REG_SPI_INTFCONFA	0x0
#define AD916x_AMP_SDOACTIVE		GENMASK(4, 3)

#define AD916x_AMP_REG_POWERDOWN	0x10
#define AD916x_AMP_PD_ADCCLOCK		BIT(0)
#define AD916x_AMP_PD_ADCCLOCK_ENABLE	0
#define AD916x_AMP_PD_ADCCLOCK_DISABLE	AD916x_AMP_PD_ADCCLOCK
#define AD916x_AMP_PD_CMDACCURRENT	BIT(3)
#define AD916x_AMP_PD_CMDACCURRENT_ENABLE	0
#define AD916x_AMP_PD_CMDACCURRENT_DISABLE	AD916x_AMP_PD_CMDACCURRENT

#define AD916x_AMP_REG_TRIM_CM		0x18
#define AD916x_AMP_TRIM_CM_MASK		GENMASK(3, 0)

#define AD916x_AMP_REG_DCOUTPUTVOLTAGE	0x19

#define AD916x_AMP_REG_ADC_START		0x1b
#define AD916x_AMP_ST_ADC_CLKF_0		BIT(0)
#define AD916x_AMP_ST_ADC_CLKF_0_ENABLE		AD916x_AMP_REG_ADC_START
#define AD916x_AMP_ST_ADC_CLKF_0_DISABLE	0

#define AD916x_AMP_REG_ADC_EOC		0x1c
#define AD916x_AMP_ADC_EOC		BIT(0)
#define AD916x_AMP_ADC_EOC_DONE		AD916x_AMP_ADC_EOC
#define AD916x_AMP_ADC_EOC_IN_PROGRESS	0

#define AD916x_AMP_REG_ADC_RESULTS	0x1d

struct ad916x_amp_state {
	struct regmap *map;
	struct spi_device *spi;
	struct mutex lock;
};

static const struct regmap_config ad916x_amp_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
};

#define AD916x_VOS_ADJ_MIN_MV	(-250)
#define AD916x_VOS_ADJ_MAX_MV	350
#define AD916x_VOS_ADJ_MIN_TO_MAX	(AD916x_VOS_ADJ_MAX_MV - AD916x_VOS_ADJ_MIN_MV)
#define AD916x_AMP_VOUT_TRIM_MAX	(BIT(8) - 1)

static int ad916x_amp_vos_adj_mv_set(struct ad916x_amp_state *st, s16 vos_adj_mv)
{
	u8 vout_trim;
	int rc;

	if (vos_adj_mv < AD916x_VOS_ADJ_MIN_MV || vos_adj_mv > AD916x_VOS_ADJ_MAX_MV)
		return -EINVAL;

	vout_trim = DIV_ROUND_CLOSEST((vos_adj_mv - AD916x_VOS_ADJ_MIN_MV)
				      * AD916x_AMP_VOUT_TRIM_MAX,
				      AD916x_VOS_ADJ_MIN_TO_MAX);

	rc = regmap_write(st->map, AD916x_AMP_REG_DCOUTPUTVOLTAGE, vout_trim);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to set DC offset: %d\n", rc);
		return rc;
	}

	return 0;
}

static int ad916x_amp_vos_adj_mv_get(struct ad916x_amp_state *st, int *vos_adj_mv)
{
	unsigned int vout_trim;
	int rc;

	rc = regmap_read(st->map, AD916x_AMP_REG_DCOUTPUTVOLTAGE, &vout_trim);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to get DC offset: %d\n", rc);
		return rc;
	}

	*vos_adj_mv = DIV_ROUND_CLOSEST(vout_trim * AD916x_VOS_ADJ_MIN_TO_MAX,
					AD916x_AMP_VOUT_TRIM_MAX) +
					AD916x_VOS_ADJ_MIN_MV;

	return 0;
}

#define AD916x_ICM_MIN_UA	6400
#define AD916x_ICM_MAX_UA	30400
#define AD916x_ICM_MIN_TO_MAX	(AD916x_ICM_MAX_UA - AD916x_ICM_MIN_UA)
#define AD916x_AMP_ICM_MAX	(BIT(4) - 1)

int ad916x_amp_icm_ua_set(struct ad916x_amp_state *st, u16 icm_ua)
{
	u8 amp_icm;
	int rc;

	if (icm_ua < AD916x_ICM_MIN_UA || icm_ua > AD916x_ICM_MAX_UA)
		return -EINVAL;

	amp_icm = DIV_ROUND_CLOSEST((icm_ua - AD916x_ICM_MIN_UA)
				    * AD916x_AMP_ICM_MAX,
				    AD916x_ICM_MIN_TO_MAX);

	rc = regmap_update_bits(st->map, AD916x_AMP_REG_TRIM_CM,
				AD916x_AMP_TRIM_CM_MASK, amp_icm);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to set input common-mode "
			"current: %d\n", rc);
		return rc;
	}

	return 0;
}

static int ad916x_amp_icm_ua_get(struct ad916x_amp_state *st, int *icm_ua)
{
	unsigned int amp_icm;
	int rc;

	rc = regmap_read(st->map, AD916x_AMP_REG_TRIM_CM, &amp_icm);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to get input common-mode "
			"current: %d\n", rc);
		return rc;
	}

	amp_icm &= AD916x_AMP_TRIM_CM_MASK;

	*icm_ua = DIV_ROUND_CLOSEST(amp_icm * AD916x_ICM_MIN_TO_MAX,
				    AD916x_AMP_ICM_MAX) + AD916x_ICM_MIN_UA;

	return 0;
}

/**
 * The ADC can be configured to use a 2MHz clock or a 250kHz clock.
 * The conversion operation can take at most 17 clock cycles.
 * For a 2MHz clock, 17 cycles are around 8.5us, and for a 250kHz clock,
 * around 68us. Use 10us sleep periods, and an 80us timeout, to accomodate
 * this timing and leave room for error.
 */
#define AD916x_AMP_ADC_EOC_SLEEP_US	10
#define AD916x_AMP_ADC_EOC_TIMEOUT_US	80
#define AD916x_AMP_MV_BGA_NOMINAL	1090
static int ad916x_get_adc_mv(struct ad916x_amp_state *st, int *adc_mv)
{
	unsigned int val;
	int rc;

	mutex_lock(&st->lock);
	rc = regmap_update_bits(st->map, AD916x_AMP_REG_POWERDOWN,
				AD916x_AMP_PD_ADCCLOCK,
				AD916x_AMP_PD_ADCCLOCK_ENABLE);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to enable ADC clock: %d\n", rc);
		goto exit;
	}

	rc = regmap_update_bits(st->map, AD916x_AMP_REG_ADC_START,
				AD916x_AMP_ST_ADC_CLKF_0,
				AD916x_AMP_ST_ADC_CLKF_0_ENABLE);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to start ADC conversion: %d\n", rc);
		goto exit;
	}

	rc = regmap_read_poll_timeout(st->map, AD916x_AMP_REG_ADC_EOC, val,
				      (val & AD916x_AMP_ADC_EOC) == AD916x_AMP_ADC_EOC_DONE,
				      AD916x_AMP_ADC_EOC_SLEEP_US,
				      AD916x_AMP_ADC_EOC_TIMEOUT_US);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to get ADC end of conversion flag: %d\n", rc);
		goto exit;
	}

	rc = regmap_read(st->map, AD916x_AMP_REG_ADC_RESULTS, &val);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to get ADC output code: %d\n", rc);
		goto exit;
	}

	*adc_mv = DIV_ROUND_CLOSEST(AD916x_AMP_MV_BGA_NOMINAL * val, 255);

	rc = regmap_update_bits(st->map, AD916x_AMP_REG_POWERDOWN, AD916x_AMP_PD_ADCCLOCK,
				AD916x_AMP_PD_ADCCLOCK_DISABLE);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to disable ADC clock: %d\n", rc);
		goto exit;
	}

exit:
	mutex_unlock(&st->lock);
	return rc;
}

enum {
	AD916x_AMP_ICM_UA,
	AD916x_AMP_VOS_ADJ_MV,
	AD916x_AMP_ADC_MV,
};

static int ad916x_amp_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val,
			       int *val2,
			       long mask)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);
	int ret = 0;

	*val = 0;
	*val2 = 0;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_read(st->map, AD916x_AMP_REG_POWERDOWN, val);
		if (ret)
			return ret;

		*val = (*val & AD916x_AMP_PD_CMDACCURRENT)
		       == AD916x_AMP_PD_CMDACCURRENT_ENABLE;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->channel) {
		case AD916x_AMP_ICM_UA:
			return ad916x_amp_icm_ua_get(st, val2) ?: IIO_VAL_INT_PLUS_MICRO;
		case AD916x_AMP_VOS_ADJ_MV:
			return ad916x_amp_vos_adj_mv_get(st, val) ?: IIO_VAL_INT;
		case AD916x_AMP_ADC_MV:
			return ad916x_get_adc_mv(st, val) ?: IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ad916x_amp_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val,
				int val2,
				long mask)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val != 0 && val != 1)
			return -EINVAL;

		return regmap_update_bits(st->map, AD916x_AMP_REG_POWERDOWN,
					  AD916x_AMP_PD_CMDACCURRENT,
					  val == 1 ? AD916x_AMP_PD_CMDACCURRENT_ENABLE :
					  AD916x_AMP_PD_CMDACCURRENT_DISABLE);
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->channel) {
		case AD916x_AMP_VOS_ADJ_MV:
			return ad916x_amp_vos_adj_mv_set(st, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ad916x_amp_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	if (!readval)
		return regmap_write(st->map, reg, writeval);
	else
		return regmap_read(st->map, reg, readval);
}

static const struct iio_info ad916x_amp_info = {
	.read_raw = ad916x_amp_read_raw,
	.write_raw = ad916x_amp_write_raw,
	.debugfs_reg_access = &ad916x_amp_reg_access,
};

static struct iio_chan_spec ad916x_amp_chan_spec[] = {
	/* icm_ua */
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = AD916x_AMP_ICM_UA,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
	},
	/* vos_adj_mv */
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = AD916x_AMP_VOS_ADJ_MV,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.output = 1,
	},
	/* adc_mv */
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = AD916x_AMP_ADC_MV,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static void ad916x_amp_setup(const struct ad916x_amp_state *st)
{
	if (!(st->spi->mode & SPI_3WIRE))
		/* set SPI to 4-wire mode */
		regmap_write(st->map, AD916x_AMP_REG_SPI_INTFCONFA,
			     AD916x_AMP_SDOACTIVE);
}

#define AD916X_AMP_ICM_DEFAULT_UA	23800
static int ad916x_amp_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad916x_amp_state *st;
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct device_node *np = spi->dev.of_node;
	const char *dev_name;
	s32 sval;
	u32 uval;
	int rc;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Failed to alloc iio dev\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);

	st->map = devm_regmap_init_spi(spi, &ad916x_amp_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	st->spi = spi;
	spi_set_drvdata(spi, st);

	dev_name = np ? np->name : dev_id->name;

	ad916x_amp_setup(st);

	if (of_property_read_s32(np, "adi,vos-adj-mv", &sval) == 0) {
		rc = ad916x_amp_vos_adj_mv_set(st, sval);
		if (rc)
			return rc;
	}

	if (of_property_read_u32(np, "adi,icm-ua", &uval))
		uval = AD916X_AMP_ICM_DEFAULT_UA;

	rc = ad916x_amp_icm_ua_set(st, uval);
	if (rc)
		return rc;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = dev_name;
	indio_dev->channels = ad916x_amp_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(ad916x_amp_chan_spec);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad916x_amp_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad916x_amp_dt_id[] = {
	{ .compatible = "adi,ad9166-amp" },
	{},
};
MODULE_DEVICE_TABLE(of, ad916x_amp_dt_id);

static const struct spi_device_id ad916x_amp_id[] = {
	{ "ad9166-amp", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, ad916x_amp_id);

static struct spi_driver ad916x_amp_driver = {
	.driver = {
		.name = "ad916x-amp",
		.of_match_table = ad916x_amp_dt_id,
	},
	.probe = ad916x_amp_probe,
	.id_table = ad916x_amp_id,
};

module_spi_driver(ad916x_amp_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD916x Amplifier");
MODULE_LICENSE("GPL v2");
