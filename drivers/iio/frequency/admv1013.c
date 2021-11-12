// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV1013 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

/* ADMV1013 Register Map */
#define ADMV1013_REG_SPI_CONTROL		0x00
#define ADMV1013_REG_ALARM			0x01
#define ADMV1013_REG_ALARM_MASKS		0x02
#define ADMV1013_REG_ENABLE			0x03
#define ADMV1013_REG_LO_AMP_I			0x05
#define ADMV1013_REG_LO_AMP_Q			0x06
#define ADMV1013_REG_OFFSET_ADJUST_I		0x07
#define ADMV1013_REG_OFFSET_ADJUST_Q		0x08
#define ADMV1013_REG_QUAD			0x09
#define ADMV1013_REG_VVA_TEMP_COMP		0x0A

/* ADMV1013_REG_SPI_CONTROL Map */
#define ADMV1013_PARITY_EN_MSK			BIT(15)
#define ADMV1013_PARITY_EN(x)			FIELD_PREP(ADMV1013_PARITY_EN_MSK, x)
#define ADMV1013_SPI_SOFT_RESET_MSK		BIT(14)
#define ADMV1013_SPI_SOFT_RESET(x)		FIELD_PREP(ADMV1013_SPI_SOFT_RESET_MSK, x)
#define ADMV1013_CHIP_ID_MSK			GENMASK(11, 4)
#define ADMV1013_CHIP_ID			0xA
#define ADMV1013_REVISION_ID_MSK		GENMASK(3, 0)
#define ADMV1013_REVISION_ID(x)			FIELD_PREP(ADMV1013_REVISION_ID_MSK, x)

/* ADMV1013_REG_ALARM Map */
#define ADMV1013_PARITY_ERROR_MSK		BIT(15)
#define ADMV1013_PARITY_ERROR(x)		FIELD_PREP(ADMV1013_PARITY_ERROR_MSK, x)
#define ADMV1013_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADMV1013_TOO_FEW_ERRORS(x)		FIELD_PREP(ADMV1013_TOO_FEW_ERRORS_MSK, x)
#define ADMV1013_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADMV1013_TOO_MANY_ERRORS(x)		FIELD_PREP(ADMV1013_TOO_MANY_ERRORS_MSK, x)
#define ADMV1013_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADMV1013_ADDRESS_RANGE_ERROR(x)		FIELD_PREP(ADMV1013_ADDRESS_RANGE_ERROR_MSK, x)

/* ADMV1013_REG_ENABLE Map */
#define ADMV1013_VGA_PD_MSK			BIT(15)
#define ADMV1013_VGA_PD(x)			FIELD_PREP(ADMV1013_VGA_PD_MSK, x)
#define ADMV1013_MIXER_PD_MSK			BIT(14)
#define ADMV1013_MIXER_PD(x)			FIELD_PREP(ADMV1013_MIXER_PD_MSK, x)
#define ADMV1013_QUAD_PD_MSK			GENMASK(13, 11)
#define ADMV1013_QUAD_PD(x)			FIELD_PREP(ADMV1013_QUAD_PD_MSK, x)
#define ADMV1013_BG_PD_MSK			BIT(10)
#define ADMV1013_BG_PD(x)			FIELD_PREP(ADMV1013_BG_PD_MSK, x)
#define ADMV1013_MIXER_IF_EN_MSK		BIT(7)
#define ADMV1013_MIXER_IF_EN(x)			FIELD_PREP(ADMV1013_MIXER_IF_EN_MSK, x)
#define ADMV1013_DET_EN_MSK			BIT(5)
#define ADMV1013_DET_EN(x)			FIELD_PREP(ADMV1013_DET_EN_MSK, x)

/* ADMV1013_REG_LO_AMP_I Map */
#define ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK	GENMASK(13, 7)
#define ADMV1013_LOAMP_PH_ADJ_I_FINE(x)		FIELD_PREP(ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK, x)
#define ADMV1013_MIXER_VGATE_MSK		GENMASK(6, 0)
#define ADMV1013_MIXER_VGATE(x)			FIELD_PREP(ADMV1013_MIXER_VGATE_MSK, x)

/* ADMV1013_REG_LO_AMP_Q Map */
#define ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK	GENMASK(13, 7)
#define ADMV1013_LOAMP_PH_ADJ_Q_FINE(x)		FIELD_PREP(ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK, x)

/* ADMV1013_REG_OFFSET_ADJUST_I Map */
#define ADMV1013_MIXER_OFF_ADJ_I_P_MSK		GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_I_P(x)		FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_I_P_MSK, x)
#define ADMV1013_MIXER_OFF_ADJ_I_N_MSK		GENMASK(8, 2)
#define ADMV1013_MIXER_OFF_ADJ_I_N(x)		FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_I_N_MSK, x)

/* ADMV1013_REG_OFFSET_ADJUST_Q Map */
#define ADMV1013_MIXER_OFF_ADJ_Q_P_MSK		GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_Q_P(x)		FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_Q_P_MSK, x)
#define ADMV1013_MIXER_OFF_ADJ_Q_N_MSK		GENMASK(8, 2)
#define ADMV1013_MIXER_OFF_ADJ_Q_N(x)		FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_Q_N_MSK, x)

/* ADMV1013_REG_QUAD Map */
#define ADMV1013_QUAD_SE_MODE_MSK		GENMASK(9, 6)
#define ADMV1013_QUAD_SE_MODE(x)		FIELD_PREP(ADMV1013_QUAD_SE_MODE_MSK, x)
#define ADMV1013_QUAD_FILTERS_MSK		GENMASK(3, 0)
#define ADMV1013_QUAD_FILTERS(x)		FIELD_PREP(ADMV1013_QUAD_FILTERS_MSK, x)

/* ADMV1013_REG_VVA_TEMP_COMP Map */
#define ADMV1013_VVA_TEMP_COMP_MSK		GENMASK(15, 0)
#define ADMV1013_VVA_TEMP_COMP(x)		FIELD_PREP(ADMV1013_VVA_TEMP_COMP_MSK, x)

enum supported_parts {
	ADMV1013,
};

struct admv1013_dev {
	struct spi_device	*spi;
	struct clk		*clkin;
	struct clock_scale	clkscale;
	/* Protect against concurrent accesses to the device */
	struct mutex		lock;
	struct regulator	*reg;
	struct notifier_block	nb;
	u64			clkin_freq;
	unsigned int		quad_se_mode;
	bool			parity_en;
	bool			vga_pd;
	bool			mixer_pd;
	bool			quad_pd;
	bool			bg_pd;
	bool			mixer_if_en;
	bool			det_en;
};

static int admv1013_spi_read(struct admv1013_dev *dev, unsigned int reg,
			      unsigned int *val)
{
	int ret;
	unsigned int cnt, temp;
	struct spi_transfer t = {0};
	u8 data[3];

	data[0] = 0x80 | (reg << 1);
	data[1] = 0x0;
	data[2] = 0x0;

	t.rx_buf = &data[0];
	t.tx_buf = &data[0];
	t.len = 3;

	ret = spi_sync_transfer(dev->spi, &t, 1);
	if (ret < 0)
		return ret;

	temp = ((data[0] | 0x80 | (reg << 1)) << 16) |
		(data[1] << 8) | data[2];

	if (dev->parity_en) {
		cnt = hweight_long(temp);
		if (!(cnt % 2))
			return -EINVAL;
	}

	*val = (temp >> 1) & 0xFFFF;

	return ret;
}

static int admv1013_spi_write(struct admv1013_dev *dev,
				      unsigned int reg,
				      unsigned int val)
{
	unsigned int cnt;
	u8 data[3];

	val = (val << 1);

	if (dev->parity_en) {
		cnt = hweight_long((reg << 17) | val);
		if (cnt % 2 == 0)
			val |= 0x1;
	}

	data[0] = (reg << 1) | (val >> 16);
	data[1] = val >> 8;
	data[2] = val;

	return spi_write(dev->spi, &data[0], 3);
}

static int admv1013_spi_update_bits(struct admv1013_dev *dev, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	int ret;
	unsigned int data, temp;

	mutex_lock(&dev->lock);
	ret = admv1013_spi_read(dev, reg, &data);
	if (ret < 0)
		goto exit;

	temp = (data & ~mask) | (val & mask);

	ret = admv1013_spi_write(dev, reg, temp);

exit:
	mutex_unlock(&dev->lock);

	return ret;
}

static int admv1013_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct admv1013_dev *dev = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_OFFSET:
		if (chan->channel2 == IIO_MOD_I) {
			ret = admv1013_spi_read(dev, ADMV1013_REG_OFFSET_ADJUST_I, &data);
			if (ret < 0)
				return ret;

			*val = (data & ADMV1013_MIXER_OFF_ADJ_I_P_MSK) >> 9;
			*val2 = (data & ADMV1013_MIXER_OFF_ADJ_I_N_MSK) >> 2;
		} else {
			ret = admv1013_spi_read(dev, ADMV1013_REG_OFFSET_ADJUST_Q, &data);
			if (ret < 0)
				return ret;

			*val = (data & ADMV1013_MIXER_OFF_ADJ_Q_P_MSK) >> 9;
			*val2 = (data & ADMV1013_MIXER_OFF_ADJ_Q_N_MSK) >> 2;
		}

		return IIO_VAL_INT_MULTIPLE;
	case IIO_CHAN_INFO_PHASE:
		if (chan->channel2 == IIO_MOD_I) {
			ret = admv1013_spi_read(dev, ADMV1013_REG_LO_AMP_I, &data);
			if (ret < 0)
				return ret;

			*val = (data & ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK) >> 7;
		} else {
			ret = admv1013_spi_read(dev, ADMV1013_REG_LO_AMP_Q, &data);
			if (ret < 0)
				return ret;

			*val = (data & ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK) >> 7;
		}

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int admv1013_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct admv1013_dev *dev = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_OFFSET:
		val2 /= 100000;

		if (chan->channel2 == IIO_MOD_I)
			ret = admv1013_spi_update_bits(dev, ADMV1013_REG_OFFSET_ADJUST_I,
							ADMV1013_MIXER_OFF_ADJ_I_P_MSK |
							ADMV1013_MIXER_OFF_ADJ_I_N_MSK,
							ADMV1013_MIXER_OFF_ADJ_I_P(val) |
							ADMV1013_MIXER_OFF_ADJ_I_N(val2));
		else
			ret = admv1013_spi_update_bits(dev, ADMV1013_REG_OFFSET_ADJUST_Q,
							ADMV1013_MIXER_OFF_ADJ_Q_P_MSK |
							ADMV1013_MIXER_OFF_ADJ_Q_N_MSK,
							ADMV1013_MIXER_OFF_ADJ_Q_P(val) |
							ADMV1013_MIXER_OFF_ADJ_Q_N(val2));

		return ret;
	case IIO_CHAN_INFO_PHASE:
		if (chan->channel2 == IIO_MOD_I)
			return admv1013_spi_update_bits(dev, ADMV1013_REG_LO_AMP_I,
							ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK,
							ADMV1013_LOAMP_PH_ADJ_I_FINE(val));
		else
			return admv1013_spi_update_bits(dev, ADMV1013_REG_LO_AMP_Q,
							ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK,
							ADMV1013_LOAMP_PH_ADJ_Q_FINE(val));
	default:
		return -EINVAL;
	}
}

static int admv1013_update_quad_filters(struct admv1013_dev *dev)
{
	unsigned int filt_raw;

	if (dev->clkin_freq >= 5400000000 && dev->clkin_freq <= 7000000000)
		filt_raw = 15;
	else if (dev->clkin_freq >= 5400000000 && dev->clkin_freq <= 8000000000)
		filt_raw = 10;
	else if (dev->clkin_freq >= 6600000000 && dev->clkin_freq <= 9200000000)
		filt_raw = 5;
	else
		filt_raw = 0;

	return admv1013_spi_update_bits(dev, ADMV1013_REG_QUAD,
					ADMV1013_QUAD_FILTERS_MSK,
					ADMV1013_QUAD_FILTERS(filt_raw));
}

static int admv1013_update_mixer_vgate(struct admv1013_dev *dev)
{
	unsigned int vcm, mixer_vgate;

	vcm = regulator_get_voltage(dev->reg);

	if (vcm >= 0 && vcm <= 1800000)
		mixer_vgate = (2389 * vcm / 1000000 + 8100) / 100;
	else if (vcm > 1800000 && vcm <= 2600000)
		mixer_vgate = (2375 * vcm / 1000000 + 125) / 100;
	else
		return -EINVAL;

	return admv1013_spi_update_bits(dev, ADMV1013_REG_LO_AMP_I,
				 ADMV1013_MIXER_VGATE_MSK,
				 ADMV1013_MIXER_VGATE(mixer_vgate));
}

static int admv1013_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv1013_dev *dev = iio_priv(indio_dev);
	int ret;

	if (read_val)
		ret = admv1013_spi_read(dev, reg, read_val);
	else
		ret = admv1013_spi_write(dev, reg, write_val);

	return ret;
}

static const struct iio_info admv1013_info = {
	.read_raw = admv1013_read_raw,
	.write_raw = admv1013_write_raw,
	.debugfs_reg_access = &admv1013_reg_access,
};

static int admv1013_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct admv1013_dev *dev = container_of(nb, struct admv1013_dev, nb);
	struct clk_notifier_data *cnd = data;

	if (action == POST_RATE_CHANGE) {
		/* cache the new rate */
		dev->clkin_freq = clk_get_rate_scaled(cnd->clk, &dev->clkscale);

		return notifier_from_errno(admv1013_update_quad_filters(dev));
	}

	return NOTIFY_OK;
}

static void admv1013_clk_notifier_unreg(void *data)
{
	struct admv1013_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}

#define ADMV1013_CHAN(_channel, rf_comp) {			\
	.type = IIO_ALTVOLTAGE,					\
	.modified = 1,						\
	.output = 1,						\
	.indexed = 1,						\
	.channel2 = IIO_MOD_##rf_comp,				\
	.channel = _channel,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) |	\
		BIT(IIO_CHAN_INFO_OFFSET)			\
	}

static const struct iio_chan_spec admv1013_channels[] = {
	ADMV1013_CHAN(0, I),
	ADMV1013_CHAN(0, Q),
};

static int admv1013_init(struct admv1013_dev *dev)
{
	int ret;
	unsigned int chip_id, enable_reg, enable_reg_msk;
	bool temp_parity = dev->parity_en;

	dev->parity_en = false;

	/* Perform a software reset */
	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_SPI_CONTROL,
				 ADMV1013_SPI_SOFT_RESET_MSK,
				 ADMV1013_SPI_SOFT_RESET(1));
	if (ret < 0)
		return ret;

	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_SPI_CONTROL,
				 ADMV1013_SPI_SOFT_RESET_MSK,
				 ADMV1013_SPI_SOFT_RESET(0));
	if (ret < 0)
		return ret;

	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_SPI_CONTROL,
				 ADMV1013_PARITY_EN_MSK,
				 ADMV1013_PARITY_EN(temp_parity));
	if (ret < 0)
		return ret;

	dev->parity_en = temp_parity;

	ret = admv1013_spi_read(dev, ADMV1013_REG_SPI_CONTROL, &chip_id);
	if (ret < 0)
		return ret;

	chip_id = (chip_id & ADMV1013_CHIP_ID_MSK) >> 4;
	if (chip_id != ADMV1013_CHIP_ID)
		return -EINVAL;

	ret = admv1013_spi_write(dev, ADMV1013_REG_VVA_TEMP_COMP, 0xE700);
	if (ret < 0)
		return ret;

	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_QUAD,
					ADMV1013_QUAD_SE_MODE_MSK,
					ADMV1013_QUAD_SE_MODE(dev->quad_se_mode));
	if (ret < 0)
		return ret;

	ret = admv1013_update_mixer_vgate(dev);
	if (ret < 0)
		return ret;

	ret = admv1013_update_quad_filters(dev);
	if (ret < 0)
		return ret;

	enable_reg_msk = ADMV1013_VGA_PD_MSK |
			ADMV1013_MIXER_PD_MSK |
			ADMV1013_QUAD_PD_MSK |
			ADMV1013_BG_PD_MSK |
			ADMV1013_MIXER_IF_EN_MSK |
			ADMV1013_DET_EN_MSK;

	enable_reg = ADMV1013_VGA_PD(dev->vga_pd) |
			ADMV1013_MIXER_PD(dev->mixer_pd) |
			ADMV1013_QUAD_PD(dev->quad_pd ? 7 : 0) |
			ADMV1013_BG_PD(dev->bg_pd) |
			ADMV1013_MIXER_IF_EN(dev->mixer_if_en) |
			ADMV1013_DET_EN(dev->det_en);

	return admv1013_spi_update_bits(dev, ADMV1013_REG_ENABLE, enable_reg_msk, enable_reg);
}

static void admv1013_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void admv1013_reg_disable(void *data)
{
	regulator_disable(data);
}

static int admv1013_dt_parse(struct admv1013_dev *dev)
{
	int ret;
	struct spi_device *spi = dev->spi;

	dev->parity_en = of_property_read_bool(spi->dev.of_node, "adi,parity-en");
	dev->vga_pd = of_property_read_bool(spi->dev.of_node, "adi,vga-pd");
	dev->mixer_pd = of_property_read_bool(spi->dev.of_node, "adi,mixer-pd");
	dev->quad_pd = of_property_read_bool(spi->dev.of_node, "adi,quad-pd");
	dev->bg_pd = of_property_read_bool(spi->dev.of_node, "adi,bg-pd");
	dev->mixer_if_en = of_property_read_bool(spi->dev.of_node, "adi,mixer-if-en");
	dev->det_en = of_property_read_bool(spi->dev.of_node, "adi,det-en");

	ret = of_property_read_u32(spi->dev.of_node, "adi,quad-se-mode", &dev->quad_se_mode);
	if (ret < 0)
		dev->quad_se_mode = 12;

	dev->reg = devm_regulator_get(&spi->dev, "vcm");
	if (IS_ERR(dev->reg))
		return PTR_ERR(dev->reg);

	dev->clkin = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin))
		return PTR_ERR(dev->clkin);

	return of_clk_get_scale(spi->dev.of_node, NULL, &dev->clkscale);
}

static int admv1013_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admv1013_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	dev = iio_priv(indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv1013_info;
	indio_dev->name = "admv1013";
	indio_dev->channels = admv1013_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv1013_channels);

	dev->spi = spi;

	ret = admv1013_dt_parse(dev);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dev->reg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable specified Common-Mode Voltage!\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, admv1013_reg_disable,
					dev->reg);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1013_clk_disable, dev->clkin);
	if (ret < 0)
		return ret;

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, &dev->clkscale);

	dev->nb.notifier_call = admv1013_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1013_clk_notifier_unreg, dev);
	if (ret < 0)
		return ret;

	mutex_init(&dev->lock);

	ret = admv1013_init(dev);
	if (ret < 0) {
		dev_err(&spi->dev, "admv1013 init failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admv1013_id[] = {
	{ "admv1013", ADMV1013 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1013_id);

static const struct of_device_id admv1013_of_match[] = {
	{ .compatible = "adi,admv1013" },
	{},
};
MODULE_DEVICE_TABLE(of, admv1013_of_match);

static struct spi_driver admv1013_driver = {
	.driver = {
			.name = "admv1013",
			.of_match_table = admv1013_of_match,
		},
	.probe = admv1013_probe,
	.id_table = admv1013_id,
};
module_spi_driver(admv1013_driver);


MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV1013");
MODULE_LICENSE("GPL v2");
