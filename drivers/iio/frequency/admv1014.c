// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV1014 driver
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

/* ADMV1014 Register Map */
#define ADMV1014_REG_SPI_CONTROL		0x00
#define ADMV1014_REG_ALARM			0x01
#define ADMV1014_REG_ALARM_MASKS		0x02
#define ADMV1014_REG_ENABLE			0x03
#define ADMV1014_REG_QUAD			0x04
#define ADMV1014_REG_LO_AMP_PHASE_ADJUST1	0x05
#define ADMV1014_REG_MIXER			0x07
#define ADMV1014_REG_IF_AMP			0x08
#define ADMV1014_REG_IF_AMP_BB_AMP		0x09
#define ADMV1014_REG_BB_AMP_AGC			0x0A
#define ADMV1014_REG_VVA_TEMP_COMP		0x0B

/* ADMV1014_REG_SPI_CONTROL Map */
#define ADMV1014_PARITY_EN_MSK			BIT(15)
#define ADMV1014_PARITY_EN(x)			FIELD_PREP(ADMV1014_PARITY_EN_MSK, x)
#define ADMV1014_SPI_SOFT_RESET_MSK		BIT(14)
#define ADMV1014_SPI_SOFT_RESET(x)		FIELD_PREP(ADMV1014_SPI_SOFT_RESET_MSK, x)
#define ADMV1014_CHIP_ID_MSK			GENMASK(11, 4)
#define ADMV1014_CHIP_ID			0x9
#define ADMV1014_REVISION_ID_MSK		GENMASK(3, 0)
#define ADMV1014_REVISION_ID(x)			FIELD_PREP(ADMV1014_REVISION_ID_MSK, x)

/* ADMV1014_REG_ALARM Map */
#define ADMV1014_PARITY_ERROR_MSK		BIT(15)
#define ADMV1014_PARITY_ERROR(x)		FIELD_PREP(ADMV1014_PARITY_ERROR_MSK, x)
#define ADMV1014_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADMV1014_TOO_FEW_ERRORS(x)		FIELD_PREP(ADMV1014_TOO_FEW_ERRORS_MSK, x)
#define ADMV1014_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADMV1014_TOO_MANY_ERRORS(x)		FIELD_PREP(ADMV1014_TOO_MANY_ERRORS_MSK, x)
#define ADMV1014_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADMV1014_ADDRESS_RANGE_ERROR(x)         FIELD_PREP(ADMV1014_ADDRESS_RANGE_ERROR_MSK, x)

/* ADMV1014_REG_ENABLE Map */
#define ADMV1014_IBIAS_PD_MSK			BIT(14)
#define ADMV1014_IBIAS_PD(x)			FIELD_PREP(ADMV1014_IBIAS_PD_MSK, x)
#define ADMV1014_P1DB_COMPENSATION_MSK		GENMASK(13, 12)
#define ADMV1014_P1DB_COMPENSATION(x)		FIELD_PREP(ADMV1014_P1DB_COMPENSATION_MSK, x)
#define ADMV1014_IF_AMP_PD_MSK			BIT(11)
#define ADMV1014_IF_AMP_PD(x)			FIELD_PREP(ADMV1014_IF_AMP_PD_MSK, x)
#define ADMV1014_QUAD_BG_PD_MSK			BIT(9)
#define ADMV1014_QUAD_BG_PD(x)			FIELD_PREP(ADMV1014_QUAD_BG_PD_MSK, x)
#define ADMV1014_BB_AMP_PD_MSK			BIT(8)
#define ADMV1014_BB_AMP_PD(x)			FIELD_PREP(ADMV1014_BB_AMP_PD_MSK, x)
#define ADMV1014_QUAD_IBIAS_PD_MSK		BIT(7)
#define ADMV1014_QUAD_IBIAS_PD(x)		FIELD_PREP(ADMV1014_QUAD_IBIAS_PD_MSK, x)
#define ADMV1014_DET_EN_MSK			BIT(6)
#define ADMV1014_DET_EN(x)			FIELD_PREP(ADMV1014_DET_EN_MSK, x)
#define ADMV1014_BG_PD_MSK			BIT(5)
#define ADMV1014_BG_PD(x)			FIELD_PREP(ADMV1014_BG_PD_MSK, x)

/* ADMV1014_REG_QUAD Map */
#define ADMV1014_QUAD_SE_MODE_MSK		GENMASK(9, 6)
#define ADMV1014_QUAD_SE_MODE(x)		FIELD_PREP(ADMV1014_QUAD_SE_MODE_MSK, x)
#define ADMV1014_QUAD_FILTERS_MSK		GENMASK(3, 0)
#define ADMV1014_QUAD_FILTERS(x)		FIELD_PREP(ADMV1014_QUAD_FILTERS_MSK, x)

/* ADMV1014_REG_LO_AMP_PHASE_ADJUST1 Map */
#define ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK	GENMASK(15, 9)
#define ADMV1014_LOAMP_PH_ADJ_I_FINE(x)		FIELD_PREP(ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK, x)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK	GENMASK(8, 2)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE(x)		FIELD_PREP(ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK, x)

/* ADMV1014_REG_MIXER Map */
#define ADMV1014_MIXER_VGATE_MSK		GENMASK(15, 9)
#define ADMV1014_MIXER_VGATE(x)			FIELD_PREP(ADMV1014_MIXER_VGATE_MSK, x)
#define ADMV1014_DET_PROG_MSK			GENMASK(6, 0)
#define ADMV1014_DET_PROG(x)			FIELD_PREP(ADMV1014_DET_PROG_MSK, x)

/* ADMV1014_REG_IF_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_I_MSK	GENMASK(11, 8)
#define ADMV1014_IF_AMP_COARSE_GAIN_I(x)	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_I_MSK, x)
#define ADMV1014_IF_AMP_FINE_GAIN_Q_MSK		GENMASK(7, 4)
#define ADMV1014_IF_AMP_FINE_GAIN_Q(x)		FIELD_PREP(ADMV1014_IF_AMP_FINE_GAIN_Q_MSK, x)
#define ADMV1014_IF_AMP_FINE_GAIN_I_MSK		GENMASK(3, 0)
#define ADMV1014_IF_AMP_FINE_GAIN_I(x)		FIELD_PREP(ADMV1014_IF_AMP_FINE_GAIN_I_MSK, x)

/* ADMV1014_REG_IF_AMP_BB_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK	GENMASK(15, 12)
#define ADMV1014_IF_AMP_COARSE_GAIN_Q(x)	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK, x)
#define ADMV1014_BB_AMP_OFFSET_Q_MSK		GENMASK(9, 5)
#define ADMV1014_BB_AMP_OFFSET_Q(x)		FIELD_PREP(ADMV1014_BB_AMP_OFFSET_Q_MSK, x)
#define ADMV1014_BB_AMP_OFFSET_I_MSK		GENMASK(4, 0)
#define ADMV1014_BB_AMP_OFFSET_I(x)		FIELD_PREP(ADMV1014_BB_AMP_OFFSET_I_MSK, x)

/* ADMV1014_REG_BB_AMP_AGC Map */
#define ADMV1014_BB_AMP_REF_GEN_MSK		GENMASK(6, 3)
#define ADMV1014_BB_AMP_REF_GEN(x)		FIELD_PREP(ADMV1014_BB_AMP_REF_GEN_MSK, x)
#define ADMV1014_BB_AMP_GAIN_CTRL_MSK		GENMASK(2, 1)
#define ADMV1014_BB_AMP_GAIN_CTRL(x)		FIELD_PREP(ADMV1014_BB_AMP_GAIN_CTRL_MSK, x)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK	BIT(0)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM(x)	FIELD_PREP(ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK, x)

/* ADMV1014_REG_VVA_TEMP_COMP Map */
#define ADMV1014_VVA_TEMP_COMP_MSK		GENMASK(15, 0)
#define ADMV1014_VVA_TEMP_COMP(x)		FIELD_PREP(ADMV1014_VVA_TEMP_COMP_MSK, x)

enum supported_parts {
	ADMV1014,
};

struct admv1014_dev {
	struct spi_device	*spi;
	struct clk		*clkin;
	struct clock_scale	clkscale;
	struct notifier_block	nb;
	/* Protect against concurrent accesses to the device */
	struct mutex		lock;
	struct regulator	*reg;
	u64			clkin_freq;
	unsigned int		quad_se_mode;
	unsigned int		p1db_comp;
	unsigned int		det_prog;
	unsigned int		bb_amp_gain_ctrl;
	bool			parity_en;
	bool			ibias_pd;
	bool			if_amp_pd;
	bool			quad_bg_pd;
	bool			bb_amp_pd;
	bool			quad_ibias_pd;
	bool			det_en;
	bool			bg_pd;
};

static const int mixer_vgate_table[] = {106, 107, 108, 110, 111, 112, 113, 114, 117, 118, 119, 120, 122, 123, 44, 45};

static int admv1014_spi_read(struct admv1014_dev *dev, unsigned int reg,
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

static int admv1014_spi_write(struct admv1014_dev *dev,
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

static int __admv1014_spi_update_bits(struct admv1014_dev *dev, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	int ret;
	unsigned int data, temp;

	ret = admv1014_spi_read(dev, reg, &data);
	if (ret < 0)
		return ret;

	temp = (data & ~mask) | (val & mask);

	return admv1014_spi_write(dev, reg, temp);
}

static int admv1014_spi_update_bits(struct admv1014_dev *dev, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	int ret;

	mutex_lock(&dev->lock);
	ret = __admv1014_spi_update_bits(dev, reg, mask, val);
	mutex_unlock(&dev->lock);
	return ret;
}

static int admv1014_update_quad_filters(struct admv1014_dev *dev)
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

	return admv1014_spi_update_bits(dev, ADMV1014_REG_QUAD,
					ADMV1014_QUAD_FILTERS_MSK,
					ADMV1014_QUAD_FILTERS(filt_raw));
}

static int admv1014_update_vcm_settings(struct admv1014_dev *dev)
{
	unsigned int i, vcm_mv, vcm_comp, bb_sw_high_low_cm;
	int ret;

	vcm_mv = regulator_get_voltage(dev->reg) / 1000;
	for (i = 0; i < ARRAY_SIZE(mixer_vgate_table); i++) {
		vcm_comp = 1050 + (i * 50) + (i / 8 * 50);
		if (vcm_mv == vcm_comp) {
			ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_MIXER,
							ADMV1014_MIXER_VGATE_MSK,
							ADMV1014_MIXER_VGATE(mixer_vgate_table[i]));
			if (ret < 0)
				return ret;

			bb_sw_high_low_cm = ~(i / 8);

			return __admv1014_spi_update_bits(dev, ADMV1014_REG_BB_AMP_AGC,
							ADMV1014_BB_AMP_REF_GEN_MSK |
							ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK,
							ADMV1014_BB_AMP_REF_GEN(i) |
							ADMV1014_BB_SWITCH_HIGH_LOW_CM(bb_sw_high_low_cm));
		}
	}

	return -EINVAL;
}

static int admv1014_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct admv1014_dev *dev = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->channel2 == IIO_MOD_I) {
			ret = admv1014_spi_read(dev, ADMV1014_REG_IF_AMP, &data);
			if (ret < 0)
				return ret;

			*val = (data & ADMV1014_IF_AMP_COARSE_GAIN_I_MSK) >> 8;
			*val2 = data & ADMV1014_IF_AMP_FINE_GAIN_I_MSK;
		} else {
			mutex_lock(&dev->lock);

			ret = admv1014_spi_read(dev, ADMV1014_REG_IF_AMP_BB_AMP, &data);
			if (ret < 0) {
				mutex_unlock(&dev->lock);
				return ret;
			}

			*val = (data & ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK) >> 12;

			ret = admv1014_spi_read(dev, ADMV1014_REG_IF_AMP, &data);
			mutex_unlock(&dev->lock);
			if (ret < 0)
				return ret;

			*val2 = (data & ADMV1014_IF_AMP_FINE_GAIN_Q_MSK) >> 4;
		}

		return IIO_VAL_INT_MULTIPLE;
	case IIO_CHAN_INFO_OFFSET:
		ret = admv1014_spi_read(dev, ADMV1014_REG_IF_AMP_BB_AMP, &data);
		if (ret < 0)
			return ret;

		if (chan->channel2 == IIO_MOD_I)
			*val = data & ADMV1014_BB_AMP_OFFSET_I_MSK;
		else
			*val = (data & ADMV1014_BB_AMP_OFFSET_Q_MSK) >> 5;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		ret = admv1014_spi_read(dev, ADMV1014_REG_LO_AMP_PHASE_ADJUST1, &data);
		if (ret < 0)
			return ret;

		if (chan->channel2 == IIO_MOD_I)
			*val = (data & ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK) >> 9;
		else
			*val = (data & ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK) >> 2;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int admv1014_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct admv1014_dev *dev = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		val = clamp_val(val, 0, 15);
		val2 /= 100000;
		val2 = clamp_val(val2, 0, 9);

		if (chan->channel2 == IIO_MOD_I) {
			ret = admv1014_spi_update_bits(dev, ADMV1014_REG_IF_AMP,
							ADMV1014_IF_AMP_COARSE_GAIN_I_MSK |
							ADMV1014_IF_AMP_FINE_GAIN_I_MSK,
							ADMV1014_IF_AMP_COARSE_GAIN_I(val) |
							ADMV1014_IF_AMP_FINE_GAIN_I(val2));
		} else {
			mutex_lock(&dev->lock);
			ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_IF_AMP_BB_AMP,
							ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK,
							ADMV1014_IF_AMP_COARSE_GAIN_Q(val));
			if (ret < 0) {
				mutex_unlock(&dev->lock);
				return ret;
			}

			ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_IF_AMP,
							ADMV1014_IF_AMP_FINE_GAIN_Q_MSK,
							ADMV1014_IF_AMP_FINE_GAIN_Q(val2));
			mutex_unlock(&dev->lock);
		}

		return ret;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->channel2 == IIO_MOD_I)
			return admv1014_spi_update_bits(dev, ADMV1014_REG_IF_AMP_BB_AMP,
							ADMV1014_BB_AMP_OFFSET_I_MSK,
							ADMV1014_BB_AMP_OFFSET_I(val));
		else
			return admv1014_spi_update_bits(dev, ADMV1014_REG_IF_AMP_BB_AMP,
							ADMV1014_BB_AMP_OFFSET_Q_MSK,
							ADMV1014_BB_AMP_OFFSET_Q(val));
	case IIO_CHAN_INFO_PHASE:
		if (chan->channel2 == IIO_MOD_I)
			return admv1014_spi_update_bits(dev, ADMV1014_REG_LO_AMP_PHASE_ADJUST1,
							ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK,
							ADMV1014_LOAMP_PH_ADJ_I_FINE(val));
		else
			return admv1014_spi_update_bits(dev, ADMV1014_REG_LO_AMP_PHASE_ADJUST1,
							ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK,
							ADMV1014_LOAMP_PH_ADJ_Q_FINE(val));
	default:
		return -EINVAL;
	}
}

static int admv1014_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv1014_dev *dev = iio_priv(indio_dev);
	int ret;

	if (read_val)
		ret = admv1014_spi_read(dev, reg, read_val);
	else
		ret = admv1014_spi_write(dev, reg, write_val);

	return ret;
}

static const struct iio_info admv1014_info = {
	.read_raw = admv1014_read_raw,
	.write_raw = admv1014_write_raw,
	.debugfs_reg_access = &admv1014_reg_access,
};

static int admv1014_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct admv1014_dev *dev = container_of(nb, struct admv1014_dev, nb);
	struct clk_notifier_data *cnd = data;

	if (action == POST_RATE_CHANGE) {
		/* cache the new rate */
		dev->clkin_freq = clk_get_rate_scaled(cnd->clk, &dev->clkscale);

		return notifier_from_errno(admv1014_update_quad_filters(dev));
	}

	return NOTIFY_OK;
}

static void admv1014_clk_notifier_unreg(void *data)
{
	struct admv1014_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}


#define ADMV1014_CHAN(_channel, rf_comp) {			\
	.type = IIO_ALTVOLTAGE,					\
	.modified = 1,						\
	.output = 1,						\
	.indexed = 1,						\
	.channel2 = IIO_MOD_##rf_comp,				\
	.channel = _channel,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE) |			\
		BIT(IIO_CHAN_INFO_OFFSET)			\
	}

static const struct iio_chan_spec admv1014_channels[] = {
	ADMV1014_CHAN(0, I),
	ADMV1014_CHAN(0, Q),
};

static int admv1014_init(struct admv1014_dev *dev)
{
	int ret;
	unsigned int chip_id, enable_reg, enable_reg_msk;
	struct spi_device *spi = dev->spi;
	bool temp_parity = dev->parity_en;

	dev->parity_en = false;

	/* Perform a software reset */
	ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_SPI_CONTROL,
				 ADMV1014_SPI_SOFT_RESET_MSK,
				 ADMV1014_SPI_SOFT_RESET(1));
	if (ret < 0) {
		dev_err(&spi->dev, "ADMV1014 SPI software reset failed.\n");
		return ret;
	}

	ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_SPI_CONTROL,
				 ADMV1014_SPI_SOFT_RESET_MSK,
				 ADMV1014_SPI_SOFT_RESET(0));
	if (ret < 0) {
		dev_err(&spi->dev, "ADMV1014 SPI software reset disable failed.\n");
		return ret;
	}

	ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_SPI_CONTROL,
				 ADMV1014_PARITY_EN_MSK,
				 ADMV1014_PARITY_EN(temp_parity));
	if (ret < 0) {
		dev_err(&spi->dev, "ADMV1014 Parity enable/disable failed.\n");
		return ret;
	}

	dev->parity_en = temp_parity;

	ret = admv1014_spi_write(dev, ADMV1014_REG_VVA_TEMP_COMP, 0x727C);
	if (ret < 0) {
		dev_err(&spi->dev, "Writing default Temperature Compensation value failed.\n");
		return ret;
	}

	ret = admv1014_spi_read(dev, ADMV1014_REG_SPI_CONTROL, &chip_id);
	if (ret < 0)
		return ret;

	chip_id = (chip_id & ADMV1014_CHIP_ID_MSK) >> 4;
	if (chip_id != ADMV1014_CHIP_ID) {
		dev_err(&spi->dev, "Invalid Chip ID.\n");
		return -EINVAL;
	}

	ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_QUAD,
				 ADMV1014_QUAD_SE_MODE_MSK,
				 ADMV1014_QUAD_SE_MODE(dev->quad_se_mode));
	if (ret < 0) {
		dev_err(&spi->dev, "Writing Quad SE Mode failed.\n");
		return ret;
	}

	ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_MIXER,
				 ADMV1014_DET_PROG_MSK,
				 ADMV1014_DET_PROG(dev->det_prog));
	if (ret < 0) {
		dev_err(&spi->dev, "Writing Digital Rx Detector failed.\n");
		return ret;
	}

	ret = __admv1014_spi_update_bits(dev, ADMV1014_REG_BB_AMP_AGC,
				 ADMV1014_BB_AMP_GAIN_CTRL_MSK,
				 ADMV1014_BB_AMP_GAIN_CTRL(dev->bb_amp_gain_ctrl));
	if (ret < 0) {
		dev_err(&spi->dev, "Writing Baseband Gain Control failed.\n");
		return ret;
	}

	ret = admv1014_update_quad_filters(dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Update Quad Filters failed.\n");
		return ret;
	}

	ret = admv1014_update_vcm_settings(dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Update VCM Settings failed.\n");
		return ret;
	}

	enable_reg_msk = ADMV1014_IBIAS_PD_MSK |
			ADMV1014_P1DB_COMPENSATION_MSK |
			ADMV1014_IF_AMP_PD_MSK |
			ADMV1014_QUAD_BG_PD_MSK |
			ADMV1014_BB_AMP_PD_MSK |
			ADMV1014_QUAD_IBIAS_PD_MSK |
			ADMV1014_DET_EN_MSK |
			ADMV1014_BG_PD_MSK;

	enable_reg = ADMV1014_IBIAS_PD(dev->ibias_pd) |
			ADMV1014_P1DB_COMPENSATION(dev->p1db_comp) |
			ADMV1014_IF_AMP_PD(dev->if_amp_pd) |
			ADMV1014_QUAD_BG_PD(dev->quad_bg_pd) |
			ADMV1014_BB_AMP_PD(dev->bb_amp_pd) |
			ADMV1014_QUAD_IBIAS_PD(dev->quad_ibias_pd)|
			ADMV1014_DET_EN(dev->det_en)|
			ADMV1014_BG_PD(dev->bg_pd);

	return __admv1014_spi_update_bits(dev, ADMV1014_REG_ENABLE, enable_reg_msk, enable_reg);
}

static void admv1014_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void admv1014_reg_disable(void *data)
{
	regulator_disable(data);
}

static int admv1014_dt_parse(struct admv1014_dev *dev)
{
	int ret;
	struct spi_device *spi = dev->spi;

	dev->parity_en = of_property_read_bool(spi->dev.of_node, "adi,parity-en");
	dev->ibias_pd = of_property_read_bool(spi->dev.of_node, "adi,ibias-pd");
	dev->if_amp_pd = of_property_read_bool(spi->dev.of_node, "adi,if-amp-pd");
	dev->quad_bg_pd = of_property_read_bool(spi->dev.of_node, "adi,quad-bg-pd");
	dev->bb_amp_pd = of_property_read_bool(spi->dev.of_node, "adi,bb-amp-pd");
	dev->quad_ibias_pd = of_property_read_bool(spi->dev.of_node, "adi,quad-ibias-pd");
	dev->det_en = of_property_read_bool(spi->dev.of_node, "adi,det-en");
	dev->bg_pd = of_property_read_bool(spi->dev.of_node, "adi,bg-pd");

	ret = of_property_read_u32(spi->dev.of_node, "adi,p1db-comp", &dev->p1db_comp);
	if (ret < 0)
		dev->p1db_comp = 3;

	ret = of_property_read_u32(spi->dev.of_node, "adi,quad-se-mode", &dev->quad_se_mode);
	if (ret < 0)
		dev->quad_se_mode = 12;

	ret = of_property_read_u32(spi->dev.of_node, "adi,det-prog", &dev->det_prog);
	if (ret < 0)
		dev->det_prog = 8;

	ret = of_property_read_u32(spi->dev.of_node, "adi,bb-amp-gain-ctrl", &dev->bb_amp_gain_ctrl);
	if (ret < 0)
		dev->bb_amp_gain_ctrl = 0;

	dev->reg = devm_regulator_get(&spi->dev, "vcm");
	if (IS_ERR(dev->reg))
		return PTR_ERR(dev->reg);

	dev->clkin = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin))
		return PTR_ERR(dev->clkin);

	return of_clk_get_scale(spi->dev.of_node, NULL, &dev->clkscale);
}

static int admv1014_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admv1014_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	dev = iio_priv(indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv1014_info;
	indio_dev->name = "admv1014";
	indio_dev->channels = admv1014_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv1014_channels);

	dev->spi = spi;

	ret = admv1014_dt_parse(dev);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dev->reg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable specified Common-Mode Voltage!\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, admv1014_reg_disable,
					dev->reg);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1014_clk_disable, dev->clkin);
	if (ret < 0)
		return ret;

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, &dev->clkscale);

	dev->nb.notifier_call = admv1014_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1014_clk_notifier_unreg, dev);
	if (ret < 0)
		return ret;

	mutex_init(&dev->lock);

	ret = admv1014_init(dev);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admv1014_id[] = {
	{ "admv1014", ADMV1014 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1014_id);

static const struct of_device_id admv1014_of_match[] = {
	{ .compatible = "adi,admv1014" },
	{},
};
MODULE_DEVICE_TABLE(of, admv1014_of_match);

static struct spi_driver admv1014_driver = {
	.driver = {
			.name = "admv1014",
			.of_match_table = admv1014_of_match,
		},
	.probe = admv1014_probe,
	.id_table = admv1014_id,
};
module_spi_driver(admv1014_driver);


MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV1014");
MODULE_LICENSE("GPL v2");
