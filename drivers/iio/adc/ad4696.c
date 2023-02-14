// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. AD4696 Driver
 *
 * Copyright (C) 2023 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regulator/consumer.h>

#define AD4696_MAX_CH			16
#define AD4696_REALBITS			16

#define AD4696_REG_SPI_CONFIG_A		0x0000
#define AD4696_SPI_CONFIG_A_SW_RST_MSB	BIT(7)
#define AD4696_SPI_CONFIG_A_ADDR_DIR	BIT(5)
#define AD4696_SPI_CONFIG_A_SW_RST_LSB	BIT(0)

#define AD4696_REG_SPI_CONFIG_B		0x0001
#define AD4696_SPI_CONFIG_B_INST_MODE	BIT(7)
#define SINGLE_INST_MODE		1
#define STREAMING_MODE			0
#define AD4696_SPI_CONFIG_B_ADDR_LEN    BIT(3)
#define ADDR_LEN_7			1
#define ADDR_LEN_15			0

#define AD4696_REG_SPI_CONFIG_C		0x0010
#define AD4696_SPI_CONFIG_C_CRC_EN	GENMASK(7, 6)
#define AD4696_SPI_CONFIG_C_MB_STRICT	BIT(5)
#define SINGLE_DATA_PHASE		1
#define INDIVIDUAL_DATA_PHASES		0
#define AD4696_SPI_CONFIG_C_CRC_EN_N	GENMASK(1, 0)

#define AD4696_REG_SETUP		0x0020
#define AD4696_SETUP_ALERT_MODE		BIT(7)
#define AD4696_SETUP_SDO_STATE		BIT(6)
#define AD4696_SETUP_STATUS_EN		BIT(5)
#define AD4696_SETUP_LDO_EN		BIT(4)
#define AD4696_SETUP_SPI_MODE		BIT(2)
#define AD4696_SETUP_CYC_CTRL		BIT(1)

#define AD4696_REG_SEQ_CTRL		0x0022
#define AD4696_SEQ_CTRL_STD_SEQ_EN	BIT(7)
#define AD4696_SEQ_CTRLNUM_SLOTS_AS(x)	((x) & 0x7F)

#define AD4696_REG_STD_SEQ_CONFIG	0x0024

#define AD4696_REG_TEMP_CTRL		0x0029
#define AD4696_TEMP_CTRL_TEMP_EN	BIT(0)

/* Status Bits */
#define STATUS_OV_ALT			BIT(4)
#define STATUS_INX			GENMASK(3, 0)

/* 5-Bit SDI Command (CMD)  */
#define AD4696_5_BIT_SDI_CMD(x)		((x) << 3)
#define CMD_REG_CONFIG_MODE		0x0A
#define CMD_TEMP_CH_SEL			0x0F
#define CMD_CONFIG_CH_SEL(x)		(0x10 | (0x0F & (x)))


#define AD4696_SWR_DELAY_US		310

struct ad4696_priv {
	struct spi_device	*spi;
	struct gpio_desc	*reset_gpio;
	struct regulator	*vref_reg;
	struct iio_chan_spec	channel[AD4696_MAX_CH];
};

static int ad4696_reg_read(struct ad4696_priv *priv,
			   unsigned int reg,
			   unsigned int *readval)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
	};
	int ret;

	tx_data[0] = ((reg >> 8) & 0xFF) | BIT(7);
	tx_data[1] = reg & 0xFF;
	tx_data[2] = 0;

	ret = spi_sync_transfer(priv->spi, &xfer, 1);
	if (ret)
		return ret;

	*readval = rx_data[2];

	return 0;
}

static int ad4696_reg_write(struct ad4696_priv *priv,
			    unsigned reg,
			    unsigned writeval)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
	};

	tx_data[0] = ((reg >> 8) & 0xFF);
	tx_data[1] = reg & 0xFF;
	tx_data[2] = writeval;

	return spi_sync_transfer(priv->spi, &xfer, 1);
}

static int ad4696_reg_write_mask(struct ad4696_priv *priv,
				 unsigned reg,
				 unsigned mask,
				 unsigned writeval)
{
	unsigned readval;
	int ret;

	ret = ad4696_reg_read(priv, reg, &readval);
	if (ret)
		return ret;

	readval &= ~mask;
	readval |= writeval;

	return ad4696_reg_write(priv, reg, readval);
}

static int ad4696_reg_access(struct iio_dev *indio_dev,
			     unsigned reg,
			     unsigned writeval,
			     unsigned *readval)
{
	struct ad4696_priv *priv = iio_priv(indio_dev);

	if (readval)
		return ad4696_reg_read(priv, reg, readval);

	return ad4696_reg_write(priv, reg, writeval);
}

static int ad4696_single_cycle_conv(struct ad4696_priv *priv,
				    unsigned chan,
				    unsigned *val)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
	};
	int ret;

	ret = ad4696_reg_write_mask(priv,
		AD4696_REG_SETUP, AD4696_SETUP_SPI_MODE,
		AD4696_SETUP_STATUS_EN | AD4696_SETUP_SPI_MODE |
		AD4696_SETUP_CYC_CTRL);
	if (ret)
		return ret;

	tx_data[0] = AD4696_5_BIT_SDI_CMD(CMD_CONFIG_CH_SEL(chan));
	tx_data[1] = 0;
	tx_data[2] = 0;

	ret = spi_sync_transfer(priv->spi, &xfer, 1);
	if (ret)
		return ret;

	tx_data[0] = AD4696_5_BIT_SDI_CMD(CMD_REG_CONFIG_MODE);
	ret = spi_sync_transfer(priv->spi, &xfer, 1);
	if (ret)
		return ret;

	if ((rx_data[2] & STATUS_INX) != chan)
		return -EIO;

	*val = (rx_data[0] << 8) | rx_data[1];

	return 0;
}

static int ad4696_setup(struct ad4696_priv *priv)
{
	int ret;
	int i;

	ret = ad4696_reg_write(priv, AD4696_REG_SPI_CONFIG_A,
			AD4696_SPI_CONFIG_A_SW_RST_MSB |
			AD4696_SPI_CONFIG_A_SW_RST_LSB);
	if (ret)
		return ret;

	usleep_range(AD4696_SWR_DELAY_US, AD4696_SWR_DELAY_US + 1);

	ret = ad4696_reg_write(priv, AD4696_REG_SPI_CONFIG_B,
			AD4696_SPI_CONFIG_B_INST_MODE);
	if (ret)
		return ret;

	ret = ad4696_reg_write(priv, AD4696_REG_SPI_CONFIG_C, 0);
	if (ret)
		return ret;

	ret = ad4696_reg_write(priv, AD4696_REG_SEQ_CTRL, 0);
	if (ret)
		return ret;

	for (i = 0; i < AD4696_MAX_CH; i++) {
		priv->channel[i].type = IIO_VOLTAGE;
		priv->channel[i].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		priv->channel[i].info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
		priv->channel[i].output = 0;
		priv->channel[i].indexed = 1;
		priv->channel[i].channel = i;
	}

	return 0;
};

static int ad4696_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct ad4696_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad4696_single_cycle_conv(priv, chan->channel, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(priv->vref_reg);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = AD4696_REALBITS;

		return IIO_VAL_FRACTIONAL_LOG2;
	}

	return -EINVAL;
}

static const struct iio_info ad4696_info = {
	.read_raw = &ad4696_read_raw,
	.debugfs_reg_access = &ad4696_reg_access,
};

static int ad4696_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad4696_priv *priv;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	priv->spi = spi;

	priv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
		GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	priv->vref_reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(priv->vref_reg))
		return PTR_ERR(priv->vref_reg);

	ret = regulator_enable(priv->vref_reg);
	if (ret)
		return ret;

	ad4696_setup(priv);

	indio_dev->name = "ad4696";
	indio_dev->channels = priv->channel;
	indio_dev->num_channels = AD4696_MAX_CH;
	indio_dev->info = &ad4696_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad4696_driver = {
	.driver = {
		.name = "ad4696",
	},
	.probe = ad4696_probe,
};
module_spi_driver(ad4696_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("IIO ADI AD4696 Driver");
MODULE_LICENSE("GPL v2");
