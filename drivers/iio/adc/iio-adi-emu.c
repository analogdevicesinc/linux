// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. Emulator Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#define ADI_EMU_REG_DEVICE_CONFIG	0x02
#define  ADI_EMU_MASK_POWER_DOWN	BIT(5)
#define ADI_EMU_REG_CNVST		0x03
#define  ADI_EMU_MASK_CNVST		BIT(0)
#define ADI_EMU_REG_CH0_DATA_HIGH	0x04
#define ADI_EMU_REG_CH0_DATA_LOW	0x05
#define ADI_EMU_REG_CH1_DATA_HIGH	0x06
#define ADI_EMU_REG_CH1_DATA_LOW	0x07

#define ADI_EMU_RD_MASK			BIT(7)
#define ADI_EMU_ADDR_MASK		GENMASK(14, 8)
#define ADI_EMU_VAL_MASK		GENMASK(7, 0)

struct adi_emu_state {
	struct spi_device *spi;
	bool enable;
};

static int adi_emu_spi_write(struct adi_emu_state *st, u8 reg, u8 val)
{
	u16 msg = 0;
	u16 tx = 0;
	struct spi_transfer xfer = {
		.tx_buf	= NULL,
		.len	= 0,
	};

	msg |= FIELD_PREP(ADI_EMU_ADDR_MSK, reg);
	msg |= FIELD_PREP(ADI_EMU_VAL_MSK, val);

	put_unaligned_be16(msg, &tx);

	xfer.tx_buf = &tx;
	xfer.len = sizeof(tx);

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int adi_emu_spi_read(struct adi_emu_state *st, u8 reg, u8 *val)
{
	u8 tx = 0;
	u8 rx = 0;
	int ret;
	struct spi_transfer xfer[] = {
		{
			.tx_buf	= NULL,
			.rx_buf	= NULL,
			.len	= 1,
		},
		{
			.tx_buf	= NULL,
			.rx_buf	= NULL,
			.len	= 1,
		},
	};

	tx |= ADI_EMU_RD_MASK;
	tx |= reg;

	xfer[0].tx_buf = &tx;
	xfer[1].rx_buf = &rx;

	ret = spi_sync_transfer(st->spi, xfer, 2);
	if (ret)
		return ret;

	*val = rx;

	return 0;
}

static int adi_emu_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct adi_emu_state *st = iio_priv(indio_dev);
	u8 high, low;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = st->enable;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ret = adi_emu_spi_write(st, ADI_EMU_REG_CNVST, ADI_EMU_MASK_CNVST);
		if (ret)
			return ret;

		if (chan->channel) {
			ret = adi_emu_spi_read(st, ADI_EMU_REG_CH1_DATA_HIGH,
					       &high);
			if (ret)
				return ret;

			ret = adi_emu_spi_read(st, ADI_EMU_REG_CH1_DATA_LOW,
					       &low);
			if (ret)
				return ret;
		} else {
			ret = adi_emu_spi_read(st, ADI_EMU_REG_CH0_DATA_HIGH,
					       &high);
			if (ret)
				return ret;

			ret = adi_emu_spi_read(st, ADI_EMU_REG_CH0_DATA_LOW,
					       &low);
			if (ret)
				return ret;
		}

		*val = (high << 8) | low;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adi_emu_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct adi_emu_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val)
			ret = adi_emu_spi_write(st, ADI_EMU_REG_DEVICE_CONFIG, 0);
		else
			ret = adi_emu_spi_write(st, ADI_EMU_REG_DEVICE_CONFIG,
						ADI_EMU_MASK_POWER_DOWN);

		st->enable = val;
		return ret;
	}

	return -EINVAL;
}

static int adi_emu_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adi_emu_state *st = iio_priv(indio_dev);

	if (readval)
		return adi_emu_spi_read(st, reg, (u8 *)readval);

	return adi_emu_spi_write(st, reg, writeval);
}

static const struct iio_info adi_emu_info = {
	.read_raw = &adi_emu_read_raw,
	.write_raw = &adi_emu_write_raw,
	.debugfs_reg_access = &adi_emu_reg_access,
};

static const struct iio_chan_spec adi_emu_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0,
		.indexed = 1,
		.channel = 0,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0,
		.indexed = 1,
		.channel = 1,
	}
};

static int adi_emu_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adi_emu_state *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->enable = false;
	st->spi = spi;

	indio_dev->name = "iio-adi-emu";
	indio_dev->channels = adi_emu_channels;
	indio_dev->num_channels = ARRAY_SIZE(adi_emu_channels);
	indio_dev->info = &adi_emu_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver adi_emu_driver = {
	.driver = {
		.name = "iio-adi-emu",
	},
	.probe = adi_emu_probe,
};
module_spi_driver(adi_emu_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("IIO ADI Emulator Driver");
MODULE_LICENSE("GPL v2");
