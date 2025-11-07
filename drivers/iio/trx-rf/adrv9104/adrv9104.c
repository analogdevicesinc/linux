// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 and similar RF Transceiver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include "linux/stddef.h"
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

#include <linux/iio/backend.h>
#include <linux/iio/iio.h>

#include "adi_adrv910x_common_types.h"
#include "adrv9104.h"

/* Do I need scan_index = -1? Navassa does not have it! */
#define ADRV9104_RX_CHAN(idx) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_ENABLE) |		\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
}

#define ADRV9104_RX_BUF_CHAN(idx, _si, _mod) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.modified = 1,						\
	.channel = idx,						\
	.channel2 = _mod,					\
	.scan_index = _si,					\
	.scan_type = {						\
		.sign = 'S',					\
		.realbits = 16,					\
		.storagebits = 16,				\
	},							\
}

#define ADRV9104_TX_CHAN {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_ENABLE) |		\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
}

#define ADRV9104_TX_BUF_CHAN(_si, _mod) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.modified = 1,						\
	.channel2 = _mod,					\
	.output = 1,						\
	.scan_index = _si,					\
	.scan_type = {						\
		.sign = 'S',					\
		.realbits = 16,					\
		.storagebits = 16,				\
	},							\
}

#define ADRV9104_TX_DDS_CHAN(_mod) {			\
	.type = IIO_ALTVOLTAGE,				\
	.indexed = 1,					\
	.modified = 1,					\
	.channel2 = _mod,				\
	.output = 1,					\
	.scan_index = -1,				\
}

/* Speak with Michael about LO's for RX being output channels */
#define ADRV9104_LO_CHAN(idx, out) {				\
	.type = IIO_ALTVOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),	\
	.output = out,						\
	.scan_index = -1,					\
}

static struct iio_chan_spec adrv9104_phy_chan[] = {
	ADRV9104_RX_CHAN(0),
	ADRV9104_LO_CHAN(0, false),
	ADRV9104_RX_BUF_CHAN(0, 0, IIO_MOD_I),
	ADRV9104_RX_BUF_CHAN(0, 1, IIO_MOD_Q),
	ADRV9104_RX_CHAN(1),
	ADRV9104_LO_CHAN(1, false),
	ADRV9104_RX_BUF_CHAN(1, 2, IIO_MOD_I),
	ADRV9104_RX_BUF_CHAN(1, 3, IIO_MOD_Q),
	ADRV9104_TX_CHAN,
	ADRV9104_LO_CHAN(0, true),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_I),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_Q),
	ADRV9104_TX_BUF_CHAN(4, IIO_MOD_I),
	ADRV9104_TX_BUF_CHAN(5, IIO_MOD_Q),
};

static const char * const adrv9104_rx_backend_names[] = {
	"rx1-backend",
	"rx2-backend",
};

static int adrv9104_backend_get(struct adrv9104_rf_phy *phy,
				struct adrv9104_chan *chan,
				struct iio_dev *indio_dev,
				const char *back_name)
{
	int ret;

	chan->back = devm_iio_backend_get(&indio_dev->dev, back_name);
	if (IS_ERR(chan->back))
		return PTR_ERR(chan->back);

	/* Make sure the core is disabled during device setup */
	iio_backend_disable(chan->back);

	ret = devm_iio_backend_request_buffer(phy->dev, chan->back, indio_dev);
	if (ret)
		return ret;

	if (chan->port == ADI_RX)
		return 0;

	/* Position for ADRV9104_TX_DDS_CHAN() channels */
	ret = iio_backend_extend_chan_spec(chan->back, &adrv9104_phy_chan[10]);
	if (ret)
		return ret;

	return iio_backend_extend_chan_spec(chan->back, &adrv9104_phy_chan[11]);
}

static int adrv9104_probe(struct spi_device *spi)
{
	struct adrv9104_rf_phy *phy;
	struct iio_dev *indio_dev;
	struct clk *dev_clk;
	int ret, c;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->dev = &spi->dev;
	phy->phy_dev.common.devHalInfo = &phy->hal;
	phy->hal.spi = spi;

	phy->hal.reset_gpio = devm_gpiod_get_optional(&spi->dev,
						      "reset", GPIOD_OUT_LOW);
	if (IS_ERR(phy->hal.reset_gpio))
		return PTR_ERR(phy->hal.reset_gpio);

	for (c = 0; c < ADRV9104_RX_MAX; c++) {
		phy->channels[c] = &phy->rx_channels[c].channel;
		phy->rx_channels[c].channel.idx = c;
		phy->rx_channels[c].channel.port = ADI_RX;

		ret = adrv9104_backend_get(phy, &phy->rx_channels[c].channel, indio_dev,
					   adrv9104_rx_backend_names[c]);
		if (ret)
			return ret;
	}

	phy->channels[ADRV9104_RX_MAX] = &phy->tx_channels[0].channel;
	phy->tx_channels[0].channel.idx = 0;
	phy->tx_channels[0].channel.port = ADI_TX;

	ret = adrv9104_backend_get(phy, &phy->tx_channels[0].channel, indio_dev, "tx-backend");
	if (ret)
		return ret;

	ret = devm_mutex_init(phy->dev, &phy->lock);
	if (ret)
		return ret;

	dev_clk = devm_clk_get_enabled(phy->dev, NULL);
	if (IS_ERR(dev_clk))
		return PTR_ERR(dev_clk);

	return 0;
}

static const struct of_device_id adrv9104_of_match[] = {
	{ .compatible = "adi,adrv9104" },
	{ }
};
MODULE_DEVICE_TABLE(of, adrv9104_of_match);

static const struct spi_device_id adrv9104_ids[] = {
	{ "adrv9104" },
	{ }
};
MODULE_DEVICE_TABLE(spi, adrv9104_ids);

static struct spi_driver adrv9104_driver = {
	.driver = {
		.name	= "adrv9104",
		.of_match_table = adrv9104_of_match,
	},
	.probe		= adrv9104_probe,
	.id_table	= adrv9104_ids,
};
module_spi_driver(adrv9104_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV9104 and similar RF Transceivers Driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_BACKEND);
