// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 Linux Kernel Interface Implementation
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>

#include "adrv9104-linux.h"

#include "adi_adrv910x_hal.h"
#include "adi_common_error_types.h"

#define ADRV9104_SPI_MAX_SIZE	4096

static int adrv9104_spi_write(void *dev_hal, const u8 *tx, u32 len)
{
	struct adrv9104_hal_cfg *hal_cfg = dev_hal;
	int left = len;
	u32 to_wr;
	int ret;

	if (!hal_cfg)
		return ADI_COMMON_ERR_NULL_PARAM;

	do {
		to_wr = left > ADRV9104_SPI_MAX_SIZE ? ADRV9104_SPI_MAX_SIZE : left;
		ret = spi_write(hal_cfg->spi, &tx[len - left], to_wr);
		if (ret)
			return ADI_COMMON_ERR_SPI_FAIL;

		left -= to_wr;
	} while (left > 0);

	return ADI_COMMON_ERR_OK;
}

static int adrv9104_spi_read(void *dev_hal, const u8 *tx, u8 *rx, u32 len)
{
	struct adrv9104_hal_cfg *hal_cfg = dev_hal;
	int left = len;
	int ret;

	if (!dev_hal)
		return ADI_COMMON_ERR_NULL_PARAM;

	do {
		struct spi_transfer t = {
			.tx_buf	= &tx[len - left],
			.rx_buf	= &rx[len - left],
			.len = left > ADRV9104_SPI_MAX_SIZE ? ADRV9104_SPI_MAX_SIZE : left,
		};

		ret = spi_sync_transfer(hal_cfg->spi, &t, 1);
		if (ret < 0)
			return ADI_COMMON_ERR_SPI_FAIL;

		left -= t.len;
	} while (left > 0);

	return ADI_COMMON_ERR_OK;
}

static int adrv9104_hw_open(void *dev_hal)
{
	return ADI_COMMON_ERR_OK;
}

static int adrv9104_hw_close(void *dev_hal)
{
	return ADI_COMMON_ERR_OK;
}

static int adrv9104_hw_reset(void *dev_hal, u8 pin_level)
{
	struct adrv9104_hal_cfg *hal_cfg = dev_hal;

	if (!dev_hal)
		return ADI_COMMON_ERR_NULL_PARAM;
	if (!hal_cfg->reset_gpio)
		return ADI_COMMON_ERR_OK;

	/*
	 * The API just passes @pin_level with the desired level. However,
	 * the pin is active low, so the logic must be inverted before calling
	 * the gpio API's. Hence if we receive 0 from the API, we want to pass
	 * 1 to the GPIO API since we want our pin to be active!
	 */
	gpiod_set_value_cansleep(hal_cfg->reset_gpio, !pin_level);

	return ADI_COMMON_ERR_OK;
}

/* HAL function pointer assignments for ADRV910X API */
int (*adi_adrv910x_hal_open)(void *dev_hal) = adrv9104_hw_open;
int (*adi_adrv910x_hal_close)(void *dev_hal) = adrv9104_hw_close;
int (*adi_adrv910x_hal_spi_write)(void *dev_hal, const u8 *tx, u32 len) = adrv9104_spi_write;
int (*adi_adrv910x_hal_spi_read)(void *dev_hal, const u8 *tx, u8 *rx, u32 len) = adrv9104_spi_read;
int (*adi_adrv910x_hal_resetbPin_set)(void *dev_hal, u8 pin_level) = adrv9104_hw_reset;
