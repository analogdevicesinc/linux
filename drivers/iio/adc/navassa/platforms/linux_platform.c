// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information.
 * see the "LICENSE.txt" file in this zip file.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include "linux_platform.h"
#include "adi_common_error.h"
#include "adi_common_hal.h"

#define SPI_MAX_SIZE	4096

/**
 * \brief Writes a message to the currently open logFile specified in the
 *        adi_hal_LogCfg_t of the devHalCfg structure passed
 *
 * Uses the vfprintf functionality to allow the user to supply the format and
 * the number of argument's that will be logged.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logLevel the log level to be written into
 * \param comment the string to include in the line added to the log.
 * \param argp variable argument list to be printed
 *
 * \retval ADI_COMMON_ERR_OK Function completed successfully, no action required
 * \retval ADI_COMMON_ERR_NULL_PARAM The function has been called with a null pointer
 * \retval ADI_COMMON_ERR_API_FAIL If the function failed to flush to write
 */
int32_t linux_log_write(void *devHalCfg, uint32_t log_level, const char *comment, va_list argp)
{
	int ret;
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;
	struct device *dev = &hal_cfg->spi->dev;
	struct va_format vaf;
	char fmt[512] = { 0 };
	const char * const log_level_char[] = {
		[ADI_LOGLEVEL_TRACE] = "[TRACE]",
		[ADI_LOGLEVEL_DEBUG] = "[DEBUG]",
		[ADI_LOGLEVEL_INFO] = "[INFO]",
		[ADI_LOGLEVEL_WARN] = "[WARN]",
		[ADI_LOGLEVEL_ERROR] = "[ERROR]",
		[ADI_LOGLEVEL_FATAL] = "[FATAL]"
	};

	if (!hal_cfg)
		return ADI_COMMON_ERR_NULL_PARAM;

	if (log_level == ADI_LOGLEVEL_NONE)
		/* If logging disabled, exit gracefully */
		return ADI_COMMON_ERR_OK;
	else if (log_level > ADI_LOGLEVEL_FATAL)
		return ADI_COMMON_ERR_INV_PARAM;

	ret = snprintf(fmt, sizeof(fmt), "%s: %s", log_level_char[log_level], comment);
	if (ret < 0)
		return ADI_COMMON_ERR_API_FAIL;

	vaf.fmt = fmt;
	vaf.va = &argp;

	switch (log_level) {
	case ADI_LOGLEVEL_TRACE:
	case ADI_LOGLEVEL_DEBUG:
		dev_dbg_ratelimited(dev, "%pV", &vaf);
		break;
	case ADI_LOGLEVEL_INFO:
		dev_info(dev, "%pV", &vaf);
		break;
	case ADI_LOGLEVEL_WARN:
		dev_warn(dev, "%pV", &vaf);
		break;
	case ADI_LOGLEVEL_ERROR:
		dev_err(dev, "%pV", &vaf);
		break;
	case ADI_LOGLEVEL_FATAL:
		dev_crit(dev, "%pV", &vaf);
		break;
	}

	return ADI_COMMON_ERR_OK;
}

/**
 * \brief Write an array of 8-bit data to a SPI device
 *
 * The function will write numTxBytes number of bytes to the SPI device
 * selected in the devHalCfg structure.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param txData Pointer to byte array txData buffer that has numTxBytes number of bytes
 * \param numTxBytes The length of txData array
 *
 * \retval ADI_COMMON_ERR_OK function completed successfully, no action required
 * \retval ADI_COMMON_ERR_NULL_PARAM the function has been called with a null pointer
 * \retval ADI_COMMON_ERR_API_FAIL the data was not written successfully
 */
int32_t linux_spi_write(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes)
{
	u32 to_wr = 0;
	int remaining = numTxBytes;
	int ret;
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;

	if (!hal_cfg)
		return ADI_COMMON_ERR_NULL_PARAM;

	do {
		to_wr = (remaining > SPI_MAX_SIZE) ? SPI_MAX_SIZE : remaining;
		ret = spi_write(hal_cfg->spi, &txData[numTxBytes - remaining], to_wr);
		if (ret)
			return ADI_COMMON_ERR_API_FAIL;

		remaining -= to_wr;
	} while (remaining > 0);

	return ADI_COMMON_ERR_OK;
}

/**
 * \brief Read one or more bytes from the device specified by the devHalCfg structure
 *
 * The function will read numTxRxBytes number of bytes from the SPI device selected in
 * the devHalCfg parameter and store the resulting data sent by the device in the rxData
 * data buffer.
 *
 * For each byte in txData written to the device, a byte is read and returned by this
 * function at the pointer provided by the rxData parameter.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param txData Pointer to byte array that has numTxRxBytes number of bytes
 * \param rxData Pointer to byte array where read back data will be returned, that is at least numTxRxBytes in size.
 * \param numTxBytes The length of txData and rxData arrays
 *
 * \retval ADI_COMMON_ERR_OK function completed successfully, no action required
 * \retval ADI_COMMON_ERR_NULL_PARAM the function has been called with a null pointer
 * \retval ADI_COMMON_ERR_API_FAIL the data was not read successfully
 */
int32_t linux_spi_read(void *devHalCfg, const uint8_t txData[], uint8_t rxData[],
		       uint32_t numTxRxBytes)
{
	int remaining = numTxRxBytes;
	int ret;
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;

	if (!devHalCfg)
		return ADI_COMMON_ERR_NULL_PARAM;

	do {
		struct spi_transfer t = {
			.tx_buf	= &txData[numTxRxBytes - remaining],
			.rx_buf	= &rxData[numTxRxBytes - remaining],
			.len	= (remaining > SPI_MAX_SIZE) ? SPI_MAX_SIZE : remaining,
		};

		ret = spi_sync_transfer(hal_cfg->spi, &t, 1);
		if (ret < 0)
			return ADI_COMMON_ERR_API_FAIL;

		remaining -= t.len;
	} while (remaining > 0);

	return ADI_COMMON_ERR_OK;
}

/**
 * \brief Provides a blocking delay of the current thread
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param time_us the time to delay in mico seconds
 *
 * \retval ADI_COMMON_ERR_OK Function completed successfully
 */
int32_t linux_wait_us(void *devHalCfg, uint32_t time_us)
{
	usleep_range(time_us, time_us + 10);
	return ADI_COMMON_ERR_OK;
}

/**
 * \brief Opens all neccessary files and device drivers for a specific device
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_COMMON_ERR_OK Function completed successfully, no action required
 */
int32_t linux_hw_open(void *devHalCfg)
{
	return ADI_COMMON_ERR_OK;
}

/**
 * \brief Gracefully shuts down the the hardware closing any open resources
 *        such as log files, I2C, SPI, GPIO drivers, timer resources, etc.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_COMMON_ERR_OK Function completed successfully, no action required
 */
int32_t linux_hw_close(void *devHalCfg)
{
	return ADI_COMMON_ERR_OK;
}

/**
 * \brief This function control a BBIC GPIO pin that connects to the reset pin
 *        of each device.
 *
 *  This function is called by each device API giving access to the Reset pin
 *  connected to each device.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param pinLevel The desired pin logic level 0=low, 1=high to set the GPIO pin to.
 *
 * \retval ADI_COMMON_ERR_OK Function completed successfully, no action required
 * \retval ADI_COMMON_ERR_NULL_PARAM The function has been called with a null pointer
 */
int32_t linux_hw_reset(void *devHalCfg, uint8_t pinLevel)
{
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;

	if (!devHalCfg)
		return ADI_COMMON_ERR_NULL_PARAM;

	/*
	 * The API just passes @pinLevel with the desired level. However,
	 * the pin is active low, so the logic must be inverted before calling
	 * the gpio API's. Hence if we receive 0 from the API, we want to pass
	 * 1 to the GPIO API since we want our pin to be active!
	 */
	gpiod_set_value_cansleep(hal_cfg->reset_gpio, !pinLevel);

	return ADI_COMMON_ERR_OK;
}

int32_t linux_mcs_pulse(void* devHalCfg, uint8_t numberOfPulses)
{
	return ADI_COMMON_ERR_OK;
}

int32_t linux_ssi_reset(void* devHalCfg)
{
	return ADI_COMMON_ERR_OK;
}

int32_t linux_image_page_get(void *devHalCfg, const char *ImagePath, uint32_t pageIndex,
			     uint32_t pageSize, uint8_t *rdBuff)
{
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;
	struct spi_device *spi = hal_cfg->spi;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, ImagePath, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "request_firmware(%s) failed with %d\n", ImagePath, ret);
		return ADI_COMMON_ERR_API_FAIL;
	}

	if ((pageIndex * pageSize) > fw->size) {
		release_firmware(fw);
		return ADI_COMMON_ERR_INV_PARAM;
	}

	memcpy(rdBuff, &fw->data[pageIndex * pageSize], pageSize);

	release_firmware(fw);

	return 0;
}

int32_t linux_rx_gain_table_entry_get(void *devHalCfg, const char *rxGainTablePath,
				      uint16_t lineCount, uint8_t *gainIndex, uint8_t *rxFeGain,
				      uint8_t *tiaControl, uint8_t *adcControl, uint8_t *extControl,
				      uint16_t *phaseOffset, int16_t *digGain)
{
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;
	struct spi_device *spi = hal_cfg->spi;
	const struct firmware *fw;
	char *line;
	int i = 0, ret, cnt = 0;

	ret = request_firmware(&fw, rxGainTablePath, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "request_firmware(%s) failed with %d\n", rxGainTablePath, ret);
		return ADI_COMMON_ERR_API_FAIL;
	}

	do {
		line = strnchr(fw->data + cnt, fw->size - cnt, '\n');
		if (!line) {
			release_firmware(fw);
			return -EINVAL;
		}
		line++;
		cnt = line - (char *)fw->data;
	} while (i++ != lineCount && cnt < fw->size);

	/* EOF */
	if (cnt >= fw->size) {
	/* Setting index to 0 will break caller loop */
		*gainIndex = 0;
		release_firmware(fw);
		return 7;
	}

	line = skip_spaces(line);

	ret = sscanf(line, "%hhu,%hhu,%hhu,%hhu,%hhu,%hu,%hd", gainIndex, rxFeGain, tiaControl,
		     adcControl, extControl, phaseOffset, digGain);

	release_firmware(fw);

	return ret < 0 ? ADI_COMMON_ERR_API_FAIL : ret;
}

int32_t linux_tx_atten_table_entry_get(void *devHalCfg, const char *txAttenTablePath,
				       uint16_t lineCount, uint16_t *attenIndex, uint8_t *txAttenHp,
				       uint16_t *txAttenMult)
{
	struct adrv9002_hal_cfg *hal_cfg = devHalCfg;
	struct spi_device *spi = hal_cfg->spi;
	const struct firmware *fw;
	char *line;
	int i = 0, ret, cnt = 0;

	ret = request_firmware(&fw, txAttenTablePath, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "request_firmware(%s) failed with %d\n", txAttenTablePath, ret);
		return ADI_COMMON_ERR_API_FAIL;
	}

	do {
		line = strnchr(fw->data + cnt, fw->size - cnt, '\n');
		if (!line) {
			release_firmware(fw);
			return ADI_COMMON_ERR_API_FAIL;
		}
		line++;
		cnt = line - (char *)fw->data;
	} while (i++ != lineCount && cnt < fw->size);

	/* EOF */
	if (cnt >= fw->size) {
		/* Setting mult to 0 will break caller loop */
		*txAttenMult = 0;
		release_firmware(fw);
		return 3;
	}

	line = skip_spaces(line);
	ret = sscanf(line, "%hu,%hhu,%hu", attenIndex, txAttenHp, txAttenMult);
	release_firmware(fw);

	return ret < 0 ? ADI_COMMON_ERR_API_FAIL : ret;
}

/* Initialization interface to open, init, close drivers and pointers to resources */
int32_t (*adi_adrv9001_hal_open)(void *devHalCfg) = linux_hw_open;
int32_t (*adi_adrv9001_hal_close)(void *devHalCfg) = linux_hw_close;
int32_t (*adi_adrv9001_hal_resetbPin_set)(void *devHalCfg, uint8_t pinLevel) = linux_hw_reset;

/* SPI Interface */
int32_t (*adi_hal_SpiWrite)(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes) = linux_spi_write;
int32_t (*adi_hal_SpiRead)(void *devHalCfg, const uint8_t txData[], uint8_t rxData[], uint32_t numRxBytes) = linux_spi_read;

/* Logging interface */
int32_t (*adi_common_hal_LogWrite)(void *devHalCfg, uint32_t logLevel, const char *comment, va_list argp) = linux_log_write;

/* Timer interface */
int32_t (*adi_common_hal_Wait_us)(void *devHalCfg, uint32_t time_us) = linux_wait_us;

/* Mcs interface */
int32_t (*adi_hal_Mcs_Pulse)(void *devHalCfg, uint8_t numberOfPulses) = linux_mcs_pulse;

/* ssi */
int32_t(*adi_hal_ssi_Reset)(void *devHalCfg) = linux_ssi_reset;

/* File IO abstraction */
int32_t (*adi_hal_ArmImagePageGet)(void *devHalCfg, const char *ImagePath, uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff) = linux_image_page_get;
int32_t (*adi_hal_StreamImagePageGet)(void *devHalCfg, const char *ImagePath, uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff) = linux_image_page_get;
int32_t (*adi_hal_RxGainTableEntryGet)(void *devHalCfg, const char *rxGainTablePath, uint16_t lineCount, uint8_t *gainIndex, uint8_t *rxFeGain,uint8_t *tiaControl, uint8_t *adcControl,
				      uint8_t *extControl, uint16_t *phaseOffset, int16_t *digGain) = linux_rx_gain_table_entry_get;
int32_t (*adi_hal_TxAttenTableEntryGet)(void *devHalCfg, const char *txAttenTablePath, uint16_t lineCount, uint16_t *attenIndex, uint8_t *txAttenHp,
				       uint16_t *txAttenMult) = linux_tx_atten_table_entry_get;

