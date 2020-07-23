/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifdef __KERNEL__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/firmware.h>

#include "adi_platform.h"

/*
 * \file
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information.
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_common_error.h"



/**
 * \brief Opens a logFile. If the file is already open it will be closed and reopened.
 *
 * This function opens the file for writing and saves the resulting file
 * descriptor to the devHalCfg structure.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param filename The user provided name of the file to open.
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 * \retval ADI_HAL_LOGGING_FAIL If the function failed to open or write to the specified filename
 */
int32_t linux_LogFileOpen(void *devHalCfg, const char *filename)
{
	return ADI_HAL_OK;
}

/**
 * \brief Flushes the logFile buffer to the currently open log file.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 */
int32_t linux_LogFileFlush(void *devHalCfg)
{
	return ADI_HAL_OK;
}

/**
 * \brief Gracefully closes the log file(s).
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 * \retval ADI_HAL_LOGGING_FAIL Error while flushing or closing the log file.
 */
int32_t linux_LogFileClose(void *devHalCfg)
{
	return ADI_HAL_OK;
}

/**
 * \brief Sets the log level, allowing the end user to select the granularity of
 *        what events get logged.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logLevel A mask of valid log levels to allow to be written to the log file.
 *
 * \retval ADI_COMMON_ACT_ERR_CHECK_PARAM    Recovery action for bad parameter check
 * \retval ADI_COMMON_ACT_NO_ACTION          Function completed successfully, no action required
 */
int32_t linux_LogLevelSet(void *devHalCfg, int32_t logLevel)
{
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL)
	{
		return ADI_COMMON_ACT_ERR_CHECK_PARAM;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	halCfg->logCfg.logLevel = (logLevel & (int32_t)ADI_HAL_LOG_ALL);

	return ADI_COMMON_ACT_NO_ACTION;
}

/**
 * \brief Gets the currently set log level: the mask of different types of log
 *         events that are currently enabled to be logged.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logLevel Returns the current log level mask.
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 */
int32_t linux_LogLevelGet(void *devHalCfg, int32_t *logLevel)
{
	int32_t halError = (int32_t)ADI_HAL_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL)
	{
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	*logLevel = halCfg->logCfg.logLevel;

	return halError;
}


/**
 * \brief Writes a message to the currently open logFile specified in the
 *        adi_hal_LogCfg_t of the devHalCfg structure passed
 *
 * Uses the vfprintf functionality to allow the user to supply the format and
 * the number of aguments that will be logged.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logLevel the log level to be written into
 * \param comment the string to include in the line added to the log.
 * \param argp variable argument list to be printed
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 * \retval ADI_HAL_LOGGING_FAIL If the function failed to flush to write
 */
int32_t linux_LogWrite(void *devHalCfg, int32_t logLevel, const char *comment, va_list argp)
{
	int32_t halError = (int32_t)ADI_HAL_OK;
	int32_t result = 0;
	adi_hal_Cfg_t *halCfg = NULL;
	char logMessage[ADI_HAL_MAX_LOG_LINE] = { 0 };
	const char *logLevelChar = NULL;
	logMessage[0] = 0;

	if (devHalCfg == NULL)
	{
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->logCfg.logLevel == (int32_t)ADI_HAL_LOG_NONE)
	{
		/* If logging disabled, exit gracefully */
		halError = (int32_t)ADI_HAL_OK;
		return halError;
	}

	if(logLevel > (int32_t)ADI_HAL_LOG_ALL)
	{
		halError = (int32_t)ADI_HAL_LOGGGING_LEVEL_FAIL;
		return halError;
	}

	/* Print Log type */
	if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_MSG) && (logLevel == (int32_t)ADI_HAL_LOG_MSG))
	{
		logLevelChar = "MESSAGE:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_WARN) && (logLevel == (int32_t)ADI_HAL_LOG_WARN))
	{
		logLevelChar = "WARNING:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_ERR) && (logLevel == (int32_t)ADI_HAL_LOG_ERR))
	{
		logLevelChar = "ERROR:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_API) && (logLevel == (int32_t)ADI_HAL_LOG_API))
	{
		logLevelChar = "API_LOG:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_BF) && (logLevel == (int32_t)ADI_HAL_LOG_BF))
	{
		logLevelChar = "BF_LOG:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_HAL) && (logLevel == (int32_t)ADI_HAL_LOG_HAL))
	{
		logLevelChar = "ADI_HAL_LOG:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_SPI) && (logLevel == (int32_t)ADI_HAL_LOG_SPI))
	{
		logLevelChar = "SPI_LOG:";
	}
	else if ((halCfg->logCfg.logLevel & ADI_HAL_LOG_API_PRIV) && (logLevel == (int32_t)ADI_HAL_LOG_API_PRIV))
	{
		logLevelChar = "API_PRIV_LOG:";
	}
	else
	{
		/* Nothing to log - exit cleanly */
		return (int32_t)ADI_HAL_OK;
	}

	result = snprintf(logMessage, ADI_HAL_MAX_LOG_LINE, "%s", logLevelChar);
	if (result < 0)
	{
		halError = (int32_t)ADI_HAL_LOGGING_FAIL;
		return halError;
	}

	result = vsnprintf(logMessage + strlen(logMessage), ADI_HAL_MAX_LOG_LINE, comment, argp) ;
	if (result < 0)
	{
		halError = (int32_t)ADI_HAL_LOGGING_FAIL;
		return halError;
	}

	result = printk("%s\n", logMessage);
	if (result < 0)
	{
		halError = (int32_t)ADI_HAL_LOGGING_FAIL;
		return halError;
	}



	return halError;
}

/**
 * \brief Opens/allocates any necessary resources to communicate via SPI to a
 *         particular device specified in the devHalCfg structure.
 *
 * This function should perform any necessary steps to open the SPI master resource
 * on the BBIC to enable SPI communications to a particular SPI device.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 * \retval ADI_HAL_SPI_FAIL the device driver was not opened successfully
 */
int32_t linux_SpiOpen(void *devHalCfg)
{
	return ADI_HAL_OK;
}

/**
 * \brief Closes any resources open/allocated for a specific SPI device
 *
 * Any information needed to close the particular SPI device should be passed in
 * the devHalCfg structure.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 * \retval ADI_HAL_SPI_FAIL the device driver was not closed successfully
 */
int32_t linux_SpiClose(void *devHalCfg)
{
	return ADI_HAL_OK;
}

/**
 * \brief Initializes the SPI device driver mode, bits per word, and speed
 *
 * Any settings needed should be passed in the devHalCfg structure
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 * \retval ADI_HAL_SPI_FAIL the SPI initialization failed
 */
int32_t linux_SpiInit(void *devHalCfg)
{
	return ADI_HAL_OK;
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
 * \retval ADI_HAL_OK function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 * \retval ADI_HAL_SPI_FAIL the data was not written successfully
 */
int32_t linux_SpiWrite(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes)
{
	static const uint32_t MAX_SIZE = 4096;
	uint32_t toWrite = 0;
	int32_t remaining = numTxBytes;
	int32_t halError = (int32_t)ADI_HAL_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL)
	{
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->spiCfg.spiActionDisable == 0)
	{
		int32_t result = 0;
		do
		{
			toWrite = (remaining > MAX_SIZE) ? MAX_SIZE : remaining;
			result = spi_write(halCfg->spi, &txData[numTxBytes - remaining], toWrite);
			if (result < 0)
				return ADI_HAL_SPI_FAIL;

			remaining -= toWrite;
		} while (remaining > 0);
	}

	return halError;
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
 * \retval ADI_HAL_OK function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 * \retval ADI_HAL_SPI_FAIL the data was not read successfully
 */
int32_t linux_SpiRead(void *devHalCfg, const uint8_t txData[], uint8_t rxData[],
			  uint32_t numTxRxBytes)
{
	static const uint32_t MAX_SIZE = 4096;
	int32_t remaining = numTxRxBytes;
	int32_t halError = (int32_t)ADI_HAL_OK;
	int32_t result = 0;
	adi_hal_Cfg_t *halCfg = NULL;

	if (!devHalCfg)
	{
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if(halCfg->spiCfg.spiActionDisable == 0)
	{
		do
		{
			struct spi_transfer	t = {
				.tx_buf		= &txData[numTxRxBytes - remaining],
				.rx_buf		= &rxData[numTxRxBytes - remaining],
				.len		= (remaining > MAX_SIZE) ? MAX_SIZE : remaining,
			};

			result = spi_sync_transfer(halCfg->spi, &t, 1);
			if (result < 0)
				return ADI_HAL_SPI_FAIL;

			remaining -= t.len;
		} while (remaining > 0);
	}

	return halError;
}

/**
 * \brief Function to open/allocate any necessary resources for the timer wait
 *        functions below.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully
 */
int32_t linux_TimerOpen(void *devHalCfg)
{
	/* ADI ZC706 platform does not require any timer open /close */
	return (int32_t)ADI_HAL_OK;
}

/**
 * \brief Function to close any necessary resources for the timer wait
 *        functions below.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully
 */
int32_t linux_TimerClose(void *devHalCfg)
{
	/* ADI ZC706 platform does not require any timer open /close */
	return (int32_t)ADI_HAL_OK;
}

/**
 * \brief Function to initialize any necessary resources for the timer wait
 *        functions below.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully
 */
int32_t linux_TimerInit(void *devHalCfg)
{
	/* ADI ZC706 platform does not require any timer init */
	return (int32_t)ADI_HAL_OK;
}

/**
 * \brief Provides a blocking delay of the current thread
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param time_us the time to delay in mico seconds
 *
 * \retval ADI_HAL_OK Function completed successfully
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 */
int32_t linux_TimerWait_us(void *devHalCfg, uint32_t time_us)
{
	int32_t halError = (int32_t)ADI_HAL_OK;

	usleep_range(time_us, time_us + 10);

	return halError;
}

/**
 * \brief Provides a blocking delay of the current thread
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param time_ms the Time to delay in milli seconds
 *
 * \retval ADI_HAL_OK Function completed successfully
 * \retval ADI_HAL_NULL_PTR the function has been called with a null pointer
 */
int32_t linux_TimerWait_ms(void *devHalCfg, uint32_t time_ms)
{
	int32_t halError = (int32_t)ADI_HAL_OK;

	msleep(time_ms);

	return halError;
}

/**
 * \brief Opens all neccessary files and device drivers for a specific device
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 * \retval errors returned by other function calls.
 */
int32_t linux_HwOpen(void *devHalCfg)
{
	return ADI_HAL_OK;
}

/**
 * \brief Gracefully shuts down the the hardware closing any open resources
 *        such as log files, I2C, SPI, GPIO drivers, timer resources, etc.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 */
int32_t linux_HwClose(void *devHalCfg)
{
	return ADI_HAL_OK;
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
 * \retval ADI_HAL_OK Function completed successfully, no action required
 * \retval ADI_HAL_NULL_PTR The function has been called with a null pointer
 */
int32_t linux_HwReset(void *devHalCfg, uint8_t pinLevel)
{
	adi_hal_Cfg_t *halCfg;

	if (!devHalCfg)
		return ADI_HAL_NULL_PTR;

	halCfg = (adi_hal_Cfg_t *)devHalCfg;
	/*
	 * The API just passes @pinLevel with the desired level. However,
	 * the pin is active low, so the logic must be inverted before calling
	 * the gpio API's. Hence if we receive 0 from the API, we want to pass
	 * 1 to the GPIO API since we want our pin to be active!
	 */
	gpiod_set_value_cansleep(halCfg->reset_gpio, !pinLevel);

	return ADI_HAL_OK;
}

/* Not supported yet */
int32_t linux_Mcs_Pulse(void* devHalCfg, uint8_t numberOfPulses)
{
    return ADI_HAL_FUNCTION_NOT_IMP;
}

/* Not supported yet */
int32_t linux_ssi_Reset(void* devHalCfg)
{
    return ADI_HAL_FUNCTION_NOT_IMP;
}

int32_t linux_ImagePageGet(void *devHalCfg, const char *ImagePath,
			   uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff)
{
    adi_hal_Cfg_t *hal = (adi_hal_Cfg_t *)devHalCfg;
    struct spi_device *spi = hal->spi;
    const struct firmware *fw;
    int ret;

    ret = request_firmware(&fw, ImagePath, &spi->dev);
    if (ret) {
	dev_err(&spi->dev,
		"request_firmware(%s) failed with %d\n", ImagePath, ret);

	return ret;
    }

    if ((pageIndex * pageSize) > fw->size)
	return -EINVAL;

    memcpy(rdBuff, &fw->data[pageIndex * pageSize], pageSize);

    release_firmware(fw);

    return 0;
}

int32_t linux_RxGainTableEntryGet(void *devHalCfg, const char *rxGainTablePath, uint16_t lineCount, uint8_t *gainIndex, uint8_t *rxFeGain,
				  uint8_t *tiaControl, uint8_t *adcControl, uint8_t *extControl, uint16_t *phaseOffset, int16_t *digGain)
{
    adi_hal_Cfg_t *hal = (adi_hal_Cfg_t *)devHalCfg;
    struct spi_device *spi = hal->spi;
    const struct firmware *fw;
    char *line;
    int i = 0, ret, cnt = 0;

    ret = request_firmware(&fw, rxGainTablePath, &spi->dev);
    if (ret) {
	dev_err(&spi->dev,
		"request_firmware(%s) failed with %d\n", rxGainTablePath, ret);
	return ret;
    }

    do {
	line = strnchr(fw->data + cnt, fw->size - cnt, '\n');
	if (!line)
		return -EINVAL;
	line++;
	cnt = line - (char *)fw->data;
    } while (i++ != lineCount && cnt < fw->size);

    /* EOF */
    if (cnt >= fw->size) {
	/* Setting index to 0 will break caller loop */
	*gainIndex = 0;
	return 7;
    }

    line = skip_spaces(line);

    ret = sscanf(line, "%hhu,%hhu,%hhu,%hhu,%hhu,%hu,%hd",
		 gainIndex,
		 rxFeGain,
		 tiaControl,
		 adcControl,
		 extControl,
		 phaseOffset,
		 digGain);

    release_firmware(fw);

    return ret;
}

int32_t linux_TxAttenTableEntryGet(void *devHalCfg, const char *txAttenTablePath, uint16_t lineCount, uint16_t *attenIndex,
				   uint8_t *txAttenHp, uint16_t *txAttenMult)
{
    adi_hal_Cfg_t *hal = (adi_hal_Cfg_t *)devHalCfg;
    struct spi_device *spi = hal->spi;
    const struct firmware *fw;
    char *line;
    int i = 0, ret, cnt = 0;

    ret = request_firmware(&fw, txAttenTablePath, &spi->dev);
    if (ret) {
	dev_err(&spi->dev,
		"request_firmware(%s) failed with %d\n", txAttenTablePath, ret);
	return ret;
    }

    do {
	line = strnchr(fw->data + cnt, fw->size - cnt, '\n');
	if (!line)
		return -EINVAL;
	line++;
	cnt = line - (char *)fw->data;
    } while (i++ != lineCount && cnt < fw->size);

    /* EOF */
    if (cnt >= fw->size) {
	/* Setting mult to 0 will break caller loop */
	*txAttenMult = 0;
	return 3;
    }

    line = skip_spaces(line);

    ret = sscanf(line, "%hu,%hhu,%hu", attenIndex, txAttenHp, txAttenMult);

    release_firmware(fw);

    return ret;
}

/*
 * Function pointer assignemt for default configuration
 */

/* Initialization interface to open, init, close drivers and pointers to resources */
int32_t (*adi_hal_HwOpen)(void *devHalCfg) = linux_HwOpen;
int32_t (*adi_hal_HwClose)(void *devHalCfg) = linux_HwClose;
int32_t (*adi_hal_HwReset)(void *devHalCfg, uint8_t pinLevel) = linux_HwReset;

/* SPI Interface */
int32_t (*adi_hal_SpiWrite)(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes) = linux_SpiWrite;
int32_t (*adi_hal_SpiRead)(void *devHalCfg, const uint8_t txData[], uint8_t rxData[], uint32_t numRxBytes) = linux_SpiRead;

/* Logging interface */
int32_t (*adi_hal_LogFileOpen)(void *devHalCfg, const char *filename) = linux_LogFileOpen;
int32_t(*adi_hal_LogLevelSet)(void *devHalCfg, int32_t logLevel) = linux_LogLevelSet;
int32_t(*adi_hal_LogLevelGet)(void *devHalCfg, int32_t *logLevel) = linux_LogLevelGet;
int32_t(*adi_hal_LogWrite)(void *devHalCfg, int32_t logLevel, const char *comment, va_list args) = linux_LogWrite;
int32_t(*adi_hal_LogFileClose)(void *devHalCfg) = linux_LogFileClose;

/* Timer interface */
int32_t (*adi_hal_Wait_ms)(void *devHalCfg, uint32_t time_ms) = linux_TimerWait_ms;
int32_t (*adi_hal_Wait_us)(void *devHalCfg, uint32_t time_us) = linux_TimerWait_us;

/* Mcs interface */
int32_t(*adi_hal_Mcs_Pulse)(void *devHalCfg, uint8_t numberOfPulses) = linux_Mcs_Pulse;

/* ssi */
int32_t(*adi_hal_ssi_Reset)(void *devHalCfg) = linux_ssi_Reset;

/* File IO abstraction */
int32_t(*adi_hal_ArmImagePageGet)(void *devHalCfg, const char *ImagePath, uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff) = linux_ImagePageGet;
int32_t(*adi_hal_StreamImagePageGet)(void *devHalCfg, const char *ImagePath, uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff) = linux_ImagePageGet;
int32_t(*adi_hal_RxGainTableEntryGet)(void *devHalCfg, const char *rxGainTablePath, uint16_t lineCount, uint8_t *gainIndex, uint8_t *rxFeGain,
				       uint8_t *tiaControl, uint8_t *adcControl, uint8_t *extControl, uint16_t *phaseOffset, int16_t *digGain) = linux_RxGainTableEntryGet;
int32_t(*adi_hal_TxAttenTableEntryGet)(void *devHalCfg, const char *txAttenTablePath, uint16_t lineCount, uint16_t *attenIndex, uint8_t *txAttenHp,
				       uint16_t *txAttenMult) = linux_TxAttenTableEntryGet;

/**
 * \brief Platform setup
 *
 * \param devHalInfo void pointer to be casted to the HAL config structure
 * \param platform Platform to be assigning the function pointers
 *
 * \return
 */
int32_t adi_hal_PlatformSetup(void *devHalInfo, adi_hal_Platforms_e platform)
{
    adi_hal_Err_e error = ADI_HAL_OK;

    adi_hal_HwOpen = linux_HwOpen;
    adi_hal_HwClose = linux_HwClose;
    adi_hal_HwReset = linux_HwReset;

    adi_hal_SpiWrite = linux_SpiWrite;
    adi_hal_SpiRead = linux_SpiRead;

    adi_hal_LogFileOpen = linux_LogFileOpen;
    adi_hal_LogLevelSet = linux_LogLevelSet;
    adi_hal_LogLevelGet = linux_LogLevelGet;
    adi_hal_LogWrite = linux_LogWrite;
    adi_hal_LogFileClose = linux_LogFileClose;

    adi_hal_Wait_us = linux_TimerWait_us;
    adi_hal_Wait_ms = linux_TimerWait_ms;

    adi_hal_Mcs_Pulse = linux_Mcs_Pulse;

    adi_hal_ssi_Reset = linux_ssi_Reset;

    adi_hal_ArmImagePageGet = linux_ImagePageGet;
    adi_hal_StreamImagePageGet = linux_ImagePageGet;
    adi_hal_RxGainTableEntryGet = linux_RxGainTableEntryGet;
    adi_hal_TxAttenTableEntryGet = linux_TxAttenTableEntryGet;

    return error;
}

#endif
