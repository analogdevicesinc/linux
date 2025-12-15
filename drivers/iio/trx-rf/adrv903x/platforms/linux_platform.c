// SPDX-License-Identifier: GPL-2.0
/**
* Copyright 2015 - 2025 Analog Devices Inc.
* Released under the ADRV903X API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/slab.h>
#include <linux/err.h>

#include "adi_platform.h"
#include "adi_platform_impl_types.h"

#include "adi_common_error.h"

#define ADI_HAL_OK ADI_HAL_ERR_OK
#define ADI_HAL_NULL_PTR ADI_HAL_ERR_NULL_PTR

typedef struct
{
    void* value;
} thread_to_value_t;

/* ADI Error Reporting Dependency on HAL_TLS_ERR being set to NULL if not configured */
static thread_to_value_t store = { NULL };

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
adi_hal_Err_e linux_adrv903x_LogFileOpen(void *devHalCfg, const char *filename)
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
adi_hal_Err_e linux_adrv903x_LogFileFlush(void *devHalCfg)
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
adi_hal_Err_e linux_adrv903x_LogFileClose(void *devHalCfg)
{
	return ADI_HAL_OK;
}

/**
 * \brief Get logging status. Functions has no effect since logging is
 *        disabled on Linux.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logStatus Logging status.
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_LogStatusGet(void* devHalCfg, adi_hal_LogStatusGet_t* const logStatus)
{
	return ADI_HAL_OK;
}

/**
 * \brief Set console output. Functions has no effect since logging is
 *        disabled on Linux.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logConsoleFlag Console logging enable flag.
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_LogConsoleSet(void* devHalCfg, const adi_hal_LogConsole_e logConsoleFlag)
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
 * \retval ADI_ADRV903X_ERR_ACT_CHECK_PARAM   Recovery action for bad parameter check
 * \retval ADI_ADRV903X_ERR_ACT_NONE          Function completed successfully, no action required
 */
adi_hal_Err_e linux_adrv903x_LogLevelSet(void *devHalCfg, const uint32_t logMask)
{
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		return ADI_COMMON_ERR_ACT_CHECK_PARAM;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	halCfg->logCfg.logMask = (logMask & (int32_t)ADI_HAL_LOG_ALL);

	return ADI_COMMON_ERR_ACT_NONE;
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
adi_hal_Err_e linux_adrv903x_LogLevelGet(void *devHalCfg, uint32_t* const logMask)
{
	int32_t halError = (int32_t)ADI_HAL_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	*logMask = halCfg->logCfg.logMask;

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

adi_hal_Err_e linux_adrv903x_LogWrite(void *devHalCfg, const adrv903x_LogLevel_e logLevel,
			const uint8_t indent, const char* const comment,
			va_list argp)
{
	int32_t halError = (int32_t)ADI_HAL_OK;
	int32_t result = 0;
	adi_hal_Cfg_t *halCfg = NULL;
	static char logMessage[ADI_HAL_MAX_LOG_LINE] = { 0 };

	const char *logLevelChar = NULL;
	logMessage[0] = 0;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->logCfg.logMask == (int32_t)ADI_HAL_LOG_NONE) {
		/* If logging disabled, exit gracefully */
		halError = (int32_t)ADI_HAL_OK;
		return halError;
	}

	if (logLevel > (int32_t)ADI_HAL_LOG_ALL) {
		halError = (int32_t)ADI_HAL_ERR_LOG;
		return halError;
	}

	/* Print Log type */
	if ((halCfg->logCfg.logMask & ADI_HAL_LOG_MSG) &&
	    (logLevel == (int32_t)ADI_HAL_LOG_MSG)) {
		logLevelChar = "MESSAGE:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_WARN) &&
		   (logLevel == (int32_t)ADI_HAL_LOG_WARN)) {
		logLevelChar = "WARNING:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_ERR) &&
		   (logLevel == (int32_t)ADI_HAL_LOG_ERR)) {
		logLevelChar = "ERROR:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_API) &&
		   (logLevel == (int32_t)ADI_HAL_LOG_API)) {
		logLevelChar = "API_LOG:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_HAL) &&
		   (logLevel == (int32_t)ADI_HAL_LOG_HAL)) {
		logLevelChar = "ADI_HAL_LOG:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_SPI) &&
		   (logLevel == (int32_t)ADI_HAL_LOG_SPI)) {
		logLevelChar = "SPI_LOG:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_API_PRIV) &&
		   (logLevel == (int32_t)ADI_HAL_LOG_API_PRIV)) {
		logLevelChar = "API_PRIV_LOG:";
	} else {
		/* Nothing to log - exit cleanly */
		return (int32_t)ADI_HAL_OK;
	}

	result = snprintf(logMessage, ADI_HAL_MAX_LOG_LINE, "%s", logLevelChar);
	if (result < 0) {
		halError = (int32_t)ADI_HAL_ERR_LOG;
		return halError;
	}

	result = vsnprintf(logMessage + strlen(logMessage),
			   ADI_HAL_MAX_LOG_LINE, comment, argp);
	if (result < 0) {
		halError = (int32_t)ADI_HAL_ERR_LOG;
		return halError;
	}

	switch (logLevel) {
	case ADI_HAL_LOG_NONE:
		break;
	case ADI_HAL_LOG_WARN:
		dev_warn(&halCfg->spi->dev, "%s", logMessage);
		break;
	case ADI_HAL_LOG_ERR:
		dev_err(&halCfg->spi->dev, "%s", logMessage);
		break;
	case ADI_HAL_LOG_SPI:
		dev_vdbg(&halCfg->spi->dev, "%s", logMessage);
		break;
	case ADI_HAL_LOG_API:
	case ADI_HAL_LOG_API_PRIV:
	case ADI_HAL_LOG_BF:
	case ADI_HAL_LOG_MSG:
	case ADI_HAL_LOG_HAL:
		dev_dbg(&halCfg->spi->dev, "%s", logMessage);
		break;
	case ADI_HAL_LOG_ALL:
		printk(logMessage);
		break;
	}

	printk("MESSAGE: %s\n", logMessage);

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
adi_hal_Err_e linux_adrv903x_SpiOpen(void *devHalCfg)
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
adi_hal_Err_e linux_adrv903x_SpiClose(void *devHalCfg)
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
int32_t linux_adrv903x_SpiInit(void *devHalCfg)
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
adi_hal_Err_e linux_adrv903x_SpiWrite(void *devHalCfg, const uint8_t txData[],
		       uint32_t numTxBytes)
{
	static const uint32_t MAX_SIZE = 4096;
	uint32_t toWrite = 0;
	int32_t remaining = numTxBytes;
	int32_t halError = (int32_t)ADI_HAL_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->spiCfg.interfaceEnabled != 0) {
		int32_t result = 0;
		do {
			toWrite = (remaining > MAX_SIZE) ? MAX_SIZE : remaining;
			result = spi_write(halCfg->spi,
					   &txData[numTxBytes - remaining],
					   toWrite);
			if (result < 0) {
				return ADI_HAL_ERR_SPI_WRITE;
			}
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
adi_hal_Err_e linux_adrv903x_SpiRead(void *devHalCfg, const uint8_t txData[], uint8_t rxData[],
		      uint32_t numTxRxBytes)
{
	static const uint32_t MAX_SIZE = 4096;
	int32_t remaining = numTxRxBytes;
	int32_t halError = (int32_t)ADI_HAL_OK;
	int32_t result = 0;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->spiCfg.interfaceEnabled != 0) {
		do {
			struct spi_transfer t = {
				.tx_buf = &txData[numTxRxBytes - remaining],
				.rx_buf = &rxData[numTxRxBytes - remaining],
				.len = (remaining > MAX_SIZE) ? MAX_SIZE :
								remaining,
			};

			result = spi_sync_transfer(halCfg->spi, &t, 1);
			if (result < 0) {
				halError = ADI_HAL_ERR_SPI_READ;
			}

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
int32_t linux_adrv903x_TimerOpen(void *devHalCfg)
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
int32_t linux_adrv903x_TimerClose(void *devHalCfg)
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
int32_t linux_adrv903x_TimerInit(void *devHalCfg)
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
adi_hal_Err_e linux_adrv903x_TimerWait_us(void *devHalCfg, const uint32_t time_us)
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
adi_hal_Err_e linux_adrv903x_TimerWait_ms(void *devHalCfg, const uint32_t time_ms)
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
adi_hal_Err_e linux_adrv903x_HwOpen(void *devHalCfg)
{
	return ADI_HAL_OK;
}
/**
 * \brief Mutex init function. No action, single-threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_mutex_init(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_OK;
}
/**
 * \brief Mutex destroy function. No action, single-threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_mutex_destroy(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_OK;
}
/**
 * \brief Mutex lock function. No action, single-threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_mutex_lock(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_OK;
}
/**
 * \brief Mutex unlock function. No action, single-threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_mutex_unlock(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_OK;
}
/**
 * \brief Tls get. This function will return the same value, single-threaded
 *
 * \param tlsType Key value associated with a thread
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
void* linux_tls_get(const adi_hal_TlsType_e tlsType)
{
	if (tlsType == HAL_TLS_END) {
		return NULL;
	}

	return store.value;
}
/**
 * \brief Tls set. This function will set the same value, single-threaded
 *
 * \param tlsType Key value associated with a thread
 * \param value Value to associate to a thread
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
adi_hal_Err_e linux_tls_set(const adi_hal_TlsType_e tlsType, void* const value)
{
	if (tlsType == HAL_TLS_END) {
		/* Special convenience case that a thread can remove all it TLS
		* items in one call by passing (HAL_TLS_END, NULL); */
		if (value != NULL) {
			return ADI_HAL_ERR_PARAM;
		}

		/* Passing NULL cannot fail by definition - no need to check rtn values */
		(void) linux_tls_set(HAL_TLS_ERR, NULL);
		(void) linux_tls_set(HAL_TLS_USR, NULL);

		return ADI_HAL_ERR_OK;
	}

	store.value = value;
	return ADI_HAL_ERR_OK;
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
adi_hal_Err_e linux_adrv903x_HwClose(void *devHalCfg)
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
adi_hal_Err_e linux_adrv903x_HwReset(void *devHalCfg, uint8_t pinLevel)
{
	adi_hal_Cfg_t *halCfg;

	if (devHalCfg == NULL) {
		return ADI_HAL_NULL_PTR;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	gpiod_set_value(halCfg->reset_gpio, pinLevel);

	return ADI_HAL_OK;
}

/*
 * Function pointer assignemt for default configuration
 */

/* Initialization interface to open, init, close drivers and pointers to resources */
adi_hal_Err_e (*adrv903x_HwOpen)(void *devHalCfg) = NULL;
adi_hal_Err_e (*adrv903x_HwClose)(void *devHalCfg) = NULL;
adi_hal_Err_e (*adrv903x_HwReset)(void *devHalCfg, uint8_t pinLevel) = NULL;
adi_hal_Err_e (*adrv903x_SpiInit)(void *devHalCfg) =
	NULL; /* TODO: remove?  called by HwOpen() */
void *(*adrv903x_DevHalCfgCreate)(uint32_t interfaceMask, uint8_t spiChipSelect,
				 const char *logFilename) = NULL;
adi_hal_Err_e (*adrv903x_DevHalCfgFree)(void *devHalCfg) = NULL;
adi_hal_Err_e (*adrv903x_HwVerify)(void *devHalCfg) = NULL;

/* SPI Interface */
adi_hal_Err_e (*adrv903x_hal_SpiWrite)(void *devHalCfg, const uint8_t txData[],
			    uint32_t numTxBytes) = NULL;

adi_hal_Err_e (*adrv903x_hal_SpiRead)(void *devHalCfg, const uint8_t txData[],
			   uint8_t rxData[], uint32_t numRxBytes) = NULL;

/* Custom SPI streaming interface*/
adi_hal_Err_e (*adrv903x_CustomSpiStreamWrite)(void *devHalCfg, const uint16_t address,
					const uint8_t txData[],
					uint32_t numTxBytes,
					uint8_t numBytesofAddress,
					uint8_t numBytesOfDataPerStream) = NULL;

adi_hal_Err_e (*adrv903x_CustomSpiStreamRead)(void *devHalCfg, const uint16_t address,
				       uint8_t rxData[], uint32_t numRxBytes,
				       uint8_t numBytesofAddress,
				       uint8_t numBytesOfDataPerStream) = NULL;

/* Logging interface */
adi_hal_Err_e (*adrv903x_LogFileOpen)(void *devHalCfg, const char *filename) = NULL;

adi_hal_Err_e (*adrv903x_LogLevelSet)(void *devHalCfg, const uint32_t logMask) = NULL;

adi_hal_Err_e (*adrv903x_LogLevelGet)(void *devHalCfg, uint32_t *const logMask) = NULL;

adi_hal_Err_e (*adrv903x_LogWrite)(void *devHalCfg, const adrv903x_LogLevel_e    logLevel,
			    const uint8_t indent, const char *comment, va_list args) = NULL;

adi_hal_Err_e (*adrv903x_LogFileClose)(void *devHalCfg) = NULL;

adi_hal_Err_e (*adi_adrv903x_LogStatusGet)(void* const devHalCfg,
				adi_hal_LogStatusGet_t* const logStatus) = NULL;

adi_hal_Err_e (*adi_adrv903x_LogConsoleSet)(void* const devHalCfg,
				const adi_hal_LogConsole_e logConsoleFlag) = NULL;

/* Timer interface */
adi_hal_Err_e (*adrv903x_Wait_ms)(void *devHalCfg, uint32_t time_ms) = NULL;

adi_hal_Err_e (*adrv903x_Wait_us)(void *devHalCfg, uint32_t time_us) = NULL;

/* Mutexes */
adi_hal_Err_e (*adi_adrv903x_MutexInit)(adi_hal_mutex_t* const mutex) = NULL;
adi_hal_Err_e (*adi_adrv903x_MutexDestroy)(adi_hal_mutex_t* const mutex) = NULL;
adi_hal_Err_e (*adi_adrv903x_MutexLock)(adi_hal_mutex_t* const mutex) = NULL;
adi_hal_Err_e (*adi_adrv903x_MutexUnlock)(adi_hal_mutex_t* const mutex) = NULL;

/* TLS */
void* (*adi_adrv903x_TlsGet)(const adi_hal_TlsType_e tlsType);
adi_hal_Err_e (*adi_adrv903x_TlsSet)(const adi_hal_TlsType_e tlsType, void* const value);

/**
 * \brief Platform setup
 *
 * \param platform Platform to be assigning the function pointers
 *
 * \return
 */
adi_hal_Err_e adrv903x_hal_PlatformSetup(adi_hal_Platforms_e platform)
{
	adi_hal_Err_e error = ADI_HAL_OK;

	switch (platform)
	{
	case ADI_LINUX:
		adrv903x_HwOpen = linux_adrv903x_HwOpen;
		adrv903x_HwClose = linux_adrv903x_HwClose;
		adrv903x_HwReset = linux_adrv903x_HwReset;

		adrv903x_hal_SpiWrite = linux_adrv903x_SpiWrite;
		adrv903x_hal_SpiRead = linux_adrv903x_SpiRead;

		adrv903x_LogFileOpen = linux_adrv903x_LogFileOpen;
		adrv903x_LogLevelSet = linux_adrv903x_LogLevelSet;
		adrv903x_LogLevelGet = linux_adrv903x_LogLevelGet;
		adrv903x_LogWrite = linux_adrv903x_LogWrite;
		adrv903x_LogFileClose = linux_adrv903x_LogFileClose;
		adi_adrv903x_LogStatusGet = linux_LogStatusGet;
		adi_adrv903x_LogConsoleSet = linux_LogConsoleSet;

		adrv903x_Wait_us = linux_adrv903x_TimerWait_us;
		adrv903x_Wait_ms = linux_adrv903x_TimerWait_ms;

		adrv903x_SpiInit = linux_adrv903x_HwOpen;
		adrv903x_HwVerify = linux_adrv903x_HwOpen;

		adi_adrv903x_MutexInit = linux_mutex_init;
		adi_adrv903x_MutexDestroy = linux_mutex_destroy;
		adi_adrv903x_MutexLock = linux_mutex_lock;
		adi_adrv903x_MutexUnlock = linux_mutex_unlock;

		adi_adrv903x_TlsGet = linux_tls_get;
		adi_adrv903x_TlsSet = linux_tls_set;
		break;
	
	default:
		error = ADI_HAL_ERR_NOT_IMPLEMENTED;
		break;
	}

	return error;
}

/*
 * FileIO abstraction using Linux kernel firmware subsystem
 *
 */

int __adrv903x_fseek (FILE * stream, long int offset, int origin)
{
	char *ptr;

	switch (origin) {
	case SEEK_END:
		ptr = stream->end + offset;
		break;
	case SEEK_SET:
		ptr = stream->start + offset;
		break;
	case SEEK_CUR:
		ptr = stream->ptr + offset;
		break;
	default:
		return -EINVAL;
	}

	if (ptr > stream->end || ptr < stream->start)
		return -1;

	stream->ptr = ptr;

	return 0;
}

long int __adrv903x_ftell (FILE *stream)
{
	return stream->ptr - stream->start;
}

FILE* __adrv903x_fopen (adi_hal_Cfg_t *hal, const char * filename, const char *mode)
{
	int ret;

	FILE *stream = devm_kzalloc(&hal->spi->dev, sizeof(*stream), GFP_KERNEL);
	stream->hal = hal;

	ret = request_firmware(&stream->fw, filename, &hal->spi->dev);
	if (ret < 0)
		return NULL;

	stream->start = stream->ptr = (char *)stream->fw->data;
	stream->end = stream->start + stream->fw->size;

	return stream;
}

int __adrv903x_fclose (FILE *stream)
{
	if (stream == NULL)
		return -ENODEV;

	if (stream->fw)
		release_firmware(stream->fw);

	devm_kfree(&stream->hal->spi->dev, stream);

	return 0;
}

char * __adrv903x_fgets(char *dst, int num, FILE *stream)
{
	char *p;
	int c;

	for (p = dst, num--; num > 0; num--) {
		if (stream->ptr + 1 > stream->end) {
			return NULL;
		}

		c = *stream->ptr++;
		*p++ = c;

		if (c == '\n')
			break;
	}
	*p = 0;
	if (p == dst)
		return NULL;

	return p;
}

size_t __adrv903x_fread(void *ptr, size_t size, size_t count, FILE *stream)
{
	int total = size * count;

	if ((stream->ptr + total) > stream->end)
		total = stream->end - stream->ptr;

	memcpy(ptr, stream->ptr, total);
	stream->ptr += total;

	return total;
}

size_t __adrv903x_fwrite (const void * ptr, size_t size, size_t count, FILE *stream)
{
	return 0;
}

int __adrv903x_fflush (FILE *stream)
{
	return 0;
}

int ferror(FILE *stream)
{
	return 0;
}

int fprintf(FILE *stream, const char *fmt, ...)
{
	return 0;
}

/*
 * Memory allocation abstraction using Linux kernel firmware subsystem
 */

void *__adrv903x_calloc(size_t size, size_t nmemb)
{
	size_t total_size;

	total_size = size * nmemb;
	return kzalloc(total_size, GFP_KERNEL);
}

void *__adrv903x_malloc(size_t size)
{
	return kzalloc(size, GFP_KERNEL);
}

/*
 * log2 using Linux kernel firmware subsystem
 */
unsigned int __adrv903x_log2(unsigned int v)
{
	return __ilog2_u32(v);
}

/*
 * Iterative algorithm based on the fact that
 * multiplication (division) in the linear domain is equivalent to
 * addition (subtraction) in the log domain.
 */

#define COMPUTE(n, d) if (neg) {a *= d; a /= n;} else {a *= n; a /= d;};

long int_20db_to_mag(long a, int mdB)
{
	unsigned neg = 0;

	if (mdB < 0) {
		neg = 1;
		mdB *= -1;
	}

	while (mdB > 0) {
		if (mdB >= 20000) {
			mdB -= 20000;
			COMPUTE(10, 1); /* 10^(20/20) */
			continue;
		}
		if (mdB >= 6000) {
			mdB -= 6000;
			COMPUTE(199526, 100000); /* 10^(6/20) */
			continue;
		}
		if (mdB >= 1000) {
			mdB -= 1000;
			COMPUTE(112202, 100000); /* 10^(1/20) */
			continue;
		}
		if (mdB >= 100) {
			mdB -= 100;
			COMPUTE(101158, 100000); /* 10^(0.1/20) */
			continue;
		}
		if (mdB >= 10) {
			mdB -= 10;
			COMPUTE(100115, 100000); /* 10^(0.01/20) */
			continue;
		}
		if (mdB >= 1) {
			mdB -= 1;
			COMPUTE(100012, 100000); /* 10^(0.001/20) */
			continue;
		}
	}

	return a;
}

/*
 * Timkeeping abstraction using Linux kernel firmware subsystem
 *
 */

void time(ktime_t *second)
{
	*second = ktime_get_seconds();
}

struct tm* __gmtime(adi_hal_Cfg_t *hal, const ktime_t *timer)
{
	struct tm *tmPtr = devm_kzalloc(&hal->spi->dev, sizeof(*tmPtr), GFP_KERNEL);

	tmPtr->tm_sec = *timer % 60;
	tmPtr->tm_min = (*timer / 60) % 60;
	tmPtr->tm_hour = (*timer / 3600) % 24;
	tmPtr->tm_mday = (*timer / 86400) + 1; // Add 1 to start from day 1
	tmPtr->tm_mon = 0; // Month is always January
	tmPtr->tm_year = 70; // Year 1970 (UNIX epoch)

	return tmPtr;
}
