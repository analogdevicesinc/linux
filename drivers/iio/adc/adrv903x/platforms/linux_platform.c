// SPDX-License-Identifier: GPL-2.0
/**
* Copyright 2015 - 2020 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/firmware.h>
#include "linux_platform.h"

typedef struct
{
    void* value;
} thread_to_value_t;

/* ADI Error Reporting Dependency on HAL_TLS_ERR being set to NULL if not configured */
static thread_to_value_t store = { NULL };

#include "adi_common_error.h"

adi_hal_Err_e linux_adrv903x_HwOpen(void *devHalCfg)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_HwReset(void *devHalCfg, uint8_t pinLevel)
{
	adi_hal_Cfg_t *halCfg;

	if (devHalCfg == NULL) {
		return ADI_HAL_ERR_NULL_PTR;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	gpiod_set_value(halCfg->reset_gpio, pinLevel);

	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_HwClose(void *devHalCfg)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_SpiWrite(void* const devHalCfg,
					      const uint8_t txData[],
					      const uint32_t numTxBytes)
{
	static const uint32_t MAX_SIZE = 4096;
	uint32_t toWrite = 0;
	int32_t remaining = numTxBytes;
	int32_t halError = (int32_t)ADI_HAL_ERR_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_ERR_NULL_PTR;
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

ADI_API adi_hal_Err_e linux_adrv903x_SpiRead(void* const devHalCfg,
					     const uint8_t txData[],
					     uint8_t rxData[],
					     const uint32_t numTxRxBytes)
{
	static const uint32_t MAX_SIZE = 4096;
	int32_t remaining = numTxRxBytes;
	int32_t halError = (int32_t)ADI_HAL_ERR_OK;
	int32_t result = 0;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_ERR_NULL_PTR;
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

ADI_API adi_hal_Err_e linux_adrv903x_LogFileOpen(void *devHalCfg, const char *filename)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_LogFileClose(void *devHalCfg)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_LogLevelSet(void* const devHalCfg,
						 const uint32_t logMask)
{
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		return ADI_HAL_ERR_PARAM;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	halCfg->logCfg.logMask = (logMask & (int32_t)ADI_HAL_LOG_ALL);

	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_LogLevelGet(void* const devHalCfg,
						 uint32_t* const logMask)
{
	int32_t halError = (int32_t)ADI_HAL_ERR_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_ERR_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	*logMask = halCfg->logCfg.logMask;

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv903x_LogWrite(void* const devHalCfg,
                                    	      const adi_hal_LogLevel_e logLevel,
                                    	      const uint8_t indent,
                                    	      const char* const comment,
                                    	      va_list argp)
{
	int32_t halError = (int32_t)ADI_HAL_ERR_OK;
	int32_t result = 0;
	adi_hal_Cfg_t *halCfg = NULL;
	static char logMessage[ADI_HAL_MAX_LOG_LINE] = { 0 };

	const char *logLevelChar = NULL;
	logMessage[0] = 0;

	if (devHalCfg == NULL) {
		halError = (int32_t)ADI_HAL_ERR_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->logCfg.logMask == (int32_t)ADI_HAL_LOG_NONE) {
		/* If logging disabled, exit gracefully */
		halError = (int32_t)ADI_HAL_ERR_OK;
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
		return (int32_t)ADI_HAL_ERR_OK;
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

ADI_API adi_hal_Err_e linux_adrv903x_TimerWait_us(void *devHalCfg,
						  uint32_t time_us)
{
	int32_t halError = (int32_t)ADI_HAL_ERR_OK;

	usleep_range(time_us, time_us + 10);

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv903x_TimerWait_ms(void *devHalCfg,
						  uint32_t time_ms)
{
	int32_t halError = (int32_t)ADI_HAL_ERR_OK;

	msleep(time_ms);

	return halError;
}

ADI_API void* linux_adrv903x_TlsGet(const adi_hal_TlsType_e tlsType)
{
	if (tlsType == HAL_TLS_END) {
		return NULL;
	}

	return store.value;
}

ADI_API adi_hal_Err_e linux_adrv903x_TlsSet(const adi_hal_TlsType_e tlsType,
				     void* const value)
{
	if (tlsType == HAL_TLS_END) {
		/* Special convenience case that a thread can remove all it TLS
		* items in one call by passing (HAL_TLS_END, NULL); */
		if (value != NULL) {
			return ADI_HAL_ERR_PARAM;
		}

		/* Passing NULL cannot fail by definition - no need to check rtn values */
		(void) linux_adrv903x_TlsSet(HAL_TLS_ERR, NULL);
		(void) linux_adrv903x_TlsSet(HAL_TLS_USR, NULL);

		return ADI_HAL_ERR_OK;
	}

	store.value = value;
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_MutexInit(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv903x_MutexDestroy(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

//ToDo
ADI_API adi_hal_Err_e linux_adrv903x_MutexLock(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

//ToDo
ADI_API adi_hal_Err_e linux_adrv903x_MutexUnlock(adi_hal_mutex_t* mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

/*
 * FileIO abstraction using Linux kernel firmware subsystem
 *
 */

int adrv903x_fseek(FILE * stream, long int offset, int origin)
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

long int adrv903x_ftell(FILE *stream)
{
	return stream->ptr - stream->start;
}

/*
 * No file logging when using Linux kernel firmware subsystem
 *
 */
int adrv903x_ferror(FILE *stream)
{
	return 0;
}

FILE* adrv903x_fopen(adi_hal_Cfg_t *hal, const char * filename, const char *mode)
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

int adrv903x_fclose(FILE *stream)
{
	if (stream == NULL)
		return -ENODEV;

	if (stream->fw)
		release_firmware(stream->fw);

	devm_kfree(&stream->hal->spi->dev, stream);

	return 0;
}

char *adrv903x_fgets(char *dst, int num, FILE *stream)
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

size_t adrv903x_fread(void *ptr, size_t size, size_t count, FILE *stream)
{
	int total = size * count;

	if ((stream->ptr + total) > stream->end)
		total = stream->end - stream->ptr;

	memcpy(ptr, stream->ptr, total);
	stream->ptr += total;

	return total;
}

/*
 * No file logging when using Linux kernel firmware subsystem
 *
 */
int adrv903x_fprintf(FILE *stream, const char *fmt, ...)
{
	return 0;
}

/*
 * No file logging when using Linux kernel firmware subsystem
 *
 */
int adrv903x_fflush(FILE *stream)
{
	return 0;
}

/*
 * Malloc using Linux kernel firmware subsystem
 *
 */
void *adrv903x_malloc(size_t size)
{
	return kzalloc(size, GFP_KERNEL);
}

/*
 * No adrv903x_CpuMemDumpBinWrite when using Linux kernel firmware subsystem
 *
 */
size_t adrv903x_fwrite(const void * ptr, size_t size, size_t count, FILE *stream)
{
	return 0;
}

/*
 * Timkeeping abstraction using Linux kernel firmware subsystem
 *
 */
void adrv903x_time(ktime_t *second)
{
	*second = ktime_get_seconds();
}

struct tm* adrv903x_gmtime(adi_hal_Cfg_t *hal, const ktime_t *timer)
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

/*
 * Memory allocation abstraction using Linux kernel firmware subsystem
 */

void *__adrv903x_calloc(size_t size, size_t nmemb)
{
	size_t total_size;

	total_size = size * nmemb;
	return kzalloc(total_size, GFP_KERNEL);
}
