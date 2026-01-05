// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright 2015 - 2025 Analog Devices Inc.
 * Released under the ADRV904X API license, for more information.
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

typedef struct {
	void *value;
} thread_to_value_t;

/* ADI Error Reporting Dependency on HAL_TLS_ERR being set to NULL if not configured */
static thread_to_value_t store = { NULL };

#include "adi_common_error.h"

adi_hal_Err_e linux_adrv904x_HwOpen(void *devHalCfg)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_HwReset(void *devHalCfg, u8 pinLevel)
{
	adi_hal_Cfg_t *halCfg;

	if (!devHalCfg)
		return ADI_HAL_ERR_NULL_PTR;

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	gpiod_set_value(halCfg->reset_gpio, pinLevel);

	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_HwClose(void *devHalCfg)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_SpiWrite(void * const devHalCfg,
					      const u8 txData[],
					      const u32 numTxBytes)
{
	static const u32 MAX_SIZE = 4096;
	u32 toWrite = 0;
	s32 remaining = numTxBytes;
	s32 halError = (s32)ADI_HAL_ERR_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (!devHalCfg) {
		halError = (s32)ADI_HAL_ERR_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->spiCfg.interfaceEnabled != 0) {
		s32 result = 0;

		do {
			toWrite = (remaining > MAX_SIZE) ? MAX_SIZE : remaining;
			result = spi_write(halCfg->spi,
					   &txData[numTxBytes - remaining],
					   toWrite);
			if (result < 0)
				return ADI_HAL_ERR_SPI_WRITE;

			remaining -= toWrite;
		} while (remaining > 0);
	}

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv904x_SpiRead(void *const devHalCfg,
					     const u8 txData[],
					     u8 rxData[],
					     const u32 numTxRxBytes)
{
	static const u32 MAX_SIZE = 4096;
	s32 remaining = numTxRxBytes;
	s32 halError = (s32)ADI_HAL_ERR_OK;
	s32 result = 0;
	adi_hal_Cfg_t *halCfg = NULL;

	if (!devHalCfg) {
		halError = (s32)ADI_HAL_ERR_NULL_PTR;
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
			if (result < 0)
				halError = ADI_HAL_ERR_SPI_READ;

			remaining -= t.len;
		} while (remaining > 0);
	}

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv904x_LogFileOpen(void *devHalCfg, const char *filename)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_LogFileClose(void *devHalCfg)
{
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_LogLevelSet(void *const devHalCfg,
						 const u32 logMask)
{
	adi_hal_Cfg_t *halCfg = NULL;

	if (!devHalCfg)
		return ADI_HAL_ERR_PARAM;

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	halCfg->logCfg.logMask = (logMask & (s32)ADI_HAL_LOG_ALL);

	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_LogLevelGet(void *const devHalCfg,
						 u32 *const logMask)
{
	s32 halError = (s32)ADI_HAL_ERR_OK;
	adi_hal_Cfg_t *halCfg = NULL;

	if (!devHalCfg) {
		halError = (s32)ADI_HAL_ERR_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	*logMask = halCfg->logCfg.logMask;

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv904x_LogWrite(void *const devHalCfg,
					      const adi_hal_LogLevel_e logLevel,
					      const u8 indent,
					      const char *const comment,
					      va_list argp)
{
	s32 halError = (s32)ADI_HAL_ERR_OK;
	s32 result = 0;
	adi_hal_Cfg_t *halCfg = NULL;
	static char logMessage[ADI_HAL_MAX_LOG_LINE] = { 0 };

	const char *logLevelChar = NULL;

	logMessage[0] = 0;

	if (!devHalCfg) {
		halError = (s32)ADI_HAL_ERR_NULL_PTR;
		return halError;
	}

	halCfg = (adi_hal_Cfg_t *)devHalCfg;

	if (halCfg->logCfg.logMask == (s32)ADI_HAL_LOG_NONE) {
		/* If logging disabled, exit gracefully */
		halError = (s32)ADI_HAL_ERR_OK;
		return halError;
	}

	if (logLevel > (s32)ADI_HAL_LOG_ALL) {
		halError = (s32)ADI_HAL_ERR_LOG;
		return halError;
	}

	/* Print Log type */
	if ((halCfg->logCfg.logMask & ADI_HAL_LOG_MSG) &&
	    logLevel == (s32)ADI_HAL_LOG_MSG) {
		logLevelChar = "MESSAGE:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_WARN) &&
		   logLevel == (s32)ADI_HAL_LOG_WARN) {
		logLevelChar = "WARNING:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_ERR) &&
		   logLevel == (s32)ADI_HAL_LOG_ERR) {
		logLevelChar = "ERROR:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_API) &&
		   logLevel == (s32)ADI_HAL_LOG_API) {
		logLevelChar = "API_LOG:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_HAL) &&
		   logLevel == (s32)ADI_HAL_LOG_HAL) {
		logLevelChar = "ADI_HAL_LOG:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_SPI) &&
		   logLevel == (s32)ADI_HAL_LOG_SPI) {
		logLevelChar = "SPI_LOG:";
	} else if ((halCfg->logCfg.logMask & ADI_HAL_LOG_API_PRIV) &&
		   logLevel == (s32)ADI_HAL_LOG_API_PRIV) {
		logLevelChar = "API_PRIV_LOG:";
	} else {
		/* Nothing to log - exit cleanly */
		return (s32)ADI_HAL_ERR_OK;
	}

	result = snprintf(logMessage, ADI_HAL_MAX_LOG_LINE, "%s", logLevelChar);
	if (result < 0) {
		halError = (s32)ADI_HAL_ERR_LOG;
		return halError;
	}

	result = vsnprintf(logMessage + strlen(logMessage),
			   ADI_HAL_MAX_LOG_LINE, comment, argp);
	if (result < 0) {
		halError = (s32)ADI_HAL_ERR_LOG;
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
		pr_info("%s", logMessage);
		break;
	}

	pr_info("MESSAGE: %s\n", logMessage);

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv904x_TimerWait_us(void *devHalCfg,
						  u32 time_us)
{
	s32 halError = (s32)ADI_HAL_ERR_OK;

	usleep_range(time_us, time_us + 10);

	return halError;
}

ADI_API adi_hal_Err_e linux_adrv904x_TimerWait_ms(void *devHalCfg,
						  u32 time_ms)
{
	s32 halError = (s32)ADI_HAL_ERR_OK;

	msleep(time_ms);

	return halError;
}

ADI_API void *linux_adrv904x_TlsGet(const adi_hal_TlsType_e tlsType)
{
	if (tlsType == HAL_TLS_END)
		return NULL;

	return store.value;
}

ADI_API adi_hal_Err_e linux_adrv904x_TlsSet(const adi_hal_TlsType_e tlsType,
					    void *const value)
{
	if (tlsType == HAL_TLS_END) {
		/* Special convenience case that a thread can remove all it TLS
		 * items in one call by passing (HAL_TLS_END, NULL);
		 */
		if (value)
			return ADI_HAL_ERR_PARAM;

		/* Passing NULL cannot fail by definition - no need to check rtn values */
		(void)linux_adrv904x_TlsSet(HAL_TLS_ERR, NULL);
		(void)linux_adrv904x_TlsSet(HAL_TLS_USR, NULL);

		return ADI_HAL_ERR_OK;
	}

	store.value = value;
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_MutexInit(adi_hal_mutex_t *mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_MutexDestroy(adi_hal_mutex_t *mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_MutexLock(adi_hal_mutex_t *mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

ADI_API adi_hal_Err_e linux_adrv904x_MutexUnlock(adi_hal_mutex_t *mutex)
{
	(void)mutex;
	return ADI_HAL_ERR_OK;
}

/*
 * FileIO abstraction using Linux kernel firmware subsystem
 *
 */

int adrv904x_fseek(FILE *stream, long offset, int origin)
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

long adrv904x_ftell(FILE *stream)
{
	return stream->ptr - stream->start;
}

/*
 * No file logging when using Linux kernel firmware subsystem
 *
 */
int adrv904x_ferror(FILE *stream)
{
	return 0;
}

FILE *adrv904x_fopen(adi_hal_Cfg_t *hal, const char *filename, const char *mode)
{
	int ret;

	FILE *stream = devm_kzalloc(&hal->spi->dev, sizeof(*stream), GFP_KERNEL);

	stream->hal = hal;

	ret = request_firmware(&stream->fw, filename, &hal->spi->dev);
	if (ret < 0)
		return NULL;

	stream->start = (char *)stream->fw->data;
	stream->ptr = stream->start;
	stream->end = stream->start + stream->fw->size;

	return stream;
}

int adrv904x_fclose(FILE *stream)
{
	if (!stream)
		return -ENODEV;

	if (stream->fw)
		release_firmware(stream->fw);

	devm_kfree(&stream->hal->spi->dev, stream);

	return 0;
}

char *adrv904x_fgets(char *dst, int num, FILE *stream)
{
	char *p;
	int c;

	for (p = dst, num--; num > 0; num--) {
		if (stream->ptr + 1 > stream->end)
			return NULL;

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

size_t adrv904x_fread(void *ptr, size_t size, size_t count, FILE *stream)
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
int adrv904x_fprintf(FILE *stream, const char *fmt, ...)
{
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = vprintk(fmt, args);
	va_end(args);

	return ret;
}

/*
 * No file logging when using Linux kernel firmware subsystem
 *
 */
int adrv904x_fflush(FILE *stream)
{
	return 0;
}

/*
 * Malloc using Linux kernel firmware subsystem
 *
 */
void *adrv904x_malloc(size_t size)
{
	return kzalloc(size, GFP_KERNEL);
}

/*
 * No adrv904x_CpuMemDumpBinWrite when using Linux kernel firmware subsystem
 *
 */
size_t adrv904x_fwrite(const void *ptr, size_t size, size_t count, FILE *stream)
{
	return 0;
}

/*
 * Timkeeping abstraction using Linux kernel firmware subsystem
 *
 */
void adrv904x_time(ktime_t *second)
{
	*second = ktime_get_seconds();
}

struct tm *adrv904x_gmtime(adi_hal_Cfg_t *hal, const ktime_t *timer)
{
	struct tm *tmPtr = devm_kzalloc(&hal->spi->dev, sizeof(*tmPtr), GFP_KERNEL);
	struct timespec64 ts;

	if (!tmPtr)
		return NULL;

	ts = ktime_to_timespec64(*timer);
	time64_to_tm(ts.tv_sec, 0, tmPtr);

	return tmPtr;
}

/*
 * Memory allocation abstraction using Linux kernel firmware subsystem
 */

void *adrv904x_calloc(size_t size, size_t nmemb)
{
	size_t total_size;

	total_size = size * nmemb;
	return kzalloc(total_size, GFP_KERNEL);
}
