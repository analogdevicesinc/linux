/**
 * \file common.c
 * \brief Contains Mykonos API common wrapper functions for user hardware platform drivers
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include "common.h"

ADI_LOGLEVEL CMB_LOGLEVEL = ADIHAL_LOG_ERROR | ADIHAL_LOG_WARNING /*| ADIHAL_LOG_MESSAGE*/;

commonErr_t CMB_closeHardware(void)
{
	return(COMMONERR_OK);
}

commonErr_t CMB_setGPIO(uint32_t GPIO)
{
	return(COMMONERR_OK);
}

commonErr_t CMB_hardReset(uint8_t spiChipSelectIndex)
{
	return(COMMONERR_FAILED);
}

commonErr_t CMB_setSPIOptions(spiSettings_t *spiSettings)
{
	return(COMMONERR_OK);
}

commonErr_t CMB_setSPIChannel(uint16_t chipSelectIndex )
{
	return(COMMONERR_OK);
}

commonErr_t CMB_SPIWriteByte(spiSettings_t *spiSettings, uint16_t addr, uint8_t data)
{
	int32_t retval = 0;
	unsigned char txbuf[] = {0x00,0x00,0x00};
	struct spi_device *spi = spiSettings->spi;

	if(CMB_LOGLEVEL & ADIHAL_LOG_SPI) {
		dev_info(&spi->dev, "SPIWrite: ADDR:0x%03X, DATA:0x%02X \n", addr, data);
	}

	txbuf[0] = ((0 & 1) << 7) | ((addr >> 8) & 0x7F);
	txbuf[1] = addr & 0xFF;
	txbuf[2] = data;

	retval = spi_write_then_read(spi, txbuf, 3, NULL, 0);
	if (retval < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, retval);
		return(COMMONERR_FAILED);
	}


	return(COMMONERR_OK);
}

commonErr_t CMB_SPIWriteBytes(spiSettings_t *spiSettings, uint16_t *addr,
			      uint8_t *data, uint32_t count)
{
	uint32_t i = 0;
	uint32_t txBufIndex = 0;
	int32_t retval = 0;
	unsigned char txbuf[SPIARRAYSIZE] = {0x00};
	uint32_t spiArrayTripSize = SPIARRAYTRIPSIZE;
	struct spi_device *spi = spiSettings->spi;

	txBufIndex = 0;

	for (i = 0; i < count; i++) {
		txbuf[txBufIndex++] = ((0 & 1) << 7) | ((addr[i] >> 8) & 0x7F);
		txbuf[txBufIndex++] = (addr[i] & 0xFF);
		txbuf[txBufIndex++] = data[i];

		if(CMB_LOGLEVEL & ADIHAL_LOG_SPI) {
			dev_info(&spi->dev, "SPIWrite: ADDR:0x%03X, DATA:0x%02X \n",
				 addr[i], data[i]);
		}

		/* Send full buffer when possible */
		if (txBufIndex >= spiArrayTripSize) {
			/* Send full buffer when possible */
			retval = spi_write_then_read(spi, txbuf, txBufIndex, NULL, 0);
			txBufIndex = 0;
		}
	}

	/* Send any data that was not sent as a full buffer before */
	if (txBufIndex > 0) {
		retval = spi_write_then_read(spi, txbuf, txBufIndex, NULL, 0);
		txBufIndex = 0;
	}


	if (retval < 0) {
		dev_err(&spi->dev, "Error writing SPI");
		return(COMMONERR_FAILED);
	}

	return(COMMONERR_OK);
}

commonErr_t CMB_SPIReadByte(spiSettings_t *spiSettings, uint16_t addr, uint8_t *readdata)
{
	uint8_t data=0;
	int32_t retval = 0;
	unsigned char txbuf[] = {0x00,0x00,0x00};
	struct spi_device *spi = spiSettings->spi;

	txbuf[0] = ((~0 & 1) << 7) | ((addr >> 8) & 0x7F);
	txbuf[1] = addr & 0xFF;

	retval = spi_write_then_read(spi, txbuf, 2, &data, 1);
	if (retval < 0) {
		dev_err(&spi->dev, "Error writing SPI");
		return(COMMONERR_FAILED);
	} else {
		*readdata =(uint8_t)data;
	}

	if(CMB_LOGLEVEL & ADIHAL_LOG_SPI) {
		dev_info(&spi->dev, "SPIRead: ADDR:0x%03X, ReadData:0x%02X\n",
			 addr, *readdata);
	}

	return(COMMONERR_OK);
}

commonErr_t CMB_SPIWriteField(spiSettings_t *spiSettings, uint16_t addr,
			      uint8_t field_val, uint8_t mask, uint8_t start_bit)
{
	struct spi_device *spi = spiSettings->spi;
	uint8_t Val = 0;

	if(CMB_LOGLEVEL & ADIHAL_LOG_SPI) {
		dev_info(&spi->dev, "SPIWriteField: ADDR:0x%03X, FIELDVAL:0x%02X, MASK:0x%02X, STARTBIT:%d\n",
			 addr, field_val, mask, start_bit);
	}

	if(CMB_SPIReadByte(spiSettings, addr, &Val)) {
		return(COMMONERR_FAILED);
	}

	Val = (Val & ~mask) | ((field_val << start_bit) & mask);

	if(CMB_SPIWriteByte(spiSettings, addr, Val)) {
		return(COMMONERR_FAILED);
	}

	return(COMMONERR_OK);
}

/* read a field in a single register space (not multibyte fields) */
commonErr_t CMB_SPIReadField(spiSettings_t *spiSettings, uint16_t addr,
			     uint8_t *field_val, uint8_t mask, uint8_t start_bit)
{
	struct spi_device *spi = spiSettings->spi;
	uint8_t data;

	if(CMB_SPIReadByte(spiSettings, addr, &data)) {
		return(COMMONERR_FAILED);
	}
	*field_val =(uint8_t)((data & mask) >> start_bit);

	if(CMB_LOGLEVEL & ADIHAL_LOG_SPI) {
		dev_info(&spi->dev, "SPIReadField: ADDR:0x%03X, MASK:0x%02X, STARTBIT:%d, FieldVal:0x%02X\n",
			 addr, mask, start_bit, *field_val);
	}

	return(COMMONERR_OK);
}

commonErr_t CMB_writeToLog(ADI_LOGLEVEL level, uint8_t deviceIndex,
			   uint32_t errorCode, const char *comment)
{

	if((CMB_LOGLEVEL & ADIHAL_LOG_ERROR) && (level == ADIHAL_LOG_ERROR)) {
		pr_err("ERROR: %d: %s", (int)errorCode, comment);
	} else if((CMB_LOGLEVEL & ADIHAL_LOG_WARNING) && (level == ADIHAL_LOG_WARNING)) {
		pr_warn("WARNING: %d: %s",(int)errorCode, comment);
	} else if((CMB_LOGLEVEL & ADIHAL_LOG_MESSAGE) && (level == ADIHAL_LOG_MESSAGE)) {
		pr_info("MESSAGE: %d: %s",(int)errorCode, comment);
	} else {
		pr_err("Undefined Log Level: 0x%X", level);
	}

	return(COMMONERR_OK);
}

/* if filename null, a default path will be used in logging.c */
commonErr_t CMB_openLog(const char *filename)
{
	return(COMMONERR_OK);
}

commonErr_t CMB_closeLog(void)
{
	return(COMMONERR_OK);
}

commonErr_t CMB_flushLog(void)
{
	return(COMMONERR_OK);
}

commonErr_t CMB_wait_ms(uint32_t time_ms)
{
	msleep(time_ms);

	return(COMMONERR_OK);
}

commonErr_t CMB_wait_us(uint32_t time_us)
{
	usleep_range(time_us, time_us + 10);

	return(COMMONERR_OK);
}

static unsigned long _desired_time_to_elapse = 0;

commonErr_t CMB_setTimeout_ms(uint32_t timeOut_ms)
{
	_desired_time_to_elapse = jiffies + msecs_to_jiffies(timeOut_ms);

	return(COMMONERR_OK);
}

commonErr_t CMB_setTimeout_us(uint32_t timeOut_us)
{
	_desired_time_to_elapse = jiffies + usecs_to_jiffies(timeOut_us);

	return(COMMONERR_OK);
}

commonErr_t CMB_hasTimeoutExpired(void)
{
	if (time_after(jiffies, _desired_time_to_elapse))
		return (COMMONERR_FAILED);
	else
		return(COMMONERR_OK);
}
