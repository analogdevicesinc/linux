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

#include "linux_hal.h"

adiHalErr_t ADIHAL_writeToLog(void *devHalInfo, adiLogLevel_t logLevel, uint32_t errorCode, const char *comment)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;

	if(devHalInfo == NULL)
	{
		return (ADIHAL_GEN_SW);
	}

	if((devHalData->logLevel & ADIHAL_LOG_ERR) && (logLevel == ADIHAL_LOG_ERR))
	{
		dev_err(&devHalData->spi->dev, "ERROR: %d: %s", (int)errorCode, comment);
	}
	else if((devHalData->logLevel & ADIHAL_LOG_WARN) && (logLevel == ADIHAL_LOG_WARN))
	{
		dev_warn(&devHalData->spi->dev, "WARNING: %d: %s", (int)errorCode, comment);
	}
	else if((devHalData->logLevel & ADIHAL_LOG_MSG) && (logLevel == ADIHAL_LOG_MSG))
	{
		dev_info(&devHalData->spi->dev, "MESSAGE: %d: %s",(int)errorCode, comment);
	}
	else
	{
		dev_warn(&devHalData->spi->dev, "Undefined Log Level: 0x%X: %d: %s", logLevel, (int)errorCode, comment);
	}

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_setTimeout(void *devHalInfo, uint32_t halTimeout_ms)
{
	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_openHw(void *devHalInfo, uint32_t halTimeout_ms)
{
	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_closeHw(void *devHalInfo)
{
	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_resetHw(void *devHalInfo)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;

	if(devHalInfo == NULL)
	{
		return (ADIHAL_GEN_SW);
	}

	dev_info(&devHalData->spi->dev, "ADIHAL_resetHw");

	gpiod_set_value(devHalData->reset_gpio, 0);
	mdelay(1);
	gpiod_set_value(devHalData->reset_gpio, 1);

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_spiWriteByte(void *devHalInfo, uint16_t addr, uint8_t data)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;
	unsigned char txbuf[3];
	int ret;

	if (devHalData->logLevel & ADIHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev, "SPIWrite: ADDR:0x%03X, DATA:0x%02X \n", addr, data);
	}

	txbuf[0] = (addr >> 8) & 0x7F;
	txbuf[1] = addr & 0xFF;
	txbuf[2] = data;

	ret = spi_write_then_read(devHalData->spi, txbuf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&devHalData->spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return(ADIHAL_SPI_FAIL);
	}

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_spiWriteBytes(void *devHalInfo, uint16_t *addr, uint8_t *data, uint32_t count)
{
	adiHalErr_t errval;
	int i;

	for (i = 0; i < count; i++) {
		errval = ADIHAL_spiWriteByte(devHalInfo, addr[i], data[i]);
		if (errval)
			return errval;

	}

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_spiReadByte(void *devHalInfo, uint16_t addr, uint8_t *readdata)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;
	uint8_t data;
	int32_t retval;
	unsigned char txbuf[2];

	if(devHalInfo == NULL)
	{
		return (ADIHAL_GEN_SW);
	}

	txbuf[0] = BIT(7) | ((addr >> 8) & 0x7F);
	txbuf[1] = addr & 0xFF;

	retval = spi_write_then_read(devHalData->spi, txbuf, 2, &data, 1);
	if (retval < 0) {
		dev_err(&devHalData->spi->dev, "%s: failed (%d)\n",
			__func__, retval);;
		return(ADIHAL_SPI_FAIL);
	} else {
		*readdata =(uint8_t)data;
	}

	if (devHalData->logLevel & ADIHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev, "SPIRead: ADDR:0x%03X, ReadData:0x%02X\n",
			addr, *readdata);
	}

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_spiReadBytes(void *devHalInfo, uint16_t *addr, uint8_t *readdata, uint32_t count)
{
	adiHalErr_t errval;
	int i;

	for (i = 0; i < count; i++) {
		errval = ADIHAL_spiReadByte(devHalInfo, addr[i], &readdata[i]);
		if (errval)
			return errval;

	}

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_spiWriteField(void *devHalInfo,
				 uint16_t addr, uint8_t fieldVal, uint8_t mask, uint8_t startBit)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;
	adiHalErr_t errval;
	uint8_t readVal;

	errval = ADIHAL_spiReadByte(devHalInfo, addr, &readVal);
	if (errval < 0)
		return errval;

	readVal = (readVal & ~mask) | ((fieldVal << startBit) & mask);

	if (devHalData->logLevel & ADIHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev,"SPIWriteField: ADDR:0x%03X, FIELDVAL:0x%02X, MASK:0x%02X, STARTBIT:%d\n",
			addr, fieldVal, mask, startBit);
	}

	return ADIHAL_spiWriteByte(devHalInfo, addr, readVal);
}

adiHalErr_t ADIHAL_spiReadField(void *devHalInfo, uint16_t addr, uint8_t *fieldVal, uint8_t mask, uint8_t startBit)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;
	adiHalErr_t errval;
	uint8_t readVal;

	errval = ADIHAL_spiReadByte(devHalInfo, addr, &readVal);
	if (errval < 0)
		return errval;

	*fieldVal = ((readVal & mask) >> startBit);

	if(devHalData->logLevel & ADIHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev,"SPIReadField: ADDR:0x%03X, MASK:0x%02X, STARTBIT:%d, FieldVal:0x%02X\n",
			addr, mask, startBit, *fieldVal);
	}

	return ADIHAL_OK;
}

adiHalErr_t  ADIHAL_wait_us(void *devHalInfo, uint32_t time_us)
{
	usleep_range(time_us, time_us + 10);

	return ADIHAL_OK;
}

adiHalErr_t ADIHAL_setLogLevel(void *devHalInfo, uint16_t logLevel)
{
	struct adrv9009_hal *devHalData = (struct adrv9009_hal *)devHalInfo;

	if(devHalInfo == NULL)
	{
		return (ADIHAL_GEN_SW);
	}

	devHalData->logLevel = logLevel;

	return ADIHAL_OK;
}
