/*
 * AD7606 SPI ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include "ad7606.h"
//=============== add by HJW 2015-06-17 =========================
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
//============================================================
#define MAX_SPI_FREQ_HZ		23500000	/* VDRIVE above 4.75 V */

static int ad7606_spi_read_block(struct device *dev,
				 int count, void *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	int i, ret;
	unsigned short *data = buf;

	ret = spi_read(spi, buf, count * 2);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI read error\n");
		return ret;
	}

	for (i = 0; i < count; i++)
		data[i] = be16_to_cpu(data[i]);

	return 0;
}

static const struct ad7606_bus_ops ad7606_spi_bops = {
	.read_block	= ad7606_spi_read_block,
};

//=============== add by HJW 2015-06-17 =========================
static struct ad7606_platform_data *
ad7606_parse_dt(struct spi_device *spi)
{
	struct device_node *node = spi->dev.of_node;
	struct ad7606_platform_data *pdata;
	unsigned gpio_convst, gpio_reset, gpio_range, gpio_os0, gpio_os1, gpio_os2, gpio_frstdata, gpio_stby;
	pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
		return NULL; /* out of memory */

	/* no such property */
	if (of_property_read_u32(node, "ad7606,default_os", &pdata->default_os) != 0)
	{
		dev_err(&spi->dev, "default_os property is not defined.\n");
		return NULL;
	}
	/* no such property */
	if (of_property_read_u32(node, "ad7606,default_range", &pdata->default_range) != 0)
	{
		dev_err(&spi->dev, "default_range property is not defined.\n");
		return NULL;
	}

	/* now get the gpio number*/
	gpio_convst = of_get_named_gpio(node, "ad7606,gpio_convst",0);
	if (IS_ERR_VALUE(gpio_convst)) {
		dev_warn(&spi->dev, "gpio_convst can not setup, set it to -1.\n");
		pdata->gpio_convst = -1;
	}
	else
	{
		pdata->gpio_convst = gpio_convst;
	}

	gpio_reset = of_get_named_gpio(node, "ad7606,gpio_reset",0);
	if (IS_ERR_VALUE(gpio_reset)) {
		dev_warn(&spi->dev, "gpio_reset can not setup, set it to -1.\n");
		pdata->gpio_reset = -1;
	}
	else
	{
		pdata->gpio_reset = gpio_reset;
	}

	gpio_range = of_get_named_gpio(node, "ad7606,gpio_range",0);
	if (IS_ERR_VALUE(gpio_range)) {
		dev_warn(&spi->dev, "gpio_range can not setup, set it to -1.\n");
		pdata->gpio_range = -1;
	}
	else
	{
		pdata->gpio_range = gpio_range;
	}

	gpio_os0 = of_get_named_gpio(node, "ad7606,gpio_os0",0);
	if (IS_ERR_VALUE(gpio_os0)) {
		dev_warn(&spi->dev, "gpio_os0 can not setup, set it to -1.\n");
		pdata->gpio_os0 = -1;
	}
	else
	{
		pdata->gpio_os0 = gpio_os0;
	}

	gpio_os1 = of_get_named_gpio(node, "ad7606,gpio_os1",0);
	if (IS_ERR_VALUE(gpio_os1)) {
		dev_warn(&spi->dev, "gpio_os1 can not setup, set it to -1.\n");
		pdata->gpio_os1 = -1;
	}
	else
	{
		pdata->gpio_os1 = gpio_os1;
	}

	gpio_os2 = of_get_named_gpio(node, "ad7606,gpio_os2",0);
	if (IS_ERR_VALUE(gpio_os2)) {
		dev_warn(&spi->dev, "gpio_os2 can not setup, set it to -1.\n");
		pdata->gpio_os2 = -1;
	}
	else
	{
		pdata->gpio_os2 = gpio_os2;
	}

	gpio_frstdata = of_get_named_gpio(node, "ad7606,gpio_frstdata",0);
	if (IS_ERR_VALUE(gpio_frstdata)) {
		dev_warn(&spi->dev, "gpio_frstdata can not setup, set it to -1.\n");
		pdata->gpio_frstdata = -1;
	}
	else
	{
		pdata->gpio_frstdata = gpio_frstdata;
	}

	gpio_stby = of_get_named_gpio(node, "ad7606,gpio_stby",0);
	if (IS_ERR_VALUE(gpio_stby)) {
		dev_warn(&spi->dev, "gpio_stby can not setup, set it to -1.\n");
		pdata->gpio_stby = -1;
	}
	else
	{
		pdata->gpio_stby = gpio_stby;
	}
	dev_info(&spi->dev, "DT parse result:\ndefault_os=%d.\ndefault_range=%d.\ngpio_convst=%d.\ngpio_reset=%d.\ngpio_range=%d.\ngpio_os0=%d.\ngpio_os1=%d.\ngpio_os2=%d.\ngpio_frstdata=%d.\ngpio_stby=%d.\n",
		pdata->default_os,pdata->default_range,pdata->gpio_convst,
		pdata->gpio_reset,pdata->gpio_range,pdata->gpio_os0,pdata->gpio_os1,
		pdata->gpio_os2,pdata->gpio_frstdata,pdata->gpio_stby);
	/* pdata is filled */
	return pdata;
}
//=============================================================

static int ad7606_spi_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
//=============== add by HJW 2015-06-17 =========================
	struct ad7606_platform_data *pdata;

	if (spi->dev.of_node != NULL)
	{
		pdata = ad7606_parse_dt(spi);
		if(pdata!=NULL)
			spi->dev.platform_data = pdata;
	}
//=============================================================
	indio_dev = ad7606_probe(&spi->dev, spi->irq, NULL,
			   spi_get_device_id(spi)->driver_data,
			   &ad7606_spi_bops);

	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	spi_set_drvdata(spi, indio_dev);

	return 0;
}

static int ad7606_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&spi->dev);

	return ad7606_remove(indio_dev, spi->irq);
}

#ifdef CONFIG_PM
static int ad7606_spi_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	ad7606_suspend(indio_dev);

	return 0;
}

static int ad7606_spi_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	ad7606_resume(indio_dev);

	return 0;
}

static const struct dev_pm_ops ad7606_pm_ops = {
	.suspend = ad7606_spi_suspend,
	.resume  = ad7606_spi_resume,
};
#define AD7606_SPI_PM_OPS (&ad7606_pm_ops)

#else
#define AD7606_SPI_PM_OPS NULL
#endif

static const struct spi_device_id ad7606_id[] = {
	{"ad7606-8", ID_AD7606_8},
	{"ad7606-6", ID_AD7606_6},
	{"ad7606-4", ID_AD7606_4},
	{}
};
MODULE_DEVICE_TABLE(spi, ad7606_id);

//=============== add by HJW 2015-06-17 =========================
//=============== only support AD7606-8 now======================
#ifdef CONFIG_OF
static const struct of_device_id ad7606_dt_ids[] = {
	{ .compatible = "adi,ad7606-8" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ad7606_dt_ids);
#endif
//=============================================================

static struct spi_driver ad7606_driver = {
	.driver = {
		.name = "ad7606",
		.owner = THIS_MODULE,
		.pm    = AD7606_SPI_PM_OPS,
//=============== add by HJW 2015-06-17 =========================
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ad7606_dt_ids),
#endif
//=============================================================
	},
	.probe = ad7606_spi_probe,
	.remove = ad7606_spi_remove,
	.id_table = ad7606_id,
};
module_spi_driver(ad7606_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
