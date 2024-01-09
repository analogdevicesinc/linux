// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2006 Juergen Beisert, Pengutronix
 * Copyright (C) 2008 Guennadi Liakhovetski, Pengutronix
 * Copyright (C) 2009 Wolfram Sang, Pengutronix
 *
 * Check max730x.c for further details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/max7301.h>

#include <asm/unaligned.h>

/* A write to the MAX7301 means one message with one transfer */
static int max7301_spi_write(struct device *dev, unsigned int reg,
				unsigned int val)
{
	struct max7301 *ts = dev_get_drvdata(dev);
	struct spi_device *spi = to_spi_device(dev);
	put_unaligned_be16((reg & 0x7F) << 8 | (val & 0xFF), &ts->data[0]);

	return spi_write(spi, &ts->data[0], 2);
}

/* A read from the MAX7301 means two transfers; here, one message each */
static int max7301_spi_read(struct device *dev, unsigned int reg)
{
	int ret;
	u16 word;
	struct spi_transfer t = {0};
	struct max7301 *ts = dev_get_drvdata(dev);
	struct spi_device *spi = to_spi_device(dev);

	put_unaligned_be16(0x80 | ((reg & 0x7F) << 8), &ts->data[0]);
	ret = spi_write(spi, &ts->data[0], 2);

	// send no-op command and receive the readout
	ts->data[0] = 0x00;
	ts->data[1] = 0x00;

	t.rx_buf = &ts->data[0];
	t.tx_buf = &ts->data[0];
	t.len = 2;

	ret = spi_sync_transfer(spi, &t, 1);
	if (ret)
		return ret;

	word = get_unaligned_be16(&ts->data[0]) & 0xFF;

	return word;
}

static int max7301_probe(struct spi_device *spi)
{
	struct max7301 *ts;
	int ret;

	ts = devm_kzalloc(&spi->dev, sizeof(struct max7301), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->read = max7301_spi_read;
	ts->write = max7301_spi_write;
	ts->dev = &spi->dev;

	ret = __max730x_probe(ts);
	return ret;
}

static int max7301_remove(struct spi_device *spi)
{
	return __max730x_remove(&spi->dev);
}

static const struct spi_device_id max7301_id[] = {
	{ "max7301", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, max7301_id);

static struct spi_driver max7301_driver = {
	.driver = {
		.name = "max7301",
	},
	.probe = max7301_probe,
	.remove = max7301_remove,
	.id_table = max7301_id,
};

static int __init max7301_init(void)
{
	return spi_register_driver(&max7301_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(max7301_init);

static void __exit max7301_exit(void)
{
	spi_unregister_driver(&max7301_driver);
}
module_exit(max7301_exit);

MODULE_AUTHOR("Juergen Beisert, Wolfram Sang");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX7301 GPIO-Expander");
