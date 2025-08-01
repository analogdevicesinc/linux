/*
 * Analog Devices AD9250-FMC-250EBZ/AD-FMCDAQ1-EBZ boards
 * SPI-SPI CPLD demux driver
 *
 * Copyright 2012-2016 Analog Devices Inc.
 * Author: Michael Hennerich <michael.hennerich@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define FMC_CPLD	0x00 /* chip_select 0 */
#define FMC_AD9517	0x84 /* chip_select 1 */
#define FMC_AD9250_0	0x80 /* chip_select 2 */
#define FMC_AD9250_1	0x81 /* chip_select 3 */
#define FMC_AD9129_0	0x82 /* chip_select 4 */
#define FMC_AD9129_1	0x83 /* chip_select 5 */
#define FMC_NUM_SLAVES	6

#define FMC_DAQ1_AD9684	0x80 /* chip_select 0 */
#define FMC_DAQ1_AD9122	0x81 /* chip_select 1 */
#define FMC_DAQ1_AD9523	0x82 /* chip_select 2 */
#define FMC_DAQ1_CPLD	0x83 /* chip_select 3 */

static const unsigned char cs_lut[FMC_NUM_SLAVES][2] = {
	{FMC_CPLD, FMC_DAQ1_AD9684},
	{FMC_AD9517, FMC_DAQ1_AD9122},
	{FMC_AD9250_0, FMC_DAQ1_AD9523},
	{FMC_AD9250_1, FMC_DAQ1_CPLD},
	{FMC_AD9129_0, 0},
	{FMC_AD9129_1, 0},
};

struct spi_ad9250 {
	struct spi_device *spi;
	uint8_t data[32] ____cacheline_aligned;
	unsigned id;
};

static inline unsigned cs_to_cpld (unsigned chip_select, unsigned id)
{
	return cs_lut[chip_select][id];
}

static int spi_ad9250_transfer_one(struct spi_controller *ctrl,
				   struct spi_message *msg)
{
	struct spi_ad9250 *spi_ad9250 = spi_controller_get_devdata(ctrl);
	struct spi_device *spi = msg->spi;
	int status = 0, i = 1;

	struct spi_transfer *tn;
	struct spi_message m;
	struct spi_transfer x[8];

	spi_message_init(&m);
	memset(x, 0, sizeof x);

	x[0].len = 1;
	x[0].tx_buf = spi_ad9250->data;
	x[0].delay.unit = SPI_DELAY_UNIT_USECS,
	x[0].delay.value = 10;
	spi_ad9250->data[0] = cs_to_cpld(spi_get_chipselect(spi, 0), spi_ad9250->id);
	spi_message_add_tail(&x[0], &m);

	list_for_each_entry(tn, &msg->transfers, transfer_list) {
		x[i] = *tn;
		spi_message_add_tail(&x[i], &m);
		i++;
		if (i > 7)
			return -EIO;
	}

 	spi_ad9250->spi->mode = spi->mode & ~SPI_3WIRE;
 	spi_setup(spi_ad9250->spi);

	spi_sync(spi_ad9250->spi, &m);

	msg->status = m.status;
	spi_finalize_current_message(ctrl);

	return status;
}

static int spi_ad9250_setup(struct spi_device *spi)
{
	if (spi->bits_per_word != 8)
		return -EINVAL;

	return 0;
}

static int spi_ad9250_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct spi_ad9250 *spi_ad9250;
	struct spi_controller *ctrl;

	ctrl = devm_spi_alloc_host(&spi->dev, sizeof(*spi_ad9250));
	if (!ctrl)
		return -ENOMEM;

	spi_ad9250 = spi_controller_get_devdata(ctrl);
	spi_ad9250->id = dev_id->driver_data;
	spi_ad9250->spi = spi;
	ctrl->num_chipselect = FMC_NUM_SLAVES;
	ctrl->mode_bits = SPI_CPHA | SPI_CPOL | SPI_3WIRE;
	ctrl->setup = spi_ad9250_setup;
	ctrl->transfer_one_message = spi_ad9250_transfer_one;
	ctrl->dev.of_node = spi->dev.of_node;

	return devm_spi_register_controller(&spi->dev, ctrl);
}

static const struct spi_device_id spi_ad9250_ids[] = {
	{ "spi-ad9250", 0 },
	{ "spi-adi-daq1", 1 },
	{ },
};
MODULE_DEVICE_TABLE(spi, spi_ad9250_ids);

static struct spi_driver spi_ad9250_driver = {
	.driver = {
		.name	= "spi-ad9250",
	},
	.id_table	= spi_ad9250_ids,
	.probe		= spi_ad9250_probe,
};
module_spi_driver(spi_ad9250_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9250-FMC/FMCDAQ1 boards SPI mux driver");
