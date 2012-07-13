/*
 * SPI OF support routines
 * Copyright (C) 2008 Secret Lab Technologies Ltd.
 *
 * Support routines for deriving SPI device attachments from the device
 * tree.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/of_irq.h>
#include <linux/of_spi.h>

/**
 * of_register_spi_devices - Register child devices onto the SPI bus
 * @master:	Pointer to spi_master device
 *
 * Registers an spi_device for each child node of master node which has a 'reg'
 * property.
 */
void of_register_spi_devices(struct spi_master *master)
{
	struct spi_device *spi;
	struct device_node *nc;
	const __be32 *prop;
	int rc;
	int len;

	if (!master->dev.of_node)
		return;

	for_each_child_of_node(master->dev.of_node, nc) {
		/* Alloc an spi_device */
		spi = spi_alloc_device(master);
		if (!spi) {
			dev_err(&master->dev, "spi_device alloc error for %s\n",
				nc->full_name);
			spi_dev_put(spi);
			continue;
		}

		/* Select device driver */
		if (of_modalias_node(nc, spi->modalias,
				     sizeof(spi->modalias)) < 0) {
			dev_err(&master->dev, "cannot find modalias for %s\n",
				nc->full_name);
			spi_dev_put(spi);
			continue;
		}

		/* Device address */
		prop = of_get_property(nc, "reg", &len);
		if (!prop || len < sizeof(*prop)) {
			dev_err(&master->dev, "%s has no 'reg' property\n",
				nc->full_name);
			spi_dev_put(spi);
			continue;
		}
		spi->chip_select = be32_to_cpup(prop);

		/* Mode (clock phase/polarity/etc.) */
		if (of_find_property(nc, "spi-cpha", NULL))
			spi->mode |= SPI_CPHA;
		if (of_find_property(nc, "spi-cpol", NULL))
			spi->mode |= SPI_CPOL;
		if (of_find_property(nc, "spi-cs-high", NULL))
			spi->mode |= SPI_CS_HIGH;
		if (of_find_property(nc, "spi-3wire", NULL))
			spi->mode |= SPI_3WIRE;


		/* Device speed */
		prop = of_get_property(nc, "spi-max-frequency", &len);
		if (!prop || len < sizeof(*prop)) {
			dev_err(&master->dev, "%s has no 'spi-max-frequency' property\n",
				nc->full_name);
			spi_dev_put(spi);
			continue;
		}
		spi->max_speed_hz = be32_to_cpup(prop);

		/* IRQ */
		spi->irq = irq_of_parse_and_map(nc, 0);

		/* Store a pointer to the node in the device structure */
		of_node_get(nc);
		spi->dev.of_node = nc;

		/* Register the new device */
		request_module(spi->modalias);
		rc = spi_add_device(spi);
		if (rc) {
			dev_err(&master->dev, "spi_device register error %s\n",
				nc->full_name);
			spi_dev_put(spi);
		}

	}
}
EXPORT_SYMBOL(of_register_spi_devices);

static int __spi_master_of_match(struct device *dev, void *data)
{
	struct device_node *of_node = data;
	return dev->of_node == of_node;
}

/**
 * spi_of_node_to_master - look up master associated with of_node
 * @of_node: pointer to the device tree node.
 * Context: can sleep
 *
 * Returns a pointer to the relevant spi_master, or NULL if there is
 * no such master registered.
 */
struct spi_master *spi_of_node_to_master(struct device_node *of_node)
{
	struct device *dev;
	struct spi_master *master = NULL;

	dev = class_find_device(&spi_master_class, NULL, of_node,
				__spi_master_of_match);
	if (dev)
		master = container_of(dev, struct spi_master, dev);

	return master;
}
EXPORT_SYMBOL_GPL(spi_of_node_to_master);
