// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI ADSP-SC5xx / SC59x TRU-backed SPI offload trigger provider.
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 *
 * These SoCs cannot forward a DREADY-style GPIO edge into a SPI DMA
 * channel directly — an edge-sensitive TRU generator is required:
 *
 *     DREADY pin → GP timer in EXTCLK mode (edge → TIMERn_TMR0m_GEN)
 *                → TRU routes to the SPI TX/RX DMA request receivers
 *                → the DMA channels advance one work unit per DREADY
 *                  (DMA_CFG.TWAIT)
 *
 * The timer is configured by timer-adi-adsp-sc5xx.c; this driver only
 * programs/clears the TRU generator → receiver routes when the SPI
 * offload consumer enables/disables the DATA_READY trigger.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <dt-bindings/soc/adi/sc5xx-tru.h>
#include <linux/soc/adi/icc.h>
#include <linux/spi/offload/provider.h>
#include <linux/spi/offload/types.h>

#define ADI_SC_TRU_MAX_SLAVES	4

struct adi_sc_tru_trigger {
	struct adi_tru *tru;
	u32 master;
	u32 slaves[ADI_SC_TRU_MAX_SLAVES];
	unsigned int nr_slaves;
};

static bool adi_sc_tru_match(struct spi_offload_trigger *trigger,
			     enum spi_offload_trigger_type type,
			     u64 *args, u32 nargs)
{
	return type == SPI_OFFLOAD_TRIGGER_DATA_READY && nargs == 0;
}

static int adi_sc_tru_enable(struct spi_offload_trigger *trigger,
			     struct spi_offload_trigger_config *config)
{
	struct adi_sc_tru_trigger *t = spi_offload_trigger_get_priv(trigger);
	unsigned int i;
	int ret;

	if (config->type != SPI_OFFLOAD_TRIGGER_DATA_READY)
		return -EINVAL;

	/*
	 * Install every requested route atomically from the framework's
	 * point of view (all-or-nothing). If any RSR write fails, undo
	 * whichever routes already went in.
	 */
	for (i = 0; i < t->nr_slaves; i++) {
		ret = adi_tru_set_trigger_by_id(t->tru, t->master, t->slaves[i]);
		if (ret)
			goto err;
	}
	return 0;

err:
	while (i-- > 0)
		adi_tru_set_trigger_by_id(t->tru, ADI_TRU_MST_NULL, t->slaves[i]);
	return ret;
}

static void adi_sc_tru_disable(struct spi_offload_trigger *trigger)
{
	struct adi_sc_tru_trigger *t = spi_offload_trigger_get_priv(trigger);
	unsigned int i;

	for (i = 0; i < t->nr_slaves; i++)
		adi_tru_set_trigger_by_id(t->tru, ADI_TRU_MST_NULL, t->slaves[i]);
}

static const struct spi_offload_trigger_ops adi_sc_tru_ops = {
	.match		= adi_sc_tru_match,
	.enable		= adi_sc_tru_enable,
	.disable	= adi_sc_tru_disable,
};

static int adi_sc_tru_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_offload_trigger_info info = { };
	struct adi_sc_tru_trigger *t;
	int ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	/* Walks the "adi,tru" phandle on our own DT node. */
	t->tru = get_adi_tru_from_node(dev);
	if (IS_ERR(t->tru))
		return dev_err_probe(dev, PTR_ERR(t->tru),
				     "TRU controller not (yet) available\n");

	ret = device_property_read_u32(dev, "adi,tru-master-id", &t->master);
	if (ret)
		return dev_err_probe(dev, ret,
				     "adi,tru-master-id required\n");

	ret = device_property_count_u32(dev, "adi,tru-slave-ids");
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "adi,tru-slave-ids required\n");
	if (ret == 0 || ret > ADI_SC_TRU_MAX_SLAVES)
		return dev_err_probe(dev, -EINVAL,
				     "adi,tru-slave-ids must list 1..%d ids\n",
				     ADI_SC_TRU_MAX_SLAVES);
	t->nr_slaves = ret;

	ret = device_property_read_u32_array(dev, "adi,tru-slave-ids",
					     t->slaves, t->nr_slaves);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read adi,tru-slave-ids\n");

	info.fwnode = dev_fwnode(dev);
	info.ops = &adi_sc_tru_ops;
	info.priv = t;

	ret = devm_spi_offload_trigger_register(dev, &info);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register SPI offload trigger\n");

	dev_info(dev, "TRU master=%u routed to %u SPI-DMA receiver(s)\n",
		 t->master, t->nr_slaves);
	return 0;
}

static const struct of_device_id adi_sc_tru_of_match[] = {
	{ .compatible = "adi,sc59x-spi-offload-trigger" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_sc_tru_of_match);

static struct platform_driver adi_sc_tru_driver = {
	.driver = {
		.name = "adi-sc-tru-offload-trigger",
		.of_match_table = adi_sc_tru_of_match,
	},
	.probe = adi_sc_tru_probe,
};
module_platform_driver(adi_sc_tru_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("ADI SC59x TRU-backed SPI offload trigger provider");
MODULE_LICENSE("GPL");
