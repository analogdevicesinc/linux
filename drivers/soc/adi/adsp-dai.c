// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices ADSP DAI driver
 *
 * Each ADSP SoC normally has two DAIs. A DAI exposes several audio-related
 * functions in the same register map. Since the registers are split
 * non-contiguously, this driver creates a single MMIO register map which is
 * used by a number of auxiliary drivers for each function. The driver is also
 * responsible for registering the corresponding auxiliary devices.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 */

#include <linux/auxiliary_bus.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

/**
 * struct adsp_dai_info - SoC description
 * @regs: physical base addresses of the DAIs, indexed by DAI number
 * @num_dais: number of DAIs
 */
struct adsp_dai_info {
	const phys_addr_t *regs;
	unsigned int num_dais;
};

static const phys_addr_t sc573_dai_regs[] = { 0x310c9000 };

static const struct adsp_dai_info sc573_dai_info = {
	.regs = sc573_dai_regs,
	.num_dais = ARRAY_SIZE(sc573_dai_regs),
};

static const phys_addr_t sc589_dai_regs[] = { 0x310c9000, 0x310cb000 };

static const struct adsp_dai_info sc589_dai_info = {
	.regs = sc589_dai_regs,
	.num_dais = ARRAY_SIZE(sc589_dai_regs),
};

static const phys_addr_t sc598_dai_regs[] = { 0x310c9000, 0x310ca000 };

static const struct adsp_dai_info sc598_dai_info = {
	.regs = sc598_dai_regs,
	.num_dais = ARRAY_SIZE(sc598_dai_regs),
};

static const struct regmap_config adsp_dai_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x3fc,
};

static const char *const adsp_dai_auxdevs[] = {
	"sru",
};

static int adsp_dai_get_id(struct device *dev, const struct resource *res)
{
	const struct adsp_dai_info *info = device_get_match_data(dev);

	for (unsigned int i = 0; i < info->num_dais; i++)
		if (info->regs[i] == res->start)
			return i;

	return dev_err_probe(dev, -EINVAL, "unknown DAI at %pa\n",
			     &res->start);
}

static int adsp_dai_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *base;
	struct clk *sclk;
	struct regmap *regmap;
	int id;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	id = adsp_dai_get_id(dev, res);
	if (id < 0)
		return id;

	sclk = devm_clk_get_enabled(dev, "sclk");
	if (IS_ERR(sclk))
		return dev_err_probe(dev, PTR_ERR(sclk), "failed to get sclk\n");

	regmap = devm_regmap_init_mmio(dev, base, &adsp_dai_regmap_cfg);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	/*
	 * Create the auxiliary devices for the logical DAI functions. The
	 * devices will have names like adsp_dai.sru.0 for the SRU of DAI0.
	 */
	for (unsigned int i = 0; i < ARRAY_SIZE(adsp_dai_auxdevs); i++) {
		const char *aux = adsp_dai_auxdevs[i];

		if (!__devm_auxiliary_device_create(dev, KBUILD_MODNAME, aux,
						    NULL, id))
			return dev_err_probe(dev, -ENODEV,
					     "failed to create %s.%d\n", aux,
					     id);
	}

	return 0;
}

static const struct of_device_id adsp_dai_of_match[] = {
	{ .compatible = "adi,adsp-sc573-dai", .data = &sc573_dai_info },
	{ .compatible = "adi,adsp-sc589-dai", .data = &sc589_dai_info },
	{ .compatible = "adi,adsp-sc598-dai", .data = &sc598_dai_info },
	{ .compatible = "adi,adsp-sc846-dai", .data = &sc598_dai_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adsp_dai_of_match);

static struct platform_driver adsp_dai_driver = {
	.probe = adsp_dai_probe,
	.driver = {
		.name = "adsp-dai",
		.of_match_table = adsp_dai_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(adsp_dai_driver);

MODULE_AUTHOR("Alvin Šipraga <alvin.sipraga@analog.com>");
MODULE_DESCRIPTION("ADI ADSP DAI driver");
MODULE_LICENSE("GPL");
