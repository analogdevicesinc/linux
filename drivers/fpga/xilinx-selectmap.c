// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xilinx Spartan6 and 7 Series SelectMAP interface driver
 *
 * (C) 2024 Charles Perry <charles.perry@savoirfairelinux.com>
 *
 * Manage Xilinx FPGA firmware loaded over the SelectMAP configuration
 * interface.
 */

#include "xilinx-core.h"

#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define IP_DATA_REG	0x18

enum xilinx_selectmap_device_ids {
	ID_XC7S_SELMAP,
	ID_XC7A_SELMAP,
	ID_XC7K_SELMAP,
	ID_XC7V_SELMAP,
	ID_ADI_8_SELMAP,
	ID_ADI_16_SELMAP,
};

struct xilinx_selectmap_conf {
	struct xilinx_fpga_core core;
	void __iomem *base;
	enum xilinx_selectmap_device_ids id;
};

#define to_xilinx_selectmap_conf(obj) \
	container_of(obj, struct xilinx_selectmap_conf, core)

static int xilinx_selectmap_write(struct xilinx_fpga_core *core,
				  const char *buf, size_t count)
{
	struct xilinx_selectmap_conf *conf = to_xilinx_selectmap_conf(core);
	size_t i;
	const u16 *buf16 = (u16 *)buf;

	switch (conf->id) {
	case ID_ADI_8_SELMAP:
		for (i = 0; i < count; ++i)
			writeb(buf[i], conf->base + IP_DATA_REG);
		break;
	case ID_ADI_16_SELMAP:
		for (i = 0; i < (count/2)+1; ++i)
			writew(cpu_to_be16(buf16[i]), conf->base + IP_DATA_REG);
		break;
	default:
		for (i = 0; i < count; ++i)
			writeb(buf[i], conf->base);
		break;
	}

	return 0;
}

static int xilinx_selectmap_probe(struct platform_device *pdev)
{
	struct xilinx_selectmap_conf *conf;
	struct gpio_desc *gpio;
	void __iomem *base;

	conf = devm_kzalloc(&pdev->dev, sizeof(*conf), GFP_KERNEL);
	if (!conf)
		return -ENOMEM;

	conf->core.dev = &pdev->dev;
	conf->core.write = xilinx_selectmap_write;
	conf->id = (enum xilinx_selectmap_device_ids)device_get_match_data(&pdev->dev);

	base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(base))
		return dev_err_probe(&pdev->dev, PTR_ERR(base),
				     "ioremap error\n");
	conf->base = base;

	/* CSI_B is active low */
	gpio = devm_gpiod_get_optional(&pdev->dev, "csi", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpio),
				     "Failed to get CSI_B gpio\n");

	/* RDWR_B is active low */
	gpio = devm_gpiod_get_optional(&pdev->dev, "rdwr", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpio),
				     "Failed to get RDWR_B gpio\n");

	return xilinx_core_probe(&conf->core);
}

static const struct of_device_id xlnx_selectmap_of_match[] = {
	{ .compatible = "xlnx,fpga-xc7s-selectmap", .data = (void *)ID_XC7S_SELMAP, }, // Spartan-7
	{ .compatible = "xlnx,fpga-xc7a-selectmap", .data = (void *)ID_XC7A_SELMAP, }, // Artix-7
	{ .compatible = "xlnx,fpga-xc7k-selectmap", .data = (void *)ID_XC7K_SELMAP, }, // Kintex-7
	{ .compatible = "xlnx,fpga-xc7v-selectmap", .data = (void *)ID_XC7V_SELMAP, }, // Virtex-7
	{ .compatible = "adi,fpga-8-selectmap", .data = (void *)ID_ADI_8_SELMAP, }, // ADI 8bit version 
	{ .compatible = "adi,fpga-16-selectmap", .data = (void *)ID_ADI_16_SELMAP, }, // ADI 16bit version
	{},
};
MODULE_DEVICE_TABLE(of, xlnx_selectmap_of_match);

static struct platform_driver xilinx_selectmap_driver = {
	.driver = {
		.name = "xilinx-selectmap",
		.of_match_table = xlnx_selectmap_of_match,
	},
	.probe  = xilinx_selectmap_probe,
};

module_platform_driver(xilinx_selectmap_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Charles Perry <charles.perry@savoirfairelinux.com>");
MODULE_DESCRIPTION("Load Xilinx FPGA firmware over SelectMap");
