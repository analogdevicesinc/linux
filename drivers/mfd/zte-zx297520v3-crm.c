// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Stefan Dösinger
 *
 * The zx297520v3 CRM controllers are primarily clock controllers, but they all have reset controls
 * as well. Top and Matrix controllers have further auxiliary functions: Reboot and USB PHY in Top,
 * hardware mutex in Matrix. This driver breaks the functionality apart to handle them in drivers in
 * individual subsystems.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>

enum zx_device_type {
	TYPE_INVALID = 0,
	TYPE_TOPCRM,
	TYPE_MATRIXCRM,
	TYPE_LSPCRM,
};

static const struct mfd_cell zx297520v3_topcrm_cells[] = {
	MFD_CELL_NAME("zx297520v3-topclk"),
	MFD_CELL_NAME("zx297520v3-topreset"),
	MFD_CELL_OF("reboot", NULL, NULL, 0, 0, "syscon-reboot"),
	MFD_CELL_NAME("zx297520v3-usb-phy"),
};

static const struct mfd_cell zx297520v3_matrixcrm_cells[] = {
	MFD_CELL_NAME("zx297520v3-matrixclk"),
	MFD_CELL_NAME("zx297520v3-matrixreset"),
};

static const struct mfd_cell zx297520v3_lspcrm_cells[] = {
	MFD_CELL_NAME("zx297520v3-lspclk"),
	MFD_CELL_NAME("zx297520v3-lspreset"),
};

static int zx297520v3_crm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct mfd_cell *cells;
	struct reset_control *reset;
	enum zx_device_type type;
	unsigned int num_cells;
	struct clk *pclk;

	type = (enum zx_device_type)(kernel_ulong_t)device_get_match_data(dev);
	switch (type) {
	case TYPE_TOPCRM:
		cells = zx297520v3_topcrm_cells;
		num_cells = ARRAY_SIZE(zx297520v3_topcrm_cells);
		break;

	case TYPE_MATRIXCRM:
		cells = zx297520v3_matrixcrm_cells;
		num_cells = ARRAY_SIZE(zx297520v3_matrixcrm_cells);
		break;

	case TYPE_LSPCRM:
		cells = zx297520v3_lspcrm_cells;
		num_cells = ARRAY_SIZE(zx297520v3_lspcrm_cells);

		pclk = devm_clk_get_enabled(dev, "pclk");
		if (IS_ERR(pclk))
			return dev_err_probe(dev, PTR_ERR(pclk), "Could not get 'pclk'\n");

		reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
		if (IS_ERR(reset))
			return dev_err_probe(dev, PTR_ERR(reset),
					     "Failed to deassert reset control\n");
		break;

	default:
		return -ENODEV;
	}

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cells, num_cells, NULL, 0, NULL);
}

static const struct of_device_id of_match_zx297520v3_crm[] = {
	{ .compatible = "zte,zx297520v3-topcrm", .data = (void *)TYPE_TOPCRM },
	{ .compatible = "zte,zx297520v3-matrixcrm", .data = (void *)TYPE_MATRIXCRM },
	{ .compatible = "zte,zx297520v3-lspcrm", .data = (void *)TYPE_LSPCRM },
	{ }
};
MODULE_DEVICE_TABLE(of, of_match_zx297520v3_crm);

static struct platform_driver zx297520v3_crm = {
	.probe = zx297520v3_crm_probe,
	.driver = {
		.name = "zx297520v3-crm",
		.of_match_table = of_match_zx297520v3_crm,
	},
};
module_platform_driver(zx297520v3_crm);

MODULE_AUTHOR("Stefan Dösinger <stefandoesinger@gmail.com>");
MODULE_DESCRIPTION("ZTE zx297520v3 CRM driver");
MODULE_LICENSE("GPL");
