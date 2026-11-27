// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Reset driver for the StarFive JHB100 SoC
 *
 * Copyright (C) 2024 StarFive Technology Co., Ltd.
 *
 * Author: Changhuang Liang <changhuang.liang@starfivetech.com>
 */

#include <dt-bindings/reset/starfive,jhb100-crg.h>
#include <linux/auxiliary_bus.h>
#include <soc/starfive/reset-starfive-jh71x0.h>

#include "reset-starfive-jh71x0.h"

static const unsigned int jhb100_sys0_map[] = {
	[JHB100_SYS0RST_RESOURCE_ARB]			= 0,
	[JHB100_SYS0RST_SYS0_IOMUX_PRESETN]		= 3,
	[JHB100_SYS0RST_SYS0H_IOMUX_PRESETN]		= 4,
	[JHB100_SYS0RST_RST_ADAPTOR_TIMEOUT_RSTN]	= 5,
	[JHB100_SYS0RST_BMCPCIERP_RSTN_BUS]		= 14,
	[JHB100_SYS0RST_BMCPCIERP_RSTN_CRG]		= 15,
	[JHB100_SYS0RST_HOSTSS0_RSTN_BUS_ESPI]		= 16,
	[JHB100_SYS0RST_HOSTSS0_RSTN_BUS_PCIE]		= 17,
	[JHB100_SYS0RST_HOSTSS0_RSTN_CRG]		= 18,
	[JHB100_SYS0RST_BMCPERIPH2_RSTN_CRG]		= 19,
	[JHB100_SYS0RST_BMCPERIPH2_RSTN_BUS]		= 20,
	[JHB100_SYS0RST_VCE_RSTN_CRG]			= 21,
	[JHB100_SYS0RST_VCE_RSTN_BUS]			= 22,
	[JHB100_SYS0RST_BMCUSB_RSTN_BUS]		= 23,
	[JHB100_SYS0RST_BMCUSB_RSTN_CRG]		= 24,
};

static const struct jh71x0_reset_info jhb100_sys0_info = {
	.nr_resets = ARRAY_SIZE(jhb100_sys0_map),
	.assert_offset = 0x12c,
	.status_offset = 0x130,
	.map = jhb100_sys0_map,
};

static int jhb100_reset_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	const struct jh71x0_reset_info *info =
		(const struct jh71x0_reset_info *)(id->driver_data);
	struct jh71x0_reset_platform_data *pdata = dev_get_platdata(&adev->dev);

	if (!info || !pdata || !pdata->base)
		return -ENODEV;

	return reset_starfive_jh71x0_register_with_info(&adev->dev, adev->dev.parent->of_node,
							pdata->base + info->assert_offset,
							pdata->base + info->status_offset,
							NULL, info, NULL);
}

static const struct auxiliary_device_id jhb100_reset_ids[] = {
	{
		.name = "clk_starfive_jh71x0.jhb100-r-sys0",
		.driver_data = (kernel_ulong_t)&jhb100_sys0_info,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(auxiliary, jhb100_reset_ids);

static struct auxiliary_driver jhb100_reset_driver = {
	.probe		= jhb100_reset_probe,
	.id_table	= jhb100_reset_ids,
};
module_auxiliary_driver(jhb100_reset_driver);

MODULE_AUTHOR("Changhuang Liang <changhuang.liang@starfivetech.com>");
MODULE_DESCRIPTION("StarFive JHB100 reset driver");
MODULE_LICENSE("GPL");
