/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <soc/imx8/sc/sci.h>

#include "../core.h"
#include "pinctrl-imx.h"

sc_ipc_t pinctrl_ipcHandle;

int imx_pmx_set_one_pin_scu(struct imx_pinctrl *ipctl, struct imx_pin *pin)
{
	return 0;
}

int imx_pinconf_backend_get_scu(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *config)
{
	sc_err_t err = SC_ERR_NONE;
	sc_ipc_t ipc = pinctrl_ipcHandle;

	if (ipc == -1) {
		printk("IPC handle not initialized!\n");
		return -EIO;
	}

	err = sc_pad_get(ipc, pin_id, (unsigned int *)config);

	if (err != SC_ERR_NONE)
		return -EIO;

	return 0;
}

int imx_pinconf_backend_set_scu(struct pinctrl_dev *pctldev, unsigned pin_id,
			    unsigned long *configs, unsigned num_configs)
{
	sc_err_t err = SC_ERR_NONE;
	sc_ipc_t ipc = pinctrl_ipcHandle;
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct imx_pinctrl_soc_info *info = ipctl->info;
	/*
	 * Mux should be done in pmx set, but we do not have a good api
	 * to handle that in scfw, so config it in pad conf func
	 */
	unsigned int mux = configs[0];
	unsigned int val = configs[1];

	if (ipc == -1) {
		printk("IPC handle not initialized!\n");
		return -EIO;
	}

	if (info->flags & IMX8_ENABLE_MUX_CONFIG)
		val |= BM_IMX8_IFMUX_ENABLE;

	if (info->flags & IMX8_ENABLE_PAD_CONFIG)
		val |= BM_IMX8_GP_ENABLE;

	if (info->flags & SHARE_MUX_CONF_REG) {
		val |= (mux << 27) & (0x7 << 27);
		err = sc_pad_set(ipc, pin_id, val);
	}

	if (err != SC_ERR_NONE)
		return -EIO;

	return 0;
}

int imx_pinctrl_parse_pin_scu(struct imx_pinctrl_soc_info *info,
			  unsigned int *pin_id, struct imx_pin *pin,
			  const __be32 **list_p, u32 generic_config)
{
	struct imx_pin_scu *pin_scu = &pin->pin_conf.pin_scu;

	pin->pin = be32_to_cpu(*((*list_p)++));
	*pin_id = pin->pin;
	pin_scu->mux = be32_to_cpu(*((*list_p)++));
	pin_scu->config = be32_to_cpu(*((*list_p)++));

	dev_dbg(info->dev, "%s: 0x%lx 0x%lx",
		 info->pins[pin->pin].name, pin_scu->mux, pin_scu->config);

	return 0;
}
