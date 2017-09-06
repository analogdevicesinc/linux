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

#include "../core.h"
#include "pinctrl-imx.h"

#define IMX_PAD_SION 0x40000000		/* set SION */

int imx_pmx_set_one_pin_mem(struct imx_pinctrl *ipctl, struct imx_pin *pin)
{
	const struct imx_pinctrl_soc_info *info = ipctl->info;
	unsigned int pin_id = pin->pin;
	struct imx_pin_reg *pin_reg;
	struct imx_pin_memmap *pin_memmap;
	pin_reg = &info->pin_regs[pin_id];
	pin_memmap = &pin->pin_conf.pin_memmap;

	if (pin_reg->mux_reg == -1) {
		dev_err(ipctl->dev, "Pin(%s) does not support mux function\n",
			info->pins[pin_id].name);
		return 0;
	}

	if (info->flags & SHARE_MUX_CONF_REG) {
		u32 reg;
		reg = readl(ipctl->base + pin_reg->mux_reg);
		reg &= ~info->mux_mask;
		reg |= (pin_memmap->mux_mode << info->mux_shift);
		writel(reg, ipctl->base + pin_reg->mux_reg);
		dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%x\n",
			pin_reg->mux_reg, reg);
	} else {
		writel(pin_memmap->mux_mode, ipctl->base + pin_reg->mux_reg);
		dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%x\n",
			pin_reg->mux_reg, pin_memmap->mux_mode);
	}

	/*
	 * If the select input value begins with 0xff, it's a quirky
	 * select input and the value should be interpreted as below.
	 *     31     23      15      7        0
	 *     | 0xff | shift | width | select |
	 * It's used to work around the problem that the select
	 * input for some pin is not implemented in the select
	 * input register but in some general purpose register.
	 * We encode the select input value, width and shift of
	 * the bit field into input_val cell of pin function ID
	 * in device tree, and then decode them here for setting
	 * up the select input bits in general purpose register.
	 */
	if (pin_memmap->input_val >> 24 == 0xff) {
		u32 val = pin_memmap->input_val;
		u8 select = val & 0xff;
		u8 width = (val >> 8) & 0xff;
		u8 shift = (val >> 16) & 0xff;
		u32 mask = ((1 << width) - 1) << shift;
		/*
		 * The input_reg[i] here is actually some IOMUXC general
		 * purpose register, not regular select input register.
		 */
		val = readl(ipctl->base + pin_memmap->input_reg);
		val &= ~mask;
		val |= select << shift;
		writel(val, ipctl->base + pin_memmap->input_reg);
	} else if (pin_memmap->input_reg) {
		/*
		 * Regular select input register can never be at offset
		 * 0, and we only print register value for regular case.
		 */
		if (ipctl->input_sel_base)
			writel(pin_memmap->input_val, ipctl->input_sel_base +
					pin_memmap->input_reg);
		else
			writel(pin_memmap->input_val, ipctl->base +
					pin_memmap->input_reg);
		dev_dbg(ipctl->dev,
			"==>select_input: offset 0x%x val 0x%x\n",
			pin_memmap->input_reg, pin_memmap->input_val);
	}

	return 0;
}

int imx_pinconf_backend_get_mem(struct pinctrl_dev *pctldev,
			    unsigned pin_id, unsigned long *config)
{
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	struct imx_pinctrl_soc_info *info = ipctl->info;
	const struct imx_pin_reg *pin_reg = &info->pin_regs[pin_id];

	if (pin_reg->conf_reg == -1) {
		dev_err(info->dev, "Pin(%s) does not support config function\n",
			info->pins[pin_id].name);
		return -EINVAL;
	}

	*config = readl(ipctl->base + pin_reg->conf_reg);

	if (info->flags & SHARE_MUX_CONF_REG)
		*config &= ~info->mux_mask;

	return 0;
}

int imx_pinconf_backend_set_mem(struct pinctrl_dev *pctldev,
			    unsigned pin_id, unsigned long *configs,
			    unsigned num_configs)
{
	struct imx_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	struct imx_pinctrl_soc_info *info = ipctl->info;
	const struct imx_pin_reg *pin_reg = &info->pin_regs[pin_id];
	int i;

	if (pin_reg->conf_reg == -1) {
		dev_err(info->dev, "Pin(%s) does not support config function\n",
			info->pins[pin_id].name);
		return -EINVAL;
	}

	dev_dbg(ipctl->dev, "pinconf set pin %s\n",
		info->pins[pin_id].name);

	for (i = 0; i < num_configs; i++) {
		if (info->flags & SHARE_MUX_CONF_REG) {
			u32 reg;
			reg = readl(ipctl->base + pin_reg->conf_reg);
			reg &= info->mux_mask;
			reg |= configs[i];
			writel(reg, ipctl->base + pin_reg->conf_reg);
			dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%x\n",
				pin_reg->conf_reg, reg);
		} else {
			writel(configs[i], ipctl->base + pin_reg->conf_reg);
			dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%lx\n",
				pin_reg->conf_reg, configs[i]);
		}
	} /* for each config */

	return 0;
}

int imx_pinctrl_parse_pin_mem(struct imx_pinctrl_soc_info *info,
			  unsigned int *grp_pin_id, struct imx_pin *pin,
			  const __be32 **list_p, u32 generic_config)
{
	struct imx_pin_memmap *pin_memmap = &pin->pin_conf.pin_memmap;
	u32 mux_reg = be32_to_cpu(*((*list_p)++));
	u32 conf_reg;
	u32 config;
	unsigned int pin_id;
	struct imx_pin_reg *pin_reg;

	if (info->flags & SHARE_MUX_CONF_REG) {
		conf_reg = mux_reg;
	} else {
		conf_reg = be32_to_cpu(*((*list_p)++));
		if (!conf_reg)
			conf_reg = -1;
	}

	pin_id = (mux_reg != -1) ? mux_reg / 4 : conf_reg / 4;
	pin_reg = &info->pin_regs[pin_id];
	pin->pin = pin_id;
	*grp_pin_id = pin_id;
	pin_reg->mux_reg = mux_reg;
	pin_reg->conf_reg = conf_reg;
	pin_memmap->input_reg = be32_to_cpu(*((*list_p)++));
	pin_memmap->mux_mode = be32_to_cpu(*((*list_p)++));
	pin_memmap->input_val = be32_to_cpu((*(*list_p)++));

	if (info->generic_pinconf) {
		/* generic pin config decoded */
		pin_memmap->config = generic_config;
	} else {
		/* legacy pin config read from devicetree */
		config = be32_to_cpu(*((*list_p)++));

		/* SION bit is in mux register */
		if (config & IMX_PAD_SION)
			pin_memmap->mux_mode |= IOMUXC_CONFIG_SION;
		pin_memmap->config = config & ~IMX_PAD_SION;
	}

	dev_dbg(info->dev, "%s: 0x%x 0x%08lx", info->pins[pin_id].name,
			pin_memmap->mux_mode, pin_memmap->config);

	return 0;
}
