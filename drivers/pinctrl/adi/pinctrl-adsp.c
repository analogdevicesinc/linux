// SPDX-License-Identifier: GPL-2.0
/*
 * PORT pin controller and GPIO driver for ADI ADSP SoCs
 *
 * Copyright (c) 2026 Analog Devices Inc.
 */

#include <dt-bindings/pinctrl/adi,adsp-pinctrl.h>
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"
#include "../pinmux.h"

/* pinmux entry decoding */
#define ADSP_PINMUX_SOC(v) FIELD_GET(GENMASK(31, 20), v)
#define ADSP_PINMUX_PIN(v) FIELD_GET(GENMASK(9, 2), v)
#define ADSP_PINMUX_ALT(v) FIELD_GET(GENMASK(1, 0), v)

#define ADSP_PINS_PER_PORT 16

#define ADSP_PORT_FER		0x00
#define ADSP_PORT_FER_SET	0x04
#define ADSP_PORT_FER_CLEAR	0x08
#define ADSP_PORT_DATA		0x0c
#define ADSP_PORT_DATA_SET	0x10
#define ADSP_PORT_DATA_CLEAR	0x14
#define ADSP_PORT_DIR		0x18
#define ADSP_PORT_DIR_SET	0x1c
#define ADSP_PORT_DIR_CLEAR	0x20
#define ADSP_PORT_INEN		0x24
#define ADSP_PORT_INEN_SET	0x28
#define ADSP_PORT_INEN_CLEAR	0x2c
#define ADSP_PORT_MUX		0x30
#define ADSP_PORT_MUX_BITS	2
#define ADSP_PORT_MUX_MASK	GENMASK(ADSP_PORT_MUX_BITS - 1, 0)

#define ADSP_PADS_PORT_DS_BITS	3
#define ADSP_PADS_PORT_DS_MASK	GENMASK(ADSP_PADS_PORT_DS_BITS - 1, 0)
#define ADSP_PADS_PORT_DS_LOW	1
#define ADSP_PADS_PORT_DS_HIGH	2

#define ADSP_NO_PINT (~0)

struct adsp_pinctrl;

struct adsp_port {
	struct adsp_pinctrl *pc;
	struct device_node *np;
	struct gpio_chip gc;
	void __iomem *regs;
	spinlock_t mux_lock;
	unsigned int index;
	unsigned int ngpio;
	unsigned int pin_base;
	unsigned int pint_base_lower;
	unsigned int pint_base_upper;
};

struct adsp_pinctrl_info {
	unsigned int soc;
	unsigned int port_pue_reg;
	unsigned int port_pud_reg;
	unsigned int port_pde_reg;
	unsigned int port_ds_reg;
};

struct adsp_pinctrl {
	struct device *dev;
	const struct adsp_pinctrl_info *info;
	struct regmap *pads;
	struct adsp_port *ports;
	unsigned int nports;
	struct pinctrl_pin_desc *pins;
	const char **pin_names;
	unsigned int npins;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pctldesc;
};

enum adsp_pin_bias {
	ADSP_PIN_BIAS_UNKNOWN,
	ADSP_PIN_BIAS_DISABLE,
	ADSP_PIN_BIAS_PULL_UP,
	ADSP_PIN_BIAS_PULL_DOWN,
};

static const char *const adsp_pin_functions[] = { "alt0", "alt1", "alt2",
						  "alt3" };

static const struct adsp_pinctrl_info sc571_pinctrl_info = {
	.soc = 0x571,
	.port_pue_reg = 0x80,
	.port_pud_reg = 0xc0,
};

static const struct adsp_pinctrl_info sc573_pinctrl_info = {
	.soc = 0x573,
	.port_pue_reg = 0x80,
	.port_pud_reg = 0xc0,
};

static const struct adsp_pinctrl_info sc584_pinctrl_info = {
	.soc = 0x584,
};

static const struct adsp_pinctrl_info sc589_pinctrl_info = {
	.soc = 0x589,
};

static const struct adsp_pinctrl_info sc594_pinctrl_info = {
	.soc = 0x594,
	.port_pue_reg = 0x98,
	.port_pde_reg = 0xc4,
	.port_ds_reg = 0x0c,
};

static const struct adsp_pinctrl_info sc598_pinctrl_info = {
	.soc = 0x598,
	.port_pue_reg = 0x98,
	.port_pde_reg = 0xc4,
	.port_ds_reg = 0x0c,
};

static struct adsp_port *adsp_pinctrl_pin_to_port(struct adsp_pinctrl *pc,
						  unsigned int pin)
{
	return pc->pins[pin].drv_data;
}

static unsigned int adsp_pinctrl_pin_to_gpio(struct adsp_pinctrl *pc,
					     unsigned int pin)
{
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);

	return pin - port->pin_base;
}

static int adsp_gpio_child_to_parent_hwirq(struct gpio_chip *gc,
					   unsigned int child,
					   unsigned int type,
					   unsigned int *parent,
					   unsigned int *parent_type)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	if (child < 8 && port->pint_base_lower != ADSP_NO_PINT)
		*parent = child + port->pint_base_lower;
	else if (child < 16 && port->pint_base_upper != ADSP_NO_PINT)
		*parent = (child - 8) + port->pint_base_upper;
	else
		return -EINVAL;

	*parent_type = type;

	return 0;
}

static void adsp_gpio_init_valid_mask(struct gpio_chip *gc,
				     unsigned long *valid_mask,
				     unsigned int ngpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	if (port->pint_base_lower == ADSP_NO_PINT)
		bitmap_clear(valid_mask, 0, 8);

	if (port->pint_base_upper == ADSP_NO_PINT)
		bitmap_clear(valid_mask, 8, 8);
}

static int adsp_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	return !!(readl(port->regs + ADSP_PORT_DATA) & BIT(gpio));
}

static int adsp_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	if (val)
		writel(BIT(gpio), port->regs + ADSP_PORT_DATA_SET);
	else
		writel(BIT(gpio), port->regs + ADSP_PORT_DATA_CLEAR);

	return 0;
}
static int adsp_gpio_get_direction(struct gpio_chip *gc, unsigned int gpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);
	unsigned int val;

	val = readl(port->regs + ADSP_PORT_DIR);

	if (val & BIT(gpio))
		return GPIO_LINE_DIRECTION_OUT;
	else
		return GPIO_LINE_DIRECTION_IN;
}

static int adsp_gpio_direction_input(struct gpio_chip *gc, unsigned int gpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	writel(BIT(gpio), port->regs + ADSP_PORT_DIR_CLEAR);
	writel(BIT(gpio), port->regs + ADSP_PORT_INEN_SET);

	return 0;
}

static int adsp_gpio_direction_output(struct gpio_chip *gc, unsigned int gpio,
				      int val)
{
	struct adsp_port *port = gpiochip_get_data(gc);
	int ret;

	ret = adsp_gpio_set(gc, gpio, val);
	if (ret)
		return ret;

	writel(BIT(gpio), port->regs + ADSP_PORT_INEN_CLEAR);
	writel(BIT(gpio), port->regs + ADSP_PORT_DIR_SET);

	return 0;
}

static int adsp_pinconf_get_pin_bias(struct adsp_pinctrl *pc, unsigned int pin)
{
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	const struct adsp_pinctrl_info *info = pc->info;
	struct regmap *pads = pc->pads;
	unsigned int reg_offset = port->index * regmap_get_reg_stride(pads);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);
	unsigned int pue = 0;
	unsigned int pud = 0;
	unsigned int pde = 0;
	bool pullup = false;
	bool pulldn = false;
	unsigned int val;

	if (info->port_pue_reg)
		pue = info->port_pue_reg + reg_offset;
	if (info->port_pud_reg)
		pud = info->port_pud_reg + reg_offset;
	if (info->port_pde_reg)
		pde = info->port_pde_reg + reg_offset;

	if (!pue && !pud && !pde)
		return -ENOTSUPP;

	if (pue) {
		regmap_read(pads, pue, &val);
		pullup = !!(val & BIT(gpio));
	}

	if (pud) {
		/* PUD takes precedence over PUE when it is present */
		regmap_read(pads, pud, &val);
		pullup = !(val & BIT(gpio));
	}

	if (pde) {
		regmap_read(pads, pde, &val);
		pulldn = !!(val & BIT(gpio));
	}

	if (pullup && !pulldn)
		return PIN_CONFIG_BIAS_PULL_UP;
	else if (pulldn && !pullup)
		return PIN_CONFIG_BIAS_PULL_DOWN;
	else if (!pullup && !pulldn)
		return PIN_CONFIG_BIAS_DISABLE;

	/* We should never get here */
	return -EINVAL;
}

static int adsp_pinconf_set_pin_bias(struct adsp_pinctrl *pc, unsigned int pin,
				     enum pin_config_param bias)
{
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	const struct adsp_pinctrl_info *info = pc->info;
	struct regmap *pads = pc->pads;
	unsigned int reg_offset = port->index * regmap_get_reg_stride(pads);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);
	bool pullup = bias == PIN_CONFIG_BIAS_PULL_UP;
	bool pulldn = bias == PIN_CONFIG_BIAS_PULL_DOWN;
	unsigned int pue = 0;
	unsigned int pud = 0;
	unsigned int pde = 0;

	/*
	 * Not all SoCs in the family support setting the bin bias. The three
	 * cases are:
	 *
	 *  1. No support at all
	 *  2. Pull-up control only
	 *  3. Pull-up and pull-down control
	 *
	 * For pull-up control, sometimes it is split across two registers, PUE
	 * (enable) and PUD (disable), where PUD takes precedence. The driver
	 * assumes that SoC info register addresses are valid when they are
	 * nonzero to determine the level of control available. Accordingly, an
	 * error is only returned when a requested bias configuration cannot be
	 * provided by the hardware.
	 */
	if (info->port_pue_reg)
		pue = info->port_pue_reg + reg_offset;
	if (info->port_pud_reg)
		pud = info->port_pud_reg + reg_offset;
	if (info->port_pde_reg)
		pde = info->port_pde_reg + reg_offset;

	if (!pue && !pud && !pde)
		return -ENOTSUPP;

	/* Disable bias */
	if (!pullup) {
		if (pud)
			regmap_set_bits(pads, pud, BIT(gpio));

		if (pue)
			regmap_clear_bits(pads, pue, BIT(gpio));
	}

	if (!pulldn) {
		if (pde)
			regmap_clear_bits(pads, pde, BIT(gpio));
	}

	/* Enable bias */
	if (pullup) {
		if (pud)
			regmap_clear_bits(pads, pud, BIT(gpio));

		if (pue)
			regmap_set_bits(pads, pue, BIT(gpio));
		else
			return -ENOTSUPP;
	}

	if (pulldn) {
		if (pde)
			regmap_set_bits(pads, pde, BIT(gpio));
		else
			return -ENOTSUPP;
	}

	return 0;
}

static int adsp_pinconf_get_slew_rate(struct adsp_pinctrl *pc, unsigned int pin)
{
	const struct adsp_pinctrl_info *info = pc->info;
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);
	unsigned int half = gpio / 8;
	unsigned int reg_offset =
		(port->index * 2 + half) * regmap_get_reg_stride(pc->pads);
	unsigned int shift = (gpio % 8) * ADSP_PADS_PORT_DS_BITS;
	unsigned int reg;
	unsigned int val;

	if (!info->port_ds_reg)
		return -ENOTSUPP;

	reg = info->port_ds_reg + reg_offset;

	regmap_read(pc->pads, reg, &val);
	val = (val >> shift) & ADSP_PADS_PORT_DS_MASK;

	return val == ADSP_PADS_PORT_DS_HIGH ? 1 : 0;
}

static int adsp_pinconf_set_slew_rate(struct adsp_pinctrl *pc, unsigned int pin,
				      u32 arg)
{
	const struct adsp_pinctrl_info *info = pc->info;
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);
	unsigned int half = gpio / 8;
	unsigned int reg_offset =
		(port->index * 2 + half) * regmap_get_reg_stride(pc->pads);
	unsigned int shift = (gpio % 8) * ADSP_PADS_PORT_DS_BITS;
	unsigned int reg;
	unsigned int val;

	if (!info->port_ds_reg)
		return -ENOTSUPP;

	reg = info->port_ds_reg + reg_offset;

	if (arg > 1)
		return -EINVAL;

	/*
	 * The reference manuals refer to drive strength, with two permissible
	 * values:
	 *
	 * - 0b001: for operating frequency <= 62.5 MHz (slow)
	 * - 0b010: for operating frequency > 62.5 MHz  (fast)
	 *
	 * This is essentially slew rate. Just accept two values (0 or 1)
	 * corresponding to the slow or fast slew rate.
	 */
	val = arg ? ADSP_PADS_PORT_DS_HIGH : ADSP_PADS_PORT_DS_LOW;

	regmap_update_bits(pc->pads, reg, ADSP_PADS_PORT_DS_MASK << shift,
			   val << shift);

	return 0;
}

static int adsp_pinconf_pin_config_get(struct pinctrl_dev *pctldev,
				       unsigned int pin, unsigned long *config)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg = pinconf_to_config_argument(*config);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN: {
		int bias = adsp_pinconf_get_pin_bias(pc, pin);
		if (bias < 0)
			return bias;
		else if (bias != param)
			return -EINVAL;
		arg = 1;
		break;
	}
	case PIN_CONFIG_SLEW_RATE: {
		int rate = adsp_pinconf_get_slew_rate(pc, pin);
		if (rate < 0)
			return rate;
		arg = rate;
		break;
	}
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int adsp_pinconf_pin_config_set(struct pinctrl_dev *pctldev,
				       unsigned int pin, unsigned long *configs,
				       unsigned int num_configs)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	int i;
	int ret;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param =
			pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = adsp_pinconf_set_pin_bias(pc, pin, param);
			break;
		case PIN_CONFIG_SLEW_RATE:
			ret = adsp_pinconf_set_slew_rate(pc, pin, arg);
			break;
		default:
			return -ENOTSUPP;
		}

		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops adsp_confops = {
	.is_generic = true,
	.pin_config_get = adsp_pinconf_pin_config_get,
	.pin_config_set = adsp_pinconf_pin_config_set,
};

static int adsp_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			       unsigned int group)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, group);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, group);
	u32 shift = ADSP_PORT_MUX_BITS * gpio;
	u32 mask = ADSP_PORT_MUX_MASK << shift;
	u32 field = func << shift;
	u32 val;

	/* Set alternate mode mux value */
	scoped_guard(spinlock, &port->mux_lock)	{
		val = readl(port->regs + ADSP_PORT_MUX);
		val &= ~mask;
		val |= field;
		writel(val, port->regs + ADSP_PORT_MUX);
	}

	/* Enable alternate function on the pin */
	writel(BIT(gpio), port->regs + ADSP_PORT_FER_SET);

	return 0;
}

static int adsp_pinmux_gpio_request_enable(struct pinctrl_dev *pctldev,
					   struct pinctrl_gpio_range *range,
					   unsigned int pin)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);

	/* Disable alternate function on the pin */
	writel(BIT(gpio), port->regs + ADSP_PORT_FER_CLEAR);

	return 0;
}

static const struct pinmux_ops adsp_pmxops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = adsp_pinmux_set_mux,
	.gpio_request_enable = adsp_pinmux_gpio_request_enable,
	.strict = true,
};

static int adsp_pinctrl_dt_subnode_to_map(struct pinctrl_dev *pctldev,
					  struct device_node *np,
					  struct pinctrl_map **map,
					  unsigned int *reserved_maps,
					  unsigned int *num_maps)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned long *configs = NULL;
	unsigned int num_configs = 0;
	int num_mux;
	int i;
	int ret;

	/* If there's no muxing, defer to the generic helper (pinconf only) */
	if (of_property_present(np, "pins"))
		return pinconf_generic_dt_subnode_to_map(
			pctldev, np, map, reserved_maps, num_maps,
			PIN_MAP_TYPE_CONFIGS_PIN);

	num_mux = of_property_count_u32_elems(np, "pinmux");
	if (num_mux <= 0)
		return num_mux ?: -EINVAL;

	ret = pinconf_generic_parse_dt_config(np, pctldev, &configs,
					      &num_configs);
	if (ret)
		return ret;

	/*
	 * Reserve maps for each pin mux, and if pinconf settings exist, also
	 * reserve a config map for each muxed pin.
	 */
	ret = pinctrl_utils_reserve_map(pctldev, map, reserved_maps, num_maps,
					num_configs ? 2 * num_mux : num_mux);
	if (ret)
		goto out;

	for (i = 0; i < num_mux; i++) {
		const char *group;
		const char *func;
		unsigned int val;
		unsigned int pin;
		unsigned int alt;

		ret = of_property_read_u32_index(np, "pinmux", i, &val);
		if (ret)
			goto out;

		/* Check that the correct mux table is being employed */
		if (ADSP_PINMUX_SOC(val) != pc->info->soc) {
			ret = -EINVAL;
			goto out;
		}

		/* Decompose the macro into pin and alternate mode indices */
		pin = ADSP_PINMUX_PIN(val);
		alt = ADSP_PINMUX_ALT(val);

		if (pin >= pctldev->desc->npins ||
		    alt >= pinmux_generic_get_function_count(pctldev)) {
			ret = -EINVAL;
			goto out;
		}

		/* Get the group (single pin) and function (alt mode) strings */
		group = pinctrl_generic_get_group_name(pctldev, pin);
		func = pinmux_generic_get_function_name(pctldev, alt);

		/* Add the muxing map */
		ret = pinctrl_utils_add_map_mux(pctldev, map, reserved_maps,
						num_maps, group, func);
		if (ret)
			goto out;

		/* If there were configs, add their map too */
		if (num_configs) {
			ret = pinctrl_utils_add_map_configs(
				pctldev, map, reserved_maps, num_maps, group,
				configs, num_configs, PIN_MAP_TYPE_CONFIGS_PIN);
			if (ret)
				goto out;
		}
	}

out:
	kfree(configs);
	return ret;
}

static int adsp_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				       struct device_node *np,
				       struct pinctrl_map **map,
				       unsigned int *num_maps)
{
	unsigned int reserved_maps;
	int ret;

	*map = NULL;
	*num_maps = 0;
	reserved_maps = 0;

	if (of_get_child_count(np)) {
		for_each_child_of_node_scoped(np, child) {
			ret = adsp_pinctrl_dt_subnode_to_map(
				pctldev, child, map, &reserved_maps, num_maps);
			if (ret)
				goto out;
		}
	} else {
		ret = adsp_pinctrl_dt_subnode_to_map(pctldev, np, map,
						     &reserved_maps, num_maps);
		if (ret)
			goto out;
	}

 out:
	if (ret)
		pinctrl_utils_free_map(pctldev, *map, *num_maps);

	return ret;
}

static const struct pinctrl_ops adsp_pctlops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = adsp_pinctrl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static void adsp_gpio_irq_enable(struct irq_data *data)
{
	struct irq_data *parent = data->parent_data;

	if (parent && parent->chip && parent->chip->irq_enable)
		parent->chip->irq_enable(parent);
}

static void adsp_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct irq_data *parent = data->parent_data;

	if (parent && parent->chip && parent->chip->irq_mask)
		parent->chip->irq_mask(parent);

	gpiochip_disable_irq(gc, data->hwirq);
}

static void adsp_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct irq_data *parent = data->parent_data;

	gpiochip_enable_irq(gc, data->hwirq);

	if (parent && parent->chip && parent->chip->irq_unmask)
		parent->chip->irq_unmask(parent);
}

static void adsp_gpio_irq_ack(struct irq_data *data)
{
	struct irq_data *parent = data->parent_data;

	if (parent && parent->chip && parent->chip->irq_ack)
		parent->chip->irq_ack(parent);
}

static const struct irq_chip adsp_gpio_irq_chip = {
	.name = "adsp-gpio",
	.irq_enable = adsp_gpio_irq_enable,
	.irq_mask = adsp_gpio_irq_mask,
	.irq_unmask = adsp_gpio_irq_unmask,
	.irq_set_type = irq_chip_set_type_parent,
	.irq_ack = adsp_gpio_irq_ack,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_SET_TYPE_MASKED,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int adsp_pinctrl_register_port(struct adsp_port *port)
{
	struct device *dev = port->pc->dev;
	struct gpio_chip *gc = &port->gc;
	struct gpio_irq_chip *girq = &gc->irq;
	struct device_node *parent_np __free(device_node) = NULL;

	port->regs = devm_of_iomap(dev, port->np, 0, NULL);
	if (IS_ERR(port->regs))
		return dev_err_probe(dev, PTR_ERR(port->regs),
				     "%pOF: failed to map regs\n", port->np);

	/* Make the gpiochip */
	gc->label = devm_kasprintf(dev, GFP_KERNEL, "adsp-port%c",
				   'a' + port->index);
	if (!gc->label)
		return -ENOMEM;

	gc->parent = dev;
	gc->fwnode = of_fwnode_handle(port->np);
	gc->owner = THIS_MODULE;
	gc->request = gpiochip_generic_request;
	gc->free = gpiochip_generic_free;
	gc->get = adsp_gpio_get;
	gc->set = adsp_gpio_set;
	gc->get_direction = adsp_gpio_get_direction;
	gc->direction_input = adsp_gpio_direction_input;
	gc->direction_output = adsp_gpio_direction_output;
	gc->set_config = gpiochip_generic_config;
	gc->base = -1;
	gc->ngpio = port->ngpio;

	if (port->pint_base_lower == ADSP_NO_PINT &&
	    port->pint_base_upper == ADSP_NO_PINT)
		goto skip_irqchip;

	/* Make the GPIO irqchip */
	parent_np = of_irq_find_parent(port->np);
	if (!parent_np)
		return -EINVAL;

	/* ... unless the parent PINT is unavailable */
	if (!of_device_is_available(parent_np))
		goto skip_irqchip;

	girq->parent_domain = irq_find_host(parent_np);
	if (!girq->parent_domain)
		return -EPROBE_DEFER;

	gpio_irq_chip_set_chip(girq, &adsp_gpio_irq_chip);
	girq->fwnode = of_fwnode_handle(port->np);
	girq->child_to_parent_hwirq = adsp_gpio_child_to_parent_hwirq;
	girq->populate_parent_alloc_arg =
		gpiochip_populate_parent_fwspec_twocell;
	girq->handler = handle_bad_irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->init_valid_mask = adsp_gpio_init_valid_mask;

skip_irqchip:
	return devm_gpiochip_add_data(dev, gc, port);
}

static int adsp_pinctrl_register_ports(struct adsp_pinctrl *pc)
{
	unsigned int i;
	int ret;

	for (i = 0; i < pc->nports; i++) {
		ret = adsp_pinctrl_register_port(&pc->ports[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int adsp_pinctrl_add_groups_and_functions(struct adsp_pinctrl *pc)
{
	struct pinctrl_dev *pctldev = pc->pctldev;
	unsigned int i, f;
	int ret;

	/*
	 * Muxing is per-pin. Add a single-pin group per pin, and add the
	 * alternate pin functions without any associated pins. While not all
	 * pins actually support every alternate function, the DT header macros
	 * only expose valid combinations.
	 */

	for (i = 0; i < pc->npins; i++) {
		ret = pinctrl_generic_add_group(pctldev, pc->pins[i].name,
						&pc->pins[i].number, 1, NULL);
		if (ret < 0)
			return ret;
	}

	for (f = 0; f < ARRAY_SIZE(adsp_pin_functions); f++) {
		ret = pinmux_generic_add_function(pctldev, adsp_pin_functions[f],
						  pc->pin_names, pc->npins, NULL);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int adsp_pinctrl_register(struct adsp_pinctrl *pc)
{
	struct device *dev = pc->dev;
	int ret;

	pc->pctldesc.name = dev_name(dev);
	pc->pctldesc.pins = pc->pins;
	pc->pctldesc.npins = pc->npins;
	pc->pctldesc.pctlops = &adsp_pctlops;
	pc->pctldesc.pmxops = &adsp_pmxops;
	if (pc->pads)
		pc->pctldesc.confops = &adsp_confops;
	pc->pctldesc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &pc->pctldesc, pc,
					     &pc->pctldev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register pinctrl\n");

	return 0;
}

static int adsp_pinctrl_build_pins(struct adsp_pinctrl *pc)
{
	struct device *dev = pc->dev;
	int i;

	pc->pins = devm_kcalloc(dev, pc->npins, sizeof(*pc->pins), GFP_KERNEL);
	if (!pc->pins)
		return -ENOMEM;

	pc->pin_names = devm_kcalloc(dev, pc->npins, sizeof(*pc->pin_names),
				     GFP_KERNEL);
	if (!pc->pin_names)
		return -ENOMEM;

	for (i = 0; i < pc->npins; i++) {
		unsigned int port = i / ADSP_PINS_PER_PORT;
		unsigned int offset = i % ADSP_PINS_PER_PORT;

		pc->pins[i].name =
			devm_kasprintf(dev, GFP_KERNEL, "P%c_%02u", 'A' + port,
				       offset); /* PA_00, PB_12, ... */
		if (!pc->pins[i].name)
			return -ENOMEM;

		pc->pins[i].number = i;
		pc->pins[i].drv_data = &pc->ports[port];
		pc->pin_names[i] = pc->pins[i].name;
	}

	return 0;
}

static int adsp_pinctrl_parse_port_interrupts(struct device_node *np,
					      unsigned int *lower,
					      unsigned int *upper,
					      unsigned int ngpio)
{
	unsigned int ranges[6];
	int count;
	int half;

	/*
	 * PORT GPIO interrupts can be sensed by a PINT interrupt controller,
	 * which is specified as an interrupt-parent of each PORT. Within the
	 * chip fabric, each PINT is typically connected to between 1 and 2
	 * PORTs. A PINT controller has 32 interrupt lines. These interrupt
	 * lines are muxed to connected PORT GPIOs at an 8 bit granularity,
	 * which is to say that each byte of PINT's 32 bit interrupt domain will
	 * correspond contiguously to either the upper- or lower-half of a
	 * PORT's 16 bit GPIO pin space, depending on the configuration. The
	 * configuration is set within the PINT controller itself. In order to
	 * construct a hierarchical interrupt domain between PINT and PORT,
	 * the adi,interrupt-ranges property is parsed below to determine the
	 * mapping between GPIO line and PINT controller hwirq.
	 *
	 * A PORT may not be muxed into a connected PINT controller at all, in
	 * which case the property is absent and no GPIO interrupt controller
	 * should be registered.
	 *
	 * A PORT may also have only half of its pin space muxed to a PINT
	 * controller. Accordingly, either the lower- or upper-half has to be
	 * masked out during registration.
	 *
	 * A PORT may have the full pin space muxed to a PINT controller, but in
	 * a non-contiguous fashion with respect to the PINT's 32 bit interrupt
	 * domain. For example, the lower-half GPIOs may correspond to PINT
	 * hwirqs 0..7, while the upper-half correspond to 24..31.
	 *
	 * Or, in the more typical use-case, the PORT's 16 GPIOs correspond to a
	 * contiguous field of PINT hwirqs 0..15 or 16..31.
	 *
	 * These various configurations are expressed in the firmware by the
	 * aforementioned adi,interrupt-ranges property: an array of 3-tuples
	 * <port_base pint_base length>. The code below decodes the array and
	 * sets the return values *lower and *upper to the corresponding PINT
	 * hwirq base for the lower- and upper-half of the GPIO pin space. If a
	 * particular half is not mapped, ADSP_NO_PINT is set instead.
	 *
	 * Based on the discussion above, a number of restrictions are applied
	 * (explained below). But to begin with, it should be clear that any
	 * valid configuration must consist of either zero, one, or two
	 * 3-tuples.
	 */

	*lower = ADSP_NO_PINT;
	*upper = ADSP_NO_PINT;

	if (!of_property_present(np, "interrupt-controller"))
		return 0;

	count = of_property_read_variable_u32_array(np, "adi,interrupt-ranges",
						    ranges, 3, 6);
	if (count < 0 || count % 3)
		return -EINVAL;

	for (half = 0; half < count / 3; half++) {
		unsigned int port_base = ranges[0 + 3 * half];
		unsigned int pint_base = ranges[1 + 3 * half];
		unsigned int length = ranges[2 + 3 * half];

		/* Mapped PORT pins must start from the lower- or upper-half */
		if (port_base != 0 && port_base != 8)
			return -EINVAL;

		/* Mapping must respect PINT muxing granularity (8 bits) */
		if (pint_base != 0 && pint_base != 8 && pint_base != 16 &&
		    pint_base != 24)
			return -EINVAL;

		/* Either half or all of the PORT pins must be mapped */
		if (length != 8 && length != ngpio)
			return -EINVAL;

		/* If all pins are mapped ... */
		if (length == ngpio) {
			/* the mapping must start from the lower-half */
			if (port_base != 0)
				return -EINVAL;

			/* and it must fit the PINT hwirq space */
			if (pint_base != 0 && pint_base != 16)
				return -EINVAL;
		}

		/* The mapping is valid - store the pint_base for this half */
		if (port_base == 0) {
			*lower = pint_base;

			/* Full mapping? Set the upper-half too */
			if (length == 16)
				*upper = pint_base + 8;
		} else if (port_base == 8)
			*upper = pint_base;
	}

	return 0;
}

static int adsp_pinctrl_parse_port_gpio_ranges(struct device_node *np,
					       unsigned int *pin_base,
					       unsigned int *ngpio)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3, 0, &args);
	if (ret)
		return ret;
	of_node_put(args.np);

	*pin_base = args.args[1];
	*ngpio = args.args[2];

	if (!*ngpio || *ngpio > 16)
		return -EINVAL;

	if (*pin_base % ADSP_PINS_PER_PORT)
		return -EINVAL;

	return 0;
}

static int adsp_pinctrl_parse_ports(struct adsp_pinctrl *pc)
{
	struct device *dev = pc->dev;
	struct device_node *np = dev->of_node;
	int ret;

	/*
	 * For each PORT, compute its index (PORTA, PORTB, etc.) and count the
	 * number of available pins based on its gpio-ranges property. From
	 * that, derive the total number of pins to expose in the pin
	 * controller: last PORT's pin base + pin count.
	 */

	for_each_available_child_of_node_scoped(np, child) {
		struct adsp_port *port;
		unsigned int pin_base;
		unsigned int ngpio;
		unsigned int index;

		if (!of_property_present(child, "gpio-controller"))
			continue;

		ret = adsp_pinctrl_parse_port_gpio_ranges(child, &pin_base,
							  &ngpio);
		if (ret)
			return dev_err_probe(dev, ret,
					     "%pOFn: bad gpio-ranges\n", child);

		index = pin_base / ADSP_PINS_PER_PORT;
		if (index >= pc->nports || index > 'Z' - 'A')
			return dev_err_probe(dev, -EINVAL,
					     "%pOFn: port %u out of range\n",
					     child, index);

		port = &pc->ports[index];
		if (port->ngpio)
			return dev_err_probe(dev, -EINVAL,
					     "%pOFn: duplicate port %u\n",
					     child, index);

		ret = adsp_pinctrl_parse_port_interrupts(child,
							 &port->pint_base_lower,
							 &port->pint_base_upper,
							 ngpio);
		if (ret)
			return dev_err_probe(
				dev, -EINVAL,
				"%pOFn: bad adi,interrupt-ranges\n", child);

		port->index = index;
		port->ngpio = ngpio;
		port->pin_base = pin_base;
		port->pc = pc;
		port->np = of_node_get(child);
		spin_lock_init(&port->mux_lock);

		/* Update total number of pins */
		pc->npins = max(pc->npins, pin_base + ngpio);
	}

	return 0;
}

static unsigned int adsp_pinctrl_count_ports(struct device_node *np)
{
	unsigned int n = 0;

	for_each_available_child_of_node_scoped(np, child)
		if (of_property_present(child, "gpio-controller"))
			n++;

	return n;
}

static int adsp_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adsp_pinctrl *pc;
	int ret;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	dev_set_drvdata(dev, pc);
	pc->dev = dev;
	pc->info = device_get_match_data(dev);

	pc->pads = syscon_regmap_lookup_by_phandle(np, "adi,pads-syscon");
	if (IS_ERR(pc->pads)) {
		if (PTR_ERR(pc->pads) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		/*
		 * PADS is optional; without it, there will be no pinconf
		 * support. Some SoCs like SC58x offer no pinconf.
		 */
		pc->pads = NULL;
	}

	pc->nports = adsp_pinctrl_count_ports(np);
	if (!pc->nports)
		return dev_err_probe(dev, -EINVAL,
				     "missing gpio-controller child nodes\n");

	pc->ports =
		devm_kcalloc(dev, pc->nports, sizeof(*pc->ports), GFP_KERNEL);
	if (!pc->ports)
		return -ENOMEM;

	ret = adsp_pinctrl_parse_ports(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_build_pins(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_register(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_add_groups_and_functions(pc);
	if (ret)
		return ret;

	ret = pinctrl_enable(pc->pctldev);
	if (ret)
		return ret;

	ret = adsp_pinctrl_register_ports(pc);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id adsp_pinctrl_of_match[] = {
	{ .compatible = "adi,adsp-sc571-pinctrl", .data = &sc571_pinctrl_info },
	{ .compatible = "adi,adsp-sc573-pinctrl", .data = &sc573_pinctrl_info },
	{ .compatible = "adi,adsp-sc584-pinctrl", .data = &sc584_pinctrl_info },
	{ .compatible = "adi,adsp-sc589-pinctrl", .data = &sc589_pinctrl_info },
	{ .compatible = "adi,adsp-sc594-pinctrl", .data = &sc594_pinctrl_info },
	{ .compatible = "adi,adsp-sc598-pinctrl", .data = &sc598_pinctrl_info },
	{}
};
MODULE_DEVICE_TABLE(of, adsp_pinctrl_of_match);

static struct platform_driver adsp_pinctrl_driver = {
	.driver = {
		.name = "adsp-pinctrl",
		.of_match_table = adsp_pinctrl_of_match,
	},
	.probe = adsp_pinctrl_probe,
};
module_platform_driver(adsp_pinctrl_driver);

MODULE_AUTHOR("Alvin Šipraga <alvin.sipraga@analog.com>");
MODULE_DESCRIPTION("ADI ADSP PORT pinctrl/GPIO driver");
MODULE_LICENSE("GPL");
