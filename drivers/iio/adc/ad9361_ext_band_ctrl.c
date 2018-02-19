/*
 * AD9361 Agile RF Transceiver
 *   Module for controlling external filter banks via GPIOs
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "ad9361.h"
#include <dt-bindings/iio/adc/adi,ad9361.h>

#define MAX_CTRL_GPIOS		256	/* Should be enough for a while */
#define MAX_CTRL_SETTINGS	512	/* Should be enough for a while;
					 * maybe make it configurable via DT (if needed)
					 */
/* Keep these names in static memory, because devm_gpiod_get() does not
 * copy the GPIO names when called. So, we get garbage GPIO names when
 * debugging.
 */
static char of_gpio_prop_names[MAX_CTRL_GPIOS][sizeof("adi,band-ctl-XXX")];

struct ad9361_ctrl_objs {
	struct gpio_desc *gpios[MAX_CTRL_GPIOS];
	int ngpios;
};

struct ad9361_band_setting {
	struct list_head list;

	const char *name;
	u64 freq_min;
	u64 freq_max;
	u32 *gpio_values;
	/* reference to the global objects to be controlled;
	 * to number of args in some calls
	 */
	struct ad9361_ctrl_objs *objs;
};

struct ad9361_ext_band_ctl {
	struct ad9361_band_setting	*tx_setting;
	struct ad9361_band_setting	*rx_setting;
	struct list_head		rx_settings;	/* RX Band settings */
	struct list_head		tx_settings;	/* TX Band settings */
	struct ad9361_ctrl_objs		objs;		/* Objects to control */
};

static int ad9361_apply_settings(struct ad9361_rf_phy *phy,
				 struct ad9361_band_setting *new,
				 struct ad9361_band_setting **curr);

static inline bool lctl_gpio_value_valid(int op)
{
	switch (op) {
	case AD9361_EXT_BAND_CTL_OP_LOW:	/* FALLTRHOUGH */
	case AD9361_EXT_BAND_CTL_OP_HIGH:	/* FALLTRHOUGH */
	case AD9361_EXT_BAND_CTL_OP_NOP:	/* FALLTRHOUGH */
	case AD9361_EXT_BAND_CTL_OP_INPUT:	/* FALLTRHOUGH */
		return true;
	default:
		return false;
	}
}

static int ad9361_populate_objs(struct device *dev,
				struct ad9361_ctrl_objs *objs)
{
	struct gpio_desc *desc;
	int ret, cnt;

	for (cnt = 0; cnt < ARRAY_SIZE(objs->gpios); cnt++) {
		desc = devm_gpiod_get(dev, of_gpio_prop_names[cnt], 0);
		if (IS_ERR(desc)) {
			ret = PTR_ERR(desc);
			if (ret == -ENOENT)
				break;
			dev_err(dev, "Error while getting gpio '%s': %d\n",
				of_gpio_prop_names[cnt], ret);
			return ret;
		}
		objs->gpios[cnt] = desc;
	}
	if (cnt == 0)
		return 0;

	objs->ngpios = cnt;

	return cnt;
}

static int ad9361_parse_gpio_settings(struct device *dev,
				      struct device_node *np,
				      struct ad9361_ext_band_ctl *ctl,
				      struct ad9361_band_setting *sett)
{
	int ret, i, pidx;
	int gpio_cnt;
	char pbuf[256];

	gpio_cnt = ctl->objs.ngpios;
	if (gpio_cnt == 0)
		return 0;

	if (!of_find_property(np, "adi,gpio-settings", NULL))
		return 0;

	sett->gpio_values = devm_kzalloc(dev,
					 sizeof(*(sett->gpio_values)) * gpio_cnt,
					 GFP_KERNEL);
	if (!sett->gpio_values)
		return -ENOMEM;

	ret = of_property_read_variable_u32_array(np, "adi,gpio-settings",
						  sett->gpio_values,
						  0, gpio_cnt);
	/* No GPIOs is a NOP */
	if (ret == 0) {
		devm_kfree(dev, sett->gpio_values);
		sett->gpio_values = NULL;
		return 0;
	}

	if (ret > 0 && (ret != gpio_cnt)) {
		dev_err(dev, "Expected %d GPIO settings ; got %d for '%s'\n",
			gpio_cnt, ret, np->name);
		ret = -EINVAL;
	}

	if (ret < 0) {
		dev_err(dev,
			"Error while parsing '%s: adi,gpio-settings': %d\n",
			np->name, ret);
		return ret;
	}

	pidx = 0;
	for (i = 0; i < gpio_cnt; i++) {
		if (!lctl_gpio_value_valid(sett->gpio_values[i])) {
			dev_err(dev,
				"Invalid setting (%u) for '%s:adi,gpio-settings[%d]'\n",
				sett->gpio_values[i], np->name, i);
			return -EINVAL;
		}
		pidx += snprintf(&pbuf[pidx], sizeof(pbuf) - pidx, "%u,",
				 sett->gpio_values[i]);
	}

	dev_dbg(dev, " * gpio settings: %s\n", pbuf);

	return 0;
}

static int ad9361_parse_setting(struct device *dev,
				struct device_node *np,
				struct ad9361_ext_band_ctl *ctl,
				struct ad9361_band_setting *sett)
{
	int ret;

	ret = ad9361_parse_gpio_settings(dev, np, ctl, sett);
	if (ret < 0)
		return ret;

	sett->objs = &ctl->objs;
	sett->name = np->name;

	return 0;
}

static int ad9361_parse_setting_with_freq_range(struct device *dev,
				   struct device_node *np,
				   struct ad9361_ext_band_ctl *ctl,
				   struct ad9361_band_setting *sett)
{
	int ret;

	ret = of_property_read_u64(np, "adi,lo-freq-min", &sett->freq_min);
	if (ret < 0) {
		dev_err(dev, "Error while parsing '%s:adi,lo-freq-min':%d\n",
			np->name, ret);
		return ret;
	}

	ret = of_property_read_u64(np, "adi,lo-freq-max", &sett->freq_max);
	if (ret < 0) {
		dev_err(dev, "Error while parsing '%s:adi,lo-freq-max':%d\n",
			np->name, ret);
		return ret;
	}

	dev_dbg(dev, " * frequency range %llu - %llu\n",
		 sett->freq_min, sett->freq_max);

	return ad9361_parse_setting(dev, np, ctl, sett);
}

static int ad9361_populate_settings(struct device *dev,
				    struct ad9361_ext_band_ctl *ctl,
				    const char *type,
				    struct list_head *lst)
{
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct ad9361_band_setting *new;
	char pnamebuf[64];
	int ret, cnt;

	INIT_LIST_HEAD(lst);

	if (!np)
		return 0;

	for (cnt = 0; cnt < MAX_CTRL_SETTINGS; cnt++) {
		snprintf(pnamebuf, sizeof(pnamebuf), "%s%d", type, cnt);
		child = of_get_child_by_name(np, pnamebuf);
		if (!child)
			break;

		new = devm_kzalloc(dev, sizeof(*new), GFP_KERNEL);
		if (!new)
			return -ENOMEM;

		dev_dbg(dev, "Found '%s'\n", child->name);
		ret = ad9361_parse_setting_with_freq_range(dev, child,
						ctl, new);
		if (ret < 0) {
			dev_err(dev, "Error while parsing '%s': %d\n",
				child->name, ret);
			return ret;
		}
		list_add_tail(&new->list, lst);
	}

	return cnt;
}

static void ad9361_init_of_gpio_names(void)
{
	int i;

	if (of_gpio_prop_names[0][0] != '\0')
		return;

	for (i = 0; i < ARRAY_SIZE(of_gpio_prop_names); i++) {
		snprintf(of_gpio_prop_names[i],
			 sizeof(of_gpio_prop_names[0]),
			 "adi,band-ctl-%d", i);
	}
}

int ad9361_register_ext_band_control(struct ad9361_rf_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	struct ad9361_ext_band_ctl *ctl;
	int ret;

	ctl = devm_kzalloc(dev, sizeof(*phy->ext_band_ctl), GFP_KERNEL);
	if (!ctl)
		return -ENOMEM;

	ad9361_init_of_gpio_names();

	ret = ad9361_populate_objs(dev, &ctl->objs);
	if (ret <= 0) {
		if (ret == 0)
			dev_info(dev, "No GPIOs defined for ext band ctrl\n");
		return ret;
	}

	ret = ad9361_populate_settings(dev, ctl, "adi_rx_band_setting_",
				       &ctl->rx_settings);
	if (ret < 0)
		return ret;

	ret = ad9361_populate_settings(dev, ctl, "adi_tx_band_setting_",
				       &ctl->tx_settings);
	if (ret < 0)
		return ret;

	phy->ext_band_ctl = ctl;

	return 0;
}

void ad9361_unregister_ext_band_control(struct ad9361_rf_phy *phy)
{
	/* Nothing to do yet */
}

static struct ad9361_band_setting *ad9361_find_first_setting(
		struct list_head *settings, u64 freq)
{
	struct ad9361_band_setting *sett;

	if (!settings)
		return NULL;

	list_for_each_entry(sett, settings, list) {
		if (sett->freq_min <= freq && freq < sett->freq_max)
			return sett;
	}

	return NULL;
}

static int ad9361_apply_gpio_settings(struct device *dev,
				      struct ad9361_band_setting *new,
				      struct ad9361_band_setting *curr)
{
	struct gpio_desc *gpio;
	int i, ret;

	if (!new->gpio_values || (new->objs->ngpios == 0))
		return 0; /* NOP */

	/* FIXME: try to use gpiod_set_array_value_complex() or similar asap
	 * to set GPIOs all at once. But with the current one it does not seem
	 * to be straightforward to switch between in/out-low/out-high.
	 */
	for (i = 0; i < new->objs->ngpios; i++) {
		/* If values are the same as previous setting, skip */
		if (curr && curr->gpio_values &&
		    new->gpio_values[i] == curr->gpio_values[i])
			continue;
		gpio = new->objs->gpios[i];
		switch (new->gpio_values[i]) {
		case AD9361_EXT_BAND_CTL_OP_INPUT:
			ret = gpiod_direction_input(gpio);
			break;
		case AD9361_EXT_BAND_CTL_OP_LOW:
			ret = gpiod_direction_output_raw(gpio, 0);
			break;
		case AD9361_EXT_BAND_CTL_OP_HIGH:
			ret = gpiod_direction_output_raw(gpio, 1);
			break;
		default:
			continue;
		}
		if (ret < 0) {
			dev_err(dev, "%s: err when setting GPIO(%d) val %d\n",
				__func__, i, ret);
			return ret;
		}
		dev_dbg(dev, "%s: GPIO(%d) set to %d\n", __func__, i,
			new->gpio_values[i]);
	}

	return 0;
}

static int ad9361_apply_settings(struct ad9361_rf_phy *phy,
				 struct ad9361_band_setting *new,
				 struct ad9361_band_setting **curr)
{
	struct ad9361_band_setting *lcurr;
	struct device *dev = &phy->spi->dev;
	int ret;

	if (curr) {
		lcurr = *curr;
	} else {
		lcurr = NULL;
		curr = &lcurr;
	}

	dev_dbg(dev, "%s: Applying setting '%s'\n", __func__,
		new->name);

	ret = ad9361_apply_gpio_settings(dev, new, lcurr);
	if (ret < 0)
		return ret;

	*curr = new;

	dev_dbg(dev, "%s: Applied setting '%s'\n", __func__,
		new->name);

	return 0;
}

int ad9361_adjust_rx_ext_band_settings(struct ad9361_rf_phy *phy, u64 freq)
{
	struct ad9361_band_setting *sett;
	struct ad9361_ext_band_ctl *ctl;
	int ret;

	if (!phy)
		return -EINVAL;

	if (!phy->ext_band_ctl)
		return 0;
	ctl = phy->ext_band_ctl;

	sett = ad9361_find_first_setting(&ctl->rx_settings, freq);
	if (!sett)
		return 0;

	/* Silently exit, if the same setting */
	if (ctl->rx_setting == sett)
		return 0;

	ret = ad9361_apply_settings(phy, sett, &ctl->rx_setting);
	if (ret < 0)
		return ret;

	return 0;
}

int ad9361_adjust_tx_ext_band_settings(struct ad9361_rf_phy *phy, u64 freq)
{
	struct ad9361_band_setting *sett;
	struct ad9361_ext_band_ctl *ctl;
	int ret;

	if (!phy)
		return -EINVAL;

	if (!phy->ext_band_ctl)
		return 0;
	ctl = phy->ext_band_ctl;

	sett = ad9361_find_first_setting(&ctl->tx_settings, freq);
	if (!sett)
		return 0;

	/* Silently exit, if the same setting */
	if (ctl->tx_setting == sett)
		return 0;

	ret = ad9361_apply_settings(phy, sett, &ctl->tx_setting);
	if (ret < 0)
		return ret;

	return 0;
}
