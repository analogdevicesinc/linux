// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for watchdog device controlled through GPIO-line
 *
 * Author: 2013, Alexander Shiyan <shc_work@mail.ru>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/stringify.h>
#include <linux/watchdog.h>

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__stringify(WATCHDOG_NOWAYOUT) ")");

#define SOFT_TIMEOUT_MIN	1
#define SOFT_TIMEOUT_DEF	60

enum {
	HW_ALGO_TOGGLE,
	HW_ALGO_LEVEL,
};

struct gpio_wdt_priv {
	struct gpio_desc	*gpiod;
	struct gpio_desc	*reset_from_wdt;
	bool			state;
	bool			always_running;
	unsigned int		hw_algo;
	struct watchdog_device	wdd;
};

static void gpio_wdt_disable(struct gpio_wdt_priv *priv)
{
	/* Eternal ping */
	gpiod_set_value_cansleep(priv->gpiod, 1);

	/* Put GPIO back to tristate */
	if (priv->hw_algo == HW_ALGO_TOGGLE)
		gpiod_direction_input(priv->gpiod);
}

static int gpio_wdt_ping(struct watchdog_device *wdd)
{
	struct gpio_wdt_priv *priv = watchdog_get_drvdata(wdd);

	switch (priv->hw_algo) {
	case HW_ALGO_TOGGLE:
		/* Toggle output pin */
		priv->state = !priv->state;
		gpiod_set_value_cansleep(priv->gpiod, priv->state);
		break;
	case HW_ALGO_LEVEL:
		/* Pulse */
		gpiod_set_value_cansleep(priv->gpiod, 1);
		udelay(1);
		gpiod_set_value_cansleep(priv->gpiod, 0);
		break;
	}
	return 0;
}

static int gpio_wdt_start(struct watchdog_device *wdd)
{
	struct gpio_wdt_priv *priv = watchdog_get_drvdata(wdd);

	priv->state = 0;
	gpiod_direction_output(priv->gpiod, priv->state);

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return gpio_wdt_ping(wdd);
}

static int gpio_wdt_stop(struct watchdog_device *wdd)
{
	struct gpio_wdt_priv *priv = watchdog_get_drvdata(wdd);

	if (!priv->always_running) {
		gpio_wdt_disable(priv);
	} else {
		set_bit(WDOG_HW_RUNNING, &wdd->status);
	}

	return 0;
}

static const struct watchdog_info gpio_wdt_ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING |
			  WDIOF_SETTIMEOUT,
	.identity	= "GPIO Watchdog",
};

static const struct watchdog_ops gpio_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= gpio_wdt_start,
	.stop		= gpio_wdt_stop,
	.ping		= gpio_wdt_ping,
};

static int gpio_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_wdt_priv *priv;
	enum gpiod_flags gflags;
	unsigned int hw_margin;
	const char *algo;
	int ret;

	priv = devm_kzalloc_obj(dev, *priv);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	ret = device_property_read_string(dev, "hw_algo", &algo);
	if (ret)
		return ret;
	if (!strcmp(algo, "toggle")) {
		priv->hw_algo = HW_ALGO_TOGGLE;
		gflags = GPIOD_IN;
	} else if (!strcmp(algo, "level")) {
		priv->hw_algo = HW_ALGO_LEVEL;
		gflags = GPIOD_OUT_LOW;
	} else {
		return -EINVAL;
	}

	priv->gpiod = devm_gpiod_get(dev, NULL, gflags);
	if (IS_ERR(priv->gpiod))
		return PTR_ERR(priv->gpiod);

	priv->reset_from_wdt = devm_gpiod_get_optional(dev, "reset-from-wdt", GPIOD_IN);
	if (IS_ERR(priv->reset_from_wdt))
		return dev_err_probe(dev, PTR_ERR(priv->reset_from_wdt),
				     "failed to get reset-from-wdt-gpios.");

	if (priv->reset_from_wdt) {
		int val = gpiod_get_value_cansleep(priv->reset_from_wdt);

		if (val < 0)
			return val;

		priv->wdd.bootstatus = val ? WDIOF_CARDRESET : 0;
	}

	ret = device_property_read_u32(dev, "hw_margin_ms", &hw_margin);
	if (ret)
		return ret;
	/* The core feeds the watchdog at half the hardware margin. */
	if (hw_margin < 2)
		return -EINVAL;

	if (device_property_present(dev, "hw-min-interval-ms")) {
		ret = device_property_read_u32(dev, "hw-min-interval-ms",
					       &priv->wdd.min_hw_heartbeat_ms);
		if (ret)
			return ret;
		if (!priv->wdd.min_hw_heartbeat_ms ||
		    priv->wdd.min_hw_heartbeat_ms >= hw_margin)
			return -EINVAL;
	}

	priv->always_running = device_property_read_bool(dev, "always-running");

	watchdog_set_drvdata(&priv->wdd, priv);

	priv->wdd.info		= &gpio_wdt_ident;
	priv->wdd.ops		= &gpio_wdt_ops;
	priv->wdd.min_timeout	= SOFT_TIMEOUT_MIN;
	priv->wdd.max_hw_heartbeat_ms = hw_margin;
	priv->wdd.parent	= dev;
	priv->wdd.timeout	= SOFT_TIMEOUT_DEF;

	watchdog_init_timeout(&priv->wdd, 0, dev);
	watchdog_set_nowayout(&priv->wdd, nowayout);

	watchdog_stop_on_reboot(&priv->wdd);

	if (priv->always_running)
		gpio_wdt_start(&priv->wdd);

	return devm_watchdog_register_device(dev, &priv->wdd);
}

static const struct of_device_id gpio_wdt_dt_ids[] = {
	{ .compatible = "linux,wdt-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_wdt_dt_ids);

static struct platform_driver gpio_wdt_driver = {
	.driver	= {
		.name		= "gpio-wdt",
		.of_match_table	= gpio_wdt_dt_ids,
	},
	.probe	= gpio_wdt_probe,
};

#ifdef CONFIG_GPIO_WATCHDOG_ARCH_INITCALL
static int __init gpio_wdt_init(void)
{
	return platform_driver_register(&gpio_wdt_driver);
}
arch_initcall(gpio_wdt_init);
#else
module_platform_driver(gpio_wdt_driver);
#endif

MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("GPIO Watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-wdt");
