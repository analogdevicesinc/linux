// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI On-Chip Watchdog Timer Driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define WATCHDOG_NAME "adi-wdt"

#define	WDOG_CTL   0x0   /* Watchdog Control Register */
#define	WDOG_CNT   0x4   /* Watchdog Count Register */
#define	WDOG_STAT  0x8   /* Watchdog Status Register */

/* Bit in SWRST that indicates boot caused by watchdog */
#define SWRST_RESET_WDOG 0x4000

/* Bit in WDOG_CTL that indicates watchdog has expired (WDR0) */
#define WDOG_EXPIRED 0x8000

/* Masks for WDEV field in WDOG_CTL register */
#define ICTL_RESET   0x0
#define ICTL_NMI     0x2
#define ICTL_GPI     0x4
#define ICTL_NONE    0x6
#define ICTL_MASK    0x6

/* Masks for WDEN field in WDOG_CTL register */
#define WDEN_MASK    0x0FF0
#define WDEN_ENABLE  0x0000
#define WDEN_DISABLE 0x0AD0

/* some defaults */
#define WATCHDOG_TIMEOUT 20

static unsigned int timeout = WATCHDOG_TIMEOUT;
static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout,
	"Watchdog timeout in seconds. (1<=timeout<=((2^32)/SCLK), default="
		__MODULE_STRING(WATCHDOG_TIMEOUT) ")");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
		__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct adi_wdt {
	struct watchdog_device wdd;
	void __iomem *base;
	spinlock_t lock;
	unsigned long rate;
	unsigned long timeout;
	bool nowayout;
	int state;
	unsigned int irq;
};

/**
 *	adi_wdt_keepalive - Keep the Userspace Watchdog Alive
 *
 *	The Userspace watchdog got a KeepAlive: schedule the next timeout.
 */
static int adi_wdt_keepalive(struct watchdog_device *wdd)
{
	struct adi_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	writel(wdt->timeout, wdt->base + WDOG_STAT);
	spin_unlock(&wdt->lock);

	return 0;
}

static int adi_wdt_stop(struct watchdog_device *wdd)
{
	struct adi_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	writel(WDEN_DISABLE, wdt->base + WDOG_CTL);
	spin_unlock(&wdt->lock);

	return 0;
}

static int adi_wdt_start(struct watchdog_device *wdd)
{
	struct adi_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	writel(WDEN_ENABLE | ICTL_RESET, wdt->base + WDOG_CTL);
	spin_unlock(&wdt->lock);

	return 0;
}

static int adi_wdt_running(struct watchdog_device *wdd)
{
	struct adi_wdt *wdt = watchdog_get_drvdata(wdd);

	return ((readl(wdt->base + WDOG_CTL) & WDEN_MASK) != WDEN_DISABLE);
}

static int adi_wdt_set_timeout(struct watchdog_device *wdd, unsigned int t)
{
	u32 cnt;
	int run;
	struct adi_wdt *wdt = watchdog_get_drvdata(wdd);

	cnt = t * wdt->rate;

	run = adi_wdt_running(wdd);

	adi_wdt_stop(wdd);
	writel(cnt, wdt->base + WDOG_CNT);
	if (run)
		adi_wdt_start(wdd);

	wdd->timeout = t;

	return 0;
}

#ifdef CONFIG_PM

static int adi_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct adi_wdt *wdt = platform_get_drvdata(pdev);

	wdt->state = adi_wdt_running(&wdt->wdd);
	adi_wdt_stop(&wdt->wdd);

	return 0;
}

static int adi_wdt_resume(struct platform_device *pdev)
{
	struct adi_wdt *wdt = platform_get_drvdata(pdev);

	if (wdt->state) {
		adi_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);
		adi_wdt_start(&wdt->wdd);
	}

	return 0;
}
#else
# define adi_wdt_suspend NULL
# define adi_wdt_resume NULL
#endif

static const struct watchdog_info adi_wdt_info = {
	.identity = "adi Watchdog",
	.options  = WDIOF_SETTIMEOUT |
		WDIOF_KEEPALIVEPING |
		WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops adi_wdt_ops = {
	.owner = THIS_MODULE,
	.start = adi_wdt_start,
	.stop = adi_wdt_stop,
	.ping = adi_wdt_keepalive,
	.set_timeout = adi_wdt_set_timeout,
};

static int adi_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct adi_wdt *wdt;
	struct resource *res;
	unsigned long sclk;
	struct clk *clk;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	wdt->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	clk = devm_clk_get(&pdev->dev, "adi-watchdog");
	if (IS_ERR(clk))
		return -ENODEV;

	sclk   = clk_get_rate(clk);
	spin_lock_init(&wdt->lock);
	wdt->wdd.parent = &pdev->dev;
	wdt->wdd.info = &adi_wdt_info;
	wdt->wdd.ops = &adi_wdt_ops;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = ~0UL / sclk;
	wdt->rate = sclk;

	watchdog_set_nowayout(&wdt->wdd, nowayout);

	ret = devm_watchdog_register_device(&pdev->dev, &wdt->wdd);
	if (ret) {
		pr_info("cannot register watchdog (%d)\n", ret);
		return ret;
	}

	watchdog_set_drvdata(&wdt->wdd, wdt);
	platform_set_drvdata(pdev, wdt);
	if (watchdog_init_timeout(&wdt->wdd, 0, &pdev->dev)) {
		if (watchdog_init_timeout(&wdt->wdd, timeout, &pdev->dev))
			return -EINVAL;

		adi_wdt_set_timeout(&wdt->wdd, timeout);
	} else
		adi_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);

	pr_info("initialized: timeout=%d sec (nowayout=%d)\n",
		timeout, nowayout);

	return 0;
}

static void adi_wdt_shutdown(struct platform_device *pdev)
{
	struct adi_wdt *wdt = platform_get_drvdata(pdev);

	adi_wdt_stop(&wdt->wdd);
}

#if defined(CONFIG_OF)
static const struct of_device_id adi_wdt_dt_ids[] = {
	{ .compatible = "adi,watchdog" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, adi_wdt_dt_ids);
#endif

static struct platform_driver adi_wdt_driver = {
	.probe     = adi_wdt_probe,
	.shutdown  = adi_wdt_shutdown,
	.suspend   = adi_wdt_suspend,
	.resume    = adi_wdt_resume,
	.driver    = {
		.name  = WATCHDOG_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(adi_wdt_dt_ids),
#endif
	},
};

module_platform_driver(adi_wdt_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("adi Watchdog Device Driver");
