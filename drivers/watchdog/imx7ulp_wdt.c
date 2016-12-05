/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

#define WDOG_CS			0x0
#define WDOG_CS_CMD32EN		(1 << 13)
#define WDOG_CS_ULK		(1 << 11)
#define WDOG_CS_RCS		(1 << 10)
#define WDOG_CS_EN		(1 << 7)
#define WDOG_CS_UPDATE		(1 << 5)

#define WDOG_CNT	0x4
#define WDOG_TOVAL	0x8

#define REFRESH_SEQ0	0xA602
#define REFRESH_SEQ1	0xB480
#define REFRESH		((REFRESH_SEQ1 << 16) | (REFRESH_SEQ0))

#define UNLOCK_SEQ0	0xC520
#define UNLOCK_SEQ1	0xD928
#define UNLOCK		((UNLOCK_SEQ1 << 16) | (UNLOCK_SEQ0))

struct imx7ulp_wdt {
	void __iomem *base;
	int rate;
	struct watchdog_device wdd;
	struct notifier_block restart_handler;
};

static inline void imx7ulp_wdt_enable(void __iomem *base, bool enable)
{
	u32 val = readl(base + WDOG_CS);

	local_irq_disable();

	writel(UNLOCK, base + WDOG_CNT);
	if (enable)
		writel(val | WDOG_CS_EN, base + WDOG_CS);
	else
		writel(val & ~WDOG_CS_EN, base + WDOG_CS);

	local_irq_enable();
}

static inline bool imx7ulp_wdt_is_enabled(void __iomem *base)
{
	u32 val = readl(base + WDOG_CS);

	return val & WDOG_CS_EN;
}

static int imx7ulp_wdt_ping(struct watchdog_device *wdog)
{
	/* refresh the wdt counter to keepalive */
	struct imx7ulp_wdt *wdt = watchdog_get_drvdata(wdog);
	local_irq_disable();
	writel(REFRESH, wdt->base + WDOG_CNT);
	local_irq_enable();
	return 0;
}

static int imx7ulp_wdt_start(struct watchdog_device *wdog)
{
	struct imx7ulp_wdt *wdt = watchdog_get_drvdata(wdog);
	imx7ulp_wdt_enable(wdt->base, true);

	return 0;
}

static int imx7ulp_wdt_stop(struct watchdog_device *wdog)
{
	struct imx7ulp_wdt *wdt = watchdog_get_drvdata(wdog);
	imx7ulp_wdt_enable(wdt->base, false);

	return 0;
}

static int imx7ulp_wdt_set_timeout(struct watchdog_device *wdog, unsigned int timeout)
{
	struct imx7ulp_wdt *wdt = watchdog_get_drvdata(wdog);
	u32 val = wdt->rate * timeout;

	local_irq_disable();

	writel(UNLOCK, wdt->base + WDOG_CNT);
	writel(val, wdt->base + WDOG_TOVAL);

	local_irq_enable();

	wdog->timeout = timeout;

	imx7ulp_wdt_ping(wdog);

	return 0;
}

static const struct watchdog_ops imx7ulp_wdt_ops = {
	.owner = THIS_MODULE,
	.start = imx7ulp_wdt_start,
	.stop  = imx7ulp_wdt_stop,
	.ping  = imx7ulp_wdt_ping,
	.set_timeout = imx7ulp_wdt_set_timeout,
};

static const struct watchdog_info imx7ulp_wdt_info = {
	.identity	= "i.MX7ULP watchdog timer",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING
			 | WDIOF_MAGICCLOSE,
};

static int imx7ulp_wdt_restart_handler(struct notifier_block *this,
			 unsigned long action, void *data)
{
	struct imx7ulp_wdt *wdt = container_of(this, struct imx7ulp_wdt, restart_handler);

	local_irq_disable();

	imx7ulp_wdt_enable(wdt->base, true);
	imx7ulp_wdt_set_timeout(&wdt->wdd, 1);

	local_irq_enable();

	/* wait for wdog to fire */
	while(true)
		;

	return NOTIFY_DONE;
}

static inline void imx7ulp_wdt_init(void __iomem *base, unsigned int timeout)
{
	u32 val;

	local_irq_disable();

	/*
	 * if the wdog is in unlocked status, the UNLOCK
	 * sequence no need to be send.
	 */
	val = readl(base + WDOG_CS);
	if (!(val & WDOG_CS_ULK)) {
		writel(UNLOCK_SEQ0, base + WDOG_CNT);
		writel(UNLOCK_SEQ1, base + WDOG_CNT);
	}
	/*set an initial timeout value in TOVAL */
	writel(timeout, base + WDOG_TOVAL);
	/* enable 32bit command sequence and reconfigure */
	val = (1 << 13) | (1 << 8) | (1 << 5);
	writel(val, base + WDOG_CS);

	local_irq_enable();
}

static int imx7ulp_wdt_probe(struct platform_device *pdev)
{
	struct imx7ulp_wdt *wdt;
	struct resource *res;
	int err;
	u32 timeout;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	platform_set_drvdata(pdev, wdt);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	/* use the 1KHz LPO as the counter clock */
	wdt->rate = 1000;

	/* init the wdd */
	wdt->wdd.info = &imx7ulp_wdt_info;
	wdt->wdd.ops = &imx7ulp_wdt_ops;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = 60;
	wdt->wdd.parent = &pdev->dev;
	watchdog_set_drvdata(&wdt->wdd, wdt);
	/*
	 * set the timeout_parm to 0 to get the timeout
	 * from 'timeout-sec' property in dtb.
	 */
	err = watchdog_init_timeout(&wdt->wdd, 0, &pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to init the wdog timeout\n");
		return err;
	}

	timeout = wdt->wdd.timeout * wdt->rate;
	/* reconfigure the watchdog timer.*/
	imx7ulp_wdt_init(wdt->base, timeout);

	err = watchdog_register_device(&wdt->wdd);
	if (err) {
		dev_err(&pdev->dev, "Failed to register watchdog device\n");
		return err;
	}

	wdt->restart_handler.notifier_call = imx7ulp_wdt_restart_handler;
	wdt->restart_handler.priority = 128;
	err = register_restart_handler(&wdt->restart_handler);
	if (err) {
		dev_err(&pdev->dev, "cannot register restart handler\n");
		watchdog_unregister_device(&wdt->wdd);
		return err;
	}

	return 0;
}

static int imx7ulp_wdt_remove(struct platform_device *pdev)
{
	struct imx7ulp_wdt *wdt = platform_get_drvdata(pdev);

	imx7ulp_wdt_stop(&wdt->wdd);

	watchdog_unregister_device(&wdt->wdd);

	return 0;
}

static void imx7ulp_wdt_shutdown(struct platform_device *pdev)
{
	struct imx7ulp_wdt *wdt = platform_get_drvdata(pdev);

	if (watchdog_active(&wdt->wdd))
		imx7ulp_wdt_stop(&wdt->wdd);
}

#ifdef CONFIG_PM_SLEEP
/* Disable watchdog before suspend */
static int imx7ulp_wdt_suspend(struct device *dev)
{
	struct imx7ulp_wdt *wdt = dev_get_drvdata(dev);

	imx7ulp_wdt_enable(wdt->base, false);

	return 0;
}

static int imx7ulp_wdt_resume(struct device *dev)
{
	struct imx7ulp_wdt *wdt = dev_get_drvdata(dev);
	u32 timeout = wdt->wdd.timeout * wdt->rate;

	if (imx7ulp_wdt_is_enabled(wdt->base))
		imx7ulp_wdt_init(wdt->base, timeout);

	if (watchdog_active(&wdt->wdd))
		imx7ulp_wdt_enable(wdt->base, true);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(imx7ulp_wdt_pm_ops, imx7ulp_wdt_suspend,
			 imx7ulp_wdt_resume);

static const struct of_device_id imx7ulp_wdt_dt_ids[] = {
	{ .compatible = "fsl,imx7ulp-wdt", },
	{ /*sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx7ulp_wdt_dt_ids);

static struct platform_driver imx7ulp_wdt_driver = {
	.probe		= imx7ulp_wdt_probe,
	.remove		= imx7ulp_wdt_remove,
	.shutdown	= imx7ulp_wdt_shutdown,
	.driver		= {
		.name	= "imx7ulp-wdt",
		.pm	= &imx7ulp_wdt_pm_ops,
		.of_match_table = imx7ulp_wdt_dt_ids,
	},
};

module_platform_driver(imx7ulp_wdt_driver);

MODULE_AUTHOR("Bai Ping <ping.bai@nxp.com>");
MODULE_DESCRIPTION("Freescale i.MX7ULP watchdog driver");
MODULE_LICENSE("GPL v2");
