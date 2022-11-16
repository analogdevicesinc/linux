// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI On-Chip Real Time Clock v2 Driver
 *  Supports SC58X
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Author: Sonic Zhang <sonic.zhang@analog.com>
 */

#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/of.h>
#include <linux/io.h>

struct adi_rtc {
	void __iomem *regs_base;
	int irq;
	struct rtc_device *rtc_dev;
	struct rtc_time rtc_alarm;
	u8 cal;
};

/* RTC register offset */
#define RTC_CLK		0
#define RTC_ALM		4
#define RTC_IEN		8
#define RTC_STAT	12
#define RTC_STPWTCH	16
#define RTC_INIT	24
#define RTC_INITSTAT	28

/* Bit values for the RTC_STAT register */
#define RTC_STAT_WRZONE		0x00000400	/* Write to RTC */
#define RTC_STAT_CLKFAIL	0x00000200	/* RTC Clock fail */
#define RTC_STAT_SWEXP		0x00000100	/* Stop Watch Expiry */
#define RTC_STAT_DAYALM		0x00000080	/* Day Alarm */
#define RTC_STAT_ALM		0x00000040	/* Alarm Flag */
#define RTC_STAT_DAY		0x00000020	/* Day event */
#define RTC_STAT_HOUR		0x00000010	/* Hour event */
#define RTC_STAT_MIN		0x00000008	/* Minute event */
#define RTC_STAT_SEC		0x00000004	/* Second event */
#define RTC_STAT_WRDONE		0x00000002	/* Write Done */
#define RTC_STAT_WRPEND		0x00000001	/* Write Pending */

/* Bit values for the RTC_IEN register */
#define RTC_IEN_EMUDIS		0x00000200	/* En intr in emulation mode */
#define RTC_IEN_CLKFAIL		0x00000100	/* RTC 1Hz clock fail intr */
#define RTC_IEN_SW		0x00000080	/* Stopwatch Intr Enable */
#define RTC_IEN_DAYALM		0x00000040	/* Day Alarm Intr Enable */
#define RTC_IEN_ALM		0x00000020	/* Alarm Intr Enable */
#define RTC_IEN_DAY		0x00000010	/* Days Intr Enable */
#define RTC_IEN_HOUR		0x00000008	/* Hours Intr Enable */
#define RTC_IEN_MIN		0x00000004	/* Minutes Intr Enable */
#define RTC_IEN_SEC		0x00000002	/* Seconds Intr Enable */
#define RTC_IEN_WRDONE		0x00000001	/* Reg Write Done intr enable*/

/* Bit values for the RTC_INIT register */
#define RTC_INIT_RDEN		0x00000020	/* Enable output bus */
#define RTC_INIT_PWDN		0x00000010	/* RTC Power Down */
#define RTC_INIT_CAL		0x0000000F	/* Time Calibration */

/* Bit values for the RTC_INITSTAT register */
#define RTC_INITSTAT_CAL	0x00000078	/* Calibration Status */
#define RTC_INITSTAT_PWDN	0x00000004	/* Status of Power Down */
#define RTC_INITSTAT_DAYALMPND	0x00000002	/* Day Alarm Pending */
#define RTC_INITSTAT_ALMPND	0x00000001	/* Alarm Pending */

/* Shift values for RTC_CLK and RTC_ALM registers */
#define DAY_BITS_OFF    17
#define HOUR_BITS_OFF   12
#define MIN_BITS_OFF    6
#define SEC_BITS_OFF    0

/* Helper functions to convert the common RTC notion of time
 * and the device internal notion that is encoded in 32bits.
 */
static inline u32 rtc_time_to_dev(unsigned long now)
{
	u32 sec  = (now % 60);
	u32 min  = (now % (60 * 60)) / 60;
	u32 hour = (now % (60 * 60 * 24)) / (60 * 60);
	u32 days = (now / (60 * 60 * 24));

	return (sec  << SEC_BITS_OFF) +
	       (min  << MIN_BITS_OFF) +
	       (hour << HOUR_BITS_OFF) +
	       (days << DAY_BITS_OFF);
}

static inline unsigned long rtc_dev_to_time(u32 rtc_dev)
{
	return (((rtc_dev >> SEC_BITS_OFF)  & 0x003F)) +
	       (((rtc_dev >> MIN_BITS_OFF)  & 0x003F) * 60) +
	       (((rtc_dev >> HOUR_BITS_OFF) & 0x001F) * 60 * 60) +
	       (((rtc_dev >> DAY_BITS_OFF)  & 0x7FFF) * 60 * 60 * 24);
}

static inline void rtc_dev_to_tm(u32 rtc_dev, struct rtc_time *tm)
{
	rtc_time64_to_tm(rtc_dev_to_time(rtc_dev), tm);
}

static DECLARE_COMPLETION(adi_rtc_wrdone);

static void adi_rtc_sync_pending(struct device *dev)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	while (readl(rtc->regs_base + RTC_STAT) & RTC_STAT_WRPEND)
		wait_for_completion_timeout(&adi_rtc_wrdone, HZ * 5);
}

/*
 * adi_rtc_reset - set RTC to sane/known state
 */
static void adi_rtc_reset(struct device *dev, u16 rtc_ien)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	adi_rtc_sync_pending(dev);
	writel(rtc_ien, rtc->regs_base + RTC_IEN);
	writel(0, rtc->regs_base + RTC_ALM);
	writel(0xFFFF, rtc->regs_base + RTC_STAT);
}

static irqreturn_t adi_rtc_interrupt(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct adi_rtc *rtc = dev_get_drvdata(dev);
	unsigned long events = 0;
	bool wrdone = false;
	u16 rtc_stat, rtc_stat_clear, rtc_ien, bits;

	rtc_stat = readl(rtc->regs_base + RTC_STAT);
	rtc_ien = readl(rtc->regs_base + RTC_IEN);
	rtc_stat_clear = 0;

	if (rtc_stat & RTC_STAT_WRDONE) {
		rtc_stat_clear |= RTC_STAT_WRDONE;
		wrdone = true;
		complete(&adi_rtc_wrdone);
	}

	bits = (RTC_STAT_ALM | RTC_STAT_DAYALM);
	if (rtc_ien & (RTC_IEN_ALM | RTC_IEN_DAYALM) && rtc_stat & bits) {
		rtc_stat_clear |= bits;
		events |= RTC_AF | RTC_IRQF;
	}

	if (rtc_ien & RTC_IEN_SEC && rtc_stat & RTC_STAT_SEC) {
		rtc_stat_clear |= RTC_STAT_SEC;
		events |= RTC_UF | RTC_IRQF;
	}

	if (events)
		rtc_update_irq(rtc->rtc_dev, 1, events);

	if (wrdone || events) {
		writel(rtc_stat_clear, rtc->regs_base + RTC_STAT);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static void adi_rtc_int_clear(struct adi_rtc *rtc)
{
	writel(readl(rtc->regs_base + RTC_IEN) &
		~(RTC_IEN_ALM | RTC_IEN_DAYALM),
		rtc->regs_base + RTC_IEN);
}

static void adi_rtc_int_set_alarm(struct adi_rtc *rtc)
{
	/* ADI RTC has different bits for whether the alarm is
	 * more than 24 hours away.
	 */
	if (rtc->rtc_alarm.tm_yday == -1) {
		writel(RTC_STAT_ALM, rtc->regs_base + RTC_STAT);
		writel(readl(rtc->regs_base + RTC_IEN) | RTC_IEN_ALM,
			rtc->regs_base + RTC_IEN);
	} else {
		writel(RTC_STAT_DAYALM, rtc->regs_base + RTC_STAT);
		writel(readl(rtc->regs_base + RTC_IEN) | RTC_IEN_DAYALM,
			rtc->regs_base + RTC_IEN);
	}
}

static int adi_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	if (enabled)
		adi_rtc_int_set_alarm(rtc);
	else
		adi_rtc_int_clear(rtc);

	return 0;
}

static int adi_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	rtc_dev_to_tm(readl(rtc->regs_base + RTC_CLK), tm);

	return 0;
}

static int adi_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);
	unsigned long now;

	now = rtc_tm_to_time64(tm);
	if (now != 0) {
		adi_rtc_sync_pending(dev);
		writel(rtc_time_to_dev(now), rtc->regs_base + RTC_CLK);
	}

	return 0;
}

static int adi_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	alrm->time = rtc->rtc_alarm;
	alrm->enabled = !!(readl(rtc->regs_base + RTC_IEN) &
				(RTC_IEN_ALM | RTC_IEN_DAYALM));

	return 0;
}

static int adi_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);
	unsigned long rtc_alarm;

	rtc_alarm = rtc_tm_to_time64(&alrm->time);
	rtc->rtc_alarm = alrm->time;

	adi_rtc_sync_pending(dev);
	writel(rtc_time_to_dev(rtc_alarm), rtc->regs_base + RTC_ALM);
	if (alrm->enabled)
		adi_rtc_int_set_alarm(rtc);

	return 0;
}

static int adi_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);
	u16 ien = readl(rtc->regs_base + RTC_IEN);

#define yesno(x) ((x) ? "yes" : "no")
	seq_printf(seq,
		"alarm_IRQ\t: %s\n"
		"wkalarm_IRQ\t: %s\n"
		"seconds_IRQ\t: %s\n",
		yesno(ien & RTC_IEN_ALM),
		yesno(ien & RTC_IEN_DAYALM),
		yesno(ien & RTC_IEN_SEC));
#undef yesno

	return 0;
}

static const struct rtc_class_ops adi_rtc_ops = {
	.read_time     = adi_rtc_read_time,
	.set_time      = adi_rtc_set_time,
	.read_alarm    = adi_rtc_read_alarm,
	.set_alarm     = adi_rtc_set_alarm,
	.proc          = adi_rtc_proc,
	.alarm_irq_enable = adi_rtc_alarm_irq_enable,
};

#ifdef CONFIG_OF
static const struct of_device_id adi_rtc_of_match[] = {
	{
		.compatible = "adi,rtc2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_rtc_of_match);
#endif

static int adi_rtc_probe(struct platform_device *pdev)
{
	struct adi_rtc *rtc;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	struct rtc_time tm;
	int ret;
	unsigned long timeout;
	u16 pwdn_stat, sec_stat;

	/* Allocate memory for RTC struct */
	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	/* Find and map resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR((void *)rtc->regs_base))
		return PTR_ERR((void *)rtc->regs_base);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		return -ENOENT;
	}

	if (node) {
		of_property_read_u8(node, "calibration", &rtc->cal);
		rtc->cal &= 0xF;
	}

	device_init_wakeup(dev, 1);

	writel((readl(rtc->regs_base + RTC_INIT) & ~RTC_INIT_PWDN) |
		rtc->cal, rtc->regs_base + RTC_INIT);

	timeout = jiffies + HZ;
	do {
		pwdn_stat = readl(rtc->regs_base + RTC_INITSTAT) & RTC_INITSTAT_PWDN;
	} while ((pwdn_stat == 1) && time_before(jiffies, timeout));
	if (time_after_eq(jiffies, timeout)) {
		dev_err(&pdev->dev, "Clear RTC PWDN timeout\n");
		return -EPERM;
	}

	timeout = jiffies + HZ;
	do {
		sec_stat = readl(rtc->regs_base + RTC_STAT) & RTC_STAT_SEC;
	} while ((sec_stat == 0) && time_before(jiffies, timeout));
	if (time_after_eq(jiffies, timeout)) {
		dev_err(&pdev->dev, "Wait RTC SEC timeout\n");
		return -EPERM;
	}

	rtc->rtc_dev = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc->rtc_dev))
		return PTR_ERR(rtc->rtc_dev);
	platform_set_drvdata(pdev, rtc);


	ret = devm_request_irq(dev, rtc->irq, adi_rtc_interrupt, 0,
				pdev->name, dev);
	if (ret)
		dev_err(&pdev->dev, "unable to request RTC IRQ\n");

	rtc->rtc_dev->ops = &adi_rtc_ops;

	ret = devm_rtc_register_device(rtc->rtc_dev);
	if (ret)
		return ret;

	ret = adi_rtc_read_time(dev, &tm);

	if ((tm.tm_year < 70) || (tm.tm_year > 138)) {
		tm.tm_year = 100;
		tm.tm_mon = 0;
		tm.tm_mday = 1;
		tm.tm_hour = 0;
		tm.tm_min = 0;
		tm.tm_sec = 0;

		ret = adi_rtc_set_time(dev, &tm);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to rset initial time.\n");
			goto out;
		}
	}
	adi_rtc_reset(dev, RTC_IEN_WRDONE);
	writel(0, rtc->regs_base + RTC_STPWTCH);

	return 0;
out:
	return ret;
}

static int adi_rtc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	adi_rtc_reset(dev, 0);

	return 0;
}

static int __maybe_unused adi_rtc_suspend(struct device *dev)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(rtc->irq);
		adi_rtc_sync_pending(dev);
	} else
		adi_rtc_int_clear(0);

	return 0;
}

static int __maybe_unused adi_rtc_resume(struct device *dev)
{
	struct adi_rtc *rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(rtc->irq);

	/*
	 * Wait till the RTC MMRs be synced into the core after waking up.
	 */
	while (!(readl(rtc->regs_base + RTC_IEN) & RTC_IEN_SEC))
		continue;
	writel(RTC_STAT_WRDONE, rtc->regs_base + RTC_STAT);
	writel(RTC_IEN_WRDONE, rtc->regs_base + RTC_IEN);

	return 0;
}

static SIMPLE_DEV_PM_OPS(adi_rtc_pm_ops, adi_rtc_suspend, adi_rtc_resume);

static struct platform_driver adi_rtc_driver = {
	.driver		= {
		.name	= "rtc-adi2",
		.pm	= &adi_rtc_pm_ops,
		.of_match_table = of_match_ptr(adi_rtc_of_match),
	},
	.probe		= adi_rtc_probe,
	.remove		= adi_rtc_remove,
};

module_platform_driver(adi_rtc_driver);

MODULE_DESCRIPTION("ADI On-Chip Real Time Clock v2 Driver");
MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_LICENSE("GPL");
