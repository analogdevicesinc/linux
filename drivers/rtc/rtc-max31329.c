// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/bcd.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#define MAX31329_STATUS_REG		0x00
#define MAX31329_INT_EN_REG		0x01
#define MAX31329_RTC_RESET_REG		0x02
#define MAX31329_RTC_CONFIG1_REG	0x03
#define MAX31329_RTC_CONFIG2_REG	0x04
#define MAX31329_SECONDS_REG		0x06
#define MAX31329_MONTH_REG		0x0B
#define MAX31329_ALM1_SEC_REG		0x0D
#define MAX31329_ALM1_DAY_DATE_REG	0x10

#define MAX31329_STATUS_REG_A1F			BIT(0)
#define MAX31329_STATUS_REG_A2F			BIT(1)
#define MAX31329_STATUS_REG_OSF			BIT(6)

#define MAX31329_INT_EN_REG_A1IE		BIT(0)
#define MAX31329_INT_EN_REG_A2IE		BIT(1)

#define MAX31329_RTC_RESET_REG_SWRST		BIT(0)

#define MAX31329_CONFIG1_REG_ENOSC		BIT(0)
#define MAX31329_CONFIG1_REG_I2C_TIMEOUT	BIT(1)
#define MAX31329_CONFIG1_REG_DATA_RET		BIT(2)
#define MAX31329_CONFIG1_REG_ENIO		BIT(3)

#define MAX31329_CONFIG2_REG_ENCLKIN		BIT(2)
#define MAX31329_CONFIG2_REG_ENCLKO		BIT(7)

#define MAX31329_HOURS_REG_AM_PM		BIT(5)
#define MAX31329_HOURS_REG_F_24_12		BIT(6)

#define MAX31329_MONTH_REG_CENTURY		BIT(7)

enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	RTC_NR_TIME
};

enum {
	RTC_ALM_SEC = 0,
	RTC_ALM_MIN,
	RTC_ALM_HOUR,
	RTC_ALM_DATE,
	RTC_ALM_MONTH,
	RTC_ALM_YEAR,
	RTC_NR_ALM_TIME
};

struct max31329_rtc_info {
	struct device *dev;
	struct rtc_device    *rtc;
	struct regmap        *map;
	struct work_struct    work;
	int irq;
};

static int max31329_check_rtc_status(struct max31329_rtc_info *info)
{
	unsigned int stat, config;
	int ret;

	ret = regmap_read(info->map, MAX31329_STATUS_REG, &stat);
	if (ret)
		return ret;

	if (stat & MAX31329_STATUS_REG_OSF)
		dev_warn(info->dev, "oscillator discontinuity flagged, time unreliable\n");

	stat &= ~(MAX31329_STATUS_REG_A1F | MAX31329_STATUS_REG_A2F);

	ret = regmap_write(info->map, MAX31329_STATUS_REG, stat);
	if (ret)
		return ret;

	ret = regmap_update_bits(info->map, MAX31329_RTC_RESET_REG,
				 MAX31329_RTC_RESET_REG_SWRST, 0);
	if (ret)
		return ret;

	config = FIELD_PREP(MAX31329_CONFIG1_REG_ENOSC, 1) |
		 FIELD_PREP(MAX31329_CONFIG1_REG_I2C_TIMEOUT, 1) |
		 FIELD_PREP(MAX31329_CONFIG1_REG_DATA_RET, 0) |
		 FIELD_PREP(MAX31329_CONFIG1_REG_ENIO, 1);

	ret = regmap_write(info->map, MAX31329_RTC_CONFIG1_REG, config);
	if (ret)
		return ret;

	/*
	 * If the alarm is pending, clear it before requesting
	 * the interrupt, so an interrupt event isn't reported
	 * before everything is initialized.
	 */
	return regmap_update_bits(info->map, MAX31329_INT_EN_REG,
				  MAX31329_INT_EN_REG_A1IE | MAX31329_INT_EN_REG_A2IE, 0);
}

static int max31329_interrupt_mode_init(struct max31329_rtc_info *info)
{
	struct device_node *node = info->dev->of_node;
	unsigned int config;
	int ret;

	if (!node)
		return 0;

	ret = regmap_read(info->map, MAX31329_RTC_CONFIG2_REG, &config);
	if (ret)
		return ret;

	if (of_property_read_bool(node, "clkin-enable"))
		config |= MAX31329_CONFIG2_REG_ENCLKIN;

	if (of_property_read_bool(node, "clko-enable"))
		config |= MAX31329_CONFIG2_REG_ENCLKO;

	return regmap_write(info->map, MAX31329_RTC_CONFIG2_REG, config);
}

static int max31329_read_time(struct device *dev, struct rtc_time *time)
{
	struct max31329_rtc_info *info = dev_get_drvdata(dev);
	unsigned int year, month, date, hour, minute, second;
	unsigned int weekday, twelve_hr, am_pm;
	unsigned int century, add_century = 0;
	u8 regs[RTC_NR_TIME];
	int ret;

	ret = regmap_bulk_read(info->map, MAX31329_SECONDS_REG, regs, RTC_NR_TIME);
	if (ret)
		return ret;

	second = regs[RTC_SEC];
	minute = regs[RTC_MIN];
	hour = regs[RTC_HOUR];
	weekday = regs[RTC_WEEKDAY];
	date = regs[RTC_DATE];
	month = regs[RTC_MONTH];
	year = regs[RTC_YEAR];

	/* Extract additional information for AM/PM and century */
	twelve_hr = hour & MAX31329_HOURS_REG_F_24_12;
	am_pm = hour & MAX31329_HOURS_REG_AM_PM;
	century = month & MAX31329_MONTH_REG_CENTURY;

	/* Write to rtc_time structure */
	time->tm_sec = bcd2bin(second);
	time->tm_min = bcd2bin(minute);
	if (twelve_hr) {
		/* Convert to 24 hr */
		if (am_pm)
			time->tm_hour = bcd2bin(hour & 0x1F) + 12;
		else
			time->tm_hour = bcd2bin(hour & 0x1F);
	} else {
		time->tm_hour = bcd2bin(hour);
	}
	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	time->tm_wday = bcd2bin(weekday) - 1;
	time->tm_mday = bcd2bin(date);
	/* Linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	time->tm_mon = bcd2bin(month & 0x1f) - 1;
	if (century)
		add_century = 100;
	/* Add 100 to support up to 2099 */
	time->tm_year = bcd2bin(regs[RTC_YEAR]) + add_century + 100;

	return 0;
}

static int max31329_set_time(struct device *dev, struct rtc_time *time)
{
	struct max31329_rtc_info *info = dev_get_drvdata(dev);
	u8 regs[RTC_NR_TIME];

	regs[RTC_SEC] = bin2bcd(time->tm_sec);
	regs[RTC_MIN] = bin2bcd(time->tm_min);
	regs[RTC_HOUR] = bin2bcd(time->tm_hour);
	/* Day of the week in linux range is 0~6 while 1~7 in RTC chip */
	regs[RTC_WEEKDAY] = bin2bcd(time->tm_wday + 1);
	regs[RTC_DATE] = bin2bcd(time->tm_mday);
	/* Linux tm_mon range:0~11, while month range is 1~12 in RTC chip */
	regs[RTC_MONTH] = bin2bcd(time->tm_mon + 1);
	if (time->tm_year >= 200) {
		regs[RTC_MONTH] |= MAX31329_MONTH_REG_CENTURY;
		regs[RTC_YEAR] = bin2bcd(time->tm_year - 200);
	} else if (time->tm_year >= 100) {
		regs[RTC_YEAR] = bin2bcd(time->tm_year - 100);
	} else {
		dev_dbg(dev, "Invalid set date! %04d-%02d-%02d %02d:%02d:%02d",
			time->tm_year + 1900, time->tm_mon + 1,
			time->tm_mday, time->tm_hour, time->tm_min,
			time->tm_sec);
		return -EINVAL;
	}

	return regmap_bulk_write(info->map, MAX31329_SECONDS_REG, regs, RTC_NR_TIME);
}

static int max31329_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31329_rtc_info *info = dev_get_drvdata(dev);
	unsigned int control, stat, month;
	int ret, century;
	u8 regs[RTC_NR_ALM_TIME];

	ret = regmap_read(info->map, MAX31329_STATUS_REG, &stat);
	if (ret)
		return ret;

	ret = regmap_read(info->map, MAX31329_INT_EN_REG, &control);
	if (ret)
		return ret;

	ret = regmap_bulk_read(info->map, MAX31329_ALM1_SEC_REG, regs, RTC_NR_ALM_TIME);
	if (ret)
		return ret;

	alarm->time.tm_sec = bcd2bin(regs[RTC_ALM_SEC] & 0x7f);
	alarm->time.tm_min = bcd2bin(regs[RTC_ALM_MIN] & 0x7f);
	alarm->time.tm_hour = bcd2bin(regs[RTC_ALM_HOUR] & 0x3f);
	alarm->time.tm_mday = bcd2bin(regs[RTC_ALM_DATE] & 0x3f);
	alarm->time.tm_mon = bcd2bin(regs[RTC_ALM_MONTH] & 0x1f);

	ret = regmap_read(info->map, MAX31329_MONTH_REG, &month);
	if (ret)
		return ret;

	century = !!(month & MAX31329_MONTH_REG_CENTURY);
	alarm->time.tm_year = bcd2bin(regs[RTC_ALM_YEAR]) + 100 + century * 100;

	alarm->enabled = !!(control & MAX31329_INT_EN_REG_A1IE);
	alarm->pending = !!(stat & MAX31329_STATUS_REG_A1F);

	return 0;
}

static int max31329_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct max31329_rtc_info *info = dev_get_drvdata(dev);
	u8 regs[RTC_NR_ALM_TIME];
	int ret;

	if (info->irq <= 0)
		return -EINVAL;

	regs[RTC_ALM_SEC] = bin2bcd(alarm->time.tm_sec);
	regs[RTC_ALM_MIN] = bin2bcd(alarm->time.tm_min);
	regs[RTC_ALM_HOUR] = bin2bcd(alarm->time.tm_hour);
	regs[RTC_ALM_DATE] = bin2bcd(alarm->time.tm_mday);
	regs[RTC_ALM_MONTH] = (bin2bcd(alarm->time.tm_mon) + 1);

	if (alarm->time.tm_year >= 200) {
		regs[RTC_ALM_YEAR] = bin2bcd(alarm->time.tm_year - 200);
	} else if (alarm->time.tm_year >= 100) {
		regs[RTC_ALM_YEAR] = bin2bcd(alarm->time.tm_year - 100);
	} else {
		dev_dbg(dev, "Invalid set alarm! %04d-%02d-%02d %02d:%02d:%02d",
			alarm->time.tm_year + 1900, alarm->time.tm_mon + 1,
			alarm->time.tm_mday, alarm->time.tm_hour, alarm->time.tm_min,
			alarm->time.tm_sec);
		return -EINVAL;
	}

	/* clear any pending alarm flag */
	ret = regmap_update_bits(info->map, MAX31329_STATUS_REG,
				 MAX31329_STATUS_REG_A1F | MAX31329_STATUS_REG_A2F, 0);
	if (ret)
		return ret;

	/* clear alarm interrupt enable bit */
	ret = regmap_update_bits(info->map, MAX31329_INT_EN_REG,
				 MAX31329_INT_EN_REG_A1IE | MAX31329_INT_EN_REG_A2IE, 0);
	if (ret)
		return ret;

	ret = regmap_bulk_write(info->map, MAX31329_ALM1_SEC_REG, regs,
				RTC_NR_ALM_TIME);
	if (ret)
		return ret;

	if (alarm->enabled) {
		ret = regmap_update_bits(info->map, MAX31329_INT_EN_REG,
					 MAX31329_INT_EN_REG_A1IE, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int max31329_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct max31329_rtc_info *info = dev_get_drvdata(dev);

	if (enabled)
		return regmap_update_bits(info->map, MAX31329_INT_EN_REG,
					  MAX31329_INT_EN_REG_A1IE, 1);
	else
		return regmap_update_bits(info->map, MAX31329_INT_EN_REG,
					  MAX31329_INT_EN_REG_A1IE, 0);
}

static irqreturn_t max31329_irq(int irq, void *dev_id)
{
	struct max31329_rtc_info *info = dev_id;

	disable_irq_nosync(irq);
	schedule_work(&info->work);

	return IRQ_HANDLED;
}

static void max31329_work(struct work_struct *work)
{
	struct max31329_rtc_info *info = container_of(work, struct max31329_rtc_info, work);
	struct mutex *lock = &info->rtc->ops_lock;
	unsigned int stat, control;
	int ret;

	mutex_lock(lock);

	ret = regmap_read(info->map, MAX31329_STATUS_REG, &stat);
	if (ret)
		goto unlock;

	if (stat & MAX31329_STATUS_REG_A1F) {
		ret = regmap_read(info->map, MAX31329_INT_EN_REG, &control);
		if (ret) {
			dev_warn(info->dev, "Read Control Register error %d\n", ret);
		} else {
			/* disable alarm1 interrupt */
			control &= ~MAX31329_INT_EN_REG_A1IE;
			ret = regmap_write(info->map, MAX31329_INT_EN_REG, control);
			if (ret)
				goto unlock;
			/* clear the alarm pend flag */
			stat &= ~MAX31329_STATUS_REG_A1F;
			ret = regmap_write(info->map, MAX31329_STATUS_REG, stat);
			if (ret)
				goto unlock;

			rtc_update_irq(info->rtc, 1, RTC_AF | RTC_IRQF);
		}
	}

	enable_irq(info->irq);
unlock:
	mutex_unlock(lock);
}

static const struct rtc_class_ops max31329_rtc_ops = {
	.read_time = max31329_read_time,
	.set_time = max31329_set_time,
	.read_alarm = max31329_read_alarm,
	.set_alarm = max31329_set_alarm,
	.alarm_irq_enable = max31329_alarm_irq_enable,
};

static const struct regmap_config config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int max31329_probe(struct i2c_client *client)
{
	struct max31329_rtc_info *info;
	int ret;

	info = devm_kzalloc(&client->dev, sizeof(struct max31329_rtc_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &client->dev;
	info->irq = client->irq;

	i2c_set_clientdata(client, info);
	devm_work_autocancel(info->dev, &info->work, max31329_work);

	info->map = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(info->map))
		return dev_err_probe(info->dev, PTR_ERR(info->map), "regmap allocation failed\n");

	info->rtc = devm_rtc_device_register(info->dev, client->name,
					     &max31329_rtc_ops, THIS_MODULE);

	if (IS_ERR(info->rtc))
		return dev_err_probe(info->dev, PTR_ERR(info->rtc), "device registration failed\n");

	ret = max31329_check_rtc_status(info);
	if (ret)
		return ret;

	ret = max31329_interrupt_mode_init(info);
	if (ret)
		return ret;

	if (info->irq > 0) {
		ret = devm_request_irq(info->dev, info->irq, max31329_irq, 0, "max31329", info);
		if (ret)
			return dev_err_probe(info->dev, PTR_ERR(&ret), "unable to request IRQ\n");

		device_set_wakeup_capable(info->dev, 1);
	}

	return 0;
}

static const struct i2c_device_id max31329_id[] = {
	{ "max31329", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31329_id);

static struct i2c_driver max31329_driver = {
	.driver = {
		.name = "rtc-max31329",
	},
	.probe_new = max31329_probe,
	.id_table = max31329_id,
};

module_i2c_driver(max31329_driver);

MODULE_DESCRIPTION("Maxim MAX31329 RTC Driver");
MODULE_AUTHOR("Zeynep Arslanbenzer <Zeynep.Arslanbenzer@analog.com>");
MODULE_LICENSE("GPL");
