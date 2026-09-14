// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX42500 - Programmable Windowed Watchdog
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/mfd/max42500.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

enum max42500_wdt_wd_mode {
	MAX42500_WD_MODE_CH_RESP,
	MAX42500_WD_MODE_SIMPLE,
	MAX42500_WD_MODE_MAX
};

enum max42500_wdt_wdog_attr {
	MAX42500_WDOG_WD_STATUS,
	MAX42500_WDOG_WD_SIMPLE_MODE_ENABLE,
	MAX42500_WDOG_WD_CLOCK_DIVIDER,
	MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL,
	MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL,
	MAX42500_WDOG_WD_ENABLE,
	MAX42500_WDOG_WD_FIRST_UPDATE_INTERVAL,
	MAX42500_WDOG_WD_KEY,
	MAX42500_WDOG_WD_LOCK_ENABLE,
	MAX42500_WDOG_WD_RESET_TWO_COUNT_ENABLE,
	MAX42500_WDOG_WD_RESET_HOLD_INTERVAL,
	MAX42500_WDOG_MAX
};

static const u8 max42500_wdt_reg_wdog[] = {
	MAX42500_REG_WDSTAT,
	MAX42500_REG_WDCDIV,
	MAX42500_REG_WDCDIV,
	MAX42500_REG_WDCFG1,
	MAX42500_REG_WDCFG1,
	MAX42500_REG_WDCFG2,
	MAX42500_REG_WDCFG2,
	MAX42500_REG_WDKEY,
	MAX42500_REG_WDLOCK,
	MAX42500_REG_RSTCTRL,
	MAX42500_REG_RSTCTRL
};

struct max42500_wdt_state {
	struct watchdog_device wwd;
	struct regmap *regmap;
	struct mutex lock; /* Protects access during data transfer */
	u8 wdog[MAX42500_WDOG_MAX];
};

/* Watchdog Reset hold time (usec) to Register Conversion */
static int max42500_wdt_convert_wd_holdreset_to_reg(long value, long *reg_val)
{
	/* Check values if divisible by 8. */
	if (value % 8)
		return -EINVAL;

	/* Valid are 0, 8, 16 and 32 only. */
	value = clamp_val(value, 0, 32);
	if (value == 0) {
		*reg_val = value;
		return 0;
	}

	/* Test for valid bits 1, 2 or 4. */
	if (((value >> 3) ^ 3)) {
		*reg_val = (value >> 4) + 1;
		return 0;
	}

	return -EINVAL;
}

/* Watchdog Key Challenge-Response Computation */
static int max42500_wdt_set_watchdog_key(struct max42500_wdt_state *st,
					  enum max42500_wdt_wd_mode mode)
{
	unsigned int reg_val = 0;
	unsigned int key;
	u8 lfsr;
	int ret;

	switch (mode) {
	case MAX42500_WD_MODE_CH_RESP:
		scoped_guard(mutex, &st->lock) {
			ret = regmap_read(st->regmap, MAX42500_REG_WDKEY, &key);
			if (ret)
				return ret;

		/* Linear-Feedback Shift Register (LFSR) Polynomial Equation */
			lfsr = ((key >> 7) ^ (key >> 5) ^ (key >> 4) ^
				(key >> 3)) & 1;
			reg_val = (key << 1) | lfsr;
		}

		return regmap_write(st->regmap, MAX42500_REG_WDKEY, reg_val);
	case MAX42500_WD_MODE_SIMPLE:
		/* Any write to WDKEY register will update the watchdog */
		return regmap_write(st->regmap, MAX42500_REG_WDKEY, reg_val);
	default:
		return -EINVAL;
	}
}

static int max42500_wdt_wdog_get_update(struct max42500_wdt_state *st,
					enum max42500_wdt_wdog_attr index)
{
	unsigned int reg_val;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, max42500_wdt_reg_wdog[index], &reg_val);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_WDOG_WD_STATUS: /* WDSTAT */
		st->wdog[index] = MAX42500_WDOG_WDSTAT(reg_val);
		return 0;
	case MAX42500_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		st->wdog[index] = MAX42500_WDOG_WDIV(reg_val);
		return 0;
	case MAX42500_WDOG_WD_SIMPLE_MODE_ENABLE: /* SWW */
		st->wdog[index] = MAX42500_WDOG_SWW(reg_val);
		return 0;
	case MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
		st->wdog[index] = MAX42500_WDOG_WDOPEN(reg_val);
		return 0;
	case MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		st->wdog[index] = MAX42500_WDOG_WDCLOSE(reg_val);
		return 0;
	case MAX42500_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		st->wdog[index] = MAX42500_WDOG_1UD(reg_val);
		return 0;
	case MAX42500_WDOG_WD_ENABLE: /* WDEN */
		st->wdog[index] = MAX42500_WDOG_WDEN(reg_val);
		return 0;
	case MAX42500_WDOG_WD_KEY: /* WDKEY */
		st->wdog[index] = MAX42500_WDOG_WDKEY(reg_val);
		return 0;
	case MAX42500_WDOG_WD_LOCK_ENABLE: /* WDLOCK */
		st->wdog[index] = MAX42500_WDOG_WDLOCK(reg_val);
		return 0;
	case MAX42500_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		st->wdog[index] = MAX42500_WDOG_RHLD(reg_val);
		return 0;
	case MAX42500_WDOG_WD_RESET_TWO_COUNT_ENABLE: /* MR1 */
		st->wdog[index] = MAX42500_WDOG_MR1(reg_val);
		return 0;
	default:
		return -EINVAL;
	}
}

static int max42500_wdt_wdog_set_update(struct max42500_wdt_state *st,
					long value,
					enum max42500_wdt_wdog_attr index)
{
	unsigned int reg_mask;
	int ret;

	guard(mutex)(&st->lock);

	switch (index) {
	case MAX42500_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		reg_mask = MAX42500_WDOG_WDIV_MASK;
		break;
	case MAX42500_WDOG_WD_SIMPLE_MODE_ENABLE: /* SWW */
		reg_mask = MAX42500_WDOG_SWW_MASK;
		break;
	case MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
		reg_mask = MAX42500_WDOG_WDOPEN_MASK;
		break;
	case MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		reg_mask = MAX42500_WDOG_WDCLOSE_MASK;
		break;
	case MAX42500_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		reg_mask = MAX42500_WDOG_1UD_MASK;
		break;
	case MAX42500_WDOG_WD_ENABLE: /* WDEN */
		reg_mask = MAX42500_WDOG_WDEN_MASK;
		break;
	case MAX42500_WDOG_WD_KEY: /* WDKEY */
		reg_mask = MAX42500_WDOG_WDKEY_MASK;
		break;
	case MAX42500_WDOG_WD_LOCK_ENABLE: /* WDLOCK */
		reg_mask = MAX42500_WDOG_WDLOCK_MASK;
		break;
	case MAX42500_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		reg_mask = MAX42500_WDOG_RHLD_MASK;
		break;
	case MAX42500_WDOG_WD_RESET_TWO_COUNT_ENABLE: /* MR1 */
		reg_mask = MAX42500_WDOG_MR1_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(st->regmap, max42500_wdt_reg_wdog[index],
				 reg_mask, value);
	if (ret)
		return ret;

	/* Update after successful register write */
	st->wdog[index] = FIELD_GET(GENMASK(7, 0), value);

	return 0;
}

static int max42500_wdt_wdog_read(struct max42500_wdt_state *st, long *value,
				  enum max42500_wdt_wdog_attr index)
{
	u8 clk_div = MAX42500_WDOG_WD_CLOCK_DIVIDER;
	u8 opn_win = MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL;
	u8 clo_win = MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL;
	int ret;

	ret = max42500_wdt_wdog_get_update(st, index);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		*value = (st->wdog[index] + 1) * 200;
		return 0;
	case MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
	case MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		ret = max42500_wdt_wdog_get_update(st, clk_div);
		if (ret)
			return ret;

		*value = ((st->wdog[index] + 1) << 3) * st->wdog[clk_div];
		return 0;
	case MAX42500_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		ret = max42500_wdt_wdog_get_update(st, clo_win);
		if (ret)
			return ret;

		ret = max42500_wdt_wdog_get_update(st, opn_win);
		if (ret)
			return ret;

		*value = (st->wdog[clo_win] + st->wdog[opn_win]) *
			 ((st->wdog[index] << 1) + 1);
		return 0;
	case MAX42500_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		*value = st->wdog[index] ?  (1 << (st->wdog[index] + 3)) : 0;
		return 0;
	default:
		*value = st->wdog[index];
		return 0;
	}
}

static int max42500_wdt_wdog_write(struct max42500_wdt_state *st, long value,
				    enum max42500_wdt_wdog_attr index)
{
	u8 clk_div = MAX42500_WDOG_WD_CLOCK_DIVIDER;
	u8 win_mode = MAX42500_WDOG_WD_SIMPLE_MODE_ENABLE;
	u8 opn_win = MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL;
	u8 clo_win = MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL;
	long wdog;
	int ret;

	switch (index) {
	case MAX42500_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		wdog = (value - 200) / 200;
		break;
	case MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
	case MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		wdog = value / (st->wdog[clk_div] << 3);
		break;
	case MAX42500_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		wdog = ((value / (st->wdog[clo_win] + st->wdog[opn_win]))
			- 1) >> 1;
		break;
	case MAX42500_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		/* Valid values are 0, 8, 16, 32 and truncates larger values */
		ret = max42500_wdt_convert_wd_holdreset_to_reg(value, &wdog);
		if (ret)
			return ret;
		break;
	case MAX42500_WDOG_WD_KEY: /* WDKEY */
		/* User cannot input watchdog key but can trigger key update */
		ret = max42500_wdt_set_watchdog_key(st, st->wdog[win_mode]);
		if (ret)
			return ret;

		return 0;
	default:
		return -EINVAL;
	}

	return max42500_wdt_wdog_set_update(st, wdog, index);
}

static int max42500_wdt_show_wd_simp_mode_enable_log(void *arg, u64 *val)
{
	long wdog_val;
	int ret;

	ret = max42500_wdt_wdog_read(arg, &wdog_val,
				      MAX42500_WDOG_WD_SIMPLE_MODE_ENABLE);
	if (ret)
		return ret;

	*val = wdog_val;
	return 0;
}

static int max42500_wdt_store_wd_simp_mode_enable_log(void *arg, u64 val)
{
	return max42500_wdt_wdog_write(arg, val,
				       MAX42500_WDOG_WD_SIMPLE_MODE_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_wdt_wd_simp_mode_enable_log,
			 max42500_wdt_show_wd_simp_mode_enable_log,
			 max42500_wdt_store_wd_simp_mode_enable_log, "%llu\n");

static int max42500_wdt_show_wd_open_win_log(void *arg, u64 *val)
{
	long wdog_val;
	int ret;

	ret = max42500_wdt_wdog_read(arg, &wdog_val,
				      MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL);
	if (ret)
		return ret;

	*val = wdog_val;
	return 0;
}

static int max42500_wdt_store_wd_open_win_log(void *arg, u64 val)
{
	return max42500_wdt_wdog_write(arg, val,
				       MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_wdt_wd_open_win_log,
			 max42500_wdt_show_wd_open_win_log,
			 max42500_wdt_store_wd_open_win_log, "%llu\n");

static int max42500_wdt_show_wd_close_win_log(void *arg, u64 *val)
{
	long wdog_val;
	int ret;

	ret = max42500_wdt_wdog_read(arg, &wdog_val,
				      MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL);
	if (ret)
		return ret;

	*val = wdog_val;
	return 0;
}

static int max42500_wdt_store_wd_close_win_log(void *arg, u64 val)
{
	return max42500_wdt_wdog_write(arg, val,
				       MAX42500_WDOG_WD_CLOSE_WINDOW_INTERVAL);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_wdt_wd_close_win_log,
			 max42500_wdt_show_wd_close_win_log,
			 max42500_wdt_store_wd_close_win_log, "%llu\n");

static int max42500_wdt_show_wd_lock_enablelog(void *arg, u64 *val)
{
	long wdog_val;
	int ret;

	ret = max42500_wdt_wdog_read(arg, &wdog_val,
				      MAX42500_WDOG_WD_LOCK_ENABLE);
	if (ret)
		return ret;

	*val = wdog_val;
	return 0;
}

static int max42500_wdt_store_wd_lock_enable_log(void *arg, u64 val)
{
	return max42500_wdt_wdog_write(arg, val,
				       MAX42500_WDOG_WD_LOCK_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_wdt_wd_lock_enable_log,
			 max42500_wdt_show_wd_lock_enablelog,
			 max42500_wdt_store_wd_lock_enable_log, "%llu\n");

static int max42500_wdt_show_wd_rst2_cnt_enable_log(void *arg, u64 *val)
{
	long wdog_val;
	int ret;

	ret = max42500_wdt_wdog_read(arg, &wdog_val,
				      MAX42500_WDOG_WD_RESET_TWO_COUNT_ENABLE);
	if (ret)
		return ret;

	*val = wdog_val;
	return 0;
}

static int max42500_wdt_store_wd_rst2_cnt_enable_log(void *arg, u64 val)
{
	return max42500_wdt_wdog_write(arg, val,
				       MAX42500_WDOG_WD_RESET_TWO_COUNT_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_wdt_wd_rst2_cnt_enable_log,
			 max42500_wdt_show_wd_rst2_cnt_enable_log,
			 max42500_wdt_store_wd_rst2_cnt_enable_log, "%llu\n");

static void max42500_wdt_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static void max42500_wdt_debugfs_init(struct max42500_wdt_state *st,
				      struct platform_device *pdev,
				      const struct device *watchdog)
{
	const char *debugfs_name;
	struct dentry *dentry;
	int ret;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "max42500-%s",
				      dev_name(watchdog));
	if (!debugfs_name)
		return;

	dentry = debugfs_create_dir(debugfs_name, NULL);
	if (IS_ERR(dentry))
		return;

	ret = devm_add_action_or_reset(&pdev->dev, max42500_wdt_debugfs_remove,
				       dentry);
	if (ret)
		return;

	/* Watchdog registers */
	debugfs_create_file_unsafe("wd_simp_mode_enable_log", 0644, dentry, st,
				   &max42500_wdt_wd_simp_mode_enable_log);
	debugfs_create_file_unsafe("wd_open_win_log", 0644, dentry, st,
				   &max42500_wdt_wd_open_win_log);
	debugfs_create_file_unsafe("wd_close_win_log", 0644, dentry, st,
				   &max42500_wdt_wd_close_win_log);
	debugfs_create_file_unsafe("wd_lock_enable_log", 0644, dentry, st,
				   &max42500_wdt_wd_lock_enable_log);
	debugfs_create_file_unsafe("wd_rst2_cnt_enable_log", 0644, dentry, st,
				   &max42500_wdt_wd_rst2_cnt_enable_log);
}

static int max42500_wdt_start(struct watchdog_device *wdd)
{
	return max42500_wdt_wdog_write(watchdog_get_drvdata(wdd), 1,
				       MAX42500_WDOG_WD_ENABLE);
}

static int max42500_wdt_stop(struct watchdog_device *wdd)
{
	return max42500_wdt_wdog_write(watchdog_get_drvdata(wdd), 0,
				       MAX42500_WDOG_WD_ENABLE);
}

static int max42500_wdt_ping(struct watchdog_device *wdd)
{
	return max42500_wdt_wdog_write(watchdog_get_drvdata(wdd), 0,
				       MAX42500_WDOG_WD_KEY);
}

static unsigned int max42500_wdt_status(struct watchdog_device *wdd)
{
	long wdog_val;
	int ret;

	ret = max42500_wdt_wdog_read(watchdog_get_drvdata(wdd), &wdog_val,
				     MAX42500_WDOG_WD_STATUS);
	if (ret)
		return MAX42500_WDOG_WDSTAT_MASK;

	return wdog_val;
}

static int max42500_wdt_set_timeout(struct watchdog_device *wdd,
				    unsigned int val)
{
	return max42500_wdt_wdog_write(watchdog_get_drvdata(wdd), val,
				       MAX42500_WDOG_WD_CLOCK_DIVIDER);
}

static int max42500_wdt_set_pretimeout(struct watchdog_device *wdd,
				       unsigned int val)
{
	return max42500_wdt_wdog_write(watchdog_get_drvdata(wdd), val,
				       MAX42500_WDOG_WD_FIRST_UPDATE_INTERVAL);
}

static int max42500_wdt_restart(struct watchdog_device *wdd,
				unsigned long val, void *info)
{
	return max42500_wdt_wdog_write(watchdog_get_drvdata(wdd), val,
				       MAX42500_WDOG_WD_RESET_HOLD_INTERVAL);
}

static const struct watchdog_ops max42500_wdt_watchdog_ops = {
	.owner = THIS_MODULE,
	.start = max42500_wdt_start,
	.stop = max42500_wdt_stop,
	.ping = max42500_wdt_ping,
	.status = max42500_wdt_status,
	.set_timeout = max42500_wdt_set_timeout,
	.set_pretimeout = max42500_wdt_set_pretimeout,
	.restart = max42500_wdt_restart,
};

static const struct watchdog_info max42500_wdt_watchdog_info = {
	.options = WDIOF_KEEPALIVEPING,
	.identity = "max42500_wdt Watchdog",
};

static int max42500_wdt_probe(struct platform_device *pdev)
{
	struct device *parent_dev = pdev->dev.parent;
	struct regmap *regmap = dev_get_drvdata(parent_dev);
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdog_dev;
	struct max42500_wdt_state *st;
	long open_wdt, close_wdt;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->regmap = regmap;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	wdog_dev = &st->wwd;
	wdog_dev->parent = &pdev->dev;
	wdog_dev->info = &max42500_wdt_watchdog_info;
	wdog_dev->ops = &max42500_wdt_watchdog_ops;

	ret = max42500_wdt_wdog_read(st, &open_wdt,
				     MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL);
	if (ret)
		return ret;

	wdog_dev->min_timeout = open_wdt;

	ret = max42500_wdt_wdog_read(st, &close_wdt,
				     MAX42500_WDOG_WD_OPEN_WINDOW_INTERVAL);
	if (ret)
		return ret;

	wdog_dev->max_timeout = open_wdt + close_wdt;

	platform_set_drvdata(pdev, st);

	max42500_wdt_debugfs_init(st, pdev, dev);

	watchdog_set_drvdata(wdog_dev, st);

	return devm_watchdog_register_device(dev, wdog_dev);
}

static const struct platform_device_id max42500_wdt_id_table[] = {
	{ "max42500-wdt" },
	{  }
};
MODULE_DEVICE_TABLE(platform, max42500_wdt_id_table);

static struct platform_driver max42500_wdt_driver = {
	.driver = {
		.name = "max42500-wdt",
	},
	.probe = max42500_wdt_probe,
	.id_table = max42500_wdt_id_table,
};
module_platform_driver(max42500_wdt_driver);

MODULE_AUTHOR("Kent Libetario <kent.libetario@analog.com>");
MODULE_DESCRIPTION("Watchdog driver for MAX42500");
MODULE_LICENSE("GPL");
