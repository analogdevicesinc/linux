// SPDX-License-Identifier: GPL-2.0
/*
 * Debugfs interface
 *
 * Copyright (C) 2020, Intel Corporation
 * Authors: Gil Fine <gil.fine@intel.com>
 *	    Mika Westerberg <mika.westerberg@linux.intel.com>
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>

#include "tb.h"
#include "sb_regs.h"

#define PORT_CAP_V1_PCIE_LEN	1
#define PORT_CAP_V2_PCIE_LEN	2
#define PORT_CAP_POWER_LEN	2
#define PORT_CAP_LANE_LEN	3
#define PORT_CAP_USB3_LEN	5
#define PORT_CAP_DP_V1_LEN	9
#define PORT_CAP_DP_V2_LEN	14
#define PORT_CAP_TMU_V1_LEN	8
#define PORT_CAP_TMU_V2_LEN	10
#define PORT_CAP_BASIC_LEN	9
#define PORT_CAP_USB4_LEN	20

#define SWITCH_CAP_TMU_LEN	26
#define SWITCH_CAP_BASIC_LEN	27

#define PATH_LEN		2

#define COUNTER_SET_LEN		3

/*
 * USB4 spec doesn't specify dwell range, the range of 100 ms to 500 ms
 * probed to give good results.
 */
#define MIN_DWELL_TIME		100 /* ms */
#define MAX_DWELL_TIME		500 /* ms */
#define DWELL_SAMPLE_INTERVAL	10

/* Sideband registers and their sizes as defined in the USB4 spec */
struct sb_reg {
	unsigned int reg;
	unsigned int size;
};

#define SB_MAX_SIZE		64

/* Sideband registers for router */
static const struct sb_reg port_sb_regs[] = {
	{ USB4_SB_VENDOR_ID, 4 },
	{ USB4_SB_PRODUCT_ID, 4 },
	{ USB4_SB_DEBUG_CONF, 4 },
	{ USB4_SB_DEBUG, 54 },
	{ USB4_SB_LRD_TUNING, 4 },
	{ USB4_SB_OPCODE, 4 },
	{ USB4_SB_METADATA, 4 },
	{ USB4_SB_LINK_CONF, 3 },
	{ USB4_SB_GEN23_TXFFE, 4 },
	{ USB4_SB_GEN4_TXFFE, 4 },
	{ USB4_SB_VERSION, 4 },
	{ USB4_SB_DATA, 64 },
};

/* Sideband registers for retimer */
static const struct sb_reg retimer_sb_regs[] = {
	{ USB4_SB_VENDOR_ID, 4 },
	{ USB4_SB_PRODUCT_ID, 4 },
	{ USB4_SB_FW_VERSION, 4 },
	{ USB4_SB_LRD_TUNING, 4 },
	{ USB4_SB_OPCODE, 4 },
	{ USB4_SB_METADATA, 4 },
	{ USB4_SB_GEN23_TXFFE, 4 },
	{ USB4_SB_GEN4_TXFFE, 4 },
	{ USB4_SB_VERSION, 4 },
	{ USB4_SB_DATA, 64 },
};

#define DEBUGFS_ATTR(__space, __write)					\
static int __space ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __space ## _show, inode->i_private);	\
}									\
									\
static const struct file_operations __space ## _fops = {		\
	.owner = THIS_MODULE,						\
	.open = __space ## _open,					\
	.release = single_release,					\
	.read  = seq_read,						\
	.write = __write,						\
	.llseek = seq_lseek,						\
}

#define DEBUGFS_ATTR_RO(__space)					\
	DEBUGFS_ATTR(__space, NULL)

#define DEBUGFS_ATTR_RW(__space)					\
	DEBUGFS_ATTR(__space, __space ## _write)

static struct dentry *tb_debugfs_root;

static void *validate_and_copy_from_user(const void __user *user_buf,
					 size_t *count)
{
	size_t nbytes;
	void *buf;

	if (!*count)
		return ERR_PTR(-EINVAL);

	if (!access_ok(user_buf, *count))
		return ERR_PTR(-EFAULT);

	buf = (void *)get_zeroed_page(GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	nbytes = min_t(size_t, *count, PAGE_SIZE);
	if (copy_from_user(buf, user_buf, nbytes)) {
		free_page((unsigned long)buf);
		return ERR_PTR(-EFAULT);
	}

	*count = nbytes;
	return buf;
}

static bool parse_line(char **line, u32 *offs, u32 *val, int short_fmt_len,
		       int long_fmt_len)
{
	char *token;
	u32 v[5];
	int ret;

	token = strsep(line, "\n");
	if (!token)
		return false;

	/*
	 * For Adapter/Router configuration space:
	 * Short format is: offset value\n
	 *		    v[0]   v[1]
	 * Long format as produced from the read side:
	 * offset relative_offset cap_id vs_cap_id value\n
	 * v[0]   v[1]            v[2]   v[3]      v[4]
	 *
	 * For Counter configuration space:
	 * Short format is: offset\n
	 *		    v[0]
	 * Long format as produced from the read side:
	 * offset relative_offset counter_id value\n
	 * v[0]   v[1]            v[2]       v[3]
	 */
	ret = sscanf(token, "%i %i %i %i %i", &v[0], &v[1], &v[2], &v[3], &v[4]);
	/* In case of Counters, clear counter, "val" content is NA */
	if (ret == short_fmt_len) {
		*offs = v[0];
		*val = v[short_fmt_len - 1];
		return true;
	} else if (ret == long_fmt_len) {
		*offs = v[0];
		*val = v[long_fmt_len - 1];
		return true;
	}

	return false;
}

#if IS_ENABLED(CONFIG_USB4_DEBUGFS_WRITE)
static ssize_t regs_write(struct tb_switch *sw, struct tb_port *port,
			  const char __user *user_buf, size_t count,
			  loff_t *ppos)
{
	struct tb *tb = sw->tb;
	char *line, *buf;
	u32 val, offset;
	int ret = 0;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	/* User did hardware changes behind the driver's back */
	add_taint(TAINT_USER, LOCKDEP_STILL_OK);

	line = buf;
	while (parse_line(&line, &offset, &val, 2, 5)) {
		if (port)
			ret = tb_port_write(port, &val, TB_CFG_PORT, offset, 1);
		else
			ret = tb_sw_write(sw, &val, TB_CFG_SWITCH, offset, 1);
		if (ret)
			break;
	}

	mutex_unlock(&tb->lock);

out:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);
	free_page((unsigned long)buf);

	return ret < 0 ? ret : count;
}

static ssize_t port_regs_write(struct file *file, const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_port *port = s->private;

	return regs_write(port->sw, port, user_buf, count, ppos);
}

static ssize_t switch_regs_write(struct file *file, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_switch *sw = s->private;

	return regs_write(sw, NULL, user_buf, count, ppos);
}

static bool parse_sb_line(char **line, u8 *reg, u8 *data, size_t data_size,
			  size_t *bytes_read)
{
	char *field, *token;
	int i;

	token = strsep(line, "\n");
	if (!token)
		return false;

	/* Parse the register first */
	field = strsep(&token, " ");
	if (!field)
		return false;
	if (kstrtou8(field, 0, reg))
		return false;

	/* Then the values for the register, up to data_size */
	for (i = 0; i < data_size; i++) {
		field = strsep(&token, " ");
		if (!field)
			break;
		if (kstrtou8(field, 0, &data[i]))
			return false;
	}

	*bytes_read = i;
	return true;
}

static ssize_t sb_regs_write(struct tb_port *port, const struct sb_reg *sb_regs,
			     size_t size, enum usb4_sb_target target, u8 index,
			     char *buf, size_t count, loff_t *ppos)
{
	u8 reg, data[SB_MAX_SIZE];
	size_t bytes_read;
	char *line = buf;

	/* User did hardware changes behind the driver's back */
	add_taint(TAINT_USER, LOCKDEP_STILL_OK);

	/*
	 * For sideband registers we accept:
	 * reg b0 b1 b2...\n
	 *
	 * Here "reg" is the byte offset of the sideband register and "b0"..
	 * are the byte values. There can be less byte values than the register
	 * size. The leftovers will not be overwritten.
	 */
	while (parse_sb_line(&line, &reg, data, ARRAY_SIZE(data), &bytes_read)) {
		const struct sb_reg *sb_reg;
		int ret;

		/* At least one byte must be passed */
		if (bytes_read < 1)
			return -EINVAL;

		/* Find the register */
		sb_reg = NULL;
		for (int i = 0; i < size; i++) {
			if (sb_regs[i].reg == reg) {
				sb_reg = &sb_regs[i];
				break;
			}
		}

		if (!sb_reg)
			return -EINVAL;

		if (bytes_read > sb_regs->size)
			return -E2BIG;

		ret = usb4_port_sb_write(port, target, index, sb_reg->reg, data,
					 bytes_read);
		if (ret)
			return ret;
	}

	return 0;
}

static ssize_t port_sb_regs_write(struct file *file, const char __user *user_buf,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_port *port = s->private;
	struct tb_switch *sw = port->sw;
	struct tb *tb = sw->tb;
	char *buf;
	int ret;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	ret = sb_regs_write(port, port_sb_regs, ARRAY_SIZE(port_sb_regs),
			    USB4_SB_TARGET_ROUTER, 0, buf, count, ppos);

	mutex_unlock(&tb->lock);
out:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);
	free_page((unsigned long)buf);

	return ret < 0 ? ret : count;
}

static ssize_t retimer_sb_regs_write(struct file *file,
				     const char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_retimer *rt = s->private;
	struct tb *tb = rt->tb;
	char *buf;
	int ret;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	pm_runtime_get_sync(&rt->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	ret = sb_regs_write(rt->port, retimer_sb_regs, ARRAY_SIZE(retimer_sb_regs),
			    USB4_SB_TARGET_RETIMER, rt->index, buf, count, ppos);

	mutex_unlock(&tb->lock);
out:
	pm_runtime_mark_last_busy(&rt->dev);
	pm_runtime_put_autosuspend(&rt->dev);
	free_page((unsigned long)buf);

	return ret < 0 ? ret : count;
}
#define DEBUGFS_MODE		0600
#else
#define port_regs_write		NULL
#define switch_regs_write	NULL
#define port_sb_regs_write	NULL
#define retimer_sb_regs_write	NULL
#define DEBUGFS_MODE		0400
#endif

#if IS_ENABLED(CONFIG_USB4_DEBUGFS_MARGINING)
/**
 * struct tb_margining - Lane margining support
 * @port: USB4 port through which the margining operations are run
 * @target: Sideband target
 * @index: Retimer index if taget is %USB4_SB_TARGET_RETIMER
 * @dev: Pointer to the device that is the target (USB4 port or retimer)
 * @caps: Port lane margining capabilities
 * @results: Last lane margining results
 * @lanes: %0, %1 or %7 (all)
 * @min_ber_level: Minimum supported BER level contour value
 * @max_ber_level: Maximum supported BER level contour value
 * @ber_level: Current BER level contour value
 * @voltage_steps: Number of mandatory voltage steps
 * @max_voltage_offset: Maximum mandatory voltage offset (in mV)
 * @voltage_steps_optional_range: Number of voltage steps for optional range
 * @max_voltage_offset_optional_range: Maximum voltage offset for the optional
 *					range (in mV).
 * @time_steps: Number of time margin steps
 * @max_time_offset: Maximum time margin offset (in mUI)
 * @voltage_time_offset: Offset for voltage / time for software margining
 * @dwell_time: Dwell time for software margining (in ms)
 * @error_counter: Error counter operation for software margining
 * @optional_voltage_offset_range: Enable optional extended voltage range
 * @software: %true if software margining is used instead of hardware
 * @time: %true if time margining is used instead of voltage
 * @right_high: %false if left/low margin test is performed, %true if
 *		right/high
 */
struct tb_margining {
	struct tb_port *port;
	enum usb4_sb_target target;
	u8 index;
	struct device *dev;
	u32 caps[2];
	u32 results[2];
	unsigned int lanes;
	unsigned int min_ber_level;
	unsigned int max_ber_level;
	unsigned int ber_level;
	unsigned int voltage_steps;
	unsigned int max_voltage_offset;
	unsigned int voltage_steps_optional_range;
	unsigned int max_voltage_offset_optional_range;
	unsigned int time_steps;
	unsigned int max_time_offset;
	unsigned int voltage_time_offset;
	unsigned int dwell_time;
	enum usb4_margin_sw_error_counter error_counter;
	bool optional_voltage_offset_range;
	bool software;
	bool time;
	bool right_high;
};

static int margining_modify_error_counter(struct tb_margining *margining,
	u32 lanes, enum usb4_margin_sw_error_counter error_counter)
{
	struct usb4_port_margining_params params = { 0 };
	struct tb_port *port = margining->port;
	u32 result;

	if (error_counter != USB4_MARGIN_SW_ERROR_COUNTER_CLEAR &&
	    error_counter != USB4_MARGIN_SW_ERROR_COUNTER_STOP)
		return -EOPNOTSUPP;

	params.error_counter = error_counter;
	params.lanes = lanes;

	return usb4_port_sw_margin(port, margining->target, margining->index,
				   &params, &result);
}

static bool supports_software(const struct tb_margining *margining)
{
	return margining->caps[0] & USB4_MARGIN_CAP_0_MODES_SW;
}

static bool supports_hardware(const struct tb_margining *margining)
{
	return margining->caps[0] & USB4_MARGIN_CAP_0_MODES_HW;
}

static bool both_lanes(const struct tb_margining *margining)
{
	return margining->caps[0] & USB4_MARGIN_CAP_0_2_LANES;
}

static unsigned int
independent_voltage_margins(const struct tb_margining *margining)
{
	return FIELD_GET(USB4_MARGIN_CAP_0_VOLTAGE_INDP_MASK, margining->caps[0]);
}

static bool supports_time(const struct tb_margining *margining)
{
	return margining->caps[0] & USB4_MARGIN_CAP_0_TIME;
}

/* Only applicable if supports_time() returns true */
static unsigned int
independent_time_margins(const struct tb_margining *margining)
{
	return FIELD_GET(USB4_MARGIN_CAP_1_TIME_INDP_MASK, margining->caps[1]);
}

static bool
supports_optional_voltage_offset_range(const struct tb_margining *margining)
{
	return margining->caps[0] & USB4_MARGIN_CAP_0_OPT_VOLTAGE_SUPPORT;
}

static ssize_t
margining_ber_level_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	unsigned int val;
	int ret = 0;
	char *buf;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	if (margining->software) {
		ret = -EINVAL;
		goto out_unlock;
	}

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		goto out_unlock;
	}

	buf[count - 1] = '\0';

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		goto out_free;

	if (val < margining->min_ber_level ||
	    val > margining->max_ber_level) {
		ret = -EINVAL;
		goto out_free;
	}

	margining->ber_level = val;

out_free:
	free_page((unsigned long)buf);
out_unlock:
	mutex_unlock(&tb->lock);

	return ret < 0 ? ret : count;
}

static void ber_level_show(struct seq_file *s, unsigned int val)
{
	if (val % 2)
		seq_printf(s, "3 * 1e%d (%u)\n", -12 + (val + 1) / 2, val);
	else
		seq_printf(s, "1e%d (%u)\n", -12 + val / 2, val);
}

static int margining_ber_level_show(struct seq_file *s, void *not_used)
{
	const struct tb_margining *margining = s->private;

	if (margining->software)
		return -EINVAL;
	ber_level_show(s, margining->ber_level);
	return 0;
}
DEBUGFS_ATTR_RW(margining_ber_level);

static int margining_caps_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	u32 cap0, cap1;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	/* Dump the raw caps first */
	cap0 = margining->caps[0];
	seq_printf(s, "0x%08x\n", cap0);
	cap1 = margining->caps[1];
	seq_printf(s, "0x%08x\n", cap1);

	seq_printf(s, "# software margining: %s\n",
		   supports_software(margining) ? "yes" : "no");
	if (supports_hardware(margining)) {
		seq_puts(s, "# hardware margining: yes\n");
		seq_puts(s, "# minimum BER level contour: ");
		ber_level_show(s, margining->min_ber_level);
		seq_puts(s, "# maximum BER level contour: ");
		ber_level_show(s, margining->max_ber_level);
	} else {
		seq_puts(s, "# hardware margining: no\n");
	}

	seq_printf(s, "# both lanes simultaneously: %s\n",
		  both_lanes(margining) ? "yes" : "no");
	seq_printf(s, "# voltage margin steps: %u\n",
		   margining->voltage_steps);
	seq_printf(s, "# maximum voltage offset: %u mV\n",
		   margining->max_voltage_offset);
	seq_printf(s, "# optional voltage offset range support: %s\n",
		   str_yes_no(supports_optional_voltage_offset_range(margining)));
	if (supports_optional_voltage_offset_range(margining)) {
		seq_printf(s, "# voltage margin steps, optional range: %u\n",
			   margining->voltage_steps_optional_range);
		seq_printf(s, "# maximum voltage offset, optional range: %u mV\n",
			   margining->max_voltage_offset_optional_range);
	}

	switch (independent_voltage_margins(margining)) {
	case USB4_MARGIN_CAP_0_VOLTAGE_MIN:
		seq_puts(s, "# returns minimum between high and low voltage margins\n");
		break;
	case USB4_MARGIN_CAP_0_VOLTAGE_HL:
		seq_puts(s, "# returns high or low voltage margin\n");
		break;
	case USB4_MARGIN_CAP_0_VOLTAGE_BOTH:
		seq_puts(s, "# returns both high and low margins\n");
		break;
	}

	if (supports_time(margining)) {
		seq_puts(s, "# time margining: yes\n");
		seq_printf(s, "# time margining is destructive: %s\n",
			   cap1 & USB4_MARGIN_CAP_1_TIME_DESTR ? "yes" : "no");

		switch (independent_time_margins(margining)) {
		case USB4_MARGIN_CAP_1_TIME_MIN:
			seq_puts(s, "# returns minimum between left and right time margins\n");
			break;
		case USB4_MARGIN_CAP_1_TIME_LR:
			seq_puts(s, "# returns left or right margin\n");
			break;
		case USB4_MARGIN_CAP_1_TIME_BOTH:
			seq_puts(s, "# returns both left and right margins\n");
			break;
		}

		seq_printf(s, "# time margin steps: %u\n",
			   margining->time_steps);
		seq_printf(s, "# maximum time offset: %u mUI\n",
			   margining->max_time_offset);
	} else {
		seq_puts(s, "# time margining: no\n");
	}

	mutex_unlock(&tb->lock);
	return 0;
}
DEBUGFS_ATTR_RO(margining_caps);

static ssize_t
margining_lanes_write(struct file *file, const char __user *user_buf,
		      size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	int ret = 0;
	char *buf;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	buf[count - 1] = '\0';

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_free;
	}

	if (!strcmp(buf, "0")) {
		margining->lanes = 0;
	} else if (!strcmp(buf, "1")) {
		margining->lanes = 1;
	} else if (!strcmp(buf, "all")) {
		/* Needs to be supported */
		if (both_lanes(margining))
			margining->lanes = 7;
		else
			ret = -EINVAL;
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&tb->lock);

out_free:
	free_page((unsigned long)buf);
	return ret < 0 ? ret : count;
}

static int margining_lanes_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	unsigned int lanes;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	lanes = margining->lanes;
	if (both_lanes(margining)) {
		if (!lanes)
			seq_puts(s, "[0] 1 all\n");
		else if (lanes == 1)
			seq_puts(s, "0 [1] all\n");
		else
			seq_puts(s, "0 1 [all]\n");
	} else {
		if (!lanes)
			seq_puts(s, "[0] 1\n");
		else
			seq_puts(s, "0 [1]\n");
	}

	mutex_unlock(&tb->lock);
	return 0;
}
DEBUGFS_ATTR_RW(margining_lanes);

static ssize_t
margining_voltage_time_offset_write(struct file *file,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	unsigned int max_margin;
	unsigned int val;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 10, &val);
	if (ret)
		return ret;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		if (!margining->software)
			return -EOPNOTSUPP;

		if (margining->time)
			max_margin = margining->time_steps;
		else
			if (margining->optional_voltage_offset_range)
				max_margin = margining->voltage_steps_optional_range;
			else
				max_margin = margining->voltage_steps;

		margining->voltage_time_offset = clamp(val, 0, max_margin);
	}

	return count;
}

static int margining_voltage_time_offset_show(struct seq_file *s,
					      void *not_used)
{
	const struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		if (!margining->software)
			return -EOPNOTSUPP;

		seq_printf(s, "%d\n", margining->voltage_time_offset);
	}

	return 0;
}
DEBUGFS_ATTR_RW(margining_voltage_time_offset);

static ssize_t
margining_error_counter_write(struct file *file, const char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	enum usb4_margin_sw_error_counter error_counter;
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	char *buf;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	buf[count - 1] = '\0';

	if (!strcmp(buf, "nop"))
		error_counter = USB4_MARGIN_SW_ERROR_COUNTER_NOP;
	else if (!strcmp(buf, "clear"))
		error_counter = USB4_MARGIN_SW_ERROR_COUNTER_CLEAR;
	else if (!strcmp(buf, "start"))
		error_counter = USB4_MARGIN_SW_ERROR_COUNTER_START;
	else if (!strcmp(buf, "stop"))
		error_counter = USB4_MARGIN_SW_ERROR_COUNTER_STOP;
	else
		return -EINVAL;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		if (!margining->software)
			return -EOPNOTSUPP;

		margining->error_counter = error_counter;
	}

	return count;
}

static int margining_error_counter_show(struct seq_file *s, void *not_used)
{
	const struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		if (!margining->software)
			return -EOPNOTSUPP;

		switch (margining->error_counter) {
		case USB4_MARGIN_SW_ERROR_COUNTER_NOP:
			seq_puts(s, "[nop] clear start stop\n");
			break;
		case USB4_MARGIN_SW_ERROR_COUNTER_CLEAR:
			seq_puts(s, "nop [clear] start stop\n");
			break;
		case USB4_MARGIN_SW_ERROR_COUNTER_START:
			seq_puts(s, "nop clear [start] stop\n");
			break;
		case USB4_MARGIN_SW_ERROR_COUNTER_STOP:
			seq_puts(s, "nop clear start [stop]\n");
			break;
		}
	}

	return 0;
}
DEBUGFS_ATTR_RW(margining_error_counter);

static ssize_t
margining_dwell_time_write(struct file *file, const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	unsigned int val;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 10, &val);
	if (ret)
		return ret;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		if (!margining->software)
			return -EOPNOTSUPP;

		margining->dwell_time = clamp(val, MIN_DWELL_TIME, MAX_DWELL_TIME);
	}

	return count;
}

static int margining_dwell_time_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		if (!margining->software)
			return -EOPNOTSUPP;

		seq_printf(s, "%d\n", margining->dwell_time);
	}

	return 0;
}
DEBUGFS_ATTR_RW(margining_dwell_time);

static ssize_t
margining_optional_voltage_offset_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	bool val;
	int ret;

	ret = kstrtobool_from_user(user_buf, count, &val);
	if (ret)
		return ret;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		margining->optional_voltage_offset_range = val;
	}

	return count;
}

static int margining_optional_voltage_offset_show(struct seq_file *s,
						  void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &tb->lock) {
		seq_printf(s, "%u\n", margining->optional_voltage_offset_range);
	}

	return 0;
}
DEBUGFS_ATTR_RW(margining_optional_voltage_offset);

static ssize_t margining_mode_write(struct file *file,
				   const char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	int ret = 0;
	char *buf;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	buf[count - 1] = '\0';

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_free;
	}

	if (!strcmp(buf, "software")) {
		if (supports_software(margining))
			margining->software = true;
		else
			ret = -EINVAL;
	} else if (!strcmp(buf, "hardware")) {
		if (supports_hardware(margining))
			margining->software = false;
		else
			ret = -EINVAL;
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&tb->lock);

out_free:
	free_page((unsigned long)buf);
	return ret ? ret : count;
}

static int margining_mode_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	const char *space = "";

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	if (supports_software(margining)) {
		if (margining->software)
			seq_puts(s, "[software]");
		else
			seq_puts(s, "software");
		space = " ";
	}
	if (supports_hardware(margining)) {
		if (margining->software)
			seq_printf(s, "%shardware", space);
		else
			seq_printf(s, "%s[hardware]", space);
	}

	mutex_unlock(&tb->lock);

	seq_puts(s, "\n");
	return 0;
}
DEBUGFS_ATTR_RW(margining_mode);

static int margining_run_sw(struct tb_margining *margining,
			    struct usb4_port_margining_params *params)
{
	u32 nsamples = margining->dwell_time / DWELL_SAMPLE_INTERVAL;
	int ret, i;

	ret = usb4_port_sw_margin(margining->port, margining->target, margining->index,
				  params, margining->results);
	if (ret)
		goto out_stop;

	for (i = 0; i <= nsamples; i++) {
		u32 errors = 0;

		ret = usb4_port_sw_margin_errors(margining->port, margining->target,
						 margining->index, &margining->results[1]);
		if (ret)
			break;

		if (margining->lanes == USB4_MARGIN_SW_LANE_0)
			errors = FIELD_GET(USB4_MARGIN_SW_ERR_COUNTER_LANE_0_MASK,
					   margining->results[1]);
		else if (margining->lanes == USB4_MARGIN_SW_LANE_1)
			errors = FIELD_GET(USB4_MARGIN_SW_ERR_COUNTER_LANE_1_MASK,
					   margining->results[1]);
		else if (margining->lanes == USB4_MARGIN_SW_ALL_LANES)
			errors = margining->results[1];

		/* Any errors stop the test */
		if (errors)
			break;

		fsleep(DWELL_SAMPLE_INTERVAL * USEC_PER_MSEC);
	}

out_stop:
	/*
	 * Stop the counters but don't clear them to allow the
	 * different error counter configurations.
	 */
	margining_modify_error_counter(margining, margining->lanes,
				       USB4_MARGIN_SW_ERROR_COUNTER_STOP);
	return ret;
}

static int margining_run_write(void *data, u64 val)
{
	struct tb_margining *margining = data;
	struct tb_port *port = margining->port;
	struct device *dev = margining->dev;
	struct tb_switch *sw = port->sw;
	struct tb_switch *down_sw;
	struct tb *tb = sw->tb;
	int ret, clx;

	if (val != 1)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_rpm_put;
	}

	if (tb_is_upstream_port(port))
		down_sw = sw;
	else if (port->remote)
		down_sw = port->remote->sw;
	else
		down_sw = NULL;

	if (down_sw) {
		/*
		 * CL states may interfere with lane margining so
		 * disable them temporarily now.
		 */
		ret = tb_switch_clx_disable(down_sw);
		if (ret < 0) {
			tb_sw_warn(down_sw, "failed to disable CL states\n");
			goto out_unlock;
		}
		clx = ret;
	}

	/* Clear the results */
	memset(margining->results, 0, sizeof(margining->results));

	if (margining->software) {
		struct usb4_port_margining_params params = {
			.error_counter = USB4_MARGIN_SW_ERROR_COUNTER_CLEAR,
			.lanes = margining->lanes,
			.time = margining->time,
			.voltage_time_offset = margining->voltage_time_offset,
			.right_high = margining->right_high,
			.optional_voltage_offset_range = margining->optional_voltage_offset_range,
		};

		tb_port_dbg(port,
			    "running software %s lane margining for %s lanes %u\n",
			    margining->time ? "time" : "voltage", dev_name(dev),
			    margining->lanes);

		ret = margining_run_sw(margining, &params);
	} else {
		struct usb4_port_margining_params params = {
			.ber_level = margining->ber_level,
			.lanes = margining->lanes,
			.time = margining->time,
			.right_high = margining->right_high,
			.optional_voltage_offset_range = margining->optional_voltage_offset_range,
		};

		tb_port_dbg(port,
			    "running hardware %s lane margining for %s lanes %u\n",
			    margining->time ? "time" : "voltage", dev_name(dev),
			    margining->lanes);

		ret = usb4_port_hw_margin(port, margining->target, margining->index, &params,
					  margining->results);
	}

	if (down_sw)
		tb_switch_clx_enable(down_sw, clx);
out_unlock:
	mutex_unlock(&tb->lock);
out_rpm_put:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(margining_run_fops, NULL, margining_run_write,
			 "%llu\n");

static ssize_t margining_results_write(struct file *file,
				       const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	/* Just clear the results */
	margining->results[0] = 0;
	margining->results[1] = 0;

	if (margining->software) {
		/* Clear the error counters */
		margining_modify_error_counter(margining,
					       USB4_MARGIN_SW_ALL_LANES,
					       USB4_MARGIN_SW_ERROR_COUNTER_CLEAR);
	}

	mutex_unlock(&tb->lock);
	return count;
}

static void voltage_margin_show(struct seq_file *s,
				const struct tb_margining *margining, u8 val)
{
	unsigned int tmp, voltage;

	tmp = FIELD_GET(USB4_MARGIN_HW_RES_1_MARGIN_MASK, val);
	voltage = tmp * margining->max_voltage_offset / margining->voltage_steps;
	seq_printf(s, "%u mV (%u)", voltage, tmp);
	if (val & USB4_MARGIN_HW_RES_1_EXCEEDS)
		seq_puts(s, " exceeds maximum");
	seq_puts(s, "\n");
	if (margining->optional_voltage_offset_range)
		seq_puts(s, " optional voltage offset range enabled\n");
}

static void time_margin_show(struct seq_file *s,
			     const struct tb_margining *margining, u8 val)
{
	unsigned int tmp, interval;

	tmp = FIELD_GET(USB4_MARGIN_HW_RES_1_MARGIN_MASK, val);
	interval = tmp * margining->max_time_offset / margining->time_steps;
	seq_printf(s, "%u mUI (%u)", interval, tmp);
	if (val & USB4_MARGIN_HW_RES_1_EXCEEDS)
		seq_puts(s, " exceeds maximum");
	seq_puts(s, "\n");
}

static int margining_results_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	/* Dump the raw results first */
	seq_printf(s, "0x%08x\n", margining->results[0]);
	/* Only the hardware margining has two result dwords */
	if (!margining->software) {
		unsigned int val;

		seq_printf(s, "0x%08x\n", margining->results[1]);

		if (margining->time) {
			if (!margining->lanes || margining->lanes == 7) {
				val = margining->results[1];
				seq_puts(s, "# lane 0 right time margin: ");
				time_margin_show(s, margining, val);
				val = margining->results[1] >>
					USB4_MARGIN_HW_RES_1_L0_LL_MARGIN_SHIFT;
				seq_puts(s, "# lane 0 left time margin: ");
				time_margin_show(s, margining, val);
			}
			if (margining->lanes == 1 || margining->lanes == 7) {
				val = margining->results[1] >>
					USB4_MARGIN_HW_RES_1_L1_RH_MARGIN_SHIFT;
				seq_puts(s, "# lane 1 right time margin: ");
				time_margin_show(s, margining, val);
				val = margining->results[1] >>
					USB4_MARGIN_HW_RES_1_L1_LL_MARGIN_SHIFT;
				seq_puts(s, "# lane 1 left time margin: ");
				time_margin_show(s, margining, val);
			}
		} else {
			if (!margining->lanes || margining->lanes == 7) {
				val = margining->results[1];
				seq_puts(s, "# lane 0 high voltage margin: ");
				voltage_margin_show(s, margining, val);
				val = margining->results[1] >>
					USB4_MARGIN_HW_RES_1_L0_LL_MARGIN_SHIFT;
				seq_puts(s, "# lane 0 low voltage margin: ");
				voltage_margin_show(s, margining, val);
			}
			if (margining->lanes == 1 || margining->lanes == 7) {
				val = margining->results[1] >>
					USB4_MARGIN_HW_RES_1_L1_RH_MARGIN_SHIFT;
				seq_puts(s, "# lane 1 high voltage margin: ");
				voltage_margin_show(s, margining, val);
				val = margining->results[1] >>
					USB4_MARGIN_HW_RES_1_L1_LL_MARGIN_SHIFT;
				seq_puts(s, "# lane 1 low voltage margin: ");
				voltage_margin_show(s, margining, val);
			}
		}
	} else {
		u32 lane_errors, result;

		seq_printf(s, "0x%08x\n", margining->results[1]);
		result = FIELD_GET(USB4_MARGIN_SW_LANES_MASK, margining->results[0]);

		if (result == USB4_MARGIN_SW_LANE_0 ||
		    result == USB4_MARGIN_SW_ALL_LANES) {
			lane_errors = FIELD_GET(USB4_MARGIN_SW_ERR_COUNTER_LANE_0_MASK,
						margining->results[1]);
			seq_printf(s, "# lane 0 errors: %u\n", lane_errors);
		}
		if (result == USB4_MARGIN_SW_LANE_1 ||
		    result == USB4_MARGIN_SW_ALL_LANES) {
			lane_errors = FIELD_GET(USB4_MARGIN_SW_ERR_COUNTER_LANE_1_MASK,
						margining->results[1]);
			seq_printf(s, "# lane 1 errors: %u\n", lane_errors);
		}
	}

	mutex_unlock(&tb->lock);
	return 0;
}
DEBUGFS_ATTR_RW(margining_results);

static ssize_t margining_test_write(struct file *file,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	int ret = 0;
	char *buf;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	buf[count - 1] = '\0';

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_free;
	}

	if (!strcmp(buf, "time") && supports_time(margining))
		margining->time = true;
	else if (!strcmp(buf, "voltage"))
		margining->time = false;
	else
		ret = -EINVAL;

	mutex_unlock(&tb->lock);

out_free:
	free_page((unsigned long)buf);
	return ret ? ret : count;
}

static int margining_test_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	if (supports_time(margining)) {
		if (margining->time)
			seq_puts(s, "voltage [time]\n");
		else
			seq_puts(s, "[voltage] time\n");
	} else {
		seq_puts(s, "[voltage]\n");
	}

	mutex_unlock(&tb->lock);
	return 0;
}
DEBUGFS_ATTR_RW(margining_test);

static ssize_t margining_margin_write(struct file *file,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;
	int ret = 0;
	char *buf;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	buf[count - 1] = '\0';

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_free;
	}

	if (margining->time) {
		if (!strcmp(buf, "left"))
			margining->right_high = false;
		else if (!strcmp(buf, "right"))
			margining->right_high = true;
		else
			ret = -EINVAL;
	} else {
		if (!strcmp(buf, "low"))
			margining->right_high = false;
		else if (!strcmp(buf, "high"))
			margining->right_high = true;
		else
			ret = -EINVAL;
	}

	mutex_unlock(&tb->lock);

out_free:
	free_page((unsigned long)buf);
	return ret ? ret : count;
}

static int margining_margin_show(struct seq_file *s, void *not_used)
{
	struct tb_margining *margining = s->private;
	struct tb *tb = margining->port->sw->tb;

	if (mutex_lock_interruptible(&tb->lock))
		return -ERESTARTSYS;

	if (margining->time) {
		if (margining->right_high)
			seq_puts(s, "left [right]\n");
		else
			seq_puts(s, "[left] right\n");
	} else {
		if (margining->right_high)
			seq_puts(s, "low [high]\n");
		else
			seq_puts(s, "[low] high\n");
	}

	mutex_unlock(&tb->lock);
	return 0;
}
DEBUGFS_ATTR_RW(margining_margin);

static struct tb_margining *margining_alloc(struct tb_port *port,
					    struct device *dev,
					    enum usb4_sb_target target,
					    u8 index, struct dentry *parent)
{
	struct tb_margining *margining;
	struct dentry *dir;
	unsigned int val;
	int ret;

	margining = kzalloc(sizeof(*margining), GFP_KERNEL);
	if (!margining)
		return NULL;

	margining->port = port;
	margining->target = target;
	margining->index = index;
	margining->dev = dev;

	ret = usb4_port_margining_caps(port, target, index, margining->caps);
	if (ret) {
		kfree(margining);
		return NULL;
	}

	/* Set the initial mode */
	if (supports_software(margining))
		margining->software = true;

	val = FIELD_GET(USB4_MARGIN_CAP_0_VOLTAGE_STEPS_MASK, margining->caps[0]);
	margining->voltage_steps = val;
	val = FIELD_GET(USB4_MARGIN_CAP_0_MAX_VOLTAGE_OFFSET_MASK, margining->caps[0]);
	margining->max_voltage_offset = 74 + val * 2;

	if (supports_optional_voltage_offset_range(margining)) {
		val = FIELD_GET(USB4_MARGIN_CAP_0_VOLT_STEPS_OPT_MASK,
				margining->caps[0]);
		margining->voltage_steps_optional_range = val;
		val = FIELD_GET(USB4_MARGIN_CAP_1_MAX_VOLT_OFS_OPT_MASK,
				margining->caps[1]);
		margining->max_voltage_offset_optional_range = 74 + val * 2;
	}

	if (supports_time(margining)) {
		val = FIELD_GET(USB4_MARGIN_CAP_1_TIME_STEPS_MASK, margining->caps[1]);
		margining->time_steps = val;
		val = FIELD_GET(USB4_MARGIN_CAP_1_TIME_OFFSET_MASK, margining->caps[1]);
		/*
		 * Store it as mUI (milli Unit Interval) because we want
		 * to keep it as integer.
		 */
		margining->max_time_offset = 200 + 10 * val;
	}

	dir = debugfs_create_dir("margining", parent);
	if (supports_hardware(margining)) {
		val = FIELD_GET(USB4_MARGIN_CAP_1_MIN_BER_MASK, margining->caps[1]);
		margining->min_ber_level = val;
		val = FIELD_GET(USB4_MARGIN_CAP_1_MAX_BER_MASK, margining->caps[1]);
		margining->max_ber_level = val;

		/* Set the default to minimum */
		margining->ber_level = margining->min_ber_level;

		debugfs_create_file("ber_level_contour", 0400, dir, margining,
				    &margining_ber_level_fops);
	}
	debugfs_create_file("caps", 0400, dir, margining, &margining_caps_fops);
	debugfs_create_file("lanes", 0600, dir, margining, &margining_lanes_fops);
	debugfs_create_file("mode", 0600, dir, margining, &margining_mode_fops);
	debugfs_create_file("run", 0600, dir, margining, &margining_run_fops);
	debugfs_create_file("results", 0600, dir, margining,
			    &margining_results_fops);
	debugfs_create_file("test", 0600, dir, margining, &margining_test_fops);
	if (independent_voltage_margins(margining) == USB4_MARGIN_CAP_0_VOLTAGE_HL ||
	    (supports_time(margining) &&
	     independent_time_margins(margining) == USB4_MARGIN_CAP_1_TIME_LR))
		debugfs_create_file("margin", 0600, dir, margining,
				    &margining_margin_fops);

	margining->error_counter = USB4_MARGIN_SW_ERROR_COUNTER_CLEAR;
	margining->dwell_time = MIN_DWELL_TIME;

	if (supports_optional_voltage_offset_range(margining))
		debugfs_create_file("optional_voltage_offset", DEBUGFS_MODE, dir, margining,
				    &margining_optional_voltage_offset_fops);

	if (supports_software(margining)) {
		debugfs_create_file("voltage_time_offset", DEBUGFS_MODE, dir, margining,
				    &margining_voltage_time_offset_fops);
		debugfs_create_file("error_counter", DEBUGFS_MODE, dir, margining,
				    &margining_error_counter_fops);
		debugfs_create_file("dwell_time", DEBUGFS_MODE, dir, margining,
				    &margining_dwell_time_fops);
	}
	return margining;
}

static void margining_port_init(struct tb_port *port)
{
	struct dentry *parent;
	char dir_name[10];

	if (!port->usb4)
		return;

	snprintf(dir_name, sizeof(dir_name), "port%d", port->port);
	parent = debugfs_lookup(dir_name, port->sw->debugfs_dir);
	port->usb4->margining = margining_alloc(port, &port->usb4->dev,
						USB4_SB_TARGET_ROUTER, 0,
						parent);
}

static void margining_port_remove(struct tb_port *port)
{
	struct dentry *parent;
	char dir_name[10];

	if (!port->usb4)
		return;

	snprintf(dir_name, sizeof(dir_name), "port%d", port->port);
	parent = debugfs_lookup(dir_name, port->sw->debugfs_dir);
	if (parent)
		debugfs_lookup_and_remove("margining", parent);

	kfree(port->usb4->margining);
	port->usb4->margining = NULL;
}

static void margining_switch_init(struct tb_switch *sw)
{
	struct tb_port *upstream, *downstream;
	struct tb_switch *parent_sw;
	u64 route = tb_route(sw);

	if (!route)
		return;

	upstream = tb_upstream_port(sw);
	parent_sw = tb_switch_parent(sw);
	downstream = tb_port_at(route, parent_sw);

	margining_port_init(downstream);
	margining_port_init(upstream);
}

static void margining_switch_remove(struct tb_switch *sw)
{
	struct tb_port *upstream, *downstream;
	struct tb_switch *parent_sw;
	u64 route = tb_route(sw);

	if (!route)
		return;

	upstream = tb_upstream_port(sw);
	parent_sw = tb_switch_parent(sw);
	downstream = tb_port_at(route, parent_sw);

	margining_port_remove(upstream);
	margining_port_remove(downstream);
}

static void margining_xdomain_init(struct tb_xdomain *xd)
{
	struct tb_switch *parent_sw;
	struct tb_port *downstream;

	parent_sw = tb_xdomain_parent(xd);
	downstream = tb_port_at(xd->route, parent_sw);

	margining_port_init(downstream);
}

static void margining_xdomain_remove(struct tb_xdomain *xd)
{
	struct tb_switch *parent_sw;
	struct tb_port *downstream;

	parent_sw = tb_xdomain_parent(xd);
	downstream = tb_port_at(xd->route, parent_sw);
	margining_port_remove(downstream);
}

static void margining_retimer_init(struct tb_retimer *rt, struct dentry *debugfs_dir)
{
	rt->margining = margining_alloc(rt->port, &rt->dev,
					USB4_SB_TARGET_RETIMER, rt->index,
					debugfs_dir);
}

static void margining_retimer_remove(struct tb_retimer *rt)
{
	kfree(rt->margining);
	rt->margining = NULL;
}
#else
static inline void margining_switch_init(struct tb_switch *sw) { }
static inline void margining_switch_remove(struct tb_switch *sw) { }
static inline void margining_xdomain_init(struct tb_xdomain *xd) { }
static inline void margining_xdomain_remove(struct tb_xdomain *xd) { }
static inline void margining_retimer_init(struct tb_retimer *rt,
					  struct dentry *debugfs_dir) { }
static inline void margining_retimer_remove(struct tb_retimer *rt) { }
#endif

static int port_clear_all_counters(struct tb_port *port)
{
	u32 *buf;
	int ret;

	buf = kcalloc(COUNTER_SET_LEN * port->config.max_counters, sizeof(u32),
		      GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = tb_port_write(port, buf, TB_CFG_COUNTERS, 0,
			    COUNTER_SET_LEN * port->config.max_counters);
	kfree(buf);

	return ret;
}

static ssize_t counters_write(struct file *file, const char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct tb_port *port = s->private;
	struct tb_switch *sw = port->sw;
	struct tb *tb = port->sw->tb;
	char *buf;
	int ret;

	buf = validate_and_copy_from_user(user_buf, &count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	/* If written delimiter only, clear all counters in one shot */
	if (buf[0] == '\n') {
		ret = port_clear_all_counters(port);
	} else  {
		char *line = buf;
		u32 val, offset;

		ret = -EINVAL;
		while (parse_line(&line, &offset, &val, 1, 4)) {
			ret = tb_port_write(port, &val, TB_CFG_COUNTERS,
					    offset, 1);
			if (ret)
				break;
		}
	}

	mutex_unlock(&tb->lock);

out:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);
	free_page((unsigned long)buf);

	return ret < 0 ? ret : count;
}

static void cap_show_by_dw(struct seq_file *s, struct tb_switch *sw,
			   struct tb_port *port, unsigned int cap,
			   unsigned int offset, u8 cap_id, u8 vsec_id,
			   int dwords)
{
	int i, ret;
	u32 data;

	for (i = 0; i < dwords; i++) {
		if (port)
			ret = tb_port_read(port, &data, TB_CFG_PORT, cap + offset + i, 1);
		else
			ret = tb_sw_read(sw, &data, TB_CFG_SWITCH, cap + offset + i, 1);
		if (ret) {
			seq_printf(s, "0x%04x <not accessible>\n", cap + offset + i);
			continue;
		}

		seq_printf(s, "0x%04x %4d 0x%02x 0x%02x 0x%08x\n", cap + offset + i,
			   offset + i, cap_id, vsec_id, data);
	}
}

static void cap_show(struct seq_file *s, struct tb_switch *sw,
		     struct tb_port *port, unsigned int cap, u8 cap_id,
		     u8 vsec_id, int length)
{
	int ret, offset = 0;

	while (length > 0) {
		int i, dwords = min(length, TB_MAX_CONFIG_RW_LENGTH);
		u32 data[TB_MAX_CONFIG_RW_LENGTH];

		if (port)
			ret = tb_port_read(port, data, TB_CFG_PORT, cap + offset,
					   dwords);
		else
			ret = tb_sw_read(sw, data, TB_CFG_SWITCH, cap + offset, dwords);
		if (ret) {
			cap_show_by_dw(s, sw, port, cap, offset, cap_id, vsec_id, length);
			return;
		}

		for (i = 0; i < dwords; i++) {
			seq_printf(s, "0x%04x %4d 0x%02x 0x%02x 0x%08x\n",
				   cap + offset + i, offset + i,
				   cap_id, vsec_id, data[i]);
		}

		length -= dwords;
		offset += dwords;
	}
}

static void port_cap_show(struct tb_port *port, struct seq_file *s,
			  unsigned int cap)
{
	struct tb_cap_any header;
	u8 vsec_id = 0;
	size_t length;
	int ret;

	ret = tb_port_read(port, &header, TB_CFG_PORT, cap, 1);
	if (ret) {
		seq_printf(s, "0x%04x <capability read failed>\n", cap);
		return;
	}

	switch (header.basic.cap) {
	case TB_PORT_CAP_PHY:
		length = PORT_CAP_LANE_LEN;
		break;

	case TB_PORT_CAP_TIME1:
		if (usb4_switch_version(port->sw) < 2)
			length = PORT_CAP_TMU_V1_LEN;
		else
			length = PORT_CAP_TMU_V2_LEN;
		break;

	case TB_PORT_CAP_POWER:
		length = PORT_CAP_POWER_LEN;
		break;

	case TB_PORT_CAP_ADAP:
		if (tb_port_is_pcie_down(port) || tb_port_is_pcie_up(port)) {
			if (usb4_switch_version(port->sw) < 2)
				length = PORT_CAP_V1_PCIE_LEN;
			else
				length = PORT_CAP_V2_PCIE_LEN;
		} else if (tb_port_is_dpin(port)) {
			if (usb4_switch_version(port->sw) < 2)
				length = PORT_CAP_DP_V1_LEN;
			else
				length = PORT_CAP_DP_V2_LEN;
		} else if (tb_port_is_dpout(port)) {
			length = PORT_CAP_DP_V1_LEN;
		} else if (tb_port_is_usb3_down(port) ||
			   tb_port_is_usb3_up(port)) {
			length = PORT_CAP_USB3_LEN;
		} else {
			seq_printf(s, "0x%04x <unsupported capability 0x%02x>\n",
				   cap, header.basic.cap);
			return;
		}
		break;

	case TB_PORT_CAP_VSE:
		if (!header.extended_short.length) {
			ret = tb_port_read(port, (u32 *)&header + 1, TB_CFG_PORT,
					   cap + 1, 1);
			if (ret) {
				seq_printf(s, "0x%04x <capability read failed>\n",
					   cap + 1);
				return;
			}
			length = header.extended_long.length;
			vsec_id = header.extended_short.vsec_id;
		} else {
			length = header.extended_short.length;
			vsec_id = header.extended_short.vsec_id;
		}
		break;

	case TB_PORT_CAP_USB4:
		length = PORT_CAP_USB4_LEN;
		break;

	default:
		seq_printf(s, "0x%04x <unsupported capability 0x%02x>\n",
			   cap, header.basic.cap);
		return;
	}

	cap_show(s, NULL, port, cap, header.basic.cap, vsec_id, length);
}

static void port_caps_show(struct tb_port *port, struct seq_file *s)
{
	int cap;

	cap = tb_port_next_cap(port, 0);
	while (cap > 0) {
		port_cap_show(port, s, cap);
		cap = tb_port_next_cap(port, cap);
	}
}

static int port_basic_regs_show(struct tb_port *port, struct seq_file *s)
{
	u32 data[PORT_CAP_BASIC_LEN];
	int ret, i;

	ret = tb_port_read(port, data, TB_CFG_PORT, 0, ARRAY_SIZE(data));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(data); i++)
		seq_printf(s, "0x%04x %4d 0x00 0x00 0x%08x\n", i, i, data[i]);

	return 0;
}

static int port_regs_show(struct seq_file *s, void *not_used)
{
	struct tb_port *port = s->private;
	struct tb_switch *sw = port->sw;
	struct tb *tb = sw->tb;
	int ret;

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_rpm_put;
	}

	seq_puts(s, "# offset relative_offset cap_id vs_cap_id value\n");

	ret = port_basic_regs_show(port, s);
	if (ret)
		goto out_unlock;

	port_caps_show(port, s);

out_unlock:
	mutex_unlock(&tb->lock);
out_rpm_put:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);

	return ret;
}
DEBUGFS_ATTR_RW(port_regs);

static void switch_cap_show(struct tb_switch *sw, struct seq_file *s,
			    unsigned int cap)
{
	struct tb_cap_any header;
	int ret, length;
	u8 vsec_id = 0;

	ret = tb_sw_read(sw, &header, TB_CFG_SWITCH, cap, 1);
	if (ret) {
		seq_printf(s, "0x%04x <capability read failed>\n", cap);
		return;
	}

	if (header.basic.cap == TB_SWITCH_CAP_VSE) {
		if (!header.extended_short.length) {
			ret = tb_sw_read(sw, (u32 *)&header + 1, TB_CFG_SWITCH,
					 cap + 1, 1);
			if (ret) {
				seq_printf(s, "0x%04x <capability read failed>\n",
					   cap + 1);
				return;
			}
			length = header.extended_long.length;
		} else {
			length = header.extended_short.length;
		}
		vsec_id = header.extended_short.vsec_id;
	} else {
		if (header.basic.cap == TB_SWITCH_CAP_TMU) {
			length = SWITCH_CAP_TMU_LEN;
		} else  {
			seq_printf(s, "0x%04x <unknown capability 0x%02x>\n",
				   cap, header.basic.cap);
			return;
		}
	}

	cap_show(s, sw, NULL, cap, header.basic.cap, vsec_id, length);
}

static void switch_caps_show(struct tb_switch *sw, struct seq_file *s)
{
	int cap;

	cap = tb_switch_next_cap(sw, 0);
	while (cap > 0) {
		switch_cap_show(sw, s, cap);
		cap = tb_switch_next_cap(sw, cap);
	}
}

static int switch_basic_regs_show(struct tb_switch *sw, struct seq_file *s)
{
	u32 data[SWITCH_CAP_BASIC_LEN];
	size_t dwords;
	int ret, i;

	/* Only USB4 has the additional registers */
	if (tb_switch_is_usb4(sw))
		dwords = ARRAY_SIZE(data);
	else
		dwords = 5;

	ret = tb_sw_read(sw, data, TB_CFG_SWITCH, 0, dwords);
	if (ret)
		return ret;

	for (i = 0; i < dwords; i++)
		seq_printf(s, "0x%04x %4d 0x00 0x00 0x%08x\n", i, i, data[i]);

	return 0;
}

static int switch_regs_show(struct seq_file *s, void *not_used)
{
	struct tb_switch *sw = s->private;
	struct tb *tb = sw->tb;
	int ret;

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_rpm_put;
	}

	seq_puts(s, "# offset relative_offset cap_id vs_cap_id value\n");

	ret = switch_basic_regs_show(sw, s);
	if (ret)
		goto out_unlock;

	switch_caps_show(sw, s);

out_unlock:
	mutex_unlock(&tb->lock);
out_rpm_put:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);

	return ret;
}
DEBUGFS_ATTR_RW(switch_regs);

static int path_show_one(struct tb_port *port, struct seq_file *s, int hopid)
{
	u32 data[PATH_LEN];
	int ret, i;

	ret = tb_port_read(port, data, TB_CFG_HOPS, hopid * PATH_LEN,
			   ARRAY_SIZE(data));
	if (ret) {
		seq_printf(s, "0x%04x <not accessible>\n", hopid * PATH_LEN);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		seq_printf(s, "0x%04x %4d 0x%02x 0x%08x\n",
			   hopid * PATH_LEN + i, i, hopid, data[i]);
	}

	return 0;
}

static int path_show(struct seq_file *s, void *not_used)
{
	struct tb_port *port = s->private;
	struct tb_switch *sw = port->sw;
	struct tb *tb = sw->tb;
	int start, i, ret = 0;

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_rpm_put;
	}

	seq_puts(s, "# offset relative_offset in_hop_id value\n");

	/* NHI and lane adapters have entry for path 0 */
	if (tb_port_is_null(port) || tb_port_is_nhi(port)) {
		ret = path_show_one(port, s, 0);
		if (ret)
			goto out_unlock;
	}

	start = tb_port_is_nhi(port) ? 1 : TB_PATH_MIN_HOPID;

	for (i = start; i <= port->config.max_in_hop_id; i++) {
		ret = path_show_one(port, s, i);
		if (ret)
			break;
	}

out_unlock:
	mutex_unlock(&tb->lock);
out_rpm_put:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);

	return ret;
}
DEBUGFS_ATTR_RO(path);

static int counter_set_regs_show(struct tb_port *port, struct seq_file *s,
				 int counter)
{
	u32 data[COUNTER_SET_LEN];
	int ret, i;

	ret = tb_port_read(port, data, TB_CFG_COUNTERS,
			   counter * COUNTER_SET_LEN, ARRAY_SIZE(data));
	if (ret) {
		seq_printf(s, "0x%04x <not accessible>\n",
			   counter * COUNTER_SET_LEN);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		seq_printf(s, "0x%04x %4d 0x%02x 0x%08x\n",
			   counter * COUNTER_SET_LEN + i, i, counter, data[i]);
	}

	return 0;
}

static int counters_show(struct seq_file *s, void *not_used)
{
	struct tb_port *port = s->private;
	struct tb_switch *sw = port->sw;
	struct tb *tb = sw->tb;
	int i, ret = 0;

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	seq_puts(s, "# offset relative_offset counter_id value\n");

	for (i = 0; i < port->config.max_counters; i++) {
		ret = counter_set_regs_show(port, s, i);
		if (ret)
			break;
	}

	mutex_unlock(&tb->lock);

out:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);

	return ret;
}
DEBUGFS_ATTR_RW(counters);

static int sb_regs_show(struct tb_port *port, const struct sb_reg *sb_regs,
			size_t size, enum usb4_sb_target target, u8 index,
			struct seq_file *s)
{
	int ret, i;

	seq_puts(s, "# register value\n");

	for (i = 0; i < size; i++) {
		const struct sb_reg *regs = &sb_regs[i];
		u8 data[64];
		int j;

		memset(data, 0, sizeof(data));
		ret = usb4_port_sb_read(port, target, index, regs->reg, data,
					regs->size);
		if (ret)
			return ret;

		seq_printf(s, "0x%02x", regs->reg);
		for (j = 0; j < regs->size; j++)
			seq_printf(s, " 0x%02x", data[j]);
		seq_puts(s, "\n");
	}

	return 0;
}

static int port_sb_regs_show(struct seq_file *s, void *not_used)
{
	struct tb_port *port = s->private;
	struct tb_switch *sw = port->sw;
	struct tb *tb = sw->tb;
	int ret;

	pm_runtime_get_sync(&sw->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_rpm_put;
	}

	ret = sb_regs_show(port, port_sb_regs, ARRAY_SIZE(port_sb_regs),
			   USB4_SB_TARGET_ROUTER, 0, s);

	mutex_unlock(&tb->lock);
out_rpm_put:
	pm_runtime_mark_last_busy(&sw->dev);
	pm_runtime_put_autosuspend(&sw->dev);

	return ret;
}
DEBUGFS_ATTR_RW(port_sb_regs);

/**
 * tb_switch_debugfs_init() - Add debugfs entries for router
 * @sw: Pointer to the router
 *
 * Adds debugfs directories and files for given router.
 */
void tb_switch_debugfs_init(struct tb_switch *sw)
{
	struct dentry *debugfs_dir;
	struct tb_port *port;

	debugfs_dir = debugfs_create_dir(dev_name(&sw->dev), tb_debugfs_root);
	sw->debugfs_dir = debugfs_dir;
	debugfs_create_file("regs", DEBUGFS_MODE, debugfs_dir, sw,
			    &switch_regs_fops);

	tb_switch_for_each_port(sw, port) {
		struct dentry *debugfs_dir;
		char dir_name[10];

		if (port->disabled)
			continue;
		if (port->config.type == TB_TYPE_INACTIVE)
			continue;

		snprintf(dir_name, sizeof(dir_name), "port%d", port->port);
		debugfs_dir = debugfs_create_dir(dir_name, sw->debugfs_dir);
		debugfs_create_file("regs", DEBUGFS_MODE, debugfs_dir,
				    port, &port_regs_fops);
		debugfs_create_file("path", 0400, debugfs_dir, port,
				    &path_fops);
		if (port->config.counters_support)
			debugfs_create_file("counters", 0600, debugfs_dir, port,
					    &counters_fops);
		if (port->usb4)
			debugfs_create_file("sb_regs", DEBUGFS_MODE, debugfs_dir,
					    port, &port_sb_regs_fops);
	}

	margining_switch_init(sw);
}

/**
 * tb_switch_debugfs_remove() - Remove all router debugfs entries
 * @sw: Pointer to the router
 *
 * Removes all previously added debugfs entries under this router.
 */
void tb_switch_debugfs_remove(struct tb_switch *sw)
{
	margining_switch_remove(sw);
	debugfs_remove_recursive(sw->debugfs_dir);
}

void tb_xdomain_debugfs_init(struct tb_xdomain *xd)
{
	margining_xdomain_init(xd);
}

void tb_xdomain_debugfs_remove(struct tb_xdomain *xd)
{
	margining_xdomain_remove(xd);
}

/**
 * tb_service_debugfs_init() - Add debugfs directory for service
 * @svc: Thunderbolt service pointer
 *
 * Adds debugfs directory for service.
 */
void tb_service_debugfs_init(struct tb_service *svc)
{
	svc->debugfs_dir = debugfs_create_dir(dev_name(&svc->dev),
					      tb_debugfs_root);
}

/**
 * tb_service_debugfs_remove() - Remove service debugfs directory
 * @svc: Thunderbolt service pointer
 *
 * Removes the previously created debugfs directory for @svc.
 */
void tb_service_debugfs_remove(struct tb_service *svc)
{
	debugfs_remove_recursive(svc->debugfs_dir);
	svc->debugfs_dir = NULL;
}

static int retimer_sb_regs_show(struct seq_file *s, void *not_used)
{
	struct tb_retimer *rt = s->private;
	struct tb *tb = rt->tb;
	int ret;

	pm_runtime_get_sync(&rt->dev);

	if (mutex_lock_interruptible(&tb->lock)) {
		ret = -ERESTARTSYS;
		goto out_rpm_put;
	}

	ret = sb_regs_show(rt->port, retimer_sb_regs, ARRAY_SIZE(retimer_sb_regs),
			   USB4_SB_TARGET_RETIMER, rt->index, s);

	mutex_unlock(&tb->lock);
out_rpm_put:
	pm_runtime_mark_last_busy(&rt->dev);
	pm_runtime_put_autosuspend(&rt->dev);

	return ret;
}
DEBUGFS_ATTR_RW(retimer_sb_regs);

/**
 * tb_retimer_debugfs_init() - Add debugfs directory for retimer
 * @rt: Pointer to retimer structure
 *
 * Adds and populates retimer debugfs directory.
 */
void tb_retimer_debugfs_init(struct tb_retimer *rt)
{
	struct dentry *debugfs_dir;

	debugfs_dir = debugfs_create_dir(dev_name(&rt->dev), tb_debugfs_root);
	debugfs_create_file("sb_regs", DEBUGFS_MODE, debugfs_dir, rt,
			    &retimer_sb_regs_fops);
	margining_retimer_init(rt, debugfs_dir);
}

/**
 * tb_retimer_debugfs_remove() - Remove retimer debugfs directory
 * @rt: Pointer to retimer structure
 *
 * Removes the retimer debugfs directory along with its contents.
 */
void tb_retimer_debugfs_remove(struct tb_retimer *rt)
{
	debugfs_lookup_and_remove(dev_name(&rt->dev), tb_debugfs_root);
	margining_retimer_remove(rt);
}

void tb_debugfs_init(void)
{
	tb_debugfs_root = debugfs_create_dir("thunderbolt", NULL);
}

void tb_debugfs_exit(void)
{
	debugfs_remove_recursive(tb_debugfs_root);
}
