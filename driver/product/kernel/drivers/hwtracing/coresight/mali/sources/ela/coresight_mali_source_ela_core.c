// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2022-2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <linux/of_platform.h>
#include <coresight-priv.h>
#include "sources/coresight_mali_sources.h"
#include "coresight-ela600.h"

/* Linux Coresight framework does not support multiple sources enabled
 * at the same time.
 *
 * To avoid Kernel instability, all Mali Coresight sources use the
 * same trace ID value as the mandatory ETM one.
 */
#define CS_MALI_TRACE_ID 0x00000010

#define CS_ELA_BASE_ADDR 0xE0043000
#define CS_GPU_COMMAND_ADDR 0x40003030
#define CS_GPU_COMMAND_TRACE_CONTROL_EN 0x000001DC

#define NELEMS(s) (sizeof(s) / sizeof((s)[0]))

#define CS_ELA_DYN_REGS_ATTR_RW(_regname)                                                  \
	static ssize_t _regname##_show(struct device *dev, struct device_attribute *attr,  \
				       char *const buf)                                    \
	{                                                                                  \
		return sprintf_reg(buf, CS_ELA_##_regname);                                \
	}                                                                                  \
	static ssize_t _regname##_store(struct device *dev, struct device_attribute *attr, \
					const char *buf, size_t count)                     \
	{                                                                                  \
		return verify_store_reg(dev, buf, count, CS_ELA_##_regname);               \
	}                                                                                  \
	static DEVICE_ATTR_RW(_regname)

#define CS_ELA_DYN_REGS_ATTR_RW_TRIG_STATE(_signo)     \
	CS_ELA_DYN_REGS_ATTR_RW(SIGSEL##_signo);       \
	CS_ELA_DYN_REGS_ATTR_RW(TRIGCTRL##_signo);     \
	CS_ELA_DYN_REGS_ATTR_RW(NEXTSTATE##_signo);    \
	CS_ELA_DYN_REGS_ATTR_RW(ACTION##_signo);       \
	CS_ELA_DYN_REGS_ATTR_RW(ALTNEXTSTATE##_signo); \
	CS_ELA_DYN_REGS_ATTR_RW(ALTACTION##_signo);    \
	CS_ELA_DYN_REGS_ATTR_RW(COMPCTRL##_signo);     \
	CS_ELA_DYN_REGS_ATTR_RW(ALTCOMPCTRL##_signo);  \
	CS_ELA_DYN_REGS_ATTR_RW(COUNTCOMP##_signo);    \
	CS_ELA_DYN_REGS_ATTR_RW(TWBSEL##_signo);       \
	CS_ELA_DYN_REGS_ATTR_RW(EXTMASK##_signo);      \
	CS_ELA_DYN_REGS_ATTR_RW(EXTCOMP##_signo);      \
	CS_ELA_DYN_REGS_ATTR_RW(QUALMASK##_signo);     \
	CS_ELA_DYN_REGS_ATTR_RW(QUALCOMP##_signo);     \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_0);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_1);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_2);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_3);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_4);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_5);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_6);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGMASK##_signo##_7);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_0);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_1);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_2);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_3);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_4);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_5);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_6);  \
	CS_ELA_DYN_REGS_ATTR_RW(SIGCOMP##_signo##_7)

#define CS_ELA_DYN_REGS_ATTR(_regname) &dev_attr_##_regname.attr

#define CS_ELA_DYN_REGS_ATTR_TRIG_STATE(_signo)                                                  \
	CS_ELA_DYN_REGS_ATTR(SIGSEL##_signo), CS_ELA_DYN_REGS_ATTR(TRIGCTRL##_signo),            \
		CS_ELA_DYN_REGS_ATTR(NEXTSTATE##_signo), CS_ELA_DYN_REGS_ATTR(ACTION##_signo),   \
		CS_ELA_DYN_REGS_ATTR(ALTNEXTSTATE##_signo),                                      \
		CS_ELA_DYN_REGS_ATTR(ALTACTION##_signo), CS_ELA_DYN_REGS_ATTR(COMPCTRL##_signo), \
		CS_ELA_DYN_REGS_ATTR(ALTCOMPCTRL##_signo),                                       \
		CS_ELA_DYN_REGS_ATTR(COUNTCOMP##_signo), CS_ELA_DYN_REGS_ATTR(TWBSEL##_signo),   \
		CS_ELA_DYN_REGS_ATTR(EXTMASK##_signo), CS_ELA_DYN_REGS_ATTR(EXTCOMP##_signo),    \
		CS_ELA_DYN_REGS_ATTR(QUALMASK##_signo), CS_ELA_DYN_REGS_ATTR(QUALCOMP##_signo),  \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_0),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_1),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_2),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_3),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_4),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_5),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_6),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGMASK##_signo##_7),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_0),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_1),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_2),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_3),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_4),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_5),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_6),                                       \
		CS_ELA_DYN_REGS_ATTR(SIGCOMP##_signo##_7)

#define WRITE_PTR_OP_CS_ELA_DYN_REGS_TRIG_STATE(_signo)                     \
	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGSEL(_signo),                 \
		     &ela_state.regs[CS_ELA_SIGSEL##_signo]),               \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_TRIGCTRL(_signo),       \
			     &ela_state.regs[CS_ELA_TRIGCTRL##_signo]),     \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_NEXTSTATE(_signo),      \
			     &ela_state.regs[CS_ELA_NEXTSTATE##_signo]),    \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_ACTION(_signo),         \
			     &ela_state.regs[CS_ELA_ACTION##_signo]),       \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_ALTNEXTSTATE(_signo),   \
			     &ela_state.regs[CS_ELA_ALTNEXTSTATE##_signo]), \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_ALTACTION(_signo),      \
			     &ela_state.regs[CS_ELA_ALTACTION##_signo]),    \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_COMPCTRL(_signo),       \
			     &ela_state.regs[CS_ELA_COMPCTRL##_signo]),     \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_ALTCOMPCTRL(_signo),    \
			     &ela_state.regs[CS_ELA_ALTCOMPCTRL##_signo]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_COUNTCOMP(_signo),      \
			     &ela_state.regs[CS_ELA_COUNTCOMP##_signo]),    \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_TWBSEL(_signo),         \
			     &ela_state.regs[CS_ELA_TWBSEL##_signo]),       \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_EXTMASK(_signo),        \
			     &ela_state.regs[CS_ELA_EXTMASK##_signo]),      \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_EXTCOMP(_signo),        \
			     &ela_state.regs[CS_ELA_EXTCOMP##_signo]),      \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_QUALMASK(_signo),       \
			     &ela_state.regs[CS_ELA_QUALMASK##_signo]),     \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_QUALCOMP(_signo),       \
			     &ela_state.regs[CS_ELA_QUALCOMP##_signo]),     \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 0),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_0]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 1),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_1]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 2),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_2]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 3),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_3]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 4),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_4]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 5),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_5]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 6),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_6]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGMASK(_signo, 7),     \
			     &ela_state.regs[CS_ELA_SIGMASK##_signo##_7]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 0),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_0]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 1),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_1]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 2),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_2]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 3),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_3]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 4),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_4]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 5),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_5]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 6),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_6]),  \
		WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_SIGCOMP(_signo, 7),     \
			     &ela_state.regs[CS_ELA_SIGCOMP##_signo##_7])

#define CS_ELA_DYN_REG_ENUM_TRIG_STATE(_signo)                                                 \
	CS_ELA_SIGSEL##_signo, CS_ELA_TRIGCTRL##_signo, CS_ELA_NEXTSTATE##_signo,              \
		CS_ELA_ACTION##_signo, CS_ELA_ALTNEXTSTATE##_signo, CS_ELA_ALTACTION##_signo,  \
		CS_ELA_COMPCTRL##_signo, CS_ELA_ALTCOMPCTRL##_signo, CS_ELA_COUNTCOMP##_signo, \
		CS_ELA_TWBSEL##_signo, CS_ELA_EXTMASK##_signo, CS_ELA_EXTCOMP##_signo,         \
		CS_ELA_QUALMASK##_signo, CS_ELA_QUALCOMP##_signo, CS_ELA_SIGMASK##_signo##_0,  \
		CS_ELA_SIGMASK##_signo##_1, CS_ELA_SIGMASK##_signo##_2,                        \
		CS_ELA_SIGMASK##_signo##_3, CS_ELA_SIGMASK##_signo##_4,                        \
		CS_ELA_SIGMASK##_signo##_5, CS_ELA_SIGMASK##_signo##_6,                        \
		CS_ELA_SIGMASK##_signo##_7, CS_ELA_SIGCOMP##_signo##_0,                        \
		CS_ELA_SIGCOMP##_signo##_1, CS_ELA_SIGCOMP##_signo##_2,                        \
		CS_ELA_SIGCOMP##_signo##_3, CS_ELA_SIGCOMP##_signo##_4,                        \
		CS_ELA_SIGCOMP##_signo##_5, CS_ELA_SIGCOMP##_signo##_6, CS_ELA_SIGCOMP##_signo##_7

enum cs_ela_dynamic_regs {
	CS_ELA_TIMECTRL,
	CS_ELA_TSSR,
	CS_ELA_ATBCTRL,
	CS_ELA_PTACTION,
	CS_ELA_AUXCTRL,
	CS_ELA_CNTSEL,

	CS_ELA_DYN_REG_ENUM_TRIG_STATE(0),
	CS_ELA_DYN_REG_ENUM_TRIG_STATE(1),
	CS_ELA_DYN_REG_ENUM_TRIG_STATE(2),
	CS_ELA_DYN_REG_ENUM_TRIG_STATE(3),
	CS_ELA_DYN_REG_ENUM_TRIG_STATE(4),

	CS_ELA_NR_DYN_REGS
};

struct cs_ela_state {
	int enabled;
	u32 regs[CS_ELA_NR_DYN_REGS];
};

#if KERNEL_VERSION(5, 3, 0) <= LINUX_VERSION_CODE
static char *type_name = "mali-source-ela";
#endif

static struct cs_ela_state ela_state = { 0 };

static void reset_dynamic_registers(void)
{
	memset(ela_state.regs, 0x00000000, sizeof(u32) * CS_ELA_NR_DYN_REGS);
}

static ssize_t is_enabled_show(struct device *dev, struct device_attribute *attr, char *const buf)
{
	return sprintf(buf, "%d\n", ela_state.enabled);
}
static DEVICE_ATTR_RO(is_enabled);

static ssize_t reset_regs_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct coresight_mali_source_drvdata *drvdata = dev_get_drvdata(dev->parent);
	if (ela_state.enabled == 1) {
		dev_err(drvdata->base.dev,
			"Config needs to be disabled before modifying registers");
		return -EINVAL;
	}
	reset_dynamic_registers();
	return count;
}
static DEVICE_ATTR_WO(reset_regs);

/* show and store functions for dynamic registers */
static ssize_t sprintf_reg(char *const buf, int reg)
{
	ssize_t ret = 0;

	ret += sprintf(buf + ret, "0x%08X\n", ela_state.regs[reg]);
	return ret;
}

static ssize_t verify_store_reg(struct device *dev, const char *buf, size_t count, int reg)
{
	struct coresight_mali_source_drvdata *drvdata = dev_get_drvdata(dev->parent);
	int items;
	u64 value;
	if (ela_state.enabled == 1) {
		dev_err(drvdata->base.dev,
			"Config needs to be disabled before modifying registers");
		return -EINVAL;
	}

	items = sscanf(buf, "%llx", &value);
	if (items <= 0 || value > U32_MAX) {
		dev_err(drvdata->base.dev, "Invalid register value");
		return -EINVAL;
	}
	ela_state.regs[reg] = (u32)value;

	return count;
}

CS_ELA_DYN_REGS_ATTR_RW(TIMECTRL);
CS_ELA_DYN_REGS_ATTR_RW(TSSR);
CS_ELA_DYN_REGS_ATTR_RW(ATBCTRL);
CS_ELA_DYN_REGS_ATTR_RW(PTACTION);
CS_ELA_DYN_REGS_ATTR_RW(AUXCTRL);
CS_ELA_DYN_REGS_ATTR_RW(CNTSEL);

CS_ELA_DYN_REGS_ATTR_RW_TRIG_STATE(0);
CS_ELA_DYN_REGS_ATTR_RW_TRIG_STATE(1);
CS_ELA_DYN_REGS_ATTR_RW_TRIG_STATE(2);
CS_ELA_DYN_REGS_ATTR_RW_TRIG_STATE(3);
CS_ELA_DYN_REGS_ATTR_RW_TRIG_STATE(4);

static struct attribute *coresight_ela_reg_attrs[] = {
	CS_ELA_DYN_REGS_ATTR(TIMECTRL),	    CS_ELA_DYN_REGS_ATTR(TSSR),
	CS_ELA_DYN_REGS_ATTR(ATBCTRL),	    CS_ELA_DYN_REGS_ATTR(PTACTION),
	CS_ELA_DYN_REGS_ATTR(AUXCTRL),	    CS_ELA_DYN_REGS_ATTR(CNTSEL),
	CS_ELA_DYN_REGS_ATTR_TRIG_STATE(0), CS_ELA_DYN_REGS_ATTR_TRIG_STATE(1),
	CS_ELA_DYN_REGS_ATTR_TRIG_STATE(2), CS_ELA_DYN_REGS_ATTR_TRIG_STATE(3),
	CS_ELA_DYN_REGS_ATTR_TRIG_STATE(4), NULL,
};

static struct attribute_group coresight_ela_reg_group = {
	.name = "regs",
	.attrs = coresight_ela_reg_attrs,
};

static struct attribute *coresight_ela_attrs[] = {
	&dev_attr_is_enabled.attr,
	&dev_attr_reset_regs.attr,
	NULL,
};

static struct attribute_group coresight_ela_group = {
	.attrs = coresight_ela_attrs,
};

static const struct attribute_group *coresight_ela_groups[] = {
	&coresight_ela_group,
	&coresight_ela_reg_group,
	NULL,
};

const struct attribute_group **coresight_mali_source_groups_get(void)
{
	return coresight_ela_groups;
}

/* Initialize ELA coresight driver */

static struct kbase_debug_coresight_csf_address_range ela_range[] = {
	{ CS_ELA_BASE_ADDR, CS_ELA_BASE_ADDR + CORESIGHT_DEVTYPE },
	{ CS_GPU_COMMAND_ADDR, CS_GPU_COMMAND_ADDR }
};

static struct kbase_debug_coresight_csf_op ela_enable_ops[] = {
	/* Clearing CTRL.RUN and the read only CTRL.TRACE_BUSY. */
	WRITE_IMM_OP(CS_ELA_BASE_ADDR + ELA_CTRL, 0x00000000),
	/* Poll CTRL.TRACE_BUSY until it becomes low to ensure that trace has stopped. */
	POLL_OP(CS_ELA_BASE_ADDR + ELA_CTRL, ELA_CTRL_TRACE_BUSY, 0x0),
	/* 0 for now. TSEN = 1 or TSINT = 8 in future */
	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_TIMECTRL, &ela_state.regs[CS_ELA_TIMECTRL]),
	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_TSSR, &ela_state.regs[CS_ELA_TSSR]),
	/* ATID[6:0] = 4; valid range 0x1-0x6F, value must be unique and needs to be
	 * known for trace extraction
	 */

	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_ATBCTRL, &ela_state.regs[CS_ELA_ATBCTRL]),
	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_PTACTION, &ela_state.regs[CS_ELA_PTACTION]),
	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_AUXCTRL, &ela_state.regs[CS_ELA_AUXCTRL]),
	WRITE_PTR_OP(CS_ELA_BASE_ADDR + ELA_CNTSEL, &ela_state.regs[CS_ELA_CNTSEL]),

	WRITE_PTR_OP_CS_ELA_DYN_REGS_TRIG_STATE(0),
	WRITE_PTR_OP_CS_ELA_DYN_REGS_TRIG_STATE(1),
	WRITE_PTR_OP_CS_ELA_DYN_REGS_TRIG_STATE(2),
	WRITE_PTR_OP_CS_ELA_DYN_REGS_TRIG_STATE(3),
	WRITE_PTR_OP_CS_ELA_DYN_REGS_TRIG_STATE(4),

	WRITE_IMM_OP(CS_GPU_COMMAND_ADDR, CS_GPU_COMMAND_TRACE_CONTROL_EN),

	WRITE_IMM_OP(CS_ELA_BASE_ADDR + ELA_CTRL, ELA_CTRL_RUN),

	BIT_OR_OP(&ela_state.enabled, 0x1),
};

static struct kbase_debug_coresight_csf_op ela_disable_ops[] = {
	WRITE_IMM_OP(CS_ELA_BASE_ADDR + ELA_CTRL, 0x00000000),
	/* Poll CTRL.TRACE_BUSY until it becomes low to ensure that trace has stopped. */
	POLL_OP(CS_ELA_BASE_ADDR + ELA_CTRL, ELA_CTRL_TRACE_BUSY, 0x0),

	BIT_AND_OP(&ela_state.enabled, 0x0),
};

int coresight_mali_sources_init_drvdata(struct coresight_mali_source_drvdata *drvdata)
{
#if KERNEL_VERSION(5, 3, 0) <= LINUX_VERSION_CODE
	drvdata->type_name = type_name;
#endif

	drvdata->base.kbase_client = kbase_debug_coresight_csf_register(
		drvdata->base.gpu_dev, ela_range, NELEMS(ela_range));
	if (drvdata->base.kbase_client == NULL) {
		dev_err(drvdata->base.dev, "Registration with full range failed unexpectedly");
		return -EINVAL;
	}

	drvdata->trcid = CS_MALI_TRACE_ID;

	drvdata->base.enable_seq.ops = ela_enable_ops;
	drvdata->base.enable_seq.nr_ops = NELEMS(ela_enable_ops);

	drvdata->base.disable_seq.ops = ela_disable_ops;
	drvdata->base.disable_seq.nr_ops = NELEMS(ela_disable_ops);

	drvdata->base.config = kbase_debug_coresight_csf_config_create(
		drvdata->base.kbase_client, &drvdata->base.enable_seq, &drvdata->base.disable_seq);
	if (!drvdata->base.config) {
		dev_err(drvdata->base.dev, "config create failed unexpectedly");
		return -EINVAL;
	}

	reset_dynamic_registers();

	return 0;
}

void coresight_mali_sources_deinit_drvdata(struct coresight_mali_source_drvdata *drvdata)
{
	if (drvdata->base.config != NULL)
		kbase_debug_coresight_csf_config_free(drvdata->base.config);

	if (drvdata->base.kbase_client != NULL)
		kbase_debug_coresight_csf_unregister(drvdata->base.kbase_client);
}

static const struct of_device_id mali_source_ids[] = { { .compatible =
								 "arm,coresight-mali-source-ela" },
						       {} };

static struct platform_driver mali_sources_platform_driver = {
	.probe      = coresight_mali_sources_probe,
	.remove     = coresight_mali_sources_remove,
	.driver = {
		.name = "coresight-mali-source-ela",
		.owner = THIS_MODULE,
		.of_match_table = mali_source_ids,
		.suppress_bind_attrs    = true,
	},
};

static int __init mali_sources_init(void)
{
	return platform_driver_register(&mali_sources_platform_driver);
}

static void __exit mali_sources_exit(void)
{
	platform_driver_unregister(&mali_sources_platform_driver);
}

module_init(mali_sources_init);
module_exit(mali_sources_exit);

MODULE_AUTHOR("Arm Ltd.");
MODULE_DESCRIPTION("Arm Coresight Mali source ELA");
MODULE_LICENSE("GPL");
