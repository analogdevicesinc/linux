// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

static int a2b_read_one_irq(struct a2b_mainnode *mainnode)
{
	struct regmap *regmap = mainnode->node.regmap;
	struct device *dev = &mainnode->node.dev;
	unsigned int val, inttype, intsrc;
	int ret;

	ret = regmap_read(regmap, A2B_INTSTAT, &val);
	if (ret < 0) {
		dev_err(dev, "unable to read INTSTAT register: %d\n", ret);
		return ret;
	}

	if (!(val & A2B_INTSTAT_IRQ))
		return -ENOENT;

	ret = regmap_read(regmap, A2B_INTTYPE, &inttype);
	if (ret < 0) {
		dev_err(dev, "unable to read INTTYPE register: %d\n", ret);
		return ret;
	}

	ret = regmap_read(regmap, A2B_INTSRC, &intsrc);
	if (ret < 0) {
		dev_err(dev, "unable to read INTSRC register: %d\n", ret);
		return ret;
	}

	ret = regmap_read(regmap, A2B_INTPND0, &val);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, A2B_INTPND0, val);
	if (ret < 0)
		return ret;

	ret = regmap_read(regmap, A2B_INTPND1, &val);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, A2B_INTPND1, val);
	if (ret < 0)
		return ret;

	if (val & A2B_INTSRC_MSTINT) {
		ret = regmap_read(regmap, A2B_INTPND2, &val);
		if (ret < 0)
			return ret;

		ret = regmap_write(regmap, A2B_INTPND2, val);
		if (ret < 0)
			return ret;
	}

	dev_dbg(dev, "%s() inttype: 0x%02x intsrc: 0x%02x \n", __func__,
		inttype, intsrc);

	switch (inttype) {
	case A2B_INTTYPE_DSCDONE:
		complete(&mainnode->discover_completion);
		break;
	case A2B_INTTYPE_MSTR_RUNNING:
		complete(&mainnode->run_completion);
		break;
	default:
		dev_info(dev, "Unhandled interrupt type 0x%02x\n", inttype);
	}

	return 0;
}

static int a2b_read_irqs(struct a2b_mainnode *mainnode)
{
	bool first = true;
	int ret;

	while (true) {
		ret = a2b_read_one_irq(mainnode);
		if (ret < 0)
			return ret;
		if (ret == -ENOENT)
			return first ? ret : 0;

		first = false;
	}
}

static irqreturn_t a2b_handle_irq(int irq, void *devid)
{
	struct a2b_mainnode *mainnode = devid;
	int ret;

	ret = a2b_read_irqs(mainnode);
	if (ret == -ENOENT)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

int a2b_wait_for_irq(struct a2b_mainnode *mainnode,
		     struct completion *completion, unsigned int timeout)
{
	int ret;

	if (mainnode->irq > 0) {
		ret = wait_for_completion_timeout(completion,
						  msecs_to_jiffies(timeout));
	} else {
		usleep_range(timeout * 1000, timeout * 1500);
		a2b_read_irqs(mainnode);
		ret = completion_done(completion);
	}

	return ret == 0 ? -ETIMEDOUT : 0;
}

int a2b_init_irq(struct a2b_mainnode *mainnode)
{
	struct regmap *regmap = mainnode->node.regmap;
	struct device *dev = &mainnode->node.dev;
	int ret;

	if (mainnode->irq > 0) {
		ret = devm_request_threaded_irq(dev, mainnode->irq, NULL,
						a2b_handle_irq, IRQF_ONESHOT,
						dev_name(dev), mainnode);
		if (ret < 0)
			return ret;
	}

	ret = regmap_write(regmap, A2B_INTMSK0,
			   A2B_INTMSK0_SRFEIEN | A2B_INTMSK0_BECIEN |
				   A2B_INTMSK0_PWREIEN | A2B_INTMSK0_CRCEIEN |
				   A2B_INTMSK0_DDEIEN | A2B_INTMSK0_HCEIEN);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, A2B_INTMSK2,
			   A2B_INTMSK2_DSCDIEN | A2B_INTMSK2_SLVIRQEN);
	if (ret < 0)
		return ret;

	return 0;
}
