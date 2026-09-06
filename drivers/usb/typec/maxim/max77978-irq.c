// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 IRQ Driver
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/usb/typec/maxim/max77978-private.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>

static const u8 max77978_mask_reg[] = {
	[UIC_INT] = REG_UIC_INT_M,
	[CC_INT] = REG_CC_INT_M,
	[PD_INT] = REG_PD_INT_M,
	[ACTION_INT] = REG_ACTION_INT_M,
};

static struct i2c_client *get_i2c(struct max77978_dev *max77978, enum max77978_irq_source src)
{
	switch (src) {
	case UIC_INT:
	case CC_INT:
	case PD_INT:
	case ACTION_INT:
		return max77978->i2c;
	default:
		return ERR_PTR(-EINVAL);
	}
}

struct max77978_irq_data {
	int mask;
	enum max77978_irq_source group;
};

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }

static const struct max77978_irq_data max77978_irqs[] = {
	DECLARE_IRQ(MAX77978_UIC_IRQ_APC_INT,		UIC_INT, BIT(7)),
	DECLARE_IRQ(MAX77978_UIC_IRQ_SYSM_INT,		UIC_INT, BIT(6)),
	DECLARE_IRQ(MAX77978_UIC_IRQ_VBUS_INT,		UIC_INT, BIT(5)),
#ifdef USE_UIC_IRQ_VBADC
	DECLARE_IRQ(MAX77978_UIC_IRQ_VBADC_INT,		UIC_INT, BIT(4)),
#else
	DECLARE_IRQ(MAX77978_UIC_IRQ_VBVOLTH_INT,	UIC_INT, BIT(4)),
#endif
	DECLARE_IRQ(MAX77978_UIC_IRQ_DCD_INT,		UIC_INT, BIT(3)),
#ifdef USE_UIC_IRQ_VBADC
	DECLARE_IRQ(MAX77978_UIC_IRQ_STOPMODE_INT,	UIC_INT, BIT(2)),
#else
	DECLARE_IRQ(MAX77978_UIC_IRQ_VBVOLTL_INT,	UIC_INT, BIT(2)),
#endif
	DECLARE_IRQ(MAX77978_UIC_IRQ_CHGT_INT,		UIC_INT, BIT(1)),
	DECLARE_IRQ(MAX77978_UIC_IRQ_UIDADC_INT,	UIC_INT, BIT(0)),

	DECLARE_IRQ(MAX77978_CC_IRQ_VCONNCOP_INT,	CC_INT, BIT(7)),
	DECLARE_IRQ(MAX77978_CC_IRQ_VSAFE0V_INT,	CC_INT, BIT(6)),
	DECLARE_IRQ(MAX77978_CC_IRQ_DETABRT_INT,	CC_INT, BIT(5)),
	DECLARE_IRQ(MAX77978_CC_IRQ_VCONNSC_INT,	CC_INT, BIT(4)),
	DECLARE_IRQ(MAX77978_CC_IRQ_CCPINSTAT_INT,	CC_INT, BIT(3)),
	DECLARE_IRQ(MAX77978_CC_IRQ_CCISTAT_INT,	CC_INT, BIT(2)),
	DECLARE_IRQ(MAX77978_CC_IRQ_CCVCNSTAT_INT,	CC_INT, BIT(1)),
	DECLARE_IRQ(MAX77978_CC_IRQ_CCSTAT_INT,		CC_INT, BIT(0)),

	DECLARE_IRQ(MAX77978_PD_IRQ_PDMSG_INT,			PD_INT, BIT(7)),
	DECLARE_IRQ(MAX77978_PD_IRQ_PS_RDY_INT,			PD_INT, BIT(6)),
	DECLARE_IRQ(MAX77978_PD_IRQ_DATAROLE_INT,		PD_INT, BIT(5)),
	DECLARE_IRQ(MAX77978_PD_IRQ_DISPLAYPORT_INT,	PD_INT, BIT(2)),

	DECLARE_IRQ(MAX77978_IRQ_ACTION7_INT,	ACTION_INT, BIT(7)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION6_INT,	ACTION_INT, BIT(6)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION5_INT,	ACTION_INT, BIT(5)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION4_INT,	ACTION_INT, BIT(4)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION3_INT,	ACTION_INT, BIT(3)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION2_INT,	ACTION_INT, BIT(2)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION1_INT,	ACTION_INT, BIT(1)),
	DECLARE_IRQ(MAX77978_IRQ_ACTION0_INT,	ACTION_INT, BIT(0)),
};

static void max77978_irq_lock(struct irq_data *data)
{
	struct max77978_dev *max77978 = irq_get_chip_data(data->irq);

	mutex_lock(&max77978->irqlock);
}

static void max77978_irq_sync_unlock(struct irq_data *data)
{
	struct max77978_dev *max77978 = irq_get_chip_data(data->irq);
	int i;

	for (i = 0; i < MAX77978_IRQ_GROUP_NR; i++) {
		u8 mask_reg = max77978_mask_reg[i];
		struct i2c_client *i2c = get_i2c(max77978, i);

		if (mask_reg == MAX77978_REG_INVALID || IS_ERR_OR_NULL(i2c))
			continue;
		max77978->irq_masks_cache[i] = max77978->irq_masks_cur[i];

		max77978_write_reg(i2c, max77978_mask_reg[i], max77978->irq_masks_cur[i]);
	}

	mutex_unlock(&max77978->irqlock);
}

static inline const struct max77978_irq_data *irq_to_max77978_irq(struct max77978_dev *max77978, int irq)
{
	int index = 0;

	if (!max77978)
		return NULL;

	index = irq - max77978->irq_base;
	if (index < 0 || index >= MAX77978_IRQ_NR)
		return NULL;

	return &max77978_irqs[index];
}

static void max77978_irq_mask(struct irq_data *data)
{
	struct max77978_dev *max77978 = irq_get_chip_data(data->irq);
	const struct max77978_irq_data *irq_data =
		irq_to_max77978_irq(max77978, data->irq);

	if (!irq_data || irq_data->group >= MAX77978_IRQ_GROUP_NR)
		return;

	max77978->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void max77978_irq_unmask(struct irq_data *data)
{
	struct max77978_dev *max77978 = irq_get_chip_data(data->irq);
	const struct max77978_irq_data *irq_data =
		irq_to_max77978_irq(max77978, data->irq);

	if (!irq_data || irq_data->group >= MAX77978_IRQ_GROUP_NR)
		return;

	max77978->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static void max77978_irq_disable(struct irq_data *data)
{
	max77978_irq_mask(data);
}

static struct irq_chip max77978_irq_chip = {
	.name			= MFD_DEV_NAME,
	.irq_bus_lock		= max77978_irq_lock,
	.irq_bus_sync_unlock	= max77978_irq_sync_unlock,
	.irq_mask		= max77978_irq_mask,
	.irq_unmask		= max77978_irq_unmask,
	.irq_disable            = max77978_irq_disable,
};

static irqreturn_t max77978_irq_thread(int irq, void *data)
{
	struct max77978_dev *max77978 = data;
	u8 irq_reg[MAX77978_IRQ_GROUP_NR] = {0};
	u8 dump_reg[10] = {0, };
	int i, rc;

	__pm_stay_awake(max77978->ws);

	if (max77978->suspended) {
		pr_err("%s:%s skip.max77978 suspended.\n", MFD_DEV_NAME, __func__);
		/* Irq will occur again because of IRQF_TRIGGER_LOW */
		wait_event_interruptible_timeout(max77978->suspend_wait,
						!max77978->suspended,
						msecs_to_jiffies(50));
		__pm_relax(max77978->ws);
		return IRQ_NONE;
	}
	if (!max77978->boot_complete) {
		__pm_relax(max77978->ws);
		return IRQ_HANDLED;
	}

	pr_info("%s: irq[%d] %d/%d", __func__,
			irq, max77978->irq, max77978->irq_base);

	rc = max77978_bulk_read(max77978->i2c, REG_UIC_INT, 4, &irq_reg[UIC_INT]);
	if (rc) {
		pr_err("%s: failed to max77978_bulk_read for REG_UIC_INT\n", __func__);
		__pm_relax(max77978->ws);
		return IRQ_HANDLED;

	}

	rc = max77978_bulk_read(max77978->i2c, REG_USBC_STATUS1, 8, dump_reg);
	if (rc) {
		pr_err("%s: failed to max77978_bulk_read for REG_UIC_INT\n", __func__);
		__pm_relax(max77978->ws);
		return IRQ_HANDLED;

	}

	pr_info("%s: HWRev=%02X.%s(0x%02X.0x%02X) FWRev=%02X.%02X\n", __func__,
			max77978->device_id, MAX77978_DEVREV_STR(max77978->device_revision),
			max77978->device_id, max77978->device_revision,
			max77978->fw_revision,
			max77978->fw_minor_revision);

	pr_info("%s: irq_reg usbcI=%x, ccI=%x, pdI=%x, actI=%x\n", __func__,
			irq_reg[UIC_INT],
			irq_reg[CC_INT],
			irq_reg[PD_INT],
			irq_reg[ACTION_INT]);

	pr_info("%s: dump_reg, S1=%x, S2=%x, bcS=%x, dpS=%x, ccS0=%x, ccS1=%x, pdS0=%x, pdS1=%x\n",
			__func__, dump_reg[0], dump_reg[1], dump_reg[2], dump_reg[3],
			dump_reg[4], dump_reg[5], dump_reg[6], dump_reg[7]);
	/* save dpstatus */
	max77978->dp_status = dump_reg[3];

	/* Apply masking */
	for (i = 0; i < MAX77978_IRQ_GROUP_NR; i++)
		irq_reg[i] &= ~max77978->irq_masks_cur[i];

	/* Report */
	for (i = 0; i < MAX77978_IRQ_NR ; i++) {
		if (irq_reg[max77978_irqs[i].group] & max77978_irqs[i].mask)
			handle_nested_irq(max77978->irq_base + i);
	}

	__pm_relax(max77978->ws);

	return IRQ_HANDLED;
}

int max77978_irq_init(struct max77978_dev *max77978)
{
	int i;
	int ret;

	if (!max77978->irq) {
		dev_warn(max77978->dev, "No interrupt specified.\n");
		max77978->irq_base = 0;
		return 0;
	}

	if (!max77978->irq_base) {
		dev_err(max77978->dev, "No interrupt base specified.\n");
		return 0;
	}

	mutex_init(&max77978->irqlock);

	/* Mask individual interrupt sources */
	for (i = 0; i < MAX77978_IRQ_GROUP_NR; i++) {
		struct i2c_client *i2c;
		/* IRQ 1:MASK 0:NOT MASK */
		max77978->irq_masks_cur[i] = 0xff;
		max77978->irq_masks_cache[i] = 0xff;

		i2c = get_i2c(max77978, i);

		if (IS_ERR_OR_NULL(i2c))
			continue;
		if (max77978_mask_reg[i] == MAX77978_REG_INVALID)
			continue;
		max77978_write_reg(i2c, max77978_mask_reg[i], 0xff);
	}

	/* Register with genirq */
	for (i = 0; i < MAX77978_IRQ_NR; i++) {
		int cur_irq;

		cur_irq = i + max77978->irq_base;

		irq_set_chip_data(cur_irq, max77978);
		irq_set_chip_and_handler(cur_irq, &max77978_irq_chip, handle_level_irq);
		irq_set_nested_thread(cur_irq, 1);
		irq_set_noprobe(cur_irq);
	}

	ret = request_threaded_irq(max77978->irq, NULL, max77978_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"max77978-irq", max77978);
	if (ret) {
		dev_err(max77978->dev, "Failed to request IRQ %d: %d\n", max77978->irq, ret);
		return ret;
	}

	return 0;
}

void max77978_irq_exit(struct max77978_dev *max77978)
{
	if (max77978->irq)
		free_irq(max77978->irq, max77978);
	mutex_destroy(&max77978->irqlock);
}

MODULE_DESCRIPTION("max77978 IRQ driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
