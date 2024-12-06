/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017~2018,2020 NXP
 *
 * Header file containing the public System Controller Interface (SCI)
 * definitions.
 */

#ifndef _SC_SCI_H
#define _SC_SCI_H

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/firmware/imx/ipc.h>

#include <linux/firmware/imx/svc/misc.h>
#include <linux/firmware/imx/svc/pm.h>
#include <linux/firmware/imx/svc/rm.h>
#include <linux/firmware/imx/svc/seco.h>

#define IMX_SC_IRQ_NUM_GROUP            9

#define IMX_SC_IRQ_GROUP_TEMP           0   /* Temp interrupts */
#define IMX_SC_IRQ_GROUP_WDOG           1   /* Watchdog interrupts */
#define IMX_SC_IRQ_GROUP_RTC            2   /* RTC interrupts */
#define IMX_SC_IRQ_GROUP_WAKE           3   /* Wakeup interrupts */
#define IMX_SC_IRQ_GROUP_SYSCTR         4   /* System counter interrupts */
#define IMX_SC_IRQ_GROUP_REBOOTED       5   /* Partition reboot complete */
#define IMX_SC_IRQ_GROUP_REBOOT         6   /* Partition reboot starting */
#define IMX_SC_IRQ_GROUP_OFFED          7   /* Partition off complete */
#define IMX_SC_IRQ_GROUP_OFF            8   /* Partition off starting */

#define IMX_SC_IRQ_RTC               BIT(0)    /* RTC interrupt */
#define IMX_SC_IRQ_WDOG              BIT(0)    /* Watch Dog interrupt */
#define IMX_SC_IRQ_SYSCTR            BIT(0)    /* System Counter interrupt */
#define IMX_SC_IRQ_BUTTON            BIT(0)    /* Button interrupt */
#define IMX_SC_IRQ_PAD               BIT(1)    /* Pad wakeup */
#define IMX_SC_IRQ_USR1              BIT(2)    /* User defined 1 */
#define IMX_SC_IRQ_USR2              BIT(3)    /* User defined 2 */
#define IMX_SC_IRQ_BC_PAD            BIT(4)    /* Pad wakeup (broadcast to all partitions) */
#define IMX_SC_IRQ_SW_WAKE           BIT(5)    /* Software requested wake */
#define IMX_SC_IRQ_SECVIO            BIT(6)    /* Security violation */
#define IMX_SC_IRQ_V2X_RESET         BIT(7)    /* V2X reset */

#if IS_ENABLED(CONFIG_IMX_SCU)
int imx_scu_enable_general_irq_channel(struct device *dev);
int imx_scu_irq_register_notifier(struct notifier_block *nb);
int imx_scu_irq_unregister_notifier(struct notifier_block *nb);
int imx_scu_irq_group_enable(u8 group, u32 mask, u8 enable);
int imx_scu_irq_get_status(u8 group, u32 *irq_status);
int imx_scu_soc_init(struct device *dev);
#else
static inline int imx_scu_soc_init(struct device *dev)
{
	return -EOPNOTSUPP;
}

static inline int imx_scu_enable_general_irq_channel(struct device *dev)
{
	return -EOPNOTSUPP;
}

static inline int imx_scu_irq_register_notifier(struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}

static inline int imx_scu_irq_unregister_notifier(struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}

static inline int imx_scu_irq_group_enable(u8 group, u32 mask, u8 enable)
{
	return -EOPNOTSUPP;
}

static inline int imx_scu_irq_get_status(u8 group, u32 *irq_status)
{
	return -EOPNOTSUPP;
}
#endif
#endif /* _SC_SCI_H */
