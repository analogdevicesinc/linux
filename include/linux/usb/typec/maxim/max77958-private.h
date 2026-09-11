/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77958 Private Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77958_PRIVATE_H__
#define __MAX77958_PRIVATE_H__

#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/typec/maxim/max77958.h>
#define MAX77958_REG_INVALID	(0xff)

enum max77958_irq_source {
		UIC_INT = 0,
		CC_INT,
		PD_INT,
		ACTION_INT,
		MAX77958_IRQ_GROUP_NR,
};

enum max77958_irq {
	CCIC_IRQ_INIT_DETECT = -1,

	/* USBC */
	MAX77958_UIC_IRQ_APC_INT,
	MAX77958_UIC_IRQ_SYSM_INT,
	MAX77958_UIC_IRQ_VBUS_INT,
	MAX77958_UIC_IRQ_VBADC_INT,
	MAX77958_UIC_IRQ_DCD_INT,
	MAX77958_UIC_IRQ_CHGT_INT,
	MAX77958_UIC_IRQ_UIDADC_INT,

	/* CC */
	MAX77958_CC_IRQ_VCONNCOP_INT,
	MAX77958_CC_IRQ_VSAFE0V_INT,
	MAX77958_CC_IRQ_DETABRT_INT,
	MAX77958_CC_IRQ_VCONNSC_INT,
	MAX77958_CC_IRQ_CCPINSTAT_INT,
	MAX77958_CC_IRQ_CCISTAT_INT,
	MAX77958_CC_IRQ_CCVCNSTAT_INT,
	MAX77958_CC_IRQ_CCSTAT_INT,

	/* PD */
	MAX77958_PD_IRQ_PDMSG_INT,
	MAX77958_PD_IRQ_PS_RDY_INT,
	MAX77958_PD_IRQ_DATAROLE_INT,
	MAX77958_PD_IRQ_DISPLAYPORT_INT,

	/*ACTION*/
	MAX77958_IRQ_ACTION0_INT,
	MAX77958_IRQ_ACTION1_INT,
	MAX77958_IRQ_ACTION2_INT,
	MAX77958_IRQ_EXTENDED_ACTION_INT,

	MAX77958_IRQ_NR,
};

struct max77958_platform_data {
	/* IRQ */
	int irq_base;
	bool wakeup;
};

struct max77958_dev {
	struct device *dev;
	struct i2c_client *i2c;
	struct mutex i2c_lock;
	struct wakeup_source *ws;

	int type;

	int irq;
	int irq_base;
	bool wakeup;
	struct mutex irqlock;
	int irq_masks_cur[MAX77958_IRQ_GROUP_NR];
	int irq_masks_cache[MAX77958_IRQ_GROUP_NR];

	u8 hw_revision;
	u8 dev_id;
	u8 fw_revision;
	u8 fw_minor_revision;
	struct work_struct wait_apcmd_resp_work;
	struct workqueue_struct *wait_apcmd_resp_work_queue;
	struct completion wait_apcmd_resp_completion;
	int fw_update_state;
	u8 dp_status;

	u8 boot_complete;

	bool shutdown;
	bool suspended;
	wait_queue_head_t suspend_wait;

	struct max77958_platform_data *pdata;
};

int max77958_irq_init(struct max77958_dev *max77958);
void max77958_irq_exit(struct max77958_dev *max77958);
int max77958_usbc_fw_update(struct max77958_dev
		*max77958, const u8 *fw_bin, int fw_bin_len, int enforce_do);
u8 max77958_read_devrev(struct max77958_dev *max77958);

#endif /* __MAX77958_PRIVATE_H__ */

