/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77958 BC1.2 Charger Detection Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77958_BC12_H__
#define __MAX77958_BC12_H__
#include <linux/usb/typec/maxim/max77958.h>

#define MAX77958_BC12_NAME	"MAX77958_BC12"

struct max77958_bc12_data {
	/* interrupt pin */
	int irq_vbusdet;
	int irq_dcdtmo;
	int irq_chgtype;
	int irq_vbadc;
	int irq_uidadc;

	u8 usbc_status1;
	u8 usbc_status2;
	u8 bc_status;
	u8 cc_status0;
	u8 cc_status1;
	u8 pd_status0;
	u8 pd_status1;

	/* Status of VBUS Dectection */
	u8 vbusdet;
	/* DCD detection timed out */
	u8 dcdtmo;
	/* Output of Charger Detection */
	enum max77958_chg_type chg_type;

	/* Output of Properietary Charger Detection */
	enum max77958_pr_chg_type pr_chg_type;

	/* CHGIN Voltage ADC interrupt */
	u8 vbadc;

	/* UID ADC Interrupt */
	u8 uidadc;
};

#endif /* __MAX77958_BC12_H__ */
