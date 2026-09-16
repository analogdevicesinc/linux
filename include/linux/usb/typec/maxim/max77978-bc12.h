/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77978 BC1.2 Charger Detection Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77978_BC12_H__
#define __MAX77978_BC12_H__
#include <linux/usb/typec/maxim/max77978.h>

#define MAX77978_BC12_NAME	"MAX77978_BC12"

struct max77978_bc12_data {
	/* interrupt pin */
	int irq_vbusdet;
	int irq_dcdtmo;
	int irq_chgtype;
#ifdef USE_UIC_IRQ_VBADC
	int irq_vbadc;
#else
	int irq_vbadch;
	int irq_vbadcl;
#endif
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
	enum max77978_chg_type chg_type;

	/* Output of Properietary Charger Detection */
	enum max77978_pr_chg_type pr_chg_type;

	/* CHGIN Voltage ADC interrupt */
	u16 vbvolt;
	u8 vbvolth;
	u8 vbvoltl;
	/* UID ADC Interrupt */
	u8 uidadc;
};

#endif /* __MAX77978_BC12_H__ */
