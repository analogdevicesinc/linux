/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77978 CC Pin Management Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77978_CC_H__
#define __MAX77978_CC_H__
#include <linux/usb/typec/maxim/max77978.h>

#define MAX77978_CC_NAME	"MAX77978_CC"

enum max77978_vcon_role_e {
	VCON_OFF,
	VCON_ON,
};

struct max77978_cc_data {
	/* interrupt pin */
	int irq_vconnocp;
	int irq_vsafe0v;
	int irq_detabrt;
	int irq_vconnsc;
	int irq_ccpinstat;
	int irq_ccistat;
	int irq_ccvcnstat;
	int irq_ccstat;

	u8 usbc_status1;
	u8 usbc_status2;
	u8 bc_status;
	u8 cc_status0;
	u8 cc_status1;
	u8 pd_status0;
	u8 pd_status1;

	u8 opcode_res;

	/* VCONN Over Current Detection */
	u8 vconnocp;
	/* VCONN Over Short Circuit Detection */
	u8 vconnsc;
	/* Status of VBUS Detection */
	u8 vsafe0v;
	/* Charger Detection Abort Status */
	u8 detabrt;
	/* Output of active CC pin */
	u8 ccpinstat;
	/* CC Pin Detected Allowed VBUS Current in UFP mode */
	u8 ccistat;
	/* Status of Vconn Output */
	u8 ccvcnstat;
	/* CC Pin State Machine Detection */
	u8 ccstat;

	enum max77978_vcon_role_e	current_vcon;
	enum max77978_vcon_role_e	previous_vcon;
	enum typec_port_type		current_port_type;
	enum typec_port_type		previous_port_type;

	struct wakeup_source *ccstat_ws;
};

#endif /* __MAX77978_CC_H__ */
