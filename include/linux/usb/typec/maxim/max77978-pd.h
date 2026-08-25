/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77978 Power Delivery Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77978_PD_H__
#define __MAX77978_PD_H__
#include <linux/usb/typec/maxim/max77978.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>
#define MAX77978_PD_NAME	"MAX77978_PD"

enum {
	CC_SNK = 0,
	CC_SRC,
	CC_NO_CONN,
};

enum {
	D2D_NONE	= 0,
	D2D_SNKONLY,
	D2D_SRCSNK,
};

typedef enum {
	PDO_TYPE_FIXED = 0,
	PDO_TYPE_BATTERY,
	PDO_TYPE_VARIABLE,
	PDO_TYPE_APDO,
} pdo_supply_type_t;

union max77978_pdo_object {
	uint32_t		data;
	struct {
		uint8_t		bdata[4];
	} BYTES;
	struct {
		uint32_t	reserved:30,
					type:2;
	} BITS_supply;
	struct {
		uint32_t	max_current:10,	/* 10mA units */
					voltage:10,		/* 50mV units */
					peak_current:2,
					reserved:2,
					unchuncked_extended_messages_supported:1,
					data_role_data:1,
					usb_communications_capable:1,
					unconstrained_power:1,
					usb_suspend_supported:1,
					dual_role_power:1,
					supply:2;		/* Fixed supply : 00b */
	} BITS_pdo_fixed;
	struct {
		uint32_t	max_current:10,		/* 10mA units */
				min_voltage:10,		/* 50mV units */
				max_voltage:10,		/* 50mV units */
				supply:2;		/* Variable Supply (non-Battery) : 10b */
	} BITS_pdo_variable;
	struct {
		uint32_t	max_allowable_power:10,		/* 250mW units */
				min_voltage:10,		/* 50mV units  */
				max_voltage:10,		/* 50mV units  */
				supply:2;		/* Battery : 01b */
	} BITS_pdo_battery;
	struct {
		uint32_t	max_current:7, 	/* 50mA units */
				reserved1:1,
				min_voltage:8, 	/* 100mV units	*/
				reserved2:1,
				max_voltage:8, 	/* 100mV units	*/
				reserved3:2,
				pps_power_limited:1,
				pps_supply:2,
				supply:2;		/* APDO : 11b */
	} BITS_pdo_programmable;
};

#define MAX_PDO_NUM				(8)
#define UNIT_FOR_VOLTAGE		(50)
#define UNIT_FOR_CURRENT		(10)
#define AVAILABLE_VOLTAGE		(9000)
#define DEFAULT_VOLTAGE			(5000)
#define UNIT_FOR_APDO_VOLTAGE	(100)
#define UNIT_FOR_APDO_CURRENT	(50)

struct max77978_pd_data {
	/* interrupt pin */
	int irq_pdmsg;
	int irq_psrdy;
	int irq_datarole;
	int irq_fct_id;
	int irq_vdm;

	u8 usbc_status1;
	u8 usbc_status2;
	u8 bc_status;
	u8 cc_status0;
	u8 cc_status1;
	u8 pd_status0;
	u8 pd_status1;

	u8 opcode_res;

	/* PD Message */
	u8 pdsmg;

	/* Data Role */
	enum typec_port_data current_port_data;
	enum typec_port_data previous_port_data;
	/* FCT cable */
	u8 fct_id;
	enum max77978_ccpd_device device;

	bool pdo_list;
	bool psrdy_received;
	bool cc_sbu_short;
	bool bPPS_on;
	bool sent_chg_info;

	struct workqueue_struct *wqueue;
	struct delayed_work retry_work;
	struct delayed_work d2d_work;
	struct delayed_work abnormal_pdo_work;

	u8 cc_status;
	u8 dp_status;

	int src_cap_done;
	int auth_type;
	int d2d_type;
	int req_pdo_type;
	bool psrdy_sent;
};

#endif /* __MAX77978_PD_H__ */
