// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 Power Delivery Driver
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
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/usb/typec/maxim/max77978-private.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>
#include <linux/usb/typec/maxim/max77978-alternate.h>

void max77978_detach_pd(struct max77978_usbc_platform_data *usbc_data)
{
	struct max77978_pd_data *pd_data = usbc_data->pd_data;

	msg_info("Detach PD CHARGER");
	pd_data->psrdy_received = false;
	pd_data->pdo_list = false;
}

void max77978_vbus_turn_on_ctrl(struct max77978_usbc_platform_data *usbc_data, bool enable)
{
	msg_info("enable=%d", enable);
	/* TODO : control charger */
}

void __maybe_unused max77978_select_pdo(void *data, int num)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct max77978_usbc_command_data cmd_data;

	init_usbc_cmd_data(&cmd_data);
	cmd_data.opcode = OPCODE_SRCCAP_REQUEST;
	cmd_data.write_data[0] = num;
	cmd_data.write_length = 1;
	cmd_data.read_length = 1;
	max77978_usbc_opcode_write(usbpd_data, &cmd_data);
}

static int max77978_check_pdo(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct max77978_usbc_command_data cmd_data;

	init_usbc_cmd_data(&cmd_data);
	cmd_data.opcode = OPCODE_CURRENT_SRCCAP;
	cmd_data.write_data[0] = 0x0;
	cmd_data.write_length = 0x1;
	cmd_data.read_length = 31;
	max77978_usbc_opcode_write(usbpd_data, &cmd_data);

	return 0;
}

void max77978_current_pdo(struct max77978_usbc_platform_data *usbc_data, unsigned char *data)
{
	int i;
	int num_of_pdo = 0;
	int sel_pdo_pos = 0;
	union max77978_pdo_object pdo_obj;
	int v_max, v_min, i_max;

	sel_pdo_pos = ((data[1] >> 3) & 0x07);
	num_of_pdo = data[1] & 0x07;
	if (num_of_pdo > MAX_PDO_NUM) {
		msg_info("update available_pdo_num[%d -> %d]", num_of_pdo, MAX_PDO_NUM);
		num_of_pdo = MAX_PDO_NUM;
	}
	msg_info("sel_pdo_pos=%d num_of_pdo=%d", sel_pdo_pos, num_of_pdo);

	for (i = 0; i < num_of_pdo; i++) {
		pdo_obj.data = (data[2 + (i * 4)]
				| (data[3 + (i * 4)] << 8)
				| (data[4 + (i * 4)] << 16)
				| (data[5 + (i * 4)] << 24));

		switch (pdo_obj.BITS_supply.type) {
		case PDO_TYPE_FIXED:
			v_max = pdo_obj.BITS_pdo_fixed.voltage * UNIT_FOR_VOLTAGE;
			v_min = 0;
			i_max = pdo_obj.BITS_pdo_fixed.max_current * UNIT_FOR_CURRENT;
			msg_info("%cPDO[%d] type=%d(FPDO) V_MAX=%d V_MIN=%d I_MAX=%d comm_capable=%d suspend=%d",
					(i == (sel_pdo_pos - 1)) ? '*' : ' ',
					i, pdo_obj.BITS_supply.type,
					v_max, v_min, i_max,
					pdo_obj.BITS_pdo_fixed.usb_communications_capable,
					pdo_obj.BITS_pdo_fixed.usb_suspend_supported);
			break;
		case PDO_TYPE_APDO:
			v_max = pdo_obj.BITS_pdo_programmable.max_voltage * UNIT_FOR_APDO_VOLTAGE;
			v_min = pdo_obj.BITS_pdo_programmable.min_voltage * UNIT_FOR_APDO_VOLTAGE;
			i_max = pdo_obj.BITS_pdo_programmable.max_current * UNIT_FOR_APDO_CURRENT;
			msg_info("%cPDO[%d] type=%d(APDO) V_MAX=%d V_MIN=%d I_MAX=%d",
					(i == (sel_pdo_pos - 1)) ? '*' : ' ',
					i, pdo_obj.BITS_supply.type,
					v_max, v_min, i_max);
			break;
		case PDO_TYPE_VARIABLE:
			v_max = pdo_obj.BITS_pdo_programmable.max_voltage * UNIT_FOR_VOLTAGE;
			v_min = pdo_obj.BITS_pdo_programmable.min_voltage * UNIT_FOR_VOLTAGE;
			i_max = pdo_obj.BITS_pdo_programmable.max_current * UNIT_FOR_CURRENT;
			msg_info("%cPDO[%d] type=%d(VPDO) V_MAX=%d V_MIN=%d I_MAX=%d",
					(i == (sel_pdo_pos - 1)) ? '*' : ' ',
					i, pdo_obj.BITS_supply.type,
					v_max, v_min, i_max);
			break;
		default:
			msg_info("%cPDO[%d] type=%d(UNKNOWN)",
					(i == (sel_pdo_pos - 1)) ? '*' : ' ',
					i, pdo_obj.BITS_supply.type);
			break;
		}
	}

	usbc_data->pd_data->pdo_list = true;
}

static int max77978_get_chg_info(struct max77978_usbc_platform_data *usbc_data)
{
	usbc_cmd_data_t cmd_data;

	init_usbc_cmd_data(&cmd_data);
	cmd_data.opcode = OPCODE_SEND_GET_REQUEST;
	cmd_data.write_data[0] = 0;
	cmd_data.write_data[1] = 0;
	cmd_data.write_data[2] = 0;
	cmd_data.write_length = 3;
	cmd_data.read_length = 1;
	max77978_usbc_opcode_write(usbc_data, &cmd_data);

	return 0;
}

static void max77978_pd_check_pdmsg(struct max77978_usbc_platform_data *usbc_data, u8 pdmsg)
{
	switch (pdmsg) {
	case PDMSG_NOTHING_HAPPENED:
		msg_info("[0x%02X] PDMSG_NOTHING_HAPPENED", pdmsg);
		break;
	case PDMSG_SNK_PSRDY_RECEIVED:
		msg_info("[0x%02X] PDMSG_SNK_PSRDY_RECEIVED", pdmsg);
		max77978_get_chg_info(usbc_data);
		break;
	case PDMSG_SNK_ERROR_RECOVERY:
		msg_info("[0x%02X] PDMSG_SNK_ERROR_RECOVERY", pdmsg);
		break;
	case PDMSG_SNK_SENDERRESPONSETIMER_TIMEOUT:
		msg_info("[0x%02X] PDMSG_SNK_SENDERRESPONSETIMER_TIMEOUT", pdmsg);
		break;
	case PDMSG_SRC_PSRDY_SENT:
		msg_info("[0x%02X] PDMSG_SRC_PSRDY_SENT", pdmsg);
		if (usbc_data->pd_pr_swap == cc_SOURCE)
			max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		break;
	case PDMSG_SRC_ERROR_RECOVERY:
		msg_info("[0x%02X] PDMSG_SRC_PSRDY_SENT", pdmsg);
		break;
	case PDMSG_DR_SWAP_REQ_RECEIVED:
		msg_info("[0x%02X] PDMSG_DR_SWAP_REQ_RECEIVED", pdmsg);
		break;
	case PDMSG_PR_SWAP_REQ_RECEIVED:
		msg_info("[0x%02X] PDMSG_PR_SWAP_REQ_RECEIVED", pdmsg);
		break;
	case PDMSG_VCONN_SWAP_REQ_RECEIVED:
		msg_info("[0x%02X] PDMSG_VCONN_SWAP_REQ_RECEIVED", pdmsg);
		break;
	case PDMSG_VDM_ATTENTION_MSG_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_ATTENTION_MSG_RECEIVED", pdmsg);
		break;
	case PDMSG_REJECT_RECEIVED:
		msg_info("[0x%02X] PDMSG_REJECT_RECEIVED", pdmsg);
		break;
	case PDMSG_SNK_DISABLED:
		msg_info("[0x%02X] PDMSG_SNK_DISABLED", pdmsg);
		break;
	case PDMSG_SRC_DISABLED:
		msg_info("[0x%02X] PDMSG_SRC_DISABLED", pdmsg);
		break;
	case PDMSG_VDM_NAK_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_NAK_RECEIVED", pdmsg);
		break;
	case PDMSG_VDM_BUSY_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_BUSY_RECEIVED", pdmsg);
		break;
	case PDMSG_VDM_ACK_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_ACK_RECEIVED", pdmsg);
		break;
	case PDMSG_VDM_REQ_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_REQ_RECEIVED", pdmsg);
		break;
	case PDMSG_VDM_DISCOVERMODE_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_DISCOVERMODEs", pdmsg);
		break;
	case PDMSG_VDM_DP_STATUS_RECEIVED:
		msg_info("[0x%02X] PDMSG_VDM_DP_STATUS", pdmsg);
		break;
	case PDMSG_SRC_SENDERRESPONSETIMER_TIMEOUT:
		msg_info("[0x%02X] PDMSG_SRC_SENDERRESPONSETIMER_TIMEOUT", pdmsg);
		max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		schedule_delayed_work(&usbc_data->vbus_hard_reset_work, msecs_to_jiffies(800));
		break;
	case PDMSG_HARDRESET_RECEIVED:
		msg_info("[0x%02X] PDMSG_HARDRESET_RECEIVED", pdmsg);
		/*turn off the vbus both Source and Sink*/
		if (usbc_data->cc_data->current_port_type == TYPEC_PORT_SRC) {
			max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
			schedule_delayed_work(&usbc_data->vbus_hard_reset_work,
				msecs_to_jiffies(760));
		}
		break;
	case PDMSG_HARDRESET_SENT:
		msg_info("[0x%02X] PDMSG_HARDRESET_SENT", pdmsg);
		/*turn off the vbus both Source and Sink*/
		if (usbc_data->cc_data->current_port_type == TYPEC_PORT_SRC) {
			max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
			schedule_delayed_work(&usbc_data->vbus_hard_reset_work,
				msecs_to_jiffies(760));
		}
		break;
	case PDMSG_PRSWAP_SRCTOSWAP:
		msg_info("[0x%02X] PDMSG_PRSWAP_SRCTOSWAP", pdmsg);
		usbc_data->pd_data->psrdy_received = false;
		usbc_data->pd_data->pdo_list = false;
		max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		break;
	case PDMSG_PRSWAP_SWAPTOSNK:
		msg_info("[0x%02X] PDMSG_PRSWAP_SWAPTOSNK", pdmsg);
		max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		break;
	case PDMSG_PRSWAP_SNKTOSWAP:
		msg_info("[0x%02X] PDMSG_PRSWAP_SNKTOSWAP", pdmsg);
		usbc_data->pd_data->psrdy_received = false;
		usbc_data->pd_data->pdo_list = false;
		max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		/* CHGINSEL disable */
		break;
	case PDMSG_PRSWAP_SWAPTOSRC:
		max77978_vbus_turn_on_ctrl(usbc_data, VBUS_ON);
		msg_info("[0x%02X] PDMSG_PRSWAP_SWAPTOSRC", pdmsg);
		break;
	default:
		msg_info("[0x%02X] PDMSG not supported", pdmsg);
		break;
	}
}

static irqreturn_t max77978_pdmsg_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_pd_data *pd_data = usbc_data->pd_data;
	u8 pdmsg = 0;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_PD_STATUS0, &pd_data->pd_status0);
	pdmsg = pd_data->pd_status0;
	max77978_pd_check_pdmsg(usbc_data, pdmsg);
	pd_data->pdsmg = pdmsg;

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_psrdy_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	u8 psrdy_received = 0;
	enum typec_pwr_opmode mode = TYPEC_PWR_MODE_USB;

	msg_irq_handle_start(irq);
	max77978_read_reg(usbc_data->i2c, REG_PD_STATUS1, &usbc_data->pd_data->pd_status1);
	psrdy_received = (usbc_data->pd_data->pd_status1 & BIT_PSRDY) >> FFS(BIT_PSRDY);

	if (usbc_data->typec_try_state_change == TRY_ROLE_SWAP_PR &&
			usbc_data->pd_support) {
		msg_info("typec_reverse_completion");
		usbc_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;
		complete(&usbc_data->typec_reverse_completion);
	}

	msg_info("psrdy_received=%d, usbc_data->pd_support=%d, cc_status=%d",
			psrdy_received, usbc_data->pd_support, usbc_data->pd_data->cc_status);

	mode = max77978_get_pd_support(usbc_data);
	typec_set_pwr_opmode(usbc_data->port, mode);

	if (psrdy_received) {
		if (usbc_data->pd_data->cc_status == CC_SNK) {
			max77978_check_pdo(usbc_data);
			usbc_data->pd_data->psrdy_received = true;
		} else if (usbc_data->pd_data->cc_status == CC_SRC)
			msg_info("Sent the PSRDY_IRQ(Source)");
		else
			msg_info("Ignore the previous PSRDY_IRQ");
	}

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static void max77978_datarole_irq_handler(void *data, int irq)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_pd_data *pd_data = usbc_data->pd_data;
	u8 data_port_bit = 0;
	enum typec_port_data port_data = 0xFF;

	max77978_read_reg(usbc_data->i2c, REG_PD_STATUS1, &pd_data->pd_status1);
	data_port_bit = (pd_data->pd_status1 & BIT_DataRole) >> FFS(BIT_DataRole);
	port_data = (data_port_bit == 1) ? TYPEC_PORT_DFP : TYPEC_PORT_UFP;
	msg_info("data_port_bit=%d port_data=%d(%s)", data_port_bit, port_data, (port_data == TYPEC_PORT_DFP) ? "TYPEC_PORT_DFP" : "TYPEC_PORT_UFP");

	/* abnormal data role without setting power role */
	if (usbc_data->cc_data->current_port_type == 0xFF) {
		msg_info("invalid current_port_type");
		return;
	}

	if (irq == CCIC_IRQ_INIT_DETECT) {
		if (usbc_data->pd_data->cc_status == CC_SNK)
			msg_info("initial time : SNK");
		else
			return;
	}

	switch (port_data) {
	case TYPEC_PORT_UFP:
		if (pd_data->current_port_data != TYPEC_PORT_UFP) {
			pd_data->previous_port_data = pd_data->current_port_data;
			pd_data->current_port_data = TYPEC_PORT_UFP;
			if (pd_data->previous_port_data != 0xFF)
				msg_info("detach previous usb (DFP) connection");
			max77978_notify_dr_status(usbc_data, 1);
			if (usbc_data->typec_try_state_change == TRY_ROLE_SWAP_DR ||
					usbc_data->typec_try_state_change == TRY_ROLE_SWAP_TYPE) {
				msg_info("typec_reverse_completion");
				usbc_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;
				complete(&usbc_data->typec_reverse_completion);
			}
		}
		msg_info("TYPEC_PORT_UFP");
		break;

	case TYPEC_PORT_DFP:
		if (pd_data->current_port_data != TYPEC_PORT_DFP) {
			pd_data->previous_port_data = pd_data->current_port_data;
			pd_data->current_port_data = TYPEC_PORT_DFP;
			if (pd_data->previous_port_data != 0xFF)
				msg_info("detach previous usb (UFP) connection");
			max77978_notify_dr_status(usbc_data, 1);
			if (usbc_data->typec_try_state_change == TRY_ROLE_SWAP_DR ||
					usbc_data->typec_try_state_change == TRY_ROLE_SWAP_TYPE) {
				msg_info("typec_reverse_completion");
				usbc_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;
				complete(&usbc_data->typec_reverse_completion);
			}
		}
		msg_info(" DFP");
		break;

	default:
		msg_info(" DATAROLE(Never Call this routine)");
		break;
	}
}

static irqreturn_t max77978_datarole_irq(int irq, void *data)
{
	msg_irq_handle_start(irq);

	max77978_datarole_irq_handler(data, irq);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_vdm_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;

	msg_irq_handle_start(irq);

	max77978_receive_alternate_message(usbc_data);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

int max77978_pd_init(struct max77978_usbc_platform_data *usbc_data)
{
	struct max77978_pd_data *pd_data = NULL;
	int ret;

	pd_data = usbc_data->pd_data;

	pd_data->irq_pdmsg = usbc_data->irq_base + MAX77978_PD_IRQ_PDMSG_INT;
	if (pd_data->irq_pdmsg) {
		ret = request_threaded_irq(pd_data->irq_pdmsg,
				NULL, max77978_pdmsg_irq,
				IRQF_ONESHOT, "pd-pdmsg-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_PD_IRQ_PDMSG_INT, ret);
			goto err_irq_pdmsg;
		}
	}

	pd_data->irq_psrdy = usbc_data->irq_base + MAX77978_PD_IRQ_PS_RDY_INT;
	if (pd_data->irq_psrdy) {
		ret = request_threaded_irq(pd_data->irq_psrdy,
				NULL, max77978_psrdy_irq,
				IRQF_ONESHOT, "pd-psrdy-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_PD_IRQ_PS_RDY_INT, ret);
			goto err_irq_psrdy;
		}
	}

	pd_data->irq_datarole = usbc_data->irq_base + MAX77978_PD_IRQ_DATAROLE_INT;
	if (pd_data->irq_datarole) {
		ret = request_threaded_irq(pd_data->irq_datarole,
				NULL, max77978_datarole_irq,
				IRQF_ONESHOT, "pd-datarole-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_PD_IRQ_DATAROLE_INT, ret);
			goto err_irq_datarole;
		}
	}

	pd_data->irq_vdm = usbc_data->irq_base + MAX77978_PD_IRQ_DISPLAYPORT_INT;
	if (pd_data->irq_vdm) {
		ret = request_threaded_irq(pd_data->irq_vdm,
				NULL, max77978_vdm_irq,
				IRQF_ONESHOT, "pd-vdm-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_PD_IRQ_DISPLAYPORT_INT, ret);
			goto err_irq_vdm;
		}
	}

	msg_info("done (irq_pdmsg=%d irq_psrdy=%d irq_datarole=%d irq_vdm=%d)", pd_data->irq_pdmsg, pd_data->irq_psrdy, pd_data->irq_datarole, pd_data->irq_vdm);
	return 0;

err_irq_vdm:
	free_irq(pd_data->irq_datarole, usbc_data);
err_irq_datarole:
	free_irq(pd_data->irq_psrdy, usbc_data);
err_irq_psrdy:
	free_irq(pd_data->irq_pdmsg, usbc_data);
err_irq_pdmsg:
	msg_err("failed");
	return ret;
}

MODULE_DESCRIPTION("max77978 PD driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
