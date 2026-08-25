// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 CC Pin Driver
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
#include <linux/usb/typec/maxim/max77978-private.h>
#include <linux/platform_device.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>
#include <linux/usb/typec/maxim/max77978-alternate.h>

enum usb_status_e {
	USB_STATUS_NOTIFY_DETACH		= 0,
	USB_STATUS_NOTIFY_ATTACH_DFP	= 1,
	USB_STATUS_NOTIFY_ATTACH_UFP	= 2,
	USB_STATUS_NOTIFY_ATTACH_DRP	= 3,
	USB_STATUS_NUM,
	HOST_OFF	= 0,
	CLIENT_OFF	= 0,
	HOST_ON		= 1,
	CLIENT_ON	= 1,
};

static void max77978_ccic_event_work(void *data, int event)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct typec_partner_desc desc = { };
	enum typec_pwr_opmode mode = TYPEC_PWR_MODE_USB;

	msg_info("typec_power_role=%d(%s), typec_data_role=%d(%s), event=%d(%s)",
			usbpd_data->typec_power_role, usbpd_data->typec_power_role == TYPEC_SINK ? "TYPEC_SINK" : usbpd_data->typec_power_role == TYPEC_SOURCE ? "TYPEC_SOURCE" : "UNKNOWN",
			usbpd_data->typec_data_role, usbpd_data->typec_data_role == TYPEC_DEVICE ? "TYPEC_DEVICE" : usbpd_data->typec_data_role == TYPEC_HOST ? "TYPEC_HOST" : "UNKNOWN",
			event, event == USB_STATUS_NOTIFY_ATTACH_UFP ? "ATTACH_UFP" : event == USB_STATUS_NOTIFY_ATTACH_DFP ? "ATTACH_DFP" : "DETACH");
	if (usbpd_data->partner == NULL) {
		if (event == USB_STATUS_NOTIFY_ATTACH_UFP) {
			mode = max77978_get_pd_support(usbpd_data);
			typec_set_pwr_opmode(usbpd_data->port, mode);
			desc.usb_pd = mode == TYPEC_PWR_MODE_PD;
			desc.accessory = TYPEC_ACCESSORY_NONE;
			desc.identity = NULL;
			usbpd_data->typec_data_role = TYPEC_DEVICE;
			typec_set_pwr_role(usbpd_data->port, usbpd_data->typec_power_role);
			typec_set_data_role(usbpd_data->port, usbpd_data->typec_data_role);
			usbpd_data->partner = typec_register_partner(usbpd_data->port, &desc);
			if (IS_ERR(usbpd_data->partner))
				usbpd_data->partner = NULL;
		} else if (event == USB_STATUS_NOTIFY_ATTACH_DFP) {
			mode = max77978_get_pd_support(usbpd_data);
			typec_set_pwr_opmode(usbpd_data->port, mode);
			desc.usb_pd = mode == TYPEC_PWR_MODE_PD;
			desc.accessory = TYPEC_ACCESSORY_NONE;
			desc.identity = NULL;
			usbpd_data->typec_data_role = TYPEC_HOST;
			typec_set_pwr_role(usbpd_data->port, usbpd_data->typec_power_role);
			typec_set_data_role(usbpd_data->port, usbpd_data->typec_data_role);
			usbpd_data->partner = typec_register_partner(usbpd_data->port, &desc);
			if (IS_ERR(usbpd_data->partner))
				usbpd_data->partner = NULL;
		} else
			msg_info("detach case");
	} else {
		msg_info("data_role changed, typec_power_role=%d typec_data_role=%d, event=%d",
			usbpd_data->typec_power_role, usbpd_data->typec_data_role, event);
		if (event == USB_STATUS_NOTIFY_ATTACH_UFP) {
			usbpd_data->typec_data_role = TYPEC_DEVICE;
			typec_set_data_role(usbpd_data->port, usbpd_data->typec_data_role);
		} else if (event == USB_STATUS_NOTIFY_ATTACH_DFP) {
			usbpd_data->typec_data_role = TYPEC_HOST;
			typec_set_data_role(usbpd_data->port, usbpd_data->typec_data_role);
		} else
			msg_info("detach case");
	}
}

static void max77978_dp_detach(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;

	msg_info("dp_is_connect %d", usbpd_data->dp_is_connect);

	usbpd_data->dp_is_connect = 0;
	usbpd_data->dp_hs_connect = 0;
	usbpd_data->is_sent_pin_configuration = 0;
}

static void max77978_notify_cci_vbus_current(struct max77978_usbc_platform_data *usbc_data)
{
	/* TODO : control charger */
}

void max77978_notify_dr_status(struct max77978_usbc_platform_data *usbpd_data, uint8_t attach)
{
	struct max77978_pd_data *pd_data = usbpd_data->pd_data;

	msg_info("PortData=%d(%s) PortType=%d(%s) attach=%d(%s)",
			pd_data->current_port_data,
			pd_data->current_port_data == TYPEC_PORT_DFP ? "TYPEC_PORT_DFP" : pd_data->current_port_data == TYPEC_PORT_UFP ? "TYPEC_PORT_UFP" : "UNKNOWN",
			usbpd_data->cc_data->current_port_type,
			usbpd_data->cc_data->current_port_type == TYPEC_PORT_SRC ? "TYPEC_PORT_SRC" : usbpd_data->cc_data->current_port_type == TYPEC_PORT_SNK ? "TYPEC_PORT_SNK" : "UNKNOWN",
			attach, attach ? "ATTACHED":"DETACHED");

	if (attach) {
		if (usbpd_data->current_connstat == WATER) {
			msg_info("blocked by WATER");
			return;
		}
		if (usbpd_data->shut_down) {
			msg_info("blocked by shutdown");
			return;
		}
		if (pd_data->current_port_data == TYPEC_PORT_UFP) {
			if (usbpd_data->is_host == HOST_ON) {
				msg_info("turn off host");
				if (usbpd_data->dp_is_connect == 1)
					max77978_dp_detach(usbpd_data);
				/* USB */
				max77978_ccic_event_work(usbpd_data, USB_STATUS_NOTIFY_DETACH);
				usbpd_data->is_host = HOST_OFF;
			}
			if (usbpd_data->is_client == CLIENT_OFF) {
				usbpd_data->is_client = CLIENT_ON;
				/* USB */
				max77978_ccic_event_work(usbpd_data, USB_STATUS_NOTIFY_ATTACH_UFP);
			}
		} else if (pd_data->current_port_data == TYPEC_PORT_DFP) {
			if (usbpd_data->is_client == CLIENT_ON) {
				msg_info("turn off client");
				max77978_ccic_event_work(usbpd_data, USB_STATUS_NOTIFY_DETACH);
				usbpd_data->is_client = CLIENT_OFF;
			}
			if (usbpd_data->is_host == HOST_OFF) {
				usbpd_data->is_host = HOST_ON;
				/* USB */
				max77978_ccic_event_work(usbpd_data, USB_STATUS_NOTIFY_ATTACH_DFP);
			}
		}
	} else {
		if (usbpd_data->dp_is_connect == 1)
			max77978_dp_detach(usbpd_data);
		usbpd_data->is_host = HOST_OFF;
		usbpd_data->is_client = CLIENT_OFF;
		/* USB */
		max77978_ccic_event_work(usbpd_data, USB_STATUS_NOTIFY_DETACH/*drp*/);
	}
}

static irqreturn_t max77978_vconnocp_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;

	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS1, &cc_data->cc_status1);
	cc_data->vconnocp = (cc_data->cc_status1 & BIT_VCONNOCP) >> FFS(BIT_VCONNOCP);
	msg_info("New VCONNOCP Status Interrupt (%d)", cc_data->vconnocp);

	return IRQ_HANDLED;
}

static irqreturn_t max77978_vsafe0v_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_BC_STATUS, &cc_data->bc_status);
	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS0, &cc_data->cc_status0);
	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS1, &cc_data->cc_status1);
	cc_data->vsafe0v = (cc_data->cc_status1 & BIT_VSAFE0V) >> FFS(BIT_VSAFE0V);

	msg_info("New VSAFE0V Status Interrupt (vsafe0v=%d) | bcs=0x%02X ccs0=0x%02X ccs1=0x%02X",
			cc_data->vsafe0v,
			cc_data->bc_status,
			cc_data->cc_status0,
			cc_data->cc_status1);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_water_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;
	u8 waterstat = 0;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS1, &cc_data->cc_status1);
	waterstat = (cc_data->cc_status1 & BIT_Wtr) >> FFS(BIT_Wtr);

	switch (waterstat) {
	case DRY:
		msg_info("== WATER RUN-DRY DETECT ==");
		if (usbc_data->current_connstat != DRY) {
			usbc_data->prev_connstat = usbc_data->current_connstat;
			usbc_data->current_connstat = DRY;
		}
		break;

	case WATER:
		msg_info("== WATER DETECT ==");
		if (usbc_data->current_connstat != WATER) {
			usbc_data->prev_connstat = usbc_data->current_connstat;
			usbc_data->current_connstat = WATER;
		}
		break;
	default:
		break;

	}

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_ccpinstat_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;
	u8 ccpinstat = 0;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS0, &cc_data->cc_status0);
	ccpinstat = (cc_data->cc_status0 & BIT_CCPinStat) >> FFS(BIT_CCPinStat);

	switch (ccpinstat) {
	case NO_DETERMINATION:
		msg_info("CCPINSTAT (NO_DETERMINATION)");
		break;
	case CC1_ACTIVE:
		msg_info("CCPINSTAT (CC1_ACTIVE)");
		break;
	case CC2_ACTVIE:
		msg_info("CCPINSTAT (CC2_ACTIVE)");
		break;
	default:
		msg_info("CCPINSTAT [%d]", ccpinstat);
		break;
	}
	cc_data->ccpinstat = ccpinstat;

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_ccistat_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;
	u8 ccistat = 0;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS0, &cc_data->cc_status0);
	ccistat = (cc_data->cc_status0 & BIT_CCIStat) >> FFS(BIT_CCIStat);
	switch (ccistat) {
	case NOT_IN_UFP_MODE:
		msg_info("NOT_IN_UFP_MODE");
		break;
	case CCI_500mA:
		msg_info("CCISTAT CCI_500mA");
		break;
	case CCI_1_5A:
		msg_info("CCISTAT CCI_1_5A");
		break;
	case CCI_3_0A:
		msg_info("CCISTAT CCI_3_0A");
		break;
	default:
		msg_info("CCINSTAT ERROR (Never Called this routine)");
		break;
	}
	cc_data->ccistat = ccistat;

	max77978_notify_cci_vbus_current(usbc_data);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_ccvnstat_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;
	u8 ccvcnstat = 0;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS0, &cc_data->cc_status0);
	ccvcnstat = (cc_data->cc_status0 & BIT_CCVcnStat) >> FFS(BIT_CCVcnStat);
	switch (ccvcnstat) {
	case 0:
		msg_info("CCVNSTAT Vconn Disabled");
		if (cc_data->current_vcon != VCON_OFF) {
			cc_data->previous_vcon = cc_data->current_vcon;
			cc_data->current_vcon = VCON_OFF;
		}
		break;
	case 1:
		msg_info("CCVNSTAT Vconn Enabled");
		if (cc_data->current_vcon != VCON_ON) {
			cc_data->previous_vcon = cc_data->current_vcon;
			cc_data->current_vcon = VCON_ON;
		}
		break;
	default:
		msg_info("CCVNSTAT ERROR (Never Called this routine)");
		break;

	}
	cc_data->ccvcnstat = ccvcnstat;

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static void max77978_ccstat_irq_handler(void *data, int irq)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_cc_data *cc_data = usbc_data->cc_data;
	u8 ccstat = 0;
	enum typec_role prev_power_role = usbc_data->typec_power_role;

	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS0, &cc_data->cc_status0);
	ccstat =  (cc_data->cc_status0 & BIT_CCStat) >> FFS(BIT_CCStat);
	if (irq == CCIC_IRQ_INIT_DETECT) {
		if (ccstat == cc_SINK)
			msg_info("initial time : SNK");
		else
			return;
	}

	if (!ccstat) {
		if (usbc_data->plug_attach_done) {
			msg_info("PLUG_DETACHED ---");
			if (usbc_data->partner) {
				msg_info("ccstat : typec_unregister_partner");
				if (!IS_ERR(usbc_data->partner))
					typec_unregister_partner(usbc_data->partner);
				usbc_data->partner = NULL;
				usbc_data->typec_power_role = TYPEC_SINK;
				usbc_data->typec_data_role = TYPEC_DEVICE;
				usbc_data->pwr_opmode = TYPEC_PWR_MODE_USB;
			}
			if (usbc_data->typec_try_state_change == TRY_ROLE_SWAP_PR ||
					usbc_data->typec_try_state_change == TRY_ROLE_SWAP_DR) {
				/* Role change try and new mode detected */
				msg_info("typec_reverse_completion, detached while swapping");
				usbc_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;
				complete(&usbc_data->typec_reverse_completion);
			}
			usbc_data->plug_attach_done = 0;
			usbc_data->cc_data->current_port_type = 0xFF;
			usbc_data->pd_data->current_port_data = 0xFF;
			usbc_data->cc_data->current_vcon = 0xFF;
		}
	} else {
		if (!usbc_data->plug_attach_done) {
			msg_info("PLUG_ATTACHED +++");
			usbc_data->plug_attach_done = 1;
		}
	}

	switch (ccstat) {
	case cc_No_Connection:
		msg_info("CCSTAT cc_No_Connection");
		usbc_data->pd_data->cc_status = CC_NO_CONN;
		usbc_data->pd_support = false;
		if (!usbc_data->typec_try_state_change)
			max77978_usbc_clear_queue(usbc_data);

		usbc_data->typec_power_role = TYPEC_SINK;

		max77978_detach_pd(usbc_data);
		usbc_data->pd_pr_swap = cc_No_Connection;
		max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		cancel_delayed_work(&usbc_data->vbus_hard_reset_work);

		/* TODO : notify cc status to charger */
		break;
	case cc_SINK:
		msg_info("CCSTAT cc_SINK");
		/* keep awake during pdd communication */
		pm_wakeup_ws_event(cc_data->ccstat_ws, 1000, false);
		usbc_data->pd_data->cc_status = CC_SNK;
		usbc_data->typec_power_role = TYPEC_SINK;
		typec_set_pwr_role(usbc_data->port, TYPEC_SINK);
		if (cc_data->current_port_type != TYPEC_PORT_SNK) {
			cc_data->previous_port_type = cc_data->current_port_type;
			cc_data->current_port_type = TYPEC_PORT_SNK;
			if (prev_power_role == TYPEC_SOURCE)
				max77978_vbus_turn_on_ctrl(usbc_data, VBUS_OFF);
		}

		/* TODO : notify cc status to charger */
		break;
	case cc_SOURCE:
		msg_info("CCSTAT : cc_SOURCE");
		usbc_data->pd_data->cc_status = CC_SRC;
		usbc_data->typec_power_role = TYPEC_SOURCE;
		typec_set_pwr_role(usbc_data->port, TYPEC_SOURCE);
		if (cc_data->current_port_type != TYPEC_PORT_SRC) {
			cc_data->previous_port_type = cc_data->current_port_type;
			cc_data->current_port_type = TYPEC_PORT_SRC;

			if (prev_power_role == TYPEC_SINK)
				max77978_vbus_turn_on_ctrl(usbc_data, VBUS_ON);
		}
		break;
	case cc_Audio_Accessory:
		msg_info("CCSTAT cc_Audio_Accessory");
		break;
	case cc_Debug_Accessory:
		msg_info("CCSTAT cc_Debug_Accessory");
		break;
	case cc_Error:
		msg_info("CCSTAT cc_Error");
		break;
	case cc_Disabled:
		msg_info("CCSTAT cc_Disabled");
		break;
	case cc_Debug_Sink:
		msg_info("CCSTAT cc_Debug_Sink");
		break;
	default:
		msg_err("CCISTAT NOT_SUPPORTED(%d)", ccstat);
		break;
	}
}

static irqreturn_t max77978_ccstat_irq(int irq, void *data)
{
	msg_irq_handle_start(irq);

	max77978_ccstat_irq_handler(data, irq);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

int max77978_cc_init(struct max77978_usbc_platform_data *usbc_data)
{
	struct max77978_cc_data *cc_data = NULL;
	int ret;

	cc_data = usbc_data->cc_data;
	cc_data->ccstat_ws = wakeup_source_register(usbc_data->dev, "max77978-ccstat");
	if (!cc_data->ccstat_ws)
		return -ENOMEM;

	cc_data->irq_vconnocp = usbc_data->irq_base + MAX77978_CC_IRQ_VCONNCOP_INT;
	if (cc_data->irq_vconnocp) {
		ret = request_threaded_irq(cc_data->irq_vconnocp,
				NULL, max77978_vconnocp_irq,
				IRQF_ONESHOT, "cc-vconnocp-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_VCONNCOP_INT, ret);
			goto err_irq_vconnocp;
		}
	}

	cc_data->irq_vsafe0v = usbc_data->irq_base + MAX77978_CC_IRQ_VSAFE0V_INT;
	if (cc_data->irq_vsafe0v) {
		ret = request_threaded_irq(cc_data->irq_vsafe0v,
				NULL, max77978_vsafe0v_irq,
				IRQF_ONESHOT, "cc-vsafe0v-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_VSAFE0V_INT, ret);
			goto err_irq_vsafe0v;
		}
	}

	cc_data->irq_vconnsc = usbc_data->irq_base + MAX77978_CC_IRQ_VCONNSC_INT;
	if (cc_data->irq_vconnsc) {
		ret = request_threaded_irq(cc_data->irq_vconnsc,
				NULL, max77978_water_irq,
				IRQF_ONESHOT, "cc-wtr-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_VCONNSC_INT, ret);
			goto err_irq_vconnsc;
		}
	}
	cc_data->irq_ccpinstat = usbc_data->irq_base + MAX77978_CC_IRQ_CCPINSTAT_INT;
	if (cc_data->irq_ccpinstat) {
		ret = request_threaded_irq(cc_data->irq_ccpinstat,
				NULL, max77978_ccpinstat_irq,
				IRQF_ONESHOT, "cc-ccpinstat-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_CCPINSTAT_INT, ret);
			goto err_irq_ccpinstat;
		}
	}
	cc_data->irq_ccistat = usbc_data->irq_base + MAX77978_CC_IRQ_CCISTAT_INT;
	if (cc_data->irq_ccistat) {
		ret = request_threaded_irq(cc_data->irq_ccistat,
				NULL, max77978_ccistat_irq,
				IRQF_ONESHOT, "cc-ccistat-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_CCISTAT_INT, ret);
			goto err_irq_ccistat;
		}
	}
	cc_data->irq_ccvcnstat = usbc_data->irq_base + MAX77978_CC_IRQ_CCVCNSTAT_INT;
	if (cc_data->irq_ccvcnstat) {
		ret = request_threaded_irq(cc_data->irq_ccvcnstat,
				NULL, max77978_ccvnstat_irq,
				IRQF_ONESHOT, "cc-ccvcnstat-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_CCVCNSTAT_INT, ret);
			goto err_irq_ccvcnstat;
		}
	}
	cc_data->irq_ccstat = usbc_data->irq_base + MAX77978_CC_IRQ_CCSTAT_INT;
	if (cc_data->irq_ccstat) {
		ret = request_threaded_irq(cc_data->irq_ccstat,
				NULL, max77978_ccstat_irq,
				IRQF_ONESHOT, "cc-ccstat-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_CC_IRQ_CCSTAT_INT, ret);
			goto err_irq_ccstat;
		}
	}
	/* check CC Pin state for cable attach booting scenario */
	max77978_ccstat_irq_handler(usbc_data, CCIC_IRQ_INIT_DETECT);
	max77978_read_reg(usbc_data->i2c, REG_CC_STATUS1, &cc_data->cc_status1);
	usbc_data->current_connstat = (cc_data->cc_status1 & BIT_Wtr) >> FFS(BIT_Wtr);

	msg_info("done (irq_vconnocp=%d irq_vsafe0v=%d irq_vconnsc=%d irq_ccpinstat=%d irq_ccistat=%d irq_ccvcnstat=%d irq_ccstat=%d | connstat=%s)", cc_data->irq_vconnocp, cc_data->irq_vsafe0v, cc_data->irq_vconnsc, cc_data->irq_ccpinstat, cc_data->irq_ccistat, cc_data->irq_ccvcnstat, cc_data->irq_ccstat, usbc_data->current_connstat ? "WATER" : "DRY");

	return 0;

err_irq_ccstat:
	free_irq(cc_data->irq_ccvcnstat, usbc_data);
err_irq_ccvcnstat:
	free_irq(cc_data->irq_ccistat, usbc_data);
err_irq_ccistat:
	free_irq(cc_data->irq_ccpinstat, usbc_data);
err_irq_ccpinstat:
	free_irq(cc_data->irq_vconnsc, usbc_data);
err_irq_vconnsc:
	free_irq(cc_data->irq_vsafe0v, usbc_data);
err_irq_vsafe0v:
	free_irq(cc_data->irq_vconnocp, usbc_data);
err_irq_vconnocp:
	wakeup_source_unregister(cc_data->ccstat_ws);
	return ret;
}

MODULE_DESCRIPTION("max77978 CC driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
