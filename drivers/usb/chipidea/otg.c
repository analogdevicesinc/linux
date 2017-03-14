/*
 * otg.c - ChipIdea USB IP core OTG driver
 *
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Author: Peter Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * This file mainly handles otgsc register, OTG fsm operations for HNP and SRP
 * are also included.
 */

#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/usb/chipidea.h>

#include "ci.h"
#include "bits.h"
#include "otg.h"
#include "otg_fsm.h"
#include "host.h"

/**
 * hw_read_otgsc returns otgsc register bits value.
 * @mask: bitfield mask
 */
u32 hw_read_otgsc(struct ci_hdrc *ci, u32 mask)
{
	struct ci_hdrc_cable *cable;
	u32 val = hw_read(ci, OP_OTGSC, mask);

	/*
	 * If using extcon framework for VBUS and/or ID signal
	 * detection overwrite OTGSC register value
	 */
	cable = &ci->platdata->vbus_extcon;
	if (!IS_ERR(cable->edev)) {
		if (cable->changed)
			val |= OTGSC_BSVIS;
		else
			val &= ~OTGSC_BSVIS;

		if (cable->connected)
			val |= OTGSC_BSV;
		else
			val &= ~OTGSC_BSV;

		if (cable->enabled)
			val |= OTGSC_BSVIE;
		else
			val &= ~OTGSC_BSVIE;
	}

	cable = &ci->platdata->id_extcon;
	if (!IS_ERR(cable->edev)) {
		if (cable->changed)
			val |= OTGSC_IDIS;
		else
			val &= ~OTGSC_IDIS;

		if (cable->connected)
			val &= ~OTGSC_ID; /* host */
		else
			val |= OTGSC_ID; /* device */

		if (cable->enabled)
			val |= OTGSC_IDIE;
		else
			val &= ~OTGSC_IDIE;
	}

	return val & mask;
}

/**
 * hw_write_otgsc updates target bits of OTGSC register.
 * @mask: bitfield mask
 * @data: to be written
 */
void hw_write_otgsc(struct ci_hdrc *ci, u32 mask, u32 data)
{
	struct ci_hdrc_cable *cable;

	cable = &ci->platdata->vbus_extcon;
	if (!IS_ERR(cable->edev)) {
		if (data & mask & OTGSC_BSVIS)
			cable->changed = false;

		/* Don't enable vbus interrupt if using external notifier */
		if (data & mask & OTGSC_BSVIE) {
			cable->enabled = true;
			data &= ~OTGSC_BSVIE;
		} else if (mask & OTGSC_BSVIE) {
			cable->enabled = false;
		}
	}

	cable = &ci->platdata->id_extcon;
	if (!IS_ERR(cable->edev)) {
		if (data & mask & OTGSC_IDIS)
			cable->changed = false;

		/* Don't enable id interrupt if using external notifier */
		if (data & mask & OTGSC_IDIE) {
			cable->enabled = true;
			data &= ~OTGSC_IDIE;
		} else if (mask & OTGSC_IDIE) {
			cable->enabled = false;
		}
	}

	hw_write(ci, OP_OTGSC, mask | OTGSC_INT_STATUS_BITS, data);
}

/**
 * ci_otg_role - pick role based on ID pin state
 * @ci: the controller
 */
enum ci_role ci_otg_role(struct ci_hdrc *ci)
{
	enum ci_role role = hw_read_otgsc(ci, OTGSC_ID)
		? CI_ROLE_GADGET
		: CI_ROLE_HOST;

	return role;
}

/*
 * Handling vbus glitch
 * We only need to consider glitch for without usb connection,
 * With usb connection, we consider it as real disconnection.
 *
 * If the vbus can't be kept above B session valid for timeout value,
 * we think it is a vbus glitch, otherwise it's a valid vbus.
 */
#define CI_VBUS_CONNECT_TIMEOUT_MS 300
static int ci_is_vbus_glitch(struct ci_hdrc *ci)
{
	int i;

	for (i = 0; i < CI_VBUS_CONNECT_TIMEOUT_MS/20; i++) {
		if (hw_read_otgsc(ci, OTGSC_AVV)) {
			return 0;
		} else if (!hw_read_otgsc(ci, OTGSC_BSV)) {
			dev_warn(ci->dev, "there is a vbus glitch\n");
			return 1;
		}
		msleep(20);
	}

	return 0;
}

void ci_handle_vbus_connected(struct ci_hdrc *ci)
{
	/*
	 * TODO: if the platform does not supply 5v to udc, or use other way
	 * to supply 5v, it needs to use other conditions to call
	 * usb_gadget_vbus_connect.
	 */
	if (!ci->is_otg)
		return;

	if (hw_read_otgsc(ci, OTGSC_BSV) && !ci_is_vbus_glitch(ci))
		usb_gadget_vbus_connect(&ci->gadget);
}

void ci_handle_vbus_change(struct ci_hdrc *ci)
{
	if (!ci->is_otg)
		return;

	if (hw_read_otgsc(ci, OTGSC_BSV) && !ci->vbus_active)
		usb_gadget_vbus_connect(&ci->gadget);
	else if (!hw_read_otgsc(ci, OTGSC_BSV) && ci->vbus_active)
		usb_gadget_vbus_disconnect(&ci->gadget);
}

/**
 * When we switch to device mode, the vbus value should be lower
 * than OTGSC_BSV before connecting to host.
 *
 * @ci: the controller
 *
 * This function returns an error code if timeout
 */
static int hw_wait_vbus_lower_bsv(struct ci_hdrc *ci)
{
	unsigned long elapse = jiffies + msecs_to_jiffies(5000);
	u32 mask = OTGSC_BSV;

	while (hw_read_otgsc(ci, mask)) {
		if (time_after(jiffies, elapse)) {
			dev_err(ci->dev, "timeout waiting for %08x in OTGSC\n",
					mask);
			return -ETIMEDOUT;
		}
		msleep(20);
	}

	return 0;
}

void ci_handle_id_switch(struct ci_hdrc *ci)
{
	enum ci_role role;
	int ret = 0;

	mutex_lock(&ci->mutex);
	role = ci_otg_role(ci);
	if (role != ci->role) {
		dev_dbg(ci->dev, "switching from %s to %s\n",
			ci_role(ci)->name, ci->roles[role]->name);

		ci_role_stop(ci);

		if (role == CI_ROLE_GADGET &&
				IS_ERR(ci->platdata->vbus_extcon.edev))
			/*
			 * Wait vbus lower than OTGSC_BSV before connecting
			 * to host. If connecting status is from an external
			 * connector instead of register, we don't need to
			 * care vbus on the board, since it will not affect
			 * external connector status.
			 */
			ret = hw_wait_vbus_lower_bsv(ci);
		else if (ci->vbus_active)
			/*
			 * If the role switch happens(e.g. during
			 * system sleep), and we lose vbus drop
			 * event, disconnect gadget for it before
			 * start host.
			 */
		       usb_gadget_vbus_disconnect(&ci->gadget);

		ci_role_start(ci, role);
		/* vbus change may have already occurred */
		if (role == CI_ROLE_GADGET)
			ci_handle_vbus_change(ci);

		/*
		 * If the role switch happens(e.g. during system
		 * sleep) and vbus keeps on afterwards, we connect
		 * gadget as vbus connect event lost.
		 */
		if (ret == -ETIMEDOUT)
			usb_gadget_vbus_connect(&ci->gadget);
	}
	mutex_unlock(&ci->mutex);
}

static void ci_handle_vbus_glitch(struct ci_hdrc *ci)
{
	bool valid_vbus_change = false;

	if (hw_read_otgsc(ci, OTGSC_BSV)) {
		if (!ci_is_vbus_glitch(ci)) {
			if (ci_otg_is_fsm_mode(ci)) {
				ci->fsm.b_sess_vld = 1;
				ci->fsm.b_ssend_srp = 0;
				otg_del_timer(&ci->fsm, B_SSEND_SRP);
				otg_del_timer(&ci->fsm, B_SRP_FAIL);
			}
			valid_vbus_change = true;
		}
	} else {
		if (ci->vbus_active && !ci_otg_is_fsm_mode(ci))
			valid_vbus_change = true;
	}

	if (valid_vbus_change) {
		ci->b_sess_valid_event = true;
		ci_otg_queue_work(ci);
	}
}

/**
 * ci_otg_work - perform otg (vbus/id) event handle
 * @work: work struct
 */
static void ci_otg_work(struct work_struct *work)
{
	struct ci_hdrc *ci = container_of(work, struct ci_hdrc, work);

	if (ci->vbus_glitch_check_event) {
		ci->vbus_glitch_check_event = false;
		pm_runtime_get_sync(ci->dev);
		ci_handle_vbus_glitch(ci);
		pm_runtime_put_sync(ci->dev);
		enable_irq(ci->irq);
		return;
	}

	if (ci_otg_is_fsm_mode(ci) && !ci_otg_fsm_work(ci)) {
		enable_irq(ci->irq);
		return;
	}

	pm_runtime_get_sync(ci->dev);
	if (ci->id_event) {
		ci->id_event = false;
		ci_handle_id_switch(ci);
	} else if (ci->b_sess_valid_event) {
		ci->b_sess_valid_event = false;
		ci_handle_vbus_change(ci);
	} else
		dev_err(ci->dev, "unexpected event occurs at %s\n", __func__);
	pm_runtime_put_sync(ci->dev);

	enable_irq(ci->irq);
}


/**
 * ci_hdrc_otg_init - initialize otg struct
 * ci: the controller
 */
int ci_hdrc_otg_init(struct ci_hdrc *ci)
{
	INIT_WORK(&ci->work, ci_otg_work);
	ci->wq = create_freezable_workqueue("ci_otg");
	if (!ci->wq) {
		dev_err(ci->dev, "can't create workqueue\n");
		return -ENODEV;
	}

	if (ci_otg_is_fsm_mode(ci))
		return ci_hdrc_otg_fsm_init(ci);

	return 0;
}

/**
 * ci_hdrc_otg_destroy - destroy otg struct
 * ci: the controller
 */
void ci_hdrc_otg_destroy(struct ci_hdrc *ci)
{
	/* Disable all OTG irq and clear status */
	hw_write_otgsc(ci, OTGSC_INT_EN_BITS | OTGSC_INT_STATUS_BITS,
						OTGSC_INT_STATUS_BITS);
	if (ci->wq) {
		flush_workqueue(ci->wq);
		destroy_workqueue(ci->wq);
		ci->wq = NULL;
	}
	if (ci_otg_is_fsm_mode(ci))
		ci_hdrc_otg_fsm_remove(ci);
}
