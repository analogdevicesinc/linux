/*
 * Copyright 2015-2017 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * USB Type-C Port Controller Interface.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/usb/typec.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>

#include "pd.h"
#include "tcpci.h"
#include "tcpm.h"

#define PD_RETRY_COUNT 3

struct tcpci {
	struct device *dev;
	struct i2c_client *client;
	struct extcon_dev *edev;

	struct tcpm_port *port;

	struct regmap *regmap;

	bool controls_vbus;
	bool drive_vbus;
	bool sink_disable;
	struct gpio_desc *ss_sel_gpio;

	struct tcpc_dev tcpc;
	unsigned int irq_mask;
};

static const unsigned int tcpci_extcon_cable[] = {
	EXTCON_USB_HOST,
	EXTCON_USB,
	EXTCON_NONE,
};

static inline struct tcpci *tcpc_to_tcpci(struct tcpc_dev *tcpc)
{
	return container_of(tcpc, struct tcpci, tcpc);
}

static int tcpci_read16(struct tcpci *tcpci, unsigned int reg,
			unsigned int *val)
{
	return regmap_raw_read(tcpci->regmap, reg, val, sizeof(u16));
}

static int tcpci_write16(struct tcpci *tcpci, unsigned int reg, u16 val)
{
	return regmap_raw_write(tcpci->regmap, reg, &val, sizeof(u16));
}

static int tcpci_vbus_force_discharge(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	if (enable)
		regmap_write(tcpci->regmap,
			TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, 0x1c);
	else
		regmap_write(tcpci->regmap,
			TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, 0);

	regmap_read(tcpci->regmap, TCPC_POWER_CTRL, &reg);
	if (enable)
		reg |= TCPC_POWER_CTRL_FORCEDISCH;
	else
		reg &= ~TCPC_POWER_CTRL_FORCEDISCH;
	ret = regmap_write(tcpci->regmap, TCPC_POWER_CTRL, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	switch (cc) {
	case TYPEC_CC_RA:
		reg = (TCPC_ROLE_CTRL_CC_RA << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RA << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	case TYPEC_CC_RD:
		reg = (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	case TYPEC_CC_RP_DEF:
		reg = (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT) |
			(TCPC_ROLE_CTRL_RP_VAL_DEF <<
			 TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg = (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT) |
			(TCPC_ROLE_CTRL_RP_VAL_1_5 <<
			 TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg = (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT) |
			(TCPC_ROLE_CTRL_RP_VAL_3_0 <<
			 TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_OPEN:
	default:
		reg = (TCPC_ROLE_CTRL_CC_OPEN << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_OPEN << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	}

	ret = regmap_write(tcpci->regmap, TCPC_ROLE_CTRL, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_start_drp_toggling(struct tcpc_dev *tcpc,
			enum typec_cc_status cc, int attach)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg = 0;

	/* Only set DRP bit for auto toggle when unattached */
	if (attach) {
		switch (cc) {
		case TYPEC_CC_RP_DEF:
			if (attach >> TYPEC_POLARITY_CC2)
				reg |= TCPC_ROLE_CTRL_CC_RP <<
					TCPC_ROLE_CTRL_CC2_SHIFT;
			else if (attach >> TYPEC_POLARITY_CC1)
				reg |= TCPC_ROLE_CTRL_CC_RP <<
					TCPC_ROLE_CTRL_CC1_SHIFT;

			reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF <<
				TCPC_ROLE_CTRL_RP_VAL_SHIFT);
			break;
		case TYPEC_CC_RP_1_5:
			if (attach >> TYPEC_POLARITY_CC2)
				reg |= TCPC_ROLE_CTRL_CC_RP <<
					TCPC_ROLE_CTRL_CC2_SHIFT;
			else if (attach >> TYPEC_POLARITY_CC1)
				reg |= TCPC_ROLE_CTRL_CC_RP <<
					TCPC_ROLE_CTRL_CC1_SHIFT;

			reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 <<
				TCPC_ROLE_CTRL_RP_VAL_SHIFT);
			break;
		case TYPEC_CC_RP_3_0:
			if (attach >> TYPEC_POLARITY_CC2)
				reg |= TCPC_ROLE_CTRL_CC_RP <<
					TCPC_ROLE_CTRL_CC2_SHIFT;
			else if (attach >> TYPEC_POLARITY_CC1)
				reg |= TCPC_ROLE_CTRL_CC_RP <<
					TCPC_ROLE_CTRL_CC1_SHIFT;

			reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 <<
				TCPC_ROLE_CTRL_RP_VAL_SHIFT);
			break;
		case TYPEC_CC_RD:
			if (attach >> TYPEC_POLARITY_CC2)
				reg |= TCPC_ROLE_CTRL_CC_RD <<
					TCPC_ROLE_CTRL_CC2_SHIFT;
			else if (attach >> TYPEC_POLARITY_CC1)
				reg |= TCPC_ROLE_CTRL_CC_RD <<
					TCPC_ROLE_CTRL_CC1_SHIFT;
			break;
		default:
			break;
		}

		/* keep the un-touched cc line to be open */
		if (attach >> TYPEC_POLARITY_CC2)
			reg |= TCPC_ROLE_CTRL_CC_OPEN <<
				TCPC_ROLE_CTRL_CC1_SHIFT;
		else if (attach >> TYPEC_POLARITY_CC1)
			reg |= TCPC_ROLE_CTRL_CC_OPEN <<
				TCPC_ROLE_CTRL_CC2_SHIFT;
	} else { /* Not attached */
		if (cc == TYPEC_CC_RD)
			reg = TCPC_ROLE_CTRL_DRP | 0xa; /* Rd */
		else
			reg = TCPC_ROLE_CTRL_DRP | 0x5; /* Rp */
	}

	regmap_write(tcpci->regmap, TCPC_ROLE_CTRL, reg);

	if (!attach)
		regmap_write(tcpci->regmap, TCPC_COMMAND,
				TCPC_CMD_LOOK4CONNECTION);
	return 0;
}

static enum typec_cc_status tcpci_to_typec_cc(unsigned int cc, bool sink)
{
	switch (cc) {
	case 0x1:
		return sink ? TYPEC_CC_RP_DEF : TYPEC_CC_RA;
	case 0x2:
		return sink ? TYPEC_CC_RP_1_5 : TYPEC_CC_RD;
	case 0x3:
		if (sink)
			return TYPEC_CC_RP_3_0;
	case 0x0:
	default:
		return TYPEC_CC_OPEN;
	}
}

static int tcpci_get_cc(struct tcpc_dev *tcpc,
			enum typec_cc_status *cc1, enum typec_cc_status *cc2)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_CC_STATUS, &reg);
	if (ret < 0)
		return ret;

	*cc1 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC1_SHIFT) &
				 TCPC_CC_STATUS_CC1_MASK,
				 reg & TCPC_CC_STATUS_TERM);
	*cc2 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC2_SHIFT) &
				 TCPC_CC_STATUS_CC2_MASK,
				 reg & TCPC_CC_STATUS_TERM);

	return 0;
}

static int tcpci_set_polarity(struct tcpc_dev *tcpc,
			      enum typec_cc_polarity polarity)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;

	ret = regmap_write(tcpci->regmap, TCPC_TCPC_CTRL,
			   (polarity == TYPEC_POLARITY_CC2) ?
			   TCPC_TCPC_CTRL_ORIENTATION : 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_set_ss_mux(struct tcpc_dev *tcpc,
				enum typec_cc_polarity polarity)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);

	if (!tcpci->ss_sel_gpio)
		return 0;

	if (polarity == TYPEC_POLARITY_CC1)
		gpiod_set_value_cansleep(tcpci->ss_sel_gpio, 1);
	else
		gpiod_set_value_cansleep(tcpci->ss_sel_gpio, 0);

	return 0;
}

static int tcpci_set_vconn(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;

	ret = regmap_update_bits(tcpci->regmap, TCPC_POWER_CTRL,
				TCPC_POWER_CTRL_VCONN_ENABLE,
				enable ? TCPC_POWER_CTRL_VCONN_ENABLE : 0);
	return ret;
}

static int tcpci_set_roles(struct tcpc_dev *tcpc, bool attached,
			   enum typec_role role, enum typec_data_role data)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	reg = PD_REV20 << TCPC_MSG_HDR_INFO_REV_SHIFT;
	if (role == TYPEC_SOURCE)
		reg |= TCPC_MSG_HDR_INFO_PWR_ROLE;
	if (data == TYPEC_HOST)
		reg |= TCPC_MSG_HDR_INFO_DATA_ROLE;
	ret = regmap_write(tcpci->regmap, TCPC_MSG_HDR_INFO, reg);
	if (ret < 0)
		return ret;

	if (data == TYPEC_HOST)
		extcon_set_state_sync(tcpci->edev, EXTCON_USB_HOST, true);
	else
		extcon_set_state_sync(tcpci->edev, EXTCON_USB_HOST, false);

	return 0;
}

static int tcpci_set_pd_rx(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg = 0;
	int ret;

	if (enable)
		reg = TCPC_RX_DETECT_SOP | TCPC_RX_DETECT_HARD_RESET;
	ret = regmap_write(tcpci->regmap, TCPC_RX_DETECT, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_get_vbus(struct tcpc_dev *tcpc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);
	if (ret < 0)
		return ret;

	ret = !!(reg & TCPC_POWER_STATUS_VBUS_PRES);

	/*
	 * If the vbus is not from itself for source, we
	 * assume the vbus is from the port partner, this
	 * is to work around the case of connect to legacy
	 * Host like PC via a fixed Rp pull up cable, so
	 * we notify the possible EXTCON_USB connection.
	 */
	if (!tcpci->drive_vbus)
		extcon_set_state_sync(tcpci->edev, EXTCON_USB, ret);

	return ret;
}

static unsigned int tcpci_get_vbus_vol(struct tcpc_dev *tcpc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg, ret = 0;

	ret = regmap_read(tcpci->regmap, TCPC_VBUS_VOLTAGE, &reg);

	/* Convert it to be the vol number(mv) */
	ret = ((reg & TCPC_VBUS_VOL_MASK) <<
		((reg & TCPC_VBUS_VOL_SCALE_FACTOR_MASK) >>
		TCPC_VBUS_VOL_SCALE_FACTOR_SHIFT)) * TCPC_VBUS_VOL_MV_UNIT;

	return ret;
}

static int tcpci_set_vbus(struct tcpc_dev *tcpc, bool source, bool sink)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;

	/* Disable both source and sink first before enabling anything */

	if (!source) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_SRC_VBUS);
		if (ret < 0)
			return ret;

		tcpci->drive_vbus = false;
		/* Enable force discharge */
		tcpci_vbus_force_discharge(tcpc, true);
	}

	if (!sink) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_SINK_VBUS);
		if (ret < 0)
			return ret;
	}

	if (source) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_SRC_VBUS_DEFAULT);
		if (ret < 0)
			return ret;
		tcpci->drive_vbus = true;
	}

	if (sink && !tcpci->sink_disable) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_SINK_VBUS);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int tcpci_pd_transmit(struct tcpc_dev *tcpc,
			     enum tcpm_transmit_type type,
			     const struct pd_message *msg)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg, cnt, header;
	int ret;

	cnt = msg ? pd_header_cnt(msg->header) * 4 : 0;
	ret = regmap_write(tcpci->regmap, TCPC_TX_BYTE_CNT, cnt + 2);
	if (ret < 0)
		return ret;

	header = msg ? msg->header : 0;
	ret = tcpci_write16(tcpci, TCPC_TX_HDR, header);
	if (ret < 0)
		return ret;

	if (cnt > 0) {
		ret = regmap_raw_write(tcpci->regmap, TCPC_TX_DATA,
				       &msg->payload, cnt);
		if (ret < 0)
			return ret;
	}

	reg = (PD_RETRY_COUNT << TCPC_TRANSMIT_RETRY_SHIFT) |
		(type << TCPC_TRANSMIT_TYPE_SHIFT);
	ret = regmap_write(tcpci->regmap, TCPC_TRANSMIT, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int tcpci_vbus_detect(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	int ret;

	if (enable) {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_ENABLE_VBUS_DETECT);
		if (ret < 0)
			return ret;
	} else {
		ret = regmap_write(tcpci->regmap, TCPC_COMMAND,
				   TCPC_CMD_DISABLE_VBUS_DETECT);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void tcpci_bist_mode(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);

	regmap_update_bits(tcpci->regmap, TCPC_TCPC_CTRL,
			   TCPC_TCPC_CTRL_BIST_MODE,
			   enable ? TCPC_TCPC_CTRL_BIST_MODE : 0);
}

static int tcpci_init(struct tcpc_dev *tcpc)
{
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned long timeout = jiffies + msecs_to_jiffies(2000); /* XXX */
	unsigned int reg;
	int ret;

	while (time_before_eq(jiffies, timeout)) {
		ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);
		if (ret < 0)
			return ret;
		if (!(reg & TCPC_POWER_STATUS_UNINIT))
			break;
		usleep_range(10000, 20000);
	}
	if (time_after(jiffies, timeout))
		return -ETIMEDOUT;

	/* Clear all events */
	ret = tcpci_write16(tcpci, TCPC_ALERT, 0xffff);
	if (ret < 0)
		return ret;

	/* Clear fault condition */
	regmap_write(tcpci->regmap, TCPC_FAULT_STATUS, 0x80);

	if (tcpci->controls_vbus)
		reg = TCPC_POWER_STATUS_VBUS_PRES;
	else
		reg = 0;
	ret = regmap_write(tcpci->regmap, TCPC_POWER_STATUS_MASK, reg);
	if (ret < 0)
		return ret;

	/* Enable Voltage Alarms Power status reporting */
	regmap_read(tcpci->regmap, TCPC_POWER_CTRL, &reg);
	reg &= ~TCPC_POWER_CTRL_DIS_VOL_ALARM;
	ret = regmap_write(tcpci->regmap, TCPC_POWER_CTRL, reg);

	reg = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_FAILED |
		TCPC_ALERT_TX_DISCARDED | TCPC_ALERT_RX_STATUS |
		TCPC_ALERT_RX_HARD_RST | TCPC_ALERT_CC_STATUS |
		TCPC_ALERT_RX_BUF_OVF | TCPC_ALERT_FAULT |
		TCPC_ALERT_V_ALARM_LO;
	if (tcpci->controls_vbus)
		reg |= TCPC_ALERT_POWER_STATUS;
	tcpci->irq_mask = reg;

	return tcpci_write16(tcpci, TCPC_ALERT_MASK, reg);
}

static irqreturn_t tcpci_irq(int irq, void *dev_id)
{
	struct tcpci *tcpci = dev_id;
	unsigned int status, reg;

	tcpci_read16(tcpci, TCPC_ALERT, &status);

	/*
	 * Clear alert status for enabled irq except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 */
	if ((status & ~TCPC_ALERT_RX_STATUS) & tcpci->irq_mask)
		tcpci_write16(tcpci, TCPC_ALERT,
			      status & ~TCPC_ALERT_RX_STATUS);

	if (status & TCPC_ALERT_CC_STATUS)
		tcpm_cc_change(tcpci->port);

	if (status & TCPC_ALERT_POWER_STATUS) {
		/* Read power status to clear the event */
		regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);

		regmap_read(tcpci->regmap, TCPC_POWER_STATUS_MASK, &reg);

		/*
		 * If power status mask has been reset, then the TCPC
		 * has reset.
		 */
		if (reg == 0xff)
			tcpm_tcpc_reset(tcpci->port);
		else
			tcpm_vbus_change(tcpci->port);
	}

	if (status & TCPC_ALERT_V_ALARM_LO)
		tcpm_vbus_low_alarm(tcpci->port);

	if (status & TCPC_ALERT_RX_STATUS) {
		struct pd_message msg;
		unsigned int cnt;

		regmap_read(tcpci->regmap, TCPC_RX_BYTE_CNT, &cnt);

		tcpci_read16(tcpci, TCPC_RX_HDR, &reg);
		msg.header = reg;

		if (WARN_ON(cnt > sizeof(msg.payload)))
			cnt = sizeof(msg.payload);

		if (cnt > 0)
			regmap_raw_read(tcpci->regmap, TCPC_RX_DATA,
					&msg.payload, cnt);

		/* Read complete, clear RX status alert bit */
		tcpci_write16(tcpci, TCPC_ALERT, TCPC_ALERT_RX_STATUS);

		tcpm_pd_receive(tcpci->port, &msg);
	}

	if (status & TCPC_ALERT_RX_BUF_OVF)
		tcpci_write16(tcpci, TCPC_ALERT,
			TCPC_ALERT_RX_BUF_OVF | TCPC_ALERT_RX_STATUS);

	/* Clear the fault status anyway */
	if (status & TCPC_ALERT_FAULT) {
		regmap_read(tcpci->regmap, TCPC_FAULT_STATUS, &reg);
		regmap_write(tcpci->regmap, TCPC_FAULT_STATUS,
				reg | TCPC_FAULT_STATUS_CLEAR);
	}

	if (status & TCPC_ALERT_RX_HARD_RST)
		tcpm_pd_hard_reset(tcpci->port);

	if (status & TCPC_ALERT_TX_SUCCESS)
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_SUCCESS);
	else if (status & TCPC_ALERT_TX_DISCARDED)
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_DISCARDED);
	else if (status & TCPC_ALERT_TX_FAILED)
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_FAILED);

	return IRQ_HANDLED;
}

static const struct regmap_config tcpci_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x7F, /* 0x80 .. 0xFF are vendor defined */
};

static const struct tcpc_config tcpci_tcpc_config = {
	.type = TYPEC_PORT_DFP,
	.default_role = TYPEC_SINK,
};

/* Populate struct tcpc_config from ACPI/device-tree */
static int tcpci_parse_config(struct tcpci *tcpci)
{
	struct tcpc_config *tcfg;
	int ret = 0;

	tcpci->controls_vbus = true; /* XXX */

	/* Alloc tcpc_config struct */
	tcpci->tcpc.config = devm_kzalloc(tcpci->dev, sizeof(*tcfg),
							GFP_KERNEL);
	if (!tcpci->tcpc.config)
		return -ENOMEM;

	tcfg = tcpci->tcpc.config;

	/* Get the port-type */
	tcfg->type = typec_get_port_type(tcpci->dev);
	if (tcfg->type == TYPEC_PORT_TYPE_UNKNOWN) {
		dev_err(tcpci->dev, "typec port type is NOT correct!\n");
		return -EINVAL;
	}

	/* Get the default-role */
	tcfg->default_role = typec_get_power_role(tcpci->dev);
	if (tcfg->default_role == TYPEC_ROLE_UNKNOWN) {
		dev_err(tcpci->dev, "typec power role is NOT correct!\n");
		return -EINVAL;
	}

	/* Check source pdo array size */
	tcfg->nr_src_pdo = device_property_read_u32_array(tcpci->dev,
						"src-pdos", NULL, 0);
	if (tcfg->nr_src_pdo <= 0 && (tcfg->type == TYPEC_PORT_DRP ||
					tcfg->type == TYPEC_PORT_DFP)) {
		dev_err(tcpci->dev, "typec source pdo is missing!\n");
		return -EINVAL;
	}

	/* Alloc src_pdo based on the array size */
	tcfg->src_pdo = devm_kzalloc(tcpci->dev,
		sizeof(*tcfg->src_pdo) * tcfg->nr_src_pdo, GFP_KERNEL);
	if (!tcfg->src_pdo)
		return -ENOMEM;

	/* Read out source pdo array */
	ret = device_property_read_u32_array(tcpci->dev, "src-pdos",
				tcfg->src_pdo, tcfg->nr_src_pdo);
	if (ret) {
		dev_err(tcpci->dev, "Failed to read src pdo!\n");
		return -EINVAL;
	}

	/* Check the num of snk pdo */
	tcfg->nr_snk_pdo = device_property_read_u32_array(tcpci->dev,
						"snk-pdos", NULL, 0);
	if (tcfg->nr_snk_pdo <= 0 && (tcfg->type == TYPEC_PORT_DRP ||
					tcfg->type == TYPEC_PORT_UFP)) {
		dev_err(tcpci->dev, "typec sink pdo is missing!\n");
		return -EINVAL;
	}

	/* alloc snk_pdo based on the array size */
	tcfg->snk_pdo = devm_kzalloc(tcpci->dev,
		sizeof(*tcfg->snk_pdo) * tcfg->nr_snk_pdo, GFP_KERNEL);
	if (!tcfg->snk_pdo)
		return -ENOMEM;

	/* Read out sink pdo array */
	ret = device_property_read_u32_array(tcpci->dev, "snk-pdos",
				tcfg->snk_pdo, tcfg->nr_snk_pdo);
	if (ret) {
		dev_err(tcpci->dev, "Failed to read snk pdo!\n");
		return -EINVAL;
	}

	/* Get the max-snk-mv max-snk-ma op-snk-mw */
	if (device_property_read_u32(tcpci->dev, "max-snk-mv",
						&tcfg->max_snk_mv) ||
		device_property_read_u32(tcpci->dev, "max-snk-ma",
						&tcfg->max_snk_ma) ||
		device_property_read_u32(tcpci->dev, "max-snk-mw",
						&tcfg->max_snk_mw) ||
		device_property_read_u32(tcpci->dev, "op-snk-mw",
						&tcfg->operating_snk_mw))
		goto snk_setting_wrong;

	/*
	 * In case DRP only for data role, power role is source only
	 * we can use this property to disable power sink.
	 */
	if (device_property_read_bool(tcpci->dev, "sink-disable"))
		tcpci->sink_disable = true;

	return 0;

snk_setting_wrong:
	if (tcfg->type == TYPEC_PORT_DRP ||
			tcfg->type == TYPEC_PORT_UFP)
		dev_err(tcpci->dev, "Failed to read snk setting!\n");

	return ret;
}

static int tcpci_ss_mux_control_init(struct tcpci *tcpci)
{
	struct device *dev = tcpci->dev;
	struct gpio_desc *gpiod_reset;

	gpiod_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gpiod_reset)) {
		dev_err(dev, "Failed to request reset gpio.");
		return PTR_ERR(gpiod_reset);
	}

	if (gpiod_reset)
		usleep_range(700, 1000);

	tcpci->ss_sel_gpio = devm_gpiod_get_optional(dev, "ss-sel",
							GPIOD_OUT_HIGH);
	if (IS_ERR(tcpci->ss_sel_gpio)) {
		dev_err(dev, "Failed to request super speed mux sel gpio.");
		return PTR_ERR(tcpci->ss_sel_gpio);
	}

	return 0;
}

static int tcpci_probe(struct i2c_client *client,
		       const struct i2c_device_id *i2c_id)
{
	struct tcpci *tcpci;
	int err;

	tcpci = devm_kzalloc(&client->dev, sizeof(*tcpci), GFP_KERNEL);
	if (!tcpci)
		return -ENOMEM;

	tcpci->client = client;
	tcpci->dev = &client->dev;
	i2c_set_clientdata(client, tcpci);
	tcpci->regmap = devm_regmap_init_i2c(client, &tcpci_regmap_config);
	if (IS_ERR(tcpci->regmap))
		return PTR_ERR(tcpci->regmap);

	tcpci->tcpc.init = tcpci_init;
	tcpci->tcpc.get_vbus = tcpci_get_vbus;
	tcpci->tcpc.set_vbus = tcpci_set_vbus;
	tcpci->tcpc.set_cc = tcpci_set_cc;
	tcpci->tcpc.get_cc = tcpci_get_cc;
	tcpci->tcpc.set_polarity = tcpci_set_polarity;
	tcpci->tcpc.set_vconn = tcpci_set_vconn;
	tcpci->tcpc.start_drp_toggling = tcpci_start_drp_toggling;
	tcpci->tcpc.vbus_detect = tcpci_vbus_detect;
	tcpci->tcpc.vbus_discharge = tcpci_vbus_force_discharge;
	tcpci->tcpc.get_vbus_vol = tcpci_get_vbus_vol;
	tcpci->tcpc.bist_mode = tcpci_bist_mode;
	tcpci->tcpc.ss_mux_sel = tcpci_set_ss_mux;

	tcpci->tcpc.set_pd_rx = tcpci_set_pd_rx;
	tcpci->tcpc.set_roles = tcpci_set_roles;
	tcpci->tcpc.pd_transmit = tcpci_pd_transmit;

	/* Allocate extcon device */
	tcpci->edev = devm_extcon_dev_allocate(&client->dev,
					tcpci_extcon_cable);
	if (IS_ERR(tcpci->edev)) {
		dev_err(&client->dev, "failed to allocate extcon dev.\n");
		return -ENOMEM;
	}

	err = devm_extcon_dev_register(&client->dev, tcpci->edev);
	if (err) {
		dev_err(&client->dev, "failed to register extcon dev.\n");
		return err;
	}

	err = tcpci_parse_config(tcpci);
	if (err < 0)
		return err;

	tcpci->port = tcpm_register_port(tcpci->dev, &tcpci->tcpc);
	if (IS_ERR(tcpci->port))
		return PTR_ERR(tcpci->port);

	err = tcpci_ss_mux_control_init(tcpci);
	if (err)
		goto err1;

	err = devm_request_threaded_irq(tcpci->dev, client->irq, NULL,
					tcpci_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					dev_name(tcpci->dev), tcpci);
	if (err < 0)
		goto err1;

	device_set_wakeup_capable(tcpci->dev, true);

	return 0;
err1:
	tcpm_unregister_port(tcpci->port);
	return err;
}

static int tcpci_remove(struct i2c_client *client)
{
	struct tcpci *tcpci = i2c_get_clientdata(client);

	tcpm_unregister_port(tcpci->port);

	return 0;
}

static int tcpci_suspend(struct device *dev)
{
	struct tcpci *tcpci = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(tcpci->client->irq);

	return 0;
}

static int tcpci_resume(struct device *dev)
{
	struct tcpci *tcpci = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(tcpci->client->irq);

	return 0;
}

static const struct dev_pm_ops tcpci_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tcpci_suspend, tcpci_resume)
};

static const struct i2c_device_id tcpci_id[] = {
	{ "tcpci", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tcpci_id);

#ifdef CONFIG_OF
static const struct of_device_id tcpci_of_match[] = {
	{ .compatible = "usb,tcpci", },
	{},
};
MODULE_DEVICE_TABLE(of, tcpci_of_match);
#endif

static struct i2c_driver tcpci_i2c_driver = {
	.driver = {
		.name = "tcpci",
		.pm = &tcpci_pm_ops,
		.of_match_table = of_match_ptr(tcpci_of_match),
	},
	.probe = tcpci_probe,
	.remove = tcpci_remove,
	.id_table = tcpci_id,
};
module_i2c_driver(tcpci_i2c_driver);

MODULE_DESCRIPTION("USB Type-C Port Controller Interface driver");
MODULE_LICENSE("GPL");
