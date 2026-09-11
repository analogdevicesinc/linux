// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77958 BC1.2 Charger Detection Driver
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
#include <linux/usb/typec/maxim/max77958-private.h>
#include <linux/platform_device.h>
#include <linux/usb/typec/maxim/max77958-usbc.h>

static int max77958_bc12_set_charger(struct max77958_usbc_platform_data *usbc_data)
{
	/* TO DO : control charger */

#if IS_ENABLED(CONFIG_MAX77960_CHARGER)
	struct max77958_bc12_data *bc12_data = usbc_data->bc12_data;
	struct power_supply *psy_charger;
	union power_supply_propval value;

	psy_charger = power_supply_get_by_name("max77963-charger");

	msg_info("BIT_ChgTyp = 0x%02X, BIT_PrChgTyp = 0x%02X", bc12_data->chg_type, bc12_data->pr_chg_type);

	if (psy_charger) {
		switch (bc12_data->chg_type) {
		case CHGTYP_NOTHING:
			value.intval = POWER_SUPPLY_TYPE_BATTERY;
			msg_info("CHGTYP is NOTHING");
			break;
		case CHGTYP_USB_SDP:
		case CHGTYP_CDP:
			value.intval = POWER_SUPPLY_TYPE_USB;
			msg_info("CHGTYP is SDP or CDP (%d)", bc12_data->chg_type);
			break;
		case CHGTYP_DCP:
			msg_info("CHGTYP is DCP");
			value.intval = POWER_SUPPLY_TYPE_MAINS;
			break;
		default:
			value.intval = -EINVAL;
			break;
		}
		power_supply_set_property(psy_charger, POWER_SUPPLY_PROP_ONLINE, &value);
		power_supply_put(psy_charger);
	}
#endif
	return 0;
}

static irqreturn_t max77958_vbadc_irq(int irq, void *data)
{
	struct max77958_usbc_platform_data *usbc_data = data;
	struct max77958_bc12_data *bc12_data = usbc_data->bc12_data;
	u8 vbadc = 0;

	msg_irq_handle_start(irq);
	msg_info("MAX77958_UIC_IRQ_VBADC_INT");

	max77958_read_reg(usbc_data->i2c, REG_USBC_STATUS1,
		&bc12_data->usbc_status1);
	vbadc = (bc12_data->usbc_status1 & BIT_VBADC) >> FFS(BIT_VBADC);
	bc12_data->vbadc = vbadc;

	/* print */
	switch (vbadc) {
	case 0:
		msg_info(" VBUS < 3.5V");
		break;
	case 1:
		msg_info(" 3.5V <=  VBUS < 4.5V");
		break;
	case 2:
		msg_info(" 4.5V <=  VBUS < 5.5V ");
		break;
	case 3:
		msg_info(" 5.5V <=  VBUS < 6.5V");
		break;
	case 4:
		msg_info(" 6.5V <=  VBUS < 7.5V");
		break;
	case 5:
		msg_info(" 7.5V <= VBUS < 8.5V");
		break;
	case 6:
		msg_info(" 8.5V <= VBUS < 9.5V");
		break;
	case 7:
		msg_info(" 9.5V <= VBUS < 10.5V");
		break;
	case 8:
		msg_info(" 10.5V <= VBUS < 11.5V");
		break;
	case 9:
		msg_info(" 11.5V <= VBUS < 12.5V");
		break;
	case 10:
		msg_info(" 12.5V <= VBUS < 13.5V");
		break;
	case 11:
		msg_info(" 13.5V <= VBUS < 14.5V");
		break;
	case 12:
		msg_info(" 14.5V <= VBUS < 15.5V");
		break;
	case 13:
		msg_info(" 15.5V <= VBUS < 16.5V");
		break;
	case 14:
		msg_info(" 16.5V <= VBUS < 17.5V");
		break;
	case 15:
		msg_info(" 17.5V <= VBUS < 18.5V");
		break;
	case 16:
		msg_info(" 18.5V <= VBUS < 19.5V");
		break;
	case 17:
		msg_info(" 19.5V <= VBUS < 20.5V");
		break;
	case 18:
		msg_info(" 20.5V <= VBUS < 21.5V");
		break;
	case 19:
		msg_info(" 21.5V <= VBUS < 22.5V");
		break;
	case 20:
		msg_info(" 22.5V <= VBUS < 23.5V");
		break;
	case 21:
		msg_info(" 23.5V <= VBUS < 24.5V");
		break;
	case 22:
		msg_info(" 24.5V <= VBUS < 25.5V");
		break;
	case 23:
		msg_info(" 25.5V <= VBUS < 26.5V");
		break;
	case 24:
		msg_info(" 26.5V <= VBUS < 27.5V");
		break;
	case 25:
		msg_info(" 27.5V <= VBUS");
		break;
	default:
		msg_info(" INVALID VBADC (%d)", vbadc);
		break;
	};

	msg_irq_handle_complete(irq);

	return IRQ_HANDLED;
}

static irqreturn_t max77958_chgtype_irq(int irq, void *data)
{
	struct max77958_usbc_platform_data *usbc_data = data;
	struct max77958_bc12_data *bc12_data = usbc_data->bc12_data;

	msg_irq_handle_start(irq);

	max77958_read_reg(usbc_data->i2c, REG_BC_STATUS, &bc12_data->bc_status);
	bc12_data->chg_type = (bc12_data->bc_status & BIT_ChgTyp) >> FFS(BIT_ChgTyp);
	bc12_data->pr_chg_type = (bc12_data->bc_status & BIT_PrChgTyp) >> FFS(BIT_PrChgTyp);
	max77958_bc12_set_charger(usbc_data);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77958_dcdtmo_irq(int irq, void *data)
{
	struct max77958_usbc_platform_data *usbc_data = data;
	struct max77958_bc12_data *bc12_data = usbc_data->bc12_data;

	msg_irq_handle_start(irq);

	max77958_read_reg(usbc_data->i2c, REG_BC_STATUS, &bc12_data->bc_status);
	msg_info("BIT_DCDTmoI occured");
	bc12_data->dcdtmo = (bc12_data->bc_status & BIT_DCDTmo) >> FFS(BIT_DCDTmo);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77958_vbusdet_irq(int irq, void *data)
{
	struct max77958_usbc_platform_data *usbc_data = data;
	struct max77958_bc12_data *bc12_data = usbc_data->bc12_data;

	msg_irq_handle_start(irq);

	max77958_read_reg(usbc_data->i2c, REG_BC_STATUS, &bc12_data->bc_status);
	if ((bc12_data->bc_status & BIT_VBUSDet) == BIT_VBUSDet) {
		msg_info(" VBUS > VVBDET");
		bc12_data->vbusdet = 1;
	} else {
		msg_info(" VBUS < VVBDET");
		bc12_data->vbusdet = 0;
	}

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

int max77958_bc12_init(struct max77958_usbc_platform_data *usbc_data)
{
	struct max77958_bc12_data *bc12_data = NULL;
	int ret;

	bc12_data = usbc_data->bc12_data;

	bc12_data->irq_vbadc = usbc_data->irq_base + MAX77958_UIC_IRQ_VBADC_INT;
	if (bc12_data->irq_vbadc) {
		ret = request_threaded_irq(bc12_data->irq_vbadc,
				NULL, max77958_vbadc_irq,
				IRQF_ONESHOT, "usbc-vbadc-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77958_UIC_IRQ_VBADC_INT, ret);
			goto err_irq_vbadc;
		}
	}

	bc12_data->irq_vbusdet = usbc_data->irq_base + MAX77958_UIC_IRQ_VBUS_INT;
	if (bc12_data->irq_vbusdet) {
		ret = request_threaded_irq(bc12_data->irq_vbusdet,
				NULL, max77958_vbusdet_irq,
				IRQF_ONESHOT, "bc-vbusdet-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77958_UIC_IRQ_VBUS_INT, ret);
			goto err_irq_vbusdet;
		}
	}

	bc12_data->irq_dcdtmo = usbc_data->irq_base + MAX77958_UIC_IRQ_DCD_INT;
	if (bc12_data->irq_dcdtmo) {
		ret = request_threaded_irq(bc12_data->irq_dcdtmo,
				NULL, max77958_dcdtmo_irq,
				IRQF_ONESHOT, "bc-dcdtmo-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77958_UIC_IRQ_DCD_INT, ret);
			goto err_irq_dcdtmo;
		}
	}

	bc12_data->irq_chgtype = usbc_data->irq_base + MAX77958_UIC_IRQ_CHGT_INT;
	if (bc12_data->irq_chgtype) {
		ret = request_threaded_irq(bc12_data->irq_chgtype,
				NULL, max77958_chgtype_irq,
				IRQF_ONESHOT, "bc-chgtype-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77958_UIC_IRQ_CHGT_INT, ret);
			goto err_irq_chgtype;
		}
	}

	msg_info("done (irq_vbadc=%d irq_vbusdet=%d irq_dcdtmo=%d irq_chgtype=%d)", bc12_data->irq_vbadc, bc12_data->irq_vbusdet, bc12_data->irq_dcdtmo, bc12_data->irq_chgtype);

	return 0;

err_irq_chgtype:
	free_irq(bc12_data->irq_dcdtmo, usbc_data);
err_irq_dcdtmo:
	free_irq(bc12_data->irq_vbusdet, usbc_data);
err_irq_vbusdet:
	free_irq(bc12_data->irq_vbadc, usbc_data);
err_irq_vbadc:
	msg_err("failed");
	return ret;
}

MODULE_DESCRIPTION("max77958 BC1.2 driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
