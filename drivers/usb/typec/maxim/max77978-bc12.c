// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 BC1.2 Charger Detection Driver
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
#include <linux/usb/typec/maxim/max77978-usbc.h>

static int max77978_bc12_set_charger(struct max77978_usbc_platform_data *usbc_data)
{
	/* TO DO : control charger */

	return 0;
}

static irqreturn_t max77978_vbadc_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_bc12_data *bc12_data = usbc_data->bc12_data;
	u16 vbadc = 0;

	msg_irq_handle_start(irq);
#ifdef USE_UIC_IRQ_VBADC
	msg_info("MAX77978_UIC_IRQ_VBADC_INT");
#else
	msg_info("MAX77978_UIC_IRQ_VBADC%c_INT", irq == bc12_data->irq_vbadch ? 'H' : 'L');
#endif

	max77978_read_reg(usbc_data->i2c, REG_VBUS_VOLTAGE_L, &bc12_data->vbvoltl);
	max77978_read_reg(usbc_data->i2c, REG_VBUS_VOLTAGE_H, &bc12_data->vbvolth);
	vbadc = ((bc12_data->vbvolth) << 8) | (bc12_data->vbvoltl);
	bc12_data->vbvolt = vbadc;

	/* print */
	if (vbadc < 140)
		msg_info("VBUS < 3.5V");
	else if (vbadc >= 140 && vbadc < 1100)
		msg_info("VBUS: %d.%dV", (vbadc * 25) / 1000, (vbadc * 25) % 1000);
	else if (vbadc >= 1100)
		msg_info("27.5V <= VBUS");
	else
		msg_err("VBUS Invalid Voltage Value");

	msg_irq_handle_complete(irq);

	return IRQ_HANDLED;
}

static irqreturn_t max77978_chgtype_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_bc12_data *bc12_data = usbc_data->bc12_data;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_BC_STATUS, &bc12_data->bc_status);
	bc12_data->chg_type = (bc12_data->bc_status & BIT_ChgTyp) >> FFS(BIT_ChgTyp);
	bc12_data->pr_chg_type = (bc12_data->bc_status & BIT_PrChgTyp) >> FFS(BIT_PrChgTyp);
	max77978_bc12_set_charger(usbc_data);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_dcdtmo_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_bc12_data *bc12_data = usbc_data->bc12_data;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_BC_STATUS, &bc12_data->bc_status);
	msg_info("BIT_DCDTmoI occured");
	bc12_data->dcdtmo = (bc12_data->bc_status & BIT_DCDTmo) >> FFS(BIT_DCDTmo);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_vbusdet_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	struct max77978_bc12_data *bc12_data = usbc_data->bc12_data;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_BC_STATUS, &bc12_data->bc_status);
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

int max77978_bc12_init(struct max77978_usbc_platform_data *usbc_data)
{
	struct max77978_bc12_data *bc12_data = NULL;
	int ret;

	bc12_data = usbc_data->bc12_data;

#ifdef USE_UIC_IRQ_VBADC
	bc12_data->irq_vbadc = usbc_data->irq_base + MAX77978_UIC_IRQ_VBADC_INT;
	if (bc12_data->irq_vbadc) {
		ret = request_threaded_irq(bc12_data->irq_vbadc,
				NULL, max77978_vbadc_irq,
				IRQF_ONESHOT, "usbc-vbadc-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_VBADC_INT, ret);
			goto err_irq_vbadc;
		}
	}
#else
	bc12_data->irq_vbadch = usbc_data->irq_base + MAX77978_UIC_IRQ_VBVOLTH_INT;
	if (bc12_data->irq_vbadch) {
		ret = request_threaded_irq(bc12_data->irq_vbadch,
				NULL, max77978_vbadc_irq,
				IRQF_ONESHOT, "usbc-vbadch-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_VBVOLTH_INT, ret);
			goto err_irq_vbadch;
		}
	}

	bc12_data->irq_vbadcl = usbc_data->irq_base + MAX77978_UIC_IRQ_VBVOLTL_INT;
	if (bc12_data->irq_vbadcl) {
		ret = request_threaded_irq(bc12_data->irq_vbadcl,
				NULL, max77978_vbadc_irq,
				IRQF_ONESHOT, "usbc-vbadc-irql", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_VBVOLTL_INT, ret);
			goto err_irq_vbadcl;
		}
	}
#endif

	bc12_data->irq_vbusdet = usbc_data->irq_base + MAX77978_UIC_IRQ_VBUS_INT;
	if (bc12_data->irq_vbusdet) {
		ret = request_threaded_irq(bc12_data->irq_vbusdet,
				NULL, max77978_vbusdet_irq,
				IRQF_ONESHOT, "bc-vbusdet-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_VBUS_INT, ret);
			goto err_irq_vbusdet;
		}
	}

	bc12_data->irq_dcdtmo = usbc_data->irq_base + MAX77978_UIC_IRQ_DCD_INT;
	if (bc12_data->irq_dcdtmo) {
		ret = request_threaded_irq(bc12_data->irq_dcdtmo,
				NULL, max77978_dcdtmo_irq,
				IRQF_ONESHOT, "bc-dcdtmo-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_DCD_INT, ret);
			goto err_irq_dcdtmo;
		}
	}

	bc12_data->irq_chgtype = usbc_data->irq_base + MAX77978_UIC_IRQ_CHGT_INT;
	if (bc12_data->irq_chgtype) {
		ret = request_threaded_irq(bc12_data->irq_chgtype,
				NULL, max77978_chgtype_irq,
				IRQF_ONESHOT, "bc-chgtype-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_CHGT_INT, ret);
			goto err_irq_chgtype;
		}
	}

#ifdef USE_UIC_IRQ_VBADC
	msg_info("done (irq_vbadc=%d irq_vbusdet=%d irq_dcdtmo=%d irq_chgtype=%d)", bc12_data->irq_vbadc, bc12_data->irq_vbusdet, bc12_data->irq_dcdtmo, bc12_data->irq_chgtype);
#else
	msg_info("done (irq_vbadch=%d irq_vbadcl=%d irq_vbusdet=%d irq_dcdtmo=%d irq_chgtype=%d)", bc12_data->irq_vbadch, bc12_data->irq_vbadcl, bc12_data->irq_vbusdet, bc12_data->irq_dcdtmo, bc12_data->irq_chgtype);
#endif

	return 0;

err_irq_chgtype:
	free_irq(bc12_data->irq_dcdtmo, usbc_data);
err_irq_dcdtmo:
	free_irq(bc12_data->irq_vbusdet, usbc_data);
err_irq_vbusdet:
#ifdef USE_UIC_IRQ_VBADC
	free_irq(bc12_data->irq_vbadc, usbc_data);
err_irq_vbadc:
#else
	free_irq(bc12_data->irq_vbadcl, usbc_data);
err_irq_vbadcl:
	free_irq(bc12_data->irq_vbadch, usbc_data);
err_irq_vbadch:
#endif
	msg_err("failed");
	return ret;
}

MODULE_DESCRIPTION("max77978 BC1.2 driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
