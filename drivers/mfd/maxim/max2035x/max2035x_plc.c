// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX20355 PLC (Power Line Communication) driver
 * Shared PLC engine for MAX20355 (Master) and MAX20357 (Slave)
 */

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

#include "max2035x.h"
#include "max2035x_registers.h"
#include "max2035x_plc.h"
#define MAX2035X_RAM_SIZE	128

static const struct max2035x_plc_irq_map max20355_plc_irq_map[] = {
	{ 0, MAX20355_INT0_ITF_RDY_STS_BIT,     MAX20355_PLC_EVENT_ITF_READY },
	{ 0, MAX20355_INT0_CH1_CON_BIT,         MAX20355_PLC_EVENT_CH1_CONNECTED },
	{ 0, MAX20355_INT0_CH2_CON_BIT,         MAX20355_PLC_EVENT_CH2_CONNECTED },
	{ 0, MAX20355_INT0_CH1_IDL_BIT,         MAX20355_PLC_EVENT_CH1_IDLE },
	{ 0, MAX20355_INT0_CH2_IDL_BIT,         MAX20355_PLC_EVENT_CH2_IDLE },
	{ 0, MAX20355_INT0_MOI_DNE_BIT,         MAX20355_PLC_EVENT_MOI_DONE },
	{ 0, MAX20355_INT0_PLC2_MOI_DET_BIT,    MAX20355_PLC_EVENT_CH2_MOI_DETECTED },
	{ 0, MAX20355_INT0_PLC1_MOI_DET_BIT,    MAX20355_PLC_EVENT_CH1_MOI_DETECTED },

	{ 1, MAX20355_INT1_SYS_ERR_BIT,         MAX20355_PLC_EVENT_SYS_ERROR },
	{ 1, MAX20355_INT1_BB_FAULT_BIT,        MAX20355_PLC_EVENT_BB_FAULT },
	{ 1, MAX20355_INT1_THM_FLT_BIT,         MAX20355_PLC_EVENT_THERMAL_FAULT },
	{ 1, MAX20355_INT1_PLC_NEW_DAT_BIT,     MAX20355_PLC_EVENT_NEW_DATA },
	{ 1, MAX20355_INT1_PLC2_CMD_DNE_BIT,    MAX20355_PLC_EVENT_CH2_CMD_DONE },
	{ 1, MAX20355_INT1_PLC1_CMD_DNE_BIT,    MAX20355_PLC_EVENT_CH1_CMD_DONE },
	{ 1, MAX20355_INT1_PLC2_CMD_ERR_BIT,    MAX20355_PLC_EVENT_CH2_CMD_ERROR },
	{ 1, MAX20355_INT1_PLC1_CMD_ERR_BIT,    MAX20355_PLC_EVENT_CH1_CMD_ERROR },

	{ 2, MAX20355_INT2_MOI_DET_BIT,         MAX20355_PLC_EVENT_MOI_DETECTED_VALID_RESULT },
	{ 2, MAX20355_INT2_RES_DET_ABR_BIT,     MAX20355_PLC_EVENT_RESISTIVE_MEASURE_ABORT },
	{ 2, MAX20355_INT2_RES_DET_OPN_BIT,     MAX20355_PLC_EVENT_RESISTIVE_MEASURE_OPEN },
	{ 2, MAX20355_INT2_RES_DET_GND_BIT,     MAX20355_PLC_EVENT_RESISTIVE_MEASURE_GND },

	{ 3, MAX20355_INT3_URT_TMO_FLT2_BIT,    MAX20355_PLC_EVENT_CH2_UART_TIMEOUT },
	{ 3, MAX20355_INT3_URT_MODFAIL2_BIT,    MAX20355_PLC_EVENT_CH2_UART_MODE_FAIL },
	{ 3, MAX20355_INT3_URT_MODDONE2_BIT,    MAX20355_PLC_EVENT_CH2_UART_MODE_DONE },
	{ 3, MAX20355_INT3_URT_TMO_FLT1_BIT,    MAX20355_PLC_EVENT_CH1_UART_TIMEOUT },
	{ 3, MAX20355_INT3_URT_MODFAIL1_BIT,    MAX20355_PLC_EVENT_CH1_UART_MODE_FAIL },
	{ 3, MAX20355_INT3_URT_MODDONE1_BIT,    MAX20355_PLC_EVENT_CH1_UART_MODE_DONE },
};

static const struct max2035x_plc_irq_map max20357_plc_irq_map[] = {
	{ 0, MAX20357_INT0_PLC_SUMACT_BIT,      MAX20357_PLC_EVENT_SUM_CUR_LIMIT },
	{ 0, MAX20357_INT0_PLC_SUMCURR_BIT,     MAX20357_PLC_EVENT_SUM_CUR },
	{ 0, MAX20357_INT0_CHG_THRM_REG_BIT,    MAX20357_PLC_EVENT_CHG_THM_SHDN },
	{ 0, MAX20357_INT0_CC1_TMO_BIT,         MAX20357_PLC_EVENT_CC1_TIMEOUT },
	{ 0, MAX20357_INT0_CHGSTAT_BIT,         MAX20357_PLC_EVENT_CHG_MODE },
	{ 0, MAX20357_INT0_SYSMINREG_BIT,       MAX20357_PLC_EVENT_SYS_VOLT_REF },
	{ 0, MAX20357_INT0_CHG_RESTA_B_BIT,     MAX20357_PLC_EVENT_CHG_RESTART },
	{ 0, MAX20357_INT0_THMSTAT_BIT,         MAX20357_PLC_EVENT_JEITA_THM_MON },

	{ 1, MAX20357_INT1_JEITA_IS_REG_BIT,    MAX20357_PLC_EVENT_JEITA_CHG_CUR_VOLT },
	{ 1, MAX20357_INT1_CHG_REV_BIT,         MAX20357_PLC_EVENT_CHG_REV_PROT },
	{ 1, MAX20357_INT1_CHG_VOLT_MODE_BIT,   MAX20357_PLC_EVENT_CHG_BAT_VOLT_REG },
	{ 1, MAX20357_INT1_CHG_VOLT_STP_BIT,    MAX20357_PLC_EVENT_CHG_STEP_CHG },
	{ 1, MAX20357_INT1_CHG_GMD_BIT,         MAX20357_PLC_EVENT_CHG_DROPOUT },
	{ 1, MAX20357_INT1_LDO_GMD_BIT,         MAX20357_PLC_EVENT_SYS_LDO_DROPOUT },
	{ 1, MAX20357_INT1_PLCOk_BIT,           MAX20357_PLC_EVENT_PLC_VOLT },
	{ 1, MAX20357_INT1_SYSREV_BIT,          MAX20357_PLC_EVENT_SYS_LDO_REV_PROT },

	{ 2, MAX20357_INT2_CHN_CON_BIT,         MAX20357_PLC_EVENT_CONNECTION },
	{ 2, MAX20357_INT2_CHN_WTY_BIT,         MAX20357_PLC_EVENT_WAITING },
	{ 2, MAX20357_INT2_CHN_IDL_BIT,         MAX20357_PLC_EVENT_IDLE },
	{ 2, MAX20357_INT2_SRT_XFER_RISE_BIT,   MAX20357_PLC_EVENT_SHORT_XFER_RISING },
	{ 2, MAX20357_INT2_SRT_XFER_FALL_BIT,   MAX20357_PLC_EVENT_SHORT_XFER_FALLING },
	{ 2, MAX20357_INT2_PLC_NEW_DAT_BIT,     MAX20357_PLC_EVENT_NEW_DATA },
	{ 2, MAX20357_INT2_PLC_CMD_DNE_BIT,     MAX20357_PLC_EVENT_CMD_DONE },
	{ 2, MAX20357_INT2_PLC_CMD_ERR_BIT,     MAX20357_PLC_EVENT_CMD_ERROR },

	{ 3, MAX20357_INT3_LNG_XFER_BIT,        MAX20357_PLC_EVENT_LONG_XFER },
	{ 3, MAX20357_INT3_BATUVLOB_BIT,        MAX20357_PLC_EVENT_BAT_UVLO },
	{ 3, MAX20357_INT3_MOI_DNE_BIT,         MAX20357_PLC_EVENT_MOI_DONE },
	{ 3, MAX20357_INT3_PLC_MOI_DET_BIT,     MAX20357_PLC_EVENT_MOI_DETECTED },
	{ 3, MAX20357_INT3_MOI_DET_BIT,         MAX20357_PLC_EVENT_MOI_DETECTED_VALID_RESULT },
	{ 3, MAX20357_INT3_RES_DET_ABR_BIT,     MAX20357_PLC_EVENT_RESISTIVE_MEASURE_ABORT },
	{ 3, MAX20357_INT3_RES_DET_OPN_BIT,     MAX20357_PLC_EVENT_RESISTIVE_MEASURE_OPEN },
	{ 3, MAX20357_INT3_RES_DET_GND_BIT,     MAX20357_PLC_EVENT_RESISTIVE_MEASURE_GND },

	{ 4, MAX20357_INT4_URT_TMO_FLT_BIT,     MAX20357_PLC_EVENT_UART_TIMEOUT },
	{ 4, MAX20357_INT4_URT_MODFAIL_BIT,     MAX20357_PLC_EVENT_UART_MODE_FAIL },
	{ 4, MAX20357_INT4_URT_MODDONE_BIT,     MAX20357_PLC_EVENT_UART_MODE_DONE },
	{ 4, MAX20357_INT4_URT_SWC_OPN_BIT,     MAX20357_PLC_EVENT_UART_SWITCH_OPEN },
	{ 4, MAX20357_INT4_DEAD_FOUND_BIT,      MAX20357_PLC_EVENT_DEAD_MASTER },
	{ 4, MAX20357_INT4_SWC_OFF_MOD_BIT,     MAX20357_PLC_EVENT_CHG_SWITCH_OFF },
	{ 4, MAX20357_INT4_CHG_PRQ_INP_BIT,     MAX20357_PLC_EVENT_CHG_PRQ_INP },

	{ 5, MAX20357_INT5_ITF_RDY_STS_BIT,     MAX20357_PLC_EVENT_ITF_READY },
	{ 5, MAX20357_INT5_WD_ITR_CLR_BIT,      MAX20357_PLC_EVENT_WD_ITR_CLR },
};

static int max2035x_read_ram_data(struct max2035x_plc *plc, u8 *data, size_t len)
{
	if (len > MAX2035X_RAM_SIZE)
		return -EINVAL;

	return regmap_bulk_read(plc->ram_regmap, 0x00, data, len);
}

static int max2035x_initialize(struct max2035x *chip)
{
	if (chip->type == MAX20357)
		regmap_update_bits(chip->regmap, MAX20357_REG_PLC_CONFIG5,
				   MAX20357_PLC_CFG5_NO_UART_MDE_BIT, 0);

	return 0;
}

static int max20355_check_moisture_status(struct max2035x_plc *plc, int target_slave)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int ret, moi_status = 0;

	ret = regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
	if (ret)
		return ret;

	if (target_slave == 1)
		moi_status = reg_val & MAX20355_STATUS0_PLC1_MOI_DET_BIT;
	else if (target_slave == 2)
		moi_status = (reg_val & MAX20355_STATUS0_PLC2_MOI_DET_BIT) >> MAX20355_STATUS0_PLC2_MOI_DET_SHIFT;

	dev_dbg(plc->dev, "PLC_%d moisture %s (0x%02x)\n",
		target_slave, moi_status ? "detected" : "not detected", reg_val);

	return 0;
}

static int max20357_check_moisture_status(struct max2035x_plc *plc, int slave_num)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int ret, moi_status;

	ret = regmap_read(chip->regmap, MAX20357_REG_STATUS5, &reg_val);
	if (ret)
		return ret;

	moi_status = (reg_val & MAX20357_STATUS5_PLC_MOI_DET_BIT) >> MAX20357_STATUS5_PLC_MOI_DET_SHIFT;

	dev_dbg(plc->dev, "CH%d moisture %s (0x%02x)\n",
		slave_num, moi_status ? "detected" : "not detected", reg_val);

	return moi_status;
}

static int max20355_check_plc_status(struct max2035x_plc *plc, int target_slave)
{
	struct max2035x *chip = plc->chip;
	u8 status_reg = (target_slave == 1) ? MAX20355_REG_STATUS1 : MAX20355_REG_STATUS2;
	unsigned int reg_val;
	int ret;

	ret = regmap_read(chip->regmap, status_reg, &reg_val);
	if (ret)
		return ret;

	reg_val &= 0xFF;

	if (reg_val)
		dev_dbg(plc->dev, "PLC_%d cmd status 0x%02x\n", target_slave, reg_val);

	return reg_val;
}

static int max20357_check_plc_status(struct max2035x_plc *plc, int slave_num)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int ret;

	ret = regmap_read(chip->regmap, MAX20357_REG_STATUS4, &reg_val);
	if (ret)
		return ret;

	reg_val &= 0xFF;

	if (reg_val)
		dev_dbg(plc->dev, "CH%d cmd status 0x%02x\n", slave_num, reg_val);

	return reg_val;
}

static void max20355_plc_handle_events(struct max2035x_plc *plc, unsigned long events)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int ret;

	if (events & BIT(MAX20355_PLC_EVENT_ITF_READY)) {
		ret = regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
		if (!ret && (reg_val & MAX20355_STATUS0_ITF_RDY_STS_BIT))
			max2035x_initialize(chip);
	}

	if (events & BIT(MAX20355_PLC_EVENT_CH1_CONNECTED)) {
		ret = regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
		if (!ret)
			dev_dbg(plc->dev, "PLC_1 %s\n",
				(reg_val & MAX20355_STATUS0_CH1_CON_STS_BIT) ? "connected" : "disconnected");
	}

	if (events & BIT(MAX20355_PLC_EVENT_CH2_CONNECTED)) {
		ret = regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
		if (!ret)
			dev_dbg(plc->dev, "PLC_2 %s\n",
				(reg_val & MAX20355_STATUS0_CH2_CON_STS_BIT) ? "connected" : "disconnected");
	}

	if (events & BIT(MAX20355_PLC_EVENT_CH1_IDLE)) {
		ret = regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
		if (!ret)
			dev_dbg(plc->dev, "PLC_1 %s\n",
				(reg_val & MAX20355_STATUS0_CH1_IDL_STS_BIT) ? "idle" : "not idle");
	}

	if (events & BIT(MAX20355_PLC_EVENT_CH2_IDLE)) {
		ret = regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
		if (!ret)
			dev_dbg(plc->dev, "PLC_2 %s\n",
				(reg_val & MAX20355_STATUS0_CH2_IDL_STS_BIT) ? "idle" : "not idle");
	}

	if (events & BIT(MAX20355_PLC_EVENT_MOI_DONE))
		dev_dbg(plc->dev, "Moisture measurement completed\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH1_MOI_DETECTED))
		max20355_check_moisture_status(plc, 1);

	if (events & BIT(MAX20355_PLC_EVENT_CH2_MOI_DETECTED))
		max20355_check_moisture_status(plc, 2);

	if (events & BIT(MAX20355_PLC_EVENT_NEW_DATA)) {
		u8 rx_buf[MAX2035X_RAM_SIZE];

		max2035x_read_ram_data(plc, rx_buf, MAX2035X_RAM_SIZE);

		ret = regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG5, &reg_val);
		if (!ret && (reg_val & MAX20355_PLC_CFG5_RAM_IS_FULL_BIT)) {
			regmap_write(chip->regmap, MAX20355_REG_PLC_CONFIG5,
				     reg_val | MAX20355_PLC_CFG5_RAM_IS_FULL_BIT);
			ret = regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG5, &reg_val);
			if (!ret && (reg_val & MAX20355_PLC_CFG5_RAM_IS_FULL_BIT))
				dev_warn(plc->dev, "RAM_is_full not cleared (0x%02x)\n", reg_val);
		}
	}

	if (events & (BIT(MAX20355_PLC_EVENT_CH1_CMD_DONE) | BIT(MAX20355_PLC_EVENT_CH1_CMD_ERROR)))
		max20355_check_plc_status(plc, 1);

	if (events & (BIT(MAX20355_PLC_EVENT_CH2_CMD_DONE) | BIT(MAX20355_PLC_EVENT_CH2_CMD_ERROR)))
		max20355_check_plc_status(plc, 2);

	if (events & BIT(MAX20355_PLC_EVENT_MOI_DETECTED_VALID_RESULT))
		dev_dbg(plc->dev, "Valid moisture detection result\n");

	if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_ABORT))
		dev_dbg(plc->dev, "Resistive measurement aborted\n");

	if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_OPEN))
		dev_dbg(plc->dev, "Resistive measurement open\n");

	if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_GND))
		dev_dbg(plc->dev, "Resistive measurement ground\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_TIMEOUT))
		dev_dbg(plc->dev, "PLC_2 UART timeout\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_MODE_FAIL))
		dev_dbg(plc->dev, "PLC_2 UART mode entry failed\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_MODE_DONE))
		dev_dbg(plc->dev, "PLC_2 UART mode entry done\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_TIMEOUT))
		dev_dbg(plc->dev, "PLC_1 UART timeout\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_MODE_FAIL))
		dev_dbg(plc->dev, "PLC_1 UART mode entry failed\n");

	if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_MODE_DONE))
		dev_dbg(plc->dev, "PLC_1 UART mode entry done\n");
}

static void max20357_plc_handle_events(struct max2035x_plc *plc, unsigned long events)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int ret;

	if (events & BIT(MAX20357_PLC_EVENT_CONNECTION)) {
		ret = regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
		if (!ret)
			dev_dbg(plc->dev, "CH%d PLC %s\n", chip->channel_id,
				(reg_val & MAX20357_STATUS0_CHN_CON_STS_BIT) ? "connected" : "disconnected");
	}

	if (events & BIT(MAX20357_PLC_EVENT_IDLE)) {
		ret = regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
		if (!ret)
			dev_dbg(plc->dev, "CH%d PLC %s\n", chip->channel_id,
				(reg_val & MAX20357_STATUS0_CHN_IDL_STS_BIT) ? "idle" : "not idle");
	}

	if (events & BIT(MAX20357_PLC_EVENT_NEW_DATA)) {
		u8 rx_buf[MAX2035X_RAM_SIZE];

		max2035x_read_ram_data(plc, rx_buf, MAX2035X_RAM_SIZE);

		ret = regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);
		if (!ret && (reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT)) {
			regmap_write(chip->regmap, MAX20357_REG_PLC_CONFIG4,
				     reg_val | MAX20357_PLC_CFG4_RAM_IS_FULL_BIT);
			ret = regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);
			if (!ret && (reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT))
				dev_warn(plc->dev, "CH%d RAM_is_full not cleared (0x%02x)\n",
					 chip->channel_id, reg_val);
		}
	}

	if (events & (BIT(MAX20357_PLC_EVENT_CMD_DONE) | BIT(MAX20357_PLC_EVENT_CMD_ERROR)))
		max20357_check_plc_status(plc, chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_BAT_UVLO))
		dev_warn(plc->dev, "CH%d battery UVLO\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_MOI_DONE))
		dev_dbg(plc->dev, "CH%d moisture measurement completed\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_MOI_DETECTED))
		max20357_check_moisture_status(plc, chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_MOI_DETECTED_VALID_RESULT))
		dev_dbg(plc->dev, "CH%d valid moisture detection result\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_ABORT))
		dev_dbg(plc->dev, "CH%d resistive measurement aborted\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_OPEN))
		dev_dbg(plc->dev, "CH%d resistive measurement open\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_GND))
		dev_dbg(plc->dev, "CH%d resistive measurement ground\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_UART_TIMEOUT))
		dev_dbg(plc->dev, "CH%d UART timeout\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_UART_MODE_FAIL))
		dev_dbg(plc->dev, "CH%d UART mode entry failed\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_UART_MODE_DONE))
		dev_dbg(plc->dev, "CH%d UART mode entry done\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_UART_SWITCH_OPEN))
		dev_dbg(plc->dev, "CH%d UART switch open\n", chip->channel_id);

	if (events & BIT(MAX20357_PLC_EVENT_DEAD_MASTER)) {
		ret = regmap_read(chip->regmap, MAX20357_REG_STATUS2, &reg_val);
		if (!ret && (reg_val & MAX20357_STATUS2_DEAD_FOUND_STS_BIT))
			dev_warn(plc->dev, "CH%d dead master detected\n", chip->channel_id);
	}

	if (events & BIT(MAX20357_PLC_EVENT_ITF_READY)) {
		ret = regmap_read(chip->regmap, MAX20357_REG_STATUS5, &reg_val);
		if (!ret && (reg_val & MAX20357_STATUS5_ITF_RDY_STS_BIT))
			max2035x_initialize(chip);
	}
}

static irqreturn_t max2035x_plc_irq_handler(int irq, void *data)
{
	struct max2035x_plc_irq_data *irq_d = data;

	set_bit(irq_d->event, &irq_d->plc->pending_events);
	schedule_work(&irq_d->plc->work);

	return IRQ_HANDLED;
}

static void max2035x_plc_work(struct work_struct *work)
{
	struct max2035x_plc *plc = container_of(work, struct max2035x_plc, work);
	unsigned long events = xchg(&plc->pending_events, 0);

	if (events && plc->handle_events)
		plc->handle_events(plc, events);
}

static int max2035x_plc_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct max2035x_plc *plc;
	int i, ret, virq;

	plc = devm_kzalloc(&pdev->dev, sizeof(*plc), GFP_KERNEL);
	if (!plc)
		return -ENOMEM;

	plc->dev = &pdev->dev;
	plc->chip = chip;
	plc->ram_regmap = chip->ram_regmap;

	INIT_WORK(&plc->work, max2035x_plc_work);

	if (chip->type == MAX20355) {
		plc->irq_map = max20355_plc_irq_map;
		plc->irq_map_size = ARRAY_SIZE(max20355_plc_irq_map);
		plc->handle_events = max20355_plc_handle_events;
	} else {
		plc->irq_map = max20357_plc_irq_map;
		plc->irq_map_size = ARRAY_SIZE(max20357_plc_irq_map);
		plc->handle_events = max20357_plc_handle_events;
	}

	plc->irq_data = devm_kcalloc(&pdev->dev, plc->irq_map_size,
				     sizeof(*plc->irq_data), GFP_KERNEL);
	if (!plc->irq_data)
		return -ENOMEM;

	if (chip->irq_data) {
		for (i = 0; i < plc->irq_map_size; i++) {
			plc->irq_data[i].plc = plc;
			plc->irq_data[i].event = plc->irq_map[i].event;

			virq = regmap_irq_get_virq(chip->irq_data, i);
			if (virq < 0) {
				dev_err(&pdev->dev, "Failed to get virq for event %d\n", i);
				continue;
			}

			ret = devm_request_threaded_irq(&pdev->dev, virq, NULL,
							max2035x_plc_irq_handler,
							IRQF_ONESHOT,
							dev_name(&pdev->dev),
							&plc->irq_data[i]);
			if (ret)
				dev_warn(&pdev->dev, "Failed to request virq %d (event %d)\n",
					 virq, i);
		}
	}

	chip->plc_data = plc;

	max2035x_initialize(chip);

	platform_set_drvdata(pdev, plc);

	max2035x_debugfs_init(plc);

	return 0;
}

static void max2035x_plc_remove(struct platform_device *pdev)
{
	struct max2035x_plc *plc = platform_get_drvdata(pdev);

	if (!plc)
		return;

	max2035x_debugfs_exit(plc);
}

static struct platform_driver max20355_plc_driver = {
	.driver = {
		.name = "max20355-plc",
	},
	.probe = max2035x_plc_probe,
	.remove = max2035x_plc_remove,
};

static struct platform_driver max20357_plc_driver = {
	.driver = {
		.name = "max20357-plc",
	},
	.probe = max2035x_plc_probe,
	.remove = max2035x_plc_remove,
};

static struct platform_driver * const plc_drivers[] = {
	&max20355_plc_driver,
	&max20357_plc_driver,
};

int __init max2035x_plc_init(void)
{
	return platform_register_drivers(plc_drivers, ARRAY_SIZE(plc_drivers));
}

void max2035x_plc_exit(void)
{
	platform_unregister_drivers(plc_drivers, ARRAY_SIZE(plc_drivers));
}
