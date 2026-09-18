// SPDX-License-Identifier: GPL-2.0-or-later
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

/* -------------------------------------------------------------------------- */
/* PLC IRQ → event mapping tables                                             */
/* -------------------------------------------------------------------------- */
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
	int moi_status = 0;

	if (regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val)) {
		dev_err(plc->dev, "%s : Failed to read MAX20355_REG_STATUS0\n", __func__);
		return -EIO;
	}

	if (target_slave == 1) {
		moi_status = reg_val & MAX20355_STATUS0_PLC1_MOI_DET_BIT;
	} else if (target_slave == 2) {
		moi_status = (reg_val & MAX20355_STATUS0_PLC2_MOI_DET_BIT) >> MAX20355_STATUS0_PLC2_MOI_DET_SHIFT;
	}

	dev_info(plc->dev, "%s : PLC_%d Moisture %s (0x%02x)\n",
		 __func__, target_slave,
		 moi_status ? "Detected" : "Not Detected", reg_val);

	return 0;
}

static int max20357_check_moisture_status(struct max2035x_plc *plc, int slave_num)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int moi_status = 0;

	if (regmap_read(chip->regmap, MAX20357_REG_STATUS5, &reg_val)) {
		dev_err(plc->dev, "%s : Failed to read MAX20357_REG_STATUS5\n", __func__);
		return -EIO;
	}

	moi_status = (reg_val & MAX20357_STATUS5_PLC_MOI_DET_BIT) >> MAX20357_STATUS5_PLC_MOI_DET_SHIFT;

	dev_info(plc->dev, "%s : [MAX20357_CH%d] Moisture %s (0x%02x)\n",
		 __func__, slave_num,
		 moi_status ? "Detected" : "Not Detected", reg_val);

	return moi_status;
}

static int max20355_check_plc_status(struct max2035x_plc *plc, int target_slave)
{
	struct max2035x *chip = plc->chip;
	u8 status_reg = (target_slave == 1) ? MAX20355_REG_STATUS1 : MAX20355_REG_STATUS2;
	unsigned int reg_val;
	enum max2035x_plc_error status;

	if (regmap_read(chip->regmap, status_reg, &reg_val)) {
		dev_err(plc->dev, "%s : Failed to read MAX20355_REG_STATUS%d\n", __func__, target_slave);
		return -EIO;
	}

	status = (enum max2035x_plc_error)(reg_val & 0xFF);

	switch (status) {
	case MAX2035X_PLC_ERR_NO_ERROR:
		dev_dbg(plc->dev, "%s : [MAX20355] CMD_DONE : NO_PLC_ERROR (PLC_%d)\n", __func__, target_slave);
		break;
	case MAX2035X_PLC_ERR_NACK_TO_CMD:
		dev_info(plc->dev, "%s : [MAX20355] CMD_DONE : NACK_TO_CMD (PLC_%d)\n", __func__, target_slave);
		break;
	case MAX2035X_PLC_ERR_BAD_CMD_ID:
		dev_info(plc->dev, "%s : [MAX20355] CMD_DONE : BAD_CMD_ID (PLC_%d)\n", __func__, target_slave);
		break;
	case MAX2035X_PLC_ERR_CMD_ERROR:
		dev_info(plc->dev, "%s : [MAX20355] CMD_DONE : CMD_ERROR (PLC_%d)\n", __func__, target_slave);
		break;
	case MAX2035X_PLC_ERR_NAK_LIMIT:
		dev_info(plc->dev, "%s : [MAX20355] CMD_DONE : NAK_LIMIT (PLC_%d)\n", __func__, target_slave);
		break;
	default:
		dev_info(plc->dev, "%s : [MAX20355] CMD_DONE : Unknown 0x%02x (PLC_%d)\n", __func__, status, target_slave);
		break;
	}

	return (int)status;
}

static int max20357_check_plc_status(struct max2035x_plc *plc, int slave_num)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	enum max2035x_plc_error status;

	if (regmap_read(chip->regmap, MAX20357_REG_STATUS4, &reg_val)) {
		dev_err(plc->dev, "%s : Failed to read MAX20357_REG_STATUS0\n", __func__);
		return -EIO;
	}

	status = (enum max2035x_plc_error)(reg_val & 0xFF);

	switch (status) {
	case MAX2035X_PLC_ERR_NO_ERROR:
		dev_dbg(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : NO_PLC_ERROR\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_NACK_TO_CMD:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : NACK_TO_CMD\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_NACK_TO_DATA:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : NACK_TO_DATA\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_NOPLC_ON_CMD:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : NO_PLC_ON_CMD\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_NACK_N_DOUTR:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : NACK_N_DOUTR\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_ERR_N_DOUTR:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : ERR_N_DOUTR\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_MST_TMO_ERR:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : MST_TMO_ERR\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_PLC_BUSY_ERR:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : PLC_BUSY_ERR\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_TX01_TMO_ERR:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : TX01_TMO_ERR\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_BAD_CMD_TX:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : BAD_CMD_TX\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_BAD_CMD_RX:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : BAD_CMD_RX\n", __func__, slave_num);
		break;
	case MAX2035X_PLC_ERR_PLC_DATA_ERR:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : PLC_DATA_ERR\n", __func__, slave_num);
		break;
	default:
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : Unknown 0x%02x\n", __func__, slave_num, status);
		break;
	}

	return (int)status;
}

static void max20355_plc_handle_events(struct max2035x_plc *plc, unsigned long events)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;

	if (events & BIT(MAX20355_PLC_EVENT_ITF_READY)) {
			regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
			if (reg_val & MAX20355_STATUS0_ITF_RDY_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20355] OTP loading completed\n", __func__);
				max2035x_initialize(chip);
			} else {
				dev_info(plc->dev, "%s : [MAX20355] OTP loading not completed\n", __func__);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_CONNECTED)) {
			regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
			if (reg_val & MAX20355_STATUS0_CH1_CON_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20355] PLC_1 Connected\n", __func__);
			} else {
				dev_info(plc->dev, "%s : [MAX20355] PLC_1 Disconnected\n", __func__);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_CONNECTED)) {
			regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
			if (reg_val & MAX20355_STATUS0_CH2_CON_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20355] PLC_2 Connected\n", __func__);
			} else {
				dev_info(plc->dev, "%s : [MAX20355] PLC_2 Disconnected\n", __func__);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_IDLE)) {
			regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
			if (reg_val & MAX20355_STATUS0_CH1_IDL_STS_BIT) {
				dev_dbg(plc->dev, "%s : [MAX20355] PLC_1 IDLE State\n", __func__);
			} else {
				dev_dbg(plc->dev, "%s : [MAX20355] PLC_1 Not in IDLE State\n", __func__);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_IDLE)) {
			regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
			if (reg_val & MAX20355_STATUS0_CH2_IDL_STS_BIT) {
				dev_dbg(plc->dev, "%s : [MAX20355] PLC_2 IDLE State\n", __func__);
			} else {
				dev_dbg(plc->dev, "%s : [MAX20355] PLC_2 Not in IDLE State\n", __func__);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_MOI_DONE)) {
			dev_dbg(plc->dev, "%s : [MAX20355] Moisture measurement is completed\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_MOI_DETECTED)) {
			max20355_check_moisture_status(plc, 1);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_MOI_DETECTED)) {
			max20355_check_moisture_status(plc, 2);
		}

		if (events & BIT(MAX20355_PLC_EVENT_NEW_DATA)) {
			u8 rx_buf[MAX2035X_RAM_SIZE];

			dev_dbg(plc->dev, "%s : [MAX20355] New PLC Data Received\n", __func__);
			max2035x_read_ram_data(plc, rx_buf, MAX2035X_RAM_SIZE);

			regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG5, &reg_val);
			if (reg_val & MAX20355_PLC_CFG5_RAM_IS_FULL_BIT) {
				regmap_write(chip->regmap, MAX20355_REG_PLC_CONFIG5, reg_val | MAX20355_PLC_CFG5_RAM_IS_FULL_BIT);
				regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG5, &reg_val);

				if (!(reg_val & MAX20355_PLC_CFG5_RAM_IS_FULL_BIT)) {
					dev_dbg(plc->dev, "%s : [MAX20355] Mailbox RAM cleared (0x%02x)\n",
					 __func__, reg_val);
				} else {
					dev_warn(plc->dev, "%s : [MAX20355] Mailbox but RAM_is_full not cleared (0x%02x)\n",
					 __func__, reg_val);
				}
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_CMD_DONE) || events & BIT(MAX20355_PLC_EVENT_CH1_CMD_ERROR)) {
			max20355_check_plc_status(plc, 1);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_CMD_DONE) || events & BIT(MAX20355_PLC_EVENT_CH2_CMD_ERROR)) {
			max20355_check_plc_status(plc, 2);
		}

		if (events & BIT(MAX20355_PLC_EVENT_MOI_DETECTED_VALID_RESULT)) {
			dev_dbg(chip->dev, "%s: [MAX20355] Valid Moisture detection Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_ABORT)) {
			dev_dbg(chip->dev, "%s: [MAX20355] Abort resistive Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_OPEN)) {
			dev_dbg(chip->dev, "%s: [MAX20355] Open resistive Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_GND)) {
			dev_dbg(chip->dev, "%s: [MAX20355] Ground resistive Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_TIMEOUT)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_2 UART Timeout\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_MODE_FAIL)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_2 UART Mode Entry Failed\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_MODE_DONE)) {
			dev_dbg(chip->dev, "%s: [MAX20355] PLC_2 UART Mode Entry Successful\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_TIMEOUT)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_1 UART Timeout\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_MODE_FAIL)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_1 UART Mode Entry Failed\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_MODE_DONE)) {
			dev_dbg(chip->dev, "%s: [MAX20355] PLC_1 UART Mode Entry Successful\n", __func__);
		}
}

static void max20357_plc_handle_events(struct max2035x_plc *plc, unsigned long events)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;

	if (events & BIT(MAX20357_PLC_EVENT_CONNECTION)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
			if (reg_val & MAX20357_STATUS0_CHN_CON_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] PLC Connected\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] PLC Disconnected\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_IDLE)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
			if (reg_val & MAX20357_STATUS0_CHN_IDL_STS_BIT) {
				dev_dbg(plc->dev, "%s : [MAX20357_CH%d] PLC IDLE State\n", __func__, chip->channel_id);
			} else {
				dev_dbg(plc->dev, "%s : [MAX20357_CH%d] PLC Not in IDLE State\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_NEW_DATA)) {
			u8 rx_buf[MAX2035X_RAM_SIZE];

			dev_dbg(plc->dev, "%s : [MAX20357_CH%d] New PLC Data Received\n", __func__, chip->channel_id);
			max2035x_read_ram_data(plc, rx_buf, MAX2035X_RAM_SIZE);

			regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);
			if (reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT) {
				regmap_write(chip->regmap, MAX20357_REG_PLC_CONFIG4, reg_val | MAX20357_PLC_CFG4_RAM_IS_FULL_BIT);
				regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);

				if (!(reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT)) {
					dev_dbg(plc->dev, "%s : [MAX20357_CH%d] Mailbox RAM cleared (0x%02x)\n",
					 __func__, chip->channel_id, reg_val);
				} else {
					dev_warn(plc->dev, "%s : [MAX20357_CH%d] Mailbox but RAM_is_full not cleared (0x%02x)\n",
					 __func__, chip->channel_id, reg_val);
				}
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_CMD_DONE) || events & BIT(MAX20357_PLC_EVENT_CMD_ERROR)) {
			max20357_check_plc_status(plc, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_BAT_UVLO)) {
			dev_info(plc->dev, "%s : [MAX20357_CH%d] Battery UVLO\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_MOI_DONE)) {
			dev_dbg(plc->dev, "%s : [MAX20357_CH%d] Moisture measurement is completed\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_MOI_DETECTED)) {
			max20357_check_moisture_status(plc, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_MOI_DETECTED_VALID_RESULT)) {
			dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Valid Moisture detection Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_ABORT)) {
			dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Abort resistive Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_OPEN)) {
			dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Open resistive Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_GND)) {
			dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Ground resistive Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_UART_TIMEOUT)) {
			dev_info(plc->dev, "%s : [MAX20357_CH%d] UART Timeout\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_UART_MODE_FAIL)) {
			dev_info(plc->dev, "%s : [MAX20357_CH%d] UART Mode Entry Failed\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_UART_MODE_DONE)) {
			regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
			if (reg_val & MAX20357_UART_CTR1_I2C_URT_ENA_BIT) {
				dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Enter UART mode\n", __func__, chip->channel_id);
			} else {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Not Enter UART mode\n", __func__, chip->channel_id);
			}

			if (reg_val & MAX20357_UART_CTR1_I2C_URT_ABR_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Abort UART mode\n", __func__, chip->channel_id);
			} else {
				dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Not abort UART mode\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_UART_SWITCH_OPEN)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS6, &reg_val);
			if (reg_val & MAX20357_STATUS6_URT_SWC_OPN_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] UART Switch is open due to timeout\n", __func__, chip->channel_id);
			} else if (reg_val & MAX20357_UART_CTR1_I2C_URT_ABR_BIT) {
				dev_dbg(chip->dev, "%s: [MAX20357_CH%d] No UART Switch is open due to timeout\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_DEAD_MASTER)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS2, &reg_val);
			if (reg_val & MAX20357_STATUS2_DEAD_FOUND_STS_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Dead master found\n", __func__, chip->channel_id);
			} else {
				dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Master is Active\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_ITF_READY)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS5, &reg_val);
			if (reg_val & MAX20357_STATUS5_ITF_RDY_STS_BIT) {
				const char *botcode_str, *reset_mode_str;

				u8 botcode = (u8)(reg_val & MAX20357_STATUS5_BOTCODE_LTC_MASK);
				switch (botcode) {
				case 0x0:
					botcode_str = "Nothing";
					break;
				case 0x1:
					botcode_str = "OFF";
					break;
				case 0x2:
					botcode_str = "SEAL";
					break;
				case 0x4:
					botcode_str = "SYSUVLO";
					break;
				case 0x8:
					botcode_str = "Thermal shutdown";
					break;
				default:
					botcode_str = "NA";
					break;
				}

				regmap_read(chip->regmap, MAX20357_REG_BOT_RDB, &reg_val);
				u8 reset_mode = (u8)((reg_val & MAX20357_BOTRDB_RESET_MODE_MASK) >> MAX20357_BOTRDB_RESET_MODE_SHIFT);

				switch (reset_mode) {
				case 0x0:
					reset_mode_str = "No reset";
					break;
				case 0x1:
					reset_mode_str = "Soft reset";
					break;
				case 0x2:
					reset_mode_str = "Hard reset";
					break;
				case 0x3:
					reset_mode_str = "Hard and soft reset";
					break;
				case 0x4:
					reset_mode_str = "UBOOT Reset";
					break;
				default:
					reset_mode_str = "NA";
					break;
				}

				dev_info(plc->dev, "%s : [MAX20357_CH%d] OTP loaded (Bot: %s / Reset: %s)\n",
				 __func__, chip->channel_id,
				 botcode_str, reset_mode_str);

				max2035x_initialize(chip);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] OTP loading not completed\n", __func__, chip->channel_id);
			}
		}
}

/* -------------------------------------------------------------------------- */
/* Virtual IRQ handler (one per event, set by regmap_irq_chip)                */
/* -------------------------------------------------------------------------- */
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

	static const struct regmap_config ram_regmap_cfg = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = MAX2035X_RAM_SIZE - 1,
		.cache_type = REGCACHE_NONE,
	};

	plc->dev = &pdev->dev;
	plc->chip = chip;

	plc->ram_regmap = devm_regmap_init_i2c(chip->ram, &ram_regmap_cfg);
	if (IS_ERR(plc->ram_regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(plc->ram_regmap),
				     "Failed to init RAM regmap\n");

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

	dev_info(&pdev->dev, "%s PLC driver probed\n", chip->info->name);

	return 0;
}

static void max2035x_plc_remove(struct platform_device *pdev)
{
	struct max2035x_plc *plc = platform_get_drvdata(pdev);

	if (!plc)
		return;

	max2035x_debugfs_exit(plc);
	dev_info(&pdev->dev, "%s : PLC driver removed\n", __func__);
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
