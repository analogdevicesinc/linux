// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX20355 PLC (Power Line Communication) driver
 * Shared PLC engine for MAX20355 (Master) and MAX20357 (Slave)
 */

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/math64.h>

#include "max2035x.h"
#include "max2035x_registers.h"
#include "max2035x_plc.h"
#include "max2035x_ram.h"

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

static ssize_t max2035x_plc_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

#define MAX2035X_PLC_ATTR(_name) \
	struct device_attribute dev_attr_##_name = \
		__ATTR(_name, 0444, max2035x_plc_show_attrs, NULL)

static MAX2035X_PLC_ATTR(soc);
static MAX2035X_PLC_ATTR(vcell);
static MAX2035X_PLC_ATTR(tte);
static MAX2035X_PLC_ATTR(avg_vcell);
static MAX2035X_PLC_ATTR(ttf);
static MAX2035X_PLC_ATTR(slave_bat_volt);
static MAX2035X_PLC_ATTR(slave_soc);

static int max20357_set_charger_enable(struct max2035x *chip, bool enable)
{
	int ret;
	u8 val = enable ? MAX20357_CHG_CNTL0_CHG_EN_BIT : 0;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CNTL0,
				 MAX20357_CHG_CNTL0_CHG_EN_BIT, val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to %s Charger (ret: %d)\n", __func__, enable ? "enable" : "disable", ret);
		return ret;
	}

	dev_info(chip->dev, "%s: %s\n", __func__, enable ? "enable" : "disable");

	return 0;
}

static int max20357_set_charger_reset(struct max2035x *chip, bool enable)
{
	int ret;
	u8 val = enable ? MAX20357_SYSTEM_CONFIG0_CHG_RES_EN_BIT : 0;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_CONFIG0,
				 MAX20357_SYSTEM_CONFIG0_CHG_RES_EN_BIT, val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to %s Charger Reset (ret: %d)\n", __func__, enable ? "enable" : "disable", ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (%s)\n", __func__, enable ? "enable" : "disable");

	return 0;
}

static int max20357_set_charger_fast_charge_current_z1(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CUR0,
							MAX20357_CHG_CUR0_CC1IFCHG_MASK,
							(val << MAX20357_CHG_CUR0_CC1IFCHG_SHIFT) & MAX20357_CHG_CUR0_CC1IFCHG_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}

static int max20357_set_charger_fast_charge_current_z2(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CUR1,
							MAX20357_CHG_CUR1_CC2IFCHG_MASK,
							(val << MAX20357_CHG_CUR1_CC2IFCHG_SHIFT) & MAX20357_CHG_CUR1_CC2IFCHG_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}


static int max20357_set_charger_precharge_voltage(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CNTL2,
							MAX20357_CHG_CNTL2_VPCHG_MASK,
							(val << MAX20357_CHG_CNTL2_VPCHG_SHIFT) & MAX20357_CHG_CNTL2_VPCHG_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}

static int max20357_set_charger_step_charge_voltage(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CFG0,
							MAX20357_CHG_CFG0_CHGSTEPRISE_MASK,
							(val << MAX20357_CHG_CFG0_CHGSTEPRISE_SHIFT) & MAX20357_CHG_CFG0_CHGSTEPRISE_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}

static int max20357_set_charger_termination_voltage(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CNTL1,
							MAX20357_CHG_CNTL1_BATREG_MASK,
							(val << MAX20357_CHG_CNTL1_BATREG_SHIFT) & MAX20357_CHG_CNTL1_BATREG_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}

static int max20357_set_charger_topoff_current(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CNTL2,
							MAX20357_CHG_CNTL2_ICHGDONE_MASK,
							(val << MAX20357_CHG_CNTL2_ICHGDONE_SHIFT) & MAX20357_CHG_CNTL2_ICHGDONE_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}


static int max20357_set_charger_safety_timer(struct max2035x *chip, u8 val)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_TMR,
							MAX20357_CHG_TMR_CHGTMR_MASK,
							(val << MAX20357_CHG_TMR_CHGTMR_SHIFT) & MAX20357_CHG_TMR_CHGTMR_MASK);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set to 0x%02x (ret: %d)\n", __func__, val, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (0x%02x)\n", __func__, val);

	return 0;
}

static int max20357_set_charger_cc_track(struct max2035x *chip, bool enable)
{
	int ret;
	u8 val = enable ? MAX20357_CHG_CTR1_CHG_CC_TRK_BIT : 0;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CTR1,
							MAX20357_CHG_CTR1_CHG_CC_TRK_BIT, val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to %s Charger CC Track (ret: %d)\n",	__func__, enable ? "enable" : "disable", ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (%s)\n", __func__, enable ? "enable" : "disable");

	return 0;
}

static int max20357_set_charger_auto_control(struct max2035x *chip, bool auto_stop, bool auto_restart)
{
	int ret;
	u8 mask = MAX20357_CHG_CNTL0_CHG_AUTOSTOP_BIT | MAX20357_CHG_CNTL0_CHG_AUTORESTA_BIT;
	u8 val = 0;

	if (auto_stop)
		val |= MAX20357_CHG_CNTL0_CHG_AUTOSTOP_BIT;
	if (auto_restart)
		val |= MAX20357_CHG_CNTL0_CHG_AUTORESTA_BIT;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CNTL0, mask, val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set Charger Auto Stop/Restart (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (Stop=%s, Restart=%s)\n",
			__func__, auto_stop ? "On" : "Off", auto_restart ? "On" : "Off");

	return 0;
}

static int max20357_set_charger_step_charge(struct max2035x *chip, bool enable, bool room_only)
{
	int ret;
	u8 mask = MAX20357_CHG_CNTL0_CC1_ENABLE_BIT | MAX20357_CHG_CNTL0_CC1_ROOM_ONLY_BIT;
	u8 val = 0;

	if (enable)
		val |= MAX20357_CHG_CNTL0_CC1_ENABLE_BIT;
	if (room_only)
		val |= MAX20357_CHG_CNTL0_CC1_ROOM_ONLY_BIT;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_CHG_CNTL0, mask, val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set Charger Step-Charge (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (Enable=%s, RoomOnly=%s)\n",
			__func__, enable ? "Yes" : "No", room_only ? "Yes" : "No");

	return 0;
}

static int max20357_set_charger_thermistor_monitor(struct max2035x *chip, enum max20357_thmen_mode mode)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_THM_CFG7,
				 MAX20357_THM_CFG7_THMEN_MASK, (u8)mode);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set mode %d (ret: %d)\n", __func__, mode, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Completed (mode 0x%x)\n", __func__, mode);

	return 0;
}

static void max20357_initialize_charger(struct max2035x *chip)
{
	/*
	 * IMPORTANT: These values must be customized based on the battery datasheet.
	 */


	/* Fast Charge Current for Zone 1 and Zone 2 */
	max20357_set_charger_fast_charge_current_z1(chip, 0x43);	/* CC1IFChg: 100mA    - CUSTOMIZE_REQUIRED */
	max20357_set_charger_fast_charge_current_z2(chip, 0x2D);	/* CC2IFChg: 50mA     - CUSTOMIZE_REQUIRED */

	/* Voltage Thresholds for each charging phase */
	max20357_set_charger_precharge_voltage(chip, 0x3);			/* VPChg: 3.0V        - CUSTOMIZE_REQUIRED */
	max20357_set_charger_step_charge_voltage(chip, 0x2);		/* ChgStepRise: 3.85V - CUSTOMIZE_REQUIRED */
	max20357_set_charger_termination_voltage(chip, 0xF);		/* BatReg: 4.2V       - CUSTOMIZE_REQUIRED */

	/* Top-off Current: Defines when to stop charging */
	max20357_set_charger_topoff_current(chip, 0x2);				/* IChgDone: 0.10 X CC2IFChg(50mA) - CUSTOMIZE_REQUIRED */

	/* Safety Timer */
	max20357_set_charger_safety_timer(chip, 0x0);				/* ChgTmr: 75 min     - CUSTOMIZE_REQUIRED */

	/* Enable CC Track to maintain clean PLC line */
	max20357_set_charger_cc_track(chip, true);

	/* Enable JEITA monitoring when PLC is present */
	max20357_set_charger_thermistor_monitor(chip, MAX20357_THM_PLC_PRESENT);

	/* Enable Step-Charging (CC1) and Auto Stop/Restart */
	max20357_set_charger_step_charge(chip, true, false);
	max20357_set_charger_auto_control(chip, true, true);

	dev_info(chip->dev, "%s: completed\n", __func__);
}

static int max2035x_initialize(struct max2035x *chip)
{
	unsigned int val;
	int ret;

	if (chip->type == MAX20355) {
		/* Unmask Interrupt */
		ret = regmap_write(chip->regmap, MAX20355_REG_INTMASK0, 0xFF);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20355_REG_INTMASK0 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20355_REG_INTMASK1, 0xFF);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20355_REG_INTMASK1 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20355_REG_INTMASK2, 0x0F);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20355_REG_INTMASK2 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20355_REG_INTMASK3, 0x3F);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20355_REG_INTMASK3 (ret: %d)\n", __func__, ret);
			return ret;
		}
	} else if (chip->type == MAX20357) {
		max20357_initialize_charger(chip);

		ret = regmap_write(chip->regmap, MAX20357_REG_INTMASK0, 0x59);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20357_REG_INTMASK0 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20357_REG_INTMASK1, 0x03);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20357_REG_INTMASK1 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20357_REG_INTMASK2, 0xE7);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20357_REG_INTMASK2 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20357_REG_INTMASK3, 0x3F);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20357_REG_INTMASK3 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_write(chip->regmap, MAX20357_REG_INTMASK4, 0xF8);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to write MAX20357_REG_INTMASK4 (ret: %d)\n", __func__, ret);
			return ret;
		}

		ret = regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG5, &val);
		if (ret) {
			dev_err(chip->dev, "Failed to read MAX20357_REG_PLC_CONFIG5 (ret: %d)\n", ret);
			return ret;
		}

		if (val & MAX20357_PLC_CFG5_NO_UART_MDE_BIT) {
			ret = regmap_update_bits(chip->regmap, MAX20357_REG_PLC_CONFIG5, MAX20357_PLC_CFG5_NO_UART_MDE_BIT, 0);
			if (ret) {
				dev_err(chip->dev, "%s : Failed to write MAX20357_REG_PLC_CONFIG5 (ret: %d)\n", __func__, ret);
				return ret;
			}
		}
	}

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

	dev_info(plc->dev, "%s : PLC_%d Moisture %s (0x%02x)\n", __func__, target_slave, moi_status ? "Detected" : "Not Detected", reg_val);

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

	dev_info(plc->dev, "%s : [MAX20357_CH%d] Moisture %s (0x%02x)\n", __func__, slave_num, moi_status ? "Detected" : "Not Detected", reg_val);

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
		dev_info(plc->dev, "%s : [MAX20355] CMD_DONE : NO_PLC_ERROR (PLC_%d)\n", __func__, target_slave);
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
		dev_info(plc->dev, "%s : [MAX20357_CH%d] CMD_DONE : NO_PLC_ERROR\n", __func__, slave_num);
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

static bool max2035x_check_cmd_ongoing(struct max2035x *chip, int target_slave)
{
	unsigned int reg_arg, reg_val;
	u8 run_bit;
	int ret;
	bool is_master = (chip->type == MAX20355);

	if (is_master) {
		if (target_slave < 1 || target_slave > 2) {
			dev_err(chip->dev, "%s : Cannot check, Invalid target slave (%d)\n", __func__, target_slave);
			return false;
		}
		reg_arg = (target_slave == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
		run_bit = MAX20355_PLC_CMD_RUN_TRG_BIT;
	} else {
		reg_arg = MAX20357_REG_PLC_CMD;
		run_bit = MAX20357_PLC_CMD_RUN_TRG_BIT;
	}

	ret = regmap_read(chip->regmap, reg_arg, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to read REG_PLC_CMD\n", __func__);
		return false;
	}

	if (reg_val & run_bit) {
		dev_info(chip->dev, "%s : Command running (0x1)\n", __func__);
		return false;
	}

	dev_info(chip->dev, "%s : No command in progress (0x0)\n", __func__);

	return true;
}

static bool max20355_is_valid_sys_req(u8 arg)
{
	switch (arg) {
	case MAX2035X_SYS_REQ_SEAL:
	case MAX2035X_SYS_REQ_SRST:
	case MAX2035X_SYS_REQ_HRST:
	case MAX2035X_SYS_REQ_FGRST:
	case MAX2035X_SYS_REQ_FIFO:
	case MAX2035X_SYS_REQ_FREE:
	case MAX2035X_SYS_REQ_IDLE:
	case MAX2035X_SYS_REQ_UB00:
	case MAX2035X_SYS_REQ_UB01:
	case MAX2035X_SYS_REQ_UB10:
	case MAX2035X_SYS_REQ_UB11:
		return true;
	default:
		return false;
	}
}

static bool max20357_is_valid_sys_req(u8 arg)
{
	switch (arg) {
	case MAX2035X_SYS_REQ_SRST:
	case MAX2035X_SYS_REQ_FGRST:
	case MAX2035X_SYS_REQ_FIFO:
	case MAX2035X_SYS_REQ_FREE:
	case MAX2035X_SYS_REQ_IDLE:
		return true;
	default:
		return false;
	}
}

static int max2035x_send_syst_req(struct max2035x *chip, int target_slave, u8 arg)
{
	unsigned int reg_arg, reg_cmd;
	int ret;
	bool is_master = (chip->type == MAX20355);

	if (is_master) {
		if (target_slave < 1 || target_slave > 2) {
			dev_err(chip->dev, "%s : Cannot send SYS_REQ, Invalid target slave (%d)\n", __func__, target_slave);
			return -EINVAL;
		}

		if (!max20355_is_valid_sys_req(arg)) {
			dev_err(chip->dev, "%s : Cannot send SYS_REQ, Invalid arg (0x%02x)\n", __func__, arg);
			return -EINVAL;
		}

		reg_arg = (target_slave == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target_slave == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		if (!max20357_is_valid_sys_req(arg)) {
			dev_err(chip->dev, "%s : Cannot send SYS_REQ, Invalid arg (0x%02x)\n", __func__, arg);
			return -EINVAL;
		}

		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
		target_slave = 0;
	}

	ret = regmap_write(chip->regmap, reg_arg, arg);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to write REG_PLC_ARG (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* Trigger: Set Bit 7 (0x80) to 1, Command Code: 0x00 (SYS_REQ) */
	ret = regmap_write(chip->regmap, reg_cmd, 0x80 | 0x00);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to write REG_PLC_CMD (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s : SYS_REQ successfully sent (arg: 0x%02x)\n",  __func__, arg);

	return 0;
}

/* --- Local request to enter seal mode via I2C (Self) --- */
static int max20357_request_seal_i2c(struct max2035x *chip)
{
	int ret;

	if (chip->type == MAX20355) {
		dev_warn(chip->dev, "%s : Cannot request Seal mode, Master not supported\n", __func__);
		return -EINVAL;
	}

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_REG0, MAX20357_SYSTEM_SEAL_I2C_CMD_BIT, MAX20357_SYSTEM_SEAL_I2C_CMD_BIT);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to update MAX20357_REG_SYSTEM_REG0 (ret: %d)\n", __func__, ret);
		return ret;
    }

	dev_info(chip->dev, "%s : Successfully request Seal Mode (CH%d)\n", __func__, chip->channel_id);

	return 0;
}

/* Remote request to enter seal mode via PLC (Counterpart) */
static int max20357_request_seal_plc(struct max2035x *chip, int target_slave)
{
	unsigned int reg_val;
	u8 ena_bit;
	int ret;

	if (chip->type == MAX20357) {
		dev_warn(chip->dev, "%s : Cannot request Seal mode, Slave not supported\n", __func__);
		return -EINVAL;
	}

	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : Cannot request Seal mode, PLC line busy or error\n", __func__);
		return -EBUSY;
	}

	dev_info(chip->dev, "%s : Requesting Seal mode for Slave %d\n", __func__, target_slave);

	ret = max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_SEAL);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to send SYS_REQ_SEAL (ret: %d)\n", __func__, ret);
		return ret;
	}

	ena_bit = (target_slave == 1) ? MAX20355_PLC_CFG2_PL1_CHN_ENA_BIT : MAX20355_PLC_CFG2_PL2_CHN_ENA_BIT;

	ret = regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG2, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to read PLC_CONFIG (ret: %d)\n", __func__, ret);
		return ret;
	}

	reg_val &= ~ena_bit;
	ret = regmap_write(chip->regmap, MAX20355_REG_PLC_CONFIG2, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to write CHN_ENA for Slave %d\n", __func__, target_slave);
		return ret;
	}

	dev_info(chip->dev, "%s : Successfully request Seal mode for Slave %d\n", __func__, target_slave);

	return 0;
}

static int max20355_soft_reset_i2c(struct max2035x *chip)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20355_REG_SYSTEM_REG0, MAX20355_SYSTEM_SOFT_RESET_BIT, MAX20355_SYSTEM_SOFT_RESET_BIT);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to update MAX20355_REG_SYSTEM_REG0 (ret: %d)\n", __func__, ret);
		return ret;
    }

	dev_info(chip->dev, "%s : Successfully sent Soft Reset\n", __func__);

	return 0;
}

static int max20357_soft_reset_i2c(struct max2035x *chip)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_REG0, MAX20357_SYSTEM_SOFT_RESET_BIT, MAX20357_SYSTEM_SOFT_RESET_BIT);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to update MAX20357_REG_SYSTEM_REG0 (ret: %d)\n", __func__, ret);
		return ret;
    }

	dev_info(chip->dev, "%s : Successfully sent Soft Reset (CH%d) \n", __func__, chip->channel_id);

	return 0;
}

static int max2035x_soft_reset_plc(struct max2035x *chip, int target_slave)
{
	bool is_master = (chip->type == MAX20355);
	const char *chip_name = is_master ? "MAX20355" : "MAX20357";
	const char *target_name;

	if (!is_master) {
		target_slave = 0;
		target_name = "Master";
	} else {
		target_name = (target_slave == 1) ? "Slave_1" : "Slave_2";
	}


	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : [%s] Cannot request Soft reset, PLC line busy or error\n", __func__, chip_name);
		return -EBUSY;
	}

	dev_info(chip->dev, "%s : [%s] Requesting Soft reset for %s\n", __func__, chip_name, target_name);

	return max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_SRST);
}

static int max2035x_soft_reset(struct max2035x *chip, bool use_plc, int target_slave)
{
	int ret;
	bool is_master = (chip->type == MAX20355);

	if (use_plc) {
		/* --- Remote Soft Reset via PLC (Counterpart) --- */
		ret = max2035x_soft_reset_plc(chip, target_slave);
		dev_info(chip->dev, "%s : Remote Soft Reset via PLC \n", __func__);
	} else {
		/* --- Local Soft Reset via I2C (Self) --- */
		if (is_master) {
			ret = max20355_soft_reset_i2c(chip);
			dev_info(chip->dev, "%s : [MAX20355] Local Soft Reset via I2C\n", __func__);
		} else {
			ret = max20357_soft_reset_i2c(chip);
			dev_info(chip->dev, "%s : [MAX20357] Local Soft Reset via I2C (CH%d) \n", __func__, chip->channel_id);
		}
	}

	return ret;
}

/* --- Local Hard Reset via I2C (Self) --- */
static int max20357_hard_reset_i2c(struct max2035x *chip)
{
	int ret;

	if (chip->type == MAX20355) {
		dev_warn(chip->dev, "%s : Cannot request Hard reset, Master not supported\n", __func__);
		return -EINVAL;
	}

	ret = regmap_update_bits(chip->regmap, MAX20357_REG_SYSTEM_REG0, MAX20357_SYSTEM_HARD_RESET_BIT, MAX20357_SYSTEM_HARD_RESET_BIT);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to update MAX20357_REG_SYSTEM_REG0 (ret: %d)\n", __func__, ret);
		return ret;
    }

	dev_info(chip->dev, "%s : Successfully request Hard reset (CH%d)\n", __func__, chip->channel_id);

	return 0;
}

/* Remote Hard Reset via PLC (Counterpart) */
static int max20357_hard_reset_plc(struct max2035x *chip, int target_slave)
{
	int ret;

	if (chip->type == MAX20357) {
		dev_warn(chip->dev, "%s : Cannot request Hard reset, Slave not supported\n", __func__);
		return -EINVAL;
	}

	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : Cannot request Hard reset, PLC line busy or error\n", __func__);
		return -EBUSY;
	}

	dev_info(chip->dev, "%s : Requesting Hard reset for Slave %d\n", __func__, target_slave);

	ret = max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_HRST);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to send SYS_REQ_HRST (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s : Successfully request Hard reset for Slave %d\n", __func__, target_slave);

	return 0;
}

static int max2035x_fuelgauge_reset(struct max2035x *chip, int target_slave)
{
	bool is_master = (chip->type == MAX20355);
	const char *chip_name = is_master ? "MAX20355" : "MAX20357";
	const char *target_name;

	if (!is_master) {
		target_slave = 0;
		target_name = "Master";
	} else {
		target_name = (target_slave == 1) ? "Slave_1" : "Slave_2";
	}

	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : [%s] Cannot request FG reset, PLC line busy or error\n", __func__, chip_name);
		return -EBUSY;
	}

	dev_info(chip->dev, "%s : [%s] Requesting FG reset for %s\n", __func__, chip_name, target_name);

	return max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_FGRST);
}

static int max2035x_request_fifo(struct max2035x *chip, int target_slave)
{
	return max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_FIFO);
}

static int max2035x_request_free(struct max2035x *chip, int target_slave)
{
	return max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_FREE);
}

static int max2035x_request_idle_mode(struct max2035x *chip, int target_slave)
{
	struct max2035x_plc *plc = chip->plc_data;
	bool is_master = (chip->type == MAX20355);
	const char *chip_name = is_master ? "MAX20355" : "MAX20357";
	const char *target_name;
	int ret;

	if (is_master) {
		if (target_slave < 1 || target_slave > 2) {
			return -EINVAL;
		}

		if (plc->bulk_transfer_active) {
			dev_warn(chip->dev, "%s : [%s] PLC_%d is busy (bulk transfer active)\n",
				 __func__, chip_name, target_slave);
			return -EAGAIN;
		}

		target_name = (target_slave == 1) ? "Slave_1" : "Slave_2";
	} else {
		target_slave = 0; // Slave always targets Master (0)
		target_name = "Master";
		if (plc->bulk_transfer_active) {
			dev_warn(chip->dev, "%s : [%s] PLC is busy (bulk transfer active)\n",
				 __func__, chip_name);
			return -EAGAIN;
		}
	}

	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : [%s] Cannot request IDLE mode, PLC line busy or error\n", __func__, chip_name);
		return -EBUSY;
	}

	ret = max2035x_send_syst_req(chip, target_slave, MAX2035X_SYS_REQ_IDLE);
	if (ret) {
		dev_err(chip->dev, "%s: [%s] Failed to send SYS_REQ for %s (%d)\n", __func__, chip_name, target_name, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: [%s] Requesting IDLE SYST_REQ to %s\n", __func__, chip_name, target_name);

	return ret;
}

static int max2035x_resume_idle_mode(struct max2035x *chip, int target_slave)
{
	unsigned int reg_cfg, reg_val;
	u8 cfg_bit;
	bool is_master = (chip->type == MAX20355);
	const char *chip_name = is_master ? "MAX20355" : "MAX20357";
	const char *target_name;
	int ret;

	if (is_master) {
		reg_cfg = MAX20355_REG_PLC_CONFIG2;
		cfg_bit = (target_slave == 1) ? MAX20355_PLC_CFG2_PL1_RES_REQ_BIT : MAX20355_PLC_CFG2_PL2_RES_REQ_BIT;

		target_name = (target_slave == 1) ? "Slave_1" : "Slave_2";
	} else {
		reg_cfg = MAX20357_REG_PLC_CONFIG4;
		cfg_bit = MAX20357_PLC_CFG4_PLC_RES_REQ_BIT;

		target_slave = 0;
		target_name = "Master";
	}

	ret = regmap_read(chip->regmap, reg_cfg, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s : [%s] Failed to read PLC_CONFIG (ret: %d)\n", __func__, chip_name, ret);
		return ret;
	}

	reg_val |= cfg_bit;
	ret = regmap_write(chip->regmap, reg_cfg, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s : [%s] Failed to write RES_REQ for %s\n", __func__, chip_name, target_name);
		return ret;
	}

	dev_info(chip->dev, "%s: [%s] Requesting resume IDLE to %s\n", __func__, chip_name, target_name);

	return 0;
}

static int max2035x_send_dout_req(struct max2035x *chip, int target_slave, u8 num_bytes)
{
	u8 len_val;
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (num_bytes < 1 || num_bytes > 128) {
		dev_err(chip->dev, "%s : Cannot send DOUT_REQ, Invalid byte count: %d (Allow 1~128)\n", __func__, num_bytes);
		return -EINVAL;
	}

	len_val = num_bytes - 1;

	if (chip->type == MAX20355) {
		if (target_slave < 1 || target_slave > 2) {
			dev_err(chip->dev, "%s : Cannot send DOUT_REQ, Invalid target slave %d\n", __func__, target_slave);
			return -EINVAL;
		}

		reg_arg = (target_slave == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target_slave == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}

	/* * Write data and trigger the command
	 * len_val : 0x00=1byte ~ 0x7F=128bytes
	 */
	ret = regmap_write(chip->regmap, reg_arg, len_val);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to write %s_REG_PLC_ARG%d (ret: %d)\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", target_slave, ret);
		return ret;
	}

	/* Trigger: Set Bit 7 (0x80) to 1, Command Code: 0x05 (DOUT_REQ) */
	ret = regmap_write(chip->regmap, reg_cmd, 0x80 | 0x05);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to write %s_REG_PLC_CMD%d (ret: %d)\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", target_slave, ret);
		return ret;
	}

	dev_info(chip->dev, "%s : DOUT_REQ successfully sent to PLC%d\n", __func__, target_slave);

	return 0;
}

static bool max2035x_is_ram_clear(struct max2035x *chip)
{
	unsigned int reg_cfg, reg_val;
	u8 ram_full_bit;
	int ret;

	if (chip->type == MAX20355) {
		reg_cfg = MAX20355_REG_PLC_CONFIG5;
		ram_full_bit = MAX20355_PLC_CFG5_RAM_IS_FULL_BIT;
	} else {
		reg_cfg = MAX20357_REG_PLC_CONFIG4;
		ram_full_bit = MAX20357_PLC_CFG4_RAM_IS_FULL_BIT;
	}

	ret = regmap_read(chip->regmap, reg_cfg, &reg_val);
    if (ret) {
		dev_err(chip->dev, "%s : Failed to read PLC_CONFIG (ret: %d)\n", __func__, ret);
		return false;
	}

	if (reg_val & ram_full_bit) {
		dev_warn(chip->dev, "%s : RAM is full (reg 0x%02x: 0x%02x), clearing flag...\n", __func__, reg_cfg, reg_val);

		reg_val |= ram_full_bit;
		ret = regmap_write(chip->regmap, MAX20355_REG_PLC_CONFIG5, reg_val);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to clear RAM_is_full\n", __func__);
			return false;
		}

		ret = regmap_read(chip->regmap, reg_cfg, &reg_val);
		if (ret) {
			dev_err(chip->dev, "%s : Failed to re-read PLC_CONFIG\n", __func__);
			return false;
		}

		if (!(reg_val & ram_full_bit)) {
			dev_info(chip->dev, "%s : RAM_is_full successfully cleared\n", __func__);
			return true;
		} else {
			dev_err(chip->dev, "%s : RAM_is_full still 1\n", __func__);
			return false;
		}
	}

	return true;
}

static int max2035x_transfer_mailbox_data(struct max2035x *chip, int target_slave, u8 *buf, size_t len)
{
	int ret;

	if (chip->type == MAX20357)
		target_slave = 0;

	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : Cannot transfer mailbox, PLC line busy or error\n", __func__);
		return -EBUSY;
	}

	if (!max2035x_is_ram_clear(chip)) {
		dev_warn(chip->dev, "%s : Cannot transfer mailbox, RAM is full\n", __func__);
		return -EBUSY;
	}

	ret = max2035x_write_ram_data(chip, buf, len);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write RAM data (%d)\n", __func__, ret);
		return ret;
	}

	ret = max2035x_send_dout_req(chip, target_slave, (u8)len);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to send DOUT_REQ (%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int max2035x_check_fifo_master(struct max2035x *chip, int target_slave)
{
	unsigned int reg_fifo, reg_val;
	u32 bit_fifo;
	int is_master, ret;

	if (chip->type == MAX20355) {
		reg_fifo = MAX20355_REG_PLC_FIFO;
		bit_fifo = (target_slave == 1) ? MAX20355_PLC_FIFO_PL1_MASTER_BIT : MAX20355_PLC_FIFO_PL2_MASTER_BIT;
	} else {
		reg_fifo = MAX20357_REG_PLC_CONFIG4;
		bit_fifo = MAX20357_PLC_CFG4_FIFO_MASTER_BIT;
	}

	ret = regmap_read(chip->regmap, reg_fifo, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read register 0x%02x\n", __func__, reg_fifo);
		return ret;
	}

	is_master = (reg_val & bit_fifo) ? 1 : 0;

	dev_info(chip->dev, "%s: FIFO master mode (%s) (reg_val: 0x%02x)\n", __func__, is_master ? "True" : "False", reg_val);

	return is_master;
}

static int max2035x_check_fifo_slave(struct max2035x *chip, int target_slave)
{
	unsigned int reg_fifo, reg_val;
	u32 bit_fifo;
	int is_slave, ret;
	struct max2035x *peer_chip;

	if (chip->type == MAX20355) {
		reg_fifo = MAX20357_REG_PLC_CONFIG4;
		bit_fifo = MAX20357_PLC_CFG4_FIFO_SLAVE_BIT;

		peer_chip = max2035x_get_device(MAX20357, target_slave);
	} else {
		reg_fifo = MAX20355_REG_PLC_FIFO;
		bit_fifo = (target_slave == 1) ? MAX20355_PLC_FIFO_PL1_SLAVE_BIT : MAX20355_PLC_FIFO_PL2_SLAVE_BIT;

		peer_chip = max2035x_get_device(MAX20355, 0);
	}

	ret = regmap_read(peer_chip->regmap, reg_fifo, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read register 0x%02x\n", __func__, reg_fifo);
		return ret;
	}

	is_slave = (reg_val & bit_fifo) ? 1 : 0;

	dev_info(peer_chip->dev, "%s: FIFO slave mode (%s) (reg_val: 0x%02x)\n", __func__, is_slave ? "True" : "False", reg_val);

	return is_slave;
}

static int max2035x_transfer_bulk_data(struct max2035x *chip, int target_slave, u8 *buf, size_t total_len)
{
	struct max2035x_plc *plc = chip->plc_data;
	size_t offset = 0;
	size_t current_chunk;
	int retry_cnt, ret;
	unsigned int reg_fifo;
	u32 bit_fifo;

	if (chip->type == MAX20357)
		target_slave = 0;

	/* ---------------------------------------------------------------
	 * PHASE 0: INITIAL PACKET LOAD
	 * --------------------------------------------------------------- */
	/* 0-0: Check RAM status (RAM_is_full == 0?) */
	if (!max2035x_is_ram_clear(chip)) {
		dev_warn(chip->dev, "%s : Cannot transfer mailbox, RAM is full\n", __func__);
		return -EBUSY;
	}

	/* 0-1: Load first data packet to RAM through I2C */
	current_chunk = min_t(size_t, total_len, 128);
	ret = max2035x_write_ram_data(chip, buf, current_chunk);
	if (ret) {
		dev_err(plc->dev, "%s : Failed write_ram_data at offset %zu (ret: %d)\n", __func__, offset, ret);
		return ret;
	}

	offset += current_chunk;

	/* ---------------------------------------------------------------
	 * PHASE 1: LOCK PLC LINE (FIFO LOCK)
	 * --------------------------------------------------------------- */
	/* 1-0: Check if plc_run_trg (0x39[7] or 0x3B[7]) is 0 */
	for (retry_cnt = 0; retry_cnt < 5; retry_cnt++) {
		if (max2035x_check_cmd_ongoing(chip, target_slave))
			break;

		dev_warn(chip->dev, "%s : PLC line busy, retrying... (%d/5)\n", __func__, retry_cnt + 1);
		reinit_completion(&plc->cmd_done);
		wait_for_completion_timeout(&plc->cmd_done, msecs_to_jiffies(100));

		if (retry_cnt == 4) {
			dev_err(chip->dev, "%s : PLC remains busy, aborting.\n", __func__);
			return -EBUSY;
		}
	}

	plc->bulk_transfer_active = true;
	reinit_completion(&plc->cmd_done);

	/* 1-1/1-2: Trigger SYST_REQ (0x04) to lock PLC line */
	ret = max2035x_request_fifo(chip, target_slave);
	if (ret) {
		dev_err(plc->dev, "%s : Failed to send FIFO request (ret: %d)\n", __func__, ret);
		goto out;
	}

	/* ---------------------------------------------------------------
	 * PHASE 2: DATA TRANSFER LOOP (Dual RAM Mechanism)
	 * --------------------------------------------------------------- */
	/* 2-0: Wait for nINT (plc_cmd_dne) & Check plc_status */
	if (!wait_for_completion_timeout(&plc->cmd_done, msecs_to_jiffies(500))) {
		dev_err(plc->dev, "%s : Timeout waiting for FIFO LOCK\n", __func__);
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = (chip->type == MAX20355) ? max20355_check_plc_status(plc, target_slave) : max20357_check_plc_status(plc, chip->channel_id);
	if (ret != 0x00) {
		dev_err(chip->dev, "%s: FIFO LOCK failed, status: 0x%02x\n", __func__, ret);
		ret = -EIO;
		goto out;
	}

	/* 2-1: Configure Transmitting Device (Read/Write 0x3D) */
	if (chip->type == MAX20355) {
		reg_fifo = MAX20355_REG_PLC_FIFO;
		bit_fifo = (target_slave == 1) ? MAX20355_PLC_FIFO_PL1_MASTER_BIT : MAX20355_PLC_FIFO_PL2_MASTER_BIT;
	} else {
		reg_fifo = MAX20357_REG_PLC_CONFIG4;
		bit_fifo = MAX20357_PLC_CFG4_FIFO_MASTER_BIT;
	}

	if (max2035x_check_fifo_master(chip, target_slave) != 1) {
		dev_err(chip->dev, "%s: Failed to set Master bit (Reg: 0x%02x)\n", __func__, reg_fifo);
		ret = -EIO;
		goto out;
	}

	while (offset < total_len || current_chunk > 0) {
		/* 2-2 & 2-3: Set length & Trigger DOUT_REQ (0x85) - Swap occurs here */
		reinit_completion(&plc->cmd_done);
		ret = max2035x_send_dout_req(chip, target_slave, (u8)current_chunk);
		if (ret) {
			dev_err(chip->dev, "%s: DOUT_REQ failed at offset %zu\n", __func__, offset);
			break;
		}

		/* 2-4: Load next packet while PLC is transmitting (Parallel) */
		if (offset < total_len) {
			size_t next_chunk = min_t(size_t, total_len - offset, 128);

			/* Check RAM_is_full immediately after Swap */
			if (!max2035x_is_ram_clear(chip)) {
				dev_err(chip->dev, "%s: RAM not clear for swap at offset %zu\n", __func__, offset);
				ret = -EIO;
				break;
			}

			/* Load the next data with I2C while PLC is shooting (parallel processing) */
			ret = max2035x_write_ram_data(chip, buf + offset, next_chunk);
			if (ret) {
				dev_err(chip->dev, "%s: RAM write failed during transmission (offset %zu)\n", __func__, offset);
				break;
			}

			offset += next_chunk;
			current_chunk = next_chunk;
		} else {
			/* Last packet triggered */
			current_chunk = 0;
		}

		/* 3-0: Wait for nINT (plc_cmd_dne) & Check plc_status */
		if (!wait_for_completion_timeout(&plc->cmd_done, msecs_to_jiffies(200))) {
			dev_err(chip->dev, "%s: Timeout waiting for DOUT_REQ ACK (offset: %zu)\n", __func__, offset);
			ret = -ETIMEDOUT;
			break;
		}

		ret = (chip->type == MAX20355) ? max20355_check_plc_status(plc, target_slave) : max20357_check_plc_status(plc, chip->channel_id);
		if (ret != 0x00) {
			dev_err(chip->dev, "%s: PLC status error 0x%02x at offset %zu\n", __func__, ret, offset);
			ret = -EIO;
			break;
		}

		if (current_chunk == 0)
			break;
	}

out:
	/* ---------------------------------------------------------------
	 * PHASE 3: FREE PLC LINE
	 * --------------------------------------------------------------- */
	/* 3-1 & 3-2: Write 0x80 to PLC_CMD (SYST_REQ: FREE FIFO) */
	reinit_completion(&plc->cmd_done);
	max2035x_request_free(chip, target_slave);

	/* 4-0: Wait for nINT (plc_cmd_dne) & Check plc_status */
	if (!wait_for_completion_timeout(&plc->cmd_done, msecs_to_jiffies(300))) {
		dev_warn(chip->dev, "%s: Timeout waiting for FREE_SYST_REQ\n", __func__);
	}

	ret = (chip->type == MAX20355) ? max20355_check_plc_status(plc, target_slave) : max20357_check_plc_status(plc, chip->channel_id);
	if (ret != 0x00) {
		dev_warn(chip->dev, "%s: PLC FREE status check failed (0x%02x)\n", __func__, ret);
	}

	if (max2035x_check_fifo_master(chip, target_slave) != 0) {
		dev_warn(chip->dev, "%s: Master bit not cleared after FREE_SYST_REQ\n", __func__);
	} else {
		dev_info(chip->dev, "%s: Master bit cleared successfully\n", __func__);
	}

	plc->bulk_transfer_active = false;

	dev_info(chip->dev, "%s: Bulk transfer finished. Sent: %zu/%zu bytes (ret: %d)\n",
		 __func__, offset, total_len, (offset == total_len) ? 0 : ret);

	return (offset == total_len) ? 0 : ret;
}

/* --- Local GPIO setting via I2C (Self) --- */
static int max2035x_set_gpio_i2c(struct max2035x *chip, int gpio_num, bool is_input, bool is_high)
{
	bool is_master = (chip->type == MAX20355);
	const char *chip_name = is_master ? "MAX20355" : "MAX20357";
	uint8_t reg_addr, reg_gpio;
	u8 bit_ctr, bit_enres, bit_enpup, bit_dout;
	uint8_t reg_val = 0;
	int ret;

	if (gpio_num < 1 || gpio_num > 4) {
		dev_err(chip->dev, "%s: [%s] Invalid GPIO number: %d (1-4 required)\n", __func__, chip_name, gpio_num);
		return -EINVAL;
	}

	if (is_master) {
		reg_gpio = MAX20355_REG_GPIO1;

		bit_ctr = MAX20355_GPIO_PLCCTR_BIT;
		bit_enres = MAX20355_GPIO_ENRES_BIT;
		bit_enpup = MAX20355_GPIO_ENPUP_BIT;
		bit_dout = MAX20355_GPIO_DOUT_BIT;
	} else {
		reg_gpio = MAX20357_REG_GPIO1;

		bit_ctr = MAX20357_GPIO_PLCCTR_BIT;
		bit_enres = MAX20357_GPIO_ENRES_BIT;
		bit_enpup = MAX20357_GPIO_ENPUP_BIT;
		bit_dout = MAX20357_GPIO_DOUT_BIT;
	}

	reg_addr = reg_gpio + (gpio_num - 1);

	reg_val &= ~bit_ctr;				// CTR = 0 (I2C)

	if (is_input) {
		/* Input Mode Configuration (DOUT (Bit 0) is ignored when configured as Input) */
		reg_val |= bit_enres;			// ENRES = 1 (Input)

		if (is_high)
			reg_val |= bit_enpup;		// ENPUP = 1 (Pull-up)
		else
			reg_val &= ~bit_enpup;		// ENPUP = 0 (Pull-down)

		dev_dbg(chip->dev, "%s: [%s] GPIO%d configuring as INPUT (%s)\n", __func__, chip_name, gpio_num, is_high ? "Pull-up" : "Pull-down");
	} else {
		/* Output Mode Configuration (ENPUP (Bit 1) is ignored when configured as Output) */
		reg_val &= ~bit_enres;			// ENRES = 0 (Output)

		if (is_high)
			reg_val &= ~bit_dout;		// DOUT = 0 (High)
		else
			reg_val |= bit_dout;		// DOUT = 1 (Low)

		dev_dbg(chip->dev, "%s: [%s] GPIO%d configuring as OUTPUT (%s)\n", __func__, chip_name, gpio_num, is_high ? "High" : "Low");
	}

	ret = regmap_write(chip->regmap, reg_addr, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: [%s] Failed to write REG_GPIO%d (0x%02x)\n", __func__, chip_name, gpio_num, reg_val);
		return ret;
	}

	dev_info(chip->dev, "%s: [%s] GPIO%d set to %s (%s) (RegVal: 0x%02x)\n", __func__, chip_name, gpio_num,
			is_input ? "Input" : "Output",
			is_input ? (is_high ? "Pull-up" : "Pull-down") : (is_high ? "High" : "Low"),
			reg_val);

	return ret;
}

static int max2035x_set_gpio(struct max2035x *chip, int target_slave, u8 arg)
{
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (chip->type == MAX20355) {
		if (target_slave < 1 || target_slave > 2) {
			dev_err(chip->dev, "%s : Cannot SET_GPIO, Invalid channel %d\n", __func__, target_slave);
			return -EINVAL;
		}

		reg_arg = (target_slave == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target_slave == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}

	ret = regmap_write(chip->regmap, reg_arg, arg);
	if (ret) {
		dev_err(chip->dev, "%s : [%s] Failed to write REG_PLC_ARG (ret: %d)\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", ret);
		return ret;
	}

	/* Trigger: Set Bit 7 (0x80) to 1, Command Code: 0x03 (SET_GPIO) */
	ret = regmap_write(chip->regmap, reg_cmd, 0x80 | 0x03);
	if (ret) {
		dev_err(chip->dev, "%s : [%s] Failed to write REG_PLC_CMD (ret: %d)\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", ret);
		return ret;
	}

	dev_info(chip->dev, "%s : SET_GPIO successfully sent to PLC%d\n", __func__, target_slave);

	return 0;
}

/* Remote GPIO setting via PLC (Counterpart) */
static int max2035x_set_gpio_plc(struct max2035x *chip, int target_slave, int gpio_num, bool is_input, bool is_high)
{
	bool is_master = (chip->type == MAX20355);
	const char *chip_name = is_master ? "MAX20355" : "MAX20357";
	uint8_t reg_arg, reg_addr, reg_gpio;
	u8 bit_ctr;
	u8 arg = 0;
	int ret;
	unsigned int val;

	if (gpio_num < 1 || gpio_num > 4) {
		dev_err(chip->dev, "%s: [%s] Invalid GPIO number: %d (1-4 required)\n", __func__, chip_name, gpio_num);
		return -EINVAL;
	}

	if (!max2035x_check_cmd_ongoing(chip, target_slave)) {
		dev_warn(chip->dev, "%s : [%s] Cannot set gpio, PLC line busy or error\n", __func__, chip_name);
		return -EBUSY;
	}

	if (is_master) {
		reg_arg = (target_slave == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;

		reg_gpio = MAX20355_REG_GPIO1;
		bit_ctr = MAX20355_GPIO_PLCCTR_BIT;
	} else {
		reg_arg = MAX20357_REG_PLC_ARG;

		reg_gpio = MAX20357_REG_GPIO1;
		bit_ctr = MAX20357_GPIO_PLCCTR_BIT;
	}

	reg_addr = reg_gpio + (gpio_num - 1);
	ret = regmap_update_bits(chip->regmap, reg_addr, bit_ctr, bit_ctr);	// CTR = 1 (PLC)
	if (ret) {
		dev_err(chip->dev, "%s: [%s] Failed to updae REG_GPIO%d for PLCCTR (ret:%d)\n",
			__func__, chip_name, gpio_num, ret);
		return ret;
	}

	ret = regmap_read(chip->regmap, reg_arg, &val);
	if (ret) {
		dev_err(chip->dev, "%s: [%s] Failed to read REG_PLC_ARG (ret:%d)\n", __func__, chip_name, ret);
		return ret;
	}
	arg = (u8)val;

	if (is_input) {
		arg |= BIT(gpio_num + 3);		// Set EnRes bit (7 to 4) to 1

		/* When Input, Dout bit is usually ignored, but keeping it 0 is safe */
		arg &= ~BIT(gpio_num - 1);
	} else {
		arg &= ~BIT(gpio_num + 3);		// Clear EnRes bit (7 to 4) to 0

		if (is_high)
			arg &= ~BIT(gpio_num - 1);	// DOUT = 0 (High)
		else
			arg |= BIT(gpio_num - 1);	// DOUT = 1 (Low)
	}

	dev_info(chip->dev, "%s: [%s] PLC%d GPIO%d -> %s %s (Arg: 0x%02x)\n",
			 __func__, chip_name, target_slave, gpio_num,
			 is_input ? "INPUT" : "OUTPUT", is_high ? "HIGH" : "LOW", arg);

	return max2035x_set_gpio(chip, target_slave, arg);
}

static int max2035x_send_uart_req(struct max2035x *chip, int target_slave, u8 arg)
{
	unsigned int reg_arg, reg_cmd;
	int ret;

	if (chip->type == MAX20355) {
		if (target_slave < 1 || target_slave > 2) {
			dev_err(chip->dev, "%s : Cannot send UART_REQ, Invalid channel %d\n", __func__, target_slave);
			return -EINVAL;
		}

		if (arg > MAX2035X_UART_LOCAL_LOOPBACK) {
			dev_err(chip->dev, "%s : Cannot send UART_REQ, Invalid arg\n", __func__);
			return -EINVAL;
		}

		reg_arg = (target_slave == 1) ? MAX20355_REG_PLC_ARG1 : MAX20355_REG_PLC_ARG2;
		reg_cmd = (target_slave == 1) ? MAX20355_REG_PLC_CMD1 : MAX20355_REG_PLC_CMD2;
	} else {
		if (!(arg == MAX2035X_UART_MANUAL || arg == MAX2035X_UART_MASTER_RX ||
			arg == MAX2035X_UART_MASTER_TX || arg == MAX2035X_UART_LOCAL_LOOPBACK ||
			arg == MAX20357_UART_PLC2_MASTER_RX || arg == MAX20357_UART_PLC2_MASTER_TX ||
			arg == MAX20357_UART_PLC2_LOOPBACK)) {
			dev_err(chip->dev, "%s : Cannot send UART_REQ, Invalid arg\n", __func__);
			return -EINVAL;
		}

		reg_arg = MAX20357_REG_PLC_ARG;
		reg_cmd = MAX20357_REG_PLC_CMD;
	}

	/* * Write data and trigger the command
	 * cmd options :
	 * 0x0: All switches are OFF (Manual configuration of UART direction through I2C command)
	 * 0x1: RX switches are ON (Master receiving, slave transimitting)
	 * 0x2: TX switches are ON (Master transmitting, slave receiving)
	 * 0x3: TX and RX switches are ON (Local loop back)
	 */
	ret = regmap_write(chip->regmap, reg_arg, arg);
	if (ret) {
		dev_err(chip->dev, "%s : [%s] Failed to write REG_PLC_ARG (ret: %d)\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", ret);
		return ret;
	}

	/* Trigger: Set Bit 7 (0x80) to 1, Command Code: 0x06 (UART_REQ) */
	ret = regmap_write(chip->regmap, reg_cmd, 0x80 | 0x06);
	if (ret) {
		dev_err(chip->dev, "%s : [%s] Failed to write REG_PLC_CMD (ret: %d)\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", ret);
		return ret;
	}

	dev_info(chip->dev, "%s : UART_REQ successfully sent to PLC%d\n", __func__, target_slave);

	return 0;
}

static int max20355_enable_uart_timeout(struct max2035x *chip, int target_slave, bool tmo_enable)
{
    u8 reg = (target_slave == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;
	u8 val = tmo_enable ? MAX20355_UART_CTR_TMO_TMR_ENA_BIT : 0;

	dev_info(chip->dev, "%s: %s UART Timeout (CH%d)\n",	__func__, tmo_enable ? "Enable" : "Disable", target_slave);

	return regmap_update_bits(chip->regmap, reg, MAX20355_UART_CTR_TMO_TMR_ENA_BIT, val);
}

static int max20357_enable_uart_timeout(struct max2035x *chip, bool tmo_enable)
{
	u8 val = tmo_enable ? MAX20357_UART_CTR1_TMO_TMR_ENA_BIT : 0;

	dev_info(chip->dev, "%s: %s UART Timeout\n", __func__, tmo_enable ? "Enable" : "Disable");

	return regmap_update_bits(chip->regmap, MAX20357_REG_UART_CTR1, MAX20357_UART_CTR1_TMO_TMR_ENA_BIT, val);
}

static int max20355_set_uart_control_mode(struct max2035x *chip, int target_slave, bool use_i2c)
{
    u8 reg = (target_slave == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;
	unsigned int reg_val;
	unsigned int mask;
	int ret;

	ret = regmap_read(chip->regmap, reg, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read MAX20355_REG_UART_CTR%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	mask = MAX20355_UART_CTR_I2C_URT_MOD_BIT | MAX20355_UART_CTR_I2C_URT_SWC_BIT;

	if (use_i2c) {
		/* Manual I2C control mode: Set corresponding bits to 1 */
		reg_val |= mask;
	} else {
		/* Automatic PLC control mode: Clear corresponding bits to 0 */
		reg_val &= ~mask;
	}

	ret = regmap_write(chip->regmap, reg, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write MAX20355_REG_UART_CTR%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: [CH_%d] Set to %s mode (0x%02x)\n",
			 __func__, target_slave, use_i2c ? "Manual(I2C)" : "Automatic(PLC)", reg_val);

	return 0;
}

static int max20357_set_uart_control_mode(struct max2035x *chip, bool use_i2c)
{
	unsigned int reg_val;
	unsigned int mask;
	int ret;

	ret = regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Read UART_CTR1 %s mode (0x%02x)\n",
			 __func__, use_i2c ? "Manual(I2C)" : "Automatic(PLC)", reg_val);


	mask = MAX20357_UART_CTR1_I2C_URT_MOD_BIT | MAX20357_UART_CTR1_I2C_URT_SWC_BIT;

	if (use_i2c) {
		/* Manual I2C control mode: Set corresponding bits to 1 */
		reg_val |= mask;
	} else {
		/* Automatic PLC control mode: Clear corresponding bits to 0 */
		reg_val &= ~mask;
	}

	ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Set to %s mode (0x%02x)\n",
			 __func__, use_i2c ? "Manual(I2C)" : "Automatic(PLC)", reg_val);

	return 0;
}

static int max20355_enter_uart_tx(struct max2035x *chip, int target_slave)
{
	int ret;

	ret = max20355_set_uart_control_mode(chip, target_slave, false);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set UART control mode of slave_%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	ret = max20355_enable_uart_timeout(chip, target_slave, true);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to enable UART timeout of slave_%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: MAX20355 Master Request -> UART TX Mode (CH%d)\n", __func__, target_slave);
	return max2035x_send_uart_req(chip, target_slave, MAX2035X_UART_MASTER_TX);
}

static int max20355_enter_uart_rx(struct max2035x *chip, int target_slave)
{
	int ret;

	ret = max20355_set_uart_control_mode(chip, target_slave, false);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set UART control mode of slave_%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	ret = max20355_enable_uart_timeout(chip, target_slave, true);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to enable UART timeout of slave_%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: MAX20355 Master Request -> UART RX Mode (CH%d)\n", __func__, target_slave);
	return max2035x_send_uart_req(chip, target_slave, MAX2035X_UART_MASTER_RX);
}

static int max20357_enter_uart_tx(struct max2035x *chip)
{
	unsigned int reg_val;
	u8 arg;
	int ret;

	ret = max20357_set_uart_control_mode(chip, false);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to set UART control mode (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	ret = max20357_enable_uart_timeout(chip, true);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to enable UART timeout (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	arg = (chip->channel_id == 1) ? MAX2035X_UART_MASTER_TX : MAX20357_UART_PLC2_MASTER_TX;

	ret = max2035x_send_uart_req(chip, chip->channel_id, arg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to send UART_REQ (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: MAX20357 CH_%d Request -> UART TX Mode (arg: 0x%02x)\n", __func__, chip->channel_id, arg);

	/* Disabled PLC FSM */
	ret = regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to read MAX20357_REG_PLC_CONFIG4 (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	reg_val &= ~MAX20357_PLC_CFG4_PLC_FSM_ENA_BIT;

	ret = regmap_write(chip->regmap, MAX20357_REG_PLC_CONFIG4, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to write MAX20357_REG_PLC_CONFIG4 (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: [CH%d] PLC FSM disabled for UART mode (0x%02x)\n", __func__, chip->channel_id, reg_val);

	/* Manually closing TX switch */
	ret = regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	reg_val |= (MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
				MAX20357_UART_CTR1_I2C_URT_ENA_BIT |
				MAX20357_UART_CTR1_I2C_URT_SWC_BIT |
				MAX20357_UART_CTR1_I2C_TX_SWC_BIT);

	reg_val &= ~MAX20357_UART_CTR1_I2C_RX_SWC_BIT;

	ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Manually closing TX switch (0x%02x)\n", __func__, reg_val);
	dev_info(chip->dev, "%s: MAX20357 Slave_%d UART TX Entry Succeeded\n", __func__, chip->channel_id);

	return 0;
}

static int max20357_enter_uart_rx(struct max2035x *chip)
{
	unsigned int reg_val;
	u8 arg;
	int ret;

	ret = max20357_set_uart_control_mode(chip, false);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to set UART control mode (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	ret = max20357_enable_uart_timeout(chip, true);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to enable UART timeout (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	arg = (chip->channel_id == 1) ? MAX2035X_UART_MASTER_RX : MAX20357_UART_PLC2_MASTER_RX;

	ret = max2035x_send_uart_req(chip, chip->channel_id, arg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to send UART_REQ (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: MAX20357 CH_%d Request -> UART RX Mode (arg: 0x%02x)\n", __func__, chip->channel_id, arg);

	/* Manually closing TX switch */
	ret = regmap_read(chip->regmap, MAX20357_REG_UART_CTR1, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	reg_val |= (MAX20357_UART_CTR1_I2C_URT_MOD_BIT |
				MAX20357_UART_CTR1_I2C_URT_ENA_BIT |
				MAX20357_UART_CTR1_I2C_URT_SWC_BIT |
				MAX20357_UART_CTR1_I2C_RX_SWC_BIT);

	reg_val &= ~MAX20357_UART_CTR1_I2C_TX_SWC_BIT;

	ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: Manually closing RX switch (0x%02x)\n", __func__, reg_val);
	dev_info(chip->dev, "%s: MAX20357 Slave_%d UART RX Entry Succeeded\n", __func__, chip->channel_id);

	return 0;
}

static int max20355_enter_uart_auto(struct max2035x *chip, int target_slave, enum max2035x_uart_timeout_value tmo_val)
{
    u8 en_bit = (target_slave == 1) ? MAX20355_UART_CTR_URT_AUTO_EN1_BIT : MAX20355_UART_CTR_URT_AUTO_EN2_BIT;
	u8 val;
	int ret;

	ret = max20355_set_uart_control_mode(chip, target_slave, false);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to set UART control mode of slave_%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	ret = max20355_enable_uart_timeout(chip, target_slave, true);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to enable UART timeout of slave_%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	val = en_bit | (tmo_val & 0x03);

	ret = regmap_write(chip->regmap, MAX20355_REG_UART_CTR0, (unsigned int)val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write MAX20355_REG_UART_CTR0 (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: MAX20355 UART Auto Mode Enabled (Timeout: %u us, Reg: 0x%02x)\n",
			 __func__,
			 (tmo_val == MAX2035X_UART_TMO_8us) ? 8 :
			 (tmo_val == MAX2035X_UART_TMO_16us) ? 16 :
			 (tmo_val == MAX2035X_UART_TMO_32us) ? 32 : 64, val);

	ret = max2035x_send_uart_req(chip, target_slave, MAX2035X_UART_MASTER_TX);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to send UART_REQ (ret: %d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int max20357_enter_uart_auto(struct max2035x *chip, enum max2035x_uart_timeout_value tmo_val)
{
	u8 val;
	int ret;

	ret = max20357_set_uart_control_mode(chip, false);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to set UART control mode (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	ret = max20357_enable_uart_timeout(chip, true);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to enable UART timeout (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	val = MAX20357_UART_CTR0_URT_AUTO_EN_BIT | (tmo_val & 0x03);

	ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR0, (unsigned int)val);
	if (ret) {
		dev_err(chip->dev, "%s: [CH%d] Failed to write MAX20357_REG_UART_CTR0 (ret: %d)\n", __func__, chip->channel_id, ret);
		return ret;
	}

	dev_info(chip->dev, "%s: [CH%d] MAX20357 UART Auto Mode Enabled (Timeout: %u us, Reg: 0x%02x)\n",
			 __func__, chip->channel_id,
			 (tmo_val == MAX2035X_UART_TMO_8us) ? 8 :
			 (tmo_val == MAX2035X_UART_TMO_16us) ? 16 :
			 (tmo_val == MAX2035X_UART_TMO_32us) ? 32 : 64, val);

	return 0;
}

static int max20355_exit_uart(struct max2035x *chip, int target_slave)
{
	int ret;
	u8 reg = (target_slave == 1) ? MAX20355_REG_UART_CTR1 : MAX20355_REG_UART_CTR2;

	ret = regmap_write(chip->regmap, reg, 0x84);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write 0x84 to MAX20355_REG_UART_CTR%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	ret = regmap_write(chip->regmap, reg, 0x00);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write 0x00 to MAX20355_REG_UART_CTR%d (ret: %d)\n", __func__, target_slave, ret);
		return ret;
	}

	dev_info(chip->dev, "%s : Successfully exit UART Mode for Slave %d\n", __func__, target_slave);

	return 0;
}

static int max20357_exit_uart(struct max2035x *chip)
{
	int ret;

	ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, 0x28);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write 0x28 to MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_write(chip->regmap, MAX20357_REG_UART_CTR1, 0x00);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to write 0x00 to MAX20357_REG_UART_CTR1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(chip->dev, "%s : Successfully exit UART Mode\n", __func__);

	return 0;
}

static int max2035x_request_off_mode(struct max2035x *chip)
{
	unsigned int reg_sts0;
	u32 bit_off_cmd;
	int ret;

	if (chip->type == MAX20355) {
		reg_sts0 = MAX20355_REG_SYSTEM_REG0;
		bit_off_cmd = MAX20355_SYSTEM_OFF_CMD_INP_BIT;
	} else {
		reg_sts0 = MAX20357_REG_SYSTEM_REG0;
		bit_off_cmd = MAX20357_SYSTEM_OFF_CMD_INP_BIT;
	}

	ret = regmap_update_bits(chip->regmap, reg_sts0, bit_off_cmd, bit_off_cmd);
	if (ret) {
		dev_err(chip->dev, "%s : Failed to update REG_SYSTEM_REG0 (ret: %d)\n", __func__, ret);
		return ret;
    }

	dev_info(chip->dev, "%s : Successfully request OFF Mode (CH%d)\n", __func__, chip->channel_id);

	return 0;
}

static int max2035x_read_soc(struct max2035x *chip)
{
	unsigned int ready_reg, byte1, byte0;
	unsigned int reg_soc_byte1, reg_soc_byte0, reg_ready;
	unsigned int ready_bit;
	u16 soc_raw;
	int ret, soc_percent_x100;

	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}

	/* Select registers based on chip type */
	if (chip->type == MAX20355) {
		reg_soc_byte1 = MAX20355_REG_SOC_BYTE_1;
		reg_soc_byte0 = MAX20355_REG_SOC_BYTE_0;
		reg_ready = MAX20355_REG_READY_REG;
		ready_bit = MAX20355_READY_SOC_RDY_BIT;
	} else {
		reg_soc_byte1 = MAX20357_REG_SOC_BYTE_1;
		reg_soc_byte0 = MAX20357_REG_SOC_BYTE_0;
		reg_ready = MAX20357_REG_READY_REG;
		ready_bit = MAX20357_READY_SOC_RDY_BIT;
	}

	/* Check if data is reliable (READY_REG) */
	ret = regmap_read(chip->regmap, reg_ready, &ready_reg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read READY_REG (ret: %d)\n", __func__, ret);
		return ret;
	}

	if (!(ready_reg & ready_bit)) {
		dev_warn(chip->dev, "%s: SOC data not reliable\n", __func__);
		return -EAGAIN;
	}

	/* Read SOC byte1 and byte0 */
	ret = regmap_read(chip->regmap, reg_soc_byte1, &byte1);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read SOC_BYTE_1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_read(chip->regmap, reg_soc_byte0, &byte0);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read SOC_BYTE_0 (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* Concatenate byte1 and byte0 */
	soc_raw = (byte1 << 8) | byte0;

	/* Convert to percentage * 100 (LSB = 0.00390625% = 1/256%) */
	/* soc_raw * 100 / 256 = soc_raw * 100 >> 8 */
	soc_percent_x100 = (soc_raw * 100) >> 8;

	dev_info(chip->dev, "%s: SOC: %d.%02d%% (0x%04x)\n",
		__func__, soc_percent_x100 / 100, soc_percent_x100 % 100, soc_raw);

	return soc_percent_x100;
}

static int max2035x_read_vcell(struct max2035x *chip)
{
	unsigned int ready_reg, byte1, byte0;
	unsigned int reg_vcell_byte1, reg_vcell_byte0, reg_ready;
	unsigned int ready_bit;
	u16 vcell_raw;
	int ret, vcell_uv;

	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}

	/* Select registers based on chip type */
	if (chip->type == MAX20355) {
		reg_vcell_byte1 = MAX20355_REG_VCELL_BYTE_1;
		reg_vcell_byte0 = MAX20355_REG_VCELL_BYTE_0;
		reg_ready = MAX20355_REG_READY_REG;
		ready_bit = MAX20355_READY_VCELL_RDY_BIT;
	} else {
		reg_vcell_byte1 = MAX20357_REG_VCELL_BYTE_1;
		reg_vcell_byte0 = MAX20357_REG_VCELL_BYTE_0;
		reg_ready = MAX20357_REG_READY_REG;
		ready_bit = MAX20357_READY_VCELL_RDY_BIT;
	}

	/* Check if data is reliable (READY_REG) */
	ret = regmap_read(chip->regmap, reg_ready, &ready_reg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read READY_REG (ret: %d)\n", __func__, ret);
		return ret;
	}

	if (!(ready_reg & ready_bit)) {
		dev_warn(chip->dev, "%s: VCELL data not reliable\n", __func__);
		return -EAGAIN;
	}

	/* Read VCELL byte1 and byte0 */
	ret = regmap_read(chip->regmap, reg_vcell_byte1, &byte1);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read VCELL_BYTE_1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_read(chip->regmap, reg_vcell_byte0, &byte0);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read VCELL_BYTE_0 (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* Concatenate byte1 and byte0 */
	vcell_raw = (byte1 << 8) | byte0;

	/* Convert to microvolts (LSB = 78.125μV = 78125nV = 78.125μV) */
	/* vcell_uv = vcell_raw * 78.125 = vcell_raw * 78125 / 1000 */
	/* Use long long to prevent overflow: 65535 * 78125 = 5,120,953,125 > INT_MAX */
	vcell_uv = (int)div_s64((s64)vcell_raw * 78125, 1000);

	dev_info(chip->dev, "%s: VCELL: %d.%03d mV (0x%04x)\n",
		__func__, vcell_uv / 1000, vcell_uv % 1000, vcell_raw);

	return vcell_uv;
}

static int max2035x_read_tte(struct max2035x *chip)
{
	unsigned int ready_reg, byte1, byte0;
	unsigned int reg_tte_byte1, reg_tte_byte0, reg_ready;
	unsigned int ready_bit;
	u16 tte_raw;
	int ret, tte_sec;

	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}

	/* Select registers based on chip type */
	if (chip->type == MAX20355) {
		reg_tte_byte1 = MAX20355_REG_TTE_BYTE_1;
		reg_tte_byte0 = MAX20355_REG_TTE_BYTE_0;
		reg_ready = MAX20355_REG_READY_REG;
		ready_bit = MAX20355_READY_TTE_RDY_BIT;
	} else {
		reg_tte_byte1 = MAX20357_REG_TTE_BYTE_1;
		reg_tte_byte0 = MAX20357_REG_TTE_BYTE_0;
		reg_ready = MAX20357_REG_READY_REG;
		ready_bit = MAX20357_READY_TTE_RDY_BIT;
	}

	/* Check if data is reliable (READY_REG) */
	ret = regmap_read(chip->regmap, reg_ready, &ready_reg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read READY_REG (ret: %d)\n", __func__, ret);
		return ret;
	}

	if (!(ready_reg & ready_bit)) {
		dev_warn(chip->dev, "%s: TTE data not reliable\n", __func__);
		return -EAGAIN;
	}

	/* Read TTE byte1 and byte0 */
	ret = regmap_read(chip->regmap, reg_tte_byte1, &byte1);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read TTE_BYTE_1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_read(chip->regmap, reg_tte_byte0, &byte0);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read TTE_BYTE_0 (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* Concatenate byte1 and byte0 */
	tte_raw = (byte1 << 8) | byte0;

	/* Convert to seconds (LSB = 5.625s = 5625ms / 1000) */
	/* tte_sec = tte_raw * 5.625 = tte_raw * 5625 / 1000 */
	tte_sec = (tte_raw * 5625) / 1000;

	dev_info(chip->dev, "%s: TTE: %d sec (%dh %dm %ds) (0x%04x)\n",
		__func__, tte_sec, tte_sec / 3600, (tte_sec % 3600) / 60, tte_sec % 60, tte_raw);

	return tte_sec;
}

static int max2035x_read_avgvcell(struct max2035x *chip)
{
	unsigned int ready_reg, byte1, byte0;
	unsigned int reg_avgvcell_byte1, reg_avgvcell_byte0, reg_ready;
	unsigned int ready_bit;
	u16 avgvcell_raw;
	int ret, avgvcell_uv;

	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}

	/* Select registers based on chip type */
	if (chip->type == MAX20355) {
		reg_avgvcell_byte1 = MAX20355_REG_AVGVCELL_BYTE_1;
		reg_avgvcell_byte0 = MAX20355_REG_AVGVCELL_BYTE_0;
		reg_ready = MAX20355_REG_READY_REG;
		ready_bit = MAX20355_READY_AVGVCELL_RDY_BIT;
	} else {
		reg_avgvcell_byte1 = MAX20357_REG_AVGVCELL_BYTE_1;
		reg_avgvcell_byte0 = MAX20357_REG_AVGVCELL_BYTE_0;
		reg_ready = MAX20357_REG_READY_REG;
		ready_bit = MAX20357_READY_AVGVCELL_RDY_BIT;
	}

	/* Check if data is reliable (READY_REG) */
	ret = regmap_read(chip->regmap, reg_ready, &ready_reg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read READY_REG (ret: %d)\n", __func__, ret);
		return ret;
	}

	if (!(ready_reg & ready_bit)) {
		dev_warn(chip->dev, "%s: AVGVCELL data not reliable (READY_REG=0x%02x)\n", __func__, ready_reg);
		return -EAGAIN;
	}

	/* Read AVGVCELL byte1 and byte0 */
	ret = regmap_read(chip->regmap, reg_avgvcell_byte1, &byte1);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read AVGVCELL_BYTE_1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_read(chip->regmap, reg_avgvcell_byte0, &byte0);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read AVGVCELL_BYTE_0 (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* Concatenate byte1 and byte0 */
	avgvcell_raw = (byte1 << 8) | byte0;

	/* Convert to microvolts (LSB = 78.125μV) */
	/* Use long long to prevent overflow: 65535 * 78125 = 5,120,953,125 > INT_MAX */
	avgvcell_uv = (int)div_s64((s64)avgvcell_raw * 78125, 1000);

	dev_info(chip->dev, "%s: AVGVCELL: %d.%03d mV (0x%04x)\n",
		__func__, avgvcell_uv / 1000, avgvcell_uv % 1000, avgvcell_raw);

	return avgvcell_uv;
}

static int max2035x_read_ttf(struct max2035x *chip)
{
	unsigned int ready_reg, byte1, byte0;
	unsigned int reg_ttf_byte1, reg_ttf_byte0, reg_ready;
	unsigned int ready_bit;
	u16 ttf_raw;
	int ret, ttf_sec;

	if (!chip) {
		pr_err("%s: chip is NULL\n", __func__);
		return -EINVAL;
	}

	/* Select registers based on chip type */
	if (chip->type == MAX20355) {
		reg_ttf_byte1 = MAX20355_REG_TTF_BYTE_1;
		reg_ttf_byte0 = MAX20355_REG_TTF_BYTE_0;
		reg_ready = MAX20355_REG_READY_REG;
		ready_bit = MAX20355_READY_TTF_RDY_BIT;
	} else {
		reg_ttf_byte1 = MAX20357_REG_TTF_BYTE_1;
		reg_ttf_byte0 = MAX20357_REG_TTF_BYTE_0;
		reg_ready = MAX20357_REG_READY_REG;
		ready_bit = MAX20357_READY_TTF_RDY_BIT;
	}

	/* Check if data is reliable (READY_REG) */
	ret = regmap_read(chip->regmap, reg_ready, &ready_reg);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read READY_REG (ret: %d)\n", __func__, ret);
		return ret;
	}

	if (!(ready_reg & ready_bit)) {
		dev_warn(chip->dev, "%s: TTF data not reliable\n", __func__);
		return -EAGAIN;
	}

	/* Read TTF byte1 and byte0 */
	ret = regmap_read(chip->regmap, reg_ttf_byte1, &byte1);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read TTF_BYTE_1 (ret: %d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_read(chip->regmap, reg_ttf_byte0, &byte0);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read TTF_BYTE_0 (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* Concatenate byte1 and byte0 */
	ttf_raw = (byte1 << 8) | byte0;

	/* Convert to seconds (LSB = 5.625s) */
	ttf_sec = (ttf_raw * 5625) / 1000;

	dev_info(chip->dev, "%s: TTF: %d sec (%dh %dm %ds) (0x%04x)\n",
		__func__, ttf_sec, ttf_sec / 3600, (ttf_sec % 3600) / 60, ttf_sec % 60, ttf_raw);

	return ttf_sec;
}

static int max20355_read_slave_battery_voltage(struct max2035x *chip, int target_slave)
{
	unsigned int reg_val, status_val;
	unsigned int reg_addr, reliable_bit;
	int ret, voltage_mv;

	if (!chip || target_slave < 1 || target_slave > 2)
		return -EINVAL;

	if (chip->type != MAX20355) {
		dev_err(chip->dev, "%s: Only MAX20355 supports slave FG reading\n", __func__);
		return -ENOTSUPP;
	}

	/* Check if data is reliable first (FG_RDY_5) */
	ret = regmap_read(chip->regmap, MAX20355_REG_FG_RDY_5, &status_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read FG_RDY_5 (ret: %d)\n", __func__, ret);
		return ret;
	}

	reliable_bit = (target_slave == 1) ? MAX20355_FG_RDY_SLV_AVGRDY1_BIT : MAX20355_FG_RDY_SLV_AVGRDY2_BIT;

	if (!(status_val & reliable_bit)) {
		dev_warn(chip->dev, "%s: Slave_%d average voltage data not reliable\n", __func__, target_slave);
		return -EAGAIN;
	}

	/* Select register based on channel */
	reg_addr = (target_slave == 1) ? MAX20355_REG_FG_RDY_1 : MAX20355_REG_FG_RDY_3;

	ret = regmap_read(chip->regmap, reg_addr, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read FG_RDY_%d (ret: %d)\n",
			__func__, (target_slave == 1) ? 1 : 3, ret);
		return ret;
	}

	/* Extract voltage value (8-bit) */
	reg_val = (reg_val & MAX20355_FG_RDY_SLV_AVG_MASK) >> MAX20355_FG_RDY_SLV_AVG_SHIFT;

	/* Convert to millivolts (LSB = 20mV) */
	voltage_mv = reg_val * 20;

	dev_info(chip->dev, "%s: Slave_%d Average Battery Voltage : %d mV (0x%02x)\n", __func__, target_slave, voltage_mv, reg_val);

	return voltage_mv;
}

static int max20355_read_slave_soc(struct max2035x *chip, int target_slave)
{
	unsigned int reg_val, status_val;
	unsigned int reg_addr, reliable_bit;
	int ret, soc_percent;

	if (!chip || target_slave < 1 || target_slave > 2)
		return -EINVAL;

	if (chip->type != MAX20355) {
		dev_err(chip->dev, "%s: Only MAX20355 supports slave FG reading\n", __func__);
		return -ENOTSUPP;
	}

	/* Check if data is reliable first (FG_RDY_5) */
	ret = regmap_read(chip->regmap, MAX20355_REG_FG_RDY_5, &status_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read FG_RDY_5 (ret: %d)\n", __func__, ret);
		return ret;
	}

	reliable_bit = (target_slave == 1) ? MAX20355_FG_RDY_SLV_SOCRDY1_BIT : MAX20355_FG_RDY_SLV_SOCRDY2_BIT;

	if (!(status_val & reliable_bit)) {
		dev_warn(chip->dev, "%s: Slave_%d SOC data not reliable\n", __func__, target_slave);
		return -EAGAIN;
	}

	/* Select register based on channel */
	reg_addr = (target_slave == 1) ? MAX20355_REG_FG_RDY_2 : MAX20355_REG_FG_RDY_4;

	ret = regmap_read(chip->regmap, reg_addr, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read FG_RDY_%d (ret: %d)\n",
			__func__, (target_slave == 1) ? 2 : 4, ret);
		return ret;
	}

	/* Extract SOC value (8-bit) */
	reg_val = (reg_val & MAX20355_FG_RDY_SLV_SOC_MASK) >> MAX20355_FG_RDY_SLV_SOC_SHIFT;

	/* SOC in percentage (LSB = 1%) */
	soc_percent = reg_val;

	dev_info(chip->dev, "%s: Slave_%d SOC: %d%% (0x%02x)\n", __func__, target_slave, soc_percent, reg_val);

	return soc_percent;
}

static int max20355_read_slave_charger_done(struct max2035x *chip, int target_slave)
{
	unsigned int reg_val;
	unsigned int chg_done_bit;
	int ret, is_done;

	if (!chip || target_slave < 1 || target_slave > 2)
		return -EINVAL;

	if (chip->type != MAX20355) {
		dev_err(chip->dev, "%s: Only MAX20355 supports slave FG reading\n", __func__);
		return -ENOTSUPP;
	}

	ret = regmap_read(chip->regmap, MAX20355_REG_FG_RDY_5, &reg_val);
	if (ret) {
		dev_err(chip->dev, "%s: Failed to read FG_RDY_5 (ret: %d)\n", __func__, ret);
		return ret;
	}

	chg_done_bit = (target_slave == 1) ? MAX20355_FG_RDY_SLV1_CHG_DNE_BIT : MAX20355_FG_RDY_SLV2_CHG_DNE_BIT;

	is_done = !!(reg_val & chg_done_bit);

	dev_info(chip->dev, "%s: CH%d charger done: %s\n", __func__, target_slave, is_done ? "YES" : "NO");

	return is_done;
}

static void max2035x_plc_handle_events(struct max2035x_plc *plc, unsigned long events)
{
	struct max2035x *chip = plc->chip;
	unsigned int reg_val;
	int ret;

	if (chip->type == MAX20355) {
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
				dev_info(plc->dev, "%s : [MAX20355] PLC_1 IDLE State\n", __func__);
			} else {
				dev_info(plc->dev, "%s : [MAX20355] PLC_1 Not in IDLE State\n", __func__);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_IDLE)) {
			regmap_read(chip->regmap, MAX20355_REG_STATUS0, &reg_val);
			if (reg_val & MAX20355_STATUS0_CH2_IDL_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20355] PLC_2 IDLE State\n", __func__);
			} else {
				dev_info(plc->dev, "%s : [MAX20355] PLC_2 Not in IDLE State\n", __func__);
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
			dev_info(plc->dev, "%s : [MAX20355] New PLC Data Received\n", __func__);

			int actual_bytes = 128;
			u8 rx_buf[128];

			if (plc->bulk_transfer_active) {
				ret = regmap_read(chip->regmap, MAX20355_REG_PLC_RX, &reg_val);
				if (!ret) {
					int plc_ch = (reg_val & MAX20355_PLC_RX_CH_SEL_BIT) ? 2 : 1;
					actual_bytes = (reg_val & 0x7F) + 1;
					dev_info(plc->dev, "%s : [MAX20355] Bulk Data received from PLC Channel %d (%d bytes)\n", __func__, plc_ch, actual_bytes);
				} else {
					actual_bytes = 128;
					dev_err(plc->dev, "%s : Failed to read PLC_RX (0x3C)\n", __func__);
				}

				max2035x_read_ram_data(chip, rx_buf, actual_bytes);
			} else {
				max2035x_read_ram_data(chip, rx_buf, 128);
			}

			regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG5, &reg_val);
			if (reg_val & MAX20355_PLC_CFG5_RAM_IS_FULL_BIT) {
				regmap_write(chip->regmap, MAX20355_REG_PLC_CONFIG5, reg_val | MAX20355_PLC_CFG5_RAM_IS_FULL_BIT);
				regmap_read(chip->regmap, MAX20355_REG_PLC_CONFIG5, &reg_val);

				if (!(reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT)) {
					dev_info(plc->dev, "%s : [MAX20355] Mailbox data received and RAM cleared (Reg: 0x%02x)\n", __func__, reg_val);
				} else {
					dev_warn(plc->dev, "%s : [MAX20355] Mailbox data received but Failed to clear RAM_is_full (Reg: 0x%02x)\n", __func__, reg_val);
				}
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_CMD_DONE) || events & BIT(MAX20355_PLC_EVENT_CH1_CMD_ERROR)) {
			max20355_check_plc_status(plc, 1);
			if (plc->bulk_transfer_active) {
				complete(&plc->cmd_done);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_CMD_DONE) || events & BIT(MAX20355_PLC_EVENT_CH2_CMD_ERROR)) {
			max20355_check_plc_status(plc, 2);
			if (plc->bulk_transfer_active) {
				complete(&plc->cmd_done);
			}
		}

		if (events & BIT(MAX20355_PLC_EVENT_MOI_DETECTED_VALID_RESULT)) {
			dev_dbg(chip->dev, "%s: [MAX20355] Valid Moisture detection Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_ABORT)) {
			dev_info(chip->dev, "%s: [MAX20355] Abort resistive Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_OPEN)) {
			dev_dbg(chip->dev, "%s: [MAX20355] Open resistive Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_RESISTIVE_MEASURE_GND)) {
			dev_info(chip->dev, "%s: [MAX20355] Ground resistive Detected\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_TIMEOUT)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_2 UART Timeout\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_MODE_FAIL)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_2 UART Mode Entry Failed\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH2_UART_MODE_DONE)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_2 UART Mode Entry Successful\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_TIMEOUT)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_1 UART Timeout\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_MODE_FAIL)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_1 UART Mode Entry Failed\n", __func__);
		}

		if (events & BIT(MAX20355_PLC_EVENT_CH1_UART_MODE_DONE)) {
			dev_info(chip->dev, "%s: [MAX20355] PLC_1 UART Mode Entry Successful\n", __func__);
		}
	} else if (chip->type == MAX20357) {
		if (events & BIT(MAX20357_PLC_EVENT_CHG_THM_SHDN)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS1, &reg_val);
			if (reg_val & MAX20357_STATUS1_CHG_THRM_REG_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] Thermal Regulation Inactive (Normal or Disabled)\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] Thermal Regulation Active (Current Reduced)\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_CC1_TIMEOUT)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS1, &reg_val);
			if (reg_val & MAX20357_STATUS1_CC1_TMO_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] CC1 timeout not expired\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] CC1 timeout expired\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_CHG_MODE)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS1, &reg_val);
			u8 chg_stat = (reg_val & MAX20357_STATUS1_CHGSTAT_MASK);

			const char *chg_stat_str = "Unknown";
			bool stop_charging = false;

			switch (chg_stat) {
			case 0x0:
				chg_stat_str = "Charger off";
				break;
			case 0x1:
				chg_stat_str = "Charger IDLE mode";
				break;
			case 0x2:
				chg_stat_str = "Pre-charge";
				break;
			case 0x3:
				chg_stat_str = "Fast-charge CC1";
				break;
			case 0x4:
				chg_stat_str = "Fast-charge CC2";
				break;
			case 0x5:
				chg_stat_str = "Fast-charge CV";
				break;
			case 0x6:
				chg_stat_str = "Maintain charger in progress";
				break;
			case 0x7:
				chg_stat_str = "Maintain charge done (Full)";
				break;
			case 0x8:
				chg_stat_str = "Fault: Pre-charge Timer Expired";
				stop_charging = true;
				break;
			case 0x9:
				chg_stat_str = "Fault: Safety Timer Expired";
				stop_charging = true;
				break;
			case 0xE:
				chg_stat_str = "CC Tracking in progress";
				break;
			case 0xF:
				chg_stat_str = "Suspend: JEITA Temperature Violation";
				break;
			default:
				chg_stat_str = "NA";
				break;
			}

			dev_info(plc->dev, "%s : [MAX20357_CH%d] Changed Charger Mode : %s (0x%02x)\n", __func__, chip->channel_id, chg_stat_str, chg_stat);

			if (stop_charging) {
				dev_err(plc->dev, "%s : [MAX20357_CH%d]Charging suspended due to critical fault!\n", __func__, chip->channel_id);

				max20357_set_charger_enable(chip, false);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_CHG_RESTART)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS1, &reg_val);
			if (reg_val & MAX20357_STATUS1_CHG_RESTA_B_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] charger restarts\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] No charger restart event\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_JEITA_THM_MON)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
			u8 thm_stat = (reg_val & MAX20357_STATUS0_THMSTAT_MASK);
			const char *thm_stat_str = "Unknown";

			switch (thm_stat) {
			case 0x0:
			case 0x1:
				thm_stat_str = "Cold zone";
				break;
			case 0x2:
				thm_stat_str = "Room zone";
				break;
			case 0x3:
				thm_stat_str = "Warm zone";
				break;
			case 0x4:
				thm_stat_str = "Hot zone";
				break;
			case 0x5:
				thm_stat_str = "No thermistor detected";
				break;
			case 0x6:
			case 0x7:
				thm_stat_str = "Thermistor monitoring disabled";
				break;
			default:
				thm_stat_str = "NA";
				break;
			}
			dev_info(plc->dev, "%s : [MAX20357_CH%d] JEITA Status: %s (0x%02x)\n", __func__, chip->channel_id, thm_stat_str, thm_stat);
		}


		if (events & BIT(MAX20357_PLC_EVENT_PLC_VOLT)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS3, &reg_val);
			if (reg_val & MAX20357_STATUS3_PLCOk_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] Vplc > Vplcdet\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] Vplc < Vplcdet (PLC input voltage not present)\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_SYS_LDO_REV_PROT)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS3, &reg_val);
			if (reg_val & MAX20357_STATUS3_SYSREV_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] Vplc > Vsys\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] Vplc < Vsys (SYS LDO is in reverse protection)\n", __func__, chip->channel_id);
			}
		}

		/* PLC Detection Mode -> Link Mode Trasition */
		if (events & BIT(MAX20357_PLC_EVENT_CONNECTION)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
			if (reg_val & MAX20357_STATUS0_CHN_CON_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] PLC Connected\n", __func__, chip->channel_id);

				/* Reset charger logic for a clean start */
				max20357_set_charger_reset(chip, true);

				/* Enable charger to start the charging sequence */
				max20357_set_charger_enable(chip, true);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] PLC Disconnected\n", __func__, chip->channel_id);

				/* Disable charger upon disconnection */
				max20357_set_charger_enable(chip, false);
			}
		}

		/* PLC Link Mode -> IDLE Mode Trasition */
		if (events & BIT(MAX20357_PLC_EVENT_IDLE)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS0, &reg_val);
			if (reg_val & MAX20357_STATUS0_CHN_IDL_STS_BIT) {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] PLC IDLE State\n", __func__, chip->channel_id);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] PLC Not in IDLE State\n", __func__, chip->channel_id);
			}
		}

		/* Data Transfer (Operation within PLC Link Mode) */
		if (events & BIT(MAX20357_PLC_EVENT_NEW_DATA)) {
			dev_info(plc->dev, "%s : [MAX20357_CH%d] New PLC Data Received\n", __func__, chip->channel_id);

			int actual_bytes = 128;
			u8 rx_buf[128];

			if (plc->bulk_transfer_active) {
				ret = regmap_read(chip->regmap, MAX20357_REG_PLC_RX, &reg_val);
				if (!ret) {
					actual_bytes = (reg_val & 0x7F) + 1;
					dev_info(plc->dev, "%s : [MAX20357_CH%d] Bulk Data received (%d bytes)\n", __func__, chip->channel_id, actual_bytes);
				} else {
					actual_bytes = 128;
					dev_err(plc->dev, "%s : [MAX20357_CH%d] Failed to read PLC_RX (0x39)\n", __func__, chip->channel_id);
				}

				max2035x_read_ram_data(chip, rx_buf, actual_bytes);
			} else {
				max2035x_read_ram_data(chip, rx_buf, 128);
			}

			regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);
			if (reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT) {
				regmap_write(chip->regmap, MAX20357_REG_PLC_CONFIG4, reg_val | MAX20357_PLC_CFG4_RAM_IS_FULL_BIT);
				regmap_read(chip->regmap, MAX20357_REG_PLC_CONFIG4, &reg_val);

				if (!(reg_val & MAX20357_PLC_CFG4_RAM_IS_FULL_BIT)) {
					dev_info(plc->dev, "%s : [MAX20357_CH%d] Mailbox data received and RAM cleared (Reg: 0x%02x)\n", __func__, chip->channel_id, reg_val);
				} else {
					dev_warn(plc->dev, "%s : [MAX20357_CH%d] Mailbox data received but Failed to clear RAM_is_full (Reg: 0x%02x)\n", __func__, chip->channel_id, reg_val);
				}
			}
		}

		if (events & BIT (MAX20357_PLC_EVENT_CMD_DONE) || events & BIT(MAX20357_PLC_EVENT_CMD_ERROR)) {
			max20357_check_plc_status(plc, chip->channel_id);
			if (plc->bulk_transfer_active) {
				complete(&plc->cmd_done);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_BAT_UVLO)) {
			dev_info(plc->dev, "%s : [MAX20357_CH%d] Battery UVLO\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_MOI_DONE)) {
			dev_dbg(plc->dev, "%s : [MAX20357_CH%d] Moisture measurement is completed\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_MOI_DETECTED)) {
			if (max20357_check_moisture_status(plc, chip->channel_id)) {
				dev_info(plc->dev, "%s: Disable charger due to moisture\n", __func__);
				max20357_set_charger_enable(chip, false);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_MOI_DETECTED_VALID_RESULT)) {
			dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Valid Moisture detection Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_ABORT)) {
			dev_info(chip->dev, "%s: [MAX20357_CH%d] Abort resistive Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_OPEN)) {
			dev_dbg(chip->dev, "%s: [MAX20357_CH%d] Open resistive Detected\n", __func__, chip->channel_id);
		}

		if (events & BIT(MAX20357_PLC_EVENT_RESISTIVE_MEASURE_GND)) {
			dev_info(chip->dev, "%s: [MAX20357_CH%d] Ground resistive Detected\n", __func__, chip->channel_id);
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
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Enter UART mode\n", __func__, chip->channel_id);
			} else {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Not Enter UART mode\n", __func__, chip->channel_id);
			}

			if (reg_val & MAX20357_UART_CTR1_I2C_URT_ABR_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Abort UART mode\n", __func__, chip->channel_id);
			} else {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Not abort UART mode\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_UART_SWITCH_OPEN)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS6, &reg_val);
			if (reg_val & MAX20357_STATUS6_URT_SWC_OPN_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] UART Switch is open due to timeout\n", __func__, chip->channel_id);
			} else if (reg_val & MAX20357_UART_CTR1_I2C_URT_ABR_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] No UART Switch is open due to timeout\n", __func__, chip->channel_id);
			}
		}

		if (events & BIT(MAX20357_PLC_EVENT_DEAD_MASTER)) {
			regmap_read(chip->regmap, MAX20357_REG_STATUS2, &reg_val);
			if (reg_val & MAX20357_STATUS2_DEAD_FOUND_STS_BIT) {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Dead master found\n", __func__, chip->channel_id);
			} else {
				dev_info(chip->dev, "%s: [MAX20357_CH%d] Master is Active\n", __func__, chip->channel_id);
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

				dev_info(plc->dev, "%s : [MAX20357_CH%d] OTP loading completed (BotCode: %s / Last Reset: %s)\n", __func__, chip->channel_id, botcode_str, reset_mode_str);

				max2035x_initialize(chip);
			} else {
				dev_info(plc->dev, "%s : [MAX20357_CH%d] OTP loading not completed\n", __func__, chip->channel_id);
			}
		}
    }
}

/* Compare register bits with mapping table to dispatch events */
static void max2035x_dispatch_plc_events(struct max2035x_plc *plc)
{
	const struct max2035x_plc_irq_map *map;
	int i, map_size;

	/* Clear previous pending events */
	bitmap_zero(&plc->pending_events, BITS_PER_LONG);

	/* Select mapping table according to chip type */
	if (plc->chip->type == MAX20355) {
		map = max20355_plc_irq_map;
		map_size = ARRAY_SIZE(max20355_plc_irq_map);
	} else {
		map = max20357_plc_irq_map;
		map_size = ARRAY_SIZE(max20357_plc_irq_map);
	}

	/* Iterate through table to check which bits are active */
	for (i = 0; i < map_size; i++) {
		if (plc->irq_cache[map[i].reg_idx] & map[i].mask) {
			set_bit(map[i].event, &plc->pending_events);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* workqueue                                                                  */
/* -------------------------------------------------------------------------- */
static void max2035x_plc_work(struct work_struct *work)
{
	struct max2035x_plc *plc = container_of(work, struct max2035x_plc, work);
	unsigned long events;

	/* Dispatch raw interrupt bits into pending events */
	max2035x_dispatch_plc_events(plc);
	events = plc->pending_events;

	/* Process events through the PLC Finite State Machine (FSM) */
	if (events) {
		max2035x_plc_handle_events(plc, events);
	}
}

/* -------------------------------------------------------------------------- */
/* Notify callback (IRQ → notify)                                             */
/* -------------------------------------------------------------------------- */
static void max2035x_plc_notify(struct max2035x_plc *plc_data, u8 *int_status)
{
	struct max2035x_plc *plc = plc_data;

	if (!plc)
		return;

    /* Copy 6-byte interrupt status from Core to PLC local cache */
    memcpy(plc->irq_cache, int_status, sizeof(plc->irq_cache));

    /* Schedule worker to dispatch and process events in process context */
    schedule_work(&plc->work);
}

ssize_t max2035x_plc_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct max2035x_plc *plc = dev_get_drvdata(dev);
	struct max2035x *chip = plc->chip;
	int ret;

	if (attr == &dev_attr_soc) {
		ret = max2035x_read_soc(chip);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%d.%02d%%\n", ret / 100, ret % 100);

	} else if (attr == &dev_attr_vcell) {
		ret = max2035x_read_vcell(chip);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%d.%03d mV\n", ret / 1000, ret % 1000);

	} else if (attr == &dev_attr_tte) {
		ret = max2035x_read_tte(chip);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%d sec\n", ret);

	} else if (attr == &dev_attr_avg_vcell) {
		ret = max2035x_read_avgvcell(chip);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%d.%03d mV\n", ret / 1000, ret % 1000);

	} else if (attr == &dev_attr_ttf) {
		ret = max2035x_read_ttf(chip);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%d sec\n", ret);

	} else if (attr == &dev_attr_slave_bat_volt) {
		if (chip->type == MAX20355) {
			int volt1, volt2;

			volt1 = max20355_read_slave_battery_voltage(chip, 1);
			if (volt1 < 0)
				return volt1;

			volt2 = max20355_read_slave_battery_voltage(chip, 2);
			if (volt2 < 0)
				return volt2;

			return sysfs_emit(buf, "slave1: %d mV\nslave2: %d mV\n", volt1, volt2);
		}
		return sysfs_emit(buf, "Not supported\n");

	} else if (attr == &dev_attr_slave_soc) {
		if (chip->type == MAX20355) {
			int soc1, soc2;

			soc1 = max20355_read_slave_soc(chip, 1);
			if (soc1 < 0)
				return soc1;

			soc2 = max20355_read_slave_soc(chip, 2);
			if (soc2 < 0)
				return soc2;

			return sysfs_emit(buf, "slave1: %d%%\nslave2: %d%%\n", soc1, soc2);
		}
		return sysfs_emit(buf, "Not supported\n");
	}

	return -EINVAL;
}

static struct attribute *max2035x_plc_attrs[] = {
	&dev_attr_soc.attr,
	&dev_attr_vcell.attr,
	&dev_attr_tte.attr,
	&dev_attr_avg_vcell.attr,
	&dev_attr_ttf.attr,
	&dev_attr_slave_bat_volt.attr,
	&dev_attr_slave_soc.attr,
	NULL,
};

static const struct attribute_group max2035x_plc_attr_group = {
	.name = "plc_control",
	.attrs = max2035x_plc_attrs,
};

static int max2035x_plc_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct max2035x_plc *plc;
	int ret;

	plc = devm_kzalloc(&pdev->dev, sizeof(*plc), GFP_KERNEL);
	if (!plc)
		return dev_err_probe(&pdev->dev, -ENOMEM, "%s : Failed to allocate MAX2035X PLC object\n", __func__);

	plc->dev = &pdev->dev;
	plc->chip = chip;

	init_completion(&plc->cmd_done);
	mutex_init(&plc->mode_lock);
	plc->bulk_transfer_active = false;

	bitmap_zero(&plc->pending_events, BITS_PER_LONG);
	INIT_WORK(&plc->work, max2035x_plc_work);

	if (chip->type == MAX20355) {
		chip->plc_notify_20355 = (max20355_plc_notify_t)max2035x_plc_notify;
	} else {
		chip->plc_notify_20357 = (max20357_plc_notify_t)max2035x_plc_notify;
	}

	chip->plc_data = plc;

	max2035x_initialize(chip);

	platform_set_drvdata(pdev, plc);

	ret = sysfs_create_group(&pdev->dev.kobj, &max2035x_plc_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "%s : Failed to create %s plc_control sysfs group\n",
				__func__, (chip->type == MAX20355) ? "MAX20355" : "MAX20357");
		return ret;
	}

	dev_info(&pdev->dev, "%s : %s PLC driver probed\n",
			__func__, (chip->type == MAX20355) ? "MAX20355" : "MAX20357");

	return 0;
}

static void max2035x_plc_remove(struct platform_device *pdev)
{
	struct max2035x_plc *plc = platform_get_drvdata(pdev);

	if (!plc)
		return;

	sysfs_remove_group(&pdev->dev.kobj, &max2035x_plc_attr_group);

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
