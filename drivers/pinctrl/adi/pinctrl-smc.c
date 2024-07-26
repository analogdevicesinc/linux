// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2022, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/arm-smccc.h>

#include "pinctrl-adi.h"

/* PINCTRL Function ID*/
#define ADI_PINCTRL_SIP_SERVICE_FUNCTION_ID                     0xC2000001

/* ADI Pinctrl SIP Service Functions*/
#define ADI_PINCTRL_INIT (0U)
#define ADI_PINCTRL_SET  (1U)
#define ADI_PINCTRL_GET  (2U)

#define MASK_AND_SHIFT(value, mask, shift) ((value & mask) >> shift)

/* SMC Handler return Status Values (res.a0 return value) */
#define ADI_PINCTRL_SMC_RETURN_SUCCESS                  (0U)
#define ADI_PINCTRL_SMC_RETURN_UNSUPPORTED_REQUEST      (0xFFFFFFFFFFFFFFFFU)

/* SMC Pinctrl Handler return values (res.a1 return value) */
#define ADI_TFA_PINCTRL_HANDLER_FAILURE                 (0U)
#define ADI_TFA_PINCTRL_HANDLER_SUCCESS                 (1U)

/* SMC Config Bitfield Config Word */
#define ADI_BITFIELD_ST_BIT_POSITION                    (0U)
#define ADI_BITFIELD_PULL_ENABLEMENT_BIT_POSITION       (1U)
#define ADI_BITFIELD_PULLUP_ENABLE_BIT_POSITION         (2U)

/* SMC GET result defines*/
#define ADI_GET_BITFIELD_1_PIN_CONFIGURED_BIT_POSITION (63U)

int adi_pinconf_get_smc(struct pinctrl_dev *pctldev, unsigned int pin_id,
			unsigned long *config)
{
	struct arm_smccc_res res;
	struct adi_pinctrl *ipctl;
	const struct adi_pin_reg *pin_reg;

	if (!pctldev || !config)
		return -EINVAL;

	ipctl = pinctrl_dev_get_drvdata(pctldev);
	pin_reg = &ipctl->pin_regs[pin_id];

	if (pin_reg->conf_reg == -1)
		return -EINVAL;

	/*
	 * Setup  smc call to perform the pinconf_get operation
	 *
	 * arm_smccc_smc expected params:
	 *    param1: SMC SIP SERVICE ID
	 *    param2: ADI Pinctrl request (GET, SET, INIT)
	 *    param3: Pin Number requested
	 *    param4: Currently UNUSED/UNDEFINED
	 *    param5: Currently UNUSED/UNDEFINED
	 *    param6: Currently UNUSED/UNDEFINED
	 *    param7: Currently UNUSED/UNDEFINED
	 *    param8: Currently UNUSED/UNDEFINED
	 *    param9: response output of the SMC call
	 *               a0= SMC return value
	 *               a1= ADI TFA Pinctrl Handler return status
	 *               a2= 64bit RESPONSE_BITFIELD_1 {PIN_CONFIGURED a2[63], //0=NOT Configured, 1=Configured
	 *                                              undefined a2[62:32],   //Undefined
	 *                                              PIN# a2[31:16],        //The requested Pin # (valid if PIN_CONFIGURED=1)
	 *                                              SourceMuxSetting a2[15:0]} //The source mux setting (valid if PIN_CONFIGURED=1)
	 *               a3= 64bit RESPONSE_BITFIELD_2 {undefined a3[63:19],
	 *                                             '3bit field' (SchmittTrigEnable | PU PD Enablement | PU Enable) a3[6:4]   //(valid if PIN_CONFIGURED=1)
	 *                                              DriveStrength a3[3:0]} //(valid only if PIN_CONFIGURED=1)
	 *
	 */
	arm_smccc_smc(ADI_PINCTRL_SIP_SERVICE_FUNCTION_ID,
		      ADI_PINCTRL_GET,
		      pin_reg->pin_num,
		      0, 0, 0, 0, 0,
		      &res);

	/*
	 *  The SMC call return status is present in res.a0,
	 *     the pinctrl TFA Handler is present in res.a1
	 */
	if (res.a0 != ADI_PINCTRL_SMC_RETURN_SUCCESS || res.a1 != ADI_TFA_PINCTRL_HANDLER_SUCCESS)
		return -EINVAL;

	/*
	 *  Here we output the received mux settings {3-bit field} , drivestrength
	 */
	if ((res.a2 & ADI_GET_BITFIELD_1_PIN_CONFIGURED_BIT_POSITION) == 0)
		*config = 0U;
	else
		*config = res.a3;

	return 0;
}

int adi_pinconf_set_smc(struct pinctrl_dev *pctldev, unsigned int pin_id,
			unsigned long *configs, unsigned int num_configs)
{
	struct arm_smccc_res res;
	struct adi_pinctrl *ipctl;
	int drive_strength;
	int schmitt_trig_enable;
	int pin_pull_enablement;
	int pin_pull_up_enable;
	int config_bitfield;
	unsigned long config;
	unsigned int pin_num;
	unsigned int mux_sel;

	if (!pctldev)
		return -EINVAL;

	ipctl = pinctrl_dev_get_drvdata(pctldev);

	pin_num = ((struct adi_pin_mio *)(configs))->input_pin;
	mux_sel = ((struct adi_pin_mio *)(configs))->mux_sel;
	config = ((struct adi_pin_mio *)(configs))->config;

	/*
	 * Setup  smc call to perform the pinconf_set operation
	 *
	 * arm_smccc_smc expected params:
	 *    param1: SMC SIP SERVICE ID
	 *    param2: ADI Pinctrl request (GET, SET, INIT)
	 *    param3: Pin Number requested
	 *    param4: Source Mux setting requested
	 *    param5: Drive Strength
	 *    param6: BIT_FIELD-3bits-(SchmittTrigEnable | PU PD Enablement | PU Enable)
	 *    param7: Base Address of Pinctrl
	 *    param8: Currently UNUSED/UNDEFINED
	 *    param9: response output of the SMC call
	 *               a0 = SMC return value
	 *               a1 = ADI TFA Pinctrl Handler return status
	 *               a2 = ADI unused
	 *               a3 = ADI unused
	 */

	drive_strength = config & ADI_CONFIG_DRIVE_STRENGTH_MASK;
	schmitt_trig_enable = (config & ADI_CONFIG_SCHMITT_TRIG_ENABLE_MASK) ? 1 : 0;
	pin_pull_enablement = (config & ADI_CONFIG_PULL_UP_DOWN_ENABLEMENT_MASK) ? 1 : 0;
	pin_pull_up_enable = (config & ADI_CONFIG_PULLUP_ENABLE_MASK) ? 1 : 0;
	config_bitfield = (schmitt_trig_enable << ADI_BITFIELD_ST_BIT_POSITION) |
			  (pin_pull_enablement << ADI_BITFIELD_PULL_ENABLEMENT_BIT_POSITION) |
			  (pin_pull_up_enable << ADI_BITFIELD_PULLUP_ENABLE_BIT_POSITION);

	arm_smccc_smc(ADI_PINCTRL_SIP_SERVICE_FUNCTION_ID,
		      ADI_PINCTRL_SET,
		      pin_num,
		      mux_sel,
		      drive_strength,
		      config_bitfield,
		      ipctl->phys_addr, 0, &res);


	/*
	 *  The SMC call return status is present in res.a0,
	 *     the pinctrl TFA Handler is present in res.a1
	 */
	if (res.a0 != ADI_PINCTRL_SMC_RETURN_SUCCESS || res.a1 != ADI_TFA_PINCTRL_HANDLER_SUCCESS)
		return -EINVAL;

	return 0;
}

MODULE_AUTHOR("Howard Massey <Howard.Massey@analog.com>");
MODULE_DESCRIPTION("ADI common SMC pinctrl driver");
MODULE_LICENSE("GPL v2");
