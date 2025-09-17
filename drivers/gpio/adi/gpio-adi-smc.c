// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/arm-smccc.h>
#include <linux/kernel.h>
#include "gpio-adi-smc.h"

/* PINTMUX Function ID*/
#define ADI_PINTMUX_SIP_SERVICE_FUNCTION_ID  (0xC2000002)

/* ADI Pintmux SIP Service Functions*/
#define ADI_PINTMUX_MAP    (1U)
#define ADI_PINTMUX_UNMAP  (2U)

/* SMC Handler return Status Values (res.a0 return value) */
#define ADI_PINTMUX_SMC_RETURN_SUCCESS              (0U)
#define ADI_PINTMUX_SMC_RETURN_UNSUPPORTED_REQUEST  (0xFFFFFFFFFFFFFFFFU)

/* SMC Pintmux Handler return values (res.a1 return value) */
#define ADI_TFA_PINTMUX_ERR_LOOKUP_FAIL  (0xFFFFFFFFFFFFFFFFU)
#define ADI_TFA_PINTMUX_ERR_MAP_FAIL     (0xFFFFFFFFFFFFFFFEU)
#define ADI_TFA_PINTMUX_ERR_NOT_MAPPED   (0xFFFFFFFFFFFFFFFDU)
#define ADI_TFA_PINTMUX_ERR_SECURITY     (0xFFFFFFFFFFFFFFFCU)

static bool adi_adrv906x_pintmux_smc(unsigned int fid, unsigned int pin_id, bool polarity, unsigned int *irq_num, uintptr_t base_addr)
{
	struct arm_smccc_res res = { 0 };

	/*
	 * Setup  smc call to perform the pintmux operation
	 *
	 * arm_smccc_smc expected params:
	 *    param1: SMC SIP SERVICE ID
	 *    param2: ADI Pintmux function id (MAP, UNMAP)
	 *    param3: Pin Number requested
	 *    param4: Polarity
	 *    param5: Base Address of Pinctrl
	 *    param6-8: Currently UNUSED/UNDEFINED
	 *    param9: response output of the SMC call
	 *               a0= SMC return value
	 *               a1= irq number (or <0 if error)
	 *
	 */
	arm_smccc_smc(ADI_PINTMUX_SIP_SERVICE_FUNCTION_ID,
		      fid,
		      pin_id,
		      polarity,
		      base_addr,
		      0, 0, 0,
		      &res);

	/* Check SMC error */
	if (res.a0 != ADI_PINTMUX_SMC_RETURN_SUCCESS) {
		printk(KERN_ERR "ADI SMC_ERR 0x%016lx\n", res.a0);
		return false;
	}

	/* Check PINTMUX function error */
	switch (res.a1) {
	case ADI_TFA_PINTMUX_ERR_LOOKUP_FAIL:
	case ADI_TFA_PINTMUX_ERR_MAP_FAIL:
	case ADI_TFA_PINTMUX_ERR_NOT_MAPPED:
	case ADI_TFA_PINTMUX_ERR_SECURITY:
		printk(KERN_ERR "ADI PINTMUX Service ERR 0x%016lx\n", res.a1);
		return false;
	default:
		break;
	}

	/* Get IRQ number */
	if (irq_num)
		*irq_num = (unsigned int)res.a1;

	return true;
}

bool adi_adrv906x_pintmux_map(unsigned int gpio, bool polarity, unsigned int *irq, uintptr_t base_addr)
{
	if (!irq) {
		printk(KERN_ERR "%s :: Invalid arguments\n", __FUNCTION__);
		return false;
	}
	return adi_adrv906x_pintmux_smc(ADI_PINTMUX_MAP, gpio, polarity, irq, base_addr);
}

bool adi_adrv906x_pintmux_unmap(unsigned int gpio, uintptr_t base_addr)
{
	return adi_adrv906x_pintmux_smc(ADI_PINTMUX_UNMAP, gpio, true, NULL, base_addr);
}
