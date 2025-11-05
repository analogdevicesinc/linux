/**
 * \file
 * \brief Contains functions to configure AUX ADC channels on the ADRV910X device
 *
 * ADRV910X API Version: $ADI_ADRV910X_API_VERSION$
 */

 /**
 * Copyright 2025 Analog Devices Inc.
 * Released under the ADRV910X API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV910X_AUXADC_TYPES_H_
#define _ADI_ADRV910X_AUXADC_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

	/**
	*  \brief Enum to select AuxADC
	*/
	typedef enum adi_adrv910x_AuxAdc
	{		
		ADI_ADRV910X_AUXADC0,
		/*!< AuxADC0 */
		ADI_ADRV910X_AUXADC1,
		/*!< AuxADC1 */
	} adi_adrv910x_AuxAdc_e;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV910X_AUXADC_TYPES_H_ */
