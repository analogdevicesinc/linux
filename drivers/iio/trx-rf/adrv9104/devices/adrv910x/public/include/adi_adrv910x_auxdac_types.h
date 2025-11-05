#pragma once
/**
 * \file
 * \brief Contains functions to configure AUX DAC channels on the ADRV910X device
 *
 * ADRV910X API Version: $ADI_ADRV910X_API_VERSION$
 */

 /**
 * Copyright 2025 Analog Devices Inc.
 * Released under the ADRV910X API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV910X_AUXDAC_TYPES_H_
#define _ADI_ADRV910X_AUXDAC_TYPES_H_

/**
*  \brief Enum to select AuxDAC
*/
typedef enum adi_adrv910x_AuxDac
{
	ADI_ADRV910X_AUXDAC0,
	/*!< AuxDAC0 */
	ADI_ADRV910X_AUXDAC1,
	/*!< AuxDAC1 */
	ADI_ADRV910X_AUXDAC2,
	/*!< AuxDAC2 */
	ADI_ADRV910X_AUXDAC3,
	/*!< AuxDAC3 */
} adi_adrv910x_AuxDac_e;

#endif /* _ADI_ADRV910X_AUXDAC_TYPES_H_ */