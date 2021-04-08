/**
* \file
* \brief Data types for configuring C0-specific Rx features
*
* Copyright 2021 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_RX_TYPES_C0_H_
#define _ADI_ADRV9001_RX_TYPES_C0_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

/**
 * \brief Structure which holds the ADC switch configuration
 */
typedef struct adi_adrv9001_RxPortSwitchCfg
{
    uint64_t  minFreqPortA_Hz;
    uint64_t  maxFreqPortA_Hz;
    uint64_t  minFreqPortB_Hz;
    uint64_t  maxFreqPortB_Hz;
    bool      enable;
} adi_adrv9001_RxPortSwitchCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RX_TYPES_C0_H_ */
