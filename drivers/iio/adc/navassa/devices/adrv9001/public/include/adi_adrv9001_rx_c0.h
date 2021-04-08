/**
 * \file
 * \brief Functions for configuring C0-specific receiver (Rx) features
 *
 * Copyright 2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RX_C0_H_
#define _ADI_ADRV9001_RX_C0_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_rx_c0_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure RX port switching
 * 
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre Channel state must be STANDBY
 *
 * \param[in] adrv9001         Context variable - Pointer to the ADRV9001 device data structure
 * \param[in] switchConfig     The desired RX port switch configuration
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Rx_C0_PortSwitch_Configure(adi_adrv9001_Device_t *adrv9001, 
                                             adi_adrv9001_RxPortSwitchCfg_t *switchConfig);

/**
 * \brief Inspect the current RX port switching settings
 * 
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre Channel state any of STANDBY, CALIBRATED, PRIMED, RF_ENABLED
 *
 * \param[in]  adrv9001         Context variable - Pointer to the ADRV9001 device data structure
 * \param[out] switchConfig     The current RX port switch configuration
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Rx_C0_PortSwitch_Inspect(adi_adrv9001_Device_t *adrv9001, 
                                           adi_adrv9001_RxPortSwitchCfg_t *switchConfig);


#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RX_C0_H_ */
