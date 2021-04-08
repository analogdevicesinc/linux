/**
 * \file
 * \brief C0-specific ADRV9001 Digital Pre-Distortion (DPD) related function prototypes
 *
 * Copyright 2019-2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_DPD_C0_H_
#define _ADI_ADRV9001_DPD_C0_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_dpd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure DPD FH frequency regions
 * 
 * \note Message type: \ref timing_mailbox "Mailbox command"
 * 
 * \pre Channel state are STANDBY, CALIBRATED
 *
 * \param[in] adrv9001		Context variable - Pointer to the ADRV9001 device settings data structure
 * \param[in] channel       The Tx channel of interest
 * \param[in] dpdFhRegions	The DPD FH frequncy regions to configure
 * \param[in] size          Size of DPD FH regions to configure
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_dpd_c0_fh_regions_Configure(adi_adrv9001_Device_t *adrv9001,
                                                 adi_common_ChannelNumber_e channel,
                                                 adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                                 uint32_t size);

/**
 * \brief Inspect DPD FH frequency regions
 * 
 * \note Message type: \ref timing_mailbox "Mailbox command"
 * 
 * \pre Channel state are STANDBY, CALIBRATED
 *
 * \param[in]  adrv9001		 Context variable - Pointer to the ADRV9001 device settings data structure
 * \param[in]  channel       The Tx channel of interest
 * \param[out] dpdFhRegions	 The DPD FH frequncy regions to configure; Maximum regions is '7'
 * \param[in]  size          Size of DPD FH regions to inspect
 * 
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_dpd_c0_fh_regions_Inspect(adi_adrv9001_Device_t *adrv9001,
                                               adi_common_ChannelNumber_e channel,
                                               adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                               uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_DPD_C0_H_ */
