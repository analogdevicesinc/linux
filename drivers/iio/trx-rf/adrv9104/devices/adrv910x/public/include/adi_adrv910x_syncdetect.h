/**
 * \file
 * \brief Contains ADRV910X Sync Detection related function prototypes for adi_adrv910x_syncdetect.c
 *
 * ADRV910X API Version: $ADI_ADRV910X_API_VERSION$
 */

 /**
 * Copyright 2021 Analog Devices Inc.
 * Released under the ADRV910X API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV910X_SYNCDETECT_H_
#define _ADI_ADRV910X_SYNCDETECT_H_

#include "adi_adrv910x_syncdetect_types.h"
#include "adi_common_error_types.h"
#include "adi_adrv910x_types.h"
#include "adi_adrv910x_error.h"
#include "adi_adrv910x_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configure monitor mode vector
 *
 * \note Message type: \ref timing_direct "Direct register access"
 *
 * \param[in]  adrv910x			   Context variable - Pointer to the ADRV910X device settings data structure
 * \param[in]  monitorModeVector   The desired monitor mode vector
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_Vector_Configure(adi_adrv910x_Device_t *adrv910x,
                                                                            adi_adrv910x_SyncDetect_SyncDetectVectorCfg_t *monitorModeVector);

/**
 * \brief Get the monitor mode vector
 *
 * \note Message type: \ref timing_direct "Direct register access"
 *
 * \param[in]  adrv910x 		   Context variable - Pointer to the ADRV910X device settings data structure
 * \param[out] monitorModeVector   The current monitor mode vector
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_Vector_Inspect(adi_adrv910x_Device_t *adrv910x,
                                                                          adi_adrv910x_SyncDetect_SyncDetectVectorCfg_t *monitorModeVector);

/**
 * \brief Configure the sync search
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
 *
 * \param[in] adrv910x	        Context variable - Pointer to the ADRV910X device settings data structure
 * \param[in] syncSearchCfg     The desired Sync search settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_SyncSearch_Configure(adi_adrv910x_Device_t *adrv910x,
                                                                               adi_adrv910x_SyncDetect_SyncSearchCfg_t *syncSearchCfg);
	
/**
 * \brief Inspect the sync search
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
 *
 * \param[in] adrv910x	        Context variable - Pointer to the ADRV910X device settings data structure
 * \param[out] syncSearchCfg    The desired Sync search settings
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_SyncSearch_Inspect(adi_adrv910x_Device_t *adrv910x,
		                                                            adi_adrv910x_SyncDetect_SyncSearchCfg_t *syncSearchCfg);

/**
 * \brief Set continuous sync detection
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \note detected must be UNASSIGNED or a valid digital GPIO pin, Analog GPIO pins are not supported.
 * 
 * \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
 *
 * \param[in] adrv910x	            Context variable - Pointer to the ADRV910X device settings data structure
 * \param[in] syncDetectionEnable   Continuous sync detection Enable, 0-disable 1-enable
 * \param[in] detectedPin           Pin used to indicate the sync pattern was detected
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_ContinuousSyncDetect_Set(adi_adrv910x_Device_t *adrv910x,
																					bool syncDetectionEnable,
																					adi_adrv910x_GpioPin_e detectedPin);
/**
 * \brief Get continuous sync detection
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
 *
 * \param[in]  adrv910x	             Context variable - Pointer to the ADRV910X device settings data structure
 * \param[out] syncDetectionEnable   Continuous sync detection Enable, 0-disable 1-enable
 * \param[out] detectedPin           Pin used to indicate the sync pattern was detected
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_ContinuousSyncDetect_Get(adi_adrv910x_Device_t *adrv910x,
																						bool *syncDetectionEnable,
																						adi_adrv910x_GpioPin_e *detectedPin);
/**
 * \brief Get continuous sync detection status and information
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
 *
 * \param[in]  adrv910x	             Context variable - Pointer to the ADRV910X device settings data structure
 * \param[out] syncDetectionResult   Information about the sync detection event
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv910x_syncDetect_ContinuousSyncDetect_Status_Get(adi_adrv910x_Device_t *adrv910x,
																							adi_adrv910x_SyncDetect_SyncDetectionResult_t *syncDetectionResult);

	
/**
* \brief Set Frequency offset correction by writing NCO Control Word into PS1
*
* \note Message type : \ref timing_direct "Direct register access"
*
* \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
*
* \param[in]  adrv910x	            Context variable - Pointer to the ADRV910X device settings data structure
* \param[in]  ncoControlWord        NCO Control Word to be updated in PS1 for Frequency Offset Correction
*
* \returns A code indicating success(ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv910x_syncDetect_ContinuousSyncDetect_RxnbNco2Ftw_Set(adi_adrv910x_Device_t *adrv910x, uint32_t ncoControlWord);

/**
* \brief Set Timing offset correction by writing Resample input into PS1
*
* \note Message type : \ref timing_direct "Direct register access"
*
* \pre Channel state is #ADI_ADRV910X_CHANNEL_CALIBRATED or #ADI_ADRV910X_CHANNEL_PRIMED or #ADI_ADRV910X_CHANNEL_RF_ENABLED
*
* \param[in]  adrv910x	            Context variable - Pointer to the ADRV910X device settings data structure
* \param[in]  resampleInputI        Resample input for the I data to be updated in PS1 for Timing Offset Correction
* \param[in]  resampleInputQ        Resample input for the Q data to be updated in PS1 for Timing Offset Correction
*
* \returns A code indicating success(ADI_COMMON_ACT_NO_ACTION) or the required action to recover
*/
int32_t adi_adrv910x_syncDetect_ContinuousSyncDetect_RxnbResampPhase_Set(adi_adrv910x_Device_t *adrv910x, int16_t resampleInputI, int16_t resampleInputQ);


#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV910X_SYNCDETECT_H_ */
