/**
 * \file
 * \brief Contains ADRV9001 related function prototypes for adi_adrv9001_stream.c
 *
 * Copyright 2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_STREAM_C0_H_
#define _ADI_ADRV9001_STREAM_C0_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_stream_types.h"
#include "adi_adrv9001_arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

 /**
 * \brief Configure the DGPIO to debug stream processor operation status
 *        Dedicated GPIO pins are used to route stream processor operation status:
 *        Rx1 stream processor --> DGPIO0
 *        Tx1 stream processor --> DGPIO1
 *        Rx2 stream processor --> DGPIO2 
 *        Tx2 stream processor --> DGPIO3
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre All channels state either in STANDBY or CALIBRATED
 *
 * \param[in]  adrv9001          Context variable - Pointer to the ADRV9001 device data structure
 * \param[in]  streamToGpioDebug Flag to enable or disable stream to GPIO debug feature
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Stream_C0_Gpio_Debug_Set(adi_adrv9001_Device_t *adrv9001, bool streamToGpioDebug);

 /**
 * \brief Get the current enabledness of stream to GPIO debug feature
 *
 * \note Message type: \ref timing_mailbox "Mailbox command"
 *
 * \pre All channels state any of STANDBY, CALIBRATED, PRIMED, RF_ENABLED
 *
 * \param[in]  adrv9001          Context variable - Pointer to the ADRV9001 device data structure
 * \param[out] streamToGpioDebug Current enabledness of stream to GPIO debug feature
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
int32_t adi_adrv9001_Stream_C0_Gpio_Debug_Get(adi_adrv9001_Device_t *adrv9001, bool *streamToGpioDebug);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_STREAM_C0_H_ */
