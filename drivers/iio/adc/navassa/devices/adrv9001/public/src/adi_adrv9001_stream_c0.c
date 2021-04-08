/**
* \file
* \brief Contains related function implementation defined in adi_adrv9001_stream.h
*
* Copyright 2021 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_stream_c0.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_radio.h"

#include "adrv9001_arm_macros.h"

#ifdef ADI_ADRV9001_SI_REV_C0

static int32_t __maybe_unused adi_adrv9001_Stream_C0_Gpio_Debug_Set_Validate(adi_adrv9001_Device_t *adrv9001)
{
    adi_adrv9001_RadioState_t currentState = { 0 };
    uint8_t chanId = 0u;
    uint8_t portId = 0u;
    /* Validate current state. All the channels must be in STANDBY or CALIBRATED state. */
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, adrv9001, &currentState);
    for (portId = 0u; portId < ADI_ADRV9001_NUM_PORTS; portId++)
    {
        for (chanId = 0u; chanId < ADI_ADRV9001_NUM_CHANNELS; chanId++)
        {
            if ((currentState.channelStates[portId][chanId] == ADI_ADRV9001_CHANNEL_PRIMED) ||
                (currentState.channelStates[portId][chanId] == ADI_ADRV9001_CHANNEL_RF_ENABLED))
            {
                ADI_ERROR_REPORT(&adrv9001->common,
                                 ADI_COMMON_ERRSRC_API,
                                 ADI_COMMON_ERR_API_FAIL,
                                 ADI_COMMON_ACT_ERR_CHECK_PARAM,
                                 currentState.channelStates[portId][chanId],
                                 "Error while attempting to send stream to GPIO debug mailbox command to ARM firmware. All the channels must be in STANDBY or CALIBRATED  state.");
                ADI_API_RETURN(adrv9001)
            }
        }
    }   
    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_Stream_C0_Gpio_Debug_Set(adi_adrv9001_Device_t *adrv9001, bool streamToGpioDebug)
{
    uint8_t armData[5] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Stream_C0_Gpio_Debug_Set_Validate, adrv9001);

    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t));
    armData[offset++] = (uint8_t)streamToGpioDebug;

    extData[0] =  0; /* Channel Mask; unused for this command */
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = ADRV9001_ARM_OBJECTID_GPIO_DEBUG_IN_STREAM;
    ADI_EXPECT(adi_adrv9001_arm_Config_Write, adrv9001, armData, sizeof(armData), extData, sizeof(extData))

    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_Stream_C0_Gpio_Debug_Get_Validate(adi_adrv9001_Device_t *adrv9001,
                                                                          bool *streamToGpioDebug)
{
    ADI_NULL_PTR_RETURN(&adrv9001->common, streamToGpioDebug);
    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_Stream_C0_Gpio_Debug_Get(adi_adrv9001_Device_t *adrv9001, bool *streamToGpioDebug)
{
    uint8_t armReadBack[1] = { 0 };
    uint8_t channelMask = 0;
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Stream_C0_Gpio_Debug_Get_Validate, adrv9001, streamToGpioDebug);

    ADI_EXPECT(adi_adrv9001_arm_Config_Read, adrv9001, ADRV9001_ARM_OBJECTID_GPIO_DEBUG_IN_STREAM, channelMask, offset, armReadBack, sizeof(armReadBack))

    *streamToGpioDebug = (bool)armReadBack[0];

    ADI_API_RETURN(adrv9001);
}

#else

int32_t adi_adrv9001_Stream_C0_Gpio_Debug_Set(adi_adrv9001_Device_t *adrv9001, bool streamToGpioDebug)
{
    ADI_ERROR_REPORT(adrv9001, 
                     ADI_COMMON_ERRSRC_API,
                     ADI_COMMON_ACT_ERR_API_NOT_IMPLEMENTED,
                     ADI_COMMON_ACT_ERR_RESET_FULL,
                     NULL,
                     "Error: attempted to call adi_adrv9001_Stream_C0_Gpio_Debug_Set() on non-C0 silicon");
    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_Stream_C0_Gpio_Debug_Get(adi_adrv9001_Device_t *adrv9001, bool *streamToGpioDebug)
{
    ADI_ERROR_REPORT(adrv9001, 
                     ADI_COMMON_ERRSRC_API,
                     ADI_COMMON_ACT_ERR_API_NOT_IMPLEMENTED,
                     ADI_COMMON_ACT_ERR_RESET_FULL,
                     NULL,
                     "Error: attempted to call adi_adrv9001_Stream_C0_Gpio_Debug_Get() on non-C0 silicon");
    ADI_API_RETURN(adrv9001);
}
#endif // ADI_ADRV9001_SI_REV_C0
