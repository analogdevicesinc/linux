/**
* \file
* \brief Functions for configuring C0-specific receiver (Rx) features
*
* Copyright 2021 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_rx_c0.h"

#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_error.h"
#include "adi_adrv9001_radio.h"

#include "adrv9001_arm_macros.h"
#include "adrv9001_reg_addr_macros.h"

#ifdef ADI_ADRV9001_SI_REV_C0

static int32_t __maybe_unused adi_adrv9001_Rx_PortSwitch_Configure_Validate(adi_adrv9001_Device_t *device,
                                                                            adi_adrv9001_RxPortSwitchCfg_t *switchConfig)
{
    adi_adrv9001_ChannelState_e state = ADI_ADRV9001_CHANNEL_STANDBY;
    adi_common_ChannelNumber_e channel = ADI_CHANNEL_1;
    static const uint32_t RX_CHANNELS[] = { ADI_ADRV9001_RX1, ADI_ADRV9001_RX2 };

    uint8_t chan_index = 0;
    uint8_t port_index = 0;

    for (channel = ADI_CHANNEL_1; channel <= ADI_CHANNEL_2; channel++)
    {
        adi_common_channel_to_index(channel, &chan_index);
        if (ADRV9001_BF_EQUAL(device->devStateInfo.initializedChannels, RX_CHANNELS[chan_index]))
        {
            ADI_EXPECT(adi_adrv9001_Radio_Channel_State_Get, device, ADI_RX, channel, &state);
            if (ADI_ADRV9001_CHANNEL_STANDBY != state)
            {
                adi_common_port_to_index(ADI_RX, &port_index);
                ADI_ERROR_REPORT(&device->common,
                                 ADI_COMMON_ERRSRC_API,
                                 ADI_COMMON_ERR_API_FAIL,
                                 ADI_COMMON_ACT_ERR_CHECK_PARAM,
                                 currentState.channelStates[port_index][chan_index],
                                 "All RX channels must be in STANDBY state to configure RX port switching settings.");
                ADI_API_RETURN(device)
            }
        }
    }

    /* NULL pointer check */
    ADI_NULL_PTR_RETURN(&device->common, switchConfig);    

    /* Freuqency range check */
    ADI_RANGE_CHECK_X(device, switchConfig->minFreqPortA_Hz, ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ, ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ, "%llu");    
    ADI_RANGE_CHECK_X(device, switchConfig->maxFreqPortA_Hz, ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ, ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ, "%llu");    
    ADI_RANGE_CHECK_X(device, switchConfig->minFreqPortB_Hz, ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ, ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ, "%llu");    
    ADI_RANGE_CHECK_X(device, switchConfig->maxFreqPortB_Hz, ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ, ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ, "%llu");

    /* Min frequency must be smaller than max */
    if ((switchConfig->maxFreqPortA_Hz < switchConfig->minFreqPortA_Hz) ||
        (switchConfig->maxFreqPortB_Hz < switchConfig->minFreqPortB_Hz))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         NULL,
                         "Min frequency cannot be larger than Max frequency.");
        ADI_API_RETURN(device)
    }

    /* Make sure the two ranges do not overlap */
    if ((switchConfig->minFreqPortA_Hz <= switchConfig->maxFreqPortB_Hz) &&
        (switchConfig->minFreqPortB_Hz <= switchConfig->maxFreqPortA_Hz))
    {
        ADI_ERROR_REPORT(&device->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_API_FAIL,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         NULL,
                         "Port A and B freuqency ranges cannot overlap.");
        ADI_API_RETURN(device)
    }
     
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_C0_PortSwitch_Configure(adi_adrv9001_Device_t *device,
                                                adi_adrv9001_RxPortSwitchCfg_t *switchConfig)
{
    uint8_t armData[41] = { 0 };
    uint8_t extData[3] = { 0 };
    uint32_t offset = 0u; 

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_PortSwitch_Configure_Validate, device, switchConfig);

    /* Take away extra 4 bytes used for padding */
    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t) - 4u);
    adrv9001_LoadFourBytes(&offset, armData, 0u);

    adrv9001_LoadEightBytes(&offset, armData, switchConfig->minFreqPortA_Hz);
    adrv9001_LoadEightBytes(&offset, armData, switchConfig->maxFreqPortA_Hz);
    adrv9001_LoadEightBytes(&offset, armData, switchConfig->minFreqPortB_Hz);
    adrv9001_LoadEightBytes(&offset, armData, switchConfig->maxFreqPortB_Hz);
    armData[offset] = switchConfig->enable;
    
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write,
               device,
               (uint32_t)ADRV9001_ADDR_ARM_MAILBOX_SET,
               &armData[0],
               sizeof(armData),
               ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4);

    extData[0] = 0;
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = ADRV9001_ARM_OBJECTID_RX_PORT_SWITCHING;
    
    ADI_EXPECT(adi_adrv9001_arm_Config_Write, device, armData, sizeof(armData), extData, sizeof(extData))

    ADI_API_RETURN(device);
}



static int32_t __maybe_unused adi_adrv9001_Rx_PortSwitch_Inspect_Validate(adi_adrv9001_Device_t *device,
                                                                          adi_adrv9001_RxPortSwitchCfg_t *switchConfig)
{
    ADI_NULL_PTR_RETURN(&device->common, switchConfig);
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_C0_PortSwitch_Inspect(adi_adrv9001_Device_t *device,
                                              adi_adrv9001_RxPortSwitchCfg_t *switchConfig)
{
    uint8_t armReadBack[33] = { 0 };
    uint8_t channelMask = 0;
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_Rx_PortSwitch_Inspect_Validate, device, switchConfig);

    ADI_EXPECT(adi_adrv9001_arm_Config_Read, device, ADRV9001_ARM_OBJECTID_RX_PORT_SWITCHING, channelMask, offset, armReadBack, sizeof(armReadBack))
    
    adrv9001_ParseEightBytes(&offset, armReadBack, &switchConfig->minFreqPortA_Hz);
    adrv9001_ParseEightBytes(&offset, armReadBack, &switchConfig->maxFreqPortA_Hz);
    adrv9001_ParseEightBytes(&offset, armReadBack, &switchConfig->minFreqPortB_Hz);
    adrv9001_ParseEightBytes(&offset, armReadBack, &switchConfig->maxFreqPortB_Hz);

    switchConfig->enable = (bool) armReadBack[offset];
    ADI_API_RETURN(device);
}

#else

int32_t adi_adrv9001_Rx_C0_PortSwitch_Configure(adi_adrv9001_Device_t *device,
                                                adi_adrv9001_RxPortSwitchCfg_t *switchConfig)
{
    ADI_ERROR_REPORT(device, 
                     ADI_COMMON_ERRSRC_API,
                     ADI_COMMON_ACT_ERR_API_NOT_IMPLEMENTED,
                     ADI_COMMON_ACT_ERR_RESET_FULL,
                     NULL,
                     "Error: attempted to call adi_adrv9001_Rx_PortSwitch_Configure() on non-C0 silicon");
    ADI_API_RETURN(device);
}

int32_t adi_adrv9001_Rx_C0_PortSwitch_Inspect(adi_adrv9001_Device_t *device,
                                              adi_adrv9001_RxPortSwitchCfg_t *switchConfig)
{
    ADI_ERROR_REPORT(device, 
                     ADI_COMMON_ERRSRC_API,
                     ADI_COMMON_ACT_ERR_API_NOT_IMPLEMENTED,
                     ADI_COMMON_ACT_ERR_RESET_FULL,
                     NULL,
                    "Error: attempted to call adi_adrv9001_Rx_PortSwitch_Inspect() on non-C0 silicon");
    ADI_API_RETURN(device);
}

#endif // ADI_ADRV9001_SI_REV_C0
