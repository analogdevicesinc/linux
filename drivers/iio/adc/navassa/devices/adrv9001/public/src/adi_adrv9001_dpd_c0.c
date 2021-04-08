/**
* \file
* \brief Contains DPD features related function implementation
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv9001_dpd.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_radio.h"

#include "adi_adrv9001_arm.h"
#include "adrv9001_arm_macros.h"

#ifdef ADI_ADRV9001_SI_REV_C0

static int32_t __maybe_unused adi_adrv9001_dpd_c0_fh_regions_Configure_Validate(adi_adrv9001_Device_t *adrv9001,
                                                                             adi_common_ChannelNumber_e channel,
                                                                             adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                                                             uint32_t size)
{
    adi_adrv9001_RadioState_t state = { 0 };
    uint8_t port_index = 0;
    uint8_t chan_index = 0;
    uint32_t i = 0;
    
    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdFhRegions);
    ADI_RANGE_CHECK(adrv9001, size, 1, 7);
    ADI_ENTRY_PTR_ARRAY_EXPECT(adrv9001, dpdFhRegions, size);
    for (i = 0; i < size; i++)
    {
        if (dpdFhRegions[i].startFrequency_Hz == 0)
        {
            if (dpdFhRegions[i].endFrequency_Hz != 0)
            {
                ADI_ERROR_REPORT(&adrv9001->common,
                                 ADI_COMMON_ERRSRC_API,
                                 ADI_COMMON_ERR_INV_PARAM,
                                 ADI_COMMON_ACT_ERR_CHECK_PARAM,
                                 channel,
                                 "Start and End frequency are a pair for a region. If start frequency is initialized with 0 then end frequency also must be 0.");		
                ADI_API_RETURN(adrv9001);
            }
        }
        else // non-zero start frequency
            {
                if (dpdFhRegions[i].endFrequency_Hz <= dpdFhRegions[i].startFrequency_Hz)
                {
                    ADI_ERROR_REPORT(&adrv9001->common,
                                     ADI_COMMON_ERRSRC_API,
                                     ADI_COMMON_ERR_INV_PARAM,
                                     ADI_COMMON_ACT_ERR_CHECK_PARAM,
                                     channel,
                                     "End frequency for a region must be greater than the start frequency.");	
                    ADI_API_RETURN(adrv9001);
                }		
            }
    }
    /* Validate state are STANDBY and CALIBRATED */
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, adrv9001, &state);
    adi_common_port_to_index(ADI_TX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if ((ADI_ADRV9001_CHANNEL_PRIMED == state.channelStates[port_index][chan_index]) ||
        (ADI_ADRV9001_CHANNEL_RF_ENABLED == state.channelStates[port_index][chan_index]))
    {
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid channel state. Channel state must be either STANDBY or CALIBRATED");
    }

    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_c0_fh_regions_Configure(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                              uint32_t size)
{
    static const uint8_t OBJID_CFG_DPD_FH_REGIONS = 0xA9;
    uint8_t armData[116] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;
    uint32_t i = 0;
    
    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_c0_fh_regions_Configure_Validate, adrv9001, channel, dpdFhRegions, size);

    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t));
    for (i = 0; i < size; i++)
    {
        adrv9001_LoadEightBytes(&offset, armData, dpdFhRegions[i].startFrequency_Hz);
        adrv9001_LoadEightBytes(&offset, armData, dpdFhRegions[i].endFrequency_Hz);
    }
    
    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_CFG_DPD_FH_REGIONS;
    ADI_EXPECT(adi_adrv9001_arm_Config_Write, adrv9001, armData, sizeof(armData), extData, sizeof(extData))

    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_dpd_c0_fh_regions_Inspect_Validate(adi_adrv9001_Device_t *adrv9001,
                                                                           adi_common_ChannelNumber_e channel,
                                                                           adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                                                           uint32_t size)
{
    adi_adrv9001_RadioState_t state = { 0 };
    uint8_t port_index = 0;
    uint8_t chan_index = 0;
    
    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdFhRegions);
    ADI_RANGE_CHECK(adrv9001, size, 1, 7);
    ADI_ENTRY_PTR_ARRAY_EXPECT(adrv9001, dpdFhRegions, size);
    /* Validate state are STANDBY and CALIBRATED */
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, adrv9001, &state);
    adi_common_port_to_index(ADI_TX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if ((ADI_ADRV9001_CHANNEL_PRIMED == state.channelStates[port_index][chan_index]) ||
        (ADI_ADRV9001_CHANNEL_RF_ENABLED == state.channelStates[port_index][chan_index]))
    {
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid channel state. Channel state must be either STANDBY or CALIBRATED");
    }
    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_c0_fh_regions_Inspect(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                            uint32_t size)
{
    static const uint8_t OBJID_CFG_DPD_FH_REGIONS = 0xA9;
    uint8_t armReadBack[112] = { 0 };
    uint8_t channelMask = 0;
    uint32_t i = 0;
    uint32_t offset = 0;
    uint32_t byteCount = size * 16;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_c0_fh_regions_Inspect_Validate, adrv9001, channel, dpdFhRegions, size);
    channelMask = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    ADI_EXPECT(adi_adrv9001_arm_Config_Read, adrv9001, OBJID_CFG_DPD_FH_REGIONS, channelMask, offset, armReadBack, byteCount)
    for(i = 0 ; i < size ; i++)
    {
        adrv9001_ParseEightBytes(&offset, armReadBack, &(dpdFhRegions[i].startFrequency_Hz));
        adrv9001_ParseEightBytes(&offset, armReadBack, &(dpdFhRegions[i].endFrequency_Hz));
    }

    ADI_API_RETURN(adrv9001);
}

#else

int32_t adi_adrv9001_dpd_c0_fh_regions_Configure(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                              uint32_t size)
{
    ADI_ERROR_REPORT(adrv9001, 
                     ADI_COMMON_ERRSRC_API,
                     ADI_COMMON_ACT_ERR_API_NOT_IMPLEMENTED,
                     ADI_COMMON_ACT_ERR_RESET_FULL,
                     NULL,
                     "Error: attempted to call adi_adrv9001_dpd_fh_regions_Configure() on non-C0 silicon");
    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_c0_fh_regions_Inspect(adi_adrv9001_Device_t *adrv9001,
                                            adi_common_ChannelNumber_e channel,
                                            adi_adrv9001_DpdFhRegions_t dpdFhRegions[],
                                            uint32_t size)
{
    ADI_ERROR_REPORT(adrv9001, 
                     ADI_COMMON_ERRSRC_API,
                     ADI_COMMON_ACT_ERR_API_NOT_IMPLEMENTED,
                     ADI_COMMON_ACT_ERR_RESET_FULL,
                     NULL,
                     "Error: attempted to call adi_adrv9001_dpd_fh_regions_Inspect() on non-C0 silicon");
    ADI_API_RETURN(adrv9001);
}

#endif // ADI_ADRV9001_SI_REV_C0
