/**
 * \file
 * \brief Contains DPD features related function implementation
 *
 * Copyright 2019-2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#include "adi_adrv9001_dpd.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_radio.h"

#include "adi_adrv9001_arm.h"
#include "adrv9001_arm_macros.h"

#ifdef ADI_ADRV9001_SI_REV_B0
#define MAX_PRELUTSCALE 0b111
#else
#define MAX_PRELUTSCALE 0b1111
#endif // ADI_ADRV9001_SI_REV_B0


static int32_t __maybe_unused adi_adrv9001_dpd_Initial_Configure_Validate(adi_adrv9001_Device_t *adrv9001,
                                      adi_common_ChannelNumber_e channel,
                                      adi_adrv9001_DpdInitCfg_t *dpdConfig)
{
    adi_adrv9001_RadioState_t state = { 0 };
    uint8_t port_index = 0;
    uint8_t chan_index = 0;

    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdConfig);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->amplifierType, ADI_ADRV9001_DPDAMPLIFIER_NONE, ADI_ADRV9001_DPDAMPLIFIER_GAN);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->lutSize, ADI_ADRV9001_DPDLUTSIZE_256, ADI_ADRV9001_DPDLUTSIZE_512);
    switch (dpdConfig->model)
    {
    case ADI_ADRV9001_DPDMODEL_0:  /* Falls through */
    case ADI_ADRV9001_DPDMODEL_1:  /* Falls through */
    case ADI_ADRV9001_DPDMODEL_3:  /* Falls through */
    case ADI_ADRV9001_DPDMODEL_4:
        break;
    default:
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         dpdConfig->model,
                         "Invalid parameter value. dpdConfig->model must be a valid adi_adrv9001_DpdModel_e");
        ADI_ERROR_RETURN(adrv9001->common.error.newAction);
    }
    ADI_RANGE_CHECK(adrv9001, dpdConfig->preLutScale, 0, MAX_PRELUTSCALE);

    /* Validate state is STANDBY */
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, adrv9001, &state);
    adi_common_port_to_index(ADI_TX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if (ADI_ADRV9001_CHANNEL_STANDBY != state.channelStates[port_index][chan_index])
    {
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid channel state. Channel must be in STANDBY state");
    }

    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_Initial_Configure(adi_adrv9001_Device_t *adrv9001,
                                           adi_common_ChannelNumber_e channel,
                                           adi_adrv9001_DpdInitCfg_t *dpdConfig)
{
    static const uint8_t OBJID_CFG_DPD_PRE_INIT_CAL = 0xAC;
    uint8_t armData[26] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Initial_Configure_Validate, adrv9001, channel, dpdConfig);

    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t));
    armData[offset++] = dpdConfig->enable;
    armData[offset++] = dpdConfig->amplifierType;
    armData[offset++] = dpdConfig->lutSize;
    armData[offset++] = dpdConfig->model;
    armData[offset++] = dpdConfig->changeModelTapOrders;
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->modelOrdersForEachTap[0]);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->modelOrdersForEachTap[1]);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->modelOrdersForEachTap[2]);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->modelOrdersForEachTap[3]);
    armData[offset++] = dpdConfig->preLutScale;

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_CFG_DPD_PRE_INIT_CAL;

    ADI_EXPECT(adi_adrv9001_arm_Config_Write, adrv9001, armData, sizeof(armData), extData, sizeof(extData))

    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_dpd_Initial_Inspect_Validate(adi_adrv9001_Device_t *adrv9001,
                                    adi_common_ChannelNumber_e channel,
                                    adi_adrv9001_DpdInitCfg_t *dpdConfig)
{
    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdConfig);

    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_Initial_Inspect(adi_adrv9001_Device_t *adrv9001,
                                         adi_common_ChannelNumber_e channel,
                                         adi_adrv9001_DpdInitCfg_t *dpdConfig)
{
    static const uint8_t OBJID_CFG_DPD_PRE_INIT_CAL = 0xAC;
    uint8_t armReadBack[22] = { 0 };
    uint8_t channelMask = 0;
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Initial_Inspect_Validate, adrv9001, channel, dpdConfig);

    channelMask = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    ADI_EXPECT(adi_adrv9001_arm_Config_Read, adrv9001, OBJID_CFG_DPD_PRE_INIT_CAL, channelMask, offset, armReadBack, sizeof(armReadBack))

    dpdConfig->enable = (bool)armReadBack[offset++];
    dpdConfig->amplifierType = armReadBack[offset++];
    dpdConfig->lutSize = armReadBack[offset++];
    dpdConfig->model = armReadBack[offset++];
    dpdConfig->changeModelTapOrders = (bool)armReadBack[offset++];
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->modelOrdersForEachTap[0]);
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->modelOrdersForEachTap[1]);
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->modelOrdersForEachTap[2]);
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->modelOrdersForEachTap[3]);
    dpdConfig->preLutScale = armReadBack[offset++];

    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_dpd_Configure_Validate(adi_adrv9001_Device_t *adrv9001,
                                                                  adi_common_ChannelNumber_e channel,
                                                                  adi_adrv9001_DpdCfg_t *dpdConfig)
{
    static const uint32_t DPD_MAX_SAMPLES = 4096;
    static const uint32_t MAX_RX_TX_NORMALIZATION_THRESHOLD_U2D30 = 1 << 30;    // 1.0 in U2.30
    adi_adrv9001_RadioState_t state = { 0 };
    uint8_t port_index = 0;
    uint8_t chan_index = 0;

    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdConfig);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->numberOfSamples, 3, DPD_MAX_SAMPLES);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->rxTxNormalizationLowerThreshold, 0, MAX_RX_TX_NORMALIZATION_THRESHOLD_U2D30);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->rxTxNormalizationUpperThreshold, 0, MAX_RX_TX_NORMALIZATION_THRESHOLD_U2D30);
#ifdef ADI_ADRV9001_SI_REV_C0
    ADI_RANGE_CHECK(adrv9001, dpdConfig->countsLessThanPowerThreshold, 0, 4096);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->countsGreaterThanPeakThreshold, 0, 4096);
#endif // ADI_ADRV9001_SI_REV_C0
    
    /* Validate state is CALIBRATED */
    ADI_EXPECT(adi_adrv9001_Radio_State_Get, adrv9001, &state);
    adi_common_port_to_index(ADI_TX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if (ADI_ADRV9001_CHANNEL_CALIBRATED != state.channelStates[port_index][chan_index])
    {
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid channel state. Channel must be in CALIBRATED state");
    }

    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_Configure(adi_adrv9001_Device_t *adrv9001,
                                   adi_common_ChannelNumber_e channel,
                                   adi_adrv9001_DpdCfg_t *dpdConfig)
{
    static const uint8_t OBJID_TC_TX_DPD = 0x44;
#ifdef ADI_ADRV9001_SI_REV_B0
    uint8_t armData[31] = { 0 };
#else
    uint8_t armData[35] = { 0 };
#endif // ADI_ADRV9001_SI_REV_B0

    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Configure_Validate, adrv9001, channel, dpdConfig);

    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t));
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->numberOfSamples);
    
#ifdef ADI_ADRV9001_SI_REV_B0
    offset += 2; /* Placeholder for future use */
    offset += 1; /* Placeholder for future use since preLutScale was removed */
    armData[offset++] = dpdConfig->outlierRemovalEnable;
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->outlierRemovalThreshold);
#endif // ADI_ADRV9001_SI_REV_B0
    
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->additionalPowerScale);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->rxTxNormalizationLowerThreshold);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->rxTxNormalizationUpperThreshold);
    
#ifdef ADI_ADRV9001_SI_REV_C0
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->detectionPowerThreshold);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->detectionPeakThreshold);
    adrv9001_LoadTwoBytes( &offset, armData, dpdConfig->countsLessThanPowerThreshold);
    adrv9001_LoadTwoBytes( &offset, armData, dpdConfig->countsGreaterThanPeakThreshold);
#endif // ADI_ADRV9001_SI_REV_C0
    
    armData[offset++] = dpdConfig->immediateLutSwitching;
    armData[offset++] = dpdConfig->useSpecialFrame;
    armData[offset++] = dpdConfig->resetLuts;

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_TC_TX_DPD;

    ADI_EXPECT(adi_adrv9001_arm_Config_Write, adrv9001, armData, sizeof(armData), extData, sizeof(extData))

    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_dpd_Inspect_Validate(adi_adrv9001_Device_t *adrv9001,
                                adi_common_ChannelNumber_e channel,
                                adi_adrv9001_DpdCfg_t *dpdConfig)
{
    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdConfig);

    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_Inspect(adi_adrv9001_Device_t *adrv9001,
                                 adi_common_ChannelNumber_e channel,
                                 adi_adrv9001_DpdCfg_t *dpdConfig)
{
    static const uint8_t OBJID_TC_TX_DPD = 0x44;
#ifdef ADI_ADRV9001_SI_REV_B0
    uint8_t armReadBack[27] = { 0 };
#else
    uint8_t armReadBack[31] = { 0 };
#endif // ADI_ADRV9001_SI_REV_B0

    uint8_t channelMask = 0;
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Inspect_Validate, adrv9001, channel, dpdConfig);

    channelMask = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);

    ADI_EXPECT(adi_adrv9001_arm_Config_Read, adrv9001, OBJID_TC_TX_DPD, channelMask, offset, armReadBack, sizeof(armReadBack))

    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->numberOfSamples);
#ifdef ADI_ADRV9001_SI_REV_B0
    offset += 2; /* Placeholder for future use */
    offset += 1; /* Placeholder for future use since preLutScale was removed */

    dpdConfig->outlierRemovalEnable = armReadBack[offset++];
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->outlierRemovalThreshold);
#endif // ADI_ADRV9001_SI_REV_B0
    
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->additionalPowerScale);
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->rxTxNormalizationLowerThreshold);
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->rxTxNormalizationUpperThreshold);
    
#ifdef ADI_ADRV9001_SI_REV_C0
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->detectionPowerThreshold);
    adrv9001_ParseFourBytes(&offset, armReadBack, &dpdConfig->detectionPeakThreshold);
    adrv9001_ParseTwoBytes( &offset, armReadBack, &dpdConfig->countsLessThanPowerThreshold);
    adrv9001_ParseTwoBytes( &offset, armReadBack, &dpdConfig->countsGreaterThanPeakThreshold);
#endif // ADI_ADRV9001_SI_REV_C0

    dpdConfig->immediateLutSwitching    = (bool)armReadBack[offset++];
    dpdConfig->useSpecialFrame          = (bool)armReadBack[offset++];
    dpdConfig->resetLuts                = (bool)armReadBack[offset++];

    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_dpd_coefficients_Set_Validate(adi_adrv9001_Device_t *adrv9001,
                                                                         adi_common_ChannelNumber_e channel,
                                                                         adi_adrv9001_DpdCoefficients_t *coefficients)
{
    adi_adrv9001_ChannelState_e state = ADI_ADRV9001_CHANNEL_STANDBY;
    uint8_t port_index = 0;
    uint8_t chan_index = 0;
    
    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&adrv9001->common, coefficients);
    ADI_RANGE_CHECK(adrv9001, coefficients->region, 0, 7);
    
    ADI_EXPECT(adi_adrv9001_Radio_Channel_State_Get, adrv9001, ADI_TX, channel, &state);
    adi_common_port_to_index(ADI_TX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if (ADI_ADRV9001_CHANNEL_CALIBRATED != state)
    {
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid channel state. Channel must be in CALIBRATED state");
    }
    
    ADI_API_RETURN(adrv9001);
}

static const uint8_t OBJID_DPD_LUT_INITIALIZE = 0xA8;

int32_t adi_adrv9001_dpd_coefficients_Set(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_DpdCoefficients_t *coefficients)
{
    uint8_t armData[216] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;
    uint8_t i = 0;
    
    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_coefficients_Set_Validate, adrv9001, channel, coefficients);
    
    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t));
    armData[offset++] = coefficients->region;
    offset += 3;    // Reserved
    for(i = 0 ; i < ADI_ADRV9001_DPD_NUM_COEFFICIENTS; i++)
    {
        armData[offset++] = coefficients->coefficients[i];
    }
    
    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_DPD_LUT_INITIALIZE;
    
    ADI_EXPECT(adi_adrv9001_arm_Config_Write, adrv9001, armData, sizeof(armData), extData, sizeof(extData))
    
    ADI_API_RETURN(adrv9001);
}

static int32_t __maybe_unused adi_adrv9001_dpd_coefficients_Get_Validate(adi_adrv9001_Device_t *adrv9001,
                                                                         adi_common_ChannelNumber_e channel,
                                                                         adi_adrv9001_DpdCoefficients_t *coefficients)
{
    adi_adrv9001_ChannelState_e state = ADI_ADRV9001_CHANNEL_STANDBY;
    uint8_t port_index = 0;
    uint8_t chan_index = 0;
    
    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);
    ADI_NULL_PTR_RETURN(&adrv9001->common, coefficients);
#ifdef ADI_ADRV9001_SI_REV_C0
    ADI_RANGE_CHECK(adrv9001, coefficients->region, 0, 7);
#endif // ADI_ADRV9001_SI_REV_C0
 
    ADI_EXPECT(adi_adrv9001_Radio_Channel_State_Get, adrv9001, ADI_TX, channel, &state);
    adi_common_port_to_index(ADI_TX, &port_index);
    adi_common_channel_to_index(channel, &chan_index);
    if (ADI_ADRV9001_CHANNEL_CALIBRATED != state)
    {
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         channel,
                         "Invalid channel state. Channel must be in CALIBRATED state");
    }
    
    ADI_API_RETURN(adrv9001);
}

int32_t adi_adrv9001_dpd_coefficients_Get(adi_adrv9001_Device_t *adrv9001,
                                          adi_common_ChannelNumber_e channel,
                                          adi_adrv9001_DpdCoefficients_t *coefficients)
{
    uint8_t armReadBack[212] = { 0 };
    uint8_t channelMask = 0;
    uint32_t offset = 0;
    uint8_t i = 0;
    
    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_coefficients_Get_Validate, adrv9001, channel, coefficients);

#ifdef ADI_ADRV9001_SI_REV_C0
    /* Write region to GET buffer */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_GET + 4, &coefficients->region, 1, ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4);
#endif // ADI_ADRV9001_SI_REV_C0
    
    channelMask = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    ADI_EXPECT(adi_adrv9001_arm_Config_Read, adrv9001, OBJID_DPD_LUT_INITIALIZE, channelMask, 0, armReadBack, sizeof(armReadBack));
    
    coefficients->region = armReadBack[offset++];
    offset += 3;    // Reserved
    for(i = 0 ; i < ADI_ADRV9001_DPD_NUM_COEFFICIENTS ; i++)
    {
        coefficients->coefficients[i] = armReadBack[offset++];
    }
    
    ADI_API_RETURN(adrv9001);
}
