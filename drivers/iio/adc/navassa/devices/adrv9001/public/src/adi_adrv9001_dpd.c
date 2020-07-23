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

static int32_t adi_adrv9001_dpd_Initial_Configure_Validate(adi_adrv9001_Device_t *adrv9001,
                                                           adi_common_ChannelNumber_e channel,
                                                           adi_adrv9001_DpdInitCfg_t *dpdConfig)
{
    adi_adrv9001_RadioState_t state = { 0 };
    uint8_t port_index = 0;
    uint8_t chan_index = 0;
    static const uint8_t MAX_PRELUTSCALE_U2D1 = 0b111;    // 3.5 in U2.1

    ADI_RANGE_CHECK(adrv9001, channel, ADI_CHANNEL_1, ADI_CHANNEL_2);

    ADI_NULL_PTR_RETURN(&adrv9001->common, dpdConfig);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->amplifierType, ADI_ADRV9001_DPD_AMPLIFIER_NONE, ADI_ADRV9001_DPD_AMPLIFIER_GAN);
    ADI_RANGE_CHECK(adrv9001, dpdConfig->lutSize, ADI_ADRV9001_DPD_LUT_SIZE_256, ADI_ADRV9001_DPD_LUT_SIZE_512);
    switch (dpdConfig->model)
    {
    case ADI_ADRV9001_DPD_MODEL_0:  /* Falls through */
    case ADI_ADRV9001_DPD_MODEL_1:  /* Falls through */
    case ADI_ADRV9001_DPD_MODEL_3:  /* Falls through */
    case ADI_ADRV9001_DPD_MODEL_4:
        break;
    default:
        ADI_ERROR_REPORT(&adrv9001->common,
                         ADI_COMMON_ERRSRC_API,
                         ADI_COMMON_ERR_INV_PARAM,
                         ADI_COMMON_ACT_ERR_CHECK_PARAM,
                         dpdConfig->model,
                         "Invalid parameter value. dpdConfig->model must be a valid adi_adrv9001_DpdModel_e");
    }
    ADI_RANGE_CHECK(adrv9001, dpdConfig->preLutScale, 0, MAX_PRELUTSCALE_U2D1);

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

    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_SET, &armData[0], sizeof(armData));

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_CFG_DPD_PRE_INIT_CAL;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, adrv9001, ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(adrv9001,
                                        ADRV9001_ARM_SET_OPCODE,
                                        extData[1],
                                        ADI_ADRV9001_DEFAULT_TIMEOUT_US,
                                        ADI_ADRV9001_DEFAULT_INTERVAL_US);

    ADI_API_RETURN(adrv9001);
}

static int32_t adi_adrv9001_dpd_Initial_Inspect_Validate(adi_adrv9001_Device_t *adrv9001,
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
    uint8_t armData[22] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Initial_Inspect_Validate, adrv9001, channel, dpdConfig);

    /* Tell the ARM how much data to read */
    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData));
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_GET, &armData[0], sizeof(uint32_t));
    offset = 0;

    /* Invoke the GET command */
    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_CFG_DPD_PRE_INIT_CAL;
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, adrv9001, ADRV9001_ARM_GET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(adrv9001,
                                        ADRV9001_ARM_GET_OPCODE,
                                        extData[1],
                                        ADI_ADRV9001_DEFAULT_TIMEOUT_US,
                                        ADI_ADRV9001_DEFAULT_INTERVAL_US);

    /* Read and parse the data */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_GET, armData, sizeof(armData), false);
    dpdConfig->enable = (bool)armData[offset++];
    dpdConfig->amplifierType = armData[offset++];
    dpdConfig->lutSize = armData[offset++];
    dpdConfig->model = armData[offset++];
    dpdConfig->changeModelTapOrders = (bool)armData[offset++];
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->modelOrdersForEachTap[0]);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->modelOrdersForEachTap[1]);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->modelOrdersForEachTap[2]);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->modelOrdersForEachTap[3]);
    dpdConfig->preLutScale = armData[offset++];

    ADI_API_RETURN(adrv9001);
}

static int32_t adi_adrv9001_dpd_Configure_Validate(adi_adrv9001_Device_t *adrv9001,
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
    uint8_t armData[30] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Configure_Validate, adrv9001, channel, dpdConfig);

    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData) - sizeof(uint32_t));
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->numberOfSamples);

    //adrv9001_LoadTwoBytes(&offset, armData, dpdConfig->additionalDelayOffset);
    offset += 2; /* Placeholder for future use */

    offset += 1; /* Placeholder for future use since preLutScale was removed */
    armData[offset++] = dpdConfig->outlierRemovalEnable;
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->outlierRemovalThreshold);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->additionalPowerScale);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->rxTxNormalizationLowerThreshold);
    adrv9001_LoadFourBytes(&offset, armData, dpdConfig->rxTxNormalizationUpperThreshold);
    armData[offset++] = dpdConfig->immediateLutSwitching;
    armData[offset++] = dpdConfig->useSpecialFrame;
    armData[offset++] = dpdConfig->resetLuts;

    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_SET, &armData[0], sizeof(armData));

    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_TC_TX_DPD;

    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, adrv9001, ADRV9001_ARM_SET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(adrv9001,
                                        ADRV9001_ARM_SET_OPCODE,
                                        extData[1],
                                        ADI_ADRV9001_DEFAULT_TIMEOUT_US,
                                        ADI_ADRV9001_DEFAULT_INTERVAL_US);

    ADI_API_RETURN(adrv9001);
}

static int32_t adi_adrv9001_dpd_Inspect_Validate(adi_adrv9001_Device_t *adrv9001,
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
    uint8_t armData[26] = { 0 };
    uint8_t extData[5] = { 0 };
    uint32_t offset = 0;

    ADI_PERFORM_VALIDATION(adi_adrv9001_dpd_Inspect_Validate, adrv9001, channel, dpdConfig);

    /* Tell the ARM how much data to read */
    adrv9001_LoadFourBytes(&offset, armData, sizeof(armData));
    ADI_EXPECT(adi_adrv9001_arm_Memory_Write, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_GET, &armData[0], sizeof(uint32_t));
    offset = 0;

    /* Invoke the GET command */
    extData[0] = adi_adrv9001_Radio_MailboxChannel_Get(ADI_TX, channel);
    extData[1] = ADRV9001_ARM_OBJECTID_CONFIG;
    extData[2] = OBJID_TC_TX_DPD;
    ADI_EXPECT(adi_adrv9001_arm_Cmd_Write, adrv9001, ADRV9001_ARM_GET_OPCODE, &extData[0], sizeof(extData));

    /* Wait for command to finish executing */
    ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(adrv9001,
                                        ADRV9001_ARM_GET_OPCODE,
                                        extData[1],
                                        ADI_ADRV9001_DEFAULT_TIMEOUT_US,
                                        ADI_ADRV9001_DEFAULT_INTERVAL_US);

    /* Read and parse the data */
    ADI_EXPECT(adi_adrv9001_arm_Memory_Read, adrv9001, ADRV9001_ADDR_ARM_MAILBOX_GET, armData, sizeof(armData), false);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->numberOfSamples);

    //adrv9001_ParseTwoBytes(&offset, armData, (uint16_t *)&dpdConfig->additionalDelayOffset);
    offset += 2; /* Placeholder for future use */
    offset += 1; /* Placeholder for future use since preLutScale was removed */

    dpdConfig->outlierRemovalEnable = armData[offset++];
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->outlierRemovalThreshold);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->additionalPowerScale);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->rxTxNormalizationLowerThreshold);
    adrv9001_ParseFourBytes(&offset, armData, &dpdConfig->rxTxNormalizationUpperThreshold);
    dpdConfig->immediateLutSwitching = (bool)armData[offset++];
    dpdConfig->useSpecialFrame = (bool)armData[offset++];

    ADI_API_RETURN(adrv9001);
}
