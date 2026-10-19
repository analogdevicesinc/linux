/*!
 * \brief     Tx Misc functional block API implementation
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_TXMISC
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_txmisc.h"
#include "adi_apollo_txmisc_local.h"

#include "adi_apollo_bf_core.h"
#include "adi_apollo_bf_tx_misc.h"

#include "adi_apollo_cduc.h"
#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"
#include "adi_utils.h"

static int32_t tx_summer_inspect(adi_apollo_device_t* device, uint32_t side_idx, adi_apollo_txpath_misc_t* dp);
static int32_t tx_cduc_dac_inspect(adi_apollo_device_t* device, uint32_t side_idx, adi_apollo_txpath_misc_t* dp);
static int32_t tx_mux_misc_inspect(adi_apollo_device_t* device, uint32_t side_idx, adi_apollo_txpath_misc_t* dp);

/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_txmisc_low_samp_set(adi_apollo_device_t *device, adi_apollo_side_select_e side_sel, uint8_t low_samp_en)
{
    int32_t err;
    uint8_t side;
    uint32_t side_index;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(side_sel, ADI_APOLLO_SIDE_NONE, ADI_APOLLO_SIDE_ALL);

    for (side_index = 0;  side_index < ADI_APOLLO_NUM_SIDES; side_index++) {
        side = (uint8_t)(side_sel & ((uint32_t)ADI_APOLLO_SIDE_A << side_index));
        if (side > 0U) {
            regmap_base_addr = calc_tx_misc_base(side_index);

            err = adi_apollo_hal_bf_set(device, BF_LOW_SAMP_TX_MISC_INFO(regmap_base_addr), low_samp_en);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_txmisc_dp_reset(adi_apollo_device_t *device, adi_apollo_side_select_e side_sel, uint8_t reset_en)
{
    int32_t err;
    uint8_t side;
    uint32_t side_index;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(side_sel, ADI_APOLLO_SIDE_NONE, ADI_APOLLO_SIDE_ALL);

    for (side_index = 0;  side_index < ADI_APOLLO_NUM_SIDES; side_index++) {
        side = (uint8_t)(side_sel & ((uint32_t)ADI_APOLLO_SIDE_A << side_index));
        if (side > 0U) {
            if (side_index == 0U) {
                err = adi_apollo_hal_bf_set(device, BF_TX_DP_RESET_A_INFO, reset_en);
            } else {
                err = adi_apollo_hal_bf_set(device, BF_TX_DP_RESET_B_INFO, reset_en);
            }
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_txmisc_cduc_dac_enable_set(adi_apollo_device_t *device, const uint16_t cducs, const uint8_t enable)
{
    int32_t err;
    uint32_t i;
    uint16_t cducs_sel;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    for (i = 0U; i < ADI_APOLLO_CDUC_NUM; i++) {
        /* Select one CDUC at a time */
        cducs_sel = (uint16_t)(cducs & ((uint32_t)ADI_APOLLO_CDUC_A0 << i));
        if (cducs_sel > 0U) {
            /* Base address is side-dependent */
            regmap_base_addr = calc_tx_misc_base(i / ADI_APOLLO_CDUC_PATHS_PER_SIDE);

            /* Program the CDUC_DAC_ENABLESx fields */
            /* Note: Hardware register bits are jumbled - A0,A2,A1,A3 order due to physical DAC layout */
            switch (i % ADI_APOLLO_CDUC_PATHS_PER_SIDE)
            {
            case 0:
                err = adi_apollo_hal_bf_set(device, BF_CDUC_DAC_ENABLES0_INFO(regmap_base_addr), enable);
                break;
            case 1:
                /* CDUC_A1 control is in bits [5:4], not [3:2] */
                err = adi_apollo_hal_bf_set(device, BF_CDUC_DAC_ENABLES2_INFO(regmap_base_addr), enable);
                break;
            case 2:
                /* CDUC_A2 control is in bits [3:2], not [5:4] */
                err = adi_apollo_hal_bf_set(device, BF_CDUC_DAC_ENABLES1_INFO(regmap_base_addr), enable);
                break;
            case 3:
                err = adi_apollo_hal_bf_set(device, BF_CDUC_DAC_ENABLES3_INFO(regmap_base_addr), enable);
                break;
            default:
                /* Should not reach here - i % 4 is always 0-3 */
                err = (int32_t)API_CMS_ERROR_INVALID_PARAM;
                break;
            }
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


int32_t adi_apollo_txmisc_inspect(adi_apollo_device_t *device, uint16_t side_sel, adi_apollo_txmisc_inspect_t *txmisc_inspect)
{
    int32_t err;
    uint32_t side;
    uint32_t side_index;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(txmisc_inspect);
    ADI_APOLLO_INVALID_PARAM_RETURN(adi_api_utils_num_selected(side_sel) != 1U);

    for (side_index = 0;  side_index < ADI_APOLLO_NUM_SIDES; side_index++) {
        uint32_t side_mask = (uint32_t)ADI_APOLLO_SIDE_A << side_index;
        side = side_sel & side_mask;
        if (side > 0U) {
            err = tx_summer_inspect(device, side_index, &txmisc_inspect->dp);
            ADI_APOLLO_ERROR_RETURN(err);
            
            err = tx_cduc_dac_inspect(device, side_index, &txmisc_inspect->dp);
            ADI_APOLLO_ERROR_RETURN(err);

            err = tx_mux_misc_inspect(device, side_index, &txmisc_inspect->dp);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Inspect only one side per call */
            break;
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t tx_summer_inspect(adi_apollo_device_t* device, uint32_t side_idx, adi_apollo_txpath_misc_t *dp)
{
    int32_t err;
    uint8_t fduc_enables[4];

    uint32_t regmap_base_addr = calc_tx_misc_base(side_idx);

    err = adi_apollo_hal_reg_get(device, REG_FDUC_ENABLES00_ADDR(regmap_base_addr), &fduc_enables[0]);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_reg_get(device, REG_FDUC_ENABLES10_ADDR(regmap_base_addr), &fduc_enables[2]);
    ADI_APOLLO_ERROR_RETURN(err);


    if (device->dev_info.is_8t8r != 0U) {
        /* Summer A0 */
        dp->fduc_cduc_summer[0] = (uint8_t)(
            ((fduc_enables[0] & 0x01U) >> 0U) | ((fduc_enables[0] & 0x04U) >> 1U) | ((fduc_enables[0] & 0x10U) >> 2U) | ((fduc_enables[0] & 0x40U) >> 3U) |
            ((fduc_enables[0] & 0x02U) << 3U) | ((fduc_enables[0] & 0x08U) >> 2U) | ((fduc_enables[0] & 0x20U) >> 1U) | ((fduc_enables[0] & 0x80U) >> 0U));
        /* Summer A1 */
        dp->fduc_cduc_summer[1] = (uint8_t)(
            ((fduc_enables[2] & 0x01U) >> 0U) | ((fduc_enables[2] & 0x04U) >> 1U) | ((fduc_enables[2] & 0x10U) >> 2U) | ((fduc_enables[2] & 0x40U) >> 3U) |
            ((fduc_enables[2] & 0x02U) << 3U) | ((fduc_enables[2] & 0x08U) >> 2U) | ((fduc_enables[2] & 0x20U) >> 1U) | ((fduc_enables[2] & 0x80U) >> 0U));
    } else {
        /* Summer A0 */
        dp->fduc_cduc_summer[0] = fduc_enables[0] & 0x0fU;

        /* Summer A1 */
        dp->fduc_cduc_summer[1] = fduc_enables[2] & 0x0fU;
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t tx_cduc_dac_inspect(adi_apollo_device_t* device, uint32_t side_idx, adi_apollo_txpath_misc_t *dp)
{
    int32_t err;
    uint32_t regmap_base_addr = calc_tx_misc_base(side_idx);
    
    /* Note: Hardware register bits are jumbled - A0,A2,A1,A3 order due to physical DAC layout */
    err = adi_apollo_hal_bf_get(device, BF_CDUC_DAC_ENABLES0_INFO(regmap_base_addr), &dp->cduc_dac_enables[0], 1);
    ADI_APOLLO_ERROR_RETURN(err);
    /* CDUC_A1 control is in bits [5:4], not [3:2] */
    err = adi_apollo_hal_bf_get(device, BF_CDUC_DAC_ENABLES2_INFO(regmap_base_addr), &dp->cduc_dac_enables[1], 1);
    ADI_APOLLO_ERROR_RETURN(err);
    /* CDUC_A2 control is in bits [3:2], not [5:4] */
    err = adi_apollo_hal_bf_get(device, BF_CDUC_DAC_ENABLES1_INFO(regmap_base_addr), &dp->cduc_dac_enables[2], 1);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_get(device, BF_CDUC_DAC_ENABLES3_INFO(regmap_base_addr), &dp->cduc_dac_enables[3], 1); 
    
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t tx_mux_misc_inspect(adi_apollo_device_t* device, uint32_t side_idx, adi_apollo_txpath_misc_t *dp)
{
    int32_t err;
    uint8_t i;
    uint8_t xbar = 0;
    uint8_t low_samp = 0;
    uint8_t modsw0 = 0;
    uint8_t modsw1 = 0;
    uint32_t regmap_base_addr = calc_tx_misc_base(side_idx);

    err = adi_apollo_hal_bf_get(device, BF_HS_XBAR_CTRL_INFO(regmap_base_addr), &xbar, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    for (i = 0; i < ADI_APOLLO_CDUC_PATHS_PER_SIDE; i++) {
        dp->mux0_sel[i] = (adi_apollo_tx_mux0_sel_e)(((uint32_t)xbar >> (2U * i)) & 0x03U);
    }

    err = adi_apollo_hal_bf_get(device, BF_MODSW0_INFO(regmap_base_addr), &modsw0, 1);
    ADI_APOLLO_ERROR_RETURN(err);
    dp->mux1_sel[0] = (adi_apollo_tx_mux1_sel_e)modsw0;

    if (device->dev_info.is_8t8r != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_MODSW1_INFO(regmap_base_addr), &modsw1, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        dp->mux1_sel[1] = (adi_apollo_tx_mux1_sel_e)modsw1;
    } else {
        dp->mux1_sel[1] = (adi_apollo_tx_mux1_sel_e)0U;
    }

    err = adi_apollo_hal_bf_get(device, BF_LOW_SAMP_TX_MISC_INFO(regmap_base_addr), &low_samp, 1);
    ADI_APOLLO_ERROR_RETURN(err);
    dp->low_samp_en = (low_samp != 0U);

    return API_CMS_ERROR_OK;
}

uint32_t calc_tx_misc_base(uint32_t idx)
{
    static const uint32_t tx_misc_regmap[ADI_APOLLO_TX_MISC_REGMAP_NUM] = {
        TX_MISC_TX_TOP_TX_DIGITAL0,

        TX_MISC_TX_TOP_TX_DIGITAL1
    };

    return tx_misc_regmap[idx];
}
/*! @} */