/*!
 * \brief     Rx Crossbar Mux functional block API implementation
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_RXMUX
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_rxmux.h"
#include "adi_apollo_rxmux_local.h"
#include "adi_apollo_rxmisc_local.h"

#include "adi_apollo_jtx.h"

#include "adi_apollo_bf_rx_misc.h"
#include "adi_apollo_bf_rx_datin.h"
#include "adi_apollo_bf_jtx_dual_link.h"

#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"

/* Forward declaration for static helper function */
static uint32_t calc_rx_datin_base_from_mux_index(uint32_t adc_mux_index);

/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_rxmux_xbar1_set(adi_apollo_device_t *device,  adi_apollo_side_select_e side_sel, adi_apollo_rx_mux0_sel_e cbout_from_adc_sel[], uint32_t length)
{
    int32_t err;
    uint8_t cb_out, side, side_index, bf_val;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(cbout_from_adc_sel);

    if (device->dev_info.is_8t8r != 0U) {
        ADI_APOLLO_INVALID_PARAM_RETURN(length != ADI_APOLLO_RX_MUX0_NUM_8T8R);
    } else {
        ADI_APOLLO_INVALID_PARAM_RETURN(length != ADI_APOLLO_RX_MUX0_NUM_4T4R);
    }

    for (side_index = 0; side_index < ADI_APOLLO_NUM_SIDES; side_index++) {
        side = (uint8_t)(side_sel & ((uint32_t)ADI_APOLLO_SIDE_A << side_index));
        if (side > 0U) {

            for (cb_out = 0; cb_out < length; cb_out++) {
                regmap_base_addr = calc_rx_datin_base_from_mux_index((uint32_t)((cb_out % 2U) + (2U*side_index)));

                bf_val = (uint8_t)cbout_from_adc_sel[cb_out];

                if (cb_out < 2U) {
                    err = adi_apollo_hal_bf_set(device, BF_ADC_0_MUX_SEL_INFO(regmap_base_addr), bf_val);
                    ADI_APOLLO_ERROR_RETURN(err);
                } else {
                    err = adi_apollo_hal_bf_set(device, BF_ADC_1_MUX_SEL_INFO(regmap_base_addr), bf_val);
                    ADI_APOLLO_ERROR_RETURN(err);
                }
            }

        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxmux_xbar2_set(adi_apollo_device_t *device,  adi_apollo_side_select_e side_sel, adi_apollo_rx_mux2_sel_e coarse_to_fine_xbar_sel[], uint32_t length)
{
    int32_t err;
    uint8_t side, side_index;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(coarse_to_fine_xbar_sel);
    ADI_APOLLO_INVALID_PARAM_RETURN(length != ADI_APOLLO_RX_MUX2_NUM);

    for (side_index = 0; side_index < ADI_APOLLO_NUM_SIDES; side_index++) {
        side = (uint8_t)(side_sel & ((uint32_t)ADI_APOLLO_SIDE_A << side_index));
        if (side > 0U) {

            regmap_base_addr = calc_rx_misc_base(side_index);

            err = adi_apollo_hal_bf_set(device, BF_CB_SEL_F0_INFO(regmap_base_addr), (uint64_t)coarse_to_fine_xbar_sel[0]);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_CB_SEL_F1_INFO(regmap_base_addr), (uint64_t)coarse_to_fine_xbar_sel[1]);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_CB_SEL_F2_INFO(regmap_base_addr), (uint64_t)coarse_to_fine_xbar_sel[2]);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_CB_SEL_F3_INFO(regmap_base_addr), (uint64_t)coarse_to_fine_xbar_sel[3]);
            ADI_APOLLO_ERROR_RETURN(err);

        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxmux_sample_xbar_set(adi_apollo_device_t *device, uint16_t link_sel, uint8_t sample_offset, adi_apollo_rxmux_sample_sel_e sample_sel[], uint32_t length)
{
    int32_t err;
    uint32_t regmap_base_addr = 0;
    uint8_t i, j;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(sample_sel);
    ADI_APOLLO_INVALID_PARAM_RETURN((sample_offset + length) > ADI_APOLLO_JESD_MAX_SAMPLE_XBAR_IDXS);

    /* Validate all sample_sel array elements before any hardware writes */
    for (j = 0; j < length; j++) {
        ADI_CMS_RANGE_CHECK(sample_sel[j], ADI_APOLLO_RXMUX_SAMPLE_FINE0_I, ADI_APOLLO_RXMUX_SAMPLE_FINE7_Q);
    }

    for (i = 0; i < ADI_APOLLO_NUM_JTX_LINKS; i++) {
        if (((1U << i) & link_sel) != 0U) {
            regmap_base_addr = calc_jtx_dual_link_base(i);
            for (j = 0; j < length; j++) {
                uint32_t conv_offset = (uint32_t)sample_offset + j;
                err = adi_apollo_hal_bf_set(device, BF_JTX_CONV_SEL_INFO(regmap_base_addr, conv_offset), (uint64_t)sample_sel[j]);
                ADI_APOLLO_ERROR_RETURN(err);
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static uint32_t calc_rx_datin_base_from_mux_index(uint32_t adc_mux_index)
{
    static uint32_t rx_datin_regmap[] = {
        RX_DATIN_RX_SLICE_0_RX_DIGITAL0, RX_DATIN_RX_SLICE_1_RX_DIGITAL0,
        RX_DATIN_RX_SLICE_0_RX_DIGITAL1, RX_DATIN_RX_SLICE_1_RX_DIGITAL1
    };
    return rx_datin_regmap[adc_mux_index];
}
/*! @} */