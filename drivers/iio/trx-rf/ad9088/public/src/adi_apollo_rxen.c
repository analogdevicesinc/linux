/*!
 * \brief     APIs for Rx power control
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_RXEN
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_rxen.h"
#include "adi_apollo_rxen_local.h"
#include "adi_apollo_pfilt.h"
#include "adi_apollo_cddc.h"
#include "adi_apollo_fddc.h"
#include "adi_apollo_private_blk_sel_types.h"

#include "adi_apollo_bf_conv_pow_onoff.h"
#include "adi_apollo_bf_txen_power_ctrl.h"
#include "adi_apollo_bf_txrx_enable.h"

#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"

/* Forward declarations for static helper functions */
static uint32_t calc_txen_power_ctrl_base(uint32_t adc_index);
static uint32_t rxen_id_to_rxen_power_ctrl_base_A(uint32_t rxen_id);
static uint32_t rxen_id_to_rxen_power_ctrl_base_B(uint32_t rxen_id);
static uint32_t calc_rx_enable_base(uint32_t index);

static adi_apollo_rxtx_enable_e conv_pow_output_to_rxtx_enable(adi_apollo_conv_pow_output_e output)
{
    switch (output) {
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN0: return ADI_APOLLO_RXTX_ENABLE_RX0;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN1: return ADI_APOLLO_RXTX_ENABLE_RX1;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN2: return ADI_APOLLO_RXTX_ENABLE_RX2;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN3: return ADI_APOLLO_RXTX_ENABLE_RX3;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN0: return ADI_APOLLO_RXTX_ENABLE_RX0;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN1: return ADI_APOLLO_RXTX_ENABLE_RX1;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN2: return ADI_APOLLO_RXTX_ENABLE_RX2;
    case ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN3: return ADI_APOLLO_RXTX_ENABLE_RX3;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN0: return ADI_APOLLO_RXTX_ENABLE_TX0;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN1: return ADI_APOLLO_RXTX_ENABLE_TX1;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN2: return ADI_APOLLO_RXTX_ENABLE_TX2;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN3: return ADI_APOLLO_RXTX_ENABLE_TX3;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN0: return ADI_APOLLO_RXTX_ENABLE_TX0;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN1: return ADI_APOLLO_RXTX_ENABLE_TX1;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN2: return ADI_APOLLO_RXTX_ENABLE_TX2;
    case ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN3: return ADI_APOLLO_RXTX_ENABLE_TX3;
    case ADI_APOLLO_CONV_POW_OUTPUT_INVALID:
    case ADI_APOLLO_CONV_POW_NUM_OUTPUTS:
        // Sentinel values - not valid outputs
        return (adi_apollo_rxtx_enable_e)(0xFF);
    default:
        return (adi_apollo_rxtx_enable_e)(0xFF);
    }
}

/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_rxen_pwrup_ctrl_config_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t adcs, const adi_apollo_rxen_pwrup_blk_config_t *config)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(adcs);

    err = adi_apollo_rxen_pwrup_ctrl_set(device, adcs, &config->ctrl);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_rxen_pwrup_ctrl_edge_set(device, adcs, &config->edge);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_rxen_pwrup_ctrl_count_set(device, adcs, &config->count);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_rxen_pwrup_ctrl_pin_set(device, adcs, config->pin_en);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_pwrup_ctrl_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t adcs, const adi_apollo_rxen_pwrup_ctrl_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t adc;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(adcs);

    for(i = 0U; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (adi_apollo_blk_sel_t)(adcs & ((uint32_t)ADI_APOLLO_RXEN_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_SM_EN_INFO(regmap_base_addr), config->sm_en);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_ENA_INFO(regmap_base_addr), config->spi_rxen_en);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_INFO(regmap_base_addr), config->spi_rxen);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_SEL_CNT_RATE_INFO(regmap_base_addr), (config->sm_clk_rate == ADI_APOLLO_PUC_CLK_RATE_FS_DIV_256) ? 1 : 0);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_pwrup_ctrl_edge_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t adcs, const adi_apollo_rxen_pwrup_ctrl_edge_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t adc;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(adcs);

    for(i = 0U; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (adi_apollo_blk_sel_t)(adcs & ((uint32_t)ADI_APOLLO_RXEN_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DAC_RISE_INFO(regmap_base_addr), (uint64_t)config->adc_rise);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DAC_FALL_INFO(regmap_base_addr), (uint64_t)config->adc_fall);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_DIG_RISE_INFO(regmap_base_addr), (uint64_t)config->dig_rise);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DIG_FALL_INFO(regmap_base_addr), (uint64_t)config->dig_fall);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_PA_RISE_INFO(regmap_base_addr), (uint64_t)config->pa_rise);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_PA_FALL_INFO(regmap_base_addr), (uint64_t)config->pa_fall);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_pwrup_ctrl_count_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t adcs, const adi_apollo_rxen_pwrup_ctrl_count_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t adc;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(adcs);

    for(i = 0U; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (adi_apollo_blk_sel_t)(adcs & ((uint32_t)ADI_APOLLO_RXEN_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXA_INFO(regmap_base_addr), config->count_maxa);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXB_INFO(regmap_base_addr), config->count_maxb);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXC_INFO(regmap_base_addr), config->count_maxc);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXD_INFO(regmap_base_addr), config->count_maxd);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXE_INFO(regmap_base_addr), config->count_maxe);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXF_INFO(regmap_base_addr), config->count_maxf);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_pwrup_ctrl_pin_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t adcs, adi_apollo_puc_en_pin_sel_e pin)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t adc;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(pin, ADI_APOLLO_PUC_EN_PIN_0, ADI_APOLLO_PUC_EN_PIN_3);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(adcs);

    for(i = 0U; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (adi_apollo_blk_sel_t)(adcs & ((uint32_t)ADI_APOLLO_RXEN_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_TXEN_SEL_INFO(regmap_base_addr), (uint64_t)pin);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_pwrup_ctrl_spien_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t adcs, uint8_t spi_en)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t adc;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_RXEN_BLK_SEL_MASK(adcs);

    for(i = 0U; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (adi_apollo_blk_sel_t)(adcs & ((uint32_t)ADI_APOLLO_RXEN_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_INFO(regmap_base_addr), spi_en);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}



int32_t adi_apollo_rxen_config_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dig_enables, const adi_apollo_rxen_blk_config_t *config) 
{
    int32_t err;
    uint8_t i, side;
    adi_apollo_blk_sel_t dig_en;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_INVALID_PARAM_RETURN(config->pfilt_sel > (uint16_t)ADI_APOLLO_PFILT_ALL);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(dig_enables);
    
    for(i = 0U; i < ADI_APOLLO_DIG_EN_NUM; i ++) {
        dig_en = (adi_apollo_blk_sel_t)(dig_enables & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_RXEN_EN_A0 << i));
        if (dig_en > 0U) {
            side = i / ADI_APOLLO_DIG_EN_PER_SIDE_NUM;
            regmap_base_addr = calc_rx_enable_base(side);

            /* Calculate side index once for reuse */
            uint8_t side_index = i % ADI_APOLLO_DIG_EN_PER_SIDE_NUM;

            /* Enable polarity */
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_POL_INFO(regmap_base_addr, side_index),
                                        (config->enable_polarity == ADI_APOLLO_DIG_EN_ACTIVE_HIGH) ? 0 : 1);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SPI enable */
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPI_INFO(regmap_base_addr, side_index),
                                        (config->spi_en == ADI_APOLLO_DISABLE) ? 0U : 1U);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SPIEN enable */
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPIEN_INFO(regmap_base_addr, side_index),
                                        (config->spien_en == ADI_APOLLO_DISABLE) ? 0U : 1U);
            ADI_APOLLO_ERROR_RETURN(err);


            /* Slice select: config.slice_sel is ignored. And a static pattern is used. */
            uint32_t slice_mask = 1U << side_index;
            err = adi_apollo_hal_bf_set(device,
                                        BF_SLICE_SEL_INFO(regmap_base_addr, side_index),
                                        (uint8_t)slice_mask);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Linx select */
            uint8_t linx_value = (side == 0U) ? config->linx_sel : (config->linx_sel >> ADI_APOLLO_NUM_LINX_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_LINX_SEL_INFO(regmap_base_addr, side_index),
                                        linx_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* PFILT select */
            uint16_t pfilt_value = (side == 0U) ? config->pfilt_sel : (config->pfilt_sel >> ADI_APOLLO_PFILT_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_PFILT_SEL_INFO(regmap_base_addr, side_index),
                                        (uint8_t)pfilt_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* CDDC select */
            uint16_t cddc_value = (side == 0U) ? config->cddc_sel : (config->cddc_sel >> ADI_APOLLO_CDDC_PER_SIDE_NUM);
            err = adi_apollo_hal_bf_set(device,
                                        BF_CDUC_CDDC_SEL_INFO(regmap_base_addr, side_index),
                                        (uint8_t)cddc_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* FDDC select */
            uint16_t fddc_value = (side == 0U) ? config->fddc_sel : (config->fddc_sel >> ADI_APOLLO_FDDC_PER_SIDE_NUM);
            err = adi_apollo_hal_bf_set(device,
                                        BF_FDUC_FDDC_SEL_INFO(regmap_base_addr, side_index),
                                        (uint8_t)fddc_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* CFIR select */
            uint8_t cfir_value = (side == 0U) ? config->cfir_sel : (config->cfir_sel >> ADI_APOLLO_NUM_CFIR_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_CFIR_SEL_INFO(regmap_base_addr, side_index),
                                        cfir_value);

            /* FSRC select */
            uint8_t fsrc_value = (side == 0U) ? config->fsrc_sel : (config->fsrc_sel >> ADI_APOLLO_NUM_FSRC_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_FSRC_SEL_INFO(regmap_base_addr, side_index),
                                        fsrc_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* JTX link select */
            uint16_t jtx_link_value = (side == 0U) ? config->jtx_link_sel : (config->jtx_link_sel >> ADI_APOLLO_NUM_JTX_LINKS_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_JRX_JTX_LINK_SEL_INFO(regmap_base_addr, side_index),
                                        (uint8_t)jtx_link_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* JTX phy select */
            uint16_t jtx_phy_value = (side == 0U) ? config->jtx_phy_sel_side_a : config->jtx_phy_sel_side_b;
            err = adi_apollo_hal_bf_set(device,
                                        BF_JRX_JTX_PHY_SEL_INFO(regmap_base_addr, side_index),
                                        (uint8_t)jtx_phy_value);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_ctrl_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dig_enables, const adi_apollo_rxen_ctrl_t *config)
{
    int32_t err;
    uint8_t i, side;
    adi_apollo_blk_sel_t dig_en;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_RXEN_BLK_SEL_MASK(dig_enables);
    
    for(i = 0U; i < ADI_APOLLO_DIG_EN_NUM; i ++) {
        dig_en = (adi_apollo_blk_sel_t)(dig_enables & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_RXEN_EN_A0 << i));
        if (dig_en > 0U) {
            side = i / ADI_APOLLO_DIG_EN_PER_SIDE_NUM;
            regmap_base_addr = calc_rx_enable_base(side);

            /* Calculate side index once for reuse */
            uint8_t side_index = i % ADI_APOLLO_DIG_EN_PER_SIDE_NUM;

            /* Enable polarity */
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_POL_INFO(regmap_base_addr, side_index),
                                        (config->enable_polarity == ADI_APOLLO_DIG_EN_ACTIVE_HIGH) ? 0 : 1);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SPI enable */
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPI_INFO(regmap_base_addr, side_index),
                                        (config->spi_en == ADI_APOLLO_DISABLE) ? 0U : 1U);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SPIEN enable */
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPIEN_INFO(regmap_base_addr, side_index),
                                        (config->spien_en == ADI_APOLLO_DISABLE) ? 0U : 1U);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_rxen_spien_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dig_enables, uint8_t spi_en)
{
    int32_t err;
    uint8_t i, side;
    adi_apollo_blk_sel_t dig_en;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_RXEN_BLK_SEL_MASK(dig_enables);
    
    for(i = 0U; i < ADI_APOLLO_DIG_EN_NUM; i ++) {
        dig_en = (adi_apollo_blk_sel_t)(dig_enables & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_RXEN_EN_A0 << i));
        if (dig_en > 0U) {
            side = i / ADI_APOLLO_DIG_EN_PER_SIDE_NUM;
            regmap_base_addr = calc_rx_enable_base(side);

            /* SPI enable */
            uint8_t side_index = i % ADI_APOLLO_DIG_EN_PER_SIDE_NUM;
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPI_INFO(regmap_base_addr, side_index),
                                        (spi_en == ADI_APOLLO_DISABLE) ? 0U : 1U);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/* HCLK period in nanoseconds for CONV_POW_ONOFF threshold conversion */
#define HCLK_PERIOD_NS 2U

/* Maximum 24-bit threshold value for CONV_POW_ONOFF */
#define CONV_POW_THRESH_MAX 0xFFFFFFU

/* Number of CONV_POW_ONOFF outputs per side */
#define CONV_POW_OUTPUTS_PER_SIDE 4


/**
 * \brief Apply static configuration to the CONV_POW_ONOFF and TXRX_ENABLE blocks.
 *
 * The required flexibility to enable/disable specific RX datapaths by GPIO is provided by the PWR_CTRL block.
 */
static int32_t tdd_rx_static_cfg_set(adi_apollo_device_t *device)
{
    int32_t err = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN0),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN1),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX1);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN2),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX2);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN3),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX3);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN0),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN1),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX1);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN2),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX2);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN3),
        (uint64_t)ADI_APOLLO_RXTX_ENABLE_RX3);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Slice select: config.slice_sel is ignored. And a static pattern is used. */
    for (uint32_t side = 0U; side < (uint32_t)ADI_APOLLO_NUM_SIDES; side++) {
        uint32_t regmap_base_addr = calc_rx_enable_base(side);
        for (uint8_t adc_id = 0U; adc_id < ADI_APOLLO_DIG_EN_PER_SIDE_NUM; adc_id++) {
            err = adi_apollo_hal_bf_set(device,
                                        BF_SLICE_SEL_INFO(regmap_base_addr, (adc_id)),
                                        (uint64_t)(1U << adc_id));
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


int32_t adi_apollo_rxen_set(adi_apollo_device_t *device,
    adi_apollo_rxtx_enable_e rx_enable,
    adi_apollo_rxen_en_select_e dest_rx_sel,
    uint32_t delay_rise_ns,
    uint32_t delay_fall_ns)
{
    int32_t err = 0;
    uint8_t conv_pow_output = 0U;
    uint8_t side = 0;
    uint32_t regmap_base_addr = 0;
    uint32_t thresh_lh = 0;
    uint32_t thresh_hl = 0;
    uint8_t pin_sel = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    ADI_APOLLO_INVALID_PARAM_RETURN(
        (rx_enable < ADI_APOLLO_RXTX_ENABLE_RX0 || rx_enable > ADI_APOLLO_RXTX_ENABLE_RX3));
    ADI_APOLLO_INVALID_PARAM_RETURN(dest_rx_sel == ADI_APOLLO_RXEN_EN_NONE);

    /* Convert nanoseconds to CONV_POW_ONOFF threshold counts (HCLK period = 2ns) */
    thresh_lh = delay_rise_ns / HCLK_PERIOD_NS;
    thresh_hl = delay_fall_ns / HCLK_PERIOD_NS;
    ADI_APOLLO_INVALID_PARAM_RETURN(thresh_lh > CONV_POW_THRESH_MAX);
    ADI_APOLLO_INVALID_PARAM_RETURN(thresh_hl > CONV_POW_THRESH_MAX);

    adi_apollo_blk_sel_t dest_rx_sel_mask = (adi_apollo_blk_sel_t)dest_rx_sel;
    ADI_APOLLO_RXEN_BLK_SEL_MASK(dest_rx_sel_mask);

    if (device->dev_info.rxen_tdd_static_cfg_done == 0U) {
        err = tdd_rx_static_cfg_set(device);
        ADI_APOLLO_ERROR_RETURN(err);
        device->dev_info.rxen_tdd_static_cfg_done = 1;
    }

    /* Loop through all the CONV_POW_ONOFF Rx outputs and configure those that are mapped to the source trigger. */
    for (conv_pow_output = (uint8_t)ADI_APOLLO_CONV_POW_OUTPUT_RX_A_EN0; conv_pow_output <= (uint8_t)ADI_APOLLO_CONV_POW_OUTPUT_RX_B_EN3; conv_pow_output++) {
        adi_apollo_rxtx_enable_e conv_pow_input = conv_pow_output_to_rxtx_enable((adi_apollo_conv_pow_output_e)conv_pow_output);

        if (conv_pow_input != rx_enable) {
            // This CONV_POW_ONOFF output is not mapped to the selected rx_enable so skip it.
            continue;
        }

        err = adi_apollo_hal_bf_set(device, BF_COUNTER_BYPASS_EN_INFO(conv_pow_output),
            ((thresh_lh == 0U) && (thresh_hl == 0U)) ? 1U : 0U);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_POWER_EN_MASK_INFO(conv_pow_output), 0);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_THRESH_LH_INFO(conv_pow_output), thresh_lh);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_THRESH_HL_INFO(conv_pow_output), thresh_hl);
        ADI_APOLLO_ERROR_RETURN(err);

    }

    /* --- POWER_CTRL block: SPI_RXEN_ENA configuration ---
     * The 4 SPI_RXEN_ENA settings on each side are wired to all channels on that side.
     * Turn off SPI_RXEN_ENA for the selected rx_enable signal on sides with dest channels. */
    pin_sel = (uint8_t)(rx_enable - ADI_APOLLO_RXTX_ENABLE_RX0);

    if ((dest_rx_sel_mask & 0x0FU) != 0U) {
        /* Target channels exist on Side A: Put TXEN_tempX signal on Side-A under gpio/TXEN control */
        regmap_base_addr = rxen_id_to_rxen_power_ctrl_base_A(pin_sel);
        err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_ENA_INFO(regmap_base_addr), 0);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if ((dest_rx_sel_mask & 0xF0U) != 0U) {
        /* Target channels exist on Side B: Put TXEN_tempX signal on Side-B under gpio/TXEN control */
        regmap_base_addr = rxen_id_to_rxen_power_ctrl_base_B(pin_sel);
        err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_ENA_INFO(regmap_base_addr), 0);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    /* --- POWER_CTRL block: per-channel rxen_sel configuration --- */
    for (uint32_t rx_chan_id = 0U; rx_chan_id < ADI_APOLLO_DIG_EN_NUM; rx_chan_id++) {

        if ((dest_rx_sel_mask & (1U << rx_chan_id)) == 0U) {
            continue;
        }

        regmap_base_addr = calc_txen_power_ctrl_base(rx_chan_id);

        /* Set rxen_sel to choose which RXEN_temp signal (0-3) controls this channel */
        pin_sel = (uint8_t)(rx_enable - ADI_APOLLO_RXTX_ENABLE_RX0);
        err = adi_apollo_hal_bf_set(device, BF_TXEN_SEL_INFO(regmap_base_addr), pin_sel);
        ADI_APOLLO_ERROR_RETURN(err);

        /* RX does not use state machine for PA delay, so always disable */
        err = adi_apollo_hal_bf_set(device, BF_SM_EN_INFO(regmap_base_addr), 0);
        ADI_APOLLO_ERROR_RETURN(err);

        /* --- TXRX_ENABLE: configure enable channel for this datapath --- */
        uint32_t side_div_result = rx_chan_id / ADI_APOLLO_DIG_EN_PER_SIDE_NUM;
        side = (uint8_t)side_div_result;
        uint32_t side_chan_id = rx_chan_id % ADI_APOLLO_DIG_EN_PER_SIDE_NUM;
        regmap_base_addr = calc_rx_enable_base(side);

        err = adi_apollo_hal_bf_set(device,
            BF_ENABLE_POL_INFO(regmap_base_addr, (side_chan_id)), 0);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device,
            BF_ENABLE_SPI_INFO(regmap_base_addr, (side_chan_id)), 0);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device,
            BF_ENABLE_SPIEN_INFO(regmap_base_addr, (side_chan_id)), 0);
        ADI_APOLLO_ERROR_RETURN(err);

    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}



static uint32_t calc_txen_power_ctrl_base(uint32_t adc_index)
{
    static uint32_t txen_power_ctrl_regmap[] = {
        /* SLICE0 instances exist in 8t8r & 4t4r  SLICE1 instances exist in 8t8r only */
        RXEN_POWER_CTRL0_RX_SLICE_0_RX_DIGITAL0,  /* A0 */
        RXEN_POWER_CTRL1_RX_SLICE_0_RX_DIGITAL0,  /* A1 */
        RXEN_POWER_CTRL0_RX_SLICE_1_RX_DIGITAL0,  /* A2 (8t8r only) */
        RXEN_POWER_CTRL1_RX_SLICE_1_RX_DIGITAL0,  /* A3 (8t8r only) */
        RXEN_POWER_CTRL0_RX_SLICE_0_RX_DIGITAL1,  /* B0 */
        RXEN_POWER_CTRL1_RX_SLICE_0_RX_DIGITAL1,  /* B1 */
        RXEN_POWER_CTRL0_RX_SLICE_1_RX_DIGITAL1,  /* B2 (8t8r only) */
        RXEN_POWER_CTRL1_RX_SLICE_1_RX_DIGITAL1   /* B3 (8t8r only) */
    };

    return (txen_power_ctrl_regmap[adc_index]);
}

static uint32_t rxen_id_to_rxen_power_ctrl_base_A(uint32_t rxen_id)
{
    static uint32_t rxen_id_to_rxen_power_ctrl_base_A_map[] = {
        /* CTRL0 instances exist in 8t8r & 4t4r  CTRL1 instances exist in 8t8r only */
        RXEN_POWER_CTRL0_RX_SLICE_0_RX_DIGITAL0,
        RXEN_POWER_CTRL1_RX_SLICE_0_RX_DIGITAL0,
        RXEN_POWER_CTRL0_RX_SLICE_1_RX_DIGITAL0,
        RXEN_POWER_CTRL1_RX_SLICE_1_RX_DIGITAL0
    };

    return (rxen_id_to_rxen_power_ctrl_base_A_map[rxen_id]);
}

static uint32_t rxen_id_to_rxen_power_ctrl_base_B(uint32_t rxen_id)
{
    static uint32_t rxen_id_to_rxen_power_ctrl_base_B_map[] = {
        /* CTRL0 instances exist in 8t8r & 4t4r  CTRL1 instances exist in 8t8r only */
        RXEN_POWER_CTRL0_RX_SLICE_0_RX_DIGITAL1,
        RXEN_POWER_CTRL1_RX_SLICE_0_RX_DIGITAL1,
        RXEN_POWER_CTRL0_RX_SLICE_1_RX_DIGITAL1,
        RXEN_POWER_CTRL1_RX_SLICE_1_RX_DIGITAL1
    };

    return (rxen_id_to_rxen_power_ctrl_base_B_map[rxen_id]);
}

static uint32_t calc_rx_enable_base(uint32_t index)
{
    static uint32_t rx_enable_regmap[] = {
        TXRX_ENABLE_RX_DIGITAL0, TXRX_ENABLE_RX_DIGITAL1
    };

    return (rx_enable_regmap[index]);
}
/*! @} */