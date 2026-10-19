/*!
 * \brief    APIs for TxEn
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_TXEN
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_txen.h"
#include "adi_apollo_txen_local.h"
#include "adi_apollo_txmux.h"
#include "adi_apollo_pfilt.h"
#include "adi_apollo_cduc.h"
#include "adi_apollo_fduc.h"
#include "adi_apollo_private_blk_sel_types.h"

#include "adi_apollo_bf_conv_pow_onoff.h"
#include "adi_apollo_bf_txen_power_ctrl.h"
#include "adi_apollo_bf_txrx_enable.h"
#include "adi_apollo_bf_tx_hsdout.h"
#include "adi_utils.h"

#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"

/* Forward declarations for static helper functions */
static uint32_t calc_tx_txen_power_ctrl_base(uint32_t dac_index);
static uint32_t txen_id_to_txen_power_ctrl_base_A(uint32_t txen_id);
static uint32_t txen_id_to_txen_power_ctrl_base_B(uint32_t txen_id);
static uint32_t calc_tx_enable_base(uint32_t index);

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

int32_t adi_apollo_txen_pwrup_ctrl_config_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, const adi_apollo_txen_pwrup_blk_config_t *config)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);

    err = adi_apollo_txen_pwrup_ctrl_set(device, dacs, &config->ctrl);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_txen_pwrup_ctrl_edge_set(device, dacs, &config->edge);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_txen_pwrup_ctrl_count_set(device, dacs, &config->count);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_txen_pwrup_ctrl_pin_set(device, dacs, config->pin_en);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_txen_pwrup_ctrl_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, const adi_apollo_txen_pwrup_ctrl_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t dac;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);

    for (i = 0U; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (adi_apollo_blk_sel_t)(dacs & ((uint32_t)ADI_APOLLO_TXEN_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_SM_EN_INFO(regmap_base_addr), config->sm_en);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_ENA_INFO(regmap_base_addr), config->spi_txen_en);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_INFO(regmap_base_addr), config->spi_txen);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_SEL_CNT_RATE_INFO(regmap_base_addr), 
                                        (config->sm_clk_rate == ADI_APOLLO_PUC_CLK_RATE_FS_DIV_256) ? 1 : 0);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_txen_pwrup_ctrl_edge_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, const adi_apollo_txen_pwrup_ctrl_edge_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t dac;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);

    for (i = 0U; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (adi_apollo_blk_sel_t)(dacs & ((uint32_t)ADI_APOLLO_TXEN_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DAC_RISE_INFO(regmap_base_addr), (uint64_t)config->dac_rise);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DAC_FALL_INFO(regmap_base_addr), (uint64_t)config->dac_fall);
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

int32_t adi_apollo_txen_pwrup_ctrl_count_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, const adi_apollo_txen_pwrup_ctrl_count_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t dac;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);

    for (i = 0U; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (adi_apollo_blk_sel_t)(dacs & ((uint32_t)ADI_APOLLO_TXEN_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_txen_power_ctrl_base(i);

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

int32_t adi_apollo_txen_pwrup_ctrl_pin_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, adi_apollo_puc_en_pin_sel_e pin)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t dac;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(pin, ADI_APOLLO_PUC_EN_PIN_0, ADI_APOLLO_PUC_EN_PIN_3);
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);

    for (i = 0U; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (adi_apollo_blk_sel_t)(dacs & ((uint32_t)ADI_APOLLO_TXEN_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_TXEN_SEL_INFO(regmap_base_addr), (uint64_t)pin);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_txen_pwrup_ctrl_sm_en_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, uint8_t sm_en)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t dac;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);
    
    for (i = 0U; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (adi_apollo_blk_sel_t)(dacs & ((uint32_t)ADI_APOLLO_TXEN_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_txen_power_ctrl_base(i);

            err = adi_apollo_hal_bf_set(device, BF_SM_EN_INFO(regmap_base_addr), sm_en);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);

}

int32_t adi_apollo_txen_pwrup_ctrl_sel_cnt_rate_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t dacs, uint8_t sel_cnt_rate)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t dac;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dacs);
    
    for (i = 0U; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (adi_apollo_blk_sel_t)(dacs & ((uint32_t)ADI_APOLLO_TXEN_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_txen_power_ctrl_base(i);
            err = adi_apollo_hal_bf_set(device, BF_SEL_CNT_RATE_INFO(regmap_base_addr),
                                        (sel_cnt_rate == (uint8_t)ADI_APOLLO_PUC_CLK_RATE_FS_DIV_256) ? 1 : 0);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);

}

/* Map Tx chanId to address for bias_force_standby_en bitfield */
// Canonical Version
static uint32_t tx_chan_id_to_bias_force_standby_en_addr[] = {
    0x6123880b, // A0
    0x6123980b, // A1
    0x6143880b, // A2
    0x6143980b, // A3
    0x61a3880b, // B0
    0x61a3980b, // B1
    0x61c3880b, // B2
    0x61c3980b  // B3
};


int32_t adi_apollo_txen_config_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t tx_enables, const adi_apollo_txen_blk_config_t *config)
{
    int32_t err;
    uint8_t chan_id, side;
    adi_apollo_blk_sel_t tx_en;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_TXEN_BLK_SEL_MASK(tx_enables);

    for (chan_id = 0U; chan_id < ADI_APOLLO_NUM_TX_EN; chan_id ++) {
        tx_en = (adi_apollo_blk_sel_t)(tx_enables & ((uint32_t)ADI_APOLLO_TX_EN_A0 << chan_id));
        if (tx_en > 0U) {

            side = chan_id / ADI_APOLLO_NUM_TX_EN_PER_SIDE;

            /* TXRX_ENABLE block on TX_SLICE0 controls both slices. There is no TXRX_ENABLE block on TX_SLICE1. */
            regmap_base_addr = calc_tx_enable_base(side);

            /* Enable polarity */
            unsigned int pol_value = (config->enable_polarity == ADI_APOLLO_TXEN_EN_ACTIVE_HIGH) ? 0U : 1U;
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_POL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        (uint8_t)pol_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SPI enable */
            unsigned int spi_enable_value = (config->spi_en == ADI_APOLLO_DISABLE) ? 0U : 1U;
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPI_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        (uint8_t)spi_enable_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SPIEN enable */
            unsigned int spien_enable_value = (config->spien_en == ADI_APOLLO_DISABLE) ? 0U : 1U;
            err = adi_apollo_hal_bf_set(device,
                                        BF_ENABLE_SPIEN_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        (uint8_t)spien_enable_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Slice select: config.slice_sel is ignored. And a static pattern is used. */
            uint32_t slice_sel_value = 1U << (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_SLICE_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        (uint8_t)slice_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Linx select */
            uint16_t linx_sel_value = (side == 0U) ? config->linx_sel : (config->linx_sel >> ADI_APOLLO_NUM_LINX_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_LINX_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        linx_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* PFILT select */
            uint16_t pfilt_sel_value = (side == 0U) ? config->pfilt_sel : (config->pfilt_sel >> ADI_APOLLO_PFILT_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_PFILT_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        pfilt_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* CDUC select */
            uint16_t cduc_sel_value = (side == 0U) ? config->cduc_sel : (config->cduc_sel >> ADI_APOLLO_CDUC_PER_SIDE_NUM);
            err = adi_apollo_hal_bf_set(device,
                                        BF_CDUC_CDDC_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        cduc_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* FDUC select */
            uint16_t fduc_sel_value = (side == 0U) ? config->fduc_sel : (config->fduc_sel >> ADI_APOLLO_FDUC_PER_SIDE_NUM);
            err = adi_apollo_hal_bf_set(device,
                                        BF_FDUC_FDDC_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        fduc_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* CFIR select */
            uint16_t cfir_sel_value = (side == 0U) ? config->cfir_sel : (config->cfir_sel >> ADI_APOLLO_NUM_CFIR_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_CFIR_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        cfir_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* FSRC select */
            uint16_t fsrc_sel_value = (side == 0U) ? config->fsrc_sel : (config->fsrc_sel >> ADI_APOLLO_NUM_FSRC_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_FSRC_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        fsrc_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* JRX link select */
            uint16_t jrx_link_sel_value = (side == 0U) ? config->jrx_link_sel : (config->jrx_link_sel >> ADI_APOLLO_NUM_JRX_LINKS_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_JRX_JTX_LINK_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        jrx_link_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);
            
            /* JRX phy0 & phy1 select */
            uint16_t jrx_phy_sel_value = (side == 0U) ? config->jrx_phy_sel_side_a : config->jrx_phy_sel_side_b;
            err = adi_apollo_hal_bf_set(device,
                                        BF_JRX_JTX_PHY_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        jrx_phy_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);            
        
            /* MODSW select */
            unsigned int modsw_sel_value = (config->modsw_sel == ADI_APOLLO_DISABLE) ? 0U : 1U;
            err = adi_apollo_hal_bf_set(device,
                                        BF_MODSW_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        (uint8_t)modsw_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* INVSINC select */
            uint16_t invsinc_sel_value = (side == 0U) ? config->invsinc_sel : (config->invsinc_sel >> ADI_APOLLO_NUM_TX_INVSINC_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_INVSINC_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        invsinc_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* GAIN select */
            uint16_t gain_sel_value = (side == 0U) ? config->gain_sel : (config->gain_sel >> ADI_APOLLO_NUM_TX_GAIN_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_GAIN_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        gain_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* SRD select */
            uint16_t srd_sel_value = (side == 0U) ? config->srd_sel : (config->srd_sel >> ADI_APOLLO_NUM_TX_SRD_PER_SIDE);
            err = adi_apollo_hal_bf_set(device,
                                        BF_SRD_SEL_INFO(regmap_base_addr, (chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE)),
                                        srd_sel_value);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Set DAC to disable output on standby signal */
            err = adi_apollo_hal_bf_set(device, tx_chan_id_to_bias_force_standby_en_addr[chan_id], 0x0104, 0);
            ADI_CMS_ERROR_RETURN(err);
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
 * The required flexibility to enable/disable specific TX/RX datapaths by GPIO is provided by the PWR_CTRL block.
 */
static int32_t tdd_tx_static_cfg_set(adi_apollo_device_t *device)
{
    int32_t err = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN0),
        ADI_APOLLO_RXTX_ENABLE_TX0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN1),
        ADI_APOLLO_RXTX_ENABLE_TX1);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN2),
        ADI_APOLLO_RXTX_ENABLE_TX2);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN3),
        ADI_APOLLO_RXTX_ENABLE_TX3);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN0),
        ADI_APOLLO_RXTX_ENABLE_TX0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN1),
        ADI_APOLLO_RXTX_ENABLE_TX1);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN2),
        ADI_APOLLO_RXTX_ENABLE_TX2);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device,
        BF_MUX_SELECT_INFO(ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN3),
        ADI_APOLLO_RXTX_ENABLE_TX3);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Slice select: config.slice_sel is ignored. And a static pattern is used. */
    for (uint8_t side = 0; side < (uint8_t)ADI_APOLLO_NUM_SIDES; side++) {
        uint32_t regmap_base_addr = calc_tx_enable_base(side);
        for (uint8_t dac_id = 0U; dac_id < ADI_APOLLO_NUM_DAC_PER_SIDE; dac_id++) {
            err = adi_apollo_hal_bf_set(device,
                                        BF_SLICE_SEL_INFO(regmap_base_addr, (dac_id)),
                                        (uint64_t)(1U << dac_id));
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    /* Ensure DAC is disabled with datapath or it will continue to operate repeating random old samples. */
    for (uint32_t tx_chan_id = 0U; tx_chan_id < ADI_APOLLO_NUM_TX_EN; tx_chan_id++) {
        err = adi_apollo_hal_bf_set(device,
            tx_chan_id_to_bias_force_standby_en_addr[tx_chan_id],
            0x0104,
            0);
        ADI_APOLLO_ERROR_RETURN(err);
    }
    
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_txen_set(adi_apollo_device_t *device,
    adi_apollo_rxtx_enable_e tx_enable,
    adi_apollo_txen_en_select_e dest_tx_sel,
    uint32_t delay_rise_ns,
    uint32_t delay_fall_ns,
    uint32_t pa_delay_rise_ns,
    uint32_t pa_delay_fall_ns)
{
    int32_t err = 0;
    uint8_t conv_pow_output = 0U;
    uint8_t side = 0;
    uint32_t regmap_base_addr = 0;
    uint32_t thresh_lh = 0;
    uint32_t thresh_hl = 0;
    uint32_t count_a = 0;
    uint32_t count_d = 0;
    uint32_t sm_clk_div = 0;
    uint8_t pin_sel = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    ADI_APOLLO_INVALID_PARAM_RETURN(
        (tx_enable < ADI_APOLLO_RXTX_ENABLE_TX0 || tx_enable > ADI_APOLLO_RXTX_ENABLE_TX3));
    ADI_APOLLO_INVALID_PARAM_RETURN(dest_tx_sel == ADI_APOLLO_TX_EN_NONE);

    /* Convert nanoseconds to CONV_POW_ONOFF threshold counts (HCLK period = 2ns) */
    thresh_lh = delay_rise_ns / HCLK_PERIOD_NS;
    thresh_hl = delay_fall_ns / HCLK_PERIOD_NS;
    ADI_APOLLO_INVALID_PARAM_RETURN(thresh_lh > CONV_POW_THRESH_MAX);
    ADI_APOLLO_INVALID_PARAM_RETURN(thresh_hl > CONV_POW_THRESH_MAX);

    adi_apollo_blk_sel_t dest_tx_sel_mask = (adi_apollo_blk_sel_t)dest_tx_sel;
    ADI_APOLLO_TXEN_BLK_SEL_MASK(dest_tx_sel_mask);

    /* Convert PA delay nanoseconds to TXEN_POWER_CTRL state machine counter counts.
     * SM runs at Fdac/32 (4T4R) or Fdac/16 (8T8R). count = delay_ns * dac_freq_hz / (div * 1e9) */
    sm_clk_div = device->dev_info.is_8t8r ? 16U : 32U;
    count_a = (uint32_t)adi_api_utils_div_floor_u64((uint64_t)pa_delay_rise_ns * device->dev_info.dev_freq_hz, (uint64_t)sm_clk_div * 1000000000ULL);
    ADI_APOLLO_INVALID_PARAM_RETURN(count_a > 0xFFU);
    count_d = (uint32_t)adi_api_utils_div_floor_u64((uint64_t)pa_delay_fall_ns * device->dev_info.dev_freq_hz, (uint64_t)sm_clk_div * 1000000000ULL);
    ADI_APOLLO_INVALID_PARAM_RETURN(count_d > 0xFFU);

    if (device->dev_info.txen_tdd_static_cfg_done == 0U) {
        err = tdd_tx_static_cfg_set(device);
        ADI_APOLLO_ERROR_RETURN(err);
        device->dev_info.txen_tdd_static_cfg_done = 1;
    }

    /* Loop through all the CONV_POW_ONOFF Tx outputs and configure those that are mapped to the source trigger. */
    for (conv_pow_output = (uint8_t)ADI_APOLLO_CONV_POW_OUTPUT_TX_A_EN0; conv_pow_output <= (uint8_t)ADI_APOLLO_CONV_POW_OUTPUT_TX_B_EN3; conv_pow_output++) {
        adi_apollo_rxtx_enable_e conv_pow_input = conv_pow_output_to_rxtx_enable((adi_apollo_conv_pow_output_e)conv_pow_output);

        if (conv_pow_input != tx_enable) {
            // This CONV_POW_ONOFF output is not mapped to the selected tx_enable so skip it.
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

    /* --- POWER_CTRL block: SPI_TXEN_ENA configuration ---
     * The 4 SPI_TXEN_ENA settings on each side are wired to all channels on that side.
     * Turn off SPI_TXEN_ENA for the selected tx_enable signal on sides with dest channels. */
    pin_sel = (uint8_t)(tx_enable - ADI_APOLLO_RXTX_ENABLE_TX0);

    /* Side A channels are bits 0-3, Side B channels are bits 4-7 */
    if ((dest_tx_sel_mask & 0x0FU) != 0U) {
        /* Target channels exist on Side A: Put TXEN_tempX signal on Side-A under gpio/TXEN control */
        regmap_base_addr = txen_id_to_txen_power_ctrl_base_A(pin_sel);
        err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_ENA_INFO(regmap_base_addr), 0);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if ((dest_tx_sel_mask & 0xF0U) != 0U) {
        /* Target channels exist on Side A: Put TXEN_tempX signal on Side-A under gpio/TXEN control */
        regmap_base_addr = txen_id_to_txen_power_ctrl_base_B(pin_sel);
        err = adi_apollo_hal_bf_set(device, BF_SPI_TXEN_ENA_INFO(regmap_base_addr), 0);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    /* --- POWER_CTRL block: per-channel txen_sel configuration --- */
    for (uint32_t tx_chan_id = 0U; tx_chan_id < ADI_APOLLO_NUM_TX_EN; tx_chan_id++) {

        if ((dest_tx_sel_mask & (1U << tx_chan_id)) == 0U) {
            continue;
        }

        regmap_base_addr = calc_tx_txen_power_ctrl_base(tx_chan_id);

        /* Set txen_sel to choose which TXEN_temp signal (0-3) controls this channel */
        pin_sel = (uint8_t)(tx_enable - ADI_APOLLO_RXTX_ENABLE_TX0);
        err = adi_apollo_hal_bf_set(device, BF_TXEN_SEL_INFO(regmap_base_addr), pin_sel);
        ADI_APOLLO_ERROR_RETURN(err);

        if ((pa_delay_rise_ns == 0U) && (pa_delay_fall_ns == 0U)) {
            /* DAC, DIG and PA all rise/fall with no delay. Totally bypass the state machine */
            err = adi_apollo_hal_bf_set(device, BF_SM_EN_INFO(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);
        } else {
            /* PA delay requested. DAC and DIG enabled at E0 (immediate with TxEN rising).
             * PA enabled at E1 after counter A if pa_delay_rise_ns > 0, else at E0. */
            err = adi_apollo_hal_bf_set(device, BF_DAC_RISE_INFO(regmap_base_addr), (uint64_t)ADI_APOLLO_PUC_EDGE_RISE_E0);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DIG_RISE_INFO(regmap_base_addr), (uint64_t)ADI_APOLLO_PUC_EDGE_RISE_E0);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_PA_RISE_INFO(regmap_base_addr),
                (uint64_t)((pa_delay_rise_ns > 0U) ? ADI_APOLLO_PUC_EDGE_RISE_E1 : ADI_APOLLO_PUC_EDGE_RISE_E0));
            ADI_APOLLO_ERROR_RETURN(err);

            /* Falling edge: DAC and DIG disabled immediately at E4.
             * PA disabled at E5 after counter D if pa_delay_fall_ns > 0, else at E4. */
            err = adi_apollo_hal_bf_set(device, BF_PA_FALL_INFO(regmap_base_addr),
                (uint64_t)((pa_delay_fall_ns > 0U) ? ADI_APOLLO_PUC_EDGE_FALL_E5 : ADI_APOLLO_PUC_EDGE_FALL_E4));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DAC_FALL_INFO(regmap_base_addr), (uint64_t)ADI_APOLLO_PUC_EDGE_FALL_E4);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DIG_FALL_INFO(regmap_base_addr), (uint64_t)ADI_APOLLO_PUC_EDGE_FALL_E4);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Counter A: delay from TxEN rising to PA enable.
             * Counter D: delay from TxEN falling to PA disable. */
            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXA_INFO(regmap_base_addr), count_a);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXB_INFO(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXC_INFO(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXD_INFO(regmap_base_addr), count_d);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXE_INFO(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_COUNT_MAXF_INFO(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_SM_EN_INFO(regmap_base_addr), 1);
            ADI_APOLLO_ERROR_RETURN(err);
        }

        /* --- TXRX_ENABLE: configure enable channel for this datapath --- */
        uint32_t side_value = tx_chan_id / ADI_APOLLO_NUM_TX_EN_PER_SIDE;
        side = (uint8_t)side_value;
        uint32_t side_chan_id = tx_chan_id % ADI_APOLLO_NUM_TX_EN_PER_SIDE;
        regmap_base_addr = calc_tx_enable_base(side);

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

static uint32_t calc_tx_txen_power_ctrl_base(uint32_t dac_index)
{
    static uint32_t tx_txen_power_ctrl_regmap[] = {
        /* SLICE0 instances exist in 8t8r & 4t4r  SLICE1 instances exist in 8t8r only */
        TXEN_POWER_CTRL0_TX_SLICE_0_TX_DIGITAL0,  /* A0 */
        TXEN_POWER_CTRL1_TX_SLICE_0_TX_DIGITAL0,  /* A1 */
        TXEN_POWER_CTRL0_TX_SLICE_1_TX_DIGITAL0,  /* A2 (8t8r only)*/
        TXEN_POWER_CTRL1_TX_SLICE_1_TX_DIGITAL0,  /* A3 (8t8r only) */
        TXEN_POWER_CTRL0_TX_SLICE_0_TX_DIGITAL1,  /* B0 */
        TXEN_POWER_CTRL1_TX_SLICE_0_TX_DIGITAL1,  /* B1 */
        TXEN_POWER_CTRL0_TX_SLICE_1_TX_DIGITAL1,  /* B2 (8t8r only) */
        TXEN_POWER_CTRL1_TX_SLICE_1_TX_DIGITAL1   /* B3 (8t8r only) */
    };

    return (tx_txen_power_ctrl_regmap[dac_index]);
}

static uint32_t txen_id_to_txen_power_ctrl_base_A(uint32_t txen_id)
{
    static uint32_t txen_id_to_txen_power_ctrl_base_A_map[] = {
        /* CTRL0 instances exist in 8t8r & 4t4r  CTRL1 instances exist in 8t8r only */
        TXEN_POWER_CTRL0_TX_SLICE_0_TX_DIGITAL0,
        TXEN_POWER_CTRL1_TX_SLICE_0_TX_DIGITAL0,
        TXEN_POWER_CTRL0_TX_SLICE_1_TX_DIGITAL0,
        TXEN_POWER_CTRL1_TX_SLICE_1_TX_DIGITAL0
    };

    return (txen_id_to_txen_power_ctrl_base_A_map[txen_id]);
}

static uint32_t txen_id_to_txen_power_ctrl_base_B(uint32_t txen_id)
{
    static uint32_t txen_id_to_txen_power_ctrl_base_B_map[] = {
        /* CTRL0 instances exist in 8t8r & 4t4r  CTRL1 instances exist in 8t8r only */
        TXEN_POWER_CTRL0_TX_SLICE_0_TX_DIGITAL1,
        TXEN_POWER_CTRL1_TX_SLICE_0_TX_DIGITAL1,
        TXEN_POWER_CTRL0_TX_SLICE_1_TX_DIGITAL1,
        TXEN_POWER_CTRL1_TX_SLICE_1_TX_DIGITAL1
    };

    return (txen_id_to_txen_power_ctrl_base_B_map[txen_id]);
}

static uint32_t calc_tx_enable_base(uint32_t index)
{
    static uint32_t tx_enable_regmap[] = {
        TXRX_ENABLE_TX_SLICE_0_TX_DIGITAL0, TXRX_ENABLE_TX_SLICE_0_TX_DIGITAL1
    };

    return (tx_enable_regmap[index]);
}

/*! @} */
