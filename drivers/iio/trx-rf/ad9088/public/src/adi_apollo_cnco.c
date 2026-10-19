/*!
 * \brief     CNCO functional block API implementation
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_CNCO
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_cnco.h"
#include "adi_apollo_nco_local.h"
#include "adi_apollo_private_blk_sel_types.h"

#include "adi_apollo_bf_txrx_coarse_nco.h"

#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"

#include "adi_utils.h"

static uint8_t profile_sel_to_bf(adi_apollo_nco_profile_sel_mode_e ps);
static int32_t bf_to_nco_profile_sel(adi_apollo_device_t *device, uint32_t regmap_base_addr, adi_apollo_nco_profile_sel_mode_e *ps);
static uint8_t nco_profile_sel_to_profile_sel_mode_bf(adi_apollo_nco_profile_sel_mode_e ps);
uint32_t calc_tx_cnco_base(uint32_t idx);
uint32_t calc_rx_cnco_base(uint32_t idx);

/*==================== P U B L I C   A P I   C O D E ====================*/
int32_t adi_apollo_cnco_profile_load(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos,
                adi_apollo_nco_profile_word_sel_e word_sel, uint8_t first, uint32_t words[], uint32_t length)
{
    int32_t err;
    uint8_t i, j;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    /* Lower bound check (< ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT) omitted - parameter is unsigned enum */
    ADI_APOLLO_INVALID_PARAM_RETURN(word_sel > ADI_APOLLO_NCO_PROFILE_PHASE_OFFSET);
    ADI_APOLLO_NULL_POINTER_RETURN(words);
    ADI_APOLLO_INVALID_PARAM_RETURN((first+length) > (uint32_t)ADI_APOLLO_CNCO_PROFILE_NUM)
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {

            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            for(j = first; j < (first+length); j++) {
                err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), j); // only bf in reg, use wr reg
                ADI_APOLLO_ERROR_RETURN(err);

                if (word_sel == ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT) {
                    err = adi_apollo_hal_paged_bf_set(device, BF_DRC_PHASE_INC_INFO(regmap_base_addr), words[j-first]);
                    ADI_APOLLO_ERROR_RETURN(err);
                } else {
                    err = adi_apollo_hal_paged_bf_set(device, BF_DRC_PHASE_OFFSET_INFO(regmap_base_addr), words[j-first]);
                    ADI_APOLLO_ERROR_RETURN(err);
                }
            }

            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_profile_load_fast(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos,
                uint8_t inc_first, uint32_t inc_words[], uint32_t inc_length,
                uint8_t offset_first, uint32_t offset_words[], uint32_t offset_length)
{
    int32_t err;
    uint8_t cnco_id, profile_id;
    uint8_t load_inc, load_offset;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    load_inc    = (inc_length > 0U) ? 1U : 0U;
    load_offset = (offset_length > 0U) ? 1U : 0U;

    if (load_inc) {
        ADI_APOLLO_INVALID_PARAM_RETURN((inc_first + inc_length) > (uint32_t)ADI_APOLLO_CNCO_PROFILE_NUM)
        ADI_APOLLO_NULL_POINTER_RETURN(inc_words);
    }
    if (load_offset) {
        ADI_APOLLO_INVALID_PARAM_RETURN((offset_first + offset_length) > (uint32_t)ADI_APOLLO_CNCO_PROFILE_NUM)
        ADI_APOLLO_NULL_POINTER_RETURN(offset_words);
    }

    for (cnco_id = 0; cnco_id < ADI_APOLLO_CNCO_NUM; cnco_id++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << cnco_id);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(cnco_id) : calc_tx_cnco_base(cnco_id);

            /* Each phase word is a full-register 32-bit field; stream its 4 bytes LSB-first in a
             * single auto-incrementing SPI transaction. Increment and offset are loaded over
             * independent profile ranges, each preceded by its own PROFILE_UPDATE_CTRL select. */
            if (load_inc) {
                for (profile_id = inc_first; profile_id < (inc_first + inc_length); profile_id++) {
                    uint8_t wbuf[4];
                    uint32_t w = inc_words[profile_id - inc_first];

                    err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), profile_id);
                    ADI_APOLLO_ERROR_RETURN(err);

                    wbuf[0] = (uint8_t)(w);
                    wbuf[1] = (uint8_t)(w >> 8);
                    wbuf[2] = (uint8_t)(w >> 16);
                    wbuf[3] = (uint8_t)(w >> 24);
                    err = adi_apollo_hal_paged_stream_set(device, REG_DRC_PHASE_INC0_ADDR(regmap_base_addr), wbuf, 4);
                    ADI_APOLLO_ERROR_RETURN(err);
                }
            }

            if (load_offset) {
                for (profile_id = offset_first; profile_id < (offset_first + offset_length); profile_id++) {
                    uint8_t wbuf[4];
                    uint32_t w = offset_words[profile_id - offset_first];

                    err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), profile_id);
                    ADI_APOLLO_ERROR_RETURN(err);

                    wbuf[0] = (uint8_t)(w);
                    wbuf[1] = (uint8_t)(w >> 8);
                    wbuf[2] = (uint8_t)(w >> 16);
                    wbuf[3] = (uint8_t)(w >> 24);
                    err = adi_apollo_hal_paged_stream_set(device, REG_DRC_PHASE_OFFSET0_ADDR(regmap_base_addr), wbuf, 4);
                    ADI_APOLLO_ERROR_RETURN(err);
                }
            }

            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_cnco_hop_enable(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, adi_apollo_coarse_nco_hop_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for(i = 0U; i < ADI_APOLLO_CNCO_NUM; i ++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DRC_NCO_CHAN_SEL_MODE_INFO(regmap_base_addr), profile_sel_to_bf(config->profile_sel_mode));
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_FREQ_COHRNCE_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint64_t)config->phase_handling);
            ADI_APOLLO_ERROR_RETURN(err);

            if (config->profile_sel_mode <= ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO) {
                    err = adi_apollo_hal_bf_set(device, BF_PROFILE_SEL_MODE_TXRX_COARSE_NCO_INFO(regmap_base_addr), nco_profile_sel_to_profile_sel_mode_bf(config->profile_sel_mode));
                    ADI_APOLLO_ERROR_RETURN(err);
            }

            if (config->auto_mode == ADI_APOLLO_NCO_AUTO_HOP_FLIP) {
                if(config->low_limit >= config->high_limit){
                    ADI_CMS_RETURN(API_CMS_ERROR_INVALID_PARAM);
                }

                err = adi_apollo_hal_bf_set(device, BF_AUTO_INC_DECB_TXRX_COARSE_NCO_INFO(regmap_base_addr), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_set(device, BF_AUTOFLIP_INCDIR_TXRX_COARSE_NCO_INFO(regmap_base_addr), 1);
                ADI_APOLLO_ERROR_RETURN(err);
            } else {
                err = adi_apollo_hal_bf_set(device, BF_AUTO_INC_DECB_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint64_t)config->auto_mode);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_set(device, BF_AUTOFLIP_INCDIR_TXRX_COARSE_NCO_INFO(regmap_base_addr), 0);
                ADI_APOLLO_ERROR_RETURN(err);
            }

            err = adi_apollo_hal_bf_set(device, BF_HOP_HIGHLIM_INFO(regmap_base_addr), (uint64_t)config->high_limit);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_HOP_LOWLIM_INFO(regmap_base_addr), (uint64_t)config->low_limit);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_HOP_CTRL_INIT_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint64_t)config->hop_ctrl_init);
            ADI_APOLLO_ERROR_RETURN(err);

            if (config->profile_sel_mode == ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP) {
                err = adi_apollo_hal_bf_set(device, BF_NEXT_HOP_NUMBER_WR_EN_INFO(regmap_base_addr), (uint64_t)config->next_hop_number_wr_en);
                ADI_APOLLO_ERROR_RETURN(err);
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_chan_pgm(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint8_t profile_num, adi_apollo_coarse_nco_chan_pgm_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_INVALID_PARAM_RETURN(profile_num > (uint8_t)(ADI_APOLLO_CNCO_PROFILE_NUM - 1U));
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for(i = 0U; i < ADI_APOLLO_CNCO_NUM; i ++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            /* Select profile to update */
            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), profile_num);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_INC_INFO(regmap_base_addr), (uint64_t)config->drc_phase_inc);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_OFFSET_INFO(regmap_base_addr), (uint64_t)config->drc_phase_offset);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_INC_FRAC_A_INFO(regmap_base_addr), (uint64_t)config->drc_phase_inc_frac_a);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_INC_FRAC_B_INFO(regmap_base_addr), (uint64_t)config->drc_phase_inc_frac_b);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_pgm(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, adi_apollo_cnco_pgm_t *config)
{
    int32_t err;
    uint8_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_NULL_POINTER_RETURN(config);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for(i = 0U; i < ADI_APOLLO_CNCO_NUM; i ++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DRC_IF_MODE_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint64_t)config->if_mode);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_MIXER_SEL_INFO(regmap_base_addr), (uint64_t)config->mixer_sel);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_CMPLX_MXR_SCALE_EN_INFO(regmap_base_addr), (uint64_t)config->cmplx_mxr_scale_en);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), config->profile_num);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_NCO_CHAN_SEL_MODE_INFO(regmap_base_addr), profile_sel_to_bf(config->profile_sel_mode));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_NCO_REGMAP_CHAN_SEL_INFO(regmap_base_addr), (uint64_t)config->profile_num);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_INC_INFO(regmap_base_addr),config->drc_phase_inc);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_OFFSET_INFO(regmap_base_addr),config->drc_phase_offset);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_INC_FRAC_A_INFO(regmap_base_addr), (uint64_t)config->drc_phase_inc_frac_a);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_INC_FRAC_B_INFO(regmap_base_addr), (uint64_t)config->drc_phase_inc_frac_b);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_MIXER_TEST_MODE_VAL_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint64_t)config->dc_testmode_value);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_COARSE_DRC_EN_INFO(regmap_base_addr), (uint64_t)config->drc_en);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_CDRC_CLK_EN_INFO(regmap_base_addr),config->debug_drc_clkoff_n);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_inspect(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cnco, adi_apollo_cnco_inspect_t *cnco_inspect)
{
    int32_t err;
    uint8_t i, j;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_NULL_POINTER_RETURN(cnco_inspect);
    ADI_APOLLO_INVALID_PARAM_RETURN(adi_api_utils_num_selected(cnco) != 1U);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cnco);

    for(i = 0U; i < ADI_APOLLO_CNCO_NUM; i ++) {
        if ((cnco & ((uint32_t)ADI_APOLLO_CNCO_A0 << i)) > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

           err = adi_apollo_hal_bf_get(device, BF_AUTO_INC_DECB_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint8_t *) &(cnco_inspect->dp_cfg.nco_auto_inc_dec), sizeof(cnco_inspect->dp_cfg.nco_auto_inc_dec));
           ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_get(device, BF_DRC_IF_MODE_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint8_t *) &(cnco_inspect->dp_cfg.nco_if_mode), sizeof(cnco_inspect->dp_cfg.nco_if_mode));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_MIXER_SEL_INFO(regmap_base_addr), (uint8_t *) &(cnco_inspect->dp_cfg.drc_mxr_sel), sizeof(cnco_inspect->dp_cfg.drc_mxr_sel));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_CMPLX_MXR_SCALE_EN_INFO(regmap_base_addr), (uint8_t *) &(cnco_inspect->dp_cfg.cmplx_mxr_mult_scale_en), sizeof(cnco_inspect->dp_cfg.cmplx_mxr_mult_scale_en));
            ADI_APOLLO_ERROR_RETURN(err);

            err = bf_to_nco_profile_sel(device, regmap_base_addr, &(cnco_inspect->dp_cfg).nco_profile_sel_mode);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_get(device, BF_DRC_PROFILE_UPDATE_INDEX_INFO(regmap_base_addr), &(cnco_inspect->profile_num), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_NCO_REGMAP_CHAN_SEL_INFO(regmap_base_addr), &(cnco_inspect->active_profile_num), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_INC_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->dp_cfg.nco_phase_inc)), 4);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_OFFSET_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->dp_cfg.nco_phase_offset)), 4);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_INC_FRAC_A_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->dp_cfg.nco_phase_inc_frac_a)), 4);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_INC_FRAC_B_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->dp_cfg.nco_phase_inc_frac_b)), 4);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_get(device, BF_MIXER_TEST_MODE_VAL_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->dp_cfg.dc_testmode_value)), 2);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_get(device, BF_COARSE_DRC_EN_INFO(regmap_base_addr), (uint8_t*) &(cnco_inspect->dp_cfg.drc_en), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_CDRC_CLK_EN_INFO(regmap_base_addr), 
                (uint8_t *) &(cnco_inspect->dp_cfg.debug_cdrc_clkoff_n), sizeof(cnco_inspect->dp_cfg.debug_cdrc_clkoff_n));
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_get(device, BF_DRC_ACTIVE_PHASE_INC_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->active_phase_inc)), 4);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_ACTIVE_PHASE_OFFSET_INFO(regmap_base_addr), (uint8_t *) (&(cnco_inspect->active_phase_offset)), 4);
            ADI_APOLLO_ERROR_RETURN(err)

            err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_DITHER_EN_INFO(regmap_base_addr), &(cnco_inspect->drc_phase_dither_en), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_DRC_AMP_DITHER_EN_INFO(regmap_base_addr), &(cnco_inspect->drc_amp_dither_en), 1);
            ADI_APOLLO_ERROR_RETURN(err);

            for (j = 0U; j < (uint8_t)ADI_APOLLO_CNCO_PROFILE_NUM; j++) {
                err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), j);  // only bf in reg, use wr reg
                ADI_APOLLO_ERROR_RETURN(err);

                err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_INC_INFO(regmap_base_addr), (uint8_t *) &cnco_inspect->dp_cfg.nco_phase_inc_words[j], 4);
                ADI_APOLLO_ERROR_RETURN(err);

                err = adi_apollo_hal_bf_get(device, BF_DRC_PHASE_OFFSET_INFO(regmap_base_addr), (uint8_t*) &cnco_inspect->dp_cfg.nco_phase_offset_words[j], 4);
                ADI_APOLLO_ERROR_RETURN(err);
            }

            /* Return REG_DRC_PROFILE_UPDATE_CTRL_ADDR back to original state */
            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), cnco_inspect->profile_num);
            ADI_APOLLO_ERROR_RETURN(err);
            
            /* Only one inspect per call */
            break;
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_enable_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint8_t enable)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i ++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_COARSE_DRC_EN_INFO(regmap_base_addr), enable);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_mode_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, adi_apollo_nco_mixer_mode_e mode)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    /* Lower bound check (< ADI_APOLLO_MXR_VAR_IF_MODE) omitted - parameter is unsigned enum */
    ADI_APOLLO_INVALID_PARAM_RETURN(mode > ADI_APOLLO_MXR_TEST_MODE);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            /* Set reg_map base address for subsequent paged writes */
            err = adi_apollo_hal_paged_base_addr_set(device, regmap_base_addr);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_bf_set(device, BF_DRC_IF_MODE_TXRX_COARSE_NCO_INFO(regmap_base_addr), mode);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_test_mode_val_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint16_t val)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_MIXER_TEST_MODE_VAL_TXRX_COARSE_NCO_INFO(regmap_base_addr), val);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_ftw_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint8_t profile_num, uint8_t active_en, uint32_t ftw)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            /* Set reg_map base address for subsequent paged writes */
            err = adi_apollo_hal_paged_base_addr_set(device, regmap_base_addr);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Select profile to update */
            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), profile_num);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_bf_set(device, BF_DRC_PHASE_INC_INFO(regmap_base_addr), ftw);
            ADI_APOLLO_ERROR_RETURN(err);

            if (active_en != 0U) {
                err = adi_apollo_hal_paged_bf_set(device, BF_DRC_NCO_REGMAP_CHAN_SEL_INFO(regmap_base_addr), profile_num);
                ADI_APOLLO_ERROR_RETURN(err);
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_pow_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint8_t profile_num, uint8_t active_en, uint32_t pow)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            /* Select profile to update */
            err = adi_apollo_hal_paged_reg_set(device, REG_DRC_PROFILE_UPDATE_CTRL_ADDR(regmap_base_addr), profile_num);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_set(device, BF_DRC_PHASE_OFFSET_INFO(regmap_base_addr), pow);
            ADI_APOLLO_ERROR_RETURN(err);

            if (active_en != 0U) {
                err = adi_apollo_hal_bf_set(device, BF_DRC_NCO_REGMAP_CHAN_SEL_INFO(regmap_base_addr), profile_num);
                ADI_APOLLO_ERROR_RETURN(err);
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_mod_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint32_t frac_a, uint32_t frac_b)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN(frac_a >= frac_b && frac_a != 0U);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            /* Set reg_map base address for subsequent paged writes */
            err = adi_apollo_hal_paged_base_addr_set(device, regmap_base_addr);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_bf_set(device, BF_DRC_PHASE_INC_FRAC_A_INFO(regmap_base_addr), frac_a);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_bf_set(device, BF_DRC_PHASE_INC_FRAC_B_INFO(regmap_base_addr), frac_b);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_bf_set(device, BF_DRC_MOD_NCO_PROFILE_UPDATE_INFO(regmap_base_addr), 0);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_paged_bf_set(device, BF_DRC_MOD_NCO_PROFILE_UPDATE_INFO(regmap_base_addr), 1);
            ADI_APOLLO_ERROR_RETURN(err);

        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_mixer_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, adi_apollo_drc_mixer_sel_e mixer)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    /* Lower bound check (< ADI_APOLLO_DRC_MIXER_REAL) omitted - parameter is unsigned enum */
    ADI_APOLLO_INVALID_PARAM_RETURN(mixer > ADI_APOLLO_DRC_MIXER_COMPLEX);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DRC_MIXER_SEL_INFO(regmap_base_addr), (uint64_t)mixer);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_next_hop_num_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, int16_t hop_num)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN(hop_num > (int16_t)(ADI_APOLLO_CNCO_PROFILE_NUM - 1));
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_reg_set(device, REG_NEXT_HOP_NUMBER_ADDR(regmap_base_addr), (uint8_t)hop_num);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


int32_t adi_apollo_cnco_profile_sel_mode_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, adi_apollo_nco_profile_sel_mode_e profile_sel_mode)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    /* Lower bound check (< ADI_APOLLO_NCO_CHAN_SEL_TRIG_AUTO) omitted - parameter is unsigned enum */
    ADI_APOLLO_INVALID_PARAM_RETURN(profile_sel_mode > ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DRC_NCO_CHAN_SEL_MODE_INFO(regmap_base_addr), profile_sel_to_bf(profile_sel_mode));
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_active_profile_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t cncos, uint8_t profile_num)
{
    int32_t err;
    uint16_t i;
    adi_apollo_blk_sel_t cnco;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN(profile_num > (uint8_t)(ADI_APOLLO_CNCO_PROFILE_NUM - 1U));
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cncos);

    for (i = 0U; i < ADI_APOLLO_CNCO_NUM; i++) {
        cnco = cncos & (adi_apollo_blk_sel_t)((uint32_t)ADI_APOLLO_CNCO_A0 << i);
        if (cnco > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DRC_NCO_REGMAP_CHAN_SEL_INFO(regmap_base_addr), profile_num);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_cnco_active_profile_read(adi_apollo_device_t *device, adi_apollo_terminal_e terminal,
                adi_apollo_blk_sel_t cnco, uint8_t first,
                uint32_t phase_inc[], uint32_t phase_inc_len, uint32_t phase_offset[], uint32_t phase_off_len)
{
    int32_t err;
    uint16_t i;
    uint8_t profile_id;
    uint8_t saved_profile;
    uint32_t regmap_base_addr = 0U;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_NULL_POINTER_RETURN(phase_inc);
    ADI_APOLLO_NULL_POINTER_RETURN(phase_offset);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(phase_inc_len != phase_off_len);
    ADI_APOLLO_INVALID_PARAM_RETURN((first + phase_inc_len) > (uint32_t)ADI_APOLLO_CNCO_PROFILE_NUM);
    ADI_APOLLO_CNCO_BLK_SEL_MASK(cnco);

    for (i = 0; i < ADI_APOLLO_CNCO_NUM; i++) {
        if ((cnco & ((uint32_t)ADI_APOLLO_CNCO_A0 << i)) > 0U) {
            regmap_base_addr = (terminal == ADI_APOLLO_RX) ? calc_rx_cnco_base(i) : calc_tx_cnco_base(i);
            break;
        }
    }
    ADI_APOLLO_INVALID_PARAM_RETURN(regmap_base_addr == 0U);

    err = adi_apollo_hal_bf_get(device, BF_DRC_NCO_REGMAP_CHAN_SEL_INFO(regmap_base_addr), &saved_profile, 1U);
    ADI_APOLLO_ERROR_RETURN(err);

    for (profile_id = first; profile_id < (first + phase_inc_len); profile_id++) {
        err = adi_apollo_cnco_active_profile_set(device, terminal, cnco, profile_id);
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_get(device, BF_DRC_ACTIVE_PHASE_INC_INFO(regmap_base_addr), (uint8_t *)&phase_inc[profile_id - first], 4U);
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_get(device, BF_DRC_ACTIVE_PHASE_OFFSET_INFO(regmap_base_addr), (uint8_t *)&phase_offset[profile_id - first], 4U);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    err = adi_apollo_cnco_active_profile_set(device, terminal, cnco, saved_profile);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

uint32_t calc_tx_cnco_base(uint32_t idx)
{
    static const uint32_t tx_cnco_regmap[ADI_APOLLO_CNCO_NUM] = {
        TX_COARSE_NCO0_TX_SLICE_0_TX_DIGITAL0,
        TX_COARSE_NCO0_TX_SLICE_1_TX_DIGITAL0,
        TX_COARSE_NCO1_TX_SLICE_0_TX_DIGITAL0,
        TX_COARSE_NCO1_TX_SLICE_1_TX_DIGITAL0,

        TX_COARSE_NCO0_TX_SLICE_0_TX_DIGITAL1,
        TX_COARSE_NCO0_TX_SLICE_1_TX_DIGITAL1,
        TX_COARSE_NCO1_TX_SLICE_0_TX_DIGITAL1,
        TX_COARSE_NCO1_TX_SLICE_1_TX_DIGITAL1
    };

    return tx_cnco_regmap[idx];
}

uint32_t calc_rx_cnco_base(uint32_t idx)
{
    static uint32_t rx_coarse_nco_regmap[ADI_APOLLO_CNCO_NUM] = {
        RX_COARSE_NCO0_RX_SLICE_0_RX_DIGITAL0, RX_COARSE_NCO0_RX_SLICE_1_RX_DIGITAL0,
        RX_COARSE_NCO1_RX_SLICE_0_RX_DIGITAL0, RX_COARSE_NCO1_RX_SLICE_1_RX_DIGITAL0,
        RX_COARSE_NCO0_RX_SLICE_0_RX_DIGITAL1, RX_COARSE_NCO0_RX_SLICE_1_RX_DIGITAL1,
        RX_COARSE_NCO1_RX_SLICE_0_RX_DIGITAL1, RX_COARSE_NCO1_RX_SLICE_1_RX_DIGITAL1
    };
    return rx_coarse_nco_regmap[idx];
}

/* Convert generic profile select mode (i.e. used by both cnco and fnco) to cnco specific 'BF_DRC_NCO_CHAN_SEL_MODE_INFO' bf value */
static uint8_t profile_sel_to_bf(adi_apollo_nco_profile_sel_mode_e ps)
{
    static const uint8_t conv_table[] = {
        ADI_APOLLO_CNCO_CHAN_SEL_TRIGGER_BASED,
        ADI_APOLLO_CNCO_CHAN_SEL_TRIGGER_BASED,
        ADI_APOLLO_CNCO_CHAN_SEL_TRIGGER_BASED,
        ADI_APOLLO_CNCO_CHAN_SEL_DIRECT_GPIO,
        ADI_APOLLO_CNCO_CHAN_SEL_DIRECT_REGMAP};
    return (conv_table[ps]);
}

static int32_t bf_to_nco_profile_sel(adi_apollo_device_t *device, uint32_t regmap_base_addr, adi_apollo_nco_profile_sel_mode_e *ps)
{
    int32_t err;

    uint8_t drc_nco_chan_sel_mode;
    uint8_t profile_sel_mode;

    err = adi_apollo_hal_bf_get(device, BF_DRC_NCO_CHAN_SEL_MODE_INFO(regmap_base_addr), (uint8_t *)&drc_nco_chan_sel_mode, sizeof(drc_nco_chan_sel_mode));
    ADI_APOLLO_ERROR_RETURN(err);

    if (drc_nco_chan_sel_mode == (uint8_t)ADI_APOLLO_CNCO_CHAN_SEL_DIRECT_REGMAP) {
        *ps = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP;
    } else if (drc_nco_chan_sel_mode == (uint8_t)ADI_APOLLO_CNCO_CHAN_SEL_DIRECT_GPIO) {
        *ps = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO;
    } else if (drc_nco_chan_sel_mode == (uint8_t)ADI_APOLLO_CNCO_CHAN_SEL_TRIGGER_BASED) {
        err = adi_apollo_hal_bf_get(device, BF_PROFILE_SEL_MODE_TXRX_COARSE_NCO_INFO(regmap_base_addr), (uint8_t *)&profile_sel_mode, sizeof(profile_sel_mode));
        ADI_APOLLO_ERROR_RETURN(err);

        *ps = (adi_apollo_nco_profile_sel_mode_e)profile_sel_mode;
    } else {
        /* Invalid mode - should not reach here */
        ADI_CMS_RETURN(API_CMS_ERROR_INVALID_PARAM);
    }

    ADI_CMS_RETURN(err);
}

/* Convert generic profile select mode (i.e. used by both cnco and fnco) to cnco specific 'BF_PROFILE_SEL_MODE_TXRX_COARSE_NCO_INFO' bf value */
static uint8_t nco_profile_sel_to_profile_sel_mode_bf(adi_apollo_nco_profile_sel_mode_e ps)
{
    static const uint8_t conv_table[] = {
        ADI_APOLLO_CNCO_TRIG_PROF_SEL_AUTO,
        ADI_APOLLO_CNCO_TRIG_PROF_SEL_REGMAP,
        ADI_APOLLO_CNCO_TRIG_PROF_SEL_GPIO};
    return (conv_table[ps]);
}
/*! @} */