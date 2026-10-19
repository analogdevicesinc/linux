/*!
 * \brief     APIs for PFILT
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_PFILT
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_pfilt.h"
#include "adi_apollo_pfilt_local.h"
#include "adi_apollo_private_device.h"
#include "adi_apollo_private_blk_sel_types.h"

#include "adi_apollo_bf_txrx_pfilt_top.h"
#include "adi_apollo_bf_txrx_pfilt_coeff.h"
#include "adi_apollo_bf_custom.h"

#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"
#include "adi_utils.h"

static int32_t write_bank(adi_apollo_device_t *device, uint32_t bank_num, uint32_t regmap_base_addr, uint32_t first_idx, int16_t ceoffs[], uint32_t n_taps);

typedef struct
{
    uint16_t n_taps;
    uint16_t idx[ADI_APOLLO_PFILT_STREAMS_NUM];
    uint16_t idx_fmtx[ADI_APOLLO_PFILT_STREAMS_NUM];
} stream_mode_loc_map_t;

static const stream_mode_loc_map_t stream_mode_loc_map[] = {
    {32, {0, 16}, {0, 0}},      // N tap
    {16, {0, 16}, {0, 0}},      // N/2 tap
    { 8, {0, 16}, {0, 0}},      // N/4 tap
    {16, {0, 16}, {0, 0}},      // Half-Matrix
    { 8, {0, 16}, {8, 24}}      // Full-Matrix
};

/* Map pfilt mode enum to coeff layout for that mode */
static const int32_t pfilt_mode_to_idx[8] = {
    0, // bypass, n/a
    2, // n/4 tap
    1, // n/2 tap
    0, // n/a
    4, // full matrix
    0, // n/a
    3, // half-cmplx
    0  // n tap
};

/*==================== F O R W A R D   D E C L A R A T I O N S ====================*/
static uint32_t calc_pfilt_base(adi_apollo_terminal_e terminal, uint32_t pfilt_index);
static uint32_t calc_pfilt_coeff_base(adi_apollo_terminal_e terminal, uint32_t global_bank_base);

/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_pfilt_mode_pgm(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, adi_apollo_pfilt_mode_pgm_t *config)
{
    int32_t err;
    uint32_t i;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    /* Common bitfield for 0 & 1 pfilts */
    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i += 2) {                               
        pfilt = (uint16_t)(pfilts & (((uint32_t)ADI_APOLLO_PFILT_A0 << i) | ((uint32_t)ADI_APOLLO_PFILT_A0 << (i+1U))));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);

            err = adi_apollo_hal_bf_set(device, BF_QUAD_MODE_INFO(regmap_base_addr), (uint64_t)config->dq_mode);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_REAL_DATA_INFO(regmap_base_addr), (uint64_t)config->data);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_MODE_SWITCH_INFO(regmap_base_addr), (uint64_t)config->mode_switch);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_ADD_SUB_SEL_INFO(regmap_base_addr), (uint64_t)config->add_sub_sel);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    /* 
     * 4t4r: Program for n=0;
     * 8t8r ADC pair 0: Program for n=0;
     * 8t8r ADC pair 1: Program for n=1
     */    
    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);

            /* v10
             * A0 and B0 get i/q_mode[0]
             * A1 and B1 get i/q_mode[1]
             */
            uint32_t mode_index = i % 2U;
            err = adi_apollo_hal_bf_set(device, BF_PFIR_I_MODE_INFO(regmap_base_addr, mode_index), (uint64_t)config->pfir_i_mode[mode_index]);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_PFIR_Q_MODE_INFO(regmap_base_addr, mode_index), (uint64_t)config->pfir_q_mode[mode_index]);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_gain_dly_pgm(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t pfilt_banks, adi_apollo_pfilt_gain_dly_pgm_t *config)
{
    int32_t err;
    uint32_t i, j;
    uint8_t pfilt_bank, pfilt_local;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    /* 
     * for each of the pfilts (4 for 8T8R: A0/A1/B0/B1, 2 for 4T4R: A0/B0)
     *    for each of the 4 banks supported in each pfilt
     *        set the parameters gain/scalar-gain/delay
     *        
     * 4t4r: Program n=0 for 1st coeff bank;
     * 4t4r: Program n=1 for 2nd coeff bank;
     * 4t4r: Program n=2 for 3rd coeff bank;
     * 4t4r: Program n=3 for 4th coeff bank;
     * 
     * 8t8r: Program n=0 for 1st coeff bank of ADC pair 0;
     * 8t8r: Program n=1 for 2nd coeff bank of ADC pair 0;
     * 8t8r: Program n=2 for 3rd coeff bank of ADC pair 0;
     * 8t8r: Program n=3 for 4th coeff bank of ADC pair 0;
     * 8t8r: Program n=4 for 1st coeff bank of ADC pair 1;
     * 8t8r: Program n=5 for 2nd coeff bank of ADC pair 1;
     * 8t8r: Program n=6 for 3rd coeff bank of ADC pair 1;
     * 8t8r: Program n=7 for 4th coeff bank of ADC pair 1;
     */
    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            /* base of pfilt */
            regmap_base_addr = calc_pfilt_base(terminal, i);

            /* For each of the banks supported in a pfilt */
            for(j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j ++) {
                pfilt_bank = (uint8_t)(pfilt_banks & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << j));
                if (pfilt_bank > 0U) {
                    /* i: 0 to 3
                     * (i % ADI_APOLLO_PFILT_PER_SIDE): 0, 1, 0, 1
                     * (i % ADI_APOLLO_PFILT_PER_SIDE) * ADI_APOLLO_PFILT_BANK_NUM: 0, 4, 0, 4
                     * pfilt_local: 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 (the bank index)
                     * */
                    uint32_t per_side_index = i % ADI_APOLLO_PFILT_PER_SIDE;
                    uint32_t bank_offset = per_side_index * ADI_APOLLO_PFILT_BANK_NUM;
                    pfilt_local = (uint8_t)(bank_offset + j);

                    err = adi_apollo_hal_bf_set(device, BF_PFIR_IX_GAIN_INFO(regmap_base_addr, pfilt_local), (uint64_t)config->pfir_ix_gain);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = adi_apollo_hal_bf_set(device, BF_PFIR_IY_GAIN_INFO(regmap_base_addr, pfilt_local), (uint64_t)config->pfir_iy_gain);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = adi_apollo_hal_bf_set(device, BF_PFIR_QX_GAIN_INFO(regmap_base_addr, pfilt_local), (uint64_t)config->pfir_qx_gain);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = adi_apollo_hal_bf_set(device, BF_PFIR_QY_GAIN_INFO(regmap_base_addr, pfilt_local), (uint64_t)config->pfir_qy_gain);
                    ADI_APOLLO_ERROR_RETURN(err);

                    err = adi_apollo_hal_bf_set(device, BF_PFIR_IX_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), config->pfir_ix_scalar_gain);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = adi_apollo_hal_bf_set(device, BF_PFIR_IY_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), config->pfir_iy_scalar_gain);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = adi_apollo_hal_bf_set(device, BF_PFIR_QX_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), config->pfir_qx_scalar_gain);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = adi_apollo_hal_bf_set(device, BF_PFIR_QY_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), config->pfir_qy_scalar_gain);
                    ADI_APOLLO_ERROR_RETURN(err);

                    err = adi_apollo_hal_bf_set(device, BF_HC_PROG_DELAY_INFO(regmap_base_addr, pfilt_local), config->hc_delay);
                    ADI_APOLLO_ERROR_RETURN(err);
               }
            }
        }
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/*
 * This function programs a set of length coefficients into a BANK
 * 
 * For each side of the chip (A and B)
 *      For each of Tx and Rx     *      
 *          For 4T4R
 *              There is one PFIR called A0 on side A and one PFIR called B0 on side B
 *              The PFIR has 4 coefficients sets associated with it
 *              (on Side A, these are at A0Bank0: 0x60360000, A0Bank1: 0x60360040, A0Bank2: 0x60560000, A0Bank3: 0x60560040)
 *              (on Side B, these are at B0Bank0: 0x60B60000, B0Bank1: 0x60B60040, B0Bank2: 0x60D60000, B0Bank3: 0x60D60040)
 *          For 8T8R
 *              There is one additional PFIR called A1 on side A and one additional PFIR called B1 on side B
 *              This additional PFIR has 4 coefficients sets associated with it
 *              (e.g. on Side A, these are at A1Bank0: 0x60361000, A1Bank1: 0x60361040, A1Bank2: 0x60561000, A1Bank3: 0x60561040)
 *              (e.g. on Side B, these are at B1Bank0: 0x60B61000, B1Bank1: 0x60B61040, B1Bank2: 0x60D61000, B1Bank3: 0x60D61040)
 *              
 *          So, in summary:
 *          There are 4 sets of coefficients (4 x 32 x int16) on each side of the chip, for Tx and Rx separately for a 4T4R chip
 *          There are 8 sets of coefficients (8 x 32 x int16) on each side of the chip, for Tx and Rx separately for a 8T8R chip
 *          
 * This function gets these parameters:
 *      RX/Tx
 *      A 4 bit mask that indicates which of the A0/A1/B0/B1 coefficient sets are to be written to with the coefficients - pfilts is a 4 bit value: B1:B0:A1:A0
 *      pfilt_banks: This is a 4 bit field indicating which banks to write to within the PFIR (3:2:1:0) with the coefficients
 * */
int32_t adi_apollo_pfilt_coeff_pgm(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t pfilt_banks, int16_t pfilt_coeff[], uint32_t length)
{
    int32_t err;
    uint32_t i, j, k;
    uint8_t pfilt_bank;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(length > ADI_APOLLO_PFILT_COEFF_NUM);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    /* Loop through A0, A1, B0, B1 */
    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            /* Loop through the 4 banks per PFIR */
            for(j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j ++) {
                pfilt_bank = (uint8_t)(pfilt_banks & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << j));
                if (pfilt_bank > 0U) {
                    regmap_base_addr = calc_pfilt_coeff_base(terminal, (i * ADI_APOLLO_PFILT_BANK_NUM) + j);
                    if ((j % ADI_APOLLO_PFILT_BANKS_PER_REGMAP) == 0U) {               //BANK0 and BANK2
                        for(k = 0; k < length; k++) {
                            err = adi_apollo_hal_bf_set(device, BF_COEFF_0_INFO(regmap_base_addr, k), (uint64_t)pfilt_coeff[k]);
                            ADI_APOLLO_ERROR_RETURN(err);
                        }
                    }
                    else {                                                          //BANK1 and BANK3
                        for(k = 0; k < length; k++) {
                            err = adi_apollo_hal_bf_set(device, BF_COEFF_1_INFO(regmap_base_addr, k), (uint64_t)pfilt_coeff[k]);
                            ADI_APOLLO_ERROR_RETURN(err);
                        }
                    }
               }
            }
        }
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);

}

int32_t adi_apollo_pfilt_coeff_ntap_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t pfilt_banks,
                                        uint16_t streams, adi_apollo_pfilt_mode_e mode, int16_t coeffs[], uint32_t coeffs_len)
{
    int32_t err;
    uint32_t i, j, s;
    uint8_t pfilt_bank;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;
    stream_mode_loc_map_t stream_ceoff_loc;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs);
    ADI_APOLLO_INVALID_PARAM_RETURN(terminal > ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilts & (uint16_t)~ADI_APOLLO_PFILT_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilt_banks & (uint8_t)~ADI_APOLLO_PFILT_BANK_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN((streams & (uint16_t)~ADI_APOLLO_PFILT_STREAM_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN(!((mode == ADI_APOLLO_PFILT_MODE_N_REAL) || (mode == ADI_APOLLO_PFILT_MODE_N_DIV_BY_2_REAL) || (mode == ADI_APOLLO_PFILT_MODE_N_DIV_BY_4_REAL)));
    ADI_APOLLO_INVALID_PARAM_RETURN((mode == ADI_APOLLO_PFILT_MODE_N_REAL) && ((coeffs_len != 32U) || (adi_api_utils_num_selected(streams) != 1U)) );
    ADI_APOLLO_INVALID_PARAM_RETURN((mode == ADI_APOLLO_PFILT_MODE_N_DIV_BY_2_REAL) && (coeffs_len != 16U));
    ADI_APOLLO_INVALID_PARAM_RETURN((mode == ADI_APOLLO_PFILT_MODE_N_DIV_BY_4_REAL) && (coeffs_len != 8U));
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    /* Get coeff bank location info for given mode */
    stream_ceoff_loc = stream_mode_loc_map[pfilt_mode_to_idx[mode]];
    
    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            for (j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j++) {
                pfilt_bank = (uint8_t)(pfilt_banks & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << j));
                if (pfilt_bank > 0U) {
                    regmap_base_addr = calc_pfilt_coeff_base(terminal, (i * ADI_APOLLO_PFILT_BANK_NUM) + j);
                    for (s = 0U; s < ADI_APOLLO_PFILT_STREAMS_NUM; s++) {
                        if ((streams & (uint8_t)((uint32_t)ADI_APOLLO_PFILT_STREAM_0 << s)) != 0U) {
                            err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx[s], coeffs, stream_ceoff_loc.n_taps);
                            ADI_APOLLO_ERROR_RETURN(err);
                        }
                    }                    
                }
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_coeff_half_complex_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t pfilt_banks,
                                               int16_t coeffs0[], int16_t coeffs1[], uint32_t coeffs_len)
{
    int32_t err;
    uint32_t i, j;
    uint8_t pfilt_bank;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;
    stream_mode_loc_map_t stream_ceoff_loc;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs0);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs1);
    ADI_APOLLO_INVALID_PARAM_RETURN(terminal > ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilts & (uint16_t)~ADI_APOLLO_PFILT_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilt_banks & (uint8_t)~ADI_APOLLO_PFILT_BANK_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN(coeffs_len != 16U);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    /* Get coeff bank location info for half-complex */
    stream_ceoff_loc = stream_mode_loc_map[pfilt_mode_to_idx[ADI_APOLLO_PFILT_MODE_HALF_COMPLEX]];

    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            for (j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j++) {
                pfilt_bank = (uint8_t)(pfilt_banks & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << j));
                if (pfilt_bank > 0U) {
                    regmap_base_addr = calc_pfilt_coeff_base(terminal, (i * ADI_APOLLO_PFILT_BANK_NUM) + j);
                    err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx[0], coeffs0, stream_ceoff_loc.n_taps);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx[1], coeffs1, stream_ceoff_loc.n_taps);
                    ADI_APOLLO_ERROR_RETURN(err);
                }
            }
        }
    }
    
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_coeff_full_matrix_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t pfilt_banks,
                                               int16_t coeffs0[], int16_t coeffs1[], int16_t coeffs2[], int16_t coeffs3[], uint32_t coeffs_len)
{
    int32_t err;
    uint32_t i, j;
    uint8_t pfilt_bank;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;
    stream_mode_loc_map_t stream_ceoff_loc;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs0);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs1);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs2);
    ADI_APOLLO_NULL_POINTER_RETURN(coeffs3);
    ADI_APOLLO_INVALID_PARAM_RETURN(terminal > ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilts & (uint16_t)~ADI_APOLLO_PFILT_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilt_banks & (uint8_t)~ADI_APOLLO_PFILT_BANK_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN(coeffs_len != 8U);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    /* Get coeff bank locations info for full matrix */
    stream_ceoff_loc = stream_mode_loc_map[pfilt_mode_to_idx[ADI_APOLLO_PFILT_MODE_MATRIX]];

    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            for (j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j++) {
                pfilt_bank = (uint8_t)(pfilt_banks & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << j));
                if (pfilt_bank > 0U) {
                    regmap_base_addr = calc_pfilt_coeff_base(terminal, (i * ADI_APOLLO_PFILT_BANK_NUM) + j);
                    err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx[0], coeffs0, stream_ceoff_loc.n_taps);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx[1], coeffs1, stream_ceoff_loc.n_taps);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx_fmtx[0], coeffs2, stream_ceoff_loc.n_taps);
                    ADI_APOLLO_ERROR_RETURN(err);
                    err = write_bank(device, j, regmap_base_addr, stream_ceoff_loc.idx_fmtx[1], coeffs3, stream_ceoff_loc.n_taps);
                    ADI_APOLLO_ERROR_RETURN(err);
                }
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_half_complex_delay_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t pfilt_banks, uint8_t delay)
{
    int32_t err;
    uint32_t i, j;
    uint8_t pfilt_bank, pfilt_local;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(terminal > ADI_APOLLO_TX);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilts & (uint16_t)~ADI_APOLLO_PFILT_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN((pfilt_banks & (uint8_t)~ADI_APOLLO_PFILT_BANK_ALL) != 0U);
    ADI_APOLLO_INVALID_PARAM_RETURN(delay > 127U);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);

            for(j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j ++) {
                pfilt_bank = (uint8_t)(pfilt_banks & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << j));
                if (pfilt_bank > 0U) {
                    uint32_t per_side_idx = i % ADI_APOLLO_PFILT_PER_SIDE;
                    uint32_t bank_off = per_side_idx * ADI_APOLLO_PFILT_BANK_NUM;
                    pfilt_local = (uint8_t)(bank_off + j);

                    err = adi_apollo_hal_bf_set(device, BF_HC_PROG_DELAY_INFO(regmap_base_addr, pfilt_local), delay);
                    ADI_APOLLO_ERROR_RETURN(err);
               }
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_coeff_transfer(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t bank_sel)
{
    int32_t err;
    uint32_t i;
    uint8_t pfilt_bank;
    adi_apollo_blk_sel_t pfilt;
    uint8_t bank_number = 0;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(adi_api_utils_num_selected(bank_sel)!= 1U);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    for(i = 0U; i < ADI_APOLLO_PFILT_BANK_NUM; i ++) {
        pfilt_bank = (uint8_t)(bank_sel & ((uint32_t)ADI_APOLLO_PFILT_BANK0 << i));
        if (pfilt_bank > 0U) {
            bank_number = (uint8_t)i;
            break;
        }
    }

    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);
            uint8_t coeff_index = i % 2U;

            err = adi_apollo_hal_bf_set(device, BF_RD_COEFF_PAGE_SEL_INFO(regmap_base_addr, coeff_index), bank_number);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Generate 0->1 pulse to transfer selected back to active */
            err = adi_apollo_hal_bf_set(device, BF_PFIR_COEFF_TRANSFER_INFO(regmap_base_addr, coeff_index), 0);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_PFIR_COEFF_TRANSFER_INFO(regmap_base_addr, coeff_index), 1);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_inspect(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t  pfilt, adi_apollo_pfilt_inspect_t *pfilt_inspect)
{
    int32_t err;
    uint32_t i, j, k;
    uint8_t pfilt_local;
    adi_apollo_blk_sel_t pfilt_temp;
    uint32_t regmap_base_addr = 0;
    uint32_t regmap_coeff_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_NULL_POINTER_RETURN(pfilt_inspect);
    ADI_APOLLO_INVALID_PARAM_RETURN(adi_api_utils_num_selected(pfilt)!= 1U);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilt);

    /* pfilt_transfer_coeff_regmode: read the bank number*/
    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt_temp = (uint16_t)(pfilt & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt_temp > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);
            uint8_t coeff_idx = i % 2U;

            err = adi_apollo_hal_bf_get(device, BF_PFIR_COEFF_TRANSFER_INFO(regmap_base_addr, coeff_idx), (uint8_t*)&(pfilt_inspect->pfir_coeff_transfer), 1);
            ADI_APOLLO_ERROR_RETURN(err);

            err = adi_apollo_hal_bf_get(device, BF_RD_COEFF_PAGE_SEL_INFO(regmap_base_addr, coeff_idx), (uint8_t*)&(pfilt_inspect->bank_sel), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            break;
        }
    }


    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i+= 2U) {                               //Common bitfield for 0 & 1U pfilts
        pfilt_temp = (uint16_t)(pfilt & (((uint32_t)ADI_APOLLO_PFILT_A0 << i) | ((uint32_t)ADI_APOLLO_PFILT_A0 << (i+1U))));
        if (pfilt_temp > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);

            /* pfilt_mode_pgm and pfilt_pgm*/
            err = adi_apollo_hal_bf_get(device, BF_QUAD_MODE_INFO(regmap_base_addr), (uint8_t*) &(pfilt_inspect->dp_cfg.dq_mode), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_REAL_DATA_INFO(regmap_base_addr), (uint8_t*) &(pfilt_inspect->dp_cfg.real_data), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_MODE_SWITCH_INFO(regmap_base_addr), (uint8_t*) &(pfilt_inspect->dp_cfg.mode_switch), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_ADD_SUB_SEL_INFO(regmap_base_addr), (uint8_t*) &(pfilt_inspect->dp_cfg.add_sub_sel), 1);
            ADI_APOLLO_ERROR_RETURN(err);

            /* pfilt_setup_gpio_transfer */
            err = adi_apollo_hal_bf_get(device, BF_EQ_GPIO_EN_INFO(regmap_base_addr), (uint8_t*) &(pfilt_inspect->eq_gpio_sel), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_GPIO_CONFIG1_INFO(regmap_base_addr), (uint8_t*) &(pfilt_inspect->gpio_config1), 1);
            ADI_APOLLO_ERROR_RETURN(err);

            break;
        }
    }

    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt_temp = (uint16_t)(pfilt & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt_temp > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);
            uint8_t mode_idx = i % 2U;

            err = adi_apollo_hal_bf_get(device, BF_PFIR_I_MODE_INFO(regmap_base_addr, mode_idx), (uint8_t*) &(pfilt_inspect->dp_cfg.i_mode[mode_idx]), 1);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_PFIR_Q_MODE_INFO(regmap_base_addr, mode_idx), (uint8_t*) &(pfilt_inspect->dp_cfg.q_mode[mode_idx]), 1);
            ADI_APOLLO_ERROR_RETURN(err);

            for (j = 0U; j < ADI_APOLLO_PFILT_BANK_NUM; j++) {
                /* pfilt_gain_dly_pgm */
                uint32_t side_idx = i % ADI_APOLLO_PFILT_PER_SIDE;
                uint32_t bank_base = side_idx * ADI_APOLLO_PFILT_BANK_NUM;
                pfilt_local = (uint8_t)(bank_base + j);

                err = adi_apollo_hal_bf_get(device, BF_PFIR_IX_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_ix_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_get(device, BF_PFIR_IY_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_iy_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_get(device, BF_PFIR_QX_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_qx_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_get(device, BF_PFIR_QY_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_qy_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);

                err = adi_apollo_hal_bf_get(device, BF_PFIR_IX_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_ix_scalar_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_get(device, BF_PFIR_IY_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_iy_scalar_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_get(device, BF_PFIR_QX_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_qx_scalar_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);
                err = adi_apollo_hal_bf_get(device, BF_PFIR_QY_SCALAR_GAIN_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.pfir_qy_scalar_gain_db[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);

                err = adi_apollo_hal_bf_get(device, BF_HC_PROG_DELAY_INFO(regmap_base_addr, pfilt_local), (uint8_t*) &(pfilt_inspect->dp_cfg.hc_prog_delay[j]), 1);
                ADI_APOLLO_ERROR_RETURN(err);

                /* pfilt_coeff_pgm */
                regmap_coeff_base_addr = calc_pfilt_coeff_base(terminal, (i * ADI_APOLLO_PFILT_BANK_NUM) + j);
                if ((j % ADI_APOLLO_PFILT_BANKS_PER_REGMAP) == 0U) {               //BANK0 and BANK2
                    for (k = 0U; k < ADI_APOLLO_PFILT_COEFF_NUM; k++) {
                        err = adi_apollo_hal_bf_get(device, BF_COEFF_0_INFO(regmap_coeff_base_addr, k), (uint8_t*) &(pfilt_inspect->dp_cfg.coeffs[j][k]), 2);
                        ADI_APOLLO_ERROR_RETURN(err);
                    }
                }
                else {                                                          //BANK1 and BANK3
                    for (k = 0U; k < ADI_APOLLO_PFILT_COEFF_NUM; k++) {
                        err = adi_apollo_hal_bf_get(device, BF_COEFF_1_INFO(regmap_coeff_base_addr, k), (uint8_t*) &(pfilt_inspect->dp_cfg.coeffs[j][k]), 2);
                        ADI_APOLLO_ERROR_RETURN(err);
                    }
                }

            }

            break;
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_mode_enable_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, uint8_t iq_sel, adi_apollo_pfilt_mode_e mode)
{
    int32_t err;
    uint32_t i;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);
            uint8_t stream_idx = i % 2U;

            if ((iq_sel & (uint8_t)ADI_APOLLO_PFILT_STREAM_0) != 0U) {
                err = adi_apollo_hal_bf_set(device, BF_PFIR_I_MODE_INFO(regmap_base_addr, stream_idx), (uint64_t)mode);
                ADI_APOLLO_ERROR_RETURN(err);
            }
            if ((iq_sel & (uint8_t)ADI_APOLLO_PFILT_STREAM_1) != 0U) {
                err = adi_apollo_hal_bf_set(device, BF_PFIR_Q_MODE_INFO(regmap_base_addr, stream_idx), (uint64_t)mode);
                ADI_APOLLO_ERROR_RETURN(err);
            }
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_profile_sel_mode_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, const adi_apollo_pfilt_profile_sel_mode_e prof_sel_mode)
{
    int32_t err;
    uint32_t i;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(prof_sel_mode > (ADI_APOLLO_PFILT_CHAN_SEL_NUM-1));
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    const uint8_t bank_regs[ADI_APOLLO_PFILT_CHAN_SEL_NUM][3] = {
        {0, 0, 0},          // BF_PFILT_TRIGGER_EN_INFO, BF_EQ_GPIO_EN_INFO, BF_GPIO_CONFIG1_INFO
        {0, 1, 0},
        {0, 1, 1},
        {1, 0, 0},
        {1, 1, 0},
        {1, 1, 1}
    };

    for(i = 0U; i < ADI_APOLLO_PFILT_NUM; i ++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);

            err = adi_apollo_hal_bf_set(device, BF_PFILT_TRIGGER_EN_INFO(regmap_base_addr), bank_regs[prof_sel_mode][0]);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_EQ_GPIO_EN_INFO(regmap_base_addr), bank_regs[prof_sel_mode][1]);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_GPIO_CONFIG1_INFO(regmap_base_addr), bank_regs[prof_sel_mode][2]);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_next_hop_num_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, int16_t hop_num)
{
    int32_t err;
    uint32_t i;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(terminal, ADI_APOLLO_RX, ADI_APOLLO_TX);
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(hop_num > (int16_t)(ADI_APOLLO_PFILT_BANK_NUM - 1U));
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);
            uint8_t hop_idx = i % 2U;

            err = adi_apollo_hal_bf_set(device, BF_RD_COEFF_PAGE_SEL_INFO(regmap_base_addr, hop_idx), (uint64_t)hop_num);   // Next active coeff bank on trig event
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_ave_mode_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t pfilts, adi_apollo_pfilt_ave_mode_e ave_mode)
{
    int32_t err;
    uint32_t i;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(ADI_APOLLO_RX, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(ave_mode >= ADI_APOLLO_PFILT_AVE_NUM);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(ADI_APOLLO_RX, i);

            err = adi_apollo_hal_bf_set(device, BF_MODE_SWITCH_INFO(regmap_base_addr), ave_mode != ADI_APOLLO_PFILT_AVE_DISABLE);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_ADD_SUB_SEL_INFO(regmap_base_addr), ave_mode == ADI_APOLLO_PFILT_AVE_ENABLE_ADD);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_QUAD_MODE_INFO(regmap_base_addr), device->dev_info.is_8t8r ? 1 : 0);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_pfilt_data_type_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t pfilts, adi_apollo_pfilt_data_e data_type)
{
    int32_t err;
    uint32_t i;
    adi_apollo_blk_sel_t pfilt;
    uint32_t regmap_base_addr = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(terminal, ADI_APOLLO_EC_PFILT_LOCK);
    ADI_APOLLO_INVALID_PARAM_RETURN(data_type > ADI_APOLLO_PFILT_REAL_DATA);
    ADI_APOLLO_PFILT_BLK_SEL_MASK(pfilts);

    for (i = 0U; i < ADI_APOLLO_PFILT_NUM; i++) {
        pfilt = (uint16_t)(pfilts & ((uint32_t)ADI_APOLLO_PFILT_A0 << i));
        if (pfilt > 0U) {
            regmap_base_addr = calc_pfilt_base(terminal, i);

            err = adi_apollo_hal_bf_set(device, BF_REAL_DATA_INFO(regmap_base_addr), (uint64_t)data_type);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static uint32_t calc_pfilt_coeff_base(adi_apollo_terminal_e terminal, uint32_t global_bank_base)
{
    static uint32_t rx_pfilt_coeff_regmap[ADI_APOLLO_PFILT_NUM * ADI_APOLLO_PFILT_BANK_NUM] = {
        RX_PFILT_COEFF0_RX_SLICE_0_RX_DIGITAL0, RX_PFILT_COEFF0_RX_SLICE_0_RX_DIGITAL0, RX_PFILT_COEFF0_RX_SLICE_1_RX_DIGITAL0, RX_PFILT_COEFF0_RX_SLICE_1_RX_DIGITAL0,
        RX_PFILT_COEFF1_RX_SLICE_0_RX_DIGITAL0, RX_PFILT_COEFF1_RX_SLICE_0_RX_DIGITAL0, RX_PFILT_COEFF1_RX_SLICE_1_RX_DIGITAL0, RX_PFILT_COEFF1_RX_SLICE_1_RX_DIGITAL0,
        RX_PFILT_COEFF0_RX_SLICE_0_RX_DIGITAL1, RX_PFILT_COEFF0_RX_SLICE_0_RX_DIGITAL1, RX_PFILT_COEFF0_RX_SLICE_1_RX_DIGITAL1, RX_PFILT_COEFF0_RX_SLICE_1_RX_DIGITAL1,
        RX_PFILT_COEFF1_RX_SLICE_0_RX_DIGITAL1, RX_PFILT_COEFF1_RX_SLICE_0_RX_DIGITAL1, RX_PFILT_COEFF1_RX_SLICE_1_RX_DIGITAL1, RX_PFILT_COEFF1_RX_SLICE_1_RX_DIGITAL1,
    };
    static uint32_t tx_pfilt_coeff_regmap[ADI_APOLLO_PFILT_NUM * ADI_APOLLO_PFILT_BANK_NUM] = {
        TX_PFILT_COEFF0_TX_SLICE_0_TX_DIGITAL0, TX_PFILT_COEFF0_TX_SLICE_0_TX_DIGITAL0, TX_PFILT_COEFF0_TX_SLICE_1_TX_DIGITAL0, TX_PFILT_COEFF0_TX_SLICE_1_TX_DIGITAL0,
        TX_PFILT_COEFF1_TX_SLICE_0_TX_DIGITAL0, TX_PFILT_COEFF1_TX_SLICE_0_TX_DIGITAL0, TX_PFILT_COEFF1_TX_SLICE_1_TX_DIGITAL0, TX_PFILT_COEFF1_TX_SLICE_1_TX_DIGITAL0,
        TX_PFILT_COEFF0_TX_SLICE_0_TX_DIGITAL1, TX_PFILT_COEFF0_TX_SLICE_0_TX_DIGITAL1, TX_PFILT_COEFF0_TX_SLICE_1_TX_DIGITAL1, TX_PFILT_COEFF0_TX_SLICE_1_TX_DIGITAL1,
        TX_PFILT_COEFF1_TX_SLICE_0_TX_DIGITAL1, TX_PFILT_COEFF1_TX_SLICE_0_TX_DIGITAL1, TX_PFILT_COEFF1_TX_SLICE_1_TX_DIGITAL1, TX_PFILT_COEFF1_TX_SLICE_1_TX_DIGITAL1,
    };
    if (terminal == ADI_APOLLO_RX) {
        return rx_pfilt_coeff_regmap[global_bank_base];
    } else {
        return tx_pfilt_coeff_regmap[global_bank_base];
    }
}

static uint32_t calc_pfilt_base(adi_apollo_terminal_e terminal, uint32_t pfilt_index)
{
    static uint32_t rx_pfilt_regmap[ADI_APOLLO_PFILT_NUM] = {
        RX_PFILT_TOP_RX_SLICE_0_RX_DIGITAL0, RX_PFILT_TOP_RX_SLICE_0_RX_DIGITAL0,
        RX_PFILT_TOP_RX_SLICE_0_RX_DIGITAL1, RX_PFILT_TOP_RX_SLICE_0_RX_DIGITAL1
    };
    static uint32_t tx_pfilt_regmap[ADI_APOLLO_PFILT_NUM] = {
        TX_PFILT_TOP_TX_SLICE_0_TX_DIGITAL0, TX_PFILT_TOP_TX_SLICE_0_TX_DIGITAL0,
        TX_PFILT_TOP_TX_SLICE_0_TX_DIGITAL1, TX_PFILT_TOP_TX_SLICE_0_TX_DIGITAL1
    };
    if (terminal == ADI_APOLLO_RX) {
        return rx_pfilt_regmap[pfilt_index];
    } else {
        return tx_pfilt_regmap[pfilt_index];
    }
}

static int32_t write_bank(adi_apollo_device_t *device, uint32_t bank_num, uint32_t regmap_base_addr, uint32_t first_idx, int16_t ceoffs[], uint32_t n_taps)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;
    uint32_t i;
    
    if (((bank_num % ADI_APOLLO_PFILT_BANKS_PER_REGMAP) == 0U)) { //BANK0 and BANK2
        for (i = 0; i < n_taps; i++) {
            err = adi_apollo_hal_bf_set(device, BF_COEFF_0_INFO(regmap_base_addr, (i + first_idx) % ADI_APOLLO_PFILT_COEFF_NUM), (uint64_t)ceoffs[i]);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    } else { //BANK1 and BANK3
        for (i = 0; i < n_taps; i++) {
            err = adi_apollo_hal_bf_set(device, BF_COEFF_1_INFO(regmap_base_addr, (i + first_idx) % ADI_APOLLO_PFILT_COEFF_NUM), (uint64_t)ceoffs[i]);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }
    
    ADI_CMS_RETURN(err);
}
/*! @} */