/*!
 * \brief     Clock and multi chip sync API implementation
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_CLK_MCS
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"

#include "adi_apollo_clk_mcs.h"
#include "adi_apollo_bf_rx_datin.h"
#include "adi_apollo_bf_tx_hsdout.h"
#include "adi_apollo_bf_mcs_sync.h"
#include "adi_apollo_bf_mcs_tdc.h"
#include "adi_apollo_bf_rtclk_gen.h"
#include "adi_apollo_bf_custom.h"

static uint32_t calc_rx_datin_base_from_adc_index(uint32_t idx);
static uint32_t calc_tx_hsdout_base(uint32_t index);
static uint32_t calc_mcs_tdc_top_dual_clk_base(uint32_t idx);
static void get_adc_fifo_offsets(adi_apollo_data_fifo_conv_clk_ratio_e clk_ratio, uint8_t is_8t8r, uint8_t *wr_offset, uint8_t *rd_offset);
static void get_dac_fifo_offsets(adi_apollo_data_fifo_conv_clk_ratio_e clk_ratio, uint8_t is_8t8r, uint8_t *wr_offset, uint8_t *rd_offset);
static int32_t mask_sync(adi_apollo_device_t *device, uint32_t reg, uint32_t info, uint64_t filter, uint64_t value);
static int32_t rxtxlink_sync(adi_apollo_device_t *device, adi_apollo_sync_mask_rxtx_link_e rxtx_link_mask);
static int32_t clk_pd_status_get(adi_apollo_device_t *device, uint32_t top_base, adi_apollo_clk_input_power_status_e *status);


#define IS_DUAL_CLK(device) ((device)->dev_info.is_dual_clk)

/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_clk_mcs_rx_data_fifo_configure(adi_apollo_device_t *device, const uint16_t adcs, adi_apollo_data_fifo_config_t *fifo_config)
{
    int32_t err;
    uint8_t i;
    uint16_t adc;
    uint32_t regmap_base_addr;
    uint8_t lat_wr;
    uint8_t lat_rd;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(fifo_config);
    ADI_APOLLO_INVALID_PARAM_RETURN(fifo_config->enable > 1U);
    ADI_APOLLO_INVALID_PARAM_RETURN(fifo_config->mode > ADI_APOLLO_DATA_FIFO_MODE_FORCE_ON);
    if (fifo_config->lat_override != 0U) {
        ADI_APOLLO_INVALID_PARAM_RETURN(fifo_config->lat_wr_offset > 0xFU);
        ADI_APOLLO_INVALID_PARAM_RETURN(fifo_config->lat_rd_offset > 0x7U);
    }

    for(i = 0; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (uint16_t)(adcs & ((uint32_t)ADI_APOLLO_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_rx_datin_base_from_adc_index(i);

            /* Enable */
            err = adi_apollo_hal_bf_set(device, BF_DFIFO_EN_RX_DATIN_INFO(regmap_base_addr), fifo_config->enable);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Mode */
            err = adi_apollo_hal_bf_set(device, BF_LAT_PGM_MODE_RX_DATIN_INFO(regmap_base_addr), (uint64_t)fifo_config->mode);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Offsets - Use defaults unless override specified */
            if (fifo_config->lat_override != 0U) {
                lat_wr = fifo_config->lat_wr_offset;
                lat_rd = fifo_config->lat_rd_offset;
            } else {
                get_adc_fifo_offsets(fifo_config->conv_clk_ratio, device->dev_info.is_8t8r, &lat_wr, &lat_rd);
            }

            err = adi_apollo_hal_bf_set(device, BF_LAT_WR_SPI_RX_DATIN_INFO(regmap_base_addr), lat_wr);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_LAT_RD_SPI_RX_DATIN_INFO(regmap_base_addr), lat_rd);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_rx_data_fifo_enable(adi_apollo_device_t *device, const uint16_t adcs, uint8_t enable)
{
    int32_t err;
    uint8_t i;
    uint16_t adc;
    uint32_t regmap_base_addr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(enable > 1U);

    for(i = 0; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (uint16_t)(adcs & ((uint32_t)ADI_APOLLO_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_rx_datin_base_from_adc_index(i);

            err = adi_apollo_hal_bf_set(device, BF_DFIFO_EN_RX_DATIN_INFO(regmap_base_addr), enable);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_rx_data_fifo_offset_set(adi_apollo_device_t *device, const uint16_t adcs, uint8_t wr_offset, uint8_t rd_offset)
{
    int32_t err;
    uint8_t i;
    uint16_t adc;
    uint32_t regmap_base_addr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(wr_offset > 0xFU);
    ADI_APOLLO_INVALID_PARAM_RETURN(rd_offset > 0x7U);

    for(i = 0; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (uint16_t)(adcs & ((uint32_t)ADI_APOLLO_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_rx_datin_base_from_adc_index(i);

            err = adi_apollo_hal_bf_set(device, BF_LAT_WR_SPI_RX_DATIN_INFO(regmap_base_addr), wr_offset);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_LAT_RD_SPI_RX_DATIN_INFO(regmap_base_addr), rd_offset);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_rx_data_fifo_mode_set(adi_apollo_device_t *device, const uint16_t adcs, adi_apollo_data_fifo_mode_e mode)
{
    int32_t err;
    uint8_t i;
    uint16_t adc;
    uint32_t regmap_base_addr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(mode > ADI_APOLLO_DATA_FIFO_MODE_FORCE_ON);

    for(i = 0; i < ADI_APOLLO_ADC_NUM; i ++) {
        adc = (uint16_t)(adcs & ((uint32_t)ADI_APOLLO_ADC_A0 << i));
        if (adc > 0U) {
            regmap_base_addr = calc_rx_datin_base_from_adc_index(i);

            err = adi_apollo_hal_bf_set(device, BF_LAT_PGM_MODE_RX_DATIN_INFO(regmap_base_addr), (uint64_t)mode);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_tx_data_fifo_configure(adi_apollo_device_t *device, const uint16_t dacs, adi_apollo_data_fifo_config_t *fifo_config)
{
    int32_t err;
    uint8_t i;
    uint16_t dac;
    uint32_t regmap_base_addr;
    uint8_t lat_wr;
    uint8_t lat_rd;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(fifo_config);

    for(i = 0; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (uint16_t)(dacs & ((uint32_t)ADI_APOLLO_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_hsdout_base(i);

            /* Enable */
            err = adi_apollo_hal_bf_set(device, BF_DFIFO_EN_TX_HSDOUT_INFO(regmap_base_addr), fifo_config->enable);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Mode */
            err = adi_apollo_hal_bf_set(device, BF_LAT_PGM_MODE_TX_HSDOUT_INFO(regmap_base_addr), (uint64_t)fifo_config->mode);
            ADI_APOLLO_ERROR_RETURN(err);

            /* Offsets - Use defaults unless override specified */
            if (fifo_config->lat_override != 0U) {
                lat_wr = fifo_config->lat_wr_offset;
                lat_rd = fifo_config->lat_rd_offset;
            } else {
                get_dac_fifo_offsets(fifo_config->conv_clk_ratio, device->dev_info.is_8t8r, &lat_wr, &lat_rd);
            }

            err = adi_apollo_hal_bf_set(device, BF_LAT_WR_SPI_TX_HSDOUT_INFO(regmap_base_addr), lat_wr);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_LAT_RD_SPI_TX_HSDOUT_INFO(regmap_base_addr), lat_rd);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_tx_data_fifo_enable(adi_apollo_device_t *device, const uint16_t dacs, uint8_t enable)
{
    int32_t err;
    uint8_t i;
    uint16_t dac;
    uint32_t regmap_base_addr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    for(i = 0; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (uint16_t)(dacs & ((uint32_t)ADI_APOLLO_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_hsdout_base(i);

            err = adi_apollo_hal_bf_set(device, BF_DFIFO_EN_TX_HSDOUT_INFO(regmap_base_addr), enable);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_tx_data_fifo_offset_set(adi_apollo_device_t *device, const uint16_t dacs, uint8_t wr_offset, uint8_t rd_offset)
{
    int32_t err;
    uint8_t i;
    uint16_t dac;
    uint32_t regmap_base_addr;

    for(i = 0; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (uint16_t)(dacs & ((uint32_t)ADI_APOLLO_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_hsdout_base(i);

            err = adi_apollo_hal_bf_set(device, BF_LAT_WR_SPI_TX_HSDOUT_INFO(regmap_base_addr), wr_offset);
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_set(device, BF_LAT_RD_SPI_TX_HSDOUT_INFO(regmap_base_addr), rd_offset);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_tx_data_fifo_mode_set(adi_apollo_device_t *device, const uint16_t dacs, adi_apollo_data_fifo_mode_e mode)
{
    int32_t err;
    uint8_t i;
    uint16_t dac;
    uint32_t regmap_base_addr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_RANGE_CHECK(mode, ADI_APOLLO_DATA_FIFO_MODE_NON_DL, ADI_APOLLO_DATA_FIFO_MODE_FORCE_ON);

    for(i = 0; i < ADI_APOLLO_DAC_NUM; i ++) {
        dac = (uint16_t)(dacs & ((uint32_t)ADI_APOLLO_DAC_A0 << i));
        if (dac > 0U) {
            regmap_base_addr = calc_tx_hsdout_base(i);

            err = adi_apollo_hal_bf_set(device, BF_LAT_PGM_MODE_TX_HSDOUT_INFO(regmap_base_addr), 1);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sync_hw_align_set(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    // Clear Rx-Tx digital mask to be synced
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RXTX_ADDR, (uint8_t)ADI_APOLLO_SYNC_MASK_RX_TX_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    // Set Rx-Tx serdes link mask to avoid getting synced
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RXTXLINK_ADDR, (uint8_t)ADI_APOLLO_SYNC_MASK_RXTX_LINK_ALL);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_clk_mcs_oneshot_sync(device);
    ADI_APOLLO_ERROR_RETURN(err);

    // Clear JTx/JRx link masks
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RXTXLINK_ADDR, (uint8_t)ADI_APOLLO_SYNC_MASK_RXTX_LINK_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sync_hw_align_get(adi_apollo_device_t *device, uint8_t *status)
{
    int32_t err;
    uint8_t data[2];

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(status);

    *status = 0;

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_ONESHOT_SYNC_DONE_INFO(MCS_SYNC_MCSTOP1), &data[0], 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_ONESHOT_SYNC_DONE_INFO(MCS_SYNC_MCSTOP2), &data[1], 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status = data[0] | (uint8_t)((data[1]) << 1);
    } else {
        err = adi_apollo_hal_bf_get(device, BF_ONESHOT_SYNC_DONE_INFO(MCS_SYNC_MCSTOP0), &data[0], 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status = data[0];
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sync_only_set(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_DYN_CFG_SYNC_INFO(MCS_SYNC_MCSTOP1), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_DYN_CFG_SYNC_INFO(MCS_SYNC_MCSTOP2), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_DYN_CFG_SYNC_INFO(MCS_SYNC_MCSTOP0), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sync_only_get(adi_apollo_device_t *device, uint8_t *status)
{
    int32_t err;
    uint8_t data[2];

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(status);

    *status = 0;

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_DYN_CFG_SYNC_DONE_INFO(MCS_SYNC_MCSTOP1), &data[0], 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_DYN_CFG_SYNC_DONE_INFO(MCS_SYNC_MCSTOP2), &data[1], 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status = data[0] | (uint8_t)((data[1]) << 1);
    } else {
        err = adi_apollo_hal_bf_get(device, BF_DYN_CFG_SYNC_DONE_INFO(MCS_SYNC_MCSTOP0), &data[0], 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status = data[0];
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_trig_sync_enable(adi_apollo_device_t *device, uint8_t enable)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_TRIGGER_SYNC_INFO(MCS_SYNC_MCSTOP1), enable);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_TRIGGER_SYNC_INFO(MCS_SYNC_MCSTOP2), enable);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_TRIGGER_SYNC_INFO(MCS_SYNC_MCSTOP0), enable);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_trig_sync_enable_get(adi_apollo_device_t *device, uint8_t *enable)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(enable)

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_TRIGGER_SYNC_INFO(MCS_SYNC_MCSTOP1), enable, 1);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_get(device, BF_TRIGGER_SYNC_INFO(MCS_SYNC_MCSTOP0), enable, 1);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_trig_reset_dsp_enable(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP1), 0x00c0);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP2), 0x00c0);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP0), 0x00c0);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_trig_reset_disable(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP1), 0x0000);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP2), 0x0000);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP0), 0x0000);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_trig_phase_get(adi_apollo_device_t *device, adi_apollo_trig_pin_e trig, uint16_t *phase_0, uint16_t *phase_1)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(phase_0);
    ADI_APOLLO_NULL_POINTER_RETURN(phase_1);
    ADI_APOLLO_INVALID_PARAM_RETURN(trig >= ADI_APOLLO_TRIG_PIN_NUM);

    *phase_0 = 0;
    *phase_1 = 0;

    switch (trig) {
    case ADI_APOLLO_TRIG_PIN_A0:
    {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_A0_INFO(MCS_SYNC_MCSTOP1), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_A0_INFO(MCS_SYNC_MCSTOP2), (uint8_t*)phase_1, (uint8_t)sizeof(*phase_1));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        else {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_A0_INFO(MCS_SYNC_MCSTOP0), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        break;
    }
    case ADI_APOLLO_TRIG_PIN_A1:
    {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_A1_INFO(MCS_SYNC_MCSTOP1), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_A1_INFO(MCS_SYNC_MCSTOP2), (uint8_t*)phase_1, (uint8_t)sizeof(*phase_1));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        else {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_A1_INFO(MCS_SYNC_MCSTOP0), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        break;
    }
    case ADI_APOLLO_TRIG_PIN_B0:
    {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_B0_INFO(MCS_SYNC_MCSTOP1), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_B0_INFO(MCS_SYNC_MCSTOP2), (uint8_t*)phase_1, (uint8_t)sizeof(*phase_1));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        else {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_B0_INFO(MCS_SYNC_MCSTOP0), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        break;
    }
    case ADI_APOLLO_TRIG_PIN_B1:
    {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_B1_INFO(MCS_SYNC_MCSTOP1), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_B1_INFO(MCS_SYNC_MCSTOP2), (uint8_t*)phase_1, (uint8_t)sizeof(*phase_1));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        else {
            err = adi_apollo_hal_bf_get(device, BF_TRIG_PHASE_B1_INFO(MCS_SYNC_MCSTOP0), (uint8_t*)phase_0, (uint8_t)sizeof(*phase_0));
            ADI_APOLLO_ERROR_RETURN(err);
        }
        break;
    }
    case ADI_APOLLO_TRIG_PIN_NUM:
        // Sentinel value - not a valid pin
        err = (int32_t)API_CMS_ERROR_INVALID_PARAM;
        break;

    default:
        err = (int32_t)API_CMS_ERROR_INVALID_PARAM;
        break;
    }

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_clk_mcs_dig_path_reset_mask_set(adi_apollo_device_t *device, adi_apollo_sync_mask_rxtx_e rxtx_mask)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_RXTX_INFO, (uint64_t)(~(uint16_t)rxtx_mask));
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_dig_path_reset_disable(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_RXTX_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_RX_TX_ALL);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_rxtx_link_reset_disable(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_RXTXLINK_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_RXTX_LINK_ALL);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_lpbk_fifo_reset_disable(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_LPBKFIFO_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_LPBKFIFO_ALL);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


int32_t adi_apollo_clk_mcs_oneshot_sync(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_ONESHOT_SYNC_INFO(MCS_SYNC_MCSTOP1), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device,  BF_ONESHOT_SYNC_DONE_INFO(MCS_SYNC_MCSTOP1), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "EAST oneshot sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);

        err = adi_apollo_hal_bf_set(device, BF_ONESHOT_SYNC_INFO(MCS_SYNC_MCSTOP2), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device,  BF_ONESHOT_SYNC_DONE_INFO(MCS_SYNC_MCSTOP2), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "WEST oneshot sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_ONESHOT_SYNC_INFO(MCS_SYNC_MCSTOP0), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device,  BF_ONESHOT_SYNC_DONE_INFO(MCS_SYNC_MCSTOP0), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "CENTER oneshot sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_dynamic_sync(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_DYN_CFG_SYNC_INFO(MCS_SYNC_MCSTOP1), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device, BF_DYN_CFG_SYNC_DONE_INFO(MCS_SYNC_MCSTOP1), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "EAST dynamic sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);

        err = adi_apollo_hal_bf_set(device, BF_DYN_CFG_SYNC_INFO(MCS_SYNC_MCSTOP2), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device, BF_DYN_CFG_SYNC_DONE_INFO(MCS_SYNC_MCSTOP2), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "WEST dynamic sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_DYN_CFG_SYNC_INFO(MCS_SYNC_MCSTOP0), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device, BF_DYN_CFG_SYNC_DONE_INFO(MCS_SYNC_MCSTOP0), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "CENTER dynamic sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_man_reconfig_sync(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_MANUAL_SYNC_INFO(MCS_SYNC_MCSTOP1), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device,  BF_MANUAL_SYNC_DONE_INFO(MCS_SYNC_MCSTOP1), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "EAST manual sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);

        err = adi_apollo_hal_bf_set(device, BF_MANUAL_SYNC_INFO(MCS_SYNC_MCSTOP2), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device,  BF_MANUAL_SYNC_DONE_INFO(MCS_SYNC_MCSTOP2), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "WEST manual sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_MANUAL_SYNC_INFO(MCS_SYNC_MCSTOP0), 0x01);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_wait_to_set(device,  BF_MANUAL_SYNC_DONE_INFO(MCS_SYNC_MCSTOP0), 100000, 10);
        ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, "CENTER manual sync bit never set", API_CMS_ERROR_OPERATION_TIMEOUT);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    // Set Rx-Tx digital root clk mask to avoid them getting re-synced
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RTCLK_ADDR, (uint8_t)ADI_APOLLO_SYNC_MASK_RTCLK_DIG_ALL);
    ADI_APOLLO_ERROR_RETURN(err);

    // Mask JTx A link 0 & link 1 and perform dynamic sync
    err = rxtxlink_sync(device, (adi_apollo_sync_mask_rxtx_link_e)((uint32_t)ADI_APOLLO_SYNC_MASK_JTX_A_LINK0 | (uint32_t)ADI_APOLLO_SYNC_MASK_JTX_A_LINK1));
    ADI_APOLLO_ERROR_RETURN(err);

    // Mask JTx B link 0 & link 1 and perform dynamic sync
    err = rxtxlink_sync(device, (adi_apollo_sync_mask_rxtx_link_e)((uint32_t)ADI_APOLLO_SYNC_MASK_JTX_B_LINK0 | (uint32_t)ADI_APOLLO_SYNC_MASK_JTX_B_LINK1));
    ADI_APOLLO_ERROR_RETURN(err);

    // Mask JRx A link 0 & link 1 and perform dynamic sync
    err = rxtxlink_sync(device, (adi_apollo_sync_mask_rxtx_link_e)((uint32_t)ADI_APOLLO_SYNC_MASK_JRX_A_LINK0 | (uint32_t)ADI_APOLLO_SYNC_MASK_JRX_A_LINK1));
    ADI_APOLLO_ERROR_RETURN(err);

    // Mask JRx B link 0 & link 1 and perform dynamic sync
    err = rxtxlink_sync(device, (adi_apollo_sync_mask_rxtx_link_e)((uint32_t)ADI_APOLLO_SYNC_MASK_JRX_B_LINK0 | (uint32_t)ADI_APOLLO_SYNC_MASK_JRX_B_LINK1));
    ADI_APOLLO_ERROR_RETURN(err);

    // Clear Rx-Tx digital root clk mask
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RTCLK_ADDR, (uint8_t)ADI_APOLLO_SYNC_MASK_RTCLK_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    // Clear JTx/JRx link masks
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RXTXLINK_ADDR, (uint8_t)ADI_APOLLO_SYNC_MASK_RXTX_LINK_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}



int32_t adi_apollo_clk_mcs_dyn_sync_sequence_run(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    /* Mask Synchronization of ADC and DAC fifos, Rx and Tx digital, JRx and JTx link, and LPBK fifo */
    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_ADCFIFO_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_ADCFIFO_ALL);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_DACFIFO_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_DACFIFO_ALL);
    ADI_APOLLO_ERROR_RETURN(err);
    
    err = adi_apollo_clk_mcs_dig_path_reset_disable(device);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_clk_mcs_rxtx_link_reset_disable(device);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_clk_mcs_lpbk_fifo_reset_disable(device);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Synchronize Root Clocks */
    err = adi_apollo_clk_mcs_dynamic_sync(device);
    ADI_APOLLO_ERROR_RETURN(err);
    
    /*
     * Allow synchronization of other blocks
     * RTCLK and RXTXLINK masks handled inside adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run()
     */
    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_ADCFIFO_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_ADCFIFO_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_DACFIFO_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_DACFIFO_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_RXTX_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_RX_TX_NONE);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device, BF_SYNC_MASK_LPBKFIFO_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_LPBKFIFO_NONE);
    ADI_APOLLO_ERROR_RETURN(err);
    
    /* Staggered Synchronization of JTx and JRx links. */
    err = adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run(device);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_subclass_set(adi_apollo_device_t *device, uint32_t subclass)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SUBCLASS_SEL_INFO(MCS_SYNC_MCSTOP1), (subclass == 0U) ? 0U : 1U);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SUBCLASS_SEL_INFO(MCS_SYNC_MCSTOP2), (subclass == 0U) ? 0U : 1U);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SUBCLASS_SEL_INFO(MCS_SYNC_MCSTOP0), (subclass == 0U) ? 0U : 1U);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_subclass_get(adi_apollo_device_t *device, uint32_t *subclass)
{
    int32_t err;
    uint8_t val_e, val_w;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(subclass);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_SUBCLASS_SEL_INFO(MCS_SYNC_MCSTOP1), &val_e, (uint8_t)sizeof(val_e));
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_SUBCLASS_SEL_INFO(MCS_SYNC_MCSTOP2), &val_w, (uint8_t)sizeof(val_w));
        ADI_APOLLO_ERROR_RETURN(err);

        if (val_e != val_w) {
            ADI_CMS_RETURN(API_CMS_ERROR_ERROR);
        }

        *subclass = val_e;
    } else {
        err = adi_apollo_hal_bf_get(device, BF_SUBCLASS_SEL_INFO(MCS_SYNC_MCSTOP0), (uint8_t *)subclass, (uint8_t)sizeof(*subclass));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_internal_sysref_per_get(adi_apollo_device_t *device, uint16_t *sysref_per)
{
    int32_t err;
    uint16_t val_e, val_w;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(sysref_per);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_INT_SYSREF_PERIOD_INFO(MCS_SYNC_MCSTOP1), (uint8_t *)&val_e, (uint8_t)sizeof(val_e));
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_INT_SYSREF_PERIOD_INFO(MCS_SYNC_MCSTOP2), (uint8_t *)&val_w, (uint8_t)sizeof(val_w));
        ADI_APOLLO_ERROR_RETURN(err);

        if (val_e != val_w) {
            ADI_CMS_RETURN(API_CMS_ERROR_ERROR);
        }

        *sysref_per = val_e;
    } else {
        err = adi_apollo_hal_bf_get(device, BF_INT_SYSREF_PERIOD_INFO(MCS_SYNC_MCSTOP0), (uint8_t *)sysref_per, (uint8_t)sizeof(*sysref_per));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sysref_phase_get(adi_apollo_device_t *device, uint32_t *sysref_phase)
{
    int32_t err;
    uint32_t val_e, val_w;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(sysref_phase);

    /* generate strobe for sysref phase read */
    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_PHASE_INFO(MCS_SYNC_MCSTOP1), 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_PHASE_INFO(MCS_SYNC_MCSTOP1), (uint8_t *)&val_e, (uint8_t)sizeof(val_e));
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_set(device, BF_SYSREF_PHASE_INFO(MCS_SYNC_MCSTOP2), 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_PHASE_INFO(MCS_SYNC_MCSTOP2), (uint8_t *)&val_w, (uint8_t)sizeof(val_w));
        ADI_APOLLO_ERROR_RETURN(err);

        if (val_e != val_w) {
            ADI_CMS_RETURN(API_CMS_ERROR_ERROR);
        }

        *sysref_phase = val_e;
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_PHASE_INFO(MCS_SYNC_MCSTOP0), 1);
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_get(device, BF_SYSREF_PHASE_INFO(MCS_SYNC_MCSTOP0), (uint8_t *)sysref_phase, (uint8_t)sizeof(*sysref_phase));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


int32_t adi_apollo_clk_mcs_sysref_count_set(adi_apollo_device_t* device, uint32_t sysref_count)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_COUNT_INFO(MCS_SYNC_MCSTOP1), sysref_count);
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_set(device, BF_SYSREF_COUNT_INFO(MCS_SYNC_MCSTOP2), sysref_count);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_COUNT_INFO(MCS_SYNC_MCSTOP0), sysref_count);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sysref_en_set(adi_apollo_device_t* device, uint8_t enable)
{

    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_EN_INFO(MCS_TDC_MCSTOP1), (enable != 0U) ? 1ULL : 0ULL);
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_set(device, BF_SYSREF_EN_INFO(MCS_TDC_MCSTOP2), (enable != 0U) ? 1ULL : 0ULL);
        ADI_APOLLO_ERROR_RETURN(err);
    }
    else {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_EN_INFO(MCS_TDC_MCSTOP0), (enable != 0U) ? 1ULL : 0ULL);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    /* Add 50us delay when enabling SYSREF to ensure receiver stability before oneshot sync */
    if (enable != 0U) {
        err = adi_apollo_hal_delay_us(device, 50);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sysref_en_get(adi_apollo_device_t* device, uint8_t* enable)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(enable);

    if (IS_DUAL_CLK(device) != 0U) {
        uint8_t enable_a;
        uint8_t enable_b;

        err = adi_apollo_hal_bf_get(device, BF_SYSREF_EN_INFO(MCS_TDC_MCSTOP1), &enable_a, sizeof(enable_a));
        ADI_APOLLO_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_get(device, BF_SYSREF_EN_INFO(MCS_TDC_MCSTOP2), &enable_b, sizeof(enable_b));
        ADI_APOLLO_ERROR_RETURN(err);

        /* For dual-clock configurations bit 0(LSB) of enable refers to A-side and bit 1 to B-side. */
        *enable = enable_b & enable_a;
    }
    else {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_EN_INFO(MCS_TDC_MCSTOP0), (uint8_t*)enable, sizeof(*enable));
        ADI_APOLLO_ERROR_RETURN(err);
   }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sysref_internal_term_en_set(adi_apollo_device_t *device, uint8_t enable)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_RX_RTERM_PD_INFO(MCS_TDC_MCSTOP1), (enable == 0U) ? 1ULL : 0ULL);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_RX_RTERM_PD_INFO(MCS_TDC_MCSTOP2), (enable == 0U) ? 1ULL : 0ULL);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_RX_RTERM_PD_INFO(MCS_TDC_MCSTOP0), (enable == 0U) ? 1ULL : 0ULL);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_sync_trig_map(adi_apollo_device_t *device, uint16_t rx_tx_select, adi_apollo_trig_pin_e trig_pin)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(trig_pin > ADI_APOLLO_TRIG_PIN_B1);

    if ((rx_tx_select & (uint16_t)ADI_APOLLO_RX_DIG_A) != 0U) {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_set(device, BF_MCS_AB_TRIG_SEL_RX_INFO(MCS_SYNC_MCSTOP1), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        } else {
            err = adi_apollo_hal_bf_set(device, BF_MCS_C_TRIG_SEL_RX_A_INFO(MCS_SYNC_MCSTOP0), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    if ((rx_tx_select & (uint16_t)ADI_APOLLO_RX_DIG_B) != 0U) {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_set(device, BF_MCS_AB_TRIG_SEL_RX_INFO(MCS_SYNC_MCSTOP2), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        } else {
            err = adi_apollo_hal_bf_set(device, BF_MCS_C_TRIG_SEL_RX_B_INFO(MCS_SYNC_MCSTOP0), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    if ((rx_tx_select & (uint16_t)ADI_APOLLO_TX_DIG_A) != 0U) {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_set(device, BF_MCS_AB_TRIG_SEL_TX_INFO(MCS_SYNC_MCSTOP1), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        } else {
            err = adi_apollo_hal_bf_set(device, BF_MCS_C_TRIG_SEL_TX_A_INFO(MCS_SYNC_MCSTOP0), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    if ((rx_tx_select & (uint16_t)ADI_APOLLO_TX_DIG_B) != 0U) {
        if (IS_DUAL_CLK(device) != 0U) {
            err = adi_apollo_hal_bf_set(device, BF_MCS_AB_TRIG_SEL_TX_INFO(MCS_SYNC_MCSTOP2), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        } else {
            err = adi_apollo_hal_bf_set(device, BF_MCS_C_TRIG_SEL_TX_B_INFO(MCS_SYNC_MCSTOP0), (uint64_t)trig_pin);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}



int32_t adi_apollo_clk_mcs_input_power_status_get(adi_apollo_device_t *device, adi_apollo_clk_input_power_status_e *status_a, adi_apollo_clk_input_power_status_e *status_b)
{
    int32_t err;

    adi_apollo_clk_input_power_status_e *status_arr[2] = {status_a, status_b};
    uint32_t top;
    uint8_t i;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(status_a);
    ADI_APOLLO_NULL_POINTER_RETURN(status_b);

    if (IS_DUAL_CLK(device) != 0U) {
        for (i = 0; i < (uint8_t)ADI_APOLLO_NUM_SIDES; i++) {
            top = calc_mcs_tdc_top_dual_clk_base(i);
            err = clk_pd_status_get(device, top, status_arr[i]);
            ADI_APOLLO_ERROR_RETURN(err);
        }
    } else {

        *status_b = ADI_APOLLO_CLK_PWR_UNUSED;

        err = clk_pd_status_get(device, MCS_TDC_MCSTOP0, status_a);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_clk_mcs_dig_clk_mask_set(adi_apollo_device_t *device, uint8_t enable)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    // mask off digital clocks from ext trigger reset
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_ADCFIFO_ADDR, (enable != 0U) ? (uint8_t)0xF : (uint8_t)0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_DACFIFO_ADDR, (enable != 0U) ? (uint8_t)0xF : (uint8_t)0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_LPBKFIFO_ADDR, (enable != 0U) ? (uint8_t)0x3 : (uint8_t)0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RTCLK_ADDR, (enable != 0U) ? (uint8_t)0xFF : (uint8_t)0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RXTX_ADDR, (enable != 0U) ? (uint8_t)0xF : (uint8_t)0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg_set(device, REG_SYNC_MASK_RXTXLINK_ADDR, (enable != 0U) ? (uint8_t)0xF : (uint8_t)0);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);

}

/*==================== S T A T I C  F U N C T I O N S ====================*/

static int32_t clk_pd_status_get(adi_apollo_device_t *device, uint32_t top_base, adi_apollo_clk_input_power_status_e *status)
{
    int32_t err;
    uint8_t scr, rx;

    /* Toggle clock comparator to get measurement */
    err = adi_apollo_hal_bf_set(device, BF_CK_CKR_PD_COMP_CK_INFO(top_base), 0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device, BF_CK_CKR_PD_COMP_CK_INFO(top_base), 1);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device, BF_CK_CKR_PD_COMP_CK_INFO(top_base), 0);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Read status bits */
    err = adi_apollo_hal_bf_get(device, BF_CK_CKR_PD_STATUS_RX_INFO(top_base), (uint8_t*)&rx, (uint8_t)sizeof(rx));
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_get(device, BF_CK_CKR_PD_STATUS_SCR_INFO(top_base), (uint8_t*)&scr, (uint8_t)sizeof(scr));
    ADI_APOLLO_ERROR_RETURN(err);

    if (((scr != 0U) && (rx != 0U))) {
        *status = ADI_APOLLO_CLK_PWR_OVERDRIVEN;
    } else if ((scr == 0U) && (rx != 0U)) {
        *status = ADI_APOLLO_CLK_PWR_GOOD;
    } else if ((scr == 0U) && (rx == 0U)) {
        *status = ADI_APOLLO_CLK_PWR_UNDERDRIVEN;
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_CLK_CKT);
    }
    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t rxtxlink_sync(adi_apollo_device_t *device, adi_apollo_sync_mask_rxtx_link_e rxtx_link_mask)
{
    return mask_sync(device, BF_SYNC_MASK_RXTXLINK_INFO, (uint64_t)ADI_APOLLO_SYNC_MASK_RXTX_LINK_ALL, (uint64_t)rxtx_link_mask);
}

static int32_t mask_sync(adi_apollo_device_t *device, uint32_t reg, uint32_t info, uint64_t filter, uint64_t value)
{
    int32_t err;

    err = adi_apollo_hal_bf_set(device, reg, info, filter & (~value));
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_clk_mcs_dynamic_sync(device);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_delay_us(device, 50);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static void get_adc_fifo_offsets(adi_apollo_data_fifo_conv_clk_ratio_e clk_ratio, uint8_t is_8t8r, uint8_t *wr_offset, uint8_t *rd_offset)
{
                                                 /* wr_ofst    rd_ofst */
    static uint8_t adc_fifo_offsets_4t4r[2][3] = {{0, 0, 0}, {3, 4, 4}};        /* 4t4r ratio offsets: 1:1, 1:2, 1:4 */
    static uint8_t adc_fifo_offsets_8t8r[2][3] = {{0, 0, 0}, {5, 5, 5}};        /* 8t8r ratio offsets: 1:1, 1:2, 1:4 */

    if (is_8t8r != 0U) {
        *wr_offset = adc_fifo_offsets_8t8r[0][clk_ratio];
        *rd_offset = adc_fifo_offsets_8t8r[1][clk_ratio];
    } else {
        *wr_offset = adc_fifo_offsets_4t4r[0][clk_ratio];
        *rd_offset = adc_fifo_offsets_4t4r[1][clk_ratio];
    }
}

static void get_dac_fifo_offsets(adi_apollo_data_fifo_conv_clk_ratio_e clk_ratio, uint8_t is_8t8r, uint8_t *wr_offset, uint8_t *rd_offset)
{
                                                 /* wr_ofst    rd_ofst */
    static uint8_t dac_fifo_offsets_4t4r[2][3] = {{0, 0, 0}, {2, 2, 2}};            /* 4t4r ratio offsets: 1:1, 1:2, 1:4 */
    static uint8_t dac_fifo_offsets_8t8r[2][3] = {{3, 2, 2}, {0, 0, 0}};            /* 8t8r ratio offsets: 1:1, 1:2, 1:4 */

    if (is_8t8r != 0U) {
        *wr_offset = dac_fifo_offsets_8t8r[0][clk_ratio];
        *rd_offset = dac_fifo_offsets_8t8r[1][clk_ratio];
    } else {
        *wr_offset = dac_fifo_offsets_4t4r[0][clk_ratio];
        *rd_offset = dac_fifo_offsets_4t4r[1][clk_ratio];
    }
}

static uint32_t calc_rx_datin_base_from_adc_index(uint32_t idx)
{
    static uint32_t rx_datin_regmap[ADI_APOLLO_ADC_NUM] = {
        RX_DATIN_RX_SLICE_0_RX_DIGITAL0,        /* A0: 4T4R/8T8R */
        RX_DATIN_RX_SLICE_1_RX_DIGITAL0,        /* A1: 4T4R/8T8R */
        RX_DATIN_RX_SLICE_0_RX_DIGITAL0,        /* A2: 8T8R */
        RX_DATIN_RX_SLICE_1_RX_DIGITAL0,        /* A3: 8T8R */

        RX_DATIN_RX_SLICE_0_RX_DIGITAL1,        /* B0: 4T4R/8T8R */
        RX_DATIN_RX_SLICE_1_RX_DIGITAL1,        /* B1: 4T4R/8T8R */
        RX_DATIN_RX_SLICE_0_RX_DIGITAL1,        /* B2: 8T8R */
        RX_DATIN_RX_SLICE_1_RX_DIGITAL1         /* B3: 8T8R */
    };
    return rx_datin_regmap[idx];
}

static uint32_t calc_mcs_tdc_top_dual_clk_base(uint32_t idx)
{
    static uint32_t mcs_tdc_top_regmap[] = {
        MCS_TDC_MCSTOP1,        /* A */
        MCS_TDC_MCSTOP2         /* B */
    };
    return mcs_tdc_top_regmap[idx];
}

static uint32_t calc_tx_hsdout_base(uint32_t index)
{
    /*
     * This table is organized for DAC context usage (fifo, scrambler) of hsdout,
     * different from datapath (e.g. inv sinc)
     */
    static uint32_t tx_hsdout_regmap[ADI_APOLLO_DAC_NUM] = {
        TX_HSDOUT0_TX_SLICE_0_TX_DIGITAL0,      /* A0  */
        TX_HSDOUT1_TX_SLICE_0_TX_DIGITAL0,      /* A1* */
        TX_HSDOUT0_TX_SLICE_1_TX_DIGITAL0,      /* A2* */
        TX_HSDOUT1_TX_SLICE_1_TX_DIGITAL0,      /* A3  */

        TX_HSDOUT0_TX_SLICE_0_TX_DIGITAL1,      /* B0  */
        TX_HSDOUT1_TX_SLICE_0_TX_DIGITAL1,      /* B1* */
        TX_HSDOUT0_TX_SLICE_1_TX_DIGITAL1,      /* B2* */
        TX_HSDOUT1_TX_SLICE_1_TX_DIGITAL1       /* B3  */
    };
    return tx_hsdout_regmap[index];
}

int32_t adi_apollo_clk_mcs_sysref_err_window_set(adi_apollo_device_t *device, uint8_t err_window_a, uint8_t err_window_b)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(err_window_a > 0x7F);
    ADI_APOLLO_INVALID_PARAM_RETURN(err_window_b > 0x7F);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_ERR_WINDOW_INFO(MCS_SYNC_MCSTOP1), err_window_a);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_ERR_WINDOW_INFO(MCS_SYNC_MCSTOP2), err_window_b);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_ERR_WINDOW_INFO(MCS_SYNC_MCSTOP0), err_window_a);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_err_window_get(adi_apollo_device_t *device, uint8_t *err_window_a, uint8_t *err_window_b)
{
    int32_t err;
    uint8_t data;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(err_window_a);
    ADI_APOLLO_NULL_POINTER_RETURN(err_window_b);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_ERR_WINDOW_INFO(MCS_SYNC_MCSTOP1), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *err_window_a = data;

        err = adi_apollo_hal_bf_get(device, BF_SYSREF_ERR_WINDOW_INFO(MCS_SYNC_MCSTOP2), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *err_window_b = data;
    } else {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_ERR_WINDOW_INFO(MCS_SYNC_MCSTOP0), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *err_window_a = data;
        *err_window_b = 0;
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_within_errwindow_get(adi_apollo_device_t *device, uint8_t *status_a, uint8_t *status_b)
{
    int32_t err;
    uint8_t data;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(status_a);
    ADI_APOLLO_NULL_POINTER_RETURN(status_b);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_WITHIN_ERRWINDOW_INFO(MCS_SYNC_MCSTOP1), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status_a = (uint8_t)data;

        err = adi_apollo_hal_bf_get(device, BF_SYSREF_WITHIN_ERRWINDOW_INFO(MCS_SYNC_MCSTOP2), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status_b = (uint8_t)data;
    } else {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_WITHIN_ERRWINDOW_INFO(MCS_SYNC_MCSTOP0), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *status_a = (uint8_t)data;
        *status_b = 0;
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_delay_set(adi_apollo_device_t *device, uint16_t delay_fclk_cycles_a, uint16_t delay_fclk_cycles_b)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(delay_fclk_cycles_a > 0xFFF);
    ADI_APOLLO_INVALID_PARAM_RETURN(delay_fclk_cycles_b > 0xFFF);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYNC_SYSREF_DELAY_INFO(MCS_SYNC_MCSTOP1), delay_fclk_cycles_a);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SYNC_SYSREF_DELAY_INFO(MCS_SYNC_MCSTOP2), delay_fclk_cycles_b);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYNC_SYSREF_DELAY_INFO(MCS_SYNC_MCSTOP0), delay_fclk_cycles_a);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_delay_get(adi_apollo_device_t *device, uint16_t *delay_fclk_cycles_a, uint16_t *delay_fclk_cycles_b)
{
    int32_t err;
    uint16_t data;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(delay_fclk_cycles_a);
    ADI_APOLLO_NULL_POINTER_RETURN(delay_fclk_cycles_b);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_SYNC_SYSREF_DELAY_INFO(MCS_SYNC_MCSTOP1), (uint8_t *)&data, sizeof(data));
        ADI_APOLLO_ERROR_RETURN(err);
        *delay_fclk_cycles_a = data;

        err = adi_apollo_hal_bf_get(device, BF_SYNC_SYSREF_DELAY_INFO(MCS_SYNC_MCSTOP2), (uint8_t *)&data, sizeof(data));
        ADI_APOLLO_ERROR_RETURN(err);
        *delay_fclk_cycles_b = data;
    } else {
        err = adi_apollo_hal_bf_get(device, BF_SYNC_SYSREF_DELAY_INFO(MCS_SYNC_MCSTOP0), (uint8_t *)&data, sizeof(data));
        ADI_APOLLO_ERROR_RETURN(err);
        *delay_fclk_cycles_a = data;
        *delay_fclk_cycles_b = 0;
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_average_set(adi_apollo_device_t *device, uint8_t average_count_a, uint8_t average_count_b)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(average_count_a > 0x7);
    ADI_APOLLO_INVALID_PARAM_RETURN(average_count_b > 0x7);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_AVERAGE_INFO(MCS_SYNC_MCSTOP1), average_count_a);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_AVERAGE_INFO(MCS_SYNC_MCSTOP2), average_count_b);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = adi_apollo_hal_bf_set(device, BF_SYSREF_AVERAGE_INFO(MCS_SYNC_MCSTOP0), average_count_a);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_average_get(adi_apollo_device_t *device, uint8_t *average_count_a, uint8_t *average_count_b)
{
    int32_t err;
    uint8_t data;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(average_count_a);
    ADI_APOLLO_NULL_POINTER_RETURN(average_count_b);

    if (IS_DUAL_CLK(device) != 0U) {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_AVERAGE_INFO(MCS_SYNC_MCSTOP1), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *average_count_a = data;

        err = adi_apollo_hal_bf_get(device, BF_SYSREF_AVERAGE_INFO(MCS_SYNC_MCSTOP2), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *average_count_b = data;
    } else {
        err = adi_apollo_hal_bf_get(device, BF_SYSREF_AVERAGE_INFO(MCS_SYNC_MCSTOP0), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *average_count_a = data;
        *average_count_b = 0;
    }

    return API_CMS_ERROR_OK;
}

int32_t adi_apollo_clk_mcs_sysref_setup_hold_get(adi_apollo_device_t *device, uint8_t spare_select,
                                                   uint8_t *sh_before_a, uint8_t *sh_after_a,
                                                   uint8_t *sh_before_b, uint8_t *sh_after_b)
{
    int32_t err;
    uint8_t data;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(sh_before_a);
    ADI_APOLLO_NULL_POINTER_RETURN(sh_after_a);
    ADI_APOLLO_NULL_POINTER_RETURN(sh_before_b);
    ADI_APOLLO_NULL_POINTER_RETURN(sh_after_b);
    ADI_APOLLO_INVALID_PARAM_RETURN(spare_select > 1);

    if (IS_DUAL_CLK(device) != 0U) {
        /* Side A */
        err = adi_apollo_hal_bf_set(device, BF_MCS_SH_EN_INFO(MCS_TDC_MCSTOP1), 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SPARE1_INFO(MCS_TDC_MCSTOP1), spare_select);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_SH_BEFORE_INFO(MCS_TDC_MCSTOP1), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *sh_before_a = data;
        err = adi_apollo_hal_bf_get(device, BF_SH_AFTER_INFO(MCS_TDC_MCSTOP1), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *sh_after_a = data;

        /* Side B */
        err = adi_apollo_hal_bf_set(device, BF_MCS_SH_EN_INFO(MCS_TDC_MCSTOP2), 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SPARE1_INFO(MCS_TDC_MCSTOP2), spare_select);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_SH_BEFORE_INFO(MCS_TDC_MCSTOP2), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *sh_before_b = data;
        err = adi_apollo_hal_bf_get(device, BF_SH_AFTER_INFO(MCS_TDC_MCSTOP2), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *sh_after_b = data;
    } else {
        /* Center (single clock) */
        err = adi_apollo_hal_bf_set(device, BF_MCS_SH_EN_INFO(MCS_TDC_MCSTOP0), 1);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_set(device, BF_SPARE1_INFO(MCS_TDC_MCSTOP0), spare_select);
        ADI_APOLLO_ERROR_RETURN(err);
        err = adi_apollo_hal_bf_get(device, BF_SH_BEFORE_INFO(MCS_TDC_MCSTOP0), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *sh_before_a = data;
        err = adi_apollo_hal_bf_get(device, BF_SH_AFTER_INFO(MCS_TDC_MCSTOP0), &data, 1);
        ADI_APOLLO_ERROR_RETURN(err);
        *sh_after_a = data;
        *sh_before_b = 0;
        *sh_after_b = 0;
    }

    return API_CMS_ERROR_OK;
}

/*! @} */