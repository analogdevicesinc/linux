/*!
 * \brief     APIs for BMEM
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_BMEM
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_bmem.h"
#include "adi_apollo_private_bmem.h"
#include "adi_apollo_private_blk_sel_types.h"

#include "adi_apollo_bf_rx_bmem.h"
#include "adi_apollo_trigts.h"
#include "adi_apollo_bf_txrx_prefsrc_reconf.h"
#include "adi_apollo_bf_mcs_sync.h"

#include "adi_apollo_config.h"
#include "adi_apollo_hal.h"

/*==================== L O C A L   F U N C T I O N S ====================*/

/**
 * \brief  Configures sync pulse routing for BMEM trigger inputs
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal  RX or TX terminal
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
static int32_t adi_apollo_bmem_sync_pulse_routing_enable(adi_apollo_device_t *device,
                                                          adi_apollo_terminal_e terminal)
{
    int32_t err;
    uint8_t side_index;
    uint32_t regmap_base_addr;

    // Table of prefsrc_reconf register bases
    static const uint32_t rx_prefsrc_bases[ADI_APOLLO_NUM_SIDES] = {
        TXRX_PREFSRC_RECONF_RX_SLICE_0_RX_DIGITAL0,
        TXRX_PREFSRC_RECONF_RX_SLICE_0_RX_DIGITAL1
    };

    static const uint32_t tx_prefsrc_bases[ADI_APOLLO_NUM_SIDES] = {
        TXRX_PREFSRC_RECONF_TX_SLICE_0_TX_DIGITAL0,
        TXRX_PREFSRC_RECONF_TX_SLICE_0_TX_DIGITAL1
    };

    // Configure both sides for sync pulse routing
    for (side_index = 0U; side_index < (uint8_t)ADI_APOLLO_NUM_SIDES; side_index++) {
        regmap_base_addr = (terminal == ADI_APOLLO_RX) ?
            rx_prefsrc_bases[side_index] : tx_prefsrc_bases[side_index];

        // Configure sync pulse routing to trigger inputs
        // Disable clock reset on trigger/sync (prevents disrupting data path)
        err = adi_apollo_hal_bf_set(device, BF_TZERO_COHERENCE_EN_INFO(regmap_base_addr), 0);
        ADI_CMS_ERROR_RETURN(err);

        err = adi_apollo_hal_bf_set(device, BF_RESYNC_EN_INFO(regmap_base_addr), 0);
        ADI_CMS_ERROR_RETURN(err);

        // Enable sync pulse routing to trigger inputs (CRITICAL for cross-side sync)
        err = adi_apollo_hal_bf_set(device, BF_NONRESYNC_SYSREF_EN_INFO(regmap_base_addr), 1);
        ADI_CMS_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_bmem_hsdin_awg_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_awg_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);


    err = adi_apollo_private_bmem_awg_config(device, ADI_APOLLO_BMEM_HSDIN, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_awg_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_awg_start(device, ADI_APOLLO_BMEM_HSDIN, bmems);
    ADI_CMS_ERROR_RETURN(err);
    
    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_awg_stop(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_awg_stop(device, ADI_APOLLO_BMEM_HSDIN, bmems);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_awg_sram_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint32_t data[], uint32_t length)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(data);
    ADI_CMS_INVALID_PARAM_CHECK(length > ADI_APOLLO_BMEM_HSDIN_MEM_SIZE_WORDS);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_awg_sram_set(device, ADI_APOLLO_BMEM_HSDIN, bmems, data, length);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_awg_sample_write(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, int16_t data16[], uint32_t data16_len)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(data16);
    ADI_CMS_INVALID_PARAM_CHECK(data16_len/2U > ADI_APOLLO_BMEM_HSDIN_MEM_SIZE_WORDS);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_awg_sample_write(device, ADI_APOLLO_BMEM_HSDIN, bmems, data16, data16_len);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_awg_sample_shared_write(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems,
                                             int16_t data16_0[], int16_t data16_1[], uint32_t data16_len,
                                             uint32_t scratch32[], uint32_t scratch32_len)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(data16_0);
    ADI_CMS_NULL_PTR_CHECK(data16_1);
    ADI_CMS_NULL_PTR_CHECK(scratch32);
    ADI_CMS_INVALID_PARAM_CHECK(data16_len > (ADI_APOLLO_BMEM_HSDIN_MEM_SIZE_BYTES / (2U * sizeof(uint16_t))));
    ADI_CMS_INVALID_PARAM_CHECK(scratch32_len < data16_len);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_awg_sample_shared_write(device, ADI_APOLLO_BMEM_HSDIN, bmems, data16_0, data16_1, data16_len, scratch32, scratch32_len);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_capture_get(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint32_t data[], uint32_t length)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(data);
    ADI_CMS_SINGLE_SELECT_CHECK(bmems);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_capture_get(device, ADI_APOLLO_BMEM_HSDIN, bmems, data, length);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_capture_run(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_capture_run(device, ADI_APOLLO_BMEM_HSDIN, bmems);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_capture_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_capture_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_CMS_INVALID_PARAM_CHECK(((uint32_t)config->end_addr_cpt - (uint32_t)config->st_addr_cpt) >= ADI_APOLLO_BMEM_HSDIN_MEM_SIZE_WORDS);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_capture_config(device, ADI_APOLLO_BMEM_HSDIN, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_capture_config_triggered(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems,
                                                        adi_apollo_bmem_capture_t *config, adi_apollo_trigger_sel_mux_e trig_source)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_CMS_INVALID_PARAM_CHECK(((uint32_t)config->end_addr_cpt - (uint32_t)config->st_addr_cpt) >= ADI_APOLLO_BMEM_HSDIN_MEM_SIZE_WORDS);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    // Configure trigger mux for all selected BMEMs
    err = adi_apollo_trigts_bmem_trig_sel_mux_set(device, ADI_APOLLO_RX, bmems, trig_source);
    ADI_CMS_ERROR_RETURN(err);

    // If using TRIG_INPUT, configure sync pulse routing for cross-side synchronization
    if (trig_source == ADI_APOLLO_TRIG_INPUT) {
        err = adi_apollo_bmem_sync_pulse_routing_enable(device, ADI_APOLLO_RX);
        ADI_CMS_ERROR_RETURN(err);
    }

    // Configure BMEM in triggered mode
    err = adi_apollo_private_bmem_capture_config_triggered(device, ADI_APOLLO_BMEM_HSDIN, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

int32_t adi_apollo_bmem_hsdin_capture_arm(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    // Arm BMEMs for triggered capture (reset and fast mode only)
    err = adi_apollo_private_bmem_capture_arm(device, ADI_APOLLO_BMEM_HSDIN, bmems);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

int32_t adi_apollo_bmem_hsdin_capture_trig_mode_enable(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    // Enable trigger mode - BMEMs will capture on next sync pulse
    err = adi_apollo_private_bmem_capture_trig_mode_enable(device, ADI_APOLLO_BMEM_HSDIN, bmems);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

int32_t adi_apollo_bmem_hsdin_capture_trigger_spi(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();

    // Issue SPI trigger to start capture
    // NOTE: trig_now() iterates Side A then Side B with separate SPI transactions,
    // so Side A can start several SPI transactions before Side B. This breaks
    // cross-side zero-skew synchronization. Use adi_apollo_bmem_hsdin_capture_trigger_sync()
    // for hardware-distributed triggering with true cross-side sync.
    err = adi_apollo_trigts_trig_now(device, ADI_APOLLO_RX, (uint16_t)ADI_APOLLO_SIDE_A | (uint16_t)ADI_APOLLO_SIDE_B);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

int32_t adi_apollo_bmem_hsdin_capture_trigger_sync(adi_apollo_device_t *device)
{
    int32_t err;
    int32_t restore_err;
    int32_t op_err = (int32_t)API_CMS_ERROR_OK;
    uint8_t synctrl_man_orig[2] = {0};
    uint16_t synctrl_man_value;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();

    // Read original synctrl_man value (16-bit register)
    err = adi_apollo_hal_bf_get(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP0), synctrl_man_orig, (uint8_t)sizeof(synctrl_man_orig));
    ADI_CMS_ERROR_RETURN(err);

    // Operations that modify synctrl_man - must restore afterward
    do {
        // Set synctrl_man to target Rx digital clock generator (bit 6)
        err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP0), (uint64_t)(1U << 6U));
        if (err != (int32_t)API_CMS_ERROR_OK) {
            op_err = err;
            break;
        }

        // Trigger manual sync pulse
        err = adi_apollo_hal_bf_set(device, BF_MANUAL_SYNC_INFO(MCS_SYNC_MCSTOP0), 0x01U);
        if (err != (int32_t)API_CMS_ERROR_OK) {
            op_err = err;
            break;
        }

        // Wait for manual_sync_done
        err = adi_apollo_hal_bf_wait_to_set(device, BF_MANUAL_SYNC_DONE_INFO(MCS_SYNC_MCSTOP0), 100000U, 10U);
        if (err != (int32_t)API_CMS_ERROR_OK) {
            op_err = (int32_t)API_CMS_ERROR_OPERATION_TIMEOUT;
            break;
        }
    } while (false);

    // Always restore original synctrl_man value
    synctrl_man_value = (uint16_t)((uint16_t)synctrl_man_orig[0] | ((uint16_t)synctrl_man_orig[1] << 8U));
    restore_err = adi_apollo_hal_bf_set(device, BF_SYNCTRL_MAN_INFO(MCS_SYNC_MCSTOP0), synctrl_man_value);

    // Return original operation error if one occurred, otherwise return restore error
    if (op_err != (int32_t)API_CMS_ERROR_OK) {
        return op_err;
    }
    return restore_err;
}

int32_t adi_apollo_bmem_hsdin_capture_wait_complete(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint32_t timeout_us)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_capture_wait_complete(device, ADI_APOLLO_BMEM_HSDIN, bmems, timeout_us);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

int32_t adi_apollo_bmem_hsdin_delay_sample_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_sample_set(device, ADI_APOLLO_BMEM_HSDIN, bmems, sample_delay);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_delay_hop_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay[], uint32_t sample_delay_length)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(sample_delay);
    ADI_CMS_INVALID_PARAM_CHECK(sample_delay_length != ADI_APOLLO_BMEM_HOP_PROFILES);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_hop_set(device, ADI_APOLLO_BMEM_HSDIN, bmems, sample_delay, sample_delay_length);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_delay_sample_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_sample_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_sample_config(device, ADI_APOLLO_BMEM_HSDIN, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_delay_hop_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_hop_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_hop_config(device, ADI_APOLLO_BMEM_HSDIN, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_hsdin_delay_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_start(device, ADI_APOLLO_BMEM_HSDIN, bmems);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_cddc_delay_sample_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_sample_set(device, ADI_APOLLO_BMEM_CDDC, bmems, sample_delay);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_cddc_delay_hop_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay[], uint32_t sample_delay_length)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(sample_delay);
    ADI_CMS_INVALID_PARAM_CHECK(sample_delay_length != ADI_APOLLO_BMEM_HOP_PROFILES);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_hop_set(device, ADI_APOLLO_BMEM_CDDC, bmems, sample_delay, sample_delay_length);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_cddc_delay_sample_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_sample_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_sample_config(device, ADI_APOLLO_BMEM_CDDC, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_cddc_delay_hop_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_hop_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_hop_config(device, ADI_APOLLO_BMEM_CDDC, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_cddc_delay_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_start(device, ADI_APOLLO_BMEM_CDDC, bmems);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_fddc_delay_sample_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay)
{
    int32_t err;
    
    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_sample_set(device, ADI_APOLLO_BMEM_FDDC, bmems, sample_delay);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_fddc_delay_hop_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay[], uint32_t sample_delay_length)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(sample_delay);
    ADI_CMS_INVALID_PARAM_CHECK(sample_delay_length != ADI_APOLLO_BMEM_HOP_PROFILES);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_hop_set(device, ADI_APOLLO_BMEM_FDDC, bmems, sample_delay, sample_delay_length);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_fddc_delay_sample_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_sample_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_sample_config(device, ADI_APOLLO_BMEM_FDDC, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_fddc_delay_hop_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_hop_t *config)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_CMS_NULL_PTR_CHECK(config);
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_hop_config(device, ADI_APOLLO_BMEM_FDDC, bmems, config);
    ADI_CMS_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_bmem_fddc_delay_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
    int32_t err;

    ADI_CMS_NULL_PTR_CHECK(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

    err = adi_apollo_private_bmem_delay_start(device, ADI_APOLLO_BMEM_FDDC, bmems);
    ADI_CMS_ERROR_RETURN(err);
    
    ADI_CMS_RETURN(err);
}
/*! @} */