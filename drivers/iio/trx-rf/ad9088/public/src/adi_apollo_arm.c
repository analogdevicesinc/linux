  /*!
 * \brief     API implementation ARM CPU
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_ARM
 * @{
 */

/*============= I N C L U D E S ============*/
#include "adi_apollo_arm.h"
#include "adi_apollo_config.h"
#include "adi_apollo_mailbox.h"

#include "adi_apollo_bf_custom.h"
#include "adi_apollo_bf_core.h"
#include "adi_apollo_bf_mb_regs.h"

#include "adi_apollo_hal.h"

#define SIM_BF_ALT18_FW_STATUS                              REG_ALT_BOOT_ADDR(18), 0x00000800
#define SIM_BF_RAM_BOOT_STATUS                              REG_ALT_BOOT_ADDR(21), 0x00000800
#define SIM_BF_RAM_BOOT_ERROR                               REG_ALT_BOOT_ADDR(22), 0x00000800

#define ADI_APOLLO_STREAM_MAX                   (9U)

typedef struct adi_apollo_stream_info
{
    uint32_t                chunk_stream_image_size  [ADI_APOLLO_STREAM_MAX];  /*!< Stream Image Size */
    uint32_t                chunk_stream_image_offset[ADI_APOLLO_STREAM_MAX];  /*!< Stream Image Offset */
    uint32_t                current_stream_bin_base_addr;                      /*!< Address to load current stream */
    uint32_t                current_stream_base_addr;                          /*!< Address to load current stream base */
    uint8_t                 current_stream_number_streams;                     /*!< Number of Streams for current stream  */
    uint8_t                 current_stream_image_index;                        /*!< Index of current stream  */
    uint32_t                current_stream_image_size;                         /*!< Image size of current stream */
} adi_apollo_stream_info_t;

static int32_t adi_apollo_arm_fw_load_check(adi_apollo_device_t *device, uint8_t *status);
static int32_t adi_apollo_arm_profile_load_check(adi_apollo_device_t *device, uint8_t *status);
static int32_t adi_apollo_arm_boot_status_clear(adi_apollo_device_t *device);
static int32_t adi_apollo_arm_core_reset(adi_apollo_device_t *device, uint8_t core);
static int32_t adi_apollo_arm_ram_lock_check(adi_apollo_device_t *device);
static int32_t adi_apollo_arm_stack_ptr_boot_addr_set(adi_apollo_device_t *device, uint8_t core, adi_apollo_arm_binary_info_t *binary_info);
static int32_t get_cpu_device_profile_start_address(adi_apollo_device_t *device, uint32_t *cpu_device_profile_addr);
static int32_t adi_apollo_arm_tye_bootstage_check(adi_apollo_device_t *device, uint8_t stage, uint8_t is_encrypted);
static int32_t device_feat_lockout_set(adi_apollo_device_t *device);
static int32_t adi_apollo_arm_stream_image_write(adi_apollo_device_t *device, uint32_t byte_offset, uint8_t *binary, uint32_t byte_count, adi_apollo_stream_info_t *stream_state);

/*==================== P U B L I C   A P I   C O D E ====================*/

int32_t adi_apollo_arm_boot_status(adi_apollo_device_t *device, uint8_t *status)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    //ADI_APOLLO_LOG_FUNC();

    // Core1 Boot Status
    err = adi_apollo_hal_bf_get(device, BF_RAM_BOOT_CORE1_STATUS, status, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_ram_boot_error_check(adi_apollo_device_t *device)
{
    int32_t err;
    int32_t status = 0x00;
    adi_apollo_cpu_errors_t cpu_errors;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_arm_err_codes_get(device, &cpu_errors);
    ADI_APOLLO_ERROR_RETURN(err);
    /* boot errors are stored under the system category */
    status = cpu_errors.system;
    ADI_APOLLO_LOG_MSG_VAR("REG_RAM_BOOT_ERROR 0x%X", status);

    /* use only 16-bits of status */
    status = status & 0xFFFF;
    ADI_APOLLO_ERROR_RETURN(status);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_rom_ec_transfer_check(adi_apollo_device_t *device)
{
    int32_t err;
    uint8_t status = 0;
    uint32_t retry_count = 50;
    uint32_t poll_delay_us = 100000;    // 100ms
    uint32_t i;
    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    adi_apollo_hal_regio_t *desc = (adi_apollo_hal_regio_t *) device->hal_info.active_regio;

    for (i = 0; i < retry_count; i++) {

        err = adi_apollo_hal_reg_poll_get(device, REG_ADDR_EXTRACT(BF_BOOT_STATUS), &status, retry_count * poll_delay_us, ADI_APOLLO_ROM_BOOT_STEP_EC_TRANSFER, 0xff);
        ADI_APOLLO_ERROR_RETURN(err);

        if ((status >= ADI_APOLLO_ROM_BOOT_STEP_EC_TRANSFER) || (!desc->poll_read_returns_val)) {
            break;
        }

        // Delay before next poll read
        err = adi_apollo_hal_delay_us(device, poll_delay_us);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if (status < ADI_APOLLO_ROM_BOOT_STEP_EC_TRANSFER) {
        err = (int32_t)API_CMS_ERROR_ERROR;
    } else {
        err = (int32_t)API_CMS_ERROR_OK;
    }

    ADI_APOLLO_LOG_MSG_VAR("EC Transfer Flag : %d.", status);
    ADI_APOLLO_LOG_MSG_VAR("EC Transfer Check : %s.", (status >= ADI_APOLLO_ROM_BOOT_STEP_EC_TRANSFER) ? "Passed" : "*** FAILED ***");

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_arm_profile_crc_valid_get(adi_apollo_device_t *device)
{
    int32_t err;
    uint8_t val;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_get(device, BF_DEVICE_PROFILE_CRC_CHECK_STATUS, &val, sizeof(val));
    ADI_APOLLO_ERROR_RETURN(err);

    return (val != 0U) ? (int32_t)API_CMS_ERROR_PROFILE_CRC : (int32_t)API_CMS_ERROR_OK;

}

int32_t adi_apollo_arm_fwload_pre_config(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    /* Check for RAM Lock */
    err = adi_apollo_arm_ram_lock_check(device);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Stop ARM core 0 */
    err = adi_apollo_hal_bf_set(device, BF_ARM0_M3_RUN_INFO, 0);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Stop ARM core 1 */
    err = adi_apollo_hal_bf_set(device, BF_ARM1_M3_RUN_INFO, 0);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Clear boot status */
    err = adi_apollo_arm_boot_status_clear(device);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Memory Bank division control set (Core 0)*/
    err = adi_apollo_hal_bf_set(device, BF_L1MEM0_SHARED_BANK_DIV_INFO, (uint64_t)ADI_APOLLO_DIV_IRAM_384K_DRAM_320K);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Memory Bank division control set (Core 1)*/
    err = adi_apollo_hal_bf_set(device, BF_L1MEM1_SHARED_BANK_DIV_INFO, (uint64_t)ADI_APOLLO_DIV_IRAM_352K_DRAM_352K);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Make core 1 Primary */
    err = adi_apollo_hal_bf_set(device, BF_CPU_0_PRIMARY, 0);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_bf_set(device, BF_CPU_1_PRIMARY, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_fwload_post_config(adi_apollo_device_t *device)
{
    int32_t err;
    uint8_t status;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    /* Reset core 0 */
    err = adi_apollo_arm_core_reset(device, (uint8_t)ADI_APOLLO_CPU_ID_0);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Start core 1 */
    err = adi_apollo_hal_bf_set(device, BF_ARM1_M3_RUN_INFO, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Start core 0 */
    err = adi_apollo_hal_bf_set(device, BF_ARM0_M3_RUN_INFO, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    // Delay 100ms to start the core
    err = adi_apollo_hal_delay_us(device, 100000);

    /* Check the FW boot status is waiting for device profile config */
    err = adi_apollo_arm_fw_load_check(device, &status);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_profile_write(adi_apollo_device_t *device, adi_apollo_top_t *profile, uint16_t chunk_sz_bytes)
{
    int32_t err;
    adi_apollo_arm_binary_info_t binary_info;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(profile);

    binary_info.binary_ptr = (uint8_t *)profile;
    binary_info.binary_sz_bytes = sizeof(struct adi_apollo_top);
    binary_info.is_cont = 0;
    err = get_cpu_device_profile_start_address(device, &(binary_info.cpu_start_addr));
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_arm_memload(device, &binary_info, chunk_sz_bytes);
    ADI_APOLLO_ERROR_RETURN(err);

    /* V10 Profile update: Clock config field 'single_dual_clk_sel' renamed to 'is_dual_clk' (type unchanged) - COMPLETED */
    device->dev_info.is_dual_clk = profile->clk_cfg.is_dual_clk;
    /* V10 Profile update: Field 'dev_clk_freq_kHz' renamed to 'dev_clk_freq_Hz' (uint32_t->uint64_t, kHz->Hz, remove *1000 multiplication) */
    device->dev_info.dev_freq_hz = (uint64_t)profile->clk_cfg.dev_clk_freq_Hz;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_profile_write_post_config(adi_apollo_device_t *device, adi_apollo_top_t *profile)
{
    int32_t err;
    uint8_t status;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(profile);

    /* set transfer done flag */
    err = adi_apollo_hal_bf_set(device, BF_CONFIG_TRANSFER_DONE, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    /* read boot status */
    err = adi_apollo_arm_profile_load_check(device, &status);
    ADI_APOLLO_ERROR_RETURN(err);

    /* check CRC */
    err = adi_apollo_arm_profile_crc_valid_get(device);
    if (err == (int32_t)API_CMS_ERROR_PROFILE_CRC) {
        ADI_APOLLO_LOG_WARN("profile crc is invalid.");
    }

    /* Store device feature enable configurations */
    err = device_feat_lockout_set(device);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_profile_buf_write(adi_apollo_device_t *device, uint8_t profile_buf[], uint32_t profile_size_bytes, uint16_t chunk_sz_bytes)
{
    int32_t err;
    adi_apollo_arm_binary_info_t binary_info;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(profile_buf);
    ADI_APOLLO_INVALID_PARAM_RETURN(profile_size_bytes != sizeof(adi_apollo_top_t));

    binary_info.binary_ptr = (uint8_t *)profile_buf;
    binary_info.binary_sz_bytes = (int32_t)profile_size_bytes;
    binary_info.is_cont = 0;
    err = get_cpu_device_profile_start_address(device, &(binary_info.cpu_start_addr));
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_arm_memload(device, &binary_info, chunk_sz_bytes);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_firmware_buf_write(adi_apollo_device_t *device, uint8_t core, uint8_t fw_image_buf[], uint32_t fw_image_size_bytes, uint16_t chunk_sz_bytes)
{
    adi_apollo_arm_binary_info_t binary_info;
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(fw_image_buf);
    ADI_APOLLO_INVALID_PARAM_RETURN(core > (uint8_t)ADI_APOLLO_CPU_ID_1);

    binary_info.binary_ptr = &fw_image_buf[0];
    binary_info.binary_sz_bytes = (int32_t)fw_image_size_bytes;
    binary_info.cpu_start_addr = (core == (uint8_t)ADI_APOLLO_CPU_ID_0) ? ADI_APOLLO_CPU0_PM_START : ADI_APOLLO_CPU1_PM_START;
    binary_info.is_cont = 0;

    /* Set Core's Stack ptr and Boot Addr */
    err = adi_apollo_arm_stack_ptr_boot_addr_set(device, core, &binary_info);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_arm_memload(device, &binary_info, chunk_sz_bytes);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_tye_firmware_buf_write(adi_apollo_device_t *device, uint32_t mem_addr_to_load, uint8_t fw_image_buf[], uint32_t fw_image_size_bytes, uint16_t chunk_sz_bytes)
{
    adi_apollo_arm_binary_info_t binary_info;
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(fw_image_buf);

    binary_info.binary_ptr = &fw_image_buf[0];
    binary_info.binary_sz_bytes = (int32_t)fw_image_size_bytes;
    binary_info.cpu_start_addr = mem_addr_to_load;
    binary_info.is_cont = 0;

    err = adi_apollo_arm_memload(device, &binary_info, chunk_sz_bytes);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_memload(adi_apollo_device_t *device, adi_apollo_arm_binary_info_t *binary_info, uint16_t chunk_sz_bytes)
{
    int32_t err;
    uint32_t cpu_address_to_load;
    int32_t pending_bytes;
    uint8_t *data_ptr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(binary_info);
    ADI_APOLLO_NULL_POINTER_RETURN(binary_info->binary_ptr);
    /* binary_sz_bytes must be >= 4: the chunk-adjustment formula
     * (sz - sz%4) produces 0 for inputs 1-3, causing div-by-zero in the
     * transfer loop. */
    ADI_APOLLO_INVALID_PARAM_RETURN(binary_info->binary_sz_bytes < 4);
    ADI_APOLLO_INVALID_PARAM_RETURN(chunk_sz_bytes < 1U || chunk_sz_bytes > (16 * 1024U)); // A0 has max stream limit of 16k
    ADI_APOLLO_INVALID_PARAM_RETURN((chunk_sz_bytes % 4U) != 0U);

    pending_bytes = binary_info->binary_sz_bytes;

    /* chunk size should not be bigger then total size and should always be % 4 */
    if ((int32_t)chunk_sz_bytes > binary_info->binary_sz_bytes) {
        int32_t aligned_size = binary_info->binary_sz_bytes - (binary_info->binary_sz_bytes % 4);
        chunk_sz_bytes = (uint16_t)aligned_size;
    }

    cpu_address_to_load = binary_info->cpu_start_addr;

    data_ptr = binary_info->binary_ptr;

    ADI_APOLLO_LOG_MSG_VAR("Loading %d bytes", pending_bytes);

    /* Send data in chunk_sz_bytes */
    while ((pending_bytes > 0) && (((int32_t)(pending_bytes / (int32_t)chunk_sz_bytes)) > 0)) {
        err = adi_apollo_hal_stream_reg32_set(device, cpu_address_to_load, (uint32_t *)data_ptr, chunk_sz_bytes / 4U, binary_info->is_cont);
        ADI_APOLLO_ERROR_RETURN(err);

        pending_bytes = (int32_t)(pending_bytes - (int32_t)chunk_sz_bytes);
        binary_info->is_cont = 1;
        data_ptr += chunk_sz_bytes;
        cpu_address_to_load += chunk_sz_bytes;
    }

    /* send remaining bytes */
    if (pending_bytes > 0) {
        /* have to reset the FIFO and reconfigure the SPI bus to 8-bit txn */
        binary_info->is_cont = 0;
        err = adi_apollo_hal_stream_reg_set(device, cpu_address_to_load, data_ptr, (uint32_t)pending_bytes, binary_info->is_cont);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_tye_bypassed_get(adi_apollo_device_t *device, uint8_t *is_bypassed)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(is_bypassed);

    err = adi_apollo_hal_bf_get(device, BF_NVMB_TE_BYPASS_INFO, is_bypassed, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_tye_boot_ready_set(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_TE_BOOT_READY_SET, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_tye_validate_boot_completion(adi_apollo_device_t *device, uint8_t is_encrypted)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(is_encrypted > 1U);

    err = adi_apollo_arm_tye_bootstage_check(device, 0, is_encrypted);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_arm_tye_bootstage_check(device, 1, is_encrypted);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_err_codes_get(adi_apollo_device_t *device, adi_apollo_cpu_errors_t *cpu_errs)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;
    uint32_t err_ptr = 0x00;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(cpu_errs);

    /* Get location of error struct */
    err = adi_apollo_hal_reg32_get(device, REG_RAM_BOOT_ERROR_PTR, &err_ptr);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_reg32_get(device, err_ptr + 0U, (uint32_t *) &(cpu_errs->last_cmd));
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg32_get(device, err_ptr + 4U, (uint32_t*) &(cpu_errs->system));
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_reg32_get(device, err_ptr + 8U, (uint32_t*) &(cpu_errs->track_cal));
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_arm_fw_irq_out_get(adi_apollo_device_t *device, uint8_t *fw_irq_out, uint32_t *irq_source)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;
    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(fw_irq_out);
    ADI_APOLLO_NULL_POINTER_RETURN(irq_source);

    err = adi_apollo_hal_bf_get(device, BF_FW_IRQ_OUT_INFO, fw_irq_out, sizeof(*fw_irq_out));
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_get(device, BF_FW_IRQ_OUT_SOURCE_INFO, (uint8_t*)irq_source, sizeof(*irq_source));
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_arm_fw_irq_out_clear(adi_apollo_device_t *device)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;
    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_FW_IRQ_OUT_INFO, 0);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_bf_set(device, BF_FW_IRQ_OUT_SOURCE_INFO, 0);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(err);
}

int32_t adi_apollo_arm_fw_irq_out_source_get(adi_apollo_device_t *device, uint32_t *source_mask)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(source_mask);

    adi_apollo_mailbox_resp_get_fw_irq_out_source_t resp;
    err = adi_apollo_mailbox_get_fw_irq_out_source(device, &resp);
    ADI_APOLLO_ERROR_RETURN(err);

    if (resp.status != (uint32_t)APOLLO_CPU_NO_ERROR) {
        ADI_APOLLO_LOG_MSG_VAR("adi_apollo_arm_fw_irq_out_source_get: mailbox response status error: %u", resp.status);
        err = (int32_t)API_CMS_ERROR_MAILBOX_RESP_STATUS;
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *source_mask = resp.source_mask;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_fw_irq_out_source_clear(adi_apollo_device_t *device, uint32_t clear_mask, uint32_t *remaining_mask)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(remaining_mask);

    adi_apollo_mailbox_cmd_clear_fw_irq_out_source_t cmd = { .clear_mask = clear_mask };
    adi_apollo_mailbox_resp_clear_fw_irq_out_source_t resp;
    err = adi_apollo_mailbox_clear_fw_irq_out_source(device, &cmd, &resp);
    ADI_APOLLO_ERROR_RETURN(err);

    if (resp.status != (uint32_t)APOLLO_CPU_NO_ERROR) {
        ADI_APOLLO_LOG_MSG_VAR("adi_apollo_arm_fw_irq_out_source_clear: mailbox response status error: %u", resp.status);
        err = (int32_t)API_CMS_ERROR_MAILBOX_RESP_STATUS;
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *remaining_mask = resp.source_mask;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_fw_irq_out_config_get(adi_apollo_device_t *device, uint32_t *config_mask)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config_mask);

    adi_apollo_mailbox_resp_get_fw_irq_out_config_t resp;
    err = adi_apollo_mailbox_get_fw_irq_out_config(device, &resp);
    ADI_APOLLO_ERROR_RETURN(err);

    if (resp.status != (uint32_t)APOLLO_CPU_NO_ERROR) {
        ADI_APOLLO_LOG_MSG_VAR("adi_apollo_arm_fw_irq_out_config_get: mailbox response status error: %u", resp.status);
        err = (int32_t)API_CMS_ERROR_MAILBOX_RESP_STATUS;
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *config_mask = resp.config_mask;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_fw_irq_out_sources_enable(adi_apollo_device_t *device, uint32_t enable_mask, uint32_t *config_mask)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config_mask);

    adi_apollo_mailbox_cmd_enable_fw_irq_out_sources_t cmd = { .enable_mask = enable_mask };
    adi_apollo_mailbox_resp_enable_fw_irq_out_sources_t resp;
    err = adi_apollo_mailbox_enable_fw_irq_out_sources(device, &cmd, &resp);
    ADI_APOLLO_ERROR_RETURN(err);

    if (resp.status != (uint32_t)APOLLO_CPU_NO_ERROR) {   /* APOLLO_CPU_DEV_TEMP_SENSOR_INVALID_CONFIG_MASK_ERROR (0xb60d) if enable_mask has bits outside 0xFF */
        ADI_APOLLO_LOG_MSG_VAR("adi_apollo_arm_fw_irq_out_sources_enable: mailbox response status error: %u", resp.status);
        err = (int32_t)API_CMS_ERROR_MAILBOX_RESP_STATUS;
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *config_mask = resp.config_mask;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_fw_irq_out_sources_disable(adi_apollo_device_t *device, uint32_t disable_mask, uint32_t *config_mask)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(config_mask);

    adi_apollo_mailbox_cmd_disable_fw_irq_out_sources_t cmd = { .disable_mask = disable_mask };
    adi_apollo_mailbox_resp_disable_fw_irq_out_sources_t resp;
    err = adi_apollo_mailbox_disable_fw_irq_out_sources(device, &cmd, &resp);
    ADI_APOLLO_ERROR_RETURN(err);

    if (resp.status != (uint32_t)APOLLO_CPU_NO_ERROR) {
        ADI_APOLLO_LOG_MSG_VAR("adi_apollo_arm_fw_irq_out_sources_disable: mailbox response status error: %u", resp.status);
        err = (int32_t)API_CMS_ERROR_MAILBOX_RESP_STATUS;
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *config_mask = resp.config_mask;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/* Wait for firmware boot status to be wait_for_config */
static int32_t adi_apollo_arm_fw_load_check(adi_apollo_device_t *device, uint8_t *status)
{
    int32_t err;
    uint8_t retry_count = ADI_APOLLO_ARM_BOOT_STATUS_READ_RETRY_COUNT;
    uint32_t poll_delay_us = 100000;    // 100ms

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    adi_apollo_hal_regio_t *desc = (adi_apollo_hal_regio_t *) device->hal_info.active_regio;

    /* Read boot status - wait for state ADI_APOLLO_RAM_BOOT_STEP_WAIT_FOR_CONFIG */
    do {

        err = adi_apollo_hal_reg_poll_get(device, REG_ADDR_EXTRACT(BF_RAM_BOOT_CORE1_STATUS), status,
                ADI_APOLLO_ARM_BOOT_STATUS_READ_RETRY_COUNT* poll_delay_us,
                ADI_APOLLO_RAM_BOOT_STEP_WAIT_FOR_CONFIG, 0xff);
        ADI_APOLLO_ERROR_RETURN(err);

        ADI_APOLLO_LOG_MSG_VAR("BF_RAM_BOOT_CORE1_STATUS: %d.", *status);

        if ((*status >= ADI_APOLLO_RAM_BOOT_STEP_WAIT_FOR_CONFIG) || (!desc->poll_read_returns_val)) {
            ADI_CMS_RETURN(API_CMS_ERROR_OK);
        }

        // Delay 'poll_delay_us' to wait before reading status again
        err = adi_apollo_hal_delay_us(device, poll_delay_us);
        ADI_APOLLO_ERROR_RETURN(err);

        retry_count--;
    } while (retry_count != 0U);

    ADI_CMS_RETURN(API_CMS_ERROR_ERROR);
}

/* Wait for firmware boot status to be >= profile crc check passed */
static int32_t adi_apollo_arm_profile_load_check(adi_apollo_device_t *device, uint8_t *status)
{
    int32_t err;
    uint8_t retry_count = ADI_APOLLO_ARM_BOOT_STATUS_READ_RETRY_COUNT;
    uint32_t poll_delay_us = 100000;    // 100ms

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    adi_apollo_hal_regio_t *desc = (adi_apollo_hal_regio_t *) device->hal_info.active_regio;

    /* Read boot status - wait for state ADI_APOLLO_RAM_BOOT_STEP_DEVICE_CONFIG_CRC_CHECK */
    do {
        err = adi_apollo_hal_reg_poll_get(device, REG_ADDR_EXTRACT(BF_RAM_BOOT_CORE1_STATUS), status,
            ADI_APOLLO_ARM_BOOT_STATUS_READ_RETRY_COUNT * poll_delay_us,
            ADI_APOLLO_RAM_BOOT_STEP_DEVICE_CONFIG_CRC_CHECK, 0xff);
        ADI_APOLLO_ERROR_RETURN(err);

        if ((*status >= ADI_APOLLO_RAM_BOOT_STEP_DEVICE_CONFIG_CRC_CHECK) || (!desc->poll_read_returns_val)) {
            ADI_CMS_RETURN(API_CMS_ERROR_OK);
        }

        // Delay 'poll_delay_us' to wait before reading status again
        err = adi_apollo_hal_delay_us(device, poll_delay_us);
        ADI_APOLLO_ERROR_RETURN(err);

        retry_count--;
    } while (retry_count != 0U);

    ADI_CMS_RETURN(API_CMS_ERROR_ERROR);
}

static int32_t adi_apollo_arm_tye_bootstage_check(adi_apollo_device_t *device, uint8_t stage, uint8_t is_encrypted)
{
    int32_t err;
    uint8_t status = 0;
    uint8_t error;
    uint8_t expected_value;
    uint32_t bootstage_reg_addr;
    uint32_t error_reg_addr;
    uint32_t error_bf_info;
    uint8_t retry_count = ADI_APOLLO_ARM_BOOT_STATUS_READ_RETRY_COUNT;
    uint32_t poll_delay_us = 100000;    // 100ms
    uint8_t i;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(is_encrypted > 1U);
    ADI_APOLLO_INVALID_PARAM_RETURN(stage > 1U);

    expected_value = (is_encrypted == 0U) ? ADI_SECURE_BOOT_STAGE_CHECK : ADI_ENCR_SECURE_BOOT_STAGE_CHECK;
    bootstage_reg_addr = (stage == 0U) ? REG_SECURE_BOOT_STAGE0 : REG_SECURE_BOOT_STAGE1;
    error_reg_addr = (stage == 0U) ? REG_ADDR_EXTRACT(BF_ERROR_CODE_0_INFO) : REG_ADDR_EXTRACT(BF_ERROR_CODE_1_INFO);
    error_bf_info = (stage == 0U) ? BF_INFO_EXTRACT(BF_ERROR_CODE_0_INFO) : BF_INFO_EXTRACT(BF_ERROR_CODE_1_INFO);

    adi_apollo_hal_regio_t *desc = (adi_apollo_hal_regio_t *) device->hal_info.active_regio;

    /* Read secure_boot_stage_x status - wait for state 'expected_value' */
    for (i = 0; i < retry_count; i++) {

        err = adi_apollo_hal_reg_poll_get(device, bootstage_reg_addr, &status,
                                        retry_count * poll_delay_us, expected_value, 0xff);
        ADI_APOLLO_ERROR_RETURN(err);

        if ((status == expected_value) || (!desc->poll_read_returns_val)) {
            break;
        }

        // Wait 'poll_delay_us' before next poll read
        err = adi_apollo_hal_delay_us(device, poll_delay_us);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    err = adi_apollo_hal_bf_get(device, error_reg_addr, error_bf_info, &error, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_APOLLO_LOG_MSG_VAR( "SECURE_BOOT_STAGE_%d CHECK: %s.", stage, (status == expected_value) ? "Passed" : "*** FAILED ***");

    if (status != expected_value) {
        err = (int32_t)API_CMS_ERROR_ERROR;
    } else {
        err = (int32_t)API_CMS_ERROR_OK;
    }

    ADI_CMS_RETURN(err);
}

static int32_t adi_apollo_arm_boot_status_clear(adi_apollo_device_t *device)
{
    int32_t err;
    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    err = adi_apollo_hal_bf_set(device, BF_RAM_BOOT_CORE1_STATUS, 0);
    ADI_APOLLO_ERROR_RETURN(err)

    err = adi_apollo_hal_bf_set(device, BF_RAM_BOOT_CORE0_STATUS, 0);
    ADI_APOLLO_ERROR_RETURN(err)

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t adi_apollo_arm_core_reset(adi_apollo_device_t* device, uint8_t core)
{
    int32_t err;
    uint32_t reg_addr = 0;
    adi_apollo_hal_protocol_e protocol;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_INVALID_PARAM_RETURN(core > (uint8_t)ADI_APOLLO_CPU_ID_1);

    reg_addr = (core == (uint8_t)ADI_APOLLO_CPU_ID_0) ? REG_ARM0_RESET_ADDR : REG_ARM1_RESET_ADDR;

    /*
    Resetting ARM core w/ HSCI resets the uP_subsystem, which resets the HSCI configuration itself
    resulting in further HSCI transactions to not go through hence switching to SPI during the reset
    */
    err = adi_apollo_hal_active_protocol_get(device, &protocol);
    ADI_APOLLO_ERROR_RETURN(err);
    err = adi_apollo_hal_active_protocol_set(device, ADI_APOLLO_HAL_PROTOCOL_SPI0);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_reg_set(device, reg_addr, 1);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_delay_us(device, 100000);
    ADI_APOLLO_ERROR_RETURN(err);

    err = adi_apollo_hal_reg_set(device, reg_addr, 0);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Restore active protocol */
    err = adi_apollo_hal_active_protocol_set(device, protocol);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t adi_apollo_arm_ram_lock_check(adi_apollo_device_t *device)
{
    int32_t err;
    uint8_t ram_lock;
    uint8_t retry_count = ADI_APOLLO_ARM_EC_TRANSFER_STATUS_READ_RETRY_COUNT;
    uint32_t poll_delay_us = 10000;    // 10ms

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();

    adi_apollo_hal_regio_t *desc = (adi_apollo_hal_regio_t *) device->hal_info.active_regio;

    /* EC RAM lock status - wait for FW to release the EC ram lock */
    do {
        err = adi_apollo_hal_reg_poll_get(device, REG_ADDR_EXTRACT(BF_EC_RAM_LOCK_INFO), &ram_lock,
            ADI_APOLLO_ARM_EC_TRANSFER_STATUS_READ_RETRY_COUNT * poll_delay_us,
            0x00, BF_EC_RAM_LOCK_MASK);
        ADI_APOLLO_ERROR_RETURN(err);

        if (((ram_lock & BF_EC_RAM_LOCK_MASK) == 0U) || (desc->poll_read_returns_val == 0U)) {
            ADI_CMS_RETURN(API_CMS_ERROR_OK);
        }

        // Delay 'poll_delay_us' to wait before reading status again
        err = adi_apollo_hal_delay_us(device, poll_delay_us);
        ADI_APOLLO_ERROR_RETURN(err);

        retry_count--;
    } while (retry_count != 0U);

    ADI_CMS_RETURN(API_CMS_ERROR_EC_RAM_LOCK_ERROR);
}


static int32_t adi_apollo_arm_stack_ptr_boot_addr_set(adi_apollo_device_t *device, uint8_t core, adi_apollo_arm_binary_info_t *binary_info)
{
    int32_t err;

    uint8_t *fw_image_buf = binary_info->binary_ptr;
    uint32_t reg_addr = 0;
    uint32_t bf_info = 0;
    uint32_t reg_val = 0;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(binary_info);
    ADI_APOLLO_NULL_POINTER_RETURN(binary_info->binary_ptr);
    ADI_APOLLO_INVALID_PARAM_RETURN(binary_info->binary_sz_bytes < 1);
    ADI_APOLLO_INVALID_PARAM_RETURN(core > (uint8_t)ADI_APOLLO_CPU_ID_1);

    /* Extract Stack pointer and update */
    reg_val = ( ((uint32_t)fw_image_buf[3] << 24) | ((uint32_t)fw_image_buf[2] << 16) | ((uint32_t)fw_image_buf[1] << 8) | (uint32_t)fw_image_buf[0] );

    reg_addr = (core == (uint8_t)ADI_APOLLO_CPU_ID_0) ? REG_ADDR_EXTRACT(BF_ARM0_STACK_PTR_INFO) : REG_ADDR_EXTRACT(BF_ARM1_STACK_PTR_INFO);
    bf_info = (core == (uint8_t)ADI_APOLLO_CPU_ID_0) ? BF_INFO_EXTRACT(BF_ARM0_STACK_PTR_INFO) : BF_INFO_EXTRACT(BF_ARM1_STACK_PTR_INFO);

    err = adi_apollo_hal_bf_set(device, reg_addr, bf_info, reg_val);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_APOLLO_LOG_MSG_VAR("Core%d Stack Pointer: \treg_addr: 0x%08X.\treg_val: 0x%08X.", core, reg_addr, reg_val);


    /* Extract Boot address and update */
    reg_val = ( ((uint32_t)fw_image_buf[7] << 24) | ((uint32_t)fw_image_buf[6] << 16) | ((uint32_t)fw_image_buf[5] << 8) | (uint32_t)fw_image_buf[4] );

    reg_addr = (core == (uint8_t)ADI_APOLLO_CPU_ID_0) ? REG_ADDR_EXTRACT(BF_ARM0_BOOT_ADDR_INFO) : REG_ADDR_EXTRACT(BF_ARM1_BOOT_ADDR_INFO);
    bf_info = (core == (uint8_t)ADI_APOLLO_CPU_ID_0) ? BF_INFO_EXTRACT(BF_ARM0_BOOT_ADDR_INFO) : BF_INFO_EXTRACT(BF_ARM1_BOOT_ADDR_INFO);

    err = adi_apollo_hal_bf_set(device, reg_addr, bf_info, reg_val);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_APOLLO_LOG_MSG_VAR("Core%d Boot Addr: \treg_addr: 0x%08X.\treg_val: 0x%08X.", core, reg_addr, reg_val);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t get_cpu_device_profile_start_address(adi_apollo_device_t *device, uint32_t *cpu_device_profile_addr)
{
    uint32_t address = ADI_APOLLO_FW_DEVICE_PROFILE_PTR;
    int32_t err;

    err = adi_apollo_hal_reg32_get(device, address, cpu_device_profile_addr);
    ADI_APOLLO_ERROR_RETURN(err);
    ADI_APOLLO_LOG_MSG_VAR("Device Profile address: 0x%x\n", *cpu_device_profile_addr);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t device_feat_lockout_set(adi_apollo_device_t *device)
{
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);

    err = adi_apollo_hal_bf_get(device, BF_EC_RX_INFO, (uint8_t *)&device->dev_info.lockout_mask.rx_lockout_mask, sizeof(device->dev_info.lockout_mask.rx_lockout_mask));
    ADI_APOLLO_ERROR_RETURN(err);
    ADI_APOLLO_LOG_VAR(ADI_CMS_LOG_API, "Device feat lockout mask (rx): 0x%08x", device->dev_info.lockout_mask.rx_lockout_mask);

    err = adi_apollo_hal_bf_get(device, BF_EC_TX_INFO, (uint8_t *)&device->dev_info.lockout_mask.tx_lockout_mask, sizeof(device->dev_info.lockout_mask.tx_lockout_mask));
    ADI_APOLLO_ERROR_RETURN(err);
    ADI_APOLLO_LOG_VAR(ADI_CMS_LOG_API, "Device feat lockout mask (tx): 0x%08x", device->dev_info.lockout_mask.tx_lockout_mask);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_stream_buf_write(adi_apollo_device_t *device, uint8_t stream_image_buf[], uint32_t stream_image_size_bytes, uint16_t chunk_sz_bytes)
{
    adi_apollo_arm_binary_info_t binary_info;
    int32_t err;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(stream_image_buf);

    binary_info.binary_ptr = &stream_image_buf[0];
    binary_info.binary_sz_bytes = (int32_t)stream_image_size_bytes;
    binary_info.cpu_start_addr = 0;
    binary_info.is_cont = 0;

    err = adi_apollo_arm_stream_load(device, &binary_info, chunk_sz_bytes);
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

int32_t adi_apollo_arm_stream_load(adi_apollo_device_t *device, adi_apollo_arm_binary_info_t *binary_info, uint16_t chunk_sz_bytes)
{
    int32_t err;
    uint32_t cpu_address_to_load;
    int32_t pending_bytes;
    uint8_t *data_ptr;

    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(binary_info);
    ADI_APOLLO_NULL_POINTER_RETURN(binary_info->binary_ptr);
    /* binary_sz_bytes must be >= 4: the chunk-adjustment formula
     * (sz - sz%4) produces 0 for inputs 1-3, causing div-by-zero in the
     * transfer loop. All valid stream binaries are at least 348 bytes,
     * so this is not a practical restriction. */
    ADI_APOLLO_INVALID_PARAM_RETURN(binary_info->binary_sz_bytes < 4);
    ADI_APOLLO_INVALID_PARAM_RETURN(chunk_sz_bytes < 1U || chunk_sz_bytes > (16 * 1024U)); // A0 has max stream limit of 16k
    ADI_APOLLO_INVALID_PARAM_RETURN((chunk_sz_bytes % 4U) != 0U);

    pending_bytes = binary_info->binary_sz_bytes;

    /* chunk size should not be bigger then total size and should always be % 4 */
    if ((int32_t)chunk_sz_bytes > binary_info->binary_sz_bytes) {
        int32_t aligned_size = binary_info->binary_sz_bytes - (binary_info->binary_sz_bytes % 4);
        chunk_sz_bytes = (uint16_t)aligned_size;
    }

    cpu_address_to_load = binary_info->cpu_start_addr;

    data_ptr = binary_info->binary_ptr;

    ADI_APOLLO_LOG_MSG_VAR("Loading stream %d bytes", pending_bytes);

    adi_apollo_stream_info_t stream_state_info;

    /* Send data in chunk_sz_bytes */
    while ((pending_bytes > 0) && (((int32_t)(pending_bytes / (int32_t)chunk_sz_bytes)) > 0)) {
        err = adi_apollo_arm_stream_image_write(device, cpu_address_to_load, data_ptr, chunk_sz_bytes, &stream_state_info);
        ADI_APOLLO_ERROR_RETURN(err);

        pending_bytes = (int32_t)(pending_bytes - (int32_t)chunk_sz_bytes);
        binary_info->is_cont = 1;
        data_ptr += chunk_sz_bytes;
        cpu_address_to_load += chunk_sz_bytes;
    }

    /* send remaining bytes */
    if (pending_bytes > 0) {
        err = adi_apollo_arm_stream_image_write(device, cpu_address_to_load, data_ptr, (uint32_t)pending_bytes, &stream_state_info);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


/**
* Based off the structure of the stream_image.bin this function extracts
* the size and offset of a single slice stream processor.
*/
static void adi_apollo_arm_stream_image_info_get(const uint8_t   stream_image[],
                                                 uint32_t* const stream_image_size,
                                                 uint32_t* const stream_image_offset)
{
    /* populating variables from stream image binary array */
    *stream_image_size   = ((((uint32_t)stream_image[3U]) << 24U) | (((uint32_t)stream_image[2U]) << 16U) | (((uint32_t)stream_image[1U]) << 8U) | ((uint32_t)stream_image[0U]));
    *stream_image_offset = ((((uint32_t)stream_image[7U]) << 24U) | (((uint32_t)stream_image[6U]) << 16U) | (((uint32_t)stream_image[5U]) << 8U) | ((uint32_t)stream_image[4U]));
}

/**
* Based off the structure of the stream_image.bin this function extracts
* the base address of a single slice stream processor.
*/
static void adi_apollo_arm_stream_image_get_base_addr(const uint8_t stream_image[], uint32_t* const base_addr)
{
    /* populating variables from stream image binary array */
    *base_addr = (((uint32_t)stream_image[3U]) << 24U) | (((uint32_t)stream_image[2U]) << 16U) | (((uint32_t)stream_image[1U]) << 8U) | (uint32_t)(stream_image[0U]);
}

/**
* Based off the structure of the stream_image.bin this function extracts
* the number of streams and slice image size.
*/
static void adi_apollo_arm_stream_image_get_number_stream_image_size(const uint8_t   stream_image[],
                                                                     uint8_t* const  number_streams,
                                                                     uint32_t* const image_size)
{
    *number_streams = stream_image[0U];
    *image_size     = (((uint32_t)stream_image[3U]) << 8) | (uint32_t)(stream_image[2U]);
}


#define REG_TX0_STREAM_CONFIG_0_ADDR            (0x6133D000)
#define REG_TX1_STREAM_CONFIG_0_ADDR            (0x6153D000)
#define REG_TX2_STREAM_CONFIG_0_ADDR            (0x61B3D000)
#define REG_TX3_STREAM_CONFIG_0_ADDR            (0x61D3D000)
#define REG_RX0_STREAM_CONFIG_0_ADDR            (0x603BC000)
#define REG_RX1_STREAM_CONFIG_0_ADDR            (0x605BC000)
#define REG_RX2_STREAM_CONFIG_0_ADDR            (0x60BBC000)
#define REG_RX3_STREAM_CONFIG_0_ADDR            (0x60DBC000)

#define ADI_APOLLO_TX_INITIALIZED_CH_OFFSET     (8)

#define APOLLO_STREAM_RESET_BIT                 (0)
#define APOLLO_RX_STREAM_RESET_BIT              (16)

#define APOLLO_RX_STREAM_BASE_BYTE0_BIT         (0)
#define APOLLO_RX_STREAM_BASE_BYTE1_BIT         (8)
#define APOLLO_RX_LAST_STREAM_NUMBER_BIT        (16)

#define APOLLO_STREAM_BASE_BYTE0_REG_OFFSET     (1U)
#define APOLLO_STREAM_BASE_BYTE1_REG_OFFSET     (2U)
#define APOLLO_LAST_STREAM_NUMBER_REG_OFFSET    (3U)

static int32_t adi_apollo_arm_stream_image_write(adi_apollo_device_t *device, uint32_t byte_offset, uint8_t *binary, uint32_t byte_count, adi_apollo_stream_info_t *stream_state)
{
    int32_t  err = (int32_t)API_CMS_ERROR_OK;
    uint32_t binary_image_size = 0U;
    uint32_t binary_image_offset = 0U;
    uint32_t i = 0U;
    uint32_t addr = 0U;
    uint32_t mask = 0U;
    uint32_t base_addr = 0U;
    uint32_t bin_offset = 0U;
    uint32_t stream_ctrl = 0U;
    uint8_t  number_streams = 0U;
    uint32_t image_size = 0U;
    uint32_t image_offset = 0U;
    uint32_t stream_ctrl_addr[] = {
        REG_STREAM_CONTROL_ADDR,
        REG_TX0_STREAM_CONFIG_0_ADDR,
        REG_TX1_STREAM_CONFIG_0_ADDR,
        REG_TX2_STREAM_CONFIG_0_ADDR,
        REG_TX3_STREAM_CONFIG_0_ADDR,
        REG_RX0_STREAM_CONFIG_0_ADDR,
        REG_RX1_STREAM_CONFIG_0_ADDR,
        REG_RX2_STREAM_CONFIG_0_ADDR,
        REG_RX3_STREAM_CONFIG_0_ADDR
    };

    /* Default value assigned by hardware */
    uint32_t core_slice_control_default_val                     = 0x80U;

    static const uint32_t STREAM_IMAGE_SLICE_INFO_START_BYTE    = 24U;
    static const uint32_t STREAM_PROCESSOR_META_DATA_SIZE_BYTES = 8U;
    static const uint32_t FINAL_STREAM_IMAGE_HEADER_SIZE_BYTES  = 0x15CU;
    
    ADI_APOLLO_NULL_POINTER_RETURN(device);
    ADI_APOLLO_LOG_FUNC();
    ADI_APOLLO_NULL_POINTER_RETURN(stream_state);
    ADI_APOLLO_INVALID_PARAM_RETURN((byte_count == 0U) || ((byte_count % 4U) > 0U));
    ADI_APOLLO_INVALID_PARAM_RETURN((byte_offset % 4U) != 0U);
    /* First call (byte_offset == 0) must supply at least the full header so that all
     * slice metadata can be parsed in a single pass. Subsequent calls may be smaller. */
    ADI_APOLLO_INVALID_PARAM_RETURN((byte_offset == 0U) && (byte_count < FINAL_STREAM_IMAGE_HEADER_SIZE_BYTES));

    /* When byte_offset is zero that indicates the first time this function
     * has been called for a particular stream_image.bin so the header info
     * must be extracted. We need all the information extracted in a single
     * call which is why min chunk size is the same as FINAL_STREAM_IMAGE_HEADER_SIZE_BYTES.
     * */
    if (byte_offset == 0U)
    {
        for (i = 0U; i < ADI_APOLLO_STREAM_MAX; ++i)
        {
            adi_apollo_arm_stream_image_info_get(&binary[STREAM_IMAGE_SLICE_INFO_START_BYTE + i * STREAM_PROCESSOR_META_DATA_SIZE_BYTES], &binary_image_size, &binary_image_offset);
            stream_state->chunk_stream_image_size[i]   = binary_image_size;
            stream_state->chunk_stream_image_offset[i] = binary_image_offset;
        }

        /* Variables used to track the status of stream_image.bin parsing
         * across multiple calls to this function. 
         * */
        stream_state->current_stream_image_index    = 0U;
        stream_state->current_stream_bin_base_addr  = 0U;
        stream_state->current_stream_base_addr      = 0U;
        stream_state->current_stream_image_size     = 0U;
        stream_state->current_stream_number_streams = 0U;
    }

    for (i = stream_state->current_stream_image_index; i < ADI_APOLLO_STREAM_MAX; ++i)
    {
        image_offset = stream_state->chunk_stream_image_offset[i];
        
        /* NOTE: the i == 0 if-statements are for the most part required because core
         * slice control registers are in Direct SPI address range instead of AHB memory. 
         * */
        
        /* The core processor binary header starts after the stream_image.bin header.
         * But what gets written to the core AHB address includes the stream_image.bin
         * header. */
        if (i == 0U)
        {
            image_offset = FINAL_STREAM_IMAGE_HEADER_SIZE_BYTES;
        }

        /* The if-statements below are to see if the current chunk of stream_image.bin
         * includes certain necessary information for a particular slice, e.g. the binary
         * base address which is the first word of a slice binary image. */
        if ((byte_offset <= image_offset) &&
            ((byte_offset + byte_count) >= image_offset + 4U))
        {
            bin_offset = image_offset - byte_offset;
            adi_apollo_arm_stream_image_get_base_addr(&binary[bin_offset], &base_addr);
            stream_state->current_stream_bin_base_addr = base_addr;
            stream_state->current_stream_image_size = stream_state->chunk_stream_image_size[i];
        }

        if ((byte_offset <= image_offset + 4U) &&
            ((byte_offset + byte_count) >= image_offset + 8U))
        {
            bin_offset = image_offset + 4U - byte_offset;
            adi_apollo_arm_stream_image_get_base_addr(&binary[bin_offset], &base_addr);
            stream_state->current_stream_base_addr = base_addr;
        }

        if ((byte_offset <= image_offset + 8U) &&
            ((byte_offset + byte_count) >= image_offset + 12U))
        {
            bin_offset = image_offset + 8U - byte_offset;
            adi_apollo_arm_stream_image_get_number_stream_image_size(&binary[bin_offset], &number_streams, &base_addr);
            stream_state->current_stream_number_streams = number_streams;
        }

        /* If these tracking variables have been set for the current slice that means we are in a state
         * where we can write the chunk to ARM memory.
         * */
        if ((stream_state->current_stream_bin_base_addr > 0U) &&
            (stream_state->current_stream_image_size > 0U))
        {
            addr = stream_ctrl_addr[i];

            /* This if-statement will only be true the first time we have encounter a slice image because
             * current_stream_image_size gets decremented on each function call, so we can use this as a way
             * to initialize slice startup items, e.g. putting the slice into reset while it's image is getting
             * written. */
            if (stream_state->current_stream_image_size == stream_state->chunk_stream_image_size[i])
            {
                /* maint stream's stream_reset is in spi-only regmap */
                if (i == 0U)
                {
                    stream_ctrl = core_slice_control_default_val | (1U << APOLLO_STREAM_RESET_BIT);
                    mask = 0xFFU;
                    if (err = adi_apollo_hal_reg_set(device, addr, (uint8_t)(stream_ctrl & mask)), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set main Stream Ctrl\n");
                        ADI_CMS_RETURN(err);
                    }
                }
                else
                {
                    /* Default for Rx stream_config_0 is zero */
                    stream_ctrl = (1U << APOLLO_RX_STREAM_RESET_BIT);
                    mask = 0xFFFFFFFFU;
                    if (err = adi_apollo_hal_reg32_set(device, addr, stream_ctrl & mask), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl\n");
                        ADI_CMS_RETURN(err);
                    }
                }
            }

            /* This if statement is in the case a byte chunk has a boundary between two slice images so we only write
             * the remaining bytes of the current slice. 
             * */
            if (byte_count >= stream_state->current_stream_image_size)
            {
                image_size = stream_state->current_stream_image_size;
            }
            else
            {
                image_size = byte_count;
            }

            /* Load the stream image to memory, starting at the memory address specified by Stream_binary_base
             * if the corresponding channel has been initialized. 
             * */
            adi_apollo_arm_binary_info_t binary_info;
            binary_info.cpu_start_addr  = stream_state->current_stream_bin_base_addr;
            binary_info.binary_ptr      = (uint8_t *)binary;
            binary_info.binary_sz_bytes = (int32_t)image_size;
            binary_info.is_cont         = 0;
            /* image_size is bounded by byte_count which comes from chunk_sz_bytes (uint16_t),
             * so it always fits in uint16_t. Cast explicitly to document the intent and
             * suppress implicit-conversion warnings under stricter build flags. */
            err = adi_apollo_arm_memload(device, &binary_info, (uint16_t)image_size);
            if (err != (int32_t)API_CMS_ERROR_OK)
                ADI_APOLLO_LOG_MSG("Failed to write stream binary\n");
            ADI_APOLLO_ERROR_RETURN(err);

            byte_count  -= image_size;
            byte_offset += image_size;
            binary      += image_size;
            stream_state->current_stream_image_size    -= image_size;
            stream_state->current_stream_bin_base_addr += image_size;

            /* After an entire slice image has been written we need to set the corresponding stream control register. */
            if (stream_state->current_stream_image_size == 0U)
            {
                /* maint stream's stream_reset is in spi-only regmap */
                if (i == 0U)
                {
                    stream_ctrl = core_slice_control_default_val;
                    mask = 0xFFU;
                    
                    /* Populate registers(stream_base_byte0) and (stream_base_byte1) with the lower 16 bits of the Stream_base. */
                    if (err = adi_apollo_hal_reg_set(device, addr + APOLLO_STREAM_BASE_BYTE0_REG_OFFSET, (uint8_t)(stream_state->current_stream_base_addr & mask)), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl - Lower Byte\n");
                        ADI_CMS_RETURN(err);
                    }

                    if (err = adi_apollo_hal_reg_set(device, addr + APOLLO_STREAM_BASE_BYTE1_REG_OFFSET, (uint8_t)((stream_state->current_stream_base_addr >> 8U) & mask)), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl - Upper Byte\n");
                        ADI_CMS_RETURN(err);
                    }

                    /* Populate register  (last_stream_num)with the No_of_streams value. */
                    if (err = adi_apollo_hal_reg_set(device, addr + APOLLO_LAST_STREAM_NUMBER_REG_OFFSET, (uint8_t)((stream_state->current_stream_number_streams - 1U) & mask)), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl - Number of Streams\n");
                        ADI_CMS_RETURN(err);
                    }

                    if (err = adi_apollo_hal_reg_set(device, addr, (uint8_t)(stream_ctrl & mask)), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl\n");
                        ADI_CMS_RETURN(err);
                    }
                }
                else 
                {
                    mask = 0xFFFFFFFFU;
                    stream_ctrl = 0U;
                    
                    /* Populate registers(stream_base_byte0) and (stream_base_byte1)with the lower 16 bits of the Stream_base. */
                    stream_ctrl |= (stream_state->current_stream_base_addr & 0xFFU) << APOLLO_RX_STREAM_BASE_BYTE0_BIT;
                    stream_ctrl |= ((stream_state->current_stream_base_addr >> 8U) & 0xFFU) << APOLLO_RX_STREAM_BASE_BYTE1_BIT;

                    /* Populate register  (last_stream_num)with the No_of_streams value. */
                    stream_ctrl |= ((stream_state->current_stream_number_streams - 1U) & 0xFFU) << APOLLO_RX_LAST_STREAM_NUMBER_BIT;
                    
                    /* Stream config 1 holds the base address & last stream number */
                    if (err = adi_apollo_hal_reg32_set(device, addr + 4U, stream_ctrl & mask), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl 1\n");
                        ADI_CMS_RETURN(err);
                    }

                    /* Just clear reset bit on other channel processors */
                    stream_ctrl = 0U;

                    if (err = adi_apollo_hal_reg32_set(device, addr, stream_ctrl & mask), err != (int32_t)API_CMS_ERROR_OK)
                    {
                        ADI_APOLLO_LOG_MSG("Failed to set Stream Ctrl\n");
                        ADI_CMS_RETURN(err);
                    }
                }

                stream_state->current_stream_image_index++;
                stream_state->current_stream_bin_base_addr  = 0U;
                stream_state->current_stream_base_addr      = 0U;
                stream_state->current_stream_image_size     = 0U;
                stream_state->current_stream_number_streams = 0U;
            }
            else
            {
                /* stream is disabled but no need to report an error */
                err = (int32_t)API_CMS_ERROR_OK;
            }
        }
        else
        {
            break;
        }

        if (byte_count == 0U)
        {
            break;
        }
    }

    ADI_CMS_RETURN(err);
}

/*! @} */