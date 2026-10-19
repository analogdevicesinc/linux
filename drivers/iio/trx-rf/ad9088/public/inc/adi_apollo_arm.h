/*!
 * \brief     Apollo ARM headers
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
#ifndef ADI_APOLLO_ARM_H__
#define ADI_APOLLO_ARM_H__

/*============= I N C L U D E S ============*/
#include "adi_apollo_arm_types.h"

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief  Clear firmware boot status.
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 * \param[out] status           Boot status - Pointer to boot status variable
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_boot_status(adi_apollo_device_t *device, uint8_t *status);

/**
 * \brief  CPU ram boot error check.
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_ram_boot_error_check(adi_apollo_device_t *device);

/**
 * \brief  Get firmware profile CRC status
 *
 * \param[in]  device Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return API_CMS_ERROR_PROFILE_CRC            Invalid profile CRC
 */
int32_t adi_apollo_arm_profile_crc_valid_get(adi_apollo_device_t *device);

/**
 * \brief  CPU rom ec transfer validation.
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_rom_ec_transfer_check(adi_apollo_device_t *device);

/**
 * \brief  Prapare cpu for firmware loading.
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fwload_pre_config(adi_apollo_device_t *device);

/**
 * \brief  Post firmware loading setup to boot cpu using uploaded fw binary.
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fwload_post_config(adi_apollo_device_t *device);

/**
 * \brief  Load memory to CPU ram
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 * \param[in]  binary_info      Pointer to the cpu binary loading config structure
 * \param[in]  chunk_sz_bytes   Number of bytes to transfer to CPU at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_memload(adi_apollo_device_t *device, adi_apollo_arm_binary_info_t *binary_info, uint16_t chunk_sz_bytes);

/**
 * \brief  Load device profile to CPU
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 * \param[in]  profile          adi_apollo_top_t structure pointer
 * \param[in]  chunk_sz_bytes   Number of bytes to transfer to CPU at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_profile_write(adi_apollo_device_t *device, adi_apollo_top_t *profile, uint16_t chunk_sz_bytes);

/**
 * \brief  Send device profile memory transfer complete indication to firmware
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 * \param[in]  profile          adi_apollo_top_t structure pointer
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_profile_write_post_config(adi_apollo_device_t *device, adi_apollo_top_t *profile);

/**
 * \brief  Load device firmware to CPU
 *
 * \param[in]  device               Context variable - Pointer to the APOLLO device data structure
 * \param[in]  core                 Target core. \ref adi_apollo_cpu_id_e
 * \param[in]  fw_image_buf         Firmware binary byte array
 * \param[in]  fw_image_size_bytes  Firmware image size
 * \param[in]  chunk_sz_bytes       Number of bytes to transfer to CPU at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_firmware_buf_write(adi_apollo_device_t *device, uint8_t core, uint8_t fw_image_buf[], uint32_t fw_image_size_bytes, uint16_t chunk_sz_bytes);

/**
 * \brief  Load device firmware to CPU
 *
 * \param[in]  device               Context variable - Pointer to the APOLLO device data structure
 * \param[in]  mem_addr_to_load     Memory address where the firware buffer needs to be written.
 * \param[in]  fw_image_buf         Firmware binary byte array
 * \param[in]  fw_image_size_bytes  Firmware image size
 * \param[in]  chunk_sz_bytes       Number of bytes to transfer to CPU at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_tye_firmware_buf_write(adi_apollo_device_t *device, uint32_t mem_addr_to_load, uint8_t fw_image_buf[], uint32_t fw_image_size_bytes, uint16_t chunk_sz_bytes);

/**
 * \brief  Load device profile byte array to CPU
 *
 * \param[in]  device               Context variable - Pointer to the APOLLO device data structure
 * \param[in]  profile_buf          Device profile byte array
 * \param[in]  profile_size_bytes   Device profile buffer size
 * \param[in]  chunk_sz_bytes       Number of bytes to transfer to CPU at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_profile_buf_write(adi_apollo_device_t *device, uint8_t profile_buf[], uint32_t profile_size_bytes, uint16_t chunk_sz_bytes);

/**
 * \brief  Check if Tiny Enclave is bypassed or not
 *
 * \param[in]   device          Context variable - Pointer to the APOLLO device data structure
 * \param[out]  is_bypassed     1 = Tiny Enclave is bypassed, load standard binary. 0 = not bypassed, load TyE App pack binaries.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_tye_bypassed_get(adi_apollo_device_t *device, uint8_t *is_bypassed);

/**
 * \brief   Triggers the arm cores to boot TyE FW
 *
 * \param[in]   device          Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_tye_boot_ready_set(adi_apollo_device_t *device);

/**
 * \brief   Validate if secure boot passes
 *
 * \param[in]   device          Context variable - Pointer to the APOLLO device data structure
 * \param[in]   is_encrypted    FW in App pack is encrypted(1) or not(0)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_tye_validate_boot_completion(adi_apollo_device_t *device, uint8_t is_encrypted);

/**
 * \brief   Get Firmware error codes
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  cpu_errs    Struct containing firmware error codes
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_err_codes_get(adi_apollo_device_t *device, adi_apollo_cpu_errors_t *cpu_errs);

/**
 * \brief  Get FW IRQ OUT status 
 *
 * \param[in]  device            Context variable - Pointer to the APOLLO device data structure
 * \param[out] fw_irq_out        Status of FW interrupt triggering - 1 triggered, 0 not triggered
 * \param[out] irq_source        Source of IRQ trigger \ref adi_apollo_fw_irq_out_source_e
 * 
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_get(adi_apollo_device_t *device, uint8_t *fw_irq_out, uint32_t *irq_source);

/**
 * \brief  Clear FW IRQ sticky bits
 *
 * \param[in] device            Context variable - Pointer to the APOLLO device data structure
 * 
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_clear(adi_apollo_device_t *device);

/**
 * \brief  Get the FW_IRQ_OUT active-source bitmask (via mailbox)
 *
 * \details Returns the 32-bit "source" register: which sources are currently active (sticky;
 *          the firmware never clears them by itself). The pin is asserted only when
 *          (source & config) != 0 - see \ref adi_apollo_arm_fw_irq_out_config_get.
 *          Source bits are defined in \ref adi_apollo_fw_irq_out_source_e (e.g.
 *          \ref ADI_APOLLO_FW_IRQ_OUT_SOURCE_TEMP_ALARM_HIGH / _LOW).
 *          This is the mailbox path (selective mask + firmware validation); the SPI-scratch path
 *          \ref adi_apollo_arm_fw_irq_out_get remains available for a fast local read.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[out] source_mask   FW_IRQ_OUT active-source bitmask (32-bit)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_source_get(adi_apollo_device_t *device, uint32_t *source_mask);

/**
 * \brief  Clear selected bits in the FW_IRQ_OUT active-source bitmask (via mailbox)
 *
 * \details The source register is sticky; the host is responsible for clearing it. After clearing,
 *          the pin drops if no (active & enabled) bit remains. If the underlying condition persists
 *          (e.g. temperature still out of range) the source re-asserts on the next measurement.
 *
 * \param[in]  device          Context variable - Pointer to the APOLLO device data structure
 * \param[in]  clear_mask      Bitmask of active bits to clear (32-bit)
 * \param[out] remaining_mask  Remaining active-source bitmask after the clear (32-bit)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_source_clear(adi_apollo_device_t *device, uint32_t clear_mask, uint32_t *remaining_mask);

/**
 * \brief  Get the FW_IRQ_OUT config (enable) bitmask (via mailbox)
 *
 * \details Returns the 32-bit "config" register: which sources are enabled to drive the pin.
 *          The pin is asserted only when (source & config) != 0. The config register powers up at
 *          0x00 (strict opt-in): until a source is enabled the pin never asserts.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[out] config_mask   FW_IRQ_OUT config (enable) bitmask (32-bit)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_config_get(adi_apollo_device_t *device, uint32_t *config_mask);

/**
 * \brief  Enable FW_IRQ_OUT sources (via mailbox)
 *
 * \details Sets the given bits in the config (enable) register; the firmware performs the
 *          read-modify-write under a critical region. Because config powers up at 0x00, this call
 *          is mandatory after configuring a source for its alarm to reach the pin, e.g. for the
 *          temperature alarm: enable_mask = (TEMP_ALARM_HIGH | TEMP_ALARM_LOW) = 0xC0.
 *          The command is validated: if enable_mask contains bits outside the supported sources
 *          (0xFF) nothing changes and the firmware returns
 *          \ref APOLLO_CPU_DEV_TEMP_SENSOR_INVALID_CONFIG_MASK_ERROR.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[in]  enable_mask   Bits to enable in the config bitmask (32-bit)
 * \param[out] config_mask   Resulting config (enable) bitmask after the update (32-bit)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_sources_enable(adi_apollo_device_t *device, uint32_t enable_mask, uint32_t *config_mask);

/**
 * \brief  Disable FW_IRQ_OUT sources (via mailbox)
 *
 * \details Clears the given bits in the config (enable) register; the firmware performs the
 *          read-modify-write under a critical region. This call is lenient: clearing unknown bits
 *          is a no-op and returns no error.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[in]  disable_mask  Bits to disable in the config bitmask (32-bit)
 * \param[out] config_mask   Resulting config (enable) bitmask after the update (32-bit)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_fw_irq_out_sources_disable(adi_apollo_device_t *device, uint32_t disable_mask, uint32_t *config_mask);

/**
 * \brief  Load device stream image to CPU
 *
 * \param[in]  device                   Context variable - Pointer to the APOLLO device data structure
 * \param[in]  stream_image_buf         Stream binary byte array
 * \param[in]  stream_image_size_bytes  Stream image size
 * \param[in]  chunk_sz_bytes           Number of bytes to transfer to stream at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_stream_buf_write(adi_apollo_device_t *device, uint8_t stream_image_buf[], uint32_t stream_image_size_bytes, uint16_t chunk_sz_bytes);

/**
 * \brief  Load memory to stream ram
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 * \param[in]  binary_info      Pointer to the stream binary loading config structure
 * \param[in]  chunk_sz_bytes   Number of bytes to transfer to stream at a time
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_arm_stream_load(adi_apollo_device_t *device, adi_apollo_arm_binary_info_t *binary_info, uint16_t chunk_sz_bytes);


#ifdef __cplusplus
}
#endif

#endif /* ADI_APOLLO_ARM_H__ */
/*! @} */
