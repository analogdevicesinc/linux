/*!
 * \brief     API header file
 *            This file contains all the publicly exposed methods and data
 *            structures to interface with API.
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_DEVICE
 * @{
 */
#ifndef ADI_APOLLO_DEVICE_H__
#define ADI_APOLLO_DEVICE_H__

/*============= I N C L U D E S ============*/
#include "adi_apollo_common.h"
#include "adi_apollo_device_types.h"

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief  Perform SPI register write access to device
 *
 * \param[in]  device   Context variable - Pointer to the APOLLO device data structure
 * \param[in]  addr     SPI address to which the value of data parameter shall be written
 * \param[in]  data     8-bit value to be written to SPI register defined by the address parameter.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_spi_register_set(adi_apollo_device_t *device, uint32_t addr, uint8_t data);

/**
 * \brief  Perform SPI register read access from device
 *
 * \param[in]  device   Context variable - Pointer to the APOLLO device data structure
 * \param[in]  addr     SPI address from which the value of data parameter shall be read,
 * \param[out]  data     Pointer to an 8-bit variable to which the value of the
 *                       SPI register at the address defined by address parameter shall be stored.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_spi_register_get(adi_apollo_device_t *device, uint32_t addr, uint8_t *data);

/**
 * \brief  Open hardware platform
 *         Just call user callback for function pointer 'hw_open'. Please note this is optional if user
 *         configure hardware platform by themselves.
 * 
 * \param[in]  device Context variable - Pointer to the APOLLO device data structure
 * \param[in]  reset_opt A parameter to indicate reset operation. \ref adi_apollo_reset_e
 *                   ADI_APOLLO_NO_RESET indicates no reset is required
 *                   ADI_APOLLO_SOFT_RESET indicates a software reset is required.
 *                   ADI_APOLLO_HARD_RESET indicates a hardware reset is required.
 *                   ADI_APOLLO_SOFT_RESET_AND_INIT indicates a software reset + initialization is required.
 *                   ADI_APOLLO_HARD_RESET_AND_INIT indicates a hardware reset + initialization is required.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_hw_open(adi_apollo_device_t *device, adi_apollo_reset_e reset_opt);

/**
 * \brief  Close hardware platform
 *         Just call user callback for function pointer 'hw_close'. Please note this is optional if user
 *         close hardware platform by themselves.
 *
 * \param[in]  device Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_hw_close(adi_apollo_device_t *device);

/**
 * \brief  Initialize apollo device
 *         This API will configure device SPI working mode, and check power supplies status. Must be called
 *         after platform SPI master is already initialized and adi_apollo_device_reset() is called.
 *
 * \param[in]  device Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_init(adi_apollo_device_t *device);

/**
 * \brief  De-initialize device
 *		   This API will do hard then soft reset.
 *
 * \param[in]  device Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_deinit(adi_apollo_device_t *device);

/**
 * \brief  Reset device
 *         Issues a hard reset or soft reset of the device.
 *         Performs a full reset of device via the hardware pin (hard) or
 *         via the SPI register (soft).
 *         Resetting all SPI registers to default and triggering the required
 *         initialization sequence.
 *         adi_apollo_device_init() will be called if operation is ADI_APOLLO_SOFT_RESET_AND_INIT or
 *         ADI_APOLLO_HARD_RESET_AND_INIT.
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[in]  reset_opt A parameter to indicate reset operation. \ref adi_apollo_reset_e
 *                   ADI_APOLLO_NO_RESET indicates no reset is required
 *                   ADI_APOLLO_SOFT_RESET indicates a software reset is required.
 *                   ADI_APOLLO_HARD_RESET indicates a hardware reset is required.
 *                   ADI_APOLLO_SOFT_RESET_AND_INIT indicates a software reset + initialization is required.
 *                   ADI_APOLLO_HARD_RESET_AND_INIT indicates a hardware reset + initialization is required.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_reset(adi_apollo_device_t *device, adi_apollo_reset_e reset_opt);

/**
 * \brief  Get chip identification data
 *         Read-back product type, identification and revision data.
 *
 * \param[in]  device   Context variable - Pointer to the APOLLO device data structure
 * \param[out]  chip_id  \ref adi_cms_chip_id_t
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_chip_id_get(adi_apollo_device_t *device, adi_cms_chip_id_t *chip_id);

/**
 * \brief  Get API revision
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[out]  rev_major Pointer to variable to store the major revision
 * \param[out]  rev_minor Pointer to variable to store the minor revision
 * \param[out]  rev_rc    Pointer to variable to store the rc    revision
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_api_revision_get(adi_apollo_device_t *device, uint16_t *rev_major,
                                           uint16_t *rev_minor, uint16_t *rev_rc);

/**
 * \brief  Get Laminate ID
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[out]  id        Pointer to silicon laminate id.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_laminate_id_get(adi_apollo_device_t *device, uint8_t *id);

/**
 * \brief  Get DIE ID
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[out]  id        Pointer to silicon DIE id.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_die_id_get(adi_apollo_device_t *device, uint8_t *id);

/**
 * \brief  Get Silicon Grade ID
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  si_grade    Pointer to Silicon Grade ID.
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_si_grade_get(adi_apollo_device_t *device, uint8_t *si_grade);

/**
 * \brief  Get sw variant Tx/Rx/Mxfe info
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  mxfe    Pointer to sw variant tx/rx/mxfe info. \ref adi_apollo_device_sw_variant_mxfe_e
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_sw_mxfe_get(adi_apollo_device_t *device, adi_apollo_device_sw_variant_mxfe_e *mxfe);

/**
 * \brief  Get sw trim info
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  sw_trim    Pointer to sw trim info. \ref adi_apollo_device_sw_variant_trim_e
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_sw_trim_get(adi_apollo_device_t *device, adi_apollo_device_sw_variant_trim_e *sw_trim);

/**
 * \brief  Get trx config info
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  trx_cfg    Pointer to trx config. \ref adi_apollo_device_rev_trx_config_e
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_trx_config_get(adi_apollo_device_t *device, adi_apollo_device_rev_trx_config_e *trx_cfg);

/**
 * \brief  Get rx type info
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  rx_type    Pointer to rx type. \ref adi_apollo_device_rev_rx_type_e
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_rx_type_get(adi_apollo_device_t *device, adi_apollo_device_rev_rx_type_e *rx_type);

/**
 * \brief  Get board variant and rev info
 *
 * \param[in]   device      Context variable - Pointer to the APOLLO device data structure
 * \param[out]  board_var_rev    Pointer to board variant and rev info. \ref adi_apollo_device_board_var_and_rev_info_t
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_board_variant_and_rev_get(adi_apollo_device_t *device, adi_apollo_device_board_var_and_rev_info_t *board_var_rev);

/**
 * \brief  Emulate Tx-only or Rx-only board variant
 *         Writes the requested variant to a dedicated scratch register
 *         so the FW can override the fuse-determined part type.
 *         Only TX_ONLY and RX_ONLY are accepted; MXFE is rejected.
 *
 * \param[in]  device   Context variable - Pointer to the APOLLO device data structure
 * \param[in]  mxfe     Variant to emulate (TX_ONLY or RX_ONLY only). \ref adi_apollo_device_sw_variant_mxfe_e
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_txrx_variant_emulate(adi_apollo_device_t *device, adi_apollo_device_sw_variant_mxfe_e mxfe);

/**
 * \brief  Emulate SW trim level
 *         Writes the requested trim level to a dedicated scratch register
 *         so the FW can override the fuse-determined trim setting.
 *         Only SW1, SW3, and SW5 are accepted; NOT_SET is rejected.
 *
 * \param[in]  device   Context variable - Pointer to the APOLLO device data structure
 * \param[in]  sw_trim  SW trim level to emulate (SW1, SW3, or SW5). \ref adi_apollo_device_sw_variant_trim_e
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_sw_trim_emulate(adi_apollo_device_t *device, adi_apollo_device_sw_variant_trim_e sw_trim);

/**
 * \brief Configure SPI settings
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  spi_config   Ptr to spi configuration \ref adi_apollo_device_spi_settings_t
 * \param[in]  spi_num      Spi num (0 = SPI0, 1 = SPI1)
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_spi_configure(adi_apollo_device_t *device, adi_apollo_device_spi_settings_t *spi_config, uint8_t spi_num);

/**
 * \brief Configure HSCI settings
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  hsci_config  Ptr to hsci configuration \ref adi_apollo_device_hsci_settings_t
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_hsci_configure(adi_apollo_device_t *device, adi_apollo_device_hsci_settings_t *hsci_config);

/**
 * \brief  Get UUID
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[out]  uuid        Array of uuid
 * \param[in]  uuid_len     Length of uuid array param. Must be set to ADI_APOLLO_UUID_NUM_BYTES
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_uuid_get(adi_apollo_device_t *device, uint8_t uuid[], uint32_t uuid_len);

/**
 * \brief  Get TMU Data
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[out]  tmu_data    Temperature data returned by CPU
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_tmu_get(adi_apollo_device_t *device, adi_apollo_device_tmu_data_t *tmu_data);

/**
 * \brief  Enable TMU
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_tmu_enable(adi_apollo_device_t *device);

/**
 * \brief  Set the global temperature alarm thresholds
 *
 * \details When any temperature sensor reading crosses a threshold, the firmware asserts the
 *          corresponding FW_IRQ_OUT temperature-alarm source bit
 *          (\ref ADI_APOLLO_FW_IRQ_OUT_SOURCE_TEMP_ALARM_HIGH / \ref ADI_APOLLO_FW_IRQ_OUT_SOURCE_TEMP_ALARM_LOW).
 *          The high threshold must be strictly greater than the low threshold or the firmware
 *          rejects the command with \ref APOLLO_CPU_DEV_TEMP_SENSOR_INVALID_THRESHOLD_ERROR.
 *          Thresholds are expressed in tenths of a degree Celsius (dC), e.g. +110.0 degC = 1100.
 *          Note: setting the thresholds only arms the source; to make the alarm reach the pin the
 *          host must also enable the source (see \ref adi_apollo_arm_fw_irq_out_sources_enable).
 *
 * \param[in]  device               Context variable - Pointer to the APOLLO device data structure
 * \param[in]  high_threshold_dc    High alarm threshold in tenths of degC
 * \param[in]  low_threshold_dc     Low alarm threshold in tenths of degC
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_temp_alarm_threshold_set(adi_apollo_device_t *device, int16_t high_threshold_dc, int16_t low_threshold_dc);

/**
 * \brief  Get the current global temperature alarm thresholds
 *
 * \details Thresholds are expressed in tenths of a degree Celsius (dC).
 *
 * \param[in]  device               Context variable - Pointer to the APOLLO device data structure
 * \param[out] high_threshold_dc    Current high threshold in tenths of degC
 * \param[out] low_threshold_dc     Current low threshold in tenths of degC
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_temp_alarm_threshold_get(adi_apollo_device_t *device, int16_t *high_threshold_dc, int16_t *low_threshold_dc);

/**
 * \brief  apollo shutdown
 *
 * \param[in]  device                       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  wait_time_between_grps_in_us Shutdown wait time between each group. \ref adi_apollo_mailbox_cmd_shutdown_t
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_device_shutdown(adi_apollo_device_t *device, uint32_t wait_time_between_grps_in_us);

#ifndef CLIENT_IGNORE
#endif /* CLIENT_IGNORE*/

#ifdef __cplusplus
}
#endif

#endif /* ADI_APOLLO_DEVICE_H__ */
/*! @} */
