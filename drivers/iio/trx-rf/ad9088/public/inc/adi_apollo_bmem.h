/*!
 * \brief     BMEM Block definition headers
 *
 * \copyright copyright(c) 2024 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_BMEM
 * @{
 */
#ifndef ADI_APOLLO_BMEM_H__
#define ADI_APOLLO_BMEM_H__

/*============= I N C L U D E S ============*/
#include "adi_apollo_bmem_types.h"
#include "adi_apollo_trigts_types.h"

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief  Configures HSDIN block BMEM to AWG (Arbitrary Waveform Generator) mode
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       \ref adi_apollo_bmem_awg_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_awg_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_awg_t *config);

/**
 * \brief  Starts HSDIN block BMEM AWG playback in continuous loop
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_awg_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief  Stops HSDIN block BMEM AWG playback
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_awg_stop(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief Writes data to HSDIN block Rx BMEM SRAM for AWG playback
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  data         Array of data to write. Each 32-bit word contains two 16-bit samples
 * \param[in]  length       Number of 32-bit words to write. 32K max (64K 16-bit samples).
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_awg_sram_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint32_t data[], uint32_t length);

/**
 * \brief  Writes 16-bit sample data to HSDIN block BMEM for AWG playback
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  data16       Array of 16-bit data samples to play out of bmem awg.
 * \param[in]  data16_len   Length of data array
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_awg_sample_write(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, int16_t data16[], uint32_t data16_len);

/**
 * \brief  Writes 16-bit sample data to HSDIN block BMEM for AWG playback on 8T8R devices
 *
 * \note
 * For 8T8R devices, BMEM instances split the sram memory between ADC pairs (e.g. A0/A2).
 * This results in half the number of samples per converter channel.
 *
 * This function requires a user provided scratch memory. Prevents exceeding local stack limits.
 * Apollo APIs don't use dynamic memory allocation.
 *
 * \param[in]  device           Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems            Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  data16_0         Array of 16-bit data samples to play out of bmem awg for first ADC in pair
 * \param[in]  data16_1         Array of 16-bit data samples to play out of bmem awg for second ADC in pair
 * \param[in]  data16_len       Length of data16_0 and data16_1 arrays
 * \param[in]  scratch32        User provided 32-bit scratch array. Used by function to assemble data and write mem.
 * \param[in]  scratch32_len    Length of scratch32 array
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_awg_sample_shared_write(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems,
                                             int16_t data16_0[], int16_t data16_1[], uint32_t data16_len,
                                             uint32_t scratch32[], uint32_t scratch32_len);

/**
 * \brief  Configures HSDIN block BMEM to Capture mode for ADC data acquisition
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       \ref adi_apollo_bmem_capture_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_capture_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_capture_t *config);

/**
 * \brief  Runs HSDIN block BMEM capture to acquire ADC samples
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_capture_run(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief  Configures BMEM for triggered capture mode
 *
 * Sets up BMEMs to wait for a trigger before starting capture. All configured
 * BMEMs will start simultaneously on the same trigger event, ensuring alignment.
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEMs \ref adi_apollo_bmem_sel_e (can be multiple, use OR)
 * \param[in]  config       Capture configuration \ref adi_apollo_bmem_capture_t
 * \param[in]  trig_source  Trigger source selection \ref adi_apollo_trigger_sel_mux_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 *
 * \note This is a one-time configuration. After calling this, use
 *       adi_apollo_bmem_hsdin_capture_arm() before each capture iteration.
 *
 * \note Trigger source options:
 *       - ADI_APOLLO_TRIG_SPI: Software trigger via SPI (debug only - cannot synchronize across both sides)
 *       - ADI_APOLLO_TRIG_INPUT: External hardware trigger pin (required for cross-side synchronization)
 *       - ADI_APOLLO_TRIG_MASTER: Internal trigger master generator
 */
int32_t adi_apollo_bmem_hsdin_capture_config_triggered(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems,
                                                        adi_apollo_bmem_capture_t *config, adi_apollo_trigger_sel_mux_e trig_source);

/**
 * \brief  Arms BMEMs for triggered capture
 *
 * Resets BMEMs and sets fast mode for converter access. Does NOT enable
 * trigger mode - call adi_apollo_bmem_hsdin_capture_trig_mode_enable()
 * separately before issuing the sync pulse.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems         Target BMEMs \ref adi_apollo_bmem_sel_e (must match config call)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref cms_error_e for details.
 *
 * \note BMEMs must first be configured with adi_apollo_bmem_hsdin_capture_config_triggered()
 */
int32_t adi_apollo_bmem_hsdin_capture_arm(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief  Enables trigger mode on BMEMs for sync-triggered capture
 *
 * Sets trig_mode=1 on selected BMEMs so they will start capture on the
 * next sync pulse. Call this after adi_apollo_bmem_hsdin_capture_arm()
 * and before issuing the sync pulse via adi_apollo_bmem_hsdin_capture_trigger_sync().
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems         Target BMEMs \ref adi_apollo_bmem_sel_e (must match config call)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 *
 * \note This is separated from capture_arm() to give users explicit control over
 *       when trigger mode is enabled, avoiding unintended captures from sync pulses
 *       during other operations (e.g., oneshot-sync, dyn_cfg_sync, clock conditioning).
 */
int32_t adi_apollo_bmem_hsdin_capture_trig_mode_enable(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief  Triggers SPI-configured BMEMs to start capture (DEBUG ONLY)
 *
 * Issues an SPI trigger to start capture on BMEMs configured with
 * ADI_APOLLO_TRIG_SPI trigger source. All armed BMEMs will start
 * simultaneously on the same trigger edge.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 *
 * \note BMEMs must be configured with ADI_APOLLO_TRIG_SPI, armed, and have trigger mode enabled before calling
 * \note This function has no effect if BMEMs are configured for external trigger sources
 * \note DEBUG ONLY: This function cannot achieve synchronization across Side A and Side B.
 *       For cross-side synchronization, use ADI_APOLLO_TRIG_INPUT mode with
 *       adi_apollo_bmem_hsdin_capture_trigger_sync() instead.
 */
int32_t adi_apollo_bmem_hsdin_capture_trigger_spi(adi_apollo_device_t *device);

/**
 * \brief  Triggers synchronized cross-side BMEM capture using MCS manual sync
 *
 * Issues a hardware-synchronized trigger via the MCS manual sync mechanism. This
 * provides 0-sample skew between Side A and Side B BMEMs for cross-side synchronized
 * capture. Use with ADI_APOLLO_TRIG_INPUT trigger source.
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 *
 * \note BMEMs must be configured with ADI_APOLLO_TRIG_INPUT, armed, and have trigger mode enabled before calling
 * \note This method uses the MCS hardware distribution network for true cross-side sync
 * \note Typical workflow: config_triggered(0x33, cfg, TRIG_INPUT) -> arm(0x33) -> trigger_sync()
 */
int32_t adi_apollo_bmem_hsdin_capture_trigger_sync(adi_apollo_device_t *device);

/**
 * \brief  Waits for all specified BMEMs to complete triggered capture
 *
 * Polls the FULL_IRQ status for all selected BMEMs until all are complete.
 * After this returns successfully, data can be read using capture_get().
 *
 * \param[in]  device        Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems         Target BMEMs \ref adi_apollo_bmem_sel_e (must match config call)
 * \param[in]  timeout_us    Maximum time to wait in microseconds
 *
 * \return API_CMS_ERROR_OK      API Completed Successfully
 * \return API_CMS_ERROR_OPERATION_TIMEOUT Timeout waiting for BMEM(s) to complete
 * \return <0                    Failed. \ref adi_cms_error_e for details.
 *
 * \note This function switches BMEMs to slow mode for SPI readout after completion
 */
int32_t adi_apollo_bmem_hsdin_capture_wait_complete(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint32_t timeout_us);

/**
 * \brief Reads captured ADC data from HSDIN block Rx BMEM SRAM
 *
 * \param[in]   device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]   bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[out]  data         Data that is read out of each \ref adi_apollo_bmem_sel_e. Each 32-bit word contains two 16-bit samples
 * \param[in]   length       Number of 32-bit words to read. 32K max (64K 16-bit samples).
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_capture_get(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint32_t data[], uint32_t length);

/**
 * \brief  Sets HSDIN block BMEM sample delay value
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  sample_delay        Sample delay
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_delay_sample_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay);

/**
 * \brief  Sets HSDIN block BMEM delay values for the 4 hopping profiles
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  sample_delay        Sample delay array - 4 elements
 * \param[in]  sample_delay_length Sample delay array length (must be 4)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_delay_hop_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay[], uint32_t sample_delay_length);

/**
 * \brief  Configures HSDIN block BMEM to sample delay mode
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       BMEM delay config \ref adi_apollo_bmem_delay_sample_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_delay_sample_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_sample_t *config);

/**
 * \brief  Configures HSDIN block BMEM to delay mode with frequency hopping support
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       BMEM delay config \ref adi_apollo_bmem_delay_hop_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_delay_hop_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_hop_t *config);

/**
 * \brief  Starts HSDIN block BMEM sample delay operation
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_hsdin_delay_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief  Sets CDDC (Coarse DDC) block BMEM sample delay value
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  sample_delay        Sample delay
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_cddc_delay_sample_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay);

/**
 * \brief  Sets CDDC (Coarse DDC) block BMEM delay values for the 4 hopping profiles
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  sample_delay        Sample delay array - 4 elements
 * \param[in]  sample_delay_length Sample delay array length (must be 4)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_cddc_delay_hop_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay[], uint32_t sample_delay_length);

/**
 * \brief  Configures CDDC (Coarse DDC) block BMEM to sample delay mode
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       BMEM delay config \ref adi_apollo_bmem_delay_sample_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_cddc_delay_sample_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_sample_t *config);

/**
 * \brief  Configures CDDC (Coarse DDC) block BMEM to delay mode with frequency hopping support
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       BMEM delay config \ref adi_apollo_bmem_delay_hop_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_cddc_delay_hop_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_hop_t *config);

/**
 * \brief  Starts CDDC (Coarse DDC) block BMEM sample delay operation
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_cddc_delay_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);

/**
 * \brief  Sets FDDC (Fine DDC) block BMEM sample delay value
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  sample_delay        Sample delay
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_fddc_delay_sample_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay);

/**
 * \brief  Sets FDDC (Fine DDC) block BMEM delay values for the 4 hopping profiles
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  sample_delay        Sample delay array - 4 elements
 * \param[in]  sample_delay_length Sample delay array length (must be 4)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_fddc_delay_hop_set(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, uint16_t sample_delay[], uint32_t sample_delay_length);

/**
 * \brief  Configures FDDC (Fine DDC) block BMEM to sample delay mode
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       BMEM delay config \ref adi_apollo_bmem_delay_sample_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_fddc_delay_sample_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_sample_t *config);

/**
 * \brief  Configures FDDC (Fine DDC) block BMEM to delay mode with frequency hopping support
 *
 * \param[in]  device       Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems        Target BMEM \ref adi_apollo_bmem_sel_e
 * \param[in]  config       BMEM delay config \ref adi_apollo_bmem_delay_hop_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_fddc_delay_hop_config(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems, adi_apollo_bmem_delay_hop_t *config);

/**
 * \brief  Starts FDDC (Fine DDC) block BMEM sample delay operation
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  bmems               Target BMEM \ref adi_apollo_bmem_sel_e
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_bmem_fddc_delay_start(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems);


#ifdef __cplusplus
}
#endif

#endif /* ADI_APOLLO_BMEM_H__ */
/*! @} */
