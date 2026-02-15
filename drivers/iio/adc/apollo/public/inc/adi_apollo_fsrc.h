/*!
 * \brief     FSRC Block definition headers
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_FSRC
 * @{
 */
#ifndef __ADI_APOLLO_FSRC_H__
#define __ADI_APOLLO_FSRC_H__

/*============= I N C L U D E S ============*/
#include "adi_apollo_fsrc_types.h"

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief  Configure FSRC parameters
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal            Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrcs               Target FSRC  \ref adi_apollo_fsrc_sel_e
 * \param[in]  config              \ref adi_apollo_fsrc_pgm_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_pgm(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrcs, adi_apollo_fsrc_pgm_t *config);

/**
 * \brief  Inspect FSRC parameters
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal            Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrc                Target FSRC  \ref adi_apollo_fsrc_sel_e
 * \param[in]  fsrc_inspect        \ref adi_apollo_fsrc_inspect_t
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_inspect(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrc, adi_apollo_fsrc_inspect_t *fsrc_inspect);


/**
 * \brief  Configure RX Rate Match FIFO parameters
 *
 * \param[in]  device     Context variable - Pointer to the APOLLO device data structure
 * \param[in]  links      Target link select
 * \param[in]  config     \ref adi_apollo_rx_rm_fifo_pgm_t
 *
 * \return API_CMS_ERROR_OK                     API Completed Successfully
 * \return <0                                   Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_rx_rm_fifo_pgm(adi_apollo_device_t *device, const uint16_t links, adi_apollo_rx_rm_fifo_pgm_t *config);


/**
 * \brief Set the FSRC rate parameters
 * 
 * FSRC rate = (rate_int +  rate_frac_a/rate_frac_b)/2^48
 * 
 * Gain Reduction = round(2^12*M/N)
 * 
 * The FSRC ratio m/n must be between 1 and 2, not inclusive (n > m)
 * 
 * Python code example for generating FSRC parameters
 * 
 * from math import gcd as mathgcd
 * from math import floor as mathfloor
 * 
 * def calculate_fsrc_parameters(fsrc_n, fsrc_m):
 *     """
 *     Calculates FSRC Ratio and Gain Settings as per the FSRC User Guide
 *     :param fsrc_n: int - FSRC Ratio Numerator
 *     :param fsrc_m: int - FSRC Ratio Denominator
 *     :return: tuple of int - int, frac_a, frac_b and gain reduction setting
 *     """
 *     p_bits = 48
 *     gain_reduction = pow(2, 12) * fsrc_m / fsrc_n
 *     if fsrc_n == 1 and fsrc_m == 1:
 *         gain_reduction = pow(2, 12) - 1
 *     else:
 *         gain_reduction = mathfloor(gain_reduction)
 *     fsrc_rate_int = int(mathfloor((fsrc_m / fsrc_n) * pow(2, p_bits)))
 *     a = (pow(2, p_bits) * fsrc_m) % fsrc_n
 *     b = fsrc_n
 *     gcd_a_b = mathgcd(a, b)
 *     fsrc_rate_frac_a = a / gcd_a_b
 *     fsrc_rate_frac_b = b / gcd_a_b
 *     return fsrc_rate_int, int(fsrc_rate_frac_a), int(fsrc_rate_frac_b), gain_reduction
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal            Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrcs               Target FSRC  \ref adi_apollo_fsrc_sel_e
 * \param[in]  rate_int            rate_int:    FSRC rate = (FSRC rate_int +  rate_frac_a/rate_frac_b)/2^48
 * \param[in]  rate_frac_a         rate_frac_a  FSRC rate = (FSRC rate_int +  rate_frac_a/rate_frac_b)/2^48
 * \param[in]  rate_frac_b         rate_frac_b  FSRC rate = (FSRC rate_int +  rate_frac_a/rate_frac_b)/2^48
 * \param[in]  gain_red            gain_red     Gain Reduction = round(2^12*M/N)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_rate_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrcs, 
    uint64_t rate_int, uint64_t rate_frac_a, uint64_t rate_frac_b, uint16_t gain_red);

/**
 * \brief Configures an FSRC block given n/m parameters.
 * 
 * FSRC ratio is n/m. The ratio must between 1 and 2, inclusive.
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal            Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrcs               Target FSRC  \ref adi_apollo_fsrc_sel_e
 * \param[in]  n                   Numerator (n > m)
 * \param[in]  m                   Denominator (m < n)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_ratio_set(adi_apollo_device_t* device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrcs, uint32_t n, uint32_t m);


/**
 * \brief Enable or disable FSRC 1x mode
 *
 * \param[in]  device              Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal            Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrcs               Target FSRC  \ref adi_apollo_fsrc_sel_e
 * \param[in]  enable              1 to enable 1x mode, 0 disable
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_mode_1x_enable_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrcs, uint8_t enable);

/**
 * \brief Get FSRC overflow status
 *
 * Reads the FSRC overflow status register. Status bits indicate overflow
 * conditions in the FSRC filter stages.
 *
 * Status bit definitions (per FSRC block):
 * - Bit 0: CH0 I-path overflow
 * - Bit 1: CH0 Q-path overflow
 * - Bit 2: CH1 I-path overflow
 * - Bit 3: CH1 Q-path overflow
 * - Bit 4: Any overflow (OR of bits 0-3)
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal  Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrc      Target FSRC block (single block only)
 * \param[out] status    Pointer to store overflow status (5 bits)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_overflow_status_get(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrc, uint8_t *status);

/**
 * \brief Clear FSRC overflow status
 *
 * Clears the FSRC overflow status register using W1C (Write-1-to-Clear).
 *
 * IMPORTANT: TX FSRC uses gated AHB clock. This requires a Write-1 followed
 * by Write-0 sequence on the status register to properly clear the status.
 * This function handles the sequence automatically.
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal  Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrcs     Target FSRC blocks
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_overflow_status_clear(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrcs);

/**
 * \brief Enable/disable FSRC overflow IRQs
 *
 * Configures the FSRC overflow interrupt enable register. When enabled,
 * overflow conditions will generate IRQ signals.
 *
 * Enable bit definitions (per FSRC block):
 * - Bit 0: Enable CH0 I-path overflow IRQ
 * - Bit 1: Enable CH0 Q-path overflow IRQ
 * - Bit 2: Enable CH1 I-path overflow IRQ
 * - Bit 3: Enable CH1 Q-path overflow IRQ
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal  Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrcs     Target FSRC blocks
 * \param[in]  enable    IRQ enable mask (4 bits, or 0xF for all)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_irq_enable_set(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrcs, uint8_t enable);

/**
 * \brief Get FSRC IRQ enable status
 *
 * Reads the FSRC overflow interrupt enable register.
 *
 * \param[in]  device    Context variable - Pointer to the APOLLO device data structure
 * \param[in]  terminal  Target terminal \ref adi_apollo_terminal_e
 * \param[in]  fsrc      Target FSRC block (single block only)
 * \param[out] enable    Pointer to store IRQ enable mask (4 bits)
 *
 * \return API_CMS_ERROR_OK    API Completed Successfully
 * \return <0                  Failed. \ref adi_cms_error_e for details.
 */
int32_t adi_apollo_fsrc_irq_enable_get(adi_apollo_device_t *device, adi_apollo_terminal_e terminal, adi_apollo_blk_sel_t fsrc, uint8_t *enable);

#ifndef CLIENT_IGNORE

#endif /* CLIENT_IGNORE*/

#ifdef __cplusplus
}
#endif

#endif /* __ADI_APOLLO_FSRC_H__ */
/*! @} */
