// SPDX-License-Identifier: GPL-2.0
/*!
 * @brief     device configuration header
 * 
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*! 
 * @addtogroup __ADI_AD9081_CONFIG_H__
 * @{
 */
#ifndef __ADI_AD9081_CONFIG_H__
#define __ADI_AD9081_CONFIG_H__

/*============= D E F I N E S ==============*/

/*============= I N C L U D E S ============*/
#include "adi_ad9081.h"
#include "adi_ad9081_bf_impala_tc.h"
#include "adi_ad9081_bf_jrxa_des.h"
#include "adi_ad9081_bf_jtx_dual_link.h"
#include "adi_ad9081_bf_jtx_qbf_ad9081.h"
#include "adi_ad9081_bf_lcpll_28nm.h"
#include "adi_ad9081_bf_main.h"
#include "adi_ad9081_bf_nb_coarse_nco.h"
#include "adi_ad9081_bf_nb_ddc_dformat.h"
#include "adi_ad9081_bf_nb_fine_nco.h"
#include "adi_ad9081_bf_rx_paging.h"
#include "adi_ad9081_bf_ser_phy.h"
#include "adi_ad9081_bf_spi_only_up.h"
#include "adi_ad9081_bf_ad9081.h"

/*============= D E F I N E S ==============*/
#if (defined(__STDC_VERSION__) && __STDC_VERSION__ == 199901L)
#define __FUNCTION_NAME__ __func__
#else
#define __FUNCTION_NAME__ __FUNCTION__
#endif

/* error report */
#if ADI_REPORT_VERBOSE > 0
#define AD9081_ERROR_REPORT(error, var, comment)                               \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_ERR, error, __FILE__,  \
				    __FUNCTION_NAME__, __LINE__, #var,         \
				    comment)
#define AD9081_WARN_REPORT(var, comment)                                       \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_WARN,                  \
				    API_CMS_ERROR_OK, __FILE__,                \
				    __FUNCTION_NAME__, __LINE__, #var,         \
				    comment)
#define AD9081_MSG_REPORT(var, comment)                                        \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_MSG, API_CMS_ERROR_OK, \
				    __FILE__, __FUNCTION_NAME__, __LINE__,     \
				    #var, comment)
#else
#define AD9081_ERROR_REPORT(error, var, comment)                               \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_ERR, error, "", "",    \
				    __LINE__, "", "")
#define AD9081_WARN_REPORT(var, comment)                                       \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_WARN,                  \
				    API_CMS_ERROR_OK, "", "", __LINE__, "",    \
				    "")
#define AD9081_MSG_REPORT(var, comment)                                        \
	adi_ad9081_hal_error_report(device, ADI_CMS_LOG_MSG, API_CMS_ERROR_OK, \
				    "", "", __LINE__, "", "")
#endif

/* error check */
#define AD9081_ERROR_RETURN(r)                                                 \
	{                                                                      \
		if (r != API_CMS_ERROR_OK) {                                   \
			return r;                                              \
		}                                                              \
	}
#define AD9081_NULL_POINTER_RETURN(p)                                          \
	{                                                                      \
		if (p == NULL) {                                               \
			AD9081_ERROR_REPORT(API_CMS_ERROR_NULL_PARAM, p,       \
					    "Null pointer passed.");           \
			return API_CMS_ERROR_NULL_PARAM;                       \
		}                                                              \
	}
#define AD9081_INVALID_PARAM_RETURN(r)                                         \
	{                                                                      \
		if (r) {                                                       \
			AD9081_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, r,    \
					    "Invalid param passed.");          \
			return API_CMS_ERROR_INVALID_PARAM;                    \
		}                                                              \
	}
#define AD9081_INVALID_PARAM_WARN(r)                                           \
	{                                                                      \
		if (r) {                                                       \
			AD9081_WARN_REPORT(r, "Invalid param passed.");        \
		}                                                              \
	}

/* log report */
#if ADI_REPORT_VERBOSE > 0
#define AD9081_LOG_FUNC()                                                      \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_MSG, "%s(...)",           \
				 __FUNCTION_NAME__)
#define AD9081_LOG_VAR(type, msg, ...)                                         \
	adi_ad9081_hal_log_write(device, type, msg, ##__VA_ARGS__)
#define AD9081_LOG_MSG(msg)                                                    \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_MSG, msg)
#define AD9081_LOG_WARN(msg)                                                   \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_WARN, msg)
#define AD9081_LOG_ERR(msg)                                                    \
	adi_ad9081_hal_log_write(device, ADI_CMS_LOG_ERR, msg)
#else
#define AD9081_LOG_FUNC(type)
#define AD9081_LOG_VAR(type, msg, ...)
#define AD9081_LOG_MSG(msg)
#define AD9081_LOG_WARN(msg)
#define AD9081_LOG_ERR(msg)
#endif

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

int32_t adi_ad9081_device_clk_pll_enable_set(adi_ad9081_device_t *device,
					     uint8_t pll_en);
int32_t adi_ad9081_device_clk_pll_div_set(adi_ad9081_device_t *device,
					  uint8_t ref_div, uint8_t m_div,
					  uint8_t pll_div, uint8_t fb_div);
int32_t adi_ad9081_device_clk_pll_startup(adi_ad9081_device_t *device,
					  uint64_t dac_clk_hz,
					  uint64_t ref_clk_hz);
int32_t adi_ad9081_device_clk_up_div_set(adi_ad9081_device_t *device,
					 uint64_t dac_clk_hz);
int32_t adi_ad9081_device_laminate_id_get(adi_ad9081_device_t *device,
					  uint8_t *id);
int32_t adi_ad9081_device_die_id_get(adi_ad9081_device_t *device, uint8_t *id);
int32_t adi_ad9081_device_power_status_check(adi_ad9081_device_t *device);
int32_t adi_ad9081_device_reg8_access_check(adi_ad9081_device_t *device);
int32_t adi_ad9081_device_reg32_access_check(adi_ad9081_device_t *device);

int32_t adi_ad9081_dac_d2a_dual_spi_enable_set(adi_ad9081_device_t *device,
					       uint8_t duals, uint8_t enable);
int32_t adi_ad9081_dac_dll_startup(adi_ad9081_device_t *device, uint8_t dacs);
int32_t adi_ad9081_dac_digital_logic_enable_set(adi_ad9081_device_t *device,
						uint8_t enable);
int32_t adi_ad9081_dac_duc_nco_ftw0_set(adi_ad9081_device_t *device,
					uint8_t dacs, uint8_t channels,
					uint64_t ftw);
int32_t adi_ad9081_dac_soft_off_gain_enable_set(adi_ad9081_device_t *device,
						uint8_t dacs, uint8_t enable);
int32_t adi_ad9081_dac_shuffle_enable_set(adi_ad9081_device_t *device,
					  uint8_t dacs, uint8_t enable);
int32_t adi_ad9081_dac_data_xor_set(adi_ad9081_device_t *device, uint8_t dacs,
				    uint8_t enable);

int32_t adi_ad9081_adc_core_setup(adi_ad9081_device_t *device,
				  uint8_t adc_cores);

int32_t adi_ad9081_jesd_tx_conv_mask_set(adi_ad9081_device_t *device,
					 adi_ad9081_jesd_link_select_e links,
					 uint8_t conv_index, uint8_t val);
int32_t adi_ad9081_jesd_tx_pll_status_get(adi_ad9081_device_t *device,
					  uint8_t *pll_locked);
int32_t adi_ad9081_jesd_pll_lock_status_get(adi_ad9081_device_t *device,
					    uint8_t *locked);

#ifdef __cplusplus
}
#endif

#endif /* __ADI_AD9081_CONFIG_H__ */

/*! @} */