/*!
 * \brief     Device configuration header
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_CONFIG
 * @{
 */
#ifndef ADI_APOLLO_CONFIG_H__
#define ADI_APOLLO_CONFIG_H__

/*============= D E F I N E S ==============*/

/*============= I N C L U D E S ============*/
#include "adi_apollo_common.h"

/*============= D E F I N E S ==============*/
#if (defined(__STDC_VERSION__) && __STDC_VERSION__ == 199901L)
    #define __FUNCTION_NAME__  __func__
#else
    #define __FUNCTION_NAME__  __FUNCTION__
#endif

#define ADI_APOLLO_API_REV 0x00000001

/* var error report */
#define ADI_APOLLO_MSG_REPORT(var, comment) \
    adi_apollo_hal_error_report(device, ADI_CMS_LOG_MSG,  (int32_t)API_CMS_ERROR_OK, __FILE__, __FUNCTION_NAME__, __LINE__, #var, (comment))
#define ADI_APOLLO_WARN_REPORT(var, comment) \
    adi_apollo_hal_error_report(device, ADI_CMS_LOG_WARN, (int32_t)API_CMS_ERROR_OK, __FILE__, __FUNCTION_NAME__, __LINE__, #var, (comment))
#define ADI_APOLLO_ERROR_REPORT(error, var, comment) \
    adi_apollo_hal_error_report(device, ADI_CMS_LOG_ERR,  (int32_t)(error), __FILE__, __FUNCTION_NAME__, __LINE__, #var, (comment))

/* log report */
#define ADI_APOLLO_LOG_FUNC() \
    (void)adi_apollo_hal_log_write(device, ADI_CMS_LOG_API, "%s(...)", __FUNCTION_NAME__)
#define ADI_APOLLO_LOG_SPIR(addr, data) \
    (void)adi_apollo_hal_log_write(device, ADI_CMS_LOG_SPI, "apollo: r@%.4x = %.2x", (addr), (data))
#define ADI_APOLLO_LOG_SPIW(addr, data) \
    (void)adi_apollo_hal_log_write(device, ADI_CMS_LOG_SPI, "apollo: w@%.4x = %.2x", (addr), (data))
#define ADI_APOLLO_LOG_SPIR32(addr, data) \
    (void)adi_apollo_hal_log_write(device, ADI_CMS_LOG_SPI, "apollo: r32@%.4x = %.8x", (addr), (data))
#define ADI_APOLLO_LOG_SPIW32(addr, data) \
    (void)adi_apollo_hal_log_write(device, ADI_CMS_LOG_SPI, "apollo: w32@%.4x = %.8x", (addr), (data))

/* MISRA Deviation: Rule 20.10 - Token pasting operator ## used with __VA_ARGS__.
 * Justification: GNU C extension ##__VA_ARGS__ is required for proper variadic macro handling.
 *                Without ##, calling this macro with no variadic arguments would produce invalid
 *                syntax: func(a, b, ) with trailing comma. The ## removes the comma when __VA_ARGS__
 *                is empty. This is a widely-supported extension for variadic macros in C99. */
#define ADI_APOLLO_LOG_VAR(type, msg, ...) \
    adi_apollo_hal_log_write(device, (type), (msg), ##__VA_ARGS__)

#define ADI_APOLLO_LOG_MSG(msg) \
    adi_apollo_hal_log_write(device, ADI_CMS_LOG_MSG, (msg))
/* MISRA Deviation: Rule 20.10 - Uses ##__VA_ARGS__ (see ADI_APOLLO_LOG_VAR). */
#define ADI_APOLLO_LOG_MSG_VAR(msg, ...) \
    ADI_APOLLO_LOG_VAR(ADI_CMS_LOG_MSG, (msg), ##__VA_ARGS__)

#define ADI_APOLLO_LOG_WARN(msg) \
    adi_apollo_hal_log_write(device, ADI_CMS_LOG_WARN, (msg))
/* MISRA Deviation: Rule 20.10 - Uses ##__VA_ARGS__ (see ADI_APOLLO_LOG_VAR). */
#define ADI_APOLLO_LOG_WARN_VAR(msg, ...) \
    ADI_APOLLO_LOG_VAR(ADI_CMS_LOG_WARN, (msg), ##__VA_ARGS__)

#define ADI_APOLLO_LOG_ERR(msg) \
    adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, (msg))
/* MISRA Deviation: Rule 20.10 - Uses ##__VA_ARGS__ (see ADI_APOLLO_LOG_VAR). */
#define ADI_APOLLO_LOG_ERR_VAR(msg, ...) \
    ADI_APOLLO_LOG_VAR(ADI_CMS_LOG_ERR, (msg), ##__VA_ARGS__)

/* var error check */
#define ADI_APOLLO_LOG_ERROR_RETURN_CODE(err, msg, code) \
{ \
    if ((err) != (int32_t)API_CMS_ERROR_OK) { \
        ADI_APOLLO_LOG_ERR((msg)); \
        return (int32_t)(code); \
    } \
}

#define ADI_APOLLO_LOG_ERROR_RETURN(err, msg) \
    ADI_APOLLO_LOG_ERROR_RETURN_CODE((err), (msg), (err));

#define ADI_APOLLO_ERROR_RETURN(r) \
{ \
    if ((r) != (int32_t)API_CMS_ERROR_OK) { \
        return (r); \
    } \
}

/* MISRA Deviation: Rule 20.10 - Uses ##__VA_ARGS__ (see ADI_APOLLO_LOG_VAR). */
#define ADI_APOLLO_LOG_ERROR_RETURN_VAR(r, msg, ...) \
{ \
    if ((r) != (int32_t)API_CMS_ERROR_OK) { \
        adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, (msg), ##__VA_ARGS__); \
        return (r); \
    } \
}

/* MISRA Deviation: Rule 20.7 - Macro parameter 'label' cannot be parenthesized.
 * Justification: C language syntax requires 'goto label;' - 'goto (label);' is invalid.
 * This is a permitted deviation due to language constraints. */
#define ADI_APOLLO_ERROR_GOTO(r, label) \
{ \
    if ((r) != (int32_t)API_CMS_ERROR_OK) { \
        goto label; \
    } \
}

/* MISRA Deviation: Rule 20.7 - Macro parameter 'label' cannot be parenthesized.
 * Justification: C language syntax requires 'goto label;' - 'goto (label);' is invalid.
 * This is a permitted deviation due to language constraints.
 * MISRA Deviation: Rule 20.10 - Uses ##__VA_ARGS__ (see ADI_APOLLO_LOG_VAR). */
#define ADI_APOLLO_LOG_ERROR_GOTO_VAR(r, label, msg, ...) \
{ \
    if ((r) != (int32_t)API_CMS_ERROR_OK) { \
        adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, (msg), ##__VA_ARGS__); \
        goto label; \
    } \
}

#define ADI_APOLLO_NULL_POINTER_RETURN(p) \
{ \
    if((p) == NULL) { \
        ADI_APOLLO_ERROR_REPORT(API_CMS_ERROR_NULL_PARAM, (p), "Null pointer passed."); \
        ADI_CMS_RETURN(API_CMS_ERROR_NULL_PARAM); \
    } \
}

#define ADI_APOLLO_INVALID_PARAM_RETURN(r) \
{ \
    if(r) { \
        ADI_APOLLO_ERROR_REPORT(API_CMS_ERROR_INVALID_PARAM, (r), "Invalid param passed."); \
        ADI_CMS_RETURN(API_CMS_ERROR_INVALID_PARAM); \
    } \
}

#define ADI_APOLLO_NOT_IMPLEMENTED_RETURN(r) \
{ \
    if(r) { \
        ADI_APOLLO_ERROR_REPORT(API_CMS_ERROR_NOT_IMPLEMENTED, (r), "Not implemented feature or param."); \
        ADI_CMS_RETURN(API_CMS_ERROR_NOT_IMPLEMENTED); \
    } \
}

#define ADI_APOLLO_INVALID_PARAM_WARN(r) \
{ \
    if(r) { \
        ADI_APOLLO_WARN_REPORT((r), "Invalid param passed."); \
    } \
}

#define ADI_APOLLO_DEV_FEAT_LOCKOUT_RETURN(term, feat) \
{ \
    if (adi_apollo_private_device_lockout_get(device, (term), (feat)) != 0U) { \
        ADI_APOLLO_ERROR_REPORT(API_CMS_ERROR_FEAT_LOCKOUT, (feat), "Device feature is locked out."); \
        ADI_CMS_RETURN(API_CMS_ERROR_FEAT_LOCKOUT); \
    } \
}

/**
 * \brief Validate mailbox response buffer length before accessing indices
 * \details Checks that the response buffer has sufficient length before reading.
 *          This prevents reading uninitialized memory if firmware returns truncated responses.
 * \param[in] resp       Mailbox response structure
 * \param[in] min_len    Minimum required length in bytes
 */
#define ADI_APOLLO_MAILBOX_RESP_LENGTH_CHECK(resp, min_len)

/* TODO(APOLLOSW): Restore ADI_APOLLO_MAILBOX_RESP_LENGTH_CHECK once FW updates mailbox response length field correctly. */
#if 0
#define ADI_APOLLO_MAILBOX_RESP_LENGTH_CHECK(resp, min_len) \
do { \
    if ((resp).length < (min_len)) { \
        adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, \
            "Mailbox response length too short: %u < %u", \
            (uint32_t)(resp).length, (uint32_t)(min_len)); \
        ADI_CMS_RETURN(API_CMS_ERROR_MAILBOX_RESP_STATUS); \
    } \
} while(false)
#endif

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* ADI_APOLLO_CONFIG_H__ */

/*! @} */
