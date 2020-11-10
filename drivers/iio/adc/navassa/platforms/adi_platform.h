/**
* \file
* \brief Contains ADI Transceiver Hardware Abstraction functions interface
*        Analog Devices maintains and provides updates to this code layer.
*        The end user should not modify this file or any code in this directory.
*/

/**
* \Page Disclaimer Legal Disclaimer
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the API license, for more information.
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef __ADI_PLATFORM_H__
#define __ADI_PLATFORM_H__

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdio.h>
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE

/**
 * BBIC Logging functions
 */
extern int32_t(*adi_hal_LogWrite)(void *devHalCfg, uint32_t logLevel, const char *comment, va_list argp);

/**
 * BBIC Timer functions
 */
extern int32_t(*adi_hal_Wait_us)(void *devHalCfg, uint32_t time_us);

#endif /* CLIENT_IGNORE */
    
#ifdef __cplusplus
}
#endif
#endif /* __ADI_PLATFORM_H__ */


