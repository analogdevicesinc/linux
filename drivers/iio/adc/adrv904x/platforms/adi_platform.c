/**
* Copyright 2015 - 2025 Analog Devices Inc.
* SPDX-License-Identifier: Apache-2.0
*/

/**
* \file adi_platform.c
*
* \brief Definitions for ADI Specific Platforms
*
* ADRV904X API Version: 2.15.0.5
*/

#include "adi_platform.h"

#ifdef _ADI_ADS10_PLATFORM
#include "ads10/ads10_init.h"
#include "ads10/ads10_i2c.h"
#include "ads10/ads10_spi.h"
#include "ads10/ads10_timer.h"
#include "ads10/ads10_bbic_control.h"
#include "posix/posix_mutex.h"
#endif

#include "../platforms/common/tls.h"
#include "../platforms/common/adi_logging.h"

/*
 * Function pointer assignment for default configuration
 */

/* Initialization interface to open, init, close drivers and pointers to resources */
adi_hal_Err_e (*adrv904x_HwOpen)(void* const devHalCfg)                          = NULL;

adi_hal_Err_e (*adrv904x_HwClose)(void* const devHalCfg)                         = NULL;

adi_hal_Err_e (*adrv904x_HwReset)(void* const devHalCfg, const uint8_t pinLevel) = NULL;

void*         (*adrv904x_DevHalCfgCreate)(   const uint32_t      interfaceMask,
                                            const uint8_t       spiChipSelect,
                                            const char* const   logFilename)    = NULL;

adi_hal_Err_e (*adrv904x_DevHalCfgFree)(void* devHalCfg)                         = NULL;

/* SPI Interface */
adi_hal_Err_e (*adrv904x_SpiWrite)(  void* const     devHalCfg,
                                    const uint8_t   txData[],
                                    const uint32_t  numTxBytes) = NULL;

adi_hal_Err_e (*adrv904x_SpiRead)(   void* const     devHalCfg,
                                    const uint8_t   txData[],
                                    uint8_t         rxData[],
                                    const uint32_t  numRxBytes) = NULL;

/* I2C Interface */
adi_hal_Err_e(*adi_hal_I2cWrite)(   void* const     devHalCfg,
                                    const uint8_t   txData[],
                                    const uint32_t  numTxBytes) = NULL;

adi_hal_Err_e(*adi_hal_I2cRead)(    void* const     devHalCfg,
                                    const uint8_t   txData[],
                                    const uint32_t  numTxBytes,
                                    uint8_t         rxData[],
                                    const uint32_t  numRxBytes) = NULL;

/* Logging interface */
adi_hal_Err_e (*adrv904x_LogFileOpen)(   void* const         devHalCfg,
                                        const char* const   filename)   = NULL;

adi_hal_Err_e (*adrv904x_LogLevelSet)(   void* const     devHalCfg,
                                        const uint32_t  logLevelMask)   = NULL;

adi_hal_Err_e (*adrv904x_LogLevelGet)(   void* const     devHalCfg,
                                        uint32_t* const logLevelMask)   = NULL;

adi_hal_Err_e (*adrv904x_LogStatusGet)(  void* const                     devHalCfg,
                                        adi_hal_LogStatusGet_t* const  logStatus)   = NULL;

adi_hal_Err_e (*adrv904x_LogConsoleSet)( void* const devHalCfg,
                                        const adi_hal_LogConsole_e  logConsoleFlag) = NULL;

adi_hal_Err_e (*adrv904x_LogWrite)(  void* const                 devHalCfg,
                                    const adi_hal_LogLevel_e    logLevel,
                                    const uint8_t               indent,
                                    const char* const           comment,
                                    va_list                     argp)   = NULL;

adi_hal_Err_e (*adrv904x_LogFileClose)(void* const devHalCfg) = NULL;

/* Timer interface */
adi_hal_Err_e (*adrv904x_Wait_ms)(void* const devHalCfg, const uint32_t time_ms) = NULL;
adi_hal_Err_e (*adrv904x_Wait_us)(void* const devHalCfg, const uint32_t time_us) = NULL;

/* BBIC control interface */
adi_hal_Err_e (*adi_hal_BbicRegisterRead)(  void* const     devHalCfg,
                                            const uint32_t  addr,
                                            uint32_t* const data)           = NULL;

adi_hal_Err_e (*adi_hal_BbicRegisterWrite)( void* const     devHalCfg,
                                            const uint32_t  addr,
                                            const uint32_t  data)           = NULL;

adi_hal_Err_e (*adi_hal_BbicRegistersRead)( void* const     devHalCfg,
                                            const uint32_t  addr,
                                            uint32_t        data[],
                                            const uint32_t  numDataWords)   = NULL;

adi_hal_Err_e (*adi_hal_BbicRegistersWrite)(void* const     devHalCfg,
                                            const uint32_t  addr,
                                            const uint32_t  data[],
                                            const uint32_t  numDataWords)   = NULL;

/* Thread Interface */
adi_hal_thread_t (*adi_hal_ThreadSelf)(void) = NULL;

adi_hal_Err_e (*adrv904x_TlsSet)(const adi_hal_TlsType_e tlsType, void* const value) = NULL;

void* (*adrv904x_TlsGet)(const adi_hal_TlsType_e tlsType) = NULL;

/* Mutex Interface */
adi_hal_Err_e(*adrv904x_MutexInit)(adi_hal_mutex_t* const mutex) = NULL;
adi_hal_Err_e(*adrv904x_MutexLock)(adi_hal_mutex_t* const mutex) = NULL;
adi_hal_Err_e(*adrv904x_MutexUnlock)(adi_hal_mutex_t* const mutex) = NULL;
adi_hal_Err_e(*adrv904x_MutexDestroy)(adi_hal_mutex_t* const mutex) = NULL;

adi_hal_Err_e(*adi_hal_BoardIdentify)(char** boardNames, int32_t* numBoards) = NULL;

ADI_API adi_hal_Err_e adrv904x_hal_PlatformSetup(const adi_hal_Platforms_e platform)
{
    adi_hal_Err_e error = ADI_HAL_ERR_PARAM;

    switch (platform)
    {
    case ADI_ADS10_PLATFORM:
#ifdef _ADI_ADS10_PLATFORM
        adrv904x_HwOpen = ads10_HwOpen;
        adrv904x_HwClose = ads10_HwClose;
        adrv904x_HwReset = ads10_HwReset;
        adrv904x_DevHalCfgCreate = ads10_DevHalCfgCreate;
        adrv904x_DevHalCfgFree = ads10_DevHalCfgFree;

#ifdef ADI_ADRV904X_SPI_DEV_DRIVER_EN
        adrv904x_SpiWrite = ads10_SpiWrite;
        adrv904x_SpiRead = ads10_SpiRead;
#else
        adrv904x_SpiWrite = ads10_SpiWrite_v2;
        adrv904x_SpiRead = ads10_SpiRead_v2;
#endif
        adi_hal_I2cWrite = NULL; /* ADS10 does not require I2C interface to any devices used in device API layer */
        adi_hal_I2cRead = NULL;  /* ADS10 does not require I2C interface to any devices used in device API layer */

        adrv904x_LogFileOpen     = adi_LogFileOpen;
        adrv904x_LogLevelSet     = adi_LogLevelSet;
        adrv904x_LogLevelGet     = adi_LogLevelGet;
        adrv904x_LogStatusGet    = adi_LogStatusGet;
        adrv904x_LogConsoleSet   = adi_LogConsoleSet;
        adrv904x_LogWrite        = adi_LogWrite;
        adrv904x_LogFileClose    = adi_LogFileClose;

        adrv904x_Wait_us = ads10_TimerWait_us;
        adrv904x_Wait_ms = ads10_TimerWait_ms;

        /* only required to support the ADI FPGA*/
        adi_hal_BbicRegisterRead   = ads10_BbicRegisterRead;
        adi_hal_BbicRegisterWrite  = ads10_BbicRegisterWrite;
        adi_hal_BbicRegistersRead  = ads10_BbicRegistersRead;
        adi_hal_BbicRegistersWrite = ads10_BbicRegistersWrite;

        adi_hal_ThreadSelf = posix_ThreadSelf;
        adrv904x_TlsGet = common_TlsGet;
        adrv904x_TlsSet = common_TlsSet;
        adrv904x_MutexInit = posix_MutexInit;
        adrv904x_MutexLock = posix_MutexLock;
        adrv904x_MutexUnlock = posix_MutexUnlock;
        adrv904x_MutexDestroy = posix_MutexDestroy;
        adi_hal_BoardIdentify = ads10_BoardIdentify;
        error = common_TlsInit();
#else
        error = ADI_HAL_ERR_NOT_IMPLEMENTED;
#endif
	case ADI_LINUX:
        adrv904x_HwOpen = linux_adrv904x_HwOpen;
        adrv904x_HwClose = linux_adrv904x_HwClose;
        adrv904x_HwReset = linux_adrv904x_HwReset;

        adrv904x_SpiWrite = linux_adrv904x_SpiWrite;
        adrv904x_SpiRead = linux_adrv904x_SpiRead;

        adrv904x_LogFileOpen = linux_adrv904x_LogFileOpen;
        adrv904x_LogLevelSet = linux_adrv904x_LogLevelSet;
        adrv904x_LogLevelGet = linux_adrv904x_LogLevelGet;
        adrv904x_LogWrite  = linux_adrv904x_LogWrite;
        adrv904x_LogFileClose  = linux_adrv904x_LogFileClose;

        adrv904x_Wait_us = linux_adrv904x_TimerWait_us;
        adrv904x_Wait_ms = linux_adrv904x_TimerWait_ms;

        adrv904x_TlsGet = linux_adrv904x_TlsGet;
        adrv904x_TlsSet = linux_adrv904x_TlsSet;

        adrv904x_MutexInit = linux_adrv904x_MutexInit;
        adrv904x_MutexLock = linux_adrv904x_MutexLock;
        adrv904x_MutexUnlock = linux_adrv904x_MutexUnlock;
        adrv904x_MutexDestroy = linux_adrv904x_MutexDestroy;

        error = ADI_HAL_ERR_OK;
        break;
                default:
            error = ADI_HAL_ERR_PARAM;
            break;
    }

    return error;
}
