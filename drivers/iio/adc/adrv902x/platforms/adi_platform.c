// SPDX-License-Identifier: GPL-2.0
/**
* Copyright 2015 - 2019 Analog Devices Inc.
* Released under the ADRV9025 API license, for more information.
* see the "LICENSE.pdf" file in this zip file.
*/

#include "adi_platform.h"

#include "ads9/ads9_init.h"
#include "ads9/ads9_spi.h"
#include "ads9/ads9_logging.h"
#include "ads9/ads9_timer.h"
#include "ads9/ads9_bbic_control.h"

#include "ads8/ads8_init.h"
#include "ads8/ads8_spi.h"
#include "ads8/ads8_logging.h"
#include "ads8/ads8_timer.h"
#include "ads8/ads8_bbic_control.h"

#include "adi_platform_identify.h"

/*
 * Function pointer assignemt for default configuration
 */

/* Initialization interface to open, init, close drivers and pointers to resources */
int32_t(*adi_hal_HwOpen)(void *devHalCfg) = NULL;
int32_t(*adi_hal_HwClose)(void *devHalCfg) = NULL;
int32_t(*adi_hal_HwReset)(void *devHalCfg, uint8_t pinLevel) = NULL;
int32_t(*adi_hal_SpiInit)(void *devHalCfg) = NULL; /* TODO: remove?  called by HwOpen() */
void* (*adi_hal_DevHalCfgCreate)(uint32_t interfaceMask, uint8_t spiChipSelect, const char *logFilename) = NULL;
int32_t(*adi_hal_DevHalCfgFree)(void *devHalCfg) = NULL;
int32_t(*adi_hal_HwVerify)(void *devHalCfg) = NULL;

/* SPI Interface */
int32_t (*adrv9025_hal_SpiWrite)(void*         devHalCfg,
                            const uint8_t txData[],
                            uint32_t      numTxBytes) = NULL;

int32_t (*adrv9025_hal_SpiRead)(void*         devHalCfg,
                           const uint8_t txData[],
                           uint8_t       rxData[],
                           uint32_t      numRxBytes) = NULL;

/* Custom SPI streaming interface*/
int32_t (*adi_hal_CustomSpiStreamWrite)(void*          devHalCfg,
                                        const uint16_t address,
                                        const uint8_t  txData[],
                                        uint32_t       numTxBytes,
                                        uint8_t        numBytesofAddress,
                                        uint8_t        numBytesOfDataPerStream) = NULL;

int32_t (*adi_hal_CustomSpiStreamRead)(void*          devHalCfg,
                                       const uint16_t address,
                                       uint8_t        rxData[],
                                       uint32_t       numRxBytes,
                                       uint8_t        numBytesofAddress,
                                       uint8_t        numBytesOfDataPerStream) = NULL;

/* Logging interface */
int32_t (*adi_hal_LogFileOpen)(void*       devHalCfg,
                               const char* filename) = NULL;

int32_t (*adi_hal_LogLevelSet)(void*   devHalCfg,
                               int32_t logLevel) = NULL;

int32_t (*adi_hal_LogLevelGet)(void*    devHalCfg,
                               int32_t* logLevel) = NULL;

int32_t (*adi_hal_LogWrite)(void*       devHalCfg,
                            int32_t     logLevel,
                            const char* comment,
                            va_list     args) = NULL;

int32_t (*adi_hal_LogFileClose)(void* devHalCfg) = NULL;

/* Timer interface */
int32_t (*adi_hal_Wait_ms)(void*    devHalCfg,
                           uint32_t time_ms) = NULL;

int32_t (*adi_hal_Wait_us)(void*    devHalCfg,
                           uint32_t time_us) = NULL;

/* only required to support the FPGA / BBIC control interface */
int32_t (*adi_hal_BbicRegisterRead)(void*     devHalCfg,
                                    uint32_t  addr,
                                    uint32_t* data) = NULL;

int32_t (*adi_hal_BbicRegisterWrite)(void*    devHalCfg,
                                     uint32_t addr,
                                     uint32_t data) = NULL;

int32_t (*adi_hal_BbicRegistersRead)(void*    devHalCfg,
                                     uint32_t addr,
                                     uint32_t data[],
                                     uint32_t numDataWords) = NULL;

int32_t (*adi_hal_BbicRegistersWrite)(void*    devHalCfg,
                                      uint32_t addr,
                                      uint32_t data[],
                                      uint32_t numDataWords) = NULL;

/**
 * \brief Platform setup
 *
 * \param devHalInfo void pointer to be casted to the HAL config structure
 * \param platform Platform to be assigning the function pointers
 *
 * \return
 */
int32_t adi_hal_PlatformSetup(void*               devHalInfo,
                              adi_hal_Platforms_e platform)
{
    UNUSED_PARA(devHalInfo);
    adi_hal_Err_e error = ADI_HAL_OK;
    switch (platform)
    {
    case ADI_ADS9_PLATFORM:
        adi_hal_HwOpen = ads9_HwOpen;
        adi_hal_HwClose         = ads9_HwClose;
        adi_hal_HwReset         = ads9_HwReset;
        adi_hal_DevHalCfgCreate = ads9_DevHalCfgCreate;
        adi_hal_DevHalCfgFree   = ads9_DevHalCfgFree;
	    adi_hal_HwVerify = ads9_HwVerify;

        adi_hal_SpiInit              = ads9_SpiInit;     /* TODO: remove?  called by HwOpen() */
        adrv9025_hal_SpiWrite             = ads9_SpiWrite_v2; //ads9_SpiWrite;
        adrv9025_hal_SpiRead              = ads9_SpiRead_v2;
        adi_hal_CustomSpiStreamWrite = NULL;
        adi_hal_CustomSpiStreamRead  = NULL;

        adi_hal_LogFileOpen  = ads9_LogFileOpen;
        adi_hal_LogLevelSet  = ads9_LogLevelSet;
        adi_hal_LogLevelGet  = ads9_LogLevelGet;
        adi_hal_LogWrite     = ads9_LogWrite;
        adi_hal_LogFileClose = ads9_LogFileClose;

        adi_hal_Wait_us = ads9_TimerWait_us;
        adi_hal_Wait_ms = ads9_TimerWait_ms;

        /* only required to support the ADI FPGA*/
        adi_hal_BbicRegisterRead   = ads9_BbicRegisterRead;
        adi_hal_BbicRegisterWrite  = ads9_BbicRegisterWrite;
        adi_hal_BbicRegistersRead  = ads9_BbicRegistersRead;
        adi_hal_BbicRegistersWrite = ads9_BbicRegistersWrite;

        break;

    case ADI_ADS8_PLATFORM:
        adi_hal_HwOpen = ads8_HwOpen;
        adi_hal_HwClose         = ads8_HwClose;
        adi_hal_HwReset         = ads8_HwReset;
        adi_hal_DevHalCfgCreate = ads8_DevHalCfgCreate;
        adi_hal_DevHalCfgFree   = ads8_DevHalCfgFree;
	    adi_hal_HwVerify = ads8_HwVerify;

        adi_hal_SpiInit              = ads8_SpiInit; /* TODO: remove?  called by HwOpen() */
        adrv9025_hal_SpiWrite             = ads8_SpiWrite_v2;
        adrv9025_hal_SpiRead              = ads8_SpiRead_v2;
        adi_hal_CustomSpiStreamWrite = NULL;
        adi_hal_CustomSpiStreamRead  = NULL;

        adi_hal_LogFileOpen  = ads8_LogFileOpen;
        adi_hal_LogLevelSet  = ads8_LogLevelSet;
        adi_hal_LogLevelGet  = ads8_LogLevelGet;
        adi_hal_LogWrite     = ads8_LogWrite;
        adi_hal_LogFileClose = ads8_LogFileClose;

        adi_hal_Wait_us = ads8_TimerWait_us;
        adi_hal_Wait_ms = ads8_TimerWait_ms;

        /* only required to support the ADI FPGA*/
        adi_hal_BbicRegisterRead   = ads8_BbicRegisterRead;
        adi_hal_BbicRegisterWrite  = ads8_BbicRegisterWrite;
        adi_hal_BbicRegistersRead  = ads8_BbicRegistersRead;
        adi_hal_BbicRegistersWrite = ads8_BbicRegistersWrite;

        break;


    default:
        error = ADI_HAL_GEN_SW;
        break;
    }

    return error;
}
