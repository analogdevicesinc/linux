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

#ifdef __GNUC__                    /* __unix__ verify if our linux image declare this */
  #define OS_Windows 0
#else     /* windows  */

#if _WIN64 == 0      /* _Win32  */
  #define OS_Windows 32
#elif _WIN64 == 1      /* _Win64  */
  #define OS_Windows 64
#endif
#endif

#include "adi_platform_types.h"

#ifdef _ADI_STS_PLATFORM
    #include "sts_hal/niSTSHAL.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE
/**
 * BBIC Init, Open, Close functions
 */
extern int32_t(*adi_hal_HwOpen)(void *devHalCfg);
extern int32_t(*adi_hal_HwClose)(void *devHalCfg);
extern int32_t(*adi_hal_HwReset)(void *devHalCfg, uint8_t pinLevel);
extern void* (*adi_hal_DevHalCfgCreate)(uint32_t interfaceMask, uint8_t spiChipSelect, const char *logFilename);
extern int32_t(*adi_hal_DevHalCfgFree)(void *devHalCfg);

/**
 * BBIC SPI functions
 */
extern int32_t(*adi_hal_SpiWrite)(void *devHalCfg, const uint8_t txData[], uint32_t numTxBytes);
extern int32_t(*adi_hal_SpiRead)(void *devHalCfg, const uint8_t txData[], uint8_t rxData[], uint32_t numRxBytes);

/**
 * BBIC I2C functions
 */
extern int32_t(*adi_hal_I2C_Write)(void *devHalCfg, uint8_t slaveAddress, const uint8_t wrData[], uint32_t numWrBytes);
extern int32_t(*adi_hal_I2C_Read)(void *devHalCfg, uint8_t slaveAddress, const uint8_t txData[], uint32_t numTxBytes, uint8_t rxData[], uint32_t numRxBytes);

/**
 * BBIC Logging functions
 */
extern int32_t(*adi_hal_LogFileOpen)(void *devHalCfg, const char *filename);
extern int32_t(*adi_hal_LogFileClose)(void *devHalCfg);
extern int32_t(*adi_hal_LogLevelSet)(void *devHalCfg, int32_t logLevel);
extern int32_t(*adi_hal_LogLevelGet)(void *devHalCfg, int32_t *logLevel);
extern int32_t(*adi_hal_LogWrite)(void *devHalCfg, int32_t logLevel, const char *comment, va_list argp);

/**
 * BBIC Timer functions
 */
extern int32_t(*adi_hal_Wait_us)(void *devHalCfg, uint32_t time_us);
extern int32_t(*adi_hal_Wait_ms)(void *devHalCfg, uint32_t time_ms);

/**
 * BBIC Hal layer setup
 */
extern int32_t adi_hal_PlatformSetup(void *devHalCfg, adi_hal_Platforms_e platform);

/**
 * BBIC File-IO abstraction
 */
extern int32_t(*adi_hal_ArmImagePageGet)(void *devHalCfg, const char *armImagePath, uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff);
extern int32_t(*adi_hal_StreamImagePageGet)(void *devHalCfg, const char *streamImagePath, uint32_t pageIndex, uint32_t pageSize, uint8_t *rdBuff);
extern int32_t(*adi_hal_RxGainTableEntryGet)(void *devHalCfg, const char *rxGainTablePath, uint16_t lineCount, uint8_t *gainIndex, uint8_t *rxFeGain, uint8_t *tiaControl, uint8_t *adcControl, uint8_t *extControl, uint16_t *phaseOffset, int16_t *digGain);
extern int32_t(*adi_hal_TxAttenTableEntryGet)(void *devHalCfg, const char *txAttenTablePath, uint16_t lineCount, uint16_t *attenIndex, uint8_t *txAttenHp, uint16_t *txAttenMult);

/**
 * BBIC GPIO functions - Currently only used to toggle hard reset pin on each device.
 *
 * ADI FPGA platform functions (only required for ADI FPGA platform)
 */
extern int32_t (*adi_hal_BbicRegisterRead)(void *devHalCfg, uint32_t addr, uint32_t *data);
extern int32_t (*adi_hal_BbicRegisterWrite)(void *devHalCfg, uint32_t addr, uint32_t data);
extern int32_t (*adi_hal_BbicRegistersRead)(void *devHalCfg, uint32_t addr, uint32_t data[], uint32_t numDataWords);
extern int32_t (*adi_hal_BbicRegistersWrite)(void *devHalCfg, uint32_t addr, uint32_t data[], uint32_t numDataWords);
#endif /* CLIENT_IGNORE */
    
/**
 * \brief Send the specified number of MCS pulses
 * 
 * \param[in] devHalCfg         User HAL configuration
 * \param[in] numberOfPulses    The number of times to pulse the MCS pin
 * 
 * \returns 0 indicating success, otherwise negative to indicate error
 */
extern int32_t (*adi_hal_Mcs_Pulse)(void *devHalCfg, uint8_t numberOfPulses);
    
/**
 * \brief Reset the SSI interface
 * 
 * \param[in] devHalCfg         User HAL configuration
 * 
 * \returns 0 indicating success, otherwise negative to indicate error
 */
extern int32_t(*adi_hal_ssi_Reset)(void *devHalCfg);

#ifdef __cplusplus
}
#endif
#endif /* __ADI_PLATFORM_H__ */


