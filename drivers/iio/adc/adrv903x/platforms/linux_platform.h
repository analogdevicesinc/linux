/**
* Copyright 2015 - 2025 Analog Devices Inc.
* SPDX-License-Identifier: Apache-2.0
*/

/**
* \file linux_platform.h
* \brief Contains ADI Transceiver Hardware Abstraction functions for the Linux Kernel.
*        Analog Devices maintains and provides updates to this code layer for the Linux platform.
*        The end user should not modify this file or any code in this directory. The end user
*        may provide a similar platform layer that can be used in place of this platform layer, 
*        that uses the same function prototypes.
*/

#ifndef __LINUX_PLATFORM_H__
#define __LINUX_PLATFORM_H__

#include "adi_platform.h"

/**
* \brief Opens all necessary files and device drivers for a specific device
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_ERR_OK Function completed successfully, no action required
*/
ADI_API adi_hal_Err_e linux_adrv903x_HwOpen(void* const devHalCfg);

/**
* \brief    Service to control HwReset Signal via a Logic Level
*
*           Caller is responsible for handling triggering (i.e. Level or Edge)
*
* \param devHalCfg  Pointer to device instance specific platform settings
* \param pinLevel The desired pin logic level 0=low, 1=high to set the GPIO pin to.
*
* \retval ADI_HAL_ERR_OK Function completed successfully, no action required
*/
ADI_API adi_hal_Err_e linux_adrv903x_HwReset(void* const devHalCfg,
					     const uint8_t pinLevel);

/**
* \brief Gracefully shuts down the hardware closing any open resources
*        such as log files, I2C, SPI, GPIO drivers, timer resources, etc.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval ADI_HAL_ERR_OK Function completed successfully, no action required
*/
ADI_API adi_hal_Err_e linux_adrv903x_HwClose(void* const devHalCfg);

/**
* \brief Write an array of 8-bit data to a SPI device
*
* The function will write numTxBytes number of bytes to the SPI device 
* selected in the devHalCfg structure.
*
* \dep_begin
* \dep{adi_hal_SpiCfg_t}
* \dep{adi_hal_SpiCfg_t->fd}
* \dep_end
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Pointer to byte array txData buffer that has numTxBytes number of bytes
* \param numTxBytes The length of txData array
*
* \retval ADI_HAL_ERR_OK function completed successfully, no action required
*/
ADI_API adi_hal_Err_e linux_adrv903x_SpiWrite(void* const devHalCfg,
					      const uint8_t txData[],
					      const uint32_t numTxBytes);

/**
* \brief Read one or more bytes from the device specified by the devHalCfg structure
*
* The function will read numTxRxBytes number of bytes from the SPI device selected in
* the devHalCfg parameter and store the resulting data sent by the device in the rxData
* data buffer.
*
* For each byte in txData written to the device, a byte is read and returned by this 
* function at the pointer provided by the rxData parameter.
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param txData Pointer to byte array that has numTxRxBytes number of bytes
* \param rxData Pointer to byte array where read back data will be returned,
		that is at least numTxRxBytes in size.
* \param numTxBytes The length of txData and rxData arrays
*
* \retval ADI_HAL_ERR_OK function completed successfully, no action required
*/
ADI_API adi_hal_Err_e linux_adrv903x_SpiRead(void* const devHalCfg,
					     const uint8_t txData[],
					     uint8_t rxData[],
					     const uint32_t numTxRxBytes);

/**
* \brief Opens a logFile. If the file is already open it will be closed and reopened.
*
* This function does nothing in case of the Linux kernel driver.
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param filename The user provided name of the file to open.
*
* \retval adi_hal_Err_e - ADI_HAL_ERR_OK if successful
*
*/
ADI_API adi_hal_Err_e linux_adrv903x_LogFileOpen(void* const devHalCfg,
						 const char* const filename);

/**
* \brief Service to Close a Log File
*
* This function does nothing in case of the Linux kernel driver.
*
* \param devHalCfg Pointer to device instance specific platform settings
*
* \retval adi_hal_Err_e - ADI_HAL_ERR_OK if successful
*
*/
ADI_API adi_hal_Err_e linux_adrv903x_LogFileClose(void* const devHalCfg);

/**
* \brief Sets the log level, allowing the end user to select the granularity of
*        what events get logged.
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param logMask A mask of valid log levels to allow to be written to the log file.
*
* \retval adi_hal_Err_e - ADI_HAL_ERR_OK if successful
*
*/
ADI_API adi_hal_Err_e linux_adrv903x_LogLevelSet(void* const devHalCfg,
						 const uint32_t logMask);

/**
 * \brief Gets the currently set log level: the mask of different types of log
 *         events that are currently enabled to be logged.
 *
 * \param devHalCfg Pointer to device instance specific platform settings
 * \param logMask Returns the current log level mask.
 *
 * \retval adi_hal_Err_e - ADI_HAL_ERR_OK if successful
 *
 */
ADI_API adi_hal_Err_e linux_adrv903x_LogLevelGet(void* const devHalCfg,
						 uint32_t* const logMask);

/**
* \brief Writes a message to the currently open logFile specified in the 
*        adi_hal_LogCfg_t of the devHalCfg structure passed
* 
* Uses the vfprintf functionality to allow the user to supply the format and
* the number of arguments that will be logged.
*
* \param devHalCfg  Pointer to device instance specific platform settings
* \param logLevel   the log level to be written into
* \param comment    the string to include in the line added to the log.
* \param argp       variable argument list to be printed
*
* \retval adi_hal_Err_e - ADI_HAL_ERR_OK if successful
*
*/
ADI_API adi_hal_Err_e linux_adrv903x_LogWrite(void* const devHalCfg,
                                    	      const adi_hal_LogLevel_e logLevel,
                                    	      const uint8_t indent,
                                    	      const char* const comment,
                                    	      va_list argp);

/**
* \brief Provides a blocking delay of the current thread
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param time_ms the Time to delay in milli seconds
*
* \retval ADI_HAL_ERR_OK Function completed successfully
*/
ADI_API adi_hal_Err_e linux_adrv903x_TimerWait_ms(void* const devHalCfg,
						  const uint32_t time_ms);

/**
* \brief Provides a blocking delay of the current thread
*
* \param devHalCfg Pointer to device instance specific platform settings
* \param time_us the time to delay in micro seconds
*
* \retval ADI_HAL_ERR_OK Function completed successfully
*/
ADI_API adi_hal_Err_e linux_adrv903x_TimerWait_us(void* const devHalCfg,
						  const uint32_t time_us);

/**
 * \brief Tls get. This function will return the same value, single-threaded
 *
 * \param tlsType Key value associated with a thread
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
ADI_API void* linux_adrv903x_TlsGet(const adi_hal_TlsType_e tlsType);

/**
 * \brief Tls set. This function will set the same value, single-threaded
 *
 * \param tlsType Key value associated with a thread
 * \param value Value to associate to a thread
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
ADI_API adi_hal_Err_e linux_adrv903x_TlsSet(const adi_hal_TlsType_e tlsType,
				     void* const value);

/**
 * \brief Mutex init function - no action, single threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
ADI_API adi_hal_Err_e linux_adrv903x_MutexInit(adi_hal_mutex_t* const mutex);

/**
 * \brief Mutex destroy function - no action, single threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
ADI_API adi_hal_Err_e linux_adrv903x_MutexDestroy(adi_hal_mutex_t* const mutex);

/**
 * \brief Mutex lock function - no action, single threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
ADI_API adi_hal_Err_e linux_adrv903x_MutexLock(adi_hal_mutex_t* const mutex);

/**
 * \brief Mutex unlock function  - no action, single threaded
 *
 * \param mutex Mutex instance
 *
 * \retval ADI_HAL_OK Function completed successfully, no action required
 */
ADI_API adi_hal_Err_e linux_adrv903x_MutexUnlock(adi_hal_mutex_t* const mutex);

#endif /*__LINUX_PLATFORM_H__*/