/**
 * \file
 * \brief Contains ADRV9001 Frequency Hopping data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_FH_TYPES_H_
#define _ADI_ADRV9001_FH_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdint.h>
#endif
#include "adi_adrv9001_gpio_types.h"

/**
 * \brief Max number of GPIO pins for frequency selection. 
 */
#define ADI_ADRV9001_FH_MAX_NUM_FREQ_SELECT_PINS 6u /* 6 pins is enough to index all 64 frequencies in table */

/**
 * \brief Max number of hop tables
 */
#define ADI_ADRV9001_FH_MAX_HOP_TABLE_SIZE 64u

/**
 * \brief Max number of GPIO pins for gain selection.
 */
/* TODO: Really 10 pins would be required to reference all entries in the tx attenuation table.
   It's really up to the user's system and use as to how many PINS can be allocated. 
   We should looked into increasing this number */
#define  ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_PINS 3u

/**
 * \brief Minimum carrier frequency supported in frequency hopping mode
 */
#define ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ 30000000llu    /* 30 MHz */

/**
 * \brief Maximum carrier frequency supported in frequency hopping mode
 */
#define ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ 3200000000llu  /* 3.2 GHz */

/**
 * \brief Enumeration of frequency hopping modes
 */
typedef enum {
    ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS          = 0u,   /*!< Frequency hopping with LO muxing. Frequency hopping table is processed before the frequency hopping operation. */
    ADI_ADRV9001_FHMODE_LO_MUX_REALTIME_PROCESS    = 1u,   /*!< Frequency hopping with LO muxing. Frequency hopping table is processed during frequency hopping operation. */
    ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS = 2u,   /*!< Frequency hopping with LO retuning during transition time. Frequency hopping table is processed during frequency hopping operation. */
} adi_adrv9001_FhMode_e;

typedef enum {
    ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP      = 0u, /*!< Automatically increment through a frequency hopping table and wrap around once end has been reached */
    ADI_ADRV9001_TABLEINDEXCTRL_AUTO_PING_PONG = 1u, /*!< Ping pong operation between ADI_ADRV9001_FHHOPTABLE_A and ADI_ADRV9001_FHHOPTABLE_B. 
                                                          Automatically increment through one frequency hopping table and switch to the other 
                                                          once end has been reached */
    ADI_ADRV9001_TABLEINDEXCTRL_GPIO           = 2u, /*!< Use GPIO pins to index entries within a frequency hopping table */
} adi_adrv9001_TableIndexCtrl_e;

/**
 * \brief Enumeration of frequency hopping table ID's 
 */
typedef enum {
    ADI_ADRV9001_FHHOPTABLE_A = 0u, /*!< ID for hop table A in arm memory */
    ADI_ADRV9001_FHHOPTABLE_B = 1u, /*!< ID for hop table B in arm memory */
} adi_adrv9001_FhHopTable_e;

/**
 * \brief Enumeration of hop frame Indices
 */
typedef enum {
    ADI_ADRV9001_FHFRAMEINDEX_CURRENT_FRAME  = 0u, /*!< Hop frame currently in progress */
    ADI_ADRV9001_FHFRAMEINDEX_UPCOMING_FRAME = 1u, /*!< The upcoming hop frame after the current*/
    ADI_ADRV9001_FHFRAMEINDEX_NEXT_FRAME     = 2u, /*!< Next hop frame after the upcoming one (i.e. next-after-next) */
} adi_adrv9001_FhFrameIndex_e;

/**
 * \brief Settings for HOP frame information
 */
typedef struct {
    uint64_t hopFrequencyHz;       /*!< Operating frequency in Hz */
    int32_t  rxOffsetFrequencyHz;  /*!< Rx Offset frequency. This field is ignored by firmware if frame is not Rx, or if profile does not operation with an IF */
    uint8_t  rxGainIndex;          /*!< Starting gain index for hop frame. This field is ignored if frame is Tx */
    uint16_t txAttenuation_mdB;    /*!< Tx attenuation index. This field is ignored if frame is Rx */
} adi_adrv9001_FhHopFrame_t;

/**
 * \brief Frequency hopping configuration data
 */
typedef struct { 
    adi_adrv9001_FhMode_e            mode;                          /*!< Frequency hopping mode */
    uint8_t                          hopSignalChannelMask;          /*!< Set of channels to be controlled by hop pin.
                                                                          D0 -  Rx1           0 = Disabled, 1 = Mapped to hop pin (Enabled for frequency hopping)
                                                                          D1 -  Rx2           0 = Disabled, 1 = Mapped to hop pin (Enabled for frequency hopping)
                                                                          D2 -  Tx1           0 = Disabled, 1 = Mapped to hop pin (Enabled for frequency hopping)
                                                                          D3 -  Tx2           0 = Disabled, 1 = Mapped to hop pin (Enabled for frequency hopping)
                                                                    */
    /* This information is not written to ARMs version of _FhCfg_t */
    adi_adrv9001_GpioCfg_t           hopSignalGpioConfig;           /*!< Pin configuration. If GPIO is unassigned, HOP signal source is defaulted to SPI mode*/                                                               
	adi_adrv9001_TableIndexCtrl_e    tableIndexCtrl;                /*!< Select control type for frequency hopping table */
    bool                             gainSetupByPin;                /*!< Use GPIO Pins to provide a Tx/Rx gain index for next hop frame. If false, gain information is provided in hop table*/
    uint8_t                          minRxGainIndex;                /*!< Minimum Rx gain index for FH operation. Used for calibration over specified gain range. Valid range is from 0 to maxGainIndex  */
    uint8_t                          maxRxGainIndex;                /*!< Maximum Rx gain index for FH operation. Used for calibration over specified gain range. Valid range is from minGainIndex to 255 */
    uint16_t                         minTxAtten_mdB;                /*!< Minimum Tx attenuation level for FH operation. Used for calibration over specified attenuation range. Valid range is from 0 to maxTxAtten_mdB */
    uint16_t                         maxTxAtten_mdB;                /*!< Maximum Tx attenuation level for FH operation. Used for calibration over specified attenuation range. Valid range is from minTxAtten_mdB to 41950 */
    uint64_t                         minOperatingFrequency_Hz;      /*!< Minimum frequency used during FH operation. Used for calibration. Valid range is from 30Mhz to maxOperatingFrequency_Hz */
    uint64_t                         maxOperatingFrequency_Hz;      /*!< Maximum frequency used during FH operation. Used for calibration. Valid range is from minOperatingFrequency_Hz to 3.2GHz */
    uint32_t                         minFrameDuration_us;           /*!< Minimum frame duration to be supported during FH operation, in microseconds. */
	uint8_t                          txAnalogPowerOnFrameDelay;     /*!< A delay specified in terms of hop frames. Tx analog front end can be delayed relative to the first Tx setup rising edge. 
                                                                         Set this field to greater than 0 to support use cases where Tx propagation delay is longer than a single hop frame duration. */
    uint8_t                          numTableIndexPins;             /*!< Number of pins for table indexing */
	adi_adrv9001_GpioCfg_t           tableIndexGpioConfig[ADI_ADRV9001_FH_MAX_NUM_FREQ_SELECT_PINS]; /*!< Pin configuration for table index select. Ignored if tableIndexCtrl is not ADI_ADRV9001_TABLEINDEXCTRL_GPIO */

	uint8_t                          numGainCtrlPins;               /*!< Number of pins assigned for gain control */
	adi_adrv9001_GpioCfg_t           gainSelectGpioConfig[ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_PINS]; /*!< Pin configuration for gain select. Ignored if gainSetupByPin is false */

    adi_adrv9001_GpioCfg_t           hopTableSwitchGpioConfig;      /*!< Pin configuration for hop table select. Not an error if unassigned, just ignored.*/
} adi_adrv9001_FhCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_FH_TYPES_H_ */
