/**
 * \file
 * \brief Types for configuring Low Dropout Regulators (LDOs)
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_POWERMANAGEMENT_TYPES_H_
#define _ADRV9001_POWERMANAGEMENT_TYPES_H_

#define ADI_ADRV9001_NUM_LDOS 19

/**
 * \brief LDO configurations for power saving
 */
typedef enum adi_adrv9001_LdoPowerSavingMode
{
    ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,   /*!< Normal operation */
    ADI_ADRV9001_LDO_POWER_SAVING_MODE_2,   /*!< Always off via PCB wiring */
    ADI_ADRV9001_LDO_POWER_SAVING_MODE_3,   /*!< Always off via software */
    ADI_ADRV9001_LDO_POWER_SAVING_MODE_4,   /*!< Always off via software, VBAT shorted to VOUT */
    ADI_ADRV9001_LDO_POWER_SAVING_MODE_5    /*!< Bypass */
} adi_adrv9001_LdoPowerSavingMode_e;

typedef struct adi_adrv9001_PowerManagementSettings
{
    /**
     * \brief Power saving mode for each LDO
     *
     * Index | LDO
     * ----- | ---------------------
     *   0   | GP_LDO_1
     *   1   | DEV_CLK_LDO
     *   2   | CONVERTER_LDO
     *   3   | RX_1_LO_LDO
     *   4   | TX_1_LO_LDO
     *   5   | GP_LDO_2
     *   6   | RX_2_LO_LDO
     *   7   | TX_2_LO_LDO
     *   8   | CLK_PLL_SYNTH_LDO
     *   9   | CLK_PLL_VCO_LDO
     *  10   | CLK_PLL_LP_SYNTH_LDO
     *  11   | CLK_PLL_LP_VCO_LDO
     *  12   | LO1_PLL_SYNTH_LDO
     *  13   | LO1_PLL_VCO_LDO
     *  14   | LO2_PLL_SYNTH_LDO
     *  15   | LO2_PLL_VCO_LDO
     *  16   | AUX_PLL_SYNTH_LDO
     *  17   | AUX_PLL_VCO_LDO
     *  18   | SRAM_LDO
     */
    adi_adrv9001_LdoPowerSavingMode_e ldoPowerSavingModes[ADI_ADRV9001_NUM_LDOS];
} adi_adrv9001_PowerManagementSettings_t;

#endif