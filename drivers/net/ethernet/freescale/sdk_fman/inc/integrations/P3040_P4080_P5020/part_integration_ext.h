/* Copyright (c) 2008-2012 Freescale Semiconductor, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************//**
 @File          part_integration_ext.h

 @Description   P3040/P4080/P5020 external definitions and structures.
*//***************************************************************************/
#ifndef __PART_INTEGRATION_EXT_H
#define __PART_INTEGRATION_EXT_H

#include "std_ext.h"
#include "dpaa_integration_ext.h"


/**************************************************************************//**
 @Group         P3040/P4080/P5020_chip_id P5020 Application Programming Interface

 @Description   P3040/P4080/P5020 Chip functions,definitions and enums.

 @{
*//***************************************************************************/

#define CORE_E500MC

#define INTG_MAX_NUM_OF_CORES   1


/**************************************************************************//**
 @Description   Module types.
*//***************************************************************************/
typedef enum e_ModuleId
{
    e_MODULE_ID_DUART_1 = 0,
    e_MODULE_ID_DUART_2,
    e_MODULE_ID_DUART_3,
    e_MODULE_ID_DUART_4,
    e_MODULE_ID_LAW,
    e_MODULE_ID_LBC,
    e_MODULE_ID_PAMU,
    e_MODULE_ID_QM,                 /**< Queue manager module */
    e_MODULE_ID_BM,                 /**< Buffer manager module */
    e_MODULE_ID_QM_CE_PORTAL_0,
    e_MODULE_ID_QM_CI_PORTAL_0,
    e_MODULE_ID_QM_CE_PORTAL_1,
    e_MODULE_ID_QM_CI_PORTAL_1,
    e_MODULE_ID_QM_CE_PORTAL_2,
    e_MODULE_ID_QM_CI_PORTAL_2,
    e_MODULE_ID_QM_CE_PORTAL_3,
    e_MODULE_ID_QM_CI_PORTAL_3,
    e_MODULE_ID_QM_CE_PORTAL_4,
    e_MODULE_ID_QM_CI_PORTAL_4,
    e_MODULE_ID_QM_CE_PORTAL_5,
    e_MODULE_ID_QM_CI_PORTAL_5,
    e_MODULE_ID_QM_CE_PORTAL_6,
    e_MODULE_ID_QM_CI_PORTAL_6,
    e_MODULE_ID_QM_CE_PORTAL_7,
    e_MODULE_ID_QM_CI_PORTAL_7,
    e_MODULE_ID_QM_CE_PORTAL_8,
    e_MODULE_ID_QM_CI_PORTAL_8,
    e_MODULE_ID_QM_CE_PORTAL_9,
    e_MODULE_ID_QM_CI_PORTAL_9,
    e_MODULE_ID_BM_CE_PORTAL_0,
    e_MODULE_ID_BM_CI_PORTAL_0,
    e_MODULE_ID_BM_CE_PORTAL_1,
    e_MODULE_ID_BM_CI_PORTAL_1,
    e_MODULE_ID_BM_CE_PORTAL_2,
    e_MODULE_ID_BM_CI_PORTAL_2,
    e_MODULE_ID_BM_CE_PORTAL_3,
    e_MODULE_ID_BM_CI_PORTAL_3,
    e_MODULE_ID_BM_CE_PORTAL_4,
    e_MODULE_ID_BM_CI_PORTAL_4,
    e_MODULE_ID_BM_CE_PORTAL_5,
    e_MODULE_ID_BM_CI_PORTAL_5,
    e_MODULE_ID_BM_CE_PORTAL_6,
    e_MODULE_ID_BM_CI_PORTAL_6,
    e_MODULE_ID_BM_CE_PORTAL_7,
    e_MODULE_ID_BM_CI_PORTAL_7,
    e_MODULE_ID_BM_CE_PORTAL_8,
    e_MODULE_ID_BM_CI_PORTAL_8,
    e_MODULE_ID_BM_CE_PORTAL_9,
    e_MODULE_ID_BM_CI_PORTAL_9,
    e_MODULE_ID_FM1,                /**< Frame manager #1 module */
    e_MODULE_ID_FM1_RTC,            /**< FM Real-Time-Clock */
    e_MODULE_ID_FM1_MURAM,          /**< FM Multi-User-RAM */
    e_MODULE_ID_FM1_BMI,            /**< FM BMI block */
    e_MODULE_ID_FM1_QMI,            /**< FM QMI block */
    e_MODULE_ID_FM1_PRS,            /**< FM parser block */
    e_MODULE_ID_FM1_PORT_HO0,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_HO1,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_HO2,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_HO3,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_HO4,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_HO5,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_HO6,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM1_PORT_1GRx0,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GRx1,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GRx2,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GRx3,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GRx4,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_10GRx0,    /**< FM Rx 10G MAC port block */
    e_MODULE_ID_FM1_PORT_1GTx0,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GTx1,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GTx2,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GTx3,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_1GTx4,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM1_PORT_10GTx0,    /**< FM Tx 10G MAC port block */
    e_MODULE_ID_FM1_PLCR,           /**< FM Policer */
    e_MODULE_ID_FM1_KG,             /**< FM Keygen */
    e_MODULE_ID_FM1_DMA,            /**< FM DMA */
    e_MODULE_ID_FM1_FPM,            /**< FM FPM */
    e_MODULE_ID_FM1_IRAM,           /**< FM Instruction-RAM */
    e_MODULE_ID_FM1_1GMDIO0,        /**< FM 1G MDIO MAC 0*/
    e_MODULE_ID_FM1_1GMDIO1,        /**< FM 1G MDIO MAC 1*/
    e_MODULE_ID_FM1_1GMDIO2,        /**< FM 1G MDIO MAC 2*/
    e_MODULE_ID_FM1_1GMDIO3,        /**< FM 1G MDIO MAC 3*/
    e_MODULE_ID_FM1_10GMDIO,        /**< FM 10G MDIO */
    e_MODULE_ID_FM1_PRS_IRAM,       /**< FM SW-parser Instruction-RAM */
    e_MODULE_ID_FM1_1GMAC0,         /**< FM 1G MAC #0 */
    e_MODULE_ID_FM1_1GMAC1,         /**< FM 1G MAC #1 */
    e_MODULE_ID_FM1_1GMAC2,         /**< FM 1G MAC #2 */
    e_MODULE_ID_FM1_1GMAC3,         /**< FM 1G MAC #3 */
    e_MODULE_ID_FM1_10GMAC0,        /**< FM 10G MAC #0 */

    e_MODULE_ID_FM2,                /**< Frame manager #2 module */
    e_MODULE_ID_FM2_RTC,            /**< FM Real-Time-Clock */
    e_MODULE_ID_FM2_MURAM,          /**< FM Multi-User-RAM */
    e_MODULE_ID_FM2_BMI,            /**< FM BMI block */
    e_MODULE_ID_FM2_QMI,            /**< FM QMI block */
    e_MODULE_ID_FM2_PRS,            /**< FM parser block */
    e_MODULE_ID_FM2_PORT_HO0,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_HO1,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_HO2,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_HO3,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_HO4,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_HO5,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_HO6,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM2_PORT_1GRx0,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_1GRx1,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_1GRx2,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_1GRx3,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_10GRx0,    /**< FM Rx 10G MAC port block */
    e_MODULE_ID_FM2_PORT_1GTx0,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_1GTx1,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_1GTx2,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_1GTx3,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM2_PORT_10GTx0,    /**< FM Tx 10G MAC port block */
    e_MODULE_ID_FM2_PLCR,           /**< FM Policer */
    e_MODULE_ID_FM2_KG,             /**< FM Keygen */
    e_MODULE_ID_FM2_DMA,            /**< FM DMA */
    e_MODULE_ID_FM2_FPM,            /**< FM FPM */
    e_MODULE_ID_FM2_IRAM,           /**< FM Instruction-RAM */
    e_MODULE_ID_FM2_1GMDIO0,        /**< FM 1G MDIO MAC 0*/
    e_MODULE_ID_FM2_1GMDIO1,        /**< FM 1G MDIO MAC 1*/
    e_MODULE_ID_FM2_1GMDIO2,        /**< FM 1G MDIO MAC 2*/
    e_MODULE_ID_FM2_1GMDIO3,        /**< FM 1G MDIO MAC 3*/
    e_MODULE_ID_FM2_10GMDIO,        /**< FM 10G MDIO */
    e_MODULE_ID_FM2_PRS_IRAM,       /**< FM SW-parser Instruction-RAM */
    e_MODULE_ID_FM2_1GMAC0,         /**< FM 1G MAC #0 */
    e_MODULE_ID_FM2_1GMAC1,         /**< FM 1G MAC #1 */
    e_MODULE_ID_FM2_1GMAC2,         /**< FM 1G MAC #2 */
    e_MODULE_ID_FM2_1GMAC3,         /**< FM 1G MAC #3 */
    e_MODULE_ID_FM2_10GMAC0,        /**< FM 10G MAC #0 */

    e_MODULE_ID_SEC_GEN,            /**< SEC 4.0 General registers      */
    e_MODULE_ID_SEC_QI,             /**< SEC 4.0 QI registers           */
    e_MODULE_ID_SEC_JQ0,            /**< SEC 4.0 JQ-0 registers         */
    e_MODULE_ID_SEC_JQ1,            /**< SEC 4.0 JQ-1 registers         */
    e_MODULE_ID_SEC_JQ2,            /**< SEC 4.0 JQ-2 registers         */
    e_MODULE_ID_SEC_JQ3,            /**< SEC 4.0 JQ-3 registers         */
    e_MODULE_ID_SEC_RTIC,           /**< SEC 4.0 RTIC registers         */
    e_MODULE_ID_SEC_DECO0_CCB0,     /**< SEC 4.0 DECO-0/CCB-0 registers */
    e_MODULE_ID_SEC_DECO1_CCB1,     /**< SEC 4.0 DECO-1/CCB-1 registers */
    e_MODULE_ID_SEC_DECO2_CCB2,     /**< SEC 4.0 DECO-2/CCB-2 registers */
    e_MODULE_ID_SEC_DECO3_CCB3,     /**< SEC 4.0 DECO-3/CCB-3 registers */
    e_MODULE_ID_SEC_DECO4_CCB4,     /**< SEC 4.0 DECO-4/CCB-4 registers */

    e_MODULE_ID_MPIC,               /**< MPIC */
    e_MODULE_ID_GPIO,               /**< GPIO */
    e_MODULE_ID_SERDES,             /**< SERDES */
    e_MODULE_ID_CPC_1,              /**< CoreNet-Platform-Cache 1 */
    e_MODULE_ID_CPC_2,              /**< CoreNet-Platform-Cache 2 */

    e_MODULE_ID_SRIO_PORTS,         /**< RapidIO controller */
    e_MODULE_ID_SRIO_MU,            /**< RapidIO messaging unit module */

    e_MODULE_ID_DUMMY_LAST
} e_ModuleId;

#define NUM_OF_MODULES  e_MODULE_ID_DUMMY_LAST

#if 0 /* using unified values */
/*****************************************************************************
 INTEGRATION-SPECIFIC MODULE CODES
******************************************************************************/
#define MODULE_UNKNOWN          0x00000000
#define MODULE_MEM              0x00010000
#define MODULE_MM               0x00020000
#define MODULE_CORE             0x00030000
#define MODULE_CHIP             0x00040000
#define MODULE_PLTFRM           0x00050000
#define MODULE_PM               0x00060000
#define MODULE_MMU              0x00070000
#define MODULE_PIC              0x00080000
#define MODULE_CPC              0x00090000
#define MODULE_DUART            0x000a0000
#define MODULE_SERDES           0x000b0000
#define MODULE_PIO              0x000c0000
#define MODULE_QM               0x000d0000
#define MODULE_BM               0x000e0000
#define MODULE_SEC              0x000f0000
#define MODULE_LAW              0x00100000
#define MODULE_LBC              0x00110000
#define MODULE_PAMU             0x00120000
#define MODULE_FM               0x00130000
#define MODULE_FM_MURAM         0x00140000
#define MODULE_FM_PCD           0x00150000
#define MODULE_FM_RTC           0x00160000
#define MODULE_FM_MAC           0x00170000
#define MODULE_FM_PORT          0x00180000
#define MODULE_FM_SP            0x00190000
#define MODULE_DPA_PORT         0x001a0000
#define MODULE_MII              0x001b0000
#define MODULE_I2C              0x001c0000
#define MODULE_DMA              0x001d0000
#define MODULE_DDR              0x001e0000
#define MODULE_ESPI             0x001f0000
#define MODULE_DPAA_IPSEC       0x00200000
#endif /* using unified values */

/*****************************************************************************
 PAMU INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define PAMU_NUM_OF_PARTITIONS  5

#define PAMU_PICS_AVICS_ERRATA_PAMU3

/*****************************************************************************
 LAW INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define LAW_NUM_OF_WINDOWS      32
#define LAW_MIN_WINDOW_SIZE     0x0000000000001000LL    /**< 4KB */
#define LAW_MAX_WINDOW_SIZE     0x0000002000000000LL    /**< 64GB */


/*****************************************************************************
 LBC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
/**************************************************************************//**
 @Group         lbc_exception_grp LBC Exception Unit

 @Description   LBC Exception unit API functions, definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Anchor        lbc_exbm

 @Collection    LBC Errors Bit Mask

                These errors are reported through the exceptions callback..
                The values can be or'ed in any combination in the errors mask
                parameter of the errors report structure.

                These errors can also be passed as a bit-mask to
                LBC_EnableErrorChecking() or LBC_DisableErrorChecking(),
                for enabling or disabling error checking.
 @{
*//***************************************************************************/
#define LBC_ERR_BUS_MONITOR     0x80000000  /**< Bus monitor error */
#define LBC_ERR_PARITY_ECC      0x20000000  /**< Parity error for GPCM/UPM */
#define LBC_ERR_WRITE_PROTECT   0x04000000  /**< Write protection error */
#define LBC_ERR_ATOMIC_WRITE    0x00800000  /**< Atomic write error */
#define LBC_ERR_ATOMIC_READ     0x00400000  /**< Atomic read error */
#define LBC_ERR_CHIP_SELECT     0x00080000  /**< Unrecognized chip select */

#define LBC_ERR_ALL             (LBC_ERR_BUS_MONITOR | LBC_ERR_PARITY_ECC | \
                                 LBC_ERR_WRITE_PROTECT | LBC_ERR_ATOMIC_WRITE | \
                                 LBC_ERR_ATOMIC_READ | LBC_ERR_CHIP_SELECT)
                                            /**< All possible errors */
/* @} */
/** @} */ /* end of lbc_exception_grp group */

#define LBC_INCORRECT_ERROR_REPORT_ERRATA

#define LBC_NUM_OF_BANKS            8
#define LBC_MAX_CS_SIZE             0x0000000100000000LL
#define LBC_ATOMIC_OPERATION_SUPPORT
#define LBC_PARITY_SUPPORT
#define LBC_ADDRESS_HOLD_TIME_CTRL
#define LBC_HIGH_CLK_DIVIDERS
#define LBC_FCM_AVAILABLE

/*****************************************************************************
 GPIO INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define GPIO_NUM_OF_PORTS   1   /**< Number of ports in GPIO module;
                                     Each port contains up to 32 i/O pins. */

#define GPIO_VALID_PIN_MASKS  \
    { /* Port A */ 0xFFFFFFFF }

#define GPIO_VALID_INTR_MASKS \
    { /* Port A */ 0xFFFFFFFF }

#endif /* __PART_INTEGRATION_EXT_H */
