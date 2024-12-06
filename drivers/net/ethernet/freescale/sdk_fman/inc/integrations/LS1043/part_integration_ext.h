/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
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

/**

 @File          part_integration_ext.h

 @Description   T4240 external definitions and structures.
*//***************************************************************************/
#ifndef __PART_INTEGRATION_EXT_H
#define __PART_INTEGRATION_EXT_H

#include "std_ext.h"
#include "ddr_std_ext.h"
#include "enet_ext.h"
#include "dpaa_integration_ext.h"


/**************************************************************************//**
 @Group         T4240_chip_id T4240 Application Programming Interface

 @Description   T4240 Chip functions,definitions and enums.

 @{
*//***************************************************************************/

#define INTG_MAX_NUM_OF_CORES   4

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
    e_MODULE_ID_IFC,
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
    e_MODULE_ID_FM,                 /**< Frame manager module */
    e_MODULE_ID_FM_RTC,             /**< FM Real-Time-Clock */
    e_MODULE_ID_FM_MURAM,           /**< FM Multi-User-RAM */
    e_MODULE_ID_FM_BMI,             /**< FM BMI block */
    e_MODULE_ID_FM_QMI,             /**< FM QMI block */
    e_MODULE_ID_FM_PARSER,          /**< FM parser block */
    e_MODULE_ID_FM_PORT_HO1,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO2,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO3,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO4,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO5,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO6,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO7,        /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_1GRx1,      /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GRx2,      /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GRx3,      /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GRx4,      /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GRx5,      /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GRx6,      /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_10GRx1,     /**< FM Rx 10G MAC port block */
    e_MODULE_ID_FM_PORT_10GRx2,     /**< FM Rx 10G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx1,      /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx2,      /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx3,      /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx4,      /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx5,      /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx6,      /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_10GTx1,     /**< FM Tx 10G MAC port block */
    e_MODULE_ID_FM_PORT_10GTx2,     /**< FM Tx 10G MAC port block */
    e_MODULE_ID_FM_PLCR,            /**< FM Policer */
    e_MODULE_ID_FM_KG,              /**< FM Keygen */
    e_MODULE_ID_FM_DMA,             /**< FM DMA */
    e_MODULE_ID_FM_FPM,             /**< FM FPM */
    e_MODULE_ID_FM_IRAM,            /**< FM Instruction-RAM */
    e_MODULE_ID_FM_1GMDIO,          /**< FM 1G MDIO MAC */
    e_MODULE_ID_FM_10GMDIO,         /**< FM 10G MDIO */
    e_MODULE_ID_FM_PRS_IRAM,        /**< FM SW-parser Instruction-RAM */
    e_MODULE_ID_FM_1GMAC1,          /**< FM 1G MAC #1 */
    e_MODULE_ID_FM_1GMAC2,          /**< FM 1G MAC #2 */
    e_MODULE_ID_FM_1GMAC3,          /**< FM 1G MAC #3 */
    e_MODULE_ID_FM_1GMAC4,          /**< FM 1G MAC #4 */
    e_MODULE_ID_FM_1GMAC5,          /**< FM 1G MAC #5 */
    e_MODULE_ID_FM_1GMAC6,          /**< FM 1G MAC #6 */
    e_MODULE_ID_FM_10GMAC1,         /**< FM 10G MAC */
    e_MODULE_ID_FM_10GMAC2,         /**< FM 10G MAC */

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

    e_MODULE_ID_PIC,                /**< PIC */
    e_MODULE_ID_GPIO,               /**< GPIO */
    e_MODULE_ID_SERDES,             /**< SERDES */
    e_MODULE_ID_CPC_1,              /**< CoreNet-Platform-Cache 1 */
    e_MODULE_ID_CPC_2,              /**< CoreNet-Platform-Cache 2 */

    e_MODULE_ID_SRIO_PORTS,         /**< RapidIO controller */

    e_MODULE_ID_DUMMY_LAST
} e_ModuleId;

#define NUM_OF_MODULES  e_MODULE_ID_DUMMY_LAST


#endif /* __PART_INTEGRATION_EXT_H */
