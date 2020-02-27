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

 @File          dpaa_integration_ext.h

 @Description   P1023 FM external definitions and structures.
*//***************************************************************************/
#ifndef __DPAA_INTEGRATION_EXT_H
#define __DPAA_INTEGRATION_EXT_H

#include "std_ext.h"


#define DPAA_VERSION    10

typedef enum e_DpaaSwPortal {
    e_DPAA_SWPORTAL0 = 0,
    e_DPAA_SWPORTAL1,
    e_DPAA_SWPORTAL2,
    e_DPAA_SWPORTAL_DUMMY_LAST
} e_DpaaSwPortal;

typedef enum {
    e_DPAA_DCPORTAL0 = 0,
    e_DPAA_DCPORTAL2,
    e_DPAA_DCPORTAL_DUMMY_LAST
} e_DpaaDcPortal;

#define DPAA_MAX_NUM_OF_SW_PORTALS      e_DPAA_SWPORTAL_DUMMY_LAST
#define DPAA_MAX_NUM_OF_DC_PORTALS      e_DPAA_DCPORTAL_DUMMY_LAST

/*****************************************************************************
 QMAN INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define QM_MAX_NUM_OF_POOL_CHANNELS 3
#define QM_MAX_NUM_OF_WQ            8
#define QM_MAX_NUM_OF_SWP_AS        2
#define QM_MAX_NUM_OF_CGS           64
#define QM_MAX_NUM_OF_FQIDS         (16*MEGABYTE)

typedef enum {
    e_QM_FQ_CHANNEL_SWPORTAL0 = 0,
    e_QM_FQ_CHANNEL_SWPORTAL1,
    e_QM_FQ_CHANNEL_SWPORTAL2,

    e_QM_FQ_CHANNEL_POOL1 = 0x21,
    e_QM_FQ_CHANNEL_POOL2,
    e_QM_FQ_CHANNEL_POOL3,

    e_QM_FQ_CHANNEL_FMAN0_SP0 = 0x40,
    e_QM_FQ_CHANNEL_FMAN0_SP1,
    e_QM_FQ_CHANNEL_FMAN0_SP2,
    e_QM_FQ_CHANNEL_FMAN0_SP3,
    e_QM_FQ_CHANNEL_FMAN0_SP4,
    e_QM_FQ_CHANNEL_FMAN0_SP5,
    e_QM_FQ_CHANNEL_FMAN0_SP6,


    e_QM_FQ_CHANNEL_CAAM = 0x80
} e_QmFQChannel;

/*****************************************************************************
 BMAN INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define BM_MAX_NUM_OF_POOLS         8

/*****************************************************************************
 SEC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define SEC_NUM_OF_DECOS    2
#define SEC_ALL_DECOS_MASK  0x00000003
#define SEC_RNGB
#define SEC_NO_ESP_TRAILER_REMOVAL

/*****************************************************************************
 FM INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define INTG_MAX_NUM_OF_FM          1

/* Ports defines */
#define FM_MAX_NUM_OF_1G_MACS       2
#define FM_MAX_NUM_OF_10G_MACS      0
#define FM_MAX_NUM_OF_MACS          (FM_MAX_NUM_OF_1G_MACS + FM_MAX_NUM_OF_10G_MACS)
#define FM_MAX_NUM_OF_OH_PORTS      5

#define FM_MAX_NUM_OF_1G_RX_PORTS   FM_MAX_NUM_OF_1G_MACS
#define FM_MAX_NUM_OF_10G_RX_PORTS  FM_MAX_NUM_OF_10G_MACS
#define FM_MAX_NUM_OF_RX_PORTS      (FM_MAX_NUM_OF_10G_RX_PORTS + FM_MAX_NUM_OF_1G_RX_PORTS)

#define FM_MAX_NUM_OF_1G_TX_PORTS   FM_MAX_NUM_OF_1G_MACS
#define FM_MAX_NUM_OF_10G_TX_PORTS  FM_MAX_NUM_OF_10G_MACS
#define FM_MAX_NUM_OF_TX_PORTS      (FM_MAX_NUM_OF_10G_TX_PORTS + FM_MAX_NUM_OF_1G_TX_PORTS)

#define FM_MAX_NUM_OF_MACSECS       1

#define FM_MACSEC_SUPPORT

#define FM_LOW_END_RESTRICTION      /* prevents the use of TX port 1 with OP port 0 */

#define FM_PORT_MAX_NUM_OF_EXT_POOLS            4           /**< Number of external BM pools per Rx port */
#define FM_PORT_MAX_NUM_OF_OBSERVED_EXT_POOLS   2           /**< Number of Offline parsing port external BM pools per Rx port */
#define FM_PORT_NUM_OF_CONGESTION_GRPS          32          /**< Total number of congestion groups in QM */
#define FM_MAX_NUM_OF_SUB_PORTALS               7

/* Rams defines */
#define FM_MURAM_SIZE                   (64*KILOBYTE)
#define FM_IRAM_SIZE(major, minor)      (32 * KILOBYTE)
#define FM_NUM_OF_CTRL                  2

/* PCD defines */
#define FM_PCD_PLCR_NUM_ENTRIES         32                  /**< Total number of policer profiles */
#define FM_PCD_KG_NUM_OF_SCHEMES        16                  /**< Total number of KG schemes */
#define FM_PCD_MAX_NUM_OF_CLS_PLANS     128                 /**< Number of classification plan entries. */
#define FM_PCD_PRS_SW_PATCHES_SIZE      0x00000240          /**< Number of bytes saved for patches */
#define FM_PCD_SW_PRS_SIZE              0x00000800          /**< Total size of SW parser area */

/* RTC defines */
#define FM_RTC_NUM_OF_ALARMS            2
#define FM_RTC_NUM_OF_PERIODIC_PULSES   2
#define FM_RTC_NUM_OF_EXT_TRIGGERS      2

/* QMI defines */
#define QMI_MAX_NUM_OF_TNUMS            15

/* FPM defines */
#define FM_NUM_OF_FMAN_CTRL_EVENT_REGS  4

/* DMA defines */
#define DMA_THRESH_MAX_COMMQ            15
#define DMA_THRESH_MAX_BUF              7

/* BMI defines */
#define BMI_MAX_NUM_OF_TASKS            64
#define BMI_MAX_NUM_OF_DMAS             16
#define BMI_MAX_FIFO_SIZE              (FM_MURAM_SIZE)
#define PORT_MAX_WEIGHT                 4

/*****************************************************************************
 FM MACSEC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define NUM_OF_RX_SC                16
#define NUM_OF_TX_SC                16

#define NUM_OF_SA_PER_RX_SC         2
#define NUM_OF_SA_PER_TX_SC         2

/**************************************************************************//**
 @Description   Enum for inter-module interrupts registration
*//***************************************************************************/

/* 1023 unique features */
#define FM_QMI_NO_ECC_EXCEPTIONS
#define FM_CSI_CFED_LIMIT
#define FM_PEDANTIC_DMA
#define FM_QMI_NO_DEQ_OPTIONS_SUPPORT
#define FM_FIFO_ALLOCATION_ALG
#define FM_DEQ_PIPELINE_PARAMS_FOR_OP
#define FM_HAS_TOTAL_DMAS
#define FM_KG_NO_IPPID_SUPPORT
#define FM_NO_GUARANTEED_RESET_VALUES
#define FM_MAC_RESET

/* FM erratas */
#define FM_RX_PREAM_4_ERRATA_DTSEC_A001
#define FM_MAGIC_PACKET_UNRECOGNIZED_ERRATA_DTSEC2      /* No implementation, Out of LLD scope */

#define FM_DEBUG_TRACE_FMAN_A004                        /* No implementation, Out of LLD scope */
#define FM_INT_BUF_LEAK_FMAN_A005                       /* No implementation, Out of LLD scope. App must avoid S/G */

#define FM_GTS_AFTER_DROPPED_FRAME_ERRATA_DTSEC_A004839

/* #define FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173 */

/*
TKT056919 - axi12axi0 can hang if read request follows the single byte write on the very next cycle
TKT038900 - FM dma lockup occur due to AXI slave protocol violation
*/
#define FM_LOCKUP_ALIGNMENT_ERRATA_FMAN_SW004


#endif /* __DPAA_INTEGRATION_EXT_H */
