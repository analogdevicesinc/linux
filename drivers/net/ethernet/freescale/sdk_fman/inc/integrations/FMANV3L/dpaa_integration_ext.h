/*
 * Copyright 2012 Freescale Semiconductor Inc.
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

 @Description   T4240 FM external definitions and structures.
*//***************************************************************************/
#ifndef __DPAA_INTEGRATION_EXT_H
#define __DPAA_INTEGRATION_EXT_H

#include "std_ext.h"


#define DPAA_VERSION    11

/**************************************************************************//**
 @Description   DPAA SW Portals Enumeration.
*//***************************************************************************/
typedef enum
{
    e_DPAA_SWPORTAL0 = 0,
    e_DPAA_SWPORTAL1,
    e_DPAA_SWPORTAL2,
    e_DPAA_SWPORTAL3,
    e_DPAA_SWPORTAL4,
    e_DPAA_SWPORTAL5,
    e_DPAA_SWPORTAL6,
    e_DPAA_SWPORTAL7,
    e_DPAA_SWPORTAL8,
    e_DPAA_SWPORTAL9,
    e_DPAA_SWPORTAL10,
    e_DPAA_SWPORTAL11,
    e_DPAA_SWPORTAL12,
    e_DPAA_SWPORTAL13,
    e_DPAA_SWPORTAL14,
    e_DPAA_SWPORTAL15,
    e_DPAA_SWPORTAL16,
    e_DPAA_SWPORTAL17,
    e_DPAA_SWPORTAL18,
    e_DPAA_SWPORTAL19,
    e_DPAA_SWPORTAL20,
    e_DPAA_SWPORTAL21,
    e_DPAA_SWPORTAL22,
    e_DPAA_SWPORTAL23,
    e_DPAA_SWPORTAL24,
    e_DPAA_SWPORTAL_DUMMY_LAST
} e_DpaaSwPortal;

/**************************************************************************//**
 @Description   DPAA Direct Connect Portals Enumeration.
*//***************************************************************************/
typedef enum
{
    e_DPAA_DCPORTAL0 = 0,
    e_DPAA_DCPORTAL1,
    e_DPAA_DCPORTAL2,
    e_DPAA_DCPORTAL_DUMMY_LAST
} e_DpaaDcPortal;

#define DPAA_MAX_NUM_OF_SW_PORTALS      e_DPAA_SWPORTAL_DUMMY_LAST
#define DPAA_MAX_NUM_OF_DC_PORTALS      e_DPAA_DCPORTAL_DUMMY_LAST

/*****************************************************************************
 QMan INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define QM_MAX_NUM_OF_POOL_CHANNELS     15      /**< Total number of channels, dedicated and pool */
#define QM_MAX_NUM_OF_WQ                8       /**< Number of work queues per channel */
#define QM_MAX_NUM_OF_CGS               256     /**< Congestion groups number */
#define QM_MAX_NUM_OF_FQIDS             (16 * MEGABYTE)
                                                /**< FQIDs range - 24 bits */

/**************************************************************************//**
 @Description   Work Queue Channel assignments in QMan.
*//***************************************************************************/
typedef enum
{
    e_QM_FQ_CHANNEL_SWPORTAL0 = 0x0,              /**< Dedicated channels serviced by software portals 0 to 24 */
    e_QM_FQ_CHANNEL_SWPORTAL1,
    e_QM_FQ_CHANNEL_SWPORTAL2,
    e_QM_FQ_CHANNEL_SWPORTAL3,
    e_QM_FQ_CHANNEL_SWPORTAL4,
    e_QM_FQ_CHANNEL_SWPORTAL5,
    e_QM_FQ_CHANNEL_SWPORTAL6,
    e_QM_FQ_CHANNEL_SWPORTAL7,
    e_QM_FQ_CHANNEL_SWPORTAL8,
    e_QM_FQ_CHANNEL_SWPORTAL9,
    e_QM_FQ_CHANNEL_SWPORTAL10,
    e_QM_FQ_CHANNEL_SWPORTAL11,
    e_QM_FQ_CHANNEL_SWPORTAL12,
    e_QM_FQ_CHANNEL_SWPORTAL13,
    e_QM_FQ_CHANNEL_SWPORTAL14,
    e_QM_FQ_CHANNEL_SWPORTAL15,
    e_QM_FQ_CHANNEL_SWPORTAL16,
    e_QM_FQ_CHANNEL_SWPORTAL17,
    e_QM_FQ_CHANNEL_SWPORTAL18,
    e_QM_FQ_CHANNEL_SWPORTAL19,
    e_QM_FQ_CHANNEL_SWPORTAL20,
    e_QM_FQ_CHANNEL_SWPORTAL21,
    e_QM_FQ_CHANNEL_SWPORTAL22,
    e_QM_FQ_CHANNEL_SWPORTAL23,
    e_QM_FQ_CHANNEL_SWPORTAL24,

    e_QM_FQ_CHANNEL_POOL1 = 0x401,               /**< Pool channels that can be serviced by any of the software portals */
    e_QM_FQ_CHANNEL_POOL2,
    e_QM_FQ_CHANNEL_POOL3,
    e_QM_FQ_CHANNEL_POOL4,
    e_QM_FQ_CHANNEL_POOL5,
    e_QM_FQ_CHANNEL_POOL6,
    e_QM_FQ_CHANNEL_POOL7,
    e_QM_FQ_CHANNEL_POOL8,
    e_QM_FQ_CHANNEL_POOL9,
    e_QM_FQ_CHANNEL_POOL10,
    e_QM_FQ_CHANNEL_POOL11,
    e_QM_FQ_CHANNEL_POOL12,
    e_QM_FQ_CHANNEL_POOL13,
    e_QM_FQ_CHANNEL_POOL14,
    e_QM_FQ_CHANNEL_POOL15,

    e_QM_FQ_CHANNEL_FMAN0_SP0 = 0x800,           /**< Dedicated channels serviced by Direct Connect Portal 0:
                                                      connected to FMan 0; assigned in incrementing order to
                                                      each sub-portal (SP) in the portal */
    e_QM_FQ_CHANNEL_FMAN0_SP1,
    e_QM_FQ_CHANNEL_FMAN0_SP2,
    e_QM_FQ_CHANNEL_FMAN0_SP3,
    e_QM_FQ_CHANNEL_FMAN0_SP4,
    e_QM_FQ_CHANNEL_FMAN0_SP5,
    e_QM_FQ_CHANNEL_FMAN0_SP6,
    e_QM_FQ_CHANNEL_FMAN0_SP7,
    e_QM_FQ_CHANNEL_FMAN0_SP8,
    e_QM_FQ_CHANNEL_FMAN0_SP9,
    e_QM_FQ_CHANNEL_FMAN0_SP10,
    e_QM_FQ_CHANNEL_FMAN0_SP11,
    e_QM_FQ_CHANNEL_FMAN0_SP12,
    e_QM_FQ_CHANNEL_FMAN0_SP13,
    e_QM_FQ_CHANNEL_FMAN0_SP14,
    e_QM_FQ_CHANNEL_FMAN0_SP15,

    e_QM_FQ_CHANNEL_RMAN_SP0 = 0x820,            /**< Dedicated channels serviced by Direct Connect Portal 1: connected to RMan */
    e_QM_FQ_CHANNEL_RMAN_SP1,

    e_QM_FQ_CHANNEL_CAAM = 0x840                 /**< Dedicated channel serviced by Direct Connect Portal 2:
                                                      connected to SEC */
} e_QmFQChannel;

/*****************************************************************************
 BMan INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define BM_MAX_NUM_OF_POOLS         64          /**< Number of buffers pools */

/*****************************************************************************
 SEC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define SEC_NUM_OF_DECOS            3
#define SEC_ALL_DECOS_MASK          0x00000003


/*****************************************************************************
 FM INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define INTG_MAX_NUM_OF_FM	    1
/* Ports defines */
#define FM_MAX_NUM_OF_1G_MACS	    5
#define FM_MAX_NUM_OF_10G_MACS	    1
#define FM_MAX_NUM_OF_MACS	    (FM_MAX_NUM_OF_1G_MACS + FM_MAX_NUM_OF_10G_MACS)
#define FM_MAX_NUM_OF_OH_PORTS	    4

#define FM_MAX_NUM_OF_1G_RX_PORTS   FM_MAX_NUM_OF_1G_MACS
#define FM_MAX_NUM_OF_10G_RX_PORTS  FM_MAX_NUM_OF_10G_MACS
#define FM_MAX_NUM_OF_RX_PORTS      (FM_MAX_NUM_OF_10G_RX_PORTS + FM_MAX_NUM_OF_1G_RX_PORTS)

#define FM_MAX_NUM_OF_1G_TX_PORTS   FM_MAX_NUM_OF_1G_MACS
#define FM_MAX_NUM_OF_10G_TX_PORTS  FM_MAX_NUM_OF_10G_MACS
#define FM_MAX_NUM_OF_TX_PORTS      (FM_MAX_NUM_OF_10G_TX_PORTS + FM_MAX_NUM_OF_1G_TX_PORTS)

#define FM_MAX_NUM_OF_MACSECS       1 /* Should be updated */

#define FM_PORT_MAX_NUM_OF_EXT_POOLS            4           /**< Number of external BM pools per Rx port */
#define FM_PORT_NUM_OF_CONGESTION_GRPS          256         /**< Total number of congestion groups in QM */
#define FM_MAX_NUM_OF_SUB_PORTALS               16
#define FM_PORT_MAX_NUM_OF_OBSERVED_EXT_POOLS   0

#define FM_VSP_MAX_NUM_OF_ENTRIES               32
#define FM_MAX_NUM_OF_PFC_PRIORITIES            8

/* RAMs defines */
#define FM_MURAM_SIZE                   (192 * KILOBYTE)
#define FM_IRAM_SIZE(major, minor)      \
    (((major == 6) && ((minor == 4) )) ? (64 * KILOBYTE) : (32 * KILOBYTE))
#define FM_NUM_OF_CTRL                  2

/* PCD defines */
#define FM_PCD_PLCR_NUM_ENTRIES         256                 /**< Total number of policer profiles */
#define FM_PCD_KG_NUM_OF_SCHEMES        32                  /**< Total number of KG schemes */
#define FM_PCD_MAX_NUM_OF_CLS_PLANS     256                 /**< Number of classification plan entries. */
#define FM_PCD_PRS_SW_PATCHES_SIZE      0x00000600          /**< Number of bytes saved for patches */
#define FM_PCD_SW_PRS_SIZE              0x00000800          /**< Total size of SW parser area */

/* RTC defines */
#define FM_RTC_NUM_OF_ALARMS            2                   /**< RTC number of alarms */
#define FM_RTC_NUM_OF_PERIODIC_PULSES   3                   /**< RTC number of periodic pulses */
#define FM_RTC_NUM_OF_EXT_TRIGGERS      2                   /**< RTC number of external triggers */

/* QMI defines */
#define QMI_MAX_NUM_OF_TNUMS            64
#define QMI_DEF_TNUMS_THRESH            32
/* FPM defines */
#define FM_NUM_OF_FMAN_CTRL_EVENT_REGS  4

/* DMA defines */
#define DMA_THRESH_MAX_COMMQ            83
#define DMA_THRESH_MAX_BUF              127

/* BMI defines */
#define BMI_MAX_NUM_OF_TASKS            64
#define BMI_MAX_NUM_OF_DMAS             32

#define BMI_MAX_FIFO_SIZE               (FM_MURAM_SIZE)
#define PORT_MAX_WEIGHT                 16

#define FM_CHECK_PORT_RESTRICTIONS(__validPorts, __newPortIndx)   TRUE

/* Unique T4240 */
#define FM_OP_OPEN_DMA_MIN_LIMIT
#define FM_NO_RESTRICT_ON_ACCESS_RSRC
#define FM_NO_OP_OBSERVED_POOLS
#define FM_FRAME_END_PARAMS_FOR_OP
#define FM_DEQ_PIPELINE_PARAMS_FOR_OP
#define FM_QMI_NO_SINGLE_ECC_EXCEPTION

#define FM_NO_GUARANTEED_RESET_VALUES

/* FM errata */
#define FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669
#define FM_RX_FIFO_CORRUPT_ERRATA_10GMAC_A006320
#define FM_OP_NO_VSP_NO_RELEASE_ERRATA_FMAN_A006675
#define FM_HEAVY_TRAFFIC_SEQUENCER_HANG_ERRATA_FMAN_A006981
#define FM_HANG_AT_RESET_MAC_CLK_DISABLED_ERRATA_FMAN_A007273

#define FM_BCB_ERRATA_BMI_SW001
#define FM_LEN_CHECK_ERRATA_FMAN_SW002
#define FM_AID_MODE_NO_TNUM_SW005 /* refer to pdm TKT068794 - only support of port_id on aid */
#define FM_ERROR_VSP_NO_MATCH_SW006 /* refer to pdm TKT174304 - no match between errorQ and VSP */

/*****************************************************************************
 RMan INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define RM_MAX_NUM_OF_IB        4           /**< Number of inbound blocks */
#define RM_NUM_OF_IBCU          8           /**< NUmber of classification units in an inbound block */

/* RMan erratas */
#define RM_ERRONEOUS_ACK_ERRATA_RMAN_A006756

/*****************************************************************************
 FM MACSEC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define NUM_OF_RX_SC                16
#define NUM_OF_TX_SC                16

#define NUM_OF_SA_PER_RX_SC         2
#define NUM_OF_SA_PER_TX_SC         2

#endif /* __DPAA_INTEGRATION_EXT_H */
