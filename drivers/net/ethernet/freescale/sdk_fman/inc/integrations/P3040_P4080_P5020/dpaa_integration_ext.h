/* Copyright (c) 2009-2012 Freescale Semiconductor, Inc
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
 @File          dpaa_integration_ext.h

 @Description   P3040/P4080/P5020 FM external definitions and structures.
*//***************************************************************************/
#ifndef __DPAA_INTEGRATION_EXT_H
#define __DPAA_INTEGRATION_EXT_H

#include "std_ext.h"


#define DPAA_VERSION    10

typedef enum {
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
    e_DPAA_SWPORTAL_DUMMY_LAST
} e_DpaaSwPortal;

typedef enum {
    e_DPAA_DCPORTAL0 = 0,
    e_DPAA_DCPORTAL1,
    e_DPAA_DCPORTAL2,
    e_DPAA_DCPORTAL3,
    e_DPAA_DCPORTAL4,
    e_DPAA_DCPORTAL_DUMMY_LAST
} e_DpaaDcPortal;

#define DPAA_MAX_NUM_OF_SW_PORTALS      e_DPAA_SWPORTAL_DUMMY_LAST
#define DPAA_MAX_NUM_OF_DC_PORTALS      e_DPAA_DCPORTAL_DUMMY_LAST

/*****************************************************************************
 QMan INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define QM_MAX_NUM_OF_POOL_CHANNELS 15              /**< Total number of channels, dedicated and pool */
#define QM_MAX_NUM_OF_WQ            8               /**< Number of work queues per channel */
#define QM_MAX_NUM_OF_SWP_AS        4
#define QM_MAX_NUM_OF_CGS           256             /**< Number of congestion groups */
#define QM_MAX_NUM_OF_FQIDS         (16 * MEGABYTE) /**< FQIDs range - 24 bits */

/**************************************************************************//**
 @Description   Work Queue Channel assignments in QMan.
*//***************************************************************************/
typedef enum
{
    e_QM_FQ_CHANNEL_SWPORTAL0 = 0,              /**< Dedicated channels serviced by software portals 0 to 9 */
    e_QM_FQ_CHANNEL_SWPORTAL1,
    e_QM_FQ_CHANNEL_SWPORTAL2,
    e_QM_FQ_CHANNEL_SWPORTAL3,
    e_QM_FQ_CHANNEL_SWPORTAL4,
    e_QM_FQ_CHANNEL_SWPORTAL5,
    e_QM_FQ_CHANNEL_SWPORTAL6,
    e_QM_FQ_CHANNEL_SWPORTAL7,
    e_QM_FQ_CHANNEL_SWPORTAL8,
    e_QM_FQ_CHANNEL_SWPORTAL9,

    e_QM_FQ_CHANNEL_POOL1 = 0x21,               /**< Pool channels that can be serviced by any of the software portals */
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

    e_QM_FQ_CHANNEL_FMAN0_SP0 = 0x40,           /**< Dedicated channels serviced by Direct Connect Portal 0:
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
/* difference between 5020 and 4080 :) */
    e_QM_FQ_CHANNEL_FMAN1_SP0 = 0x60,
    e_QM_FQ_CHANNEL_FMAN1_SP1,
    e_QM_FQ_CHANNEL_FMAN1_SP2,
    e_QM_FQ_CHANNEL_FMAN1_SP3,
    e_QM_FQ_CHANNEL_FMAN1_SP4,
    e_QM_FQ_CHANNEL_FMAN1_SP5,
    e_QM_FQ_CHANNEL_FMAN1_SP6,
    e_QM_FQ_CHANNEL_FMAN1_SP7,
    e_QM_FQ_CHANNEL_FMAN1_SP8,
    e_QM_FQ_CHANNEL_FMAN1_SP9,
    e_QM_FQ_CHANNEL_FMAN1_SP10,
    e_QM_FQ_CHANNEL_FMAN1_SP11,

    e_QM_FQ_CHANNEL_CAAM = 0x80,                /**< Dedicated channel serviced by Direct Connect Portal 2:
                                                     connected to SEC 4.x */

    e_QM_FQ_CHANNEL_PME = 0xA0,                 /**< Dedicated channel serviced by Direct Connect Portal 3:
                                                     connected to PME */
    e_QM_FQ_CHANNEL_RAID = 0xC0                 /**< Dedicated channel serviced by Direct Connect Portal 4:
                                                     connected to RAID */
} e_QmFQChannel;

/*****************************************************************************
 BMan INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define BM_MAX_NUM_OF_POOLS         64          /**< Number of buffers pools */


/*****************************************************************************
 FM INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define INTG_MAX_NUM_OF_FM          2

/* Ports defines */
#define FM_MAX_NUM_OF_1G_MACS       5
#define FM_MAX_NUM_OF_10G_MACS      1
#define FM_MAX_NUM_OF_MACS          (FM_MAX_NUM_OF_1G_MACS + FM_MAX_NUM_OF_10G_MACS)
#define FM_MAX_NUM_OF_OH_PORTS      7

#define FM_MAX_NUM_OF_1G_RX_PORTS   FM_MAX_NUM_OF_1G_MACS
#define FM_MAX_NUM_OF_10G_RX_PORTS  FM_MAX_NUM_OF_10G_MACS
#define FM_MAX_NUM_OF_RX_PORTS      (FM_MAX_NUM_OF_10G_RX_PORTS + FM_MAX_NUM_OF_1G_RX_PORTS)

#define FM_MAX_NUM_OF_1G_TX_PORTS   FM_MAX_NUM_OF_1G_MACS
#define FM_MAX_NUM_OF_10G_TX_PORTS  FM_MAX_NUM_OF_10G_MACS
#define FM_MAX_NUM_OF_TX_PORTS      (FM_MAX_NUM_OF_10G_TX_PORTS + FM_MAX_NUM_OF_1G_TX_PORTS)

#define FM_PORT_MAX_NUM_OF_EXT_POOLS            8           /**< Number of external BM pools per Rx port */
#define FM_PORT_NUM_OF_CONGESTION_GRPS          256         /**< Total number of congestion groups in QM */
#define FM_MAX_NUM_OF_SUB_PORTALS               12
#define FM_PORT_MAX_NUM_OF_OBSERVED_EXT_POOLS   0

/* Rams defines */
#define FM_MURAM_SIZE                   (160*KILOBYTE)
#define FM_IRAM_SIZE(major, minor)      (64 * KILOBYTE)
#define FM_NUM_OF_CTRL                  2

/* PCD defines */
#define FM_PCD_PLCR_NUM_ENTRIES         256             /**< Total number of policer profiles */
#define FM_PCD_KG_NUM_OF_SCHEMES        32              /**< Total number of KG schemes */
#define FM_PCD_MAX_NUM_OF_CLS_PLANS     256             /**< Number of classification plan entries. */
#define FM_PCD_PRS_SW_PATCHES_SIZE      0x00000200      /**< Number of bytes saved for patches */
#define FM_PCD_SW_PRS_SIZE              0x00000800      /**< Total size of SW parser area */

/* RTC defines */
#define FM_RTC_NUM_OF_ALARMS            2                   /**< RTC number of alarms */
#define FM_RTC_NUM_OF_PERIODIC_PULSES   2                   /**< RTC number of periodic pulses */
#define FM_RTC_NUM_OF_EXT_TRIGGERS      2                   /**< RTC number of external triggers */

/* QMI defines */
#define QMI_MAX_NUM_OF_TNUMS            64
#define QMI_DEF_TNUMS_THRESH            48

/* FPM defines */
#define FM_NUM_OF_FMAN_CTRL_EVENT_REGS  4

/* DMA defines */
#define DMA_THRESH_MAX_COMMQ            31
#define DMA_THRESH_MAX_BUF              127

/* BMI defines */
#define BMI_MAX_NUM_OF_TASKS            128
#define BMI_MAX_NUM_OF_DMAS             32
#define BMI_MAX_FIFO_SIZE               (FM_MURAM_SIZE)
#define PORT_MAX_WEIGHT                 16


#define FM_CHECK_PORT_RESTRICTIONS(__validPorts, __newPortIndx)   TRUE

/* p4080-rev1 unique features */
#define QM_CGS_NO_FRAME_MODE

/* p4080 unique features */
#define FM_NO_DISPATCH_RAM_ECC
#define FM_NO_WATCHDOG
#define FM_NO_TNUM_AGING
#define FM_KG_NO_BYPASS_FQID_GEN
#define FM_KG_NO_BYPASS_PLCR_PROFILE_GEN
#define FM_NO_BACKUP_POOLS
#define FM_NO_OP_OBSERVED_POOLS
#define FM_NO_ADVANCED_RATE_LIMITER
#define FM_NO_OP_OBSERVED_CGS
#define FM_HAS_TOTAL_DMAS
#define FM_KG_NO_IPPID_SUPPORT
#define FM_NO_GUARANTEED_RESET_VALUES
#define FM_MAC_RESET

/* FM erratas */
#define FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
#define FM_TX_SHORT_FRAME_BAD_TS_ERRATA_10GMAC_A006     /* No implementation, Out of LLD scope */
#define FM_TX_FIFO_CORRUPTION_ERRATA_10GMAC_A007
#define FM_ECC_HALT_NO_SYNC_ERRATA_10GMAC_A008
#define FM_TX_INVALID_ECC_ERRATA_10GMAC_A009            /* Out of LLD scope, user may disable ECC exceptions using FM_DisableRamsEcc */
#define FM_BAD_VLAN_DETECT_ERRATA_10GMAC_A010

#define FM_RX_PREAM_4_ERRATA_DTSEC_A001
#define FM_GRS_ERRATA_DTSEC_A002
#define FM_BAD_TX_TS_IN_B_2_B_ERRATA_DTSEC_A003
#define FM_GTS_ERRATA_DTSEC_A004
#define FM_GTS_AFTER_MAC_ABORTED_FRAME_ERRATA_DTSEC_A0012
#define FM_GTS_UNDERRUN_ERRATA_DTSEC_A0014
#define FM_GTS_AFTER_DROPPED_FRAME_ERRATA_DTSEC_A004839

#define FM_MAGIC_PACKET_UNRECOGNIZED_ERRATA_DTSEC2          /* No implementation, Out of LLD scope */
#define FM_TX_LOCKUP_ERRATA_DTSEC6

#define FM_HC_DEF_FQID_ONLY_ERRATA_FMAN_A003                /* Implemented by ucode */
#define FM_DEBUG_TRACE_FMAN_A004                            /* No implementation, Out of LLD scope */

#define FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173

#define FM_10G_REM_N_LCL_FLT_EX_10GMAC_ERRATA_SW005

#define FM_LEN_CHECK_ERRATA_FMAN_SW002

#define FM_NO_CTXA_COPY_ERRATA_FMAN_SW001
#define FM_KG_ERASE_FLOW_ID_ERRATA_FMAN_SW004

/*****************************************************************************
 FM MACSEC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define NUM_OF_RX_SC                16
#define NUM_OF_TX_SC                16

#define NUM_OF_SA_PER_RX_SC         2
#define NUM_OF_SA_PER_TX_SC         2


#endif /* __DPAA_INTEGRATION_EXT_H */
