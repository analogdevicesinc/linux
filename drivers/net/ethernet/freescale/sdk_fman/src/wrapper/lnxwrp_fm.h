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

/*
 @File          lnxwrp_fm.h

 @Author        Shlomi Gridish

 @Description   FM Linux wrapper functions.

*/

#ifndef __LNXWRP_FM_H__
#define __LNXWRP_FM_H__

#include <linux/fsl_qman.h> /* struct qman_fq */

#include "std_ext.h"
#include "error_ext.h"
#include "list_ext.h"

#include "lnxwrp_fm_ext.h"

#define FM_MAX_NUM_OF_ADV_SETTINGS          10

#define LNXWRP_FM_NUM_OF_SHARED_PROFILES    16

#if defined(CONFIG_FMAN_DISABLE_OH_TO_REUSE_RESOURCES)
#define FM_10G_OPENDMA_MIN_TRESHOLD 8 /* 10g minimum treshold if only HC is enabled and no OH port enabled */
#define FM_OPENDMA_RX_TX_RAPORT 2 /* RX = 2*TX */
#else
#define FM_10G_OPENDMA_MIN_TRESHOLD 7 /* 10g minimum treshold if 7 OH ports are enabled */
#define FM_OPENDMA_RX_TX_RAPORT 1 /* RX = TX */
#endif
#define FM_DEFAULT_TX10G_OPENDMA 8 /* default TX 10g open dmas */
#define FM_DEFAULT_RX10G_OPENDMA 8 /* default RX 10g open dmas */

#define FRAG_MANIP_SPACE 128
#define FRAG_DATA_ALIGN 64

#ifndef CONFIG_FSL_FM_MAX_FRAME_SIZE
#define CONFIG_FSL_FM_MAX_FRAME_SIZE 0
#endif

#ifndef CONFIG_FSL_FM_RX_EXTRA_HEADROOM
#define CONFIG_FSL_FM_RX_EXTRA_HEADROOM       16
#endif

typedef enum {
    e_NO_PCD = 0,
    e_FM_PCD_3_TUPLE
} e_LnxWrpFmPortPcdDefUseCase;


typedef struct t_FmTestFq {
    struct qman_fq      fq_base;
    t_Handle            h_Arg;
} t_FmTestFq;

typedef struct {
    uint8_t                     id; /* sw port id, see SW_PORT_ID_TO_HW_PORT_ID() in fm_common.h */
    int                         minor;
    char                        name[20];
    bool                        active;
    uint64_t                    phys_baseAddr;
    uint64_t                    baseAddr;               /* Port's *virtual* address */
    uint32_t                    memSize;
    t_WrpFmPortDevSettings      settings;
    t_FmExtPools                opExtPools;
    uint8_t                     totalNumOfSchemes;
    uint8_t                     schemesBase;
    uint8_t                     numOfSchemesUsed;
    uint32_t                    pcdBaseQ;
    uint16_t                    pcdNumOfQs;
    struct fm_port_pcd_param    pcd_owner_params;
    e_LnxWrpFmPortPcdDefUseCase defPcd;
    t_Handle                    h_DefNetEnv;
    t_Handle                    h_Schemes[FM_PCD_KG_NUM_OF_SCHEMES];
    t_FmBufferPrefixContent     buffPrefixContent;
    t_Handle                    h_Dev;
    t_Handle                    h_DfltVsp;
    t_Handle                    h_LnxWrpFmDev;
    uint16_t                    txCh;
    struct device               *dev;
    struct device_attribute     *dev_attr_stats;
    struct device_attribute     *dev_attr_regs;
    struct device_attribute     *dev_attr_bmi_regs;
    struct device_attribute     *dev_attr_qmi_regs;
#if (DPAA_VERSION >= 11)
    struct device_attribute     *dev_attr_ipv4_opt;
#endif
    struct device_attribute     *dev_attr_dsar_regs;
    struct device_attribute     *dev_attr_dsar_mem;
    struct auto_res_tables_sizes dsar_table_sizes;
} t_LnxWrpFmPortDev;

typedef struct {
    uint8_t                     id;
    bool                        active;
    uint64_t                    baseAddr;
    uint32_t                    memSize;
    t_WrpFmMacDevSettings       settings;
    t_Handle                    h_Dev;
    t_Handle                    h_LnxWrpFmDev;
} t_LnxWrpFmMacDev;

/* information about all active ports for an FMan.
 * !Some ports may be disabled by u-boot, thus will not be available */
struct fm_active_ports {
    uint32_t num_oh_ports;
    uint32_t num_tx_ports;
    uint32_t num_rx_ports;
    uint32_t num_tx25_ports;
    uint32_t num_rx25_ports;
    uint32_t num_tx10_ports;
    uint32_t num_rx10_ports;
};

/* FMan resources precalculated at fm probe based
 * on available FMan port. */
struct fm_resource_settings {
    /* buffers - fifo sizes */
    uint32_t tx1g_num_buffers;
    uint32_t rx1g_num_buffers;
    uint32_t tx2g5_num_buffers; /* Not supported yet by LLD */
    uint32_t rx2g5_num_buffers; /* Not supported yet by LLD */
    uint32_t tx10g_num_buffers;
    uint32_t rx10g_num_buffers;
    uint32_t oh_num_buffers;
    uint32_t shared_ext_buffers;

    /* open DMAs */
    uint32_t tx_1g_dmas;
    uint32_t rx_1g_dmas;
    uint32_t tx_2g5_dmas; /* Not supported yet by LLD */
    uint32_t rx_2g5_dmas; /* Not supported yet by LLD */
    uint32_t tx_10g_dmas;
    uint32_t rx_10g_dmas;
    uint32_t oh_dmas;
    uint32_t shared_ext_open_dma;

    /* Tnums */
    uint32_t tx_1g_tnums;
    uint32_t rx_1g_tnums;
    uint32_t tx_2g5_tnums; /* Not supported yet by LLD */
    uint32_t rx_2g5_tnums; /* Not supported yet by LLD */
    uint32_t tx_10g_tnums;
    uint32_t rx_10g_tnums;
    uint32_t oh_tnums;
    uint32_t shared_ext_tnums;
};

typedef struct {
    uint8_t                     id;
    char                        name[10];
    bool                        active;
    bool                        pcdActive;
    bool                        prsActive;
    bool                        kgActive;
    bool                        ccActive;
    bool                        plcrActive;
    e_LnxWrpFmPortPcdDefUseCase defPcd;
    uint32_t                    usedSchemes;
    uint8_t                     totalNumOfSharedSchemes;
    uint8_t                     sharedSchemesBase;
    uint8_t                     numOfSchemesUsed;
    uint8_t                     defNetEnvId;
    uint64_t                    fmPhysBaseAddr;
    uint64_t                    fmBaseAddr;
    uint32_t                    fmMemSize;
    uint64_t                    fmMuramPhysBaseAddr;
    uint64_t                    fmMuramBaseAddr;
    uint32_t                    fmMuramMemSize;
    uint64_t                    fmRtcPhysBaseAddr;
    uint64_t                    fmRtcBaseAddr;
    uint32_t                    fmRtcMemSize;
    uint64_t                    fmVspPhysBaseAddr;
    uint64_t                    fmVspBaseAddr;
    uint32_t                    fmVspMemSize;
    int                         irq;
    int                         err_irq;
    t_WrpFmDevSettings          fmDevSettings;
    t_WrpFmPcdDevSettings       fmPcdDevSettings;
    t_Handle                    h_Dev;
    uint16_t                    hcCh;

    t_Handle                    h_MuramDev;
    t_Handle                    h_PcdDev;
    t_Handle                    h_RtcDev;

    t_Handle			h_DsarRxPort;
    t_Handle			h_DsarTxPort;

    t_LnxWrpFmPortDev           hcPort;
    t_LnxWrpFmPortDev           opPorts[FM_MAX_NUM_OF_OH_PORTS-1];
    t_LnxWrpFmPortDev           rxPorts[FM_MAX_NUM_OF_RX_PORTS];
    t_LnxWrpFmPortDev           txPorts[FM_MAX_NUM_OF_TX_PORTS];
    t_LnxWrpFmMacDev            macs[FM_MAX_NUM_OF_MACS];
    struct fm_active_ports      fm_active_ports_info;
    struct fm_resource_settings fm_resource_settings_info;

    struct device               *dev;
    struct resource             *res;
    int                         major;
    struct class                *fm_class;
    struct device_attribute     *dev_attr_stats;
    struct device_attribute     *dev_attr_regs;
    struct device_attribute     *dev_attr_risc_load;

    struct device_attribute     *dev_pcd_attr_stats;
    struct device_attribute     *dev_plcr_attr_regs;
    struct device_attribute     *dev_prs_attr_regs;
    struct device_attribute     *dev_fm_fpm_attr_regs;
    struct device_attribute     *dev_fm_kg_attr_regs;
    struct device_attribute     *dev_fm_kg_pe_attr_regs;
    struct device_attribute     *dev_attr_muram_free_size;
    struct device_attribute     *dev_attr_fm_ctrl_code_ver;


    struct qman_fq              *hc_tx_conf_fq, *hc_tx_err_fq, *hc_tx_fq;
} t_LnxWrpFmDev;

typedef struct {
    t_LnxWrpFmDev   *p_FmDevs[INTG_MAX_NUM_OF_FM];
} t_LnxWrpFm;
#define LNXWRP_FM_OBJECT(ptr)   LIST_OBJECT(ptr, t_LnxWrpFm, fms[((t_LnxWrpFmDev *)ptr)->id])


t_Error  LnxwrpFmIOCTL(t_LnxWrpFmDev *p_LnxWrpFmDev, unsigned int cmd, unsigned long arg, bool compat);
t_Error  LnxwrpFmPortIOCTL(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev, unsigned int cmd, unsigned long arg, bool compat);


#if 0
static __inline__ t_Error AllocSchemesForPort(t_LnxWrpFmDev *p_LnxWrpFmDev, uint8_t numSchemes, uint8_t *p_BaseSchemeNum)
{
    uint32_t    schemeMask;
    uint8_t     i;

    if (!numSchemes)
        RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

    schemeMask = 0x80000000;
    *p_BaseSchemeNum = 0xff;

    for (i=0; schemeMask && numSchemes; schemeMask>>=1, i++)
        if ((p_LnxWrpFmDev->usedSchemes & schemeMask) == 0)
        {
            p_LnxWrpFmDev->usedSchemes |= schemeMask;
            numSchemes--;
            if (*p_BaseSchemeNum==0xff)
                *p_BaseSchemeNum = i;
        }
        else if (*p_BaseSchemeNum!=0xff)
            RETURN_ERROR(MINOR, E_INVALID_STATE, ("Fragmentation on schemes array!!!"));

    if (numSchemes)
        RETURN_ERROR(MINOR, E_FULL, ("schemes!!!"));
    return E_OK;
}
#endif

void LnxWrpPCDIOCTLTypeChecking(void);
void LnxWrpPCDIOCTLEnumChecking(void);

#endif /* __LNXWRP_FM_H__ */
