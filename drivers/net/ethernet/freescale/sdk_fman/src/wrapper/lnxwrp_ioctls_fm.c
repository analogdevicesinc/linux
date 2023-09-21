/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 * Copyright 2021 NXP
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
 @File          lnxwrp_ioctls_fm.c
 @Author        Shlomi Gridish
 @Description   FM Linux wrapper functions.
*/

/* Linux Headers ------------------- */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <asm/errno.h>
#ifndef CONFIG_FMAN_ARM
#include <sysdev/fsl_soc.h>
#include <linux/fsl/svr.h>
#endif

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "part_ext.h"
#include "fm_ioctls.h"
#include "fm_pcd_ioctls.h"
#include "fm_port_ioctls.h"
#include "fm_vsp_ext.h"

#ifndef CONFIG_FMAN_ARM
#define IS_T1023_T1024	(SVR_SOC_VER(mfspr(SPRN_SVR)) == SVR_T1024 || \
			SVR_SOC_VER(mfspr(SPRN_SVR)) == SVR_T1023)
#endif

#define __ERR_MODULE__  MODULE_FM

#if defined(CONFIG_COMPAT)
#include "lnxwrp_ioctls_fm_compat.h"
#endif

#include "lnxwrp_fm.h"

#define CMP_IOC_DEFINE(def) (IOC_##def != def)

/* fm_pcd_ioctls.h === fm_pcd_ext.h assertions */
#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_PRIVATE_HDRS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_NUM_OF_HDRS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_INTERCHANGEABLE_HDRS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_NUM_OF_GENERIC_REGS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_MAX_NUM_OF_EXTRACTS_PER_KEY)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_NUM_OF_EXTRACT_MASKS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_NUM_OF_DEFAULT_GROUPS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_NUM_OF_LABELS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_SW_PRS_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_MANIP_INSRT_TEMPLATE_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if DPAA_VERSION >= 11
#if CMP_IOC_DEFINE(FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES)
#error Error: please synchronize IOC_ defines!
#endif
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_TREES)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_GROUPS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_UNITS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_KEYS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_SIZE_OF_KEY)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_LAST_KEY_INDEX)
#error Error: please synchronize IOC_ defines!
#endif

/* net_ioctls.h === net_ext.h assertions */
#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPP_PID)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPP_COMPRESSED)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPPoE_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPPMUX_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPPMUX_SUBFRAME_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_ETH_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPv4_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPv6_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_ICMP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IGMP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_TCP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_SCTP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_DCCP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_UDP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_UDP_ENCAP_ESP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPHC_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_SCTP_CHUNK_DATA_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_L2TPv2_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_L2TPv3_CTRL_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_L2TPv3_SESS_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_VLAN_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_LLC_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_NLPID_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_SNAP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_LLC_SNAP_ALL_FIELDS)
#warning Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_ARP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_RFC2684_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_USER_DEFINED_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PAYLOAD_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_GRE_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_MINENCAP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPSEC_AH_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPSEC_ESP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_MPLS_LABEL_STACK_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_MACSEC_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

/* fm_ioctls.h === fm_ext.h assertions */
#if CMP_IOC_DEFINE(FM_MAX_NUM_OF_VALID_PORTS)
#error Error: please synchronize IOC_ defines!
#endif

void LnxWrpPCDIOCTLTypeChecking(void)
{
    /* fm_ext.h == fm_ioctls.h */
    ASSERT_COND(sizeof(ioc_fm_port_bandwidth_params) == sizeof(t_FmPortsBandwidthParams));
    ASSERT_COND(sizeof(ioc_fm_revision_info_t) == sizeof(t_FmRevisionInfo));

    /* fm_pcd_ext.h == fm_pcd_ioctls.h */
    /*ioc_fm_pcd_counters_params_t  : NOT USED */
    /*ioc_fm_pcd_exception_params_t : private */
#if (DPAA_VERSION >= 11)
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_capwap_params_t) == sizeof(t_FmPcdManipFragCapwapParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_capwap_params_t) == sizeof(t_FmPcdManipReassemCapwapParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_by_hdr_params_t) == sizeof(t_FmPcdManipHdrInsrtByHdrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_ip_params_t) == sizeof(t_FmPcdManipHdrInsrtIpParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_t) == sizeof(t_FmPcdManipHdrInsrt));
    ASSERT_COND(sizeof(ioc_fm_manip_hdr_info_t) == sizeof(t_FmManipHdrInfo));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_rmv_by_hdr_params_t) == sizeof(t_FmPcdManipHdrRmvByHdrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_special_offload_capwap_params_t) == sizeof(t_FmPcdManipSpecialOffloadCapwapParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_capwap_stats_t) == sizeof(t_FmPcdManipFragCapwapStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_capwap_stats_t) == sizeof(t_FmPcdManipReassemCapwapStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_params_t) == sizeof(t_FmPcdManipFragParams));
#endif /* (DPAA_VERSION >= 11) */

    ASSERT_COND(sizeof(ioc_fm_pcd_prs_label_params_t) == sizeof(t_FmPcdPrsLabelParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_prs_sw_params_t) == sizeof(t_FmPcdPrsSwParams));
    /*ioc_fm_pcd_kg_dflt_value_params_t : private */
    ASSERT_COND(sizeof(ioc_fm_pcd_hdr_protocol_opt_u) == sizeof(u_FmPcdHdrProtocolOpt));
    ASSERT_COND(sizeof(ioc_fm_pcd_fields_u) == sizeof(t_FmPcdFields));
    ASSERT_COND(sizeof(ioc_fm_pcd_from_hdr_t) == sizeof(t_FmPcdFromHdr));
    ASSERT_COND(sizeof(ioc_fm_pcd_from_field_t) == sizeof(t_FmPcdFromField));
    ASSERT_COND(sizeof(ioc_fm_pcd_distinction_unit_t) == sizeof(t_FmPcdDistinctionUnit));

#if defined(CONFIG_ARM64)
    /* different alignment */
    ASSERT_COND(sizeof(ioc_fm_pcd_net_env_params_t) == sizeof(t_FmPcdNetEnvParams) + sizeof(void *) + 4);
#else
#if !defined(CONFIG_COMPAT)
    /* different alignment */
    ASSERT_COND(sizeof(ioc_fm_pcd_net_env_params_t) == sizeof(t_FmPcdNetEnvParams) + sizeof(void *));
#endif
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_extract_entry_t) == sizeof(t_FmPcdExtractEntry));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_extract_mask_t) == sizeof(t_FmPcdKgExtractMask));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_extract_dflt_t) == sizeof(t_FmPcdKgExtractDflt));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_key_extract_and_hash_params_t) == sizeof(t_FmPcdKgKeyExtractAndHashParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_extracted_or_params_t) == sizeof(t_FmPcdKgExtractedOrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_scheme_counter_t) == sizeof(t_FmPcdKgSchemeCounter));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_plcr_profile_t) == sizeof(t_FmPcdKgPlcrProfile));
#if (DPAA_VERSION >= 11)
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_storage_profile_t) == sizeof(t_FmPcdKgStorageProfile));
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_cc_t) == sizeof(t_FmPcdKgCc));
#if !defined(CONFIG_COMPAT)
    /* different alignment */
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_scheme_params_t) == sizeof(t_FmPcdKgSchemeParams) + sizeof(void *));
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_cc_params_t) == sizeof(t_FmPcdCcNextCcParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_plcr_params_t) == sizeof(t_FmPcdCcNextPlcrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_enqueue_params_t) == sizeof(t_FmPcdCcNextEnqueueParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_kg_params_t) == sizeof(t_FmPcdCcNextKgParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_engine_params_t) == sizeof(t_FmPcdCcNextEngineParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_key_params_t) == sizeof(t_FmPcdCcKeyParams));
    ASSERT_COND(sizeof(ioc_keys_params_t) == sizeof(t_KeysParams));
#if !defined(CONFIG_COMPAT)
    /* different alignment */
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_node_params_t) == sizeof(t_FmPcdCcNodeParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_hash_table_params_t) == sizeof(t_FmPcdHashTableParams) + sizeof(void *));
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_grp_params_t) == sizeof(t_FmPcdCcGrpParams));
#if !defined(CONFIG_COMPAT)
    /* different alignment */
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_tree_params_t) == sizeof(t_FmPcdCcTreeParams) + sizeof(void *));
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_byte_rate_mode_param_t) == sizeof(t_FmPcdPlcrByteRateModeParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_non_passthrough_alg_param_t) == sizeof(t_FmPcdPlcrNonPassthroughAlgParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_next_engine_params_u) == sizeof(u_FmPcdPlcrNextEngineParams));
    /*ioc_fm_pcd_port_params_t : private */
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_profile_params_t) == sizeof(t_FmPcdPlcrProfileParams) + sizeof(void *));
    /*ioc_fm_pcd_cc_tree_modify_next_engine_params_t : private */

#ifdef FM_CAPWAP_SUPPORT
#error TODO: unsupported feature
/*
    ASSERT_COND(sizeof(TODO) == sizeof(t_FmPcdManipHdrInsrtByTemplateParams));
    ASSERT_COND(sizeof(TODO) == sizeof(t_CapwapFragmentationParams));
    ASSERT_COND(sizeof(TODO) == sizeof(t_CapwapReassemblyParams));
*/
#endif

    /*ioc_fm_pcd_cc_node_modify_next_engine_params_t : private */
    /*ioc_fm_pcd_cc_node_remove_key_params_t : private */
    /*ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t : private */
    /*ioc_fm_pcd_cc_node_modify_key_params_t : private */
    /*ioc_fm_manip_hdr_info_t : private */
    /*ioc_fm_pcd_hash_table_set_t : private */

    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_ip_params_t) == sizeof(t_FmPcdManipFragIpParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_ip_params_t) == sizeof(t_FmPcdManipReassemIpParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_special_offload_ipsec_params_t) == sizeof(t_FmPcdManipSpecialOffloadIPSecParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_special_offload_params_t) == sizeof(t_FmPcdManipSpecialOffloadParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_rmv_generic_params_t) == sizeof(t_FmPcdManipHdrRmvGenericParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_generic_params_t) == sizeof(t_FmPcdManipHdrInsrtGenericParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_params_t) == sizeof(t_FmPcdManipHdrInsrtParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_rmv_params_t) == sizeof(t_FmPcdManipHdrRmvParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_params_t) == sizeof(t_FmPcdManipHdrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_params_t) == sizeof(t_FmPcdManipFragParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_params_t) == sizeof(t_FmPcdManipReassemParams));
#if !defined(CONFIG_COMPAT)
    /* different alignment */
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_params_t) == sizeof(t_FmPcdManipParams) + sizeof(void *));
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_ip_stats_t) == sizeof(t_FmPcdManipReassemIpStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_ip_stats_t) == sizeof(t_FmPcdManipFragIpStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_stats_t) == sizeof(t_FmPcdManipReassemStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_stats_t) == sizeof(t_FmPcdManipFragStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_stats_t) == sizeof(t_FmPcdManipStats));
#if DPAA_VERSION >= 11
    ASSERT_COND(sizeof(ioc_fm_pcd_frm_replic_group_params_t) == sizeof(t_FmPcdFrmReplicGroupParams) + sizeof(void *));
#endif

    /* fm_port_ext.h == fm_port_ioctls.h */
    ASSERT_COND(sizeof(ioc_fm_port_rate_limit_t) == sizeof(t_FmPortRateLimit));
    ASSERT_COND(sizeof(ioc_fm_port_pcd_params_t) == sizeof(t_FmPortPcdParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_scheme_select_t) == sizeof(t_FmPcdKgSchemeSelect));
    ASSERT_COND(sizeof(ioc_fm_pcd_port_schemes_params_t) == sizeof(t_FmPcdPortSchemesParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_prs_start_t) == sizeof(t_FmPcdPrsStart));

    return;
}

#define ASSERT_IOC_NET_ENUM(def) ASSERT_COND((unsigned long)e_IOC_NET_##def == (unsigned long)def)

void LnxWrpPCDIOCTLEnumChecking(void)
{
    /* net_ext.h == net_ioctls.h : sampling checks */
    ASSERT_IOC_NET_ENUM(HEADER_TYPE_MACSEC);
    ASSERT_IOC_NET_ENUM(HEADER_TYPE_PPP);
    ASSERT_IOC_NET_ENUM(MAX_HEADER_TYPE_COUNT);

    /* fm_ext.h == fm_ioctls.h */
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_TYPE_DUMMY == (unsigned long)e_FM_PORT_TYPE_DUMMY);
    ASSERT_COND((unsigned long)e_IOC_EX_MURAM_ECC == (unsigned long)e_FM_EX_MURAM_ECC);
    ASSERT_COND((unsigned long)e_IOC_FM_COUNTERS_DEQ_CONFIRM == (unsigned long)e_FM_COUNTERS_DEQ_CONFIRM);

    /* fm_pcd_ext.h == fm_pcd_ioctls.h */
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES == (unsigned long)e_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PRS_EXCEPTION_SINGLE_ECC == (unsigned long)e_FM_PCD_PRS_EXCEPTION_SINGLE_ECC);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PRS == (unsigned long)e_FM_PCD_PRS);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_EXTRACT_FULL_FIELD == (unsigned long)e_FM_PCD_EXTRACT_FULL_FIELD);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_EXTRACT_FROM_FLOW_ID == (unsigned long)e_FM_PCD_EXTRACT_FROM_FLOW_ID);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_KG_EXTRACT_PORT_PRIVATE_INFO == (unsigned long)e_FM_PCD_KG_EXTRACT_PORT_PRIVATE_INFO);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_KG_DFLT_ILLEGAL == (unsigned long)e_FM_PCD_KG_DFLT_ILLEGAL);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_KG_GENERIC_NOT_FROM_DATA == (unsigned long)e_FM_PCD_KG_GENERIC_NOT_FROM_DATA);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_HDR_INDEX_LAST == (unsigned long)e_FM_PCD_HDR_INDEX_LAST);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_SHARED == (unsigned long)e_FM_PCD_PLCR_SHARED);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_RFC_4115 == (unsigned long)e_FM_PCD_PLCR_RFC_4115);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_COLOR_AWARE == (unsigned long)e_FM_PCD_PLCR_COLOR_AWARE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_OVERRIDE == (unsigned long)e_FM_PCD_PLCR_OVERRIDE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_FULL_FRM_LEN == (unsigned long)e_FM_PCD_PLCR_FULL_FRM_LEN);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_ROLLBACK_FULL_FRM_LEN == (unsigned long)e_FM_PCD_PLCR_ROLLBACK_FULL_FRM_LEN);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_PACKET_MODE == (unsigned long)e_FM_PCD_PLCR_PACKET_MODE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_DROP_FRAME == (unsigned long)e_FM_PCD_DROP_FRAME);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_PROFILE_RECOLOURED_RED_PACKET_TOTAL_COUNTER == (unsigned long)e_FM_PCD_PLCR_PROFILE_RECOLOURED_RED_PACKET_TOTAL_COUNTER);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_ACTION_INDEXED_LOOKUP == (unsigned long)e_FM_PCD_ACTION_INDEXED_LOOKUP);
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR == (unsigned long)e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR);
#if !defined(FM_CAPWAP_SUPPORT)
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_INSRT_GENERIC == (unsigned long)e_FM_PCD_MANIP_INSRT_GENERIC);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_RMV_GENERIC == (unsigned long)e_FM_PCD_MANIP_RMV_GENERIC);
#else
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_INSRT_BY_TEMPLATE == (unsigned long)e_FM_PCD_MANIP_INSRT_BY_TEMPLATE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_RMV_BY_HDR == (unsigned long)e_FM_PCD_MANIP_RMV_BY_HDR);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_RMV_BY_HDR_FROM_START == (unsigned long)e_FM_PCD_MANIP_RMV_BY_HDR_FROM_START);
#endif
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_TIME_OUT_BETWEEN_FRAG == (unsigned long)e_FM_PCD_MANIP_TIME_OUT_BETWEEN_FRAG);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_EIGHT_WAYS_HASH == (unsigned long)e_FM_PCD_MANIP_EIGHT_WAYS_HASH);

#ifdef FM_CAPWAP_SUPPORT
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_STATS_PER_FLOWID == (unsigned long)e_FM_PCD_STATS_PER_FLOWID);
#endif
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_SPECIAL_OFFLOAD == (unsigned long)e_FM_PCD_MANIP_SPECIAL_OFFLOAD);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_CC_STATS_MODE_FRAME == (unsigned long)e_FM_PCD_CC_STATS_MODE_FRAME);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_CONTINUE_WITHOUT_FRAG == (unsigned long)e_FM_PCD_MANIP_CONTINUE_WITHOUT_FRAG);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_SPECIAL_OFFLOAD_IPSEC == (unsigned long)e_FM_PCD_MANIP_SPECIAL_OFFLOAD_IPSEC);

    /* fm_port_ext.h == fm_port_ioctls.h */
#if !defined(FM_CAPWAP_SUPPORT)
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR == (unsigned long)e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR);
#else
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR == (unsigned long)e_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR);
#endif
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_COUNTERS_DEQ_CONFIRM == (unsigned long)e_FM_PORT_COUNTERS_DEQ_CONFIRM);
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8 == (unsigned long)e_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8);

    return;
}

static t_Error LnxwrpFmPcdIOCTL(t_LnxWrpFmDev *p_LnxWrpFmDev, unsigned int cmd, unsigned long arg, bool compat)
{
    t_Error err = E_OK;

/*
Status: PCD API to fmlib (file: drivers/net/dpa/NetCommSw/inc/Peripherals/fm_pcd_ext.h):

    FM_PCD_PrsLoadSw
    FM_PCD_SetAdvancedOffloadSupport
    FM_PCD_Enable
    FM_PCD_Disable
    FM_PCD_ForceIntr
    FM_PCD_AllowHcUsage
    FM_PCD_SetException
    FM_PCD_KgSetAdditionalDataAfterParsing
    FM_PCD_KgSetDfltValue
    FM_PCD_NetEnvCharacteristicsSet
    FM_PCD_NetEnvCharacteristicsDelete
    FM_PCD_KgSchemeSet
    FM_PCD_KgSchemeDelete
    FM_PCD_MatchTableSet
    FM_PCD_MatchTableDelete
    FM_PCD_CcRootBuild
    FM_PCD_CcRootDelete
    FM_PCD_PlcrProfileSet
    FM_PCD_PlcrProfileDelete
    FM_PCD_CcRootModifyNextEngine
    FM_PCD_MatchTableModifyNextEngine
    FM_PCD_MatchTableModifyMissNextEngine
    FM_PCD_MatchTableRemoveKey
    FM_PCD_MatchTableAddKey
    FM_PCD_MatchTableModifyKeyAndNextEngine
    FM_PCD_HashTableSet
    FM_PCD_HashTableDelete
    FM_PCD_HashTableAddKey
    FM_PCD_HashTableRemoveKey
    FM_PCD_MatchTableModifyKey
    FM_PCD_ManipNodeReplace
    FM_PCD_ManipNodeSet
    FM_PCD_ManipNodeDelete

Status: not exported, should be thru sysfs
    FM_PCD_KgSchemeGetCounter
    FM_PCD_KgSchemeSetCounter
    FM_PCD_PlcrProfileGetCounter
    FM_PCD_PlcrProfileSetCounter

Status: not exported
    FM_PCD_MatchTableFindNRemoveKey
    FM_PCD_MatchTableFindNModifyNextEngine
    FM_PCD_MatchTableFindNModifyKeyAndNextEngine
    FM_PCD_MatchTableFindNModifyKey
    FM_PCD_MatchTableGetIndexedHashBucket
    FM_PCD_MatchTableGetNextEngine
    FM_PCD_MatchTableGetKeyCounter

Status: not exported, would be nice to have
    FM_PCD_HashTableModifyNextEngine
    FM_PCD_HashTableModifyMissNextEngine
    FM_PCD_HashTableGetMissNextEngine
    FM_PCD_ManipGetStatistics

Status: not exported
#if DPAA_VERSION >= 11

    FM_VSP_GetStatistics -- it's not available yet
#endif

Status: feature not supported
#ifdef FM_CAPWAP_SUPPORT
#error unsupported feature
    FM_PCD_StatisticsSetNode
#endif

 */
    _fm_ioctl_dbg("cmd:0x%08x(type:0x%02x, nr:%u).\n",
            cmd, _IOC_TYPE(cmd), _IOC_NR(cmd) - 20);

    switch (cmd)
    {
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_PRS_LOAD_SW_COMPAT:
#endif
        case FM_PCD_IOC_PRS_LOAD_SW:
        {
            ioc_fm_pcd_prs_sw_params_t *param;
            uint8_t                    *p_code;

            param = (ioc_fm_pcd_prs_sw_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_prs_sw_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_prs_sw_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_prs_sw_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_prs_sw_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_prs_sw_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_prs_sw_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_prs_sw_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_prs_sw_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_fm_pcd_prs_sw(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_prs_sw_params_t *)arg,
                            sizeof(ioc_fm_pcd_prs_sw_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (!param->p_code || !param->size)
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            p_code = (uint8_t *) XX_Malloc(param->size);
            if (!p_code)
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
            }

            memset(p_code, 0, param->size);
            if (copy_from_user(p_code, param->p_code, param->size))
            {
                XX_Free(p_code);
                XX_Free(param);
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            param->p_code = p_code;

            err = FM_PCD_PrsLoadSw(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdPrsSwParams*)param);

            XX_Free(p_code);
            XX_Free(param);
            break;
        }

        case FM_PCD_IOC_SET_ADVANCED_OFFLOAD_SUPPORT:
            err = FM_PCD_SetAdvancedOffloadSupport(p_LnxWrpFmDev->h_PcdDev);
            break;

        case FM_PCD_IOC_ENABLE:
            err = FM_PCD_Enable(p_LnxWrpFmDev->h_PcdDev);
            break;

        case FM_PCD_IOC_DISABLE:
            err = FM_PCD_Disable(p_LnxWrpFmDev->h_PcdDev);
            break;

        case FM_PCD_IOC_FORCE_INTR:
        {
            int exception;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(exception, (int *) compat_ptr(arg)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }
            else
#endif
            {
                if (get_user(exception, (int *)arg))
                   RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_ForceIntr(p_LnxWrpFmDev->h_PcdDev, (e_FmPcdExceptions)exception);
            break;
        }

        case FM_PCD_IOC_ALLOW_HC_USAGE:
        {
        	uint8_t allow_hc;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(allow_hc, (uint8_t *) compat_ptr(arg)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }
            else
#endif
            {
                if (get_user(allow_hc, (uint8_t *)arg))
                   RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_AllowHcUsage(p_LnxWrpFmDev->h_PcdDev, allow_hc);
        	break;
        }

        case FM_PCD_IOC_SET_EXCEPTION:
        {
            ioc_fm_pcd_exception_params_t *param;

            param = (ioc_fm_pcd_exception_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_exception_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_exception_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_pcd_exception_params_t *)compat_ptr(arg),
                                    sizeof(ioc_fm_pcd_exception_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_exception_params_t *)arg,
                                    sizeof(ioc_fm_pcd_exception_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

			err = FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
					(e_FmPcdExceptions)param->exception,
					param->enable);

            XX_Free(param);
            break;
        }

        case FM_PCD_IOC_KG_SET_ADDITIONAL_DATA_AFTER_PARSING:
        {
            uint8_t payloadOffset;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(payloadOffset, (uint8_t*) compat_ptr(arg)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }
            else
#endif
            {
                if (get_user(payloadOffset, (uint8_t*) arg))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_KgSetAdditionalDataAfterParsing(p_LnxWrpFmDev->h_PcdDev, payloadOffset);
            break;
        }

        case FM_PCD_IOC_KG_SET_DFLT_VALUE:
        {
            ioc_fm_pcd_kg_dflt_value_params_t *param;

            param = (ioc_fm_pcd_kg_dflt_value_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_kg_dflt_value_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_dflt_value_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_dflt_value_params_t *)compat_ptr(arg),
                                    sizeof(ioc_fm_pcd_kg_dflt_value_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_dflt_value_params_t *)arg,
                                    sizeof(ioc_fm_pcd_kg_dflt_value_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PCD_KgSetDfltValue(p_LnxWrpFmDev->h_PcdDev, param->valueId, param->value);

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_NET_ENV_CHARACTERISTICS_SET_COMPAT:
#endif
        case FM_PCD_IOC_NET_ENV_CHARACTERISTICS_SET:
        {
            ioc_fm_pcd_net_env_params_t  *param;

            param = (ioc_fm_pcd_net_env_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_net_env_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_net_env_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_net_env_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_net_env_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_net_env_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_net_env_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_net_env_params_t *) compat_ptr(arg),
                                    sizeof(ioc_compat_fm_pcd_net_env_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_net_env(compat_param, param, COMPAT_US_TO_K);
                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_net_env_params_t *) arg,
                            sizeof(ioc_fm_pcd_net_env_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            param->id = FM_PCD_NetEnvCharacteristicsSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdNetEnvParams*)param);

            if (!param->id)
            {
                XX_Free(param);
                err = E_INVALID_VALUE;
                /* Since the LLD has no errno-style error reporting,
                   we're left here with no other option than to report
                   a generic E_INVALID_VALUE */
                break;
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_net_env_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_net_env_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_net_env_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_net_env_params_t));
                compat_copy_fm_pcd_net_env(compat_param, param, COMPAT_K_TO_US);

                if (copy_to_user((ioc_compat_fm_pcd_net_env_params_t *) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_net_env_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_net_env_params_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_net_env_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_NET_ENV_CHARACTERISTICS_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_NET_ENV_CHARACTERISTICS_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_obj_delete(&compat_id, &id);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_NetEnvCharacteristicsDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_KG_SCHEME_SET_COMPAT:
#endif
        case FM_PCD_IOC_KG_SCHEME_SET:
        {
            ioc_fm_pcd_kg_scheme_params_t *param;

            param = (ioc_fm_pcd_kg_scheme_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_kg_scheme_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_scheme_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_params_t *compat_param = NULL;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));

                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_kg_scheme_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_kg_scheme_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_kg_scheme(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_scheme_params_t *)arg,
                            sizeof(ioc_fm_pcd_kg_scheme_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            param->id = FM_PCD_KgSchemeSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdKgSchemeParams*)param);

            if (!param->id)
            {
                XX_Free(param);
                err = E_INVALID_VALUE;
                /* Since the LLD has no errno-style error reporting,
                   we're left here with no other option than to report
                   a generic E_INVALID_VALUE */
                break;
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));
                compat_copy_fm_pcd_kg_scheme(compat_param, param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_kg_scheme_params_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_kg_scheme_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_kg_scheme_params_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_kg_scheme_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_KG_SCHEME_GET_CNTR_COMPAT:
#endif
        case FM_PCD_IOC_KG_SCHEME_GET_CNTR:
        {
            ioc_fm_pcd_kg_scheme_spc_t *param;

            param = (ioc_fm_pcd_kg_scheme_spc_t *) XX_Malloc(sizeof(ioc_fm_pcd_kg_scheme_spc_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_scheme_spc_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_spc_t *compat_param = NULL;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_spc_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_spc_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_spc_t));

                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_kg_scheme_spc_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_kg_scheme_spc_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_kg_scheme_spc(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_scheme_spc_t *)arg,
                            sizeof(ioc_fm_pcd_kg_scheme_spc_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            param->val = FM_PCD_KgSchemeGetCounter((t_Handle)param->id);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_spc_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_spc_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_spc_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_spc_t));
                compat_copy_fm_pcd_kg_scheme_spc(compat_param, param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_kg_scheme_spc_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_kg_scheme_spc_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_kg_scheme_spc_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_kg_scheme_spc_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_KG_SCHEME_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_KG_SCHEME_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_obj_delete(&compat_id, &id);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_KgSchemeDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_SET_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_SET:
        {
            ioc_fm_pcd_cc_node_params_t *param;
            uint8_t                     *keys;
            uint8_t                     *masks;
            int                         i,k;

            param = (ioc_fm_pcd_cc_node_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_params_t) +
                    2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_params_t) +
                    2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);

            keys = (uint8_t *) (param + 1);
            masks = keys + IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_params_t *) XX_Malloc(
                                    sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                                    2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                        2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);

                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_params_t *)arg, sizeof(ioc_fm_pcd_cc_node_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            ASSERT_COND(param->keys_params.num_of_keys <= IOC_FM_PCD_MAX_NUM_OF_KEYS);
            ASSERT_COND(param->keys_params.key_size <= IOC_FM_PCD_MAX_SIZE_OF_KEY);

            /* support for indexed lookup */
            if( !(param->extract_cc_params.type == e_IOC_FM_PCD_EXTRACT_NON_HDR &&
                  param->extract_cc_params.extract_params.extract_non_hdr.src == e_IOC_FM_PCD_EXTRACT_FROM_HASH &&
                  param->extract_cc_params.extract_params.extract_non_hdr.action == e_IOC_FM_PCD_ACTION_INDEXED_LOOKUP))
            {
                for (i=0, k=0;
                     i < param->keys_params.num_of_keys;
                     i++, k += IOC_FM_PCD_MAX_SIZE_OF_KEY)
                {
                    if (param->keys_params.key_params[i].p_key &&
                            param->keys_params.key_size)
                    {
                        if (copy_from_user(&keys[k],
                                    param->keys_params.key_params[i].p_key,
                                    param->keys_params.key_size))
                        {
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->keys_params.key_params[i].p_key = &keys[k];
                    }

                    if (param->keys_params.key_params[i].p_mask)
                    {
                        if (copy_from_user(&masks[k],
                                    param->keys_params.key_params[i].p_mask,
                                    param->keys_params.key_size))
                        {
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->keys_params.key_params[i].p_mask = &masks[k];
                    }
                }
            }

            param->id = FM_PCD_MatchTableSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdCcNodeParams*)param);

            if (!param->id) {
                XX_Free(param);
                err = E_INVALID_VALUE;
                /* Since the LLD has no errno-style error reporting,
                   we're left here with no other option than to report
                   a generic E_INVALID_VALUE */
                break;
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_params_t *compat_param;
                compat_param = (ioc_compat_fm_pcd_cc_node_params_t *) XX_Malloc(
                                            sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                                            2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                        2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
                compat_copy_fm_pcd_cc_node(compat_param, param, COMPAT_K_TO_US);

                if (copy_to_user((ioc_compat_fm_pcd_cc_node_params_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_node_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_cc_node_params_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_cc_node_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_obj_delete(&compat_id, &id);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_MatchTableDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_ROOT_BUILD_COMPAT:
#endif
        case FM_PCD_IOC_CC_ROOT_BUILD:
        {
            ioc_fm_pcd_cc_tree_params_t *param;

            param = (ioc_fm_pcd_cc_tree_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_cc_tree_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_tree_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tree_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tree_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_tree_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tree_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_tree(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_tree_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tree_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            param->id = FM_PCD_CcRootBuild(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdCcTreeParams*)param);

            if (!param->id) {
                XX_Free(param);
                err = E_INVALID_VALUE;
                /* Since the LLD has no errno-style error reporting,
                   we're left here with no other option than to report
                   a generic E_INVALID_VALUE */
                break;
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tree_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tree_params_t *) XX_Malloc(sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tree_params_t));

                compat_copy_fm_pcd_cc_tree(compat_param, param, COMPAT_K_TO_US);

                if (copy_to_user((ioc_compat_fm_pcd_cc_tree_params_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_tree_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_cc_tree_params_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_cc_tree_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_ROOT_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_CC_ROOT_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_obj_delete(&compat_id, &id);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_CcRootDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_PLCR_PROFILE_SET_COMPAT:
#endif
        case FM_PCD_IOC_PLCR_PROFILE_SET:
        {
            ioc_fm_pcd_plcr_profile_params_t *param;

            param = (ioc_fm_pcd_plcr_profile_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_plcr_profile_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_plcr_profile_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_plcr_profile_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_plcr_profile_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                if (copy_from_user(compat_param, (
                            ioc_compat_fm_pcd_plcr_profile_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_plcr_profile_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_plcr_profile(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_plcr_profile_params_t *)arg,
                                    sizeof(ioc_fm_pcd_plcr_profile_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (!param->modify &&
                (((t_FmPcdPlcrProfileParams*)param)->id.newParams.profileType != e_FM_PCD_PLCR_SHARED))
            {
                t_Handle h_Port;
                ioc_fm_pcd_port_params_t *port_params;

                port_params = (ioc_fm_pcd_port_params_t*) XX_Malloc(sizeof(ioc_fm_pcd_port_params_t));
                if (!port_params)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(port_params, 0, sizeof(ioc_fm_pcd_port_params_t));
                if (copy_from_user(port_params, (ioc_fm_pcd_port_params_t*)((t_FmPcdPlcrProfileParams*)param)->id.newParams.h_FmPort,
                            sizeof(ioc_fm_pcd_port_params_t)))
                {
                    XX_Free(port_params);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                switch(port_params->port_type)
                {
                    case (e_IOC_FM_PORT_TYPE_RX):
                        if (port_params->port_id < FM_MAX_NUM_OF_1G_RX_PORTS) {
                            h_Port = p_LnxWrpFmDev->rxPorts[port_params->port_id].h_Dev;
                            break;
                        }
                        goto invalid_port_id;

                    case (e_IOC_FM_PORT_TYPE_RX_10G):
                        if (port_params->port_id < FM_MAX_NUM_OF_10G_RX_PORTS) {
#ifndef CONFIG_FMAN_ARM
                            if (IS_T1023_T1024) {
                                h_Port = p_LnxWrpFmDev->rxPorts[port_params->port_id].h_Dev;
                            } else {
#else
                            {
#endif
                                h_Port = p_LnxWrpFmDev->rxPorts[port_params->port_id + FM_MAX_NUM_OF_1G_RX_PORTS].h_Dev;
                            }
                            break;
                        }
                        goto invalid_port_id;

                    case (e_IOC_FM_PORT_TYPE_OH_OFFLINE_PARSING):
                        if (port_params->port_id && port_params->port_id < FM_MAX_NUM_OF_OH_PORTS) {
                            h_Port = p_LnxWrpFmDev->opPorts[port_params->port_id - 1].h_Dev;
                            break;
                        }
                        goto invalid_port_id;

                    default:
invalid_port_id:
                        XX_Free(port_params);
                        XX_Free(param);
                        RETURN_ERROR(MINOR, E_INVALID_SELECTION, NO_MSG);
                }

                ((t_FmPcdPlcrProfileParams*)param)->id.newParams.h_FmPort = h_Port;
                XX_Free(port_params);
            }

            param->id = FM_PCD_PlcrProfileSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdPlcrProfileParams*)param);

            if (!param->id) {
                XX_Free(param);
                err = E_INVALID_VALUE;
                /* Since the LLD has no errno-style error reporting,
                   we're left here with no other option than to report
                   a generic E_INVALID_VALUE */
                break;
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_plcr_profile_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_plcr_profile_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                compat_copy_fm_pcd_plcr_profile(compat_param, param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_plcr_profile_params_t *) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_plcr_profile_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_plcr_profile_params_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_plcr_profile_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_PLCR_PROFILE_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_PLCR_PROFILE_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_obj_delete(&compat_id, &id);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_PlcrProfileDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_ROOT_MODIFY_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_CC_ROOT_MODIFY_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_tree_modify_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_tree_modify_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_tree_modify_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_tree_modify_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_fm_pcd_cc_tree_modify_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_tree_modify_next_engine_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tree_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PCD_CcRootModifyNextEngine(param->id,
                                                param->grp_indx,
                                                param->indx,
                                                (t_FmPcdCcNextEngineParams*)(&param->cc_next_engine_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_node_modify_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_next_engine_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableModifyNextEngine(param->id,
                    param->key_indx,
                    (t_FmPcdCcNextEngineParams*)(&param->cc_next_engine_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_MISS_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_MISS_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_node_modify_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) compat_ptr(arg),
                                    sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_next_engine_params_t *) arg,
                                    sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableModifyMissNextEngine(param->id,
                    (t_FmPcdCcNextEngineParams*)(&param->cc_next_engine_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_REMOVE_KEY_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_REMOVE_KEY:
        {
            ioc_fm_pcd_cc_node_remove_key_params_t *param;

            param = (ioc_fm_pcd_cc_node_remove_key_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_remove_key_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_remove_key_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_remove_key_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_remove_key_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_remove_key_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_remove_key_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_remove_key_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_remove_key_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                param->id = compat_ptr(compat_param->id);
                param->key_indx = compat_param->key_indx;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_remove_key_params_t *) arg,
                            sizeof(ioc_fm_pcd_cc_node_remove_key_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableRemoveKey(param->id, param->key_indx);

            XX_Free(param);
            break;
        }
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_ADD_KEY_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_ADD_KEY:
        {
            ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_key_and_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)arg,
                                    sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (param->key_size)
            {
                int size = 0;

                if (param->key_params.p_key)  size += param->key_size;
                if (param->key_params.p_mask) size += param->key_size;

                if (size)
                {
                    uint8_t *p_tmp;

                    p_tmp = (uint8_t*) XX_Malloc(size);
                    if (!p_tmp)
                    {
                        XX_Free(param);
                        RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD key/mask"));
                    }

                    if (param->key_params.p_key)
                    {
                        if (copy_from_user(p_tmp, param->key_params.p_key, param->key_size))
                        {
                            XX_Free(p_tmp);
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->key_params.p_key = p_tmp;
                    }

                    if (param->key_params.p_mask)
                    {
                        p_tmp += param->key_size;
                        if (copy_from_user(p_tmp, param->key_params.p_mask, param->key_size))
                        {
                            XX_Free(p_tmp - param->key_size);
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->key_params.p_mask = p_tmp;
                    }
                }
            }

            err = FM_PCD_MatchTableAddKey(
                    param->id,
                    param->key_indx,
                    param->key_size,
                    (t_FmPcdCcKeyParams*)&param->key_params);

            if (param->key_params.p_key)
                XX_Free(param->key_params.p_key);
            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_KEY_AND_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_KEY_AND_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_key_and_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableModifyKeyAndNextEngine(param->id,
                    param->key_indx,
                    param->key_size,
                    (t_FmPcdCcKeyParams*)(&param->key_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_GET_KEY_STAT_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_GET_KEY_STAT:
        {
            ioc_fm_pcd_cc_tbl_get_stats_t param;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tbl_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tbl_get_stats_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_tbl_get_stats_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t)))
                {
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_tbl_get_stats(compat_param, &param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(&param, (ioc_fm_pcd_cc_tbl_get_stats_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tbl_get_stats_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_MatchTableGetKeyStatistics((t_Handle) param.id,
                                                     param.key_index,
                                                     (t_FmPcdCcKeyStatistics *) &param.statistics);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tbl_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tbl_get_stats_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                compat_copy_fm_pcd_cc_tbl_get_stats(compat_param, &param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_cc_tbl_get_stats_t*) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t))){
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
                }
                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_cc_tbl_get_stats_t *)arg,
                                  &param,
                                  sizeof(ioc_fm_pcd_cc_tbl_get_stats_t)))
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
            }

            break;
        }


#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_GET_MISS_STAT_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_GET_MISS_STAT:
        {
            ioc_fm_pcd_cc_tbl_get_stats_t param;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tbl_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tbl_get_stats_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_tbl_get_stats_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t)))
                {
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_tbl_get_stats(compat_param, &param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(&param, (ioc_fm_pcd_cc_tbl_get_stats_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tbl_get_stats_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_MatchTableGetMissStatistics((t_Handle) param.id,
                                                     (t_FmPcdCcKeyStatistics *) &param.statistics);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tbl_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tbl_get_stats_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                compat_copy_fm_pcd_cc_tbl_get_stats(compat_param, &param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_cc_tbl_get_stats_t*) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t))){
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
                }
                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_cc_tbl_get_stats_t *)arg,
                                  &param,
                                  sizeof(ioc_fm_pcd_cc_tbl_get_stats_t)))
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
            }

            break;
        }


#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_HASH_TABLE_GET_MISS_STAT_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_GET_MISS_STAT:
        {
            ioc_fm_pcd_cc_tbl_get_stats_t param;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tbl_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tbl_get_stats_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_tbl_get_stats_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t)))
                {
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_tbl_get_stats(compat_param, &param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(&param, (ioc_fm_pcd_cc_tbl_get_stats_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tbl_get_stats_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_HashTableGetMissStatistics((t_Handle) param.id,
                                                     (t_FmPcdCcKeyStatistics *) &param.statistics);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tbl_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tbl_get_stats_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t));
                compat_copy_fm_pcd_cc_tbl_get_stats(compat_param, &param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_cc_tbl_get_stats_t*) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_tbl_get_stats_t))){
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
                }
                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_cc_tbl_get_stats_t *)arg,
                                  &param,
                                  sizeof(ioc_fm_pcd_cc_tbl_get_stats_t)))
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
            }

            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_HASH_TABLE_SET_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_SET:
        {
            ioc_fm_pcd_hash_table_params_t *param;

            param = (ioc_fm_pcd_hash_table_params_t*) XX_Malloc(
                    sizeof(ioc_fm_pcd_hash_table_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_hash_table_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_hash_table_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_hash_table_params_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_hash_table_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_hash_table_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_hash_table_params_t*)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_hash_table_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_hash_table(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_hash_table_params_t *)arg,
                                    sizeof(ioc_fm_pcd_hash_table_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            param->id = FM_PCD_HashTableSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdHashTableParams *) param);

            if (!param->id)
            {
                XX_Free(param);
                err = E_INVALID_VALUE;
                /* Since the LLD has no errno-style error reporting,
                   we're left here with no other option than to report
                   a generic E_INVALID_VALUE */
                break;
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_hash_table_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_hash_table_params_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_hash_table_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_hash_table_params_t));
                compat_copy_fm_pcd_hash_table(compat_param, param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_hash_table_params_t*) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_hash_table_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
			if (copy_to_user((ioc_fm_pcd_hash_table_params_t *)arg,
					 param,
					 sizeof(ioc_fm_pcd_hash_table_params_t))) {
				FM_PCD_HashTableDelete(param->id);
				err = E_READ_FAILED;
			}
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_HASH_TABLE_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0, sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                id.obj = compat_pcd_id2ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_HashTableDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_HASH_TABLE_ADD_KEY_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_ADD_KEY:
        {
            ioc_fm_pcd_hash_table_add_key_params_t *param = NULL;

            param = (ioc_fm_pcd_hash_table_add_key_params_t*) XX_Malloc(
                    sizeof(ioc_fm_pcd_hash_table_add_key_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_hash_table_add_key_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_hash_table_add_key_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_hash_table_add_key_params_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_hash_table_add_key_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_hash_table_add_key_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_hash_table_add_key_params_t*) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_hash_table_add_key_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                if (compat_param->key_size)
                {
                    param->p_hash_tbl = compat_pcd_id2ptr(compat_param->p_hash_tbl);
                    param->key_size   = compat_param->key_size;

                    compat_copy_fm_pcd_cc_key(&compat_param->key_params, &param->key_params, COMPAT_US_TO_K);
                }
                else
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    err = E_INVALID_VALUE;
                    break;
                }

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_hash_table_add_key_params_t*) arg,
                            sizeof(ioc_fm_pcd_hash_table_add_key_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (param->key_size)
            {
                int size = 0;

                if (param->key_params.p_key)  size += param->key_size;
                if (param->key_params.p_mask) size += param->key_size;

                if (size)
                {
                    uint8_t *p_tmp;

                    p_tmp = (uint8_t*) XX_Malloc(size);
                    if (!p_tmp)
                    {
                        XX_Free(param);
                        RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD key/mask"));
                    }

                    if (param->key_params.p_key)
                    {
                        if (copy_from_user(p_tmp, param->key_params.p_key, param->key_size))
                        {
                            XX_Free(p_tmp);
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->key_params.p_key = p_tmp;
                    }

                    if (param->key_params.p_mask)
                    {
                        p_tmp += param->key_size;
                        if (copy_from_user(p_tmp, param->key_params.p_mask, param->key_size))
                        {
                            XX_Free(p_tmp - param->key_size);
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->key_params.p_mask = p_tmp;
                    }
                }
            }

            err = FM_PCD_HashTableAddKey(
                    param->p_hash_tbl,
                    param->key_size,
                    (t_FmPcdCcKeyParams*)&param->key_params);

            if (param->key_params.p_key)
                XX_Free(param->key_params.p_key);
            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_HASH_TABLE_REMOVE_KEY_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_REMOVE_KEY:
        {
            ioc_fm_pcd_hash_table_remove_key_params_t *param = NULL;

            param = (ioc_fm_pcd_hash_table_remove_key_params_t*) XX_Malloc(
                    sizeof(ioc_fm_pcd_hash_table_remove_key_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_hash_table_remove_key_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_hash_table_remove_key_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_hash_table_remove_key_params_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_hash_table_remove_key_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_hash_table_remove_key_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_hash_table_remove_key_params_t*) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_hash_table_remove_key_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                param->p_hash_tbl = compat_pcd_id2ptr(compat_param->p_hash_tbl);
                param->key_size   = compat_param->key_size;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_hash_table_remove_key_params_t*)arg,
                            sizeof(ioc_fm_pcd_hash_table_remove_key_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (param->key_size)
            {
                uint8_t *p_key;

                p_key = (uint8_t*) XX_Malloc(param->key_size);
                if (!p_key)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                if (param->p_key && copy_from_user(p_key, param->p_key, param->key_size))
                {
                    XX_Free(p_key);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
                param->p_key = p_key;
            }

            err = FM_PCD_HashTableRemoveKey(
                    param->p_hash_tbl,
                    param->key_size,
                    param->p_key);

            if (param->p_key)
                XX_Free(param->p_key);
            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_KEY_COMPAT:
#endif
        case FM_PCD_IOC_MATCH_TABLE_MODIFY_KEY:
        {
            ioc_fm_pcd_cc_node_modify_key_params_t  *param;

            param = (ioc_fm_pcd_cc_node_modify_key_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_key_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_key_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_key_params_t  *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_key_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_key_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_key_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_node_modify_key_params_t *)compat_ptr(arg),
                                    sizeof(ioc_compat_fm_pcd_cc_node_modify_key_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_key(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_key_params_t *)arg,
                                    sizeof(ioc_fm_pcd_cc_node_modify_key_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (param->key_size)
            {
                int size = 0;

                if (param->p_key)  size += param->key_size;
                if (param->p_mask) size += param->key_size;

                if (size)
                {
                    uint8_t *p_tmp;

                    p_tmp = (uint8_t*) XX_Malloc(size);
                    if (!p_tmp)
                    {
                        XX_Free(param);
                        RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD key/mask"));
                    }

                    if (param->p_key)
                    {
                        if (copy_from_user(p_tmp, param->p_key, param->key_size))
                        {
                            XX_Free(p_tmp);
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->p_key = p_tmp;
                    }

                    if (param->p_mask)
                    {
                        p_tmp += param->key_size;
                        if (copy_from_user(p_tmp, param->p_mask, param->key_size))
                        {
                            XX_Free(p_tmp - param->key_size);
                            XX_Free(param);
                            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                        }

                        param->p_mask = p_tmp;
                    }
                }
            }

            err = FM_PCD_MatchTableModifyKey(param->id,
                    param->key_indx,
                    param->key_size,
                    param->p_key,
                    param->p_mask);

            if (param->p_key)
                XX_Free(param->p_key);
            else if (param->p_mask)
                XX_Free(param->p_mask);
            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MANIP_NODE_SET_COMPAT:
#endif
        case FM_PCD_IOC_MANIP_NODE_SET:
        {
            ioc_fm_pcd_manip_params_t *param;
            uint8_t *p_data = NULL;
            uint8_t size;

            param = (ioc_fm_pcd_manip_params_t *) XX_Malloc(
                        sizeof(ioc_fm_pcd_manip_params_t));

            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_manip_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_manip_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_manip_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_manip_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_manip_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_manip_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_manip_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_fm_pcd_manip_set_node(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_manip_params_t *)arg,
                                            sizeof(ioc_fm_pcd_manip_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (param->type == e_IOC_FM_PCD_MANIP_HDR)
            {
                size = param->u.hdr.insrt_params.u.generic.size;
                p_data = (uint8_t *) XX_Malloc(size);
                if (!p_data )
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, NO_MSG);
                }

                if (param->u.hdr.insrt_params.u.generic.p_data &&
                        copy_from_user(p_data,
                            param->u.hdr.insrt_params.u.generic.p_data, size))
                {
                    XX_Free(p_data);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                param->u.hdr.insrt_params.u.generic.p_data = p_data;
            }

            if (param->id)
            {
                /* Security Hole: the user can pass any piece of garbage
                   in 'param->id', and that will go straight through to the LLD,
                   no checks being done by the wrapper! */
                err = FM_PCD_ManipNodeReplace(
                        (t_Handle) param->id,
                        (t_FmPcdManipParams*) param);
                if (err)
                {
                    if (p_data)
                        XX_Free(p_data);
                    XX_Free(param);
                    break;
                }
            }
            else
            {
                param->id = FM_PCD_ManipNodeSet(
                        p_LnxWrpFmDev->h_PcdDev,
                        (t_FmPcdManipParams*) param);
                if (!param->id)
                {
                    if (p_data)
                        XX_Free(p_data);
                    XX_Free(param);
                    err = E_INVALID_VALUE;
                    /* Since the LLD has no errno-style error reporting,
                       we're left here with no other option than to report
                       a generic E_INVALID_VALUE */
                    break;
                }
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_manip_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_manip_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_manip_params_t));
                if (!compat_param)
                {
                    if (p_data)
                        XX_Free(p_data);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_manip_params_t));

                compat_fm_pcd_manip_set_node(compat_param, param, COMPAT_K_TO_US);

                if (copy_to_user((ioc_compat_fm_pcd_manip_params_t *) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_manip_params_t)))
                    err = E_READ_FAILED;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_pcd_manip_params_t *)arg,
                            param, sizeof(ioc_fm_pcd_manip_params_t)))
                    err = E_READ_FAILED;
            }

            if (p_data)
                XX_Free(p_data);
            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MANIP_NODE_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_MANIP_NODE_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0, sizeof(ioc_fm_obj_t));
#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_obj_delete(&compat_id, &id);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_ManipNodeDelete(id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
	case FM_PCD_IOC_MANIP_GET_STATS_COMPAT:
#endif
        case FM_PCD_IOC_MANIP_GET_STATS:
	{
            ioc_fm_pcd_manip_get_stats_t param;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_manip_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_manip_get_stats_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_manip_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_manip_get_stats_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_manip_get_stats_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_manip_get_stats_t)))
                {
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_manip_get_stats(compat_param, &param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(&param, (ioc_fm_pcd_manip_get_stats_t *)arg,
                            sizeof(ioc_fm_pcd_manip_get_stats_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PCD_ManipGetStatistics((t_Handle) param.id,
						(t_FmPcdManipStats*) &param.stats);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_manip_get_stats_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_manip_get_stats_t*) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_manip_get_stats_t));
                if (!compat_param)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_manip_get_stats_t));
                compat_copy_fm_pcd_manip_get_stats(compat_param, &param, COMPAT_K_TO_US);
                if (copy_to_user((ioc_compat_fm_pcd_manip_get_stats_t*) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_manip_get_stats_t))){
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
                }
                XX_Free(compat_param);
            }
            else
#endif
            if (copy_to_user((ioc_fm_pcd_manip_get_stats_t *)arg,
                                  &param,
                                  sizeof(ioc_fm_pcd_manip_get_stats_t)))
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);

            break;
	}

#if (DPAA_VERSION >= 11)
#if defined(CONFIG_COMPAT)
	case FM_PCD_IOC_FRM_REPLIC_GROUP_SET_COMPAT:
#endif
	case FM_PCD_IOC_FRM_REPLIC_GROUP_SET:
	{
		ioc_fm_pcd_frm_replic_group_params_t *param;

		param = (ioc_fm_pcd_frm_replic_group_params_t *) XX_Malloc(
				sizeof(ioc_fm_pcd_frm_replic_group_params_t));
		if (!param)
			RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

		memset(param, 0, sizeof(ioc_fm_pcd_frm_replic_group_params_t));

#if defined(CONFIG_COMPAT)
		if (compat)
		{
			ioc_compat_fm_pcd_frm_replic_group_params_t
				*compat_param;

			compat_param =
				(ioc_compat_fm_pcd_frm_replic_group_params_t *)
					XX_Malloc(sizeof(ioc_compat_fm_pcd_frm_replic_group_params_t));
			if (!compat_param)
			{
				XX_Free(param);
				RETURN_ERROR(MINOR, E_NO_MEMORY,
						("IOCTL FM PCD"));
			}

			memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_frm_replic_group_params_t));
			if (copy_from_user(compat_param,
				(ioc_compat_fm_pcd_frm_replic_group_params_t *)
					compat_ptr(arg),
					sizeof(ioc_compat_fm_pcd_frm_replic_group_params_t))) {
				XX_Free(compat_param);
				XX_Free(param);
				RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
			}

			compat_copy_fm_pcd_frm_replic_group_params(compat_param,
					param, COMPAT_US_TO_K);

			XX_Free(compat_param);
		}
		else
#endif
		{
			if (copy_from_user(param,
				(ioc_fm_pcd_frm_replic_group_params_t *)arg,
				sizeof(ioc_fm_pcd_frm_replic_group_params_t)))
			{
				XX_Free(param);
				RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
			}
		}

		param->id = FM_PCD_FrmReplicSetGroup(p_LnxWrpFmDev->h_PcdDev,
				(t_FmPcdFrmReplicGroupParams*)param);

		if (!param->id) {
			XX_Free(param);
			err = E_INVALID_VALUE;
			/*
			 * Since the LLD has no errno-style error reporting,
			 * we're left here with no other option than to report
			 * a generic E_INVALID_VALUE
			 */
			break;
		}

#if defined(CONFIG_COMPAT)
		if (compat)
		{
			ioc_compat_fm_pcd_frm_replic_group_params_t
				*compat_param;

			compat_param =
				(ioc_compat_fm_pcd_frm_replic_group_params_t *)
					XX_Malloc(sizeof(ioc_compat_fm_pcd_frm_replic_group_params_t));
			if (!compat_param)
			{
				XX_Free(param);
				RETURN_ERROR(MINOR, E_NO_MEMORY,
						("IOCTL FM PCD"));
			}

			memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_frm_replic_group_params_t));
			compat_copy_fm_pcd_frm_replic_group_params(compat_param,
					param, COMPAT_K_TO_US);
			if (copy_to_user(
				(ioc_compat_fm_pcd_frm_replic_group_params_t *)
					compat_ptr(arg),
					compat_param,
					sizeof(ioc_compat_fm_pcd_frm_replic_group_params_t)))
				err = E_WRITE_FAILED;

			XX_Free(compat_param);
		}
		else
#endif
		{
			if (copy_to_user(
				(ioc_fm_pcd_frm_replic_group_params_t *)arg,
				param,
				sizeof(ioc_fm_pcd_frm_replic_group_params_t)))
				err = E_WRITE_FAILED;
		}

		XX_Free(param);
		break;
	}
	break;

#if defined(CONFIG_COMPAT)
	case FM_PCD_IOC_FRM_REPLIC_GROUP_DELETE_COMPAT:
#endif
	case FM_PCD_IOC_FRM_REPLIC_GROUP_DELETE:
	{
		ioc_fm_obj_t id;

		memset(&id, 0, sizeof(ioc_fm_obj_t));
#if defined(CONFIG_COMPAT)
		if (compat)
		{
			ioc_compat_fm_obj_t compat_id;

			if (copy_from_user(&compat_id,
					(ioc_compat_fm_obj_t *) compat_ptr(arg),
					sizeof(ioc_compat_fm_obj_t)))
				break;
			compat_obj_delete(&compat_id, &id);
		}
		else
#endif
		{
			if (copy_from_user(&id, (ioc_fm_obj_t *) arg,
					sizeof(ioc_fm_obj_t)))
				break;
		}

		return FM_PCD_FrmReplicDeleteGroup(id.obj);
	}
	break;

#if defined(CONFIG_COMPAT)
	case FM_PCD_IOC_FRM_REPLIC_MEMBER_ADD_COMPAT:
#endif
	case FM_PCD_IOC_FRM_REPLIC_MEMBER_ADD:
	{
		ioc_fm_pcd_frm_replic_member_params_t param;

#if defined(CONFIG_COMPAT)
		if (compat)
		{
			ioc_compat_fm_pcd_frm_replic_member_params_t compat_param;

			if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
				RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

			compat_copy_fm_pcd_frm_replic_member_params(&compat_param, &param, COMPAT_US_TO_K);
		}
		else
#endif
			if (copy_from_user(&param, (void *)arg, sizeof(param)))
				RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

		return FM_PCD_FrmReplicAddMember(param.member.h_replic_group,
			param.member.member_index,
			(t_FmPcdCcNextEngineParams*)&param.next_engine_params);
	}
	break;

#if defined(CONFIG_COMPAT)
	case FM_PCD_IOC_FRM_REPLIC_MEMBER_REMOVE_COMPAT:
#endif
	case FM_PCD_IOC_FRM_REPLIC_MEMBER_REMOVE:
	{
		ioc_fm_pcd_frm_replic_member_t param;

#if defined(CONFIG_COMPAT)
		if (compat)
		{
			ioc_compat_fm_pcd_frm_replic_member_t compat_param;

			if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
				RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

			compat_copy_fm_pcd_frm_replic_member(&compat_param, &param, COMPAT_US_TO_K);
		}
		else
#endif
			if (copy_from_user(&param, (void *)arg, sizeof(param)))
				RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

		return FM_PCD_FrmReplicRemoveMember(param.h_replic_group, param.member_index);
	}
	break;

#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_CONFIG_COMPAT:
#endif
    case FM_IOC_VSP_CONFIG:
    {
        ioc_fm_vsp_params_t param;

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_vsp_params_t compat_param;

            if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            compat_copy_fm_vsp_params(&compat_param, &param, COMPAT_US_TO_K);
        }
        else
#endif
            if (copy_from_user(&param, (void *)arg, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
        {
            uint8_t portId = param.port_params.port_id;
            param.liodn_offset =
                p_LnxWrpFmDev->rxPorts[portId].settings.param.specificParams.rxParams.liodnOffset;
        }
        param.p_fm = p_LnxWrpFmDev->h_Dev;
        param.id = FM_VSP_Config((t_FmVspParams *)&param);

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_vsp_params_t compat_param;

            memset(&compat_param, 0, sizeof(compat_param));
            compat_copy_fm_vsp_params(&compat_param, &param, COMPAT_K_TO_US);

            if (copy_to_user(compat_ptr(arg), &compat_param, sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
        }
        else
#endif
            if (copy_to_user((void *)arg, &param, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
        break;
    }

#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_INIT_COMPAT:
#endif
    case FM_IOC_VSP_INIT:
    {
        ioc_fm_obj_t id;

        memset(&id, 0, sizeof(ioc_fm_obj_t));
#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_obj_t compat_id;

            if (copy_from_user(&compat_id,
                    (ioc_compat_fm_obj_t *) compat_ptr(arg),
                    sizeof(ioc_compat_fm_obj_t)))
                break;
            id.obj = compat_pcd_id2ptr(compat_id.obj);
        }
        else
#endif
        {
            if (copy_from_user(&id, (ioc_fm_obj_t *) arg,
                    sizeof(ioc_fm_obj_t)))
                break;
        }

        return FM_VSP_Init(id.obj);
    }

#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_FREE_COMPAT:
#endif
    case FM_IOC_VSP_FREE:
    {
        ioc_fm_obj_t id;

        memset(&id, 0, sizeof(ioc_fm_obj_t));
#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_obj_t compat_id;

            if (copy_from_user(&compat_id,
                    (ioc_compat_fm_obj_t *) compat_ptr(arg),
                    sizeof(ioc_compat_fm_obj_t)))
                break;
            compat_obj_delete(&compat_id, &id);
        }
        else
#endif
        {
            if (copy_from_user(&id, (ioc_fm_obj_t *) arg,
                    sizeof(ioc_fm_obj_t)))
                break;
        }

        return FM_VSP_Free(id.obj);
    }

#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_CONFIG_POOL_DEPLETION_COMPAT:
#endif
    case FM_IOC_VSP_CONFIG_POOL_DEPLETION:
    {
        ioc_fm_buf_pool_depletion_params_t param;

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_buf_pool_depletion_params_t compat_param;

            if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            compat_copy_fm_buf_pool_depletion_params(&compat_param, &param, COMPAT_US_TO_K);
        }
        else
#endif
            if (copy_from_user(&param, (void *)arg, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        if (FM_VSP_ConfigPoolDepletion(param.p_fm_vsp,
                    (t_FmBufPoolDepletion *)&param.fm_buf_pool_depletion))
            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        break;
    }


#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_CONFIG_BUFFER_PREFIX_CONTENT_COMPAT:
#endif
    case FM_IOC_VSP_CONFIG_BUFFER_PREFIX_CONTENT:
    {
        ioc_fm_buffer_prefix_content_params_t param;

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_buffer_prefix_content_params_t compat_param;

            if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            compat_copy_fm_buffer_prefix_content_params(&compat_param, &param, COMPAT_US_TO_K);
        }
        else
#endif
            if (copy_from_user(&param, (void *)arg, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        if (FM_VSP_ConfigBufferPrefixContent(param.p_fm_vsp,
                (t_FmBufferPrefixContent *)&param.fm_buffer_prefix_content))
            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        break;
    }

#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_CONFIG_NO_SG_COMPAT:
#endif
    case FM_IOC_VSP_CONFIG_NO_SG:
    {
        ioc_fm_vsp_config_no_sg_params_t param;

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_vsp_config_no_sg_params_t compat_param;

            if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            compat_copy_fm_vsp_config_no_sg_params(&compat_param, &param, COMPAT_US_TO_K);
        }
        else
#endif
            if (copy_from_user(&param, (void *)arg, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        if (FM_VSP_ConfigNoScatherGather(param.p_fm_vsp, param.no_sg))
            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        break;
    }

#if defined(CONFIG_COMPAT)
    case FM_IOC_VSP_GET_BUFFER_PRS_RESULT_COMPAT:
#endif
    case FM_IOC_VSP_GET_BUFFER_PRS_RESULT:
    {
        ioc_fm_vsp_prs_result_params_t param;

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_vsp_prs_result_params_t compat_param;

            if (copy_from_user(&compat_param, compat_ptr(arg), sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            compat_copy_fm_vsp_prs_result_params(&compat_param, &param, COMPAT_US_TO_K);
        }
        else
#endif
            if (copy_from_user(&param, (void *)arg, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        /* this call just adds the parse results offset to p_data */
        param.p_data = FM_VSP_GetBufferPrsResult(param.p_fm_vsp, param.p_data);

        if (!param.p_data)
            RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

#if defined(CONFIG_COMPAT)
        if (compat)
        {
            ioc_compat_fm_vsp_prs_result_params_t compat_param;

            memset(&compat_param, 0, sizeof(compat_param));
            compat_copy_fm_vsp_prs_result_params(&compat_param, &param, COMPAT_K_TO_US);

            if (copy_to_user(compat_ptr(arg), &compat_param, sizeof(compat_param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
        }
        else
#endif
            if (copy_to_user((void *)arg, &param, sizeof(param)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

        break;
    }
#endif /* (DPAA_VERSION >= 11) */

#ifdef FM_CAPWAP_SUPPORT
#warning "feature not supported!"
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_STATISTICS_SET_NODE_COMPAT:
#endif
        case FM_PCD_IOC_STATISTICS_SET_NODE:
        {
/*          ioc_fm_pcd_stats_params_t param;
            ...
            param->id = FM_PCD_StatisticsSetNode(p_LnxWrpFmDev->h_PcdDev,
                                (t_FmPcdStatsParams *)&param);
*/
            err = E_NOT_SUPPORTED;
            break;
        }
#endif /* FM_CAPWAP_SUPPORT */

        default:
            RETURN_ERROR(MINOR, E_INVALID_SELECTION,
                ("invalid ioctl: cmd:0x%08x(type:0x%02x, nr: %d.\n",
                cmd, _IOC_TYPE(cmd), _IOC_NR(cmd)));
    }

    if (err)
        RETURN_ERROR(MINOR, err, ("IOCTL FM PCD"));

    return E_OK;
}

void FM_Get_Api_Version(ioc_fm_api_version_t *p_version)
{
	p_version->version.major = FMD_API_VERSION_MAJOR;
	p_version->version.minor = FMD_API_VERSION_MINOR;
	p_version->version.respin = FMD_API_VERSION_RESPIN;
	p_version->version.reserved = 0;
}

t_Error LnxwrpFmIOCTL(t_LnxWrpFmDev *p_LnxWrpFmDev, unsigned int cmd, unsigned long arg, bool compat)
{
    t_Error err = E_OK;

    switch (cmd)
    {
        case FM_IOC_SET_PORTS_BANDWIDTH:
        {
            ioc_fm_port_bandwidth_params *param;

            param = (ioc_fm_port_bandwidth_params*) XX_Malloc(sizeof(ioc_fm_port_bandwidth_params));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_port_bandwidth_params));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_bandwidth_params*)compat_ptr(arg), sizeof(ioc_fm_port_bandwidth_params)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_bandwidth_params*)arg, sizeof(ioc_fm_port_bandwidth_params)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err =  FM_SetPortsBandwidth(p_LnxWrpFmDev->h_Dev, (t_FmPortsBandwidthParams*) param);

            XX_Free(param);
            break;
        }

        case FM_IOC_GET_REVISION:
        {
            ioc_fm_revision_info_t *param;

            param = (ioc_fm_revision_info_t *) XX_Malloc(sizeof(ioc_fm_revision_info_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            FM_GetRevision(p_LnxWrpFmDev->h_Dev, (t_FmRevisionInfo*)param);
            /* This one never returns anything other than E_OK */

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_to_user((ioc_fm_revision_info_t *)compat_ptr(arg),
                            param,
                            sizeof(ioc_fm_revision_info_t))){
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
                 }
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_revision_info_t *)arg,
                            param,
                            sizeof(ioc_fm_revision_info_t))){
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_READ_FAILED, NO_MSG);
                }
            }
            XX_Free(param);
            break;
        }

        case FM_IOC_SET_COUNTER:
        {
            ioc_fm_counters_params_t *param;

            param = (ioc_fm_counters_params_t *) XX_Malloc(sizeof(ioc_fm_counters_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_counters_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)compat_ptr(arg), sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)arg, sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

			err = FM_ModifyCounter(p_LnxWrpFmDev->h_Dev,
					       (e_FmCounters)param->cnt,
					       param->val);

            XX_Free(param);
            break;
        }

        case FM_IOC_GET_COUNTER:
        {
            ioc_fm_counters_params_t *param;

            param = (ioc_fm_counters_params_t *) XX_Malloc(sizeof(ioc_fm_counters_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_counters_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)compat_ptr(arg), sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)arg, sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

			param->val = FM_GetCounter(p_LnxWrpFmDev->h_Dev,
						   (e_FmCounters)param->cnt);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_to_user((ioc_fm_counters_params_t *)compat_ptr(arg), param, sizeof(ioc_fm_counters_params_t)))
                    err = E_READ_FAILED;
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_counters_params_t *)arg, param, sizeof(ioc_fm_counters_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

        case FM_IOC_FORCE_INTR:
        {
            ioc_fm_exceptions param;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(param, (ioc_fm_exceptions*) compat_ptr(arg)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }
            else
#endif
            {
                if (get_user(param, (ioc_fm_exceptions*)arg))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_ForceIntr(p_LnxWrpFmDev->h_Dev, (e_FmExceptions)param);
            break;
        }

	case FM_IOC_GET_API_VERSION:
	{
		ioc_fm_api_version_t version;

		FM_Get_Api_Version(&version);

#if defined(CONFIG_COMPAT)
		if (compat)
		{
			if (copy_to_user(
				(ioc_fm_api_version_t *)compat_ptr(arg),
				&version, sizeof(version)))
				err = E_READ_FAILED;
		}
		else
#endif
		{
			if (copy_to_user((ioc_fm_api_version_t *)arg,
				&version, sizeof(version)))
				err = E_READ_FAILED;
		}
	}
	break;

        case FM_IOC_CTRL_MON_START:
        {
            FM_CtrlMonStart(p_LnxWrpFmDev->h_Dev);
        }
        break;

        case FM_IOC_CTRL_MON_STOP:
        {
            FM_CtrlMonStop(p_LnxWrpFmDev->h_Dev);
        }
        break;

#if defined(CONFIG_COMPAT)
        case FM_IOC_CTRL_MON_GET_COUNTERS_COMPAT:
#endif
        case FM_IOC_CTRL_MON_GET_COUNTERS:
        {
            ioc_fm_ctrl_mon_counters_params_t param;
            t_FmCtrlMon mon;

#if defined(CONFIG_COMPAT)
            ioc_compat_fm_ctrl_mon_counters_params_t compat_param;

            if (compat)
            {
                if (copy_from_user(&compat_param, (void *)compat_ptr(arg),
                            sizeof(compat_param)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                param.fm_ctrl_index = compat_param.fm_ctrl_index;
                param.p_mon = (fm_ctrl_mon_t *)compat_ptr(compat_param.p_mon);
            }
            else
#endif
            {
                if (copy_from_user(&param, (void *)arg, sizeof(ioc_fm_ctrl_mon_counters_params_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            if (FM_CtrlMonGetCounters(p_LnxWrpFmDev->h_Dev, param.fm_ctrl_index, &mon))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            if (copy_to_user(param.p_mon, &mon, sizeof(t_FmCtrlMon)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
        }
        break;

        default:
            return LnxwrpFmPcdIOCTL(p_LnxWrpFmDev, cmd, arg, compat);
    }

    if (err)
        RETURN_ERROR(MINOR, E_INVALID_OPERATION, ("IOCTL FM"));

    return E_OK;
}

t_Error LnxwrpFmPortIOCTL(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev, unsigned int cmd, unsigned long arg, bool compat)
{
    t_Error err = E_OK;

    _fm_ioctl_dbg("cmd:0x%08x(type:0x%02x, nr:%u).\n",
        cmd, _IOC_TYPE(cmd), _IOC_NR(cmd) - 70);

    switch (cmd)
    {
        case FM_PORT_IOC_DISABLE:
            FM_PORT_Disable(p_LnxWrpFmPortDev->h_Dev);
            /* deliberately ignoring error codes here */
            return E_OK;

        case FM_PORT_IOC_ENABLE:
            FM_PORT_Enable(p_LnxWrpFmPortDev->h_Dev);
            /* deliberately ignoring error codes here */
            return E_OK;

        case FM_PORT_IOC_SET_ERRORS_ROUTE:
        {
            ioc_fm_port_frame_err_select_t errs;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(errs, (ioc_fm_port_frame_err_select_t*)compat_ptr(arg)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }
            else
#endif
            {
                if (get_user(errs, (ioc_fm_port_frame_err_select_t*)arg))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PORT_SetErrorsRoute(p_LnxWrpFmPortDev->h_Dev, (fmPortFrameErrSelect_t)errs);
            break;
        }

        case FM_PORT_IOC_SET_RATE_LIMIT:
        {
            ioc_fm_port_rate_limit_t *param;

            param = (ioc_fm_port_rate_limit_t *) XX_Malloc(sizeof(ioc_fm_port_rate_limit_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_rate_limit_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_rate_limit_t *)compat_ptr(arg), sizeof(ioc_fm_port_rate_limit_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_rate_limit_t *)arg, sizeof(ioc_fm_port_rate_limit_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err =  FM_PORT_SetRateLimit(p_LnxWrpFmPortDev->h_Dev, (t_FmPortRateLimit *)param);

            XX_Free(param);
            break;
        }

        case FM_PORT_IOC_REMOVE_RATE_LIMIT:
            FM_PORT_DeleteRateLimit(p_LnxWrpFmPortDev->h_Dev);
            /* deliberately ignoring error codes here */
            return E_OK;

        case FM_PORT_IOC_ALLOC_PCD_FQIDS:
        {
            ioc_fm_port_pcd_fqids_params_t *param;

            if (!p_LnxWrpFmPortDev->pcd_owner_params.cba)
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("No one to listen on this PCD!!!"));

            param = (ioc_fm_port_pcd_fqids_params_t *) XX_Malloc(sizeof(ioc_fm_port_pcd_fqids_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_pcd_fqids_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_pcd_fqids_params_t *)compat_ptr(arg),
                                    sizeof(ioc_fm_port_pcd_fqids_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_pcd_fqids_params_t *)arg,
                                    sizeof(ioc_fm_port_pcd_fqids_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (p_LnxWrpFmPortDev->pcd_owner_params.cba(p_LnxWrpFmPortDev->pcd_owner_params.dev,
                                                        param->num_fqids,
                                                        param->alignment,
                                                        &param->base_fqid))
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("can't allocate fqids for PCD!!!"));
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_to_user((ioc_fm_port_pcd_fqids_params_t *)compat_ptr(arg),
                                  param, sizeof(ioc_fm_port_pcd_fqids_params_t)))
                    err = E_READ_FAILED;
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_port_pcd_fqids_params_t *)arg,
                                  param, sizeof(ioc_fm_port_pcd_fqids_params_t)))
                    err = E_READ_FAILED;
            }

            XX_Free(param);
            break;
        }

        case FM_PORT_IOC_FREE_PCD_FQIDS:
        {
            uint32_t base_fqid;

            if (!p_LnxWrpFmPortDev->pcd_owner_params.cbf)
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("No one to listen on this PCD!!!"));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(base_fqid, (uint32_t*) compat_ptr(arg)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }
            else
#endif
            {
                if (get_user(base_fqid, (uint32_t*)arg))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            if (p_LnxWrpFmPortDev->pcd_owner_params.cbf(p_LnxWrpFmPortDev->pcd_owner_params.dev, base_fqid))
               err = E_WRITE_FAILED;

            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_SET_PCD_COMPAT:
#endif
        case FM_PORT_IOC_SET_PCD:
        {
            ioc_fm_port_pcd_params_t      *port_pcd_params;
            ioc_fm_port_pcd_prs_params_t  *port_pcd_prs_params;
            ioc_fm_port_pcd_cc_params_t   *port_pcd_cc_params;
            ioc_fm_port_pcd_kg_params_t   *port_pcd_kg_params;
            ioc_fm_port_pcd_plcr_params_t *port_pcd_plcr_params;

            port_pcd_params = (ioc_fm_port_pcd_params_t *) XX_Malloc(
                    sizeof(ioc_fm_port_pcd_params_t) +
                    sizeof(ioc_fm_port_pcd_prs_params_t) +
                    sizeof(ioc_fm_port_pcd_cc_params_t) +
                    sizeof(ioc_fm_port_pcd_kg_params_t) +
                    sizeof(ioc_fm_port_pcd_plcr_params_t));
            if (!port_pcd_params)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(port_pcd_params, 0,
                    sizeof(ioc_fm_port_pcd_params_t) +
                    sizeof(ioc_fm_port_pcd_prs_params_t) +
                    sizeof(ioc_fm_port_pcd_cc_params_t) +
                    sizeof(ioc_fm_port_pcd_kg_params_t) +
                    sizeof(ioc_fm_port_pcd_plcr_params_t));

            port_pcd_prs_params  = (ioc_fm_port_pcd_prs_params_t *)  (port_pcd_params + 1);
            port_pcd_cc_params   = (ioc_fm_port_pcd_cc_params_t *)   (port_pcd_prs_params + 1);
            port_pcd_kg_params   = (ioc_fm_port_pcd_kg_params_t *)   (port_pcd_cc_params + 1);
            port_pcd_plcr_params = (ioc_fm_port_pcd_plcr_params_t *) (port_pcd_kg_params + 1);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_port_pcd_params_t      *compat_port_pcd_params;
                ioc_fm_port_pcd_prs_params_t         *same_port_pcd_prs_params;
                ioc_compat_fm_port_pcd_cc_params_t   *compat_port_pcd_cc_params;
                ioc_compat_fm_port_pcd_kg_params_t   *compat_port_pcd_kg_params;
                ioc_compat_fm_port_pcd_plcr_params_t *compat_port_pcd_plcr_params;

                compat_port_pcd_params = (ioc_compat_fm_port_pcd_params_t *) XX_Malloc(
                                sizeof(ioc_compat_fm_port_pcd_params_t) +
                                sizeof(ioc_fm_port_pcd_prs_params_t) +
                                sizeof(ioc_compat_fm_port_pcd_cc_params_t) +
                                sizeof(ioc_compat_fm_port_pcd_kg_params_t) +
                                sizeof(ioc_compat_fm_port_pcd_plcr_params_t));
                if (!compat_port_pcd_params)
                {
                    XX_Free(port_pcd_params);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));
                }

                memset(compat_port_pcd_params, 0,
                        sizeof(ioc_compat_fm_port_pcd_params_t) +
                        sizeof(ioc_fm_port_pcd_prs_params_t) +
                        sizeof(ioc_compat_fm_port_pcd_cc_params_t) +
                        sizeof(ioc_compat_fm_port_pcd_kg_params_t) +
                        sizeof(ioc_compat_fm_port_pcd_plcr_params_t));
                same_port_pcd_prs_params    = (ioc_fm_port_pcd_prs_params_t *) (compat_port_pcd_params + 1);
                compat_port_pcd_cc_params   = (ioc_compat_fm_port_pcd_cc_params_t *) (same_port_pcd_prs_params + 1);
                compat_port_pcd_kg_params   = (ioc_compat_fm_port_pcd_kg_params_t *) (compat_port_pcd_cc_params + 1);
                compat_port_pcd_plcr_params = (ioc_compat_fm_port_pcd_plcr_params_t *) (compat_port_pcd_kg_params + 1);

                if (copy_from_user(compat_port_pcd_params,
                            (ioc_compat_fm_port_pcd_params_t*) compat_ptr(arg),
                            sizeof(ioc_compat_fm_port_pcd_params_t)))
                    err = E_WRITE_FAILED;

                while (!err) /* pseudo-while */
                {
                    /* set pointers from where to copy from: */
                    port_pcd_params->p_prs_params          = compat_ptr(compat_port_pcd_params->p_prs_params); /* same structure */
                    port_pcd_params->p_cc_params           = compat_ptr(compat_port_pcd_params->p_cc_params);
                    port_pcd_params->p_kg_params           = compat_ptr(compat_port_pcd_params->p_kg_params);
                    port_pcd_params->p_plcr_params         = compat_ptr(compat_port_pcd_params->p_plcr_params);
                    port_pcd_params->p_ip_reassembly_manip = compat_ptr(compat_port_pcd_params->p_ip_reassembly_manip);
#if (DPAA_VERSION >= 11)
                    port_pcd_params->p_capwap_reassembly_manip = compat_ptr(compat_port_pcd_params->p_capwap_reassembly_manip);
#endif
                    /* the prs member is the same, no compat structure...memcpy only */
                    if (port_pcd_params->p_prs_params)
                    {
                        if (copy_from_user(same_port_pcd_prs_params,
                                port_pcd_params->p_prs_params,
                                sizeof(ioc_fm_port_pcd_prs_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        memcpy(port_pcd_prs_params, same_port_pcd_prs_params, sizeof(ioc_fm_port_pcd_prs_params_t));
                        port_pcd_params->p_prs_params = port_pcd_prs_params;
                    }

                    if (port_pcd_params->p_cc_params)
                    {
                        if (copy_from_user(compat_port_pcd_cc_params,
                                port_pcd_params->p_cc_params,
                                sizeof(ioc_compat_fm_port_pcd_cc_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_cc_params = port_pcd_cc_params;
                    }

                    if (port_pcd_params->p_kg_params)
                    {
                        if (copy_from_user(compat_port_pcd_kg_params,
                                port_pcd_params->p_kg_params,
                                sizeof(ioc_compat_fm_port_pcd_kg_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_kg_params = port_pcd_kg_params;
                    }

                    if (port_pcd_params->p_plcr_params)
                    {
                        if (copy_from_user(compat_port_pcd_plcr_params,
                                    port_pcd_params->p_plcr_params,
                                    sizeof(ioc_compat_fm_port_pcd_plcr_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_plcr_params = port_pcd_plcr_params;
                    }

                    break; /* pseudo-while: always run once! */
                }

                if (!err)
                    compat_copy_fm_port_pcd(compat_port_pcd_params, port_pcd_params, COMPAT_US_TO_K);

                XX_Free(compat_port_pcd_params);
            }
            else
#endif
            {
                if (copy_from_user(port_pcd_params,
                            (ioc_fm_port_pcd_params_t*) arg,
                            sizeof(ioc_fm_port_pcd_params_t)))
                    err = E_WRITE_FAILED;

                while (!err) /* pseudo-while */
                {
                    if (port_pcd_params->p_prs_params)
                    {
                        if (copy_from_user(port_pcd_prs_params,
                                port_pcd_params->p_prs_params,
                                sizeof(ioc_fm_port_pcd_prs_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_prs_params = port_pcd_prs_params;
                    }

                    if (port_pcd_params->p_cc_params)
                    {
                        if (copy_from_user(port_pcd_cc_params,
                                port_pcd_params->p_cc_params,
                                sizeof(ioc_fm_port_pcd_cc_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_cc_params = port_pcd_cc_params;
                    }

                    if (port_pcd_params->p_kg_params)
                    {
                        if (copy_from_user(port_pcd_kg_params,
                                port_pcd_params->p_kg_params,
                                sizeof(ioc_fm_port_pcd_kg_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_kg_params = port_pcd_kg_params;
                    }

                    if (port_pcd_params->p_plcr_params)
                    {
                        if (copy_from_user(port_pcd_plcr_params,
                                port_pcd_params->p_plcr_params,
                                sizeof(ioc_fm_port_pcd_plcr_params_t)))
                        {
                            err = E_WRITE_FAILED;
                            break; /* from pseudo-while */
                        }

                        port_pcd_params->p_plcr_params = port_pcd_plcr_params;
                    }

                    break; /* pseudo-while: always run once! */
                }
            }

            if (!err)
                err = FM_PORT_SetPCD(p_LnxWrpFmPortDev->h_Dev, (t_FmPortPcdParams*) port_pcd_params);

            XX_Free(port_pcd_params);
            break;
        }

        case FM_PORT_IOC_DELETE_PCD:
            err = FM_PORT_DeletePCD(p_LnxWrpFmPortDev->h_Dev);
            break;

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME_COMPAT:
#endif
        case FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME:
        {
            ioc_fm_pcd_kg_scheme_select_t *param;

            param = (ioc_fm_pcd_kg_scheme_select_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_kg_scheme_select_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_scheme_select_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_select_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_select_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_select_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_select_t));
                if (copy_from_user(compat_param,
                                   (ioc_compat_fm_pcd_kg_scheme_select_t *) compat_ptr(arg),
                                   sizeof(ioc_compat_fm_pcd_kg_scheme_select_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_kg_scheme_select(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_scheme_select_t *)arg,
                                   sizeof(ioc_fm_pcd_kg_scheme_select_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err =  FM_PORT_PcdKgModifyInitialScheme(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdKgSchemeSelect *)param);

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE_COMPAT:
#endif
        case FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PORT_PcdPlcrModifyInitialProfile(p_LnxWrpFmPortDev->h_Dev, id.obj);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_KG_BIND_SCHEMES_COMPAT:
#endif
        case FM_PORT_IOC_PCD_KG_BIND_SCHEMES:
        {
            ioc_fm_pcd_port_schemes_params_t *param;

            param = (ioc_fm_pcd_port_schemes_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_port_schemes_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0 , sizeof(ioc_fm_pcd_port_schemes_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_port_schemes_params_t compat_param;

                if (copy_from_user(&compat_param,
                            (ioc_compat_fm_pcd_port_schemes_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_port_schemes_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_kg_schemes_params(&compat_param, param, COMPAT_US_TO_K);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_port_schemes_params_t *) arg,
                            sizeof(ioc_fm_pcd_port_schemes_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PORT_PcdKgBindSchemes(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdPortSchemesParams *)param);

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES_COMPAT:
#endif
        case FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES:
        {
            ioc_fm_pcd_port_schemes_params_t *param;

            param = (ioc_fm_pcd_port_schemes_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_port_schemes_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0 , sizeof(ioc_fm_pcd_port_schemes_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_port_schemes_params_t compat_param;

                if (copy_from_user(&compat_param,
                            (ioc_compat_fm_pcd_port_schemes_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_port_schemes_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_kg_schemes_params(&compat_param, param, COMPAT_US_TO_K);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_port_schemes_params_t *) arg,
                            sizeof(ioc_fm_pcd_port_schemes_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err =  FM_PORT_PcdKgUnbindSchemes(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdPortSchemesParams *)param);

            XX_Free(param);
            break;
        }

        case FM_PORT_IOC_PCD_PLCR_ALLOC_PROFILES:
        {
            uint16_t num;
            if (get_user(num, (uint16_t*) arg))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            err = FM_PORT_PcdPlcrAllocProfiles(p_LnxWrpFmPortDev->h_Dev, num);
            break;
        }

        case FM_PORT_IOC_PCD_PLCR_FREE_PROFILES:
            err = FM_PORT_PcdPlcrFreeProfiles(p_LnxWrpFmPortDev->h_Dev);
            break;

        case FM_PORT_IOC_DETACH_PCD:
            err = FM_PORT_DetachPCD(p_LnxWrpFmPortDev->h_Dev);
            break;

        case FM_PORT_IOC_ATTACH_PCD:
            err = FM_PORT_AttachPCD(p_LnxWrpFmPortDev->h_Dev);
            break;

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_CC_MODIFY_TREE_COMPAT:
#endif
        case FM_PORT_IOC_PCD_CC_MODIFY_TREE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

                compat_copy_fm_port_pcd_modify_tree(&compat_id, &id, COMPAT_US_TO_K);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            err = FM_PORT_PcdCcModifyTree(p_LnxWrpFmPortDev->h_Dev, id.obj);
            break;
        }

        case FM_PORT_IOC_ADD_CONGESTION_GRPS:
        case FM_PORT_IOC_REMOVE_CONGESTION_GRPS:
        {
            ioc_fm_port_congestion_groups_t *param;

            param = (ioc_fm_port_congestion_groups_t*) XX_Malloc(sizeof(ioc_fm_port_congestion_groups_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_congestion_groups_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (t_FmPortCongestionGrps*) compat_ptr(arg),
                            sizeof(t_FmPortCongestionGrps)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif /* CONFIG_COMPAT */
            {
                if (copy_from_user(param, (t_FmPortCongestionGrps*) arg,
                            sizeof(t_FmPortCongestionGrps)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = (cmd == FM_PORT_IOC_ADD_CONGESTION_GRPS)
                ? FM_PORT_AddCongestionGrps(p_LnxWrpFmPortDev->h_Dev, (t_FmPortCongestionGrps*) param)
                : FM_PORT_RemoveCongestionGrps(p_LnxWrpFmPortDev->h_Dev, (t_FmPortCongestionGrps*) param)
                ;

            XX_Free(param);
            break;
        }

        case FM_PORT_IOC_ADD_RX_HASH_MAC_ADDR:
        case FM_PORT_IOC_REMOVE_RX_HASH_MAC_ADDR:
        {
            ioc_fm_port_mac_addr_params_t *param;

            param = (ioc_fm_port_mac_addr_params_t*) XX_Malloc(
                    sizeof(ioc_fm_port_mac_addr_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_mac_addr_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_mac_addr_params_t*) compat_ptr(arg),
                            sizeof(ioc_fm_port_mac_addr_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif /* CONFIG_COMPAT */
            {
                if (copy_from_user(param, (ioc_fm_port_mac_addr_params_t*) arg,
                            sizeof(ioc_fm_port_mac_addr_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            if (p_LnxWrpFmPortDev->pcd_owner_params.dev)
            {
                int id = -1;

                switch(p_LnxWrpFmPortDev->settings.param.portType)
                {
                    case e_FM_PORT_TYPE_RX:
                    case e_FM_PORT_TYPE_TX:
                        id = p_LnxWrpFmPortDev->id;
                    break;
                    case e_FM_PORT_TYPE_RX_10G:
                    case e_FM_PORT_TYPE_TX_10G:
                        id = p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_MACS;
                    break;
                    default:
                        err = E_NOT_AVAILABLE;
                        REPORT_ERROR(MINOR, err, ("Attempt to add/remove hash MAC addr. to/from MAC-less port!"));
                }
                if (id >= 0)
                {
                    t_LnxWrpFmDev *fm = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
                    t_Handle mac_handle = fm->macs[id].h_Dev;

                    err = (cmd == FM_PORT_IOC_ADD_RX_HASH_MAC_ADDR)
                        ? FM_MAC_AddHashMacAddr(mac_handle, (t_EnetAddr*) param)
                        : FM_MAC_RemoveHashMacAddr(mac_handle, (t_EnetAddr*) param);
                }
            }
            else
            {
                err = E_NOT_AVAILABLE;
                REPORT_ERROR(MINOR, err, ("Port not initialized or other error!?!?"));
            }

            XX_Free(param);
            break;
        }

        case FM_PORT_IOC_SET_TX_PAUSE_FRAMES:
        {
            t_LnxWrpFmDev *p_LnxWrpFmDev =
                    (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
            ioc_fm_port_tx_pause_frames_params_t param;
            int mac_id = p_LnxWrpFmPortDev->id;

            if(&p_LnxWrpFmDev->txPorts[mac_id] != p_LnxWrpFmPortDev)
                mac_id += FM_MAX_NUM_OF_1G_MACS; /* 10G port */

            if (copy_from_user(&param, (ioc_fm_port_tx_pause_frames_params_t *)arg,
                        sizeof(ioc_fm_port_tx_pause_frames_params_t)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            if (p_LnxWrpFmDev && p_LnxWrpFmDev->macs[mac_id].h_Dev)
            {
                FM_MAC_SetTxPauseFrames(p_LnxWrpFmDev->macs[mac_id].h_Dev,
                        param.priority,
                        param.pause_time,
                        param.thresh_time);
            }
            else
            {
                err = E_NOT_AVAILABLE;
                REPORT_ERROR(MINOR, err, ("Port not initialized or other error!"));
            }

            break;
        }

        case FM_PORT_IOC_CONFIG_BUFFER_PREFIX_CONTENT:
        {
            ioc_fm_buffer_prefix_content_t *param;

            param = (ioc_fm_buffer_prefix_content_t*) XX_Malloc(sizeof(ioc_fm_buffer_prefix_content_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_buffer_prefix_content_t));

            if (copy_from_user(param, (ioc_fm_buffer_prefix_content_t*) arg,
                        sizeof(ioc_fm_buffer_prefix_content_t)))
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            if (FM_PORT_ConfigBufferPrefixContent(p_LnxWrpFmPortDev->h_Dev,
                    (t_FmBufferPrefixContent *)param))
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            XX_Free(param);
            break;
        }

#if (DPAA_VERSION >= 11)
#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_VSP_ALLOC_COMPAT:
#endif
        case FM_PORT_IOC_VSP_ALLOC:
        {
            ioc_fm_port_vsp_alloc_params_t *param;
            t_LnxWrpFmDev *p_LnxWrpFmDev;
            t_LnxWrpFmPortDev *p_LnxWrpFmTxPortDev;

            param = (ioc_fm_port_vsp_alloc_params_t *) XX_Malloc(
                    sizeof(ioc_fm_port_vsp_alloc_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_vsp_alloc_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_port_vsp_alloc_params_t *compat_param;

                compat_param = (ioc_compat_fm_port_vsp_alloc_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_port_vsp_alloc_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_port_vsp_alloc_params_t));
                if (copy_from_user(compat_param,
                                   (ioc_compat_fm_port_vsp_alloc_params_t *) compat_ptr(arg),
                                   sizeof(ioc_compat_fm_port_vsp_alloc_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }

                compat_copy_fm_port_vsp_alloc_params(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_vsp_alloc_params_t *)arg,
                                   sizeof(ioc_fm_port_vsp_alloc_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            /* Userspace may not have the Tx port t_handle when issuing the IOCTL */
            if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX ||
                    p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX_10G)
            {
                /* Determine the Tx port t_Handle from the Rx port id */
                p_LnxWrpFmDev = p_LnxWrpFmPortDev->h_LnxWrpFmDev;
                p_LnxWrpFmTxPortDev = &p_LnxWrpFmDev->txPorts[p_LnxWrpFmPortDev->id];
                param->p_fm_tx_port = p_LnxWrpFmTxPortDev->h_Dev;
            }

            if (FM_PORT_VSPAlloc(p_LnxWrpFmPortDev->h_Dev, (t_FmPortVSPAllocParams *)param))
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);
            }

            XX_Free(param);
            break;
        }
#endif /* (DPAA_VERSION >= 11) */

        case FM_PORT_IOC_GET_MAC_STATISTICS:
        {
            t_LnxWrpFmDev *p_LnxWrpFmDev =
                    (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
            ioc_fm_port_mac_statistics_t param;
            int mac_id = p_LnxWrpFmPortDev->id;

            if (!p_LnxWrpFmDev)
                RETURN_ERROR(MINOR, E_NOT_AVAILABLE, ("Port not initialized or other error!"));

            if (&p_LnxWrpFmDev->txPorts[mac_id] != p_LnxWrpFmPortDev &&
                &p_LnxWrpFmDev->rxPorts[mac_id] != p_LnxWrpFmPortDev)
                mac_id += FM_MAX_NUM_OF_1G_MACS; /* 10G port */

            if (!p_LnxWrpFmDev->macs[mac_id].h_Dev)
                RETURN_ERROR(MINOR, E_NOT_AVAILABLE, ("Port not initialized or other error!"));

            if (FM_MAC_GetStatistics(p_LnxWrpFmDev->macs[mac_id].h_Dev,
                        (t_FmMacStatistics *)&param))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            if (copy_to_user((ioc_fm_port_mac_statistics_t *)arg, &param,
                        sizeof(ioc_fm_port_mac_statistics_t)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            break;
        }

        case FM_PORT_IOC_GET_MAC_FRAME_SIZE_COUNTERS:
        {
            t_LnxWrpFmDev *p_LnxWrpFmDev =
                    (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
            ioc_fm_port_mac_frame_size_counters_t param;
            t_FmMacFrameSizeCounters frameSizeCounters;
            int mac_id = p_LnxWrpFmPortDev->id;

            if (!p_LnxWrpFmDev)
                RETURN_ERROR(MINOR, E_NOT_AVAILABLE, ("Port not initialized or other error!"));

            if (&p_LnxWrpFmDev->txPorts[mac_id] != p_LnxWrpFmPortDev &&
                &p_LnxWrpFmDev->rxPorts[mac_id] != p_LnxWrpFmPortDev)
                mac_id += FM_MAX_NUM_OF_1G_MACS; /* 10G port */

            if (!p_LnxWrpFmDev->macs[mac_id].h_Dev)
                RETURN_ERROR(MINOR, E_NOT_AVAILABLE, ("Port not initialized or other error!"));

            if (copy_from_user(&param, (ioc_fm_port_mac_frame_size_counters_t *)arg,
                        sizeof(ioc_fm_port_mac_frame_size_counters_t)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            if (FM_MAC_GetFrameSizeCounters(p_LnxWrpFmDev->macs[mac_id].h_Dev,
                        &frameSizeCounters, param.type))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            param.count_pkts_64 = frameSizeCounters.count_pkts_64;
            param.count_pkts_65_to_127 = frameSizeCounters.count_pkts_65_to_127;
            param.count_pkts_128_to_255 = frameSizeCounters.count_pkts_128_to_255;
            param.count_pkts_256_to_511 = frameSizeCounters.count_pkts_256_to_511;
            param.count_pkts_512_to_1023 = frameSizeCounters.count_pkts_512_to_1023;
            param.count_pkts_1024_to_1518 = frameSizeCounters.count_pkts_1024_to_1518;
            param.count_pkts_1519_to_1522 = frameSizeCounters.count_pkts_1519_to_1522;

            if (copy_to_user((ioc_fm_port_mac_frame_size_counters_t *)arg, &param,
                        sizeof(ioc_fm_port_mac_frame_size_counters_t)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            break;
        }

        case FM_PORT_IOC_GET_BMI_COUNTERS:
        {
            t_LnxWrpFmDev *p_LnxWrpFmDev =
                    (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
            ioc_fm_port_bmi_stats_t param;

            if (!p_LnxWrpFmDev)
                RETURN_ERROR(MINOR, E_NOT_AVAILABLE, ("Port not initialized or other error!"));

            if (FM_PORT_GetBmiCounters(p_LnxWrpFmPortDev->h_Dev,
                        (t_FmPortBmiStats *)&param))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            if (copy_to_user((ioc_fm_port_bmi_stats_t *)arg, &param,
                        sizeof(ioc_fm_port_bmi_stats_t)))
                RETURN_ERROR(MINOR, E_WRITE_FAILED, NO_MSG);

            break;
        }

        default:
            RETURN_ERROR(MINOR, E_INVALID_SELECTION,
                ("invalid ioctl: cmd:0x%08x(type:0x%02x, nr:0x%02x.\n",
                cmd, _IOC_TYPE(cmd), _IOC_NR(cmd)));
    }

    if (err)
        RETURN_ERROR(MINOR, E_INVALID_OPERATION, ("IOCTL FM PORT"));

    return E_OK;
}

/*****************************************************************************/
/*               API routines for the FM Linux Device                        */
/*****************************************************************************/

static int fm_open(struct inode *inode, struct file *file)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = NULL;
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = NULL;
    unsigned int        major = imajor(inode);
    unsigned int        minor = iminor(inode);
    struct device_node  *fm_node;
    static struct of_device_id fm_node_of_match[] = {
        { .compatible = "fsl,fman", },
        { /* end of list */ },
    };

    DBG(TRACE, ("Opening minor - %d - ", minor));

    if (file->private_data != NULL)
        return 0;

    /* Get all the FM nodes */
    for_each_matching_node(fm_node, fm_node_of_match) {
        struct platform_device    *of_dev;

        of_dev = of_find_device_by_node(fm_node);
        if (unlikely(of_dev == NULL)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("fm id!"));
            return -ENXIO;
        }

        p_LnxWrpFmDev = (t_LnxWrpFmDev *)fm_bind(&of_dev->dev);
        if (p_LnxWrpFmDev->major == major)
            break;
        fm_unbind((struct fm *)p_LnxWrpFmDev);
        p_LnxWrpFmDev = NULL;
    }

    if (!p_LnxWrpFmDev)
        return -ENODEV;

    if (minor == DEV_FM_MINOR_BASE)
        file->private_data = p_LnxWrpFmDev;
    else if (minor == DEV_FM_PCD_MINOR_BASE)
        file->private_data = p_LnxWrpFmDev;
    else {
        if (minor == DEV_FM_OH_PORTS_MINOR_BASE)
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->hcPort;
        else if ((minor > DEV_FM_OH_PORTS_MINOR_BASE) && (minor < DEV_FM_RX_PORTS_MINOR_BASE))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->opPorts[minor-DEV_FM_OH_PORTS_MINOR_BASE-1];
        else if ((minor >= DEV_FM_RX_PORTS_MINOR_BASE) && (minor < DEV_FM_TX_PORTS_MINOR_BASE))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[minor-DEV_FM_RX_PORTS_MINOR_BASE];
        else if ((minor >= DEV_FM_TX_PORTS_MINOR_BASE) && (minor < DEV_FM_MAX_MINORS))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[minor-DEV_FM_TX_PORTS_MINOR_BASE];
        else
            return -EINVAL;

        /* if trying to open port, check if it initialized */
        if (!p_LnxWrpFmPortDev->h_Dev)
            return -ENODEV;

        p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *)fm_port_bind(p_LnxWrpFmPortDev->dev);
        file->private_data = p_LnxWrpFmPortDev;
        fm_unbind((struct fm *)p_LnxWrpFmDev);
    }

    if (file->private_data == NULL)
         return -ENXIO;

    return 0;
}

static int fm_close(struct inode *inode, struct file *file)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev;
    unsigned int        minor = iminor(inode);
    int                 err = 0;

    DBG(TRACE, ("Closing minor - %d - ", minor));

    if ((minor == DEV_FM_MINOR_BASE) ||
        (minor == DEV_FM_PCD_MINOR_BASE))
    {
        p_LnxWrpFmDev = (t_LnxWrpFmDev*)file->private_data;
        if (!p_LnxWrpFmDev)
            return -ENODEV;
        fm_unbind((struct fm *)p_LnxWrpFmDev);
    }
    else if (((minor >= DEV_FM_OH_PORTS_MINOR_BASE) && (minor < DEV_FM_RX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_RX_PORTS_MINOR_BASE) && (minor < DEV_FM_TX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_TX_PORTS_MINOR_BASE) && (minor < DEV_FM_MAX_MINORS)))
    {
        p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)file->private_data;
        if (!p_LnxWrpFmPortDev)
            return -ENODEV;
        fm_port_unbind((struct fm_port *)p_LnxWrpFmPortDev);
    }

    return err;
}

static int fm_ioctls(unsigned int minor, struct file *file, unsigned int cmd, unsigned long arg, bool compat)
{
    DBG(TRACE, ("IOCTL minor - %u, cmd - 0x%08x, arg - 0x%08lx \n", minor, cmd, arg));

    if ((minor == DEV_FM_MINOR_BASE) ||
        (minor == DEV_FM_PCD_MINOR_BASE))
    {
        t_LnxWrpFmDev *p_LnxWrpFmDev = ((t_LnxWrpFmDev*)file->private_data);
        if (!p_LnxWrpFmDev)
            return -ENODEV;
        if (LnxwrpFmIOCTL(p_LnxWrpFmDev, cmd, arg, compat))
            return -EFAULT;
    }
    else if (((minor >= DEV_FM_OH_PORTS_MINOR_BASE) && (minor < DEV_FM_RX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_RX_PORTS_MINOR_BASE) && (minor < DEV_FM_TX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_TX_PORTS_MINOR_BASE) && (minor < DEV_FM_MAX_MINORS)))
    {
        t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = ((t_LnxWrpFmPortDev*)file->private_data);
        if (!p_LnxWrpFmPortDev)
            return -ENODEV;
        if (LnxwrpFmPortIOCTL(p_LnxWrpFmPortDev, cmd, arg, compat))
            return -EFAULT;
    }
    else
    {
        REPORT_ERROR(MINOR, E_INVALID_VALUE, ("minor"));
        return -ENODEV;
    }

    return 0;
}

#ifdef CONFIG_COMPAT
static long fm_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned int minor = iminor(file->f_path.dentry->d_inode);
    long res;

    fm_mutex_lock();
    res = fm_ioctls(minor, file, cmd, arg, true);
    fm_mutex_unlock();

    return res;
}
#endif

static long fm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned int minor = iminor(file->f_path.dentry->d_inode);
    long res;

    fm_mutex_lock();
    res = fm_ioctls(minor, file, cmd, arg, false);
    fm_mutex_unlock();

    return res;
}

/* Globals for FM character device */
struct file_operations fm_fops =
{
    .owner =            THIS_MODULE,
    .unlocked_ioctl =   fm_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl =     fm_compat_ioctl,
#endif
    .open =             fm_open,
    .release =          fm_close,
};
