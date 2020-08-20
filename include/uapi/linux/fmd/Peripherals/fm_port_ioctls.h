/* Copyright (c) 2008-2012 Freescale Semiconductor, Inc.
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

/******************************************************************************
 @File          fm_port_ioctls.h

 @Description   FM Port routines
*//***************************************************************************/
#ifndef __FM_PORT_IOCTLS_H
#define __FM_PORT_IOCTLS_H

#include "enet_ext.h"
#include "net_ioctls.h"
#include "fm_ioctls.h"
#include "fm_pcd_ioctls.h"


/**************************************************************************//**

 @Group         lnx_ioctl_FM_grp Frame Manager Linux IOCTL API

 @Description   FM Linux ioctls definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         lnx_ioctl_FM_PORT_grp FM Port

 @Description   FM Port API

                The FM uses a general module called "port" to represent a Tx port
                (MAC), an Rx port (MAC), offline parsing flow or host command
                flow. There may be up to 17 (may change) ports in an FM - 5 Tx
                ports (4 for the 1G MACs, 1 for the 10G MAC), 5 Rx Ports, and 7
                Host command/Offline parsing ports. The SW driver manages these
                ports as sub-modules of the FM, i.e. after an FM is initialized,
                its ports may be initialized and operated upon.

                The port is initialized aware of its type, but other functions on
                a port may be indifferent to its type. When necessary, the driver
                verifies coherency and returns error if applicable.

                On initialization, user specifies the port type and it's index
                (relative to the port's type). Host command and Offline parsing
                ports share the same id range, I.e user may not initialized host
                command port 0 and offline parsing port 0.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   An enum for defining port PCD modes.
                (Must match enum e_FmPortPcdSupport defined in fm_port_ext.h)

                This enum defines the superset of PCD engines support - i.e. not
                all engines have to be used, but all have to be enabled. The real
                flow of a specific frame depends on the PCD configuration and the
                frame headers and payload.
                Note: the first engine and the first engine after the parser (if
                exists) should be in order, the order is important as it will
                define the flow of the port. However, as for the rest engines
                (the ones that follows), the order is not important anymore as
                it is defined by the PCD graph itself.
*//***************************************************************************/
typedef enum ioc_fm_port_pcd_support {
      e_IOC_FM_PORT_PCD_SUPPORT_NONE = 0                /**< BMI to BMI, PCD is not used */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_ONLY                /**< Use only Parser */
    , e_IOC_FM_PORT_PCD_SUPPORT_PLCR_ONLY               /**< Use only Policer */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR            /**< Use Parser and Policer */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG              /**< Use Parser and Keygen */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC       /**< Use Parser, Keygen and Coarse Classification */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR
                                                        /**< Use all PCD engines */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR     /**< Use Parser, Keygen and Policer */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_CC              /**< Use Parser and Coarse Classification */
    , e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_CC_AND_PLCR     /**< Use Parser and Coarse Classification and Policer */
    , e_IOC_FM_PORT_PCD_SUPPORT_CC_ONLY                 /**< Use only Coarse Classification */
#if (defined(FM_CAPWAP_SUPPORT) && (DPAA_VERSION == 10))
    , e_IOC_FM_PORT_PCD_SUPPORT_CC_AND_KG               /**< Use Coarse Classification,and Keygen */
    , e_IOC_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR      /**< Use Coarse Classification, Keygen and Policer */
#endif /* FM_CAPWAP_SUPPORT */
} ioc_fm_port_pcd_support;


/**************************************************************************//**
 @Collection   FM Frame error
*//***************************************************************************/
typedef uint32_t    ioc_fm_port_frame_err_select_t;     /**< typedef for defining Frame Descriptor errors */

/* @} */


/**************************************************************************//**
 @Description   An enum for defining Dual Tx rate limiting scale.
                (Must match e_FmPortDualRateLimiterScaleDown defined in fm_port_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_port_dual_rate_limiter_scale_down {
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_NONE = 0,           /**< Use only single rate limiter  */
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_2,    /**< Divide high rate limiter by 2 */
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_4,    /**< Divide high rate limiter by 4 */
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8     /**< Divide high rate limiter by 8 */
} ioc_fm_port_dual_rate_limiter_scale_down;

/**************************************************************************//**
 @Description   A structure for defining Tx rate limiting
                (Must match struct t_FmPortRateLimit defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_rate_limit_t {
    uint16_t                            max_burst_size;         /**< in KBytes for Tx ports, in frames
                                                                     for offline parsing ports. (note that
                                                                     for early chips burst size is
                                                                     rounded up to a multiply of 1000 frames).*/
    uint32_t                            rate_limit;             /**< in Kb/sec for Tx ports, in frame/sec for
                                                                     offline parsing ports. Rate limit refers to
                                                                     data rate (rather than line rate). */
    ioc_fm_port_dual_rate_limiter_scale_down rate_limit_divider;    /**< For offline parsing ports only. Not-valid
                                                                     for some earlier chip revisions */
} ioc_fm_port_rate_limit_t;



/**************************************************************************//**
 @Group         lnx_ioctl_FM_PORT_runtime_control_grp FM Port Runtime Control Unit

 @Description   FM Port Runtime control unit API functions, definitions and enums.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   An enum for defining FM Port counters.
                (Must match enum e_FmPortCounters defined in fm_port_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_port_counters {
    e_IOC_FM_PORT_COUNTERS_CYCLE,                       /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_TASK_UTIL,                   /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_QUEUE_UTIL,                  /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_DMA_UTIL,                    /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_FIFO_UTIL,                   /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_RX_PAUSE_ACTIVATION,         /**< BMI Rx only performance counter */
    e_IOC_FM_PORT_COUNTERS_FRAME,                       /**< BMI statistics counter */
    e_IOC_FM_PORT_COUNTERS_DISCARD_FRAME,               /**< BMI statistics counter */
    e_IOC_FM_PORT_COUNTERS_DEALLOC_BUF,                 /**< BMI deallocate buffer statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_BAD_FRAME,                /**< BMI Rx only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_LARGE_FRAME,              /**< BMI Rx only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_FILTER_FRAME,             /**< BMI Rx & OP only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_LIST_DMA_ERR,             /**< BMI Rx, OP & HC only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD,   /**< BMI Rx, OP & HC statistics counter */
    e_IOC_FM_PORT_COUNTERS_PREPARE_TO_ENQUEUE_COUNTER,  /**< BMI Rx, OP & HC only statistics counter */
    e_IOC_FM_PORT_COUNTERS_WRED_DISCARD,                /**< BMI OP & HC only statistics counter */
    e_IOC_FM_PORT_COUNTERS_LENGTH_ERR,                  /**< BMI non-Rx statistics counter */
    e_IOC_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT,           /**< BMI non-Rx statistics counter */
    e_IOC_FM_PORT_COUNTERS_DEQ_TOTAL,                   /**< QMI total QM dequeues counter */
    e_IOC_FM_PORT_COUNTERS_ENQ_TOTAL,                   /**< QMI total QM enqueues counter */
    e_IOC_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT,            /**< QMI counter */
    e_IOC_FM_PORT_COUNTERS_DEQ_CONFIRM                  /**< QMI counter */
} ioc_fm_port_counters;

typedef struct ioc_fm_port_bmi_stats_t {
    uint32_t cnt_cycle;
    uint32_t cnt_task_util;
    uint32_t cnt_queue_util;
    uint32_t cnt_dma_util;
    uint32_t cnt_fifo_util;
    uint32_t cnt_rx_pause_activation;
    uint32_t cnt_frame;
    uint32_t cnt_discard_frame;
    uint32_t cnt_dealloc_buf;
    uint32_t cnt_rx_bad_frame;
    uint32_t cnt_rx_large_frame;
    uint32_t cnt_rx_filter_frame;
    uint32_t cnt_rx_list_dma_err;
    uint32_t cnt_rx_out_of_buffers_discard;
    uint32_t cnt_wred_discard;
    uint32_t cnt_length_err;
    uint32_t cnt_unsupported_format;
} ioc_fm_port_bmi_stats_t;

/**************************************************************************//**
 @Description   Structure for Port id parameters.
                (Description may be inaccurate;
                must match struct t_FmPortCongestionGrps defined in fm_port_ext.h)

                Fields commented 'IN' are passed by the port module to be used
                by the FM module.
                Fields commented 'OUT' will be filled by FM before returning to port.
*//***************************************************************************/
typedef struct ioc_fm_port_congestion_groups_t {
    uint16_t    num_of_congestion_grps_to_consider;     /**< The number of required congestion groups
                                                             to define the size of the following array */
    uint8_t     congestion_grps_to_consider [FM_PORT_NUM_OF_CONGESTION_GRPS];
                                                        /**< An array of CG indexes;
                                                             Note that the size of the array should be
                                                             'num_of_congestion_grps_to_consider'. */
#if DPAA_VERSION >= 11
    bool        pfc_priorities_enable[FM_PORT_NUM_OF_CONGESTION_GRPS][FM_MAX_NUM_OF_PFC_PRIORITIES];
                                                        /**< A matrix that represents the map between the CG ids
                                                             defined in 'congestion_grps_to_consider' to the priorities
                                                             mapping array. */
#endif /* DPAA_VERSION >= 11 */
} ioc_fm_port_congestion_groups_t;



/**************************************************************************//**
 @Function      FM_PORT_Disable

 @Description   Gracefully disable an FM port. The port will not start new tasks after all
                tasks associated with the port are terminated.

 @Return        0 on success; error code otherwise.

 @Cautions      This is a blocking routine, it returns after port is
                gracefully stopped, i.e. the port will not except new frames,
                but it will finish all frames or tasks which were already began
*//***************************************************************************/
#define FM_PORT_IOC_DISABLE   _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(1))

/**************************************************************************//**
 @Function      FM_PORT_Enable

 @Description   A runtime routine provided to allow disable/enable of port.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_ENABLE   _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(2))

/**************************************************************************//**
 @Function      FM_PORT_SetRateLimit

 @Description   Calling this routine enables rate limit algorithm.
                By default, this functionality is disabled.
                Note that rate-limit mechanism uses the FM time stamp.
                The selected rate limit specified here would be
                rounded DOWN to the nearest 16M.

                May be used for Tx and offline parsing ports only

 @Param[in]     ioc_fm_port_rate_limit A structure of rate limit parameters

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_SET_RATE_LIMIT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(3), ioc_fm_port_rate_limit_t)

/**************************************************************************//**
 @Function      FM_PORT_DeleteRateLimit

 @Description   Calling this routine disables the previously enabled rate limit.

                May be used for Tx and offline parsing ports only

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_DELETE_RATE_LIMIT   _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(5))
#define FM_PORT_IOC_REMOVE_RATE_LIMIT   FM_PORT_IOC_DELETE_RATE_LIMIT


/**************************************************************************//**
 @Function      FM_PORT_AddCongestionGrps

 @Description   This routine effects the corresponding Tx port.
                It should be called in order to enable pause
                frame transmission in case of congestion in one or more
                of the congestion groups relevant to this port.
                Each call to this routine may add one or more congestion
                groups to be considered relevant to this port.

                May be used for Rx, or RX+OP ports only (depending on chip)

 @Param[in]     ioc_fm_port_congestion_groups_t - A pointer to an array of
                                                congestion group ids to consider.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_ADD_CONGESTION_GRPS    _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(34), ioc_fm_port_congestion_groups_t)

/**************************************************************************//**
 @Function      FM_PORT_RemoveCongestionGrps

 @Description   This routine effects the corresponding Tx port. It should be
                called when congestion groups were
                defined for this port and are no longer relevant, or pause
                frames transmitting is not required on their behalf.
                Each call to this routine may remove one or more congestion
                groups to be considered relevant to this port.

                May be used for Rx, or RX+OP ports only (depending on chip)

 @Param[in]     ioc_fm_port_congestion_groups_t - A pointer to an array of
                                                congestion group ids to consider.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_REMOVE_CONGESTION_GRPS    _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(35), ioc_fm_port_congestion_groups_t)

/**************************************************************************//**
 @Function      FM_PORT_SetErrorsRoute

 @Description   Errors selected for this routine will cause a frame with that error
                to be enqueued to error queue.
                Errors not selected for this routine will cause a frame with that error
                to be enqueued to the one of the other port queues.
                By default all errors are defined to be enqueued to error queue.
                Errors that were configured to be discarded (at initialization)
                may not be selected here.

                May be used for Rx and offline parsing ports only

 @Param[in]     ioc_fm_port_frame_err_select_t  A list of errors to enqueue to error queue

 @Return        0 on success; error code otherwise.

 @Cautions      Allowed only following FM_PORT_Config() and before FM_PORT_Init().
                (szbs001: How is it possible to have one function that needs to be
                          called BEFORE FM_PORT_Init() implemented as an ioctl,
                          which will ALWAYS be called AFTER the FM_PORT_Init()
                          for that port!?!?!?!???!?!??!?!?)
*//***************************************************************************/
#define FM_PORT_IOC_SET_ERRORS_ROUTE   _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(4), ioc_fm_port_frame_err_select_t)


/**************************************************************************//**
 @Group         lnx_ioctl_FM_PORT_pcd_runtime_control_grp FM Port PCD Runtime Control Unit

 @Description   FM Port PCD Runtime control unit API functions, definitions and enums.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   A structure defining the KG scheme after the parser.
                (Must match struct t_FmPcdKgSchemeSelect defined in fm_port_ext.h)

                This is relevant only to change scheme selection mode - from
                direct to indirect and vice versa, or when the scheme is selected directly,
                to select the scheme id.

*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_scheme_select_t {
    bool        direct;                     /**< TRUE to use 'scheme_id' directly, FALSE to use LCV.*/
    void       *scheme_id;                  /**< Relevant for 'direct'=TRUE only.
                                                 'scheme_id' selects the scheme after parser. */
} ioc_fm_pcd_kg_scheme_select_t;

/**************************************************************************//**
 @Description   Scheme IDs structure
                (Must match struct t_FmPcdPortSchemesParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_port_schemes_params_t {
    uint8_t     num_of_schemes;                         /**< Number of schemes for port to be bound to. */
    void        *scheme_ids[FM_PCD_KG_NUM_OF_SCHEMES];  /**< Array of 'num_of_schemes' schemes for the
                                                             port to be bound to */
} ioc_fm_pcd_port_schemes_params_t;

/**************************************************************************//**
 @Description   A union for defining port protocol parameters for parser
                (Must match union u_FmPcdHdrPrsOpts defined in fm_port_ext.h)
*//***************************************************************************/
typedef union ioc_fm_pcd_hdr_prs_opts_u {
    /* MPLS */
    struct {
        bool                label_interpretation_enable;/**< When this bit is set, the last MPLS label will be
                                                             interpreted as described in HW spec table. When the bit
                                                             is cleared, the parser will advance to MPLS next parse */
        ioc_net_header_type next_parse;                 /**< must be equal or higher than IPv4 */
    } mpls_prs_options;

    /* VLAN */
    struct {
        uint16_t            tag_protocol_id1;           /**< User defined Tag Protocol Identifier, to be recognized
                                                             on VLAN TAG on top of 0x8100 and 0x88A8 */
        uint16_t            tag_protocol_id2;           /**< User defined Tag Protocol Identifier, to be recognized
                                                             on VLAN TAG on top of 0x8100 and 0x88A8 */
    } vlan_prs_options;

    /* PPP */
    struct{
        bool                enable_mtu_check;           /**< Check validity of MTU according to RFC2516 */
    } pppoe_prs_options;

    /* IPV6 */
    struct {
        bool                routing_hdr_disable;        /**< Disable routing header */
    } ipv6_prs_options;

    /* UDP */
    struct {
        bool                pad_ignore_checksum;        /**< TRUE to ignore pad in checksum */
    } udp_prs_options;

    /* TCP */
    struct {
        bool                pad_ignore_checksum;        /**< TRUE to ignore pad in checksum */
    } tcp_prs_options;
} ioc_fm_pcd_hdr_prs_opts_u;

/**************************************************************************//**
 @Description   A structure for defining each header for the parser
                (must match struct t_FmPcdPrsAdditionalHdrParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_prs_additional_hdr_params_t {
    ioc_net_header_type         hdr;                /**< Selected header */
    bool                        err_disable;        /**< TRUE to disable error indication */
    bool                        soft_prs_enable;    /**< Enable jump to SW parser when this
                                                         header is recognized by the HW parser. */
    uint8_t                     index_per_hdr;      /**< Normally 0, if more than one sw parser
                                                         attachments exists for the same header,
                                                         (in the main sw parser code) use this
                                                         index to distinguish between them. */
    bool                        use_prs_opts;       /**< TRUE to use parser options. */
    ioc_fm_pcd_hdr_prs_opts_u   prs_opts;           /**< A unuion according to header type,
                                                         defining the parser options selected.*/
} ioc_fm_pcd_prs_additional_hdr_params_t;

/**************************************************************************//**
 @Description   A structure for defining port PCD parameters
                (Must match t_FmPortPcdPrsParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_prs_params_t {
    uint8_t                         prs_res_priv_info;      /**< The private info provides a method of inserting
                                                                 port information into the parser result. This information
                                                                 may be extracted by KeyGen and be used for frames
                                                                 distribution when a per-port distinction is required,
                                                                 it may also be used as a port logical id for analyzing
                                                                 incoming frames. */
    uint8_t                         parsing_offset;         /**< Number of bytes from begining of packet to start parsing */
    ioc_net_header_type             first_prs_hdr;          /**< The type of the first header axpected at 'parsing_offset' */
    bool                            include_in_prs_statistics; /**< TRUE to include this port in the parser statistics */
    uint8_t                         num_of_hdrs_with_additional_params;
                                                            /**< Normally 0, some headers may get special parameters */
    ioc_fm_pcd_prs_additional_hdr_params_t  additional_params[IOC_FM_PCD_PRS_NUM_OF_HDRS];
                                                            /**< 'num_of_hdrs_with_additional_params' structures
                                                                  additional parameters for each header that requires them */
    bool                            set_vlan_tpid1;         /**< TRUE to configure user selection of Ethertype to
                                                                 indicate a VLAN tag (in addition to the TPID values
                                                                 0x8100 and 0x88A8). */
    uint16_t                        vlan_tpid1;             /**< extra tag to use if set_vlan_tpid1=TRUE. */
    bool                            set_vlan_tpid2;         /**< TRUE to configure user selection of Ethertype to
                                                                 indicate a VLAN tag (in addition to the TPID values
                                                                 0x8100 and 0x88A8). */
    uint16_t                        vlan_tpid2;             /**< extra tag to use if set_vlan_tpid1=TRUE. */
} ioc_fm_port_pcd_prs_params_t;

/**************************************************************************//**
 @Description   A structure for defining coarse alassification parameters
                (Must match t_FmPortPcdCcParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_cc_params_t {
    void                *cc_tree_id; /**< CC tree id */
} ioc_fm_port_pcd_cc_params_t;

/**************************************************************************//**
 @Description   A structure for defining keygen parameters
                (Must match t_FmPortPcdKgParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_kg_params_t {
    uint8_t             num_of_schemes;                 /**< Number of schemes for port to be bound to. */
    void               *scheme_ids[FM_PCD_KG_NUM_OF_SCHEMES];
                                                        /**< Array of 'num_of_schemes' schemes for the
                                                             port to be bound to */
    bool                direct_scheme;                  /**< TRUE for going from parser to a specific scheme,
                                                             regardless of parser result */
    void               *direct_scheme_id;               /**< Scheme id, as returned by FM_PCD_KgSetScheme;
                                                             relevant only if direct=TRUE. */
} ioc_fm_port_pcd_kg_params_t;

/**************************************************************************//**
 @Description   A structure for defining policer parameters
                (Must match t_FmPortPcdPlcrParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_plcr_params_t {
    void                *plcr_profile_id;               /**< Selected profile handle;
                                                             relevant in one of the following cases:
                                                             e_IOC_FM_PORT_PCD_SUPPORT_PLCR_ONLY or
                                                             e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR were selected,
                                                             or if any flow uses a KG scheme where policer
                                                                profile is not generated (bypass_plcr_profile_generation selected) */
} ioc_fm_port_pcd_plcr_params_t;

/**************************************************************************//**
 @Description   A structure for defining port PCD parameters
                (Must match struct t_FmPortPcdParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_params_t {
    ioc_fm_port_pcd_support         pcd_support;    /**< Relevant for Rx and offline ports only.
                                                         Describes the active PCD engines for this port. */
    void                            *net_env_id;    /**< HL Unused in PLCR only mode */
    ioc_fm_port_pcd_prs_params_t    *p_prs_params;  /**< Parser parameters for this port */
    ioc_fm_port_pcd_cc_params_t     *p_cc_params;   /**< Coarse classification parameters for this port */
    ioc_fm_port_pcd_kg_params_t     *p_kg_params;   /**< Keygen parameters for this port */
    ioc_fm_port_pcd_plcr_params_t   *p_plcr_params; /**< Policer parameters for this port */
    void                            *p_ip_reassembly_manip;/**< IP Reassembly manipulation */
#if (DPAA_VERSION >= 11)
    void                            *p_capwap_reassembly_manip;/**< CAPWAP Reassembly manipulation */
#endif /* (DPAA_VERSION >= 11) */
} ioc_fm_port_pcd_params_t;

/**************************************************************************//**
 @Description   A structure for defining the Parser starting point
                (Must match struct t_FmPcdPrsStart defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_prs_start_t {
    uint8_t             parsing_offset; /**< Number of bytes from begining of packet to
                                             start parsing */
    ioc_net_header_type first_prs_hdr;  /**< The type of the first header axpected at
                                             'parsing_offset' */
} ioc_fm_pcd_prs_start_t;


/**************************************************************************//**
 @Description   FQID parameters structure
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_fqids_params_t {
    uint32_t            num_fqids;  /**< Number of fqids to be allocated for the port */
    uint8_t             alignment;  /**< Alignment required for this port */
    uint32_t            base_fqid;  /**< output parameter - the base fqid */
} ioc_fm_port_pcd_fqids_params_t;


/**************************************************************************//**
 @Function      FM_PORT_IOC_ALLOC_PCD_FQIDS

 @Description   Allocates FQID's

                May be used for Rx and offline parsing ports only

 @Param[in,out] ioc_fm_port_pcd_fqids_params_t  Parameters for allocating FQID's

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_ALLOC_PCD_FQIDS   _IOWR(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(19), ioc_fm_port_pcd_fqids_params_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_FREE_PCD_FQIDS

 @Description   Frees previously-allocated FQIDs

                May be used for Rx and offline parsing ports only

 @Param[in]		uint32_t	Base FQID of previously allocated range.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_FREE_PCD_FQIDS   _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(19), uint32_t)


/**************************************************************************//**
 @Function      FM_PORT_SetPCD

 @Description   Calling this routine defines the port's PCD configuration.
                It changes it from its default configuration which is PCD
                disabled (BMI to BMI) and configures it according to the passed
                parameters.

                May be used for Rx and offline parsing ports only

 @Param[in]     ioc_fm_port_pcd_params_t    A Structure of parameters defining the port's PCD
                                            configuration.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_SET_PCD_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(20), ioc_compat_fm_port_pcd_params_t)
#endif
#define FM_PORT_IOC_SET_PCD _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(20), ioc_fm_port_pcd_params_t)

/**************************************************************************//**
 @Function      FM_PORT_DeletePCD

 @Description   Calling this routine releases the port's PCD configuration.
                The port returns to its default configuration which is PCD
                disabled (BMI to BMI) and all PCD configuration is removed.

                May be used for Rx and offline parsing ports which are
                in PCD mode only

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_DELETE_PCD _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(21))

/**************************************************************************//**
 @Function      FM_PORT_AttachPCD

 @Description   This routine may be called after FM_PORT_DetachPCD was called,
                to return to the originally configured PCD support flow.
                The couple of routines are used to allow PCD configuration changes
                that demand that PCD will not be used while changes take place.

                May be used for Rx and offline parsing ports which are
                in PCD mode only

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_ATTACH_PCD _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(23))

/**************************************************************************//**
 @Function      FM_PORT_DetachPCD

 @Description   Calling this routine detaches the port from its PCD functionality.
                The port returns to its default flow which is BMI to BMI.

                May be used for Rx and offline parsing ports which are
                in PCD mode only

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#define FM_PORT_IOC_DETACH_PCD _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(22))

/**************************************************************************//**
 @Function      FM_PORT_PcdPlcrAllocProfiles

 @Description   This routine may be called only for ports that use the Policer in
                order to allocate private policer profiles.

 @Param[in]     uint16_t       The number of required policer profiles

 @Return        0 on success; error code otherwise.

 @Cautions      Allowed before FM_PORT_SetPCD() only.
*//***************************************************************************/
#define FM_PORT_IOC_PCD_PLCR_ALLOC_PROFILES     _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(24), uint16_t)

/**************************************************************************//**
 @Function      FM_PORT_PcdPlcrFreeProfiles

 @Description   This routine should be called for freeing private policer profiles.

 @Return        0 on success; error code otherwise.

 @Cautions      Allowed before FM_PORT_SetPCD() only.
*//***************************************************************************/
#define FM_PORT_IOC_PCD_PLCR_FREE_PROFILES     _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(25))

/**************************************************************************//**
 @Function      FM_PORT_PcdKgModifyInitialScheme

 @Description   This routine may be called only for ports that use the keygen in
                order to change the initial scheme frame should be routed to.
                The change may be of a scheme id (in case of direct mode),
                from direct to indirect, or from indirect to direct - specifying the scheme id.

 @Param[in]     ioc_fm_pcd_kg_scheme_select_t   A structure of parameters for defining whether
                                                a scheme is direct/indirect, and if direct - scheme id.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(26), ioc_compat_fm_pcd_kg_scheme_select_t)
#endif
#define FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(26), ioc_fm_pcd_kg_scheme_select_t)

/**************************************************************************//**
 @Function      FM_PORT_PcdPlcrModifyInitialProfile

 @Description   This routine may be called for ports with flows
                e_IOC_FM_PCD_SUPPORT_PLCR_ONLY or e_IOC_FM_PCD_SUPPORT_PRS_AND_PLCR  only,
                to change the initial Policer profile frame should be routed to.
                The change may be of a profile and/or absolute/direct mode selection.

 @Param[in]     ioc_fm_obj_t       Policer profile Id as returned from FM_PCD_PlcrSetProfile.

 @Return        0 on success; error code otherwise.
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(27), ioc_compat_fm_obj_t)
#endif
#define FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(27), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PORT_PcdCcModifyTree

 @Description   This routine may be called to change this port connection to
                a pre-initializes coarse classification Tree.

 @Param[in]     ioc_fm_obj_t    Id of new coarse classification tree selected for this port.

 @Return        0 on success; error code otherwise.

 @Cautions      Allowed only following FM_PORT_SetPCD() and FM_PORT_DetachPCD()
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_PCD_CC_MODIFY_TREE_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(28), ioc_compat_fm_obj_t)
#endif
#define FM_PORT_IOC_PCD_CC_MODIFY_TREE _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(28), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PORT_PcdKgBindSchemes

 @Description   These routines may be called for modifying the binding of ports
                to schemes. The scheme itself is not added,
                just this specific port starts using it.

 @Param[in]     ioc_fm_pcd_port_schemes_params_t    Schemes parameters structre

 @Return        0 on success; error code otherwise.

 @Cautions      Allowed only following FM_PORT_SetPCD().
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_PCD_KG_BIND_SCHEMES_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(30), ioc_compat_fm_pcd_port_schemes_params_t)
#endif
#define FM_PORT_IOC_PCD_KG_BIND_SCHEMES _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(30), ioc_fm_pcd_port_schemes_params_t)

/**************************************************************************//**
 @Function      FM_PORT_PcdKgUnbindSchemes

 @Description   These routines may be called for modifying the binding of ports
                to schemes. The scheme itself is not removed or invalidated,
                just this specific port stops using it.

 @Param[in]     ioc_fm_pcd_port_schemes_params_t    Schemes parameters structre

 @Return        0 on success; error code otherwise.

 @Cautions      Allowed only following FM_PORT_SetPCD().
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(31), ioc_compat_fm_pcd_port_schemes_params_t)
#endif
#define FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(31), ioc_fm_pcd_port_schemes_params_t)

typedef struct ioc_fm_port_mac_addr_params_t {
    uint8_t addr[ENET_NUM_OCTETS_PER_ADDRESS];
} ioc_fm_port_mac_addr_params_t;

/**************************************************************************//**
 @Function      FM_MAC_AddHashMacAddr

 @Description   Add an Address to the hash table. This is for filter purpose only.

 @Param[in]     ioc_fm_port_mac_addr_params_t - Ethernet Mac address

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_MAC_Init(). It is a filter only address.
 @Cautions      Some address need to be filtered out in upper FM blocks.
*//***************************************************************************/
#define FM_PORT_IOC_ADD_RX_HASH_MAC_ADDR   _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(36), ioc_fm_port_mac_addr_params_t)

/**************************************************************************//**
 @Function      FM_MAC_RemoveHashMacAddr

 @Description   Delete an Address to the hash table. This is for filter purpose only.

 @Param[in]     ioc_fm_port_mac_addr_params_t - Ethernet Mac address

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_MAC_Init().
*//***************************************************************************/
#define FM_PORT_IOC_REMOVE_RX_HASH_MAC_ADDR   _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(37), ioc_fm_port_mac_addr_params_t)

typedef struct ioc_fm_port_tx_pause_frames_params_t {
    uint8_t  priority;
    uint16_t pause_time;
    uint16_t thresh_time;
} ioc_fm_port_tx_pause_frames_params_t;

/**************************************************************************//**
 @Function      FM_MAC_SetTxPauseFrames

 @Description   Enable/Disable transmission of Pause-Frames.
                The routine changes the default configuration:
                pause-time - [0xf000]
                threshold-time - [0]

 @Param[in]     ioc_fm_port_tx_pause_frames_params_t A structure holding the required parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_MAC_Init().
                PFC is supported only on new mEMAC; i.e. in MACs that don't have
                PFC support (10G-MAC and dTSEC), user should use 'FM_MAC_NO_PFC'
                in the 'priority' field.
*//***************************************************************************/
#define FM_PORT_IOC_SET_TX_PAUSE_FRAMES       _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(40), ioc_fm_port_tx_pause_frames_params_t)

typedef struct ioc_fm_port_mac_statistics_t {
    /* RMON */
        uint64_t  e_stat_pkts_64;            /**< r-10G tr-DT 64 byte frame counter */
        uint64_t  e_stat_pkts_65_to_127;     /**< r-10G 65 to 127 byte frame counter */
        uint64_t  e_stat_pkts_128_to_255;    /**< r-10G 128 to 255 byte frame counter */
        uint64_t  e_stat_pkts_256_to_511;    /**< r-10G 256 to 511 byte frame counter */
        uint64_t  e_stat_pkts_512_to_1023;   /**< r-10G 512 to 1023 byte frame counter */
        uint64_t  e_stat_pkts_1024_to_1518;  /**< r-10G 1024 to 1518 byte frame counter */
        uint64_t  e_stat_pkts_1519_to_1522;  /**< r-10G 1519 to 1522 byte good frame count */
    /* */
        uint64_t  e_stat_fragments;          /**< Total number of packets that were less than 64 octets long with a wrong CRC.*/
        uint64_t  e_stat_jabbers;            /**< Total number of packets longer than valid maximum length octets */
        uint64_t  e_stat_drop_events;        /**< number of dropped packets due to internal errors of the MAC Client (during recieve). */
        uint64_t  e_stat_CRC_align_errors;   /**< Incremented when frames of correct length but with CRC error are received.*/
        uint64_t  e_stat_undersize_pkts;     /**< Incremented for frames under 64 bytes with a valid FCS and otherwise well formed;
                                                This count does not include range length errors */
        uint64_t  e_stat_oversize_pkts;      /**< Incremented for frames which exceed 1518 (non VLAN) or 1522 (VLAN) and contains
                                                a valid FCS and otherwise well formed */
    /* Pause */
        uint64_t  te_stat_pause;             /**< Pause MAC Control received */
        uint64_t  re_stat_pause;             /**< Pause MAC Control sent */
    /* MIB II */
        uint64_t  if_in_octets;              /**< Total number of byte received. */
        uint64_t  if_in_pkts;                /**< Total number of packets received.*/
        uint64_t  if_in_ucast_pkts;          /**< Total number of unicast frame received;
                                             NOTE: this counter is not supported on dTSEC MAC */
        uint64_t  if_in_mcast_pkts;          /**< Total number of multicast frame received*/
        uint64_t  if_in_bcast_pkts;          /**< Total number of broadcast frame received */
        uint64_t  if_in_discards;            /**< Frames received, but discarded due to problems within the MAC RX. */
        uint64_t  if_in_errors;              /**< Number of frames received with error:
                                                   - FIFO Overflow Error
                                                   - CRC Error
                                                   - Frame Too Long Error
                                                   - Alignment Error
                                                   - The dedicated Error Code (0xfe, not a code error) was received */
        uint64_t  if_out_octets;             /**< Total number of byte sent. */
        uint64_t  if_out_pkts;               /**< Total number of packets sent .*/
        uint64_t  if_out_ucast_pkts;         /**< Total number of unicast frame sent;
                                             NOTE: this counter is not supported on dTSEC MAC */
        uint64_t  if_out_mcast_pkts;         /**< Total number of multicast frame sent */
        uint64_t  if_out_bcast_pkts;         /**< Total number of multicast frame sent */
        uint64_t  if_out_discards;           /**< Frames received, but discarded due to problems within the MAC TX N/A!.*/
        uint64_t  if_out_errors;             /**< Number of frames transmitted with error:
                                                   - FIFO Overflow Error
                                                   - FIFO Underflow Error
                                                   - Other */
} ioc_fm_port_mac_statistics_t;

/**************************************************************************//**
 @Function      FM_MAC_GetStatistics

 @Description   get all MAC statistics counters

 @Param[out]    ioc_fm_port_mac_statistics_t    A structure holding the statistics

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_PORT_IOC_GET_MAC_STATISTICS        _IOR(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(41), ioc_fm_port_mac_statistics_t)

/**************************************************************************//**
 @Function      FM_PORT_ConfigBufferPrefixContent

 @Description   Defines the structure, size and content of the application buffer.
                The prefix will
                In Tx ports, if 'passPrsResult', the application
                should set a value to their offsets in the prefix of
                the FM will save the first 'privDataSize', than,
                depending on 'passPrsResult' and 'passTimeStamp', copy parse result
                and timeStamp, and the packet itself (in this order), to the
                application buffer, and to offset.
                Calling this routine changes the buffer margins definitions
                in the internal driver data base from its default
                configuration: Data size:  [DEFAULT_FM_SP_bufferPrefixContent_privDataSize]
                               Pass Parser result: [DEFAULT_FM_SP_bufferPrefixContent_passPrsResult].
                               Pass timestamp: [DEFAULT_FM_SP_bufferPrefixContent_passTimeStamp].

                May be used for all ports

 @Param[in]     ioc_fm_buffer_prefix_content_t  A structure holding the required parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Config() and before FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_CONFIG_BUFFER_PREFIX_CONTENT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(39), ioc_fm_buffer_prefix_content_t)

#if (DPAA_VERSION >= 11)
typedef struct ioc_fm_port_vsp_alloc_params_t {
    uint8_t     num_of_profiles;          /**< Number of Virtual Storage Profiles */
    uint8_t     dflt_relative_id;         /**< The default Virtual-Storage-Profile-id dedicated to Rx/OP port
                                             The same default Virtual-Storage-Profile-id will be for coupled Tx port
                                             if relevant function called for Rx port */
    void    *p_fm_tx_port;             /**< Handle to coupled Tx Port; not relevant for OP port. */
}ioc_fm_port_vsp_alloc_params_t;

/**************************************************************************//**
 @Function      FM_PORT_VSPAlloc

 @Description   This routine allocated VSPs per port and forces the port to work
                in VSP mode. Note that the port is initialized by default with the
                physical-storage-profile only.

 @Param[in]     h_FmPort    A handle to a FM Port module.
 @Param[in]     p_Params    A structure of parameters for allocation VSP's per port

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init(), and before FM_PORT_SetPCD()
                and also before FM_PORT_Enable() (i.e. the port should be disabled).
*//***************************************************************************/
#if defined(FM_COMPAT)
#define FM_PORT_IOC_VSP_ALLOC_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(38), ioc_compat_fm_port_vsp_alloc_params_t)
#endif
#define FM_PORT_IOC_VSP_ALLOC _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(38), ioc_fm_port_vsp_alloc_params_t)
#endif /* (DPAA_VERSION >= 11) */

/**************************************************************************//**
 @Function      FM_PORT_GetBmiCounters

 @Description   Read port's BMI stat counters and place them into
                a designated structure of counters.

 @Param[in]     h_FmPort    A handle to a FM Port module.
 @Param[out]    p_BmiStats  counters structure

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/

#define FM_PORT_IOC_GET_BMI_COUNTERS _IOR(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(42), ioc_fm_port_bmi_stats_t)

typedef struct ioc_fm_port_mac_frame_size_counters_t {

        e_CommMode type;
        uint64_t  count_pkts_64;            /**< 64 byte frame counter */
        uint64_t  count_pkts_65_to_127;     /**< 65 to 127 byte frame counter */
        uint64_t  count_pkts_128_to_255;    /**< 128 to 255 byte frame counter */
        uint64_t  count_pkts_256_to_511;    /**< 256 to 511 byte frame counter */
        uint64_t  count_pkts_512_to_1023;   /**< 512 to 1023 byte frame counter */
        uint64_t  count_pkts_1024_to_1518;  /**< 1024 to 1518 byte frame counter */
        uint64_t  count_pkts_1519_to_1522;  /**< 1519 to 1522 byte good frame count */
} ioc_fm_port_mac_frame_size_counters_t;

/**************************************************************************//**
 @Function      FM_MAC_GetFrameSizeCounters

 @Description   get MAC statistics counters for different frame size

 @Param[out]    ioc_fm_port_mac_frame_size_counters_t    A structure holding the counters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_PORT_IOC_GET_MAC_FRAME_SIZE_COUNTERS        _IOR(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(43), ioc_fm_port_mac_frame_size_counters_t)


/** @} */ /* end of lnx_ioctl_FM_PORT_pcd_runtime_control_grp group */
/** @} */ /* end of lnx_ioctl_FM_PORT_runtime_control_grp group */

/** @} */ /* end of lnx_ioctl_FM_PORT_grp group */
/** @} */ /* end of lnx_ioctl_FM_grp group */
#endif /* __FM_PORT_IOCTLS_H */
