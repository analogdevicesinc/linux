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

/**************************************************************************//**
 @File          fm_ioctls.h

 @Description   FM Char device ioctls
*//***************************************************************************/
#ifndef __FM_IOCTLS_H
#define __FM_IOCTLS_H


/**************************************************************************//**
 @Group         lnx_ioctl_FM_grp Frame Manager Linux IOCTL API

 @Description   FM Linux ioctls definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection    FM IOCTL device ('/dev') definitions
*//***************************************************************************/
#define DEV_FM_NAME                 "fm" /**< Name of the FM chardev */

#define DEV_FM_MINOR_BASE           0
#define DEV_FM_PCD_MINOR_BASE       (DEV_FM_MINOR_BASE + 1)                                 /*/dev/fmx-pcd */
#define DEV_FM_OH_PORTS_MINOR_BASE  (DEV_FM_PCD_MINOR_BASE + 1)                             /*/dev/fmx-port-ohy */
#define DEV_FM_RX_PORTS_MINOR_BASE  (DEV_FM_OH_PORTS_MINOR_BASE + FM_MAX_NUM_OF_OH_PORTS)   /*/dev/fmx-port-rxy */
#define DEV_FM_TX_PORTS_MINOR_BASE  (DEV_FM_RX_PORTS_MINOR_BASE + FM_MAX_NUM_OF_RX_PORTS)   /*/dev/fmx-port-txy */
#define DEV_FM_MAX_MINORS           (DEV_FM_TX_PORTS_MINOR_BASE + FM_MAX_NUM_OF_TX_PORTS)

#define FM_IOC_NUM(n)       (n)
#define FM_PCD_IOC_NUM(n)   (n+20)
#define FM_PORT_IOC_NUM(n)  (n+70)
/* @} */

#define IOC_FM_MAX_NUM_OF_PORTS         64


/**************************************************************************//**
 @Description   Enum for defining port types
                (must match enum e_FmPortType defined in fm_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_port_type {
    e_IOC_FM_PORT_TYPE_OH_OFFLINE_PARSING = 0,  /**< Offline parsing port */
    e_IOC_FM_PORT_TYPE_RX,                      /**< 1G Rx port */
    e_IOC_FM_PORT_TYPE_RX_10G,                  /**< 10G Rx port */
    e_IOC_FM_PORT_TYPE_TX,                      /**< 1G Tx port */
    e_IOC_FM_PORT_TYPE_TX_10G,                  /**< 10G Tx port */
    e_IOC_FM_PORT_TYPE_DUMMY
} ioc_fm_port_type;


/**************************************************************************//**
 @Group         lnx_ioctl_FM_lib_grp FM library

 @Description   FM API functions, definitions and enums
                The FM module is the main driver module and is a mandatory module
                for FM driver users. Before any further module initialization,
                this module must be initialized.
                The FM is a "single-tone" module. It is responsible of the common
                HW modules: FPM, DMA, common QMI, common BMI initializations and
                run-time control routines. This module must be initialized always
                when working with any of the FM modules.
                NOTE - We assumes that the FML will be initialize only by core No. 0!

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   FM Exceptions
*//***************************************************************************/
typedef enum ioc_fm_exceptions {
    e_IOC_FM_EX_DMA_BUS_ERROR,              /**< DMA bus error. */
    e_IOC_EX_DMA_READ_ECC,               /**< Read Buffer ECC error (Valid for FM rev < 6)*/
    e_IOC_EX_DMA_SYSTEM_WRITE_ECC,       /**< Write Buffer ECC error on system side (Valid for FM rev < 6)*/
    e_IOC_EX_DMA_FM_WRITE_ECC,           /**< Write Buffer ECC error on FM side (Valid for FM rev < 6)*/
    e_IOC_EX_DMA_SINGLE_PORT_ECC,        /**< Single Port ECC error on FM side (Valid for FM rev > 6)*/
    e_IOC_EX_FPM_STALL_ON_TASKS,         /**< Stall of tasks on FPM */
    e_IOC_EX_FPM_SINGLE_ECC,             /**< Single ECC on FPM. */
    e_IOC_EX_FPM_DOUBLE_ECC,             /**< Double ECC error on FPM ram access */
    e_IOC_EX_QMI_SINGLE_ECC,             /**< Single ECC on QMI. */
    e_IOC_EX_QMI_DOUBLE_ECC,             /**< Double bit ECC occurred on QMI */
    e_IOC_EX_QMI_DEQ_FROM_UNKNOWN_PORTID,/**< Dequeue from unknown port id */
    e_IOC_EX_BMI_LIST_RAM_ECC,           /**< Linked List RAM ECC error */
    e_IOC_EX_BMI_STORAGE_PROFILE_ECC,    /**< Storage Profile ECC Error */
    e_IOC_EX_BMI_STATISTICS_RAM_ECC,     /**< Statistics Count RAM ECC Error Enable */
    e_IOC_EX_BMI_DISPATCH_RAM_ECC,       /**< Dispatch RAM ECC Error Enable */
    e_IOC_EX_IRAM_ECC,                   /**< Double bit ECC occurred on IRAM*/
    e_IOC_EX_MURAM_ECC                   /**< Double bit ECC occurred on MURAM*/
} ioc_fm_exceptions;

/**************************************************************************//**
 @Group         lnx_ioctl_FM_runtime_control_grp FM Runtime Control Unit

 @Description   FM Runtime control unit API functions, definitions and enums.
                The FM driver provides a set of control routines for each module.
                These routines may only be called after the module was fully
                initialized (both configuration and initialization routines were
                called). They are typically used to get information from hardware
                (status, counters/statistics, revision etc.), to modify a current
                state or to force/enable a required action. Run-time control may
                be called whenever necessary and as many times as needed.
 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection   General FM defines.
 *//***************************************************************************/
#define IOC_FM_MAX_NUM_OF_VALID_PORTS  (FM_MAX_NUM_OF_OH_PORTS + \
                                        FM_MAX_NUM_OF_1G_RX_PORTS +  \
                                        FM_MAX_NUM_OF_10G_RX_PORTS + \
                                        FM_MAX_NUM_OF_1G_TX_PORTS +  \
                                        FM_MAX_NUM_OF_10G_TX_PORTS)
/* @} */

/**************************************************************************//**
 @Description   Structure for Port bandwidth requirement. Port is identified
                by type and relative id.
                (must be identical to t_FmPortBandwidth defined in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_bandwidth_t {
    ioc_fm_port_type    type;           /**< FM port type */
    uint8_t             relative_port_id; /**< Type relative port id */
    uint8_t             bandwidth;      /**< bandwidth - (in term of percents) */
} ioc_fm_port_bandwidth_t;

/**************************************************************************//**
 @Description   A Structure containing an array of Port bandwidth requirements.
                The user should state the ports requiring bandwidth in terms of
                percentage - i.e. all port's bandwidths in the array must add
                up to 100.
                (must be identical to t_FmPortsBandwidthParams defined in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_bandwidth_params {
    uint8_t                     num_of_ports;
                                /**< num of ports listed in the array below */
    ioc_fm_port_bandwidth_t     ports_bandwidths[IOC_FM_MAX_NUM_OF_VALID_PORTS];
                                /**< for each port, it's bandwidth (all port's
                                  bandwidths must add up to 100.*/
} ioc_fm_port_bandwidth_params;

/**************************************************************************//**
 @Description   enum for defining FM counters
*//***************************************************************************/
typedef enum ioc_fm_counters {
    e_IOC_FM_COUNTERS_ENQ_TOTAL_FRAME,              /**< QMI total enqueued frames counter */
    e_IOC_FM_COUNTERS_DEQ_TOTAL_FRAME,              /**< QMI total dequeued frames counter */
    e_IOC_FM_COUNTERS_DEQ_0,                        /**< QMI 0 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_1,                        /**< QMI 1 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_2,                        /**< QMI 2 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_3,                        /**< QMI 3 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_FROM_DEFAULT,             /**< QMI dequeue from default queue counter */
    e_IOC_FM_COUNTERS_DEQ_FROM_CONTEXT,             /**< QMI dequeue from FQ context counter */
    e_IOC_FM_COUNTERS_DEQ_FROM_FD,                  /**< QMI dequeue from FD command field counter */
    e_IOC_FM_COUNTERS_DEQ_CONFIRM,                  /**< QMI dequeue confirm counter */
} ioc_fm_counters;

typedef struct ioc_fm_obj_t {
    void            *obj;
} ioc_fm_obj_t;

/**************************************************************************//**
 @Description   A structure for returning revision information
                (must match struct t_FmRevisionInfo declared in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_revision_info_t {
    uint8_t         major;               /**< Major revision */
    uint8_t         minor;               /**< Minor revision */
} ioc_fm_revision_info_t;

/**************************************************************************//**
 @Description   A structure for FM counters
*//***************************************************************************/
typedef struct ioc_fm_counters_params_t {
    ioc_fm_counters cnt;                /**< The requested counter */
    uint32_t        val;                /**< The requested value to get/set from/into the counter */
} ioc_fm_counters_params_t;

typedef union ioc_fm_api_version_t {
    struct {
        uint8_t major;
        uint8_t minor;
        uint8_t respin;
        uint8_t reserved;
    } version;
    uint32_t ver;
} ioc_fm_api_version_t;

#if (DPAA_VERSION >= 11)
/**************************************************************************//**
 @Description   A structure of information about each of the external
                buffer pools used by a port or storage-profile.
                (must be identical to t_FmExtPoolParams defined in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_ext_pool_params {
    uint8_t                 id;     /**< External buffer pool id */
    uint16_t                size;   /**< External buffer pool buffer size */
} ioc_fm_ext_pool_params;

/**************************************************************************//**
 @Description   A structure for informing the driver about the external
                buffer pools allocated in the BM and used by a port or a
                storage-profile.
                (must be identical to t_FmExtPools defined in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_ext_pools {
    uint8_t                 num_of_pools_used;     /**< Number of pools use by this port */
    ioc_fm_ext_pool_params  ext_buf_pool[FM_PORT_MAX_NUM_OF_EXT_POOLS];
                                                /**< Parameters for each port */
} ioc_fm_ext_pools;

typedef struct ioc_fm_vsp_params_t {
    void                *p_fm;              /**< A handle to the FM object this VSP related to */
    ioc_fm_ext_pools    ext_buf_pools;        /**< Which external buffer pools are used
                                                 (up to FM_PORT_MAX_NUM_OF_EXT_POOLS), and their sizes.
                                                 parameter associated with Rx / OP port */
    uint16_t            liodn_offset;        /**< VSP's LIODN offset */
    struct {
        ioc_fm_port_type port_type;          /**< Port type */
        uint8_t         port_id;             /**< Port Id - relative to type */
    } port_params;
    uint8_t             relative_profile_id;  /**< VSP Id - relative to VSP's range
                                                 defined in relevant FM object */
    void                *id;                /**< return value */
} ioc_fm_vsp_params_t;
#endif /* (DPAA_VERSION >= 11) */

/**************************************************************************//**
 @Description   A structure for defining BM pool depletion criteria
*//***************************************************************************/
typedef struct ioc_fm_buf_pool_depletion_t {
    bool        pools_grp_mode_enable;              /**< select mode in which pause frames will be sent after
                                                         a number of pools (all together!) are depleted */
    uint8_t     num_of_pools;                       /**< the number of depleted pools that will invoke
                                                         pause frames transmission. */
    bool        pools_to_consider[BM_MAX_NUM_OF_POOLS];
                                                    /**< For each pool, TRUE if it should be considered for
                                                         depletion (Note - this pool must be used by this port!). */
    bool        single_pool_mode_enable;            /**< select mode in which pause frames will be sent after
                                                         a single-pool is depleted; */
    bool        pools_to_consider_for_single_mode[BM_MAX_NUM_OF_POOLS];
                                                    /**< For each pool, TRUE if it should be considered for
                                                         depletion (Note - this pool must be used by this port!) */
#if (DPAA_VERSION >= 11)
    bool        pfc_priorities_en[FM_MAX_NUM_OF_PFC_PRIORITIES];
                                                    /**< This field is used by the MAC as the Priority Enable Vector in the PFC frame
                                                         which is transmitted */
#endif /* (DPAA_VERSION >= 11) */
} ioc_fm_buf_pool_depletion_t;

#if (DPAA_VERSION >= 11)
typedef struct ioc_fm_buf_pool_depletion_params_t {
    void        *p_fm_vsp;
    ioc_fm_buf_pool_depletion_t fm_buf_pool_depletion;
} ioc_fm_buf_pool_depletion_params_t;
#endif /* (DPAA_VERSION >= 11) */

typedef struct ioc_fm_buffer_prefix_content_t {
    uint16_t    priv_data_size;       /**< Number of bytes to be left at the beginning
                                         of the external buffer; Note that the private-area will
                                         start from the base of the buffer address. */
    bool        pass_prs_result;      /**< TRUE to pass the parse result to/from the FM;
                                         User may use FM_PORT_GetBufferPrsResult() in order to
                                         get the parser-result from a buffer. */
    bool        pass_time_stamp;      /**< TRUE to pass the timeStamp to/from the FM
                                         User may use FM_PORT_GetBufferTimeStamp() in order to
                                         get the parser-result from a buffer. */
    bool        pass_hash_result;     /**< TRUE to pass the KG hash result to/from the FM
                                         User may use FM_PORT_GetBufferHashResult() in order to
                                         get the parser-result from a buffer. */
    bool        pass_all_other_pcd_info; /**< Add all other Internal-Context information:
                                         AD, hash-result, key, etc. */
    uint16_t    data_align;          /**< 0 to use driver's default alignment [64],
                                         other value for selecting a data alignment (must be a power of 2);
                                         if write optimization is used, must be >= 16. */
    uint8_t     manip_extra_space;    /**< Maximum extra size needed (insertion-size minus removal-size);
                                         Note that this field impacts the size of the buffer-prefix
                                         (i.e. it pushes the data offset);
                                         This field is irrelevant if DPAA_VERSION==10 */
} ioc_fm_buffer_prefix_content_t;

typedef struct ioc_fm_buffer_prefix_content_params_t {
    void        *p_fm_vsp;
    ioc_fm_buffer_prefix_content_t fm_buffer_prefix_content;
} ioc_fm_buffer_prefix_content_params_t;

#if (DPAA_VERSION >= 11)
typedef struct ioc_fm_vsp_config_no_sg_params_t {
    void        *p_fm_vsp;
    bool        no_sg;
} ioc_fm_vsp_config_no_sg_params_t;

typedef struct ioc_fm_vsp_prs_result_params_t {
    void        *p_fm_vsp;
    void        *p_data;
} ioc_fm_vsp_prs_result_params_t;
#endif

typedef struct fm_ctrl_mon_t {
    uint8_t     percent_cnt[2];
} fm_ctrl_mon_t;

typedef struct ioc_fm_ctrl_mon_counters_params_t {
    uint8_t     fm_ctrl_index;
    fm_ctrl_mon_t *p_mon;
} ioc_fm_ctrl_mon_counters_params_t;

/**************************************************************************//**
 @Function      FM_IOC_SET_PORTS_BANDWIDTH

 @Description   Sets relative weights between ports when accessing common resources.

 @Param[in]     ioc_fm_port_bandwidth_params    Port bandwidth percentages,
 their sum must equal 100.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_SET_PORTS_BANDWIDTH                             _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(2), ioc_fm_port_bandwidth_params)

/**************************************************************************//**
 @Function      FM_IOC_GET_REVISION

 @Description   Returns the FM revision

 @Param[out]    ioc_fm_revision_info_t  A structure of revision information parameters.

 @Return        None.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_GET_REVISION                                    _IOR(FM_IOC_TYPE_BASE, FM_IOC_NUM(3), ioc_fm_revision_info_t)

/**************************************************************************//**
 @Function      FM_IOC_GET_COUNTER

 @Description   Reads one of the FM counters.

 @Param[in,out] ioc_fm_counters_params_t The requested counter parameters.

 @Return        Counter's current value.

 @Cautions      Allowed only following FM_Init().
                Note that it is user's responsibilty to call this routine only
                for enabled counters, and there will be no indication if a
                disabled counter is accessed.
*//***************************************************************************/
#define FM_IOC_GET_COUNTER                                    _IOWR(FM_IOC_TYPE_BASE, FM_IOC_NUM(4), ioc_fm_counters_params_t)

/**************************************************************************//**
 @Function      FM_IOC_SET_COUNTER

 @Description   Sets a value to an enabled counter. Use "0" to reset the counter.

 @Param[in]     ioc_fm_counters_params_t The requested counter parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_SET_COUNTER                                    _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(5), ioc_fm_counters_params_t)

/**************************************************************************//**
 @Function      FM_IOC_FORCE_INTR

 @Description   Causes an interrupt event on the requested source.

 @Param[in]     ioc_fm_exceptions   An exception to be forced.

 @Return        E_OK on success; Error code if the exception is not enabled,
                or is not able to create interrupt.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_FORCE_INTR                                    _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(6), ioc_fm_exceptions)

/**************************************************************************//**
 @Function      FM_IOC_GET_API_VERSION

 @Description   Reads the FMD IOCTL API version.

 @Param[in,out] ioc_fm_api_version_t The requested counter parameters.

 @Return        Version's value.
*//***************************************************************************/
#define FM_IOC_GET_API_VERSION                               _IOR(FM_IOC_TYPE_BASE, FM_IOC_NUM(7), ioc_fm_api_version_t)

#if (DPAA_VERSION >= 11)
/**************************************************************************//**
 @Function      FM_VSP_Config

 @Description   Creates descriptor for the FM VSP module.

                The routine returns a handle (descriptor) to the FM VSP object.
                This descriptor must be passed as first parameter to all other
                FM VSP function calls.

                No actual initialization or configuration of FM hardware is
                done by this routine.

@Param[in]      p_FmVspParams   Pointer to data structure of parameters

 @Retval        Handle to FM VSP object, or NULL for Failure.
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_CONFIG_COMPAT                             _IOWR(FM_IOC_TYPE_BASE, FM_IOC_NUM(8), ioc_compat_fm_vsp_params_t)
#endif
#define FM_IOC_VSP_CONFIG                                    _IOWR(FM_IOC_TYPE_BASE, FM_IOC_NUM(8), ioc_fm_vsp_params_t)

/**************************************************************************//**
 @Function      FM_VSP_Init

 @Description   Initializes the FM VSP module

 @Param[in]     h_FmVsp - FM VSP module descriptor

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_INIT_COMPAT                               _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(9), ioc_compat_fm_obj_t)
#endif
#define FM_IOC_VSP_INIT                                      _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(9), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_VSP_Free

 @Description   Frees all resources that were assigned to FM VSP module.

                Calling this routine invalidates the descriptor.

 @Param[in]     h_FmVsp - FM VSP module descriptor

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_FREE_COMPAT                               _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(10), ioc_compat_fm_obj_t)
#endif
#define FM_IOC_VSP_FREE                                      _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(10), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_VSP_ConfigPoolDepletion

 @Description   Calling this routine enables pause frame generation depending on the
                depletion status of BM pools. It also defines the conditions to activate
                this functionality. By default, this functionality is disabled.

 @Param[in]     ioc_fm_buf_pool_depletion_params_t      A structure holding the required parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_VSP_Config() and before FM_VSP_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_CONFIG_POOL_DEPLETION_COMPAT              _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(11), ioc_compat_fm_buf_pool_depletion_params_t)
#endif
#define FM_IOC_VSP_CONFIG_POOL_DEPLETION                     _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(11), ioc_fm_buf_pool_depletion_params_t)

/**************************************************************************//**
 @Function      FM_VSP_ConfigBufferPrefixContent

 @Description   Defines the structure, size and content of the application buffer.

                The prefix will
                In VSPs defined for Tx ports, if 'passPrsResult', the application
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

 @Param[in]     ioc_fm_buffer_prefix_content_params_t   A structure holding the required parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_VSP_Config() and before FM_VSP_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_CONFIG_BUFFER_PREFIX_CONTENT_COMPAT       _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(12), ioc_compat_fm_buffer_prefix_content_params_t)
#endif
#define FM_IOC_VSP_CONFIG_BUFFER_PREFIX_CONTENT              _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(12), ioc_fm_buffer_prefix_content_params_t)

/**************************************************************************//**
 @Function      FM_VSP_ConfigNoScatherGather

 @Description   Calling this routine changes the possibility to receive S/G frame
                in the internal driver data base
                from its default configuration: optimize = [DEFAULT_FM_SP_noScatherGather]

 @Param[in]     ioc_fm_vsp_config_no_sg_params_t        A structure holding the required parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_VSP_Config() and before FM_VSP_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_CONFIG_NO_SG_COMPAT                     _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(13), ioc_compat_fm_vsp_config_no_sg_params_t)
#endif
#define FM_IOC_VSP_CONFIG_NO_SG                            _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(13), ioc_fm_vsp_config_no_sg_params_t)

/**************************************************************************//**
 @Function      FM_VSP_GetBufferPrsResult

 @Description   Returns the pointer to the parse result in the data buffer.
                In Rx ports this is relevant after reception, if parse
                result is configured to be part of the data passed to the
                application. For non Rx ports it may be used to get the pointer
                of the area in the buffer where parse result should be
                initialized - if so configured.
                See FM_VSP_ConfigBufferPrefixContent for data buffer prefix
                configuration.

 @Param[in]     ioc_fm_vsp_prs_result_params_t  A structure holding the required parameters.

 @Return        Parse result pointer on success, NULL if parse result was not
                configured for this port.

 @Cautions      Allowed only following FM_VSP_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_VSP_GET_BUFFER_PRS_RESULT_COMPAT            _IOWR(FM_IOC_TYPE_BASE, FM_IOC_NUM(14), ioc_compat_fm_vsp_prs_result_params_t)
#endif
#define FM_IOC_VSP_GET_BUFFER_PRS_RESULT                   _IOWR(FM_IOC_TYPE_BASE, FM_IOC_NUM(14), ioc_fm_vsp_prs_result_params_t)
#endif /* (DPAA_VERSION >= 11) */

/**************************************************************************//**
 @Function      FM_CtrlMonStart

 @Description   Start monitoring utilization of all available FM controllers.

                In order to obtain FM controllers utilization the following sequence
                should be used:
                -# FM_CtrlMonStart()
                -# FM_CtrlMonStop()
                -# FM_CtrlMonGetCounters() - issued for each FM controller

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_CTRL_MON_START                              _IO(FM_IOC_TYPE_BASE, FM_IOC_NUM(15))


/**************************************************************************//**
 @Function      FM_CtrlMonStop

 @Description   Stop monitoring utilization of all available FM controllers.

                In order to obtain FM controllers utilization the following sequence
                should be used:
                -# FM_CtrlMonStart()
                -# FM_CtrlMonStop()
                -# FM_CtrlMonGetCounters() - issued for each FM controller

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_CTRL_MON_STOP                               _IO(FM_IOC_TYPE_BASE, FM_IOC_NUM(16))

/**************************************************************************//**
 @Function      FM_CtrlMonGetCounters

 @Description   Obtain FM controller utilization parameters.

                In order to obtain FM controllers utilization the following sequence
                should be used:
                -# FM_CtrlMonStart()
                -# FM_CtrlMonStop()
                -# FM_CtrlMonGetCounters() - issued for each FM controller

 @Param[in]     ioc_fm_ctrl_mon_counters_params_t       A structure holding the required parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_IOC_CTRL_MON_GET_COUNTERS_COMPAT                _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(17), ioc_compat_fm_ctrl_mon_counters_params_t)
#endif
#define FM_IOC_CTRL_MON_GET_COUNTERS                       _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(17), ioc_fm_ctrl_mon_counters_params_t)

/** @} */ /* end of lnx_ioctl_FM_runtime_control_grp group */
/** @} */ /* end of lnx_ioctl_FM_lib_grp group */
/** @} */ /* end of lnx_ioctl_FM_grp */

#define FMD_API_VERSION_MAJOR 21
#define FMD_API_VERSION_MINOR 1 
#define FMD_API_VERSION_RESPIN 0

#endif /* __FM_IOCTLS_H */
