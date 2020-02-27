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

/******************************************************************************
 @File          lnxwrp_fm_ext.h

 @Description   TODO
*//***************************************************************************/

#ifndef __LNXWRP_FM_EXT_H
#define __LNXWRP_FM_EXT_H

#include "std_ext.h"
#include "sys_ext.h"
#include "fm_ext.h"
#include "fm_muram_ext.h"
#include "fm_pcd_ext.h"
#include "fm_port_ext.h"
#include "fm_mac_ext.h"
#include "fm_rtc_ext.h"


/**************************************************************************//**
 @Group         FM_LnxKern_grp Frame Manager Linux wrapper API

 @Description   FM API functions, definitions and enums.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         FM_LnxKern_init_grp Initialization Unit

 @Description   Initialization Unit

                Initialization Flow:
                Initialization of the FM Module will be carried out by the Linux
                kernel according to the following sequence:
                a. Calling the initialization routine with no parameters.
                b. The driver will register to the Device-Tree.
                c. The Linux Device-Tree will initiate a call to the driver for
                   initialization.
                d. The driver will read the appropriate information from the Device-Tree
                e. [Optional] Calling the advance initialization routines to change
                   driver's defaults.
                f. Initialization of the device will be automatically upon using it.

 @{
*//***************************************************************************/

typedef struct t_WrpFmDevSettings
{
    t_FmParams                  param;
    t_SysObjectAdvConfigEntry   *advConfig;
} t_WrpFmDevSettings;

typedef struct t_WrpFmPcdDevSettings
{
    t_FmPcdParams               param;
    t_SysObjectAdvConfigEntry   *advConfig;
} t_WrpFmPcdDevSettings;

typedef struct t_WrpFmPortDevSettings
{
    bool                        frag_enabled;
    t_FmPortParams              param;
    t_SysObjectAdvConfigEntry   *advConfig;
} t_WrpFmPortDevSettings;

typedef struct t_WrpFmMacDevSettings
{
    t_FmMacParams               param;
    t_SysObjectAdvConfigEntry   *advConfig;
} t_WrpFmMacDevSettings;


/**************************************************************************//**
 @Function      LNXWRP_FM_Init

 @Description   Initialize the FM linux wrapper.

 @Return        A handle (descriptor) of the newly created FM Linux wrapper
                structure.
*//***************************************************************************/
t_Handle LNXWRP_FM_Init(void);

/**************************************************************************//**
 @Function      LNXWRP_FM_Free

 @Description   Free the FM linux wrapper.

 @Param[in]     h_LnxWrpFm   - A handle to the FM linux wrapper.

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error  LNXWRP_FM_Free(t_Handle h_LnxWrpFm);

/**************************************************************************//**
 @Function      LNXWRP_FM_GetMacHandle

 @Description   Get the FM-MAC LLD handle from the FM linux wrapper.

 @Param[in]     h_LnxWrpFm   - A handle to the FM linux wrapper.
 @Param[in]     fmId         - Index of the FM device to get the MAC handle from.
 @Param[in]     macId        - Index of the mac handle.

 @Return        A handle of the LLD compressor.
*//***************************************************************************/
t_Handle LNXWRP_FM_GetMacHandle(t_Handle h_LnxWrpFm, uint8_t fmId, uint8_t macId);

#ifdef CONFIG_FSL_SDK_FMAN_TEST
t_Handle LNXWRP_FM_TEST_Init(void);
t_Error  LNXWRP_FM_TEST_Free(t_Handle h_FmTestLnxWrp);
#endif /* CONFIG_FSL_SDK_FMAN_TEST */

/** @} */ /* end of FM_LnxKern_init_grp group */


/**************************************************************************//**
 @Group         FM_LnxKern_ctrl_grp Control Unit

 @Description   Control Unit

                TODO
 @{
*//***************************************************************************/

#include "lnxwrp_fsl_fman.h"

/** @} */ /* end of FM_LnxKern_ctrl_grp group */
/** @} */ /* end of FM_LnxKern_grp group */


#endif /* __LNXWRP_FM_EXT_H */
