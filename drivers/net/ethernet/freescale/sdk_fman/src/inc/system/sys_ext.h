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

#ifndef __SYS_EXT_H
#define __SYS_EXT_H

#include "std_ext.h"


/**************************************************************************//**
 @Group         sys_grp     System Interfaces

 @Description   Linux system programming interfaces.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         sys_gen_grp     System General Interface

 @Description   General definitions, structures and routines of the linux
                system programming interface.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection    Macros for Advanced Configuration Requests
 @{
*//***************************************************************************/
#define SYS_MAX_ADV_CONFIG_ARGS     4
                                    /**< Maximum number of arguments in
                                         an advanced configuration entry */
/* @} */

/**************************************************************************//**
 @Description   System Object Advanced Configuration Entry

                This structure represents a single request for an advanced
                configuration call on the initialized object. An array of such
                requests may be contained in the settings structure of the
                corresponding object.

                The maximum number of arguments is limited to #SYS_MAX_ADV_CONFIG_ARGS.
*//***************************************************************************/
typedef struct t_SysObjectAdvConfigEntry
{
    void        *p_Function;    /**< Pointer to advanced configuration routine */

    uintptr_t    args[SYS_MAX_ADV_CONFIG_ARGS];
                                /**< Array of arguments for the specified routine;
                                     All arguments should be casted to uint32_t. */
} t_SysObjectAdvConfigEntry;


/** @} */ /* end of sys_gen_grp */
/** @} */ /* end of sys_grp */

#define NCSW_PARAMS(_num, _params)   ADV_CONFIG_PARAMS_##_num _params

#define ADV_CONFIG_PARAMS_1(_type) \
    , (_type)p_Entry->args[0]

#define SET_ADV_CONFIG_ARGS_1(_arg0)        \
    p_Entry->args[0] = (uintptr_t )(_arg0);   \

#define ARGS(_num, _params) SET_ADV_CONFIG_ARGS_##_num _params

#define ADD_ADV_CONFIG_START(_p_Entries, _maxEntries)           \
    {                                                           \
        t_SysObjectAdvConfigEntry   *p_Entry;                   \
        t_SysObjectAdvConfigEntry   *p_Entrys = (_p_Entries);   \
        int                         i=0, max = (_maxEntries);   \

#define ADD_ADV_CONFIG_END \
    }

#define ADV_CONFIG_CHECK_START(_p_Entry)                        \
    {                                                           \
        t_SysObjectAdvConfigEntry   *p_Entry = _p_Entry;        \
        t_Error                     errCode;                    \

#define ADV_CONFIG_CHECK(_handle, _func, _params)               \
        if (p_Entry->p_Function == _func)                       \
        {                                                       \
            errCode = _func(_handle _params);                   \
        } else

#endif /* __SYS_EXT_H */
