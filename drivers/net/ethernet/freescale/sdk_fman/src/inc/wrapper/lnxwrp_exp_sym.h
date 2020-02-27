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

/*
 @File          lnxwrp_exp_sym.h
 @Description   FMan exported routines
*/

#ifndef __LNXWRP_EXP_SYM_H
#define __LNXWRP_EXP_SYM_H

#include "fm_port_ext.h"
#include "fm_pcd_ext.h"
#include "fm_mac_ext.h"


/* FMAN Port exported routines */
EXPORT_SYMBOL(FM_PORT_Disable);
EXPORT_SYMBOL(FM_PORT_Enable);
EXPORT_SYMBOL(FM_PORT_SetPCD);
EXPORT_SYMBOL(FM_PORT_DeletePCD);

/* Runtime PCD exported routines */
EXPORT_SYMBOL(FM_PCD_Enable);
EXPORT_SYMBOL(FM_PCD_Disable);
EXPORT_SYMBOL(FM_PCD_GetCounter);
EXPORT_SYMBOL(FM_PCD_PrsLoadSw);
EXPORT_SYMBOL(FM_PCD_KgSetDfltValue);
EXPORT_SYMBOL(FM_PCD_KgSetAdditionalDataAfterParsing);
EXPORT_SYMBOL(FM_PCD_SetException);
EXPORT_SYMBOL(FM_PCD_ModifyCounter);
EXPORT_SYMBOL(FM_PCD_SetPlcrStatistics);
EXPORT_SYMBOL(FM_PCD_SetPrsStatistics);
EXPORT_SYMBOL(FM_PCD_ForceIntr);
EXPORT_SYMBOL(FM_PCD_HcTxConf);

EXPORT_SYMBOL(FM_PCD_NetEnvCharacteristicsSet);
EXPORT_SYMBOL(FM_PCD_NetEnvCharacteristicsDelete);
EXPORT_SYMBOL(FM_PCD_KgSchemeSet);
EXPORT_SYMBOL(FM_PCD_KgSchemeDelete);
EXPORT_SYMBOL(FM_PCD_KgSchemeGetCounter);
EXPORT_SYMBOL(FM_PCD_KgSchemeSetCounter);
EXPORT_SYMBOL(FM_PCD_CcRootBuild);
EXPORT_SYMBOL(FM_PCD_CcRootDelete);
EXPORT_SYMBOL(FM_PCD_MatchTableSet);
EXPORT_SYMBOL(FM_PCD_MatchTableDelete);
EXPORT_SYMBOL(FM_PCD_CcRootModifyNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableModifyNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableFindNModifyNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableModifyMissNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableRemoveKey);
EXPORT_SYMBOL(FM_PCD_MatchTableFindNRemoveKey);
EXPORT_SYMBOL(FM_PCD_MatchTableAddKey);
EXPORT_SYMBOL(FM_PCD_MatchTableModifyKeyAndNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableFindNModifyKeyAndNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableModifyKey);
EXPORT_SYMBOL(FM_PCD_MatchTableFindNModifyKey);
EXPORT_SYMBOL(FM_PCD_MatchTableGetIndexedHashBucket);
EXPORT_SYMBOL(FM_PCD_MatchTableGetNextEngine);
EXPORT_SYMBOL(FM_PCD_MatchTableGetKeyCounter);
EXPORT_SYMBOL(FM_PCD_MatchTableGetKeyStatistics);
EXPORT_SYMBOL(FM_PCD_MatchTableFindNGetKeyStatistics);
EXPORT_SYMBOL(FM_PCD_MatchTableGetMissStatistics);
EXPORT_SYMBOL(FM_PCD_HashTableGetMissStatistics);
EXPORT_SYMBOL(FM_PCD_HashTableSet);
EXPORT_SYMBOL(FM_PCD_HashTableDelete);
EXPORT_SYMBOL(FM_PCD_HashTableAddKey);
EXPORT_SYMBOL(FM_PCD_HashTableRemoveKey);
EXPORT_SYMBOL(FM_PCD_HashTableModifyNextEngine);
EXPORT_SYMBOL(FM_PCD_HashTableModifyMissNextEngine);
EXPORT_SYMBOL(FM_PCD_HashTableGetMissNextEngine);
EXPORT_SYMBOL(FM_PCD_HashTableFindNGetKeyStatistics);
EXPORT_SYMBOL(FM_PCD_PlcrProfileSet);
EXPORT_SYMBOL(FM_PCD_PlcrProfileDelete);
EXPORT_SYMBOL(FM_PCD_PlcrProfileGetCounter);
EXPORT_SYMBOL(FM_PCD_PlcrProfileSetCounter);
EXPORT_SYMBOL(FM_PCD_ManipNodeSet);
EXPORT_SYMBOL(FM_PCD_ManipNodeDelete);
EXPORT_SYMBOL(FM_PCD_ManipGetStatistics);
EXPORT_SYMBOL(FM_PCD_ManipNodeReplace);
#if (DPAA_VERSION >= 11)
EXPORT_SYMBOL(FM_PCD_FrmReplicSetGroup);
EXPORT_SYMBOL(FM_PCD_FrmReplicDeleteGroup);
EXPORT_SYMBOL(FM_PCD_FrmReplicAddMember);
EXPORT_SYMBOL(FM_PCD_FrmReplicRemoveMember);
#endif /* DPAA_VERSION >= 11 */

#ifdef FM_CAPWAP_SUPPORT
EXPORT_SYMBOL(FM_PCD_StatisticsSetNode);
#endif /* FM_CAPWAP_SUPPORT */

EXPORT_SYMBOL(FM_PCD_SetAdvancedOffloadSupport);

/* FMAN MAC exported routines */
EXPORT_SYMBOL(FM_MAC_GetStatistics);

EXPORT_SYMBOL(FM_MAC_GetFrameSizeCounters);

EXPORT_SYMBOL(FM_GetSpecialOperationCoding);

#endif /* __LNXWRP_EXP_SYM_H */
