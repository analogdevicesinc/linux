/*
 * Copyright 2017 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


///////////////////////////////////////////////////////////////////////////////
//
// vpu_regdef.h
//
// Description: 
//
// Register definition
//
// Authors:
//
///////////////////////////////////////////////////////////////////////////////

#ifndef __VPU_REGDEF_H__
#define __VPU_REGDEF_H__

#define SCB_XREG_SLV_BASE                               0x00000000
#define SCB_SCB_BLK_CTRL                                0x00070000
#define SCB_BLK_CTRL_XMEM_RESET_SET                     0x00000090
#define SCB_BLK_CTRL_CACHE_RESET_SET                    0x000000A0
#define SCB_BLK_CTRL_CACHE_RESET_CLR                    0x000000A4
#define SCB_BLK_CTRL_SCB_CLK_ENABLE_SET                 0x00000100

#define XMEM_CONTROL                                    0x00041000

#define DEC_MFD_XREG_SLV_BASE                           0x00180000

#define MFD_HIF                                         0x0001C000
#define MFD_HIF_MSD_REG_HOST_INTERRUPT_ENABLE           0x00000014
#define MFD_HIF_MSD_REG_INTERRUPT_STATUS                0x00000018
#define MFD_HIF_MSD_REG_FAST_INTERRUPT_ENABLE           0x0000001C
#define MFD_SIF                                         0x0001D000
#define MFD_SIF_CTRL_STATUS                             0x000000F0
#define MFD_SIF_INTR_STATUS                             0x000000F4
#define MFD_SIF_INTR_FORCE                              0x000000F8
#define MFD_MCX                                         0x00020800

#define MFD_BLK_CTRL                                    0x00030000
#define MFD_BLK_CTRL_MFD_SYS_RESET_SET                  0x00000000
#define MFD_BLK_CTRL_MFD_SYS_RESET_CLR                  0x00000004
#define MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET           0x00000100
#define MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_CLR           0x00000104

#endif //__VPU_REGDEF_H__
