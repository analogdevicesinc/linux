/* Copyright (c) 2008-2012 Freescale Semiconductor, Inc
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
 @File          part_integration_ext.h

 @Description   P1023 external definitions and structures.
*//***************************************************************************/
#ifndef __PART_INTEGRATION_EXT_H
#define __PART_INTEGRATION_EXT_H

#include "std_ext.h"
#include "dpaa_integration_ext.h"


/**************************************************************************//**
 @Group         1023_chip_id P1023 Application Programming Interface

 @Description   P1023 Chip functions,definitions and enums.

 @{
*//***************************************************************************/

#define INTG_MAX_NUM_OF_CORES   2


/**************************************************************************//**
 @Description   Module types.
*//***************************************************************************/
typedef enum e_ModuleId
{
    e_MODULE_ID_LAW,            /**< Local Access module                     */
    e_MODULE_ID_ECM,            /**< e500 Coherency Module                   */
    e_MODULE_ID_DDR,            /**< DDR memory controller                   */
    e_MODULE_ID_I2C_1,          /**< I2C 1                                   */
    e_MODULE_ID_I2C_2,          /**< I2C 1                                   */
    e_MODULE_ID_DUART_1,        /**< DUART module 1                          */
    e_MODULE_ID_DUART_2,        /**< DUART module 2                          */
    e_MODULE_ID_LBC,            /**< Local bus memory controller module      */
    e_MODULE_ID_PCIE_1,         /**< PCI Express 1 controller module         */
    e_MODULE_ID_PCIE_ATMU_1,    /**< PCI 1 ATMU Window                       */
    e_MODULE_ID_PCIE_2,         /**< PCI Express 2 controller module         */
    e_MODULE_ID_PCIE_ATMU_2,    /**< PCI 2 ATMU Window                       */
    e_MODULE_ID_PCIE_3,         /**< PCI Express 3 controller module         */
    e_MODULE_ID_PCIE_ATMU_3,    /**< PCI 3 ATMU Window                       */
    e_MODULE_ID_MSI,            /**< MSI registers                           */
    e_MODULE_ID_L2_SRAM,        /**< L2/SRAM Memory-Mapped controller module */
    e_MODULE_ID_DMA_1,          /**< DMA controller 1                        */
    e_MODULE_ID_DMA_2,          /**< DMA controller 2                        */
    e_MODULE_ID_EPIC,           /**< Programmable interrupt controller       */
    e_MODULE_ID_ESPI,           /**< ESPI module                             */
    e_MODULE_ID_GPIO,           /**< General Purpose I/O                     */
    e_MODULE_ID_SEC_GEN,        /**< SEC 4.0 General registers               */
    e_MODULE_ID_SEC_QI,         /**< SEC 4.0 QI registers                    */
    e_MODULE_ID_SEC_JQ0,        /**< SEC 4.0 JQ-0 registers                  */
    e_MODULE_ID_SEC_JQ1,        /**< SEC 4.0 JQ-1 registers                  */
    e_MODULE_ID_SEC_JQ2,        /**< SEC 4.0 JQ-2 registers                  */
    e_MODULE_ID_SEC_JQ3,        /**< SEC 4.0 JQ-3 registers                  */
    e_MODULE_ID_SEC_RTIC,       /**< SEC 4.0 RTIC registers                  */
    e_MODULE_ID_SEC_DECO0_CCB0, /**< SEC 4.0 DECO-0/CCB-0 registers          */
    e_MODULE_ID_SEC_DECO1_CCB1, /**< SEC 4.0 DECO-1/CCB-1 registers          */
    e_MODULE_ID_SEC_DECO2_CCB2, /**< SEC 4.0 DECO-2/CCB-2 registers          */
    e_MODULE_ID_SEC_DECO3_CCB3, /**< SEC 4.0 DECO-3/CCB-3 registers          */
    e_MODULE_ID_SEC_DECO4_CCB4, /**< SEC 4.0 DECO-4/CCB-4 registers          */
    e_MODULE_ID_USB_DR_1,       /**< USB 2.0 module 1                        */
    e_MODULE_ID_USB_DR_2,       /**< USB 2.0 module 2                        */
    e_MODULE_ID_ETSEC_MII_MNG,  /**< MII MNG registers                       */
    e_MODULE_ID_ETSEC_1,        /**< ETSEC module 1                             */
    e_MODULE_ID_ETSEC_2,        /**< ETSEC module 2                             */
    e_MODULE_ID_GUTS,           /**< Serial DMA                              */
    e_MODULE_ID_PM,             /**< Performance Monitor module              */
    e_MODULE_ID_QM,                 /**< Queue manager module */
    e_MODULE_ID_BM,                 /**< Buffer manager module */
    e_MODULE_ID_QM_CE_PORTAL,
    e_MODULE_ID_QM_CI_PORTAL,
    e_MODULE_ID_BM_CE_PORTAL,
    e_MODULE_ID_BM_CI_PORTAL,
    e_MODULE_ID_FM,                /**< Frame manager #1 module */
    e_MODULE_ID_FM_RTC,            /**< FM Real-Time-Clock */
    e_MODULE_ID_FM_MURAM,          /**< FM Multi-User-RAM */
    e_MODULE_ID_FM_BMI,            /**< FM BMI block */
    e_MODULE_ID_FM_QMI,            /**< FM QMI block */
    e_MODULE_ID_FM_PRS,            /**< FM parser block */
    e_MODULE_ID_FM_PORT_HO0,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO1,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO2,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO3,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_HO4,       /**< FM Host-command/offline-parsing port block */
    e_MODULE_ID_FM_PORT_1GRx0,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GRx1,     /**< FM Rx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx0,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PORT_1GTx1,     /**< FM Tx 1G MAC port block */
    e_MODULE_ID_FM_PLCR,           /**< FM Policer */
    e_MODULE_ID_FM_KG,             /**< FM Keygen */
    e_MODULE_ID_FM_DMA,            /**< FM DMA */
    e_MODULE_ID_FM_FPM,            /**< FM FPM */
    e_MODULE_ID_FM_IRAM,           /**< FM Instruction-RAM */
    e_MODULE_ID_FM_1GMDIO0,        /**< FM 1G MDIO MAC 0*/
    e_MODULE_ID_FM_1GMDIO1,        /**< FM 1G MDIO MAC 1*/
    e_MODULE_ID_FM_PRS_IRAM,       /**< FM SW-parser Instruction-RAM */
    e_MODULE_ID_FM_RISC0,          /**< FM risc #0 */
    e_MODULE_ID_FM_RISC1,          /**< FM risc #1 */
    e_MODULE_ID_FM_1GMAC0,         /**< FM 1G MAC #0 */
    e_MODULE_ID_FM_1GMAC1,         /**< FM 1G MAC #1 */
    e_MODULE_ID_FM_MACSEC,         /**< FM MACSEC */

    e_MODULE_ID_DUMMY_LAST
} e_ModuleId;

#define NUM_OF_MODULES  e_MODULE_ID_DUMMY_LAST


#define P1023_OFFSET_LAW                    0x00000C08
#define P1023_OFFSET_ECM                    0x00001000
#define P1023_OFFSET_DDR                    0x00002000
#define P1023_OFFSET_I2C1                   0x00003000
#define P1023_OFFSET_I2C2                   0x00003100
#define P1023_OFFSET_DUART1                 0x00004500
#define P1023_OFFSET_DUART2                 0x00004600
#define P1023_OFFSET_LBC                    0x00005000
#define P1023_OFFSET_ESPI                   0x00007000
#define P1023_OFFSET_PCIE2                  0x00009000
#define P1023_OFFSET_PCIE2_ATMU             0x00009C00
#define P1023_OFFSET_PCIE1                  0x0000A000
#define P1023_OFFSET_PCIE1_ATMU             0x0000AC00
#define P1023_OFFSET_PCIE3                  0x0000B000
#define P1023_OFFSET_PCIE3_ATMU             0x0000BC00
#define P1023_OFFSET_DMA2                   0x0000C100
#define P1023_OFFSET_GPIO                   0x0000F000
#define P1023_OFFSET_L2_SRAM                0x00020000
#define P1023_OFFSET_DMA1                   0x00021100
#define P1023_OFFSET_USB1                   0x00022000
#define P1023_OFFSET_SEC_GEN                0x00030000
#define P1023_OFFSET_SEC_JQ0                0x00031000
#define P1023_OFFSET_SEC_JQ1                0x00032000
#define P1023_OFFSET_SEC_JQ2                0x00033000
#define P1023_OFFSET_SEC_JQ3                0x00034000
#define P1023_OFFSET_SEC_RTIC               0x00036000
#define P1023_OFFSET_SEC_QI                 0x00037000
#define P1023_OFFSET_SEC_DECO0_CCB0         0x00038000
#define P1023_OFFSET_SEC_DECO1_CCB1         0x00039000
#define P1023_OFFSET_SEC_DECO2_CCB2         0x0003a000
#define P1023_OFFSET_SEC_DECO3_CCB3         0x0003b000
#define P1023_OFFSET_SEC_DECO4_CCB4         0x0003c000
#define P1023_OFFSET_PIC                    0x00040000
#define P1023_OFFSET_MSI                    0x00041600
#define P1023_OFFSET_AXI                    0x00081000
#define P1023_OFFSET_QM                     0x00088000
#define P1023_OFFSET_BM                     0x0008A000
#define P1022_OFFSET_PM                     0x000E1000

#define P1023_OFFSET_GUTIL                  0x000E0000
#define P1023_OFFSET_PM                     0x000E1000
#define P1023_OFFSET_DEBUG                  0x000E2000
#define P1023_OFFSET_SERDES                 0x000E3000
#define P1023_OFFSET_ROM                    0x000F0000
#define P1023_OFFSET_FM                     0x00100000

#define P1023_OFFSET_FM_MURAM               (P1023_OFFSET_FM + 0x00000000)
#define P1023_OFFSET_FM_BMI                 (P1023_OFFSET_FM + 0x00080000)
#define P1023_OFFSET_FM_QMI                 (P1023_OFFSET_FM + 0x00080400)
#define P1023_OFFSET_FM_PRS                 (P1023_OFFSET_FM + 0x00080800)
#define P1023_OFFSET_FM_PORT_HO0            (P1023_OFFSET_FM + 0x00081000)
#define P1023_OFFSET_FM_PORT_HO1            (P1023_OFFSET_FM + 0x00082000)
#define P1023_OFFSET_FM_PORT_HO2            (P1023_OFFSET_FM + 0x00083000)
#define P1023_OFFSET_FM_PORT_HO3            (P1023_OFFSET_FM + 0x00084000)
#define P1023_OFFSET_FM_PORT_HO4            (P1023_OFFSET_FM + 0x00085000)
#define P1023_OFFSET_FM_PORT_1GRX0          (P1023_OFFSET_FM + 0x00088000)
#define P1023_OFFSET_FM_PORT_1GRX1          (P1023_OFFSET_FM + 0x00089000)
#define P1023_OFFSET_FM_PORT_1GTX0          (P1023_OFFSET_FM + 0x000A8000)
#define P1023_OFFSET_FM_PORT_1GTX1          (P1023_OFFSET_FM + 0x000A9000)
#define P1023_OFFSET_FM_PLCR                (P1023_OFFSET_FM + 0x000C0000)
#define P1023_OFFSET_FM_KG                  (P1023_OFFSET_FM + 0x000C1000)
#define P1023_OFFSET_FM_DMA                 (P1023_OFFSET_FM + 0x000C2000)
#define P1023_OFFSET_FM_FPM                 (P1023_OFFSET_FM + 0x000C3000)
#define P1023_OFFSET_FM_IRAM                (P1023_OFFSET_FM + 0x000C4000)
#define P1023_OFFSET_FM_PRS_IRAM            (P1023_OFFSET_FM + 0x000C7000)
#define P1023_OFFSET_FM_RISC0               (P1023_OFFSET_FM + 0x000D0000)
#define P1023_OFFSET_FM_RISC1               (P1023_OFFSET_FM + 0x000D0400)
#define P1023_OFFSET_FM_MACSEC              (P1023_OFFSET_FM + 0x000D8000)
#define P1023_OFFSET_FM_1GMAC0              (P1023_OFFSET_FM + 0x000E0000)
#define P1023_OFFSET_FM_1GMDIO0             (P1023_OFFSET_FM + 0x000E1120)
#define P1023_OFFSET_FM_1GMAC1              (P1023_OFFSET_FM + 0x000E2000)
#define P1023_OFFSET_FM_1GMDIO1             (P1023_OFFSET_FM + 0x000E3000)
#define P1023_OFFSET_FM_RTC                 (P1023_OFFSET_FM + 0x000FE000)

/* Offsets relative to QM or BM portals base */
#define P1023_OFFSET_PORTALS_CE_AREA        0x00000000        /* cache enabled area */
#define P1023_OFFSET_PORTALS_CI_AREA        0x00100000        /* cache inhibited area */

#define P1023_OFFSET_PORTALS_CE(portal)     (P1023_OFFSET_PORTALS_CE_AREA + 0x4000 * (portal))
#define P1023_OFFSET_PORTALS_CI(portal)     (P1023_OFFSET_PORTALS_CI_AREA + 0x1000 * (portal))

/**************************************************************************//**
 @Description   Transaction source ID (for memory controllers error reporting).
*//***************************************************************************/
typedef enum e_TransSrc
{
    e_TRANS_SRC_PCIE_2          = 0x01, /**< PCIe port 2                    */
    e_TRANS_SRC_PCIE_1          = 0x02, /**< PCIe port 1                    */
    e_TRANS_SRC_PCIE_3          = 0x03, /**< PCIe port 3                    */
    e_TRANS_SRC_LBC             = 0x04, /**< Enhanced local bus             */
    e_TRANS_SRC_DPAA_SW_PORTALS = 0x0E, /**< DPAA software portals or SRAM  */
    e_TRANS_SRC_DDR             = 0x0F, /**< DDR controller                 */
    e_TRANS_SRC_CORE_INS_FETCH  = 0x10, /**< Processor (instruction)        */
    e_TRANS_SRC_CORE_DATA       = 0x11, /**< Processor (data)               */
    e_TRANS_SRC_DMA             = 0x15  /**< DMA                            */
} e_TransSrc;

/**************************************************************************//**
 @Description   Local Access Window Target interface ID
*//***************************************************************************/
typedef enum e_P1023LawTargetId
{
    e_P1023_LAW_TARGET_PCIE_2       = 0x01, /**< PCI Express 2 target interface */
    e_P1023_LAW_TARGET_PCIE_1       = 0x02, /**< PCI Express 1 target interface */
    e_P1023_LAW_TARGET_PCIE_3       = 0x03, /**< PCI Express 3 target interface */
    e_P1023_LAW_TARGET_LBC          = 0x04, /**< Local bus target interface */
    e_P1023_LAW_TARGET_QM_PORTALS   = 0x0E, /**< Queue Manager Portals */
    e_P1023_LAW_TARGET_BM_PORTALS   = 0x0E, /**< Buffer Manager Portals */
    e_P1023_LAW_TARGET_SRAM         = 0x0E, /**< SRAM scratchpad */
    e_P1023_LAW_TARGET_DDR          = 0x0F, /**< DDR target interface */
    e_P1023_LAW_TARGET_NONE         = 0xFF  /**< Invalid target interface */
} e_P1023LawTargetId;


/**************************************************************************//**
 @Group         1023_init_grp P1023 Initialization Unit

 @Description   P1023 initialization unit API functions, definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   Part ID and revision number
*//***************************************************************************/
typedef enum e_P1023DeviceName
{
    e_P1023_REV_INVALID     = 0x00000000,       /**< Invalid revision */
    e_SC1023_REV_1_0        = (int)0x80FC0010,  /**< SC1023 rev 1.0 */
    e_SC1023_REV_1_1        = (int)0x80FC0011,  /**< SC1023 rev 1.1 */
    e_P1023_REV_1_0         = (int)0x80FE0010,  /**< P1023 rev 1.0 with security */
    e_P1023_REV_1_1         = (int)0x80FE0011,  /**< P1023 rev 1.1 with security */
    e_P1017_REV_1_1         = (int)0x80FF0011,  /**< P1017 rev 1.1 with security */
    e_P1023_REV_1_0_NO_SEC  = (int)0x80F60010,  /**< P1023 rev 1.0 without security */
    e_P1023_REV_1_1_NO_SEC  = (int)0x80F60011,  /**< P1023 rev 1.1 without security */
    e_P1017_REV_1_1_NO_SEC  = (int)0x80F70011   /**< P1017 rev 1.1 without security */
} e_P1023DeviceName;

/**************************************************************************//**
 @Description   structure representing P1023 initialization parameters
*//***************************************************************************/
typedef struct t_P1023Params
{
    uintptr_t   ccsrBaseAddress;        /**< CCSR base address (virtual) */
    uintptr_t   bmPortalsBaseAddress;   /**< Portals base address (virtual) */
    uintptr_t   qmPortalsBaseAddress;   /**< Portals base address (virtual) */
} t_P1023Params;

/**************************************************************************//**
 @Function      P1023_ConfigAndInit

 @Description   General initiation of the chip registers.

 @Param[in]     p_P1023Params  - A pointer to data structure of parameters

 @Return        A handle to the P1023 data structure.
*//***************************************************************************/
t_Handle P1023_ConfigAndInit(t_P1023Params *p_P1023Params);

/**************************************************************************//**
 @Function      P1023_Free

 @Description   Free all resources.

 @Param         h_P1023 - (In) The handle of the initialized P1023 object.

 @Return        E_OK on success; Other value otherwise.
*//***************************************************************************/
t_Error P1023_Free(t_Handle h_P1023);

/**************************************************************************//**
 @Function      P1023_GetRevInfo

 @Description   This routine enables access to chip and revision information.

 @Param[in]     gutilBase       - Base address of P1023 GUTIL registers.

 @Return        Part ID and revision.
*//***************************************************************************/
e_P1023DeviceName P1023_GetRevInfo(uintptr_t gutilBase);

/**************************************************************************//**
 @Function      P1023_GetE500Factor

 @Description   Returns E500 core clock multiplication factor.

 @Param[in]     gutilBase       - Base address of P1023 GUTIL registers.
 @Param[in]     coreId          - Id of the requested core.
 @Param[out]    p_E500MulFactor - Returns E500 to CCB multification factor.
 @Param[out]    p_E500DivFactor - Returns E500 to CCB division factor.

 @Return        E_OK on success; Other value otherwise.
*
*//***************************************************************************/
t_Error P1023_GetE500Factor(uintptr_t    gutilBase,
                            uint32_t    coreId,
                            uint32_t    *p_E500MulFactor,
                            uint32_t    *p_E500DivFactor);

/**************************************************************************//**
 @Function      P1023_GetFmFactor

 @Description   returns FM multiplication factors. (This value is returned using
                two parameters to avoid using float parameter).

 @Param[in]     gutilBase       - Base address of P1023 GUTIL registers.
 @Param[out]    p_FmMulFactor   - returns E500 to CCB multification factor.
 @Param[out]    p_FmDivFactor   - returns E500 to CCB division factor.

 @Return        E_OK on success; Other value otherwise.
*//***************************************************************************/
t_Error  P1023_GetFmFactor(uintptr_t gutilBase, uint32_t *p_FmMulFactor, uint32_t *p_FmDivFactor);

/**************************************************************************//**
 @Function      P1023_GetCcbFactor

 @Description   returns system multiplication factor.

 @Param[in]     gutilBase       - Base address of P1023 GUTIL registers.

 @Return        System multiplication factor.
*//***************************************************************************/
uint32_t P1023_GetCcbFactor(uintptr_t gutilBase);

#if 0
/**************************************************************************//**
 @Function      P1023_GetDdrFactor

 @Description   returns the multiplication factor of the clock in for the DDR clock .
                Note: assumes the ddr_in_clk is identical to the sys_in_clk

 @Param[in]     gutilBase       - Base address of P1023 GUTIL registers.
 @Param         p_DdrMulFactor  - returns DDR in clk multification factor.
 @Param         p_DdrDivFactor  - returns DDR division factor.

 @Return        E_OK on success; Other value otherwise..
*//***************************************************************************/
t_Error P1023_GetDdrFactor( uintptr_t   gutilBase,
                            uint32_t    *p_DdrMulFactor,
                            uint32_t    *p_DdrDivFactor);

/**************************************************************************//**
 @Function      P1023_GetDdrType

 @Description   returns the multiplication factor of the clock in for the DDR clock .

 @Param[in]     gutilBase       - Base address of P1023 GUTIL registers.
 @Param         p_DdrType   - (Out) returns DDR type DDR1/DDR2/DDR3.

 @Return        E_OK on success; Other value otherwise.
*//***************************************************************************/
t_Error P1023_GetDdrType(uintptr_t gutilBase, e_DdrType *p_DdrType );
#endif

/** @} */ /* end of 1023_init_grp group */
/** @} */ /* end of 1023_grp group */

#define CORE_E500V2

#if 0 /* using unified values */
/*****************************************************************************
 INTEGRATION-SPECIFIC MODULE CODES
******************************************************************************/
#define MODULE_UNKNOWN          0x00000000
#define MODULE_MEM              0x00010000
#define MODULE_MM               0x00020000
#define MODULE_CORE             0x00030000
#define MODULE_P1023            0x00040000
#define MODULE_MII              0x00050000
#define MODULE_PM               0x00060000
#define MODULE_MMU              0x00070000
#define MODULE_PIC              0x00080000
#define MODULE_L2_CACHE         0x00090000
#define MODULE_DUART            0x000a0000
#define MODULE_SERDES           0x000b0000
#define MODULE_PIO              0x000c0000
#define MODULE_QM               0x000d0000
#define MODULE_BM               0x000e0000
#define MODULE_SEC              0x000f0000
#define MODULE_FM               0x00100000
#define MODULE_FM_MURAM         0x00110000
#define MODULE_FM_PCD           0x00120000
#define MODULE_FM_RTC           0x00130000
#define MODULE_FM_MAC           0x00140000
#define MODULE_FM_PORT          0x00150000
#define MODULE_FM_MACSEC        0x00160000
#define MODULE_FM_MACSEC_SECY   0x00170000
#define MODULE_FM_SP            0x00280000
#define MODULE_ECM              0x00190000
#define MODULE_DMA              0x001a0000
#define MODULE_DDR              0x001b0000
#define MODULE_LAW              0x001c0000
#define MODULE_LBC              0x001d0000
#define MODULE_I2C              0x001e0000
#define MODULE_ESPI             0x001f0000
#define MODULE_PCI              0x00200000
#define MODULE_DPA_PORT         0x00210000
#define MODULE_USB              0x00220000
#endif /* using unified values */

/*****************************************************************************
 LBC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
/**************************************************************************//**
 @Group         lbc_exception_grp LBC Exception Unit

 @Description   LBC Exception unit API functions, definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Anchor        lbc_exbm

 @Collection    LBC Errors Bit Mask

                These errors are reported through the exceptions callback..
                The values can be or'ed in any combination in the errors mask
                parameter of the errors report structure.

                These errors can also be passed as a bit-mask to
                LBC_EnableErrorChecking() or LBC_DisableErrorChecking(),
                for enabling or disabling error checking.
 @{
*//***************************************************************************/
#define LBC_ERR_BUS_MONITOR     0x80000000  /**< Bus monitor error */
#define LBC_ERR_PARITY_ECC      0x20000000  /**< Parity error for GPCM/UPM */
#define LBC_ERR_WRITE_PROTECT   0x04000000  /**< Write protection error */
#define LBC_ERR_CHIP_SELECT     0x00080000  /**< Unrecognized chip select */

#define LBC_ERR_ALL             (LBC_ERR_BUS_MONITOR | LBC_ERR_PARITY_ECC | \
                                 LBC_ERR_WRITE_PROTECT | LBC_ERR_CHIP_SELECT)
                                            /**< All possible errors */
/* @} */
/** @} */ /* end of lbc_exception_grp group */

#define LBC_NUM_OF_BANKS            2
#define LBC_MAX_CS_SIZE             0x0000000100000000LL
#define LBC_ATOMIC_OPERATION_SUPPORT
#define LBC_PARITY_SUPPORT
#define LBC_ADDRESS_SHIFT_SUPPORT
#define LBC_ADDRESS_HOLD_TIME_CTRL
#define LBC_HIGH_CLK_DIVIDERS
#define LBC_FCM_AVAILABLE


/*****************************************************************************
 LAW INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define LAW_ARCH_CCB
#define LAW_NUM_OF_WINDOWS      12
#define LAW_MIN_WINDOW_SIZE     0x0000000000001000LL    /**< 4KB */
#define LAW_MAX_WINDOW_SIZE     0x0000001000000000LL    /**< 32GB */


/*****************************************************************************
 SPI INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define SPI_NUM_OF_CONTROLLERS      1

/*****************************************************************************
 PCI/PCIe INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/

#define PCI_MAX_INBOUND_WINDOWS_NUM     4
#define PCI_MAX_OUTBOUND_WINDOWS_NUM    5

/**************************************************************************//**
 @Description   Target interface of an inbound window
*//***************************************************************************/
typedef enum e_PciTargetInterface
{
    e_PCI_TARGET_PCIE_2         = 0x1,  /**<  PCI Express target interface 2 */
    e_PCI_TARGET_PCIE_1         = 0x2,  /**<  PCI Express target interface 1 */
    e_PCI_TARGET_PCIE_3         = 0x3,  /**<  PCI Express target interface 3 */
    e_PCI_TARGET_LOCAL_MEMORY   = 0xF   /**<  Local Memory (DDR SDRAM, Local Bus, SRAM) target interface */

} e_PciTargetInterface;

/*****************************************************************************
 DDR INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define DDR_NUM_OF_VALID_CS         2

/*****************************************************************************
 SEC INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define SEC_ERRATA_STAT_REGS_UNUSABLE

/*****************************************************************************
 DMA INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define DMA_NUM_OF_CONTROLLERS      2




/*****************************************************************************
 1588 INTEGRATION-SPECIFIC DEFINITIONS
******************************************************************************/
#define PTP_V2

/**************************************************************************//**
 @Function      P1023_GetMuxControlReg

 @Description   Returns the value of PMUXCR (Alternate Function Signal Multiplex
                Control Register)

 @Param[in]     gutilBase   - Base address of P1023 GUTIL registers.

 @Return        Value of PMUXCR
*//***************************************************************************/
uint32_t P1023_GetMuxControlReg(uintptr_t gutilBase);

/**************************************************************************//**
 @Function      P1023_SetMuxControlReg

 @Description   Sets the value of PMUXCR (Alternate Function Signal Multiplex
                Control Register)

 @Param[in]     gutilBase   - Base address of P1023 GUTIL registers.
 @Param[in]     val         - the new value for PMUXCR.

 @Return        None
*//***************************************************************************/
void P1023_SetMuxControlReg(uintptr_t gutilBase, uint32_t val);

/**************************************************************************//**
 @Function      P1023_GetDeviceDisableStatusRegister

 @Description   Returns the value of DEVDISR (Device Disable Register)

 @Param[in]     gutilBase   - Base address of P1023 GUTIL registers.

 @Return        Value of DEVDISR
*//***************************************************************************/
uint32_t P1023_GetDeviceDisableStatusRegister(uintptr_t gutilBase);

/**************************************************************************//**
 @Function      P1023_GetPorDeviceStatusRegister

 @Description   Returns the value of POR Device Status Register

 @Param[in]     gutilBase   - Base address of P1023 GUTIL registers.

 @Return        POR Device Status Register
*//***************************************************************************/
uint32_t P1023_GetPorDeviceStatusRegister(uintptr_t gutilBase);

/**************************************************************************//**
 @Function      P1023_GetPorBootModeStatusRegister

 @Description   Returns the value of POR Boot Mode Status Register

 @Param[in]     gutilBase   - Base address of P1023 GUTIL registers.

 @Return        POR Boot Mode Status Register value
*//***************************************************************************/
uint32_t P1023_GetPorBootModeStatusRegister(uintptr_t gutilBase);


#define PORDEVSR_SGMII1_DIS     0x10000000
#define PORDEVSR_SGMII2_DIS     0x08000000
#define PORDEVSR_ECP1           0x02000000
#define PORDEVSR_IO_SEL         0x00780000
#define PORDEVSR_IO_SEL_SHIFT   19
#define PORBMSR_HA              0x00070000
#define PORBMSR_HA_SHIFT        16

#define DEVDISR_QM_BM           0x80000000
#define DEVDISR_FM              0x40000000
#define DEVDISR_PCIE1           0x20000000
#define DEVDISR_MAC_SEC         0x10000000
#define DEVDISR_ELBC            0x08000000
#define DEVDISR_PCIE2           0x04000000
#define DEVDISR_PCIE3           0x02000000
#define DEVDISR_CAAM            0x01000000
#define DEVDISR_USB0            0x00800000
#define DEVDISR_1588            0x00020000
#define DEVDISR_CORE0           0x00008000
#define DEVDISR_TB0             0x00004000
#define DEVDISR_CORE1           0x00002000
#define DEVDISR_TB1             0x00001000
#define DEVDISR_DMA1            0x00000400
#define DEVDISR_DMA2            0x00000200
#define DEVDISR_DDR             0x00000010
#define DEVDISR_TSEC1           0x00000080
#define DEVDISR_TSEC2           0x00000040
#define DEVDISR_SPI             0x00000008
#define DEVDISR_I2C             0x00000004
#define DEVDISR_DUART           0x00000002


#endif /* __PART_INTEGRATION_EXT_H */
