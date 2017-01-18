/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*!
 * @file  devices/MX8/include/fsl_bitaccess.h
 *
 * Header file containing register access macros.
 *
 * \addtogroup Peripheral_access_layer (HAL) Device Peripheral Access Layer
 *
 * @{
 */

#ifndef _FSL_BITACCESS_H
#define _FSL_BITACCESS_H  1

/*!
 * @addtogroup SCF Register Access Macros
 * @{
 */

/*
 * Macros for single instance registers
 */

#define BF_SET(reg, field)       HW_##reg##_SET(BM_##reg##_##field)
#define BF_CLR(reg, field)       HW_##reg##_CLR(BM_##reg##_##field)
#define BF_TOG(reg, field)       HW_##reg##_TOG(BM_##reg##_##field)

#define BF_SETV(reg, field, v)   HW_##reg##_SET(BF_##reg##_##field(v))
#define BF_CLRV(reg, field, v)   HW_##reg##_CLR(BF_##reg##_##field(v))
#define BF_TOGV(reg, field, v)   HW_##reg##_TOG(BF_##reg##_##field(v))

#define BV_FLD(reg, field, sym)  BF_##reg##_##field(BV_##reg##_##field##__##sym)
#define BV_VAL(reg, field, sym)  BV_##reg##_##field##__##sym

#define BF_RD(reg, field)        HW_##reg.B.field
#define BF_WR(reg, field, v)     BW_##reg##_##field(v)


/*******************************************************************************
 * Macros to create bitfield mask, shift, and width from CMSIS definitions
 ******************************************************************************/

/* Bitfield Mask */
#define SCF_BMSK(bit) (bit ## _MASK)

/* Bitfield Left Shift */
#define SCF_BLSH(bit) (bit ## _SHIFT)

/* Bitfield Width */
#define SCF_BWID(bit) (bit ## _WIDTH)

/* Bitfield Value */
#define SCF_BVAL(bit, val) ((val) << (SCF_BLSH(bit)))


/*******************************************************************************
 * Macros to set, clear, extact, and insert bitfields into register structures
 * or local variables
 ******************************************************************************/

/* Bitfield Set */
#define SCF_BSET(var, bit) (var |= (SCF_BMSK(bit)))

/* Bitfield Clear */
#define SCF_BCLR(var, bit) (var &= (~(SCF_BMSK(bit))))

/* Bitfield Extract */
#define SCF_BEXR(var, bit) ((var & (SCF_BMSK(bit))) >> (SCF_BLSH(bit)))

/* Bitfield Insert */
#define SCF_BINS(var, bit, val) (var = (var & (~(SCF_BMSK(bit)))) | SCF_BVAL(bit, val))


/*******************************************************************************
 * Macros to set, clear, extact, and insert bitfields into register structures
 * that support SCT
 ******************************************************************************/

#ifdef EMUL
/* Emulation does not have SCT hardware and must fallback to non-SCT definitions */

/* SCT Bitfield Set */
#define SCF_SCT_BSET(var, bit) (SCF_BSET(var, bit))

/* SCT Bitfield Clear */
#define SCF_SCT_BCLR(var, bit) (SCF_BCLR(var, bit))

/* SCT Bitfield Insert */
#define SCF_SCT_BINS(var, bit, val) (SCF_BINS(var, bit, val))

#else
/*!  @todo Port macros leverage SCT register access hardware */

/* SCT Bitfield Set */
#define SCF_SCT_BSET(var, bit) (SCF_BSET(var, bit))

/* SCT Bitfield Clear */
#define SCF_SCT_BCLR(var, bit) (SCF_BCLR(var, bit))

/* SCT Bitfield Insert */
#define SCF_SCT_BINS(var, bit, val) (SCF_BINS(var, bit, val))

#endif /* EMUL */

/*!
 * @}
 */ /* end of group SCF */

/*!
 * @}
 */ /* end of group Peripheral_access_layer */

#endif /* _FSL_BITACCESS_H */

/******************************************************************************/
