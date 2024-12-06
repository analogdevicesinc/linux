/*****************************************************************************
 *    The GPL License (GPL)
 *
 *    Copyright (c) 2015-2017, VeriSilicon Inc.
 *    Copyright (c) 2011-2014, Google Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 *****************************************************************************/

#ifndef SOFTWARE_LINUX_DWL_DWL_DEFS_H_
#define SOFTWARE_LINUX_DWL_DWL_DEFS_H_

#define DWL_MPEG2_E 31 /* 1 bit */
#define DWL_VC1_E 29   /* 2 bits */
#define DWL_JPEG_E 28  /* 1 bit */
#define DWL_MPEG4_E 26 /* 2 bits */
#define DWL_H264_E 24  /* 2 bits */
#define DWL_VP6_E 23   /* 1 bit */
#define DWL_RV_E 26    /* 2 bits */
#define DWL_VP8_E 23   /* 1 bit */
#define DWL_VP7_E 24   /* 1 bit */
#define DWL_WEBP_E 19  /* 1 bit */
#define DWL_AVS_E 22   /* 1 bit */
#if 0
#define DWL_PP_E       16  /* 1 bit */
#endif
#define DWL_PP_E 31    /* 1 bit */
#define DWL_HEVC_E 5   /* 2 bits */
#define DWL_VP9_E 3    /* 2 bits */

#define DWL_HEVC_ENA 0  /* 1 bits */
#define DWL_VP9_ENA 1   /* 1 bits */
#define DWL_RFC_E 2     /* 1 bits */
#define DWL_DS_E 3      /* 1 bits */
#define DWL_HEVC_VER 8  /* 4 bits */
#define DWL_VP9_PROFILE 12 /* 3 bits */
#define DWL_RING_E 16   /* 1 bits */

#define HANTRODEC_IRQ_STAT_DEC 1
#define HANTRODEC_IRQ_STAT_DEC_OFF (HANTRODEC_IRQ_STAT_DEC * 4)

#define HANTRODECPP_SYNTH_CFG 60
#define HANTRODECPP_SYNTH_CFG_OFF (HANTRODECPP_SYNTH_CFG * 4)
#define HANTRODEC_SYNTH_CFG 50
#define HANTRODEC_SYNTH_CFG_OFF (HANTRODEC_SYNTH_CFG * 4)
#define HANTRODEC_SYNTH_CFG_2 54
#define HANTRODEC_SYNTH_CFG_2_OFF (HANTRODEC_SYNTH_CFG_2 * 4)
#define HANTRODEC_SYNTH_CFG_3 56
#define HANTRODEC_SYNTH_CFG_3_OFF (HANTRODEC_SYNTH_CFG_3 * 4)
#define HANTRODEC_CFG_STAT 23
#define HANTRODEC_CFG_STAT_OFF (HANTRODEC_CFG_STAT * 4)


#define HANTRODEC_DEC_E 0x01
#define HANTRODEC_PP_E 0x01
#define HANTRODEC_DEC_ABORT 0x20
#define HANTRODEC_DEC_IRQ_DISABLE 0x10
#define HANTRODEC_DEC_IRQ 0x100
#define HANTRODEC_DEC_DONE 0x1000

/* Legacy from G1 */
#define HANTRO_IRQ_STAT_DEC          1
#define HANTRO_IRQ_STAT_DEC_OFF      (HANTRO_IRQ_STAT_DEC * 4)
#define HANTRO_IRQ_STAT_PP           60
#define HANTRO_IRQ_STAT_PP_OFF       (HANTRO_IRQ_STAT_PP * 4)

#define HANTROPP_SYNTH_CFG           100
#define HANTROPP_SYNTH_CFG_OFF       (HANTROPP_SYNTH_CFG * 4)
#define HANTRODEC_SYNTH_CFG          50
#define HANTRODEC_SYNTH_CFG_OFF      (HANTRODEC_SYNTH_CFG * 4)
#define HANTRODEC_SYNTH_CFG_2        54
#define HANTRODEC_SYNTH_CFG_2_OFF    (HANTRODEC_SYNTH_CFG_2 * 4)

#define HANTRO_DEC_E                 0x01
#define HANTRO_PP_E                  0x01
#define HANTRO_DEC_ABORT             0x20
#define HANTRO_DEC_IRQ_DISABLE       0x10
#define HANTRO_PP_IRQ_DISABLE        0x10
#define HANTRO_DEC_IRQ               0x100
#define HANTRO_PP_IRQ                0x100

#endif /* SOFTWARE_LINUX_DWL_DWL_DEFS_H_ */
