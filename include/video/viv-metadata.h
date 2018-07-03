/******************************************************************************\
|*                                                                            *|
|* Copyright (c) 2007-2018 by Vivante Corp.  All rights reserved.             *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* Vivante Corporation.  This is proprietary information owned by Vivante     *|
|* Corporation.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of Vivante Corporation.                                 *|
|*                                                                            *|
\******************************************************************************/

#ifndef __VIV_METADATA_H__
#define __VIV_METADATA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Macro to combine four characters into a Character Code. */
#define __FOURCC(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

#define VIV_VIDMEM_METADATA_MAGIC __FOURCC('v', 'i', 'v', 'm')

/* Compressed format now was defined same as dec400d, should be general. */
typedef enum _VIV_COMPRESS_FMT
{
    _VIV_CFMT_ARGB8 = 0,
    _VIV_CFMT_XRGB8,
    _VIV_CFMT_AYUV,
    _VIV_CFMT_UYVY,
    _VIV_CFMT_YUY2,
    _VIV_CFMT_YUV_ONLY,
    _VIV_CFMT_UV_MIX,
    _VIV_CFMT_ARGB4,
    _VIV_CFMT_XRGB4,
    _VIV_CFMT_A1R5G5B5,
    _VIV_CFMT_X1R5G5B5,
    _VIV_CFMT_R5G6B5,
    _VIV_CFMT_Z24S8,
    _VIV_CFMT_Z24,
    _VIV_CFMT_Z16,
    _VIV_CFMT_A2R10G10B10,
    _VIV_CFMT_BAYER,
    _VIV_CFMT_SIGNED_BAYER,
    _VIV_CFMT_VAA16,
    _VIV_CFMT_S8,

    _VIV_CFMT_MAX,
} _VIV_COMPRESS_FMT;

/* Metadata for cross-device fd share with additional (ts) info. */
typedef struct _VIV_VIDMEM_METADATA
{
    uint32_t magic;

    int32_t  ts_fd;
    void *   ts_dma_buf;

    uint32_t fc_enabled;
    uint32_t fc_value;
    uint32_t fc_value_upper;

    uint32_t compressed;
    uint32_t compress_format;
} _VIV_VIDMEM_METADATA;

#ifdef __cplusplus
}
#endif

#endif /* __VIV_METADATA_H__ */

