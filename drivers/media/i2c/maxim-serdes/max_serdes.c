// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include "max_serdes.h"

#define MAX_FMT(_code, _dt, _bpp, _dbl) \
{                       \
    .code = (_code),            \
    .dt = (_dt),                \
    .bpp = (_bpp),              \
    .dbl = (_dbl),              \
}

static const struct max_format max_formats[] = {
    MAX_FMT(MEDIA_BUS_FMT_YUYV8_1X16, MAX_DT_YUV422_8B, 16, 0),
    MAX_FMT(MEDIA_BUS_FMT_YUYV10_1X20, MAX_DT_YUV422_10B, 20, 0),
    MAX_FMT(MEDIA_BUS_FMT_RGB565_1X16, MAX_DT_RGB565, 16, 0),
    MAX_FMT(MEDIA_BUS_FMT_RGB666_1X18, MAX_DT_RGB666, 18, 0),
    MAX_FMT(MEDIA_BUS_FMT_RGB888_1X24, MAX_DT_RGB888, 24, 0),
    MAX_FMT(MEDIA_BUS_FMT_SBGGR8_1X8, MAX_DT_RAW8, 8, 1),
    MAX_FMT(MEDIA_BUS_FMT_SGBRG8_1X8, MAX_DT_RAW8, 8, 1),
    MAX_FMT(MEDIA_BUS_FMT_SGRBG8_1X8, MAX_DT_RAW8, 8, 1),
    MAX_FMT(MEDIA_BUS_FMT_SRGGB8_1X8, MAX_DT_RAW8, 8, 1),
    MAX_FMT(MEDIA_BUS_FMT_SBGGR10_1X10, MAX_DT_RAW10, 10, 1),
    MAX_FMT(MEDIA_BUS_FMT_SGBRG10_1X10, MAX_DT_RAW10, 10, 1),
    MAX_FMT(MEDIA_BUS_FMT_SGRBG10_1X10, MAX_DT_RAW10, 10, 1),
    MAX_FMT(MEDIA_BUS_FMT_SRGGB10_1X10, MAX_DT_RAW10, 10, 1),
    MAX_FMT(MEDIA_BUS_FMT_SBGGR12_1X12, MAX_DT_RAW12, 12, 1),
    MAX_FMT(MEDIA_BUS_FMT_SGBRG12_1X12, MAX_DT_RAW12, 12, 1),
    MAX_FMT(MEDIA_BUS_FMT_SGRBG12_1X12, MAX_DT_RAW12, 12, 1),
    MAX_FMT(MEDIA_BUS_FMT_SRGGB12_1X12, MAX_DT_RAW12, 12, 1),
    MAX_FMT(MEDIA_BUS_FMT_SBGGR14_1X14, MAX_DT_RAW14, 14, 0),
    MAX_FMT(MEDIA_BUS_FMT_SGBRG14_1X14, MAX_DT_RAW14, 14, 0),
    MAX_FMT(MEDIA_BUS_FMT_SGRBG14_1X14, MAX_DT_RAW14, 14, 0),
    MAX_FMT(MEDIA_BUS_FMT_SRGGB14_1X14, MAX_DT_RAW14, 14, 0),
    MAX_FMT(MEDIA_BUS_FMT_SBGGR16_1X16, MAX_DT_RAW16, 16, 0),
    MAX_FMT(MEDIA_BUS_FMT_SGBRG16_1X16, MAX_DT_RAW16, 16, 0),
    MAX_FMT(MEDIA_BUS_FMT_SGRBG16_1X16, MAX_DT_RAW16, 16, 0),
    MAX_FMT(MEDIA_BUS_FMT_SRGGB16_1X16, MAX_DT_RAW16, 16, 0),
};

const struct max_format *max_format_by_code(u32 code)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(max_formats); i++)
        if (max_formats[i].code == code)
            return &max_formats[i];

    return NULL;
}
EXPORT_SYMBOL_GPL(max_format_by_code);

const struct max_format *max_format_by_dt(u8 dt)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(max_formats); i++)
        if (max_formats[i].dt == dt)
            return &max_formats[i];

    return NULL;
}
EXPORT_SYMBOL_GPL(max_format_by_dt);
