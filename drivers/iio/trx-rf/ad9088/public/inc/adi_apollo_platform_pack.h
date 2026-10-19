/**
 * \file adi_apollo_platform_pack.h
 *
 * \brief   Contains platform pack define
 *
 * \details Contains platform pack define
 *
 * APOLLO API Version: $ADI_APOLLO_API_VERSION$
 */

/**
 * Disclaimer Legal Disclaimer
 * Copyright 2019 - 2019 Analog Devices Inc.
 * Released under the APOLLO API license, for more information
 * see the "LICENSE.PDF" file in this zip file.
 */

#ifndef ADI_APOLLO_PLATFORM_PACK_H__
#define ADI_APOLLO_PLATFORM_PACK_H__

#ifndef ADI_APOLLO_FW
#include "adi_apollo_user.h"
#endif


#if (!defined(ADI_APOLLO_PACK_START) || !defined(ADI_APOLLO_PACK_FINISH))
#ifdef __GNUC__
#define ADI_APOLLO_PACK_START     _Pragma("pack(1)")
#define ADI_APOLLO_PACK_FINISH    _Pragma("pack()")
#elif defined  __ICCARM__
#define ADI_APOLLO_PACK_START     _Pragma("pack(1)")
#define ADI_APOLLO_PACK_FINISH    _Pragma("pack()")
#elif defined _MSC_VER
#define ADI_APOLLO_PACK_START     __pragma(pack(1))
#define ADI_APOLLO_PACK_FINISH    __pragma(pack())
#else
#error ( "Define the ADI_APOLLO_PACK_START and ADI_APOLLO_PACK_FINISH macros for your compiler." )
#endif
#endif

#ifndef ADI_APOLLO_PACKED
#if defined  __ICCARM__
/*
 * MISRA Deviation: Rule 20.7 (MISRA C:2012) / Rule 19.10 (MISRA C:2004)
 * Error[Pm154]: Macro parameter 'd' cannot be parenthesized.
 *
 * Justification: ADI_APOLLO_PACKED() is used for structure packing. The parameter
 * must be a structure definition (code block), which cannot be enclosed in
 * parentheses - doing so would be syntactically invalid C.
 * This is a permitted deviation due to language constraints.
 */
#pragma diag_suppress=Pm154
#define ADI_APOLLO_PACKED(d)    _Pragma("pack(1)") d _Pragma("pack()")
#pragma diag_default=Pm154
#else
#define ADI_APOLLO_PACKED(d)    ADI_APOLLO_PACK_START d ADI_APOLLO_PACK_FINISH
#endif
#endif

#endif /* ADI_APOLLO_PLATFORM_PACK_H__ */
