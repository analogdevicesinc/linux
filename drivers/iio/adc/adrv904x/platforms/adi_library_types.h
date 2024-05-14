/**
* Copyright 2015 - 2022 Analog Devices Inc.
* Released under the ADRV904X API license, for more information
* see the "LICENSE.pdf" file in this zip file.
*/

/**
* \file adi_library_types.h
*
* ADRV904X API Version: 2.10.0.4
*/
#ifndef _ADI_LIBRARY_TYPES_H_
#define _ADI_LIBRARY_TYPES_H_

#ifdef __KERNEL__
#include <linux/int_log.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/time.h>
#else
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <limits.h>
#include <ctype.h>
#endif

#ifdef __KERNEL__
typedef time64_t time_t;
#define INT16_MAX                                       S16_MAX
#define INT16_MIN                                       S16_MIN
#define UINT16_MAX                                      U16_MAX
#define UINT32_MAX                                      U32_MAX

#define PRIX32                                          "lX"
#define PRIu32                                          "u"
#define PRId64                                          "d"
#define PRIX64                                          "llX"
#define time_t                                          time64_t
#endif

/* stddef.h */
#define ADI_LIBRARY_OFFSETOF                            offsetof

/* stdio.h */
#ifndef __KERNEL__
#define ADI_LIBRARY_PRINTF                              printf
#else
#define ADI_LIBRARY_PRINTF                              printk
#endif
#define ADI_LIBRARY_FPRINTF                             fprintf
#ifndef __KERNEL__
#define ADI_LIBRARY_SPRINTF                             sprintf
#endif
#define ADI_LIBRARY_SNPRINTF                            snprintf
#ifndef __KERNEL__
#define ADI_LIBRARY_VPRINTF                             vprintf
#endif
#define ADI_LIBRARY_VSNPRINTF                           vsnprintf
#define ADI_LIBRARY_FFLUSH                              fflush
#define ADI_LIBRARY_FSEEK                               __adrv904x_fseek
#define ADI_LIBRARY_FREAD                               __adrv904x_fread
#define ADI_LIBRARY_FWRITE                              __adrv904x_fwrite
#define ADI_LIBRARY_FOPEN                               fopen
#define ADI_LIBRARY_FOPEN_S                             fopen_s
#define ADI_LIBRARY_FCLOSE                              __adrv904x_fclose
#define ADI_LIBRARY_FTELL                               __adrv904x_ftell
#define ADI_LIBRARY_FERROR                              ferror
#ifndef __KERNEL__
#define ADI_LIBRARY_SETVBUF                             setvbuf
#endif

/* stdlib.h */
#ifndef __KERNEL__
#define ADI_LIBRARY_CALLOC                              calloc
#define ADI_LIBRARY_FREE                                free
#define ADI_LIBRARY_RAND                                rand
#define ADI_LIBRARY_EXIT                                exit
#define ADI_LIBRARY_ABS                                 abs
#else
#define ADI_LIBRARY_FREE                                kfree
#define ADI_LIBRARY_CALLOC                              __adrv904x_calloc
#endif

/* stdarg.h */
#define ADI_LIBRARY_VA_START                            va_start
#define ADI_LIBRARY_VA_END                              va_end

/* math.h*/
#ifndef __KERNEL__
#define ADI_LIBRARY_CEIL                                ceil
#endif

/* string.h */
#define ADI_LIBRARY_MEMSET                              memset
#define ADI_LIBRARY_MEMCPY                              memcpy
#define ADI_LIBRARY_STRLEN                              strlen
#define ADI_LIBRARY_MEMCMP                              memcmp
#define ADI_LIBRARY_STRCMP                              strcmp
#define ADI_LIBRARY_STRTOK                              strtok
#define ADI_LIBRARY_STRTOK_R                            strtok_r

#if __STDC_VERSION__ >= 199901L     /* C99 */
    #define ADI_LIBRARY_STRNLEN                         adi_library_strnlen     /* ADI Implementation Required for C99 Make File Test */
#else
    #define ADI_LIBRARY_STRNLEN                         strnlen
#endif

#define ADI_LIBRARY_STRCHR                              strchr
#define ADI_LIBRARY_STRCAT                              strcat
#define ADI_LIBRARY_STRNCAT                             strncat
#define ADI_LIBRARY_STRNCPY                             strncpy

#if __STDC_VERSION__ >= 199901L     /* C99 */
    #define ADI_LIBRARY_STRDUP                          adi_library_strdup      /* ADI Implementation Required for C99 Make File Test */
#else
    #define ADI_LIBRARY_STRDUP                          strdup
#endif

#define ADI_LIBRARY_STRSTR                              strstr

/* time.h */

#define ADI_LIBRARY_TIME                                time
#define ADI_LIBRARY_LOCALTIME_R                         localtime_r
#define ADI_LIBRARY_MKTIME                              mktime
#define ADI_LIBRARY_CTIME                               ctime
#define ADI_LIBRARY_NANOSLEEP                           nanosleep
#ifndef __KERNEL__
#define ADI_LIBRARY_CLOCK                               clock
#else
#define ADI_LIBRARY_CLOCK                               ktime_get_ns
#endif
#define ADI_LIBRARY_CLOCKS_PER_SEC                      CLOCKS_PER_SEC
#define ADI_LIBRARY_GMTIME                              gmtime

/* ctype.h */
#define ADI_LIBRARY_TOUPPER                             toupper
#define ADI_LIBRARY_TOLOWER                             tolower


#endif  /* _ADI_LIBRARY_TYPES_H_ */
