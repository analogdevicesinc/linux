/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
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

/******************************************************************************
 @File          fsl_fman_test.h

 @Description
*//***************************************************************************/

#ifndef __FSL_FMAN_TEST_H
#define __FSL_FMAN_TEST_H

#include <linux/types.h>
#include <linux/smp.h>  /* raw_smp_processor_id() */

//#define FMT_K_DBG
//#define FMT_K_DBG_RUNTIME

#define _fmt_prk(stage, format, arg...)	\
	printk(stage "fmt (cpu:%u): " format, raw_smp_processor_id(), ##arg)

#define _fmt_inf(format, arg...) _fmt_prk(KERN_INFO, format, ##arg)
#define _fmt_wrn(format, arg...) _fmt_prk(KERN_WARNING, format, ##arg)
#define _fmt_err(format, arg...) _fmt_prk(KERN_ERR, format, ##arg)

/* there are two macros for debugging: for runtime and generic.
 * Helps when the runtime functions are not targeted for debugging,
 * thus all the unnecessary information will be skipped.
 */
/* used for generic debugging */
#if defined(FMT_K_DBG)
	#define _fmt_dbg(format, arg...) \
		printk("fmt [%s:%u](cpu:%u) - " format,	\
			__func__, __LINE__, raw_smp_processor_id(), ##arg)
#else
#	define _fmt_dbg(arg...)
#endif

/* used for debugging runtime functions */
#if defined(FMT_K_DBG_RUNTIME)
	#define _fmt_dbgr(format, arg...) \
		printk("fmt [%s:%u](cpu:%u) - " format, \
			__func__, __LINE__, raw_smp_processor_id(), ##arg)
#else
#	define _fmt_dbgr(arg...)
#endif

#define FMT_RX_ERR_Q    0xffffffff
#define FMT_RX_DFLT_Q   0xfffffffe
#define FMT_TX_ERR_Q    0xfffffffd
#define FMT_TX_CONF_Q   0xfffffffc

#define FMAN_TEST_MAX_TX_FQS 8

#endif /* __FSL_FMAN_TEST_H */
