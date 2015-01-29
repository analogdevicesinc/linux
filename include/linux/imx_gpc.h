/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*
 * @file linux/imx_gpc.h
 *
 * @brief Global header file for imx GPC
 *
 * @ingroup GPC
 */
#ifndef __LINUX_IMX_GPC_H__
#define __LINUX_IMX_GPC_H__

#ifdef CONFIG_HAVE_IMX_GPC
int imx_gpc_mf_request_on(unsigned int irq, unsigned int on);
#else
static inline int imx_gpc_mf_request_on(unsigned int irq, unsigned int on) { return 0; }
#endif

#endif /* __LINUX_IMX_GPC_H__ */
