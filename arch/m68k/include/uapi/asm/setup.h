/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
** asm/setup.h -- Definition of the Linux/m68k setup information
**
** Copyright 1992 by Greg Harp
**
** This file is subject to the terms and conditions of the GNU General Public
** License.  See the file COPYING in the main directory of this archive
** for more details.
*/

#ifndef _UAPI_M68K_SETUP_H
#define _UAPI_M68K_SETUP_H

#ifdef __KERNEL__
#define COMMAND_LINE_SIZE	CONFIG_COMMAND_LINE_SIZE
#else
#define COMMAND_LINE_SIZE	256
#endif

#endif /* _UAPI_M68K_SETUP_H */
