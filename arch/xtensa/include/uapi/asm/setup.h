/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * include/asm-xtensa/setup.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2005 Tensilica Inc.
 */

#ifndef _XTENSA_SETUP_H
#define _XTENSA_SETUP_H

#ifdef __KERNEL__
#define COMMAND_LINE_SIZE	CONFIG_COMMAND_LINE_SIZE
#else
#define COMMAND_LINE_SIZE	256
#endif

#endif
