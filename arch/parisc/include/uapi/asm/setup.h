/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _PARISC_SETUP_H
#define _PARISC_SETUP_H

#ifdef __KERNEL__
#define COMMAND_LINE_SIZE	CONFIG_COMMAND_LINE_SIZE
#else
#define COMMAND_LINE_SIZE	1024
#endif

#endif /* _PARISC_SETUP_H */
