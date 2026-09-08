/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef __ASM_GENERIC_SETUP_H
#define __ASM_GENERIC_SETUP_H

#ifdef __KERNEL__
#define COMMAND_LINE_SIZE	CONFIG_COMMAND_LINE_SIZE
#else
#define COMMAND_LINE_SIZE	512
#endif

#endif	/* __ASM_GENERIC_SETUP_H */
