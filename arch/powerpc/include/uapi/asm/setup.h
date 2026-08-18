/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_ASM_POWERPC_SETUP_H
#define _UAPI_ASM_POWERPC_SETUP_H

#ifdef __KERNEL__
#define COMMAND_LINE_SIZE	CONFIG_COMMAND_LINE_SIZE
#else
#define COMMAND_LINE_SIZE	2048
#endif

#endif /* _UAPI_ASM_POWERPC_SETUP_H */
