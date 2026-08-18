/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *	Just a place holder. 
 */

#ifndef _UAPI_SPARC_SETUP_H
#define _UAPI_SPARC_SETUP_H

#ifdef __KERNEL__
# define COMMAND_LINE_SIZE CONFIG_COMMAND_LINE_SIZE
#else
# if defined(__sparc__) && defined(__arch64__)
#  define COMMAND_LINE_SIZE 2048
# else
#  define COMMAND_LINE_SIZE 256
# endif
#endif


#endif /* _UAPI_SPARC_SETUP_H */
