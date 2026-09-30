/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LINUX_KALLSYMS_INTERNAL_H_
#define LINUX_KALLSYMS_INTERNAL_H_

/*
 * Provide compile-constants for scripts/kallsyms.c
 * so it can build the corresponding kallsyms_marker[] table.
 * and wrap the rest in __KERNEL__
 */
#define KALLSYMS_MARKER_SHIFT 4
#define KALLSYMS_MARKER_SIZE  (1U << KALLSYMS_MARKER_SHIFT)
#define KALLSYMS_MARKER_MASK  (KALLSYMS_MARKER_SIZE - 1U)

#ifdef __KERNEL__
#include <linux/types.h>

extern const int kallsyms_offsets[];
extern const u8 kallsyms_names[];

extern const unsigned int kallsyms_num_syms;

extern const char kallsyms_token_table[];
extern const u16 kallsyms_token_index[];

extern const unsigned int kallsyms_markers[];
extern const u8 kallsyms_seqs_of_names[];
#endif /* __KERNEL__ */

#endif // LINUX_KALLSYMS_INTERNAL_H_
