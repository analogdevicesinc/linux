/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LINUX_KALLSYMS_INTERNAL_H_
#define LINUX_KALLSYMS_INTERNAL_H_

#include <linux/types.h>

/*
 * These will be re-linked against their real values during the second link
 * stage. Preliminary values must be provided in the linker script using the
 * PROVIDE() directive so that the first link stage can complete successfully.
 */
extern const unsigned long kallsyms_addresses[];
extern const int kallsyms_offsets[];
extern const u8 kallsyms_names[];

extern const unsigned int kallsyms_num_syms;
extern const unsigned long kallsyms_relative_base;

extern const char kallsyms_token_table[];
extern const u16 kallsyms_token_index[];

extern const unsigned int kallsyms_markers[];
extern const u8 kallsyms_seqs_of_names[];

#endif // LINUX_KALLSYMS_INTERNAL_H_
