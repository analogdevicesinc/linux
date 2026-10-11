/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PTR_EQ_H
#define _LINUX_PTR_EQ_H

/*
 * Compare two addresses while preserving the address dependencies for
 * later use of the address. It should be used when comparing an address
 * returned by rcu_dereference().
 *
 * This is needed to prevent the compiler CSE and SSA GVN optimizations
 * from using @a (or @b) in places where the source refers to @b (or @a)
 * based on the fact that after the comparison, the two are known to be
 * equal, which does not preserve address dependencies and allows the
 * following misordering speculations:
 *
 * - If @b is a constant, the compiler can issue the loads which depend
 *   on @a before loading @a.
 * - If @b is a register populated by a prior load, weakly-ordered
 *   CPUs can speculate loads which depend on @a before loading @a.
 *
 * The same logic applies with @a and @b swapped.
 *
 * Return value: true if pointers are equal, false otherwise.
 *
 * The compiler barrier() is ineffective at fixing this issue. It does
 * not prevent the compiler CSE from losing the address dependency:
 *
 * int fct_2_volatile_barriers(void)
 * {
 *     int *a, *b;
 *
 *     do {
 *         a = READ_ONCE(p);
 *         asm volatile ("" : : : "memory");
 *         b = READ_ONCE(p);
 *     } while (a != b);
 *     asm volatile ("" : : : "memory");  <-- barrier()
 *     return *b;
 * }
 *
 * With gcc 14.2 (arm64):
 *
 * fct_2_volatile_barriers:
 *         adrp    x0, .LANCHOR0
 *         add     x0, x0, :lo12:.LANCHOR0
 * .L2:
 *         ldr     x1, [x0]  <-- x1 populated by first load.
 *         ldr     x2, [x0]
 *         cmp     x1, x2
 *         bne     .L2
 *         ldr     w0, [x1]  <-- x1 is used for access which should depend on b.
 *         ret
 *
 * On weakly-ordered architectures, this lets CPU speculation use the
 * result from the first load to speculate "ldr w0, [x1]" before
 * "ldr x2, [x0]".
 * Based on the RCU documentation, the control dependency does not
 * prevent the CPU from speculating loads.
 */

#include <asm/ptr_eq.h>

#endif	/* _LINUX_PTR_EQ_H */
