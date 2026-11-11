/* SPDX-License-Identifier: GPL-2.0 */
/* There is nothing for KASAN or KCSAN to check in user space. */
#define instrument_read(p, size)	((void)(p), (void)(size))
