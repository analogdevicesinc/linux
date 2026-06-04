/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LIB_KSTRTOX_H
#define _LIB_KSTRTOX_H

#include <linux/args.h>

#define KSTRTOX_OVERFLOW	(1U << 31)
const char *_parse_integer_fixup_radix(const char *s, unsigned int *base);
unsigned int _parse_integer_limit(const char *s, unsigned int base, unsigned long long *res,
				  size_t max_chars);

#define _parse_integer0(s, base, res, ...)						\
	_parse_integer_limit(s, base, res, INT_MAX)

#define _parse_integer(s, base, res, ...)						\
	CONCATENATE(_parse_integer, COUNT_ARGS(__VA_ARGS__))(s, base, res, __VA_ARGS__)

#endif
