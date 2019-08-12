/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_LTC2947_H
#define _LINUX_LTC2947_H

struct regmap;

extern const struct of_device_id ltc2947_of_match[];

int ltc2947_core_probe(struct regmap *map, const char *name);

#endif
