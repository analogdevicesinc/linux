/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_STARFIVE_RESET_JH71X0_H
#define __SOC_STARFIVE_RESET_JH71X0_H

#include <linux/compiler_types.h>

/*
 * Platform data handed to the reset auxiliary device created by the
 * StarFive JH71X0 clock drivers.
 */
struct jh71x0_reset_platform_data {
	void __iomem *base;
};

#endif
