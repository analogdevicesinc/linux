/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021 Emil Renner Berthing <kernel@esmil.dk>
 */

#ifndef __RESET_STARFIVE_JH71X0_H
#define __RESET_STARFIVE_JH71X0_H

/**
 * struct jh71x0_reset_info - description of a reset controller
 * @nr_resets: number of resets, which is the number of @map entries if @map
 *             is set
 * @assert_offset: offset of the first assert register
 * @status_offset: offset of the first status register
 * @map: optional table of hardware reset lines, i.e. bit numbers in the
 *       assert/status registers, indexed by reset ID, for controllers whose
 *       reset IDs do not map one-to-one to the hardware reset lines; NULL if
 *       the reset ID is the hardware reset line
 */
struct jh71x0_reset_info {
	unsigned int nr_resets;
	unsigned int assert_offset;
	unsigned int status_offset;
	const unsigned int *map;
};

int reset_starfive_jh71x0_register(struct device *dev, struct device_node *of_node,
				   void __iomem *assert, void __iomem *status,
				   const u32 *asserted, unsigned int nr_resets,
				   struct module *owner);

int reset_starfive_jh71x0_register_with_info(struct device *dev, struct device_node *of_node,
					     void __iomem *assert, void __iomem *status,
					     const u32 *asserted,
					     const struct jh71x0_reset_info *info,
					     struct module *owner);

#endif /* __RESET_STARFIVE_JH71X0_H */
