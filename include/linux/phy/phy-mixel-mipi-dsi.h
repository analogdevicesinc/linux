/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef PHY_MIXEL_MIPI_DSI_H_
#define PHY_MIXEL_MIPI_DSI_H_

#include "phy.h"

#if IS_ENABLED(CONFIG_PHY_MIXEL_MIPI_DSI)
int mixel_phy_mipi_set_phy_speed(struct phy *phy,
		unsigned long bit_clk,
		unsigned long ref_clk,
		bool best_match);
#else
int mixel_phy_mipi_set_phy_speed(struct phy *phy,
		unsigned long bit_clk,
		unsigned long ref_clk,
		bool best_match)
{
	return -ENODEV;
}
#endif

#endif	/* PHY_MIXEL_MIPI_DSI_H_ */
