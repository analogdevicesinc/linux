/**
 * io.h - Cadence USB3 IO Header
 *
 * Copyright (C) 2016 Cadence Design Systems - https://www.cadence.com/
 *
 * Authors: Rafal Ozieblo <rafalo@cadence.com>,
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __DRIVERS_USB_CDNS_IO_H
#define __DRIVERS_USB_CDNS_IO_H

#include <linux/io.h>

static inline u32 cdns_readl(uint32_t __iomem *reg)
{
	u32 value = 0;

	value = readl(reg);
	return value;
}

static inline void cdns_writel(uint32_t __iomem *reg, u32 value)
{
	writel(value, reg);
}


#endif /* __DRIVERS_USB_CDNS_IO_H */
