/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef BCM2835_MFD_PM_H
#define BCM2835_MFD_PM_H

#include <linux/regmap.h>

enum bcm2835_soc {
	BCM2835_PM_SOC_BCM2835,
	BCM2835_PM_SOC_BCM2711,
	BCM2835_PM_SOC_BCM2712,
};

struct bcm2835_pm {
	struct device *dev;
	void __iomem *base;
	void __iomem *asb;
	void __iomem *rpivid_asb;
	enum bcm2835_soc soc;
};

#endif /* BCM2835_MFD_PM_H */
