/*
 * Copyright (C) 2017 NXP
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

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/io.h>

#include <video/imx-dcss.h>
#include "dcss-prv.h"

struct dcss_dec400d_priv {
	struct dcss_soc *dcss;
	void __iomem *dec400d_reg;
};

int dcss_dec400d_init(struct dcss_soc *dcss, unsigned long dec400d_base)
{
	struct dcss_dec400d_priv *dec400d;

	dec400d = devm_kzalloc(dcss->dev, sizeof(*dec400d), GFP_KERNEL);
	if (!dec400d)
		return -ENOMEM;

	dcss->dec400d_priv = dec400d;
	dec400d->dcss = dcss;

	dec400d->dec400d_reg = devm_ioremap(dcss->dev, dec400d_base, SZ_4K);
	if (!dec400d->dec400d_reg) {
		dev_err(dcss->dev, "dec400d: unable to remap dec400d base\n");
		return -ENOMEM;
	}

	/* PUT IN BYPASS FOR NOW */
//[lp]	dcss_writel(0, dec400d->dec400d_reg + 0x2c0);

	return 0;
}

void dcss_dec400d_exit(struct dcss_soc *dcss)
{
}
