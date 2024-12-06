// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ELE Random Number Generator Driver NXP's Platforms
 *
 * Copyright 2024 NXP
 */

#include "ele_trng.h"
#include "ele_fw_api.h"

struct ele_trng {
	struct hwrng rng;
	struct se_if_priv *priv;
};

int ele_trng_init(struct se_if_priv *priv)
{
	struct ele_trng *trng;
	int ret;

	trng = devm_kzalloc(priv->dev, sizeof(*trng), GFP_KERNEL);
	if (!trng)
		return -ENOMEM;

	trng->priv        = priv;
	trng->rng.name    = "ele-trng";
	trng->rng.read    = ele_get_hwrng;
	trng->rng.priv    = (unsigned long)trng;
	trng->rng.quality = 1024;

	dev_dbg(priv->dev, "registering ele-trng\n");

	ret = devm_hwrng_register(priv->dev, &trng->rng);
	if (ret)
		return ret;

	dev_info(priv->dev, "Successfully registered ele-trng\n");
	return 0;
}

int ele_get_hwrng(struct hwrng *rng,
		  void *data, size_t len, bool wait)
{
	struct ele_trng *trng = (struct ele_trng *)rng->priv;

	return ele_get_random(trng->priv, data, len);
}
