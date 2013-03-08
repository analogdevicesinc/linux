/*
 * SSM4567 amplifier audio driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "tdmc.h"
#include "ssm4567.h"

static int ssm4567_tdmc_probe(struct tdmc_slave *slave)
{
	return ssm4567_probe(&slave->dev, true,
		devm_tdmc_regmap_init(slave, &ssm4567_regmap_config));
}

static void ssm4567_tdmc_remove(struct tdmc_slave *slave)
{
	snd_soc_unregister_codec(&slave->dev);
}

static const struct of_device_id ssm4567_of_id_table[] = {
	{ .compatible = "adi,ssm4567" },
	{ },
};

static struct tdmc_slave_driver ssm4567_tdmc_driver = {
	.driver = {
		.name = "ssm4567",
		.of_match_table = ssm4567_of_id_table,
		.owner = THIS_MODULE,
	},
	.probe = ssm4567_tdmc_probe,
	.remove = ssm4567_tdmc_remove,
};
module_tdmc_slave_driver(ssm4567_tdmc_driver);

MODULE_DESCRIPTION("ASoC SSM4567 tdmc driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
