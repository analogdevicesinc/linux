#ifndef __SND_SOC_CODECS_SSM4329_H__
#define __SND_SOC_CODECS_SSM4329_H__


#include <linux/regmap.h>

#define SSM4329_PLL 0
#define SSM4329_PLL_SRC_MCLKIN	0
#define SSM4329_PLL_SRC_FSYNC1	1
#define SSM4329_PLL_SRC_BCLK1	2
#define SSM4329_PLL_SRC_FSYNC2	3
#define SSM4329_PLL_SRC_BCLK2	4

struct device;

int ssm4329_probe(struct device *dev, struct regmap *regmap);

extern const struct regmap_config ssm4329_regmap_config;

#endif
