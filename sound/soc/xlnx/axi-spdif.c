/*
 *  Copyright (C) 2012, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/dma-mapping.h>
#include <linux/debugfs.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

struct axi_spdif {
	void __iomem *base;

	unsigned int clock;

	struct debugfs_regset32 regset;
};

#define AXI_SPDIF_REG_CTRL 0x0
#define AXI_SPDIF_REG_STAT 0x4

#define AXI_SPDIF_CTRL_TXDATA BIT(1)
#define AXI_SPDIF_CTRL_TXEN BIT(0)
#define AXI_SPDIF_CTRL_RATIO_OFFSET 8
#define AXI_SPDIF_CTRL_RATIO_MASK (0xff << 8)

static struct debugfs_reg32 axi_spdif_debugfs_regs[] = {
    { "Control", AXI_SPDIF_REG_CTRL },
    { "Status", AXI_SPDIF_REG_STAT },
};

static inline uint32_t axi_spdif_read(const struct axi_spdif *spdif,
	unsigned int reg)
{
	return readl(spdif->base + reg);
}

static inline void axi_spdif_write(const struct axi_spdif *spdif,
	unsigned int reg, uint32_t value)
{
	writel(value, spdif->base + reg);
}

static int axi_spdif_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct axi_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	uint32_t ctrl = axi_spdif_read(spdif, AXI_SPDIF_REG_CTRL);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ctrl |= AXI_SPDIF_CTRL_TXDATA;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ctrl &= ~AXI_SPDIF_CTRL_TXDATA;
		break;
	default:
		return -EINVAL;
	}

	axi_spdif_write(spdif, AXI_SPDIF_REG_CTRL, ctrl);

	return 0;
}

static int axi_spdif_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct axi_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	uint32_t ctrl = axi_spdif_read(spdif, AXI_SPDIF_REG_CTRL);
	unsigned int ratio;

	ratio = DIV_ROUND_CLOSEST(spdif->clock, (params_rate(params) * 64 * 2)) - 1;

	ctrl &= ~AXI_SPDIF_CTRL_RATIO_MASK;
	ctrl |= ratio << AXI_SPDIF_CTRL_RATIO_OFFSET;

	axi_spdif_write(spdif, AXI_SPDIF_REG_CTRL, ctrl);

	return 0;
}
/*
static int axi_spdif_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct axi_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	uint32_t ctrl;

	if (dai->active)
		return 0;

	ctrl = axi_spdif_read(spdif, AXI_SPDIF_REG_CTRL);
	ctrl |= AXI_SPDIF_CTRL_TXEN;
	axi_spdif_write(spdif, AXI_SPDIF_REG_CTRL, ctrl);

	return 0;
}

static void axi_spdif_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct axi_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	uint32_t ctrl;

	if (dai->active)
		return;

	ctrl = axi_spdif_read(spdif, AXI_SPDIF_REG_CTRL);
	ctrl &= ~AXI_SPDIF_CTRL_TXEN;
	axi_spdif_write(spdif, AXI_SPDIF_REG_CTRL, ctrl);
}
*/
static const struct snd_soc_dai_ops axi_spdif_dai_ops = {
	.trigger = axi_spdif_trigger,
	.hw_params = axi_spdif_hw_params,
};

static struct snd_soc_dai_driver axi_spdif_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &axi_spdif_dai_ops,
};

static int __devinit axi_spdif_probe(struct platform_device *pdev)
{
	struct axi_spdif *spdif;
	struct resource *res;
	int ret;

	spdif = devm_kzalloc(&pdev->dev, sizeof(*spdif), GFP_KERNEL);

	if (!spdif)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	spdif->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!spdif->base)
		return -EBUSY;

	platform_set_drvdata(pdev, spdif);

	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &spdif->clock);

	ret = snd_soc_register_dai(&pdev->dev, &axi_spdif_dai);

	if (ret) {
		dev_err(&pdev->dev, "Failed to register DAI\n");
		return ret;
	}

	spdif->regset.base = spdif->base;
	spdif->regset.regs = axi_spdif_debugfs_regs;
	spdif->regset.nregs = ARRAY_SIZE(axi_spdif_debugfs_regs);

	debugfs_create_regset32(dev_name(&pdev->dev), 0777, NULL, &spdif->regset);

	axi_spdif_write(spdif, AXI_SPDIF_REG_CTRL, AXI_SPDIF_CTRL_TXEN);

	return 0;
}

static int __devexit axi_spdif_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);

	return 0;
}

static const struct of_device_id axi_spdif_of_match[] __devinitconst = {
	{ .compatible = "adi,axi-spdif-tx-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, axi_spdif_of_match);

static struct platform_driver axi_spdif_driver = {
	.driver = {
		.name = "axi-spdif",
		.owner = THIS_MODULE,
		.of_match_table = axi_spdif_of_match,
	},
	.probe = axi_spdif_probe,
	.remove = __devexit_p(axi_spdif_dev_remove),
};
module_platform_driver(axi_spdif_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("AXI SPDIF driver");
MODULE_LICENSE("GPL");
