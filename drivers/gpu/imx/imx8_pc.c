/*
 * Copyright 2018,2019 NXP
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/imx8-pc.h>

#define REG0				0x0
#define PIX_COMBINE_ENABLE		BIT(0)
#define DISP_PIX_COMBINE_BYPASS(n)	BIT(1 + 21 * (n))
#define DISP_HSYNC_POLARITY(n)		BIT(2 + 11 * (n))
#define DISP_HSYNC_POLARITY_POS(n)	DISP_HSYNC_POLARITY(n)
#define DISP_VSYNC_POLARITY(n)		BIT(3 + 11 * (n))
#define DISP_VSYNC_POLARITY_POS(n)	DISP_VSYNC_POLARITY(n)
#define DISP_DVALID_POLARITY(n)		BIT(4 + 11 * (n))
#define DISP_DVALID_POLARITY_POS(n)	DISP_DVALID_POLARITY(n)
#define VSYNC_MASK_ENABLE		BIT(5)
#define SKIP_MODE			BIT(6)
#define SKIP_NUMBER(n)			(((n) & 0x3F) << 7)
#define DISP_PIX_DATA_FORMAT_MASK(n)    (0x7 << (16 + (n) * 3))
#define DISP_PIX_DATA_FORMAT_SHIFT(n)   (16 + (n) * 3)
enum {
	RGB = 0,
	YUV444,
	YUV422,
	SPLIT_RGB,
};

#define REG1				0x10
#define BUF_ACTIVE_DEPTH(n)		((n) & 0x7FF)

#define REG2				0x20
#define PC_SW_RESET_N			BIT(0)
#define DISP_SW_RESET_N(n)		BIT(1 + (n))
#define PC_FULL_RESET_N			(PC_SW_RESET_N |	\
					 DISP_SW_RESET_N(0) |	\
					 DISP_SW_RESET_N(1))

struct pc {
	struct device *dev;
	void __iomem *base;
	struct list_head list;
};

static DEFINE_MUTEX(pc_list_mutex);
static LIST_HEAD(pc_list);

static inline u32 pc_read(struct pc *pc, unsigned int offset)
{
	return readl(pc->base + offset);
}

static inline void pc_write(struct pc *pc, unsigned int offset, u32 value)
{
	writel(value, pc->base + offset);
}

static void pc_reset(struct pc *pc)
{
	pc_write(pc, REG2, 0);
	usleep_range(1000, 2000);
	pc_write(pc, REG2, PC_FULL_RESET_N);
}

void pc_enable(struct pc *pc)
{
	u32 val;

	if (WARN_ON(!pc))
		return;

	val = pc_read(pc, REG0);
	val |= PIX_COMBINE_ENABLE;
	pc_write(pc, REG0, val);

	dev_dbg(pc->dev, "enable\n");
}
EXPORT_SYMBOL_GPL(pc_enable);

void pc_disable(struct pc *pc)
{
	if (WARN_ON(!pc))
		return;

	pc_reset(pc);

	dev_dbg(pc->dev, "disable\n");
}
EXPORT_SYMBOL_GPL(pc_disable);

void pc_configure(struct pc *pc, unsigned int di, unsigned int frame_width,
		u32 mode, u32 format)
{
	u32 val;

	if (WARN_ON(!pc))
		return;

	if (WARN_ON(di != 0 && di != 1))
		return;

	dev_dbg(pc->dev, "configure mode-0x%08x frame_width-%u\n",
							mode, frame_width);

	val = pc_read(pc, REG0);
	if (mode == PC_BYPASS) {
		val |= DISP_PIX_COMBINE_BYPASS(di);
	} else if (mode == PC_COMBINE) {
		val &= ~DISP_PIX_COMBINE_BYPASS(di);
		frame_width /= 4;
	}

	pc_write(pc, REG0, val);
	pc_write(pc, REG1, BUF_ACTIVE_DEPTH(frame_width));
}
EXPORT_SYMBOL_GPL(pc_configure);

struct pc *pc_lookup_by_phandle(struct device *dev, const char *name)
{
	struct device_node *pc_node = of_parse_phandle(dev->of_node,
							name, 0);
	struct pc *pc;

	mutex_lock(&pc_list_mutex);
	list_for_each_entry(pc, &pc_list, list) {
		if (pc_node == pc->dev->of_node) {
			mutex_unlock(&pc_list_mutex);
			device_link_add(dev, pc->dev,
					DL_FLAG_AUTOREMOVE_CONSUMER);
			return pc;
		}
	}
	mutex_unlock(&pc_list_mutex);

	return NULL;
}
EXPORT_SYMBOL_GPL(pc_lookup_by_phandle);

static int pc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct pc *pc;
	u32 val;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	pc->dev = dev;
	platform_set_drvdata(pdev, pc);
	mutex_lock(&pc_list_mutex);
	list_add(&pc->list, &pc_list);
	mutex_unlock(&pc_list_mutex);

	pc_reset(pc);

	/*
	 * assume data enable is active high and HSYNC/VSYNC are active low
	 * also, bypass combine at startup
	 */
	val = DISP_DVALID_POLARITY_POS(0) | DISP_DVALID_POLARITY_POS(1) |
	      DISP_PIX_COMBINE_BYPASS(0)  | DISP_PIX_COMBINE_BYPASS(1)  |
	      VSYNC_MASK_ENABLE;

	pc_write(pc, REG0, val);

	return 0;
}

static int pc_remove(struct platform_device *pdev)
{
	struct pc *pc = platform_get_drvdata(pdev);

	mutex_lock(&pc_list_mutex);
	list_del(&pc->list);
	mutex_unlock(&pc_list_mutex);

	return 0;
}

static const struct of_device_id pc_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-pixel-combiner", },
	{ .compatible = "fsl,imx8qxp-pixel-combiner", },
	{ /* sentinel */ },
};

struct platform_driver pc_drv = {
	.probe = pc_probe,
	.remove = pc_remove,
	.driver = {
		.name = "imx8-pixel-combiner",
		.of_match_table = pc_dt_ids,
	},
};
module_platform_driver(pc_drv);

MODULE_DESCRIPTION("i.MX8 Pixel Combiner driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
