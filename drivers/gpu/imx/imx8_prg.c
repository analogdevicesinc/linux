/*
 * Copyright 2017-2019 NXP
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
#include <drm/drm_fourcc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <video/imx8-prefetch.h>

#define SET			0x4
#define CLR			0x8
#define TOG			0xc

#define PRG_CTRL		0x00
#define BYPASS			BIT(0)
#define SC_DATA_TYPE		BIT(2)
#define SC_DATA_TYPE_8BIT	0
#define SC_DATA_TYPE_10BIT	BIT(2)
#define UV_EN			BIT(3)
#define HANDSHAKE_MODE		BIT(4)
#define HANDSHAKE_MODE_4LINES	0
#define HANDSHAKE_MODE_8LINES	BIT(4)
#define SHADOW_LOAD_MODE	BIT(5)
#define DES_DATA_TYPE		0x30000
enum {
	DES_DATA_TYPE_32BPP = (0 << 16),
	DES_DATA_TYPE_24BPP = (1 << 16),
	DES_DATA_TYPE_16BPP = (2 << 16),
	DES_DATA_TYPE_8BPP = (3 << 16),
};
#define SOFTRST			BIT(30)
#define SHADOW_EN		BIT(31)

#define PRG_STATUS		0x10
#define BUFFER_VALID_B		BIT(1)
#define BUFFER_VALID_A		BIT(0)

#define PRG_REG_UPDATE		0x20
#define REG_UPDATE		BIT(0)

#define PRG_STRIDE		0x30
#define STRIDE(n)		(((n) - 1) & 0xffff)

#define PRG_HEIGHT		0x40
#define HEIGHT(n)		(((n) - 1) & 0xffff)

#define PRG_BADDR		0x50

#define PRG_OFFSET		0x60
#define Y(n)			(((n) & 0x7) << 16)
#define X(n)			((n) & 0xffff)

#define PRG_WIDTH		0x70
#define WIDTH(n)		(((n) - 1) & 0xffff)

struct prg {
	struct device *dev;
	void __iomem *base;
	struct list_head list;
	struct clk *clk_apb;
	struct clk *clk_rtram;
	bool is_auxiliary;
	bool is_blit;
};

static DEFINE_MUTEX(prg_list_mutex);
static LIST_HEAD(prg_list);

static inline u32 prg_read(struct prg *prg, unsigned int offset)
{
	return readl(prg->base + offset);
}

static inline void prg_write(struct prg *prg, u32 value, unsigned int offset)
{
	writel(value, prg->base + offset);
}

static void prg_reset(struct prg *prg)
{
	if (prg->is_blit)
		usleep_range(10, 20);

	prg_write(prg, SOFTRST, PRG_CTRL + SET);

	if (prg->is_blit)
		usleep_range(10, 20);
	else
		usleep_range(1000, 2000);

	prg_write(prg, SOFTRST, PRG_CTRL + CLR);
}

void prg_enable(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg_write(prg, BYPASS, PRG_CTRL + CLR);
}
EXPORT_SYMBOL_GPL(prg_enable);

void prg_disable(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg_write(prg, BYPASS, PRG_CTRL);
}
EXPORT_SYMBOL_GPL(prg_disable);

void prg_configure(struct prg *prg, unsigned int width, unsigned int height,
		   unsigned int x_offset, unsigned int y_offset,
		   unsigned int stride, unsigned int bits_per_pixel,
		   unsigned long baddr, u32 format, u64 modifier,
		   bool start)
{
	unsigned int burst_size;
	unsigned int mt_w = 0, mt_h = 0;	/* w/h in a micro-tile */
	unsigned long _baddr;
	u32 val;

	if (WARN_ON(!prg))
		return;

	if (start)
		prg_reset(prg);

	/* prg finer cropping into micro-tile block - top/left start point */
	switch (modifier) {
	case DRM_FORMAT_MOD_NONE:
		break;
	case DRM_FORMAT_MOD_AMPHION_TILED:
		mt_w = 8;
		mt_h = 8;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		mt_w = (bits_per_pixel == 16) ? 8 : 4;
		mt_h = 4;
		break;
	default:
		dev_err(prg->dev, "unsupported modifier 0x%016llx\n", modifier);
		return;
	}

	if (modifier) {
		x_offset %= mt_w;
		y_offset %= mt_h;

		/* consider x offset to calculate stride */
		_baddr = baddr + (x_offset * (bits_per_pixel / 8));
	} else {
		x_offset = 0;
		y_offset = 0;
		_baddr = baddr;
	}

	/*
	 * address TKT343664:
	 * fetch unit base address has to align to burst_size
	 */
	burst_size = 1 << (ffs(_baddr) - 1);
	burst_size = round_up(burst_size, 8);
	burst_size = min(burst_size, 128U);

	/*
	 * address TKT339017:
	 * fixup for burst size vs stride mismatch
	 */
	if (modifier)
		stride = round_up(stride + round_up(_baddr % 8, 8), burst_size);
	else
		stride = round_up(stride, burst_size);

	/*
	 * address TKT342628(part 1):
	 * when prg stride is less or equals to burst size,
	 * the auxiliary prg height needs to be a half
	 */
	if (prg->is_auxiliary && stride <= burst_size) {
		height /= 2;
		if (modifier)
			y_offset /= 2;
	}

	prg_write(prg, STRIDE(stride), PRG_STRIDE);
	prg_write(prg, WIDTH(width), PRG_WIDTH);
	prg_write(prg, HEIGHT(height), PRG_HEIGHT);
	prg_write(prg, X(x_offset) | Y(y_offset), PRG_OFFSET);
	prg_write(prg, baddr, PRG_BADDR);

	val = prg_read(prg, PRG_CTRL);
	val &= ~SC_DATA_TYPE;
	val |= SC_DATA_TYPE_8BIT;
	val &= ~HANDSHAKE_MODE;
	if (format == DRM_FORMAT_NV21 || format == DRM_FORMAT_NV12) {
		val |= HANDSHAKE_MODE_8LINES;
		/*
		 * address TKT342628(part 2):
		 * when prg stride is less or equals to burst size,
		 * we disable UV_EN bit for the auxiliary prg
		 */
		if (prg->is_auxiliary && stride > burst_size)
			val |= UV_EN;
		else
			val &= ~UV_EN;
	} else {
		val |= HANDSHAKE_MODE_4LINES;
		val &= ~UV_EN;
	}
	val |= SHADOW_LOAD_MODE;
	val &= ~DES_DATA_TYPE;
	switch (bits_per_pixel) {
	case 32:
		val |= DES_DATA_TYPE_32BPP;
		break;
	case 24:
		val |= DES_DATA_TYPE_24BPP;
		break;
	case 16:
		val |= DES_DATA_TYPE_16BPP;
		break;
	case 8:
		val |= DES_DATA_TYPE_8BPP;
		break;
	}
	if (start)
		/* no shadow for the first frame */
		val &= ~SHADOW_EN;
	else
		val |= SHADOW_EN;
	prg_write(prg, val, PRG_CTRL);

	dev_dbg(prg->dev, "bits per pixel %u\n", bits_per_pixel);
}
EXPORT_SYMBOL_GPL(prg_configure);

void prg_reg_update(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg_write(prg, REG_UPDATE, PRG_REG_UPDATE);
}
EXPORT_SYMBOL_GPL(prg_reg_update);

void prg_shadow_enable(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg_write(prg, SHADOW_EN, PRG_CTRL + SET);
}
EXPORT_SYMBOL_GPL(prg_shadow_enable);

bool prg_stride_supported(struct prg *prg, unsigned int stride)
{
	return stride < 0x10000;
}
EXPORT_SYMBOL_GPL(prg_stride_supported);

bool prg_stride_double_check(struct prg *prg,
			     unsigned int width, unsigned int x_offset,
			     unsigned int bits_per_pixel, u64 modifier,
			     unsigned int stride, dma_addr_t baddr)
{
	unsigned int burst_size;
	unsigned int mt_w = 0;	/* w in a micro-tile */
	dma_addr_t _baddr;

	if (WARN_ON(!prg))
		return false;

	/* prg finer cropping into micro-tile block - top/left start point */
	switch (modifier) {
	case DRM_FORMAT_MOD_NONE:
		break;
	case DRM_FORMAT_MOD_AMPHION_TILED:
		mt_w = 8;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		mt_w = (bits_per_pixel == 16) ? 8 : 4;
		break;
	default:
		dev_err(prg->dev, "unsupported modifier 0x%016llx\n", modifier);
		return false;
	}

	if (modifier) {
		x_offset %= mt_w;

		/* consider x offset to calculate stride */
		_baddr = baddr + (x_offset * (bits_per_pixel / 8));
	} else {
		_baddr = baddr;
	}

	/*
	 * address TKT343664:
	 * fetch unit base address has to align to burst size
	 */
	burst_size = 1 << (ffs(_baddr) - 1);
	burst_size = round_up(burst_size, 8);
	burst_size = min(burst_size, 128U);

	/*
	 * address TKT339017:
	 * fixup for burst size vs stride mismatch
	 */
	if (modifier)
		stride = round_up(stride + round_up(_baddr % 8, 8), burst_size);
	else
		stride = round_up(stride, burst_size);

	return stride < 0x10000;
}
EXPORT_SYMBOL_GPL(prg_stride_double_check);

void prg_set_auxiliary(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg->is_auxiliary = true;
}
EXPORT_SYMBOL_GPL(prg_set_auxiliary);

void prg_set_primary(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg->is_auxiliary = false;
}
EXPORT_SYMBOL_GPL(prg_set_primary);

void prg_set_blit(struct prg *prg)
{
	if (WARN_ON(!prg))
		return;

	prg->is_blit = true;
}
EXPORT_SYMBOL_GPL(prg_set_blit);

struct prg *
prg_lookup_by_phandle(struct device *dev, const char *name, int index)
{
	struct device_node *prg_node = of_parse_phandle(dev->of_node,
							name, index);
	struct prg *prg;

	mutex_lock(&prg_list_mutex);
	list_for_each_entry(prg, &prg_list, list) {
		if (prg_node == prg->dev->of_node) {
			mutex_unlock(&prg_list_mutex);
			device_link_add(dev, prg->dev,
					DL_FLAG_AUTOREMOVE_CONSUMER);
			return prg;
		}
	}
	mutex_unlock(&prg_list_mutex);

	return NULL;
}
EXPORT_SYMBOL_GPL(prg_lookup_by_phandle);

static const struct of_device_id prg_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-prg", },
	{ .compatible = "fsl,imx8qxp-prg", },
	{ /* sentinel */ },
};

static int prg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct prg *prg;

	prg = devm_kzalloc(dev, sizeof(*prg), GFP_KERNEL);
	if (!prg)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	prg->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(prg->base))
		return PTR_ERR(prg->base);

	prg->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(prg->clk_apb))
		return PTR_ERR(prg->clk_apb);
	clk_prepare_enable(prg->clk_apb);

	prg->clk_rtram = devm_clk_get(dev, "rtram");
	if (IS_ERR(prg->clk_rtram))
		return PTR_ERR(prg->clk_rtram);
	clk_prepare_enable(prg->clk_rtram);

	prg->dev = dev;
	platform_set_drvdata(pdev, prg);
	mutex_lock(&prg_list_mutex);
	list_add(&prg->list, &prg_list);
	mutex_unlock(&prg_list_mutex);

	prg_reset(prg);

	return 0;
}

static int prg_remove(struct platform_device *pdev)
{
	struct prg *prg = platform_get_drvdata(pdev);

	mutex_lock(&prg_list_mutex);
	list_del(&prg->list);
	mutex_unlock(&prg_list_mutex);

	clk_disable_unprepare(prg->clk_rtram);
	clk_disable_unprepare(prg->clk_apb);

	return 0;
}

struct platform_driver prg_drv = {
	.probe = prg_probe,
	.remove = prg_remove,
	.driver = {
		.name = "imx8-prg",
		.of_match_table = prg_dt_ids,
	},
};
module_platform_driver(prg_drv);

MODULE_DESCRIPTION("i.MX8 PRG driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
