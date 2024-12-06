// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023 NXP
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>

#define PIXEL_INTERLEAVER_CTRL	0x4
#define  DISP_IN_SEL		BIT(1)
#define  MODE			BIT(0)

#define CTRL(n)			(0x0 + 0x10000 * (n))
#define  YUV_CONV		BIT(13)
#define  PIXEL_FORMAT		BIT(12)
#define  DE_POLARITY		BIT(11)
#define  VSYNC_POLARITY		BIT(10)
#define  HSYNC_POLARITY		BIT(9)

#define SWRST(n)		(0x20 + 0x10000 * (n))
#define  SW_RST			BIT(1)

#define IE(n)			(0x30 + 0x10000 * (n))
#define IS(n)			(0x40 + 0x10000 * (n))
#define  FOVF(n)		BIT(0 + 1 * (n))

#define ICTRL(n)		(0x50 + 0x10000 * (n))
#define  WIDTH_MASK		GENMASK(11, 0)
#define  WIDTH(n)		FIELD_PREP(WIDTH_MASK, (n))

#define STREAMS			2
#define NO_INTER_STREAM		2

#define DRIVER_NAME		"imx95-pixel-interleaver"

enum imx95_pinter_mode {
	BYPASS,
	STREAM0_SPLIT2,
	STREAM1_SPLIT2,
};

struct imx95_pinter_channel {
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct imx95_pinter *pinter;
	unsigned int sid;	/* stream id */
	bool is_available;
};

struct imx95_pinter {
	struct device *dev;
	void __iomem *regs;
	struct regmap *regmap;
	struct clk *clk_bus;
	unsigned int irq;
	struct imx95_pinter_channel ch[STREAMS];
	enum imx95_pinter_mode mode;
};

static void imx95_pinter_sw_reset(struct imx95_pinter_channel *ch)
{
	struct imx95_pinter *pinter = ch->pinter;

	clk_prepare_enable(pinter->clk_bus);

	writel(SW_RST, pinter->regs + SWRST(ch->sid));
	usleep_range(10, 20);
	writel(0, pinter->regs + SWRST(ch->sid));

	clk_disable_unprepare(pinter->clk_bus);
}

static int imx95_pinter_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct imx95_pinter_channel *ch = bridge->driver_private;
	struct imx95_pinter *pinter = ch->pinter;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
		dev_err(pinter->dev, "do not support creating a drm_connector\n");
		return -EINVAL;
	}

	if (!bridge->encoder) {
		dev_err(pinter->dev, "missing encoder\n");
		return -ENODEV;
	}

	return drm_bridge_attach(bridge->encoder, ch->next_bridge, bridge,
				 DRM_BRIDGE_ATTACH_NO_CONNECTOR);
}

static void
imx95_pinter_bridge_mode_set(struct drm_bridge *bridge,
			     const struct drm_display_mode *mode,
			     const struct drm_display_mode *adjusted_mode)
{
	struct imx95_pinter_channel *ch = bridge->driver_private;
	struct imx95_pinter *pinter = ch->pinter;
	u32 val;

	imx95_pinter_sw_reset(ch);

	clk_prepare_enable(pinter->clk_bus);

	/* HSYNC and VSYNC are active low. Data Enable is active high */
	val = HSYNC_POLARITY | VSYNC_POLARITY;
	writel(val, pinter->regs + CTRL(ch->sid));

	/* set width if interleaving is used */
	if (pinter->mode != BYPASS)
		writel(WIDTH(mode->hdisplay / 2), pinter->regs + ICTRL(ch->sid));

	/* enable interrupts */
	writel(FOVF(0) | FOVF(1), pinter->regs + IE(ch->sid));

	clk_disable_unprepare(pinter->clk_bus);
}

static void imx95_pinter_bridge_enable(struct drm_bridge *bridge)
{
	struct imx95_pinter_channel *ch = bridge->driver_private;
	struct imx95_pinter *pinter = ch->pinter;

	switch (pinter->mode) {
	case BYPASS:
		regmap_write(pinter->regmap, PIXEL_INTERLEAVER_CTRL, 0);
		break;
	case STREAM0_SPLIT2:
		regmap_write(pinter->regmap, PIXEL_INTERLEAVER_CTRL, MODE);
		break;
	case STREAM1_SPLIT2:
		regmap_write(pinter->regmap, PIXEL_INTERLEAVER_CTRL,
			     MODE | DISP_IN_SEL);
		break;
	}
}

static u32 *
imx95_pinter_bridge_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
					      struct drm_bridge_state *bridge_state,
					      struct drm_crtc_state *crtc_state,
					      struct drm_connector_state *conn_state,
					      u32 output_fmt,
					      unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	if (output_fmt != MEDIA_BUS_FMT_RGB888_1X24)
		return NULL;

	*num_input_fmts = 1;

	input_fmts = kmalloc(sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	input_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;

	return input_fmts;
}

static const struct drm_bridge_funcs imx95_pinter_bridge_funcs = {
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		= drm_atomic_helper_bridge_reset,
	.attach			= imx95_pinter_bridge_attach,
	.mode_set		= imx95_pinter_bridge_mode_set,
	.enable			= imx95_pinter_bridge_enable,
	.atomic_get_input_bus_fmts =
				imx95_pinter_bridge_atomic_get_input_bus_fmts,
};

static irqreturn_t pinter_irq_handler(int irq, void *data)
{
	struct imx95_pinter *pinter = data;
	struct device *dev = pinter->dev;
	u32 val;
	int i;

	clk_prepare_enable(pinter->clk_bus);

	for (i = 0; i < STREAMS; i++) {
		/* disable interrupts */
		writel(0, pinter->regs + IE(i));

		/* report errors */
		val = readl(pinter->regs + IS(i));
		if (val & FOVF(0))
			dev_err(dev, "stream%d FIFO0 overflows\n", i);
		if (val & FOVF(1))
			dev_err(dev, "stream%d FIFO1 overflows\n", i);

		/* write one to clear interrupt status */
		writel(val, pinter->regs + IS(i));
	}

	clk_disable_unprepare(pinter->clk_bus);

	return IRQ_HANDLED;
}

static int imx95_pinter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child, *remote;
	struct device_node *remote_port;
	struct imx95_pinter_channel *ch;
	struct imx95_pinter *pinter;
	u8 inter_stream;
	int ret;
	u32 i;

	pinter = devm_kzalloc(dev, sizeof(*pinter), GFP_KERNEL);
	if (!pinter)
		return -ENOMEM;

	pinter->dev = dev;
	platform_set_drvdata(pdev, pinter);

	pinter->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pinter->regs))
		return PTR_ERR(pinter->regs);

	pinter->regmap = syscon_regmap_lookup_by_phandle(np, "nxp,blk-ctrl");
	if (IS_ERR(pinter->regmap))
		return dev_err_probe(dev, PTR_ERR(pinter->regmap),
				     "failed to get blk-ctrl regmap\n");

	pinter->clk_bus = devm_clk_get(dev, NULL);
	if (IS_ERR(pinter->clk_bus))
		return dev_err_probe(dev, PTR_ERR(pinter->clk_bus),
				     "failed to get bus clock\n");

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	pinter->irq = ret;

	ret = devm_request_threaded_irq(dev, pinter->irq, NULL,
					pinter_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					dev_name(dev), pinter);
	if (ret < 0) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	inter_stream = NO_INTER_STREAM;
	ret = of_property_read_u8(np, "nxp,pixel-interleaving-stream-id",
				  &inter_stream);
	if (ret && ret != -EINVAL) {
		dev_err(dev, "failed to get interleaving stream ID: %d\n",
			ret);
		return ret;
	}

	switch (inter_stream) {
	case 0:
		pinter->mode = STREAM0_SPLIT2;
		break;
	case 1:
		pinter->mode = STREAM1_SPLIT2;
		break;
	default:
		pinter->mode = BYPASS;
		break;
	}

	for_each_available_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i > 1) {
			ret = -EINVAL;
			dev_err(dev, "invalid channel(%u) node address\n", i);
			goto free_child;
		}

		ch = &pinter->ch[i];
		ch->pinter = pinter;
		ch->sid = i;

		remote = of_graph_get_remote_node(child, 1, 0);
		if (!remote) {
			ret = -ENODEV;
			dev_err(dev,
				"channel%u failed to get port1's remote node: %d\n",
				i, ret);
			goto free_child;
		}

		remote_port = of_graph_get_port_by_id(remote, i);
		if (!remote_port) {
			of_node_put(remote);
			ret = -ENODEV;
			dev_err(dev, "failed to get remote port@%u\n", i);
			goto free_child;
		}

		of_node_put(remote);

		ch->next_bridge = of_drm_find_bridge(remote_port);
		if (!ch->next_bridge) {
			of_node_put(remote_port);
			ret = -EPROBE_DEFER;
			dev_dbg(dev, "channel%u failed to find next bridge: %d\n",
				i, ret);
			goto free_child;
		}

		of_node_put(remote_port);

		imx95_pinter_sw_reset(ch);

		ch->bridge.driver_private = ch;
		ch->bridge.funcs = &imx95_pinter_bridge_funcs;
		ch->bridge.of_node = child;
		ch->is_available = true;

		drm_bridge_add(&ch->bridge);
	}

	return 0;

free_child:
	of_node_put(child);

	if (i == 1 && pinter->ch[0].next_bridge)
		drm_bridge_remove(&pinter->ch[0].bridge);

	return ret;
}

static int imx95_pinter_remove(struct platform_device *pdev)
{
	struct imx95_pinter *pinter = platform_get_drvdata(pdev);
	struct imx95_pinter_channel *ch;
	int i;

	for (i = 0; i < 2; i++) {
		ch = &pinter->ch[i];

		if (!ch->is_available)
			continue;

		drm_bridge_remove(&ch->bridge);
		ch->is_available = false;
	}

	return 0;
}

static const struct of_device_id imx95_pinter_dt_ids[] = {
	{ .compatible = "nxp,imx95-pixel-interleaver", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx95_pinter_dt_ids);

static struct platform_driver imx95_pinter_driver = {
	.probe	= imx95_pinter_probe,
	.remove	= imx95_pinter_remove,
	.driver	= {
		.of_match_table = imx95_pinter_dt_ids,
		.name = DRIVER_NAME,
	},
};

module_platform_driver(imx95_pinter_driver);

MODULE_DESCRIPTION("i.MX95 pixel interleaver driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
