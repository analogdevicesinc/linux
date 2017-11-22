/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include "mxc-mipi-csi2.h"
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

#define MXC_MIPI_CSI2_YAV_DRIVER_NAME	"mxc-mipi-csi2_yav"
#define MXC_MIPI_CSI2_YAV_SUBDEV_NAME	MXC_MIPI_CSI2_DRIVER_NAME

#define	GPR_CSI2_1_RX_ENABLE		BIT(13)
#define	GPR_CSI2_1_VID_INTFC_ENB	BIT(12)
#define	GPR_CSI2_1_PD_RX 		BIT(11)
#define	GPR_CSI2_1_HSEL			BIT(10)
#define	GPR_CSI2_1_AUTO_PD_EN 		BIT(9)
#define	GPR_CSI2_1_CONT_CLK_MODE 	BIT(8)
#define	GPR_CSI2_1_S_PRG_RXHS_SETTLE(x)	(((x) & 0x3F) << 2)
#define	GPR_CSI2_1_RX_RCAL		(3)

static struct mxc_mipi_csi2_dev *sd_to_mxc_mipi_csi2_dev(struct v4l2_subdev
							 *sdev)
{
	return container_of(sdev, struct mxc_mipi_csi2_dev, sd);
}

#ifdef debug
static void mxc_mipi_csi2_reg_dump(struct mxc_mipi_csi2_dev *csi2dev)
{
	printk("MIPI CSI2 HC register dump, mipi csi%d\n", csi2dev->id);
	printk("MIPI CSI2 HC num of lanes     0x100 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x100));
	printk("MIPI CSI2 HC dis lanes        0x104 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x104));
	printk("MIPI CSI2 HC BIT ERR          0x108 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x108));
	printk("MIPI CSI2 HC IRQ STATUS       0x10C = 0x%x\n",
	       readl(csi2dev->base_regs + 0x10C));
	printk("MIPI CSI2 HC IRQ MASK         0x110 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x110));
	printk("MIPI CSI2 HC ULPS STATUS      0x114 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x114));
	printk("MIPI CSI2 HC DPHY ErrSotHS    0x118 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x118));
	printk("MIPI CSI2 HC DPHY ErrSotSync  0x11c = 0x%x\n",
	       readl(csi2dev->base_regs + 0x11c));
	printk("MIPI CSI2 HC DPHY ErrEsc      0x120 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x120));
	printk("MIPI CSI2 HC DPHY ErrSyncEsc  0x124 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x124));
	printk("MIPI CSI2 HC DPHY ErrControl  0x128 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x128));
	printk("MIPI CSI2 HC DISABLE_PAYLOAD  0x12C = 0x%x\n",
	       readl(csi2dev->base_regs + 0x12C));
	printk("MIPI CSI2 HC DISABLE_PAYLOAD  0x130 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x130));
	printk("MIPI CSI2 HC IGNORE_VC        0x180 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x180));
	printk("MIPI CSI2 HC VID_VC           0x184 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x184));
	printk("MIPI CSI2 HC FIFO_SEND_LEVEL  0x188 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x188));
	printk("MIPI CSI2 HC VID_VSYNC        0x18C = 0x%x\n",
	       readl(csi2dev->base_regs + 0x18C));
	printk("MIPI CSI2 HC VID_SYNC_FP      0x190 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x190));
	printk("MIPI CSI2 HC VID_HSYNC        0x194 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x194));
	printk("MIPI CSI2 HC VID_HSYNC_BP     0x198 = 0x%x\n",
	       readl(csi2dev->base_regs + 0x198));
}
#else
static void mxc_mipi_csi2_reg_dump(struct mxc_mipi_csi2_dev *csi2dev)
{
}
#endif

static int mxc_mipi_csi2_phy_reset(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	phandle phandle;
	u32 out_val[3];
	int ret;

	ret = of_property_read_u32_array(np, "csis-phy-reset", out_val, 3);
	if (ret) {
		dev_info(dev, "no csis-hw-reset property found\n");
	} else {
		phandle = *out_val;

		node = of_find_node_by_phandle(phandle);
		if (!node) {
			dev_dbg(dev, "not find src node by phandle\n");
			ret = PTR_ERR(node);
		}
		csi2dev->hw_reset.src = syscon_node_to_regmap(node);
		if (IS_ERR(csi2dev->hw_reset.src)) {
			dev_err(dev, "failed to get src regmap\n");
			ret = PTR_ERR(csi2dev->hw_reset.src);
		}
		of_node_put(node);
		if (ret < 0)
			return ret;

		csi2dev->hw_reset.req_src = out_val[1];
		csi2dev->hw_reset.rst_val = out_val[2];

		/* reset mipi phy */
		regmap_update_bits(csi2dev->hw_reset.src,
				   csi2dev->hw_reset.req_src,
				   csi2dev->hw_reset.rst_val,
				   csi2dev->hw_reset.rst_val);
		msleep(20);
	}

	return ret;
}

static int mxc_mipi_csi2_phy_gpr(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	phandle phandle;
	u32 out_val[2];
	int ret;

	ret = of_property_read_u32_array(np, "phy-gpr", out_val, 2);
	if (ret) {
		dev_dbg(dev, "no phy-gpr property found\n");
	} else {
		phandle = *out_val;

		node = of_find_node_by_phandle(phandle);
		if (!node) {
			dev_dbg(dev, "not find gpr node by phandle\n");
			ret = PTR_ERR(node);
		}
		csi2dev->phy_gpr.gpr = syscon_node_to_regmap(node);
		if (IS_ERR(csi2dev->phy_gpr.gpr)) {
			dev_err(dev, "failed to get gpr regmap\n");
			ret = PTR_ERR(csi2dev->phy_gpr.gpr);
		}
		of_node_put(node);
		if (ret < 0)
			return ret;

		csi2dev->phy_gpr.req_src = out_val[1];

		regmap_update_bits(csi2dev->phy_gpr.gpr,
				   csi2dev->phy_gpr.req_src,
				   0x3FFF,
				   GPR_CSI2_1_RX_ENABLE |
				   GPR_CSI2_1_VID_INTFC_ENB |
				   GPR_CSI2_1_HSEL |
				   GPR_CSI2_1_CONT_CLK_MODE |
				   GPR_CSI2_1_S_PRG_RXHS_SETTLE(9));
	}

	return ret;
}

static void mxc_mipi_csi2_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	mxc_mipi_csi2_phy_gpr(csi2dev);
}

static void mxc_mipi_csi2_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	/* Disable Data lanes */
	writel(0xf, csi2dev->base_regs + CSI2RX_CFG_DISABLE_DATA_LANES);
}

static void mxc_mipi_csi2_hc_config(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val0, val1;
	u32 i;

	val0 = 0;

	/* Lanes */
	writel(csi2dev->num_lanes - 1,
	       csi2dev->base_regs + CSI2RX_CFG_NUM_LANES);

	for (i = 0; i < csi2dev->num_lanes; i++)
		val0 |= (1 << (csi2dev->data_lanes[i] - 1));

	val1 = 0xF & ~val0;
	writel(val1, csi2dev->base_regs + CSI2RX_CFG_DISABLE_DATA_LANES);

	/* Mask interrupt */
	writel(0x1FF, csi2dev->base_regs + CSI2RX_IRQ_MASK);

	writel(1, csi2dev->base_regs + 0x180);
	/* vid_vc */
	writel(1, csi2dev->base_regs + 0x184);
	writel(0x300, csi2dev->base_regs + 0x188);
}

static int mipi_csi2_clk_init(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;

	csi2dev->clk_apb = devm_clk_get(dev, "clk_apb");
	if (IS_ERR(csi2dev->clk_apb)) {
		dev_err(dev, "failed to get csi apb clk\n");
		return PTR_ERR(csi2dev->clk_apb);
	}

	csi2dev->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(csi2dev->clk_core)) {
		dev_err(dev, "failed to get csi core clk\n");
		return PTR_ERR(csi2dev->clk_core);
	}

	csi2dev->clk_esc = devm_clk_get(dev, "clk_esc");
	if (IS_ERR(csi2dev->clk_esc)) {
		dev_err(dev, "failed to get csi esc clk\n");
		return PTR_ERR(csi2dev->clk_esc);
	}

	csi2dev->clk_pxl = devm_clk_get(dev, "clk_pxl");
	if (IS_ERR(csi2dev->clk_pxl)) {
		dev_err(dev, "failed to get csi pixel link clk\n");
		return PTR_ERR(csi2dev->clk_pxl);
	}

	return 0;
}

static int mipi_csi2_clk_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	int ret;

	ret = clk_prepare_enable(csi2dev->clk_apb);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_apb error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_core);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_core error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_esc);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_esc error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_pxl error\n", __func__);
		return ret;
	}
	return ret;
}

static void mipi_csi2_clk_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	clk_disable_unprepare(csi2dev->clk_apb);
	clk_disable_unprepare(csi2dev->clk_core);
	clk_disable_unprepare(csi2dev->clk_esc);
	clk_disable_unprepare(csi2dev->clk_pxl);
}

static int mipi_csi2_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

/*
 * V4L2 subdev operations
 */
static int mipi_csi2_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int mipi_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct device *dev = &csi2dev->pdev->dev;
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;
	int ret = 0;

	dev_dbg(&csi2dev->pdev->dev, "%s: %d, csi2dev: 0x%x\n",
		__func__, enable, csi2dev->flags);

	if (enable) {
		if (!csi2dev->running) {
			pm_runtime_get_sync(dev);
			mxc_mipi_csi2_phy_reset(csi2dev);
			mxc_mipi_csi2_hc_config(csi2dev);
			mxc_mipi_csi2_enable(csi2dev);
			mxc_mipi_csi2_reg_dump(csi2dev);
		}
		v4l2_subdev_call(sensor_sd, video, s_stream, true);
		csi2dev->running++;

	} else {

		v4l2_subdev_call(sensor_sd, video, s_stream, false);
		csi2dev->running--;
		if (!csi2dev->running) {
			pm_runtime_put(dev);
			mxc_mipi_csi2_disable(csi2dev);
		}
	}

	return ret;
}

static int mipi_csis_enum_framesizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;

	return v4l2_subdev_call(sensor_sd, pad, enum_frame_size, NULL, fse);
}

static int mipi_csis_enum_frameintervals(struct v4l2_subdev *sd,
					 struct v4l2_subdev_pad_config *cfg,
					 struct v4l2_subdev_frame_interval_enum
					 *fie)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;

	return v4l2_subdev_call(sensor_sd, pad, enum_frame_interval, NULL, fie);
}

static int mipi_csis_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;

	return v4l2_subdev_call(sensor_sd, pad, enum_mbus_code, NULL, code);
}

static int mipi_csi2_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;

	if (fmt->pad)
		return -EINVAL;

	return v4l2_subdev_call(sensor_sd, pad, get_fmt, NULL, fmt);
}

static int mipi_csi2_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	/* TODO */
	return 0;
}

static int mipi_csis_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;

	return v4l2_subdev_call(sensor_sd, video, s_parm, a);
}

static int mipi_csis_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sensor_sd = csi2dev->sensor_sd;

	return v4l2_subdev_call(sensor_sd, video, g_parm, a);
}

static const struct v4l2_subdev_internal_ops mipi_csi2_sd_internal_ops = {
	.open = mipi_csi2_open,
};

static struct v4l2_subdev_pad_ops mipi_csi2_pad_ops = {
	.enum_frame_size = mipi_csis_enum_framesizes,
	.enum_frame_interval = mipi_csis_enum_frameintervals,
	.enum_mbus_code = mipi_csis_enum_mbus_code,
	.get_fmt = mipi_csi2_get_fmt,
	.set_fmt = mipi_csi2_set_fmt,
};

static struct v4l2_subdev_core_ops mipi_csi2_core_ops = {
	.s_power = mipi_csi2_s_power,
};

static struct v4l2_subdev_video_ops mipi_csi2_video_ops = {
	.s_stream = mipi_csi2_s_stream,

	.s_parm = mipi_csis_s_parm,
	.g_parm = mipi_csis_g_parm,
};

static struct v4l2_subdev_ops mipi_csi2_subdev_ops = {
	.core = &mipi_csi2_core_ops,
	.video = &mipi_csi2_video_ops,
	.pad = &mipi_csi2_pad_ops,
};

static int mipi_csi2_parse_dt(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_fwnode_endpoint endpoint;
	u32 i;

	csi2dev->id = of_alias_get_id(node, "csi");

	csi2dev->vchannel = of_property_read_bool(node, "virtual-channel");

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(dev, "No port node at %s\n", node->full_name);
		return -EINVAL;
	}

	/* Get port node */
	v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &endpoint);

	csi2dev->num_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < csi2dev->num_lanes; i++)
		csi2dev->data_lanes[i] = endpoint.bus.mipi_csi2.data_lanes[i];

	of_node_put(node);

	return 0;
}

static inline struct mxc_mipi_csi2_dev
*notifier_to_mipi_dev(struct v4l2_async_notifier *n)
{
	return container_of(n, struct mxc_mipi_csi2_dev, subdev_notifier);
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct mxc_mipi_csi2_dev *csi2dev = notifier_to_mipi_dev(notifier);

	/* Find platform data for this sensor subdev */
	if (csi2dev->asd.match.fwnode.fwnode == of_fwnode_handle(subdev->dev->of_node))
		csi2dev->sensor_sd = subdev;

	if (subdev == NULL)
		return -EINVAL;

	v4l2_info(&csi2dev->v4l2_dev, "Registered sensor subdevice: %s\n",
		  subdev->name);

	return 0;
}

static int mipi_csis_subdev_host(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_node *parent = dev->of_node;
	struct device_node *node, *port, *rem;
	int ret;

	/* Attach sensors linked to csi receivers */
	for_each_available_child_of_node(parent, node) {
		if (of_node_cmp(node->name, "port"))
			continue;

		/* The csi node can have only port subnode. */
		port = of_get_next_child(node, NULL);
		if (!port)
			continue;
		rem = of_graph_get_remote_port_parent(port);
		of_node_put(port);
		if (rem == NULL) {
			v4l2_info(&csi2dev->v4l2_dev,
				  "Remote device at %s not found\n",
				  port->full_name);
			return -1;
		} else
			v4l2_info(&csi2dev->v4l2_dev,
				  "Remote device at %s XXX found\n",
				  port->full_name);

		csi2dev->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		csi2dev->asd.match.fwnode.fwnode = of_fwnode_handle(rem);
		csi2dev->async_subdevs[0] = &csi2dev->asd;

		of_node_put(rem);
		break;
	}

	csi2dev->subdev_notifier.subdevs = csi2dev->async_subdevs;
	csi2dev->subdev_notifier.num_subdevs = 1;
	csi2dev->subdev_notifier.bound = subdev_notifier_bound;

	ret = v4l2_async_notifier_register(&csi2dev->v4l2_dev,
					   &csi2dev->subdev_notifier);
	if (ret)
		dev_err(dev,
			"Error register async notifier regoster, ret %d\n",
			ret);

	return ret;
}

static int mipi_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem_res;
	struct mxc_mipi_csi2_dev *csi2dev;
	int ret = -ENOMEM;

	dev_info(&pdev->dev, "%s\n", __func__);
	csi2dev = devm_kzalloc(dev, sizeof(*csi2dev), GFP_KERNEL);
	if (!csi2dev)
		return -ENOMEM;

	csi2dev->pdev = pdev;

	ret = mipi_csi2_parse_dt(csi2dev);
	if (ret < 0)
		return -EINVAL;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi2dev->base_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(csi2dev->base_regs)) {
		dev_err(dev, "Failed to get mipi csi2 HC register\n");
		return PTR_ERR(csi2dev->base_regs);
	}

	ret = mipi_csi2_clk_init(csi2dev);
	if (ret < 0)
		return -EINVAL;

	v4l2_subdev_init(&csi2dev->sd, &mipi_csi2_subdev_ops);

	csi2dev->sd.owner = THIS_MODULE;
	snprintf(csi2dev->sd.name, sizeof(csi2dev->sd.name), "%s.%d",
		 MXC_MIPI_CSI2_YAV_SUBDEV_NAME, csi2dev->id);

	csi2dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2dev->sd.dev = &pdev->dev;

	/* First register a v4l2 device */
	ret = v4l2_device_register(dev, &csi2dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register v4l2 device.\n");
		return -EINVAL;
	}
	ret = v4l2_async_register_subdev(&csi2dev->sd);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s--Async register faialed, ret=%d\n",
			__func__, ret);
		goto e_v4l_dev;
	}

	ret = mipi_csis_subdev_host(csi2dev);
	if (ret < 0)
		goto e_clkdis;

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&csi2dev->sd, pdev);

	/* .. and a pointer to the subdev. */
	platform_set_drvdata(pdev, csi2dev);

	ret = mipi_csi2_clk_enable(csi2dev);
	if (ret < 0)
		goto e_clkdis;

	dev_info(&pdev->dev, "lanes: %d, name: %s\n",
		 csi2dev->num_lanes, csi2dev->sd.name);

	csi2dev->running = 0;
	csi2dev->flags = MXC_MIPI_CSI2_PM_POWERED;
	pm_runtime_enable(&pdev->dev);

	return 0;

e_clkdis:
	v4l2_async_unregister_subdev(&csi2dev->sd);
e_v4l_dev:
	v4l2_device_unregister(&csi2dev->v4l2_dev);
	return ret;
}

static int mipi_csi2_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);

	mipi_csi2_clk_disable(csi2dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int mipi_csi2_pm_runtime_resume(struct device *dev)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);
	int ret;

	ret = mipi_csi2_clk_enable(csi2dev);
	if (ret < 0) {
		dev_info(dev, "%s:%d fail\n", __func__, __LINE__);
		return -EAGAIN;
	}

	return 0;
}

static int mipi_csi2_runtime_pm_suspend(struct device *dev)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);

	mipi_csi2_clk_disable(csi2dev);

	return 0;
}

static int mipi_csi2_pm_suspend(struct device *dev)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = &csi2dev->sd;

	if (csi2dev->flags & MXC_MIPI_CSI2_PM_SUSPENDED)
		return 0;

	if (csi2dev->running)
		mipi_csi2_s_stream(sd, false);
	mipi_csi2_clk_disable(csi2dev);
	csi2dev->flags &= ~MXC_MIPI_CSI2_PM_POWERED;
	csi2dev->flags |= MXC_MIPI_CSI2_PM_SUSPENDED;

	return 0;
}

static int mipi_csi2_pm_resume(struct device *dev)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = &csi2dev->sd;
	int ret;

	if (csi2dev->flags & MXC_MIPI_CSI2_PM_POWERED)
		return 0;

	ret = mipi_csi2_clk_enable(csi2dev);
	if (ret < 0) {
		dev_info(dev, "%s:%d fail\n", __func__, __LINE__);
		return -EAGAIN;
	}

	if (csi2dev->running)
		mipi_csi2_s_stream(sd, true);
	csi2dev->flags |= MXC_MIPI_CSI2_PM_POWERED;
	csi2dev->flags &= ~MXC_MIPI_CSI2_PM_SUSPENDED;

	return 0;
}

static const struct dev_pm_ops mipi_csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mipi_csi2_pm_suspend, mipi_csi2_pm_resume)
	SET_RUNTIME_PM_OPS(mipi_csi2_runtime_pm_suspend,
				mipi_csi2_pm_runtime_resume,
				NULL)
};

static const struct of_device_id mipi_csi2_of_match[] = {
	{.compatible = "fsl,mxc-mipi-csi2_yav",},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mipi_csi2_of_match);

static struct platform_driver mipi_csi2_driver = {
	.driver = {
		   .name = MXC_MIPI_CSI2_YAV_DRIVER_NAME,
		   .of_match_table = mipi_csi2_of_match,
		   .pm = &mipi_csi_pm_ops,
		   },
	.probe = mipi_csi2_probe,
	.remove = mipi_csi2_remove,
};

module_platform_driver(mipi_csi2_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC MIPI CSI2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_MIPI_CSI2_YAV_DRIVER_NAME);
