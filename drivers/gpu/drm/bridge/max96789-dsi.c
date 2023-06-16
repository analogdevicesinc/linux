// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core MFD support for Analog Devices MAX96789 MIPI-DSI serializer
 *
 * Copyright 2023 NXP
 */

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/mfd/maxim_serdes.h>
#include <linux/mfd/max96789.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <video/videomode.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_module.h>

struct max96789_dsi {
	struct drm_bridge bridge;
	struct drm_bridge *remote_bridge;

	struct device *dev;
	struct max96789 *max96789_mfd;

	struct device_node *host_node;
	struct mipi_dsi_device *dsi;

	int data_lanes;

	enum max96789_dsi_port_id id;
};

static void max96789_dsi_setup_for_dual_link(struct max96789_dsi *max96789_dsi)
{
	struct max96789 *mfd = max96789_dsi->max96789_mfd;
	enum max96789_video_pipe_id video_pipe = VIDEO_PIPE_X;
	int reg_mask, reg_val;
	int shift;

	/* Clock select and start DSI port */
	reg_mask = CLK_SELX | CLK_SELY | CLK_SELZ | CLK_SELU;
	reg_mask |= (max96789_dsi->id == DSI_PORT_A ? START_PORTA : START_PORTB) | ENABLE_LINE_INFO;

	reg_val = max96789_dsi->id == DSI_PORT_A ? (CLK_SELY | CLK_SELZ | CLK_SELU) : CLK_SELX;
	reg_val |= max96789_dsi->id == DSI_PORT_A ? START_PORTA : START_PORTB;
	reg_val |= ENABLE_LINE_INFO;
	regmap_update_bits(mfd->regmap, MAX96789_FRONTTOP_PORT_SEL, reg_mask, reg_val);

	/* Start video */
	reg_val = max96789_dsi->id == DSI_PORT_A ? (START_PORTAX | START_PORTAZ) :
						   (START_PORTBX | START_PORTBZ);
	regmap_update_bits(mfd->regmap, MAX96789_FRONTTOP_VIDEO_PIPE_START, reg_val, reg_val);

	/* Set DSI Port Lane Mapping  */
	regmap_write(mfd->regmap, MAX96789_MIPI_RX_LANE_MAP(max96789_dsi->id),
		     max96789_dsi->id == DSI_PORT_A ? 0x4E : 0xE4);

	/* Number of Lanes */
	shift = max96789_dsi->id == DSI_PORT_A ? CTRL0_NUM_LANES_SHIFT : CTRL1_NUM_LANES_SHIFT;
	reg_val = (max96789_dsi->data_lanes - 1) << shift;
	reg_mask = max96789_dsi->id == DSI_PORT_A ? CTRL0_NUM_LANES_MASK : CTRL1_NUM_LANES_MASK;
	regmap_update_bits(mfd->regmap, MAX96789_MIPI_RX_LANES_NUM, reg_mask, reg_val);

	regmap_write(mfd->regmap, MAX96789_MIPI_RX_PHY_CFG,
		     max96789_dsi->id == DSI_PORT_A ? PHY_CONFIG_ONLY_PORT_A_EN :
						      PHY_CONFIG_ONLY_PORT_B_EN);

	regmap_update_bits(mfd->regmap, MAX96789_MIPI_DSI_CFG_ERR_CHK(max96789_dsi->id),
			   DISABLE_EOTP, DISABLE_EOTP);

	/* Set soft_dtx_en */
	regmap_write(mfd->regmap, MAX96789_FRONTTOP_PIPE_SW_OVR(video_pipe), 0x98);

	/* Set soft_dtx */
	regmap_write(mfd->regmap, MAX96789_FRONTTOP_PIPE_SW_OVR_VAL(video_pipe), 0x24);

	/* Set port to transmit packets from */
	regmap_write(mfd->regmap, MAX96789_CFGV_VIDEO_TX3(video_pipe), VID_TX_SPLT_MASK_A);

	/* FIFO/DESKEW_EN */
	regmap_write(mfd->regmap, MAX96789_MIPI_DSI_SKEW(max96789_dsi->id), 0xC1);

	/* Video Pipe Enable */
	regmap_update_bits(mfd->regmap, MAX96789_DEV_PIPE_EN, VID_TX_EN_X | 0x03,
			   VID_TX_EN_X | 0x03);
}

static void max96789_dsi_bridge_atomic_pre_enable(struct drm_bridge *bridge,
						  struct drm_bridge_state *old_bridge_state)
{
	struct max96789_dsi *max96789_dsi = container_of(bridge, struct max96789_dsi, bridge);
	struct max96789 *mfd = max96789_dsi->max96789_mfd;
	int reg_mask, reg_val;
	int shift;
	enum max96789_video_pipe_id video_pipe;

	if (mfd->gmsl2_dual_link) {
		max96789_dsi_setup_for_dual_link(max96789_dsi);
		return;
	}

	video_pipe = max96789_dsi->id == DSI_PORT_A ? VIDEO_PIPE_X : VIDEO_PIPE_Z;

	/* Clock select and start DSI port */
	reg_mask = video_pipe == VIDEO_PIPE_X ? (CLK_SELX | CLK_SELY) : (CLK_SELZ | CLK_SELU);
	reg_mask |= (max96789_dsi->id == DSI_PORT_A ? START_PORTA : START_PORTB) | ENABLE_LINE_INFO;

	reg_val = (video_pipe == VIDEO_PIPE_X ? 0 : (CLK_SELZ | CLK_SELU)) |
		  (max96789_dsi->id == DSI_PORT_A ? START_PORTA : START_PORTB) | ENABLE_LINE_INFO;
	regmap_update_bits(mfd->regmap, MAX96789_FRONTTOP_PORT_SEL, reg_mask, reg_val);

	/* Start video */
	reg_val = max96789_dsi->id == DSI_PORT_A ? START_PORTAX : START_PORTBZ;
	regmap_update_bits(mfd->regmap, MAX96789_FRONTTOP_VIDEO_PIPE_START, reg_val, reg_val);

	/* Set DSI Port Lane Mapping  */
	regmap_write(mfd->regmap, MAX96789_MIPI_RX_LANE_MAP(max96789_dsi->id),
		     max96789_dsi->id == DSI_PORT_A ? 0x4E : 0xE4);

	/* Number of Lanes */
	shift = max96789_dsi->id == DSI_PORT_A ? CTRL0_NUM_LANES_SHIFT : CTRL1_NUM_LANES_SHIFT;
	reg_val = (max96789_dsi->data_lanes - 1) << shift;
	reg_mask = max96789_dsi->id == DSI_PORT_A ? CTRL0_NUM_LANES_MASK : CTRL1_NUM_LANES_MASK;
	regmap_update_bits(mfd->regmap, MAX96789_MIPI_RX_LANES_NUM, reg_mask, reg_val);

	/* Set phy_config */
	regmap_write(mfd->regmap, MAX96789_MIPI_RX_PHY_CFG, PHY_CONFIG_BOTH_PORTS_EN);

	regmap_update_bits(mfd->regmap, MAX96789_MIPI_DSI_CFG_ERR_CHK(max96789_dsi->id),
			   DISABLE_EOTP, DISABLE_EOTP);

	/* Set soft_dtx_en */
	regmap_write(mfd->regmap, MAX96789_FRONTTOP_PIPE_SW_OVR(video_pipe), 0x98);

	/* Set soft_dtx */
	regmap_write(mfd->regmap, MAX96789_FRONTTOP_PIPE_SW_OVR_VAL(video_pipe), 0x24);

	/* Set port to transmit packets from */
	regmap_write(mfd->regmap, MAX96789_CFGV_VIDEO_TX3(video_pipe),
		     video_pipe == VIDEO_PIPE_X ? 0x10 : 0x20);

	/* FIFO/DESKEW_EN */
	regmap_write(mfd->regmap, MAX96789_MIPI_DSI_SKEW(max96789_dsi->id), 0xC1);

	/* Video Pipe Enable */
	/*
	 * TODO: find out why we need to write the 2 LSB reserved bits. Without setting them,
	 * changing modes does not work properly.
	 */
	regmap_update_bits(mfd->regmap, MAX96789_DEV_PIPE_EN,
			   (video_pipe == VIDEO_PIPE_X ? VID_TX_EN_X : VID_TX_EN_Z) | 0x03,
			   (video_pipe == VIDEO_PIPE_X ? VID_TX_EN_X : VID_TX_EN_Z) | 0x03);
}

static void max96789_dsi_bridge_atomic_disable(struct drm_bridge *bridge,
					       struct drm_bridge_state *old_bridge_state)
{
	struct max96789_dsi *max96789_dsi = container_of(bridge, struct max96789_dsi, bridge);
	struct max96789 *mfd = max96789_dsi->max96789_mfd;
	enum max96789_video_pipe_id video_pipe;

	if (mfd->gmsl2_dual_link) {
		regmap_write(mfd->regmap, MAX96789_DEV_PIPE_EN, 0x0);
		return;
	}

	video_pipe = max96789_dsi->id == DSI_PORT_A ? VIDEO_PIPE_X : VIDEO_PIPE_Z;

	regmap_update_bits(mfd->regmap, MAX96789_DEV_PIPE_EN,
			   (video_pipe == VIDEO_PIPE_X ? VID_TX_EN_X : VID_TX_EN_Z), 0);
}

static int max96789_dsi_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct max96789_dsi *max96789_dsi = container_of(bridge, struct max96789_dsi, bridge);
	struct device *dev = max96789_dsi->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi_dev;
	const struct mipi_dsi_device_info info = {.type = "max96789-dsi", };
	int ret;

	max96789_dsi->remote_bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 1);
	if (IS_ERR(max96789_dsi->remote_bridge)) {
		dev_err(dev, "Couldn't find remote bridge: %ld\n",
			PTR_ERR(max96789_dsi->remote_bridge));

		return PTR_ERR(max96789_dsi->remote_bridge);
	}

	ret = drm_bridge_attach(bridge->encoder, max96789_dsi->remote_bridge, bridge, flags);
	if (ret) {
		dev_err(dev, "Cannot attach remote bridge: %d\n", ret);
		return ret;
	}

	host = of_find_mipi_dsi_host_by_node(max96789_dsi->host_node);
	if (!host) {
		dev_err(dev, "Failed to find dsi host\n");
		return -ENODEV;
	}

	dsi_dev = devm_mipi_dsi_device_register_full(dev, host, &info);
	if (IS_ERR(dsi_dev)) {
		dev_err(dev, "Failed to create dsi device\n");
		return PTR_ERR(dsi_dev);
	}

	max96789_dsi->dsi = dsi_dev;

	dsi_dev->lanes = max96789_dsi->data_lanes;
	dsi_dev->format = MIPI_DSI_FMT_RGB888;
	dsi_dev->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			      MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE;

	ret = devm_mipi_dsi_attach(dev, dsi_dev);
	if (ret < 0)
		dev_err(dev, "Failed to attach dsi to host\n");

	return ret;
}

static void max96789_dsi_bridge_detach(struct drm_bridge *bridge)
{
	struct max96789_dsi *max96789_dsi = container_of(bridge, struct max96789_dsi, bridge);

	mipi_dsi_detach(max96789_dsi->dsi);
	mipi_dsi_device_unregister(max96789_dsi->dsi);
}

static void max96789_dsi_mode_set(struct drm_bridge *bridge, const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct max96789_dsi *max96789_dsi = container_of(bridge, struct max96789_dsi, bridge);
	struct max96789 *max96789 = max96789_dsi->max96789_mfd;
	struct videomode vm;
	u8 reg_val;

	drm_display_mode_to_videomode(adjusted_mode, &vm);

	/* HSYNC_WIDTH_L */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_CFG_HSYNC_WIDTH_L(max96789_dsi->id),
		     vm.hsync_len & 0xff);

	/* VSYNC_WIDTH_L */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_CFG_VSYNC_WIDTH_L(max96789_dsi->id),
		     vm.vsync_len & 0xff);

	/* HSYNC_WIDTH_H/VSYNC_WIDTH_H */
	reg_val = ((vm.hsync_len >> 8) << HSYNC_WIDTH_H_SHIFT) & HSYNC_WIDTH_H_MASK;
	reg_val |= ((vm.vsync_len >> 8) << VSYNC_WIDTH_H_SHIFT) & VSYNC_WIDTH_H_MASK;
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_CFG_SYNC(max96789_dsi->id), reg_val);

	/* VFP_L */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_VFP_L(max96789_dsi->id),
		     vm.vfront_porch & 0xff);

	/* VBP_H */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_VBP_H(max96789_dsi->id),
		     vm.vback_porch >> 4);

	/* VFP_H/VBP_L */
	reg_val = ((vm.vfront_porch >> 8) << VFP_H_SHIFT) & VFP_H_MASK;
	reg_val |= (vm.vback_porch << VBP_L_SHIFT) & VBP_L_MASK;
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_VP(max96789_dsi->id), reg_val);

	/* VRES_L */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_VACTIVE_L(max96789_dsi->id),
		     vm.vactive & 0xff);

	/* VRES_H */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_VACTIVE_H(max96789_dsi->id),
		     ((vm.vactive >> 8) << VACTIVE_H_SHIFT) & VACTIVE_H_MASK);

	/* HFP_L */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_HFP_L(max96789_dsi->id),
		     vm.hfront_porch & 0xff);

	/* HBP_H */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_HBP_H(max96789_dsi->id),
		     vm.hback_porch >> 4);

	/* HFP_H/HBP_L */
	reg_val = ((vm.hfront_porch >> 8) << HFP_H_SHIFT) & HFP_H_MASK;
	reg_val |= (vm.hback_porch << HBP_L_SHIFT) & HBP_L_MASK;
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_HP(max96789_dsi->id), reg_val);

	/* HRES_L */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_HACTIVE_L(max96789_dsi->id),
		     vm.hactive & 0xff);

	/* HRES_H */
	regmap_write(max96789->regmap, MAX96789_MIPI_DSI_HACTIVE_H(max96789_dsi->id),
		     ((vm.hactive >> 8) << HACTIVE_H_SHIFT) & HACTIVE_H_MASK);

	/* HSYNC/VSYNC polarity */
	reg_val = vm.flags & DISPLAY_FLAGS_HSYNC_HIGH ? HSYNC_POLARITY : 0;
	reg_val |= vm.flags & DISPLAY_FLAGS_VSYNC_HIGH ? VSYNC_POLARITY : 0;

	regmap_update_bits(max96789->regmap, MAX96789_MIPI_DSI_CFG(max96789_dsi->id),
			   HSYNC_POLARITY | VSYNC_POLARITY, reg_val);
}

static const struct drm_bridge_funcs max96789_dsi_bridge_funcs = {
	.atomic_pre_enable = max96789_dsi_bridge_atomic_pre_enable,
	.atomic_disable = max96789_dsi_bridge_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = max96789_dsi_bridge_attach,
	.detach = max96789_dsi_bridge_detach,
	.mode_set = max96789_dsi_mode_set,
};

static int max96789_dsi_dt_parse(struct max96789_dsi *max96789_dsi)
{
	struct device *dev = max96789_dsi->dev;
	struct device_node *mipi_in_endpoint;
	int data_lanes;
	int ret;

	ret = of_property_read_u32(dev->of_node, "reg", &max96789_dsi->id);
	if (ret < 0) {
		dev_err(dev, "could not get DSI port no\n");
		return ret;
	}

	if (max96789_dsi->id > 1) {
		dev_err(dev, "wrong DSI port no\n");
		return -EINVAL;
	}

	max96789_dsi->host_node = of_graph_get_remote_node(dev->of_node, 0, 0);
	if (!max96789_dsi->host_node)
		return -ENODEV;

	of_node_put(max96789_dsi->host_node);

	mipi_in_endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	data_lanes = drm_of_get_data_lanes_count(mipi_in_endpoint, 1, 4);
	if (data_lanes < 0) {
		dev_err(dev, "no data-lanes property provided for port %d ep 0\n",
			max96789_dsi->id);
		return data_lanes;
	}

	max96789_dsi->data_lanes = data_lanes;

	max96789_dsi->bridge.driver_private = max96789_dsi;
	max96789_dsi->bridge.funcs = &max96789_dsi_bridge_funcs;
	max96789_dsi->bridge.of_node = dev->of_node;

	drm_bridge_add(&max96789_dsi->bridge);

	return 0;
}

static int max96789_dsi_reset(struct max96789_dsi *max96789_dsi)
{
	struct max96789 *mfd = max96789_dsi->max96789_mfd;
	struct regmap *rmap = mfd->regmap;
	int ret = 0;

	/* reset MIPI receiver */
	ret |= regmap_update_bits(rmap, MAX96789_MIPI_RX_PHY_CFG, MIPI_RX_RESET, MIPI_RX_RESET);
	usleep_range(50, 100);
	ret |= regmap_update_bits(rmap, MAX96789_MIPI_RX_PHY_CFG, MIPI_RX_RESET, 0);

	/* stop all video pipes */
	ret = regmap_write(rmap, MAX96789_FRONTTOP_VIDEO_PIPE_START, 0);

	/* disable all video pipes */
	ret |= regmap_update_bits(rmap, MAX96789_DEV_PIPE_EN,
				  VID_TX_EN_X | VID_TX_EN_Y | VID_TX_EN_Z | VID_TX_EN_U, 0);

	/* disable DSI ports A and B and reset clocks */
	ret |= regmap_write(rmap, MAX96789_FRONTTOP_PORT_SEL, 0);

	if (ret)
		return -EIO;

	return 0;
}

static const struct of_device_id max96789_of_match[] = {
	{ .compatible = "maxim,max96789", .data = (void *)ID_MAX96789 },
	{}
};
MODULE_DEVICE_TABLE(of, max96789_of_match);

static int max96789_dsi_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max96789_dsi *max96789_dsi;
	struct max96789 *mfd = dev_get_drvdata(dev->parent);
	const struct of_device_id *match;
	int ret;

	match = of_match_node(max96789_of_match, dev->parent->of_node);
	if (!match) {
		dev_err(dev, "invalid compatible string\n");
		return -ENODEV;
	}

	if (!mfd->link_setup_finished)
		return -EPROBE_DEFER;

	max96789_dsi = devm_kzalloc(dev, sizeof(*max96789_dsi), GFP_KERNEL);
	if (!max96789_dsi)
		return -ENOMEM;

	max96789_dsi->dev = dev;
	max96789_dsi->max96789_mfd = mfd;

	dev_set_drvdata(dev, max96789_dsi);

	ret = max96789_dsi_reset(max96789_dsi);
	if (ret < 0) {
		dev_err(dev, "dsi registers reset failed\n");
		return ret;
	}

	return max96789_dsi_dt_parse(max96789_dsi);
}

static const struct of_device_id max96789_dsi_of_match[] = {
	{ .compatible = "maxim,max96789-dsi", .data = (void *)ID_MAX96789 },
	{}
};
MODULE_DEVICE_TABLE(of, max96789_dsi_of_match);

static struct platform_driver max96789_dsi_platform_driver = {
	.probe	= max96789_dsi_drm_probe,
	.driver	= {
		.name	= "max96789-dsi",
		.of_match_table = of_match_ptr(max96789_dsi_of_match),
	},
};

drm_module_platform_driver(max96789_dsi_platform_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_DESCRIPTION("MAX96789 serializer DSI bridge driver");
MODULE_LICENSE("GPL v2");
