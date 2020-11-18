/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/clk.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_edid.h>

#include "axi_hdmi_drv.h"

#define AXI_HDMI_STATUS_VMDA_UNDERFLOW	BIT(4)
#define AXI_HDMI_STATUS_VMDA_OVERFLOW	BIT(3)
#define AXI_HDMI_STATUS_VMDA_BE_ERROR	BIT(2)
#define AXI_HDMI_STATUS_VMDA_TPM_OOS	BIT(1)
#define AXI_HDMI_STATUS_HDMI_TPM_OOS	BIT(0)

#define AXI_HDMI_COLOR_PATTERN_ENABLE	BIT(24)

#define AXI_HDMI_REG_RESET		0x040
#define AXI_HDMI_REG_CTRL		0x044
#define AXI_HDMI_REG_SOURCE_SEL		0x048
#define AXI_HDMI_REG_COLORPATTERN	0x04c
#define AXI_HDMI_REG_STATUS		0x05c
#define AXI_HDMI_REG_VDMA_STATUS	0x060
#define AXI_HDMI_REG_TPM_STATUS		0x064
#define AXI_HDMI_REG_CLIPP_MAX  	0x068
#define AXI_HDMI_REG_CLIPP_MIN  	0x06c
#define AXI_HDMI_REG_HTIMING1		0x400
#define AXI_HDMI_REG_HTIMING2		0x404
#define AXI_HDMI_REG_HTIMING3		0x408
#define AXI_HDMI_REG_VTIMING1		0x440
#define AXI_HDMI_REG_VTIMING2		0x444
#define AXI_HDMI_REG_VTIMING3		0x448

#define AXI_HDMI_RESET_ENABLE		BIT(0)

#define AXI_HDMI_CTRL_SS_BYPASS		BIT(2)
#define AXI_HDMI_CTRL_FULL_RANGE	BIT(1)
#define AXI_HDMI_CTRL_CSC_BYPASS	BIT(0)

#define AXI_HDMI_SOURCE_SEL_COLORPATTERN	0x3
#define AXI_HDMI_SOURCE_SEL_TESTPATTERN		0x2
#define AXI_HDMI_SOURCE_SEL_NORMAL		0x1
#define AXI_HDMI_SOURCE_SEL_NONE		0x0

static const struct debugfs_reg32 axi_hdmi_encoder_debugfs_regs[] = {
	{ "Reset", AXI_HDMI_REG_RESET },
	{ "Control", AXI_HDMI_REG_CTRL },
	{ "Source select", AXI_HDMI_REG_SOURCE_SEL },
	{ "Colorpattern", AXI_HDMI_REG_COLORPATTERN },
	{ "Status", AXI_HDMI_REG_STATUS },
	{ "VDMA status", AXI_HDMI_REG_VDMA_STATUS },
	{ "TPM status", AXI_HDMI_REG_TPM_STATUS },
	{ "HTiming1", AXI_HDMI_REG_HTIMING1 },
	{ "HTiming2", AXI_HDMI_REG_HTIMING2 },
	{ "HTiming3", AXI_HDMI_REG_HTIMING3 },
	{ "VTiming1", AXI_HDMI_REG_VTIMING1 },
	{ "VTiming2", AXI_HDMI_REG_VTIMING2 },
	{ "VTiming3", AXI_HDMI_REG_VTIMING3 },
};

static const uint16_t adv7511_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

struct axi_hdmi_encoder {
	struct drm_encoder_slave encoder;
	struct drm_connector connector;

#ifdef CONFIG_DEBUG_FS
	struct debugfs_regset32 regset;
#endif
};

static inline struct axi_hdmi_encoder *to_axi_hdmi_encoder(struct drm_encoder *enc)
{
	return container_of(enc, struct axi_hdmi_encoder, encoder.base);
}

static inline struct drm_encoder *connector_to_encoder(struct drm_connector *connector)
{
	struct axi_hdmi_encoder *enc = container_of(connector, struct axi_hdmi_encoder, connector);
	return &enc->encoder.base;
}

static void axi_hdmi_set_color_range(struct axi_hdmi_private *private,
	unsigned int low, unsigned int high)
{
	writel(high, private->base + AXI_HDMI_REG_CLIPP_MAX);
	writel(low, private->base + AXI_HDMI_REG_CLIPP_MIN);
}

#ifdef CONFIG_DEBUG_FS

static int axi_hdmi_debugfs_cp_get(void *data, u64 *val)
{
	struct axi_hdmi_private *private = data;
	*val = readl(private->base + AXI_HDMI_REG_COLORPATTERN);
	return 0;
}

static int axi_hdmi_debugfs_cp_set(void *data, u64 val)
{
	struct axi_hdmi_private *private = data;

	writel(val, private->base + AXI_HDMI_REG_COLORPATTERN);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(axi_hdmi_cp_fops, axi_hdmi_debugfs_cp_get,
	axi_hdmi_debugfs_cp_set, "0x%06llx\n");

static const char * const axi_hdmi_mode_text[] = {
	[AXI_HDMI_SOURCE_SEL_NONE] = "none",
	[AXI_HDMI_SOURCE_SEL_NORMAL] = "normal",
	[AXI_HDMI_SOURCE_SEL_TESTPATTERN] = "testpattern",
	[AXI_HDMI_SOURCE_SEL_COLORPATTERN] = "colorpattern",
};

static ssize_t axi_hdmi_read_mode(struct file *file, char __user *userbuf,
	size_t count, loff_t *ppos)
{
	struct axi_hdmi_private *private = file->private_data;
	uint32_t src;
	const char *fmt;
	size_t len = 0;
	char buf[50];
	int i;

	src = readl(private->base + AXI_HDMI_REG_SOURCE_SEL);

	for (i = 0; i < ARRAY_SIZE(axi_hdmi_mode_text); i++) {
		if (src == i)
			fmt = "[%s] ";
		else
			fmt = "%s ";
		len += scnprintf(buf + len, sizeof(buf) - len, fmt,
				axi_hdmi_mode_text[i]);
	}

	buf[len - 1] = '\n';

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t axi_hdmi_set_mode(struct file *file, const char __user *userbuf,
	size_t count, loff_t *ppos)
{
	struct axi_hdmi_private *private = file->private_data;
	char buf[20];
	unsigned int ctrl;
	unsigned int i;

	count = min_t(size_t, count, sizeof(buf) - 1);
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';

	for (i = 0; i < ARRAY_SIZE(axi_hdmi_mode_text); i++) {
		if (sysfs_streq(axi_hdmi_mode_text[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(axi_hdmi_mode_text))
		return -EINVAL;

	writel(i, private->base + AXI_HDMI_REG_SOURCE_SEL);

	if (i == AXI_HDMI_SOURCE_SEL_TESTPATTERN) {
		axi_hdmi_set_color_range(private, 0, 0xffffff);
		ctrl = AXI_HDMI_CTRL_CSC_BYPASS | AXI_HDMI_CTRL_SS_BYPASS |
			AXI_HDMI_CTRL_FULL_RANGE;
	} else {
		if (private->is_rgb) {
			axi_hdmi_set_color_range(private, 0, 0xffffff);
			ctrl = AXI_HDMI_CTRL_CSC_BYPASS;
		} else {
			axi_hdmi_set_color_range(private, 0x101010, 0xf0ebf0);
			ctrl = 0;
		}
	}

	writel(ctrl, private->base + AXI_HDMI_REG_CTRL);

	return count;
}

static const struct file_operations axi_hdmi_mode_fops = {
	.open = simple_open,
	.read = axi_hdmi_read_mode,
	.write = axi_hdmi_set_mode,
};

static void axi_hdmi_debugfs_init(struct axi_hdmi_encoder *encoder)
{
	struct axi_hdmi_private *priv = encoder->encoder.base.dev->dev_private;

	encoder->regset.base = priv->base;
	encoder->regset.regs = axi_hdmi_encoder_debugfs_regs;
	encoder->regset.nregs = ARRAY_SIZE(axi_hdmi_encoder_debugfs_regs);

	debugfs_create_regset32(dev_name(encoder->encoder.base.dev->dev), S_IRUGO, NULL, &encoder->regset);
	debugfs_create_file("color_pattern", 0600, NULL, priv, &axi_hdmi_cp_fops);
	debugfs_create_file("mode", 0600, NULL, priv, &axi_hdmi_mode_fops);
}

#else

static inline void axi_hdmi_debugfs_init(struct axi_hdmi_encoder *enc)
{
}

#endif

static void axi_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct axi_hdmi_private *private = encoder->dev->dev_private;

	if (!private->clk_enabled) {
		clk_prepare_enable(private->hdmi_clock);
		private->clk_enabled = true;
	}
	writel(AXI_HDMI_RESET_ENABLE, private->base + AXI_HDMI_REG_RESET);
}

static void axi_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct axi_hdmi_private *private = encoder->dev->dev_private;

	writel(0, private->base + AXI_HDMI_REG_RESET);
	if (private->clk_enabled) {
		clk_disable_unprepare(private->hdmi_clock);
		private->clk_enabled = false;
	}
}

static void axi_hdmi_encoder_mode_set(struct drm_encoder *encoder,
	struct drm_crtc_state *crtc_state,
	struct drm_connector_state *conn_state)
{
	struct axi_hdmi_private *private = encoder->dev->dev_private;
	struct drm_display_mode *mode = &crtc_state->mode;
	unsigned int h_de_min, h_de_max;
	unsigned int v_de_min, v_de_max;
	unsigned int val;

	h_de_min = mode->htotal - mode->hsync_start;
	h_de_max = h_de_min + mode->hdisplay;
	v_de_min = mode->vtotal - mode->vsync_start;
	v_de_max = v_de_min + mode->vdisplay;

	val = (mode->hdisplay << 16) | mode->htotal;
	writel(val,  private->base + AXI_HDMI_REG_HTIMING1);
	val = mode->hsync_end - mode->hsync_start;
	writel(val,  private->base + AXI_HDMI_REG_HTIMING2);
	val = (h_de_max << 16) | h_de_min;
	writel(val,  private->base + AXI_HDMI_REG_HTIMING3);

	val = (mode->vdisplay << 16) | mode->vtotal;
	writel(val,  private->base + AXI_HDMI_REG_VTIMING1);
	val = mode->vsync_end - mode->vsync_start;
	writel(val,  private->base + AXI_HDMI_REG_VTIMING2);
	val = (v_de_max << 16) | v_de_min;
	writel(val,  private->base + AXI_HDMI_REG_VTIMING3);

	clk_set_rate(private->hdmi_clock, mode->clock * 1000);
}

static const struct drm_encoder_helper_funcs axi_hdmi_encoder_helper_funcs = {
	.enable = axi_hdmi_encoder_enable,
	.disable = axi_hdmi_encoder_disable,
	.atomic_mode_set = axi_hdmi_encoder_mode_set,
};

static void axi_hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	struct axi_hdmi_encoder *axi_hdmi_encoder =
		to_axi_hdmi_encoder(encoder);

	drm_encoder_cleanup(encoder);
	kfree(axi_hdmi_encoder);
}

static const struct drm_encoder_funcs axi_hdmi_encoder_funcs = {
	.destroy = axi_hdmi_encoder_destroy,
};

struct drm_encoder *axi_hdmi_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	struct axi_hdmi_encoder *axi_hdmi_encoder;
	struct axi_hdmi_private *priv = dev->dev_private;
	struct drm_bridge *bridge;
	int ret;

	axi_hdmi_encoder = kzalloc(sizeof(*axi_hdmi_encoder), GFP_KERNEL);
	if (!axi_hdmi_encoder)
		return NULL;

	encoder = &axi_hdmi_encoder->encoder.base;
	encoder->possible_crtcs = 1;

	drm_encoder_init(dev, encoder, &axi_hdmi_encoder_funcs,
			DRM_MODE_ENCODER_TMDS, NULL);
	drm_encoder_helper_add(encoder, &axi_hdmi_encoder_helper_funcs);

	bridge = of_drm_find_bridge(priv->encoder_slave->dev.of_node);
	if (bridge) {
		bridge->encoder = encoder;
		encoder->bridge = bridge;
		ret = drm_bridge_attach(encoder, bridge, NULL);
		if (ret) {
		    drm_encoder_cleanup(encoder);
		    return NULL;
		}
	}


	axi_hdmi_debugfs_init(axi_hdmi_encoder);

	writel(AXI_HDMI_SOURCE_SEL_NORMAL, priv->base + AXI_HDMI_REG_SOURCE_SEL);
	if (priv->is_rgb) {
		writel(AXI_HDMI_CTRL_CSC_BYPASS, priv->base + AXI_HDMI_REG_CTRL);
		axi_hdmi_set_color_range(priv, 0, 0xffffff);
	} else {
		axi_hdmi_set_color_range(priv, 0x101010, 0xf0ebf0);
	}

	return encoder;
}
