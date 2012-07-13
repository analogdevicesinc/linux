#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#include "analog_drm_drv.h"

#include "../i2c/adv7511.h"

#define ANALOG_REG_CTRL			0x04
#define ANALOG_REG_HTIMING1		0x08
#define ANALOG_REG_HTIMING2		0x0C
/*
#define ANALOG_REG_VTIMING		0x0c
*/
#define ANALOG_REG_VTIMING1		0x10
#define ANALOG_REG_VTIMING2		0x14
#define ANALOG_REG_STATUS		0x10
#define ANALOG_REG_COLOR_PATTERN	0x1c

#define ANALOG_CTRL_ENABLE		BIT(0)
#define ANALOG_CTRL_CSC_BYPASS		BIT(1)
#define ANALOG_CTRL_TPG_ENABLE		BIT(2)

#define ANALOG_STATUS_VMDA_UNDERFLOW	BIT(4)
#define ANALOG_STATUS_VMDA_OVERFLOW	BIT(3)
#define ANALOG_STATUS_VMDA_BE_ERROR	BIT(2)
#define ANALOG_STATUS_VMDA_TPM_OOS	BIT(1)
#define ANALOG_STATUS_HDMI_TPM_OOS	BIT(0)

#define ANALOG_COLOR_PATTERN_ENABLE	BIT(24)

static struct debugfs_reg32 analog_drm_encoder_debugfs_regs[] = {
	{ "Control", ANALOG_REG_CTRL },
	{ "HTiming1", ANALOG_REG_HTIMING1 },
	{ "HTiming2", ANALOG_REG_HTIMING2 },
	{ "VTiming1", ANALOG_REG_VTIMING1 },
	{ "VTiming2", ANALOG_REG_VTIMING2 },
	{ "Status", ANALOG_REG_STATUS },
};

static uint16_t adv7511_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

/*
static struct adv7511_video_input_config adv7511_config_imageon = {
	.id = ADV7511_INPUT_ID_16_20_24BIT_YCbCr422_EMBEDDED_SYNC,
	.input_style = ADV7511_INPUT_STYLE1,
	.sync_pulse = ADV7511_INPUT_SYNC_PULSE_NONE,
	.clock_delay = ADV7511_INPUT_CLOCK_DELAY_NONE,
	.reverse_bitorder = 0,
	.bit_justification = 0,
	.vsync_polartity_low = false,
	.hsync_polartity_low = false,
	.up_conversion_first_order_interpolation = false,
	.input_color_depth = ADV7511_INPUT_COLOR_DEPTH_8BIT,
	.output_format = ADV7511_OUTPUT_FORMAT_RGB_444,
	.csc_enable = true,
	.csc_coefficents = adv7511_csc_ycbcr_to_rgb,
	.csc_scaling_factor = 3,
	.bit_justification = ADV7511_INPUT_BIT_JUSTIFICATION_LEFT,
};
*/

static struct adv7511_video_input_config adv7511_config_zc702 = {
	.id = ADV7511_INPUT_ID_16_20_24BIT_YCbCr422_SEPARATE_SYNC,
	.input_style = ADV7511_INPUT_STYLE1,
	.sync_pulse = ADV7511_INPUT_SYNC_PULSE_NONE,
	.clock_delay = ADV7511_INPUT_CLOCK_DELAY_NONE,
	.reverse_bitorder = 0,
	.vsync_polartity_low = false,
	.hsync_polartity_low = false,
	.up_conversion_first_order_interpolation = false,
	.input_color_depth = ADV7511_INPUT_COLOR_DEPTH_8BIT,
	.output_format = ADV7511_OUTPUT_FORMAT_RGB_444,
	.csc_enable = true,
	.csc_coefficents = adv7511_csc_ycbcr_to_rgb,
	.csc_scaling_factor = ADV7511_CSC_SCALING_4,
	.bit_justification = ADV7511_INPUT_BIT_JUSTIFICATION_RIGHT,
	.tmds_clock_inversion = true,
};

struct analog_drm_encoder {
	struct drm_encoder_slave encoder;
	struct drm_connector connector;

	struct debugfs_regset32 regset;
};


static inline struct analog_drm_encoder *to_analog_encoder(struct drm_encoder *enc)
{
	return container_of(enc, struct analog_drm_encoder, encoder.base);
}

static inline struct drm_encoder *connector_to_encoder(struct drm_connector *connector)
{
	struct analog_drm_encoder *enc = container_of(connector, struct analog_drm_encoder, connector);
	return &enc->encoder.base;
}

static int analog_drm_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder);

static inline struct drm_encoder_slave_funcs *
get_slave_funcs(struct drm_encoder *enc)
{
	if (!to_encoder_slave(enc))
		return NULL;

	return to_encoder_slave(enc)->slave_funcs;
}

static int debugfs_cp_get(void *data, u64 *val)
{
	struct analog_drm_private *private = data;
	*val = ioread32(private->base + ANALOG_REG_COLOR_PATTERN);
	return 0;
}

static int debugfs_cp_set(void *data, u64 val)
{
	struct analog_drm_private *private = data;
	iowrite32(0x0000000, private->base + ANALOG_REG_COLOR_PATTERN);
	iowrite32(0x1000000 | val, private->base + ANALOG_REG_COLOR_PATTERN);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_cp, debugfs_cp_get, debugfs_cp_set, "0x%08llx\n");

static void analog_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct analog_drm_encoder *analog_encoder = to_analog_encoder(encoder);
	struct drm_connector *connector = &analog_encoder->connector;
	struct analog_drm_private *private = encoder->dev->dev_private;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct adv7511_video_input_config config = adv7511_config_zc702;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		iowrite32(ANALOG_CTRL_ENABLE, private->base + ANALOG_REG_CTRL);
		if (connector->display_info.raw_edid) {
			struct edid *edid = (struct edid *)connector->display_info.raw_edid;
			config.hdmi_mode = drm_detect_hdmi_monitor(edid);
		} else {
			config.hdmi_mode = false;
		}

		printk("raw_edid: %p %d\n", connector->display_info.raw_edid,
			connector->display_info.color_formats);

		if ((connector->display_info.color_formats & DRM_COLOR_FORMAT_YCRCB422) &&
			config.hdmi_mode) {
			config.csc_enable = false;
			config.output_format = ADV7511_OUTPUT_FORMAT_YCBCR_422;
			pr_info("Using YCbCr output\n");
		} else {
			config.csc_enable = true;
			config.output_format = ADV7511_OUTPUT_FORMAT_RGB_444;
			pr_info("Using RGB output\n");
		}

		sfuncs->set_config(encoder, &config);
		break;
	default:
		iowrite32(0, private->base + ANALOG_REG_CTRL);
		break;
	}

	if (sfuncs && sfuncs->dpms)
		sfuncs->dpms(encoder, mode);
}

struct analog_drm_crtc_clock_setting {
	long clock;
	uint32_t settings[10];
};

static const struct analog_drm_crtc_clock_setting clock_settings[] = {
	{ /* 162 Mhz */
		.clock = 162000,
		.settings = {
			0x00c4, 0x0080, 0x2042, 0x0209, 0x0080,
			0x023f, 0x7c01, 0x7fe9, 0x0100, 0x1090,
		},
	},
	{ /* 154 Mhz */
		.clock = 154000,
		.settings = {
			0x0082, 0x0000, 0x2187, 0x0514, 0x0000,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x1090,
		},
	},
	{ /* 148.5 Mhz */
		.clock = 148500,
		.settings = {
			0x00c3, 0x0000, 0x2146, 0x0619, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 146.25 Mhz */
		.clock = 146250,
		.settings = {
			0x00c3, 0x0000, 0x2187, 0x071d, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 138.5 Mhz */
		.clock = 138500,
		.settings = {
			0x0083, 0x0080, 0x2187, 0x0597, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x1090,
		},
	},
	{ /* 135 Mhz */
		.clock = 135000,
		.settings = {
			0x0104, 0x0000, 0x2083, 0x034e, 0x0080,
			0x015e, 0x7c01, 0x7fe9, 0x0100, 0x8090,
		},
	},
	{ /* 108 Mhz */
		.clock = 108000,
		.settings = {
			0x0145, 0x0000, 0x2083, 0x034e, 0x0080,
			0x015e, 0x7c01, 0x7fe9, 0x0100, 0x8090,
		},
	},
	{ /* 106.5 Mhz */
		.clock = 106500,
		.settings = {
			0x00c4, 0x0080, 0x2146, 0x0515, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x1090,
		},
	},
	{ /* 78.8 Mhz */
		.clock = 78800,
		.settings = {
			0x0146, 0x0080, 0x2042, 0x0187, 0x0080,
			0x02ee, 0x7c01, 0x7fe9, 0x0800, 0x9090,
		},
	},
	{ /* 75 Mhz */
		.clock = 75000,
		.settings = {
			0x0104, 0x0000, 0x1041, 0x0042, 0x0080,
			0x03e8, 0x2001, 0x23e9, 0x0100, 0x9890,
		},
	},
	{ /* 74.25 Mhz */
		.clock = 74250,
		.settings = {
			0x0186, 0x0000, 0x2146, 0x0619, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 65 Mhz */
		.clock = 65000,
		.settings = {
			0x0145, 0x0000, 0x0082, 0x0187, 0x0080,
			0x02ee, 0x7c01, 0x7fe9, 0x0800, 0x9090,
		},
	},
	{ /* 57.284 Mhz */
		.clock = 57284,
		.settings = {
			0x0208, 0x0000, 0x0186, 0x06dc, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 50 Mhz */
		.clock = 50000,
		.settings = {
			0x0186, 0x0000, 0x1041, 0x0042, 0x0080,
			0x03e8, 0x2001, 0x23e9, 0x0100, 0x9890,
		},
	},
	{ /* 49.5 Mhz */
		.clock = 49500,
		.settings = {
			0x02cb, 0x0000, 0x2105, 0x0619, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 40 Mhz */
		.clock = 40000,
		.settings = {
			0x01c8, 0x0080, 0x1041, 0x0042, 0x0080,
			0x03e8, 0x2001, 0x23e9, 0x0100, 0x9890,
		},
	},
	{ /* 36 Mhz */
		.clock = 36000,
		.settings = {
			0x030d, 0x0080, 0x0041, 0x0105, 0x0080,
			0x03e8, 0x6401, 0x67e9, 0x0100, 0x9090,
		},
	},
	{ /* 31.5 Mhz */
		.clock = 31500,
		.settings = {
			0x030d, 0x0080, 0x0208, 0x07e0, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 30.24 Mhz */
		.clock = 30240,
		.settings = {
			0x038f, 0x0080, 0x2187, 0x071d, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 28.32 Mhz */
		.clock = 28320,
		.settings = {
			0x030d, 0x0080, 0x2187, 0x05d7, 0x0000,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x1090,
		},

	},
	{ /* 27 Mhz */
		.clock = 27000,
		.settings = {
			0x0514, 0x0000, 0x2083, 0x034e, 0x0080,
			0x015e, 0x7c01, 0x7fe9, 0x0100, 0x8090,
		},
	},
	{ /* 25.2 MHz */
		.clock = 25200,
		.settings = {
			0x030d, 0x0080, 0x028a, 0x07e0, 0x0080,
			0x00fa, 0x7c01, 0x7fe9, 0x0800, 0x8090,
		},
	},
	{ /* 25.175 Mhz */
		.clock = 25175,
		.settings = {
			0x034d, 0x0000, 0x2146, 0x0492, 0x0000,
			0x0113, 0x7c01, 0x7fe9, 0x0100, 0x8090,
		},
	},
	{ /* 13.5 MHz */
		.clock = 13500,
		.settings = {
			0x0a28, 0x0000, 0x2083, 0x034e, 0x0080,
			0x015e, 0x7c01, 0x7fe9, 0x0100, 0x8090,
		},
	},
};

static const struct analog_drm_crtc_clock_setting *analog_drm_encoder_closest_clock(long clock)
{
	const struct analog_drm_crtc_clock_setting *best_setting = NULL;
	long best_diff, diff;
	unsigned  int i;

	best_diff = LONG_MAX;

	for (i = 0; i < ARRAY_SIZE(clock_settings); ++i) {
		diff = abs(clock_settings[i].clock - clock);
		if (diff < best_diff) {
			best_diff = diff;
			best_setting = &clock_settings[i];
		}
	}

	return best_setting;
}

static bool analog_drm_encoder_mode_fixup(struct drm_encoder *encoder,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	if (sfuncs && sfuncs->mode_fixup)
		return sfuncs->mode_fixup(encoder, mode, adjusted_mode);

	return true;
}

static void analog_drm_write_clock_reg(struct analog_drm_private *private,
	unsigned int reg, unsigned int val)
{
	iowrite32(val, private->base_clock + reg);
}

#define ANALOG_CLOCK_REG_UPDATE_ENABLE	0x04
#define ANALOG_CLOCK_REG_CONFIG(x)	(0x08 + (0x04 * x))

static int analog_drm_encoder_set_clock(struct analog_drm_private *private, long clock)
{
	const struct analog_drm_crtc_clock_setting *best_setting;
	unsigned int i;

	best_setting = analog_drm_encoder_closest_clock(clock);

	if (!best_setting)
		return -EINVAL;

	printk("setting clock to: %lu\n", best_setting->clock);

	analog_drm_write_clock_reg(private, ANALOG_CLOCK_REG_UPDATE_ENABLE, 0);

	for (i = 0; i < 10; ++i) {
		analog_drm_write_clock_reg(private, ANALOG_CLOCK_REG_CONFIG(i),
			best_setting->settings[i]);
	}

	analog_drm_write_clock_reg(private, ANALOG_CLOCK_REG_UPDATE_ENABLE, 1);

	return 0;
}

static void analog_drm_encoder_mode_set(struct drm_encoder *encoder,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct analog_drm_private *private = encoder->dev->dev_private;
	unsigned int h_de_min, h_de_max;
	unsigned int v_de_min, v_de_max;
	unsigned int htotal, hactive;
	unsigned int vtotal, vactive;

	if (sfuncs && sfuncs->mode_set)
		sfuncs->mode_set(encoder, mode, adjusted_mode);

	htotal = mode->htotal;
	hactive = mode->hdisplay;
	vtotal = mode->vtotal;
	vactive = mode->vdisplay;

	hactive = mode->hsync_end - mode->hsync_start;
	vactive = mode->vsync_end - mode->vsync_start;

	h_de_min =  htotal - mode->hsync_start;
	h_de_max =  h_de_min + mode->hdisplay;
	v_de_min =  vtotal - mode->vsync_start;
	v_de_max =  v_de_min + mode->vdisplay;

	iowrite32((hactive << 16) | htotal, private->base + ANALOG_REG_HTIMING1);
	iowrite32((h_de_min << 16) | h_de_max, private->base + ANALOG_REG_HTIMING2);
	iowrite32((vactive << 16) | vtotal, private->base + ANALOG_REG_VTIMING1);
	iowrite32((v_de_min << 16) | v_de_max, private->base + ANALOG_REG_VTIMING2);

	analog_drm_encoder_set_clock(private, mode->clock);
}

static void analog_drm_encoder_commit(struct drm_encoder *encoder)
{
	analog_drm_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void analog_drm_encoder_prepare(struct drm_encoder *encoder)
{
	analog_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static struct drm_crtc *analog_drm_encoder_get_crtc(struct drm_encoder *encoder)
{
	return encoder->crtc;
}

static struct drm_encoder_helper_funcs analog_encoder_helper_funcs = {
	.dpms		= analog_drm_encoder_dpms,
	.mode_fixup	= analog_drm_encoder_mode_fixup,
	.mode_set	= analog_drm_encoder_mode_set,
	.prepare	= analog_drm_encoder_prepare,
	.commit		= analog_drm_encoder_commit,
	.get_crtc	= analog_drm_encoder_get_crtc,
};

static void analog_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct analog_drm_encoder *analog_encoder =
		to_analog_encoder(encoder);

	if (sfuncs && sfuncs->destroy)
		sfuncs->destroy(encoder);

	drm_encoder_cleanup(encoder);
	encoder->dev->mode_config.num_encoder--;
	kfree(analog_encoder);
}

static struct drm_encoder_funcs analog_encoder_funcs = {
	.destroy = analog_drm_encoder_destroy,
};

static struct i2c_board_info fmc_adv7511_encoder_info[] = {
	{
		I2C_BOARD_INFO("adv7511", 0x39),
	},
	{ }
};

struct drm_encoder *analog_drm_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct analog_drm_encoder *analog_encoder;
	struct analog_drm_private *priv = dev->dev_private;

	analog_encoder = kzalloc(sizeof(*analog_encoder), GFP_KERNEL);
	if (!analog_encoder) {
		DRM_ERROR("failed to allocate encoder\n");
		return NULL;
	}

	encoder = &analog_encoder->encoder.base;
	encoder->possible_crtcs = 1;

	drm_encoder_init(dev, encoder, &analog_encoder_funcs,
			DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(encoder, &analog_encoder_helper_funcs);

	drm_i2c_encoder_init(dev, to_encoder_slave(encoder),
		priv->slave_adapter,
		fmc_adv7511_encoder_info);

	connector = &analog_encoder->connector;

	analog_drm_connector_init(dev, connector, encoder);

	analog_encoder->regset.base = priv->base;
	analog_encoder->regset.regs = analog_drm_encoder_debugfs_regs;
	analog_encoder->regset.nregs = ARRAY_SIZE(analog_drm_encoder_debugfs_regs);
	debugfs_create_regset32(dev_name(dev->dev), S_IRUGO, NULL, &analog_encoder->regset);
	debugfs_create_file("color_pattern", 0666, NULL, priv, &fops_cp);

	return encoder;
}

static struct i2c_adapter *analog_drm_get_ddc_adapter(struct drm_device *dev)
{
	struct analog_drm_private *priv = dev->dev_private;

	return priv->ddc_adapter;
}

static int analog_drm_connector_get_modes(struct drm_connector *connector)
{
	struct drm_encoder *encoder = connector_to_encoder(connector);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct i2c_adapter *adapter = analog_drm_get_ddc_adapter(connector->dev);
	struct edid *edid;
	int count = 0;

	kfree(connector->display_info.raw_edid);
	connector->display_info.raw_edid = NULL;

	if (adapter) {
		edid = drm_get_edid(connector, adapter);
		drm_mode_connector_update_edid_property(connector,
							edid);
		count += drm_add_edid_modes(connector, edid);
	} else {
		if (sfuncs && sfuncs->get_modes)
			count += sfuncs->get_modes(encoder, connector);
	}

	return count;
}

static int analog_drm_connector_mode_valid(struct drm_connector *connector,
	struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static struct drm_encoder *analog_drm_best_encoder(struct drm_connector *connector)
{
	return connector_to_encoder(connector);
}

static struct drm_connector_helper_funcs analog_connector_helper_funcs = {
	.get_modes	= analog_drm_connector_get_modes,
	.mode_valid	= analog_drm_connector_mode_valid,
	.best_encoder	= analog_drm_best_encoder,
};

static enum drm_connector_status
analog_drm_connector_detect(struct drm_connector *connector, bool force)
{
	enum drm_connector_status status = connector_status_unknown;
	struct drm_encoder *encoder = connector_to_encoder(connector);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct i2c_adapter *adapter = analog_drm_get_ddc_adapter(connector->dev);
	struct edid *edid;

	if (adapter) {
		kfree(connector->display_info.raw_edid);
		connector->display_info.raw_edid = NULL;

		edid = drm_get_edid(connector, adapter);
		if (edid)
			status = connector_status_connected;
		else
			status = connector_status_disconnected;
	}

	if (sfuncs && sfuncs->detect)
		status = sfuncs->detect(encoder, connector);

	return status;
}

static void analog_drm_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs analog_connector_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.detect		= analog_drm_connector_detect,
	.destroy	= analog_drm_connector_destroy,
};

static int analog_drm_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder)
{
	int type;
	int err;

	type = DRM_MODE_CONNECTOR_HDMIA;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
						DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_init(dev, connector, &analog_connector_funcs, type);
	drm_connector_helper_add(connector, &analog_connector_helper_funcs);

	err = drm_sysfs_connector_add(connector);
	if (err)
		goto err_connector;

	connector->encoder = encoder;

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		DRM_ERROR("failed to attach a connector to a encoder\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	drm_sysfs_connector_remove(connector);
err_connector:
	drm_connector_cleanup(connector);
	return err;
}
