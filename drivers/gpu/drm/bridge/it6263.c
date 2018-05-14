/*
 * Copyright 2017-2018 NXP
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

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#define REG_VENDOR_ID(n)	(0x00 + (n))	/* n: 0/1 */
#define REG_DEVICE_ID(n)	(0x02 + (n))	/* n: 0/1 */
#define LVDS_VENDER_ID_LOW	0x15
#define LVDS_VENDER_ID_HIGH	0xCA
#define LVDS_DEVICE_ID_LOW	0x61
#define LVDS_DEVICE_ID_HIGH	0x62
#define HDMI_VENDER_ID_LOW	0x01
#define HDMI_VENDER_ID_HIGH	0xCA
#define HDMI_DEVICE_ID_LOW	0x13
#define HDMI_DEVICE_ID_HIGH	0x76

/* LVDS registers */
#define LVDS_REG_SW_RST		0x05
#define SOFT_REFCLK_DM_RST	BIT(0)
#define SOFT_PCLK_DM_RST	BIT(1)

#define LVDS_REG_MODE		0x2C
#define LVDS_COLOR_DEPTH	0x3
enum {
	LVDS_COLOR_DEPTH_18,
	LVDS_COLOR_DEPTH_24,
	LVDS_COLOR_DEPTH_30,
	LVDS_COLOR_DEPTH_36,
};
#define LVDS_OUT_MAP		BIT(4)
#define VESA			BIT(4)
#define JEIDA			0
#define DMODE			BIT(7)
#define SPLIT_MODE		BIT(7)
#define SINGLE_MODE		0

#define LVDS_REG_STABLE		0x30
#define VIDEO_STABLE		BIT(0)
#define PCLK_LOCK		BIT(1)

#define LVDS_REG_39		0x39

#define LVDS_REG_PLL		0x3C
#define LVDS_REG_AFE_3E		0x3E
#define LVDS_REG_AFE_3F		0x3F
#define LVDS_REG_AFE_47		0x47
#define LVDS_REG_AFE_48		0x48
#define LVDS_REG_AFE_4F		0x4F
#define LVDS_REG_52		0x52
#define LVDS_REG_PCLK_CNT_HIGH	0x57
#define LVDS_REG_PCLK_CNT_LOW	0x58

/*
 * HDMI registers
 *
 * Registers are separated into three banks:
 * 1) common bank: 0x00 ~ 0x2F
 * 2) bank0: 0x30  ~ 0xFF
 * 3) bank1: 0x130 ~ 0x1FF	(HDMI packet registers)
 *
 * Use register HDMI_REG_BANK_CTRL @ 0x0F[1:0] to select bank0/1:
 * 2b'00 - bank0
 * 2b'01 - bank1
 */

/******************************/
/* HDMI register common bank  */
/******************************/

/* HDMI genernal registers */
#define HDMI_REG_SW_RST		0x04
#define SOFTREF_RST		BIT(5)
#define SOFTA_RST		BIT(4)
#define SOFTV_RST		BIT(3)
#define AUD_RST			BIT(2)
#define HDCP_RST		BIT(0)
#define HDMI_RST_ALL		(SOFTREF_RST | SOFTA_RST | SOFTV_RST | \
				 AUD_RST | HDCP_RST)

#define HDMI_REG_INT_CTRL	0x05
#define INTPOL_ACTH		BIT(7)
#define INTPOL_ACTL		0
#define INTIOMODE_OPENDRAIN	BIT(6)
#define INTIOMODE_PUSHPULL	0
#define SELXTAL			BIT(5)	/* REFCLK <= XTALCLK */
#define SELXTAL_QUARTER		0	/* REFCLK <= OSCCLK/4 */
#define PDREFCNT(n)		(((n) >> 2) << 2)	/* REFCLK Div(n) */
#define PDREFCLK		BIT(1)
#define PDTXCLK_GATED		BIT(0)
#define PDTXCLK_ACTIVE		0

#define HDMI_REG_INT_STAT(n)	(0x05 + (n))	/* n: 1/2/3 */
#define HDMI_REG_INT_MASK(n)	(0x08 + (n))	/* n: 1/2/3 */

/* INT1 */
#define INT_AUD_OVERFLOW	BIT(7)
#define INT_RDDC_NOACK		BIT(5)
#define INT_DDCFIFO_ERR		BIT(4)
#define INT_DDC_BUS_HANG	BIT(2)
#define INT_RX_SENSE		BIT(1)
#define INT_HPD			BIT(0)

/* INT2 */
#define INT_VID_UNSTABLE	BIT(6)
#define INT_PKTACP		BIT(5)
#define INT_PKTNULL		BIT(4)
#define INT_PKTGEN		BIT(3)
#define INT_KSVLIST_CHK		BIT(2)
#define INT_AUTH_DONE		BIT(1)
#define INT_AUTH_FAIL		BIT(0)

/* INT3 */
#define INT_AUD_CTS		BIT(6)
#define INT_VSYNC		BIT(5)
#define INT_VIDSTABLE		BIT(4)
#define INT_PKTMPG		BIT(3)
#define INT_PKTGBD		BIT(2)
#define INT_PKTAUD		BIT(1)
#define INT_PKTAVI		BIT(0)

#define INT_MASK_AUD_CTS	BIT(5)
#define INT_MASK_VSYNC		BIT(4)
#define INT_MASK_VIDSTABLE	BIT(3)
#define INT_MASK_PKTMPG		BIT(2)
#define INT_MASK_PKTGBD		BIT(1)
#define INT_MASK_PKTAUD		BIT(0)

#define HDMI_REG_INT_CLR(n)	(0x0C + (n))	/* n: 0/1 */

/* CLR0 */
#define INT_CLR_PKTACP		BIT(7)
#define INT_CLR_PKTNULL		BIT(6)
#define INT_CLR_PKTGEN		BIT(5)
#define INT_CLR_KSVLIST_CHK	BIT(4)
#define INT_CLR_AUTH_DONE	BIT(3)
#define INT_CLR_AUTH_FAIL	BIT(2)
#define INT_CLR_RXSENSE		BIT(1)
#define INT_CLR_HPD		BIT(0)

/* CLR1 */
#define INT_CLR_VSYNC		BIT(7)
#define INT_CLR_VIDSTABLE	BIT(6)
#define INT_CLR_PKTMPG		BIT(5)
#define INT_CLR_PKTGBD		BIT(4)
#define INT_CLR_PKTAUD		BIT(3)
#define INT_CLR_PKTAVI		BIT(2)
#define INT_CLR_VID_UNSTABLE	BIT(0)

#define HDMI_REG_SYS_STATUS	0x0E
#define INT_ACTIVE		BIT(7)
#define HPDETECT		BIT(6)
#define RXSENDETECT		BIT(5)
#define TXVIDSTABLE		BIT(4)
#define CTSINTSTEP		0xC
#define CLR_AUD_CTS		BIT(1)
#define INTACTDONE		BIT(0)

#define HDMI_REG_BANK_CTRL	0x0F
#define BANK_SEL(n)		((n) ? 1 : 0)

/* HDMI System DDC control registers */
#define HDMI_REG_DDC_MASTER_CTRL	0x10
#define MASTER_SEL_HOST			BIT(0)
#define MASTER_SEL_HDCP			0

#define HDMI_REG_DDC_HEADER		0x11
#define DDC_HDCP_ADDRESS		0x74

#define HDMI_REG_DDC_REQOFF		0x12
#define HDMI_REG_DDC_REQCOUNT		0x13
#define HDMI_REG_DDC_EDIDSEG		0x14

#define HDMI_REG_DDC_CMD		0x15
#define DDC_CMD_SEQ_BURSTREAD		0x0
#define DDC_CMD_LINK_CHKREAD		0x2
#define DDC_CMD_EDID_READ		0x3
#define DDC_CMD_FIFO_CLR		0x9
#define DDC_CMD_GEN_SCLCLK		0xA
#define DDC_CMD_ABORT			0xF

#define HDMI_REG_DDC_STATUS		0x16
#define DDC_DONE			BIT(7)
#define DDC_ACT				BIT(6)
#define DDC_NOACK			BIT(5)
#define DDC_WAITBUS			BIT(4)
#define DDC_ARBILOSE			BIT(3)
#define DDC_ERROR			(DDC_NOACK | DDC_WAITBUS | DDC_ARBILOSE)
#define DDC_FIFOFULL			BIT(2)
#define DDC_FIFOEMPTY			BIT(1)

#define HDMI_DDC_FIFO_SIZE		32	/* bytes */
#define HDMI_REG_DDC_READFIFO		0x17
#define HDMI_REG_ROM_STAT		0x1C
#define HDMI_REG_LVDS_PORT		0x1D	/* LVDS input ctrl i2c addr */
#define HDMI_REG_LVDS_PORT_EN		0x1E	/* and to enable */
#define LVDS_INPUT_CTRL_I2C_ADDR	0x33

/***********************/
/* HDMI register bank0 */
/***********************/

/* HDMI clock control registers */
#define HDMI_REG_CLK_CTRL1		0x59
#define EN_TXCLK_COUNT			BIT(5)
#define VDO_LATCH_EDGE			BIT(3)

/* HDMI AFE registers */
#define HDMI_REG_AFE_DRV_CTRL		0x61
#define AFE_DRV_PWD			BIT(5)
#define AFE_DRV_RST			BIT(4)
#define AFE_DRV_PDRXDET			BIT(2)
#define AFE_DRV_TERMON			BIT(1)
#define AFE_DRV_ENCAL			BIT(0)

#define HDMI_REG_AFE_XP_CTRL		0x62
#define AFE_XP_GAINBIT			BIT(7)
#define AFE_XP_PWDPLL			BIT(6)
#define AFE_XP_ENI			BIT(5)
#define AFE_XP_ER0			BIT(4)
#define AFE_XP_RESETB			BIT(3)
#define AFE_XP_PWDI			BIT(2)
#define AFE_XP_DEI			BIT(1)
#define AFE_XP_DER			BIT(0)

#define HDMI_REG_AFE_ISW_CTRL		0x63
#define AFE_RTERM_SEL			BIT(7)
#define AFE_IP_BYPASS			BIT(6)
#define AFE_DRV_ISW			0x38
#define AFE_DRV_ISWK			7

#define HDMI_REG_AFE_IP_CTRL		0x64
#define AFE_IP_GAINBIT			BIT(7)
#define AFE_IP_PWDPLL			BIT(6)
#define AFE_IP_CKSEL			0x30
#define AFE_IP_ER0			BIT(3)
#define AFE_IP_RESETB			BIT(2)
#define AFE_IP_ENC			BIT(1)
#define AFE_IP_EC1			BIT(0)

/* HDMI input data format registers */
#define HDMI_REG_INPUT_MODE		0x70
#define IN_RGB				0x00
#define IN_YUV422			0x40
#define IN_YUV444			0x80

#define HDMI_REG_TXFIFO_RST		0x71
#define ENAVMUTERST			BIT(0)
#define TXFFRST				BIT(1)

/* HDMI pattern generation SYNC/DE registers */
#define HDMI_REG_9X(n)			(0x90 + (n))	/* n: 0x0 ~ 0xF */
#define HDMI_REG_AX(n)			(0xA0 + (n))	/* n: 0x0 ~ 0xF */
#define HDMI_REG_B0			0xB0

/* HDMI general control registers */
#define HDMI_REG_HDMI_MODE		0xC0
#define TX_HDMI_MODE			1
#define TX_DVI_MODE			0

#define HDMI_REG_GCP			0xC1
#define AVMUTE				BIT(0)
#define BLUE_SCR_MUTE			BIT(1)
#define NODEF_PHASE			BIT(2)
#define PHASE_RESYNC			BIT(3)
#define HDMI_COLOR_DEPTH		0x70
enum {
	HDMI_COLOR_DEPTH_DEF = 0x0,	/* default as 24bit */
	HDMI_COLOR_DEPTH_24  = 0x40,
	HDMI_COLOR_DEPTH_30  = 0x50,
	HDMI_COLOR_DEPTH_36  = 0x60,
	HDMI_COLOR_DEPTH_48  = 0x70,
};

#define HDMI_REG_OESS_CYCLE		0xC3
#define HDMI_REG_ENCRYPTION		0xC4	/* HDCP */

#define HDMI_REG_PKT_SINGLE_CTRL	0xC5
#define SINGLE_PKT			BIT(0)
#define BURST_PKT			0

#define HDMI_REG_PKT_GENERAL_CTRL	0xC6
#define HDMI_REG_NULL_CTRL		0xC9
#define HDMI_REG_ACP_CTRL		0xCA
#define HDMI_REG_ISRC1_CTRL		0xCB
#define HDMI_REG_ISRC2_CTRL		0xCC
#define HDMI_REG_AVI_INFOFRM_CTRL	0xCD
#define HDMI_REG_AUD_INFOFRM_CTRL	0xCE
#define HDMI_REG_SPD_INFOFRM_CTRL	0xCF
#define HDMI_REG_MPG_INFOFRM_CTRL	0xD0
#define ENABLE_PKT			BIT(0)
#define REPEAT_PKT			BIT(1)

/***********************/
/* HDMI register bank1 */
/***********************/

/* AVI packet registers */
#define HDMI_REG_AVI_DB1		0x58
#define AVI_DB1_COLOR_SPACE		0x60
enum {
	AVI_COLOR_SPACE_RGB    = 0x00,
	AVI_COLOR_SPACE_YUV422 = 0x20,
	AVI_COLOR_SPACE_YUV444 = 0x40,
};

struct it6263 {
	struct i2c_client *hdmi_i2c;
	struct i2c_client *lvds_i2c;
	struct regmap *hdmi_regmap;
	struct regmap *lvds_regmap;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct gpio_desc *reset_gpio;
	bool is_hdmi;
	bool split_mode;
};

static inline struct it6263 *bridge_to_it6263(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6263, bridge);
}

static inline struct it6263 *connector_to_it6263(struct drm_connector *con)
{
	return container_of(con, struct it6263, connector);
}

static inline void lvds_update_bits(struct it6263 *it6263, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(it6263->lvds_regmap, reg, mask, val);
}

static inline void hdmi_update_bits(struct it6263 *it6263, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(it6263->hdmi_regmap, reg, mask, val);
}

static void it6263_reset(struct it6263 *it6263)
{
	if (!it6263->reset_gpio)
		return;

	gpiod_set_value_cansleep(it6263->reset_gpio, 0);

	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(it6263->reset_gpio, 1);

	/*
	 * The chip maker says the low pulse should be at least 40ms,
	 * so 41ms is sure to be enough.
	 */
	usleep_range(41000, 45000);

	gpiod_set_value_cansleep(it6263->reset_gpio, 0);

	/* somehow, addtional time to wait the high voltage to be stable */
	usleep_range(5000, 6000);
}

static enum drm_connector_status
it6263_connector_detect(struct drm_connector *connector, bool force)
{
	struct it6263 *it6263 = connector_to_it6263(connector);
	unsigned int status;
	int i;

	/*
	 * FIXME: We read status tens of times to workaround
	 * cable detection failure issue at boot time on some
	 * platforms.
	 * Spin on this for up to one second.
	 */
	for (i = 0; i < 100; i++) {
		regmap_read(it6263->hdmi_regmap, HDMI_REG_SYS_STATUS, &status);
		if (status & HPDETECT)
			return connector_status_connected;
		usleep_range(5000, 10000);
	}

	return connector_status_disconnected;
}

static const struct drm_connector_funcs it6263_connector_funcs = {
	.detect = it6263_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int
it6263_read_edid(void *data, u8 *buf, unsigned int block, size_t len)
{
	struct it6263 *it6263 = data;
	struct regmap *regmap = it6263->hdmi_regmap;
	unsigned long timeout;
	unsigned int status, count, val;
	unsigned int segment = block >> 1;
	unsigned int start = (block % 2) * EDID_LENGTH;

	regmap_write(regmap, HDMI_REG_DDC_MASTER_CTRL, MASTER_SEL_HOST);
	regmap_write(regmap, HDMI_REG_DDC_HEADER, DDC_ADDR << 1);
	regmap_write(regmap, HDMI_REG_DDC_EDIDSEG, segment);

	while (len) {
		/* clear DDC FIFO */
		regmap_write(regmap, HDMI_REG_DDC_CMD, DDC_CMD_FIFO_CLR);

		timeout = jiffies + msecs_to_jiffies(10);
		do {
			regmap_read(regmap, HDMI_REG_DDC_STATUS, &status);
		} while (!(status & DDC_DONE) && time_before(jiffies, timeout));

		if (!(status & DDC_DONE)) {
			dev_err(&it6263->hdmi_i2c->dev,
						"failed to clear DDC FIFO\n");
			return -ETIMEDOUT;
		}

		count = len > HDMI_DDC_FIFO_SIZE ? HDMI_DDC_FIFO_SIZE : len;

		/* fire the read command */
		regmap_write(regmap, HDMI_REG_DDC_REQOFF, start);
		regmap_write(regmap, HDMI_REG_DDC_REQCOUNT, count);
		regmap_write(regmap, HDMI_REG_DDC_CMD, DDC_CMD_EDID_READ);

		start += count;
		len -= count;

		/* wait for reading done */
		timeout = jiffies + msecs_to_jiffies(250);
		do {
			regmap_read(regmap, HDMI_REG_DDC_STATUS, &status);
			if (status & DDC_ERROR) {
				dev_err(&it6263->hdmi_i2c->dev, "DDC error\n");
				return -EIO;
			}
		} while (!(status & DDC_DONE) && time_before(jiffies, timeout));

		if (!(status & DDC_DONE)) {
			dev_err(&it6263->hdmi_i2c->dev,
						"failed to read EDID\n");
			return -ETIMEDOUT;
		}

		/* cache to buffer */
		for (; count > 0; count--) {
			regmap_read(regmap, HDMI_REG_DDC_READFIFO, &val);
			*(buf++) = val;
		}
	}

	return 0;
}

static int it6263_get_modes(struct drm_connector *connector)
{
	struct it6263 *it6263 = connector_to_it6263(connector);
	struct regmap *regmap = it6263->hdmi_regmap;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	struct edid *edid;
	int num = 0;
	int ret;

	regmap_write(regmap, HDMI_REG_DDC_MASTER_CTRL, MASTER_SEL_HOST);

	edid = drm_do_get_edid(connector, it6263_read_edid, it6263);
	drm_mode_connector_update_edid_property(connector, edid);
	if (edid) {
		num = drm_add_edid_modes(connector, edid);
		it6263->is_hdmi = drm_detect_hdmi_monitor(edid);
		kfree(edid);
	}

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	return num;
}

enum drm_mode_status it6263_mode_valid(struct drm_connector *connector,
					struct drm_display_mode *mode)
{
	if (mode->clock > 150000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static const struct drm_connector_helper_funcs it6263_connector_helper_funcs = {
	.get_modes = it6263_get_modes,
	.mode_valid = it6263_mode_valid,
};

static void it6263_bridge_disable(struct drm_bridge *bridge)
{
	struct it6263 *it6263 = bridge_to_it6263(bridge);
	struct regmap *regmap = it6263->hdmi_regmap;

	/* AV mute */
	hdmi_update_bits(it6263, HDMI_REG_GCP, AVMUTE, AVMUTE);

	if (it6263->is_hdmi)
		regmap_write(regmap, HDMI_REG_PKT_GENERAL_CTRL, 0);

	hdmi_update_bits(it6263, HDMI_REG_SW_RST, SOFTV_RST, SOFTV_RST);
	regmap_write(regmap, HDMI_REG_AFE_DRV_CTRL, AFE_DRV_RST | AFE_DRV_PWD);
}

static void it6263_bridge_enable(struct drm_bridge *bridge)
{
	struct it6263 *it6263 = bridge_to_it6263(bridge);
	struct regmap *regmap = it6263->hdmi_regmap;
	unsigned long timeout;
	unsigned int status;

	regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(1));
	/* set the color space to RGB in the AVI packet */
	hdmi_update_bits(it6263, HDMI_REG_AVI_DB1, AVI_DB1_COLOR_SPACE,
							AVI_COLOR_SPACE_RGB);
	regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(0));

	/* software video reset */
	hdmi_update_bits(it6263, HDMI_REG_SW_RST, SOFTV_RST, SOFTV_RST);
	usleep_range(1000, 2000);
	hdmi_update_bits(it6263, HDMI_REG_SW_RST, SOFTV_RST, 0);

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		regmap_read(regmap, HDMI_REG_SYS_STATUS, &status);
	} while (!(status & TXVIDSTABLE) && time_before(jiffies, timeout));

	if (!(status & TXVIDSTABLE))
		dev_warn(&it6263->hdmi_i2c->dev,
				"failed to wait for video stable\n");

	regmap_write(regmap, HDMI_REG_AFE_DRV_CTRL, 0);

	/* AV unmute */
	hdmi_update_bits(it6263, HDMI_REG_GCP, AVMUTE, 0);

	if (it6263->is_hdmi)
		regmap_write(regmap, HDMI_REG_PKT_GENERAL_CTRL,
						ENABLE_PKT | REPEAT_PKT);
}

static void it6263_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adj)
{
	struct it6263 *it6263 = bridge_to_it6263(bridge);
	struct regmap *regmap = it6263->hdmi_regmap;
	bool pclk_high = adj->clock > 80000 ? true : false;

	regmap_write(regmap, HDMI_REG_HDMI_MODE,
				it6263->is_hdmi ? TX_HDMI_MODE : TX_DVI_MODE);

	dev_dbg(&it6263->hdmi_i2c->dev, "%s mode\n",
				it6263->is_hdmi ? "HDMI" : "DVI");

	/* setup AFE */
	regmap_write(regmap, HDMI_REG_AFE_DRV_CTRL, AFE_DRV_RST);
	if (pclk_high)
		regmap_write(regmap, HDMI_REG_AFE_XP_CTRL,
						AFE_XP_GAINBIT | AFE_XP_RESETB);
	else
		regmap_write(regmap, HDMI_REG_AFE_XP_CTRL,
						AFE_XP_ER0 | AFE_XP_RESETB);
	regmap_write(regmap, HDMI_REG_AFE_ISW_CTRL, 0x10);
	if (pclk_high)
		regmap_write(regmap, HDMI_REG_AFE_IP_CTRL,
						AFE_IP_GAINBIT | AFE_IP_RESETB);
	else
		regmap_write(regmap, HDMI_REG_AFE_IP_CTRL,
						AFE_IP_ER0 | AFE_IP_RESETB);
}

static int it6263_bridge_attach(struct drm_bridge *bridge)
{
	struct it6263 *it6263 = bridge_to_it6263(bridge);
	struct drm_device *drm = bridge->dev;
	int ret;

	if (!drm_core_check_feature(drm, DRIVER_ATOMIC)) {
		dev_err(&it6263->hdmi_i2c->dev,
			"it6263 driver only copes with atomic updates\n");
		return -ENOTSUPP;
	}

	it6263->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
				   DRM_CONNECTOR_POLL_DISCONNECT;
	ret = drm_connector_init(drm, &it6263->connector,
				 &it6263_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(&it6263->hdmi_i2c->dev,
				"Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&it6263->connector,
				 &it6263_connector_helper_funcs);
	drm_mode_connector_attach_encoder(&it6263->connector, bridge->encoder);

	return ret;
}

static const struct drm_bridge_funcs it6263_bridge_funcs = {
	.attach = it6263_bridge_attach,
	.mode_set = it6263_bridge_mode_set,
	.disable = it6263_bridge_disable,
	.enable = it6263_bridge_enable,
};

static int it6263_check_chipid(struct it6263 *it6263)
{
	struct device *dev = &it6263->hdmi_i2c->dev;
	u8 vendor_id[2], device_id[2];
	int ret;

	ret = regmap_bulk_read(it6263->hdmi_regmap, REG_VENDOR_ID(0),
				&vendor_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (vendor_id[0] != HDMI_VENDER_ID_LOW ||
	    vendor_id[1] != HDMI_VENDER_ID_HIGH) {
		dev_err(dev,
			"Invalid hdmi vendor id %02x %02x(expect 0x01 0xca)\n",
			vendor_id[0], vendor_id[1]);
		return -EINVAL;
	}

	ret = regmap_bulk_read(it6263->hdmi_regmap, REG_DEVICE_ID(0),
				&device_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (device_id[0] != HDMI_DEVICE_ID_LOW ||
	    device_id[1] != HDMI_DEVICE_ID_HIGH) {
		dev_err(dev,
			"Invalid hdmi device id %02x %02x(expect 0x13 0x76)\n",
			device_id[0], device_id[1]);
		return -EINVAL;
	}

	ret = regmap_bulk_read(it6263->lvds_regmap, REG_VENDOR_ID(0),
				&vendor_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (vendor_id[0] != LVDS_VENDER_ID_LOW ||
	    vendor_id[1] != LVDS_VENDER_ID_HIGH) {
		dev_err(dev,
			"Invalid lvds vendor id %02x %02x(expect 0x15 0xca)\n",
			vendor_id[0], vendor_id[1]);
		return -EINVAL;
	}

	ret = regmap_bulk_read(it6263->lvds_regmap, REG_DEVICE_ID(0),
				&device_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (device_id[0] != LVDS_DEVICE_ID_LOW ||
	    device_id[1] != LVDS_DEVICE_ID_HIGH) {
		dev_err(dev,
			"Invalid lvds device id %02x %02x(expect 0x61 0x62)\n",
			device_id[0], device_id[1]);
		return -EINVAL;
	}

	return ret;
}

static void it6263_lvds_reset(struct it6263 *it6263)
{
	/* AFE PLL reset */
	lvds_update_bits(it6263, LVDS_REG_PLL, 0x1, 0x0);
	usleep_range(1000, 2000);
	lvds_update_bits(it6263, LVDS_REG_PLL, 0x1, 0x1);

	/* pclk reset */
	lvds_update_bits(it6263, LVDS_REG_SW_RST,
				SOFT_PCLK_DM_RST, SOFT_PCLK_DM_RST);
	usleep_range(1000, 2000);
	lvds_update_bits(it6263, LVDS_REG_SW_RST, SOFT_PCLK_DM_RST, 0x0);

	usleep_range(1000, 2000);
}

static void it6263_lvds_set_interface(struct it6263 *it6263)
{
	/* color depth */
	lvds_update_bits(it6263, LVDS_REG_MODE, LVDS_COLOR_DEPTH,
						LVDS_COLOR_DEPTH_24);

	/* jeida mapping */
	lvds_update_bits(it6263, LVDS_REG_MODE, LVDS_OUT_MAP, JEIDA);

	if (it6263->split_mode) {
		lvds_update_bits(it6263, LVDS_REG_MODE, DMODE, SPLIT_MODE);
		lvds_update_bits(it6263, LVDS_REG_52, BIT(1), BIT(1));
	} else {
		lvds_update_bits(it6263, LVDS_REG_MODE, DMODE, SINGLE_MODE);
		lvds_update_bits(it6263, LVDS_REG_52, BIT(1), 0);
	}
}

static void it6263_lvds_set_afe(struct it6263 *it6263)
{
	struct regmap *regmap = it6263->lvds_regmap;

	regmap_write(regmap, LVDS_REG_AFE_3E, 0xaa);
	regmap_write(regmap, LVDS_REG_AFE_3F, 0x02);
	regmap_write(regmap, LVDS_REG_AFE_47, 0xaa);
	regmap_write(regmap, LVDS_REG_AFE_48, 0x02);
	regmap_write(regmap, LVDS_REG_AFE_4F, 0x11);

	lvds_update_bits(it6263, LVDS_REG_PLL, 0x07, 0);
}

static void it6263_hdmi_config(struct it6263 *it6263)
{
	regmap_write(it6263->hdmi_regmap, HDMI_REG_INPUT_MODE, IN_RGB);

	hdmi_update_bits(it6263, HDMI_REG_GCP, HDMI_COLOR_DEPTH,
						HDMI_COLOR_DEPTH_24);
}

static const struct regmap_range it6263_hdmi_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0x1ff },
};

static const struct regmap_access_table it6263_hdmi_volatile_table = {
	.yes_ranges = it6263_hdmi_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(it6263_hdmi_volatile_ranges),
};

static const struct regmap_config it6263_hdmi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &it6263_hdmi_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_range it6263_lvds_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xff },
};

static const struct regmap_access_table it6263_lvds_volatile_table = {
	.yes_ranges = it6263_lvds_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(it6263_lvds_volatile_ranges),
};

static const struct regmap_config it6263_lvds_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &it6263_lvds_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static const struct i2c_board_info it6263_lvds_i2c = {
	I2C_BOARD_INFO("it6263_LVDS_i2c", LVDS_INPUT_CTRL_I2C_ADDR),
};

static int it6263_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
#if IS_ENABLED(CONFIG_OF_DYNAMIC)
	struct device_node *remote_node = NULL, *endpoint = NULL;
	struct of_changeset ocs;
	struct property *prop;
#endif
	struct it6263 *it6263;
	int ret;

	it6263 = devm_kzalloc(dev, sizeof(*it6263), GFP_KERNEL);
	if (!it6263)
		return -ENOMEM;

	it6263->split_mode = of_property_read_bool(np, "split-mode");

	it6263->hdmi_i2c = client;
	it6263->lvds_i2c = i2c_new_device(client->adapter, &it6263_lvds_i2c);
	if (!it6263->lvds_i2c) {
		ret = -ENODEV;
		goto of_reconfig;
	}

	it6263->hdmi_regmap = devm_regmap_init_i2c(client,
						&it6263_hdmi_regmap_config);
	if (IS_ERR(it6263->hdmi_regmap)) {
		ret = PTR_ERR(it6263->hdmi_regmap);
		goto unregister_lvds_i2c;
	}

	it6263->lvds_regmap = devm_regmap_init_i2c(it6263->lvds_i2c,
						&it6263_lvds_regmap_config);
	if (IS_ERR(it6263->lvds_regmap)) {
		ret = PTR_ERR(it6263->lvds_regmap);
		goto unregister_lvds_i2c;
	}

	it6263->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							GPIOD_OUT_LOW);
	if (IS_ERR(it6263->reset_gpio)) {
		ret = PTR_ERR(it6263->reset_gpio);

		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get reset gpio: %d\n", ret);

		goto unregister_lvds_i2c;
	}

	it6263_reset(it6263);

	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_SW_RST, HDMI_RST_ALL);
	if (ret)
		goto unregister_lvds_i2c;

	usleep_range(1000, 2000);

	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_LVDS_PORT,
				LVDS_INPUT_CTRL_I2C_ADDR << 1);
	if (ret)
		goto unregister_lvds_i2c;

	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_LVDS_PORT_EN, 0x01);
	if (ret)
		goto unregister_lvds_i2c;

	/* select HDMI bank0 */
	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL,
				BANK_SEL(0));
	if (ret)
		goto unregister_lvds_i2c;

	ret = it6263_check_chipid(it6263);
	if (ret)
		goto unregister_lvds_i2c;

	it6263_lvds_reset(it6263);
	it6263_lvds_set_interface(it6263);
	it6263_lvds_set_afe(it6263);
	it6263_hdmi_config(it6263);

	it6263->bridge.funcs = &it6263_bridge_funcs;
	it6263->bridge.of_node = np;
	ret = drm_bridge_add(&it6263->bridge);
	if (ret) {
		dev_err(dev, "Failed to add drm_bridge\n");
		return ret;
	}

	i2c_set_clientdata(client, it6263);

	return ret;

unregister_lvds_i2c:
	i2c_unregister_device(it6263->lvds_i2c);
	if (ret == -EPROBE_DEFER)
		return ret;

of_reconfig:
#if IS_ENABLED(CONFIG_OF_DYNAMIC)
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (endpoint)
		remote_node = of_graph_get_remote_port_parent(endpoint);

	if (remote_node) {
		int num_endpoints = 0;

		/*
		 * Remote node should have two endpoints (input and output: us)
		 * If remote node has more than two endpoints, probably that it
		 * has more outputs, so there is no need to disable it.
		 */
		endpoint = NULL;
		while ((endpoint = of_graph_get_next_endpoint(remote_node,
							      endpoint)))
			num_endpoints++;

		if (num_endpoints > 2) {
			of_node_put(remote_node);
			return ret;
		}

		prop = devm_kzalloc(dev, sizeof(*prop), GFP_KERNEL);
		prop->name = devm_kstrdup(dev, "status", GFP_KERNEL);
		prop->value = devm_kstrdup(dev, "disabled", GFP_KERNEL);
		prop->length = 9;
		of_changeset_init(&ocs);
		of_changeset_update_property(&ocs, remote_node, prop);
		ret = of_changeset_apply(&ocs);
		if (!ret)
			dev_warn(dev,
				"Probe failed. Remote port '%s' disabled\n",
				remote_node->full_name);

		of_node_put(remote_node);
	};
#endif

	return ret;
}

static int it6263_remove(struct i2c_client *client)

{
	struct it6263 *it6263 = i2c_get_clientdata(client);

	drm_bridge_remove(&it6263->bridge);
	i2c_unregister_device(it6263->lvds_i2c);

	return 0;
}

static const struct of_device_id it6263_dt_ids[] = {
	{ .compatible = "ite,it6263", },
	{ }
};
MODULE_DEVICE_TABLE(of, it6263_dt_ids);

static const struct i2c_device_id it6263_i2c_ids[] = {
	{ "it6263", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, it6263_i2c_ids);

static struct i2c_driver it6263_driver = {
	.probe = it6263_probe,
	.remove = it6263_remove,
	.driver = {
		.name = "it6263",
		.of_match_table = it6263_dt_ids,
	},
	.id_table = it6263_i2c_ids,
};
module_i2c_driver(it6263_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("ITE Tech. Inc. IT6263 LVDS->HDMI bridge");
MODULE_LICENSE("GPL");
